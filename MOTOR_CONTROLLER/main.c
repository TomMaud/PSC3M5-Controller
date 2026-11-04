/******************************************************************************
 * File Name:   main.c
 *
 * Description:
 * Sensorless 3-shunt Field-Oriented Control (FOC) for a PMSM using
 * Infineon's PSoC Control C3 MCU and MCDI solution personality.
 *
 * PC control over UART:
 * - Motor drive is enabled only while the PC monitor app is connected.
 * - A heartbeat monitors the PC connection.
 * - An LED indicates the connection state.
 * - Motor telemetry is streamed while connected.
 * - Speed commands are received using R<rpm>\n.
 *
 *******************************************************************************
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation. All rights reserved.
 ******************************************************************************/

/*******************************************************************************
 * Header Files
 ******************************************************************************/

#include "HardwareIface.h"
#include "Params.h"
#include "cybsp.h"
#include "Controller.h"
#include "MotorCtrlHWConfig.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"
#include "cy_syslib.h"
#include "cy_systick.h"
#include "mtb_hal.h"
#include "cycfg_pins.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "ParamConfig.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/

/* PC link settings */

#define HEARTBEAT_BYTE              ((uint8_t)'H')
#define LINK_TIMEOUT_MS             (1000U)
#define TELEMETRY_PERIOD_MS         (100U)

/* Speed command settings */

#define PI_F                        (3.14159265f)
#define MAX_REQUEST_RPM             (6000U)
#define MIN_REQUEST_RPM             (300U)

/* Telemetry conversion */

#define MOTOR_POLE_PAIRS            (4.0f)
#define RAD_S_TO_RPM                (9.5492966f)

/* Direction reversal settings */

#define REVERSE_STOP_THRESHOLD_RAD_S    (2.0f)
#define REVERSE_STOP_TIMEOUT_MS         (5000U)

/* Link LED */

#define LINK_LED_PORT                CYBSP_USER_LED2_PORT
#define LINK_LED_PIN                 CYBSP_USER_LED2_NUM
#define LINK_LED_ON_LEVEL            (1U)

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t DEBUG_UART_hal_obj;

static volatile uint32_t sys_millis = 0U;

static uint32_t last_heartbeat_ms = 0U;
static bool heartbeat_seen = false;

/* Motor direction */

static bool reversed = false;
static bool direction_change_pending = false;

static int32_t pending_rpm = 0;
static uint32_t direction_stop_start_ms = 0U;

/* UART RPM command parser */

static bool rpm_command_active = false;
static bool rpm_command_invalid = false;

static uint32_t rpm_command_value = 0U;

static bool rpm_command_negative = false;
static bool rpm_command_sign_allowed = true;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

static void systick_handler(void);
static void debug_uart_init(void);

static void link_poll(void);
static void process_uart_byte(uint8_t rx);

static bool link_is_connected(uint32_t now);
static void link_led_set(bool on);

static void motor_drive_set(bool enable);
static void motor_set_speed_rpm(int32_t rpm);

static void handle_direction_change(void);

static void print_telemetry(void);

/*******************************************************************************
 * Function Name: systick_handler
 ******************************************************************************/

static void systick_handler(void)
{
    sys_millis++;
}

/*******************************************************************************
 * Function Name: debug_uart_init
 ******************************************************************************/

static void debug_uart_init(void)
{
    cy_rslt_t result;

    result = (cy_rslt_t)Cy_SCB_UART_Init(
        DEBUG_UART_HW,
        &DEBUG_UART_config,
        &DEBUG_UART_context
    );

    CY_ASSERT(result == CY_RSLT_SUCCESS);

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    result = mtb_hal_uart_setup(
        &DEBUG_UART_hal_obj,
        &DEBUG_UART_hal_config,
        &DEBUG_UART_context,
        NULL
    );

    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    CY_ASSERT(result == CY_RSLT_SUCCESS);

    (void)result;
}

/*******************************************************************************
 * Function Name: motor_drive_set
 *
 * Summary:
 * Enables or disables the motor drive.
 ******************************************************************************/

static void motor_drive_set(bool enable)
{
    motor[0].vars_ptr->en = enable;

#if defined(APP_KIT_PSC3M5_2GO)
    Cy_GPIO_Write(EN_IPM_PORT, EN_IPM_NUM, enable);
#endif
}

/*******************************************************************************
 * Function Name: motor_set_speed_rpm
 *
 * Summary:
 * Converts an RPM request into the normalised external speed command.
 *
 * The command is normalised against the configured maximum mechanical
 * speed in rad/s.
 *
 * A zero RPM command retains the current direction.
 ******************************************************************************/

static void motor_set_speed_rpm(int32_t rpm)
{
    float max_speed_rad_s;
    float requested_speed_rad_s;
    float command;

    max_speed_rad_s =
        motor[0].params_ptr->sys.cmd.w_max.mech;

    if (max_speed_rad_s <= 0.0f)
    {
        motor[0].vars_ptr->cmd_ext = 0.0f;
        return;
    }

    /* Determine direction.
       Retain the existing direction when rpm == 0. */

    if (rpm < 0)
    {
        motor[0].vars_ptr->dir = -1.0f;
        rpm = -rpm;
    }
    else if (rpm > 0)
    {
        motor[0].vars_ptr->dir = 1.0f;
    }

    /* Limit requested speed */

    if ((uint32_t)rpm > MAX_REQUEST_RPM)
    {
        rpm = (int32_t)MAX_REQUEST_RPM;
    }

    /* Convert RPM to mechanical rad/s */

    requested_speed_rad_s =
        (float)rpm * (2.0f * PI_F / 60.0f);

    /* Normalise against configured maximum speed */

    command = requested_speed_rad_s / max_speed_rad_s;

    if (command > 1.0f)
    {
        command = 1.0f;
    }

    if (command < 0.0f)
    {
        command = 0.0f;
    }

    motor[0].vars_ptr->cmd_ext = command;

    printf(
        "[Command] Requested: %ld RPM | Direction: %s | Normalised: %.3f\r\n",
        (long)(motor[0].vars_ptr->dir < 0.0f ? -rpm : rpm),
        motor[0].vars_ptr->dir < 0.0f ? "Reverse" : "Forward",
        (double)command
    );

    fflush(stdout);
}

/*******************************************************************************
 * Function Name: handle_direction_change
 *
 * Summary:
 * Handles motor direction reversal without immediately switching direction.
 *
 * The motor is first commanded to zero speed while the inverter remains
 * enabled. The direction is changed once measured speed is sufficiently
 * close to zero.
 ******************************************************************************/

static void handle_direction_change(void)
{
    float speed;
    uint32_t now = sys_millis;

    if (!direction_change_pending)
    {
        return;
    }

    /* Cancel reversal if the PC disconnects */

    if (!link_is_connected(now))
    {
        direction_change_pending = false;
        pending_rpm = 0;

        motor_set_speed_rpm(0);
        motor_drive_set(false);

        printf(
            "[Motor] Direction change cancelled - PC disconnected\r\n"
        );

        return;
    }

    /* Read estimated electrical speed */

    speed = motor[0].vars_ptr->w_final_filt.elec;

    /* Wait until the motor is sufficiently close to zero speed */

    if ((speed < REVERSE_STOP_THRESHOLD_RAD_S) &&
        (speed > -REVERSE_STOP_THRESHOLD_RAD_S))
    {
        int32_t new_rpm = pending_rpm;

        /* Apply the new direction and speed */

        motor_set_speed_rpm(new_rpm);

        reversed = (new_rpm < 0);

        direction_change_pending = false;
        pending_rpm = 0;

        motor_drive_set(true);

        printf(
            "[Motor] Direction changed to %s\r\n",
            reversed ? "Reverse" : "Forward"
        );
    }

    /* Safety timeout */

    else if ((now - direction_stop_start_ms) >=
             REVERSE_STOP_TIMEOUT_MS)
    {
        direction_change_pending = false;
        pending_rpm = 0;

        motor_set_speed_rpm(0);
        motor_drive_set(false);

        printf(
            "[Motor] Direction change timed out - motor disabled\r\n"
        );
    }
}

/*******************************************************************************
 * Function Name: process_uart_byte
 *
 * Summary:
 * Processes individual bytes received over UART.
 *
 * Supported commands:
 *
 *   H       - Heartbeat
 *   R500\n  - Set motor speed to 500 RPM
 *   R1000\n - Set motor speed to 1000 RPM
 *   R-1000\n - Set motor speed to -1000 RPM
 *   R0\n    - Stop the motor
 *
 ******************************************************************************/

static void process_uart_byte(uint8_t rx)
{
    /* Heartbeat is always recognised */

    if (rx == HEARTBEAT_BYTE)
    {
        last_heartbeat_ms = sys_millis;
        heartbeat_seen = true;
        return;
    }

    /* Start of a new RPM command */

    if (rx == (uint8_t)'R')
    {
        rpm_command_active = true;
        rpm_command_invalid = false;
        rpm_command_value = 0U;

        rpm_command_negative = false;
        rpm_command_sign_allowed = true;

        return;
    }

    /* Ignore bytes when not receiving an RPM command */

    if (!rpm_command_active)
    {
        return;
    }

    /* End of command */

    if ((rx == '\n') || (rx == '\r'))
    {
        if (!rpm_command_invalid)
        {
            int32_t requested_rpm = (int32_t)rpm_command_value;

            if (rpm_command_negative)
            {
                requested_rpm = -requested_rpm;
            }

            bool requested_reverse = (requested_rpm < 0);

            /* Stop requests within the deadband */

            if (rpm_command_value <= MIN_REQUEST_RPM)
            {
                direction_change_pending = false;
                pending_rpm = 0;

                motor_set_speed_rpm(0);
                motor_drive_set(false);

                printf("[Motor] Stopped\r\n");
            }

            /* A direction reversal is required */

            else if (requested_reverse != reversed)
            {
                pending_rpm = requested_rpm;

                /* Command zero speed while keeping the inverter enabled.
                   This allows the speed controller to decelerate the motor. */

                motor_set_speed_rpm(0);

                if (link_is_connected(sys_millis))
                {
                    motor_drive_set(true);
                }

                direction_stop_start_ms = sys_millis;
                direction_change_pending = true;

                printf(
                    "[Motor] Decelerating before direction change\r\n"
                );
            }

            /* Direction has not changed */

            else
            {
                direction_change_pending = false;
                pending_rpm = 0;

                motor_set_speed_rpm(requested_rpm);

                if (link_is_connected(sys_millis))
                {
                    motor_drive_set(true);
                    printf("[Motor] Enabled\r\n");
                }
            }
        }
        else
        {
            printf("[Command] Invalid RPM command received\r\n");
        }

        rpm_command_active = false;
        return;
    }

    /* Allow an optional minus sign immediately after R */

    if ((rx == '-') && rpm_command_sign_allowed)
    {
        rpm_command_negative = true;
        rpm_command_sign_allowed = false;
        return;
    }

    /* Process numerical characters */

    if ((rx >= '0') && (rx <= '9'))
    {
        uint32_t digit = (uint32_t)(rx - '0');

        rpm_command_sign_allowed = false;

        if (rpm_command_value > ((MAX_REQUEST_RPM - digit) / 10U))
        {
            rpm_command_invalid = true;
        }
        else
        {
            rpm_command_value =
                (rpm_command_value * 10U) + digit;
        }
    }
    else
    {
        rpm_command_invalid = true;
    }
}

/*******************************************************************************
 * Function Name: link_poll
 *
 * Summary:
 * Drains the UART RX FIFO and processes received bytes.
 ******************************************************************************/

static void link_poll(void)
{
    while (Cy_SCB_UART_GetNumInRxFifo(DEBUG_UART_HW) > 0U)
    {
        uint8_t rx = (uint8_t)Cy_SCB_UART_Get(DEBUG_UART_HW);

        process_uart_byte(rx);
    }
}

/*******************************************************************************
 * Function Name: link_is_connected
 ******************************************************************************/

static bool link_is_connected(uint32_t now)
{
    return heartbeat_seen &&
           ((now - last_heartbeat_ms) < LINK_TIMEOUT_MS);
}

/*******************************************************************************
 * Function Name: link_led_set
 ******************************************************************************/

static void link_led_set(bool on)
{
    Cy_GPIO_Write(
        LINK_LED_PORT,
        LINK_LED_PIN,
        on ? LINK_LED_ON_LEVEL : (1U - LINK_LED_ON_LEVEL)
    );
}

/*******************************************************************************
 * Function Name: print_telemetry
 *
 * Summary:
 * Prints motor state, speed, current, voltage and fault telemetry.
 ******************************************************************************/

static void print_telemetry(void)
{
    //uint8_t current_state = motor[0].sm_ptr->current;

    //uint32_t vdc_adc = mcu[0].dma_results[2];
    //uint32_t iu_adc = mcu[0].dma_results[0];

    //uint16_t fault_status = faults[0].flags.all;

    //float pot_raw = motor[0].sensor_iface_ptr->pot.raw;

    float est_speed = motor[0].vars_ptr->w_final_filt.elec;

    float speed_rpm =
        (est_speed * RAD_S_TO_RPM) / MOTOR_POLE_PAIRS;

    /* Filtered phase-current feedback */

    float current_u = motor[0].sensor_iface_ptr->i_u.filt;
    float current_v = motor[0].sensor_iface_ptr->i_v.filt;
    float current_w = motor[0].sensor_iface_ptr->i_w.filt;

    /* Phase-to-neutral voltage feedback */

    float voltage_u = 0.0f;
    float voltage_v = 0.0f;
    float voltage_w = 0.0f;

#if !defined(MOTOR_CTRL_DISABLE_ADDON_FEATURES)

    if (hw_fcn.ArePhaseVoltagesMeasured(motor[0].motor_instance))
    {
        voltage_u = motor[0].vars_ptr->v_uvw_n_fb.u;
        voltage_v = motor[0].vars_ptr->v_uvw_n_fb.v;
        voltage_w = motor[0].vars_ptr->v_uvw_n_fb.w;
    }

#endif

    printf(
        "[Telemetry] State: %u | En: %u | Pot: %.3f | "
        "Speed: %.2f rad/s | Speed: %.2f rpm | "
        "Vdc_ADC: %lu | Iu_ADC: %lu | "
        "I_U: %.3f A | I_V: %.3f A | I_W: %.3f A | "
        "V_U: %.2f V | V_V: %.2f V | V_W: %.2f V | "
        "Faults: 0x%04X\r\n",

        //(unsigned)current_state,
        (unsigned)motor[0].vars_ptr->en,
        //(double)pot_raw,
        //(double)est_speed,
        (double)speed_rpm,
        //(unsigned long)vdc_adc,
        //(unsigned long)iu_adc,
        (double)current_u,
        (double)current_v,
        (double)current_w,
        (double)voltage_u,
        (double)voltage_v,
        (double)voltage_w,
        (unsigned)fault_status
    );

    fflush(stdout);
}

/*******************************************************************************
 * Function Name: main
 *
 * Summary:
 * Initialises the motor controller, UART and PC connection monitoring.
 ******************************************************************************/

int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();

    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialise controller */

    HW_IFACE_ConnectFcnPointers();

    STATE_MACHINE_Init();

    /* Start with motor drive disabled */

    motor_drive_set(false);

    link_led_set(false);

    /* Enable global interrupts */

    __enable_irq();

    /* Initialise 1 ms system tick */

    Cy_SysTick_Init(
        CY_SYSTICK_CLOCK_SOURCE_CLK_CPU,
        SystemCoreClock / 1000UL
    );

    Cy_SysTick_SetCallback(0U, systick_handler);

    /* Initialise debug UART */

    debug_uart_init();

    /* Use the external command source for motor speed control */

    PARAMS_SetCommandSource(&motor[0], External);

    (void)result;

    bool connected = false;

    uint32_t last_telemetry_ms = sys_millis;

    /***************************************************************************
     * Main Loop
     ***************************************************************************/

    for (;;)
    {
        /* Process incoming UART data */

        link_poll();

        /* Process any pending direction reversal */

        handle_direction_change();

        uint32_t now = sys_millis;

        bool link_now = link_is_connected(now);

        /* Detect connection state changes */

        if (link_now != connected)
        {
            connected = link_now;

            link_led_set(connected);

            if (connected)
            {
                /* PC has connected */

                printf("\x1b[2J\x1b[;H");

                printf(
                    "PMSM FOC Motor Telemetry - "
                    "PC connected, motor enabled\r\n"
                );

                fflush(stdout);

                last_telemetry_ms = now;

                /* Begin at zero RPM */

                direction_change_pending = false;
                pending_rpm = 0;

                motor_set_speed_rpm(0);

                motor_drive_set(true);
            }
            else
            {
                /* PC has disconnected */

                direction_change_pending = false;
                pending_rpm = 0;

                motor_drive_set(false);

                motor_set_speed_rpm(0);

                printf(
                    "[Link] PC disconnected - motor disabled\r\n"
                );

                fflush(stdout);
            }
        }

        /* Keep the motor disabled whenever the PC is disconnected */

        if (!connected)
        {
            motor_drive_set(false);
        }

        /* Periodic telemetry while connected */

        else if ((now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS)
        {
            last_telemetry_ms = now;

            print_telemetry();
        }

#if (CPU_LOAD_CALC_ENABLED)
        MCU_CPULoadCalc();
#endif
    }
}