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
#include "cy_rtc.h"
#include "cycfg_peripherals.h"


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/_types.h>

#include "ParamConfig.h"

#include "cy_pdl.h"


/*******************************************************************************
 * Macros
 ******************************************************************************/

/* PC link settings */


#define CURRENT_STREAM_ENABLED      (1U)
#define CURRENT_STREAM_PERIOD_MS    (5U)

/* Set to 1 to bring back the noisy debug prints */
#define DEBUG_VERBOSE               (0U)

#if CURRENT_STREAM_ENABLED
#define TELEMETRY_PERIOD_MS         (10U)
#else
#define TELEMETRY_PERIOD_MS         (100U)
#endif

#define HEARTBEAT_BYTE              ((uint8_t)'H')
#define LINK_TIMEOUT_MS             (1000U)
#define TIME_UPDATE_PERIOD_MS		(500U)

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



#define USER_BUTTON_PORT GPIO_PRT4
#define USER_BUTTON_PIN  6U

static volatile bool sleep_button_pressed = false;
static bool sleep_pending = false;
static uint32_t sleep_stop_time = 0U;
static bool sleeping = false;
static volatile bool wake_from_sleep = false;

static void sleep_button_isr(void)
{
    Cy_GPIO_ClearInterrupt(
        CYBSP_USER_BTN_PORT,
        CYBSP_USER_BTN_PIN
    );

    if (sleeping)
    {
        wake_from_sleep = true;
    }
    else
    {
        sleep_button_pressed = true;
    }
}

static void sleep_button_init(void)
{
    cy_stc_sysint_t button_irq_config =
    {
        .intrSrc = CYBSP_USER_BTN_IRQ,
        .intrPriority = 2U
    };

    Cy_GPIO_SetDrivemode(
        CYBSP_USER_BTN_PORT,
        CYBSP_USER_BTN_PIN,
        CY_GPIO_DM_PULLUP
    );

    Cy_GPIO_SetInterruptEdge(
        CYBSP_USER_BTN_PORT,
        CYBSP_USER_BTN_PIN,
        CY_GPIO_INTR_FALLING
    );

    Cy_GPIO_SetInterruptMask(
        CYBSP_USER_BTN_PORT,
        CYBSP_USER_BTN_PIN,
        1UL
    );

    Cy_SysInt_Init(
        &button_irq_config,
        sleep_button_isr
    );

    NVIC_ClearPendingIRQ(CYBSP_USER_BTN_IRQ);
    NVIC_EnableIRQ(CYBSP_USER_BTN_IRQ);
}

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

static bool source_command_active = false;
static uint8_t source_command_index = 0;
static char source_command[8];

static bool webpage_control_enabled = true;

/* UART RTC time command parser */

static bool time_command_active = false;

static uint8_t time_command_index = 0;

static char time_command[32];
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

static void print_telemetry(long now);

static void M1_Hall_GPIO_Init(void);

static bool pot_armed = false;

static void enter_deep_sleep(void)
{
    cy_en_syspm_status_t status;

    /* Disable motor */
    motor_drive_set(false);

    /* Mark MCU as sleeping */
    sleeping = true;

    sleep_button_pressed = false;
    wake_from_sleep = false;

    /* Clear pending button interrupt */
    Cy_GPIO_ClearInterrupt(
        CYBSP_USER_BTN_PORT,
        CYBSP_USER_BTN_PIN
    );

    NVIC_ClearPendingIRQ(CYBSP_USER_BTN_IRQ);
    NVIC_EnableIRQ(CYBSP_USER_BTN_IRQ);

    __enable_irq();

    /* Configure Deep Sleep */
    Cy_SysPm_SetDeepSleepMode(CY_SYSPM_MODE_DEEPSLEEP);

    printf("[Sleep] \r\n");
    fflush(stdout);

    /* Enter Deep Sleep */
    status = Cy_SysPm_CpuEnterDeepSleep(
        CY_SYSPM_WAIT_FOR_INTERRUPT
    );

    /* Execution resumes after wake-up */

    if (status != CY_SYSPM_SUCCESS)
    {
        sleeping = false;
        printf("[Sleep] Deep Sleep entry failed: %d\r\n", status);
        return;
    }

    sleeping = false;

    printf("[Wake] MCU waking up\r\n");


	cybsp_init();
	HW_IFACE_ConnectFcnPointers();
	STATE_MACHINE_Init();
	M1_Hall_GPIO_Init();
	debug_uart_init();
	for (int i = 0; i < 10; i++)
	{
	    Cy_SCB_UART_PutString(DEBUG_UART_HW, "WAKE:\n");
	    while (!Cy_SCB_UART_IsTxComplete(DEBUG_UART_HW)) { }
	    Cy_SysLib_Delay(30);
	}
	sleep_button_init();
	__enable_irq();  
	Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, SystemCoreClock / 1000UL);
	Cy_SysTick_SetCallback(0U, systick_handler);
    printf("[Wake] UART and SysTick restored\r\n");
    fflush(stdout);
	sleep_button_pressed = false;
	wake_from_sleep = false;
	sleep_pending = false;
	sleep_stop_time = 0U;
	pot_armed = false;   
	motor_drive_set(false);
}


static void RTC_Init_Time(void)
{
    cy_en_rtc_status_t rtc_status;

    // Initialise the RTC
    rtc_status = Cy_RTC_Init(&srss_0_rtc_0_config);

    if (rtc_status != CY_RTC_SUCCESS)
    {
        printf("RTC initialisation failed: %d\n", rtc_status);
        return;
    }

    // Set the initial time.
    // Example: 15:52:00, 10 November 2026.
    
}

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
 * Function Name: M1_Hall_GPIO_Init
 *
 * Summary:
 * Configures the X7 Hall sensor inputs for Motor 1.
 *
 * X7:
 * P3.0 = Hall U
 * P3.1 = Hall V
 * P3.2 = Hall W
 ******************************************************************************/

static void M1_Hall_GPIO_Init(void)
{
    Cy_GPIO_Pin_FastInit(
        GPIO_PRT3,
        0U,
        CY_GPIO_DM_HIGHZ,
        0UL,
        HSIOM_SEL_GPIO
    );

    Cy_GPIO_Pin_FastInit(
        GPIO_PRT3,
        1U,
        CY_GPIO_DM_HIGHZ,
        0UL,
        HSIOM_SEL_GPIO
    );

    Cy_GPIO_Pin_FastInit(
        GPIO_PRT3,
        2U,
        CY_GPIO_DM_HIGHZ,
        0UL,
        HSIOM_SEL_GPIO
    );

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
    }

    /* Safety timeout */

    else if ((now - direction_stop_start_ms) >=
             REVERSE_STOP_TIMEOUT_MS)
    {
        direction_change_pending = false;
        pending_rpm = 0;

        motor_set_speed_rpm(0);
        motor_drive_set(false);
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
    /* =========================================================
       HEARTBEAT
       ========================================================= */
    if (rx == HEARTBEAT_BYTE)
    {
        last_heartbeat_ms = sys_millis;
        heartbeat_seen = true;
        return;
    }


    /* =========================================================
       PROCESS RTC TIME COMMAND
       This MUST be before command-start detection.
       ========================================================= */
    if (time_command_active)
    {
        if ((rx == '\n') || (rx == '\r'))
        {
            int hour, minute, second;
            int day, month, year;

            time_command[time_command_index] = '\0';

            if (sscanf(
                    time_command,
                    ":%d:%d:%d,%d/%d/%d",
                    &hour,
                    &minute,
                    &second,
                    &day,
                    &month,
                    &year
                ) == 6)
            {
                if (hour >= 0 && hour <= 23 &&
                    minute >= 0 && minute <= 59 &&
                    second >= 0 && second <= 59 &&
                    day >= 1 && day <= 31 &&
                    month >= 1 && month <= 12 &&
                    year >= 0 && year <= 99)
                {
                    cy_en_rtc_status_t rtc_status;

                    rtc_status = Cy_RTC_SetDateAndTimeDirect(
                        second,
                        minute,
                        hour,
                        day,
                        month,
                        year
                    );

                    if (rtc_status == CY_RTC_SUCCESS)
                    {
                        printf(
                            "[RTC] Updated: %02d:%02d:%02d "
                            "%02d/%02d/20%02d\r\n",
                            hour,
                            minute,
                            second,
                            day,
                            month,
                            year
                        );
                    }
                    else
                    {
                        printf(
                            "[RTC] Failed to update: %d\r\n",
                            rtc_status
                        );
                    }
                }
                else
                {
                    printf("[RTC] Invalid date/time values\r\n");
                }
            }
            else
            {
                printf(
                    "[RTC] Invalid time command: %s\r\n",
                    time_command
                );
            }

            time_command_active = false;
            time_command_index = 0U;

            return;
        }

        /* Store incoming RTC characters */
        if (time_command_index < (sizeof(time_command) - 1U))
        {
            time_command[time_command_index++] = (char)rx;
        }
        else
        {
            time_command_active = false;
            time_command_index = 0U;

            printf("[RTC] Time command too long\r\n");
        }

        return;
    }


    /* =========================================================
       PROCESS SOURCE COMMAND
       S:PAGE = External/Webpage control
       S:POT  = Internal/Potentiometer control
       ========================================================= */
    if (source_command_active)
    {
        /* Ignore colon */
        if (rx == ':')
        {
            return;
        }

        /* End of source command */
        if ((rx == '\n') || (rx == '\r'))
        {
            source_command[source_command_index] = '\0';

            if (strcmp(source_command, "PAGE") == 0)
            {
                webpage_control_enabled = true;

                PARAMS_SetCommandSource(
                    &motor[0],
                    External
                );
            }
            else if (strcmp(source_command, "POT") == 0)
            {
                webpage_control_enabled = false;

                PARAMS_SetCommandSource(
                    &motor[0],
                    Internal
                );

                direction_change_pending = false;
                pending_rpm = 0;


            
            }

            source_command_active = false;
            source_command_index = 0U;

            return;
        }

        /* Store source command characters */
        if (source_command_index < (sizeof(source_command) - 1U))
        {
            source_command[source_command_index++] = (char)rx;
        }
        else
        {
            source_command_active = false;
            source_command_index = 0U;

        }

        return;
    }


    /* =========================================================
       PROCESS RPM COMMAND
       ========================================================= */
    if (rpm_command_active)
    {
        /* Ignore RPM commands if webpage control is disabled */
        if (!webpage_control_enabled)
        {
            rpm_command_active = false;
            return;
        }

        /* End of RPM command */
        if ((rx == '\r') || (rx == '\n'))
        {
            if (!rpm_command_invalid)
            {
                int32_t requested_rpm =
                    (int32_t)rpm_command_value;

                /* Apply negative sign */
                if (rpm_command_negative)
                {
                    requested_rpm = -requested_rpm;
                }

                bool requested_reverse =
                    (requested_rpm < 0);


                /* ---------------------------------------------
                   STOP MOTOR
                   --------------------------------------------- */
                if (rpm_command_value <= MIN_REQUEST_RPM)
                {
                    direction_change_pending = false;
                    pending_rpm = 0;

                    motor_set_speed_rpm(0);
                    motor_drive_set(false);

                    printf("[Motor] Stopped\r\n");
                }


                /* ---------------------------------------------
                   DIRECTION CHANGE
                   --------------------------------------------- */
                else if (requested_reverse != reversed)
                {
                    pending_rpm = requested_rpm;

                    motor_set_speed_rpm(0);

                    if (link_is_connected(sys_millis))
                    {
                        motor_drive_set(true);
                    }

                    direction_stop_start_ms = sys_millis;
                    direction_change_pending = true;
                }


                /* ---------------------------------------------
                   NORMAL SPEED COMMAND
                   --------------------------------------------- */
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
                printf(
                    "[Command] Invalid RPM command received\r\n"
                );
            }

            rpm_command_active = false;

            return;
        }


        /* Optional minus sign */
        if ((rx == '-') && rpm_command_sign_allowed)
        {
            rpm_command_negative = true;
            rpm_command_sign_allowed = false;

            return;
        }


        /* Numerical characters */
        if ((rx >= '0') && (rx <= '9'))
        {
            uint32_t digit = (uint32_t)(rx - '0');

            rpm_command_sign_allowed = false;

            /* Check for overflow / maximum RPM */
            if (rpm_command_value >
                ((MAX_REQUEST_RPM - digit) / 10U))
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

        return;
    }


    /* =========================================================
       START NEW COMMANDS
       Only reached when no command is currently active.
       ========================================================= */

    /* Start RTC command */
    if (rx == 'T')
    {
        time_command_active = true;

        source_command_active = false;
        rpm_command_active = false;

        time_command_index = 0U;

        memset(time_command, 0, sizeof(time_command));

        return;
    }


    /* Start RPM command */
    if (rx == 'R')
    {
        rpm_command_active = true;

        time_command_active = false;
        source_command_active = false;

        rpm_command_invalid = false;
        rpm_command_value = 0U;
        rpm_command_negative = false;
        rpm_command_sign_allowed = true;

        return;
    }


    /* Start source command */
    if (rx == 'S')
    {
        source_command_active = true;

        time_command_active = false;
        rpm_command_active = false;

        source_command_index = 0U;

        memset(source_command, 0, sizeof(source_command));

        return;
    }


    /* =========================================================
       IGNORE UNRECOGNISED BYTES
       ========================================================= */
    return;
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

		#if DEBUG_VERBOSE
		        printf("[UART RX] 0x%02X '%c'\r\n",
		               rx,
		               (rx >= 32U && rx <= 126U) ? rx : '.');
		#endif


        if (rx == HEARTBEAT_BYTE)
        {
            link_led_set(true);
        }

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


static void send_rtc_time(void)
{
    cy_stc_rtc_config_t rtc_now;
    char buffer[40];

    Cy_RTC_GetDateAndTime(&rtc_now);

    snprintf(
        buffer,
        sizeof(buffer),
        "TIME:%02lu:%02lu:%02lu,%02lu/%02lu/20%02lu\n",
        (unsigned long)rtc_now.hour,
        (unsigned long)rtc_now.min,
        (unsigned long)rtc_now.sec,
        (unsigned long)rtc_now.date,
        (unsigned long)rtc_now.month,
        (unsigned long)rtc_now.year
    );

    printf("%s", buffer);
    fflush(stdout);
}


/*******************************************************************************
 * Function Name: print_telemetry
 *
 * Summary:
 * Prints motor state, speed, current, voltage and fault telemetry.
 ******************************************************************************/

static void print_telemetry(long now)
{
    uint8_t current_state = motor[0].sm_ptr->current;


    uint32_t fault_status = faults[0].flags.sw.reg;

  

    float est_speed = motor[0].vars_ptr->w_final_filt.elec;
	float max_speed_rad_s =
    motor[0].params_ptr->sys.cmd.w_max.mech;

	float requested_rpm = 0.0f;
	

	if (motor[0].params_ptr->sys.cmd.source == External)
	{
	    requested_rpm =
	        motor[0].vars_ptr->cmd_ext *
	        max_speed_rad_s *
	        RAD_S_TO_RPM *
	        motor[0].vars_ptr->dir;
	}
	else
	{
		float pot_raw = motor[0].sensor_iface_ptr->pot.raw;
	    requested_rpm = pot_raw * MOTOR_MAX_SPEED;
	}
    float speed_rpm =
        (est_speed * RAD_S_TO_RPM) / MOTOR_POLE_PAIRS;



    /* Hall sensor readings */
    unsigned int hall_signal = 999U;
	#if DEBUG_VERBOSE
    uint32_t gpio_u = Cy_GPIO_Read(GPIO_PRT3, 0U);
    uint32_t gpio_v = Cy_GPIO_Read(GPIO_PRT3, 1U);
    uint32_t gpio_w = Cy_GPIO_Read(GPIO_PRT3, 2U);

    printf("[Hall Direct] U:%lu V:%lu W:%lu\r\n",
           (unsigned long)gpio_u,
           (unsigned long)gpio_v,
           (unsigned long)gpio_w);
#endif

    if (motor[0].hall_ptr != NULL)
    {
        hall_signal =
            (unsigned int)motor[0].hall_ptr->signal.uvw;
    }

	int32_t iu_ma =
        (int32_t)(motor[0].sensor_iface_ptr->i_u.filt * 1000.0f);

    int32_t iv_ma =
        (int32_t)(motor[0].sensor_iface_ptr->i_v.filt * 1000.0f);

    int32_t iw_ma =
        (int32_t)(motor[0].sensor_iface_ptr->i_w.filt * 1000.0f);


   printf(
    "[Telemetry] State: %u | En: %u  "
    "Speed: %.2f rpm | "
    "ReqRPM: %ld | "
    "Faults: 0x%08lX | "
    "Hall: %u | Time: %lu | Iu: %ld | Iv: %ld | Iw: %ld \r\n",

        (unsigned)current_state,
        (unsigned)motor[0].vars_ptr->en,
        (double)speed_rpm,
		(long)requested_rpm,
        (unsigned long)fault_status,
        hall_signal,
		(unsigned long)now,
        (long)iu_ma,
        (long)iv_ma,
        (long)iw_ma
		
    );

    fflush(stdout);
}




void Motor_Control_POT_Control(void)
{
    static bool pot_armed = false;

    float pot = motor[0].sensor_iface_ptr->pot.raw;


    if (motor[0].params_ptr->sys.cmd.source != Internal)
    {
        return;
    }

    if (pot <= 0.025f)
    {
        pot_armed = true;
        motor_drive_set(false);
    }
    else if ((pot >= 0.05f) && pot_armed)
    {
        motor_drive_set(true);
    }
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

HW_IFACE_ConnectFcnPointers();

STATE_MACHINE_Init();

/* Reconfigure Hall pins AFTER state machine initialisation */
M1_Hall_GPIO_Init();
sleep_button_init();

motor_drive_set(false);
link_led_set(false);

__enable_irq();

    /* Initialise 1 ms system tick */

    Cy_SysTick_Init(
        CY_SYSTICK_CLOCK_SOURCE_CLK_CPU,
        SystemCoreClock / 1000UL
    );

    Cy_SysTick_SetCallback(0U, systick_handler);

    /* Initialise debug UART */

    debug_uart_init();
	
	RTC_Init_Time();

    /* Use the external command source for motor speed control */

    PARAMS_SetCommandSource(&motor[0], External);

    (void)result;

    bool connected = false;

    uint32_t last_telemetry_ms = sys_millis;
	uint32_t last_time_ms = sys_millis;
    uint32_t last_stream_ms = sys_millis;

    /***************************************************************************
     * Main Loop
     ***************************************************************************/

    for (;;)
    {
	/* Detect a button press */
if (sleep_button_pressed)
{
	printf("Button press");
    sleep_button_pressed = false;

    if (!sleep_pending)
    {
        sleep_pending = true;

        /* Request a controlled deceleration */
        motor_set_speed_rpm(0);

        printf("[Sleep] Stop requested\r\n");
    }
}

/* Wait for the motor to stop before sleeping */
if (sleep_pending)
{
    float mechanical_speed =
        motor[0].vars_ptr->w_final_filt.elec /
        MOTOR_POLE_PAIRS;

    if (fabsf(mechanical_speed) < 2.0f)
    {
        if (sleep_stop_time == 0U)
        {
            sleep_stop_time = sys_millis;
        }

        /* Require low speed to remain stable for 500 ms */
        if ((sys_millis - sleep_stop_time) >= 500U)
        {
            motor_drive_set(false);

            static const char sleep_message[] = "SLEEP:\n";

			printf("[Sleep] Sending request to Pico W\r\n");
			fflush(stdout);
			
			/* Send directly through the UART connected to the Pico */
			Cy_SCB_UART_PutString(DEBUG_UART_HW, sleep_message);
			
			/* Wait until the final byte has physically left the UART */
			while (!Cy_SCB_UART_IsTxComplete(DEBUG_UART_HW))
			{
			}
			
			printf("[Sleep] Request transmitted\r\n");
			fflush(stdout);

            sleep_pending = false;
            sleep_stop_time = 0U;
			Cy_GPIO_Write(
			        LINK_LED_PORT,
			        LINK_LED_PIN,
			       0);

            enter_deep_sleep();
        }
    }
    else
    {
        sleep_stop_time = 0U;
    }
}
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

                last_telemetry_ms = now;

                /* Begin at zero RPM */

                direction_change_pending = false;
                pending_rpm = 0;

                if (motor[0].params_ptr->sys.cmd.source == External)
					{
					    motor_set_speed_rpm(0);
					    motor_drive_set(true);
					}
            }
            else
            {
                /* PC has disconnected */

                direction_change_pending = false;
                pending_rpm = 0;

                if (motor[0].params_ptr->sys.cmd.source == External)
					{
					    motor_drive_set(false);
					    motor_set_speed_rpm(0);
					}

                fflush(stdout);
            }
        }
        /* Keep the motor disabled whenever the PC is disconnected */


bool pot_mode =
    (motor[0].params_ptr->sys.cmd.source == Internal);

if (pot_mode)
{
    /* Potentiometer control does not require a PC heartbeat */
    Motor_Control_POT_Control();
}
else
{
    if (!connected)
    {
        motor_drive_set(false);
    }
}

/* Send telemetry whenever the Pico heartbeat is active */
if (connected)
{
    if ((now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS)
    {
        last_telemetry_ms = now;
        print_telemetry(now);
    }

    if ((now - last_time_ms) >= TIME_UPDATE_PERIOD_MS)
    {
        last_time_ms = now;
        send_rtc_time();
    }
}

#if (CPU_LOAD_CALC_ENABLED)
        MCU_CPULoadCalc();
#endif
    }
}