/******************************************************************************
- File Name: main.c
*
- Description: Combined PMSM FOC control, potentiometer management, and real-time 
               motor telemetry readouts using ModusToolbox Motor Suite library.
*
- Related Document: See README.md
*
*******************************************************************************
- (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
- Technologies AG. All rights reserved.
*******************************************************************************/

/*******************************************************************************
- Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "cy_syslib.h"
#include "cy_systick.h"

#include "HardwareIface.h"
#include "Controller.h"
#include "MotorCtrlHWConfig.h"

/*******************************************************************************
- UART & Global Variables
*****************************************************************************/
static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t DEBUG_UART_hal_obj;
static volatile uint32_t sys_millis = 0U;

/* XMC7x/T2G - GCC_ARM: EEPROM storage */
#if defined(APP_KIT_XMC7200_DC_V1) || defined(APP_KIT_TRAVEO_T2G_B_H_MC1)
uint8_t Em_Eeprom_Storage[srss_0_eeprom_0_PHYSICAL_SIZE] __attribute__ ((section(".cy_em_eeprom")));
#endif

/*******************************************************************************
- Function Prototypes
********************************************************************************/
#if defined(APP_KIT_PSC3M5_2GO)
void Motor_Control_POT_Control(void);
#endif

static void systick_handler(void);

/*******************************************************************************
- SysTick Handler
********************************************************************************/
static void systick_handler(void)
{
    sys_millis++;
}

/*******************************************************************************
- Function Name: main
********************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined(CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

#if defined(APP_KIT_XMC7200_DC_V1) || defined(APP_KIT_TRAVEO_T2G_B_H_MC1)
    SCB_DisableDCache(); 
#endif

    result = cybsp_init(); 
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize controller & state machine */
    HW_IFACE_ConnectFcnPointers();
    STATE_MACHINE_Init();

#if defined(CTRL_METHOD_RFO)
    for (uint8_t i = 0U; i < MOTOR_CTRL_NO_OF_MOTOR; i++)
    {
        if (params[i].ctrl.mode == Position_Mode_FOC_Encoder_Align_Startup)
        {
            MOTOR_CTRL_SetDriveEnable(&motor[i], false);
        }
    }
#endif

    /* Enable global interrupts */
    __enable_irq();

    /* Configure SysTick for 1 ms interrupt */
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, SystemCoreClock / 1000UL);
    Cy_SysTick_SetCallback(0U, systick_handler);

    /* Initialize Debug UART */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Print startup banner */
    printf("\x1b[2J\x1b[;H");
    printf("========================================\r\n");
    printf("PMSM FOC Motor Telemetry Readout\r\n");
    printf("========================================\r\n");
    fflush(stdout);

    uint32_t last_telemetry_ms = sys_millis;

    (void)(result);

    for (;;)
    {
#if defined(APP_KIT_PSC3M5_2GO)
        Motor_Control_POT_Control();
#endif

#if (CPU_LOAD_CALC_ENABLED)
        MCU_CPULoadCalc();
#endif

        uint32_t now = sys_millis;

        /* Periodic Motor Telemetry Readout (Every 250 ms) */
        if ((now - last_telemetry_ms) >= 250U)
        {
            last_telemetry_ms = now;

            /* Extract parameters mapped from the Motor Control Library */
            uint8_t  current_state = motor[0].sm_ptr->current;     /* Current state machine state[cite: 18] */
            uint32_t vdc_adc       = mcu[0].dma_results[2];         /* DC bus voltage raw ADC count[cite: 26] */
            uint32_t iu_adc        = mcu[0].dma_results[0];         /* Phase U current raw ADC count[cite: 26] */
            uint16_t fault_status  = faults[0].flags.all;           /* Active system fault flags[cite: 65] */
            float    est_speed     = motor[0].vars_ptr->w_final_filt.elec; /* Filtered electrical speed feedback[cite: 52] */

            printf("[Telemetry] State: %u | Speed: %.2f rad/s | Speed: %.2f rpm | Vdc_ADC: %lu | Iu_ADC: %lu | Faults: 0x%04X\r\n",
                   (unsigned)current_state,
                   (double)est_speed,
					(double)(est_speed/6.3)*10,
                   (unsigned long)vdc_adc,
                   (unsigned long)iu_adc,
                   (unsigned)fault_status);
            fflush(stdout);
        }
    }
}

#if defined(APP_KIT_PSC3M5_2GO)
/*******************************************************************************
- Function Name: Motor_Control_POT_Control
********************************************************************************
- Summary: Controls potentiometer input and enables/disables the gate drive.
*******************************************************************************/
void Motor_Control_POT_Control(void)
{
    static bool flag;
    static float t_min_configured;
    static bool state_check = false;

    /* Disable the driver when pot value is <= 5% */
    if (motor[0].params_ptr->sys.cmd.source == Internal)
    {
        if (motor[0].sensor_iface_ptr->pot.raw >= 0.05f)
        {
            if (flag == true)
            {
                flag = false;
                motor[0].vars_ptr->en = true;
            }
        }
        else if (motor[0].sensor_iface_ptr->pot.raw <= 0.025f)
        {
            flag = true;
            motor[0].vars_ptr->en = false;
        }
    }
    else
    {
        flag = false;
    }

    /* Enable or disable Gate driver */
    Cy_GPIO_Write(EN_IPM_PORT, EN_IPM_NUM, motor[0].vars_ptr->en);

    /* Reduce the minimum time for current measurement in profile mode */
    if ((Prof_Rot_Lock <= motor[0].sm_ptr->current) &&
        (motor[0].sm_ptr->current <= Prof_Lq) &&
        (motor[0].params_ptr->ctrl.mode == Profiler_Mode))
    {
        if (state_check == false)
        {
            MCU_EnterCriticalSection();
            t_min_configured = motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min;
            motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min = (t_min_configured <= 3.0f) ? 0.0f : t_min_configured - 3.0f;
            motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_d_min = 2.0f * motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min * motor[0].params_ptr->sys.samp.fpwm;
            MCU_ExitCriticalSection();
            state_check = true;
        }
    }
    else
    {
        if (state_check == true)
        {
            MCU_EnterCriticalSection();
            motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min = t_min_configured;
            motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_d_min = 2.0f * motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min * motor[0].params_ptr->sys.samp.fpwm;
            MCU_ExitCriticalSection();
        }
        state_check = false;
    }
}
#endif