/******************************************************************************
 * File Name:   main.c
 *
 * Description:
 * Reads the X7 Hall sensors on P3.0, P3.1 and P3.2, calculates motor RPM,
 * and determines rotation direction from Hall transitions.
 ******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"
#include <stdint.h>
#include <stdio.h>
#include "cy_syslib.h"
#include "cy_systick.h"

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define MOTOR_POLE_PAIRS             4U
#define HALL_TRANSITIONS_PER_ELECTRICAL_REV 6U
#define HALL_TRANSITIONS_PER_MECHANICAL_REV \
    (MOTOR_POLE_PAIRS * HALL_TRANSITIONS_PER_ELECTRICAL_REV)

/*******************************************************************************
 * UART variables
 ******************************************************************************/

static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t DEBUG_UART_hal_obj;

/*******************************************************************************
 * Millisecond Tick Counter (SysTick)
 ******************************************************************************/

static volatile uint32_t sys_millis = 0U;

static void systick_handler(void)
{
    sys_millis++;
}

/*******************************************************************************
 * Hall sensor helpers
 ******************************************************************************/

static uint8_t read_hall_state(void)
{
    uint8_t h1 = (uint8_t)Cy_GPIO_Read(GPIO_PRT3, 0);
    uint8_t h2 = (uint8_t)Cy_GPIO_Read(GPIO_PRT3, 1);
    uint8_t h3 = (uint8_t)Cy_GPIO_Read(GPIO_PRT3, 2);

    return (uint8_t)((h1 << 2) | (h2 << 1) | h3);
}

static uint8_t hall_state_valid(uint8_t hall)
{
    /* 000 and 111 are invalid for the usual three-Hall arrangement. */
    return (hall != 0U) && (hall != 7U);
}

static int8_t get_hall_direction(uint8_t prev, uint8_t curr)
{
    /* Forward vs Reverse transition check based on standard 6-state sequence */
    switch (prev)
    {
        case 1:
            if (curr == 3) return  1; /* Forward */
            if (curr == 5) return -1; /* Reverse */
            break;
        case 3:
            if (curr == 2) return  1;
            if (curr == 1) return -1;
            break;
        case 2:
            if (curr == 6) return  1;
            if (curr == 3) return -1;
            break;
        case 6:
            if (curr == 4) return  1;
            if (curr == 2) return -1;
            break;
        case 4:
            if (curr == 5) return  1;
            if (curr == 6) return -1;
            break;
        case 5:
            if (curr == 1) return  1;
            if (curr == 4) return -1;
            break;
        default:
            break;
    }
    return 0; // Unknown or skipped transition
}

/*******************************************************************************
 * Main
 ******************************************************************************/

int main(void)
{
    cy_rslt_t result;

#if defined(CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Configure SysTick for a 1 ms interrupt using the CPU clock */
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, SystemCoreClock / 1000UL);
    Cy_SysTick_SetCallback(0U, systick_handler);

    /* Initialize debug UART. */
    result = (cy_rslt_t)Cy_SCB_UART_Init(
        DEBUG_UART_HW,
        &DEBUG_UART_config,
        &DEBUG_UART_context);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    result = mtb_hal_uart_setup(
        &DEBUG_UART_hal_obj,
        &DEBUG_UART_hal_config,
        &DEBUG_UART_context,
        NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Configure X7 Hall inputs: H1=P3.0, H2=P3.1, H3=P3.2. */
    Cy_GPIO_Pin_FastInit(GPIO_PRT3, 0, CY_GPIO_DM_HIGHZ, 0UL, HSIOM_SEL_GPIO);
    Cy_GPIO_Pin_FastInit(GPIO_PRT3, 1, CY_GPIO_DM_HIGHZ, 0UL, HSIOM_SEL_GPIO);
    Cy_GPIO_Pin_FastInit(GPIO_PRT3, 2, CY_GPIO_DM_HIGHZ, 0UL, HSIOM_SEL_GPIO);

    printf("\x1b[2J\x1b[;H");
    printf("========================================\r\n");
    printf("Hall Sensor RPM & Direction Test\r\n");
    printf("Pole pairs: %u\r\n", (unsigned)MOTOR_POLE_PAIRS);
    printf("Transitions per mechanical revolution: %u\r\n",
           (unsigned)HALL_TRANSITIONS_PER_MECHANICAL_REV);
    printf("========================================\r\n");

    uint8_t previous_hall = read_hall_state();
    uint32_t previous_transition_ms = 0U;
    uint8_t have_previous_transition = 0U;

    for (;;)
    {
        uint8_t hall = read_hall_state();

        if ((hall != previous_hall) && hall_state_valid(hall))
        {
            uint32_t now_ms = sys_millis;
            int8_t dir = get_hall_direction(previous_hall, hall);

            if (have_previous_transition)
            {
                /* Unsigned subtraction also handles tick counter wraparound. */
                uint32_t delta_ms = now_ms - previous_transition_ms;

                if (delta_ms > 0U)
                {
                    uint32_t rpm =
                        60000U /
                        (HALL_TRANSITIONS_PER_MECHANICAL_REV * delta_ms);

                    const char* dir_str = (dir > 0) ? "FWD" : ((dir < 0) ? "REV" : "UNK");

                    printf("Hall=0x%02X  Dir=%s  Interval=%lu ms  RPM=%lu\r\n",
                           (unsigned)hall,
                           dir_str,
                           (unsigned long)delta_ms,
                           (unsigned long)rpm);
                }
            }

            previous_transition_ms = now_ms;
            have_previous_transition = 1U;
            previous_hall = hall;
        }
    
}}