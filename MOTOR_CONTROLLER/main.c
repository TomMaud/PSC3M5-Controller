/******************************************************************************
 * File Name:   main.c
 *
 * Description:
 * Test X7 Hall sensor inputs on P3.0, P3.1 and P3.2.
 *
 * The three Hall inputs are configured as ordinary GPIO inputs and their
 * current states are printed continuously over the debug UART.
 ******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_hal.h"
#include <stdint.h>

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t DEBUG_UART_hal_obj;


/*******************************************************************************
 * Function Definitions
 ******************************************************************************/

int main(void)
{
    cy_rslt_t result;

#if defined(CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    cyhal_wdt_free(&wdt_obj);
#endif

    /***************************************************************************
     * Initialize the device and board peripherals
     ***************************************************************************/

    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /***************************************************************************
     * Enable global interrupts
     ***************************************************************************/

    __enable_irq();

    /***************************************************************************
     * Debug UART initialization
     ***************************************************************************/

    result = (cy_rslt_t)Cy_SCB_UART_Init(
        DEBUG_UART_HW,
        &DEBUG_UART_config,
        &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(
        &DEBUG_UART_hal_obj,
        &DEBUG_UART_hal_config,
        &DEBUG_UART_context,
        NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-IO */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

    /* Retarget-IO initialization failed */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /***************************************************************************
     * Configure X7 Hall sensor inputs
     *
     * X7:
     *   Pin 2 = H1 = P3.0
     *   Pin 3 = H2 = P3.1
     *   Pin 4 = H3 = P3.2
     *
     * Configure all three as ordinary GPIO inputs.
     ***************************************************************************/

    Cy_GPIO_Pin_FastInit(
        GPIO_PRT3,
        0,
        CY_GPIO_DM_HIGHZ,
        0UL,
        HSIOM_SEL_GPIO);

    Cy_GPIO_Pin_FastInit(
        GPIO_PRT3,
        1,
        CY_GPIO_DM_HIGHZ,
        0UL,
        HSIOM_SEL_GPIO);

    Cy_GPIO_Pin_FastInit(
        GPIO_PRT3,
        2,
        CY_GPIO_DM_HIGHZ,
        0UL,
        HSIOM_SEL_GPIO);

    /***************************************************************************
     * Startup message
     ***************************************************************************/

    printf("\x1b[2J\x1b[;H");

    printf("========================================\r\n");
    printf("X7 Hall Sensor GPIO Test\r\n");
    printf("========================================\r\n");
    printf("P3.0 = H1\r\n");
    printf("P3.1 = H2\r\n");
    printf("P3.2 = H3\r\n");
    printf("========================================\r\n");
    printf("Reading Hall inputs...\r\n\r\n");


    /***************************************************************************
     * Main loop
     ***************************************************************************/
	uint8_t pasthall = 0;

    for (;;)
    {
        /* Read the three Hall inputs */
        uint8_t h1 = Cy_GPIO_Read(GPIO_PRT3, 0);
        uint8_t h2 = Cy_GPIO_Read(GPIO_PRT3, 1);
        uint8_t h3 = Cy_GPIO_Read(GPIO_PRT3, 2);

        /* Combine into a 3-bit Hall value */
        uint8_t hall = (h1 << 2) | (h2 << 1) | h3;

        if (hall != pasthall){
        printf(
            "H1=%u  H2=%u  H3=%u  Hall=%u%u%u  Value=0x%02X\r\n",
            h1,
            h2,
            h3,
            h1,
            h2,
            h3,
            hall);
		pasthall = hall;
}

    }
}

/* [] END OF FILE */