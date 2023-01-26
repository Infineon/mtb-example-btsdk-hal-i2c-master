/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
/* Read motion sensor acceleration data and display to the PUART */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#if defined(CYW20706A2) || defined(CYW43012C0)
#include "wiced_hal_puart.h"
#endif
#if !defined(CYW20706A2)
#include "cycfg_pins.h"
#endif
#ifdef BTSTACK_VER
#include "wiced_memory.h"
#include "bt_types.h"
#endif

/*****************************    Constants   *****************************/
/* Thread will delay so that sensor values are read every 1s */
#define THREAD_DELAY_IN_MS          (1000)
#ifdef BTSTACK_VER
#define BT_STACK_HEAP_SIZE          1024 * 6
wiced_bt_heap_t *p_default_heap = NULL;
#endif

/*****************************    Variables   *****************************/

/*****************************    Function Prototypes   *******************/
static wiced_result_t
i2c_master_management_callback(wiced_bt_management_evt_t event,
                               wiced_bt_management_evt_data_t *p_event_data);
void sensor_timer_callback(TIMER_PARAM_TYPE arg);
wiced_timer_t msec_timer;

void sensor_init();
void sensor_read();

/******************************************************************************
 *                              Function Definitions
 ******************************************************************************/

/*
 Function name:
 application_start

 Function Description:
 @brief    Starting point of your application. Entry point to the application.
           Set device configuration and start Bluetooth stack initialization.
           The actual application initialization will happen when stack reports
           that Bluetooth device is ready.

 @param void

 @return void
 */
void application_start(void)
{
    wiced_result_t result = WICED_BT_SUCCESS;

    /* WICED_BT_TRACE_ENABLE*/
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#ifdef CYW20706A2
    wiced_hal_puart_init();
    // Please see the User Documentation to reference the valid pins.
    // CTS and RTS are defined non-zero #if PUART_RTS_CTS_FLOW, see wiced_platform.h
    if(!wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, WICED_PUART_CTS, WICED_PUART_RTS))
    {
        WICED_BT_TRACE("wiced_hal_puart_select_uart_pads failed!!\n");
    }
#endif
#ifdef BTSTACK_VER
    /* Create default heap */
    p_default_heap = wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE);
#endif
    WICED_BT_TRACE("************Starting I2C Master Application**********\n\r");
    // use a timer for periodic i2c device reads
    wiced_init_timer( &msec_timer, sensor_timer_callback, 0, WICED_MILLI_SECONDS_TIMER );
    wiced_start_timer( &msec_timer, THREAD_DELAY_IN_MS );
    sensor_init();
}

/*
 Function Name:
 sensor_timer_callback

 Function Description:
 @brief  data is read from sensor periodically

 @param  arg           unused

 @return  none
 */
void sensor_timer_callback(TIMER_PARAM_TYPE arg)
{
    /* Read the sensor data */
    sensor_read();

    /* Send the thread to sleep for a period of time */
    wiced_start_timer( &msec_timer, THREAD_DELAY_IN_MS );
}
