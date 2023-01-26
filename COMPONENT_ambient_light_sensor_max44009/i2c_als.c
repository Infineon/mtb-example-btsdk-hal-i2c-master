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
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#if defined(CYW20706A2)
#define I2C_SCL 0
#define I2C_SDA 0
#else
#include "cycfg_pins.h"
#endif

#include "max_44009.h"

/*****************************    Constants   *****************************/

/*****************************    Variables   *****************************/
/* Structure to hold sensor data read from I2C */

/*****************************    Function Prototypes   *******************/

void sensor_init();
void sensor_read();

/******************************************************************************
 *                              Function Definitions
 ******************************************************************************/

void sensor_init()
{
    max44009_user_set_t ambient_light_config;

    memset(&ambient_light_config, 0, sizeof(max44009_user_set_t));

    ambient_light_config.scl_pin = I2C_SCL;
    ambient_light_config.sda_pin = I2C_SDA;
    ambient_light_config.irq_pin = WICED_HAL_GPIO_PIN_UNUSED;
    ambient_light_config.irq_enable_reg_value = 0;
    ambient_light_config.cfg_reg_value = 0;
    ambient_light_config.upper_threshold_reg_value = 0;
    ambient_light_config.low_threshold_reg_value = 0;
    ambient_light_config.threshold_timer_reg_value = 0;
    max44009_init(&ambient_light_config, NULL, NULL);
    WICED_BT_TRACE("init_light_sensor done\n");
}

void sensor_read()
{
    WICED_BT_TRACE("Ambient light level = %6d\n", max44009_read_ambient_light());
}
