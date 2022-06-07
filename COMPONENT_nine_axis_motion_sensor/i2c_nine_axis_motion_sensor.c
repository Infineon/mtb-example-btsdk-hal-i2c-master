/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
#if defined(CYW20706A2) || defined(CYW43012C0)
#include "wiced_hal_puart.h"
#endif

/*****************************    Constants   *****************************/

/* Motion sensor registers and settings */
#define ACCEL_ADDRESS     (0x6A) /* This is 0xD4 shifted right by 1 */
#define ACCEL_CONFIG_REG  (0x20)
#define ACCEL_CONFIG_VAL  (0x40)

/*****************************    Variables   *****************************/
/* Structure to hold sensor data read from I2C */
struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
} __attribute__((packed)) accelData;

/* Address of first Data register, Total of six registers */
uint8_t accelDataReg = 0x28;

/*****************************    Function Prototypes   *******************/

void sensor_init();
void sensor_read();

/******************************************************************************
 *                              Function Definitions
 ******************************************************************************/

void sensor_init()
{

    /*Initialize I2C and set speed to 400kHz */
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

     /* Write to the configuration register. 2 bytes are sent,first the
      * register location and then the register value */
     uint8_t data[] = {ACCEL_CONFIG_REG, ACCEL_CONFIG_VAL};
     wiced_hal_i2c_write(data, sizeof(data), ACCEL_ADDRESS);
}

void sensor_read()
{
    /* Read the sensor data */
    /* We need to send a write to the data register followed by a read of
    * 6 bytes into the accelData structure */
    wiced_hal_i2c_combined_read((uint8_t *)&accelData,
                                sizeof(accelData),
                                (uint8_t *)&accelDataReg,
                                sizeof(accelDataReg),
                                ACCEL_ADDRESS);

    WICED_BT_TRACE("Ax=%6d       Ay=%6d       Az=%6d\n\r",
                accelData.ax,accelData.ay,accelData.az);
}
