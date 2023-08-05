/*
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "nvs_flash.h"

#include "stm32_legacy.h"
#include "platform.h"
#include "system.h"
#define DEBUG_MODULE "APP_MAIN"
#include "debug_cf.h"


#include "mpu6050.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#define PROXIMITY_ENABLED


// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000

//static bool isinit_i2cPort[2] = {0, 0};

// Cost definitions of busses
// static const I2cDef sensorBusDef = {
//     .i2cPort            = I2C_NUM_0,
//     .i2cClockSpeed      = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
//     .gpioSCLPin         = CONFIG_I2C0_PIN_SCL,
//     .gpioSDAPin         = CONFIG_I2C0_PIN_SDA,
//     .gpioPullup         = GPIO_PULLUP_DISABLE,
// };

// I2cDrv sensorsBus = {
//     .def                = &sensorBusDef,
// };

#define xGyroreference  1

void app_main()
{


    /*
    * Initialize the platform and Launch the system task
    * app_main will initialize and start everything
    */

    /* initialize nvs flash prepare for Wi-Fi */
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    /*Initialize the platform.*/
    if (platformInit() == false) {
        while (1);//if  firmware is running on the wrong hardware, Halt
    }

    /*launch the system task */
    //systemLaunch();

    i2cdrvInit(&sensorsBus);

    mpu6050Init(&sensorsBus);

    if(mpu6050Test()) // check connection
    {
        DEBUG_PRINTI("\n mpu connection passed\n");
    }
    else
    {
        DEBUG_PRINTI("\n mpu connection failed!!!!!!!!!!!!\n");
    }

    mpu6050Reset(); // reset
    vTaskDelay(100);
    mpu6050SetClockSource(xGyroreference); // set clock source
    mpu6050SetSleepEnabled(0); // wake up

    mpu6050SetFullScaleGyroRange(MPU6050_GYRO_FS_2000); // set gyro range
    mpu6050SetFullScaleAccelRange(MPU6050_ACCEL_FS_16); // set accel range

    mpu6050SetRate(MPU6050_RA_SMPLRT_DIV); // set sample rate

    if(mpu6050SelfTest()) // perform self test
    {
        DEBUG_PRINTI("\n mpu self test passed\n");
    }
    else
    {
        DEBUG_PRINTI("\n mpu self test failed!!!!!!!!!!!!\n");
    }

    

    int16_t *ax, *ay, *az, *gx, *gy, *gz;
    int16_t ax_v, ay_v, az_v, gx_v, gy_v, gz_v;
    ax = &ax_v;
    ay = &ay_v;
    az = &az_v;
    gx = &gx_v;
    gy = &gy_v;
    gz = &gz_v;

    for (int i = 0; i < 1000; i ++)
    {
        mpu6050GetMotion6(ax, ay, az, gx, gy, gz);
        DEBUG_PRINTI("mpu6050GetMotion6 results = %d | %d | %d | %d | %d | %d ", *ax, *ay, *az, *gx, *gy, *gz);
        vTaskDelay(100);
    }



    pwm_timmer_init();
    motorsInit(motorMapDefaultBrushed);

    motorsSetRatio(MOTOR_M1, 50000);
    motorsSetRatio(MOTOR_M2, 50000);
    motorsSetRatio(MOTOR_M3, 50000);
    motorsSetRatio(MOTOR_M4, 50000);
}
