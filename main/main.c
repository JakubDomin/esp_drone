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
#include "attitude_controller.h"
#include "power_distribution.h"
#include "controller_pid.h"
#include "ms5611.h"
#include "filter.h"
#include "static_mem.h"

#include <stdbool.h>
#include <inttypes.h>
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "version.h"
#include "config.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "adc_esp32.h"
#include "pm_esplane.h"
#include "config.h"
#include "system.h"
#include "platform.h"
#include "configblock.h"
#include "worker.h"
#include "freeRTOSdebug.h"
#include "wifi_esp32.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "console.h"
#include "wifilink.h"
#include "mem.h"
#include "queuemonitor.h"
#include "buzzer.h"
#include "sound.h"
#include "sysload.h"
#include "estimator_kalman.h"
#include "app.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "SYS"
#include "debug_cf.h"
#include "static_mem.h"
#include "cfassert.h"



#define PROXIMITY_ENABLED


// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000
#define SENSORS_NBR_OF_BIAS_SAMPLES  512
#define GYRO_NBR_OF_AXES                3
#define GYRO_VARIANCE_BASE              10000
#define GYRO_VARIANCE_THRESHOLD_X       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z       (GYRO_VARIANCE_BASE)
#define GYRO_MIN_BIAS_TIMEOUT_MS        M2T(1*1000)
#define MPU6050_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)
#define SENSORS_DEG_PER_LSB_CFG MPU6050_DEG_PER_LSB_2000
#define xGyroreference  1
#define bitsToDeg2000   16.4    
#define bitsToG16       2048   

typedef struct
{
  Axis3f     bias;
  Axis3f     variance;
  Axis3f     mean;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static void sensorsBiasObjInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}

static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

  meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}


/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool sensorsFindBiasValue(BiasObj* bias)
{
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled)
  {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

    if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = bias->mean.x;
      bias->bias.y = bias->mean.y;
      bias->bias.z = bias->mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}


static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPower;

int16_t rollOutput, pitchOutput, yawOutput;

///           TASKS



static void main_loop_task(void* param) {

  int16_t *ax, *ay, *az, *g_pitch, *g_roll, *g_yaw;
  int16_t ax_v, ay_v, az_v, gx_v, gy_v, gz_v;
  static float pitchAngle, rollAngle, yawAngle;
  static int32_t previousTime, currentTime, elapsedTime;
  ax = &ax_v;
  ay = &ay_v;
  az = &az_v;
  g_pitch = &gx_v;
  g_roll = &gy_v;
  g_yaw = &gz_v;

  static control_t control = {.pitch = 0, .roll = 0, .yaw = 0, .thrust = 20000};
  attitude_t rateDesired = {.pitch = 0, .roll = 0, .yaw = 0, .timestamp = 0};
  attitude_t attitudeDesired = rateDesired;
  setpoint_t setPoint = {.mode.yaw = modeVelocity, .mode.x = modeDisable, .mode.y = modeDisable,
                          .mode.roll = modeVelocity, .mode.pitch = modeVelocity,
                          .attitudeRate = rateDesired};
  attitude_t EulerrateDesired = {.pitch = 0, .roll = 0, .yaw = 0, .timestamp = 0};
  sensorData_t sensors;
  state_t state;
  BiasObj bias;
  int16_t r, p;

  while(1)
  {
    mpu6050GetMotion6(ax, ay, az, g_roll, g_pitch, g_yaw); // chyba git
    sensors.acc.x = *ax / bitsToG16;
    sensors.acc.y = *ay / bitsToG16;
    sensors.acc.z = *az / bitsToG16;
    sensors.gyro.x = *g_pitch / bitsToDeg2000;
    sensors.gyro.y = *g_roll / bitsToDeg2000;
    sensors.gyro.z = *g_yaw / bitsToDeg2000;
    // measure time for gyroscope deg/s to deg conversion
    previousTime = currentTime;
    currentTime = T2M(xTaskGetTickCount()); // get current time in miliseconds
    elapsedTime = (currentTime - previousTime) / 1000; // get elapsed time in seconds

    // gyroscope deg/s to deg conversion
    pitchAngle = pitchAngle + sensors.gyro.x * elapsedTime;
    rollAngle = rollAngle + sensors.gyro.y * elapsedTime;
    yawAngle = yawAngle + sensors.gyro.z * elapsedTime;

    // correct gyro error
    // sensors.gyro.x = (sensors.gyro.x - bias.bias.x) * SENSORS_DEG_PER_LSB_CFG;
    // sensors.gyro.y = (sensors.gyro.y - bias.bias.y) * SENSORS_DEG_PER_LSB_CFG;
    // sensors.gyro.z = (sensors.gyro.z - bias.bias.z) * SENSORS_DEG_PER_LSB_CFG;
    // applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensors.gyro); // LPF Filter, to avoid high-frequency interference

    state.attitude.pitch = sensors.gyro.x;
    state.attitude.roll = sensors.gyro.y;
    state.attitude.yaw = sensors.gyro.z;

    // float yaw = atan2f(2*(this->q[1]*this->q[2]+this->q[0]*this->q[3]) , this->q[0]*this->q[0] + this->q[1]*this->q[1] - this->q[2]*this->q[2] - this->q[3]*this->q[3]);
    // float pitch = asinf(-2*(this->q[1]*this->q[3] - this->q[0]*this->q[2]));
    // float roll = atan2f(2*(this->q[2]*this->q[3]+this->q[0]*this->q[1]) , this->q[0]*this->q[0] - this->q[1]*this->q[1] - this->q[2]*this->q[2] + this->q[3]*this->q[3]);

    DEBUG_PRINTI("mpu6050GetMotion6 results = accel_x = %f | accel_y = %f | accel_z = %f | pitch = %f | roll = %f | yaw = %f ___> elapsedTime = %ld\n",
                  sensors.acc.x, sensors.acc.y, sensors.acc.z, pitchAngle, rollAngle, yawAngle, elapsedTime);

    // attitudeControllerCorrectAttitudePID(state.attitude.roll, state.attitude.pitch, state.attitude.yaw,
    //                                      attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
    //                                      &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    attitudeControllerCorrectRatePID(*g_pitch, -(*g_roll), *g_yaw,
                                      rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control.roll,
                                        &control.pitch,
                                        &control.yaw);

    // controllerPid(&control, &setPoint, &sensors, &state, 10);

    // control.pitch = rateDesired.pitch;
    // control.roll = rateDesired.roll;
    // control.yaw = rateDesired.yaw;

    r = control.roll / 2.0f;
    p = control.pitch / 2.0f;

    motorPower.m1 = (control.thrust - r + p + control.yaw);
    motorPower.m2 = (control.thrust - r - p - control.yaw);
    motorPower.m3 = (control.thrust + r - p + control.yaw);
    motorPower.m4 = (control.thrust + r + p - control.yaw);
    DEBUG_PRINTI("rateDesired.roll = %f ||||| rateDesired.pitch = %f ||||| rateDesired.yaw = %f \n", rateDesired.roll, rateDesired.pitch, rateDesired.yaw);
    DEBUG_PRINTI("rollOutput = %d }}}} pitchOutput = %d }}}} yawOutput = %d \n", rollOutput, pitchOutput, yawOutput);
    DEBUG_PRINTI("motorPower_1 = %d || motorPower_2 = %d || motorPower_3 = %d || motorPower_4 = %d \n\n", motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4);

    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);

    vTaskDelay(300);
    
  }
}


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

    // if(ms5611Init(&sensorsBus)) // initialize sensor
    // {
    //     DEBUG_PRINTI("\n ms5611 initialization passed\n");
    // }
    // else
    // {
    //     DEBUG_PRINTI("\n ms5611 initialization failed!!!!!!!!!!!!\n");
    // }

    // if(ms5611SelfTest()) // perform self test
    // {
    //     DEBUG_PRINTI("\n ms5611 self test passed\n");
    // }
    // else
    // {
    //     DEBUG_PRINTI("\n ms5611 self test failed!!!!!!!!!!!!\n");
    // }

    pwm_timmer_init();
    motorsInit(motorMapDefaultBrushed);



    controllerPidInit();
    if(controllerPidTest())
    {
        DEBUG_PRINTI("\npid initialized\n");
    }
    else
    {
        DEBUG_PRINTI("\npid initialization failed !!!!\n");
    }

    // // Init second order filer for accelerometer and gyro
    // for (uint8_t i = 0; i < 3; i++)
    // {
    //     lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    //     lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
    // }

    // calibrate gyro
    // sensorsBiasObjInit(&bias);
    // for (int i = 0; i < 1030; i++)
    // {
    //     mpu6050GetRotation(g_roll, g_pitch, g_yaw);
    //     sensorsAddBiasValue(&bias, *g_roll, *g_pitch, *g_yaw);
    // }
    // sensorsFindBiasValue(&bias);

    //STATIC_MEM_TASK_CREATE(main_loop_task, main_loop_task, "MAIN_LOOP_TASK", NULL, 2);
    //xTaskCreate(main_loop_task, "main_loop_task", 2000, NULL, 2, NULL);
    StackType_t xStack[2000];
    StaticTask_t xTaskBuffer;
    xTaskCreateStatic(main_loop_task, "main_loop_task", 2000, (void *) 1, 2, xStack, &xTaskBuffer);

    vTaskStartScheduler();
    while (1)
    {
    }
}
