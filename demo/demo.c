/*
 * Copyright (c) 2022-2023 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>

#include "cmsis_vio.h"
#include "cmsis_os2.h"

#include "sds.h"

#include "sensor_drv.h"
#include "sensor_config.h"

#include <string.h>

#include "./Qeexo/QxAutoMLUser.h"

// Configuration
#ifndef SDS_BUF_SIZE_ACCEL
#define SDS_BUF_SIZE_ACCEL          8192U
#endif
#ifndef SDS_BUF_SIZE_GYRO
#define SDS_BUF_SIZE_GYRO           8192U
#endif
#ifndef SDS_BUF_SIZE_MAGNO
#define SDS_BUF_SIZE_MAGNO          8192U
#endif
#ifndef SDS_THRESHOLD_ACCEL
#define SDS_THRESHOLD_ACCEL         12U
#endif
#ifndef SDS_THRESHOLD_GYRO
#define SDS_THRESHOLD_GYRO          12U
#endif
#ifndef SDS_THRESHOLD_MAGNO
#define SDS_THRESHOLD_MAGNO         12U
#endif

#ifndef SENSOR_POLLING_INTERVAL
#define SENSOR_POLLING_INTERVAL             12U  /* 12ms */
#endif

#ifndef SENSOR_BUF_SIZE
#define SENSOR_BUF_SIZE                     1024U
#endif

// Sensor identifiers
static sensorId_t sensorId_accel             = NULL;
static sensorId_t sensorId_gyro              = NULL;
static sensorId_t sensorId_magno             = NULL;

// Sensor configuration
static sensorConfig_t *sensorConfig_accel    = NULL;
static sensorConfig_t *sensorConfig_gyro     = NULL;
static sensorConfig_t *sensorConfig_magno    = NULL;

// SDS identifiers
static sdsId_t sdsId_accel                   = NULL;
static sdsId_t sdsId_gyro                    = NULL;
static sdsId_t sdsId_magno                   = NULL;

// SDS buffers
static uint8_t sdsBuf_accel[SDS_BUF_SIZE_ACCEL];
static uint8_t sdsBuf_gyro[SDS_BUF_SIZE_GYRO];
static uint8_t sdsBuf_magno[SDS_BUF_SIZE_MAGNO];

// Temporary sensor buffer
static uint8_t sensorBuf[SENSOR_BUF_SIZE];

// Structured Sensor Buffer Data
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;

} SENSOR_AxesRaw_t;

static uint8_t sensorAxesBuf[SENSOR_BUF_SIZE];

// Sensor close flag
static uint8_t close_flag = 0U;

// Thread identifiers
static osThreadId_t thrId_demo           = NULL;
static osThreadId_t thrId_read_sensors   = NULL;

#define EVENT_DATA_ACCEL       (1U << 0)
#define EVENT_DATA_GYRO        (1U << 1)
#define EVENT_DATA_MAGNO       (1U << 2)
#define EVENT_BUTTON           (1U << 3)
#define EVENT_DATA_MASK        (EVENT_DATA_ACCEL | EVENT_DATA_GYRO | EVENT_DATA_MAGNO)
#define EVENT_MASK             (EVENT_DATA_MASK | EVENT_BUTTON)

#define EVENT_CLOSE                     (1U << 0)


// Read sensor thread
static __NO_RETURN void read_sensors (void *argument) {
  uint32_t num, buf_size;
  uint32_t timestamp;
  uint8_t  event_close_sent = 0U;
  (void)   argument;

  timestamp = osKernelGetTickCount();
  for (;;) {

    // Collect sensor data.
    if (sensorGetStatus(sensorId_accel).active != 0U) {
      num = sizeof(sensorBuf) / sensorConfig_accel->sample_size;
      num = sensorReadSamples(sensorId_accel, num, sensorBuf, sizeof(sensorBuf));
      if ((num != 0U) && (num < SENSOR_BUF_SIZE)) {
        memcpy(sensorAxesBuf, sensorBuf, sensorConfig_accel->sample_size * num);
        QxFillSensorData(QXSENSOR_TYPE_ACCEL, sensorAxesBuf, num * sensorConfig_accel->sample_size);
      }
    }

    if (sensorGetStatus(sensorId_gyro).active != 0U) {
      num = sizeof(sensorBuf) / sensorConfig_gyro->sample_size;
      num = sensorReadSamples(sensorId_gyro, num, sensorBuf, sizeof(sensorBuf));
      if ((num != 0U) && (num < SENSOR_BUF_SIZE)) {
        memcpy(sensorAxesBuf, sensorBuf, sensorConfig_gyro->sample_size * num);
        QxFillSensorData(QXSENSOR_TYPE_GYRO, sensorAxesBuf, num * sensorConfig_gyro->sample_size);
      }
    }

    if (sensorGetStatus(sensorId_magno).active != 0U) {
      num = sizeof(sensorBuf) / sensorConfig_magno->sample_size;
      num = sensorReadSamples(sensorId_magno, num, sensorBuf, sizeof(sensorBuf));
      if ((num != 0U) && (num < SENSOR_BUF_SIZE)) {
        memcpy(sensorAxesBuf, sensorBuf, sensorConfig_magno->sample_size * num);
        QxFillSensorData(QXSENSOR_TYPE_MAG, sensorAxesBuf, num * sensorConfig_magno->sample_size);
      }
    }

    timestamp += SENSOR_POLLING_INTERVAL;
    osDelayUntil(timestamp);
  }
}


// SDS event callback
static void sds_event_callback (sdsId_t id, uint32_t event, void *arg) {
  (void)arg;

  if ((event & SDS_EVENT_DATA_HIGH) != 0U) {
    if (id == sdsId_accel) {
      osThreadFlagsSet(thrId_demo, EVENT_DATA_ACCEL);
    }
    if (id == sdsId_gyro) {
      osThreadFlagsSet(thrId_demo, EVENT_DATA_GYRO);
    }
    if (id == sdsId_magno) {
      osThreadFlagsSet(thrId_demo, EVENT_DATA_MAGNO);
    }
  }
}


const char *qx_predict_classes[] = {"REST", "SHAKE", "WAVE"};

// Sensor Demo
void __NO_RETURN demo(void) {
  uint32_t  n, num, flags, buf_size;
  uint32_t  buf[2];
  int16_t *data_i16 = (int16_t *)buf;
  uint32_t timestamp;
  uint32_t cur_ts;
  uint32_t last_pred_ts = 0;
  int16_t prediction = 0;

  thrId_demo = osThreadGetId();

  // Get sensor identifier
  sensorId_accel     = sensorGetId("accel");
  sensorId_gyro     = sensorGetId("gyro");
  sensorId_magno     = sensorGetId("magno");

  // Get sensor configuration
  sensorConfig_accel     = sensorGetConfig(sensorId_accel);
  sensorConfig_gyro     = sensorGetConfig(sensorId_gyro);
  sensorConfig_magno     = sensorGetConfig(sensorId_magno);

  // Open SDS
  sdsId_accel     = sdsOpen(sdsBuf_accel,
                            sizeof(sdsBuf_accel),
                            0U, SDS_THRESHOLD_ACCEL);
  sdsId_gyro      = sdsOpen(sdsBuf_gyro,
                            sizeof(sdsBuf_gyro),
                            0U, SDS_THRESHOLD_GYRO);
  sdsId_magno     = sdsOpen(sdsBuf_magno,
                            sizeof(sdsBuf_magno),
                            0U, SDS_THRESHOLD_MAGNO);

  // Register SDS events
  sdsRegisterEvents(sdsId_accel,     sds_event_callback, SDS_EVENT_DATA_HIGH, NULL);
  sdsRegisterEvents(sdsId_gyro,      sds_event_callback, SDS_EVENT_DATA_HIGH, NULL);
  sdsRegisterEvents(sdsId_magno,     sds_event_callback, SDS_EVENT_DATA_HIGH, NULL);

  // Create sensor thread
  thrId_read_sensors = osThreadNew(read_sensors, NULL, NULL);
  // Give some time for the sensor reading thread to get started.
  timestamp = osKernelGetTickCount();
  last_pred_ts = timestamp;
  timestamp += SENSOR_POLLING_INTERVAL;
  osDelayUntil(timestamp);

  // Get things enabled right at the start, give us some time to start collecting data.
  sdsClear(sdsId_accel);
  sensorEnable(sensorId_accel);
  sdsClear(sdsId_gyro);
  sensorEnable(sensorId_gyro);
  sdsClear(sdsId_magno);
  sensorEnable(sensorId_magno);

  timestamp = osKernelGetTickCount();
  last_pred_ts = timestamp;
  timestamp += SENSOR_POLLING_INTERVAL;
  printf("Waiting until next timestamp to give time to collect samples before starting loop: %d\r\n", timestamp);
  osDelayUntil(timestamp);

  for(;;) {
    timestamp = osKernelGetTickCount();

    // If it has been long enough, call QxClassify
    cur_ts = osKernelGetTickCount();
    if ((cur_ts < 20200) && ((cur_ts - last_pred_ts) >= PRED_CLASSIFICATION_INTERVAL_IN_MSECS)) {
      // Stop doing classification after 20 seconds, that's all of the data we have.
      // See what the result of Qeexo AutoML Model prediction is at the current timestamp
      last_pred_ts = osKernelGetTickCount();
      prediction = QxClassify();
      printf("TS: %d Prediction: %d - %s\r\n", cur_ts, prediction, qx_predict_classes[prediction]);
    }
    timestamp += PRED_CLASSIFICATION_INTERVAL_IN_MSECS;
    osDelayUntil(timestamp);
  }

}
