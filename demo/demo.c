/*
 * Copyright (c) 2022 Arm Limited. All rights reserved.
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
#define SDS_THRESHOLD_ACCEL         6U
#endif
#ifndef SDS_THRESHOLD_GYRO
#define SDS_THRESHOLD_GYRO          6U
#endif
#ifndef SDS_THRESHOLD_MAGNO
#define SDS_THRESHOLD_MAGNO         6U
#endif

#ifndef SENSOR_POLLING_INTERVAL
#define SENSOR_POLLING_INTERVAL             1U  /* 1ms */
#endif

#ifndef SENSOR_BUF_SIZE
#define SENSOR_BUF_SIZE                     6U
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

// Thread identifiers
static osThreadId_t thrId_demo         = NULL;
static osThreadId_t thrId_read_sensors = NULL;

#define EVENT_DATA_ACCEL       (1U << 0)
#define EVENT_DATA_GYRO        (1U << 1)
#define EVENT_DATA_MAGNO       (1U << 2)
#define EVENT_BUTTON           (1U << 3)
#define EVENT_DATA_MASK        (EVENT_DATA_ACCEL | EVENT_DATA_GYRO | EVENT_DATA_MAGNO)
#define EVENT_MASK             (EVENT_DATA_MASK | EVENT_BUTTON)

// Read sensor thread
static __NO_RETURN void read_sensors (void *argument) {
  uint32_t num, buf_size;
  uint32_t timestamp;
  (void)   argument;

  timestamp = osKernelGetTickCount();
  for (;;) {
    if (sensorGetStatus(sensorId_accel).active != 0U) {
      num = sizeof(sensorBuf) / sensorConfig_accel->sample_size;
      //printf("%s: Got initial num %d to pass to: sensorReadSamples(%d, %d, %s, %d)\r\n", sensorConfig_accel->name, num, (int)sensorId_accel, num, sensorBuf, sizeof(sensorBuf));
      num = sensorReadSamples(sensorId_accel, num, sensorBuf, sizeof(sensorBuf));
      //printf("%s: sensorReadSamples return num: %d\r\n", sensorConfig_accel->name, num);
      if (num != 0U) {
        buf_size = num * sensorConfig_accel->sample_size;
        //printf("%s: About to call sdsWrite(%d, %s, %d) with buf_size=%d from num=%d * sample_size=%d\r\n", sensorConfig_accel->name, (int)sensorId_accel, sensorBuf, buf_size, buf_size, num, sensorConfig_accel->sample_size);
        num = sdsWrite(sdsId_accel, sensorBuf, buf_size);
        if (num != buf_size) {
          printf("%s: SDS write failed buf_size=%d, num=%d\r\n", sensorConfig_accel->name, buf_size, num);
        }
      }
    }

    if (sensorGetStatus(sensorId_gyro).active != 0U) {
      num = sizeof(sensorBuf) / sensorConfig_gyro->sample_size;
      num = sensorReadSamples(sensorId_gyro, num, sensorBuf, sizeof(sensorBuf));
      if (num != 0U) {
        buf_size = num * sensorConfig_gyro->sample_size;
        num = sdsWrite(sdsId_gyro, sensorBuf, buf_size);
        if (num != buf_size) {
          printf("%s: SDS write failed buf_size=%d num=%d\r\n", sensorConfig_gyro->name, buf_size, num);
        }
      }
    }

    if (sensorGetStatus(sensorId_magno).active != 0U) {
      num = sizeof(sensorBuf) / sensorConfig_magno->sample_size;
      num = sensorReadSamples(sensorId_magno, num, sensorBuf, sizeof(sensorBuf));
      if (num != 0U) {
        buf_size = num * sensorConfig_magno->sample_size;
        num = sdsWrite(sdsId_magno, sensorBuf, buf_size);
        if (num != buf_size) {
          printf("%s: SDS write failed buf_size=%d num=%d\r\n", sensorConfig_magno->name, buf_size, num);
        }
      }
    }

    timestamp += SENSOR_POLLING_INTERVAL;
    osDelayUntil(timestamp);
  }
}

// Button thread
static __NO_RETURN void button (void *argument) {
  uint32_t value, value_last = 0U;
  (void)   argument;

  for (;;) {
    // Monitor user button
    value = vioGetSignal(vioBUTTON0);
    if (value != value_last) {
      value_last = value;
      if (value == vioBUTTON0) {
        // Button pressed
        osThreadFlagsSet(thrId_demo, EVENT_BUTTON);
      }
    }
    osDelay(100U);
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

// Sensor Demo
void __NO_RETURN demo(void) {
  uint32_t  n, num, flags;
  uint32_t  buf[2];
  int16_t *data_i16 = (int16_t *)buf;


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

  // Create button thread
  osThreadNew(button, NULL, NULL);

  for(;;) {
    flags = osThreadFlagsWait(EVENT_MASK, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) == 0U) {

      // Button pressed event
      if (flags & EVENT_BUTTON) {
        printf("Button pressed\r\n");

        if (sensorGetStatus(sensorId_accel).active == 0U) {
          sdsClear(sdsId_accel);
          sensorEnable(sensorId_accel);
          printf("accel enabled\r\n");
        } else {
          sensorDisable(sensorId_accel);
          printf("accel disabled\r\n");
        }

        if (sensorGetStatus(sensorId_gyro).active == 0U) {
          sdsClear(sdsId_gyro);
          sensorEnable(sensorId_gyro);
          printf("gyro enabled\r\n");
        } else {
          sensorDisable(sensorId_gyro);
          printf("gyro disabled\r\n");
        }

        if (sensorGetStatus(sensorId_magno).active == 0U) {
          sdsClear(sdsId_magno);
          sensorEnable(sensorId_magno);
          printf("magno enabled\r\n");
        } else {
          sensorDisable(sensorId_magno);
          printf("magno disabled\r\n");
        }

      }

      // Accelerometer data event
      if ((flags & EVENT_DATA_ACCEL) != 0U) {

        for (n = 0U; n < (SDS_THRESHOLD_ACCEL / sensorConfig_accel->sample_size); n++) {
          num = sdsRead(sdsId_accel, buf, sensorConfig_accel->sample_size);
          if (num == sensorConfig_accel->sample_size) {
            printf("%s: x=%i, y=%i, z=%i\r\n",sensorConfig_accel->name,
                                              data_i16[0], data_i16[1], data_i16[2]);
          }
        }
      }

      // Gyro data event
      if ((flags & EVENT_DATA_GYRO) != 0U) {
        for (n = 0U; n < (SDS_THRESHOLD_GYRO / sensorConfig_gyro->sample_size); n++) {
          num = sdsRead(sdsId_gyro, buf, sensorConfig_gyro->sample_size);
          if (num == sensorConfig_gyro->sample_size) {
            printf("%s: x=%i, y=%i, z=%i\r\n",sensorConfig_gyro->name,
                                              data_i16[0], data_i16[1], data_i16[2]);
          }
        }
      }
      // Magno data event.
      if ((flags & EVENT_DATA_MAGNO) != 0U) {
        for (n = 0U; n < (SDS_THRESHOLD_MAGNO / sensorConfig_magno->sample_size); n++) {
          num = sdsRead(sdsId_magno, buf, sensorConfig_magno->sample_size);
          if (num == sensorConfig_magno->sample_size) {
            printf("%s: x=%i, y=%i, z=%i\r\n",sensorConfig_magno->name,
                                              data_i16[0], data_i16[1], data_i16[2]);
          }
        }
      }

    }
  }
}
