/**
 * @file
 *
 * @brief Generic uC Driver API
 *
 * @ingroup I2CMicroController
 */

#ifndef _CFS_TEST_H
#define _CFS_TEST_H

#include "gen-uC.h"
#include "sensor-mpu6050.h"

#include <stdio.h>
#include <string.h>
#include <bsp.h>

#define TASK_ATTRIBUTES RTEMS_FLOATING_POINT

#define PAYLOAD_BYTES 29

static const char bus_path[] = "/dev/i2c-2";
static const char genuC_path[] = "/dev/i2c-2.genuC-0";
static const char mpu6050_path[] = "/dev/i2c-2.mpu6050-0";

typedef struct
{
    uint8_t AppID;    // 1 byte
    uint8_t CommandErrorCounter;    // 1 byte
    uint8_t CommandCounter;         // 1 byte
    uint8_t spare[2];               // 2 bytes
    float AccelRead[3];           // 1 float = 4 bytes ==> 12 bytes
    float GyroRead[3];            // 1 float = 4 bytes ==> 12 bytes
} ALTITUDE_APP_HkTlm_Payload_t;   // Total = 29 bytes


#endif /* _CFS_TEST_H */
