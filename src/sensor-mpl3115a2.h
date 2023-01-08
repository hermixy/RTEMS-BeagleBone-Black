/**
 * @file
 *
 * @brief Pressure Sensor MPL3115A2 Driver API
 *
 * @ingroup I2CSensorMPL3115A2
 */


#ifndef _DEV_I2C_SENSOR_MPL3115A2_H
#define _DEV_I2C_SENSOR_MPL3115A2_H

#include <dev/i2c/i2c.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Device address
#define MPL3115A2_ADDRESS               0x60

/** MPL3115A2 registers **/
#define MPL3115A2_REGISTER_STATUS       0x00

#define MPL3115A2_REGISTER_PRESSURE_MSB 0x01
#define MPL3115A2_REGISTER_PRESSURE_CSB 0x02
#define MPL3115A2_REGISTER_PRESSURE_LSB 0x03
#define MPL3115A2_REGISTER_TEMP_MSB     0x04
#define MPL3115A2_REGISTER_TEMP_LSB     0x05
#define MPL3115A2_REGISTER_DR_STATUS    0x06
#define MPL3115A2_OUT_P_DELTA_MSB       0x07
#define MPL3115A2_OUT_P_DELTA_CSB       0x08
#define MPL3115A2_OUT_P_DELTA_LSB       0x09
#define MPL3115A2_OUT_T_DELTA_MSB       0x0A
#define MPL3115A2_OUT_T_DELTA_LSB       0x0B
#define MPL3115A2_WHOAMI                0x0C
#define MPL3115A2_BAR_IN_MSB            0x14
#define MPL3115A2_BAR_IN_LSB            0x15
#define MPL3115A2_OFF_H                 0x2D

/** MPL3115A2 status register bits **/
#define MPL3115A2_REGISTER_STATUS_TDR   0x02
#define MPL3115A2_REGISTER_STATUS_PDR   0x04
#define MPL3115A2_REGISTER_STATUS_PTDR  0x08

/** MPL3115A2 PT DATA register bits **/
#define MPL3115A2_PT_DATA_CFG           0x13
#define MPL3115A2_PT_DATA_CFG_TDEFE     0x01
#define MPL3115A2_PT_DATA_CFG_PDEFE     0x02
#define MPL3115A2_PT_DATA_CFG_DREM      0x04

/** MPL3115A2 control registers **/
#define MPL3115A2_CTRL_REG1             0x26
#define MPL3115A2_CTRL_REG2             0x27
#define MPL3115A2_CTRL_REG3             0x28
#define MPL3115A2_CTRL_REG4             0x29
#define MPL3115A2_CTRL_REG5             0x2A

/** MPL3115A2 control register bits **/
#define MPL3115A2_CTRL_REG1_SBYB        0x01
#define MPL3115A2_CTRL_REG1_OST         0x02
#define MPL3115A2_CTRL_REG1_RST         0x04
#define MPL3115A2_CTRL_REG1_RAW         0x40
#define MPL3115A2_CTRL_REG1_ALT         0x80
#define MPL3115A2_CTRL_REG1_BAR         0x00

/** MPL3115A2 oversample values **/
#define MPL3115A2_CTRL_REG1_OS1         0x00
#define MPL3115A2_CTRL_REG1_OS2         0x08
#define MPL3115A2_CTRL_REG1_OS4         0x10
#define MPL3115A2_CTRL_REG1_OS8         0x18
#define MPL3115A2_CTRL_REG1_OS16        0x20
#define MPL3115A2_CTRL_REG1_OS32        0x28
#define MPL3115A2_CTRL_REG1_OS64        0x30
#define MPL3115A2_CTRL_REG1_OS128       0x38

/** MPL3115A2 measurement modes **/
typedef enum {
  MPL3115A2_BAROMETER = 0,
  MPL3115A2_ALTIMETER,
} mpl3115a2_mode_t;

/** MPL3115A2 measurement types **/
typedef enum {
  MPL3115A2_PRESSURE,
  MPL3115A2_ALTITUDE,
  MPL3115A2_TEMPERATURE,
} mpl3115a2_meas_t;

#define MPL3115A2_REGISTER_STARTCONVERSION (0x12)

/**
 * @defgroup I2CSensorMPL3115A2 Pressure Sensor MPL3115A2 Driver
 *
 * @ingroup I2CDevice
 *
 * @brief Driver for MPL3115A2 Pressure sensor.
 *
 * @{
 */

#define delay_ms(m) rtems_task_wake_after(1 + ((m)/rtems_configuration_get_milliseconds_per_tick()))

typedef struct {
  mpl3115a2_mode_t currentMode;
  uint8_t rawData[5];
  union {
      struct {
        uint8_t SBYB : 1;
        uint8_t OST : 1;
        uint8_t RST : 1;
        uint8_t OS : 3;
        uint8_t RAW : 1;
        uint8_t ALT : 1;
      } bit;
      uint8_t reg;
  } ctrl_reg1;
}MPL3115A2_Data_t;


#define pressure_read
#define temperature_read

typedef enum {
  SENSOR_MPL3115A2_BEGIN
} sensor_mpl3115a2_command;

int i2c_dev_register_sensor_mpl3115a2(const char *bus_path, const char *dev_path);
int sensor_mpl3115a2_begin(int fd);


// I2C functions
#ifdef pressure_read
  float sensor_mpl3115a2_getPressure(void);
  float sensor_mpl3115a2_getAltitude(void);
  int8_t sensor_mpl3115a2_getAltitudeOffset(void);
  void sensor_mpl3115a2_setAltitudeOffset(int8_t offset);
  void sensor_mpl3115a2_setSeaPressure(float SLP);
#endif

#ifdef temperature_read
  float sensor_mpl3115a2_getTemperature(void);
#endif

void sensor_mpl3115a2_setMode(mpl3115a2_mode_t mode);
void sensor_mpl3115a2_startOneShot(void);
bool sensor_mpl3115a2_conversionComplete(void);
float sensor_mpl3115a2_getLastConversionResults(mpl3115a2_meas_t value);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_I2C_SENSOR_MPL3115A2_H */
