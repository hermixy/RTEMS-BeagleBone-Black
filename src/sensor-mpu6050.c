/**
 * @file
 *
 * @brief Temperature Sensor mpu6050 Driver Implementation
 *
 * @ingroup I2CSensormpu6050
 */

/*
 * Copyright (c) 2017 embedded brains GmbH.  All rights reserved.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include "sensor-mpu6050.h"

static const char mpu6050_path[] = "/dev/i2c-2.mpu6050-0";

static int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t *val){
  uint8_t out[2] = { ptr, val };
  i2c_msg msgs[1] = {
    {
      .addr = dev->address,
      .flags = 0,
      .len = (uint16_t) sizeof(out),
      .buf = &out[0]
    }
  };

  return i2c_bus_transfer(dev->bus, &msgs[0], RTEMS_ARRAY_SIZE(msgs));
}

#ifdef hechopormi
static int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t * buff){

  printf("Empiezo a leer bytes\n");
  int fd;
  int rv;

  uint8_t tmp;

  uint16_t nr_bytes = 1;
  printf("Los datos para esta op son nr_bytes:%d\n",nr_bytes);
  uint16_t chip_address = (uint16_t) 0x68;
  printf("Los datos para esta op son chip_address:%d\n",chip_address);
  uint8_t data_address = (uint8_t) register_add;
  printf("Los datos para esta op son data_address:%d\n",data_address);

  printf("Los datos para esta op son bus_path:%s\n",&mpu6050_path[0]);



  fd = open(&mpu6050_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus");
    return 1;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, tmp);

  close(fd);

  buff[0] = tmp;

  return rv;
}

static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, int16_t * buff){
  int rv;
  uint8_t value[nr_bytes];
  i2c_msg msgs[] = {{
    .addr = i2c_address,
    .flags = 0,
    .buf = &data_address,
    .len = 1,
  }, {
    .addr = i2c_address,
    .flags = I2C_M_RD,
    .buf = value,
    .len = nr_bytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };
  uint16_t i;

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    printf("ioctl failed");
  } else {
    for (i = 0; i < nr_bytes; ++i) {
      buff[i] = value[i];
    }
    printf("\n");
  }

  return rv;
}
#endif

int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes){
  int rv;
  uint8_t value[nr_bytes];
  i2c_msg msgs[] = {{
    .addr = i2c_address,
    .flags = 0,
    .buf = &data_address,
    .len = 1,
  }, {
    .addr = i2c_address,
    .flags = I2C_M_RD,
    .buf = value,
    .len = nr_bytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };
  uint16_t i;

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    perror("ioctl failed");
  } else {
    for (i = 0; i < nr_bytes; ++i) {
      printf("0x%02x ", value[i]);
    }
    printf("\n");
  }

  return rv;
}

#ifdef reg_16
static int sensor_mpu6050_set_reg_16(i2c_dev *dev, int ptr, uint8_t *val){
  uint8_t out[3] = {(uint8_t) ptr, (uint8_t) (val >> 8), (uint8_t) val };
  i2c_msg msgs[1] = {
    {
      .addr = dev->address,
      .flags = 0,
      .len = (uint16_t) sizeof(out),
      .buf = &out[0]
    }
  };

  return i2c_bus_transfer(dev->bus, &msgs[0], RTEMS_ARRAY_SIZE(msgs));
}

static int sensor_mpu6050_get_reg_16(const char *bus, uint8_t data_address, int16_t * buff){
  int fd;
  int rv;

  uint8_t tmp[2];

  uint16_t nr_bytes = 2;
  uint16_t chip_address = (uint16_t) 0x68;

  data_address = (uint8_t) strtoul(dat_add, NULL, 0);

  fd = open(bus, O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus");
    return 1;
  }

  rv = get_bytes(fd, chip_address, data_address, nr_bytes, tmp);

  buff[0] = tmp[0];
  buff[1] = tmp[1];

  close(fd);

  return rv;
}
#endif

static int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg){
  int err;

  switch (command) {
    case SENSOR_MPU6050_SET_CONF:
      err = sensor_mpu6050_set_reg_8(dev, PWR_MGT_1, 0x08);                     //Temp sensor disabled, internal 8MHz oscillator and cycle disabled
      err = err + sensor_mpu6050_set_reg_8(dev, SIGNAL_PATH_RESET, 0b00000111); //Accelerometer, Gyroscope and Thermometer path reset
      err = err + sensor_mpu6050_set_reg_8(dev, ACCEL_CONFIG, 0b00000001);      //5Hz filter +-2g
      err = err + sensor_mpu6050_set_reg_8(dev, INT_ENABLE, 1<<WOM_EN);         //Disables interrupts in MPU6050
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
}

int sensor_mpu6050_set_conf(int fd){
  return ioctl(fd, SENSOR_MPU6050_SET_CONF, NULL);
}

#ifdef hechopormi
int sensor_mpu6050_get_gyro(int16_t * buff){

	uint8_t tmp[2];
  int err = 0;

	err = sensor_mpu6050_get_gyro_X(tmp);
	buff[0] = (tmp[0]<<8)|(tmp[1]);
	err = err + sensor_mpu6050_get_gyro_Y(tmp);
	buff[1] = (tmp[0]<<8)|(tmp[1]);
	err = err + sensor_mpu6050_get_gyro_Z(tmp);
	buff[2] = (tmp[0]<<8)|(tmp[1]);

  return err;
}

int sensor_mpu6050_get_accel(int16_t * buff){

	uint8_t tmp[2];
  int err = 0;

  printf("Entre a la func 1\n");

	err = sensor_mpu6050_get_accel_X(tmp);
	buff[0] = (tmp[0]<<8)|(tmp[1]);
	err = err + sensor_mpu6050_get_accel_Y(tmp);
	buff[1] = (tmp[0]<<8)|(tmp[1]);
	err = err + sensor_mpu6050_get_accel_Z(tmp);
	buff[2] = (tmp[0]<<8)|(tmp[1]);

  return err;
}

static int sensor_mpu6050_get_gyro_X(uint8_t * buff){
  int err = 0;

	err = sensor_mpu6050_get_reg_8(GYRO_XOUT_H, buff);
	err = err + sensor_mpu6050_get_reg_8(GYRO_XOUT_L, buff+1);

  return err;
}

static int sensor_mpu6050_get_gyro_Y(uint8_t * buff){

  int err = 0;

	err = sensor_mpu6050_get_reg_8(GYRO_YOUT_H, buff);
	err = err + sensor_mpu6050_get_reg_8(GYRO_YOUT_L, buff+1);

  return err;
}

static int sensor_mpu6050_get_gyro_Z(uint8_t * buff){

  int err = 0;

	err = sensor_mpu6050_get_reg_8(GYRO_ZOUT_H, buff);
	err = err + sensor_mpu6050_get_reg_8(GYRO_ZOUT_L, buff+1);

  return err;
}

static int sensor_mpu6050_get_accel_X(uint8_t * buff){

  int err = 0;

  printf("Entre a la func 2\n");

	err = sensor_mpu6050_get_reg_8(ACCEL_XOUT_H, buff);
  printf("Pase a  2_2\n");
	err = err + sensor_mpu6050_get_reg_8(ACCEL_XOUT_L, buff+1);

  return err;
}

static int sensor_mpu6050_get_accel_Y(uint8_t * buff){

  int err = 0;

  printf("Entre a la func 3\n");

	err = sensor_mpu6050_get_reg_8(ACCEL_YOUT_H, buff);
  printf("Pase a  3_2\n");
	err = err + sensor_mpu6050_get_reg_8(ACCEL_YOUT_L, buff+1);

  return err;
}

static int sensor_mpu6050_get_accel_Z(uint8_t * buff){

  int err = 0;

  printf("Entre a la func 4\n");

	err = sensor_mpu6050_get_reg_8(ACCEL_ZOUT_H, buff);

  printf("Pase a  4_2\n");
	err = err + sensor_mpu6050_get_reg_8(ACCEL_ZOUT_L, buff+1);

  return err;
}

#endif


int i2c_dev_register_sensor_mpu6050(const char *bus_path, const char *dev_path){
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, MPU6050_ADDRESS);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = sensor_mpu6050_ioctl;

  return i2c_dev_register(dev, dev_path);
}
