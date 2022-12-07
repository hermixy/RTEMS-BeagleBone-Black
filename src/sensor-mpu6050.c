/**
 * @file
 *
 * @brief Accelerometer Sensor MPU6050 Driver Implementation
 *
 * @ingroup I2CSensorMPU6050
 */

#include "sensor-mpu6050.h"

static const char bus_path[] = "/dev/i2c-2";

static int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);
static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);

static int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val);
static int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff);

static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff){
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
    printf("ioctl failed...\n");
  } else {

    free(*buff);
    *buff = malloc(nr_bytes * sizeof(uint8_t));

    for (i = 0; i < nr_bytes; ++i) {
      (*buff)[i] = value[i];
    }
  }

  return rv;
}

static int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val){
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

static int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff){

  int fd;
  int rv;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(1 * sizeof(uint8_t));

  uint16_t nr_bytes = (uint16_t) 1;
  uint16_t chip_address = (uint16_t) 0x68;
  uint8_t data_address = (uint8_t) register_add;

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  (*buff)[0] = *tmp;
  free(tmp);

  return rv;
}

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

int i2c_dev_register_sensor_mpu6050(const char *bus_path, const char *dev_path){
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, MPU6050_ADDRESS);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = sensor_mpu6050_ioctl;

  return i2c_dev_register(dev, dev_path);
}

int sensor_mpu6050_set_conf(int fd){
  return ioctl(fd, SENSOR_MPU6050_SET_CONF, NULL);
}

#ifdef gyroscope_read

static int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis);

int sensor_mpu6050_get_gyro(int16_t **buff){

	uint8_t *tmp;
  tmp = NULL;

  int err = 0;

  free(*buff);
  *buff = malloc(3 * sizeof(uint8_t));

  tmp = NULL;
	err = sensor_mpu6050_get_gyro_axis(&tmp, X);
	(*buff)[0] = (tmp[0]<<8)|(tmp[1]);

  tmp = NULL;
	err = err + sensor_mpu6050_get_gyro_axis(&tmp, Y);
	(*buff)[1] = (tmp[0]<<8)|(tmp[1]);

  tmp = NULL;
	err = err + sensor_mpu6050_get_gyro_axis(&tmp, Z);
	(*buff)[2] = (tmp[0]<<8)|(tmp[1]);

  free(tmp);

  if(err != 0)
    printf("There was an error when reading gyroscope registers...\n");

  return err;
}

static int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis){

  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(2 * sizeof(uint8_t));

  switch (axis) {
    case X:
      err = sensor_mpu6050_get_reg_8(GYRO_XOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_XOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Y:
      err = sensor_mpu6050_get_reg_8(GYRO_YOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_YOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Z:
      err = sensor_mpu6050_get_reg_8(GYRO_ZOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_ZOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    default:
      err = -1;
  }

  return err;
}
#endif

#ifdef accelerometer_read

static int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis);

int sensor_mpu6050_get_accel(int16_t **buff){

	uint8_t *tmp;
  tmp = NULL;

  int err = 0;

  free(*buff);
  *buff = malloc(3 * sizeof(uint8_t));

  tmp = NULL;
	err = sensor_mpu6050_get_accel_axis(&tmp, X);
	(*buff)[0] = (tmp[0]<<8)|(tmp[1]);

  tmp = NULL;
	err = err + sensor_mpu6050_get_accel_axis(&tmp, Y);
	(*buff)[1] = (tmp[0]<<8)|(tmp[1]);

  tmp = NULL;
	err = err + sensor_mpu6050_get_accel_axis(&tmp, Z);
	(*buff)[2] = (tmp[0]<<8)|(tmp[1]);

  free(tmp);

  if(err != 0)
    printf("There was an error when reading accelerometer registers...\n");

  return err;
}

static int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis){

  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(2 * sizeof(uint8_t));

  switch (axis) {
    case X:
      err = sensor_mpu6050_get_reg_8(ACCEL_XOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_XOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Y:
      err = sensor_mpu6050_get_reg_8(ACCEL_YOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_YOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Z:
      err = sensor_mpu6050_get_reg_8(ACCEL_ZOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_ZOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    default:
      err = -1;
  }

  return err;
}
#endif
