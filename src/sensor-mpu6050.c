/**
 * @file
 *
 * @brief IMU Sensor MPU6050 Driver Implementation
 *
 * @ingroup I2CSensorMPU6050
 */

#include "sensor-mpu6050.h"

static const char bus_path[] = "/dev/i2c-2";

SENSOR_MPU6050_Priv_Data_t SENSOR_MPU6050_Priv_Data;

/*
* PROTOTYPES
*/
static int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);
static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);

static int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val);
static int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff);

static int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis);
static int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis);

static float wrap(float angle,float limit);
static void setGyroOffsets(float x, float y, float z);
static void setAccOffsets(float x, float y, float z);
static void setFilterGyroCoef(float gyro_coeff);

static int sensor_mpu6050_update_accel(void);

static int sensor_mpu6050_update_gyro(void);

static int sensor_mpu6050_update_temp(void);

/*
* STATIC FUNCTIONS
*/

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

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
    case SENSOR_MPU6050_BEGIN:
      setFilterGyroCoef(DEFAULT_GYRO_COEFF);
      setGyroOffsets(0,0,0);
      setAccOffsets(0,0,0);

      // Sanity check
      uint8_t *whoami;
      whoami = NULL;
      sensor_mpu6050_get_reg_8(MPU6050_WHOAMI, &whoami);
      if ((*whoami) != 0x68) {
        err = -ENOTTY;
        break;
      }

      // Sensor configuration
      err =       sensor_mpu6050_set_reg_8(dev, PWR_MGT_1, 0x00);               //Temp sensor enabled, internal 8MHz oscillator and cycle disabled
      err = err + sensor_mpu6050_set_reg_8(dev, SIGNAL_PATH_RESET, 0b00000111); //Accelerometer, Gyroscope and Thermometer path reset
      err = err + sensor_mpu6050_set_reg_8(dev, ACCEL_CONFIG, 0b00000001);      //5Hz filter +-2g
      err = err + sensor_mpu6050_set_reg_8(dev, GYRO_CONFIG, 0b00000000);       //+-250 deg/s
      err = err + sensor_mpu6050_set_reg_8(dev, INT_ENABLE, 1<<WOM_EN);         //Disables interrupts in MPU6050

      sensor_mpu6050_read_data();
      SENSOR_MPU6050_Priv_Data.angleX = SENSOR_MPU6050_Priv_Data.angleAccX;
      SENSOR_MPU6050_Priv_Data.angleY = SENSOR_MPU6050_Priv_Data.angleAccY;
      SENSOR_MPU6050_Priv_Data.preInterval = runtime_ms(); // may cause lack of angular accuracy if begin() is much before the first update()
      break;

    case SENSOR_MPU6050_SET_REG:
      // err = sensor_mpu6050_set_reg_8(dev, RegisterPtr, DataVal);
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
}

static void setGyroOffsets(float x, float y, float z){
  SENSOR_MPU6050_Priv_Data.gyroXoffset = x;
  SENSOR_MPU6050_Priv_Data.gyroYoffset = y;
  SENSOR_MPU6050_Priv_Data.gyroZoffset = z;
}

static void setAccOffsets(float x, float y, float z){
  SENSOR_MPU6050_Priv_Data.accXoffset = x;
  SENSOR_MPU6050_Priv_Data.accYoffset = y;
  SENSOR_MPU6050_Priv_Data.accZoffset = z;
}

static void setFilterGyroCoef(float gyro_coeff){
  if ((gyro_coeff<0) || (gyro_coeff>1)){ gyro_coeff = DEFAULT_GYRO_COEFF; } // prevent bad gyro coeff, should throw an error...
  SENSOR_MPU6050_Priv_Data.filterGyroCoef = gyro_coeff;
}

static int sensor_mpu6050_update_gyro(void){
  int16_t var;

	uint8_t *tmp;
  tmp = NULL;

  int err = 0;

  tmp = NULL;
	err = sensor_mpu6050_get_gyro_axis(&tmp, X);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.gyroX = (float)(var * (250.0/32768.0)) - SENSOR_MPU6050_Priv_Data.gyroXoffset;

  tmp = NULL;
	err = err + sensor_mpu6050_get_gyro_axis(&tmp, Y);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.gyroY = (float)(var * (250.0/32768.0)) - SENSOR_MPU6050_Priv_Data.gyroYoffset;

  tmp = NULL;
	err = err + sensor_mpu6050_get_gyro_axis(&tmp, Z);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.gyroZ = (float)(var * (250.0/32768.0)) - SENSOR_MPU6050_Priv_Data.gyroZoffset;

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

static int sensor_mpu6050_update_accel(void){

  int16_t var;

	uint8_t *tmp;
  tmp = NULL;

  int err = 0;

  tmp = NULL;
	err = sensor_mpu6050_get_accel_axis(&tmp, X);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.accX = (float)(var * (9.81/16384.0)) - SENSOR_MPU6050_Priv_Data.accXoffset;

  tmp = NULL;
	err = err + sensor_mpu6050_get_accel_axis(&tmp, Y);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.accY = (float)(var * (9.81/16384.0)) - SENSOR_MPU6050_Priv_Data.accYoffset;

  tmp = NULL;
	err = err + sensor_mpu6050_get_accel_axis(&tmp, Z);
  var = (tmp[0]<<8)|(tmp[1]);
	SENSOR_MPU6050_Priv_Data.accZ = (float)(var * (9.81/16384.0)) - SENSOR_MPU6050_Priv_Data.accZoffset;

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

static int sensor_mpu6050_update_temp(void){

  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  uint8_t raw_data[2];

  err = sensor_mpu6050_get_reg_8(TEMP_OUT_H, &tmp);
  raw_data[0] = (*tmp);
  tmp = NULL;
  err = err + sensor_mpu6050_get_reg_8(TEMP_OUT_L, &tmp);
  raw_data[1] = (*tmp);

  free(tmp);

  if(err != 0){
    printf("There was an error when reading temperature registers...\n");
    return -1;
  }

  int16_t raw_temp = raw_data[0] << 8 | raw_data[1];

  SENSOR_MPU6050_Priv_Data.temp = ((float)raw_temp / 340.0) + 36.53;

  return err;

}

/*
* PUBLIC FUNCTIONS
*/

int i2c_dev_register_sensor_mpu6050(const char *bus_path, const char *dev_path){
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, MPU6050_ADDRESS);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = sensor_mpu6050_ioctl;

  return i2c_dev_register(dev, dev_path);
}

int sensor_mpu6050_begin(int fd){
  return ioctl(fd, SENSOR_MPU6050_BEGIN, NULL);
}

int sensor_mpu6050_set_register(int fd){
  return ioctl(fd, SENSOR_MPU6050_SET_REG, NULL);
}

void sensor_mpu6050_calcOffsets(void){
  setGyroOffsets(0,0,0);
  setAccOffsets(0,0,0);
  float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro

  for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
    sensor_mpu6050_update_accel();
    sensor_mpu6050_update_gyro();
    sensor_mpu6050_update_temp();
  	ag[0] += SENSOR_MPU6050_Priv_Data.accX;
  	ag[1] += SENSOR_MPU6050_Priv_Data.accY;
  	ag[2] += (SENSOR_MPU6050_Priv_Data.accZ-1.0);
  	ag[3] += SENSOR_MPU6050_Priv_Data.gyroX;
  	ag[4] += SENSOR_MPU6050_Priv_Data.gyroY;
  	ag[5] += SENSOR_MPU6050_Priv_Data.gyroZ;
  	delay_ms(1); // wait a little bit between 2 measurements
  }

  SENSOR_MPU6050_Priv_Data.accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
  SENSOR_MPU6050_Priv_Data.accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
  SENSOR_MPU6050_Priv_Data.accZoffset = ag[2] / CALIB_OFFSET_NB_MES;

  SENSOR_MPU6050_Priv_Data.gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
  SENSOR_MPU6050_Priv_Data.gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
  SENSOR_MPU6050_Priv_Data.gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
}

void sensor_mpu6050_read_data(){
  // retrieve raw data
  sensor_mpu6050_update_accel();
  sensor_mpu6050_update_gyro();
  sensor_mpu6050_update_temp();

  // estimate tilt angles: this is an approximation for small angles!
  float sgZ = SENSOR_MPU6050_Priv_Data.accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
  SENSOR_MPU6050_Priv_Data.angleAccX =   atan2(SENSOR_MPU6050_Priv_Data.accY, sgZ*sqrt(SENSOR_MPU6050_Priv_Data.accZ*SENSOR_MPU6050_Priv_Data.accZ + SENSOR_MPU6050_Priv_Data.accX*SENSOR_MPU6050_Priv_Data.accX)) * RAD_2_DEG; // [-180,+180] deg
  SENSOR_MPU6050_Priv_Data.angleAccY = - atan2(SENSOR_MPU6050_Priv_Data.accX,     sqrt(SENSOR_MPU6050_Priv_Data.accZ*SENSOR_MPU6050_Priv_Data.accZ + SENSOR_MPU6050_Priv_Data.accY*SENSOR_MPU6050_Priv_Data.accY)) * RAD_2_DEG; // [- 90,+ 90] deg

  unsigned long Tnew = runtime_ms();
  float dt = (Tnew - SENSOR_MPU6050_Priv_Data.preInterval) * 1e-3;
  SENSOR_MPU6050_Priv_Data.preInterval = Tnew;

  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyMPU6050/issues/6
  SENSOR_MPU6050_Priv_Data.angleX = wrap(SENSOR_MPU6050_Priv_Data.filterGyroCoef*(SENSOR_MPU6050_Priv_Data.angleAccX + wrap(SENSOR_MPU6050_Priv_Data.angleX +     SENSOR_MPU6050_Priv_Data.gyroX*dt - SENSOR_MPU6050_Priv_Data.angleAccX,180)) + (1.0-SENSOR_MPU6050_Priv_Data.filterGyroCoef)*SENSOR_MPU6050_Priv_Data.angleAccX,180);
  SENSOR_MPU6050_Priv_Data.angleY = wrap(SENSOR_MPU6050_Priv_Data.filterGyroCoef*(SENSOR_MPU6050_Priv_Data.angleAccY + wrap(SENSOR_MPU6050_Priv_Data.angleY + sgZ*SENSOR_MPU6050_Priv_Data.gyroY*dt - SENSOR_MPU6050_Priv_Data.angleAccY, 90)) + (1.0-SENSOR_MPU6050_Priv_Data.filterGyroCoef)*SENSOR_MPU6050_Priv_Data.angleAccY, 90);
  SENSOR_MPU6050_Priv_Data.angleZ += SENSOR_MPU6050_Priv_Data.gyroZ*dt; // not wrapped

}

float sensor_mpu6050_get_temp(void){
  sensor_mpu6050_update_temp();
  return SENSOR_MPU6050_Priv_Data.temp;
}

SENSOR_MPU6050_Data_t sensor_mpu6050_get_data(void){
  SENSOR_MPU6050_Data_t Data;
  Data.temp = SENSOR_MPU6050_Priv_Data.temp;
  Data.accX = SENSOR_MPU6050_Priv_Data.accX;
  Data.accY = SENSOR_MPU6050_Priv_Data.accY;
  Data.accZ = SENSOR_MPU6050_Priv_Data.accZ;
  Data.gyroX = SENSOR_MPU6050_Priv_Data.gyroX;
  Data.gyroY = SENSOR_MPU6050_Priv_Data.gyroY;
  Data.gyroZ = SENSOR_MPU6050_Priv_Data.gyroZ;
  Data.angleX = SENSOR_MPU6050_Priv_Data.angleX;
  Data.angleY = SENSOR_MPU6050_Priv_Data.angleY;
  Data.angleZ = SENSOR_MPU6050_Priv_Data.angleZ;

  return Data;
}
