/**
 * @file
 *
 * @brief IMU Sensor MPL3115A2 Driver Implementation
 *
 * @ingroup I2CSensorMPL3115A2
 */

#include "sensor-mpl3115a2.h"

static const char bus_path[] = "/dev/i2c-2";

MPL3115A2_Data_t MPL3115A2_Data;

static int sensor_mpl3115a2_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);
static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);
static int set_bytes(uint16_t chip_address, uint8_t **val, int numBytes);

static int sensor_mpl3115a2_set_reg_8(uint8_t register_add, uint8_t value);
static int sensor_mpl3115a2_get_reg_8(uint8_t register_add, uint8_t **buff);

static bool begin(void);
static int readMeasurement(uint8_t **buff);

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

static int set_bytes(uint16_t chip_address, uint8_t **val, int numBytes){

  int fd;
  int rv;

  if(chip_address == 0){
    chip_address = (uint16_t) MPL3115A2_ADDRESS;
  }

  uint8_t writebuff[numBytes];

  for(int i = 0; i<numBytes; i++){
    writebuff[i] = (*val)[i];
  }

  i2c_msg msgs[] = {{
    .addr = chip_address,
    .flags = 0,
    .buf = writebuff,
    .len = numBytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    perror("ioctl failed");
  }
  close(fd);

  return rv;
}

static int sensor_mpl3115a2_get_reg_8(uint8_t register_add, uint8_t **buff){

  int fd;
  int rv;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(1 * sizeof(uint8_t));

  uint16_t nr_bytes = (uint16_t) 1;
  uint16_t chip_address = (uint16_t) MPL3115A2_ADDRESS;
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

static int sensor_mpl3115a2_set_reg_8(uint8_t register_add, uint8_t value){
  int rv;

  int nr_bytes = 2;
  uint8_t *val;
  val = NULL;
  val = malloc(nr_bytes * sizeof(uint8_t));

  val[0] = register_add;
  val[1] = value;

  rv = set_bytes(MPL3115A2_ADDRESS, &val, nr_bytes);

  return rv;
}

static int sensor_mpl3115a2_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg){
  int err;

  switch (command) {
    case SENSOR_MPL3115A2_BEGIN:
      if(begin()){
        err = 0;
      }else{
        err = -1;
      }
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
}

int i2c_dev_register_sensor_mpl3115a2(const char *bus_path, const char *dev_path){
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, MPL3115A2_ADDRESS);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = sensor_mpl3115a2_ioctl;

  return i2c_dev_register(dev, dev_path);
}

int sensor_mpl3115a2_begin(int fd){
  return ioctl(fd, SENSOR_MPL3115A2_BEGIN, NULL);
}


static bool begin(void) {
  uint8_t *tmp;
  tmp = NULL;

  // sanity check
  uint8_t *whoami;
  whoami = NULL;
  sensor_mpl3115a2_get_reg_8(MPL3115A2_WHOAMI, &whoami);
  if ((*whoami) != 0xC4) {
    return false;
  }

  // software reset
  sensor_mpl3115a2_set_reg_8(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
  sensor_mpl3115a2_get_reg_8(MPL3115A2_CTRL_REG1, &tmp);
  while ((*tmp) & MPL3115A2_CTRL_REG1_RST)
    delay_ms(10);

  // set oversampling and altitude mode
  MPL3115A2_Data.currentMode = MPL3115A2_ALTIMETER;
  MPL3115A2_Data.ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
  sensor_mpl3115a2_set_reg_8(MPL3115A2_CTRL_REG1, MPL3115A2_Data.ctrl_reg1.reg);

  // enable data ready events for pressure/altitude and temperature
  sensor_mpl3115a2_set_reg_8(MPL3115A2_PT_DATA_CFG, MPL3115A2_PT_DATA_CFG_TDEFE |
                                    MPL3115A2_PT_DATA_CFG_PDEFE |
                                    MPL3115A2_PT_DATA_CFG_DREM);

  return true;
}

#ifdef pressure_read

  float sensor_mpl3115a2_getPressure() {
    if (MPL3115A2_Data.currentMode != MPL3115A2_BAROMETER)
      sensor_mpl3115a2_setMode(MPL3115A2_BAROMETER);
    sensor_mpl3115a2_startOneShot();
    while (!sensor_mpl3115a2_conversionComplete())
      delay_ms(10);
    return sensor_mpl3115a2_getLastConversionResults(MPL3115A2_PRESSURE);
  }

  float sensor_mpl3115a2_getAltitude() {
    if (MPL3115A2_Data.currentMode != MPL3115A2_ALTIMETER)
      sensor_mpl3115a2_setMode(MPL3115A2_ALTIMETER);
    sensor_mpl3115a2_startOneShot();
    while (!sensor_mpl3115a2_conversionComplete())
      delay_ms(10);
    return sensor_mpl3115a2_getLastConversionResults(MPL3115A2_ALTITUDE);
  }

  int8_t sensor_mpl3115a2_getAltitudeOffset(void) {
    uint8_t *tmp;
    tmp = NULL;
    sensor_mpl3115a2_get_reg_8(MPL3115A2_OFF_H, &tmp);
    return (int8_t)(*tmp);
  }

  void sensor_mpl3115a2_setAltitudeOffset(int8_t offset) {
    sensor_mpl3115a2_set_reg_8(MPL3115A2_OFF_H, (uint8_t)(offset));
  }

  // Set the local sea level pressure in hPa
  void sensor_mpl3115a2_setSeaPressure(float SLP) {
    // multiply by 100 to convert hPa to Pa
    // divide by 2 to convert to 2 Pa per LSB
    // convert to integer
    uint16_t bar = SLP * 50;

    // write result to register
    uint8_t *val;
    val = NULL;
    val = malloc(3 * sizeof(uint8_t));

    val[0] = MPL3115A2_BAR_IN_MSB;
    val[1] = bar >> 8;
    val[2] = bar & 0xFF;

    set_bytes(MPL3115A2_ADDRESS, &val, 3);
  }
#endif

#ifdef temperature_read
  float sensor_mpl3115a2_getTemperature() {
    sensor_mpl3115a2_startOneShot();
    while (!sensor_mpl3115a2_conversionComplete())
      delay_ms(10);
    return sensor_mpl3115a2_getLastConversionResults(MPL3115A2_TEMPERATURE);
  }
#endif

// Set measurement mode. Can be MPL3115A2_BAROMETER or MPL3115A2_ALTIMETER.
void sensor_mpl3115a2_setMode(mpl3115a2_mode_t mode) {
  // assumes STANDBY mode
  uint8_t *tmp;
  tmp = NULL;
  sensor_mpl3115a2_get_reg_8(MPL3115A2_CTRL_REG1, &tmp);
  MPL3115A2_Data.ctrl_reg1.reg = (*tmp);
  MPL3115A2_Data.ctrl_reg1.bit.ALT = mode;
  sensor_mpl3115a2_set_reg_8(MPL3115A2_CTRL_REG1, MPL3115A2_Data.ctrl_reg1.reg);
  MPL3115A2_Data.currentMode = mode;
}

void sensor_mpl3115a2_startOneShot(void) {
  // wait for one-shot to clear before proceeding
  uint8_t *tmp;
  tmp = NULL;
  sensor_mpl3115a2_get_reg_8(MPL3115A2_CTRL_REG1, &tmp);
  MPL3115A2_Data.ctrl_reg1.reg = (*tmp);
  while (MPL3115A2_Data.ctrl_reg1.bit.OST) {
    delay_ms(10);
    tmp = NULL;
    sensor_mpl3115a2_get_reg_8(MPL3115A2_CTRL_REG1, &tmp);
    MPL3115A2_Data.ctrl_reg1.reg = (*tmp);
  }
  // initiate one-shot measurement
  MPL3115A2_Data.ctrl_reg1.bit.OST = 1;
  sensor_mpl3115a2_set_reg_8(MPL3115A2_CTRL_REG1, MPL3115A2_Data.ctrl_reg1.reg);
}

bool sensor_mpl3115a2_conversionComplete(void) {
  // PTDR bit works for either pressure or temperature
  // 0: No new set of data ready
  // 1: A new set of data is ready
  uint8_t *tmp;
  tmp = NULL;
  sensor_mpl3115a2_get_reg_8(MPL3115A2_REGISTER_STATUS, &tmp);
  return (((*tmp) & MPL3115A2_REGISTER_STATUS_PTDR) !=
          0);
}

float sensor_mpl3115a2_getLastConversionResults(mpl3115a2_meas_t value) {
  int rv;
  uint8_t *val;

  val = NULL;
  val = malloc(5 * sizeof(uint8_t));
  rv = readMeasurement(&val);

  if (rv >= 0){
    for (int i = 0; i < 5; i++) {
      MPL3115A2_Data.rawData[i] = (val)[i];
    }
  }
  uint32_t pressure;
  int32_t alt;
  int16_t t;

  switch (value) {
  case MPL3115A2_PRESSURE:
    pressure = (uint32_t)(MPL3115A2_Data.rawData[0]) << 16 |
               (uint32_t)(MPL3115A2_Data.rawData[1]) << 8 |
               (uint32_t)(MPL3115A2_Data.rawData[2]);
    return (float)(pressure) / 6400.0;
  case MPL3115A2_ALTITUDE:
    alt = (uint32_t)(MPL3115A2_Data.rawData[0]) << 24 |
          (uint32_t)(MPL3115A2_Data.rawData[1]) << 16 |
          (uint32_t)(MPL3115A2_Data.rawData[2]) << 8;
    return (float)(alt) / 65536.0;
  case MPL3115A2_TEMPERATURE:
  default:
    t = (uint16_t)(MPL3115A2_Data.rawData[3]) << 8 |
        (uint16_t)(MPL3115A2_Data.rawData[4]);
    return (float)(t) / 256.0;
  }
}

static int readMeasurement(uint8_t **buff){
  uint16_t nr_bytes = (uint16_t) 5;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(nr_bytes * sizeof(uint8_t));

  uint16_t chip_address = (uint16_t) MPL3115A2_ADDRESS;
  uint8_t data_address = (uint8_t) MPL3115A2_REGISTER_PRESSURE_MSB;

  int fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  int rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  for (int i = 0; i < nr_bytes; ++i) {
    (*buff)[i] = tmp[i];
  }
  free(tmp);

  return rv;
}
