#include "LoRa.h"

int _ss = LORA_DEFAULT_SS_PIN;
int _reset = LORA_DEFAULT_RESET_PIN;
int _dio0 = LORA_DEFAULT_DIO0_PIN;
long _frequency = 0;
int _implicitHeaderMode = 0;

static const char bus_path[] = "/dev/spi-0";

int spi_bus_register_ra02(void){
  spi_bus *bus;

  bus = spi_bus_alloc_and_init (sizeof(*bus));
  if (bus == NULL) {
    return -1;
  }

  bus->ioctl = bus_ra02_ioctl;

  return spi_bus_register(bus, bus_path);
}

static int bus_ra02_ioctl(spi_bus *bus, ioctl_command_t command, void *arg){
  int err;

  switch (command) {
    case LORA_BEGIN:
      err = begin(915E6);
      break;

    case LORA_END:
      end();
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
}

int module_ra02_begin(int fd){
  return ioctl(fd, LORA_BEGIN, NULL);
}

int module_ra02_end(int fd){
  return ioctl(fd, LORA_END, NULL);
}

int begin(long frequency){

  rtems_status_code sc;
  // setup pin
  sc = rtems_gpio_request_pin(_ss, DIGITAL_OUTPUT, false, false, NULL);
  if (sc != RTEMS_SUCCESSFUL ){
     printf("Failed to request CS\n");
     return(sc);
  }

  // set SS high
  rtems_gpio_set(_ss);

  if (_reset != -1) {
    sc = rtems_gpio_request_pin(_reset, DIGITAL_OUTPUT, false, false, NULL);
    if (sc != RTEMS_SUCCESSFUL ){
       printf("Failed to request Reset\n");
       return(sc);
    }

    // perform reset
    rtems_gpio_clear(_reset);
    rtems_task_wake_after(10);
    rtems_gpio_set(_reset);
    rtems_task_wake_after(10);
  }

  // check version
  uint8_t *version;
  version = NULL;
  readRegister(REG_VERSION, &version);
  if (*version != 0x12) {
    return 0;
  }

  // put in sleep mode
  sleep_module();

  // set frequency
  setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_LNA, &tmp);
  writeRegister(REG_LNA, *tmp | 0x03);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);

  // put in standby mode
  idle_module();

  return 1;
}

void end(){
  // put in sleep mode
  sleep_module();
}

int beginPacket(int implicitHeader){
  if (isTransmitting()) {
    return 0;
  }

  // put in standby mode
  idle_module();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int endPacket(bool async){

  // Not going to use async mode
  // if ((async) && (_onTxDone))
  //     writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE

  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  async = false;
  if (!async) {
    // wait for TX done
    uint8_t *tmp;
    while ((*tmp & IRQ_TX_DONE_MASK) == 0) {
      tmp = NULL;
      readRegister(REG_IRQ_FLAGS, &tmp);
    }
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return 1;
}//NOT CHECKED

bool isTransmitting(){
  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_OP_MODE, &tmp);
  if ((*tmp & MODE_TX) == MODE_TX) {
    return true;
  }

  tmp = NULL;
  readRegister(REG_IRQ_FLAGS, &tmp);
  if (*tmp & IRQ_TX_DONE_MASK) {
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

long packetFrequencyError(){
  int32_t freqError = 0;
  uint8_t *tmp;

  tmp = NULL;
  readRegister(REG_FREQ_ERROR_MSB, &tmp);
  freqError = (int32_t)(*tmp & 0b111);
  freqError <<= 8L;
  tmp = NULL;
  readRegister(REG_FREQ_ERROR_MID, &tmp);
  freqError += (int32_t)(*tmp);
  freqError <<= 8L;
  tmp = NULL;
  readRegister(REG_FREQ_ERROR_LSB, &tmp);
  freqError += (int32_t)(*tmp);

  tmp = NULL;
  readRegister(REG_FREQ_ERROR_MSB, &tmp);
  if (*tmp & 0b1000) { // Sign bit is on
     freqError -= 524288; // 0b1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = (((float)(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return (long)(fError);
}

int rssi(){
  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_RSSI_VALUE, &tmp);
  return (*tmp - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t write_module(uint8_t byte){
  return write_comp(&byte, sizeof(byte));
}

size_t write_comp(const uint8_t *buffer, size_t size){
  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_PAYLOAD_LENGTH, &tmp);
  int currentLength = *tmp;

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

void idle_module(){
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sleep_module(){
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void setTxPower(int level, int outputPin){
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      writeRegister(REG_PA_DAC, 0x87);
      setOCP(140);
    } else {
      if (level < 2) {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      writeRegister(REG_PA_DAC, 0x84);
      setOCP(100);
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void setFrequency(long frequency){
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int getSpreadingFactor(){
  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_2, &tmp);
  return *tmp >> 4;
}

void setSpreadingFactor(int sf){
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_2, &tmp);
  writeRegister(REG_MODEM_CONFIG_2, (*tmp & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

long getSignalBandwidth(){
  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_1, &tmp);
  uint8_t bw = (*tmp >> 4);

  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }

  return -1;
}

void setSignalBandwidth(long sbw){
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_1, &tmp);
  writeRegister(REG_MODEM_CONFIG_1, (*tmp & 0x0f) | (bw << 4));
  setLdoFlag();
}

void setLdoFlag(){
  // Section 4.1.1.5
  long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

  // Section 4.1.1.6
  bool ldoOn = symbolDuration > 16;

  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_3, &tmp);
  uint8_t config3 = *tmp;

  if (ldoOn)
    config3 |=  (1<<(3));
  else
    config3 &= ~(1<<(3));

  writeRegister(REG_MODEM_CONFIG_3, config3);
}

void setCodingRate4(int denominator){
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_1, &tmp);
  writeRegister(REG_MODEM_CONFIG_1, (*tmp & 0xf1) | (cr << 1));
}

void setPreambleLength(long length){
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void setSyncWord(int sw){
  writeRegister(REG_SYNC_WORD, sw);
}

void enableCrc(){
  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_2, &tmp);
  writeRegister(REG_MODEM_CONFIG_2, *tmp | 0x04);
}

void disableCrc(){
  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_2, &tmp);
  writeRegister(REG_MODEM_CONFIG_2, *tmp & 0xfb);
}

void enableInvertIQ(){
  writeRegister(REG_INVERTIQ,  0x66);
  writeRegister(REG_INVERTIQ2, 0x19);
}

void disableInvertIQ(){
  writeRegister(REG_INVERTIQ,  0x27);
  writeRegister(REG_INVERTIQ2, 0x1d);
}

void setOCP(uint8_t mA){
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void setGain(uint8_t gain){
  // check allowed range
  if (gain > 6) {
    gain = 6;
  }

  // set to standby
  idle_module();

  // set gain
  if (gain == 0) {
    // if gain = 0, enable AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);
  } else {
    // disable AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x00);

    // clear Gain and set LNA boost
    writeRegister(REG_LNA, 0x03);

    // set gain
    uint8_t *tmp;
    tmp = NULL;
    readRegister(REG_LNA, &tmp);
    writeRegister(REG_LNA, *tmp | (gain << 5));
  }
}

void setPins(int ss, int reset, int dio0){
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

void explicitHeaderMode(){
  _implicitHeaderMode = 0;

  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_1, &tmp);
  writeRegister(REG_MODEM_CONFIG_1, *tmp & 0xfe);
}

void implicitHeaderMode(){
  _implicitHeaderMode = 1;

  uint8_t *tmp;
  tmp = NULL;
  readRegister(REG_MODEM_CONFIG_1, &tmp);
  writeRegister(REG_MODEM_CONFIG_1, *tmp | 0x01);
}

void readRegister(uint8_t address, uint8_t **buff){
  uint8_t *tmp;
  tmp = NULL;
  singleTransfer(address & 0x7f, 0x00, &tmp);

  (*buff)[0] = *tmp;
  free(tmp);
}

void writeRegister(uint8_t address, uint8_t value){
  uint8_t *tmp;
  tmp = NULL;
  singleTransfer(address | 0x80, value, &tmp);

  free(tmp);
}

int singleTransfer(uint8_t data_address, uint8_t value, uint8_t **buff){
  uint8_t response[1];
  uint8_t data_address_buff[1];
  uint8_t value_buff[1];

  int rv;
  int fd;
  spi_ioc_transfer msg_address = {
    .len = 1,
    .rx_buf = response,
    .tx_buf = data_address_buff,
    .speed_hz = (uint32_t) 915E6,
    .bits_per_word = 8,
    .cs_change = true,
    .mode = SPI_MODE_0 | SPI_NO_CS,
  };
  spi_ioc_transfer msg_val ={
    .len = 1,
    .rx_buf = response,
    .tx_buf = value_buff,
    .speed_hz = (uint32_t) 915E6,
    .bits_per_word = 8,
    .cs_change = true,
    .mode = SPI_MODE_0 | SPI_NO_CS,
  };

  data_address_buff[0] = data_address;
  value_buff[0] = value;

  rtems_gpio_clear(_ss);

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus");
    return 1;
  }

  // First write the register address
  rv = ioctl(fd, SPI_IOC_MESSAGE(1), &msg_address);
  if (rv == -1) {
    printf("Couldn't send the address message");
    return 1;
  } else {
    printf("received: %02x", response[1]);
  }

  //Then write the value into the address
  rv = ioctl(fd, SPI_IOC_MESSAGE(1), &msg_val);
  if (rv == -1) {
    printf("Couldn't send the value message");
    return 1;
  } else {
    printf("received: %02x", response[1]);
    free(*buff);
    *buff = malloc(1 * sizeof(uint8_t));
    (*buff)[1] = response[1];
  }
  close(fd);

  // set SS high
  rtems_gpio_set(_ss);

  return 0;
}
