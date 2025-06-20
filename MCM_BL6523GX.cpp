/*
*	BL6523GX Energy Meter IC Support for Arduino
*	Author: Christopher Mendez | @mcmchris
*	Date: 22/10/2024
*/
#include <Arduino.h>
#include "MCM_BL6523GX.h"


#define BL6523GX_DEBUG 1
#if BL6523GX_DEBUG
#define DBG(...)                 \
  {                              \
    Serial.println(__VA_ARGS__); \
  }
#define ERR(...)                 \
  {                              \
    Serial.println(__VA_ARGS__); \
  }
#else
#define DBG(...)
#define ERR(...)
#endif /* BL6523GX_DBG */

bool BL6523GX::begin(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
{

serialPtr = &serial;  // default

#if defined (ARDUINO_RASPBERRY_PI_PICO)
  serialPtr->begin(4800);
#else
  serialPtr->begin(4800, SERIAL_8N1, rxPin, txPin);
#endif
  return true;
}


uint8_t BL6523GX::_culcCheckSum(uint8_t *txData, int txLenght, uint8_t *rxData, int rxLenght) {

  uint8_t checksum = 0;
  for (int i = 1; i < txLenght; i++) {
    checksum += txData[i];
  }
  for (int i = 0; i < rxLenght; i++) {
    checksum += rxData[i];
  }
  checksum = ~checksum;
  return checksum;
}

bool BL6523GX::_writeRegister(uint8_t address, uint32_t data) {
  //read buffer clear
  while (serialPtr->available() != 0) {
    serialPtr->read();
  }

  //Register Unlock
  uint8_t unlockTxData[6] = { 0xCA, 0x3E, 0x55, 0, 0, 0 };
  unlockTxData[5] = _culcCheckSum(unlockTxData, sizeof(unlockTxData) - 1, 0, 0);
  serialPtr->write(unlockTxData, sizeof(unlockTxData));

  //Write Register
  uint8_t txData[6] = { 0xCA, address, (uint8_t)(data), (uint8_t)(data >> 8), (uint8_t)(data >> 16) };
  //uint8_t txData[6] = { 0xCA, address, (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)(data) };
  txData[5] = _culcCheckSum(txData, sizeof(txData) - 1, 0, 0);
  serialPtr->write(txData, sizeof(txData));

  return true;
}

bool BL6523GX::_readRegister(uint8_t address, uint32_t *data) {
  uint8_t txData[] = { 0x35, address };
  serialPtr->write(txData, sizeof(txData));

  uint8_t rxData[4] = { 0, 0, 0, 0 };
  uint32_t startTime = millis();
  while (serialPtr->available() != sizeof(rxData)) {
    delay(10);
    if ((millis() - startTime) > timeout)
      break;
  }
  int rxDataLength = serialPtr->readBytes(rxData, sizeof(rxData));

  if (rxDataLength == 0) {
    ERR("Serial Timeout.");
    return false;
  }

  uint8_t checksum = _culcCheckSum(txData, sizeof(txData), rxData, sizeof(rxData) - 1);

  if (rxData[3] != checksum) {
    char message[128];
    sprintf(message, "Checksum error true:%x read:%x.", checksum, rxData[3]);
    ERR(message);
    return false;
  }

  *data = ((uint32_t)rxData[2] << 16) | ((uint32_t)rxData[1] << 8) | (uint32_t)rxData[0];
  return true;
}

bool BL6523GX::Reset() {
  if (false == _writeRegister(0x3F, 0x5A5A5A)) {
    ERR("Can not write SOFT_RESET register.");
    return false;
  }
  while (serialPtr->available() != 0) {
    serialPtr->read();
  }

  delay(500);
  return true;
}

bool BL6523GX::setCFOutputMode(uint16_t cf_div) {
  if (false == _writeRegister(0x19, cf_div)) {
    ERR("Can not write WA_CFDIV register.");
    return false;
  }
  while (serialPtr->available() != 0) {
    serialPtr->read();
  }

  delay(500);
  return true;
}

bool BL6523GX::setCal(uint16_t V_CAL, uint16_t I_CAL, uint16_t P_CAL){
  _V_CAL = V_CAL;
  _I_CAL = I_CAL;
  _P_CAL = P_CAL;

  return true;
}

bool BL6523GX::setGain(int V_GAIN , int IB_GAIN, int IA_GAIN) {

  uint32_t gain_data = intToGain(V_GAIN) << 8 | intToGain(IB_GAIN) << 4 | intToGain(IA_GAIN);

  Serial.println(gain_data, BIN);

  if (false == _writeRegister(0x15, gain_data)) {  //Voltage Gain, Current B Gain, Current A Gain
    ERR("Can not write GAIN register.");
    return false;
  }
  while (serialPtr->available() != 0) {
    serialPtr->read();
  }

  delay(500);
  return true;
}

bool BL6523GX::getVoltage(float *voltage) {
  uint32_t data;
  if (false == _readRegister(0x07, &data)) {
    ERR("Can not read V_RMS register.");
    return false;
  }
  if (_V_CAL == 0) {
    ERR("Voltage calibration factor is 0!");
    return false;
  }

  *voltage = (float)data / _V_CAL; //50387.1788455;  // 5731.66529943  / 50641.219513
  return true;
}

bool BL6523GX::getCurrent(float *currentA, float *currentB) {
  uint32_t dataA;
  if (false == _readRegister(0x05, &dataA)) {
    ERR("Can not read IA_RMS register.");
    return false;
  }

  *currentA = (float)dataA / _I_CAL; // 55559.025609

  uint32_t dataB;
  if (false == _readRegister(0x06, &dataB)) {
    ERR("Can not read IB_RMS register.");
    return false;
  }

  *currentB = (float)dataB / _I_CAL; // 55559.025609

  return true;
}

bool BL6523GX::getFrequency(float *freq) {
  uint32_t data;
  if (false == _readRegister(0x09, &data)) {
    ERR("Can not read FREQ register.");
    return false;
  }

  *freq = (87.3906 * 3579545.0) / (32.0*(float)data);  // 87.3906 * osc_freq : constants defined by datasheet
  return true;
}

bool BL6523GX::getActivePower(float *powerA, float *powerB) {
  uint32_t dataA;
  if (false == _readRegister(0x0A, &dataA)) {
    ERR("Can not read POWER_A register.");
    return false;
  }

  if ((float)dataA >= pow(2, 23)) {
    *powerA = ((float)dataA - pow(2, 24)) / _P_CAL; //481.462140704;
  } else {
    *powerA = (float)dataA / _P_CAL; //481.462140704;
  }
  

  uint32_t dataB;
  if (false == _readRegister(0x13, &dataB)) {
    ERR("Can not read POWER_B register.");
    return false;
  }


  if ((float)dataB >= pow(2, 23)) {
    *powerB = ((float)dataB - pow(2, 24)) / _P_CAL; //481.462140704;
  } else {
    *powerB = (float)dataB / _P_CAL; //481.462140704;
  }

  return true;
}

bool BL6523GX::getAparentPower(float *apower) {
  uint32_t data;
  if (false == _readRegister(0x0B, &data)) {
    ERR("Can not read VA register.");
    return false;
  }
  if ((float)data >= pow(2, 23)) {
    *apower = ((float)data - pow(2, 24)) / _P_CAL; //481.462140704;
  } else {
    *apower = (float)data / _P_CAL; //481.462140704;
  }

  return true;
}

bool BL6523GX::getCFCount(float *cf_count) {
  uint32_t data;
  if (false == _readRegister(0x0C, &data)) {
    ERR("Can not read WATTHR register.");
    return false;
  }
  float div;
  getCFOutputMode(&div);

  *cf_count = (float)data; //*(1000.0/(2062.0*(div/4.0)));
  return true;
}

bool BL6523GX::getActiveEnergy( float *activeEnergy ) {
  uint32_t data;
  if (false == _readRegister(0x0D, &data)) {
    ERR("Can not read VAHR register.");
    return false;
  }
  
  *activeEnergy = (float)data; // / 481.462140704
  return true;
}

bool BL6523GX::getLineWattHr(float *l_watt_hr) {
  uint32_t data;
  if (false == _readRegister(0x04, &data)) {
    ERR("Can not read LINE_WATTHR register.");
    return false;
  }

  *l_watt_hr = (float)data;
  return true;
}

bool BL6523GX::setLinecyc() {
  if (false == _writeRegister(0x31, 0x001)) { 
    ERR("Can not write LINECCC register.");
    return false;
  }
  while (serialPtr->available() != 0) {
    serialPtr->read();
  }

  delay(500);
  return true;
}

bool BL6523GX::getLinecyc(float *linecyc) {
  uint32_t data;
  if (false == _readRegister(0x31, &data)) {
    ERR("Can not read LINECYC register.");
    return false;
  }

  *linecyc = (float)data;
  return true;
}

bool BL6523GX::getPowerFactor(float *pf) {
  uint32_t data;
  if (false == _readRegister(0x08, &data)) {
    ERR("Can not read FREQ register.");
    return false;
  }

  bool pf_bit = data >> 23;  // PF signed bit

  if (pf_bit) {  // positive
    *pf = ((float)data - pow(2, 24)) / pow(2, 23);
  } else {  // negative
    *pf = (float)data / pow(2, 23);
  }

  return true;
}

/*
Energy Channel Selection
  ch = 0: the CF register will measure energy on channel A
  ch = 1:  the CF register will measure energy on channel B
CF Accumulation Setup
  cf_mode = 0: absolute energy count
  cf_mode = 1: positive energy count
  cf_mode = 2: arithmetical energy count
  cf_mode = 3: negative energy count
Control CF output
  dis_out = 0: CF enabled
  dis_out = 1: CF disabled
Energy Register Accumulation
  energy_math = 0: algebraic sum accumulation
  energy_math = 1: absolute accumulation
*/
bool BL6523GX::setMode(bool ch, uint8_t cf_mode, bool dis_out, bool energy_math) {

  uint32_t mode_data = energy_math << 21 | dis_out << 17 | cf_mode << 8 | ch;

  //Serial.println(mode_data, BIN);

  if (false == _writeRegister(0x14, mode_data)) { // first bit define which channel to CF respond to 0 = A, 1 = B
    ERR("Can not write MODE register.");
    return false;
  }
  while (serialPtr->available() != 0) {
    serialPtr->read();
  }

  delay(500);
  return true;
}

bool BL6523GX::getCFOutputMode(float *div) {
  uint32_t data;
  if (false == _readRegister(0x19, &data)) {
    ERR("Can not read WA_CFDIV register.");
    return false;
  }

  *div = (uint32_t)data;
  return true;
}

uint8_t BL6523GX::intToGain(uint8_t gain)
{
  uint8_t data;
  if (gain <= 32)
  {
    switch (gain)
    {
    case 1:
      data = 0b0000;
      break;
    case 2:
      data = 0b0001;
      break;
    case 4:
      data = 0b0010;
      break;
    case 8:
      data = 0b0011;
      break;
    case 16:
      data = 0b0100;
      break;
    case 24:
      data = 0b0101;
      break;
    case 32:
      data = 0b0110;
      break;
    default:
      Serial.println("Gain out of range!");
      data = 0b0000;
    }
  }

  return data;
}

bool BL6523GX::getMode(uint32_t *mode) {
  uint32_t data;
  if (false == _readRegister(0x14, &data)) {
    ERR("Can not read MODE register.");
    return false;
  }

  *mode = (uint32_t)data;
  return true;
}

/*******************************************************************/
/*

bool getGain(uint32_t *gain) {
  uint32_t data;
  if (false == _readRegister(0x15, &data)) {
    ERR("Can not read GAIN register.");
    return false;
  }

  *gain = data;
  return true;
}

bool getCF(uint32_t *cf) {
  uint32_t data;
  if (false == _readRegister(0x19, &data)) {
    ERR("Can not read WA_CFDIV register.");
    return false;
  }

  *cf = (uint32_t)data;
  return true;
}

bool getMode(uint32_t *mode) {
  uint32_t data;
  if (false == _readRegister(0x14, &data)) {
    ERR("Can not read MODE register.");
    return false;
  }

  *mode = (uint32_t)data;
  return true;
}

bool setMode() {
  if (false == _writeRegister(0x14, 0x5A5A5A)) {
    ERR("Can not write SOFT_RESET register.");
    return false;
  }
  while (serialPtr->available() != 0) {
    serialPtr->read();
  }

  delay(500);
  return true;
}

void ReverseArray(uint8_t arr[], int size) {
  for (int i = 0; i < size / 2; i++) {
    int temp = arr[i];
    arr[i] = arr[size - 1 - i];
    arr[size - 1 - i] = temp;
  }
}

*/