/*
*	BL6523GX Energy Meter IC Support for Arduino
*	Author: Christopher Mendez | @mcmchris
*	Date: 22/10/2024
*/
#include <Arduino.h>
#include "MCM_BL6523GX.h"

#define BL_Serial Serial0

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

bool BL6523GX::begin(uint32_t baud_rate, uint8_t rxPin, uint8_t txPin)
{

  /* For M5STACK_PAPER */
  // Serial2.begin(4800, SERIAL_8N1, 18, 19);

  BL_Serial.begin(baud_rate, SERIAL_8N1, rxPin, txPin);

  delay(500);
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
  while (BL_Serial.available() != 0) {
    BL_Serial.read();
  }

  //Register Unlock
  uint8_t unlockTxData[6] = { 0xCA, 0x3E, 0x55, 0, 0, 0 };
  unlockTxData[5] = _culcCheckSum(unlockTxData, sizeof(unlockTxData) - 1, 0, 0);
  BL_Serial.write(unlockTxData, sizeof(unlockTxData));

  //Write Register
  uint8_t txData[6] = { 0xCA, address, (uint8_t)(data), (uint8_t)(data >> 8), (uint8_t)(data >> 16) };
  //uint8_t txData[6] = { 0xCA, address, (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)(data) };
  txData[5] = _culcCheckSum(txData, sizeof(txData) - 1, 0, 0);
  BL_Serial.write(txData, sizeof(txData));

  return true;
}

bool BL6523GX::_readRegister(uint8_t address, uint32_t *data) {
  uint8_t txData[] = { 0x35, address };
  BL_Serial.write(txData, sizeof(txData));

  uint8_t rxData[4] = { 0, 0, 0, 0 };
  uint32_t startTime = millis();
  while (BL_Serial.available() != sizeof(rxData)) {
    delay(10);
    if ((millis() - startTime) > timeout)
      break;
  }
  int rxDataLength = BL_Serial.readBytes(rxData, sizeof(rxData));

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
  while (BL_Serial.available() != 0) {
    BL_Serial.read();
  }

  delay(500);
  return true;
}

bool BL6523GX::setCFOutputMode() {
  if (false == _writeRegister(0x19, 0x080)) {
    ERR("Can not write WA_CFDIV register.");
    return false;
  }
  while (BL_Serial.available() != 0) {
    BL_Serial.read();
  }

  delay(500);
  return true;
}

bool BL6523GX::setGain() {

  if (false == _writeRegister(0x15, 0b000100010001)) {  //Voltage Gain, Current B Gain, Current A Gain
    ERR("Can not write GAIN register.");
    return false;
  }
  while (BL_Serial.available() != 0) {
    BL_Serial.read();
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

  *voltage = (float)data / 5731.66529943;  // / 44122.3179841
  return true;
}

bool BL6523GX::getCurrent(float *currentA, float *currentB) {
  uint32_t dataA;
  if (false == _readRegister(0x05, &dataA)) {
    ERR("Can not read IA_RMS register.");
    return false;
  }

  *currentA = (float)dataA / 54955.4219366;

  uint32_t dataB;
  if (false == _readRegister(0x06, &dataB)) {
    ERR("Can not read IB_RMS register.");
    return false;
  }

  *currentB = (float)dataB / 54955.4219366;

  return true;
}

bool BL6523GX::getFrequency(float *freq) {
  uint32_t data;
  if (false == _readRegister(0x09, &data)) {
    ERR("Can not read FREQ register.");
    return false;
  }

  *freq = (2.73449463549 * 3579545.0) / (float)data;  //
  return true;
}

bool BL6523GX::getActivePower(float *powerA, float *powerB) {
  uint32_t dataA;
  if (false == _readRegister(0x0A, &dataA)) {
    ERR("Can not read POWER_A register.");
    return false;
  }
  if ((float)dataA >= pow(2, 23)) {
    *powerA = ((float)dataA - pow(2, 24) / 58.16538);  //
  } else {
    *powerA = (float)dataA / 58.16538;
  }

  uint32_t dataB;
  if (false == _readRegister(0x13, &dataB)) {
    ERR("Can not read POWER_B register.");
    return false;
  }
  if ((float)dataB >= pow(2, 23)) {
    *powerB = ((float)dataB - pow(2, 24) / 58.16538);  //
  } else {
    *powerB = (float)dataB / 58.16538;
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
    *apower = ((float)data - pow(2, 24) / 58.16538);  //
  } else {
    *apower = (float)data / 58.16538;
  }

  return true;
}

bool BL6523GX::getActiveEnergy(float *activeEnergy) {
  uint32_t data;
  if (false == _readRegister(0x0C, &data)) {
    ERR("Can not read WATTHR register.");
    return false;
  }

  *activeEnergy = (float)data;  //

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
  while (BL_Serial.available() != 0) {
    BL_Serial.read();
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