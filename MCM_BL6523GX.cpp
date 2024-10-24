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

bool BL6523GX::setCFOutputMode(uint16_t cf_div) {
  if (false == _writeRegister(0x19, cf_div)) {
    ERR("Can not write WA_CFDIV register.");
    return false;
  }
  while (BL_Serial.available() != 0) {
    BL_Serial.read();
  }

  delay(500);
  return true;
}

bool BL6523GX::setGain(byte V_GAIN , byte IB_GAIN, byte IA_GAIN) {

  uint32_t gain_data = intToGain(V_GAIN) << 8 | intToGain(IB_GAIN) << 4 | intToGain(IA_GAIN);

  Serial.println(gain_data, BIN);

  if (false == _writeRegister(0x15, gain_data)) {  //Voltage Gain, Current B Gain, Current A Gain
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

  *voltage = (float)data / 50387.1788455;  // 5731.66529943  / 50641.219513
  return true;
}

bool BL6523GX::getCurrent(float *currentA, float *currentB) {
  uint32_t dataA;
  if (false == _readRegister(0x05, &dataA)) {
    ERR("Can not read IA_RMS register.");
    return false;
  }

  *currentA = (float)dataA / 55559.025609; /// 54955.4219366

  uint32_t dataB;
  if (false == _readRegister(0x06, &dataB)) {
    ERR("Can not read IB_RMS register.");
    return false;
  }

  *currentB = (float)dataB / 55559.025609; /// 54955.4219366

  return true;
}

bool BL6523GX::getFrequency(float *freq) {
  uint32_t data;
  if (false == _readRegister(0x09, &data)) {
    ERR("Can not read FREQ register.");
    return false;
  }

  //*freq = (2.73449463549 * 3579545.0) / (float)data;  //
  *freq = (87.3906 * 3579545.0) / (32.0*(float)data);  // constants defined by datasheet
  return true;
}

bool BL6523GX::getActivePower(float *powerA, float *powerB) {
  uint32_t dataA;
  if (false == _readRegister(0x0A, &dataA)) {
    ERR("Can not read POWER_A register.");
    return false;
  }

  if ((float)dataA >= pow(2, 23)) {
    *powerA = ((float)dataA - pow(2, 24)) / 481.462140704;  //
  } else {
    *powerA = (float)dataA / 481.462140704;
  }
  

  uint32_t dataB;
  if (false == _readRegister(0x13, &dataB)) {
    ERR("Can not read POWER_B register.");
    return false;
  }


  if ((float)dataB >= pow(2, 23)) {
    *powerB = ((float)dataB - pow(2, 24)) / 481.462140704;  //
  } else {
    *powerB = (float)dataB / 481.462140704;
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
    *apower = ((float)data - pow(2, 24) / 481.462140704);  //
  } else {
    *apower = (float)data / 481.462140704;
  }

  return true;
}

bool BL6523GX::getActiveEnergy(float *activeEnergy) {
  uint32_t data;
  if (false == _readRegister(0x0C, &data)) {
    ERR("Can not read WATTHR register.");
    return false;
  }
  float div;
  getCFOutputMode(&div);
  Serial.println(div);
  
  *activeEnergy = (float)data*(1000.0/(2062.0*(div/4.0)));
  return true;
}

bool BL6523GX::getAparentEnergy( float *aparentEnergy ) {
  uint32_t data;
  if (false == _readRegister(0x0D, &data)) {
    ERR("Can not read VAHR register.");
    return false;
  }
  
  *aparentEnergy = (float)data; // / 481.462140704
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
  while (BL_Serial.available() != 0) {
    BL_Serial.read();
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

bool BL6523GX::setMode() {
  if (false == _writeRegister(0x14, 0b001000000000000000010001)) { //0b000000000000000000010000 // first bit define which channel to CF respond to 0 = A, 1 = B
    ERR("Can not write MODE register.");
    return false;
  }
  while (BL_Serial.available() != 0) {
    BL_Serial.read();
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

byte BL6523GX::intToGain(uint8_t gain){
    uint8_t data;
    switch(gain){
        case 1:
            data = 0b000;
            break;
        case 2:
            data = 0b001;
            break;
        case 4:
            data = 0b010;
            break;
        case 8:
            data = 0b011;
            break;
        case 16:
            data = 0b100;
            break;
        case 24:
            data = 0b101;
            break;
        case 32:
            data = 0b110;
            break;
        default:
        Serial.println("Gain out of range");
            break;
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