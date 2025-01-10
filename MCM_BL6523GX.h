/*
*	BL6523GX Energy Meter IC Support for Arduino
*	Author: Christopher Mendez | @mcmchris
*	Date: 22/10/2024
*/

#ifndef MCM_BL6523GX_h
#define MCM_BL6523GX_h

class BL6523GX
{
  public:
    bool begin(uint32_t baud_rate, uint8_t rxPin, uint8_t txPin);
    bool getCurrent( float *currentA, float *currentB );  //[A]
    bool getVoltage( float *voltage );  //[V]
    bool getActivePower( float *activePowerA, float *activePowerB );  //[W]
    bool getAparentPower( float *apower );  //[VA]
    bool getActiveEnergy( float *activeEnergy );  //[Wh]
    bool getCFCount( float *cf_count );  //[Wh]
    bool getPowerFactor( float *pf );  //[0-1.0]
    bool getFrequency( float *freq );  //[Hz]
    bool setCFOutputMode(uint16_t cf_div); //Energy pulse output CF pin 1 - 256 (2^n)
    bool getCFOutputMode(float *div); //Energy pulse output CF pin 1 - 256 (2^n)
    bool setGain(int V_GAIN = 1, int IB_GAIN = 1, int IA_GAIN = 1); // 2^n (max n = 32)
    bool setMode(bool ch, uint8_t cf_mode, bool dis_out, bool energy_math); // 2^n (max n = 5)
    bool getMode(uint32_t *mode); // 2^n (max n = 5)
    bool Reset();
    bool getLineWattHr(float *l_watt_hr);
    bool getLinecyc(float *linecyc);
    bool setLinecyc(); //
    bool setCal(uint16_t V_CAL, uint16_t I_CAL, uint16_t P_CAL); 

  private:
    uint16_t _V_CAL;
    uint16_t _I_CAL;
    uint16_t _P_CAL;

    const uint16_t timeout = 1000;  //Serial timeout[ms]
    uint8_t _culcCheckSum( uint8_t *txData , int txLenght , uint8_t *rxData , int rxLenght );
    bool _writeRegister( uint8_t address , uint32_t data );
    bool _readRegister( uint8_t address , uint32_t *data );
    uint8_t intToGain(uint8_t gain);
};
#endif /* BL0940 */