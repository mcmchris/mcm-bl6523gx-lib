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
    BL6523GX();
    ~BL6523GX(); 
    bool getCurrent( float *currentA, float *currentB );  //[A]
    bool getVoltage( float *voltage );  //[V]
    bool getActivePower( float *activePowerA, float *activePowerB );  //[W]
    bool getAparentPower( float *apower );  //[VA]
    bool getActiveEnergy( float *activeEnergy );  //[Wh]
    bool getPowerFactor( float *pf );  //[0-1.0]
    bool getFrequency( float *freq );  //[Hz]
    bool setCFOutputMode(); //Energy pulse output CF pin
    bool setGain(); // 2^n (max n = 5)
    bool Reset();

  private:
    const uint16_t timeout = 1000;  //Serial timeout[ms]
    uint8_t _culcCheckSum( uint8_t *txData , int txLenght , uint8_t *rxData , int rxLenght );
    bool _writeRegister( uint8_t address , uint32_t data );
    bool _readRegister( uint8_t address , uint32_t *data );
};
#endif /* BL0940 */