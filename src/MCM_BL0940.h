/*
*	BL0940 Energy Meter IC Support for Arduino, ESP32, RASP Pi Pico
*	Author: Christopher Mendez | @mcmchris
*	Date: 18/02/2023
* 	Tutorial:
*	Based on Kohacraft previews work.
*/

#pragma once

#include <Arduino.h>

#ifndef MCM_BL0940_h
#define MCM_BL0940_h

class BL0940
{
  public:

    bool begin(HardwareSerial& serial, int8_t rxPin = -1, int8_t txPin = -1);
    bool setCurrentOffset(int8_t offset);
    bool setActivePowerOffset(int8_t offset);
    bool setActivePowerNoLoadThreshold(int8_t threshold);
    bool getCurrent( float *current );  //[A]
    bool getVoltage( float *voltage );  //[V]
    bool getActivePower( float *activePower );  //[W]
    bool getActiveEnergy( float *activeEnergy );  //[kWh]
    bool getPowerFactor( float *powerFactor );  //[%]
    bool getTemperature( float *temperature );  //[deg C]
    bool setFrequency( uint32_t Hz = 60 );  //50 or 60  [Hz]
    bool setUpdateRate( uint32_t rate = 400 );  //400 or 800  [ms]
    bool setOverCurrentDetection( float detectionCurrent = 15.0 );  //[A] CF pin is high if current is larger than detectionCurrent
    bool setCFOutputMode(); //Energy pulse output CF pin
    bool Reset();

  private:
    HardwareSerial* serialPtr = nullptr;  // pointer to HardwareSerial
    const uint16_t timeout = 1000;  //Serial timeout[ms]
    const float Vref = 1.218; //[V]
    const float R5 = 2;   //[Ohm]
    const float Rt = 1000.0;  //n:1
    const float R8 = 462.0;  //[kOhm]
    const float R9 = 0.0;  //[kOhm]
    const float R10 = 0.0;  //[kOhm]
    const float R11 = 0.0;  //[kOhm]
    const float R12 = 0.0;  //[kOhm]
    const float R6 = 120;  //[Ohm]
    uint16_t Hz = 50;   //[Hz]
    uint16_t updateRate = 400; //[ms]

    uint8_t _culcCheckSum( uint8_t *txData , int txLenght , uint8_t *rxData , int rxLenght );
    bool _writeRegister( uint8_t address , uint32_t data );
    bool _readRegister( uint8_t address , uint32_t *data );
    
};
#endif /* BL0940 */
