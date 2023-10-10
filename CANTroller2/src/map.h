// Soren: Modified to prevent blocking. Below are library .h file and .c file with additional timer and logic
// This is a library for the Qwiic MicroPressure Sensor, which can read from 0 to 25 PSI.
// By: Alex Wende July 2020 (Beerware license).
#pragma once
#include <Wire.h>   // for i2c bus support
#include "utils.h"  // for Timer class
#define DEFAULT_ADDRESS 0x18
#define MAXIMUM_PSI     25
#define MINIMUM_PSI     0
#define BUSY_FLAG       0x20
#define INTEGRITY_FLAG  0x04
#define MATH_SAT_FLAG   0x01
#define OUTPUT_MAX      0xE66666
#define OUTPUT_MIN      0x19999A
enum Pressure_Units {PSI, Pa, kPa, torr, inHg, atm, bar};
enum read_phases {idle, requesting, reading};

class SparkFun_MicroPressure {
  public:
    SparkFun_MicroPressure(int8_t eoc_pin=-1, int8_t rst_pin=-1, uint8_t minimumPSI=MINIMUM_PSI, uint8_t maximumPSI=MAXIMUM_PSI);
    bool begin(uint8_t deviceAddress = DEFAULT_ADDRESS, TwoWire &wirePort = Wire);
    uint8_t readStatus(void);
    float readPressure(Pressure_Units units=PSI);
  private:
    bool value_ready = false;
    float pressure = NAN;
    int8_t _address, _eoc, _rst;
    uint8_t _minPsi, _maxPsi, readphase, status;
    TwoWire *_i2cPort;
};
// Constructor and sets default values.
// - (Optional) eoc_pin, End of Conversion indicator. Default: -1 (skip)
// - (Optional) rst_pin, Reset pin for MPR sensor. Default: -1 (skip)
// - minimum/maximum PSI, minimum range value of the sensor (in PSI). Default: 0
// - maximumPSI, maximum range value of the sensor (in pSI). Default: 25
SparkFun_MicroPressure::SparkFun_MicroPressure(int8_t eoc_pin, int8_t rst_pin, uint8_t minimumPSI, uint8_t maximumPSI) {
    _eoc = eoc_pin;
    _rst = rst_pin;
    _minPsi = minimumPSI;
    _maxPsi = maximumPSI;
}
// Initialize hardware
// - deviceAddress, I2C address of the sensor. Default: 0x18
// - wirePort, sets the I2C bus used for communication. Default: Wire
// - Returns 0/1: 0: sensor not found, 1: sensor connected  */
bool SparkFun_MicroPressure::begin(uint8_t deviceAddress, TwoWire &wirePort) {
    _address = deviceAddress;
    _i2cPort = &wirePort;
    if(_eoc != -1) pinMode(_eoc, INPUT);
    if(_rst != -1) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst,LOW);
        delay(5);
        digitalWrite(_rst,HIGH);
        delay(5);
    }
    readphase = idle;
    _i2cPort->beginTransmission(_address);
    return !(_i2cPort->endTransmission());
}
// Read the status byte of the sensor - Returns status byte
uint8_t SparkFun_MicroPressure::readStatus(void) {
    _i2cPort->requestFrom(_address,1);
    return _i2cPort->read();
}
// Read the Pressure Sensor Reading - (optional) Pressure_Units, can return various pressure units. Default: PSI
float SparkFun_MicroPressure::readPressure(Pressure_Units units) {
    if (readphase == idle) {
        _i2cPort->beginTransmission(_address);
        _i2cPort->write((uint8_t)0xAA);
        _i2cPort->write((uint8_t)0x00);
        _i2cPort->write((uint8_t)0x00);
        _i2cPort->endTransmission();
        // if(_eoc != -1) while(!digitalRead(_eoc)) delay(1);  // Wait for new pressure reading available // Use GPIO pin if defined
        // else { // Check status byte if GPIO is not defined
        readphase = requesting;
    }
    if (readphase == requesting) {
        status = readStatus();
        if (status&BUSY_FLAG && (status!=0xFF)) {
            readphase = requesting;
            return pressure;
        }
        readphase = reading;
    }
    _i2cPort->requestFrom(_address,4);
    uint8_t status = _i2cPort->read();
    if((status & INTEGRITY_FLAG) || (status & MATH_SAT_FLAG)) {
        readphase = idle;
        return NAN;
    } //  check memory integrity and math saturation bit
    uint32_t reading = 0;
    for(uint8_t i=0;i<3;i++) {  //  read 24-bit pressure
        reading |= _i2cPort->read();
        if(i != 2) reading = reading<<8;
    }
    pressure = (reading - OUTPUT_MIN) * (_maxPsi - _minPsi);
    pressure = (pressure / (OUTPUT_MAX - OUTPUT_MIN)) + _minPsi;
    if(units == Pa)        pressure *= 6894.7573; //Pa (Pascal)
    else if(units == kPa)  pressure *= 6.89476;   //kPa (kilopascal)
    else if(units == torr) pressure *= 51.7149;   //torr (mmHg)
    else if(units == inHg) pressure *= 2.03602;   //inHg (inch of mercury)
    else if(units == atm)  pressure *= 0.06805;   //atm (atmosphere)
    else if(units == bar)  pressure *= 0.06895;   //bar
    readphase = idle;
    return pressure;
}