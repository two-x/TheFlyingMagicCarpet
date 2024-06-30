#pragma once
#include <Wire.h>   // for i2c bus support
enum i2c_nodes : int { i2c_touch, i2c_lightbox, i2c_airvelo, i2c_map, num_i2c_slaves };  // i2c_touch, 

class I2C {
  private:
    bool disabled = false;
    int _devicecount = 0;
    uint8_t _detaddrs[10];  // addresses detected, unordered
    uint8_t _devaddrs[num_i2c_slaves];  // addresses of known devices, ordered per enum
    bool _detected[num_i2c_slaves];  // detection status of known devices, ordered per enum
    int _sda_pin, _scl_pin;
    Timer scanTimer;
    int lastsens = i2c_map;
    void fill_det_array() {
        for (int sl = 0; sl<num_i2c_slaves; sl++) {
            _detected[sl] = false;
            for (int i = 0; i<_devicecount; i++)
                if (_detaddrs[i] == _devaddrs[sl]) _detected[sl] = true;
        }
    }
  public:
    int i2cbaton = i2c_lightbox;             // A semaphore mechanism to prevent bus conflict on i2c bus
    I2C(int sda_pin_arg, int scl_pin_arg) : _sda_pin(sda_pin_arg), _scl_pin(scl_pin_arg) {}
    int setup(uint8_t touch_addr, uint8_t lightbox_addr, uint8_t airvelo_addr, uint8_t map_addr) {
        _devaddrs[i2c_touch] = touch_addr;
        _devaddrs[i2c_lightbox] = lightbox_addr;
        _devaddrs[i2c_airvelo] = airvelo_addr;
        _devaddrs[i2c_map] = map_addr;
        Serial.printf("I2C bus"); delay(1);  // Attempt to force print to happen before init
        scanTimer.reset();
        if (disabled) return 0;
        Wire.begin(_sda_pin, _scl_pin, i2c_frequency);  // I2c bus needed for airflow sensor
        byte error, address;
        Serial.printf(" scan..");
        _devicecount = 0;
        for (address = 1; address < 127; address++ ) {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            if (error == 0) {
                Serial.printf (" found addr: 0x%s%x", (address < 16) ? "0" : "", address);
                _detaddrs[_devicecount++] = address;
            }
            else if (error==4) Serial.printf (" error addr: 0x%s%x", (address < 16) ? "0" : "", address);
        }
        if (scanTimer.elapsed() > 5000000) Serial.printf(" timeout & fail bus scan.");
        if (_devicecount == 0) Serial.printf(" no devices found.");
        Serial.printf(" ..done\n");
        fill_det_array();
        return detected(i2c_touch);
    }
    bool detected_by_addr(uint8_t addr) {  // argument is an i2c address
        for (int i=0; i < _devicecount; i++) if (_detaddrs[i] == addr) return true;
        return false;
    }
    bool detected(int device) {  // argument is one of the enums
        return _detected[device];
    }
    void pass_i2c_baton() {
        // Serial.printf("%d->", i2cbaton);
        if (!use_i2c_baton) return;
        if (i2cbaton == i2c_airvelo || i2cbaton == i2c_map) i2cbaton = (captouch) ? i2c_touch : i2c_lightbox; 
        else if (i2cbaton == i2c_touch) i2cbaton = i2c_lightbox;
        else if (i2cbaton == i2c_lightbox) {
            i2cbaton = (lastsens == i2c_airvelo) ? i2c_map : i2c_airvelo;
            lastsens = i2cbaton;
        }
        // Serial.printf("\r-%d-", i2cbaton);
    }
    bool not_my_turn(int checkdev) {
        bool retval = (use_i2c_baton && (checkdev != i2cbaton));
        // Serial.printf("b:%d c:%d nmt:%d\n", i2cbaton, checkdev, retval);
        return retval;       
    }
};

// map.h - an i2c sensor to track the air pressure in our intake manifold. This, together with the air velocity
// and temperature gives us the mass air flow which is proportional to how hard the engine is working.
// I stole this library and modified it as such. to not block for 6-7ms on each read. - Soren
// SparkFun_MicroPressure library by Alex Wende July 2020 (Beerware license)
// This is a library for the Qwiic MicroPressure Sensor, which can read from 0 to 25 PSI.
enum Pressure_Units {PSI, PA, KPA, TORR, INHG, ATM, BAR};  // {PSI, Pa, kPa, torr, inHg, atm, bar};

class SparkFun_MicroPressure {
  public:
    static constexpr uint8_t addr = 0x18;
    SparkFun_MicroPressure(int8_t eoc_pin=-1, int8_t rst_pin=-1, uint8_t minimumPSI=MINIMUM_PSI, uint8_t maximumPSI=MAXIMUM_PSI);
    bool begin(uint8_t deviceAddress = addr, TwoWire &wirePort = Wire);
    uint8_t readStatus(void);
    float readPressure(Pressure_Units units=PSI, bool noblock=false);
    uint8_t get_addr();
  private:
    static constexpr int MAXIMUM_PSI = 25;
    static constexpr int MINIMUM_PSI = 0;
    static constexpr uint8_t BUSY_FLAG = 0x20;
    static constexpr uint8_t INTEGRITY_FLAG = 0x04;
    static constexpr uint8_t MATH_SAT_FLAG = 0x01;
    static constexpr uint32_t OUTPUT_MAX = 0xE66666;
    static constexpr uint32_t OUTPUT_MIN = 0x19999A;
    bool run_preamble = true;
    float pressure = NAN;
    int8_t _address, _eoc, _rst;
    uint8_t _minPsi, _maxPsi, _status;
    TwoWire *_i2cPort = &Wire;
};
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
    run_preamble = true;
    _i2cPort->beginTransmission(_address);
    //return !(_i2cPort->endTransmission());
    uint8_t error = _i2cPort->endTransmission();
    if(error == 0) return true;
    else           return false;
}
uint8_t SparkFun_MicroPressure::readStatus(void) {
    _i2cPort->requestFrom(_address,1);
    return _i2cPort->read();
}
float SparkFun_MicroPressure::readPressure(Pressure_Units units, bool noblock) {
    if (run_preamble) {
        _i2cPort->beginTransmission(_address);
        _i2cPort->write((uint8_t)0xAA);
        _i2cPort->write((uint8_t)0x00);
        _i2cPort->write((uint8_t)0x00);
        _i2cPort->endTransmission();
    }
    run_preamble = false;
    if (_eoc != -1) { // Use GPIO pin if defined
        while (!digitalRead(_eoc)) {
            if (noblock) return NAN;
            delay(1);
        }
    }
    else { // Check status byte if GPIO is not defined
        _status = readStatus();
        while((_status&BUSY_FLAG) && (_status!=0xFF)) {
            if (noblock) return NAN;
            delay(1);
            _status = readStatus();
        }
    }
    run_preamble = true;
    _i2cPort->requestFrom(_address,4);
    _status = _i2cPort->read();
    if ((_status & INTEGRITY_FLAG) || (_status & MATH_SAT_FLAG)) return NAN; //  check memory integrity and math saturation bit
    int reading = 0;
    for (int i=0; i<3; i++) {  //  read 24-bit pressure
        reading |= _i2cPort->read();
        if (i != 2) reading = reading<<8;
    }
    pressure = (reading - OUTPUT_MIN) * (_maxPsi - _minPsi);
    pressure = (pressure / (OUTPUT_MAX - OUTPUT_MIN)) + _minPsi;
    if (units == PA)        pressure *= 6894.7573; //Pa (Pascal)
    else if (units == KPA)  pressure *= 6.89476;   //kPa (kilopascal)
    else if (units == TORR) pressure *= 51.7149;   //torr (mmHg)
    else if (units == INHG) pressure *= 2.03602;   //inHg (inch of mercury)
    else if (units == ATM)  pressure *= 0.06805;   //atm (atmosphere)
    else if (units == BAR)  pressure *= 0.06895;   //bar
    return pressure;
}

// LightingBox - object to manage 12c communications link to our lighting box
// Our protocol is: 1st nibble of 1st byte contains 4-bit command/request code. The 2nd nibble and any additional bytes contain data, as required by the code
// code: 0x0F = status flags. F bits: 0=syspower, 1=panicstop, 2=warning, 3=alarm
// code: 0x1R = entered runmode given by R (no additional bytes)
// code: 0x2H,0xLL = speedometer value update. 12-bit value contained in HLL is speed in hundredths-of-mph
class LightingBox {  // represents the lighting controller i2c slave endpoint
  private:
    Timer send_timer{250000};
    int runmode_last = STANDBY;
    uint16_t speed_last;
    uint8_t status_nibble_last;
    I2C* i2c;
    // DiagRuntime* diag;
  public:
    static constexpr uint8_t addr = 0x69;
    LightingBox(I2C* _i2c) : i2c{_i2c} {}  // LightingBox(DiagRuntime* _diag) : diag(_diag) {}
    void setup() {
        Serial.printf("Lighting box serial comm..\n");
    }
    bool sendstatus() {
        uint8_t byt = 0x00;  // command template for status update
        uint8_t warning = 0;  // (diag->worst_sensor(3) != _None);
        uint8_t alarm = 0;  // (diag->worst_sensor(4) != _None);
        byt |= (uint8_t)syspower | (uint8_t)(panicstop << 1) | (uint8_t)(warning << 2) | (alarm << 3);  // insert status bits nibble
        if (byt == status_nibble_last) return false;
        Wire.beginTransmission(addr);
        Wire.write(byt);
        Wire.endTransmission(addr);
        status_nibble_last = byt;
        return true;
    }
    bool sendrunmode(int runmode) {
        uint8_t byt = 0x10;  // command template for runmode update
        if (runmode == runmode_last) return false;
        byt |= (uint8_t)(runmode & 0x0f);  // insert runmode in 2nd nibble
        Wire.beginTransmission(addr);
        Wire.write(byt);
        Wire.endTransmission(addr);
        runmode_last = runmode;
        return true;
    }
    bool sendspeed(float _speed) {
        uint8_t byt = 0x20;  // command template for speed update
        uint16_t speed = (uint16_t)(_speed * 100);
        if (speed == speed_last) return false;
        byt |= ((speed >> 8) & 0x0f);
        Wire.beginTransmission(addr);
        Wire.write(byt);
        byt = (uint8_t)(speed & 0xff);
        Wire.write(byt);
        Wire.endTransmission();
        speed_last = speed;
        return true;
    }
    uint8_t get_addr() { return addr; }
    void update(int runmode, float speed) {
        bool sent = false;
        if (i2c->not_my_turn(i2c_lightbox)) return;
        // if (i2c->detected(i2c_lightbox) && send_timer.expireset()) {
        if (send_timer.expireset()) {  // enable for now to test
            sent = sendstatus();
            sent |= sendrunmode(runmode);
            if (!sent) sent = sendspeed(speed);
        }
        i2c->pass_i2c_baton();
    }
};