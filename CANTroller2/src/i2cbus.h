#pragma once
#include <Wire.h>   // for i2c bus support
enum i2c_nodes { I2CBogus=-1, I2CTouch=0, I2CLightbox=1, I2CAirVelo=2, I2CMAP=3, NumI2CSlaves=4 };  // I2CTouch, 
std::string i2ccard[(int)NumI2CSlaves] = { "touch", "litbox", "airvel", "mapsns" };
uint8_t known_i2c_addr[(int)NumI2CSlaves] = { 0x38, 0x69, 0x28, 0x18 };

class I2C {
  private:
    bool disabled = false;
    int _sda_pin, _scl_pin, _devicecount = 0;
    uint8_t _detaddrs[10];  // addresses detected, unordered
    uint8_t _devaddrs[NumI2CSlaves];  // addresses of known devices, ordered per enum
    bool _detected[NumI2CSlaves];  // detection status of known devices, ordered per enum
    Timer scanTimer;
    void fill_det_array() {
        for (int sl = 0; sl<NumI2CSlaves; sl++) {
            _detected[sl] = false;
            for (int i = 0; i<_devicecount; i++)
                if (_detaddrs[i] == _devaddrs[sl]) _detected[sl] = true;
        }
    }
  public:
    int i2cbaton = I2CLightbox;             // A semaphore mechanism to prevent bus conflict on i2c bus
    I2C(int sda_pin_arg, int scl_pin_arg) : _sda_pin(sda_pin_arg), _scl_pin(scl_pin_arg) {}
    int setup(uint8_t touch_addr, uint8_t lightbox_addr, uint8_t airvelo_addr, uint8_t map_addr) {
        _devaddrs[I2CTouch] = touch_addr;
        _devaddrs[I2CLightbox] = lightbox_addr;
        _devaddrs[I2CAirVelo] = airvelo_addr;
        _devaddrs[I2CMAP] = map_addr;
        ezread.squintf(ezread.highlightcolor, "I2C bus scanning..\n");
        delay(1);  // attempt to force print to happen before init
        scanTimer.reset();
        if (disabled) return 0;
        Wire.begin(_sda_pin, _scl_pin, i2c_frequency);  // I2c bus needed for airflow sensor
        Wire.setTimeOut(25);  // set ms to move on if a device holds SCL for too long. default = 50
        byte error, address;
        _devicecount = 0;
        for (address = 1; address < 127; address++ ) {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            int devindex = -1;
            for (int i=0; i<(int)NumI2CSlaves; i++) if (address == known_i2c_addr[i]) devindex = i;
            if (error == 0) {
                ezread.squintf("  found device addr 0x%s%x : %s\n", (address < 16) ? "0" : "", address, i2ccard[devindex].c_str());
                _detaddrs[_devicecount++] = address;
            }
            else if (error == 4) {
                ezread.squintf(ezread.madcolor, "  error device addr 0x%s%x\n", (address < 16) ? "0" : "", address);
            }
        }
        if (scanTimer.elapsed() > 5000000) ezread.squintf(ezread.madcolor, "  err: i2c timeout & fail bus scan\n");
        if (_devicecount == 0) ezread.squintf(ezread.sadcolor, "  no devices found\n");
        fill_det_array();
        return detected(I2CTouch);
    }
    bool detected_by_addr(uint8_t addr) {  // argument is an i2c address
        for (int i=0; i < _devicecount; i++) if (_detaddrs[i] == addr) return true;
        return false;
    }
    bool detected(int device) {  // argument is one of the enums
        return _detected[device];
    }
    void pass_i2c_baton() {
        static int lastsens = I2CMAP;
        // Serial.printf("%d->", i2cbaton);
        if (!use_i2c_baton) return;
        if (i2cbaton == I2CAirVelo || i2cbaton == I2CMAP) i2cbaton = I2CTouch;
        else if (i2cbaton == I2CTouch) i2cbaton = I2CLightbox;
        else if (i2cbaton == I2CLightbox) {
            if (lastsens == I2CAirVelo) i2cbaton = I2CMAP;
            else i2cbaton = I2CAirVelo;
            lastsens = i2cbaton;
        }
        // Serial.printf("\r-%d-", i2cbaton);
    }
    bool not_my_turn(int checkdev) {
        return (use_i2c_baton && (checkdev != i2cbaton));
    }
};

// map.h - an i2c sensor to track the air pressure in our intake manifold. This, together with the air velocity
// and temperature gives us the mass air flow which is proportional to how hard the engine is working.
// I stole this library and modified it as such. to not block for 6-7ms on each read. - Soren
// SparkFun_MicroPressure library by Alex Wende July 2020 (Beerware license)
// This is a library for the Qwiic MicroPressure Sensor, which can read from 0 to 25 PSI.
enum MapUnits { MapUnitPSI, MapUnitPA, MapUnitKPA, MapUnitTORR, MapUnitINHG, MapUnitATM, MapUnitBAR };  // {PSI, Pa, kPa, torr, inHg, atm, bar};
// enum MapPhases { MapIdle, MapWaiting };
enum MapErrors { MapErrNone=0, MapErrWaiting=1, MapErrIntegrity=2, MapErrMath=3 };

class SparkFun_MicroPressure {
  public:
    SparkFun_MicroPressure(int eoc_pin=-1, int rst_pin=-1, uint8_t minimumPSI=MINIMUM_PSI, uint8_t maximumPSI=MAXIMUM_PSI);
    bool begin(uint8_t deviceAddress=DEFAULT_ADDRESS, TwoWire &wirePort=Wire);    
    uint8_t readStatus(void);
    float readPressure(MapUnits units=MapUnitPSI, bool blocking=true);
    uint8_t get_addr() { return (uint8_t)_addr; }; // WIP debugging
    int err_status() { return _errorflag; };
    float get_pressure() { return _pressure; };
  private:
    bool _respect_integrity_errors = false; // workaround due to constant integrity flag errors on my devboard - TODO debug this!

    static constexpr uint8_t DEFAULT_ADDRESS = 0x18;
    static constexpr int MAXIMUM_PSI = 25;
    static constexpr int MINIMUM_PSI = 0;
    static constexpr uint8_t BUSY_FLAG = 0x20;
    static constexpr uint8_t INTEGRITY_FLAG = 0x04;
    static constexpr uint8_t MATH_SAT_FLAG = 0x01;
    static constexpr uint32_t OUTPUT_MAX = 0xE66666;
    static constexpr uint32_t OUTPUT_MIN = 0x19999A;
    // int _readphase = MapIdle;
    int _eoc, _rst;
    int _errorflag = MapErrNone;
    float _pressure = NAN;
    uint8_t _addr, _minPsi, _maxPsi, _statusbyte;
    TwoWire *_i2cPort;
};
// - (Optional) eoc_pin, End of Conversion indicator. Default: -1 (skip)
// - (Optional) rst_pin, Reset pin for MPR sensor. Default: -1 (skip)
// - minimum/maximum PSI, minimum range value of the sensor (in PSI). Default: 0
// - maximumPSI, maximum range value of the sensor (in pSI). Default: 25
SparkFun_MicroPressure::SparkFun_MicroPressure(int eoc_pin, int rst_pin, uint8_t minimumPSI, uint8_t maximumPSI) {
    _eoc = eoc_pin;
    _rst = rst_pin;
    _minPsi = minimumPSI;
    _maxPsi = maximumPSI;
}
// begin() initialize hardware
// - deviceAddress, I2C address of the sensor. Default: 0x18
// - wirePort, sets the I2C bus used for communication. Default: Wire
// - Returns 0/1: 0: sensor not found, 1: sensor connected  */
bool SparkFun_MicroPressure::begin(uint8_t deviceAddress, TwoWire &wirePort) {
    _addr = deviceAddress;
    _i2cPort = &wirePort;
    if(_eoc != -1) pinMode(_eoc, INPUT);
    if(_rst != -1) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, LOW);
        delay(5);
        digitalWrite(_rst, HIGH);
    }
    delay(15);
    _i2cPort->beginTransmission(_addr);
    uint8_t error = _i2cPort->endTransmission();
    if (error == 0) return true;  // success
    return false; // fail
}
uint8_t SparkFun_MicroPressure::readStatus(void) {
    _i2cPort->requestFrom(_addr, (uint8_t)1); // WIP debugging
    return _i2cPort->read();
}
float SparkFun_MicroPressure::readPressure(MapUnits units, bool blocking) {
    // static Timer read_timer{500000};  // half-second timeout to avoid hanging during failed read // WIP debugging
    if (_errorflag != MapErrWaiting) {
        _i2cPort->beginTransmission(_addr);
        _i2cPort->write((uint8_t)0xAA);
        _i2cPort->write((uint8_t)0x00);
        _i2cPort->write((uint8_t)0x00);
        _i2cPort->endTransmission();
    }
    if (_eoc != -1) { // Use GPIO pin if defined
        while (!digitalRead(_eoc)) {
            if (!blocking) {
                _errorflag = MapErrWaiting;
                return NAN;  // return _pressure;  may be better to return last read good value while waiting for new one
            }
            delay(1);
        }
    }
    else { // Check status byte if GPIO is not defined
        _statusbyte = readStatus();
        while((_statusbyte & BUSY_FLAG) && (_statusbyte != 0xff)) {
            if (!blocking) {
                _errorflag = MapErrWaiting;
                return NAN;  // return _pressure;  may be better to return last read good value while waiting for new one
            }
            delay(1);
            _statusbyte = readStatus();
        }
    }
    _i2cPort->requestFrom(_addr, (uint8_t)4); // WIP debugging
    _statusbyte = _i2cPort->read();

    // if ((_statusbyte & INTEGRITY_FLAG) || (_statusbyte & MATH_SAT_FLAG)) return NAN; //  check memory integrity and math saturation bit
    
    if (_statusbyte & INTEGRITY_FLAG) {
        _errorflag = MapErrIntegrity;
        static bool err_printed = false;
        if (!err_printed) ezread.squintf(ezread.sadcolor, "warn: mapsens integrity flag (0x%02X)\n", _statusbyte);
        err_printed = true;  // print error only once on first read. it will spam otherwise
        // if (_respect_integrity_errors) return NAN;
    }
    else if (_statusbyte & MATH_SAT_FLAG) {
        _errorflag = MapErrMath;
        static bool err_printed = false;
        if (!err_printed) ezread.squintf(ezread.sadcolor, "warn: mapsens math sat flag (0x%02X)\n", _statusbyte);
        err_printed = true;  // print error only once on first read. it will spam otherwise
        // return NAN;  // math sat errors seem fairly rare (unlike integrity errs), so we can skip reading when we get one. TODO debug this!
    }
    else _errorflag = MapErrNone;

    int reading = 0;
    for (int i=0; i<3; i++) {  //  read 24-bit pressure
        reading |= _i2cPort->read();
        if (i != 2) reading = reading<<8;
    }
    _pressure = (reading - OUTPUT_MIN) * (_maxPsi - _minPsi);
    _pressure = (_pressure / (OUTPUT_MAX - OUTPUT_MIN)) + _minPsi;
    if (units == MapUnitPA)        _pressure *= 6894.7573f; //Pa (Pascal)
    else if (units == MapUnitKPA)  _pressure *= 6.89476f;   //kPa (kilopascal)
    else if (units == MapUnitTORR) _pressure *= 51.7149f;   //torr (mmHg)
    else if (units == MapUnitINHG) _pressure *= 2.03602f;   //inHg (inch of mercury)
    else if (units == MapUnitATM)  _pressure *= 0.06805f;   //atm (atmosphere)
    else if (units == MapUnitBAR)  _pressure *= 0.06895f;   //bar
    return _pressure;
}

// LightingBox - object to manage 12c communications link to our lighting box
// Our protocol is: 1st nibble of 1st byte contains 4-bit command/request code. The 2nd nibble and any additional bytes contain data, as required by the code
// code: 0x0F = status flags. F bits: 0=syspower, 1=panicstop, 2=warning, 3=alarm
// code: 0x1R = entered runmode given by R (no additional bytes)
// code: 0x2H,0xLL = speedometer value update. 12-bit value contained in HLL is speed in hundredths-of-mph
class LightingBox {  // represents the lighting controller i2c slave endpoint
  private:
    Timer send_timer{250000};
    int runmode_last = Standby;
    uint16_t speed_last;
    uint8_t status_nibble_last;
    I2C* i2c;
    uint8_t _addr = known_i2c_addr[I2CLightbox];
    // DiagRuntime* diag;
  public:
    LightingBox(I2C* _i2c) : i2c{_i2c} {}  // LightingBox(DiagRuntime* _diag) : diag(_diag) {}
    void update(float speed) {
        bool sent = false;
        if (i2c->not_my_turn(I2CLightbox)) return;
        // if (i2c->detected(I2CLightbox) && send_timer.expireset()) {
        if (send_timer.expireset()) {  // enable for now to test
            sent = sendstatus();  // send status if it changed since last time
            sent |= sendrunmode(runmode);  // send runmode if it changed since last time
            if (!sent) sent = sendspeed(speed);  // if neither of above was sent, then send speed (to prevent long bus use)
        }
        i2c->pass_i2c_baton();
        return;
    }
    void setup() {
        ezread.squintf(ezread.highlightcolor, "Lightbox (i2c 0x%02x) init\n", _addr);  // ezread.squintf("Lighting box serial comm..\n");
    }
    uint8_t get_addr() { return _addr; }
  private:
    bool sendstatus() {
        uint8_t byt = 0x00;  // command template for status update
        uint8_t warning = 0;  // (diag->worst_sensor(3) != _None);
        uint8_t alarm = 0;  // (diag->worst_sensor(4) != _None);
        byt |= (uint8_t)syspower.val() | (uint8_t)(panicstop << 1) | (uint8_t)(warning << 2) | (alarm << 3);  // insert status bits nibble
        if (byt == status_nibble_last) return false;  // bail if no change to status occured
        Wire.beginTransmission(_addr);
        Wire.write(byt);
        Wire.endTransmission(_addr);
        status_nibble_last = byt;
        return true;
    }
    bool sendrunmode(int runmode) {
        uint8_t byt = 0x10;  // command template for runmode update
        if (runmode == runmode_last) return false;  // bail if no change to runmode occured
        byt |= (uint8_t)(runmode & 0x0f);  // insert runmode in 2nd nibble
        Wire.beginTransmission(_addr);
        Wire.write(byt);
        Wire.endTransmission(_addr);
        runmode_last = runmode;
        return true;
    }
    bool sendspeed(float _speed) {
        uint8_t byt = 0x20;  // command template for speed update
        uint16_t speed = (uint16_t)(_speed * 100.0f);
        if (speed == speed_last) return false;  // bail if speed has not changed
        byt |= ((uint8_t)(speed >> 8) & 0x0f);
        Wire.beginTransmission(_addr);
        Wire.write(byt);
        byt = (uint8_t)(speed & 0xff);
        Wire.write(byt);
        Wire.endTransmission();
        speed_last = speed;
        return true;
    }
};