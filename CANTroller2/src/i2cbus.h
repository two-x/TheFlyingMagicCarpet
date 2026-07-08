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
                ezread.squintf("  found device addr 0x%s%x : %s\n", (address < 16) ? "0" : "", address, (devindex >= 0) ? i2ccard[devindex].c_str() : "unknown");
                if (_devicecount < 10) _detaddrs[_devicecount++] = address;
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
    void reinit_wire() {  // call after any library (e.g. LovyanGFX touch) that may reinitialize I2C port 0
        Wire.begin(_sda_pin, _scl_pin, i2c_frequency);
        Wire.setTimeOut(25);
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
    bool _verbose = false;
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
    if (_errorflag != MapErrWaiting) {
        // Issue measurement command
        _i2cPort->beginTransmission(_addr);
        _i2cPort->write((uint8_t)0xAA);
        _i2cPort->write((uint8_t)0x00);
        _i2cPort->write((uint8_t)0x00);
        _i2cPort->endTransmission();
        if (!blocking) {
            // Defer read to next call after _mapretry_timeout (>=8ms > 6.4ms max conversion time).
            // Reading status immediately after endTransmission() is a race: the sensor may not have
            // asserted BUSY yet, causing the poll to see "not busy" and read stale/garbage bytes.
            _errorflag = MapErrWaiting;
            return NAN;
        }
        delay(10);  // blocking: wait beyond max conversion time before reading (6.4ms max at 3.3V)
    }
    // Non-blocking 2nd call: at least _mapretry_timeout has elapsed since the command, sensor is done.
    // Blocking: waited 10ms above. In both cases, read 4 bytes directly without status polling.
    _errorflag = MapErrNone;
    _i2cPort->requestFrom(_addr, (uint8_t)4);
    _statusbyte = _i2cPort->read();
    if (_statusbyte & BUSY_FLAG) {  // shouldn't happen, but handle gracefully (retry next call)
        for (int i = 0; i < 3; i++) _i2cPort->read();
        _errorflag = MapErrWaiting;
        return NAN;
    }
    if (_statusbyte & MATH_SAT_FLAG) {
        _errorflag = MapErrMath;
        static bool err_printed = false;
        if (!err_printed && _verbose) {
            ezread.squintf(ezread.sadcolor, "warn: mapsens math sat flag (0x%02X)\n", _statusbyte);
            err_printed = true;
        }
    }
    else if (_statusbyte & INTEGRITY_FLAG) {
        _errorflag = MapErrIntegrity;
        static bool err_printed = false;
        if (!err_printed && _verbose) {
            ezread.squintf(ezread.sadcolor, "warn: mapsens integrity flag (0x%02X)\n", _statusbyte);
            err_printed = true;
        }
    }
    int reading = 0;
    for (int i = 0; i < 3; i++) {
        reading |= _i2cPort->read();
        if (i != 2) reading = reading<<8;
    }
    _pressure = ((float)reading - (float)OUTPUT_MIN) * (_maxPsi - _minPsi);
    _pressure = (_pressure / (OUTPUT_MAX - OUTPUT_MIN)) + _minPsi;
    if (units == MapUnitPA)        _pressure *= 6894.7573f;
    else if (units == MapUnitKPA)  _pressure *= 6.89476f;
    else if (units == MapUnitTORR) _pressure *= 51.7149f;
    else if (units == MapUnitINHG) _pressure *= 2.03602f;
    else if (units == MapUnitATM)  _pressure *= 0.06805f;
    else if (units == MapUnitBAR)  _pressure *= 0.06895f;
    return _pressure;
}

// FS3000 air velocity sensor - inlined from SparkFun_FS3000_Arduino_Library by Pete Lewis (SparkFun, 2021, MIT)
// Inlined here so the fix to readRaw() survives PlatformIO reinstalls (same approach as MicroPressure above).
// Key fix in readRaw(): return 0xFFFF only when count==0 (all-zeros buffer falsely passes checksum).
// Counts 1-408 are below rated minimum but indicate a live sensor; readMetersPerSecond() clamps them to 0.0 mph.
constexpr uint8_t AIRFLOW_RANGE_7_MPS  = 0x00;  // FS3000-1005, 0–7.23 m/s
constexpr uint8_t AIRFLOW_RANGE_15_MPS = 0x01;  // FS3000-1015, 0–15 m/s

class FS3000 {
  public:
    FS3000() {}
    bool begin(TwoWire &wirePort = Wire);
    bool isConnected();
    void setRange(uint8_t range);
    uint16_t readRaw();
    float readMetersPerSecond();
    float readMilesPerHour();
  private:
    static constexpr uint8_t FS3000_DEVICE_ADDRESS = 0x28;
    TwoWire *_i2cPort = &Wire;
    uint8_t _buff[5] = {};
    uint8_t _range = AIRFLOW_RANGE_7_MPS;
    float _mpsDataPoint[13] = {0, 1.07f, 2.01f, 3.00f, 3.97f, 4.96f, 5.98f, 6.99f, 7.23f};
    int   _rawDataPoint[13] = {409, 915, 1522, 2066, 2523, 2908, 3256, 3572, 3686};
    bool readData(uint8_t* buf);
    bool checksum(uint8_t* data);
};
bool FS3000::begin(TwoWire &wirePort) {
    _i2cPort = &wirePort;
    return isConnected();
}
bool FS3000::isConnected() {
    _i2cPort->beginTransmission(FS3000_DEVICE_ADDRESS);
    return (_i2cPort->endTransmission() == 0);
}
void FS3000::setRange(uint8_t range) {
    _range = range;
    const float mps7[9]  = {0, 1.07f, 2.01f, 3.00f, 3.97f, 4.96f, 5.98f, 6.99f, 7.23f};
    const int   raw7[9]  = {409, 915, 1522, 2066, 2523, 2908, 3256, 3572, 3686};
    const float mps15[13] = {0, 2.00f, 3.00f, 4.00f, 5.00f, 6.00f, 7.00f, 8.00f, 9.00f, 10.00f, 11.00f, 13.00f, 15.00f};
    const int   raw15[13] = {409, 1203, 1597, 1908, 2187, 2400, 2629, 2801, 3006, 3178, 3309, 3563, 3686};
    if (_range == AIRFLOW_RANGE_7_MPS)
        for (int i = 0; i < 9;  i++) { _mpsDataPoint[i] = mps7[i];  _rawDataPoint[i] = raw7[i]; }
    else if (_range == AIRFLOW_RANGE_15_MPS)
        for (int i = 0; i < 13; i++) { _mpsDataPoint[i] = mps15[i]; _rawDataPoint[i] = raw15[i]; }
}
bool FS3000::readData(uint8_t* buf) {
    _i2cPort->requestFrom(FS3000_DEVICE_ADDRESS, (uint8_t)5);
    uint8_t i = 0;
    while (_i2cPort->available() && i < 5) buf[i++] = _i2cPort->read();
    while (_i2cPort->available()) _i2cPort->read();
    return (i == 5);
}
bool FS3000::checksum(uint8_t* data) {
    uint8_t sum = 0;
    for (int i = 1; i <= 4; i++) sum += data[i];
    return ((uint8_t)(sum + data[0]) == 0x00);
}
uint16_t FS3000::readRaw() {
    if (!readData(_buff)) return 0xFFFF;
    if (!checksum(_buff)) return 0xFFFF;
    uint16_t airflowRaw = ((_buff[1] & 0x0F) << 8) | _buff[2];
    if (airflowRaw == 0) return 0xFFFF;  // all-zeros passes checksum by coincidence — definitive I2C failure
    return airflowRaw;  // counts 1-408: live sensor below rated minimum; readMetersPerSecond() clamps to 0.0 mph
}
float FS3000::readMetersPerSecond() {
    int airflowRaw = readRaw();
    if (airflowRaw == 0xFFFF) return NAN;
    if (airflowRaw <= 409) return 0.0f;
    if (airflowRaw >= 3686) return (_range == AIRFLOW_RANGE_15_MPS) ? 15.00f : 7.23f;
    int data_position = 0;
    uint8_t dataPointsNum = (_range == AIRFLOW_RANGE_15_MPS) ? 13 : 9;
    for (int i = 0; i < dataPointsNum; i++)
        if (airflowRaw > _rawDataPoint[i]) data_position = i;
    float percentage = (float)(airflowRaw - _rawDataPoint[data_position]) / (float)(_rawDataPoint[data_position+1] - _rawDataPoint[data_position]);
    return _mpsDataPoint[data_position] + percentage * (_mpsDataPoint[data_position+1] - _mpsDataPoint[data_position]);
}
float FS3000::readMilesPerHour() { return readMetersPerSecond() * 2.2369362912f; }

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