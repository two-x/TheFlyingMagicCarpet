#pragma once
#include <iostream>
#include <map>
#include <memory> // for std::shared_ptr
#include <SparkFun_FS3000_Arduino_Library.h>  // For air velocity sensor  http://librarymanager/All#SparkFun_FS3000
#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include "driver/rmt.h"
#include <ESP32Servo.h>        // Makes PWM output to control motors (for rudimentary control of our gas and steering)
// #include "uictrl.h"

// This enum class represent the components which can be simulated (sensor). It's a uint8_t type under the covers, so it can be used as an index
// typedef uint8_t opt_t;
enum class sens : int { none=0, joy=1, pressure=2, brkpos=3, speedo=4, tach=5, airvelo=6, mapsens=7, engtemp=8, mulebatt=9, starter=10, basicsw=11, NUM_SENSORS=12 };  //, ignition, syspower };  // , NUM_SENSORS, err_flag };
enum class src : int { UNDEF=0, FIXED=1, PIN=2, TOUCH=3, POT=4, CALC=5 };

int sources[static_cast<int>(sens::NUM_SENSORS)] = { static_cast<int>(src::UNDEF) };

#include "i2cbus.h"
// #include "xtensa/core-macros.h"  // access to ccount register for esp32 timing ticks

// NOTE: the following classes all contain their own initial config values (for simplicity). We could instead use Config objects and pass them in at construction time, which might be
//       a little cleaner organization-wise, since all the configs would be consolidated. It would also allow us to read Configs from storage somewhere, so we could have persistent
//       calibration.

// Param is a value which is constrained between min/max limits, representing a "raw" (aka unfiltered) quantity. A value with tight limits
// (wehere min=val=max) is constant and cannot be changed without changing the limits. An unconstrained value can be represented by setting
// either or both min/max to infinity.
class Potentiometer {
  protected:
    float adc_min = 300; // TUNED 230603 - Used only in determining theconversion factor
    float adc_max = 4095; // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
    float _ema_alpha = 0.35;
    float _pc_min = 0.0;
    float _pc_max = 100.0;
    float _pc_activity_margin = 2.0;
    uint8_t _pin;
    float _val = 0.0, _activity_ref;
  public:
    Potentiometer(uint8_t arg_pin) : _pin(arg_pin) {}
    Potentiometer() = delete; // must have a pin defined
    sens senstype = sens::none;
    void setup() {
        printf("Pot setup..\n");
        set_pin(_pin, INPUT);
        _activity_ref = _val;
    }
    void update() {
        float new_val = map(static_cast<float>(analogRead(_pin)), adc_min, adc_max, _pc_min, _pc_max);
        new_val = constrain(new_val, _pc_min, _pc_max); // the lower limit of the adc reading isn't steady (it will dip below zero) so constrain it back in range
        _val = ema_filt(new_val, _val, _ema_alpha);
        if (std::abs(_val - _activity_ref) > _pc_activity_margin) {
            kick_inactivity_timer(7);  // evidence of user activity
            _activity_ref = _val;
        }
    }
    template<typename VAL_T>
    VAL_T mapToRange(VAL_T min, VAL_T max) {
        return static_cast<VAL_T>(map(_val, _pc_min, _pc_max, static_cast<float>(min), static_cast<float>(max)));
    }
    float val() { return _val; }
    float min() { return _pc_min; }
    float max() { return _pc_max; }
};
template<typename VALUE_T>
class Param {
  protected:
    std::shared_ptr<VALUE_T> _val, _min, _max;
    VALUE_T _last; // keep track of the last value (still needed?)
    bool _saturated = false; // keeps track of whether the value has been constrained by the limits or not
    // NOTE: add centerpoint...?

    void constrain_value() {
        _saturated = false;
        if (*_val < *_min) {
            *_val = *_min;
            _saturated = true;  // Constraint was necessary
        }
        else if (*_val > *_max) {
            *_val = *_max;
            _saturated = true;  // Constraint was necessary
        }
    }
    
  public:
    String _long_name = "Unnamed value";
    String _short_name = "noname";

    // Creates a constant Param with the default value for VALUE_T
    // NOTE: this is really only needed for initalization cases where we don't have a valid starting value when we first make the Param
    Param(){
        _val = std::make_shared<VALUE_T>();
        _min = std::make_shared<VALUE_T>();
        _max = std::make_shared<VALUE_T>();
        _last = *_val;
    }
    // Creates a constant Param (with min/max == val)
    Param(VALUE_T arg_val) {
        _val = std::make_shared<VALUE_T>(arg_val);
        _min = std::make_shared<VALUE_T>(arg_val);
        _max = std::make_shared<VALUE_T>(arg_val);
        _last = *_val;
    } 
    // Creates a regular constrained Param
    Param(VALUE_T arg_val, VALUE_T arg_min, VALUE_T arg_max) {
        _val = std::make_shared<VALUE_T>(arg_val);
        _min = std::make_shared<VALUE_T>();
        _max = std::make_shared<VALUE_T>();
        set_limits(arg_min, arg_max);
        _last = *_val;
    } 
    // Creates a constraned Param which uses external limits.
    // NOTE: if using external limits, it's (currently) possible to get stale values, since there is no
    //       callback mechanism in place. We could get around this by calling constrain_value() on every
    //       get() call, but that seems like overkill...
    Param(VALUE_T arg_val, std::shared_ptr<VALUE_T> arg_min_ptr, std::shared_ptr<VALUE_T> arg_max_ptr) {
        _val = std::make_shared<VALUE_T>(arg_val);
        _min = std::make_shared<VALUE_T>();
        _max = std::make_shared<VALUE_T>();
        set_limits(arg_min_ptr, arg_max_ptr);
        _last = *_val;
    } 

    void set_limits(VALUE_T arg_min, VALUE_T arg_max) {  // Use if min/max are kept in-class
        if (arg_min > arg_max)
            printf("Error: min is >= max\n");
        else {
            *_min = arg_min;
            *_max = arg_max;
            constrain_value();
        }
    }
    void set_limits(std::shared_ptr<VALUE_T> arg_min_ptr, std::shared_ptr<VALUE_T> arg_max_ptr) { // Use if min/max are external
        if (arg_min_ptr.get() > arg_max_ptr.get())
            printf("Error: *min is > *max\n");
        else {
            _min = arg_min_ptr;
            _max = arg_max_ptr;
            constrain_value();
        }
    }

    // return value indicates if the value actually changed or not
    bool set(VALUE_T arg_val) {
        _last = *_val;
        *_val = arg_val;
        constrain_value();
        if (*_val != _last) {
            return true;
        }
        return false;
    }

    bool add(VALUE_T arg_add) {
        return set(*_val + arg_add);
    }
    // Getter functions
    VALUE_T val() { return *_val; }
    VALUE_T min() { return *_min; }
    VALUE_T max() { return *_max; }
    VALUE_T* ptr() { return _val.get(); }
    VALUE_T* min_ptr() { return _min.get(); }
    VALUE_T* max_ptr() { return _max.get(); }
    std::shared_ptr<VALUE_T> shptr() { return _val; }
    std::shared_ptr<VALUE_T> min_shptr() { return _min; }
    std::shared_ptr<VALUE_T> max_shptr() { return _max; }
    VALUE_T last() { return _last; } // NOTE: currently unused, do we still need this for drawing purposes?
    bool saturated() { return _saturated; }
};

// Device class - is a base class for any connected system device or signal associated with a pin
class Device {
  protected:
    // Which types of sources are possible for this device?
    uint8_t _pin;
    bool _enabled = true;
    Potentiometer* _pot; // to pull input from the pot if we're in simulation mode
    src _source = src::UNDEF;
    bool _can_source[6] = { true, true, false, true, false, false };
                        // UNDEF, FIXED,  PIN, TOUCH,  POT,  CALC
    // source handling functions (should be overridden in child classes as needed)
    virtual void set_val_from_undef() {}
    virtual void set_val_from_fixed() {}
    virtual void set_val_from_pin() {}
    virtual void set_val_from_touch() {}
    virtual void set_val_from_pot() {}
    virtual void set_val_from_calc() {}
    virtual void set_val_common() {}  // Runs when setting val from any source, after one of the above
    virtual void update_source() {}
  public:
    Timer timer;  // Can be used for external purposes
    String _long_name = "Unknown device";
    String _short_name = "device";
    sens senstype = sens::none;
    Device() = delete; // should always be created with a pin
    // NOTE: should we start in PIN mode?
    Device(uint8_t arg_pin) : _pin(arg_pin) {}
    bool can_source(src arg_source) { return _can_source[static_cast<uint8_t>(arg_source)]; }
    bool set_source(src arg_source) {
        if (_can_source[static_cast<uint8_t>(arg_source)]) {
            _source = arg_source;
            update_source();
            sources[static_cast<int>(senstype)] = static_cast<int>(arg_source);
            return true;
        } 
        return false;
    }

    void update() {
        if (!_enabled) return; // do nothing if the Device is disabled
        switch (_source) {
            case src::UNDEF:
                set_val_from_undef();
                break;
            case src::FIXED:
                set_val_from_fixed();
                break;
            case src::PIN:
                set_val_from_pin();
                break;
            case src::TOUCH:
                set_val_from_touch();
                break;
            case src::POT:
                set_val_from_pot();
                break;
            case src::CALC:
                set_val_from_calc();
                break;
            default:
                // should never get here
                printf("invalid Device source: %d\n", _source);
        }
        set_val_common();
    }
    void attach_pot(Potentiometer &pot_arg) {
        _pot = &pot_arg;
    }
    // Setters and getters
    void set_enabled(bool arg_enable) { _enabled = arg_enable; }
    void set_can_source(src arg_source, bool is_possible) { _can_source[static_cast<uint8_t>(arg_source)] = is_possible; }
    src source() { return _source; }
    uint8_t pin() { return _pin; }
    bool enabled() { return _enabled; }
};

// Device::Transducer is a base class for any system devices that convert real-world values <--> signals in either direction. It has a "native"
// value which represents the sensed or driven hardware input/output. It also has a "human" value which represents the logical or human-readable
// equivalent value. Adjusting either value will automatically change the other one.
enum class TransducerDirection : uint8_t {REV, FWD}; // possible dir values. REV means native sensed value has the opposite polarity of the real world effect (for example, if we sense fewer us per rotation, the engine is going faster)
template<typename NATIVE_T, typename HUMAN_T>
class Transducer : public Device {
  private:
    String _long_name = "Unknown transducer";
    String _short_name = "xducer";
  protected:
    // Multiplier and adder values to plug in for unit conversion math
    NATIVE_T _val_raw;  // Keep track of the most recent unfiltered and unconstrained native value, for monitoring and diag purposes
    float _m_factor = 1.0;
    float _b_offset = 0.0;
    float _zerovalue = NAN;  // value to return from to_native conversion when val is zero (needed for speedometer)
    bool _invert = false;  // Flag to indicated if unit conversion math should multiply or divide
    TransducerDirection dir = TransducerDirection::FWD; // NOTE: what is this for, exactly?
    
    // conversion functions (can be overridden in child classes different conversion methods are needed)
    virtual HUMAN_T from_native(NATIVE_T arg_val_native) {
        float arg_val_f = static_cast<float>(arg_val_native); // convert everything to floats so we don't introduce rounding errors
        float min_f = static_cast<float>(_native.min());
        float max_f = static_cast<float>(_native.max());
        float ret = -1;
        if (!_invert) {
            if (dir == TransducerDirection::REV) {
                ret = min_f + max_f - _b_offset - _m_factor * arg_val_f;
            }
            else ret = _b_offset + _m_factor * arg_val_f;
        } else if (std::abs(arg_val_f) > 0.000001) { // NOTE: isn't 0.0 a valid value tho?  Soren: Only if the pulley can rotate all the way around in under 1 planck time
            if (dir == TransducerDirection::REV) {
                ret = min_f + max_f - _b_offset - _m_factor / arg_val_f;
            }
            else ret = _b_offset + _m_factor / arg_val_f;
        } else {
            printf ("err: from_native conversion div/zero attempt. max=%5.2f, d=%d, i=%d, v=%5.2f, m=%5.2f, b=%5.2f\n", max_f, dir, _invert, arg_val_f, _m_factor, _b_offset);
            ret = max_f;  // Best return given division would be infinite
        }
        return static_cast<HUMAN_T>(ret);
    }

    virtual NATIVE_T to_native(HUMAN_T arg_val_human) {
        float arg_val_f = static_cast<float>(arg_val_human); // convert everything to floats so we don't introduce rounding errors
        float min_f = static_cast<float>(_human.min());
        float max_f = static_cast<float>(_human.max());
        float ret = -1;
        if (dir == TransducerDirection::REV) {
            arg_val_f = min_f + max_f - arg_val_f;
        }
        if (_invert && std::abs(arg_val_f - _b_offset) > 0.000001) {  // trying to dodge risk of floating point error comparing near-zero values
            ret = _m_factor / (arg_val_f - _b_offset);
        } else if (!_invert && std::abs(_m_factor) > 0.000001) {  // risk here of floating point error comparing near-zero values
            ret = (arg_val_f - _b_offset) / _m_factor;
        } else if (!std::isnan(_zerovalue)) {
            ret = _zerovalue;
        } else {
            printf ("err: to_native conversion div/zero attempt. max=%5.2f, d=%d, i=%d, v=%5.2f, m=%5.2f, b=%5.2f\n", max_f, dir, _invert, arg_val_f, _m_factor, _b_offset);
            ret = max_f;  // Best return given division would be infinite
        }
        return static_cast<NATIVE_T>(ret);
    }

    // NOTE: do we really need two values? or should this just be a single value and get converted wherever needed?
    // To hold val/min/max display values in display units (like V, mph, etc.)
    Param<HUMAN_T> _human;
    Param<NATIVE_T> _native;
    
    // override these in children that need to react to limits changing
    virtual void update_native_limits() {}
    virtual void update_human_limits() {}
  public:
    Transducer(uint8_t arg_pin) : Device(arg_pin) {}
    Transducer() = delete;

    void set_native_limits(Param<NATIVE_T> &arg_min, Param<NATIVE_T> &arg_max) {
        if (arg_min.val() > arg_max.val()) {
            dir = TransducerDirection::REV;
            _native.set_limits(arg_max.val(), arg_min.val());
        }
        else {
            dir = TransducerDirection::FWD;
            _native.set_limits(arg_min.val(), arg_max.val());
        }
        update_native_limits();

    }
    void set_human_limits(Param<HUMAN_T> &arg_min, Param<HUMAN_T> &arg_max) {
        if (arg_min.val() > arg_max.val()) {
            dir = TransducerDirection::REV;
            _human.set_limits(arg_max.val(), arg_min.val());
        }
        else {
            dir = TransducerDirection::FWD;
            _human.set_limits(arg_min.val(), arg_max.val());
        }
        update_human_limits();
    }
    void set_native_limits(NATIVE_T arg_min, NATIVE_T arg_max) {
        if (arg_min > arg_max) {
            dir = TransducerDirection::REV;
            _native.set_limits(arg_max, arg_min);
        }
        else {
            dir = TransducerDirection::FWD;
            _native.set_limits(arg_min, arg_max);
        }
        update_native_limits();
        // Need to set human limits here.
        // HUMAN_T human_min = from_native(_native.min());
        // HUMAN_T human_max = from_native(_native.max());
    }
    void set_human_limits(HUMAN_T arg_min, HUMAN_T arg_max) {
        if (arg_min > arg_max) {
            dir = TransducerDirection::REV;
            _human.set_limits(arg_max, arg_min);
        }
        else {
            dir = TransducerDirection::FWD;
            _human.set_limits(arg_min, arg_max);
        }
        update_human_limits();
        // need to set native limits here
        // _human.set_native_limits(from_human(_human.min()), from_human(_human.max()));

    }

    bool set_native(NATIVE_T arg_val_native) {
        _val_raw = arg_val_native;
        if (_native.set(arg_val_native)) {
            _human.set(from_native(_native.val()));
            return true;
        }
        return false;
    }
    bool add_native(NATIVE_T arg_add_native) {
        _val_raw += arg_add_native;
        if (_native.add(arg_add_native)) {
            _human.set(from_native(_native.val()));
            return true;
        }
        return false;
    }
    bool set_human(HUMAN_T arg_val_human) {
        _val_raw = to_native(arg_val_human);
        if (_human.set(arg_val_human)) {
            _native.set(to_native(_human.val()));
            return true;
        }
        return false;
    }
    bool add_human(HUMAN_T arg_add_human) {
        HUMAN_T delta = (HUMAN_T)(arg_add_human * tuning_rate_pcps * loop_avg_us * (max_human() - min_human()) / (100.0 * 1000000));
        _val_raw += to_native(delta);
        if (_human.add(delta)) {
            _native.set(to_native(_human.val()));
            return true;
        }
        return false;
    }

    // Convert units from base numerical value to disp units:  val_native = m-factor*val_numeric + offset  -or-  val_native = m-factor/val_numeric + offset  where m-factor, b-offset, invert are set here
    void set_convert(float arg_m_factor, float arg_b_offset, bool arg_invert) {
        _m_factor = arg_m_factor;
        _b_offset = arg_b_offset;
        _invert = arg_invert;
    }
    // Getter functions
    NATIVE_T native() { return _native.val(); }
    HUMAN_T human() { return _human.val(); }
    NATIVE_T raw() { return _val_raw; }  // This is a native unit value, unconstrained and unfiltered
    NATIVE_T min_native() { return _native.min(); }
    NATIVE_T max_native() { return _native.max(); }
    HUMAN_T min_human() { return _human.min(); }
    HUMAN_T max_human() { return _human.max(); }
    HUMAN_T* min_human_ptr() { return _human.min_ptr(); }
    HUMAN_T* max_human_ptr() { return _human.max_ptr(); }
    NATIVE_T* native_ptr() { return _native.ptr(); }
    HUMAN_T* human_ptr() { return _human.ptr(); }
    std::shared_ptr<NATIVE_T> native_shptr() { return _native.shptr(); }
    std::shared_ptr<HUMAN_T> human_shptr() { return _human.shptr(); }
};

// Sensor class - is a base class for control system sensors, ie anything that measures real world data or electrical signals 
template<typename NATIVE_T, typename HUMAN_T>
class Sensor : public Transducer<NATIVE_T, HUMAN_T> {
  private:
    String _long_name = "Unknown sensor";
    String _short_name = "sensor";
  protected:
    float _ema_alpha = 0.1;
    Param<HUMAN_T> _val_filt;
    bool _should_filter = false;

    void calculate_ema() { // Exponential Moving Average
        if (_should_filter) {
            float cur_val = static_cast<float>(this->_human.val());
            float filt_val = static_cast<float>(_val_filt.val());
            _val_filt.set(ema_filt(cur_val, filt_val, _ema_alpha));
        } else {
            _val_filt.set(this->_human.val());
            _should_filter = true;
        }
    }

    virtual void set_val_from_touch() {
        this->_val_filt.set(this->_human.val());
    }
    virtual void set_val_from_pot() {
        this->_human.set(this->_pot->mapToRange(this->_human.min(), this->_human.max()));
        this->_val_raw = this->_native.val();  // Arguably pot should set the raw value and let the filter work normally, instead of this
        this->_val_filt.set(this->_human.val()); // don't filter the value we get from the pot, the pot output is already filtered
    }

    virtual void update_human_limits() { _val_filt.set_limits(this->_human.min_shptr(), this->_human.max_shptr()); } // make sure our filtered value has the same limits as our regular value
    virtual void update_source() { if (this->_source == src::PIN) _should_filter = false; } // if we just switched to pin input, the old filtered value is not valid

  public:
    Sensor(uint8_t pin) : Transducer<NATIVE_T, HUMAN_T>(pin) {}  
    void set_ema_alpha(float arg_alpha) { _ema_alpha = arg_alpha; }
    float ema_alpha() { return _ema_alpha; }
    HUMAN_T filt() { return _val_filt.val(); }
    HUMAN_T* filt_ptr() { return _val_filt.ptr(); }
    std::shared_ptr<HUMAN_T> filt_shptr() { return _val_filt.shptr(); } // NOTE: should just be public?
};

// Base class for sensors which communicate using i2c.
// NOTE: this is a strange type of Sensor, it's not really a Transducer but it does do filtering. maybe we should rethink the hierarchy a little?
//       I think we can move Param to common.h and add a class ExponentialMovingAverage there as well that just has the ema functionality, then make
//       I2CSensor a child of -> Device, ExponentialMovingAverage and not a Sensor at all.
class I2CSensor : public Sensor<float,float> {
  protected:
    bool _detected = false;
    I2C* _i2c;

    // implement in child classes using the appropriate i2c sensor
    virtual float read_sensor() = 0;
    virtual void set_val_from_pin() {
        this->set_native(read_sensor());
        calculate_ema();  // Sensor EMA filter
    }

    virtual void set_val_from_pot() {
        this->_human.set(this->_pot->mapToRange(this->_human.min(), this->_human.max()));
        this->_val_raw = this->_native.val();
        this->_val_filt.set(this->_human.val()); // don't filter the value we get from the pot, the pot output is already filtered
    }

    virtual void update_human_limits() {
        _native.set_limits(_human.min_shptr(), _human.max_shptr());  // Our two i2c sensors (airvelo & MAP) don't reveal low-level readings, so we only get human units
        _val_filt.set_limits(_human.min_shptr(), _human.max_shptr());
    }
  public:
    uint8_t addr;
    I2CSensor(I2C* i2c_arg, uint8_t i2c_address_arg) : Sensor<float,float>(-1), _i2c(i2c_arg), addr(i2c_address_arg) { set_can_source(src::PIN, true); }
    I2CSensor() = delete;
    String _long_name = "Unknown I2C device";
    String _short_name = "i2cdev";
    virtual void setup() {
        _detected = _i2c->detected_by_addr(addr);
        set_source(src::PIN); // we aren't actually reading from a pin but the point is the same...
    }
};

// AirVeloSensor measures the air intake into the engine in MPH. It communicates with the external sensor using i2c.
class AirVeloSensor : public I2CSensor {
  protected:
    // NOTE: would all AirVeloSensors have the same address? how does this get determined?
    float _min_mph = 0.0;
    float _abs_max_mph = 33.55; // Sensor maximum mph reading.  Our sensor mounted in 2-in ID intake tube
    float _initial_max_mph = 28.5;  // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * ((2 * 2.54) / 2)^2) 1/cm2 * 1/160934 mi/cm = 28.5 mi/hr (mph)            // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * (2.85 / 2)^2) 1/cm2 * 1/160934 mi/cm = 90.58 mi/hr (mph) (?!)  
    float _initial_airvelo_mph = 0.0;
    float _initial_ema_alpha = 0.2;
    FS3000 _sensor;
    float goodreading = NAN;
    int64_t airvelo_read_period_us = 35000;
    Timer airveloTimer;
    virtual float read_sensor() {
        if (!_i2c->detected(i2c_airvelo)) return NAN;
        else if (_i2c->not_my_turn(i2c_airvelo)) return goodreading;
        else if (airveloTimer.expireset()) goodreading = _sensor.readMilesPerHour();  // note, this returns a float from 0-33.55 for the FS3000-1015 
        return goodreading;
    }
  public:
    static constexpr uint8_t addr = 0x28;
    sens senstype = sens::airvelo;
    AirVeloSensor(I2C* i2c_arg) : I2CSensor(i2c_arg, addr) {
        _ema_alpha = _initial_ema_alpha;
        set_human_limits(_min_mph, _initial_max_mph);
        set_can_source(src::POT, true);
        airveloTimer.set(airvelo_read_period_us);
    }
    AirVeloSensor() = delete;
    String _long_name = "Air velocity sensor";
    String _short_name = "airvel";

    virtual void set_val_common() {
        if (_i2c->i2cbaton == i2c_airvelo) _i2c->pass_i2c_baton();
    }
    void setup() {
        _m_factor = 1.0;
        _b_offset = 0.0;
        printf("%s..", this->_long_name.c_str());
        I2CSensor::setup();
        printf(" %sdetected", _detected ? "" : "not ");
        if (_detected) {
            if (_sensor.begin() == false) {
                printf(", not responding\n");  // Begin communication with air flow sensor) over I2C 
                set_source(src::FIXED); // sensor is detected but not working, leave it in an error state ('fixed' as in not changing)
            } else {
                _sensor.setRange(AIRFLOW_RANGE_15_MPS);
                printf (" and responding properly\n");
            }
        } else {
            printf("\n");
            set_source(src::UNDEF); // don't even have a device at all...
        }
    }
    // Getter functions
    float min_mph() { return _human.min(); }
    float max_mph() { return _human.max(); }
    float abs_max_mph() { return _abs_max_mph; }
    float* max_mph_ptr() { return _human.max_ptr(); }
    std::shared_ptr<float> max_mph_shptr() { return _human.max_shptr(); }
};

// MAPSensor measures the air pressure of the engine manifold in PSI. It communicates with the external sensor using i2c.
class MAPSensor : public I2CSensor {
  protected:
    // NOTE: would all MAPSensors have the same address? how does this get determined?
    float _abs_min_atm = 0.06;  // Sensor min
    float _abs_max_atm = 2.46;  // Sensor max
    float _initial_min_atm = 0.68;  // Typical low map for a car is 10.8 psi = 22 inHg, 1 psi = 0.068 atm
    float _initial_max_atm = 1.02;
    float _initial_atm = 1.00;  // 1 atm = 14.6959 psi
    float _initial_ema_alpha = 0.2;
    Timer map_read_timer;
    uint32_t map_read_timeout = 100000, map_retry_timeout = 10000;
    float goodreading = NAN;
    SparkFun_MicroPressure _sensor;
    virtual float read_sensor() {
        if (!_i2c->detected(i2c_map)) return NAN;
        else if (_i2c->not_my_turn(i2c_map)) return goodreading;
        else if (map_read_timer.expired()) {
            float temp = _sensor.readPressure(ATM, true);  // _sensor.readPressure(PSI);  // <- blocking version takes 6.5ms to read
            if (!std::isnan(temp)) {
                goodreading = temp;
                map_read_timer.set(map_read_timeout);
            }
            else map_read_timer.set(map_retry_timeout);
            // Serial.printf("av:%f\n", goodreading);
            // this->_val_raw = this->human_val();  // (NATIVE_T)goodreading; // note, this returns a float from 0-33.55 for the FS3000-1015             
        }
        return goodreading;
    }
  public:
    static constexpr uint8_t addr = 0x18;
    sens senstype = sens::mapsens;
    MAPSensor(I2C* i2c_arg) : I2CSensor(i2c_arg, addr) {
        _ema_alpha = _initial_ema_alpha;
        set_human_limits(_initial_min_atm, _initial_max_atm);
        set_can_source(src::POT, true);
        map_read_timer.set(map_read_timeout);
    }
    MAPSensor() = delete;
    String _long_name = "Manifold Air Pressure sensor";
    String _short_name = "map";

    virtual void set_val_common() {
        if (_i2c->i2cbaton == i2c_map) _i2c->pass_i2c_baton();
    }

    void setup() {
        _m_factor = 1.0;
        _b_offset = 0.0;
        printf("%s..", this->_long_name.c_str());
        I2CSensor::setup();
        printf(" %sdetected", _detected ? "" : "not ");
        if (_detected) {
            if (_sensor.begin() == false) {
                printf(", not responding\n");  // Begin communication with air flow sensor) over I2C 
                set_source(src::FIXED); // sensor is detected but not working, leave it in an error state ('fixed' as in not changing)
            } else {
                printf(" and reading %f atm\n", _sensor.readPressure(ATM));
                // printf("  Sensor responding properly\n");
            }
        } else {
            printf("\n");
            set_source(src::UNDEF); // don't even have a device at all...
        }
    }
    // Getter functions
    float min_atm() { return _human.min(); }
    float max_atm() { return _human.max(); }
    float abs_min_atm() { return _abs_min_atm; }
    float abs_max_atm() { return _abs_max_atm; }
    float* min_atm_ptr() { return _human.min_ptr(); }
    float* max_atm_ptr() { return _human.max_ptr(); }
    std::shared_ptr<float> min_atm_shptr() { return _human.min_shptr(); }
    std::shared_ptr<float> max_atm_shptr() { return _human.max_shptr(); }
};

// class AnalogSensor are sensors where the value is based on an ADC reading (eg brake pressure, brake actuator position, pot)
template<typename NATIVE_T, typename HUMAN_T>
class AnalogSensor : public Sensor<NATIVE_T, HUMAN_T> {
  protected:
    virtual void set_val_from_pin() {
        this->set_native(static_cast<NATIVE_T>(analogRead(this->_pin)));  // Soren: can this be done without two casts?
        this->calculate_ema(); // filtered values are kept in human format
    }
  public:
    AnalogSensor(uint8_t arg_pin) : Sensor<NATIVE_T, HUMAN_T>(arg_pin) {}
    String _long_name = "Unknown analog sensor";
    String _short_name = "analog";
    void setup() {
        set_pin(this->_pin, INPUT);
        this->set_source(src::PIN);
    }
    int32_t adc() { return this->_native.val(); }  // Soren: I created these to be available to any child sensors, but untested and not confident it's right
    int32_t min_adc() { return this->_native.min(); }
    int32_t max_adc() { return this->_native.max(); }
};

// CarBattery reads the voltage level from the Mule battery
class CarBattery : public AnalogSensor<int32_t, float> {
  protected:
    float _initial_adc = adcmidscale_adc;
    float _initial_v = 10.0;
    float _abs_min_v = 0.0; // The min vehicle voltage we can sense
    float _abs_max_v = 16.0; // The min vehicle voltage we can sense
    float _op_min_v = 7.0; // The min vehicle voltage we expect to see
    float _op_max_v = 13.8;  // The max vehicle voltage we expect to see.
    float _v_per_adc = _abs_max_v / adcrange_adc;
    float _ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
  public:
    sens senstype = sens::mulebatt;
    CarBattery(uint8_t arg_pin) : AnalogSensor<int32_t, float>(arg_pin) {
        _m_factor = _v_per_adc;
        _human.set_limits(_abs_min_v, _abs_max_v);
        _native.set_limits(0.0, adcrange_adc);
        set_native(_initial_adc);
        set_can_source(src::PIN, true);
    }
    CarBattery() = delete;
    void setup() {
        printf("%s..\n", this->_long_name.c_str());
        AnalogSensor::setup();
    }
    void set_val_from_touch() {
        set_human(12.0);
    }

    String _long_name = "Vehicle battery voltage";
    String _short_name = "mulbat";
    float v() { return _human.val(); }
    float min_v() { return _human.min(); }
    float max_v() { return _human.max(); }
    float op_min_v() { return _op_min_v; }
    float op_max_v() { return _op_max_v; }
};

// LiPoBatt reads the voltage level from a LiPo cell
class LiPoBatt : public AnalogSensor<int32_t, float> {
  protected:
    float _initial_adc = adcmidscale_adc;
    float _initial_v = 10.0;
    float _abs_min_v = 0.0; // The min lipo voltage we can sense
    float _abs_max_v = 4.8; // The min lipo voltage we can sense
    float _op_min_v = 3.2; // The min lipo voltage we expect to see
    float _op_max_v = 4.3;  // The max lipo voltage we expect to see
    float _initial_v_per_adc = _abs_max_v / adcrange_adc;
    float _initial_ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
  public:
    sens senstype = sens::none;
    LiPoBatt(uint8_t arg_pin) : AnalogSensor<int32_t, float>(arg_pin) {
        _ema_alpha = _initial_ema_alpha;
        _m_factor = _initial_v_per_adc;
        _human.set_limits(_abs_min_v, _abs_max_v);
        _native.set_limits(0.0, adcrange_adc);
        set_native(_initial_adc);
        set_can_source(src::PIN, true);
    }
    LiPoBatt() = delete;
    void setup() {
        printf("%s..\n", this->_long_name.c_str());
        AnalogSensor::setup();
    }
    String _long_name = "LiPo pack voltage ";
    String _short_name = "lipo";
    float v() { return _human.val(); }
    float min_v() { return _human.min(); }
    float max_v() { return _human.max(); }
    float op_min_v() { return _op_min_v; }
    float op_max_v() { return _op_max_v; }
};
// PressureSensor represents a brake fluid pressure sensor.
// It extends AnalogSensor to handle reading an analog pin
// and converting the ADC value to a pressure value in PSI.
class PressureSensor : public AnalogSensor<int32_t, float> {
  public:
    sens senstype = sens::pressure;
    int32_t op_min_adc = 658; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
    // Soren 230920: Reducing max to value even wimpier than Chris' pathetic 2080 adc (~284 psi) brake press, to prevent overtaxing the motor
    int32_t op_max_adc = 2080; // ~208psi by this math - "Maximum" braking
    // static constexpr int32_t max_adc = 2080; // ~284psi by this math - Sensor measured maximum reading. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as [wimp] chris can push
    float initial_psi_per_adc = 1000.0 * (3.3 - 0.554) / ( (adcrange_adc - op_min_adc) * (4.5 - 0.554) ); // 1000 psi * (adc_max v - v_min v) / ((4095 adc - 658 adc) * (v-max v - v-min v)) = 0.2 psi/adc 
    float initial_offset = 0.0;
    static constexpr bool initial_invert = false;
    float initial_ema_alpha = 0.15;
    float hold_initial_psi = 45;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
    float hold_increment_psi = 3;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
    float panic_initial_psi = 80; // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
    float panic_increment_psi = 5; // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
    float margin_psi = 1;  // Max acceptible error when checking psi levels
    String _long_name = "Brake pressure sensor";
    String _short_name = "presur";
    PressureSensor(uint8_t arg_pin) : AnalogSensor<int32_t, float>(arg_pin) {
        _ema_alpha = initial_ema_alpha;
        _m_factor = initial_psi_per_adc;
        _b_offset = -from_native(op_min_adc);
        _invert = initial_invert;
        set_native_limits(op_min_adc, op_max_adc);
        set_human_limits(from_native(op_min_adc), from_native(op_max_adc));
        set_native(op_min_adc);
        set_can_source(src::PIN, true);
        set_can_source(src::POT, true);            
    }
    PressureSensor() = delete;
    void setup() {
        printf("%s..\n", this->_long_name.c_str());
        AnalogSensor::setup();
    }
    float psi() { return _human.val(); }
    float min_psi() { return _human.min(); }
    float max_psi() { return _human.max(); }
    float op_min_psi() { return from_native(op_min_adc); }
    float op_max_psi() { return from_native(op_max_adc); }
};

// BrakePositionSensor represents a linear position sensor
// for measuring brake position (TODO which position? pad? pedal?)
// Extends AnalogSensor for handling analog pin reading and conversion.
class BrakePositionSensor : public AnalogSensor<int32_t, float> {
  protected:
    // TODO: add description
    std::shared_ptr<float> _zeropoint;
    // void set_val_from_touch() { _val_filt.set((op_min_retract_in + *_zeropoint) / 2); } // To keep brake position in legal range during simulation
  public:
    sens senstype = sens::brkpos;
    int32_t abs_min_retract_adc = 0;
    int32_t abs_max_extend_adc = adcrange_adc;
    float park_in = 4.234;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (in)
    float op_min_retract_in = 0.506;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (in)
    float op_max_extend_in = park_in; // 4.624;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (in)
    float margin_in = .01;  // TODO: add description
    float initial_zeropoint_in = 3.179;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (in)
    int32_t op_min_retract_adc = 76;  // Calculated on windows calculator. Calculate it here, silly
    int32_t op_max_extend_adc = 965;  // Calculated on windows calculator. Calculate it here, silly
    float abs_min_retract_in = 0.335;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. ("in"sandths of an inch)
    float abs_max_extend_in = 8.300;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (in)
    float initial_in_per_adc = 3.3 * 10000.0 / (3.3 * adcrange_adc * 557); // 3.3 v * 10k ohm * 1/5 1/v * 1/4095 1/adc * 1/557 in/ohm = 0.0029 in/adc
    static constexpr bool initial_invert = false;
    float initial_offset = 0.0;
    float initial_ema_alpha = 0.35;
    String _long_name = "Brake position sensor";
    String _short_name = "brkpos";

    BrakePositionSensor(uint8_t arg_pin) : AnalogSensor<int32_t, float>(arg_pin) {
        _ema_alpha = initial_ema_alpha;
        _m_factor = initial_in_per_adc;
        _invert = initial_invert;
        _b_offset = initial_offset;
        _zeropoint = std::make_shared<float>(initial_zeropoint_in);
        // Soren: this line might be why we broke our brake motor at bm23:
        // set_human_limits(op_min_retract_in, op_max_extend_in);  // wouldn't this be safer?
        set_human_limits(op_min_retract_in, op_max_extend_in);            
        set_native_limits(op_min_retract_adc, op_max_extend_adc);
        set_can_source(src::PIN, true);
        set_can_source(src::POT, true);
    }
    BrakePositionSensor() = delete;
    void setup() {
        printf("%s..\n", this->_long_name.c_str());
        AnalogSensor::setup();
    }
    // is tha brake motor parked?
    bool parked() { return std::abs(_val_filt.val() - park_in) <= margin_in; }

    float in() { return _human.val(); }
    float min_in() { return _human.min(); }
    float max_in() { return _human.max(); }
    float op_min_in() { return op_min_retract_in; }
    float op_max_in() { return op_max_extend_in; }
    // float absmin_in() { return abs_min_retract_in; }
    // float absmax_in() { return abs_max_extend_in; }
    float parkpos() { return park_in; }
    float margin() { return margin_in; }
    float zeropoint() { return *_zeropoint; }
    float* zeropoint_ptr() { return _zeropoint.get(); }
    std::shared_ptr<float> zeropoint_shptr() { return _zeropoint; }
};

// class PulseSensor are hall-monitor sensors where the value is based on magnetic pulse timing of a rotational Source (eg tachometer, speedometer)
template<typename HUMAN_T>
class PulseSensor : public Sensor<int32_t, HUMAN_T> {
  protected:
    int64_t _stop_timeout_us = 1250000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
    Timer _stop_timer;
    bool _negative = false;
    float _stop_thresh;
    float _last_read_time_us;
    volatile int64_t _isr_us = 0;
    volatile int64_t _isr_timer_start_us = 0;
    volatile int64_t _isr_timer_read_us = 0;
    int32_t _isr_buf_us = 0;
    int64_t _delta_abs_min_us; // must be passed into constructor

    // Shadows a hall sensor being triggered by a passing magnet once per pulley turn. The ISR calls
    // esp_timer_get_time() on every pulse to know the time since the previous pulse. I tested this on the bench up
    // to about 0.750 mph which is as fast as I can move the magnet with my hand, and it works.
    // Update: Janky bench test appeared to work up to 11000 rpm.
    void IRAM_ATTR _isr() { // The isr gets the period of the vehicle pulley rotations.
        _isr_timer_read_us = esp_timer_get_time();
        int64_t time_us = _isr_timer_read_us - _isr_timer_start_us;
        if (time_us > _delta_abs_min_us) {  // ignore spurious triggers or bounces
            _isr_timer_start_us = _isr_timer_read_us;
            _isr_us = time_us;
        }
    }

    virtual void set_val_from_pin() {
        _isr_buf_us = static_cast<int32_t>(_isr_us);  // Copy delta value (in case another interrupt happens during handling)
        _isr_us = 0;  // Indicates to isr we processed this value
        if (_isr_buf_us) {  // If a valid rotation has happened since last time, delta will have a value
            this->set_native(_isr_buf_us);
            this->calculate_ema();
            _last_read_time_us = _stop_timer.elapsed();
            _stop_timer.reset();
        }
        // NOTE: should be checking filt here maybe?
        if (_stop_timer.expired()) {  // If time between pulses is long enough an engine can't run that slow
            this->_human.set(0.0);
            this->_val_filt.set(0.0);
        }        
    }

  public:
    PulseSensor(uint8_t arg_pin, int64_t delta_abs_min_us_arg, float stop_thresh_arg) : Sensor<int32_t, HUMAN_T>(arg_pin), _stop_timer(_stop_timeout_us), _delta_abs_min_us(delta_abs_min_us_arg), _stop_thresh(stop_thresh_arg) {}
    PulseSensor() = delete;
    String _long_name = "Unknown Hall Effect sensor";
    String _short_name = "pulsen";
    void setup() {
        set_pin(this->_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(this->_pin), [this]{ _isr(); }, _negative ? FALLING : RISING);
        this->set_source(src::PIN);
    }
    bool stopped() { return this->_val_filt.val() < _stop_thresh; }  // Note due to weird float math stuff, can not just check if tach == 0.0
    float last_read_time() { return _last_read_time_us; }

    int32_t us() { return this->_native.val(); }  // Soren: I created these to be available to any child sensors, but untested and not confident it's right
    int32_t min_us() { return this->_native.min(); }
    int32_t max_us() { return this->_native.max(); }
};

// Tachometer represents a magnetic pulse measurement of the enginge rotation.
// It extends PulseSensor to handle reading a hall monitor sensor and converting RPU to RPM
class Tachometer : public PulseSensor<float> {
  protected:
    int64_t _delta_abs_min_us = 6500;  // 6500 us corresponds to about 10000 rpm, which isn't possible. Use to reject retriggers
    float _stop_thresh_rpm = 0.2;  // Below which the engine is considered stopped
    float _abs_max_rpm = 7000.0;  // Max possible engine rotation speed
    float _redline_rpm = 5500.0;  // Max possible engine rotation speed
    // NOTE: should we start at 50rpm? shouldn't it be zero?
    float _initial_rpm = 50.0; // Current engine speed, raw value converted to rpm (in rpm)
    float _initial_rpm_per_rpus = 60.0 * 1000000.0;  // 1 rot/us * 60 sec/min * 1000000 us/sec = 60000000 rot/min (rpm)
    static constexpr bool _initial_invert = true;
    static constexpr int64_t _initial_zerovalue = 999999;
    float _initial_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
  public:
    sens senstype = sens::tach;
    float _govern_rpm = _redline_rpm;    
    Tachometer(uint8_t arg_pin) : PulseSensor<float>(arg_pin, _delta_abs_min_us, _stop_thresh_rpm) {
        _ema_alpha = _initial_ema_alpha;
        _m_factor = _initial_rpm_per_rpus;
        _invert = _initial_invert;
        _negative = true;
        _zerovalue = _initial_zerovalue;
        set_human_limits(0.0, _redline_rpm);
        set_native_limits(0.0, _stop_timeout_us);
        set_human(_initial_rpm);
        set_can_source(src::PIN, true);
        set_can_source(src::POT, true);
    }
    Tachometer() = delete;
    void setup() {
        printf("%s..\n", this->_long_name.c_str());
        PulseSensor::setup();
    }
    String _long_name = "Tachometer";
    String _short_name = "tach";
    // Query/getter functions
    float rpm() { return _human.val(); }
    // bool engine_stopped() { return stopped(); }
    bool engine_stopped() { return _val_filt.val() < _stop_thresh_rpm ; }  // Note due to weird float math stuff, can not just check if tach == 0.0
    float redline_rpm() { return _human.max(); }
    float govern_rpm() { return _govern_rpm; }
    float* govern_rpm_ptr() { return &_govern_rpm; }
    void set_govern_rpm(float newgovern) { _govern_rpm = newgovern; }
    float abs_max_rpm() { return _abs_max_rpm; }
    float* redline_rpm_ptr() { return _human.max_ptr(); }
    std::shared_ptr<float> redline_rpm_shptr() { return _human.max_shptr(); }
};

// Speedometer represents a magnetic pulse measurement of the enginge rotation.
// It extends PulseSensor to handle reading a hall monitor sensor and converting RPU to MPH
class Speedometer : public PulseSensor<float> {
  protected:
    int64_t _delta_abs_min_us = 4500;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers
    float _stop_thresh_mph = 0.2;  // Below which the car is considered stopped
    float _min_mph = 0.0;
    float _max_mph = 25.0; // What is max speed car can ever go
    float _initial_mph = 0.0; // Current speed, raw value converted to mph (in mph)
    float _initial_redline_mph = 15.0; // What is our steady state speed at redline? Pulley rotation frequency (in milli-mph)
    float _initial_mph_per_rpus = 1000000.0 * 3600.0 * 20 * 3.14159 / (19.85 * 12 * 5280);  // 1 pulrot/us * 1000000 us/sec * 3600 sec/hr * 1/19.85 whlrot/pulrot * 20*pi in/whlrot * 1/12 ft/in * 1/5280 mi/ft = 179757 mi/hr (mph)
    static constexpr bool _initial_invert = true;
    int64_t _initial_zerovalue = 999999;
    float _initial_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
    float _govern_mph, _idle_mph;
  public:
    sens senstype = sens::speedo;
    Speedometer(uint8_t arg_pin) : PulseSensor<float>(arg_pin, _delta_abs_min_us, _stop_thresh_mph) {
        _ema_alpha = _initial_ema_alpha;
        _m_factor = _initial_mph_per_rpus;
        _invert = _initial_invert;
        _zerovalue = _initial_zerovalue;
        set_human_limits(_min_mph, _initial_redline_mph);
        set_native_limits(0.0, _stop_timeout_us);
        set_human(_initial_mph);
        set_can_source(src::PIN, true);
        set_can_source(src::POT, true);
    }
    Speedometer() = delete;
    void setup() {
        printf("%s..\n", this->_long_name.c_str());
        PulseSensor::setup();
    }
    String _long_name = "Speedometer";
    String _short_name = "speedo";
    // Query/getter functions
    float mph() { return _human.val(); }
    // bool car_stopped() { return stopped(); }
    bool car_stopped() { return _val_filt.val() < _stop_thresh_mph ; }  // Note due to weird float math stuff, can not just check if tach == 0.0
    float redline_mph() { return _human.max(); }
    float govern_mph() { return _govern_mph; }
    float idle_mph() { return _idle_mph; }
    float* idle_mph_ptr() { return &_idle_mph; }
    void set_govern_mph(float newgovern) { _govern_mph = newgovern; }
    float max_mph() { return _max_mph; }
    float* redline_mph_ptr() { return _human.max_ptr(); }
    std::shared_ptr<float> redline_mph_shptr() { return _human.max_shptr(); }
};
// NOTE: I implemented the gas servo, but it looks like it's all in native units. should it still be a transducer?
// ServoPWM is a base class for our type of actuators, where by varying a pulse width (in us), motors move.
//    e.g. the gas, brake and steering motors. The gas motor is an actual servo, the others are controlled with servo signaling via jaguars.
template<typename NATIVE_T, typename HUMAN_T>
class ServoPWM : public Transducer<NATIVE_T, HUMAN_T> {
  protected:
    Servo _servo;

    // NOTE: should be marked 'override' but compiler says it doesn't override anything...?
    void set_native_limits(Param<NATIVE_T> &minParam, Param<NATIVE_T> &maxParam) {
        this->set_native_limits(minParam, maxParam);
        _servo.attach(this->_pin, this->min_native->val(), this->max_native->val());
    }

    void set_human_limits(Param<HUMAN_T> &minParam, Param<HUMAN_T> &maxParam) {
        this->set_human_limits(minParam, maxParam);
        _servo.attach(this->_pin, this->min_native->val(), this->max_native->val());
    }

  public:
    // ServoPWM(uint8_t pin, uint_t freq) : Transducer<NATIVE_T, HUMAN_T>(pin) {
    ServoPWM(uint8_t pin, uint8_t freq) : Transducer<NATIVE_T, HUMAN_T>(pin) {
        _servo.setPeriodHertz(freq);
        _servo.attach(this->_pin, this->_native.min(), this->_native.max());
        // _servo.attach(this->_pin, this->_native.abs_min(), this->_native.abs_max());
    }
    ServoPWM() = delete;
    String _long_name = "Unknown PWM motor output";
    String _short_name = "pwmout";

    void setup() {
        set_pin(this->_pin, OUTPUT);
    }
    void write() {
        this->_val_raw = this->_native.val();
        _servo.writeMicroseconds((int32_t)this->_val_raw);  // Write result to servo interface
    }
};
// Device::Toggle is a base class for system signals or devices having a boolean value
class Toggle : public Device {
  public:
    // NOTE: how should we handle simulatability? I almost think it should be done at a higher level than the device...
    // NOTE: should use Param here maybe?
    bool val, val_last, can_sim;
    Toggle(int32_t arg_pin) : Device(arg_pin){  // std::string& eng_name, 
        can_sim = true;
    }
};
// Device::Toggle::InToggle is system signals or devices having a boolean value that serve as inputs (eg basicsw, cruisesw)
class InToggle : public Toggle {
  public:
    InToggle(int32_t arg_pin) : Toggle(arg_pin) {
        set_can_source(src::PIN, true);
        _source = src::PIN;
    }
    void set_val(bool arg_val) {
        if (_source != src::PIN) {
            val_last = val;
            val = arg_val;
        }
    }
    void read() {
        val_last = val;
        val = digitalRead(_pin);
    }
};

// Device::Toggle::OutToggle is system signals or devices having a boolean value that serve as outputs (eg ignition, leds, etc.)
class OutToggle : public Toggle {
  public:
    OutToggle(int32_t arg_pin) : Toggle(arg_pin) {
        // NOTE: LIVE is not a valid source, what should this be? Not PIN, we write to that. CALC maybe?
        // val_source = LIVE;
    }
    void set_val(bool arg_val) {
        val_last = val;
        val = arg_val;
        // if (val_source == LIVE) write();
    }
    void write() {
        digitalWrite(_pin, val);
    }
};
// NOTE: if devices.h gets to be too long, we can (and maybe just should) move this to a separate file, it's not really a device...

// Simulator manages the source handling logic for all simulatable components. Currently, components can recieve simulated input from either the touchscreen, or from
// NOTE: this class is designed to be backwards-compatible with existing code, which does everything with global booleans. if/when we switch all our Devices to use sources,
//       we can simplify the logic here a lot.
class Simulator {
  private:
    // NOTE: if we only simulated devices, we could keep track of simulability in the Device class. We could keep the default source in Device the same way.
    // 3-tuple for a given component, keeping track of simulability status (whether this component is allowed to be simulated), an associated Device (a pointer to the actual component),
    // and a default source (the mode we would like the component to switch back to when it stops being simulated)
    typedef std::tuple<bool, Device*, src> simulable_t;
    std::map<sens, simulable_t> _devices; // a collection of simulatable components
    bool _enabled = false; // keep track of whether the simulator is running or not
    sens _potmap; // keep track of which component is getting info from the pot
    Potentiometer& _pot;
  public:
    Simulator(Potentiometer& pot_arg, sens potmap_arg=sens::none) : _pot(pot_arg) {
        for (uint8_t sensor = (uint8_t)sens::none + 1; sensor < (uint8_t)sens::NUM_SENSORS; sensor++)
            set_can_sim((sens)sensor, false);   // initially turn off simulation of sensors  // static constexpr bool initial_sim_joy = false;
        set_potmap(potmap_arg); // set initial pot map
    }  // syspower, ignition removed, as they are not sensors or even inputs

    void updateSimulationStatus(bool enableSimulation) {
        // If the simulation status hasn't changed, there's nothing to do
        if (_enabled == enableSimulation) return;
        // Iterate over all devices
        for (auto &deviceEntry : _devices) {
            bool can_sim = std::get<0>(deviceEntry.second);
            // If the device can be simulated and isn't being mapped from the potentiometer
            if (can_sim && _potmap != deviceEntry.first) {
                Device *d = std::get<1>(deviceEntry.second);
                // If the device exists
                // NOTE: the nullptr checks here and below exist so that we can work with boolean components as well as Devices, for backwards compatability
                if (d != nullptr) {
                    // If we're enabling the simulation, set the device's source to the touchscreen
                    // Otherwise, set it to its default mode
                    if (enableSimulation) {
                        d->set_source(src::TOUCH);
                    } else {
                        src default_mode = std::get<2>(deviceEntry.second);
                        d->set_source(default_mode);
                    }
                }
            }
        }
        // Update the simulation status
        _enabled = enableSimulation;
    }

    // turn on the simulator. all components which are set to be simulated will switch to simulated input
    void enable() {
        updateSimulationStatus(true);
    }

    // turn off the simulator. all devices will be set to their default input (if they are not being mapped from the pot)
    void disable() {
        updateSimulationStatus(false);
    }

    void toggle() {
        return _enabled ? disable() : enable();
    }

    // check if a componenet is currently being simulated (by either the touchscreen or the pot)
    bool simulating() { return _enabled; }  // equivalent to enabled()  // Maybe include || potmapping() too ?

    bool simulating(sens arg_sensor) {
        return can_sim(arg_sensor) && (_enabled || _potmap == arg_sensor);
    }

    // associate a Device and a given fall-back source with a sensor
    void register_device(sens arg_sensor, Device &d, src default_mode) {
        bool can_sim = false; // by default, disable simulation for this component
        auto kv = _devices.find(arg_sensor); // look for the component
        if (kv != _devices.end()) {
            can_sim = std::get<0>(kv->second); // if an entry for the component already existed, preserve its simulatability status
            if (can_sim) { // if simulability has already been enabled...
                if (arg_sensor == _potmap) { // ...and the pot is supposed to map to this component...
                    d.set_source(src::POT); // ...then set the input source for the associated Device to read from the pot
                } else if (_enabled) { // ...and the pot isn't mapping to this component, but the simulator is running...
                    d.set_source(src::TOUCH); // ...then set the input source for the associated Device to read from the touchscreen
                }
            }
        }
        if (d.can_source(src::POT)) {
            d.attach_pot(_pot); // if this device can be mapped from the pot, connect it to pot input
        }
        _devices[arg_sensor] = simulable_t(can_sim, &d, default_mode); // store info for this component
    }

    // check if a component can be simulated (by either the touchscreen or the pot)
    bool can_sim(sens arg_sensor) {
        auto kv = _devices.find(arg_sensor); // look for the component
        if (kv != _devices.end()) {
            return std::get<0>(kv->second); // if it exists, check the simulability status
        }
        return false; // couldn't find component, so there's no way we can simulate it
    }
    bool touchable(sens arg_sensor) {
        return can_sim(arg_sensor) && (sources[static_cast<int>(arg_sensor)] == static_cast<int>(src::TOUCH));
    }
    // set simulatability status for a component
    void set_can_sim(sens arg_sensor, int32_t can_sim) { set_can_sim(arg_sensor, (can_sim > 0)); }  // allows interpreting -1 as 0, convenient for our tuner etc.
    
    void set_can_sim(sens arg_sensor, bool can_sim) {
        auto kv = _devices.find(arg_sensor); // look for component
        if (kv != _devices.end()) { // if an entry for this component already exists, check if the new simulatability status is different from the old
            bool old_can_sim = std::get<0>(kv->second);
            if (can_sim != old_can_sim) { // if the simulation status has changed, we need to update the input source for the component
                src default_mode = src::UNDEF;
                Device *d = std::get<1>(kv->second);
                if (d != nullptr) { // if there is no associated Device with this component then input handling is done in the main code
                    default_mode = std::get<2>(kv->second); // preserve the stored default controller mode
                    if (can_sim) { // if we just enabled simulatability...
                        if (arg_sensor == _potmap) { // ...and the pot is supposed to map to this component...
                            d->set_source(src::POT); // ...then set the input source for the associated Device to read from the pot
                        } else if (_enabled) { // ...and the pot isn't mapping to this component, but the simulator is running...
                            d->set_source(src::TOUCH); // ...then set the input source for the associated Device to read from the touchscreen
                        }
                    } else {
                        d->set_source(default_mode); // we disabled simulation for this component, set it back to its default input source
                    }
                }
                kv->second = simulable_t(can_sim, d, default_mode); // update the entry with the new simulatability status
            }
        } else {
            _devices[arg_sensor] = simulable_t(can_sim, nullptr, src::UNDEF); // add a new entry with the simulatability status for this component
        }
    }

    // set the component to be overridden by the pot (the pot can only override one component at a time)
    void set_potmap(sens arg_sensor) {
        if (arg_sensor != _potmap) { // if we're mapping to a different component, we need to reset the input source for the old one
            auto kv = _devices.find(_potmap);
            if (kv != _devices.end()) {
                Device *d = std::get<1>(kv->second);
                if (d != nullptr) { // if we were mapping to a component with an associated Device...
                    bool _can_sim = std::get<0>(kv->second);
                    if (_enabled && _can_sim) { // ...and the simulator is on, and we're able to be simulated...
                        d->set_source(src::TOUCH); // ...then set the input source to the touchscreen
                    } else { // ...and either the simulator is off or we aren't allowing simualtion for this component...
                        src default_mode = std::get<2>(kv->second);
                        d->set_source(default_mode); // then set the input source for the component to its default
                    }
                } // ...else this component is a boolean, and input source handling is done elsewhere
            }
            kv = _devices.find(arg_sensor); // we need to set the input source of the newly-overridden component
            if (kv != _devices.end()) {
                Device *d = std::get<1>(kv->second);
                if (d != nullptr ) { // if  we're mapping to a component with an associated device, we need to change the input source to the pot
                    if (d->can_source(src::POT)) { // ...and we're allowed to map to this component...
                        bool _can_sim = std::get<0>(kv->second);
                        if (_can_sim) { // if we allow simualation for this componenent...
                            d->set_source(src::POT); // ...then set its input source to the pot
                        }
                    } else {
                        printf("invalid pot map selected: %d/n", arg_sensor);
                    }
                }
            }
            _potmap = arg_sensor;
        }
    }
    void set_potmap(int32_t arg_sensor) { set_potmap(static_cast<sens>(arg_sensor)); }

    // Getter functions
    bool potmapping(sens s) { return can_sim(s) && _potmap == s; }  // query if a certain sensor is being potmapped
    bool potmapping(int32_t s) { return can_sim(static_cast<sens>(s)) && (_potmap == static_cast<sens>(s)); }  // query if a certain sensor is being potmapped        
    bool potmapping() { return can_sim(_potmap) && !(_potmap == sens::none); }  // query if any sensors are being potmapped
    int32_t potmap() { return static_cast<int32_t>(_potmap); }  // query which sensor is being potmapped
    bool enabled() { return _enabled; }
    bool* enabled_ptr() { return &_enabled; }
};
class RMTInput
{
  public:
    RMTInput(rmt_channel_t channel, gpio_num_t gpio)
    {
        channel_ = channel;
        gpio_ = gpio;
    }
    void init()
    {
        rmt_config_t _config;
        _config.channel = channel_;
        _config.gpio_num = gpio_;
        _config.clk_div = 50; // slowed from 80 because the buffer was getting full
        _config.mem_block_num = 1;
        _config.rmt_mode = RMT_MODE_RX;
        _config.flags = 0;
        _config.rx_config.filter_en = true;          // Enable the filter
        _config.rx_config.filter_ticks_thresh = 100; // Set the filter threshold
        _config.rx_config.idle_threshold = 12000;    // Set the idle threshold

        esp_err_t config_result = rmt_config(&_config);
        if (config_result != ESP_OK)
        {
            Serial.printf("Failed to configure RMT: %d\n", config_result);
            // while (1); // halt execution
        }
        esp_err_t install_result = rmt_driver_install(channel_, 2000, 0);
        if (install_result != ESP_OK)
        {
            Serial.printf("Failed to install RMT driver: %d\n", install_result);
            // while (1); // halt execution
        }
        rmt_get_ringbuf_handle(channel_, &rb_);
        if (rb_ == NULL)
        {
            Serial.println("Failed to initialize ring buffer");
            // while (1); // halt execution
        }
        esp_err_t start_result = rmt_rx_start(channel_, 1);
        if (start_result != ESP_OK)
        {
            Serial.printf("Failed to start RMT receiver: %d\n", start_result);
            // while (1); // halt execution
        }
    }
    int32_t readPulseWidth(bool persistence)  // persistence means the last reading will be returned until a newer one is gathered. Otherwise 0 if no reading
    {
        size_t rx_size = 0;
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb_, &rx_size, 0);
        if (item != NULL && rx_size == sizeof(rmt_item32_t))
        {
            pulse_width = item->duration0 + item->duration1;
            vRingbufferReturnItem(rb_, (void *)item);
            if (!persistence || pulse_width > 0) pulse_width_last = pulse_width * scale_factor;
        }
        else pulse_width = 0;
        return (persistence) ? pulse_width_last : pulse_width; // No data
    }
  private:
    rmt_channel_t channel_;
    gpio_num_t gpio_;
    int32_t pulse_width, pulse_width_last;
    float scale_factor = 0.625;
    RingbufHandle_t rb_;
};
class Hotrc {  // All things Hotrc, in a convenient, easily-digestible format the kids will just love
  public:
    float ema_alpha = 0.075;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1).
    float pc[NUM_AXES][NUM_VALUS];           // values range from -100% to 100% are all derived or auto-assigned
    int32_t us[NUM_CHANS][NUM_VALUS] = {
        {  971, 1470, 1968, 0, 1500, 0, 0, 0 },     // 1000-30+1, 1500-30,  2000-30-2   // [HORZ] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        { 1081, 1580, 2078, 0, 1500, 0, 0, 0 },     // 1000+80+1, 1500+80,  2000+80-2,  // [VERT] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        { 1151, 1500, 1848, 0, 1500, 0, 0, 0 },     // 1000+150+1,   1500, 2000-150-2,  // [CH3] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        { 1251, 1500, 1748, 0, 1500, 0, 0, 0 }, };  // 1000+250+1,   1500, 2000-250-2,  // [CH4] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
    float ema_us[NUM_AXES] = { 1500.0, 1500.0 };  // [HORZ/VERT]
    int32_t absmin_us = 880;
    int32_t absmax_us = 2080;
    int32_t deadband_us = 13;  // All [DBBOT] and [DBTOP] values above are derived from this by calling calc_params()
    int32_t margin_us = 20;  // All [MARGIN] values above are derived from this by calling calc_params()
    int32_t failsafe_us = 880; // Hotrc must be configured per the instructions: search for "HotRC Setup Procedure"
    int32_t failsafe_margin_us = 100; // in the carpet dumpster file: https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA/edit
    int32_t failsafe_pad_us = 10;
  private:
    Simulator* sim;
    Potentiometer* pot;
    static const uint32_t failsafe_timeout = 15000;
    Timer failsafe_timer;  // How long to receive failsafe pulse value continuously before recognizing radio is lost. To prevent false positives
    bool _radiolost = true;
    bool sw[NUM_CHANS] = { 1, 1, 0, 0 };  // index[2]=CH3, index[3]=CH4 and using [0] and [1] indices for LAST values of ch3 and ch4 respectively
    bool _sw_event[NUM_CHANS];  // First 2 indices are unused.  What a tragic waste
    RMTInput rmt[NUM_CHANS] = {
        RMTInput(RMT_CHANNEL_4, gpio_num_t(hotrc_ch1_h_pin)),  // hotrc[HORZ]
        RMTInput(RMT_CHANNEL_5, gpio_num_t(hotrc_ch2_v_pin)),  // hotrc[VERT]
        RMTInput(RMT_CHANNEL_6, gpio_num_t(hotrc_ch3_pin)),  // hotrc[CH3]
        RMTInput(RMT_CHANNEL_7, gpio_num_t(hotrc_ch4_pin)),  // hotrc[CH4]
    };
    bool spike_signbit;
    int32_t spike_length, this_delta, interpolated_slope, loopindex, previndex, spike_cliff[NUM_AXES];
    int32_t spike_threshold[NUM_AXES] = { 6, 6 };
    int32_t prespike_index[NUM_AXES] = { -1, -1 };
    int32_t index[NUM_AXES] = { 1, 1 };  // index is the oldest values are popped from then new incoming values pushed in to the LIFO
    static const int32_t depth = 9;  // more depth will reject longer spikes at the expense of controller delay
    int32_t raw_history[NUM_AXES][depth], filt_history[NUM_AXES][depth];  // Values before and after filtering.
  public:
    Hotrc(Simulator* _sim, Potentiometer* _pot) : sim(_sim), pot(_pot) {
        calc_params();
    }
    void setup() {
        printf("Hotrc init.. Starting rmt..\n");
        for (int axis=HORZ; axis<=CH4; axis++) rmt[axis].init();  // Set up 4 RMT receivers, one per channel
        failsafe_timer.set(failsafe_timeout); 
    }
    void calc_params() {
        for (int8_t axis=HORZ; axis<=VERT; axis++) {
            us[axis][DBBOT] = us[axis][CENT] - deadband_us;
            us[axis][DBTOP] = us[axis][CENT] + deadband_us;
            us[axis][MARGIN] = margin_us;
            pc[axis][OPMIN] = -100.0;
            pc[axis][CENT] = 0.0;
            pc[axis][OPMAX] = 100.0;
            pc[axis][DBBOT] = pc[axis][CENT] - us_to_pc(axis, deadband_us);  // us_to_pc(axis, deadband_us);
            pc[axis][DBTOP] = pc[axis][CENT] + us_to_pc(axis, deadband_us);  // us_to_pc(axis, deadband_us);
            pc[axis][MARGIN] = us_to_pc(axis, margin_us);  // us_to_pc(axis, margin_us);
        }
    }
    int update() {
        toggles_update();
        if (!(sim->simulating(sens::joy))) direction_update();
        radiolost_update();
        return joydir();
    }
    void toggles_reset() {  // Shouldn't be necessary to reset events due to sw_event(ch) auto-resets when read
        for (int8_t ch = CH3; ch <= CH4; ch++) _sw_event[ch] = false;
    }
    bool radiolost() { return _radiolost; }
    bool* radiolost_ptr() { return &_radiolost; }
    bool sw_event(uint8_t ch) {  // returns if there's an event on the given channel then resets that channel
        bool retval = _sw_event[ch];
        _sw_event[ch] = false;
        return retval;        
    }
    void set_pc(int8_t axis, int8_t param, float val) { pc[axis][param] = val; }
    void set_us(int8_t axis, int8_t param, int32_t val) { us[axis][param] = val; }
    int32_t next_unfilt_rawval (uint8_t axis) { return raw_history[axis][index[axis]]; }  // helps to debug the filter from outside the class
    int joydir(int8_t axis = VERT) {
        if (axis == VERT) return ((pc[axis][FILT] > pc[axis][DBTOP]) ? JOY_UP : ((pc[axis][FILT] < pc[axis][DBBOT]) ? JOY_DN : JOY_CENT));
        return ((pc[axis][FILT] > pc[axis][DBTOP]) ? JOY_RT : ((pc[axis][FILT] < pc[axis][DBBOT]) ? JOY_LT : JOY_CENT));
    }  // return (pc[axis][FILT] > pc[axis][DBTOP]) ? ((axis == VERT) ? JOY_UP : JOY_RT) : (pc[axis][FILT] < pc[axis][DBBOT]) ? ((axis == VERT) ? JOY_DN : JOY_LT) : JOY_CENT;
  private:
    void toggles_update() {  //
        for (int8_t chan = CH3; chan <= CH4; chan++) {
            us[chan][RAW] = (int32_t)(rmt[chan].readPulseWidth(true));
            sw[chan] = (us[chan][RAW] <= us[chan][CENT]); // Ch3 switch true if short pulse, otherwise false  us[CH3][CENT]
            if ((sw[chan] != sw[chan-2]) && !_radiolost) {
                _sw_event[chan] = true; // So a handler routine can be signaled. Handler must reset this to false. Skip possible erroneous events while radio lost, because on powerup its switch pulses go low
                kick_inactivity_timer(5);  // evidence of user activity
            }
            sw[chan-2] = sw[chan];  // chan-2 index being used to store previous values of index chan
        }
    }
    void direction_update() {
        if (sim->simulating(sens::joy)) {
            if (sim->potmapping(sens::joy)) pc[HORZ][FILT] = pot->mapToRange(pc[HORZ][OPMIN], pc[HORZ][OPMAX]);  // overwrite horz value if potmapping
            // Serial.printf("%d %d %lf\n",sim->potmapping(sens::joy),sim->potmapping(), pc[HORZ][FILT]);
        }
        else for (int8_t axis = HORZ; axis <= VERT; axis++) {
            us[axis][RAW] = (int32_t)(rmt[axis].readPulseWidth(true));
            us[axis][RAW] = spike_filter(axis, us[axis][RAW]);  // Not exactly "raw" any more after spike filter (not to mention really several readings in the past), but that's what we need
            ema_filt(us[axis][RAW], &ema_us[axis], ema_alpha);  // Need unconstrained ema-filtered vertical for radio lost detection 
            if (us[axis][RAW] >= us[axis][CENT])  // pc[axis][RAW] = us_to_pc(axis, us[axis][RAW]);
                pc[axis][RAW] = map((float)us[axis][RAW], (float)us[axis][CENT], (float)us[axis][OPMAX], pc[axis][CENT], pc[axis][OPMAX]);
            else pc[axis][RAW] = map((float)us[axis][RAW], (float)us[axis][CENT], (float)us[axis][OPMIN], pc[axis][CENT], pc[axis][OPMIN]);
            ema_filt(pc[axis][RAW], &(pc[axis][FILT]), ema_alpha);  // do ema filter to determine joy_vert_filt
            if (ema_us[axis] > us[axis][DBBOT] && ema_us[axis] < us[axis][DBTOP]) pc[axis][FILT] = pc[axis][CENT];  // if within the deadband set joy_axis_filt to CENTer value
            else if (!_radiolost) kick_inactivity_timer(6);  // otherwise indicate evidence of user activity
            if (_radiolost) pc[axis][FILT] = pc[axis][CENT];  // if radio lost set joy_axis_filt to CENTer value
        }
        for (int8_t axis = HORZ; axis <= VERT; axis++) pc[axis][FILT] = constrain(pc[axis][FILT], pc[axis][OPMIN], pc[axis][OPMAX]);
    }
    bool radiolost_update() {
        if (ema_us[VERT] > failsafe_us + failsafe_margin_us) {
            failsafe_timer.reset();
            _radiolost = false;
        }
        else if (failsafe_timer.expired()) _radiolost = true;
        return _radiolost;
    }
    float us_to_pc(int8_t _axis, int32_t _us) {  // float us_to_pc(int8_t axis, int32_t us) {
        return (float)_us * (pc[VERT][OPMAX] - pc[VERT][OPMIN]) / (float)(us[VERT][OPMAX] - us[VERT][OPMIN]);
        // if (us >= us[axis][CENT])
        //     return map((float)us, (float)us[axis][CENT], (float)us[axis][OPMAX], pc[axis][CENT], pc[axis][OPMAX]);
        // else return map((float)us, (float)us[axis][CENT], (float)us[axis][OPMIN], pc[axis][CENT], pc[axis][OPMIN]);
    }
    // Spike filter pushes new hotrc readings into a LIFO ring buffer, replaces any well-defined spikes with values 
    // interpolated from before and after the spike. Also smoothes out abrupt value changes that don't recover later
    int32_t spike_filter (uint8_t axis, int32_t new_val) {  // pushes next val in, massages any detected spikes, returns filtered past value
        previndex = (depth + index[axis] - 1) % depth;  // previndex is where the incoming new value will be stored
        this_delta = new_val - filt_history[axis][previndex];  // Value change since last reading
        if (std::abs(this_delta) > spike_cliff[axis]) {  // If new value is a cliff edge (start or end of a spike)
            if (prespike_index[axis] == -1) {  // If this cliff edge is the start of a new spike
                prespike_index[axis] = previndex;  // save index of last good value just before the cliff
                spike_signbit = signbit(this_delta);  // Save the direction of the cliff
            }
            else if (spike_signbit == signbit(this_delta)) {  // If this cliff edge deepens an in-progress spike (or more likely the change is valid)
                inject_interpolations(axis, previndex, filt_history[axis][previndex]);  // Smoothly grade the values from before the last cliff to previous value
                prespike_index[axis] = previndex;  // Consider this cliff edge the start of the spike instead
            }
            else {  // If this cliff edge is a recovery of an in-progress spike
                inject_interpolations(axis, index[axis], new_val);  // Fill in the spiked values with interpolated values
                prespike_index[axis] = -1;  // Cancel the current spike
            }
        }
        else if (prespike_index[axis] == index[axis]) {  // If a current spike lasted thru our whole buffer
            inject_interpolations (axis, previndex, filt_history[axis][previndex]);  // Smoothly grade the whole buffer
            prespike_index[axis] = -1;  // Cancel the current spike
        }
        int32_t returnval = filt_history[axis][index[axis]];  // Save the incumbent value at current index (oldest value) into buffer
        filt_history[axis][index[axis]] = new_val;
        raw_history[axis][index[axis]] = new_val;
        ++(index[axis]) %= depth;  // Update index for next time
        return returnval;  // Return the saved old value
    }
    void inject_interpolations(uint8_t axis, int32_t endspike_index, int32_t endspike_val) {  // Replaces values between indexes with linear interpolated values
        spike_length = ((depth + endspike_index - prespike_index[axis]) % depth) - 1;  // Equal to the spiking values count plus one
        if (!spike_length) return;  // Two cliffs in the same direction on consecutive readings needs no adjustment, also prevents divide by zero 
        interpolated_slope = (endspike_val - filt_history[axis][prespike_index[axis]]) / spike_length;
        loopindex = 0;
        while (++loopindex <= spike_length)
            filt_history[axis][(prespike_index[axis] + loopindex) % depth] = filt_history[axis][prespike_index[axis]] + loopindex * interpolated_slope;
    }
};