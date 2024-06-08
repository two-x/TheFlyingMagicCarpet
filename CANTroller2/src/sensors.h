#pragma once
#include <iostream>
#include <map>
#include <memory> // for std::shared_ptr
#include <SparkFun_FS3000_Arduino_Library.h>  // For air velocity sensor  http://librarymanager/All#SparkFun_FS3000
#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include "driver/rmt.h"
#include <ESP32Servo.h>        // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <Preferences.h>  // Functions for writing values to nvs flash partition

// This enum class represent the components which can be simulated (sensor). It's int type under the covers, so it can be used as an index
// typedef int opt_t;
enum si_native_conversion_methods { LinearMath=0, AbsLimMap=1, OpLimMap=2 };
enum class sens : int { none=0, joy=1, pressure=2, brkpos=3, speedo=4, tach=5, airvelo=6, mapsens=7, engtemp=8, mulebatt=9, starter=10, basicsw=11, NUM_SENSORS=12 };  //, ignition, syspower };  // , NUM_SENSORS, err_flag };
enum class src : int { UNDEF=0, FIXED=1, PIN=2, TOUCH=3, POT=4, CALC=5 };

int sources[static_cast<int>(sens::NUM_SENSORS)] = { static_cast<int>(src::UNDEF) };
#include "i2cbus.h"

// Potentiometer does an analog read from a pin and maps it to a percent (0%-100%). We filter the value to keep it smooth.
class Potentiometer {
  protected:
    float _ema_alpha = 0.99;
    float _opmin = 0.0, _opmax = 100.0 ;  // in percent
    float _opmin_native = 380; // TUNED 230603 - Used only in determining theconversion factor
    float _opmax_native = 4095; // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
    float _absmin_native = 0.0, _absmax_native = static_cast<float>(adcrange_adc);
    float _absmin = 0.0, _absmax = 100.0;
    float _activity_margin_pc = 7.5;
    int _pin;
    float _pc = 50.0, _raw, _native, _activity_ref;  // values in filtered percent, raw percent, raw adc
    Timer pot_timer{100000};  // adc cannot read too fast w/o errors, so give some time between readings
  public:
    // tune these by monitoring adc_raw and twist pot to each limit. set min at the highest value seen when turned full CCW, and vice versas for max, then trim each toward adc_midrange until you get full 0 to 100% range
    Potentiometer(int arg_pin) : _pin(arg_pin) {}
    Potentiometer() = delete; // must have a pin defined
    sens senstype = sens::none;
    void setup() {
        ezread.squintf("Pot setup..\n");
        set_pin(_pin, INPUT);
        _activity_ref = _pc;
    }
    void update() {
        if (pot_timer.expireset()) {
            _native = static_cast<float>(analogRead(_pin));
            _raw = map(_native, _opmin_native, _opmax_native, _opmin, _opmax);
            ema_filt(_raw, &_pc, _ema_alpha);
            _pc = constrain(_pc, _absmin, _absmax); // the lower limit of the adc reading isn't steady (it will dip below zero) so constrain it back in range
            if (std::abs(_pc - _activity_ref) > _activity_margin_pc) {
                // ezread.squintf("a:%ld n:%lf v:%lf r:%lf m:%lf ", adc_raw, new_val, _val, _activity_ref, _pc_activity_margin);
                kick_inactivity_timer(HUPot);  // evidence of user activity
                _activity_ref = _pc;
                // ezread.squintf("r2:%lf\n", _activity_ref);
            }
        }
    }
    float mapToRange(float argmin, float argmax) {
        return map(_pc, _opmin, _opmax, argmin, argmax);
    }
    float val() { return _pc; }
    float raw() { return _raw; }
    float native() { return _native; }
    float opmin() { return _opmin; }
    float opmax() { return _opmax; }
    float opmin_native() { return _opmin_native; }
    float opmax_native() { return _opmax_native; }
};
// NOTE: the following classes all contain their own initial config values (for simplicity). We could instead use Config objects and pass them in at construction time, which might be
//       a little cleaner organization-wise, since all the configs would be consolidated. It would also allow us to read Configs from storage somewhere, so we could have persistent
//       calibration.

// Param is a value which is constrained between min/max limits, representing a "raw" (aka unfiltered) quantity. A value with tight limits
// (wehere min=val=max) is constant and cannot be changed without changing the limits. An unconstrained value can be represented by setting
// either or both min/max to infinity.
class Param {
  protected:
    float _min_internal = 0.0, _max_internal = 0.0;  // these are internally managed values our limit pointers can point at if nothing external is given
    float _val, _last;  // this is our primary value and its previous value
    float* _min_ptr = &_min_internal;
    float* _max_ptr = &_max_internal;
    void constrain_value() { _val = constrain(_val, *_min_ptr, *_max_ptr); }
  public:
    Param() { _last = _val; }
    Param(float arg_val, float arg_min, float arg_max) {  // Creates a regular constrained Param
        _val = arg_val;
        *_min_ptr = arg_min;
        *_max_ptr = arg_max;
        set_limits(*_min_ptr, *_max_ptr);
        _last = _val;
    } 
    Param(float arg_val) { Param(arg_val, arg_val, arg_val); }  // Creates a constant Param (with min/max == val) which cannot be changed
    Param(float arg_val, float* arg_min_ptr, float* arg_max_ptr) {  // Creates a constraned Param which uses external limits.
        _val = arg_val;
        _min_ptr = arg_min_ptr;
        _max_ptr = arg_max_ptr;
        set_limits(*_min_ptr, *_max_ptr);
        _last = _val;
    }
    // NOTE: if using external limits, it's (currently) possible to get stale values, since there is no
    //       callback mechanism in place. We could get around this by calling constrain_value() on every
    //       get() call, but that seems like overkill...
    void set_limits(float arg_min, float arg_max) {  // Use if min/max are kept in-class
        if (arg_min > arg_max) {
            ezread.squintf("Err: Param set_limits(): %lf (min) is >= %lf (max)\n", arg_min, arg_max);
            return;
        }
        *_min_ptr = arg_min;
        *_max_ptr = arg_max;
        constrain_value();
    }
    void set_limits(float* arg_min_ptr, float* arg_max_ptr) { // Use if min/max are external
        if (*arg_min_ptr > *arg_max_ptr) {
            ezread.squintf("Err: Param set_limits(): %lf (*min) is >= %lf (*max)\n", *arg_min_ptr, *arg_max_ptr);
            return;
        }
        _min_ptr = arg_min_ptr;
        _max_ptr = arg_max_ptr;
        constrain_value();
    }
    // return value indicates if the value actually changed or not
    bool set(float arg_val) {
        if (std::abs(arg_val) < float_zero) arg_val = 0.0;  // avoid stupidly low near-zero values that happens sometimes
        if (std::abs(_val - arg_val) < float_zero) return false;
        _last = _val;
        _val = arg_val;
        constrain_value();
        return true;
    }
    bool add(float arg_add) { return set(_val + arg_add); }
    float val() { return _val; }
    float min() { return *_min_ptr; }
    float max() { return *_max_ptr; }
    float* ptr() { return &_val; }
    float* min_ptr() { return _min_ptr; }
    float* max_ptr() { return _max_ptr; }
    float last() { return _last; } // NOTE: currently unused, do we still need this for drawing purposes?
};
// Device class - is a base class for any connected system device or signal associated with a pin
class Device {
  protected:
    // Which types of sources are possible for this device?
    int _pin;
    bool _enabled = true;
    Potentiometer* _pot; // to pull input from the pot if we're in simulation mode
    src _source = src::UNDEF;
    bool _can_source[6] = { true, true, false, true, false, false };  // [UNDEF/FIXED/PIN/TOUCH/POT/CALC]
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
    std::string _long_name = "Unknown device";
    std::string _short_name = "device";
    std::string _native_units = "";
    std::string _si_units = "";
    Timer timer;  // Can be used for external purposes
    sens senstype = sens::none;
    Device() = delete; // should always be created with a pin
    // NOTE: should we start in PIN mode?
    Device(int arg_pin) : _pin(arg_pin) {}
    bool can_source(src arg_source) { return _can_source[static_cast<int>(arg_source)]; }
    bool set_source(src arg_source) {
        if (_can_source[static_cast<int>(arg_source)]) {
            _source = arg_source;
            update_source();
            sources[static_cast<int>(senstype)] = static_cast<int>(arg_source);
            return true;
        } 
        return false;
    }
    void update() {  // I changed case statement into if statements and now is 10 lines long instead of 28.  case sucks like that
        if (!_enabled) return; // do nothing if the Device is disabled
        if (_source == src::UNDEF) set_val_from_undef();
        else if (_source == src::FIXED) set_val_from_fixed();
        else if (_source == src::PIN) set_val_from_pin();
        else if (_source == src::TOUCH) set_val_from_touch();
        else if (_source == src::POT) set_val_from_pot();
        else if (_source == src::CALC) set_val_from_calc();
        else ezread.squintf("invalid Device source: %d\n", _source);
        set_val_common();
    }
    void attach_pot(Potentiometer &pot_arg) {
        _pot = &pot_arg;
    }
    void set_enabled(bool arg_enable) { _enabled = arg_enable; }
    void set_can_source(src arg_source, bool is_possible) { _can_source[static_cast<int>(arg_source)] = is_possible; }
    src source() { return _source; }
    int pin() { return _pin; }
    bool enabled() { return _enabled; }
};

enum class TransDir : int { REV=0, FWD=1, NumTransDir=2 }; // possible dir values. REV means native sensed value has the opposite polarity of the real world effect (for example, brake position lower inches of extension means higher applied brakes)
enum TransType { ActuatorType=0, SensorType=1, NumTransType=2 }; // possible dir values. REV means native sensed value has the opposite polarity of the real world effect (for example, brake position lower inches of extension means higher applied brakes)
std::string transtypecard[NumTransType] = { "actuator", "sensor" };
std::string transdircard[(int)TransDir::NumTransDir] = { "rev", "fwd" };

// Transducer class
// Device::Transducer is a base class for any system devices that convert real-world values <--> signals in either direction. it has a "native"
// value which represents the sensed or driven hardware input/output. it also has a "si" value which represents the logical or human-readable
// equivalent value. by default, adjusting either value will automatically change the other one. this class holds key device values, converts between different units they might use, and manages their constraint within specified range limits
// Each xducer has a primary value associated with it, eg for the speedo this is our speed. Here we maintain 3 separate versions of our primary value:
//   Variables: 
//     _si (Param): a possibly-filtered value in human-readable units like feet or psi, auto constrained within specified absolute limits*.
//     _native (Param) : a raw (unfiltered) value in machine-interface units, often adc counts or microseconds (us), auto constrained within specified absolute limits.
//     _si_raw (float) : is a raw (unfiltered) si-unit conversion of the raw value, useful if scrutinizing the effects of filters, etc.
//   Units:  units of measure appropriate to the device. these appear in variable and function names, or when absent the default "si" is assumed. Here are the possibilities:  
//     "si": (default) values kept in standard units of measure which humans prefer, such as mph, feet-per-second, etc.
//     "native": values kept in whatever units are native to the specific device/driver, usually adc counts (for analog sensors) or microseconds (for pwm)... depends on the sensor type
//     "pc": values scaled to a percentage of the operational range, where opmin = 0% and opmax = 100%
//   Conversions:  set conversion_method to best suit the most reliable data you have for your sensor. this decides what math to do when converting between native and si unit values. Here are the choices:
//     AbsLimMap : runs map() function to scale value from one absolute range to the other. For this you must first have explicitly set abs ranges for both native and si
//     OpLimMap : same as AbsLimMap but transpose across the operational ranges rather than the absolute ranges
//     LinearMath : uses y = mx + b generic linear equation to convert. if using this then setting the abs or op range in one unit will auto calc the other. for this you need to set accurate values for the following:
//       _mfactor : conversion rate for si-units/native-units.  (eg for feet (si) and yards (native) would be 3 ft/yd)
//       _boffset : constant in si units to add after native-to-si conversion, or subtract before si-to-native conversion
//   Limits:
//     "absmin"/"absmax": defines the whole range the transducer is capable of. all values are auto-constrained to this range, also tunability of operational limits are confined to this range
//     "opmin"/"opmax": defines the healthy operational range the transducer. Actuators should be constrained to this range, Sensors should be expected to read within this range or flag an error
class Transducer : public Device {
  protected:
    float _mfactor = 1.0, _boffset = 0.0, touch_val;
    float _opmin = NAN, _opmax = NAN, _opmin_native = NAN, _opmax_native = NAN, _margin = 0.0;
    TransDir _dir = TransDir::FWD;
    int conversion_method = LinearMath;  // the default method
    Param _si, _native;
    float _zeropoint, _si_raw;  // si_raw is an output for display purposes, only meaningful for sensors, not actuators. managed here because that's easier
  public:
    Transducer(int arg_pin) : Device(arg_pin) {
        _long_name = "Unknown transducer";
        _short_name = "xducer";
    }
    Transducer() = delete;  // this ensures a pin is always provided
    int _transtype;
    virtual void setup() {
        // print_config(true, false);  // print header
        // if (_pin < 255) ezread.squintf(" on pin %d\n", _pin);
    }  // printf("%s..\n", _long_name.c_str()); }
    virtual float from_native(float arg_native) {  // these linear conversion functions change absolute native values to si and back - overwrite in child classes as needed
        float ret = NAN;
        if (conversion_method == AbsLimMap) ret = map(arg_native, _native.min(), _native.max(), _si.min(), _si.max());
        else if (conversion_method == OpLimMap) ret = map(arg_native, _opmin_native, _opmax_native, _opmin, _opmax);
        else if (conversion_method == LinearMath) ret = _boffset + _mfactor * arg_native; // ezread.squintf("%lf = %lf + %lf * %lf\n", ret, _boffset, _mfactor, arg_val_f);
        if (std::isnan(ret)) ezread.squintf("Err: from_native unable to convert %lf (min %lf, max %lf)\n", arg_native, _native.min(), _native.max());
        if (std::abs(ret) < float_conversion_zero) return 0.0;  // reject any stupidly small near-zero values
        return ret;
    }
    virtual float to_native(float arg_si) {  // convert an absolute si value to native units
        float ret = NAN;
        if (conversion_method == AbsLimMap) ret = map(arg_si, _si.min(), _si.max(), _native.min(), _native.max());  // TODO : this math does not work if _invert == true!
        else if (conversion_method == OpLimMap) ret = map(arg_si, _opmin, _opmax, _opmin_native, _opmax_native);  // TODO : this math does not work if _invert == true!
        else if (conversion_method == LinearMath) ret = (arg_si - _boffset) / _mfactor;
        else if (std::isnan(ret)) ezread.squintf("Err: to_native unable to convert %lf (min %lf, max %lf)\n", arg_si, _si.min(), _si.max());
        if (std::abs(ret) < float_conversion_zero) return 0.0;  // reject any stupidly small near-zero values
        return ret;
    }
    virtual float to_pc(float arg_si) {  // convert an absolute si value to percent form.  note this honors the _dir setting
        float ret = map(arg_si, _opmin, _opmax, 100.0 * (_dir == TransDir::REV), 100.0 * (_dir == TransDir::FWD));
        if (std::abs(ret) < float_conversion_zero) return 0.0;  // round off absurdly small values
        return ret;
    }
    virtual float from_pc(float arg_pc) {  // convert an absolute percent value to si unit form.  note this honors the _dir setting
        float ret = map(arg_pc, 100.0 * (_dir == TransDir::REV), 100.0 * (_dir == TransDir::FWD), _opmin, _opmax);
        if (std::abs(ret) < float_conversion_zero) return 0.0;  // round off absurdly small values
        return ret;
    }
    // To set limits, depends on your conversion method being used :
    //   linear_math: call either one of set_abslim() or set_abslim_native(), whichever one you have the values to define the range and all si and native abs values will be set automatically.
    //                op limits will also be reconstrained to the updated abs values, however if you have a specific op range then
    //   abslimmap: call both of set_abslim() and set_abslim_native(), providing all 4 values which are later used to convert values. specify false for the autocalc argument
    //   oplimmap:  call both of set_oplim() and set_oplim_native(), providing all 4 values which are later used to convert values. specify false for the autocalc argument
    virtual void set_abslim(float argmin=NAN, float argmax=NAN, bool autocalc=true) { // these are absmin and absmax limits. values where NAN is passed in won't be used
        if (std::isnan(argmin)) argmin = _si.min();                               // use incumbent min value if none was passed in
        if (std::isnan(argmax)) argmax = _si.max();                               // use incumbent max value if none was passed in
        _si.set_limits(argmin, argmax);                                           // commit to the Param accordingly
        _si_raw = constrain(_si_raw, _si.min(), _si.max());                       // in case we have new limits, re-constrain the raw si value we also manage
        if ((conversion_method == LinearMath) && autocalc) {                      // if we know our conversion formula, and not instructed to skip autocalculation...
            _native.set_limits(to_native(_si.min()), to_native(_si.max()));       // then convert the new values and ensure si and native stay equivalent
        }
        set_oplim(NAN, NAN, false);                                               // just to enforce any re-constraints if needed to keep op limits inside abs limits
    }
    virtual void set_abslim_native(float argmin=NAN, float argmax=NAN, bool autocalc=true) { // these are absmin and absmax limits. values where NAN is passed in won't be used
        if (std::isnan(argmin)) argmin = _native.min();                              // use incumbent min value if none was passed in
        if (std::isnan(argmax)) argmax = _native.max();                              // use incumbent max value if none was passed in
        _native.set_limits(argmin, argmax);                                          // commit to the Param accordingly
        if ((conversion_method == LinearMath) && autocalc) {                         // if we know our conversion formula, and not instructed to skip autocalculation...
            _si.set_limits(from_native(_native.min()), from_native(_native.max()));  // then convert the new values and ensure si and native stay equivalent
        }
        set_oplim_native(NAN, NAN, false);                                           // just to enforce any re-constraints if needed to keep op limits inside abs limits
    }
    virtual void set_oplim(float argmin=NAN, float argmax=NAN, bool autocalc=true) {  // these are opmin and opmax limits. values where NAN is passed in won't be used
        if (!std::isnan(argmin)) _opmin = argmin;                             // if min value was passed in then set opmin to it
        else if (std::isnan(_opmin)) _opmin = _si.min();                      // otherwise if opmin has no value then set it to absmin
        if (!std::isnan(argmax)) _opmax = argmax;                             // if max value was passed in then set opmax to it
        else if (std::isnan(_opmax)) _opmax = _si.max();                      // otherwise if opmax has no value then set it to absmax
        _opmin = constrain(_opmin, _si.min(), _opmax);                        // constrain to ensure absmin <= opmin <= opmax <= absmax
        _opmax = constrain(_opmax, _opmin, _si.max());                        // constrain to ensure absmin <= opmin <= opmax <= absmax
        if ((conversion_method != OpLimMap) && autocalc) {                    // if we know our conversion formula, and not instructed to skip autocalculation...
            set_oplim_native(to_native(_opmin), to_native(_opmax), false);    // then convert the new values and ensure si and native stay equivalent
        }
    }
    virtual void set_oplim_native(float argmin=NAN, float argmax=NAN, bool autocalc=true) { // these are opmin and opmax limits. values where NAN is passed in won't be used
        if (!std::isnan(argmin)) _opmin_native = argmin;                               // if min value was passed in then set opmin to it
        else if (std::isnan(_opmin_native)) _opmin_native = _native.min();             // otherwise if opmin has no value then set it to absmin
        if (!std::isnan(argmax)) _opmax_native = argmax;                               // if max value was passed in then set opmax to it
        else if (std::isnan(_opmax_native)) _opmax_native = _native.max();             // otherwise if opmax has no value then set it to absmax
        _opmin_native = constrain(_opmin_native, _native.min(), _opmax_native);        // constrain to ensure absmin <= opmin <= opmax <= absmax
        _opmax_native = constrain(_opmax_native, _opmin_native, _native.max());        // constrain to ensure absmin <= opmin <= opmax <= absmax
        if ((conversion_method != OpLimMap) && autocalc) {                             // if we know our conversion formula, and not instructed to skip autocalculation...
            set_oplim(from_native(_opmin_native), from_native(_opmax_native), false);  // then convert the new values and ensure si and native stay equivalent
        }
    }
    bool set_native(float arg_val_native) {
        if (!_native.set(arg_val_native)) return false;
        _si.set(from_native(_native.val()));
        _si_raw = _si.val();
        return true;
    }
    bool set_si(float arg_val_si) {
        if (!_si.set(arg_val_si)) return false;
        _si_raw = _si.val();
        _native.set(to_native(_si.val()));
        return true;
    }
    bool set_pc(float arg_val_pc) { 
        return set_si(from_pc(arg_val_pc));
    }
    bool add_native(float arg_add) {  // this scales the given add amount to a consistent rate of change 
        float delta = arg_add * tuning_rate_pcps * loop_avg_us * (_opmax_native - _opmin_native) / (100.0 * 1000000.0);  // this acceleration logic doesn't belong here
        return set_native(constrain(_native.val() + delta, _opmin_native, _opmax_native));
    }
    bool add_pc(float arg_add) {  // this scales the given add amount to a consistent rate of change
        return add_si(from_pc(arg_add));
    }
    bool add_si(float arg_add) {  // this scales the given add amount to a consistent rate of change
        float delta = arg_add * tuning_rate_pcps * loop_avg_us * (_opmax - _opmin) / (100.0 * 1000000.0);  // this acceleration logic doesn't belong here
        return set_si(constrain(_si.val() + delta, _opmin, _opmax));
    }
    void set_margin(float arg_marg) { _margin = arg_marg; }
    
    void set_conversions(float arg_mfactor=NAN, float arg_boffset=NAN) {  // arguments passed in as NAN will not be used.  // Convert units from base numerical value to disp units:  val_native = m-factor*val_numeric + offset  -or-  val_native = m-factor/val_numeric + offset  where m-factor, b-offset, invert are set here
        if (!std::isnan(arg_mfactor)) {
            if (std::abs(arg_mfactor) < float_zero) ezread.squintf("Err: can not support _mfactor of zero\n");
            else _mfactor = arg_mfactor;
        }
        if (!std::isnan(arg_mfactor)) _boffset = arg_boffset;
    }
    virtual void tedit(float tdelta) {  // for touchscreen editing of the value
        touch_val = constrain(_si.val() + tdelta, _opmin, _opmax);
    }
    virtual void print_config(bool header=true, bool ranges=true) {
        if (header) {
            Serial.printf("%s %s", _long_name.c_str(), transtypecard[_transtype].c_str());
            ezread.printf("%s", _short_name.c_str()); 
            if (_pin < 255 && _pin <= 0) {
                Serial.printf(", pin %d", _pin);
                ezread.printf(" p%d", _pin);
            }
            Serial.printf(": %.2lf %s = %.2lf %s = %.2lf %%\n", _si.val(), _si_units.c_str(), _native.val(), _native_units.c_str(), pc()); 
            ezread.printf(": %.2lf%s = %.2lf%s = %.2lf%%\n", _si.val(), _si_units.c_str(), _native.val(), _native_units.c_str(), pc()); 
        }
        if (ranges) {
            Serial.printf("  op: %.2lf - %.2lf %s (%.2lf - %.2lf %s)\n", _opmin, _opmax, _si_units.c_str(), _opmin_native, _opmax_native, _native_units.c_str());
            Serial.printf("  abs: %.2lf - %.2lf %s (%.2lf - %.2lf %s)\n", _si.min(), _si.max(), _si_units.c_str(), _native.min(), _native.max(), _native_units.c_str());
            ezread.printf("  op: %.2lf-%.2lf%s (%.2lf-%.2lf%s)\n", _opmin, _opmax, _si_units.c_str(), _opmin_native, _opmax_native, _native_units.c_str());
            ezread.printf("  abs: %.2lf-%.2lf%s (%.2lf-%.2lf%s)\n", _si.min(), _si.max(), _si_units.c_str(), _native.min(), _native.max(), _native_units.c_str());
        }
    }
    float val() { return _si.val(); }  // this is the si-unit filtered value (default for general consumption)
    float* ptr() { return _si.ptr(); }
    float absmin() { return _si.min(); }
    float absmax() { return _si.max(); }
    float* absmin_ptr() { return _si.min_ptr(); }
    float* absmax_ptr() { return _si.max_ptr(); }
    float opmin() { return _opmin; }
    float opmax() { return _opmax; }
    float* opmin_ptr() { return &_opmin; }
    float* opmax_ptr() { return &_opmax; }
    float margin() { return _margin; }
    float raw() { return _si_raw; }  // this is the si-unit raw value, constrained to abs range but otherwise unfiltered
    float native() { return _native.val(); }  // This is a native unit value, constrained to abs range but otherwise unfiltered
    float* native_ptr() { return _native.ptr(); }
    float absmin_native() { return _native.min(); }
    float absmax_native() { return _native.max(); }
    float opmin_native() { return _opmin_native; }
    float opmax_native() { return _opmax_native; }
    float margin_native() { return to_native(_margin); }
    float pc() { return to_pc(_si.val()); }  // get value as a percent of the operational range
    float raw_pc() { return to_pc(_si_raw); }  // get raw value in percent
    float margin_pc() { return std::abs(to_pc(_margin)); }
    float zeropoint() { return _zeropoint; }  // zeropoint is the pressure at which we can consider the brake is released (if position is unavailable)
    float zeropoint_pc() { return to_pc(_zeropoint); }
    float* zeropoint_ptr() { return &_zeropoint; }
};

// Sensor class - is a base class for control system sensors, ie anything that measures real world data or electrical signals 
// 
// Sensor class notes:  Some definitions (240522 soren)
// Sensors typically apply filtering to the incoming values read. Postprocessing is done to the _si Param value accessible externally by val()
//   the values are described below. all this is inherited from Transducer, just with filtering applied:
//     "_native": this unfiltered value is in units direct from the sensor/driver, constrained to abs range. get externally using native()
//     "_raw": this is the same unfiltered value just converted to the default si units. get externally using raw()
//     "_si": this is the value after filtering is applied, in default si units, externally accessible using val()
//     "pc()": the filtered value is available scaled as a percentage of the operatioal range. It's not stored, but accessible using pc()
class Sensor : public Transducer {
  protected:
    float _ema_alpha = 0.1;
    bool _first_filter_run = false;

    void calculate_ema() { // Exponential Moving Average
        if (_first_filter_run) {
            set_si(_si_raw);
            _first_filter_run = false;  // soren: I commented this out, wouldn't this always turn on filtering?
        }
        else set_si(ema_filt(_si_raw, _si.val(), _ema_alpha));
    }
    virtual void set_val_from_touch() {  // for example by the onscreen simulator interface. TODO: examine this functionality, it aint right
        set_si(touch_val);                  // wtf is this supposed to do?
        // set_si(_si.val + touch.fdelta);  // i would think this should look something like this (needs some coding on the other side to support)
    }
    virtual float read_sensor() {
        ezread.squintf("Err: %s does not have an overridden read_sensor() function\n", _short_name.c_str());
        return NAN;
    }
    virtual void set_val_from_pin() {
        set_native(read_sensor());
        calculate_ema();  // Sensor EMA filter
    }
    virtual void set_val_from_pot() {
        set_si(_pot->mapToRange(_opmin, _opmax));  // as currently written, the pot will spoof both si and native raw values in addition to the filtered si value.  do we want this?
    }
    // virtual void update_si_limits() { _val_filt.set_limits(_si.min_shptr(), _si.max_shptr()); } // make sure our filtered value has the same limits as our regular value
    void update_source() { if (_source == src::PIN) _first_filter_run = true; } // if we just switched to pin input, the old filtered value is not valid
  public:
    Sensor(int pin) : Transducer(pin) {
        _long_name = "Unknown";
        _short_name = "unksen";
    }  
    virtual void setup() {
        _transtype = SensorType;
        Transducer::setup();
    }
    void set_ema_alpha(float arg_alpha) { _ema_alpha = arg_alpha; }
    float ema_alpha() { return _ema_alpha; }
    bool released() { return (std::abs(val() - _zeropoint) <= _margin); }
};

// Base class for sensors which communicate using i2c.
// NOTE: this is a strange type of Sensor, it's not really a Transducer but it does do filtering. maybe we should rethink the hierarchy a little?
//       I think we can move Param to common.h and add a class ExponentialMovingAverage there as well that just has the ema functionality, then make
//       I2CSensor a child of -> Device, ExponentialMovingAverage and not a Sensor at all.
class I2CSensor : public Sensor {
  protected:
    bool _detected = false;
    I2C* _i2c;
    // implement in child classes using the appropriate i2c sensor
    // virtual void set_val_from_pot() {
    //     _si.set(_pot->mapToRange(_si.min(), _si.max()));
    //     // _val_raw = _native.val();
    //     set_si(_si.val()); // don't filter the value we get from the pot, the pot output is already filtered
    // }
    // virtual void update_si_limits() {
    //     _native.set_limits(_si.min_shptr(), _si.max_shptr());  // Our two i2c sensors (airvelo & MAP) don't reveal low-level readings, so we only get si units
    //     _val_filt.set_limits(_si.min_shptr(), _si.max_shptr());
    // }
  public:
    uint8_t addr;
    I2CSensor(I2C* i2c_arg, uint8_t i2c_address_arg) : Sensor(-1), _i2c(i2c_arg), addr(i2c_address_arg) {
        _long_name = "Unknown I2C";
        _short_name = "unki2c";
        set_can_source(src::PIN, true);
    }
    I2CSensor() = delete;
    virtual void setup() {
        Sensor::setup();
        _detected = _i2c->detected_by_addr(addr);
    }
};
class AirVeloSensor : public I2CSensor {  // AirVeloSensor measures the air intake into the engine in MPH. It communicates with the external sensor using i2c.
  protected:
    FS3000 _sensor;
    float goodreading = NAN;
    int64_t airvelo_read_period_us = 35000;
    Timer airveloTimer;
    float read_sensor() {
        if (!_i2c->detected(i2c_airvelo)) return NAN;
        else if (_i2c->not_my_turn(i2c_airvelo)) return goodreading;
        else if (airveloTimer.expireset()) goodreading = _sensor.readMilesPerHour();  // note, this returns a float from 0-33.55 for the FS3000-1015 
        return goodreading;
    }
  public:
    static constexpr uint8_t addr = 0x28;
    sens senstype = sens::airvelo;
    AirVeloSensor(I2C* i2c_arg) : I2CSensor(i2c_arg, addr) {
        _long_name = "Manifold Air Velocity";
        _short_name = "airvel";
        _native_units = "mph";
        _si_units = "mph";
    }
    AirVeloSensor() = delete;

    void set_val_common() {
        if (_i2c->i2cbaton == i2c_airvelo) _i2c->pass_i2c_baton();
    }
    void setup() {  // ezread.squintf("%s..", _long_name.c_str());
        I2CSensor::setup();
        set_si(0.0);  // initialize value
        set_abslim(0.0, 33.55);  // set abs range. defined in this case by the sensor spec max reading
        set_oplim(0.0, 28.5);  // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * ((2 * 2.54) / 2)^2) 1/cm2 * 1/160934 mi/cm = 28.5 mi/hr (mph)            // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * (2.85 / 2)^2) 1/cm2 * 1/160934 mi/cm = 90.58 mi/hr (mph) (?!)  
        set_ema_alpha(0.2);  // note: all the conversion constants for this sensor are actually correct being the defaults 
        set_can_source(src::POT, true);
        airveloTimer.set(airvelo_read_period_us);
        print_config(true, false);
        ezread.squintf("  airvelo sensor %sdetected", _detected ? "" : "not ");
        if (_detected) {
            if (!_sensor.begin()) {
                ezread.squintf(", not responding\n");  // Begin communication with air flow sensor) over I2C 
                set_source(src::FIXED); // sensor is detected but not working, leave it in an error state ('fixed' as in not changing)
            }
            else {
                _sensor.setRange(AIRFLOW_RANGE_15_MPS);
                ezread.squintf(" and responding properly\n");
            }
        }
        else {
            ezread.squintf("\n");
            set_source(src::UNDEF); // don't even have a device at all...
        }
        if (_detected) print_config(false, true);
    }
};
class MAPSensor : public I2CSensor {  // MAPSensor measures the air pressure of the engine manifold in PSI. It communicates with the external sensor using i2c.
  protected:
    SparkFun_MicroPressure _sensor;
    float goodreading = NAN;
    Timer mapreadTimer;
    int mapread_timeout = 100000, mapretry_timeout = 10000;
    float read_sensor() {
        if (!_i2c->detected(i2c_map)) return NAN;
        else if (_i2c->not_my_turn(i2c_map)) return goodreading;
        else if (mapreadTimer.expired()) {
            float temp = _sensor.readPressure(ATM, true);  // _sensor.readPressure(PSI);  // <- blocking version takes 6.5ms to read
            if (!std::isnan(temp)) {
                goodreading = temp;
                mapreadTimer.set(mapread_timeout);
            }
            else mapreadTimer.set(mapretry_timeout);  // ezread.squintf("av:%f\n", goodreading);
        }
        return goodreading;
    }
  public:
    static constexpr uint8_t addr = 0x18;  // NOTE: would all MAPSensors have the same address?  ANS: yes by default, or an alternate fixed addr can be hardware-selected by hooking a pin low or something
    sens senstype = sens::mapsens;
    MAPSensor(I2C* i2c_arg) : I2CSensor(i2c_arg, addr) {
        _long_name = "Manifold Air Pressure";
        _short_name = "map";
        _native_units = "atm";
        _si_units = "atm";
    }
    MAPSensor() = delete;
    void set_val_common() {
        if (_i2c->i2cbaton == i2c_map) _i2c->pass_i2c_baton();
    }
    void setup() {
        // ezread.squintf("%s..", _long_name.c_str());
        I2CSensor::setup();
        set_si(1.0);  // initialize value
        set_abslim(0.06, 2.46);  // set abs range. defined in this case by the sensor spec max reading
        set_oplim(0.68, 1.02);  // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * ((2 * 2.54) / 2)^2) 1/cm2 * 1/160934 mi/cm = 28.5 mi/hr (mph)            // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * (2.85 / 2)^2) 1/cm2 * 1/160934 mi/cm = 90.58 mi/hr (mph) (?!)  
        set_ema_alpha(0.2);  // note: all the conversion constants for this sensor are actually correct being the defaults 
        set_can_source(src::POT, true);
        mapreadTimer.set(mapread_timeout);
        print_config(true, false);
        ezread.squintf("  map sensor %sdetected", _detected ? "" : "not ");
        if (_detected) {
            if (!_sensor.begin()) {
                ezread.squintf(", not responding\n");  // Begin communication with air flow sensor) over I2C 
                set_source(src::FIXED); // sensor is detected but not working, leave it in an error state ('fixed' as in not changing)
            }
            else ezread.squintf(" and reading %f atm\n", _sensor.readPressure(ATM));
        }
        else {
            ezread.squintf("\n");
            set_source(src::UNDEF); // don't even have a device at all...
        }
        if (_detected) print_config(false, true);
    }
};
class AnalogSensor : public Sensor {  // class AnalogSensor are sensors where the value is based on an ADC reading (eg brake pressure, brake actuator position, pot)
  protected:
    Timer read_timer{25000};  // adc cannot read too fast w/o errors, so give some time between readings
    void set_val_from_pin() {
        if (read_timer.expireset()) {
            set_native(static_cast<float>(analogRead(_pin)));  // Soren: can this be done without two casts?
            calculate_ema(); // filtered values are kept in si format
        }
    }
  public:
    AnalogSensor(int arg_pin) : Sensor(arg_pin) {
        _long_name = "Unknown Analog";
        _short_name = "analog";
        _native_units = "adc";
    }
    virtual void setup() {
        Sensor::setup();
        set_pin(_pin, INPUT);
        set_can_source(src::PIN, true);
        set_can_source(src::POT, true);
        set_source(src::PIN);
        set_abslim_native(0.0, (float)adcrange_adc, false);  // do not autocalc the si units because our math is not set up yet (is in child classes)
    }
};
class CarBattery : public AnalogSensor {  // CarBattery reads the voltage level from the Mule battery
  public:
    sens senstype = sens::mulebatt;
    CarBattery(int arg_pin) : AnalogSensor(arg_pin) {
        _long_name = "Vehicle Battery Voltage";
        _short_name = "mulbat";
        _native_units = "adc";
        _si_units = "V";
    }
    CarBattery() = delete;    
    void setup() {  // printf("%s..\n", _long_name.c_str());
        AnalogSensor::setup();
        set_conversions(0.00404, 0.0);  // 240605 m calculated from multimeter readings vs adc counts taken across a few samples
        set_abslim(0.0, 15.1);  // set abs range. dictated in this case by the max voltage a battery charger might output
        set_oplim(10.7, 13.9);  // set op range. dictated by the expected range of voltage of a loaded lead-acid battery across its discharge curve
        set_si(11.5);  // initialize value, just set to generic rest voltage of a lead-acid battery
        set_ema_alpha(0.2);  // note: all the conversion constants for this sensor are actually correct being the defaults 
        set_can_source(src::POT, true);
        print_config();
    }
    void set_val_from_touch() { set_si(12.0); }  // what exactly is going on here? maybe an attempt to prevent always showing battery errors on dev boards?
};
// PressureSensor represents a brake fluid pressure sensor.
// It extends AnalogSensor to handle reading an analog pin
// and converting the ADC value to a pressure value in PSI.
class PressureSensor : public AnalogSensor {
  public:
    sens senstype = sens::pressure;
    // int opmin_adc, opmax_adc, absmin_adc, absmax_adc; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
    // Soren 230920: Reducing max to value even wimpier than Chris' pathetic 2080 adc (~284 psi) brake press, to prevent overtaxing the motor
    float hold_initial, hold_increment, panic_initial, panic_increment;  // , _margin_psi, _zeropoint_psi;
    PressureSensor(int arg_pin) : AnalogSensor(arg_pin) {
        _long_name = "Brake Pressure";
        _short_name = "presur";
        _native_units = "adc";
        _si_units = "psi";
    }
    // the sensor output voltage spec range is 0.5-4.5 V to indicate 0-1000 psi.
    // Our ADC top is just 3.3V tho, so we can sense up to 695 psi (calculated) at our max adc
    // calculated using spec values:
    //   sensor gives:  1000 psi / (4.5 V - 0.5 V) = 250 psi/V   (or 0.004 V/psi)
    //   our adc:  4096 adc / 3.3 V = 1241 adc/V   (or 0.000806 V/adc)
    //   resulting slope:  250 psi/V * 0.000806 V/adc = 0.2 psi/adc  (or 5 adc/psi)
    //   zero psi point:  0.5 V * 1241 adc/V = 621 adc
    //   fullscale reading:  0.2 psi/adc * (4095 adc - 621 adc) = 695 psi
    // math to convert (based on spec values):
    //   psi = 0.2 * (adc - 621)  ->  psi = 0.2 * adc - 124  ->  adc = 5 * psi + 621

    // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
    // ~208psi by this math - "Maximum" braking  // older?  int max_adc = 2080; // ~284psi by this math - Sensor measured maximum reading. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as [wimp] chris can push
    // _absmin_adc = 0; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
    // _absmax_adc = adcrange_adc; // ~208psi by this math - "Maximum" braking  // older?  int max_adc = 2080; // ~284psi by this math - Sensor measured maximum reading. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as [wimp] chris can push
    PressureSensor() = delete;
    void setup() {
        AnalogSensor::setup();
        float min_adc = 658.0;  // temporarily hold this key value for use in mfactor/boffset calculations and op limit settings below (which must happen in that order) 
        float m = 1000.0 * (3.3 - 0.554) / (((float)adcrange_adc - min_adc) * (4.5 - 0.554)); // 1000 psi * (adc_max v - v_min v) / ((4095 adc - 658 adc) * (v-max v - v-min v)) = 0.2 psi/adc
        float b = -1.0 * min_adc * m;  // -658 adc * 0.2 psi/adc = -131.6 psi
        set_conversions(m, b);
        set_abslim_native(0.0, (float)adcrange_adc);  // set abslims after m and b are set
        set_oplim_native(min_adc, 2080.0);            // set oplims after abslims are set
        set_oplim(4.6, 350.0);  // 240605 these are the extremes seen with these settings. Is this line necessary though? (soren)
        // ezread.squintf(" | oplim_native = %lf, %lf | ", _opmin_native, _opmax_native);
        set_ema_alpha(0.15);
        set_margin(1.0);       // max acceptible error when checking psi levels
        hold_initial = 120.0;  // pressure applied when brakes are hit to auto-stop or auto-hold the car (adc count 0-4095)
        hold_increment = 3.0;  // incremental pressure added periodically when auto stopping (adc count 0-4095)
        panic_initial = 140.0; // pressure initially applied when brakes are hit to auto-stop the car (adc count 0-4095)
        panic_increment = 5.0; // incremental pressure added periodically when auto stopping (adc count 0-4095)
        _zeropoint = _opmin;   // used when releasing the brake in case position is not available
        set_native(_opmin_native);
        print_config();
    }
    float hold_initial_pc() { return to_pc(hold_initial); }  // pressure applied when brakes are hit to auto-stop or auto-hold the car (adc count 0-4095)
    float hold_increment_pc() { return to_pc(hold_increment); }  // incremental pressure added periodically when auto stopping (adc count 0-4095)
    float panic_initial_pc() { return to_pc(panic_initial); }  // pressure initially applied when brakes are hit to auto-stop the car (adc count 0-4095)
    float panic_increment_pc() { return to_pc(panic_increment); }  // incremental pressure added periodically when auto stopping (adc count 0-4095)
    bool parked() { return (std::abs(val() - _opmin) <= _margin); }  // is tha brake motor parked?
    float parkpos() { return _opmin; }
};
// BrakePositionSensor represents a linear position sensor
// for measuring brake position (TODO which position? pad? pedal?)
// Extends AnalogSensor for handling analog pin reading and conversion.
class BrakePositionSensor : public AnalogSensor {
  public:
    sens senstype = sens::brkpos;
    BrakePositionSensor(int arg_pin) : AnalogSensor(arg_pin) {
        _long_name = "Brake Position";
        _short_name = "brkpos";
        _native_units = "adc";
        _si_units = "in";
    }
    BrakePositionSensor() = delete;
    void setup() {
        AnalogSensor::setup();
        // printf("%s..\n", _long_name.c_str());
        _dir = TransDir::REV;  // causes percent conversions to use inverted scale 
        conversion_method = AbsLimMap;  // because using map conversions, need to set abslim for si and native separately, but don't need mfactor/boffset
        #if BrakeThomson
            set_abslim(0.335, 8.3, false);  // TUNED 230602
            set_abslim_native(979, 3103, false);  // NOT TUNED - these values stolen from LAE actuator below. needs tuning!
            set_oplim(0.506, 4.234)  // 4.624  //TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (in)
            _zeropoint = 3.179;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (in)
        #else  // if LAE motor
            // 240513 cal data LAE actuator:  measured to tip of piston
            // fully retracted 0.95 in, Vpot = 0.83 V (1179 adc), fully extended 8.85 in (), Vpot = 2.5 V (3103 adc)
            // calc (2.5 - 0.83) / 3.3 = 0.506 . 0.506 * 4096 = 2072 . (8.85 - 0.95) / 2072 = .00381 in/adc or 262 adc/in
            
            set_abslim_native(979, 3103, false);  // TUNED 240513 - don't remember if values read from screen or calculated.  need to redo
            
            // TUNED 240513 - absmin.  Retract value corresponding with the absolute minimum retract actuator is capable of (in)
            // TUNED 240513 - absmax.  Extend value corresponding with the absolute max extension actuator is capable of (in)
            set_abslim(0.95, 8.85, false);  // TUNED 240513 - actuator inches measured
            // TUNE. - opmin. Retract limit during nominal operation. Brake motor is prevented from pushing past this (in)
            // TUNE. - opmax. Best position to park the actuator out of the way so we can use the pedal (in)  
            set_oplim(2.703, 5.7);  // 240605 determined opmin on vehicle, with LAE motor connected w/ quicklink + carabeener
            _zeropoint = 5.5;  // TUNE. - inches Brake position value corresponding to the point where fluid PSI hits zero (in)
        #endif
        set_ema_alpha(0.35);
        set_margin(0.01);  // TODO: add description
        // _mfactor = (_absmax - _absmin) / (float)(_absmax_adc - _absmin_adc);  // (8.85 in - 0.95 in) / (3103 adc - 979 adc) = 0.00372 in/adc
        // _boffset = -2.69;  //  979 adc * 0.00372 in/adc - 0.95 in = -2.69 in
        print_config();
    }
    bool parked() { return (std::abs(val() - _opmax) <= _margin); }  // is tha brake motor parked?
    float parkpos() { return _opmax; }
};
// class PulseSensor are hall-effect based magnetic field sensors where the value is based on magnetic pulse timing of a rotational Source (eg tachometer, speedometer)
class PulseSensor : public Sensor {
  protected:
    // volatile int64_t timestamp_last_us;  // _stop_timeout_us = 1250000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
    Timer _stop_timer;
    bool _low_pulse = true, _pin_level, _pin_inactive = false;
    float _freqdiv = 1.0, _idle = 600.0, _idle_cold = 750.0, _idle_hot = 500.0;  // an external ripple counter divides pulse stream frequency by this, we need to compensate
    Timer pinactivitytimer{1500000};  // timeout we assume pin isn't active if no pulses occur
    volatile int64_t _isr_us = 0;
    volatile int64_t _isr_time_last_us = 0;
    volatile int64_t _isr_time_current_us = 0;
    volatile float _us;
    // we maintain our min and max pulse period, for each pulse sensor
    // absmax_us is the reciprocal of our native absmin value in MHz. once max_us has elapsed since the last pulse our si sets to zero
    // absmin_us is the reciprocal of our native absmax value in MHz. any pulse received within min_us of the previous pulse is ignored
    float _absmax_us = 1500000, _absmin_us = 6500; //  at min = 6500 us:   1000000 us/sec / 6500 us = 154 Hz max pulse frequency.  max is chosen just arbitrarily
    volatile int64_t _absmin_us_64 = (int64_t)_absmin_us;
    // Shadows a hall sensor being triggered by a passing magnet once per pulley turn. The ISR calls
    // esp_timer_get_time() on every pulse to know the time since the previous pulse. I tested this on the bench up
    // to about 0.750 mph which is as fast as I can move the magnet with my hand, and it works.
    // Update: Janky bench test appeared to work up to 11000 rpm.
    void IRAM_ATTR _isr() { // The isr gets the period of the vehicle pulley rotations.
        _isr_time_current_us = esp_timer_get_time();
        int64_t time_us = _isr_time_current_us - _isr_time_last_us;
        if (time_us >= _absmin_us_64) {  // ignore spurious triggers or bounces
            _isr_time_last_us = _isr_time_current_us;
            _us = time_us;
            // _pin_level = !_pin_level;
            _pin_level = read_pin(_pin);
        }
    }
    void set_pin_inactive() {
        static bool pinlevel_last;
        if (pinlevel_last != _pin_level) {
            pinactivitytimer.reset();
            _pin_inactive = false;
        }
        else if (pinactivitytimer.expired()) _pin_inactive = true;
        pinlevel_last = _pin_level;
    }
    virtual float read_sensor() {
        float _isr_buf_us = static_cast<float>(_us);  // Copy delta value (in case another interrupt happens during handling)
        float new_native = _native.val();  // initialize our return value to the current native value
        if (_isr_buf_us > _absmax_us) new_native = _native.min();  // if it's been too long since last pulse return zero
        else if (_isr_buf_us >= _absmin_us) new_native = us_to_hz(_isr_buf_us);  // otherwise if the pulse isn't too soon after the last one (possible bounce) then convert as a valid reading
        set_pin_inactive();
        return new_native;  // too-short pulse times are presumably bounces and are ignored, keeping the existing native value
    }
    float us_to_hz(float arg_us) {
        if (std::abs(arg_us) > float_zero) return 1000000.0 / arg_us;
        ezread.squintf("Err: us_to_hz() refusing to take reciprocal of zero\n");
        return absmax();
    }
    float hz_to_us(float arg_hz) {
        if (std::abs(arg_hz) > float_zero) return 1000000.0 / arg_hz;  // math is actually the same in both directions us -> hz or hz -> us
        ezread.squintf("Err: hz_to_us() refusing to take reciprocal of zero\n");
        return _absmax_us;
    }
  public:
    std::string _uber_native_units = "us";  // these pulse sensors actually deal in us, more native than Hz but Hz is compatible w/ our common conversion algos

    PulseSensor(int arg_pin, float arg_freqdiv=1.0) : Sensor(arg_pin), _freqdiv(arg_freqdiv) {
        _long_name = "Unknown Hall-Effect";
        _short_name = "pulsen";
        _native_units = "Hz";
    }
    PulseSensor() = delete;
    void print_config(bool header=true, bool ranges=true) {
        Transducer::print_config(header, ranges);
        if (ranges) ezread.squintf("  pulsewidth now = %.0lf %s, abs range: %.0lf-%.0lf %s\n", _us, _uber_native_units.c_str(), _absmin_us, _absmax_us, _uber_native_units.c_str());
    }
    // from our limits we will derive our min and max pulse period in us to use for bounce rejection and zero threshold respectively
    // overload the normal function so we can also include us calculations 
    void set_abslim_native(float arg_min, float arg_max, bool calc_si=true) {  // overload the normal function so we can also include us calculations 
        if ((!isnan(arg_min) && (std::abs(arg_min) <= float_zero)) || (!isnan(arg_max) && (std::abs(arg_max) <= float_zero))) {
            ezread.squintf("Err: pulse sensor can not have limit of 0\n");
            return;  // we can't accept 0 Hz for opmin
        }
        Transducer::set_abslim_native(arg_min, arg_max, calc_si);
        _absmax_us = 1000000.0 / _native.min();  // also set us limits from here, converting Hz to us. note min/max are swapped
        _absmin_us = 1000000.0 / _native.max();  // also set us limits from here, converting Hz to us. note min/max are swapped
        _absmin_us_64 = (int)_absmin_us;         // make an int copy for the isr to use conveniently
    }
    virtual void setup() {
        Sensor::setup();
        set_pin(_pin, INPUT_PULLUP);
        set_can_source(src::PIN, true);
        set_source(src::PIN);
        attachInterrupt(digitalPinToInterrupt(_pin), [this]{ _isr(); }, _low_pulse ? FALLING : RISING);
        set_can_source(src::POT, true);
    }
    // float last_read_time() { return _last_read_time_us; }
    // bool stopped() { return (esp_timer_get_time() - _last_read_time_us > _opmax_native); }  // Note due to weird float math stuff, can not just check if tach == 0.0
    bool stopped() { return (std::abs(val() - _opmin) <= _margin); }  // Note due to weird float math stuff, can not just check if tach == 0.0
    bool* pin_inactive_ptr() { return &_pin_inactive; }
    bool* pin_level_ptr() { return &_pin_level; }
    float absmin_us() { return _absmin_us; }
    float absmax_us() { return _absmax_us; }
    float absmin_ms() { return _absmin_us / 1000.0; }
    float absmax_ms() { return _absmax_us / 1000.0; }
    float idle() { return _idle; }
    float* idle_ptr() { return &_idle; }
    void set_idle(float newidle) { _idle = constrain(newidle, _opmin, _opmax); }
    float us() { return _us; }
    float ms() { return _us / 1000.0; }
};
// Tachometer represents a magnetic pulse measurement of the enginge rotation.
// It extends PulseSensor to handle reading a hall monitor sensor and converting RPU to RPM
class Tachometer : public PulseSensor {
  public:
    sens senstype = sens::tach;
    Tachometer(int arg_pin, float arg_freqdiv) : PulseSensor(arg_pin, arg_freqdiv) {
        _long_name = "Tachometer";
        _short_name = "tach";
        _native_units = "Hz";
        _si_units = "rpm";
    }
    Tachometer() = delete;
    void setup() {  // printf("%s..\n", _long_name.c_str());
        PulseSensor::setup();
        float m = 60.0 * _freqdiv;  // 1 Hz = 1 pulse/sec * 8 rot/pulse * 60 sec/min = 480 rot/min, (so 480 rpm/Hz)
        set_conversions(m, 0.0);
        set_abslim(0.0, 4500.0);  // the max readable engine speed also defines the pulse debounce rejection threshold. the lower this speed, the more impervious to bouncing we are
        // do i need this line?
        // set_abslim_native(us_to_hz(6000), NAN, false);  // for pulse sensor, set absmin_native to define the stop timeout period. Less Hz means more us which slows our detection of being stopped
        
        set_oplim(0.0, 3600.0);  // aka redline,  Max possible engine rotation speed (tunable) corresponds to 1 / (3600 rpm * 1/60 min/sec) = 60 Hz
        set_ema_alpha(0.015);  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
        set_margin(10.0);
        set_si(50.0);
        _us = hz_to_us(_native.val());
        print_config();
    }
    // float idle() { return _idle; }
    // float* idle_ptr() { return &_idle; }
    float idle_cold() { return _idle_cold; }
    float idle_hot() { return _idle_hot; }
    void set_idlecold(float newidlecold) { _idle_cold = constrain(newidlecold, _idle_hot + 1.0, _opmax); }
    void set_idlehot(float newidlehot) { _idle_hot = constrain(newidlehot, _opmin, _idle_cold - 1.0); }
};

// Speedometer represents a magnetic pulse measurement of the enginge rotation.
// It extends PulseSensor to handle reading a hall monitor sensor and converting RPU to MPH
class Speedometer : public PulseSensor {
  public:
    sens senstype = sens::speedo;
    Speedometer(int arg_pin, float arg_freqdiv) : PulseSensor(arg_pin, arg_freqdiv) {
        _long_name = "Speedometer";
        _short_name = "speedo";
        _native_units = "Hz";
        _si_units = "mph";
    }
    Speedometer() = delete;
    void setup() {
        PulseSensor::setup();
        // printf("%s..\n", _long_name.c_str());
        float m = 3600.0 * 20 * M_PI * _freqdiv / (2 * 12 * 5280);  // 1 Hz = 1 pulse/sec * 3600 sec/hr * 1/2 whlrot/pulse * 20*pi in/whlrot * 1/12 ft/in * 1/5280 mi/ft = 1.785 mi/hr,  (so 1.8 mph per Hz)
        set_conversions(m, 0.0);
        set_abslim(0.0, 25.0);  // the max readable vehicle speed also defines the pulse debounce rejection threshold. the lower this speed, the more impervious to bouncing we are

        // do i need this line?
        // set_abslim_native(us_to_hz(10000), NAN, false);  // for pulse sensor, set absmin_native to define the stop timeout period. Less Hz means more us which slows our detection of being stopped

        set_oplim(0.0, 15.0);  // aka redline,  Max possible engine rotation speed (tunable) corresponds to 1 / (3600 rpm * 1/60 min/sec) = 60 Hz
        set_ema_alpha(0.015);  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
        set_margin(0.2);
        set_si(50.0);
        _us = hz_to_us(_native.val());
        _idle = 3.0;  // estimate of speed when idling forward on flat ground (in mph)
        print_config();
    }
};
// RCChannel is a class for the channels of the hotrc
// it contains the association with the RMT channels and probably filtering
// I imagine it might make sense to have two child classes, one for the two toggle channels, and one for the two analog channels
// Anyway eventually we'd like to move channel input related functionality from hotrc class into these classes, 
// then hotrc contains four instances, one per channel, which it can manage
class RCChannel : public Sensor {  // class for each channel of the hotrc
  protected:
    virtual float read_sensor() {
        // return new_native;  // too-short pulse times are presumably bounces and are ignored, keeping the existing native value
        return NAN;
    }
  public:
    RCChannel(int arg_pin) : Sensor(arg_pin) {
        _long_name = "RC Channel";
        _short_name = "rcchan";
        _native_units = "us";
        _si_units = "%";
    }
    RCChannel() = delete;
    virtual void setup() {
        Sensor::setup();
        set_pin(_pin, INPUT);
        set_can_source(src::PIN, true);
        set_source(src::PIN);
    }
};
class RCToggle : public RCChannel {};
class RCAnalog : public RCChannel {};

// NOTE: I implemented the gas servo, but it looks like it's all in native units. should it still be a transducer?
// ServoMotor is a base class for our type of actuators, where by varying a pulse width (in us), motors move.
//    e.g. the gas, brake and steering motors. The gas motor is an actual servo, the others are controlled with servo signaling via jaguars.
// template<typename float, typename float>
class ServoMotor2 : public Transducer {
  protected:
    Servo motor;
    Timer updatetimer{85000}, outchangetimer;
    float lastoutput, max_out_change_rate_pcps = 800.0;
    int _pin, _freq;
    virtual float write_sensor() {  // NOTE: should be marked 'override' but compiler says it doesn't override anything...?
        ezread.squintf("Err: %s does not have an overridden write_sensor() function\n", _short_name.c_str());
        return NAN;
    }
    void changerate_limiter() {
        float max_out_change_pc = max_out_change_rate_pcps * outchangetimer.elapsed() / 1000000.0;
        outchangetimer.reset();
        set_si(constrain(_si.val(), lastoutput - max_out_change_pc, lastoutput + max_out_change_pc));
        // lastoutput = pc[OUT];  // NOTE you must set lastoutput = pc[OUT]
    }
  public:
    // ServoMotor(int pin, int freq) : Transducer<float, float>(pin) {
    ServoMotor2(int pin, int freq) : Transducer(pin), _freq(freq) {
        set_pin(_pin, OUTPUT);   // needed?
        _transtype = ActuatorType;
        motor.setPeriodHertz(_freq);
        motor.attach(_pin, _native.min(), _native.max());  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
        _long_name = "Unknown PWM motor";
        _short_name = "pwmout";
        _native_units = "us";
    }
    ServoMotor2() = delete;
    void setup() {}
    void write() {
        if (!std::isnan(_native.val())) motor.writeMicroseconds((int)(_native.val()));
        lastoutput = _native.val();    
        // _val_raw = _native.val();
        // _servo.writeMicroseconds((int)_val_raw);  // Write result to servo interface
    }
    float max_changerate() { return max_out_change_rate_pcps; };
    void set_max_changerate(float a_newrate) { max_out_change_rate_pcps = a_newrate; };
};
class Jaguar : public ServoMotor2 {
  public:
    Jaguar(int pin, int freq) : ServoMotor2(pin, freq) {
        conversion_method = OpLimMap;
        _long_name = "Unknown Jaguar controller";
        _short_name = "unkjag";
        _si_units = "V";
    }
    Jaguar() = delete;
    void setup() {
        set_abslim(45.0, 168.2);  //this should also set oplim to the same
        float m = (_si.max() - _si.min()) / (_native.max() / _native.min());  // (180 - 0) / (2500 - 500) = 0.09 deg/us
        set_abslim(0.0, 180.0, false);
    }
};
class ThrottleServo2 : public ServoMotor2 {
  protected:
    Param governor_pc, idle_si, idletemp_f;
    float max_throttle_angular_velocity_pcps;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
    
    // float idle_si[NUM_MOTORVALS] = { 45.0, NAN, 60.0, 58.0, NAN, 43.0, 75.0, 1.0 };          // in angular degrees [OPMIN(hot)/-/OPMAX(cold)/OUT/-/ABSMIN/ABSMAX/MARGIN]
    // float idletemp_f[NUM_MOTORVALS] = { 60.0, NAN, 205.0, 75.0, NAN, 40.0, 225.0, 1.5};      // in degrees F [OPMIN/-/OPMAX/OUT/-/ABSMIN/ABSMAX/MARGIN]
    float idle_pc = 11.3;                              // idle percent is derived from the si (degrees) value
    float starting_pc = 25.0;                          // percent throttle to open to while starting the car
  public:
    ThrottleServo2(int pin, int freq) : ServoMotor2(pin, freq) {
        _dir = TransDir::FWD;  // if your servo goes CCW with increasing pulsewidths, change to REV
        _long_name = "Throttle servo";
        _short_name = "throtl";
        _si_units = "deg";
    }
    ThrottleServo2() = delete;
    void setup() {
        set_abslim(0.0, 180.0, false);
        set_abslim_native(500.0, 2500.0, false);
        float m = (_si.max() - _si.min()) / (_native.max() / _native.min());  // (180 - 0) / (2500 - 500) = 0.09 deg/us
        set_conversions(m, 0.0);
        governor_pc.set(95);
        idle_si.set_limits(43.0, 75.0);
        idle_si.set(58.0);

    }
        // set_oplim_native(1000.0, 2000.0, false);       
        // jaguar range in degrees: (45.0, 168.2);
};
class BrakeMotor2 : public Jaguar {
  public:
    BrakeMotor2(int pin, int freq) : Jaguar(pin, freq) {
        _long_name = "Brake motor";
        _short_name = "brake";
    }
    BrakeMotor2() = delete;
};
class SteerMotor2 : public Jaguar {
  public:
    SteerMotor2(int pin, int freq) : Jaguar(pin, freq) {
        _long_name = "Steering motor";
        _short_name = "steer";
    }
    SteerMotor2() = delete;
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
    Preferences* _myprefs;
  public:
    Simulator(Potentiometer& pot_arg, Preferences* myprefs) : _pot(pot_arg), _myprefs(myprefs) {
        for (int sensor = (int)sens::none + 1; sensor < (int)sens::NUM_SENSORS; sensor++) set_can_sim((sens)sensor, false);   // initially turn off simulation of sensors  // static constexpr bool initial_sim_joy = false;
        set_potmap(); // set initial pot map
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
    void enable() { updateSimulationStatus(true); }            // turn on the simulator. all components which are set to be simulated will switch to simulated input
    void disable() { updateSimulationStatus(false); }          // turn off the simulator. all devices will be set to their default input (if they are not being mapped from the pot)
    void toggle() { return _enabled ? disable() : enable(); }  // check if a componenet is currently being simulated (by either the touchscreen or the pot)
    bool simulating() { return _enabled; }                     // equivalent to enabled()  // Maybe include || potmapping() too ?
    bool simulating(sens arg_sensor) { return can_sim(arg_sensor) && (_enabled || _potmap == arg_sensor); }
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
  private:
    void set_can_sim_nosave(sens arg_sensor, bool can_sim) {  // set a device so simulator will include it when enabled. does not write to flash
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
                        }
                        else if (_enabled) { // ...and the pot isn't mapping to this component, but the simulator is running...
                            d->set_source(src::TOUCH); // ...then set the input source for the associated Device to read from the touchscreen
                        }
                    }
                    else {
                        d->set_source(default_mode); // we disabled simulation for this component, set it back to its default input source
                    }
                }
                kv->second = simulable_t(can_sim, d, default_mode); // update the entry with the new simulatability status
            }
        }
        else {
            _devices[arg_sensor] = simulable_t(can_sim, nullptr, src::UNDEF); // add a new entry with the simulatability status for this component
        }
    }
  public:
    void set_can_sim(sens arg_sensor, bool can_sim) {  // this wrapper function sets a device as able to be simulated, then store to flash
        set_can_sim_nosave(arg_sensor, can_sim);  // set the device simulatability status
        save_cansim();  // re-save can-sim status word to flash (makes setting permanent across boots)
    }
    void set_can_sim(sens arg_sensor, int can_sim) { 
        set_can_sim(arg_sensor, (can_sim > 0));  // allows interpreting -1 as 0, convenient for our tuner etc.
    }
    void save_cansim() {  // compress can_sim status of all devices into a 32 bit int, and save it to flash
        uint32_t simword = 0;
        for (int s=1; s<(int)sens::NUM_SENSORS; s++) {
            simword = simword | ((uint32_t)can_sim((sens)s) << s);
        }
        _myprefs->putUInt("cansim", simword);
    }
    void recall_cansim() {  // pull 32 bit int containing can_sim status of all devices from previous flash save, and set all devices accordingly
        uint32_t simword = _myprefs->getUInt("cansim", 0);
        for (int s=1; s<(int)sens::NUM_SENSORS; s++) {
            set_can_sim_nosave((sens)s, (bool)((simword >> s) & 1));
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
            _myprefs->putUInt("potmap", static_cast<uint32_t>(_potmap));
        }
    }
    void set_potmap() { 
        set_potmap(static_cast<sens>(_myprefs->getUInt("potmap", static_cast<uint32_t>(sens::none))));
    }
    // Getter functions
    bool potmapping(sens s) { return can_sim(s) && _potmap == s; }  // query if a certain sensor is being potmapped
    bool potmapping(int s) { return can_sim(static_cast<sens>(s)) && (_potmap == static_cast<sens>(s)); }  // query if a certain sensor is being potmapped        
    bool potmapping() { return can_sim(_potmap) && !(_potmap == sens::none); }  // query if any sensors are being potmapped
    int potmap() { return static_cast<int>(_potmap); }  // query which sensor is being potmapped
    bool enabled() { return _enabled; }
    bool* enabled_ptr() { return &_enabled; }
};
class RMTInput {
  public:
    RMTInput(rmt_channel_t channel, gpio_num_t gpio) {
        channel_ = channel;
        gpio_ = gpio;
    }
    void init() {
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
        if (config_result != ESP_OK) {
            ezread.squintf("Failed to configure RMT: %d\n", config_result);
            // while (1); // halt execution
        }
        esp_err_t install_result = rmt_driver_install(channel_, 2000, 0);
        if (install_result != ESP_OK) {
            ezread.squintf("Failed to install RMT driver: %d\n", install_result);
            // while (1); // halt execution
        }
        rmt_get_ringbuf_handle(channel_, &rb_);
        if (rb_ == NULL) {
            Serial.println("Failed to initialize ring buffer");
            // while (1); // halt execution
        }
        esp_err_t start_result = rmt_rx_start(channel_, 1);
        if (start_result != ESP_OK) {
            ezread.squintf("Failed to start RMT receiver: %d\n", start_result);
            // while (1); // halt execution
        }
    }
    int readPulseWidth(bool persistence) { // persistence means the last reading will be returned until a newer one is gathered. Otherwise 0 if no reading
        size_t rx_size = 0;
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb_, &rx_size, 0);
        if (item != NULL && rx_size == sizeof(rmt_item32_t)) {
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
    int pulse_width, pulse_width_last;
    float scale_factor = 0.625;
    RingbufHandle_t rb_;
};
class Hotrc {  // All things Hotrc, in a convenient, easily-digestible format the kids will just love
  public:
    float ema_alpha = 0.065;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1).
    float pc[NUM_AXES][NUM_VALUS];           // values range from -100% to 100% are all derived or auto-assigned
    int us[NUM_CHANS][NUM_VALUS] = {
        // vals for hotrc v1 (with matte black "HotRC" sticker/receiver)
        // {  971, 1470, 1968, 0, 1500, 0, 0, 0 },     // 1000-30+1, 1500-30,  2000-30-2   // [HORZ] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        // { 1081, 1580, 2078, 0, 1500, 0, 0, 0 },     // 1000+80+1, 1500+80,  2000+80-2,  // [VERT] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        // { 1151, 1500, 1848, 0, 1500, 0, 0, 0 },     // 1000+150+1,   1500, 2000-150-2,  // [CH3] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        // { 1251, 1500, 1748, 0, 1500, 0, 0, 0 }, };  // 1000+250+1,   1500, 2000-250-2,  // [CH4] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        // vals for hotrc v2 (with gloss black "HotRC" sticker/receiver)
        {  973, 1477, 1960, 0, 1500, 0, 0, 0 },     // (974-1981) 1000-30+1, 1500-30,  2000-30-2   // [HORZ] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        { 1081, 1577, 2072, 0, 1500, 0, 0, 0 },     // (1084-2091) 1000+80+1, 1500+80,  2000+80-2,  // [VERT] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        { 1202, 1606, 1806, 0, 1500, 0, 0, 0 },     // (1204-1809) 1000+150+1,   1500, 2000-150-2,  // [CH3] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
        { 1304, 1505, 1705, 0, 1500, 0, 0, 0 }, };  // (1304-1707) 1000+250+1,   1500, 2000-250-2,  // [CH4] [OPMIN/CENT/OPMAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
    float ema_us[NUM_AXES] = { 1500.0, 1500.0 };  // [HORZ/VERT]
    int absmin_us = 880;
    int absmax_us = 2091;
    int deadband_us = 15;  // All [DBBOT] and [DBTOP] values above are derived from this by calling calc_params()
    int margin_us = 13;  // All [MARGIN] values above are derived from this by calling calc_params()
    int failsafe_us = 880; // Hotrc must be configured per the instructions: search for "HotRC Setup Procedure"
    int failsafe_margin_us = 100; // in the carpet dumpster file: https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA/edit
    int failsafe_pad_us = 10;
  private:
    Simulator* sim;
    Potentiometer* pot;
    static const int failsafe_timeout = 15000;
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
    int spike_length, this_delta, interpolated_slope, loopindex, previndex, spike_cliff[NUM_AXES];
    int spike_threshold[NUM_AXES] = { 6, 6 };
    int prespike_index[NUM_AXES] = { -1, -1 };
    int index[NUM_AXES] = { 1, 1 };  // index is the oldest values are popped from then new incoming values pushed in to the LIFO
    static const int depth = 9;  // more depth will reject longer spikes at the expense of controller delay
    int raw_history[NUM_AXES][depth], filt_history[NUM_AXES][depth];  // Values before and after filtering.
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
        float m_factor;
        for (int axis=HORZ; axis<=VERT; axis++) {
            us[axis][DBBOT] = us[axis][CENT] - deadband_us;
            us[axis][DBTOP] = us[axis][CENT] + deadband_us;
            us[axis][MARGIN] = margin_us;
            pc[axis][OPMIN] = -100.0;
            pc[axis][CENT] = 0.0;
            pc[axis][OPMAX] = 100.0;
            m_factor = (pc[axis][OPMAX] - pc[axis][OPMIN]) / (float)(us[axis][OPMAX] - us[axis][OPMIN]);
            pc[axis][DBBOT] = pc[axis][CENT] - deadband_us * m_factor;
            pc[axis][DBTOP] = pc[axis][CENT] + deadband_us * m_factor;
            pc[axis][MARGIN] = margin_us * m_factor;
        }
    }
    void update() {
        toggles_update();
        if (!(sim->simulating(sens::joy))) direction_update();
        radiolost_update();
    }
    void toggles_reset() {  // Shouldn't be necessary to reset events due to sw_event(ch) auto-resets when read
        for (int ch = CH3; ch <= CH4; ch++) _sw_event[ch] = false;
    }
    bool radiolost() { return _radiolost; }
    bool* radiolost_ptr() { return &_radiolost; }
    bool sw_event(int ch) {  // returns if there's an event on the given channel then resets that channel
        bool retval = _sw_event[ch];
        _sw_event[ch] = false;
        return retval;        
    }
    void set_pc(int axis, int param, float val) { pc[axis][param] = val; }
    void set_us(int axis, int param, int val) { us[axis][param] = val; }
    int next_unfilt_rawval (int axis) { return raw_history[axis][index[axis]]; }  // helps to debug the filter from outside the class
    int joydir(int axis = VERT) {
        if (axis == VERT) return ((pc[axis][FILT] > pc[axis][DBTOP]) ? JOY_UP : ((pc[axis][FILT] < pc[axis][DBBOT]) ? JOY_DN : JOY_CENT));
        return ((pc[axis][FILT] > pc[axis][DBTOP]) ? JOY_RT : ((pc[axis][FILT] < pc[axis][DBBOT]) ? JOY_LT : JOY_CENT));
    }  // return (pc[axis][FILT] > pc[axis][DBTOP]) ? ((axis == VERT) ? JOY_UP : JOY_RT) : (pc[axis][FILT] < pc[axis][DBBOT]) ? ((axis == VERT) ? JOY_DN : JOY_LT) : JOY_CENT;
  private:
    void toggles_update() {  //
        for (int chan = CH3; chan <= CH4; chan++) {
            us[chan][RAW] = (int)(rmt[chan].readPulseWidth(true));
            sw[chan] = (us[chan][RAW] <= us[chan][CENT]); // Ch3 switch true if short pulse, otherwise false  us[CH3][CENT]
            if ((sw[chan] != sw[chan-2]) && !_radiolost) {
                _sw_event[chan] = true; // So a handler routine can be signaled. Handler must reset this to false. Skip possible erroneous events while radio lost, because on powerup its switch pulses go low
                kick_inactivity_timer(HURCTog);  // evidence of user activity
            }
            sw[chan-2] = sw[chan];  // chan-2 index being used to store previous values of index chan
        }
    }
    float us_to_pc(int axis, int _us, bool deadbands=false) {
        if (deadbands) {
            if (_us >= us[axis][DBTOP]) return map((float)_us, (float)us[axis][DBTOP], (float)us[axis][OPMAX], pc[axis][CENT], pc[axis][OPMAX]);
            if (_us <= us[axis][DBBOT]) return map((float)_us, (float)us[axis][DBBOT], (float)us[axis][OPMIN], pc[axis][CENT], pc[axis][OPMIN]);
            return pc[axis][CENT];
        }
        if (_us >= us[axis][CENT]) return map((float)_us, (float)us[axis][CENT], (float)us[axis][OPMAX], pc[axis][CENT], pc[axis][OPMAX]);
        return map((float)_us, (float)us[axis][CENT], (float)us[axis][OPMIN], pc[axis][CENT], pc[axis][OPMIN]);    
    }
    void direction_update() {
        if (sim->simulating(sens::joy)) {
            if (sim->potmapping(sens::joy)) pc[HORZ][FILT] = pot->mapToRange(pc[HORZ][OPMIN], pc[HORZ][OPMAX]);  // overwrite horz value if potmapping
            // ezread.squintf("%d %d %lf\n",sim->potmapping(sens::joy),sim->potmapping(), pc[HORZ][FILT]);
        }
        else for (int axis = HORZ; axis <= VERT; axis++) {  // read pulses and update filtered percent values
            us[axis][RAW] = (int)(rmt[axis].readPulseWidth(true));
            int us_spike = spike_filter(axis, us[axis][RAW]);
            ema_filt(us_spike, &us[axis][FILT], ema_alpha);
            pc[axis][RAW] = us_to_pc(axis, us[axis][RAW], false);
            pc[axis][FILT] = us_to_pc(axis, us[axis][FILT], true);
            if (_radiolost) pc[axis][FILT] = pc[axis][CENT];  // if radio lost set joy_axis_filt to CENTer value
            else if (std::abs(pc[axis][FILT] - pc[axis][CENT]) > pc[axis][MARGIN]) kick_inactivity_timer(HURCTrig);  // indicate evidence of user activity
        }
        for (int axis = HORZ; axis <= VERT; axis++) pc[axis][FILT] = constrain(pc[axis][FILT], pc[axis][OPMIN], pc[axis][OPMAX]);
    }
    bool radiolost_update() {
        if (us[VERT][FILT] > failsafe_us + failsafe_margin_us) {
            failsafe_timer.reset();
            _radiolost = false;
        }
        else if (failsafe_timer.expired()) _radiolost = true;
        return _radiolost;
    }
    // Spike filter pushes new hotrc readings into a LIFO ring buffer, replaces any well-defined spikes with values 
    // interpolated from before and after the spike. Also smoothes out abrupt value changes that don't recover later
    int spike_filter (int axis, int new_val) {  // pushes next val in, massages any detected spikes, returns filtered past value
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
        int returnval = filt_history[axis][index[axis]];  // Save the incumbent value at current index (oldest value) into buffer
        filt_history[axis][index[axis]] = new_val;
        raw_history[axis][index[axis]] = new_val;
        ++(index[axis]) %= depth;  // Update index for next time
        return returnval;  // Return the saved old value
    }
    void inject_interpolations(int axis, int endspike_index, int endspike_val) {  // Replaces values between indexes with linear interpolated values
        spike_length = ((depth + endspike_index - prespike_index[axis]) % depth) - 1;  // Equal to the spiking values count plus one
        if (!spike_length) return;  // Two cliffs in the same direction on consecutive readings needs no adjustment, also prevents divide by zero 
        interpolated_slope = (endspike_val - filt_history[axis][prespike_index[axis]]) / spike_length;
        loopindex = 0;
        while (++loopindex <= spike_length)
            filt_history[axis][(prespike_index[axis] + loopindex) % depth] = filt_history[axis][prespike_index[axis]] + loopindex * interpolated_slope;
    }
};