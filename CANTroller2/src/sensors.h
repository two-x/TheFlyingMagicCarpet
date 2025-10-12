#pragma once
#include <iostream>
#include <map>
#include <memory> // for std::shared_ptr
#include <SparkFun_FS3000_Arduino_Library.h>  // for air velocity sensor  http://librarymanager/All#SparkFun_FS3000
#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include "driver/rmt.h"
#include <ESP32Servo.h>        // makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <Preferences.h>  // functions for writing values to nvs flash partition

// this enum class represent the components which can be simulated (sensor). It's int type under the covers, so it can be used as an index
// typedef int opt_t;
enum si_native_convmethods { LinearMath=0, AbsLimMap=1, OpLimMap=2 };
enum class sens : int { none=0, joy=1, pressure=2, brkpos=3, speedo=4, tach=5, airvelo=6, mapsens=7, engtemp=8, mulebatt=9, starter=10, basicsw=11, NumSensors=12 };  //, ignition, syspower };  // , NumSensors, err_flag };
enum class src : int { Undef=0, Fixed=1, Pin=2, Sim=3, Pot=4, Calc=5 };

int sources[static_cast<int>(sens::NumSensors)] = { static_cast<int>(src::Undef) };

// Potentiometer does an analog read from a pin and maps it to a percent (0%-100%). We filter the value to keep it smooth.
class Potentiometer {
  protected:
    float _ema_alpha = 0.2;
    float _opmin = 0.0, _opmax = 100.0 ;  // in percent
    float _opmin_native = 380; // TUNED 230603 - used only in determining theconversion factor
    float _opmax_native = 4095; // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
    float _absmin_native = 0.0, _absmax_native = static_cast<float>(adcrange_adc);
    float _absmin = 0.0, _absmax = 100.0;
    float _margin_pc = 5.0;
    int _pin;
    float _pc = 50.0, _raw, _native, _activity_ref;  // values in filtered percent, raw percent, raw adc
    Timer pot_timer{100000};  // adc cannot read too fast w/o errors, so give some time between readings
  public:
    // tune these by monitoring adc_raw and twist pot to each limit. set min at the highest value seen when turned full CCW, and vice versas for max, then trim each toward adc_midrange until you get full 0 to 100% range
    Potentiometer(int arg_pin) : _pin(arg_pin) {}
    Potentiometer() = delete; // must have a pin defined
    sens _senstype = sens::none;
    void setup() {
        ezread.squintf(ezread.highlightcolor, "Potentiometer init\n");
        set_pin(_pin, INPUT);
        _activity_ref = _pc;
    }
    void update() {
        if (pot_timer.expireset()) {
            _native = static_cast<float>(analogRead(_pin));
            _raw = map(_native, _opmin_native, _opmax_native, _opmin, _opmax);
            _pc = ema_filt(_raw, _pc, _ema_alpha);
            _pc = constrain(_pc, _absmin, _absmax); // the lower limit of the adc reading isn't steady (it will dip below zero) so constrain it back in range
            // Serial.printf("r: %.6lf f: %.6lf\n", _raw, _pc);
            if (std::abs(_pc - _activity_ref) > _margin_pc) {
                // ezread.squintf("a:%ld n:%lf v:%lf r:%lf m:%lf ", adc_raw, new_val, _val, _activity_ref, _pc_activity_margin);
                kick_inactivity_timer(HuPot);  // evidence of user activity
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
    float absmin_native() { return _absmin_native; }
    float absmax_native() { return _absmax_native; }
    float margin() { return _margin_pc; }
    float* ptr() { return &_pc; }
    float* raw_ptr() { return &_raw; }
    float* native_ptr() { return &_native; }
    float* opmin_ptr() { return &_opmin; }
    float* opmax_ptr() { return &_opmax; }
    float* opmin_native_ptr() { return &_opmin_native; }
    float* opmax_native_ptr() { return &_opmax_native; }    
    float* margin_ptr() { return &_margin_pc; }

};
// note: the following classes all contain their own initial config values (for simplicity). We could instead use Config objects and pass them in at construction time, which might be
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
    Param(float arg_val, float arg_min, float arg_max) {  // creates a regular constrained Param
        _val = arg_val;
        *_min_ptr = arg_min;
        *_max_ptr = arg_max;
        set_limits(*_min_ptr, *_max_ptr);
        _last = _val;
    } 
    Param(float arg_val) { Param(arg_val, arg_val, arg_val); }  // creates a constant Param (with min/max == val) which cannot be changed
    Param(float arg_val, float* arg_min_ptr, float* arg_max_ptr) {  // creates a constraned Param which uses external limits.
        _val = arg_val;
        _min_ptr = arg_min_ptr;
        _max_ptr = arg_max_ptr;
        set_limits(*_min_ptr, *_max_ptr);
        _last = _val;
    }
    // note: if using external limits, it's (currently) possible to get stale values, since there is no
    //       callback mechanism in place. We could get around this by calling constrain_value() on every
    //       get() call, but that seems like overkill...
    void set_limits(float arg_min, float arg_max) {  // use if min/max are kept in-class
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
        bool ret = true;
        _last = _val;
        if (std::isnan(arg_val) && std::isnan(_last)) ret = false;
        else {
            cleanzero(&arg_val);  // avoid stupidly low near-zero values that happens sometimes
            if (iszero(_val - arg_val)) ret = false;
        }
        _val = arg_val;
        if (!std::isnan(_val)) constrain_value();
        return ret;
    }
    // bool add(float arg_add) { return set(_val + arg_add); }
    float val() { return _val; }
    float min() { return *_min_ptr; }
    float max() { return *_max_ptr; }
    float* ptr() { return &_val; }
    float* min_ptr() { return _min_ptr; }
    float* max_ptr() { return _max_ptr; }
    float last() { return _last; } // note: currently unused, do we still need this for drawing purposes?
};
// Device class - is a base class for any connected system device or signal associated with a pin
class Device {
  protected:
    // which types of sources are possible for this device?
    int _pin;
    // bool _enabled = true;
    int val_refresh_period = 5000;  // minimum delay between value updates, just to be efficient w/ processing. (overload this per device)
    Timer valrefreshtimer;
    Potentiometer* _pot; // to pull input from the pot if we're in simulation mode
    src _source = src::Undef;
    bool _can_source[6] = { true, true, false, true, false, false };  // [Undef/Fixed/Pin/Sim/Pot/Calc]
    // source handling functions (should be overridden in child classes as needed)
    virtual void set_val_from_undef() {}
    virtual void set_val_from_fixed() {}
    virtual void set_val_from_pin() {}
    virtual void set_val_from_sim() {}
    virtual void set_val_from_pot() {}
    virtual void set_val_from_calc() {}
    virtual void set_val_common() {}  // runs when setting val from any source, after one of the above
    virtual void update_source() {}
  public:
    std::string _long_name = "Unknown device";
    std::string _short_name = "device";
    std::string _native_units, _si_units;
    Timer timer;  // can be used for external purposes
    sens _senstype = sens::none;
    Device() = delete; // should always be created with a pin
    // note: should we start in Pin mode?
    Device(int arg_pin) : _pin(arg_pin) {
        valrefreshtimer.set(val_refresh_period);
    }
    bool can_source(src arg_source) { return _can_source[static_cast<int>(arg_source)]; }
    bool set_source(src arg_source) {
        if (_can_source[static_cast<int>(arg_source)]) {
            _source = arg_source;
            update_source();
            sources[static_cast<int>(_senstype)] = static_cast<int>(arg_source);
            return true;
        } 
        return false;
    }
    void update() {  // I changed case statement into if statements and now is 10 lines long instead of 28.  case sucks like that
        if (valrefreshtimer.expired()) {
            valrefreshtimer.set(val_refresh_period);  // allows for dynamic adjustment of update period as needed each device
            if (runmode == LowPower) return; // do nothing if in lowpower mode or this device is disabled
            // if (!_enabled) return; // do nothing if in lowpower mode or this device is disabled
            if (_source == src::Undef) set_val_from_undef();
            else if (_source == src::Fixed) set_val_from_fixed();
            else if (_source == src::Pin) set_val_from_pin();
            else if (_source == src::Sim) set_val_from_sim();
            else if (_source == src::Pot) set_val_from_pot();
            else if (_source == src::Calc) set_val_from_calc();
            else ezread.squintf("invalid Device source: %d\n", _source);
            set_val_common();
        }
    }
    void attach_pot(Potentiometer &pot_arg) {
        _pot = &pot_arg;
    }
    // void set_enabled(bool arg_enable) { _enabled = arg_enable; }
    void set_can_source(src arg_source, bool is_possible) { _can_source[static_cast<int>(arg_source)] = is_possible; }
    src source() { return _source; }
    int pin() { return _pin; }
    // bool enabled() { return _enabled; }
};

enum class TransDir : int { Rev=0, Fwd=1, NumTransDir=2 }; // possible dir values. Rev means native sensed value has the opposite polarity of the real world effect (for example, brake position lower inches of extension means higher applied brakes)
enum TransType { ActuatorType=0, SensorType=1, NumTransType=2 }; // possible dir values. Rev means native sensed value has the opposite polarity of the real world effect (for example, brake position lower inches of extension means higher applied brakes)
std::string transtypecard[NumTransType] = { "actuator", "sensor" };
std::string transdircard[(int)TransDir::NumTransDir] = { "rev", "fwd" };

// Transducer class
// Device::Transducer is a base class for any system devices that convert real-world values <--> signals in either direction. it has a "native"
// value which represents the sensed or driven hardware input/output. it also has a "si" value which represents the logical or human-readable
// equivalent value. by default, adjusting either value will automatically change the other one. this class holds key device values, converts between different units they might use, and manages their constraint within specified range limits
// each xducer has a primary value associated with it, eg for the speedo this is our speed. Here we maintain 3 separate versions of our primary value:
//   Variables: 
//     _si (Param): a possibly-filtered value in human-readable units like feet or psi, auto constrained within specified absolute limits*.
//     _native (Param) : a raw (unfiltered) value in machine-interface units, often adc counts or microseconds (us), auto constrained within specified absolute limits.
//     _si_raw (float) : is a raw (unfiltered) si-unit conversion of the raw value, useful if scrutinizing the effects of filters, etc.
//   Units:  units of measure appropriate to the device. these appear in variable and function names, or when absent the default "si" is assumed. Here are the possibilities:  
//     "si": (default) values kept in standard units of measure which humans prefer, such as mph, feet-per-second, etc.
//     "native": values kept in whatever units are native to the specific device/driver, usually adc counts (for analog sensors) or microseconds (for pwm)... depends on the sensor type
//     "pc": values scaled to a percentage of the operational range, where opmin = 0% and opmax = 100%
//   Conversions:  set _convmethod to best suit the most reliable data you have for your sensor. this decides what math to do when converting between native and si unit values. Here are the choices:
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
    float _mfactor = 1.0, _boffset = 0.0, sim_val, _zeropoint, _si_raw;  // si_raw is only meaningful for sensors, not actuators. managed here because that's easier
    float _opmin = NAN, _opmax = NAN, _opmin_native = NAN, _opmax_native = NAN, _margin = 0.0;
    int _transtype, _convmethod = LinearMath;  // the default method
    Param _si, _native;
    TransDir _dir = TransDir::Fwd;
  public:
    // operator float() { return _si.val(); }
    Transducer(int arg_pin) : Device(arg_pin) {
        _long_name = "Unknown transducer";
        _short_name = "xducer";
    }
    Transducer() = delete;  // this ensures a pin is always provided
    virtual void setup() {
        // print_config(true, false);  // print header
        // if (_pin < 255) ezread.squintf(" on pin %d\n", _pin);
    }   // ezread.squintf("%s..\n", _long_name.c_str()); }
    // virtual void tedit(float tdelta) {  // for simulator editing of the value
    //     sim_val = constrain(_si.val() + tdelta, _opmin, _opmax);
    // }
    virtual void print_config(bool header=true, bool ranges=true) {
        if (header) {
            ezread.squintf(ezread.highlightcolor, "%s (p%d):\n", _long_name.c_str(), _pin);  // , transtypecard[_transtype].c_str());
            ezread.squintf("  val = %.2lf %s = %.2lf %s = %.2lf %%\n", _si.val(), _si_units.c_str(), _native.val(), _native_units.c_str(), pc()); 
        }
        if (ranges) {
            ezread.squintf("  op: %.2lf - %.2lf %s (%.2lf - %.2lf %s)\n", _opmin, _opmax, _si_units.c_str(), _opmin_native, _opmax_native, _native_units.c_str());
            ezread.squintf("  abs: %.2lf - %.2lf %s (%.2lf - %.2lf %s)\n", _si.min(), _si.max(), _si_units.c_str(), _native.min(), _native.max(), _native_units.c_str());
        }
    }
    virtual float from_native(float arg_native) {  // these linear conversion functions change absolute native values to si and back - overwrite in child classes as needed
        float ret = NAN;
        if (_convmethod == AbsLimMap) ret = map(arg_native, _native.min(), _native.max(), _si.min(), _si.max());
        else if (_convmethod == OpLimMap) ret = map(arg_native, _opmin_native, _opmax_native, _opmin, _opmax);
        else if (_convmethod == LinearMath) ret = _boffset + _mfactor * arg_native; // ezread.squintf("%lf = %lf + %lf * %lf\n", ret, _boffset, _mfactor, arg_val_f);
        else ezread.squintf(ezread.madcolor, "err: %s from_native convert %lf (min %lf, max %lf)\n", _short_name.c_str(), arg_native, _native.min(), _native.max());
        cleanzero(&ret, float_conversion_zero);   // if (std::abs(ret) < float_conversion_zero) ret = 0.0;  // reject any stupidly small near-zero values
        return ret;
    }
    virtual float to_native(float arg_si) {  // convert an absolute si value to native units
        float ret = NAN;
        if (_convmethod == AbsLimMap) ret = map(arg_si, _si.min(), _si.max(), _native.min(), _native.max());  // TODO : this math does not work if _invert == true!
        else if (_convmethod == OpLimMap) ret = map(arg_si, _opmin, _opmax, _opmin_native, _opmax_native);  // TODO : this math does not work if _invert == true!
        else if ((_convmethod == LinearMath) && !iszero(_mfactor)) ret = (arg_si - _boffset) / _mfactor;
        else ezread.squintf(ezread.madcolor, "err: %s to_native can't convert %lf (min %lf, max %lf)\n", _short_name.c_str(), arg_si, _si.min(), _si.max());
        cleanzero(&ret, float_conversion_zero);
        return ret;
    }
    virtual float to_pc(float arg_si) {  // convert an absolute si value to percent form.  note this honors the _dir setting
        if (std::isnan(arg_si)) return NAN;
        float ret = map(arg_si, _opmin, _opmax, 100.0 * (_dir == TransDir::Rev), 100.0 * (_dir == TransDir::Fwd));
        cleanzero(&ret, float_conversion_zero);
        return ret;
    }
    virtual float from_pc(float arg_pc) {  // convert an absolute percent value to si unit form.  note this honors the _dir setting
        if (std::isnan(arg_pc)) return NAN;
        float ret = map(arg_pc, 100.0 * (_dir == TransDir::Rev), 100.0 * (_dir == TransDir::Fwd), _opmin, _opmax);
        cleanzero(&ret, float_conversion_zero);
        return ret;
    }
    // to set limits, depends on your conversion method being used :
    //   linear_math: call either one of set_abslim() or set_abslim_native(), whichever one you have the values to define the range and all si and native abs values will be set automatically.
    //                op limits will also be reconstrained to the updated abs values, however if you have a specific op range then
    //   abslimmap: call both of set_abslim() and set_abslim_native(), providing all 4 values which are later used to convert values. specify false for the autocalc argument
    //   oplimmap:  call both of set_oplim() and set_oplim_native(), providing all 4 values which are later used to convert values. specify false for the autocalc argument
    virtual void set_abslim(float argmin=NAN, float argmax=NAN, bool autocalc=true) { // these are absmin and absmax limits. values where NAN is passed in won't be used
        if (std::isnan(argmin)) argmin = _si.min();                               // use incumbent min value if none was passed in
        if (std::isnan(argmax)) argmax = _si.max();                               // use incumbent max value if none was passed in
        _si.set_limits(argmin, argmax);                                           // commit to the Param accordingly
        _si_raw = constrain(_si_raw, _si.min(), _si.max());                       // in case we have new limits, re-constrain the raw si value we also manage
        if ((_convmethod == LinearMath) && autocalc) {                      // if we know our conversion formula, and not instructed to skip autocalculation...
            _native.set_limits(to_native(_si.min()), to_native(_si.max()));       // then convert the new values and ensure si and native stay equivalent
        }
        set_oplim(NAN, NAN, false);                                               // just to enforce any re-constraints if needed to keep op limits inside abs limits
    }
    virtual void set_abslim_native(float argmin=NAN, float argmax=NAN, bool autocalc=true) { // these are absmin and absmax limits. values where NAN is passed in won't be used
        if (std::isnan(argmin)) argmin = _native.min();                              // use incumbent min value if none was passed in
        if (std::isnan(argmax)) argmax = _native.max();                              // use incumbent max value if none was passed in
        _native.set_limits(argmin, argmax);                                          // commit to the Param accordingly
        if ((_convmethod == LinearMath) && autocalc) {                         // if we know our conversion formula, and not instructed to skip autocalculation...
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
        if ((_convmethod != OpLimMap) && autocalc) {                    // if we know our conversion formula, and not instructed to skip autocalculation...
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
        if ((_convmethod != OpLimMap) && autocalc) {                             // if we know our conversion formula, and not instructed to skip autocalculation...
            set_oplim(from_native(_opmin_native), from_native(_opmax_native), false);  // then convert the new values and ensure si and native stay equivalent
        }
    }
    bool set_native(float arg_val_native, bool also_set_raw=true) {
        bool ret = true;
        if (!_native.set(arg_val_native)) ret = false;
        if (std::isnan(arg_val_native)) _si.set(NAN);
        else _si.set(from_native(_native.val()));
        if (also_set_raw) _si_raw = _si.val();
        return ret;
    }
    bool set_si(float arg_val_si, bool also_set_raw=true) {
        bool ret = true;
        if (!_si.set(arg_val_si)) ret = false;
        if (also_set_raw) _si_raw = _si.val();
        if (std::isnan(arg_val_si)) _native.set(NAN);
        else _native.set(to_native(_si.val()));
        return ret;
    }
    bool set_pc(float arg_val_pc, bool also_set_raw=true) { 
        return set_si(from_pc(arg_val_pc), also_set_raw);
    }
    bool sim_si(float arg_val_si, bool also_set_raw=true) {
        if (_source != src::Sim && _source != src::Pot) return false;
        if (!_si.set(arg_val_si)) return false;
        if (also_set_raw) _si_raw = _si.val();
        _native.set(to_native(_si.val()));
        return true;
    }
    void set_margin(float arg_marg) { _margin = arg_marg; }
    void set_conversions(float arg_mfactor=NAN, float arg_boffset=NAN) {  // arguments passed in as NAN will not be used.  // Convert units from base numerical value to disp units:  val_native = m-factor*val_numeric + offset  -or-  val_native = m-factor/val_numeric + offset  where m-factor, b-offset, invert are set here
        if (!std::isnan(arg_mfactor)) {
            if (iszero(arg_mfactor)) ezread.squintf("Err: can not support _mfactor of zero\n");
            else _mfactor = arg_mfactor;
        }
        if (!std::isnan(arg_boffset)) _boffset = arg_boffset;
    }
    float val() { return _si.val(); }  // this is the si-unit filtered value (default for general consumption)
    float native() { return _native.val(); }  // This is a native unit value, constrained to abs range but otherwise unfiltered
    float pc() { return to_pc(_si.val()); }  // get value as a percent of the operational range
    float raw() { return _si_raw; }  // this is the si-unit raw value, constrained to abs range but otherwise unfiltered
    float raw_pc() { return to_pc(_si_raw); }  // get raw value in percent
    float absmin() { return _si.min(); }
    float absmax() { return _si.max(); }
    float absmin_native() { return _native.min(); }
    float absmax_native() { return _native.max(); }
    float opmin() { return _opmin; }
    float opmax() { return _opmax; }
    float opmin_native() { return _opmin_native; }
    float opmax_native() { return _opmax_native; }
    float margin() { return _margin; }
    float margin_native() { return to_native(_margin); }
    float margin_pc() { return std::abs(to_pc(_margin)); }
    float zeropoint() { return _zeropoint; }  // zeropoint is the pressure at which we can consider the brake is released (if position is unavailable)
    float zeropoint_pc() { return to_pc(_zeropoint); }
    float mfactor() { return _mfactor; }
    float boffset() { return _boffset; }
    float* ptr() { return _si.ptr(); }
    float* native_ptr() { return _native.ptr(); }
    float* absmin_ptr() { return _si.min_ptr(); }
    float* absmax_ptr() { return _si.max_ptr(); }
    float* opmin_ptr() { return &_opmin; }
    float* opmax_ptr() { return &_opmax; }
    float* margin_ptr() { return &_margin; }
    float* zeropoint_ptr() { return &_zeropoint; }
};
// Sensor class - is a base class for control system sensors, ie anything that measures real world data or electrical signals 
// 
// Sensor class notes:  Some definitions (240522 soren)
// sensors typically apply filtering to the incoming values read. Postprocessing is done to the _si Param value accessible externally by val()
//   the values are described below. all this is inherited from Transducer, just with filtering applied:
//     "_native": this unfiltered value is in units direct from the sensor/driver, constrained to abs range. get externally using native()
//     "_raw": this is the same unfiltered value just converted to the default si units. get externally using raw()
//     "_si": this is the value after filtering is applied, in default si units, externally accessible using val()
//     "pc()": the filtered value is available scaled as a percentage of the operatioal range. it's not stored, but accessible using pc()
class Sensor : public Transducer {
  protected:
    float _ema_alpha = 0.1;
    bool _first_filter_run = false, _detected = false, _responding = false;

    void calculate_ema() { // Exponential Moving Average
        if (_first_filter_run) {
            set_si(_si_raw, false);
            _first_filter_run = false;
            return;
        }
        set_si(ema_filt(_si_raw, _si.val(), _ema_alpha), false);
    }
    virtual void set_val_from_undef() {  // for example by the onscreen simulator interface. TODO: examine this functionality, it aint right
        set_si(NAN);
    }
    virtual void set_val_from_fixed() {  // for example by the onscreen simulator interface. TODO: examine this functionality, it aint right
        set_si(NAN);
    }
    virtual void set_val_from_sim() {  // for example by the onscreen simulator interface. TODO: examine this functionality, it aint right
        // sim_si(sim_val);                 // wtf is this supposed to do?
        // set_si(_si.val + touch.fdelta);  // I would think this should look something like this (needs some coding on the other side to support)
    }
    // virtual float read_sensor() {
    //     ezread.squintf(ezread.madcolor, "err: %s does not have an overridden read_sensor() function\n", _short_name.c_str());
    //     return NAN;
    // }
    virtual void set_val_from_pin() {  // must be overridden by children
        set_si(NAN);
    }
    virtual void set_val_from_pot() {
        set_si(_pot->mapToRange(_opmin, _opmax));  // as currently written, the pot will spoof both si and native raw values in addition to the filtered si value.  do we want this?
    }
    // virtual void update_si_limits() { _val_filt.set_limits(_si.min_shptr(), _si.max_shptr()); } // make sure our filtered value has the same limits as our regular value
    void update_source() { if (_source == src::Pin) _first_filter_run = true; } // if we just switched to pin input, the old filtered value is not valid
  public:
    Sensor(int pin) : Transducer(pin) {
        _long_name = "Unknown";
        _short_name = "unksen";
    }  
    virtual void setup() {
        _transtype = SensorType;
        // if (!_detected) set_source(src::Fixed);
        Transducer::setup();
    }
    void set_ema_alpha(float arg_alpha) { _ema_alpha = std::fmaxf(0.0f, arg_alpha); }
    float ema_alpha() { return _ema_alpha; }
    bool released() { return (std::abs(val() - _zeropoint) <= _margin); }
};
// trying to begin to make temp sensors conform to this class structure so i can simulate them. may be throwaway?
// class TempSensor : public Sensor {
//   protected:
//     TemperatureSensorManager* _tempsens;
//   public:
//     void setup() {
//         set_can_source(src::Pot, true);
//     }
// };


// Base class for sensors which communicate using i2c.
// note: this is a strange type of Sensor, it's not really a Transducer but it does do filtering. maybe we should rethink the hierarchy a little?
//       I think we can move Param to common.h and add a class ExponentialMovingAverage there as well that just has the ema functionality, then make
//       I2CSensor a child of -> Device, ExponentialMovingAverage and not a Sensor at all.
class I2CSensor : public Sensor {
  protected:
    I2C* _i2c;
    int _i2c_bus_index = I2CBogus;  // children must set this to identify self in calls to I2C bus class
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
    virtual float read_i2c_sensor() {  // childrem must override this
        ezread.squintf(ezread.madcolor, "err: %s needs overridden read_i2c_sensor()\n", _short_name.c_str());
        return NAN;
    }
    void print_on_boot(bool detected, bool responding) {
        // if (header) Transducer::print_config(true, false);
        ezread.squintf(ezread.highlightcolor, "%s sensor (i2c 0x%02x) %sdetected\n", _long_name.c_str(), addr, _detected ? "" : "not ");
        if (detected) {
            if (responding) ezread.squintf("  reading %.4f %s\n", read_i2c_sensor(), _si_units.c_str());
            else ezread.squintf(ezread.sadcolor, "  no response\n");  // begin communication with air flow sensor) over I2C 
                // set_source(src::Fixed); // sensor is detected but not working, leave it in an error state ('fixed' as in not changing)
        }
        else {
            ezread.squintf("\n");
            // set_source(src::Fixed); // don't even have a device at all..
        }
    }
    void set_val_from_pin() {
        if (_i2c->not_my_turn(_i2c_bus_index)) return;
        if (!_i2c->detected(_i2c_bus_index) || !_responding) _native.set(NAN);  // if bus/sensor failure set value to nan
        else _native.set(read_i2c_sensor());                           // otherwise take a new native reading from the bus
        if (_i2c->i2cbaton == _i2c_bus_index) _i2c->pass_i2c_baton();  // deal with bus semaphore, since we're done with it
        if (std::isnan(_native.val())) set_si(NAN);                    // propagate nan native value to all values
        else {                                                         // if reading was good
            _si_raw = from_native(_native.val());                      // convert native to raw si value
            calculate_ema();                                           // set si filtered value based on new raw reading
        }
    }
  public:
    uint8_t addr;
    I2CSensor(I2C* i2c_arg, uint8_t i2c_address_arg) : Sensor(-1), _i2c(i2c_arg), addr(i2c_address_arg) {
        _long_name = "Unknown i2c sensor";
        _short_name = "unki2c";
        set_can_source(src::Pin, true);
    }
    I2CSensor() = delete;
    virtual void setup() {  // call at the end of child setup() override functions
        set_can_source(src::Pot, true);
        set_conversions(1.0, 0.0); // our i2c sensors so far provide si units directly.
        _detected = _i2c->detected_by_addr(addr);
        // if (!_detected || !_responding) _enabled = false;
        print_on_boot(_detected, _responding);
        Sensor::setup();
    }
};
class AirVeloSensor : public I2CSensor {  // AirVeloSensor measures the air intake into the engine in MPH. It communicates with the external sensor using i2c.
  protected:
    FS3000 _sensor;
    int64_t val_refresh_period = 85000;
    int _i2c_bus_index = I2CAirVelo;  // to identify self in calls to I2C bus class
    float read_i2c_sensor() {
        return _sensor.readMilesPerHour();  // note, this returns a float from 0-33.55 for the FS3000-1015 
    }
  public:
    static constexpr uint8_t addr = 0x28;
    sens _senstype = sens::airvelo;
    AirVeloSensor(I2C* i2c_arg) : I2CSensor(i2c_arg, addr) {
        _long_name = "Air velocity";
        _short_name = "airvel";
        _native_units = "mph";
        _si_units = "mph";
    }
    AirVeloSensor() = delete;
    void setup() {  // ezread.squintf("%s..", _long_name.c_str());
        set_si(0.0);  // initialize value
        set_abslim(0.0, 33.55);  // set abs range. defined in this case by the sensor spec max reading
        set_oplim(0.0, 27.36);  // 620/2 cm3/rot * 4800 rot/min * 60 min/hr * 1/160934 mi/cm * 1/pi * 1/((2 in * 2.54 cm/in) / 2)^2) 1/cm2  = 27.36 mi/hr (mph
        // set_oplim(0.0, 28.5);  // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * ((2 * 2.54) / 2)^2) 1/cm2 * 1/160934 mi/cm = 28.5 mi/hr (mph)            // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * (2.85 / 2)^2) 1/cm2 * 1/160934 mi/cm = 90.58 mi/hr (mph) (?!)  
        set_ema_alpha(0.2);  // note: all the conversion constants for this sensor are actually correct being the defaults 
        _responding = !_sensor.begin();
        if (_responding) _sensor.setRange(AIRFLOW_RANGE_15_MPS);
        I2CSensor::setup();
    }
};
class MAPSensor : public I2CSensor {  // MAPSensor measures the air pressure of the engine manifold in PSI. It communicates with the external sensor using i2c.
  protected:
    SparkFun_MicroPressure _sensor;
    int64_t val_refresh_period = 120000; // , mapretry_timeout = 10000;
    int _i2c_bus_index = I2CMAP;  // to identify self in calls to I2C bus class
    float read_i2c_sensor() {
        return _sensor.readPressure(ATM, false);  // _sensor.readPressure(PSI);  // <- blocking version takes 6.5ms to read
    }
  public:
    static constexpr uint8_t addr = 0x18;  // note: would all MAPSensors have the same address?  ANS: yes by default, or an alternate fixed addr can be hardware-selected by hooking a pin low or something
    sens _senstype = sens::mapsens;
    MAPSensor(I2C* i2c_arg) : I2CSensor(i2c_arg, addr) {
        _long_name = "MAP";
        _short_name = "map";
        _native_units = "atm";
        _si_units = "atm";
    }
    MAPSensor() = delete;
    void setup() {
        set_si(1.0);  // initialize value
        set_abslim(0.06, 2.46);  // set abs range. defined in this case by the sensor spec max reading
        set_oplim(0.68, 1.02);  // set in atm empirically
        set_ema_alpha(0.2);
        _responding = !_sensor.begin();
        I2CSensor::setup();
    }
};
class AnalogSensor : public Sensor {  // class AnalogSensor are sensors where the value is based on an ADC reading (eg brake pressure, brake actuator position, pot)
  protected:
    int64_t val_refresh_period = 25000; // , mapretry_timeout = 10000;
    void set_val_from_pin() {
        if (!_native.set(static_cast<float>(analogRead(_pin)))) return;
        _si_raw = from_native(_native.val());
        calculate_ema(); // filtered values are kept in si format
    }
  public:
    AnalogSensor(int arg_pin) : Sensor(arg_pin) {
        _long_name = "Unknown analog";
        _short_name = "analog";
        _native_units = "adc";
    }
    virtual void setup() {  // child classes call this before setting limits
        Sensor::setup();
        set_pin(_pin, INPUT);
        set_can_source(src::Pin, true);
        set_can_source(src::Pot, true);
        set_source(src::Pin);
        set_abslim_native(0.0, (float)adcrange_adc, false);  // do not autocalc the si units because our math is not set up yet (is in child classes)
    }
};
class CarBattery : public AnalogSensor {  // CarBattery reads the voltage level from the Mule battery
  public:
    sens _senstype = sens::mulebatt;
    CarBattery(int arg_pin) : AnalogSensor(arg_pin) {
        _long_name = "Battery";
        _short_name = "mulbat";
        _native_units = "adc";
        _si_units = "V";
    }
    CarBattery() = delete;    
    void setup() {  // ezread.squintf("%s..\n", _long_name.c_str());
        AnalogSensor::setup();
        set_conversions(0.004075, 0.0);  // 240627 calibrated against my best multimeter: m = 0.004075
        set_abslim(0.0, 15.1);  // set abs range. dictated in this case by the max voltage a battery charger might output
        set_oplim(10.7, 14.8);  // set op range. dictated by the expected range of voltage of a loaded lead-acid battery across its discharge curve
        set_si(11.5);  // initialize value, just set to generic rest voltage of a lead-acid battery
        set_ema_alpha(0.005);
        set_can_source(src::Pot, true);
        print_config();
    }
    void set_val_from_sim() { set_si(12.0); }  // what exactly is going on here? maybe an attempt to prevent always showing battery errors on dev boards?
    // void update() {
        // Serial.printf("\n\n%.4lf", val());
        // Device::update();
        // Serial.printf(" -> %.4lf\n\n", val());
    // }
};
// PressureSensor represents a brake fluid pressure sensor.
// it extends AnalogSensor to handle reading an analog pin
// and converting the ADC value to a pressure value in PSI.
class PressureSensor : public AnalogSensor {
  public:
    sens _senstype = sens::pressure;
    // int opmin_adc, opmax_adc, absmin_adc, absmax_adc; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
    // Soren 230920: reducing max to value even wimpier than Chris' pathetic 2080 adc (~284 psi) brake press, to prevent overtaxing the motor
    PressureSensor(int arg_pin) : AnalogSensor(arg_pin) {
        _long_name = "BrkPressure";
        _short_name = "presur";
        _native_units = "adc";
        _si_units = "psi";
    }
    // Older notes:
    // the sensor output voltage spec range is 0.5-4.5 V to indicate 0-1000 psi
    // our ADC top is just 3.3V tho, so we can sense up to 695 psi (calculated) at our max adc
    // calculated using spec values:
    //   sensor gives:  1000 psi / (4.5 V - 0.5 V) = 250 psi/V   (or 0.004 V/psi)
    //   our adc:  4096 adc / 3.3 V = 1241 adc/V   (or 0.000806 V/adc)
    //   resulting slope:  250 psi/V * 0.000806 V/adc = 0.2 psi/adc  (or 5 adc/psi)
    //   zero psi point:  0.5 V * 1241 adc/V = 621 adc
    //   fullscale reading:  0.2 psi/adc * (4095 adc - 621 adc) = 695 psi
    // math to convert (based on spec values):
    //   psi = 0.2 * (adc - 621)  ->  psi = 0.2 * adc - 124  ->  adc = 5 * psi + 621

    PressureSensor() = delete;
    void setup() {
        AnalogSensor::setup();
        
        // (old) pre-bm24 calibrating notes:
        // * hook to sensor and release brake, then pump and press as hard as you can. Get adc vals and set here as oplims_native. 
        // * released and point of pedal begins moving (chain slack already minimized):  posn = 4.13in, pres = 0 psi (615 adc)
        // * point just before pressure begins to rise from 0 (zeropoint):  posn = 3.63 in, pres = 0 psi (618 adc)
        // * max i can push w/ foot:  posn = 2.32 in (1352 adc), pres = 313 psi (2172 adc)

        // 2025 Brake calibration procedure (pressure-related steps):   circa 250720
        // 1-8) first setup up some position sensor values - see the cal procedure in the BrakePositionSensor class for these steps  
        // 9) (on car w/ linkage, in cal mode) set the ema alpha value:   (this process is somewhat iterative, needs work)
        //    * watch datapage value for pressure psi change after going directly from high to no brake. this settling time should be just fast enough for the pid to work
        //    * lower alpha will stabilize sensor readings, but slow down the settling time. the rest of the cal procedure below will need to be redone if alpha is changed
        //    * settled on alpha of (0.03) on 250720
        //    * put this value in the set_ema_alpha() call below
        //    !! * TODO!  try slowing down the brake pid to mitigate calculus inaccuracies from low alpha values. in theory the pid math loop should run just slower than the sensor value takes to be accurate. needs on-car experimentation to optimize
        // 10) (on car w/ linkage, in cal mode) determine the adc opmin:
        //     * fully release brake (slack in linkage), reading value from the datapage
        //     * as value jumps around, go for the lowest value you see. (682) 250720
        //     * put this value in the min_adc assignment below
        // 11) (on car w/ linkage, in cal mode) set the adc opmax, by watching datapage value while pushing brake to max
        //     * guidelines: use one hand to grab dash for bracing, pump brake twice then push hard as soren can (no other aids)
        //     * as value jumps around, go for the lowest value.  (2200) 250720
        //     * put this value in the set_oplim_native() call below (2nd argument)
        // 12-13) finalize the position sensor values - see the cal procedure in the BrakePositionSensor class for these steps  

        float min_adc = 650.0;  // from step #2 above. max value set in function call below  // 650 to 713  (earlier i was seeing 662 min. hmm)
        float noise_margin = 45.0;  // how much the min value can jump to above the number above due to noise
        float m = 1000.0 * (3.3 - 0.554) / (((float)adcrange_adc - min_adc) * (4.5 - 0.554)); // 1000 psi * (adc_max v - v_min v) / ((4095 adc - 684 adc) * (v-max v - v-min v)) = 0.1358 psi/adc
        float b = -1.0 * min_adc * m;  // -684 adc * 0.1358 psi/adc = -92.88 psi
        set_conversions(m, b);
        set_abslim_native(0.0, (float)adcrange_adc);  // set native abslims after m and b are set.  si abslims will autocalc
        set_oplim_native(min_adc, 2176.0);            // from step #3 above  // set this after abslims. si oplims will autocalc. bm24: max before pedal interference = 1385 (previous:) 2350 adc is the adc when I push as hard as i can (soren 240609)
        set_oplim(0.0, NAN);  // now set just the si oplim to exactly zero. This will cause native oplims to autocalc such that any small errors don't cause our si oplim to be nonzero
        // set_oplim(4.6, 350.0);  // 240605 these are the extremes seen with these settings. Is this line necessary though? (soren)
        // ezread.squintf(" | oplim_native = %lf, %lf | ", _opmin_native, _opmax_native);
        set_ema_alpha(0.03);   // from step #1 above  // 2024 was 0.055
        set_margin(noise_margin);       // max acceptible error when checking psi levels
        _zeropoint = from_native(min_adc + noise_margin / 2.0f);   // tuning 250720 set to 686, avg value on screen (was chging +/- 5 adc), when at zeropoint value set in position sensor (4.07in)  ////    pushing the pedal just enough to take up the useless play, braking only barely starting. I saw adc = 680. convert this to si
        set_native(_opmin_native);
        print_config();
    }
    bool parked() { return (std::abs(val() - _opmin) <= _margin); }  // is tha brake motor parked?
    float parkpos() { return _opmin; }
    float zeropoint_pc() { return to_pc(_zeropoint); }
    float parkpos_pc() { return to_pc(_opmax); }

};
// BrakePositionSensor represents a linear position sensor for measuring brake pedal position
// extends AnalogSensor for handling analog pin reading and conversion.
//
// WIP! 2025 Brake calibration procedure (position-related steps):   circa 250720
//
// 1) (w/o linkage, in cal mode), measure inches from housing end to piston end at both motor extremes (1.22, 7.26) on 250719 w/ gmw1 motor. this can also be done on the bench
// 2) (w/o linkage, in cal mode), get corresponding bkakposn adc values from datapage, at motor extremes (1885, 2933) on 250719 w/ gmw1 motor. this can also be done on the bench
// 3a) (on car w/o linkage, in cal mode) we will find position op limits:
//     * w/pedal released get opmax inches.
//     * watch pressure as you push brake as hard as you can w/ foot, note this pressure
//     * In cal mode use actuator to push brake to the same pressure. The corresponding brake positionis its op minimum
// 3b) (math) add a buffer, like 0.75" to the approximate range b/c motor can pull harder, and for chain slack (3.0) 250719 w/ gmw1 motor
// 3c) (math) center this range within the abslim range above, to get approximate opmin & opmax (2.73, 5.73) on 250719 w/ gmw1 motor
// 4) (math) determine fake zeropoint value to midpoint of oprange above (4.23) on 250719 w/ gmw1 motor
// 5) compile in these results as follows (instead of values being used currently) and upload:
//    set_abslim(1.22, 7.26, false);        // from step #1
//    set_abslim_native(1885, 2933, false); // from step #2
//    set_oplim(2.73, 5.95);                // from step #3c
//    _zeropoint = 4.23;                    // from step #4
// 6) (on car w/o linkage, in cal mode), set the motor so datapage value matches the approximate opmax value from step #3c
// 7) if necessary, adjust the shim block under the actuator housing so the unit has minimal clearance to it when pedal is fully depressed and linkage is tight
// 8) attach brake pedal linkage, make as short as possible w/ a minimum of slack while still allowing pedal to fully release
// 9) adjust brake pressure ema alpha value, if necessary - see cal procedure in the PressureSensor class for this step
// 10) calibrate brake pressure opmin: see cal procedure in the PressureSensor class for this step
// 11) calibrate brake pressure opmax: see cal procedure in the PressureSensor class for this step
// 12a) determine the pressure/position zeropoint, ie where is the point where the pressure first begins to increase from min value. (based on pressure readings while calbraking carefully)
// 12b) read zeropoint pressure from datapage (goes in pressure class)
// 12c) read zeropoint position from datapage in inches (4.07) on 250720 w/ gmw1 motor
// !! 13) *NOT* complete!  (on car w/ linkage, in ?? mode) fine-tune the operational limits to final values (finish detailing this step)  
class BrakePositionSensor : public AnalogSensor {
  public:
    sens _senstype = sens::brkpos;
    BrakePositionSensor(int arg_pin) : AnalogSensor(arg_pin) {
        _long_name = "BrkPosition";
        _short_name = "brkpos";
        _native_units = "adc";
        _si_units = "in";
    }
    BrakePositionSensor() = delete;
    void setup() {
        AnalogSensor::setup();
        _dir = TransDir::Rev;  // causes percent conversions to use inverted scale 
        _convmethod = AbsLimMap;  // because using map conversions, need to set abslim for si and native separately, but don't need mfactor/boffset
        set_abslim(1.22, 7.26, false);        // from step #1   // need false argument to prevent autocalculation
        set_abslim_native(1885, 2933, false); // from step #2   // need false argument to prevent autocalculation
        set_oplim(1.22, 5.9f);                // from step #13  !! 250720 not yet finalized! (do step 13)
        _zeropoint = 5.7;                    // from step #12c
        // don't also set native oplims as they will autocalc from oplims setting
        // set_oplim_native(1445, 1923);  // 240609 1445 (2.68in) is full push, and 1923 (4.5in) is park position (with simple quicklink +carabeener linkage)
        set_ema_alpha(0.35);
        set_margin(0.2);  // TODO: add description
        // _mfactor = (_absmax - _absmin) / (float)(_absmax_adc - _absmin_adc);  // (8.85 in - 0.95 in) / (3103 adc - 979 adc) = 0.00372 in/adc
        // _boffset = -2.69;  //  979 adc * 0.00372 in/adc - 0.95 in = -2.69 in
        print_config();
    }
    bool parked() { return (std::abs(val() - _opmax) <= _margin); }  // is tha brake motor parked?
    float parkpos() { return _opmax; }
    float parkpos_pc() { return to_pc(_opmax); }
    float zeropoint_pc() { return to_pc(_zeropoint); }
};
// class PulseSensor are hall-effect based magnetic field sensors where the value is based on magnetic pulse timing of a rotational Source (eg tachometer, speedometer)
class PulseSensor : public Sensor {
  protected:
    // volatile int64_t timestamp_last_us;  // _stop_timeout_us = 1250000;  // time after last magnet pulse when we can assume the engine is stopped (in us)
    Timer _pulsecount_timer{1000000};
    bool _low_pulse = true, _pin_level, _pin_inactive = false;
    float _freqfactor = 1.0;  // a fixed freq compensation factor for certain externals like multiple magnets (val <1) or divider circuitry (val >1).  also, the best gay club in miami
    // float _calfactor = 1.0;   // a tunable/calibratable factor same as the above, where if <1 gives fewer si-units/pulse-hz and vice versa.         also, most popular weight-loss plan in miami
    Timer pinactivitytimer{1500000};  // timeout we assume pin isn't active if no pulses occur
    volatile int64_t _isr_us = 0;
    volatile int64_t _isr_time_last_us = 0;
    volatile int64_t _isr_time_current_us = 0;
    volatile int _isr_pulse_period_us = 0;
    volatile int _pulse_count = 0;  // record pulses occurring for debug/monitoring pulse activity
    int _pulses_per_sec = 0;
    float _ema_tau_min_us = 1000.0f;      // default tau min in us. must be >0
    float _ema_tau_max_us = 10000000.0f;  // default tau max in us
    float _ema_tau_us = 2500000.0f;       // default tau value in us. must be >0

    // we maintain our min and max pulse period, for each pulse sensor
    // absmin_us is calculated based on absmax_native (Hz) using overloaded set_abslim_native() function
    // absmax_us is our stop timeout, and not based on absmin_native (Hz). It must be set using set_abmax_us() function
    float _us, _absmin_us, _absmax_us = 1500000; //  at min = 6500 us:   1000000 us/sec / 6500 us = 154 Hz max pulse frequency.  max is chosen just arbitrarily
    int64_t _absmin_us_64;  // so the isr doesn't have to cast type. 

    // shadows a hall sensor being triggered by a passing magnet once per pulley turn. The ISR calls
    // esp_timer_get_time() on every pulse to know the time since the previous pulse. I tested this on the bench up
    // to about 0.750 mph which is as fast as I can move the magnet with my hand, and it works.
    // update: Janky bench test appeared to work up to 11000 rpm.
    void IRAM_ATTR _isr() { // The isr gets the period of the vehicle pulley rotations.
        _isr_time_current_us = esp_timer_get_time();
        int64_t time_us = _isr_time_current_us - _isr_time_last_us;
        if (time_us < _absmin_us_64) return;  // ignore spurious triggers or bounces
        _isr_time_last_us = _isr_time_current_us;
        _isr_us = time_us;
        _pulse_count++;
    }

    // modified ema filter where filtering is smoother the less often it's called
    float scaling_ema_filt(float raw, float filt, float dt_s, float tau_s) {
        dt_s = std::fmaxf(0.0f, dt_s);      // pretend negative times are 0
        return filt * std::exp(-dt_s / tau_s) + raw * (1 - std::exp(-dt_s / tau_s));
    }

    // override standard ema filter so filtering is smoother the less frequently we get pulses
    void calculate_ema() {
        if (_first_filter_run) {  // initialization on first run works the same as in Sensor::calculate_ema()
            set_si(_si_raw, false);
            _first_filter_run = false;
            return;
        }
        set_si(scaling_ema_filt(_si_raw, _si.val(), _us, _ema_tau_us), false);
        // set_si(ema_filt(_si_raw, _si.val(), _ema_alpha), false);
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
    void set_val_from_pin() {
        float _isr_buf_us = static_cast<float>(_isr_us);     // copy delta value (in case another interrupt happens during handling)

        if ((esp_timer_get_time() - _isr_time_last_us) >= _absmax_us) _native.set(0.0f); // if last pulse was >absmax ago, call it 0 Hz
        else if (_isr_buf_us < _absmin_us) _native.set(NAN); // if pulse is too soon after the last one (possible bounce), set native to nan
        else _native.set(us_to_hz(_isr_buf_us));             // else convert valid us reading to native hz value
        
        _us = iszero(_native.val()) ? NAN : _isr_buf_us;     // determine us pulsewidth value
        
        if (!std::isnan(_native.val())) {         // if native Hz value is valid
            _si_raw = from_native(_native.val()); // convert for valid si value
            calculate_ema();                      // apply ema filter for valid si filt value
        }
        _pin_level = read_pin(_pin);              // _pin_level = !_pin_level;
        set_pin_inactive();                       // for idiot light showing pulse activity
        if (_pulsecount_timer.expireset()) {
            _pulses_per_sec = _pulse_count;       // pulses_per_sec can be used for monitoring/debugging;
            _pulse_count = 0;
        }
    }
    float us_to_hz(float arg) {
        if (iszero(arg) || std::isnan(arg)) return 0.0;  // zero is a special value meaning we timed out
        return 1000000.0 / arg;
        // ezread.squintf("Err: %s us_to_hz() reciprocal of zero\n", _short_name.c_str());
        // return absmax();
    }
    float hz_to_us(float arg) { return us_to_hz(arg); }  // the same math converts in either direction
  public:
    std::string _true_native_units = "us";  // these pulse sensors actually deal in us, more native than Hz but Hz is compatible w/ our common conversion algos

    PulseSensor(int arg_pin, float arg_freqfactor=1.0) : Sensor(arg_pin), _freqfactor(arg_freqfactor) {
        _long_name = "Unknown hall-effect";
        _short_name = "pulsen";
        _native_units = "Hz";
    }
    PulseSensor() = delete;
    void print_config(bool header=true, bool ranges=true) {
        Transducer::print_config(header, ranges);
        if (ranges) ezread.squintf("  pulse = %.0lf %s, abs: %.0lf-%.0lf %s\n", _us, _true_native_units.c_str(), _absmin_us, _absmax_us, _true_native_units.c_str());
    }
    // from our limits we will derive our min and max pulse period in us to use for bounce rejection and zero threshold respectively
    // Overload the normal function so we can also include absmin_us calculation. Note absmax_us is set with a separate function 
    void set_abslim_native(float arg_min, float arg_max, bool calc_si=true) {  // use calc_si = false to avoid recalculating si value
        if ((!std::isnan(arg_min) && iszero(arg_min)) || (!std::isnan(arg_max) && iszero(arg_max))) {
            ezread.squintf(ezread.madcolor, "err: pulse sensor %s can't have limit of 0\n", _short_name.c_str());
            return;  // we can't accept 0 Hz for either absmin or absmax
        }
        Transducer::set_abslim_native(arg_min, arg_max, calc_si);
        if (!std::isnan(arg_max)) {  // now we set absmin_us to match absmax_native (hz). absmax_us is set in its own function
            _absmin_us = hz_to_us(_native.max());  // also set us limits from here, converting Hz to us. note min/max are swapped
            _absmin_us_64 = (int64_t)_absmin_us;       // make an int copy for the isr to use conveniently
        }
    }
    // this function allows direct setting of the stop timeout value (pulse-to-pulse time at which sensor will be considered stopped)
    void set_absmax_us(float arg_max) { _absmax_us = arg_max; }
    virtual void setup() {
        Sensor::setup();
        set_pin(_pin, INPUT_PULLUP);
        set_can_source(src::Pin, true);
        set_source(src::Pin);
        attachInterrupt(digitalPinToInterrupt(_pin), [this]{ _isr(); }, _low_pulse ? FALLING : RISING);
        set_can_source(src::Pot, true);
    }
    // float last_read_time() { return _last_read_time_us; }
    bool stopped() { return iszero(val()); }
    // old: bool stopped() { return (std::abs(val() - _opmin) <= _margin); }  // Note due to weird float math stuff, can not just check if tach == 0.0
    // older:  bool stopped() { return (esp_timer_get_time() - _last_read_time_us > _opmax_native); }  // Note due to weird float math stuff, can not just check if tach == 0.0
    void set_ema_tau(float newtau) { _ema_tau_us = constrain(newtau, _ema_tau_min_us, _ema_tau_max_us); }
    float ema_tau() { return _ema_tau_us; }
    float ema_tau_min() { return _ema_tau_min_us; }
    float ema_tau_max() { return _ema_tau_max_us; }
    bool* pin_inactive_ptr() { return &_pin_inactive; }
    bool* pin_level_ptr() { return &_pin_level; }
    float absmin_us() { return _absmin_us; }
    float absmax_us() { return _absmax_us; }
    float absmin_ms() { return _absmin_us / 1000.0f; }
    float absmax_ms() { return _absmax_us / 1000.0f; }
    float us() { return _us; }
    float ms() { return _us / 1000.0f; }
    int alt_native() { return _pulses_per_sec; };  // returns pulse freq in hz based on count, rather than conversion (just a check to debug) (only updates once per second)
};
// Tachometer represents a magnetic pulse measurement of the engine rotation
// it extends PulseSensor to handle reading a hall monitor sensor and converting RPU to RPM
class Tachometer : public PulseSensor {
  public:
    sens _senstype = sens::tach;
    float _calfactor = 1.0;  // a tunable/calibratable factor like _freqfactor, where if <1 gives fewer si-units/pulse-hz and vice versa.            also, the worst weight-loss scam in miami
    float _governmax_rpm;
    float _idle = 590.0, _idle_cold = 680.0, _idle_hot = 500.0;  // , _mfactor = 1.0;  // m is si/native conversion factor, informs transducer
    // float _calfactor = 0.25;  // a tunable/calibratable factor like _freqfactor, where if <1 gives fewer si-units/pulse-hz and vice versa.            also, the worst weight-loss scam in miami
    float _ema_tau_min_us = 500.0f;     // default tau min in us. must be >0
    float _ema_tau_max_us = 200000.0f;  // default tau max in us
    float _ema_tau_us = 16000.0f;       // default tau value in us. must be >0
    
    Tachometer(int arg_pin, float arg_freqfactor) : PulseSensor(arg_pin, arg_freqfactor) {  // where actual_hz = pulses_hz * freqfactor (due to external circuitry or magnet arrangement)
        _long_name = "Tachometer";
        _short_name = "tach";
        _si_units = "rpm";
    }
    Tachometer() = delete;
    float calc_mfactor() { return 60.0 * _calfactor / _freqfactor; }  // 1 Hz = 1 pulse/sec * 60 sec/min * 1.0 / 0.125 pulse/rot = 480 rot/min, (so 480 rpm/Hz)
    // float calc_mfactor() { return 60.0 * _calfactor / _freqfactor; }  // 1 Hz = 1 pulse/sec * 60 sec/min * 0.25 / 0.125 pulse/rot = 120 rot/min, (so 120 rpm/Hz)
    void setup() {  // ezread.squintf("%s..\n", _long_name.c_str());
        PulseSensor::setup();
        // note the max pulse freq at the mcu pin is max-rpm/m , or:  pinmax hz = 4500 rpm * (1/480) hz/rpm = 9.375 hz  (max freq at pin)
        // bc of the divider max freq in the wire is max-pin-hz/freqfactor , or:  wiremax hz = 9.375 hz / 0.125 = 75 hz  (max freq in wire)
        // perhaps want an rc lowpass at divider input  w/ R=22kohm, C=220nF (round up)

        // now we find our final rpm reading seems to be about 4x what it should. until we figure out the root cause, i'm adding a cal factor to compensate it
        float mfact = calc_mfactor();
        // mfact = 60.0 / _freqfactor;  // 1 Hz = 1 pulse/sec * 60 sec/min / 0.125 pulse/rot = 480 rot/min, (so 480 rpm/Hz)
        set_conversions(mfact, 0.0f);
        set_abslim(0.0f, 4800.0f);  // estimating the highest rpm we could conceivably ever see from the engine. but may lower b/c the max readable engine speed also defines the pulse debounce rejection threshold. the lower this speed, the more impervious to bouncing we are
        set_absmax_us(430000.0f);  // this sets the max pulse-to-pulse period to be considered as stopped.
        set_oplim(0.0f, 3600.0f);  // aka redline,  Max acceptable engine rotation speed (tunable) corresponds to 1 / (3600 rpm * 1/60 min/sec) = 60 Hz
        _governmax_rpm = _opmax * governor / 100.0;
        set_ema_alpha(0.015f);  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
        set_margin(15.0f);
        set_si(0.0f);
        _us = hz_to_us(_native.val());
        print_config();
    }
    void set_val_from_pin() {
        PulseSensor::set_val_from_pin();
        _governmax_rpm = _opmax * governor / 100.0;
    }
    float idle() { return _idle; }
    float* idle_ptr() { return &_idle; }
    float idle_cold() { return _idle_cold; }
    float idle_hot() { return _idle_hot; }
    float governmax() { return _governmax_rpm; }
    float* governmax_ptr() { return &_governmax_rpm; }
    void set_idle(float newidle) { _idle = constrain(newidle, _opmin, _opmax); }
    void set_idlecold(float newidlecold) { _idle_cold = constrain(newidlecold, _idle_hot + 1.0, _opmax); }
    void set_idlehot(float newidlehot) { _idle_hot = constrain(newidlehot, _opmin, _idle_cold - 1.0); }
    // bool stopped() {
    //     if (bootbutton_val) 
    //       ezread.squintf("tac.st: v=%4.1f, om=%4.1f, m=%4.1f, e%d\n", val(), _opmin, _margin, (int)(std::abs(val() - _opmin) <= _margin));  // spam catcher works on this, (w/o bootbutton condition)
    //     return PulseSensor::stopped();
    // }
};

// Speedometer represents a magnetic pulse measurement of the enginge rotation
// it extends PulseSensor to handle reading a hall monitor sensor and converting RPU to MPH

// ~ 200ms pulses according to datapage when wheel was turning steady 3.3s per rotation, should have been 1.65s pulses! So, suspect super noisy.
// also that 220ms was not steady in the slightest, jumped continuously from ~100ms to ~380ms.
// at higher rpm wheel went faster but was reporting max 20mph when wheel maybe at jogging speed (could be similar 4x calfactor like tach?)
class Speedometer : public PulseSensor {
  public:
    sens _senstype = sens::speedo;
    Speedometer(int arg_pin, float arg_freqfactor) : PulseSensor(arg_pin, arg_freqfactor) {
        _long_name = "Speedometer";
        _short_name = "speedo";
        _si_units = "mph";
    }
    Speedometer() = delete;
    void setup() {
        PulseSensor::setup();
        float mfact = 3600.0 * 20 * M_PI / (_freqfactor * 12 * 5280); // 1 Hz = 1 pulse/sec * 3600 sec/hr * 20*pi in/whlrot * 1/2 whlrot/pulse * 1/12 ft/in * 1/5280 mi/ft = 1.785 mi/hr,  (so 1.785 mph/Hz)
        // note the max pulse freq is max-mph/m , or:  pinmax hz = 20 mph * (1 / 1.785) hz/mph = 11.2 hz  (max freq at pin and in wire)
        // perhaps want an rc lowpass at divider input  w/ R=22kohm, C=1uF

        set_conversions(mfact, 0.0f);
        set_abslim(0.0f, 18.0f);  // the max readable vehicle speed also defines the pulse debounce rejection threshold. the lower this speed, the more impervious to bouncing we are
        set_absmax_us(1300000.0f);  // this sets the max pulse-to-pulse period to be considered as stopped.
        set_oplim(0.0f, 15.0f);  // aka redline,  Max possible engine rotation speed (tunable) corresponds to 1 / (3600 rpm * 1/60 min/sec) = 60 Hz
        set_ema_alpha(0.003f);  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
        set_margin(0.2f);
        // set_si(50.0);
        _us = hz_to_us(_native.val());
        // _idle = 3.0;  // estimate of speed when idling forward on flat ground (in mph)
        print_config();
    }
};

// below are beginnings of making hotrc and motors also conform to this class structure. unfinished and unused
//
// RCChannel is a class for the channels of the hotrc
// it contains the association with the RMT channels and probably filtering
// I imagine it might make sense to have two child classes, one for the two toggle channels, and one for the two analog channels
// Anyway eventually we'd like to move channel input related functionality from hotrc class into these classes, 
// then hotrc contains four instances, one per channel, which it can manage
class RCChannel : public Sensor {  // class for each channel of the hotrc
  protected:
  public:
    RCChannel(int arg_pin) : Sensor(arg_pin) {
        _long_name = "RC channel";
        _short_name = "rcchan";
        _native_units = "us";
        _si_units = "%";
    }
    RCChannel() = delete;
    virtual void setup() {
        Sensor::setup();
        set_pin(_pin, INPUT);
        set_can_source(src::Pin, true);
        set_source(src::Pin);
    }
};
class RCToggle : public RCChannel {};
class RCAnalog : public RCChannel {};

// note: I implemented the gas servo, but it looks like it's all in native units. should it still be a transducer?
// ServoMotor is a base class for our type of actuators, where by varying a pulse width (in us), motors move.
//   The gas motor is an actual rc servo, the brake and steering are 12v dc motors with voltage controlled based on servo-like pwm signals (feature of the jaguars)
class ServoMotor2 : public Transducer {
  protected:
    Servo motor;
    Timer updatetimer{85000}, outchangetimer;
    float lastoutput, max_out_change_rate_pcps = 800.0;
    int _pin, _freq;
    virtual float write_sensor() {  // note: should be marked 'override' but compiler says it doesn't override anything...?
        ezread.squintf(ezread.madcolor, "Err: %s does not have an overridden write_sensor() function\n", _short_name.c_str());
        return NAN;
    }
    void changerate_limiter() {
        float max_out_change_pc = max_out_change_rate_pcps * outchangetimer.elapsed() / 1000000.0;
        outchangetimer.reset();
        set_si(constrain(_si.val(), lastoutput - max_out_change_pc, lastoutput + max_out_change_pc));
        // lastoutput = pc[Out];  // note: you must set lastoutput = pc[Out]
    }
  public:
    // ServoMotor(int pin, int freq) : Transducer<float, float>(pin) {
    ServoMotor2(int pin, int freq) : Transducer(pin), _freq(freq) {
        set_pin(_pin, OUTPUT);   // needed?
        _transtype = ActuatorType;
        _long_name = "Unknown PWM motor";
        _short_name = "pwmout";
        _native_units = "us";
    }
    ServoMotor2() = delete;
    void setup() {  // from child classes first set up limits, then run this
        motor.setPeriodHertz(_freq);
        motor.attach(_pin, _native.min(), _native.max());  // servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    }
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
        _convmethod = OpLimMap;
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
    
    // float idle_si[NumMotorVals] = { 45.0, NAN, 60.0, 58.0, NAN, 43.0, 75.0, 1.0 };          // in angular degrees [OpMin(hot)/-/OpMax(cold)/Out/-/AbsMin/AbsMax/Margin]
    // float idletemp_f[NumMotorVals] = { 60.0, NAN, 205.0, 75.0, NAN, 40.0, 225.0, 1.5};      // in degrees F [OpMin/-/OpMax/Out/-/AbsMin/AbsMax/Margin]
    float idle_pc = 11.3;                              // idle percent is derived from the si (degrees) value
    float starting_pc = 25.0;                          // percent throttle to open to while starting the car
  public:
    ThrottleServo2(int pin, int freq) : ServoMotor2(pin, freq) {
        _dir = TransDir::Fwd;  // if your servo goes CCW with increasing pulsewidths, change to Rev
        _long_name = "Throttle servo";
        _short_name = "throtl";
        _si_units = "deg";
    }
    ThrottleServo2() = delete;
    void setup() {
        _convmethod = AbsLimMap;
        set_abslim(0.0, 180.0, false);
        set_abslim_native(500.0, 2500.0, false);
        float m = (_si.max() - _si.min()) / (_native.max() / _native.min());  // (180 - 0) / (2500 - 500) = 0.09 deg/us
        set_conversions(m, 0.0);
        governor_pc.set(95);
        idle_si.set_limits(43.0, 75.0);
        idle_si.set(58.0);
        ServoMotor2::setup();

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
// note: if devices.h gets to be too long, we can (and maybe just should) move this to a separate file, it's not really a device...
// simulator manages the source handling logic for all simulatable components. Currently, components can recieve simulated input from either the touchscreen, or from
// note: this class is designed to be backwards-compatible with existing code, which does everything with global booleans. if/when we switch all our Devices to use sources,
//       we can simplify the logic here a lot.

// Simulator class: manager class for the simulator settings and status. Note simulation values are not assigned here
//   appropriately, the objects for each simulated transducer would s  objects to sensorather, a simulation management object.
class Simulator {
  private:
    // note: if we only simulated devices, we could keep track of simulability in the Device class. We could keep the default source in Device the same way.
    // 3-tuple for a given component, keeping track of simulability status (whether this component is allowed to be simulated), an associated Device (a pointer to the actual component),
    // and a default source (the mode we would like the component to switch back to when it stops being simulated)
    typedef std::tuple<bool, Device*, src> simulable_t;
    std::map<sens, simulable_t> _devices; // a collection of simulatable components
    bool _enabled = false; // keep track of whether the simulator is running or not
    int _registered_device_count = 0;
    sens _potmap; // keep track of which component is getting info from the pot
    Potentiometer& _pot;
    Preferences* _myprefs;
  public:
    Simulator(Potentiometer& pot_arg, Preferences* myprefs) : _pot(pot_arg), _myprefs(myprefs) {
        for (int sensor = (int)sens::none + 1; sensor < (int)sens::NumSensors; sensor++) set_can_sim((sens)sensor, false);   // initially turn off simulation of sensors  // static constexpr bool initial_sim_joy = false;
        set_potmap(); // set initial pot map
    }  // syspower, ignition removed, as they are not sensors or even inputs

    void updateSimulationStatus(bool enableSimulation) {
        if (_enabled == enableSimulation) return;  // if the simulation status hasn't changed, there's nothing to do
        for (auto &deviceEntry : _devices) {  // iterate over all devices
            bool can_sim = std::get<0>(deviceEntry.second);
            if (can_sim && _potmap != deviceEntry.first) {  // if the device can be simulated and isn't being mapped from the potentiometer
                Device *d = std::get<1>(deviceEntry.second);
                if (d != nullptr) {  // if the device exists... (note: the nullptr checks here and below exist so that we can work with boolean components as well as Devices, for backwards compatability)
                    if (enableSimulation) d->set_source(src::Sim);  // if we're enabling the simulation, set the device's source to the simulator
                    else {                                          // otherwise, set it to its default mode
                        src default_mode = std::get<2>(deviceEntry.second);
                        d->set_source(default_mode);
                    }
                }
            }
        }
        _enabled = enableSimulation;  // update the simulation status
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
                if (arg_sensor == _potmap) d.set_source(src::Pot); // ...and the pot is supposed to map to this component, thenset the input source for the associated Device to read from the pot
                else if (_enabled) d.set_source(src::Sim); // ...or otherwise if the pot isn't mapping to this component, but the simulator is running, then set the input source for the associated Device to take values from the simulator
            }
        }
        if (d.can_source(src::Pot)) d.attach_pot(_pot); // if this device can be mapped from the pot, connect it to pot input
        _devices[arg_sensor] = simulable_t(can_sim, &d, default_mode); // store info for this component
        _registered_device_count++;
    }
    
    bool can_sim(sens arg_sensor) {  // check if a component can be simulated (by either the touchscreen or the pot)
        auto kv = _devices.find(arg_sensor); // look for the component
        if (kv != _devices.end()) return std::get<0>(kv->second); // if it exists, check the simulability status
        return false; // couldn't find component, so there's no way we can simulate it
    }
    // bool touchable(sens arg_sensor) {
    //     return can_sim(arg_sensor) && (sources[static_cast<int>(arg_sensor)] == static_cast<int>(src::Touch));
    // }
  private:
    void set_can_sim_nosave(sens arg_sensor, bool can_sim) {  // set a device so simulator will include it when enabled. does not write to flash
        auto kv = _devices.find(arg_sensor); // look for component
        if (kv != _devices.end()) { // if an entry for this component already exists, check if the new simulatability status is different from the old
            bool old_can_sim = std::get<0>(kv->second);
            if (can_sim != old_can_sim) { // if the simulation status has changed, we need to update the input source for the component
                src default_mode = src::Undef;
                Device *d = std::get<1>(kv->second);
                if (d != nullptr) { // if there is no associated Device with this component then input handling is done in the main code
                    default_mode = std::get<2>(kv->second); // preserve the stored default controller mode
                    if (can_sim) { // if we just enabled simulatability...
                        if (arg_sensor == _potmap) d->set_source(src::Pot); // if the pot is supposed to map to this component, then set the input source for the associated Device to read from the pot
                        else if (_enabled) d->set_source(src::Sim); // otherwise if the simulator is running, then set the input source for the associated Device to read from the simulator
                    }
                    else d->set_source(default_mode); // we disabled simulation for this component, set it back to its default input source
                }
                kv->second = simulable_t(can_sim, d, default_mode); // update the entry with the new simulatability status
            }
        }
        else _devices[arg_sensor] = simulable_t(can_sim, nullptr, src::Undef); // add a new entry with the simulatability status for this component
    }
  public:
    void set_can_sim(sens arg_sensor, bool can_sim) {  // this wrapper function sets a device as able to be simulated, then store to flash
        set_can_sim_nosave(arg_sensor, can_sim);  // set the device simulatability status
        save_cansim();  // re-save can-sim status word to flash (makes setting permanent across boots)
    }
    void set_can_sim(sens arg_sensor, int can_sim) { set_can_sim(arg_sensor, (can_sim > 0)); } // allows interpreting -1 as 0, convenient for our tuner etc.
    void save_cansim() {  // compress can_sim status of all devices into a 32 bit int, and save it to flash
        uint32_t simword = 0;
        for (int s=1; s<(int)sens::NumSensors; s++) simword = simword | ((uint32_t)can_sim((sens)s) << s);
        _myprefs->putUInt("cansim", simword);
    }
    void recall_cansim() {  // pull 32 bit int containing can_sim status of all devices from previous flash save, and set all devices accordingly
        uint32_t simword = _myprefs->getUInt("cansim", 0);
        for (int s=1; s<(int)sens::NumSensors; s++) set_can_sim_nosave((sens)s, (bool)((simword >> s) & 1));
    }
    // set the component to be overridden by the pot (the pot can only override one component at a time)
    void set_potmap(sens arg_sensor) {
        if (arg_sensor != _potmap) { // if we're mapping to a different component, we need to reset the input source for the old one
            auto kv = _devices.find(_potmap);
            if (kv != _devices.end()) {
                Device *d = std::get<1>(kv->second);
                if (d != nullptr) { // if we were mapping to a component with an associated Device...
                    bool _can_sim = std::get<0>(kv->second);
                    if (_enabled && _can_sim) d->set_source(src::Sim); // ...and the simulator is on, and we're able to be simulated, then set the input source to the simulator
                    else { // (otherwise either the simulator is off or we aren't allowing simulation for this component)
                        src default_mode = std::get<2>(kv->second);
                        d->set_source(default_mode); // then set the input source for the component to its default
                    }
                } // ...else this component is a boolean, and input source handling is done elsewhere
            }
            kv = _devices.find(arg_sensor); // we need to set the input source of the newly-overridden component
            if (kv != _devices.end()) {
                Device *d = std::get<1>(kv->second);
                if (d != nullptr ) { // if  we're mapping to a component with an associated device, we need to change the input source to the pot
                    if (d->can_source(src::Pot)) { // ...and we're allowed to map to this component...
                        bool _can_sim = std::get<0>(kv->second);
                        if (_can_sim) d->set_source(src::Pot); // if we allow simulation for this componenent, then set its input source to the pot
                    }
                    else ezread.squintf("invalid pot map: %d\n", arg_sensor);
                }
            }
            _potmap = arg_sensor;
            _myprefs->putUInt("potmap", static_cast<uint32_t>(_potmap));
        }
    }
    void set_potmap() { set_potmap(static_cast<sens>(_myprefs->getUInt("potmap", static_cast<uint32_t>(sens::none)))); }
    bool potmapping(sens s) { return can_sim(s) && _potmap == s; }  // query if a certain sensor is being potmapped
    bool potmapping(int s) { return can_sim(static_cast<sens>(s)) && (_potmap == static_cast<sens>(s)); }  // query if a certain sensor is being potmapped        
    bool potmapping() { return can_sim(_potmap) && !(_potmap == sens::none); }  // query if any sensors are being potmapped
    int potmap() { return static_cast<int>(_potmap); }  // query which sensor is being potmapped
    bool enabled() { return _enabled; }
    bool* enabled_ptr() { return &_enabled; }
    int registered_device_count() { return _registered_device_count; }
};

// rmtinput is used by hotrc class below
class RMTInput {  // note: for some reason if we ever stop reading our rmt objects, the code fails and floods the console w/ rmt failures :(
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
        _config.rx_config.filter_en = true;          // enable the filter
        _config.rx_config.filter_ticks_thresh = 100; // set the filter threshold
        _config.rx_config.idle_threshold = 12000;    // set the idle threshold

        esp_err_t config_result = rmt_config(&_config);
        if (config_result != ESP_OK) ezread.squintf("Failed to configure RMT: %d\n", config_result);  // while (1); // halt execution
        
        esp_err_t install_result = rmt_driver_install(channel_, 2000, 0);
        if (install_result != ESP_OK) ezread.squintf("Failed to install RMT driver: %d\n", install_result);  // while (1); // halt execution
        
        rmt_get_ringbuf_handle(channel_, &rb_);
        if (rb_ == NULL) Serial.println("Failed to initialize ring buffer");  // while (1); // halt execution
        
        esp_err_t start_result = rmt_rx_start(channel_, 1);
        if (start_result != ESP_OK) ezread.squintf("Failed to start RMT receiver: %d\n", start_result); // while (1); // halt execution
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
        return (persistence) ? pulse_width_last : pulse_width; // no data
    }
  private:
    rmt_channel_t channel_;
    gpio_num_t gpio_;
    int pulse_width, pulse_width_last;
    float scale_factor = 0.625f;
    RingbufHandle_t rb_;
};
class Hotrc {  // all things Hotrc, in a convenient, easily-digestible format the kids will just love
  public:
    bool _verbose = false;  // causes console print whenever an unfiltered switch event is queried externally, thus canceling the in-progress filtering
    float absmin_us = 880.0f;  // min configurable pulsewidth for the hotrc
    float absmax_us = 2120.0f; // max configurable pulsewidth for the hotrc
    float pc[NumAxes][NumValues], sim_raw_pc[NumAxes]; // values range from -100% to 100%. pc[][] are all derived or auto-assigned. sim_raw_pc[] are simulated inputs
    float us[NumChans][NumValues] = {  // these inherently integral values are kept as floats for more abstractified unit management
        {  969, 1473, 1977, 0, 1500, 0, 0, 0 },     // (974-1981) 1000-30+1, 1500-30,  2000-30-2   // [Horz] [OpMin/Cent/OpMax/Raw/Filt/-/-/Margin]
        { 1080, 1583, 2087, 0, 1500, 0, 0, 0 },     // (1084-2091) 1000+80+1, 1500+80,  2000+80-2, // [Vert] [OpMin/Cent/OpMax/Raw/Filt/-/-/Margin]
        { 1200, 1606, 1810, 0, 1500, 0, 0, 0 },     // (1204-1809) 1000+150+1,   1500, 2000-150-2, // [Ch3] [OpMin/Cent/OpMax/Raw/Filt/-/-/Margin]
        { 1300, 1505, 1710, 0, 1500, 0, 0, 0 }, };  // (1304-1707) 1000+250+1,   1500, 2000-250-2, // [Ch4] [OpMin/Cent/OpMax/Raw/Filt/-/-/Margin]
        // note: opmin/opmax range should be set to be just smaller than the actual measured range, to prevent out-of-range errors. this way it will reach all the way to 100%
        //   margin should be set just larger than the largest difference between an opmin/max value and its corresponding actual measured limit, to prevent triggering errors
    float deadband_pc, deadband_us = 20.0f;  // size of each side of the center deadband in us.  pc value is derived  // was 15
    float sim_deadband_pc = 7.5f;  // when simulating joystick using display buttons, allows rounding of near-zero values to zero
    float margin_us = 15.0f;    // all [Margin] values above are derived from this by calling derive()  // was 12
    float failsafe_us = 880.0f; // Hotrc must be configured per the instructions: search for "HotRC Setup Procedure"
    float failsafe_margin_us = 100.0f; // in the carpet dumpster file: https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA/edit
    float ema_alpha = 0.065f;          // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1).
  private:
    Simulator* sim;
    Potentiometer* pot;
    bool _radiolost = true, _radiolost_untested = true; // has any radiolost condition been detected since boot?  allows us to verify radiolost works before driving 
    float spike_us[NumAxes] = { us[Horz][Filt], us[Vert][Filt] }; // [Horz/Vert]  // added
    float ema_us[NumAxes] = { us[Horz][Filt], us[Vert][Filt] }; // [Horz/Vert]  // un-deprecated. seeded with fake initial values to not break the ema filter functionality
    bool _sw_val[NumChans], _sw_new[NumChans]; // whether pulsewidth is above center. note: index[2]=Ch3, index[3]=Ch4 (1st 2 indices unused)
    bool _sw_pending[NumChans], _sw_event_unfilt[NumChans], _sw_event_filt[NumChans]; // track unfilt events, filt events, and pending filt events
    int _last_ch4_source = StartUnknown;  // part of debugging phantom starts
    RMTInput rmt[NumChans] = {
        RMTInput(RMT_CHANNEL_4, gpio_num_t(hotrc_ch1_h_pin)),  // hotrc[Horz]
        RMTInput(RMT_CHANNEL_5, gpio_num_t(hotrc_ch2_v_pin)),  // hotrc[Vert]
        RMTInput(RMT_CHANNEL_6, gpio_num_t(hotrc_ch3_pin)),    // hotrc[Ch3]
        RMTInput(RMT_CHANNEL_7, gpio_num_t(hotrc_ch4_pin)),    // hotrc[Ch4]
    };
    Timer simBtnTimer, ch4BtnTimer;  // keep track of last activity on all possible sources of ReqOn, to serve as a safety net preventing phantom starts
    float us_to_pc(int axis, float _us) {
        if (_us >= us[axis][Cent]) return map(_us, us[axis][Cent], us[axis][OpMax], pc[axis][Cent], pc[axis][OpMax]);
        return map(_us, us[axis][Cent], us[axis][OpMin], pc[axis][Cent], pc[axis][OpMin]);    
    }
    void derive() {  // need to run this upon objct creation, and again whenever any tunable parameter is changed
        for (int ch=0; ch<NumChans; ch++) {                      // derive us values for all 4 channels
            us[ch][Margin] = margin_us;                          // same margin value is ultimately used on all channels
            us[ch][OpMin] = constrain(us[ch][OpMin], absmin_us, us[ch][Cent] - us[ch][Margin]);  // ensure any oplimit tuning stays in sensible range
            us[ch][OpMax] = constrain(us[ch][OpMax], us[ch][Cent] + us[ch][Margin], absmax_us);
        }
        for (int axis=Horz; axis<=Vert; axis++) {  // derive pc values for the 2 analog directional axes
            pc[axis][OpMin] = -100.0;
            pc[axis][Cent] = 0.0;
            pc[axis][OpMax] = 100.0;
            pc[axis][Margin] = us_to_pc(axis, us[axis][Margin]);
        }
        set_deadband_us(deadband_us);  // set deadband to itself, b/c the setter forces derivation of percent value
    }
  public:
    Hotrc(Simulator* _sim, Potentiometer* _pot) : sim(_sim), pot(_pot) { derive(); }
    void setup() {
        ezread.squintf(ezread.highlightcolor, "Hotrc init.. starting rmt..\n");
        for (int axis=Horz; axis<=Ch4; axis++) rmt[axis].init();  // set up 4 RMT receivers, one per channel
        read_all_channels();  // must read values before initializing toggles below
        toggles_init();       // initialize toggles to prevent spurious sw events on boot
    }
    void update() {          // run this in each loop
        read_all_channels(); // read new raw pwm values from rmt buffer. critical to do this continually, fast enough to avoid rmt buffer overflow 
        radiolost_update();  // determine if the radio receiver detects good signal
        toggles_update();    // handle button presses on the digital channels
        direction_update();  // update directional values from the analog channels
    }
    void set_deadband_us(float val) {
        deadband_us = constrain(val, 0.0f, us[Horz][OpMax] - us[Horz][Cent]);  // using Horz for this b/c it's the same for either axis
        deadband_pc = us_to_pc(Horz, val);  // convert in order to derive pc deadband value
    }
    void set(float* member, float val) {  // generic setter for any member floats. basically makes sure to rerun derive() after
        *member = val;
        derive();
    }
    void set(float* member, int val) { set(member, static_cast<float>(val)); }  // to accept int arguments too
    int joydir(int axis=Vert) {
        if (axis == Vert) return (pc[axis][Filt] > pc[axis][Cent]) ? HrcUp : (pc[axis][Filt] < pc[axis][Cent]) ? HrcDn : HrcCent;
        return (pc[axis][Filt] > pc[axis][Cent]) ? HrcRt : (pc[axis][Filt] < pc[axis][Cent]) ? HrcLt : HrcCent;
    }  // return (pc[axis][Filt] > pc[axis][Cent]) ? ((axis == Vert) ? HrcUp : HrcRt) : (pc[axis][Filt] < pc[axis][Cent]) ? ((axis == Vert) ? HrcDn : HrcLt) : HrcCent;
    void sim_button_press(int chan) {  // handles ch4 action button on display menu
        _sw_event_filt[chan] = true;
        if (chan == Ch4) {
            simBtnTimer.reset();  // so we can know time since last sim button press, to help prevent phantom starter events
            _last_ch4_source = StartTouch;  // also to help phantom starter events
        }
    }
    bool sw_event_filt(int ch) {  // returns if a stable & filtered event occurred on the given channel, and if so, resets that channel
        if (_sw_event_filt[ch]) {
            toggle_reset(ch);  // if there is an event, reset all channel events and in-progress pending events
            if (_verbose) ezread.squintf("hotrc: ch%d filt event serviced\n", (ch == Ch3) ? 3 : 4);
            return true;
        }
        return false;
    }
    bool sw_event_unfilt(int ch) {  // returns if any event occurred on the given channel, and if so, resets that channel. note: calling this cancels any in-progress event filtering
        if (_sw_event_unfilt[ch] || _sw_event_filt[ch]) {
            toggle_reset(ch);  // if there is an event, reset all channel events and in-progress pending events
            if (_verbose) ezread.squintf("hotrc: ch%d unfilt event serviced\n", (ch == Ch3) ? 3 : 4);
            return true;
        }
        return false;
    }
    bool radiolost() { return _radiolost; }
    bool radiolost_untested() { return _radiolost_untested; }
    bool* radiolost_ptr() { return &_radiolost; }
    bool* radiolost_untested_ptr() { return &_radiolost_untested; }
    int last_ch4_source() { return _last_ch4_source; }
    int sim_button_last_ms() { return simBtnTimer.elapsed() / 1000; }  // returns time since last display menu ch4 button press. to help prevent phantom starter turnon
    int ch4_button_last_ms() { return ch4BtnTimer.elapsed() / 1000; }  // returns time since last actual ch4 button press. to help prevent phantom starter turnon
  private:
    void read_channel(int ch) { us[ch][Raw] = (float)(rmt[ch].readPulseWidth(true)); }
    void read_all_channels() {
        static bool sw_old[NumChans];  // for debug. note on first time thru sw_old is meaningless, may cause a print
        for (int ch = Horz; ch <= Vert; ch++) {      // analog channels
            read_channel(ch);                        // read channel
            pc[ch][Raw] = us_to_pc(ch, us[ch][Raw]); // update analog percent value from us read value
        }
        for (int ch = Ch3; ch <= Ch4; ch++) {        // digital channels
            read_channel(ch);                        // read channel
            _sw_new[ch] = toggle_getval(ch);         // set switch value based on new reading
            if (_verbose && _sw_new[ch] != sw_old[ch]) ezread.squintf("hotrc: read ch%d toggle %d->%d\n", (ch == Ch3) ? 3 : 4, (int)sw_old[ch], (int)_sw_new[ch]);
            sw_old[ch] = _sw_new[ch];
        }
    }
    void radiolost_update() {  // note: must initialize _radiolost & _radiolost_untested to true, and must run after reading us[Vert][Raw]
        static bool lost_last = _radiolost;
        _radiolost = (us[Vert][Raw] <= failsafe_us + failsafe_margin_us);  // is the newest reading in the failsafe range?
        if (_radiolost != lost_last) toggles_init(); // when radio comes in or out, re-initialize toggles to prevent spurious sw events
        if (_radiolost) _radiolost_untested = false;  // on first valid detection of radiolost, radiolost detection is known to work
        lost_last = _radiolost;  // remember current reading for comparison on next loop
    }
    void toggles_init() { for (int ch=Ch3; ch<=Ch4; ch++) toggle_reset(ch); } // init switches to newest read values to prevent spurious events. run on boot & when radio gets lost or un-lost
    bool toggle_getval(int ch) { return (us[ch][Raw] >= us[ch][Cent]); } // pulsewidth higher than center value is considered val=1
    void toggle_reset(int ch) {                         // resets events and pending events on the given channel
        _sw_new[ch] = _sw_val[ch] = toggle_getval(ch);  // set both last and val based on last read value
        _sw_event_filt[ch] = _sw_event_unfilt[ch] = _sw_pending[ch] = false;  // cancel all events
    }
    // hotrc digital button handler which rejects spurious values which could cause false events (re: phantom starter bug)
    //   seems to work except it generates a Ch4 sleep request shortly after boot (should be a simple fix)
    void toggles_update() {  // note, below the two timer[] indices are each 2 less than those of their corresponding switches (avoids creating 2 extra Timer instances)
        static Timer filttimer[NumChans-2] = { 60000, 60000 }; // timers ensure no spurious events, changes must persist this long to count (no human can click this fast). see comment above re: offset indices
        int event_expiration_us = 1500000;             // to cancel events if they have become stale on a human timescale
        for (int ch = Ch3; ch <= Ch4; ch++) {          // do once for each digital channel
            if (_sw_new[ch] != _sw_val[ch]) {          // if new read value differs from prev valid value ...
                if (!_sw_pending[ch]) {
                    _sw_pending[ch] = _sw_event_unfilt[ch] = true; // if we don't already have a new pending value change, flag an unfiltered event occurred, and we have a potential filtered value change, 
                    if (_verbose) ezread.squintf("hotrc: new ch%d unfilt event\n", (ch == Ch3) ? 3 : 4);
                    filttimer[ch-2].reset();           // set us up for filtering the next value change
                }
                else if (filttimer[ch-2].expired()) {  // if we do have a switch pending, and the validity timer expires. until the timer runs out all new readings must also differ from prev valid value, for the pending value to commit
                    _sw_val[ch] = _sw_new[ch];         // commit the new value
                    _sw_event_filt[ch] = true;         // flag that a filtered switch event occurred, detectable by external code and reset outside this function
                    if (_verbose) ezread.squintf("hotrc: new ch%d filt event\n", (ch == Ch3) ? 3 : 4);
                    _sw_pending[ch] = false;           // reset pending state
                    kick_inactivity_timer(HuRCTog);    // register that human activity occurred
                    if (ch == Ch4) {                   // special mechanisms for ch4 to help prevent phantom starter events
                        ch4BtnTimer.reset();           // so we can know time since last ch4 button press
                        _last_ch4_source = StartHotrc; // also to help phantom starter events
                    }
                }  // ezread.squintf("Hrc Ch%d l:%d v:%d p:%d f:%d u:%d\n", (ch == Ch3) ? 3 : 4, (int)_sw_val[ch], (int)_sw_new[ch], (int)_sw_pending[ch], _sw_event_filt[ch], _sw_event_unfilt[ch]);
            }
            else {  // otherwise if the new read value is equal to the previous valid value
                if (_sw_pending[ch]) ezread.squintf(ezread.sadcolor, "warn: hotrc ch%d spurious reading detect\n", (ch == Ch3) ? 3 : 4);
                _sw_pending[ch] = false; // cancel any pending event due to the new value didn't hold throughout the timeout period
            }
            if (_sw_event_unfilt[ch] && (filttimer[ch-2].elapsed() >= event_expiration_us)) {  // is there a super stale event?
                toggle_reset(ch);  // cancel it
                if (_verbose) ezread.squintf("hotrc: expired ch%d event\n", (ch == Ch3) ? 3 : 4);
            }
        }
    }
    // enforces deadbands if within them (returning true), otherwise rescales to full range (returning false)
    bool remove_deadbands(float* _val, float _db, float _cent, float _min, float _max) {
        bool in_deadbands = false;
        if (*_val > _cent + _db) *_val = map(*_val, _cent + _db, _max, _cent, _max);      // if above upper deadband, scale to full range 
        else if (*_val < _cent - _db) *_val = map(*_val, _cent - _db, _min, _cent, _min); // if below lower deadband, scale to full range 
        else {                                                                            // if within deadbands, set to center value
            *_val = _cent;
            in_deadbands = true;
        }
        return in_deadbands;
    }
    void clean_sim_vals() {  // prevent false values when turning on simulator
        static bool simulating_last = sim->simulating(sens::joy);
        if (sim->simulating(sens::joy) && !simulating_last) 
            for (int axis = Horz; axis <= Vert; axis++) sim_raw_pc[axis] = pc[axis][Filt];
        simulating_last = sim->simulating(sens::joy);
    }
    void set_direction(int axis) {  // for radio values, do filtration and set us and pc values based on new reading
        spike_us[axis] = spike_filter(axis, us[axis][Raw]);  // apply spike filter on raw reading
        us[axis][Filt] = ema_us[axis] = ema_filt(spike_us[axis], ema_us[axis], ema_alpha); // apply ema filter on spike filter output
        bool in_deadbands = remove_deadbands(&us[axis][Filt], deadband_us, us[axis][Cent], us[axis][OpMin], us[axis][OpMax]); // enforce deadbands
        if (_radiolost) pc[axis][Filt] = pc[axis][Cent];     // if radio lost set pc value to center for sanity, but keep us value for debug
        else {                                               // otherwise if radio is not lost
            pc[axis][Filt] = us_to_pc(axis, us[axis][Filt]); // convert filtered us value to percent
            if (!in_deadbands) kick_inactivity_timer((axis == Horz) ? HuRCJoy : HuRCTrig); // register evidence of user activity
        }
    }
    void set_sim_direction(int axis) {     // when simulating, set values respecting deadbands
        pc[axis][Filt] = sim_raw_pc[axis]; // commit externally set or potmapped sim value
        remove_deadbands(&pc[axis][Filt], sim_deadband_pc, pc[axis][Cent], pc[axis][OpMin], pc[axis][OpMax]); // enforce sim deadbands
    }
    void direction_update() { // update all directional values based on latest read or simulated values
        clean_sim_vals(); // prevent false values when turning on simulator
        if (sim->simulating() && sim->simulating(sens::joy)) set_sim_direction(Vert); // correctly navigate confusing sim function naming
        else set_direction(Vert);
        if (sim->simulating(sens::joy)) {
            if (sim->potmapping(sens::joy)) sim_raw_pc[Horz] = pot->mapToRange(pc[Horz][OpMin], pc[Horz][OpMax]); // if potmapping set horz to pot
            set_sim_direction(Horz);
        }
        else set_direction(Horz);
        for (int axis = Horz; axis <= Vert; axis++) pc[axis][Filt] = constrain(pc[axis][Filt], pc[axis][OpMin], pc[axis][OpMax]);
    }
    // spike_filter() : I wrote this custom filter to clean up some specific anomalies i noticed with the pwm signals coming
    // from the hotrc, where often the incoming values change suddenly then (usually) quickly jump back by the same amount. 
    // This works by pushing new hotrc readings into a FIFO ring buffer, and replacing any well-defined spikes with values 
    // interpolated from before and after the spike, thereby erasing any spikes that recover fast enough.
    // Also if a detected cliff edge (potential spike) doesn't recover in time, it will smooth out the transition linearly.
    // The cost of this is our readings are delayed by a number of readings (equal to the maximum erasable spike duration).
    static const int ringdepth = 9;  // more depth will reject longer spikes at the expense of increased controller delay
    float spike_cliff[NumAxes] = { 6.0f, 6.0f };  // spike_cliff is min diff of consecutive values to count as a spike
    int prespike_idx[NumAxes] = { -1, -1 }, ringidx[NumAxes] = { 1, 1 };  // prespike of -1 means no current spike
    float ringbuf[NumAxes][ringdepth];
    float spike_filter(int axis, float new_val) {  // pass a fresh reading in, will return a filtered reading to use instead
        static int num_calls[NumAxes] = { 0, 0 };  // track #function calls, to mitigate the initially-empty ring buffer 
        static bool spike_signbit[NumAxes];
        int previdx = (ringdepth + ringidx[axis] - 1) % ringdepth; // previdx is where the incoming new value will be stored
        if (++num_calls[axis] <= ringdepth) ringbuf[axis][previdx] = new_val;  // until buf is filled, fake the values
        float this_delta = new_val - ringbuf[axis][previdx];   // value change since last reading
        if (std::abs(this_delta) > spike_cliff[axis]) {        // if new value is a cliff edge (start or end of a spike)
            if (prespike_idx[axis] == -1) {                    // if this cliff edge is the start of a new spike
                prespike_idx[axis] = previdx;                  // save idx of last good value just before the cliff
                spike_signbit[axis] = std::signbit(this_delta);     // save the direction of the cliff
            }
            else if (spike_signbit[axis] == std::signbit(this_delta)) {  // if this cliff edge deepens an in-progress spike, or is a continuance of a valid rapid change
                inject_interpolations(axis, previdx, ringbuf[axis][previdx]); // smooth out the values between the last cliff & previous value
                prespike_idx[axis] = previdx;                                 // consider this cliff edge the start of the spike instead
            }
            else {                                                   // if this cliff edge is a recovery of an in-progress spike
                inject_interpolations(axis, ringidx[axis], new_val); // fill in the spiked value section with interpolated values
                prespike_idx[axis] = -1;                             // cancel the current spike
            }
        }
        else if (prespike_idx[axis] == ringidx[axis]) {                   // if the whole buffer cycled before the current spike recovered
            inject_interpolations(axis, previdx, ringbuf[axis][previdx]); // smoothly grade the whole buffer
            prespike_idx[axis] = -1;                                      // cancel the current spike
        }
        float returnval = ringbuf[axis][ringidx[axis]]; // pull the incumbent value at current (oldest) index for return
        ringbuf[axis][ringidx[axis]] = new_val;         // save the newest reading into same location
        ++ringidx[axis] %= ringdepth;                   // advance the index (with wraparound) for next time
        return returnval;                               // return the value, now with any spikes removed or smoothed by the filter
    }
    void inject_interpolations(int axis, int endspike_idx, float endspike_val) {  // replaces values between indices w/ linear interpolated values
        int spike_length = ((ringdepth + endspike_idx - prespike_idx[axis]) % ringdepth) - 1;  // equal to the spiking values count plus one
        if (spike_length <= 0) return;  // two cliffs in the same direction on consecutive readings needs no adjustment, also prevents divide by zero 
        float interp_slope = (endspike_val - ringbuf[axis][prespike_idx[axis]]) / spike_length;
        for (int idx=1; idx<=spike_length; idx++)  // fill specified section of the buffer with interpolated values
            ringbuf[axis][(prespike_idx[axis] + idx) % ringdepth] = ringbuf[axis][prespike_idx[axis]] + idx * interp_slope;
    }
};