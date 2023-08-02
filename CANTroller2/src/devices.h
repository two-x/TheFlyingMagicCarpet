#pragma once
#ifndef CLASSES_H
#define CLASSES_H
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <map>
#include <tuple>
// #include <ranges> // for std::view::keys
#include <memory> // for std::shared_ptr
#include "Arduino.h"
#include "utils.h"
#include "uictrl.h"
// #include "xtensa/core-macros.h"  // access to ccount register for esp32 timing ticks

// Param is a value which is constrained between min/max limits, representing a "raw" (aka unfiltered) quantity. A value with tight limits
// (wehere min=val=max) is constant and cannot be changed without changing the limits. An unconstrained value can be represented by setting
// either or both min/max to infinity.
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
    Param(VALUE_T arg_val, std::shared_ptr<VALUE_T> arg_min, std::shared_ptr<VALUE_T> arg_max) {
        _val = std::make_shared<VALUE_T>(arg_val);
        _min = std::make_shared<VALUE_T>();
        _max = std::make_shared<VALUE_T>();
        set_limits(arg_min, arg_max);
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
    void set_limits(std::shared_ptr<VALUE_T> arg_min, std::shared_ptr<VALUE_T> arg_max) { // Use if min/max are external
        if (arg_min.get() > arg_max.get())
            printf("Error: *min is > *max\n");
        else {
            _min = arg_min;
            _max = arg_max;
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

    VALUE_T get() { return *_val; }
    VALUE_T get_min() { return *_min; }
    VALUE_T get_max() { return *_max; }
    std::shared_ptr<VALUE_T> get_ptr() { return _val; }
    std::shared_ptr<VALUE_T> get_min_ptr() { return _min; }
    std::shared_ptr<VALUE_T> get_max_ptr() { return _max; }
    VALUE_T get_last() { return _last; } // NOTE: currently unused, do we still need this for drawing purposes?
    bool get_saturated() { return _saturated; }
};

enum class ControllerMode : uint8_t {UNDEF=0, FIXED, PIN, TOUCH, POT, CALC};

// Device class - is a base class for any connected system device or signal associated with a pin
class Device {
  protected:
    // Which types of sources are possible for this device?
    uint8_t _pin;
    bool _enabled = true;
    Potentiometer* _pot; // to pull input from the pot if we're in simulation mode
    ControllerMode _source = ControllerMode::UNDEF;
    bool _can_source[6] = { true,       // UNDEF
                            true,       // FIXED
                            false,      // PIN
                            true,       // TOUCH
                            false,      // POT
                            false };    // CALC

    // source handling functions (should be overridden in child classes as needed)
    virtual void handle_undef_mode() {}
    virtual void handle_fixed_mode() {}
    virtual void handle_pin_mode() {}
    virtual void handle_touch_mode() {}
    virtual void handle_pot_mode() {}
    virtual void handle_calc_mode() {}
  public:
    Timer timer;  // Can be used for external purposes

    Device() = delete; // should always be created with a pin
    // NOTE: should we start in PIN mode?
    Device(uint8_t arg_pin) : _pin(arg_pin) {}

    bool can_source(ControllerMode arg_source) { return _can_source[static_cast<uint8_t>(arg_source)]; }
    bool set_source(ControllerMode arg_source) {
        if (_can_source[static_cast<uint8_t>(arg_source)]) {
            _source = arg_source;
            return true;
        } 
        return false;
    }

    void update() {
        if (!_enabled) return; // do nothing if the Device is disabled
        switch (_source) {
            case ControllerMode::UNDEF:
                handle_undef_mode();
                break;
            case ControllerMode::FIXED:
                handle_fixed_mode();
                break;
            case ControllerMode::PIN:
                handle_pin_mode();
                break;
            case ControllerMode::TOUCH:
                handle_touch_mode();
                break;
            case ControllerMode::POT:
                handle_pot_mode();
                break;
            case ControllerMode::CALC:
                handle_calc_mode();
                break;
            default:
                // should never get here
                printf("invalid Device source: %d\n", _source);
        }
    }

    void set_pot_input(Potentiometer &pot_arg) {
        _pot = &pot_arg;
    }

    void set_enabled(bool arg_enable) { _enabled = arg_enable; }
    void set_can_source(ControllerMode arg_source, bool is_possible) { _can_source[static_cast<uint8_t>(arg_source)] = is_possible; }
    ControllerMode source() { return _source; }
    uint8_t get_pin() { return _pin; }
    bool get_enabled() { return _enabled; }
};

// Device::Transducer is a base class for any system devices that convert real-world values <--> signals in either direction. It has a "native"
// value which represents the sensed or driven hardware input/output. It also has a "human" value which represents the logical or human-readable
// equivalent value. Adjusting either value will automatically change the other one.
enum class TransducerDirection : uint8_t {REV, FWD}; // possible dir values. REV means native sensed value has the opposite polarity of the real world effect (for example, if we sense fewer us per rotation, the engine is going faster)
template<typename NATIVE_T, typename HUMAN_T>
class Transducer : public Device {
  protected:
    // Multiplier and adder values to plug in for unit conversion math
    float _m_factor;
    float _b_offset;  
    bool _invert;  // Flag to indicated if unit conversion math should multiply or divide
    TransducerDirection dir = TransducerDirection::FWD; // NOTE: what is this for, exactly?
    
    // conversion functions (can be overridden in child classes different conversion methods are needed)
    virtual HUMAN_T from_native(NATIVE_T arg_val_native) {
        float arg_val_f = static_cast<float>(arg_val_native); // convert everything to floats so we don't introduce rounding errors
        float min_f = static_cast<float>(native.get_min());
        float max_f = static_cast<float>(native.get_max());
        float ret = -1;
        if (!_invert) {
            if (dir == TransducerDirection::REV) {
                ret = min_f + (max_f - (_b_offset + (_m_factor * arg_val_f)));
            }
            ret = _b_offset + (_m_factor * arg_val_f);
        } else if (arg_val_f) { // NOTE: isn't 0.0 a valid value tho?
            if (dir == TransducerDirection::REV) {
                ret = min_f + (max_f - (_b_offset + (_m_factor / arg_val_f)));
            }
            ret = _b_offset + (_m_factor / arg_val_f);
        } else {
            printf ("Error: unit conversion refused to divide by zero\n");
            // NOTE: hmmmm, couldn't -1 be a valid value in some caes?
        }
        return static_cast<HUMAN_T>(ret);
    }

    virtual NATIVE_T to_native(HUMAN_T arg_val_human) {
        float arg_val_f = static_cast<float>(arg_val_human); // convert everything to floats so we don't introduce rounding errors
        float min_f = static_cast<float>(human.get_min());
        float max_f = static_cast<float>(human.get_max());
        float ret = -1;
        if (dir == TransducerDirection::REV) {
            arg_val_f = min_f + (max_f - arg_val_f);
        }
        if (_invert && (arg_val_f - _b_offset)) {
            ret = _m_factor / (arg_val_f - _b_offset);
        } else if (!_invert && _m_factor) {
            ret = (arg_val_f - _b_offset) / _m_factor;
        } else {
            printf ("Error: unit conversion refused to divide by zero\n");
            // NOTE: hmmmm, couldn't -1 be a valid value in some caes?
        }
        return static_cast<NATIVE_T>(ret);
    }

    // NOTE: do we really need two values? or should this just be a single value and get converted wherever needed?
    // To hold val/min/max display values in display units (like V, mph, etc.)
    Param<HUMAN_T> human;
    Param<NATIVE_T> native;
  public:
    Transducer(uint8_t arg_pin) : Device(arg_pin) {}
    Transducer() = delete;

    void set_native_limits(Param<NATIVE_T> &arg_min, Param<NATIVE_T> &arg_max) {
        if (arg_min.get() > arg_max.get()) {
            dir = TransducerDirection::REV;
            native.set_limits(arg_max, arg_min);
        }
        else {
            dir = TransducerDirection::FWD;
            native.set_limits(arg_min, arg_max);
        }
    }
    void set_human_limits(Param<HUMAN_T> &arg_min, Param<HUMAN_T> &arg_max) {
        if (arg_min.get() > arg_max.get()) {
            dir = TransducerDirection::REV;
            human.set_limits(arg_max, arg_min);
        }
        else {
            dir = TransducerDirection::FWD;
            human.set_limits(arg_min, arg_max);
        }
    }

    bool set_native(NATIVE_T arg_val_native) {
        if (native.set(arg_val_native)) {
            human.set(from_native(native.get()));
            return true;
        }
        return false;
    }
    bool add_native(NATIVE_T arg_add_native) {
        if (native.add(arg_add_native)) {
            human.set(from_native(native.get()));
            return true;
        }
        return false;
    }
    bool set_human(HUMAN_T arg_val_human) {
        if (human.set(arg_val_human)) {
            native.set(to_native(human.get()));
            return true;
        }
        return false;
    }
    bool add_human(HUMAN_T arg_add_human) {
        if (human.add(arg_add_human)) {
            native.set(to_native(human.get()));
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

    NATIVE_T get_native() { return native.get(); }
    HUMAN_T get_human() { return human.get(); }
    NATIVE_T get_min_native() { return native.get_min(); }
    NATIVE_T get_max_native() { return native.get_max(); }
    HUMAN_T get_min_human() { return human.get_min(); }
    HUMAN_T get_max_human() { return human.get_max(); }
    std::shared_ptr<NATIVE_T> get_native_ptr() { return native.get_ptr(); }
    std::shared_ptr<HUMAN_T> get_human_ptr() { return human.get_ptr(); }
};

// Sensor class - is a base class for control system sensors, ie anything that measures real world data or electrical signals 
template<typename NATIVE_T, typename HUMAN_T>
class Sensor : public Transducer<NATIVE_T, HUMAN_T> {
  protected:
    float _ema_alpha = 0.1;
    Param<HUMAN_T> _val_filt;
    // NOTE: what does ema stand for...?
    void ema(HUMAN_T arg_new_val_native) {
        float new_val = _ema_alpha * static_cast<float>(arg_new_val_native);
        float old_val = (1 - _ema_alpha) * static_cast<float>(_val_filt.get());
        _val_filt.set(static_cast<HUMAN_T>(new_val + old_val));
    }
  public:
    Sensor(uint8_t pin) : Transducer<NATIVE_T, HUMAN_T>(pin), _val_filt(this->human.get()), _ema_alpha() {
        _val_filt.set_limits(this->human.get_min_ptr(), this->human.get_max_ptr());
    }  
    void set_ema_alpha(float arg_alpha) { _ema_alpha = arg_alpha; }
    float get_ema_alpha() { return _ema_alpha; }
    HUMAN_T get_filtered_value() { return _val_filt.get(); }
    std::shared_ptr<HUMAN_T> get_filtered_value_ptr() { return _val_filt.get_ptr(); } // NOTE: should just be public?
};

// class AnalogSensor are sensors where the value is based on an ADC reading (eg brake pressure, brake actuator position, pot)
template<typename NATIVE_T, typename HUMAN_T>
class AnalogSensor : public Sensor<NATIVE_T, HUMAN_T> {
  protected:
    virtual void handle_pin_mode() {
        this->set_native(static_cast<NATIVE_T>(analogRead(this->_pin)));
        this->ema(this->human.get()); // filtered values are kept in human format
    }
    virtual void handle_touch_mode() {
        this->_val_filt.set(this->human.get());
    }
    virtual void handle_pot_mode() {
        this->human.set(this->_pot->mapToRange(this->human.get_min(), this->human.get_max()));
        this->_val_filt.set(this->human.get()); // don't filter the value we get from the pot, the pot output is already filtered
    }
  public:
    AnalogSensor(uint8_t arg_pin) : Sensor<NATIVE_T, HUMAN_T>(arg_pin) {}
};

// PressureSensor represents a brake fluid pressure sensor.
// It extends AnalogSensor to handle reading an analog pin
// and converting the ADC value to a pressure value in PSI.
class PressureSensor : public AnalogSensor<int32_t, float> {
    public:
        // NOTE: for now lets keep all the config stuff here in the class. could also read in values from a config file at some point.
        static constexpr int32_t min_adc = 658; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
        static constexpr int32_t max_adc = 2080; // Sensor measured maximum reading. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as [wimp] chris can push
        static constexpr float initial_psi_per_adc = 1000.0 * (3.3 - 0.554) / ( (adcrange_adc - min_adc) * (4.5 - 0.554) ); // 1000 psi * (adc_max v - v_min v) / ((4095 adc - 658 adc) * (v-max v - v-min v)) = 0.2 psi/adc 
        static constexpr float initial_ema_alpha = 0.1;
        static constexpr float initial_offset = 0.0;
        static constexpr bool initial_invert = false;

        PressureSensor(uint8_t arg_pin) : AnalogSensor<int32_t, float>(arg_pin) {
            _ema_alpha = initial_ema_alpha;
            _m_factor = initial_psi_per_adc;
            _b_offset = initial_offset;
            _invert = initial_invert;
            native.set_limits(min_adc, max_adc);
            human.set_limits(from_native(native.get_min()), from_native(native.get_max()));
            set_native(min_adc);
            set_can_source(ControllerMode::PIN, true);
            set_can_source(ControllerMode::POT, true);
        }
        PressureSensor() = delete;

        void setup() {
            set_pin(_pin, INPUT);
            set_source(ControllerMode::PIN);
        }
};

// BrakePositionSensor represents a linear position sensor
// for measuring brake position (TODO which position? pad? pedal?)
// Extends AnalogSensor for handling analog pin reading and conversion.
class BrakePositionSensor : public AnalogSensor<int32_t, float> {
    protected:
        // TODO: add description
        float _zeropoint;
        void handle_touch_mode() { _val_filt.set((nom_lim_retract_in + _zeropoint) / 2); } // To keep brake position in legal range during simulation
    public:
        // NOTE: for now lets keep all the config stuff here in the class. could also read in values from a config file at some point.
        static constexpr float nom_lim_retract_in = 0.506;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (in)
        static constexpr float nom_lim_extend_in = 4.624;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (in)
        static constexpr float abs_min_retract_in = 0.335;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. ("in"sandths of an inch)
        static constexpr float abs_max_extend_in = 8.300;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (in)
        static constexpr float park_in = 4.234;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (in)
        static constexpr float margin_in = .029;  // TODO: add description
        static constexpr float initial_in_per_adc = 3.3 * 10000.0 / (5.0 * adcrange_adc * 557); // 3.3 v * 10k ohm * 1/5 1/v * 1/4095 1/adc * 1/557 in/ohm = 0.0029 in/adc
        static constexpr float initial_zeropoint_in = 3.179;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (in)
        static constexpr float initial_ema_alpha = 0.25;
        static constexpr float initial_offset = 0.0;
        static constexpr bool initial_invert = false;

        BrakePositionSensor(uint8_t arg_pin) : AnalogSensor<int32_t, float>(arg_pin) {
            _ema_alpha = initial_ema_alpha;
            _m_factor = initial_in_per_adc;
            _b_offset = initial_offset;
            _invert = initial_invert;
            _zeropoint = initial_zeropoint_in;
            human.set_limits(nom_lim_retract_in, nom_lim_extend_in);
            native.set_limits(to_native(human.get_min()), to_native(human.get_max()));
            set_can_source(ControllerMode::PIN, true);
            set_can_source(ControllerMode::POT, true);
        }
        BrakePositionSensor() = delete;

        void setup() {
            set_pin(_pin, INPUT);
            set_source(ControllerMode::PIN);
        }
        
        // is tha brake motor parked?
        bool parked() { return abs(_val_filt.get() - park_in) <= margin_in; }

        float get_park_position() { return park_in; }
        float get_margin() { return margin_in; }
        float get_zeropoint() { return _zeropoint; }
        void set_zeropoint(float zeropoint_arg) { _zeropoint = zeropoint_arg; }
};

class HotrcManager {
  protected:
    bool spike_signbit;
    int32_t spike_cliff, spike_length, this_delta, interpolated_slope, loopindex, previndex;
    int32_t prespike_index = -1;
    int32_t index = 1;  // index is the oldest values are popped from then new incoming values pushed in to the LIFO
    int32_t depth = 9;  // Longest spike the filter can detect
    int32_t filt_history[9];  // Values after filtering.  It didn't accept filt_history[depth] - wtf
    int32_t raw_history[9];  // Copies of the values read (don't need separate buffer, but useful to debug the filter)
  public:
    HotrcManager (int32_t spike_threshold) { spike_cliff = spike_threshold; }
    // Spike filter pushes new hotrc readings into a LIFO ring buffer, replaces any well-defined spikes with values 
    // interpolated from before and after the spike. Also smoothes out abrupt value changes that don't recover later
    int32_t spike_filter (int32_t new_val) {  // pushes next val in, massages any detected spikes, returns filtered past value
        previndex = (depth + index - 1) % depth;  // previndex is where the incoming new value will be stored
        this_delta = new_val - filt_history[previndex];  // Value change since last reading
        if (std::abs(this_delta) > spike_cliff) {  // If new value is a cliff edge (start or end of a spike)
            if (prespike_index == -1) {  // If this cliff edge is the start of a new spike
                prespike_index = previndex;  // save index of last good value just before the cliff
                spike_signbit = signbit (this_delta);  // Save the direction of the cliff
            }
            else if (spike_signbit == signbit (this_delta)) {  // If this cliff edge deepens an in-progress spike (or more likely the change is valid)
                inject_interpolations (previndex, filt_history[previndex]);  // Smoothly grade the values from before the last cliff to previous value
                prespike_index = previndex;  // Consider this cliff edge the start of the spike instead
            }
            else {  // If this cliff edge is a recovery of an in-progress spike
                inject_interpolations (index, new_val);  // Fill in the spiked values with interpolated values
                prespike_index = -1;  // Cancel the current spike
            }
        }
        else if (prespike_index == index) {  // If a current spike lasted thru our whole buffer
            inject_interpolations (previndex, filt_history[previndex]);  // Smoothly grade the whole buffer
            prespike_index = -1;  // Cancel the current spike
        }
        int32_t returnval = filt_history[index];  // Save the incumbent value at current index (oldest value) into buffer
        filt_history[index] = new_val;
        raw_history[index] = new_val;
        index = (index + 1) % depth;  // Update index for next time
        return returnval;  // Return the saved old value
    }
    void inject_interpolations (int32_t endspike_index, int32_t endspike_val) {  // Replaces values between indexes with linear interpolated values
        spike_length = ((depth + endspike_index - prespike_index) % depth) - 1;  // Equal to the spiking values count plus one
        if (!spike_length) return;  // Two cliffs in the same direction on consecutive readings needs no adjustment, also prevents divide by zero 
        interpolated_slope = (endspike_val - filt_history[prespike_index]) / spike_length;
        loopindex = 0;
        while (++loopindex <= spike_length) {
            filt_history[(prespike_index + loopindex) % depth] = filt_history[prespike_index] + loopindex * interpolated_slope;
        }
    }
    int32_t get_next_rawval () { return raw_history[index]; }  // helps to debug the filter from outside the class
};

// Sensor (int32_t arg_pin, bool arg_dir, float arg_val_min, float arg_val_max)  // std::string& eng_name, 
// : Transducer (arg_pin, arg_dir) {
//     set_limits(arg_val_min, arg_val_max);
// }
// Sensor (int32_t arg_pin, bool arg_dir, float arg_val_min, float arg_val_max, float arg_val_cent)  // std::string& eng_name, 
// : Sensor (arg_pin, arg_dir, arg_val_min, arg_val_max) {
//     set_center(arg_val_cent);
// }
// Sensor (int32_t arg_pin) 
// : Device (arg_pin) {}
// float getval (int32_t arg_hist) {  // returns the output value _RAW or _FILT. Use hist to retreive past values 0 (newest) - 9 (oldest)
//     if (arg_hist < 0 || arg_hist >= sizeof(vals)) printf ("Transducer::val(): Max value history is past %d values\n", sizeof(vals)-1);            
//     else return vals[d_val - &vals[0] + sizeof(vals) - arg_hist];
// }

// // Device::Transducer::Sensor::PulseSensor are sensors where the value is based on the measured period between successive pulses (eg tach, speedo)
// class PulseSensor : public Sensor {
//   protected:
//     Timer PulseTimer;  // OK to not be volatile?
//   public:
//     volatile int32_t delta_us;
//     int32_t delta_impossible_us, stop_timeout_us;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers

//     PulseSensor(int32_t arg_pin, int32_t arg_impossible_us, int32_t arg_stop_timeout_us) 
//     : Device(arg_pin) {
//         delta_us = 0;
//         delta_impossible_us = arg_impossible_us;
//         stop_timeout_us = arg_stop_timeout_us;
//         val_source = _LIVE;
//         // pinMode(pin, INPUT_PULLUP);
//         // attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
//     }
//     void isr(void) {  //  A better approach would be to read, reset, and restart a hardware interval timer in the isr.  Better still would be to regularly read a hardware binary counter clocked by the pin - no isr.
//         int32_t temp_us = PulseTimer.elapsed();
//         if (temp_us > delta_impossible_us) {
//             delta_us = temp_us;    
//             PulseTimer.reset();
//         }
//     }
//     void calc() {
//         if (val_source != _TOUCH && val_source != _POT) {
//             float val_temp;
//             if (PulseTimer.elapsed() < stop_timeout_us) {
//                 if (delta_us <= 0) printf ("Warning: PulseSensor::calc sees delta_us <= 0\n");
//                 else val_temp = conversion_factor/delta_us;
//             }
//             else val_temp = 0;     
//             if (filt_lp_spike(val_temp)) {
//                 if (val_temp == 0) assign_val(0.0);
//                 else assign_val ( filt_ema(val_temp, *d_val, ema_alpha) );
//             }
//         }
//     }
// };
// // Device::Transducer::Sensor::InPWM are servo-pwm standard signals being read in, with variable pulsewidth
// class InPWM : public Sensor {
//   protected:
//     Timer PulseTimer;  // OK to not be volatile?
//   public:
//     InPWM(int32_t arg_pin, bool arg_dir, float arg_val_min, float arg_val_max, float arg_val_cent)
//     : Sensor(arg_pin, arg_dir, arg_val_min, arg_val_max, arg_val_cent)  {}

// };
// // Device::Transducer::Sensor::InPWM::InPWMToggle are servo-pwm standard signals being read in, but only valid values are pulse_min (0) and pulse_max (1) (eg hotrc ch3-4)
// class InPWMToggle : public InPWM {
//   protected:
//     Timer PulseTimer;  // OK to not be volatile?
//   public:
//     InPWMToggle(int32_t arg_pin, bool arg_dir, float arg_val_min, float arg_val_max, float arg_val_cent)
//     : Sensor(arg_pin, arg_dir, arg_val_min, arg_val_max, arg_val_cent)  {}

// };

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
        _servo.attach(this->_pin, this->min_native->get(), this->max_native->get());
    }

    void set_human_limits(Param<HUMAN_T> &minParam, Param<HUMAN_T> &maxParam) {
        this->set_human_limits(minParam, maxParam);
        _servo.attach(this->_pin, this->min_native->get(), this->max_native->get());
    }

  public:
    ServoPWM(uint8_t pin) : Transducer<NATIVE_T, HUMAN_T>(pin) {
        _servo.attach(this->_pin);
    }
    ServoPWM() = delete;
    void setup() {
        set_pin(this->_pin, OUTPUT);
    }
    void write() {
        _servo.writeMicroseconds((int32_t)this->native.get());  // Write result to servo interface
    }
};

// JagMotor is a class specifically for the brake and steering motors. The jaguar stops the motor when receiving 1500 us pulse,
//    and varies the speed in one direction if pulse is 1500 to (max~2500) us, the other direction if pulse is 1500 to (min~500) us.
//    Effectively the difference is these have a center value.
// class JagMotor : public ServoPWM {
//   public:
//     JagMotor (int32_t arg_pin) : ServoPWM(arg_pin) {}
//     JagMotor() = delete;
// };

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
        set_can_source(ControllerMode::PIN, true);
        _source = ControllerMode::PIN;
    }
    void set_val(bool arg_val) {
        if (_source != ControllerMode::PIN) {
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

// NOTE: what functions does a controller need?
// class Controller {};

// class HotRc : public Controller {
//   protected:
//     InPWM horz, vert;
//     InPWMToggle ch3, ch4;
//   public:
//     HotRC (int32_t arg_horz_pin, int32_t arg_vert_pin, int32_t arg_ch3_pin, int32_t arg_ch4_pin) {
//         InPWM horz(arg_horz_pin, )
//     }
// };

// class Joystick : public Controller {
//   protected:
//     AnalogSensor Horz, Vert;  
// };

// // Device::Transducer is a base class for any system devices that convert real_world <--> signals in either direction
// class Transducer : virtual public Device {
//   protected:
//     bool centermode, saturated;
//     float vhist[5];  // vals[] is some past values, [0] beling most recent. 
//     void hist_init (float arg_val) {
//         for (int x = 0; x < sizeof(vals); x++) vhist[x] = arg_val;
//     }
//     void new_val (float new_val) {
//         for (int x = sizeof(vhist)-1; x >= 1; x--) vhist[x] = vhist[x-1];
//         vhist[0] = *val;
//         *val = new_val;
//     }
//   public:
//     enum relativity { ABSOLUTE, RELATIVE };
//     float val_cent, val_margin;
//     Transducer (int32_t arg_pin) 
//     : Device (arg_pin) { // std::string& eng_name, 
//         centermode = ABSOLUTE;
//         val_margin = 0.0;
//         val_cent = 
//         // set_limits(arg_val_min, arg_val_max);
//         hist_init(0);
//     }
//     void set_center (float arg_val_cent) {
//         if (arg_val_cent <= min_val || arg_val_cent >= max_val) {
//             printf ("Transducer::set_limits(): Centerpoint must fall within min/max limits\n");
//             return;
//         }
//         else {
//             val_cent = arg_val_cent;
//             centermode = RELATIVE;
//         }
//     }
//     void set (float val) {
//     }
// };
// // Transducer (int32_t arg_pin, bool arg_dir, float arg_val_min, float arg_val_max) { // std::string& eng_name, 
// void add_val (float arg_add_val) {  // 
//     assign_val(*d_val + arg_add_val);
// }

// class TuneEditor {
// };
// class Simulator {
// };
// class Settings {
// };

// NOTE: if devices.h gets to be too long, we can (and maybe just should) move this to a separate file, it's not really a device...

// This enum class represent the components which can be simulated (SimOption). It's a uint8_t type under the covers, so it can be used as an index
typedef uint8_t opt_t;
enum class SimOption : opt_t { none=0, pressure, brkpos, tach, airflow, mapsens, speedo, battery, coolant, joy, basicsw, cruisesw, syspower, starter, ignition };

// Simulator manages the ControllerMode handling logic for all simulatable components. Currently, components can recieve simulated input from either the touchscreen, or from
// NOTE: this class is designed to be backwards-compatible with existing code, which does everything with global booleans. if/when we switch all our Devices to use ControllerModes,
//       we can simplify the logic here a lot.
class Simulator {
    private:
        // NOTE: if we only simulated devices, we could keep track of simulability in the Device class. We could keep the default ControllerMode in Device the same way.
        // 3-tuple for a given component, keeping track of simulability status (whether this component is allowed to be simulated), an associated Device (a pointer to the actual component),
        // and a default ControllerMode (the mode we would like the component to switch back to when it stops being simulated)
        typedef std::tuple<bool, Device*, ControllerMode> simulable_t;
        std::map<SimOption, simulable_t> _devices; // a collection of simulatable components
        bool _enabled = false; // keep track of whether the simulator is running or not
        SimOption _pot_overload; // keep track of which component is getting info from the pot

        Potentiometer& _pot;
    public:
        static constexpr SimOption initial_pot_overload = SimOption::speedo;
        
        // initial simulation settings
        static constexpr bool initial_sim_joy = false;
        static constexpr bool initial_sim_tach = true;
        static constexpr bool initial_sim_speedo = true;
        static constexpr bool initial_sim_brkpos = true;
        static constexpr bool initial_sim_basicsw = true;
        static constexpr bool initial_sim_cruisesw = false;
        static constexpr bool initial_sim_pressure = true;
        static constexpr bool initial_sim_syspower = true;
        static constexpr bool initial_sim_starter = true;
        static constexpr bool initial_sim_ignition = true;
        static constexpr bool initial_sim_airflow = false;
        static constexpr bool initial_sim_mapsens = true;
        static constexpr bool initial_sim_battery = true;
        static constexpr bool initial_sim_coolant = true;

        Simulator(Potentiometer& pot_arg, SimOption overload_arg=SimOption::none) : _pot(pot_arg) {
            // set initial simulatability status for all components
            set_can_simulate(SimOption::joy, initial_sim_joy);
            set_can_simulate(SimOption::tach, initial_sim_tach);
            set_can_simulate(SimOption::speedo, initial_sim_speedo);
            set_can_simulate(SimOption::brkpos, initial_sim_brkpos);
            set_can_simulate(SimOption::basicsw, initial_sim_basicsw);
            set_can_simulate(SimOption::cruisesw, initial_sim_cruisesw);
            set_can_simulate(SimOption::pressure, initial_sim_pressure);
            set_can_simulate(SimOption::syspower, initial_sim_syspower);
            set_can_simulate(SimOption::starter, initial_sim_starter);
            set_can_simulate(SimOption::ignition, initial_sim_ignition);
            set_can_simulate(SimOption::airflow, initial_sim_airflow);
            set_can_simulate(SimOption::battery, initial_sim_battery);
            set_can_simulate(SimOption::coolant, initial_sim_coolant);
            set_pot_overload(overload_arg); // set initial pot overload
        }

        // turn on the simulator. all components which are set to be simulated will switch to simulated input
        void enable() {
            if (!_enabled) {
                for ( auto &kv : _devices ) { // go through all our components
                    bool can_sim = std::get<0>(kv.second); // check simulatability status
                    if (can_sim && _pot_overload != kv.first) { // if component has simulation enabled and it's not being overloaded by the pot...
                        Device *d = std::get<1>(kv.second);
                        // NOTE: the nullptr checks here and below exist so that we can work with boolean components as well as Devices, for backwards compatability
                        if (d != nullptr) {
                           d->set_source(ControllerMode::TOUCH); // ...then set component to take input from the touchscreen
                        }
                    }
                }
                _enabled = true;
            }
        }

        // turn off the simulator. all devices will be set to their default input (if they are not being overloaded by the pot)
        void disable() {
            if (_enabled) {
                for ( auto &kv : _devices ) { // go through all our components
                    bool can_sim = std::get<0>(kv.second);
                    if (can_sim && _pot_overload != kv.first) { // if component has simulation enabled and it's not being overloaded by the pot...
                        Device *d = std::get<1>(kv.second);
                        if (d != nullptr) {
                            ControllerMode default_mode = std::get<2>(kv.second);
                            d->set_source(default_mode); // ...then it's in simulation mode and needs to be set back to it's default mode, since we are no longer simulating
                        }
                    }
                }
                _enabled = false;
            }
        }

        // toggle the simulator on and off
        void toggle() {
            return _enabled ? disable() : enable();
        }

        // check if a componenet is currently being simulated (by either the touchscreen or the pot)
        bool simulating(SimOption option) {
            return can_simulate(option) && (_enabled || _pot_overload == option);
        }

        // associate a Device and a given fall-back ControllerMode with a SimOption
        void register_device(SimOption option, Device &d, ControllerMode default_mode) {
            bool can_sim = false; // by default, disable simulation for this component
            auto kv = _devices.find(option); // look for the component
            if (kv != _devices.end()) {
                can_sim = std::get<0>(kv->second); // if an entry for the component already existed, preserve its simulatability status
            }
            d.set_pot_input(_pot);
            _devices[option] = simulable_t(can_sim, &d, default_mode); // store info for this component
        }

        // check if a component can be simulated (by either the touchscreen or the pot)
        bool can_simulate(SimOption option) {
            auto kv = _devices.find(option); // look for the component
            if (kv != _devices.end()) {
                return std::get<0>(kv->second); // if it exists, check the simulability status
            }
            return false; // couldn't find component, so there's no way we can simulate it
        }

        // set simulatability status for a component
        void set_can_simulate(SimOption option, bool can_sim) {
            auto kv = _devices.find(option); // look for component
            if (kv != _devices.end()) { // if an entry for this component already exists, check if the new simulatability status is different from the old
                bool old_can_sim = std::get<0>(kv->second);
                if (can_sim != old_can_sim) { // if the simulation status has changed, we need to update the input source for the component
                    ControllerMode default_mode = ControllerMode::UNDEF;
                    Device *d = std::get<1>(kv->second);
                    if (d != nullptr) { // if there is no associated Device with this component then input handling is done in the main code
                        default_mode = std::get<2>(kv->second); // preserve the stored default controller mode
                        if (can_sim) { // if we just enabled simulatability...
                            if (option == _pot_overload) { // ...and the pot is supposed to overload this component...
                                    d->set_source(ControllerMode::POT); // ...then set the input source for the associated Device to read from the pot
                            } else if (_enabled) { // ...and the pot isn't overloading this component, but the simulator is running...
                                    d->set_source(ControllerMode::TOUCH); // ...then set the input source for the associated Device to read from the touchscreen
                            }
                        } else {
                            d->set_source(default_mode); // we disabled simulation for this component, set it back to its default input source
                        }
                    }
                    kv->second = simulable_t(can_sim, d, default_mode); // update the entry with the new simulatability status
                }
            } else {
                _devices[option] = simulable_t(can_sim, nullptr, ControllerMode::UNDEF); // add a new entry with the simulatability status for this component
            }
        }

        // set the component to be overridden by the pot (the pot can only override one component at a time)
        void set_pot_overload(SimOption option) {
            if (option != _pot_overload) { // if we're overloading a different component, we need to reset the input source for the old one
                auto kv = _devices.find(_pot_overload);
                if (kv != _devices.end()) {
                    Device *d = std::get<1>(kv->second);
                    if (d != nullptr) { // if we were overloading a component with an associated Device...
                        bool can_sim = std::get<0>(kv->second);
                        if (_enabled && can_sim) { // ...and the simulator is on, and we're able to be simulated...
                            d->set_source(ControllerMode::TOUCH); // ...then set the input source to the touchscreen
                        } else { // ...and either the simulator is off or we aren't allowing simualtion for this component...
                            ControllerMode default_mode = std::get<2>(kv->second);
                            d->set_source(default_mode); // then set the input source for the component to its default
                        }
                    } // ...else this component is a boolean, and input source handling is done elsewhere
                }
                kv = _devices.find(option); // we need to set the input source of the newly-overridden component
                if (kv != _devices.end()) {
                    Device *d = std::get<1>(kv->second);
                    if (d != nullptr ) { // if  we're overloading a component with an associated device, we need to change the input source to the pot
                        if (d->can_source(ControllerMode::POT)) { // ...and we're allowed to overload this component...
                            bool can_sim = std::get<0>(kv->second);
                            if (can_sim) { // if we allow simualation for this componenent...
                                d->set_source(ControllerMode::POT); // ...then set its input source to the pot
                            }
                        } else {
                            printf("invalid pot overload selected: %d/n", option);
                        }
                    }
                }
                _pot_overload = option;
            }
        }

        bool get_enabled() { return _enabled; }
        SimOption get_pot_overload() { return _pot_overload; }
};

#endif  // CLASSES_H