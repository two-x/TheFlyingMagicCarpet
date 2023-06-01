#ifndef DEVICES_H
#define DEVICES_H
#include <iostream>
#include <string>
#include <Servo.h>  // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include "Arduino.h"
#include "globals.h"

// class Variable {  // A wrapper for all veriables of consequence in our code.
//     public:
//         Variable(int32_t arg1) { int32_t value = arg1; }
//         Variable(double arg1) { double value = arg1; }
//         Variable(bool arg1) { bool value = arg1; }
// }

// Param is a value affecting control, which holds a double value, and can be displayed
class Param {
  protected:
    double effect_max_val, effect_min_val, last_val, last2_val; //, effect_cent_val;  // Values corresponding to a max/min real world effect (might be numerically reversed)
    bool constrain_it (double* arg_value, double arg_min, double arg_max) {  // This should be called "constrain" not "constrain_it"
        if (*arg_value < arg_min) *arg_value = arg_min;
        else if (*arg_value > arg_max) *arg_value = arg_max;
        else return true;  // No constraint was necessary
        return false;  // If constraint was necessary
    }
  public:
    #define _FWD 1
    #define _REV -1
    bool can_sim, can_tune;
    int32_t dir;  // For the case a lower val causes a higher real-life effect, 
    char disp_name[9], disp_units[5];
    double* val; double* min_val; double* max_val;  // Reference to numerical max/min values
    Param (double* val, std::string arg_name, std::string arg_units, double* min_val, double* max_val) {
        set_limits (min_val, max_val);
        constrain_it (val, *min_val, *max_val);
        strcpy(disp_name, arg_name.c_str());
        strcpy(disp_units, arg_units.c_str());
    }
    Param (double* val, std::string arg_name, std::string arg_units, double effect_min_val, double effect_max_val) {
        set_limits (effect_min_val, effect_max_val);
        Param (val, arg_name, arg_units, &effect_min_val, &effect_max_val);
    }
    void set_limits (double* min_val, double* arg_max_val) {
        dir = (*min_val <= *arg_max_val) * 2 - 1;
        max_val = (dir == _FWD) ? arg_max_val : min_val;
        if (dir == _REV) min_val = max_val;
    }
    void set_limits (double effect_min_val, double effect_max_val) {
        min_val = &effect_min_val;
        max_val = &effect_max_val;
        set_limits (min_val, max_val);
    }
};
//: Param (val, arg_name, arg_units, &effect_min_val, &effect_max_val) {}

// class Setting : public Variable {
// }

// Timer
// For controlling event timing. Communicates in signed int32 values but uses uint32 under the hood to
// last 72 minutes between overflows. However you can only time for 36 minutes max.
// Note: experimenting with use of micros() instead of mycros(), check for errors on overflow and possibly revert
class Timer {
  volatile uint32_t start_us;  // start time in us
    int32_t timeout_us = 0;  // in us
  public:
    // std::string eng_name;
    Timer(void) { // }: eng_name("timer") {
        start_us = micros();
    };
    Timer(int32_t arg1) { // }: eng_name("timer") {
        set(arg1);
    };
    // Timer(const std::string& eng_name, int32_t arg1) {
    //     set(arg1);
    // };
    void reset()  { start_us = micros(); }
    bool expired()  { return (abs((int32_t)(micros() - start_us)) > timeout_us); }
    int32_t elapsed()  { return abs((int32_t)(micros() - start_us)); }
    int32_t timeout()  { return (int32_t)timeout_us; }
    void set(int32_t arg1)  {
        timeout_us = arg1;
        start_us = micros();
    }
    int32_t remain()  { 
        uint32_t temp = abs((int32_t)(micros() - start_us));
        return (timeout_us - (int32_t)temp);
    }
};

// Device is a base class for any connected system device or signal
class Device {
  protected:
    bool direction, can_sim_touch, can_sim_pot;
    int32_t pin, val_source;
  public:
    #define _PIN 0  // source is from reading an input pin
    #define _TOUCH 1  // source is from touch ui simulator
    #define _POT 2  // source is simulated using pot
    #define _LIVE 3   // source is determined by control algorithms
    Device (int32_t arg_pin) {  // std::string& eng_name, 
        can_sim_touch = false;
        val_source = _PIN;
        set_pin(arg_pin);
    }
    Device (int32_t arg_pin, bool arg_dir) 
    : Device(arg_pin) {  // std::string& eng_name, 
        set_dir(arg_dir);
    }
    void set_val_source (int32_t arg_source) {
        if (arg_source == _POT && can_sim_pot) val_source = _POT;
        else if (arg_source == _TOUCH && can_sim_touch) val_source = _TOUCH;
        else arg_source = _PIN;
    }
    void set_dir ( bool arg_dir) { direction = arg_dir; }
    void set_pin ( bool arg_pin) { pin = arg_pin; }
    int32_t get_vsource () { return val_source; }
};

// Device::Toggle is a base class for system signals or devices having a boolean value
class Toggle : virtual public Device {
  public:
    bool val, val_last, can_sim;
    int32_t pin, val_source;
    Toggle(int32_t arg_pin) {  // std::string& eng_name, 
        pin = arg_pin;
        can_sim = true;
    }
};

// Device::Toggle::InToggle is system signals or devices having a boolean value that serve as inputs (eg basicsw, cruisesw)
class InToggle : virtual public Toggle {  // Toggle
  public:
    InToggle(int32_t arg_pin)   // std::string& eng_name, 
    : Toggle(arg_pin) {
        val_source = _PIN;
    }
    void set_val(bool arg_val) {
        if (val_source != _PIN) {
            val_last = val;
            val = arg_val;
        }
    }
    void read() {
        val_last = val;
        val = digitalRead(pin);
    }
};

// Device::Toggle::OutToggle is system signals or devices having a boolean value that serve as outputs (eg ignition, leds, etc.)
class OutToggle : virtual public Toggle {  // Toggle
  public:
    OutToggle(int32_t arg_pin)  // std::string& eng_name, 
    : Toggle(arg_pin) {
        val_source = _LIVE;
    }
    void set_val(bool arg_val) {
        val_last = val;
        val = arg_val;
        if (val_source == _LIVE) write();
    }
    void write() {
        digitalWrite(pin, val);
    }
};

// Device::Transducer is a base class for any system devices that convert real_world <--> signals in either direction
class Transducer : virtual public Device {
  protected:
    double val_min, val_max, val_center, val_margin, ema_alpha;
    double vals[10]; double* d_val;  // vals[] is ring buffer of past values. d_val is pointer to current value (in ring buffer)
    bool centermode, saturated;
    void hist_init (double arg_val) {
        for (int x = 0; x < sizeof(vals); x++) vals[x] = arg_val;
        d_val = &vals[0];
    }
  public:
    #define ABSOLUTE 0  // assign to outCenterMode if pid output just spans a range, rather than deviating from a centerpoint 
    #define RELATIVE 1  // assign to outCenterMode if pid output deviates from a centerpoint 
    #define _RAW 0
    #define _FILT 1
    // Transducer (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max) { // std::string& eng_name, 
    Transducer (int32_t arg_pin, bool arg_dir) 
    : Device (arg_pin, arg_dir) { // std::string& eng_name, 
        centermode = ABSOLUTE;
        // set_limits(arg_val_min, arg_val_max);
        hist_init(0);
    }
    Transducer (int32_t arg_pin) {
        bool arg_dir = _FWD;
        Transducer(arg_pin, arg_dir);
    }
    // Transducer (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent) {  // std::string& eng_name, 
    //     Transducer (arg_pin, arg_dir, arg_val_min, arg_val_max);  // eng_name, 
    //     set_center (arg_val_cent);
    // }
    void assign_val (double arg_new_val) {  // 
        d_val++;
        if (d_val >= &vals[0] + sizeof(vals)) d_val -= sizeof(vals);
        *d_val = arg_new_val;
        saturated = !constrainit(d_val);
    }
    void add_val (double arg_add_val) {  // 
        assign_val(*d_val + arg_add_val);
    }
    void set_center (double arg_val_cent) {
        if (arg_val_cent <= val_min || arg_val_cent >= val_max) {
            printf ("Transducer::set_limits(): Centerpoint must fall within min/max limits\n");
            return;
        }
        else {
            val_center = arg_val_cent;
            centermode = RELATIVE;
        }
    }
    void set_limits (double arg_val_min, double arg_val_max) {
        if (arg_val_min >= arg_val_max) {
            printf ("Transducer::set_limits(): min must be less than max\n");
            return;
        }
        val_min = arg_val_min;
        val_max = arg_val_max;
    }
    void set_limits (double arg_val_min, double arg_val_max, double arg_val_cent) {
        set_limits(arg_val_min, arg_val_max);
        set_center(arg_val_cent);
    }
};

// Device::Transducer::Sensor is a base class for control system sensors, ie anything that measures real world data or electrical signals 
class Sensor : virtual public Transducer {
  protected:
    bool lp_spike_filtering;
    double conversion_factor, lp_thresh, spike_thresh, raw_val;  // Multiplier to convert values from direct measured units (adc, us) into useful units (rpm, mmph, etc.)
  public:
    Sensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)  // std::string& eng_name, 
    : Transducer (arg_pin, arg_dir) {
        conversion_factor = 1.0;
        lp_spike_filtering = false;
        raw_val = 0;
        set_limits(arg_val_min, arg_val_max);
    }
    Sensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)  // std::string& eng_name, 
    : Sensor (arg_pin, arg_dir, arg_val_min, arg_val_max) {
        set_center(arg_val_cent);
    }
    Sensor (int32_t arg_pin) 
    : Device (arg_pin) {}
    // Sensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)  // std::string& eng_name, 
    // : Transducer (arg_pin, arg_dir) {
    //     conversion_factor = 1.0;
    //     lp_spike_filtering = false;
    //     raw_val = 0;
    //     set_limits(arg_val_min, arg_val_max, arg_val_cent);
    // }
    double getval (int32_t arg_hist) {  // returns the output value _RAW or _FILT. Use hist to retreive past values 0 (newest) - 9 (oldest)
        if (arg_hist < 0 || arg_hist >= sizeof(vals)) printf ("Transducer::val(): Max value history is past %d values\n", sizeof(vals)-1);            
        else return vals[d_val - &vals[0] + sizeof(vals) - arg_hist];
    }
    double getval (void) { return *d_val; }
    double getraw (void) { return raw_val; }
    double filter (void) {
        return ema_alpha * raw_val + (1-ema_alpha) * (*d_val);
    }

    bool new_val (double arg_val) {  // return true if given new value need not be rejected due to spiked value
        raw_val = arg_val;
        if (!lp_spike_filtering) return true;
        else return ( abs( raw_val - getval(2) ) > lp_thresh || raw_val - getval(1) > spike_thresh );  // Should this be raw not filt?
    }
    void set_conversion_factor(double arg_factor) {
        conversion_factor = arg_factor;
    }
    void set_ema_alpha (double arg_alpha) {
        ema_alpha = arg_alpha;
    }
    void set_lp_spike_thresh (double arg_lp_thresh, double arg_spike_thresh) {
        lp_spike_filtering = true;
        lp_thresh = arg_lp_thresh;  // max delta acceptable over three consecutive readings
        spike_thresh = arg_spike_thresh;  //min val delta between two readings considered a spike to ignore
    }
};

// Device::Transducer::Sensor::AnalogSensor are sensors where the value is based on an ADC reading (eg brake pressure, brake actuator position, pot)
class AnalogSensor : virtual public Sensor {
  public:
    AnalogSensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)  // std::string& eng_name, 
    : Sensor (arg_pin, arg_dir, arg_val_min, arg_val_max) {}

    void read() {
        int32_t read_val = analogRead(pin);
        // filter, etc.
        assign_val((double)read_val); 
    }
};

// Device::Transducer::Sensor::PulseSensor are sensors where the value is based on the measured period between successive pulses (eg tach, speedo)
class PulseSensor : public Sensor {
  protected:
    Timer PulseTimer;  // OK to not be volatile?
  public:
    volatile int32_t delta_us;
    int32_t delta_impossible_us, stop_timeout_us;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers

    PulseSensor(int32_t arg_pin, int32_t arg_impossible_us, int32_t arg_stop_timeout_us) 
    : Device(arg_pin) {
        delta_us = 0;
        delta_impossible_us = arg_impossible_us;
        stop_timeout_us = arg_stop_timeout_us;
        val_source = _LIVE;
        // pinMode(pin, INPUT_PULLUP);
        // attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
    }
    void isr(void) {  //  A better approach would be to read, reset, and restart a hardware interval timer in the isr.  Better still would be to regularly read a hardware binary counter clocked by the pin - no isr.
        int32_t temp_us = PulseTimer.elapsed();
        if (temp_us > delta_impossible_us) {
            delta_us = temp_us;    
            PulseTimer.reset();
        }
    }
    void calc() {
        if (val_source != _TOUCH && val_source != _POT) {
            double val_temp;
            if (PulseTimer.elapsed() < stop_timeout_us) {
                if (delta_us <= 0) printf ("Warning: PulseSensor::calc sees delta_us <= 0\n");
                else val_temp = conversion_factor/delta_us;
            }
            else val_temp = 0;     
            if (filt_lp_spike(val_temp)) {
                if (val_temp == 0) assign_val(0.0);
                else assign_val ( filt_ema(val_temp, *d_val, ema_alpha) );
            }
        }
    }
};
// Device::Transducer::Sensor::InPWM are servo-pwm standard signals being read in, with variable pulsewidth
class InPWM : public Sensor {
  protected:
    Timer PulseTimer;  // OK to not be volatile?
  public:
    InPWM(int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)
    : Sensor(arg_pin, arg_dir, arg_val_min, arg_val_max, arg_val_cent)  {}

};
// Device::Transducer::Sensor::InPWM::InPWMToggle are servo-pwm standard signals being read in, but only valid values are pulse_min (0) and pulse_max (1) (eg hotrc ch3-4)
class InPWMToggle : public InPWM {
  protected:
    Timer PulseTimer;  // OK to not be volatile?
  public:
    InPWMToggle(int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)
    : Sensor(arg_pin, arg_dir, arg_val_min, arg_val_max, arg_val_cent)  {}

};
// Device::Transducer::ServoPWM is a base class for control system actuators, ie anything that turns signals into real world physical change
class ServoPWM : virtual public Transducer {
  protected:
    static Servo servo;
  public:
    ServoPWM (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max) {  // std::string& eng_name, 
        Transducer (arg_pin, arg_dir);
        servo.attach(pin);
        set_limits(arg_val_min, arg_val_max);
    }
    void write() {
        servo.writeMicroseconds((int32_t)*d_val);  // Write result to jaguar servo interface
    }
};

// Device::Transducer::ServoPWM::JagMotor is a class specifically for the brake linear actuator motor
class JagMotor : public ServoPWM {
  protected:
    static Servo servo;
  public:
    JagMotor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)  // std::string& eng_name, 
    : ServoPWM(arg_pin, arg_dir, arg_val_min, arg_val_max) {}
};

class Controller {};

class HotRc : public Controller {
  protected:
    InPWM horz, vert;
    InPWMToggle ch3, ch4;
  public:
    HotRC (int32_t arg_horz_pin, int32_t arg_vert_pin, int32_t arg_ch3_pin, int32_t arg_ch4_pin) {
        InPWM horz(arg_horz_pin, )
    }
};

class Joystick : public Controller {
  protected:
    AnalogSensor Horz, Vert;  
};

class ControlLoop {
};

class TuneEditor {
};
class Simulator {
};
class Settings {
};

#endif  // DEVICES_H
