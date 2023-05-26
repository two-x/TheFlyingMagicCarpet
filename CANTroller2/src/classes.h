#ifndef CLASSES_H
#define CLASSES_H

// #include <stdio.h>
#include <iostream>
#include <string>
#include "Arduino.h"
#include "globals.h"

// using namespace std;

// class Variable {  // A wrapper for all veriables of consequence in our code.
//     public:
//         Variable(int32_t arg1) { int32_t value = arg1; }
//         Variable(double arg1) { double value = arg1; }
//         Variable(bool arg1) { bool value = arg1; }
// }
// class Setting : public Variable {
// }
// class Parameter : public Variable {
// }

// Timer
// For controlling event timing. Communicates in signed int32 values but uses uint32 under the hood to
// last 72 minutes between overflows. However you can only time for 36 minutes max.
// Note: experimenting with use of micros() instead of mycros(), check for errors on overflow and possibly revert
class Timer {
  volatile uint32_t start_us;  // start time in us
    int32_t timeout_us = 0;  // in us
  public:
    std::string eng_name;
    Timer(void) : eng_name("timer") {
        start_us = micros();
    };
    Timer(int32_t arg1) : eng_name("timer") {
        set(arg1);
    };
    Timer(const std::string& eng_name, int32_t arg1) {
        set(arg1);
    };
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
        // return (timeout_us - (int32_t)temp);
    }
};

// Param is a value affecting control, which holds a double value, and can be displayed
class Param {
  protected:
    std::string eng_name, disp_name, disp_units;
  public:
    double val;
    Param (std::string& eng_name, double arg_val) {
        val = arg_val;
        disp_name = "";
        disp_units = "";
    }
    Param (std::string& eng_name, double arg_val, std::string& disp_name, std::string& disp_units) { val = arg_val; }
    void set_disp_strings (std::string& disp_name, std::string& disp_units) {}
    std::string& eng_name () { return eng_name; }
    std::string& disp_name () { return disp_name; }
    std::string& disp_units () { return disp_units; }
};

// Device is a base class for any connected system device or signal
class Device {
  protected:
    bool direction, can_sim_touch, can_sim_pot;
    int32_t pin, val_source;
  public:
    #define PIN 0  // source is from reading an input pin
    #define TOUCH 1  // source is from touch ui simulator
    #define POT 2  // source is simulated using pot
    #define LIVE 3   // source is determined by control algorithms
    Device (std::string& eng_name, bool arg_dir) {
        can_sim_touch = false;
        val_source = PIN;
        direction = arg_dir;
    }
    void set_val_source (int32_t arg_source) {
        if (arg_source == POT && can_sim_pot) val_source = POT;
        else if (arg_source == TOUCH && can_sim_touch) val_source = TOUCH;
        else arg_source = PIN;
    }
    int32_t val_source () { return val_source; }
};

// Device::Transducer is a base class for any system devices that convert real_world <--> signals in either direction
class Transducer : public Device {
  protected:
    double val_min, val_max, val_center, val_margin, ema_alpha;
    double vals[10]; double* d_val;  // vals[] is ring buffer of past values. d_val is pointer to current value (in ring buffer)
    bool centermode, saturated;
    void hist_init (double arg_val) {
        for (int x = 0; x < sizeof(vals); x++) vals[x] = arg_val;
        d_val = &vals[0];
    }
    bool con_strain (double* arg_value) {  // This should be called "constrain" not "con_strain"
        if (*arg_value < val_min) *arg_value = val_min;
        else if (*arg_value > val_max) *arg_value = val_max;
        else return true;  // No constraint was necessary
        return false;  // If constraint was necessary
    }
  public:
    #define ABSOLUTE 0  // assign to outCenterMode if pid output just spans a range, rather than deviating from a centerpoint 
    #define RELATIVE 1  // assign to outCenterMode if pid output deviates from a centerpoint 
    #define RAW 0
    #define FILT 1
    Transducer (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)
    : Device (eng_name, arg_dir) {
        pin = arg_pin;
        centermode = ABSOLUTE;
        set_limits(arg_val_min, arg_val_max);
        hist_init(0);
    }
    Transducer (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent) {
        Transducer (eng_name, arg_pin, arg_dir, arg_val_min, arg_val_max);
        set_center (arg_val_cent);
    }
    void assign_val (double arg_new_val) {  // 
        d_val++;
        if (d_val >= &vals[0] + sizeof(vals)) d_val -= sizeof(vals);
        *d_val = arg_new_val;
        saturated = !con_strain(d_val);
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

// Device::Transducer::Actuator is a base class for control system actuators, ie anything that turns signals into real world physical change
class Actuator : public Transducer {
  protected:
    static Servo servo;
  public:
    Actuator (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)
    : Transducer (eng_name, arg_pin, arg_dir, arg_val_min, arg_val_max) {
        servo.attach(pin);
    }
    void write() {
        servo.writeMicroseconds((int32_t)*d_val);  // Write result to jaguar servo interface
    }
};

// Device::Transducer::Sensor is a base class for control system sensors, ie anything that measures real world data or electrical signals 
class Sensor : public Transducer {
  protected:
    bool lp_spike_filtering;
    double conversion_factor, lp_thresh, spike_thresh;  // Multiplier to convert values from direct measured units (adc, us) into useful units (rpm, mmph, etc.)
  public:
    Sensor (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)
    : Transducer (eng_name, arg_pin, arg_dir, arg_val_min, arg_val_max) {
        conversion_factor = 1.0;
        lp_spike_filtering = false;
    }
    Sensor (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)
    : Transducer (eng_name, arg_pin, arg_dir, arg_val_min, arg_val_max, arg_val_cent) {
        conversion_factor = 1.0;
        lp_spike_filtering = false;
    }

    double val (bool arg_filt, int32_t arg_hist) {  // returns the output value RAW or FILT. Use hist to retreive past values 0 (newest) - 9 (oldest)
        if (arg_hist < 0 || arg_hist >= sizeof(vals)) {
            printf ("Transducer::val(): Max value history is past %d values\n", sizeof(vals)-1);
            return;
        }            
        return vals[ (int32_t)raw + sizeof(vals) * (arg_filt + 1) - arg_hist ];
    }
    double raw () { val(RAW, 0); }
    double filt () { val(FILT, 0); }
    double filt_ema (int32_t raw_value, double filt_value, double alpha) {
        return (int32_t)((alpha*(double)raw_value) + ((1-alpha)*filt_value));
    }
    bool filt_lp_spike (double arg_val) {  // return true if given new value need not be rejected due to spiked value
        if (!lp_spike_filtering) return true;
        else return ( abs( arg_val - val(FILT, 2) ) > lp_thresh || arg_val - val(FILT, 1) > spike_thresh );  // Should this be raw not filt?
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

// Device::Transducer::Actuator::BrakeMotor is a class specifically for the brake linear actuator motor
class BrakeMotor : public Actuator {
  protected:
    static Servo servo;
  public:
    BrakeMotor (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)
    : Actuator(eng_name, arg_pin, arg_dir, arg_val_min, arg_val_max) {}
};

// Device::Transducer::Sensor::AnalogSensor are those where the value is based on an ADC reading (eg brake pressure, brake actuator position, pot)
class AnalogSensor : public Sensor {
  public:
    void read() {
        int32_t read_val = analogRead(pin);
        // filter, etc.
        assign_val((double)read_val); 
    }
};

// Device::Transducer::Sensor::PulseSensor are those where the value is based on the measured period between successive pulses (eg tach, speedo)
class PulseSensor : public Sensor {
  protected:
    Timer PulseTimer;  // OK to not be volatile?
  public:
    int32_t pin;
    volatile int32_t delta_us;
    int32_t delta_impossible_us, stop_timeout_us;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers

    PulseSensor(int32_t arg_pin, int32_t arg_impossible_us, int32_t arg_stop_timeout_us) {
        delta_us = 0;
        delta_impossible_us = arg_impossible_us;
        stop_timeout_us = arg_stop_timeout_us;
        
        pin = arg_pin;
        val_source = LIVE;
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
        if (val_source != TOUCH && val_source != POT) {
            double val_temp;
            if (PulseTimer.elapsed() < stop_timeout_us) {
                if (delta_us <= 0) printf ("Warning: PulseSensor::calc sees delta_us <= 0\n");
                else val_temp = conversion_factor/delta_us;
            }
            else val_temp = 0;     
            
            // carspeed_mmph = (int32_t)(179757270/(double)speedo_delta_us); // Update car speed value  
            // magnets/us * 179757270 (1 rot/magnet * 1000000 us/sec * 3600 sec/hr * 1/19.85 gearing * 20*pi in/rot * 1/12 ft/in * 1000/5280 milli-mi/ft gives milli-mph  // * 1/1.15 knots/mph gives milliknots
            // Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
            
                
            if (filt_lp_spike(val_temp)) {
                if (val_temp == 0) assign_val(0.0);
                else assign_val ( filt_ema(val_temp, *d_val, ema_alpha) );
            }
        }
    }
};

// Device::Encoder is a class for rotary encoder input devices
class Encoder : public Device {
  public:
};

// Device::Toggle is a base class for system signals or devices having a boolean value
class Toggle : public Device {
  public:
    bool val, val_last, can_sim;
    int32_t pin, val_source;
    Toggle(std::string& eng_name, int32_t arg_pin) {
        pin = arg_pin;
        can_sim = true;
    }
};

// Device::Toggle::InToggle is system signals or devices having a boolean value that serve as inputs (eg basicsw, cruisesw)
class InToggle : public Toggle {
  public:
    InToggle(std::string& eng_name, int32_t arg_pin) 
    : Toggle(eng_name, arg_pin) {
        val_source = PIN;
    }
    void set_val(bool arg_val) {
        if (val_source != PIN) {
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
class OutToggle : public Toggle {
  public:
    OutToggle(std::string& eng_name, int32_t arg_pin)
    : Toggle(eng_name, arg_pin) {
        val_source = LIVE;
    }
    void set_val(bool arg_val) {
        val_last = val;
        val = arg_val;
        if (val_source == LIVE) write();
    }
    void write() {
        digitalWrite(pin, val);
    }
};


#endif
