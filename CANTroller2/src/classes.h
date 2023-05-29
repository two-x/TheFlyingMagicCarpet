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

class Device {
  protected:
    bool direction, can_sim_touch, can_sim_pot;
    int32_t pin, val_source;
  public:
    #define PIN 0
    #define TOUCH 1
    #define POT 2
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
        centermode = false;
        set_limits(arg_val_min, arg_val_max);
        hist_init();
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
            centermode = ABSOLUTE;
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
class Actuator : public Transducer {
  protected:
    static Servo servo;
  public:
    Actuator (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)
    : Transducer (eng_name, arg_pin, arg_dir, arg_val_min, arg_val_max) {
        servo.attach(pin);
    }
    void write() {
        servo.writeMicroseconds((int32_t)*filt);  // Write result to jaguar servo interface
    }
};
class Sensor : public Transducer {
  protected:
    void set_ema_alpha (double arg_alpha) {
        ema_alpha = arg_alpha;
    }
  public:
    Sensor (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)
      : Transducer (eng_name, arg_pin, arg_dir, arg_val_min, arg_val_max) {}
    Sensor (std::string& eng_name, int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent) {
      : Transducer (eng_name, arg_pin, arg_dir, arg_val_min, arg_val_max, arg_val_cent);
    }
    double val (bool arg_filt, int32_t arg_hist) {  // returns the output value RAW or FILT. Use hist to retreive past values 0 (newest) - 9 (oldest)
        if (arg_hist < 0 || arg_hist >= histsize) {
            printf ("Transducer::val(): Max value history is past 9 values\n");
            return;
        }            
        return vals[ (int32_t)raw + histsize * (arg_filt + 1) - arg_hist ];
    }
    double raw () { val(RAW, 0); }
    double filt () { val(FILT, 0); }
    double filt_ema (int32_t raw_value, double filt_value, double alpha) {
        return (int32_t)((alpha*(double)raw_value) + ((1-alpha)*filt_value));
    }
};
class BrakeMotor : public Actuator {
  protected:
    static Servo servo;
  public:
    BrakeMotor (double* arg_val, int32_t arg_pin, double arg_val_min, double arg_val_max)
    : Actuator(arg_val, arg_pin, arg_val_min, arg_val_max) {
};
class AnalogSensor : public Sensor {
  public:
    void read() {
        set_val(analogRead(pin));  // Write result to jaguar servo interface
    }
};
class PulseSensor : public Sensor {
  public:
};
class Encoder : public Device {
  public:
};
class Toggle : public Device {
  public:
    bool* my_val;
    bool val_last;
    static bool can_sim = true;
    int32_t sim_source = PIN;
    Toggle(string arg_name, bool *arg_val)
      : Device (arg_name) {  // call constructor from parent class
        my_val = arg_val;
    }
};


#endif
