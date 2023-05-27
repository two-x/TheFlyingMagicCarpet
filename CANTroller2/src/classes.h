#ifndef CLASSES_H
#define CLASSES_H

// #include <stdio.h>
#include <iostream>
#include <string>
#include "Arduino.h"
#include "globals.h"

// using namespace std;

// Timer
// For controlling event timing. Communicates in signed int32 values but uses uint32 under the hood to
// last 72 minutes between overflows. However you can only time for 36 minutes max.
// Note: experimenting with use of micros() instead of mycros(), check for errors on overflow and possibly revert
class Timer {
    volatile uint32_t start_us;  // start time in us
    int32_t timeout_us = 0;  // in us
    public:
        Timer(void) { start_us = micros(); };
        Timer(int32_t arg1) { set(arg1); };

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

class Device {
    public:
        char* eng_name, disp_name, disp_units;
        int32_t sim_source, can_sim;
        Device (string arg_name) {
            can_sim = false;
            sim_source = SENSOR;
            arg_name.copy(eng_name,arg_name.length());
        }
        // draw_value (int32_t arg_x, int32_t arg_y, int32_t arg_color, int32_t arg_bgcolor)
};
class Transducer : public Device {
    public:
        #define ABSOLUTE 0  // assign to outCenterMode if pid output just spans a range, rather than deviating from a centerpoint 
        #define RELATIVE 1  // assign to outCenterMode if pid output deviates from a centerpoint 
        #define RAW 0
        #define FILT 1
        double val_min, val_max, val_center, val_margin, ema_alpha;
        double vals[20]; double* raw; double* filt;
        bool centermode;
        int32_t pin, histsize;
        // Transducer (string arg_name, double *arg_val) 
        // : Device (arg_name) {  // call constructor from parent class
        //     my_val = arg_val;
        // }
        Transducer (int32_t arg_pin, double arg_val_min, double arg_val_max) {
            pin = arg_pin;
            set_limits(arg_val_min, arg_val_max);
            history_init();
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
        bool con_strain (double* arg_value) {  // This should be called "constrain" not "con_strain"
            if (*arg_value < val_min) *arg_value = val_min;
            else if (*arg_value > val_max) *arg_value = val_max;
            else return true;  // No constraint was necessary
            return false;  // If constraint was necessary
        }
        void set_limits (double arg_val_min, double arg_val_max, double arg_val_cent) {
            set_limits(arg_val_min, arg_val_max);
            set_center(arg_val_cent);
        }
        void set_ema_alpha (double arg_alpha) {
            ema_alpha = arg_alpha;
        }
        double filt_ema (int32_t raw_value, double filt_value, double alpha) {
            return (int32_t)((alpha*(double)raw_value) + ((1-alpha)*filt_value));
        }
        void history_init (double arg_val) {
            histsize = sizeof(vals)/2;
            for (int x = 0; x < histsize; x++) {
                vals[x] = arg_val;
                vals[x+histsize] = arg_val;
            }
            raw = &vals[0];
            filt = &vals[histsize];
        }
        double val (bool arg_filt, int32_t arg_hist) {  // returns the output value RAW or FILT. Use hist to retreive past values 0 (newest) - 9 (oldest)
            if (arg_hist < 0 || arg_hist >= histsize) {
                printf ("Transducer::val(): Max value history is 9 past values\n");
                return;
            }            
            return vals[ (int32_t)raw + histsize * (arg_filt + 1) - arg_hist ];
        }
        double raw () { val(RAW, 0); }
        double filt () { val(FILT, 0); }
        
        
};
class Actuator : public Transducer {
    private:
        static Servo servo;
    public:
        // : Transducer (arg_name, *arg_val)
        // : Transducer (arg_name) {}
        Actuator (double* arg_val, int32_t arg_pin, double arg_val_min, double arg_val_max)
        : Transducer (arg_val, arg_pin, arg_val_min, arg_val_max) {
           servo.attach(pin);
        }
        void write_pin() {
            servo.writeMicroseconds(*my_val);  // Write result to jaguar servo interface
        }
};
class Sensor : public Transducer {
    public:
};
class BrakeMotor : public Actuator {
    private:
        static Servo servo;
    public:
        BrakeMotor (double* arg_val, int32_t arg_pin, double arg_val_min, double arg_val_max)
        : Actuator(arg_val, arg_pin, arg_val_min, arg_val_max) {
 }
class AnalogSensor : public Sensor {
    public:
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
        int32_t sim_source = SENSOR;
        Toggle(string arg_name, bool *arg_val)
        : Device (arg_name) {  // call constructor from parent class
            my_val = arg_val;
        }
};


#endif
