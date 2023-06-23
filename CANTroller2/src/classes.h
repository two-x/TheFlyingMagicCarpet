#ifndef CLASSES_H
#define CLASSES_H
#include <stdio.h>
#include <iostream>
#include <cmath>
#include "Arduino.h"
#include "globals.h"
// #include "xtensa/core-macros.h"  // access to ccount register for esp32 timing ticks
using namespace std;

// int32_t nanos(void) {  // Uses CCount register of ESP
//     uint32_t ccount = XTHAL_GET_CCOUNT();
//     return (int32_t)(temp &= 0x7fffffff);
// }

// Param is a value affecting control, which holds a double value, and can be displayed
class Param {
  public:
    bool saturated;
  protected:
    double val_priv, min_priv, max_priv;  // To hold val/min/max actual values in case they are passed to us by value. Otherwise if external references used, these are unused.
    double val_last;
    bool constrain_it (double* arg_value, double arg_min, double arg_max) {  // This should be called "constrain" not "constrain_it"
        if (*arg_value < arg_min) {
            *arg_value = arg_min;
            return true;  // Constraint was necessary
        }
        else if (*arg_value > arg_max) {
            *arg_value = arg_max;
            return true;  // Constraint was necessary
        }
        return false;  // No constraint was necessary
    }
  public:
    bool dirty = true, rounding = true;  // Has value been updated since last time value was displayed
    int32_t max_precision = 4;
    char disp_name[9] = "unnamed ";
    char disp_units[5] = "    ";
    double* p_val = &val_priv; double* p_min = &min_priv; double* p_max = &max_priv;  // Pointers to value/max/min, could be internal or external reference
    Param (double* arg_p_val) { p_val = arg_p_val; }
    Param (double arg_val) { 
        val_priv = arg_val;
        *p_min = arg_val;  // Initialize to given value, presumably call set_limits to change later
        *p_max = arg_val;  // Initialize to given value, presumably call set_limits to change later
    }
    void set_names (const string arg_name, const string arg_units) {
        strcpy(disp_name, arg_name.c_str());
        strcpy(disp_units, arg_units.c_str());
    }
    double round (double val, int32_t digits) { return (rounding) ? (std::round(val * std::pow (10, digits)) / std::pow (10, digits)) : val; }
    double round (double val) { return round (val, max_precision); }
    void set_limits (double* arg_min, double* arg_max) {  // Use if min/max are external references
        if (*arg_min > *arg_max) printf ("Error: *min is >= *max\n");
        else {
            p_min = arg_min;
            p_max = arg_max;
            saturated = constrain_it (p_val, *p_min, *p_min);
        }
    }
    void set_limits (double arg_min, double arg_max) {  // Use if min/max are kept in-class
        if (arg_min > arg_max) printf ("Error: min is >= max\n");
        else {
            min_priv = round (arg_min);
            max_priv = round (arg_max);
            saturated = constrain_it (p_val, *p_min, *p_min);
        }
    }
    bool set (double arg_val) {
        saturated = constrain_it (&arg_val, *p_min, *p_max);
        if (*p_val != arg_val) {
            dirty = true;
            val_last = *p_val;
            *p_val = arg_val;
            return true;
        }
        return false;
    }
    bool add (double arg_add) { 
        if (set (*p_val + arg_add)) return true;
        return false;
    }
    void draw (int32_t lineno) {
        if (dirty) draw_dynamic (lineno, *p_val, *p_min, *p_max, -1);
        dirty = false;
    }
    void draw (int32_t lineno, int32_t target) {
        if (dirty) draw_dynamic (lineno, *p_val, *p_min, *p_max, target);
        dirty = false;
    }
};

// Transducer class has all features of Param class but the inherited Pram::val is considered the "raw" value, whereas this class
//    operates on "disp"lay units that are real world human understandable like rpm, mph, angle or percent, rather than adc count
//    or us pulse width). So this class handles the conversions necessry for that.
class Transducer : virtual public Param {
  public:
    int32_t _REV = -1, _FWD = 1;  // possible dir values. REV means raw sensed value has the opposite polarity of the real world effect (for example, if we sense fewer us per rotation, the engine is going faster)
  protected:
    double val_disp, val_disp_last, min_disp, max_disp;  // To hold val/min/max display values in display units (like V, mph, etc.)
    double m_factor = 1.0, b_offset = 0.0;  // Multiplier and adder values to plug in for unit conversion math
    bool invert = false;  // Flag to indicated if unit conversion math should multiply or divide
    double* p_min_disp = &min_disp;
    double* p_max_disp = &max_disp;
    char disp_disp_units[5] = "    ";
    double to_disp_units (double numeric_val) {
        if (!invert) {
            if (dir == _REV) return *p_min_disp + (*p_max_disp - (b_offset + m_factor * numeric_val));
            return b_offset + m_factor * numeric_val;
        }
        if (numeric_val) {
            if (dir == _REV) return *p_min_disp + (*p_max_disp - (b_offset + m_factor/numeric_val));
            return b_offset + m_factor/numeric_val;
        } 
        printf ("Error: unit conversion refused to divide by zero\n");
        return -1;
    }
    double from_disp_units (double disp_val) {
        if (dir == _REV) disp_val = *p_min_disp + (*p_max_disp - disp_val);
        if (invert && (disp_val != b_offset)) return m_factor / (disp_val - b_offset);
        else if (!invert && m_factor) return (disp_val - b_offset) / m_factor;
        printf ("Error: unit conversion refused to divide by zero\n");
        return -1;
    }
  public:
    int32_t dir = _FWD;  // // Belongs in a child class for devices. For the case a lower val causes a higher real-life effect, 
    Transducer (double arg_disp_val)  // pass in initial value
      : Param(from_disp_units (arg_disp_val)) {  
        val_disp = arg_disp_val;
        min_disp = to_disp_units (*p_min);
        max_disp = to_disp_units (*p_max);
    }
    // Use if the value at *p_val corresponds with a real-world effect having different units and possibly in the opposite direction as the underlying numerical values
    void set_limits (double* arg_min_disp, double* arg_max_disp) {  // Direction dir applies when disp limits set.  dir is set to reverse if given minimum > maximum
        if (*arg_min_disp > *arg_max_disp) {
            dir = _REV;
            p_min_disp = arg_max_disp;
            p_max_disp = arg_min_disp;
        }
        else {
            dir = _FWD;
            p_min_disp = arg_min_disp;
            p_max_disp = arg_max_disp;
        }
        Param::set_limits(from_disp_units(*p_min_disp), from_disp_units(*p_max_disp));
    }
    void set_names (const string arg_name, const string arg_raw_units, const string arg_disp_units) {
        strcpy(disp_name, arg_name.c_str());
        strcpy(disp_units, arg_raw_units.c_str());
        strcpy(disp_disp_units, arg_disp_units.c_str());
    }
    // Convert units from base numerical value to disp units:  val_disp = m-factor*val_numeric + offset  -or-  val_disp = m-factor/val_numeric + offset  where m-factor, b-offset, invert are set here
    void set_convert (double arg_m_factor, double arg_b_offset, bool arg_invert) {
        m_factor = arg_m_factor;
        b_offset = arg_b_offset;
        invert = arg_invert;
        dir = (to_disp_units (*p_min) <= to_disp_units (*p_max)) ? _FWD : _REV;
        *p_min_disp = to_disp_units ((dir == _FWD) ? *p_min : *p_max);
        *p_max_disp = to_disp_units ((dir == _FWD) ? *p_max : *p_min);
        val_disp = to_disp_units (*p_val);
        saturated = constrain_it (&val_disp, (*p_min_disp), (*p_max_disp));
    }
    bool set (double arg_val) {
        if (Param::set (from_disp_units(arg_val))) {
            val_disp_last = val_disp;
            val_disp = to_disp_units(*p_val);
            return true;
        }
        return false;
    }
    bool add (double arg_add) {
        if (set (val_disp + arg_add)) return true;
        return false;
    }
    void draw (int32_t lineno) {
        if (dirty) draw_dynamic (lineno, val_disp, *p_min_disp, *p_max_disp, -1);
        dirty = false;
    }
    void draw (int32_t lineno, int32_t target) {
        if (dirty) draw_dynamic (lineno, val_disp, *p_min_disp, *p_max_disp, target);
        dirty = false;
    }
};

class Hotrc {
  public:
    int32_t index = 1;
    int32_t padding = 7;
    int32_t depth;
  protected:
    int32_t avg, min_index, max_index;
    int32_t history[30];
    uint32_t sum;
  public:
    Hotrc (int32_t arg_val, int32_t arg_padding) {
        depth = 30;
        for (int32_t x = 0; x < depth; x++) history[x] = arg_val;
        avg = arg_val;
        sum = avg * depth;
        min_index = 0;
        max_index = 0;
        if (arg_padding >= -1) padding = arg_padding;
    }
    Hotrc (int32_t val) { Hotrc (val, -1); }    
    int32_t calc (int32_t val) {
        sum += val - history[index];
        avg = sum/depth;
        if (++index >= depth) index -= depth;  
        if (val < history[min_index]) min_index = index;
        else if (index >= depth) for (int32_t x = 0; x <= depth; x++) if (history[x] < history[min_index]) min_index = x;
        if (val > history[max_index]) max_index = index;
        else if (index >= depth) for (int32_t x = 0; x <= depth; x++) if (history[x] > history[max_index]) max_index = x;
        return avg;
    }
    void print (void) {
        std::cout << "Hotrc:" << history[index] << " av:" << avg << " min:" << min_index << " max:" << max_index << std::endl;
    }
    int32_t get_min () { return history[min_index]-padding; }
    int32_t get_max () { return history[max_index]+padding; }
    int32_t get_pad () { return padding; }
    int32_t get_avg () { return avg; }
};
    // void code_from_globals (void) {
        // // Code from globals.h maybe to marge into here
        // bool hotrc_radio_detected = false;
        // bool hotrc_radio_detected_last = hotrc_radio_detected;
        // bool hotrc_suppress_next_event = true;  // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
        // Timer hotrcPulseTimer;  // OK to not be volatile?
        //
        // int32_t hotrc_pos_failsafe_min_adc = 450;  // The failsafe setting in the hotrc must be set to a trigger level equal to max amount of trim upward from trigger released.
        // int32_t hotrc_pos_failsafe_max_adc = 530;
        // int32_t hotrc_panic_timeout = 1000000;  // how long to receive flameout-range signal from hotrc vertical before panic stopping
        // Timer hotrcPanicTimer(hotrc_panic_timeout);
        // //  ---- tunable ----
        // //double hotrc_pulse_period_us = 1000000.0 / 50;
        // in setup:
        // hotrcPanicTimer.reset();
        //
        // // Detect loss of radio reception and panic stop
        //     if (ctrl_pos_adc[VERT][FILT] > hotrc_pos_failsafe_min_adc && ctrl_pos_adc[VERT][FILT] < hotrc_pos_failsafe_max_adc) {
        //         if (hotrcPanicTimer.expired()) {
        //             hotrc_radio_detected = false;
        //             hotrc_suppress_next_event = true;  // reject spurious ch3 switch event upon next hotrc poweron
        //         }
        //     }
        //     hotrcPanicTimer.reset();
        //     hotrc_radio_detected = true;
        // }
        // hotrc_radio_detected_last = hotrc_radio_detected;
    // }


//
// // Device is a base class for any connected system device or signal
// class Device : public Param {
//   public:
//     enum sources { _UNDEF, _PIN, _TOUCH, _POT, _CODE };  // source of value
//     int32_t pin, source;
//     bool can_sim_touch, can_sim_pot;
//     Device (int32_t arg_pin) {
//         can_sim_touch = true;
//         can_sim_pot = false;
//         source = _PIN;
//         set_pin(arg_pin);
//     }
//     Device (void) : Device (-1) {};
//     void set_source (int32_t arg_source) {
//         if (arg_source == _POT && can_sim_pot) source = _POT;
//         else if (arg_source == _TOUCH && can_sim_touch) source = _TOUCH;
//         else source = arg_source;
//     }
//     void set_pin (bool arg_pin) { pin = arg_pin; }
// };

// // Device::Transducer is a base class for any system devices that convert real_world <--> signals in either direction
// class Transducer : virtual public Device {
//   protected:
//     bool centermode, saturated;
//     double vhist[5];  // vals[] is some past values, [0] beling most recent. 
//     void hist_init (double arg_val) {
//         for (int x = 0; x < sizeof(vals); x++) vhist[x] = arg_val;
//     }
//     void new_val (double new_val) {
//         for (int x = sizeof(vhist)-1; x >= 1; x--) vhist[x] = vhist[x-1];
//         vhist[0] = *val;
//         *val = new_val;
//     }
//   public:
//     enum relativity { ABSOLUTE, RELATIVE };
//     double val_cent, val_margin;
//     Transducer (int32_t arg_pin) 
//     : Device (arg_pin) { // std::string& eng_name, 
//         centermode = ABSOLUTE;
//         val_margin = 0.0;
//         val_cent = 
//         // set_limits(arg_val_min, arg_val_max);
//         hist_init(0);
//     }
//     void set_center (double arg_val_cent) {
//         if (arg_val_cent <= min_val || arg_val_cent >= max_val) {
//             printf ("Transducer::set_limits(): Centerpoint must fall within min/max limits\n");
//             return;
//         }
//         else {
//             val_cent = arg_val_cent;
//             centermode = RELATIVE;
//         }
//     }
//     void set (double val) {
//     }
// };
// // Transducer (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max) { // std::string& eng_name, 
// void add_val (double arg_add_val) {  // 
//     assign_val(*d_val + arg_add_val);
// }


// // Device::Transducer::Sensor is a base class for control system sensors, ie anything that measures real world data or electrical signals 
// class Sensor : virtual public Transducer {
//   protected:
//     bool lp_spike_filtering;
//     double conversion_factor, lp_thresh, spike_thresh, raw_val;  // Multiplier to convert values from direct measured units (adc, us) into useful units (rpm, mmph, etc.)
//   public:
//     #define _RAW 0
//     #define _FILT 1
//     Sensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)  // std::string& eng_name, 
//     : Transducer (arg_pin, arg_dir) {
//         conversion_factor = 1.0;
//         lp_spike_filtering = false;
//         raw_val = 0;
//         set_limits(arg_val_min, arg_val_max);
//     }
//     Sensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)  // std::string& eng_name, 
//     : Sensor (arg_pin, arg_dir, arg_val_min, arg_val_max) {
//         set_center(arg_val_cent);
//     }
//     Sensor (int32_t arg_pin) 
//     : Device (arg_pin) {}
//     // Sensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)  // std::string& eng_name, 
//     // : Transducer (arg_pin, arg_dir) {
//     //     conversion_factor = 1.0;
//     //     lp_spike_filtering = false;
//     //     raw_val = 0;
//     //     set_limits(arg_val_min, arg_val_max, arg_val_cent);
//     // }
//     double getval (int32_t arg_hist) {  // returns the output value _RAW or _FILT. Use hist to retreive past values 0 (newest) - 9 (oldest)
//         if (arg_hist < 0 || arg_hist >= sizeof(vals)) printf ("Transducer::val(): Max value history is past %d values\n", sizeof(vals)-1);            
//         else return vals[d_val - &vals[0] + sizeof(vals) - arg_hist];
//     }
//     double getval (void) { return *d_val; }
//     double getraw (void) { return raw_val; }
//     double filter (void) {
//         return ema_alpha * raw_val + (1-ema_alpha) * (*d_val);
//     }

//     bool new_val (double arg_val) {  // return true if given new value need not be rejected due to spiked value
//         raw_val = arg_val;
//         if (!lp_spike_filtering) return true;
//         else return ( abs( raw_val - getval(2) ) > lp_thresh || raw_val - getval(1) > spike_thresh );  // Should this be raw not filt?
//     }
//     void set_conversion_factor(double arg_factor) {
//         conversion_factor = arg_factor;
//     }
//     void set_ema_alpha (double arg_alpha) {
//         ema_alpha = arg_alpha;
//     }
//     void set_lp_spike_thresh (double arg_lp_thresh, double arg_spike_thresh) {
//         lp_spike_filtering = true;
//         lp_thresh = arg_lp_thresh;  // max delta acceptable over three consecutive readings
//         spike_thresh = arg_spike_thresh;  //min val delta between two readings considered a spike to ignore
//     }
// };

// // Device::Transducer::Sensor::AnalogSensor are sensors where the value is based on an ADC reading (eg brake pressure, brake actuator position, pot)
// class AnalogSensor : virtual public Sensor {
//   public:
//     AnalogSensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)  // std::string& eng_name, 
//     : Sensor (arg_pin, arg_dir, arg_val_min, arg_val_max) {}

//     void read() {
//         int32_t read_val = analogRead(pin);
//         // filter, etc.
//         assign_val((double)read_val); 
//     }
// };

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
//             double val_temp;
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
//     InPWM(int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)
//     : Sensor(arg_pin, arg_dir, arg_val_min, arg_val_max, arg_val_cent)  {}

// };
// // Device::Transducer::Sensor::InPWM::InPWMToggle are servo-pwm standard signals being read in, but only valid values are pulse_min (0) and pulse_max (1) (eg hotrc ch3-4)
// class InPWMToggle : public InPWM {
//   protected:
//     Timer PulseTimer;  // OK to not be volatile?
//   public:
//     InPWMToggle(int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)
//     : Sensor(arg_pin, arg_dir, arg_val_min, arg_val_max, arg_val_cent)  {}

// };
// // Device::Transducer::ServoPWM is a base class for control system actuators, ie anything that turns signals into real world physical change
// class ServoPWM : virtual public Transducer {
//   protected:
//     static Servo servo;
//   public:
//     ServoPWM (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max) {  // std::string& eng_name, 
//         Transducer (arg_pin, arg_dir);
//         servo.attach(pin);
//         set_limits(arg_val_min, arg_val_max);
//     }
//     void write() {
//         servo.writeMicroseconds((int32_t)*d_val);  // Write result to jaguar servo interface
//     }
// };

// // Device::Transducer::ServoPWM::JagMotor is a class specifically for the brake linear actuator motor
// class JagMotor : public ServoPWM {
//   protected:
//     static Servo servo;
//   public:
//     JagMotor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)  // std::string& eng_name, 
//     : ServoPWM(arg_pin, arg_dir, arg_val_min, arg_val_max) {}
// };

// // // Device::Toggle is a base class for system signals or devices having a boolean value
// // class Toggle : virtual public Device {
// //   public:
// //     bool val, val_last, can_sim;
// //     int32_t pin, val_source;
// //     Toggle(int32_t arg_pin) {  // std::string& eng_name, 
// //         pin = arg_pin;
// //         can_sim = true;
// //     }
// // };

// // // Device::Toggle::InToggle is system signals or devices having a boolean value that serve as inputs (eg basicsw, cruisesw)
// // class InToggle : virtual public Toggle {  // Toggle
// //   public:
// //     InToggle(int32_t arg_pin)   // std::string& eng_name, 
// //     : Toggle(arg_pin) {
// //         val_source = _PIN;
// //     }
// //     void set_val(bool arg_val) {
// //         if (val_source != _PIN) {
// //             val_last = val;
// //             val = arg_val;
// //         }
// //     }
// //     void read() {
// //         val_last = val;
// //         val = digitalRead(pin);
// //     }
// // };

// // // Device::Toggle::OutToggle is system signals or devices having a boolean value that serve as outputs (eg ignition, leds, etc.)
// // class OutToggle : virtual public Toggle {  // Toggle
// //   public:
// //     OutToggle(int32_t arg_pin)  // std::string& eng_name, 
// //     : Toggle(arg_pin) {
// //         val_source = _LIVE;
// //     }
// //     void set_val(bool arg_val) {
// //         val_last = val;
// //         val = arg_val;
// //         if (val_source == _LIVE) write();
// //     }
// //     void write() {
// //         digitalWrite(pin, val);
// //     }
// // };


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

// class ControlLoop {
// };

// class TuneEditor {
// };
// class Simulator {
// };
// class Settings {
// };

#endif  // CLASSES_H