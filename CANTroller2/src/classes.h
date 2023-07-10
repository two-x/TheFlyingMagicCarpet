#ifndef CLASSES_H
#define CLASSES_H
#include <stdio.h>
#include <iostream>
#include <cmath>
#include "Arduino.h"
#include "globals.h"
#include "display.h"
// #include "xtensa/core-macros.h"  // access to ccount register for esp32 timing ticks
using namespace std;

// int32_t nanos(void) {  // Uses CCount register of ESP
//     uint32_t ccount = XTHAL_GET_CCOUNT();
//     return (int32_t)(temp &= 0x7fffffff);
// }

// Param is a value, possibly constrained between min/max limits, which holds a double value representing a "raw" (aka unfiltered) quantity, which can be displayed
class Param {
  public:
    bool saturated;
  protected:
    double val_raw, min_raw, max_raw;  // To hold val/min/max actual values in case they are passed to us by value. Otherwise if external references used, these are unused.
    double val_raw_last;
    bool constrain_it (double* arg_val, double arg_min, double arg_max) {  // This should be called "constrain" not "constrain_it"
        if (*arg_val < arg_min) {
            *arg_val = arg_min;
            return true;  // Constraint was necessary
        }
        else if (*arg_val > arg_max) {
            *arg_val = arg_max;
            return true;  // Constraint was necessary
        }
        return false;  // No constraint was necessary
    }
  public:
    bool dirty = true, rounding = true;  // Has value been updated since last time value was displayed
    int32_t max_precision = 3;
    char disp_name[9] = "unnamed ";
    char disp_raw_units[5] = "    ";
    double* p_val_raw = &val_raw; double* p_min_raw = &min_raw; double* p_max_raw = &max_raw;  // Pointers to value/max/min, could be internal or external reference
    Param (double* arg_p_val_raw) { p_val_raw = arg_p_val_raw; }
    Param (double arg_val_raw) { 
        val_raw = arg_val_raw;
        *p_min_raw = arg_val_raw;  // Initialize to given value, presumably call set_limits to change later
        *p_max_raw = arg_val_raw;  // Initialize to given value, presumably call set_limits to change later
    }
    Param (void) { Param((double)-1.0); }
    void set_names (const string arg_name, const string arg_units) {
        strcpy (disp_name, arg_name.c_str());
        strcpy (disp_raw_units, arg_units.c_str());
    }
    double round (double val, int32_t digits) { return (rounding) ? (std::round(val * std::pow (10, digits)) / std::pow (10, digits)) : val; }
    double round (double val) { return round (val, max_precision); }
    void set_limits_raw (double* arg_min_raw, double* arg_max_raw) {  // Use if min/max are external references
        if (*arg_min_raw > *arg_max_raw) printf ("Error: *min is >= *max\n");
        else {
            p_min_raw = arg_min_raw;
            p_max_raw = arg_max_raw;
            saturated = constrain_it (p_val_raw, *p_min_raw, *p_max_raw);
        }
    }
    void set_limits_raw (double arg_raw_min, double arg_raw_max) {  // Use if min/max are kept in-class
        if (arg_raw_min > arg_raw_max) printf ("Error: min is >= max\n");
        else {
            min_raw = round (arg_raw_min);
            max_raw = round (arg_raw_max);
            saturated = constrain_it (p_val_raw, *p_min_raw, *p_max_raw);
        }
    }
    bool set_raw (double arg_val_raw) {
        saturated = constrain_it (&arg_val_raw, *p_min_raw, *p_max_raw);
        if (*p_val_raw != arg_val_raw) {
            dirty = true;
            val_raw_last = *p_val_raw;
            *p_val_raw = arg_val_raw;
            return true;
        }
        return false;
    }
    bool add_raw (double arg_add_raw) { 
        if (set_raw (*p_val_raw + arg_add_raw)) return true;
        return false;
    }
    void draw_name (Display &display, int32_t lineno, int32_t target=-1) {
        if (dirty) display.draw_dynamic (lineno, *p_val_raw, *p_min_raw, *p_max_raw, target);
        dirty = false;
    }
    double get_raw () { return *p_val_raw; }
    double get_min_raw () { return *p_min_raw; }
    double get_max_raw () { return *p_max_raw; }
    char* get_name () { return disp_name; }
    char* get_units () { return disp_raw_units; }
    
    
};

// Device class - is a base class for any connected system device or signal
// This is a Param with the added concept of simulatability, and association with a physical pin
class Device : virtual public Param {  // This class is truly virtual, in that it's not complete to have instances, just multiple children
  public:
    enum sources { _UNDEF, _CONST, _PIN, _TOUCH, _POT, _CALC };  // source of value. _TOUCH when simulated with touchscreen buttons
    Timer timer();  // Can be used for external purposes
  protected:
    bool can_source[6] = { true, true, false, true, false, false };  // Which types of sources are possible for this device?
    int32_t pin = -1, enabled = true;
    int32_t source = _UNDEF;
  public:
    Device (double* arg_p_val_raw) : Param (arg_p_val_raw) {}
    Device (double arg_val_raw) : Param (arg_val_raw) {}
    Device (void) : Device ((double)-1.0) {}
    void set_source (int32_t arg_source) { if (can_source[arg_source]) source = arg_source; }
    void set_pin (bool arg_pin) { pin = arg_pin; }
    void set_enabled (bool arg_enable) { enabled = arg_enable; }
    void set_can_source (int32_t arg_source, bool is_possible) { can_source[arg_source] = is_possible; }
    int32_t get_source (void) { return source; }
    int32_t get_pin (void) { return pin; }
    bool get_enabled (void) { return enabled; }
    bool get_can_source (int32_t arg_source) { return can_source[arg_source]; }
};

// Transducer class has all features of Device class but now there is also a "native" version of the value which represents the sensed
//    or driven hardware input/output. This class contains unit conversion between the two.
class Transducer : virtual public Device {
  public:
    int32_t _REV = -1, _FWD = 1;  // possible dir values. REV means native sensed value has the opposite polarity of the real world effect (for example, if we sense fewer us per rotation, the engine is going faster)
    double val_native, val_native_last, min_native, max_native;  // To hold val/min/max display values in display units (like V, mph, etc.)
  protected:
    double m_factor = 1.0, b_offset = 0.0;  // Multiplier and adder values to plug in for unit conversion math
    bool invert = false;  // Flag to indicated if unit conversion math should multiply or divide
    double* p_min_native = &min_native;
    double* p_max_native = &max_native;
    char disp_native_units[5] = "    ";
    double native_to_raw (double arg_val_native) {
        if (!invert) {
            if (dir == _REV) return *p_min_native + (*p_max_native - (b_offset + m_factor * arg_val_native));
            return b_offset + m_factor * arg_val_native;
        }
        if (arg_val_native) {
            if (dir == _REV) return *p_min_native + (*p_max_native - (b_offset + m_factor / arg_val_native));
            return b_offset + m_factor/arg_val_native;
        } 
        printf ("Error: unit conversion refused to divide by zero\n");
        return -1;
    }
    double raw_to_native (double arg_val_raw) {
        if (dir == _REV) arg_val_raw = *p_min_native + (*p_max_native - arg_val_raw);
        if (invert && (arg_val_raw - b_offset)) return m_factor / (arg_val_raw - b_offset);
        else if (!invert && m_factor) return (arg_val_raw - b_offset) / m_factor;
        printf ("Error: unit conversion refused to divide by zero\n");
        return -1;
    }
  public:
    int32_t dir = _FWD;  // // Belongs in a child class for devices. For the case a lower val causes a higher real-life effect, 
    Transducer (double arg_val_native) : Device() {  
        val_native = arg_val_native;
        set_raw (native_to_raw (arg_val_native));
        min_native = raw_to_native (*p_min_raw);
        max_native = raw_to_native (*p_max_raw);
    }
    Transducer (void) { Transducer((double)-1.0); }
    // Use if the value at *p_val_raw corresponds with a real-world effect having different units and possibly in the opposite direction as the underlying numerical values
    void set_limits (double* arg_min_native, double* arg_max_native) {  // Direction dir applies when disp limits set.  dir is set to reverse if given minimum > maximum
        if (*arg_min_native > *arg_max_native) {
            dir = _REV;
            p_min_native = arg_max_native;
            p_max_native = arg_min_native;
        }
        else {
            dir = _FWD;
            p_min_native = arg_min_native;
            p_max_native = arg_max_native;
        }
        Param::set_limits_raw(native_to_raw(*p_min_native), native_to_raw(*p_max_native));
    }
    void set_names (const string arg_name, const string arg_units_raw, const string arg_units_native) {
        strcpy (disp_name, arg_name.c_str());
        strcpy (disp_raw_units, arg_units_raw.c_str());
        strcpy (disp_native_units, arg_units_native.c_str());
    }
    // Convert units from base numerical value to disp units:  val_native = m-factor*val_numeric + offset  -or-  val_native = m-factor/val_numeric + offset  where m-factor, b-offset, invert are set here
    void set_convert (double arg_m_factor, double arg_b_offset, bool arg_invert) {
        m_factor = arg_m_factor;
        b_offset = arg_b_offset;
        invert = arg_invert;
        dir = (raw_to_native (*p_min_raw) <= raw_to_native (*p_max_raw)) ? _FWD : _REV;
        *p_min_native = raw_to_native ((dir == _FWD) ? *p_min_raw : *p_max_raw);
        *p_max_native = raw_to_native ((dir == _FWD) ? *p_max_raw : *p_min_raw);
        val_native = raw_to_native (*p_val_raw);
        saturated = constrain_it (&val_native, (*p_min_native), (*p_max_native));
    }
    bool set_native (double arg_val_native) {
        if (Param::set_raw (native_to_raw (arg_val_native))) {
            val_native_last = val_native;
            val_native = raw_to_native (*p_val_raw);
            return true;
        }
        return false;
    }
    bool add_native (double arg_add) {
        if (set_native (val_native + arg_add)) return true;
        return false;
    }
    bool set_raw (int32_t arg_val_raw) {        
        Param::set_raw ((double)arg_val_raw);
        double temp = raw_to_native (*p_val_raw);
        if (temp != val_native) {
            dirty = true;
            val_native_last = val_native;
            val_native = temp;
            return true;
        }
        return false;
    }
    void draw_native (Display &d, int32_t lineno, int32_t target=-1) {
        if (dirty) d.draw_dynamic (lineno, val_native, *p_min_native, *p_max_native, target);
        dirty = false;
    }
    double get_native () { return val_native; }
    double get_min_native () { return *p_min_native; }
    double get_max_native () { return *p_max_native; }
};

// Sensor class - is a base class for control system sensors, ie anything that measures real world data or electrical signals 
class Sensor : virtual public Transducer {
  protected:
    double ema_alpha = 0.1;
    double val_filt;
  public:
    Sensor (double arg_val_native) : Transducer (arg_val_native) { val_filt = arg_val_native; }
    void set_ema_alpha (double arg_alpha) { ema_alpha = arg_alpha; }
    void ema (double arg_new_val_native) { val_filt = ema_alpha * arg_new_val_native + (1 - ema_alpha) * (val_filt); }
    double get_ema_alpha (void) { return ema_alpha; }
    double get_filt (void) { return val_filt; }
};

// class AnalogSensor are sensors where the value is based on an ADC reading (eg brake pressure, brake actuator position, pot)
class AnalogSensor : virtual public Sensor {
  public:
    AnalogSensor (int32_t arg_val_native) : Sensor (arg_val_native) {}
    void read() {
        val_native = analogRead(pin);
        ema ((double)val_native);
    }
};

class PressureSensor : virtual public AnalogSensor {

};

// class TempByPeef {
//   public:
//     #include <OneWire.h>
//     static int secs = 0;
//     static byte data[2];
//     static long f;
//     static long pf;
//     static int16_t raw;


//     TempByPeef (OneWire* onewire_bus) {
//         ds.reset();
//         ds.write(0xCC);        // All Devices present - Skip ROM ID
//         ds.write(0x44);        // start conversion, with parasite power on at the end
//     }

// };

// #include <OneWire.h>

// static int secs = 0;
// static byte data[2];
// static long f;
// static long pf;
// static int16_t raw;


// OneWire  ds(14);  // pin - a 4.7K resistor is necessary

// void setup(void){
//   Serial.begin(115200);
//   Serial.println();
//   Serial.println("Ready");

//   ds.reset();
//   ds.write(0xCC);        // All Devices present - Skip ROM ID
//   ds.write(0x44);        // start conversion, with parasite power on at the end

// }

// void loop(void){
//   if (millis() % 1000 == 0){
//     ds.reset();
//     ds.write(0xCC);        // All Devices present - Skip ROM ID
//     ds.write(0xBE);         // Read Scratchpad
//     data[0] = ds.read();
//     data[1] = ds.read();
//     raw = (data[1] << 8) | data[0];
//     pf = f;
//     f = ((long)raw * 180 / 16 + 3205) / 10;
//     secs++;
//     delay(10);
//     if (abs(pf-f) >= 20 && pf != 0) {
//       Serial.println("Bad-T-" + String(f));
//       f = pf;
//     }
//     ds.reset();
//     ds.write(0xCC);        // All Devices present - Skip ROM ID
//     ds.write(0x44);        // start conversion, with parasite power on at the end

//     Serial.println(String(f/10) + "." + String(f%10) + "°f");
//   }

//   yield();
// }


class TempSensorBus : virtual public Sensor {
//   public:
//     enum temp_sensors { AMBIENT, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };
//   protected:
//     // sample_period 2000000;
//     enum temp_status { IDLE, CONVERT, DELAY };
//     int32_t status = IDLE;
//     int32_t detected_devices = 0;
//     int32_t precision = 9;  // 9-12 bit resolution
//     int32_t current_index = 0;
//     bool blocking_convert = false, verbose = true, read_lock = true;
//     double sens_lim_min = -67.0;  // Minimum reading of sensor is -25 C = -67 F
//     double sens_lim_max = 257.0;  // Maximum reading of sensor is 125 C = 257 F
//     // double roomtemp = 77.0;  // "Room" temperature is 25 C = 77 F
//   public:
//     TempSensorBus (int32_t arg_pin) : Sensor() {
//         set_pin (arg_pin);
//         set_limits (sens_lim_min, sens_lim_max);
//         TempSensorBus::init (true);
//     }
//     void init (bool read_initial_temps) {
//         OneWire this->onewire (pin);
//         DallasTemperature this->tempbus (&pin);
//         tempbus.setWaitForConversion (true);  // Whether to block during conversion process
//         tempbus.setCheckForConversion (true);  // Do not listen to device for conversion result, instead we will wait the worst-case period
//         tempbus.begin();
//         detected_devices = tempbus.getDeviceCount();
//         DeviceAddress this->addrs[detected_devices];
//         double this->temps[detected_devices];
//         if (verbose) printf ("Temp sensors: Detected %d devices. Parasitic power: %s\n", temp_detected_device_ct, ((tempbus.isParasitePowerMode()) ? "On" : "Off"));  // , DEC);
//         if (read_initial_temps) request(true);  // This blocks for over 100ms
//         DeviceAddress temporary_addr;
//         for (int32_t index = 0; index < detected_devices; index++) {
//             int32_t succeed = tempbus.getAddress (temporary_addr, index);
//             if (verbose) {
//                 if (succeed) printf ("Found sensor: index %d, addr %d\n", index, temporary_addr);  // temp_addrs[x]
//                 else printf ("Found ghost device: index %d, addr unknown\n", index);  // printAddress (temp_addrs[x]);
//             }
//             tempbus.setResolution (temporary_addr, precision);  // temp_addrs[x]
//             if (read_initial_temps) read_temp (index);
//             if (verbose) printf ("Sensor %ld temp: %lf degF\n", index, temps[index]);
//         }
//         if (read_initial_temps) {
//             request(true);
//             for (int32_t index; index < detected_devices; index++) read_temp (index);
//             tempbus.setWaitForConversion (blocking_convert);  // makes it async
//         }
//     }
//     void request (bool arg_wait_for_convert=false) {
//         if (enabled && timer.expired()) {
//             read_lock = true;
//             if (++temp_current_index >= 2) temp_current_index -= 2;  // replace 1 with arraysize(temps)
//             tempbus.setWaitForConversion (arg_wait_for_convert || blocking_convert);  //
//             tempbus.requestTemperatures();
//             timer.set (750000 / (1 << (12 - temperature_precision)));  // Give some time before reading temp
//             tempbus.setWaitForConversion (true);
//             if (!arg_wait_for_convert && !blocking_convert) timer.set(750000 / (1 << (12 - precision)));  // Give some time before reading temp
//             temp_status = CONVERT;
//         }
//     }
//     void read_temp (int32_t arg_index) {
//         if (timer.expired()) {
//             read_lock = false;
//             temps[arg_index] = tempbus.getTempFByIndex (arg_index);
//         }
//     }
//     void set_precision (int32_t arg_bits) { precision = constrain (arg_bits, 9, 12); }
//     int32_t get_precision (void) { return precision; }
//     bool get_locked (void) {
//         if (timer.expired()) read_lock = false;
//         return read_lock;
//     }
//     void get_temp (int32_t arg_index) { return (temps[arg_index]); }
};

class Hotrc {
  protected:
    int32_t* val;
    int32_t avg, min_index, max_index, failsafe_min, failsafe_max;
    int32_t depth = 250, index = 1, padding = 7, calc_count = 0;
    int32_t history[250];  // It will not accept history[depth] - wtf
    uint32_t sum;
    bool detect_ready = false;
  public:
    Hotrc (int32_t* arg_val, int32_t arg_failsafe_min, int32_t arg_failsafe_max, int32_t arg_padding) {
        val = arg_val;
        for (int32_t x = 0; x < depth; x++) history[x] = *val;
        avg = *val;
        sum = avg * depth;
        min_index = 0;
        max_index = 0;
        failsafe_min = arg_failsafe_min;
        failsafe_max = arg_failsafe_max;
        if (arg_padding != -1) padding = arg_padding;
    }
    void set_failsafe (int32_t arg_failsafe_min, int32_t arg_failsafe_max) {
        failsafe_min = arg_failsafe_min - padding;
        failsafe_max = arg_failsafe_max + padding;
    }
    void set_failsafe (void) {
        failsafe_min = history[min_index] - padding;
        failsafe_max = history[max_index] + padding;
    }
    void set_pad (int32_t arg_pad) { padding = arg_pad; } 
    int32_t calc (void) {
        int32_t nextindex = (index+1) % depth;
        sum += *val - history[nextindex];
        avg = sum/depth;
        int32_t save_min = history[min_index];
        int32_t save_max = history[max_index];
        history[nextindex] = *val;
        if (*val <= save_min) min_index = nextindex;
        else if (min_index == nextindex) for (int32_t x = 0; x < depth; x++) if (history[x] < history[min_index]) min_index = x;
        if (*val >= save_max) max_index = nextindex;
        else if (max_index == nextindex) for (int32_t x = 0; x < depth; x++) if (history[x] > history[max_index]) max_index = x;
        if (!detect_ready) if (++calc_count >= depth) detect_ready = true;
        index = nextindex;
        return avg;
    }
    void print (void) { std::cout << "Hotrc:" << history[index] << " avg:" << avg << " min[" << min_index << "]:" << history[min_index] << " max[" << max_index << "]:" << history[max_index] << std::endl; }
    bool connection_lost (void) { return (detect_ready && (history[min_index] < failsafe_min || history[max_index] > failsafe_max)); }
    int32_t get_min (void) { return history[min_index]-padding; }
    int32_t get_max (void) { return history[max_index]+padding; }
    int32_t get_pad (void) { return padding; }
    int32_t get_avg (void) { return avg; }
    int32_t get_failsafe_min (void) { return failsafe_min; }
    int32_t get_failsafe_max (void) { return failsafe_max; }
};

// Sensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max)  // std::string& eng_name, 
// : Transducer (arg_pin, arg_dir) {
//     set_limits(arg_val_min, arg_val_max);
// }
// Sensor (int32_t arg_pin, bool arg_dir, double arg_val_min, double arg_val_max, double arg_val_cent)  // std::string& eng_name, 
// : Sensor (arg_pin, arg_dir, arg_val_min, arg_val_max) {
//     set_center(arg_val_cent);
// }
// Sensor (int32_t arg_pin) 
// : Device (arg_pin) {}
// double getval (int32_t arg_hist) {  // returns the output value _RAW or _FILT. Use hist to retreive past values 0 (newest) - 9 (oldest)
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

// // ServoPWM is a base class for our type of actuators, where by varying a pulse width (in us), motors move.
// //    e.g. the gas, brake and steering motors. The gas motor is an actual servo, the others are controlled with servo signaling via jaguars.
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

// // JagMotor is a class specifically for the brake and steering motors. The jaguar stops the motor when receiving 1500 us pulse,
// //    and varies the speed in one direction if pulse is 1500 to (max~2500) us, the other direction if pulse is 1500 to (min~500) us.
// //    Effectively the difference is these have a center value.
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

// class TuneEditor {
// };
// class Simulator {
// };
// class Settings {
// };

#endif  // CLASSES_H