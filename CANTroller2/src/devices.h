#ifndef CLASSES_H
#define CLASSES_H
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <memory> // for shared_ptr
#include "Arduino.h"
#include "utils.h"
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
    bool _can_source[6] = { true,       // UNDEF
                            true,       // FIXED
                            false,      // PIN
                            true,       // TOUCH
                            false,      // POT
                            false };    // CALC
    ControllerMode _source = ControllerMode::UNDEF;
    uint8_t _pin;
    bool _enabled = true;
  public:
    Timer timer;  // Can be used for external purposes

    Device() = delete; // should always be created with a pin
    Device(uint8_t arg_pin) : _pin(arg_pin) {}

    bool can_source(ControllerMode arg_source) { return _can_source[static_cast<uint8_t>(arg_source)]; }
    bool set_source(ControllerMode arg_source) {
        if (_can_source[static_cast<uint8_t>(arg_source)]) {
            _source = arg_source;
            return true;
        } 
        return false;
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
    // TODO: move these into a child class as needed
    // float m_factor = 1.0, b_offset = 0.0;  // Multiplier and adder values to plug in for unit conversion math
    // bool invert = false;  // Flag to indicated if unit conversion math should multiply or divide
    // float native_to_human(float arg_val_native) {
    //     if (!invert) {
    //         if (dir == REV) return min_native->get() + (max_native->get() - (b_offset + m_factor * arg_val_native));
    //         return b_offset + m_factor * arg_val_native;
    //         // if (dir == REV) return *p_min_native + (*p_max_native - (b_offset + m_factor * arg_val_native));
    //         // return b_offset + m_factor * arg_val_native;
    //     }
    //     if (arg_val_native) {
    //         if (dir == REV) return min_native->get() + (max_native->get() - (b_offset + m_factor / arg_val_native));
    //         return b_offset + m_factor/arg_val_native;
    //         // if (dir == _REV) return *p_min_native + (*p_max_native - (b_offset + m_factor / arg_val_native));
    //         // return b_offset + m_factor/arg_val_native;
    //     } 
    //     printf ("Error: unit conversion refused to divide by zero\n");
    //     return -1;
    // }
    // float human_to_native(float arg_val_human) {
    //     if (dir == REV) {
    //         arg_val_human = min_native->get() + (max_native->get() - arg_val_human);
    //     }
    //     // if (dir == _REV) arg_val_human = *p_min_native + (*p_max_native - arg_val_human);
    //     if (invert && (arg_val_human - b_offset)) {
    //         return m_factor / (arg_val_human - b_offset);
    //     } else if (!invert && m_factor) {
    //         return (arg_val_human - b_offset) / m_factor;
    //     }
    //     // if (invert && (arg_val_human - b_offset)) return m_factor / (arg_val_human - b_offset);
    //     // if (invert && (arg_val_human - b_offset)) return m_factor / (arg_val_human - b_offset);
    //     // else if (!invert && m_factor) return (arg_val_human - b_offset) / m_factor;
    //     printf ("Error: unit conversion refused to divide by zero\n");
    //     return -1;
    // }
    //
    // // Convert units from base numerical value to disp units:  val_native = m-factor*val_numeric + offset  -or-  val_native = m-factor/val_numeric + offset  where m-factor, b-offset, invert are set here
    // void set_convert(float arg_m_factor, float arg_b_offset, bool arg_invert) {
    //     m_factor = arg_m_factor;
    //     b_offset = arg_b_offset;
    //     invert = arg_invert;
    //     // NOTE: this isn't changing here, it should probably be set elsewhere
    //     // dir = (human_to_native(min_human->get()) <= human_to_native(max_human->get())) ? FWD : REV;
    //     // NOTE: what is this supposed to be doing?
    //     // *p_min_native = human_to_native ((dir == _FWD) ? *p_min_human : *p_max_human);
    //     // *p_max_native = human_to_native ((dir == _FWD) ? *p_max_human : *p_min_human);

    //     
    //     native.set(human_to_native(human.get()));
    // }

        // virtual HUMAN_T from_native(NATIVE_T arg_val_native) {
        //     float arg_val_f = static_cast<float>(arg_val_native);
        //     float min_f = static_cast<float>(native.get_min());
        //     float max_f = static_cast<float>(native.get_max());
        //     float ret = -1;
        //     if (!_invert) {
        //         if (dir == TransducerDirection::REV) {
        //             ret = min_f + (max_f - (_b_offset + (_m_factor * arg_val_f)));
        //         }
        //         ret = _b_offset + (_m_factor * arg_val_f);
        //     } else if (arg_val_f) { // NOTE: isn't 0.0 a valid value tho?
        //         if (dir == TransducerDirection::REV) {
        //             ret = min_f + (max_f - (_b_offset + (_m_factor / arg_val_f)));
        //         }
        //         ret = _b_offset + (_m_factor / arg_val_f);
        //     } else {
        //         printf ("Error: unit conversion refused to divide by zero\n");
        //         // NOTE: hmmmm, couldn't -1 be a valid value in some caes?
        //     }
        //     return static_cast<HUMAN_T>(ret);
        // }

        // virtual NATIVE_T to_native(HUMAN_T arg_val_human) {
        //     float arg_val_f = static_cast<float>(arg_val_human);
        //     float min_f = static_cast<float>(human.get_min());
        //     float max_f = static_cast<float>(human.get_max());
        //     float ret = -1;
        //     if (dir == TransducerDirection::REV) {
        //         arg_val_f = min_f + (max_f - arg_val_f);
        //     }
        //     if (_invert && (arg_val_f - _b_offset)) {
        //         ret = _m_factor / (arg_val_f - _b_offset);
        //     } else if (!_invert && _m_factor) {
        //         ret = (arg_val_f - _b_offset) / _m_factor;
        //     } else {
        //         printf ("Error: unit conversion refused to divide by zero\n");
        //         // NOTE: hmmmm, couldn't -1 be a valid value in some caes?
        //     }
        //     return static_cast<NATIVE_T>(ret);
        // }

    // NOTE: do we really need two values? or should this just be a single value and get converted wherever needed?
    // To hold val/min/max display values in display units (like V, mph, etc.)
    Param<HUMAN_T> human;
    Param<NATIVE_T> native;

    TransducerDirection dir = TransducerDirection::FWD; // Belongs in a child class for devices. For the case a lower val causes a higher real-life effect, 

    virtual HUMAN_T from_native(NATIVE_T arg_val_native) = 0;
    virtual NATIVE_T to_native(HUMAN_T arg_val_native) = 0;

  public:
    Transducer(uint8_t arg_pin) : Device(arg_pin) {}
    Transducer() = delete;

    // NOTE: do we want to be able to set min/max separately?
    // NOTE: do we want to be able to have limits maintained internally (so not references)?
    // NOTE: since we really only have one value and two representations, should these limit calls change both limits?
    void set_native_limits(Param<NATIVE_T> &arg_min, Param<NATIVE_T> &arg_max) {
        if (arg_min.get() > arg_max.get()) {
            dir = TransducerDirection::REV;
            native.set_limits(arg_max, arg_min);
            // NOTE: should change the human val as well if constrained
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
    // NOTE: pot could be an actual Device as well...
    float *_pot_val; // to pull input from the pot if we're in simulation mode
  public:
    AnalogSensor(uint8_t arg_pin) : Sensor<NATIVE_T, HUMAN_T>(arg_pin) {}
    void read() {
        switch (this->_source) {
            case ControllerMode::PIN: 
                this->native.set(analogRead(this->_pin));
                this->ema(this->human.get()); // filtered values are kept in human format
                break;
            case ControllerMode::TOUCH:
                this->_val_filt.set(this->human.get());
                break;
            case ControllerMode::POT:
                this->human.set(map(*this->_pot_val, 0.0, 100.0, this->human.get_min(), this->human.get_max()));
                this->_val_filt.set(this->human.get());
                break;
        } // take no action in other modes
    }
    void enable_pot_input(float *pot_val_arg) {
        this->set_can_source(ControllerMode::POT, true);
        _pot_val = pot_val_arg;
    }
};

// TODO: add description
class PressureSensor : public AnalogSensor<int32_t, float> {
    protected:
        // Multiplier and adder values to plug in for unit conversion math
        float _m_factor;
        float _b_offset;  
        bool _invert;  // Flag to indicated if unit conversion math should multiply or divide

        virtual float from_native(int32_t arg_val_native) {
            float arg_val_f = static_cast<float>(arg_val_native);
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
            return ret;
        }

        virtual int32_t to_native(float arg_val_human) {
            float arg_val_f = static_cast<float>(arg_val_human);
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
            return ret;
        }

    public:
        // NOTE: for now lets keep all the config stuff here in the class. could also read in values from a config file at some point.
        static constexpr int32_t min_adc_range = 0;
        static constexpr int32_t max_adc_range = 4095;
        static constexpr int32_t min_adc = 658; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
        static constexpr int32_t max_adc = 2080; // Sensor measured maximum reading. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as [wimp] chris can push

        // 1000 psi * (adc_max v - v_min v) / ((4095 adc - 658 adc) * (v-max v - v-min v)) = 0.2 psi/adc 
        static constexpr float initial_psi_per_adc = 1000.0 * (3.3 - 0.554) / ( (max_adc_range - min_adc) * (4.5 - 0.554) ); 
        static constexpr float initial_ema_alpha = 0.1;
        static constexpr float initial_offset = 0.0;
        static constexpr bool initial_invert = false;

        PressureSensor(uint8_t arg_pin, float* pot_arg=nullptr) : AnalogSensor<int32_t, float>(arg_pin) {
            _ema_alpha = initial_ema_alpha;
            _m_factor = initial_psi_per_adc;
            _b_offset = initial_offset;
            _invert = initial_invert;
            native.set_limits(min_adc, max_adc);
            human.set_limits(from_native(native.get_min()), from_native(native.get_max()));
            set_native(min_adc);
            set_can_source(ControllerMode::PIN, true);
            if (pot_arg)
                enable_pot_input(pot_arg);
        }
        PressureSensor() = delete;

        void setup() {
            set_pin(_pin, INPUT);
            set_source(ControllerMode::PIN);
        }
    
        // Convert units from base numerical value to disp units:  val_native = m-factor*val_numeric + offset  -or-  val_native = m-factor/val_numeric + offset  where m-factor, b-offset, invert are set here
        void set_convert(float arg_m_factor, float arg_b_offset, bool arg_invert) {
            _m_factor = arg_m_factor;
            _b_offset = arg_b_offset;
            _invert = arg_invert;
            set_native(native.get());
        }
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


// class TempSensorBus : virtual public Sensor {
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
//     float sens_lim_min = -67.0;  // Minimum reading of sensor is -25 C = -67 F
//     float sens_lim_max = 257.0;  // Maximum reading of sensor is 125 C = 257 F
//     // float roomtemp = 77.0;  // "Room" temperature is 25 C = 77 F
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
//         float this->temps[detected_devices];
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
// };

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
    
    // Spike filter pushes new hotrc readings into a LIFO array, replaces any well-defined spikes with values 
    // interpolated from before and after the spike. Also smooths out abrupt value changes that don't recover later
    int32_t spike_filter (int32_t new_val) {  // pushes next val in, massages any detected spikes, returns filtered past value
        previndex = (depth + index - 1) % depth;  // previndex is where the incoming new value will be stored
        this_delta = new_val - filt_history[previndex];  // Value change since last reading
        // if (button_it) printf (" %1ld%1ld:%4ld ", index, previndex, this_delta);
        // if (button_it) printf ("%s", (prespike_index != -1) ? "^" : " ");
        if (std::abs(this_delta) > spike_cliff) {  // If new value is a cliff edge (start or end of a spike)
            if (prespike_index == -1) {  // If this cliff edge is the start of a new spike
                // if (button_it) printf ("A ");
                prespike_index = previndex;  // save index of last good value just before the cliffgit push
                spike_signbit = signbit (this_delta);  // Save cliff steepness
            }
            else if (spike_signbit == signbit (this_delta)) {  // If this cliff edge deepens an in-progress spike
                // if (button_it) printf ("B");
                inject_interpolations (previndex, filt_history[previndex]);  // Smoothly grade the values from before the last cliff to previous value
                prespike_index = previndex;  // Consider this cliff edge the start of the spike instead
            }
            else {  // If this cliff edge is a recovery of the existing spike
                // if (button_it) printf ("C");
                // !! Linearly interpolate replacement values for the spike values between the two edges
                inject_interpolations (index, new_val);  // Fill in the spike with interpolated values
                prespike_index = -1;  // Cancel the current spike
            }
        }
        else if (prespike_index == index) {  // If a current spike lasted thru our whole buffer
            // if (button_it) printf ("D");
            inject_interpolations (previndex, filt_history[previndex]);  // Smoothly grade the whole buffer
            prespike_index = -1;  // Cancel the current spike
        }
        else {
            // if (button_it) printf ("E ");
        }  // If the new value is not a cliff edge (any action needed?)
        int32_t returnval = filt_history[index];  // Save the incumbent value at current index (oldest value) into buffer
        filt_history[index] = new_val;
        raw_history[index] = new_val;
        index = (index + 1) % depth;  // Update index for next time
        return returnval;  // Return the saved old value
    }
    void inject_interpolations (int32_t endspike_index, int32_t endspike_val) {  // Replaces values between indexes with linear interpolated values
        spike_length = ((depth + endspike_index - prespike_index) % depth) - 1;  // Equal to the spiking values count plus one
        // if (button_it) printf ("%1ld", spike_length);
        if (!spike_length) return;  // Two cliffs in the same direction on consecutive readings needs no adjustment, also prevents divide by zero 
        interpolated_slope = (endspike_val - filt_history[prespike_index]) / spike_length;
        loopindex = 0;
        while (++loopindex <= spike_length) {  // Total loop count is spike_length minus one (or not?)
            filt_history[(prespike_index + loopindex) % depth] = filt_history[prespike_index] + loopindex * interpolated_slope;
        }
    }
    int32_t get_next_rawval () {  // helps to debug the filter from outside the class
        return raw_history[index];
    }
};

// class Hotrc {
//   protected:
//     int32_t* val;
//     int32_t avg, min_index, max_index, failsafe_min, failsafe_max;
//     int32_t depth = 100, index = 1, padding = 7, calc_count = 0;
//     int32_t history[100];  // It will not accept history[depth] - wtf
//     uint32_t sum;
//     bool detect_ready = false;
//   public:
//     Hotrc (int32_t* arg_val, int32_t arg_failsafe_min, int32_t arg_failsafe_max, int32_t arg_padding) {
//         val = arg_val;
//         for (int32_t x = 0; x < depth; x++) history[x] = *val;
//         avg = *val;
//         sum = avg * depth;
//         min_index = 0;
//         max_index = 0;
//         failsafe_min = arg_failsafe_min;
//         failsafe_max = arg_failsafe_max;
//         if (arg_padding != -1) padding = arg_padding;
//     }
//     void set_failsafe (int32_t arg_failsafe_min, int32_t arg_failsafe_max) {  // for manual set failsafe values
//         failsafe_min = arg_failsafe_min - padding;
//         failsafe_max = arg_failsafe_max + padding;
//     }
//     void set_failsafe (void) {  // intended to call from calibration routine while handle is at failsafe value
//         failsafe_min = history[min_index] - padding;
//         failsafe_max = history[max_index] + padding;
//     }
//     void set_pad (int32_t arg_pad) { padding = arg_pad; } 
//     int32_t calc (void) {
//         int32_t nextindex = (index+1) % depth;
//         sum += *val - history[nextindex];
//         avg = sum/depth;
//         int32_t save_min = history[min_index];
//         int32_t save_max = history[max_index];
//         history[nextindex] = *val;
//         if (*val <= save_min) min_index = nextindex;
//         else if (min_index == nextindex) for (int32_t x = 0; x < depth; x++) if (history[x] < history[min_index]) min_index = x;
//         if (*val >= save_max) max_index = nextindex;
//         else if (max_index == nextindex) for (int32_t x = 0; x < depth; x++) if (history[x] > history[max_index]) max_index = x;
//         if (!detect_ready) if (++calc_count >= depth) detect_ready = true;
//         index = nextindex;
//         return avg;
//     }
//     void print (void) { std::cout << "Hotrc:" << history[index] << " avg:" << avg << " min[" << min_index << "]:" << history[min_index] << " max[" << max_index << "]:" << history[max_index] << std::endl; }
//     bool connection_lost (void) { return (detect_ready && (history[min_index] < failsafe_min || history[max_index] > failsafe_max)); }
//     int32_t get_min (void) { return history[min_index]-padding; }
//     int32_t get_max (void) { return history[max_index]+padding; }
//     int32_t get_pad (void) { return padding; }
//     int32_t get_avg (void) { return avg; }
//     int32_t get_failsafe_min (void) { return failsafe_min; }
//     int32_t get_failsafe_max (void) { return failsafe_max; }
// };

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

#endif  // CLASSES_H