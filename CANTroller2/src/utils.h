/* Contains utility functions, classes, and defines */
#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <cstdint> // for uint types
#include <cstdio> // for printf

/* Value Defines */
#define adcbits 12
#define adcrange_adc 4095  // = 2^adcbits-1
#define adcmidscale_adc 2047  // = 2^(adcbits-1)-1

/* Utility Functions */
#define arraysize(x) ((int32_t)(sizeof(x) / sizeof((x)[0])))  // A macro function to determine the length of string arrays
#define floor(amt, lim) ((amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)

// TODO: check to see if these are different from the builtins, if not then we shouldn't bother redefining them
#undef max
inline float max (float a, float b) { return (a > b) ? a : b; }
inline int32_t max (int32_t a, int32_t b) { return (a > b) ? a : b; }

#undef min
inline float min (float a, float b) { return (a < b) ? a : b; }
inline int32_t min (int32_t a, int32_t b) { return (a < b) ? a : b; }

#undef constrain
inline float constrain (float amt, float low, float high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline int32_t constrain (int32_t amt, int32_t low, int32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline uint32_t constrain (uint32_t amt, uint32_t low, uint32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }

#undef map
inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
inline int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}

bool rounding = true;
float dround (float val, int32_t digits) { return (rounding) ? (std::round(val * std::pow (10, digits)) / std::pow (10, digits)) : val; }

/* Pin Operations */
void set_pin(int32_t pin, int32_t mode) { if (pin >= 0 && pin != 255) pinMode (pin, mode); }
void write_pin(int32_t pin, int32_t val) {  if (pin >= 0 && pin != 255) digitalWrite (pin, val); }
int32_t read_pin(int32_t pin) { return (pin >= 0 && pin != 255) ? digitalRead (pin) : -1; }

float convert_units(float from_units, float convert_factor, bool invert, float in_offset = 0.0, float out_offset = 0.0) {
    if (!invert) return out_offset + convert_factor * (from_units - in_offset);
    if (from_units - in_offset) return out_offset + convert_factor / (from_units - in_offset);
    printf ("convert_units refused to divide by zero: %lf, %lf, %d, %lf, %lf", from_units, convert_factor, invert, in_offset, out_offset);
    return -1;
}

// Exponential Moving Average filter : Smooth out noise on inputs. 0 < alpha < 1 where lower = smoother and higher = more responsive
// Pass in a fresh raw value, address of filtered value, and alpha factor, filtered value will get updated
float ema_filt(float raw, float filt, float alpha) {
    return (alpha * raw) + ((1 - alpha) * filt);
}
template<typename RAW_T, typename FILT_T>
void ema_filt(RAW_T raw, FILT_T* filt, float alpha) {
    float raw_f = static_cast<float>(raw);
    float filt_f = static_cast<float>(*filt);
    *filt = static_cast<FILT_T>(ema_filt(raw_f, filt_f, alpha));
}

class Timer {  // Beware, this 54 bit microsecond timer overflows after every 571 years
  protected:
    volatile int64_t start_us, timeout_us;
  public:
    Timer() { reset(); }
    Timer (int64_t arg_timeout_us) { set (arg_timeout_us); }
    void IRAM_ATTR set (int64_t arg_timeout_us) {
        timeout_us = arg_timeout_us;
        start_us = esp_timer_get_time();
    }
    void IRAM_ATTR reset() { start_us = esp_timer_get_time(); }
    bool IRAM_ATTR expired() { return esp_timer_get_time() >= start_us + timeout_us; }
    bool IRAM_ATTR expireset() {  // Like expired() but immediately resets if expired
        int64_t now_us = esp_timer_get_time();
        if (now_us < start_us + timeout_us) return false;
        start_us = now_us;
        return true;
    }    
    int64_t IRAM_ATTR elapsed() { return esp_timer_get_time() - start_us; }
    int64_t IRAM_ATTR get_timeout() { return timeout_us; }
};

class I2C {
    private:
        int32_t _devicecount = 0;
        uint8_t _addrs[10];
        uint8_t _sda_pin, _scl_pin;
    public:
        I2C(uint8_t sda_pin_arg, uint8_t scl_pin_arg) : _sda_pin(sda_pin_arg), _scl_pin(scl_pin_arg) {}

        void init() {
            printf("I2C driver ");
            Wire.begin(_sda_pin, _scl_pin);  // I2c bus needed for airflow sensor
            byte error, address;
            printf(" scanning ...");
            _devicecount = 0;
            for (address = 1; address < 127; address++ ) {
                Wire.beginTransmission(address);
                error = Wire.endTransmission();
                if (error == 0) {
                    printf (" found addr: 0x%s%x", (address < 16) ? "0" : "", address);
                    _addrs[_devicecount++] = address;
                }
                else if (error==4) printf (" error addr: 0x%s%x", (address < 16) ? "0" : "", address);
            }
            if (_devicecount == 0) printf(" no devices found\n");
            else printf(" done\n");
        }

        bool device_detected(uint8_t addr) {
            for (int32_t i=0; i < _devicecount; i++) {
                if (_addrs[i] == addr) return true;
            }
            return false;
        }
};

#endif // UTILS_H