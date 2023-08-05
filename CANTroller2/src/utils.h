/* Contains utility functions, classes, and defines */

#ifndef UTILS_H
#define UTILS_H

#include <cstdint> // for uint types
#include <cstdio> // for printf

/* Config Defines */
#define pwm_jaguars true
#ifdef pwm_jaguars
#else  // jaguars controlled over asynchronous serial port
    #include <HardwareSerial.h>
    HardwareSerial jagPort(1);  // Open serisl port to communicate with jaguar controllers for steering & brake motors
#endif

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

class Timer {  // 32 bit microsecond timer overflows after 71.5 minutes, so we use 64 bits
  protected:
    // TODO: should check whether these actually use larger types (often compilers will just use their largest word)
    volatile int64_t start_us = 0;
    volatile int64_t timeout_us = 0;
  public:
    Timer() { reset(); }
    Timer(uint32_t arg_timeout_us) { set((int64_t)arg_timeout_us); }
    void IRAM_ATTR set(int64_t arg_timeout_us) {
        timeout_us = arg_timeout_us;
        reset();
     }
    void IRAM_ATTR reset() { start_us = esp_timer_get_time(); }
    bool IRAM_ATTR expired() { return esp_timer_get_time() > start_us + timeout_us; }
    bool IRAM_ATTR expireset (void) {  // Like expired() but automatically resets if expired
        if (esp_timer_get_time() <= start_us + timeout_us) return false;
        reset();
        return true;
    }    
    int64_t IRAM_ATTR elapsed() { return esp_timer_get_time() - start_us; }
    int64_t IRAM_ATTR get_timeout() { return timeout_us; }
};
#endif // UTILS_H