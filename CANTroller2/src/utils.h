/* Contains utility functions, classes, and defines */

#ifndef UTILS_H
#define UTILS_H

#include <cstdint> // for uint types
#include <cstdio> // for printf

// pin operations that first check if pin exists for the current board
void set_pin(int32_t pin, int32_t mode) { if (pin >= 0 && pin != 255) pinMode (pin, mode); }
void write_pin(int32_t pin, int32_t val) {  if (pin >= 0 && pin != 255) digitalWrite (pin, val); }
int32_t read_pin(int32_t pin) { return (pin >= 0 && pin != 255) ? digitalRead (pin) : -1; }

float convert_units(float from_units, float convert_factor, bool invert, float in_offset = 0.0, float out_offset = 0.0) {
    if (!invert) return out_offset + convert_factor * (from_units - in_offset);
    if (from_units - in_offset) return out_offset + convert_factor / (from_units - in_offset);
    printf ("convert_units refused to divide by zero: %lf, %lf, %d, %lf, %lf", from_units, convert_factor, invert, in_offset, out_offset);
    return -1;
}

#undef map
inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
inline int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
class Timer {  // 32 bit microsecond timer overflows after 71.5 minutes
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