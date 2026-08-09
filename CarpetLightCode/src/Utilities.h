
#ifndef __UTILITIES_H
#define __UTILITIES_H

#ifdef __EMSCRIPTEN__
#include "HalShim.h"
#else
#include <Arduino.h>
#endif

// The Due's ADC resolution, as one shared constant: set for real here
// (setup() calls analogReadResolution(ADC_RESOLUTION_BITS)) and read back
// from here everywhere a raw ADC reading's range matters (AudioBoard.h's
// scale(), LedController.h's Potentiometer), instead of each spot
// independently hardcoding its own assumed max value -- which is exactly
// how those two drifted apart before (scale() hardcoded 1023, nothing
// actually configured the ADC to be 10-bit, it just happened to default
// there). The WASM visualizer also reads this same constant back via the
// bridge (web_getAdcResolutionBits()) so its simulated ADC quantization
// step always matches whatever this is currently set to.
#define ADC_RESOLUTION_BITS 10
#define ADC_MAX_VALUE ( ( 1 << ADC_RESOLUTION_BITS ) - 1 )

class Timer {
  protected:
    volatile uint32_t start, tout;
  public:
    Timer() { reset(); }
    Timer(uint32_t arg_timeout) { set(arg_timeout); }
    void set(uint32_t arg_timeout) {                                          // sets the timeout to the given number (in ms) and zeroes the timer
        tout = arg_timeout;
        start = millis();
    }
    void reset() { start = millis(); }                            // zeroes the timer
    uint32_t elapsed() { return millis() - start; }                    // returns milliseconds elapsed since last reset
    bool elapsed(uint32_t check) { return millis() - start >= check; } // returns whether the given amount of ms have elapsed since last reset
    uint32_t timeout() { return tout; }                                            // getter function returns the currently set timeout value in ms
    bool expired() { return millis() - start >= tout; }           // returns whether more than the previously-set timeout has elapsed since last reset
    bool expireset() {                                                        // like expired() but automatically resets if expired
        if (millis() - start < tout) return false;
        start = millis();
        return true;
    }
};

#endif