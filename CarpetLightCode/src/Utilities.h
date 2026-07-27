
#ifndef __UTILITIES_H
#define __UTILITIES_H

#include <Arduino.h>

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