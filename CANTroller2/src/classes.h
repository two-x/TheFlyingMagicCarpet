#ifndef CLASSES_H
#define CLASSES_H

#include <stdio.h>
#include "Arduino.h"

using namespace std;

int32_t mycros(void) {  // This is "my" micros() function that returns signed int32
    uint32_t temp = micros();
    return (int32_t)(temp &= 0x7fffffff);  // Note this overflows every 35 min due to being only 31 bits. 
}

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

// class Transducer {
// }

// class Actuator : public Transducer {
// }
// class Sensor : public Transducer {
// }
// class AnalogSensor : public Sensor {
// }
// class PulseFreq : public Sensor {
// }

// class Encoder : public Transducer {
// }
// class Toggle : public Transducer {
// }






#endif
