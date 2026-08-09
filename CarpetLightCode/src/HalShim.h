/* HalShim.h
 *
 *    Arduino-API-compatible shim for the Emscripten/WASM build target (see
 *    the migration plan at ~/.claude/plans/quiet-watching-cherny.md).
 *    <Arduino.h> doesn't exist under Emscripten -- this provides drop-in
 *    replacements for exactly the subset of it this codebase actually uses,
 *    so every other file (light shows, AudioBoard, etc.) compiles completely
 *    unmodified on both targets.
 *
 *    Real hardware build: contributes nothing (this file is skipped
 *    entirely; Utilities.h includes <Arduino.h> as before).
 *
 *    WASM build: values are driven by inject*() functions called from JS
 *    before each carpetTick() (see the bridge API in the migration plan) --
 *    this file only defines the STORAGE and the Arduino-signature-compatible
 *    accessor functions, not how JS populates them.
 */

#ifndef __HAL_SHIM_H
#define __HAL_SHIM_H

#ifdef __EMSCRIPTEN__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif
#ifndef HALF_PI
#define HALF_PI 1.5707963267948966192313216916398f
#endif
#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559f
#endif

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
#define RISING 2
#define FALLING 3

// analog pin names -- arbitrary distinct integers (matching real hardware's
// own convention of analog pins living past the last digital pin number in
// its own address space) high enough to never collide with a real digital
// pin number this codebase uses elsewhere
#define A0 100
#define A1 101
#define A2 102
#define A3 103
#define A4 104
#define A5 105
#define A6 106
#define A7 107

typedef bool boolean;
typedef uint8_t byte;

// --- min/max/constrain/map, same semantics as Arduino's own macros ---
template <typename T> T hal_min( T a, T b ) { return a < b ? a : b; }
template <typename T> T hal_max( T a, T b ) { return a > b ? a : b; }
#ifndef min
#define min(a,b) hal_min((a),(b))
#endif
#ifndef max
#define max(a,b) hal_max((a),(b))
#endif
#ifndef constrain
#define constrain(x,lo,hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
#endif
#ifndef map
inline long map( long x, long in_min, long in_max, long out_min, long out_max ) {
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif
inline long abs( long x ) { return x < 0 ? -x : x; }

// --- time -- WASM has no real wall clock the firmware should see; JS
// injects "now" once per carpetTick() call (see halSetMillis()) and every
// millis()/micros() call within that same tick returns the same frozen
// value, exactly like a real single-threaded loop() iteration would
// perceive time as constant for its own duration.
static uint32_t hal_millis_ = 0;
inline uint32_t millis() { return hal_millis_; }
inline uint32_t micros() { return hal_millis_ * 1000; }
inline void halSetMillis( uint32_t ms ) { hal_millis_ = ms; }

// delay()/delayMicroseconds() are genuinely blocking on real hardware and
// several real timing bugs this project has already hit (Flame's 8Hz
// strobe, DMX's 2ms enforced spin) depended on that blocking actually
// costing wall-clock time. We have no real clock to block against here, so
// these intentionally do nothing FAST -- see the migration plan's output-
// path design for where blocking cost gets faithfully reproduced instead
// (the WASM NeoPixel controller / dmx_send() stub spend real synchronous
// time deliberately, at the specific call sites where it actually matters,
// rather than here in a generic delay() no-op that every caller
// incidentally hits).
inline void delay( uint32_t ) {}
inline void delayMicroseconds( uint32_t ) {}

// --- pins -- WASM has no real pins. Read/write into a small array JS pokes
// via injectPinState()/reads via halGetPinState() (used for things not
// covered by a more specific injector, e.g. diagnostics); analogRead/
// digitalWrite/pinMode/digitalRead are all Arduino-signature-compatible.
static const int HAL_NUM_PINS = 128;
static int hal_pinState_[ HAL_NUM_PINS ] = { 0 };
inline void pinMode( int, int ) {}
// analogReadResolution() has nothing to configure here -- there's no real
// ADC to reconfigure, the visualizer independently reads the FW's own
// ADC_RESOLUTION_BITS constant (Utilities.h) via the bridge instead of
// observing this call. Still needs to exist so setup()'s real call to it
// compiles under WASM.
inline void analogReadResolution( int ) {}

// Every digitalWrite() gets appended to a small log JS can drain after each
// web_setup()/web_tick() call. Built specifically so the visualizer can
// simulate the MSGEQ7 chip (src/AudioBoard.h) by watching the REAL
// STROBE/RESET pin writes the FW's own Read_Frequencies()/setup() produce,
// rather than guessing at their timing independently -- matches this
// project's rule that simulated-external-hardware behavior (the chip, not
// the FW) is legitimate visualizer code, but it must be driven by the FW's
// actual output, never an assumption about what the FW "should" be doing.
// Generic (logs every pin, not just STROBE/RESET) to keep this shim free of
// AudioBoard-specific knowledge; the visualizer filters by pin number,
// which it reads from AudioBoard.h's own STROBE/RESET #defines via the
// bridge rather than hardcoding 6/7 a third time.
static const int HAL_PIN_LOG_CAPACITY = 128;
static int hal_pinLogPin_[ HAL_PIN_LOG_CAPACITY ];
static int hal_pinLogVal_[ HAL_PIN_LOG_CAPACITY ];
static int hal_pinLogCount_ = 0;
inline void halClearPinLog() { hal_pinLogCount_ = 0; }
inline void digitalWrite( int pin, int val ) {
   if ( pin >= 0 && pin < HAL_NUM_PINS ) hal_pinState_[ pin ] = val;
   if ( hal_pinLogCount_ < HAL_PIN_LOG_CAPACITY ) {
      hal_pinLogPin_[ hal_pinLogCount_ ] = pin;
      hal_pinLogVal_[ hal_pinLogCount_ ] = val;
      ++hal_pinLogCount_;
   }
}
inline int digitalRead( int pin ) { return ( pin >= 0 && pin < HAL_NUM_PINS ) ? hal_pinState_[ pin ] : LOW; }

// analogRead() usually just returns a pin's fixed injected value, EXCEPT
// AudioBoard::Read_Frequencies() reads ONE pin (DC_One) 7 times in a tight
// loop, once per MSGEQ7 bin -- for that pattern, halSetAnalogCycle() lets JS
// inject a short sequence for a pin instead of a single value, consumed one
// call at a time and wrapping (so it also works if some future caller polls
// more than 7 times without a fresh inject).
static const int HAL_MAX_ANALOG_CYCLE = 8;
static int hal_analogCycle_[ HAL_NUM_PINS ][ HAL_MAX_ANALOG_CYCLE ];
static int hal_analogCycleLen_[ HAL_NUM_PINS ] = { 0 };
static int hal_analogCycleIdx_[ HAL_NUM_PINS ] = { 0 };
inline int analogRead( int pin ) {
   if ( pin < 0 || pin >= HAL_NUM_PINS ) return 0;
   if ( hal_analogCycleLen_[ pin ] > 0 ) {
      int v = hal_analogCycle_[ pin ][ hal_analogCycleIdx_[ pin ] % hal_analogCycleLen_[ pin ] ];
      ++hal_analogCycleIdx_[ pin ];
      return v;
   }
   return hal_pinState_[ pin ];
}
inline void halSetPinState( int pin, int val ) { if ( pin >= 0 && pin < HAL_NUM_PINS ) hal_pinState_[ pin ] = val; }
// call once per simulated poll (e.g. once per 7-bin MSGEQ7 read) -- resets
// the cycle position to the start of THIS injection, so each poll starts
// from bin 0 regardless of how many reads the previous poll consumed.
inline void halSetAnalogCycle( int pin, const int * values, int len ) {
   if ( pin < 0 || pin >= HAL_NUM_PINS ) return;
   if ( len > HAL_MAX_ANALOG_CYCLE ) len = HAL_MAX_ANALOG_CYCLE;
   for ( int i = 0; i < len; ++i ) hal_analogCycle_[ pin ][ i ] = values[ i ];
   hal_analogCycleLen_[ pin ] = len;
   hal_analogCycleIdx_[ pin ] = 0;
}

// --- interrupts -- no ISR concept in WASM. attachInterrupt()/interrupts()/
// noInterrupts() are no-ops; the encoder/button HAL swap (LedController.h)
// calls its own update functions directly from injected input instead of
// relying on a real interrupt firing (see the migration plan's phase 2).
inline void attachInterrupt( int, void (*)(), int ) {}
inline void detachInterrupt( int ) {}
inline void interrupts() {}
inline void noInterrupts() {}
inline int digitalPinToInterrupt( int pin ) { return pin; }

// --- random -- Arduino-signature-compatible, backed by libc rand(). Not
// bit-identical to the AVR/SAM random() sequence (a known, accepted
// divergence -- see the migration plan's risk list); shows only use this
// for visual variety, never a specific reproducible sequence.
inline long random( long howbig ) { return howbig <= 0 ? 0 : ( ::rand() % howbig ); }
inline long random( long howsmall, long howbig ) { return howsmall >= howbig ? howsmall : howsmall + random( howbig - howsmall ); }
inline void randomSeed( unsigned long seed ) { ::srand( (unsigned int)seed ); }

// --- Serial -- every call site here is diagnostic console output with no
// effect on light-show computation. print()/println() forward to stdout
// (visible in the browser devtools console when run under Node, or simply
// discarded under a browser main-thread Module -- either is fine, nothing
// downstream depends on this reaching anywhere in particular).
#include <cstdio>
struct HalSerial {
   void begin( long ) {}
   void flush() {}
   void print( const char * s ) { fputs( s, stdout ); }
   void print( int v ) { printf( "%d", v ); }
   void print( unsigned int v ) { printf( "%u", v ); }
   void print( float v ) { printf( "%f", v ); }
   void println( const char * s ) { fputs( s, stdout ); fputc( '\n', stdout ); }
   void println( int v ) { printf( "%d\n", v ); }
   void println( unsigned int v ) { printf( "%u\n", v ); }
   void println( float v ) { printf( "%f\n", v ); }
   void println() { fputc( '\n', stdout ); }
};
static HalSerial Serial;

#endif // __EMSCRIPTEN__

#endif // __HAL_SHIM_H
