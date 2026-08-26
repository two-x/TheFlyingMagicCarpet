
#ifndef __UTILITIES_H
#define __UTILITIES_H

#ifdef __EMSCRIPTEN__
#include "HalShim.h"
#else
#include <Arduino.h>
#endif
#include <stdarg.h>

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

// Diagnostic-only: the currently active light show's own display name
// (`showName(currMode)`, see CarpetLightLogic.cpp), set once per loop()
// iteration. Exists so low-level code that has no business depending on
// CarpetLightLogic.cpp (e.g. CarpetGeometry.h reporting which show
// triggered a rare error condition) can still say which show was running,
// without a backward/circular include -- this codebase effectively
// compiles as one translation unit (a single real .cpp file), so a plain
// global defined here is visible everywhere, same as every other
// file-scope static this project already defines directly in a header.
const char * g_currentShowName = "boot";

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

/* --- nospam_printf() ---
 *
 * Rate-limited console logging: prints immediately the first time a given
 * call SITE is reached, then suppresses further prints from that same site
 * until spamDelayMs has passed since the last one actually printed. Each
 * call site gets its own independent cooldown -- a site whose message is
 * currently suppressed never affects a different site's own timing.
 *
 * "Call site" here means "this literal format string" -- nospam_printf_
 * below keys its per-site Timer registry on the fmt POINTER, not its
 * content, which is why every call site should use a real, textually
 * distinct literal for fmt (completely normal for a log message anyway --
 * nobody writes two logically-different warnings with byte-identical
 * text). String literal addresses are stable for the life of the program,
 * so this needs no macro, `__LINE__` trick, or caller-supplied Timer --
 * genuinely just a function, per request.
 *
 * Two forms: `nospam_printf(spamDelayMs, fmt, ...)` (explicit cooldown) and
 * `nospam_printf(fmt, ...)` (defaults to 2000ms) -- both funnel into the
 * same va_list-based core.
 *
 * Registry is a small fixed array (no dynamic allocation) -- if it ever
 * fills up (more distinct call sites than NOSPAM_MAX_SITES_), further
 * NEW sites just print unthrottled rather than silently dropping output;
 * already-registered sites are unaffected.
 */
static const uint8_t NOSPAM_MAX_SITES_ = 16;
static const char * nospamSites_[ NOSPAM_MAX_SITES_ ] = { nullptr };
static Timer nospamTimers_[ NOSPAM_MAX_SITES_ ];
static uint8_t nospamSiteCount_ = 0;

inline void nospam_vprintf_( uint32_t spamDelayMs, const char * fmt, va_list args ) {
   uint8_t slot = NOSPAM_MAX_SITES_; // sentinel: "not registered"
   for ( uint8_t i = 0; i < nospamSiteCount_; ++i ) {
      if ( nospamSites_[ i ] == fmt ) { slot = i; break; }
   }
   bool firstEver = false;
   if ( slot == NOSPAM_MAX_SITES_ && nospamSiteCount_ < NOSPAM_MAX_SITES_ ) {
      slot = nospamSiteCount_++;
      nospamSites_[ slot ] = fmt;
      firstEver = true;
   }
   bool registryFull = ( slot == NOSPAM_MAX_SITES_ );
   bool shouldPrint = firstEver || registryFull || nospamTimers_[ slot ].expired();
   if ( !shouldPrint ) return;
   if ( !registryFull ) nospamTimers_[ slot ].set( spamDelayMs );

   char buf[ 160 ];
   vsnprintf( buf, sizeof( buf ), fmt, args );
   Serial.print( buf );
}

inline void nospam_printf( uint32_t spamDelayMs, const char * fmt, ... ) {
   va_list args;
   va_start( args, fmt );
   nospam_vprintf_( spamDelayMs, fmt, args );
   va_end( args );
}
inline void nospam_printf( const char * fmt, ... ) {
   va_list args;
   va_start( args, fmt );
   nospam_vprintf_( 2000, fmt, args );
   va_end( args );
}

#endif