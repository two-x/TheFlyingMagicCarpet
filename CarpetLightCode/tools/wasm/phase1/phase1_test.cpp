// Phase 1: HalShim.h + AudioBoard.h compiling standalone under Emscripten,
// with a JS harness driving simulated ADC bin values through the real
// AGC/hit-decay pipeline. See tools/wasm/README.md / the migration plan.
#include "../../../src/AudioBoard.h"
#include <emscripten/emscripten.h>

extern "C" {

// one simulated poll: nowMs advances the shim clock, bins[7] are the raw
// 0-1023 MSGEQ7 values for this instant, fed through the exact same
// Read_Frequencies()/pollFrequencies() path real hardware uses.
EMSCRIPTEN_KEEPALIVE
void phase1_poll( uint32_t nowMs, int b0, int b1, int b2, int b3, int b4, int b5, int b6 ) {
   halSetMillis( nowMs );
   int bins[ 7 ] = { b0, b1, b2, b3, b4, b5, b6 };
   halSetAnalogCycle( DC_One, bins, 7 );
   AudioBoard::pollFrequencies( nowMs );
}

EMSCRIPTEN_KEEPALIVE
int phase1_getHitPercent( int band ) { return AudioBoard::getHitPercent( (AudioBand)band ); }

EMSCRIPTEN_KEEPALIVE
int phase1_getNormalPercent( int band ) { return AudioBoard::getNormalPercent( (AudioBand)band ); }

EMSCRIPTEN_KEEPALIVE
int phase1_getRawPercent( int band ) { return AudioBoard::getRawPercent( (AudioBand)band ); }

EMSCRIPTEN_KEEPALIVE
void phase1_setPeakThresholdPercent( float pct ) { AudioBoard::setPeakThresholdPercent( pct ); }

EMSCRIPTEN_KEEPALIVE
void phase1_setNoiseFloorPercent( float pct ) { AudioBoard::setNoiseFloorPercent( pct ); }

EMSCRIPTEN_KEEPALIVE
void phase1_setHitDecayMs( float ms ) { AudioBoard::setHitDecayMs( ms ); }

}
