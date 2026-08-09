// Phase 4: the REAL CarpetLightLogic.cpp (setup()/loop(), all 5 real shows,
// CarpetGeometry, MagicCarpet) compiling and running under Emscripten.
// #includes the real .cpp directly (Arduino sketches have no main() of
// their own -- the framework calls setup()/loop()), matching exactly how
// the real Due build treats it. See tools/wasm/README.md / the migration
// plan.
#include "../../../src/CarpetLightLogic.cpp"
#include <emscripten/emscripten.h>

extern "C" {

EMSCRIPTEN_KEEPALIVE
void web_setup() { setup(); }

EMSCRIPTEN_KEEPALIVE
void web_tick( uint32_t nowMs ) {
   halSetMillis( nowMs );
   loop();
}

EMSCRIPTEN_KEEPALIVE
uint8_t * web_getRopeLedsPtr() { return (uint8_t *)carpet->ropeLeds; }
EMSCRIPTEN_KEEPALIVE
uint8_t * web_getMegabarLedsPtr() { return (uint8_t *)carpet->megabarLeds; }
EMSCRIPTEN_KEEPALIVE
uint8_t * web_getChinaLedsPtr() { return (uint8_t *)carpet->chinaLeds; }
EMSCRIPTEN_KEEPALIVE
int web_getNumRopeLeds() { return NUM_NEO_LEDS_ACTUAL; }
EMSCRIPTEN_KEEPALIVE
int web_getNumMegabarLeds() { return NUM_MEGABAR_LEDS; }
EMSCRIPTEN_KEEPALIVE
int web_getNumChinaLeds() { return NUM_CHINA_LEDS; }
EMSCRIPTEN_KEEPALIVE
int web_sizeofRopeLed() { return sizeof( carpet->ropeLeds[ 0 ] ); }
EMSCRIPTEN_KEEPALIVE
int web_sizeofMegabarLed() { return sizeof( carpet->megabarLeds[ 0 ] ); }

EMSCRIPTEN_KEEPALIVE
const char * web_getCurrentShowName() { return showName( currMode ); }
EMSCRIPTEN_KEEPALIVE
const char * web_getCurrentVariationName() { return currLightShow->variationName(); }

// test-only convenience -- real UI drives this via the encoder/button
// injectors from phase 2, this just makes show-cycling easy to poke at
// directly for this harness
EMSCRIPTEN_KEEPALIVE
void web_forcePressShort() {
   using namespace LedControl;
   digitalWrite( PushButtonEdge::pin_, LOW ); PushButtonEdge::isr();
   halSetMillis( hal_millis_ + 50 );
   digitalWrite( PushButtonEdge::pin_, HIGH ); PushButtonEdge::isr();
}

// ---------------------------------------------------------------------
// Audio injection + readback. Per the visualizer's architecture rule (see
// tools/wasm/README.md / the migration plan): the visualizer may only feed
// a simulated raw signal in (here, one MSGEQ7 poll's worth of 7 bin
// values) and read the REAL AudioBoard's own computed output back out --
// no hit-detection, level-smoothing, or AGC math may be reimplemented in
// JS. web_injectAdcBins() writes into the same static scratch buffer
// AudioBoard::pollFrequencies() (called every web_tick(), from the real
// loop()) reads via Read_Frequencies()'s analogRead(DC_One) x7 cycle --
// see HalShim.h's halSetAnalogCycle().
static int adcBinsScratch_[ 7 ];
EMSCRIPTEN_KEEPALIVE
int * web_getAdcBinsBufferPtr() { return adcBinsScratch_; }
EMSCRIPTEN_KEEPALIVE
void web_injectAdcBins() { halSetAnalogCycle( DC_One, adcBinsScratch_, 7 ); }

EMSCRIPTEN_KEEPALIVE
int web_audioGetHitPercent( int band ) { return AudioBoard::getHitPercent( (AudioBand)band ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetNormalPercent( int band ) { return AudioBoard::getNormalPercent( (AudioBand)band ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetRawPercent( int band ) { return AudioBoard::getRawPercent( (AudioBand)band ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetRmsPercent( int band ) { return AudioBoard::getRmsPercent( (AudioBand)band ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetOverallLevelPercent() { return AudioBoard::getOverallLevelPercent(); }
EMSCRIPTEN_KEEPALIVE
int web_audioIsSilent() { return AudioBoard::silent_ ? 1 : 0; }

EMSCRIPTEN_KEEPALIVE
void web_audioSetAgcMode( int mode ) { AudioBoard::setAgcMode( (uint8_t)mode ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetAgcMode() { return AudioBoard::getAgcMode(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetNoiseFloorPercent( float pct ) { AudioBoard::setNoiseFloorPercent( pct ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetNoiseFloorPercent() { return AudioBoard::getNoiseFloorPercent(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetPeakThresholdPercent( float pct ) { AudioBoard::setPeakThresholdPercent( pct ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetPeakThresholdPercent() { return AudioBoard::getPeakThresholdPercent(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetHitDecayMs( float ms ) { AudioBoard::setHitDecayMs( ms ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetHitDecayMs() { return AudioBoard::getHitDecayMs(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetAudioForesightMs( float ms ) { AudioBoard::setAudioForesightMs( ms ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetAudioForesightMs() { return AudioBoard::getAudioForesightMs(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetHitPredictionMs( float ms ) { AudioBoard::setHitPredictionMs( ms ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetHitPredictionMs() { return AudioBoard::getHitPredictionMs(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetHitPredictionStyle( int style ) { AudioBoard::setHitPredictionStyle( (uint8_t)style ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetHitPredictionStyle() { return AudioBoard::getHitPredictionStyle(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetSoundReactivityEnabled( int enabled ) { AudioBoard::setSoundReactivityEnabled( enabled != 0 ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetSoundReactivityEnabled() { return AudioBoard::getSoundReactivityEnabled() ? 1 : 0; }

// ---------------------------------------------------------------------
// MSGEQ7 chip simulation support. The visualizer models the physical
// MSGEQ7 chip + surrounding analog circuitry itself (legitimate -- it's
// external hardware, not FW logic), but it must be driven by the FW's own
// REAL STROBE/RESET pin writes and the FW's own real ADC-range constant,
// never an independent guess at either. See HalShim.h's pin-write log.
EMSCRIPTEN_KEEPALIVE
int web_getAdcResolutionBits() { return ADC_RESOLUTION_BITS; }
EMSCRIPTEN_KEEPALIVE
int web_getAdcMaxValue() { return ADC_MAX_VALUE; }
EMSCRIPTEN_KEEPALIVE
int web_getStrobePin() { return STROBE; }
EMSCRIPTEN_KEEPALIVE
int web_getResetPin() { return RESET; }

EMSCRIPTEN_KEEPALIVE
int * web_getPinLogPinsPtr() { return hal_pinLogPin_; }
EMSCRIPTEN_KEEPALIVE
int * web_getPinLogValsPtr() { return hal_pinLogVal_; }
EMSCRIPTEN_KEEPALIVE
int web_getPinLogCount() { return hal_pinLogCount_; }
EMSCRIPTEN_KEEPALIVE
void web_clearPinLog() { halClearPinLog(); }

}
