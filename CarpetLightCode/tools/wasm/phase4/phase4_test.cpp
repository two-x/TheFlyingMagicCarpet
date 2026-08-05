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

}
