// Phase 0 toolchain proof: does FastLED's core pixel/color math compile
// clean under Emscripten, with zero Arduino-API dependencies?
// Deliberately includes only the portable pixel-math headers, not the
// <FastLED.h> umbrella -- chipsets.h's hardware-controller-class macros
// don't resolve under this FastLED version's WASM namespace (fl::
// ClocklessController vs the bare name chipsets.h's macro expects), and we
// don't need any real chipset driver classes anyway since the WASM build
// bypasses FastLED's controller system entirely (see the migration plan).
#include "fl/stdint.h"
#include "cpp_compat.h"
#include "fastled_config.h"
#include "led_sysdefs.h"
#include "lib8tion.h"
#include "pixeltypes.h"
#include "hsv2rgb.h"
#include "colorutils.h"

#include <emscripten/emscripten.h>

extern "C" {

EMSCRIPTEN_KEEPALIVE
int phase0_getR() {
   CRGB a = CRGB::Red;
   CRGB b = CRGB::Blue;
   CRGB c = blend( a, b, 128 ); // real FastLED color math, not a literal
   return c.r;
}

EMSCRIPTEN_KEEPALIVE
int phase0_getG() {
   CRGB a = CRGB::Red;
   CRGB b = CRGB::Blue;
   CRGB c = blend( a, b, 128 );
   return c.g;
}

EMSCRIPTEN_KEEPALIVE
int phase0_getB() {
   CRGB a = CRGB::Red;
   CRGB b = CRGB::Blue;
   CRGB c = blend( a, b, 128 );
   return c.b;
}

EMSCRIPTEN_KEEPALIVE
int phase0_getHsvR() {
   CHSV hsv( 96, 255, 255 ); // green
   CRGB rgb = hsv;
   return rgb.r;
}

}
