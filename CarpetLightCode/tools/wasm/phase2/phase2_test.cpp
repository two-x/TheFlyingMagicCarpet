// Phase 2: LedController.h's pot/encoder/button HAL swap -- verifies the
// REAL PushButton/Encoder classes (interrupt-replaced, not reimplemented)
// behave correctly when driven by direct injector calls instead of a real
// interrupt firing. See tools/wasm/README.md / the migration plan.
#include "../../../src/LedController.h"
#include <emscripten/emscripten.h>

using namespace LedControl;

static Encoder * enc = nullptr;
static const int PIN_A = 10, PIN_B = 11, PIN_SW = 12, PIN_POT = A1;

extern "C" {

EMSCRIPTEN_KEEPALIVE
void phase2_setup() {
   enc = getEncoder( PIN_A, PIN_B, PIN_SW );
}

EMSCRIPTEN_KEEPALIVE
void phase2_setMillis( uint32_t ms ) { halSetMillis( ms ); }

// direct position injection -- see LedController.h's digitalWriteDirect/
// digitalReadDirect swap comment for why this is legitimate: no real
// quadrature A/B toggling needed since JS already knows the direction.
EMSCRIPTEN_KEEPALIVE
void phase2_injectEncoderSteps( int delta ) {
   if ( delta > 0 ) { EncoderImpl::a_ = 1; EncoderImpl::b_ = 0; for ( int i = 0; i < delta; ++i ) EncoderImpl::updatePosition(); }
   else if ( delta < 0 ) { EncoderImpl::a_ = 0; EncoderImpl::b_ = 0; for ( int i = 0; i < -delta; ++i ) EncoderImpl::updatePosition(); }
}

EMSCRIPTEN_KEEPALIVE
int phase2_readPositionDelta() { return enc->readPositionDelta(); }

EMSCRIPTEN_KEEPALIVE
void phase2_resetPositionDelta() { enc->resetPositionDelta(); }

// simulates a real button edge exactly the way the real ISR would see it:
// set the pin level, then call the SAME isr() function a real interrupt
// would have called -- PushButton::update()'s replay logic downstream is
// completely unmodified.
EMSCRIPTEN_KEEPALIVE
void phase2_injectButtonEdge( bool down ) {
   digitalWrite( PushButtonEdge::pin_, down ? LOW : HIGH ); // active low
   PushButtonEdge::isr();
}

EMSCRIPTEN_KEEPALIVE
void phase2_update() { enc->update(); }

EMSCRIPTEN_KEEPALIVE
int phase2_short() { return enc->button.shortpress(); }
EMSCRIPTEN_KEEPALIVE
int phase2_medium() { return enc->button.mediumpress(); }
EMSCRIPTEN_KEEPALIVE
int phase2_long() { return enc->button.longpress(); }
EMSCRIPTEN_KEEPALIVE
int phase2_double() { return enc->button.doublepress(); }

EMSCRIPTEN_KEEPALIVE
void phase2_setPotRaw( int raw ) { halSetPinState( PIN_POT, raw ); }
EMSCRIPTEN_KEEPALIVE
float phase2_readPotPercent() {
   static Potentiometer pot( PIN_POT );
   return pot.readPercent();
}

}
