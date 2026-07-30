/* LedController.h
 *
 *    Defines functions and interrupts for controller knobs.
 *
 *    Author: Anders Linn
 *    Date: July 2017
 */

#ifndef __LED_CONTROLLER_H
#define __LED_CONTROLLER_H

#include "Utilities.h"

// max voltage from an analog input pin
#define MAX_VOLTAGE 1023

// TODO: these are useful, move them somewhere common
inline void digitalWriteDirect( int pin, bool val ) {
   if ( val ) {
      g_APinDescription[ pin ].pPort->PIO_SODR = g_APinDescription[ pin ].ulPin;
   } else {
      g_APinDescription[ pin ].pPort->PIO_CODR = g_APinDescription[ pin ].ulPin;
   }
}

inline int digitalReadDirect( int pin ) {
   return !!( g_APinDescription[ pin ].pPort->PIO_PDSR &
              g_APinDescription[ pin ].ulPin );
}

namespace LedControl {

class Potentiometer {
 public:
   Potentiometer( uint8_t pin ) : pin_( pin ) {}

   // the value light shows should see: the real live pot reading, EXCEPT
   // while a low-power simulation is active (see updateLowPower()), during
   // which it instead returns a synthetic value fading from the pot's real
   // reading at the moment low power engaged down to 0 over FADE_DURATION_MS,
   // then holding at 0 -- i.e. every show's own pot-driven behavior smoothly
   // winds down to its "pot at zero" state without the show needing to know
   // anything about low-power mode. A real manual turn away from that
   // captured starting position (more than NOISE_FLOOR_RAW) cancels the
   // simulation for the rest of this low-power episode and hands back live
   // control -- see README.md, "Low power mode".
   uint16_t read() {
      uint16_t raw = readLive();
      if ( !simulationActive_ ) return raw;
      uint16_t diff = ( raw > fadeStartRaw_ ) ? ( raw - fadeStartRaw_ ) : ( fadeStartRaw_ - raw );
      if ( diff >= NOISE_FLOOR_RAW ) {
         simulationActive_ = false; // manual override -- sticks until the next low-power rising edge
         return raw;
      }
      if ( fadeTimer_.expired() ) return 0;
      float frac = 1.0f - ( (float)fadeTimer_.elapsed() / (float)FADE_DURATION_MS ); // 1 -> 0 over 20s
      return (uint16_t)( fadeStartRaw_ * frac + 0.5f );
   }

   // the true, always-live pot reading, bypassing the low-power simulation --
   // for config-mode UI code, which must never be affected by it
   uint16_t readLive() {
      return analogRead( pin_ );
   }

   // call once per main loop iteration, ModeShow only (never during config
   // screens -- see CarpetLightLogic.cpp). wantLowPower is the vehicle's
   // current low-power state per SpeedLink. Edge-triggered: a fresh fade
   // only (re)starts on the rising edge (just-entered low power), so a
   // manual-turn cancellation (above) isn't immediately re-armed by the
   // vehicle still reporting the same ongoing low-power state next loop.
   void updateLowPower( bool wantLowPower ) {
      if ( wantLowPower && !lowPowerEngaged_ ) {
         simulationActive_ = true;
         fadeStartRaw_ = readLive();
         fadeTimer_.set( FADE_DURATION_MS ); // starts the fade-to-zero countdown now
      } else if ( !wantLowPower ) {
         simulationActive_ = false; // exiting low power snaps straight back to live
      }
      lowPowerEngaged_ = wantLowPower;
   }

 private:
   static const uint32_t FADE_DURATION_MS = 20000;
   static const uint16_t NOISE_FLOOR_RAW = (uint16_t)( 0.02f * MAX_VOLTAGE + 0.5f ); // ~2%

   uint8_t pin_;
   bool lowPowerEngaged_ = false;   // tracks the vehicle's actual reported state
   bool simulationActive_ = false;  // are we currently overriding read()'s return value?
   uint16_t fadeStartRaw_ = 0;
   Timer fadeTimer_;
};

enum ButtonPress { PressNone = 0, PressShort = 1, PressMedium = 2, PressLong = 3, PressExtraLong = 4, PressDouble = 5 };

// Polled, not interrupt-driven: press timing thresholds are coarse (hundreds of
// ms), so a per-loop update() is plenty responsive. Medium/long are classified
// only at release, by how long the timer had been running.
//
// Extra-long and double are both "live" events: they become valid the instant
// they're detected during a button-down event, without waiting for release.
// Once either fires, no other press event can fire for the rest of that same
// hold -- not the other one, not a threshold-crossing flash, not a release
// classification -- until the button is physically released.
//
// Short press is special even beyond that: a release that lands in
// short-press range isn't reported right away. Instead it starts a
// doublePressWindowMillis_ watch for a second press. If none shows up in
// time, it resolves to a genuine short press (delayed by that window). If a
// second press starts within the window, that press IS the double, valid
// immediately on its down-edge -- the original short press is discarded.
//
// Edge capture is interrupt-driven, not polled: the button pin is watched by
// a CHANGE interrupt (PushButtonEdge::isr(), below), which timestamps every
// down/up transition into a small queue the instant it happens. update()
// drains that queue and replays each edge using ITS OWN timestamp, rather
// than sampling the pin's live state once per call. This matters because the
// main loop's iteration time is dominated by FastLED.show() (~14ms, most of
// it spent with interrupts disabled during rope LED bit-banging -- see
// README's timing report) plus other blocking work; a plain polled
// digitalRead() can miss a short tap entirely if the whole press-and-release
// happens between two update() calls. An edge interrupt still can't fire
// during FastLED's disabled-interrupt window, but that window is much
// narrower than a full loop iteration, so this meaningfully shrinks the
// blind spot rather than eliminating the dependency on loop cadence entirely.
namespace PushButtonEdge {

struct Edge {
   bool down;
   uint32_t atMillis;
};

static const uint8_t QUEUE_SIZE = 8; // headroom for rapid taps/bounce piling up between polls
volatile Edge queue_[ QUEUE_SIZE ];
volatile uint8_t queueTail_ = 0; // index of the oldest unread edge
volatile uint8_t queueCount_ = 0;
uint8_t pin_ = 0;

void isr() {
   if ( queueCount_ >= QUEUE_SIZE ) return; // main loop has fallen too far behind -- drop rather than overwrite
   uint8_t head = ( queueTail_ + queueCount_ ) % QUEUE_SIZE;
   queue_[ head ].down = !digitalReadDirect( pin_ ); // active low
   queue_[ head ].atMillis = millis();
   queueCount_++;
}

// pop the oldest queued edge; caller must ensure queueCount_ > 0 first.
// interrupts are briefly (a few instructions) disabled around the read/index
// update so a same-moment ISR firing can't tear it -- unrelated to, and much
// shorter than, FastLED's own disabled-interrupt window.
Edge pop() {
   noInterrupts();
   Edge e; // manual field copy -- volatile source has no implicit copy ctor
   e.down = queue_[ queueTail_ ].down;
   e.atMillis = queue_[ queueTail_ ].atMillis;
   queueTail_ = ( queueTail_ + 1 ) % QUEUE_SIZE;
   queueCount_--;
   interrupts();
   return e;
}

} // end namespace PushButtonEdge

class PushButton {
 public:
   void setup( uint8_t pin ) {
      pin_ = pin;
      pinMode( pin_, INPUT );
      digitalWriteDirect( pin_, LOW );
      down_ = false;
      PushButtonEdge::pin_ = pin;
      attachInterrupt( digitalPinToInterrupt( pin ), PushButtonEdge::isr, CHANGE );
   }

   // call once per main loop iteration to refresh press state
   void update() {
      // replay every edge captured since the last call, oldest first, each
      // using its own recorded timestamp -- see the class comment above for
      // why this (rather than a live digitalRead) is what actually catches
      // presses the main loop would otherwise have been too busy to see.
      while ( PushButtonEdge::queueCount_ > 0 ) {
         PushButtonEdge::Edge e = PushButtonEdge::pop();
         processEdge( e.down, e.atMillis );
      }

      // no more historical edges -- if still held, check live threshold
      // crossings against the current time (unchanged from before). NOTE:
      // this and secondPressWindowStartMillis_ below deliberately stay raw
      // millis() math rather than Timer -- both compare against a timestamp
      // that can come from a REPLAYED historical edge (PushButtonEdge::Edge::
      // atMillis, possibly several ms in the past by the time it's
      // processed), not "now". Timer only models "elapsed since I was last
      // reset to now", so it can't represent that.
      if ( down_ && !liveActionFired_ ) {
         uint32_t heldMillis = millis() - pressStartMillis_;
         if ( !mediumThresholdFired_ && heldMillis >= mediumPressMillis_ ) {
            mediumThresholdFired_ = true;
            crossedMediumFlag_ = true;
         }
         if ( !longThresholdFired_ && heldMillis >= longPressMillis_ ) {
            longThresholdFired_ = true;
            crossedLongFlag_ = true;
         }
         if ( heldMillis >= extraLongPressMillis_ ) {
            liveActionFired_ = true;
            action_ = PressExtraLong;
         }
      }

      // resolve a pending short press once its watch window elapses with no
      // second press having shown up
      if ( awaitingSecondPress_ && millis() - secondPressWindowStartMillis_ >= doublePressWindowMillis_ ) {
         awaitingSecondPress_ = false;
         action_ = PressShort;
      }
   }

   bool isDown() {
      return down_;
   }

   // returns true once per short press (delayed by the double-press watch
   // window); clears the flag unless autoreset is false
   bool shortpress( bool autoreset = true ) {
      bool ret = ( action_ == PressShort );
      if ( ret && autoreset ) action_ = PressNone;
      return ret;
   }

   // returns true once per medium press; clears the flag unless autoreset is false
   bool mediumpress( bool autoreset = true ) {
      bool ret = ( action_ == PressMedium );
      if ( ret && autoreset ) action_ = PressNone;
      return ret;
   }

   // returns true once per long press; clears the flag unless autoreset is false
   bool longpress( bool autoreset = true ) {
      bool ret = ( action_ == PressLong );
      if ( ret && autoreset ) action_ = PressNone;
      return ret;
   }

   // returns true once per extra-long press (valid while still held); clears
   // the flag unless autoreset is false
   bool extralongpress( bool autoreset = true ) {
      bool ret = ( action_ == PressExtraLong );
      if ( ret && autoreset ) action_ = PressNone;
      return ret;
   }

   // returns true once per double press (valid while the second press is
   // still held); clears the flag unless autoreset is false
   bool doublepress( bool autoreset = true ) {
      bool ret = ( action_ == PressDouble );
      if ( ret && autoreset ) action_ = PressNone;
      return ret;
   }

   // one-shot: true for a single update() call, right as a held press crosses
   // the medium-press threshold. meant for live UI feedback (e.g. a flash).
   bool crossedMediumThreshold( bool autoreset = true ) {
      bool ret = crossedMediumFlag_;
      if ( ret && autoreset ) crossedMediumFlag_ = false;
      return ret;
   }

   // one-shot: true for a single update() call, right as a held press crosses
   // the long-press threshold. meant for live UI feedback (e.g. a flash).
   bool crossedLongThreshold( bool autoreset = true ) {
      bool ret = crossedLongFlag_;
      if ( ret && autoreset ) crossedLongFlag_ = false;
      return ret;
   }

   void setMediumPressMillis( uint32_t ms ) {
      mediumPressMillis_ = ms;
   }

   void setLongPressMillis( uint32_t ms ) {
      longPressMillis_ = ms;
   }

   void setExtraLongPressMillis( uint32_t ms ) {
      extraLongPressMillis_ = ms;
   }

   void setDoublePressWindowMillis( uint32_t ms ) {
      doublePressWindowMillis_ = ms;
   }

 private:
   // applies one historical edge (from the interrupt queue) using its own
   // recorded timestamp -- same classification logic the old polled version
   // had, just data-driven instead of sampled live, so it's correct even
   // when replaying a press+release that completed entirely between
   // update() calls.
   void processEdge( bool nowDown, uint32_t atMillis ) {
      if ( nowDown && !down_ ) {
         // this press just started -- reset all per-press live-fire tracking
         pressStartMillis_ = atMillis;
         mediumThresholdFired_ = false;
         longThresholdFired_ = false;
         liveActionFired_ = false;

         if ( awaitingSecondPress_ && ( atMillis - secondPressWindowStartMillis_ ) <= doublePressWindowMillis_ ) {
            // this is press #2 of a potential double -- valid immediately
            awaitingSecondPress_ = false;
            action_ = PressDouble;
            liveActionFired_ = true;
         }
      } else if ( !nowDown && down_ ) {
         // just released -- classify by how long it lasted, unless a live
         // event (extra-long or double) already locked in this hold
         if ( !liveActionFired_ ) {
            classifyOnRelease( atMillis - pressStartMillis_, atMillis );
         }
      }
      down_ = nowDown;
   }

   void classifyOnRelease( uint32_t heldMillis, uint32_t atMillis ) {
      if ( heldMillis >= extraLongPressMillis_ ) {
         action_ = PressExtraLong; // safety net: covers a release polled in the
                                    // same update() call that crossed 3s
      } else if ( heldMillis >= longPressMillis_ ) {
         action_ = PressLong;
      } else if ( heldMillis >= mediumPressMillis_ ) {
         action_ = PressMedium;
      } else {
         // short -- but don't report it yet; wait to see if a second press
         // follows within the double-press window
         awaitingSecondPress_ = true;
         secondPressWindowStartMillis_ = atMillis;
      }
   }

   uint8_t pin_;
   bool down_ = false;
   int action_ = PressNone;
   uint32_t pressStartMillis_ = 0;
   bool mediumThresholdFired_ = false;
   bool longThresholdFired_ = false;
   bool crossedMediumFlag_ = false;
   bool crossedLongFlag_ = false;
   bool awaitingSecondPress_ = false; // watching for a second press after a short release
   bool liveActionFired_ = false;     // extra-long or double already fired live this hold --
                                       // locks out every other press event until release
   uint32_t secondPressWindowStartMillis_ = 0;
   uint32_t mediumPressMillis_ = 300;       // held at least this long (ms) -> medium press
   uint32_t longPressMillis_ = 1500;        // held at least this long (ms) -> long press
   uint32_t extraLongPressMillis_ = 3000;   // held at least this long (ms) -> extra-long press
   uint32_t doublePressWindowMillis_ = 250; // a second press must start within this long of release
};

namespace EncoderImpl {

uint8_t pinA_;
uint8_t pinB_;
int8_t pos_;
int a_;
int b_;

void updatePosition() {
   if ( a_ ^ b_ ) {
      ++pos_;
   } else {
      --pos_;
   }
}

void resetPosition() {
   pos_ = 0;
}

void callbackA() {
   a_ = digitalReadDirect( pinA_ );
   updatePosition();
}

void callbackB() {
   b_ = digitalReadDirect( pinB_ );
}

} // end namespace EncoderImpl

class Encoder {
   friend Encoder * getEncoder( uint8_t pinA, uint8_t pinB, uint8_t pinSw );
 public:
   PushButton button; // the encoder's own integrated shaft-click switch

   int readPositionDelta() {
      return EncoderImpl::pos_;
   }

   void resetPositionDelta() {
      EncoderImpl::resetPosition();
   }

   // call once per main loop iteration to refresh button press state.
   // rotation is handled by the A/B interrupts and needs no polling.
   void update() {
      button.update();
   }

 private:
   Encoder() {}; // define constructor as private to force factory to be used
};

Encoder * getEncoder( uint8_t pinA, uint8_t pinB, uint8_t pinSw ) {
   static Encoder enc;
   static bool first = true;
   if ( first ) {
      EncoderImpl::pinA_ = pinA;
      EncoderImpl::pinB_ = pinB;
      EncoderImpl::pos_ = 0;
      pinMode( EncoderImpl::pinA_, INPUT );
      pinMode( EncoderImpl::pinB_, INPUT );
      uint8_t interruptA = digitalPinToInterrupt( pinA );
      uint8_t interruptB = digitalPinToInterrupt( pinB );
      attachInterrupt( interruptA, EncoderImpl::callbackA, CHANGE );
      attachInterrupt( interruptB, EncoderImpl::callbackB, CHANGE );
      EncoderImpl::a_ = digitalReadDirect( EncoderImpl::pinA_ );
      EncoderImpl::b_ = digitalReadDirect( EncoderImpl::pinB_ );
      enc.button.setup( pinSw );
      first = false;
   }
   return &enc;
}

} // end namespace LedControl

#endif
