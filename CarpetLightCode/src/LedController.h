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
   uint16_t read() {
      return analogRead( pin_ );
   }
 private:
   uint8_t pin_;
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
class PushButton {
 public:
   void setup( uint8_t pin ) {
      pin_ = pin;
      pinMode( pin_, INPUT );
      digitalWriteDirect( pin_, LOW );
      down_ = false;
   }

   // call once per main loop iteration to refresh press state
   void update() {
      bool nowDown = !digitalReadDirect( pin_ ); // active low
      if ( nowDown ) {
         if ( !down_ ) {
            // this press just started -- reset all per-press live-fire tracking
            pressTimer_.reset();
            mediumThresholdFired_ = false;
            longThresholdFired_ = false;
            liveActionFired_ = false;

            if ( awaitingSecondPress_ && secondPressWindowTimer_.elapsed() <= doublePressWindowMillis_ ) {
               // this is press #2 of a potential double -- valid immediately
               awaitingSecondPress_ = false;
               action_ = PressDouble;
               liveActionFired_ = true;
            }
         } else if ( !liveActionFired_ ) {
            // still held, and nothing has locked in yet for this hold --
            // live one-shot threshold crossings, for UI feedback and for
            // extra-long (valid without waiting for release)
            uint32_t heldMillis = pressTimer_.elapsed();
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
      } else if ( down_ ) {
         // just released -- classify by how long it lasted, unless a live
         // event (extra-long or double) already locked in this hold
         if ( !liveActionFired_ ) {
            classifyOnRelease( pressTimer_.elapsed() );
         }
      }
      down_ = nowDown;

      // resolve a pending short press once its watch window elapses with no
      // second press having shown up
      if ( awaitingSecondPress_ && secondPressWindowTimer_.elapsed() >= doublePressWindowMillis_ ) {
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
   void classifyOnRelease( uint32_t heldMillis ) {
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
         secondPressWindowTimer_.reset();
      }
   }

   uint8_t pin_;
   bool down_ = false;
   int action_ = PressNone;
   Timer pressTimer_;
   bool mediumThresholdFired_ = false;
   bool longThresholdFired_ = false;
   bool crossedMediumFlag_ = false;
   bool crossedLongFlag_ = false;
   bool awaitingSecondPress_ = false; // watching for a second press after a short release
   bool liveActionFired_ = false;     // extra-long or double already fired live this hold --
                                       // locks out every other press event until release
   Timer secondPressWindowTimer_;
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
