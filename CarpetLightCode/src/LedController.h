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
      return ( MAX_VOLTAGE - analogRead( pin_ ) );
   }
 private:
   uint8_t pin_;
};

enum ButtonPress { PressNone = 0, PressShort = 1, PressLong = 2, PressExtraLong = 3 };

// Polled, not interrupt-driven: press timing thresholds are coarse (hundreds of
// ms), so a per-loop update() is plenty responsive. The press is classified only
// once, at release, by how long the timer had been running -- nothing fires
// while the button is still held down.
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
      if ( nowDown && !down_ ) {
         pressTimer_.reset(); // press just started
      } else if ( !nowDown && down_ ) {
         // just released -- classify the whole press by how long it lasted
         uint32_t heldMillis = pressTimer_.elapsed();
         if ( heldMillis >= extraLongPressMillis_ ) {
            action_ = PressExtraLong;
         } else if ( heldMillis >= longPressMillis_ ) {
            action_ = PressLong;
         } else {
            action_ = PressShort;
         }
      }
      down_ = nowDown;
   }

   bool isDown() {
      return down_;
   }

   // returns true once per short press; clears the flag unless autoreset is false
   bool shortpress( bool autoreset = true ) {
      bool ret = ( action_ == PressShort );
      if ( ret && autoreset ) action_ = PressNone;
      return ret;
   }

   // returns true once per long press; clears the flag unless autoreset is false
   bool longpress( bool autoreset = true ) {
      bool ret = ( action_ == PressLong );
      if ( ret && autoreset ) action_ = PressNone;
      return ret;
   }

   // returns true once per extra-long press; clears the flag unless autoreset is false
   bool extralongpress( bool autoreset = true ) {
      bool ret = ( action_ == PressExtraLong );
      if ( ret && autoreset ) action_ = PressNone;
      return ret;
   }

   void setLongPressMillis( uint32_t ms ) {
      longPressMillis_ = ms;
   }

   void setExtraLongPressMillis( uint32_t ms ) {
      extraLongPressMillis_ = ms;
   }

 private:
   uint8_t pin_;
   bool down_ = false;
   int action_ = PressNone;
   Timer pressTimer_;
   uint32_t longPressMillis_ = 300;       // held at least this long (ms) -> long press
   uint32_t extraLongPressMillis_ = 1500; // held at least this long (ms) -> extra-long press
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
