/* LedController.h
 *
 *    Defines functions and interrupts for controller knobs.
 *
 *    Author: Anders Linn
 *    Date: July 2017
 */

#ifndef __LED_CONTROLLER_H
#define __LED_CONTROLLER_H

// max voltage from an analog input pin
#define MAX_VOLTAGE 1023

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

namespace PushButtonImpl {

static bool down_;
static uint32_t lastChangeTimestamp_;

static void _lowCallback() {
   down_ = true;
   lastChangeTimestamp_ = millis();
}

static void _highCallback() {
   down_ = false;
   lastChangeTimestamp_ = millis();
}

} // end namespace PushButtonImpl

class PushButton {
   friend PushButton & getPushButton( uint8_t pin );
 public:
   bool isDown() {
      return PushButtonImpl::down_;
   }

   uint32_t durationInMillis() {
      return millis() - PushButtonImpl::lastChangeTimestamp_;
   }
 private:
   PushButton() {}; // define constructor as private to force factory to be used
};

static PushButton & getPushButton( uint8_t pin ) {
   static PushButton pb;
   static bool first = true;
   if ( first ) {
      PushButtonImpl::down_ = false;
      PushButtonImpl::lastChangeTimestamp_ = millis();
      pinMode( pin, INPUT_PULLUP ); // TODO: really a pullup?
      uint8_t interrupt = digitalPinToInterrupt( pin );
      attachInterrupt( interrupt, PushButtonImpl::_lowCallback, FALLING );
      attachInterrupt( interrupt, PushButtonImpl::_highCallback, RISING );
      first = false;
   }
   return pb;
}

namespace EncoderImpl {

static uint8_t pinA_;
static uint8_t pinB_;
static int pos_;
static uint8_t a_;
static uint8_t b_;

static void updatePosition() {
   if ( a_ ^ b_ ) {
      ++pos_;
   } else {
      --pos_;
   }
}

static void callbackA() {
   a_ = digitalRead( pinA_ );
}

static void callbackB() {
   b_ = digitalRead( pinB_ );
}

} // end namespace EncoderImpl

class Encoder {
   friend Encoder & getEncoder( uint8_t pinA, uint8_t pinB );
 public:
   int readPositionDelta() {
      int tmp = EncoderImpl::pos_;
      EncoderImpl::pos_ = 0;
      return tmp;
   }

 private:
   Encoder() {}; // define constructor as private to force factory to be used
};

static Encoder & getEncoder( uint8_t pinA, uint8_t pinB ) {
   static Encoder enc;
   static bool first = true;
   if ( first ) {
      EncoderImpl::pinA_ = pinA;
      EncoderImpl::pinB_ = pinB;
      EncoderImpl::pos_ = 0;
      pinMode( EncoderImpl::pinA_, INPUT_PULLUP );
      pinMode( EncoderImpl::pinB_, INPUT_PULLUP );
      uint8_t interruptA = digitalPinToInterrupt( pinA );
      uint8_t interruptB = digitalPinToInterrupt( pinB );
      attachInterrupt( interruptA, EncoderImpl::callbackA, CHANGE );
      attachInterrupt( interruptB, EncoderImpl::callbackB, CHANGE );
      EncoderImpl::a_ = digitalRead( EncoderImpl::pinA_ );
      EncoderImpl::b_ = digitalRead( EncoderImpl::pinB_ );
      first = false;
   }
   return enc;
}

} // end namespace LedControl

#endif
