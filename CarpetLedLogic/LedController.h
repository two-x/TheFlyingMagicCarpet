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

namespace PushButtonImpl {

bool down_;
uint8_t buttonPin_;
uint32_t lastChangeTimestamp_;

void callback() {
   down_ = !digitalReadDirect( buttonPin_ );
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

PushButton & getPushButton( uint8_t pin ) {
   static PushButton pb;
   static bool first = true;
   if ( first ) {
      PushButtonImpl::buttonPin_ = pin;
      PushButtonImpl::down_ = true;
      PushButtonImpl::lastChangeTimestamp_ = millis();
      pinMode( pin, INPUT );
      digitalWriteDirect( pin, LOW );
      uint8_t interrupt = digitalPinToInterrupt( pin );
      attachInterrupt( interrupt, PushButtonImpl::callback, CHANGE );
      first = false;
   }
   return pb;
}

namespace EncoderImpl {

uint8_t pinA_;
uint8_t pinB_;
uint8_t pos_;
bool a_;
bool b_;

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
   static uint32_t timestamp = 0;
   uint32_t time = millis();
   if ( time - timestamp < 20 ) {
      return;
   }
   a_ = digitalReadDirect( pinA_ );
   updatePosition();
}

void callbackB() {
   static uint32_t timestamp = 0;
   uint32_t time = millis();
   if ( time - timestamp < 20 ) {
      return;
   }
   b_ = digitalReadDirect( pinB_ );
}

} // end namespace EncoderImpl

class Encoder {
   friend Encoder & getEncoder( uint8_t pinA, uint8_t pinB );
 public:
   int readPositionDelta() {
      return EncoderImpl::pos_;
   }

   void resetPositionDelta() {
      EncoderImpl::resetPosition();
   }

 private:
   Encoder() {}; // define constructor as private to force factory to be used
};

Encoder & getEncoder( uint8_t pinA, uint8_t pinB ) {
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
      EncoderImpl::a_ = digitalReadDirect( EncoderImpl::pinA_ );
      EncoderImpl::b_ = digitalReadDirect( EncoderImpl::pinB_ );
      first = false;
   }
   return enc;
}

} // end namespace LedControl

#endif
