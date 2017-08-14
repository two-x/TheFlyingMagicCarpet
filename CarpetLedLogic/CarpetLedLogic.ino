/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   TODO: need to decide how to structure the main event loop. Could use an
 *         all-timer setup, or have the loop run continuously and check the time as
 *         it goes around each time. Maybe a combination: straight loop for
 *         handling controls, but draw everything on timers.
 *
 *   Author: Anders Linn
 *   Date: July 2017
 */

#ifndef __DMXSIMPLE_H
#define __DMXSIMPLE_H
#include <DmxSimple.h>
#endif

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "MagicCarpet.h"

// The Flying Magic Carpet (TM)
MagicCarpet * carpet;

// controls
Potentiometer * pot;
PushButton * button;
Encoder * encoder;

void setup() {
   // TODO: set up any other cards here

   // set up the controller
   pot = new Potentiometer( POT_ANALOG_PIN );
   button = getPushButton( BUTTON_PIN );
   encoder = getEncoder( ENCODER_A_PIN, ENCODER_B_PIN );

   // setup the carpet
   carpet = theMagicCarpet();
   carpet->setup();
}

void loop() {
   static uint32_t clock;
   clock = millis();

   // read encoder
   // based on diff, switch between existing light shows

   // TODO: time this!
   carpet->show();
}
