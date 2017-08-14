/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: July 2017
 */

#include "MagicCarpet.h"

// The Flying Magic Carpet (TM)
MagicCarpet * carpet;

void setup() {
   // TODO: set up any other cards here

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
