/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: July 2017
 */

#include "MagicCarpet.h"

// light shows
#include "DemoShow.h"
#include "FlameShow.h"
#include "NightriderShow.h"

// The Flying Magic Carpet (TM)
MagicCarpet * carpet;

LightShow * currLightShow;

void setup() {
   // setup the carpet
   carpet = theMagicCarpet();
   carpet->setup();

   // TODO: i don't think i like this, lets use a static class instead.
   // currLightShow = new DemoShow( carpet );
   // currLightShow = new FlameShow( carpet );
   currLightShow = new NightriderShow( carpet );
   currLightShow->start();
   // Serial.begin(9600);
}

void loop() {
   static uint32_t clock;

   clock = millis();

   // AudioBoard::pollFrequencies( clock );

   // read encoder
   static const uint8_t numModes = 2;
   static uint8_t currMode = 0;
   static uint8_t prevMode = currMode;

   // FIXME: don't turn the encoder backwards....
   int delta = carpet->encoder->readPositionDelta() % numModes;
   carpet->encoder->resetPositionDelta();
   currMode = ( currMode + delta ) % numModes;
   // Serial.println( "currMode" );
   // Serial.println( currMode );
   // Serial.println( "delta" );
   // Serial.println( delta );
   if ( currMode != prevMode ) {
      delete currLightShow;
      // based on diff, switch between existing light shows
      switch ( currMode ) {
         case 0:
            currLightShow = new NightriderShow( carpet );
            break;
         case 1:
            currLightShow = new FlameShow( carpet );
            break;
         // case 2:
         //    currLightShow = new DemoShow( carpet );
         //    break;
         default:
            // we fucked up
            carpet->error();
      }
      currLightShow->start();
      prevMode = currMode;
   }

   currLightShow->update( clock );

   carpet->show();
}
