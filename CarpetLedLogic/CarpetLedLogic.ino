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

// The Flying Magic Carpet (TM)
MagicCarpet * carpet;

LightShow * currLightShow;

void setup() {
   // setup the carpet
   carpet = theMagicCarpet();
   carpet->setup();

   // TODO: i don't think i like this, lets use a static class instead.
   // currLightShow = new DemoShow( carpet );
   // currLightShow->start();
   Serial.begin(9600);
   carpet->megabarLeds[ 0 ] = CRGB::Black;
}

void loop() {
   static uint32_t clock;

   clock = millis();

   // read encoder
   // based on diff, switch between existing light shows

   // currLightShow->update( clock );

   static uint32_t ts = clock;
   if ( clock - ts > 300 ) {
      ts = clock;
      Serial.println( "enc" );
      Serial.println( carpet->encoder->readPositionDelta() );
      Serial.println( "but" );
      Serial.println( carpet->button->isDown() );
      Serial.println( "pot" );
      Serial.println( carpet->pot->read() );
   }

   carpet->show();
}
