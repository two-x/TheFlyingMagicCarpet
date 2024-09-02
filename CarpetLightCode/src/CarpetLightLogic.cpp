/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: July 2017
 */

#include "MagicCarpet.h"

#include "AudioBoard.h"

// light shows
#include "DemoShow.h"
#include "FlameShow.h"
#include "NightriderShow.h"

// The Flying Magic Carpet (TM)
MagicCarpet * carpet;

LightShow * currLightShow;

void setup() {
   Serial.begin(9600);
   // AudioBoard::setup();

   // setup the carpet
   carpet = theMagicCarpet();
   carpet->setup();

   // TODO: i don't think i like this, lets use a static class instead.
   // currLightShow = new DemoShow( carpet );

   // currLightShow = new FlameShow( carpet );
   currLightShow = new NightriderShow( carpet );

   currLightShow->start();
}

void loop() {
   static uint32_t clock;

   //Serial.println( carpet->pot->read() );

   clock = millis();

   AudioBoard::pollFrequencies( clock );

   if ( true ) {
      // Serial.println("millis");
      // Serial.println( clock );
      // Serial.println("output");
      // Serial.println( AudioBoard::bin_low );
      // Serial.println( AudioBoard::bin_mid );
      // Serial.println( AudioBoard::bin_high );
      // Serial.println("input");
      // Serial.println( AudioBoard::Frequencies_Mono[0] );
      // Serial.println( AudioBoard::Frequencies_Mono[1] );
      // Serial.println( AudioBoard::Frequencies_Mono[2] );
      // Serial.println( AudioBoard::Frequencies_Mono[4] );
      // Serial.println( AudioBoard::Frequencies_Mono[5] );
      // Serial.println( AudioBoard::Frequencies_Mono[6] );
      // Serial.println( AudioBoard::Frequencies_Mono[7] );
   }

   // CRGB clr2 = CRGB::Red;
   // CRGB clr1 = CRGB::Cyan;

   // for ( int j = NEO6_OFFSET; j < NEO6_OFFSET + 7; ++j ) {
   //    carpet->ropeLeds[ j ] = blend( clr1, clr2, scaleTo255( AudioBoard::Frequencies_Mono[j], 1024, 0 ) );
   // }
   // carpet->ropeLeds[ NEO6_OFFSET + 7 ] = blend( clr1, clr2, AudioBoard::getLow() );
   // carpet->ropeLeds[ NEO6_OFFSET + 8 ] = blend( clr1, clr2, AudioBoard::getMid() );
   // carpet->ropeLeds[ NEO6_OFFSET + 9 ] = blend( clr1, clr2, AudioBoard::getHigh() );

   // for ( int j = NEO6_OFFSET + 7; j < NEO6_OFFSET + 10; ++j ) {
   //    Serial.print( "freqcolor" );
   //    Serial.println( j );
   //    Serial.println( carpet->ropeLeds[ j ].r );
   //    Serial.println( carpet->ropeLeds[ j ].g );
   //    Serial.println( carpet->ropeLeds[ j ].b );
   // }

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
   // Serial.println( "button" );
   // Serial.println( carpet->button->isDown() );
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
            // we fucked up, just reset
            // carpet->error(); // uncomment for debugging
            currMode = 0;
            currLightShow = new NightriderShow( carpet );
      }
      currLightShow->start();
      prevMode = currMode;
   }

   currLightShow->update( clock );

   carpet->show();
}
