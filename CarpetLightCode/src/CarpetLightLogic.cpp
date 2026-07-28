/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: July 2017
 */
#include "Utilities.h"
#include "MagicCarpet.h"

#include "AudioBoard.h"

// light shows
#include "DemoShow.h"
#include "FlameShow.h"
#include "NightriderShow.h"
#include "BumpingAudioShow.h"

// The Flying Magic Carpet (TM)
MagicCarpet * carpet;

LightShow * currLightShow;

static const uint8_t numModes = 3;
static uint8_t currMode = 0;
static uint8_t prevMode = currMode;
static uint8_t currVariation[ numModes ] = { 0, 0, 0 };
static bool lightsOn = true; // always boots on; toggled by an extra-long press

LightShow * makeShow( uint8_t mode, uint8_t variation ) {
   switch ( mode ) {
      case 0:
         return new NightriderShow( carpet, variation );
      case 1:
         return new FlameShow( carpet, variation );
      case 2:
         return new EqualizerShow( carpet );
      default:
         // we fucked up, just reset
         // carpet->error(); // uncomment for debugging
         return new NightriderShow( carpet, variation );
   }
}

void setup() {
   Serial.begin(9600);
   // AudioBoard::setup();

   // setup the carpet
   carpet = theMagicCarpet();
   carpet->setup(); // also loads persisted show/variation state from flash (Nvm::load())

   currMode = Nvm::loadedShow();
   if ( currMode >= numModes ) currMode = 0; // guard against stale/out-of-range flash data
   for ( uint8_t i = 0; i < numModes; ++i ) {
      currVariation[ i ] = Nvm::loadedVariation( i );
   }
   prevMode = currMode;

   currLightShow = makeShow( currMode, currVariation[ currMode ] );
   currLightShow->start();
}

void loop() {
   static uint32_t clock;

   /*Serial.print("pot: ");
   Serial.print( carpet->pot->read() );
   Serial.print(" encoder a: ");
   Serial.print(digitalReadDirect(ENCODER_A_PIN));
   Serial.print(" encoder b: ");
   Serial.print(digitalReadDirect(ENCODER_B_PIN));
   Serial.print(" switch: ");
   Serial.print(digitalReadDirect(BUTTON_PIN)); 
   Serial.print("\n");
   Serial.flush();*/

   static int last = millis();
   bool should_print = millis() - last > 400;
   clock = millis();

   AudioBoard::pollFrequencies( clock );

   carpet->encoder->update(); // refresh button short/medium/long/extra-long press state

   // press-hold feedback: flash the perimeter as thresholds are crossed, live
   if ( carpet->encoder->button.crossedMediumThreshold() ) {
      carpet->flashRope( 1 );
   }
   if ( carpet->encoder->button.crossedLongThreshold() ) {
      carpet->flashRope( 2 );
   }

   // extra-long press toggles the whole rig on/off; valid the instant it's crossed
   if ( carpet->encoder->button.extralongpress() ) {
      lightsOn = !lightsOn;
   }

   if ( false && should_print ) {
      last = millis();
      Serial.println("millis");
      Serial.println( clock );
      Serial.println("output");
      Serial.println( AudioBoard::bin_low );
      Serial.println( AudioBoard::bin_mid );
      Serial.println( AudioBoard::bin_high );
      Serial.println("input");
      Serial.println( AudioBoard::Frequencies_Mono[0] );
      Serial.println( AudioBoard::Frequencies_Mono[1] );
      Serial.println( AudioBoard::Frequencies_Mono[2] );
      Serial.println( AudioBoard::Frequencies_Mono[3] );
      Serial.println( AudioBoard::Frequencies_Mono[4] );
      Serial.println( AudioBoard::Frequencies_Mono[5] );
      Serial.println( AudioBoard::Frequencies_Mono[6] );
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

   // cycle light shows on each short press of the button
   if ( carpet->encoder->button.shortpress() ) {
      currMode = ( currMode + 1 ) % numModes;
      Nvm::saveShow( currMode );
   }
   Serial.print( "currMode " );
   Serial.println( currMode );
   if ( currMode != prevMode ) {
      delete currLightShow;
      currLightShow = makeShow( currMode, currVariation[ currMode ] );
      currLightShow->start();
      prevMode = currMode;
   }

   currLightShow->update( clock );

   // persist a show's variation the moment it actually changes
   uint8_t variation = currLightShow->variation();
   if ( variation != currVariation[ currMode ] ) {
      currVariation[ currMode ] = variation;
      Nvm::saveVariation( currMode, variation );
   }

   if ( !lightsOn ) carpet->clear(); // master override -- still let shows run "invisibly" underneath

   carpet->show();
}
