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

   AudioBoard::pollFrequencies( clock );

   if ( true ) {
      Serial.println("millis");
      Serial.println( clock );
      Serial.println("output");
      Serial.println( AudioBoard::bin_low );
      Serial.println( AudioBoard::bin_mid );
      Serial.println( AudioBoard::bin_high );
      Serial.println("input");
      Serial.println(AudioBoard::Frequencies_Mono[0]);
      Serial.println(AudioBoard::Frequencies_Mono[1]);
      Serial.println(AudioBoard::Frequencies_Mono[2]);
      Serial.println(AudioBoard::Frequencies_Mono[3]);
      Serial.println(AudioBoard::Frequencies_Mono[4]);
      Serial.println(AudioBoard::Frequencies_Mono[5]);
      Serial.println(AudioBoard::Frequencies_Mono[6]);
   }

   static const CRGBPalette16 clr( CRGB::Red, CRGB::Black );

   // for ( int j = 0; j < NUM_MEGABAR_LEDS; ++j ) {
   //    carpet->megabarLeds[ j ] = CRGB::Green;
   // }
   // if ( carpet->button->isDown() ) {
   //    for ( int j = 0; j < NUM_MEGABAR_LEDS; ++j ) {
   //       carpet->megabarLeds[ j ] = CRGB::Red;
   //    }
   // } else {
   //    uint16_t val = scaleTo255( carpet->pot->read(), 1024, 0 );
   //    for ( int j = 0; j < NUM_MEGABAR_LEDS; ++j ) {
   //       carpet->megabarLeds[ j ] = CHSV( val, 255, 255 );
   //    }
   // }
   for ( int j = 0; j < 7; ++j ) {
      carpet->ropeLeds[ NEO1_OFFSET - 1 - j ] = ColorFromPalette( clr, scaleTo255( AudioBoard::Frequencies_Mono[j], 1024, 0 ) );
   }
   carpet->ropeLeds[ NEO1_OFFSET - 8 ] = ColorFromPalette( clr, AudioBoard::getLow() );
   carpet->ropeLeds[ NEO1_OFFSET - 9 ] = ColorFromPalette( clr, AudioBoard::getMid() );
   carpet->ropeLeds[ NEO1_OFFSET - 10 ] = ColorFromPalette( clr, AudioBoard::getHigh() );

   // for ( int j = 0; j < NEO1_OFFSET; ++j ) {
   //    carpet->ropeLeds[ j ] = CHSV( scaleTo255( carpet->pot->read(), 1024, 0 ), 255, 255 );
   // }

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
