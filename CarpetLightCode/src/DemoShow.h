#include "LightShow.h"

class DemoShow : public LightShow {
 public:
   DemoShow( MagicCarpet * carpetArg ) : LightShow( carpetArg ) {}

   void start() {
      for ( int i = NUM_MEGABAR_LEDS - 1; i >= 0; --i ) {
         carpet->megabarLeds[ i ] = CRGB::Blue;
         carpet->chinaLeds[ i ] = CRGB::Green;
      }
      while (true) {
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
           carpet->ropeLeds[i] = CRGB::Red;
         }
         carpet->show();
         FastLED.delay( 1000 );
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
           carpet->ropeLeds[i] = CRGB::Blue;
         }
         carpet->show();
         FastLED.delay( 1000 );
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
           carpet->ropeLeds[i] = CRGB::Green;
         }
         carpet->show();
         FastLED.delay( 1000 );
      }
   }

   void update( uint32_t timestamp ) {
      static uint32_t ropeTime = 0;
      static uint32_t dmxTime = 0;
      static int loc = 0;
      static int end = NUM_NEO_LEDS;
      static CRGB clr = CRGB::Green;
      static CRGB oldclr = CRGB::Red;

      // update the rope every 100ms
      if ( true || timestamp - ropeTime > 200 ) {
         ropeTime = timestamp;
         if ( end == 0 ) {
           end = NUM_NEO_LEDS;
           oldclr = clr;
           clr = CHSV( random8(), 255, 255 );
           return;
         }
         for ( int i = 0; i < NUM_NEO_LEDS; ++i ) {
           if ( i == loc ) {
             carpet->ropeLeds[i] = clr;
             break;
           } else {
             carpet->ropeLeds[i] = oldclr;
           }
         }
         ++loc;
         loc %= end;
         if ( loc == 0 ) {
           --end;
         }
      }

      static const CRGBPalette16 dmxPalette( CRGB::Black, CRGB::Red,
                                             CRGB::Yellow, CRGB::White );
                                             // CRGB::White, CRGB::Yellow,
                                             // CRGB::Red, CRGB::Black );

      static const int rate = 10000; // millis
      static uint8_t pos = 0;
      uint32_t diff = timestamp - dmxTime;
      if ( diff > rate ) {
         dmxTime = timestamp;
         diff = 0;
         ++pos;
      }

      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
         int index = scaleTo255( i + pos, NUM_MEGABAR_LEDS - 1, 0 );
         int diffIndex = scaleTo255( diff, rate, 0 );
         carpet->megabarLeds[ i ] = CHSV( index + diffIndex, 255, 255 );
      }
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         int index = scaleTo255( i, NUM_CHINA_LEDS - 1, 0 );
         int diffIndex = scaleTo255( diff, rate, 0 );
         carpet->chinaLeds[ i ] = CHSV( index + diffIndex, 255, 255 );
      }
      // for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
      //    int index = scaleTo255( i + pos, NUM_MEGABAR_LEDS - 1, 0 );
      //    int diffIndex = scaleTo255( diff, rate, 0 );
      //    CRGB clr1 = ColorFromPalette( dmxPalette, index );
      //    CRGB clr2 = ColorFromPalette( dmxPalette, index + 1 );
      //    carpet->megabarLeds[ i ] = blend( clr1, clr2, diffIndex );
      // }
      // for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
      //    int index = scaleTo255( i, NUM_CHINA_LEDS - 1, 0 );
      //    int diffIndex = scaleTo255( diff, rate, 0 );
      //    CRGB clr1 = ColorFromPalette( dmxPalette, index );
      //    CRGB clr2 = ColorFromPalette( dmxPalette, index + 1 );
      //    carpet->chinaLeds[ i ] = blend( clr1, clr2, diffIndex );
      // }
   }
};
