/* MagicCarpet.h
 *
 *    This is a data abstraction of the carpet.
 *
 *    TODO: we might want to add fade-in functionality here, so that instead of
 *          changing directly to the next desired color we fade from the previous
 *          one. Right now, any such fading has to be written into the light shows.
 *          If we do this, we should add a toggle to allow light shows to disable
 *          automatic fades if they want to.
 *    TODO: total power consumption throttle
 *
 *    Author: Anders Linn
 *    Date: June 2017
 */

#ifndef __MAGIC_CARPET_H
#define __MAGIC_CARPET_H

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "LedController.h"
#include "AudioBoard.h"
#include "ArmDmx.h"
#include "Nvm.h"
#include "SpeedLink.h"

// Controller constants
#define POT_ANALOG_PIN 3
#define ENCODER_SW_PIN 3 // FIXME: same pin number as POT_ANALOG_PIN, likely a wiring/config bug
#define ENCODER_A_PIN 5
#define ENCODER_B_PIN 4

// DMX constants
#define NUM_MEGABAR_LEDS 12
#define NUM_CHINA_LEDS 8
#define NUM_DMX_LEDS NUM_MEGABAR_LEDS + NUM_CHINA_LEDS
#define SIZEOF_MEGABAR_LEDS ( NUM_MEGABAR_LEDS * sizeof( CRGB ) )
#define SIZEOF_CHINA_LEDS ( NUM_CHINA_LEDS * sizeof( CRGBWUA ) )
#define TOTAL_DMX_SIZE ( SIZEOF_MEGABAR_LEDS + SIZEOF_CHINA_LEDS )

// Neopixel constants
// TODO: convert the user-facing values to constexpr global vars
//
// NUM_NEOPIXEL_STRIPS/NEO_PIN4-7 intentionally stay at 8 below even though
// only 4 rope strips physically exist -- do NOT "fix" this to 4, it'll
// silently corrupt memory. See README.md, "Perimeter rope lights".
#define SIZEOF_SMALL_NEO 156 // 108
#define SIZEOF_LARGE_NEO 352 // 145
#define SIZEOF_SMALL_NEO_HALF 78
#define SIZEOF_LARGE_NEO_HALF 176
#define SIZEOF_LARGE_NEO_CORNER 33
#define NUM_NEOPIXEL_STRIPS 8
#define NUM_NEO_LEDS_ACTUAL ((SIZEOF_SMALL_NEO * 2) + (SIZEOF_LARGE_NEO * 2))
#define NUM_NEO_LEDS_PER_STRIP LedUtil::resizeCRGBW( SIZEOF_LARGE_NEO )
#define NUM_NEO_SHOW_LEDS ( NUM_NEO_LEDS_PER_STRIP * NUM_NEOPIXEL_STRIPS )
#define SIZEOF_NEO_STRIP ( NUM_NEO_LEDS_PER_STRIP * sizeof( CRGB ) )
#define SIZEOF_NEO_LEDS ( NUM_NEO_LEDS_ACTUAL * sizeof( CRGBW ) )
#define SIZEOF_NEO_SHOW_LEDS ( NUM_NEOPIXEL_STRIPS * SIZEOF_NEO_STRIP )
#define NEO0_OFFSET 0 // small
#define NEO1_OFFSET SIZEOF_SMALL_NEO // large
#define NEO2_OFFSET ( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO ) // large
#define NEO3_OFFSET ( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO * 2 ) // small
#define NEO_PORT_BANK ( WS2811_PORTD )
#define NEO_PIN0 25
#define NEO_PIN1 26
#define NEO_PIN2 27
#define NEO_PIN3 28
#define NEO_PIN4 14
#define NEO_PIN5 15
#define NEO_PIN6 29
#define NEO_PIN7 11

/* Positional Constants
 *
 * These are aliases for the index values of the carpet's led arrays. Their are two
 * different types defined: directional and temporal. Both led array start from the
 * midpoint of the front of the carpet (FRONT or TWELVE). So far these are defined
 * only for the rope light arrays, but we can do this for the dmx leds too if we
 * feel the need.
 */
#define FRONT (SIZEOF_SMALL_NEO_HALF)
#define FRONT_RIGHT (SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO_CORNER)
#define RIGHT (SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO_HALF)
#define BACK_RIGHT (SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO - SIZEOF_LARGE_NEO_CORNER)
#define BACK (SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO + SIZEOF_SMALL_NEO_HALF)
#define BACK_LEFT ((SIZEOF_SMALL_NEO * 2) + SIZEOF_LARGE_NEO + SIZEOF_LARGE_NEO_CORNER)
#define LEFT ((SIZEOF_SMALL_NEO * 2) + SIZEOF_LARGE_NEO + SIZEOF_LARGE_NEO_HALF)
#define FRONT_LEFT ((SIZEOF_SMALL_NEO * 2) + (SIZEOF_LARGE_NEO * 2) - SIZEOF_LARGE_NEO_CORNER)

#define TWELVE FRONT
#define ONE_THIRTY FRONT_RIGHT
#define THREE RIGHT
#define FOUR_THIRTY BACK_RIGHT
#define SIX BACK
#define SEVEN_THIRTY BACK_LEFT
#define NINE LEFT
#define TEN_THIRTY FRONT_LEFT

// index of the center-front megabar pair (addr 01, two physical fixtures,
// same DMX address) -- see README.md, "Megabars"
#define HEADLIGHT_INDEX 0

class MagicCarpet {
 private:

   /* FastLED doesn't support rgbw leds. We work around this by offsetting the color
    * values to accomodate the white value. See CRGBW.h for more details.
    */
   CRGB ropeShowLeds[ NUM_NEO_SHOW_LEDS ];

   // see README.md, "Brightness system"
   float globalBrightness_ = 100.0f;   // 0-100
   float headlightBrightness_ = 50.0f; // 50-100, per-fixture (see README -- default
                                        // 50% per fixture makes the combined pair
                                        // match one normal megabar's brightness)
   float chinaBrightness_ = 100.0f;    // 0-100

   // converts a percentage (0-100) into a raw linear-domain 0-255 value.
   // the only place a percent should ever become a raw hardware value.
   static uint8_t percentToRaw( float percent ) {
      percent = constrain( percent, 0.0f, 100.0f );
      return (uint8_t)( ( percent / 100.0f ) * 255.0f + 0.5f );
   }

   // like delay(), but keeps the button's state machine alive throughout --
   // a plain delay() goes totally blind to button input for its duration,
   // which for flashRope()'s multi-flash sequences (up to ~460ms) is long
   // enough to drop part of a rapid follow-up double-press.
   void delayPolling( uint32_t ms ) {
      uint32_t start = millis();
      do {
         encoder->button.update();
         delay( 1 );
      } while ( millis() - start < ms );
   }

 public:

   /* DON'T CHANGE THE ORDER OF THESE ARRAYS!
    * They're declared separately to make them easy to work with, but treated as a
    * single continguous array when passed into dmx_send.
    */
   CRGB megabarLeds[ NUM_MEGABAR_LEDS ];
   CRGBWUA chinaLeds[ NUM_CHINA_LEDS ];

   // neopixel leds
   CRGBW ropeLeds[ NUM_NEO_LEDS_ACTUAL ];

   // controls
   LedControl::Potentiometer * pot;
   LedControl::Encoder * encoder; // includes the encoder's integrated shaft-click button (encoder->button)

   // brightness, in percent -- see the comment on the private fields above
   void setGlobalBrightness( float percent ) {
      globalBrightness_ = constrain( percent, 0.0f, 100.0f );
   }
   float getGlobalBrightness() {
      return globalBrightness_;
   }

   void setHeadlightBrightness( float percent ) {
      headlightBrightness_ = constrain( percent, 50.0f, 100.0f );
   }
   float getHeadlightBrightness() {
      return headlightBrightness_;
   }

   void setChinaBrightness( float percent ) {
      chinaBrightness_ = constrain( percent, 0.0f, 100.0f );
   }
   float getChinaBrightness() {
      return chinaBrightness_;
   }

   void setup() {
      // seed random so we always get different random patterns
      randomSeed( analogRead( 0 ) );

      Nvm::load(); // recall persisted show/variation selection before anything else needs it

      AudioBoard::setup();
      SpeedLink::setup(); // Wire1 I2C slave, receives vehicle speed from CANTroller2

      digitalWrite( 2, HIGH );

      // set up the controller
      pot = new LedControl::Potentiometer( POT_ANALOG_PIN );
      encoder = LedControl::getEncoder( ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_SW_PIN );

      // add dmx leds
      dmx_init( TOTAL_DMX_SIZE );

      // add eight channels of rope leds
      FastLED.addLeds<NEO_PORT_BANK,NUM_NEOPIXEL_STRIPS>( ropeShowLeds,
                                                          NUM_NEO_LEDS_PER_STRIP );

      clear(); // there might be stale values left in the leds, start from scratch

      show();
   }

   void show() {
      /* the lights aren't always arranged the same way as they are addressed, so in
       * some cases we need to reverse whatever has been programmed in the user array
       * in order to keep the addressing consistent. This is done in-place.
       *
       * TODO: if we start running too slow we can look at ways to get around this
       *       reversal
       */
      LedUtil::reverse( ropeLeds, SIZEOF_SMALL_NEO );
      // LedUtil::reverse( ropeLeds + NEO2_OFFSET, SIZEOF_LARGE_NEO );
      LedUtil::reverse( ropeLeds + SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO, SIZEOF_SMALL_NEO );

      LedUtil::convertNeoArray( ropeLeds, ropeShowLeds,
                                SIZEOF_SMALL_NEO );
      LedUtil::convertNeoArray( ropeLeds + SIZEOF_SMALL_NEO,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP,
                                SIZEOF_LARGE_NEO );
      // LedUtil::convertNeoArray( ropeLeds + NEO2_OFFSET,
      //                           ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 2,
      //                           SIZEOF_LARGE_NEO );
      // LedUtil::convertNeoArray( ropeLeds + NEO3_OFFSET,
      //                           ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 3,
      //                           SIZEOF_SMALL_NEO );
      LedUtil::convertNeoArray( ropeLeds + SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 4,
                                SIZEOF_SMALL_NEO );
      LedUtil::convertNeoArray( ropeLeds + (SIZEOF_SMALL_NEO * 2) + SIZEOF_LARGE_NEO,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 5,
                                SIZEOF_LARGE_NEO );

      // make sure to reverse the values so the user has a consistent view
      LedUtil::reverse( ropeLeds, SIZEOF_SMALL_NEO );
      LedUtil::reverse( ropeLeds + SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO, SIZEOF_SMALL_NEO );

      // we don't have to pass the china light array separately. Instead, we treat
      // both arrays as a single big array, since they're contiguous in memory.
      dmx_send( ( uint8_t * ) megabarLeds );

      FastLED.show();
   }

   void clearMegabars() {
      memset( megabarLeds, 0, SIZEOF_MEGABAR_LEDS );
   }

   void clearChinas() {
      memset( chinaLeds, 0, SIZEOF_CHINA_LEDS );
   }

   // china fixtures are 6-channel RGBWUA -- CRGBWUA::u (aliased "black" in
   // the union, see CRGBW.h) is the UV/blacklight channel. Full on or full
   // off, independent of whatever else china is currently showing.
   void setBlacklight( bool on ) {
      uint8_t level = on ? 255 : 0;
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) chinaLeds[ i ].u = level;
   }

   void clearRope() {
      memset( ropeLeds, 0, SIZEOF_NEO_LEDS );
      memset( ropeShowLeds, 0, SIZEOF_NEO_SHOW_LEDS );
   }

   // clears all the lights back to full black
   void clear() {
      clearMegabars();
      clearChinas();
      clearRope();
   }

   // flashes the perimeter rope LEDs white `count` times, as button-hold
   // feedback. each flash is 55ms, padded by 15ms of black before and after;
   // multiple flashes are separated by an additional 85ms. blocking, since
   // this is only ever a brief pause during a press the user is still holding.
   void flashRope( uint8_t count ) {
      for ( uint8_t i = 0; i < count; ++i ) {
         clearRope();
         show();
         delayPolling( 15 );
         for ( int j = 0; j < NUM_NEO_LEDS_ACTUAL; ++j ) ropeLeds[ j ].w = 255;
         show();
         delayPolling( 55 );
         clearRope();
         show();
         delayPolling( 15 );
         if ( i + 1 < count ) delayPolling( 85 );
      }
   }

   // renders the whole rig as solid red at the given brightness PERCENT
   // (0-100), for the global-brightness configuration preview. rope leds are
   // gamma-corrected automatically inside show(); megabar/china deliberately
   // are NOT gamma-corrected here (or anywhere else in this class) -- gamma
   // correction is a nonlinear curve, and applying it after scaling would
   // throw off the linear brightness math these previews rely on (see
   // applyBrightnessCeiling(), which is equally gamma-free for the same
   // reason). Only FlameShow gamma-corrects its own megabar output; that's a
   // pre-existing inconsistency, not something to match here.
   void showSolidRed( float percent ) {
      uint8_t linearBrightness = percentToRaw( percent );
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         ropeLeds[ i ] = CRGB::Black;
         ropeLeds[ i ].r = linearBrightness;
      }
      LedUtil::fill( megabarLeds, CRGB( linearBrightness, 0, 0 ), NUM_MEGABAR_LEDS );
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         chinaLeds[ i ] = CRGB( linearBrightness, 0, 0 );
         chinaLeds[ i ].w = 0;
      }
   }

   // preview for the headlight-brightness setting: rope and china off, all
   // other megabars shown at the (already-committed) global brightness as a
   // fixed reference, and the headlight shown at global brightness scaled by
   // liveHeadlightPercent (the not-yet-committed value being adjusted) -- so
   // the two can be visually compared side by side.
   void showHeadlightPreview( float liveHeadlightPercent ) {
      clearRope();
      clearChinas();
      uint8_t globalRaw = percentToRaw( globalBrightness_ );
      uint8_t headlightRaw = scale8( globalRaw, percentToRaw( liveHeadlightPercent ) );
      LedUtil::fill( megabarLeds, CRGB( globalRaw, 0, 0 ), NUM_MEGABAR_LEDS );
      megabarLeds[ HEADLIGHT_INDEX ] = CRGB( headlightRaw, 0, 0 );
   }

   // preview for the china-brightness setting: rope off, megabars shown at
   // their normal effective brightness (respecting the committed headlight
   // setting) as a fixed reference, and china shown at global brightness
   // scaled by liveChinaPercent (the not-yet-committed value being adjusted)
   // -- so the two can be visually compared side by side.
   void showChinaPreview( float liveChinaPercent ) {
      clearRope();
      uint8_t globalRaw = percentToRaw( globalBrightness_ );
      uint8_t headlightRaw = scale8( globalRaw, percentToRaw( headlightBrightness_ ) );
      uint8_t chinaRaw = scale8( globalRaw, percentToRaw( liveChinaPercent ) );
      LedUtil::fill( megabarLeds, CRGB( globalRaw, 0, 0 ), NUM_MEGABAR_LEDS );
      megabarLeds[ HEADLIGHT_INDEX ] = CRGB( headlightRaw, 0, 0 );
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         chinaLeds[ i ] = CRGB( chinaRaw, 0, 0 );
         chinaLeds[ i ].w = 0;
      }
   }

   // lights a 10-LED window on one side strip, sliding from backCornerIdx
   // (0%) to frontCornerIdx (100%) as percent varies, always 10 LEDs wide.
   // each lit LED's color reflects its OWN position along the whole side
   // (red at the back corner, green at the front corner), not the window's
   // position, so the lit window's hue shifts as it slides. segmentLen is
   // derived from the two corner indices passed in, not hardcoded to the
   // full 352-LED channel -- callers are expected to pass the true-corner
   // constants (FRONT_RIGHT/BACK_RIGHT, BACK_LEFT/FRONT_LEFT), which land 33
   // LEDs in from each end of the physical channel (SIZEOF_LARGE_NEO_CORNER),
   // so the window doesn't wrap into the front/back edge LEDs at either end.
   void renderSideIndicator( int backCornerIdx, int frontCornerIdx, float percent ) {
      const int segmentLen = abs( frontCornerIdx - backCornerIdx ) + 1;
      static const int windowWidth = 10;
      int direction = ( frontCornerIdx > backCornerIdx ) ? 1 : -1;
      float t = constrain( percent, 0.0f, 100.0f ) / 100.0f;
      int maxOffset = segmentLen - windowWidth;
      int windowOffset = (int)( t * maxOffset + 0.5f );
      for ( int k = 0; k < windowWidth; ++k ) {
         int localOffset = windowOffset + k;
         int idx = backCornerIdx + direction * localOffset;
         float f = (float)localOffset / (float)( segmentLen - 1 ); // 0=back(red) .. 1=front(green)
         uint8_t hue = (uint8_t)( f * 96.0f );
         ropeLeds[ idx ] = CHSV( hue, 255, 255 );
         ropeLeds[ idx ].w = 0;
      }
   }

   // audio configuration screen: a 3-band VU meter across the front rope
   // strip (ropeLeds[0..SIZEOF_SMALL_NEO-1], 156 LEDs), a 10-LED position
   // indicator sliding along each side strip showing the live noise-floor
   // percent, and all megabars glowing blue proportional to the current
   // full-spectrum audio level. China is blanked.
   //
   // The front strip splits into 3 equal 52-LED segments in array order:
   // treble, mid, bass. Each segment shows a dim (50%) green->yellow->red
   // reference gradient across its length, with a bright white fill from the
   // segment's start up to the live level's position -- at max level the
   // whole segment goes solid white. trebleLevel/midLevel/bassLevel are
   // 0-255 (AudioBoard::getHigh/getMid/getLow range).
   //
   // Each side strip, true corner to true corner (286 LEDs -- the 352-LED
   // physical channel minus 33 at each end, see renderSideIndicator()) shows
   // a 10-LED window at the position corresponding to liveNoiseFloorPercent
   // (0-100): at the back corner at 0%, sliding to the front corner at 100%.
   //
   // fullSpectrumLevel (0-255, AudioBoard::getFullSpectrum() -- already
   // silence-gated and auto-gain-scaled if enabled) sets all megabars' blue
   // brightness uniformly.
   void showAudioMeter( uint8_t trebleLevel, uint8_t midLevel, uint8_t bassLevel,
                        float liveNoiseFloorPercent, uint8_t fullSpectrumLevel ) {
      clearRope();
      clearChinas();
      static const int segmentLen = SIZEOF_SMALL_NEO / 3; // 52
      uint8_t levels[ 3 ] = { trebleLevel, midLevel, bassLevel };
      for ( int seg = 0; seg < 3; ++seg ) {
         int filledCount = ( (int)levels[ seg ] * segmentLen + 127 ) / 255; // round
         for ( int p = 0; p < segmentLen; ++p ) {
            int i = seg * segmentLen + p;
            if ( p < filledCount ) {
               ropeLeds[ i ] = CRGB::Black;
               ropeLeds[ i ].w = 255;
            } else {
               uint8_t hue = (uint8_t)( 96 - ( 96 * p ) / ( segmentLen - 1 ) ); // green(96) -> red(0)
               ropeLeds[ i ] = CHSV( hue, 255, 128 ); // 128 = 50% brightness
               ropeLeds[ i ].w = 0;
            }
         }
      }

      // right side: true corner to true corner (see renderSideIndicator's comment) --
      // this stops 33 LEDs short of each end of the 352-LED channel, so it
      // doesn't wrap into the front/back edges near the corners
      renderSideIndicator( BACK_RIGHT, FRONT_RIGHT, liveNoiseFloorPercent );
      // left side: true corner to true corner
      renderSideIndicator( BACK_LEFT, FRONT_LEFT, liveNoiseFloorPercent );

      LedUtil::fill( megabarLeds, CRGB( 0, 0, fullSpectrumLevel ), NUM_MEGABAR_LEDS );
   }

   // power-saving A/B test: front half of the rig (rope's front edge + front
   // half of both sides, china[0..3]) shows the given HSV color rendered
   // straight to RGB (w=0); the back half (rope's back edge + back half of
   // both sides, china[4..7]) shows the same color translated to RGBW by
   // moving the shared min(r,g,b) out of the color channels and into the
   // white channel -- same hue/brightness, less color-LED power. Megabars
   // are off; unlike the other config previews this one deliberately ignores
   // the committed global/headlight/china brightness ceiling (hue/sat/
   // brightness here are already a full manual color spec, not a dimming
   // level) -- see README.md.
   void showPowerTest( uint8_t hue, uint8_t sat, uint8_t val ) {
      clearMegabars();

      CRGB straightClr = CHSV( hue, sat, val );
      uint8_t minCh = min( straightClr.r, min( straightClr.g, straightClr.b ) );
      CRGBW savingClr;
      savingClr.r = straightClr.r - minCh;
      savingClr.g = straightClr.g - minCh;
      savingClr.b = straightClr.b - minCh;
      savingClr.w = minCh;

      // front half = front edge + front half of each side, forming an
      // upside-down U to the carpet's front-to-back center line
      int frontBoundaryRight = SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO_HALF;
      int frontBoundaryLeft = ( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO ) + SIZEOF_LARGE_NEO_HALF;
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         bool frontHalf = ( i < frontBoundaryRight ) || ( i >= frontBoundaryLeft );
         if ( frontHalf ) {
            ropeLeds[ i ] = straightClr;
            ropeLeds[ i ].w = 0;
         } else {
            ropeLeds[ i ] = savingClr;
         }
      }

      // china[0..3] are the front-corner pairs, china[4..7] the back-corner pairs
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         if ( i < 4 ) {
            chinaLeds[ i ] = straightClr;
            chinaLeds[ i ].w = 0;
         } else {
            chinaLeds[ i ] = savingClr;
         }
      }
   }

   // scales every light array in-place per the configured global/headlight/
   // china brightness percentages (linear domain), capping all light shows'
   // output. call once per frame, after a show writes its output and before
   // show() pushes it to hardware.
   void applyBrightnessCeiling() {
      uint8_t globalRaw = percentToRaw( globalBrightness_ );
      uint8_t headlightRaw = scale8( globalRaw, percentToRaw( headlightBrightness_ ) );
      uint8_t chinaRaw = scale8( globalRaw, percentToRaw( chinaBrightness_ ) );

      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         ropeLeds[ i ].r = scale8( ropeLeds[ i ].r, globalRaw );
         ropeLeds[ i ].g = scale8( ropeLeds[ i ].g, globalRaw );
         ropeLeds[ i ].b = scale8( ropeLeds[ i ].b, globalRaw );
         ropeLeds[ i ].w = scale8( ropeLeds[ i ].w, globalRaw );
      }
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
         uint8_t raw = ( i == HEADLIGHT_INDEX ) ? headlightRaw : globalRaw;
         megabarLeds[ i ].r = scale8( megabarLeds[ i ].r, raw );
         megabarLeds[ i ].g = scale8( megabarLeds[ i ].g, raw );
         megabarLeds[ i ].b = scale8( megabarLeds[ i ].b, raw );
      }
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         chinaLeds[ i ].r = scale8( chinaLeds[ i ].r, chinaRaw );
         chinaLeds[ i ].g = scale8( chinaLeds[ i ].g, chinaRaw );
         chinaLeds[ i ].b = scale8( chinaLeds[ i ].b, chinaRaw );
         chinaLeds[ i ].w = scale8( chinaLeds[ i ].w, chinaRaw );
      }
   }

   void error() {
      while( true ) {
         static CRGB clr = CRGB::Black;
         static CRGBWUA chinaClr = clr;
         chinaClr.a = 255;
         LedUtil::fill( ropeLeds, CRGB::Red, NUM_NEO_LEDS_ACTUAL );
         LedUtil::fill( megabarLeds, CRGB::Yellow, NUM_DMX_LEDS );
         LedUtil::fill( chinaLeds, chinaClr, NUM_CHINA_LEDS );
         FastLED.delay( 1000 );
         LedUtil::fill( ropeLeds, clr, NUM_NEO_LEDS_ACTUAL );
         LedUtil::fill( megabarLeds, clr, NUM_DMX_LEDS );
         LedUtil::fill( chinaLeds, clr, NUM_CHINA_LEDS );
         FastLED.delay( 1000 );
      }
   }
};

// singleton representing the one-and-only Flying Magic Carpet (TM)
MagicCarpet * theMagicCarpet() {
   static MagicCarpet carpet;
   return &carpet;
}

#endif
