/* NightriderShow.h
 *
 * A basic version of the Nightrider light effect, which is just a simple chase that
 * starts in the corners and bounces off the middles. Lame, but it lights up the
 * carpet and doesn't rely on the sound.
 *
 * Author: Anders Linn
 * Date: August 2017
 */

#include "LightShow.h"
#include "CarpetGeometry.h"
#include "LightSetters.h"
#include <math.h>

/*
const CRGB topC[] {
   CRGB( 0, 255, 127), // summer day
   CRGB( 178, 118, 50), // end summer day
   CRGB( 178, 102, 9), // deep dark night
   CRGB( 0, 255, 72), // desert afternoon
   CRGB( 43, 123, 93), //under the sea
   CRGB( 236, 84, 81), // beach party
   CRGB( 248, 182, 124)
};

const CRGB bottomC[] {
 CRGB( 0, 201, 100 ),
 CRGB( 254, 181, 97 ),
 CRGB( 0, 68, 101 ),
 CRGB( 76, 255, 127 ),
 CRGB( 50, 199, 143 ),
 CRGB( 182, 35, 99 ),
 CRGB( 248, 247, 5 ),
};
*/

// X-macro list: single source of truth for both the enum below and
// variationName() -- see LightShow.h's own comment on the pattern.
#define NIGHTRIDER_VARIATIONS(X) X(VarManualHue) X(VarAutoHueCycle) X(VarAutoWithSound)

class NightriderShow : public LightShow {
 private:
   enum Variation { NIGHTRIDER_VARIATIONS(LIGHTSHOW_ENUM_ENTRY) };
   static const uint8_t numVariations_ = 3;
   uint8_t variation_;

   // variations 1/2's flood sin/cos frequency is pot-driven via the shared
   // energy setting (see LightShow.h) -- replaces its OLD role of setting
   // the hue-auto-cycle period (that's now a fixed rate, see autoHue below
   // -- the pot's fully committed to flood frequency now). Variation 0's
   // pot binding (direct hue pick) is unrelated and stays a plain live
   // read, unchanged.
   PotEnergyTakeover energyTakeover_;
   SettlePrinter huePrinter_;

   // --- flood (megabar+china) behavior, ALL 3 variations -------------
   // Per request: floods split into 2 groups, "every 2nd lamp going
   // around the car" (even/odd index -- both megabars and chinas are
   // already laid out in physical order around the car by address, same
   // convention LighthouseShow/EqualizerShow's own angle-based groupings
   // rely on). Group1 brightness follows a sine wave, group2 a cosine
   // (same frequency, so they're always 90 degrees out of phase with each
   // other), both ranging 50%-100% of max (never fully dark). Variation 0
   // (manual hue) still uses this same flood system -- only ITS rope
   // travel timing and pot role are exempted (see update()) -- so its
   // floods always run at the fixed default rate below (pot's busy
   // picking hue in that variation, nothing left to adjust flood speed
   // with).
   static constexpr float FLOOD_DEFAULT_HZ = 0.5f;
   static constexpr float FLOOD_MIN_HZ = 0.25f, FLOOD_MAX_HZ = 2.0f;
   float floodHz_ = FLOOD_DEFAULT_HZ;
   float floodPhaseRad_ = 0.0f; // monotonically accumulating; sin/cos read directly off this, so a live frequency change never causes a discontinuous jump
   // group1/group2 swap which of the 2 rope colors they're currently
   // showing every 1.5x the sin/cos period, crossfading over 0.25x the
   // period on each swap -- see update()'s own math.
   bool floodGroup1IsClr1_ = true;
   float floodSwapElapsedS_ = 0.0f;
   float floodFadeElapsedS_ = 999.0f; // starts "already settled" (past any fade window)

   // --- rope traveling-segment position, auto variations only ---------
   // Per request: time between direction reversals = the flood sin/cos's
   // own period (1/floodHz_) for variations 1/2. Variation 0 (manual hue)
   // keeps its own original, unrelated timing (see update()) -- this timer
   // only accumulates for the 2 auto variations.
   float floodRopeTimeS_ = 0.0f;

 public:
   NightriderShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ) {}

   uint8_t variation() {
      return variation_;
   }
   uint8_t numVariations() { return numVariations_; }

   const char * variationName() {
      switch ( variation_ ) { NIGHTRIDER_VARIATIONS(LIGHTSHOW_VARIATION_NAME_CASE) }
      return "?";
   }
   #undef NIGHTRIDER_VARIATIONS // X-macro's job is done, keep the namespace clean

   void start() {
      for ( int i = NEO3_OFFSET; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         LightSetters::setColor( carpet, LightSetters::TargetNeo, CRGB::Black, LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      energyTakeover_.reset( carpet );
      floodPhaseRad_ = 0.0f;
      floodGroup1IsClr1_ = true;
      floodSwapElapsedS_ = 0.0f;
      floodFadeElapsedS_ = 999.0f;
      floodRopeTimeS_ = 0.0f;
   }

   void update( uint32_t time ) {
      static const CRGBPalette256 clr( CRGB::Red, CRGB::Black );
      static uint32_t timestamp = time;
      static const uint32_t rate = 300; // move the lights every 200ms
      //static const uint32_t littleRate = rate;
      //static const uint32_t bigRate = ( littleRate * SIZEOF_LARGE_NEO ) / SIZEOF_SMALL_NEO;
      static uint8_t bigPos = 0;
      static uint8_t littlePos = 0;
      static uint8_t bigPosDir = 1;
      static uint8_t littlePosDir = 1;
      static uint32_t prevUpdateMs = time;
      float dtSec = (float)( time - prevUpdateMs ) / 1000.0f;
      prevUpdateMs = time;
      if ( dtSec < 0.0f || dtSec > 1.0f ) dtSec = 0.0f; // first call / wraparound guard

      // variation select: each encoder detent moves to the next variation
      int varDelta = carpet->encoder->readPositionDelta();
      carpet->encoder->resetPositionDelta();
      if ( varDelta != 0 ) {
         int newVariation = ( (int)variation_ + varDelta ) % (int)numVariations_;
         if ( newVariation < 0 ) newVariation += numVariations_;
         variation_ = (uint8_t)newVariation;
      }

      // flood sin/cos frequency: fixed default for manual hue (pot's busy
      // picking hue there), pot-adjustable 0.25-2Hz (soft takeover, same
      // convention as every other pot-driven live setting) for the 2 auto
      // variations -- see this member's own comment.
      if ( variation_ == VarManualHue ) {
         floodHz_ = FLOOD_DEFAULT_HZ;
      } else {
         float potFrac = energyTakeover_.update( carpet ) / 100.0f; // 0..1
         floodHz_ = FLOOD_MIN_HZ + ( FLOOD_MAX_HZ - FLOOD_MIN_HZ ) * potFrac;
      }
      floodPhaseRad_ += TWO_PI * floodHz_ * dtSec;
      while ( floodPhaseRad_ >= TWO_PI * 1000.0f ) floodPhaseRad_ -= TWO_PI * 1000.0f; // keep bounded, doesn't affect sinf/cosf's own periodicity

      if ( variation_ == VarManualHue ) {
         // retained exactly as-is, per request -- this variation's rope
         // travel timing and pot role are both unrelated to the flood
         // system above.
         uint32_t diff = time - timestamp;
         if ( true || diff > rate ) {
            timestamp = time;
            diff = 0;
            if ( bigPos == SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER - 1 ) {
               bigPosDir = 0;
            } else if ( bigPos == 0 && bigPosDir == 0 ) {
               bigPosDir = 1;
            }
            if ( littlePos == SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER - 1 ) {
               littlePosDir = 0;
            } else if ( littlePos == 0 && littlePosDir == 0 ) {
               littlePosDir = 1;
            }
            if ( bigPosDir ) {
               ++bigPos;
            } else {
               --bigPos;
            }
            if ( littlePosDir ) {
               ++littlePos;
            } else {
               --littlePos;
            }
         }
      } else {
         // auto variations: time between direction reversals = the flood
         // sin/cos's own period (1/floodHz_) -- a triangle wave driven by
         // its own continuously-accumulating time (not floodPhaseRad_
         // directly, since "time between reversals" = the FULL period
         // here, not the half-period a sine's own peak-to-trough spacing
         // would give).
         floodRopeTimeS_ += dtSec;
         float reversalPeriodS = 1.0f / floodHz_;
         float roundTripS = 2.0f * reversalPeriodS;
         while ( floodRopeTimeS_ >= roundTripS ) floodRopeTimeS_ -= roundTripS;
         float phase = floodRopeTimeS_ / reversalPeriodS; // 0..2
         float triFrac = ( phase <= 1.0f ) ? phase : ( 2.0f - phase ); // 0..1..0
         bigPos = (uint8_t)( triFrac * ( SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER - 1 ) + 0.5f );
         littlePos = (uint8_t)( triFrac * ( SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER - 1 ) + 0.5f );
      }

      uint8_t val1;
      static float autoHue = 0.0f; // 0-255 continuous hue position, variations 1/2
      if ( variation_ == VarManualHue ) {
         // pot directly selects the color pair, as before
         val1 = (uint8_t)( carpet->pot->readPercent() / 100.0f * 255.0f + 0.5f );
         huePrinter_.update( val1, "hue:" );
      } else {
         // color automatically slides through the spectrum at a fixed
         // rate -- no longer pot-controlled (the pot's now dedicated to
         // flood frequency, see floodHz_ above), just a steady, reasonable
         // "auto cycle" pace.
         static constexpr float AUTO_HUE_PERIOD_S = 60.0f; // one full spectrum cycle per minute
         autoHue += ( dtSec / AUTO_HUE_PERIOD_S ) * 255.0f;
         while ( autoHue >= 255.0f ) autoHue -= 255.0f;
         val1 = (uint8_t)autoHue;
      }
      const uint8_t val2 = ( val1 + 128 ) % 255;
      const CRGB clr1 = CHSV( val1, 255, 255 );
      const CRGB clr2 = CHSV( val2, 255, 255 );

      for ( int i = FRONT; i < FRONT_RIGHT; ++i ) {
         int i_adj = i - FRONT;
         int val = scaleTo255( abs(littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      for ( int i = FRONT_RIGHT; i < RIGHT; ++i ) {
         int i_adj = i - FRONT_RIGHT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      for ( int i = RIGHT; i < REAR_RIGHT; ++i ) {
         int i_adj = i - RIGHT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      for ( int i = REAR_RIGHT; i < REAR; ++i ) {
         int i_adj = i - REAR_RIGHT;
         int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      for ( int i = REAR; i < REAR_LEFT; ++i ) {
         int i_adj = i - REAR;
         int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      for ( int i = REAR_LEFT; i < LEFT; ++i ) {
         int i_adj = i - REAR_LEFT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      for ( int i = LEFT; i < FRONT_LEFT; ++i ) {
         int i_adj = i - LEFT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      for ( int i = FRONT_LEFT; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         int i_adj = i - FRONT_LEFT;
         int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      for ( int i = 0; i < FRONT; ++i ) {
         int i_adj = i + SIZEOF_LARGE_NEO_CORNER;
         int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( clr1, clr2, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      

      // Reversing already-written pixels in place, not writing a NEW color
      // by position -- outside what LightSetters' position-addressed
      // setters model, so this stays direct raw-array access on purpose.
      LedUtil::reverse( carpet->ropeLeds + FRONT, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + REAR, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + RIGHT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + LEFT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );

      // FLOODS (megabar + china), all 3 variations -- per request: 2
      // groups (every 2nd lamp going around the car), group1 brightness a
      // sine wave, group2 a cosine (both 50%-100% of max, same
      // floodHz_/floodPhaseRad_ driving the rope reversal timing above).
      // group1/group2 swap which of the 2 rope colors they're currently
      // showing every 1.5x the sin/cos period, crossfading over 0.25x the
      // period on each swap.
      float group1Frac = 0.75f + 0.25f * sinf( floodPhaseRad_ ); // 50%-100%
      float group2Frac = 0.75f + 0.25f * cosf( floodPhaseRad_ );

      floodSwapElapsedS_ += dtSec;
      float swapPeriodS = 1.5f / floodHz_;
      if ( floodSwapElapsedS_ >= swapPeriodS ) {
         floodSwapElapsedS_ -= swapPeriodS;
         floodGroup1IsClr1_ = !floodGroup1IsClr1_;
         floodFadeElapsedS_ = 0.0f;
      }
      floodFadeElapsedS_ += dtSec;
      float fadeDurationS = 0.25f / floodHz_;
      uint8_t fadeF = (uint8_t)( 255.0f * constrain( floodFadeElapsedS_ / fadeDurationS, 0.0f, 1.0f ) + 0.5f );
      // only 2 possible colors total, so "the other one" IS last swap's
      // color -- no separate fade-from capture needed.
      CRGB group1Now = floodGroup1IsClr1_ ? clr1 : clr2, group1Other = floodGroup1IsClr1_ ? clr2 : clr1;
      CRGB group2Now = floodGroup1IsClr1_ ? clr2 : clr1, group2Other = floodGroup1IsClr1_ ? clr1 : clr2;
      CRGB group1Clr = blend( group1Other, group1Now, fadeF );
      CRGB group2Clr = blend( group2Other, group2Now, fadeF );
      group1Clr.nscale8( (uint8_t)( group1Frac * 255.0f + 0.5f ) );
      group2Clr.nscale8( (uint8_t)( group2Frac * 255.0f + 0.5f ) );

      // Stepped by real angle (every next 30deg around the ring), not by
      // raw DMX-order index parity -- per request, so this pattern's
      // intent ("alternate every OTHER megabar around the car") stays
      // obvious/robust if ever asked to change (e.g. "alternate every
      // 60deg instead"), rather than depending on knowing DMX order
      // happens to match angle order today.
      for ( int step = 0; step < 12; ++step ) {
         float angle = (float)step * 30.0f;
         CRGB clr = ( step % 2 == 0 ) ? group1Clr : group2Clr;
         LightSetters::setColor( carpet, LightSetters::TargetMegabar, clr, LightSetters::ByAngleDeg{ angle } );
      }

      // china: same group coloring as megabars, plus (VarAutoWithSound
      // only) a white-channel overlay on bass hits -- per request, this
      // OVERRIDES the white channel for the hit's own duration without
      // otherwise disturbing the group/swap color underneath (a separate
      // channel on the same CRGBW pixel, not a replacement of the base
      // color).
      uint8_t chinaWhite = 0;
      if ( variation_ == VarAutoWithSound ) {
         chinaWhite = (uint8_t)( (uint16_t)AudioBoard::getBandHitPercent( BandBass ) * 255 / 100 );
      }
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
         CRGB clr = ( c % 2 == 0 ) ? group1Clr : group2Clr;
         LightSetters::setColor( carpet, LightSetters::TargetChina, clr, LightSetters::ByID{ (uint8_t)c } );
         LightSetters::setWhite( carpet, LightSetters::TargetChina, chinaWhite, LightSetters::ByID{ (uint8_t)c } );
      }

      /*
      static uint8_t white = 0;
      if ( carpet->encoder->button.isDown() ) {
         if ( white < 255 ) {
            ++white;
         }
      } else {
         if ( white > 0 ) {
            --white;
         }
      }
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         carpet->ropeLeds[ i ].w = white;
      }
      */
   }
};
