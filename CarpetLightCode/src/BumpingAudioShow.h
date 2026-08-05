/* EqualizerShow.h
 *
 * A basic version of the Nightrider light effect, which is just a simple chase that
 * starts in the corners and bounces off the middles. Lame, but it lights up the
 * carpet and doesn't rely on the sound.
 *
 * Variation 1: a dual VU meter along the two side rope runs -- bass grows from
 * the back corner toward the front (red), treble grows from the front corner
 * toward the back (blue), reusing this show's own red=bass/blue=treble
 * convention from variation 0's megabar tinting. Each meter's 100%-level reach
 * is 15% of the true-corner-to-center half-length past center (RIGHT/LEFT are
 * already the exact side midpoints -- see MagicCarpet.h's positional
 * constants), so maxed-out bass and treble simultaneously cross by about that
 * much right around the middle of the car, per request.
 *
 * Author: Anders Linn
 * Date: August 2017
 */

#include "LightShow.h"
#include <math.h>

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

// Front/rear vs left/right megabar grouping for new_standard's cycle 2 --
// derived from LighthouseShow.h's own megabar angle convention
// (wrap360(-m*30), front=0/right=90/back=180/left=270 -- the only per-
// megabar geometric convention firmware actually has, positional (x,y)
// tables being a visualizer-only rendering concept), classifying each of
// the 12 evenly-30-deg-spaced megabars into whichever pair of opposite
// edges its angle sits closest to.
static const int NEWSTD_FRONT_REAR_MEGABARS[6] = { 0, 1, 5, 6, 7, 11 };
static const int NEWSTD_LEFT_RIGHT_MEGABARS[6] = { 2, 3, 4, 8, 9, 10 };
// China grouping, per README's china layout / SpeedStripesShow.h's own
// comment: [1,2] aimed along the front edge, [5,6] along the back edge,
// [0,3,4,7] aimed along a side edge.
static const int NEWSTD_CHINA_FRONTBACK[4] = { 1, 2, 5, 6 };
static const int NEWSTD_CHINA_SIDES[4] = { 0, 3, 4, 7 };
// pixel_war china pairing: each of the 8 chinas paired with the one one
// address above it.
static const int PW_CHINA_PAIRS[4][2] = { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 } };

class EqualizerShow : public LightShow {
 private:
   enum Variation { VarChase = 0, VarVuMeter = 1, VarNewStandard = 2, VarPixelWar = 3 };
   static const uint8_t numVariations_ = 4;
   uint8_t variation_;
   Timer eqFrameTimer_; // tracks dt between update() calls, for the 2 variations below

   /* ---- new_standard / pixel_war shared state --------------------------
      Ported from the visualizer prototype (tools/visualizer/carpet-
      visualizer.html) -- see its own inline comments for the full spec
      reasoning; this is a faithful port of that already-tuned behavior,
      not a fresh design. */
   static constexpr float NEWSTD_HUE_PERIOD_S = 20.0f; // 10s red->yellow, 10s back -- full cycle
   static constexpr float NEWSTD_SAT_PERIOD_S = 14.0f; // 7/10 of the hue period
   static const uint8_t NEWSTD_HUE_RED = 0, NEWSTD_HUE_YELLOW = 42, NEWSTD_HUE_TREBLE = 160; // blue, constant
   float newStdHueTimeS_ = 0.0f, newStdSatTimeS_ = 0.0f; // hue time freezes during silence, sat time never does

   bool newStdChinaCycleStarted_ = false;
   Timer newStdChinaCycleTimer_;
   uint8_t newStdChinaCycle_ = 0;
   bool newStdMegabarCycleStarted_ = false;
   Timer newStdMegabarCycleTimer_;
   uint8_t newStdMegabarCycle_ = 0;
   bool newStdCycle1TrebleIsMod3_ = true;
   bool newStdCycle2FrontRearIsBass_ = true;
   bool newStdCycle3BassAssignment_[ NUM_MEGABAR_LEDS ] = { false };
   float newStdMegabarHeat_[ NUM_MEGABAR_LEDS ] = { 0.0f }; // cycle 3 only
   uint8_t newStdPrevBassHit_ = 0, newStdPrevTrebleHit_ = 0;

   // Shared "no sound" flood behavior (all 4 variations, per request):
   // floods sit at a dim 15% baseline, and every 100-500ms a random flood
   // swaps its color assignment with a random flood of the other color,
   // spiking to full brightness then easing to 80% over ~200ms.
   static const int NEWSTD_NUM_FLOODS = NUM_MEGABAR_LEDS + NUM_CHINA_LEDS;
   bool newStdFloodIsB_[ NEWSTD_NUM_FLOODS ];
   float newStdFloodBrightness_[ NEWSTD_NUM_FLOODS ];
   bool newStdSwapArmed_ = false;
   uint32_t newStdNextSwapMs_ = 0;

   // Cross-fade state for china/megabar CYCLE transitions (not silence
   // mode, which already has its own spike/decay) -- on a cycle change,
   // whatever color each flood was actually last showing is captured as
   // the fade start, and blended toward the new cycle's own (still live,
   // per-frame) target color over 200ms.
   static const uint32_t NEWSTD_FADE_MS = 200;
   CRGB newStdChinaFadeFrom_[ NUM_CHINA_LEDS ];
   uint32_t newStdChinaFadeStartMs_ = 0;
   CRGB newStdLastChinaColor_[ NUM_CHINA_LEDS ];
   CRGB newStdMegabarFadeFrom_[ NUM_MEGABAR_LEDS ];
   uint32_t newStdMegabarFadeStartMs_ = 0;
   CRGB newStdLastMegabarColor_[ NUM_MEGABAR_LEDS ];

   // pixel_war: territory war (rope), hit-picked megabars, paired china.
   static constexpr float PW_MIN_FRAC = 0.125f, PW_MAX_FRAC = 0.875f; // 12.5%..87.5% of the loop
   static constexpr float PW_FADE_FRAC = 1.0f / 14.0f; // border cross-fade width
   static constexpr float PW_HIT_IMPULSE = 0.12f;  // territory fraction jumped per qualifying hit edge
   static constexpr float PW_RECOVER_RATE = 0.35f; // fraction/sec pulled back toward center between hits
   static constexpr float PW_WALK_MAX = 150.0f;    // LED/s
   float pwBassFrac_ = 0.5f;
   float pwRotationOffsetLed_ = 0.0f, pwWalkspeed_ = 0.0f, pwWalkAccumMs_ = 0.0f;
   float pwMegabarHeat_[ NUM_MEGABAR_LEDS ] = { 0.0f };
   bool pwMegabarColorIsB_[ NUM_MEGABAR_LEDS ] = { true };
   int pwLastPicked_[ 2 ] = { -1, -1 };
   uint8_t pwPrevBassHit_ = 0, pwPrevTrebleHit_ = 0;
   bool pwChinaPairIsBass_[ 4 ] = { true, true, false, false };
   int pwChinaHitCount_ = 0;

   // triple-strobe on bass hits (see updateStrobe()) -- nonvolatile, toggled
   // by a double press while this show is active (see CarpetLightLogic.cpp)
   bool strobeEnabled_;
   bool strobeActive_ = false;
   Timer strobeTimer_;

   // hit-suppression state: once a hit strobes, its level is remembered, and
   // any further hit that doesn't exceed it is suppressed as long as it's
   // within 3s of the last qualifying hit (strobed or suppressed) -- a gap of
   // 3s with no hits at all clears this, so the next hit always strobes
   // fresh, regardless of level. See updateStrobe().
   bool hadHit_ = false;     // Timer has no "unset" state of its own
   Timer lastHitTimer_;      // timeout armed to SILENCE_RESET_MS on the first hit
   int suppressionPeak_ = 0;

   // pot adjusts AudioBoard's shared peak threshold while this show (either
   // variation) is active -- soft takeover, same convention as config mode's
   // livePercentFor(): holds at the last-committed value until the pot
   // actually moves since this show became active, so it doesn't jump.
   // Reverted back to the committed value on leaving this show (see
   // CarpetLightLogic.cpp's show-change block), so casual live tweaking here
   // never overwrites what's saved to flash.
   static constexpr float POT_TAKEOVER_THRESHOLD_PERCENT = 2.0f;
   float potEntryPercent_ = 0.0f;
   bool potTakenOver_ = false;
   SettlePrinter thresholdPrinter_;

 public:
   EqualizerShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0, bool initialStrobeEnabled = false )
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ), strobeEnabled_( initialStrobeEnabled ) {}

   uint8_t variation() {
      return variation_;
   }

   const char * variationName() {
      if ( variation_ == VarChase ) return "chase";
      if ( variation_ == VarVuMeter ) return "VU meter";
      if ( variation_ == VarNewStandard ) return "new_standard";
      return "pixel_war";
   }

   bool getStrobeEnabled() {
      return strobeEnabled_;
   }
   void setStrobeEnabled( bool enabled ) {
      strobeEnabled_ = enabled;
   }

   void start() {
      for ( int i = NEO3_OFFSET; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         carpet->ropeLeds[ i ] = CRGB::Black;
      }
      potEntryPercent_ = carpet->pot->readPercent();
      potTakenOver_ = false;
      eqFrameTimer_.reset();

      newStdHueTimeS_ = 0.0f; newStdSatTimeS_ = 0.0f;
      newStdChinaCycleStarted_ = false; newStdChinaCycle_ = 0;
      newStdMegabarCycleStarted_ = false; newStdMegabarCycle_ = 0;
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdMegabarHeat_[ m ] = 0.0f;
      newStdPrevBassHit_ = 0; newStdPrevTrebleHit_ = 0;
      for ( int i = 0; i < NEWSTD_NUM_FLOODS; ++i ) { newStdFloodIsB_[ i ] = ( i % 2 == 0 ); newStdFloodBrightness_[ i ] = 0.15f; }
      newStdSwapArmed_ = false;
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) newStdLastChinaColor_[ c ] = CRGB::Black;
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdLastMegabarColor_[ m ] = CRGB::Black;
      newStdChinaFadeStartMs_ = 0; newStdMegabarFadeStartMs_ = 0;

      pwBassFrac_ = 0.5f;
      pwRotationOffsetLed_ = 0.0f; pwWalkspeed_ = 0.0f; pwWalkAccumMs_ = 0.0f;
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) pwMegabarHeat_[ m ] = 0.0f;
      pwLastPicked_[ 0 ] = -1; pwLastPicked_[ 1 ] = -1;
      pwPrevBassHit_ = 0; pwPrevTrebleHit_ = 0;
      // randomly assign 2 of the 4 china pairs to bass, 2 to treble
      int idxs[ 4 ] = { 0, 1, 2, 3 };
      for ( int i = 3; i > 0; --i ) { int j = random( i + 1 ); int t = idxs[ i ]; idxs[ i ] = idxs[ j ]; idxs[ j ] = t; }
      for ( int p = 0; p < 4; ++p ) pwChinaPairIsBass_[ p ] = false;
      pwChinaPairIsBass_[ idxs[ 0 ] ] = true; pwChinaPairIsBass_[ idxs[ 1 ] ] = true;
      pwChinaHitCount_ = 0;
   }

   void update( uint32_t time ) {
      // variation select: each encoder detent moves to the next variation,
      // same convention as NightriderShow/FlameShow
      int varDelta = carpet->encoder->readPositionDelta();
      carpet->encoder->resetPositionDelta();
      if ( varDelta != 0 ) {
         int newVariation = ( (int)variation_ + varDelta ) % (int)numVariations_;
         if ( newVariation < 0 ) newVariation += numVariations_;
         variation_ = (uint8_t)newVariation;
      }

      float dtSec = (float)eqFrameTimer_.elapsed() / 1000.0f;
      eqFrameTimer_.reset();

      // pot -> live peak threshold, soft takeover (see member comment above)
      float potPercent = carpet->pot->readPercent();
      if ( !potTakenOver_ ) {
         if ( fabsf( potPercent - potEntryPercent_ ) >= POT_TAKEOVER_THRESHOLD_PERCENT ) potTakenOver_ = true;
      }
      if ( potTakenOver_ ) {
         AudioBoard::setPeakThresholdPercent( potPercent );
      }
      thresholdPrinter_.update( (int)( AudioBoard::getPeakThresholdPercent() + 0.5f ), "PkThresh:", "%" );

      if ( variation_ == VarNewStandard ) {
         updateNewStandard( time, dtSec );
      } else if ( variation_ == VarPixelWar ) {
         updatePixelWar( time, dtSec );
      } else if ( variation_ == VarVuMeter ) {
         updateVuMeter( time );
         // Per request, all 4 variations go dark-with-nothing-to-react-to
         // the same way during silence -- new_standard/pixel_war have their
         // own internal silence handling above; Chase/VU meter get it as a
         // post-step override on top of their own (otherwise near-dark)
         // output, same as the visualizer prototype.
         if ( AudioBoard::silent_ ) updateSilenceFloods( time, dtSec, CRGB( 255, 0, 0 ), CRGB( 0, 0, 255 ) ); // red=bass, blue=treble, this show's own convention
      } else {
      static const CRGBPalette256 clr( CRGB::Red, CRGB::Black );
      static uint32_t timestamp = time;
      static const uint32_t rate = 300; // move the lights every 200ms
      static const uint32_t littleRate = rate;
      static const uint32_t bigRate = ( littleRate * SIZEOF_LARGE_NEO ) / SIZEOF_SMALL_NEO;
      static uint8_t bigPos = 0;
      static uint8_t littlePos = 0;
      static uint8_t bigPosDir = 1;
      static uint8_t littlePosDir = 1;

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

      //const uint8_t val1 = carpet->pot->read() / 4;
      //const uint8_t val2 = ( val1 + 128 ) % 255;
      //const CRGB clr1 = CHSV( val1, 255, 255 );
      //const CRGB clr2 = CHSV( val2, 255, 255 );;
      float satFraction = currentSatFraction( time );
      CRGB clr1 = desaturate( CRGB(0,0,255), satFraction );
      CRGB clr2 = desaturate( CRGB(255,0,0), satFraction );
      // CRGB clr3 = CRGB(0,255,0);
      // Serial.println( "potval" );
      // Serial.println( val1 );
      //const CRGBW clr1 = topC[val1];
      //const CRGBW clr2 = bottomC[val1];
      // Serial.println( "CRGB1" );
      // Serial.println( clr1.r );
      // Serial.println( clr1.g );
      // Serial.println( clr1.b );
      // Serial.println( clr1.w );
      // Serial.println( "CRGB2" );
      // Serial.println( clr2.r );
      // Serial.println( clr2.g );
      // Serial.println( clr2.b );
      // Serial.println( clr2.w );
      // int diffIndex = scaleTo255( diff, rate, 0 );

      for ( int i = FRONT; i < FRONT_RIGHT; ++i ) {
         int i_adj = i - FRONT;
        int val = scaleTo255( abs(littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Red;
      }
      for ( int i = FRONT_RIGHT; i < RIGHT; ++i ) {
         int i_adj = i - FRONT_RIGHT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Green;
      }
      for ( int i = RIGHT; i < BACK_RIGHT; ++i ) {
         int i_adj = i - RIGHT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Blue;
      }
      for ( int i = BACK_RIGHT; i < BACK; ++i ) {
         int i_adj = i - BACK_RIGHT;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Yellow;
      }
      for ( int i = BACK; i < BACK_LEFT; ++i ) {
         int i_adj = i - BACK;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Orange;
      }
      for ( int i = BACK_LEFT; i < LEFT; ++i ) {
         int i_adj = i - BACK_LEFT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Purple;
      }
      for ( int i = LEFT; i < FRONT_LEFT; ++i ) {
         int i_adj = i - LEFT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Pink;
      }
      for ( int i = FRONT_LEFT; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         int i_adj = i - FRONT_LEFT;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Grey;
      }
      for ( int i = 0; i < FRONT; ++i ) {
         int i_adj = i + SIZEOF_LARGE_NEO_CORNER;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Grey;
      }


      LedUtil::reverse( carpet->ropeLeds + FRONT, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
       LedUtil::reverse( carpet->ropeLeds + BACK, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + RIGHT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + LEFT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );

      // switched to the shared hit value (attack/decay peak-hold now lives
      // in AudioBoard, tunable via the Audio config screen's decay-rate
      // subsetting) instead of this show's own ad-hoc lastlow tracking --
      // per request, all EqualizerShow variations use hit instead of level.
      CRGB dmxclr;
      uint8_t lastlow = (uint8_t)( (uint16_t)AudioBoard::getHitPercent( BandLow ) * 255 / 100 );
      dmxclr = blend( CRGB::Black, clr2, lastlow );
      carpet->megabarLeds[1] = dmxclr;
      carpet->megabarLeds[2] = dmxclr;
      carpet->megabarLeds[4] = dmxclr;
      carpet->megabarLeds[5] = dmxclr;
      carpet->megabarLeds[7] = dmxclr;
      carpet->megabarLeds[8] = dmxclr;
      carpet->megabarLeds[10] = dmxclr;
      carpet->megabarLeds[11] = dmxclr;


      int highval = (int)( (uint16_t)AudioBoard::getHitPercent( BandHigh ) * 255 / 100 );
      // Serial.print("highval: ");
      // Serial.println(highval);
      /*static int lasthigh = highval;
      Serial.print("lasthigh: ");
      Serial.println(highval);
      Serial.flush();
      if (highval > 100 && highval > lasthigh) {
       lasthigh = highval;
      } else {
       lasthigh = lasthigh > 15 ? lasthigh - 15 : 0;
      }
      Serial.print("lasthigh: ");
      Serial.println(lasthigh);
      Serial.flush();*/
      // dmxclr = blend( dmxclr, clr1, highval );
      dmxclr = blend( CRGB::Black, clr1, highval );

      carpet->megabarLeds[0] = dmxclr;
      carpet->megabarLeds[3] = dmxclr;
      carpet->megabarLeds[6] = dmxclr;
      carpet->megabarLeds[9] = dmxclr;



      //CRGB dmxclr = blend( clr1, clr2, lowval);
      //dmxclr = blend( clr2, clr3, highval);
      //dmxclr = CRGB(lowval,highval,30);



      /*int midval = AudioBoard::getMid();
      int highval = AudioBoard::getHigh();
      static int maxlow = 0;
      static int minlow = 255;
      maxlow = max(maxlow, lowval);
      Serial.println("maxlow");
      Serial.println(maxlow);
      minlow = min(minlow, lowval);
      Serial.println("minlow");
      Serial.println(minlow);
      // lowval = lowval > 75 ? lowval - 75 : 0;
      // midval = midval > 75 ? midval - 75 : 0;
     //  highval = highval > 75 ? highval - 75 : 0;
      // CRGB dmxclr = (lowval > midval && lowval >= highval) ? CRGB::Red :(highval > midval) ? CRGB::Green : CRGB::Blue;
      float avg = lowval + midval + highval;
      int dmxred = (255 * ( ((float) lowval) / avg));
      int dmxblue = (255 * ( ((float) midval) / avg));
      int dmxgreen = (255 * ( ((float) highval) / avg));
      Serial.println("dmxred:");
      Serial.println(dmxred);
      Serial.println("dmxblue:");
      Serial.println(dmxblue);
      Serial.println("dmxgreen:");
      Serial.println(dmxgreen);
      //CRGB dmxclr(dmxred, 0, dmxgreen);
      Serial.println("low: ");
      Serial.println(lowval);
      Serial.println("mid");
      Serial.println(midval);
      Serial.println("high");
      Serial.println(highval);*/
      // CRGB dmxclr(lowval, midval, highval);
      //LedUtil::fill( carpet->megabarLeds, dmxclr, NUM_MEGABAR_LEDS );
      //LedUtil::fill( carpet->chinaLeds, dmxclr, NUM_CHINA_LEDS );

      // disabled: button now drives global show-cycling, not this show's white glow
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
      if ( AudioBoard::silent_ ) updateSilenceFloods( time, dtSec, CRGB( 255, 0, 0 ), CRGB( 0, 0, 255 ) ); // red=bass, blue=treble, same as VU meter's override above
      } // end variation 0

      updateStrobe( time );
   }

 private:
   // variation 1: dual VU meter. Bass (AudioBoard::getLow()) is based across
   // the WHOLE back of the car -- the back edge rope is always part of it,
   // not just the two back corners -- and grows forward from there along
   // both sides, red. Treble (getHigh()) mirrors this from the whole front
   // edge, growing backward along both sides, blue -- same red=bass/blue=
   // treble convention as variation 0's megabar tinting above. Both meters
   // use the same attack/decay peak-hold ballistics as variation 0's lowval/
   // lastlow (jump up fast past a threshold, fall back 15/frame otherwise),
   // just kept as separate state. Brightness of each meter (base edge and
   // side fill alike) scales directly with its current level -- a quiet
   // signal shows a dim meter, not just a short one.
   //
   // RIGHT/LEFT are already the exact midpoints of each side run (see
   // MagicCarpet.h's positional constants), and FRONT_RIGHT/BACK_RIGHT (resp.
   // FRONT_LEFT/BACK_LEFT) the true corners -- so halfLen_ below is exactly
   // the true-corner-to-center distance, reusing the same geometry as
   // variation 0's "big" segments (SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER).
   // Each meter's 100%-level reach is halfLen_ plus 15% of halfLen_, so maxed
   // bass and treble together cross by about 15% of half the car's length,
   // right around the middle -- per request.
   void updateVuMeter( uint32_t time ) {
      // hit already implements this meter's attack/decay ballistics (see
      // AudioBoard::updateHitTracking(), tunable via the Audio config
      // screen's decay-rate subsetting), so no more local peak-hold state.
      int vuLastBass = (int)( (uint16_t)AudioBoard::getHitPercent( BandLow ) * 255 / 100 );
      int vuLastTreble = (int)( (uint16_t)AudioBoard::getHitPercent( BandHigh ) * 255 / 100 );

      static const float halfLen_ = SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER; // true corner to center, 143
      static const float maxReach_ = halfLen_ * 1.15f; // 100%-level reach: 15% of halfLen_ past center
      float bassLitCount = ( vuLastBass / 255.0f ) * maxReach_;
      float trebleLitCount = ( vuLastTreble / 255.0f ) * maxReach_;

      // brightness follows level directly -- a quiet meter is a dim meter
      float satFraction = currentSatFraction( time );
      CRGB bassClr = desaturate( CRGB( 255, 0, 0 ), satFraction );   // red, per request
      CRGB trebleClr = desaturate( CRGB( 0, 0, 255 ), satFraction ); // blue, matching variation 0's high=blue convention
      bassClr.nscale8( (uint8_t)vuLastBass );
      trebleClr.nscale8( (uint8_t)vuLastTreble );

      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();

      // back edge: always part of the bass meter's base, full width
      for ( int i = SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; ++i ) {
         carpet->ropeLeds[ i ] = bassClr;
      }
      // front edge: always part of the treble meter's base, full width
      for ( int i = 0; i < SIZEOF_SMALL_NEO; ++i ) {
         carpet->ropeLeds[ i ] = trebleClr;
      }

      // right side: FRONT_RIGHT/BACK_RIGHT are the reference corner-to-
      // center points bassLitCount/trebleLitCount are calibrated against
      // (see halfLen_/maxReach_ above) -- kept fixed. The loop itself,
      // however, covers the side channel's ENTIRE physical strand (true
      // corner to true corner: SIZEOF_SMALL_NEO to SIZEOF_SMALL_NEO+
      // SIZEOF_LARGE_NEO-1), not just the span between the reference
      // points.
      //
      // BUGFIX: this loop used to run FRONT_RIGHT..BACK_RIGHT only (the
      // reference points themselves), leaving the ~33-LED zone on EACH side
      // of that span (between the true physical corner and its inset
      // reference point) completely dark regardless of level -- neither
      // this loop nor the front/back base-edge fill above ever touched it.
      // Widening the loop to the true strand bounds fixes this using this
      // side's OWN distance-from-reference math (which, past a reference
      // point, is always "closer than any possible LitCount," i.e. reads as
      // an unconditional, level-proportional continuation of that meter --
      // physically correct, and entirely local to this side's own strand,
      // never reaching into the front/back channels' own logic the way an
      // earlier version of this fix mistakenly did).
      for ( int i = SIZEOF_SMALL_NEO; i < SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; ++i ) {
         float distFromFront = i - FRONT_RIGHT;
         float distFromBack = BACK_RIGHT - i;
         carpet->ropeLeds[ i ] = vuMeterColor( distFromBack < bassLitCount, distFromFront < trebleLitCount, bassClr, trebleClr );
      }
      // left side: same widening, BACK_LEFT/FRONT_LEFT stay the fixed
      // reference points, loop covers the full physical strand.
      for ( int i = SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         float distFromBack = i - BACK_LEFT;
         float distFromFront = FRONT_LEFT - i;
         carpet->ropeLeds[ i ] = vuMeterColor( distFromBack < bassLitCount, distFromFront < trebleLitCount, bassClr, trebleClr );
      }
   }

   // slow rolling desaturation applied to every base color used in both
   // variations: saturation drifts from 100% down to 85% and back over a
   // 30-second period (smooth sine, not a hard bounce). satFraction is
   // 1.0-0.85, the fraction of full saturation currently in effect.
   static float currentSatFraction( uint32_t time ) {
      static const uint32_t periodMs = 30000;
      float phase = (float)( time % periodMs ) / (float)periodMs; // 0..1 over one period
      return 0.925f + 0.075f * cosf( 2.0f * PI * phase ); // 1.0 at phase 0, 0.85 at phase 0.5, back to 1.0 at phase 1
   }

   // reduces a color's saturation to satFraction of full (1.0 = unchanged,
   // 0.0 = fully gray) while holding its hue and value (max channel) fixed --
   // raises each channel toward the max channel rather than converting
   // through HSV, so it's exact for these already-fully-saturated base colors
   // and a safe no-op on anything less saturated to begin with (e.g. white).
   static CRGB desaturate( CRGB clr, float satFraction ) {
      uint8_t maxC = max( clr.r, max( clr.g, clr.b ) );
      if ( maxC == 0 ) return clr;
      uint8_t minFloor = (uint8_t)( maxC * ( 1.0f - satFraction ) + 0.5f );
      if ( clr.r < minFloor ) clr.r = minFloor;
      if ( clr.g < minFloor ) clr.g = minFloor;
      if ( clr.b < minFloor ) clr.b = minFloor;
      return clr;
   }

   // (1-cos(x))/2 naturally spends more real TIME near 0 and 1 (where its own
   // derivative is smallest) than near 0.5 -- "slowing down near each end so
   // every value gets about equal time," with no separate easing curve.
   // BUGFIX ("floods hella dim vs SpeedStripes"): the bass/treble base
   // fills below used to scale straight linearly with hit% (0% hit = 0%
   // brightness), reading visibly dim next to a non-audio-reactive show
   // like SpeedStripes's default variant, which is always at full
   // brightness. Zebra (SpeedStripes' own audio-reactive variation)
   // already established the right pattern for this project -- a resting
   // floor that hits push up from, not a floor-less linear scale --
   // applied here too, with a lower floor than zebra's own 75% since this
   // show is meant to read as flashier/more dynamic.
   static constexpr float EQ_HIT_BRIGHTNESS_FLOOR = 0.65f;
   static uint8_t hitBrightnessByte( uint8_t pct ) {
      float frac = max( EQ_HIT_BRIGHTNESS_FLOOR, (float)pct / 100.0f );
      return (uint8_t)( frac * 255.0f + 0.5f );
   }
   void newStdColors( CRGB & Bcolor, CRGB & Tcolor ) {
      float hueFrac = ( 1.0f - cosf( 2.0f * PI * newStdHueTimeS_ / NEWSTD_HUE_PERIOD_S ) ) / 2.0f;
      float satFrac = ( 1.0f - cosf( 2.0f * PI * newStdSatTimeS_ / NEWSTD_SAT_PERIOD_S ) ) / 2.0f;
      uint8_t satByte = (uint8_t)( ( 75.0f + 25.0f * satFrac ) / 100.0f * 255.0f + 0.5f );
      uint8_t bHue = (uint8_t)( (float)NEWSTD_HUE_RED + ( (float)NEWSTD_HUE_YELLOW - (float)NEWSTD_HUE_RED ) * hueFrac + 0.5f );
      Bcolor = CHSV( bHue, satByte, 255 );
      Tcolor = CHSV( NEWSTD_HUE_TREBLE, satByte, 255 );
   }

   // Shared "no sound" flood behavior, used by all 4 variations -- see the
   // class member comment above. colorB/colorT are whichever two base
   // colors the calling variation uses (Bcolor/Tcolor for new_standard/
   // pixel_war, bass-red/treble-blue for Chase/VU meter).
   void updateSilenceFloods( uint32_t time, float dtSec, const CRGB & colorB, const CRGB & colorT ) {
      if ( !newStdSwapArmed_ || time >= newStdNextSwapMs_ ) {
         newStdSwapArmed_ = true;
         newStdNextSwapMs_ = time + 100 + random( 400 );
         int bIdxs[ NEWSTD_NUM_FLOODS ], tIdxs[ NEWSTD_NUM_FLOODS ];
         int bCount = 0, tCount = 0;
         for ( int i = 0; i < NEWSTD_NUM_FLOODS; ++i ) {
            if ( newStdFloodIsB_[ i ] ) bIdxs[ bCount++ ] = i; else tIdxs[ tCount++ ] = i;
         }
         if ( bCount > 0 && tCount > 0 ) {
            int a = bIdxs[ random( bCount ) ];
            int b = tIdxs[ random( tCount ) ];
            newStdFloodIsB_[ a ] = false; newStdFloodIsB_[ b ] = true;
            newStdFloodBrightness_[ a ] = 1.0f; newStdFloodBrightness_[ b ] = 1.0f;
         }
      }
      for ( int i = 0; i < NEWSTD_NUM_FLOODS; ++i ) {
         float target = newStdFloodBrightness_[ i ] > 0.15f + 1e-3f ? 0.8f : 0.15f;
         newStdFloodBrightness_[ i ] += ( target - newStdFloodBrightness_[ i ] ) * min( 1.0f, dtSec / 0.2f );
         CRGB color = newStdFloodIsB_[ i ] ? colorB : colorT;
         color.nscale8( (uint8_t)( newStdFloodBrightness_[ i ] * 255.0f + 0.5f ) );
         if ( i < NUM_MEGABAR_LEDS ) { carpet->megabarLeds[ i ] = color; }
         else { carpet->chinaLeds[ i - NUM_MEGABAR_LEDS ] = color; carpet->chinaLeds[ i - NUM_MEGABAR_LEDS ].w = 0; }
      }
   }

   // BUGFIX (rope war "does nothing"): see the visualizer prototype's own
   // comment for the full diagnosis -- the original continuous tug-of-war
   // (push proportional to hit LEVEL every frame, weak constant recover)
   // reached its territory cap within about 1 real second under sustained
   // one-sided hits and then sat there perfectly static for as long as
   // that side kept dominating, which under real repeating music read as
   // "does nothing" almost immediately. Replaced with a discrete per-hit
   // IMPULSE plus much stronger continuous recovery, so the boundary keeps
   // visibly swinging back and forth instead of snapping to an extreme.
   //
   // Random walk: a fresh kick every 50ms (an independent random kick, not
   // one scaled by the current walkspeed -- scaling by current speed
   // stalls forever at 0 and never starts moving).
   void pwStepRandomWalk( float dtMs ) {
      pwWalkAccumMs_ += dtMs;
      while ( pwWalkAccumMs_ >= 50.0f ) {
         pwWalkAccumMs_ -= 50.0f;
         float accel = ( (float)random( -1000, 1001 ) / 1000.0f * 10.0f ) * 3.0f;
         pwWalkspeed_ = constrain( pwWalkspeed_ + accel, -PW_WALK_MAX, PW_WALK_MAX );
         pwRotationOffsetLed_ += pwWalkspeed_ * 0.05f;
      }
   }
   CRGB pwColorAt( int i, const CRGB & Bcolor, const CRGB & Tcolor ) {
      float raw = fmodf( (float)i + pwRotationOffsetLed_, (float)NUM_NEO_LEDS_ACTUAL );
      if ( raw < 0.0f ) raw += (float)NUM_NEO_LEDS_ACTUAL;
      float normPos = raw / (float)NUM_NEO_LEDS_ACTUAL;
      float halfFade = PW_FADE_FRAC / 2.0f;
      float dWrap = min( normPos, 1.0f - normPos );
      float dMid = fabsf( normPos - pwBassFrac_ );
      dMid = min( dMid, 1.0f - dMid );
      bool inB = normPos < pwBassFrac_;
      float nearestD = min( dWrap, dMid );
      if ( nearestD >= halfFade ) return inB ? Bcolor : Tcolor;
      float f = 0.5f - 0.5f * ( nearestD / halfFade );
      uint8_t f255 = (uint8_t)( f * 255.0f + 0.5f );
      return inB ? blend( Bcolor, Tcolor, f255 ) : blend( Tcolor, Bcolor, f255 );
   }

   void updateNewStandard( uint32_t time, float dtSec ) {
      bool silent = AudioBoard::silent_;
      newStdSatTimeS_ += dtSec;
      if ( !silent ) newStdHueTimeS_ += dtSec;
      CRGB Bcolor, Tcolor;
      newStdColors( Bcolor, Tcolor );
      uint8_t bassHit = AudioBoard::getHitPercent( BandLow ), trebleHit = AudioBoard::getHitPercent( BandHigh );

      // ROPE: the default variation's single traveling segment, plus 3 more
      // evenly spaced (a quarter of the loop) apart, all in Bcolor.
      float chasePos = fmodf( (float)time / 1000.0f * 60.0f, (float)NUM_NEO_LEDS_ACTUAL );
      float quarter = (float)NUM_NEO_LEDS_ACTUAL / 4.0f;
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         float nearest = 1e9f;
         for ( int s = 0; s < 4; ++s ) {
            float segPos = fmodf( chasePos + s * quarter, (float)NUM_NEO_LEDS_ACTUAL );
            float d = min( fabsf( (float)i - segPos ), (float)NUM_NEO_LEDS_ACTUAL - fabsf( (float)i - segPos ) );
            if ( d < nearest ) nearest = d;
         }
         uint8_t amt = (uint8_t)max( 0.0f, 255.0f - nearest * 4.0f );
         carpet->ropeLeds[ i ] = blend( CRGB::Black, Bcolor, amt );
         carpet->ropeLeds[ i ].w = 0;
      }

      if ( silent ) {
         updateSilenceFloods( time, dtSec, Bcolor, Tcolor );
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdLastMegabarColor_[ m ] = carpet->megabarLeds[ m ];
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) newStdLastChinaColor_[ c ] = carpet->chinaLeds[ c ];
         return;
      }

      // CHINA: 4 cycles, 8s each, auto-advancing. First-ever call starts ON
      // cycle 0.
      if ( !newStdChinaCycleStarted_ ) {
         newStdChinaCycleStarted_ = true;
         newStdChinaCycleTimer_.reset();
      } else if ( newStdChinaCycleTimer_.elapsed() >= 8000 ) {
         newStdChinaCycleTimer_.reset();
         newStdChinaCycle_ = ( newStdChinaCycle_ + 1 ) % 4;
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) newStdChinaFadeFrom_[ c ] = newStdLastChinaColor_[ c ];
         newStdChinaFadeStartMs_ = time;
      }
      CRGB bassChina = Bcolor, trebleChina = Tcolor;
      bassChina.nscale8( hitBrightnessByte( bassHit ) );
      trebleChina.nscale8( hitBrightnessByte( trebleHit ) );
      if ( newStdChinaCycle_ == 0 ) {
         for ( int i = 0; i < 4; ++i ) carpet->chinaLeds[ NEWSTD_CHINA_FRONTBACK[ i ] ] = bassChina;
         for ( int i = 0; i < 4; ++i ) carpet->chinaLeds[ NEWSTD_CHINA_SIDES[ i ] ] = trebleChina;
      } else if ( newStdChinaCycle_ == 1 ) {
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) carpet->chinaLeds[ c ] = bassChina;
      } else { // 2 and 3 both read as "all treble" per spec
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) carpet->chinaLeds[ c ] = trebleChina;
      }
      if ( time - newStdChinaFadeStartMs_ < NEWSTD_FADE_MS ) {
         uint8_t fadeF = (uint8_t)( 255.0f * (float)( time - newStdChinaFadeStartMs_ ) / (float)NEWSTD_FADE_MS );
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) carpet->chinaLeds[ c ] = blend( newStdChinaFadeFrom_[ c ], carpet->chinaLeds[ c ], fadeF );
      }
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) { newStdLastChinaColor_[ c ] = carpet->chinaLeds[ c ]; carpet->chinaLeds[ c ].w = 0; }

      // MEGABARS: 3 cycles, 8s each, auto-advancing.
      if ( !newStdMegabarCycleStarted_ ) {
         newStdMegabarCycleStarted_ = true;
         newStdMegabarCycleTimer_.reset();
         newStdCycle1TrebleIsMod3_ = random( 2 ) == 0;
      } else if ( newStdMegabarCycleTimer_.elapsed() >= 8000 ) {
         newStdMegabarCycleTimer_.reset();
         newStdMegabarCycle_ = ( newStdMegabarCycle_ + 1 ) % 3;
         if ( newStdMegabarCycle_ == 0 ) newStdCycle1TrebleIsMod3_ = random( 2 ) == 0;
         if ( newStdMegabarCycle_ == 1 ) newStdCycle2FrontRearIsBass_ = random( 2 ) == 0;
         if ( newStdMegabarCycle_ == 2 ) {
            for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) { newStdCycle3BassAssignment_[ m ] = random( 2 ) == 0; newStdMegabarHeat_[ m ] = 0.0f; }
         }
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdMegabarFadeFrom_[ m ] = newStdLastMegabarColor_[ m ];
         newStdMegabarFadeStartMs_ = time;
      }
      CRGB bassMb = Bcolor, trebleMb = Tcolor;
      bassMb.nscale8( hitBrightnessByte( bassHit ) );
      trebleMb.nscale8( hitBrightnessByte( trebleHit ) );
      if ( newStdMegabarCycle_ == 0 ) {
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
            bool isMod3 = ( m % 3 == 0 );
            carpet->megabarLeds[ m ] = ( isMod3 == newStdCycle1TrebleIsMod3_ ) ? trebleMb : bassMb;
         }
      } else if ( newStdMegabarCycle_ == 1 ) {
         int swapNum = (int)( newStdMegabarCycleTimer_.elapsed() / 2000 ); // every 1/4 of 8s = 2s
         bool frontRearIsBass = ( swapNum % 2 == 0 ) ? newStdCycle2FrontRearIsBass_ : !newStdCycle2FrontRearIsBass_;
         for ( int i = 0; i < 6; ++i ) carpet->megabarLeds[ NEWSTD_FRONT_REAR_MEGABARS[ i ] ] = frontRearIsBass ? bassMb : trebleMb;
         for ( int i = 0; i < 6; ++i ) carpet->megabarLeds[ NEWSTD_LEFT_RIGHT_MEGABARS[ i ] ] = frontRearIsBass ? trebleMb : bassMb;
      } else {
         // Cycle 3: fixed half/half bass-treble assignment; each hit lights
         // 2 currently-black megabars in their own assigned color, falling
         // back to the 2 currently-dimmest if none are fully black.
         bool bassEdge = bassHit > 20 && newStdPrevBassHit_ <= 20;
         bool trebleEdge = trebleHit > 20 && newStdPrevTrebleHit_ <= 20;
         newStdPrevBassHit_ = bassHit; newStdPrevTrebleHit_ = trebleHit;
         if ( bassEdge || trebleEdge ) {
            int black[ NUM_MEGABAR_LEDS ], blackCount = 0;
            for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) if ( newStdMegabarHeat_[ m ] <= 0.02f ) black[ blackCount++ ] = m;
            int pick[ 2 ] = { -1, -1 };
            if ( blackCount >= 2 ) {
               for ( int n = 0; n < 2; ++n ) {
                  int j = random( blackCount );
                  pick[ n ] = black[ j ];
                  black[ j ] = black[ --blackCount ];
               }
            } else {
               // 2 dimmest -- simple selection over 12 entries
               int order[ NUM_MEGABAR_LEDS ]; for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) order[ m ] = m;
               for ( int a = 0; a < NUM_MEGABAR_LEDS - 1; ++a )
                  for ( int b = a + 1; b < NUM_MEGABAR_LEDS; ++b )
                     if ( newStdMegabarHeat_[ order[ b ] ] < newStdMegabarHeat_[ order[ a ] ] ) { int t = order[ a ]; order[ a ] = order[ b ]; order[ b ] = t; }
               pick[ 0 ] = order[ 0 ]; pick[ 1 ] = order[ 1 ];
            }
            for ( int n = 0; n < 2; ++n ) if ( pick[ n ] >= 0 ) newStdMegabarHeat_[ pick[ n ] ] = 1.0f;
         }
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
            newStdMegabarHeat_[ m ] = max( 0.0f, newStdMegabarHeat_[ m ] - dtSec / 0.4f );
            CRGB base = newStdCycle3BassAssignment_[ m ] ? Bcolor : Tcolor;
            base.nscale8( (uint8_t)( newStdMegabarHeat_[ m ] * 255.0f + 0.5f ) );
            carpet->megabarLeds[ m ] = base;
         }
      }
      if ( time - newStdMegabarFadeStartMs_ < NEWSTD_FADE_MS ) {
         uint8_t fadeF = (uint8_t)( 255.0f * (float)( time - newStdMegabarFadeStartMs_ ) / (float)NEWSTD_FADE_MS );
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) carpet->megabarLeds[ m ] = blend( newStdMegabarFadeFrom_[ m ], carpet->megabarLeds[ m ], fadeF );
      }
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdLastMegabarColor_[ m ] = carpet->megabarLeds[ m ];
   }

   void updatePixelWar( uint32_t time, float dtSec ) {
      bool silent = AudioBoard::silent_;
      newStdSatTimeS_ += dtSec;
      if ( !silent ) newStdHueTimeS_ += dtSec;
      CRGB Bcolor, Tcolor;
      newStdColors( Bcolor, Tcolor );
      uint8_t bassHit = AudioBoard::getHitPercent( BandLow ), trebleHit = AudioBoard::getHitPercent( BandHigh );

      if ( silent ) {
         updateSilenceFloods( time, dtSec, Bcolor, Tcolor );
         return;
      }

      bool bassEdge = bassHit > 20 && pwPrevBassHit_ <= 20;
      bool trebleEdge = trebleHit > 20 && pwPrevTrebleHit_ <= 20;
      pwPrevBassHit_ = bassHit; pwPrevTrebleHit_ = trebleHit;

      // ROPE: two territories fighting over the loop, slowly rotating.
      pwStepRandomWalk( dtSec * 1000.0f );
      if ( bassEdge ) pwBassFrac_ += PW_HIT_IMPULSE;
      if ( trebleEdge ) pwBassFrac_ -= PW_HIT_IMPULSE;
      pwBassFrac_ += ( 0.5f - pwBassFrac_ ) * PW_RECOVER_RATE * dtSec;
      pwBassFrac_ = constrain( pwBassFrac_, PW_MIN_FRAC, PW_MAX_FRAC );
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) { carpet->ropeLeds[ i ] = pwColorAt( i, Bcolor, Tcolor ); carpet->ropeLeds[ i ].w = 0; }

      // MEGABARS: on every hit, 2 random megabars (excluding whichever 2
      // the previous hit picked, and any still mid-flash) light up in that
      // hit's own color, then fade.
      if ( bassEdge || trebleEdge ) {
         bool isB = bassEdge; // if both edge the same frame, bass wins the tie
         int eligible[ NUM_MEGABAR_LEDS ], eligibleCount = 0;
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
            if ( pwMegabarHeat_[ m ] > 0.02f ) continue;
            if ( m == pwLastPicked_[ 0 ] || m == pwLastPicked_[ 1 ] ) continue;
            eligible[ eligibleCount++ ] = m;
         }
         if ( eligibleCount < 2 ) {
            eligibleCount = 0;
            for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) if ( pwMegabarHeat_[ m ] <= 0.02f ) eligible[ eligibleCount++ ] = m;
         }
         int picked[ 2 ] = { -1, -1 };
         for ( int n = 0; n < 2 && eligibleCount > 0; ++n ) {
            int j = random( eligibleCount );
            picked[ n ] = eligible[ j ];
            eligible[ j ] = eligible[ --eligibleCount ];
         }
         for ( int n = 0; n < 2; ++n ) if ( picked[ n ] >= 0 ) { pwMegabarHeat_[ picked[ n ] ] = 1.0f; pwMegabarColorIsB_[ picked[ n ] ] = isB; }
         pwLastPicked_[ 0 ] = picked[ 0 ]; pwLastPicked_[ 1 ] = picked[ 1 ];
      }
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         pwMegabarHeat_[ m ] = max( 0.0f, pwMegabarHeat_[ m ] - dtSec / 0.4f );
         CRGB base = pwMegabarColorIsB_[ m ] ? Bcolor : Tcolor;
         base.nscale8( (uint8_t)( pwMegabarHeat_[ m ] * 255.0f + 0.5f ) );
         carpet->megabarLeds[ m ] = base;
      }

      // CHINA: 4 pairs, 2 assigned to bass and 2 to treble -- each pair
      // shows its assigned band's color, brightness tracking that band's
      // own hit level live. Every 4 hit edges, a random pair swaps bands
      // with a random pair currently on the other band, keeping the 2/2
      // split.
      if ( bassEdge ) ++pwChinaHitCount_;
      if ( trebleEdge ) ++pwChinaHitCount_;
      if ( pwChinaHitCount_ >= 4 ) {
         pwChinaHitCount_ -= 4;
         int grabbed = random( 4 );
         int opposite[ 4 ], oppositeCount = 0;
         for ( int p = 0; p < 4; ++p ) if ( pwChinaPairIsBass_[ p ] != pwChinaPairIsBass_[ grabbed ] ) opposite[ oppositeCount++ ] = p;
         if ( oppositeCount > 0 ) {
            int other = opposite[ random( oppositeCount ) ];
            bool tmp = pwChinaPairIsBass_[ grabbed ];
            pwChinaPairIsBass_[ grabbed ] = pwChinaPairIsBass_[ other ];
            pwChinaPairIsBass_[ other ] = tmp;
         }
      }
      for ( int p = 0; p < 4; ++p ) {
         bool isBass = pwChinaPairIsBass_[ p ];
         uint8_t hitPct = isBass ? bassHit : trebleHit;
         CRGB lit = isBass ? Bcolor : Tcolor;
         lit.nscale8( hitBrightnessByte( hitPct ) );
         for ( int k = 0; k < 2; ++k ) { int c = PW_CHINA_PAIRS[ p ][ k ]; carpet->chinaLeds[ c ] = lit; carpet->chinaLeds[ c ].w = 0; }
      }
   }

   static CRGB vuMeterColor( bool bassLit, bool trebleLit, const CRGB & bassClr, const CRGB & trebleClr ) {
      if ( bassLit && trebleLit ) return blend( bassClr, trebleClr, 128 ); // the "crossing" zone
      if ( bassLit ) return bassClr;
      if ( trebleLit ) return trebleClr;
      return CRGB::Black;
   }

   // triple-strobe: on a qualifying bass hit (same attack condition as the
   // lastlow/vuLastBass peak-hold above: bassRaw > 80 and rising), flash all
   // 8 china and the 8 megabars nearest the 4 corners (idx 1,2,4,5,7,8,10,11
   // -- the ones NOT aligned with an edge midpoint, i.e. every megabar except
   // the headlight[0]/left[3]/back[6]/right[9]) full white for 3 pulses,
   // 30ms on each, 20ms gaps between.
   //
   // Hit suppression: once a hit strobes, its level becomes suppressionPeak_.
   // Any further qualifying hit that doesn't exceed that peak is suppressed
   // (no strobe), as long as it's within 3s of the last qualifying hit of
   // any kind (strobed or suppressed) -- lastHitMillis_ tracks that gap. A
   // gap of 3s+ with no qualifying hits at all clears the suppression, so
   // the next hit always strobes fresh and becomes the new peak, regardless
   // of its level.
   void updateStrobe( uint32_t time ) {
      if ( !strobeEnabled_ ) {
         strobeActive_ = false;
         return;
      }

      bool isHit = AudioBoard::getHitNonzero( BandLow );
      int bassHit = AudioBoard::getHitPercent( BandLow );

      if ( isHit ) {
         static const uint32_t SILENCE_RESET_MS = 3000;
         bool silenceExpired = !hadHit_ || lastHitTimer_.elapsed( SILENCE_RESET_MS );
         bool exceedsPeak = bassHit > suppressionPeak_;
         if ( !strobeActive_ && ( silenceExpired || exceedsPeak ) ) {
            strobeActive_ = true;
            strobeTimer_.reset();
            suppressionPeak_ = bassHit;
         }
         hadHit_ = true;
         lastHitTimer_.reset();
      }

      if ( !strobeActive_ ) return;

      static const uint32_t onMs = 30, gapMs = 20;
      static const uint32_t pulseStart[3] = { 0, onMs + gapMs, 2 * ( onMs + gapMs ) };
      uint32_t elapsed = strobeTimer_.elapsed();
      if ( elapsed >= pulseStart[ 2 ] + onMs ) {
         strobeActive_ = false; // sequence finished -- this frame renders normally
         return;
      }

      bool lit = false;
      for ( int p = 0; p < 3; ++p ) {
         if ( elapsed >= pulseStart[ p ] && elapsed < pulseStart[ p ] + onMs ) { lit = true; break; }
      }

      static const int cornerMegabars[ 8 ] = { 1, 2, 4, 5, 7, 8, 10, 11 };
      for ( int i = 0; i < 8; ++i ) carpet->megabarLeds[ cornerMegabars[ i ] ] = lit ? CRGB::White : CRGB::Black;
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         carpet->chinaLeds[ i ] = lit ? CRGB::White : CRGB::Black;
         carpet->chinaLeds[ i ].w = lit ? 255 : 0;
      }
   }
};
