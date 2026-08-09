/* LighthouseShow.h
 *
 *    Two independent rotating "lighthouse" beams, each a pair of opposite-
 *    facing (180deg apart) cones, point at the car's own center, expanding
 *    outward. Rewritten to be based on real angle math throughout (was
 *    previously "patched together": megabars used a discrete nearest-
 *    neighbor snap instead of the rope's own continuous falloff, cone
 *    width didn't match spec, and overlapping beams summed hues together,
 *    which reads as a muddy wash rather than either beam's real color) --
 *    see the class's own per-section comments below for exactly what
 *    changed and why.
 *
 *    THE ALGORITHM (per explicit spec): each beam is 2 cones, fixed
 *    180deg apart, cone tips at the origin. Each cone's leading/trailing
 *    edges are a fixed CONE_FULL_WIDTH_DEG (25.3deg) apart -- a light at
 *    some angle is lit at full brightness whenever it's within that
 *    25.3deg window, with an additional angle-based fade zone
 *    (CONE_FADE_DEG, 25% of the cone's own width) on each side easing
 *    down to black. This ONE falloff function (coneBrightnessAt()) is now
 *    used identically for rope, megabars, AND china -- evaluated at each
 *    fixture's own real physical angle (from CarpetGeometry, the
 *    project's shared centralized geometry source), not a per-fixture-
 *    type bespoke shape. That's what was actually causing "neopixel
 *    segment widths don't match the floods": megabars and rope were
 *    using two different width models entirely.
 *
 *    Where the 2 beams' cones overlap: NO additive color mixing (reads as
 *    a muddy wash) -- per explicit correction, only the brighter of the 2
 *    beams shows at all, using that beam's own exact color, at that
 *    beam's own computed brightness.
 *
 *    Rotation/color: each beam's angular velocity (deg/s, range +/-360 =
 *    +/-1Hz) is its own independent random walk -- see RandomWalk below --
 *    that can drift through zero and reverse direction over time, per
 *    request; angles themselves stay full float precision throughout,
 *    never quantized. Beam 1's hue always increases (never reverses --
 *    "always clockwise" per request), at a rate that itself random-walks
 *    between 0 (frozen) and a max of 1 full spectrum cycle per 20
 *    seconds. Saturation random-walks between 75% and 100%, same
 *    mechanism. Beam 2 shares beam 1's saturation exactly and uses the
 *    complementary hue (+128), and has its own independent rotation
 *    random walk, but no independent hue/sat walks of its own.
 *
 *    Variations:
 *      0 Default   -- rope+megabar join both beams; china stays off except
 *                      for the bass-hit strobe.
 *      1 No Strobe -- rope+megabar as Default, but china ALSO joins both
 *                      beams (same beam-crossing effect as rope/megabar,
 *                      at each china's own position angle) instead of
 *                      strobing -- this variation used to mean "no china
 *                      at all"; per request that's now Default's job
 *                      (unchanged from its original behavior) and THIS
 *                      variation is the one where china contributes to
 *                      the beam effect.
 *      2 China React -- rope+megabar still sweep the 2 beams as normal;
 *                      chinas switch entirely to a discrete, non-
 *                      continuous audio response instead (see
 *                      updateChinaReact() below) -- 2 of the 4 china
 *                      pairs assigned to bass, 2 to treble (same pairing/
 *                      swap-every-4-hits mechanic as EqualizerShow's
 *                      pixel_war variation), both bands rendered in WHITE
 *                      (not the beams' own colors), using a dedicated
 *                      edge-triggered flash (newHitEdge()) instead of the
 *                      usual smooth hit-percent value: on each qualifying
 *                      leading edge a pair jumps STRAIGHT to 10% of
 *                      global max, holds CHINA_REACT_HOLD_MS (110ms), then
 *                      decays back to black over 1/4 of AudioBoard's own
 *                      configured hit-decay duration (i.e. 4x faster than
 *                      a normal hit's own decay).
 *
 *    Perf note: this show iterates over all 1016 rope LEDs every frame,
 *    and (per SpeedStripesShow's hard-learned lesson) the Due's SAM3X8E
 *    has no hardware FPU -- CarpetGeometry's neopixel table is precomputed
 *    once at boot (not here), so the per-LED cost here is just the
 *    circular-distance/falloff math, no sin/cos/atan2 per LED.
 */

#ifndef __LIGHTHOUSE_SHOW_H
#define __LIGHTHOUSE_SHOW_H

#include "LightShow.h"
#include "CarpetGeometry.h"
#include "AudioBoard.h"
#include <math.h>

class LighthouseShow : public LightShow {
 private:
   enum Variation { VarDefault = 0, VarNoStrobe = 1, VarChinaReact = 2 };
   static const uint8_t numVariations_ = 3;
   uint8_t variation_;

   // smoothly ramps toward a freshly-randomized target every ~0.8-1.2s,
   // rather than jumping -- used identically for both beams' rotation
   // velocity, the shared hue rate, and the shared saturation.
   struct RandomWalk {
      float rampStart = 0.0f;
      float rampTarget = 0.0f;
      Timer tickTimer;
      bool initialized = false;

      float value( float minVal, float maxVal, float maxStep ) {
         if ( !initialized ) {
            initialized = true;
            tickTimer.set( 800 + random( 400 ) );
            rampStart = rampTarget = minVal + ( maxVal - minVal ) * 0.5f; // start at midpoint
         }
         if ( tickTimer.expired() ) {
            rampStart = rampTarget;
            float step = ( (float)random( -1000, 1001 ) / 1000.0f ) * maxStep;
            rampTarget = constrain( rampStart + step, minVal, maxVal );
            tickTimer.set( 800 + random( 400 ) );
         }
         float frac = (float)tickTimer.elapsed() / (float)tickTimer.timeout();
         if ( frac > 1.0f ) frac = 1.0f;
         return rampStart + ( rampTarget - rampStart ) * frac;
      }
   };

   RandomWalk velWalk1_, velWalk2_; // deg/s, range +/-(ceiling from globalEnergyPercent)
   RandomWalk hueRateWalk_;         // hue-units/sec, range 0..(255/20)
   RandomWalk satWalk_;             // fraction, range 0.75..1.0, starts at 0.90

   float angle1_ = 0.0f, angle2_ = 180.0f; // deg, 0-360, full float precision throughout
   float vel1_ = 0.0f, vel2_ = 0.0f;       // deg/s, current (post-random-walk) value
   float hue1_ = 0.0f;                     // 0-255 float for precision; only ever increases

   Timer frameTimer_; // tracks dt between update() calls

   // pot -> each beam's max rotation speed ceiling -- see class comment
   PotEnergyTakeover energyTakeover_;

   // --- china-react variation (VarChinaReact) state ---
   static const int NUM_CHINA_PAIRS_ = 4;
   static const int chinaPairs_[ NUM_CHINA_PAIRS_ ][ 2 ];
   bool chinaPairIsBass_[ NUM_CHINA_PAIRS_ ] = { true, true, false, false };
   int chinaHitCount_ = 0;
   uint32_t chinaPairFlashUntilMs_[ NUM_CHINA_PAIRS_ ] = { 0, 0, 0, 0 };
   uint8_t prevBassPct_ = 0, prevTreblePct_ = 0;

   static float wrap360( float deg ) {
      while ( deg < 0.0f ) deg += 360.0f;
      while ( deg >= 360.0f ) deg -= 360.0f;
      return deg;
   }

   // shortest signed distance (deg, -180..180) from a to b going around the
   // circle -- avoids fmodf's heavier software-emulated division cost
   // (this runs thousands of times/frame across the rope loop, the
   // hottest float call site in this codebase).
   static float circularDelta( float a, float b ) {
      float d = b - a + 180.0f;
      if ( d < 0.0f ) d += 360.0f;
      else if ( d >= 360.0f ) d -= 360.0f;
      return d - 180.0f;
   }

   // smooth ease from 255 (t=0) down to 0 (t=255) -- flat tangent at both
   // ends ("very smooth", not a linear ramp) -- integer-only
   // smoothstep(1 - t/255)*255, no float trig, no lookup table needed
   static uint8_t smoothstep8( uint8_t t ) {
      uint32_t x = 255 - t;
      uint32_t smooth = ( x * x * ( 3 * 255 - 2 * x ) ) / ( 255ul * 255ul );
      return (uint8_t)smooth;
   }

   // Per spec: each cone is CONE_FULL_WIDTH_DEG (25.3deg) wide, leading to
   // trailing edge -- full brightness inside that window, then an
   // additional fade zone on each side (25% of the cone's own width) easing
   // to black, nothing beyond that. ONE shared shape for rope/megabar/china
   // alike (see class comment) -- no more separately-tuned per-fixture-type
   // widths.
   static constexpr float CONE_FULL_WIDTH_DEG = 25.3125f;                          // 33.75 * 0.75 -- another 25% narrower per follow-up request (was 45 originally)
   static constexpr float CONE_HALF_WIDTH_DEG = CONE_FULL_WIDTH_DEG / 2.0f;       // 12.65625
   static constexpr float CONE_FADE_DEG = CONE_FULL_WIDTH_DEG * 0.25f;            // 6.328125, each side
   static const uint32_t CHINA_REACT_HOLD_MS = 220; // doubled per request -- VarChinaReact's full-brightness hold before its trailing decay kicks in, see updateChinaReact()
   static uint8_t coneBrightnessAt( float d ) {
      if ( d <= CONE_HALF_WIDTH_DEG ) return 255;
      float outer = CONE_HALF_WIDTH_DEG + CONE_FADE_DEG;
      if ( d >= outer ) return 0;
      float fadeFrac = ( d - CONE_HALF_WIDTH_DEG ) / CONE_FADE_DEG; // 0..1
      return smoothstep8( (uint8_t)( fadeFrac * 255.0f + 0.5f ) );
   }

   // One beam's brightness contribution at a given point angle -- nearest
   // of its 2 opposite (180deg apart) cones.
   static uint8_t beamBrightnessAt( float pointAngle, float beamAngle ) {
      float d1 = fabsf( circularDelta( pointAngle, beamAngle ) );
      float d2 = fabsf( circularDelta( pointAngle, wrap360( beamAngle + 180.0f ) ) );
      return coneBrightnessAt( min( d1, d2 ) );
   }

   // BUGFIX (per explicit correction -- "no additive mixing, that looks
   // terrible"): this used to sum both beams' hues together wherever
   // their falloffs overlapped, which reads as a muddy blend rather than
   // either beam's real color. Now picks the single brighter beam
   // entirely -- its own hue, at its own computed brightness -- with no
   // blending of any kind between the two.
   static CRGB winnerColorAt( float pointAngle, float angle1, uint8_t hue1, float angle2, uint8_t hue2, uint8_t satByte ) {
      uint8_t b1 = beamBrightnessAt( pointAngle, angle1 );
      uint8_t b2 = beamBrightnessAt( pointAngle, angle2 );
      uint8_t winBright = max( b1, b2 );
      if ( winBright == 0 ) return CRGB::Black;
      uint8_t winHue = ( b1 >= b2 ) ? hue1 : hue2;
      return (CRGB)CHSV( winHue, satByte, winBright );
   }

   // Edge-triggered "new hit" detector for VarChinaReact -- deliberately
   // NOT AudioBoard::getHitPercent() (the usual smoothly-decaying hit
   // value every other show uses) per explicit request; this just watches
   // the live post-gain level cross up through the same 20% threshold
   // convention used elsewhere in this codebase for edge detection.
   static bool newHitEdge( AudioBand band, uint8_t & prevPct ) {
      uint8_t pct = AudioBoard::getNormalPercent( band );
      bool edge = pct > 20 && prevPct <= 20;
      prevPct = pct;
      return edge;
   }

   void updateChinaReact( uint32_t time ) {
      bool bassEdge = newHitEdge( BandBass, prevBassPct_ );
      bool trebleEdge = newHitEdge( BandTreble, prevTreblePct_ );
      if ( bassEdge || trebleEdge ) ++chinaHitCount_;
      if ( bassEdge || trebleEdge ) {
         for ( int p = 0; p < NUM_CHINA_PAIRS_; ++p ) {
            if ( ( chinaPairIsBass_[ p ] && bassEdge ) || ( !chinaPairIsBass_[ p ] && trebleEdge ) ) {
               chinaPairFlashUntilMs_[ p ] = time + CHINA_REACT_HOLD_MS; // straight to 10% global max, holds this long, then decays -- see the render loop below
            }
         }
      }
      if ( chinaHitCount_ >= 4 ) {
         chinaHitCount_ -= 4;
         int grabbed = random( NUM_CHINA_PAIRS_ );
         int opposite[ NUM_CHINA_PAIRS_ ], oppositeCount = 0;
         for ( int p = 0; p < NUM_CHINA_PAIRS_; ++p ) if ( chinaPairIsBass_[ p ] != chinaPairIsBass_[ grabbed ] ) opposite[ oppositeCount++ ] = p;
         if ( oppositeCount > 0 ) {
            int other = opposite[ random( oppositeCount ) ];
            bool tmp = chinaPairIsBass_[ grabbed ];
            chinaPairIsBass_[ grabbed ] = chinaPairIsBass_[ other ];
            chinaPairIsBass_[ other ] = tmp;
         }
      }
      static const uint8_t FLASH_BRIGHTNESS = 26; // 10% of 255
      for ( int p = 0; p < NUM_CHINA_PAIRS_; ++p ) {
         uint8_t bright = 0;
         if ( time < chinaPairFlashUntilMs_[ p ] ) {
            bright = FLASH_BRIGHTNESS; // instant attack straight to full, holds at CHINA_REACT_HOLD_MS, per request
         } else {
            // trailing decay, per request: same shape as AudioBoard's own
            // hit-decay ramp, but at 1/4 its currently configured duration
            // (i.e. 4x faster) -- a quick snappy tail rather than a lingering
            // one, distinct from the instant-cutoff this used to have.
            float decayMs = AudioBoard::getHitDecayMs() / 4.0f;
            float elapsedMs = (float)( time - chinaPairFlashUntilMs_[ p ] );
            if ( decayMs > 0.0f && elapsedMs < decayMs ) {
               bright = (uint8_t)( (float)FLASH_BRIGHTNESS * ( 1.0f - elapsedMs / decayMs ) + 0.5f );
            }
         }
         CRGB clr = CRGB( bright, bright, bright );
         for ( int k = 0; k < 2; ++k ) {
            int c = chinaPairs_[ p ][ k ];
            carpet->chinaLeds[ c ] = clr;
            carpet->chinaLeds[ c ].w = 0;
         }
      }
   }

 public:
   LighthouseShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ) {}

   uint8_t variation() { return variation_; }
   const char * variationName() {
      if ( variation_ == VarDefault ) return "default";
      if ( variation_ == VarNoStrobe ) return "no strobe";
      return "china react";
   }

   void start() {
      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();
      frameTimer_.reset();
      energyTakeover_.reset( carpet );

      // seed each beam's initial rotation speed directly (RandomWalk::value()
      // would otherwise auto-init both to the range midpoint, i.e. 0deg/s,
      // on its first call) -- beam 1 starts clockwise at 5s/rotation
      // (72deg/s), beam 2 starts counterclockwise at 2s/rotation (-180deg/s),
      // per request. From here both drift normally via the random walk.
      velWalk1_.rampStart = velWalk1_.rampTarget = 72.0f;
      velWalk1_.initialized = true;
      velWalk1_.tickTimer.set( 800 + random( 400 ) );
      velWalk2_.rampStart = velWalk2_.rampTarget = -180.0f;
      velWalk2_.initialized = true;
      velWalk2_.tickTimer.set( 800 + random( 400 ) );

      // seed saturation's start explicitly too (same reasoning as the
      // velocity walks above -- RandomWalk::value() would otherwise
      // auto-init to the range midpoint, 87.5%, not the requested 90%)
      satWalk_.rampStart = satWalk_.rampTarget = 0.90f;
      satWalk_.initialized = true;
      satWalk_.tickTimer.set( 800 + random( 400 ) );

      int idxs[ NUM_CHINA_PAIRS_ ] = { 0, 1, 2, 3 };
      for ( int i = NUM_CHINA_PAIRS_ - 1; i > 0; --i ) { int j = random( i + 1 ); int t = idxs[ i ]; idxs[ i ] = idxs[ j ]; idxs[ j ] = t; }
      for ( int p = 0; p < NUM_CHINA_PAIRS_; ++p ) chinaPairIsBass_[ p ] = false;
      chinaPairIsBass_[ idxs[ 0 ] ] = true; chinaPairIsBass_[ idxs[ 1 ] ] = true;
      chinaHitCount_ = 0;
      prevBassPct_ = 0; prevTreblePct_ = 0;
      for ( int p = 0; p < NUM_CHINA_PAIRS_; ++p ) chinaPairFlashUntilMs_[ p ] = 0;
   }

   void update( uint32_t time ) {
      // variation select: each encoder detent moves to the next variation,
      // same convention as every other multi-variation show
      int varDelta = carpet->encoder->readPositionDelta();
      carpet->encoder->resetPositionDelta();
      if ( varDelta != 0 ) {
         int newVariation = ( (int)variation_ + varDelta ) % (int)numVariations_;
         if ( newVariation < 0 ) newVariation += numVariations_;
         variation_ = (uint8_t)newVariation;
      }

      float dtSec = (float)frameTimer_.elapsed() / 1000.0f;
      frameTimer_.reset();

      // pot -> rotation speed attenuation, via the shared energy setting --
      // applied AFTER the random walk (see below), not by narrowing its
      // range, per request ("before any rotational speed limit attenuation
      // set by the potentiometer")
      float energyFrac = energyTakeover_.update( carpet ) / 100.0f;

      // --- random-walked parameters (angles/rates stay full float
      // precision throughout -- see class comment) ---
      static const float VEL_CEIL_DEG_PER_SEC = 360.0f;
      float rawVel1 = velWalk1_.value( -VEL_CEIL_DEG_PER_SEC, VEL_CEIL_DEG_PER_SEC, 10.0f );
      float rawVel2 = velWalk2_.value( -VEL_CEIL_DEG_PER_SEC, VEL_CEIL_DEG_PER_SEC, 10.0f );
      vel1_ = rawVel1 * energyFrac;
      vel2_ = rawVel2 * energyFrac;
      static const float MAX_HUE_RATE = 255.0f / 20.0f; // full spectrum in as little as 20s
      float hueRate = hueRateWalk_.value( 0.0f, MAX_HUE_RATE, MAX_HUE_RATE * 0.1f ); // never negative -- always "clockwise"
      float satFraction = satWalk_.value( 0.75f, 1.0f, 0.013f );

      angle1_ = wrap360( angle1_ + vel1_ * dtSec );
      angle2_ = wrap360( angle2_ + vel2_ * dtSec );
      hue1_ += hueRate * dtSec;
      while ( hue1_ >= 256.0f ) hue1_ -= 256.0f;

      uint8_t hue1Byte = (uint8_t)hue1_;
      uint8_t hue2Byte = (uint8_t)( hue1Byte + 128 ); // complementary, wraps
      uint8_t satByte = (uint8_t)( satFraction * 255.0f + 0.5f );

      // --- rope: evaluated at each LED's own real angle, from
      // CarpetGeometry's shared, precomputed-once neopixel table (no more
      // per-show angle cache/derivation).
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         float a = CarpetGeometry::getNeoGeom( i ).angleFromForwardDeg;
         carpet->ropeLeds[ i ] = winnerColorAt( a, angle1_, hue1Byte, angle2_, hue2Byte, satByte );
         carpet->ropeLeds[ i ].w = 0;
      }

      // --- megabars: SAME falloff function as the rope, evaluated at each
      // megabar's own real position angle (CarpetGeometry) -- no more
      // separate discrete nearest-neighbor snap (that was the actual cause
      // of "neopixel segment widths don't match the floods").
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         float a = CarpetGeometry::getMegabarPosition( m ).positionAngleDeg; // same field china uses below -- for megabars this numerically equals beamAngleDeg, but positionAngleDeg is the semantically correct one to ask for here
         carpet->megabarLeds[ m ] = winnerColorAt( a, angle1_, hue1Byte, angle2_, hue2Byte, satByte );
      }

      // --- china: variation-dependent (see class comment) ---
      // BUGFIX/UNDO: china-beam-crossing was originally requested as its
      // OWN separate variation (replacing the old "no chinas" one), not as
      // an addition to Default -- an earlier pass here mistakenly added it
      // to Default too. Default now goes back to leaving china off except
      // for the bass-hit strobe below (its original behavior); only
      // VarNoStrobe carries the beam-crossing china effect.
      if ( variation_ == VarChinaReact ) {
         updateChinaReact( time );
      } else if ( variation_ == VarNoStrobe ) {
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
            // positionAngleDeg (precomputed once in CarpetGeometry::begin(),
            // not a live atan2f here) -- POSITION angle, not aim; see
            // CarpetGeometry.h's china caveat for why those two differ
            float a = CarpetGeometry::getChinaPosition( c ).positionAngleDeg;
            CRGB clr = winnerColorAt( a, angle1_, hue1Byte, angle2_, hue2Byte, satByte );
            carpet->chinaLeds[ c ] = clr;
            carpet->chinaLeds[ c ].w = 0;
         }
      } else { // VarDefault -- china off except the strobe below
         carpet->clearChinas();
      }

      // bass-hit china strobe -- Default only (VarNoStrobe drops it per
      // its own name/request; VarChinaReact's own china behavior takes
      // priority there instead)
      if ( variation_ == VarDefault ) updateStrobe_( time );
   }

 private:
   // triple-strobe on bass hits -- same timing/suppression convention as
   // EqualizerShow's own strobe (BumpingAudioShow.h): 3 pulses, 30ms on/
   // 20ms gap, with hit-suppression so it doesn't refire on every single
   // hit at the same level.
   bool strobeActive_ = false;
   bool hadHit_ = false;
   Timer strobeTimer_, lastHitTimer_;
   int suppressionPeak_ = 0;
   void updateStrobe_( uint32_t time ) {
      bool isHit = AudioBoard::getHitNonzero( BandBass );
      int bassHit = AudioBoard::getHitPercent( BandBass );
      if ( isHit ) {
         static const uint32_t SILENCE_RESET_MS = 3000;
         bool silenceExpired = !hadHit_ || lastHitTimer_.elapsed( SILENCE_RESET_MS );
         bool exceedsPeak = bassHit > suppressionPeak_;
         if ( !strobeActive_ && ( silenceExpired || exceedsPeak ) ) {
            strobeActive_ = true; strobeTimer_.reset(); suppressionPeak_ = bassHit;
         }
         hadHit_ = true; lastHitTimer_.reset();
      }
      if ( !strobeActive_ ) return;
      static const uint32_t onMs = 30, gapMs = 20;
      static const uint32_t pulseStart[ 3 ] = { 0, onMs + gapMs, 2 * ( onMs + gapMs ) };
      uint32_t elapsed = strobeTimer_.elapsed();
      if ( elapsed >= pulseStart[ 2 ] + onMs ) { strobeActive_ = false; return; }
      bool lit = false;
      for ( int p = 0; p < 3; ++p ) if ( elapsed >= pulseStart[ p ] && elapsed < pulseStart[ p ] + onMs ) { lit = true; break; }
      if ( !lit ) return;
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) { carpet->chinaLeds[ c ] = CRGB::White; carpet->chinaLeds[ c ].w = 255; }
   }
};

const int LighthouseShow::chinaPairs_[ LighthouseShow::NUM_CHINA_PAIRS_ ][ 2 ] = { { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 } };

#endif
