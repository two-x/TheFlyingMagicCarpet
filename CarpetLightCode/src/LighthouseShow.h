/* LighthouseShow.h
 *
 *    Two independent rotating "lighthouse" beams. Each beam is a single
 *    randomly-drifting angle that lights up BOTH its own angle AND that
 *    angle+180 simultaneously (same color at both) -- so 2 independent
 *    beams produce 4 lit clusters total, on megabars and on the rope.
 *
 *    Per beam, on megabars: each of the 12 fixed megabar positions (30 deg
 *    apart) gets a smooth, continuous brightness as a function of its own
 *    angular distance from the beam's current (continuously-moving, never
 *    quantized) angle -- same solid-then-feathered-fade-to-black shape as
 *    the rope (see below), evaluated at 12 discrete points instead of ~1016.
 *    Sampling a smoothly-varying field at sparse fixed points, rather than
 *    the old discrete "nearest gets full, 2 neighbors get a fixed half"
 *    snap, is what fakes a continuous sweep out of only 12 fixed fixtures.
 *
 *    Rotation: each beam's angular velocity (deg/s, range +/-360 = +/-1Hz)
 *    is its own independent random walk -- see RandomWalk below -- that can
 *    drift through zero and reverse direction over time, per request.
 *
 *    Color: beam 1's hue always increases (never reverses -- "always
 *    clockwise" per request), at a rate that itself random-walks between 0
 *    (frozen) and a max of 1 full spectrum cycle per 20 seconds. Saturation
 *    random-walks between 87% and 100%, same mechanism. Beam 2 shares beam
 *    1's saturation exactly and uses the complementary hue (+128), and has
 *    its own independent rotation random walk, but no independent hue/sat
 *    walks of its own.
 *
 *    Rope AND megabars share one falloff shape per cluster (independently
 *    tunable constants per request -- ROPE_SEGMENT_HALF_WIDTH_DEG/
 *    ROPE_FADE_WIDTH_DEG vs MEGABAR_..., currently equal but meant to be
 *    retuned separately after testing): solid full brightness out to
 *    SEGMENT_HALF_WIDTH_DEG (15 deg, i.e. a 30-deg-wide solid segment),
 *    then a smoothstep (flat-tangent, "very smooth") ease down to black
 *    over the next FADE_WIDTH_DEG (7.5 deg) -- so the two "first pure
 *    black" points on either side of one cluster sit 45 deg apart
 *    (15+7.5 on each side). Beyond that, fully black -- no more blending
 *    across the whole gap to the next cluster. Where two clusters'
 *    falloffs overlap (megabars or rope), hue is additive (summed,
 *    wrapped, same as before) and brightness is the max of the
 *    contributors, not summed -- same "additive hue, same brightness"
 *    rule the megabars always used, now applied identically to the rope.
 *
 *    China: all 8 fixtures share one color, crossfading smoothly between
 *    beam 1's and beam 2's color at a rate equal to the average of the two
 *    beams' |angular velocity| -- reusing that deg/s number directly as a
 *    color-phase rate, not a physical angle. Fixed at 80% of the show's own
 *    max output (compounds with the user's committed china brightness %,
 *    same as every other show's raw output does).
 *
 *    Perf note: this show iterates over all 1016 rope LEDs every frame, and
 *    (per SpeedStripesShow's hard-learned lesson) the Due's SAM3X8E has no
 *    hardware FPU -- so per-LED work deliberately avoids sinf/cosf/atan2f
 *    entirely, using FastLED's table-based sin8()/cos8()/triwave8() and a
 *    one-time-cached rope angle geometry (cacheAngles(), computed once in
 *    start(), not per frame) instead.
 */

#ifndef __LIGHTHOUSE_SHOW_H
#define __LIGHTHOUSE_SHOW_H

#include "LightShow.h"
#include <math.h>

class LighthouseShow : public LightShow {
 private:
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
   RandomWalk satWalk_;             // fraction, range 0.87..1.0

   float angle1_ = 0.0f, angle2_ = 180.0f; // deg, 0-360
   float vel1_ = 0.0f, vel2_ = 0.0f;       // deg/s, current (post-random-walk) value
   float hue1_ = 0.0f;                     // 0-255 float for precision; only ever increases
   float chinaPhaseDeg_ = 0.0f;

   Timer frameTimer_; // tracks dt between update() calls

   // pot -> each beam's max rotation speed ceiling, 0.5Hz (180deg/s) to 1Hz
   // (360deg/s), driven by the shared globalEnergyPercent (see LightShow.h)
   // -- adjusting the pot here also becomes the new starting point for
   // NightriderShow's auto-cycle variation and FlameSparkle
   PotEnergyTakeover energyTakeover_;

   float ropeAngleCache_[ NUM_NEO_LEDS_ACTUAL ]; // see class comment: cached once, not per-frame

   // compass-style angle (degrees, 0-360) of a point (x,y) in feet from the
   // carpet's center, clockwise from front: front=0, right=90, back=180,
   // left=270 -- same convention as SpeedStripesShow.
   float angleDeg( float x, float y ) {
      float a = atan2f( x, y ) * 180.0f / PI;
      if ( a < 0.0f ) a += 360.0f;
      return a;
   }

   // true angular position of a rope LED from its actual physical (x,y) on
   // the carpet's 16ft x 12ft rectangle -- same geometry as SpeedStripesShow.
   float ropeAngleDeg( int i ) {
      static const float halfWidthFt = 6.0f;
      static const float halfLengthFt = 8.0f;
      static const float ledsPerFootWidth = SIZEOF_SMALL_NEO / 12.0f;   // 13
      static const float ledsPerFootLength = SIZEOF_LARGE_NEO / 16.0f;  // 22
      float x, y;
      if ( i < SIZEOF_SMALL_NEO ) {
         x = halfWidthFt - i / ledsPerFootWidth;
         y = halfLengthFt;
      } else if ( i < SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO ) {
         float local = i - SIZEOF_SMALL_NEO;
         x = halfWidthFt;
         y = halfLengthFt - local / ledsPerFootLength;
      } else if ( i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO ) {
         float local = i - ( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO );
         x = -halfWidthFt + local / ledsPerFootWidth;
         y = -halfLengthFt;
      } else {
         float local = i - ( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO );
         x = -halfWidthFt;
         y = -halfLengthFt + local / ledsPerFootLength;
      }
      return angleDeg( x, y );
   }

   void cacheAngles() {
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) ropeAngleCache_[ i ] = ropeAngleDeg( i );
   }

   static float wrap360( float deg ) {
      while ( deg < 0.0f ) deg += 360.0f;
      while ( deg >= 360.0f ) deg -= 360.0f;
      return deg;
   }

   // shortest signed distance (deg, -180..180) from a to b going around the circle
   static float circularDelta( float a, float b ) {
      float d = fmodf( b - a + 180.0f, 360.0f );
      if ( d < 0.0f ) d += 360.0f;
      return d - 180.0f;
   }

   // smooth ease from 255 (t=0) down to 0 (t=255) -- flat tangent at both
   // ends ("very smooth", not a linear/triwave8 ramp) -- integer-only
   // smoothstep(1 - t/255)*255, no float trig, no lookup table needed
   static uint8_t smoothstep8( uint8_t t ) {
      uint32_t x = 255 - t;
      uint32_t smooth = ( x * x * ( 3 * 255 - 2 * x ) ) / ( 255ul * 255ul );
      return (uint8_t)smooth;
   }

   // one cluster's brightness contribution (0-255) at angular distance d
   // (0-180 deg) away: solid 255 out to halfWidthDeg, smoothstep fade to 0
   // over the next fadeWidthDeg, flat 0 beyond that -- shared shape for
   // both rope and megabars, just with separately-tunable widths (see
   // class comment)
   static uint8_t clusterBrightness( float d, float halfWidthDeg, float fadeWidthDeg ) {
      if ( d <= halfWidthDeg ) return 255;
      float outer = halfWidthDeg + fadeWidthDeg;
      if ( d >= outer ) return 0;
      float fadeFrac = ( d - halfWidthDeg ) / fadeWidthDeg; // 0..1
      return smoothstep8( (uint8_t)( fadeFrac * 255.0f + 0.5f ) );
   }

 public:
   LighthouseShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ) {}

   void start() {
      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();
      frameTimer_.reset();
      cacheAngles();
      energyTakeover_.reset( carpet );
   }

   void update( uint32_t time ) {
      float dtSec = (float)frameTimer_.elapsed() / 1000.0f;
      frameTimer_.reset();

      // pot -> max rotation speed ceiling, via the shared energy setting
      float energyFrac = energyTakeover_.update( carpet ) / 100.0f;

      // --- random-walked parameters ---
      float velCeilDegPerSec = ( 0.5f + 0.5f * energyFrac ) * 360.0f; // 0.5-1.0Hz -> 180-360deg/s
      vel1_ = velWalk1_.value( -velCeilDegPerSec, velCeilDegPerSec, 10.0f ); // +/-10deg/s step, unchanged
      vel2_ = velWalk2_.value( -velCeilDegPerSec, velCeilDegPerSec, 10.0f );
      static const float MAX_HUE_RATE = 255.0f / 20.0f; // full spectrum in as little as 20s
      float hueRate = hueRateWalk_.value( 0.0f, MAX_HUE_RATE, MAX_HUE_RATE * 0.1f ); // never negative -- always "clockwise"
      float satFraction = satWalk_.value( 0.87f, 1.0f, 0.013f );

      angle1_ = wrap360( angle1_ + vel1_ * dtSec );
      angle2_ = wrap360( angle2_ + vel2_ * dtSec );
      hue1_ += hueRate * dtSec;
      while ( hue1_ >= 256.0f ) hue1_ -= 256.0f;

      uint8_t hue1Byte = (uint8_t)hue1_;
      uint8_t hue2Byte = (uint8_t)( hue1Byte + 128 ); // complementary, wraps
      uint8_t satByte = (uint8_t)( satFraction * 255.0f + 0.5f );

      // the 4 lit clusters: beam 1 at its own angle and angle+180 (same
      // color both places), beam 2 likewise
      float clusterAngle[ 4 ] = { angle1_, wrap360( angle1_ + 180.0f ), angle2_, wrap360( angle2_ + 180.0f ) };
      uint8_t clusterHue[ 4 ] = { hue1Byte, hue1Byte, hue2Byte, hue2Byte };

      // --- megabars: smooth per-position falloff, sampled at each of the
      // 12 fixed 30-deg-apart positions -- see class comment. Independent
      // constants from the rope's, by request (tune separately after testing).
      // Fade set to 5deg/side (solid core left at 15deg, i.e. still a
      // 30deg-wide solid beam) to help smooth the megabar-to-megabar travel
      // -- total reach between first-black points on each side: 40deg.
      static const float MEGABAR_SEGMENT_HALF_WIDTH_DEG = 15.0f;
      static const float MEGABAR_FADE_WIDTH_DEG = 5.0f;
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         float megabarAngle = m * 30.0f;
         uint16_t hueSum = 0;
         uint8_t maxBright = 0;
         bool anySet = false;
         for ( int c = 0; c < 4; ++c ) {
            float d = fabsf( circularDelta( megabarAngle, clusterAngle[ c ] ) );
            uint8_t contribBright = clusterBrightness( d, MEGABAR_SEGMENT_HALF_WIDTH_DEG, MEGABAR_FADE_WIDTH_DEG );
            if ( contribBright > 0 ) {
               hueSum += clusterHue[ c ];
               maxBright = max( maxBright, contribBright );
               anySet = true;
            }
         }
         carpet->megabarLeds[ m ] = anySet ? (CRGB)CHSV( (uint8_t)hueSum, satByte, maxBright ) : CRGB::Black;
      }

      // --- rope: same falloff shape, own constants, evaluated at each of
      // the ~1016 rope LEDs' actual angular positions. No more blending
      // across the whole gap between clusters -- fully black beyond the
      // fade width now, per request.
      static const float ROPE_SEGMENT_HALF_WIDTH_DEG = 15.0f;
      static const float ROPE_FADE_WIDTH_DEG = 7.5f;
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         float a = ropeAngleCache_[ i ];

         uint16_t hueSum = 0;
         uint8_t maxBright = 0;
         bool anySet = false;
         for ( int c = 0; c < 4; ++c ) {
            float d = fabsf( circularDelta( a, clusterAngle[ c ] ) );
            uint8_t contribBright = clusterBrightness( d, ROPE_SEGMENT_HALF_WIDTH_DEG, ROPE_FADE_WIDTH_DEG );
            if ( contribBright > 0 ) {
               hueSum += clusterHue[ c ];
               maxBright = max( maxBright, contribBright );
               anySet = true;
            }
         }

         carpet->ropeLeds[ i ] = anySet ? (CRGB)CHSV( (uint8_t)hueSum, satByte, maxBright ) : CRGB::Black;
         carpet->ropeLeds[ i ].w = 0;
      }

      // --- china: shared crossfade between the 2 beams' colors ---
      float avgAbsVel = ( fabsf( vel1_ ) + fabsf( vel2_ ) ) / 2.0f; // deg/s, reused directly as a color-phase rate
      chinaPhaseDeg_ = wrap360( chinaPhaseDeg_ + avgAbsVel * dtSec );
      uint8_t chinaPhase8 = (uint8_t)( chinaPhaseDeg_ / 360.0f * 255.0f );
      uint8_t chinaBlend = sin8( chinaPhase8 ); // smooth 0-255 oscillation, no float trig
      CRGB chinaColor = blend( (CRGB)CHSV( hue1Byte, satByte, 255 ), (CRGB)CHSV( hue2Byte, satByte, 255 ), chinaBlend );
      chinaColor.nscale8( 204 ); // 80% of this show's own max output
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         carpet->chinaLeds[ i ] = chinaColor;
         carpet->chinaLeds[ i ].w = 0;
      }
   }
};

#endif
