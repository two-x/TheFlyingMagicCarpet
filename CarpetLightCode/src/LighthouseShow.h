/* LighthouseShow.h
 *
 *    Two independent rotating "lighthouse" beams. Each beam is a single
 *    randomly-drifting angle that lights up BOTH its own angle AND that
 *    angle+180 simultaneously (same color at both) -- so 2 independent
 *    beams produce 4 lit clusters total, on megabars and on the rope.
 *
 *    Per beam, on megabars: the nearest megabar (of the 12, 30 deg apart) to
 *    the beam's current angle is full brightness (V=255); its 2 immediate
 *    neighbors are half brightness (V=128). Same pattern at angle+180.
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
 *    Rope: each of the 4 clusters lights a (30/360 of the full loop)-wide
 *    segment at full brightness in its beam's color. Everywhere else, color
 *    smoothly blends between the two flanking clusters going around the
 *    ring, and brightness dips smoothly from 100% at a segment's edge down
 *    to 50% at the midpoint of the gap and back to 100% at the next
 *    segment -- never a hard cut. Where two clusters' segments overlap
 *    (megabars or rope), hue is additive (summed, wrapped) and brightness
 *    is NOT summed (stays at the flat max).
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

   // merges one cluster's contribution into a megabar slot: first writer
   // sets it; subsequent writers (overlap) sum hue (wrapped) and keep
   // brightness at the max of the two, per request ("additive hue, same brightness")
   void mergeMegabar( bool set[ 12 ], uint8_t hue[ 12 ], uint8_t sat[ 12 ], uint8_t bright[ 12 ],
                       int idx, uint8_t h, uint8_t s, uint8_t b ) {
      if ( !set[ idx ] ) {
         set[ idx ] = true;
         hue[ idx ] = h;
         sat[ idx ] = s;
         bright[ idx ] = b;
      } else {
         hue[ idx ] = (uint8_t)( hue[ idx ] + h ); // wraps -- additive hue
         bright[ idx ] = max( bright[ idx ], b );
      }
   }

   void addMegabarCluster( bool set[ 12 ], uint8_t hue[ 12 ], uint8_t sat[ 12 ], uint8_t bright[ 12 ],
                            float clusterAngle, uint8_t h, uint8_t s ) {
      int nearest = ( (int)( clusterAngle / 30.0f + 0.5f ) ) % 12;
      if ( nearest < 0 ) nearest += 12;
      int neighborA = ( nearest + 11 ) % 12;
      int neighborB = ( nearest + 1 ) % 12;
      mergeMegabar( set, hue, sat, bright, nearest, h, s, 255 );
      mergeMegabar( set, hue, sat, bright, neighborA, h, s, 128 );
      mergeMegabar( set, hue, sat, bright, neighborB, h, s, 128 );
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

      // --- megabars ---
      bool mbSet[ 12 ] = { false };
      uint8_t mbHue[ 12 ], mbSat[ 12 ], mbBright[ 12 ];
      for ( int c = 0; c < 4; ++c ) {
         addMegabarCluster( mbSet, mbHue, mbSat, mbBright, clusterAngle[ c ], clusterHue[ c ], satByte );
      }
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
         carpet->megabarLeds[ i ] = mbSet[ i ] ? (CRGB)CHSV( mbHue[ i ], satByte, mbBright[ i ] ) : CRGB::Black;
      }

      // --- rope ---
      static const float SEGMENT_HALF_WIDTH_DEG = 15.0f; // (30/360 total) wide, so +/-15 from center
      // sorted cluster order, needed to find each background LED's flanking
      // pair -- insertion sort, only 4 elements
      int order[ 4 ] = { 0, 1, 2, 3 };
      for ( int a = 1; a < 4; ++a ) {
         int key = order[ a ], j = a - 1;
         while ( j >= 0 && clusterAngle[ order[ j ] ] > clusterAngle[ key ] ) {
            order[ j + 1 ] = order[ j ];
            --j;
         }
         order[ j + 1 ] = key;
      }

      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         float a = ropeAngleCache_[ i ];

         uint16_t hueSum = 0;
         bool anySet = false;
         for ( int c = 0; c < 4; ++c ) {
            if ( fabsf( circularDelta( a, clusterAngle[ c ] ) ) <= SEGMENT_HALF_WIDTH_DEG ) {
               hueSum += clusterHue[ c ];
               anySet = true;
            }
         }

         CRGB result;
         if ( anySet ) {
            result = CHSV( (uint8_t)hueSum, satByte, 255 );
         } else {
            // find the flanking clusters (trailing/leading) in sorted order
            int leadPos = 0;
            while ( leadPos < 4 && circularDelta( clusterAngle[ order[ leadPos ] ], a ) < 0.0f ) ++leadPos;
            int lead = order[ leadPos % 4 ];
            int trail = order[ ( leadPos + 3 ) % 4 ];

            float gapStart = wrap360( clusterAngle[ trail ] + SEGMENT_HALF_WIDTH_DEG );
            float gapSpan = circularDelta( gapStart, wrap360( clusterAngle[ lead ] - SEGMENT_HALF_WIDTH_DEG ) );
            if ( gapSpan < 0.0f ) gapSpan += 360.0f;
            float into = circularDelta( gapStart, a );
            if ( into < 0.0f ) into += 360.0f;
            float t = ( gapSpan > 0.0f ) ? constrain( into / gapSpan, 0.0f, 1.0f ) : 0.0f;

            CRGB trailClr = CHSV( clusterHue[ trail ], satByte, 255 );
            CRGB leadClr = CHSV( clusterHue[ lead ], satByte, 255 );
            result = blend( trailClr, leadClr, (uint8_t)( t * 255.0f ) );

            // 100% at each edge, dipping to 50% at the gap's midpoint --
            // triwave8 (0->255->0 across the full input range) inverted,
            // no float trig needed (see class comment)
            uint8_t brightness = 255 - ( triwave8( (uint8_t)( t * 255.0f ) ) / 2 );
            result.nscale8( brightness );
         }
         carpet->ropeLeds[ i ] = result;
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
