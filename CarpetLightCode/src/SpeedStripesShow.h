/* SpeedStripesShow.h
 *
 *    Speed-reactive show driven by CANTroller2's vehicle speed telemetry
 *    (see SpeedLink.h). The two long side rope strips are the only fixtures
 *    that physically run front-to-back along the car's 16ft length (see
 *    README.md, "Perimeter rope lights"); each is divided into 4 alternating
 *    lit/dark bands of 88 LEDs (a quarter of the car's length, 4ft) that
 *    scroll from front toward back at a rate proportional to live speed.
 *
 *    Every other fixture that has any meaningful along-the-length position
 *    joins in too, sampling the exact same scrolling pattern at its own
 *    physical position, so a given stripe's color matches everywhere it's
 *    visible at once -- underneath (china), out to the sides (megabars), and
 *    on top (rope). The pattern also extends conceptually beyond the car's
 *    own front/back edges, so the front/rear megabars and china preview a
 *    stripe approaching before it reaches the carpet, and show it receding
 *    after it leaves:
 *
 *      - Front 3 megabars (idx 11, 0, 1 -- the headlight and its neighbors)
 *        show the next stripe, sampled one stripe-width (4ft) ahead of the
 *        front corner -- a preview, well before it arrives.
 *      - Front 2 china aimed along the front edge (idx 1, 2) show that same
 *        stripe, sampled right at a 1ft-from-the-edge point -- picks it up
 *        just as it's about to cross onto the carpet.
 *      - Rear 3 megabars (idx 5, 6, 7) and rear 2 china aimed along the back
 *        edge (idx 5, 6) mirror the above symmetrically for the stripe
 *        exiting the back.
 *      - The 4 china aimed along a side edge (idx 0/7 right, 3/4 left) sample
 *        their documented positions directly: 1/3 and 2/3 of the way back
 *        from the front.
 *      - The remaining 6 side megabars (idx 2,3,4 left; 8,9,10 right) sample
 *        that same 1/3, 1/2, 2/3 partition by angle, so the ones nearest a
 *        side china line up with it exactly.
 *
 *    Positions in this "moving" mode are all expressed in the same
 *    coordinate as renderSide()'s localOffset (0 = back corner,
 *    SIZEOF_LARGE_NEO-1 = front corner) and fed through one continuous,
 *    periodic sampling function -- valid at any real value, including the
 *    "ahead of the front corner" / "behind the back corner" positions the
 *    front/rear fixtures use, no special-casing needed. Stripe edges are a
 *    smoothed (tanh-shaped) square wave rather than a hard cut, per request
 *    -- FADE_STEEPNESS in sampleWave() controls how soft that transition
 *    looks; lower is softer/more sinusoidal, higher is closer to a hard edge.
 *
 *    Stopped mode: whenever speed is 0, or no speed packet has arrived in
 *    over 4 seconds (SpeedLink gone stale), the front-to-back scroll/preview
 *    behavior above is replaced entirely by the same 4-band pattern spinning
 *    clockwise around the carpet's true center at ~0.25Hz (one revolution
 *    every 4s). This needs real geometry -- the carpet is a 16ft x 12ft
 *    rectangle, not a circle, so "angle from center" is computed from each
 *    fixture's actual (x,y) position on that rectangle (ropeAngleDeg()/
 *    chinaAngleDeg()), not approximated from array index. Megabars already
 *    have exact angles (idx*30 deg, per README) and need no such lookup.
 */

#ifndef __SPEED_STRIPES_SHOW_H
#define __SPEED_STRIPES_SHOW_H

#include "LightShow.h"
#include "SpeedLink.h"
#include <math.h>

class SpeedStripesShow : public LightShow {
 private:
   static const int stripeWidth_ = SIZEOF_LARGE_NEO / 4; // 88 LEDs = 4ft, a quarter of the car's 16ft length
   float scrollOffset_ = 0.0f;       // LEDs, grows over time as the moving pattern scrolls front->back
   float rotationPhaseDeg_ = 0.0f;   // degrees, grows over time as the stopped pattern spins clockwise
   uint32_t lastUpdateMillis_ = 0;

   // smoothly-faded color (no hard edges) for an alternating lit/dark
   // pattern at a given position along some periodic axis (period in the
   // same units as pos -- LEDs for the moving mode, degrees for the spin).
   // valid for any real pos, the wave is periodic so it "just works" for
   // positions beyond a strip's own physical range too.
   CRGB sampleWave( float pos, float period, const CRGB & clr ) {
      static const float FADE_STEEPNESS = 3.0f; // higher = harder edges, lower = softer/more sinusoidal
      float wave = tanhf( FADE_STEEPNESS * sinf( 2.0f * PI * pos / period ) ); // smoothed square wave, -1..1
      uint8_t brightness = (uint8_t)( ( 0.5f + 0.5f * wave ) * 255.0f + 0.5f );
      CRGB result = clr;
      result.nscale8( brightness );
      return result;
   }

   // localOffset for a fixture's along-the-length position, given as a
   // fraction measured "back from the front" (matching the README's china
   // wording) -- 0 = at the front corner, 1 = at the back corner.
   float localOffsetForFracBackFromFront( float frac ) {
      return ( SIZEOF_LARGE_NEO - 1 ) * ( 1.0f - frac );
   }

   void renderSide( int backCornerIdx, int frontCornerIdx, const CRGB & clr ) {
      int direction = ( frontCornerIdx > backCornerIdx ) ? 1 : -1;
      for ( int localOffset = 0; localOffset < SIZEOF_LARGE_NEO; ++localOffset ) {
         int idx = backCornerIdx + direction * localOffset;
         carpet->ropeLeds[ idx ] = sampleWave( (float)localOffset + scrollOffset_, stripeWidth_ * 2.0f, clr );
         carpet->ropeLeds[ idx ].w = 0;
      }
   }

   // compass-style angle (degrees, 0-360) of a point (x,y) in feet from the
   // carpet's center, clockwise from front: front=0, right=90, back=180,
   // left=270. x = right/left offset, y = front/back offset.
   float angleDeg( float x, float y ) {
      float a = atan2f( x, y ) * 180.0f / PI;
      if ( a < 0.0f ) a += 360.0f;
      return a;
   }

   // true angular position of a rope LED, computed from its actual physical
   // (x,y) on the carpet's 16ft x 12ft rectangle (half-length 8ft, half-width
   // 6ft) -- not approximated from array index. Index ranges/corner
   // orientations per README's "Perimeter rope lights" channel table.
   float ropeAngleDeg( int i ) {
      static const float halfWidthFt = 6.0f;
      static const float halfLengthFt = 8.0f;
      static const float ledsPerFootWidth = SIZEOF_SMALL_NEO / 12.0f;   // 13
      static const float ledsPerFootLength = SIZEOF_LARGE_NEO / 16.0f;  // 22
      float x, y;
      if ( i < SIZEOF_SMALL_NEO ) { // front edge: idx0 near front-right, idx155 near front-left
         x = halfWidthFt - i / ledsPerFootWidth;
         y = halfLengthFt;
      } else if ( i < SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO ) { // right side: local0 near front, local351 near back
         float local = i - SIZEOF_SMALL_NEO;
         x = halfWidthFt;
         y = halfLengthFt - local / ledsPerFootLength;
      } else if ( i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO ) { // back edge: local0 near back-left, local155 near back-right
         float local = i - ( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO );
         x = -halfWidthFt + local / ledsPerFootWidth;
         y = -halfLengthFt;
      } else { // left side: local0 near back-left, local351 near front-left
         float local = i - ( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO );
         x = -halfWidthFt;
         y = -halfLengthFt + local / ledsPerFootLength;
      }
      return angleDeg( x, y );
   }

   // true angular position of a china fixture, from its corner's actual
   // (x,y) -- the 2 fixtures at a given corner share that corner's angle.
   float chinaAngleDeg( int idx ) {
      static const float halfWidthFt = 6.0f;
      static const float halfLengthFt = 8.0f;
      switch ( idx ) {
         case 0: case 1: return angleDeg(  halfWidthFt,  halfLengthFt ); // front-right
         case 2: case 3: return angleDeg( -halfWidthFt,  halfLengthFt ); // front-left
         case 4: case 5: return angleDeg( -halfWidthFt, -halfLengthFt ); // back-left
         default:        return angleDeg(  halfWidthFt, -halfLengthFt ); // back-right (6, 7)
      }
   }

   // stopped-mode render: the same 4-band pattern, spun clockwise around the
   // carpet's true center at rotationPhaseDeg_'s current phase. Every rope
   // LED (the full loop, unlike the moving mode which leaves the front/back
   // edges off), every megabar, and every china fixture samples the wave at
   // its own true angular position -- minus the phase, so increasing phase
   // moves the pattern toward increasing angle, i.e. clockwise.
   void renderSpin( const CRGB & clr ) {
      static const float period = 180.0f; // 2 bands (90 deg each) per half-turn, 4 total per full turn
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         carpet->ropeLeds[ i ] = sampleWave( ropeAngleDeg( i ) - rotationPhaseDeg_, period, clr );
         carpet->ropeLeds[ i ].w = 0;
      }
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
         carpet->megabarLeds[ i ] = sampleWave( (float)( i * 30 ) - rotationPhaseDeg_, period, clr );
      }
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         carpet->chinaLeds[ i ] = sampleWave( chinaAngleDeg( i ) - rotationPhaseDeg_, period, clr );
         carpet->chinaLeds[ i ].w = 0;
      }
   }

 public:
   SpeedStripesShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ) {}

   void start() {
      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();
      scrollOffset_ = 0.0f;
      rotationPhaseDeg_ = 0.0f;
      lastUpdateMillis_ = 0;
   }

   void update( uint32_t time ) {
      if ( lastUpdateMillis_ == 0 ) lastUpdateMillis_ = time;
      float dtSec = (float)( time - lastUpdateMillis_ ) / 1000.0f;
      lastUpdateMillis_ = time;

      // pot picks the pattern color, same convention as NightriderShow variation 0
      uint8_t hue = carpet->pot->read() / 4;
      CRGB clr = CHSV( hue, 255, 255 );

      // "stopped": speed is 0, or SpeedLink hasn't heard from the vehicle in
      // over 4 seconds (its own staleness threshold, not the 2s default)
      bool fresh = SpeedLink::isFresh( 4000 );
      float speedMph = fresh ? SpeedLink::getSpeedMph() : 0.0f;
      bool stopped = !fresh || ( speedMph == 0.0f );

      if ( stopped ) {
         static const float DEG_PER_SEC = 90.0f; // 0.25Hz = 1 revolution / 4s = 90 deg/s
         rotationPhaseDeg_ += DEG_PER_SEC * dtSec;
         while ( rotationPhaseDeg_ >= 360.0f ) rotationPhaseDeg_ -= 360.0f;
         renderSpin( clr );
         return;
      }

      static const float LEDS_PER_MPH_PER_SEC = 4.0f;
      scrollOffset_ += speedMph * LEDS_PER_MPH_PER_SEC * dtSec;
      float period = stripeWidth_ * 2.0f;
      while ( scrollOffset_ >= period ) scrollOffset_ -= period; // keep it bounded; sampleWave() is periodic anyway

      // front/back rope edges stay off -- they run side to side, not front to back
      for ( int i = 0; i < SIZEOF_SMALL_NEO; ++i ) carpet->ropeLeds[ i ] = CRGB::Black;
      for ( int i = SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; ++i ) {
         carpet->ropeLeds[ i ] = CRGB::Black;
      }

      // right side: back corner near SIZEOF_SMALL_NEO+SIZEOF_LARGE_NEO, front corner near SIZEOF_SMALL_NEO
      renderSide( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO - 1, SIZEOF_SMALL_NEO, clr );
      // left side: back corner near SIZEOF_SMALL_NEO*2+SIZEOF_LARGE_NEO, front corner at the far end of the array
      renderSide( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO, NUM_NEO_LEDS_ACTUAL - 1, clr );

      carpet->clearMegabars();
      carpet->clearChinas();

      static const float CAR_LENGTH_FT = 16.0f; // front-to-back, measured
      static const float LEDS_PER_FOOT = SIZEOF_LARGE_NEO / CAR_LENGTH_FT; // 22
      static const float EDGE_APPROACH_FT = 1.0f; // china "about 1ft from edge" pickup point

      // front: megabars preview a stripe-width (4ft) out; china pick it up right at the 1ft mark
      CRGB frontLeadClr = sampleWave( ( SIZEOF_LARGE_NEO - 1 ) + stripeWidth_ + scrollOffset_, period, clr );
      carpet->megabarLeds[ 11 ] = frontLeadClr;
      carpet->megabarLeds[ 0 ]  = frontLeadClr; // headlight -- brightness still governed separately, see MagicCarpet
      carpet->megabarLeds[ 1 ]  = frontLeadClr;
      CRGB frontEdgeClr = sampleWave( ( SIZEOF_LARGE_NEO - 1 ) + LEDS_PER_FOOT * EDGE_APPROACH_FT + scrollOffset_, period, clr );
      carpet->chinaLeds[ 1 ] = frontEdgeClr; carpet->chinaLeds[ 1 ].w = 0;
      carpet->chinaLeds[ 2 ] = frontEdgeClr; carpet->chinaLeds[ 2 ].w = 0;

      // rear: mirror of the above, behind the back corner
      CRGB rearLeadClr = sampleWave( -(float)stripeWidth_ + scrollOffset_, period, clr );
      carpet->megabarLeds[ 5 ] = rearLeadClr;
      carpet->megabarLeds[ 6 ] = rearLeadClr;
      carpet->megabarLeds[ 7 ] = rearLeadClr;
      CRGB rearEdgeClr = sampleWave( -LEDS_PER_FOOT * EDGE_APPROACH_FT + scrollOffset_, period, clr );
      carpet->chinaLeds[ 5 ] = rearEdgeClr; carpet->chinaLeds[ 5 ].w = 0;
      carpet->chinaLeds[ 6 ] = rearEdgeClr; carpet->chinaLeds[ 6 ].w = 0;

      // side positions, shared by the side china and the remaining side megabars
      float pos13 = localOffsetForFracBackFromFront( 1.0f / 3.0f ) + scrollOffset_;
      float pos12 = localOffsetForFracBackFromFront( 0.5f ) + scrollOffset_;
      float pos23 = localOffsetForFracBackFromFront( 2.0f / 3.0f ) + scrollOffset_;
      CRGB clr13 = sampleWave( pos13, period, clr );
      CRGB clr12 = sampleWave( pos12, period, clr );
      CRGB clr23 = sampleWave( pos23, period, clr );

      // side china: front one on each side at 1/3 back from front, rear one at 2/3
      carpet->chinaLeds[ 0 ] = clr13; carpet->chinaLeds[ 0 ].w = 0; // front-right, right edge
      carpet->chinaLeds[ 7 ] = clr23; carpet->chinaLeds[ 7 ].w = 0; // back-right, right edge
      carpet->chinaLeds[ 3 ] = clr13; carpet->chinaLeds[ 3 ].w = 0; // front-left, left edge
      carpet->chinaLeds[ 4 ] = clr23; carpet->chinaLeds[ 4 ].w = 0; // back-left, left edge

      // remaining 3 megabars per side, same 1/3, 1/2, 2/3 partition by angle
      carpet->megabarLeds[ 2 ]  = clr13; // 60 deg, left, nearer front
      carpet->megabarLeds[ 3 ]  = clr12; // 90 deg, dead left
      carpet->megabarLeds[ 4 ]  = clr23; // 120 deg, left, nearer back
      carpet->megabarLeds[ 10 ] = clr13; // 300 deg, right, nearer front
      carpet->megabarLeds[ 9 ]  = clr12; // 270 deg, dead right
      carpet->megabarLeds[ 8 ]  = clr23; // 240 deg, right, nearer back
   }
};

#endif
