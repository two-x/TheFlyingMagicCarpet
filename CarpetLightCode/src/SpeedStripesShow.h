/* SpeedStripesShow.h
 *
 *    Speed-reactive show driven by CANTroller2's vehicle speed telemetry
 *    (see SpeedLink.h). The two long side rope strips are the only fixtures
 *    that physically run front-to-back along the car's 16ft length (see
 *    README.md, "Perimeter rope lights"); each is divided into 4 alternating
 *    lit/dark bands of 88 LEDs (a quarter of the car's length, 4ft) that
 *    scroll from front toward back at a rate proportional to live speed --
 *    stripes sit still at a standstill or with no fresh telemetry.
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
 *    Positions are all expressed in the same coordinate as renderSide()'s
 *    localOffset (0 = back corner, SIZEOF_LARGE_NEO-1 = front corner) and
 *    fed through one continuous, periodic sampling function -- valid at any
 *    real value, including the "ahead of the front corner" / "behind the
 *    back corner" positions the front/rear fixtures use, no special-casing
 *    needed. Stripe edges are a smoothed (tanh-shaped) square wave rather
 *    than a hard cut, per request -- FADE_STEEPNESS in sampleStripe() controls
 *    how soft that transition looks; lower is softer/more sinusoidal, higher
 *    is closer to a hard edge.
 */

#ifndef __SPEED_STRIPES_SHOW_H
#define __SPEED_STRIPES_SHOW_H

#include "LightShow.h"
#include "SpeedLink.h"
#include <math.h>

class SpeedStripesShow : public LightShow {
 private:
   static const int stripeWidth_ = SIZEOF_LARGE_NEO / 4; // 88 LEDs = 4ft, a quarter of the car's 16ft length
   float scrollOffset_ = 0.0f; // LEDs, grows over time as the pattern scrolls front->back
   uint32_t lastUpdateMillis_ = 0;

   // smoothly-faded color (no hard edges) for the alternating lit/dark
   // stripe pattern at a given position along the length axis. valid for any
   // real pos, including beyond the physical strip's 0..SIZEOF_LARGE_NEO-1
   // range -- the wave is periodic, so it "just works" for the front/rear
   // preview positions out past the car's own corners.
   CRGB sampleStripe( float pos, const CRGB & clr ) {
      static const float FADE_STEEPNESS = 3.0f; // higher = harder edges, lower = softer/more sinusoidal
      float period = stripeWidth_ * 2.0f;
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
         carpet->ropeLeds[ idx ] = sampleStripe( (float)localOffset + scrollOffset_, clr );
         carpet->ropeLeds[ idx ].w = 0;
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
      lastUpdateMillis_ = 0;
   }

   void update( uint32_t time ) {
      if ( lastUpdateMillis_ == 0 ) lastUpdateMillis_ = time;
      float dtSec = (float)( time - lastUpdateMillis_ ) / 1000.0f;
      lastUpdateMillis_ = time;

      static const float LEDS_PER_MPH_PER_SEC = 4.0f;
      float speedMph = SpeedLink::isFresh() ? SpeedLink::getSpeedMph() : 0.0f;
      scrollOffset_ += speedMph * LEDS_PER_MPH_PER_SEC * dtSec;
      float period = stripeWidth_ * 2.0f;
      while ( scrollOffset_ >= period ) scrollOffset_ -= period; // keep it bounded; sampleStripe() is periodic anyway

      // pot picks the stripe color, same convention as NightriderShow variation 0
      uint8_t hue = carpet->pot->read() / 4;
      CRGB clr = CHSV( hue, 255, 255 );

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
      CRGB frontLeadClr = sampleStripe( ( SIZEOF_LARGE_NEO - 1 ) + stripeWidth_ + scrollOffset_, clr );
      carpet->megabarLeds[ 11 ] = frontLeadClr;
      carpet->megabarLeds[ 0 ]  = frontLeadClr; // headlight -- brightness still governed separately, see MagicCarpet
      carpet->megabarLeds[ 1 ]  = frontLeadClr;
      CRGB frontEdgeClr = sampleStripe( ( SIZEOF_LARGE_NEO - 1 ) + LEDS_PER_FOOT * EDGE_APPROACH_FT + scrollOffset_, clr );
      carpet->chinaLeds[ 1 ] = frontEdgeClr; carpet->chinaLeds[ 1 ].w = 0;
      carpet->chinaLeds[ 2 ] = frontEdgeClr; carpet->chinaLeds[ 2 ].w = 0;

      // rear: mirror of the above, behind the back corner
      CRGB rearLeadClr = sampleStripe( -(float)stripeWidth_ + scrollOffset_, clr );
      carpet->megabarLeds[ 5 ] = rearLeadClr;
      carpet->megabarLeds[ 6 ] = rearLeadClr;
      carpet->megabarLeds[ 7 ] = rearLeadClr;
      CRGB rearEdgeClr = sampleStripe( -LEDS_PER_FOOT * EDGE_APPROACH_FT + scrollOffset_, clr );
      carpet->chinaLeds[ 5 ] = rearEdgeClr; carpet->chinaLeds[ 5 ].w = 0;
      carpet->chinaLeds[ 6 ] = rearEdgeClr; carpet->chinaLeds[ 6 ].w = 0;

      // side positions, shared by the side china and the remaining side megabars
      float pos13 = localOffsetForFracBackFromFront( 1.0f / 3.0f ) + scrollOffset_;
      float pos12 = localOffsetForFracBackFromFront( 0.5f ) + scrollOffset_;
      float pos23 = localOffsetForFracBackFromFront( 2.0f / 3.0f ) + scrollOffset_;
      CRGB clr13 = sampleStripe( pos13, clr );
      CRGB clr12 = sampleStripe( pos12, clr );
      CRGB clr23 = sampleStripe( pos23, clr );

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
