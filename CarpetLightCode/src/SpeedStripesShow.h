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
 *        that same 1/3, 1/2, 2/3 partition, so the ones nearest a side china
 *        line up with it exactly.
 *
 *    Positions are all expressed in the same coordinate as renderSide()'s
 *    localOffset (0 = back corner, SIZEOF_LARGE_NEO-1 = front corner) and
 *    fed through one continuous, periodic sampling function -- valid at any
 *    real value, including the "ahead of the front corner" / "behind the
 *    back corner" positions the front/rear fixtures use, no special-casing
 *    needed. Stripe edges are a smoothed square wave rather than a hard cut
 *    -- FADE_CONTRAST in sampleWave() controls how soft that transition
 *    looks; lower is softer/more analog, higher is closer to a hard edge.
 *
 *    Color: each stripe (one lit band, one full period of the wave) gets its
 *    own 2-color gradient -- one color at its leading edge, one at its
 *    trailing edge, fading smoothly between them across the stripe's own
 *    width -- rather than the whole show sharing one color. Which 2 colors a
 *    given stripe gets is deterministic from which period-cycle it is
 *    (hashPair(), a cheap integer hash of floor(pos/period)), so a specific
 *    physical stripe keeps its own stable color pair as it scrolls, without
 *    needing to track state per stripe instance. A slow desaturation cycle
 *    (100%->85%->100% over 30s, same technique as EqualizerShow) breathes on
 *    top of that.
 *
 *    Stopped mode: whenever speed is 0, or no speed packet has arrived in
 *    over 4 seconds (SpeedLink gone stale), scrolling simply freezes --
 *    scrollOffset_ stops advancing, so every stripe (rope/megabar/china
 *    alike, since they all derive from the same frozen scrollOffset_) just
 *    holds in place exactly as it was. After a further 10 continuous
 *    seconds stopped, each stripe's own hue pair starts slowly, randomly
 *    meandering (meanderHue_/meanderRateWalk_ -- same RandomWalk-based
 *    technique as LighthouseShow's beam hue and FlameShow's shifting hues,
 *    duplicated here per this codebase's per-show-helper convention) -- one
 *    shared drifting offset added to every stripe's own hash-derived base
 *    hue pair, so they all evolve together while still looking distinct
 *    (their bases differ). Resets to a fresh start every time a new stop
 *    begins.
 */

#ifndef __SPEED_STRIPES_SHOW_H
#define __SPEED_STRIPES_SHOW_H

#include "LightShow.h"
#include "SpeedLink.h"
#include <math.h>

class SpeedStripesShow : public LightShow {
 private:
   static const int stripeWidth_ = SIZEOF_LARGE_NEO / 4; // 88 LEDs = 4ft, a quarter of the car's 16ft length
   // sampleWave()'s period is always stripeWidth_*2 at every call site in
   // this file -- a true compile-time constant, not a runtime parameter --
   // so it's fixed-pointed here once (8 fractional bits = 1/256 LED
   // precision) rather than passed in and handled with float division.
   static const int FIXED_SHIFT_ = 8;
   static const int32_t PERIOD_FIXED_ = ( (int32_t)stripeWidth_ * 2 ) << FIXED_SHIFT_;
   float scrollOffset_ = 0.0f; // LEDs, grows over time while moving; frozen while stopped
   Timer frameTimer_;          // tracks dt between update() calls

   // random walk toward a freshly-randomized target every ~0.8-1.2s, rather
   // than jumping -- same technique as LighthouseShow's beam hue rate,
   // duplicated here rather than shared (this codebase's convention: small
   // per-show helpers, not a shared header).
   struct RandomWalk {
      float rampStart = 0.0f;
      float rampTarget = 0.0f;
      Timer tickTimer;
      bool initialized = false;

      float value( float minVal, float maxVal, float maxStep ) {
         if ( !initialized ) {
            initialized = true;
            tickTimer.set( 800 + random( 400 ) );
            rampStart = rampTarget = minVal + ( maxVal - minVal ) * 0.5f;
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

   bool wasStopped_ = false;
   Timer stoppedTimer_;          // how long we've been continuously stopped
   RandomWalk meanderRateWalk_;  // hue-units/sec, range 0..(255/20)
   float meanderHue_ = 0.0f;     // 0-255, shared drift added on top of every stripe's own hash-derived hues

   // slow rolling desaturation, same technique as EqualizerShow's
   // currentSatFraction()/desaturate(): saturation drifts from 100% down to
   // 85% and back over a smooth 30-second sine cycle.
   static float currentSatFraction( uint32_t time ) {
      static const uint32_t periodMs = 30000;
      float phase = (float)( time % periodMs ) / (float)periodMs;
      return 0.925f + 0.075f * cosf( 2.0f * PI * phase );
   }

   static CRGB desaturate( CRGB clr, float satFraction ) {
      uint8_t maxC = max( clr.r, max( clr.g, clr.b ) );
      if ( maxC == 0 ) return clr;
      uint8_t minFloor = (uint8_t)( maxC * ( 1.0f - satFraction ) + 0.5f );
      if ( clr.r < minFloor ) clr.r = minFloor;
      if ( clr.g < minFloor ) clr.g = minFloor;
      if ( clr.b < minFloor ) clr.b = minFloor;
      return clr;
   }

   // smoothly-faded brightness envelope (no hard edges) for the alternating
   // lit/dark pattern, combined with a per-stripe 2-color gradient -- see
   // the class comment for how the color pair is derived. satFraction and
   // meanderOffset are passed in precomputed once per frame by the caller
   // (not recomputed here), since this runs per-LED (over 1000x/frame) and
   // the Due's SAM3X8E (Cortex-M3) has no hardware FPU -- every avoidable
   // float transcendental call here is a real, measurable per-frame cost
   // (this was the actual cause of a previously-reported "everything feels
   // choppy" regression, since this show's update() keeps running in the
   // background even while a config screen is on-screen). sin8() (FastLED's
   // table-based fast sine) plus a cheap integer contrast boost stands in
   // for sinf()+tanhf() for the brightness envelope; the color hash below is
   // plain integer multiply/xor. Position wrapping/period-cycle math is
   // fixed-point integer (PERIOD_FIXED_ above) rather than fmodf()/floorf()
   // -- the SAM3X8E lacks a hardware FPU (so those are software-emulated,
   // division-based routines) but its Cortex-M3 core does have hardware
   // integer divide, so this is a genuine speedup, not a lateral move. The
   // one unavoidable float op is the single pos->fixed-point conversion at
   // the top, converting once rather than doing float division repeatedly.
   CRGB sampleWave( float pos, float satFraction, float meanderOffset ) {
      int32_t posFixed = (int32_t)( pos * (float)( 1 << FIXED_SHIFT_ ) + ( pos >= 0.0f ? 0.5f : -0.5f ) );
      int32_t wrappedFixed = posFixed % PERIOD_FIXED_;
      if ( wrappedFixed < 0 ) wrappedFixed += PERIOD_FIXED_;
      uint8_t phase8 = (uint8_t)( ( wrappedFixed * 255 ) / PERIOD_FIXED_ );

      static const int16_t FADE_CONTRAST = 2; // lower = softer/more analog, higher = closer to a hard edge
      int16_t centered = ( (int16_t)sin8( phase8 ) - 128 ) * FADE_CONTRAST;
      if ( centered > 127 ) centered = 127;
      if ( centered < -128 ) centered = -128;
      uint8_t brightness = (uint8_t)( centered + 128 );

      // which period-cycle this position falls in -> a deterministic
      // (hashed) pseudo-random pair of hues, so a given physical stripe
      // keeps a stable identity as it scrolls, without stored per-instance
      // state. Knuth multiplicative hash -- cheap, integer-only. Exact
      // (no rounding): posFixed - wrappedFixed is always a whole multiple
      // of PERIOD_FIXED_ by construction, same value floorf(pos/period)
      // would have given, just without ever calling it.
      int32_t stripeIdx = ( posFixed - wrappedFixed ) / PERIOD_FIXED_;
      uint32_t h = (uint32_t)stripeIdx * 2654435761u;
      h ^= h >> 15;
      uint8_t leadHue = (uint8_t)( ( h & 0xFF ) + (uint8_t)meanderOffset );
      uint8_t trailHue = (uint8_t)( ( ( h >> 8 ) & 0xFF ) + (uint8_t)meanderOffset );

      CRGB leadClr = desaturate( CHSV( leadHue, 255, 255 ), satFraction );
      CRGB trailClr = desaturate( CHSV( trailHue, 255, 255 ), satFraction );
      CRGB result = blend( trailClr, leadClr, phase8 );

      result.nscale8( brightness );
      return result;
   }

   // localOffset for a fixture's along-the-length position, given as a
   // fraction measured "back from the front" (matching the README's china
   // wording) -- 0 = at the front corner, 1 = at the back corner.
   float localOffsetForFracBackFromFront( float frac ) {
      return ( SIZEOF_LARGE_NEO - 1 ) * ( 1.0f - frac );
   }

   void renderSide( int backCornerIdx, int frontCornerIdx, float satFraction, float meanderOffset ) {
      int direction = ( frontCornerIdx > backCornerIdx ) ? 1 : -1;
      for ( int localOffset = 0; localOffset < SIZEOF_LARGE_NEO; ++localOffset ) {
         int idx = backCornerIdx + direction * localOffset;
         carpet->ropeLeds[ idx ] = sampleWave( (float)localOffset + scrollOffset_, satFraction, meanderOffset );
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
      frameTimer_.reset();
      wasStopped_ = false;
      meanderHue_ = 0.0f;
   }

   void update( uint32_t time ) {
      float dtSec = (float)frameTimer_.elapsed() / 1000.0f;
      frameTimer_.reset();

      // computed once per frame, not per-LED -- see sampleWave()'s comment
      float satFraction = currentSatFraction( time );

      // "stopped": speed is 0, or SpeedLink hasn't heard from the vehicle in
      // over 4 seconds (its own staleness threshold, not the 2s default)
      bool fresh = SpeedLink::isFresh( 4000 );
      float speedMph = fresh ? SpeedLink::getSpeedMph() : 0.0f;
      bool stopped = !fresh || ( speedMph == 0.0f );

      float meanderOffset = 0.0f;
      if ( stopped ) {
         if ( !wasStopped_ ) {
            wasStopped_ = true;
            stoppedTimer_.reset(); // starts the 10s "hold still" countdown fresh each time a stop begins
         }
         static const uint32_t MEANDER_DELAY_MS = 10000;
         if ( stoppedTimer_.elapsed() >= MEANDER_DELAY_MS ) {
            static const float MAX_MEANDER_RATE = 255.0f / 20.0f; // full spectrum in as little as 20s, same convention as Lighthouse/Flame
            float meanderRate = meanderRateWalk_.value( 0.0f, MAX_MEANDER_RATE, MAX_MEANDER_RATE * 0.1f ); // never negative -- always one direction
            meanderHue_ += meanderRate * dtSec;
            while ( meanderHue_ >= 256.0f ) meanderHue_ -= 256.0f;
         }
         meanderOffset = meanderHue_;
         // scrollOffset_ deliberately NOT advanced -- every stripe just holds
         // exactly where it was the moment we stopped
      } else {
         wasStopped_ = false;
         meanderHue_ = 0.0f; // next stop starts fresh, not mid-drift

         static const float LEDS_PER_MPH_PER_SEC = 4.0f;
         scrollOffset_ += speedMph * LEDS_PER_MPH_PER_SEC * dtSec;
         float period = stripeWidth_ * 2.0f;
         while ( scrollOffset_ >= period ) scrollOffset_ -= period; // keep it bounded; sampleWave() is periodic anyway
      }

      // front/back rope edges stay off -- they run side to side, not front to back
      for ( int i = 0; i < SIZEOF_SMALL_NEO; ++i ) carpet->ropeLeds[ i ] = CRGB::Black;
      for ( int i = SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; ++i ) {
         carpet->ropeLeds[ i ] = CRGB::Black;
      }

      // right side: back corner near SIZEOF_SMALL_NEO+SIZEOF_LARGE_NEO, front corner near SIZEOF_SMALL_NEO
      renderSide( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO - 1, SIZEOF_SMALL_NEO, satFraction, meanderOffset );
      // left side: back corner near SIZEOF_SMALL_NEO*2+SIZEOF_LARGE_NEO, front corner at the far end of the array
      renderSide( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO, NUM_NEO_LEDS_ACTUAL - 1, satFraction, meanderOffset );

      carpet->clearMegabars();
      carpet->clearChinas();

      static const float CAR_LENGTH_FT = 16.0f; // front-to-back, measured
      static const float LEDS_PER_FOOT = SIZEOF_LARGE_NEO / CAR_LENGTH_FT; // 22
      static const float EDGE_APPROACH_FT = 1.0f; // china "about 1ft from edge" pickup point

      // front: megabars preview a stripe-width (4ft) out; china pick it up right at the 1ft mark
      CRGB frontLeadClr = sampleWave( ( SIZEOF_LARGE_NEO - 1 ) + stripeWidth_ + scrollOffset_, satFraction, meanderOffset );
      carpet->megabarLeds[ 11 ] = frontLeadClr;
      carpet->megabarLeds[ 0 ]  = frontLeadClr; // headlight -- brightness still governed separately, see MagicCarpet
      carpet->megabarLeds[ 1 ]  = frontLeadClr;
      CRGB frontEdgeClr = sampleWave( ( SIZEOF_LARGE_NEO - 1 ) + LEDS_PER_FOOT * EDGE_APPROACH_FT + scrollOffset_, satFraction, meanderOffset );
      carpet->chinaLeds[ 1 ] = frontEdgeClr; carpet->chinaLeds[ 1 ].w = 0;
      carpet->chinaLeds[ 2 ] = frontEdgeClr; carpet->chinaLeds[ 2 ].w = 0;

      // rear: mirror of the above, behind the back corner
      CRGB rearLeadClr = sampleWave( -(float)stripeWidth_ + scrollOffset_, satFraction, meanderOffset );
      carpet->megabarLeds[ 5 ] = rearLeadClr;
      carpet->megabarLeds[ 6 ] = rearLeadClr;
      carpet->megabarLeds[ 7 ] = rearLeadClr;
      CRGB rearEdgeClr = sampleWave( -LEDS_PER_FOOT * EDGE_APPROACH_FT + scrollOffset_, satFraction, meanderOffset );
      carpet->chinaLeds[ 5 ] = rearEdgeClr; carpet->chinaLeds[ 5 ].w = 0;
      carpet->chinaLeds[ 6 ] = rearEdgeClr; carpet->chinaLeds[ 6 ].w = 0;

      // side positions, shared by the side china and the remaining side megabars
      float pos13 = localOffsetForFracBackFromFront( 1.0f / 3.0f ) + scrollOffset_;
      float pos12 = localOffsetForFracBackFromFront( 0.5f ) + scrollOffset_;
      float pos23 = localOffsetForFracBackFromFront( 2.0f / 3.0f ) + scrollOffset_;
      CRGB clr13 = sampleWave( pos13, satFraction, meanderOffset );
      CRGB clr12 = sampleWave( pos12, satFraction, meanderOffset );
      CRGB clr23 = sampleWave( pos23, satFraction, meanderOffset );

      // side china: front one on each side at 1/3 back from front, rear one at 2/3
      carpet->chinaLeds[ 0 ] = clr13; carpet->chinaLeds[ 0 ].w = 0; // front-right, right edge
      carpet->chinaLeds[ 7 ] = clr23; carpet->chinaLeds[ 7 ].w = 0; // back-right, right edge
      carpet->chinaLeds[ 3 ] = clr13; carpet->chinaLeds[ 3 ].w = 0; // front-left, left edge
      carpet->chinaLeds[ 4 ] = clr23; carpet->chinaLeds[ 4 ].w = 0; // back-left, left edge

      // remaining 3 megabars per side, same 1/3, 1/2, 2/3 partition
      carpet->megabarLeds[ 2 ]  = clr13; // 60 deg, left, nearer front
      carpet->megabarLeds[ 3 ]  = clr12; // 90 deg, dead left
      carpet->megabarLeds[ 4 ]  = clr23; // 120 deg, left, nearer back
      carpet->megabarLeds[ 10 ] = clr13; // 300 deg, right, nearer front
      carpet->megabarLeds[ 9 ]  = clr12; // 270 deg, dead right
      carpet->megabarLeds[ 8 ]  = clr23; // 240 deg, right, nearer back
   }
};

#endif
