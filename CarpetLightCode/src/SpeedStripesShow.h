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
#include "AudioBoard.h"
#include <math.h>

class SpeedStripesShow : public LightShow {
 private:
   enum Variation { VarDefault = 0, VarZebra = 1 };
   static const uint8_t numVariations_ = 2;
   uint8_t variation_;

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

   /* ---- Variation 1: zebra -- deterministic rainbow-hue stripes
      (consistently-increasing hue, not the default variation's random-hash
      stripes above), speed-desaturated, with an optional periodic
      black-stripe overlay and china-only sound reactivity. Own dedicated
      along-length position (zebraPosFt_, feet, grows as the car drives)
      rather than sharing scrollOffset_/sampleWave() -- a materially
      different pattern (hard hue steps + a scheduled black overlay vs. a
      hashed 2-color gradient wave), so it gets its own small state instead
      of contorting the default variation's machinery to fit both.

      Per-fixture along-length position uses this show's own existing
      feet/LED conventions (CAR_LENGTH_FT etc. below) -- not the true
      ground-spot optics the visualizer's zebra used (wash centerDist/aim
      angle projections, which don't exist anywhere in firmware). Same
      qualitative behavior (front fixtures preview a stripe out, side
      fixtures sample the 1/3-1/2-2/3 partition, per README's china
      layout), simplified to fit this file's existing geometry. */
   static const uint8_t ZEBRA_HUE_STEP = 11; // hue units (0-255) per stripe -- ~23 stripes (~207ft) per full rainbow cycle
   static constexpr float ZEBRA_STRIPE_WIDTH_FT = 6.0f;
   static constexpr float ZEBRA_FADE_FT = 3.0f;          // stripe-to-stripe and black-overlay border cross-fade width
   static constexpr float ZEBRA_MIN_BLACK_WIDTH_FT = 0.5f; // smallest black-stripe width the pot will ever produce, once producing any at all
   static constexpr float ZEBRA_MIN_GAP_FT = 5.0f;         // colored gap between black stripes never shrinks below this
   static constexpr float ZEBRA_SCHEDULE_LOOKAHEAD_FT = 30.0f; // comfortably past any fixture's own |pos| offset
   static constexpr float ZEBRA_CHINA_REST_FRACTION = 0.70f;   // china's resting brightness fraction of global max while there's been recent sound

   float zebraPosFt_ = 0.0f; // feet, monotonically grows while moving -- this variation's own scrolling position

   struct ZebraSeg { float startPos, endPos, width; bool isBlack; };
   static const int ZEBRA_SCHEDULE_MAX = 24; // generous fixed ring-buffer size -- worst case (min width+gap alternating) across 2x lookahead is well under this
   ZebraSeg zebraSchedule_[ ZEBRA_SCHEDULE_MAX ];
   int zebraScheduleHead_ = 0, zebraScheduleCount_ = 0;
   float zebraFrontierPos_ = 0.0f;
   bool zebraFrontierValid_ = false;

   Timer zebraSilenceTimer_; // time since AudioBoard was last NOT silent

   void resetZebraSchedule() {
      zebraScheduleHead_ = 0; zebraScheduleCount_ = 0; zebraFrontierValid_ = false;
   }
   ZebraSeg & zebraScheduleAt( int i ) { return zebraSchedule_[ ( zebraScheduleHead_ + i ) % ZEBRA_SCHEDULE_MAX ]; }
   void zebraSchedulePush( float startPos, float endPos, float width, bool isBlack ) {
      int idx = ( zebraScheduleHead_ + zebraScheduleCount_ ) % ZEBRA_SCHEDULE_MAX;
      zebraSchedule_[ idx ].startPos = startPos; zebraSchedule_[ idx ].endPos = endPos;
      zebraSchedule_[ idx ].width = width; zebraSchedule_[ idx ].isBlack = isBlack;
      if ( zebraScheduleCount_ < ZEBRA_SCHEDULE_MAX ) ++zebraScheduleCount_;
      else zebraScheduleHead_ = ( zebraScheduleHead_ + 1 ) % ZEBRA_SCHEDULE_MAX; // ring full -- oldest just got overwritten
   }
   void zebraSchedulePopFront() {
      if ( zebraScheduleCount_ > 0 ) { zebraScheduleHead_ = ( zebraScheduleHead_ + 1 ) % ZEBRA_SCHEDULE_MAX; --zebraScheduleCount_; }
   }

   // stopped = 100% saturated, 25mph = 60% saturated, linear between; holds
   // at 60% past 25mph.
   static float zebraSatFraction( float speedMph ) {
      float t = constrain( speedMph, 0.0f, 25.0f ) / 25.0f;
      return 1.0f - 0.4f * t;
   }
   static CRGB zebraStripeColor( int32_t k, float satFraction ) {
      int32_t hue = ( ( k * (int32_t)ZEBRA_HUE_STEP ) % 256 + 256 ) % 256;
      return CHSV( (uint8_t)hue, (uint8_t)( satFraction * 255.0f + 0.5f ), 255 );
   }
   // hard hue-stripe boundaries with a ZEBRA_FADE_FT-wide linear cross-fade
   // centered on each one -- same shape as the default variation's stripe
   // boundaries, just linear instead of a smoothed square wave (zebra's
   // hue step is a hard integer jump, not a 2-color gradient to ease
   // through).
   CRGB zebraHueColorAt( float pos, float satFraction ) {
      float contPos = pos / ZEBRA_STRIPE_WIDTH_FT;
      int32_t k = (int32_t)floorf( contPos );
      float frac = contPos - (float)k;
      float distToBoundaryFt = min( frac, 1.0f - frac ) * ZEBRA_STRIPE_WIDTH_FT;
      float halfFade = ZEBRA_FADE_FT / 2.0f;
      if ( distToBoundaryFt >= halfFade ) return zebraStripeColor( k, satFraction );
      if ( frac < 0.5f ) {
         float f = 0.5f + 0.5f * ( distToBoundaryFt / halfFade );
         return blend( zebraStripeColor( k - 1, satFraction ), zebraStripeColor( k, satFraction ), (uint8_t)( f * 255.0f + 0.5f ) );
      }
      float f = 0.5f * ( 1.0f - distToBoundaryFt / halfFade );
      return blend( zebraStripeColor( k, satFraction ), zebraStripeColor( k + 1, satFraction ), (uint8_t)( f * 255.0f + 0.5f ) );
   }
   // BUGFIX (ported from the visualizer prototype): this used to compute
   // the black/colored cell purely from the CURRENT pot value at whatever
   // position was being queried, so turning the pot retroactively rewrote
   // every stripe still in view. Replaced with an explicit,
   // incrementally-extended schedule -- each cell, once generated,
   // permanently keeps the width it was given at that moment; turning the
   // pot only changes what gets generated for territory not yet reached.
   // Pot mapping is dead-zone-free: pot=0% is the only fully-off point,
   // any turn above that immediately produces at least
   // ZEBRA_MIN_BLACK_WIDTH_FT and scales up to 40ft at 100%.
   void zebraEnsureScheduleTo( float neededPos ) {
      if ( !zebraFrontierValid_ ) { zebraFrontierPos_ = neededPos; zebraFrontierValid_ = true; }
      float potPercent = carpet->pot->readPercent();
      float potWidth = potPercent <= 0.0f ? 0.0f : ZEBRA_MIN_BLACK_WIDTH_FT + ( potPercent / 100.0f ) * ( 40.0f - ZEBRA_MIN_BLACK_WIDTH_FT );
      while ( zebraFrontierPos_ < neededPos ) {
         if ( potWidth < ZEBRA_MIN_BLACK_WIDTH_FT ) { zebraFrontierPos_ = neededPos; break; }
         bool lastWasBlack = zebraScheduleCount_ ? zebraScheduleAt( zebraScheduleCount_ - 1 ).isBlack : false;
         bool isBlack = !lastWasBlack;
         float width = isBlack ? potWidth : max( potWidth, ZEBRA_MIN_GAP_FT );
         float start = zebraFrontierPos_, end = start + width;
         zebraSchedulePush( start, end, width, isBlack );
         zebraFrontierPos_ = end;
      }
      float pruneBefore = neededPos - 2.0f * ZEBRA_SCHEDULE_LOOKAHEAD_FT;
      while ( zebraScheduleCount_ > 2 && zebraScheduleAt( 0 ).endPos < pruneBefore ) zebraSchedulePopFront();
   }
   float zebraBlackFractionAt( float pos ) {
      float halfFade = ZEBRA_FADE_FT / 2.0f;
      for ( int i = zebraScheduleCount_ - 1; i >= 0; --i ) {
         ZebraSeg & s = zebraScheduleAt( i );
         if ( pos < s.startPos || pos >= s.endPos ) continue;
         float distStart = pos - s.startPos, distEnd = s.endPos - pos;
         float dist = min( distStart, distEnd );
         float target = s.isBlack ? 1.0f : 0.0f;
         if ( dist >= halfFade ) return target;
         bool nearStart = distStart < distEnd;
         bool hasNeighbor = nearStart ? ( i - 1 >= 0 ) : ( i + 1 < zebraScheduleCount_ );
         float neighborTarget;
         if ( hasNeighbor ) {
            ZebraSeg & nb = nearStart ? zebraScheduleAt( i - 1 ) : zebraScheduleAt( i + 1 );
            neighborTarget = nb.isBlack ? 1.0f : 0.0f;
         } else {
            neighborTarget = s.isBlack ? 0.0f : 1.0f; // no neighbor generated yet -- assume the opposite (matches the alternating pattern)
         }
         float f = 0.5f - 0.5f * ( dist / halfFade ); // 0.5 exactly at the border, 0 at the fade's own edge
         return target + ( neighborTarget - target ) * f;
      }
      return 0.0f; // outside the generated range entirely (shouldn't normally happen) -- default colored
   }
   float zebraBlackFraction( float pos ) {
      zebraEnsureScheduleTo( pos + ZEBRA_SCHEDULE_LOOKAHEAD_FT );
      return zebraBlackFractionAt( pos );
   }
   CRGB zebraColorAt( float pos, float satFraction ) {
      CRGB hueColor = zebraHueColorAt( pos, satFraction );
      float blackFrac = zebraBlackFraction( pos );
      if ( blackFrac <= 0.0f ) return hueColor;
      return blend( hueColor, CRGB::Black, (uint8_t)( blackFrac * 255.0f + 0.5f ) );
   }
   // China-only (megabars are excluded per request -- always global max, see
   // updateZebra()) sound-reactive brightness: full global max after 10s+
   // of silence, otherwise ZEBRA_CHINA_REST_FRACTION of max, except bass
   // hits push it up to their own level (as a fraction of max), never below
   // that resting floor.
   float zebraChinaBrightnessFraction() {
      if ( !AudioBoard::silent_ ) zebraSilenceTimer_.reset();
      if ( zebraSilenceTimer_.elapsed() >= 10000 ) return 1.0f;
      float bassFrac = AudioBoard::getBandHitPercent( BandBass ) / 100.0f;
      return max( ZEBRA_CHINA_REST_FRACTION, bassFrac );
   }
   void renderZebraSide( int backCornerIdx, int frontCornerIdx, float satFraction ) {
      static const float halfLengthFt = 8.0f;
      static const float ledsPerFootLength = SIZEOF_LARGE_NEO / 16.0f;
      int direction = ( frontCornerIdx > backCornerIdx ) ? 1 : -1;
      for ( int localOffset = 0; localOffset < SIZEOF_LARGE_NEO; ++localOffset ) {
         int idx = backCornerIdx + direction * localOffset;
         float posFt = zebraPosFt_ + ( (float)localOffset / ledsPerFootLength - halfLengthFt );
         carpet->ropeLeds[ idx ] = zebraColorAt( posFt, satFraction );
         carpet->ropeLeds[ idx ].w = 0;
      }
   }
   void updateZebra( uint32_t time, float dtSec, bool fresh, float speedMph ) {
      zebraPosFt_ += SpeedLink::mphToFtPerSec( speedMph ) * dtSec;
      float satFraction = zebraSatFraction( fresh ? speedMph : 0.0f );

      carpet->clearRope(); carpet->clearMegabars(); carpet->clearChinas();

      // front/back small rope edges run side-to-side, not front-to-back --
      // no along-length position of their own, so (unlike the default
      // variation, which turns them off) they just take on the nearest
      // corner's own zebra color, matching whichever stripe is currently
      // at that end of the car.
      static const float halfLengthFt = 8.0f;
      CRGB frontEdgeClr = zebraColorAt( zebraPosFt_ + halfLengthFt, satFraction );
      CRGB backEdgeClr  = zebraColorAt( zebraPosFt_ - halfLengthFt, satFraction );
      for ( int i = 0; i < SIZEOF_SMALL_NEO; ++i ) { carpet->ropeLeds[ i ] = frontEdgeClr; carpet->ropeLeds[ i ].w = 0; }
      for ( int i = SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; ++i ) {
         carpet->ropeLeds[ i ] = backEdgeClr; carpet->ropeLeds[ i ].w = 0;
      }
      renderZebraSide( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO - 1, SIZEOF_SMALL_NEO, satFraction );
      renderZebraSide( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO, NUM_NEO_LEDS_ACTUAL - 1, satFraction );

      // Megabars: always global max, excluded from sound reactivity
      // entirely, per request -- china alone carries zebra's audio
      // response now.
      CRGB frontLeadClr = zebraColorAt( zebraPosFt_ + halfLengthFt + ZEBRA_STRIPE_WIDTH_FT, satFraction );
      carpet->megabarLeds[ 11 ] = frontLeadClr;
      carpet->megabarLeds[ 0 ]  = frontLeadClr; // headlight -- brightness still governed separately, see MagicCarpet
      carpet->megabarLeds[ 1 ]  = frontLeadClr;
      CRGB rearLeadClr = zebraColorAt( zebraPosFt_ - halfLengthFt - ZEBRA_STRIPE_WIDTH_FT, satFraction );
      carpet->megabarLeds[ 5 ] = rearLeadClr;
      carpet->megabarLeds[ 6 ] = rearLeadClr;
      carpet->megabarLeds[ 7 ] = rearLeadClr;
      CRGB clr13 = zebraColorAt( zebraPosFt_ + halfLengthFt * ( 1.0f / 3.0f ), satFraction );
      CRGB clr12 = zebraColorAt( zebraPosFt_, satFraction );
      CRGB clr23 = zebraColorAt( zebraPosFt_ - halfLengthFt * ( 1.0f / 3.0f ), satFraction );
      carpet->megabarLeds[ 2 ]  = clr13;
      carpet->megabarLeds[ 3 ]  = clr12;
      carpet->megabarLeds[ 4 ]  = clr23;
      carpet->megabarLeds[ 10 ] = clr13;
      carpet->megabarLeds[ 9 ]  = clr12;
      carpet->megabarLeds[ 8 ]  = clr23;

      // China: same positions as megabars' front/rear/side groups, per
      // README's china layout ([1,2] front edge, [5,6] back edge, [0]/[3]
      // 1/3 back, [7]/[4] 2/3 back) -- sound-reactive brightness on top.
      float chinaBrightness = zebraChinaBrightnessFraction();
      CRGB frontEdgeChina = zebraColorAt( zebraPosFt_ + halfLengthFt, satFraction );
      CRGB rearEdgeChina  = zebraColorAt( zebraPosFt_ - halfLengthFt, satFraction );
      CRGB china13 = clr13, china23 = clr23;
      CRGB * chinaSrc[ NUM_CHINA_LEDS ] = { &china13, &frontEdgeChina, &frontEdgeChina, &china13, &china23, &rearEdgeChina, &rearEdgeChina, &china23 };
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
         CRGB clr = *chinaSrc[ c ];
         clr.nscale8( (uint8_t)( chinaBrightness * 255.0f + 0.5f ) );
         carpet->chinaLeds[ c ] = clr;
         carpet->chinaLeds[ c ].w = 0;
      }
   }

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
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ) {}

   uint8_t variation() {
      return variation_;
   }
   const char * variationName() {
      return variation_ == VarZebra ? "zebra" : "default";
   }

   void start() {
      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();
      scrollOffset_ = 0.0f;
      frameTimer_.reset();
      wasStopped_ = false;
      meanderHue_ = 0.0f;
      zebraPosFt_ = 0.0f;
      resetZebraSchedule();
      zebraSilenceTimer_.reset();
   }

   void update( uint32_t time ) {
      // variation select: each encoder detent moves to the next variation,
      // same convention as EqualizerShow/FlameShow.
      int varDelta = carpet->encoder->readPositionDelta();
      carpet->encoder->resetPositionDelta();
      if ( varDelta != 0 ) {
         int newVariation = ( (int)variation_ + varDelta ) % (int)numVariations_;
         if ( newVariation < 0 ) newVariation += numVariations_;
         variation_ = (uint8_t)newVariation;
      }

      float dtSec = (float)frameTimer_.elapsed() / 1000.0f;
      frameTimer_.reset();

      // "stopped": speed is 0, or SpeedLink hasn't heard from the vehicle in
      // over 4 seconds (its own staleness threshold, not the 2s default) --
      // shared by both variations.
      bool fresh = SpeedLink::isFresh( 4000 );
      float speedMph = fresh ? SpeedLink::getSpeedMph() : 0.0f;

      if ( variation_ == VarZebra ) {
         updateZebra( time, dtSec, fresh, speedMph );
         return;
      }

      // computed once per frame, not per-LED -- see sampleWave()'s comment
      float satFraction = currentSatFraction( time );

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
