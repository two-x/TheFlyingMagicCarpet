/* SpeedStripesShow.h
 *
 *    Speed-reactive show driven by CANTroller2's vehicle speed telemetry
 *    (see SpeedLink.h). The two long side rope strips physically run
 *    front-to-back along the car's 16ft length (see README.md, "Perimeter
 *    rope lights"); each is divided into 4 alternating lit/dark bands,
 *    each STRIPE_WIDTH_FT_ (4ft, a quarter of the car's length) wide, that
 *    scroll from front toward back at real vehicle speed.
 *
 *    BUGFIX (the actual root cause the user flagged as making the default
 *    variation "completely wrong," confirmed by comparison against the
 *    zebra variation, which was already correct): this used to keep its
 *    own separate internal bookkeeping in LED/pixel units, scrolling at a
 *    hand-invented `LEDS_PER_MPH_PER_SEC = 4.0f` rate that had nothing to
 *    do with real vehicle speed -- roughly 8x slower than real, since the
 *    real conversion (SpeedLink::mphToFtPerSec(), 1.46667 ft/s per mph) at
 *    this show's 22px/ft side-strand density works out to ~32 LEDs/mph/s,
 *    not 4. Fixed by unifying onto the SAME real feet-based scroll zebra
 *    already used (SpeedLink::mphToFtPerSec()) -- position/width/period
 *    are now all real feet throughout, matching every fixture's own real
 *    CarpetGeometry dimY, with LightSetters (see its own file header)
 *    doing every position->pixel/fixture resolution. No show-local pixel-
 *    unit bookkeeping remains anywhere in this file.
 *
 *    Every other fixture that has any meaningful along-the-length position
 *    joins in too, sampling the exact same scrolling pattern at its own
 *    real physical position (CarpetGeometry dimY), so a given stripe's
 *    color matches everywhere it's visible at once -- underneath (china),
 *    out to the sides (megabars), and on top (rope). The pattern also
 *    extends conceptually beyond the car's own front/back edges, so the
 *    front/rear megabars and china preview a stripe approaching before it
 *    reaches the carpet, and show it receding after it leaves:
 *
 *      - Front 3 megabars (the headlight and its neighbors) show the next
 *        stripe, sampled one stripe-width (4ft) ahead of the front
 *        megabar's own real Y -- a preview, well before it arrives.
 *      - Front china aimed along the front edge show that same stripe,
 *        sampled 1ft ahead of their own real Y -- picks it up just as
 *        it's about to cross onto the carpet.
 *      - Rear megabars/china mirror the above symmetrically for the
 *        stripe exiting the back.
 *      - Every side china and side megabar samples its OWN independent
 *        real dimY directly -- no shared/approximated position, no
 *        grouping beyond the explicit front/rear preview trios above.
 *
 *    Position is real feet throughout (car-center-relative, Y+=front,
 *    same convention as CarpetGeometry) fed through one continuous,
 *    periodic sampling function -- valid at any real value, including the
 *    "ahead of the front edge" / "behind the back edge" positions the
 *    front/rear fixtures use, no special-casing needed. Stripe edges are
 *    a smoothed square wave rather than a hard cut -- FADE_CONTRAST in
 *    sampleWave() controls how soft that transition looks; lower is
 *    softer/more analog, higher is closer to a hard edge.
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
 *    scrollOffsetFt_ stops advancing, so every stripe (rope/megabar/china
 *    alike, since they all derive from the same frozen scrollOffsetFt_)
 *    just holds in place exactly as it was. After a further 10 continuous
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
#include "CarpetGeometry.h"
#include "LightSetters.h"
#include <math.h>

class SpeedStripesShow : public LightShow {
 private:
   enum Variation { VarDefault = 0, VarZebra = 1 };
   static const uint8_t numVariations_ = 2;
   uint8_t variation_;

   static constexpr float STRIPE_WIDTH_FT_ = CarpetGeometry::CAR_LENGTH_FT / 4.0f; // 4.0ft, a quarter of the car's 16ft length -- the real physical stripe width
   // sampleWave()'s period is always STRIPE_WIDTH_FT_*2 at every call site
   // in this file -- a true compile-time constant, not a runtime
   // parameter -- so it's fixed-pointed here once (8 fractional bits =
   // 1/256 ft precision) rather than passed in and handled with float
   // division.
   static const int FIXED_SHIFT_ = 8;
   static const int32_t PERIOD_FIXED_ = (int32_t)( STRIPE_WIDTH_FT_ * 2.0f * (float)( 1 << FIXED_SHIFT_ ) + 0.5f );
   float scrollOffsetFt_ = 0.0f; // real feet, grows over time while moving; frozen while stopped
   Timer frameTimer_;            // tracks dt between update() calls

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

      Per-fixture along-length position is each fixture's own INDEPENDENT
      real ground-spot dimY, from CarpetGeometry (mandate: no more
      hand-tuned fraction-of-half-length approximation, and no sharing of
      one sampled value between 2 different real fixtures -- see
      updateZebra() and the default variation below, both converted the
      same way). */
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
   static CHSV zebraStripeColor( int32_t k, float satFraction ) {
      int32_t hue = ( ( k * (int32_t)ZEBRA_HUE_STEP ) % 256 + 256 ) % 256;
      return CHSV( (uint8_t)hue, (uint8_t)( satFraction * 255.0f + 0.5f ), 255 );
   }
   // hard hue-stripe boundaries with a ZEBRA_FADE_FT-wide linear cross-fade
   // centered on each one -- same shape as the default variation's stripe
   // boundaries, just linear instead of a smoothed square wave (zebra's
   // hue step is a hard integer jump, not a 2-color gradient to ease
   // through). Stays HSV except inside the fade zone, where 2 different
   // hues genuinely need to cross-fade -- same documented RGB-blend
   // exception as sampleWave() above (see its comment for why).
   CRGB zebraHueColorAt( float pos, float satFraction ) {
      float contPos = pos / ZEBRA_STRIPE_WIDTH_FT;
      int32_t k = (int32_t)floorf( contPos );
      float frac = contPos - (float)k;
      float distToBoundaryFt = min( frac, 1.0f - frac ) * ZEBRA_STRIPE_WIDTH_FT;
      float halfFade = ZEBRA_FADE_FT / 2.0f;
      if ( distToBoundaryFt >= halfFade ) return zebraStripeColor( k, satFraction );
      CRGB a, b; float f;
      if ( frac < 0.5f ) {
         f = 0.5f + 0.5f * ( distToBoundaryFt / halfFade );
         a = zebraStripeColor( k - 1, satFraction ); b = zebraStripeColor( k, satFraction );
      } else {
         f = 0.5f * ( 1.0f - distToBoundaryFt / halfFade );
         a = zebraStripeColor( k, satFraction ); b = zebraStripeColor( k + 1, satFraction );
      }
      return blend( a, b, (uint8_t)( f * 255.0f + 0.5f ) );
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
      // blending toward pure black is just a brightness scale -- no hue
      // ambiguity (black has no hue), so this is an exact equivalent of
      // the old blend(hueColor,Black,...) call, not an approximation.
      hueColor.nscale8( (uint8_t)( 255.0f * ( 1.0f - blackFrac ) + 0.5f ) );
      return hueColor;
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
   // Sweeps every real neopixel on one side by its own real Y (same
   // CarpetGeometry/LightSetters pattern as renderSide() above -- see its
   // comment) -- no more show-local corner/direction/localOffset math.
   void renderZebraSide( CarpetGeometry::CarSide side, float satFraction ) {
      uint16_t count = CarpetGeometry::getSidePixelCount( side );
      for ( uint16_t local = 0; local < count; ++local ) {
         int32_t neoId = CarpetGeometry::getNeoByYPixelId( side, local );
         uint16_t raw = CarpetGeometry::neoIdToRawIndex( neoId );
         float yFt = CarpetGeometry::getNeoGeom( raw ).yPercent / 100.0f * CarpetGeometry::CAR_LENGTH_FT;
         CRGB color = zebraColorAt( zebraPosFt_ + yFt, satFraction );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, color, LightSetters::NeoByCircumferenceID{ neoId } );
         LightSetters::setWhite( carpet, LightSetters::TargetNeo, 0, LightSetters::NeoByCircumferenceID{ neoId } );
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
      static const float halfLengthFt = CarpetGeometry::CAR_LENGTH_FT / 2.0f; // true car edge -- the rope's own physical corner, not a flood fixture
      CRGB frontEdgeClr = zebraColorAt( zebraPosFt_ + halfLengthFt, satFraction );
      CRGB backEdgeClr  = zebraColorAt( zebraPosFt_ - halfLengthFt, satFraction );
      LightSetters::setColor( carpet, LightSetters::TargetNeo, frontEdgeClr,
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideFront, 0 },
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideFront, (int32_t)CarpetGeometry::getSidePixelCount( CarpetGeometry::CarSideFront ) - 1 } );
      LightSetters::setColor( carpet, LightSetters::TargetNeo, backEdgeClr,
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideBack, 0 },
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideBack, (int32_t)CarpetGeometry::getSidePixelCount( CarpetGeometry::CarSideBack ) - 1 } );
      renderZebraSide( CarpetGeometry::CarSideRight, satFraction );
      renderZebraSide( CarpetGeometry::CarSideLeft, satFraction );

      // Megabars: always global max, excluded from sound reactivity
      // entirely, per request -- china alone carries zebra's audio
      // response now. zebraPosFt_ is already a car-center-relative,
      // Y+=front feet coordinate -- each fixture's own real
      // CarpetGeometry dimY is directly addable to it.
      float frontMegabarY = CarpetGeometry::getMegabar( CarpetGeometry::Megabar0deg ).dimY;
      CRGB frontLeadClr = zebraColorAt( zebraPosFt_ + frontMegabarY + ZEBRA_STRIPE_WIDTH_FT, satFraction );
      // BUGFIX: these were ByYFt (nearest-Y search) on a fixture whose ID
      // was already known -- real megabar Y values are NOT unique (every
      // left/right mirror pair shares Y, e.g. Megabar30deg/Megabar330deg
      // both sit at the same Y), so a self-referential ByYFt search
      // silently redirected the higher-id member of every tied pair onto
      // its lower-id twin, leaving it permanently dark ("half the
      // megabars/chinas never light up"). ByID is unambiguous by
      // construction -- use it whenever the fixture is already known,
      // reserve ByYFt/ByAngleDeg for genuine "find whichever fixture is
      // nearest X" searches. Same fix applied everywhere in this file.
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, frontLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar330deg } );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, frontLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar0deg } ); // headlight -- brightness still governed separately, see MagicCarpet
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, frontLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar30deg } );
      float rearMegabarY = CarpetGeometry::getMegabar( CarpetGeometry::Megabar180deg ).dimY;
      CRGB rearLeadClr = zebraColorAt( zebraPosFt_ + rearMegabarY - ZEBRA_STRIPE_WIDTH_FT, satFraction );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, rearLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar150deg } );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, rearLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar180deg } );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, rearLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar210deg } );
      // Every fixture below samples its OWN independent real dimY -- no
      // sharing between fixtures, china or megabar alike (2 fixtures at
      // the same real Y, e.g. the front-left/front-right pair, legitimately
      // get the same color because their real geometry actually is
      // symmetric, not because the code reuses one value for both).
      auto zebraAt = [&]( float dimY ) { return zebraColorAt( zebraPosFt_ + dimY, satFraction ); };
      auto setMegabarZebra = [&]( CarpetGeometry::MegabarName m ) {
         float y = CarpetGeometry::getMegabar( m ).dimY;
         LightSetters::setColor( carpet, LightSetters::TargetMegabar, zebraAt( y ), LightSetters::ByID{ (uint8_t)m } );
      };
      setMegabarZebra( CarpetGeometry::Megabar60deg );
      setMegabarZebra( CarpetGeometry::Megabar90deg );
      setMegabarZebra( CarpetGeometry::Megabar120deg );
      setMegabarZebra( CarpetGeometry::Megabar300deg );
      setMegabarZebra( CarpetGeometry::Megabar270deg );
      setMegabarZebra( CarpetGeometry::Megabar240deg );

      // China: sound-reactive brightness on top, same positions-from-real-
      // geometry approach, per README's china layout.
      float chinaBrightness = zebraChinaBrightnessFraction();
      uint8_t brightnessScale = (uint8_t)( chinaBrightness * 255.0f + 0.5f );
      for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) {
         float y = CarpetGeometry::getChina( c ).dimY;
         CRGB clr = zebraAt( y );
         clr.nscale8( brightnessScale );
         LightSetters::setColor( carpet, LightSetters::TargetChina, clr, LightSetters::ByID{ c } );
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

   // HSV-native (mandate: stay in HSV wherever a show doesn't have a real
   // reason not to) -- "desaturate toward white by satFraction" is just
   // scaling the S channel down, no RGB channel-floor math needed. Same
   // qualitative effect as the old RGB-floor version (both wash the color
   // toward white as satFraction drops toward 0); the exact curve isn't
   // bit-identical, but neither was tuned against a specific target curve.
   static CHSV desaturateHSV( CHSV clr, float satFraction ) {
      clr.sat = (uint8_t)( (float)clr.sat * satFraction + 0.5f );
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
   // BUGFIX ("megabars should always be on, never black" -- confirmed not
   // true at any point in this show's real history, going back to its very
   // first commit, so not something a revert could fix; this is the actual
   // mechanism responsible): this used to ALSO multiply the hue-blended
   // result by a separate sine-derived "brightness" pulse (amplified 2x
   // then hard-clamped to [0,255], intended as a stripe-boundary contrast
   // effect) -- the clamping forced a genuine, sustained flat-black
   // plateau at every stripe boundary, not just a dip. Removed entirely:
   // the hue blend below (leadClr->trailClr across each stripe's own
   // width) is already a smooth, continuous transition between two always-
   // fully-saturated CHSV(_,255,255) colors, which can never itself
   // produce black -- no separate brightness modulation is needed for
   // stripe boundaries to read as smoothed rather than hard-cut.
   // pos is real feet, car-center-relative, Y+=front (same convention as
   // CarpetGeometry) -- NOT LED/pixel units; the fixed-point math below is
   // purely an internal optimization (avoids float modulo), agnostic to
   // what unit pos is expressed in.
   CRGB sampleWave( float pos, float satFraction, float meanderOffset ) {
      int32_t posFixed = (int32_t)( pos * (float)( 1 << FIXED_SHIFT_ ) + ( pos >= 0.0f ? 0.5f : -0.5f ) );
      int32_t wrappedFixed = posFixed % PERIOD_FIXED_;
      if ( wrappedFixed < 0 ) wrappedFixed += PERIOD_FIXED_;
      uint8_t phase8 = (uint8_t)( ( wrappedFixed * 255 ) / PERIOD_FIXED_ );

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

      CHSV leadHsv = desaturateHSV( CHSV( leadHue, 255, 255 ), satFraction );
      CHSV trailHsv = desaturateHSV( CHSV( trailHue, 255, 255 ), satFraction );
      // Stays HSV up to here -- converts to CRGB only for this one blend,
      // which is a deliberate, documented exception (see class comment):
      // FastLED's blend() interpolates RGB channels, not hue; a true HSV
      // blend would rotate around the hue wheel instead (shorter-path
      // hue rotation, not a straight RGB crossfade), which is a real,
      // different visual result -- confirmed with the user to keep the
      // existing RGB-domain crossfade rather than change this show's
      // already-tuned stripe-boundary look.
      CRGB leadClr = leadHsv, trailClr = trailHsv;
      return blend( trailClr, leadClr, phase8 );
   }

   // Sweeps every real neopixel on one side, sampling the wave at ITS OWN
   // real Y (CarpetGeometry, via LightSetters' side-local PixelId lookup
   // -- O(1)/pixel, same "already know the exact index, address it
   // directly" pattern as LighthouseShow's rope loop) -- no more show-
   // local "backCorner/frontCorner/direction/localOffset" bookkeeping
   // duplicating what CarpetGeometry's NeoGeom already knows for real.
   void renderSide( CarpetGeometry::CarSide side, float satFraction, float meanderOffset ) {
      uint16_t count = CarpetGeometry::getSidePixelCount( side );
      for ( uint16_t local = 0; local < count; ++local ) {
         int32_t neoId = CarpetGeometry::getNeoByYPixelId( side, local );
         uint16_t raw = CarpetGeometry::neoIdToRawIndex( neoId );
         float yFt = CarpetGeometry::getNeoGeom( raw ).yPercent / 100.0f * CarpetGeometry::CAR_LENGTH_FT;
         CRGB color = sampleWave( yFt + scrollOffsetFt_, satFraction, meanderOffset );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, color, LightSetters::NeoByCircumferenceID{ neoId } );
         LightSetters::setWhite( carpet, LightSetters::TargetNeo, 0, LightSetters::NeoByCircumferenceID{ neoId } );
      }
   }

 public:
   SpeedStripesShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ) {}

   uint8_t variation() {
      return variation_;
   }
   uint8_t numVariations() { return numVariations_; }
   const char * variationName() {
      return variation_ == VarZebra ? "zebra" : "default";
   }

   void start() {
      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();
      scrollOffsetFt_ = 0.0f;
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
         // scrollOffsetFt_ deliberately NOT advanced -- every stripe just
         // holds exactly where it was the moment we stopped
      } else {
         wasStopped_ = false;
         meanderHue_ = 0.0f; // next stop starts fresh, not mid-drift

         // Real feet/sec, same conversion zebra already used -- see class
         // comment's BUGFIX note (this used to be a hand-invented,
         // ~8x-too-slow LED-based rate with no real relationship to speed).
         scrollOffsetFt_ += SpeedLink::mphToFtPerSec( speedMph ) * dtSec;
         float periodFt = STRIPE_WIDTH_FT_ * 2.0f;
         while ( scrollOffsetFt_ >= periodFt ) scrollOffsetFt_ -= periodFt; // keep it bounded; sampleWave() is periodic anyway
      }

      // front/back rope edges stay off -- they run side to side, not front
      // to back. Range-set spans the whole side (0..count-1 in that
      // side's own local pixel numbering).
      LightSetters::setColor( carpet, LightSetters::TargetNeo, CHSV( 0, 0, 0 ),
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideFront, 0 },
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideFront, (int32_t)CarpetGeometry::getSidePixelCount( CarpetGeometry::CarSideFront ) - 1 } );
      LightSetters::setColor( carpet, LightSetters::TargetNeo, CHSV( 0, 0, 0 ),
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideBack, 0 },
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideBack, (int32_t)CarpetGeometry::getSidePixelCount( CarpetGeometry::CarSideBack ) - 1 } );

      renderSide( CarpetGeometry::CarSideRight, satFraction, meanderOffset );
      renderSide( CarpetGeometry::CarSideLeft, satFraction, meanderOffset );

      carpet->clearMegabars();
      carpet->clearChinas();

      static const float EDGE_APPROACH_FT = 1.0f; // china "about 1ft from edge" pickup point

      // front: megabars preview a stripe-width (4ft) out from their own real
      // ground-spot Y; china pick it up right at the 1ft-from-edge mark from
      // theirs -- base position is each fixture's own real CarpetGeometry
      // dimY. Written via LightSetters::ByID -- BUGFIX: this used to be
      // ByYFt (nearest-Y search) even though the fixture was already known;
      // real megabar/china Y values are NOT unique across left/right mirror
      // pairs (e.g. Megabar30deg/Megabar330deg share Y), so a self-
      // referential ByYFt search silently redirected the higher-id member
      // of every tied pair onto its lower-id twin, leaving it permanently
      // dark ("half the megabars/chinas never light up"). ByID is
      // unambiguous by construction -- reserve ByYFt/ByAngleDeg for
      // genuine "find whichever fixture is nearest X" searches, not for
      // addressing a fixture whose identity is already known.
      float frontMegabarY = CarpetGeometry::getMegabar( CarpetGeometry::Megabar0deg ).dimY;
      CRGB frontLeadClr = sampleWave( frontMegabarY + STRIPE_WIDTH_FT_ + scrollOffsetFt_, satFraction, meanderOffset );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, frontLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar330deg } );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, frontLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar0deg } ); // headlight -- brightness still governed separately, see MagicCarpet
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, frontLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar30deg } );
      float frontChinaY = CarpetGeometry::getChina( CarpetGeometry::ChinaFrontRightFront ).dimY;
      CRGB frontEdgeClr = sampleWave( frontChinaY + EDGE_APPROACH_FT + scrollOffsetFt_, satFraction, meanderOffset );
      LightSetters::setColor( carpet, LightSetters::TargetChina, frontEdgeClr, LightSetters::ByID{ CarpetGeometry::ChinaFrontRightFront } );
      LightSetters::setColor( carpet, LightSetters::TargetChina, frontEdgeClr, LightSetters::ByID{ CarpetGeometry::ChinaFrontLeftFront } );

      // rear: mirror of the above, behind the back corner
      float rearMegabarY = CarpetGeometry::getMegabar( CarpetGeometry::Megabar180deg ).dimY;
      CRGB rearLeadClr = sampleWave( rearMegabarY - STRIPE_WIDTH_FT_ + scrollOffsetFt_, satFraction, meanderOffset );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, rearLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar150deg } );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, rearLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar180deg } );
      LightSetters::setColor( carpet, LightSetters::TargetMegabar, rearLeadClr, LightSetters::ByID{ CarpetGeometry::Megabar210deg } );
      float rearChinaY = CarpetGeometry::getChina( CarpetGeometry::ChinaBackLeftBack ).dimY;
      CRGB rearEdgeClr = sampleWave( rearChinaY - EDGE_APPROACH_FT + scrollOffsetFt_, satFraction, meanderOffset );
      LightSetters::setColor( carpet, LightSetters::TargetChina, rearEdgeClr, LightSetters::ByID{ CarpetGeometry::ChinaBackLeftBack } );
      LightSetters::setColor( carpet, LightSetters::TargetChina, rearEdgeClr, LightSetters::ByID{ CarpetGeometry::ChinaBackRightBack } );

      // side positions: every fixture samples its OWN independent real
      // dimY -- no sharing between china and megabar (an earlier version
      // of this code had china and its geometrically-nearest side megabar
      // share one sampled value; that was never something asked for --
      // it was a holdover from the old pre-geometry code, which happened
      // to reuse one hand-tuned fraction constant for both simply because
      // that's what the old approximation had on hand).
      auto sideColorAt = [&]( float dimY ) { return sampleWave( dimY + scrollOffsetFt_, satFraction, meanderOffset ); };
      auto setChinaBySide = [&]( CarpetGeometry::ChinaName c ) {
         float y = CarpetGeometry::getChina( c ).dimY;
         LightSetters::setColor( carpet, LightSetters::TargetChina, sideColorAt( y ), LightSetters::ByID{ (uint8_t)c } );
      };
      auto setMegabarBySide = [&]( CarpetGeometry::MegabarName m ) {
         float y = CarpetGeometry::getMegabar( m ).dimY;
         LightSetters::setColor( carpet, LightSetters::TargetMegabar, sideColorAt( y ), LightSetters::ByID{ (uint8_t)m } );
      };
      setChinaBySide( CarpetGeometry::ChinaFrontRightSide );
      setChinaBySide( CarpetGeometry::ChinaBackRightSide );
      setChinaBySide( CarpetGeometry::ChinaFrontLeftSide );
      setChinaBySide( CarpetGeometry::ChinaBackLeftSide );

      setMegabarBySide( CarpetGeometry::Megabar60deg );
      setMegabarBySide( CarpetGeometry::Megabar90deg );
      setMegabarBySide( CarpetGeometry::Megabar120deg );
      setMegabarBySide( CarpetGeometry::Megabar300deg );
      setMegabarBySide( CarpetGeometry::Megabar270deg );
      setMegabarBySide( CarpetGeometry::Megabar240deg );
   }
};

#endif
