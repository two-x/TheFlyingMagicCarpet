/* SpeedStripesShow.h
 *
 *    Speed-reactive show driven by CANTroller2's vehicle speed telemetry
 *    (see SpeedLink.h). The two long side rope strips physically run
 *    front-to-rear along the car's 16ft length (see README.md, "Perimeter
 *    rope lights"); each is divided into 4 alternating lit/dark bands,
 *    each STRIPE_WIDTH_FT_ (4ft, a quarter of the car's length) wide, that
 *    scroll from front toward rear at real vehicle speed.
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
 *    extends conceptually beyond the car's own front/rear edges, so the
 *    front/rear megabars and china preview a stripe approaching before it
 *    reaches the carpet, and show it receding after it leaves:
 *
 *      - Front 3 megabars (the headlight and its neighbors) each preview
 *        the stripe one stripe-width (4ft) ahead of THEIR OWN real Y --
 *        every megabar in the trio samples its own individual real
 *        CarpetGeometry dimY plus that same 4ft lead, not a single
 *        shared reference point. BUGFIX: this used to compute one color
 *        from only the cardinal (headlight) megabar's own Y and copy it
 *        onto its 2 neighbors, which silently ignored that those
 *        neighbors sit at a materially different real Y than the
 *        headlight (they only match EACH OTHER's Y, by real mirror
 *        symmetry) -- visibly, all 3 changed color in lockstep regardless
 *        of the headlight's own Y actually leading them.
 *      - Front china aimed along the front edge each preview that same
 *        stripe, sampled 1ft ahead of THEIR OWN real Y (same per-fixture
 *        fix as above) -- picks it up just as it's about to cross onto
 *        the carpet.
 *      - Rear megabars/china mirror the above symmetrically for the
 *        stripe exiting the rear.
 *      - Every side china and side megabar samples its OWN independent
 *        real dimY directly, no lead offset -- no shared/approximated
 *        position anywhere in this show, front/rear preview groups
 *        included.
 *
 *    Position is real feet throughout (car-center-relative, Y+=front,
 *    same convention as CarpetGeometry) fed through one continuous,
 *    periodic sampling function -- valid at any real value, including the
 *    "ahead of the front edge" / "behind the rear edge" positions the
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
 *    technique as LighthouseShow's beam hue, duplicated here per this
 *    codebase's per-show-helper convention) -- one
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

// X-macro list: single source of truth for both the enum below and
// variationName() -- see LightShow.h's own comment on the pattern.
#define SPEEDSTRIPES_VARIATIONS(X) X(VarDefault) X(VarZebra)

class SpeedStripesShow : public LightShow {
 private:
   enum Variation { SPEEDSTRIPES_VARIATIONS(LIGHTSHOW_ENUM_ENTRY) };
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
   // Pot-percent dead zone at the bottom of the pot's travel, below which
   // black-stripe width is exactly 0 (no stripes at all) -- matches this
   // codebase's own established real-world pot-noise assumption
   // (LedController.h's Potentiometer::NOISE_FLOOR_RAW, ~2% of raw ADC
   // range, used there for the identical reason: ordinary pot jitter near
   // a rest position). BUGFIX ("flickering to black when pot is at
   // minimum"): width used to be computed as
   // `potPercent<=0 ? 0 : ZEBRA_MIN_BLACK_WIDTH_FT + ...` -- a HARD
   // discontinuity between 0 (at exactly 0%) and an instant jump to
   // ZEBRA_MIN_BLACK_WIDTH_FT (0.5ft) at any reading above 0%, however
   // tiny. Real pots don't hold exactly 0.00% at rest, and ordinary ADC
   // noise added more jitter on top -- so a pot sitting at its physical
   // minimum could read anywhere in a small band straddling that cliff,
   // popping stripes fully in and out every time it crossed. Width is now
   // a flat, provable 0 across this whole dead zone (immune to any noise
   // within it, not just usually-near-zero), then ramps CONTINUOUSLY
   // (zero jump at the boundary) from 0 up to 40ft at 100% -- see
   // zebraEnsureScheduleTo().
   static constexpr float ZEBRA_WIDTH_DEADZONE_PERCENT = 2.0f;
   // Separate from the dead zone above -- purely a practical floor so the
   // fixed-size schedule ring buffer below can't be asked to generate an
   // unbounded number of segments as width shrinks toward (but stays
   // above) the dead zone. Below this, a stretch is treated the same as
   // "no stripes" rather than generating vanishingly-thin ones. See
   // ZEBRA_SCHEDULE_MAX's own comment for the sizing math this drives.
   static constexpr float ZEBRA_MIN_GENERATE_WIDTH_FT = 0.25f;
   static constexpr float ZEBRA_SCHEDULE_LOOKAHEAD_FT = 30.0f; // comfortably past any fixture's own |pos| offset
   static constexpr float ZEBRA_CHINA_REST_FRACTION = 0.70f;   // china's resting brightness fraction of global max while there's been recent sound

   float zebraPosFt_ = 0.0f; // feet, monotonically grows while moving -- this variation's own scrolling position

   struct ZebraSeg { float startPos, endPos, width; bool isBlack; };
   // worst case is min-generatable-width black+gap alternating (period = 4 *
   // ZEBRA_MIN_GENERATE_WIDTH_FT = 1.0ft) across 2x lookahead (60ft) -- 60
   // segments -- sized with margin above that
   static const int ZEBRA_SCHEDULE_MAX = 80;
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
   // Pot mapping: flat 0 (guaranteed, see ZEBRA_WIDTH_DEADZONE_PERCENT's
   // own comment) through the bottom dead zone, then a continuous ramp
   // (no jump at the boundary) up to 40ft at 100%. Occurrence period (one
   // black stripe's start to the next) is always exactly 4x the current
   // black width -- i.e. the colored gap is always 3x the black width --
   // so wider stripes also come proportionally less often, rather than
   // width and spacing scaling independently.
   void zebraEnsureScheduleTo( float neededPos ) {
      if ( !zebraFrontierValid_ ) { zebraFrontierPos_ = neededPos; zebraFrontierValid_ = true; }
      float potPercent = carpet->pot->readPercent();
      float potWidth = potPercent <= ZEBRA_WIDTH_DEADZONE_PERCENT ? 0.0f
         : ( potPercent - ZEBRA_WIDTH_DEADZONE_PERCENT ) / ( 100.0f - ZEBRA_WIDTH_DEADZONE_PERCENT ) * 40.0f;
      while ( zebraFrontierPos_ < neededPos ) {
         if ( potWidth < ZEBRA_MIN_GENERATE_WIDTH_FT ) { zebraFrontierPos_ = neededPos; break; }
         bool lastWasBlack = zebraScheduleCount_ ? zebraScheduleAt( zebraScheduleCount_ - 1 ).isBlack : false;
         bool isBlack = !lastWasBlack;
         float width = isBlack ? potWidth : 3.0f * potWidth;
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
   // updateZebra()) sound-reactive brightness: full global max after
   // ZEBRA_CHINA_SILENCE_MS of silence, otherwise ZEBRA_CHINA_REST_FRACTION
   // of max, except bass hits push it up to their own level (as a fraction
   // of max), never below that resting floor.
   static const uint32_t ZEBRA_CHINA_SILENCE_MS = 750; // was 10000, per request ("shows that change light behavior after silence is detected" should do it sooner)
   float zebraChinaBrightnessFraction() {
      if ( !AudioBoard::isSilent() ) zebraSilenceTimer_.reset();
      if ( zebraSilenceTimer_.elapsed() >= ZEBRA_CHINA_SILENCE_MS ) return 1.0f;
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

      // front/rear small rope edges run side-to-side, not front-to-rear --
      // no along-length position of their own, so (unlike the default
      // variation, which turns them off) they just take on the nearest
      // corner's own zebra color, matching whichever stripe is currently
      // at that end of the car.
      static const float halfLengthFt = CarpetGeometry::CAR_LENGTH_FT / 2.0f; // true car edge -- the rope's own physical corner, not a flood fixture
      CRGB frontEdgeClr = zebraColorAt( zebraPosFt_ + halfLengthFt, satFraction );
      CRGB rearEdgeClr  = zebraColorAt( zebraPosFt_ - halfLengthFt, satFraction );
      LightSetters::setColor( carpet, LightSetters::TargetNeo, frontEdgeClr,
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideFront, 0 },
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideFront, (int32_t)CarpetGeometry::getSidePixelCount( CarpetGeometry::CarSideFront ) - 1 } );
      LightSetters::setColor( carpet, LightSetters::TargetNeo, rearEdgeClr,
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideRear, 0 },
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideRear, (int32_t)CarpetGeometry::getSidePixelCount( CarpetGeometry::CarSideRear ) - 1 } );
      renderZebraSide( CarpetGeometry::CarSideRight, satFraction );
      renderZebraSide( CarpetGeometry::CarSideLeft, satFraction );

      // Megabars: always global max, excluded from sound reactivity
      // entirely, per request -- china alone carries zebra's audio
      // response now. zebraPosFt_ is already a car-center-relative,
      // Y+=front feet coordinate -- each fixture's own real
      // CarpetGeometry dimY is directly addable to it.
      //
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
      //
      // BUGFIX ("megabar0/30/330 all change color at the exact same
      // instant, despite megabar0's real Y differing materially from its
      // two neighbors"): the front/rear "preview" trio used to compute
      // ONE shared color from only the cardinal fixture's own Y
      // (Megabar0deg/Megabar180deg) and copy it onto the other 2
      // fixtures in the group, silently ignoring their own different
      // real Y (Megabar30deg/Megabar330deg share a Y with each other by
      // real mirror symmetry, but NOT with Megabar0deg). leadFt
      // (optional, default 0) is the same "preview, sampled some
      // distance ahead of/behind this fixture's own real Y" concept as
      // before -- now applied per-fixture instead of computed once and
      // copy-pasted, so 2 fixtures only ever land on the same color when
      // their real geometry actually ties, never because the code reused
      // one shared value.
      auto zebraAt = [&]( float dimY, float leadFt = 0.0f ) { return zebraColorAt( zebraPosFt_ + dimY + leadFt, satFraction ); };
      auto setMegabarZebra = [&]( CarpetGeometry::MegabarName m, float leadFt = 0.0f ) {
         float y = CarpetGeometry::getMegabar( m ).dimY;
         LightSetters::setColor( carpet, LightSetters::TargetMegabar, zebraAt( y, leadFt ), LightSetters::ByID{ (uint8_t)m } );
      };
      setMegabarZebra( CarpetGeometry::Megabar330deg, ZEBRA_STRIPE_WIDTH_FT );
      setMegabarZebra( CarpetGeometry::Megabar0deg, ZEBRA_STRIPE_WIDTH_FT ); // headlight -- brightness still governed separately, see MagicCarpet
      setMegabarZebra( CarpetGeometry::Megabar30deg, ZEBRA_STRIPE_WIDTH_FT );
      setMegabarZebra( CarpetGeometry::Megabar150deg, -ZEBRA_STRIPE_WIDTH_FT );
      setMegabarZebra( CarpetGeometry::Megabar180deg, -ZEBRA_STRIPE_WIDTH_FT );
      setMegabarZebra( CarpetGeometry::Megabar210deg, -ZEBRA_STRIPE_WIDTH_FT );
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
   // BUGFIX history: an earlier version multiplied the hue-blended result
   // by a sine-derived "brightness" pulse amplified 2x then hard-clamped
   // to [0,255], meant as a brief stripe-boundary contrast dip -- the
   // clamping forced a sustained flat-black plateau far wider than a dip.
   // That was fixed by deleting the brightness step entirely, which
   // over-corrected: it silently made this variation incapable of ever
   // going dark at all, contradicting this file's own class comment
   // ("4 alternating lit/dark bands" via a "smoothed square wave" /
   // FADE_CONTRAST) and leaving it as a continuous hue wash with no actual
   // stripes -- confirmed by the user as "wrecked" against zebra (which
   // does alternate lit/black) as the reference for correct behavior.
   // Restored below with FastLED's safe nscale8() instead of a manual
   // amplify+clamp, so a real half-period black band is a deliberate,
   // bounded scale, not a saturating multiply that can overshoot.
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
      CRGB result = blend( trailClr, leadClr, phase8 );

      // smoothed-square-wave brightness envelope over the period: bright
      // through the first half (phase8 0-127, the lit band), black
      // through the second half (128-255, the dark band). sin8(phase8)
      // already peaks at phase8=64 (mid lit-band) and troughs at
      // phase8=192 (mid dark-band); FADE_CONTRAST steepens that smooth
      // sine into a flat-top/flat-bottom shape (higher = harder-edged
      // transition, lower = softer), bounded to int16_t range before
      // nscale8() applies it -- no unbounded amplify-then-clamp.
      static const int16_t FADE_CONTRAST = 6;
      int16_t centered = (int16_t)sin8( phase8 ) - 128;
      int16_t steepened = centered * FADE_CONTRAST;
      if ( steepened > 127 ) steepened = 127;
      if ( steepened < -128 ) steepened = -128;
      result.nscale8( (uint8_t)( steepened + 128 ) );
      return result;
   }

   // Sweeps every real neopixel on one side, sampling the wave at ITS OWN
   // real Y (CarpetGeometry, via LightSetters' side-local PixelId lookup
   // -- O(1)/pixel, same "already know the exact index, address it
   // directly" pattern as LighthouseShow's rope loop) -- no more show-
   // local "rearCorner/frontCorner/direction/localOffset" bookkeeping
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
      switch ( variation_ ) { SPEEDSTRIPES_VARIATIONS(LIGHTSHOW_VARIATION_NAME_CASE) }
      return "?";
   }
   #undef SPEEDSTRIPES_VARIATIONS // X-macro's job is done, keep the namespace clean

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

      // front/rear rope edges stay off -- they run side to side, not front
      // to rear. Range-set spans the whole side (0..count-1 in that
      // side's own local pixel numbering).
      LightSetters::setColor( carpet, LightSetters::TargetNeo, CHSV( 0, 0, 0 ),
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideFront, 0 },
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideFront, (int32_t)CarpetGeometry::getSidePixelCount( CarpetGeometry::CarSideFront ) - 1 } );
      LightSetters::setColor( carpet, LightSetters::TargetNeo, CHSV( 0, 0, 0 ),
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideRear, 0 },
         LightSetters::NeoByXPixelId{ CarpetGeometry::CarSideRear, (int32_t)CarpetGeometry::getSidePixelCount( CarpetGeometry::CarSideRear ) - 1 } );

      renderSide( CarpetGeometry::CarSideRight, satFraction, meanderOffset );
      renderSide( CarpetGeometry::CarSideLeft, satFraction, meanderOffset );

      carpet->clearMegabars();
      carpet->clearChinas();

      static const float EDGE_APPROACH_FT = 1.0f; // china "about 1ft from edge" pickup point

      // Every fixture below samples its OWN independent real dimY --
      // never shared between fixtures, china or megabar alike. Written
      // via LightSetters::ByID -- BUGFIX: this used to be ByYFt (nearest-
      // Y search) even though the fixture was already known; real
      // megabar/china Y values are NOT unique across left/right mirror
      // pairs (e.g. Megabar30deg/Megabar330deg share Y), so a self-
      // referential ByYFt search silently redirected the higher-id member
      // of every tied pair onto its lower-id twin, leaving it permanently
      // dark ("half the megabars/chinas never light up"). ByID is
      // unambiguous by construction -- reserve ByYFt/ByAngleDeg for
      // genuine "find whichever fixture is nearest X" searches, not for
      // addressing a fixture whose identity is already known.
      //
      // This is also the reason this entire show is unaffected by
      // CarpetGeometry::getMegabarByY/getChinaByY's own tie ambiguity
      // (see their comment there): this file NEVER calls those reverse
      // (value -> fixture) searches at all, for any of its Y-based
      // positioning -- every read below is the forward direction (known
      // fixture ID -> that fixture's own real dimY), and every write is
      // ByID. A getter that can return "ambiguous, no unique answer" is
      // only a problem for code that actually calls it.
      //
      // BUGFIX ("megabar0/30/330 all change color at the exact same
      // instant, despite megabar0's real Y differing materially from its
      // two neighbors"): the front/rear "preview" trio/pair below used to
      // compute ONE shared color from only the cardinal fixture's own Y
      // (Megabar0deg/Megabar180deg/ChinaFrontRight/ChinaRearLeft) and
      // copy it onto the other fixtures in the group, silently ignoring
      // their own different real Y (Megabar30deg/Megabar330deg share a Y
      // with each other by real mirror symmetry, but NOT with
      // Megabar0deg). leadFt (optional, default 0) is the same "preview,
      // sampled some distance ahead of/behind this fixture's own real Y"
      // concept as before -- now applied per-fixture instead of computed
      // once and copy-pasted, so 2 fixtures only ever land on the same
      // color when their real geometry actually ties, never because the
      // code reused one shared value. Also fixes the same latent issue
      // for china's front/rear pair, and matches the class comment's own
      // stated principle for the side fixtures below (previously true
      // only for those, not the front/rear groups).
      auto sideColorAt = [&]( float dimY, float leadFt = 0.0f ) { return sampleWave( dimY + leadFt + scrollOffsetFt_, satFraction, meanderOffset ); };
      auto setChinaBySide = [&]( CarpetGeometry::ChinaName c, float leadFt = 0.0f ) {
         float y = CarpetGeometry::getChina( c ).dimY;
         LightSetters::setColor( carpet, LightSetters::TargetChina, sideColorAt( y, leadFt ), LightSetters::ByID{ (uint8_t)c } );
      };
      auto setMegabarBySide = [&]( CarpetGeometry::MegabarName m, float leadFt = 0.0f ) {
         float y = CarpetGeometry::getMegabar( m ).dimY;
         LightSetters::setColor( carpet, LightSetters::TargetMegabar, sideColorAt( y, leadFt ), LightSetters::ByID{ (uint8_t)m } );
      };

      // front: megabars preview a stripe-width (4ft) out from their own
      // real Y; china pick it up right at the 1ft-from-edge mark from
      // theirs.
      setMegabarBySide( CarpetGeometry::Megabar330deg, STRIPE_WIDTH_FT_ );
      setMegabarBySide( CarpetGeometry::Megabar0deg, STRIPE_WIDTH_FT_ ); // headlight -- brightness still governed separately, see MagicCarpet
      setMegabarBySide( CarpetGeometry::Megabar30deg, STRIPE_WIDTH_FT_ );
      setChinaBySide( CarpetGeometry::ChinaFrontRight, EDGE_APPROACH_FT );
      setChinaBySide( CarpetGeometry::ChinaFrontLeft, EDGE_APPROACH_FT );

      // rear: mirror of the above, behind the rear corner
      setMegabarBySide( CarpetGeometry::Megabar150deg, -STRIPE_WIDTH_FT_ );
      setMegabarBySide( CarpetGeometry::Megabar180deg, -STRIPE_WIDTH_FT_ );
      setMegabarBySide( CarpetGeometry::Megabar210deg, -STRIPE_WIDTH_FT_ );
      setChinaBySide( CarpetGeometry::ChinaRearLeft, -EDGE_APPROACH_FT );
      setChinaBySide( CarpetGeometry::ChinaRearRight, -EDGE_APPROACH_FT );

      // side positions: no lead offset, straight at each fixture's own real Y.
      setChinaBySide( CarpetGeometry::ChinaRightFront );
      setChinaBySide( CarpetGeometry::ChinaRightRear );
      setChinaBySide( CarpetGeometry::ChinaLeftFront );
      setChinaBySide( CarpetGeometry::ChinaLeftRear );

      setMegabarBySide( CarpetGeometry::Megabar60deg );
      setMegabarBySide( CarpetGeometry::Megabar90deg );
      setMegabarBySide( CarpetGeometry::Megabar120deg );
      setMegabarBySide( CarpetGeometry::Megabar300deg );
      setMegabarBySide( CarpetGeometry::Megabar270deg );
      setMegabarBySide( CarpetGeometry::Megabar240deg );
   }
};

#endif
