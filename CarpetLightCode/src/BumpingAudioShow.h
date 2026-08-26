/* EqualizerShow.h
 *
 * Audio-reactive light show, 3 variations (VU meter / new_standard /
 * pixel_war) -- see variationName() and each variation's own update
 * logic below for what they do. Defaults to new_standard on boot (see
 * Nvm.h's resetShowVariationToDefaults()).
 *
 * Variation 0 (VU meter): a dual VU meter along the two side rope runs --
 * bass grows from the rear corner toward the front (red), treble grows
 * from the front corner toward the rear (blue). Each meter's 100%-level
 * reach is 15% of the true-corner-to-center half-length past center
 * (RIGHT/LEFT are already the exact side midpoints -- see MagicCarpet.h's
 * positional constants), so maxed-out bass and treble simultaneously
 * cross by about that much right around the middle of the car, per
 * request.
 *
 * Author: Anders Linn
 * Date: August 2017
 */

#include "LightShow.h"
#include "CarpetGeometry.h"
#include "LightSetters.h"
#include <math.h>

// BUGFIX (mandate): the front/rear-vs-left/right megabar grouping and
// front/rear-vs-side china grouping used to be hardcoded literal index
// arrays, hand-derived once from LighthouseShow.h's angle convention and
// never revisited -- exactly the "someone worked out the geometry by hand
// and hardcoded the result" pattern this session's mandate exists to
// catch (values confirmed correct against real CarpetGeometry angles
// before removal, so this wasn't an active bug, just a real duplication
// risk). Replaced with EqualizerShow::classifyNewStdFromGeometry() below,
// computed live from CarpetGeometry, same pattern as this file's own
// pre-existing classifyVuFloodsFromGeometry().

class EqualizerShow : public LightShow {
 private:
   // NOT using the LightShow.h X-macro/humanizeEnumName_() pattern the
   // other 4 shows use -- these display names genuinely don't fit that
   // mechanical "strip Var, lowercase, space before each cap" rule: "VU
   // meter" keeps VU as a real uppercase acronym (not "vu meter"), and
   // new_standard/pixel_war use an underscore convention that matches how
   // they're referred to throughout this file's own code/comments (not
   // "new standard" etc). Forcing the mechanical rule here would silently
   // change established display names that are also used as jargon
   // elsewhere in this file. variationName() below stays a plain
   // hand-written switch instead -- still real FW code the visualizer
   // reads via getCurrentVariationName(), just not mechanically
   // re-derivable from the enum text for this one show.
   enum Variation { VarVuMeter = 0, VarNewStandard = 1, VarPixelWar = 2 };
   static const uint8_t numVariations_ = 3;
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

   // Drives which option china/megabars are currently showing, for
   // new_standard. Replaces the old fixed-8000ms-timer advance: per
   // request, the option now changes on the START of the (X*4)th next
   // BASS hit since the last change (X an integer, re-rolled 2-4 each time
   // this fires), and the newly-chosen option is picked at RANDOM from the
   // available count rather than round-robin-incremented. China and
   // megabars each get their own independent instance (own hit count, own X,
   // own current option) -- see onBassEdge() below.
   struct OptionCycler {
      uint8_t option = 0;
      uint8_t numOptions = 3;
      int hitsSinceChange = 0;
      void reset( uint8_t numOpts ) {
         numOptions = numOpts;
         option = 0;
         hitsSinceChange = 0;
      }
      // call once per bass-hit edge, passing the CURRENT pot-controlled
      // cycle length in beats (see EqualizerShow::cycleBeats_/
      // potToCycleBeats() below) -- returns true if the option just
      // changed. Used to internally random-reroll its own 8/12/16
      // threshold; that's now a single shared, deterministic,
      // pot-adjustable value instead.
      bool onBassEdge( int cycleBeats ) {
         if ( ++hitsSinceChange < cycleBeats ) return false;
         hitsSinceChange = 0;
         option = ( numOptions > 1 ) ? (uint8_t)random( numOptions ) : 0;
         return true;
      }
   };
   // VU meter (variation 0) floods -- ported from the visualizer's own JS
   // mirror (showEqualizer()'s variation-1 branch in carpet-visualizer.html),
   // whose own comment on this exact spot says it plumbly: "not in the real
   // firmware yet ... added per request so the floods aren't dark during
   // this variation." Confirmed via direct testing: with the JS's fix never
   // ported back, updateVuMeter() clears megabar/china every call and never
   // refills them except during silence's swap-flash idle pattern -- floods
   // stay completely black through any amount of real, actual sound. Real
   // front/rear role from CarpetGeometry (dimY>0 = front), not the JS
   // mirror's visualizer-only canvas-pixel CAR_Y comparison.
   enum VuFbRole { VuRoleFront = 0, VuRoleRear = 1 };
   uint8_t vuMegabarRole_[ NUM_MEGABAR_LEDS ];
   uint8_t vuChinaRole_[ NUM_CHINA_LEDS ];
   void classifyVuFloodsFromGeometry() {
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         vuMegabarRole_[ m ] = ( CarpetGeometry::getMegabar( m ).dimY > 0.0f ) ? VuRoleFront : VuRoleRear;
      }
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
         vuChinaRole_[ c ] = ( CarpetGeometry::getChina( c ).dimY > 0.0f ) ? VuRoleFront : VuRoleRear;
      }
   }

   // new_standard's own front/rear-vs-side china grouping -- computed live
   // from real CarpetGeometry angles, same pattern as
   // classifyVuFloodsFromGeometry() above (see this file's top-of-file
   // BUGFIX comment for what this replaced). China (unlike megabars) isn't
   // evenly 30deg-spaced, so there's no clean "step by angle" loop for it
   // the way megabars get below -- classified once per fixture instead.
   // Megabar patterns (front/rear-vs-left/right, cardinal-vs-corner) are
   // computed directly inline, stepping by real 30deg multiples, at each
   // use site below -- no cached per-index classification needed there,
   // per explicit request ("handled within the lightshow").
   static float angDist_( float a, float b ) {
      float d = fabsf( a - b );
      return ( d > 180.0f ) ? 360.0f - d : d;
   }
   bool chinaIsFrontRear_[ NUM_CHINA_LEDS ];
   void classifyNewStdFromGeometry() {
      for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) {
         float a = CarpetGeometry::getChina( c ).dimAngleDeg;
         float dFrontRear = min( angDist_( a, 0.0f ), angDist_( a, 180.0f ) );
         float dSide = min( angDist_( a, 90.0f ), angDist_( a, 270.0f ) );
         chinaIsFrontRear_[ c ] = ( dFrontRear <= dSide );
      }
   }

   OptionCycler chinaCycler_, megabarCycler_;
   Timer megabarOptionTimer_; // time-since-entering-current-megabar-option -- only option 1 (front/rear vs L/R) still needs an interior timer, for its own 2s sub-swap
   bool newStdCycle1TrebleIsMod3_ = true;
   bool newStdCycle2FrontRearIsBass_ = true;
   // cycle 2 (few-random-floods) only: which color a megabar renders while
   // lit -- set at PICK time (see below), not pre-assigned, so a megabar
   // always shows the color of whichever band actually just picked it
   // (BUGFIX: "the random megabars only respond to bass, yet sometimes one
   // of them is in the treble color" -- the old newStdCycle3BassAssignment_
   // pre-randomized each megabar's color ONCE on entering this option,
   // independent of which band's edge actually picked it on any given hit,
   // so picks and colors could mismatch).
   bool newStdMegabarIsBass_[ NUM_MEGABAR_LEDS ] = { false };
   // bass's own pick count re-rolls (2-4) on every bass hit; treble's pick
   // count is always ceil(that/2) of whatever bass most recently rolled --
   // remembered here since bass/treble hits land on independent edges (see
   // the new_standard branch's two separate NewHit() checks below).
   uint8_t newStdLastBassPickCount_ = 2;
   float newStdMegabarHeat_[ NUM_MEGABAR_LEDS ] = { 0.0f }; // cycle 2 only

   // Shared "no sound" flood behavior (all 3 variations, per request):
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

   // pixel_war ("war"): restored per request to the original, simpler
   // concept -- BOTH colors respond to bass hits (no treble input at all
   // anymore), trading turns every cycleBeats_ (pot-controlled, shared
   // with new_standard -- see that member's own comment) bass hits.
   static constexpr float PW_MIN_FRAC = 0.125f, PW_MAX_FRAC = 0.875f; // 12.5%..87.5% of the loop
   static constexpr float PW_FADE_FRAC = 1.0f / 42.0f; // border cross-fade width -- 1/3 of the original 1/14, per request
   static constexpr float PW_HIT_IMPULSE = 0.24f;  // territory fraction jumped per qualifying hit edge (2x the original 0.12, per request)
   static constexpr float PW_RECOVER_FRAC = 0.5f;  // fraction of EACH hit's own impulse given back afterward, per request -- net territory gain per hit is PW_HIT_IMPULSE * (1 - PW_RECOVER_FRAC)
   // Impact and recovery are 2 distinct, time-separated phases (per
   // request), not one combined instantaneous step: the full impulse
   // lands immediately on the hit; PW_RECOVER_FRAC of it is given back
   // PW_RECOVER_DELAY_MS later (a baseline ~300ms "impact, then rebound"
   // feel, halved per the follow-up "reduce the time between impact and
   // recovery by 2x"). pwPendingRecoveryFrac_/pwRecoveryDelayRemainingMs_
   // track whichever recovery is currently counting down; a new hit before
   // the previous recovery lands simply replaces it (see updatePixelWar()).
   static constexpr float PW_RECOVER_DELAY_MS = 150.0f;
   static constexpr float PW_WALK_MAX = 37.5f;     // LED/s -- 25% of the original 150 (rate of travel around the rope's perimeter reduced 75% per request)
   float pwBassFrac_ = 0.5f;
   float pwPendingRecoveryFrac_ = 0.0f;      // 0 = no recovery currently pending
   float pwRecoveryDelayRemainingMs_ = 0.0f;
   float pwRotationOffsetLed_ = 0.0f, pwWalkspeed_ = 0.0f, pwWalkAccumMs_ = 0.0f;
   float pwMegabarHeat_[ NUM_MEGABAR_LEDS ] = { 0.0f };
   // which color source a megabar renders while lit, set at PICK time (per
   // request, "always use the right color for its freq band, like china") --
   // 0=bass (activeColor, respects the war's current turn), 1=treble
   // (Tcolor, fixed -- doesn't participate in turn-trading). See
   // PW_BASS_PICK/PW_TREBLE_PICK below.
   enum PwColorSrc { PwSrcBass = 0, PwSrcTreble = 1 };
   uint8_t pwMegabarColorSrc_[ NUM_MEGABAR_LEDS ] = { PwSrcBass };
   // per request: the war/territory mechanic itself stays bass-only, but the
   // megabar flash pool also includes an independently-triggered treble
   // picker -- treble count is always ceil(bass count/2). Each pool
   // re-picks on its OWN band's real hit edge (see AudioBoard::NewBandHit()
   // below), independent of the other. Midrange reactivity removed per
   // request -- no mid pool anywhere in this show anymore.
   static const int PW_BASS_PICK = 2;
   static const int PW_TREBLE_PICK = 1; // ceil(PW_BASS_PICK/2)
   static const int PW_MAX_PICK = PW_BASS_PICK + PW_TREBLE_PICK; // shared anti-repeat memory
   int pwLastPicked_[ PW_MAX_PICK ] = { -1, -1, -1 };
   bool pwTurnIsB_ = true;  // whose turn it currently is
   int pwTurnHitCount_ = 0; // bass hits so far this turn, vs cycleBeats_

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

   // pot adjusts AudioBoard's shared peak threshold while VU meter is
   // active -- soft takeover, same convention as config mode's
   // livePercentFor(): holds at the last-committed value until the pot
   // actually moves since this show became active, so it doesn't jump.
   // Reverted back to the committed value on leaving this show (see
   // CarpetLightLogic.cpp's show-change block), so casual live tweaking here
   // never overwrites what's saved to flash. new_standard/pixel_war no
   // longer use the pot for this -- see cycleBeats_ below, their own
   // separate pot role.
   static constexpr float POT_TAKEOVER_THRESHOLD_PERCENT = 2.0f;
   float potEntryPercent_ = 0.0f;
   bool potTakenOver_ = false;
   SettlePrinter thresholdPrinter_;

   // new_standard/pixel_war only: pot controls how many bass beats each
   // "effects cycle" lasts (china/megabar option-cycler advance threshold
   // for new_standard; the color-turn interval for pixel_war). 15 discrete
   // values, always even: fixed 2/4/6/8 at the low end, then log-ramped
   // (rounded to nearest 2) from 8 up to 128 across the rest of the pot's
   // travel -- worked out by hand per spec: ratio r = (128/8)^(1/11),
   // v_i = 8*r^i for i=1..11, each rounded to the nearest multiple of 2.
   static const uint8_t CYCLE_BEATS_TABLE[ 15 ];
   uint8_t cycleBeats_ = 8; // default -- see table index 3
   static uint8_t potToCycleBeats( float potPercent ) {
      int idx = (int)( potPercent / 100.0f * 14.0f + 0.5f );
      if ( idx < 0 ) idx = 0;
      if ( idx > 14 ) idx = 14;
      return CYCLE_BEATS_TABLE[ idx ];
   }

 public:
   EqualizerShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0, bool initialStrobeEnabled = false )
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ), strobeEnabled_( initialStrobeEnabled ) {}

   uint8_t variation() {
      return variation_;
   }
   uint8_t numVariations() { return numVariations_; }

   const char * variationName() {
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
         LightSetters::setColor( carpet, LightSetters::TargetNeo, CRGB::Black, LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      potEntryPercent_ = carpet->pot->readPercent();
      potTakenOver_ = false;
      eqFrameTimer_.reset();
      classifyVuFloodsFromGeometry();
      classifyNewStdFromGeometry();

      newStdHueTimeS_ = 0.0f; newStdSatTimeS_ = 0.0f;
      chinaCycler_.reset( 3 );
      megabarCycler_.reset( 3 );
      megabarOptionTimer_.reset();
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) { newStdMegabarHeat_[ m ] = 0.0f; newStdMegabarIsBass_[ m ] = false; }
      newStdLastBassPickCount_ = 2;
      for ( int i = 0; i < NEWSTD_NUM_FLOODS; ++i ) { newStdFloodIsB_[ i ] = ( i % 2 == 0 ); newStdFloodBrightness_[ i ] = 0.15f; }
      newStdSwapArmed_ = false;
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) newStdLastChinaColor_[ c ] = CRGB::Black;
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdLastMegabarColor_[ m ] = CRGB::Black;
      newStdChinaFadeStartMs_ = 0; newStdMegabarFadeStartMs_ = 0;

      pwBassFrac_ = 0.5f;
      pwPendingRecoveryFrac_ = 0.0f; pwRecoveryDelayRemainingMs_ = 0.0f;
      pwRotationOffsetLed_ = 0.0f; pwWalkspeed_ = 0.0f; pwWalkAccumMs_ = 0.0f;
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) { pwMegabarHeat_[ m ] = 0.0f; pwMegabarColorSrc_[ m ] = PwSrcBass; }
      for ( int n = 0; n < PW_MAX_PICK; ++n ) pwLastPicked_[ n ] = -1;
      pwTurnIsB_ = true;
      pwTurnHitCount_ = 0;
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

      float potPercent = carpet->pot->readPercent();
      if ( variation_ == VarNewStandard || variation_ == VarPixelWar ) {
         // pot -> effects-cycle length in beats, for these 2 variations only
         // -- see cycleBeats_'s own member comment.
         cycleBeats_ = potToCycleBeats( potPercent );
      } else {
         // pot -> live peak threshold, soft takeover (see member comment above)
         if ( !potTakenOver_ ) {
            if ( fabsf( potPercent - potEntryPercent_ ) >= POT_TAKEOVER_THRESHOLD_PERCENT ) potTakenOver_ = true;
         }
         if ( potTakenOver_ ) {
            AudioBoard::setPeakThresholdPercent( potPercent );
         }
         thresholdPrinter_.update( (int)( AudioBoard::getPeakThresholdPercent() + 0.5f ), "PkThresh:", "%" );
      }

      if ( variation_ == VarNewStandard ) {
         // Inlined directly here (per explicit request) rather than a
         // separately-dispatched private method -- new_standard is the
         // only variation that ever used this logic now that sub_standard
         // (its only other former caller) is gone, so the extra function
         // boundary no longer served any sharing purpose.
         bool silent = AudioBoard::isSilent();
         newStdSatTimeS_ += dtSec;
         if ( !silent ) newStdHueTimeS_ += dtSec;
         CRGB Bcolor, Tcolor;
         newStdColors( Bcolor, Tcolor );
         uint8_t bassHit = AudioBoard::getBandHitPercent( BandBass ), trebleHit = AudioBoard::getBandHitPercent( BandTreble );

         // ROPE: per explicit request, matches the floodlight color scheme
         // below (Bcolor/Tcolor) rather than a separate fixed palette --
         // still the bouncing 2-segment animation (a "big" segment per
         // long side, a "little" one per short side, each independently
         // reflecting off its own end), just recolored. Own local statics
         // (position/direction/timestamp).
         static uint32_t timestamp = time;
         static const uint32_t rate = 300;
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

         for ( int i = FRONT; i < FRONT_RIGHT; ++i ) {
            int i_adj = i - FRONT;
            int val = scaleTo255( abs( littlePos - i_adj ), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }
         for ( int i = FRONT_RIGHT; i < RIGHT; ++i ) {
            int i_adj = i - FRONT_RIGHT;
            int val = scaleTo255( abs( bigPos - i_adj ), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }
         for ( int i = RIGHT; i < REAR_RIGHT; ++i ) {
            int i_adj = i - RIGHT;
            int val = scaleTo255( abs( bigPos - i_adj ), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }
         for ( int i = REAR_RIGHT; i < REAR; ++i ) {
            int i_adj = i - REAR_RIGHT;
            int val = scaleTo255( abs( littlePos - i_adj ), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }
         for ( int i = REAR; i < REAR_LEFT; ++i ) {
            int i_adj = i - REAR;
            int val = scaleTo255( abs( littlePos - i_adj ), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }
         for ( int i = REAR_LEFT; i < LEFT; ++i ) {
            int i_adj = i - REAR_LEFT;
            int val = scaleTo255( abs( bigPos - i_adj ), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }
         for ( int i = LEFT; i < FRONT_LEFT; ++i ) {
            int i_adj = i - LEFT;
            int val = scaleTo255( abs( bigPos - i_adj ), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }
         for ( int i = FRONT_LEFT; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            int i_adj = i - FRONT_LEFT;
            int val = scaleTo255( abs( littlePos - i_adj ), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }
         for ( int i = 0; i < FRONT; ++i ) {
            int i_adj = i + SIZEOF_LARGE_NEO_CORNER;
            int val = scaleTo255( abs( littlePos - i_adj ), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
            LightSetters::setColor( carpet, LightSetters::TargetNeo, blend( Bcolor, Tcolor, val ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
         }

         LedUtil::reverse( carpet->ropeLeds + FRONT, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
         LedUtil::reverse( carpet->ropeLeds + REAR, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
         LedUtil::reverse( carpet->ropeLeds + RIGHT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );
         LedUtil::reverse( carpet->ropeLeds + LEFT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );

         if ( silent ) {
            updateSilenceFloods( time, dtSec, Bcolor, Tcolor );
            for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdLastMegabarColor_[ m ] = carpet->megabarLeds[ m ];
            for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) newStdLastChinaColor_[ c ] = carpet->chinaLeds[ c ];
         } else {

         // Shared bass/treble edge, used below both to drive china/megabar's
         // OptionCycler (bass edges only, per request) and cycle-2's own
         // per-hit heat mechanics. Uses the real FW edge flag
         // (AudioBoard::NewBandHit(), computed against the live peak
         // threshold at the true ~30ms audio poll rate) rather than a hand-
         // rolled ">20% and was <=20%" check sampled once per render frame --
         // BUGFIX ("megabars sometimes don't respond to a hit even though
         // chinas do"): the old check missed any hit whose predecessor hadn't
         // yet decayed back below the hardcoded 20% mark by the time the next
         // hit rose (common at faster tempos / slower hit-decay settings),
         // since china's own per-frame amplitude-based render still visibly
         // brightened even on a missed edge while megabar's edge-gated pick
         // logic silently did nothing. NewHit() can't miss a hit this way --
         // it's set the instant updateBandLevels() sees the real crossing,
         // independent of render-frame timing.
         bool bassEdge = AudioBoard::NewBandHit( BandBass );
         bool trebleEdge = AudioBoard::NewBandHit( BandTreble );

         bool chinaChanged = false, megabarChanged = false;
         if ( bassEdge ) {
            chinaChanged = chinaCycler_.onBassEdge( cycleBeats_ );
            megabarChanged = megabarCycler_.onBassEdge( cycleBeats_ );
         }

         // CHINA: 3 options (collapsed from the old 4-count cycle, whose
         // indices 2/3 rendered identically anyway), option chosen at random by
         // chinaCycler_ above rather than round-robin.
         if ( chinaChanged ) {
            for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) newStdChinaFadeFrom_[ c ] = newStdLastChinaColor_[ c ];
            newStdChinaFadeStartMs_ = time;
         }
         CRGB bassChina = Bcolor, trebleChina = Tcolor;
         bassChina.nscale8( (uint8_t)( (uint16_t)bassHit * 255 / 100 ) );
         trebleChina.nscale8( (uint8_t)( (uint16_t)trebleHit * 255 / 100 ) );
         if ( chinaCycler_.option == 0 ) {
            for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) LightSetters::setColor( carpet, LightSetters::TargetChina, chinaIsFrontRear_[ c ] ? bassChina : trebleChina, LightSetters::ByID{ c } );
         } else if ( chinaCycler_.option == 1 ) {
            for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) LightSetters::setColor( carpet, LightSetters::TargetChina, bassChina, LightSetters::ByID{ c } );
         } else {
            for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) LightSetters::setColor( carpet, LightSetters::TargetChina, trebleChina, LightSetters::ByID{ c } );
         }
         if ( time - newStdChinaFadeStartMs_ < NEWSTD_FADE_MS ) {
            uint8_t fadeF = (uint8_t)( 255.0f * (float)( time - newStdChinaFadeStartMs_ ) / (float)NEWSTD_FADE_MS );
            for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) LightSetters::setColor( carpet, LightSetters::TargetChina, blend( newStdChinaFadeFrom_[ c ], carpet->chinaLeds[ c ], fadeF ), LightSetters::ByID{ c } );
         }
         for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) {
            newStdLastChinaColor_[ c ] = carpet->chinaLeds[ c ];
            LightSetters::setWhite( carpet, LightSetters::TargetChina, 0, LightSetters::ByID{ c } );
         }

         // MEGABARS: 3 options, option chosen at random by megabarCycler_
         // above. Each option's own random sub-state is re-rolled fresh on
         // ENTRY to that option (i.e. the frame megabarChanged is true AND
         // that's the option just landed on) -- previously this reroll
         // happened inline with the round-robin advance itself; now it has
         // to be looked up by option number since the next option isn't a
         // fixed +1 anymore.
         if ( megabarChanged ) {
            for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdMegabarFadeFrom_[ m ] = newStdLastMegabarColor_[ m ];
            newStdMegabarFadeStartMs_ = time;
            megabarOptionTimer_.reset();
            if ( megabarCycler_.option == 0 ) newStdCycle1TrebleIsMod3_ = random( 2 ) == 0;
            if ( megabarCycler_.option == 1 ) newStdCycle2FrontRearIsBass_ = random( 2 ) == 0;
            if ( megabarCycler_.option == 2 ) {
               for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdMegabarHeat_[ m ] = 0.0f;
            }
         }
         CRGB bassMb = Bcolor, trebleMb = Tcolor;
         bassMb.nscale8( (uint8_t)( (uint16_t)bassHit * 255 / 100 ) );
         trebleMb.nscale8( (uint8_t)( (uint16_t)trebleHit * 255 / 100 ) );
         if ( megabarCycler_.option == 0 ) {
            // cardinal (0/90/180/270deg) vs corner megabars -- stepped by real
            // angle (every 30deg), not DMX-order index, per explicit request.
            for ( int step = 0; step < 12; ++step ) {
               float angle = (float)step * 30.0f;
               bool isMod3 = ( step % 3 == 0 );
               CRGB clr = ( isMod3 == newStdCycle1TrebleIsMod3_ ) ? trebleMb : bassMb;
               LightSetters::setColor( carpet, LightSetters::TargetMegabar, clr, LightSetters::ByAngleDeg{ angle } );
            }
         } else if ( megabarCycler_.option == 1 ) {
            // front/rear vs left/right -- stepped by real angle; classified
            // inline (nearest to the front-rear axis vs the left-right axis)
            // rather than a precomputed per-index table.
            int swapNum = (int)( megabarOptionTimer_.elapsed() / 2000 ); // every 2s
            bool frontRearIsBass = ( swapNum % 2 == 0 ) ? newStdCycle2FrontRearIsBass_ : !newStdCycle2FrontRearIsBass_;
            for ( int step = 0; step < 12; ++step ) {
               float angle = (float)step * 30.0f;
               float dFrontRear = min( angDist_( angle, 0.0f ), angDist_( angle, 180.0f ) );
               float dLeftRight = min( angDist_( angle, 90.0f ), angDist_( angle, 270.0f ) );
               bool isFrontRear = ( dFrontRear <= dLeftRight );
               bool wantsBass = isFrontRear ? frontRearIsBass : !frontRearIsBass;
               LightSetters::setColor( carpet, LightSetters::TargetMegabar, wantsBass ? bassMb : trebleMb, LightSetters::ByAngleDeg{ angle } );
            }
         } else {
            // Few-random-floods phase. On each qualifying BASS edge, a fresh
            // 2-4 (re-rolled every hit, per request) currently-black megabars
            // light up in Bcolor; on each qualifying TREBLE edge, ceil(that
            // bass count/2) currently-black megabars light up in Tcolor --
            // each band's own pool is picked independently, at THAT band's own
            // hit, and the megabar's color is set right then (fixing the old
            // "picked megabar shows the wrong band's color" mismatch -- see
            // newStdMegabarIsBass_'s own comment above). Falls back to the N
            // dimmest if fewer than N are fully black.
            auto pickAndLight = [&]( int numToPick, bool isBass ) {
               int black[ NUM_MEGABAR_LEDS ], blackCount = 0;
               for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) if ( newStdMegabarHeat_[ m ] <= 0.02f ) black[ blackCount++ ] = m;
               int pick[ 4 ] = { -1, -1, -1, -1 }; // 4 = max possible numToPick (bass's own 2-4 range)
               if ( blackCount >= numToPick ) {
                  for ( int n = 0; n < numToPick; ++n ) {
                     int j = random( blackCount );
                     pick[ n ] = black[ j ];
                     black[ j ] = black[ --blackCount ];
                  }
               } else {
                  // N dimmest -- simple selection over 12 entries
                  int order[ NUM_MEGABAR_LEDS ]; for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) order[ m ] = m;
                  for ( int a = 0; a < NUM_MEGABAR_LEDS - 1; ++a )
                     for ( int b = a + 1; b < NUM_MEGABAR_LEDS; ++b )
                        if ( newStdMegabarHeat_[ order[ b ] ] < newStdMegabarHeat_[ order[ a ] ] ) { int t = order[ a ]; order[ a ] = order[ b ]; order[ b ] = t; }
                  for ( int n = 0; n < numToPick; ++n ) pick[ n ] = order[ n ];
               }
               for ( int n = 0; n < numToPick; ++n ) if ( pick[ n ] >= 0 ) {
                  newStdMegabarHeat_[ pick[ n ] ] = 1.0f;
                  newStdMegabarIsBass_[ pick[ n ] ] = isBass;
               }
            };
            if ( bassEdge ) {
               newStdLastBassPickCount_ = 2 + random( 3 ); // 2-4
               pickAndLight( newStdLastBassPickCount_, true );
            }
            if ( trebleEdge ) {
               int numTreble = ( newStdLastBassPickCount_ + 1 ) / 2; // ceil(bass/2)
               pickAndLight( numTreble, false );
            }
            for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
               newStdMegabarHeat_[ m ] = max( 0.0f, newStdMegabarHeat_[ m ] - dtSec / 0.4f );
               CRGB base = newStdMegabarIsBass_[ m ] ? Bcolor : Tcolor;
               base.nscale8( (uint8_t)( newStdMegabarHeat_[ m ] * 255.0f + 0.5f ) );
               LightSetters::setColor( carpet, LightSetters::TargetMegabar, base, LightSetters::ByID{ (uint8_t)m } );
            }
         }
         if ( time - newStdMegabarFadeStartMs_ < NEWSTD_FADE_MS ) {
            uint8_t fadeF = (uint8_t)( 255.0f * (float)( time - newStdMegabarFadeStartMs_ ) / (float)NEWSTD_FADE_MS );
            for ( uint8_t m = 0; m < NUM_MEGABAR_LEDS; ++m ) LightSetters::setColor( carpet, LightSetters::TargetMegabar, blend( newStdMegabarFadeFrom_[ m ], carpet->megabarLeds[ m ], fadeF ), LightSetters::ByID{ m } );
         }
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) newStdLastMegabarColor_[ m ] = carpet->megabarLeds[ m ];
         } // end !silent
      } else if ( variation_ == VarPixelWar ) {
         updatePixelWar( time, dtSec );
      } else { // VarVuMeter
         updateVuMeter( time );
         // Per request, every variation goes dark-with-nothing-to-react-to
         // the same way during silence -- new_standard/pixel_war have
         // their own internal silence handling above; VU meter gets it as
         // a post-step override on top of its own (otherwise near-dark)
         // output, same as the visualizer prototype.
         if ( AudioBoard::isSilent() ) updateSilenceFloods( time, dtSec, CRGB( 255, 0, 0 ), CRGB( 0, 0, 255 ) ); // red=bass, blue=treble, this show's own convention
      }

      updateStrobe( time );
   }

 private:
   // variation 0: dual VU meter. Bass (AudioBoard::getLow()) is based across
   // the WHOLE rear of the car -- the rear edge rope is always part of it,
   // not just the two rear corners -- and grows forward from there along
   // both sides, red. Treble (getHigh()) mirrors this from the whole front
   // edge, growing rearward along both sides, blue. Both meters
   // use the same attack/decay peak-hold ballistics as hit's own
   // (jump up fast past a threshold, fall back 15/frame otherwise),
   // just kept as separate state. Brightness of each meter (base edge and
   // side fill alike) scales directly with its current level -- a quiet
   // signal shows a dim meter, not just a short one.
   //
   // RIGHT/LEFT are already the exact midpoints of each side run (see
   // MagicCarpet.h's positional constants), and FRONT_RIGHT/REAR_RIGHT (resp.
   // FRONT_LEFT/REAR_LEFT) the true corners -- so halfLen_ below is exactly
   // the true-corner-to-center distance. Each meter's 100%-level reach is
   // halfLen_ plus 15% of halfLen_, so maxed bass and treble together cross
   // by about 15% of half the car's length, right around the middle --
   // per request.
   void updateVuMeter( uint32_t time ) {
      // hit already implements this meter's attack/decay ballistics (see
      // AudioBoard::updateBandLevels(), tunable via the Audio config
      // screen's decay-rate subsetting), so no more local peak-hold state.
      int vuLastBass = (int)( (uint16_t)AudioBoard::getBandHitPercent( BandBass ) * 255 / 100 );
      int vuLastTreble = (int)( (uint16_t)AudioBoard::getBandHitPercent( BandTreble ) * 255 / 100 );

      static const float halfLen_ = SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER; // true corner to center, 143
      static const float maxReach_ = halfLen_ * 1.15f; // 100%-level reach: 15% of halfLen_ past center
      float bassLitCount = ( vuLastBass / 255.0f ) * maxReach_;
      float trebleLitCount = ( vuLastTreble / 255.0f ) * maxReach_;

      // brightness follows level directly -- a quiet meter is a dim meter
      float satFraction = currentSatFraction( time );
      CRGB bassClr = desaturate( CRGB( 255, 0, 0 ), satFraction );   // red, per request
      CRGB trebleClr = desaturate( CRGB( 0, 0, 255 ), satFraction ); // blue, matching this show's high=blue convention
      bassClr.nscale8( (uint8_t)vuLastBass );
      trebleClr.nscale8( (uint8_t)vuLastTreble );

      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();

      // rear edge: always part of the bass meter's base, full width
      for ( int i = SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; ++i ) {
         LightSetters::setColor( carpet, LightSetters::TargetNeo, bassClr, LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      // front edge: always part of the treble meter's base, full width
      for ( int i = 0; i < SIZEOF_SMALL_NEO; ++i ) {
         LightSetters::setColor( carpet, LightSetters::TargetNeo, trebleClr, LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }

      // right side: FRONT_RIGHT/REAR_RIGHT are the reference corner-to-
      // center points bassLitCount/trebleLitCount are calibrated against
      // (see halfLen_/maxReach_ above) -- kept fixed. The loop itself,
      // however, covers the side channel's ENTIRE physical strand (true
      // corner to true corner: SIZEOF_SMALL_NEO to SIZEOF_SMALL_NEO+
      // SIZEOF_LARGE_NEO-1), not just the span between the reference
      // points.
      //
      // BUGFIX: this loop used to run FRONT_RIGHT..REAR_RIGHT only (the
      // reference points themselves), leaving the ~33-LED zone on EACH side
      // of that span (between the true physical corner and its inset
      // reference point) completely dark regardless of level -- neither
      // this loop nor the front/rear base-edge fill above ever touched it.
      // Widening the loop to the true strand bounds fixes this using this
      // side's OWN distance-from-reference math (which, past a reference
      // point, is always "closer than any possible LitCount," i.e. reads as
      // an unconditional, level-proportional continuation of that meter --
      // physically correct, and entirely local to this side's own strand,
      // never reaching into the front/rear channels' own logic the way an
      // earlier version of this fix mistakenly did).
      for ( int i = SIZEOF_SMALL_NEO; i < SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; ++i ) {
         float distFromFront = i - FRONT_RIGHT;
         float distFromRear = REAR_RIGHT - i;
         LightSetters::setColor( carpet, LightSetters::TargetNeo, vuMeterColor( distFromRear < bassLitCount, distFromFront < trebleLitCount, bassClr, trebleClr ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }
      // left side: same widening, REAR_LEFT/FRONT_LEFT stay the fixed
      // reference points, loop covers the full physical strand.
      for ( int i = SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         float distFromRear = i - REAR_LEFT;
         float distFromFront = FRONT_LEFT - i;
         LightSetters::setColor( carpet, LightSetters::TargetNeo, vuMeterColor( distFromRear < bassLitCount, distFromFront < trebleLitCount, bassClr, trebleClr ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
      }

      // Floods (megabar/china) -- see the class member comment above
      // (vuMegabarRole_/vuChinaRole_/classifyVuFloodsFromGeometry()) for
      // why this exists: without it these sit fully cleared/black for as
      // long as any real sound is present, only ever lighting during
      // silence's swap-flash idle pattern.
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         CRGB clr = ( vuMegabarRole_[ m ] == VuRoleFront ) ? trebleClr : bassClr;
         LightSetters::setColor( carpet, LightSetters::TargetMegabar, clr, LightSetters::ByID{ (uint8_t)m } );
      }
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
         CRGB clr = ( vuChinaRole_[ c ] == VuRoleFront ) ? trebleClr : bassClr;
         LightSetters::setColor( carpet, LightSetters::TargetChina, clr, LightSetters::ByID{ (uint8_t)c } );
      }
   }

   // slow rolling desaturation applied to every base color used across
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
   // BUGFIX HISTORY ("floods hella dim vs SpeedStripes"): a first attempt
   // at this fix applied a flat 65% brightness FLOOR to every hit-driven
   // fill below (hitBrightnessByte(), since removed) -- fixed the average
   // brightness, but compressed away the dark/light contrast a hit needs
   // to read as reactive at all, causing a real regression (megabars/
   // china stopped visibly responding to individual hits, since even "no
   // hit" now looked 65% as bright as "hit"). Fixed for real in
   // AudioBoard::getBandHitPercent() itself instead -- during an actual hit it
   // now reports flat max (100%) rather than the literal analog level, so
   // quiet moments stay genuinely dark (full contrast/reactivity) while
   // hits themselves always read as a full, punchy flash. This file just
   // uses AudioBoard::getBandHitPercent()'s return value directly again, same
   // as any other show.
   void newStdColors( CRGB & Bcolor, CRGB & Tcolor ) {
      float hueFrac = ( 1.0f - cosf( 2.0f * PI * newStdHueTimeS_ / NEWSTD_HUE_PERIOD_S ) ) / 2.0f;
      float satFrac = ( 1.0f - cosf( 2.0f * PI * newStdSatTimeS_ / NEWSTD_SAT_PERIOD_S ) ) / 2.0f;
      uint8_t satByte = (uint8_t)( ( 75.0f + 25.0f * satFrac ) / 100.0f * 255.0f + 0.5f );
      uint8_t bHue = (uint8_t)( (float)NEWSTD_HUE_RED + ( (float)NEWSTD_HUE_YELLOW - (float)NEWSTD_HUE_RED ) * hueFrac + 0.5f );
      Bcolor = CHSV( bHue, satByte, 255 );
      Tcolor = CHSV( NEWSTD_HUE_TREBLE, satByte, 255 );
   }

   // Shared "no sound" flood behavior, used by all 3 variations -- see the
   // class member comment above. colorB/colorT are whichever two base
   // colors the calling variation uses (Bcolor/Tcolor for new_standard/
   // pixel_war, bass-red/treble-blue for VU meter).
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
         if ( i < NUM_MEGABAR_LEDS ) {
            LightSetters::setColor( carpet, LightSetters::TargetMegabar, color, LightSetters::ByID{ (uint8_t)i } );
         } else {
            uint8_t chinaId = (uint8_t)( i - NUM_MEGABAR_LEDS );
            LightSetters::setColor( carpet, LightSetters::TargetChina, color, LightSetters::ByID{ chinaId } );
            LightSetters::setWhite( carpet, LightSetters::TargetChina, 0, LightSetters::ByID{ chinaId } );
         }
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

   // Restored per request to the original, simpler "war" concept: BOTH
   // colors reference bass hits (no treble input anywhere in here anymore)
   // -- they trade turns, one qualifying bass hit at a time, every
   // cycleBeats_ hits (the pot-controlled, shared-with-new_standard cycle
   // length -- see that member's own comment). Only the currently-"turned"
   // color visibly reacts to each hit; territory/megabar-pick/china-flash
   // all key off whichever color currently has the turn.
   void updatePixelWar( uint32_t time, float dtSec ) {
      bool silent = AudioBoard::isSilent();
      newStdSatTimeS_ += dtSec;
      if ( !silent ) newStdHueTimeS_ += dtSec;
      CRGB Bcolor, Tcolor;
      newStdColors( Bcolor, Tcolor );
      uint8_t bassHit = AudioBoard::getBandHitPercent( BandBass );

      if ( silent ) {
         updateSilenceFloods( time, dtSec, Bcolor, Tcolor );
         return;
      }

      // Same NewHit()-based edge detection as new_standard above (see its
      // own comment) -- fixes the same "megabar misses a hit chinas still
      // show" class of bug here too. Per request, this cycle's treble
      // trigger is sourced from specific bins rather than the curated
      // AudioBand grouping: treble is just the top 2 bins (FreqBin5/6 --
      // 6250/16000Hz), NOT AudioBand's own BandTreble (which also includes
      // FreqBin4/2500Hz). Midrange reactivity removed per request.
      bool bassEdge = AudioBoard::NewBandHit( BandBass );
      bool trebleEdge = AudioBoard::NewBinHit( FreqBin5 ) || AudioBoard::NewBinHit( FreqBin6 );

      if ( bassEdge ) {
         if ( ++pwTurnHitCount_ >= cycleBeats_ ) {
            pwTurnHitCount_ = 0;
            pwTurnIsB_ = !pwTurnIsB_;
         }
      }
      CRGB activeColor = pwTurnIsB_ ? Bcolor : Tcolor;

      // ROPE: territory war -- every qualifying bass hit pushes territory
      // toward whichever color currently has the turn immediately (the
      // impact); PW_RECOVER_FRAC of that same push is given back
      // PW_RECOVER_DELAY_MS later (the recovery) -- 2 distinct,
      // time-separated phases, not one combined instantaneous step (see
      // PW_RECOVER_DELAY_MS's own comment). A hit that lands while a
      // previous recovery is still pending replaces it outright (that
      // earlier impulse's own give-back is superseded by this newer hit's
      // own, rather than stacking).
      pwStepRandomWalk( dtSec * 1000.0f );
      if ( bassEdge ) {
         float impulse = pwTurnIsB_ ? PW_HIT_IMPULSE : -PW_HIT_IMPULSE;
         pwBassFrac_ += impulse;
         pwPendingRecoveryFrac_ = impulse * PW_RECOVER_FRAC;
         pwRecoveryDelayRemainingMs_ = PW_RECOVER_DELAY_MS;
      } else if ( pwPendingRecoveryFrac_ != 0.0f ) {
         pwRecoveryDelayRemainingMs_ -= dtSec * 1000.0f;
         if ( pwRecoveryDelayRemainingMs_ <= 0.0f ) {
            pwBassFrac_ -= pwPendingRecoveryFrac_;
            pwPendingRecoveryFrac_ = 0.0f;
         }
      }
      pwBassFrac_ = constrain( pwBassFrac_, PW_MIN_FRAC, PW_MAX_FRAC );
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         int32_t neoId = CarpetGeometry::rawIndexToNeoId( i );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, pwColorAt( i, Bcolor, Tcolor ), LightSetters::NeoByCircumferenceID{ neoId } );
         LightSetters::setWhite( carpet, LightSetters::TargetNeo, 0, LightSetters::NeoByCircumferenceID{ neoId } );
      }

      // MEGABARS: 2 independent pools, each picking its own currently-dark
      // megabars on its OWN band's real hit edge, then fading -- bass
      // (PW_BASS_PICK, always Bcolor) + treble (PW_TREBLE_PICK =
      // ceil(bass/2), always Tcolor), per request (midrange pool removed).
      // BUGFIX ("bass hits respond using treble color"): the bass pool used
      // to render with activeColor (whichever color currently has the
      // WAR's turn -- Bcolor or Tcolor, alternating) instead of a fixed
      // Bcolor, so during T's turn a genuine bass hit would render
      // identically to a treble hit, indistinguishable from each other.
      // activeColor stays turn-dependent for the ROPE/CHINA (that's the
      // actual "war" visual, unchanged) -- only the megabar flash pool's
      // bass color is now fixed, same as treble always was, so each pool's
      // color always identifies which band triggered it, like china's own
      // bass/treble split does.
      // wasLastPicked avoidance is shared across both pools (one combined
      // memory, PW_MAX_PICK slots) so back-to-back picks from DIFFERENT
      // bands still avoid immediately repeating a spot.
      auto pwPickAndLight = [&]( int numToPick, uint8_t colorSrc ) {
         int eligible[ NUM_MEGABAR_LEDS ], eligibleCount = 0;
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
            if ( pwMegabarHeat_[ m ] > 0.02f ) continue;
            bool wasLastPicked = false;
            for ( int n = 0; n < PW_MAX_PICK; ++n ) if ( m == pwLastPicked_[ n ] ) { wasLastPicked = true; break; }
            if ( wasLastPicked ) continue;
            eligible[ eligibleCount++ ] = m;
         }
         if ( eligibleCount < numToPick ) {
            eligibleCount = 0;
            for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) if ( pwMegabarHeat_[ m ] <= 0.02f ) eligible[ eligibleCount++ ] = m;
         }
         for ( int n = 0; n < numToPick && eligibleCount > 0; ++n ) {
            int j = random( eligibleCount );
            int m = eligible[ j ];
            eligible[ j ] = eligible[ --eligibleCount ];
            pwMegabarHeat_[ m ] = 1.0f;
            pwMegabarColorSrc_[ m ] = colorSrc;
            for ( int s = PW_MAX_PICK - 1; s > 0; --s ) pwLastPicked_[ s ] = pwLastPicked_[ s - 1 ];
            pwLastPicked_[ 0 ] = m;
         }
      };
      if ( bassEdge ) pwPickAndLight( PW_BASS_PICK, PwSrcBass );
      if ( trebleEdge ) pwPickAndLight( PW_TREBLE_PICK, PwSrcTreble );
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         pwMegabarHeat_[ m ] = max( 0.0f, pwMegabarHeat_[ m ] - dtSec / 0.4f );
         CRGB base = ( pwMegabarColorSrc_[ m ] == PwSrcBass ) ? Bcolor : Tcolor;
         base.nscale8( (uint8_t)( pwMegabarHeat_[ m ] * 255.0f + 0.5f ) );
         LightSetters::setColor( carpet, LightSetters::TargetMegabar, base, LightSetters::ByID{ (uint8_t)m } );
      }

      // CHINA: all 8 flash together in the ACTIVE color, brightness
      // tracking the live bass hit level -- simpler than the old 2-band
      // pair-split, since there's no treble input left to split against.
      CRGB chinaLit = activeColor;
      chinaLit.nscale8( (uint8_t)( (uint16_t)bassHit * 255 / 100 ) );
      for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) {
         LightSetters::setColor( carpet, LightSetters::TargetChina, chinaLit, LightSetters::ByID{ c } );
         LightSetters::setWhite( carpet, LightSetters::TargetChina, 0, LightSetters::ByID{ c } );
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
   // the headlight[0]/left[3]/rear[6]/right[9]) full white for 3 pulses,
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

      bool isHit = AudioBoard::getBandHitNonzero( BandBass );
      int bassHit = AudioBoard::getBandHitPercent( BandBass );

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
      CRGB strobeClr = lit ? CRGB::White : CRGB::Black;
      for ( int i = 0; i < 8; ++i ) LightSetters::setColor( carpet, LightSetters::TargetMegabar, strobeClr, LightSetters::ByID{ (uint8_t)cornerMegabars[ i ] } );
      for ( uint8_t i = 0; i < NUM_CHINA_LEDS; ++i ) {
         LightSetters::setColor( carpet, LightSetters::TargetChina, strobeClr, LightSetters::ByID{ i } );
         LightSetters::setWhite( carpet, LightSetters::TargetChina, lit ? (uint8_t)255 : (uint8_t)0, LightSetters::ByID{ i } );
      }
   }
};
const uint8_t EqualizerShow::CYCLE_BEATS_TABLE[ 15 ] = { 2, 4, 6, 8, 10, 14, 18, 22, 28, 36, 46, 60, 78, 100, 128 };
