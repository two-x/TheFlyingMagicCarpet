/* LighthouseShow.h
 *
 *    Two independent rotating "lighthouse" beams, each a pair of opposite-
 *    facing (180deg apart) cones, point at the car's own center, expanding
 *    outward. Real angle math throughout: rope/megabar/china all evaluate
 *    the SAME falloff function (coneBrightnessAt()) at each fixture's own
 *    real physical angle (from CarpetGeometry) -- no per-fixture-type
 *    bespoke shape.
 *
 *    Where the 2 beams' cones overlap: NO additive color mixing (reads as
 *    a muddy wash) -- only the brighter of the 2 beams shows at all, using
 *    that beam's own exact color, at that beam's own computed brightness
 *    (winnerColorAt()). Megabars specifically break an exact brightness
 *    tie by alternating winner by megabar index (see winnerColorAt()'s
 *    preferBeam2OnTie param) rather than always favoring beam 1 -- per
 *    explicit request, so a tie can never systematically starve one
 *    color's megabar coverage.
 *
 *    Rotation/color: each beam's angular velocity (deg/s, range +/-360 =
 *    +/-1Hz) is its own independent random walk -- see RandomWalk below --
 *    that can drift through zero and reverse direction over time; angles
 *    themselves stay full float precision throughout, never quantized.
 *    Beam 1's hue always increases (never reverses -- "always clockwise"),
 *    at a rate that itself random-walks between 0 (frozen) and a max of 1
 *    full spectrum cycle per 20 seconds. Saturation random-walks between
 *    70% and 100% (starts at 80%), same mechanism -- shared exactly by
 *    both beams (not independent per-beam saturation). Beam 2 shares
 *    beam 1's saturation and uses the complementary hue (+128), and has
 *    its own independent rotation random walk, but no independent hue/sat
 *    walks of its own.
 *
 *    Beam cone width (CONE_HALF_WIDTH_DEG/CONE_FULL_WIDTH_DEG below) is
 *    sized so that AT ANY beam angle, at least 2 real megabars always fall
 *    within its guaranteed-coverage core -- derived from the real,
 *    measured megabar ground-spot angles (CarpetGeometry's mbSpotXY),
 *    not the nominal 30deg-apart naming. See CONE_HALF_WIDTH_DEG's own
 *    comment for the exact derivation and worst-case proof. Independently,
 *    MIN_BEAM_SEPARATION_DEG keeps the 2 beams' cores from ever touching,
 *    so one beam's guaranteed megabar coverage can never be contested/
 *    stolen by the other beam's winner-take-all comparison, even though
 *    both beams' rotation velocities are free to be the same sign (i.e.
 *    both CW or both CCW) and could otherwise drift arbitrarily close
 *    together. See the MIN_BEAM_SEPARATION_DEG enforcement block in
 *    update() for the mechanism (a soft, symmetric angular "push-apart",
 *    not a velocity restriction).
 *
 *    Within a beam's own guaranteed-coverage core, brightness is NOT flat
 *    -- it eases smoothly from full (255, dead center) down to a floor
 *    (CORE_FLOOR_BRIGHTNESS) at the core's own outer edge, then continues
 *    easing from that floor down to black over the further CONE_FADE_DEG
 *    zone beyond the guarantee. Per explicit request: this gives the 2
 *    always-guaranteed megabars on each side of a beam a smooth, "analog"
 *    brightness change as the beam sweeps past (nearer to dead-center =
 *    brighter), rather than snapping between a flat plateau and off. The
 *    floor keeps the worst-case-farthest guaranteed megabar always
 *    clearly, visibly lit -- never fading toward black within the
 *    guarantee itself.
 *
 *    Variations:
 *      0 Default   -- rope+megabar+china all join both beams (the same
 *                      beam-crossing color effect everywhere); on top of
 *                      that, china ALSO gets a bass-hit white strobe --
 *                      applied to the White channel only (LightSetters::
 *                      setWhite()), never touching the RGB channel a
 *                      china's own beam-crossing color already occupies,
 *                      so the two effects genuinely superimpose (the
 *                      white element can strobe without interrupting the
 *                      RGB beam color underneath it) instead of one
 *                      overwriting the other.
 *      1 No Strobe -- identical rope/megabar/china beam-crossing effect as
 *                      Default, just without the white strobe layered on
 *                      top of china.
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
#include "LightSetters.h"
#include "AudioBoard.h"
#include <math.h>

// X-macro list: single source of truth for both the enum below and
// variationName() -- see LightShow.h's own comment on the pattern.
#define LIGHTHOUSE_VARIATIONS(X) X(VarDefault) X(VarNoStrobe)

class LighthouseShow : public LightShow {
 private:
   enum Variation { LIGHTHOUSE_VARIATIONS(LIGHTSHOW_ENUM_ENTRY) };
   static const uint8_t numVariations_ = 2;
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
   RandomWalk satWalk_;             // fraction, range 0.70..1.0, starts at 0.80

   float angle1_ = 0.0f, angle2_ = 180.0f; // deg, 0-360, full float precision throughout
   float vel1_ = 0.0f, vel2_ = 0.0f;       // deg/s, current (post-random-walk) value
   float hue1_ = 0.0f;                     // 0-255 float for precision; only ever increases

   // BUGFIX ("beams end up going the same speed in the same direction"):
   // the MIN_BEAM_SEPARATION_DEG enforcement below corrects the FULL
   // shortfall every single frame it triggers, snapping effSep to exactly
   // the minimum -- not merely "soft", as its own comment claims. Given
   // the guaranteed-free corridor is only 20deg wide (90 max - 70 min) and
   // each beam's raw velocity can reach 360deg/s, two independently
   // walking beams typically close that 20deg in well under a second, then
   // stay pinned: from that point on, ANY further net closing tendency in
   // vel1_/vel2_ gets fully cancelled every frame, which forces the two
   // beams' EFFECTIVE angular rates to become equal (moving together at
   // their shared average) for as long as that tendency persists -- which,
   // since vel1_/vel2_ themselves only drift slowly (+/-10deg/s per ~1s
   // random-walk tick), can be many seconds. The position clamp itself is
   // correct and stays untouched (it's what keeps the hard "2 megabars
   // always covered" guarantee -- see class comment -- genuinely
   // unconditional). What's missing is a real bounce: a decaying velocity
   // bias injected on every collision, so the pair visibly separates again
   // afterward instead of staying rigidly pinned, while the underlying
   // random walks keep evolving independently underneath it.
   float bounceVel1_ = 0.0f, bounceVel2_ = 0.0f; // deg/s, decays toward 0
   static constexpr float BOUNCE_DECAY_TAU_SEC = 1.5f;
   static constexpr float BOUNCE_KICK_DEG_PER_SEC = 40.0f;
   // BUGFIX ("both beams always end up going counterclockwise, not one
   // CW/one CCW"): the kick above was being re-applied every single frame
   // the collision condition stayed true, not once per collision. A
   // single 40deg/s kick is small next to the beams' combined closing
   // rate (up to 720deg/s), so it typically took many consecutive frames
   // to actually separate -- and since kickSign's sign stays the same for
   // as long as the same 2 beams keep approaching from the same side,
   // those repeated kicks all landed in the same direction on both
   // bounceVel1_/bounceVel2_ each frame, growing essentially unbounded
   // (a real per-frame impulse vs. a decay rate ~180x slower) until the
   // accumulated bias dwarfed the raw random-walked vel1_/vel2_ entirely
   // and dictated the beams' apparent rotation itself, rather than merely
   // nudging them apart. Fixed by making the kick edge-triggered -- once
   // per collision EPISODE (the frame separation first drops under the
   // minimum), not once per frame the beams happen to still be touching
   // it -- a real single bounce impulse, not a ratcheting force. The
   // position clamp itself still runs every frame regardless (unchanged,
   // keeps the megabar guarantee unconditional).
   bool wasColliding_ = false;

   Timer frameTimer_; // tracks dt between update() calls

   // pot -> each beam's max rotation speed ceiling -- see class comment
   PotEnergyTakeover energyTakeover_;

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

   // CONE_HALF_WIDTH_DEG derivation (per explicit request: "widen the
   // beams to barely enough to include 2 megabars, confirm now at no time
   // is it possible for either beam to ever not have at least one megabar
   // lit"): computed offline from CarpetGeometry's real mbSpotXY ground-
   // spot table (NOT the nominal 30deg-apart naming) via
   // angleFromForward_(x,y) for all 12 megabars, giving real angles
   // {0, 28.685, 57.5145, 90, 122.4855, 151.315, 180, 208.685, 237.5145,
   // 270, 302.4855, 331.315}. For a symmetric window of half-width h
   // centered anywhere on the circle to guarantee >=2 of these points
   // fall within it at EVERY possible center position, the binding
   // (worst-case) constraint is a window centered EXACTLY ON a megabar
   // whose two neighbors are its own maximum single-gap distance away
   // (NOT half that gap -- centering between two points is the easier
   // case) -- verified by a fine-grained numerical sweep (0.005deg step)
   // over all 12 real megabar angles, every possible center position, and
   // both variations of a binary search on h: the exact threshold is
   // h=32.4855deg (equal to the real largest adjacent-megabar gap, which
   // is also, not coincidentally, exactly the distance from megabar[3]
   // (90deg) to megabar[4] (122.4855deg) or its 3 rotational-symmetry
   // equivalents). Set here with a +1.5deg safety margin above that exact
   // threshold (float-precision robustness, not part of the real
   // guarantee) -- rounds to 34deg.
   static constexpr float CONE_HALF_WIDTH_DEG = 34.0f;
   static constexpr float CONE_FULL_WIDTH_DEG = CONE_HALF_WIDTH_DEG * 2.0f;    // 68deg
   static constexpr float CONE_FADE_DEG = CONE_FULL_WIDTH_DEG * 0.25f;        // 17deg, beyond the guaranteed core
   // Brightness floor at the guaranteed core's own outer edge (not 0) --
   // see class comment: keeps the worst-case-farthest guaranteed megabar
   // always clearly lit even as brightness eases down from full center
   // brightness, rather than the old flat-255 plateau.
   static const uint8_t CORE_FLOOR_BRIGHTNESS = 128;
   // Minimum angular separation enforced between the 2 beams' cone
   // centers (see update()'s enforcement block) -- CONE_FULL_WIDTH_DEG
   // (so the 2 beams' guaranteed cores can just barely never touch) plus
   // a 2deg margin so a worst-case contested point can never land on an
   // exact brightness tie between "beam A's own core edge" (=
   // CORE_FLOOR_BRIGHTNESS) and "beam B's fade zone at the same
   // distance" -- confirmed via simulation (fine sweep over all valid
   // beam-angle-pairs honoring this minimum): worst-case guaranteed
   // winning brightness for each beam's own each-side megabar is ~197,
   // comfortably above CORE_FLOOR_BRIGHTNESS, let alone 0.
   static constexpr float MIN_BEAM_SEPARATION_DEG = CONE_FULL_WIDTH_DEG + 2.0f; // 70deg (max possible is 90deg -- 20deg of slack)

   // Rope-only cone width -- 25% narrower than the width above, per
   // explicit request. Megabars/china keep the full CONE_HALF_WIDTH_DEG:
   // that width is a hard geometric guarantee (see its own derivation
   // comment) -- shrinking it there would break "at least 2 megabars
   // always lit". Rope has no such guarantee to protect, so it's free to
   // be narrower purely for visual effect.
   static constexpr float ROPE_CONE_HALF_WIDTH_DEG = CONE_HALF_WIDTH_DEG * 0.75f;
   static constexpr float ROPE_CONE_FADE_DEG = CONE_FADE_DEG * 0.75f;

   // Per spec: full brightness at dead center, easing down to
   // CORE_FLOOR_BRIGHTNESS at the guaranteed core's own edge
   // (CONE_HALF_WIDTH_DEG), then continuing to ease from that floor down
   // to black over the further CONE_FADE_DEG zone. Both segments use the
   // same smoothstep8 shape and meet with matching (zero) tangent at the
   // join, so the combined curve has no visible kink. ONE shared shape
   // for rope/megabar/china alike (see class comment) -- no more
   // separately-tuned per-fixture-type widths.
   static uint8_t coneBrightnessAt( float d, float halfWidth = CONE_HALF_WIDTH_DEG, float fadeDeg = CONE_FADE_DEG ) {
      if ( d >= halfWidth + fadeDeg ) return 0;
      if ( d <= halfWidth ) {
         float frac = d / halfWidth; // 0..1
         uint8_t eased = smoothstep8( (uint8_t)( frac * 255.0f + 0.5f ) ); // 255->0 as frac 0->1
         return (uint8_t)( CORE_FLOOR_BRIGHTNESS + ( (uint32_t)eased * ( 255 - CORE_FLOOR_BRIGHTNESS ) ) / 255 );
      }
      float fadeFrac = ( d - halfWidth ) / fadeDeg; // 0..1
      uint8_t eased = smoothstep8( (uint8_t)( fadeFrac * 255.0f + 0.5f ) ); // 255->0 as fadeFrac 0->1
      return (uint8_t)( ( (uint32_t)eased * CORE_FLOOR_BRIGHTNESS ) / 255 );
   }

   // One beam's brightness contribution at a given point angle -- nearest
   // of its 2 opposite (180deg apart) cones. halfWidth/fadeDeg default to
   // the megabar/china-guaranteeing width; the rope loop passes its own
   // narrower ROPE_CONE_* constants instead (see their declaration
   // comment).
   static uint8_t beamBrightnessAt( float pointAngle, float beamAngle, float halfWidth = CONE_HALF_WIDTH_DEG, float fadeDeg = CONE_FADE_DEG ) {
      float d1 = fabsf( circularDelta( pointAngle, beamAngle ) );
      float d2 = fabsf( circularDelta( pointAngle, wrap360( beamAngle + 180.0f ) ) );
      return coneBrightnessAt( min( d1, d2 ), halfWidth, fadeDeg );
   }

   // No additive mixing where the 2 beams' cones overlap -- picks the
   // single brighter beam entirely, its own hue at its own computed
   // brightness. preferBeam2OnTie: per explicit request, an EXACT
   // brightness tie (real and non-negligible for megabars specifically,
   // since brightness is uint8_t-quantized to 256 levels -- not a
   // razor-thin float coincidence) must not always resolve to the same
   // beam; callers that iterate a small, fixed set of fixtures (megabars)
   // alternate this per-fixture so a tie always still lets each color
   // "win" somewhere rather than one color systematically losing every
   // tie. Defaults false (beam 1 wins ties) for rope/china, unchanged
   // from prior behavior -- alternation is opted into per-call, not
   // global. Returns CHSV, not CRGB -- stays in HSV space until the
   // actual write (see LightSetters.h); RGB conversion happens exactly
   // once, at that point, via FastLED's integer-only hsv2rgb_rainbow().
   static CHSV winnerColorAt( float pointAngle, float angle1, uint8_t hue1, float angle2, uint8_t hue2, uint8_t satByte, bool preferBeam2OnTie = false, float halfWidth = CONE_HALF_WIDTH_DEG, float fadeDeg = CONE_FADE_DEG ) {
      uint8_t b1 = beamBrightnessAt( pointAngle, angle1, halfWidth, fadeDeg );
      uint8_t b2 = beamBrightnessAt( pointAngle, angle2, halfWidth, fadeDeg );
      uint8_t winBright = max( b1, b2 );
      if ( winBright == 0 ) return CHSV( 0, 0, 0 );
      bool beam1Wins = ( b1 == b2 ) ? !preferBeam2OnTie : ( b1 > b2 );
      uint8_t winHue = beam1Wins ? hue1 : hue2;
      return CHSV( winHue, satByte, winBright );
   }

   // Shared china render: same beam-crossing color effect as rope/
   // megabar, at each china's own real position angle -- used by BOTH
   // variations now (see class comment).
   void renderChinaBeams_( uint8_t hue1Byte, uint8_t hue2Byte, uint8_t satByte ) {
      for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) {
         // dimAngleDeg (precomputed once in CarpetGeometry::begin(), not a
         // live atan2f here) -- ground-spot angle, not aim; see
         // CarpetGeometry.h's file header for why those two differ
         float a = CarpetGeometry::getChina( c ).dimAngleDeg;
         LightSetters::setColor( carpet, LightSetters::TargetChina, winnerColorAt( a, angle1_, hue1Byte, angle2_, hue2Byte, satByte ), LightSetters::ByAngleDeg{ a } );
         LightSetters::setWhite( carpet, LightSetters::TargetChina, 0, LightSetters::ByAngleDeg{ a } );
      }
   }

 public:
   LighthouseShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ) {}

   uint8_t variation() { return variation_; }
   uint8_t numVariations() { return numVariations_; }
   const char * variationName() {
      switch ( variation_ ) { LIGHTHOUSE_VARIATIONS(LIGHTSHOW_VARIATION_NAME_CASE) }
      return "?";
   }
   #undef LIGHTHOUSE_VARIATIONS // X-macro's job is done, keep the namespace clean

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
      // auto-init to the range midpoint, 85%, not the requested 80%)
      satWalk_.rampStart = satWalk_.rampTarget = 0.80f;
      satWalk_.initialized = true;
      satWalk_.tickTimer.set( 800 + random( 400 ) );
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
      float satFraction = satWalk_.value( 0.70f, 1.0f, 0.013f );

      // decay any active bounce bias (see its own declaration comment)
      // before applying it this frame -- exponential, time-constant
      // BOUNCE_DECAY_TAU_SEC, frame-rate independent.
      float bounceDecay = expf( -dtSec / BOUNCE_DECAY_TAU_SEC );
      bounceVel1_ *= bounceDecay;
      bounceVel2_ *= bounceDecay;

      angle1_ = wrap360( angle1_ + ( vel1_ + bounceVel1_ ) * dtSec );
      angle2_ = wrap360( angle2_ + ( vel2_ + bounceVel2_ ) * dtSec );

      // Prevent the 2 beams' cones from ever overlapping -- see class
      // comment and MIN_BEAM_SEPARATION_DEG's own comment. Each beam has
      // 2 antipodal cones, so the distance that matters is the CLOSEST of
      // all 4 possible cone-pairings: reduce the raw signed angle1/angle2
      // delta into 0..90 by that antipodal symmetry (effSep). If it's
      // ever below the minimum, push the 2 angles apart symmetrically
      // (50/50 split -- neither beam unilaterally "wins" position) by
      // exactly the shortfall, so the hard guarantee holds every single
      // frame, unconditionally. On top of that hard correction, also
      // inject a decaying outward bounceVel kick (see its own declaration
      // comment) so the pair actually separates again afterward instead
      // of staying rigidly pinned at the boundary.
      {
         float rawDelta = circularDelta( angle1_, angle2_ ); // -180..180, signed, angle1->angle2
         float absDelta = fabsf( rawDelta );
         float effSep = min( absDelta, 180.0f - absDelta );  // 0..90
         bool nowColliding = effSep < MIN_BEAM_SEPARATION_DEG;
         if ( nowColliding ) {
            float shortfall = MIN_BEAM_SEPARATION_DEG - effSep;
            float sign = ( rawDelta >= 0.0f ) ? 1.0f : -1.0f;
            // absDelta<=90: closest pairing is angle1 vs angle2 directly --
            // widen by growing |rawDelta|. absDelta>90: closest pairing is
            // the ANTIPODAL one (angle1 vs angle2+180) -- widen by
            // shrinking |rawDelta| back toward 90 instead.
            float pushDelta = ( absDelta <= 90.0f ) ? ( sign * shortfall ) : ( -sign * shortfall );
            angle1_ = wrap360( angle1_ - pushDelta * 0.5f );
            angle2_ = wrap360( angle2_ + pushDelta * 0.5f );

            // one-shot impulse, only on the frame this collision episode
            // began -- see wasColliding_'s own comment for why re-kicking
            // every frame while still touching was the actual bug.
            if ( !wasColliding_ ) {
               float kickSign = ( absDelta <= 90.0f ) ? sign : -sign; // matches pushDelta's own sign convention
               bounceVel1_ -= kickSign * BOUNCE_KICK_DEG_PER_SEC;
               bounceVel2_ += kickSign * BOUNCE_KICK_DEG_PER_SEC;
            }
         }
         wasColliding_ = nowColliding;
      }

      hue1_ += hueRate * dtSec;
      while ( hue1_ >= 256.0f ) hue1_ -= 256.0f;

      uint8_t hue1Byte = (uint8_t)hue1_;
      uint8_t hue2Byte = (uint8_t)( hue1Byte + 128 ); // complementary, wraps
      uint8_t satByte = (uint8_t)( satFraction * 255.0f + 0.5f );

      // --- rope: evaluated at each LED's own real angle, from
      // CarpetGeometry's shared, precomputed-once neopixel table (no more
      // per-show angle cache/derivation). Writes via LightSetters using
      // NeoByCircumferenceID, not NeoByAngleDeg -- this loop already knows
      // exactly which NeoID it's writing (no search needed), so ID
      // addressing goes through the setter abstraction at O(1)/pixel;
      // NeoByAngleDeg would instead be an O(1016) nearest-match search
      // PER pixel here (~1M comparisons/frame on a no-FPU MCU) for a
      // result already known for free. The color COMPUTATION itself is
      // 100% angle-based (winnerColorAt only ever sees real angles) --
      // only this array-addressing step is by ID.
      for ( uint16_t raw = 0; raw < NUM_NEO_LEDS_ACTUAL; ++raw ) {
         float a = CarpetGeometry::getNeoGeom( raw ).angleFromForwardDeg;
         CHSV color = winnerColorAt( a, angle1_, hue1Byte, angle2_, hue2Byte, satByte, false, ROPE_CONE_HALF_WIDTH_DEG, ROPE_CONE_FADE_DEG );
         int32_t neoId = CarpetGeometry::rawIndexToNeoId( raw );
         LightSetters::setColor( carpet, LightSetters::TargetNeo, color, LightSetters::NeoByCircumferenceID{ neoId } );
         LightSetters::setWhite( carpet, LightSetters::TargetNeo, 0, LightSetters::NeoByCircumferenceID{ neoId } );
      }

      // --- megabars: SAME falloff function as the rope, evaluated at each
      // megabar's own real position angle (CarpetGeometry). Alternates
      // the exact-tie tie-break by megabar index (preferBeam2OnTie) --
      // see winnerColorAt()'s own comment.
      for ( uint8_t m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         float a = CarpetGeometry::getMegabar( m ).dimAngleDeg; // ground-spot angle -- the only geometry a real show should ever consult
         bool preferBeam2OnTie = ( m % 2 ) != 0;
         LightSetters::setColor( carpet, LightSetters::TargetMegabar, winnerColorAt( a, angle1_, hue1Byte, angle2_, hue2Byte, satByte, preferBeam2OnTie ), LightSetters::ByAngleDeg{ a } );
      }

      // --- china: both variations now share the same beam-crossing
      // effect (see class comment) -- Default layers a white strobe on
      // top of it, VarNoStrobe doesn't.
      renderChinaBeams_( hue1Byte, hue2Byte, satByte );
      if ( variation_ == VarDefault ) updateStrobe_( time );
   }

 private:
   // triple-strobe on bass hits: 3 pulses, 30ms on/20ms gap. White-channel
   // only (see class comment) -- never touches china's RGB color, which
   // renderChinaBeams_() above has already set to that china's own
   // beam-crossing color this same frame; the two effects superimpose
   // instead of one overwriting the other.
   // BUGFIX ("only strobes a couple times then stops"): this used to
   // additionally require each new hit's level to EXCEED the previous
   // strobing hit's own level (suppressionPeak_), or a full 3s of silence,
   // before re-arming -- fine for a track with rising dynamics, but a
   // fairly steady bassline (typical of most music, including the Beats
   // synthetic patterns) hits roughly the same level every time, so only
   // the very first hit ever strobed and it never re-armed again short of
   // a genuine 3s silence gap. Simplified: any qualifying hit re-triggers
   // as soon as the PREVIOUS strobe sequence has finished (~130ms) --
   // strobeActive_ alone already prevents re-triggering mid-sequence, no
   // loudness/silence gating needed on top of that.
   bool strobeActive_ = false;
   Timer strobeTimer_;
   void updateStrobe_( uint32_t time ) {
      bool isHit = AudioBoard::getBandHitNonzero( BandBass );
      if ( isHit && !strobeActive_ ) {
         strobeActive_ = true; strobeTimer_.reset();
      }
      if ( !strobeActive_ ) return;
      static const uint32_t onMs = 30, gapMs = 20;
      static const uint32_t pulseStart[ 3 ] = { 0, onMs + gapMs, 2 * ( onMs + gapMs ) };
      uint32_t elapsed = strobeTimer_.elapsed();
      if ( elapsed >= pulseStart[ 2 ] + onMs ) { strobeActive_ = false; return; }
      bool lit = false;
      for ( int p = 0; p < 3; ++p ) if ( elapsed >= pulseStart[ p ] && elapsed < pulseStart[ p ] + onMs ) { lit = true; break; }
      if ( !lit ) return;
      LightSetters::setWhite( carpet, LightSetters::TargetChina, 255, LightSetters::ByAngleDeg{ 0.0f }, LightSetters::ByAngleDeg{ 359.999f } ); // full range -- every china
   }
};

#endif
