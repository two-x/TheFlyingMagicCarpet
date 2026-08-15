/* FlameShow.h
 *
 * It's fucking flames how cool is that. What else do you even need to know.
 *
 * TODO: but really, write a real description when you know how this works
 *
 * Author: Anders Linn
 * Date: August 2017
 */

#include "LightShow.h"
#include "AudioBoard.h"

// TODO: build a better flame palette. maybe use presets? or pass in a palette?
static const CRGBPalette256 flames(
      CRGB::DarkRed,
      CRGB::Red,
      CRGB::Orange,
      CRGB::Yellow );

  // Second, this palette is like the heat colors, but blue/aqua instead of red/yellow
static const CRGBPalette256 waterflames(
      CRGB::DarkBlue,
      CRGB::Blue,
      CRGB::Aqua,
      CRGB::White );
  
class FlameShow : public LightShow {
 private:
   enum Variation { VarWaterflames = 0, VarFlames = 1, VarShiftingHues = 2, VarHueToWhite = 3 };

   // TODO: tune this
   static const uint8_t baseCoolingRate = 10;
   static const uint8_t baseSparkingRate = 15; // maybe set this based on music?
   static const uint8_t numModes_ = 4;

   // variations 2/3 ("shifting hues" / "hue to white"): smoothly ramps
   // toward a freshly-randomized target every ~0.8-1.2s rather than jumping
   // -- same technique as LighthouseShow's beam hue rate, duplicated here
   // rather than shared (this codebase's convention: small per-show
   // helpers, not a shared header). Built on Timer rather than raw
   // millis()-diffing.
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
   RandomWalk hueRateWalk_;
   float shiftHue_ = 0.0f;   // 0-255, only ever increases (same "always one direction" rule as Lighthouse)
   Timer hueFrameTimer_;     // tracks dt between update() calls for the hue drift
   // Floods (megabar/china) used to share the rope's own shiftHue_/
   // shiftingPalette_ outright -- per request, their hue drift is now a
   // separate, independently-tracked value advancing at exactly 1/3 the
   // rope's rate (reusing the SAME per-frame hueRate, just divided by 3, so
   // it still speeds up/slows down proportionally with the pot exactly like
   // the rope's own drift does -- "same as the neos, just 3x slower").
   float floodShiftHue_ = 0.0f;
   CRGBPalette16 floodShiftingPalette_;
   // variation 2's saturation random-walks 87%-100%, same range/technique as
   // LighthouseShow's satWalk_ -- duplicated here per this codebase's
   // convention rather than shared.
   RandomWalk satWalk_;
   CRGBPalette16 shiftingPalette_;  // rebuilt once per update() call, not per-LED -- see paletteColor()

   uint8_t currTemperature[ NUM_NEO_LEDS_ACTUAL ] = { 0 };
   uint8_t prevTemperature[ NUM_NEO_LEDS_ACTUAL ] = { 0 };
   uint8_t mode_;

   // floodlight sparkle: china is treated as equivalent to megabars (same
   // treatment for both), per request. Independent per-fixture "heat" that
   // cools every cycle and gets exactly 2 fresh random sparks per cycle,
   // same cooling rate (so pulses last the same "width"/duration) and same
   // cycle cadence as the rope's own cool/spark step below (so pulse
   // frequency matches the rope's, which is the pot-adjustable delay() at
   // the bottom of update()). Rendered as a blend on top of the existing
   // audio-tinted dmxclr fill, using heat itself as the blend fraction, so
   // a spark rises and fades smoothly rather than snapping on/off.
   uint8_t megabarHeat[ NUM_MEGABAR_LEDS ] = { 0 };
   uint8_t chinaHeat[ NUM_CHINA_LEDS ] = { 0 };

   // Silence-only idle "breathing" base for floods (megabar/china) --
   // replaces the flat audio-tinted fill below, but ONLY while bass has
   // been quiet for a sustained period (floodIsSilent_ below -- real
   // sound reactivity is completely untouched and takes back over the
   // instant bass returns). Deliberately NOT AudioBoard::silent_: that
   // flag compares against noiseFloorPercent_, which defaults to 0% --
   // at 0% the threshold is literally zero, so silent_ can never become
   // true no matter how quiet the input actually is (confirmed: a fresh
   // boot's own status line prints "NoiseFl:0%"). Tracks the SAME bass
   // hit-percent already driving the reactive fill below instead, with
   // its own 4s hold (matching AudioBoard::silent_'s window) so a normal
   // gap between beats doesn't flicker between the two render modes.
   static const uint32_t FLOOD_SILENCE_HOLD_MS = 4000;
   Timer floodNotSilentTimer_;
   bool floodIsSilent_ = false;

   // Roughly half the fixtures sit at a resting IDLE_ON_BRIGHTNESS_FRAC brightness at any
   // moment, each smoothly fading to black on a slow, pot-adjustable,
   // staggered schedule while a different random one fades back up to
   // take its place -- floods "breathe" instead of sitting static at one
   // vivid color. Sparkle (megabarHeat_/chinaHeat_ below) layers on top
   // of either base unchanged, full brightness, per request.
   static constexpr float IDLE_ON_BRIGHTNESS_FRAC = 0.65f;
   static constexpr uint32_t IDLE_FADE_DURATION_MS = 2500; // time for one fixture to fade fully in or out -- exact number not specified, chosen to read as a slow, visible breathe
   static const int MAX_IDLE_FIXTURES = NUM_MEGABAR_LEDS > NUM_CHINA_LEDS ? NUM_MEGABAR_LEDS : NUM_CHINA_LEDS;
   struct IdleBreather {
      float frac[ MAX_IDLE_FIXTURES ] = { 0 };
      float target[ MAX_IDLE_FIXTURES ] = { 0 };
      bool inited = false;
      Timer swapTimer;

      // Slow, pot-adjustable cadence -- same energyFrac input every other
      // cadence in this file uses, but landing in a "several seconds"
      // range and explicitly 1/3 as frequent as sparkle's own energy
      // scaling, per request ("1/3 the energy level of the current
      // sparkle effects") -- same "same rate, divided by 3" precedent as
      // floodShiftHue_ above, just applied to a period instead of a rate.
      static uint32_t swapIntervalMs( float energyFrac ) {
         const float BASE_MS = 6000.0f, MIN_MS = 2000.0f;
         return (uint32_t)( BASE_MS - ( BASE_MS - MIN_MS ) * energyFrac );
      }

      void step( int count, float dtSec, float energyFrac ) {
         if ( !inited ) {
            inited = true;
            for ( int i = 0; i < count; ++i ) { target[ i ] = ( i % 2 == 0 ) ? 1.0f : 0.0f; frac[ i ] = target[ i ]; }
            swapTimer.set( swapIntervalMs( energyFrac ) );
         }
         if ( swapTimer.expired() ) {
            swapTimer.set( swapIntervalMs( energyFrac ) );
            int onCand[ MAX_IDLE_FIXTURES ], onCount = 0;
            int offCand[ MAX_IDLE_FIXTURES ], offCount = 0;
            for ( int i = 0; i < count; ++i ) {
               if ( target[ i ] > 0.5f ) onCand[ onCount++ ] = i;
               else offCand[ offCount++ ] = i;
            }
            if ( onCount > 0 ) target[ onCand[ random( onCount ) ] ] = 0.0f;
            if ( offCount > 0 ) target[ offCand[ random( offCount ) ] ] = 1.0f;
         }
         float maxStep = dtSec * 1000.0f / IDLE_FADE_DURATION_MS;
         for ( int i = 0; i < count; ++i ) {
            if ( frac[ i ] < target[ i ] ) frac[ i ] = min( target[ i ], frac[ i ] + maxStep );
            else if ( frac[ i ] > target[ i ] ) frac[ i ] = max( target[ i ], frac[ i ] - maxStep );
         }
      }
   };
   IdleBreather megabarIdle_, chinaIdle_;

   // sparkle cadence + variations 2/3's hue-rate ceiling both driven by the
   // shared energy setting (see LightShow.h) -- adjusting it here also
   // becomes the new starting point for NightriderShow and LighthouseShow
   PotEnergyTakeover energyTakeover_;

 public:

   FlameShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ), mode_( initialVariation % numModes_ ) {}

   uint8_t variation() {
      return mode_;
   }

   const char * variationName() {
      if ( mode_ == VarWaterflames ) return "waterflames";
      if ( mode_ == VarFlames ) return "flames";
      if ( mode_ == VarShiftingHues ) return "shifting hues";
      return "hue to white";
   }

   // mode_==0/1 use the fixed 256-entry palettes above. mode_==2/3 use
   // shiftingPalette_ (full saturation, fixed -- "as it is now"), rebuilt
   // once per update() call in update() itself, not here, so this stays
   // cheap to call per-LED:
   //   mode_==2: a 4-stop dark->bright glow all sharing one
   //             continuously-drifting hue.
   //   mode_==3: the same continuously-drifting hue at one end, fading to
   //             fixed white at the other -- "one color travels the hue
   //             spectrum... the other is always white".
   CRGB paletteColor( uint8_t index ) {
      if ( mode_ == VarWaterflames ) return ColorFromPalette( waterflames, index );
      if ( mode_ == VarFlames ) return ColorFromPalette( flames, index );
      return ColorFromPalette( shiftingPalette_, index );
   }
   // Floods' own version -- identical to paletteColor() for modes 0/1 (no
   // drift to slow down there, the palette's fixed), but modes 2/3 read
   // from floodShiftingPalette_ (the independently, 3x-slower-drifting
   // hue) instead of the rope's own shiftingPalette_.
   CRGB floodPaletteColor( uint8_t index ) {
      if ( mode_ == VarWaterflames ) return ColorFromPalette( waterflames, index );
      if ( mode_ == VarFlames ) return ColorFromPalette( flames, index );
      return ColorFromPalette( floodShiftingPalette_, index );
   }

   void start() {
      CRGB clr = ColorFromPalette( flames, 0 );
      LedUtil::fill( carpet->ropeLeds, clr, NUM_NEO_LEDS_ACTUAL );
      energyTakeover_.reset( carpet );

      // seed saturation's start explicitly (same reasoning as
      // LighthouseShow's velocity walks -- RandomWalk::value() would
      // otherwise auto-init to the range midpoint, 87.5%, not the
      // requested 90%)
      satWalk_.rampStart = satWalk_.rampTarget = 0.90f;
      satWalk_.initialized = true;
      satWalk_.tickTimer.set( 800 + random( 400 ) );
   }

   void update( uint32_t time ) {
      static uint32_t timestamp = 0;

      // pick color: each encoder detent cycles to the next color combo
      int delta = carpet->encoder->readPositionDelta();
      carpet->encoder->resetPositionDelta();
      if ( delta != 0 ) {
         int newMode = ( (int)mode_ + delta ) % (int)numModes_;
         if ( newMode < 0 ) newMode += numModes_;
         mode_ = (uint8_t)newMode;
      }


      // shared energy setting drives both the sparkle-cycle delay (as
      // before) AND variations 2/3's max hue rate (new) -- 100% (pot fully
      // up) = fastest cadence + fastest max hue drift, 0% = slowest of both.
      // (Fixes a real inversion bug found here during this refactor: the
      // old potFrac-from-potval math had the hue-rate ceiling backwards
      // -- pot UP used to give the LOWER 50% ceiling, contradicting both
      // this project's "pot=energy, up=hyper" convention and this file's
      // own README description, even though the cadence half was already
      // correct.)
      float energyFrac = energyTakeover_.update( carpet ) / 100.0f; // 0..1, 1=hyper
      uint16_t potval = (uint16_t)( 255.0f * ( 1.0f - energyFrac ) + 0.5f ); // inverted: hyper -> less delay

      // variations 2/3's hue drift -- advances every call (not just
      // rate-gated fire-sim cycles) so it stays smooth regardless of the
      // sparkle cadence. Energy scales the max rate: 0% -> 50% of the base
      // max, 100% -> 100% -- direction and the random-walk mechanism are
      // unchanged, only the ceiling it wanders within.
      float hueDtSec = (float)hueFrameTimer_.elapsed() / 1000.0f;
      hueFrameTimer_.reset();
      static const float BASE_MAX_HUE_RATE = 255.0f / 20.0f; // full spectrum in as little as 20s, same as LighthouseShow
      float maxHueRate = BASE_MAX_HUE_RATE * ( 0.5f + 0.5f * energyFrac );
      float hueRate = hueRateWalk_.value( 0.0f, maxHueRate, maxHueRate * 0.1f ); // never negative -- always one direction
      shiftHue_ += hueRate * hueDtSec;
      while ( shiftHue_ >= 256.0f ) shiftHue_ -= 256.0f;
      uint8_t shiftHueByte = (uint8_t)shiftHue_;
      // floods' own drift -- exactly 1/3 the rope's rate, see this member's
      // own comment above.
      floodShiftHue_ += ( hueRate / 3.0f ) * hueDtSec;
      while ( floodShiftHue_ >= 256.0f ) floodShiftHue_ -= 256.0f;
      uint8_t floodShiftHueByte = (uint8_t)floodShiftHue_;
      if ( mode_ == VarHueToWhite || mode_ == VarShiftingHues ) {
         // shared by both hue-shifting variations: saturation random-walks
         // 75%-100% (widened from a fixed/87%-100% range -- was reading as
         // too saturated all the time), starting at 90%, same range/
         // technique as LighthouseShow's satWalk_.
         uint8_t satByte = (uint8_t)( satWalk_.value( 0.75f, 1.0f, 0.013f ) * 255.0f + 0.5f );
         if ( mode_ == VarHueToWhite ) {
            // one color (the drifting hue) fading to the other (fixed white)
            CRGB driftClr = CHSV( shiftHueByte, satByte, 255 );
            shiftingPalette_ = CRGBPalette16( driftClr, driftClr, CRGB::White, CRGB::White );
            CRGB floodDriftClr = CHSV( floodShiftHueByte, satByte, 255 );
            floodShiftingPalette_ = CRGBPalette16( floodDriftClr, floodDriftClr, CRGB::White, CRGB::White );
         } else {
            // two complementary colors, both full brightness -- one drifting
            // hue (shiftHueByte, same rotation as before) and its complement
            // (+128), sharing the same random-walked saturation. Previously
            // this was a single hue ramped only through 4 brightness levels
            // (60/140/220/255), which read as dim/washed-out at the low end
            // -- replaced with an actual 2-color contrast instead of a
            // brightness ramp.
            CRGB colorA = CHSV( shiftHueByte, satByte, 255 );
            CRGB colorB = CHSV( (uint8_t)( shiftHueByte + 128 ), satByte, 255 );
            shiftingPalette_ = CRGBPalette16( colorA, colorA, colorB, colorB );
            CRGB floodColorA = CHSV( floodShiftHueByte, satByte, 255 );
            CRGB floodColorB = CHSV( (uint8_t)( floodShiftHueByte + 128 ), satByte, 255 );
            floodShiftingPalette_ = CRGBPalette16( floodColorA, floodColorA, floodColorB, floodColorB );
         }
      } else {
         shiftingPalette_ = CRGBPalette16( CHSV( shiftHueByte, 255, 60 ), CHSV( shiftHueByte, 255, 140 ),
                                           CHSV( shiftHueByte, 255, 220 ), CHSV( shiftHueByte, 255, 255 ) );
      }

      // uint8_t coolingRate = baseCoolingRate + potval;
      // uint8_t sparkingRate = baseSparkingRate + potval;
      uint8_t coolingRate = baseCoolingRate + 65;
      uint8_t sparkingRate = baseSparkingRate + 45;
      coolingRate = coolingRate > 255 ? 255 : coolingRate;
      sparkingRate = sparkingRate > 255 ? 255 : sparkingRate;

      // BUGFIX ("pot fritzing, whole strip strobes to black at ~8Hz at
      // some pot settings"): this cadence gate used to be a hardcoded
      // 10ms, with a SEPARATE plain (blocking) delay(potval) at the very
      // end of update() actually controlling the pot's effect on speed.
      // A blocking delay() freezes the entire main loop -- not just this
      // show, ALL button/encoder/I2C polling too -- for up to 255ms at a
      // time. Worse, since that blocking delay dominates the real elapsed
      // time between update() calls, the 10ms gate above always passed
      // trivially, so the ENTIRE cool+spark fire-sim pass ran as one lump
      // once per (potval-length) blocked cycle instead of many small
      // ticks. At pot~51%, potval~125ms -- almost exactly 8Hz -- and
      // applying the fixed-magnitude coolingRate constant just once per
      // that whole interval was enough to cool the entire rope to near-
      // black before the next lump of sparks partially relit it, reading
      // as a rope-wide strobe. Fixed by using potval AS this gate's own
      // threshold (non-blocking -- the codebase's own established pattern
      // for pot-controlled cadence elsewhere) and removing the blocking
      // delay() entirely below.
      if ( time - timestamp > potval ) {
         timestamp = time;
         // cool everything
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            prevTemperature[ i ] = qsub8( prevTemperature[ i ], coolingRate );
         }

         // disperse heat
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            uint16_t lw = i > 0 ? i - 1 : NUM_NEO_LEDS_ACTUAL - 1;
            uint16_t hi = i < NUM_NEO_LEDS_ACTUAL - 1 ? i + 1 : 0;
            currTemperature[ i ] = ( prevTemperature[ lw ] +
                                     prevTemperature[ hi ] +
                                     prevTemperature[ i ] ) / 3;
         }

         // add sparks
         // TODO: this isn't enough for us, we'll want to spread our sparks out around the car
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            if ( random8() < sparkingRate ) {
               currTemperature[ i ] = qadd8( currTemperature[ i ],
                                             random8( 160, 255 ) );
            }
         }

         // floodlight sparkle (see the class member comment): same cadence
         // as the rope (this whole block), but cools twice as fast -- a
         // shorter flash duration than the rope's own sparks, per request
         // (floods read as too long/lingering at the rope's cooling rate).
         //
         // BUGFIX ("pot only affects the energy of the neopixels, not the
         // floods"): floodSparkingRate used to be its own rate derived
         // from the FIXED sparkingRate constant (same one the rope always
         // uses, itself not pot-scaled -- only this whole block's own
         // CADENCE is pot-driven) -- floods and rope were two independent
         // simulations that merely happened to tick on the same clock, so
         // nothing tied the floods' own visible energy to how hot the rope
         // actually currently looks. Now measures the rope's own just-
         // computed currTemperature[] directly (its real, live average
         // heat, continually re-derived every tick) and drives floods from
         // THAT instead -- floods are now a direct reflection of the
         // rope's actual current energy, not a parallel, independently-
         // tuned process.
         uint32_t ropeTempSum = 0;
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) ropeTempSum += currTemperature[ i ];
         uint8_t ropeAvgTemp = (uint8_t)( ropeTempSum / NUM_NEO_LEDS_ACTUAL );
         uint8_t floodCoolingRate = qadd8( coolingRate, coolingRate ); // 2x rope's rate, clamped at 255
         for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) megabarHeat[ i ] = qsub8( megabarHeat[ i ], floodCoolingRate );
         for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) chinaHeat[ i ] = qsub8( chinaHeat[ i ], floodCoolingRate );
         uint8_t floodSparkingRate = ropeAvgTemp / 3; // /3 keeps the earlier "floods entirely too hyper" fix
         float sparkFraction = (float)floodSparkingRate / 255.0f;
         int megabarSparkCount = (int)( NUM_MEGABAR_LEDS * sparkFraction + 0.5f );
         int chinaSparkCount = (int)( NUM_CHINA_LEDS * sparkFraction + 0.5f );
         for ( int s = 0; s < megabarSparkCount; ++s ) {
            uint8_t idx = random8( NUM_MEGABAR_LEDS );
            megabarHeat[ idx ] = qadd8( megabarHeat[ idx ], random8( 160, 255 ) );
         }
         for ( int s = 0; s < chinaSparkCount; ++s ) {
            uint8_t idx = random8( NUM_CHINA_LEDS );
            chinaHeat[ idx ] = qadd8( chinaHeat[ idx ], random8( 160, 255 ) );
         }

         // assign color
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            carpet->ropeLeds[ i ] = paletteColor( currTemperature[ i ] );
         }

         // store prev color for next round
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            prevTemperature[ i ] = currTemperature[ i ];
         }
      } else {
         // NOTE: i don't think this is needed? wont the lights just keep thier color...?
         // assign color
         /*
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            CRGB newclr, oldclr;
            if ( mode ) {
               newclr = ColorFromPalette( flames, currTemperature[ i ] );
               oldclr = ColorFromPalette( flames, prevTemperature[ i ] );
            } else {
               newclr = ColorFromPalette( waterflames, currTemperature[ i ] );
               oldclr = ColorFromPalette( waterflames, prevTemperature[ i ] );
            }
            int val = scaleTo255( time - timestamp, rate, 0 );
            // Serial.println( val );
            carpet->ropeLeds[ i ] = blend( newclr, oldclr, val );
         }
         */
      }

      // BUGFIX ("floods hella dim/flickery"): this used to read
      // getNormalPercent() -- the RAW, instantaneous post-gain level,
      // which follows bass's actual waveform directly. At a bass
      // frequency of 40-80Hz that waveform crosses back near zero dozens
      // of times a second, so sampling it unsmoothed every loop iteration
      // made the flood fill flicker essentially randomly between dark and
      // lit rather than holding at a sustained brightness. getHitPercent()
      // is the peak-held value every other hit-driven show already uses
      // (decays smoothly over hitDecayMs_ instead of tracking the raw
      // wave), which is what a "how hard is bass hitting right now" flood
      // fill actually wants.
      // BUGFIX ("used to be good, floods always on now, not reactive"): a
      // 65% brightness FLOOR was added here alongside the switch above (to
      // match EqualizerShow's own floor, added for a genuinely different
      // complaint -- "reads dim at anything less than a maxed hit"), but
      // that's a separate concern from the flicker this function actually
      // fixes: getHitPercent() already decays smoothly to a real 0% in
      // silence, so the flicker problem was already solved without any
      // floor at all. The floor just meant floods could never read as
      // genuinely dark/off again, which is what "always on" was really
      // about. Removed -- silence now reads as silence.
      float bassHitPercent = AudioBoard::getBandHitPercent( BandBass );
      if ( bassHitPercent > 1.0f ) { floodNotSilentTimer_.reset(); floodIsSilent_ = false; }
      else if ( floodNotSilentTimer_.elapsed() >= FLOOD_SILENCE_HOLD_MS ) floodIsSilent_ = true;

      if ( floodIsSilent_ ) {
         // Silence-only idle "breathing" base -- see IdleBreather/
         // IDLE_ON_BRIGHTNESS_FRAC comment above.
         megabarIdle_.step( NUM_MEGABAR_LEDS, hueDtSec, energyFrac );
         chinaIdle_.step( NUM_CHINA_LEDS, hueDtSec, energyFrac );
         CRGB restColor = floodPaletteColor( 0 );
         for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
            CRGB clr = restColor;
            clr.nscale8( (uint8_t)( IDLE_ON_BRIGHTNESS_FRAC * megabarIdle_.frac[ i ] * 255.0f + 0.5f ) );
            LedUtil::gammaCorrect( clr );
            carpet->megabarLeds[ i ] = clr;
         }
         for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
            CRGB clr = restColor;
            clr.nscale8( (uint8_t)( IDLE_ON_BRIGHTNESS_FRAC * chinaIdle_.frac[ i ] * 255.0f + 0.5f ) );
            LedUtil::gammaCorrect( clr );
            carpet->chinaLeds[ i ] = clr;
         }
      } else {
         // Sound present -- exactly sha 59d94b4's sound-reactive fill,
         // untouched (only its 65% brightness floor was ever removed, see
         // BUGFIX comment above).
         float dmxvalFrac = bassHitPercent / 100.0f;
         int dmxval = (int)( dmxvalFrac * 255.0f + 0.5f );
         CRGB dmxclr = floodPaletteColor( dmxval );
         LedUtil::gammaCorrect( dmxclr );
         LedUtil::fill( carpet->megabarLeds, dmxclr, NUM_MEGABAR_LEDS );
         LedUtil::fill( carpet->chinaLeds, dmxclr, NUM_CHINA_LEDS );
      }

      // floodlight sparkle: blend each fixture's spark color on top of the
      // audio-tinted base, using heat itself as the blend fraction -- 0
      // stays the base color, 255 is a fully bright spark, so a pulse rises
      // and fades smoothly instead of snapping on/off. Same color source
      // (the active palette) and same gamma correction as the base, for
      // consistency -- see the megabarHeat_/chinaHeat_ member comment.
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
         CRGB sparkClr = floodPaletteColor( megabarHeat[ i ] );
         LedUtil::gammaCorrect( sparkClr );
         carpet->megabarLeds[ i ] = blend( carpet->megabarLeds[ i ], sparkClr, megabarHeat[ i ] );
      }
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         CRGB sparkClr = floodPaletteColor( chinaHeat[ i ] );
         LedUtil::gammaCorrect( sparkClr );
         carpet->chinaLeds[ i ] = blend( (CRGB)carpet->chinaLeds[ i ], sparkClr, chinaHeat[ i ] );
      }
   }
};

