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
#include "CarpetGeometry.h"

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

   // Floods (megabar/china) real base-color logic -- ported from the
   // visualizer's own JS mirror (showFlame() in carpet-visualizer.html),
   // which had been left completely unchanged since before the WASM
   // migration (predates the switch to real FW LED readback in Dev Tool
   // mode) while the real FW here had only ever done a flat single-color
   // fill. The JS mirror turns out to be the "golden" behavior everyone
   // remembered -- this replaces the old flat fill with the real thing.
   //
   // Hit-reactive gate: any real bass/treble hit within
   // HIT_REACTIVE_WINDOW_MS switches ALL floods to sound-reactive mode
   // (megabars alternate bass color/treble color per fixture; china
   // splits front/rear, with a periodic slow swap of which physical group
   // currently shows which color). Otherwise (idle) each fixture
   // independently sits at one of the palette's two reference colors
   // (cool/hot ends), re-picked whenever that fixture sparks -- sparkle
   // itself (megabarHeat_/chinaHeat_ above) is unchanged either way.
   //
   // China's real front/rear role comes from CarpetGeometry (see
   // classifyChinaFromGeometry(), called once in start()) rather than the
   // JS mirror's own old hardcoded FLAME_CHINA_FRONTBACK/SIDES index
   // lists -- china only has 2 distinct real ground-spot positions
   // (front/rear, no "side" case), same fix already applied to
   // SpeedStripesShow.h/BumpingAudioShow.h's china grouping.
   static const uint32_t HIT_REACTIVE_WINDOW_MS = 3000;
   static const uint8_t FLOOD_LO_INDEX = 40, FLOOD_HI_INDEX = 230; // palette reference points, matches the JS mirror
   Timer lastHitTimer_;
   bool everHit_ = false; // starts idle at boot, same as the JS mirror's flameLastHitMs=-99999
   bool megabarColorPick_[ NUM_MEGABAR_LEDS ]; // idle mode: true=lo, false=hi -- this fixture's current base color
   bool chinaColorPick_[ NUM_CHINA_LEDS ];
   bool chinaSwapBassOnFront_ = true;
   Timer chinaSwapTimer_;
   enum FbRole { RoleFront = 0, RoleRear = 1 };
   uint8_t chinaRole_[ NUM_CHINA_LEDS ];

   void classifyChinaFromGeometry() {
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
         chinaRole_[ c ] = ( CarpetGeometry::getChinaPosition( c ).yFt > 0.0f ) ? RoleFront : RoleRear;
      }
   }

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
   uint8_t numVariations() { return numModes_; }

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

      // floods: real china front/rear roles, initial random per-fixture
      // idle color pick, initial china bass/treble swap timer -- see the
      // member comment above.
      classifyChinaFromGeometry();
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) megabarColorPick_[ i ] = random8( 2 ) == 0;
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) chinaColorPick_[ i ] = random8( 2 ) == 0;
      chinaSwapTimer_.set( 2000 + random( 5000 ) );
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
            megabarColorPick_[ idx ] = random8( 2 ) == 0; // re-roll idle base color on each spark, matches JS mirror
         }
         for ( int s = 0; s < chinaSparkCount; ++s ) {
            uint8_t idx = random8( NUM_CHINA_LEDS );
            chinaHeat[ idx ] = qadd8( chinaHeat[ idx ], random8( 160, 255 ) );
            chinaColorPick_[ idx ] = random8( 2 ) == 0;
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

      // Floods -- see the class member comment above for the full design
      // (ported from the visualizer's showFlame() JS mirror).
      float bassHitPercent = AudioBoard::getBandHitPercent( BandBass );
      float trebleHitPercent = AudioBoard::getBandHitPercent( BandTreble );
      if ( bassHitPercent > 0.0f || trebleHitPercent > 0.0f ) { lastHitTimer_.reset(); everHit_ = true; }
      bool hitReactive = everHit_ && lastHitTimer_.elapsed() < HIT_REACTIVE_WINDOW_MS;

      if ( hitReactive ) {
         if ( chinaSwapTimer_.expired() ) {
            chinaSwapBassOnFront_ = !chinaSwapBassOnFront_;
            chinaSwapTimer_.set( 2000 + random( 5000 ) );
         }
         CRGB bassClr = floodPaletteColor( FLOOD_LO_INDEX );
         bassClr.nscale8( (uint8_t)( bassHitPercent / 100.0f * 255.0f + 0.5f ) );
         LedUtil::gammaCorrect( bassClr );
         CRGB trebleClr = floodPaletteColor( FLOOD_HI_INDEX );
         trebleClr.nscale8( (uint8_t)( trebleHitPercent / 100.0f * 255.0f + 0.5f ) );
         LedUtil::gammaCorrect( trebleClr );
         for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) carpet->megabarLeds[ i ] = ( i % 2 == 0 ) ? bassClr : trebleClr;
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
            bool showsBass = ( ( chinaRole_[ c ] == RoleFront ) == chinaSwapBassOnFront_ );
            carpet->chinaLeds[ c ] = showsBass ? bassClr : trebleClr;
         }
      } else {
         // Idle sparkle: each fixture sits at one of the palette's two
         // reference colors (its own current megabarColorPick_/
         // chinaColorPick_, re-rolled on each spark -- see the spark loop
         // above), blended with its own spark heat on top -- 0 stays the
         // base color, 255 is a fully bright spark, so a pulse rises and
         // fades smoothly instead of snapping on/off.
         CRGB loColor = floodPaletteColor( FLOOD_LO_INDEX );
         CRGB hiColor = floodPaletteColor( FLOOD_HI_INDEX );
         for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
            CRGB base = megabarColorPick_[ i ] ? loColor : hiColor;
            CRGB sparkClr = floodPaletteColor( megabarHeat[ i ] );
            CRGB clr = blend( base, sparkClr, megabarHeat[ i ] );
            LedUtil::gammaCorrect( clr );
            carpet->megabarLeds[ i ] = clr;
         }
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
            CRGB base = chinaColorPick_[ c ] ? loColor : hiColor;
            CRGB sparkClr = floodPaletteColor( chinaHeat[ c ] );
            CRGB clr = blend( base, sparkClr, chinaHeat[ c ] );
            LedUtil::gammaCorrect( clr );
            carpet->chinaLeds[ c ] = clr;
         }
      }
   }
};

