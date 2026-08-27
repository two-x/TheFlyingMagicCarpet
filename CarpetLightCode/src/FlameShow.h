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
#include "LightSetters.h"

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

// X-macro list: single source of truth for both the enum below and
// variationName() -- see LightShow.h's own comment on the pattern.
#define FLAME_VARIATIONS(X) X(VarWaterflames) X(VarFlames)

class FlameShow : public LightShow {
 private:
   enum Variation { FLAME_VARIATIONS(LIGHTSHOW_ENUM_ENTRY) };

   // TODO: tune this
   static const uint8_t baseCoolingRate = 10;
   static const uint8_t baseSparkingRate = 15; // maybe set this based on music?
   static const uint8_t numModes_ = 2;

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
   // idle-mode base-color coin flip (megabarColorPick_/chinaColorPick_
   // below) -- per explicit request, weighted so a flood is more often
   // sitting at the dim/base reference color than the bright one, rather
   // than a plain 50/50 flip. Still a real coin flip (not a fixed
   // population split) -- each fixture rerolls independently on its own
   // spark, same mechanism as before, just re-weighted.
   static const uint8_t BASE_COLOR_PICK_PERCENT = 65;
   static bool pickBaseColor() { return random8( 100 ) < BASE_COLOR_PICK_PERCENT; }
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
         chinaRole_[ c ] = ( CarpetGeometry::getChina( c ).dimY > 0.0f ) ? RoleFront : RoleRear;
      }
   }

   // sparkle cadence driven by the shared energy setting (see LightShow.h)
   // -- adjusting it here also becomes the new starting point for
   // NightriderShow and LighthouseShow
   PotEnergyTakeover energyTakeover_;

 public:

   FlameShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ), mode_( initialVariation % numModes_ ) {}

   uint8_t variation() {
      return mode_;
   }
   uint8_t numVariations() { return numModes_; }

   const char * variationName() {
      switch ( mode_ ) { FLAME_VARIATIONS(LIGHTSHOW_VARIATION_NAME_CASE) }
      return "?";
   }
   #undef FLAME_VARIATIONS // X-macro's job is done, keep the namespace clean

   // Both remaining variations use one of the 2 fixed 256-entry palettes
   // above. Floods share the exact same palette lookup as the rope now --
   // there's no independent drift left to track separately (that only
   // existed for the deleted shifting-hue variations).
   CRGB paletteColor( uint8_t index ) {
      return ( mode_ == VarWaterflames ) ? ColorFromPalette( waterflames, index ) : ColorFromPalette( flames, index );
   }

   void start() {
      CRGB clr = ColorFromPalette( flames, 0 );
      LightSetters::setColor( carpet, LightSetters::TargetNeo, clr, LightSetters::NeoByCircumferenceID{ 0 }, LightSetters::NeoByCircumferenceID{ (int32_t)NUM_NEO_LEDS_ACTUAL - 1 } );
      energyTakeover_.reset( carpet );

      // floods: real china front/rear roles, initial random per-fixture
      // idle color pick, initial china bass/treble swap timer -- see the
      // member comment above.
      classifyChinaFromGeometry();
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) megabarColorPick_[ i ] = pickBaseColor();
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) chinaColorPick_[ i ] = pickBaseColor();
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

      // shared energy setting drives the sparkle-cycle delay -- 100% (pot
      // fully up) = fastest cadence, 0% = slowest.
      float energyFrac = energyTakeover_.update( carpet ) / 100.0f; // 0..1, 1=hyper
      uint16_t potval = (uint16_t)( 255.0f * ( 1.0f - energyFrac ) + 0.5f ); // inverted: hyper -> less delay

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
            megabarColorPick_[ idx ] = pickBaseColor(); // re-roll idle base color on each spark
         }
         for ( int s = 0; s < chinaSparkCount; ++s ) {
            uint8_t idx = random8( NUM_CHINA_LEDS );
            chinaHeat[ idx ] = qadd8( chinaHeat[ idx ], random8( 160, 255 ) );
            chinaColorPick_[ idx ] = pickBaseColor();
         }

         // assign color
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            LightSetters::setColor( carpet, LightSetters::TargetNeo, paletteColor( currTemperature[ i ] ), LightSetters::NeoByCircumferenceID{ CarpetGeometry::rawIndexToNeoId( i ) } );
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
         // BUGFIX: FLOOD_LO_INDEX/FLOOD_HI_INDEX aren't equally bright colors
         // in the fire palette (index 40 caps around RGB(193,0,0), index 230
         // around RGB(255,229,0) -- roughly 3.6x apart in perceived
         // brightness), so a genuine 100% hit on both bands read as bass way
         // dimmer than treble. maximizeBrightness() scales each color's own
         // max channel up to 255 BEFORE the hit-percent dimmer is applied, so
         // a full hit reaches true full intensity on both bands regardless
         // of which palette hue each one happens to be.
         CRGB bassClr = paletteColor( FLOOD_LO_INDEX );
         bassClr.maximizeBrightness();
         bassClr.nscale8( (uint8_t)( bassHitPercent / 100.0f * 255.0f + 0.5f ) );
         LedUtil::gammaCorrect( bassClr );
         CRGB trebleClr = paletteColor( FLOOD_HI_INDEX );
         trebleClr.maximizeBrightness();
         trebleClr.nscale8( (uint8_t)( trebleHitPercent / 100.0f * 255.0f + 0.5f ) );
         LedUtil::gammaCorrect( trebleClr );
         for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
            LightSetters::setColor( carpet, LightSetters::TargetMegabar, ( i % 2 == 0 ) ? bassClr : trebleClr, LightSetters::ByID{ (uint8_t)i } );
         }
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
            bool showsBass = ( ( chinaRole_[ c ] == RoleFront ) == chinaSwapBassOnFront_ );
            LightSetters::setColor( carpet, LightSetters::TargetChina, showsBass ? bassClr : trebleClr, LightSetters::ByID{ (uint8_t)c } );
         }
      } else {
         // Idle sparkle: each fixture sits at one of the palette's two
         // reference colors (its own current megabarColorPick_/
         // chinaColorPick_, re-rolled on each spark -- see the spark loop
         // above), blended with its own spark heat on top -- 0 stays the
         // base color, 255 is a fully bright spark, so a pulse rises and
         // fades smoothly instead of snapping on/off.
         CRGB loColor = paletteColor( FLOOD_LO_INDEX );
         CRGB hiColor = paletteColor( FLOOD_HI_INDEX );
         for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
            CRGB base = megabarColorPick_[ i ] ? loColor : hiColor;
            CRGB sparkClr = paletteColor( megabarHeat[ i ] );
            CRGB clr = blend( base, sparkClr, megabarHeat[ i ] );
            LedUtil::gammaCorrect( clr );
            LightSetters::setColor( carpet, LightSetters::TargetMegabar, clr, LightSetters::ByID{ (uint8_t)i } );
         }
         for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
            CRGB base = chinaColorPick_[ c ] ? loColor : hiColor;
            CRGB sparkClr = paletteColor( chinaHeat[ c ] );
            CRGB clr = blend( base, sparkClr, chinaHeat[ c ] );
            LedUtil::gammaCorrect( clr );
            LightSetters::setColor( carpet, LightSetters::TargetChina, clr, LightSetters::ByID{ (uint8_t)c } );
         }
      }
   }
};

