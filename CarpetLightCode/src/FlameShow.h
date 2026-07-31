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
      static uint32_t rate = 10;

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

      if ( time - timestamp > rate ) {
         timestamp = time;
         // cool everything
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            prevTemperature[ i ] = qsub8( prevTemperature[ i ], coolingRate );
         }

         // disperse heat
         for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            uint16_t lw = i > 0 ? i - 1 : NUM_NEO_LEDS_ACTUAL - 1;
            uint16_t hi = i < NUM_NEO_LEDS_ACTUAL - 1 ? i + i : 0;
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
         // Spark count is proportional, not fixed: the fraction of
         // floodlights sparkling per cycle equals the fraction of rope LEDs
         // sparkling per cycle (sparkingRate/255, the same per-LED
         // probability used just above), applied to each fixture count and
         // rounded to the nearest whole fixture -- floods are still chosen
         // at random.
         uint8_t floodCoolingRate = qadd8( coolingRate, coolingRate ); // 2x rope's rate, clamped at 255
         for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) megabarHeat[ i ] = qsub8( megabarHeat[ i ], floodCoolingRate );
         for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) chinaHeat[ i ] = qsub8( chinaHeat[ i ], floodCoolingRate );
         float sparkFraction = (float)sparkingRate / 255.0f;
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

      int dmxval = (int)AudioBoard::getNormalPercent( BandLow ) * 255 / 100; // AudioBoard's getters are percent now; converted back to this show's own 0-255 palette-index scale
      //Serial.println( dmxval );
      CRGB dmxclr = paletteColor( dmxval );
      LedUtil::gammaCorrect( dmxclr );
      LedUtil::fill( carpet->megabarLeds, dmxclr, NUM_MEGABAR_LEDS );
      LedUtil::fill( carpet->chinaLeds, dmxclr, NUM_CHINA_LEDS );

      // floodlight sparkle: blend each fixture's spark color on top of the
      // audio-tinted base, using heat itself as the blend fraction -- 0
      // stays the base color, 255 is a fully bright spark, so a pulse rises
      // and fades smoothly instead of snapping on/off. Same color source
      // (the active palette) and same gamma correction as the base, for
      // consistency -- see the megabarHeat_/chinaHeat_ member comment.
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
         CRGB sparkClr = paletteColor( megabarHeat[ i ] );
         LedUtil::gammaCorrect( sparkClr );
         carpet->megabarLeds[ i ] = blend( carpet->megabarLeds[ i ], sparkClr, megabarHeat[ i ] );
      }
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         CRGB sparkClr = paletteColor( chinaHeat[ i ] );
         LedUtil::gammaCorrect( sparkClr );
         carpet->chinaLeds[ i ] = blend( (CRGB)carpet->chinaLeds[ i ], sparkClr, chinaHeat[ i ] );
      }

      // use this instead of the rate
      delay( potval );
   }
};

