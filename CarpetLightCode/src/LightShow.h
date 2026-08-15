/* LightShow.h
 *
 * The base class for creating light shows. The constructor takes a MagicCarpet
 * argument. Write directly to the led arrays in that MagicCarpet instance. Read
 * potentiometer/sound input from that MagicCarpet instance.
 *
 * The start function will be called by the main logic loop whenever the mode is
 * switched to this light show. Set up any initial state here.
 *
 * The update function will be called by the main logic loop as frequently
 * as possible. It takes a time argument which represents the current system time.
 * Use this to determine when to write new values.
 *
 * TODO: add functionality to add scheduled updates and write a common time checking
 *       class that calls each update function when the desired time increment is
 *       reached.
 */

#ifndef __LIGHTSHOW_H
#define __LIGHTSHOW_H

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "MagicCarpet.h"
#include "AudioBoard.h"

/*
DEFINE_GRADIENT_PALETTE( topColors_t ) {
   0, 0, 255, 127, // summer day
   // 4, 252, 25, 166,
   // 8, 255, 68, 45,
   // 12, 255, 95, 45,
   // 16, 45, 255, 201,
   // 20, 16, 150, 204,
   // 24, 212, 38, 71,
   // 28, 219, 211, 61,
   // 32, 122, 217, 255,
   36, 178, 118, 50, // end summer day
   // 40, 73, 191, 94,
   // 44, 53, 38, 79,
   // 48, 235, 130, 0,
   52, 178, 102, 9, // deep dark night
   // 56, 152, 51, 15,
   // 60, 100, 34, 10,
   // 64, 0, 227, 53,
   // 68, 168, 18, 176,
   // 72, 189, 10, 26,
   // 76, 255, 249, 64,
   // 80, 122, 153, 100,
   // 84, 15, 127, 64,
   // 88, 35, 164, 229,
   // 92, 6, 23, 127,
   // 96, 1, 5, 25,
   100, 0, 255, 72, // desert afternoon
   // 104, 204, 127, 20,
   // 108, 255, 211, 50,
   // 112, 114, 163, 255,
   // 116, 204, 129, 71,
   // 120, 126, 210, 197,
   // 124, 159, 79, 80,
   // 128, 173, 255, 126,
   // 132, 253, 120, 255,
   // 136, 204, 80, 84,
   // 140, 255, 125, 130,
   // 144, 127, 0, 52,
   // 148, 254, 65, 25,
   // 152, 177, 38, 9,
   // 156, 251, 98, 25,
   // 160, 88, 251, 190,
   164, 43, 123, 93, //under the sea
   // 168, 38, 212, 250,
   // 172, 199, 182, 10,
   // 176, 30, 0, 30,
   // 180, 88, 11, 109,
   // 184, 12, 196, 236,
   // 188, 60, 82, 134,
   // 192, 37, 51, 83,
   // 196, 42, 78, 106,
   // 200, 119, 96, 77,
   // 204, 93, 170, 139,
   // 208, 135, 246, 202,
   // 212, 32, 191, 220,
   // 216, 22, 233, 149,
   220, 236, 84, 81, // beach party
   //224, 185, 49, 45,
   //230, 239, 186, 106,
   //236, 120, 224, 244,
   //240, 246, 0, 100,
   //244, 0, 169, 9,
   //248, 74, 245, 83,
   //252, 133, 99, 247,
   256, 248, 182, 124
};

DEFINE_GRADIENT_PALETTE( bottomColors_t ) {
 0, 0, 201, 100,
 // 4, 153, 15, 101,
 // 8, 255, 25, 68,
 // 12, 178, 54, 16,
 // 16, 36, 204, 161,
 // 20, 45, 85, 255,
 // 24, 255, 71, 106,
 // 28, 255, 247, 97,
 // 32, 147, 224, 255,
 36, 254, 181, 97,
 // 40, 97, 255, 125,
 // 44, 36, 26, 53,
 // 48, 255, 152, 25,
 52, 0, 68, 101,
 // 56, 228, 77, 23,
 // 60, 0, 151, 35,
 // 64, 0, 87, 20,
 // 68, 241, 26, 252,
 // 72, 255, 39, 58,
 // 76, 255, 251, 140,
 // 80, 196, 229, 173,
 // 84, 45, 178, 151,
 // 88, 13, 47, 255,
 // 92, 4, 14, 76,
 // 96, 2, 10, 51,
 100, 76, 255, 127,
 // 104, 255, 169, 50,
 // 108, 191, 158, 37,
 // 112, 190, 213, 255,
 // 116, 76, 61, 50,
 // 120, 49, 82, 77,
 // 124, 255, 145, 101,
 // 128, 110, 178, 70,
 // 132, 254, 202, 255,
 // 136, 76, 53, 54,
 // 140, 255, 201, 204,
 // 144, 203, 0, 83,
 // 148, 101, 51, 40,
 // 152, 200, 6, 0,
 // 156, 225, 88, 22,
 // 160, 50, 173, 200,
 164, 50, 199, 143,
 // 168, 10, 66, 199,
 // 172, 168, 250, 38,
 // 176, 14, 0, 58,
 // 180, 160, 0, 63,
 // 184, 24, 135, 159,
 // 188, 93, 64, 185,
 // 192, 71, 98, 159,
 // 196, 72, 134, 182,
 // 200, 118, 132, 130,
 // 204, 10, 59, 68,
 // 208, 21, 125, 144,
 // 212, 101, 207, 226,
 // 216, 11, 209, 245,
 220, 182, 35, 99,
 // 224, 235, 155, 34,
 // 230, 242, 217, 180,
 // 236, 123, 191, 183,
 // 240, 118, 36, 69,
 // 244, 0, 246, 13,
 // 248, 59, 194, 66,
 // 252, 79, 51, 170,
 256, 248, 247, 50
};

static CRGBPalette256 topColors = topColors_t;
static CRGBPalette256 bottomColors = bottomColors_t;
*/

// prints "<label><value><suffix>" once, 1s after a pot-adjusted value stops
// changing -- not on every frame while it's still moving. General-purpose:
// covers any pot binding in normal run mode that doesn't already have its
// own live console feedback (config mode's screens print their own way
// instead -- see printLiveValue() in CarpetLightLogic.cpp). Duplicated as a
// tiny struct per this codebase's convention (see RandomWalk).
struct SettlePrinter {
   int lastValue = 0;
   bool armed = false; // false until the first update(), so boot doesn't print immediately
   Timer settleTimer{ 1000 };
   bool printed = true;

   void update( int value, const char * label, const char * suffix = "" ) {
      if ( !armed || value != lastValue ) {
         armed = true;
         lastValue = value;
         settleTimer.reset();
         printed = false;
      }
      if ( !printed && settleTimer.expired() ) {
         Serial.print( label );
         Serial.print( value );
         Serial.println( suffix );
         printed = true;
      }
   }
};

// shared "how energetic" setting, 0-100%, pot-driven, shared across every
// show/variation where the pot represents the light show's overall energy
// level (NightriderShow's auto-cycle variation, FlameShow's fire-sim cadence
// + hue-drift ceiling, LighthouseShow's rotation ceiling). Adjusting the pot
// in any ONE of them updates this one value, so it becomes the new starting
// point for all the others too, next time you're on one of them. Not
// persisted to flash -- live-only, same as each of these was before.
static float globalEnergyPercent = 100.0f; // placeholder until the real pot is read for the very first time -- see globalEnergyEverTakenOver_ below
// BUGFIX ("Flame is wildly hyperactive on first-ever entry, calms down
// after touching the pot"): globalEnergyPercent's hardcoded 100.0f default
// used to stick until the pot moved >=2% from wherever it was when the
// CURRENT show's own PotEnergyTakeover::reset() ran -- fine for switching
// BETWEEN shows that already share a committed energy level (deliberately
// avoids a jump), but wrong for the very first show entered in the group
// all session: there's no prior committed value to protect yet, so
// defaulting to 100% (rather than wherever the pot's actually resting)
// meant fresh sessions started at max fire-sim cadence/hue-rate -- looks
// chaotic on Flame specifically since cooling/sparking magnitude per step
// is fixed, so cadence alone (potval) controls real-time churn rate.
// Fixed by treating the very first update() call across the WHOLE group
// (tracked once, globally, never per-show) as an automatic takeover.
static bool globalEnergyEverTakenOver_ = false;
static SettlePrinter globalEnergyPrinter_; // shared across the whole group, not per-show

// small reusable per-show soft-takeover tracker binding the pot to
// globalEnergyPercent -- duplicated as a tiny struct (not a full
// base-class mechanism), matching this codebase's existing
// duplicate-small-helpers convention (see RandomWalk in FlameShow.h/
// LighthouseShow.h/SpeedStripesShow.h). Each show in the shared group
// declares one member, calls reset() from its own start() (so switching TO
// that show never jumps to wherever the pot happens to be), and update()
// once per frame to get this frame's energy percent.
struct PotEnergyTakeover {
   float entryPercent = 0.0f;
   bool takenOver = false;

   void reset( MagicCarpet * carpet ) {
      entryPercent = carpet->pot->readPercent();
      takenOver = false;
   }

   // returns 0-100%: the shared value if this show's pot hasn't moved
   // enough yet since reset(), or the live pot position (updating the
   // shared value, for every show in the group) once it has
   float update( MagicCarpet * carpet ) {
      float potPercent = carpet->pot->readPercent();
      if ( !globalEnergyEverTakenOver_ ) {
         // nothing committed yet anywhere in the group -- no prior value to
         // protect against jumping away from, so just take the real pot
         // reading immediately rather than sitting at the 100% placeholder
         takenOver = true;
         globalEnergyEverTakenOver_ = true;
      } else if ( !takenOver ) {
         static const float THRESHOLD_PERCENT = 2.0f;
         if ( fabsf( potPercent - entryPercent ) >= THRESHOLD_PERCENT ) takenOver = true;
      }
      if ( takenOver ) globalEnergyPercent = potPercent;
      globalEnergyPrinter_.update( (int)( globalEnergyPercent + 0.5f ), "energy:", "%" );
      // Floor the RETURNED value only (not globalEnergyPercent itself) --
      // a pot resting at literal 0% (the visualizer's default, and in
      // principle reachable on real hardware too) would otherwise multiply
      // straight through to zero velocity/cadence in shows that do
      // value*energyFrac directly (Lighthouse), freezing them dead until
      // the pot moves. Flooring the stored value instead was tried first
      // and didn't work -- the next frame just re-synced to the still-0%
      // pot and undid it.
      static const float ENERGY_FLOOR_PERCENT = 20.0f;
      return fmaxf( globalEnergyPercent, ENERGY_FLOOR_PERCENT );
   }
};

class LightShow {
 protected:
   MagicCarpet * carpet;
 public:
   // static CRGB getColor( uint8_t paletteIndex, uint8_t colorIndex ) {
   //    // CRGB clr1 = ColorFromPalette( topColors, paletteIndex );
   //    // CRGB clr2 = ColorFromPalette( bottomColors, paletteIndex );
   //    // Serial.println( "CRGB1" );
   //    // Serial.println( clr1.r );
   //    // Serial.println( clr1.g );
   //    // Serial.println( clr1.b );
   //    // Serial.println( "CRGB2" );
   //    // Serial.println( clr2.r );
   //    // Serial.println( clr2.g );
   //    // Serial.println( clr2.b );
   //    return blend( clr1, clr2, colorIndex );
   // }

   LightShow( MagicCarpet * carpetArg ) : carpet( carpetArg ) {}
   virtual void start() = 0;
   virtual void update( uint32_t timestamp ) = 0;
   // current variation index, for shows with more than one (encoder-selected).
   // shows without variations can leave this as the default.
   virtual uint8_t variation() { return 0; }
   // short (<30 char), human-readable name for the CURRENT variation() value
   // -- used for the console print on selection (see CarpetLightLogic.cpp).
   // shows without meaningful variations can leave this as the default.
   virtual const char * variationName() { return "default"; }
   // total number of selectable variations this show has -- used by the
   // visualizer to enumerate the real variation list at launch (never
   // hardcoded there, see tools/wasm/bridge/web_bridge.cpp's
   // web_getNumVariations()). Shows without meaningful variations can
   // leave this as the default.
   virtual uint8_t numVariations() { return 1; }
   virtual ~LightShow() {}
};

#endif
