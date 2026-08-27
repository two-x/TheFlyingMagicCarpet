/* MagicCarpet.h
 *
 *    This is a data abstraction of the carpet.
 *
 *    TODO: we might want to add fade-in functionality here, so that instead of
 *          changing directly to the next desired color we fade from the previous
 *          one. Right now, any such fading has to be written into the light shows.
 *          If we do this, we should add a toggle to allow light shows to disable
 *          automatic fades if they want to.
 *    TODO: total power consumption throttle
 *
 *    Author: Anders Linn
 *    Date: June 2017
 */

#ifndef __MAGIC_CARPET_H
#define __MAGIC_CARPET_H

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "LedController.h"
#include "AudioBoard.h"
#ifndef __EMSCRIPTEN__
#include "ArmDmx.h" // #includes "sam.h" at file scope -- genuinely can't compile under Emscripten regardless of whether dmx_init/dmx_send are called (see the migration plan)
#include "Ws281xDma.h" // right/left long strips -- see its own header comment. Same Emscripten-incompatibility reason as ArmDmx.h above.
#endif
#include "Nvm.h"
#include "SpeedLink.h"

// Controller constants
#define POT_ANALOG_PIN 3
#define ENCODER_SW_PIN 3 // FIXME: same pin number as POT_ANALOG_PIN, likely a wiring/config bug
#define ENCODER_A_PIN 5
#define ENCODER_B_PIN 4

// DMX constants
#define NUM_MEGABAR_LEDS 12
#define NUM_CHINA_LEDS 8
#define NUM_DMX_LEDS NUM_MEGABAR_LEDS + NUM_CHINA_LEDS
#define SIZEOF_MEGABAR_LEDS ( NUM_MEGABAR_LEDS * sizeof( CRGB ) )
#define SIZEOF_CHINA_LEDS ( NUM_CHINA_LEDS * sizeof( CRGBWAU ) )
#define TOTAL_DMX_SIZE ( SIZEOF_MEGABAR_LEDS + SIZEOF_CHINA_LEDS )

// Neopixel constants
// TODO: convert the user-facing values to constexpr global vars
//
// BUGFIX/HARDWARE CHANGE: the 2 long strips (right/left, 352 LEDs each)
// moved off the PORTD bank onto their own dedicated USART DMA channels
// (see Ws281xDma.h) -- driving them there instead of bit-banging frees up
// real loop time and was the actual point of the physical rewiring.
// PHASE-1 FOLLOWUP: rear ALSO moved off the shared PORTD block driver, not
// onto USART (that's a later phase) but onto its own independent
// FastLED ClocklessController<> instance on the SAME physical pin (26) --
// this was needed to give rear its own relaxed bus timing, since
// FastLED's block driver only supports ONE shared timing config across
// every lane it drives (see setup()'s own comment). NUM_NEOPIXEL_STRIPS
// is now 1 -- just front, the only strand left on the actual shared
// PORTD block driver.
#define SIZEOF_SMALL_NEO 156 // 108
#define SIZEOF_LARGE_NEO 352 // 145
#define SIZEOF_SMALL_NEO_HALF 78
#define SIZEOF_LARGE_NEO_HALF 176
#define SIZEOF_LARGE_NEO_CORNER 33
#define NUM_NEOPIXEL_STRIPS 8
#define NUM_NEO_SMALL_LEDS ( NUM_NEOPIXEL_STRIPS / 2 )
#define NUM_NEO_LARGE_LEDS NUM_NEO_SMALL_LEDS
#define NUM_NEO_LEDS ( ( SIZEOF_SMALL_NEO * NUM_NEO_SMALL_LEDS ) + \
                       ( SIZEOF_LARGE_NEO * NUM_NEO_LARGE_LEDS ) )
#define NUM_NEO_LEDS_ACTUAL ((SIZEOF_SMALL_NEO * 2) + (SIZEOF_LARGE_NEO * 2))
#define NUM_NEO_LEDS_PER_STRIP LedUtil::resizeCRGBW( SIZEOF_LARGE_NEO )
#define NUM_NEO_SHOW_LEDS ( NUM_NEO_LEDS_PER_STRIP * NUM_NEOPIXEL_STRIPS )
#define SIZEOF_NEO_STRIP ( NUM_NEO_LEDS_PER_STRIP * sizeof( CRGB ) )
#define SIZEOF_NEO_LEDS ( NUM_NEO_LEDS_ACTUAL * sizeof( CRGBW ) )
#define SIZEOF_NEO_SHOW_LEDS ( NUM_NEOPIXEL_STRIPS * SIZEOF_NEO_STRIP )
// BUGFIX ("whole segments of the neopixels are simply never lit" on the
// left side, in every show that addresses rope pixels by side/Y position
// -- SpeedStripesShow's renderSide()/renderZebraSide(), and the NEO3_OFFSET
// clear loops in NightriderShow::start()/BumpingAudioShow::start()):
// NEO0-3_OFFSET are the logical Front/Right/Rear/Left strand-start offsets
// into ropeLeds[] (see CarpetGeometry.h's buildStrandInfo_(), the only
// place that ties these 4 to their CarSide meaning). NEO0/NEO1/NEO2 happen
// to land correctly by arithmetic coincidence, but NEO3 (Left's start) was
// computed as if Rear (the strand immediately before Left) were
// SIZEOF_LARGE_NEO, a leftover assumption from an old 8-lane PORTD
// topology this project no longer has -- Rear is actually SIZEOF_SMALL_NEO,
// same as Front. That put Left's local index 0 exactly
// (SIZEOF_LARGE_NEO - SIZEOF_SMALL_NEO) = 196 pixels too far into the
// array, so every Left-side show loop (which iterates exactly
// SIZEOF_LARGE_NEO local indices) silently orphaned the first 196 real
// Left-strand pixels (never addressed by any local index at all) while
// wrapping its last 196 local indices around onto Front's own raw range
// instead. Right is unaffected (its own true start, SIZEOF_SMALL_NEO,
// doesn't depend on Rear's size at all).
#define NEO0_OFFSET 0 // small -- Front
#define NEO1_OFFSET SIZEOF_SMALL_NEO // large -- Right
#define NEO2_OFFSET ( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO ) // small -- Rear
#define NEO3_OFFSET ( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO ) // large -- Left
#define NEO4_OFFSET ( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO * 2 ) // small
#define NEO5_OFFSET ( SIZEOF_SMALL_NEO * 3 + SIZEOF_LARGE_NEO * 2 ) // large
#define NEO6_OFFSET ( SIZEOF_SMALL_NEO * 3 + SIZEOF_LARGE_NEO * 3 ) // large
#define NEO7_OFFSET ( SIZEOF_SMALL_NEO * 3 + SIZEOF_LARGE_NEO * 4 ) // small
// The pin order for the port bank we are using is: 25,26,27,28,14,15,29,11
// The stock block-driver enums only offer two timings: WS2811_PORTD (800kHz,
// 320/320/640ns) or WS2811_400_PORTD (400kHz, 800/800/900ns) -- 400kHz was
// too slow and still flickered. Instead setup() instantiates the PORTD block
// driver directly (InlineBlockClocklessController) with these hand-tuned
// intermediate timings (ns) so we can relax the bus by a smaller amount. The
// block driver shares ONE timing across all lanes, so this slows the whole
// display -- acceptable to fix the one flickering strand. Currently ~625kHz
// (400/400/800); nudge back toward 320/320/640 for faster, up for slower.
// #define NEO_PORT_BANK ( WS2811_PORTD )
#define NEO_BANK_T1 400
#define NEO_BANK_T2 400
#define NEO_BANK_T3 800
#define NEO_PIN0 25 // FRONT
#define NEO_PIN1 26 // LEFT
#define NEO_PIN2 27
#define NEO_PIN3 28
#define NEO_PIN4 14 // RIGHT
#define NEO_PIN5 15 // REAR
#define NEO_PIN6 29
#define NEO_PIN7 11

/* Positional Constants
 *
 * These are aliases for the index values of the carpet's led arrays. Their are two
 * different types defined: directional and temporal. Both led array start from the
 * midpoint of the front of the carpet (FRONT or TWELVE). So far these are defined
 * only for the rope light arrays, but we can do this for the dmx leds too if we
 * feel the need.
 */
#define FRONT (SIZEOF_SMALL_NEO_HALF)
#define FRONT_RIGHT (SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO_CORNER)
#define RIGHT (SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO_HALF)
#define REAR_RIGHT (SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO - SIZEOF_LARGE_NEO_CORNER)
#define REAR (SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO + SIZEOF_SMALL_NEO_HALF)
#define REAR_LEFT ((SIZEOF_SMALL_NEO * 2) + SIZEOF_LARGE_NEO + SIZEOF_LARGE_NEO_CORNER)
#define LEFT ((SIZEOF_SMALL_NEO * 2) + SIZEOF_LARGE_NEO + SIZEOF_LARGE_NEO_HALF)
#define FRONT_LEFT ((SIZEOF_SMALL_NEO * 2) + (SIZEOF_LARGE_NEO * 2) - SIZEOF_LARGE_NEO_CORNER)

#define TWELVE FRONT
#define ONE_THIRTY FRONT_RIGHT
#define THREE RIGHT
#define FOUR_THIRTY REAR_RIGHT
#define SIX REAR
#define SEVEN_THIRTY REAR_LEFT
#define NINE LEFT
#define TEN_THIRTY FRONT_LEFT

// index of the center-front megabar pair (addr 01, two physical fixtures,
// same DMX address) -- see README.md, "Megabars"
#define HEADLIGHT_INDEX 0

class MagicCarpet {
 private:

   // see README.md, "Brightness system"
   float globalBrightness_ = 100.0f;   // 0-100
   float headlightBrightness_ = 50.0f; // 50-100, per-fixture (see README -- default
                                        // 50% per fixture makes the combined pair
                                        // match one normal megabar's brightness)
   float chinaBrightness_ = 100.0f;    // 0-100

   // converts a percentage (0-100) into a raw linear-domain 0-255 value.
   // the only place a percent should ever become a raw hardware value.
   static uint8_t percentToRaw( float percent ) {
      percent = constrain( percent, 0.0f, 100.0f );
      return (uint8_t)( ( percent / 100.0f ) * 255.0f + 0.5f );
   }

   // like delay(), but keeps the button's state machine alive throughout --
   // a plain delay() goes totally blind to button input for its duration,
   // which for flashRope()'s multi-flash sequences (up to ~460ms) is long
   // enough to drop part of a rapid follow-up double-press.
   //
   // BUGFIX: under __EMSCRIPTEN__ this was a genuine, unrecoverable infinite
   // loop -- HalShim's millis() is frozen for the whole duration of one
   // web_tick() call (it only advances at the START of the next JS
   // animation frame) and delay() there is a no-op, so nothing could ever
   // make waitTimer.expired() become true from inside this synchronous
   // call. On real hardware delay(1) genuinely lets time pass each
   // iteration, so this works fine there -- the loop itself is correct,
   // it's the WASM environment that can't support "block briefly while
   // real time passes" at all (there's no such thing as a "next frame"
   // until this call returns). Skip the wait entirely under WASM instead
   // of hanging -- costs flashRope() its real timing/pacing there (visual
   // polish only, a long-press threshold-cross cue), not real hardware
   // behavior, which this #ifdef doesn't touch.
#ifdef __EMSCRIPTEN__
   void delayPolling( uint32_t ms ) {
      encoder->button.update();
   }
#else
   void delayPolling( uint32_t ms ) {
      Timer waitTimer( ms );
      do {
         encoder->button.update();
         delay( 1 );
      } while ( !waitTimer.expired() );
   }
#endif

   /* FastLED doesn't support rgbw leds. We work around this by offsetting the color
    * values to accomodate the white value. See CRGBW.h for more details.
    *
    * These are the real, post-convertNeoArray() bus-write buffers -- the
    * exact bytes show() hands to FastLED/Ws281xDma each frame (RGBW source
    * pixels repacked 3-into-4 as fake RGB triplets for the WS281x wire
    * protocol -- see LedUtil::convertNeoArray()'s own comment). They're
    * populated for real under __EMSCRIPTEN__ too (unconditionally, unlike
    * the hardware calls below), so the visualizer can read the true
    * bus-write bytes rather than only the pre-conversion ropeLeds[] --
    * NOTE this is raw wire-order data, not directly renderable per-fixture
    * without inverting the repacking (deliberately not attempted yet, see
    * the migration plan's "phase 6").
    */
   CRGB ropeShowLeds[ NUM_NEO_SHOW_LEDS ];

 public:

   /* DON'T CHANGE THE ORDER OF THESE ARRAYS!
    * They're declared separately to make them easy to work with, but treated as a
    * single continguous array when passed into dmx_send.
    */
   CRGB megabarLeds[ NUM_MEGABAR_LEDS ];
   CRGBWAU chinaLeds[ NUM_CHINA_LEDS ];

   // neopixel leds
   CRGBW ropeLeds[ NUM_NEO_LEDS_ACTUAL ];

   // controls
   LedControl::Potentiometer * pot;
   LedControl::Encoder * encoder; // includes the encoder's integrated shaft-click button (encoder->button)

   // brightness, in percent -- see the comment on the private fields above
   void setGlobalBrightness( float percent ) {
      globalBrightness_ = constrain( percent, 0.0f, 100.0f );
   }
   float getGlobalBrightness() {
      return globalBrightness_;
   }

   void setHeadlightBrightness( float percent ) {
      headlightBrightness_ = constrain( percent, 50.0f, 100.0f );
   }
   float getHeadlightBrightness() {
      return headlightBrightness_;
   }

   void setChinaBrightness( float percent ) {
      chinaBrightness_ = constrain( percent, 0.0f, 100.0f );
   }
   float getChinaBrightness() {
      return chinaBrightness_;
   }

   void setup() {
      // seed random so we always get different random patterns
      randomSeed( analogRead( 0 ) );

      Nvm::load(); // recall persisted show/variation selection before anything else needs it

      AudioBoard::setup();
      SpeedLink::setup(); // Wire I2C slave (pins 20/21), receives vehicle telemetry from CANTroller2

      digitalWrite( 2, HIGH );

      // set up the controller
      pot = new LedControl::Potentiometer( POT_ANALOG_PIN );
      encoder = LedControl::getEncoder( ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_SW_PIN );

#ifndef __EMSCRIPTEN__
      // add dmx leds
      dmx_init( TOTAL_DMX_SIZE );


      // add eight channels of rope leds via the PORTD block driver, using our
      // own relaxed timing (NEO_BANK_T* above) instead of a stock enum. GRB
      // matches the 2-arg block addLeds default and the CRGBW packing scheme
      // (see Ws281xDma.h's encode() comment).
      FastLED.addLeds( new InlineBlockClocklessController<
                          NUM_NEOPIXEL_STRIPS, PORTD_FIRST_PIN,
                          NS( NEO_BANK_T1 ), NS( NEO_BANK_T2 ), NS( NEO_BANK_T3 ),
                          GRB>(),
                       ropeShowLeds, NUM_NEO_LEDS_PER_STRIP );
#endif
      // WASM: no real DMX/NeoPixel hardware to attach -- see the migration
      // plan's output-path design (phase 5: protocol-level output
      // simulation) for what replaces this.

      clear(); // there might be stale values left in the leds, start from scratch

      show();
   }

   void show() {
      /* the lights aren't always arranged the same way as they are addressed, so in
       * some cases we need to reverse whatever has been programmed in the user array
       * in order to keep the addressing consistent. This is done in-place.
       *
       * TODO: if we start running too slow we can look at ways to get around this
       *       reversal
       */
      LedUtil::reverse( ropeLeds, SIZEOF_SMALL_NEO );
      LedUtil::reverse( ropeLeds + SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO, SIZEOF_SMALL_NEO );

      // FRONT
      LedUtil::convertNeoArray( ropeLeds, ropeShowLeds,
                                SIZEOF_SMALL_NEO );
      // RIGHT
      LedUtil::convertNeoArray( ropeLeds + SIZEOF_SMALL_NEO,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 4,
                                SIZEOF_LARGE_NEO );

      // REAR
      LedUtil::convertNeoArray( ropeLeds + SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP * 5,
                                SIZEOF_SMALL_NEO );

      // LEFT
      LedUtil::convertNeoArray( ropeLeds + (SIZEOF_SMALL_NEO * 2) + SIZEOF_LARGE_NEO,
                                ropeShowLeds + NUM_NEO_LEDS_PER_STRIP,
                                SIZEOF_LARGE_NEO );


      // make sure to reverse the values so the user has a consistent view
      LedUtil::reverse( ropeLeds, SIZEOF_SMALL_NEO );
      LedUtil::reverse( ropeLeds + SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO, SIZEOF_SMALL_NEO );

#ifndef __EMSCRIPTEN__
      // we don't have to pass the china light array separately. Instead, we treat
      // both arrays as a single big array, since they're contiguous in memory.
      dmx_send( ( uint8_t * ) megabarLeds );

      FastLED.show();
#endif
      // WASM: no real bus write (no hardware) -- but ropeShowLeds
      // above are now real,
      // populated bus-write buffers regardless of target, and
      // megabarLeds/chinaLeds ARE the literal DMX bus buffer already
      // (dmx_send() sends their raw memory directly, no conversion step
      // -- see CRGBWAU's own comment), so all 3 real output buffers are
      // available to read post-show() on any target. Rendering from
      // ropeShowLeds still isn't
      // attempted (wire-order data, not directly per-fixture -- see their
      // declaration comment); the visualizer keeps rendering from
      // ropeLeds[]/megabarLeds[]/chinaLeds[] directly.
   }

   void clearMegabars() {
      memset( megabarLeds, 0, SIZEOF_MEGABAR_LEDS );
   }

   void clearChinas() {
      memset( chinaLeds, 0, SIZEOF_CHINA_LEDS );
   }

   // china fixtures are 6-channel RGBWAU -- CRGBWAU::u (aliased "black" in
   // the union, see CRGBW.h) is the UV/blacklight channel. Full on or full
   // off, independent of whatever else china is currently showing.
   void setBlacklight( bool on ) {
      uint8_t level = on ? 255 : 0;
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) chinaLeds[ i ].u = level;
   }

   void clearRope() {
      memset( ropeLeds, 0, SIZEOF_NEO_LEDS );
      memset( ropeShowLeds, 0, SIZEOF_NEO_SHOW_LEDS );
   }

   // clears all the lights back to full black
   void clear() {
      clearMegabars();
      clearChinas();
      clearRope();
   }

#ifdef __EMSCRIPTEN__
   // debug/inspection readback only (see ropeShowLeds' own declaration
   // comment) -- real hardware never needs this, ropeShowLeds stays
   // private there.
   CRGB * getRopeShowLedsPtr() { return ropeShowLeds; }
#endif

   // flashes the perimeter rope LEDs white `count` times, as button-hold
   // feedback. each flash is 55ms, padded by 15ms of black before and after;
   // multiple flashes are separated by an additional 85ms. blocking, since
   // this is only ever a brief pause during a press the user is still holding.
   void flashRope( uint8_t count ) {
      for ( uint8_t i = 0; i < count; ++i ) {
         clearRope();
         show();
         delayPolling( 15 );
         for ( int j = 0; j < NUM_NEO_LEDS_ACTUAL; ++j ) ropeLeds[ j ].w = 255;
         show();
         delayPolling( 55 );
         clearRope();
         show();
         delayPolling( 15 );
         if ( i + 1 < count ) delayPolling( 85 );
      }
   }

   // renders the whole rig as solid red at the given brightness PERCENT
   // (0-100), for the global-brightness configuration preview. rope leds are
   // gamma-corrected automatically inside show(); megabar/china deliberately
   // are NOT gamma-corrected here (or anywhere else in this class) -- gamma
   // correction is a nonlinear curve, and applying it after scaling would
   // throw off the linear brightness math these previews rely on (see
   // applyBrightnessCeiling(), which is equally gamma-free for the same
   // reason). Only FlameShow gamma-corrects its own megabar output; that's a
   // pre-existing inconsistency, not something to match here.
   void showSolidRed( float percent ) {
      uint8_t linearBrightness = percentToRaw( percent );
      // BUGFIX: .w was never cleared here, unlike chinaLeds[i].w below --
      // currLightShow->update() keeps running "invisibly" underneath this
      // preview (see its own call site's comment in CarpetLightLogic.cpp),
      // so leftover .w wasn't a static residual value, it was the ACTIVE
      // show's own live, per-pixel-varying white channel leaking straight
      // through into what should be a flat solid-red preview. Folded
      // through the RGBW->3-into-4 packing scheme (which interleaves W
      // bytes across neighboring pixels' output slots), a rapidly-changing,
      // spatially-varying W produces exactly the "random full-saturation
      // hues, flashing" corruption seen on the real car.
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         ropeLeds[ i ] = CRGB::Black;
         ropeLeds[ i ].r = linearBrightness;
         ropeLeds[ i ].w = 0;
      }
      LedUtil::fill( megabarLeds, CRGB( linearBrightness, 0, 0 ), NUM_MEGABAR_LEDS );
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         chinaLeds[ i ] = CRGB( linearBrightness, 0, 0 );
         chinaLeds[ i ].w = 0;
      }
   }

   // preview for the headlight-brightness setting: rope and china off, all
   // other megabars shown at full max (a fixed, unambiguous reference --
   // previously the already-committed global brightness, which made
   // comparison meaningless whenever global itself was dim), and the
   // headlight shown at global brightness scaled by liveHeadlightPercent
   // (the not-yet-committed value being adjusted) -- so the two can be
   // visually compared side by side.
   void showHeadlightPreview( float liveHeadlightPercent ) {
      clearRope();
      clearChinas();
      uint8_t globalRaw = percentToRaw( globalBrightness_ );
      uint8_t headlightRaw = scale8( globalRaw, percentToRaw( liveHeadlightPercent ) );
      LedUtil::fill( megabarLeds, CRGB( 255, 0, 0 ), NUM_MEGABAR_LEDS );
      megabarLeds[ HEADLIGHT_INDEX ] = CRGB( headlightRaw, 0, 0 );
   }

   // preview for the china-brightness setting: rope off, megabars shown at
   // their normal effective brightness (respecting the committed headlight
   // setting) as a fixed reference, and china shown at global brightness
   // scaled by liveChinaPercent (the not-yet-committed value being adjusted)
   // -- so the two can be visually compared side by side.
   void showChinaPreview( float liveChinaPercent ) {
      clearRope();
      uint8_t globalRaw = percentToRaw( globalBrightness_ );
      uint8_t headlightRaw = scale8( globalRaw, percentToRaw( headlightBrightness_ ) );
      uint8_t chinaRaw = scale8( globalRaw, percentToRaw( liveChinaPercent ) );
      LedUtil::fill( megabarLeds, CRGB( globalRaw, 0, 0 ), NUM_MEGABAR_LEDS );
      megabarLeds[ HEADLIGHT_INDEX ] = CRGB( headlightRaw, 0, 0 );
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         chinaLeds[ i ] = CRGB( chinaRaw, 0, 0 );
         chinaLeds[ i ].w = 0;
      }
   }

   // lights a 10-LED window on one side strip, sliding from rearCornerIdx
   // (0%) to frontCornerIdx (100%) as percent varies, always 10 LEDs wide.
   // each lit LED's color reflects its OWN position along the whole side
   // (red at the rear corner, green at the front corner), not the window's
   // position, so the lit window's hue shifts as it slides. segmentLen is
   // derived from the two corner indices passed in, not hardcoded to the
   // full 352-LED channel -- callers are expected to pass the true-corner
   // constants (FRONT_RIGHT/REAR_RIGHT, REAR_LEFT/FRONT_LEFT), which land 33
   // LEDs in from each end of the physical channel (SIZEOF_LARGE_NEO_CORNER),
   // so the window doesn't wrap into the front/rear edge LEDs at either end.
   void renderSideIndicator( int rearCornerIdx, int frontCornerIdx, float percent, CRGB color ) {
      const int segmentLen = abs( frontCornerIdx - rearCornerIdx ) + 1;
      static const int windowWidth = 10;
      int direction = ( frontCornerIdx > rearCornerIdx ) ? 1 : -1;
      float t = constrain( percent, 0.0f, 100.0f ) / 100.0f;
      int maxOffset = segmentLen - windowWidth;
      int windowOffset = (int)( t * maxOffset + 0.5f );
      for ( int k = 0; k < windowWidth; ++k ) {
         int localOffset = windowOffset + k;
         int idx = rearCornerIdx + direction * localOffset;
         ropeLeds[ idx ] = color;
         ropeLeds[ idx ].w = 0;
      }
   }

   // Draws the noise-floor (blue) and peak-threshold (red) windows on one
   // side strip together, instead of as two independent renderSideIndicator()
   // calls -- the two used to be drawn with no knowledge of each other, so
   // whenever they were close enough for their 10-LED windows to overlap
   // (up to fully coincident when set equal), the red one (drawn second)
   // silently painted over some or all of the blue one, making it
   // invisible. Since noise floor is cross-clamped elsewhere to never
   // exceed peak threshold, the windows can only ever collide from below --
   // nudge them apart symmetrically (clamped to the strip's own ends)
   // whenever they'd overlap, so both stay fully visible side by side
   // instead of one hiding the other.
   void renderSideIndicatorPair( int rearCornerIdx, int frontCornerIdx, float noiseFloorPercent, float peakThresholdPercent ) {
      const int segmentLen = abs( frontCornerIdx - rearCornerIdx ) + 1;
      static const int windowWidth = 10;
      const int maxOffset = segmentLen - windowWidth;
      int noiseOffset = (int)( constrain( noiseFloorPercent, 0.0f, 100.0f ) / 100.0f * maxOffset + 0.5f );
      int peakOffset = (int)( constrain( peakThresholdPercent, 0.0f, 100.0f ) / 100.0f * maxOffset + 0.5f );
      int gap = peakOffset - noiseOffset;
      if ( gap < windowWidth ) {
         int deficit = windowWidth - gap;
         int pushDown = deficit / 2, pushUp = deficit - pushDown;
         noiseOffset = max( 0, noiseOffset - pushDown );
         peakOffset = min( maxOffset, peakOffset + pushUp );
         if ( peakOffset - noiseOffset < windowWidth ) { // one side hit its bound -- compensate on the other
            if ( noiseOffset == 0 ) peakOffset = min( maxOffset, windowWidth );
            else if ( peakOffset == maxOffset ) noiseOffset = max( 0, maxOffset - windowWidth );
         }
      }
      int direction = ( frontCornerIdx > rearCornerIdx ) ? 1 : -1;
      for ( int k = 0; k < windowWidth; ++k ) {
         ropeLeds[ rearCornerIdx + direction * ( noiseOffset + k ) ] = CRGB::Blue;
         ropeLeds[ rearCornerIdx + direction * ( noiseOffset + k ) ].w = 0;
      }
      for ( int k = 0; k < windowWidth; ++k ) {
         ropeLeds[ rearCornerIdx + direction * ( peakOffset + k ) ] = CRGB::Red;
         ropeLeds[ rearCornerIdx + direction * ( peakOffset + k ) ].w = 0;
      }
   }

   // audio configuration screen: a 3-band VU meter across the front rope
   // strip (ropeLeds[0..SIZEOF_SMALL_NEO-1], 156 LEDs), a position indicator
   // sliding along each side strip, and all megabars glowing blue
   // proportional to the current full-spectrum audio level.
   //
   // The front strip splits into 3 equal 52-LED segments in array order:
   // treble, mid, bass. Each segment shows a dim (50%) green->yellow->red
   // reference gradient across its length, with a bright white fill from the
   // segment's start up to the live level's position -- at max level the
   // whole segment goes solid white. trebleLevel/midLevel/bassLevel are
   // 0-255 (AudioBoard::getHigh/getMid/getLow range).
   //
   // isAutoGainSubsetting selects what the side strips show (no more rainbow
   // -- each indicator is now a flat color, by request):
   //  - false (noise floor / peak threshold, merged into one subsetting):
   //    BOTH values are shown at once, so you can see where the other one
   //    sits while dialing in this one -- noise floor as a blue 10-LED
   //    window, peak threshold as a red 10-LED window, each at its own
   //    percent position (0=rear corner, 100=front corner, see
   //    renderSideIndicator()).
   //  - true (auto-gain): a single white 10-LED window snapped to the rear
   //    corner (AGCoff), middle (AGCband), or front corner (AGCfull) --
   //    a 3-position indicator, one snap point per AGCMode.
   //
   // fullSpectrumLevel (0-255, AudioBoard::getBandNormalPercent(BandFull)
   // scaled up from percent -- already silence-aware and auto-gain-scaled
   // if enabled, via the unified audio pipeline) sets all megabars' blue
   // brightness uniformly.
   void showAudioMeter( uint8_t trebleLevel, uint8_t midLevel, uint8_t bassLevel,
                        bool isAutoGainSubsetting, float noiseFloorPercentToShow, float peakThresholdPercentToShow,
                        uint8_t agcModeToShow, uint8_t fullSpectrumLevel ) {
      clearRope();
      // very simple per-bin floodlight confirmation, one raw hardware bin per
      // china fixture (blue, same convention as the megabar glow below) --
      // not meant to look good, just to let you visually confirm each of the
      // 7 raw frequency bins is actually reaching a floodlight. Only 7 of the
      // 8 china fixtures are used, one per bin; the 8th stays off. Brightness
      // tracks the bin's level directly, except a fixture flashes solid white
      // for PEAK_FLASH_MS the instant its bin rises above the same
      // AudioBoard::PEAK_THRESHOLD EqualizerShow's bass strobe uses -- an
      // edge trigger (like EqualizerShow's isHit), not sustained, so it's a
      // single quick flash per crossing rather than staying white throughout
      // a loud passage.
      static const uint8_t PEAK_FLASH_MS = 40;
      static uint8_t lastBinLevel_[ 7 ] = { 0, 0, 0, 0, 0, 0, 0 };
      static Timer binFlashTimer_[ 7 ];
      uint8_t peakThresholdRaw = AudioBoard::getPeakThresholdRaw();
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         if ( i < 7 ) {
            uint8_t level = scale( AudioBoard::Frequencies_Mono[ i ] );
            bool rising = level > peakThresholdRaw && lastBinLevel_[ i ] <= peakThresholdRaw;
            lastBinLevel_[ i ] = level;
            if ( rising ) binFlashTimer_[ i ].reset();
            bool flashing = binFlashTimer_[ i ].elapsed() < PEAK_FLASH_MS;
            chinaLeds[ i ] = flashing ? CRGB::White : CRGB( 0, 0, level );
            chinaLeds[ i ].w = flashing ? 255 : 0;
         } else {
            chinaLeds[ i ] = CRGB::Black;
            chinaLeds[ i ].w = 0;
         }
      }
      // Inset from BOTH outer ends of the 156-LED front string, so the
      // meter no longer uses the whole thing -- per request, the outer
      // ends (nearest each front corner) are hidden under the wings when
      // they're up, so a meter using the full width was partly invisible
      // then. PLACEHOLDER: 20 LEDs/side is a guess (~1.5ft on the small
      // channel's ~13 LEDs/ft), not a measured value -- adjust to whatever
      // the wings actually cover on the real car.
      static const int wingInset = 20;
      static const int usableLen = SIZEOF_SMALL_NEO - 2 * wingInset;
      static const int segmentLen = usableLen / 3;
      for ( int i = 0; i < wingInset; ++i ) { ropeLeds[ i ] = CRGB::Black; ropeLeds[ i ].w = 0; }
      for ( int i = SIZEOF_SMALL_NEO - wingInset; i < SIZEOF_SMALL_NEO; ++i ) { ropeLeds[ i ] = CRGB::Black; ropeLeds[ i ].w = 0; }
      uint8_t levels[ 3 ] = { trebleLevel, midLevel, bassLevel };
      for ( int seg = 0; seg < 3; ++seg ) {
         int filledCount = ( (int)levels[ seg ] * segmentLen + 127 ) / 255; // round
         for ( int p = 0; p < segmentLen; ++p ) {
            int i = wingInset + seg * segmentLen + p;
            if ( p < filledCount ) {
               ropeLeds[ i ] = CRGB::Black;
               ropeLeds[ i ].w = 255;
            } else {
               uint8_t hue = (uint8_t)( 96 - ( 96 * p ) / ( segmentLen - 1 ) ); // green(96) -> red(0)
               ropeLeds[ i ] = CHSV( hue, 255, 77 ); // 77 = 128 * 0.6 (dimmed 40% further per request)
               ropeLeds[ i ].w = 0;
            }
         }
      }
      // while dialing in noise floor or peak threshold, overlay one blue
      // pixel (noise floor) and one red pixel (peak threshold) on each of
      // the 3 segments above, at the position along that segment's own
      // 0-255 range -- live, since AudioBoard's live-adjusted value is
      // already applied by the caller before this is drawn. Not shown while
      // adjusting auto-gain, since neither value is in play there.
      if ( !isAutoGainSubsetting ) {
         uint8_t noiseFloorRaw = AudioBoard::getNoiseFloorRaw();
         uint8_t peakThresholdRaw = AudioBoard::getPeakThresholdRaw();
         int noiseFloorPos = ( (int)noiseFloorRaw * ( segmentLen - 1 ) + 127 ) / 255;
         int peakThresholdPos = ( (int)peakThresholdRaw * ( segmentLen - 1 ) + 127 ) / 255;
         // cheat so both dots stay visible once they've run into each other
         // -- noise floor is cross-clamped elsewhere to never exceed peak
         // threshold, so they can only collide (become equal), never cross.
         // Nudge the blue dot back by 1 pixel so red shows up right next to
         // it instead of silently overwriting it (red is drawn second,
         // below) at the exact position adjustment is now disallowed past.
         if ( noiseFloorPos == peakThresholdPos ) {
            if ( noiseFloorPos > 0 ) --noiseFloorPos; else ++peakThresholdPos;
         }
         for ( int seg = 0; seg < 3; ++seg ) {
            ropeLeds[ wingInset + seg * segmentLen + noiseFloorPos ] = CRGB::Blue;
            ropeLeds[ wingInset + seg * segmentLen + noiseFloorPos ].w = 0;
            ropeLeds[ wingInset + seg * segmentLen + peakThresholdPos ] = CRGB::Red;
            ropeLeds[ wingInset + seg * segmentLen + peakThresholdPos ].w = 0;
         }
      }

      // side strips, true corner to true corner (see renderSideIndicator's
      // comment) -- this stops 33 LEDs short of each end of the 352-LED
      // channel, so it doesn't wrap into the front/rear edges near the corners
      if ( isAutoGainSubsetting ) {
         float snapPercent = (float)agcModeToShow / (float)( NumAGCModes - 1 ) * 100.0f; // 0%=AGCoff, 50%=AGCband, 100%=AGCfull
         renderSideIndicator( REAR_RIGHT, FRONT_RIGHT, snapPercent, CRGB::White );
         renderSideIndicator( REAR_LEFT, FRONT_LEFT, snapPercent, CRGB::White );
      } else {
         renderSideIndicatorPair( REAR_RIGHT, FRONT_RIGHT, noiseFloorPercentToShow, peakThresholdPercentToShow );
         renderSideIndicatorPair( REAR_LEFT, FRONT_LEFT, noiseFloorPercentToShow, peakThresholdPercentToShow );
      }

      LedUtil::fill( megabarLeds, CRGB( 0, 0, fullSpectrumLevel ), NUM_MEGABAR_LEDS );
   }

   // shared by showHitDecayMeter()'s VU meters below: fills [0,barLen) of
   // ropeLeds starting at ropeOffset with barColor up to litPos, black past
   // it, then always stamps a red dot at peakPos (even if that pixel would
   // otherwise be unlit) and a blue dot at decayDotPos -- the current
   // decay-rate SETTING's position along this meter's own length (0=0ms,
   // barLen-1=the top of the setting's range), entirely unrelated to any
   // audio level. If any of the 3 marker positions coincide, nudge apart by
   // 1 pixel (same collision-avoidance trick as showAudioMeter's dots above)
   // so the later-drawn color never silently hides an earlier one.
   void renderVuBar( int ropeOffset, int barLen, int litPos, CRGB barColor, int peakPos, int decayDotPos ) {
      for ( int p = 0; p < barLen; ++p ) {
         ropeLeds[ ropeOffset + p ] = ( p < litPos ) ? barColor : CRGB::Black;
         ropeLeds[ ropeOffset + p ].w = 0;
      }
      if ( decayDotPos == peakPos ) {
         if ( decayDotPos > 0 ) --decayDotPos; else ++peakPos;
      }
      ropeLeds[ ropeOffset + peakPos ] = CRGB::Red;
      ropeLeds[ ropeOffset + peakPos ].w = 0;
      ropeLeds[ ropeOffset + decayDotPos ] = CRGB::Blue;
      ropeLeds[ ropeOffset + decayDotPos ].w = 0;
   }

   // Audio config screen's decay-rate subsetting (subsetting 3): visualizes
   // hit-tracking (see AudioBoard.h's raw/normal/hit system) against either a
   // simulated or live signal, so the decay rate can be tuned by eye (see
   // CarpetLightLogic.cpp's sim/live toggle, bound to a short press while
   // this subsetting is showing). China stays off in both modes, per
   // request. decayMs/decayRangeMs place a blue dot on every VU meter marking
   // where the current decay-rate SETTING sits within its own adjustable
   // range (0 to decayRangeMs) -- that dot's position depends only on the
   // setting, never on any audio level.
   //
   //  - simMode: every light tracks the single simulated hit value in green,
   //    except the front strip, which shows ONE big VU meter across all 156
   //    LEDs: white up to the current normalized level, switching to red for
   //    whatever portion of that lit range is above the peak threshold, plus
   //    a red dot always marking the threshold position itself. If hit is
   //    still decaying down from a level higher than the current normalized
   //    level, the gap between them also shows red (the decay's lingering
   //    tail past where the live signal has already dropped to).
   //  - live mode: the front strip instead shows 3 separate VU meters (one
   //    per band, in the same treble/mid/bass array order as
   //    showAudioMeter()), each driven by that band's own hit value, colored
   //    blue/green/red respectively (peak-threshold dots are always red
   //    regardless of a bar's own color). Megabars split into a repeating
   //    3-position pattern -- treble(blue), mid(green), bass(red) -- around
   //    all 12, each showing that band's own hit value.
   void showHitDecayMeter( bool simMode, float decayMs, float decayRangeMs ) {
      clearChinas();
      // decayRangeMs can legitimately be 0 (e.g. hit-prediction's range is
      // capped to the live foresight setting, which defaults to 0) -- guard
      // the divide rather than let a 0/0 NaN propagate into an (int) cast,
      // which is undefined behavior and could corrupt an array index below.
      float rangeFrac = ( decayRangeMs > 0.0f ) ? constrain( decayMs / decayRangeMs, 0.0f, 1.0f ) : 0.0f;
      int decayDotPos156 = (int)( rangeFrac * ( SIZEOF_SMALL_NEO - 1 ) + 0.5f );
      int peakPos156 = ( (int)AudioBoard::getPeakThresholdRaw() * ( SIZEOF_SMALL_NEO - 1 ) + 127 ) / 255;

      if ( simMode ) {
         uint8_t hitRaw = (uint8_t)( (uint16_t)AudioBoard::getBandHitPercent( BandBass ) * 255 / 100 );
         uint8_t normalRaw = (uint8_t)( (uint16_t)AudioBoard::getBandNormalPercent( BandBass ) * 255 / 100 );
         int normalPos = ( (int)normalRaw * ( SIZEOF_SMALL_NEO - 1 ) + 127 ) / 255;
         int hitPos = ( (int)hitRaw * ( SIZEOF_SMALL_NEO - 1 ) + 127 ) / 255;

         LedUtil::fill( megabarLeds, CRGB( 0, hitRaw, 0 ), NUM_MEGABAR_LEDS );
         for ( int i = SIZEOF_SMALL_NEO; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
            ropeLeds[ i ] = CRGB( 0, hitRaw, 0 );
            ropeLeds[ i ].w = 0;
         }
         // front strip: white up to normalPos, switching to red past peakPos
         // (i.e. red for whatever's currently above threshold), then red
         // again from normalPos up to hitPos if hit is still lagging above
         // the live signal (decay not yet caught up)
         for ( int p = 0; p < SIZEOF_SMALL_NEO; ++p ) {
            CRGB c = CRGB::Black;
            if ( p < normalPos ) c = ( p < peakPos156 ) ? CRGB::White : CRGB::Red;
            else if ( p < hitPos ) c = CRGB::Red;
            ropeLeds[ p ] = c;
            ropeLeds[ p ].w = 0;
         }
         int peakDot = peakPos156, decayDot = decayDotPos156;
         if ( decayDot == peakDot ) { if ( decayDot > 0 ) --decayDot; else ++peakDot; }
         ropeLeds[ peakDot ] = CRGB::Red; ropeLeds[ peakDot ].w = 0;
         ropeLeds[ decayDot ] = CRGB::Blue; ropeLeds[ decayDot ].w = 0;
      } else {
         static const int segLen = SIZEOF_SMALL_NEO / 3; // 52
         static const AudioBand segBand[ 3 ] = { BandTreble, BandMid, BandBass };
         static const CRGB segColor[ 3 ] = { CRGB::Blue, CRGB::Green, CRGB::Red };
         int peakPosSeg = ( (int)AudioBoard::getPeakThresholdRaw() * ( segLen - 1 ) + 127 ) / 255;
         int decayDotPosSeg = (int)( rangeFrac * ( segLen - 1 ) + 0.5f );
         for ( int seg = 0; seg < 3; ++seg ) {
            uint8_t hitRaw = (uint8_t)( (uint16_t)AudioBoard::getBandHitPercent( segBand[ seg ] ) * 255 / 100 );
            int hitPos = ( (int)hitRaw * ( segLen - 1 ) + 127 ) / 255;
            renderVuBar( seg * segLen, segLen, hitPos, segColor[ seg ], peakPosSeg, decayDotPosSeg );
         }
         // megabars: repeating treble(blue)/mid(green)/bass(red) triple,
         // each showing that band's own hit value
         for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
            int which = m % 3;
            uint8_t hitRaw = (uint8_t)( (uint16_t)AudioBoard::getBandHitPercent( segBand[ which ] ) * 255 / 100 );
            CRGB c = segColor[ which ];
            c.nscale8( hitRaw );
            megabarLeds[ m ] = c;
         }
      }
   }

   // Audio config screen's foresight-adjustment subsetting: dim blue ambient
   // wash across the whole rope, just to indicate this screen is active (not
   // audio-reactive itself), with a 2-pixel white marker overlaid on the
   // front strip showing the current foresight setting's position within
   // its 0-700ms range (0ms at one end, 700ms at the other) -- same "meter
   // position reflects a SETTING, not an audio level" idea as the decay-rate
   // screen's blue dot. China stays off, same as the decay-rate screen.
   //
   // Megabars ARE live audio-reactive here, in the same repeating 3-position
   // pattern as the decay-rate screen's live mode -- treble(blue)/mid(green)/
   // bass(red) every third megabar around all 12, each showing that band's
   // own hit value -- so you can watch the flash and tune the pot until it
   // lands in sync with the beat you hear (hit is already delayed by
   // whatever foresightMs is currently being adjusted, since AudioBoard
   // applies that delay upstream of every getter). Each megabar has a 15%
   // brightness floor in its own band color regardless of hit, so which
   // fixture maps to which frequency stays visible even at rest between hits.
   void showForesightAdjust( float foresightMs, float foresightRangeMs ) {
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         ropeLeds[ i ] = CRGB( 0, 0, 40 ); // dim blue ambient wash
         ropeLeds[ i ].w = 0;
      }
      int markerPos = (int)( constrain( foresightMs / foresightRangeMs, 0.0f, 1.0f ) * ( SIZEOF_SMALL_NEO - 2 ) + 0.5f );
      ropeLeds[ markerPos ] = CRGB::White; ropeLeds[ markerPos ].w = 0;
      ropeLeds[ markerPos + 1 ] = CRGB::White; ropeLeds[ markerPos + 1 ].w = 0;

      clearChinas();

      static const AudioBand segBand[ 3 ] = { BandTreble, BandMid, BandBass };
      static const CRGB segColor[ 3 ] = { CRGB::Blue, CRGB::Green, CRGB::Red };
      static const uint8_t MIN_BRIGHTNESS = 38; // 15% of 255
      for ( int m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         int which = m % 3;
         uint8_t hitRaw = (uint8_t)( (uint16_t)AudioBoard::getBandHitPercent( segBand[ which ] ) * 255 / 100 );
         CRGB c = segColor[ which ];
         c.nscale8( max( hitRaw, MIN_BRIGHTNESS ) );
         megabarLeds[ m ] = c;
      }
   }

   // Global sound-reactivity toggle screen (SubAudioReactivityEnable): the
   // chinas play a fake, steady series of "hits" as direct feedback for
   // whichever state is currently live -- per request, so disabling it
   // shows its own consequence directly (chinas doing nothing) rather
   // than just a number/label. Rope/megabars stay off -- china-only, per
   // request. Not audio-driven at all (deliberately fake/simulated, same
   // idea as a config screen's other synthetic test signals) so it reads
   // identically regardless of whatever's actually playing right now.
   void showSoundReactivityToggle( bool liveEnabled ) {
      clearRope();
      clearMegabars();
      static const uint32_t PERIOD_MS = 600, ON_MS = 120;
      bool lit = liveEnabled && ( millis() % PERIOD_MS ) < ON_MS;
      for ( int c = 0; c < NUM_CHINA_LEDS; ++c ) {
         chinaLeds[ c ] = lit ? CRGB::White : CRGB::Black;
         chinaLeds[ c ].w = lit ? 255 : 0;
      }
   }

   // Auto-peak mode screen (SubAudioAutoPeak): 3-state now (AutoPeakOff/
   // AutoPeakFull/AutoPeakBin, see AudioBoard.h's AutoPeakMode), so the
   // front edge's own neos (indices [0, SIZEOF_SMALL_NEO), the small-neo
   // strip) split into 3 equal thirds instead of the old on/off halves --
   // same left-to-right-as-mode-value-increases convention as before, one
   // third lit per mode. Rest of rope/megabars/china stay dark so the
   // split reads cleanly.
   void showAutoPeakToggle( uint8_t liveMode ) {
      clearRope();
      clearMegabars();
      clearChinas();
      static const int segLen = SIZEOF_SMALL_NEO / 3; // 52
      int litStart = liveMode * segLen;
      int litEnd = ( liveMode >= 2 ) ? SIZEOF_SMALL_NEO : litStart + segLen; // last third absorbs any rounding remainder
      for ( int i = litStart; i < litEnd; ++i ) {
         ropeLeds[ i ] = CRGB::White;
         ropeLeds[ i ].w = 0;
      }
   }

   // power-saving A/B test: left half of the rig (rope's left side + left
   // half of both front/rear edges, china[2..5]) shows the given HSV color
   // rendered straight to RGB (w=0); the right half (rope's right side +
   // right half of both front/rear edges, china[0,1,6,7]) shows the same
   // color translated to RGBW by moving the shared min(r,g,b) out of the
   // color channels and into the white channel -- same hue/brightness, less
   // color-LED power. Megabars are off; unlike the other config previews
   // this one deliberately ignores the committed global/headlight/china
   // brightness ceiling (hue/sat/brightness here are already a full manual
   // color spec, not a dimming level) -- see README.md.
   void showPowerTest( uint8_t hue, uint8_t sat, uint8_t val ) {
      clearMegabars();

      CRGB straightClr = CHSV( hue, sat, val );
      uint8_t minCh = min( straightClr.r, min( straightClr.g, straightClr.b ) );
      CRGBW savingClr;
      savingClr.r = straightClr.r - minCh;
      savingClr.g = straightClr.g - minCh;
      savingClr.b = straightClr.b - minCh;
      savingClr.w = minCh;

      // BUGFIX: this had the same front/rear-edge-direction bug LighthouseShow's
      // ropeAngleDeg() had (see its README entry) -- i < FRONT / i >= REAR
      // used to be swapped, so this was reporting the true right half as
      // "left" and vice versa, and the ~30 LEDs of the right-side strip's
      // corner-wrap portion nearest the front edge visibly mismatched the
      // (mislabeled) front-edge LEDs right next to them. Now: right half is
      // the single contiguous span [FRONT,REAR) -- front edge's higher-
      // index/right portion, all of the right side, rear edge's lower-
      // index/right portion (matching the now-corrected direction: front
      // edge runs left-to-right as index increases, rear edge right-to-left)
      // -- and left half is everything outside that span, forming a
      // sideways U to the carpet's left-to-right center line.
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         bool leftHalf = ( i < FRONT ) || ( i >= REAR );
         if ( leftHalf ) {
            ropeLeds[ i ] = straightClr;
            ropeLeds[ i ].w = 0;
         } else {
            ropeLeds[ i ] = savingClr;
         }
      }

      // china[2,3] = front-left, china[4,5] = rear-left; china[0,1] =
      // front-right, china[6,7] = rear-right -- see README.md, "China lights"
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         bool leftHalf = ( i >= 2 && i < 6 );
         if ( leftHalf ) {
            chinaLeds[ i ] = straightClr;
            chinaLeds[ i ].w = 0;
         } else {
            chinaLeds[ i ] = savingClr;
         }
      }
   }

   // scales every light array in-place per the configured global/headlight/
   // china brightness percentages (linear domain), capping all light shows'
   // output. call once per frame, after a show writes its output and before
   // show() pushes it to hardware.
   void applyBrightnessCeiling() {
      uint8_t globalRaw = percentToRaw( globalBrightness_ );
      uint8_t headlightRaw = scale8( globalRaw, percentToRaw( headlightBrightness_ ) );
      uint8_t chinaRaw = scale8( globalRaw, percentToRaw( chinaBrightness_ ) );

      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         ropeLeds[ i ].r = scale8( ropeLeds[ i ].r, globalRaw );
         ropeLeds[ i ].g = scale8( ropeLeds[ i ].g, globalRaw );
         ropeLeds[ i ].b = scale8( ropeLeds[ i ].b, globalRaw );
         ropeLeds[ i ].w = scale8( ropeLeds[ i ].w, globalRaw );
      }
      for ( int i = 0; i < NUM_MEGABAR_LEDS; ++i ) {
         uint8_t raw = ( i == HEADLIGHT_INDEX ) ? headlightRaw : globalRaw;
         megabarLeds[ i ].r = scale8( megabarLeds[ i ].r, raw );
         megabarLeds[ i ].g = scale8( megabarLeds[ i ].g, raw );
         megabarLeds[ i ].b = scale8( megabarLeds[ i ].b, raw );
      }
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         chinaLeds[ i ].r = scale8( chinaLeds[ i ].r, chinaRaw );
         chinaLeds[ i ].g = scale8( chinaLeds[ i ].g, chinaRaw );
         chinaLeds[ i ].b = scale8( chinaLeds[ i ].b, chinaRaw );
         chinaLeds[ i ].w = scale8( chinaLeds[ i ].w, chinaRaw );
      }
   }

   void error() {
      while( true ) {
         static CRGB clr = CRGB::Black;
         static CRGBWAU chinaClr = clr;
         chinaClr.a = 255;
         LedUtil::fill( ropeLeds, CRGB::Red, NUM_NEO_LEDS_ACTUAL );
         LedUtil::fill( megabarLeds, CRGB::Yellow, NUM_DMX_LEDS );
         LedUtil::fill( chinaLeds, chinaClr, NUM_CHINA_LEDS );
         FastLED.delay( 1000 );
         LedUtil::fill( ropeLeds, clr, NUM_NEO_LEDS_ACTUAL );
         LedUtil::fill( megabarLeds, clr, NUM_DMX_LEDS );
         LedUtil::fill( chinaLeds, clr, NUM_CHINA_LEDS );
         FastLED.delay( 1000 );
      }
   }
};

// singleton representing the one-and-only Flying Magic Carpet (TM)
MagicCarpet * theMagicCarpet() {
   static MagicCarpet carpet;
   return &carpet;
}

#endif
