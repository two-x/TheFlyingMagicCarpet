/* SpeedStripesShow.h
 *
 *    Speed-reactive show driven by CANTroller2's vehicle speed telemetry
 *    (see SpeedLink.h). Only the two long side rope strips run front-to-back
 *    along the car's length (see README.md, "Perimeter rope lights") -- the
 *    front/back edges and megabars/china don't, so this show is confined to
 *    those two strips and blanks everything else, same approach as
 *    MagicCarpet::showAudioMeter().
 *
 *    Each side strip (352 LEDs, full car length) is divided into 4 alternating
 *    lit/dark bands of 88 LEDs each -- a quarter of the car's length wide, per
 *    the original request. The band pattern continuously scrolls from front
 *    toward back at a rate proportional to the live speed reading, giving a
 *    sense of forward motion (like streaks flowing backward past a moving
 *    vehicle); at a standstill (or with no fresh telemetry) the bands sit
 *    still. LEDS_PER_MPH_PER_SEC is a to-taste scroll-rate constant, not a
 *    value derived from anything physical.
 */

#ifndef __SPEED_STRIPES_SHOW_H
#define __SPEED_STRIPES_SHOW_H

#include "LightShow.h"
#include "SpeedLink.h"

class SpeedStripesShow : public LightShow {
 private:
   static const int stripeWidth_ = SIZEOF_LARGE_NEO / 4; // 88, a quarter of the car's length
   float scrollOffset_ = 0.0f; // LEDs, grows over time as the pattern scrolls front->back
   uint32_t lastUpdateMillis_ = 0;

   // lights one side strip's 352 LEDs, walking from the back corner to the
   // front corner (same back->front traversal convention as
   // MagicCarpet::renderSideIndicator()), banding by (localOffset + scroll).
   void renderSide( int backCornerIdx, int frontCornerIdx, const CRGB & clr ) {
      int direction = ( frontCornerIdx > backCornerIdx ) ? 1 : -1;
      for ( int localOffset = 0; localOffset < SIZEOF_LARGE_NEO; ++localOffset ) {
         int idx = backCornerIdx + direction * localOffset;
         int band = (int)( ( localOffset + scrollOffset_ ) / stripeWidth_ );
         bool lit = ( band % 2 ) == 0;
         carpet->ropeLeds[ idx ] = lit ? clr : CRGB::Black;
         carpet->ropeLeds[ idx ].w = 0;
      }
   }

 public:
   SpeedStripesShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0 )
      : LightShow( carpetArg ) {}

   void start() {
      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();
      scrollOffset_ = 0.0f;
      lastUpdateMillis_ = 0;
   }

   void update( uint32_t time ) {
      if ( lastUpdateMillis_ == 0 ) lastUpdateMillis_ = time;
      float dtSec = (float)( time - lastUpdateMillis_ ) / 1000.0f;
      lastUpdateMillis_ = time;

      static const float LEDS_PER_MPH_PER_SEC = 4.0f;
      float speedMph = SpeedLink::isFresh() ? SpeedLink::getSpeedMph() : 0.0f;
      scrollOffset_ += speedMph * LEDS_PER_MPH_PER_SEC * dtSec;
      while ( scrollOffset_ >= ( stripeWidth_ * 2 ) ) scrollOffset_ -= ( stripeWidth_ * 2 ); // keep it bounded

      // pot picks the stripe color, same convention as NightriderShow variation 0
      uint8_t hue = carpet->pot->read() / 4;
      CRGB clr = CHSV( hue, 255, 255 );

      carpet->clearMegabars();
      carpet->clearChinas();
      for ( int i = 0; i < SIZEOF_SMALL_NEO; ++i ) carpet->ropeLeds[ i ] = CRGB::Black; // front edge off
      for ( int i = SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; ++i ) {
         carpet->ropeLeds[ i ] = CRGB::Black; // back edge off
      }

      // right side: back corner near SIZEOF_SMALL_NEO+SIZEOF_LARGE_NEO, front corner near SIZEOF_SMALL_NEO
      renderSide( SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO - 1, SIZEOF_SMALL_NEO, clr );
      // left side: back corner near SIZEOF_SMALL_NEO*2+SIZEOF_LARGE_NEO, front corner at the far end of the array
      renderSide( SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO, NUM_NEO_LEDS_ACTUAL - 1, clr );
   }
};

#endif
