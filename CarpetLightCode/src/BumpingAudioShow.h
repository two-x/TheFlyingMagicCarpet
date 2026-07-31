/* EqualizerShow.h
 *
 * A basic version of the Nightrider light effect, which is just a simple chase that
 * starts in the corners and bounces off the middles. Lame, but it lights up the
 * carpet and doesn't rely on the sound.
 *
 * Variation 1: a dual VU meter along the two side rope runs -- bass grows from
 * the back corner toward the front (red), treble grows from the front corner
 * toward the back (blue), reusing this show's own red=bass/blue=treble
 * convention from variation 0's megabar tinting. Each meter's 100%-level reach
 * is 15% of the true-corner-to-center half-length past center (RIGHT/LEFT are
 * already the exact side midpoints -- see MagicCarpet.h's positional
 * constants), so maxed-out bass and treble simultaneously cross by about that
 * much right around the middle of the car, per request.
 *
 * Author: Anders Linn
 * Date: August 2017
 */

#include "LightShow.h"
#include <math.h>

const CRGB topC[] {
   CRGB( 0, 255, 127), // summer day
   CRGB( 178, 118, 50), // end summer day
   CRGB( 178, 102, 9), // deep dark night
   CRGB( 0, 255, 72), // desert afternoon
   CRGB( 43, 123, 93), //under the sea
   CRGB( 236, 84, 81), // beach party
   CRGB( 248, 182, 124)
};

const CRGB bottomC[] {
 CRGB( 0, 201, 100 ),
 CRGB( 254, 181, 97 ),
 CRGB( 0, 68, 101 ),
 CRGB( 76, 255, 127 ),
 CRGB( 50, 199, 143 ),
 CRGB( 182, 35, 99 ),
 CRGB( 248, 247, 5 ),
};

class EqualizerShow : public LightShow {
 private:
   static const uint8_t numVariations_ = 2;
   uint8_t variation_;

   // triple-strobe on bass hits (see updateStrobe()) -- nonvolatile, toggled
   // by a double press while this show is active (see CarpetLightLogic.cpp)
   bool strobeEnabled_;
   bool strobeActive_ = false;
   Timer strobeTimer_;
   int lastBassForTrigger_ = 0;

   // hit-suppression state: once a hit strobes, its level is remembered, and
   // any further hit that doesn't exceed it is suppressed as long as it's
   // within 3s of the last qualifying hit (strobed or suppressed) -- a gap of
   // 3s with no hits at all clears this, so the next hit always strobes
   // fresh, regardless of level. See updateStrobe().
   bool hadHit_ = false;     // Timer has no "unset" state of its own
   Timer lastHitTimer_;      // timeout armed to SILENCE_RESET_MS on the first hit
   int suppressionPeak_ = 0;

   // pot adjusts AudioBoard's shared peak threshold while this show (either
   // variation) is active -- soft takeover, same convention as config mode's
   // livePercentFor(): holds at the last-committed value until the pot
   // actually moves since this show became active, so it doesn't jump.
   // Reverted back to the committed value on leaving this show (see
   // CarpetLightLogic.cpp's show-change block), so casual live tweaking here
   // never overwrites what's saved to flash.
   static const uint16_t POT_TAKEOVER_THRESHOLD = (uint16_t)( 0.02f * MAX_VOLTAGE + 0.5f );
   uint16_t potEntryRaw_ = 0;
   bool potTakenOver_ = false;
   SettlePrinter thresholdPrinter_;

 public:
   EqualizerShow( MagicCarpet * carpetArg, uint8_t initialVariation = 0, bool initialStrobeEnabled = false )
      : LightShow( carpetArg ), variation_( initialVariation % numVariations_ ), strobeEnabled_( initialStrobeEnabled ) {}

   uint8_t variation() {
      return variation_;
   }

   const char * variationName() {
      return variation_ == 0 ? "chase" : "VU meter";
   }

   bool getStrobeEnabled() {
      return strobeEnabled_;
   }
   void setStrobeEnabled( bool enabled ) {
      strobeEnabled_ = enabled;
   }

   void start() {
      for ( int i = NEO3_OFFSET; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         carpet->ropeLeds[ i ] = CRGB::Black;
      }
      potEntryRaw_ = carpet->pot->read();
      potTakenOver_ = false;
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

      // pot -> live peak threshold, soft takeover (see member comment above)
      uint16_t potRaw = carpet->pot->read();
      if ( !potTakenOver_ ) {
         uint16_t diff = ( potRaw > potEntryRaw_ ) ? ( potRaw - potEntryRaw_ ) : ( potEntryRaw_ - potRaw );
         if ( diff >= POT_TAKEOVER_THRESHOLD ) potTakenOver_ = true;
      }
      if ( potTakenOver_ ) {
         AudioBoard::setPeakThresholdPercent( (float)potRaw / (float)MAX_VOLTAGE * 100.0f );
      }
      thresholdPrinter_.update( (int)( AudioBoard::getPeakThresholdPercent() + 0.5f ), "PkThresh:", "%" );

      if ( variation_ == 1 ) {
         updateVuMeter( time );
      } else {
      static const CRGBPalette256 clr( CRGB::Red, CRGB::Black );
      static uint32_t timestamp = time;
      static const uint32_t rate = 300; // move the lights every 200ms
      static const uint32_t littleRate = rate;
      static const uint32_t bigRate = ( littleRate * SIZEOF_LARGE_NEO ) / SIZEOF_SMALL_NEO;
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

      //const uint8_t val1 = carpet->pot->read() / 4;
      //const uint8_t val2 = ( val1 + 128 ) % 255;
      //const CRGB clr1 = CHSV( val1, 255, 255 );
      //const CRGB clr2 = CHSV( val2, 255, 255 );;
      float satFraction = currentSatFraction( time );
      CRGB clr1 = desaturate( CRGB(0,0,255), satFraction );
      CRGB clr2 = desaturate( CRGB(255,0,0), satFraction );
      // CRGB clr3 = CRGB(0,255,0);
      // Serial.println( "potval" );
      // Serial.println( val1 );
      //const CRGBW clr1 = topC[val1];
      //const CRGBW clr2 = bottomC[val1];
      // Serial.println( "CRGB1" );
      // Serial.println( clr1.r );
      // Serial.println( clr1.g );
      // Serial.println( clr1.b );
      // Serial.println( clr1.w );
      // Serial.println( "CRGB2" );
      // Serial.println( clr2.r );
      // Serial.println( clr2.g );
      // Serial.println( clr2.b );
      // Serial.println( clr2.w );
      // int diffIndex = scaleTo255( diff, rate, 0 );

      for ( int i = FRONT; i < FRONT_RIGHT; ++i ) {
         int i_adj = i - FRONT;
        int val = scaleTo255( abs(littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Red;
      }
      for ( int i = FRONT_RIGHT; i < RIGHT; ++i ) {
         int i_adj = i - FRONT_RIGHT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Green;
      }
      for ( int i = RIGHT; i < BACK_RIGHT; ++i ) {
         int i_adj = i - RIGHT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Blue;
      }
      for ( int i = BACK_RIGHT; i < BACK; ++i ) {
         int i_adj = i - BACK_RIGHT;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Yellow;
      }
      for ( int i = BACK; i < BACK_LEFT; ++i ) {
         int i_adj = i - BACK;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Orange;
      }
      for ( int i = BACK_LEFT; i < LEFT; ++i ) {
         int i_adj = i - BACK_LEFT;
         int val = scaleTo255( abs( bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Purple;
      }
      for ( int i = LEFT; i < FRONT_LEFT; ++i ) {
         int i_adj = i - LEFT;
         int val = scaleTo255( abs(bigPos - i_adj), SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Pink;
      }
      for ( int i = FRONT_LEFT; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         int i_adj = i - FRONT_LEFT;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Grey;
      }
      for ( int i = 0; i < FRONT; ++i ) {
         int i_adj = i + SIZEOF_LARGE_NEO_CORNER;
        int val = scaleTo255( abs( littlePos - i_adj), SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER, 0 );
         carpet->ropeLeds[ i ] = blend( clr1, clr2, val );
         // carpet->ropeLeds[i] = CRGB::Grey;
      }


      LedUtil::reverse( carpet->ropeLeds + FRONT, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
       LedUtil::reverse( carpet->ropeLeds + BACK, SIZEOF_SMALL_NEO_HALF + SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + RIGHT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );
      LedUtil::reverse( carpet->ropeLeds + LEFT, SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER );

      CRGB dmxclr;
      // int dmxval = AudioBoard::getLow();
      // Serial.println( dmxval );
      int lowval = AudioBoard::getLow();
      static int lastlow = lowval;
      // Serial.print("lowval: ");
      // Serial.println(lowval);
      // Serial.flush();
      if (lowval > AudioBoard::getPeakThresholdRaw() && lowval > lastlow) {
       lastlow = lowval;
      } else {
       lastlow = lastlow > 15 ? lastlow - 15 : 0;
      }
      // Serial.print("lastlow: ");
      // Serial.println(lastlow);
      // Serial.flush();
      dmxclr = blend( CRGB::Black, clr2, lastlow );
      carpet->megabarLeds[1] = dmxclr;
      carpet->megabarLeds[2] = dmxclr;
      carpet->megabarLeds[4] = dmxclr;
      carpet->megabarLeds[5] = dmxclr;
      carpet->megabarLeds[7] = dmxclr;
      carpet->megabarLeds[8] = dmxclr;
      carpet->megabarLeds[10] = dmxclr;
      carpet->megabarLeds[11] = dmxclr;


      int highval = AudioBoard::getHigh() > 150 ? AudioBoard::getHigh() : 0;
      // Serial.print("highval: ");
      // Serial.println(highval);
      /*static int lasthigh = highval;
      Serial.print("lasthigh: ");
      Serial.println(highval);
      Serial.flush();
      if (highval > 100 && highval > lasthigh) {
       lasthigh = highval;
      } else {
       lasthigh = lasthigh > 15 ? lasthigh - 15 : 0;
      }
      Serial.print("lasthigh: ");
      Serial.println(lasthigh);
      Serial.flush();*/
      // dmxclr = blend( dmxclr, clr1, highval );
      dmxclr = blend( CRGB::Black, clr1, highval );

      carpet->megabarLeds[0] = dmxclr;
      carpet->megabarLeds[3] = dmxclr;
      carpet->megabarLeds[6] = dmxclr;
      carpet->megabarLeds[9] = dmxclr;



      //CRGB dmxclr = blend( clr1, clr2, lowval);
      //dmxclr = blend( clr2, clr3, highval);
      //dmxclr = CRGB(lowval,highval,30);



      /*int midval = AudioBoard::getMid();
      int highval = AudioBoard::getHigh();
      static int maxlow = 0;
      static int minlow = 255;
      maxlow = max(maxlow, lowval);
      Serial.println("maxlow");
      Serial.println(maxlow);
      minlow = min(minlow, lowval);
      Serial.println("minlow");
      Serial.println(minlow);
      // lowval = lowval > 75 ? lowval - 75 : 0;
      // midval = midval > 75 ? midval - 75 : 0;
     //  highval = highval > 75 ? highval - 75 : 0;
      // CRGB dmxclr = (lowval > midval && lowval >= highval) ? CRGB::Red :(highval > midval) ? CRGB::Green : CRGB::Blue;
      float avg = lowval + midval + highval;
      int dmxred = (255 * ( ((float) lowval) / avg));
      int dmxblue = (255 * ( ((float) midval) / avg));
      int dmxgreen = (255 * ( ((float) highval) / avg));
      Serial.println("dmxred:");
      Serial.println(dmxred);
      Serial.println("dmxblue:");
      Serial.println(dmxblue);
      Serial.println("dmxgreen:");
      Serial.println(dmxgreen);
      //CRGB dmxclr(dmxred, 0, dmxgreen);
      Serial.println("low: ");
      Serial.println(lowval);
      Serial.println("mid");
      Serial.println(midval);
      Serial.println("high");
      Serial.println(highval);*/
      // CRGB dmxclr(lowval, midval, highval);
      //LedUtil::fill( carpet->megabarLeds, dmxclr, NUM_MEGABAR_LEDS );
      //LedUtil::fill( carpet->chinaLeds, dmxclr, NUM_CHINA_LEDS );

      // disabled: button now drives global show-cycling, not this show's white glow
      /*
      static uint8_t white = 0;
      if ( carpet->encoder->button.isDown() ) {
         if ( white < 255 ) {
            ++white;
         }
      } else {
         if ( white > 0 ) {
            --white;
         }
      }
      for ( int i = 0; i < NUM_NEO_LEDS_ACTUAL; ++i ) {
         carpet->ropeLeds[ i ].w = white;
      }
      */
      } // end variation 0

      updateStrobe( time );
   }

 private:
   // variation 1: dual VU meter. Bass (AudioBoard::getLow()) is based across
   // the WHOLE back of the car -- the back edge rope is always part of it,
   // not just the two back corners -- and grows forward from there along
   // both sides, red. Treble (getHigh()) mirrors this from the whole front
   // edge, growing backward along both sides, blue -- same red=bass/blue=
   // treble convention as variation 0's megabar tinting above. Both meters
   // use the same attack/decay peak-hold ballistics as variation 0's lowval/
   // lastlow (jump up fast past a threshold, fall back 15/frame otherwise),
   // just kept as separate state. Brightness of each meter (base edge and
   // side fill alike) scales directly with its current level -- a quiet
   // signal shows a dim meter, not just a short one.
   //
   // RIGHT/LEFT are already the exact midpoints of each side run (see
   // MagicCarpet.h's positional constants), and FRONT_RIGHT/BACK_RIGHT (resp.
   // FRONT_LEFT/BACK_LEFT) the true corners -- so halfLen_ below is exactly
   // the true-corner-to-center distance, reusing the same geometry as
   // variation 0's "big" segments (SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER).
   // Each meter's 100%-level reach is halfLen_ plus 15% of halfLen_, so maxed
   // bass and treble together cross by about 15% of half the car's length,
   // right around the middle -- per request.
   void updateVuMeter( uint32_t time ) {
      static int vuLastBass = 0;
      static int vuLastTreble = 0;

      int bassRaw = AudioBoard::getLow();
      if ( bassRaw > AudioBoard::getPeakThresholdRaw() && bassRaw > vuLastBass ) {
         vuLastBass = bassRaw;
      } else {
         vuLastBass = vuLastBass > 15 ? vuLastBass - 15 : 0;
      }
      int trebleRaw = AudioBoard::getHigh();
      if ( trebleRaw > AudioBoard::getPeakThresholdRaw() && trebleRaw > vuLastTreble ) {
         vuLastTreble = trebleRaw;
      } else {
         vuLastTreble = vuLastTreble > 15 ? vuLastTreble - 15 : 0;
      }

      static const float halfLen_ = SIZEOF_LARGE_NEO_HALF - SIZEOF_LARGE_NEO_CORNER; // true corner to center, 143
      static const float maxReach_ = halfLen_ * 1.15f; // 100%-level reach: 15% of halfLen_ past center
      float bassLitCount = ( vuLastBass / 255.0f ) * maxReach_;
      float trebleLitCount = ( vuLastTreble / 255.0f ) * maxReach_;

      // brightness follows level directly -- a quiet meter is a dim meter
      float satFraction = currentSatFraction( time );
      CRGB bassClr = desaturate( CRGB( 255, 0, 0 ), satFraction );   // red, per request
      CRGB trebleClr = desaturate( CRGB( 0, 0, 255 ), satFraction ); // blue, matching variation 0's high=blue convention
      bassClr.nscale8( (uint8_t)vuLastBass );
      trebleClr.nscale8( (uint8_t)vuLastTreble );

      carpet->clearRope();
      carpet->clearMegabars();
      carpet->clearChinas();

      // back edge: always part of the bass meter's base, full width
      for ( int i = SIZEOF_SMALL_NEO + SIZEOF_LARGE_NEO; i < SIZEOF_SMALL_NEO * 2 + SIZEOF_LARGE_NEO; ++i ) {
         carpet->ropeLeds[ i ] = bassClr;
      }
      // front edge: always part of the treble meter's base, full width
      for ( int i = 0; i < SIZEOF_SMALL_NEO; ++i ) {
         carpet->ropeLeds[ i ] = trebleClr;
      }

      // right side: front corner at FRONT_RIGHT, back corner at BACK_RIGHT
      for ( int i = FRONT_RIGHT; i <= BACK_RIGHT; ++i ) {
         float distFromFront = i - FRONT_RIGHT;
         float distFromBack = BACK_RIGHT - i;
         carpet->ropeLeds[ i ] = vuMeterColor( distFromBack < bassLitCount, distFromFront < trebleLitCount, bassClr, trebleClr );
      }
      // left side: back corner at BACK_LEFT, front corner at FRONT_LEFT
      for ( int i = BACK_LEFT; i <= FRONT_LEFT; ++i ) {
         float distFromBack = i - BACK_LEFT;
         float distFromFront = FRONT_LEFT - i;
         carpet->ropeLeds[ i ] = vuMeterColor( distFromBack < bassLitCount, distFromFront < trebleLitCount, bassClr, trebleClr );
      }
   }

   // slow rolling desaturation applied to every base color used in both
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
   // the headlight[0]/left[3]/back[6]/right[9]) full white for 3 pulses,
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

      int bassRaw = AudioBoard::getLow();
      bool isHit = ( bassRaw > AudioBoard::getPeakThresholdRaw() && bassRaw > lastBassForTrigger_ );
      lastBassForTrigger_ = bassRaw;

      if ( isHit ) {
         static const uint32_t SILENCE_RESET_MS = 3000;
         bool silenceExpired = !hadHit_ || lastHitTimer_.elapsed( SILENCE_RESET_MS );
         bool exceedsPeak = bassRaw > suppressionPeak_;
         if ( !strobeActive_ && ( silenceExpired || exceedsPeak ) ) {
            strobeActive_ = true;
            strobeTimer_.reset();
            suppressionPeak_ = bassRaw;
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
      for ( int i = 0; i < 8; ++i ) carpet->megabarLeds[ cornerMegabars[ i ] ] = lit ? CRGB::White : CRGB::Black;
      for ( int i = 0; i < NUM_CHINA_LEDS; ++i ) {
         carpet->chinaLeds[ i ] = lit ? CRGB::White : CRGB::Black;
         carpet->chinaLeds[ i ].w = lit ? 255 : 0;
      }
   }
};
