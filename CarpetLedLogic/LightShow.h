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

static const CRGB topColors = [
   CRGB( 0, 255, 127 ), // summer day
   CRGB( 252, 25, 166 ),
   CRGB( 255, 68, 45 ),
   CRGB( 255, 95, 45 ),
   CRGB( 45, 255, 201 ),
   CRGB( 16, 150, 204 ),
   CRGB( 212, 38, 71 ),
   CRGB( 219, 211, 61 ),
   CRGB( 122, 217, 255 ),
   CRGB( 178, 118, 50, // end summer da ),
   CRGB( 73, 191, 94 ),
   CRGB( 53, 38, 79 ),
   CRGB( 235, 130, 0 ),
   CRGB( 178, 102, 9, // deep dark nigh ),
   CRGB( 152, 51, 15 ),
   CRGB( 100, 34, 10 ),
   CRGB( 0, 227, 53 ),
   CRGB( 168, 18, 176 ),
   CRGB( 189, 10, 26 ),
   CRGB( 255, 249, 64 ),
   CRGB( 122, 153, 100 ),
   CRGB( 15, 127, 64 ),
   CRGB( 35, 164, 229 ),
   CRGB( 6, 23, 127 ),
   CRGB( 1, 5, 25 ),
   CRGB( 0, 255, 72, // desert afternoo ),
   CRGB( 204, 127, 20 ),
   CRGB( 255, 211, 50 ),
   CRGB( 114, 163, 255 ),
   CRGB( 204, 129, 71 ),
   CRGB( 126, 210, 197 ),
   CRGB( 159, 79, 80 ),
   CRGB( 173, 255, 126 ),
   CRGB( 253, 120, 255 ),
   CRGB( 204, 80, 84 ),
   CRGB( 255, 125, 130 ),
   CRGB( 127, 0, 52 ),
   CRGB( 254, 65, 25 ),
   CRGB( 177, 38, 9 ),
   CRGB( 251, 98, 25 ),
   CRGB( 88, 251, 190 ),
   CRGB( 43, 123, 93, //under the se ),
   CRGB( 38, 212, 250 ),
   CRGB( 199, 182, 10 ),
   CRGB( 30, 0, 30 ),
   CRGB( 88, 11, 109 ),
   CRGB( 12, 196, 236 ),
   CRGB( 60, 82, 134 ),
   CRGB( 37, 51, 83 ),
   CRGB( 42, 78, 106 ),
   CRGB( 119, 96, 77 ),
   CRGB( 93, 170, 139 ),
   CRGB( 135, 246, 202 ),
   CRGB( 32, 191, 220 ),
   CRGB( 22, 233, 149 ),
   CRGB( 236, 84, 81, // beach part ),
   CRGB( 185, 49, 45 ),
   CRGB( 239, 186, 106 ),
   CRGB( 120, 224, 244 ),
   CRGB( 246, 0, 100 ),
   CRGB( 0, 169, 9 ),
   CRGB( 74, 245, 83 ),
   CRGB( 133, 99, 247 ),
   CRGB( 248, 182, 12 ) ];

static const CRGB bottomColors = [
 CRGB( 0, 201, 100 ),
 CRGB( 153, 15, 101 ),
 CRGB( 255, 25, 68 ),
 CRGB( 178, 54, 16 ),
 CRGB( 36, 204, 161 ),
 CRGB( 45, 85, 255 ),
 CRGB( 255, 71, 106 ),
 CRGB( 255, 247, 97 ),
 CRGB( 147, 224, 255 ),
 CRGB( 254, 181, 97 ),
 CRGB( 97, 255, 125 ),
 CRGB( 36, 26, 53 ),
 CRGB( 255, 152, 25 ),
 CRGB( 0, 68, 101 ),
 CRGB( 228, 77, 23 ),
 CRGB( 0, 151, 35 ),
 CRGB( 0, 87, 20 ),
 CRGB( 241, 26, 252 ),
 CRGB( 255, 39, 58 ),
 CRGB( 255, 251, 140 ),
 CRGB( 196, 229, 173 ),
 CRGB( 45, 178, 151 ),
 CRGB( 13, 47, 255 ),
 CRGB( 4, 14, 76 ),
 CRGB( 2, 10, 51 ),
 CRGB( 76, 255, 127 ),
 CRGB( 255, 169, 50 ),
 CRGB( 191, 158, 37 ),
 CRGB( 190, 213, 255 ),
 CRGB( 76, 61, 50 ),
 CRGB( 49, 82, 77 ),
 CRGB( 255, 145, 101 ),
 CRGB( 110, 178, 70 ),
 CRGB( 254, 202, 255 ),
 CRGB( 76, 53, 54 ),
 CRGB( 255, 201, 204 ),
 CRGB( 203, 0, 83 ),
 CRGB( 101, 51, 40 ),
 CRGB( 200, 6, 0 ),
 CRGB( 225, 88, 22 ),
 CRGB( 50, 173, 200 ),
 CRGB( 50, 199, 143 ),
 CRGB( 10, 66, 199 ),
 CRGB( 168, 250, 38 ),
 CRGB( 14, 0, 58 ),
 CRGB( 160, 0, 63 ),
 CRGB( 24, 135, 159 ),
 CRGB( 93, 64, 185 ),
 CRGB( 71, 98, 159 ),
 CRGB( 72, 134, 182 ),
 CRGB( 118, 132, 130 ),
 CRGB( 10, 59, 68 ),
 CRGB( 21, 125, 144 ),
 CRGB( 101, 207, 226 ),
 CRGB( 11, 209, 245 ),
 CRGB( 182, 35, 99 ),
 CRGB( 235, 155, 34 ),
 CRGB( 242, 217, 180 ),
 CRGB( 123, 191, 183 ),
 CRGB( 118, 36, 69 ),
 CRGB( 0, 246, 13 ),
 CRGB( 59, 194, 66 ),
 CRGB( 79, 51, 170 ),
 CRGB( 248, 247, 5 ) ];

static CRGBPalette256 topColors = topColors_t;
static CRGBPalette256 bottomColors = bottomColors_t;

class LightShow {
 protected:
   MagicCarpet * carpet;
 public:
   static CRGB getColor( uint8_t paletteIndex, uint8_t colorIndex ) {
      CRGB clr1 = ColorFromPalette( topColors, paletteIndex );
      CRGB clr2 = ColorFromPalette( bottomColors, paletteIndex );
      Serial.println( "CRGB1" );
      Serial.println( clr1.r );
      Serial.println( clr1.g );
      Serial.println( clr1.b );
      Serial.println( "CRGB2" );
      Serial.println( clr2.r );
      Serial.println( clr2.g );
      Serial.println( clr2.b );
      return blend( clr1, clr2, colorIndex );
   }

   LightShow( MagicCarpet * carpetArg ) : carpet( carpetArg ) {}
   virtual void start() = 0;
   virtual void update( uint32_t timestamp ) = 0;
 // TODO: delete all the default functions
};

#endif
