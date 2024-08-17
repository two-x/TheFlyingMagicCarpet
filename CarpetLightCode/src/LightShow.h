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
 // TODO: delete all the default functions
};

#endif
