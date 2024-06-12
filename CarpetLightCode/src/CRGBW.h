/* CRGBW.h
 *
 *   A helper class for using the RGBW robe lights with the FastLED CRGB class. When
 *   FastLED makes the switch to support RGBW we should scrap this code and use their
 *   implementation.
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#ifndef __CRBGW_H
#define __CRBGW_H

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include <assert.h>
#include <stdint.h>
#include <math.h>
#include "lib8tion.h"
#include "color.h"

#include "LedConsts.h"

int scaleTo255( int x, int max, int min ) {
   return 255 + ( ( -255 ) * ( x - min ) ) / ( max - min );
}

struct CRGBW : CRGB {
   union {
     uint8_t white;
     uint8_t w;
   };

 	 inline CRGBW() __attribute__((always_inline)) : CRGB() {
      w = 0;
   }

   // allow copy construction
 	 inline CRGBW( const CRGBW& rhs ) __attribute__((always_inline)) : CRGB( rhs ) {
      w = rhs.w;
   }

   // allow copy construction from regular CRGB object
 	 inline CRGBW( const CRGB& rhs ) __attribute__((always_inline)) : CRGB( rhs ) {}

   // allow construction from HSV color
 	 inline CRGBW( const CHSV& rhs ) __attribute__((always_inline)) : CRGB( rhs ) {}

    // allow assignment from one RGBW struct to another
 	 inline CRGBW& operator=( const CRGBW& rhs ) __attribute__((always_inline)) {
      w = rhs.w;
      return (CRGBW&) CRGB::operator=( rhs );
   }

   // allow assignment from one RGB struct to another
 	 inline CRGBW& operator=( const CRGB& rhs ) __attribute__((always_inline)) {
      return (CRGBW&) CRGB::operator=( rhs );
   }

   // allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code
 	 inline CRGBW& operator=( const uint32_t colorcode ) __attribute__((always_inline)) {
          return (CRGBW&) CRGB::operator=( colorcode );
      }

   // allow assignment from R, G, and B
 	 inline CRGBW& setRGB( uint8_t nr, uint8_t ng,
                         uint8_t nb ) __attribute__((always_inline)) {
      return (CRGBW&) CRGB::setRGB( nr, ng, nb );
   }

   // allow assignment from R, G, B, and W
 	 inline CRGBW& setRGBW( uint8_t nr, uint8_t ng, uint8_t nb,
                          uint8_t nw ) __attribute__((always_inline)) {
      w = nw;
      return (CRGBW&) CRGB::setRGB( nr, ng, nb );
   }

   // allow assignment from H, S, and V
 	 inline CRGBW& setHSV( uint8_t hue, uint8_t sat,
                         uint8_t val ) __attribute__((always_inline)) {
      return (CRGBW&) CRGB::setHSV( hue, sat, val );
   }

   // allow assignment from just a Hue, saturation and value automatically at max.
 	 inline CRGBW& setHue( uint8_t hue ) __attribute__((always_inline)) {
      return (CRGBW&) CRGB::setHue( hue );
   }

   // allow assignment from HSV color
 	 inline CRGB& operator=( const CHSV& rhs ) __attribute__((always_inline)) {
      return (CRGBW&) CRGB::operator=( rhs );
   }

   // allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code
 	 inline CRGBW& setColorCode( uint32_t colorcode ) __attribute__((always_inline)) {
      return (CRGBW&) CRGB::setColorCode( colorcode );
   }

   // allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code w/ w
 	 inline CRGBW& setColorCode( uint32_t colorcode, uint8_t iw ) __attribute__((always_inline)) {
      w = iw;
      return (CRGBW&) CRGB::setColorCode( colorcode );
   }

   // add one RGB to another, saturating at 0xFF for each channel
   inline CRGBW& operator+=( const CRGB& rhs ) {
      return (CRGBW&) CRGB::operator+=( rhs );
   }

   // add one RGB to another, saturating at 0xFF for each channel
   inline CRGBW& operator+=( const CRGBW& rhs ) {
       w = qadd8( w, rhs.w );
      return (CRGBW&) CRGB::operator+=( rhs );
   }

   // add a constant to each channel, saturating at 0xFF
   // this is NOT an operator+= overload because the compiler
   // can't usefully decide when it's being passed a 32-bit
   // constant (e.g. CRGB::Red) and an 8-bit one (CRGB::Blue)
   inline CRGBW& addToRGB( uint8_t d ) {
      return (CRGBW&) CRGB::addToRGB( d );
   }

   // add a constant to each channel, saturating at 0xFF
   // this is NOT an operator+= overload because the compiler
   // can't usefully decide when it's being passed a 32-bit
   // constant (e.g. CRGB::Red) and an 8-bit one (CRGB::Blue)
   inline CRGBW& addToRGBW( uint8_t d ) {
       w = qadd8( w, d );
      return (CRGBW&) CRGB::addToRGB( d );
   }

   // subtract one RGB from another, saturating at 0x00 for each channel
   inline CRGBW& operator-=( const CRGB& rhs ) {
      return (CRGBW&) CRGB::operator-=( rhs );
   }

   // subtract a constant from each channel, saturating at 0x00
   // this is NOT an operator-= overload because the compiler
   // can't usefully decide when it's being passed a 32-bit
   // constant (e.g. CRGB::Red) and an 8-bit one (CRGB::Blue)
   inline CRGB& subtractFromRGB( uint8_t d ) {
      return (CRGBW&) CRGB::subtractFromRGB( d );
   }

   // subtract a constant of '1' from each channel, saturating at 0x00
   inline CRGB& operator--() __attribute__((always_inline)){
      return (CRGBW&) CRGB::subtractFromRGB( 1 );
   }

   // subtract a constant of '1' from each channel, saturating at 0x00
   inline CRGB operator--( int ) __attribute__((always_inline)) {
      return (CRGBW&) CRGB::subtractFromRGB( 1 );
   }

   // add a constant of '1' from each channel, saturating at 0xFF
   inline CRGB& operator++() __attribute__((always_inline)) {
      return (CRGBW&) CRGB::addToRGB( 1 );
   }

   // add a constant of '1' from each channel, saturating at 0xFF
   inline CRGB operator++( int ) __attribute__((always_inline)) {
      return (CRGBW&) CRGB::addToRGB( 1 );
   }

   // divide each of the channels by a constant
   inline CRGB& operator/=( uint8_t d ) {
      return (CRGBW&) CRGB::operator/=( d );
   }

   // right shift each of the channels by a constant
   inline CRGB& operator>>=( uint8_t d) {
      return (CRGBW&) CRGB::operator>>=( d );
   }

   // multiply each of the channels by a constant,
   // saturating each channel at 0xFF
   inline CRGB& operator*=( uint8_t d ) {
      return (CRGBW&) CRGB::operator*=( d );
   }

   // scale down a RGB to N 256ths of it's current brightness, using
   // 'video' dimming rules, which means that unless the scale factor is ZERO
   // each channel is guaranteed NOT to dim down to zero.  If it's already
   // nonzero, it'll stay nonzero, even if that means the hue shifts a little
   // at low brightness levels.
   inline CRGB& nscale8_video( uint8_t scaledown ) {
      return (CRGBW&) CRGB::nscale8_video( scaledown );
   }

   // %= is a synonym for nscale8_video.  Think of it is scaling down
   // by "a percentage"
   inline CRGB& operator%=( uint8_t scaledown ) {
      return (CRGBW&) CRGB::operator%=( scaledown );
   }

   // fadeLightBy is a synonym for nscale8_video( ..., 255-fadefactor)
   inline CRGB& fadeLightBy( uint8_t fadefactor ) {
      return (CRGBW&) CRGB::fadeLightBy( fadefactor );
   }

   // scale down a RGB to N 256ths of it's current brightness, using
   // 'plain math' dimming rules, which means that if the low light levels
   // may dim all the way to 100% black.
   inline CRGB& nscale8( uint8_t scaledown ) {
      return (CRGBW&) CRGB::nscale8( scaledown );
   }

   // scale down a RGB to N 256ths of it's current brightness, using
   // 'plain math' dimming rules, which means that if the low light levels
   // may dim all the way to 100% black.
   inline CRGB& nscale8( const CRGB & scaledown ) {
      return (CRGBW&) CRGB::nscale8( scaledown );
   }

   // return a CRGB object that is a scaled down version of this object
   inline CRGB scale8( const CRGB & scaledown ) const {
      return CRGB::scale8( scaledown );
   }

   // fadeToBlackBy is a synonym for nscale8( ..., 255-fadefactor)
   inline CRGB& fadeToBlackBy( uint8_t fadefactor ) {
      return (CRGBW&) CRGB::fadeToBlackBy( fadefactor );
   }

   // "or" operator brings each channel up to the higher of the two values
   inline CRGB& operator|=( const CRGB& rhs ) {
      return (CRGBW&) CRGB::operator|=( rhs );
   }

   // "or" operator brings each channel up to the higher of the two values
   inline CRGB& operator|=( uint8_t d ) {
      return (CRGBW&) CRGB::operator|=( d );
   }

   // "and" operator brings each channel down to the lower of the two values
   inline CRGB& operator&=( const CRGB& rhs ) {
      return (CRGBW&) CRGB::operator&=( rhs);
   }

   // "and" operator brings each channel down to the lower of the two values
   inline CRGB& operator&=( uint8_t d ) {
      return (CRGBW&) CRGB::operator&=( d );
   }

   // this allows testing a CRGB for zero-ness
   inline operator bool() const __attribute__((always_inline)) {
      return r || g || b || w;
   }

   /// invert each channel
   inline CRGB operator-() {
      return CRGB::operator-();
   }

   // Get the 'luma' of a CRGB object - aka roughly how much light the
   // CRGB pixel is putting out (from 0 to 255).
   inline uint8_t getLuma()  const {
      return CRGB::getLuma();
   }

   // Get the average of the R, G, and B values
   inline uint8_t getAverageLight()  const {
      return CRGB::getAverageLight();
   }

   // maximize the brightness of this CRGBW object
   inline void maximizeBrightness( uint8_t limit = 255 )  {
      uint8_t max = red;
      if( green > max) max = green;
      if( blue > max) max = blue;
      if( white > max) max = white;
      uint16_t factor = ((uint16_t)(limit) * 256) / max;
      red =   (red   * factor) / 256;
      green = (green * factor) / 256;
      blue =  (blue  * factor) / 256;
      white =  (white  * factor) / 256;
   }

   // return a new CRGB object after performing a linear interpolation between this object and the passed in object
   inline CRGB lerp8( const CRGB& other, fract8 frac ) const {
      return CRGB::lerp8( other, frac );
   }

   // return a new CRGB object after performing a linear interpolation between this object and the passed in object
   inline CRGB lerp16( const CRGB& other, fract16 frac ) const {
      return CRGB::lerp16( other, frac );
   }

   // return a new CRGB object after performing a linear interpolation between this object and the passed in object
   inline CRGBW lerp8( const CRGBW& other, fract8 frac ) const {
      CRGBW ret = (CRGBW) CRGB::lerp8( other, frac );
      ret.w = lerp8by8( w, other.w, frac);
      return ret;
   }

   // return a new CRGB object after performing a linear interpolation between this object and the passed in object
   inline CRGBW lerp16( const CRGBW& other, fract16 frac ) const {
      CRGBW ret = (CRGBW) CRGB::lerp16( other, frac );
      ret.w = lerp16by16( w, other.w, frac );
      return ret;
   }

} CRGBW_t;

struct CRGBWUA : CRGBW {
   union {
       uint8_t u;
       uint8_t black;
   };
   union {
       uint8_t a;
       // TODO: figure out what a is for...
       uint8_t dunno;
   };

 	 inline CRGBWUA() __attribute__((always_inline)) : CRGBW() {
      u = 0;
      a = 0;
   }

   // allow copy construction
 	 inline CRGBWUA( const CRGBWUA& rhs ) __attribute__((always_inline)) : CRGBW( rhs ) {
      u = rhs.u;
      a = rhs.a;
   }

   // allow copy construction from CRGBW object
 	 inline CRGBWUA( const CRGBW& rhs ) __attribute__((always_inline)) : CRGBW( rhs ) {}

   // allow copy construction from regular CRGB object
 	 inline CRGBWUA( const CRGB& rhs ) __attribute__((always_inline)) : CRGBW( rhs ) {}

   // allow construction from HSV color
 	 inline CRGBWUA( const CHSV& rhs ) __attribute__((always_inline)) : CRGBW( rhs ) {}

    // allow assignment from one RGBWUA struct to another
 	 inline CRGBWUA& operator=( const CRGBWUA& rhs ) __attribute__((always_inline)) {
      u = rhs.u;
      a = rhs.a;
      return (CRGBWUA&) CRGBW::operator=( rhs );
   }

   // allow assignment from one RGBW struct to another
 	 inline CRGBWUA& operator=( const CRGBW& rhs ) __attribute__((always_inline)) {
      return (CRGBWUA&) CRGBW::operator=( rhs );
   }

   // allow assignment from HSV color
 	 inline CRGBWUA& operator=( const CHSV& rhs ) __attribute__((always_inline)) {
      return (CRGBWUA&) CRGBW::operator=( rhs );
   }

   // allow assignment from one RGB struct to another
 	 inline CRGBWUA& operator=( const CRGB& rhs ) __attribute__((always_inline)) {
      return (CRGBWUA&) CRGB::operator=( rhs );
   }

   // allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code
 	 inline CRGBWUA& operator=( const uint32_t colorcode ) __attribute__((always_inline)) {
          return (CRGBWUA&) CRGB::operator=( colorcode );
      }

} CRGBWUA_t;

/* A collection of utility functions for dealing with CRGB/CRGBW values.
 *
 * Conversion functions:
 *    FastLED doesn't support rgbw/rgbwua leds. We work around this by offsetting the
 *    color values to accomodate the white value. For three rgbw leds we end up
 *    sending four rgb packets. For dmx, this is straightforward. Each of the values
 *    simply shifts by one to handle the extra value:
 *
 *    (4-channel dmx: rgbw)
 *      red on led 1, green on led 1, blue on led 1
 *      white led 1, red on led 2, green led 2
 *      blue led 2, white led 2, red led 3
 *      green led 3, blue led 3, white led 3
 *
 *    TODO: what does a stand for?
 *    (6-channel dmx: rgbwua)
 *      red on led 1, green on led 1, blue on led 1
 *      white led 1, u/v on led 2, 'a' led 2
 *
 *    However, the protocol for the neopixel rgbw leds shuffles the values around in
 *    an odd way. The ordering is as follows:
 *
 *      red on led 1, green on led 1, blue on led 1
 *      green led 2, white on led 1, red led 2
 *      white led 2, blue led 2, green led 3
 *      blue led 3, red led 3, white led 3
 *
 *    credit to user joekitch on the arduino forum for figuring out neopixel order
 *    https://forum.arduino.cc/index.php?topic=432470.0
 *
 * Gamma Correction:
 *    Defines a single constant function gammaCorrect() which takes a reference to a
 *    CRGB object and corrects the values inside. CRGBW objects can be used as well,
 *    but we don't gamma correct for the white (what would that even mean?).
 *
 *    Gamma: 2.8
 */
namespace LedUtil {

constexpr uint16_t resizeCRGBW( uint16_t leds ) {
   return ( ( leds * 4 ) / 3 ) + ( ( leds % 3 ) != 0 );
}

template< class T >
static void reverse( T *arr, uint16_t size ) {
   for ( int i = 0; i < size / 2; ++i ) {
      int rearIndex = size - i - 1;
      T tmp = arr[ rearIndex ];
      arr[ rearIndex ] = arr[ i ];
      arr[ i ] = tmp;
   }
}

inline uint8_t gamR( uint8_t r ) { return gammaR[ r ]; }
inline uint8_t gamG( uint8_t g ) { return gammaG[ g ]; }
inline uint8_t gamB( uint8_t b ) { return gammaB[ b ]; }

inline CRGB & gammaCorrect( CRGB & clr ) {
   clr.r = gamR( clr.r );
   clr.g = gamG( clr.g );
   clr.b = gamB( clr.b );
   return clr;
}

static void convertNeo( CRGBW *src, CRGB *dst ) {
   dst[ 0 ] = CRGB( gamR( src[ 0 ].r ), gamG( src[ 0 ].g ), gamB( src[ 0 ].b ) );
   dst[ 1 ] = CRGB( gamG( src[ 1 ].g ), src[ 0 ].w, gamR( src[ 1 ].r ) );
   dst[ 2 ] = CRGB( src[ 1 ].w, gamB( src[ 1 ].b ), gamG( src[ 2 ].g ) );
   dst[ 3 ] = CRGB( gamB( src[ 2 ].b ), gamR( src[ 2 ].r ), src[ 2 ].w );
}

// NOTE: this function assumes that dst is large enough to fit all of src,
//       so it's up to you to make sure you've allocated enough space!
static void convertNeoArray( CRGBW *src, CRGB *dst, uint16_t srcSize ) {
   int s = 0, d = 0;
   while ( s < srcSize ) {
     convertNeo( src + s, dst + d );
     s += 3;
     d += 4;
   }
}

template< typename C >
static void fill( C *src, CRGB clr, uint16_t srcSize ) {
   for ( uint16_t i = 0; i < srcSize; ++i ) {
      src[ i ] = clr;
   }
}

} // end namespace LedUtil

# endif
