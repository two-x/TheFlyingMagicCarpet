#ifndef __CRBGW_H
#define __CRBGW_H

#include "FastLED.h"

#include <stdint.h>
#include "lib8tion.h"
#include "color.h"

FASTLED_NAMESPACE_BEGIN

struct CRGBW;

// Representation of an RGBW pixel (Red, Green, Blue, White)
struct CRGBW {
	union {
		struct {
            union {
                uint8_t r;
                uint8_t red;
            };
            union {
                uint8_t g;
                uint8_t green;
            };
            union {
                uint8_t b;
                uint8_t blue;
            };
            union {
                uint8_t w;
                uint8_t white;
            };
        };
		uint8_t raw[4];
	};

	inline uint8_t& operator[] (uint8_t x) __attribute__((always_inline)) {
        return raw[x];
  }

  inline const uint8_t& operator[] (uint8_t x) const __attribute__((always_inline)) {
      return raw[x];
  }

    // default values are UNINITIALIZED
	inline CRGBW() __attribute__((always_inline)) {}

    /// allow construction from R, G, B
    inline CRGBW( uint8_t ir, uint8_t ig, uint8_t ib)  __attribute__((always_inline))
        : r(ir), g(ig), b(ib)
    {
    }

    /// allow construction from R, G, B, W
    inline CRGBW( uint8_t ir, uint8_t ig, uint8_t ib, uint8_t iw)  __attribute__((always_inline))
        : r(ir), g(ig), b(ib), w(iw)
    {
    }

    /// allow construction from 32-bit (really 24-bit) bit 0xRRGGBB color code
    inline CRGBW( uint32_t colorcode)  __attribute__((always_inline))
    : r((colorcode >> 16) & 0xFF), g((colorcode >> 8) & 0xFF), b((colorcode >> 0) & 0xFF)
    {
    }

    /// allow construction from 32-bit (really 24-bit) bit 0xRRGGBB color code w/ white
    inline CRGBW( uint32_t colorcode, uint8_t iw)  __attribute__((always_inline))
    : r((colorcode >> 16) & 0xFF), g((colorcode >> 8) & 0xFF), b((colorcode >> 0) & 0xFF), w(iw)
    {
    }

    /// allow construction from a LEDColorCorrection enum
    inline CRGBW( LEDColorCorrection colorcode) __attribute__((always_inline))
    : r((colorcode >> 16) & 0xFF), g((colorcode >> 8) & 0xFF), b((colorcode >> 0) & 0xFF)
    {

    }

    /// allow construction from a LEDColorCorrection enum w/ white
    inline CRGBW( LEDColorCorrection colorcode, uint8_t iw) __attribute__((always_inline))
    : r((colorcode >> 16) & 0xFF), g((colorcode >> 8) & 0xFF), b((colorcode >> 0) & 0xFF), w(iw)
    {

    }

    /// allow construction from a ColorTemperature enum
    inline CRGBW( ColorTemperature colorcode) __attribute__((always_inline))
    : r((colorcode >> 16) & 0xFF), g((colorcode >> 8) & 0xFF), b((colorcode >> 0) & 0xFF)
    {

    }

    /// allow construction from a ColorTemperature enum w/ white
    inline CRGBW( ColorTemperature colorcode, uint8_t iw) __attribute__((always_inline))
    : r((colorcode >> 16) & 0xFF), g((colorcode >> 8) & 0xFF), b((colorcode >> 0) & 0xFF), w(iw)
    {

    }

    /// allow copy construction
	inline CRGBW(const CRGBW& rhs) __attribute__((always_inline))
    {
        r = rhs.r;
        g = rhs.g;
        b = rhs.b;
        w = rhs.w;
    }

    /// allow copy construction from regular CRGB object
	inline CRGBW(const CRGB& rhs) __attribute__((always_inline))
    {
        r = rhs.r;
        g = rhs.g;
        b = rhs.b;
    }

    /// allow construction from HSV color
	inline CRGBW(const CHSV& rhs) __attribute__((always_inline))
    {
       CRGB tmp;
        hsv2rgb_rainbow( rhs, tmp);
        *this = tmp;
    }

    /// allow assignment from one RGB struct to another
	inline CRGBW& operator= (const CRGBW& rhs) __attribute__((always_inline))
    {
        r = rhs.r;
        g = rhs.g;
        b = rhs.b;
        w = rhs.w;
        return *this;
    }

    /// allow assignment from one RGB struct to another
	inline CRGBW& operator= (const CRGB& rhs) __attribute__((always_inline))
    {
        r = rhs.r;
        g = rhs.g;
        b = rhs.b;
        return *this;
    }

    /// allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code
	inline CRGBW& operator= (const uint32_t colorcode) __attribute__((always_inline))
    {
        r = (colorcode >> 16) & 0xFF;
        g = (colorcode >>  8) & 0xFF;
        b = (colorcode >>  0) & 0xFF;
        return *this;
    }

    /// allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code w/ w
	inline CRGBW& operator= (const uint32_t colorcode, uint8_t iw) __attribute__((always_inline))
    {
        r = (colorcode >> 16) & 0xFF;
        g = (colorcode >>  8) & 0xFF;
        b = (colorcode >>  0) & 0xFF;
        w = iw;
        return *this;
    }

    /// allow assignment from R, G, and B
	inline CRGBW& setRGBW(uint8_t nr, uint8_t ng, uint8_t nb) __attribute__((always_inline))
    {
        r = nr;
        g = ng;
        b = nb;
        return *this;
    }

    /// allow assignment from R, G, B, and W
	inline CRGBW& setRGBW(uint8_t nr, uint8_t ng, uint8_t nb, uint8_t nw) __attribute__((always_inline))
    {
        r = nr;
        g = ng;
        b = nb;
        w = nw;
        return *this;
    }

    /// allow assignment from H, S, and V
	inline CRGB& setHSV (uint8_t hue, uint8_t sat, uint8_t val) __attribute__((always_inline))
    {
       CRGB tmp;
        hsv2rgb_rainbow( CHSV(hue, sat, val), tmp);
        *this = tmp;
    }

    /// allow assignment from just a Hue, saturation and value automatically at max.
	inline CRGB& setHue (uint8_t hue) __attribute__((always_inline))
    {
       CRGB tmp;
        hsv2rgb_rainbow( CHSV(hue, 255, 255), tmp);
        *this = tmp;
    }

    /// allow assignment from HSV color
	inline CRGB& operator= (const CHSV& rhs) __attribute__((always_inline))
    {
       CRGB tmp;
        hsv2rgb_rainbow( rhs, tmp);
        *this = tmp;
    }

    /// allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code
	inline CRGB& setColorCode (uint32_t colorcode) __attribute__((always_inline))
    {
        r = (colorcode >> 16) & 0xFF;
        g = (colorcode >>  8) & 0xFF;
        b = (colorcode >>  0) & 0xFF;
        return *this;
    }

    /// allow assignment from 32-bit (really 24-bit) 0xRRGGBB color code w/ w
	inline CRGBW& setColorCode (uint32_t colorcode, uint8_t iw) __attribute__((always_inline))
    {
        r = (colorcode >> 16) & 0xFF;
        g = (colorcode >>  8) & 0xFF;
        b = (colorcode >>  0) & 0xFF;
        w = iw;
        return *this;
    }


    /// add one RGB to another, saturating at 0xFF for each channel
    inline CRGBW& operator+= (const CRGB& rhs )
    {
        r = qadd8( r, rhs.r);
        g = qadd8( g, rhs.g);
        b = qadd8( b, rhs.b);
        return *this;
    }

    /// add one RGB to another, saturating at 0xFF for each channel
    inline CRGBW& operator+= (const CRGBW& rhs )
    {
        r = qadd8( r, rhs.r);
        g = qadd8( g, rhs.g);
        b = qadd8( b, rhs.b);
        w = qadd8( w, rhs.w);
        return *this;
    }

    /// add a constant to each channel, saturating at 0xFF
    /// this is NOT an operator+= overload because the compiler
    /// can't usefully decide when it's being passed a 32-bit
    /// constant (e.g. CRGB::Red) and an 8-bit one (CRGB::Blue)
    inline CRGBW& addToRGB (uint8_t d )
    {
        r = qadd8( r, d);
        g = qadd8( g, d);
        b = qadd8( b, d);
        return *this;
    }

    /// add a constant to each channel, saturating at 0xFF
    /// this is NOT an operator+= overload because the compiler
    /// can't usefully decide when it's being passed a 32-bit
    /// constant (e.g. CRGB::Red) and an 8-bit one (CRGB::Blue)
    inline CRGB& addToRGBW (uint8_t d )
    {
        r = qadd8( r, d);
        g = qadd8( g, d);
        b = qadd8( b, d);
        w = qadd8( w, d);
        return *this;
    }

    /// subtract one RGB from another, saturating at 0x00 for each channel
    inline CRGBW& operator-= (const CRGB& rhs )
    {
        r = qsub8( r, rhs.r);
        g = qsub8( g, rhs.g);
        b = qsub8( b, rhs.b);
        return *this;
    }

    /// subtract one RGB from another, saturating at 0x00 for each channel
    inline CRGBW& operator-= (const CRGBW& rhs )
    {
        r = qsub8( r, rhs.r);
        g = qsub8( g, rhs.g);
        b = qsub8( b, rhs.b);
        w = qsub8( w, rhs.w);
        return *this;
    }
/*
    /// subtract a constant from each channel, saturating at 0x00
    /// this is NOT an operator+= overload because the compiler
    /// can't usefully decide when it's being passed a 32-bit
    /// constant (e.g. CRGB::Red) and an 8-bit one (CRGB::Blue)
    inline CRGB& subtractFromRGB(uint8_t d )
    {
        r = qsub8( r, d);
        g = qsub8( g, d);
        b = qsub8( b, d);
        return *this;
    }

    /// subtract a constant of '1' from each channel, saturating at 0x00
    inline CRGB& operator-- ()  __attribute__((always_inline))
    {
        subtractFromRGB(1);
        return *this;
    }

    /// subtract a constant of '1' from each channel, saturating at 0x00
    inline CRGB operator-- (int )  __attribute__((always_inline))
    {
        CRGB retval(*this);
        --(*this);
        return retval;
    }

    /// add a constant of '1' from each channel, saturating at 0xFF
    inline CRGB& operator++ ()  __attribute__((always_inline))
    {
        addToRGB(1);
        return *this;
    }

    /// add a constant of '1' from each channel, saturating at 0xFF
    inline CRGB operator++ (int )  __attribute__((always_inline))
    {
        CRGB retval(*this);
        ++(*this);
        return retval;
    }

    /// divide each of the channels by a constant
    inline CRGB& operator/= (uint8_t d )
    {
        r /= d;
        g /= d;
        b /= d;
        return *this;
    }

    /// right shift each of the channels by a constant
    inline CRGB& operator>>= (uint8_t d)
    {
      r >>= d;
      g >>= d;
      b >>= d;
      return *this;
    }

    /// multiply each of the channels by a constant,
    /// saturating each channel at 0xFF
    inline CRGB& operator*= (uint8_t d )
    {
        r = qmul8( r, d);
        g = qmul8( g, d);
        b = qmul8( b, d);
        return *this;
    }

    /// scale down a RGB to N 256ths of it's current brightness, using
    /// 'video' dimming rules, which means that unless the scale factor is ZERO
    /// each channel is guaranteed NOT to dim down to zero.  If it's already
    /// nonzero, it'll stay nonzero, even if that means the hue shifts a little
    /// at low brightness levels.
    inline CRGB& nscale8_video (uint8_t scaledown )
    {
        nscale8x3_video( r, g, b, scaledown);
        return *this;
    }

    /// %= is a synonym for nscale8_video.  Think of it is scaling down
    /// by "a percentage"
    inline CRGB& operator%= (uint8_t scaledown )
    {
        nscale8x3_video( r, g, b, scaledown);
        return *this;
    }

    /// fadeLightBy is a synonym for nscale8_video( ..., 255-fadefactor)
    inline CRGB& fadeLightBy (uint8_t fadefactor )
    {
        nscale8x3_video( r, g, b, 255 - fadefactor);
        return *this;
    }

    /// scale down a RGB to N 256ths of it's current brightness, using
    /// 'plain math' dimming rules, which means that if the low light levels
    /// may dim all the way to 100% black.
    inline CRGB& nscale8 (uint8_t scaledown )
    {
        nscale8x3( r, g, b, scaledown);
        return *this;
    }

    /// scale down a RGB to N 256ths of it's current brightness, using
    /// 'plain math' dimming rules, which means that if the low light levels
    /// may dim all the way to 100% black.
    inline CRGB& nscale8 (const CRGB & scaledown )
    {
        r = ::scale8(r, scaledown.r);
        g = ::scale8(g, scaledown.g);
        b = ::scale8(b, scaledown.b);
        return *this;
    }

    /// return a CRGB object that is a scaled down version of this object
    inline CRGB scale8 (const CRGB & scaledown ) const
    {
        CRGB out;
        out.r = ::scale8(r, scaledown.r);
        out.g = ::scale8(g, scaledown.g);
        out.b = ::scale8(b, scaledown.b);
        return out;
    }

    /// fadeToBlackBy is a synonym for nscale8( ..., 255-fadefactor)
    inline CRGB& fadeToBlackBy (uint8_t fadefactor )
    {
        nscale8x3( r, g, b, 255 - fadefactor);
        return *this;
    }

    /// "or" operator brings each channel up to the higher of the two values
    inline CRGB& operator|= (const CRGB& rhs )
    {
        if( rhs.r > r) r = rhs.r;
        if( rhs.g > g) g = rhs.g;
        if( rhs.b > b) b = rhs.b;
        return *this;
    }

    /// "or" operator brings each channel up to the higher of the two values
    inline CRGB& operator|= (uint8_t d )
    {
        if( d > r) r = d;
        if( d > g) g = d;
        if( d > b) b = d;
        return *this;
    }

    /// "and" operator brings each channel down to the lower of the two values
    inline CRGB& operator&= (const CRGB& rhs )
    {
        if( rhs.r < r) r = rhs.r;
        if( rhs.g < g) g = rhs.g;
        if( rhs.b < b) b = rhs.b;
        return *this;
    }

    /// "and" operator brings each channel down to the lower of the two values
    inline CRGB& operator&= (uint8_t d )
    {
        if( d < r) r = d;
        if( d < g) g = d;
        if( d < b) b = d;
        return *this;
    }

    /// this allows testing a CRGB for zero-ness
    inline operator bool() const __attribute__((always_inline))
    {
        return r || g || b;
    }

    /// invert each channel
    inline CRGB operator- ()
    {
        CRGB retval;
        retval.r = 255 - r;
        retval.g = 255 - g;
        retval.b = 255 - b;
        return retval;
    }

#if (defined SmartMatrix_h || defined SmartMatrix3_h)
    operator rgb24() const {
      rgb24 ret;
      ret.red = r;
      ret.green = g;
      ret.blue = b;
      return ret;
    }
#endif

    /// Get the 'luma' of a CRGB object - aka roughly how much light the
    /// CRGB pixel is putting out (from 0 to 255).
    inline uint8_t getLuma ( )  const {
        //Y' = 0.2126 R' + 0.7152 G' + 0.0722 B'
        //     54            183       18 (!)

        uint8_t luma = scale8_LEAVING_R1_DIRTY( r, 54) + \
        scale8_LEAVING_R1_DIRTY( g, 183) + \
        scale8_LEAVING_R1_DIRTY( b, 18);
        cleanup_R1();
        return luma;
    }

    /// Get the average of the R, G, and B values
    inline uint8_t getAverageLight( )  const {
#if FASTLED_SCALE8_FIXED == 1
        const uint8_t eightyfive = 85;
#else
        const uint8_t eightyfive = 86;
#endif
        uint8_t avg = scale8_LEAVING_R1_DIRTY( r, eightyfive) + \
        scale8_LEAVING_R1_DIRTY( g, eightyfive) + \
        scale8_LEAVING_R1_DIRTY( b, eightyfive);
        cleanup_R1();
        return avg;
    }

    /// maximize the brightness of this CRGB object
    inline void maximizeBrightness( uint8_t limit = 255 )  {
        uint8_t max = red;
        if( green > max) max = green;
        if( blue > max) max = blue;
        uint16_t factor = ((uint16_t)(limit) * 256) / max;
        red =   (red   * factor) / 256;
        green = (green * factor) / 256;
        blue =  (blue  * factor) / 256;
    }

    /// return a new CRGB object after performing a linear interpolation between this object and the passed in object
    inline CRGB lerp8( const CRGB& other, fract8 frac) const
    {
      CRGB ret;

      ret.r = lerp8by8(r,other.r,frac);
      ret.g = lerp8by8(g,other.g,frac);
      ret.b = lerp8by8(b,other.b,frac);

      return ret;
    }

    /// return a new CRGB object after performing a linear interpolation between this object and the passed in object
    inline CRGB lerp16( const CRGB& other, fract16 frac) const
    {
      CRGB ret;

      ret.r = lerp16by16(r<<8,other.r<<8,frac)>>8;
      ret.g = lerp16by16(g<<8,other.g<<8,frac)>>8;
      ret.b = lerp16by16(b<<8,other.b<<8,frac)>>8;

      return ret;
    }
*/
};

