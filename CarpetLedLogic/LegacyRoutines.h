// TODO: this filename is mispelled
// TODO: add header

// ported over from eliot's old code

//this is the Arduino Uno code for the DMX organ
//written by Eliot Barker
//August 2011
//
//for the DMX organ board built by Eliot
//July 2011
//
//note that the mode select switch hardware has some changes made.
//MODELSE4 and MODESEL1 connect to arduino UART TX and RX, so they are difficult to use and debug.
//instead those outputs should be connected to IO 8 and IO 9.
//
//Mode switch (4-mode version)
//Mode definition
//ModeSwitchInput = C --> 8 --> 4 --> 0 in clockwise direction
//
//

// I2C rountines for Eliot's Organ board
// written by cameron burge July, 2011

#ifndef __DMXSIMPLE_H
#define __DMXSIMPLE_H
#include <DmxSimple.h>
#endif

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#ifndef __WIRE_H
#define __WIRE_H
#include <Wire.h>
#endif

#include "MagicCarpet.h"

// number of dmx leds
#define NUM_DMX_LEDS 18
#define NUM_CONVERTED_DMX_LEDS ( NUM_DMX_LEDS + ( NUM_DMX_LEDS / 3 ) )

// all the dmx lights are on the same pin
#define DMX_PIN 3

// analog inputs
#define ANALOG_LOW_PIN 3
#define ANALOG_MID_PIN 2
#define ANALOG_HIGH_PIN 1
#define ANALOG_BRIGHTNESS_PIN 0

// inputs from mode select switch
#define MODE0_PIN 8
#define MODE1_PIN 9
#define MODE2_PIN 2
#define MODE3_PIN 4

// inputs from wireless board
#define INPUT0_PIN 5
#define INPUT1_PIN 6
#define INPUT2_PIN 7

// button ids from wireless board
#define UP_BUTTON 3
#define LEFT_BUTTON 4
#define RIGHT_BUTTON 1
#define DOWN_BUTTON 7
#define CENTER_BUTTON 6

//I2C address of the LP3950
#define LP3950_address 0x50
#define ON 1
#define OFF 0

//Sets up the values of the LP3950 registers
#define RGB_PWM ON    //set bit 7 reg 0 to ON to enable PWM control, if OFF you only
                      //have on/off control of leds via xSWx bits
#define RGB_START ON  //must be set to ON or PWM and blinking control will be disabled

//the following 6 bits control whether the lowside switches which drive the 
//LEDs and the Arduino ADC inputs are enabled
#define RGB_RSW1 ON
#define RGB_GSW1 ON
#define RGB_BSW1 ON
#define RGB_RSW2 ON
#define RGB_GSW2 ON
#define RGB_BSW2 ON

//ON sets the beginning time of the turn-on slope. The on-time is relative to the selected 
//blinking cycle length. On-setting N (N = 0–15) sets the on-time to N/16 * cycle length.
#define RON 0x2
#define GON 0x2
#define BON 0x2

//OFF sets the beginning time of the turn-off slope. Off-time is relative to blinking 
//cycle length in the same way as on-time.
#define ROFF 0x2
#define GOFF 0x2
#define BOFF 0x2

//SLOPE sets the turn-on and turn-off slopes. Fastest slope is set by [0000] and slowest 
//by [1111]. SLOPE changes the duty cycle at constant, programmable rate. For each slope 
//setting the maximum slope time appears at maximum DUTY setting. When DUTY is reduced, 
//the slope time decreases proportionally. For example, in case of maximum DUTY, the sloping
//time can be adjusted from 31 ms [0000] to 930 ms [1111]. For DUTY [0111] the sloping time
//is 14 ms [0000] to 434 ms [1111]. The blinking cycle has no effect on SLOPE.
#define RSLOPE 0xA
#define GSLOPE 0xA
#define BSLOPE 0xA

//DUTY sets the brightness of the LED by adjusting the duty cycle of the PWM driver. The 
//minimum DUTY cycle is 0% [0000] and the maximum in the flash mode is A 100% [1111]. 
//The peak pulse current is determined by the external resistor, LED forward voltage drop
//and the boost voltage. In the normal mode the maximum duty cycle is 33%.
#define RDUTY 0xA
#define GDUTY 0xA
#define BDUTY 0xA

//CYCLE sets the blinking cycle: [000] for 0.25s, [001] for 0.5s, [010] for 1.0s, [011] 
//for 2.0s. and [1XX] for 4.0s CYCLE effects to all RGB LEDs
#define CYCLE0 OFF
#define CYCLE1 ON
#define CYCLE2 OFF

//Setting these bits to ON enables the PWM to the low side FET switches
#define R1_PMW ON
#define G1_PMW ON
#define B1_PMW ON
#define R2_PMW ON
#define G2_PMW ON
#define B2_PMW ON

//Set to on to enable boost converter
#define EN_BOOST OFF

//MUST BE SET ON if set to OFF device will enter a low power standby mode
//(think regs can still be writen in standby)
#define NSTBY ON

//Flash mode enable control for RGB1 and RGB2. In the flash mode (EN_FLASH = 1) 
//RGB outputs are PWM controlled simultaneously, not in 3-phase system as in the normal mode.
#define EN_FLASH ON

//dont know what this does
#define AUTOLOAD_EN OFF

//Both bits should be to ON to enable both channel outputs to Audio sync
#define RGB_SEL0 ON
#define RGB_SEL1 ON

//Input signal gain control. Gain has a range from 0 dB to 21 dB with 3 dB steps
#define GAIN_SEL0 ON
#define GAIN_SEL1 OFF
#define GAIN_SEL2 ON

//Automatic gain control. Set EN_AGC = 1 to enable automatic control or 0 to disable.
//When EN_AGC is disabled, the audio input signal gain value is defined by GAIN_SEL
#define EN_AGC ON

//Synchronization mode selector. Set SYNC_MODE = 0 for amplitude synchronization. 
//Set SYNC_MODE = 1 for frequency synchronization
#define SYNC_MODE 1

//Audio synchronization enabled.
#define EN_SYNC ON

//[00] ... Single ended input signal, ASE.
//[01] ... Differential input signal, AD1 and AD2.
//[10] ... Stereo input or single ended and differential input signal.
//Note: Sum of input signals divided by 2.
//[11] ... No input
#define INPUT_SEL0 1
#define INPUT_SEL1 0

//see datasheet, changes how lights behave and depend if you are in freq or amp sync
#define MODE_CTRL0 0
#define MODE_CTRL1 0

//Control for speed of the mapping
#define SPEED_CTRL0 0
#define SPEED_CTRL1 0

MagicCarpet * carpet;

// TODO: there's probably a FastLED alternative to this.
//======SINE Lookup Array
extern const uint8_t SINELUT[];

void strobeHit() {
  for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
    carpet->dmxLeds[ i ] = CRGB::White; // white is all ones
  }
  carpet->show();
  FastLED.delay( 10 );
  carpet->clearDmx();
  FastLED.delay( 10 );
}

// Note: this only fades out the red
void fadeOut() {
   for ( int red = 255; red >= 0; --red ) {
      for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
        carpet->dmxLeds[ i ] = CRGB::Red;
      }
      carpet->show();
      FastLED.delay( 1 );
  }
}

// TODO: this seems like a really useless effect. Not sure if i ported it wrong, or
//       maybe it just never did what was inteneded? Needs fixing or removing.
void chaseBurst() {
  for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
     carpet->dmxLeds[ i ] = CRGB::Red;
  }
  carpet->show();
  FastLED.delay( 10 );
}

// Scene #1
// this sets all lights to have red bass, green mids, and blue highs
void dmxScene1() {
   CRGB inputColor = carpet->readAnalogInput();
   for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
      carpet->dmxLeds[ i ] = inputColor;
   }
   FastLED.setBrightness( 255 );
   carpet->show();
}

// Scene #2
// this sets lights to be R, G, or B
// with brightness proportional to music L, M, H
// and when a beat is detected, the colors shift by one.
// rope light TBD
void dmxScene2() {
   static CRGB previousColor = CRGB::Black;
   static uint8_t arrayOffset = 0;

   CRGB inputColor = carpet->readAnalogInput();
   uint8_t brightness = carpet->readBrightnessInput();

   // TODO: we should set the RGB values back to zero before we call this
   for ( int i = 0; i < NUM_DMX_LEDS; i += 3 ) {
      int j = ( i + arrayOffset ) % NUM_DMX_LEDS;
      carpet->dmxLeds[ i ].red = inputColor.red;
      if ( i + 1 <= NUM_DMX_LEDS ) {
         j = ( i + arrayOffset + 1 ) % NUM_DMX_LEDS;
         carpet->dmxLeds[ i + 1 ].green = inputColor.green;
         if ( i + 2 <= NUM_DMX_LEDS ) {
            j = ( i + arrayOffset + 2 ) % NUM_DMX_LEDS;
            carpet->dmxLeds[ i + 2 ].blue = inputColor.blue;
         }
      }
   }

   // TODO: why are we checking whether the previous color was larger than the new
   // color? shouldn't we only care when the new color is larger? since that's the
   // upswing of the beat?
   // Note: the bitwise-or is intentional here, for performance
   if ( inputColor.red - previousColor.red >= 70 |
        previousColor.red - inputColor.red >= 70 |
        carpet->readDigitalInput() == LEFT_BUTTON ) {
      arrayOffset = ++arrayOffset % NUM_DMX_LEDS;
   }
   FastLED.setBrightness( brightness );
   carpet->show();
}

//Scene 3
//Legacy 'PAR for the course' scene
//only responds to the beat, like the incandescent floods did
void dmxScene3() {
   static CRGB previousColor = CRGB::Black;
   static uint8_t arrayOffset = 0;

   CRGB inputColor = carpet->readAnalogInput();
   uint8_t brightness = carpet->readBrightnessInput();

   // TODO: we should set the RGB values back to zero before we call this
   for ( int i = 0; i < NUM_DMX_LEDS; i += 3 ) {
      int j = ( i + arrayOffset ) % NUM_DMX_LEDS;
      carpet->dmxLeds[ i ].red = 255;
      if ( i + 1 <= NUM_DMX_LEDS ) {
         j = ( i + arrayOffset + 1 ) % NUM_DMX_LEDS;
         carpet->dmxLeds[ i + 1 ].green = 255;
         if ( i + 2 <= NUM_DMX_LEDS ) {
            j = ( i + arrayOffset + 2 ) % NUM_DMX_LEDS;
            carpet->dmxLeds[ i + 2 ].blue = 255;
         }
      }
   }

   // TODO: why are we checking whether the previous color was larger than the new
   // color? shouldn't we only care when the new color is larger? since that's the
   // upswing of the beat?
   // Note: the bitwise-or is intentional here, for performance
   if ( inputColor.red - previousColor.red >= 70 |
        previousColor.red - inputColor.red >= 70 |
        carpet->readDigitalInput() == LEFT_BUTTON ) {
      arrayOffset = ++arrayOffset % NUM_DMX_LEDS;
   }
   FastLED.setBrightness( brightness );
   carpet->show();
}

//Scene 4
//SLUT Heartbeat
//doesnt respond to music, just heartbeats and rope chases
void dmxScene4() {
   static uint8_t heartColor = 0;
   static uint8_t sineIndex = 0;

   uint8_t brightness = carpet->readBrightnessInput();
   uint8_t wirelessInput = carpet->readDigitalInput();

   if ( wirelessInput == LEFT_BUTTON ) {
      heartColor = 0;
   } else if ( wirelessInput == UP_BUTTON ) {
      heartColor = 1;
   } else if ( wirelessInput == RIGHT_BUTTON ) {
      heartColor = 2;
   } else if ( wirelessInput == DOWN_BUTTON ) {
      heartColor = 3;
   } else if ( wirelessInput == CENTER_BUTTON ) {
      strobeHit();
   }

   // TODO: we should set the RGB values back to zero before we call this
   switch ( heartColor ) {
    case 0:
      // RED HEARTBEAT
      for ( int j = 0; j <= NUM_DMX_LEDS; ++j ) {
         carpet->dmxLeds[ j ].red = SINELUT[ sineIndex];
      }
      break;
    case 1:
      // GREEN HEARTBEAT
      for ( int j = 0; j <= NUM_DMX_LEDS; ++j ) {
         carpet->dmxLeds[ j ].green = SINELUT[ sineIndex ];
      }
      break;
    case 2:
      // BLUE HEARTBEAT
      for ( int j = 0; j <= NUM_DMX_LEDS; ++j ) {
         carpet->dmxLeds[ j ].blue = SINELUT[ sineIndex ];
      }
      break;
    case 3:
      // TODO: very strange arithmatic here, what are we doing here exactly? Could we
      //       not just pulse the brightness instead? and maybe use the knob for
      //       color selection?
      // ORANGE HEARTBEAT
      for ( int j = 0; j <= NUM_DMX_LEDS; ++j ) {
        carpet->dmxLeds[ j ].red = 2 * SINELUT[ sineIndex ] - 254;
        carpet->dmxLeds[ j ].green = 2 * SINELUT[ ( sineIndex + 85 ) % 255 ] - 254;
        carpet->dmxLeds[ j ].blue = 2 * SINELUT[ ( sineIndex + 170 ) % 255 ] - 254;
      }
      break;
    default:
      break;
   }

   // the following seems to be for the rope light, I don't think it's been used for
   // awhile. We can pretty easily convert it to use the new rope light LEDS.
   /*
     //Rope light fade to black
     DmxSimple.write( 45, (2*SINELUT[i])-254);
     DmxSimple.write( 46, (2*SINELUT[((i + 64) % 255)])-254);
     DmxSimple.write( 47, (2*SINELUT[((i + 128) % 255)])-254);
     DmxSimple.write( 48, (2*SINELUT[((i + 192) % 255)])-254);

     //Rope light fade to half mode
     DmxSimple.write( 45, SINELUT[i]);
     DmxSimple.write( 46, SINELUT[((i + 64) % 255)]);
     DmxSimple.write( 47, SINELUT[((i + 128) % 255)]);
     DmxSimple.write( 48, SINELUT[((i + 192) % 255)]);
   */

   // update the sine index for the next round
   // sineIndex = ++sineIndex % 256;
   static bool up = true;
   if ( up ) {
      ++sineIndex;
   } else {
      --sineIndex;
   }
   if ( sineIndex == 256 ) {
      up = false;
   } else if ( sineIndex == 0 ) {
      up = true;
   }

   FastLED.setBrightness( brightness );
   carpet->show();
   FastLED.delay( 10 ); // delay so it seems to pulse slowly
}

//Scene 4
//Mid-Low Energy
//chasing with the lead light showing R+G+B color organ
//and trailing lights keeping their last value (then fading out)
/* NOT IMPLEMENTED IN OLD CODE */

void write_reg( int reg, unsigned char data ) {
   Wire.beginTransmission( LP3950_address );
   Wire.write( reg );
   Wire.write( data );
   Wire.endTransmission();
}

void dmxSetup() {
   carpet = theMagicCarpet();
   carpet->setup();

   // join i2c bus (address optional for master)
   Wire.begin();

   // declare variables to mask register bits into
   unsigned char  RGBCONTROL = 0;
   unsigned char  RED = 0;
   unsigned char  GREEN = 0;
   unsigned char  BLUE = 0;
   unsigned char  RSLOPEDUTY = 0;
   unsigned char  GSLOPEDUTY = 0;
   unsigned char  BSLOPEDUTY = 0;
   unsigned char  CYCLEPWM = 0;
   unsigned char  ENABLES = 0;
   unsigned char  BOOSTFREQ = 0;
   unsigned char  BOOSTVOLTAGE = 0;
   unsigned char  AUDCON1 = 0;
   unsigned char  AUDCON2 = 0;

   // bunch of code which masks all the LP3950 register controls bits into
   // bytes which can be writen in to the register

   //assembling register 0x00 RGB_control
   RGBCONTROL = ( (RGB_PWM << 7)|(RGB_START << 6)|(RGB_RSW1 << 5)|(RGB_GSW1 << 4)|
   (RGB_BSW1 << 3)|(RGB_RSW2 << 2)|(RGB_GSW2 << 1)|(RGB_BSW2) );

   RED = ( (RON << 4)|(ROFF) );

   GREEN = ( (GON << 4)|(GOFF) );

   BLUE = ( (BON << 4)|(BOFF) );

   RSLOPEDUTY = ( (RSLOPE << 4)|(RDUTY) );

   GSLOPEDUTY = ( (GSLOPE << 4)|(GDUTY) );

   BSLOPEDUTY = ( (BSLOPE << 4)|(BDUTY) );

   CYCLEPWM = ( (CYCLE1 << 7)|(CYCLE0 << 6)|(R1_PMW << 5)|(G1_PMW << 4)|
   (B1_PMW << 3)|(R2_PMW << 2)|(G2_PMW << 1)|(B2_PMW) );

   ENABLES = ( (CYCLE2 << 7)|(NSTBY << 6)|(EN_BOOST << 5)|(EN_FLASH << 4)|
   (0 << 3)|(AUTOLOAD_EN << 2)|(RGB_SEL1 << 1)|(RGB_SEL0) );

   // Boost convert is not connected so I have fixed the regs to it is off
   BOOSTFREQ = 0x01;
   BOOSTVOLTAGE = 0x3F;

   AUDCON1 = ( (GAIN_SEL2 << 7)|(GAIN_SEL1 << 6)|(GAIN_SEL0 << 5)|(SYNC_MODE << 4)|
   (EN_AGC << 3)|(EN_SYNC << 2)|(INPUT_SEL1 << 1)|(INPUT_SEL0) );

   AUDCON2 = ( (0 << 7)|(0 << 6)|(0 << 5)|(0 << 4)|
   (MODE_CTRL1 << 3)|(MODE_CTRL0 << 2)|(SPEED_CTRL1 << 1)|(SPEED_CTRL0) );

   // configure LP3950 registers
   write_reg( 0x00, RGBCONTROL );
   write_reg( 0x01, RED );
   write_reg( 0x02, GREEN );
   write_reg( 0x03, BLUE );
   write_reg( 0x04, RSLOPEDUTY );
   write_reg( 0x05, GSLOPEDUTY );
   write_reg( 0x06, BSLOPEDUTY );
   write_reg( 0x07, CYCLEPWM );
   write_reg( 0x0B, ENABLES );
   write_reg( 0x0C, BOOSTFREQ );
   write_reg( 0x0D, BOOSTVOLTAGE );
   write_reg( 0x2A, AUDCON1 );
   write_reg( 0x2B, AUDCON2 );
}

void dmxLoop() {
   // needs to be static so it persists after the loop restarts. Set it to a high
   // value to start with so we read the mode as a new mode the first time through
   // the loop
   static uint8_t lastModeSwitchInput = 0xFF;

   uint8_t modeSwitchInput = carpet->readMode();
   if ( modeSwitchInput != lastModeSwitchInput ) {
      // we've changed modes, clear the lights so we start from scratch again
      carpet->clearDmx();
   }
   lastModeSwitchInput = modeSwitchInput;

   CRGB inputColor = carpet->readAnalogInput();
   // TODO: hmmmm, how does FastLED deal with dmx brightness?
   uint8_t brightness = carpet->readBrightnessInput();

   // main logic selection
   switch ( modeSwitchInput ) {
    case 0x0:
      dmxScene1();
      break;
    case 0x4: // this is the third position
      dmxScene2();
      break;
    case 0x8: // this is the fourth position
      dmxScene3();
      break;
    case 0xc:
      dmxScene4();
      break;
    default:
      break;
   }

   // these are the responses to wireless button presses
   // they do not vary by mode, but they could...
   uint8_t wirelessInput = carpet->readDigitalInput();
   if ( wirelessInput == CENTER_BUTTON ) {
      strobeHit();
   } else if ( wirelessInput == DOWN_BUTTON ) {
     fadeOut();
   } else if ( wirelessInput == LEFT_BUTTON ) {
     chaseBurst();
   }
}

// TODO: move this (and gamma correction array) to another file
//======SINE Lookup Array
const uint8_t SINELUT[ 256 ] = {
 190,
 192,
 193,
 195,
 196,
 198,
 199,
 201,
 202,
 204,
 205,
 207,
 208,
 210,
 211,
 213,
 214,
 216,
 217,
 218,
 220,
 221,
 223,
 224,
 225,
 226,
 228,
 229,
 230,
 231,
 232,
 234,
 235,
 236,
 237,
 238,
 239,
 240,
 241,
 242,
 243,
 243,
 244,
 245,
 246,
 246,
 247,
 248,
 248,
 249,
 249,
 250,
 250,
 251,
 251,
 252,
 252,
 252,
 252,
 253,
 253,
 253,
 253,
 253,
 253,
 253,
 253,
 253,
 253,
 252,
 252,
 252,
 252,
 251,
 251,
 251,
 250,
 250,
 249,
 249,
 248,
 247,
 247,
 246,
 245,
 245,
 244,
 243,
 242,
 241,
 240,
 239,
 238,
 237,
 236,
 235,
 234,
 233,
 232,
 231,
 229,
 228,
 227,
 226,
 224,
 223,
 222,
 220,
 219,
 218,
 216,
 215,
 213,
 212,
 211,
 209,
 208,
 206,
 205,
 203,
 202,
 200,
 199,
 197,
 195,
 194,
 192,
 191,
 189,
 188,
 186,
 185,
 183,
 181,
 180,
 178,
 177,
 175,
 174,
 172,
 171,
 169,
 168,
 167,
 165,
 164,
 162,
 161,
 160,
 158,
 157,
 156,
 154,
 153,
 152,
 151,
 149,
 148,
 147,
 146,
 145,
 144,
 143,
 142,
 141,
 140,
 139,
 138,
 137,
 136,
 135,
 135,
 134,
 133,
 133,
 132,
 131,
 131,
 130,
 130,
 129,
 129,
 129,
 128,
 128,
 128,
 128,
 127,
 127,
 127,
 127,
 127,
 127,
 127,
 127,
 127,
 127,
 128,
 128,
 128,
 128,
 129,
 129,
 130,
 130,
 131,
 131,
 132,
 132,
 133,
 134,
 134,
 135,
 136,
 137,
 137,
 138,
 139,
 140,
 141,
 142,
 143,
 144,
 145,
 146,
 148,
 149,
 150,
 151,
 152,
 154,
 155,
 156,
 157,
 159,
 160,
 162,
 163,
 164,
 166,
 167,
 169,
 170,
 172,
 173,
 175,
 176,
 178,
 179,
 181,
 182,
 184,
 185,
 187,
 188,
 190 };

