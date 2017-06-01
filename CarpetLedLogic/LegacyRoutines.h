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

#ifndef __FASTLED_H
#define __FASTLED_H

#include <DmxSimple.h>
#include <FastLED.h>

#endif

// number of dmx leds
#define NUM_DMX_LEDS 10

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

// max voltage from an analog input pin
#define MAX_VOLTAGE 1023

// TODO: there's probably a FastLED alternative to this.
//======SINE Lookup Array
extern const uint8_t SINELUT[];

// the dmx led array
CRGB dmxLeds[ NUM_DMX_LEDS ];

uint8_t readMode() {
   return digitalRead( MODE2_PIN ) << 3 | digitalRead( MODE3_PIN ) << 2;
}

uint8_t readDigitalInput() {
   return digitalRead( INPUT0_PIN ) << 2 |
          digitalRead( INPUT1_PIN ) << 1 |
          digitalRead( INPUT2_PIN );
}

CRGB readAnalogInput() {
   // TODO: name these pins
   // TODO: why are we inverting the values here?
   // convert analog inputs into rgb values between 0-255
   uint16_t r = ( MAX_VOLTAGE - analogRead( ANALOG_LOW_PIN ) ) / 4;
   uint16_t g = ( MAX_VOLTAGE - analogRead( ANALOG_MID_PIN ) ) / 4;
   uint16_t b = ( MAX_VOLTAGE - analogRead( ANALOG_HIGH_PIN ) ) / 4;
   return CRGB( r, g, b );
}

uint8_t readBrightnessInput() {
   uint16_t ret = ( MAX_VOLTAGE - analogRead( ANALOG_BRIGHTNESS_PIN ) ) / 4;
   return ret;
}

void clearDmxLeds() {
  for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
    dmxLeds[ i ] = CRGB::Black; // black is all zeros
  }
  FastLED.show(); // TODO: do we really need the show here? i don't think so...
}

void strobeHit() {
  for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
    dmxLeds[ i ] = CRGB::White; // white is all zeros
  }
  FastLED.show();
  FastLED.delay( 10 );
  clearDmxLeds();
  FastLED.delay( 10 );
}

// Note: this only fades out the red
void fadeOut() {
   for ( int red = 255; red >= 0; --red ) {
      for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
        dmxLeds[ i ] = CRGB::Red;
      }
      FastLED.show();
      FastLED.delay( 1 );
  }
}

// TODO: this seems like a really useless effect. Not sure if i ported it wrong, or
//       maybe it just never did what was inteneded? Needs fixing or removing.
void chaseBurst() {
  for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
     dmxLeds[ i ] = CRGB::Red;
  }
  FastLED.show();
  FastLED.delay( 10 );
}

// Scene #1
// this sets all lights to have red bass, green mids, and blue highs
void dmxScene1() {
   CRGB inputColor = readAnalogInput();
   for ( int i = 0; i < NUM_DMX_LEDS; ++i ) {
      dmxLeds[ i ] = inputColor;
   }
   FastLED.setBrightness( 255 );
   FastLED.show();
}

// Scene #2
// this sets lights to be R, G, or B
// with brightness proportional to music L, M, H
// and when a beat is detected, the colors shift by one.
// rope light TBD
void dmxScene2() {
   static CRGB previousColor = CRGB::Black;
   static uint8_t arrayOffset = 0;

   CRGB inputColor = readAnalogInput();
   uint8_t brightness = readBrightnessInput();

   // TODO: we should set the RGB values back to zero before we call this
   for ( int i = 0; i < NUM_DMX_LEDS; i += 3 ) {
      int j = ( i + arrayOffset ) % NUM_DMX_LEDS;
      dmxLeds[ i ].red = inputColor.red;
      if ( i + 1 <= NUM_DMX_LEDS ) {
         j = ( i + arrayOffset + 1 ) % NUM_DMX_LEDS;
         dmxLeds[ i + 1 ].green = inputColor.green;
         if ( i + 2 <= NUM_DMX_LEDS ) {
            j = ( i + arrayOffset + 2 ) % NUM_DMX_LEDS;
            dmxLeds[ i + 2 ].blue = inputColor.blue;
         }
      }
   }

   // TODO: why are we checking whether the previous color was larger than the new
   // color? shouldn't we only care when the new color is larger? since that's the
   // upswing of the beat?
   // Note: the bitwise-or is intentional here, for performance
   if ( inputColor.red - previousColor.red >= 70 |
        previousColor.red - inputColor.red >= 70 |
        readDigitalInput() == LEFT_BUTTON ) {
      arrayOffset = ++arrayOffset % NUM_DMX_LEDS;
   }
   FastLED.setBrightness( brightness );
   FastLED.show();
}

//Scene 3
//Legacy 'PAR for the course' scene
//only responds to the beat, like the incandescent floods did
void dmxScene3() {
   static CRGB previousColor = CRGB::Black;
   static uint8_t arrayOffset = 0;

   CRGB inputColor = readAnalogInput();
   uint8_t brightness = readBrightnessInput();

   // TODO: we should set the RGB values back to zero before we call this
   for ( int i = 0; i < NUM_DMX_LEDS; i += 3 ) {
      int j = ( i + arrayOffset ) % NUM_DMX_LEDS;
      dmxLeds[ i ].red = 255;
      if ( i + 1 <= NUM_DMX_LEDS ) {
         j = ( i + arrayOffset + 1 ) % NUM_DMX_LEDS;
         dmxLeds[ i + 1 ].green = 255;
         if ( i + 2 <= NUM_DMX_LEDS ) {
            j = ( i + arrayOffset + 2 ) % NUM_DMX_LEDS;
            dmxLeds[ i + 2 ].blue = 255;
         }
      }
   }

   // TODO: why are we checking whether the previous color was larger than the new
   // color? shouldn't we only care when the new color is larger? since that's the
   // upswing of the beat?
   // Note: the bitwise-or is intentional here, for performance
   if ( inputColor.red - previousColor.red >= 70 |
        previousColor.red - inputColor.red >= 70 |
        readDigitalInput() == LEFT_BUTTON ) {
      arrayOffset = ++arrayOffset % NUM_DMX_LEDS;
   }
   FastLED.setBrightness( brightness );
   FastLED.show();
}

//Scene 4
//SLUT Heartbeat
//doesnt respond to music, just heartbeats and rope chases
void dmxScene4() {
   static uint8_t heartColor = 0;
   static uint8_t sineIndex = 0;

   uint8_t brightness = readBrightnessInput();
   uint8_t wirelessInput = readDigitalInput();

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
        dmxLeds[ j ].red = SINELUT[ sineIndex];
      }
      break;
    case 1:
      // GREEN HEARTBEAT
      for ( int j = 0; j <= NUM_DMX_LEDS; ++j ) {
        dmxLeds[ j ].green = SINELUT[ sineIndex ];
      }
      break;
    case 2:
      // BLUE HEARTBEAT
      for ( int j = 0; j <= NUM_DMX_LEDS; ++j ) {
        dmxLeds[ j ].blue = SINELUT[ sineIndex ];
      }
      break;
    case 3:
      // TODO: very strange arithmatic here, what are we doing here exactly? Could we
      //       not just pulse the brightness instead? and maybe use the knob for
      //       color selection?
      // ORANGE HEARTBEAT
      for ( int j = 0; j <= NUM_DMX_LEDS; ++j ) {
        dmxLeds[ j ].red = 2 * SINELUT[ sineIndex ] - 254;
        dmxLeds[ j ].green = 2 * SINELUT[ ( sineIndex + 85 ) % 255 ] - 254;
        dmxLeds[ j ].blue = 2 * SINELUT[ ( sineIndex + 170 ) % 255 ] - 254;
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
   sineIndex = ++sineIndex % NUM_DMX_LEDS;

   FastLED.setBrightness( brightness );
   FastLED.show();
   FastLED.delay( 10 ); // delay so it seems to pulse slowly
}

//Scene 4
//Mid-Low Energy
//chasing with the lead light showing R+G+B color organ
//and trailing lights keeping their last value (then fading out)
/* NOT IMPLEMENTED IN OLD CODE */

void dmxSetup() {
  //set inputs from wireless board
  pinMode( INPUT0_PIN, INPUT );
  pinMode( INPUT1_PIN, INPUT );
  pinMode( INPUT1_PIN, INPUT );

  //set inputs from Mode Select Switch
  pinMode( MODE0_PIN, INPUT );
  pinMode( MODE1_PIN, INPUT );
  pinMode( MODE2_PIN, INPUT );
  pinMode( MODE3_PIN, INPUT );

  FastLED.addLeds<DMXSIMPLE, DMX_PIN, RGB>(dmxLeds, NUM_DMX_LEDS);
}

void dmxLoop() {
   // needs to be static so it persists after the loop restarts. Set it to a high
   // value to start with so we read the mode as a new mode the first time through
   // the loop
   static uint8_t lastModeSwitchInput = 0xFF;

   uint8_t modeSwitchInput = readMode();
   if ( modeSwitchInput != lastModeSwitchInput ) {
      // we've changed modes, clear the lights so we start from scratch again
      clearDmxLeds();
   }
   lastModeSwitchInput = modeSwitchInput;

   CRGB inputColor = readAnalogInput();
   // TODO: hmmmm, how does FastLED deal with dmx brightness?
   uint8_t brightness = readBrightnessInput();

   // main logic selection
   switch ( modeSwitchInput ) {
    case 0x0:
      dmxScene1();
      break;
    case 0x4:
      dmxScene2();
      break;
    case 0x8:
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
   uint8_t wirelessInput = readDigitalInput();
   if ( wirelessInput == CENTER_BUTTON ) {
      strobeHit();
   } else if ( wirelessInput == DOWN_BUTTON ) {
     fadeOut();
   } else if ( wirelessInput == LEFT_BUTTON ) {
     chaseBurst();
   }
}

// TODO: move this (and gamma correction aray) to another file
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

