/* AudioBoard.h
 *
 *    All of the #defines needed for the LP3950 audio board are included here.
 *    The setupAudioBoard function writes all the required registers.
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#ifndef __AUDIO_BOARD_H
#define __AUDIO_BOARD_H

#ifndef __WIRE_H
#define __WIRE_H
#include <Wire.h>
#endif

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

void write_reg( int reg, unsigned char data ) {
   Wire.beginTransmission( LP3950_address );
   Wire.write( reg );
   Wire.write( data );
   Wire.endTransmission();
}

void setupAudioBoard() {
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

#endif
