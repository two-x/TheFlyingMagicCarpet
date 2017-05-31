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

#include <Wire.h>
#include <DmxSimple.h>


//======Compiler Definitions
#define NUMMEGABARS 9 //the number of Mega Bars connected to the DMX bus. 4-ch mode assumed. Mega bars must be addressed as 001, 005, 009, etc
#define DELAYPERIOD 100 //in ms. this should correspond to delay on audio channels.
#define SAMPLEPERIOD 10000 //in us. this should ideally be an even divisor of DELAYPERIOD, and determines the audio sample rate & DMX update rate.

#define UPBUTTON 3
#define LEFTBUTTON 4
#define RIGHTBUTTON 1
#define DOWNBUTTON 7
#define CENTERBUTTON 6

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


//======Function Definitions
void update_megabars(int offset);
void clear_megabars();
void fade_megabars();
void strobe_hit();
void fade_out();
void chase_burst();


//======Program Variables
int MegaBarRed[NUMMEGABARS]; 
int MegaBarGreen[NUMMEGABARS];
int MegaBarBlue[NUMMEGABARS];
int MegaBarBrightness[NUMMEGABARS];

//======SINE Lookup Array
unsigned char SINELUT[256] = {
 190 	,
 192 	,
 193 	,
 195 	,
 196 	,
 198 	,
 199 	,
 201 	,
 202 	,
 204 	,
 205 	,
 207 	,
 208 	,
 210 	,
 211 	,
 213 	,
 214 	,
 216 	,
 217 	,
 218 	,
 220 	,
 221 	,
 223 	,
 224 	,
 225 	,
 226 	,
 228 	,
 229 	,
 230 	,
 231 	,
 232 	,
 234 	,
 235 	,
 236 	,
 237 	,
 238 	,
 239 	,
 240 	,
 241 	,
 242 	,
 243 	,
 243 	,
 244 	,
 245 	,
 246 	,
 246 	,
 247 	,
 248 	,
 248 	,
 249 	,
 249 	,
 250 	,
 250 	,
 251 	,
 251 	,
 252 	,
 252 	,
 252 	,
 252 	,
 253 	,
 253 	,
 253 	,
 253 	,
 253 	,
 253 	,
 253 	,
 253 	,
 253 	,
 253 	,
 252 	,
 252 	,
 252 	,
 252 	,
 251 	,
 251 	,
 251 	,
 250 	,
 250 	,
 249 	,
 249 	,
 248 	,
 247 	,
 247 	,
 246 	,
 245 	,
 245 	,
 244 	,
 243 	,
 242 	,
 241 	,
 240 	,
 239 	,
 238 	,
 237 	,
 236 	,
 235 	,
 234 	,
 233 	,
 232 	,
 231 	,
 229 	,
 228 	,
 227 	,
 226 	,
 224 	,
 223 	,
 222 	,
 220 	,
 219 	,
 218 	,
 216 	,
 215 	,
 213 	,
 212 	,
 211 	,
 209 	,
 208 	,
 206 	,
 205 	,
 203 	,
 202 	,
 200 	,
 199 	,
 197 	,
 195 	,
 194 	,
 192 	,
 191 	,
 189 	,
 188 	,
 186 	,
 185 	,
 183 	,
 181 	,
 180 	,
 178 	,
 177 	,
 175 	,
 174 	,
 172 	,
 171 	,
 169 	,
 168 	,
 167 	,
 165 	,
 164 	,
 162 	,
 161 	,
 160 	,
 158 	,
 157 	,
 156 	,
 154 	,
 153 	,
 152 	,
 151 	,
 149 	,
 148 	,
 147 	,
 146 	,
 145 	,
 144 	,
 143 	,
 142 	,
 141 	,
 140 	,
 139 	,
 138 	,
 137 	,
 136 	,
 135 	,
 135 	,
 134 	,
 133 	,
 133 	,
 132 	,
 131 	,
 131 	,
 130 	,
 130 	,
 129 	,
 129 	,
 129 	,
 128 	,
 128 	,
 128 	,
 128 	,
 127 	,
 127 	,
 127 	,
 127 	,
 127 	,
 127 	,
 127 	,
 127 	,
 127 	,
 127 	,
 128 	,
 128 	,
 128 	,
 128 	,
 129 	,
 129 	,
 130 	,
 130 	,
 131 	,
 131 	,
 132 	,
 132 	,
 133 	,
 134 	,
 134 	,
 135 	,
 136 	,
 137 	,
 137 	,
 138 	,
 139 	,
 140 	,
 141 	,
 142 	,
 143 	,
 144 	,
 145 	,
 146 	,
 148 	,
 149 	,
 150 	,
 151 	,
 152 	,
 154 	,
 155 	,
 156 	,
 157 	,
 159 	,
 160 	,
 162 	,
 163 	,
 164 	,
 166 	,
 167 	,
 169 	,
 170 	,
 172 	,
 173 	,
 175 	,
 176 	,
 178 	,
 179 	,
 181 	,
 182 	,
 184 	,
 185 	,
 187 	,
 188 	,
 190 };
 
void write_reg( int reg, unsigned char data );
unsigned char read_reg( int reg );
void readtest();

unsigned char reading = 0;
unsigned char data = 0;
int counter = 0;
int reg = 0;
int HeartColor = 1;
  
int color, knob;
int level = 0;
int LowLevel, MidLevel, HiLevel;
int red, green, blue;
int brightness;
int previous_red, previous_green, previous_blue;
int array_offset;
int hitcount = 0;
int BeatPeriod = 0;  
int minBeatPeriod = 100;
 
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

unsigned char WirelessInput = 0;
unsigned char ModeSwitchInput = 0;
unsigned char LastModeSwitchInput = 0;
  
void setup()
{
    /* The most common pin for DMX output is pin 3, which DmxSimple
  ** uses by default. If you need to change that, do it here. */
  DmxSimple.usePin(3);
  
  //set pin7 to be an output.
  //pinMode(7, OUTPUT);
  
  //set inputs from wireless board
  pinMode (5, INPUT);
  pinMode (6, INPUT);
  pinMode (7, INPUT);
  
  //set inputs from Mode Select Switch
  pinMode (8, INPUT);
  pinMode (9, INPUT);
  pinMode (2, INPUT);
  pinMode (4, INPUT);
  
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
  
  Serial.println("welcome to arduino");   // print the reading
  
  //declare variables to mask register bits into
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

  
  
  //bunch of code which masks all the LP3950 register controls bits into 
  //bytes which can be writen in to the register
  
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
  
  //Boost convert is not connected so I have fixed the regs to it is off
  BOOSTFREQ = 0x01;
  BOOSTVOLTAGE = 0x3F;
  
  AUDCON1 = ( (GAIN_SEL2 << 7)|(GAIN_SEL1 << 6)|(GAIN_SEL0 << 5)|(SYNC_MODE << 4)|
  (EN_AGC << 3)|(EN_SYNC << 2)|(INPUT_SEL1 << 1)|(INPUT_SEL0) );
  
  AUDCON2 = ( (0 << 7)|(0 << 6)|(0 << 5)|(0 << 4)|
  (MODE_CTRL1 << 3)|(MODE_CTRL0 << 2)|(SPEED_CTRL1 << 1)|(SPEED_CTRL0) );
  
//remove if(0) statement inorder to see binary format of variables to used to write regs
if(0)
{
  Serial.println( RGBCONTROL, BIN);
  Serial.println( RED, BIN);
  Serial.println( GREEN, BIN);
  Serial.println( BLUE, BIN);
  Serial.println( RSLOPEDUTY, BIN);
  Serial.println( GSLOPEDUTY, BIN);
  Serial.println( BSLOPEDUTY, BIN);
  Serial.println( CYCLEPWM, BIN);
  Serial.println( ENABLES, BIN);
  Serial.println( BOOSTFREQ, BIN);
  Serial.println( BOOSTVOLTAGE, BIN);
  Serial.println( AUDCON1, BIN);
  Serial.println( AUDCON2, BIN);  
}  
  
 
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
  
  clear_megabars();
}

void loop()
{

/*  if( counter < 1 )
  {
    Serial.println("The registers are set to:");
  
    readtest();
    counter++;
  }
  delay(800);                  // wait a bit
*/
 //reflects board
 
// Read the wireless inputs
WirelessInput = ((digitalRead(7) << 2) | (digitalRead (6) << 1) | digitalRead (5) );
Serial.println( WirelessInput, HEX);

//read the mode select switch
//ModeSwitchInput = ((digitalRead(8) << 4) | (digitalRead(4) << 3) | (digitalRead(2) << 2) | (digitalRead(9) )); //old
ModeSwitchInput = ((digitalRead(4) << 3) | (digitalRead(2) << 2) ); //old
if (ModeSwitchInput != LastModeSwitchInput)
//Serial.println (ModeSwitchInput, HEX);
clear_megabars ();
LastModeSwitchInput = ModeSwitchInput;


    // ANDERS: these read the current values of low, mid, hi
    LowLevel = analogRead (3);
    MidLevel = analogRead (2);
    HiLevel = analogRead (1);
    knob = analogRead (0);

    // ANDERS: cache the previous values   
    previous_red = red;
    previous_green = green;
    previous_blue = blue;

    // ANDERS: convert low, mid, hi into rgb values
    //if (LowLevel >= red)
      red = (1023 - LowLevel)/4;
    //if (MidLevel >= green)
      green = (1023 - MidLevel)/4;
    //if (HiLevel >= blue)
      blue = (1023 - HiLevel)/4;  
    brightness = (1023 - knob)/4;
    

//====================================================
// Scene #1
// this sets all lights to have red bass, green mids, and blue highs
//
if (ModeSwitchInput == 0x0)
{
  for (int i=0; (i<NUMMEGABARS); i++)
  {
    MegaBarRed[i] = red;
    MegaBarGreen[i] = green;
    MegaBarBlue[i] = blue;
    MegaBarBrightness[i] = 255;
  } 
  
   update_megabars (0);
}

//====================================================
// Anders Scene #1
// Each time the bass hits, set it to high. Otherwise, pulse the lights
// and slowly fade. We can do something similar for the other levels if
// this works, though maybe we should have a few static lights for safety.
//
/*if (ModeSwitchInput == 0x0)
{
  static int brightness_anders = 0;
  static int pulse_anders = 0;

  // ANDERS: hmmm, i wonder if we could do some weird shit to match the beat with the pulse?
  //         Also, could probably use the sine function for this and do it proper, this is a hack
  if ( (red - previous_red) >= 120 ) //30-July - this had a bitwise instead of logial OR
  {
    brightness_anders = 255;
  } else if ( brightness_anders > 0 ) {
    brightness_anders--;
  }

  for (int i=0; (i<NUMMEGABARS); i++)
  {
    // ANDERS: if we triggered the bass, mids, and highs, set the lights to full levels
    MegaBarRed[i] =  brightness_anders;
    // ANDERS: lets worry about these later
    MegaBarGreen[i] = 0;
    MegaBarBlue[i] =  ( 255 - brightness_anders ) / 2;
    MegaBarBrightness[i] = 255;
  } 
  
   update_megabars (0);
}  
*/
//====================================================
// Anders Scene #2
// this sets all lights to have red bass, green mids, and blue highs.
// it pulses the lights from front to back, so front is solid, middle
// pulses slowly, and back pulses quickly
//
//static int brightness_anders = 0;
//static int pulse_anders = 0;
//
//if (ModeSwitchInput == 0x4)
//{
//  static int pulse_mid_anders = 0;
//  static int pulse_back_anders = 0;
//  // ANDERS: hmmm, i wonder if we could do some weird shit to match the beat with the pulse?
//  //         Also, could probably use the sine function for this and do it proper, this is a hack
//if (ModeSwitchInput == 0x4)
//{
//  for (int i=0; (i<NUMMEGABARS); i++)
//  {
//    // ANDERS: if we triggered the bass, mids, and highs, set the lights to full levels
//    MegaBarRed[i] = red;
//    // ANDERS: lets worry about these later
//    // MegaBarGreen[i] = green;
//    // MegaBarBlue[i] = blue;
//    // ANDERS: just guessing on which bars are in front
//    if ( i < ( NUMMEGABARS / 3 ) ) {
//       MegaBarBrightness[i] = 255;
//    } else if ( i < ( ( NUMMEGABARS / 3 ) * 2 ) ) {
//       static up = 0;
//       static delay = 0;
//       if ( delay == 0 ) {
//          delay = 1;
//       } else if ( up == 0 ) {
//          pulse_mid_anders--;
//       } else {
//          pulse_mid_anders++;
//       }
//       delay = 0;
//       if ( pulse_mid_anders == 50 ) {
//         up = 1;
//       } else if ( pulse_mid_anders == 255 ) {
//         up = 0;
//       }
//       MegaBarBrightness[i] = pulse_mid_anders;
//    }
//    } else {
//       static up = 0;
//       if ( up == 0 ) {
//          pulse_mid_anders--;
//       } else {
//          pulse_mid_anders++;
//       if ( pulse_mid_anders == 50 ) {
//         up = 1;
//       } else if ( pulse_mid_anders == 255 ) {
//         up = 0;
//       }
//       MegaBarBrightness[i] = pulse_mid_anders;
//    }
//  } 
//  
//   update_megabars (0);
//}
  
//====================================================
// Scene #2
// this sets lights to be R, G, or B
// with brightness proportional to music L, M, H
// and when a beat is detected, the colors shift by one.
// rope light TBD
if (ModeSwitchInput == 0x4)
{
  
  for (int i=0; (i<NUMMEGABARS); i+= 3)
  {
    MegaBarRed[i] = red;
    MegaBarBrightness[i] = brightness;
    if (i+1 <= NUMMEGABARS)
    {
      MegaBarGreen[i+1] = green;
      MegaBarBrightness[i+1] = brightness;
      if (i+2 <= NUMMEGABARS)
      {
        MegaBarBlue[i+2] = blue;
        MegaBarBrightness[i+2] = brightness;
      }
    }
  } 

  if ( ((red - previous_red) >= 70 ) || ((previous_red - red) >= 70 ) || WirelessInput == LEFTBUTTON ) //30-July - this had a bitwise instead of logial OR
  {
    array_offset ++;
    array_offset = array_offset % NUMMEGABARS;
    //digitalWrite(7, digitalRead(7) ^ 1); //debug only
  }
  
  update_megabars (array_offset);
} 
  
//Scene 3
//Legacy 'PAR for the course' scene
//only responds to the beat, like the incandescent floods did

if (ModeSwitchInput == 0x8)
{
  
  for (int i=0; (i<NUMMEGABARS); i+= 3)
  {
    MegaBarRed[i] = 255;
    MegaBarBrightness[i] = brightness;
    if (i+1 <= NUMMEGABARS)
    {
      MegaBarGreen[i+1] = 255;
      MegaBarBrightness[i+1] = brightness;
      if (i+2 <= NUMMEGABARS)
      {
        MegaBarBlue[i+2] = 255;
        MegaBarBrightness[i+2] = brightness;
      }
    }
  } 
  
  if ( ((red - previous_red) >= 70 ) || ((previous_red - red) >= 70 ) || WirelessInput == LEFTBUTTON ) //30-July - this had a bitwise instead of logial OR
  {
    array_offset ++;
    array_offset = array_offset % NUMMEGABARS;
    //digitalWrite(7, digitalRead(7) ^ 1); //debug only
  }
  
  update_megabars (array_offset);
  
}
  
  
  
//Scene 4
//SLUT Heartbeat
//doesnt respond to music, just heartbeats and rope chases

if (ModeSwitchInput == 0xC)
{

  
 for (int i = 0; i <= 254; i ++)
   {
  
     if (HeartColor == 1)
     {
       //RED HEARTBEAT    
       for (int j = 1; j <= (NUMMEGABARS*4); j+=4)
       {
         DmxSimple.write( j, SINELUT[i]);
         DmxSimple.write( j+1, 0);
         DmxSimple.write( j+2, 0);
         DmxSimple.write( j+3, 255);
       }
     }
     
     if (HeartColor == 2)
     {
       //GREEN HEARTBEAT
       for (int j = 1; j <= (NUMMEGABARS*4); j+=4)
       {
         DmxSimple.write( j, 0);
         DmxSimple.write( j+1, SINELUT[i]);
         DmxSimple.write( j+2, 0);
         DmxSimple.write( j+3, 255);
       }
     }
     
     if (HeartColor == 3)
     {
       //BLUE HEARTBEAT
       for (int j = 1; j <= (NUMMEGABARS*4); j+=4)
       {
         DmxSimple.write( j, 0);
         DmxSimple.write( j+1, 0);
         DmxSimple.write( j+2, SINELUT[i]);
         DmxSimple.write( j+3, 255);
       }
     }
     
     if (HeartColor == 4)
     {
       //ORANGE HEARTBEAT
       for (int j = 1; j <= (NUMMEGABARS*4); j+=4)
       {
         DmxSimple.write( j, 2*SINELUT[i]-254);
         DmxSimple.write( j+1, 2*SINELUT[((i + 85) % 255)]-254);
         DmxSimple.write( j+2, 2*SINELUT[((i + 170) % 255)]-254);
         DmxSimple.write( j+3, 255);
       }
     }
     
          //Uncomment below 4 lines for Ropelight SINE sequencing fade to black mode
    /* 
     DmxSimple.write( 45, (2*SINELUT[i])-254);
     DmxSimple.write( 46, (2*SINELUT[((i + 64) % 255)])-254);
     DmxSimple.write( 47, (2*SINELUT[((i + 128) % 255)])-254);
     DmxSimple.write( 48, (2*SINELUT[((i + 192) % 255)])-254);
    */     
    
    //Rope light fade to half mode
     DmxSimple.write( 45, SINELUT[i]);
     DmxSimple.write( 46, SINELUT[((i + 64) % 255)]);
     DmxSimple.write( 47, SINELUT[((i + 128) % 255)]);
     DmxSimple.write( 48, SINELUT[((i + 192) % 255)]);
   
   ModeSwitchInput = ((digitalRead(4) << 3) | (digitalRead(2) << 2) ); //old
   if (ModeSwitchInput != 0xC)
     break; 
   
   WirelessInput = ((digitalRead(7) << 2) | (digitalRead (6) << 1) | digitalRead (5) );
   if (WirelessInput == LEFTBUTTON)
     HeartColor = 1;
   if (WirelessInput == UPBUTTON)
     HeartColor = 2;
   if (WirelessInput == RIGHTBUTTON)
     HeartColor = 3;
   if (WirelessInput == DOWNBUTTON)
     HeartColor = 4;
   if (WirelessInput == CENTERBUTTON)
     strobe_hit ();

   knob = analogRead (0);
   brightness = (1023 - knob)/4; 
   
   delay (10);
   }
}
  
  
//Scene 4
//Mid-Low Energy
//chasing with the lead light showing R+G+B color organ
//and trailing lights keeping their last value (then fading out)




//these are the responses to wireless button presses
//these do not vary by mode, but they could...

  if (WirelessInput == CENTERBUTTON)
    strobe_hit ();
  
  if (WirelessInput == DOWNBUTTON)
    fade_out ();
    
  if (WirelessInput == LEFTBUTTON)
    chase_burst ();

    
  
}

void clear_megabars ()
{
  // initialize all the outputs to 0: 
  for (int thisMegaBar = 0; thisMegaBar < NUMMEGABARS; thisMegaBar++)
  {
    MegaBarRed[thisMegaBar] = 0; 
    MegaBarGreen[thisMegaBar] = 0;
    MegaBarBlue[thisMegaBar] = 0;  
  }
  
  update_megabars ( 0 );
}



void fade_megabars (int fadeNumber)
{
  //needs improvement, this should be made non-linear to better match how we perceive dimming of lights.
    for (int thisMegaBar = 0; thisMegaBar < NUMMEGABARS; thisMegaBar++)
  {
    MegaBarRed[thisMegaBar] -= fadeNumber;
    if (MegaBarRed[thisMegaBar] <= 0)
      MegaBarRed[thisMegaBar] = 0;
      
    MegaBarGreen[thisMegaBar] -= fadeNumber;
    if (MegaBarGreen[thisMegaBar] <= 0)
      MegaBarGreen[thisMegaBar] = 0;
      
    MegaBarBlue[thisMegaBar] -= fadeNumber;
    if (MegaBarBlue[thisMegaBar] <= 0)
      MegaBarBlue[thisMegaBar] = 0;  
  }
}



void update_megabars (int offset)
{
  int j;
  offset = offset % NUMMEGABARS;
  for (int i=0; (i<NUMMEGABARS); i++)
  {
    j = ((i + offset) % NUMMEGABARS);
    DmxSimple.write( ((4*j)+1), MegaBarRed[i]);
    DmxSimple.write( ((4*j)+2), MegaBarGreen[i]);
    DmxSimple.write( ((4*j)+3), MegaBarBlue[i]);
    DmxSimple.write( ((4*j)+4), MegaBarBrightness[i]);
  }
}

void strobe_hit ()
{
  for (int j=0; j<=5; j++)
  {
    for (int i=0; (i<NUMMEGABARS); i++)
    {
      DmxSimple.write( ((4*i)+1), 255);
      DmxSimple.write( ((4*i)+2), 255);
      DmxSimple.write( ((4*i)+3), 255);
      DmxSimple.write( ((4*i)+4), MegaBarBrightness[i]);
    }
    delay (10);
    clear_megabars ();
    delay (10);
  }
}

void fade_out ()
{
  for (int red=255; red>=0; red--)
  {
    for (int i=0; (i<NUMMEGABARS); i++)
    {
      DmxSimple.write( ((4*i)+1), red);
      DmxSimple.write( ((4*i)+2), 0);
      DmxSimple.write( ((4*i)+3), 0);
      DmxSimple.write( ((4*i)+4), 255);
    }
    delay (1);
  }
}

void chase_burst ()
{
  for (int j=0; j<=2; j++)
  {
    for (int i=0; (i<NUMMEGABARS); i++)
    {
      DmxSimple.write( ((4*i)+1), 255);
      DmxSimple.write( ((4*i)+2), 0);
      DmxSimple.write( ((4*i)+3), 0);
      DmxSimple.write( ((4*i)+4), 255);
      delay(20);
    }
  }
}

void write_reg( int reg, unsigned char data)
{
      // write a register
      Wire.beginTransmission(LP3950_address);  // chip address
      Wire.write(reg);                          // reg  
      Wire.write(data);                         // data
      Wire.endTransmission();                  // stop transmitting
}

unsigned char read_reg( int reg )
{
     
      // request to read a reg
      Wire.beginTransmission(LP3950_address); // address
      Wire.write(reg);             // register to read
      Wire.endTransmission();      // stop transmitting
    
    
      // request a  data byte
      Wire.requestFrom(LP3950_address, 1);    // request 2 bytes from slave device #112
     
      while(Wire.available())    // slave may write less than requested
      { 
        reading = Wire.read();
      } 
      
      return reading;

}

void readtest()
{
      Serial.print("0x00: 0x");
      Serial.println( read_reg( 0x00 ), HEX);
      Serial.print("0x01: 0x");
      Serial.println( read_reg( 0x01 ), HEX);
      Serial.print("0x02: 0x");
      Serial.println( read_reg( 0x02 ), HEX);
      Serial.print("0x03: 0x");
      Serial.println( read_reg( 0x03 ), HEX);
      Serial.print("0x04: 0x");
      Serial.println( read_reg( 0x04 ), HEX);
      Serial.print("0x05: 0x");
      Serial.println( read_reg( 0x05 ), HEX);
      Serial.print("0x06: 0x");
      Serial.println( read_reg( 0x06 ), HEX);
      Serial.print("0x07: 0x");
      Serial.println( read_reg( 0x07 ), HEX);
      Serial.print("0x0B: 0x");
      Serial.println( read_reg( 0x0B ), HEX);
      Serial.print("0x0C: 0x");
      Serial.println( read_reg( 0x0C ), HEX);
      Serial.print("0x0D: 0x");
      Serial.println( read_reg( 0x0D ), HEX);
      Serial.print("0x2A: 0x");
      Serial.println( read_reg( 0x2A ), HEX);
      Serial.print("0x2B: 0x");
      Serial.println( read_reg( 0x2B ), HEX);
}
