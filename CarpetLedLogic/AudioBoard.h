/* AudioBoard.h
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#ifndef __AUDIO_BOARD_H
#define __AUDIO_BOARD_H

//****************TUNABLE SYSTEM PARAMTERS************************
//all the other art cars might need different values.

#define NOISE_FLOOR_LOW 75
#define NOISE_FLOOR_MID 75
#define NOISE_FLOOR_HIGH 75

//Declare Spectrum Shield pin connections
#define STROBE 7
#define RESET 6
#define DC_One A0
#define LOW_OUTPUT 9
#define MID_OUTPUT 10
#define HIGH_OUTPUT 11
#define ARR 8
#define SGAIN 25

inline int scale( int x ) { return ( 255 * x ) / 1023; }

class AudioBoard {
 public:
   //Define spectrum variables
   static int Frequencies_Mono[7];
   static int bin_low, bin_mid, bin_high;

   static void Read_Frequencies(){
      //Read frequencies for each band
      //Initialize Spectrum Analyzers
      delay(1);
      digitalWrite(RESET, LOW);
      delay(1);
      digitalWrite(RESET, HIGH);
      delay(1);
      for ( int freq_amp = 0; freq_amp < 7 ; ++freq_amp ) {
        Frequencies_Mono[freq_amp] = analogRead(DC_One);
        digitalWrite(STROBE, LOW); //toggle pin of spectrum shield to get next bin value
         delay(1);
        // delayMicroseconds( 10000 );
        digitalWrite(STROBE, HIGH);
        // delayMicroseconds( 10000 );
         delay(1);
      }
   }

   static void Into_3_Bins(){
     //amalgamate into 3 bins by averaging
     bin_low = ((Frequencies_Mono[0]+Frequencies_Mono[1]+Frequencies_Mono[2])/3);
     bin_mid = ((Frequencies_Mono[3]+Frequencies_Mono[4])/2);
     bin_high = ((Frequencies_Mono[5]+Frequencies_Mono[6])/2);
   
   
   /*
     //amalgamate into 3 bins by taking MAX value
     bin_low = max (Frequencies_Mono[0], Frequencies_Mono[1]);
     bin_low = max (bin_low, Frequencies_Mono[2]);
     bin_mid = max (Frequencies_Mono[3], Frequencies_Mono[4]);
     bin_high = max (Frequencies_Mono[5], Frequencies_Mono[6]);
   */
   
   //Don's test to try to eliminate "randomness" from mixing mulitple signals
   /*
     //amalgamate into 3 bins by taking the following frequencies and ignoring others
   
   
     // we should use a measure for this instead fo prinitnt uncodin
     bin_low = Frequencies_Mono[0];
     bin_mid = Frequencies_Mono[3];
     bin_high = Frequencies_Mono[6];
   */
     
   }


   //analogRead returns 0-1023, and analogWrite works 0-255. Make sure there's no rollover.
   static void Clipping_Basic() {
      bin_low = scale( bin_low );
      bin_mid = scale( bin_mid );
      bin_high = scale( bin_high );
   }

   static void Noisefloor_Compensate(){
   //here at my desk, I see that the 'floor' of each channel is between 60-70. Let's try a simple offset, plus clip so the value is never lower than 0
   /****
     bin_low -= NOISE_FLOOR_LOW;
     bin_mid -= NOISE_FLOOR_MID;
     bin_high -= NOISE_FLOOR_HIGH;
     
     if(bin_low < 0)
       bin_low = 0;
     if(bin_mid < 0)
       bin_mid = 0;
     if(bin_high < 0)
       bin_high = 0;
       //NOTE FORM DON: This algorithm lowers all LED OUTPUTS by the floor values, so max output = MAX - FLOOR, aka 185 instead of 255
       ****/
   
     //Don's proposed code change
     if(bin_low < NOISE_FLOOR_LOW)
       bin_low = 0;
     if(bin_mid < NOISE_FLOOR_MID)
       bin_mid = 0;
     if(bin_high < NOISE_FLOOR_HIGH)
       bin_high = 0;
   }


   //this is the tricky part.
   //without this, the LEDs are mostly-off at low volumes, and mostly-on at high volumes.
   //in practicality, there's an in-between mode that sorta looks best. But it's hard to quantify.
   //perhaps it's having an average target for each output.
   //perhaps it's enforcing that each LED turns all the way off every 5 sec?
   //
   //essentially we're trying to make an AGC (auto gain control) circuit digitally
   //so this discussion may be of use:
   //https://www.dsprelated.com/showthread/comp.dsp/21943-1.php
   //
   //
   //whatever algorithm is used, it should have a few tunable parameters. Maybe 'gain change rate' and 'target amplitude' if it's servoing gain around.
   static void Normalize(){
      static int low_hist[ARR], mid_hist[ARR], high_hist[ARR];
      static int low_average, mid_average, high_average = 0;
      static int counter = 0;
      static int low_gain = SGAIN, mid_gain = SGAIN, high_gain = SGAIN;
      static int gain = SGAIN;

      //this currently does not support normalizing at low volumes (basically impossible for the situation)
      //the "fix" will be to ensure the user inputs high enough volume that noise is overwhelmed by music
      
      //gather recent history
      for(int i = ARR-1; i > 0; i--){
         low_hist[i] = low_hist[i-1];
         mid_hist[i] = mid_hist[i-1];
         high_hist[i] = high_hist[i-1];   
      }
      low_hist[0] = bin_low;
      mid_hist[0] = bin_mid;
      high_hist[0] = bin_high;

      //average in history
      for(int i = ARR-1; i >= 0; i--){
         low_average = low_average + low_hist[i];
         mid_average = mid_average + mid_hist[i];
         high_average = high_average + high_hist[i];  
      }
      low_average = low_average / ARR;
      mid_average = mid_average / ARR;
      high_average = high_average / ARR;

      //use gain on bins
//      bin_low = (bin_low * low_gain) / SGAIN;
//      bin_mid = (bin_mid * mid_gain) / SGAIN;
//      bin_high = (bin_high * high_gain) / SGAIN;
//    
      bin_low = (bin_low * gain) / SGAIN;
      bin_mid = (bin_mid * gain) / SGAIN;
      bin_high = (bin_high * gain) / SGAIN;

      //adjust gain if average is too high and the current note is too high
      //however, if average and current note is too low, set gain back to 100% (rarely the case, unless volume is set to below operating range)
      if(low_average > 200 && bin_low > 200){
        low_gain--;
      }
      else if(low_average < 100 && bin_low < 100){
        low_gain = SGAIN;
      }
      if(mid_average > 200 && bin_mid > 200){
        mid_gain--;
      }
      else if(mid_average < 100 && bin_mid < 100){
        mid_gain = SGAIN;
      }
      if(high_average > 200 && bin_high > 200){
        high_gain--;
      }
      else if(high_average < 100 && bin_high < 100){
        high_gain = SGAIN;
      }

      //the following is an attempt to use one gain instead of three separate
      if(mid_average > 200 && bin_mid > 200){
        gain--;
      }
      else if(mid_average < 100 && bin_mid < 100){
        gain = SGAIN;
      }
  
      //very relaxed upward gain correction, this takes ~ 3 seconds to return from 20% gain to 100%
      counter++;
      if(counter > 30){
        counter = 0;
        low_gain++;
        mid_gain++;
        high_gain++;

        gain++;
      }

      //sanity check gains
      if(low_gain > SGAIN){
        low_gain = SGAIN;
      }
      if(mid_gain > SGAIN){
        mid_gain = SGAIN;
      } 
      if(high_gain > SGAIN){
        high_gain = SGAIN;
      }

      if(gain > SGAIN){
        gain = SGAIN;
      }
   }

 public:

   static void pollFrequencies( uint32_t time ) {
     static uint32_t timestamp;
     // TODO: fiddle around with this value to find the right tuning
     if ( time - timestamp > 30 ) {
        timestamp = time;
        Read_Frequencies();
        Into_3_Bins();
        Clipping_Basic();
        Noisefloor_Compensate();
     }
   }

   static uint8_t getLow() {
      return bin_low;
   }

   static uint8_t getMid() {
      return bin_low;
   }

   static uint8_t getHigh() {
      return bin_low;
   }

   static void setup() {
      //Set spectrum Shield pin configurations
      pinMode(STROBE, OUTPUT);
      pinMode(RESET, OUTPUT);
      pinMode(DC_One, INPUT);
      digitalWrite(STROBE, HIGH);
      digitalWrite(RESET, HIGH);

      //Initialize Spectrum Analyzers
      digitalWrite(STROBE, HIGH);
      delay(1);
      digitalWrite(RESET, LOW);
      delay(1);
      digitalWrite(STROBE, LOW);
      delay(1);
      digitalWrite(STROBE, HIGH);
      delay(1);
      digitalWrite(RESET, HIGH);
   }
};

int AudioBoard::Frequencies_Mono[7];
int AudioBoard::bin_low = 0;
int AudioBoard::bin_mid = 0;
int AudioBoard::bin_high = 0;

#endif
