/* AudioBoard.h
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#ifndef __AUDIO_BOARD_H
#define __AUDIO_BOARD_H

#include "Utilities.h"

//****************TUNABLE SYSTEM PARAMTERS************************
//all the other art cars might need different values.

// OBSOLETE: only consumed by Noisefloor_Compensate() below, which is never
// called anywhere in this codebase (verified -- no call sites exist outside
// its own definition). Superseded by the percent-based, time-gated silence
// detection in updateAutoGainAndSilence() / setNoiseFloorPercent().
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
#define ARR 8       // OBSOLETE: only used by Normalize() below, which is never called -- see its comment
#define SGAIN 25    // OBSOLETE: only used by Normalize() below, which is never called -- see its comment

#define NORMALIZED_AUDIO 0

inline int scale( int x ) { return ( ( 255 * x ) / 1023 ); }

class AudioBoard {
 public:
   //Define spectrum variables
   static int Frequencies_Mono[7];
   static int bin_low, bin_mid, bin_high;

   static void Read_Frequencies(){
      //Read frequencies for each band
      // STROBE/RESET are inverted by hardware between the Due and the chip
      // (an inverting buffer sits in between) -- every HIGH/LOW below is
      // deliberately the opposite of the chip's own datasheet levels, so
      // that what actually reaches the chip is correct. See also setup().
      for ( int freq_amp = 0; freq_amp < 7 ; ++freq_amp ) {
        Frequencies_Mono[freq_amp] = analogRead(DC_One);
        delayMicroseconds( 15 );
        digitalWrite(STROBE, HIGH); //toggle pin of spectrum shield to get next bin value -- inverted: chip actually sees LOW here
        delayMicroseconds( 100 );
        digitalWrite(STROBE, LOW); // inverted: chip actually sees HIGH here
        delayMicroseconds( 85 ); // datasheet min for read after hi edge on strobe is 63-67us
      }
   }

   static void Into_3_Bins(){
     //amalgamate into 3 bins by averaging
     // bin_low = ((Frequencies_Mono[0]+ ( Frequencies_Mono[1] * 0.3 ) )/1.3);
     // bin_mid = ((Frequencies_Mono[3]+Frequencies_Mono[4])/2);
     // bin_high = ((Frequencies_Mono[5]+Frequencies_Mono[6])/2);
   
   
   /*
     //amalgamate into 3 bins by taking MAX value
     bin_low = max (Frequencies_Mono[0], Frequencies_Mono[1]);
     bin_low = max (bin_low, Frequencies_Mono[2]);
     bin_mid = max (Frequencies_Mono[3], Frequencies_Mono[4]);
     bin_high = max (Frequencies_Mono[5], Frequencies_Mono[6]);
   */
   
   //Don's test to try to eliminate "randomness" from mixing mulitple signals
     //amalgamate into 3 bins by taking the following frequencies and ignoring others
   
   
     bin_low = Frequencies_Mono[1];
     bin_mid = Frequencies_Mono[3];
     bin_high = Frequencies_Mono[6];
     
   }


   //analogRead returns 0-1023, and analogWrite works 0-255. Make sure there's no rollover.
   static void Clipping_Basic() {
      if (NORMALIZED_AUDIO) {
        // normalize - there's always some noise
        const int nmin = 30;
        const int nmax = 800;
        // map the operational range into the output
        bin_low = ((int64_t)(bin_low - nmin) * (255)) / (nmax - nmin);
        bin_mid = ((int64_t)(bin_mid - nmin) * (255)) / (nmax - nmin);
        bin_high = ((int64_t)(bin_high - nmin) * (255)) / (nmax - nmin);
      } else {
        bin_low = scale(bin_low);
        bin_mid = scale(bin_mid);
        bin_high = scale( bin_high );
      }
   }

   // OBSOLETE -- never called anywhere in this codebase (verified: no call
   // sites exist outside this definition). A flat per-band cutoff against the
   // hardcoded 0-255 NOISE_FLOOR_LOW/MID/HIGH constants above. Superseded by
   // the percent-based, 4-second-continuous silence detection in
   // updateAutoGainAndSilence() (see getLow()/getMid()/getHigh(), which
   // already silence-gate via that newer mechanism instead). Left in place
   // for reference; safe to delete.
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
   //
   // OBSOLETE -- never called anywhere in this codebase (verified: no call
   // sites exist outside this definition). This is an earlier attempt at
   // auto-gain control: a threshold-based integrator that nudges 3 separate
   // per-band gains (low_gain/mid_gain/high_gain, plus a combined `gain`) up
   // or down by 1 per poll, based on whether an 8-sample rolling average
   // (ARR) is above/below fixed 100/200 thresholds -- and it operates on
   // bin_low/bin_mid/bin_high (the 3 curated VU-meter bins).
   //
   // The LIVE auto-gain system (updateAutoGainAndSilence()/getFullSpectrum(),
   // below) is a different design operating on a different signal: it uses a
   // true rolling-4-second-peak ratio (255/rollingPeak_) rather than a
   // threshold integrator, and applies to rawFullSpectrum() (the max of all 7
   // raw bins), not bin_low/mid/high. Because they touch different variables,
   // simply re-enabling this function wouldn't corrupt the live system's
   // state -- but it WOULD reintroduce a second, independently-designed AGC
   // scheme running in parallel, covering a different signal than the live
   // one. Don't re-enable this without accounting for that overlap. Left in
   // place for reference; safe to delete.
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

   /* --- full-spectrum level, rolling peak / auto-gain, rolling average /
    * noise-floor silence detection ---
    *
    * "Full spectrum" = the max of all 7 raw hardware bins (not just the 3
    * curated into bin_low/mid/high), scaled to 0-255.
    *
    * Auto-gain: a rolling max over the trailing 4s (rollingPeak_, recomputed
    * from a timestamped ring buffer every poll) sets a slow-moving gain
    * (255/rollingPeak_) so quiet passages still reach full scale. Applying
    * that gain to a sample louder than anything seen in the last 4s would
    * clip above 255 -- clamping the result to 255 is exactly "reduce the
    * scaling factor so this instant peaks at the max value", just computed
    * as a clamp rather than by solving for an adjusted per-sample gain (same
    * result).
    *
    * Silence detection: a separate EMA average (also ~4s time constant)
    * tracks a running level. If that average stays below the noise floor for
    * a full 4s straight, audio is considered off and every level getter
    * (getLow/getMid/getHigh/getFullSpectrum) returns 0 until the average
    * rises back above the floor (which un-silences immediately, no delay).
    * The noise floor is compared against the RAW average, not the gained
    * value, so auto-gain can never mask silence.
    *
    * This is the ONLY active gain-control/silence-detection logic in this
    * file. Normalize() and Noisefloor_Compensate() (above) are an earlier,
    * unrelated attempt at the same general goal -- both are obsolete and
    * never called; see their own comments for why they don't conflict with
    * this one.
    */
   static const uint16_t PEAK_WINDOW_MS = 4000;
   static const uint8_t PEAK_BUFFER_SIZE = 150; // ~4s of samples at the ~30ms poll rate, with margin

   static uint8_t peakSamples_[ PEAK_BUFFER_SIZE ];
   static uint32_t peakTimestamps_[ PEAK_BUFFER_SIZE ];
   static uint8_t peakHead_;
   static uint8_t peakCount_;
   static uint8_t rollingPeak_;

   static bool autoGainEnabled_;
   static float noiseFloorPercent_; // 0-100

   static float emaAverage_;
   static uint32_t lastPollTime_;
   static Timer silenceTimer_; // time spent continuously below the noise floor
   static bool silent_;

   static uint8_t rawFullSpectrum() {
      uint16_t maxVal = 0;
      for ( int i = 0; i < 7; ++i ) {
         if ( (uint16_t)Frequencies_Mono[ i ] > maxVal ) maxVal = Frequencies_Mono[ i ];
      }
      return (uint8_t)scale( maxVal );
   }

   static void updateAutoGainAndSilence( uint32_t nowMs ) {
      uint8_t raw = rawFullSpectrum();

      // rolling peak: sliding 4s window max, via a timestamped ring buffer
      peakSamples_[ peakHead_ ] = raw;
      peakTimestamps_[ peakHead_ ] = nowMs;
      peakHead_ = ( peakHead_ + 1 ) % PEAK_BUFFER_SIZE;
      if ( peakCount_ < PEAK_BUFFER_SIZE ) peakCount_++;
      uint8_t peak = 0;
      for ( uint8_t i = 0; i < peakCount_; ++i ) {
         if ( nowMs - peakTimestamps_[ i ] <= PEAK_WINDOW_MS && peakSamples_[ i ] > peak ) {
            peak = peakSamples_[ i ];
         }
      }
      rollingPeak_ = peak;

      // rolling average via EMA, tuned for a ~4s time constant regardless of
      // the actual (slightly variable) interval between polls
      uint32_t dt = ( lastPollTime_ == 0 ) ? 30 : ( nowMs - lastPollTime_ );
      lastPollTime_ = nowMs;
      float alpha = (float)dt / ( (float)PEAK_WINDOW_MS + (float)dt );
      emaAverage_ = emaAverage_ + alpha * ( (float)raw - emaAverage_ );

      // noise-floor silence detection, on the RAW (un-gained) average
      uint8_t noiseFloorRaw = (uint8_t)( constrain( noiseFloorPercent_, 0.0f, 100.0f ) / 100.0f * 255.0f + 0.5f );
      if ( emaAverage_ >= noiseFloorRaw ) {
         silenceTimer_.reset();
         silent_ = false;
      } else if ( silenceTimer_.elapsed() >= PEAK_WINDOW_MS ) {
         silent_ = true;
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
        updateAutoGainAndSilence( time );
     }
   }

   static uint8_t getLow( int threshold = 255 ) {
      if ( silent_ ) return 0;
      return bin_low < threshold ? bin_low : 0;
   }

   static uint8_t getMid( int threshold = 255 ) {
      if ( silent_ ) return 0;
      return bin_mid < threshold ? bin_mid : 0;
   }

   static uint8_t getHigh( int threshold = 255 ) {
      if ( silent_ ) return 0;
      return bin_high < threshold ? bin_high : 0;
   }

   // "current full-spectrum audio level" -- silence-gated, and auto-gained if
   // enabled (raw otherwise). This is what drives the audio-screen's megabar
   // glow, and is the intended general-purpose "how loud is it right now" call.
   static uint8_t getFullSpectrum() {
      return getFullSpectrum( autoGainEnabled_ );
   }

   // same, but with auto-gain explicitly overridden rather than using the
   // committed autoGainEnabled_ flag -- lets the config screen preview what
   // auto-gain (enabled or disabled) would look like without mutating any
   // committed state, so cancelling a live preview needs no revert step.
   static uint8_t getFullSpectrum( bool useAutoGain ) {
      if ( silent_ ) return 0;
      uint8_t raw = rawFullSpectrum();
      if ( !useAutoGain || rollingPeak_ == 0 ) return raw;
      float gained = (float)raw * ( 255.0f / (float)rollingPeak_ );
      if ( gained > 255.0f ) gained = 255.0f; // clip prevention, see comment above
      return (uint8_t)( gained + 0.5f );
   }

   static void setAutoGainEnabled( bool enabled ) {
      autoGainEnabled_ = enabled;
   }
   static bool getAutoGainEnabled() {
      return autoGainEnabled_;
   }

   static void setNoiseFloorPercent( float percent ) {
      noiseFloorPercent_ = constrain( percent, 0.0f, 100.0f );
   }
   static float getNoiseFloorPercent() {
      return noiseFloorPercent_;
   }

   static void setup() {
      //Set spectrum Shield pin configurations
      // STROBE/RESET are inverted by hardware between the Due and the chip
      // (an inverting buffer sits in between), so every level below is
      // deliberately the opposite of the chip's own datasheet/reset-sequence
      // levels -- what actually reaches the chip is correct.
      pinMode(STROBE, OUTPUT);
      pinMode(RESET, OUTPUT);
      pinMode(DC_One, INPUT);
      digitalWrite(STROBE, LOW);  // inverted: chip actually sees HIGH here
      digitalWrite(RESET, LOW);   // inverted: chip actually sees HIGH here

      //Initialize Spectrum Analyzers
      digitalWrite(STROBE, LOW);  // inverted: chip actually sees HIGH here
      delayMicroseconds(100);
      digitalWrite(RESET, HIGH);  // inverted: chip actually sees LOW here
      delayMicroseconds(100);
      digitalWrite(STROBE, HIGH); // inverted: chip actually sees LOW here
      delayMicroseconds(100);
      digitalWrite(STROBE, LOW);  // inverted: chip actually sees HIGH here
      delayMicroseconds(100);
      digitalWrite(RESET, LOW);   // inverted: chip actually sees HIGH here
   }
};

int AudioBoard::Frequencies_Mono[7];
int AudioBoard::bin_low = 0;
int AudioBoard::bin_mid = 0;
int AudioBoard::bin_high = 0;

uint8_t AudioBoard::peakSamples_[ AudioBoard::PEAK_BUFFER_SIZE ];
uint32_t AudioBoard::peakTimestamps_[ AudioBoard::PEAK_BUFFER_SIZE ];
uint8_t AudioBoard::peakHead_ = 0;
uint8_t AudioBoard::peakCount_ = 0;
uint8_t AudioBoard::rollingPeak_ = 0;
bool AudioBoard::autoGainEnabled_ = false;
float AudioBoard::noiseFloorPercent_ = 0.0f;
float AudioBoard::emaAverage_ = 0.0f;
uint32_t AudioBoard::lastPollTime_ = 0;
Timer AudioBoard::silenceTimer_;
bool AudioBoard::silent_ = false;

#endif
