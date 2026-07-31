/* AudioBoard.h
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#ifndef __AUDIO_BOARD_H
#define __AUDIO_BOARD_H

#include "Utilities.h"
#include <math.h>

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
inline uint8_t rawToPercent( uint8_t raw ) { return (uint8_t)( ( (uint16_t)raw * 100 + 127 ) / 255 ); }

// which band a getter call is asking about -- see AudioBoard's raw/normal/
// hit getters below. BandFull is the overall full-spectrum signal (max of
// all 7 raw hardware bins, same source as the older getFullSpectrum()), not
// one of the 3 curated VU-meter bins -- it's also every getter's default
// when called with no argument at all.
enum AudioBand { BandLow, BandMid, BandHigh, BandFull };
static const uint8_t NUM_AUDIO_BANDS = 4;

class AudioBoard {
 public:
   //Define spectrum variables
   static int Frequencies_Mono[7];
   static int bin_low, bin_mid, bin_high;

   // shared "this counts as a hit" level -- EqualizerShow's bass strobe
   // trigger and the Audio config screen's per-bin peak flash both compare
   // against this same live-adjustable value (getPeakThresholdRaw()), so
   // they agree on what counts as a peak. Live-adjustable + persisted, same
   // as the noise floor (see setPeakThresholdPercent()/getPeakThresholdRaw()
   // below) -- default approximates the old hardcoded 80/255.

   // --- per-band raw/normal/hit tracking ---
   //
   // Three values are kept per band (low/mid/high), each 0-255 internally,
   // exposed only as percent (0-100) or a nonzero/bool test to code outside
   // this file:
   //   raw    -- bin_low/mid/high straight from Into_3_Bins()+Clipping_Basic(),
   //             before any adjustment at all.
   //   normal -- raw after silence-squelch (0 while AudioBoard::silent_) and
   //             auto-gain (255/rollingPeak_, when enabled) -- same
   //             adjustments getFullSpectrum() already applies, now also
   //             applied per-band instead of only to the full-spectrum sum.
   //   hit    -- a peak-hold derived from normal: while normal is at or
   //             above the live peak threshold, hit tracks it directly
   //             (jumps instantly, even back down, as long as it stays at
   //             or above threshold). The instant normal drops below
   //             threshold, hit ignores normal entirely and decays on its
   //             own at a fixed, time-based rate (hitDecayMs_ -- ms to
   //             decay from full (255) to 0, adjustable, default 300ms),
   //             until either it reaches 0 or normal rises back above
   //             threshold. Because the decay is computed from real
   //             elapsed wall-clock time (see updateHitTracking()) rather
   //             than "how many polls have happened," the decay-rate
   //             SETTING'S timing is independent of how often
   //             pollFrequencies() actually runs -- polling faster or
   //             slower changes how fresh/granular the underlying raw
   //             signal is, but not how many milliseconds hit takes to
   //             decay from max to zero. (The MSGEQ7 chip's own analog
   //             sample-and-hold does have its own per-strobe decay
   //             characteristic, which this can't see or compensate for --
   //             that would affect how accurately raw/normal track the
   //             true instantaneous level at a given poll rate, but not
   //             hit's own decay timing, which never looks at poll count.)
   static uint8_t rawLevel_[ NUM_AUDIO_BANDS ];
   static uint8_t normalLevel_[ NUM_AUDIO_BANDS ];
   static float hitLevel_[ NUM_AUDIO_BANDS ]; // float for sub-integer decay precision; read back rounded
   static float hitDecayMs_;    // ms to decay from 255 to 0, adjustable + persisted, default 300
   static Timer hitTimer_;      // tracks dt between updateHitTracking() calls

   // edge-triggered "a new hit just started" flag per band -- set the instant
   // a band crosses from at-or-below to above the peak threshold, consumed
   // (cleared) the next time that band's newHit() is called. See that
   // function below. Also times out on its own (NEW_HIT_TIMEOUT_MS)
   // if nobody calls the corresponding getter in time -- otherwise a hit
   // nobody asked about yet could sit pending indefinitely and then
   // erroneously report as "just happened" much later, whenever the caller
   // finally does poll it.
   static const uint32_t NEW_HIT_TIMEOUT_MS = 1000;
   static bool hitWasAbove_[ NUM_AUDIO_BANDS ];
   static bool newHitPending_[ NUM_AUDIO_BANDS ];
   static Timer newHitPendingTimer_[ NUM_AUDIO_BANDS ];

   static void updateHitTracking() {
      uint32_t dtMs = hitTimer_.elapsed();
      hitTimer_.reset();
      uint8_t peakRaw = getPeakThresholdRaw();
      float decayAmount = ( 255.0f / hitDecayMs_ ) * (float)dtMs;
      for ( int b = 0; b < NUM_AUDIO_BANDS; ++b ) {
         bool isAbove = normalLevel_[ b ] >= peakRaw;
         if ( isAbove ) {
            hitLevel_[ b ] = normalLevel_[ b ];
         } else {
            hitLevel_[ b ] -= decayAmount;
            if ( hitLevel_[ b ] < 0.0f ) hitLevel_[ b ] = 0.0f;
         }
         if ( isAbove && !hitWasAbove_[ b ] ) {
            newHitPending_[ b ] = true;
            newHitPendingTimer_[ b ].reset();
         }
         hitWasAbove_[ b ] = isAbove;
         if ( newHitPending_[ b ] && newHitPendingTimer_[ b ].elapsed() >= NEW_HIT_TIMEOUT_MS ) {
            newHitPending_[ b ] = false;
         }
      }
   }

   // returns true if this band has crossed above the peak threshold since
   // the last time this same function was called, false otherwise -- a
   // one-shot, consumed-on-read edge trigger (same "ask once, get it once"
   // convention as Button::shortpress()/etc.), not a level test.
   // band defaults to BandFull (the overall spectrum) when called with no argument
   static bool newHit( AudioBand band = BandFull ) { bool v = newHitPending_[ band ]; newHitPending_[ band ] = false; return v; }

   static uint8_t getRawPercent( AudioBand band = BandFull ) { return rawToPercent( rawLevel_[ band ] ); }
   static bool getRawNonzero( AudioBand band = BandFull ) { return rawLevel_[ band ] > 0; }
   static uint8_t getNormalPercent( AudioBand band = BandFull ) { return rawToPercent( normalLevel_[ band ] ); }
   static bool getNormalNonzero( AudioBand band = BandFull ) { return normalLevel_[ band ] > 0; }
   static uint8_t getHitPercent( AudioBand band = BandFull ) { return rawToPercent( (uint8_t)( hitLevel_[ band ] + 0.5f ) ); }
   // true whenever this band's hit level is currently above the live peak
   // threshold, false otherwise -- NOT a plain nonzero test (hit stays
   // nonzero throughout its whole decay tail after dropping back below
   // threshold, which shouldn't still read as "a hit" once it's decayed
   // past the same crossing point everything else here uses).
   static bool getHitNonzero( AudioBand band = BandFull ) { return hitLevel_[ band ] > getPeakThresholdRaw(); }

   // RMS ("root mean square") combination of all 7 raw hardware bins -- an
   // alternative "how loud overall" reading to BandFull's max-of-7 (which
   // tracks whichever single band is loudest right now, favoring
   // transients). RMS instead represents overall acoustic energy across the
   // whole spectrum: broadband/noisy sound reads louder here than a single
   // narrow tone at the same peak, which max() alone can't distinguish.
   // Deliberately NOT polled/tracked every frame like raw/normal/hit above
   // -- sqrtf() is real cost on this FPU-less board, so it's only computed
   // fresh from Frequencies_Mono[] the instant one of these is actually
   // called, not paid for on every poll whether anyone's asking or not.
   static uint8_t getRmsPercent() {
      uint32_t sumSquares = 0;
      for ( int i = 0; i < 7; ++i ) {
         uint8_t v = (uint8_t)scale( Frequencies_Mono[ i ] );
         sumSquares += (uint32_t)v * (uint32_t)v;
      }
      float rms = sqrtf( (float)sumSquares / 7.0f );
      return rawToPercent( (uint8_t)( rms + 0.5f ) );
   }
   static bool getRmsNonzero() {
      for ( int i = 0; i < 7; ++i ) {
         if ( Frequencies_Mono[ i ] > 0 ) return true;
      }
      return false;
   }

   // adjustable range is 0ms (instant decay) to 1000ms (1 full second) --
   // per request, set live via the Audio config screen's decay-rate
   // subsetting, pot-adjusted. 0ms still behaves safely in
   // updateHitTracking() above: 255.0f/0.0f is IEEE-754 +infinity, which
   // immediately drives hitLevel_ to (and clamped at) 0 on the very next
   // update, i.e. genuinely instant decay, no divide-by-zero fault.
   static void setHitDecayMs( float ms ) { hitDecayMs_ = constrain( ms, 0.0f, 1000.0f ); }
   static float getHitDecayMs() { return hitDecayMs_; }

   // 0-1000ms, adjustable + persisted, default 0 (freshest sample, maximal
   // "predictive" lead) -- see the foresight block above for what this does.
   static void setAudioForesightMs( float ms ) { audioForesightMs_ = constrain( ms, 0.0f, (float)FORESIGHT_BUFFER_MS ); }
   static float getAudioForesightMs() { return audioForesightMs_; }

   // NOT tied to POLL_INTERVAL_MS above, deliberately -- the 200us delays
   // below are the MSGEQ7's own datasheet-mandated minimum STROBE hold/
   // settle time (63-67us minimum, per the comment below; 200us is margin
   // above that), a hardware protocol requirement independent of how often
   // we choose to call this function. Coupling them to the poll interval
   // would be wrong: shrinking POLL_INTERVAL_MS to poll faster would also
   // shrink these below the chip's minimum, corrupting every bin read.
   // POLL_INTERVAL_MS only needs to stay >= this function's own real
   // execution time (7 bins x ~400us of strobe delay alone, so >=~2.8ms
   // before analogRead() overhead) for the poll gate to mean anything --
   // true today by a wide margin (30ms), but worth keeping in mind if
   // POLL_INTERVAL_MS is ever tuned down.
   static void Read_Frequencies(){
      //Read frequencies for each band
      // STROBE/RESET are inverted by hardware between the Due and the chip
      // (an inverting buffer sits in between) -- every HIGH/LOW below is
      // deliberately the opposite of the chip's own datasheet levels, so
      // that what actually reaches the chip is correct. See also setup().
      for ( int freq_amp = 0; freq_amp < 7 ; ++freq_amp ) {
        Frequencies_Mono[freq_amp] = analogRead(DC_One);
        digitalWrite(STROBE, HIGH); //toggle pin of spectrum shield to get next bin value -- inverted: chip actually sees LOW here
        delayMicroseconds( 200 );
        digitalWrite(STROBE, LOW); // inverted: chip actually sees HIGH here
        delayMicroseconds( 200 ); // datasheet min for read after hi edge on strobe is 63-67us
      }
   }

   static void Into_3_Bins(){
     // MSGEQ7's 7 bins have fixed datasheet center frequencies: 63, 160,
     // 400, 1000, 2500, 6250, 16000 Hz (bins 0-6 respectively -- see
     // BIN_FREQ_LABEL in CarpetLightLogic.cpp, used for the noise-floor
     // config screen's live bin readout). Mapped to bass/mid/treble by the
     // requested crossover points -- bass: 0-85Hz, mid: 85Hz-3.8kHz,
     // treble: 3.8kHz+:
     //   bin0 (63Hz)                        -> bass
     //   bin1,2,3,4 (160/400/1000/2500Hz)   -> mid
     //   bin5,6 (6250/16000Hz)              -> treble
     // Only bin0 falls below 85Hz, so bass is necessarily just that one bin
     // -- MSGEQ7's own bin spacing doesn't offer a second bin under 85Hz to
     // average with. Mid and treble now genuinely combine multiple bins
     // (averaged) instead of each picking one arbitrary representative bin
     // and discarding the rest, as the code previously did.
     //
     // UPDATED per request: bass is bin0 alone (unchanged); mid is the
     // LARGER of bins 2/3 (400Hz/1000Hz) rather than an average of 4 bins;
     // treble is the LARGER of bins 5/6 (6250Hz/16000Hz) rather than their
     // average. max() preserves whichever sub-band actually has energy
     // right now instead of diluting it against a quieter neighbor.
     bin_low = Frequencies_Mono[0];
     bin_mid = max( Frequencies_Mono[2], Frequencies_Mono[3] );
     bin_high = max( Frequencies_Mono[5], Frequencies_Mono[6] );
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
   static float peakThresholdPercent_; // 0-100

   static float emaAverage_;
   static Timer pollTimer_; // tracks dt between updateAutoGainAndSilence() calls
   static Timer silenceTimer_; // time spent continuously below the noise floor
   static bool silent_;

   static uint8_t rawFullSpectrum() {
      uint16_t maxVal = 0;
      for ( int i = 0; i < 7; ++i ) {
         if ( (uint16_t)Frequencies_Mono[ i ] > maxVal ) maxVal = Frequencies_Mono[ i ];
      }
      return (uint8_t)scale( maxVal );
   }

   // --- audio foresight ---
   //
   // The MSGEQ7/DSP pipeline "sees" a waveform before that same sound
   // actually reaches the speakers -- audioForesightMs_ is how many
   // milliseconds ahead of the audible output our reading currently is.
   // Every band's raw sample is buffered into a short per-band ring buffer
   // (history, timestamped) as it's measured; what pollFrequencies() then
   // exposes as rawLevel_[band] is looked up from that history at
   // "now - audioForesightMs_" rather than the just-measured value directly
   // -- i.e. holding our own reaction back so it lines up with when the
   // sound is actually heard, instead of reacting the instant we merely see
   // it. Default 0ms (use the freshest sample) keeps the natural "lights
   // react slightly before the beat is heard" effect that comes for free
   // from the DSP's own inherent look-ahead; dialing foresight UP trades
   // that predictive lead away for tighter sync with the true acoustic
   // output. Only the exposed raw/normal/hit values are delayed this way --
   // updateAutoGainAndSilence()'s own internal AGC/silence tracking
   // (rollingPeak_/emaAverage_/silent_ above) deliberately keeps working off
   // the true instantaneous signal, since that's a control loop, not
   // something a person perceives directly, and delaying it too would just
   // add unnecessary lag to the gain/silence response.
   // POLL_INTERVAL_MS is the single source of truth for how often
   // pollFrequencies()/pollSimulated() actually read a fresh sample (see
   // their own "> POLL_INTERVAL_MS" gates below) -- FORESIGHT_BUFFER_SIZE is
   // *derived* from it (ceiling division, so the buffer always spans at
   // least FORESIGHT_BUFFER_MS regardless of what POLL_INTERVAL_MS is set
   // to, plus a small fixed margin for poll-timing jitter) rather than
   // hardcoded to a value that assumes today's 30ms. If the poll rate is
   // ever changed, the buffer resizes itself accordingly at compile time --
   // nothing here needs to be hand-retuned to match.
   static const uint16_t POLL_INTERVAL_MS = 30;
   static const uint16_t FORESIGHT_BUFFER_MS = 1000; // must cover the full 0-1000ms adjustable range
   static const uint16_t FORESIGHT_BUFFER_SIZE = ( FORESIGHT_BUFFER_MS + POLL_INTERVAL_MS - 1 ) / POLL_INTERVAL_MS + 2;
   static uint8_t rawHistory_[ NUM_AUDIO_BANDS ][ FORESIGHT_BUFFER_SIZE ]; // band-major; all 4 bands share one timestamp array below since they're always sampled together
   static uint32_t historyTimestamps_[ FORESIGHT_BUFFER_SIZE ];
   static uint16_t historyHead_;
   static uint16_t historyCount_;
   static float audioForesightMs_; // 0-1000, adjustable + persisted, default 0

   // pushes this poll's just-measured raw samples (one per band) into the
   // ring buffer
   static void pushRawHistory( uint8_t freshLow, uint8_t freshMid, uint8_t freshHigh, uint8_t freshFull, uint32_t nowMs ) {
      rawHistory_[ BandLow ][ historyHead_ ] = freshLow;
      rawHistory_[ BandMid ][ historyHead_ ] = freshMid;
      rawHistory_[ BandHigh ][ historyHead_ ] = freshHigh;
      rawHistory_[ BandFull ][ historyHead_ ] = freshFull;
      historyTimestamps_[ historyHead_ ] = nowMs;
      historyHead_ = ( historyHead_ + 1 ) % FORESIGHT_BUFFER_SIZE;
      if ( historyCount_ < FORESIGHT_BUFFER_SIZE ) historyCount_++;
   }

   // returns whichever buffered sample's timestamp is closest to
   // "nowMs - audioForesightMs_" -- a linear scan, same style as the
   // windowed-max scan updateAutoGainAndSilence() already does over
   // peakSamples_/peakTimestamps_ above (order within the buffer doesn't
   // matter for either kind of scan, only which timestamp is closest/
   // largest, so no circular-order bookkeeping is needed to read it back)
   static uint8_t lookupDelayed( AudioBand band, uint32_t nowMs ) {
      uint32_t targetMs = ( nowMs > (uint32_t)audioForesightMs_ ) ? ( nowMs - (uint32_t)audioForesightMs_ ) : 0;
      uint16_t bestIdx = 0;
      uint32_t bestDelta = 0xFFFFFFFFu;
      for ( uint16_t i = 0; i < historyCount_; ++i ) {
         uint32_t ts = historyTimestamps_[ i ];
         uint32_t delta = ( ts > targetMs ) ? ( ts - targetMs ) : ( targetMs - ts );
         if ( delta < bestDelta ) { bestDelta = delta; bestIdx = i; }
      }
      return rawHistory_[ band ][ bestIdx ];
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
      uint32_t dt = pollTimer_.elapsed();
      pollTimer_.reset();
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
     if ( time - timestamp > POLL_INTERVAL_MS ) {
        timestamp = time;
        Read_Frequencies();
        Into_3_Bins();
        Clipping_Basic();
        updateAutoGainAndSilence( time );

        // raw: straight from the bins, before any adjustment at all.
        // BandFull is the overall spectrum (max of all 7 raw bins, same
        // source as the older getFullSpectrum()), not one of the 3 curated
        // bins -- computed here so it flows through the exact same
        // silence-squelch/auto-gain/hit treatment as the other 3 bands below.
        //
        // Freshly-measured samples are pushed into the per-band history
        // ring buffer, then rawLevel_[] is read back from that same history
        // at "now - audioForesightMs_" rather than used directly -- see
        // the foresight block above for why.
        pushRawHistory( (uint8_t)bin_low, (uint8_t)bin_mid, (uint8_t)bin_high, rawFullSpectrum(), time );
        rawLevel_[ BandLow ] = lookupDelayed( BandLow, time );
        rawLevel_[ BandMid ] = lookupDelayed( BandMid, time );
        rawLevel_[ BandHigh ] = lookupDelayed( BandHigh, time );
        rawLevel_[ BandFull ] = lookupDelayed( BandFull, time );

        // normal: silence-squelched + auto-gained, same treatment
        // getFullSpectrum() already applies to the full-spectrum sum, now
        // also applied per-band (and to BandFull, above)
        for ( int b = 0; b < NUM_AUDIO_BANDS; ++b ) {
           if ( silent_ ) {
              normalLevel_[ b ] = 0;
              continue;
           }
           float gained = (float)rawLevel_[ b ];
           if ( autoGainEnabled_ && rollingPeak_ > 0 ) {
              gained *= ( 255.0f / (float)rollingPeak_ );
              if ( gained > 255.0f ) gained = 255.0f; // clip prevention, see getFullSpectrum()'s comment
           }
           normalLevel_[ b ] = (uint8_t)( gained + 0.5f );
        }

        updateHitTracking();
     }
   }

   // alternative to pollFrequencies() for the Audio config screen's decay-
   // rate subsetting while showing the simulated signal (see
   // updateSimulatedBand() below) instead of real audio -- feeds the same
   // eased square wave into all 4 bands' raw/normal levels and runs it
   // through the exact same updateHitTracking() real audio uses, so hit's
   // decay behavior being tuned here is the real code path, not a separate
   // mock. Gated at the same ~30ms cadence as pollFrequencies() so the
   // decay-rate setting is exercised under the same real poll-rate the
   // hardware uses. Caller is responsible for calling this INSTEAD OF
   // pollFrequencies() while the simulated signal is active, never both.
   static void pollSimulated( uint32_t time ) {
      static uint32_t timestamp;
      if ( time - timestamp > POLL_INTERVAL_MS ) {
         timestamp = time;
         uint8_t simRaw = updateSimulatedBand();
         rawLevel_[ BandLow ] = rawLevel_[ BandMid ] = rawLevel_[ BandHigh ] = rawLevel_[ BandFull ] = simRaw;
         normalLevel_[ BandLow ] = normalLevel_[ BandMid ] = normalLevel_[ BandHigh ] = normalLevel_[ BandFull ] = simRaw;
         updateHitTracking();
      }
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

   // best-effort estimate, in percent, of the source's own volume-knob
   // position -- NOT "how loud is it right now" (that's getFullSpectrum(),
   // which swings with the music's own dynamics regardless of the knob).
   // Uses the rolling 4s peak (rollingPeak_) rather than the instantaneous
   // level or the EMA average, since the loudest moments in a trailing
   // window track a knob's ceiling far more reliably than any single
   // instant does, and rolling_peak already resets fast enough (4s) to
   // follow an actual knob turn. Necessarily approximate -- a spectrum
   // analyzer chip has no direct visibility into the source's own volume
   // control -- but it's the best signal already available here. Currently
   // unused by anything; exists as a general-purpose accessor for later use.
   static uint8_t getOverallLevelPercent() {
      return (uint8_t)( ( (uint16_t)rollingPeak_ * 100 + 127 ) / 255 );
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
   // same 0-100% -> 0-255 conversion used internally by updateAutoGainAndSilence()
   static uint8_t getNoiseFloorRaw() {
      return (uint8_t)( constrain( noiseFloorPercent_, 0.0f, 100.0f ) / 100.0f * 255.0f + 0.5f );
   }

   static void setPeakThresholdPercent( float percent ) {
      peakThresholdPercent_ = constrain( percent, 0.0f, 100.0f );
   }
   static float getPeakThresholdPercent() {
      return peakThresholdPercent_;
   }
   static uint8_t getPeakThresholdRaw() {
      return (uint8_t)( constrain( peakThresholdPercent_, 0.0f, 100.0f ) / 100.0f * 255.0f + 0.5f );
   }

   // simulated single-band waveform for the Audio config screen's decay-rate
   // subsetting (see CarpetLightLogic.cpp) -- lets the decay behavior be
   // tuned without needing real audio. Alternates high/low phases, holding
   // each phase's duration for 10 seconds before switching between 40ms and
   // 80ms phases. Only advances while actively polled (updateSimulatedBand()
   // must be called every frame the decay-rate screen is showing the
   // simulated signal) -- not run continuously in the background.
   //
   // The waveform's SHAPE (not just its phase duration) depends on which
   // phase length is currently active, both built from the same fixed 40ms
   // raised-cosine transition (SIM_EDGE_MS), applied at the start of every
   // phase, easing from the previous phase's target to the current one:
   //   - 40ms phase: the 40ms transition consumes the ENTIRE phase, so
   //     there's never a flat plateau -- back-to-back rising/falling
   //     raised-cosine halves are mathematically identical to one
   //     continuous sine wave (period = 2x40 = 80ms).
   //   - 80ms phase: the same 40ms transition only fills HALF the phase,
   //     leaving a flat plateau at max (or 0) for the remaining 40ms --
   //     i.e. it rises with a sine-shaped edge, stays at max volume, then
   //     eases back down with the same sine-shaped curvature right around
   //     the next toggle.
   static const uint32_t SIM_EDGE_MS = 40;
   static uint32_t simPhaseMs_;
   static Timer simStateTimer_; // switches simPhaseMs_ between 40/80 every 10s
   static Timer simPhaseTimer_; // toggles high/low within the current simPhaseMs_, and times the ease-in from phase start

   static bool simPhaseHigh_;

   // returns the simulated band's instantaneous level (0-255) and advances
   // the internal waveform state by however much time has passed since the
   // last call
   static uint8_t updateSimulatedBand() {
      if ( simStateTimer_.expireset() ) {
         simPhaseMs_ = ( simPhaseMs_ == 40 ) ? 80 : 40;
      }
      if ( simPhaseTimer_.expired() ) {
         simPhaseHigh_ = !simPhaseHigh_;
         simPhaseTimer_.set( simPhaseMs_ );
         simPhaseTimer_.reset();
      }
      uint8_t currTarget = simPhaseHigh_ ? 255 : 0;
      uint8_t prevTarget = simPhaseHigh_ ? 0 : 255; // square wave always alternates, so the previous phase's target is just the opposite extreme
      uint32_t elapsed = simPhaseTimer_.elapsed();
      if ( elapsed >= SIM_EDGE_MS ) return currTarget;
      float f = (float)elapsed / (float)SIM_EDGE_MS;
      float eased = 0.5f - 0.5f * cosf( PI * f ); // raised-cosine: sine-family curvature, 0 at f=0, 1 at f=1
      return (uint8_t)( (float)prevTarget + ( (float)currTarget - (float)prevTarget ) * eased + 0.5f );
   }

   // resets the simulated square wave to a fresh start -- call on entering
   // the decay-rate subsetting, so it doesn't carry over stale phase/timer
   // state from a previous visit
   static void resetSimulatedBand() {
      simPhaseMs_ = 40;
      simPhaseHigh_ = true;
      simStateTimer_.set( 10000 );
      simStateTimer_.reset();
      simPhaseTimer_.set( simPhaseMs_ );
      simPhaseTimer_.reset();
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
float AudioBoard::peakThresholdPercent_ = 31.0f; // approx. the old hardcoded 80/255, overwritten by Nvm at boot
float AudioBoard::emaAverage_ = 0.0f;
Timer AudioBoard::pollTimer_;
Timer AudioBoard::silenceTimer_;
bool AudioBoard::silent_ = false;

uint8_t AudioBoard::rawLevel_[ NUM_AUDIO_BANDS ] = { 0, 0, 0, 0 };
uint8_t AudioBoard::normalLevel_[ NUM_AUDIO_BANDS ] = { 0, 0, 0, 0 };
float AudioBoard::hitLevel_[ NUM_AUDIO_BANDS ] = { 0.0f, 0.0f, 0.0f, 0.0f };
float AudioBoard::hitDecayMs_ = 300.0f; // overwritten by Nvm at boot
Timer AudioBoard::hitTimer_;
bool AudioBoard::hitWasAbove_[ NUM_AUDIO_BANDS ] = { false, false, false, false };
bool AudioBoard::newHitPending_[ NUM_AUDIO_BANDS ] = { false, false, false, false };
Timer AudioBoard::newHitPendingTimer_[ NUM_AUDIO_BANDS ];

uint8_t AudioBoard::rawHistory_[ NUM_AUDIO_BANDS ][ AudioBoard::FORESIGHT_BUFFER_SIZE ] = { { 0 } };
uint32_t AudioBoard::historyTimestamps_[ AudioBoard::FORESIGHT_BUFFER_SIZE ] = { 0 };
uint16_t AudioBoard::historyHead_ = 0;
uint16_t AudioBoard::historyCount_ = 0;
float AudioBoard::audioForesightMs_ = 0.0f; // overwritten by Nvm at boot

uint32_t AudioBoard::simPhaseMs_ = 40;
Timer AudioBoard::simStateTimer_( 10000 );
Timer AudioBoard::simPhaseTimer_( 40 );
bool AudioBoard::simPhaseHigh_ = true;

#endif
