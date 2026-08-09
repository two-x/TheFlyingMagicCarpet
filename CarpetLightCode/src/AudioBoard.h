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

//Declare Spectrum Shield pin connections
#define STROBE 7
#define RESET 6
#define DC_One A0
#define LOW_OUTPUT 9
#define MID_OUTPUT 10
#define HIGH_OUTPUT 11

inline int scale( int x ) { return ( ( 255 * x ) / ADC_MAX_VALUE ); } // see Utilities.h


inline uint8_t rawToPercent( uint8_t raw ) { return (uint8_t)( ( (uint16_t)raw * 100 + 127 ) / 255 ); }

// which band a getter call is asking about -- see AudioBoard's per-band
// level getters below. BandFull is the overall full-spectrum signal (max of
// all 7 raw hardware bins), not one of the 4 curated sub-bands -- it's also
// every getter's default when called with no argument at all. Each maps to
// a fixed set of the MSGEQ7 chip's own 7 bins (datasheet center frequencies
// 63/160/400/1000/2500/6250/16000Hz, indices 0-6) -- see bandRawBinMax()
// below for exactly which bins feed which band.
enum AudioBand { BandBass = 0, BandMidbass, BandMid, BandTreble, BandFull, NumFreqBands };

// AGC (auto-gain) operating mode -- see updateBandLevels()'s f/g/h steps.
//   AGCoff  -- no gain applied at all, levelFilt == levelClean.
//   AGCband -- each band boosted by its OWN independently-tracked rolling-
//              peak gain.
//   AGCfull -- every band boosted by the SAME gain, derived from BandFull's
//              own rolling peak -- one shared, full-spectrum-derived gain
//              applied uniformly (this was the only AGC behavior that
//              existed before this file's per-band rework).
enum AGCMode { AGCoff = 0, AGCband, AGCfull, NumAGCModes };

// which shape (if any) the hit-prediction lead-up follows -- see
// AudioBoard::computePredictedRamp()/computePulseTrainValue() below.
enum HitPredictionStyle { PredictDisabled = 0, PredictExponential = 1, PredictPulseTrain = 2, PredictTwoPulse = 3 };
static const uint8_t NUM_HIT_PREDICTION_STYLES = 4;

class AudioBoard {
 public:
   //Define spectrum variables
   static int Frequencies_Mono[7];

   // --- per-band level tracking ---
   //
   // Every band keeps 5 values, ALL in percent (0-100) units:
   //   levelRaw   -- straight conversion of this band's own bin(s) (max of
   //                 whichever raw hardware bins it covers), no adjustment
   //                 at all. Ungated even by soundReactivityEnabled_ --
   //                 diagnostic screens should still show real signal with
   //                 reactivity globally off.
   //   levelClean -- levelRaw, floored to 0 if at or below the live noise
   //                 floor.
   //   levelFilt  -- levelClean, optionally AGC-boosted per the live
   //                 AGCMode (see updateBandLevels() below) -- this is what
   //                 every light show actually reacts to as "the current
   //                 level."
   //   levelHit   -- peak-hold derived from levelFilt: jumps to a flat 100
   //                 the instant levelFilt reaches the live peak threshold,
   //                 stays at 100 for as long as it does, then decays
   //                 (time-based, hitDecayMs_) once levelFilt drops back
   //                 below threshold -- snapping the rest of the way to 0
   //                 as soon as the decaying value rejoins levelFilt, rather
   //                 than lingering below it. Predictive lead-up (see
   //                 computePredictedRamp()) can also push this up ahead of
   //                 an actual crossing, peaking at 100.
   //   levelRMS   -- root-mean-square of this band's own levelClean history
   //                 over the trailing PEAK_WINDOW_MS -- overall acoustic
   //                 energy in that band, as opposed to levelFilt's
   //                 instantaneous reading.
   struct BandLevels {
      float levelRaw = 0.0f;
      float levelClean = 0.0f;
      float levelFilt = 0.0f;
      float levelHit = 0.0f;
      float levelRMS = 0.0f;
   };
   static BandLevels bandLevels_[ NumFreqBands ];

   static float hitDecayMs_;    // ms to decay from 100 to 0, adjustable + persisted, default 300
   static Timer hitTimer_;      // tracks dt between updateBandLevels() calls
   static bool simModeActive_;  // set true/false at the top of pollSimulated()/pollFrequencies() respectively

   // edge-triggered "this band just started a new hit" flag -- one per band,
   // set when THAT band crosses from at-or-below to above the peak threshold
   // during updateBandLevels(), consumed (cleared) by the next NewHit(band)
   // call for that same band. Independent per band -- a bass hit doesn't
   // consume/clear treble's own pending flag or vice versa.
   static bool newHit_[ NumFreqBands ];
   static bool hitWasAbove_[ NumFreqBands ]; // per-band edge-detection state feeding newHit_ above

   // returns (and clears) whether THIS band has had a new hit since the last
   // time this was called for it.
   static bool NewHit( AudioBand band = BandFull ) {
      bool v = newHit_[ band ];
      newHit_[ band ] = false;
      return v;
   }

   // 0-audioForesightMs_ (dynamic range -- can't predict further ahead than
   // the foresight lookahead actually gives us data for), adjustable +
   // persisted, default 0 (no predictive lead-up at all). See
   // computePredictedRamp() below for the actual math.
   static float hitPredictionMs_;
   // which shape the lead-up follows, adjustable + persisted, default
   // PredictExponential -- see HitPredictionStyle above.
   static uint8_t hitPredictionStyle_;

   // same "now - foresight" clamp used by lookupDelayed() -- shared here so
   // updateBandLevels()'s prediction scan and the raw lookup always agree on
   // exactly which buffered moment counts as "the current display position"
   static uint32_t currentDisplayMs( uint32_t nowMs ) {
      return ( nowMs > (uint32_t)audioForesightMs_ ) ? ( nowMs - (uint32_t)audioForesightMs_ ) : 0;
   }

   // Trapezoidal "rounded square wave": timeIntoPulse 0..widthMs, ramping
   // linearly 0->peak over the first third, flat at peak over the middle
   // third, ramping peak->0 over the last third -- cheap integer math, no
   // hard on/off edges.
   static uint8_t shapePulse( uint32_t timeIntoPulse, uint32_t widthMs, uint8_t peak ) {
      uint32_t rampMs = widthMs / 3;
      if ( timeIntoPulse < rampMs ) return (uint8_t)( ( (uint32_t)peak * timeIntoPulse ) / rampMs );
      if ( timeIntoPulse < widthMs - rampMs ) return peak;
      uint32_t remain = widthMs - timeIntoPulse;
      return (uint8_t)( ( (uint32_t)peak * remain ) / rampMs );
   }

   // PredictPulseTrain style: a drumroll of 30ms-wide pulses (rounded per
   // shapePulse() above, peaking at 100 -- the current (100%) hit-level
   // value, per request, not the live peak-threshold level as this used to)
   // leading up to the predicted hit, fixed to its own 300ms lead-in
   // regardless of hitPredictionMs_ (which only still gates how far ahead a
   // hit can be FOUND at all -- see computePredictedRamp()) -- pulse-to-
   // pulse spacing starts at 90ms and narrows to 30ms (touching, back-to-
   // back) right at the hit. Pulse onset times are generated by a small
   // (~6-iteration) integer loop walking backward from the 300ms mark,
   // recomputing the linearly-interpolated spacing at each step (forward-
   // Euler style) rather than solving the continuous spacing-vs-time
   // relationship analytically (which would need a logarithm) -- cheap, and
   // close enough at this timescale.
   static uint8_t computePulseTrainValue( uint32_t timeUntilHit ) {
      static const uint32_t LEAD_MS = 300;
      static const uint32_t MIN_SPACING_MS = 30;
      static const uint32_t MAX_SPACING_MS = 90;
      static const uint32_t PULSE_WIDTH_MS = 30;
      if ( timeUntilHit > LEAD_MS ) return 0;
      uint32_t onset = LEAD_MS;
      while ( true ) {
         if ( timeUntilHit <= onset && ( onset - timeUntilHit ) < PULSE_WIDTH_MS ) {
            return shapePulse( onset - timeUntilHit, PULSE_WIDTH_MS, 100 );
         }
         if ( onset == 0 ) return 0;
         uint32_t spacing = MIN_SPACING_MS + ( onset * ( MAX_SPACING_MS - MIN_SPACING_MS ) ) / LEAD_MS;
         onset = ( onset >= spacing ) ? ( onset - spacing ) : 0;
      }
   }

   // PredictTwoPulse style: just two 35ms-wide pulses (shapePulse()-rounded,
   // peaking at 100, same as computePulseTrainValue() above), 45ms apart,
   // anchored at min(audioForesightMs_, 400ms) before the predicted hit --
   // capped to the live foresight setting rather than a fixed 400ms, since
   // there's no buffered lookahead data past whatever foresight actually
   // provides.
   static uint8_t computeTwoPulseValue( uint32_t timeUntilHit ) {
      static const uint32_t MAX_LEAD_MS = 400;
      static const uint32_t SPACING_MS = 45;
      static const uint32_t PULSE_WIDTH_MS = 35;
      uint32_t leadMs = (uint32_t)audioForesightMs_;
      if ( leadMs > MAX_LEAD_MS ) leadMs = MAX_LEAD_MS;
      if ( timeUntilHit > leadMs ) return 0;
      uint32_t onsetA = leadMs;
      uint32_t onsetB = ( leadMs >= SPACING_MS ) ? ( leadMs - SPACING_MS ) : 0;
      if ( timeUntilHit <= onsetA && ( onsetA - timeUntilHit ) < PULSE_WIDTH_MS ) {
         return shapePulse( onsetA - timeUntilHit, PULSE_WIDTH_MS, 100 );
      }
      if ( timeUntilHit <= onsetB && ( onsetB - timeUntilHit ) < PULSE_WIDTH_MS ) {
         return shapePulse( onsetB - timeUntilHit, PULSE_WIDTH_MS, 100 );
      }
      return 0;
   }

   // Looks ahead in band's buffered history, from the current display
   // position out to hitPredictionMs_ further into what's already been
   // sampled but not yet displayed (bounded by audioForesightMs_, which is
   // exactly how much "not yet displayed" data actually exists), for the
   // EARLIEST upcoming sample that will cross the peak threshold. If found,
   // shapes the lead-up per hitPredictionStyle_:
   //   PredictDisabled     -- always 0, no lookahead performed at all.
   //   PredictExponential  -- a rising ramp (0-100) as a cheap-integer-math
   //     stand-in for "an exponential curve with a vertical asymptote lined
   //     up exactly on the predicted hit": let timeUntilHit be milliseconds
   //     from the display position to the predicted hit (hitPredictionMs_
   //     down to ~1). ramp = 100*(hitPredictionMs_-timeUntilHit)/timeUntilHit
   //     -- 0 at timeUntilHit==hitPredictionMs_ (the farthest predictive
   //     point), growing without bound as timeUntilHit shrinks toward 0,
   //     clamped to 100 -- a true vertical asymptote at the hit's own
   //     instant, via one subtract/multiply/divide instead of expf().
   //   PredictPulseTrain   -- see computePulseTrainValue() above.
   //   PredictTwoPulse     -- see computeTwoPulseValue() above.
   // Returns 0 if no upcoming hit is found in the window (including always,
   // when hitPredictionMs_ is 0 -- no lookahead requested). Threshold-
   // crossing is tested against the buffered RAW-percent history directly
   // (not a re-derived levelClean/levelFilt for that past instant, which
   // would need the noise-floor/AGC state as it stood back then) -- a
   // reasonable approximation, not an exact replay.
   static uint8_t computePredictedRamp( AudioBand band, uint32_t displayMs ) {
      if ( hitPredictionStyle_ == PredictDisabled || hitPredictionMs_ < 1.0f ) return 0;
      uint32_t hp = (uint32_t)hitPredictionMs_;
      uint32_t windowEnd = displayMs + hp;
      float threshPct = constrain( peakThresholdPercent_, 0.0f, 100.0f );
      uint32_t bestHitMs = 0;
      bool found = false;
      for ( uint16_t i = 0; i < historyCount_; ++i ) {
         uint32_t ts = historyTimestamps_[ i ];
         if ( ts <= displayMs || ts > windowEnd ) continue; // strictly ahead of display position, within the lookahead window
         if ( rawHistory_[ band ][ i ] < threshPct ) continue;
         if ( !found || ts < bestHitMs ) { bestHitMs = ts; found = true; }
      }
      if ( !found ) return 0;
      uint32_t timeUntilHit = bestHitMs - displayMs; // 1..hp
      if ( hitPredictionStyle_ == PredictPulseTrain ) return computePulseTrainValue( timeUntilHit );
      if ( hitPredictionStyle_ == PredictTwoPulse ) return computeTwoPulseValue( timeUntilHit );
      uint32_t ramp = ( 100UL * ( hp - timeUntilHit ) ) / timeUntilHit;
      return (uint8_t)( ramp > 100 ? 100 : ramp );
   }

   static uint8_t getRawPercent( AudioBand band = BandFull ) { return (uint8_t)( bandLevels_[ band ].levelRaw + 0.5f ); }
   static bool getRawNonzero( AudioBand band = BandFull ) { return bandLevels_[ band ].levelRaw > 0.0f; }
   // BUGFIX: these 4 getters are what every light show actually reacts to
   // (raw above stays ungated -- diagnostic screens like the noise-floor
   // bin readout should still show real signal even with reactivity
   // globally off). Gating here, once, centrally, means the global disable
   // (see soundReactivityEnabled_ below) doesn't need to be checked
   // separately inside every single show. (soundReactivityEnabled_ is also
   // baked into levelFilt/levelHit themselves inside updateBandLevels(), so
   // this is a belt-and-suspenders check, not the only place it's applied.)
   static uint8_t getNormalPercent( AudioBand band = BandFull ) { return soundReactivityEnabled_ ? (uint8_t)( bandLevels_[ band ].levelFilt + 0.5f ) : 0; }
   static bool getNormalNonzero( AudioBand band = BandFull ) { return soundReactivityEnabled_ && bandLevels_[ band ].levelFilt > 0.0f; }
   static uint8_t getHitPercent( AudioBand band = BandFull ) { return soundReactivityEnabled_ ? (uint8_t)( bandLevels_[ band ].levelHit + 0.5f ) : 0; }
   // true whenever this band's hit level is currently above the live peak
   // threshold, false otherwise -- NOT a plain nonzero test (hit stays
   // nonzero throughout its decay tail after dropping back below threshold,
   // which shouldn't still read as "a hit" once decayed past that point).
   static bool getHitNonzero( AudioBand band = BandFull ) { return soundReactivityEnabled_ && bandLevels_[ band ].levelHit > peakThresholdPercent_; }
   static uint8_t getRmsPercent( AudioBand band = BandFull ) { return (uint8_t)( bandLevels_[ band ].levelRMS + 0.5f ); }
   static bool getRmsNonzero( AudioBand band = BandFull ) { return bandLevels_[ band ].levelRMS > 0.0f; }

   // adjustable range is 0ms (instant decay) to 1000ms (1 full second) --
   // per request, set live via the Audio config screen's decay-rate
   // subsetting, pot-adjusted. 0ms still behaves safely in
   // updateBandLevels() below: 100.0f/0.0f is IEEE-754 +infinity, which
   // immediately drives levelHit to (and clamped at) 0 on the very next
   // update, i.e. genuinely instant decay, no divide-by-zero fault.
   static void setHitDecayMs( float ms ) { hitDecayMs_ = constrain( ms, 0.0f, 1000.0f ); }
   static float getHitDecayMs() { return hitDecayMs_; }

   // 0-1000ms, adjustable + persisted, default 0 (freshest sample, maximal
   // "predictive" lead) -- see the foresight block above for what this does.
   static void setAudioForesightMs( float ms ) {
      audioForesightMs_ = constrain( ms, 0.0f, (float)FORESIGHT_BUFFER_MS );
      // hitPredictionMs_'s range is capped to audioForesightMs_ (can't
      // predict further ahead than there's buffered-but-undisplayed data
      // for) -- re-clamp it here too in case foresight was just reduced
      // below whatever prediction was previously set to.
      if ( hitPredictionMs_ > audioForesightMs_ ) hitPredictionMs_ = audioForesightMs_;
   }
   static float getAudioForesightMs() { return audioForesightMs_; }

   // 0-audioForesightMs_ (dynamic range, see hitPredictionMs_'s declaration
   // comment), adjustable + persisted, default 0. Live-adjustable from the
   // Audio config screen's hit-prediction subsetting, right alongside hit
   // decay rate -- see the class comment on computePredictedRamp() above.
   static void setHitPredictionMs( float ms ) { hitPredictionMs_ = constrain( ms, 0.0f, audioForesightMs_ ); }
   static float getHitPredictionMs() { return hitPredictionMs_; }

   static void setHitPredictionStyle( uint8_t style ) { hitPredictionStyle_ = ( style >= NUM_HIT_PREDICTION_STYLES ) ? ( NUM_HIT_PREDICTION_STYLES - 1 ) : style; }
   static uint8_t getHitPredictionStyle() { return hitPredictionStyle_; }

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
        // after initial reset sequence, We left  pin STROBE LOW / chip STROBE HIGH 
        digitalWrite(STROBE, HIGH); // chip STROBE LOW  // toggle pin of spectrum shield to get next bin value -- inverted: chip actually sees LOW here
        delayMicroseconds( 70 );    // chip STROBE falling edge to OUT valid  To(min) = 36 us
        Frequencies_Mono[freq_amp] = analogRead(DC_One);
        // delayMicroseconds( 10 ); // just padding to ensure read happens w/i chip strobe high pulse - not required
        digitalWrite(STROBE, LOW);  // chip STROBE HIGH // will read on next chip LOW transition
        delayMicroseconds( 35 );    // chip STROBE high pulse  Ts(min) = 18 us
      }
   }

   // 0-1023 raw ADC reading -> 0-100 percent, in one step (scale() to 0-255
   // then rawToPercent() to 0-100).
   static uint8_t analogToPercent( int adcVal ) { return rawToPercent( (uint8_t)scale( adcVal ) ); }

   // this band's own raw bin maximum (still 0-1023 ADC domain -- see
   // analogToPercent() above for the %-conversion step) -- the fixed bin
   // groupings are per request:
   //   BandBass    -- bin0 (63Hz) alone.
   //   BandMidbass -- bin1 (160Hz) alone -- previously unused by anything.
   //   BandMid     -- larger of bin2/bin3 (400/1000Hz).
   //   BandTreble  -- largest of bin4/bin5/bin6 (2500/6250/16000Hz).
   //   BandFull    -- largest of all 7 bins.
   static int bandRawBinMax( AudioBand band ) {
      switch ( band ) {
         case BandBass:    return Frequencies_Mono[ 0 ];
         case BandMidbass: return Frequencies_Mono[ 1 ];
         case BandMid:     return max( Frequencies_Mono[ 2 ], Frequencies_Mono[ 3 ] );
         case BandTreble:  return max( Frequencies_Mono[ 4 ], max( Frequencies_Mono[ 5 ], Frequencies_Mono[ 6 ] ) );
         default: { // BandFull
            int m = Frequencies_Mono[ 0 ];
            for ( int i = 1; i < 7; ++i ) if ( Frequencies_Mono[ i ] > m ) m = Frequencies_Mono[ i ];
            return m;
         }
      }
   }

   /* --- per-band AGC history + rolling boost multiplier ---
    *
    * One ring buffer PER BAND (agcHistory_, levelClean values, %), sharing
    * one timestamp array (agcHistoryTimestamps_) since every band is always
    * sampled on the same poll -- same space-saving convention the foresight
    * buffer below already uses. agcBoost_ holds each band's most recently
    * calculated multiplier (100/that band's own trailing rolling peak,
    * clamped to never divide by zero) -- see computeAgcBoost_() and
    * updateBandLevels()'s (d)/(e) steps.
    */
   static const uint16_t AGC_HISTORY_SIZE = 150; // ~4s of samples at the ~30ms poll rate, with margin -- same sizing as the old whole-spectrum rollingPeak_ buffer
   static float agcHistory_[ NumFreqBands ][ AGC_HISTORY_SIZE ];
   static uint32_t agcHistoryTimestamps_[ AGC_HISTORY_SIZE ];
   static uint8_t agcHistoryHead_;
   static uint16_t agcHistoryCount_;
   static float agcBoost_[ NumFreqBands ];

   static const uint16_t AGC_WINDOW_MS = 4000;

   static float computeAgcBoost_( AudioBand band ) {
      uint32_t nowMs = agcHistoryTimestamps_[ agcHistoryHead_ ]; // this poll's own timestamp -- see updateBandLevels(), which writes this before calling here
      float peak = 0.0f;
      for ( uint16_t i = 0; i < agcHistoryCount_; ++i ) {
         if ( ( nowMs - agcHistoryTimestamps_[ i ] ) <= AGC_WINDOW_MS && agcHistory_[ band ][ i ] > peak ) peak = agcHistory_[ band ][ i ];
      }
      if ( peak <= 0.0f ) return 1.0f; // nothing to normalize against yet -- no boost, matches the old rollingPeak_==0 behavior
      return 100.0f / peak;
   }

   // RMS of this band's own levelClean history over the same trailing
   // window computeAgcBoost_() uses -- "an appropriate algorithm" per
   // request; sqrtf() here is real cost on this FPU-less board (same
   // concern the old getRmsPercent() was written to avoid by only computing
   // on-demand), but per request this now runs proactively every poll, for
   // every band.
   static float computeBandRms_( AudioBand band ) {
      uint32_t nowMs = agcHistoryTimestamps_[ agcHistoryHead_ ];
      float sumSquares = 0.0f;
      uint16_t n = 0;
      for ( uint16_t i = 0; i < agcHistoryCount_; ++i ) {
         if ( ( nowMs - agcHistoryTimestamps_[ i ] ) > AGC_WINDOW_MS ) continue;
         float v = agcHistory_[ band ][ i ];
         sumSquares += v * v;
         ++n;
      }
      if ( n == 0 ) return 0.0f;
      return sqrtf( sumSquares / (float)n );
   }

   static bool autoGainSuppressed_;
   // Global kill switch for all sound-reactive light behavior -- see
   // getNormalPercent()/getHitPercent()/etc. above, which gate on this
   // centrally so no individual show needs its own check. Default true
   // (on); persisted via Nvm, see setSoundReactivityEnabled().
   static bool soundReactivityEnabled_;
   static uint8_t agcMode_; // AGCoff/AGCband/AGCfull, persisted -- see AGCMode above
   static float noiseFloorPercent_; // 0-100
   static float peakThresholdPercent_; // 0-100

   static void setAgcMode( uint8_t mode ) { agcMode_ = ( mode >= NumAGCModes ) ? ( NumAGCModes - 1 ) : mode; }
   static uint8_t getAgcMode() { return agcMode_; }
   // While set (see CarpetLightLogic.cpp), forces AGCoff behavior for as
   // long as the noise-floor/peak-threshold config subsetting is active,
   // without touching the persisted setting itself -- dialing either of
   // those in against a live auto-gained signal is confusing (the gain
   // constantly renormalizes what you're trying to measure against a fixed
   // floor/threshold). agcHistory_/agcBoost_ keep tracking in the
   // background regardless, so auto-gain picks back up smoothly (no jump)
   // once this is cleared.
   static void setAutoGainSuppressed( bool suppressed ) { autoGainSuppressed_ = suppressed; }

   /* --- Step 6 of the audio-refactor spec: the per-poll per-band pipeline.
    * Runs directly after each fresh set of ADC values are read
    * (Read_Frequencies()), looping over every defined frequency band and,
    * for each: (b) raw %, (c) noise-floor-clean %, (d) push clean into this
    * band's own AGC history ring buffer, (e) recalculate this band's AGC
    * boost multiplier from that history, (f/g/h) derive the filtered level
    * per the live AGCMode, (i) recalculate this band's RMS from its own
    * history, (j) detect a new-hit edge (sets that band's own newHit_
    * flag), (k) update the hit level (100 flat on/through a hit, decaying
    * from 100 once the filtered level drops back below threshold, snapping
    * to 0 once decay rejoins the filtered level, with predictive lead-up
    * blended on top, peaking at 100).
    *
    * All 5 bands are processed in ONE ordered pass (not two separate
    * passes) -- meaning in AGCfull mode, a band processed before BandFull
    * in enum order (Bass/Midbass/Mid/Treble, all < BandFull) reads
    * agcBoost_[BandFull] one poll cycle (~30ms) stale, since BandFull's own
    * boost for THIS poll hasn't been computed yet when they need it. Given
    * ~30ms of lag on a slow-moving rolling-4s-window gain figure, this is
    * negligible in practice and not worth a second pass to avoid.
    */
   static void updateBandLevels( uint32_t nowMs ) {
      uint32_t dtMs = hitTimer_.elapsed();
      hitTimer_.reset();
      float noiseFloorPct = constrain( noiseFloorPercent_, 0.0f, 100.0f );
      float threshPct = constrain( peakThresholdPercent_, 0.0f, 100.0f );
      uint32_t displayMs = currentDisplayMs( nowMs );
      uint8_t effectiveAgcMode = autoGainSuppressed_ ? (uint8_t)AGCoff : agcMode_;

      // this poll's own AGC-history slot, written before the per-band loop
      // so computeAgcBoost_()/computeBandRms_() (which scan the whole
      // buffer, including this slot) see a valid timestamp for it
      agcHistoryTimestamps_[ agcHistoryHead_ ] = nowMs;
      if ( agcHistoryCount_ < AGC_HISTORY_SIZE ) ++agcHistoryCount_;

      for ( int b = 0; b < NumFreqBands; ++b ) {
         BandLevels & bl = bandLevels_[ b ];
         AudioBand band = (AudioBand)b;

         bl.levelRaw = analogToPercent( bandRawBinMax( band ) );                     // b
         bl.levelClean = ( bl.levelRaw <= noiseFloorPct ) ? 0.0f : bl.levelRaw;       // c
         agcHistory_[ b ][ agcHistoryHead_ ] = bl.levelClean;                        // d
         agcBoost_[ b ] = computeAgcBoost_( band );                                  // e

         if ( !soundReactivityEnabled_ ) {
            bl.levelFilt = 0.0f;
         } else if ( effectiveAgcMode == AGCfull ) {
            bl.levelFilt = min( 100.0f, bl.levelClean * agcBoost_[ BandFull ] );      // f
         } else if ( effectiveAgcMode == AGCband ) {
            bl.levelFilt = min( 100.0f, bl.levelClean * agcBoost_[ b ] );             // g
         } else {
            bl.levelFilt = bl.levelClean;                                           // h
         }

         bl.levelRMS = computeBandRms_( band );                                      // i

         bool isAbove = soundReactivityEnabled_ && bl.levelFilt >= threshPct;
         if ( isAbove && !hitWasAbove_[ b ] ) newHit_[ b ] = true;                   // j
         hitWasAbove_[ b ] = isAbove;

         if ( isAbove ) {                                                           // k
            bl.levelHit = 100.0f;
         } else if ( bl.levelHit > 0.0f ) {
            bl.levelHit -= ( 100.0f / hitDecayMs_ ) * (float)dtMs;
            if ( bl.levelHit < 0.0f ) bl.levelHit = 0.0f;
            if ( bl.levelHit <= bl.levelFilt ) bl.levelHit = 0.0f;
         }
         uint8_t predicted = computePredictedRamp( band, displayMs );
         if ( (float)predicted > bl.levelHit ) bl.levelHit = predicted;
      }

      agcHistoryHead_ = ( agcHistoryHead_ + 1 ) % AGC_HISTORY_SIZE; // advance AFTER the loop, once every band has written its slot
   }

   /* --- full-spectrum level, rolling peak, rolling average / noise-floor
    * silence detection --
    *
    * Distinct from (and NOT touched by) the per-band AGC rework above --
    * "full spectrum" here is the max of all 7 raw hardware bins scaled to
    * 0-255, used only by getFullSpectrum() (the audio-screen's megabar
    * glow) and the silence detector (silent_, which many light shows check
    * directly). rollingPeak_ is its own rolling-4s-peak, kept only for
    * getFullSpectrum()'s own optional auto-gain preview -- unrelated to
    * agcBoost_[]/AGCMode above.
    *
    * Silence detection: a separate EMA average (also ~4s time constant)
    * tracks a running level. If that average stays below the noise floor
    * for a full 4s straight, silent_ becomes true until the average rises
    * back above the floor (which un-silences immediately, no delay). The
    * noise floor is compared against the RAW average, not any gained value.
    */
   static const uint16_t PEAK_WINDOW_MS = 4000;
   static const uint8_t PEAK_BUFFER_SIZE = 150; // ~4s of samples at the ~30ms poll rate, with margin

   static uint8_t peakSamples_[ PEAK_BUFFER_SIZE ];
   static uint32_t peakTimestamps_[ PEAK_BUFFER_SIZE ];
   static uint8_t peakHead_;
   static uint8_t peakCount_;
   static uint8_t rollingPeak_;

   static float emaAverage_;
   static Timer pollTimer_; // tracks dt between updateFullSpectrumAndSilence() calls
   static Timer silenceTimer_; // time spent continuously below the noise floor
   static bool silent_;

   static uint8_t rawFullSpectrum() {
      uint16_t maxVal = 0;
      for ( int i = 0; i < 7; ++i ) {
         if ( (uint16_t)Frequencies_Mono[ i ] > maxVal ) maxVal = Frequencies_Mono[ i ];
      }
      return (uint8_t)scale( maxVal );
   }

   static void updateFullSpectrumAndSilence( uint32_t nowMs ) {
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
        simModeActive_ = false;
        Read_Frequencies();
        updateFullSpectrumAndSilence( time );

        // foresight history: every band's just-measured raw % is buffered
        // (timestamped) as it's read -- see the foresight block below for
        // why lookups happen at "now - audioForesightMs_" rather than using
        // the fresh sample directly.
        for ( int b = 0; b < NumFreqBands; ++b ) {
           pushRawHistorySample( (AudioBand)b, analogToPercent( bandRawBinMax( (AudioBand)b ) ), time );
        }

        updateBandLevels( time );
     }
   }

   // alternative to pollFrequencies() for the Audio config screen's decay-
   // rate and hit-prediction subsettings while showing the simulated signal
   // (see updateSimulatedBand() below) instead of real audio -- feeds the
   // same eased square wave into all bands, through the exact same history-
   // buffer/foresight-lookup/updateBandLevels() pipeline real audio uses,
   // so both the decay behavior AND the predictive lead-up being tuned here
   // are the real code paths, not a separate mock. Gated at the same ~30ms
   // cadence as pollFrequencies() so both settings are exercised under the
   // same real poll-rate the hardware uses. Caller is responsible for
   // calling this INSTEAD OF pollFrequencies() while the simulated signal
   // is active, never both.
   static void pollSimulated( uint32_t time ) {
      static uint32_t timestamp;
      if ( time - timestamp > POLL_INTERVAL_MS ) {
         timestamp = time;
         simModeActive_ = true;
         uint8_t simRawAdc = updateSimulatedBand(); // 0-255 domain (see its own comment) -- convert to the % domain this file now uses throughout
         uint8_t simRawPct = rawToPercent( simRawAdc );
         for ( int b = 0; b < NumFreqBands; ++b ) pushRawHistorySample( (AudioBand)b, simRawPct, time );
         updateBandLevels( time );
      }
   }

   // "current full-spectrum audio level" -- silence-gated, and auto-gained
   // if enabled (raw otherwise). This is what drives the audio-screen's
   // megabar glow, and is the intended general-purpose "how loud is it
   // right now" call. Distinct from, and unaffected by, the per-band
   // AGCMode/agcBoost_ system above -- see updateFullSpectrumAndSilence()'s
   // own comment.
   static uint8_t getFullSpectrum() {
      return getFullSpectrum( agcMode_ != AGCoff );
   }

   // same, but with auto-gain explicitly overridden rather than using the
   // committed agcMode_, so the config screen can preview auto-gain
   // on/off without mutating any committed state
   static uint8_t getFullSpectrum( bool useAutoGain ) {
      if ( silent_ ) return 0;
      uint8_t raw = rawFullSpectrum();
      if ( !useAutoGain || rollingPeak_ == 0 ) return raw;
      float gained = (float)raw * ( 255.0f / (float)rollingPeak_ );
      if ( gained > 255.0f ) gained = 255.0f; // clip prevention
      return (uint8_t)( gained + 0.5f );
   }

   // best-effort estimate, in percent, of the source's own volume-knob
   // position -- NOT "how loud is it right now" (that's getFullSpectrum(),
   // which swings with the music's own dynamics regardless of the knob).
   // Uses the rolling 4s peak (rollingPeak_) rather than the instantaneous
   // level or the EMA average, since the loudest moments in a trailing
   // window track a knob's ceiling far more reliably than any single
   // instant does. Currently unused by anything; exists as a general-
   // purpose accessor for later use.
   static uint8_t getOverallLevelPercent() {
      return (uint8_t)( ( (uint16_t)rollingPeak_ * 100 + 127 ) / 255 );
   }

   static void setSoundReactivityEnabled( bool enabled ) {
      soundReactivityEnabled_ = enabled;
   }
   static bool getSoundReactivityEnabled() {
      return soundReactivityEnabled_;
   }

   static void setNoiseFloorPercent( float percent ) {
      noiseFloorPercent_ = constrain( percent, 0.0f, 100.0f );
   }
   static float getNoiseFloorPercent() {
      return noiseFloorPercent_;
   }
   // same 0-100% -> 0-255 conversion used internally by
   // updateFullSpectrumAndSilence()
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

   // --- audio foresight ---
   //
   // The MSGEQ7/DSP pipeline "sees" a waveform before that same sound
   // actually reaches the speakers -- audioForesightMs_ is how many
   // milliseconds ahead of the audible output our reading currently is.
   // Every band's raw sample (%, this file's domain throughout) is buffered
   // into a short per-band ring buffer (history, timestamped) as it's
   // measured; what pollFrequencies()/pollSimulated() then feed into
   // updateBandLevels() as this poll's raw reading is looked up from that
   // history at "now - audioForesightMs_" rather than the just-measured
   // value directly -- i.e. holding our own reaction back so it lines up
   // with when the sound is actually heard, instead of reacting the instant
   // we merely see it. Default 0ms (use the freshest sample) keeps the
   // natural "lights react slightly before the beat is heard" effect that
   // comes for free from the DSP's own inherent look-ahead; dialing
   // foresight UP trades that predictive lead away for tighter sync with
   // the true acoustic output. Only what feeds updateBandLevels() is
   // delayed this way -- updateFullSpectrumAndSilence()'s own internal AGC/
   // silence tracking deliberately keeps working off the true instantaneous
   // signal, since that's a control loop, not something a person perceives
   // directly, and delaying it too would just add unnecessary lag to the
   // gain/silence response.
   static const uint16_t POLL_INTERVAL_MS = 30;
   static const uint16_t FORESIGHT_BUFFER_MS = 1000; // must cover the full 0-1000ms adjustable range
   static const uint16_t FORESIGHT_BUFFER_SIZE = ( FORESIGHT_BUFFER_MS + POLL_INTERVAL_MS - 1 ) / POLL_INTERVAL_MS + 2;
   static uint8_t rawHistory_[ NumFreqBands ][ FORESIGHT_BUFFER_SIZE ]; // band-major, values in %; all bands share one timestamp array below since they're always sampled together
   static uint32_t historyTimestamps_[ FORESIGHT_BUFFER_SIZE ];
   static uint16_t historyHead_;
   static uint16_t historyCount_;
   static float audioForesightMs_; // 0-1000, adjustable + persisted, default 0

   // pushes one band's just-measured raw %-sample into the shared-timestamp
   // ring buffer; caller loops this once per band per poll (see
   // pollFrequencies()/pollSimulated())
   static void pushRawHistorySample( AudioBand band, uint8_t rawPct, uint32_t nowMs ) {
      rawHistory_[ band ][ historyHead_ ] = rawPct;
      if ( band == BandFull ) { // last band pushed each poll (see the calling loops) -- advance head/count and stamp the shared timestamp once, not once per band
         historyTimestamps_[ historyHead_ ] = nowMs;
         historyHead_ = ( historyHead_ + 1 ) % FORESIGHT_BUFFER_SIZE;
         if ( historyCount_ < FORESIGHT_BUFFER_SIZE ) historyCount_++;
      }
   }

   // returns whichever buffered %-sample's timestamp is closest to
   // "nowMs - audioForesightMs_"
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

   // simulated single-band waveform for the Audio config screen's decay-rate
   // subsetting (see CarpetLightLogic.cpp) -- lets the decay behavior be
   // tuned without needing real audio. Alternates high/low phases, holding
   // each phase's duration for 10 seconds before switching between 40ms and
   // 80ms phases. Only advances while actively polled (updateSimulatedBand()
   // must be called every frame the decay-rate screen is showing the
   // simulated signal) -- not run continuously in the background. Returns
   // 0-255 (this is the one place in the file still in that domain, purely
   // as a synthetic waveform generator -- pollSimulated() converts it to %
   // immediately, see above).
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

      // initial reset sequence (per datasheet)
      digitalWrite(STROBE, HIGH); // chip STROBE LOW  (ext inverter)
      digitalWrite(RESET, HIGH);  // chip RESET  LOW  (ext inverter)
      digitalWrite(STROBE, LOW);  // chip STROBE HIGH (ext inverter)
      // delayMicroseconds(100);
      digitalWrite(RESET, LOW);   // chip RESET  HIGH (ext inverter)
      delayMicroseconds(100);     // chip Strb pulse width Ts(min) = 18 us
      digitalWrite(STROBE, HIGH); // chip STROBE LOW  (ext inverter)
      delayMicroseconds(100);     // chip Rst pulse width  Tr(min) = 100 us 
      digitalWrite(STROBE, LOW);  // chip STROBE HIGH (ext inverter)
      // delayMicroseconds(100);
      digitalWrite(RESET, HIGH);  // chip RESET  LOW  (ext inverter)
      delayMicroseconds(100);     // chip Rst falling to Strb falling Trs(min) = 72 us   // ensure any following strobe is valid
   }
};

int AudioBoard::Frequencies_Mono[7];

AudioBoard::BandLevels AudioBoard::bandLevels_[ NumFreqBands ];
float AudioBoard::hitDecayMs_ = 300.0f; // overwritten by Nvm at boot
Timer AudioBoard::hitTimer_;
bool AudioBoard::simModeActive_ = false;
bool AudioBoard::newHit_[ NumFreqBands ] = { false, false, false, false, false };
bool AudioBoard::hitWasAbove_[ NumFreqBands ] = { false, false, false, false, false };
float AudioBoard::hitPredictionMs_ = 0.0f; // overwritten by Nvm at boot
uint8_t AudioBoard::hitPredictionStyle_ = PredictExponential; // overwritten by Nvm at boot

float AudioBoard::agcHistory_[ NumFreqBands ][ AudioBoard::AGC_HISTORY_SIZE ] = { { 0.0f } };
uint32_t AudioBoard::agcHistoryTimestamps_[ AudioBoard::AGC_HISTORY_SIZE ] = { 0 };
uint8_t AudioBoard::agcHistoryHead_ = 0;
uint16_t AudioBoard::agcHistoryCount_ = 0;
float AudioBoard::agcBoost_[ NumFreqBands ] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };

uint8_t AudioBoard::peakSamples_[ AudioBoard::PEAK_BUFFER_SIZE ];
uint32_t AudioBoard::peakTimestamps_[ AudioBoard::PEAK_BUFFER_SIZE ];
uint8_t AudioBoard::peakHead_ = 0;
uint8_t AudioBoard::peakCount_ = 0;
uint8_t AudioBoard::rollingPeak_ = 0;
bool AudioBoard::autoGainSuppressed_ = false;
bool AudioBoard::soundReactivityEnabled_ = true;
uint8_t AudioBoard::agcMode_ = AGCoff; // overwritten by Nvm at boot
float AudioBoard::noiseFloorPercent_ = 0.0f;
float AudioBoard::peakThresholdPercent_ = 31.0f; // approx. the old hardcoded 80/255, overwritten by Nvm at boot
float AudioBoard::emaAverage_ = 0.0f;
Timer AudioBoard::pollTimer_;
Timer AudioBoard::silenceTimer_;
bool AudioBoard::silent_ = false;

uint8_t AudioBoard::rawHistory_[ NumFreqBands ][ AudioBoard::FORESIGHT_BUFFER_SIZE ] = { { 0 } };
uint32_t AudioBoard::historyTimestamps_[ AudioBoard::FORESIGHT_BUFFER_SIZE ] = { 0 };
uint16_t AudioBoard::historyHead_ = 0;
uint16_t AudioBoard::historyCount_ = 0;
float AudioBoard::audioForesightMs_ = 0.0f; // overwritten by Nvm at boot

uint32_t AudioBoard::simPhaseMs_ = 40;
Timer AudioBoard::simStateTimer_( 10000 );
Timer AudioBoard::simPhaseTimer_( 40 );
bool AudioBoard::simPhaseHigh_ = true;

#endif
