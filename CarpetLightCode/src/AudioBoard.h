/* AudioBoard.h
 *
 *   Author: Anders Linn
 *   Date: June 2017
 */

#ifndef __AUDIO_BOARD_H
#define __AUDIO_BOARD_H

#include "Utilities.h"
#include "LoopTime.h"
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

// The MSGEQ7 chip's own 7 hardware bins, in the chip's own datasheet order
// (FreqBin0=63Hz .. FreqBin6=16kHz) -- the finest-grained unit AudioBoard
// tracks state for. AudioBand (below) is a coarser, curated grouping of
// these bins that most callers still want; a few callers (see
// BumpingAudioShow.h's pixel_war) need a specific bin or an ad-hoc bin
// group instead -- see the getBinXxx() getters below, distinct from the
// getBandXxx() ones.
enum AudioBin { FreqBin0 = 0, FreqBin1, FreqBin2, FreqBin3, FreqBin4, FreqBin5, FreqBin6, NumBins };

// which band a getter call is asking about -- see AudioBoard's per-band
// level getters below. BandFull is the overall full-spectrum signal (max of
// all 7 bins), not one of the 4 curated sub-bands -- it's also every
// getter's default when called with no argument at all. Each is a fixed
// group of the AudioBin values above -- see BAND_BINS below for exactly
// which bins feed which band. Band-taking getters aggregate (max) across
// whichever bins the band covers, computed fresh on every call from each
// bin's own independently-tracked level.
enum AudioBand { BandBass = 0, BandMidbass, BandMid, BandTreble, BandFull, NumFreqBands };

// AGC (auto-gain) operating mode -- see updateBandLevels()'s AGC step.
//   AGCoff  -- no gain applied at all, levelFilt == levelClean.
//   AGCband -- each band boosted by its OWN independently-tracked gain.
//   AGCfull -- every band boosted by the SAME gain, derived from BandFull's
//              own signal -- one shared, full-spectrum-derived gain
//              applied uniformly.
enum AGCMode { AGCoff = 0, AGCband, AGCfull, NumAGCModes };

// which shape (if any) the hit-prediction lead-up follows -- see
// AudioBoard::computePredictedRamp_()/computePulseTrainValue() below.
enum HitPredictionStyle { PredictDisabled = 0, PredictExponential = 1, PredictPulseTrain = 2, PredictTwoPulse = 3 };
static const uint8_t NUM_HIT_PREDICTION_STYLES = 4;

// which of the 5 processing stages a stored history value represents -- see
// the big storage comment on audioData_ below.
enum AudioLevelType { LevelRaw = 0, LevelClean, LevelFilt, LevelRMS, LevelHit, NumLevelTypes };

// Auto-peak operating mode -- see getEffectivePeakThresholdPercent()'s own
// comment for the full rule. Named/shaped to mirror AGCMode exactly:
//   AutoPeakOff  -- disabled. Every bin uses the plain global threshold,
//                   nothing excluded.
//   AutoPeakFull -- every INCLUDED bin shares the SAME single effective
//                   threshold, derived from the whole included group's own
//                   collective loudness (like AGCfull: one shared value,
//                   not per-bin differentiated).
//   AutoPeakBin  -- each INCLUDED bin gets its OWN effective threshold,
//                   scaled relative to the loudest currently-included bin
//                   (like AGCband: per-bin differentiated).
// Excluded bins (see autoPeakIncluded_ below) always read hit=0, in both
// Full and Bin modes.
enum AutoPeakMode { AutoPeakOff = 0, AutoPeakFull = 1, AutoPeakBin = 2, NumAutoPeakModes };

class AudioBoard {
 private:

   /* --- unified audio history: storage + the two time pointers ---
    *
    * Replaces what used to be 3 separate, independently-bookkept ring
    * buffers (per-bin AGC history, the foresight raw-history buffer, and
    * the full-spectrum rolling-peak/EMA silence tracker) with ONE ring
    * buffer and two pointers into it.
    *
    * audioData_[type][bin][slot] -- percent (0-100), or LEVEL_INVALID
    * (255) if this slot hasn't been computed yet this lap. Time is the
    * innermost/fastest-varying dimension (matching the old agcHistory_/
    * rawHistory_ convention) since every real operation on this data scans
    * across time for one fixed (type,bin) pair.
    * audioTimestamps_[slot] -- millis() this slot was captured, shared by
    * every bin/type at that slot (they're always sampled together, one
    * poll = one slot).
    *
    * Two pointers, both indices into the SAME ring buffer:
    *   - audioReadIndex_ ("audioReadTime") -- advances by 1 every poll, at
    *     the very start, before anything else happens. Always the most
    *     recently captured instant. INTERNAL ONLY -- no getter ever reads
    *     here; only the pipeline itself (noise removal, AGC scan, RMS
    *     scan, silence scan, hit lookahead) touches this index directly.
    *   - lightsShowIndex_ ("lightsShowTime") -- recomputed every poll,
    *     immediately after audioReadIndex_ advances, as the row whose
    *     timestamp is nearest to audioReadIndex_'s timestamp minus the
    *     live audioForesightMs_ minus the current EMA loop time (see
    *     LoopTime.h -- corrects for the main loop's own render-to-photon
    *     delay, a separate real contributor to sync error on top of
    *     foresight, present even when foresight is 0). EVERY external
    *     getter reads only from this index.
    *
    * Depth: covers AGC_WINDOW_MS (4.0s) of history FLAT -- foresight eats
    * into this same fixed budget rather than extending it, so
    * audioReadIndex_ always wraps cleanly with zero special-casing
    * relative to lightsShowIndex_. AGC's own boost calc doesn't need a
    * trailing-window carve-out anymore anyway (see scanWholeBuffer_()
    * below) -- it uses the ENTIRE buffer, so there's nothing to lose by
    * foresight eating into the budget at high settings.
    */
   static const uint16_t POLL_INTERVAL_MS = 30;
   static const uint16_t AGC_WINDOW_MS = 4000; // total buffer depth -- foresight eats into this, doesn't extend it
   static const uint16_t FORESIGHT_BUFFER_MS = 700; // max audioForesightMs_ -- must match SubAudioForesight's adjustable range in CarpetLightLogic.cpp
   static const uint16_t AUDIO_HISTORY_SIZE = ( AGC_WINDOW_MS + POLL_INTERVAL_MS - 1 ) / POLL_INTERVAL_MS + 2; // +2 slots margin, same convention as the old buffers
   static const uint8_t LEVEL_INVALID = 255; // valid range is 0-100; 255 unambiguously means "not computed this lap"

   static uint8_t audioData_[ NumLevelTypes ][ NumBins ][ AUDIO_HISTORY_SIZE ];
   static uint32_t audioTimestamps_[ AUDIO_HISTORY_SIZE ];
   static uint16_t audioReadIndex_; // "audioReadTime" -- write head, internal only
   static uint16_t audioCount_;
   static uint16_t lightsShowIndex_; // "lightsShowTime" -- every getter reads here

   // which AudioBin values each AudioBand covers -- BAND_BIN_COUNT[band] of
   // them, listed in BAND_BINS[band][0..count-1] (rows padded to 7 wide,
   // trailing entries beyond each row's own count are unused/ignored).
   static const uint8_t BAND_BIN_COUNT[ NumFreqBands ];
   static const AudioBin BAND_BINS[ NumFreqBands ][ NumBins ];

   // returns v, or 0 if v is the LEVEL_INVALID sentinel (shouldn't happen
   // for anything read at lightsShowIndex_ in normal operation -- every
   // field there is always computed before any getter can run -- but kept
   // as a defensive floor rather than ever surfacing 255% to a caller).
   static uint8_t valOrZero_( uint8_t v ) { return ( v == LEVEL_INVALID ) ? 0 : v; }

   // --- auto-peak per-bin inclusion (see AutoPeakMode above) ---
   //
   // A bin becomes "included" automatically, as a side effect, the moment
   // ANY hit-family getter is called for it (getBinHitPercent/
   // getBandHitPercent/+Nonzero/NewBinHit/NewBandHit) -- no light show
   // configures this explicitly. It stays included until
   // clearAutoPeakInclusion() runs, which happens automatically on boot
   // and on every show/variation change (see CarpetLightLogic.cpp), and
   // may also be called explicitly by a show for its own reasons. No
   // persistence needed -- this is purely transient, rebuilt fresh every
   // time a show starts querying its own bands again.
   static bool autoPeakIncluded_[ NumBins ];
   static void markIncluded_( AudioBin bin ) { autoPeakIncluded_[ bin ] = true; }
   static void markBandIncluded_( AudioBand band ) {
      uint8_t n = BAND_BIN_COUNT[ band ];
      for ( uint8_t i = 0; i < n; ++i ) markIncluded_( BAND_BINS[ band ][ i ] );
   }

   // the one shared implementation every getBandXxx() getter is built on --
   // max of the given levelType across whichever bins the band covers, at
   // lightsShowIndex_.
   static uint8_t aggregateBandType_( AudioBand band, AudioLevelType type ) {
      uint8_t m = 0;
      uint8_t n = BAND_BIN_COUNT[ band ];
      for ( uint8_t i = 0; i < n; ++i ) {
         uint8_t v = valOrZero_( audioData_[ type ][ BAND_BINS[ band ][ i ] ][ lightsShowIndex_ ] );
         if ( v > m ) m = v;
      }
      return m;
   }

   static float hitDecayMs_;    // ms to decay from 100 to 0, adjustable + persisted, default 300
   static Timer hitTimer_;      // tracks dt between updateBandLevels() calls
   static bool simModeActive_;  // set true/false at the top of pollSimulated()/pollFrequencies() respectively

   // hit envelope's persistent value, per bin -- kept as a float separate
   // from the rounded uint8_t stored in audioData_[LevelHit] so repeated
   // small decay steps don't compound rounding error tick over tick. This
   // IS what audioData_[LevelHit][bin][lightsShowIndex_] gets rounded into
   // every poll, not a second independent source of truth.
   static float hitPersist_[ NumBins ];
   static uint32_t hitOnsetMs_[ NumBins ]; // timestamp (lightsShowTime's own ms) this bin's CURRENT hit began
   static bool hitWasAbove_[ NumBins ];    // per-bin edge-detection state, evaluated once per poll at lightsShowIndex_

   // this poll's "was lightsShowIndex_ a fresh hit edge for this bin" flag
   // -- NewBinHit() below is the only thing that ever reads it, and
   // dedupes against lastReportedHitMs_ so repeated/multiple-caller reads
   // of the SAME lightsShowTime instant only ever report true once.
   static bool newHitAtLightsShow_[ NumBins ];
   static uint32_t lastReportedHitMs_[ NumBins ];
   static bool lastReportedHitEverValid_[ NumBins ];

   // Per-bin AGC boost multiplier, and the shared AGCfull equivalent --
   // now a genuinely STATEFUL envelope follower (not recomputed from
   // scratch each poll): instant-down the moment a louder sample appears
   // ANYWHERE in the buffer (including foresight-buffered future rows --
   // see scanWholeBuffer_() below, this is what makes AGC proactive
   // whenever foresight is nonzero), continuous fixed-rate rise otherwise.
   // Classic fast-attack/slow-release compressor envelope shape.
   static float agcBoost_[ NumBins ];
   static float agcBoostFull_;

   // one full-buffer scan over a bin's LevelClean history serves AGC boost
   // (peak), RMS (sumSquares/n), and silence (anyNonzero) all at once.
   // NOT time-windowed -- the ring buffer's own fixed depth (AGC_WINDOW_MS,
   // flat) already IS the window, so every currently-valid slot
   // unconditionally belongs in it; no per-entry timestamp comparison
   // needed. Includes audioReadIndex_'s own just-written row (valid by the
   // time this runs, in step F, after step E already wrote it) -- and,
   // whenever foresight > 0, everything captured-but-not-yet-"shown" ahead
   // of lightsShowTime too, which is the whole point: a loud sample sitting
   // in that buffered future already pulls the scan's peak up (and this
   // bin's boost down) before it's actually the displayed instant.
   struct WholeScan_ { float peak; float sumSquares; uint16_t n; bool anyNonzero; };
   static WholeScan_ scanWholeBuffer_( uint8_t bin ) {
      WholeScan_ r = { 0.0f, 0.0f, 0, false };
      for ( uint16_t i = 0; i < audioCount_; ++i ) {
         uint8_t v = audioData_[ LevelClean ][ bin ][ i ];
         if ( v == LEVEL_INVALID ) continue;
         float f = (float)v;
         if ( f > r.peak ) r.peak = f;
         r.sumSquares += f * f;
         ++r.n;
         if ( v > 0 ) r.anyNonzero = true;
      }
      return r;
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
   // shapePulse() above, peaking at 100) leading up to the predicted hit,
   // fixed to its own 300ms lead-in -- pulse-to-pulse spacing starts at
   // 90ms and narrows to 30ms (touching, back-to-back) right at the hit.
   // Pulse onset times are generated by a small (~6-iteration) integer
   // loop walking backward from the 300ms mark, recomputing the linearly-
   // interpolated spacing at each step (forward-Euler style) rather than
   // solving the continuous spacing-vs-time relationship analytically
   // (which would need a logarithm) -- cheap, and close enough at this
   // timescale.
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

   // PredictTwoPulse style: just two 35ms-wide pulses (shapePulse()-
   // rounded, peaking at 100), 45ms apart, anchored at
   // min(audioForesightMs_, 400ms) before the predicted hit -- capped to
   // the live foresight setting rather than a fixed 400ms, since there's
   // no buffered lookahead data past whatever foresight actually provides.
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

   // Predictive lead-up, folded directly out of the buffer split: scans
   // levelClean (NOT levelFilt -- rows strictly ahead of lightsShowIndex_
   // haven't had levelFilt computed yet, that only happens once
   // lightsShowTime itself reaches them) from just after showIdx forward
   // through readIdx (inclusive) -- exactly the span between lightsShowTime
   // and audioReadTime, i.e. the whole live foresight window, for the
   // EARLIEST upcoming sample crossing the plain peak threshold. If found,
   // shapes the lead-up per hitPredictionStyle_:
   //   PredictDisabled     -- always 0, no lookahead performed at all.
   //   PredictExponential  -- a rising ramp (0-100) as a cheap-integer-math
   //     stand-in for "an exponential curve with a vertical asymptote lined
   //     up exactly on the predicted hit": ramp =
   //     100*(audioForesightMs_-timeUntilHit)/timeUntilHit -- 0 at
   //     timeUntilHit==audioForesightMs_ (the farthest predictive point),
   //     growing without bound as timeUntilHit shrinks toward 0, clamped
   //     to 100.
   //   PredictPulseTrain   -- see computePulseTrainValue() above.
   //   PredictTwoPulse     -- see computeTwoPulseValue() above.
   // No separate hitPredictionMs_ dial exists anymore -- prediction always
   // runs over the whole live foresight span; the individual shape
   // functions above keep their own fixed internal lead/spacing constants
   // as timing offsets within that.
   static uint8_t computePredictedRamp_( AudioBin bin, uint16_t showIdx, uint16_t readIdx, uint32_t showMs ) {
      if ( hitPredictionStyle_ == PredictDisabled || showIdx == readIdx ) return 0;
      float threshPct = getPlainPeakThresholdPercent();
      uint32_t bestHitMs = 0;
      bool found = false;
      uint16_t i = (uint16_t)( ( showIdx + 1 ) % AUDIO_HISTORY_SIZE );
      while ( true ) {
         uint8_t clean = audioData_[ LevelClean ][ bin ][ i ];
         if ( clean != LEVEL_INVALID && (float)clean >= threshPct ) {
            bestHitMs = audioTimestamps_[ i ];
            found = true;
            break; // walking forward chronologically -- first crossing found is the earliest
         }
         if ( i == readIdx ) break;
         i = (uint16_t)( ( i + 1 ) % AUDIO_HISTORY_SIZE );
      }
      if ( !found ) return 0;
      uint32_t timeUntilHit = ( bestHitMs > showMs ) ? ( bestHitMs - showMs ) : 0;
      if ( timeUntilHit == 0 ) return 100;
      if ( hitPredictionStyle_ == PredictPulseTrain ) return computePulseTrainValue( timeUntilHit );
      if ( hitPredictionStyle_ == PredictTwoPulse ) return computeTwoPulseValue( timeUntilHit );
      uint32_t hp = (uint32_t)audioForesightMs_;
      if ( hp < 1 || timeUntilHit >= hp ) return 0;
      uint32_t ramp = ( 100UL * ( hp - timeUntilHit ) ) / timeUntilHit;
      return (uint8_t)( ramp > 100 ? 100 : ramp );
   }

   static bool autoGainSuppressed_;
   // Global kill switch for all sound-reactive light behavior -- see the
   // getters below, which gate on this centrally so no individual show
   // needs its own check. Default true (on); persisted via Nvm, see
   // setSoundReactivityEnabled().
   static bool soundReactivityEnabled_;
   static uint8_t agcMode_; // AGCoff/AGCband/AGCfull, persisted -- see AGCMode above
   static float noiseFloorPercent_; // 0-100, the GLOBAL setting -- see binNoiseFloorPercent_ below for the derived per-bin values actually used
   static float peakThresholdPercent_; // 0-100

   /* --- per-bin noise floor, from measured self-noise voltages ---
    *
    * Scoped self-noise differs consistently per bin (measured with the
    * phone muted): bins 0-4 ~0.17V, bin 5 ~0.280V, bin 6 (highest freq)
    * ~0.56V. BIN_NOISE_VOLTAGE holds these raw scope readings (volts, not
    * pre-converted percent, so a future re-measurement can be hand-edited
    * in directly) -- only the RATIO between bins ever matters downstream,
    * which is identical whether stored as volts or as %-of-max.
    * binNoiseFloorPercent_[] is derived from it: the bin with the highest
    * voltage (bin6) gets the global noiseFloorPercent_ setting directly;
    * every other bin's floor scales down proportionally to its own
    * voltage ratio against that max. Recomputed whenever
    * setNoiseFloorPercent() is called (see recomputeBinNoiseFloors_()).
    */
   static const float BIN_NOISE_VOLTAGE[ NumBins ];
   static float binNoiseFloorPercent_[ NumBins ];
   static void recomputeBinNoiseFloors_() {
      float maxV = 0.0f;
      for ( uint8_t bin = 0; bin < NumBins; ++bin ) if ( BIN_NOISE_VOLTAGE[ bin ] > maxV ) maxV = BIN_NOISE_VOLTAGE[ bin ];
      for ( uint8_t bin = 0; bin < NumBins; ++bin ) {
         binNoiseFloorPercent_[ bin ] = ( maxV <= 0.0f ) ? noiseFloorPercent_ : ( noiseFloorPercent_ * ( BIN_NOISE_VOLTAGE[ bin ] / maxV ) );
      }
   }

   // --- auto-peak (per-bin threshold scaling -- see AutoPeakMode above) ---
   static uint8_t autoPeakMode_; // persisted -- see setAutoPeakMode()
   // Bin mode only: an included bin whose own RMS is below this fraction
   // of the loudest currently-included bin's RMS gets hit=0 too (too quiet
   // RIGHT NOW relative to the rest of the included group to be worth
   // reacting to) -- compile-time for now.
   static constexpr float AUTO_PEAK_MIN_INCLUSION_LEVEL = 50.0f;

   // max RMS (at lightsShowIndex_) among currently-included bins, or 0 if
   // none included/all silent -- the shared reference both AutoPeakFull
   // and AutoPeakBin scale against.
   static float maxIncludedRms_() {
      float m = 0.0f;
      for ( uint8_t bin = 0; bin < NumBins; ++bin ) {
         if ( !autoPeakIncluded_[ bin ] ) continue;
         float rms = (float)valOrZero_( audioData_[ LevelRMS ][ bin ][ lightsShowIndex_ ] );
         if ( rms > m ) m = rms;
      }
      return m;
   }

   // Hit sustain envelope: a hit starts at a flat 100% (so the leading edge
   // always reads full strength). If the SAME hit is still continuously
   // above threshold past HIT_SUSTAIN_HOLD_MS since its own onset, ease
   // down to HIT_SUSTAIN_PERCENT over the following HIT_SUSTAIN_EASE_MS,
   // then hold there for as long as the hit continues -- so a long
   // sustained tone doesn't sit pinned at 100% the whole time. Once the
   // hit actually ends, the existing decay path takes over from whatever
   // hitPersist_ currently holds (100, mid-ease, or the 90% sustain
   // floor), not always from 100.
   static const uint16_t HIT_SUSTAIN_HOLD_MS = 75;
   static const uint16_t HIT_SUSTAIN_EASE_MS = 25;
   static constexpr float HIT_SUSTAIN_PERCENT = 90.0f;

   static bool silent_; // internal only -- see isSilent() below

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

   /* --- the per-poll pipeline --- runs once per real poll (~30ms), from
    * pollFrequencies()/pollSimulated() below. Step by step:
    *   A) advance audioReadTime (the write head), invalidate its slot
    *   B) recompute lightsShowTime = nearest row to (nowMs - foresight -
    *      EMA loop time)
    *   C) capture this poll's raw % per bin at audioReadTime
    *   D) timestamp audioReadTime's slot
    *   E) noise removal (per-bin floors, global all-below gate) ->
    *      levelClean at audioReadTime
    *   F) AGC envelope (whole-buffer scan, instant-down/continuous-rise)
    *      -> levelFilt at lightsShowTime; the same scan also produces RMS
    *      and silence (see scanWholeBuffer_() above)
    *   G) hit envelope + predictive lookahead -> levelHit at lightsShowTime,
    *      gated by auto-peak inclusion/mode
    */
   static void updateBandLevels( uint32_t nowMs, bool simulated, uint8_t simRawPct ) {
      uint32_t dtMs = hitTimer_.elapsed();
      hitTimer_.reset();
      uint8_t effectiveAgcMode = autoGainSuppressed_ ? (uint8_t)AGCoff : agcMode_;

      // (A)
      audioReadIndex_ = (uint16_t)( ( audioReadIndex_ + 1 ) % AUDIO_HISTORY_SIZE );
      if ( audioCount_ < AUDIO_HISTORY_SIZE ) ++audioCount_;
      for ( uint8_t t = 0; t < NumLevelTypes; ++t )
         for ( uint8_t bin = 0; bin < NumBins; ++bin )
            audioData_[ t ][ bin ][ audioReadIndex_ ] = LEVEL_INVALID;

      // (B) nearest-timestamp match, not exact arithmetic -- self-correcting
      // against real poll-to-poll jitter. Excludes audioReadIndex_ itself
      // (just invalidated, not written yet this lap) -- falls back to it
      // if nothing else valid exists yet (boot bootstrap). Target pulls
      // back by BOTH the live foresight setting AND the current EMA loop
      // time (see LoopTime.h) -- the latter applies even at foresight=0.
      uint32_t backMs = (uint32_t)audioForesightMs_ + (uint32_t)LoopTime::getEmaMs();
      uint32_t targetMs = ( nowMs > backMs ) ? ( nowMs - backMs ) : 0;
      uint16_t bestIdx = audioReadIndex_;
      uint32_t bestDelta = 0xFFFFFFFFu;
      for ( uint16_t i = 0; i < audioCount_; ++i ) {
         if ( i == audioReadIndex_ ) continue;
         uint32_t ts = audioTimestamps_[ i ];
         uint32_t delta = ( ts > targetMs ) ? ( ts - targetMs ) : ( targetMs - ts );
         if ( delta < bestDelta ) { bestDelta = delta; bestIdx = i; }
      }
      lightsShowIndex_ = bestIdx;

      // (C)
      if ( simulated ) {
         for ( int bin = 0; bin < NumBins; ++bin ) audioData_[ LevelRaw ][ bin ][ audioReadIndex_ ] = simRawPct;
      } else {
         Read_Frequencies();
         for ( int bin = 0; bin < NumBins; ++bin ) audioData_[ LevelRaw ][ bin ][ audioReadIndex_ ] = analogToPercent( Frequencies_Mono[ bin ] );
      }

      // (D)
      audioTimestamps_[ audioReadIndex_ ] = nowMs;

      // (E) global gate: only pass raw through as clean if AT LEAST ONE
      // bin is above ITS OWN per-bin floor -- otherwise every bin's clean
      // is 0 this poll, even one sitting just above its own floor. The
      // per-bin floors exist to correctly calibrate "is THIS bin real
      // signal or just its own self-noise"; the all-below test is the
      // right way to combine those per-bin tests into one "is anything
      // real happening at all" global gate.
      bool anyAboveFloor = false;
      for ( int bin = 0; bin < NumBins; ++bin ) {
         if ( (float)audioData_[ LevelRaw ][ bin ][ audioReadIndex_ ] > binNoiseFloorPercent_[ bin ] ) { anyAboveFloor = true; break; }
      }
      for ( int bin = 0; bin < NumBins; ++bin ) {
         audioData_[ LevelClean ][ bin ][ audioReadIndex_ ] = anyAboveFloor ? audioData_[ LevelRaw ][ bin ][ audioReadIndex_ ] : (uint8_t)0;
      }

      // (F) AGC envelope + RMS + silence, one whole-buffer scan per bin
      uint32_t showMs = audioTimestamps_[ lightsShowIndex_ ];
      float fullPeak = 0.0f;
      bool anySoundInWindow = false;
      // ceiling = 100/noiseFloor is inherent (smallest nonzero clean value
      // is always > the (highest, bin6) noise floor), so the rise rate is
      // naturally bounded without a separate clamp
      float ceilNoiseFloor = ( noiseFloorPercent_ > 0.1f ) ? noiseFloorPercent_ : 0.1f;
      float maxBoost = 100.0f / ceilNoiseFloor;
      float riseRatePerMs = ( maxBoost - 1.0f ) / (float)AGC_WINDOW_MS; // full range top-to-bottom takes exactly one buffer-length
      for ( int bin = 0; bin < NumBins; ++bin ) {
         WholeScan_ sc = scanWholeBuffer_( (uint8_t)bin );
         float target = ( sc.peak <= 0.0f ) ? 1.0f : ( 100.0f / sc.peak );
         if ( target < agcBoost_[ bin ] ) agcBoost_[ bin ] = target; // instant attack
         else agcBoost_[ bin ] = min( target, agcBoost_[ bin ] + riseRatePerMs * (float)dtMs ); // continuous release, never overshoots target
         float rms = ( sc.n == 0 ) ? 0.0f : sqrtf( sc.sumSquares / (float)sc.n );
         audioData_[ LevelRMS ][ bin ][ lightsShowIndex_ ] = (uint8_t)( rms + 0.5f );
         if ( sc.peak > fullPeak ) fullPeak = sc.peak;
         if ( sc.anyNonzero ) anySoundInWindow = true;
      }
      float fullTarget = ( fullPeak <= 0.0f ) ? 1.0f : ( 100.0f / fullPeak );
      if ( fullTarget < agcBoostFull_ ) agcBoostFull_ = fullTarget;
      else agcBoostFull_ = min( fullTarget, agcBoostFull_ + riseRatePerMs * (float)dtMs );
      // silent only once a full window of real history has actually
      // accumulated -- avoids spuriously reporting silence right after
      // boot before there's been time to see any sound at all
      silent_ = ( showMs >= AGC_WINDOW_MS || audioCount_ >= AUDIO_HISTORY_SIZE ) ? !anySoundInWindow : false;

      for ( int bin = 0; bin < NumBins; ++bin ) {
         uint8_t clean = audioData_[ LevelClean ][ bin ][ lightsShowIndex_ ];
         float filt;
         if ( !soundReactivityEnabled_ ) filt = 0.0f;
         else if ( effectiveAgcMode == AGCfull ) filt = min( 100.0f, (float)clean * agcBoostFull_ );
         else if ( effectiveAgcMode == AGCband ) filt = min( 100.0f, (float)clean * agcBoost_[ bin ] );
         else filt = (float)clean;
         audioData_[ LevelFilt ][ bin ][ lightsShowIndex_ ] = (uint8_t)( filt + 0.5f );
      }

      // (G)
      for ( int bin = 0; bin < NumBins; ++bin ) {
         // auto-peak exclusion: forced hit=0, skip the rest of this bin's
         // envelope entirely -- not a decay, an immediate hard zero
         if ( autoPeakMode_ != AutoPeakOff && !autoPeakIncluded_[ bin ] ) {
            hitPersist_[ bin ] = 0.0f;
            audioData_[ LevelHit ][ bin ][ lightsShowIndex_ ] = 0;
            hitWasAbove_[ bin ] = false;
            newHitAtLightsShow_[ bin ] = false;
            continue;
         }

         uint8_t filt = audioData_[ LevelFilt ][ bin ][ lightsShowIndex_ ];
         float threshPct = getEffectivePeakThresholdPercent( (AudioBin)bin );
         bool isAbove = soundReactivityEnabled_ && (float)filt >= threshPct;
         bool freshEdge = isAbove && !hitWasAbove_[ bin ];
         newHitAtLightsShow_[ bin ] = freshEdge;
         if ( freshEdge ) hitOnsetMs_[ bin ] = showMs;
         hitWasAbove_[ bin ] = isAbove;

         float hitVal;
         if ( isAbove ) {
            uint32_t heldMs = showMs - hitOnsetMs_[ bin ];
            if ( heldMs <= HIT_SUSTAIN_HOLD_MS ) {
               hitVal = 100.0f;
            } else if ( heldMs <= (uint32_t)HIT_SUSTAIN_HOLD_MS + HIT_SUSTAIN_EASE_MS ) {
               float f = (float)( heldMs - HIT_SUSTAIN_HOLD_MS ) / (float)HIT_SUSTAIN_EASE_MS;
               hitVal = 100.0f - f * ( 100.0f - HIT_SUSTAIN_PERCENT );
            } else {
               hitVal = HIT_SUSTAIN_PERCENT;
            }
         } else {
            hitVal = hitPersist_[ bin ];
            if ( hitVal > 0.0f ) {
               hitVal -= ( 100.0f / hitDecayMs_ ) * (float)dtMs;
               if ( hitVal < 0.0f ) hitVal = 0.0f;
               if ( hitVal <= (float)filt ) hitVal = 0.0f;
            }
         }

         uint8_t predicted = computePredictedRamp_( (AudioBin)bin, lightsShowIndex_, audioReadIndex_, showMs );
         if ( (float)predicted > hitVal ) hitVal = (float)predicted;

         hitPersist_[ bin ] = hitVal;
         audioData_[ LevelHit ][ bin ][ lightsShowIndex_ ] = (uint8_t)( hitVal + 0.5f );
      }
   }

 public:
   //Define spectrum variables
   static int Frequencies_Mono[7];

   // clears every bin's auto-peak inclusion (back to fully excluded) --
   // called automatically on boot and on every show/variation change (see
   // CarpetLightLogic.cpp); a show may also call this explicitly for its
   // own reasons (e.g. resetting inclusion mid-show before switching which
   // of its own effects are currently active).
   static void clearAutoPeakInclusion() {
      for ( uint8_t bin = 0; bin < NumBins; ++bin ) autoPeakIncluded_[ bin ] = false;
   }

   // returns (and clears) whether THIS bin has had a new hit since the
   // last time this was checked FOR THIS SAME lightsShowTime instant --
   // dedupes by timestamp (not raw index, which can be reused across ring-
   // buffer wraps) so calling this more than once, or from more than one
   // caller, while lightsShowTime sits on the same row never reports a
   // second true for that one edge. Also auto-includes this bin in
   // auto-peak (see clearAutoPeakInclusion() above) -- any code asking for
   // a bin's hit-edge state is implicitly declaring it cares about that
   // bin's own hit behavior.
   static bool NewBinHit( AudioBin bin ) {
      markIncluded_( bin );
      uint32_t rowMs = audioTimestamps_[ lightsShowIndex_ ];
      if ( lastReportedHitEverValid_[ bin ] && lastReportedHitMs_[ bin ] == rowMs ) return false;
      bool edge = newHitAtLightsShow_[ bin ];
      if ( edge ) {
         lastReportedHitMs_[ bin ] = rowMs;
         lastReportedHitEverValid_[ bin ] = true;
      }
      return edge;
   }
   // band version: true if ANY bin the band covers has a pending hit --
   // consumes (dedupes) every one of that band's bins regardless, not just
   // the first one found, so a bin's pending edge can't leak into a later
   // call through a DIFFERENT band that happens to also cover it.
   static bool NewBandHit( AudioBand band = BandFull ) {
      bool any = false;
      uint8_t n = BAND_BIN_COUNT[ band ];
      for ( uint8_t i = 0; i < n; ++i ) {
         if ( NewBinHit( BAND_BINS[ band ][ i ] ) ) any = true;
      }
      return any;
   }

   // true once BandFull's levelClean has been entirely zero (i.e. at/below
   // each bin's own noise floor, across every bin) continuously for the
   // full buffer depth -- computed automatically as part of the per-poll
   // pipeline (see updateBandLevels()'s step F), leveraging the same real
   // history AGC/RMS already scan. Internal state (silent_) is private;
   // this is the only way to read it externally now.
   static bool isSilent() { return silent_; }

   // Every one of these comes in two distinctly-named forms (not an
   // overload pair) -- getBandXxx(AudioBand) aggregates (max) across
   // whichever bins that band covers; getBinXxx(AudioBin) reads exactly one
   // bin, no aggregation. Both always read from lightsShowIndex_ ("what the
   // user is currently hearing"), never audioReadIndex_.
   static uint8_t getBinRawPercent( AudioBin bin ) { return valOrZero_( audioData_[ LevelRaw ][ bin ][ lightsShowIndex_ ] ); }
   static uint8_t getBandRawPercent( AudioBand band = BandFull ) { return aggregateBandType_( band, LevelRaw ); }
   static bool getBinRawNonzero( AudioBin bin ) { return getBinRawPercent( bin ) > 0; }
   static bool getBandRawNonzero( AudioBand band = BandFull ) { return getBandRawPercent( band ) > 0; }
   // BUGFIX: these 4 getters are what every light show actually reacts to
   // (raw above stays ungated -- diagnostic screens like the noise-floor
   // bin readout should still show real signal even with reactivity
   // globally off). Gating here, once, centrally, means the global disable
   // (see soundReactivityEnabled_ above) doesn't need to be checked
   // separately inside every single show.
   static uint8_t getBinNormalPercent( AudioBin bin ) { return soundReactivityEnabled_ ? valOrZero_( audioData_[ LevelFilt ][ bin ][ lightsShowIndex_ ] ) : 0; }
   static uint8_t getBandNormalPercent( AudioBand band = BandFull ) { return soundReactivityEnabled_ ? aggregateBandType_( band, LevelFilt ) : 0; }
   // preview variant: recomputes normal level under an EXPLICIT AGCMode,
   // not the committed agcMode_ -- lets the Audio config screen's AGC-mode
   // subsetting preview a live, not-yet-committed mode choice without
   // mutating (and needing to revert) agcMode_ itself. Cheap: agcBoost_/
   // agcBoostFull_ are already recomputed every poll regardless of which
   // mode is actually selected, this just picks a different final branch.
   static uint8_t getBandNormalPercentPreview( AudioBand band, uint8_t agcModeOverride ) {
      if ( !soundReactivityEnabled_ ) return 0;
      uint8_t m = 0;
      uint8_t n = BAND_BIN_COUNT[ band ];
      for ( uint8_t i = 0; i < n; ++i ) {
         AudioBin bin = BAND_BINS[ band ][ i ];
         uint8_t clean = valOrZero_( audioData_[ LevelClean ][ bin ][ lightsShowIndex_ ] );
         float filt;
         if ( agcModeOverride == AGCfull ) filt = min( 100.0f, (float)clean * agcBoostFull_ );
         else if ( agcModeOverride == AGCband ) filt = min( 100.0f, (float)clean * agcBoost_[ bin ] );
         else filt = (float)clean;
         uint8_t v = (uint8_t)( filt + 0.5f );
         if ( v > m ) m = v;
      }
      return m;
   }
   static bool getBinNormalNonzero( AudioBin bin ) { return getBinNormalPercent( bin ) > 0; }
   static bool getBandNormalNonzero( AudioBand band = BandFull ) { return getBandNormalPercent( band ) > 0; }
   static uint8_t getBinHitPercent( AudioBin bin ) { markIncluded_( bin ); return soundReactivityEnabled_ ? valOrZero_( audioData_[ LevelHit ][ bin ][ lightsShowIndex_ ] ) : 0; }
   static uint8_t getBandHitPercent( AudioBand band = BandFull ) { markBandIncluded_( band ); return soundReactivityEnabled_ ? aggregateBandType_( band, LevelHit ) : 0; }
   // true whenever this bin/band's hit level is currently above the live
   // peak threshold, false otherwise -- NOT a plain nonzero test (hit stays
   // nonzero throughout its decay tail after dropping back below threshold,
   // which shouldn't still read as "a hit" once decayed past that point).
   static bool getBinHitNonzero( AudioBin bin ) { markIncluded_( bin ); return soundReactivityEnabled_ && (float)getBinHitPercent( bin ) > getPlainPeakThresholdPercent(); }
   static bool getBandHitNonzero( AudioBand band = BandFull ) { markBandIncluded_( band ); return soundReactivityEnabled_ && (float)getBandHitPercent( band ) > getPlainPeakThresholdPercent(); }
   static uint8_t getBinRmsPercent( AudioBin bin ) { return valOrZero_( audioData_[ LevelRMS ][ bin ][ lightsShowIndex_ ] ); }
   static uint8_t getBandRmsPercent( AudioBand band = BandFull ) { return aggregateBandType_( band, LevelRMS ); }
   static bool getBinRmsNonzero( AudioBin bin ) { return getBinRmsPercent( bin ) > 0; }
   static bool getBandRmsNonzero( AudioBand band = BandFull ) { return getBandRmsPercent( band ) > 0; }

   // adjustable range is 0ms (instant decay) to 1000ms (1 full second) --
   // set live via the Audio config screen's decay-rate subsetting, pot-
   // adjusted. 0ms still behaves safely: 100.0f/0.0f is IEEE-754
   // +infinity, which immediately drives the hit value to (and clamped at)
   // 0 on the very next update, i.e. genuinely instant decay, no divide-
   // by-zero fault.
   static void setHitDecayMs( float ms ) { hitDecayMs_ = constrain( ms, 0.0f, 1000.0f ); }
   static float getHitDecayMs() { return hitDecayMs_; }

   // 0-FORESIGHT_BUFFER_MS, adjustable + persisted, default 0 (freshest
   // sample, maximal "predictive" lead). Prediction (see
   // computePredictedRamp_() above) always runs over this whole span --
   // there's no separate, independently-adjustable prediction-distance
   // setting anymore.
   static void setAudioForesightMs( float ms ) { audioForesightMs_ = constrain( ms, 0.0f, (float)FORESIGHT_BUFFER_MS ); }
   static float getAudioForesightMs() { return audioForesightMs_; }

   static void setHitPredictionStyle( uint8_t style ) { hitPredictionStyle_ = ( style >= NUM_HIT_PREDICTION_STYLES ) ? ( NUM_HIT_PREDICTION_STYLES - 1 ) : style; }
   static uint8_t getHitPredictionStyle() { return hitPredictionStyle_; }
   static float audioForesightMs_; // 0-FORESIGHT_BUFFER_MS, adjustable + persisted, default 340
   static uint8_t hitPredictionStyle_; // adjustable + persisted, default PredictExponential -- see HitPredictionStyle above

   static void setAgcMode( uint8_t mode ) { agcMode_ = ( mode >= NumAGCModes ) ? ( NumAGCModes - 1 ) : mode; }
   static uint8_t getAgcMode() { return agcMode_; }
   // While set (see CarpetLightLogic.cpp), forces AGCoff behavior for as
   // long as the noise-floor/peak-threshold config subsetting is active,
   // without touching the persisted setting itself -- dialing either of
   // those in against a live auto-gained signal is confusing (the gain
   // constantly renormalizes what you're trying to measure against a fixed
   // floor/threshold). The AGC envelope keeps running in the background
   // regardless, so auto-gain picks back up smoothly (no jump) once this
   // is cleared.
   static void setAutoGainSuppressed( bool suppressed ) { autoGainSuppressed_ = suppressed; }

   static void setSoundReactivityEnabled( bool enabled ) { soundReactivityEnabled_ = enabled; }
   static bool getSoundReactivityEnabled() { return soundReactivityEnabled_; }

   static void setNoiseFloorPercent( float percent ) {
      noiseFloorPercent_ = constrain( percent, 0.0f, 100.0f );
      recomputeBinNoiseFloors_();
   }
   static float getNoiseFloorPercent() { return noiseFloorPercent_; }
   // same 0-100% -> 0-255 conversion used internally
   static uint8_t getNoiseFloorRaw() { return (uint8_t)( constrain( noiseFloorPercent_, 0.0f, 100.0f ) / 100.0f * 255.0f + 0.5f ); }

   static void setPeakThresholdPercent( float percent ) { peakThresholdPercent_ = constrain( percent, 0.0f, 100.0f ); }
   static float getPeakThresholdPercent() { return peakThresholdPercent_; }
   static uint8_t getPeakThresholdRaw() { return (uint8_t)( constrain( peakThresholdPercent_, 0.0f, 100.0f ) / 100.0f * 255.0f + 0.5f ); }

   static void setAutoPeakMode( uint8_t mode ) { autoPeakMode_ = ( mode >= NumAutoPeakModes ) ? ( NumAutoPeakModes - 1 ) : mode; }
   static uint8_t getAutoPeakMode() { return autoPeakMode_; }

   /* THE common getter for actually deciding "is this bin's level a hit
    * right now" -- full rule:
    *   AutoPeakOff  -- plain global threshold, every bin.
    *   AutoPeakFull -- excluded bins: 101 (unreachable). Included bins:
    *     ALL share ONE threshold, globalThresh * (groupRms/100), where
    *     groupRms is the max RMS among currently-included bins -- the
    *     whole included set is treated like one shared band, the same
    *     way AGCfull derives one shared gain from BandFull rather than
    *     differentiating per bin.
    *   AutoPeakBin  -- excluded bins: 101. Included bins each get their
    *     OWN threshold, globalThresh * (thisBinRms/maxIncludedRms) --
    *     differentiated per bin, the same way AGCband differentiates per
    *     bin instead of sharing one gain. A bin whose own RMS is below
    *     AUTO_PEAK_MIN_INCLUSION_LEVEL of the group's max also reads 101
    *     (too quiet RIGHT NOW relative to the rest of the included group).
    * Only ever used for the real hit-ACCEPTANCE decision in
    * updateBandLevels() -- display/preview getters use
    * getBinAutoScaledPeakThresholdPercent()/
    * getBandAutoScaledPeakThresholdPercent() below instead (same idea,
    * clamped back into 0-100 for showing on a meter).
    */
   static float getEffectivePeakThresholdPercent( AudioBin bin ) {
      float base = constrain( peakThresholdPercent_, 0.0f, 100.0f );
      if ( autoPeakMode_ == AutoPeakOff ) return base;
      if ( !autoPeakIncluded_[ bin ] ) return 101.0f;
      float groupRms = maxIncludedRms_();
      if ( groupRms <= 0.0f ) return 101.0f;
      if ( autoPeakMode_ == AutoPeakFull ) return base * ( groupRms / 100.0f );
      // AutoPeakBin
      float thisRms = (float)valOrZero_( audioData_[ LevelRMS ][ bin ][ lightsShowIndex_ ] );
      float ratio = thisRms / groupRms;
      if ( ratio * 100.0f < AUTO_PEAK_MIN_INCLUSION_LEVEL ) return 101.0f;
      return base * ratio;
   }
   // Plain (non-auto-peak) threshold -- used by every getter/scan below
   // that ISN'T the real-time hit-acceptance decision above (predictive
   // lookahead, and the simple "is the hit level still fresh" nonzero
   // checks): the stored hit value itself already reflects whatever the
   // auto-peak rule above decided at the moment it was actually set,
   // re-deriving that same per-bin state here would be redundant.
   static float getPlainPeakThresholdPercent() { return constrain( peakThresholdPercent_, 0.0f, 100.0f ); }

   // Display/preview versions of the rule above, clamped back into 0-100
   // (the internal 101 "currently unreachable" sentinel would just read as
   // pinned at the very top of a meter, so clamp it there directly) --
   // NOT used for any real hit decision, only for showing where auto-peak
   // currently has a bin/band's threshold pinned.
   static float getBinAutoScaledPeakThresholdPercent( AudioBin bin ) {
      return min( 100.0f, getEffectivePeakThresholdPercent( bin ) );
   }
   static float getBandAutoScaledPeakThresholdPercent( AudioBand band = BandFull ) {
      if ( autoPeakMode_ == AutoPeakOff ) return constrain( peakThresholdPercent_, 0.0f, 100.0f );
      uint8_t n = BAND_BIN_COUNT[ band ];
      for ( uint8_t i = 0; i < n; ++i ) {
         if ( getEffectivePeakThresholdPercent( BAND_BINS[ band ][ i ] ) > 100.0f ) return 100.0f;
      }
      return constrain( peakThresholdPercent_, 0.0f, 100.0f );
   }

   static void pollFrequencies( uint32_t time ) {
     static uint32_t timestamp;
     // TODO: fiddle around with this value to find the right tuning
     if ( time - timestamp > POLL_INTERVAL_MS ) {
        timestamp = time;
        simModeActive_ = false;
        updateBandLevels( time, false, 0 );
     }
   }

   // alternative to pollFrequencies() for the Audio config screen's decay-
   // rate subsetting while showing the simulated signal (see
   // updateSimulatedBand() below) instead of real audio -- feeds the same
   // eased square wave into all bands, through the exact same pipeline
   // real audio uses, so the decay behavior being tuned here is the real
   // code path, not a separate mock. Gated at the same ~30ms cadence as
   // pollFrequencies() so both settings are exercised under the same real
   // poll-rate the hardware uses. Caller is responsible for calling this
   // INSTEAD OF pollFrequencies() while the simulated signal is active,
   // never both.
   static void pollSimulated( uint32_t time ) {
      static uint32_t timestamp;
      if ( time - timestamp > POLL_INTERVAL_MS ) {
         timestamp = time;
         simModeActive_ = true;
         uint8_t simRawAdc = updateSimulatedBand(); // 0-255 domain (see its own comment)
         uint8_t simRawPct = rawToPercent( simRawAdc );
         updateBandLevels( time, true, simRawPct );
      }
   }

   // simulated single-band waveform for the Audio config screen's decay-
   // rate subsetting (see CarpetLightLogic.cpp) -- lets the decay behavior
   // be tuned without needing real audio. Alternates high/low phases,
   // holding each phase's duration for 10 seconds before switching between
   // 40ms and 80ms phases. Only advances while actively polled
   // (updateSimulatedBand() must be called every frame the decay-rate
   // screen is showing the simulated signal) -- not run continuously in
   // the background. Returns 0-255 (this is the one place in the file
   // still in that domain, purely as a synthetic waveform generator --
   // pollSimulated() converts it to % immediately, see above).
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

uint8_t AudioBoard::audioData_[ NumLevelTypes ][ NumBins ][ AudioBoard::AUDIO_HISTORY_SIZE ]; // static storage, sentinel 0 at boot is fine (immediately overwritten before ever read externally)
uint32_t AudioBoard::audioTimestamps_[ AudioBoard::AUDIO_HISTORY_SIZE ] = { 0 };
uint16_t AudioBoard::audioReadIndex_ = 0;
uint16_t AudioBoard::audioCount_ = 0;
uint16_t AudioBoard::lightsShowIndex_ = 0;

const uint8_t AudioBoard::BAND_BIN_COUNT[ NumFreqBands ] = { 1, 1, 2, 3, 7 };
const AudioBin AudioBoard::BAND_BINS[ NumFreqBands ][ NumBins ] = {
   { FreqBin0 },                                                                    // BandBass
   { FreqBin1 },                                                                   // BandMidbass
   { FreqBin2, FreqBin3 },                                                      // BandMid
   { FreqBin4, FreqBin5, FreqBin6 },                                         // BandTreble
   { FreqBin0, FreqBin1, FreqBin2, FreqBin3, FreqBin4, FreqBin5, FreqBin6 }, // BandFull
};
float AudioBoard::hitDecayMs_ = 300.0f; // overwritten by Nvm at boot
Timer AudioBoard::hitTimer_;
bool AudioBoard::simModeActive_ = false;

bool AudioBoard::autoPeakIncluded_[ NumBins ] = { false };

float AudioBoard::hitPersist_[ NumBins ] = { 0.0f };
uint32_t AudioBoard::hitOnsetMs_[ NumBins ] = { 0 };
bool AudioBoard::hitWasAbove_[ NumBins ] = { false };
bool AudioBoard::newHitAtLightsShow_[ NumBins ] = { false };
uint32_t AudioBoard::lastReportedHitMs_[ NumBins ] = { 0 };
bool AudioBoard::lastReportedHitEverValid_[ NumBins ] = { false };

uint8_t AudioBoard::hitPredictionStyle_ = PredictExponential; // overwritten by Nvm at boot

float AudioBoard::agcBoost_[ NumBins ] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
float AudioBoard::agcBoostFull_ = 1.0f;

bool AudioBoard::autoGainSuppressed_ = false;
bool AudioBoard::soundReactivityEnabled_ = true;
uint8_t AudioBoard::agcMode_ = AGCoff; // overwritten by Nvm at boot
float AudioBoard::noiseFloorPercent_ = 19.5f; // overwritten by Nvm at boot
float AudioBoard::peakThresholdPercent_ = 31.0f; // approx. the old hardcoded 80/255, overwritten by Nvm at boot

// measured (scope, phone muted), volts -- see binNoiseFloorPercent_'s own
// comment. Hand-edit these if re-measured.
const float AudioBoard::BIN_NOISE_VOLTAGE[ NumBins ] = { 0.17f, 0.17f, 0.17f, 0.17f, 0.17f, 0.280f, 0.56f };
float AudioBoard::binNoiseFloorPercent_[ NumBins ] = { 19.5f, 19.5f, 19.5f, 19.5f, 19.5f, 19.5f, 19.5f }; // recomputed for real before first poll, see setNoiseFloorPercent()

uint8_t AudioBoard::autoPeakMode_ = AutoPeakBin; // overwritten by Nvm at boot

bool AudioBoard::silent_ = false;

float AudioBoard::audioForesightMs_ = 340.0f; // overwritten by Nvm at boot

uint32_t AudioBoard::simPhaseMs_ = 40;
Timer AudioBoard::simStateTimer_( 10000 );
Timer AudioBoard::simPhaseTimer_( 40 );
bool AudioBoard::simPhaseHigh_ = true;

#endif
