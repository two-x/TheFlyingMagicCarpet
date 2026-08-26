/* LoopTime.h
 *
 * Tracks main-loop iteration timing. Two purposes: (1) feeds
 * AudioBoard::updateBandLevels()'s lightsShowTime target, correcting for
 * the main loop's own render-to-photon delay -- the gap between "this
 * poll computed levelHit for lightsShowTime" (near the top of loop()) and
 * "the LEDs for that same iteration actually finished transmitting"
 * (FastLED.show(), near the bottom) is essentially one loop iteration's
 * own duration, and is a real, separate contributor to audio/light sync
 * error on top of whatever audioForesightMs_ already corrects for --
 * FastLED.show()'s own blocking cost alone is documented at ~14.4ms, well
 * within the range humans can perceive as audio/visual desync. (2) general
 * diagnostics (most-recent/EMA/max-ever/trailing-window-peak getters).
 */

#ifndef __LOOP_TIME_H
#define __LOOP_TIME_H

#include "Utilities.h"

class LoopTime {
 private:
   static uint32_t lastCallMs_;
   static bool everCalled_;

   static uint32_t mostRecentMs_;
   static float emaMs_;
   // EMA time constant: loop time is fairly stable WITHIN a given code path
   // (same show, same config screen) -- this mainly needs to settle to a
   // new steady-state within about a second after a real change (switching
   // shows, entering/leaving a heavier config screen), not react instantly
   // frame-to-frame. 750ms splits "fast enough to matter for the
   // lightsShowTime correction above" and "slow enough not to jitter."
   static const uint16_t EMA_TAU_MS = 750;

   static uint32_t maxEverMs_;
   static uint32_t setupDoneAtMs_;
   static bool recordingMaxEver_; // false until 2s after markSetupDone() -- excludes setup()'s own boot-wait/first-frame irregularities

   // trailing-2s windowed peak -- ring buffer sized against a conservative
   // 10ms fastest-possible-loop floor (real hardware's own FastLED.show()
   // alone already costs ~14.4ms every iteration, so loop time should never
   // actually get close to this floor -- margin, not an expected case).
   static const uint16_t WINDOW_MS = 2000;
   static const uint16_t WINDOW_BUFFER_SIZE = WINDOW_MS / 10 + 10;
   static uint32_t windowSamples_[ WINDOW_BUFFER_SIZE ];
   static uint32_t windowTimestamps_[ WINDOW_BUFFER_SIZE ];
   static uint16_t windowHead_;
   static uint16_t windowCount_;
   static uint32_t windowPeakMs_;

 public:
   // call once, at the end of setup() -- starts the 2s warm-up clock
   // max-ever recording waits out before it starts actually recording
   static void markSetupDone( uint32_t nowMs ) {
      setupDoneAtMs_ = nowMs;
   }

   // call once, at a fixed point every loop() iteration -- the very top is
   // simplest and most representative of the real iteration-to-iteration
   // gap this class exists to measure
   static void update( uint32_t nowMs ) {
      if ( !everCalled_ ) { lastCallMs_ = nowMs; everCalled_ = true; return; }
      uint32_t dt = nowMs - lastCallMs_;
      lastCallMs_ = nowMs;
      mostRecentMs_ = dt;

      float alpha = (float)dt / ( (float)EMA_TAU_MS + (float)dt );
      emaMs_ = emaMs_ + alpha * ( (float)dt - emaMs_ );

      if ( !recordingMaxEver_ && ( nowMs - setupDoneAtMs_ ) >= 2000 ) recordingMaxEver_ = true;
      if ( recordingMaxEver_ && dt > maxEverMs_ ) maxEverMs_ = dt;

      windowSamples_[ windowHead_ ] = dt;
      windowTimestamps_[ windowHead_ ] = nowMs;
      windowHead_ = (uint16_t)( ( windowHead_ + 1 ) % WINDOW_BUFFER_SIZE );
      if ( windowCount_ < WINDOW_BUFFER_SIZE ) ++windowCount_;
      uint32_t peak = 0;
      for ( uint16_t i = 0; i < windowCount_; ++i ) {
         if ( ( nowMs - windowTimestamps_[ i ] ) <= WINDOW_MS && windowSamples_[ i ] > peak ) peak = windowSamples_[ i ];
      }
      windowPeakMs_ = peak;
   }

   static uint32_t getMostRecentMs() { return mostRecentMs_; }
   static float getEmaMs() { return emaMs_; }
   static uint32_t getMaxEverMs() { return maxEverMs_; }
   static uint32_t getWindowPeakMs() { return windowPeakMs_; }
};

uint32_t LoopTime::lastCallMs_ = 0;
bool LoopTime::everCalled_ = false;
uint32_t LoopTime::mostRecentMs_ = 0;
float LoopTime::emaMs_ = 0.0f;
uint32_t LoopTime::maxEverMs_ = 0;
uint32_t LoopTime::setupDoneAtMs_ = 0;
bool LoopTime::recordingMaxEver_ = false;
uint32_t LoopTime::windowSamples_[ LoopTime::WINDOW_BUFFER_SIZE ] = { 0 };
uint32_t LoopTime::windowTimestamps_[ LoopTime::WINDOW_BUFFER_SIZE ] = { 0 };
uint16_t LoopTime::windowHead_ = 0;
uint16_t LoopTime::windowCount_ = 0;
uint32_t LoopTime::windowPeakMs_ = 0;

#endif
