/* Nvm.h
 *
 *    Persists the selected light show, each show's variation, across power
 *    cycles. The Due has no real EEPROM, so this rides on DueFlashStorage,
 *    which reserves a block of program flash for small amounts of data.
 *    Note (from that library): re-uploading firmware wipes this data, but it
 *    survives ordinary power-off/power-on.
 */

#ifndef __NVM_H
#define __NVM_H

#include <DueFlashStorage.h>

namespace Nvm {

static DueFlashStorage flash;
static const uint8_t MAGIC = 0x47; // bump this if State's layout ever changes
static const uint8_t MAX_SHOWS = 8; // headroom for future shows, no relayout needed

// all brightness/threshold fields are percentages (0-100), never raw 0-255
// hardware values -- see MagicCarpet's setGlobalBrightness()/etc.
struct State {
   uint8_t magic;
   uint8_t currShow;
   uint8_t variation[ MAX_SHOWS ];
   uint8_t globalBrightnessPercent;    // 0-100, default 100 (unrestricted)
   uint8_t headlightBrightnessPercent; // 50-100, per-fixture, default 50 (see MagicCarpet.h)
   uint8_t chinaBrightnessPercent;     // 0-100, default 100 (unrestricted)
   uint8_t noiseFloorPercent;          // 0-100, default 0 -- audio below this is "silence" (see AudioBoard.h)
   uint8_t peakThresholdPercent;       // 0-100, default 31 (approx. the old hardcoded 80/255) -- audio above
                                        // this counts as a "hit"/peak (see AudioBoard.h::PEAK_THRESHOLD)
   uint8_t agcMode;                    // AGCoff/AGCband/AGCfull, default AGCoff -- see AudioBoard.h's AGCMode
   uint8_t testHue;                    // 0-255, default 0 -- power-saving test color (see CarpetLightLogic.cpp)
   uint8_t testSat;                    // 0-255, default 255
   uint8_t testBrightness;             // 0-255, default 255
   uint8_t equalizerStrobeEnabled;     // 0/1, default 0 (off) -- see BumpingAudioShow.h's EqualizerShow
   uint16_t hitDecayMs;                // 0-1000, default 300 -- ms for a "hit" to decay from full to zero (see AudioBoard.h)
   uint16_t audioForesightMs;          // 0-1000, default 0 -- how far back in time sampled audio is looked up (see AudioBoard.h)
   uint16_t hitPredictionMs;           // 0-audioForesightMs, default 0 -- predictive lead-up before an upcoming hit (see AudioBoard.h)
   uint8_t hitPredictionStyle;         // 0=disabled/1=exponential/2=pulse train, default 1 (see AudioBoard.h's HitPredictionStyle)
   uint8_t soundReactivityEnabled;     // 0/1, default 1 (on) -- global kill switch for all sound-reactive light behavior (see AudioBoard.h)
};

static State state;

// call once at boot, before reading any loaded*() value
inline void load() {
   memcpy( &state, flash.readAddress( 0 ), sizeof( State ) );
   if ( state.magic != MAGIC ) {
      // first boot ever, or the layout changed since the last save -- reset to defaults
      state.magic = MAGIC;
      state.currShow = 0;
      for ( uint8_t i = 0; i < MAX_SHOWS; ++i ) state.variation[ i ] = 0;
      state.globalBrightnessPercent = 100;
      state.headlightBrightnessPercent = 50;
      state.chinaBrightnessPercent = 100;
      state.noiseFloorPercent = 0;
      state.peakThresholdPercent = 31;
      state.agcMode = AGCoff;
      state.testHue = 0;
      state.testSat = 255;
      state.testBrightness = 255;
      state.equalizerStrobeEnabled = 0;
      state.hitDecayMs = 300;
      state.audioForesightMs = 0;
      state.hitPredictionMs = 0;
      state.hitPredictionStyle = 1; // PredictExponential
      state.soundReactivityEnabled = 1;
      flash.write( 0, (byte *)&state, sizeof( State ) );
   }
}

inline uint8_t loadedShow() {
   return state.currShow;
}

inline uint8_t loadedVariation( uint8_t show ) {
   return show < MAX_SHOWS ? state.variation[ show ] : 0;
}

inline uint8_t loadedGlobalBrightness() {
   return state.globalBrightnessPercent;
}

inline uint8_t loadedHeadlightBrightness() {
   return state.headlightBrightnessPercent;
}

inline uint8_t loadedChinaBrightness() {
   return state.chinaBrightnessPercent;
}

inline uint8_t loadedNoiseFloor() {
   return state.noiseFloorPercent;
}

inline uint8_t loadedPeakThreshold() {
   return state.peakThresholdPercent;
}

inline uint8_t loadedAgcMode() {
   return state.agcMode;
}

inline uint8_t loadedTestHue() {
   return state.testHue;
}

inline uint8_t loadedTestSat() {
   return state.testSat;
}

inline uint8_t loadedTestBrightness() {
   return state.testBrightness;
}

inline bool loadedEqualizerStrobeEnabled() {
   return state.equalizerStrobeEnabled != 0;
}

inline uint16_t loadedHitDecayMs() {
   return state.hitDecayMs;
}

inline uint16_t loadedAudioForesightMs() {
   return state.audioForesightMs;
}

inline uint16_t loadedHitPredictionMs() {
   return state.hitPredictionMs;
}

inline uint8_t loadedHitPredictionStyle() {
   return state.hitPredictionStyle;
}

inline bool loadedSoundReactivityEnabled() {
   return state.soundReactivityEnabled != 0;
}

inline void saveShow( uint8_t show ) {
   state.currShow = show;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveVariation( uint8_t show, uint8_t variation ) {
   if ( show >= MAX_SHOWS ) return;
   state.variation[ show ] = variation;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveGlobalBrightness( uint8_t percent ) {
   state.globalBrightnessPercent = percent;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveHeadlightBrightness( uint8_t percent ) {
   state.headlightBrightnessPercent = percent;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveChinaBrightness( uint8_t percent ) {
   state.chinaBrightnessPercent = percent;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveNoiseFloor( uint8_t percent ) {
   state.noiseFloorPercent = percent;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void savePeakThreshold( uint8_t percent ) {
   state.peakThresholdPercent = percent;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveAgcMode( uint8_t mode ) {
   state.agcMode = mode;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveTestHue( uint8_t hue ) {
   state.testHue = hue;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveTestSat( uint8_t sat ) {
   state.testSat = sat;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveTestBrightness( uint8_t brightness ) {
   state.testBrightness = brightness;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveEqualizerStrobeEnabled( bool enabled ) {
   state.equalizerStrobeEnabled = enabled ? 1 : 0;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveHitDecayMs( uint16_t ms ) {
   state.hitDecayMs = ms;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveAudioForesightMs( uint16_t ms ) {
   state.audioForesightMs = ms;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveHitPredictionMs( uint16_t ms ) {
   state.hitPredictionMs = ms;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveHitPredictionStyle( uint8_t style ) {
   state.hitPredictionStyle = style;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

inline void saveSoundReactivityEnabled( bool enabled ) {
   state.soundReactivityEnabled = enabled ? 1 : 0;
   flash.write( 0, (byte *)&state, sizeof( State ) );
}

} // end namespace Nvm

#endif
