/* CarpetLedLogic.ino
 *
 *   This is the main entry point for the carpet's LED system.
 *
 *   Author: Anders Linn
 *   Date: July 2017
 */
#include "Utilities.h"
#include "MagicCarpet.h"

#include "AudioBoard.h"

// light shows
#include "DemoShow.h"
#include "FlameShow.h"
#include "NightriderShow.h"
#include "BumpingAudioShow.h"
#include "SpeedStripesShow.h"
#include "LighthouseShow.h"
#include "CarpetGeometry.h"

// The Flying Magic Carpet (TM)
MagicCarpet * carpet;

LightShow * currLightShow;

enum ShowMode { ShowNightrider = 0, ShowFlame = 1, ShowEqualizer = 2, ShowSpeedStripes = 3, ShowLighthouse = 4 };
static const uint8_t numModes = 5;
static ShowMode currMode = ShowNightrider;
static ShowMode prevMode = currMode;
static uint8_t currVariation[ numModes ] = { 0, 0, 0, 0, 0 };
static bool lightsOn = true; // always boots on; toggled by an extra-long press
static bool blacklightOn = false; // always boots off; toggled by a double press in ModeShow -- not persisted, same as lightsOn

// config mode cycle: long-press (from ModeShow) always lands on
// ModeConfigBrightness; each medium-press advances to the next one, wrapping
// back around. Each top-level setting can have multiple "subsettings";
// short-press cycles those. Double-press commits whichever subsetting is
// currently showing. Long-press is the only way to cancel.
enum AppMode { ModeShow, ModeConfigBrightness, ModeConfigAudio, ModeConfigPowerTest };
static AppMode appMode = ModeShow;
// configSubsetting's meaning depends on the current appMode -- it can't be a
// single enum type, so it stays a plain uint8_t, but each mode's own set of
// values is named below for readability at comparison sites.
enum BrightnessSubsetting { SubBrightnessGlobal = 0, SubBrightnessHeadlight = 1, SubBrightnessChina = 2 };
// noise floor and peak threshold used to be two separate subsettings (0/1),
// but short press had no bound meaning in either one beyond the generic
// "cycle to next subsetting" -- merged into one subsetting, with short press
// repurposed to toggle which of the two the pot currently targets instead
// (same kind of subsetting-specific short-press override as SubAudioHitDecay
// below), rather than spending a whole subsetting slot on each.
enum AudioSubsetting { SubAudioThreshold = 0, SubAudioHitDecay = 1, SubAudioHitPrediction = 2, SubAudioForesight = 3, SubAudioAutoGain = 4, SubAudioReactivityEnable = 5, SubAudioAutoPeak = 6 };
enum PowerTestSubsetting { SubPowerTestHue = 0, SubPowerTestSat = 1, SubPowerTestBrightness = 2 };
static uint8_t configSubsetting = 0;
static float noiseFloorPercent = 0.0f; // committed value; audio below this is "silence" -- see AudioBoard.h
static float peakThresholdPercent = 0.0f; // committed value; audio above this is a "hit"/peak -- see AudioBoard.h
// which of noiseFloorPercent/peakThresholdPercent the pot currently targets
// within SubAudioThreshold -- toggled by short press, always starts false
// (noise floor) on a fresh visit to the subsetting (see cycleSubsetting()).
static bool adjustingPeakThreshold = false;
static uint8_t liveAgcMode = AGCoff; // live (not-yet-committed) AGC mode, Audio subsetting SubAudioAutoGain -- see AGCMode in AudioBoard.h
// fixed-width (4 chars) like every other short console label in this file --
// "Off " keeps a trailing space so the live '\r' line never leaves a stray
// character behind when switching down from "Band"/"Full"
const char * agcModeName( uint8_t mode ) {
   if ( mode == AGCoff ) return "Off ";
   if ( mode == AGCband ) return "Band";
   return "Full"; // AGCfull
}
static bool liveSoundReactivityEnabled = true; // live (not-yet-committed) global reactivity toggle, Audio subsetting SubAudioReactivityEnable
static bool liveAutoPeakEnabled = false; // live (not-yet-committed) auto-peak toggle, Audio subsetting SubAudioAutoPeak -- see AudioBoard::autoPeakEnabled_
static float committedHitDecayMs = 300.0f; // committed value; ms for a "hit" to decay from full to zero -- see AudioBoard.h
static const float HIT_DECAY_RANGE_MS = 1000.0f; // adjustable range is 0-1000ms, per request
// SubAudioHitDecay and SubAudioHitPrediction share this same sim/live test
// setup (same square-wave signal, same VU-meter screen -- hit prediction's
// lookahead needs the same buffered history the decay test already
// exercises): starts in simulated-signal mode every fresh visit to EITHER
// one (see cycleSubsetting()); a short press while on either subsetting
// toggles to/from live audio instead of the usual "cycle to next
// subsetting" behavior (see loop()'s didShort handling) -- same kind of
// subsetting-specific override PowerTest already establishes for encoder
// rotation.
static bool audioSimMode = true;

static float committedHitPredictionMs = 0.0f; // committed value; predictive lead-up before an upcoming hit, capped to committedAudioForesightMs -- see AudioBoard.h
// SubAudioHitPrediction's pot adjusts the ms amount above; its encoder
// (otherwise idle on this subsetting) cycles which shape the lead-up
// follows instead -- same live/committed + revert-on-leave convention as
// liveAgcMode above, just encoder- instead of pot-driven.
static uint8_t committedHitPredictionStyle = 1; // PredictExponential, see AudioBoard.h's HitPredictionStyle
static uint8_t liveHitPredictionStyle = 1;

static float committedAudioForesightMs = 0.0f; // committed value; how far back in time sampled audio is looked up -- see AudioBoard.h
static const float AUDIO_FORESIGHT_RANGE_MS = 1000.0f; // adjustable range is 0-1000ms, per request

// PowerTest: an HSV test color, dialed in with the encoder (rotation edits
// whichever of hue/sat/brightness the current subsetting selects), used to
// visually compare a straight HSV->RGB rendering against an RGBW power-saving
// translation of the same color -- see MagicCarpet::showPowerTest(). The 3
// live fields persist across subsetting cycling within one visit to this
// screen (so dialing hue, then sat, then brightness edits one composite
// color), and re-sync to the committed color each time the screen is freshly
// entered (see enterConfigMode()) -- no explicit revert-on-cancel is needed
// beyond that, since these live fields have no effect on anything outside
// this screen.
static uint8_t committedTestHue = 0, committedTestSat = 255, committedTestBrightness = 255;
static uint8_t liveTestHue = 0, liveTestSat = 255, liveTestBrightness = 255;

// console output while in a config screen: enter announces once ("set
// GlobalMaxBr:42%"), a live line ("GlobalMaxBr:42%") re-prints in place via
// '\r' (not '\n') while adjusting so it visibly updates as the pot/encoder
// moves, and commit prints once ("ok GlobalMaxBr:42%"). Throttled to ~150ms
// rather than every loop iteration -- at 115200 baud (11.5KB/s) even the
// longest live line (noise floor/threshold's Lo/Md/Hi + 7 bins, ~70 chars)
// fits comfortably every loop iteration bandwidth-wise, but printing it
// every ~15ms is still pure waste: nobody can read a line updating that
// fast, and every Serial.print() call still costs real cycles even once the
// bytes themselves are cheap.
static Timer liveValuePrintTimer_( 150 );

// soft takeover: entering a config screen (or subsetting) shouldn't make the
// preview jump to wherever the pot/encoder physically happens to be sitting.
// It holds at the already-committed value until the pot moves noticeably (2%
// of its full range) or the encoder is twisted at all since entry -- then it
// hands over to live tracking.
static const float POT_TAKEOVER_THRESHOLD_PERCENT = 2.0f;
static float configEntryPotPercent = 0.0f;
static bool configPotTakenOver = false;

uint8_t numSubsettingsFor( AppMode mode ) {
   if ( mode == ModeConfigBrightness ) return 3;
   if ( mode == ModeConfigAudio ) return 7;
   if ( mode == ModeConfigPowerTest ) return 3;
   return 1;
}

// forward declaration -- defined further down, but enterConfigMode()/
// cycleSubsetting() below need to call it on entry
void printEnteringSetting( AppMode mode, uint8_t subsetting );

// re-syncs every takeover tracker to "nothing has changed yet" -- call
// whenever a new top-level setting or a new subsetting is entered
void resetTakeoverState() {
   configEntryPotPercent = carpet->pot->readLivePercent(); // config screens always see the true live pot, never the low-power simulation
   configPotTakenOver = false;
   carpet->encoder->resetPositionDelta();
   liveAgcMode = AudioBoard::getAgcMode();
   liveHitPredictionStyle = AudioBoard::getHitPredictionStyle();
   liveSoundReactivityEnabled = AudioBoard::getSoundReactivityEnabled();
   liveAutoPeakEnabled = AudioBoard::getAutoPeakEnabled();
}

void enterConfigMode( AppMode mode ) {
   appMode = mode;
   configSubsetting = 0;
   resetTakeoverState();
   if ( mode == ModeConfigPowerTest ) {
      // fresh visit -- start editing from the last committed color
      liveTestHue = committedTestHue;
      liveTestSat = committedTestSat;
      liveTestBrightness = committedTestBrightness;
   }
   printEnteringSetting( appMode, configSubsetting );
}

void cycleSubsetting() {
   configSubsetting = ( configSubsetting + 1 ) % numSubsettingsFor( appMode );
   resetTakeoverState();
   if ( appMode == ModeConfigAudio && configSubsetting == SubAudioThreshold ) {
      // fresh visit always starts targeting noise floor -- see
      // adjustingPeakThreshold's declaration comment
      adjustingPeakThreshold = false;
   } else if ( appMode == ModeConfigAudio && ( configSubsetting == SubAudioHitDecay || configSubsetting == SubAudioHitPrediction ) ) {
      // fresh visit to either screen always starts in simulated-signal
      // mode, per request -- see audioSimMode's declaration comment
      audioSimMode = true;
      AudioBoard::resetSimulatedBand();
   }
   printEnteringSetting( appMode, configSubsetting );
}

AppMode nextConfigMode( AppMode mode ) {
   switch ( mode ) {
      case ModeConfigBrightness: return ModeConfigAudio;
      case ModeConfigAudio:      return ModeConfigPowerTest;
      case ModeConfigPowerTest:  return ModeConfigBrightness;
      default:                  return ModeConfigBrightness;
   }
}

// maps the live pot reading linearly to a percent in [minPercent,maxPercent].
// fullRange=false restricts to the pot's top half (used by global brightness).
float potToPercent( float minPercent, float maxPercent, bool fullRange ) {
   float potPercent = carpet->pot->readLivePercent(); // config screens always see the true live pot, never the low-power simulation
   float frac;
   if ( fullRange ) {
      frac = potPercent / 100.0f;
   } else {
      frac = ( potPercent <= 50.0f ) ? 0.0f : ( ( potPercent - 50.0f ) / 50.0f );
   }
   return minPercent + frac * ( maxPercent - minPercent );
}

// the live value a config screen should currently show: the committed value,
// until the pot has moved POT_TAKEOVER_THRESHOLD_PERCENT from its reading
// when this screen was entered -- then it switches to tracking the pot live.
float livePercentFor( float minPercent, float maxPercent, bool fullRange, float committedPercent ) {
   if ( !configPotTakenOver ) {
      float potPercent = carpet->pot->readPercent();
      float diff = fabsf( potPercent - configEntryPotPercent );
      if ( diff >= POT_TAKEOVER_THRESHOLD_PERCENT ) configPotTakenOver = true;
   }
   if ( !configPotTakenOver ) return committedPercent;
   return potToPercent( minPercent, maxPercent, fullRange );
}

// display names for AudioBoard's HitPredictionStyle enum, per request:
// "exponential rise," "machine gun," "drum intro," disabled called "off"
const char * hitPredictionStyleName( uint8_t style ) {
   if ( style == PredictDisabled ) return "Off";
   if ( style == PredictExponential ) return "ExpRise";
   if ( style == PredictPulseTrain ) return "MachineGun";
   return "DrumIntro"; // PredictTwoPulse
}

// short CamelCase labels, no spaces -- keeps each print short (less time
// spent in blocking Serial.print() at 9600 baud) and easy to scan
const char * settingName( AppMode mode, uint8_t subsetting ) {
   if ( mode == ModeConfigBrightness ) {
      if ( subsetting == SubBrightnessGlobal ) return "GlobalMaxBr";
      if ( subsetting == SubBrightnessHeadlight ) return "HeadliteBr";
      return "ChinaBr";
   }
   if ( mode == ModeConfigAudio ) {
      if ( subsetting == SubAudioThreshold ) return adjustingPeakThreshold ? "PkThresh" : "NoiseFl";
      if ( subsetting == SubAudioAutoGain ) return "AGC";
      if ( subsetting == SubAudioHitDecay ) return "HitDecay";
      if ( subsetting == SubAudioHitPrediction ) return "HitPredict";
      if ( subsetting == SubAudioReactivityEnable ) return "SndReact";
      if ( subsetting == SubAudioAutoPeak ) return "AutoPk";
      return "Foresight";
   }
   if ( mode == ModeConfigPowerTest ) {
      if ( subsetting == SubPowerTestHue ) return "Hue";
      if ( subsetting == SubPowerTestSat ) return "Sat";
      return "Val";
   }
   return "?";
}

// fixed-width right-justified int prints -- used only on the '\r' live line
// so its length holds steady at the widest value that field can ever take,
// instead of jittering shorter/longer as digit count changes
void printPad3( int v ) { char buf[ 5 ]; snprintf( buf, sizeof( buf ), "%3d", v ); Serial.print( buf ); }
void printPad4( int v ) { char buf[ 6 ]; snprintf( buf, sizeof( buf ), "%4d", v ); Serial.print( buf ); }

// one-char marker for a percent (0-100) level: '^' above the peak threshold
// (a "hit", same threshold EqualizerShow's strobe triggers on -- '^' as the
// up-pointing opposite of '_'), '_' below the noise floor, '-' in between
void printLevelMarker( int percent ) {
   if ( percent > AudioBoard::getPeakThresholdPercent() ) Serial.print( "^" );
   else if ( percent < AudioBoard::getNoiseFloorPercent() ) Serial.print( "_" );
   else Serial.print( "-" );
}

// prints just the value (no label) -- shared by the entry announcement, the
// live line, and the commit confirmation, so all 3 format it identically.
// pad=true (live line only) right-justifies to the field's max width.
void printSettingValue( AppMode mode, uint8_t subsetting, float livePercent, bool pad = false ) {
   if ( mode == ModeConfigBrightness || ( mode == ModeConfigAudio && subsetting == SubAudioThreshold ) ) {
      int v = (int)( livePercent + 0.5f ); // 0-100
      if ( pad ) printPad3( v ); else Serial.print( v );
      Serial.print( "%" );
   } else if ( mode == ModeConfigAudio && subsetting == SubAudioAutoGain ) {
      Serial.print( agcModeName( liveAgcMode ) ); // "Off"/"Band"/"Full"
   } else if ( mode == ModeConfigAudio && subsetting == SubAudioReactivityEnable ) {
      Serial.print( liveSoundReactivityEnabled ? "Ena" : "Dis" ); // already fixed-width
   } else if ( mode == ModeConfigAudio && subsetting == SubAudioAutoPeak ) {
      Serial.print( liveAutoPeakEnabled ? "Ena" : "Dis" ); // already fixed-width
   } else if ( mode == ModeConfigAudio ) { // SubAudioHitDecay or SubAudioForesight: both in ms (0-1000, not a percent)
      int v = (int)( livePercent + 0.5f );
      if ( pad ) printPad4( v ); else Serial.print( v );
      Serial.print( "ms" );
   } else if ( mode == ModeConfigPowerTest ) {
      uint8_t v = ( subsetting == SubPowerTestHue ) ? liveTestHue : ( subsetting == SubPowerTestSat ) ? liveTestSat : liveTestBrightness; // 0-255
      if ( pad ) printPad3( v ); else Serial.print( v );
   }
}

// tracks whether the last thing on the console is a still-open '\r' live
// line -- if so, the next one-shot line must start with '\n' first, or its
// (often shorter) text would only partially overwrite the live line's
// leftover tail instead of starting clean
static bool liveLineOpen_ = false;

// one-time announcement on entering a screen or cycling to a new subsetting
// -- shows the CURRENT COMMITTED value (nothing has been adjusted yet)
void printEnteringSetting( AppMode mode, uint8_t subsetting ) {
   if ( liveLineOpen_ ) { Serial.print( "\n" ); liveLineOpen_ = false; }
   Serial.print( "set " );
   Serial.print( settingName( mode, subsetting ) );
   Serial.print( ":" );
   if ( mode == ModeConfigBrightness ) {
      float v = ( subsetting == SubBrightnessGlobal ) ? carpet->getGlobalBrightness() :
                ( subsetting == SubBrightnessHeadlight ) ? carpet->getHeadlightBrightness() : carpet->getChinaBrightness();
      Serial.print( (int)( v + 0.5f ) );
      Serial.println( "%" );
   } else if ( mode == ModeConfigAudio ) {
      if ( subsetting == SubAudioThreshold ) {
         Serial.print( (int)( ( adjustingPeakThreshold ? peakThresholdPercent : noiseFloorPercent ) + 0.5f ) );
         Serial.println( "%" );
      } else if ( subsetting == SubAudioAutoGain ) {
         Serial.println( agcModeName( AudioBoard::getAgcMode() ) );
      } else if ( subsetting == SubAudioReactivityEnable ) {
         Serial.println( AudioBoard::getSoundReactivityEnabled() ? "Ena" : "Dis" );
      } else if ( subsetting == SubAudioAutoPeak ) {
         Serial.println( AudioBoard::getAutoPeakEnabled() ? "Ena" : "Dis" );
      } else if ( subsetting == SubAudioHitDecay ) {
         Serial.print( (int)( committedHitDecayMs + 0.5f ) );
         Serial.println( "ms" );
      } else if ( subsetting == SubAudioHitPrediction ) {
         Serial.print( (int)( committedHitPredictionMs + 0.5f ) );
         Serial.print( "ms " );
         Serial.println( hitPredictionStyleName( committedHitPredictionStyle ) );
      } else { // SubAudioForesight
         Serial.print( (int)( committedAudioForesightMs + 0.5f ) );
         Serial.println( "ms" );
      }
   } else if ( mode == ModeConfigPowerTest ) {
      uint8_t v = ( subsetting == SubPowerTestHue ) ? committedTestHue : ( subsetting == SubPowerTestSat ) ? committedTestSat : committedTestBrightness;
      Serial.println( v );
   }
}

// MSGEQ7 datasheet center frequencies for the 7 hardware bins, in order
static const char * const BIN_FREQ_LABEL[ 7 ] = { "63", "160", "400", "1k", "2.5k", "6.25k", "16k" };

// live-updating line while adjusting -- ends in '\r' (not '\n') so it
// overwrites itself in place on the terminal instead of scrolling. nfloor
// additionally appends the 7 raw spectrum-shield bins on the SAME line (a
// genuinely separate, independently-live-updating second line isn't
// achievable on a plain serial terminal without ANSI cursor codes, since two
// '\r'-only lines with no '\n' between them just overwrite each other).
void printLiveValue( AppMode mode, uint8_t subsetting, float livePercent ) {
   if ( mode == ModeConfigAudio && subsetting == SubAudioThreshold ) { // noise floor / peak threshold, merged
      // this line is already packed (Lo/Md/Hi + 7 bins) -- skip the usual
      // "NoiseFl:<value>"/"PkThresh:<value>" prefix entirely here, by request, to save chars
      // curated bass/mid/treble derived from the raw bins below (see
      // AudioBoard::bandRawBinMax()) -- same values EqualizerShow/NightriderShow react to.
      // one-char marker after each: '^' above AudioBoard::PEAK_THRESHOLD (a
      // "hit", same threshold EqualizerShow's strobe triggers on), '_' below
      // the noise floor, '-' in between
      Serial.print( "Lo:" ); printPad3( AudioBoard::getBandNormalPercent( BandBass ) ); Serial.print( "%" ); printLevelMarker( AudioBoard::getBandRawPercent( BandBass ) );
      Serial.print( " Md:" ); printPad3( AudioBoard::getBandNormalPercent( BandMid ) ); Serial.print( "%" ); printLevelMarker( AudioBoard::getBandRawPercent( BandMid ) );
      Serial.print( " Hi:" ); printPad3( AudioBoard::getBandNormalPercent( BandTreble ) ); Serial.print( "%" ); printLevelMarker( AudioBoard::getBandRawPercent( BandTreble ) );
      // the 7 raw bins themselves, no label prefix (kept minimal)
      Serial.print( " " );
      for ( int i = 0; i < 7; ++i ) {
         Serial.print( BIN_FREQ_LABEL[ i ] );
         Serial.print( ":" );
         printPad4( AudioBoard::Frequencies_Mono[ i ] ); // 0-1023 (10-bit ADC read)
         if ( i < 6 ) Serial.print( " " );
      }
   } else {
      Serial.print( settingName( mode, subsetting ) );
      Serial.print( ":" );
      printSettingValue( mode, subsetting, livePercent, true );
   }
   Serial.print( "\r" ); // every field above is fixed-width, so the line holds steady length -- no extra pad needed
   liveLineOpen_ = true;
}

void printCommitted( AppMode mode, uint8_t subsetting, float livePercent ) {
   if ( liveLineOpen_ ) { Serial.print( "\n" ); liveLineOpen_ = false; }
   Serial.print( "ok " );
   Serial.print( settingName( mode, subsetting ) );
   Serial.print( ":" );
   printSettingValue( mode, subsetting, livePercent );
   Serial.println();
}

// undoes the live-preview side effect on AudioBoard's committed-facing state
// (noise floor / peak threshold) when leaving that subsetting any way other
// than committing (advancing, cycling away, or cancelling)
void revertAudioLivePreview() {
   if ( appMode != ModeConfigAudio ) return;
   if ( configSubsetting == SubAudioThreshold ) {
      if ( adjustingPeakThreshold ) AudioBoard::setPeakThresholdPercent( peakThresholdPercent );
      else AudioBoard::setNoiseFloorPercent( noiseFloorPercent );
   } else if ( configSubsetting == SubAudioHitDecay ) AudioBoard::setHitDecayMs( committedHitDecayMs );
   else if ( configSubsetting == SubAudioHitPrediction ) {
      AudioBoard::setHitPredictionMs( committedHitPredictionMs );
      AudioBoard::setHitPredictionStyle( committedHitPredictionStyle );
   } else if ( configSubsetting == SubAudioForesight ) AudioBoard::setAudioForesightMs( committedAudioForesightMs );
}

LightShow * makeShow( ShowMode mode, uint8_t variation ) {
   switch ( mode ) {
      case ShowNightrider:
         return new NightriderShow( carpet, variation );
      case ShowFlame:
         return new FlameShow( carpet, variation );
      case ShowEqualizer:
         return new EqualizerShow( carpet, variation, Nvm::loadedEqualizerStrobeEnabled() );
      case ShowSpeedStripes:
         return new SpeedStripesShow( carpet, variation );
      case ShowLighthouse:
         return new LighthouseShow( carpet, variation );
      default:
         // we fucked up, just reset
         // carpet->error(); // uncomment for debugging
         return new NightriderShow( carpet, variation );
   }
}

const char * showName( ShowMode mode ) {
   switch ( mode ) {
      case ShowNightrider:   return "Nightrider";
      case ShowFlame:        return "FlameSparkle";
      case ShowEqualizer:    return "Equalizer";
      case ShowSpeedStripes: return "SpeedStripes";
      case ShowLighthouse:   return "Lighthouse";
      default:               return "?";
   }
}

// vertically compact: one banner line, one line with every config setting's
// current value, instead of a line per value
void printWelcome() {
   Serial.println( "--FMC--" );
   Serial.print( "show:" ); Serial.print( showName( currMode ) );
   Serial.print( "(v" ); Serial.print( currVariation[ currMode ] ); Serial.print( ")" );
   Serial.print( " GlobalMaxBr:" ); Serial.print( (int)( carpet->getGlobalBrightness() + 0.5f ) ); Serial.print( "%" );
   Serial.print( " HeadliteBr:" ); Serial.print( (int)( carpet->getHeadlightBrightness() + 0.5f ) ); Serial.print( "%" );
   Serial.print( " ChinaBr:" ); Serial.print( (int)( carpet->getChinaBrightness() + 0.5f ) ); Serial.print( "%" );
   Serial.print( " NoiseFl:" ); Serial.print( (int)( noiseFloorPercent + 0.5f ) ); Serial.print( "%" );
   Serial.print( " AGC:" ); Serial.print( agcModeName( AudioBoard::getAgcMode() ) );
   Serial.print( " H:" ); Serial.print( committedTestHue );
   Serial.print( " S:" ); Serial.print( committedTestSat );
   Serial.print( " V:" ); Serial.print( committedTestBrightness );
   Serial.print( " EqStrobe:" ); Serial.println( Nvm::loadedEqualizerStrobeEnabled() ? "Ena" : "Dis" );
   Serial.print( "SpeedLink@0x69:" );
   Serial.println( SpeedLink::isFresh() ? "ok" : "none" );
}

void setup() {
   Serial.begin(115200); // Due's Programming Port UART -- independent of upload/bootloader protocol, safe to raise
   // Explicitly configures the real ADC hardware to ADC_RESOLUTION_BITS
   // (see Utilities.h) instead of relying on the Due core's own implicit
   // 10-bit default -- makes the assumption real/enforced rather than
   // incidental, and gives the WASM visualizer's simulated ADC quantization
   // step (which reads this same constant back via the bridge) one single
   // place to ever go out of sync with, instead of two independently-
   // hardcoded guesses (AudioBoard.h's scale() and LedController.h's
   // MAX_VOLTAGE used to each hardcode 1023 on their own).
   analogReadResolution( ADC_RESOLUTION_BITS );
   // AudioBoard::setup();

   // setup the carpet
   carpet = theMagicCarpet();
   carpet->setup(); // also loads persisted show/variation state from flash (Nvm::load())
   CarpetGeometry::begin(); // precomputes every fixture/neopixel's geometry once, before any show runs

   currMode = (ShowMode)Nvm::loadedShow();
   if ( currMode >= numModes ) currMode = ShowNightrider; // guard against stale/out-of-range flash data
   for ( uint8_t i = 0; i < numModes; ++i ) {
      currVariation[ i ] = Nvm::loadedVariation( i );
   }
   prevMode = currMode;
   carpet->setGlobalBrightness( Nvm::loadedGlobalBrightness() );
   carpet->setHeadlightBrightness( Nvm::loadedHeadlightBrightness() );
   carpet->setChinaBrightness( Nvm::loadedChinaBrightness() );
   noiseFloorPercent = Nvm::loadedNoiseFloor();
   AudioBoard::setNoiseFloorPercent( noiseFloorPercent );
   peakThresholdPercent = Nvm::loadedPeakThreshold();
   AudioBoard::setPeakThresholdPercent( peakThresholdPercent );
   AudioBoard::setAgcMode( Nvm::loadedAgcMode() );
   AudioBoard::setSoundReactivityEnabled( Nvm::loadedSoundReactivityEnabled() );
   AudioBoard::setAutoPeakEnabled( Nvm::loadedAutoPeakEnabled() );
   committedHitDecayMs = Nvm::loadedHitDecayMs();
   AudioBoard::setHitDecayMs( committedHitDecayMs );
   committedAudioForesightMs = Nvm::loadedAudioForesightMs();
   AudioBoard::setAudioForesightMs( committedAudioForesightMs );
   // must load after foresight above -- setHitPredictionMs()'s clamp uses
   // the currently-set audioForesightMs_ as its upper bound
   committedHitPredictionMs = Nvm::loadedHitPredictionMs();
   AudioBoard::setHitPredictionMs( committedHitPredictionMs );
   committedHitPredictionStyle = Nvm::loadedHitPredictionStyle();
   AudioBoard::setHitPredictionStyle( committedHitPredictionStyle );
   committedTestHue = liveTestHue = Nvm::loadedTestHue();
   committedTestSat = liveTestSat = Nvm::loadedTestSat();
   committedTestBrightness = liveTestBrightness = Nvm::loadedTestBrightness();

   currLightShow = makeShow( currMode, currVariation[ currMode ] );
   currLightShow->start();

   // give CANTroller2 a moment to send its first speed packet (it ticks
   // roughly every 250ms) so the boot banner can report whether the host is
   // actually present, instead of always saying "no host" before one arrives
   delay( 600 );

   printWelcome();
}

void loop() {
   static uint32_t clock;
   clock = millis();

   // Suppress auto-gain while dialing in noise floor/peak threshold --
   // reading either of those against a live-renormalizing signal is
   // confusing, since the gain keeps rescaling what you're trying to
   // measure against a fixed value. Set unconditionally from current state
   // every loop (not just on entry) so it can't get stuck on if this
   // subsetting is ever left through a path other than the usual
   // short/medium/long presses.
   AudioBoard::setAutoGainSuppressed( appMode == ModeConfigAudio && configSubsetting == SubAudioThreshold );

   // decay-rate/hit-prediction screens' simulated signal replaces real audio
   // polling entirely while it's active, never runs alongside it
   bool usingSimulatedAudio = ( appMode == ModeConfigAudio &&
         ( configSubsetting == SubAudioHitDecay || configSubsetting == SubAudioHitPrediction ) && audioSimMode );
   if ( usingSimulatedAudio ) {
      AudioBoard::pollSimulated( clock );
   } else {
      AudioBoard::pollFrequencies( clock );
   }

   carpet->encoder->update(); // refresh button short/medium/long/extra-long/double press state

   // press-hold feedback: single flash live the instant a hold crosses the
   // long-press threshold -- no feedback for crossing medium, by request
   if ( carpet->encoder->button.crossedLongThreshold() ) {
      carpet->flashRope( 1 );
   }

   // consume every press type exactly once per loop, regardless of mode, so a
   // press irrelevant to the current mode doesn't go stale and misfire later
   bool didShort = carpet->encoder->button.shortpress();
   bool didMedium = carpet->encoder->button.mediumpress();
   bool didLong = carpet->encoder->button.longpress();
   bool didExtraLong = carpet->encoder->button.extralongpress();
   bool didDouble = carpet->encoder->button.doublepress();

   if ( appMode == ModeConfigBrightness || appMode == ModeConfigAudio || appMode == ModeConfigPowerTest ) {
      // these 3 screens take over the whole visual with a live preview

      // Audio subsetting 2 (auto-gain toggle) and PowerTest (all 3 subsettings)
      // read encoder rotation -- this has to happen BEFORE currLightShow->update()
      // below, since the active show also consumes encoder rotation for its own
      // variation selection and would otherwise eat it first.
      if ( appMode == ModeConfigAudio && configSubsetting == SubAudioAutoGain ) {
         int delta = carpet->encoder->readPositionDelta();
         if ( delta != 0 ) {
            carpet->encoder->resetPositionDelta();
            // cycles through AGCoff -> AGCband -> AGCfull in that order, one
            // step per detent regardless of the detent's raw magnitude --
            // right advances, left goes back, CLAMPED at both ends (does
            // NOT loop around), per request. If this is backwards on the
            // actual hardware (readPositionDelta's sign depends on encoder
            // wiring, unverified), just flip this test.
            int newMode = (int)liveAgcMode + ( delta > 0 ? 1 : -1 );
            liveAgcMode = (uint8_t)constrain( newMode, 0, (int)NumAGCModes - 1 );
         }
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioReactivityEnable ) {
         int delta = carpet->encoder->readPositionDelta();
         if ( delta != 0 ) {
            carpet->encoder->resetPositionDelta();
            liveSoundReactivityEnabled = ( delta > 0 ); // same direct-pick convention as AutoGain above
         }
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioAutoPeak ) {
         int delta = carpet->encoder->readPositionDelta();
         if ( delta != 0 ) {
            carpet->encoder->resetPositionDelta();
            liveAutoPeakEnabled = ( delta > 0 ); // same direct-pick convention as SndReact above
         }
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitPrediction ) {
         // pot (below) adjusts the ms amount; encoder rotation here cycles
         // which shape the lead-up follows instead, wrapping through
         // AudioBoard's HitPredictionStyle enum -- see liveHitPredictionStyle's
         // declaration comment
         int delta = carpet->encoder->readPositionDelta();
         if ( delta != 0 ) {
            carpet->encoder->resetPositionDelta();
            int newStyle = ( (int)liveHitPredictionStyle + delta ) % (int)NUM_HIT_PREDICTION_STYLES;
            if ( newStyle < 0 ) newStyle += NUM_HIT_PREDICTION_STYLES;
            liveHitPredictionStyle = (uint8_t)newStyle;
            AudioBoard::setHitPredictionStyle( liveHitPredictionStyle ); // live feedback while adjusting; reverted below if not committed
         }
      } else if ( appMode == ModeConfigPowerTest ) {
         int delta = carpet->encoder->readPositionDelta();
         if ( delta != 0 ) {
            carpet->encoder->resetPositionDelta();
            static const int STEP = 4; // per detent; tune to taste
            if ( configSubsetting == SubPowerTestHue ) {
               liveTestHue = (uint8_t)( liveTestHue + delta * STEP ); // hue wraps naturally (uint8_t overflow)
            } else if ( configSubsetting == SubPowerTestSat ) {
               liveTestSat = (uint8_t)constrain( (int)liveTestSat + delta * STEP, 0, 255 );
            } else {
               liveTestBrightness = (uint8_t)constrain( (int)liveTestBrightness + delta * STEP, 0, 255 );
            }
         }
      }

      currLightShow->update( clock ); // keep the show progressing "invisibly" so it doesn't jump on cancel

      float livePercent = 0.0f;
      if ( appMode == ModeConfigBrightness ) {
         if ( configSubsetting == SubBrightnessGlobal ) {
            livePercent = livePercentFor( 0.0f, 100.0f, false, carpet->getGlobalBrightness() ); // pot's bottom half unused
            carpet->showSolidRed( livePercent );
         } else if ( configSubsetting == SubBrightnessHeadlight ) {
            livePercent = livePercentFor( 50.0f, 100.0f, true, carpet->getHeadlightBrightness() );
            carpet->showHeadlightPreview( livePercent );
         } else { // SubBrightnessChina
            livePercent = livePercentFor( 0.0f, 100.0f, true, carpet->getChinaBrightness() );
            carpet->showChinaPreview( livePercent );
         }
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitDecay ) {
         // decay-rate subsetting -- pot adjusts 0-1000ms (soft takeover, same
         // convention as every other subsetting), live-previewed on
         // AudioBoard immediately (reverted below if not committed), shown
         // via its own dedicated meter rather than showAudioMeter()
         livePercent = livePercentFor( 0.0f, HIT_DECAY_RANGE_MS, true, committedHitDecayMs );
         AudioBoard::setHitDecayMs( livePercent );
         carpet->showHitDecayMeter( audioSimMode, livePercent, HIT_DECAY_RANGE_MS );
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitPrediction ) {
         // pot adjusts 0-committedAudioForesightMs (dynamic range, can't
         // predict further ahead than foresight allows -- see
         // AudioBoard::setHitPredictionMs()'s clamp); encoder (above) cycles
         // the style live already. Reuses showHitDecayMeter()'s VU-meter
         // screen -- same square-wave test signal, same blue "current
         // setting's position" dot, just showing prediction's own value/
         // range instead of decay's.
         livePercent = livePercentFor( 0.0f, committedAudioForesightMs, true, committedHitPredictionMs );
         AudioBoard::setHitPredictionMs( livePercent );
         carpet->showHitDecayMeter( audioSimMode, livePercent, committedAudioForesightMs );
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioForesight ) {
         // foresight subsetting -- pot adjusts 0-1000ms (soft takeover, same
         // convention as every other subsetting), live-previewed on
         // AudioBoard immediately (reverted below if not committed) so the
         // megabars' hit-based glow reacts under the live setting as it's
         // tuned by ear/eye, shown via its own dedicated screen
         livePercent = livePercentFor( 0.0f, AUDIO_FORESIGHT_RANGE_MS, true, committedAudioForesightMs );
         AudioBoard::setAudioForesightMs( livePercent );
         carpet->showForesightAdjust( livePercent, AUDIO_FORESIGHT_RANGE_MS );
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioReactivityEnable ) {
         // per request: chinas play a fake, steady series of "hits" as
         // direct feedback for whichever state (Ena/Dis) is currently live
         // -- reactivity ON pulses them like a hit would; OFF leaves them
         // dark, demonstrating exactly what turning this off actually does
         // (rather than a meter/number, which wouldn't show the
         // consequence as directly).
         carpet->showSoundReactivityToggle( liveSoundReactivityEnabled );
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioAutoPeak ) {
         carpet->showAutoPeakToggle( liveAutoPeakEnabled );
      } else if ( appMode == ModeConfigAudio ) {
         // reference (committed) position for whichever of the two isn't
         // being actively adjusted right now -- both are always shown at
         // once (blue = noise floor, red = peak threshold) so you can see
         // where the other one sits while dialing in this one
         float showNoiseFloor = noiseFloorPercent;
         float showThreshold = peakThresholdPercent;

         if ( configSubsetting == SubAudioThreshold && !adjustingPeakThreshold ) {
            livePercent = livePercentFor( 0.0f, 100.0f, true, noiseFloorPercent );
            livePercent = min( livePercent, peakThresholdPercent ); // never cross the peak threshold
            AudioBoard::setNoiseFloorPercent( livePercent ); // live feedback while adjusting; reverted below if not committed
            showNoiseFloor = livePercent;
         } else if ( configSubsetting == SubAudioThreshold ) { // adjustingPeakThreshold
            livePercent = livePercentFor( 0.0f, 100.0f, true, peakThresholdPercent );
            livePercent = max( livePercent, noiseFloorPercent ); // never cross the noise floor
            AudioBoard::setPeakThresholdPercent( livePercent ); // live feedback while adjusting; reverted below if not committed
            showThreshold = livePercent;
         }

         uint8_t fullSpectrumLevel = AudioBoard::getFullSpectrum(
               ( configSubsetting == SubAudioAutoGain ? liveAgcMode : AudioBoard::getAgcMode() ) != AGCoff );
         carpet->showAudioMeter( (uint8_t)( (uint16_t)AudioBoard::getBandNormalPercent( BandTreble ) * 255 / 100 ),
                                 (uint8_t)( (uint16_t)AudioBoard::getBandNormalPercent( BandMid ) * 255 / 100 ),
                                 (uint8_t)( (uint16_t)AudioBoard::getBandNormalPercent( BandBass ) * 255 / 100 ),
                                 configSubsetting == SubAudioAutoGain, showNoiseFloor, showThreshold, liveAgcMode, fullSpectrumLevel );
      } else { // ModeConfigPowerTest
         carpet->showPowerTest( liveTestHue, liveTestSat, liveTestBrightness );
      }

      if ( liveValuePrintTimer_.expireset() ) printLiveValue( appMode, configSubsetting, livePercent );

      if ( didDouble ) {
         // commit whichever subsetting is currently showing
         if ( appMode == ModeConfigBrightness && configSubsetting == SubBrightnessGlobal ) {
            carpet->setGlobalBrightness( livePercent );
            Nvm::saveGlobalBrightness( (uint8_t)( livePercent + 0.5f ) );
         } else if ( appMode == ModeConfigBrightness && configSubsetting == SubBrightnessHeadlight ) {
            carpet->setHeadlightBrightness( livePercent );
            Nvm::saveHeadlightBrightness( (uint8_t)( livePercent + 0.5f ) );
         } else if ( appMode == ModeConfigBrightness ) { // SubBrightnessChina
            carpet->setChinaBrightness( livePercent );
            Nvm::saveChinaBrightness( (uint8_t)( livePercent + 0.5f ) );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioThreshold && !adjustingPeakThreshold ) {
            noiseFloorPercent = livePercent;
            Nvm::saveNoiseFloor( (uint8_t)( noiseFloorPercent + 0.5f ) );
            AudioBoard::setNoiseFloorPercent( noiseFloorPercent );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioThreshold ) { // adjustingPeakThreshold
            peakThresholdPercent = livePercent;
            Nvm::savePeakThreshold( (uint8_t)( peakThresholdPercent + 0.5f ) );
            AudioBoard::setPeakThresholdPercent( peakThresholdPercent );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioAutoGain ) {
            AudioBoard::setAgcMode( liveAgcMode );
            Nvm::saveAgcMode( liveAgcMode );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioReactivityEnable ) {
            AudioBoard::setSoundReactivityEnabled( liveSoundReactivityEnabled );
            Nvm::saveSoundReactivityEnabled( liveSoundReactivityEnabled );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioAutoPeak ) {
            AudioBoard::setAutoPeakEnabled( liveAutoPeakEnabled );
            Nvm::saveAutoPeakEnabled( liveAutoPeakEnabled );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitDecay ) {
            committedHitDecayMs = livePercent;
            Nvm::saveHitDecayMs( (uint16_t)( committedHitDecayMs + 0.5f ) );
            AudioBoard::setHitDecayMs( committedHitDecayMs );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitPrediction ) {
            // commits both the pot-driven ms amount and the encoder-driven
            // style together, same as PowerTest's hue/sat/brightness triple
            committedHitPredictionMs = livePercent;
            Nvm::saveHitPredictionMs( (uint16_t)( committedHitPredictionMs + 0.5f ) );
            AudioBoard::setHitPredictionMs( committedHitPredictionMs );
            committedHitPredictionStyle = liveHitPredictionStyle;
            Nvm::saveHitPredictionStyle( committedHitPredictionStyle );
            AudioBoard::setHitPredictionStyle( committedHitPredictionStyle );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioForesight ) {
            committedAudioForesightMs = livePercent;
            Nvm::saveAudioForesightMs( (uint16_t)( committedAudioForesightMs + 0.5f ) );
            AudioBoard::setAudioForesightMs( committedAudioForesightMs );
         } else if ( appMode == ModeConfigPowerTest ) { // commits the whole hue/sat/brightness triple at once
            committedTestHue = liveTestHue;
            committedTestSat = liveTestSat;
            committedTestBrightness = liveTestBrightness;
            Nvm::saveTestHue( committedTestHue );
            Nvm::saveTestSat( committedTestSat );
            Nvm::saveTestBrightness( committedTestBrightness );
         }
         printCommitted( appMode, configSubsetting, livePercent );
         carpet->flashRope( 1 ); // confirms the setting was committed
         appMode = ModeShow;
      } else if ( didMedium ) {
         revertAudioLivePreview();
         carpet->flashRope( 2 ); // confirms advancing to the next config setting
         enterConfigMode( nextConfigMode( appMode ) );
      } else if ( didShort ) {
         // BUGFIX: subsettings SubAudioThreshold/SubAudioHitDecay/
         // SubAudioHitPrediction override short-press's usual "cycle to
         // next subsetting" meaning with their own toggle (which target the
         // pot controls / simulated vs live signal) -- but the old code
         // NEVER fell through to cycleSubsetting() from any of these three,
         // so once Audio config was entered (always landing on
         // SubAudioThreshold first), short press could never advance past
         // it. SubAudioHitDecay/SubAudioHitPrediction/SubAudioForesight/
         // SubAudioAutoGain were consequently unreachable through normal
         // navigation -- almost certainly the actual bug behind "AGC enable
         // missing from config." Fixed the same way for all three: toggle
         // as before, but once the toggle returns to its own default
         // (noise floor / simulated signal), this same short press ALSO
         // advances to the next subsetting instead of stopping -- so it
         // takes 2 short presses to see both states before moving on,
         // rather than looping on one subsetting forever.
         if ( appMode == ModeConfigAudio && configSubsetting == SubAudioThreshold ) {
            revertAudioLivePreview(); // undo whichever target's live preview was applied before switching
            if ( !adjustingPeakThreshold ) {
               adjustingPeakThreshold = true;
               resetTakeoverState(); // fresh soft takeover for the newly-selected target
               printEnteringSetting( appMode, configSubsetting );
            } else {
               adjustingPeakThreshold = false; // back to default before advancing
               cycleSubsetting();
            }
         } else if ( appMode == ModeConfigAudio && ( configSubsetting == SubAudioHitDecay || configSubsetting == SubAudioHitPrediction ) ) {
            if ( audioSimMode ) {
               audioSimMode = false;
            } else {
               audioSimMode = true; // back to default before advancing
               AudioBoard::resetSimulatedBand();
               cycleSubsetting();
            }
         } else {
            revertAudioLivePreview();
            cycleSubsetting();
         }
      } else if ( didLong ) {
         revertAudioLivePreview();
         appMode = ModeShow; // cancel, no save
      }

   } else {
      // ModeShow

      if ( didExtraLong ) lightsOn = !lightsOn;
      if ( didDouble ) {
         if ( currMode == ShowEqualizer ) {
            // Equalizer show active: double press toggles its triple-strobe
            // setting instead of blacklight (see EqualizerShow::updateStrobe())
            EqualizerShow * eq = static_cast<EqualizerShow *>( currLightShow );
            bool newState = !eq->getStrobeEnabled();
            eq->setStrobeEnabled( newState );
            Nvm::saveEqualizerStrobeEnabled( newState );
         } else {
            blacklightOn = !blacklightOn; // china UV channel, full on/off -- see MagicCarpet::setBlacklight()
         }
      }
      if ( didLong ) enterConfigMode( ModeConfigBrightness );
      if ( didShort ) {
         currMode = (ShowMode)( ( (uint8_t)currMode + 1 ) % numModes );
         Nvm::saveShow( currMode );
      }

      if ( currMode != prevMode ) {
         // leaving Equalizer: revert AudioBoard's peak threshold back to the
         // last-committed value, undoing whatever its own pot binding left
         // live (see EqualizerShow::update()) -- same revert-on-leave
         // philosophy as the config screens' live previews
         if ( prevMode == ShowEqualizer ) AudioBoard::setPeakThresholdPercent( peakThresholdPercent );
         delete currLightShow;
         currLightShow = makeShow( currMode, currVariation[ currMode ] );
         currLightShow->start();
         prevMode = currMode;
         blacklightOn = false; // a show change disables blacklight if it was left on
         Serial.print( "show:" ); Serial.println( showName( currMode ) ); // abbreviated, on selection
      }

      // low-power pot simulation only applies here, never during config
      // screens -- see LedController.h's Potentiometer::updateLowPower()
      carpet->pot->updateLowPower( SpeedLink::isLowPower() );

      currLightShow->update( clock );

      // persist a show's variation the moment it actually changes
      uint8_t variation = currLightShow->variation();
      if ( variation != currVariation[ currMode ] ) {
         currVariation[ currMode ] = variation;
         Nvm::saveVariation( currMode, variation );
         Serial.print( showName( currMode ) ); Serial.print( ":" ); Serial.println( currLightShow->variationName() );
      }

      carpet->applyBrightnessCeiling();
      carpet->setBlacklight( blacklightOn );

      if ( !lightsOn ) carpet->clear(); // master override -- still let shows run "invisibly" underneath (also zeroes blacklight)
   }

   carpet->show();
}
