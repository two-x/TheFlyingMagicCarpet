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
enum AudioSubsetting { SubAudioThreshold = 0, SubAudioHitDecay = 1, SubAudioForesight = 2, SubAudioAutoGain = 3 };
enum PowerTestSubsetting { SubPowerTestHue = 0, SubPowerTestSat = 1, SubPowerTestBrightness = 2 };
static uint8_t configSubsetting = 0;
static float noiseFloorPercent = 0.0f; // committed value; audio below this is "silence" -- see AudioBoard.h
static float peakThresholdPercent = 0.0f; // committed value; audio above this is a "hit"/peak -- see AudioBoard.h
// which of noiseFloorPercent/peakThresholdPercent the pot currently targets
// within SubAudioThreshold -- toggled by short press, always starts false
// (noise floor) on a fresh visit to the subsetting (see cycleSubsetting()).
static bool adjustingPeakThreshold = false;
static bool liveAutoGainEnabled = false; // live (not-yet-committed) auto-gain toggle state, Audio subsetting SubAudioAutoGain
static float committedHitDecayMs = 300.0f; // committed value; ms for a "hit" to decay from full to zero -- see AudioBoard.h
static const float HIT_DECAY_RANGE_MS = 1000.0f; // adjustable range is 0-1000ms, per request
// Audio subsetting 3 (decay rate): starts in simulated-signal mode every
// fresh visit (see cycleSubsetting()); a short press while on this
// subsetting toggles to/from live audio instead of the usual "cycle to next
// subsetting" behavior (see loop()'s didShort handling) -- same kind of
// subsetting-specific override PowerTest already establishes for encoder
// rotation.
static bool audioSimMode = true;

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
   if ( mode == ModeConfigAudio ) return 4;
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
   liveAutoGainEnabled = AudioBoard::getAutoGainEnabled();
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
   } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitDecay ) {
      // fresh visit to the decay-rate screen always starts in simulated-
      // signal mode, per request -- see audioSimMode's declaration comment
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
      Serial.print( liveAutoGainEnabled ? "Ena" : "Dis" ); // already fixed-width
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
         Serial.println( AudioBoard::getAutoGainEnabled() ? "Ena" : "Dis" );
      } else if ( subsetting == SubAudioHitDecay ) {
         Serial.print( (int)( committedHitDecayMs + 0.5f ) );
         Serial.println( "ms" );
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
      // curated 0-255 low/mid/high derived from the raw bins below (see
      // AudioBoard::Into_3_Bins()) -- same values EqualizerShow/NightriderShow react to.
      // one-char marker after each: '^' above AudioBoard::PEAK_THRESHOLD (a
      // "hit", same threshold EqualizerShow's strobe triggers on), '_' below
      // the noise floor, '-' in between
      Serial.print( "Lo:" ); printPad3( AudioBoard::getNormalPercent( BandLow ) ); Serial.print( "%" ); printLevelMarker( AudioBoard::getRawPercent( BandLow ) );
      Serial.print( " Md:" ); printPad3( AudioBoard::getNormalPercent( BandMid ) ); Serial.print( "%" ); printLevelMarker( AudioBoard::getRawPercent( BandMid ) );
      Serial.print( " Hi:" ); printPad3( AudioBoard::getNormalPercent( BandHigh ) ); Serial.print( "%" ); printLevelMarker( AudioBoard::getRawPercent( BandHigh ) );
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
   else if ( configSubsetting == SubAudioForesight ) AudioBoard::setAudioForesightMs( committedAudioForesightMs );
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
   Serial.print( " AGC:" ); Serial.print( AudioBoard::getAutoGainEnabled() ? "Ena" : "Dis" );
   Serial.print( " H:" ); Serial.print( committedTestHue );
   Serial.print( " S:" ); Serial.print( committedTestSat );
   Serial.print( " V:" ); Serial.print( committedTestBrightness );
   Serial.print( " EqStrobe:" ); Serial.println( Nvm::loadedEqualizerStrobeEnabled() ? "Ena" : "Dis" );
   Serial.print( "SpeedLink@0x69:" );
   Serial.println( SpeedLink::isFresh() ? "ok" : "none" );
}

void setup() {
   Serial.begin(115200); // Due's Programming Port UART -- independent of upload/bootloader protocol, safe to raise
   // AudioBoard::setup();

   // setup the carpet
   carpet = theMagicCarpet();
   carpet->setup(); // also loads persisted show/variation state from flash (Nvm::load())

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
   AudioBoard::setAutoGainEnabled( Nvm::loadedAutoGainEnabled() );
   committedHitDecayMs = Nvm::loadedHitDecayMs();
   AudioBoard::setHitDecayMs( committedHitDecayMs );
   committedAudioForesightMs = Nvm::loadedAudioForesightMs();
   AudioBoard::setAudioForesightMs( committedAudioForesightMs );
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

   // decay-rate screen's simulated signal replaces real audio polling
   // entirely while it's active, never runs alongside it
   bool usingSimulatedAudio = ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitDecay && audioSimMode );
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
            // direction picks the state directly, not a toggle: right (positive
            // delta) always turns on, left (negative delta) always turns off.
            // if this is backwards on the actual hardware (readPositionDelta's
            // sign depends on encoder wiring, unverified), just flip this test.
            liveAutoGainEnabled = ( delta > 0 );
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
      } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioForesight ) {
         // foresight subsetting -- pot adjusts 0-1000ms (soft takeover, same
         // convention as every other subsetting), live-previewed on
         // AudioBoard immediately (reverted below if not committed) so the
         // megabars' hit-based glow reacts under the live setting as it's
         // tuned by ear/eye, shown via its own dedicated screen
         livePercent = livePercentFor( 0.0f, AUDIO_FORESIGHT_RANGE_MS, true, committedAudioForesightMs );
         AudioBoard::setAudioForesightMs( livePercent );
         carpet->showForesightAdjust( livePercent, AUDIO_FORESIGHT_RANGE_MS );
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
               configSubsetting == SubAudioAutoGain ? liveAutoGainEnabled : AudioBoard::getAutoGainEnabled() );
         carpet->showAudioMeter( (uint8_t)( (uint16_t)AudioBoard::getNormalPercent( BandHigh ) * 255 / 100 ),
                                 (uint8_t)( (uint16_t)AudioBoard::getNormalPercent( BandMid ) * 255 / 100 ),
                                 (uint8_t)( (uint16_t)AudioBoard::getNormalPercent( BandLow ) * 255 / 100 ),
                                 configSubsetting == SubAudioAutoGain, showNoiseFloor, showThreshold, liveAutoGainEnabled, fullSpectrumLevel );
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
            AudioBoard::setAutoGainEnabled( liveAutoGainEnabled );
            Nvm::saveAutoGainEnabled( liveAutoGainEnabled );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitDecay ) {
            committedHitDecayMs = livePercent;
            Nvm::saveHitDecayMs( (uint16_t)( committedHitDecayMs + 0.5f ) );
            AudioBoard::setHitDecayMs( committedHitDecayMs );
         } else if ( appMode == ModeConfigAudio ) { // SubAudioForesight
            committedAudioForesightMs = livePercent;
            Nvm::saveAudioForesightMs( (uint16_t)( committedAudioForesightMs + 0.5f ) );
            AudioBoard::setAudioForesightMs( committedAudioForesightMs );
         } else { // ModeConfigPowerTest -- commits the whole hue/sat/brightness triple at once
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
         if ( appMode == ModeConfigAudio && configSubsetting == SubAudioThreshold ) {
            // this subsetting overrides short-press's usual "cycle to next
            // subsetting" meaning -- it toggles which of noise floor/peak
            // threshold the pot targets instead (see adjustingPeakThreshold's
            // declaration comment)
            revertAudioLivePreview(); // undo whichever target's live preview was applied before switching
            adjustingPeakThreshold = !adjustingPeakThreshold;
            resetTakeoverState(); // fresh soft takeover for the newly-selected target
            printEnteringSetting( appMode, configSubsetting );
         } else if ( appMode == ModeConfigAudio && configSubsetting == SubAudioHitDecay ) {
            // this subsetting overrides short-press's usual "cycle to next
            // subsetting" meaning -- it toggles between the simulated and
            // live signal instead (same kind of subsetting-specific
            // override PowerTest establishes for encoder rotation)
            audioSimMode = !audioSimMode;
            if ( audioSimMode ) AudioBoard::resetSimulatedBand();
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
