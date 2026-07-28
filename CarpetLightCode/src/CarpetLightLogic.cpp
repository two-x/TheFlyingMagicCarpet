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

// The Flying Magic Carpet (TM)
MagicCarpet * carpet;

LightShow * currLightShow;

static const uint8_t numModes = 3;
static uint8_t currMode = 0;
static uint8_t prevMode = currMode;
static uint8_t currVariation[ numModes ] = { 0, 0, 0 };
static bool lightsOn = true; // always boots on; toggled by an extra-long press

// config mode cycle: long-press (from ModeShow) always lands on ModeConfigGlobal;
// each double-press advances to the next one, wrapping back around.
enum AppMode { ModeShow, ModeConfigGlobal, ModeConfigHeadlight, ModeConfigChina, ModeConfigAudio };
static AppMode appMode = ModeShow;
static float audioThresholdPercent = 0.0f; // not yet applied to anything; see loop() below
// static Timer audioThresholdPrintTimer( 1000 ); // uncomment for periodic live-value debug output

// TEMPORARY: raw spectrum-shield hardware diagnostic. Remove (or comment out,
// per usual) once the audio-input hardware issue is sorted -- see AudioBoard.h.
static Timer rawAudioDiagTimer( 300 );

// soft pot takeover: entering a config screen shouldn't make the preview jump
// to wherever the pot physically happens to be sitting. instead it holds at
// the already-committed value until the pot moves noticeably (2% of its full
// range) from where it was on entry -- then it hands over to live tracking.
static const uint16_t POT_TAKEOVER_THRESHOLD = (uint16_t)( 0.02f * MAX_VOLTAGE + 0.5f );
static uint16_t configEntryPotRaw = 0;
static bool configPotTakenOver = false;

void enterConfigMode( AppMode mode ) {
   appMode = mode;
   configEntryPotRaw = carpet->pot->read();
   configPotTakenOver = false;
}

AppMode nextConfigMode( AppMode mode ) {
   switch ( mode ) {
      case ModeConfigGlobal:    return ModeConfigHeadlight;
      case ModeConfigHeadlight: return ModeConfigChina;
      case ModeConfigChina:     return ModeConfigAudio;
      case ModeConfigAudio:     return ModeConfigGlobal;
      default:                 return ModeConfigGlobal;
   }
}

// maps the live pot reading linearly to a percent in [minPercent,maxPercent].
// fullRange=false restricts to the pot's top half (used by global brightness).
float potToPercent( float minPercent, float maxPercent, bool fullRange ) {
   uint16_t potRaw = carpet->pot->read();
   float frac;
   if ( fullRange ) {
      frac = potRaw / (float)MAX_VOLTAGE;
   } else {
      uint16_t mid = MAX_VOLTAGE / 2;
      frac = ( potRaw <= mid ) ? 0.0f : ( (float)( potRaw - mid ) / (float)( MAX_VOLTAGE - mid ) );
   }
   return minPercent + frac * ( maxPercent - minPercent );
}

// the live value a config screen should currently show: the committed value,
// until the pot has moved POT_TAKEOVER_THRESHOLD from its reading when this
// screen was entered -- then it switches to tracking the pot live.
float livePercentFor( float minPercent, float maxPercent, bool fullRange, float committedPercent ) {
   if ( !configPotTakenOver ) {
      uint16_t potRaw = carpet->pot->read();
      uint16_t diff = ( potRaw > configEntryPotRaw ) ? ( potRaw - configEntryPotRaw ) : ( configEntryPotRaw - potRaw );
      if ( diff >= POT_TAKEOVER_THRESHOLD ) configPotTakenOver = true;
   }
   if ( !configPotTakenOver ) return committedPercent;
   return potToPercent( minPercent, maxPercent, fullRange );
}

void printPercentSetting( const char * name, float percent ) {
   Serial.print( name );
   Serial.print( ": " );
   Serial.print( (int)( percent + 0.5f ) );
   Serial.println( "%" );
}

LightShow * makeShow( uint8_t mode, uint8_t variation ) {
   switch ( mode ) {
      case 0:
         return new NightriderShow( carpet, variation );
      case 1:
         return new FlameShow( carpet, variation );
      case 2:
         return new EqualizerShow( carpet );
      default:
         // we fucked up, just reset
         // carpet->error(); // uncomment for debugging
         return new NightriderShow( carpet, variation );
   }
}

const char * showName( uint8_t mode ) {
   switch ( mode ) {
      case 0:  return "Nightrider";
      case 1:  return "Flame";
      case 2:  return "Equalizer";
      default: return "?";
   }
}

// vertically compact: one banner line, one line with every config setting's
// current value, instead of a line per value
void printWelcome() {
   Serial.println( "=== Flying Magic Carpet ===" );
   Serial.print( "Show=" ); Serial.print( showName( currMode ) );
   Serial.print( "(var " ); Serial.print( currVariation[ currMode ] ); Serial.print( ")" );
   Serial.print( "  Global=" ); Serial.print( (int)( carpet->getGlobalBrightness() + 0.5f ) ); Serial.print( "%" );
   Serial.print( "  Headlight=" ); Serial.print( (int)( carpet->getHeadlightBrightness() + 0.5f ) ); Serial.print( "%" );
   Serial.print( "  China=" ); Serial.print( (int)( carpet->getChinaBrightness() + 0.5f ) ); Serial.print( "%" );
   Serial.print( "  Audio=" ); Serial.print( (int)( audioThresholdPercent + 0.5f ) ); Serial.println( "%" );
}

void setup() {
   Serial.begin(9600);
   // AudioBoard::setup();

   // setup the carpet
   carpet = theMagicCarpet();
   carpet->setup(); // also loads persisted show/variation state from flash (Nvm::load())

   currMode = Nvm::loadedShow();
   if ( currMode >= numModes ) currMode = 0; // guard against stale/out-of-range flash data
   for ( uint8_t i = 0; i < numModes; ++i ) {
      currVariation[ i ] = Nvm::loadedVariation( i );
   }
   prevMode = currMode;
   carpet->setGlobalBrightness( Nvm::loadedGlobalBrightness() );
   carpet->setHeadlightBrightness( Nvm::loadedHeadlightBrightness() );
   carpet->setChinaBrightness( Nvm::loadedChinaBrightness() );
   audioThresholdPercent = Nvm::loadedAudioThreshold();

   currLightShow = makeShow( currMode, currVariation[ currMode ] );
   currLightShow->start();

   printWelcome();
}

void loop() {
   static uint32_t clock;
   clock = millis();

   AudioBoard::pollFrequencies( clock );

   carpet->encoder->update(); // refresh button short/medium/long/extra-long/double press state

   // press-hold feedback: flash the perimeter as thresholds are crossed, live
   if ( carpet->encoder->button.crossedMediumThreshold() ) {
      carpet->flashRope( 1 );
   }
   if ( carpet->encoder->button.crossedLongThreshold() ) {
      carpet->flashRope( 2 );
   }

   // consume every press type exactly once per loop, regardless of mode, so a
   // press irrelevant to the current mode doesn't go stale and misfire later
   bool didShort = carpet->encoder->button.shortpress();
   bool didMedium = carpet->encoder->button.mediumpress();
   bool didLong = carpet->encoder->button.longpress();
   bool didExtraLong = carpet->encoder->button.extralongpress();
   bool didDouble = carpet->encoder->button.doublepress();

   if ( appMode == ModeConfigGlobal || appMode == ModeConfigHeadlight || appMode == ModeConfigChina || appMode == ModeConfigAudio ) {
      // these 4 screens take over the whole visual with a live preview

      currLightShow->update( clock ); // keep the show progressing "invisibly" so it doesn't jump on cancel

      float livePercent;
      if ( appMode == ModeConfigGlobal ) {
         livePercent = livePercentFor( 0.0f, 100.0f, false, carpet->getGlobalBrightness() ); // pot's bottom half unused
         carpet->showSolidRed( livePercent );
      } else if ( appMode == ModeConfigHeadlight ) {
         livePercent = livePercentFor( 50.0f, 100.0f, true, carpet->getHeadlightBrightness() );
         carpet->showHeadlightPreview( livePercent );
      } else if ( appMode == ModeConfigChina ) {
         livePercent = livePercentFor( 0.0f, 100.0f, true, carpet->getChinaBrightness() );
         carpet->showChinaPreview( livePercent );
      } else { // ModeConfigAudio
         livePercent = livePercentFor( 0.0f, 100.0f, true, audioThresholdPercent );
         // if ( audioThresholdPrintTimer.expireset() ) {
         //    printPercentSetting( "audio threshold (live)", livePercent );
         // }
         // TEMPORARY diagnostic: all 7 raw (unprocessed, 0-1023) spectrum-shield
         // bins, to check whether the hardware is responding to input at all,
         // independent of the 3 bins the VU meter actually displays.
         if ( rawAudioDiagTimer.expireset() ) {
            Serial.print( "raw bins: " );
            for ( int i = 0; i < 7; ++i ) {
               Serial.print( AudioBoard::Frequencies_Mono[ i ] );
               Serial.print( i < 6 ? ", " : "\n" );
            }
         }
         carpet->showAudioMeter( AudioBoard::getHigh(), AudioBoard::getMid(), AudioBoard::getLow(), livePercent );
      }

      if ( didShort ) {
         if ( appMode == ModeConfigGlobal ) {
            carpet->setGlobalBrightness( livePercent );
            Nvm::saveGlobalBrightness( (uint8_t)( livePercent + 0.5f ) );
            printPercentSetting( "Global brightness", livePercent );
         } else if ( appMode == ModeConfigHeadlight ) {
            carpet->setHeadlightBrightness( livePercent );
            Nvm::saveHeadlightBrightness( (uint8_t)( livePercent + 0.5f ) );
            printPercentSetting( "Headlight brightness", livePercent );
         } else if ( appMode == ModeConfigChina ) {
            carpet->setChinaBrightness( livePercent );
            Nvm::saveChinaBrightness( (uint8_t)( livePercent + 0.5f ) );
            printPercentSetting( "China brightness", livePercent );
         } else { // ModeConfigAudio
            audioThresholdPercent = livePercent;
            Nvm::saveAudioThreshold( (uint8_t)( audioThresholdPercent + 0.5f ) );
            printPercentSetting( "Audio threshold", audioThresholdPercent );
         }
         carpet->flashRope( 1 ); // confirms the setting was committed
         appMode = ModeShow;
      } else if ( didDouble ) {
         carpet->flashRope( 2 ); // confirms advancing to the next config setting
         enterConfigMode( nextConfigMode( appMode ) );
      } else if ( didMedium || didLong ) {
         appMode = ModeShow; // cancel, no save
      }

   } else {
      // ModeShow

      if ( didExtraLong ) lightsOn = !lightsOn;
      if ( didLong ) enterConfigMode( ModeConfigGlobal );
      if ( didShort ) {
         currMode = ( currMode + 1 ) % numModes;
         Nvm::saveShow( currMode );
      }

      if ( currMode != prevMode ) {
         delete currLightShow;
         currLightShow = makeShow( currMode, currVariation[ currMode ] );
         currLightShow->start();
         prevMode = currMode;
      }

      currLightShow->update( clock );

      // persist a show's variation the moment it actually changes
      uint8_t variation = currLightShow->variation();
      if ( variation != currVariation[ currMode ] ) {
         currVariation[ currMode ] = variation;
         Nvm::saveVariation( currMode, variation );
      }

      carpet->applyBrightnessCeiling();

      if ( !lightsOn ) carpet->clear(); // master override -- still let shows run "invisibly" underneath
   }

   carpet->show();
}
