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
static const uint8_t MAGIC = 0x37; // bump this if State's layout ever changes
static const uint8_t MAX_SHOWS = 8; // headroom for future shows, no relayout needed

struct State {
   uint8_t magic;
   uint8_t currShow;
   uint8_t variation[ MAX_SHOWS ];
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
      flash.write( 0, (byte *)&state, sizeof( State ) );
   }
}

inline uint8_t loadedShow() {
   return state.currShow;
}

inline uint8_t loadedVariation( uint8_t show ) {
   return show < MAX_SHOWS ? state.variation[ show ] : 0;
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

} // end namespace Nvm

#endif
