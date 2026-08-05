// Phase 3: Nvm.h (in-memory + localStorage) and SpeedLink.h (direct
// injection, no Wire) HAL shims. See tools/wasm/README.md / the migration
// plan. AudioBoard.h included first, matching MagicCarpet.h's real include
// order -- Nvm.h's AGCoff default reset references AudioBoard's AGCMode enum.
#include "../../../src/AudioBoard.h"
#include "../../../src/Nvm.h"
#include "../../../src/SpeedLink.h"
#include <emscripten/emscripten.h>

extern "C" {

EMSCRIPTEN_KEEPALIVE
void phase3_nvmLoad() { Nvm::load(); }

EMSCRIPTEN_KEEPALIVE
uint8_t phase3_loadedGlobalBrightness() { return Nvm::loadedGlobalBrightness(); }

EMSCRIPTEN_KEEPALIVE
void phase3_saveGlobalBrightness( uint8_t pct ) { Nvm::saveGlobalBrightness( pct ); }

EMSCRIPTEN_KEEPALIVE
uint8_t phase3_loadedAgcMode() { return Nvm::loadedAgcMode(); }

EMSCRIPTEN_KEEPALIVE
void phase3_saveAgcMode( uint8_t mode ) { Nvm::saveAgcMode( mode ); }

EMSCRIPTEN_KEEPALIVE
void phase3_speedSetup() { SpeedLink::setup(); }

EMSCRIPTEN_KEEPALIVE
void phase3_injectSpeed( uint16_t hundredthsMph ) { SpeedLink::injectSpeedHundredthsMph( hundredthsMph ); }

EMSCRIPTEN_KEEPALIVE
void phase3_injectRunmode( uint8_t mode ) { SpeedLink::injectRunmode( mode ); }

EMSCRIPTEN_KEEPALIVE
float phase3_getSpeedMph() { return SpeedLink::getSpeedMph(); }

EMSCRIPTEN_KEEPALIVE
int phase3_isLowPower() { return SpeedLink::isLowPower(); }

EMSCRIPTEN_KEEPALIVE
int phase3_isFresh( uint32_t staleMs ) { return SpeedLink::isFresh( staleMs ); }

EMSCRIPTEN_KEEPALIVE
void phase3_setMillis( uint32_t ms ) { halSetMillis( ms ); }

}
