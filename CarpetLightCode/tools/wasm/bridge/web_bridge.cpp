// Phase 4: the REAL CarpetLightLogic.cpp (setup()/loop(), all 5 real shows,
// CarpetGeometry, MagicCarpet) compiling and running under Emscripten.
// #includes the real .cpp directly (Arduino sketches have no main() of
// their own -- the framework calls setup()/loop()), matching exactly how
// the real Due build treats it. See tools/wasm/README.md / the migration
// plan.
#include "../../../src/CarpetLightLogic.cpp"
#include <emscripten/emscripten.h>

extern "C" {

EMSCRIPTEN_KEEPALIVE
void web_setup() { setup(); }

EMSCRIPTEN_KEEPALIVE
void web_tick( uint32_t nowMs ) {
   halSetMillis( nowMs );
   loop();
}

EMSCRIPTEN_KEEPALIVE
uint8_t * web_getRopeLedsPtr() { return (uint8_t *)carpet->ropeLeds; }
EMSCRIPTEN_KEEPALIVE
uint8_t * web_getMegabarLedsPtr() { return (uint8_t *)carpet->megabarLeds; }
EMSCRIPTEN_KEEPALIVE
uint8_t * web_getChinaLedsPtr() { return (uint8_t *)carpet->chinaLeds; }
EMSCRIPTEN_KEEPALIVE
int web_getNumRopeLeds() { return NUM_NEO_LEDS_ACTUAL; }
EMSCRIPTEN_KEEPALIVE
int web_getNumMegabarLeds() { return NUM_MEGABAR_LEDS; }
EMSCRIPTEN_KEEPALIVE
int web_getNumChinaLeds() { return NUM_CHINA_LEDS; }

// Real, post-convertNeoArray() bus-write buffers -- the exact bytes show()
// hands to FastLED/Ws281xDma each frame (WS281x wire-protocol RGBW->RGB
// repacking, see LedUtil::convertNeoArray()'s comment and MagicCarpet.h's
// ropeShowLeds declaration comment). Raw wire-order data, not directly
// renderable per-fixture -- exposed for inspection/diffing, not rendering.
// DMX needs no equivalent export: megabarLeds/chinaLeds (already exposed
// above) ARE the literal DMX bus buffer, dmx_send() sends their raw memory
// directly with no conversion step.
EMSCRIPTEN_KEEPALIVE
uint8_t * web_getRopeShowLedsPtr() { return (uint8_t *)carpet->ropeShowLeds; }
EMSCRIPTEN_KEEPALIVE
uint8_t * web_getRightShowLedsPtr() { return (uint8_t *)carpet->rightShowLeds; }
EMSCRIPTEN_KEEPALIVE
uint8_t * web_getLeftShowLedsPtr() { return (uint8_t *)carpet->leftShowLeds; }
EMSCRIPTEN_KEEPALIVE
int web_getNumRopeShowLeds() { return NUM_NEO_SHOW_LEDS; }
EMSCRIPTEN_KEEPALIVE
int web_getNumSideShowLeds() { return NUM_NEO_LEDS_PER_STRIP; }

// Real fixture GROUND-SPOT positions, straight from CarpetGeometry.h --
// the visualizer reads these instead of maintaining its own position
// tables, per the zero-duplication mandate. This is dimX/dimY -- what a
// real light show sees (china pairs are co-located here, matching real
// show behavior; see CarpetGeometry.h's file header for why).
EMSCRIPTEN_KEEPALIVE
float web_getMegabarXFt( int idx ) { return CarpetGeometry::getMegabar( (uint8_t)idx ).dimX; }
EMSCRIPTEN_KEEPALIVE
float web_getMegabarYFt( int idx ) { return CarpetGeometry::getMegabar( (uint8_t)idx ).dimY; }
EMSCRIPTEN_KEEPALIVE
float web_getChinaXFt( int idx ) { return CarpetGeometry::getChina( (uint8_t)idx ).dimX; }
EMSCRIPTEN_KEEPALIVE
float web_getChinaYFt( int idx ) { return CarpetGeometry::getChina( (uint8_t)idx ).dimY; }
// Real fixture MOUNT position (fixtureDimX/Y/Z) -- visualizer-only (no
// real light show reads this); china pairs are genuinely offset here
// (2 physical fixtures sharing one corner bracket, aimed ~90deg apart),
// unlike their co-located ground spot above. Z is feet off the ground,
// for the visualizer's 3D-model rendering (light cone height).
EMSCRIPTEN_KEEPALIVE
float web_getMegabarFixtureXFt( int idx ) { return CarpetGeometry::getMegabar( (uint8_t)idx ).fixtureDimX; }
EMSCRIPTEN_KEEPALIVE
float web_getMegabarFixtureYFt( int idx ) { return CarpetGeometry::getMegabar( (uint8_t)idx ).fixtureDimY; }
EMSCRIPTEN_KEEPALIVE
float web_getChinaFixtureXFt( int idx ) { return CarpetGeometry::getChina( (uint8_t)idx ).fixtureDimX; }
EMSCRIPTEN_KEEPALIVE
float web_getChinaFixtureYFt( int idx ) { return CarpetGeometry::getChina( (uint8_t)idx ).fixtureDimY; }
EMSCRIPTEN_KEEPALIVE
float web_getMegabarZFt( int idx ) { return CarpetGeometry::getMegabar( (uint8_t)idx ).fixtureDimZ; }
EMSCRIPTEN_KEEPALIVE
float web_getChinaZFt( int idx ) { return CarpetGeometry::getChina( (uint8_t)idx ).fixtureDimZ; }
// Real DMX addresses, straight from CarpetGeometry.h -- see README.md.
EMSCRIPTEN_KEEPALIVE
int web_getMegabarDmxAddress( int idx ) { return CarpetGeometry::getMegabar( (uint8_t)idx ).dmxAddress; }
EMSCRIPTEN_KEEPALIVE
int web_getChinaDmxAddress( int idx ) { return CarpetGeometry::getChina( (uint8_t)idx ).dmxAddress; }
// Angle-based getters -- exposed for direct verification/debugging.
EMSCRIPTEN_KEEPALIVE
int web_getMegabarByAngle( float deg ) { return CarpetGeometry::getMegabarByAngle( deg ); }
EMSCRIPTEN_KEEPALIVE
int web_getChinaByAngle( float deg ) { return CarpetGeometry::getChinaByAngle( deg ); }
// Neo CarSide-based getters -- exposed for direct verification/debugging.
EMSCRIPTEN_KEEPALIVE
int web_getNeoByYPixel( int side, float yPixel ) { return CarpetGeometry::getNeoByYPixel( (CarpetGeometry::CarSide)side, yPixel ); }
EMSCRIPTEN_KEEPALIVE
int web_getNeoByXPixel( int side, float xPixel ) { return CarpetGeometry::getNeoByXPixel( (CarpetGeometry::CarSide)side, xPixel ); }
EMSCRIPTEN_KEEPALIVE
float web_getMaxYPixel( int side ) { return CarpetGeometry::getMaxYPixel( (CarpetGeometry::CarSide)side ); }
EMSCRIPTEN_KEEPALIVE
float web_getMaxXPixel( int side ) { return CarpetGeometry::getMaxXPixel( (CarpetGeometry::CarSide)side ); }
// Real vehicle dimensions -- the visualizer's canvas layout reads these
// instead of keeping its own hardcoded CAR_W_FT/CAR_H_FT/FRINGE_LENGTH_Z_FT.
EMSCRIPTEN_KEEPALIVE
float web_getCarWidthFt() { return CarpetGeometry::CAR_WIDTH_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getCarLengthFt() { return CarpetGeometry::CAR_LENGTH_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getFringeLengthZFt() { return CarpetGeometry::FRINGE_LENGTH_Z_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getSurfaceBaseHeightFt() { return CarpetGeometry::SURFACE_BASE_HEIGHT_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getSurfaceThicknessFt() { return CarpetGeometry::SURFACE_THICKNESS_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getCarpetThicknessFt() { return CarpetGeometry::CARPET_THICKNESS_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getSideUndulationAmplitudeFt() { return CarpetGeometry::SIDE_UNDULATION_AMPLITUDE_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getSideUndulationWavelengthFt() { return CarpetGeometry::SIDE_UNDULATION_WAVELENGTH_FT; }
// Round decoration table -- visualizer-only geometry, moved into
// CarpetGeometry.h so it has one real source of truth instead of its own
// hardcoded pixel-space literals.
EMSCRIPTEN_KEEPALIVE
float web_getTableCenterXFt() { return CarpetGeometry::TABLE_CENTER_X_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTableCenterYFt() { return CarpetGeometry::TABLE_CENTER_Y_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTableDiameterFt() { return CarpetGeometry::TABLE_DIAMETER_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTableHeightAboveSurfaceFt() { return CarpetGeometry::TABLE_HEIGHT_ABOVE_SURFACE_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTableclothTasselHeightFt() { return CarpetGeometry::TABLECLOTH_TASSEL_HEIGHT_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTableclothFringeDropFt() { return CarpetGeometry::TABLECLOTH_FRINGE_DROP_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTableclothFringeThicknessFt() { return CarpetGeometry::TABLECLOTH_FRINGE_THICKNESS_FT; }
// Corner tassels + fringe strand spec -- real dimensions, visualizer
// decides how densely/simplified to actually render them (see its own
// comments), but the real facts live here, one source of truth.
EMSCRIPTEN_KEEPALIVE
float web_getTasselSphereDiameterFt() { return CarpetGeometry::TASSEL_SPHERE_DIAMETER_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTasselSphereBelowSurfaceFt() { return CarpetGeometry::TASSEL_SPHERE_BELOW_SURFACE_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTasselCylinderDiameterFt() { return CarpetGeometry::TASSEL_CYLINDER_DIAMETER_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTasselCylinderHeightFt() { return CarpetGeometry::TASSEL_CYLINDER_HEIGHT_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTasselConeTopDiameterFt() { return CarpetGeometry::TASSEL_CONE_TOP_DIAMETER_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTasselConeBottomDiameterFt() { return CarpetGeometry::TASSEL_CONE_BOTTOM_DIAMETER_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getTasselConeBottomAboveGroundFt() { return CarpetGeometry::TASSEL_CONE_BOTTOM_ABOVE_GROUND_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getFringeStrandsPerIn() { return CarpetGeometry::FRINGE_STRANDS_PER_IN; }
EMSCRIPTEN_KEEPALIVE
float web_getFringeStrandDiameterFt() { return CarpetGeometry::FRINGE_STRAND_DIAMETER_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getMegabarFrontPairSpacingFt() { return CarpetGeometry::MEGABAR_FRONT_PAIR_SPACING_FT; }
// Vehicle undercarriage (chassis) + wheels -- not consumed by anything yet
// (no 3D mode exists), exported now so a future 3D pass reads real FW
// dimensions instead of re-guessing/hardcoding them independently.
EMSCRIPTEN_KEEPALIVE
float web_getChassisWidthFt() { return CarpetGeometry::CHASSIS_WIDTH_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getChassisLengthFt() { return CarpetGeometry::CHASSIS_LENGTH_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getChassisCenterXFt() { return CarpetGeometry::CHASSIS_CENTER_X_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getChassisCenterYFt() { return CarpetGeometry::CHASSIS_CENTER_Y_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getChassisBottomHeightFt() { return CarpetGeometry::CHASSIS_BOTTOM_HEIGHT_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getChassisTopHeightFt() { return CarpetGeometry::CHASSIS_TOP_HEIGHT_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getWheelDiameterFt() { return CarpetGeometry::WHEEL_DIAMETER_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getWheelWidthFt() { return CarpetGeometry::WHEEL_WIDTH_FT; }
EMSCRIPTEN_KEEPALIVE
float web_getFenderClearanceFt() { return CarpetGeometry::FENDER_CLEARANCE_FT; }
EMSCRIPTEN_KEEPALIVE
int web_sizeofRopeLed() { return sizeof( carpet->ropeLeds[ 0 ] ); }
EMSCRIPTEN_KEEPALIVE
int web_sizeofMegabarLed() { return sizeof( carpet->megabarLeds[ 0 ] ); }
EMSCRIPTEN_KEEPALIVE
int web_sizeofChinaLed() { return sizeof( carpet->chinaLeds[ 0 ] ); }

EMSCRIPTEN_KEEPALIVE
const char * web_getCurrentShowName() { return showName( currMode ); }
EMSCRIPTEN_KEEPALIVE
const char * web_getCurrentVariationName() { return currLightShow->variationName(); }
// Real per-show variation count (LightShow::numVariations(), overridden by
// each show) -- lets the visualizer enumerate the REAL variation list for
// every show at launch instead of hardcoding one, per the zero-duplication
// mandate. Reflects whichever show is currently selected; the visualizer's
// enumeration pass calls web_setCurrentShow() first for each show in turn.
EMSCRIPTEN_KEEPALIVE
int web_getNumVariations() { return currLightShow->numVariations(); }

// test-only convenience -- real UI drives this via the encoder/button
// injectors below, this just makes show-cycling easy to poke at directly
// for a bare harness
EMSCRIPTEN_KEEPALIVE
void web_forcePressShort() {
   using namespace LedControl;
   digitalWrite( PushButtonEdge::pin_, LOW ); PushButtonEdge::isr();
   halSetMillis( hal_millis_ + 50 );
   digitalWrite( PushButtonEdge::pin_, HIGH ); PushButtonEdge::isr();
}

// ---------------------------------------------------------------------
// Real input injection -- the visualizer's zero-duplication mandate (see
// claude_dev_prompts.md) requires the Dev Tool role to drive the REAL
// PushButton/Encoder/Potentiometer state machines, not classify presses
// or navigate config-mode in JS. These mirror what web_forcePressShort()
// already did for its own narrow case, generalized so JS controls timing.

// Button down/up -- JS calls these on real mousedown/mouseup (or touch
// start/end), letting the real PushButton::update() state machine (run
// every web_tick()) classify the press tier from genuine elapsed time
// between the two calls, exactly as the real interrupt-driven hardware
// path would. See PushButtonEdge::isr()'s own "active low" comment --
// down pulls the pin LOW.
EMSCRIPTEN_KEEPALIVE
void web_injectButtonDown() {
   using namespace LedControl;
   digitalWrite( PushButtonEdge::pin_, LOW );
   PushButtonEdge::isr();
}
EMSCRIPTEN_KEEPALIVE
void web_injectButtonUp() {
   using namespace LedControl;
   digitalWrite( PushButtonEdge::pin_, HIGH );
   PushButtonEdge::isr();
}

// Encoder rotation -- EncoderImpl::updatePosition() (LedController.h)
// only ever looks at the CURRENT a_/b_ pin values (a_^b_ decides
// increment vs decrement), not a true quadrature edge sequence, so one
// call per unit step with a_/b_ set to encode the desired direction is
// exactly equivalent to what the real A/B interrupts would have produced
// -- no need to simulate a multi-edge quadrature waveform.
EMSCRIPTEN_KEEPALIVE
void web_injectEncoderDelta( int steps ) {
   using namespace LedControl;
   int dir = ( steps > 0 ) ? 1 : -1;
   int n = steps > 0 ? steps : -steps;
   for ( int i = 0; i < n; ++i ) {
      EncoderImpl::a_ = ( dir > 0 ) ? 1 : 0;
      EncoderImpl::b_ = 0;
      EncoderImpl::updatePosition();
   }
}

// Pot -- writes the real Potentiometer's own analog pin via the HalShim's
// analog-injection path (halSetAnalogPinState(), which applies the same
// below-A0 auto-offset real analogRead() does -- see HalShim.h's BUGFIX
// comment; POT_ANALOG_PIN and ENCODER_SW_PIN share the literal value 3 in
// source, real hardware disambiguates via that offset, and the shim now
// does too).
EMSCRIPTEN_KEEPALIVE
void web_injectPotPercent( float pct ) {
   if ( pct < 0.0f ) pct = 0.0f;
   if ( pct > 100.0f ) pct = 100.0f;
   int raw = (int)( pct / 100.0f * (float)MAX_VOLTAGE + 0.5f );
   halSetAnalogPinState( POT_ANALOG_PIN, raw );
}
// Vehicle speed -- SpeedLink's own direct-injection equivalents of a real
// I2C speed packet (see SpeedLink.h's injectSpeedHundredthsMph()/
// injectRunmode(), already written for exactly this, just never wired to
// the bridge/UI before now). Real FW consumers (SpeedStripesShow.h) read
// this back through SpeedLink::isFresh()/getSpeedMph() -- no duplication
// of speed-reactive logic here, only the raw injected value.
EMSCRIPTEN_KEEPALIVE
void web_injectSpeedMph( float mph ) {
   if ( mph < 0.0f ) mph = 0.0f;
   SpeedLink::injectSpeedHundredthsMph( (uint16_t)( mph * 100.0f + 0.5f ) );
}
EMSCRIPTEN_KEEPALIVE
void web_injectRunmode( int mode ) { SpeedLink::injectRunmode( (uint8_t)mode ); }

// ---------------------------------------------------------------------
// App-mode / config-navigation status -- lets JS show real status text
// (which screen/subsetting is active) without tracking that state
// independently. settingName()/appMode/configSubsetting are CarpetLightLogic.cpp's
// own real state, visible here since this file #includes it directly.
EMSCRIPTEN_KEEPALIVE
int web_getAppMode() { return (int)appMode; }
EMSCRIPTEN_KEEPALIVE
int web_getConfigSubsetting() { return (int)configSubsetting; }
EMSCRIPTEN_KEEPALIVE
const char * web_getSettingName() { return settingName( appMode, configSubsetting ); }

// ---------------------------------------------------------------------
// Direct show/variation selection -- a "convenience control" (see
// claude_dev_prompts.md prompt 6): real hardware can only CYCLE shows via
// repeated short-press, so jumping straight to an arbitrary index has no
// physical-hardware equivalent action, same category as the audio sliders
// already skipping real hardware's multi-press menu navigation. The value
// landed on is 100% real committed state either way -- makeShow()/start()
// are the exact same calls the real short-press handler makes.
// Visualizer-only: forces the LIVE show/variation state back to the real
// compile-time defaults (Nvm::resetShowVariationToDefaults() -- the exact
// same values/branch real hardware's "first boot ever" flash-reset uses,
// no separate JS-side copy of the default show/variation index), WITHOUT
// touching persisted storage -- unlike web_setCurrentShow()/
// web_setCurrentVariation(), this deliberately does not call Nvm::save*(),
// so it can't clobber a real saved selection, it only affects what this
// page load starts out rendering. Mirrors setup()'s own hydrate-from-Nvm
// sequence (lines above in this same translation unit) so the resulting
// currMode/currVariation/currLightShow are indistinguishable from a real
// fresh boot. Called once by the visualizer right after web_setup(),
// before enumerateShowsFromWasm() captures the show/variation it'll
// restore back to.
EMSCRIPTEN_KEEPALIVE
void web_resetToDefaultShow() {
   Nvm::resetShowVariationToDefaults();
   currMode = (ShowMode)Nvm::loadedShow();
   for ( uint8_t i = 0; i < numModes; ++i ) currVariation[ i ] = Nvm::loadedVariation( i );
   prevMode = currMode;
   delete currLightShow;
   currLightShow = makeShow( currMode, currVariation[ currMode ] );
   currLightShow->start();
}

EMSCRIPTEN_KEEPALIVE
int web_getNumShows() { return (int)numModes; }
EMSCRIPTEN_KEEPALIVE
int web_getCurrentShowIndex() { return (int)currMode; }
EMSCRIPTEN_KEEPALIVE
int web_getCurrentVariationIndex() { return (int)currLightShow->variation(); }
EMSCRIPTEN_KEEPALIVE
void web_setCurrentShow( int mode ) {
   if ( mode < 0 || mode >= (int)numModes || (ShowMode)mode == currMode ) return;
   currMode = (ShowMode)mode;
   Nvm::saveShow( currMode );
   delete currLightShow;
   currLightShow = makeShow( currMode, currVariation[ currMode ] );
   currLightShow->start();
   prevMode = currMode;
}
EMSCRIPTEN_KEEPALIVE
void web_setCurrentVariation( int variation ) {
   if ( variation < 0 || variation == (int)currVariation[ currMode ] ) return;
   currVariation[ currMode ] = (uint8_t)variation;
   Nvm::saveVariation( currMode, (uint8_t)variation );
   delete currLightShow;
   currLightShow = makeShow( currMode, (uint8_t)variation );
   currLightShow->start();
}

// ---------------------------------------------------------------------
// Brightness + blacklight -- the visualizer's "Quick Adjust" panel sliders
// call these directly (a real, always-live shortcut to the same state the
// Brightness config screens adjust -- see claude_dev_prompts.md's
// help-fw-io/help-shortcuts distinction), rather than scaling
// ropeLeds/megabarLeds/chinaLeds a second time in JS after reading them
// back already-scaled by the real MagicCarpet::applyBrightnessCeiling().
EMSCRIPTEN_KEEPALIVE
void web_setGlobalBrightness( float pct ) { carpet->setGlobalBrightness( pct ); }
EMSCRIPTEN_KEEPALIVE
float web_getGlobalBrightness() { return carpet->getGlobalBrightness(); }
EMSCRIPTEN_KEEPALIVE
void web_setHeadlightBrightness( float pct ) { carpet->setHeadlightBrightness( pct ); }
EMSCRIPTEN_KEEPALIVE
float web_getHeadlightBrightness() { return carpet->getHeadlightBrightness(); }
EMSCRIPTEN_KEEPALIVE
void web_setChinaBrightness( float pct ) { carpet->setChinaBrightness( pct ); }
EMSCRIPTEN_KEEPALIVE
float web_getChinaBrightness() { return carpet->getChinaBrightness(); }
// Mirrors exactly what the real double-press handler in CarpetLightLogic.cpp
// does (toggle blacklightOn, call carpet->setBlacklight()) -- both touched
// together so the Quick Adjust checkbox and a real double-press can never
// silently disagree about the current state.
EMSCRIPTEN_KEEPALIVE
void web_setBlacklightOn( int on ) {
   blacklightOn = ( on != 0 );
   carpet->setBlacklight( blacklightOn );
}
EMSCRIPTEN_KEEPALIVE
int web_getBlacklightOn() { return blacklightOn ? 1 : 0; }

// ---------------------------------------------------------------------
// Audio injection + readback. Per the visualizer's architecture rule (see
// tools/wasm/README.md / the migration plan): the visualizer may only feed
// a simulated raw signal in (here, one MSGEQ7 poll's worth of 7 bin
// values) and read the REAL AudioBoard's own computed output back out --
// no hit-detection, level-smoothing, or AGC math may be reimplemented in
// JS. web_injectAdcBins() writes into the same static scratch buffer
// AudioBoard::pollFrequencies() (called every web_tick(), from the real
// loop()) reads via Read_Frequencies()'s analogRead(DC_One) x7 cycle --
// see HalShim.h's halSetAnalogCycle().
static int adcBinsScratch_[ 7 ];
EMSCRIPTEN_KEEPALIVE
int * web_getAdcBinsBufferPtr() { return adcBinsScratch_; }
EMSCRIPTEN_KEEPALIVE
void web_injectAdcBins() { halSetAnalogCycle( DC_One, adcBinsScratch_, 7 ); }

EMSCRIPTEN_KEEPALIVE
int web_audioGetHitPercent( int band ) { return AudioBoard::getBandHitPercent( (AudioBand)band ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetNormalPercent( int band ) { return AudioBoard::getBandNormalPercent( (AudioBand)band ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetRawPercent( int band ) { return AudioBoard::getBandRawPercent( (AudioBand)band ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetRmsPercent( int band ) { return AudioBoard::getBandRmsPercent( (AudioBand)band ); }
EMSCRIPTEN_KEEPALIVE
int web_audioIsSilent() { return AudioBoard::isSilent() ? 1 : 0; }

EMSCRIPTEN_KEEPALIVE
void web_audioSetAgcMode( int mode ) { AudioBoard::setAgcMode( (uint8_t)mode ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetAgcMode() { return AudioBoard::getAgcMode(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetNoiseFloorPercent( float pct ) { AudioBoard::setNoiseFloorPercent( pct ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetNoiseFloorPercent() { return AudioBoard::getNoiseFloorPercent(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetPeakThresholdPercent( float pct ) { AudioBoard::setPeakThresholdPercent( pct ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetPeakThresholdPercent() { return AudioBoard::getPeakThresholdPercent(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetAutoPeakMode( int mode ) { AudioBoard::setAutoPeakMode( (uint8_t)mode ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetAutoPeakMode() { return AudioBoard::getAutoPeakMode(); }
// slider dot: always BandFull, per request (the default arg)
EMSCRIPTEN_KEEPALIVE
float web_audioGetAutoScaledPeakThresholdPercent() { return AudioBoard::getBandAutoScaledPeakThresholdPercent(); }
// VU marks: one call per row, that row's own band
EMSCRIPTEN_KEEPALIVE
float web_audioGetBandAutoScaledPeakThresholdPercent( int band ) { return AudioBoard::getBandAutoScaledPeakThresholdPercent( (AudioBand)band ); }
// debug aid: the theoretical max percent value (always exactly 100, but
// sourced from the FW's own percent domain rather than hardcoded in JS) --
// per request, a calibration marker that should always render at the VU
// bar's own far-right edge; if it doesn't, the bug is in the bar's own
// rendering/scaling, not the FW's audio math.
EMSCRIPTEN_KEEPALIVE
int web_audioGetMaxPercent() { return 100; }

EMSCRIPTEN_KEEPALIVE
void web_audioSetHitDecayMs( float ms ) { AudioBoard::setHitDecayMs( ms ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetHitDecayMs() { return AudioBoard::getHitDecayMs(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetAudioForesightMs( float ms ) { AudioBoard::setAudioForesightMs( ms ); }
EMSCRIPTEN_KEEPALIVE
float web_audioGetAudioForesightMs() { return AudioBoard::getAudioForesightMs(); }

// no separate hit-prediction-distance setter/getter anymore -- prediction
// always spans the whole live audioForesightMs range now, see AudioBoard.h

EMSCRIPTEN_KEEPALIVE
void web_audioSetHitPredictionStyle( int style ) { AudioBoard::setHitPredictionStyle( (uint8_t)style ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetHitPredictionStyle() { return AudioBoard::getHitPredictionStyle(); }

EMSCRIPTEN_KEEPALIVE
void web_audioSetSoundReactivityEnabled( int enabled ) { AudioBoard::setSoundReactivityEnabled( enabled != 0 ); }
EMSCRIPTEN_KEEPALIVE
int web_audioGetSoundReactivityEnabled() { return AudioBoard::getSoundReactivityEnabled() ? 1 : 0; }

// ---------------------------------------------------------------------
// MSGEQ7 chip simulation support. The visualizer models the physical
// MSGEQ7 chip + surrounding analog circuitry itself (legitimate -- it's
// external hardware, not FW logic), but it must be driven by the FW's own
// REAL STROBE/RESET pin writes and the FW's own real ADC-range constant,
// never an independent guess at either. See HalShim.h's pin-write log.
EMSCRIPTEN_KEEPALIVE
int web_getAdcResolutionBits() { return ADC_RESOLUTION_BITS; }
EMSCRIPTEN_KEEPALIVE
int web_getAdcMaxValue() { return ADC_MAX_VALUE; }
EMSCRIPTEN_KEEPALIVE
int web_getStrobePin() { return STROBE; }
EMSCRIPTEN_KEEPALIVE
int web_getResetPin() { return RESET; }

EMSCRIPTEN_KEEPALIVE
int * web_getPinLogPinsPtr() { return hal_pinLogPin_; }
EMSCRIPTEN_KEEPALIVE
int * web_getPinLogValsPtr() { return hal_pinLogVal_; }
EMSCRIPTEN_KEEPALIVE
int web_getPinLogCount() { return hal_pinLogCount_; }
EMSCRIPTEN_KEEPALIVE
void web_clearPinLog() { halClearPinLog(); }

}
