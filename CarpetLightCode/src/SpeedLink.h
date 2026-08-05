/* SpeedLink.h
 *
 *    I2C slave interface to CANTroller2 (the vehicle's main control system,
 *    a separate board acting as I2C master). Listens on Wire -- the Due's
 *    primary I2C bus, pins 20 (SDA)/21 (SCL) on the communication port
 *    header (confirmed against framework-arduino-sam's variant.h: PIN_WIRE_SDA/
 *    PIN_WIRE_SCL = 20/21; Wire1 is the separate SDA1/SCL1 pins near the USB
 *    connectors, pins 70/71, no longer used here) -- at address 0x69,
 *    matching CANTroller2's known_i2c_addr table (see its src/i2cbus.h).
 *
 *    Packet format (CANTroller2's i2cbus.h, LightingBox class), no
 *    checksum/length byte -- I2C START/STOP delimits each packet:
 *       byte1 top nibble 0x0: status flags, 1 byte total (ignored here)
 *       byte1 top nibble 0x1: runmode,      1 byte total --
 *          byte1 = 0x10 | runmode, sent only when runmode changes. Runmodes
 *          (CANTroller2's src/globals.h): Basic=0, LowPower=1, Standby=2,
 *          Stall=3, Hold=4, Fly=5, Cruise=6, Cal=7.
 *       byte1 top nibble 0x2: speed update, 2 bytes total --
 *          byte1 = 0x20 | (high nibble of a 12-bit hundredths-of-mph value)
 *          byte2 = low byte of that value
 *    Note CANTroller2 only transmits 12 bits of speed (top nibble of the
 *    16-bit value is discarded on its sending side), so the max
 *    representable speed is 0x0FFF = 40.95 mph -- a sending-side protocol
 *    ceiling, not something fixable from here.
 */

#ifndef __SPEED_LINK_H
#define __SPEED_LINK_H

#ifndef __EMSCRIPTEN__
#include <Wire.h>
#endif
#include "Utilities.h"

#define LIGHTBOX_I2C_ADDR 0x69
#define RUNMODE_LOW_POWER 1 // CANTroller2's globals.h: enum runmode { Basic=0, LowPower=1, ... }
#define RUNMODE_UNKNOWN 0xFF // sentinel: no runmode packet received yet

class SpeedLink {
 private:
   static volatile uint16_t speedHundredthsMph_;
   static volatile bool everReceived_; // sentinel: Timer has no "unset" state of its own
   static Timer lastUpdateTimer_;      // Timer's own start/tout fields are volatile, so this is ISR-safe
   static volatile uint8_t runmode_;

#ifndef __EMSCRIPTEN__
   static void onReceive( int numBytes ) {
      if ( numBytes < 1 || !Wire.available() ) return;
      uint8_t byte1 = Wire.read();
      uint8_t cmd = ( byte1 >> 4 ) & 0x0F;
      if ( cmd == 0x2 && Wire.available() ) {
         uint8_t byte2 = Wire.read();
         speedHundredthsMph_ = ( (uint16_t)( byte1 & 0x0F ) << 8 ) | byte2;
         everReceived_ = true;
         lastUpdateTimer_.reset();
      } else if ( cmd == 0x1 ) {
         runmode_ = byte1 & 0x0F;
      }
      while ( Wire.available() ) Wire.read(); // drain anything unexpected/extra
   }
#endif

 public:
   static void setup() {
#ifndef __EMSCRIPTEN__
      Wire.begin( LIGHTBOX_I2C_ADDR );
      Wire.onReceive( onReceive );
#endif
      // WASM: no real I2C bus -- the visualizer injects speed/runmode
      // directly via injectSpeedHundredthsMph()/injectRunmode() below
      // instead (per the "poke it into the FW variable" model), so there's
      // nothing else to set up here.
   }

#ifdef __EMSCRIPTEN__
   // direct-injection equivalents of what onReceive() does for a real 0x2/
   // 0x1 packet, respectively -- same effect, just JS-triggered instead of
   // I2C-triggered.
   static void injectSpeedHundredthsMph( uint16_t v ) {
      speedHundredthsMph_ = v;
      everReceived_ = true;
      lastUpdateTimer_.reset();
   }
   static void injectRunmode( uint8_t mode ) {
      runmode_ = mode;
   }
#endif

   static uint16_t getSpeedHundredthsMph() {
      return speedHundredthsMph_;
   }

   static float getSpeedMph() {
      return speedHundredthsMph_ / 100.0f;
   }

   // 1 mph = 5280ft / 3600s = 1.4667 ft/s. Exposed as a standalone converter
   // (not just off the live reading) since shows often need to convert an
   // already-fetched/staleness-zeroed mph value rather than re-reading here.
   static float mphToFtPerSec( float mph ) {
      return mph * 1.46667f;
   }

   // feet/sec is the more directly useful unit for shows that scroll or
   // position content along physical rope/fixture geometry (CarpetGeometry
   // distances are all in feet)
   static float getSpeedFtPerSec() {
      return mphToFtPerSec( getSpeedMph() );
   }

   // true if a speed packet has arrived within the last staleMs -- lets a
   // speed-reactive show fall back to "stopped" behavior if the link drops
   // instead of freezing on the last received value forever
   static bool isFresh( uint32_t staleMs = 2000 ) {
      return everReceived_ && !lastUpdateTimer_.elapsed( staleMs );
   }

   // last received runmode, or RUNMODE_UNKNOWN before the first runmode
   // packet arrives. Unlike speed, runmode packets are only sent on change
   // (see CANTroller2's sendrunmode()), so there's no freshness/staleness
   // concept here -- the last received value is definitionally still current.
   static uint8_t getRunmode() {
      return runmode_;
   }

   static bool isLowPower() {
      return runmode_ == RUNMODE_LOW_POWER;
   }
};

volatile uint16_t SpeedLink::speedHundredthsMph_ = 0;
volatile bool SpeedLink::everReceived_ = false;
Timer SpeedLink::lastUpdateTimer_;
volatile uint8_t SpeedLink::runmode_ = RUNMODE_UNKNOWN;

#endif
