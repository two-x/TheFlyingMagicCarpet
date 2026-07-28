/* SpeedLink.h
 *
 *    I2C slave interface to CANTroller2 (the vehicle's main control system,
 *    a separate board acting as I2C master). Listens on Wire1 -- the Due's
 *    second I2C bus, physically the SDA1/SCL1 pins nearest the USB
 *    connectors -- at address 0x69, matching CANTroller2's known_i2c_addr
 *    table (see its src/i2cbus.h).
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

#include <Wire.h>

#define LIGHTBOX_I2C_ADDR 0x69
#define RUNMODE_LOW_POWER 1 // CANTroller2's globals.h: enum runmode { Basic=0, LowPower=1, ... }
#define RUNMODE_UNKNOWN 0xFF // sentinel: no runmode packet received yet

class SpeedLink {
 private:
   static volatile uint16_t speedHundredthsMph_;
   static volatile uint32_t lastUpdateMillis_;
   static volatile uint8_t runmode_;

   static void onReceive( int numBytes ) {
      if ( numBytes < 1 || !Wire1.available() ) return;
      uint8_t byte1 = Wire1.read();
      uint8_t cmd = ( byte1 >> 4 ) & 0x0F;
      if ( cmd == 0x2 && Wire1.available() ) {
         uint8_t byte2 = Wire1.read();
         speedHundredthsMph_ = ( (uint16_t)( byte1 & 0x0F ) << 8 ) | byte2;
         lastUpdateMillis_ = millis();
      } else if ( cmd == 0x1 ) {
         runmode_ = byte1 & 0x0F;
      }
      while ( Wire1.available() ) Wire1.read(); // drain anything unexpected/extra
   }

 public:
   static void setup() {
      Wire1.begin( LIGHTBOX_I2C_ADDR );
      Wire1.onReceive( onReceive );
   }

   static uint16_t getSpeedHundredthsMph() {
      return speedHundredthsMph_;
   }

   static float getSpeedMph() {
      return speedHundredthsMph_ / 100.0f;
   }

   // true if a speed packet has arrived within the last staleMs -- lets a
   // speed-reactive show fall back to "stopped" behavior if the link drops
   // instead of freezing on the last received value forever
   static bool isFresh( uint32_t staleMs = 2000 ) {
      return lastUpdateMillis_ != 0 && ( millis() - lastUpdateMillis_ ) < staleMs;
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
volatile uint32_t SpeedLink::lastUpdateMillis_ = 0;
volatile uint8_t SpeedLink::runmode_ = RUNMODE_UNKNOWN;

#endif
