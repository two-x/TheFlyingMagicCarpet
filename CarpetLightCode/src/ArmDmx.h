/* ArmDmx.h
 *
 * A library for writing dmx output from a plain bit-banged GPIO pin on the
 * Arduino Due.
 *
 * REWORK v2 (still trying to make bit-banged DMX viable, to eventually free
 * a USART for the neo strands): the first bit-bang attempt used a plain
 * per-bit loop (`if (bit) hi() else lo(); delayMicroseconds(4);`), with
 * cli()/sei() scoped various ways. All variants LOOKED correct on a scope
 * at normal zoom (clean voltage, right-looking bit widths) yet fixtures
 * still misbehaved, while real USART0 hardware -- proven to work flawlessly
 * on this exact chain -- did not. Root cause: the `if/else` branch on each
 * bit's value doesn't cost the same number of CPU cycles both ways on this
 * core's pipeline, and which flash line gets fetched next depends on which
 * path is taken -- so the per-bit overhead is DATA-DEPENDENT, varying with
 * the actual DMX values being sent, rather than a small constant offset a
 * receiver could tolerate.
 *
 * This version removes that branch entirely. The whole frame (break, MAB,
 * every data bit) is precomputed up front into a plain 0/1 array, and the
 * transmit loop just walks it with ONE unconditional register write per
 * slot -- identical instructions every iteration, regardless of the data.
 * Uses 3.6us/slot (not DMX512's nominal 4.0us) -- empirically confirmed
 * clean and working on this exact hardware chain via the real USART0 test
 * earlier this session (3.6us bit width measured, fixtures flawless), and
 * faster means less blocking time per frame.
 *
 * DMX_DELAY_ITERS is a starting estimate (see its own comment) -- verify/
 * tune against a scope (both individual slot width and total frame
 * duration), the same iterative process used for FastLED's own WS281x
 * timing constants elsewhere in this codebase.
 *
 * Author: Pascal wrote the core of the dmx logic, this is a port of that code
 * Date: August 2017
 */

#ifndef __ARM_DMX_H
#define __ARM_DMX_H

#include "Utilities.h"

#define DMX_PIN            (18)     // plain GPIO, no USART peripheral muxed onto it
#define DMX_BIT_US         (3.6f)   // validated working rate, see this file's header comment
#define DMX_FRAMING_BITS   (11)     // 1 start + 8 data (LSB first) + 2 stop (8N2)
#define DMX_BREAK_SLOTS    (28)     // 28 * 3.6us = 100.8us, spec min is 92us
#define DMX_MAB_SLOTS      (7)      // 7 * 3.6us = 25.2us, spec min is 12us

// direct, fixed bit-band-alias register access -- see dmxReg's own comment.
typedef FastPinBB<DMX_PIN> DmxPin;

/* DMX state */
bool dmx_inited = false;         // To prevent double-init
uint8_t *_dmx_buf;                // Raw byte buffer: start code + real channel data
size_t dmx_buflen;                // Real channel data length (excludes start code)
uint8_t *_dmx_wave;                // Precomputed per-slot 0/1 waveform, whole frame
size_t dmx_waveLen;                // Total slot count in _dmx_wave
volatile uint32_t * dmxReg;        // Fixed bit-band alias address for DMX_PIN -- computed
                                    // once in dmx_init(); DmxPin::hi()/lo() write 1/0 to
                                    // this SAME address (bit-band aliasing), so the
                                    // transmit loop can write it directly with no branch.

// Inline, call-free cycle-counted busy-wait -- avoids delayMicroseconds()'s
// per-call overhead (a real function call/return on top of its own
// counting loop). Matches the Arduino SAM core's own delayMicroseconds()
// pattern (subs/bne, ~3 cycles/iteration) but inlined directly here so
// there's no CALL/RET cost stacked on top of it in the hot loop.
static inline void dmxDelayCycles( uint32_t n ) __attribute__((always_inline));
static inline void dmxDelayCycles( uint32_t n ) {
   __asm__ __volatile__(
      "1: \n"
      " subs %0, #1 \n"
      " bne 1b \n"
      : "+r" (n) :
   );
}

// Empirically calibrated against a real scope trace on pin 18, same as
// every other timing constant tuned this session -- NOT derived from a
// theoretical cycles-per-iteration count (a first attempt at that,
// subtracting an assumed loop-overhead in the wrong units, measured
// 2.91us/slot average against a 3.6us target -- ~19% too fast). Scaled
// directly off that real measurement: 89 * (3.6/2.91) =~ 110. If a
// re-scope still shows drift, keep scaling this proportionally off the
// new measurement rather than re-deriving cycle counts from theory.
#define DMX_DELAY_ITERS        ( 110 )

/* Init and start the DMX system */
void dmx_init(size_t buflen) {
   // Don't double-init
   if(dmx_inited == true) {
     return;
   }

   dmx_buflen = buflen;
   _dmx_buf = (uint8_t *)malloc(buflen+1);
   memset(_dmx_buf, 0, buflen+1);

   dmx_waveLen = DMX_BREAK_SLOTS + DMX_MAB_SLOTS + (buflen + 1) * DMX_FRAMING_BITS;
   _dmx_wave = (uint8_t *)malloc(dmx_waveLen);

   pinMode( DMX_PIN, OUTPUT );
   dmxReg = DmxPin::port(); // fixed bit-band alias address, computed once
   DmxPin::hi(); // idle mark state
   dmx_inited = true;
}

// Precompute the ENTIRE frame's waveform (break, MAB, every data bit) into
// _dmx_wave as plain 0/1 values -- NOT time-critical, runs with interrupts
// enabled. The transmit loop in dmx_send() then just walks this array with
// zero data-dependent branching -- see this file's own header comment.
static void dmx_buildWave() {
   size_t idx = 0;
   for ( size_t i = 0; i < DMX_BREAK_SLOTS; ++i ) _dmx_wave[ idx++ ] = 0;
   for ( size_t i = 0; i < DMX_MAB_SLOTS; ++i )   _dmx_wave[ idx++ ] = 1;
   for ( size_t b = 0; b < dmx_buflen + 1; ++b ) {
      uint8_t byteVal = _dmx_buf[ b ];
      _dmx_wave[ idx++ ] = 0; // start bit
      for ( uint8_t bit = 0; bit < 8; ++bit ) _dmx_wave[ idx++ ] = ( byteVal >> bit ) & 1; // LSB first
      _dmx_wave[ idx++ ] = 1; // stop bit 1
      _dmx_wave[ idx++ ] = 1; // stop bit 2
   }
}

void dmx_send(uint8_t *buf) {
  static const uint32_t minResetTime = 2;
  static Timer sendTimer( minResetTime );
  while ( !sendTimer.expireset() ); // spin until minResetTime has passed since the last send, then reset
  _dmx_buf[0] = 0;
  memcpy(_dmx_buf+1, buf, dmx_buflen);

  dmx_buildWave(); // not time-critical, interrupts stay enabled for this part

  // The only part that needs to be jitter-free: walk the precomputed
  // waveform with one unconditional write per slot, no branching on data.
  cli();
  for ( size_t i = 0; i < dmx_waveLen; ++i ) {
     *dmxReg = _dmx_wave[ i ];
     dmxDelayCycles( DMX_DELAY_ITERS );
  }
  sei();
  return;
}

#endif
