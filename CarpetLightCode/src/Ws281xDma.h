/* Ws281xDma.h
 *
 *    Drives the rope's 2 long strips (right/left, 352 LEDs each) via 2
 *    independent DMA (PDC) channels. The 2 short strips (front/rear, 156
 *    LEDs each) are NOT handled here -- see MagicCarpet.h, they're still
 *    driven via FastLED's WS2811_PORTD bank, just reduced from 8 lanes to
 *    the 2 it now actually needs.
 *
 *    Why only 2 DMA channels, not 4: the SAM3X8E has exactly 4 usable
 *    synchronous-serial-plus-PDC peripherals reachable from any Due header
 *    pin at all -- USART0/1/3 and SPI0 (checked; USART2 and SSC also exist
 *    on-chip but neither has a single pin broken out anywhere on the Due,
 *    confirmed empirically against framework-arduino-sam's variant.cpp).
 *    USART0 (pin 18) is already committed to the DMX RS-485 output
 *    (ArmDmx.h, DMX_UART) and stays there -- moving it elsewhere wouldn't
 *    free up a channel, it'd just relocate which peripheral is taken.
 *    SPI0's pin (75, "MOSI", ICSP header) was ruled out as impractical to
 *    reach on the current shield. That leaves USART1 + USART3 for the 2
 *    longest (most expensive-to-bit-bang) strips; the 2 short strips stay
 *    on the PORTD bank, now down to just the 2 lanes they need.
 *
 *    Technique: each USART is put into "USART in SPI Master mode" (a
 *    genuine hardware feature -- confirmed via component_usart.h's
 *    US_MR_USART_MODE_SPI_MASTER -- which shifts bits continuously with NO
 *    start/stop framing, unlike normal async UART mode), each with its own
 *    independent PDC (peripheral DMA controller) and its own dedicated
 *    pin, so both channels transmit simultaneously -- the CPU only kicks
 *    each one off and later polls completion, rather than bit-banging the
 *    whole transmission with interrupts disabled the way the old 8-lane
 *    PORTD-only path did for every strip. Confirmed empirically (grepped
 *    FastLED's own platforms/arm/sam/ source) that FastLED itself never
 *    uses PDC/DMA for this chip -- this is genuinely new capability, not
 *    something already available via a library flag.
 *
 *    Every WS281x logical bit is encoded as 3 output bits at a 2.4MHz
 *    shift rate (84MHz MCK / 35, an exact integer divide -- see BAUD_CD
 *    below, so there's no rounding drift over a transmission): a "0" bit
 *    becomes 100, a "1" bit becomes 110, giving a 1250ns WS281x bit period
 *    (416.7ns x 3) -- the standard WS2812 datasheet timing, comfortably
 *    inside the usual +/-150ns tolerance for both WS2811 and WS2812
 *    variants. This is the well-known "UART/SPI NeoPixel trick."
 *
 *    IMPORTANT -- NOT hardware-verified. Every register name/value below
 *    was checked against the SAM3X8E's own CMSIS headers (real facts, not
 *    guessed) and against a prior, differently-pinned implementation of
 *    this exact technique (soren54.lights@a45f735) that compiled clean,
 *    but WS281x's timing tolerance is tight enough that actual signal
 *    correctness has to be checked against the real strips -- bring up
 *    ONE channel before trusting both. One specific known unknown: whether
 *    USART-SPI-mode's TXD idles high or at the last bit's level between
 *    frames (matters for the reset gap between frames -- both bit
 *    encodings above conveniently end in a low bit, so if it idles at the
 *    last bit's level, that's already correct; if it idles high instead,
 *    that's the first thing to revisit if the strips glitch/flicker every
 *    other frame).
 *
 *    Wiring: the actual source of truth is MagicCarpet.h's NEO_PIN_RIGHT/
 *    NEO_PIN_LEFT/NEO_PIN0/NEO_PIN_REAR #defines, not this comment -- this
 *    file derives its real hardware config from whatever pin numbers get
 *    passed into setup() below (see setupUsartChannel()'s own comment).
 *    As of this writing: right=pin14 (TXD3/USART3), left=pin16 (TXD1/
 *    USART1), front=pin25 and rear=pin26 (both PORTD bank, unchanged from
 *    this file's perspective -- see MagicCarpet.h for that side).
 *    DMX stays on Due pin 18 (USART0 / TXD0), untouched.
 *    Neither USART needs its SCK pin connected to anything -- WS281x is
 *    single-wire, and the peripheral's internal shift clock runs
 *    regardless of whether SCK is physically muxed out.
 */

#ifndef __WS281X_DMA_H
#define __WS281X_DMA_H

#include <Arduino.h>

namespace Ws281xDma {

// 84MHz MCK / 35 = 2.4MHz shift rate = 416.667ns/output-bit -> 3
// output-bits per WS281x bit = 1250ns WS281x bit period (standard WS2812
// timing). Shared by both channels so they run at the same rate.
static const uint32_t BAUD_CD = 35;

// real byte count per channel -- not padded to anything else's length.
// 470 is LedUtil::resizeCRGBW(352) -- the actual CRGB-slot count
// LedUtil::convertNeoArray() writes for a 352-LED source run (see
// MagicCarpet.h) -- x3 for RGB bytes per slot.
static const uint16_t CH_BYTES = 470 * 3; // 352 LEDs
static uint8_t encodeTable[ 256 ][ 3 ];
static uint8_t rightEncoded[ CH_BYTES * 3 ];
static uint8_t leftEncoded[ CH_BYTES * 3 ];

static void buildEncodeTable() {
   for ( int v = 0; v < 256; ++v ) {
      uint32_t bits = 0; // low 24 bits used, MSB-first
      for ( int b = 7; b >= 0; --b ) {
         uint32_t triplet = ( v & ( 1 << b ) ) ? 0x6u /* 110 */ : 0x4u /* 100 */;
         bits = ( bits << 3 ) | triplet;
      }
      encodeTable[ v ][ 0 ] = (uint8_t)( bits >> 16 );
      encodeTable[ v ][ 1 ] = (uint8_t)( bits >> 8 );
      encodeTable[ v ][ 2 ] = (uint8_t)( bits );
   }
}

// Derives which USART owns a given Due pin -- this one fact (pin -> which
// USART peripheral) isn't in g_APinDescription itself (its ulPeripheralId
// field is the PIO CONTROLLER's clock ID, e.g. ID_PIOD, not the USART's),
// so it has to be told explicitly; SAM3X8E's pin-to-USART routing is fixed
// silicon, not configurable software state either way. Everything else
// (port, pin bitmask, peripheral-mux type) IS read from g_APinDescription
// at setup() time -- the actual live pin table the Arduino core itself
// uses -- rather than hand-copied literals, so MagicCarpet.h's NEO_PIN_
// RIGHT/LEFT defines are genuinely what configures this, not just
// documentation of it.
static bool usartForPin( uint32_t pin, Usart *& usart, uint32_t & periphId ) {
   if ( pin == 14 ) { usart = USART3; periphId = ID_USART3; return true; } // TXD3
   if ( pin == 16 ) { usart = USART1; periphId = ID_USART1; return true; } // TXD1
   return false; // pin isn't wired to any USART TXD on this board -- see this function's own comment
}

// configures one USART as a framing-free "SPI master" shift-out channel on
// its TXD pin only (RXD/SCK are left unconfigured -- nothing needs to read
// them). port/mask/peripheral-mux type come from g_APinDescription[pin],
// the Due's own live pin table (framework-arduino-sam's variant.cpp) --
// see usartForPin()'s comment for why the USART identity itself is still a
// small explicit fact rather than also read from that table.
static void setupUsartChannel( uint32_t pin ) {
   Usart * usart; uint32_t periphId;
   if ( !usartForPin( pin, usart, periphId ) ) {
      while ( true ) {} // wrong pin for a USART TXD -- fail loudly at boot rather than silently drive nothing
   }
   const PinDescription & pd = g_APinDescription[ pin ];
   pmc_enable_periph_clk( periphId );
   PIO_Configure( pd.pPort, pd.ulPinType, pd.ulPin, PIO_DEFAULT );
   usart->US_CR = US_CR_RSTRX | US_CR_RSTTX;
   usart->US_MR = US_MR_USART_MODE_SPI_MASTER | US_MR_USCLKS_MCK | US_MR_CHRL_8_BIT |
                  US_MR_PAR_NO | US_MR_CHMODE_NORMAL;
   usart->US_BRGR = US_BRGR_CD( BAUD_CD );
   usart->US_CR = US_CR_TXEN;
}

// resolved once in setup() below, then reused by show() every frame rather
// than re-deriving from the pin number each time.
static Usart * rightUsart_ = nullptr;
static Usart * leftUsart_ = nullptr;

// call once at boot (see MagicCarpet::setup()) -- rightPin/leftPin are
// MagicCarpet.h's NEO_PIN_RIGHT/NEO_PIN_LEFT, the real source of truth for
// this wiring.
static void setup( uint32_t rightPin, uint32_t leftPin ) {
   buildEncodeTable();
   setupUsartChannel( rightPin );
   setupUsartChannel( leftPin );
   uint32_t periphId; // discarded -- setupUsartChannel() above already used it; just need the Usart* here
   usartForPin( rightPin, rightUsart_, periphId );
   usartForPin( leftPin, leftUsart_, periphId );
}

static void encode( const uint8_t * src, uint8_t * dst, uint16_t srcBytes ) {
   for ( uint16_t i = 0; i < srcBytes; ++i ) {
      const uint8_t * enc = encodeTable[ src[ i ] ];
      dst[ i * 3 + 0 ] = enc[ 0 ];
      dst[ i * 3 + 1 ] = enc[ 1 ];
      dst[ i * 3 + 2 ] = enc[ 2 ];
   }
}

static void waitUsartDone( Usart * usart ) { while ( usart->US_TCR != 0 ); }

static void startUsartTx( Usart * usart, uint8_t * buf, uint16_t len ) {
   usart->US_TPR = (uint32_t)buf;
   usart->US_TCR = len;
   usart->US_PTCR = PERIPH_PTCR_TXTEN;
}

// call once per frame, after ropeShowLeds has been filled by
// LedUtil::convertNeoArray() exactly as before (see MagicCarpet::show())
// -- srcRight/srcLeft are the same bytes that used to go to FastLED's
// right/left PORTD lanes before those 2 moved here.
static void show( const uint8_t * srcRight, const uint8_t * srcLeft ) {
   // previous frame must be fully clocked out before its encode buffer is
   // overwritten or a new transfer started on the same channel -- in
   // practice this essentially never actually waits, since main-loop
   // cadence is far longer than either of these transfers
   waitUsartDone( rightUsart_ );
   waitUsartDone( leftUsart_ );

   encode( srcRight, rightEncoded, CH_BYTES );
   encode( srcLeft, leftEncoded, CH_BYTES );

   startUsartTx( rightUsart_, rightEncoded, CH_BYTES * 3 );
   startUsartTx( leftUsart_, leftEncoded, CH_BYTES * 3 );
}

} // namespace Ws281xDma

#endif
