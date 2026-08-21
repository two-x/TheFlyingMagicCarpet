/* Ws281xDma.h
 *
 *    Drives 3 of the rope's 4 strips via independent DMA (PDC) channels:
 *    right/left (352 LEDs each) via USART1/USART3 in SPI-master mode, and
 *    now rear (156 LEDs) via the real SPI0 peripheral on pin 75 (MOSI, the
 *    ICSP header) -- reachable now that it's been physically soldered in
 *    (previously ruled out as impractical on the stock shield). Only front
 *    (156 LEDs) is still bit-banged, via FastLED's ClocklessController on
 *    pin 25 -- see MagicCarpet.h.
 *
 *    Why these specific peripherals: the SAM3X8E has exactly 4 usable
 *    synchronous-serial-plus-PDC peripherals reachable from any Due header
 *    pin at all -- USART0/1/3 and SPI0 (checked; USART2 and SSC also exist
 *    on-chip but neither has a single pin broken out anywhere on the Due,
 *    confirmed empirically against framework-arduino-sam's variant.cpp).
 *    USART0 (pin 18) is committed to DMX (see ArmDmx.h, real USART0
 *    hardware -- bit-banging it was tried and abandoned, see that file's
 *    own header). That leaves USART1 + USART3 (right/left) + SPI0 (rear,
 *    this file) -- all 3 of the DMA-capable peripherals this chip actually
 *    has pins for are now in use. Front is the one strand that structurally
 *    can't have one (there isn't a 4th), so it stays bit-banged.
 *
 *    Technique (right/left): each USART is put into "USART in SPI Master
 *    mode" (a genuine hardware feature -- confirmed via component_usart.h's
 *    US_MR_USART_MODE_SPI_MASTER -- which shifts bits continuously with NO
 *    start/stop framing, unlike normal async UART mode), each with its own
 *    independent PDC and its own dedicated pin, so both channels transmit
 *    simultaneously -- the CPU only kicks each one off and later polls
 *    completion, rather than bit-banging the whole transmission with
 *    interrupts disabled.
 *
 *    Technique (rear): the real SPI0 peripheral, genuinely distinct
 *    silicon from the USARTs (different register set -- Spi, not Usart --
 *    but the SAME underlying PDC block layout: every PDC-capable
 *    peripheral on this chip has its transfer-pointer/counter/control
 *    registers at a fixed +0x100 byte offset from the peripheral's own
 *    base address, confirmed against component_pdc.h's generic Pdc struct
 *    matching USART's own inline US_TPR/US_TCR/US_PTCR field offsets
 *    exactly -- so the same wait/start pattern below works for both,
 *    just addressed through a Pdc* for SPI0 instead of Usart*-typed
 *    fields). SPI0's MOSI (pin 75) is dedicated silicon, unlike USART TXD
 *    which can be one of several pins -- not pin-configurable the way
 *    setupUsartChannel() is. SPI_CSR_CSAAT (chip select stays active
 *    between transfers) is set so bytes shift out back-to-back with no
 *    artificial CS-toggle gap -- CS/SCK aren't physically connected to
 *    the LED strip at all (WS281x is single-wire, MOSI only), this bit
 *    only affects whether the PDC's own byte-to-byte timing stays
 *    continuous.
 *
 *    Confirmed empirically (grepped FastLED's own platforms/arm/sam/
 *    source) that FastLED itself never uses PDC/DMA for this chip on
 *    either peripheral type -- this is genuinely new capability, not
 *    something already available via a library flag.
 *
 *    Every WS281x logical bit is encoded as 3 output bits (a "0" bit
 *    becomes 100, a "1" bit becomes 110) at an MCK-integer-divide shift
 *    rate, giving a real WS281x bit period around the standard 1250ns
 *    WS2812 datasheet timing, comfortably inside the usual +/-150ns
 *    tolerance. This is the well-known "UART/SPI NeoPixel trick," and
 *    applies identically whether the shift register underneath is a USART
 *    in SPI mode or real SPI0 -- same encode table, same 3-bits-per-bit
 *    scheme, only the peripheral setup/PDC-kickoff differs. Right and left
 *    run at an independently relaxed rate (see BAUD_CD_RIGHT/BAUD_CD_LEFT
 *    below) that fixed real corruption on the car; rear gets its own rate
 *    (BAUD_CD_REAR) chosen to closely match its own previously-proven
 *    ~1600ns bit period from when it was bit-banged via FastLED at 25%-
 *    relaxed timing (T1=400/T2=400/T3=800ns) -- not copied from right/
 *    left's rate, which happens to be a different (also relaxed, but not
 *    identical) period.
 *
 *    IMPORTANT -- NOT hardware-verified. Every register name/value below
 *    was checked against the SAM3X8E's own CMSIS headers (real facts, not
 *    guessed), and right/left's USART-SPI-mode path is already confirmed
 *    working on the real car -- but the SPI0/rear path here is new and
 *    untested on real hardware. Bring it up and verify against a scope
 *    before trusting it the way right/left are trusted.
 *
 *    Wiring: the actual source of truth is MagicCarpet.h's NEO_PIN_RIGHT/
 *    NEO_PIN_LEFT/NEO_PIN0/NEO_PIN_REAR #defines, not this comment -- this
 *    file derives its real hardware config from whatever pin numbers get
 *    passed into setup() below. As of this writing: right=pin14 (TXD3/
 *    USART3), left=pin16 (TXD1/USART1), rear=pin75 (MOSI/SPI0), front=
 *    pin25 (PORTD bank, unchanged -- see MagicCarpet.h for that side).
 *    DMX stays on Due pin 18 (USART0 / TXD0), untouched.
 *    None of these need their SCK pin connected to anything -- WS281x is
 *    single-wire, and each peripheral's internal shift clock runs
 *    regardless of whether SCK is physically muxed out.
 */

#ifndef __WS281X_DMA_H
#define __WS281X_DMA_H

#include <Arduino.h>
#include "CRGBW.h" // for LedUtil::resizeCRGBW() -- see CH_BYTES's own comment

namespace Ws281xDma {

// 84MHz MCK / 35 = 2.4MHz shift rate = 416.667ns/output-bit -> 3
// output-bits per WS281x bit = 1250ns WS281x bit period (standard WS2812
// timing).
//
// Small baud experiments (36 slower, then 34 faster) on a single SHARED
// rate for both channels didn't resolve right/left's corruption -- see
// git history. Relaxing LEFT alone by 25% (44) confirmed fixed on the
// real car (was the jittery tail-end corruption). Per request, RIGHT now
// gets the SAME relaxed rate too, so all 4 strands run identically
// relaxed timing.
// 35 * 1.25 = 43.75 -> 44 (closest achievable integer clock divisor):
// 84MHz/44 = 1.909MHz shift rate = 523.8ns/output-bit -> 1571.4ns WS281x
// bit period, ~25.7% longer than nominal (1250ns) -- the closest integer
// CD to a genuine 25% relaxation.
static const uint32_t BAUD_CD_RIGHT = 44;
static const uint32_t BAUD_CD_LEFT = 44;
// Chosen to closely match rear's own previously-proven ~1600ns WS281x bit
// period (from its 25%-relaxed FastLED timing, T1=400+T2=400+T3=800ns) --
// NOT copied from BAUD_CD_RIGHT/LEFT, which happen to target a different
// (also relaxed) period. Target: 1600ns/3 output-bits = 533.3ns/output-bit
// -> 1.875MHz shift rate -> 84MHz/1.875MHz = 44.8 -> nearest integer 45.
// 84MHz/45 = 1.8667MHz shift rate = 535.7ns/output-bit -> 1607.1ns WS281x
// bit period -- within 0.4% of the proven-good 1600ns.
static const uint32_t BAUD_CD_REAR = 45;

// real byte count per channel -- not padded to anything else's length.
// BUGFIX: was hardcoded to `470 * 3`, a STALE value -- LedUtil.h's own
// resizeCRGBW() bugfix comment documents that 470 was the OLD, undercounting
// formula's result for 352 LEDs; the real, current value is 472 (confirmed
// by calling the real function below instead of re-deriving/hardcoding it a
// second time). The stale literal silently truncated every frame's transfer
// by 2 CRGB slots (6 real bytes) at the tail of each 352-LED strip -- not
// large enough to be the primary cause of any large-scale corruption, but a
// real, confirmed bug regardless, now eliminated by deriving from the same
// bugfixed source of truth MagicCarpet.h uses instead of hand-copying its
// result.
static const uint16_t CH_BYTES = LedUtil::resizeCRGBW( 352 ) * 3; // 352 LEDs
// rear's own real byte count -- 156 real LEDs, matching MagicCarpet.h's
// SIZEOF_SMALL_NEO (hardcoded here rather than #included, same pattern
// CH_BYTES above already uses for the 352 literal -- this file is a
// standalone, lower-level header). Keep in sync with SIZEOF_SMALL_NEO if
// that ever changes.
static const uint16_t CH_BYTES_REAR = LedUtil::resizeCRGBW( 156 ) * 3; // 156 LEDs
static uint8_t encodeTable[ 256 ][ 3 ];
static uint8_t rightEncoded[ CH_BYTES * 3 ];
static uint8_t leftEncoded[ CH_BYTES * 3 ];
static uint8_t rearEncoded[ CH_BYTES_REAR * 3 ];

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
static void setupUsartChannel( uint32_t pin, uint32_t baudCd ) {
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
   usart->US_BRGR = US_BRGR_CD( baudCd );
   usart->US_CR = US_CR_TXEN;
}

// SPI0's PDC block (transfer pointer/counter/control registers) isn't
// exposed as named fields on the Spi struct the way USART's are (US_TPR/
// US_TCR/US_PTCR) -- component_pdc.h's generic Pdc struct documents why:
// every PDC-capable peripheral on this chip has that block at a fixed
// +0x100 byte offset from its own base address (confirmed: USART's own
// inline US_TPR/US_TCR/US_PTCR sit at +0x108/+0x10C/+0x120, exactly
// Pdc's own +0x8/+0xC/+0x20 offsets plus that same +0x100). This just
// does that pointer arithmetic explicitly for SPI0.
static inline Pdc * spiPdc( Spi * spi ) { return (Pdc *)( (uint8_t *)spi + 0x100 ); }

// configures the real SPI0 peripheral as a framing-free continuous shift-
// out channel on its dedicated MOSI pin (pin 75 -- unlike USART TXD, SPI0's
// MOSI isn't a choice, it's fixed silicon, so this doesn't take a pin
// parameter the way setupUsartChannel() does). SPI_CSR_CSAAT keeps chip-
// select asserted between transfers so the PDC shifts bytes out back-to-
// back with no gap -- CS itself isn't physically wired to anything, WS281x
// only reads MOSI.
static void setupSpi0Channel( uint32_t baudCd ) {
   const uint32_t pin = 75; // SPI0 MOSI, PA26, peripheral A -- fixed, not configurable
   const PinDescription & pd = g_APinDescription[ pin ];
   pmc_enable_periph_clk( ID_SPI0 );
   PIO_Configure( pd.pPort, pd.ulPinType, pd.ulPin, PIO_DEFAULT );
   SPI0->SPI_CR = SPI_CR_SPIDIS;
   SPI0->SPI_CR = SPI_CR_SWRST;
   SPI0->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PCS( 0 ); // fixed CS0 (PS bit left clear), mode-fault detection off (not multi-master)
   SPI0->SPI_CSR[ 0 ] = SPI_CSR_CSAAT | SPI_CSR_BITS_8_BIT | SPI_CSR_SCBR( baudCd );
   SPI0->SPI_CR = SPI_CR_SPIEN;
   // DIAGNOSTIC: real, confirmed silence on pin 75 during active repeated
   // show() calls (scoped constant 3.3V, not just an idle-gap catch) --
   // right/left's PDC (proven working) never needed this, but SPI's
   // transmit-register interlock is not identical to USART's internally,
   // and PDC's "load next byte" trigger may need TDR to have been written
   // at least once before it starts firing on its own. Prime it manually
   // with one dummy byte so the pipeline is definitely warm before show()
   // ever relies on PDC's automatic per-byte trigger. If this fixes it,
   // the theory was right; if not, the bug is elsewhere and this line can
   // come back out.
   SPI0->SPI_TDR = 0;
   while ( !( SPI0->SPI_SR & SPI_SR_TXEMPTY ) );
}

// resolved once in setup() below, then reused by show() every frame rather
// than re-deriving from the pin number each time.
static Usart * rightUsart_ = nullptr;
static Usart * leftUsart_ = nullptr;
static Pdc * rearPdc_ = nullptr;

// call once at boot (see MagicCarpet::setup()) -- rightPin/leftPin are
// MagicCarpet.h's NEO_PIN_RIGHT/NEO_PIN_LEFT, the real source of truth for
// this wiring. rearPin exists only as a self-check (must be 75 -- SPI0's
// MOSI is fixed silicon, not actually configurable by this parameter) so a
// mismatch here fails loudly rather than silently driving the wrong pin.
// enableRear: DIAGNOSTIC toggle -- pin 75's real SPI0 signal is confirmed
// dead (flat 3.3V, scoped during active repeated show() calls, not just an
// idle-gap catch), so this defaults OFF while isolating whether the bug is
// in this SPI0 config or the physical connection. When off, setupSpi0Channel()
// is never called -- pin 75 stays under plain PIO control, free for
// MagicCarpet.h to bit-bang instead via the SAME proven ClocklessController
// mechanism that already worked on pin 26, just retargeted to pin 75. All
// the SPI0 code stays intact below, just unreached -- flip this back to
// true once the physical connection is confirmed good.
static const bool enableRear = false;
static void setup( uint32_t rightPin, uint32_t leftPin, uint32_t rearPin ) {
   buildEncodeTable();
   setupUsartChannel( rightPin, BAUD_CD_RIGHT );
   setupUsartChannel( leftPin, BAUD_CD_LEFT );
   uint32_t periphId; // discarded -- setupUsartChannel() above already used it; just need the Usart* here
   usartForPin( rightPin, rightUsart_, periphId );
   usartForPin( leftPin, leftUsart_, periphId );
   if ( !enableRear ) return; // see enableRear's own comment
   if ( rearPin != 75 ) {
      while ( true ) {} // SPI0's MOSI is fixed at pin 75 -- fail loudly at boot rather than silently drive nothing
   }
   setupSpi0Channel( BAUD_CD_REAR );
   rearPdc_ = spiPdc( SPI0 );
}

// BUGFIX: was encoding src[] bytes in their literal stored order (raw
// R,G,B). FastLED's own WS2811_PORTD driver -- still handling front/rear
// -- transmits in GRB order by default (its 2-arg addLeds<CHIPSET,LANES>
// overload forwards to <CHIPSET,LANES,GRB>, confirmed in FastLED.h), and
// LedUtil::convertNeo()'s whole 3-RGBW-into-4-fake-RGB packing scheme
// (CRGBW.h) is only correct when transmitted in THAT order -- each real
// RGBW chip expects a contiguous G,R,B,W nibble from the shuffled stream.
// This path bypasses FastLED entirely, so it must apply the same R<->G
// swap itself per 3-byte slot, or the packing (correct for front/rear)
// desyncs on the wire: same total byte/pixel count (so positions stay
// right), but every real LED's channels land shifted -- exactly the
// "colors off, positions fine" symptom reported on the real car.
static void encode( const uint8_t * src, uint8_t * dst, uint16_t srcBytes ) {
   for ( uint16_t i = 0; i + 2 < srcBytes; i += 3 ) {
      const uint8_t * enc0 = encodeTable[ src[ i + 1 ] ]; // wire byte 0 <- src G (slot's 2nd byte)
      const uint8_t * enc1 = encodeTable[ src[ i + 0 ] ]; // wire byte 1 <- src R (slot's 1st byte)
      const uint8_t * enc2 = encodeTable[ src[ i + 2 ] ]; // wire byte 2 <- src B, unchanged
      dst[ ( i + 0 ) * 3 + 0 ] = enc0[ 0 ]; dst[ ( i + 0 ) * 3 + 1 ] = enc0[ 1 ]; dst[ ( i + 0 ) * 3 + 2 ] = enc0[ 2 ];
      dst[ ( i + 1 ) * 3 + 0 ] = enc1[ 0 ]; dst[ ( i + 1 ) * 3 + 1 ] = enc1[ 1 ]; dst[ ( i + 1 ) * 3 + 2 ] = enc1[ 2 ];
      dst[ ( i + 2 ) * 3 + 0 ] = enc2[ 0 ]; dst[ ( i + 2 ) * 3 + 1 ] = enc2[ 1 ]; dst[ ( i + 2 ) * 3 + 2 ] = enc2[ 2 ];
   }
}

static void waitUsartDone( Usart * usart ) { while ( usart->US_TCR != 0 ); }
static void waitPdcDone( Pdc * pdc ) { while ( pdc->PERIPH_TCR != 0 ); }

static void startUsartTx( Usart * usart, uint8_t * buf, uint16_t len ) {
   usart->US_TPR = (uint32_t)buf;
   usart->US_TCR = len;
   usart->US_PTCR = PERIPH_PTCR_TXTEN;
}
static void startPdcTx( Pdc * pdc, uint8_t * buf, uint16_t len ) {
   pdc->PERIPH_TPR = (uint32_t)buf;
   pdc->PERIPH_TCR = len;
   pdc->PERIPH_PTCR = PERIPH_PTCR_TXTEN;
}

// call once per frame, after ropeShowLeds/rearShowLeds have been filled by
// LedUtil::convertNeoArray() exactly as before (see MagicCarpet::show())
// -- srcRight/srcLeft/srcRear are the same bytes that used to go to
// FastLED's PORTD lanes before these strands moved here.
static void show( const uint8_t * srcRight, const uint8_t * srcLeft, const uint8_t * srcRear ) {
   // previous frame must be fully clocked out before its encode buffer is
   // overwritten or a new transfer started on the same channel -- in
   // practice this essentially never actually waits, since main-loop
   // cadence is far longer than any of these transfers
   waitUsartDone( rightUsart_ );
   waitUsartDone( leftUsart_ );

   encode( srcRight, rightEncoded, CH_BYTES );
   encode( srcLeft, leftEncoded, CH_BYTES );

   startUsartTx( rightUsart_, rightEncoded, CH_BYTES * 3 );
   startUsartTx( leftUsart_, leftEncoded, CH_BYTES * 3 );

   if ( !enableRear ) return; // see enableRear's own comment
   waitPdcDone( rearPdc_ );
   encode( srcRear, rearEncoded, CH_BYTES_REAR );
   startPdcTx( rearPdc_, rearEncoded, CH_BYTES_REAR * 3 );
}

} // namespace Ws281xDma

#endif
