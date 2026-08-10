/* main.cpp
 *
 *   CarpetLightWifiESP -- dumb-pipe WiFi<->UART bridge for CarpetLightCode's
 *   realtime radio monitor/interface mode. See platformio.ini and
 *   CarpetLightCode's README.md ("Realtime radio monitor/interface role")
 *   for the full design.
 *
 *   This firmware only ever knows about frame BOUNDARIES (a 2-byte
 *   big-endian length prefix), never what's inside them -- it hosts the
 *   WiFi AP + WebSocket server and forwards payload bytes to/from the
 *   Due's UART verbatim. All real protocol logic (message types, field
 *   layouts) lives on the Due (RadioLink class) and in the visualizer's
 *   JS. Keeping this firmware this dumb is deliberate: it should never
 *   need updating again once it's working, regardless of how the
 *   Due<->visualizer protocol evolves later.
 */

#include <WiFi.h>
#include <WebSocketsServer.h>
#include "../../wifiauth.h"

// Dedicated hardware UART to the Due (ESP32's UART2), separate from
// UART0 (used for USB-serial programming/console on this board). Default
// Arduino-ESP32 Serial2 pins: RX2=GPIO16, TX2=GPIO17. Wire ESP32
// TX2(17) -> Due's radio-link RX pin, ESP32 RX2(16) -> Due's radio-link
// TX pin, plus a shared GND -- see CarpetLightCode's plan for which Due
// pins those are (a dedicated timer-ISR bit-banged UART, not a hardware
// USART -- all 3 of the Due's usable USARTs are already committed to
// DMX and the 2 DMA-driven NeoPixel channels).
#define DUE_SERIAL Serial2
#define DUE_BAUD   115200

static const size_t MAX_FRAME_BYTES = 1024; // generous headroom -- see plan's bandwidth estimate (audio bins + occasional config deltas, a few hundred bytes/sec)

WebSocketsServer webSocket( 81 );

// ---- UART RX (Due -> ESP32 -> WebSocket) framing state ----------------
static uint8_t uartRxBuf[ MAX_FRAME_BYTES ];
static size_t uartRxLen = 0;    // payload bytes of the in-progress frame received so far
static int uartRxExpected = -1; // in-progress frame's expected payload length, -1 = still reading the length prefix
static uint8_t uartLenBuf[ 2 ];
static size_t uartLenBytesGot = 0;

static int connectedClientId = -1; // -1 = no client; single-client design (one tablet at a time, matching the visualizer's own model)

static void resetUartFrameState() {
   uartRxLen = 0;
   uartRxExpected = -1;
   uartLenBytesGot = 0;
}

// Drains whatever bytes are currently sitting in the UART's own receive
// buffer, reassembles complete frames, and forwards each one to the
// connected WebSocket client (if any) as a single binary message.
// Non-blocking -- only acts on bytes already received, never waits.
static void pumpUartToWebSocket() {
   while ( DUE_SERIAL.available() > 0 ) {
      if ( uartRxExpected < 0 ) {
         uartLenBuf[ uartLenBytesGot++ ] = (uint8_t)DUE_SERIAL.read();
         if ( uartLenBytesGot == 2 ) {
            uartRxExpected = ( (int)uartLenBuf[ 0 ] << 8 ) | (int)uartLenBuf[ 1 ];
            if ( uartRxExpected > (int)MAX_FRAME_BYTES ) {
               resetUartFrameState(); // corrupt/desynced stream -- drop, resync on the next length pair
               continue;
            }
         }
      } else {
         uartRxBuf[ uartRxLen++ ] = (uint8_t)DUE_SERIAL.read();
         if ( (int)uartRxLen >= uartRxExpected ) {
            if ( connectedClientId >= 0 ) {
               webSocket.sendBIN( connectedClientId, uartRxBuf, uartRxLen );
            }
            resetUartFrameState();
         }
      }
   }
}

// Sends one WebSocket-received payload out to the Due as a framed UART packet.
static void sendFrameToDue( const uint8_t * payload, size_t len ) {
   if ( len > MAX_FRAME_BYTES ) return; // drop oversized -- real size limits are a Due/visualizer-side protocol concern, not this firmware's
   uint8_t lenPrefix[ 2 ] = { (uint8_t)( ( len >> 8 ) & 0xFF ), (uint8_t)( len & 0xFF ) };
   DUE_SERIAL.write( lenPrefix, 2 );
   DUE_SERIAL.write( payload, len );
}

static void onWebSocketEvent( uint8_t clientId, WStype_t type, uint8_t * payload, size_t len ) {
   switch ( type ) {
      case WStype_CONNECTED:
         connectedClientId = clientId; // a new connection simply takes over -- single-client design
         break;
      case WStype_DISCONNECTED:
         if ( (int)clientId == connectedClientId ) connectedClientId = -1;
         break;
      case WStype_BIN:
         sendFrameToDue( payload, len );
         break;
      default:
         break; // binary-only protocol -- ignore text frames, pings, etc.
   }
}

void setup() {
   DUE_SERIAL.begin( DUE_BAUD );

   WiFi.mode( WIFI_AP );
   WiFi.softAP( WIFI_SSID, WIFI_PASSWORD );

   webSocket.begin();
   webSocket.onEvent( onWebSocketEvent );
}

void loop() {
   webSocket.loop();
   pumpUartToWebSocket();
}
