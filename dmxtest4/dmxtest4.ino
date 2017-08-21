#include "sam.h"
#include <FastLED.h>
#include "CRGBW.h"

// Units are in timer ticks. 
#define DMX_BRKTIME       (100)       // Time to hold serial in break before mark
#define DMX_MARKTIME      (25)         // Time to hold serial in mark after break
#define DMX_BAUDRATE      (250000)     // DMX baurate
#define DMX_FRAMING       (SERIAL_8N2) // Serial framing configuraiton
#define DMX_UART          (USART0)     // Due USART device (must be a USART not a UART)
#define DMX_SERIAL        (Serial1)    // Arduinoish serial object associated with above USART

/* DMX state */
bool dmx_inited = false;    // To prevent double-init
uint8_t *_dmx_buf;          // Private buffer the DMA to serial
size_t dmx_buflen;          // Buffer length

/* Init and start the DMX system */
void dmx_init(size_t buflen) {
   // Don't double-init
   if(dmx_inited == true) {
     return;
   }

   dmx_buflen = buflen;
   _dmx_buf = (uint8_t *)malloc(buflen+1);
   memset(_dmx_buf, 0, buflen+1);

   // Init serial port
   DMX_SERIAL.begin(DMX_BAUDRATE, DMX_FRAMING);
}

void dmx_send(uint8_t *buf)
{
  static const uint32_t minResetTime = 10;
  static uint32_t timeSinceLastSend = 0;
  while ( millis() - timeSinceLastSend < minResetTime );
  timeSinceLastSend = millis();
  _dmx_buf[0] = 0;
  memcpy(_dmx_buf+1, buf, dmx_buflen);
  DMX_UART->US_CR |= US_CR_STTBRK;   // Start break
  delayMicroseconds(DMX_BRKTIME);
  DMX_UART->US_CR |= US_CR_STPBRK;  // Stop break signal
  delayMicroseconds(DMX_MARKTIME);
  DMX_SERIAL.write(_dmx_buf, dmx_buflen);
  return;
}

CRGB rope[ 1024 ];
CRGBWUA dmx[ 18 ];
void setup() {
  Serial.begin(9600);
  Serial.print( "sizeof(CRGB):" );
  Serial.println( sizeof(CRGB) );
  Serial.print( "sizeof(CRGBW):" );
  Serial.println( sizeof(CRGBW) );
  Serial.print( "sizeof(CRGBWUA):" );
  Serial.println( sizeof(CRGBWUA) );
  pinMode( 32, OUTPUT );
  pinMode( 37, OUTPUT );
  digitalWrite(32, LOW); 
  digitalWrite(37, LOW); 
  FastLED.addLeds<WS2811_PORTD, 8>( rope, 128 );
  dmx_init(18*sizeof(CRGB));
}

void loop() {
  for (int i = 0; i < 1024; ++i ) {
     rope[ i ].r = random(255);
     rope[ i ].b = random(255);
     rope[ i ].g = random(255);
  }
  
  static bool loop = true;
  for(int i=0;i<18;i++) {
      dmx[i] = CRGB::White;
      // if ( loop ) {
      //    dmx[i] = CRGB::Green;
      // } else {
      //    dmx[i] = CRGB::Red;
      // }
      // dmx[i].r = random(255);
      // dmx[i].b = random(255);
      // dmx[i].g = random(255);
  }
   
   loop = !loop;
   digitalWrite(32, HIGH);
   dmx_send((uint8_t *)dmx);
   digitalWrite(32, LOW);

   digitalWrite(37, HIGH);
   FastLED.show();
   digitalWrite(37, LOW);

   delay( 1000 );
   // delayMicroseconds( 100000 );
}
