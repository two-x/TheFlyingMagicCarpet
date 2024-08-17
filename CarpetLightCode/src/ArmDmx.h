/* ArmDmx.h
 *
 * A library for writing dmx output from the serial port on the Arduino Due.
 *
 * Author: Pascal wrote the core of the dmx logic, this is a port of that code
 * Date: August 2017
 */

#ifndef __ARM_DMX_H
#define __ARM_DMX_H

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include <esp_dmx.h>

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

const dmx_port_t dmx_num = DMX_NUM_1;
const uint8_t dmx_pin = 17;

/* Init and start the DMX system */
void dmx_init(size_t buflen) {
   // Don't double-init
   if(dmx_inited == true) {
     return;
   }

   dmx_buflen = buflen;
   _dmx_buf = (uint8_t *)malloc(buflen+1);
   memset(_dmx_buf, 0, buflen+1);

  // First, use the default DMX configuration...
  dmx_config_t config = DMX_CONFIG_DEFAULT;

  // ...declare the driver's DMX personalities...
  const int personality_count = 1;
  dmx_personality_t personalities[] = {
    {1, "Default Personality"}
  };

  // ...install the DMX driver...
  dmx_driver_install(dmx_num, &config, personalities, personality_count);

  // ...and then set the communication pins!
  const int tx_pin = 17;
  // const int rx_pin = 16;
  // const int rts_pin = 21;
  dmx_set_pin(dmx_num, tx_pin, DMX_PIN_NO_CHANGE, DMX_PIN_NO_CHANGE);
}

void dmx_send(uint8_t *buf) {
  static const uint32_t minResetTime = 2;
  static uint32_t timeSinceLastSend = 0;
  while ( millis() - timeSinceLastSend < minResetTime );
  timeSinceLastSend = millis();
  _dmx_buf[0] = 0;
  memcpy(_dmx_buf+1, buf, dmx_buflen);

  // what is this break all about?
  // DMX_UART->US_CR |= US_CR_STTBRK;   // Start break
  // delayMicroseconds(DMX_BRKTIME);
  // DMX_UART->US_CR |= US_CR_STPBRK;  // Stop break signal
  // delayMicroseconds(DMX_MARKTIME);

  // Write to the packet and send it.
  dmx_write(dmx_num, _dmx_buf, dmx_buflen);
  dmx_send(dmx_num);
  // Block until the packet is finished sending.
  dmx_wait_sent(dmx_num, DMX_TIMEOUT_TICK);
  return;
}

#endif
