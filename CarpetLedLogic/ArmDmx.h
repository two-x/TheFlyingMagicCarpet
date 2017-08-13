/* ArmDmx.h
 *
 * A library for writing dmx output from the serial port on the Arduino Due.
 *
 * Author: Pascal wrote the core of the dmx logic, this is a port of that code
 * Date: August 2017
 */

#ifndef __ARM_DMX_H
#define __ARM_DMX_H

#ifndef __SAM_H
#define __SAM_H
#include "sam.h"
#endif

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

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

void dmx_send(uint8_t *buf) {
  _dmx_buf[0] = 0;
  memcpy(_dmx_buf+1, buf, dmx_buflen);
  DMX_UART->US_CR |= US_CR_STTBRK;   // Start break
  delayMicroseconds(DMX_BRKTIME);
  DMX_UART->US_CR |= US_CR_STPBRK;  // Stop break signal
  delayMicroseconds(DMX_MARKTIME);
  DMX_SERIAL.write(_dmx_buf, dmx_buflen);
  return;
}

#endif
