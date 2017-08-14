/* LightShow.h
 *
 * The base class for creating light shows. The constructor takes a MagicCarpet
 * argument. Write directly to the led arrays in that MagicCarpet instance. Read
 * potentiometer/sound input from that MagicCarpet instance.
 *
 * The start function will be called by the main logic loop whenever the mode is
 * switched to this light show. Set up any initial state here.
 *
 * The update function will be called by the main logic loop as frequently
 * as possible. It takes a time argument which represents the current system time.
 * Use this to determine when to write new values.
 *
 * TODO: add functionality to add scheduled updates and write a common time checking
 *       class that calls each update function when the desired time increment is
 *       reached.
 */

#ifndef __LIGHTSHOW_H
#define __LIGHTSHOW_H

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "MagicCarpet.h"

class LightShow {
 protected:
   MagicCarpet * carpet;
 public:
   LightShow( MagicCarpet * carpetArg ) : carpet( carpetArg ) {}
   virtual void start() = 0;
   virtual void update( uint32_t timestamp ) = 0;
 // TODO: delete all the default functions
};

#endif
