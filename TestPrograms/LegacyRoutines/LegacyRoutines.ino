/* LegacyRoutines.ino
 *
 * Entry point for running the ported version of eliot's code. Mostly of interest
 * for debugging, and if we ever want to get retro.
 *
 *   Author: Anders Linn
 *   Date: July 2017
 */

#ifndef __DMXSIMPLE_H
#define __DMXSIMPLE_H
#include <DmxSimple.h>
#endif

#ifndef __FASTLED_H
#define __FASTLED_H
#include <FastLED.h>
#endif

#include "CRGBW.h"
#include "MagicCarpet.h"
#include "LegacyRoutines.h"

void setup() {
   LegacyRoutines::dmxSetup();
}

void loop() {
   LegacyRoutines::dmxLoop();
}
