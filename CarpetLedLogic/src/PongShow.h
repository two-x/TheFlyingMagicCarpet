/* PongShow.h
 *
 * TODO
 *
 * Author: Anders Linn
 * Date: August 2017
 */

#include <pair>
#include "LightShow.h"

#define PONG_BALL_RADIUS 50
#define PONG_EDGE_SPACE 25

class PongShow : public LightShow {
 private:
   // carpet is 216 x 290
   uint16_t grid[ SIZEOF_SMALL_NEO + PONG_EDGE_SPACE ][ SIZEOF_LARGE_NEO + PONG_EDGE_SPACE ] = 0;
   std::pair<uint16_t,uint16_t> center;

 public:
   PongShow( MagicCarpet * carpetArg ) : LightShow( carpetArg ) {}

   void start() {
      uint16_t r = random16();
      uint16_t c = random16();

      r = r > 

      center = std::make_pair( random16(), random16() );
      if (
   }

   void update();
};

