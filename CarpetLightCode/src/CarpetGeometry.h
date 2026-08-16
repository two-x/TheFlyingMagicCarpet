/* CarpetGeometry.h
 *
 *    Single centralized source of truth for every physical/geometric fact
 *    about the car and its lighting fixtures -- vehicle model dimensions,
 *    where each megabar/china actually is, and where each individual
 *    neopixel sits along the rope's perimeter. Every value here is either
 *    a fixed compile-time constant, or derived exactly ONCE by begin()
 *    (call from setup(), before any light show runs) into a plain array --
 *    no light show should ever recompute this geometry itself at runtime.
 *
 *    Coordinate convention used throughout: origin at the car's own
 *    center, X+ = right, Y+ = FRONT (forward direction of travel), all
 *    distances in feet unless a name says otherwise (e.g. *Percent).
 *    Angles are compass-style, clockwise from forward: 0=front, 90=right,
 *    180=back, 270=left -- same convention LighthouseShow.h/
 *    SpeedStripesShow.h already use.
 *
 *    Fixture (x,y) positions below are ported from the visualizer
 *    prototype's own hand-tuned MEGABAR_POS_FT/CHINA_POS tables (tools/
 *    visualizer/carpet-visualizer.html) -- the most carefully-measured
 *    version of this data that existed anywhere in this codebase before
 *    now, just never centralized or available to firmware. Z heights are
 *    from the visualizer's own FIXTURE_HEIGHTS_FT. Per explicit request,
 *    beamYawDeg/beamPitchDeg in getFloodMount() are left at 0 for now --
 *    real measured values to replace them are still to come; everything
 *    else (light-show angle math) already derives from the X,Y positions
 *    and each fixture's own beamAngleDeg in getFloodPosition() instead, so
 *    those 0es don't block anything currently using this file.
 */

#ifndef __CARPET_GEOMETRY_H
#define __CARPET_GEOMETRY_H

#include <math.h>
#include "MagicCarpet.h" // NUM_MEGABAR_LEDS, NUM_CHINA_LEDS, SIZEOF_SMALL_NEO, SIZEOF_LARGE_NEO, NUM_NEO_LEDS_ACTUAL

class CarpetGeometry {
 public:
   enum FixtureType { FixtureMegabar = 0, FixtureChina = 1 };
   enum Strand { StrandFront = 0, StrandRight = 1, StrandBack = 2, StrandLeft = 3 };
   enum Side { SideRight = 0, SideLeft = 1 };

   /* ---- Vehicle 3D model ---- */
   static constexpr float CAR_WIDTH_FT = 12.0f;  // left-right
   static constexpr float CAR_LENGTH_FT = 16.0f; // front-back
   // PLACEHOLDER -- not yet measured on the real car; a plausible fringe/
   // tassel trim length (6in) so callers have a real number to work with
   // rather than 0. Replace once measured.
   static constexpr float FRINGE_LENGTH_FT = 0.5f;
   // PLACEHOLDER -- the carpet surface isn't perfectly flat (real fabric
   // over a frame sags/domes slightly); these define a simple sine
   // undulation on top of a flat base until real measured contour data is
   // available. Amplitude is deliberately small (1.2in) so it's a subtle
   // ripple, not a dominant shape; wavelength set to 1 full cycle across
   // the car's own length.
   static constexpr float SURFACE_BASE_HEIGHT_FT = 0.0f;
   static constexpr float SURFACE_UNDULATION_AMPLITUDE_FT = 0.1f;
   static constexpr float SURFACE_UNDULATION_WAVELENGTH_FT = CAR_LENGTH_FT;

   // Surface height (feet above SURFACE_BASE_HEIGHT_FT) at a car-relative
   // (x,y) point, feet -- a simple 2D sine ripple running front-to-back.
   // Cheap enough to call per-frame/per-LED if ever needed, but still only
   // ever needs this one central definition.
   static float getSurfaceHeightAt( float xFt, float yFt ) {
      (void)xFt; // undulation currently runs only along Y (front-back); reserved for a future X-dependent term
      return SURFACE_BASE_HEIGHT_FT + SURFACE_UNDULATION_AMPLITUDE_FT *
             sinf( 2.0f * PI * yFt / SURFACE_UNDULATION_WAVELENGTH_FT );
   }
   // Same, from polar (distance, angle-from-forward) instead of (x,y).
   static float getSurfaceHeightAtPolar( float distFt, float angleFromForwardDeg ) {
      float rad = angleFromForwardDeg * PI / 180.0f;
      float xFt = distFt * sinf( rad );
      float yFt = distFt * cosf( rad );
      return getSurfaceHeightAt( xFt, yFt );
   }

   /* ---- Array 1: flood fixture positions (overhead projection) --------
      One entry per megabar + china, X/Y/beam angle referenced to car
      center, pointed forward. beamAngleDeg is each fixture's own overhead
      aim direction (compass convention above) -- for megabars this is
      also their physical mounting angle (wrap360(-m*30), matching
      LighthouseShow.h's existing convention); for china it's derived from
      which corner + which of the pair (front/back-aimed vs side-aimed)
      each one is, per README.md's china layout. */
   struct FloodPosition {
      uint8_t id;         // index within its own type (0-11 megabar, 0-7 china)
      FixtureType type;
      float beamAngleDeg; // overhead aim direction, compass-from-forward
      float xFt, yFt;      // mounting position, car-center-relative
      float positionAngleDeg; // derived: compass angle of this fixture's OWN position (atan2 of xFt,yFt), same convention as NeoGeom::angleFromForwardDeg -- for megabars this equals beamAngleDeg (aim==position, see below), for china it does NOT (china's beamAngleDeg is aim direction, not position) -- precomputed here so callers who want "where a rotating beam sweeps past" (not "which way this fixture points") never need their own live atan2f, e.g. LighthouseShow.h's china-beam-crossing effect
      float distanceFt;   // derived: distance from car center, same convention as NeoGeom::distanceFt
   };

   /* ---- Array 2: flood fixture mounting height + orientation ----------
      Ground-referenced (Z=0 at the ground under car center). X/Y here
      currently duplicate FloodPosition's (only the visualizer's own 3D
      model uses X/Y directly today; firmware light shows use
      getFloodPosition() instead) -- kept as its own array/struct anyway,
      matching the requested layout, since Z/yaw/pitch belong with mount
      data conceptually even though X/Y are redundant with array 1 today. */
   struct FloodMount {
      uint8_t id;
      FixtureType type;
      float xFt, yFt, zFt;      // zFt from the visualizer's own measured FIXTURE_HEIGHTS_FT
      float beamYawDeg, beamPitchDeg; // 0 for now -- see file header comment
   };

   /* ---- Array 3: per-neopixel abstracted perimeter geometry ------------
      Index 0 = front-left corner (matches the real wiring direction:
      front strand runs left->right, right side front->back, back strand
      right->left, left side back->front -- see README.md "Perimeter rope
      lights"). X/Y are percent of car width/length (-50..+50, 0=center) --
      distanceFt/angleFromForwardDeg are derived from those once, here,
      not left for every caller to recompute. */
   struct NeoGeom {
      Strand strand;
      Side side;          // which half of the car this pixel is geometrically on, regardless of which strand it's part of
      float xPercent, yPercent; // -50..+50, of CAR_WIDTH_FT/CAR_LENGTH_FT respectively
      float distanceFt;   // derived: distance from car center
      float angleFromForwardDeg; // derived: compass angle, 0=front/forward
   };

   static uint16_t getNumPixelsOnStrand( Strand s ) {
      return ( s == StrandFront || s == StrandBack ) ? SIZEOF_SMALL_NEO : SIZEOF_LARGE_NEO;
   }
   static uint16_t getNumPixelsTotal() { return NUM_NEO_LEDS_ACTUAL; }

   static const FloodPosition & getFloodPosition( FixtureType type, uint8_t idx ) {
      return floodPositions_[ globalFloodIndex_( type, idx ) ];
   }
   static const FloodPosition & getMegabarPosition( uint8_t m ) { return getFloodPosition( FixtureMegabar, m ); }
   static const FloodPosition & getChinaPosition( uint8_t c ) { return getFloodPosition( FixtureChina, c ); }

   static const FloodMount & getFloodMount( FixtureType type, uint8_t idx ) {
      return floodMounts_[ globalFloodIndex_( type, idx ) ];
   }
   static const FloodMount & getMegabarMount( uint8_t m ) { return getFloodMount( FixtureMegabar, m ); }
   static const FloodMount & getChinaMount( uint8_t c ) { return getFloodMount( FixtureChina, c ); }

   static const NeoGeom & getNeoGeom( uint16_t pixelIdx ) { return neoGeom_[ pixelIdx ]; }

   // Call once from setup(), before any light show runs.
   static void begin() {
      buildFloodPositions_();
      buildFloodMounts_();
      buildNeoGeom_();
   }

 private:
   static const uint8_t NUM_FLOODS_ = NUM_MEGABAR_LEDS + NUM_CHINA_LEDS; // 20
   static FloodPosition floodPositions_[ NUM_FLOODS_ ];
   static FloodMount floodMounts_[ NUM_FLOODS_ ];
   static NeoGeom neoGeom_[ NUM_NEO_LEDS_ACTUAL ];

   static uint8_t globalFloodIndex_( FixtureType type, uint8_t idx ) {
      return ( type == FixtureMegabar ) ? idx : ( NUM_MEGABAR_LEDS + idx );
   }

   static float wrap360_( float deg ) {
      while ( deg < 0.0f ) deg += 360.0f;
      while ( deg >= 360.0f ) deg -= 360.0f;
      return deg;
   }
   // compass angle (0=front,90=right,180=back,270=left) of a car-relative (x,y) point
   static float angleFromForward_( float xFt, float yFt ) {
      return wrap360_( atan2f( xFt, yFt ) * 180.0f / PI );
   }

   // Megabar (x,y), ft, Y+=front -- ported from the visualizer's
   // MEGABAR_POS_FT (see its own long comment there for how these 12
   // positions were hand-tuned); that table uses Y+=BACK, so signs are
   // flipped here to match this file's Y+=front convention.
   static void buildFloodPositions_() {
      static const float halfW = CAR_WIDTH_FT / 2.0f, halfH = CAR_LENGTH_FT / 2.0f; // 6, 8
      static const float STD_INSET_FT = 2.5f;
      static const float FRONT_INSET_FT = 2.0f, BACK_INSET_FT = 1.5f;
      static const float FRONT_NEAR_GAP_FT = 0.3f;
      static const float CORNER_PULL_FT = 1.0f;
      const float sideEdgeY = halfW / tanf( 60.0f * PI / 180.0f ); // ~3.46ft
      const float centerX = halfW - STD_INSET_FT;                  // 3.5ft
      const float nearX = centerX + FRONT_NEAR_GAP_FT - CORNER_PULL_FT; // 2.8ft
      const float mbXY[ NUM_MEGABAR_LEDS ][ 2 ] = {
         {         0.0f,  ( halfH - FRONT_INSET_FT ) }, // 0 front-facing (2 lamps)
         {      -nearX,   ( halfH - STD_INSET_FT ) },   // 1
         {    -centerX,    sideEdgeY },                  // 2
         {    -centerX,    0.0f },                        // 3
         {    -centerX,   -sideEdgeY },                  // 4
         {      -nearX,  -( halfH - STD_INSET_FT ) },   // 5
         {         0.0f, -( halfH - BACK_INSET_FT ) }, // 6 back-facing
         {       nearX,  -( halfH - STD_INSET_FT ) },   // 7
         {     centerX,   -sideEdgeY },                  // 8
         {     centerX,    0.0f },                        // 9
         {     centerX,    sideEdgeY },                  // 10
         {       nearX,   ( halfH - STD_INSET_FT ) },   // 11
      };
      for ( uint8_t m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         FloodPosition & p = floodPositions_[ globalFloodIndex_( FixtureMegabar, m ) ];
         p.id = m; p.type = FixtureMegabar;
         p.xFt = mbXY[ m ][ 0 ]; p.yFt = mbXY[ m ][ 1 ];
         p.beamAngleDeg = wrap360_( -(float)m * 30.0f ); // matches LighthouseShow.h's megabar angle convention
         p.positionAngleDeg = angleFromForward_( p.xFt, p.yFt ); // == beamAngleDeg for megabars (mounting angle IS aim angle here), computed independently anyway so callers never have to know/assume that equivalence
         p.distanceFt = sqrtf( p.xFt * p.xFt + p.yFt * p.yFt );
      }

      // CAVEAT: china's beamAngleDeg below is its AIM direction (ported
      // from the visualizer's own CHINA_AIM_DEG), which is a DIFFERENT
      // thing from its POSITION angle (positionAngleDeg, precomputed below
      // for every fixture) -- e.g. LighthouseShow.h's china-beam-crossing
      // effect uses position, not aim, since it cares about where a
      // rotating beam sweeps PAST, not which way the china itself points).
      // The exact
      // numeric convention CHINA_AIM_DEG's source data uses hasn't been
      // independently cross-checked against a real photo/measurement --
      // treat beamAngleDeg as approximate until verified on the car.
      //
      // China: 2 fixtures per corner (front-right/front-left/back-left/
      // back-right), one aimed along the near front/back edge, one aimed
      // along the near side edge. BUGFIX (mandate violation, not a
      // measurement question): this used to treat both fixtures in a pair
      // as co-located, deliberately dropping the visualizer's own
      // CHINA_POS nudge as "cosmetic, just for rendering legibility" --
      // but that reasoning was backwards. Two real, distinct fixtures
      // physically cannot occupy the same point; the nudge's DIRECTION in
      // the visualizer's table isn't arbitrary either -- each fixture is
      // offset toward its own real aim direction (the side-aimed one sits
      // slightly further out to the side, the front/back-aimed one
      // slightly further toward that edge), which is exactly the
      // physically sensible way to mount 2 fixtures sharing one corner
      // bracket but aimed 90 degrees apart. Only the exact MAGNITUDE
      // (chosen in the visualizer for 2 rendered dots to look distinct,
      // not from a tape measure) was ever actually cosmetic -- kept here
      // as the best available estimate (0.262ft) until a real measurement
      // replaces it. This is now the ONE real source for this geometry;
      // the visualizer reads it back rather than keeping its own nudged
      // copy (see web_getChinaXFt/YFt in web_bridge.cpp).
      static const float CORNER_INSET_FT = 1.25f;
      static const float cx = halfW - CORNER_INSET_FT, cy = halfH - CORNER_INSET_FT;
      static const float CHINA_NUDGE_FT = 0.262f; // see BUGFIX comment above -- estimate, not yet measured
      static const float rotationFactor = 15.0f; // matches CHINA_AIM_DEG's own "X" constant
      const float chXY[ NUM_CHINA_LEDS ][ 2 ] = {
         {  cx + CHINA_NUDGE_FT,  cy },                    // 0 front-right, aimed side  -> nudged further right
         {  cx,  cy + CHINA_NUDGE_FT },                    // 1 front-right, aimed front -> nudged further front
         { -cx,  cy + CHINA_NUDGE_FT },                    // 2 front-left, aimed front  -> nudged further front
         { -cx - CHINA_NUDGE_FT,  cy },                    // 3 front-left, aimed side   -> nudged further left
         { -cx - CHINA_NUDGE_FT, -cy },                    // 4 back-left, aimed side    -> nudged further left
         { -cx, -cy - CHINA_NUDGE_FT },                    // 5 back-left, aimed back    -> nudged further back
         {  cx, -cy - CHINA_NUDGE_FT },                    // 6 back-right, aimed back   -> nudged further back
         {  cx + CHINA_NUDGE_FT, -cy },                    // 7 back-right, aimed side   -> nudged further right
      };
      // beam aim, compass-from-forward (front=0,right=90,back=180,left=270)
      // -- converted from CHINA_AIM_DEG's own front=180 convention by
      // adding 180 and wrapping.
      const float chAimDeg[ NUM_CHINA_LEDS ] = {
         wrap360_( ( 180.0f - rotationFactor ) + 180.0f ), // 0 aimed side
         wrap360_( ( 270.0f + rotationFactor ) + 180.0f ), // 1 aimed front
         wrap360_( (  90.0f - rotationFactor ) + 180.0f ), // 2 aimed front
         wrap360_( ( 180.0f + rotationFactor ) + 180.0f ), // 3 aimed side
         wrap360_( ( 360.0f - rotationFactor ) + 180.0f ), // 4 aimed side
         wrap360_( (  90.0f + rotationFactor ) + 180.0f ), // 5 aimed back
         wrap360_( ( 270.0f - rotationFactor ) + 180.0f ), // 6 aimed back
         wrap360_( (   0.0f + rotationFactor ) + 180.0f ), // 7 aimed side
      };
      for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) {
         FloodPosition & p = floodPositions_[ globalFloodIndex_( FixtureChina, c ) ];
         p.id = c; p.type = FixtureChina;
         p.xFt = chXY[ c ][ 0 ]; p.yFt = chXY[ c ][ 1 ];
         p.beamAngleDeg = chAimDeg[ c ];
         p.positionAngleDeg = angleFromForward_( p.xFt, p.yFt ); // NOT beamAngleDeg -- china's aim direction differs from its position, see this loop's caveat comment above
         p.distanceFt = sqrtf( p.xFt * p.xFt + p.yFt * p.yFt );
      }
   }

   static void buildFloodMounts_() {
      // Z heights from the visualizer's own FIXTURE_HEIGHTS_FT (megabarFront:
      // 1.0, megabarDefault: 1.5, china: 2.0) -- the only real measured mount
      // heights that exist anywhere in this codebase so far.
      static const float MEGABAR_FRONT_Z_FT = 1.0f, MEGABAR_DEFAULT_Z_FT = 1.5f, CHINA_Z_FT = 2.0f;
      for ( uint8_t m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         const FloodPosition & pos = getMegabarPosition( m );
         FloodMount & mnt = floodMounts_[ globalFloodIndex_( FixtureMegabar, m ) ];
         mnt.id = m; mnt.type = FixtureMegabar;
         mnt.xFt = pos.xFt; mnt.yFt = pos.yFt;
         mnt.zFt = ( m == 0 ) ? MEGABAR_FRONT_Z_FT : MEGABAR_DEFAULT_Z_FT; // m0 = headlight, the front-facing pair
         mnt.beamYawDeg = 0.0f; mnt.beamPitchDeg = 0.0f; // see file header comment -- not yet measured
      }
      for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) {
         const FloodPosition & pos = getChinaPosition( c );
         FloodMount & mnt = floodMounts_[ globalFloodIndex_( FixtureChina, c ) ];
         mnt.id = c; mnt.type = FixtureChina;
         mnt.xFt = pos.xFt; mnt.yFt = pos.yFt;
         mnt.zFt = CHINA_Z_FT;
         mnt.beamYawDeg = 0.0f; mnt.beamPitchDeg = 0.0f;
      }
   }

   // Front strand: Y=+50%, X sweeps -50%(left)..+50%(right) as index rises.
   // Right strand: X=+50%, Y sweeps +50%(front)..-50%(back).
   // Back strand:  Y=-50%, X sweeps +50%(right)..-50%(left).
   // Left strand:  X=-50%, Y sweeps -50%(back)..+50%(front).
   // (front L->R, right F->B, back R->L, left B->F -- matches the real
   // wiring direction, see README.md "Perimeter rope lights".)
   static void buildNeoGeom_() {
      uint16_t i = 0;
      for ( uint16_t k = 0; k < SIZEOF_SMALL_NEO; ++k, ++i ) { // front
         float u = (float)k / (float)( SIZEOF_SMALL_NEO - 1 );
         setNeoGeom_( i, StrandFront, -50.0f + 100.0f * u, 50.0f );
      }
      for ( uint16_t k = 0; k < SIZEOF_LARGE_NEO; ++k, ++i ) { // right
         float u = (float)k / (float)( SIZEOF_LARGE_NEO - 1 );
         setNeoGeom_( i, StrandRight, 50.0f, 50.0f - 100.0f * u );
      }
      for ( uint16_t k = 0; k < SIZEOF_SMALL_NEO; ++k, ++i ) { // back
         float u = (float)k / (float)( SIZEOF_SMALL_NEO - 1 );
         setNeoGeom_( i, StrandBack, 50.0f - 100.0f * u, -50.0f );
      }
      for ( uint16_t k = 0; k < SIZEOF_LARGE_NEO; ++k, ++i ) { // left
         float u = (float)k / (float)( SIZEOF_LARGE_NEO - 1 );
         setNeoGeom_( i, StrandLeft, -50.0f, -50.0f + 100.0f * u );
      }
   }
   static void setNeoGeom_( uint16_t i, Strand strand, float xPercent, float yPercent ) {
      NeoGeom & g = neoGeom_[ i ];
      g.strand = strand;
      g.side = ( xPercent < 0.0f ) ? SideLeft : SideRight;
      g.xPercent = xPercent; g.yPercent = yPercent;
      float xFt = xPercent / 100.0f * CAR_WIDTH_FT, yFt = yPercent / 100.0f * CAR_LENGTH_FT;
      g.distanceFt = sqrtf( xFt * xFt + yFt * yFt );
      g.angleFromForwardDeg = angleFromForward_( xFt, yFt );
   }
};

CarpetGeometry::FloodPosition CarpetGeometry::floodPositions_[ CarpetGeometry::NUM_FLOODS_ ];
CarpetGeometry::FloodMount CarpetGeometry::floodMounts_[ CarpetGeometry::NUM_FLOODS_ ];
CarpetGeometry::NeoGeom CarpetGeometry::neoGeom_[ NUM_NEO_LEDS_ACTUAL ];

#endif
