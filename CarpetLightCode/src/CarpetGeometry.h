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
 *    now, just never centralized or available to firmware.
 *
 *    ---- DimX/DimY vs FixtureDimX/FixtureDimY/FixtureDimZ (per explicit
 *    spec) ----
 *    Every FloodGeometry record stores BOTH of these, since real hardware
 *    genuinely has both, but they mean different things and have very
 *    different consumers:
 *      - dimX/dimY ("Dim") -- the GROUND SPOT: where a fixture's light
 *        actually lands. This is the ONLY geometry any real light-show
 *        algorithm should ever consult (dimAngleDeg/dimDistFt are derived
 *        from it once, here). China fixtures are mounted in pairs that
 *        share one corner bracket but aim along DIFFERENT edges (one
 *        front/back, one side) -- their ground spots are two genuinely
 *        DISTINCT points, well separated in angle (~14deg vs ~66deg for
 *        the front-right pair, confirmed empirically -- nowhere close to
 *        co-located, and also nowhere close to a naive 45deg-apart
 *        guess), computed from README.md's own "~1/3 of the way along its
 *        assigned edge" spec. An earlier version of this file modeled
 *        these as co-located -- a real bug (not a measurement question),
 *        since it silently changed what every real show reading china
 *        angle actually sees; see buildFloods_()'s own comment.
 *      - fixtureDimX/fixtureDimY/fixtureDimZ ("FixtureDim") -- the
 *        physical FIXTURE MOUNT position. Visualizer-3D-rendering-only;
 *        confirmed no real light show reads this. A completely separate,
 *        much smaller-scale concept than the ground-spot separation above
 *        -- 2 real china fixtures sharing one corner bracket are mounted
 *        only ~0.26ft apart, vs. several feet of real ground-spot
 *        separation driven by aim/distance. Each china's fixtureDimX/Y
 *        below is its own individually hand-entered estimate -- there is
 *        no shared offset formula/constant computing these from dimX/Y;
 *        they're independent values, not a derived adjustment.
 *      - For megabars, no independent mount-offset is known, so
 *        fixtureDimX/Y just copy dimX/Y; only fixtureDimZ (mount height)
 *        is independently meaningful there.
 *
 *    Per explicit request, beamYawDeg/beamPitchDeg (a fixture's own aim
 *    direction, as opposed to where its light lands) are NOT stored here
 *    at all -- confirmed no real show reads anything but ground-spot
 *    position; a fixture's real aim direction is a visualizer-rendering-
 *    only concern, computed there from fixture-mount vs. ground-spot
 *    geometry, not duplicated here.
 */

#ifndef __CARPET_GEOMETRY_H
#define __CARPET_GEOMETRY_H

#include <math.h>
#include "MagicCarpet.h" // NUM_MEGABAR_LEDS, NUM_CHINA_LEDS, SIZEOF_SMALL_NEO, SIZEOF_LARGE_NEO, NUM_NEO_LEDS_ACTUAL

class CarpetGeometry {
 public:
   enum FixtureType { FixtureMegabar = 0, FixtureChina = 1 };
   // Strand = a physical WIRING concept (which wire run a pixel is on) --
   // internal plumbing only. No public geometry getter/setter light shows
   // touch should ever take a Strand parameter; see CarSide below, which
   // is what those actually take. Kept distinct on purpose: today the two
   // happen to correspond 1:1 (front strand == front side, etc.), but a
   // light show's "which side of the car" query is a geometric concept
   // that must stay decoupled from however the perimeter happens to be
   // wired -- that coupling is assumed in exactly one place, strandForSide_().
   enum Strand { StrandFront = 0, StrandRight = 1, StrandBack = 2, StrandLeft = 3 };
   // CarSide = which of the 4 sides of the CAR -- the geometric concept
   // every neo position getter/setter below actually takes. Distinct from
   // Strand (hardware wiring, see above) and from the (unrelated,
   // pre-existing) binary Side enum below (which classifies a pixel as
   // simply left-half/right-half of the car, regardless of which of the 4
   // sides or strands it's on).
   enum CarSide { CarSideFront = 0, CarSideRight = 1, CarSideBack = 2, CarSideLeft = 3 };
   enum Side { SideRight = 0, SideLeft = 1 };

   // Named indices, in DMX address order (see README.md, "Megabars"/"China
   // lights") -- callers should prefer these over raw integer indices
   // wherever the specific fixture matters, e.g. CarpetGeometry::getChina(
   // CarpetGeometry::ChinaFrontRightFront) rather than getChina(1).
   enum MegabarName {
      Megabar0deg = 0, Megabar30deg, Megabar60deg, Megabar90deg, Megabar120deg, Megabar150deg,
      Megabar180deg, Megabar210deg, Megabar240deg, Megabar270deg, Megabar300deg, Megabar330deg,
      NumMegabars
   };
   enum ChinaName {
      ChinaFrontRightSide = 0,  // addr 37 -- front-right corner, aimed along the right (side) edge
      ChinaFrontRightFront,     // addr 43 -- front-right corner, aimed along the front edge
      ChinaFrontLeftFront,      // addr 49 -- front-left corner, aimed along the front edge
      ChinaFrontLeftSide,       // addr 55 -- front-left corner, aimed along the left (side) edge
      ChinaBackLeftSide,        // addr 61 -- back-left corner, aimed along the left (side) edge
      ChinaBackLeftBack,        // addr 67 -- back-left corner, aimed along the back edge
      ChinaBackRightBack,       // addr 73 -- back-right corner, aimed along the back edge
      ChinaBackRightSide,       // addr 79 -- back-right corner, aimed along the right (side) edge
      NumChinas
   };

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

   /* ---- Centralized per-fixture geometry record -- one entry per
      megabar + china. See file header comment for the dim* vs
      fixtureDim* distinction. */
   struct FloodGeometry {
      uint8_t id;          // index within its own type (0-11 megabar, 0-7 china)
      FixtureType type;
      uint8_t dmxAddress;  // megabar: 1+3*id; china: 37+6*id -- see README.md

      float dimX, dimY;         // ground-spot position, car-center-relative, ft -- what light shows use
      // Both derived from dimX/dimY (hardcoded above), same simple 2-point
      // polar conversion for every flood, megabar and china alike:
      //   dimAngleDeg = angle of the straight segment from car center to
      //                 this flood's own ground spot (atan2 of dimX,dimY)
      //   dimDistFt   = length of that same segment
      // This is what makes getMegabarByAngle()/getChinaByAngle() (and every
      // real light show that uses them) land exactly on the hardcoded spot
      // table above, whatever it currently says -- no separate "nominal"
      // angle is ever consulted. Fixture names (Megabar30deg etc.) stay as
      // plain nomenclature only; they are NOT a promise that a fixture's
      // real dimAngleDeg equals its own name.
      float dimAngleDeg;        // derived: compass angle of the ground spot
      float dimDistFt;          // derived: distance of the ground spot from car center

      float fixtureDimX, fixtureDimY, fixtureDimZ; // fixture mount position -- visualizer-only, see file header
   };

   /* ---- Per-neopixel abstracted perimeter geometry ------------
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

   static const FloodGeometry & getFlood( FixtureType type, uint8_t idx ) {
      return floods_[ globalFloodIndex_( type, idx ) ];
   }
   static const FloodGeometry & getMegabar( uint8_t m ) { return getFlood( FixtureMegabar, m ); }
   static const FloodGeometry & getChina( uint8_t c ) { return getFlood( FixtureChina, c ); }

   // Real-world CARTESIAN bounds of the X/Y axes -- car-center-relative,
   // same origin/sign convention the By{X,Y}{Ft,Pixel} getters/setters
   // use (0 = car center, never a wire's own local numbering). Ft is in
   // feet; Pixel is the SAME cartesian position rescaled to that side's
   // real pixel density (e.g. getMaxYFt(CarSideLeft)==8.0f,
   // getMaxYPixel(CarSideLeft)==176.0f -- 22px/ft * 8ft). side must still
   // be supplied (CarSideFront/Back for the X pair, CarSideLeft/Right for
   // the Y pair) since Pixel needs it for density and Ft takes it for
   // signature symmetry with Pixel, even though the Ft value itself is
   // the same for either valid side of a given axis.
   static float getMinXFt( CarSide side ) { (void)side; return -CAR_WIDTH_FT / 2.0f; }
   static float getMaxXFt( CarSide side ) { (void)side; return CAR_WIDTH_FT / 2.0f; }
   static float getMinYFt( CarSide side ) { (void)side; return -CAR_LENGTH_FT / 2.0f; }
   static float getMaxYFt( CarSide side ) { (void)side; return CAR_LENGTH_FT / 2.0f; }
   static float getMinXPixel( CarSide side ) { return getMinXFt( side ) * pixelsPerFootForSide_( side ); }
   static float getMaxXPixel( CarSide side ) { return getMaxXFt( side ) * pixelsPerFootForSide_( side ); }
   static float getMinYPixel( CarSide side ) { return getMinYFt( side ) * pixelsPerFootForSide_( side ); }
   static float getMaxYPixel( CarSide side ) { return getMaxYFt( side ) * pixelsPerFootForSide_( side ); }

   // Perimeter ("circumference") ID bounds -- unlike the cartesian bounds
   // above, NeoID is a single flat address space around the WHOLE
   // perimeter, not a per-side one, so this takes no side argument at
   // all: always 0 and NUM_NEO_LEDS_ACTUAL-1. Pairs with the pre-existing
   // getNeoByCircumferenceID().
   static int32_t getMinCircumferenceID() { return 0; }
   static int32_t getMaxCircumferenceID() { return (int32_t)NUM_NEO_LEDS_ACTUAL - 1; }

   // "Closest match": nearest fixture (of the given type) to the query
   // value in the requested dimension -- equivalent to picking the
   // decision boundary as the midpoint between each pair of adjacent
   // distinct ground-spot values, since a 1D nearest-neighbor partition
   // IS exactly that set of midpoints. Every china's real ground spot is
   // now a genuinely distinct point (see buildFloods_()'s own comment),
   // so these have no built-in tie case among china anymore -- an exact
   // tie is still theoretically possible (e.g. a hypothetical future
   // fixture placed at an identical value), resolved deterministically to
   // the lower id.
   static MegabarName getMegabarByAngle( float angleDeg ) { return (MegabarName)nearestByAngle_( FixtureMegabar, NumMegabars, angleDeg ); }
   static ChinaName   getChinaByAngle( float angleDeg )   { return (ChinaName)nearestByAngle_( FixtureChina, NumChinas, angleDeg ); }
   static MegabarName getMegabarByX( float xFt ) { return (MegabarName)nearestByX_( FixtureMegabar, NumMegabars, xFt ); }
   static ChinaName   getChinaByX( float xFt )   { return (ChinaName)nearestByX_( FixtureChina, NumChinas, xFt ); }
   static MegabarName getMegabarByY( float yFt ) { return (MegabarName)nearestByY_( FixtureMegabar, NumMegabars, yFt ); }
   static ChinaName   getChinaByY( float yFt )   { return (ChinaName)nearestByY_( FixtureChina, NumChinas, yFt ); }
   // GETTER only, no corresponding setter (see LightSetters.h) -- distance
   // from center is a legitimate thing to ASK ("what's the nearest fixture
   // to this radius"), but ambiguous to WRITE to: many fixtures share the
   // same real distance by symmetry (e.g. all 4 side-china pairs), so
   // there's no single obviously-correct fixture for a show to silently
   // set. The tie-break below (lowest id) is deterministic but arbitrary --
   // fine for a caller that inspects the result, not fine to write blind.
   static MegabarName getMegabarByDist( float distFt ) { return (MegabarName)nearestByDist_( FixtureMegabar, NumMegabars, distFt ); }
   static ChinaName   getChinaByDist( float distFt )   { return (ChinaName)nearestByDist_( FixtureChina, NumChinas, distFt ); }
   // Direct lookup by ID/enum -- included for naming symmetry with the
   // By{Angle,X,Y} family above; identical to getMegabar()/getChina(),
   // just named to match.
   static MegabarName getMegabarByID( uint8_t id ) { return (MegabarName)id; }
   static ChinaName   getChinaByID( uint8_t id )   { return (ChinaName)id; }

   static const NeoGeom & getNeoGeom( uint16_t pixelIdx ) { return neoGeom_[ pixelIdx ]; }

   /* ---- Neo circumference indexing ("NeoID") + strand abstraction ----
      NeoID 0 = the rope pixel immediately RIGHT of the front-center gap
      (the small front strand has an even pixel count, so there's no
      single center pixel -- see MagicCarpet.h's FRONT constant, which
      this is built from), increasing clockwise around the whole perimeter
      (matching the real wiring direction -- see NeoGeom's own class
      comment). Unlike a raw ropeLeds[] array index, a NeoID auto-wraps for
      ANY integer, positive or negative, in or out of [0,total) -- see
      neoIdToRawIndex()/neoIdWrap() -- so a caller can address e.g. "3
      pixels left of NeoID 0" as simply neoId=-3 with no manual modulo. */
   struct NeoStrandInfo {
      Strand strand;
      uint16_t startNeoId; // this strand's first pixel, in NeoID space (wraps -- e.g. StrandFront's own span straddles the NeoID 0 point by construction)
      uint16_t length;
   };
   static const NeoStrandInfo & getStrandInfo( Strand s ) { return strandInfo_[ s ]; }

   static uint16_t neoIdWrap( int32_t id ) {
      int32_t n = (int32_t)NUM_NEO_LEDS_ACTUAL;
      int32_t r = id % n;
      if ( r < 0 ) r += n;
      return (uint16_t)r;
   }
   static uint16_t neoIdToRawIndex( int32_t neoId ) { return neoIdWrap( neoId + (int32_t)FRONT ); }
   static int32_t rawIndexToNeoId( uint16_t rawIndex ) { return (int32_t)neoIdWrap( (int32_t)rawIndex - (int32_t)FRONT ); }

   // "Closest match" neo getters -- same nearest-match semantics as the
   // flood By{Angle,X,Y} getters above, returning a NeoID. Take a CarSide
   // (which of the 4 sides of the CAR), never a Strand (which wire it's
   // on) -- see CarSide's own comment above. X only varies meaningfully on
   // the Front/Back side and Y only on the Left/Right side (a side's "off
   // axis" coordinate barely moves along its own length), so only those
   // pairings are provided -- see LightSetters.h, which is where the
   // actual (Side,X)/(Side,Y) ambiguity this resolves is explained in
   // full. XPixel/YPixel take the SAME cartesian, car-center-relative
   // position as XFt/YFt, just rescaled to that side's real pixel density
   // (see getMax{X,Y}Pixel above) -- NOT a side's own wire-order/local
   // pixel numbering. That's a different concept, with "Id" in the name:
   // getNeoByXPixelId/getNeoByYPixelId below, an O(1) direct lookup (no
   // nearest-match search needed -- given a side and a wire-local index,
   // the NeoID is exactly known), for callers that already have a raw
   // pixel-index (e.g. a scrolling animation's own array offset).
   static int32_t getNeoByAngleDeg( float angleDeg ) { return rawIndexToNeoId( (uint16_t)nearestNeoGlobalByAngle_( angleDeg ) ); }
   static int32_t getNeoByXFt( CarSide side, float xFt ) { return nearestNeoOnStrand_( strandForSide_( side ), xFt, /*useX*/true ); }
   static int32_t getNeoByYFt( CarSide side, float yFt ) { return nearestNeoOnStrand_( strandForSide_( side ), yFt, /*useX*/false ); }
   static int32_t getNeoByXPixel( CarSide side, float xPixel ) { return getNeoByXFt( side, xPixel / pixelsPerFootForSide_( side ) ); }
   static int32_t getNeoByYPixel( CarSide side, float yPixel ) { return getNeoByYFt( side, yPixel / pixelsPerFootForSide_( side ) ); }
   static int32_t getNeoByCircumferenceID( int32_t neoId ) { return neoIdWrap( neoId ); } // trivial -- included for naming symmetry, see getMegabarByID/getChinaByID
   // Wire-order/local-index lookup, O(1) -- NOT cartesian, see above.
   static int32_t getNeoByXPixelId( CarSide side, int32_t localIndex ) { return neoIdWrap( (int32_t)strandInfo_[ strandForSide_( side ) ].startNeoId + localIndex ); }
   static int32_t getNeoByYPixelId( CarSide side, int32_t localIndex ) { return neoIdWrap( (int32_t)strandInfo_[ strandForSide_( side ) ].startNeoId + localIndex ); }
   // How many pixels are wired on a given side -- for a show that wants
   // to sweep every pixel on one side (e.g. `for (local = 0; local <
   // getSidePixelCount(side); ++local) neoId = getNeoByYPixelId(side,local)`),
   // without ever needing to know the underlying Strand.
   static uint16_t getSidePixelCount( CarSide side ) { return strandInfo_[ strandForSide_( side ) ].length; }

   // The ONE place CarSide<->Strand coupling is assumed (see both enums'
   // own comments) -- everything above this line in the public API deals
   // only in CarSide; everything below (NeoStrandInfo, getStrandInfo,
   // buildStrandInfo_) is internal wiring-order plumbing.
   static Strand strandForSide_( CarSide side ) { return (Strand)side; }
   // Real pixel density (LEDs/foot) of the strand behind a given side --
   // Front/Back share one density (SIZEOF_SMALL_NEO over CAR_WIDTH_FT),
   // Left/Right share the other (SIZEOF_LARGE_NEO over CAR_LENGTH_FT).
   static float pixelsPerFootForSide_( CarSide side ) {
      Strand s = strandForSide_( side );
      float axisLenFt = ( s == StrandFront || s == StrandBack ) ? CAR_WIDTH_FT : CAR_LENGTH_FT;
      return (float)getNumPixelsOnStrand( s ) / axisLenFt;
   }

   // Call once from setup(), before any light show runs.
   static void begin() {
      buildFloods_();
      buildNeoGeom_();
      buildStrandInfo_();
   }

 private:
   static const uint8_t NUM_FLOODS_ = NUM_MEGABAR_LEDS + NUM_CHINA_LEDS; // 20
   static FloodGeometry floods_[ NUM_FLOODS_ ];
   static NeoGeom neoGeom_[ NUM_NEO_LEDS_ACTUAL ];
   static NeoStrandInfo strandInfo_[ 4 ];

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
   // shortest unsigned angular distance between 2 compass angles, 0..180
   static float circularAbsDelta_( float aDeg, float bDeg ) {
      float d = fmodf( fabsf( aDeg - bDeg ), 360.0f );
      return ( d > 180.0f ) ? 360.0f - d : d;
   }

   static uint8_t nearestByAngle_( FixtureType type, uint8_t count, float queryDeg ) {
      uint8_t best = 0; float bestDelta = 1e9f;
      for ( uint8_t i = 0; i < count; ++i ) {
         float d = circularAbsDelta_( getFlood( type, i ).dimAngleDeg, queryDeg );
         if ( d < bestDelta ) { bestDelta = d; best = i; }
      }
      return best;
   }
   static uint8_t nearestByX_( FixtureType type, uint8_t count, float queryXFt ) {
      uint8_t best = 0; float bestDelta = 1e9f;
      for ( uint8_t i = 0; i < count; ++i ) {
         float d = fabsf( getFlood( type, i ).dimX - queryXFt );
         if ( d < bestDelta ) { bestDelta = d; best = i; }
      }
      return best;
   }
   static uint8_t nearestByY_( FixtureType type, uint8_t count, float queryYFt ) {
      uint8_t best = 0; float bestDelta = 1e9f;
      for ( uint8_t i = 0; i < count; ++i ) {
         float d = fabsf( getFlood( type, i ).dimY - queryYFt );
         if ( d < bestDelta ) { bestDelta = d; best = i; }
      }
      return best;
   }
   static uint8_t nearestByDist_( FixtureType type, uint8_t count, float queryFt ) {
      uint8_t best = 0; float bestDelta = 1e9f;
      for ( uint8_t i = 0; i < count; ++i ) {
         float d = fabsf( getFlood( type, i ).dimDistFt - queryFt );
         if ( d < bestDelta ) { bestDelta = d; best = i; }
      }
      return best;
   }

   // nearest neopixel (raw array index, any strand) to a compass angle
   static uint16_t nearestNeoGlobalByAngle_( float queryDeg ) {
      uint16_t best = 0; float bestDelta = 1e9f;
      for ( uint16_t raw = 0; raw < NUM_NEO_LEDS_ACTUAL; ++raw ) {
         float d = circularAbsDelta_( neoGeom_[ raw ].angleFromForwardDeg, queryDeg );
         if ( d < bestDelta ) { bestDelta = d; best = raw; }
      }
      return best;
   }
   // nearest neopixel WITHIN one strand, by real cartesian X or Y (feet)
   // -- backs getNeoBy{X,Y}Ft() above (getNeoBy{X,Y}Pixel() delegates to
   // those after rescaling by pixelsPerFootForSide_(), so this is the one
   // real implementation for both units).
   static int32_t nearestNeoOnStrand_( Strand strand, float queryFt, bool useX ) {
      const NeoStrandInfo & info = strandInfo_[ strand ];
      int32_t best = (int32_t)info.startNeoId; float bestDelta = 1e9f;
      for ( uint16_t local = 0; local < info.length; ++local ) {
         int32_t neoId = (int32_t)info.startNeoId + local;
         uint16_t raw = neoIdToRawIndex( neoId );
         const NeoGeom & g = neoGeom_[ raw ];
         float fieldVal = useX ? ( g.xPercent / 100.0f * CAR_WIDTH_FT ) : ( g.yPercent / 100.0f * CAR_LENGTH_FT );
         float d = fabsf( fieldVal - queryFt );
         if ( d < bestDelta ) { bestDelta = d; best = neoId; }
      }
      return neoIdWrap( best );
   }

   // Megabar (x,y), ft, Y+=front -- ported from the visualizer's
   // MEGABAR_POS_FT (see its own long comment there for how these 12
   // positions were hand-tuned); that table uses Y+=BACK, so signs are
   // flipped here to match this file's Y+=front convention.
   static void buildFloods_() {
      static const float halfW = CAR_WIDTH_FT / 2.0f, halfH = CAR_LENGTH_FT / 2.0f; // 6, 8
      static const float STD_INSET_FT = 2.5f;
      static const float FRONT_INSET_FT = 2.0f, BACK_INSET_FT = 1.5f;
      // TEMPORARY (per request, "try this"): 0/90/180/270deg (indices 0/3/
      // 6/9) unchanged. The other 8 (30/60/120/150/210/240/300/330deg)
      // recomputed from a new algorithm, precomputed offline (no live trig
      // added to the FW) -- the fixture stays at its real, hand-tuned mount
      // position (nearX/centerX+sideEdgeY, same values as before); the
      // ground spot is that same fixture position PLUS 8ft "as the crow
      // flies" continuing outward in the fixture's own nominal compass
      // direction (0deg=car-forward, clockwise -- e.g. the 30deg fixture's
      // spot is 8ft further out along the 30deg ray, not along whatever
      // angle its real (slightly-off-30) position happens to sit at).
      // Since the real fixture isn't exactly ON its own nominal-angle ray
      // from center, the resulting spot's angle-from-center lands CLOSE to
      // but not exactly on the nominal value (e.g. 30deg fixture's spot is
      // at 28.68deg from center) -- same "close, hand-tuned, not a clean
      // grid" character the original table always had.
      // Real, hand-tuned fixture MOUNT positions -- UNCHANGED from the
      // original table (mirror-corrected only). NOT moved by the spot
      // recalculation below.
      const float sideEdgeY = halfW / tanf( 60.0f * PI / 180.0f ); // ~3.46ft
      const float centerX = halfW - STD_INSET_FT;                  // 3.5ft
      const float nearX = centerX + 0.3f - 1.0f;                   // 2.8ft (FRONT_NEAR_GAP_FT - CORNER_PULL_FT)
      const float mbFixtureXY[ NUM_MEGABAR_LEDS ][ 2 ] = {
         {         0.0f,  ( halfH - FRONT_INSET_FT ) }, // 0
         {       nearX,   ( halfH - STD_INSET_FT ) },   // 1 (30deg)
         {     centerX,    sideEdgeY },                  // 2 (60deg)
         {     centerX,    0.0f },                        // 3 (90deg)
         {     centerX,   -sideEdgeY },                  // 4 (120deg)
         {       nearX,  -( halfH - STD_INSET_FT ) },   // 5 (150deg)
         {         0.0f, -( halfH - BACK_INSET_FT ) }, // 6
         {      -nearX,  -( halfH - STD_INSET_FT ) },   // 7 (210deg)
         {    -centerX,   -sideEdgeY },                  // 8 (240deg)
         {    -centerX,    0.0f },                        // 9 (270deg)
         {    -centerX,    sideEdgeY },                  // 10 (300deg)
         {      -nearX,   ( halfH - STD_INSET_FT ) },   // 11 (330deg)
      };
      // Z heights from the visualizer's own FIXTURE_HEIGHTS_FT (megabarFront:
      // 1.0, megabarDefault: 1.5, china: 2.0) -- the only real measured mount
      // heights that exist anywhere in this codebase so far.
      static const float MEGABAR_FRONT_Z_FT = 1.0f, MEGABAR_DEFAULT_Z_FT = 1.5f;
      // Ground spot -- dimX/dimY -- fully independent hardcoded values, same
      // as fixtureDimX/Y above, NOT derived from the fixture at build or run
      // time (per request: every flood's fixture and spot are both
      // independent hardcoded locations, china included). 0/90/180/270deg
      // (indices 0/3/6/9) equal their fixture position, unchanged. All 8
      // others were precomputed OFFLINE (not in this file): fixture position
      // plus 8ft "as the crow flies", continuing outward in a throw
      // direction rotated from that fixture's own nominal compass angle --
      // 60/120/240/300deg rotated 5deg TOWARD their nearest orthogonal
      // neighbor (90/270deg, confirmed good, measured); 30/150/210/330deg
      // now at their plain nominal throw direction (0deg offset) -- 3deg,
      // then 1.5deg, then 1.0deg, then 0.5deg AWAY from 0/180deg were all
      // stepped down in turn as each measured too much, landing back at
      // nominal. Both are visual-equalization trial amounts (per request),
      // meant to be measured and recalibrated, not precisely derived.
      const float mbSpotXY[ NUM_MEGABAR_LEDS ][ 2 ] = {
         {         0.0f,    6.0f },     // 0
         {         6.8000f,  12.4282f }, // 1 (30deg, throw dir 30.0deg, nominal)
         {        10.7505f,   6.8450f }, // 2 (60deg, throw dir 65deg)
         {         3.5f,     0.0f },     // 3 (90deg)
         {        10.7505f,  -6.8450f }, // 4 (120deg, throw dir 115deg)
         {         6.8000f, -12.4282f }, // 5 (150deg, throw dir 150.0deg, nominal)
         {         0.0f,   -6.5f },     // 6
         {        -6.8000f, -12.4282f }, // 7 (210deg, throw dir 210.0deg, nominal)
         {       -10.7505f,  -6.8450f }, // 8 (240deg, throw dir 245deg)
         {        -3.5f,     0.0f },     // 9
         {       -10.7505f,   6.8450f }, // 10 (300deg, throw dir 295deg)
         {        -6.8000f,  12.4282f }, // 11 (330deg, throw dir 330.0deg, nominal)
      };
      for ( uint8_t m = 0; m < NUM_MEGABAR_LEDS; ++m ) {
         FloodGeometry & f = floods_[ globalFloodIndex_( FixtureMegabar, m ) ];
         f.id = m; f.type = FixtureMegabar;
         f.dmxAddress = (uint8_t)( 1 + 3 * m ); // see README.md, "Megabars"
         f.fixtureDimX = mbFixtureXY[ m ][ 0 ]; f.fixtureDimY = mbFixtureXY[ m ][ 1 ];
         f.fixtureDimZ = ( m == 0 ) ? MEGABAR_FRONT_Z_FT : MEGABAR_DEFAULT_Z_FT; // m0 = headlight, the front-facing pair
         f.dimX = mbSpotXY[ m ][ 0 ]; f.dimY = mbSpotXY[ m ][ 1 ];
         // angle/dist of the straight car-center-to-spot segment -- see
         // dimAngleDeg/dimDistFt's own comment on the FloodGeometry struct.
         f.dimAngleDeg = angleFromForward_( f.dimX, f.dimY );
         f.dimDistFt = sqrtf( f.dimX * f.dimX + f.dimY * f.dimY );
      }

      // China: 2 fixtures per corner (front-right/front-left/back-left/
      // back-right), one aimed along the near front/back edge, one aimed
      // along the near side edge. BUGFIX (a real error, not a measurement
      // question -- confirmed against README.md's own china table, which
      // this session had stopped cross-checking against): ground spots
      // (dimX/dimY) were modeled as CO-LOCATED within each pair. They are
      // NOT -- each china's real bright spot is "~1/3 of the way along
      // its assigned edge, measured from its own corner toward the far
      // corner" (README.md, "China lights"), and the 2 fixtures in a pair
      // are aimed along DIFFERENT edges (one front/back, one side), so
      // their real ground spots are two genuinely different points, well
      // separated in angle (confirmed empirically: front-right's front-
      // aimed fixture sits ~14deg from center, its side-aimed corner-
      // neighbor ~66deg -- nowhere close to co-located, and also nowhere
      // close to a naive 45deg-apart guess). This is the value every
      // real light show actually reads (e.g. LighthouseShow.h's per-
      // fixture beam-crossing effect) -- getting it wrong here was a real
      // bug, not a cosmetic one. fixtureDimX/Y below is a completely
      // separate, much smaller-scale concept (the ~0.26ft physical mount
      // offset between 2 fixtures sharing one corner bracket) -- unrelated
      // to this several-foot, aim-driven ground-spot separation.
      static const float CHINA_Z_FT = 2.0f;
      // "~1/3 of the way along its assigned edge" -- edge full length, not
      // half (a front/back-aimed spot moves along the CAR_WIDTH_FT-long
      // front/back edge; a side-aimed spot moves along the
      // CAR_LENGTH_FT-long side edge), measured from the TRUE car corner.
      static const float WIDTH_EDGE_OFFSET_FT = CAR_WIDTH_FT / 3.0f;   // ~4.0ft, front/back-aimed spots move this far in X
      static const float LENGTH_EDGE_OFFSET_FT = CAR_LENGTH_FT / 3.0f; // ~5.33ft, side-aimed spots move this far in Y
      const float chDimXY[ NUM_CHINA_LEDS ][ 2 ] = {
         {  halfW,                    halfH - LENGTH_EDGE_OFFSET_FT }, // 0 front-right, aimed right/side edge -- ~1/3 back from front
         {  halfW - WIDTH_EDGE_OFFSET_FT,  halfH },                     // 1 front-right, aimed front edge     -- ~1/3 in from right
         { -halfW + WIDTH_EDGE_OFFSET_FT,  halfH },                     // 2 front-left, aimed front edge      -- ~1/3 in from left
         { -halfW,                    halfH - LENGTH_EDGE_OFFSET_FT }, // 3 front-left, aimed left/side edge  -- ~1/3 back from front
         { -halfW,                   -halfH + LENGTH_EDGE_OFFSET_FT }, // 4 back-left, aimed left/side edge   -- ~1/3 forward from back
         { -halfW + WIDTH_EDGE_OFFSET_FT, -halfH },                     // 5 back-left, aimed back edge        -- ~1/3 in from left
         {  halfW - WIDTH_EDGE_OFFSET_FT, -halfH },                     // 6 back-right, aimed back edge       -- ~1/3 in from right
         {  halfW,                   -halfH + LENGTH_EDGE_OFFSET_FT }, // 7 back-right, aimed right/side edge -- ~1/3 forward from back
      };
      // Each fixture's own real mount position, hand-entered individually
      // (no shared formula/offset applied at build time -- these ARE the
      // measured/estimated values, not derived from cx/cy at runtime).
      const float chMountXY[ NUM_CHINA_LEDS ][ 2 ] = {
         {  5.012f,  6.750f },   // 0 front-right, aimed side
         {  4.750f,  7.012f },   // 1 front-right, aimed front
         { -4.750f,  7.012f },   // 2 front-left, aimed front
         { -5.012f,  6.750f },   // 3 front-left, aimed side
         { -5.012f, -6.750f },   // 4 back-left, aimed side
         { -4.750f, -7.012f },   // 5 back-left, aimed back
         {  4.750f, -7.012f },   // 6 back-right, aimed back
         {  5.012f, -6.750f },   // 7 back-right, aimed side
      };
      for ( uint8_t c = 0; c < NUM_CHINA_LEDS; ++c ) {
         FloodGeometry & f = floods_[ globalFloodIndex_( FixtureChina, c ) ];
         f.id = c; f.type = FixtureChina;
         f.dmxAddress = (uint8_t)( 37 + 6 * c ); // see README.md, "China lights"
         f.dimX = chDimXY[ c ][ 0 ]; f.dimY = chDimXY[ c ][ 1 ];
         // angle/dist of the straight car-center-to-spot segment -- same
         // derivation as megabar's above, see dimAngleDeg/dimDistFt's own
         // comment on the FloodGeometry struct.
         f.dimAngleDeg = angleFromForward_( f.dimX, f.dimY );
         f.dimDistFt = sqrtf( f.dimX * f.dimX + f.dimY * f.dimY );
         f.fixtureDimX = chMountXY[ c ][ 0 ]; f.fixtureDimY = chMountXY[ c ][ 1 ];
         f.fixtureDimZ = CHINA_Z_FT;
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

   // Strand boundaries derived from the same real offsets MagicCarpet.h's
   // own hardware-output path uses (NEO0..3_OFFSET) -- one real source,
   // just re-expressed in NeoID space instead of raw array index.
   static void buildStrandInfo_() {
      strandInfo_[ StrandFront ] = { StrandFront, neoIdWrap( (int32_t)NEO0_OFFSET - (int32_t)FRONT ), getNumPixelsOnStrand( StrandFront ) };
      strandInfo_[ StrandRight ] = { StrandRight, neoIdWrap( (int32_t)NEO1_OFFSET - (int32_t)FRONT ), getNumPixelsOnStrand( StrandRight ) };
      strandInfo_[ StrandBack ]  = { StrandBack,  neoIdWrap( (int32_t)NEO2_OFFSET - (int32_t)FRONT ), getNumPixelsOnStrand( StrandBack ) };
      strandInfo_[ StrandLeft ]  = { StrandLeft,  neoIdWrap( (int32_t)NEO3_OFFSET - (int32_t)FRONT ), getNumPixelsOnStrand( StrandLeft ) };
   }
};

CarpetGeometry::FloodGeometry CarpetGeometry::floods_[ CarpetGeometry::NUM_FLOODS_ ];
CarpetGeometry::NeoGeom CarpetGeometry::neoGeom_[ NUM_NEO_LEDS_ACTUAL ];
CarpetGeometry::NeoStrandInfo CarpetGeometry::strandInfo_[ 4 ];

#endif
