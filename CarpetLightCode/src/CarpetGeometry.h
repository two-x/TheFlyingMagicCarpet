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
 *    180=rear, 270=left -- same convention LighthouseShow.h/
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
 *        front/rear, one side) -- their ground spots are two genuinely
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
   enum Strand { StrandFront = 0, StrandRight = 1, StrandRear = 2, StrandLeft = 3 };
   // CarSide = which of the 4 sides of the CAR -- the geometric concept
   // every neo position getter/setter below actually takes. Distinct from
   // Strand (hardware wiring, see above) and from the (unrelated,
   // pre-existing) binary Side enum below (which classifies a pixel as
   // simply left-half/right-half of the car, regardless of which of the 4
   // sides or strands it's on).
   enum CarSide { CarSideFront = 0, CarSideRight = 1, CarSideRear = 2, CarSideLeft = 3 };
   enum Side { SideRight = 0, SideLeft = 1 };

   // Named indices, in DMX address order (see README.md, "Megabars"/"China
   // lights") -- callers should prefer these over raw integer indices
   // wherever the specific fixture matters, e.g. CarpetGeometry::getChina(
   // CarpetGeometry::ChinaFrontRight) rather than getChina(1).
   enum MegabarName {
      Megabar0deg = 0, Megabar30deg, Megabar60deg, Megabar90deg, Megabar120deg, Megabar150deg,
      Megabar180deg, Megabar210deg, Megabar240deg, Megabar270deg, Megabar300deg, Megabar330deg,
      NumMegabars
   };
   enum ChinaName {
      ChinaRightFront = 0,  // addr 37 -- front-right corner, aimed along the right (side) edge
      ChinaFrontRight,      // addr 43 -- front-right corner, aimed along the front edge
      ChinaFrontLeft,       // addr 49 -- front-left corner, aimed along the front edge
      ChinaLeftFront,       // addr 55 -- front-left corner, aimed along the left (side) edge
      ChinaLeftRear,        // addr 61 -- rear-left corner, aimed along the left (side) edge
      ChinaRearLeft,        // addr 67 -- rear-left corner, aimed along the rear edge
      ChinaRearRight,       // addr 73 -- rear-right corner, aimed along the rear edge
      ChinaRightRear,       // addr 79 -- rear-right corner, aimed along the right (side) edge
      NumChinas
   };

   /* ---- Vehicle 3D model ---- */
   static constexpr float CAR_WIDTH_FT = 12.0f;  // left-right
   static constexpr float CAR_LENGTH_FT = 16.0f; // front-rear
   // Real measured fringe/tassel trim length, hanging down off the car's
   // edge -- a Z-axis (vertical) dimension, hence the _Z_ in the name.
   static constexpr float FRINGE_LENGTH_Z_FT = 1.0f;
   // Real measured carpet deck surface height off the ground -- was 2.5ft,
   // corrected down 3in in one pass then another 3in in a follow-up (6in
   // total, "lower everything toward the ground by 3in" applied twice).
   // Everything else that positions relative to the deck (table, chassis
   // top, tassel attachment, etc.) references this constant rather than
   // an independent copy, so the correction propagates automatically --
   // EXCEPT whatever is deliberately anchored to true ground instead
   // (TASSEL_CONE_BOTTOM_ABOVE_GROUND_FT, CHASSIS_BOTTOM_HEIGHT_FT, wheel
   // height), which stays put by construction since those don't reference
   // this constant at all.
   static constexpr float SURFACE_BASE_HEIGHT_FT = 2.5f - 3.0f / 12.0f - 3.0f / 12.0f;
   // Real measured deck panel thickness -- the visible "edge" between the
   // deck's top (SURFACE_BASE_HEIGHT_FT) and bottom (CHASSIS_TOP_HEIGHT_FT
   // below, which is defined to equal this same underside) faces.
   static constexpr float SURFACE_THICKNESS_FT = 0.5f / 12.0f;
   // Real measured thickness of the carpet/rug fabric itself, a SEPARATE
   // physical layer sitting ON TOP of the rigid deck panel above
   // (SURFACE_BASE_HEIGHT_FT is the deck panel's own top face -- the
   // carpet's own top surface is SURFACE_BASE_HEIGHT_FT +
   // CARPET_THICKNESS_FT, not the same surface).
   static constexpr float CARPET_THICKNESS_FT = 0.5f / 12.0f;
   // Real measured property: the rope along the LEFT/RIGHT side strands
   // physically undulates up and down (peak-to-peak 3.5in) as you move
   // front-to-rear -- the front/rear strands do NOT undulate (flat, always
   // exactly SURFACE_BASE_HEIGHT_FT). Wavelength set to 1 full cycle
   // across the car's own length (not independently measured).
   static constexpr float SIDE_UNDULATION_PEAK_TO_PEAK_FT = 3.5f / 12.0f;
   static constexpr float SIDE_UNDULATION_AMPLITUDE_FT = SIDE_UNDULATION_PEAK_TO_PEAK_FT / 2.0f;
   // CORRECTED per follow-up request: front/rear edges must be minima
   // AND there must be 2 more minima in between (4 minima total: both
   // edges + 2 interior), with 3 maxima total (center plus one on each
   // side) -- 7 extrema alternating min/max/min/max/min/max/min across
   // the car's length, which needs k*halfL = 3*PI (not PI), i.e. exactly
   // 3 full wavelengths span the car's length (not 1) -- paired with
   // getSurfaceHeightAt()'s own cosf() (not sinf()) below, so Y=0 is a
   // peak, Y=+-halfL are the outermost troughs, and Y=+-CAR_LENGTH_FT/6
   // are the 2 additional interior troughs.
   static constexpr float SIDE_UNDULATION_WAVELENGTH_FT = CAR_LENGTH_FT / 3.0f;

   // Round decoration table + tablecloth on the deck -- NOT a light
   // fixture (no dimZ/mount concept, visualizer-rendering-only, same
   // category as beamYawDeg/beamPitchDeg in the file header's own note).
   // Moved here from the visualizer's own hardcoded literals so it has
   // exactly one source of truth like every other real-world measurement
   // in this file. Centered left-right (X=0), offset forward of car
   // center (Y+).
   static constexpr float TABLE_CENTER_X_FT = 0.0f;
   static constexpr float TABLE_CENTER_Y_FT = 2.5f;
   static constexpr float TABLE_DIAMETER_FT = 3.0f;
   // Table top height above the LOCAL real deck surface at the table's own
   // position (i.e. add to getSurfaceHeightAt(..., TABLE_CENTER_Y_FT), not
   // to SURFACE_BASE_HEIGHT_FT directly -- undulation, where it occurs,
   // already reduces this). The table itself is rigid/level (parallel to
   // the true ground), not something that warps with the deck.
   static constexpr float TABLE_HEIGHT_ABOVE_SURFACE_FT = 7.0f / 12.0f; // was 10.5in, shortened by 1/3 per request
   // Small dark-red tassels hanging from each of the top tablecloth's 4
   // corners (short enough that none reach the carpet surface below --
   // real headroom is TABLE_HEIGHT_ABOVE_SURFACE_FT, comfortably more
   // than this). Real perimeter fringe around the top tablecloth's own
   // square edge -- hangs straight down (per explicit request, does NOT
   // enlarge the cloth's own footprint), same reduced-representative-
   // count rendering approach as the main perimeter fringe.
   static constexpr float TABLECLOTH_TASSEL_HEIGHT_FT = 2.0f / 12.0f;
   static constexpr float TABLECLOTH_FRINGE_DROP_FT = 2.0f / 12.0f;
   static constexpr float TABLECLOTH_FRINGE_THICKNESS_FT = 0.125f / 12.0f;

   // Corner tassels: real per-part dimensions (see CarpetGeometry.h
   // consumers for the assembly order -- sphere, then a wrapping cylinder,
   // then a flaring cone, top to bottom). Centered at each true deck
   // corner (X,Y); Z anchors given per-part below since they chain
   // (sphere anchored off the local deck surface, cone anchored off the
   // true ground -- 2 independent real reference heights, not one).
   static constexpr float TASSEL_SPHERE_DIAMETER_FT = 5.0f / 12.0f; // was 4in, grown 25% per request (applies to corner AND side tassels, both scaled from this same real spec)
   static constexpr float TASSEL_SPHERE_BELOW_SURFACE_FT = 2.0f / 12.0f; // sphere center, below the LOCAL real deck surface at that corner -- was 1in, whole assembly lowered 1in per request (paired with TASSEL_CONE_BOTTOM_ABOVE_GROUND_FT below)
   static constexpr float TASSEL_CYLINDER_DIAMETER_FT = 3.5f / 12.0f;
   static constexpr float TASSEL_CYLINDER_HEIGHT_FT = 0.5f / 12.0f; // was 1.5in, shortened 1in so the corner tassel's total height is 1in shorter overall, per request
   static constexpr float TASSEL_CONE_TOP_DIAMETER_FT = 3.0f / 12.0f;
   static constexpr float TASSEL_CONE_BOTTOM_DIAMETER_FT = 8.0f / 12.0f;
   static constexpr float TASSEL_CONE_BOTTOM_ABOVE_GROUND_FT = 3.0f / 12.0f; // absolute height above true ground -- not relative to the sphere/cylinder above it -- was 4in, whole assembly lowered 1in per request

   // Perimeter fringe: real strand spec (visualizer renders a reduced
   // representative count for performance/legibility, not this literal
   // density -- see the visualizer's own comment on that simplification;
   // the real facts belong here regardless of how they get rendered).
   static constexpr float FRINGE_STRANDS_PER_IN = 5.0f;
   static constexpr float FRINGE_STRAND_DIAMETER_FT = 0.1875f / 12.0f; // was ~1/8in (0.15in), increased to 3/16in per request

   // Megabar0 ("the front-facing pair," see MegabarName/buildFloods_'s own
   // comments) is really 2 physical fixture housings mounted directly next
   // to each other, both driven by the same logical LED/DMX slot and both
   // aimed at the SAME real ground spot -- per explicit request, a
   // visualizer-3D-rendering-only fact (which real light show cares how
   // many physical housings share one logical channel), so it lives here
   // alongside the other Visualizer-3D-rendering-only mount facts rather
   // than changing NUM_MEGABAR_LEDS/the fixture table's own shape.
   // Estimated ("mounted directly next to each other," not independently
   // measured) -- flagged the same way SIDE_TASSEL_ROPE_LENGTH_FT's own
   // assumed value is.
   static constexpr float MEGABAR_FRONT_PAIR_SPACING_FT = 8.0f / 12.0f;

   // Vehicle undercarriage (chassis) -- NOT rendered anywhere yet (the
   // visualizer is 2D top-down only, no 3D mode exists), included so a
   // future 3D pass has real dimensions to build from instead of guessing.
   // Visualizer-rendering-only, same "no real light show reads this"
   // category as the table/fringe geometry above. A plain 4x8ft
   // rectangular prism, centered on car center (matches the car's own X=0
   // convention), running from the ground up to the bottom of the deck
   // surface -- rendered (per request) as a dull dark grey block, no color
   // constant here since that's a rendering choice, not a dimension.
   static constexpr float CHASSIS_WIDTH_FT = 4.0f;    // X extent
   static constexpr float CHASSIS_LENGTH_FT = 8.0f;   // Y extent
   static constexpr float CHASSIS_CENTER_X_FT = 0.0f; // centered left-right on car center
   static constexpr float CHASSIS_CENTER_Y_FT = 0.0f; // centered front-rear on car center
   static constexpr float CHASSIS_BOTTOM_HEIGHT_FT = 4.0f / 12.0f;    // 4in ground clearance
   // Meets the underside of the deck panel -- the panel's own underside
   // undulates along with its top (same amplitude, see
   // SIDE_UNDULATION_AMPLITUDE_FT) and sits SURFACE_THICKNESS_FT below the
   // top, so a flat chassis top must be set to the LOWEST point that
   // underside ever reaches (not SURFACE_BASE_HEIGHT_FT itself), or the
   // chassis visibly pokes through the deck wherever undulation dips low.
   static constexpr float CHASSIS_TOP_HEIGHT_FT = SURFACE_BASE_HEIGHT_FT - SIDE_UNDULATION_AMPLITUDE_FT - SURFACE_THICKNESS_FT;

   // Wheels: one at each of the 4 extreme corners of the chassis footprint
   // above (X=+-CHASSIS_WIDTH_FT/2, Y=+-CHASSIS_LENGTH_FT/2), resting on
   // the ground (center height = WHEEL_DIAMETER_FT/2). Diameter is the
   // wheel's own rolling-direction/height extent (Y-Z plane); width is its
   // axle-direction extent (X, side-to-side thickness).
   static constexpr float WHEEL_DIAMETER_FT = 16.5f / 12.0f;
   static constexpr float WHEEL_WIDTH_FT = 8.5f / 12.0f;
   // Fender cutout carved out of the chassis prism around each wheel --
   // gap between the wheel's own footprint and the chassis body panel, on
   // whichever sides actually face solid chassis material. Was a
   // "generous" 6in placeholder; now a precise 4in per request.
   static constexpr float FENDER_CLEARANCE_FT = 4.0f / 12.0f;

   // Surface height (feet above the ground) at a given front-rear position
   // (yFt) on the given side of the car. Only CarSideLeft/CarSideRight
   // undulate (a real physical property of the rope mount along those 2
   // strands); CarSideFront/CarSideRear are flat, always exactly
   // SURFACE_BASE_HEIGHT_FT, regardless of yFt.
   static float getSurfaceHeightAt( CarSide side, float yFt ) {
      if ( side != CarSideLeft && side != CarSideRight ) return SURFACE_BASE_HEIGHT_FT;
      return SURFACE_BASE_HEIGHT_FT + SIDE_UNDULATION_AMPLITUDE_FT *
             cosf( 2.0f * PI * yFt / SIDE_UNDULATION_WAVELENGTH_FT );
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
      front strand runs left->right, right side front->rear, rear strand
      right->left, left side rear->front -- see README.md "Perimeter rope
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
      return ( s == StrandFront || s == StrandRear ) ? SIZEOF_SMALL_NEO : SIZEOF_LARGE_NEO;
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
   // be supplied (CarSideFront/Rear for the X pair, CarSideLeft/Right for
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
   // BUGFIX, revised: ByX/ByY used to silently return a single "winner"
   // even when the query was a genuine tie -- fixtures repeat X across the
   // front/rear mirror line and repeat Y across the left/right mirror line
   // (e.g. Megabar30deg and Megabar150deg share the same dimX; Megabar30deg
   // and Megabar330deg share the same dimY), so "nearest fixture to
   // X=6.8ft" had a real 2-way tie between fixtures on opposite sides of
   // the car -- whichever the loop happened to see first silently won.
   //
   // side (CarSide) is now REQUIRED -- per explicit request, reversing an
   // earlier version of this fix that deliberately omitted it (a side
   // argument was judged to risk misrouting a query that wasn't actually
   // ambiguous). With side required, only the fixtures on that side are
   // ever candidates: for ByX, side must be CarSideFront/CarSideRear,
   // filtering on each fixture's OWN dimY sign (>=0 counts as Front, <=0
   // as Rear -- inclusive both ways, so a fixture sitting exactly on the
   // line, e.g. Megabar90deg/Megabar270deg at dimY==0, is reachable from
   // either side rather than falling into a dead zone); ByY mirrors this
   // on dimX/CarSideLeft/CarSideRight. This removes the mirror-line tie
   // entirely for every query that supplies the correct side.
   //
   // Tie-detection STILL runs, defensively, WITHIN whichever side was
   // requested -- not removed just because a side is now mandatory. The
   // real risk a side argument can't rule out is a FUTURE geometry change
   // (fixture repositioning) introducing a same-side collision no one
   // intended; if that ever happens, this still refuses to guess rather
   // than silently pick one. Confirmed, by checking every current fixture
   // position by hand: no such same-side tie exists today, for either
   // fixture type, on either axis, even counting the two on-the-line
   // megabars in both their buckets. CPU cost of the check itself is
   // negligible -- same O(count) linear scan as before (<=12 fixtures),
   // with one extra float comparison per fixture for the side filter.
   //
   // Two distinct error conditions, BOTH logged (rate-limited, see
   // nospam_printf in Utilities.h, naming the getter/side/value and which
   // light show triggered it) before returning the out-of-range sentinel
   // (MegabarName)NumMegabars / (ChinaName)NumChinas -- one past the last
   // real index, mirrors getNeoBy{X,Y}Ft's own INT32_MIN sentinel below:
   //   - a genuine tie (2+ fixtures on the requested side equally close)
   //   - an invalid side/axis pairing (e.g. CarSideLeft passed to ByX) --
   //     unlike getNeoByXFt/getNeoByYFt's own SILENT sentinel return for
   //     this same case, this one logs -- a real caller-code bug, worth
   //     surfacing on the console rather than just failing quietly.
   // Callers that pass the result through directly to an array index must
   // check for the sentinel first; LightSetters (which builds its setters
   // on top of these) already does.
   //
   // Efficiency: a full linear scan (12 megabars / 8 china) on every call
   // is trivial on this MCU and happens at most once per setColor/setWhite
   // call, never per-pixel -- no lookup table is needed. If some future
   // caller pattern made this a real hot path, a tie-lookup table could be
   // precomputed once in begin() instead; not done here since nothing
   // currently justifies the added complexity.
   static MegabarName getMegabarByX( CarSide side, float xFt ) { uint8_t id = nearestXChecked_( FixtureMegabar, NumMegabars, side, xFt ); return (id == TIE_SENTINEL_) ? (MegabarName)NumMegabars : (MegabarName)id; }
   static ChinaName   getChinaByX( CarSide side, float xFt )   { uint8_t id = nearestXChecked_( FixtureChina, NumChinas, side, xFt );     return (id == TIE_SENTINEL_) ? (ChinaName)NumChinas     : (ChinaName)id; }
   static MegabarName getMegabarByY( CarSide side, float yFt ) { uint8_t id = nearestYChecked_( FixtureMegabar, NumMegabars, side, yFt ); return (id == TIE_SENTINEL_) ? (MegabarName)NumMegabars : (MegabarName)id; }
   static ChinaName   getChinaByY( CarSide side, float yFt )   { uint8_t id = nearestYChecked_( FixtureChina, NumChinas, side, yFt );     return (id == TIE_SENTINEL_) ? (ChinaName)NumChinas     : (ChinaName)id; }
   // getMegabarByDist/getChinaByDist -- COMMENTED OUT, per explicit request
   // (confirmed unused by any real light show, ARM/WASM build clean without
   // them): distance-from-center is ambiguous even as a plain GETTER, not
   // just as a would-be setter target -- many fixtures share the exact
   // same real distance by symmetry (e.g. every side-china pair, or all 4
   // of the "12.74ft" megabars), so "nearest fixture to this radius" always
   // has a real tie, arbitrarily broken to the lowest id below. If a future
   // light show genuinely needs "which fixtures are near this distance,"
   // it should loop getMegabar(i)/getChina(i) itself and compare dimDistFt
   // directly (it can trivially collect ALL ties that way, which a single
   // nearest-match getter can never do) rather than reintroduce a getter
   // that silently drops every fixture but one.
   // static MegabarName getMegabarByDist( float distFt ) { return (MegabarName)nearestByDist_( FixtureMegabar, NumMegabars, distFt ); }
   // static ChinaName   getChinaByDist( float distFt )   { return (ChinaName)nearestByDist_( FixtureChina, NumChinas, distFt ); }
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
   // the Front/Rear side and Y only on the Left/Right side (a side's "off
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
   // Side here isn't optional -- it selects which physical STRAND to
   // search at all (X is genuinely meaningless as a search key on the
   // left/right strands, whose X is pinned constant the whole way; same
   // for Y on front/rear), so there's no way to answer "nearest by X"
   // without first knowing which strand. An invalid axis/side pairing
   // (e.g. getNeoByYFt(CarSideFront,...) -- Front doesn't disambiguate Y
   // at all) is a programming error, not a legitimate query -- logged (see
   // nospam_printf in Utilities.h, same treatment the flood ByX/ByY
   // getters' own invalid-side/tie cases get) and returns the out-of-range
   // sentinel INT32_MIN (never a valid NeoID; mirrors the flood getters'
   // own NumMegabars/NumChinas sentinel above) rather than silently
   // searching whichever strand Front happens to map to. Callers that pass
   // the result through directly to an array index must check for it
   // first; LightSetters (which builds its setters on top of these)
   // already does.
   static int32_t getNeoByXFt( CarSide side, float xFt ) {
      if ( side != CarSideFront && side != CarSideRear ) {
         nospam_printf( "AMBIG getNeoByXFt side=%d(need F/R) x=%.2f show=%s\n", (int)side, xFt, g_currentShowName );
         return INT32_MIN;
      }
      return nearestNeoOnStrand_( strandForSide_( side ), xFt, /*useX*/true );
   }
   static int32_t getNeoByYFt( CarSide side, float yFt ) {
      if ( side != CarSideLeft && side != CarSideRight ) {
         nospam_printf( "AMBIG getNeoByYFt side=%d(need L/R) y=%.2f show=%s\n", (int)side, yFt, g_currentShowName );
         return INT32_MIN;
      }
      return nearestNeoOnStrand_( strandForSide_( side ), yFt, /*useX*/false );
   }
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
   // Front/Rear share one density (SIZEOF_SMALL_NEO over CAR_WIDTH_FT),
   // Left/Right share the other (SIZEOF_LARGE_NEO over CAR_LENGTH_FT).
   static float pixelsPerFootForSide_( CarSide side ) {
      Strand s = strandForSide_( side );
      float axisLenFt = ( s == StrandFront || s == StrandRear ) ? CAR_WIDTH_FT : CAR_LENGTH_FT;
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
   // compass angle (0=front,90=right,180=rear,270=left) of a car-relative (x,y) point
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
   // Tie-checked nearest search -- see getMegabarByX/getChinaByX's own
   // comment for why this exists (a caller-supplied side can't correctly
   // resolve flood X/Y ambiguity, since the ambiguity is a property of the
   // QUERY, not of a hardcoded half of the car). TIE_SENTINEL_ (an
   // impossible fixture index -- both real counts are well under 255)
   // means "2+ fixtures are equally close," anything else is the single
   // real winner. TIE_EPSILON_FT_ guards against float-noise ties; every
   // real coordinate here is a compile-time literal, so a genuine tie
   // compares bit-exact in practice and this rarely even matters.
   static const uint8_t TIE_SENTINEL_ = 255;
   static constexpr float TIE_EPSILON_FT_ = 1e-4f;
   // Messages are kept short (character count, not information) -- each
   // still names the exact getter, side, query value, show, and (for a
   // tie) the tie count; distinct literal per (type,axis) so each of the
   // 4 real getters keeps its own independent nospam_printf cooldown.
   static uint8_t nearestXChecked_( FixtureType type, uint8_t count, CarSide side, float queryXFt ) {
      if ( side != CarSideFront && side != CarSideRear ) {
         if ( type == FixtureMegabar ) nospam_printf( "AMBIG getMegabarByX side=%d(need F/R) x=%.2f show=%s\n", (int)side, queryXFt, g_currentShowName );
         else nospam_printf( "AMBIG getChinaByX side=%d(need F/R) x=%.2f show=%s\n", (int)side, queryXFt, g_currentShowName );
         return TIE_SENTINEL_;
      }
      uint8_t best = 0; float bestDelta = 1e9f; uint8_t tieCount = 0;
      for ( uint8_t i = 0; i < count; ++i ) {
         float y = getFlood( type, i ).dimY;
         if ( ( side == CarSideFront ) ? ( y < 0.0f ) : ( y > 0.0f ) ) continue; // not on the requested side
         float d = fabsf( getFlood( type, i ).dimX - queryXFt );
         if ( d < bestDelta - TIE_EPSILON_FT_ ) { bestDelta = d; best = i; tieCount = 1; }
         else if ( fabsf( d - bestDelta ) <= TIE_EPSILON_FT_ ) { ++tieCount; }
      }
      if ( tieCount > 1 ) {
         if ( type == FixtureMegabar ) nospam_printf( "AMBIG getMegabarByX side=%d x=%.2f tie=%d show=%s\n", (int)side, queryXFt, tieCount, g_currentShowName );
         else nospam_printf( "AMBIG getChinaByX side=%d x=%.2f tie=%d show=%s\n", (int)side, queryXFt, tieCount, g_currentShowName );
      }
      return ( tieCount == 1 ) ? best : TIE_SENTINEL_; // tieCount==0 (no fixture on this side -- can't currently happen) also falls through to the sentinel, silently
   }
   static uint8_t nearestYChecked_( FixtureType type, uint8_t count, CarSide side, float queryYFt ) {
      if ( side != CarSideLeft && side != CarSideRight ) {
         if ( type == FixtureMegabar ) nospam_printf( "AMBIG getMegabarByY side=%d(need L/R) y=%.2f show=%s\n", (int)side, queryYFt, g_currentShowName );
         else nospam_printf( "AMBIG getChinaByY side=%d(need L/R) y=%.2f show=%s\n", (int)side, queryYFt, g_currentShowName );
         return TIE_SENTINEL_;
      }
      uint8_t best = 0; float bestDelta = 1e9f; uint8_t tieCount = 0;
      for ( uint8_t i = 0; i < count; ++i ) {
         float x = getFlood( type, i ).dimX;
         if ( ( side == CarSideRight ) ? ( x < 0.0f ) : ( x > 0.0f ) ) continue; // not on the requested side
         float d = fabsf( getFlood( type, i ).dimY - queryYFt );
         if ( d < bestDelta - TIE_EPSILON_FT_ ) { bestDelta = d; best = i; tieCount = 1; }
         else if ( fabsf( d - bestDelta ) <= TIE_EPSILON_FT_ ) { ++tieCount; }
      }
      if ( tieCount > 1 ) {
         if ( type == FixtureMegabar ) nospam_printf( "AMBIG getMegabarByY side=%d y=%.2f tie=%d show=%s\n", (int)side, queryYFt, tieCount, g_currentShowName );
         else nospam_printf( "AMBIG getChinaByY side=%d y=%.2f tie=%d show=%s\n", (int)side, queryYFt, tieCount, g_currentShowName );
      }
      return ( tieCount == 1 ) ? best : TIE_SENTINEL_; // tieCount==0 (no fixture on this side -- can't currently happen) also falls through to the sentinel, silently
   }
   // nearestByDist_ -- COMMENTED OUT along with getMegabarByDist/
   // getChinaByDist above (see that comment); no longer called anywhere.

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
   // positions were hand-tuned); that table uses Y+=REAR, so signs are
   // flipped here to match this file's Y+=front convention.
   static void buildFloods_() {
      static const float halfW = CAR_WIDTH_FT / 2.0f, halfH = CAR_LENGTH_FT / 2.0f; // 6, 8
      static const float STD_INSET_FT = 2.5f;
      static const float FRONT_INSET_FT = 2.0f, REAR_INSET_FT = 1.5f;
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
         {         0.0f, -( halfH - REAR_INSET_FT ) }, // 6
         {      -nearX,  -( halfH - STD_INSET_FT ) },   // 7 (210deg)
         {    -centerX,   -sideEdgeY },                  // 8 (240deg)
         {    -centerX,    0.0f },                        // 9 (270deg)
         {    -centerX,    sideEdgeY },                  // 10 (300deg)
         {      -nearX,   ( halfH - STD_INSET_FT ) },   // 11 (330deg)
      };
      // Z heights from the visualizer's own FIXTURE_HEIGHTS_FT (megabarFront:
      // 1.0, megabarDefault: 1.5, china: 2.0) -- the only real measured mount
      // heights that exist anywhere in this codebase so far.
      // Corrected per request, in 2 rounds: front-facing pair (megabar0,
      // both physical fixtures) raised 2in then lowered 3in (net -1in from
      // the original 1.0ft); all others dropped 4in then raised 1in (net
      // -3in from the original 1.5ft) -- kept as 2 separate constants since
      // they're 2 independently-measured real facts, not one shared value.
      // "Lower everything by 3in" (per request, applies to fixture mount
      // heights too -- not in that request's exceptions list) -- another
      // 3in off both, on top of the earlier corrections.
      static const float MEGABAR_FRONT_Z_FT = 1.0f + 2.0f / 12.0f - 3.0f / 12.0f - 3.0f / 12.0f, MEGABAR_DEFAULT_Z_FT = 1.5f - 4.0f / 12.0f + 1.0f / 12.0f - 3.0f / 12.0f;
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
      // 0/3/6/9 (front/right/rear/left, the 4 cardinal fixtures) CORRECTED
      // per request: previously equal to their own fixture position (a
      // zero-length aim, valid enough for a flat top-down 2D render but
      // degenerate in 3D -- with fixtureDimZ above the deck and dimZ=0 at
      // true ground, a zero horizontal offset makes the fixture-to-spot
      // vector point straight down instead of outward). Given the same
      // real 8ft "as the crow flies" throw distance every other megabar's
      // spot already uses (see comment above this table), continuing
      // straight out along each fixture's own existing direction (front/
      // rear along Y, right/left along X).
      const float mbSpotXY[ NUM_MEGABAR_LEDS ][ 2 ] = {
         {         0.0f,   14.0f },     // 0 (was 6.0, co-located with its own fixture -- now 8ft further out)
         {         6.8000f,  12.4282f }, // 1 (30deg, throw dir 30.0deg, nominal)
         {        10.7505f,   6.8450f }, // 2 (60deg, throw dir 65deg)
         {        11.5f,     0.0f },     // 3 (was 3.5, co-located with its own fixture -- now 8ft further out)
         {        10.7505f,  -6.8450f }, // 4 (120deg, throw dir 115deg)
         {         6.8000f, -12.4282f }, // 5 (150deg, throw dir 150.0deg, nominal)
         {         0.0f,  -14.5f },     // 6 (was -6.5, co-located with its own fixture -- now 8ft further out)
         {        -6.8000f, -12.4282f }, // 7 (210deg, throw dir 210.0deg, nominal)
         {       -10.7505f,  -6.8450f }, // 8 (240deg, throw dir 245deg)
         {       -11.5f,     0.0f },     // 9 (was -3.5, co-located with its own fixture -- now 8ft further out)
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

      // China: 2 fixtures per corner (front-right/front-left/rear-left/
      // rear-right), one aimed along the near front/rear edge, one aimed
      // along the near side edge. BUGFIX (a real error, not a measurement
      // question -- confirmed against README.md's own china table, which
      // this session had stopped cross-checking against): ground spots
      // (dimX/dimY) were modeled as CO-LOCATED within each pair. They are
      // NOT -- each china's real bright spot is "~1/3 of the way along
      // its assigned edge, measured from its own corner toward the far
      // corner" (README.md, "China lights"), and the 2 fixtures in a pair
      // are aimed along DIFFERENT edges (one front/rear, one side), so
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
      static const float CHINA_Z_FT = 2.0f - 3.0f / 12.0f - 3.0f / 12.0f; // was 2.0ft, lowered 3in then another 3in per request (same correction as SURFACE_BASE_HEIGHT_FT above)
      // "~1/3 of the way along its assigned edge" -- edge full length, not
      // half (a front/rear-aimed spot moves along the CAR_WIDTH_FT-long
      // front/rear edge; a side-aimed spot moves along the
      // CAR_LENGTH_FT-long side edge), measured from the TRUE car corner.
      static const float WIDTH_EDGE_OFFSET_FT = CAR_WIDTH_FT / 3.0f;   // ~4.0ft, front/rear-aimed spots move this far in X
      static const float LENGTH_EDGE_OFFSET_FT = CAR_LENGTH_FT / 3.0f; // ~5.33ft, side-aimed spots move this far in Y
      const float chDimXY[ NUM_CHINA_LEDS ][ 2 ] = {
         {  halfW,                    halfH - LENGTH_EDGE_OFFSET_FT }, // 0 front-right, aimed right/side edge -- ~1/3 rear from front
         {  halfW - WIDTH_EDGE_OFFSET_FT,  halfH },                     // 1 front-right, aimed front edge     -- ~1/3 in from right
         { -halfW + WIDTH_EDGE_OFFSET_FT,  halfH },                     // 2 front-left, aimed front edge      -- ~1/3 in from left
         { -halfW,                    halfH - LENGTH_EDGE_OFFSET_FT }, // 3 front-left, aimed left/side edge  -- ~1/3 rear from front
         { -halfW,                   -halfH + LENGTH_EDGE_OFFSET_FT }, // 4 rear-left, aimed left/side edge   -- ~1/3 forward from rear
         { -halfW + WIDTH_EDGE_OFFSET_FT, -halfH },                     // 5 rear-left, aimed rear edge        -- ~1/3 in from left
         {  halfW - WIDTH_EDGE_OFFSET_FT, -halfH },                     // 6 rear-right, aimed rear edge       -- ~1/3 in from right
         {  halfW,                   -halfH + LENGTH_EDGE_OFFSET_FT }, // 7 rear-right, aimed right/side edge -- ~1/3 forward from rear
      };
      // Each fixture's own real mount position, hand-entered individually
      // (no shared formula/offset applied at build time -- these ARE the
      // measured/estimated values, not derived from cx/cy at runtime).
      //
      // The 4 SIDE-aimed china (indices 0/3/4/7 -- ChinaRightFront/
      // ChinaLeftFront/ChinaLeftRear/ChinaRightRear) CORRECTED per
      // request: at the original hand-entered spacing, their real 8in-
      // diameter cylindrical fixture housing overlapped their corner-
      // mate's (the front/rear-aimed china sharing the same corner
      // bracket, ~0.37ft center-to-center vs. 0.667ft combined diameter).
      // Shoved toward the X axis (Y moved toward 0, X unchanged) just
      // enough for the 2 housings to clear with a 1/8in gap. dy is the
      // real fixed X separation (0.262ft between every such pair) and the
      // required center-to-center distance (8in housing + 1/8in
      // clearance = 8.125in) via the right triangle the 2 fixtures form.
      static const float CHINA_CORNER_PAIR_DX_FT = 0.262f;
      static const float CHINA_HOUSING_CLEAR_DIST_FT = 8.125f / 12.0f;
      static const float CHINA_SIDE_MOUNT_DY_FT = sqrtf( CHINA_HOUSING_CLEAR_DIST_FT * CHINA_HOUSING_CLEAR_DIST_FT - CHINA_CORNER_PAIR_DX_FT * CHINA_CORNER_PAIR_DX_FT );
      const float chMountXY[ NUM_CHINA_LEDS ][ 2 ] = {
         {  5.012f,  7.012f - CHINA_SIDE_MOUNT_DY_FT },   // 0 front-right, aimed side (was 6.750)
         {  4.750f,  7.012f },                             // 1 front-right, aimed front
         { -4.750f,  7.012f },                             // 2 front-left, aimed front
         { -5.012f,  7.012f - CHINA_SIDE_MOUNT_DY_FT },   // 3 front-left, aimed side (was 6.750)
         { -5.012f, -7.012f + CHINA_SIDE_MOUNT_DY_FT },   // 4 rear-left, aimed side (was -6.750)
         { -4.750f, -7.012f },                             // 5 rear-left, aimed rear
         {  4.750f, -7.012f },                             // 6 rear-right, aimed rear
         {  5.012f, -7.012f + CHINA_SIDE_MOUNT_DY_FT },   // 7 rear-right, aimed side (was -6.750)
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
   // Right strand: X=+50%, Y sweeps +50%(front)..-50%(rear).
   // Rear strand:  Y=-50%, X sweeps +50%(right)..-50%(left).
   // Left strand:  X=-50%, Y sweeps -50%(rear)..+50%(front).
   // (front L->R, right F->R, rear R->L, left R->F -- matches the real
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
      for ( uint16_t k = 0; k < SIZEOF_SMALL_NEO; ++k, ++i ) { // rear
         float u = (float)k / (float)( SIZEOF_SMALL_NEO - 1 );
         setNeoGeom_( i, StrandRear, 50.0f - 100.0f * u, -50.0f );
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
      strandInfo_[ StrandRear ]  = { StrandRear,  neoIdWrap( (int32_t)NEO2_OFFSET - (int32_t)FRONT ), getNumPixelsOnStrand( StrandRear ) };
      strandInfo_[ StrandLeft ]  = { StrandLeft,  neoIdWrap( (int32_t)NEO3_OFFSET - (int32_t)FRONT ), getNumPixelsOnStrand( StrandLeft ) };
   }
};

CarpetGeometry::FloodGeometry CarpetGeometry::floods_[ CarpetGeometry::NUM_FLOODS_ ];
CarpetGeometry::NeoGeom CarpetGeometry::neoGeom_[ NUM_NEO_LEDS_ACTUAL ];
CarpetGeometry::NeoStrandInfo CarpetGeometry::strandInfo_[ 4 ];

#endif
