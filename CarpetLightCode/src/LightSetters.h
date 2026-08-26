/* LightSetters.h
 *
 *    Single centralized place for every light show to WRITE colors, by
 *    position, into the real hardware output arrays (MagicCarpet's
 *    megabarLeds/chinaLeds/ropeLeds) -- the write-side counterpart to
 *    CarpetGeometry's read-only getters.
 *
 *    Class encapsulation: this lives in its own file/class, not folded
 *    into CarpetGeometry, because it needs BOTH CarpetGeometry (position
 *    lookups) AND MagicCarpet (the actual output arrays), while
 *    CarpetGeometry itself stays a pure, MagicCarpet-independent
 *    read-only geometry model -- no write access, no hardware-output
 *    dependency, so it can be reasoned about (and reused as-is, e.g. by
 *    the visualizer through the WASM bridge) without pulling in
 *    MagicCarpet's whole surface. LightSetters is the one place that
 *    depends on both.
 *
 *    Every setColor/setWhite primarily takes color as CHSV, not CRGB --
 *    HSV is this codebase's human-intuitive color space (hue/saturation/
 *    brightness map directly onto how a light show's own parameters are
 *    usually expressed), and every write funnels through FastLED's
 *    hsv2rgb_rainbow() (via CRGB/CRGBW's existing operator=(const CHSV&))
 *    -- confirmed integer-only, no floats/trig, the one real conversion
 *    point, same as every other show already assigning a CHSV into a
 *    CRGB/CRGBW. A parallel CRGB-accepting setColor() overload set exists
 *    for the one legitimate exception (see SpeedStripesShow.h): a genuine
 *    2-hue RGB-domain crossfade produces a real CRGB that was never HSV
 *    to begin with, so round-tripping it through CHSV would be pure
 *    lossy busywork. Both overload sets share the exact same position-
 *    resolution logic (forEach*_ below) -- only the final array-write
 *    step differs. Megabars have no white channel (plain CRGB) --
 *    setWhite() on a megabar target is a documented no-op, not an error.
 *
 *    Every setter follows one shape: (carpet, target-if-needed,
 *    color-or-white, start-position[, end-position]). Omitting
 *    end-position (its default, NAN) sets only the single nearest
 *    fixture/pixel to start-position -- the exact same one the
 *    same-shaped CarpetGeometry getter would return (in fact these
 *    setters are implemented directly on top of those getters). Providing
 *    both instead sets every fixture/pixel whose own position falls
 *    within [start,end] in that dimension (wrapping through 360 for
 *    angle, same convention as CarpetGeometry's own angle math).
 *
 *    Position-tag structs (ByAngleDeg/ByXFt/etc.) exist so C++ overload
 *    resolution -- not a separate function per permutation, and not a
 *    runtime dimension-enum + big switch -- picks the right internal path
 *    from the argument's own type. This keeps the whole permutation
 *    matrix (2 write-modes x {4 flood dims + 6 neo dims}) down to a small,
 *    consistently-shaped set of overloads instead of ~40 separately-named
 *    functions.
 *
 *    Neo axes deliberately do NOT include (Left/RightSide, X) or
 *    (Front/RearSide, Y) -- those pairings are genuinely ambiguous, not
 *    just unsupported: a strand's own "off axis" coordinate barely moves
 *    along its length (e.g. every pixel on the front strand has nearly
 *    the same Y, since Y only varies meaningfully going around the sides),
 *    so "nearest by Y among front-strand-only pixels" is a near-total tie
 *    across all of them. Only (Front/RearSide, X) and (Left/RightSide, Y)
 *    are provided, each in both feet and strand-local pixel-index units
 *    (pixel units let a show that already thinks in LED-index space, e.g.
 *    a scrolling animation, skip the feet round-trip entirely).
 */

#ifndef __LIGHT_SETTERS_H
#define __LIGHT_SETTERS_H

#include "CarpetGeometry.h"

class LightSetters {
 public:
   enum Target { TargetMegabar = CarpetGeometry::FixtureMegabar, TargetChina = CarpetGeometry::FixtureChina, TargetNeo };

   // ---- flood (Megabar/China) position tags: Angle/X/Y (ft), or a direct ID ----
   // No ByDistFt setter -- confirmed with the user: distance-from-center
   // is ambiguous as a WRITE target (many fixtures share the same real
   // distance by symmetry, e.g. all 4 side-china pairs), so there's no
   // single obviously-correct fixture to pick. getMegabarByDist/
   // getChinaByDist (CarpetGeometry.h) are commented out entirely now
   // (see their own comment there) -- they're ambiguous even as a plain
   // getter, not just as a would-be setter target here, so there was
   // nothing safe left to expose.
   //
   // ByXFt/ByYFt now carry a CarSide field, same shape as the neo tags
   // further down -- per explicit request, required rather than optional.
   // Scopes BOTH the single-match form (start only, end omitted) and the
   // range form (start+end both given) to fixtures on that one side --
   // for a range query, only start.side is consulted, end.side is unused
   // (both structs must share a type, but a range is one coherent query on
   // one side; a show wanting both sides for a range makes 2 calls, same
   // as the single-match form already requires). CarpetGeometry still
   // detects genuine same-side ties defensively (see its own comment) and
   // logs+returns its out-of-range sentinel; forEachFloodLinear_ below
   // checks for that sentinel and silently sets nothing rather than
   // writing to a wrong/undefined fixture. An invalid side/axis pairing is
   // logged the same way (see nospam_printf, Utilities.h) before this also
   // sets nothing.
   struct ByAngleDeg { float deg; };
   struct ByXFt      { CarpetGeometry::CarSide side; float ft; }; // side must be CarSideFront/CarSideRear
   struct ByYFt      { CarpetGeometry::CarSide side; float ft; }; // side must be CarSideLeft/CarSideRight
   struct ByID       { uint8_t id; }; // exact fixture -- no nearest-match, no range form (an ID range has no obvious "between" geometry)

   // ---- neo position tags: global AngleDeg, or (CarSide, value) in ft/pixel, or CircumferenceID ----
   // CarSide = which of the 4 sides of the CAR (CarpetGeometry::CarSide) --
   // never the underlying Strand (which wire it's on); that hardware
   // concept stays fully internal to CarpetGeometry, see its own comment.
   struct NeoByAngleDeg        { float deg; };
   struct NeoByXFt             { CarpetGeometry::CarSide side; float ft; };    // side should be CarSideFront/CarSideRear
   struct NeoByYFt             { CarpetGeometry::CarSide side; float ft; };    // side should be CarSideLeft/CarSideRight
   struct NeoByXPixel          { CarpetGeometry::CarSide side; float pixel; }; // cartesian, car-center-relative -- side should be CarSideFront/CarSideRear
   struct NeoByYPixel          { CarpetGeometry::CarSide side; float pixel; }; // cartesian, car-center-relative -- side should be CarSideLeft/CarSideRight
   struct NeoByCircumferenceID { int32_t id; };
   // Wire-order/local-index position, NOT cartesian -- "Id" inserted in
   // the name per explicit request, kept fully separate from the
   // cartesian NeoBy{X,Y}Pixel above. side should be CarSideFront/
   // CarSideRear for X, CarSideLeft/CarSideRight for Y (same restriction,
   // even though the lookup itself is axis-agnostic -- it's really just
   // "the Nth pixel wired on this side").
   struct NeoByXPixelId        { CarpetGeometry::CarSide side; int32_t localIndex; };
   struct NeoByYPixelId        { CarpetGeometry::CarSide side; int32_t localIndex; };

   // ================= flood setters =================
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, ByAngleDeg start, ByAngleDeg end = ByAngleDeg{ NAN } ) {
      forEachFloodAngle_( target, start.deg, end.deg, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodColor_( carpet, t, id, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, ByXFt start, ByXFt end = ByXFt{ CarpetGeometry::CarSideFront, NAN } ) {
      forEachFloodLinear_( target, start.side, start.ft, end.ft, DimX_, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodColor_( carpet, t, id, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, ByYFt start, ByYFt end = ByYFt{ CarpetGeometry::CarSideLeft, NAN } ) {
      forEachFloodLinear_( target, start.side, start.ft, end.ft, DimY_, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodColor_( carpet, t, id, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, ByID single ) {
      writeFloodColor_( carpet, floodTypeOf_( target ), single.id, color );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, ByAngleDeg start, ByAngleDeg end = ByAngleDeg{ NAN } ) {
      forEachFloodAngle_( target, start.deg, end.deg, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodWhite_( carpet, t, id, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, ByXFt start, ByXFt end = ByXFt{ CarpetGeometry::CarSideFront, NAN } ) {
      forEachFloodLinear_( target, start.side, start.ft, end.ft, DimX_, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodWhite_( carpet, t, id, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, ByYFt start, ByYFt end = ByYFt{ CarpetGeometry::CarSideLeft, NAN } ) {
      forEachFloodLinear_( target, start.side, start.ft, end.ft, DimY_, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodWhite_( carpet, t, id, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, ByID single ) {
      writeFloodWhite_( carpet, floodTypeOf_( target ), single.id, whiteVal );
   }

   // ================= neo setters =================
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, NeoByAngleDeg start, NeoByAngleDeg end = NeoByAngleDeg{ NAN } ) {
      (void)target; forEachNeoAngle_( start.deg, end.deg, [&]( int32_t neoId ) { writeNeoColor_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, NeoByXFt start, NeoByXFt end = NeoByXFt{ CarpetGeometry::CarSideFront, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.ft, end.ft, true, false, [&]( int32_t neoId ) { writeNeoColor_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, NeoByYFt start, NeoByYFt end = NeoByYFt{ CarpetGeometry::CarSideLeft, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.ft, end.ft, false, false, [&]( int32_t neoId ) { writeNeoColor_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, NeoByXPixel start, NeoByXPixel end = NeoByXPixel{ CarpetGeometry::CarSideFront, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.pixel, end.pixel, true, true, [&]( int32_t neoId ) { writeNeoColor_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, NeoByYPixel start, NeoByYPixel end = NeoByYPixel{ CarpetGeometry::CarSideLeft, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.pixel, end.pixel, false, true, [&]( int32_t neoId ) { writeNeoColor_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, NeoByCircumferenceID start, NeoByCircumferenceID end = NeoByCircumferenceID{ INT32_MIN } ) {
      (void)target; forEachNeoID_( start.id, end.id, [&]( int32_t neoId ) { writeNeoColor_( carpet, neoId, color ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, NeoByAngleDeg start, NeoByAngleDeg end = NeoByAngleDeg{ NAN } ) {
      (void)target; forEachNeoAngle_( start.deg, end.deg, [&]( int32_t neoId ) { writeNeoWhite_( carpet, neoId, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, NeoByXFt start, NeoByXFt end = NeoByXFt{ CarpetGeometry::CarSideFront, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.ft, end.ft, true, false, [&]( int32_t neoId ) { writeNeoWhite_( carpet, neoId, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, NeoByYFt start, NeoByYFt end = NeoByYFt{ CarpetGeometry::CarSideLeft, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.ft, end.ft, false, false, [&]( int32_t neoId ) { writeNeoWhite_( carpet, neoId, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, NeoByXPixel start, NeoByXPixel end = NeoByXPixel{ CarpetGeometry::CarSideFront, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.pixel, end.pixel, true, true, [&]( int32_t neoId ) { writeNeoWhite_( carpet, neoId, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, NeoByYPixel start, NeoByYPixel end = NeoByYPixel{ CarpetGeometry::CarSideLeft, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.pixel, end.pixel, false, true, [&]( int32_t neoId ) { writeNeoWhite_( carpet, neoId, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, NeoByCircumferenceID start, NeoByCircumferenceID end = NeoByCircumferenceID{ INT32_MIN } ) {
      (void)target; forEachNeoID_( start.id, end.id, [&]( int32_t neoId ) { writeNeoWhite_( carpet, neoId, whiteVal ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, NeoByXPixelId start, NeoByXPixelId end = NeoByXPixelId{ CarpetGeometry::CarSideFront, INT32_MIN } ) {
      (void)target; forEachNeoOnSideById_( start.side, start.localIndex, end.localIndex, [&]( int32_t neoId ) { writeNeoColor_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CHSV color, NeoByYPixelId start, NeoByYPixelId end = NeoByYPixelId{ CarpetGeometry::CarSideLeft, INT32_MIN } ) {
      (void)target; forEachNeoOnSideById_( start.side, start.localIndex, end.localIndex, [&]( int32_t neoId ) { writeNeoColor_( carpet, neoId, color ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, NeoByXPixelId start, NeoByXPixelId end = NeoByXPixelId{ CarpetGeometry::CarSideFront, INT32_MIN } ) {
      (void)target; forEachNeoOnSideById_( start.side, start.localIndex, end.localIndex, [&]( int32_t neoId ) { writeNeoWhite_( carpet, neoId, whiteVal ); } );
   }
   static void setWhite( MagicCarpet * carpet, Target target, uint8_t whiteVal, NeoByYPixelId start, NeoByYPixelId end = NeoByYPixelId{ CarpetGeometry::CarSideLeft, INT32_MIN } ) {
      (void)target; forEachNeoOnSideById_( start.side, start.localIndex, end.localIndex, [&]( int32_t neoId ) { writeNeoWhite_( carpet, neoId, whiteVal ); } );
   }

   // ================= setColor, CRGB variant (RGB-blend exception -- see file header) =================
   // Same position-resolution logic as the CHSV overloads above, just
   // writing CRGB directly with no HSV round-trip. No CRGB setWhite --
   // the white channel is a plain uint8_t regardless of color space, the
   // existing setWhite() overloads already cover it.
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, ByAngleDeg start, ByAngleDeg end = ByAngleDeg{ NAN } ) {
      forEachFloodAngle_( target, start.deg, end.deg, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodColorRGB_( carpet, t, id, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, ByXFt start, ByXFt end = ByXFt{ CarpetGeometry::CarSideFront, NAN } ) {
      forEachFloodLinear_( target, start.side, start.ft, end.ft, DimX_, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodColorRGB_( carpet, t, id, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, ByYFt start, ByYFt end = ByYFt{ CarpetGeometry::CarSideLeft, NAN } ) {
      forEachFloodLinear_( target, start.side, start.ft, end.ft, DimY_, [&]( CarpetGeometry::FixtureType t, uint8_t id ) { writeFloodColorRGB_( carpet, t, id, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, ByID single ) {
      writeFloodColorRGB_( carpet, floodTypeOf_( target ), single.id, color );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, NeoByAngleDeg start, NeoByAngleDeg end = NeoByAngleDeg{ NAN } ) {
      (void)target; forEachNeoAngle_( start.deg, end.deg, [&]( int32_t neoId ) { writeNeoColorRGB_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, NeoByXFt start, NeoByXFt end = NeoByXFt{ CarpetGeometry::CarSideFront, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.ft, end.ft, true, false, [&]( int32_t neoId ) { writeNeoColorRGB_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, NeoByYFt start, NeoByYFt end = NeoByYFt{ CarpetGeometry::CarSideLeft, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.ft, end.ft, false, false, [&]( int32_t neoId ) { writeNeoColorRGB_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, NeoByXPixel start, NeoByXPixel end = NeoByXPixel{ CarpetGeometry::CarSideFront, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.pixel, end.pixel, true, true, [&]( int32_t neoId ) { writeNeoColorRGB_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, NeoByYPixel start, NeoByYPixel end = NeoByYPixel{ CarpetGeometry::CarSideLeft, NAN } ) {
      (void)target; forEachNeoOnSide_( start.side, start.pixel, end.pixel, false, true, [&]( int32_t neoId ) { writeNeoColorRGB_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, NeoByCircumferenceID start, NeoByCircumferenceID end = NeoByCircumferenceID{ INT32_MIN } ) {
      (void)target; forEachNeoID_( start.id, end.id, [&]( int32_t neoId ) { writeNeoColorRGB_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, NeoByXPixelId start, NeoByXPixelId end = NeoByXPixelId{ CarpetGeometry::CarSideFront, INT32_MIN } ) {
      (void)target; forEachNeoOnSideById_( start.side, start.localIndex, end.localIndex, [&]( int32_t neoId ) { writeNeoColorRGB_( carpet, neoId, color ); } );
   }
   static void setColor( MagicCarpet * carpet, Target target, CRGB color, NeoByYPixelId start, NeoByYPixelId end = NeoByYPixelId{ CarpetGeometry::CarSideLeft, INT32_MIN } ) {
      (void)target; forEachNeoOnSideById_( start.side, start.localIndex, end.localIndex, [&]( int32_t neoId ) { writeNeoColorRGB_( carpet, neoId, color ); } );
   }

 private:
   enum FloodDim_ { DimX_, DimY_ };

   static CarpetGeometry::FixtureType floodTypeOf_( Target target ) {
      return ( target == TargetChina ) ? CarpetGeometry::FixtureChina : CarpetGeometry::FixtureMegabar;
   }
   static uint8_t floodCount_( Target target ) {
      return ( target == TargetChina ) ? (uint8_t)CarpetGeometry::NumChinas : (uint8_t)CarpetGeometry::NumMegabars;
   }
   static void writeFloodColor_( MagicCarpet * carpet, CarpetGeometry::FixtureType type, uint8_t id, CHSV color ) {
      if ( type == CarpetGeometry::FixtureChina ) carpet->chinaLeds[ id ] = color;
      else carpet->megabarLeds[ id ] = color;
   }
   static void writeFloodColorRGB_( MagicCarpet * carpet, CarpetGeometry::FixtureType type, uint8_t id, CRGB color ) {
      if ( type == CarpetGeometry::FixtureChina ) carpet->chinaLeds[ id ] = color;
      else carpet->megabarLeds[ id ] = color;
   }
   static void writeFloodWhite_( MagicCarpet * carpet, CarpetGeometry::FixtureType type, uint8_t id, uint8_t whiteVal ) {
      if ( type == CarpetGeometry::FixtureChina ) carpet->chinaLeds[ id ].w = whiteVal;
      // megabars are plain CRGB, no white channel -- documented no-op, see file header
   }
   static void writeNeoColor_( MagicCarpet * carpet, int32_t neoId, CHSV color ) {
      carpet->ropeLeds[ CarpetGeometry::neoIdToRawIndex( neoId ) ] = color;
   }
   static void writeNeoColorRGB_( MagicCarpet * carpet, int32_t neoId, CRGB color ) {
      carpet->ropeLeds[ CarpetGeometry::neoIdToRawIndex( neoId ) ] = color;
   }
   static void writeNeoWhite_( MagicCarpet * carpet, int32_t neoId, uint8_t whiteVal ) {
      carpet->ropeLeds[ CarpetGeometry::neoIdToRawIndex( neoId ) ].w = whiteVal;
   }

   static float wrap360_( float deg ) {
      float d = fmodf( deg, 360.0f );
      if ( d < 0.0f ) d += 360.0f;
      return d;
   }

   template < typename Fn >
   static void forEachFloodAngle_( Target target, float startDeg, float endDeg, Fn fn ) {
      CarpetGeometry::FixtureType type = floodTypeOf_( target );
      if ( isnan( endDeg ) ) {
         uint8_t id = ( type == CarpetGeometry::FixtureChina )
            ? (uint8_t)CarpetGeometry::getChinaByAngle( startDeg )
            : (uint8_t)CarpetGeometry::getMegabarByAngle( startDeg );
         fn( type, id );
         return;
      }
      uint8_t count = floodCount_( target );
      float span = wrap360_( endDeg - startDeg );
      for ( uint8_t id = 0; id < count; ++id ) {
         float offset = wrap360_( CarpetGeometry::getFlood( type, id ).dimAngleDeg - startDeg );
         if ( offset < span ) fn( type, id );
      }
   }
   template < typename Fn >
   static void forEachFloodLinear_( Target target, CarpetGeometry::CarSide side, float startVal, float endVal, FloodDim_ dim, Fn fn ) {
      CarpetGeometry::FixtureType type = floodTypeOf_( target );
      uint8_t count = floodCount_( target );
      if ( isnan( endVal ) ) {
         uint8_t id;
         if ( dim == DimX_ ) id = ( type == CarpetGeometry::FixtureChina ) ? (uint8_t)CarpetGeometry::getChinaByX( side, startVal ) : (uint8_t)CarpetGeometry::getMegabarByX( side, startVal );
         else id = ( type == CarpetGeometry::FixtureChina ) ? (uint8_t)CarpetGeometry::getChinaByY( side, startVal ) : (uint8_t)CarpetGeometry::getMegabarByY( side, startVal );
         if ( id >= count ) return; // CarpetGeometry returned its out-of-range tie/invalid-side sentinel (already logged there) -- nothing unambiguous to write to
         fn( type, id );
         return;
      }
      // range mode -- bypasses the CarpetGeometry getters entirely (no
      // single-match/tie concept applies to a range), so the same
      // side/axis validity they check internally is checked once here,
      // logged the same way (see nospam_printf, Utilities.h)
      bool sideOk = ( dim == DimX_ ) ? ( side == CarpetGeometry::CarSideFront || side == CarpetGeometry::CarSideRear )
                                     : ( side == CarpetGeometry::CarSideLeft  || side == CarpetGeometry::CarSideRight );
      if ( !sideOk ) {
         const char * dimName = ( dim == DimX_ ) ? "ByXFt" : "ByYFt";
         const char * needSide = ( dim == DimX_ ) ? "F/R" : "L/R";
         const char * targetName = ( type == CarpetGeometry::FixtureChina ) ? "China" : "Megabar";
         nospam_printf( "AMBIG set*(%s %s range) side=%d(need %s) [%.2f,%.2f] show=%s\n",
                         targetName, dimName, (int)side, needSide, startVal, endVal, g_currentShowName );
         return;
      }
      float lo = min( startVal, endVal ), hi = max( startVal, endVal );
      for ( uint8_t id = 0; id < count; ++id ) {
         const CarpetGeometry::FloodGeometry & f = CarpetGeometry::getFlood( type, id );
         float otherCoord = ( dim == DimX_ ) ? f.dimY : f.dimX;
         bool onSide = ( dim == DimX_ )
            ? ( ( side == CarpetGeometry::CarSideFront ) ? ( otherCoord >= 0.0f ) : ( otherCoord <= 0.0f ) )
            : ( ( side == CarpetGeometry::CarSideRight ) ? ( otherCoord >= 0.0f ) : ( otherCoord <= 0.0f ) );
         if ( !onSide ) continue;
         float v = ( dim == DimX_ ) ? f.dimX : f.dimY;
         if ( v >= lo && v <= hi ) fn( type, id );
      }
   }

   template < typename Fn >
   static void forEachNeoAngle_( float startDeg, float endDeg, Fn fn ) {
      if ( isnan( endDeg ) ) { fn( CarpetGeometry::getNeoByAngleDeg( startDeg ) ); return; }
      float span = wrap360_( endDeg - startDeg );
      for ( uint16_t raw = 0; raw < NUM_NEO_LEDS_ACTUAL; ++raw ) {
         float offset = wrap360_( CarpetGeometry::getNeoGeom( raw ).angleFromForwardDeg - startDeg );
         if ( offset < span ) fn( CarpetGeometry::rawIndexToNeoId( raw ) );
      }
   }
   // Takes a CarSide (which of the 4 sides of the car), never a Strand --
   // maps to the underlying wiring internally via CarpetGeometry's own
   // strandForSide_(), the one place that coupling is assumed. Pixel
   // units are cartesian (same car-center-relative position as Ft, just
   // rescaled) -- rescale to Ft once here and operate in Ft throughout, no
   // separate pixel-space branch needed.
   template < typename Fn >
   static void forEachNeoOnSide_( CarpetGeometry::CarSide side, float startVal, float endVal, bool useX, bool pixelUnits, Fn fn ) {
      // Range mode below bypasses CarpetGeometry::getNeoBy{X,Y}Ft entirely
      // (it iterates the whole strand directly), so the same axis/side
      // validity those getters check internally is checked once here up
      // front, covering both the single-match and range branches -- see
      // their own comment for why an invalid pairing (e.g. side=Front for
      // a Y query) can't be resolved by silently picking a strand anyway.
      bool sideValid = useX ? ( side == CarpetGeometry::CarSideFront || side == CarpetGeometry::CarSideRear )
                            : ( side == CarpetGeometry::CarSideLeft  || side == CarpetGeometry::CarSideRight );
      if ( !sideValid ) {
         const char * axisName = useX ? "X" : "Y";
         const char * unitName = pixelUnits ? "Pixel" : "Ft";
         const char * needSide = useX ? "F/R" : "L/R";
         nospam_printf( "AMBIG set*(NeoBy%s%s) side=%d(need %s) [%.2f,%.2f] show=%s\n",
                         axisName, unitName, (int)side, needSide, startVal, endVal, g_currentShowName );
         return;
      }
      float ppf = pixelUnits ? CarpetGeometry::pixelsPerFootForSide_( side ) : 1.0f;
      float startFt = pixelUnits ? startVal / ppf : startVal;
      float endFt = pixelUnits ? endVal / ppf : endVal; // NAN / ppf is still NAN
      if ( isnan( endFt ) ) {
         int32_t neoId = useX ? CarpetGeometry::getNeoByXFt( side, startFt ) : CarpetGeometry::getNeoByYFt( side, startFt );
         fn( neoId ); // side already validated above -- can't be the INT32_MIN sentinel
         return;
      }
      const CarpetGeometry::NeoStrandInfo & info = CarpetGeometry::getStrandInfo( CarpetGeometry::strandForSide_( side ) );
      float lo = min( startFt, endFt ), hi = max( startFt, endFt );
      for ( uint16_t local = 0; local < info.length; ++local ) {
         int32_t neoId = (int32_t)info.startNeoId + local;
         uint16_t raw = CarpetGeometry::neoIdToRawIndex( neoId );
         const CarpetGeometry::NeoGeom & g = CarpetGeometry::getNeoGeom( raw );
         float v = useX ? ( g.xPercent / 100.0f * CarpetGeometry::CAR_WIDTH_FT ) : ( g.yPercent / 100.0f * CarpetGeometry::CAR_LENGTH_FT );
         if ( v >= lo && v <= hi ) fn( neoId );
      }
   }
   // Wire-order/local-index range -- NOT cartesian, backs the
   // NeoBy{X,Y}PixelId setters (see their own comment).
   template < typename Fn >
   static void forEachNeoOnSideById_( CarpetGeometry::CarSide side, int32_t startId, int32_t endId, Fn fn ) {
      if ( endId == INT32_MIN ) { fn( CarpetGeometry::getNeoByXPixelId( side, startId ) ); return; }
      const CarpetGeometry::NeoStrandInfo & info = CarpetGeometry::getStrandInfo( CarpetGeometry::strandForSide_( side ) );
      int32_t lo = min( startId, endId ), hi = max( startId, endId );
      for ( int32_t local = max( lo, (int32_t)0 ); local <= min( hi, (int32_t)info.length - 1 ); ++local ) {
         fn( (int32_t)info.startNeoId + local );
      }
   }
   template < typename Fn >
   static void forEachNeoID_( int32_t startId, int32_t endId, Fn fn ) {
      if ( endId == INT32_MIN ) { fn( CarpetGeometry::neoIdWrap( startId ) ); return; }
      int32_t lo = min( startId, endId ), hi = max( startId, endId );
      for ( int32_t id = lo; id <= hi; ++id ) fn( CarpetGeometry::neoIdWrap( id ) );
   }
};

#endif
