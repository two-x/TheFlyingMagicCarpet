/* interface-mode.js
 *
 *    Everything Interface role needs, extracted into its own file so
 *    Dev Tool mode's code can be *structurally* incapable of reaching
 *    any of it, not just "not supposed to call it" by convention.
 *
 *    Interface role is on hold (no real radio link exists yet -- see
 *    ~/.claude/plans/quiet-watching-cherny.md, Project B) -- confirmed via
 *    a mandate audit that visualizerRole never actually gets set to
 *    'interface' anywhere in the current UI, making every function below
 *    100% unreachable today. Kept here, out of the way, rather than
 *    deleted, so it's easy to pick back up whenever Project B's real
 *    radio link exists for real. This file is a deliberate, approved
 *    exception to the zero-duplication mandate (claude_dev_prompts.md
 *    prompt 6): it re-implements show/config-screen rendering in JS
 *    rather than streaming real LED bytes over a bandwidth-constrained
 *    radio link. Must be re-ported from the real FW whenever the
 *    corresponding src/*Show.h / MagicCarpet::showXXX() logic changes --
 *    it does not update itself, and (being unreachable) nothing will
 *    ever notice it drifting until Interface role is actually revived.
 *
 *    Loaded as a classic (non-module) script, deliberately -- shares the
 *    same global scope as carpet-visualizer.html's own inline script (no
 *    ES module `type="module"`, which file:// blocks via CORS on the
 *    module fetch). Everything here depends on globals that script
 *    defines first (Ctl, ropeLeds/megabarLeds/chinaLeds, blend/scale255/
 *    hsv/makeRandomWalk/randomWalkValue/pickOverlapColor/boostPreserveHue,
 *    AudioBoard, megabarPos/CHINA_POS/megabarAngle/ledPoint,
 *    MEGABAR_WASH/CHINA_WASH and friends, NUM_MEGABAR/NUM_CHINA/NUM_NEO,
 *    CAR_X/CAR_Y/PX_PER_FT, WASM_BAND/BAND_LOW/BAND_HIGH, HEADLIGHT_INDEX)
 *    -- see the <script> tag ordering in carpet-visualizer.html, which
 *    loads this file at exactly the point those are already defined but
 *    before renderCurrent() (which calls renderInterfaceMode() below) is
 *    ever invoked.
 */

/* =====================================================================
   SHOW LOGIC -- ported from the firmware's *Show.h files. Visual
   approximations where noted; audio-reactive math is faithful.
   ===================================================================== */
let showT0 = performance.now();
let lastFrameT = performance.now();

function updatePotEnergy() { return Ctl.pot; } // 0-100, shared "energy" stand-in

// Precomputed once (fixture positions/washes are static): for each
// megabar, the rope LED index whose own angular position (from car
// center) most closely matches a straight line from car center through
// that megabar's ground spot -- i.e. "the neopixel that line passes
// thru," per request. Used by showNightrider below whenever there's no
// audio to react to, so the floods still have a physically-motivated
// color instead of going dark or freezing on a stale value.
const NIGHTRIDER_MEGABAR_ROPE_IDX = Array.from({ length: NUM_MEGABAR }, (_, m) => {
  const wash = m === HEADLIGHT_INDEX ? MEGABAR_FRONT_WASH : MEGABAR_WASH;
  const spot = groundSpotPx(megabarPos(m), megabarAngle(m), wash);
  const spotAng = (Math.atan2(spot.y - CAR_Y, spot.x - CAR_X) * 180 / Math.PI + 90 + 360) % 360;
  let best = 0, bestDelta = Infinity;
  for (let i = 0; i < NUM_NEO; i++) {
    const pt = ledPoint(i);
    const ang = (Math.atan2(pt.y - CAR_Y, pt.x - CAR_X) * 180 / Math.PI + 90 + 360) % 360;
    let delta = Math.abs(ang - spotAng);
    if (delta > 180) delta = 360 - delta;
    if (delta < bestDelta) { bestDelta = delta; best = i; }
  }
  return best;
});

function showNightrider(nowMs, dtMs) {
  clearRope(); clearMegabars(); clearChinas();
  const t = (nowMs - showT0) / 1000;
  let hue;
  if (Ctl.variation === 0) {
    hue = Math.round(Ctl.pot / 100 * 255);
  } else {
    // variation 2 ("Auto cycle, no audio") reuses variation 1's exact hue
    // drift -- the two only differ in how the floods behave below.
    const periodSec = 20 * 60 * Math.pow(1 / (20*60), Ctl.pot / 100); // 20min -> 1s
    hue = Math.round((t % periodSec) / periodSec * 255);
  }
  const c1 = hsv(hue, 255, 255), c2 = hsv((hue + 128) % 256, 255, 255);
  // Real Nightrider bounces independently within each of 8 edge segments
  // (front/back halves x2, left/right halves x2), all sharing the same
  // phase -- so the chase point appears simultaneously at several points
  // around the full loop, not as a single dot circling once. Approximated
  // here as 8 equal segments (not the real firmware's exact unequal
  // corner-vs-straight boundaries) each with their own synchronized
  // bounce, which gets the "handful of segments visible at once" look.
  const NUM_SEGMENTS = 8;
  const segLen = NUM_NEO / NUM_SEGMENTS;
  const bouncePeriodSec = 2.4;
  const phase = (t % bouncePeriodSec) / bouncePeriodSec;
  const triangle = phase < 0.5 ? phase * 2 : (1 - phase) * 2; // 0->1->0
  const bouncePos = triangle * segLen;
  for (let i = 0; i < NUM_NEO; i++) {
    const posInSeg = i % segLen;
    const d = Math.abs(posInSeg - bouncePos);
    const f = Math.max(0, 255 - d * 10);
    ropeLeds[i] = blend(c1, c2, f);
  }
  // Variation 2 ("Auto cycle, no audio") never reacts to sound at all;
  // variations 0/1 fall back to the same no-audio behavior whenever
  // there's genuinely nothing to react to (AudioBoard.silent) instead of
  // sitting at black -- per request: draw a line from car center through
  // each flood's own ground spot, and color that flood with whichever
  // neopixel color that line passes through (NIGHTRIDER_MEGABAR_ROPE_IDX,
  // precomputed above), so idle floods still read as part of the same
  // chase pattern the rope is showing.
  if (Ctl.variation === 2 || AudioBoard.silent) {
    for (let m = 0; m < NUM_MEGABAR; m++) megabarLeds[m] = ropeLeds[NIGHTRIDER_MEGABAR_ROPE_IDX[m]];
  } else {
    const lowHit = AudioBoard.getHitPct(BAND_LOW) / 100 * 255;
    const highHit = AudioBoard.getHitPct(BAND_HIGH) / 100 * 255;
    // NOTE: c1/c2 (both hsv(_,255,255)) can land on hues with different
    // r+g+b sums (255 at a pure primary vs up to 510 at a secondary), which
    // used to get "corrected" here by force-brightening one color to match
    // the other's sum -- reverted, since for a large gap that means pushing
    // a saturated color almost all the way to white, destroying its hue
    // entirely (confirmed: this is why waterflames/hue-to-white briefly
    // rendered every hit as plain white). Fixed for real at the RENDERING
    // level instead -- see drawGroundEllipse()/drawWash()'s brightness
    // metric, which no longer depends on hue at all.
    const mbLow = blend({r:0,g:0,b:0}, c2, lowHit);
    const mbHigh = blend({r:0,g:0,b:0}, c1, highHit);
    for (let m = 0; m < NUM_MEGABAR; m++) megabarLeds[m] = (m % 3 === 0) ? mbHigh : mbLow;
  }
}

/* ---- FlameShow: real Fire2012-style heat-diffusion sparkle, ported from
   src/FlameShow.h ---- Previous visualizer version was a plain sine-wave
   flicker approximation with no real per-LED texture -- that's the
   visualizer bug behind "just two fading colors, not little points of
   sparkle." This ports the real cool/diffuse/spark heat simulation
   per-LED, mapped through the same 4 palette variations. (While porting
   this, found and fixed a real out-of-bounds array read in the actual
   firmware's diffusion step -- src/FlameShow.h used `i+i` instead of `i+1`
   for the "higher" neighbor index, reading past the array for roughly the
   upper half of it. Fixed there too; flagged separately since it changes
   real hardware behavior.) */
// BUGFIX: waterflames' top stop used to be pure White (matching the real
// firmware's own per-LED sparkle palette, where hot embers legitimately
// blow out to white -- that's still fine for the ROPE's sparkle texture,
// see flamePaletteColor() below). The problem was reusing this SAME array
// to also pick the flood bass/treble reference colors (see floodHuePair()
// below): sampling near the top stop landed treble very close to White,
// reading as "the floods are just white" for hits. Changed to a vivid,
// non-white cyan instead -- flames' own array (DarkRed..Yellow) never had
// this problem since none of ITS stops are white either, so this just
// brings waterflames' data in line with flames' own pattern.
const FLAME_PALETTES = [
  [[0,0,90],[10,30,255],[70,200,255],[120,230,255]],   // 0 waterflames: DarkBlue,Blue,Aqua,BrightCyan
  [[90,0,0],[255,20,0],[255,120,0],[255,220,60]],       // 1 flames: DarkRed,Red,Orange,Yellow
];
function flamePaletteColor(variation, index, driftHue, satByte) {
  index = Math.max(0, Math.min(255, index));
  if (variation < 2) {
    const pal = FLAME_PALETTES[variation];
    const idx = (index / 255) * (pal.length - 1);
    const lo = Math.floor(idx), hi = Math.min(pal.length - 1, lo + 1), f = idx - lo;
    return { r: pal[lo][0]+(pal[hi][0]-pal[lo][0])*f, g: pal[lo][1]+(pal[hi][1]-pal[lo][1])*f, b: pal[lo][2]+(pal[hi][2]-pal[lo][2])*f };
  }
  if (variation === 3) {
    // hue-to-white: white is the SECONDARY (low-index/base) color, the
    // drifting hue is primary (high-index/accent) -- reversed from real
    // firmware's own CRGBPalette16(driftClr, driftClr, White, White)
    // (driftClr low, White high), a deliberate visualizer-only deviation
    // per request.
    const driftClr = hsv(driftHue, satByte, 255);
    const white = { r: 255, g: 255, b: 255 };
    if (index < 85) return white;
    if (index > 170) return driftClr;
    return blend(white, driftClr, ((index - 85) / 85) * 255);
  }
  // variation 2: shifting hues -- 2 complementary full-brightness colors
  const colorA = hsv(driftHue, satByte, 255), colorB = hsv((driftHue + 128) % 256, satByte, 255);
  if (index < 85) return colorA;
  if (index > 170) return colorB;
  return blend(colorA, colorB, ((index - 85) / 85) * 255);
}
// Bass/treble FLOOD reference colors (loColor/hiColor) -- ONE common
// algorithm for all 4 variations, per request: every flame variation is,
// at its core, two constant hues (a more-common "primary" and a
// less-common "secondary"), regardless of how the per-LED sparkle
// texture's brightness/blend moves around live. Variations 1 (flames) and
// 2 (shift hues) already read this way naturally -- this just makes 0
// (waterflames) and 3 (hue-white) use the SAME two-constant-hues shape
// instead of their own bespoke sampling, which is what was producing a
// literal white color for one of the two (see FLAME_PALETTES' own comment
// for waterflames' half of this; hue-to-white's own palette function
// deliberately returns flat White below index 85, which this bypasses for
// flood purposes specifically -- the rope's per-LED sparkle still calls
// flamePaletteColor() directly and keeps blowing out to true white at its
// hottest, unchanged, since that's the "hue-TO-white" look's whole point).
function rgbToHsv(c) {
  const r = c.r / 255, g = c.g / 255, b = c.b / 255;
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
  let h = 0;
  if (d > 0) {
    if (mx === r) h = ((g - b) / d) % 6;
    else if (mx === g) h = (b - r) / d + 2;
    else h = (r - g) / d + 4;
    h *= 60;
    if (h < 0) h += 360;
  }
  const s = mx === 0 ? 0 : d / mx;
  return { h: h / 360 * 255, s: s * 255, v: mx * 255 };
}
// Reconstructs a color at its own hue, but a CALLER-SUPPLIED saturation
// and full (255) value -- the brightest that hue/sat combination can
// reach. Used to compare two "reference" colors (e.g. a fire palette's
// dim, highly-saturated "cool" stop vs its much brighter, far LESS
// saturated "hot" stop) on equal footing, so a bass hit and a treble hit
// read at the same true maximum brightness. Forcing a SHARED saturation
// (not each color's own -- see BUGFIX below) matters because at a fixed
// V=255, saturation alone sets the ceiling on r+g+b: a fully-saturated
// primary hue tops out at sum=255, a fully desaturated one at sum=765 --
// preserving each color's own very different saturation would leave most
// of the original gap in place. Real, portable RGB math (not a rendering-
// only trick) -- the equivalent firmware fix is
// rgb2hsv_approximate()+CHSV(h,satByte,255).
function hueAtFullBrightness(c, satByte) {
  const { h } = rgbToHsv(c);
  return hsv(Math.round(h), satByte, 255);
}
function floodHuePair(variation, driftHue, satByte) {
  if (variation === 3) {
    // Per request: one of the two colors is constant white, the other
    // varies in hue as before (previously both varied, 128 apart).
    return [hsv(driftHue, satByte, 255), { r: 255, g: 255, b: 255 }];
  }
  if (variation < 2) {
    // BUGFIX: raw palette samples at these indices have very different
    // inherent r+g+b sums (dim, highly-saturated "cool" stop vs bright,
    // much-less-saturated "hot" stop) -- as hit-flash reference colors
    // that's a genuine bass-vs-treble brightness mismatch in the actual
    // RGB values (not just a visualizer display artifact -- FlameShow.h's
    // own paletteColor() lookup uses the exact same palette, so real
    // hardware shows the same dim bass floods). A first attempt at this
    // fix preserved each color's own (very different) saturation and only
    // forced V=255, which left most of the gap in place (see
    // hueAtFullBrightness's own comment for why) -- forcing a SHARED
    // saturation (satByte) actually closes it.
    return [hueAtFullBrightness(flamePaletteColor(variation, 40, driftHue, satByte), satByte), hueAtFullBrightness(flamePaletteColor(variation, 230, driftHue, satByte), satByte)];
  }
  return [flamePaletteColor(variation, 40, driftHue, satByte), flamePaletteColor(variation, 230, driftHue, satByte)];
}

const flameCurrTemp = new Array(NUM_NEO).fill(0);
const flamePrevTemp = new Array(NUM_NEO).fill(0);
const flameMegabarHeat = new Array(NUM_MEGABAR).fill(0);
const flameChinaHeat = new Array(NUM_CHINA).fill(0);
const flameMegabarColorPick = new Array(NUM_MEGABAR).fill(0).map(() => Math.random() < 0.5);
const flameChinaColorPick = new Array(NUM_CHINA).fill(0).map(() => Math.random() < 0.5);
const flameHueRateWalk = makeRandomWalk(), flameSatWalk = makeRandomWalk();
let flameShiftHue = 0, flameLastHitMs = -99999, flameLastStepMs = -99999;
let flameChinaSwapBassOnSides = true, flameChinaSwapNextAt = 0;
const FLAME_CHINA_FRONTBACK = [1, 2, 5, 6], FLAME_CHINA_SIDES = [0, 3, 4, 7]; // matches SpeedStripesShow's front/back vs side china grouping
function resetFlameState() {
  flameCurrTemp.fill(0); flamePrevTemp.fill(0);
  flameMegabarHeat.fill(0); flameChinaHeat.fill(0);
  flameShiftHue = 0; flameHueRateWalk.initialized = false; flameSatWalk.initialized = false;
  flameLastHitMs = -99999; flameChinaSwapNextAt = 0;
}

// RETIRED: this was the only place Flame's real algorithm lived (real
// FlameShow.h had only ever done a flat single-color flood fill). Per the
// zero-duplication mandate, the real logic now lives in FlameShow.h
// itself (ported from this function, unchanged since before the WASM
// migration) -- see its class comment. Interface mode isn't a connected
// feature yet (no ESP32 radio bridge exists), so this is a no-op rather
// than a second copy of the algorithm to keep in sync.
function showFlame(nowMs, dtMs) {}

// True corner-to-corner geometry, matching BumpingAudioShow.h's updateVuMeter()
// exactly: SIZEOF_LARGE_NEO_CORNER (33 LEDs) inset from each side channel's
// boundary marks where the "true corner" sits for this meter -- the small
// gap between that and the front/back edge is deliberately left unlit by
// the real firmware, replicated here rather than filled in.
const SIZEOF_LARGE_NEO_CORNER = 33;
const VU_HALFLEN = SIZEOF_LARGE_NEO / 2 - SIZEOF_LARGE_NEO_CORNER; // 143, true corner to center
const VU_MAX_REACH = VU_HALFLEN * 1.15; // 100%-level reach: 15% past center, so maxed meters cross there
const VU_FRONT_RIGHT = RIGHT + SIZEOF_LARGE_NEO_CORNER;
const VU_BACK_RIGHT = REAR - SIZEOF_LARGE_NEO_CORNER; // BUGFIX: referenced the global's OLD name (BACK) -- renamed to REAR in carpet-visualizer.html earlier this session, left stale here since this file wasn't part of that sweep
const VU_BACK_LEFT = LEFT + SIZEOF_LARGE_NEO_CORNER;
const VU_FRONT_LEFT = NUM_NEO - SIZEOF_LARGE_NEO_CORNER;

function showEqualizer(nowMs, dtMs) {
  if (Ctl.variation === 2) { showEqualizerNewStandard(nowMs, dtMs); return; }
  if (Ctl.variation === 3) { showEqualizerPixelWar(nowMs, dtMs); return; }
  clearRope(); clearChinas();
  const t = (nowMs - showT0) / 1000;
  const c1 = { r: 0, g: 0, b: 255 }, c2 = { r: 255, g: 0, b: 0 };
  if (Ctl.variation === 1) {
    // dual VU meter, faithful port of the real firmware: bass (red) is
    // based across the WHOLE back edge and grows forward along both sides;
    // treble (blue) is based across the WHOLE front edge and grows
    // backward along both sides -- symmetric on both sides by construction
    // (same VU_HALFLEN/VU_MAX_REACH used for left and right), fixing the
    // previous asymmetric/uneven-height bug. Brightness tracks level
    // directly (dim meter for a quiet signal, not just a short one).
    const bassHitPct = AudioBoard.getHitPct(BAND_LOW), trebleHitPct = AudioBoard.getHitPct(BAND_HIGH);
    // Meter LENGTH along the rope stays driven by the true, uncompensated
    // hit% (it's meant to represent the actual audio level spatially).
    const bassLitCount = (bassHitPct / 100) * VU_MAX_REACH;
    const trebleLitCount = (trebleHitPct / 100) * VU_MAX_REACH;
    // Meter COLOR/brightness gets the same perceptual boost as the chase
    // variant above (pure blue reads dimmer than pure red on-screen at
    // equal channel values -- sRGB perceptual-luminance weighting), so red
    // and blue floods read equally bright at equal hit%.
    const trebleColorPct = Math.min(100, trebleHitPct * 1.6);
    const bassClr = scale255(c2, Math.round(bassHitPct/100*255));
    const trebleClr = scale255(c1, Math.round(trebleColorPct/100*255));
    for (let i = REAR; i < LEFT; i++) ropeLeds[i] = bassClr;   // rear edge: always bass base, full width
    for (let i = 0; i < RIGHT; i++) ropeLeds[i] = trebleClr;   // front edge: always treble base, full width
    // refBack/refFront: the meter's own corner-to-center reference points
    // (VU_HALFLEN/VU_MAX_REACH are calibrated from these, NOT the physical
    // strand ends -- matches the real firmware's identical corner-inset
    // convention). loopStart/loopEnd: the actual LED range to paint, which
    // covers this side's ENTIRE physical strand (true corner to true
    // corner), not just the reference span -- so the ~33-LED zone between
    // the true corner and the inset reference point is filled by this same
    // side-local distance logic (naturally reading as "always lit,
    // proportional to level" there, since it's past the reference point in
    // BOTH functions' math), never by reaching into a neighboring strand's
    // own front/back edge logic. Each side stays self-contained.
    function vuSide(refBack, refFront, loopStart, loopEnd, dir) {
      for (let i = loopStart; dir > 0 ? i <= loopEnd : i >= loopEnd; i += dir) {
        const distFromBack = dir > 0 ? (i - refBack) : (refBack - i);
        const distFromFront = dir > 0 ? (refFront - i) : (i - refFront);
        const bassLit = distFromBack < bassLitCount, trebleLit = distFromFront < trebleLitCount;
        if (bassLit && trebleLit) ropeLeds[i] = (Math.random() < 0.5) ? bassClr : trebleClr; // crossing zone: equal contributors, random pick instead of blending
        else if (bassLit) ropeLeds[i] = bassClr;
        else if (trebleLit) ropeLeds[i] = trebleClr;
        else ropeLeds[i] = {r:0,g:0,b:0};
      }
    }
    vuSide(VU_BACK_RIGHT, VU_FRONT_RIGHT, REAR - 1, RIGHT, -1);      // right strand: true back corner -> true front corner
    vuSide(VU_BACK_LEFT, VU_FRONT_LEFT, LEFT, NUM_NEO - 1, 1);       // left strand: true back corner -> true front corner
    // Floods (megabar + china) reproduce the same meter, split by their own
    // physical front/back position -- not in the real firmware yet (which
    // only drives the rope here), added per request so the floods aren't
    // dark during this variation.
    clearMegabars();
    for (let m = 0; m < NUM_MEGABAR; m++) {
      megabarLeds[m] = (megabarPos(m).y < CAR_Y) ? trebleClr : bassClr;
    }
    for (let c = 0; c < NUM_CHINA; c++) {
      chinaLeds[c] = (CHINA_POS[c].y < CAR_Y) ? trebleClr : bassClr;
    }
  } else {
    const pos = (t * 60) % NUM_NEO;
    for (let i = 0; i < NUM_NEO; i++) {
      const d = Math.min(Math.abs(i - pos), NUM_NEO - Math.abs(i - pos));
      ropeLeds[i] = blend(c1, c2, Math.max(0, 255 - d * 4));
    }
    const lowHit = AudioBoard.getHitPct(BAND_LOW)/100*255;
    // BUGFIX (visualizer display, not data/code): the treble megabars
    // (m%3===0) blend toward pure blue (c1) while the bass ones blend
    // toward pure red (c2) -- at EQUAL hit percentages this produces
    // numerically-symmetric RGB values, but pure blue reads noticeably
    // dimmer than pure red on a screen (sRGB/rec709 perceptual-luminance
    // weighting: red contributes ~5x more to perceived brightness than
    // blue at the same channel value). The underlying hit percentages
    // themselves are computed identically for both bands -- this is a
    // display-only compensation, not a change to any audio-reactivity
    // data, so it's scoped to just this chase variant's rendering.
    const highHit = Math.min(255, AudioBoard.getHitPct(BAND_HIGH)/100*255 * 1.6);
    for (let m = 0; m < NUM_MEGABAR; m++) {
      megabarLeds[m] = (m % 3 === 0) ? blend({r:0,g:0,b:0}, c1, highHit) : blend({r:0,g:0,b:0}, c2, lowHit);
    }
  }
  if (AudioBoard.silent) {
    // same floodlight swap-flash behavior as new_standard/pixel_war during
    // silence, per request -- overrides just the megabar/china output
    // computed above (which would otherwise sit near-dark with no sound to
    // react to); rope is left as whatever the variation above already drew.
    showEqSilenceFloods(nowMs, dtMs, c2, c1); // c2=bass/red, c1=treble/blue, this show's own convention
  }
}

/* ---- Equalizer variation: new_standard ----------------------------------
   Whole-show 2-color palette (Bcolor/Tcolor), auto-cycling china/megabar
   patterns (8s per cycle each, independent cycle counts), 4 rope chase
   segments, and a distinct silence behavior. See the per-section comments
   below for each piece; this is a best-effort literal implementation of a
   long, detailed spec -- a few genuinely underspecified corners (noted
   inline) were resolved with a reasonable, clearly-flagged judgment call
   rather than left unbuilt. */
const NEWSTD_HUE_PERIOD_S = 20; // 10s red->yellow, 10s back -- full cycle
const NEWSTD_SAT_PERIOD_S = 14; // 7/10 of the hue period, per request
const NEWSTD_HUE_RED = 0, NEWSTD_HUE_YELLOW = 42, NEWSTD_HUE_TREBLE = 160; // blue, constant
// Front/rear vs left/right megabar grouping for cycle 2 -- classified by
// which axis each fixture's own ground-spot offset from car center is
// larger along, not a hardcoded index list, so it stays correct if
// MEGABAR_POS_FT is ever retuned.
const NEWSTD_FRONT_REAR_MEGABARS = Array.from({length:NUM_MEGABAR}, (_,m)=>m).filter(m => {
  const p = megabarPos(m); return Math.abs(p.y - CAR_Y) >= Math.abs(p.x - CAR_X);
});
const NEWSTD_LEFT_RIGHT_MEGABARS = Array.from({length:NUM_MEGABAR}, (_,m)=>m).filter(m => !NEWSTD_FRONT_REAR_MEGABARS.includes(m));

let newStdHueTimeS = 0, newStdSatTimeS = 0; // hue time freezes during silence, sat time never does (per request)
let newStdChinaCycleStart = -1, newStdChinaCycle = 0;
let newStdMegabarCycleStart = -1, newStdMegabarCycle = 0;
let newStdCycle1TrebleIsMod3 = true;      // megabar cycle 1's randomized-at-start assignment
let newStdCycle2FrontRearIsBass = true;   // megabar cycle 2's randomized-at-start assignment (swaps every 2s)
let newStdCycle3BassAssignment = null;    // megabar cycle 3's fixed-per-cycle array, true=bass
let newStdMegabarHeat = new Array(NUM_MEGABAR).fill(0);   // cycle 3 only: 0=black, decays from 1
let newStdPrevBassHit = 0, newStdPrevTrebleHit = 0;       // edge-detection for "a hit just happened"
// Silence-mode per-flood state (megabars 0..NUM_MEGABAR-1, then china
// NUM_MEGABAR..NUM_MEGABAR+NUM_CHINA-1, one shared array).
const NEWSTD_NUM_FLOODS = NUM_MEGABAR + NUM_CHINA;
let newStdFloodIsB = new Array(NEWSTD_NUM_FLOODS).fill(false);
let newStdFloodBrightness = new Array(NEWSTD_NUM_FLOODS).fill(0.15);
let newStdNextSwapMs = -1;
// Cross-fade state for china/megabar CYCLE transitions (not silence mode,
// which already has its own spike/decay) -- on a cycle change, whatever
// color each flood was actually last showing is captured as the fade
// start, and blended toward the new cycle's own (still live, per-frame)
// target color over 200ms, so a phase change eases rather than snaps.
const NEWSTD_FADE_MS = 200;
let newStdChinaFadeFrom = new Array(NUM_CHINA).fill(null);
let newStdChinaFadeStartMs = -1e9;
let newStdLastChinaColor = new Array(NUM_CHINA).fill(null);
let newStdMegabarFadeFrom = new Array(NUM_MEGABAR).fill(null);
let newStdMegabarFadeStartMs = -1e9;
let newStdLastMegabarColor = new Array(NUM_MEGABAR).fill(null);
// Shared "no sound" flood behavior, used by ALL 4 equalizer variations (per
// request that Chase/VU meter go dark-with-nothing-to-react-to during
// silence the same way new_standard/pixel_war already do): floods sit at a
// dim 15% baseline, and every 100-500ms a random flood swaps its color
// assignment with a random flood of the other color, spiking to full
// brightness then easing to 80% over ~200ms. colorB/colorT are whichever
// two base colors the calling variation uses (Bcolor/Tcolor for
// new_standard/pixel_war, bass-red/treble-blue for Chase/VU meter).
function showEqSilenceFloods(nowMs, dtMs, colorB, colorT) {
  const dtSec = Math.max(0, Math.min(0.2, dtMs / 1000));
  if (newStdNextSwapMs < 0 || nowMs >= newStdNextSwapMs) {
    newStdNextSwapMs = nowMs + 100 + Math.random() * 400;
    const bIdxs = [], tIdxs = [];
    for (let i = 0; i < NEWSTD_NUM_FLOODS; i++) (newStdFloodIsB[i] ? bIdxs : tIdxs).push(i);
    if (bIdxs.length && tIdxs.length) {
      const a = bIdxs[Math.floor(Math.random() * bIdxs.length)];
      const b = tIdxs[Math.floor(Math.random() * tIdxs.length)];
      newStdFloodIsB[a] = false; newStdFloodIsB[b] = true;
      newStdFloodBrightness[a] = 1.0; newStdFloodBrightness[b] = 1.0;
    }
  }
  for (let i = 0; i < NEWSTD_NUM_FLOODS; i++) {
    const target = newStdFloodBrightness[i] > 0.15 + 1e-3 ? 0.8 : 0.15;
    newStdFloodBrightness[i] += (target - newStdFloodBrightness[i]) * Math.min(1, dtSec / 0.2);
    const color = newStdFloodIsB[i] ? colorB : colorT;
    const lit = scale255(color, newStdFloodBrightness[i] * 255);
    if (i < NUM_MEGABAR) megabarLeds[i] = lit; else chinaLeds[i - NUM_MEGABAR] = lit;
  }
}
function resetNewStandardState() {
  newStdHueTimeS = 0; newStdSatTimeS = 0;
  newStdChinaCycleStart = -1; newStdChinaCycle = 0;
  newStdMegabarCycleStart = -1; newStdMegabarCycle = 0;
  newStdCycle3BassAssignment = null;
  newStdMegabarHeat.fill(0);
  newStdPrevBassHit = 0; newStdPrevTrebleHit = 0;
  for (let i = 0; i < NEWSTD_NUM_FLOODS; i++) newStdFloodIsB[i] = i % 2 === 0;
  newStdFloodBrightness.fill(0.15);
  newStdNextSwapMs = -1;
  newStdChinaFadeFrom.fill(null); newStdChinaFadeStartMs = -1e9; newStdLastChinaColor.fill(null);
  newStdMegabarFadeFrom.fill(null); newStdMegabarFadeStartMs = -1e9; newStdLastMegabarColor.fill(null);
}
// BUGFIX HISTORY ("floods hella dim, compare EQ vs SpeedStripes"): a first
// attempt at this fix applied a flat 65% brightness FLOOR to every hit-
// driven fill below (hitBrightnessByte(), since removed) -- fixed the
// average brightness, but compressed away the dark/light contrast a hit
// needs to read as reactive at all, causing a real regression (megabars/
// china stopped visibly responding to individual hits). Fixed for real in
// AudioBoard.getHitPct() itself instead -- during an actual hit it now
// reports flat max (100%) rather than the literal analog level, so quiet
// moments stay genuinely dark while hits always read as a full flash.
function newStdColors() {
  // (1-cos(x))/2 naturally spends more real TIME near 0 and 1 (where its
  // own derivative is smallest) than near 0.5 -- exactly "slowing down
  // near each end so every value gets about equal time," with no separate
  // easing curve needed on top of a plain oscillator.
  const hueFrac = (1 - Math.cos(2*Math.PI*newStdHueTimeS/NEWSTD_HUE_PERIOD_S)) / 2;
  const satFrac = (1 - Math.cos(2*Math.PI*newStdSatTimeS/NEWSTD_SAT_PERIOD_S)) / 2;
  const satByte = Math.round((75 + 25*satFrac) / 100 * 255);
  const bHue = Math.round(NEWSTD_HUE_RED + (NEWSTD_HUE_YELLOW - NEWSTD_HUE_RED) * hueFrac);
  return { Bcolor: hsv(bHue, satByte, 255), Tcolor: hsv(NEWSTD_HUE_TREBLE, satByte, 255) };
}
function showEqualizerNewStandard(nowMs, dtMs) {
  clearRope(); clearChinas(); clearMegabars();
  const dtSec = Math.max(0, Math.min(0.2, dtMs / 1000));
  const silent = AudioBoard.silent;
  newStdSatTimeS += dtSec;
  if (!silent) newStdHueTimeS += dtSec;
  const { Bcolor, Tcolor } = newStdColors();
  const bassHit = AudioBoard.getHitPct(BAND_LOW), trebleHit = AudioBoard.getHitPct(BAND_HIGH);

  // ROPE: the default variation's single traveling segment, plus 3 more
  // evenly spaced (90 = a quarter of the loop) apart, all in Bcolor.
  const chasePos = (nowMs / 1000 * 60) % NUM_NEO;
  const quarter = NUM_NEO / 4;
  for (let i = 0; i < NUM_NEO; i++) {
    let nearest = Infinity;
    for (let s = 0; s < 4; s++) {
      const segPos = (chasePos + s * quarter) % NUM_NEO;
      const d = Math.min(Math.abs(i - segPos), NUM_NEO - Math.abs(i - segPos));
      if (d < nearest) nearest = d;
    }
    ropeLeds[i] = blend({r:0,g:0,b:0}, Bcolor, Math.max(0, 255 - nearest * 4));
  }

  if (silent) {
    // Silence mode entirely replaces the sound-reactive cycling below --
    // see showEqSilenceFloods() (shared with pixel_war and, for their own
    // base colors, Chase/VU meter too).
    showEqSilenceFloods(nowMs, dtMs, Bcolor, Tcolor);
    for (let m = 0; m < NUM_MEGABAR; m++) newStdLastMegabarColor[m] = { r: megabarLeds[m].r, g: megabarLeds[m].g, b: megabarLeds[m].b };
    for (let c = 0; c < NUM_CHINA; c++) newStdLastChinaColor[c] = { r: chinaLeds[c].r, g: chinaLeds[c].g, b: chinaLeds[c].b };
    return;
  }

  // CHINA: 4 cycles, 8s each, auto-advancing. First-ever call starts ON
  // cycle 0 (not already advanced past it) -- fresh entry into the show
  // shouldn't skip straight to cycle 1.
  if (newStdChinaCycleStart < 0) {
    newStdChinaCycleStart = nowMs;
  } else if (nowMs - newStdChinaCycleStart >= 8000) {
    newStdChinaCycleStart = nowMs;
    newStdChinaCycle = (newStdChinaCycle + 1) % 4;
    for (let c = 0; c < NUM_CHINA; c++) newStdChinaFadeFrom[c] = newStdLastChinaColor[c] || { r: 0, g: 0, b: 0 };
    newStdChinaFadeStartMs = nowMs;
  }
  const bassChina = scale255(Bcolor, bassHit/100*255), trebleChina = scale255(Tcolor, trebleHit/100*255);
  if (newStdChinaCycle === 0) {
    for (const c of FLAME_CHINA_FRONTBACK) chinaLeds[c] = bassChina;
    for (const c of FLAME_CHINA_SIDES) chinaLeds[c] = trebleChina;
  } else if (newStdChinaCycle === 1) {
    for (let c = 0; c < NUM_CHINA; c++) chinaLeds[c] = bassChina;
  } else { // 2 and 3 both read as "all treble" per spec (3 = opposite of all-bass cycle 2, 4 = all treble -- same result)
    for (let c = 0; c < NUM_CHINA; c++) chinaLeds[c] = trebleChina;
  }
  if (nowMs - newStdChinaFadeStartMs < NEWSTD_FADE_MS) {
    const fadeF = Math.round(255 * Math.max(0, nowMs - newStdChinaFadeStartMs) / NEWSTD_FADE_MS);
    for (let c = 0; c < NUM_CHINA; c++) chinaLeds[c] = blend(newStdChinaFadeFrom[c], chinaLeds[c], fadeF);
  }
  for (let c = 0; c < NUM_CHINA; c++) newStdLastChinaColor[c] = { r: chinaLeds[c].r, g: chinaLeds[c].g, b: chinaLeds[c].b };

  // MEGABARS: 3 cycles, 8s each, auto-advancing. Same "start ON cycle 0"
  // fix as china above -- also seeds cycle 0's own randomized state on
  // first entry, which previously only ever ran via the increment path.
  if (newStdMegabarCycleStart < 0) {
    newStdMegabarCycleStart = nowMs;
    newStdCycle1TrebleIsMod3 = Math.random() < 0.5;
  } else if (nowMs - newStdMegabarCycleStart >= 8000) {
    newStdMegabarCycleStart = nowMs;
    newStdMegabarCycle = (newStdMegabarCycle + 1) % 3;
    if (newStdMegabarCycle === 0) newStdCycle1TrebleIsMod3 = Math.random() < 0.5;
    if (newStdMegabarCycle === 1) newStdCycle2FrontRearIsBass = Math.random() < 0.5;
    if (newStdMegabarCycle === 2) {
      newStdCycle3BassAssignment = new Array(NUM_MEGABAR).fill(false).map(() => Math.random() < 0.5);
      newStdMegabarHeat.fill(0);
    }
    for (let m = 0; m < NUM_MEGABAR; m++) newStdMegabarFadeFrom[m] = newStdLastMegabarColor[m] || { r: 0, g: 0, b: 0 };
    newStdMegabarFadeStartMs = nowMs;
  }
  const bassMb = scale255(Bcolor, bassHit/100*255), trebleMb = scale255(Tcolor, trebleHit/100*255);
  if (newStdMegabarCycle === 0) {
    for (let m = 0; m < NUM_MEGABAR; m++) {
      const isMod3 = (m % 3 === 0);
      megabarLeds[m] = (isMod3 === newStdCycle1TrebleIsMod3) ? trebleMb : bassMb;
    }
  } else if (newStdMegabarCycle === 1) {
    const swapNum = Math.floor((nowMs - newStdMegabarCycleStart) / 2000); // every 1/4 of 8s = 2s
    const frontRearIsBass = (swapNum % 2 === 0) ? newStdCycle2FrontRearIsBass : !newStdCycle2FrontRearIsBass;
    for (const m of NEWSTD_FRONT_REAR_MEGABARS) megabarLeds[m] = frontRearIsBass ? bassMb : trebleMb;
    for (const m of NEWSTD_LEFT_RIGHT_MEGABARS) megabarLeds[m] = frontRearIsBass ? trebleMb : bassMb;
  } else {
    // Cycle 3: fixed half/half bass-treble assignment; each hit lights 2
    // currently-black (heat<=0) megabars in their own assigned color,
    // falling back to the 2 currently-dimmest if none are fully black
    // (clarified) -- heat decays back toward black over ~400ms.
    const bassEdge = bassHit > 20 && newStdPrevBassHit <= 20;
    const trebleEdge = trebleHit > 20 && newStdPrevTrebleHit <= 20;
    newStdPrevBassHit = bassHit; newStdPrevTrebleHit = trebleHit;
    if (bassEdge || trebleEdge) {
      const black = [], order = Array.from({length:NUM_MEGABAR}, (_,m)=>m);
      for (const m of order) if (newStdMegabarHeat[m] <= 0.02) black.push(m);
      let pick;
      if (black.length >= 2) {
        pick = [];
        const pool = black.slice();
        for (let n = 0; n < 2; n++) pick.push(pool.splice(Math.floor(Math.random()*pool.length), 1)[0]);
      } else {
        pick = order.slice().sort((a,b) => newStdMegabarHeat[a]-newStdMegabarHeat[b]).slice(0, 2);
      }
      for (const m of pick) newStdMegabarHeat[m] = 1.0;
    }
    for (let m = 0; m < NUM_MEGABAR; m++) {
      newStdMegabarHeat[m] = Math.max(0, newStdMegabarHeat[m] - dtSec / 0.4);
      const base = newStdCycle3BassAssignment[m] ? Bcolor : Tcolor;
      megabarLeds[m] = scale255(base, newStdMegabarHeat[m] * 255);
    }
  }
  if (nowMs - newStdMegabarFadeStartMs < NEWSTD_FADE_MS) {
    const fadeF = Math.round(255 * Math.max(0, nowMs - newStdMegabarFadeStartMs) / NEWSTD_FADE_MS);
    for (let m = 0; m < NUM_MEGABAR; m++) megabarLeds[m] = blend(newStdMegabarFadeFrom[m], megabarLeds[m], fadeF);
  }
  for (let m = 0; m < NUM_MEGABAR; m++) newStdLastMegabarColor[m] = { r: megabarLeds[m].r, g: megabarLeds[m].g, b: megabarLeds[m].b };
}

/* ---- Equalizer variation: pixel_war --------------------------------------
   Shares new_standard's Bcolor/Tcolor palette and silence behavior. The
   rope becomes two colored "territories" (Bcolor/Tcolor) that grow and
   shrink based on which frequency is hitting more, slowly rotating around
   the loop via a random walk.

   BUGFIX (rope war "does nothing"): the original continuous tug-of-war
   here (push proportional to hit LEVEL every frame, weak constant recover)
   reached its territory cap within about 1 REAL second under sustained
   one-sided hits and then sat there perfectly static for as long as that
   side kept dominating -- verified via a headless-Chrome test holding
   bass at max for 9s straight: pwBassFrac hit 0.875 by t=990ms and never
   moved again through t=8910ms. Since real music re-triggers the same
   side repeatedly, this meant the war effectively settled almost
   immediately and then read as inert for the rest of the show -- exactly
   "does nothing." Replaced with a discrete per-hit IMPULSE model (each
   qualifying hit edge nudges the territory a fixed amount, closer to the
   original spec's per-hit spring idea) plus a much stronger continuous
   recovery pulling back to center between hits, so the boundary keeps
   visibly swinging back and forth rather than snapping to an extreme and
   parking there. */
const PW_MIN_FRAC = 0.125, PW_MAX_FRAC = 0.875; // 12.5%..87.5% of the loop, per spec
const PW_FADE_FRAC = 1 / 14; // border cross-fade width, fixed at the spec's stated max/default (C/14)
const PW_HIT_IMPULSE = 0.12;  // territory fraction jumped per qualifying hit edge
const PW_RECOVER_RATE = 0.35; // fraction/sec pulled back toward center between hits -- fast enough that the war stays visibly contested instead of freezing at the cap
const PW_WALK_MAX = 150;      // pix(LED)/s, per spec
let pwBassFrac = 0.5;         // Bcolor's fraction of the loop; Tcolor gets the rest
let pwRotationOffsetLed = 0, pwWalkspeed = 0, pwWalkAccumMs = 0;
let pwMegabarHeat = new Array(NUM_MEGABAR).fill(0);
let pwMegabarColorIsB = new Array(NUM_MEGABAR).fill(true);
let pwLastPicked = [];
let pwPrevBassHit = 0, pwPrevTrebleHit = 0;
// CHINA (per explicit re-spec): the 8 chinas are grouped into 4 pairs, each
// lamp paired with the one one address above it (0&1, 2&3, 4&5, 6&7). 2
// pairs are randomly assigned to bass, 2 to treble at reset; every 4 hit
// edges (either band), a random pair is grabbed and swapped with a random
// pair currently on the opposite band, keeping the 2/2 split.
const PW_CHINA_PAIRS = [[0, 1], [2, 3], [4, 5], [6, 7]];
let pwChinaPairIsBass = [true, true, false, false];
let pwChinaHitCount = 0;
function resetPixelWarState() {
  pwBassFrac = 0.5;
  pwRotationOffsetLed = 0; pwWalkspeed = 0; pwWalkAccumMs = 0;
  pwMegabarHeat.fill(0);
  pwLastPicked = [];
  pwPrevBassHit = 0; pwPrevTrebleHit = 0;
  const idxs = [0, 1, 2, 3];
  for (let i = idxs.length - 1; i > 0; i--) { const j = Math.floor(Math.random() * (i + 1)); [idxs[i], idxs[j]] = [idxs[j], idxs[i]]; }
  pwChinaPairIsBass = [false, false, false, false];
  pwChinaPairIsBass[idxs[0]] = true; pwChinaPairIsBass[idxs[1]] = true;
  pwChinaHitCount = 0;
}
// Random walk exactly per spec's cadence (a fresh kick every 50ms) --
// simplified the accel formula itself (see above), since as literally
// written it scales accel BY current walkspeed, which stalls forever at
// walkspeed=0 and never actually starts walking; a plain independent
// random kick each tick is what a "random walk ranging +/-150 pix/s"
// actually needs to move at all.
function pwStepRandomWalk(dtMs) {
  pwWalkAccumMs += dtMs;
  while (pwWalkAccumMs >= 50) {
    pwWalkAccumMs -= 50;
    const accel = (Math.random() * 20 - 10) * 3;
    pwWalkspeed = Math.max(-PW_WALK_MAX, Math.min(PW_WALK_MAX, pwWalkspeed + accel));
    pwRotationOffsetLed += pwWalkspeed * 0.05; // 50ms of travel at the new speed
  }
}
function pwColorAt(i, Bcolor, Tcolor) {
  const normPos = ((((i + pwRotationOffsetLed) % NUM_NEO) + NUM_NEO) % NUM_NEO) / NUM_NEO;
  const halfFade = PW_FADE_FRAC / 2;
  let dWrap = Math.min(normPos, 1 - normPos);       // distance to the 0/1 wrap boundary (Tcolor meets Bcolor)
  let dMid = Math.abs(normPos - pwBassFrac);
  dMid = Math.min(dMid, 1 - dMid);                   // distance to the Bcolor/Tcolor boundary at pwBassFrac
  const inB = normPos < pwBassFrac;
  const nearestD = Math.min(dWrap, dMid);
  if (nearestD >= halfFade) return inB ? Bcolor : Tcolor;
  const f = 0.5 - 0.5 * (nearestD / halfFade); // 0.5 at the border, 0 at the fade's own edge
  return inB ? blend(Bcolor, Tcolor, Math.round(f * 255)) : blend(Tcolor, Bcolor, Math.round(f * 255));
}
function showEqualizerPixelWar(nowMs, dtMs) {
  clearRope(); clearChinas(); clearMegabars();
  const dtSec = Math.max(0, Math.min(0.2, dtMs / 1000));
  const silent = AudioBoard.silent;
  newStdSatTimeS += dtSec;
  if (!silent) newStdHueTimeS += dtSec;
  const { Bcolor, Tcolor } = newStdColors();
  const bassHit = AudioBoard.getHitPct(BAND_LOW), trebleHit = AudioBoard.getHitPct(BAND_HIGH);

  if (silent) {
    // same silence behavior as new_standard -- see showEqSilenceFloods()
    showEqSilenceFloods(nowMs, dtMs, Bcolor, Tcolor);
    return;
  }

  // Hit-edge detection, shared by the rope push, megabar picking, and china
  // pair swapping below.
  const bassEdge = bassHit > 20 && pwPrevBassHit <= 20;
  const trebleEdge = trebleHit > 20 && pwPrevTrebleHit <= 20;
  pwPrevBassHit = bassHit; pwPrevTrebleHit = trebleHit;

  // ROPE: two territories fighting over the loop, slowly rotating. Each
  // qualifying hit edge gives its side a discrete push; continuous recovery
  // pulls the boundary back toward center the rest of the time -- see the
  // BUGFIX comment above this function for why it's edge-triggered rather
  // than level-continuous.
  pwStepRandomWalk(dtMs);
  if (bassEdge) pwBassFrac += PW_HIT_IMPULSE;
  if (trebleEdge) pwBassFrac -= PW_HIT_IMPULSE;
  pwBassFrac += (0.5 - pwBassFrac) * PW_RECOVER_RATE * dtSec;
  pwBassFrac = Math.max(PW_MIN_FRAC, Math.min(PW_MAX_FRAC, pwBassFrac));
  for (let i = 0; i < NUM_NEO; i++) ropeLeds[i] = pwColorAt(i, Bcolor, Tcolor);

  // MEGABARS: on every hit, 2 random megabars (excluding whichever 2 the
  // previous hit picked, and any still mid-flash) light up in that hit's
  // own color, then fade.
  if (bassEdge || trebleEdge) {
    const isB = bassEdge; // if both happened to edge the same frame, bass wins the tie
    let eligible = [];
    for (let m = 0; m < NUM_MEGABAR; m++) {
      if (pwMegabarHeat[m] > 0.02) continue;
      if (pwLastPicked.includes(m)) continue;
      eligible.push(m);
    }
    if (eligible.length < 2) eligible = Array.from({length:NUM_MEGABAR}, (_,m)=>m).filter(m => pwMegabarHeat[m] <= 0.02);
    const picked = [];
    const pool = eligible.slice();
    for (let n = 0; n < 2 && pool.length; n++) picked.push(pool.splice(Math.floor(Math.random() * pool.length), 1)[0]);
    for (const m of picked) { pwMegabarHeat[m] = 1.0; pwMegabarColorIsB[m] = isB; }
    pwLastPicked = picked;
  }
  for (let m = 0; m < NUM_MEGABAR; m++) {
    pwMegabarHeat[m] = Math.max(0, pwMegabarHeat[m] - dtSec / 0.4);
    const base = pwMegabarColorIsB[m] ? Bcolor : Tcolor;
    megabarLeds[m] = scale255(base, pwMegabarHeat[m] * 255);
  }

  // CHINA: 4 pairs, 2 assigned to bass and 2 to treble (see PW_CHINA_PAIRS
  // comment above) -- each pair shows its assigned band's color, brightness
  // tracking that band's own hit level live (a little local VU meter per
  // pair). Every 4 hit edges, a random pair swaps bands with a random pair
  // currently on the other band, keeping the 2/2 split.
  if (bassEdge) pwChinaHitCount++;
  if (trebleEdge) pwChinaHitCount++;
  if (pwChinaHitCount >= 4) {
    pwChinaHitCount -= 4;
    const grabbed = Math.floor(Math.random() * 4);
    const opposite = [0, 1, 2, 3].filter(p => pwChinaPairIsBass[p] !== pwChinaPairIsBass[grabbed]);
    if (opposite.length) {
      const other = opposite[Math.floor(Math.random() * opposite.length)];
      const tmp = pwChinaPairIsBass[grabbed];
      pwChinaPairIsBass[grabbed] = pwChinaPairIsBass[other];
      pwChinaPairIsBass[other] = tmp;
    }
  }
  for (let p = 0; p < 4; p++) {
    const isBass = pwChinaPairIsBass[p];
    const hitPct = isBass ? bassHit : trebleHit;
    const lit = scale255(isBass ? Bcolor : Tcolor, Math.max(0, Math.min(100, hitPct)) / 100 * 255);
    for (const c of PW_CHINA_PAIRS[p]) chinaLeds[c] = lit;
  }
}

/* ---- SpeedStripesShow: ground-fixed stripes -----------------------------
   Rewritten per explicit spec (replaces the earlier LED-index/phase-based
   version entirely). Modeled directly in real-world feet, car-relative:
   origin at the car's own center, Y increasing toward the FRONT (note:
   this is the OPPOSITE sign from MEGABAR_POS_FT's internal feet, which
   uses Y=back+ to mirror canvas-pixel Y directly -- kept local to this
   show rather than flipping that shared convention everywhere else).

   Mechanism: the ground has infinite stripes, each SS_STRIPE_WIDTH_FT
   wide, of a random-but-STABLE color. A stripe's leading edge (larger Y)
   and trailing edge (leading edge - SS_STRIPE_WIDTH_FT, which is also the
   next stripe's leading edge) both descend together at the current
   vehicle speed, ft/s (the ground is fixed; the car driving forward makes
   the pattern slide backward in the car's own reference frame). The
   very-first stripe's leading edge starts (on show entry/reset) level
   with the largest Y among all floodlight ground spots -- i.e. even with
   the frontmost throw. Every fixture (megabar, china, and each rope LED
   by its own car-relative position) shows whichever stripe's leading edge
   has most recently descended past its own Y, cross-fading linearly into
   the adjacent stripe's color over SS_FADE_FT feet centered on the
   boundary -- same fade width for rope and every flood, so a boundary
   reads as one continuous transition sweeping across the whole car, not
   just along the rope. */
const SS_STRIPE_WIDTH_FT = 13; // trailing edge Y = leading edge Y - this
const SS_FADE_FT = 3; // stripe-to-stripe cross-fade width, feet -- rope and all floods alike

// Car-relative feet, Y+ = front -- see block comment above.
function canvasToCarFt(pt) { return { x: (pt.x - CAR_X) / PX_PER_FT, y: (CAR_Y - pt.y) / PX_PER_FT }; }
// Ground-spot center for a fixture: mount position + aim direction *
// centerDist, same formula drawGroundEllipse()/drawWash() use to place
// the actual rendered glow (kept as its own small helper here rather than
// refactoring those, to avoid touching proven rendering code).
function groundSpotPx(originPx, azimuthDeg, wash) {
  const a = (azimuthDeg - 90) * Math.PI / 180;
  const d = wash.centerDist * PX_PER_FT;
  return { x: originPx.x + Math.cos(a) * d, y: originPx.y + Math.sin(a) * d };
}
// Ground-spot centers for every megabar/china, car-relative feet --
// computed once (fixture positions/washes are static), reused every frame.
const SS_MEGABAR_SPOT_FT = Array.from({ length: NUM_MEGABAR }, (_, m) => {
  const wash = m === HEADLIGHT_INDEX ? MEGABAR_FRONT_WASH : MEGABAR_WASH;
  return canvasToCarFt(groundSpotPx(megabarPos(m), megabarAngle(m), wash));
});
const SS_CHINA_SPOT_FT = Array.from({ length: NUM_CHINA }, (_, c) =>
  canvasToCarFt(groundSpotPx(CHINA_POS[c], CHINA_AIM_DEG[c], CHINA_WASH_FOR[c]))
);
// Rope LED positions, car-relative feet -- an idealized SHARP-CORNERED
// rectangle (not the rope's own rounded-corner cosmetic path from
// perimeterPoint()/ledPoint()): the LEDs sit in a simple orthogonal
// arrangement and this show only needs each one's real along-the-car
// position. Index order/direction matches ledPoint() (front L->R, right
// side F->B, back R->L, left side B->F). Computed once.
const SS_CAR_HALF_W_FT = CAR_W_FT / 2, SS_CAR_HALF_H_FT = CAR_H_FT / 2;
function ssLedCarFt(i) {
  if (i < RIGHT) { const u = i / SIZEOF_SMALL_NEO; return { x: -SS_CAR_HALF_W_FT + u * CAR_W_FT, y: SS_CAR_HALF_H_FT }; }
  if (i < REAR) { const u = (i - RIGHT) / SIZEOF_LARGE_NEO; return { x: SS_CAR_HALF_W_FT, y: SS_CAR_HALF_H_FT - u * CAR_H_FT }; }
  if (i < LEFT) { const u = (i - REAR) / SIZEOF_SMALL_NEO; return { x: SS_CAR_HALF_W_FT - u * CAR_W_FT, y: -SS_CAR_HALF_H_FT }; }
  const u = (i - LEFT) / SIZEOF_LARGE_NEO; return { x: -SS_CAR_HALF_W_FT, y: -SS_CAR_HALF_H_FT + u * CAR_H_FT };
}
const SS_LED_Y_FT = Array.from({ length: NUM_NEO }, (_, i) => ssLedCarFt(i).y);

let ssRefEdgeY = 0; // current Y of the k=0 stripe's leading edge
function resetSpeedStripesState() {
  ssRefEdgeY = Math.max(...SS_MEGABAR_SPOT_FT.map(p => p.y), ...SS_CHINA_SPOT_FT.map(p => p.y));
  resetZebraSchedule();
}
// Deterministic pseudo-random color per stripe index (stable forever for a
// given k, so a physical stripe keeps its color as it scrolls) -- same
// imul/xorshift hash technique this show's earlier version used. Negative
// k works fine (Math.imul/|0 both handle signed 32-bit input).
function ssStripeColor(k) {
  let h = Math.imul(k | 0, 2654435761) >>> 0;
  h = (h ^ (h >>> 15)) >>> 0;
  return hsv(h & 0xFF, 255, 255);
}
// Generic periodic-stripe cross-fade: given a stripe width and a
// colorForIndex(k) picker, returns whichever stripe index y's continuous
// position currently falls in, cross-fading into the neighboring index
// over fadeFt feet centered on the boundary. fadeFt: 0 (or omitted) for a
// hard-edged solid color. Factored out of ssColorAt() so zebra's own
// (differently-colored, differently-spaced) hue stripes and black-stripe
// overlay can reuse the exact same, already-correct boundary-fade math
// instead of a second hand-rolled copy of it.
function periodicStripeColorAt(y, stripeWidthFt, fadeFt, colorForIndex) {
  const contPos = (ssRefEdgeY - y) / stripeWidthFt;
  const k = Math.floor(contPos);
  if (!fadeFt) return colorForIndex(k);
  const frac = contPos - k;
  const distToBoundaryFt = Math.min(frac, 1 - frac) * stripeWidthFt;
  const halfFade = fadeFt / 2;
  if (distToBoundaryFt >= halfFade) return colorForIndex(k);
  if (frac < 0.5) { // near the leading edge -- blending in from the previous stripe
    const f = 0.5 + 0.5 * (distToBoundaryFt / halfFade);
    return blend(colorForIndex(k - 1), colorForIndex(k), Math.round(f * 255));
  }
  // near the trailing edge -- blending out toward the next stripe back
  const f = 0.5 * (1 - distToBoundaryFt / halfFade);
  return blend(colorForIndex(k), colorForIndex(k + 1), Math.round(f * 255));
}
// fadeFt: 0 (or omitted) for a hard-edged solid stripe color; >0 to
// linearly cross-fade between the two adjacent stripes' colors over that
// many total feet, centered on whichever boundary is nearest.
function ssColorAt(y, fadeFt) {
  return periodicStripeColorAt(y, SS_STRIPE_WIDTH_FT, fadeFt, ssStripeColor);
}

/* ---- Zebra variation: deterministic rainbow-hue stripes (not the random-
   hash stripes above), speed-desaturated, with an optional periodic black-
   stripe overlay and flood-only sound reactivity. Reuses ssRefEdgeY (the
   same scrolling position all SpeedStripes variations share) and
   periodicStripeColorAt() above, just with its own stripe width/coloring. */
const ZEBRA_STRIPE_WIDTH_FT = 6;
const ZEBRA_HUE_STEP = 11; // hue units (0-255) per stripe, consistent increase -- ~23 stripes (~207ft) per full rainbow cycle
// stopped = 100% saturated, 25mph = 60% saturated, linear between; holds at
// 60% past 25mph (no further instruction past that point, so it doesn't
// desaturate away to nothing at highway speed).
function zebraSatFraction() {
  const t = Math.max(0, Math.min(25, Ctl.speedMph)) / 25;
  return 1 - 0.4 * t;
}
function zebraStripeColor(k) {
  const hue = (((k * ZEBRA_HUE_STEP) % 256) + 256) % 256;
  return hsv(hue, Math.round(zebraSatFraction() * 255), 255);
}
// Black stripes: pot sets their width, 0ft (off) to 40ft; the gap between
// consecutive black stripes is always 2x that width, so the occurrence
// period (one black stripe's start to the next) is always exactly 3x the
// black width -- wider stripes come proportionally less often, rather than
// width and spacing scaling independently. Per request this rides on top
// of the hue progression as a pure overlay (computed from the same
// absolute scroll position, not from the hue stripe index k), so cutting
// in a black stripe never shifts or pauses the rainbow's own advance.
//
// BUGFIX: this used to compute the black/colored cell purely from the
// CURRENT pot value at whatever position was being queried -- so turning
// the pot retroactively rewrote every stripe still in view (already-placed
// black stripes would shrink/grow/move as you kept adjusting), not just
// newly-encountered ground ahead of the car. Replaced with an explicit,
// incrementally-extended schedule: each cell, once generated (its host
// position has come within reach of any fixture), permanently keeps the
// width it was given at that moment -- turning the pot only changes what
// gets generated for territory that hasn't been reached yet.
//
// pos = y - ssRefEdgeY: monotonically increasing for a fixed car-relative
// y as time passes (ssRefEdgeY only ever decreases), so it's a stable
// "how far the car has driven past this point's own generation" coordinate
// -- exactly the timeline the schedule is built along.
const ZEBRA_SCHEDULE_LOOKAHEAD_FT = 30; // comfortably past any fixture's own |y|
let zebraSchedule = [];      // [{startPos, endPos, width, isBlack}, ...] ascending by startPos
let zebraFrontierPos = null; // largest pos already generated
function resetZebraSchedule() {
  zebraSchedule = [];
  zebraFrontierPos = null;
}
const ZEBRA_MIN_BLACK_WIDTH_FT = 0.5; // smallest black-stripe width the pot will ever produce, once it's producing any at all
// BUGFIX: pot used to map linearly straight to width (0-40ft), and any
// result under 0.5ft got treated as "off" -- so the whole 0%-1.25% range
// of the pot all landed on the identical "no black stripes" result, a
// dead zone doing nothing before the effect ever kicked in. Remapped so
// pot=0% is the ONLY off point and any turn above that immediately jumps
// to the practical minimum (0.5ft) and scales continuously from there to
// 40ft at 100%, so every bit of pot travel above 0 does something.
function zebraEnsureScheduleTo(neededPos) {
  if (zebraFrontierPos === null) zebraFrontierPos = neededPos;
  const potWidth = Ctl.pot <= 0 ? 0 : ZEBRA_MIN_BLACK_WIDTH_FT + (Ctl.pot / 100) * (40 - ZEBRA_MIN_BLACK_WIDTH_FT); // ft -- read once per newly-generated stretch, not per query
  while (zebraFrontierPos < neededPos) {
    if (potWidth < ZEBRA_MIN_BLACK_WIDTH_FT) { zebraFrontierPos = neededPos; break; } // no black stripes right now -- nothing to schedule, stays colored by default
    const lastWasBlack = zebraSchedule.length ? zebraSchedule[zebraSchedule.length - 1].isBlack : false;
    const isBlack = !lastWasBlack;
    const width = isBlack ? potWidth : 2 * potWidth;
    const start = zebraFrontierPos, end = start + width;
    zebraSchedule.push({ startPos: start, endPos: end, width, isBlack });
    zebraFrontierPos = end;
  }
  // prune well-behind entries so the schedule doesn't grow unbounded over a long session
  const pruneBefore = neededPos - 2 * ZEBRA_SCHEDULE_LOOKAHEAD_FT;
  while (zebraSchedule.length > 2 && zebraSchedule[0].endPos < pruneBefore) zebraSchedule.shift();
}
function zebraBlackFractionAt(pos) {
  const halfFade = SS_FADE_FT / 2;
  for (let i = zebraSchedule.length - 1; i >= 0; i--) {
    const s = zebraSchedule[i];
    if (pos < s.startPos || pos >= s.endPos) continue;
    const distStart = pos - s.startPos, distEnd = s.endPos - pos;
    const dist = Math.min(distStart, distEnd);
    const target = s.isBlack ? 1 : 0;
    if (dist >= halfFade) return target;
    const nearStart = distStart < distEnd;
    const neighbor = nearStart ? zebraSchedule[i - 1] : zebraSchedule[i + 1];
    const neighborTarget = neighbor ? (neighbor.isBlack ? 1 : 0) : (s.isBlack ? 0 : 1); // no neighbor generated yet -- assume the opposite (matches the alternating pattern)
    const f = 0.5 - 0.5 * (dist / halfFade); // 0.5 exactly at the border, 0 at the fade's own edge
    return target + (neighborTarget - target) * f;
  }
  return 0; // outside the generated range entirely (shouldn't normally happen) -- default colored
}
function zebraBlackFraction(y) {
  const pos = y - ssRefEdgeY;
  zebraEnsureScheduleTo(pos + ZEBRA_SCHEDULE_LOOKAHEAD_FT);
  return zebraBlackFractionAt(pos);
}
// China-only (megabars are excluded per request -- always global max, see
// below) sound-reactive brightness: full global max after 10s+ of silence,
// otherwise 75% of max (raised from 65%), except bass hits push it up to
// their own level (as a fraction of max), never below that 75% resting
// floor.
const ZEBRA_CHINA_REST_FRACTION = 0.75;
let ssZebraLastSoundMs = 0;
function zebraFloodBrightnessFraction(nowMs) {
  if (!AudioBoard.silent) ssZebraLastSoundMs = nowMs;
  if (nowMs - ssZebraLastSoundMs >= 10000) return 1.0;
  const bassFrac = AudioBoard.getHitPct(BAND_LOW) / 100;
  return Math.max(ZEBRA_CHINA_REST_FRACTION, bassFrac);
}
function zebraColorAt(y) {
  const hueColor = periodicStripeColorAt(y, ZEBRA_STRIPE_WIDTH_FT, SS_FADE_FT, zebraStripeColor);
  const blackFrac = zebraBlackFraction(y);
  return blackFrac > 0 ? blend(hueColor, { r: 0, g: 0, b: 0 }, Math.round(blackFrac * 255)) : hueColor;
}

function showSpeedStripes(nowMs, dtMs) {
  const dtSec = Math.max(0, Math.min(0.2, dtMs / 1000));
  ssRefEdgeY -= Ctl.speedMph * 1.4667 * dtSec; // 1.4667 ft/s per mph, same conversion the ground texture scroll uses

  clearRope(); clearMegabars(); clearChinas();
  if (Ctl.variation === 2) { // zebra
    // Megabars: always global max, excluded from sound reactivity entirely
    // per request (china alone carries the zebra's audio response now).
    for (let m = 0; m < NUM_MEGABAR; m++) {
      megabarLeds[m] = zebraColorAt(SS_MEGABAR_SPOT_FT[m].y);
    }
    for (let i = 0; i < NUM_NEO; i++) {
      ropeLeds[i] = zebraColorAt(SS_LED_Y_FT[i]); // rope isn't sound-reactive here, per request ("floods" only)
    }
    const chinaBrightness = zebraFloodBrightnessFraction(nowMs);
    for (let c = 0; c < NUM_CHINA; c++) {
      chinaLeds[c] = scale255(zebraColorAt(SS_CHINA_SPOT_FT[c].y), chinaBrightness * 255);
    }
    return;
  }
  for (let m = 0; m < NUM_MEGABAR; m++) {
    megabarLeds[m] = ssColorAt(SS_MEGABAR_SPOT_FT[m].y, SS_FADE_FT);
  }
  for (let i = 0; i < NUM_NEO; i++) {
    ropeLeds[i] = ssColorAt(SS_LED_Y_FT[i], SS_FADE_FT);
  }
  if (Ctl.variation === 1) { // 2nd variation: china lit by the exact same algorithm
    for (let c = 0; c < NUM_CHINA; c++) {
      chinaLeds[c] = ssColorAt(SS_CHINA_SPOT_FT[c].y, SS_FADE_FT);
    }
  }
}

// Persistent state, integrated incrementally each frame (angle += vel*dt)
// -- NOT recomputed from elapsed-time-since-show-start, which was the bug:
// with angle = vel*t, changing the pot (which changes vel) instantly
// reflows the ENTIRE elapsed history through the new velocity, snapping
// the beams to a different position the moment you touch the pot. This
// matches how the real firmware does it (angle1_ = wrap360(angle1_ +
// vel1_*dtSec), see LighthouseShow.h) -- current position always carries
// forward continuously; only the RATE changes when the pot moves.
// lhAngle2 seeds at 180 (not 0), matching the real firmware
// (LighthouseShow.h: angle1_=0, angle2_=180) -- BUGFIX: previously both
// started at the same 0deg, so with the pot down near 0 (rotation frozen)
// both beam-pairs land on the exact same position and pickOverlapColor's
// "brightest wins, exact ties broken randomly" logic flickers between
// their two hues every single frame -- reads as "one beam, stuck, flashing
// colors like a lunatic," confirmed to reproduce exactly this way.
let lhAngle1 = 0, lhAngle2 = 180, lhHue1 = 0;
let lhVel1 = 72, lhVel2 = -180; // deg/s, current post-random-walk value
const lhVelWalk1 = makeRandomWalk(), lhVelWalk2 = makeRandomWalk(), lhHueRateWalk = makeRandomWalk(), lhSatWalk = makeRandomWalk();
function resetLighthouseState() {
  lhAngle1 = 0; lhAngle2 = 180; lhHue1 = 0;
  lhHueRateWalk.initialized = false;
  lhVel1 = 72; lhVel2 = -180;
  // seed each beam's initial rotation speed directly (randomWalkValue()
  // would otherwise auto-init both to the range midpoint, 0deg/s) -- beam 1
  // starts clockwise at 5s/rotation (72deg/s), beam 2 counterclockwise at
  // 2s/rotation (-180deg/s), matching LighthouseShow.h's start(). From here
  // both drift normally via the random walk.
  const nowMs0 = performance.now();
  lhVelWalk1.initialized = true; lhVelWalk1.rampStart = lhVelWalk1.rampTarget = 72; lhVelWalk1.tickT0 = nowMs0; lhVelWalk1.tickDur = 800 + Math.random() * 400;
  lhVelWalk2.initialized = true; lhVelWalk2.rampStart = lhVelWalk2.rampTarget = -180; lhVelWalk2.tickT0 = nowMs0; lhVelWalk2.tickDur = 800 + Math.random() * 400;
  // seed saturation's start explicitly too (randomWalkValue() would
  // otherwise auto-init to the new range's midpoint, 85%, not the
  // requested 80%), matching LighthouseShow.h's start()
  lhSatWalk.initialized = true; lhSatWalk.rampStart = lhSatWalk.rampTarget = 0.80; lhSatWalk.tickT0 = nowMs0; lhSatWalk.tickDur = 800 + Math.random() * 400;
  lhStrobeActive = false; lhHadHit = false; lhSuppressionPeak = 0; lhStrobeLastMs = 0;
}

// China bass-strobe: same triple-pulse timing/suppression as EqualizerShow's
// strobe (real firmware, BumpingAudioShow.h) -- borrowed here since real
// Lighthouse currently leaves china off entirely. Bass hits only.
let lhStrobeActive = false, lhStrobeT0 = 0;
let lhHadHit = false, lhLastHitT = 0, lhSuppressionPeak = 0, lhStrobeLastMs = 0;
// DEVIATION from real firmware: BumpingAudioShow.h's suppressionPeak_ never
// decays -- it's only cleared by a full 3s gap with NO qualifying bass hits
// at all. Under continuous music that gap essentially never happens, so
// after the first loud hit (or two) sets a high peak, it silently latches
// there and the strobe stops firing for the rest of the show except on a
// rare new loudest-ever hit -- exactly the "works once, then never again
// except after a lull" behavior reported as broken. Decaying the peak back
// down over time lets moderate hits clear it again periodically, so it
// keeps reacting through a whole show instead of firing once and going
// quiet, while still avoiding a strobe on literally every single hit.
const LH_SUPPRESSION_DECAY_PER_SEC = 60; // percentage points/sec
function updateLighthouseStrobe(nowMs) {
  const dtSec = Math.max(0, Math.min(0.5, (nowMs - lhStrobeLastMs) / 1000));
  lhStrobeLastMs = nowMs;
  lhSuppressionPeak = Math.max(0, lhSuppressionPeak - LH_SUPPRESSION_DECAY_PER_SEC * dtSec);
  const bassHit = AudioBoard.getHitPct(BAND_LOW);
  const isHit = bassHit > 0;
  if (isHit) {
    const silenceExpired = !lhHadHit || (nowMs - lhLastHitT) >= 3000;
    const exceedsPeak = bassHit > lhSuppressionPeak;
    if (!lhStrobeActive && (silenceExpired || exceedsPeak)) {
      lhStrobeActive = true; lhStrobeT0 = nowMs; lhSuppressionPeak = bassHit;
    }
    lhHadHit = true; lhLastHitT = nowMs;
  }
  if (!lhStrobeActive) return false;
  const onMs = 30, gapMs = 20;
  const pulseStart = [0, onMs+gapMs, 2*(onMs+gapMs)];
  const elapsed = nowMs - lhStrobeT0;
  if (elapsed >= pulseStart[2] + onMs) { lhStrobeActive = false; return false; }
  for (let p = 0; p < 3; p++) if (elapsed >= pulseStart[p] && elapsed < pulseStart[p]+onMs) return true;
  return false;
}

// Cone width: sized so at least 2 real megabars always fall within the
// guaranteed core at ANY beam angle -- see LighthouseShow.h's
// CONE_HALF_WIDTH_DEG comment for the full derivation (computed from
// CarpetGeometry's real megabar ground-spot angles, not nominal 30deg
// spacing; exact worst-case threshold 32.4855deg, +1.5deg margin -> 34).
// ONE shared shape for rope/megabar/china alike, same as firmware.
const LH_CONE_HALF_WIDTH_DEG = 34, LH_CONE_FULL_WIDTH_DEG = LH_CONE_HALF_WIDTH_DEG * 2, LH_CONE_FADE_DEG = LH_CONE_FULL_WIDTH_DEG * 0.25;
const LH_CORE_FLOOR_BRIGHTNESS = 128; // brightness at the guaranteed core's own outer edge -- never flat, never black within the guarantee
const LH_MIN_BEAM_SEPARATION_DEG = LH_CONE_FULL_WIDTH_DEG + 2; // keeps the 2 beams' cores from ever touching -- see LighthouseShow.h
function lhCircDelta(a, b) { let d = (b - a + 180) % 360; if (d < 0) d += 360; return d - 180; }
// Smooth ease 255->0 across t=0..1, flat tangent at both ends -- matches
// LighthouseShow.h's smoothstep8() shape (here in plain float, not
// integer-quantized, since this JS renderer has no FPU-avoidance need).
function lhSmoothstep(t) { const x = 1 - t; return x * x * (3 - 2 * x); }
// Per spec: full brightness at dead center, easing (not flat) down to
// LH_CORE_FLOOR_BRIGHTNESS at the guaranteed core's own edge, then
// continuing to ease from that floor down to black over the further fade
// zone -- gives the 2 always-guaranteed megabars on each side of a beam a
// smooth, analog brightness change as the beam sweeps past, and keeps the
// worst-case-farthest guaranteed megabar always clearly lit (never black)
// within the guarantee itself. See LighthouseShow.h's coneBrightnessAt().
function lhConeBrightnessAt(d) {
  if (d >= LH_CONE_HALF_WIDTH_DEG + LH_CONE_FADE_DEG) return 0;
  if (d <= LH_CONE_HALF_WIDTH_DEG) {
    const eased = lhSmoothstep(d / LH_CONE_HALF_WIDTH_DEG); // 255-ish(1)->0 as d 0->half
    return Math.round(LH_CORE_FLOOR_BRIGHTNESS + eased * (255 - LH_CORE_FLOOR_BRIGHTNESS));
  }
  const eased = lhSmoothstep((d - LH_CONE_HALF_WIDTH_DEG) / LH_CONE_FADE_DEG);
  return Math.round(eased * LH_CORE_FLOOR_BRIGHTNESS);
}
function lhBeamBrightnessAt(pointAngle, beamAngle) {
  const d1 = Math.abs(lhCircDelta(pointAngle, beamAngle));
  const d2 = Math.abs(lhCircDelta(pointAngle, wrap360(beamAngle + 180)));
  return lhConeBrightnessAt(Math.min(d1, d2));
}
// BUGFIX (per explicit correction -- "no additive mixing, that looks
// terrible"): used to sum both beams' hues together wherever their
// falloffs overlapped (pickOverlapColor), reading as a muddy blend rather
// than either beam's real color. Now picks the single brighter beam
// entirely, with no blending between the two. preferBeam2OnTie: an EXACT
// brightness tie is real and non-negligible for megabars specifically
// (brightness is rounded to an 8-bit level, not a razor-thin float
// coincidence) -- callers iterating the fixed megabar set alternate this
// per-fixture so a tie never systematically favors the same beam, per
// explicit request ("one should light for each color always" on a tie).
// See LighthouseShow.h's winnerColorAt().
function lhWinnerColorAt(pointAngle, angle1, hue1, angle2, hue2, satByte, preferBeam2OnTie) {
  const b1 = lhBeamBrightnessAt(pointAngle, angle1);
  const b2 = lhBeamBrightnessAt(pointAngle, angle2);
  const winBright = Math.max(b1, b2);
  if (winBright === 0) return { r: 0, g: 0, b: 0 };
  const beam1Wins = b1 === b2 ? !preferBeam2OnTie : b1 > b2;
  return hsv(beam1Wins ? hue1 : hue2, satByte, winBright);
}
function showLighthouse(nowMs, dtMs) {
  clearRope(); clearMegabars(); clearChinas();
  const dtSec = Math.max(0, Math.min(0.2, dtMs / 1000)); // clamp huge gaps (tab backgrounded, etc.)
  const energyFrac = Ctl.pot / 100;

  // random-walked rotation velocity/hue-rate/saturation -- matches
  // LighthouseShow.h exactly (this used to be a fixed vel=72*energyFrac
  // formula with no random walk at all, a real divergence from firmware).
  const VEL_CEIL = 360;
  const rawVel1 = randomWalkValue(lhVelWalk1, nowMs, -VEL_CEIL, VEL_CEIL, 10);
  const rawVel2 = randomWalkValue(lhVelWalk2, nowMs, -VEL_CEIL, VEL_CEIL, 10);
  lhVel1 = rawVel1 * energyFrac; lhVel2 = rawVel2 * energyFrac;
  const MAX_HUE_RATE = 255 / 20;
  const hueRate = randomWalkValue(lhHueRateWalk, nowMs, 0, MAX_HUE_RATE, MAX_HUE_RATE * 0.1);
  const satFraction = randomWalkValue(lhSatWalk, nowMs, 0.70, 1.0, 0.013);

  lhAngle1 = wrap360(lhAngle1 + lhVel1 * dtSec);
  lhAngle2 = wrap360(lhAngle2 + lhVel2 * dtSec);

  // Prevent the 2 beams' cones from ever overlapping -- see
  // LighthouseShow.h's MIN_BEAM_SEPARATION_DEG comment. Both beams' random-
  // walked velocities are free to share sign (both CW or both CCW), so
  // they can drift arbitrarily close; when they'd cross the minimum
  // effective separation (reduced 0..90 by each beam's own 180deg
  // antipodal symmetry), push them apart symmetrically by the shortfall
  // instead -- a soft per-frame "bounce", not a velocity restriction.
  {
    const rawDelta = lhCircDelta(lhAngle1, lhAngle2); // -180..180, signed, angle1->angle2
    const absDelta = Math.abs(rawDelta);
    const effSep = Math.min(absDelta, 180 - absDelta); // 0..90
    if (effSep < LH_MIN_BEAM_SEPARATION_DEG) {
      const shortfall = LH_MIN_BEAM_SEPARATION_DEG - effSep;
      const sign = rawDelta >= 0 ? 1 : -1;
      const pushDelta = absDelta <= 90 ? sign * shortfall : -sign * shortfall;
      lhAngle1 = wrap360(lhAngle1 - pushDelta * 0.5);
      lhAngle2 = wrap360(lhAngle2 + pushDelta * 0.5);
    }
  }

  lhHue1 = (lhHue1 + hueRate * dtSec) % 256;
  const angle1 = lhAngle1, angle2 = lhAngle2;
  const hue1 = Math.round(lhHue1), hue2 = (hue1 + 128) % 256;
  const satByte = Math.round(satFraction * 255);

  // Rope: evaluated at each LED's own real angle, from CarpetGeometry (the
  // project's shared centralized geometry source) -- no more separate
  // per-show angle derivation.
  for (let i = 0; i < NUM_NEO; i++) {
    const a = CarpetGeometry.getNeoGeom(i).angleFromForwardDeg;
    ropeLeds[i] = lhWinnerColorAt(a, angle1, hue1, angle2, hue2, satByte, false);
  }
  // Megabars: SAME falloff function as the rope, at each megabar's own
  // real position angle (CarpetGeometry). Alternates the exact-tie
  // tie-break by megabar index -- see lhWinnerColorAt()'s own comment.
  for (let m = 0; m < NUM_MEGABAR; m++) {
    const a = CarpetGeometry.getMegabarPosition(m).positionAngleDeg; // same field china uses below -- numerically == beamAngleDeg for megabars, but positionAngleDeg is the semantically correct one to ask for here
    megabarLeds[m] = lhWinnerColorAt(a, angle1, hue1, angle2, hue2, satByte, (m % 2) !== 0);
  }

  // China: both variations now share the same beam-crossing effect as
  // rope/megabar (at each china's own POSITION angle, not its aim
  // direction -- see CarpetGeometry's own china caveat comment) -- per
  // explicit request. Default additionally layers a bass-hit white
  // strobe on top (below).
  for (let c = 0; c < NUM_CHINA; c++) {
    const a = CarpetGeometry.getChinaPosition(c).positionAngleDeg;
    chinaLeds[c] = lhWinnerColorAt(a, angle1, hue1, angle2, hue2, satByte, false);
  }

  // bass-hit china strobe -- Default only (No Strobe drops it per its own
  // name/request). APPROXIMATION: real firmware drives this on china's
  // independent White channel (RGBW hardware) so it superimposes on top
  // of the beam-crossing RGB color without touching it at all -- this JS
  // renderer's chinaLeds[] models only 3 channels (no W), so there's no
  // exact equivalent. Closest available: an additive (not overwriting)
  // brighten-toward-white, capped well below full saturation (150, a
  // deliberately-chosen visualizer approximation, not a real W-diode
  // calibration) so the underlying beam color still visibly shows through
  // -- the qualitative "superimposed, not replaced" effect, even though
  // this can't be a precise stand-in for genuinely independent channels.
  if (Ctl.variation === 0 && updateLighthouseStrobe(nowMs)) {
    const STROBE_ADD = 150;
    for (let c = 0; c < NUM_CHINA; c++) {
      const cur = chinaLeds[c];
      chinaLeds[c] = { r: Math.min(255, cur.r + STROBE_ADD), g: Math.min(255, cur.g + STROBE_ADD), b: Math.min(255, cur.b + STROBE_ADD) };
    }
  }
}

/* ---- audio config screens ---- */
// lights a 10-LED window sliding from backCornerIdx (0%) to frontCornerIdx
// (100%) as percent varies, flat color -- ported from MagicCarpet.h's
// renderSideIndicator(). Was entirely missing from the visualizer before
// (a real gap, not a rendering-style choice): the side strips showed
// nothing at all during this screen.
function renderSideIndicatorWindow(backCornerIdx, frontCornerIdx, percent, color) {
  const segmentLen = Math.abs(frontCornerIdx - backCornerIdx) + 1;
  const windowWidth = 10;
  const direction = frontCornerIdx > backCornerIdx ? 1 : -1;
  const t = Math.max(0, Math.min(100, percent)) / 100;
  const maxOffset = segmentLen - windowWidth;
  const windowOffset = Math.round(t * maxOffset);
  for (let k = 0; k < windowWidth; k++) {
    ropeLeds[backCornerIdx + direction * (windowOffset + k)] = color;
  }
}
// Draws the noise-floor (blue) and peak-threshold (red) windows on one side
// strip together, so they can nudge apart when close -- BUGFIX (ported to
// firmware too, MagicCarpet::renderSideIndicatorPair()): drawing these as
// two independent renderSideIndicatorWindow() calls meant that whenever
// they were close enough for their 10-LED windows to overlap (up to fully
// coincident when set equal), the red one (drawn second) silently painted
// over some or all of the blue one, making it invisible ("segments
// overlap when one is set equal to the other" bug report). Since noise
// floor is cross-clamped elsewhere to never exceed peak threshold, the
// windows can only ever collide from below -- nudge them apart
// symmetrically (clamped to the strip's own ends) whenever they'd
// overlap, so both stay fully visible side by side.
function renderSideIndicatorPair(backCornerIdx, frontCornerIdx, noiseFloorPercent, peakThresholdPercent) {
  const segmentLen = Math.abs(frontCornerIdx - backCornerIdx) + 1;
  const windowWidth = 10;
  const maxOffset = segmentLen - windowWidth;
  const offsetFor = (pct) => Math.round(Math.max(0, Math.min(100, pct)) / 100 * maxOffset);
  let noiseOffset = offsetFor(noiseFloorPercent);
  let peakOffset = offsetFor(peakThresholdPercent);
  const gap = peakOffset - noiseOffset;
  if (gap < windowWidth) {
    const deficit = windowWidth - gap;
    const pushDown = Math.floor(deficit / 2), pushUp = deficit - pushDown;
    noiseOffset = Math.max(0, noiseOffset - pushDown);
    peakOffset = Math.min(maxOffset, peakOffset + pushUp);
    if (peakOffset - noiseOffset < windowWidth) { // one side hit its bound -- compensate on the other
      if (noiseOffset === 0) peakOffset = Math.min(maxOffset, windowWidth);
      else if (peakOffset === maxOffset) noiseOffset = Math.max(0, maxOffset - windowWidth);
    }
  }
  const direction = frontCornerIdx > backCornerIdx ? 1 : -1;
  for (let k = 0; k < windowWidth; k++) ropeLeds[backCornerIdx + direction * (noiseOffset + k)] = { r: 0, g: 0, b: 255 };
  for (let k = 0; k < windowWidth; k++) ropeLeds[backCornerIdx + direction * (peakOffset + k)] = { r: 255, g: 0, b: 0 };
}
// per-china edge-detect state for cfgMeterScreen()'s peak-threshold flash
// -- see its own comment below for what this fixes.
const cfgMeterChinaWasAbove = new Array(7).fill(false);
const cfgMeterChinaFlashUntil = new Array(7).fill(0);
// If no real hit has landed in the last 1.5s (e.g. dialing in noise floor/
// peak threshold with no audio source playing), synthesize a steady series
// of typical-width hits at 3/sec so there's still SOMETHING to look at
// while adjusting -- display-only, never touches AudioBoard/FW state, just
// a local stand-in signal for this one screen's own rendering.
let cfgMeterLastRealHitMs = -99999;
function cfgMeterFeedbackHitPct(nowMs) {
  const realHit = AudioBoard.getHitPct(BAND_FULL);
  if (realHit > 0) cfgMeterLastRealHitMs = nowMs;
  if (nowMs - cfgMeterLastRealHitMs <= 1500) return realHit;
  const PERIOD_MS = 1000 / 3; // 3 fake hits/sec
  const WIDTH_FRAC = 0.35;    // "typical" pulse width within each period
  const phase = (nowMs % PERIOD_MS) / PERIOD_MS;
  return phase < WIDTH_FRAC ? Math.round(100 * (1 - phase / WIDTH_FRAC)) : 0;
}
function cfgMeterScreen(nowMs) {
  clearRope(); clearChinas(); clearMegabars();
  // Inset from BOTH outer ends of the 156-LED front string (ported to
  // firmware too) -- per request, the outer ends (nearest each front
  // corner) are hidden under the wings when they're up, so a meter using
  // the whole string was partly invisible then. PLACEHOLDER: 20 LEDs/side
  // is a guess (~1.5ft on the front channel's ~13 LEDs/ft), not a
  // measured value -- adjust to whatever the wings actually cover.
  const wingInset = 20;
  const usableLen = SIZEOF_SMALL_NEO - 2 * wingInset;
  const segLen = Math.floor(usableLen / 3);
  for (let i = 0; i < wingInset; i++) ropeLeds[i] = { r: 0, g: 0, b: 0 };
  for (let i = SIZEOF_SMALL_NEO - wingInset; i < SIZEOF_SMALL_NEO; i++) ropeLeds[i] = { r: 0, g: 0, b: 0 };
  const levels = [AudioBoard.getNormalPct(BAND_HIGH), AudioBoard.getNormalPct(BAND_MID), AudioBoard.getNormalPct(BAND_LOW)];
  for (let seg = 0; seg < 3; seg++) {
    const filled = Math.round(levels[seg]/100 * segLen);
    for (let p = 0; p < segLen; p++) {
      const i = wingInset + seg*segLen + p;
      if (p < filled) ropeLeds[i] = { r: 255, g: 255, b: 255 };
      else { const hue = Math.round(96 - 96*p/(segLen-1)); ropeLeds[i] = hsv(hue,255,77); } // 77 = 128 * 0.6 (rainbow background dimmed 40% per request)
    }
  }
  // BUGFIX: this screen is reused for BOTH the noise-floor/peak-threshold
  // subsetting (0) AND the auto-gain subsetting (4) -- see
  // AUDIO_SUB_SCREEN -- but never actually branched on which one is
  // active, always drawing the noise/peak dots and windows regardless.
  // Firmware's showAudioMeter() only draws those while
  // !isAutoGainSubsetting, showing a plain white on/off snap window on the
  // side strips instead while on subsetting 4 -- ported that distinction
  // here, since without it the Auto-Gain subsetting was visually
  // indistinguishable from Noise Floor/Peak Threshold (part of why it read
  // as "missing from config" -- reaching it, once that navigation bug
  // above is fixed, still wouldn't have shown anything different).
  const isAutoGainSubsetting = Ctl.appMode === 'audio' && Ctl.configSubsetting === 4;
  if (isAutoGainSubsetting) {
    const snapPct = AudioBoard.agcMode / 2 * 100; // 0%=off, 50%=band, 100%=full -- 3-position snap, matches firmware's showAudioMeter()
    renderSideIndicatorWindow(VU_BACK_RIGHT, VU_FRONT_RIGHT, snapPct, { r: 255, g: 255, b: 255 });
    renderSideIndicatorWindow(VU_BACK_LEFT, VU_FRONT_LEFT, snapPct, { r: 255, g: 255, b: 255 });
  } else {
    // noise floor (blue) / peak threshold (red) dots, overlaid on each of the
    // 3 front segments at that value's own position along the segment --
    // previously missing entirely.
    const noiseFloorPct = AudioBoard.noiseFloorPct, peakThreshPct = AudioBoard.peakThresholdPct;
    let noiseFloorPos = Math.round(noiseFloorPct/100*(segLen-1));
    let peakThreshPos = Math.round(peakThreshPct/100*(segLen-1));
    if (noiseFloorPos === peakThreshPos) { if (noiseFloorPos > 0) noiseFloorPos--; else peakThreshPos++; }
    for (let seg = 0; seg < 3; seg++) {
      ropeLeds[wingInset + seg*segLen + noiseFloorPos] = { r: 0, g: 0, b: 255 };
      ropeLeds[wingInset + seg*segLen + peakThreshPos] = { r: 255, g: 0, b: 0 };
    }
    // side strips: sliding indicator windows, true corner to true corner --
    // previously missing entirely, the actual bug behind "I don't see that at all."
    renderSideIndicatorPair(VU_BACK_RIGHT, VU_FRONT_RIGHT, noiseFloorPct, peakThreshPct);
    renderSideIndicatorPair(VU_BACK_LEFT, VU_FRONT_LEFT, noiseFloorPct, peakThreshPct);
  }
  // per-bin china confirmation (7 of 8 fixtures, one raw frequency bin each)
  //
  // BUGFIX: per README, each fixture is supposed to flash solid white for
  // 40ms, edge-triggered, the instant its own bin rises above the live
  // peak threshold ("floods don't respect the peak thresh setting" bug
  // report) -- this flash was never implemented at all, so china here
  // never visibly reacted to the peak-threshold value regardless of where
  // it was set. Edge-detected (not level-triggered) so a sustained loud
  // bin flashes once on crossing, not solid white the whole time it stays
  // above threshold.
  const peakRaw = Math.round(AudioBoard.peakThresholdPct / 100 * 255);
  for (let c = 0; c < 7; c++) {
    const bin = (AudioBoard.bins && AudioBoard.bins[c]) || 0;
    const above = bin > peakRaw;
    if (above && !cfgMeterChinaWasAbove[c]) cfgMeterChinaFlashUntil[c] = nowMs + 40;
    cfgMeterChinaWasAbove[c] = above;
    chinaLeds[c] = (nowMs < cfgMeterChinaFlashUntil[c]) ? { r: 255, g: 255, b: 255 } : { r: 0, g: 0, b: bin };
  }
  // Megabars pulse with the full-range HIT level (not bass, and not a
  // steady level) so they actually demonstrate what these thresholds do to
  // real hit detection -- falls back to a synthesized series of typical
  // hits if no real audio has landed in the last 1.5s, see
  // cfgMeterFeedbackHitPct().
  const full = cfgMeterFeedbackHitPct(nowMs) / 100 * 255;
  for (let m = 0; m < NUM_MEGABAR; m++) megabarLeds[m] = { r: 0, g: 0, b: full };
}
function cfgDecayOrPredictScreen() {
  clearRope(); clearChinas(); clearMegabars();
  const segBand = [BAND_HIGH, BAND_MID, BAND_LOW];
  const segColor = [{r:0,g:0,b:255},{r:0,g:255,b:0},{r:255,g:0,b:0}];
  const segLen = Math.floor(SIZEOF_SMALL_NEO/3);
  for (let seg = 0; seg < 3; seg++) {
    const hit = AudioBoard.getHitPct(segBand[seg])/100 * segLen;
    for (let p = 0; p < segLen; p++) {
      ropeLeds[seg*segLen+p] = p < hit ? segColor[seg] : {r:0,g:0,b:0};
    }
  }
  for (let m = 0; m < NUM_MEGABAR; m++) {
    const which = m % 3;
    const hitRaw = AudioBoard.getHitPct(segBand[which])/100*255;
    megabarLeds[m] = scale255(segColor[which], Math.max(hitRaw, 38));
  }
}
function cfgForesightScreen() {
  for (let i = 0; i < NUM_NEO; i++) ropeLeds[i] = { r: 0, g: 0, b: 40 };
  clearChinas();
  const segBand = [BAND_HIGH, BAND_MID, BAND_LOW];
  const segColor = [{r:0,g:0,b:255},{r:0,g:255,b:0},{r:255,g:0,b:0}];
  for (let m = 0; m < NUM_MEGABAR; m++) {
    const which = m % 3;
    const hitRaw = AudioBoard.getHitPct(segBand[which])/100*255;
    megabarLeds[m] = scale255(segColor[which], Math.max(hitRaw, 38));
  }
}

// Global sound-reactivity toggle screen -- matches MagicCarpet.h's
// showSoundReactivityToggle() exactly: chinas play a fake, steady series
// of "hits" as direct feedback for whichever state is currently live, so
// disabling it shows its own consequence directly (chinas doing nothing)
// rather than just a label. Rope/megabars stay off. Not audio-driven at
// all (deliberately fake/simulated), so it reads identically regardless
// of whatever's actually playing.
function cfgReactivityScreen(nowMs) {
  clearRope(); clearMegabars(); clearChinas();
  const PERIOD_MS = 600, ON_MS = 120;
  const lit = AudioBoard.soundReactivityEnabled && (nowMs % PERIOD_MS) < ON_MS;
  for (let c = 0; c < NUM_CHINA; c++) chinaLeds[c] = lit ? { r: 255, g: 255, b: 255 } : { r: 0, g: 0, b: 0 };
}

// Auto-peak mode screen (SubAudioAutoPeak) -- mirrors MagicCarpet::
// showAutoPeakToggle() exactly: 3-state now (AutoPeakOff/AutoPeakFull/
// AutoPeakBin), front edge's own neos (indices [0, SIZEOF_SMALL_NEO),
// matching the real FW's ropeLeds[] index range for this same physical
// strip) split into 3 equal thirds, one lit per mode value.
function cfgAutoPeakScreen() {
  clearRope(); clearMegabars(); clearChinas();
  const mode = AudioBoard.autoPeakMode;
  const segLen = Math.floor(SIZEOF_SMALL_NEO / 3);
  const start = mode * segLen;
  const end = mode >= 2 ? SIZEOF_SMALL_NEO : start + segLen;
  for (let i = start; i < end; i++) ropeLeds[i] = { r: 255, g: 255, b: 255 };
}

// Brightness config screens: solid white preview across whichever fixture
// group the current subsetting controls, so moving the pot shows its effect
// directly (the global/china brightness ceiling applied below then scales
// it same as any real show).
function brightnessScreen() {
  clearRope(); clearMegabars(); clearChinas();
  const white = { r: 255, g: 255, b: 255 };
  if (Ctl.configSubsetting === 0) {
    for (let i = 0; i < NUM_NEO; i++) ropeLeds[i] = white;
    for (let m = 0; m < NUM_MEGABAR; m++) megabarLeds[m] = white;
    for (let c = 0; c < NUM_CHINA; c++) chinaLeds[c] = white;
  } else if (Ctl.configSubsetting === 1) {
    // All OTHER megabars shown at full max (a fixed, unambiguous reference)
    // so the live-adjusted headlight can be compared directly against it --
    // previously left fully off (via the clearMegabars() above), making
    // that comparison impossible. HEADLIGHT_INDEX gets overwritten right
    // after by the same live-adjusted white every other subsetting uses,
    // then dimmed by the live headlightBr% down in renderCurrent()'s hf
    // scaling, same as during normal show playback.
    for (let m = 0; m < NUM_MEGABAR; m++) megabarLeds[m] = white;
  } else {
    for (let c = 0; c < NUM_CHINA; c++) chinaLeds[c] = white;
  }
}
// PowerTest: a power-saving experiment -- front half of the car shows a
// direct HSV->RGB conversion of the test color, back half pulls the shared
// low channel out into an (unmodeled here) white element by simply
// subtracting it, so the two halves should read as roughly the same color
// at a glance despite different underlying channel mixes.
// per README: left half (straight HSV->RGB) = china [2,3,4,5] (front-left +
// back-left pairs); right half (RGBW power-saving translation) = china
// [0,1,6,7] (front-right + back-right pairs) -- NOT a simple first-4/
// last-4 split, since the pairs wrap around index 0.
const POWERTEST_CHINA_LEFT = [2, 3, 4, 5], POWERTEST_CHINA_RIGHT = [0, 1, 6, 7];
function powerTestScreen() {
  clearMegabars();
  const color = hsv(Ctl.testHue, Ctl.testSat, Ctl.testBrightness);
  const w = Math.min(color.r, color.g, color.b);
  const backHalf = { r: color.r - w, g: color.g - w, b: color.b - w };
  // left/right determined by each LED's actual physical X position (not an
  // index range) -- the true split includes the left side run PLUS the
  // left half of both the front and back edges, not a contiguous index span.
  for (let i = 0; i < NUM_NEO; i++) ropeLeds[i] = (ledPoint(i).x < CAR_X) ? color : backHalf;
  for (const c of POWERTEST_CHINA_LEFT) chinaLeds[c] = color;
  for (const c of POWERTEST_CHINA_RIGHT) chinaLeds[c] = backHalf;
}
// Interface-role-only renderer -- a deliberate, approved exception to the
// zero-duplication mandate (claude_dev_prompts.md prompt 6): re-implements
// show/config-screen rendering in JS rather than streaming real LED bytes
// over the radio link, for bandwidth. NEVER called when visualizerRole
// is 'dev' -- see renderCurrent() above. Must be re-ported from the real
// FW whenever the corresponding src/*Show.h / MagicCarpet::showXXX()
// logic changes (see claude_dev_prompts.md prompt 6's "Interface role"
// paragraph) -- it does not update itself.
function renderInterfaceMode(nowMs, dtMs) {
  switch (Ctl.currMode) {
    case 'nightrider': showNightrider(nowMs, dtMs); break;
    case 'flame': showFlame(nowMs, dtMs); break;
    case 'equalizer': showEqualizer(nowMs, dtMs); break;
    case 'speedstripes': showSpeedStripes(nowMs, dtMs); break;
    case 'lighthouse': showLighthouse(nowMs, dtMs); break;
    case 'cfg_meter': cfgMeterScreen(nowMs); break;
    case 'cfg_decay': cfgDecayOrPredictScreen(); break;
    case 'cfg_predict': cfgDecayOrPredictScreen(); break;
    case 'cfg_foresight': cfgForesightScreen(); break;
    case 'cfg_reactivity': cfgReactivityScreen(nowMs); break;
    case 'cfg_autopeak': cfgAutoPeakScreen(); break;
    case 'brightness': brightnessScreen(); break;
    case 'powertest': powerTestScreen(); break;
  }
  // global/china brightness ceiling (linear scale, no gamma -- matches
  // applyBrightnessCeiling()'s own linear-only approach)
  //
  // BUGFIX: this used to mutate each array slot's color object IN PLACE
  // (p.r*=gf etc). Many shows assign the SAME shared color object
  // reference to multiple array slots (e.g. Lighthouse's `megabarLeds[m] =
  // (m%3===0) ? mbHigh : mbLow` -- mbHigh/mbLow are each ONE object reused
  // across several indices), which is harmless on its own, but in-place
  // mutation here meant that shared object got scaled by gf/hf/cf ONCE PER
  // ALIASED SLOT, not once total -- e.g. a color shared across all 12
  // megabars got hit with gf 12 TIMES over (compounding: gf=0.75 -> 0.75^12
  // = ~3%, gf=0.5 -> 0.5^12 = ~0.02%), reading as fully black at any
  // globalBr below 100%. Fixed by writing a freshly-scaled object to each
  // slot instead of mutating whatever object happens to be there, so
  // aliased slots each get scaled exactly once regardless of sharing
  // upstream.
  const gf = Ctl.globalBr / 100;
  for (let i = 0; i < ropeLeds.length; i++) ropeLeds[i] = scale255(ropeLeds[i], gf * 255);
  for (let i = 0; i < megabarLeds.length; i++) megabarLeds[i] = scale255(megabarLeds[i], gf * 255);
  const hf = Ctl.headlightBr / 100;
  megabarLeds[HEADLIGHT_INDEX] = scale255(megabarLeds[HEADLIGHT_INDEX], hf * 255);
  const cf = Ctl.chinaBr / 100 * gf;
  for (let i = 0; i < chinaLeds.length; i++) chinaLeds[i] = scale255(chinaLeds[i], cf * 255);
  if (Ctl.blacklight) {
    // Real hardware's blacklight is a genuine UV LED channel (full on/off,
    // see MagicCarpet::setBlacklight()) with no RGB equivalent to render
    // literally -- this purple-ish minimum-floor is a visualizer-only
    // stand-in. Dimmed 25% per request (visualizer-cosmetic only, doesn't
    // correspond to any real hardware brightness change since the real
    // channel has no brightness dial).
    for (let i = 0; i < chinaLeds.length; i++) {
      const p = chinaLeds[i];
      chinaLeds[i] = { r: Math.max(p.r, 110 * 0.75), g: Math.max(p.g, 0), b: Math.max(p.b, 255 * 0.75) };
    }
  }
}
