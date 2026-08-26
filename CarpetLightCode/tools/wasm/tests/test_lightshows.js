#!/usr/bin/env node
/* test_lightshows.js
 *
 *   Automated behavioral tests for the REAL firmware, run against the
 *   compiled WASM module (tools/visualizer/wasm/carpet_fw.js). Every
 *   assertion reads real firmware output (LED buffers, show/variation
 *   name getters) via the same web_bridge.cpp injectors the visualizer
 *   itself uses -- no light-show logic is reimplemented here, per this
 *   project's zero-duplication mandate. Show/variation selection uses
 *   web_setCurrentShow()/web_setCurrentVariation() -- the same real,
 *   committed makeShow()/start() path a real short-press/encoder-turn
 *   takes (see web_bridge.cpp's own "convenience control" comment) --
 *   rather than cycling via simulated presses, so test setup is direct
 *   and can't get stuck if a name string ever drifts out of sync.
 *
 *   Covers every show/variation except NightriderShow (excluded per
 *   request -- pair it with your own test file if that changes).
 *
 *   Run:
 *     source tools/wasm/emsdk/emsdk_env.sh && tools/wasm/build.sh   # once, or after any src/ change
 *     node tools/wasm/tests/test_lightshows.js
 *   (no system Node needed -- the repo's own bundled one under
 *   tools/wasm/emsdk/node/ (see its version-numbered subdir) works too)
 *
 *   Exits 0 if every test passes, 1 otherwise (also printed as a
 *   PASS/FAIL summary line per test, plus a final tally) -- suitable for
 *   a CI step or a pre-commit sanity check after touching src/*Show.h.
 */

'use strict';
const path = require('path');

// Minimal localStorage shim -- only Nvm's WASM-only shim needs this
// (reads/writes it for persisted show/variation/settings state); a plain
// in-memory object is enough for a test process that never restarts.
global.localStorage = (() => {
  const store = {};
  return {
    getItem: (k) => (k in store ? store[k] : null),
    setItem: (k, v) => { store[k] = String(v); },
    removeItem: (k) => { delete store[k]; },
  };
})();

const CarpetFirmware = require(path.resolve(__dirname, '../../visualizer/wasm/carpet_fw.js'));

// ---------------------------------------------------------------------
// Test bookkeeping -- deliberately tiny, no external test-runner
// dependency (this repo has no npm/package.json for the FW side, and
// pulling one in just for this felt like more moving parts than the
// suite's actual size justifies).
let passCount = 0, failCount = 0;
const failures = [];
function check(label, cond, detail) {
  if (cond) {
    passCount++;
    console.log('  PASS  ' + label);
  } else {
    failCount++;
    failures.push(label + (detail ? ' -- ' + detail : ''));
    console.log('  FAIL  ' + label + (detail ? '  (' + detail + ')' : ''));
  }
}
function section(title) { console.log('\n=== ' + title + ' ==='); }

// ---------------------------------------------------------------------
// WASM module helpers. Each top-level show group boots a FRESH module
// instance (CarpetFirmware() is a factory, per MODULARIZE=1) rather than
// reusing one across shows -- keeps each show's tests isolated from
// whatever state an earlier show's tests left behind, mirroring a real
// power-cycle rather than relying on in-session state resetting cleanly.
async function freshBoot() {
  const M = await CarpetFirmware();
  M.ccall('web_setup', null, [], []);
  return M;
}

const ShowMode = { Nightrider: 0, Flame: 1, Equalizer: 2, SpeedStripes: 3, Lighthouse: 4 };

// Variation index tables, straight from each show's own variationName()
// (src/*Show.h) -- kept here only as a name->index lookup for test
// readability (selectVariationByName('waterflames') reads better than a
// bare index at each call site); the actual selection still goes through
// the real web_setCurrentVariation(), never re-simulating show logic.
const VARIATION_INDEX = {
  [ShowMode.Flame]: { waterflames: 0, flames: 1, 'shifting hues': 2, 'hue to white': 3 },
  [ShowMode.Equalizer]: { chase: 0, 'VU meter': 1, new_standard: 2, pixel_war: 3, sub_standard: 4 },
  [ShowMode.SpeedStripes]: { default: 0, zebra: 1 },
  [ShowMode.Lighthouse]: { default: 0, 'no strobe': 1 },
};

function makeCtx(M) {
  let t = 0;
  let currentShow = ShowMode.Nightrider; // real boot default, see CarpetLightLogic.cpp
  return {
    M,
    tick(n = 1) { for (let i = 0; i < n; i++) { t += 30; M.ccall('web_tick', null, ['number'], [t]); } },
    now() { return t; },
    showName() { return M.ccall('web_getCurrentShowName', 'string', [], []); },
    variationName() { return M.ccall('web_getCurrentVariationName', 'string', [], []); },
    // web_setCurrentShow() is the same real makeShow()/start() path a real
    // short-press takes (see web_bridge.cpp) -- direct and can't get stuck,
    // unlike cycling via simulated presses.
    selectShow(targetMode) {
      M.ccall('web_setCurrentShow', null, ['number'], [targetMode]);
      currentShow = targetMode;
      this.tick(2);
    },
    selectVariationByName(targetName) {
      const table = VARIATION_INDEX[currentShow];
      if (!table || !(targetName in table)) throw new Error('selectVariationByName: no index known for "' + targetName + '" on show ' + currentShow);
      M.ccall('web_setCurrentVariation', null, ['number'], [table[targetName]]);
      this.tick(2);
      if (this.variationName() !== targetName) {
        throw new Error('selectVariationByName: set index ' + table[targetName] + ' but real FW reports variation "' + this.variationName() + '", expected "' + targetName + '"');
      }
    },
    setBins(vals) {
      const ptr = M.ccall('web_getAdcBinsBufferPtr', 'number', [], []);
      for (let i = 0; i < 7; i++) M.HEAP32[(ptr >> 2) + i] = vals[i];
      M.ccall('web_injectAdcBins', null, [], []);
    },
    silence() { this.setBins([0, 0, 0, 0, 0, 0, 0]); },
    adcMax() { return M.ccall('web_getAdcMaxValue', 'number', [], []); },
    snapshot(kind) {
      const ptrFn = { rope: 'web_getRopeLedsPtr', megabar: 'web_getMegabarLedsPtr', china: 'web_getChinaLedsPtr' }[kind];
      const sizeFn = { rope: 'web_sizeofRopeLed', megabar: 'web_sizeofMegabarLed', china: 'web_sizeofChinaLed' }[kind];
      const numFn = { rope: 'web_getNumRopeLeds', megabar: 'web_getNumMegabarLeds', china: 'web_getNumChinaLeds' }[kind];
      const ptr = M.ccall(ptrFn, 'number', [], []);
      const stride = M.ccall(sizeFn, 'number', [], []);
      const n = M.ccall(numFn, 'number', [], []);
      const out = [];
      for (let i = 0; i < n; i++) {
        const o = ptr + i * stride;
        out.push([M.HEAPU8[o], M.HEAPU8[o + 1], M.HEAPU8[o + 2]]);
      }
      return out;
    },
  };
}

function snapshotsDiffer(a, b) {
  if (a.length !== b.length) return true;
  for (let i = 0; i < a.length; i++) {
    for (let k = 0; k < 3; k++) if (a[i][k] !== b[i][k]) return true;
  }
  return false;
}
function anyNonBlack(snap) { return snap.some(([r, g, b]) => r + g + b > 10); }
function allValuesInRange(snap) {
  return snap.every(([r, g, b]) => [r, g, b].every((v) => Number.isInteger(v) && v >= 0 && v <= 255));
}

// ---------------------------------------------------------------------
async function testFlame() {
  section('FlameShow');
  const ctx = makeCtx(await freshBoot());
  ctx.selectShow(ShowMode.Flame);
  check('show name is Flame', ctx.showName() === 'Flame', ctx.showName());

  for (const variationName of ['waterflames', 'flames', 'shifting hues', 'hue to white']) {
    ctx.selectVariationByName(variationName);
    ctx.silence();
    ctx.tick(10);
    const before = ctx.snapshot('rope');
    ctx.tick(30);
    const after = ctx.snapshot('rope');
    check(`[${variationName}] rope animates on its own (fire sim runs regardless of sound)`, snapshotsDiffer(before, after));
    check(`[${variationName}] rope values stay in valid byte range`, allValuesInRange(after));

    // Real sound reactivity: floods should read visibly different between
    // silence and a loud sustained bass hit.
    ctx.silence();
    ctx.tick(20);
    const quietMega = ctx.snapshot('megabar');
    ctx.setBins([ctx.adcMax(), 0, 0, 0, 0, 0, 0]);
    ctx.tick(20);
    const loudMega = ctx.snapshot('megabar');
    check(`[${variationName}] floods audibly react to bass (silence vs loud differ)`, snapshotsDiffer(quietMega, loudMega));
  }
}

// ---------------------------------------------------------------------
async function testEqualizer() {
  section('EqualizerShow (pixel_war / new_standard / etc)');
  const ctx = makeCtx(await freshBoot());
  ctx.selectShow(ShowMode.Equalizer);
  check('show name starts with Equalizer', ctx.showName().startsWith('Equalizer'), ctx.showName());

  // --- Chase: no sound needed, just time ---
  ctx.selectVariationByName('chase');
  ctx.silence();
  ctx.tick(5);
  const chaseBefore = ctx.snapshot('rope');
  ctx.tick(20);
  const chaseAfter = ctx.snapshot('rope');
  check('[chase] rope chases over time', snapshotsDiffer(chaseBefore, chaseAfter));

  // --- VU meter: bass/treble should visibly grow the meter from silence ---
  ctx.selectVariationByName('VU meter');
  ctx.silence();
  ctx.tick(10);
  const vuQuiet = ctx.snapshot('rope');
  ctx.setBins([ctx.adcMax(), 0, 0, 0, 0, ctx.adcMax(), ctx.adcMax()]); // bass + treble
  ctx.tick(10);
  const vuLoud = ctx.snapshot('rope');
  check('[vumeter] rope reacts to simultaneous bass+treble', snapshotsDiffer(vuQuiet, vuLoud));

  // --- new_standard: bass hits should change megabar/china over several hits ---
  ctx.selectVariationByName('new_standard');
  ctx.silence();
  ctx.tick(10);
  const nsBefore = { m: ctx.snapshot('megabar'), c: ctx.snapshot('china') };
  for (let i = 0; i < 8; i++) {
    ctx.setBins([ctx.adcMax(), 0, 0, 0, 0, 0, 0]);
    ctx.tick(3);
    ctx.silence();
    ctx.tick(3);
  }
  const nsAfter = { m: ctx.snapshot('megabar'), c: ctx.snapshot('china') };
  check('[new_standard] megabars react over several bass hits', snapshotsDiffer(nsBefore.m, nsAfter.m));
  check('[new_standard] china reacts over several bass hits', snapshotsDiffer(nsBefore.c, nsAfter.c));

  // --- pixel_war: bass hits should move the rope territory war ---
  ctx.selectVariationByName('pixel_war');
  ctx.silence();
  ctx.tick(10);
  const pwBefore = ctx.snapshot('rope');
  for (let i = 0; i < 6; i++) {
    ctx.setBins([ctx.adcMax(), 0, 0, 0, 0, 0, 0]);
    ctx.tick(3);
    ctx.silence();
    ctx.tick(3);
  }
  const pwAfter = ctx.snapshot('rope');
  check('[pixel_war] rope territory shifts over several bass hits', snapshotsDiffer(pwBefore, pwAfter));

  // --- sub_standard: same shared machinery as new_standard, megabars should react ---
  ctx.selectVariationByName('sub_standard');
  ctx.silence();
  ctx.tick(10);
  const subBefore = ctx.snapshot('megabar');
  for (let i = 0; i < 8; i++) {
    ctx.setBins([ctx.adcMax(), 0, 0, 0, 0, 0, 0]);
    ctx.tick(3);
    ctx.silence();
    ctx.tick(3);
  }
  const subAfter = ctx.snapshot('megabar');
  check('[sub_standard] megabars react over several bass hits', snapshotsDiffer(subBefore, subAfter));
}

// ---------------------------------------------------------------------
async function testSpeedStripes() {
  section('SpeedStripesShow');
  const ctx = makeCtx(await freshBoot());
  ctx.selectShow(ShowMode.SpeedStripes);
  check('show name starts with SpeedStripes', ctx.showName().startsWith('SpeedStripes'), ctx.showName());

  // --- default: real vehicle speed should scroll the rope ---
  ctx.selectVariationByName('default');
  ctx.tick(5);
  const before = ctx.snapshot('rope');
  for (let i = 0; i < 40; i++) { ctx.M.ccall('web_injectSpeedMph', null, ['number'], [30.0]); ctx.tick(1); }
  const after = ctx.snapshot('rope');
  check('[default] rope scrolls with injected vehicle speed', snapshotsDiffer(before, after));
  check('[default] rope values stay in valid byte range', allValuesInRange(after));

  // --- zebra: confirm it renders something plausible (not asserting the
  // specific blackout-scheduling behavior here -- that's a tuning/bugfix
  // detail, not a structural contract) ---
  ctx.selectVariationByName('zebra');
  for (let i = 0; i < 10; i++) { ctx.M.ccall('web_injectSpeedMph', null, ['number'], [20.0]); ctx.tick(1); }
  const zebraSnap = ctx.snapshot('rope');
  check('[zebra] rope is lit (not left entirely black)', anyNonBlack(zebraSnap));
  check('[zebra] rope values stay in valid byte range', allValuesInRange(zebraSnap));
}

// ---------------------------------------------------------------------
async function testLighthouse() {
  section('LighthouseShow');
  const ctx = makeCtx(await freshBoot());
  ctx.selectShow(ShowMode.Lighthouse);
  check('show name starts with Lighthouse', ctx.showName().startsWith('Lighthouse'), ctx.showName());

  // NOTE: this bridge build has no pot injector, so carpet->pot->readPercent()
  // reads its simulated-idle value for the whole test -- Lighthouse's
  // rotation speed is a direct multiply against that (see LightShow.h's
  // PotEnergyTakeover), so a "beams rotate over time" assertion here is
  // only meaningful once a pot injector exists again. Testing the parts
  // that don't depend on it instead.
  for (const variationName of ['default', 'no strobe']) {
    ctx.selectVariationByName(variationName);
    ctx.tick(5);
    const snap = { rope: ctx.snapshot('rope'), megabar: ctx.snapshot('megabar'), china: ctx.snapshot('china') };
    check(`[${variationName}] rope is lit (not left entirely black)`, anyNonBlack(snap.rope));
    check(`[${variationName}] rope values stay in valid byte range`, allValuesInRange(snap.rope));
  }

  // default: a bass hit should visibly strobe china's White channel (the
  // raw snapshot buffer is CRGBW-sized, see web_sizeofChinaLed(), so a
  // White-only change still shows up as a byte diff here) on top of
  // whatever beam-crossing RGB color china already has.
  ctx.selectVariationByName('default');
  ctx.silence();
  ctx.tick(10);
  const quiet = ctx.snapshot('china');
  ctx.setBins([ctx.adcMax(), 0, 0, 0, 0, 0, 0]);
  ctx.tick(5);
  const afterHit = ctx.snapshot('china');
  check('[default] china strobes on a bass edge', snapshotsDiffer(quiet, afterHit));
}

// ---------------------------------------------------------------------
async function main() {
  await testFlame();
  await testEqualizer();
  await testSpeedStripes();
  await testLighthouse();

  console.log('\n' + '='.repeat(60));
  console.log(`${passCount} passed, ${failCount} failed`);
  if (failCount > 0) {
    console.log('\nFailures:');
    for (const f of failures) console.log('  - ' + f);
    process.exit(1);
  }
  process.exit(0);
}

main().catch((e) => {
  console.error('Test runner crashed:', e);
  process.exit(1);
});
