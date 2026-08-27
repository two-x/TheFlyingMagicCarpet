# WASM build (visualizer's real-firmware backend)

Compiles the real firmware (`src/*.h`/`.cpp`) to WebAssembly so
`tools/visualizer/carpet-visualizer.html` can run it directly instead of a
hand-ported JS mirror. See `/Users/soren/.claude/plans/quiet-watching-cherny.md`
for the full migration plan/phases. This build is completely separate from
`platformio.ini` / the real Arduino Due hardware target -- neither is ever
touched by anything under `tools/wasm/`.

## One-time setup

```bash
cd tools/wasm
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest   # pinned: 6.0.5 as of this writing
./emsdk activate latest
```

`tools/wasm/emsdk/` is gitignored (~1.7GB) -- every contributor who wants to
*build* the WASM module needs this installed locally, but nobody who just
wants to *use* the visualizer does, since the built `.wasm`/`.js` output
(`tools/visualizer/wasm/`) is committed.

Every build command needs the emsdk environment sourced first (shell state
doesn't persist across separate terminal sessions):

```bash
source tools/wasm/emsdk/emsdk_env.sh
```

## Building

```bash
./tools/wasm/build.sh
```

This: (1) copies `.pio/libdeps/.../FastLED/src` into the gitignored
`tools/wasm/vendor/fastled_src/` (regenerated fresh every build, so a
FastLED version bump via `platformio.ini` is picked up automatically), (2)
applies the patches in `tools/wasm/patches/` (see below) via
`patch_fastled.sh`, (3) compiles `tools/wasm/bridge/web_bridge.cpp` (which
`#include`s the real `src/CarpetLightLogic.cpp` directly -- Arduino
sketches have no `main()` of their own, the framework calls `setup()`/
`loop()`, and this bridge does the same) plus FastLED's portable pixel-math
sources (`colorutils.cpp`, `hsv2rgb.cpp`, `lib8tion.cpp` -- see below for why
this list is shorter than you might expect), (4) base64-inlines the
resulting `.wasm` into the `.js` glue with a small `Module.instantiateWasm`
shim (see below), output to `tools/visualizer/wasm/carpet_fw.js` (~180KB as
of this writing -- a single file, no separate `.wasm` to fetch).

### Why the `.wasm` is base64-inlined into the `.js` (the `file://` problem)

The visualizer needs to work when a remote developer just double-clicks
`carpet-visualizer.html` -- opened via `file://`, no local server. Emscripten's
default glue JS loads the `.wasm` via `fetch()`, which fails under `file://`
(browsers block `fetch()` of local files as a CORS violation). The
"documented" fix, `-s SINGLE_FILE=1`, is **broken in this Emscripten build
(6.0.5)**: it produces a module that fails to instantiate
(`WebAssembly.instantiate(): unknown type form`, or a memory-size decode
error depending on which `ALLOW_MEMORY_GROWTH`/`INITIAL_MEMORY` combination is
tried) -- reproducible even over plain HTTP with a trivial no-dependency test
module, so it's a genuine encoder bug in this Emscripten version, not
something specific to this project's build.

The other documented escape hatch, pre-setting `Module.wasmBinary` before the
glue JS runs, also does nothing in this build -- reading the actual generated
glue source shows `var wasmBinary;` is declared but never assigned from
`Module['wasmBinary']` anywhere. The override that *does* work, and is what
`build.sh`'s Python post-processing step wires up, is `Module.instantiateWasm`:
a callback the glue calls directly (`createWasm()`'s `var instantiateWasm =
Module["instantiateWasm"]`) in place of its normal fetch-then-instantiate
path, if present. `build.sh` base64-encodes the built `.wasm`, prepends it to
the glue JS as a `Uint8Array`, and injects
`Module.instantiateWasm = (imports, cb) => WebAssembly.instantiate(bytes,
imports).then(r => cb(r.instance, r.module))` right after the glue's own
`var Module = moduleArg;` line (matched via regex so it survives both
pretty-printed and minified `-O` output). No separate `.wasm` file is ever
fetched, so this loads identically under `file://` and `http://`.

## Why FastLED needs patching for this build (the real story)

**Important, and easy to get wrong if you go looking at FastLED's own
GitHub source while debugging this:** the `fastled/FastLED@3.6.0` package
PlatformIO actually installs (`.pio/libdeps/.../FastLED/src`) is an older,
plainer layout than the current FastLED `main` branch/newer releases --
`uint8_t`/`uint16_t` types throughout (not the `fl::u8`/`fl::u16` namespace
wrapper newer FastLED uses), and **no `platforms/wasm/` or `platforms/stub/`
directories at all** -- `grep -r EMSCRIPTEN .pio/libdeps/.../FastLED/src`
comes back completely empty. This installed release simply predates
FastLED's own WASM/emscripten platform support; it was never a case of us
opting out of FastLED's own WASM compat layer, because for this version
there isn't one to opt out of. (This has already changed once during this
project's life -- the very patches this section used to describe assumed a
newer FastLED tree that *did* have `platforms/wasm/`/`platforms/stub/`, and
broke when this file's own history diverged from whatever the registry
serves. If a future FastLED version bump changes any of this again, the
right move is to `grep -r EMSCRIPTEN` the freshly-vendored copy again and
re-diagnose from there, not to assume anything below is still accurate.)

Since there's no WASM branch, `em++`-compiling this code with `__EMSCRIPTEN__`
defined hits FastLED's *hardware*-platform dispatch with nothing matching,
which goes one of two ways depending on the file:

- **`platforms.h`'s dispatch has a silent `#else` -- real AVR headers.** An
  unrecognized platform (which `__EMSCRIPTEN__` is, to this file) falls all
  the way through to `#include "platforms/avr/fastled_avr.h"`, pulling in
  real `avr/io.h`-style hardware register access that can't compile for a
  wasm32 target.
- **`led_sysdefs.h`'s dispatch has an explicit `#error`.** Same shape of
  dispatch, but its `#else` is `#error "This platform isn't recognized by
  FastLED... yet."` -- a hard compile failure instead of a silent wrong
  include.
- **`fastpin.h` needs `RwReg`/`RoReg` pin-register typedefs** that only a
  real platform's `led_sysdefs_*.h` (the file the above dispatch would have
  selected) defines -- so once real platform headers are off the table,
  `fastpin.h` itself won't compile either.
- **`fastspi.h`/`chipsets.h` define FastLED's real hardware-chipset
  controller classes** (`ClocklessController` and friends) -- not something
  that fails to compile outright, but pulling them in is pointless *and*
  drags in more platform-specific assumptions than necessary, since we
  never instantiate any of them (`MagicCarpet::setup()`'s
  `FastLED.addLeds<NEO_PORT_BANK,...>()` and `MagicCarpet::show()`'s
  `FastLED.show()` are both excluded under `__EMSCRIPTEN__`, see
  `MagicCarpet.h`).

**The fix, all gated behind one flag (`-DFASTLED_SKIP_WASM_ARDUINO_COMPAT`,
only ever passed by our own `build.sh` -- real hardware is completely
unaffected):** small patches (see `tools/wasm/patches/*.patch`, applied
fresh to a vendored copy every build -- never hand-edit
`tools/wasm/vendor/`, it's regenerated and gitignored) that:

1. Skip `platforms.h`'s `#include` from `FastLED.h` entirely.
2. Skip `fastpin.h`'s `#include` from `FastLED.h` entirely -- nothing in
   our actual build (`colorutils.cpp`/`hsv2rgb.cpp`/`lib8tion.cpp`,
   `CarpetLightLogic.cpp`'s own `#include` chain) touches FastLED's
   `Pin`/`OutputPin` classes; we drive pins through our own
   `src/HalShim.h` instead.
3. Skip `fastspi.h`/`chipsets.h`'s `#include` from `FastLED.h` entirely.
4. Add an `#elif defined(__EMSCRIPTEN__) && defined(FASTLED_SKIP_WASM_ARDUINO_COMPAT)`
   branch to `led_sysdefs.h`'s platform dispatch, ahead of its `#error`,
   that includes nothing (none of the real per-platform defines it would
   normally provide are needed once (1)-(3) above are in effect).
5. Replace `FastLED.h`'s real `CFastLED` class (normally ~500 lines of
   per-chipset `addLeds<>()` overloads) with a minimal stub exposing just
   the handful of methods our own code actually calls from paths that stay
   compiled on every target (`FastLED.delay()`, used by `DemoShow.h` and
   `MagicCarpet::error()` -- both present in the build but not reached in
   normal operation), and make the global `FastLED` instance a C++17
   `inline` variable (its real definition normally lives in `FastLED.cpp`,
   which we don't compile).

Our own `src/HalShim.h` (`millis()`, ADC values, pin state -- all
JS-injectable) remains the *only* Arduino-compat layer in the build; there's
no competing one from FastLED to reconcile it with in this FastLED release,
and no `fl::` namespace to bridge back to bare `CRGB`/`CHSV` either (this
layout's `pixeltypes.h` already declares them as bare global types).

**One more wrinkle:** this FastLED layout keeps `colorutils.cpp`/
`hsv2rgb.cpp`/`lib8tion.cpp` directly under `src/` (no `fl/` subdirectory),
has no separate `crgb.cpp` (`CRGB`'s methods are header-only, inline in
`pixeltypes.h`), and no separate `fill.cpp` (`fill_solid()`/`fill_rainbow()`/
etc. already live inside `colorutils.cpp`) -- `build.sh`'s compiled-sources
list reflects that. These are their own translation units that never see
our project's `Utilities.h`/`HalShim.h` at all, so `-include src/HalShim.h`
is passed to the *entire* `em++` invocation (applies to every `.cpp` on the
command line, including these), ensuring `millis()`/`micros()` resolve
consistently to our own injectable versions everywhere, not just in the
files that happen to `#include` `Utilities.h` themselves.

None of this required editing PlatformIO's own managed FastLED copy under
`.pio/libdeps/` (which it can silently regenerate) -- everything lives in
`tools/wasm/patches/` as reviewable, versioned diffs, applied fresh on
every build.

## Phase 0 proof

`tools/wasm/phase0/` and `tools/wasm/phase1/` through `phase4/` are the
migration plan's own toolchain-proof artifacts, kept for
reference/regression-checking if a future FastLED version bump changes any
of the above. Not part of the real build (`tools/wasm/bridge/web_bridge.cpp`
is the canonical entry point `build.sh` actually compiles).
