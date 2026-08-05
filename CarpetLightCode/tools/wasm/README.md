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
sources, (4) base64-inlines the resulting `.wasm` into the `.js` glue with a
small `Module.instantiateWasm` shim (see below), output to
`tools/visualizer/wasm/carpet_fw.js` (~140KB as of this writing -- a single
file, no separate `.wasm` to fetch).

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

FastLED already ships its own `__EMSCRIPTEN__` platform support -- but it's
built for FastLED's *own* sketch-compiler/web-player pipeline
(`platforms/wasm/`: `js_bindings.cpp`, `active_strip_data.cpp`, `ui.cpp`,
etc. -- their own `uv run ci/wasm_compile.py` + Docker toolchain), not a
general-purpose "compile arbitrary Arduino-shaped C++ with a custom JS
bridge" target. We don't use that pipeline (see the migration plan's
build-tooling discussion for why); we `#include` FastLED's headers directly
via a standalone `em++` invocation instead. That surfaces two distinct,
unrelated problems:

**1. `chipsets.h`'s `FASTLED_CLOCKLESS_CONTROLLER` macro doesn't resolve.**
It expands to a bare `ClocklessController`, but this FastLED version's WASM
platform (`platforms/stub/clockless_stub.h`'s `__EMSCRIPTEN__` branch,
which redirects to `platforms/wasm/clockless.h`) only ever defines
`fl::ClocklessController`. **Moot for us**: we never instantiate any
FastLED hardware-controller class at all (`MagicCarpet::setup()`'s
`FastLED.addLeds<NEO_PORT_BANK,...>()` and `MagicCarpet::show()`'s
`FastLED.show()` are both excluded under `__EMSCRIPTEN__`, see
`MagicCarpet.h`) -- so instead of fixing the macro, our patched `FastLED.h`
skips `platforms.h`/`chipsets.h`/`fastspi.h` entirely, and replaces the
`CFastLED` class (normally ~500 lines of per-chipset `addLeds<>()`
overloads) with a minimal stub exposing just the handful of methods our own
code actually calls from paths that stay compiled on every target
(`FastLED.delay()`, used by `DemoShow.h` and `MagicCarpet::error()` -- both
present in the build but not reached in normal operation).

**2. FastLED's own WASM-Arduino-compat shim collides with ours.** The
moment `__EMSCRIPTEN__` is defined, FastLED unconditionally pulls in its
own `millis()`/`micros()`/`min()`/`max()`/`constrain()`/`analogRead()`/
`digitalWrite()`/`Serial` emulation (`platforms/wasm/js.h`,
`platforms/stub/Arduino.h`, `platforms/stub/time_stub.h`,
`platforms/stub/led_sysdefs_stub_generic.h` -- reached via at least three
independent include paths, since several of FastLED's own files do
self-referential `#include "FastLED.h"`/`#include "..."` that always
resolve to their own colocated directory regardless of `-I` search order,
which is why a simple shadow-header trick doesn't work here and these files
needed direct, in-place patches instead). This is a **real, unrelated**
Arduino-compat layer FastLED provides for *its own* purposes -- but our
project already has one (`src/HalShim.h`), with **injectable** behavior
(JS-controlled `millis()`, ADC values, pin state) instead of FastLED's
fixed one (real wall-clock `millis()`, `random()`-backed `analogRead()`,
no-op `digitalWrite()`). Having both visible in the same translation unit
is a hard conflict (declaration with different language linkage,
redefinition errors) -- not a matter of picking whose version is "correct."

**The fix, all gated behind one flag (`-DFASTLED_SKIP_WASM_ARDUINO_COMPAT`,
only ever passed by our own `build.sh` -- real hardware and FastLED's own
normal WASM usage are completely unaffected):** every file above gets a
small patch (see `tools/wasm/patches/*.patch`, applied fresh to a vendored
copy every build -- never hand-edit `tools/wasm/vendor/`, it's regenerated
and gitignored) wrapping its competing declarations in
`#ifndef FASTLED_SKIP_WASM_ARDUINO_COMPAT`. One additional patch to
`FastLED.h` re-adds `using namespace fl;` at global scope, since disabling
`platforms/stub/Arduino.h`'s body also disables its
`FASTLED_USING_NAMESPACE` invocation, which is what normally makes
`fl::CRGB`/`fl::CHSV`/etc. usable as bare `CRGB`/`CHSV` (our project's code,
like most FastLED users' code, refers to them unqualified).

**One more wrinkle, unrelated to any of the above:** FastLED's separately-
compiled `.cpp` sources (`fl/colorutils.cpp`, `hsv2rgb.cpp`, `lib8tion.cpp`,
`crgb.cpp`, `fl/fill.cpp` -- linked directly into the build, providing
`blend()`/`hsv2rgb_rainbow()`/`fill_gradient_RGB()`/etc.) are their own
translation units that never see our project's `Utilities.h`/`HalShim.h`
at all, so `-include src/HalShim.h` is passed to the *entire* `em++`
invocation (applies to every `.cpp` on the command line, including these),
ensuring `millis()`/`micros()` resolve consistently to our own injectable
versions everywhere, not just in the files that happen to `#include`
`Utilities.h` themselves.

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
