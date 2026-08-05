#pragma once
// Shadow header (found via an -I entry that precedes FastLED's real src/ on
// the include path) working around a real gap in this FastLED version's
// WASM platform: platforms/stub/clockless_stub.h's __EMSCRIPTEN__ branch
// redirects to platforms/wasm/clockless.h, which declares ClocklessController
// inside `namespace fl` -- but chipsets.h's own FASTLED_CLOCKLESS_CONTROLLER
// macro still expands to a bare, non-namespaced ClocklessController. Real
// hardware builds never hit this (ARDUINO is defined, a different platform
// branch entirely); FastLED's own wasm_compile.py pipeline apparently papers
// over it some other way we're not replicating by compiling raw emcc.
//
// We don't need any of chipsets.h's actual hardware-chipset-controller
// classes at all -- the WASM build bypasses FastLED's controller system
// entirely (see tools/wasm/README.md) -- so just pull the missing name into
// global scope and chain to the real header via #include_next.
using fl::ClocklessController;
#include_next "chipsets.h"
