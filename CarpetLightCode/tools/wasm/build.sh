#!/bin/bash
# Builds the real firmware (src/CarpetLightLogic.cpp and everything it
# #includes) to WebAssembly for tools/visualizer/carpet-visualizer.html.
# See tools/wasm/README.md for the full story; this script just codifies
# the working incantation discovered getting Phase 4 of the migration plan
# (~/.claude/plans/quiet-watching-cherny.md) to compile.
#
# Usage:
#   source tools/wasm/emsdk/emsdk_env.sh   # one-time per shell session
#   ./tools/wasm/build.sh

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"
PIO_FASTLED_SRC="$REPO_ROOT/.pio/libdeps/esp32-s3-devkitc-1/FastLED/src"
VENDOR_FASTLED_SRC="$SCRIPT_DIR/vendor/fastled_src"
BRIDGE_SRC="$SCRIPT_DIR/bridge/web_bridge.cpp"
OUT_DIR="$REPO_ROOT/tools/visualizer/wasm"
HALSHIM="$REPO_ROOT/src/HalShim.h"

if ! command -v em++ >/dev/null 2>&1; then
  echo "error: em++ not found. Run: source tools/wasm/emsdk/emsdk_env.sh" >&2
  exit 1
fi

if [ ! -d "$PIO_FASTLED_SRC" ]; then
  echo "error: $PIO_FASTLED_SRC not found -- run 'pio run' at least once first (fetches FastLED via PlatformIO)." >&2
  exit 1
fi

# Refresh the vendored+patched FastLED copy from whatever PlatformIO
# currently has installed, then re-apply our WASM-specific patches. Done on
# every build (not committed -- see .gitignore) so a FastLED version bump
# via platformio.ini is picked up automatically; the patch step itself is
# small and idempotent.
rm -rf "$VENDOR_FASTLED_SRC"
mkdir -p "$SCRIPT_DIR/vendor"
cp -r "$PIO_FASTLED_SRC" "$VENDOR_FASTLED_SRC"

echo "Patching vendored FastLED for FASTLED_SKIP_WASM_ARDUINO_COMPAT..."
"$SCRIPT_DIR/patch_fastled.sh" "$VENDOR_FASTLED_SRC"

mkdir -p "$OUT_DIR"

em++ \
  -I"$REPO_ROOT/src" \
  -I"$VENDOR_FASTLED_SRC" \
  -include "$HALSHIM" \
  -DFASTLED_SKIP_WASM_ARDUINO_COMPAT \
  -D__EMSCRIPTEN__ \
  -std=gnu++17 -O2 \
  -s MODULARIZE=1 -s EXPORT_NAME=CarpetFirmware \
  -s EXPORTED_FUNCTIONS='["_web_setup","_web_tick","_web_getRopeLedsPtr","_web_getMegabarLedsPtr","_web_getChinaLedsPtr","_web_getNumRopeLeds","_web_getNumMegabarLeds","_web_getNumChinaLeds","_web_sizeofRopeLed","_web_sizeofMegabarLed","_web_sizeofChinaLed","_web_getCurrentShowName","_web_getCurrentVariationName","_web_forcePressShort","_web_injectButtonDown","_web_injectButtonUp","_web_injectEncoderDelta","_web_injectPotPercent","_web_getAppMode","_web_getConfigSubsetting","_web_getSettingName","_web_setGlobalBrightness","_web_getGlobalBrightness","_web_setHeadlightBrightness","_web_getHeadlightBrightness","_web_setChinaBrightness","_web_getChinaBrightness","_web_setBlacklightOn","_web_getBlacklightOn","_web_getAdcBinsBufferPtr","_web_injectAdcBins","_web_audioGetHitPercent","_web_audioGetNormalPercent","_web_audioGetRawPercent","_web_audioGetRmsPercent","_web_audioGetOverallLevelPercent","_web_audioIsSilent","_web_audioSetAgcMode","_web_audioGetAgcMode","_web_audioSetNoiseFloorPercent","_web_audioGetNoiseFloorPercent","_web_audioSetPeakThresholdPercent","_web_audioGetPeakThresholdPercent","_web_audioSetHitDecayMs","_web_audioGetHitDecayMs","_web_audioSetAudioForesightMs","_web_audioGetAudioForesightMs","_web_audioSetHitPredictionMs","_web_audioGetHitPredictionMs","_web_audioSetHitPredictionStyle","_web_audioGetHitPredictionStyle","_web_audioSetSoundReactivityEnabled","_web_audioGetSoundReactivityEnabled","_web_audioSetAutoPeakEnabled","_web_audioGetAutoPeakEnabled","_web_audioGetAutoScaledPeakThresholdPercent","_web_audioGetBandAutoScaledPeakThresholdPercent","_web_audioGetMaxPercent","_web_getAdcResolutionBits","_web_getAdcMaxValue","_web_getStrobePin","_web_getResetPin","_web_getPinLogPinsPtr","_web_getPinLogValsPtr","_web_getPinLogCount","_web_clearPinLog"]' \
  -s EXPORTED_RUNTIME_METHODS='["ccall","cwrap","HEAPU8","HEAP32","setValue","getValue"]' \
  -o "$OUT_DIR/carpet_fw_raw.js" \
  "$BRIDGE_SRC" \
  "$VENDOR_FASTLED_SRC/fl/colorutils.cpp" \
  "$VENDOR_FASTLED_SRC/hsv2rgb.cpp" \
  "$VENDOR_FASTLED_SRC/lib8tion.cpp" \
  "$VENDOR_FASTLED_SRC/crgb.cpp" \
  "$VENDOR_FASTLED_SRC/fl/fill.cpp"

# Emscripten's default glue JS loads the .wasm via fetch(), which fails
# under file:// (CORS blocks fetching local files). -s SINGLE_FILE=1 is
# broken in this Emscripten build (6.0.5) -- it produces a WASM blob that
# fails to instantiate ("unknown type form" / memory-size decode errors,
# reproducible even over HTTP with a trivial no-op module, so it's a genuine
# encoder bug here, not something about this project's build). Also note:
# this glue JS's `var wasmBinary;` is never wired up to `Module['wasmBinary']`
# at all (checked the generated source directly) -- pre-setting that field,
# the "documented" override, silently does nothing. The override that *does*
# work is `Module['instantiateWasm']`, which the glue calls directly instead
# of its normal fetch path if present. So: base64-embed the real .wasm
# ourselves and prepend a small instantiateWasm shim.
python3 - "$OUT_DIR/carpet_fw_raw.js" "$OUT_DIR/carpet_fw_raw.wasm" "$OUT_DIR/carpet_fw.js" <<'PYEOF'
import base64
import re
import sys

glue_path, wasm_path, out_path = sys.argv[1:4]
wasm_b64 = base64.b64encode(open(wasm_path, "rb").read()).decode("ascii")
glue = open(glue_path).read()

shim = f'''// Auto-generated by tools/wasm/build.sh -- inlines the .wasm as base64 and
// supplies Module.instantiateWasm so no fetch() of a separate .wasm file is
// ever attempted (required for the visualizer to work opened via file://).
var __carpetWasmBytes = Uint8Array.from(atob("{wasm_b64}"), c => c.charCodeAt(0));
'''

# Matches both the pretty-printed ("var Module = moduleArg;") and minified
# ("var Module=moduleArg;") forms -- whichever -O level em++ produced.
marker_re = re.compile(r"var\s+Module\s*=\s*moduleArg;")
m = marker_re.search(glue)
if not m:
    sys.exit("build.sh: could not find 'var Module=moduleArg;' in generated glue JS -- Emscripten output format changed, update the marker regex")
inject = (
    'if(!Module["instantiateWasm"]){Module["instantiateWasm"]=(imports,successCallback)=>{'
    "WebAssembly.instantiate(__carpetWasmBytes,imports).then((result)=>{"
    "successCallback(result.instance,result.module)"
    "});return{}"
    "};}"
)
glue = shim + glue[:m.end()] + inject + glue[m.end():]
open(out_path, "w").write(glue)
PYEOF

rm -f "$OUT_DIR/carpet_fw_raw.js" "$OUT_DIR/carpet_fw_raw.wasm"

echo "Built $OUT_DIR/carpet_fw.js (wasm inlined as base64 with an instantiateWasm shim -- no separate .wasm fetch, works from file:// with no local server)"
