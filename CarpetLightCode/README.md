# Carpet Lighting Hardware Layout

Reference documentation for the physical arrangement of the carpet's lighting hardware — the perimeter rope lights, the megabar floods, and the china-light floods. Source diagrams (hand-drawn) live in the "carpet information dumpster" doc, Lights section:
https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA

All directions below (front/back/left/right) are relative to the carpet's own forward direction of travel, not compass or diagram orientation.

## Perimeter rope lights — `ropeLeds[NUM_NEO_LEDS_ACTUAL]`, 1016 LEDs total

Only **4 continuous physical strips** exist. `NEO0_OFFSET`..`NEO3_OFFSET` (the 4 real strips, reading from `ropeLeds[]`) are the ones actually in use; `NEO4_OFFSET`..`NEO7_OFFSET` were leftover offsets from an original 8-strip plan that was never wired, and have been removed from code.

`NUM_NEOPIXEL_STRIPS` stays at **8** in code even though only 4 strips are physically wired — don't "fix" this to 4. `MagicCarpet::show()`'s 4 active `convertNeoArray()` calls write into the `ropeShowLeds` FastLED output buffer at strip-slots **0, 1, 4, and 5** (not 0-3), so the buffer has to stay sized for 6 slots or those slot-4/5 writes run off the end. This also means it's **not confirmed** which physical pins (of `NEO_PIN0`-`NEO_PIN7`) the 2 "extra" strips are actually wired to — don't assume it's simply "the first 4 pins."

Each real strip runs straight through the corner(s) it passes — wiring does **not** break at a corner. Splits instead land 30 LEDs in from each true corner, along the front/back edges:

| Channel | Length | Runs along | Local index 0 | Local index max |
| --- | --- | --- | --- | --- |
| CH1 | 156 LEDs | Front edge | near front-right | 155, near front-left |
| CH2 | 352 LEDs | Right side | near front-right | 351, near back-right |
| CH3 | 156 LEDs | Back edge | near back-left | 155, near back-right |
| CH4 | 352 LEDs | Left side | near back-left | 351, near front-left |

(30 + 156 + 30 = 216 front/back width; 30 + 292 + 30 = 352 side height.)

`ropeLeds[]` is logically one loop that starts/wraps at FRONT (see the `FRONT`/`RIGHT`/`BACK`/`LEFT`/etc. positional constants in `MagicCarpet.h`). `show()` reverses some sub-ranges before pushing to hardware, to reconcile logical vs. physical wire order. The exact `ropeLeds[]` global index ↔ per-channel local index above hasn't been independently verified — trust the `FRONT`/`RIGHT`/`BACK`/`LEFT` constants in code over the channel-local numbers in this doc if they ever conflict.

**Bus timing** (`NEO_PORT_BANK = WS2811_PORTD`, `FastLED.addLeds<NEO_PORT_BANK,NUM_NEOPIXEL_STRIPS>()`): checked against FastLED's own source (`chipsets.h`/`FastLED.h`) rather than assumed. `WS2811_PORTD` resolves to an `InlineBlockClocklessController` with bit timing `NS(320),NS(320),NS(640)` — FastLED's standard **800kHz** WS281x profile (the fast one; there's also a slower 400kHz WS2811 mode this isn't using), and this is the fastest bit rate the WS281x protocol spec supports at all — there's no faster setting to switch to. More importantly, `PORTD` is a genuinely **parallel** output: one Port D byte-write toggles up to 8 physical pins per bit-time, so all wired strips clock out simultaneously rather than one after another — already the fastest multi-strip approach FastLED offers on a non-DMA MCU like the SAM3X8E, far better than looping single-strip `addLeds()` calls (which would take roughly as many times longer as there are strips). Per-frame cost is still real, though: every lane clocks the same pixel count as the *longest* wired channel (`NUM_NEO_LEDS_PER_STRIP`, ≈470 RGBW-emulation-inflated slots from the 352-LED side channels), so `show()` blocks for roughly 470 × 24 bits × ~1.28ns/bit ≈ 14ms with interrupts disabled, every single frame — an inherent cost of the WS281x protocol at this LED count, not something more FastLED configuration can reduce further. One side effect worth knowing: because parallel clockless controllers share one pixel count across all lanes, the two shorter 156-LED channels (front/back edges) get silently padded out to that same ~470-slot length each frame — wasted bus time on those two, but fixing it would mean rewiring which physical run lands in which lane, not a software change.

## Megabars — `megabarLeds[12]`, DMX addr 1, 4, 7, ..., 34 (RGB, 3 channels each)

Mounted in a ring, roughly horizontal (aimed slightly downward but mostly level), each one pointed 30° further around than the last — together they extend the light pattern outward in a full circle around the platform.

`megabarLeds[i]` corresponds to DMX address `(1 + 3*i)`.

| Index | Addr | Direction |
| --- | --- | --- |
| `[0]` | 01 (×2 fixtures, same address) | Straight ahead — **front** (0°) |
| `[1]` | 04 | 30° |
| `[2]` | 07 | 60° |
| `[3]` | 10 | 90° — straight out the **left** side |
| `[4]` | 13 | 120° |
| `[5]` | 16 | 150° |
| `[6]` | 19 | 180° — straight out the **back** |
| `[7]` | 22 | 210° |
| `[8]` | 25 | 240° |
| `[9]` | 28 | 270° — straight out the **right** side |
| `[10]` | 31 | 300° |
| `[11]` | 34 | 330° — back toward front |

Address 01 is the only doubled-up fixture (two physical units, same DMX address); every other address is a single unit. Going in increasing address order, the ring sweeps front → left → back → right → back to front.

In code, `megabarLeds[HEADLIGHT_INDEX]` (index `[0]`, addr 01) is called the **headlight** and gets its own brightness control, separate from the rest — see "Brightness system" below.

## China lights — `chinaLeds[8]`, DMX addr 37, 43, ..., 79 (RGBWUA, 6 channels each)

Very bright, mounted in pairs at the 4 corners. Aimed primarily **straight down** (unlike the megabars), but X/Y aim still matters — each beam scatters well past its bright spot in the direction it's aimed. Together they flood the ground under the whole perimeter, making the platform look like it's floating on a cushion of light.

The 2 extra channels beyond RGBW are `CRGBWUA::u` (**UV/blacklight** — confirmed; RGBWUA is a standard 6-channel fixture layout, and the union in `CRGBW.h` aliases `u` as `black`) and `.a` (amber, unconfirmed — still a guess, `CRGBW.h` has it marked `// TODO: figure out what a is for...`).

At each corner, the 2 fixtures split duty: one lights the edge running one way from that corner, the other lights the edge running the other way. Each bright spot lands ~1/3 of the way along its assigned edge, measured **from its own corner toward the far corner** (light continues to scatter further past that point, toward the far corner).

| Index | Addr | Corner | Aimed along | Bright spot |
| --- | --- | --- | --- | --- |
| `[0]` | 37 | Front-right | Right edge | ~1/3 back from front |
| `[1]` | 43 | Front-right | Front edge | ~1/3 in from right |
| `[2]` | 49 | Front-left | Front edge | ~1/3 in from left |
| `[3]` | 55 | Front-left | Left edge | ~1/3 back from front |
| `[4]` | 61 | Back-left | Left edge | ~1/3 forward from back |
| `[5]` | 67 | Back-left | Back edge | ~1/3 in from left |
| `[6]` | 73 | Back-right | Back edge | ~1/3 in from right |
| `[7]` | 79 | Back-right | Right edge | ~1/3 forward from back |

## Brightness system

All brightness control lives in `MagicCarpet` and is expressed only as **percentages** (floats) — light shows and UI code never touch raw 0-255 hardware values directly; `percentToRaw()` is the sole, private conversion point. Three independent percentages compound multiplicatively via plain linear scaling (`scale8()`). Megabar/china are deliberately **not** gamma-corrected anywhere in this brightness math (gamma is a nonlinear curve — applying it after scaling breaks the linear addition the headlight math depends on); gamma correction only applies to the rope, automatically, inside `show()`. Only `FlameShow` gamma-corrects its own megabar output on top of this — a pre-existing inconsistency, not something the brightness system does:

| Setting | Range | Default | Scales |
| --- | --- | --- | --- |
| `globalBrightness_` | 0-100% | 100% (unrestricted) | Everything: rope, all megabars, all china |
| `headlightBrightness_` | **50-100%** | 50% | Just the headlight (`megabarLeds[HEADLIGHT_INDEX]`), on top of global |
| `chinaBrightness_` | 0-100% | 100% (unrestricted) | All of `chinaLeds[]`, on top of global |

**Headlight brightness is a direct per-fixture percentage** of the global max — same units as every other brightness setting, no special conversion. It's floored at 50% rather than 0%: below that, the combined pair would look dimmer than any other single megabar, which isn't a useful setting to offer. At 50% (default) each fixture individually runs at half the global max — since the headlight is two physical fixtures at the same DMX address pointed the same direction, their light output adds linearly, so the pair combined looks like one normal megabar at 100%. At 100% each fixture runs at its own full global max — the pair combined looks twice as bright as one normal megabar. (This additive reasoning is exactly why the megabar/china brightness math stays gamma-free — see above; gamma-correcting before this addition would badly under-represent the combined output.)

`MagicCarpet::applyBrightnessCeiling()` applies all this every frame, after a show writes its output and before `show()` pushes to hardware. All three settings persist to flash (`Nvm.h`) and survive power cycles.

## UI — encoder & button

### Button press tiers

Classified by hold duration (`LedControl::PushButton` in `LedController.h`). Short/medium/long are decided **at release**; extra-long and double are decided **live**, the instant they're detected, without waiting for release:

| Press | Duration | When it fires |
| --- | --- | --- |
| Short | < 300ms | ~250ms after release (see double-press below) |
| Medium | 300-1500ms | At release |
| Long | 1500-3000ms | At release |
| Extra-long | ≥ 3000ms | Live, the instant 3000ms is crossed while still held |
| Double | 2nd press starts within 250ms of a short press's release | Live, the instant the 2nd press begins |

Once extra-long or double fires live during a hold, nothing else can fire for the rest of that same hold (not the other one, not a threshold-crossing flash, not a release classification) — the button is locked until physically released.

A short press is deliberately delayed ~250ms past release: if a second press begins in that window, it becomes a double instead (firing immediately), and the original short press never fires. If nothing follows, the short press finally fires ~250ms late.

**Edge capture is interrupt-driven, not polled.** The button pin is watched by a `CHANGE` interrupt (`PushButtonEdge::isr()`) that timestamps every down/up transition into a small queue the instant it happens; `PushButton::update()` drains that queue each loop and replays every edge using its own recorded timestamp, rather than sampling the pin's live state once per call. This matters because the main loop's iteration time is dominated by `FastLED.show()` (~14ms, most of it spent with interrupts disabled during rope LED bit-banging — see the timing report below) plus other blocking work; a plain polled `digitalRead()` can miss a short tap entirely if the whole press-and-release happens between two `update()` calls. An edge interrupt still can't fire during FastLED's own disabled-interrupt window, but that window is much narrower than a full loop iteration, so this meaningfully shrinks the blind spot rather than eliminating the loop-cadence dependency entirely.

(Aside: `ENCODER_SW_PIN` and `POT_ANALOG_PIN` in `MagicCarpet.h` both use the literal value `3` and used to carry a `FIXME` suspecting a pin conflict — that's a false alarm, not a real conflict. On the Due, `analogRead()` auto-offsets any value below `A0` by adding `A0` to it (`wiring_analog.c`: `if (ulPin < A0) ulPin += A0;`), so `POT_ANALOG_PIN 3` actually addresses pin **A3**, while `ENCODER_SW_PIN 3` is read via `digitalReadDirect()`, which addresses raw digital pin **D3** — two different physical pins despite the shared literal `3` in source. Confirmed against the actual `analogRead()` implementation, not assumed.)

### Perimeter flash feedback

The rope perimeter flashes white as UI feedback (`MagicCarpet::flashRope()` — 55ms white padded by 15ms black on each side; 85ms gap between multiple flashes in one call; blocking, but polls the button throughout via `delayPolling()` rather than going fully blind — a flash sequence can run up to ~255ms, long enough to otherwise drop part of a rapid follow-up double-press):

- **1 flash**, live, the instant a hold crosses the long-press threshold (1500ms) — no feedback for crossing the medium-press threshold (300ms) by request
- **1 flash** when a double press commits a config setting/subsetting (not in normal mode, where double press does other things — see below)
- **2 flashes** when a medium press advances to the next config setting (not in normal mode — a normal-mode medium press currently does nothing)

### Normal mode

- **Short press**: cycles to the next light show (Nightrider → FlameSparkle → Equalizer → SpeedStripes → Lighthouse → ...). Persisted to flash immediately.
- **Extra-long press**: toggles the whole rig on/off. Always boots **on** — this one is intentionally *not* persisted.
- **Double press**: toggles blacklight (china's UV channel, `CRGBWUA::u` — see "China lights" above) full on/off, independent of whatever the active show is doing — **except while the Equalizer show is active**, where double press instead toggles that show's own triple-strobe setting (see "Equalizer show variations" below); blacklight isn't reachable by double press during Equalizer. Blacklight is intentionally volatile — always boots **off**, never persisted, and is also force-reset to off whenever the light show changes (short press), not just when the whole rig is turned off (extra-long press). The Equalizer strobe toggle is a separate, ordinary persisted setting (`Nvm::saveEqualizerStrobeEnabled()`) — it survives power cycles like any other config value and is unaffected by the blacklight reset-on-show-change rule.
- **Long press**: enters configuration mode (see below).
- **Encoder rotation**: per-show, selects that show's own variation (not global) — e.g. `NightriderShow` has 2 variations (pot picks a live hue pair, vs. hue auto-cycles at a pot-controlled rate), `FlameShow` has 4 (see below), `EqualizerShow` has 2 (see below). Each show's current variation is persisted per-show, so switching shows and back recalls where you left it. Selecting either a show or a variation prints a short, abbreviated line to the console (`showName()`, and each show's own `LightShow::variationName()` override — see `LightShow.h`), e.g. `show:FlameSparkle` or `FlameSparkle:hue to white`.
- **Pot**: meaning depends on the active show's current variation (e.g. live hue, animation speed, palette rate) — see each show's own code. Where the pot represents overall show *energy* specifically, it's a **shared global setting** — see below.

**Global energy setting** (`globalEnergyPercent`, `PotEnergyTakeover` struct — `LightShow.h`): `NightriderShow`'s auto-cycle variation, `FlameShow`'s sparkle cadence + hue-drift ceiling (all 4 variations), and `LighthouseShow`'s rotation-speed ceiling all read from one shared 0-100% value instead of each having its own independent pot binding — adjusting the pot in *any one* of these becomes the new value for all the others too, so they all track together rather than drifting apart. `NightriderShow`'s manual-hue variation and `EqualizerShow`'s peak-threshold binding are deliberately **not** part of this group (they aren't "energy" — a color pick and a detection threshold, respectively).

Each show in the group still applies soft takeover independently (`PotEnergyTakeover::reset()`, called from that show's own `start()`): switching *to* one of them never jumps to wherever the pot physically happens to be — it holds the shared value until the pot actually moves since that show became active, then takes over and starts updating the shared value live for every show in the group. Live-only, not persisted to flash — it always starts at 100% (fully energetic) on boot.

Whenever the shared value changes, the console prints `energy:<percent>%` — but throttled to fire once, 1 second after the pot *stops* moving, not on every frame while it's turning (`SettlePrinter` struct, `LightShow.h`). The same throttled-print pattern also covers two pot bindings that didn't otherwise have any console feedback on adjustment: `NightriderShow`'s manual-hue variation prints `hue:<value>`, and `EqualizerShow`'s peak-threshold binding (below) prints `PkThresh:<percent>%`.

### Configuration mode

A 3-screen cycle, and each screen has multiple **subsettings** (Brightness has 3: global, headlight, china; Audio has 3: noise floor, peak threshold, auto-gain enable; PowerTest has 3: hue, saturation, brightness). Button roles inside config mode:

- **Medium press**: advances to the next screen (Brightness → Audio → PowerTest → Brightness), no save. (This *replaces* medium press's old role of cancelling — a normal-mode medium press, and a config-mode medium press, no longer cancel anything.)
- **Double press**: commits whichever subsetting is currently showing, saves it to flash, prints it to the console, flashes once, and exits to normal mode.
- **Short press**: cycles to the next subsetting *within* the current screen (a no-op on the 3 screens that only have one).
- **Long press**: the *only* thing that cancels now — exits without saving, from any screen/subsetting. (Long press still also does double duty from normal mode: it's what enters config mode in the first place, always landing on screen 1.)

The active light show keeps running "invisibly" underneath the whole time config mode is open, so nothing jumps when you cancel.

**Console output**: `Serial.begin(115200)` (raised from the old 9600 — this only affects the debug-print channel on the Due's Programming Port UART, entirely independent of the separate upload/bootloader protocol, so it's safe to raise; `platformio.ini`'s `monitor_speed` is set to match). The console stays quiet except for exactly 3 kinds of line per setting, all short intercaps abbreviations (`GlobalMaxBr`, `HeadliteBr`, `ChinaBr`, `NoiseFl`, `PkThresh`, `AGC`, `Hue`, `Sat`, `Val`) to keep each print cheap. This abbreviated, minimal-character convention is applied to every console print in the codebase, not just config mode — the boot banner (`printWelcome()`) and the normal-mode show/variation-select lines follow the same style (`show:FlameSparkle`, `GlobalMaxBr:57%`, `AGC:Ena`, etc.) rather than spelled-out words:

- On entering a screen or cycling to a subsetting: one line announcing it and its current committed value, e.g. `set GlobalMaxBr:42%`.
- While actively adjusting (pot or encoder past the soft-takeover threshold): a live line that re-prints in place via `\r` (not `\n`) so it overwrites itself instead of scrolling, e.g. `GlobalMaxBr: 57%`, throttled to ~150ms so it doesn't itself start blocking the main loop. Every numeric field on this line is right-justified, fixed-width (`printPad3`/`printPad4` in `CarpetLightLogic.cpp` — 3 digits for 0-100%/0-255 fields, 4 digits for the 0-1023 ADC bin readings), so the line's total length never jitters shorter/longer as a value's digit count changes. Noise floor and peak threshold (2a/2b) are a special case: the usual `NoiseFl:<value>`/`PkThresh:<value>` prefix is dropped entirely (by request — this line is already packed) and replaced with the curated low/mid/high values, shown as percent (`AudioBoard::getLow/getMid/getHigh()`, 0-255 raw, converted — same values `EqualizerShow`/`NightriderShow` react to), then the 7 raw spectrum-shield bins with no label prefix, each still tagged by its own MSGEQ7 datasheet center frequency: `Lo: 12%^ Md: 45%- Hi:  3%_  63:  12 160:  45 400: ... 1k: ... 2.5k: ... 6.25k: ... 16k: ...`. Each of Lo/Md/Hi gets a trailing one-char marker: `^` if it's above the live peak threshold (a "hit" — same shared, live-adjustable value `EqualizerShow`'s bass strobe triggers on), `_` if it's below the live noise floor, `-` in between. A genuinely separate second live line isn't possible on a plain serial terminal without ANSI cursor codes, which is why all of this rides on one line.
- On double-press commit: one line confirming the saved value, e.g. `ok GlobalMaxBr:57%`.

Since a live line ends in `\r` with no `\n`, the next one-shot line (the next setting's announcement, or a commit) would otherwise only partially overwrite it — a `liveLineOpen_` flag in `CarpetLightLogic.cpp` tracks this and inserts one `\n` before that next line whenever a live line was left open, so it always starts clean instead of showing a garbled mix of old and new text.

**Soft takeover**: entering a screen *or* a subsetting doesn't snap the preview to wherever the pot/encoder physically happens to be. It holds at the already-committed value until the pot moves at least 2% of its full range, or the encoder is twisted at all, since entry — only then does it hand over to live tracking (`livePercentFor()` / `resetTakeoverState()` in `CarpetLightLogic.cpp`). This avoids a value jumping the instant you enter a screen or cycle a subsetting.

| # | Screen | Subsetting | Control → | Live preview |
| --- | --- | --- | --- | --- |
| 1a | Brightness: global | 0 | Pot, 0-100% (top half only) | Rope + all megabars + china: solid red at the live value |
| 1b | Brightness: headlight | 1 | Pot, 50-100% (full pot) | Rope + china off; other megabars solid red at the *committed* global brightness (reference); headlight solid red at global × live headlight |
| 1c | Brightness: china | 2 | Pot, 0-100% (full pot) | Rope off; megabars solid red at their normal effective brightness (committed global + headlight); china solid red at global × live china |
| 2a | Audio: noise floor | 0 | Pot, 0-100% (full pot), clamped to never exceed the peak threshold | See below |
| 2b | Audio: peak threshold | 1 | Pot, 0-100% (full pot), clamped to never go below the noise floor | See below |
| 2c | Audio: auto-gain enable | 2 | Encoder, right = on, left = off | See below |
| 3a | PowerTest: hue | 0 | Encoder rotation | See below |
| 3b | PowerTest: saturation | 1 | Encoder rotation | See below |
| 3c | PowerTest: brightness | 2 | Encoder rotation | See below |

**Audio screen visual** (`MagicCarpet::showAudioMeter()`, same for all 3 subsettings): rope is otherwise entirely off; the front strip, both sides, all megabars, and china light up. This whole screen is genuinely live sound-reactive (driven every loop iteration by the same `AudioBoard` accessors `EqualizerShow` itself uses), not a static preview. Noise floor and peak threshold are a matched pair throughout this screen: noise floor is always blue, peak threshold is always red, and (per request) there's no more rainbow-by-position anywhere on it.

- **China (floodlights)**: a very simple, deliberately-not-fancy per-bin confirmation view — one of the 7 raw hardware frequency bins (`AudioBoard::Frequencies_Mono[i]`, scaled 0-1023 → 0-255) drives one china fixture each in blue (same convention as the megabar glow below), so you can visually confirm each individual bin is actually reaching a floodlight. Only 7 of the 8 china fixtures are used, one per bin; the 8th stays off. A fixture's brightness tracks its bin's level directly, except the instant that bin rises above the live peak threshold (`AudioBoard::getPeakThresholdRaw()`, the same shared, live-adjustable value `EqualizerShow`'s bass strobe trigger uses) it flashes solid white for 40ms — edge-triggered (like `EqualizerShow`'s own `isHit`), so it's one quick flash per crossing, not sustained white through a whole loud passage.
- **Front strip** (156 LEDs) splits into 3 equal 52-LED segments in array order: treble, mid, bass (`AudioBoard::getHigh()`/`getMid()`/`getLow()`, 0-255 each, silence-gated but never auto-gained — see "Audio processing" below). Each segment shows a dim (50%) reference gradient across its length — green → yellow/orange (center) → red — with a bright white fill from the segment's start up to the current level's position; a segment goes fully solid white at max level. (Fixed a real pre-existing bug in `AudioBoard::getHigh()` along the way — it was returning `bin_mid` instead of `bin_high`, a copy-paste error that also affected `NightriderShow` and `EqualizerShow`.) While adjusting noise floor or peak threshold (2a/2b, not 2c), each of the 3 segments additionally gets one blue pixel at the live noise-floor position and one red pixel at the live peak-threshold position along that segment's own 0-255 range — a second, per-band confirmation of where each threshold currently sits, updating live as either value is dialed in.
- **Each side strip, true corner to true corner** (286 LEDs — the 352-LED physical channel minus 33 LEDs at each end, so it doesn't wrap into the front/back edges near the corners; see `renderSideIndicator()`) now shows one or two flat-colored 10-LED windows instead of the old position-colored rainbow. On 2a/2b, *both* noise floor and peak threshold are shown at once — a blue window at the live noise-floor percent and a red window at the live peak-threshold percent (back corner = 0%, front corner = 100%) — so you can always see where the other one sits while dialing in whichever you're actively on. On 2c it's a single white window snapped to one end as a plain on/off indicator: back corner = auto-gain off, front corner = on. Left and right mirror each other.
- **All 12 megabars** glow blue, brightness proportional to `AudioBoard::getFullSpectrum()` (silence-gated, auto-gained if enabled) — a live readout of "how loud is it right now," independent of which subsetting is active. While adjusting subsetting 2c specifically, this reflects the *live* (not-yet-committed) auto-gain toggle state rather than the last-committed one, via `AudioBoard::getFullSpectrum(bool)`'s override parameter — this sidesteps needing to revert anything if you cancel out of that subsetting without committing.

Noise floor's and peak threshold's live values *are* temporarily applied to `AudioBoard` while adjusting either one (so the VU meter/megabar glow/china flash react live) — and explicitly reverted back to the last-committed value (`revertAudioLivePreview()` in `CarpetLightLogic.cpp`) if you leave that subsetting any way other than committing (cycling away, advancing, or cancelling). Each is also live-clamped against the other's *current* value while adjusting, so noise floor can never be pushed above peak threshold or vice versa. Auto-gain doesn't need any of this, per the override-parameter approach above.

**Peak threshold** (`AudioBoard::getPeakThresholdRaw()`, 0-100% like noise floor, persisted via `Nvm::savePeakThreshold()`, default ≈31% ≈ the old hardcoded 80/255) is the "this counts as a hit" level shared by `EqualizerShow`'s bass strobe trigger, the Audio screen's per-bin china peak flash, and the console's `^`/`_`/`-` level markers (see below) — previously a compile-time constant, now live-adjustable from this screen exactly like noise floor.

**PowerTest screen** (`MagicCarpet::showPowerTest()`): an A/B test comparing a straight HSV→RGB rendering of a live-editable color against an RGBW power-saving translation of that same color, to gauge how much the translation actually saves and whether it still reads as "the same color." Megabars are off. Unlike every other config screen, this one is driven entirely by the **encoder**, not the pot — rotation edits whichever of hue/saturation/brightness the current subsetting (3a/3b/3c) selects, in steps of 4 per detent; hue wraps around, saturation/brightness clamp at 0/255. All 3 fields are edited as one composite color: cycling 3a→3b→3c with short-press keeps whatever you've dialed into the other two, and double-press commits and persists all 3 at once (`Nvm::saveTestHue/Sat/Brightness`). This screen deliberately ignores the committed global/headlight/china brightness ceiling — hue/sat/brightness here are already a full manual color spec, not a dimming level, so what you see is the two renderings compared at face value.

The rig splits into a front half and a back half, forming an upside-down U (front edge + the front half of both sides) down to the carpet's front-to-back center line:

- **Front half** — rope's front edge, rope's front half of both sides, and china `[0..3]` (the two front-corner pairs) — renders the color straight: `CHSV(hue,sat,val)` converted to RGB, white channel off.
- **Back half** — rope's back edge, rope's back half of both sides, and china `[4..7]` (the two back-corner pairs) — renders the **power-saving translation**: take `min(r,g,b)` from the straight color, subtract it from each of r/g/b (so the smallest channel bottoms out at 0), and add that same amount to the white channel instead. Same apparent hue and brightness, less of it coming from the color LEDs.

## Lighthouse show (`LighthouseShow.h`)

Two independent rotating "lighthouse" beams. Each beam is a **single randomly-drifting angle** that lights up **both its own angle and that angle+180° at once**, in the same color — so 2 independent beams produce 4 lit clusters total, on megabars and on the rope. (This resolves an apparent contradiction in how it was specified: "180° opposite" describes each beam's own two sides, which are always locked together; "independent rotation" describes beam 1 vs. beam 2, which drift completely separately.)

- **Megabars**: per cluster, the nearest of the 12 megabars (30° apart) is full brightness (V=255); its 2 immediate neighbors are half brightness (V=128).
- **Rotation**: each beam's angular velocity is its own independent random walk (`LighthouseShow::RandomWalk`) — smoothly ramps toward a freshly-randomized target (±10°/s step) roughly every second, so it can drift through zero and reverse direction over time, per request. Its ceiling is driven by the **shared global energy setting** (see "Normal mode" above): ±180°/s (0.5Hz) at 0% up to ±360°/s (1Hz) at 100%.
- **Color**: beam 1's hue always increases (never reverses), at a rate that itself random-walks between 0 (frozen) and a max of 1 full spectrum cycle per 20 seconds. Saturation random-walks between 87% and 100%, same mechanism, shared by both beams. Beam 2 always uses beam 1's saturation exactly and the complementary hue (+128) — it has its own independent rotation only, no independent color of its own.
- **Rope**: each cluster lights a `(30/360 of the full loop)`-wide segment at full brightness in its beam's color. Everywhere else, color blends smoothly between the two flanking clusters going around the ring, and brightness dips smoothly from 100% at a segment's edge down to 50% at the gap's midpoint and back to 100% at the next segment — never a hard cut (`triwave8()`-based, no float trig).
- **Overlap**: where two clusters' segments overlap (megabars or rope), hue is additive (summed, wrapped) and brightness is **not** summed — it stays at the flat max, per request.
- **China**: all 8 fixtures share one color, crossfading smoothly between beam 1's and beam 2's color (`sin8()`-based) at a rate equal to the average of the two beams' `|angular velocity|` — reusing that deg/s number directly as a color-phase rate, not a physical angle. Fixed at 80% of the show's own max output (compounds with the user's committed china brightness %, same as every other show's raw output does).
- **Performance**: like `SpeedStripesShow`, this iterates all 1016 rope LEDs every frame. Per that show's hard-learned lesson (see its own README section), per-LED work here deliberately avoids `sinf`/`cosf`/`atan2f` — rope angle geometry is cached once in `start()` (not recomputed per frame), and brightness/color-phase math uses FastLED's table-based `sin8()`/`triwave8()` instead of float trig.

## FlameSparkle show variations (`FlameShow.h`, shown as "FlameSparkle")

The rope runs a classic Fire2012-style heat simulation (cool → disperse → spark → assign-color-from-palette). 4 variations, cycled by encoder rotation:

- **Variation 0/1** ("waterflames"/"flames"): the two original fixed 256-entry palettes (`DarkBlue→Blue→Aqua→White` / `DarkRed→Red→Orange→Yellow`).
- **Variation 2** ("shifting hues"): a 4-stop dark→bright glow, all 4 stops sharing one continuously-drifting hue (full saturation) instead of a fixed color — the hue only ever increases (never reverses), at a rate that itself random-walks, same technique as `LighthouseShow`'s beam hue rate (duplicated locally, not shared — this codebase's convention).
- **Variation 3** ("hue to white"): the same drifting hue, but fading to fixed white at the hot end instead of to a brighter version of itself.
- The **shared global energy setting** (see "Normal mode" above) does two jobs at once, across all 4 variations: it controls the sparkle/fire-sim cadence (via the `delay(potval)` at the bottom of `update()`, inverted so 100% energy = least delay = fastest), and in variations 2/3 specifically it also scales the max hue-drift rate — 0% = 50% of the base max (a full spectrum cycle in as little as 40s instead of 20s), 100% = 100%. Direction and the random-walk mechanism are unchanged, only the ceiling it wanders within shifts. (This migration also fixed a real bug: the hue-rate ceiling used to be wired backwards — pot fully up gave the *lower* 50% ceiling — contradicting both the "up = more energy" convention and this file's own prior description here; the cadence half was already correct.)

**Floodlight sparkle**: the rope's random "spark" pulses (bright, brief, then cooling) now have a matching effect on the floodlights — china is treated as equivalent to megabars (same treatment for both). Each fixture has its own independent "heat" that cools every cycle at the same rate as the rope (so pulses last the same duration) and gets fresh random sparks at the same cadence as the rope's own cool/spark step (tied to the same pot-adjustable delay). The **number** of fixtures sparkling per cycle is proportional, not fixed: the fraction of floodlights sparkling equals the fraction of rope LEDs sparkling (`sparkingRate/255`), rounded to the nearest whole fixture, chosen at random each time. Rendered as a blend on top of the existing audio-tinted base color, using heat itself as the blend fraction, so a spark rises and fades smoothly rather than snapping on/off.

## Equalizer show variations (`BumpingAudioShow.h`, `EqualizerShow`)

2 variations, cycled by encoder rotation like any other show:

- **Variation 0** (default): the original bouncing-chase pattern (unrelated to audio) on the rope, with megabars tinted by bass (red, `getLow()`) and treble (blue, `getHigh()`) using an attack/decay peak-hold (jumps up fast past the live peak threshold, decays 15/frame otherwise).
- **Variation 1**: a dual VU meter, based across the whole front/back of the car rather than a single corner. Bass is based on the *entire back edge* (always lit, not just the two back corners) and grows forward along both sides as it rises (red); treble is based on the entire front edge and grows backward along both sides (blue) — same red=bass/blue=treble convention as variation 0, and the same attack/decay peak-hold ballistics, just kept as separate state. `RIGHT`/`LEFT` are already the exact midpoints of each side run (see the positional constants in `MagicCarpet.h`), so each meter's 100%-level reach along the sides is set to 15% of the true-corner-to-center half-length *past* that midpoint — meaning maxed-out bass and treble simultaneously cross by about that much right around the middle of the car. Where they overlap, the two colors blend 50/50. Brightness of each meter (base edge and side fill alike) scales directly with its current level, not just how far it reaches — a quiet meter is dim, not just short. Megabars and china are off in this variation.
- **Pot** (either variation): live-adjusts `AudioBoard`'s shared peak threshold (the same `PkThresh` value the Audio config screen sets, 0-100% across the full pot; prints `PkThresh:<percent>%` 1s after the pot stops moving) — soft takeover, holds the last-committed value until the pot actually moves since this show became active. This is deliberately *not* part of the shared global energy group above (a detection threshold isn't "energy"), and unlike that group, it *is* tied to a persisted value: leaving this show (switching to any other) reverts it back to whatever's actually saved to flash, so casual tweaking here while a show is running never silently overwrites the committed setting — only dialing it in from the Audio config screen itself does.

**Triple-strobe** (either variation): toggled by a double press while Equalizer is active (see "Normal mode" above) — nonvolatile. When enabled, a qualifying bass hit (same attack threshold against the live peak threshold used for the peak-hold above) flashes all 8 china and the 8 megabars nearest the 4 corners (every megabar except the headlight/left/back/right cardinals — i.e. all except index `0`/`3`/`6`/`9`) full white for 3 pulses, 30ms on each, 20ms gaps between.

Hit suppression prevents the strobe from machine-gunning during a sustained loud passage: the level of the hit that caused the last strobe is remembered, and any further hit that doesn't exceed it is suppressed as long as it's within 3 seconds of the last qualifying hit (strobed or suppressed). A gap of 3+ seconds with no qualifying hits at all clears that memory, so the next hit always strobes fresh regardless of its level.

**Slow desaturation cycle** (both variations): every base color (`clr1`/`clr2` in variation 0, `bassClr`/`trebleClr` in variation 1) has its saturation drift from 100% down to 85% and back over a smooth 30-second sine cycle (`currentSatFraction()`), rather than staying fixed -- a slow, subtle "breathing" of color intensity. `desaturate()` holds hue and value fixed and only raises the minimum channel, so it's exact for these fully-saturated base colors and a no-op on anything already less saturated (e.g. the strobe's white flash, or the "crossing" blend zone once it's already partway desaturated).

## Audio processing (`AudioBoard.h`)

- **Full-spectrum level**: `getFullSpectrum()` is the max of all 7 raw hardware bins (not just the 3 curated into `bin_low`/`bin_mid`/`bin_high` for the VU meter), scaled to 0-255. This is the "how loud is it right now" signal auto-gain and the megabar glow are built on.
- **Rolling peak / auto-gain**: a sliding 4-second-window max (`rollingPeak_`, via a timestamped ring buffer, recomputed every ~30ms poll) sets a slow-moving gain (`255/rollingPeak_`) so quiet passages still reach full scale once enabled. Applying that gain to a sample louder than anything in the last 4s would push it past 255 — clamping to 255 achieves exactly "reduce the scaling factor so this instant peaks at max," just computed as a clamp rather than solving for a per-sample adjusted gain.
- **Rolling average / silence detection**: a separate EMA average (also ~4s time constant, computed with a `dt`-aware alpha so it stays consistent even though the actual poll interval isn't perfectly fixed) tracks a running level. If that average stays below the noise floor for a full 4 seconds continuously, audio is considered "off," and every level getter (`getLow`/`getMid`/`getHigh`/`getFullSpectrum`) returns 0 until the average rises back above the floor (which un-silences immediately, no delay on the way back up). The noise floor is compared against the **raw**, un-gained average specifically, so auto-gain can never mask silence by amplifying noise above the threshold.
- **Auto-gain is off by default** at first boot; it, the noise floor, and the peak threshold all persist to flash once configured (`Nvm.h`).
- Auto-gain deliberately only affects `getFullSpectrum()`, not the 3 curated VU-meter bins (`getLow`/`getMid`/`getHigh`) — those stay raw (but still silence-gated), so this doesn't retroactively change how `NightriderShow`/`FlameShow`/`EqualizerShow` already react to audio.
- **Peak threshold** (`getPeakThresholdRaw()`, live-adjustable + persisted, default ≈31%) is the shared "this counts as a hit" level — see the Configuration mode section above for where it's set and everything that reads it.
- **`getOverallLevelPercent()`**: a best-effort 0-100% estimate of the audio *source's own volume-knob position*, as distinct from `getFullSpectrum()`'s "how loud is it right this instant" (which swings with the music's own dynamics regardless of the knob). Built on the rolling 4s peak (`rollingPeak_`) rather than the instantaneous level or the EMA average, since the loudest moments in a trailing window track a knob's ceiling more reliably than any single instant, while still resetting fast enough (4s) to follow an actual knob turn. Necessarily approximate — a spectrum-analyzer chip has no direct line to the source's own volume control — but the best signal already available here. Not currently called from anywhere; exists as a general-purpose accessor for future use.

## Vehicle speed link (`SpeedLink.h`) and `SpeedStripesShow`

The Due's primary I2C bus (`Wire` — pins 20 (SDA)/21 (SCL) on the communication port header; confirmed against `framework-arduino-sam`'s `variant.h`. `Wire1`, the separate `SDA1`/`SCL1` pins near the USB connectors, is *not* used here) runs as an I2C **slave** at address `0x69`, receiving vehicle telemetry from CANTroller2 (the sibling project, acting as I2C master over its own separate `Wire` bus). Protocol (see CANTroller2's `src/i2cbus.h`, `LightingBox`): no checksum/length byte, I2C START/STOP delimits each packet, top nibble of the first byte is a command code — `0x0`=status flags (1 byte, ignored here), `0x1`=runmode (1 byte — see "Low power mode" below), `0x2`=speed update (2 bytes: low nibble of byte 1 + all of byte 2 form a 12-bit hundredths-of-mph value).

Note CANTroller2 only ever transmits 12 bits of speed (the top nibble of its 16-bit internal value is discarded on the sending side), so the max representable speed is `0x0FFF` = 40.95 mph — a limitation of the sending side, not something fixable from here.

`SpeedLink::getSpeedMph()` returns the last received value; `SpeedLink::isFresh()` reports whether a packet has arrived recently (within 2s by default), so a speed-reactive show can fall back to "stopped" if the link drops instead of freezing on a stale value.

`SpeedLink` also tracks CANTroller2's runmode packets (`0x1R`, sent only on change) via `getRunmode()`/`isLowPower()`. Runmode `1` is `LowPower` (CANTroller2's `globals.h`: `Basic=0, LowPower=1, Standby=2, Stall=3, Hold=4, Fly=5, Cruise=6, Cal=7`).

### Low power mode

Every light show derives its behavior from the live pot reading (`carpet->pot->read()`), so rather than teaching each show about low power, the simulation lives one layer down, in `LedControl::Potentiometer` (`LedController.h`) — every show gets it for free, with no show-file changes:

- **Entering low power** (`SpeedLink::isLowPower()` goes true): `Potentiometer::read()` stops returning the real pot value and instead returns a synthetic value that fades from the pot's actual reading *at the moment low power engaged* down to 0 over 20 seconds, then holds at 0 — same as if someone slowly turned the pot down and left it there.
- **Manual override**: if the real pot moves more than ~2% away from that captured starting position at any point during the fade or the zero-hold, the simulation cancels for the rest of this low-power episode and `read()` goes back to returning the true live value — a real turn always wins. This is edge-triggered: the cancellation sticks even though the vehicle keeps reporting `LowPower` every subsequent loop; only a fresh rising edge (low power ends, then re-engages) re-arms the fade.
- **Exiting low power** (any runmode other than `LowPower` arrives): the simulation cancels immediately, same as a manual override, snapping straight back to live tracking (no fade back up).

This only runs while `ModeShow` is active (`carpet->pot->updateLowPower()` is only called from that branch of `loop()`) — config screens always read the pot live via a separate `Potentiometer::readLive()`, so adjusting brightness/audio/color settings is never affected by low-power state.

**`SpeedStripesShow`**: the two long side rope strips are the only fixtures that physically run front-to-back along the car's 16ft length (the front/back rope edges don't, and stay off — see "Perimeter rope lights" above). Each side strip (352 LEDs) is divided into 4 bands of 88 LEDs each — a quarter of the car's length (4ft) — that alternate lit/dark with a smoothed (not hard-edged) transition between them. The pattern continuously scrolls from front toward back at a rate proportional to the live speed reading (`LEDS_PER_MPH_PER_SEC` in the show, a to-taste constant), giving a sense of forward motion.

**Color**: each stripe (one lit band, i.e. one full period of the wave) gets its own 2-color gradient — one color at its leading edge, one at its trailing edge, fading smoothly across the stripe's own width — rather than the whole show sharing one hue. Which pair of colors a given stripe shows is deterministic from *which period-cycle it is* (`floor(pos/period)`, run through a cheap Knuth multiplicative integer hash), so a specific physical stripe keeps a stable color identity as it scrolls along, without needing to track per-instance state for however many stripes happen to be visible at once. The same slow desaturation cycle as `EqualizerShow` (100%→85%→100% saturation over a 30-second sine) breathes on top.

**Performance note**: the Due's SAM3X8E is a Cortex-M3 with **no hardware FPU**, so `sinf()`/`tanhf()`/`atan2f()`/`cosf()` are all expensive software-emulated calls — and this show's own `update()` keeps running in the background even while a config screen is on-screen (`currLightShow->update()` is called unconditionally every loop, "invisibly," so the show doesn't jump on cancel). An earlier version called `atan2f()` per rope LED per frame (1016×/frame, for a since-removed spin-mode geometry) and recomputed the desaturation `cosf()` per LED instead of once per frame — expensive enough to be felt as general UI choppiness elsewhere in the system, not just in this show. The current version avoids float trig entirely in the per-LED hot path: `currentSatFraction()` is computed once per `update()` call and passed down instead of recomputed per LED, the color hash is plain integer multiply/xor, and the brightness envelope uses FastLED's table-based `sin8()` plus a cheap integer contrast boost instead of `sinf()+tanhf()`.

Every other fixture with any meaningful along-the-length position joins in, sampling that same scrolling pattern at its own physical position — so a given stripe's color matches everywhere it's visible at once (underneath via china, out to the sides via megabars, on top via rope). The sampling function is periodic and defined for any position, including points beyond the car's own front/back corners, so the fixtures pointed forward/backward can preview a stripe approaching and show it receding without any special-casing:

| Fixture(s) | Position sampled |
| --- | --- |
| Megabars `[11,0,1]` (front 3, incl. headlight) | 4ft ahead of the front corner — a preview, well before the stripe arrives |
| China `[1,2]` (aimed along the front edge) | 1ft ahead of the front corner — picks the same stripe up just as it's about to cross onto the carpet |
| Megabars `[5,6,7]` (rear 3) | 4ft behind the back corner — mirrors the front, for the stripe that just left |
| China `[5,6]` (aimed along the back edge) | 1ft behind the back corner — mirrors the front |
| China `[0]`/`[3]` (front-right/front-left, aimed along a side edge) | 1/3 of the way back from the front |
| China `[7]`/`[4]` (back-right/back-left, aimed along a side edge) | 2/3 of the way back from the front |
| Megabars `[2,10]` / `[3,9]` / `[4,8]` (remaining 3 per side) | Same 1/3, 1/2, 2/3 partition by angle — `[2]`/`[10]` line up with the 1/3 china, `[4]`/`[8]` with the 2/3 china |

All of the above (16ft/12ft carpet dimensions, the 4ft/1ft preview distances, and the fade steepness) are hardcoded per user-measured/user-specified values in `SpeedStripesShow.h` — adjust the constants there if the carpet's dimensions ever change.

**Stopped mode**: whenever speed is 0, or `SpeedLink` hasn't heard from the vehicle in over 4 seconds (its own staleness threshold — stricter than `isFresh()`'s 2s default), the pattern simply **freezes in place** — `scrollOffset_` stops advancing, so every stripe (rope, megabar, and china alike, since they all derive their position from that same frozen value) holds exactly where it was the instant the car stopped. After a further **10 continuous seconds** stopped, each stripe's own hue pair starts slowly, randomly meandering across the spectrum — one shared drifting offset (`meanderHue_`, driven by a `RandomWalk`-based rate, same technique as `LighthouseShow`'s beam hue and `FlameShow`'s shifting hues) is added on top of every stripe's own hash-derived base hues, so they all evolve together over time while still looking distinct from each other (their bases differ). Coming to a fresh stop always restarts this from scratch — no carried-over drift from a previous stop.

## Timing convention: use `Timer`, not raw `millis()`

Any "how much time has passed since X" check should use `Utilities.h`'s `Timer` class (`.elapsed()`, `.expired()`, `.expireset()`) rather than hand-rolled `millis() - savedTimestamp` comparisons — it reads more clearly at the call site and keeps the rollover-safe subtraction logic in one place. This is applied throughout: `Potentiometer`'s low-power fade, `MagicCarpet::delayPolling()`, `ArmDmx.h`'s inter-send spacing, `AudioBoard`'s poll-interval and silence tracking, `EqualizerShow`'s strobe timing, and the `RandomWalk` helper duplicated in `FlameShow`/`LighthouseShow`.

Two spots deliberately still use raw `millis()` math, because `Timer` only models "elapsed since I was last reset to now" and can't represent what these need:

- **`PushButtonEdge`'s edge queue and `PushButton::processEdge()`** (`LedController.h`) — edges are timestamped in an interrupt the instant they happen, then replayed later using that recorded (possibly several ms old) timestamp, not "now". A `Timer` reset always means "starting from now"; there's no way to reset one to an arbitrary past moment.
- **`AudioBoard`'s rolling-peak ring buffer** (`peakTimestamps_[]`) — an array of independently-timestamped samples all compared against the *current* time to find which are still inside the trailing 4s window, not a single running duration.

## Future direction

Light shows should eventually be defined in terms of human-meaningful physical zones/effects rather than raw array indices — e.g.:

- **A "side" preset**: a run of megabars + whichever china pair(s) wash that same edge (e.g. "left side floods" = megabars `[3]`,`[4]`,`[5]` (addr 10,13,16) + china `[3]`,`[4]` (addr 55,61)).
- **A "sweep"**: driven by front-to-back distance along the perimeter, applied symmetrically to both sides at once.
- **A "lighthouse" effect**: driven by one rotating angle that picks both the nearest megabar and a same-angle window of perimeter LEDs at once.

None of that abstraction exists in code yet — this doc records the raw hardware facts it would need to be built on.
