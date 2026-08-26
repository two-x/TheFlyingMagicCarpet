# Carpet Lighting Hardware Layout

Reference docs for the carpet's lighting hardware — perimeter rope, megabar floods, china floods. Hand-drawn source diagrams: "carpet information dumpster" doc, Lights section:
https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA

Directions (front/rear/left/right) are relative to the carpet's own direction of travel, not compass or diagram orientation.

**See also:** this file covers firmware/hardware. For the *visualizer tool itself* (architecture, controls, dev workflow), see its in-app "Visualizer help" popup (Settings menu). This file is also viewable inside the visualizer via Settings → "FW readme", kept word-for-word in sync with this one. For Claude Code standing prompts that keep both in sync automatically (plus auto-generated commit messages), see [`claude_dev_prompts.md`](claude_dev_prompts.md).

## Contents

- [Browser visualizer](#browser-visualizer)
  - [Access it live](#access-it-live)
  - [Run it locally instead](#run-it-locally-instead)
  - [Features](#features)
  - [Controls](#controls)
  - [Mic and system audio permissions](#mic-and-system-audio-permissions)
  - [Also available as a Claude Artifact](#also-available-as-a-claude-artifact)
- [Perimeter rope lights](#perimeter-rope-lights)
- [Megabars](#megabars)
- [China lights](#china-lights)
- [Brightness system](#brightness-system)
- [Encoder and button UI](#encoder-and-button-ui)
  - [Button press tiers](#button-press-tiers)
  - [Perimeter flash feedback](#perimeter-flash-feedback)
  - [Normal mode](#normal-mode)
  - [Configuration mode](#configuration-mode)
- [Lighthouse show](#lighthouse-show)
- [FlameSparkle show variations](#flamesparkle-show-variations)
- [Equalizer show variations](#equalizer-show-variations)
- [Audio processing](#audio-processing)
- [Vehicle speed link and SpeedStripesShow](#vehicle-speed-link-and-speedstripesshow)
  - [Low power mode](#low-power-mode)
- [Timing convention](#timing-convention)
- [Future direction](#future-direction)

## Browser visualizer

A self-contained single-file HTML/JS/Canvas simulator, built so a remote developer can debug firmware, add shows, and tune existing ones from `src/*.h` alone — no physical car needed. **Not an independent creative sandbox**: its light-show logic is not a JS port of anything — the real firmware (`src/*.h`, every real show, `CarpetGeometry.h`, `AudioBoard.h`'s math, config-mode navigation, all of it) is compiled to WebAssembly and run directly in-browser, unmodified, and the visualizer reads its real LED output back out of WASM memory to draw. There is nothing to "keep in sync": a bug in real firmware shows up here because it's the same compiled code, not a copy of it. Legitimate JS-only content is limited to simulating things *external* to firmware — audio input (mic/tab/file/synthetic tone standing in for the real MSGEQ7 chip), the vehicle speed SpeedLink would deliver, and the pot/encoder/button inputs themselves — plus a separate, structurally unreachable "Interface" role (`interface-mode.js`) for a future low-bandwidth radio-link mode that can't stream real LED bytes and so deliberately re-implements rendering; that mode is dormant until Project B's radio link exists. Renders a schematic top-down view with every fixture (rope, megabars, china) responding as on real hardware.

### Access it live

Every push to any branch auto-deploys that branch's `carpet-visualizer.html` via GitHub Actions (`.github/workflows/deploy-visualizer.yml`) to GitHub Pages:

```text
https://two-x.github.io/TheFlyingMagicCarpet/<your-branch>/carpet-visualizer.html
```

`<your-branch>` = your branch name with `/` flattened to `-` (`feature/foo` → `feature-foo`). Push, wait for the "deploy-visualizer" Actions check to go green (usually under a minute), then load that URL — always your latest pushed commit, no build/publish step. It's a public URL, shareable with anyone to view or drive your in-progress work live.

### Run it locally instead

No build step, server, or dependencies — one HTML file. From a terminal (e.g. VSCode's integrated terminal, `` Ctrl+` ``):

```sh
open tools/visualizer/carpet-visualizer.html
```

(macOS `open`; Windows/Linux: file manager, or `start tools\visualizer\carpet-visualizer.html` / `xdg-open`.) Dragging the file from VSCode's editor tab into a browser usually **fails** (VSCode hands the browser an internal reference its sandbox rejects) — use the command above, drag from Finder/Explorer instead, or right-click in VSCode's Explorer sidebar → Copy Path → paste into the address bar.

### Features

- **Light shows**: Nightrider, Lighthouse, FlameSparkle, Equalizer, SpeedStripes — matching firmware, each with its own variations where the real show has them.
- **Config-mode screens**: Noise/Peak meter, Hit decay meter, Foresight adjust (hit-prediction style rides along on this same screen) — the same audio-tuning screens as the real controller, navigable identically (see Controls below).
- **Audio-reactive**, from 4 sources: **Mic**; **System** (shared browser tab, or whole-screen audio on Windows — plays speaker output directly, no OS loopback tool needed); **File** (any local audio file); or **Tone** (synthetic generator with genre patterns — simple, house, trap, boom-bap/hip-hop, chris (bass music), lo-fi — for testing without real music). Trap/boom-bap/chris/house synthesize bass + treble only, no midrange, by request.
- **Landscape/portrait viewport** toggle (top-left) — landscape (default) rotates the car 90° so front points right, suited to a wide monitor; portrait matches the car's true top-down aspect.
- Scrollable/pinchable **zoom** (20–100ft simulated viewer distance), capped so you can't zoom out past the point the ground already fills the view.
- A **Quick Adjust** panel — direct, always-live overrides (brightness levels, hit decay/foresight (plus hit-prediction style), blacklight, fixture-dot coloring), alongside (not replacing) the config-mode path to the same values.
- A live 4-band (bass/mid/treble/full) audio level meter, doubling as the tone generator's expand/collapse control.

### Controls

The control panel at the top of the sidebar mimics the real hardware's pot, rotary encoder, and 5 button-press lengths:

- **Pot knob**: click and hold, then move the mouse — the knob tracks the angle from its center to the cursor while held (0% = 150° counterclockwise of straight up, 50% = straight up, 100% = 150° clockwise).
- **Encoder knob**: click its left half for one detent left (counterclockwise), right half for one detent right — curved arrows show which side does which, flashing on whichever just fired. Holding past 500ms auto-repeats at 10 detents/sec. The `pos N` readout is a session convenience, not real hardware state — resets to 0 five seconds after mouse release. In normal show mode the encoder cycles the active show's variation (matching firmware — `NightriderShow.h`/`BumpingAudioShow.h`, both reading `carpet->encoder->readPositionDelta()`); in config mode it drives whichever subsetting the status bar shows.
- **Left / Right / Pot / Short / Med / Long / XL / Double**: a live 4×2 table under the knobs (replacing the old 5 clickable buttons) states in plain language what each input currently does (e.g. "Short: next show (lighthouse)" in normal mode, "Pot: set headlight brightness (50-100%)" on that config subsetting), updating live. Each entry's bold first word flashes orange for 150ms when that input fires; Pot's stays lit while dragged. The 5 press-lengths still trigger exactly as on real hardware — click-and-hold the encoder's own center pushbutton for the matching duration (exact thresholds under "Button press tiers" below) — the table just describes them now instead of also being clickable.

Everything else (show/variation dropdowns, speed, brightness sliders, audio source buttons) is a direct, labeled, always-live control — hover the `?` tooltips for anything not obvious.

### Mic and system audio permissions

**If Mic or System audio silently stays at 0**, or the browser complains about permissions, the OS is blocking capture:

- **macOS**: System Settings → Privacy & Security → Microphone → enable your browser. "System" (tab/system audio) additionally needs Screen & System Audio Recording → enable the browser (Chrome's tab-audio picker uses this permission even without capturing video). Fully quit and reopen the browser after granting either — a mid-session grant needs a relaunch.
- **Windows**: Settings → Privacy & security → Microphone → "Let apps access your microphone" → enable your browser. For "System" audio, use the browser's "Share tab/system audio" checkbox in the screen-share picker (if greyed out, also check "Let desktop apps access your microphone" — some browsers route system-audio capture through that same pipeline).

### Also available as a Claude Artifact

Separately from the GitHub Pages URLs above, this file also gets published as a shareable Claude Artifact when worked on through Claude — ask Claude for the current link (not tracked in git).

## Perimeter rope lights

Only **4 continuous physical strips** exist. `NEO0_OFFSET`..`NEO3_OFFSET` (reading from `ropeLeds[]`) are the ones in use; `NEO4_OFFSET`..`NEO7_OFFSET` are leftover offsets from a never-wired 8-strip plan, removed from code.

`NUM_NEOPIXEL_STRIPS` stays at **8** in code despite only 4 strips being wired — don't "fix" this to 4. `MagicCarpet::show()`'s 4 active `convertNeoArray()` calls write into the `ropeShowLeds` FastLED output buffer at strip-slots **0, 1, 4, and 5** (not 0-3), so the buffer must stay sized for 6 slots or those slot-4/5 writes run off the end. Also **not confirmed**: which physical pins (of `NEO_PIN0`-`NEO_PIN7`) the 2 "extra" strips are wired to — don't assume "the first 4 pins."

Each real strip runs straight through the corner(s) it passes — wiring does **not** break at a corner. Splits instead land 30 LEDs in from each true corner, along the front/rear edges:

| Channel | Length | Runs along | Local index 0 | Local index max |
| --- | --- | --- | --- | --- |
| CH1 | 156 LEDs | Front edge | near front-right | 155, near front-left |
| CH2 | 352 LEDs | Right side | near front-right | 351, near rear-right |
| CH3 | 156 LEDs | Rear edge | near rear-left | 155, near rear-right |
| CH4 | 352 LEDs | Left side | near rear-left | 351, near front-left |

(30 + 156 + 30 = 216 front/rear width; 30 + 292 + 30 = 352 side height.)

`ropeLeds[]` is logically one loop that starts/wraps at FRONT (see the `FRONT`/`RIGHT`/`REAR`/`LEFT`/etc. positional constants in `MagicCarpet.h`). `show()` reverses some sub-ranges before pushing to hardware, to reconcile logical vs. physical wire order. The `ropeLeds[]` global-index ↔ per-channel local-index mapping above hasn't been independently verified — trust the code constants over this doc's channel-local numbers if they ever conflict.

**Bus timing** (`NEO_PORT_BANK = WS2811_PORTD`, `FastLED.addLeds<NEO_PORT_BANK,NUM_NEOPIXEL_STRIPS>()`): checked against FastLED's own source (`chipsets.h`/`FastLED.h`), not assumed. `WS2811_PORTD` resolves to an `InlineBlockClocklessController` at `NS(320),NS(320),NS(640)` — FastLED's standard **800kHz** WS281x profile (vs. a slower 400kHz WS2811 mode this isn't using), the fastest bit rate the protocol spec allows. More importantly, `PORTD` is genuinely **parallel**: one Port D byte-write toggles up to 8 pins per bit-time, so all wired strips clock out simultaneously — already the fastest multi-strip approach FastLED offers on a non-DMA MCU like the SAM3X8E, far better than looping single-strip `addLeds()` calls. Per-frame cost is still real: every lane clocks the *longest* wired channel's pixel count (`NUM_NEO_LEDS_PER_STRIP`, ≈470 RGBW-inflated slots from the 352-LED side channels), so `show()` blocks ~470 × 24 bits × ~1.28ns/bit ≈ 14ms with interrupts disabled, every frame — inherent to WS281x at this LED count, not reducible via FastLED config. Side effect: the two shorter 156-LED front/rear channels get silently padded to that same ~470-slot length each frame — wasted bus time, fixable only by rewiring which run lands in which lane.

**DMA alternative** (`Ws281xDma.h` — written, compiles, not wired into `MagicCarpet::show()` yet, pending a pin conflict with the DMX RS-485 output on USART0): 3 of the SAM3X8E's 4 USARTs run in "SPI Master" synchronous mode (`US_MR_USART_MODE_SPI_MASTER`, continuous bit-shifting) plus the chip's one true SPI peripheral, each with its own PDC (DMA) channel and dedicated pin — 4 genuinely parallel, hardware-DMA-fed outputs instead of one CPU-bit-banged bank. Each WS281x bit encodes as 3 output bits at a 2.4MHz shift rate (exact 84MHz/35 divide), giving standard 1250ns timing. Once a channel's buffer loads, the CPU is free — no interrupts-disabled busy-wait — and each channel clocks only its own real LED count, no cross-lane padding. Confirmed (via FastLED's source) it never uses DMA on this chip for any Due driver — genuinely new capability, not a library flag.

## Megabars

Mounted in a ring, roughly horizontal (aimed slightly downward but mostly level), each pointed 30° further around than the last — together extending the light pattern outward in a full circle around the platform.

`megabarLeds[i]` corresponds to DMX address `(1 + 3*i)`.

| Index | Addr | Direction |
| --- | --- | --- |
| `[0]` | 01 (×2 fixtures, same address) | Straight ahead — **front** (0°) |
| `[1]` | 04 | 30° |
| `[2]` | 07 | 60° |
| `[3]` | 10 | 90° — straight out the **left** side |
| `[4]` | 13 | 120° |
| `[5]` | 16 | 150° |
| `[6]` | 19 | 180° — straight out the **rear** |
| `[7]` | 22 | 210° |
| `[8]` | 25 | 240° |
| `[9]` | 28 | 270° — straight out the **right** side |
| `[10]` | 31 | 300° |
| `[11]` | 34 | 330° — rear toward front |

Address 01 is the only doubled-up fixture (two physical units, same DMX address); every other address is a single unit. Going in increasing address order, the ring sweeps front → left → rear → right → back to front.

In code, `megabarLeds[HEADLIGHT_INDEX]` (index `[0]`, addr 01) is called the **headlight** and gets its own brightness control, separate from the rest — see "Brightness system" below.

## China lights

Very bright, mounted in pairs at the 4 corners. Aimed primarily **straight down** (unlike the megabars), but X/Y aim still matters — each beam scatters well past its bright spot. Together they flood the ground under the whole perimeter, making the platform look like it's floating on light.

The 2 extra channels beyond RGBW are `CRGBWUA::u` (**UV/blacklight** — confirmed; RGBWUA is a standard 6-channel layout, and `CRGBW.h`'s union aliases `u` as `black`) and `.a` (amber, unconfirmed — `CRGBW.h` still marks it `// TODO: figure out what a is for...`). **Bugfix**: `u`/`a`'s declaration order was swapped — DMX has no packing constraint (unlike NeoPixel), so `dmx_send()` sends `CRGBWUA`'s raw memory directly, meaning struct order *is* DMX channel order. `u` used to be declared first (5th channel), but hardware confirmed that's actually amber, with UV on the 6th — enabling "blacklight" was visibly lighting amber instead. Fixed by swapping the order.

At each corner, the 2 fixtures split duty, one per adjoining edge. Each bright spot lands ~1/3 of the way along its assigned edge, measured **from its own corner toward the far corner** (light keeps scattering past that point).

| Index | Addr | Corner | Aimed along | Bright spot |
| --- | --- | --- | --- | --- |
| `[0]` | 37 | Front-right | Right edge | ~1/3 rear from front |
| `[1]` | 43 | Front-right | Front edge | ~1/3 in from right |
| `[2]` | 49 | Front-left | Front edge | ~1/3 in from left |
| `[3]` | 55 | Front-left | Left edge | ~1/3 rear from front |
| `[4]` | 61 | Rear-left | Left edge | ~1/3 forward from rear |
| `[5]` | 67 | Rear-left | Rear edge | ~1/3 in from left |
| `[6]` | 73 | Rear-right | Rear edge | ~1/3 in from right |
| `[7]` | 79 | Rear-right | Right edge | ~1/3 forward from rear |

## Brightness system

All brightness control lives in `MagicCarpet`, expressed only as **percentages** (floats) — shows/UI never touch raw 0-255 values; `percentToRaw()` is the sole, private conversion point. Three independent percentages compound multiplicatively via plain linear scaling (`scale8()`). Megabar/china are deliberately **not** gamma-corrected in this math (gamma is nonlinear — applying it after scaling breaks the linear addition the headlight math depends on); gamma applies only to the rope, automatically, inside `show()`. Only `FlameShow` gamma-corrects its own megabar output on top of this — a pre-existing inconsistency, not part of the brightness system itself.

**Bugfix (`CRGBW.h`'s `convertNeo()`)**: the rope's white channel used to get zero gamma correction while R/G/B always did (an old comment flagged this as a conscious but unresolved call). Gamma compensates for the nonlinear relationship between PWM duty cycle and *perceived* brightness — a property of vision, not LED color — so it applies to white exactly as much as R/G/B. Leaving W raw looked inconsistent next to corrected R/G/B (surfaced via the PowerTest A/B screen: the straight-RGB half desaturated smoothly, the RGBW "power-saving" half — which moves color into W — didn't, reading as white overpowering at low-to-mid values). Fixed by adding `gamW()`, reusing `gammaG`'s table (uncapped 0-255, unlike `gammaR`/`gammaB` which are capped for R/B white-balance — that cap doesn't obviously apply to white). A general fix to the shared NeoPixel conversion path, affecting every show's use of the rope's white channel, not just PowerTest.

| Setting | Range | Default | Scales |
| --- | --- | --- | --- |
| `globalBrightness_` | 0-100% | 100% (unrestricted) | Everything: rope, all megabars, all china |
| `headlightBrightness_` | **50-100%** | 50% | Just the headlight (`megabarLeds[HEADLIGHT_INDEX]`), on top of global |
| `chinaBrightness_` | 0-100% | 100% (unrestricted) | All of `chinaLeds[]`, on top of global |

**Headlight brightness is a direct per-fixture percentage** of the global max — same units as every other setting, no special conversion. Floored at 50% rather than 0% since below that the pair would look dimmer than any single megabar, not a useful setting. At 50% (default) each fixture runs at half the global max — since the headlight is two physical fixtures at the same DMX address, their output adds linearly, so the pair looks like one normal megabar at 100%. At 100% each fixture runs at full global max — the pair looks twice as bright as one normal megabar. (This additive reasoning is why megabar/china brightness math stays gamma-free — see above; gamma-correcting before this addition would badly under-represent combined output.)

`MagicCarpet::applyBrightnessCeiling()` applies all this every frame, after a show writes its output and before `show()` pushes to hardware. All three settings persist to flash (`Nvm.h`) across power cycles.

## Encoder and button UI

### Button press tiers

Classified by hold duration (`LedControl::PushButton` in `LedController.h`). Short/medium/long decide **at release**; extra-long and double decide **live**, the instant detected:

| Press | Duration | When it fires |
| --- | --- | --- |
| Short | < 300ms | ~250ms after release (see double-press below) |
| Medium | 300-1500ms | At release |
| Long | 1500-3000ms | At release |
| Extra-long | ≥ 3000ms | Live, the instant 3000ms is crossed while still held |
| Double | 2nd press starts within 250ms of a short press's release | Live, the instant the 2nd press begins |

Once extra-long or double fires live during a hold, nothing else can fire for the rest of that hold (not the other one, not a threshold flash, not a release classification) — locked until physically released.

A short press is deliberately delayed ~250ms past release: a second press in that window becomes a double instead (firing immediately) and the original short press never fires; otherwise the short press finally fires ~250ms late.

**Edge capture is interrupt-driven, not polled.** A `CHANGE` interrupt (`PushButtonEdge::isr()`) timestamps every down/up transition into a small queue the instant it happens; `PushButton::update()` drains that queue each loop and replays every edge by its own recorded timestamp, rather than sampling the pin's live state once per call. This matters because the main loop is dominated by `FastLED.show()` (~14ms, mostly interrupts-disabled during rope bit-banging — see the timing report below); a plain polled `digitalRead()` could miss a short tap entirely between two `update()` calls. An edge interrupt still can't fire during FastLED's own disabled-interrupt window, but that window is much narrower than a full loop iteration, so this meaningfully shrinks (not eliminates) the blind spot.

(Aside: `ENCODER_SW_PIN` and `POT_ANALOG_PIN` in `MagicCarpet.h` both use literal `3` and used to carry a `FIXME` suspecting a pin conflict — a false alarm. On the Due, `analogRead()` auto-offsets any value below `A0` by adding `A0` (`wiring_analog.c`: `if (ulPin < A0) ulPin += A0;`), so `POT_ANALOG_PIN 3` addresses pin **A3**, while `ENCODER_SW_PIN 3` is read via `digitalReadDirect()`, addressing raw digital pin **D3** — different physical pins despite the shared literal. Confirmed against the actual `analogRead()` implementation, not assumed.)

### Perimeter flash feedback

The rope perimeter flashes white as UI feedback (`MagicCarpet::flashRope()` — 55ms white padded by 15ms black each side; 85ms gap between flashes in one call; blocking, but polls the button throughout via `delayPolling()` rather than going fully blind — a sequence can run up to ~255ms, long enough to otherwise drop part of a rapid follow-up double-press):

- **1 flash**, live, the instant a hold crosses the long-press threshold (1500ms) — no feedback for the medium-press threshold (300ms), by request
- **1 flash** when a double press commits a config setting/subsetting (not in normal mode, where double press does other things — see below)
- **2 flashes** when a medium press advances to the next config setting (a normal-mode medium press currently does nothing)

### Normal mode

- **Short press**: cycles to the next light show (Nightrider → FlameSparkle → Equalizer → SpeedStripes → Lighthouse → ...). Persisted to flash immediately.
- **Extra-long press**: toggles the whole rig on/off. Always boots **on** — intentionally *not* persisted.
- **Double press**: toggles blacklight (china's UV channel, `CRGBWUA::u` — see "China lights" above) full on/off, independent of the active show — **except while Equalizer is active**, where double press instead toggles its own triple-strobe setting (see "Equalizer show variations" below); blacklight isn't reachable by double press during Equalizer. Blacklight is intentionally volatile — always boots **off**, never persisted, force-reset to off on every show change (short press), not just rig-off. The Equalizer strobe toggle is a separate, ordinary persisted setting (`Nvm::saveEqualizerStrobeEnabled()`) — survives power cycles like any other config value, unaffected by the blacklight reset-on-show-change rule.
- **Long press**: enters configuration mode (see below).
- **Encoder rotation**: per-show, selects that show's own variation (not global) — e.g. `NightriderShow` has 2 (pot picks a live hue pair, vs. hue auto-cycling at a pot-controlled rate), `FlameShow` has 4 (below), `EqualizerShow` has 2 (below). Each show's variation persists per-show, so switching shows and back recalls where you left it. Selecting a show or variation prints an abbreviated console line (`showName()`, each show's own `LightShow::variationName()` — see `LightShow.h`), e.g. `show:FlameSparkle` or `FlameSparkle:hue to white`.
- **PressLeft / PressRight** (`LedController.h`'s `PushButton::handleRotationDuringHold()`): rotating the encoder WHILE its button is held cancels whatever press-tier was in progress (short/medium/long/extra-long/double all voided for the rest of that hold) and fires this instead, immediately, repeatably for further rotation in the same hold. Default (normal mode only): nudges the committed peak threshold ±1%/detent, applied live and debounce-saved to flash (~1s after adjustments stop, so rapid detents don't hammer flash — same idea as `SettlePrinter`'s throttled prints). A show may override via `LightShow::onPressLeft()`/`onPressRight()` instead (return `true` to suppress the default) — none currently do.
- **Pot**: meaning depends on the active show's variation (live hue, animation speed, palette rate — see each show's code). Where the pot represents overall show *energy* specifically, it's a **shared global setting** — see below.

**Global energy setting** (`globalEnergyPercent`, `PotEnergyTakeover` struct — `LightShow.h`): `NightriderShow`'s auto-cycle variation, `FlameShow`'s sparkle cadence + hue-drift ceiling (all 4 variations), and `LighthouseShow`'s rotation-speed ceiling all read one shared 0-100% value instead of each having an independent pot binding — adjusting the pot in *any one* updates the value for all, so they track together instead of drifting apart. `NightriderShow`'s manual-hue variation and `EqualizerShow`'s peak-threshold binding are deliberately **not** part of this group (not "energy" — a color pick and a detection threshold, respectively).

Each show in the group still applies soft takeover independently (`PotEnergyTakeover::reset()`, from that show's own `start()`): switching *to* one never jumps to wherever the pot physically sits — it holds the shared value until the pot moves since that show became active, then takes over live for every show in the group. Live-only, not persisted — always starts at 100% (fully energetic) on boot.

Whenever the shared value changes, the console prints `energy:<percent>%`, throttled to fire once, 1 second after the pot *stops* moving, not every frame while turning (`SettlePrinter` struct, `LightShow.h`). The same throttled-print pattern covers two other pot bindings that otherwise had no console feedback: `NightriderShow`'s manual-hue variation prints `hue:<value>`, and `EqualizerShow`'s peak-threshold binding (below) prints `PkThresh:<percent>%`.

### Configuration mode

A 3-screen cycle, each with multiple **subsettings** (Brightness: 3 — global, headlight, china; Audio: 6 — noise floor/peak threshold merged into one, hit decay rate, audio foresight (hit-prediction *style* rides along on this same screen — see below, there's no separate prediction-distance subsetting), AGC mode, sound-reactivity enable, auto-peak enable; PowerTest: 3 — hue, saturation, brightness). Button roles inside config mode:

- **Medium press**: advances to the next screen (Brightness → Audio → PowerTest → Brightness), no save. (Replaces medium press's old cancel role — neither normal- nor config-mode medium press cancels anything anymore.)
- **Double press**: commits the current subsetting, saves to flash, prints to console, flashes once, exits to normal mode.
- **Short press**: cycles to the next subsetting *within* the current screen (no-op on the 3 single-subsetting screens) — **except** on 2 Audio subsettings where it means something else instead (below): merged noise-floor/peak-threshold (2a) and hit-decay-rate (2b) — each has its own one-off toggle (which value the pot targets / simulated vs. live signal) before falling through to the usual advance.
- **Long press**: the *only* thing that cancels now — exits without saving, from any screen/subsetting. (Also still what enters config mode from normal mode, always landing on screen 1.)

Each screen mode and subsetting is a named enum (`ShowMode`; `AppMode`; `BrightnessSubsetting`/`AudioSubsetting`/`PowerTestSubsetting` in `CarpetLightLogic.cpp`), not a raw index — `configSubsetting` stays a plain `uint8_t` (its meaning depends on the active screen), but every comparison uses the named constant (`SubAudioAutoGain`, not `1`). Each multi-variation show similarly declares its own `Variation` enum instead of comparing raw numbers.

**Noise floor and peak threshold used to be separate subsettings.** Short press had no meaning in either beyond generic cycling, so they're merged into one (2a) — short press toggles which the pot targets. Switching targets first reverts the other's in-progress live preview to its last-committed value (so nothing uncommitted lingers applied to `AudioBoard`) and resets pot soft-takeover so the new target doesn't jump.

The active light show keeps running "invisibly" underneath the whole time config mode is open, so nothing jumps on cancel.

**Blackout** (`MagicCarpet::clear()` — clears rope + megabars + china together; predates this, was already the standard "zero every light" call): now also runs on entering/advancing/cycling/leaving config mode (`enterConfigMode()`/`cycleSubsetting()`, and both the commit and cancel exit paths in `loop()`), on every show change and every variation change, and once at the end of `setup()` — a brief guaranteed-blank frame at every one of those transitions, so nothing from a previous screen/show/variation ever visibly bleeds into the next one even for a frame.

**Console output**: `Serial.begin(115200)` (raised from 9600 — only affects the Due's Programming Port debug channel, independent of the upload/bootloader protocol, so safe to raise; `platformio.ini`'s `monitor_speed` matches). The console stays quiet except for 3 kinds of line per setting, all short intercaps abbreviations (`GlobalMaxBr`, `HeadliteBr`, `ChinaBr`, `NoiseFl`, `PkThresh`, `AGC`, `Hue`, `Sat`, `Val`) to keep prints cheap — applied codebase-wide, not just config mode (boot banner, show/variation-select lines: `show:FlameSparkle`, `GlobalMaxBr:57%`, `AGC:Ena`):

- Entering a screen/subsetting: one line announcing it and its committed value, e.g. `set GlobalMaxBr:42%`.
- While adjusting (past soft-takeover threshold): a live line re-printing via `\r` (not `\n`) so it overwrites in place, e.g. `GlobalMaxBr: 57%`, throttled to ~150ms. Numeric fields are right-justified, fixed-width (`printPad3`/`printPad4` — 3 digits for 0-100%/0-255, 4 for 0-1023 ADC readings) so line length never jitters. Noise floor/peak threshold (2a) is a special case: the line leads with the setting actually being adjusted (`PkThresh: 68%` or `NoiseFl: 18%`, whichever short press currently targets), then curated low/mid/high **normal** levels (`AudioBoard::getBandNormalPercent(Band)`, percent), then the 7 raw bins unlabeled, each tagged by its MSGEQ7 center frequency: `PkThresh: 68% Lo: 12%^ Md: 45%- Hi:  3%_  63:  12 160:  45 400: ... 1k: ... 2.5k: ... 6.25k: ... 16k: ...`. Lo/Md/Hi each get a trailing marker (checked against **raw**, pre-squelch/AGC): `^` above peak threshold, `_` below noise floor, `-` between. No separate 2nd live line is possible on a plain serial terminal without ANSI codes, hence all this on one line.
- On double-press commit: one line confirming, e.g. `ok GlobalMaxBr:57%`.

Since a live line ends in `\r` with no `\n`, the next one-shot line would only partially overwrite it — a `liveLineOpen_` flag tracks this and inserts one `\n` first whenever a live line was left open, so the next line always starts clean.

**Soft takeover**: entering a screen or subsetting doesn't snap the preview to wherever the pot/encoder physically sits — it holds the committed value until the pot moves ≥2% of its range, or the encoder turns at all, since entry (`livePercentFor()`/`resetTakeoverState()`). Avoids a value jumping the instant you enter a screen or cycle a subsetting.

| # | Screen | Subsetting | Control → | Live preview |
| --- | --- | --- | --- | --- |
| 1a | Brightness: global | 0 | Pot, 0-100% (top half only) | Rope + all megabars + china: solid red at the live value |
| 1b | Brightness: headlight | 1 | Pot, 50-100% (full pot) | Rope + china off; other megabars solid red at **full max** (a fixed, unambiguous reference — previously the committed global brightness, which made comparison meaningless whenever global itself was dim); headlight solid red at global × live headlight |
| 1c | Brightness: china | 2 | Pot, 0-100% (full pot) | Rope off; megabars solid red at their normal effective brightness (committed global + headlight); china solid red at global × live china |
| 2a | Audio: noise floor / peak threshold (merged) | 0 | Pot, 0-100% (full pot), targets whichever of the two short press last selected, each clamped to never cross the other | See below |
| 2b | Audio: hit decay rate | 1 | Pot, 0-1000ms (full pot); short press toggles simulated/live signal instead of cycling | See below |
| 2c | Audio: audio foresight | 2 | Pot, 0-700ms (full pot); encoder cycles hit-prediction style (otherwise idle on this subsetting) | See below |
| 2d | Audio: AGC mode | 3 | Encoder, cycles Off/Band/Full (`AGCMode` — see "Audio processing" below) | See below |
| 2e | Audio: sound-reactivity enable | 4 | Encoder, right = on, left = off — global kill switch for all sound-reactive light behavior | (no dedicated visual, reuses the 2a/2d-style meter) |
| 2f | Audio: auto-peak mode | 5 | Encoder, cycles Off/Full/Bin, clamped at both ends -- same convention as AGC mode (see "Auto-peak" below) | Front-edge neo split: 3 equal thirds, one lit per mode |
| 3a | PowerTest: hue | 0 | Encoder rotation | See below |
| 3b | PowerTest: saturation | 1 | Encoder rotation | See below |
| 3c | PowerTest: brightness | 2 | Encoder rotation | See below |

**Audio screen visual** (`MagicCarpet::showAudioMeter()`, subsettings 2a and 2d only — 2b/2c have their own dedicated screens, below): rope off; front strip, both sides, all megabars, and china light up. Genuinely live sound-reactive (same `AudioBoard` accessors `EqualizerShow` itself uses), not a static preview. Noise floor and peak threshold are a matched pair: noise floor always blue, peak threshold always red, no rainbow-by-position. On the front strip's 3 VU segments, each value gets its own single-pixel dot; since noise floor is clamped to never exceed peak threshold, the dots can only collide, never cross — on collision the blue dot nudges back 1px (or red forward 1, at the edge) so both stay visible.

- **China (floodlights)**: a simple per-bin confirmation view — one of the 7 raw bins (`AudioBoard::Frequencies_Mono[i]`, scaled 0-1023 → 0-255) drives one china fixture each in blue, confirming each bin reaches a floodlight. 7 of 8 fixtures used, one per bin; the 8th stays off. Brightness tracks the bin's level directly, except it flashes solid white for 40ms the instant that bin crosses the live peak threshold (`AudioBoard::getPeakThresholdRaw()`, same value `EqualizerShow`'s bass strobe uses) — edge-triggered, one flash per crossing.
- **Front strip** (156 LEDs) splits into 3 equal 52-LED segments: treble, mid, bass (`AudioBoard::getBandNormalPercent(BandTreble/BandMid/BandBass)`, silence-aware, never AGC-boosted). Each shows a dim (50%) reference gradient — green → yellow/orange → red — with a bright white fill up to the current level; fully solid white at max. On 2a, each segment also gets a blue pixel at the live noise-floor position and a red pixel at the live peak-threshold position.
- **Each side strip, true corner to true corner** (286 LEDs — the 352-LED channel minus 33 at each end, so it doesn't wrap into front/rear edges; `renderSideIndicator()`) shows flat-colored 10-LED windows. On 2a, both noise floor (blue) and peak threshold (red) show at once regardless of pot target (rear corner = 0%, front = 100%). On 2d, a single white window snaps to one of 3 positions (rear = AGCoff, middle = AGCband, front = AGCfull), reflecting the live AGC mode. Left/right mirror.
- **All 12 megabars** glow blue, brightness proportional to `AudioBoard::getBandNormalPercentPreview(BandFull, mode)` (silence-aware, AGC-boosted per `mode`) — a live "how loud right now" readout regardless of subsetting. While on 2d, `mode` is the *live* (uncommitted) AGC mode selection instead of the committed one — this Preview overload recomputes the boosted level under any given mode on the fly (the AGC scan already tracks both bin-level and full-spectrum rolling peaks every poll regardless of which mode is active), so previewing a live mode change needs no mutate/revert of `AudioBoard`'s own committed state, unlike noise floor/peak threshold below.

Noise floor and peak threshold's live values *are* temporarily applied to `AudioBoard` while adjusting (so VU/glow/flash react live), reverted to last-committed (`revertAudioLivePreview()`) on leaving that subsetting any way but committing. Each is live-clamped against the other's current value so neither can cross. AGC mode needs none of this, per its override-parameter approach.

**Peak threshold** (`AudioBoard::getPeakThresholdRaw()`, 0-100% like noise floor, persisted via `Nvm::savePeakThreshold()`, default **68%**) is the "this counts as a hit" level shared by `EqualizerShow`'s bass strobe, the Audio screen's per-bin china flash, and the console's `^`/`_`/`-` markers below.

**Hit decay rate screen (2b)** (`MagicCarpet::showHitDecayMeter()`): tunes `AudioBoard`'s shared hit-decay rate (0-1000ms, pot-adjusted, soft takeover, default 300ms — see "Audio processing" below). China off in both modes below.

- **Defaults to a simulated signal**, not live audio, so decay can be tuned without music. Waveform shape depends on phase length: 40ms phase = smooth continuous sine; 80ms phase = sine-shaped rise, holds at max, eases down the same way at the next transition. Both come from one fixed 40ms raised-cosine transition per phase (`AudioBoard::updateSimulatedBand()`) — at 40ms it consumes the whole phase (pure sine, no plateau); at 80ms only half (leaving a flat plateau). Phase length alternates 40ms/80ms every 10s.
- **Short press swaps to live audio** instead of simulated — a one-off override specific to this subsetting (short press again swaps back). Always starts in simulated mode on entry.
- **Simulated mode visual**: every light tracks the single simulated hit value in green, except the front strip, which shows one big VU meter across all 156 LEDs — white up to the current level, red for the portion above peak threshold, plus a red dot marking the threshold. A decay gap above the live signal also shows red.
- **Live mode visual**: front strip shows 3 separate VU meters (treble/mid/bass order), each driven by that band's hit value in blue/green/red — threshold dots always red. Megabars repeat a 3-position pattern — treble (blue), mid (green), bass (red) — around all 12, each showing that band's hit value.
- **Blue decay-position dot**: on every VU meter in both modes, positioned purely by the decay-rate *setting* (0ms one end, 1000ms the other) — unrelated to audio level, just a readout of where the setting sits.

**Audio foresight screen (2c)** (`MagicCarpet::showForesightAdjust()`): tunes the shared audio-foresight delay (0-700ms, pot-adjusted, soft takeover, default 0ms — see "Audio processing" below). Always live audio — no meaningful synthetic stand-in for real acoustic latency. Hit-prediction's *style* rides along on this same screen (there's no separate prediction-*distance* subsetting anymore — prediction always spans this whole live foresight range):

- **Rope**: solid dim blue across ~1016 LEDs, a "you're on this screen" indicator, not audio-reactive. Front strip also gets a 2-pixel white marker positioned by the foresight *setting* (0ms one end, 700ms other) — same idea as the decay screen's blue dot.
- **Megabars**: live audio-reactive, same repeating 3-position pattern as the decay screen's live mode — treble (blue)/mid (green)/bass (red) every third megabar, each showing that band's **hit** value (already delayed by the foresight being adjusted, applied upstream of every getter) — so a flash syncs to the beat once foresight is dialed in. Each megabar keeps a 15% brightness floor in its band color so fixture-to-frequency mapping stays visible between hits.
- **China stays off**, same as the decay screen.
- **Encoder**: idle otherwise on this subsetting, so it's repurposed to cycle hit-prediction *style* live — each detent advances `AudioBoard`'s `HitPredictionStyle` (**Off**, **Exponential rise**, **Machine gun**, **Drum intro** — see "Audio processing" below), wrapping. Pot-driven foresight ms and encoder-driven style commit together on double-press, revert together on leaving without committing (matching how PowerTest commits its hue/sat/brightness triple together).

**PowerTest screen** (`MagicCarpet::showPowerTest()`): an A/B test comparing straight HSV→RGB rendering of a live-editable color against its RGBW power-saving translation, to gauge savings and whether it still reads as "the same color." Megabars off. Driven entirely by the **encoder**, not the pot — rotation edits whichever of hue/saturation/brightness the current subsetting (3a/3b/3c) selects, steps of 4/detent; hue wraps, saturation/brightness clamp at 0/255. All 3 fields edit as one composite color: cycling 3a→3b→3c via short-press keeps the other two, double-press commits and persists all 3 at once (`Nvm::saveTestHue/Sat/Brightness`). Deliberately ignores the committed brightness ceiling — hue/sat/brightness here are already a full manual color spec, so the two renderings compare at face value.

The rig splits into a left/right half, a sideways U (rope's left side + left half of both front/rear edges) down to the carpet's center line:

- **Left half** — rope's left side, left half of both front/rear edges, china `[2..5]` (front-left/rear-left pairs) — renders straight: `CHSV(hue,sat,val)` → RGB, white channel off.
- **Right half** — rope's right side, right half of both front/rear edges, china `[0,1,6,7]` (front-right/rear-right pairs) — renders the **power-saving translation**: take `min(r,g,b)`, subtract from each of r/g/b (smallest channel bottoms at 0), add that amount to white instead. Same apparent hue/brightness, less from the color LEDs.

**Bugfix**: the rope's left/right split had the same front/rear-direction bug as `LighthouseShow`'s `ropeAngleDeg()` (below) — front/rear halves were swapped, most visible where the front edge meets the right-side strip's corner-wrap. Fixed the same way: `i < FRONT` + `i >= REAR` now form the left half in one simplified check, `[FRONT,REAR)` the right half.

## Lighthouse show

Two independent rotating "lighthouse" beams, each a pair of opposite-facing (180° apart) cones, tip at the car's center, expanding outward. Real angle math throughout — rope/megabar/china all evaluate the same falloff function at each fixture's own real physical angle (`CarpetGeometry`), not a per-fixture-type bespoke shape.

- **No additive mixing**: where the 2 beams' cones overlap, only the brighter beam shows at all — its own exact color at its own computed brightness. No hue blending. Megabars specifically break an *exact* brightness tie (real and non-negligible, since brightness is quantized to a uint8, not a razor-thin float coincidence) by alternating the winner by megabar index, rather than always favoring beam 1 — so a tie can never systematically starve one color's megabar coverage.
- **Cone width**: sized so at least 2 real megabars always fall within a beam's guaranteed-coverage core, at *any* beam angle — derived from `CarpetGeometry`'s real measured megabar ground-spot angles (not the nominal 30°-apart naming, which differs from the real throw-corrected positions). The exact worst-case threshold is 32.4855° (computed from the real angle table); `CONE_HALF_WIDTH_DEG` is set to 34° for a small float-precision margin.
- **Brightness within the guaranteed core is not flat**: it eases smoothly (via `smoothstep8()`) from full (255) at dead center down to a floor (`CORE_FLOOR_BRIGHTNESS`, 128) at the core's own outer edge, then keeps easing from that floor to black over a further fade zone beyond the guarantee — a smooth, "analog" brightness change as a beam sweeps past a megabar (nearer to center = brighter), while the floor keeps the worst-case-farthest guaranteed megabar always clearly, visibly lit.
- **Minimum beam separation**: the 2 beams' rotation velocities are independent random walks and can share sign (both drifting clockwise, or both counterclockwise), so nothing stops them drifting arbitrarily close together on their own. A per-frame enforcement keeps the 2 beams' guaranteed cores from ever touching (pushing the 2 angles apart symmetrically, 50/50, whenever they'd close past the minimum) — a soft "bounce," not a velocity restriction — so one beam's guaranteed megabar coverage can never be contested/stolen by the other beam's winner-take-all comparison.
- **Rotation**: each beam's angular velocity is its own independent random walk (`LighthouseShow::RandomWalk`) — ramps toward a fresh random target (±10°/s step) roughly every second, drifting through zero and reversing over time. Beam 1 starts +72°/s (5s/rotation); beam 2 starts -180°/s (2s/rotation). The walk wanders within a fixed ±360°/s ceiling; the **shared global energy setting** attenuates the raw output afterward instead of narrowing that range (0% = stopped, 100% = unchanged).
- **Color**: beam 1's hue always increases (never reverses) at a random-walking rate, 0 (frozen) to 1 full cycle/20s. Saturation random-walks 70-100%, starting at 80%, shared exactly by both beams (not independent per-beam). Beam 2 uses beam 1's saturation exactly plus the complementary hue (+128) — independent rotation only, no independent color.
- **Rope**: same falloff/winner-take-all as megabars, evaluated at each of ~1016 cached LED angular positions instead of 12 fixed ones.
- **Variations** (2, cycled by encoder rotation):
  - **Default**: rope+megabar+china all join both beams (the same beam-crossing effect everywhere); china additionally gets a bass-hit white triple-strobe (3 pulses, 30ms on/20ms gap) layered on top, on the White channel only (`LightSetters::setWhite()`) — never touching the RGB channel a china's beam-crossing color already occupies, so the white strobe and the RGB beam color genuinely superimpose instead of one overwriting the other.
  - **No Strobe**: identical rope/megabar/china beam-crossing effect as Default, just without the white strobe on china.
- **Performance**: iterates all 1016 rope LEDs every frame, avoiding `sinf`/`cosf`/`atan2f`/`floorf` in the hot path — `CarpetGeometry`'s neopixel table is precomputed once at boot, falloff math uses table-based `sin8()`-style integer arithmetic (`smoothstep8()`). `circularDelta()` (per-beam angular distance, thousands of times/frame — the hottest float call site in the codebase) avoids `fmodf()` (expensive on this FPU-less board) via one bounded branch instead.

## FlameSparkle show variations

The rope runs a classic Fire2012-style heat simulation (cool → disperse → spark → assign-color-from-palette). 4 variations, cycled by encoder rotation:

- **Variation 0/1** ("waterflames"/"flames"): the two original fixed 256-entry palettes (`DarkBlue→Blue→Aqua→White` / `DarkRed→Red→Orange→Yellow`).
- **Variation 2** ("shifting hues"): **two complementary colors**, both full brightness — one continuously-drifting hue (never reverses, rate random-walks, technique duplicated from `LighthouseShow`'s beam hue rate) and its complement (+128), sharing one random-walked saturation. Replaces an earlier single-hue/4-brightness-level design that read as dim/washed-out at the low end.
- **Variation 3** ("hue to white"): the same drifting hue, fading to fixed white at the hot end instead of a brighter version of itself.
- Variations 2/3 share one random-walked saturation (75-100%, starting at 90%, widened from a fixed 87-100% range that read too saturated). Variation 3's drifting-hue end used to be hardcoded to full saturation; now breathes the same as variation 2's.
- The **shared global energy setting** does two jobs across all 4 variations: controls sparkle/fire-sim cadence (`delay(potval)`, inverted so 100% energy = fastest), and in variations 2/3 also scales max hue-drift rate — 0% = 50% of base max (full cycle in 40s instead of 20s), 100% = 100%. Only the ceiling shifts, not the walk mechanism. (Also fixed a real bug: this ceiling used to be wired backwards, pot-up giving the *lower* 50% ceiling.)

**Floodlight sparkle**: the rope's random "spark" pulses (bright, brief, cooling) now match on floodlights — china treated like megabars. Each fixture gets its own independent "heat" with fresh random sparks at the rope's own cool/spark cadence (same pot-adjustable delay), cooling at **twice the rope's rate** (shorter flashes, by request — the rope's own rate read as too lingering). Fixture count sparkling per cycle is proportional: fraction of floodlights = fraction of rope LEDs sparkling (`sparkingRate/255`), rounded to nearest fixture, chosen at random. Rendered as a blend on top of the audio-tinted base color using heat as the blend fraction, so sparks rise/fade smoothly rather than snapping.

## Equalizer show variations

2 variations, cycled by encoder rotation. Both react to `AudioBoard`'s shared **hit** value (`getBandHitPercent(Band)`/`getBandHitNonzero(Band)`, see "Audio processing" below) instead of each rolling its own peak-hold logic — decay behavior is uniform and tunable from Audio config subsetting 2b.

- **Variation 0** (default): the original bouncing-chase pattern (unrelated to audio) on the rope, megabars tinted by bass (red, low band's hit) and treble (blue, high band's hit).
- **Variation 1**: a dual VU meter across the whole front/rear rather than one corner. Bass is based on the *entire rear edge* (always lit) and grows forward along both sides as it rises (red); treble is based on the entire front edge and grows rearward (blue) — same red=bass/blue=treble convention, both driven by hit. `RIGHT`/`LEFT` are the exact midpoints of each side run, so each meter's 100%-reach is 15% of the true-corner-to-center half-length past that midpoint — maxed bass/treble simultaneously cross near the car's middle, blending 50/50 where they overlap. Brightness scales with level, not just reach — a quiet meter is dim, not just short. Megabars/china off.
  - **Bugfix**: `FRONT_RIGHT`/`REAR_RIGHT`/`REAR_LEFT`/`FRONT_LEFT` are inset `SIZEOF_LARGE_NEO_CORNER` (33) LEDs in from each side's true physical corner — the per-side loop used to run only between those inset points, leaving the ~33-LED zone at each true corner permanently dark. Fixed by widening each side's loop to its full physical strand while keeping the reference points (and meter calibration) unchanged; past a reference point the same distance math correctly reads as "always lit, proportional to level." Deliberately does **not** reach into the front/rear channels' own logic (an earlier draft did, incorrectly — each side stays self-contained).
- **Pot** (either variation): live-adjusts the shared peak threshold (`PkThresh`, 0-100% full pot, prints `PkThresh:<percent>%` 1s after settling) — soft takeover. Deliberately *not* part of the global energy group (a threshold isn't "energy"); unlike that group it *is* tied to a persisted value — leaving the show reverts it to whatever's saved, so casual tweaking never silently overwrites the committed setting.

**Triple-strobe** (either variation): toggled by double press while Equalizer is active — nonvolatile. When enabled, a qualifying bass hit (`AudioBoard::getBandHitNonzero(BandBass)`) flashes all 8 china and the 8 corner-adjacent megabars (all except headlight/left/rear/right cardinals, index `0`/`3`/`6`/`9`) full white for 3 pulses, 30ms on/20ms gap.

Hit suppression prevents machine-gunning during a sustained loud passage: the triggering hit's level is remembered, and any further hit not exceeding it is suppressed within 3s of the last qualifying hit. A 3+s gap with no qualifying hits clears the memory, so the next hit always strobes fresh.

**Slow desaturation cycle** (both variations): every base color (`clr1`/`clr2` in variation 0, `bassClr`/`trebleClr` in variation 1) drifts saturation 100%→85%→100% over a smooth 30s sine (`currentSatFraction()`) — a subtle "breathing." `desaturate()` holds hue/value fixed, raising only the minimum channel — exact for these fully-saturated colors, a no-op on anything already less saturated.

## Audio processing

Tracking is **per-bin** first, per-band second: each of the MSGEQ7's 7 raw hardware bins (`AudioBin` enum, `FreqBin0`=63Hz .. `FreqBin6`=16kHz, chip datasheet order) independently tracks its own `raw`/`normal` (`levelClean`/`levelFilt` internally)/`hit`/`RMS` state. A band (`AudioBand` enum: `BandBass`/`BandMidbass`/`BandMid`/`BandTreble`/`BandFull`) is a fixed named group of bins — `BandBass`=bin0, `BandMidbass`=bin1, `BandMid`=bins 2-3, `BandTreble`=bins 4-6, `BandFull`=all 7 — and every `getBandXxx()` getter computes fresh each call as the **max** across its bins (not an average, so a loud narrow sub-band isn't diluted). `getBinXxx()` reads one bin directly, for shows needing finer granularity (e.g. pixel_war's treble/mid triggers in `BumpingAudioShow.h`). `BandFull` is every band getter's default with no argument.

- **Four tracked values per bin**, exposed only as percent (0-100) or nonzero/bool — nothing outside `AudioBoard` sees a raw 0-255 value:
  - **raw** (`getBandRawPercent`/`getBinRawPercent` + `Nonzero`) — straight ADC reading, no adjustment. Ungated even by the sound-reactivity kill switch, so diagnostics still show real signal with it off.
  - **normal** (`getBandNormalPercent`/`getBinNormalPercent` + `Nonzero`) — raw floored to 0 at/below noise floor, then optionally AGC-boosted (below). What every show actually reacts to.
  - **hit** (`getBandHitPercent`/`getBinHitPercent` + `Nonzero`) — peak-hold derived from normal: jumps to flat 100% the instant normal reaches the live effective peak threshold (see Auto-peak below), holds 75ms; if still ongoing, eases to a 90% sustain floor over the next 25ms and holds there (so a long tone isn't pinned at 100%). Decays from *whatever it was holding* — 100%, mid-ease, or the 90% floor — once normal drops back below threshold, at a fixed adjustable rate (**hit decay rate**, 0-1000ms full-to-zero, default 300ms, subsetting 2b, `Nvm::saveHitDecayMs()`), snapping to 0 once it rejoins normal. `Nonzero` means "currently above threshold" — goes false on decay even while hit itself still ebbs. Predictive lead-up (Hit prediction below) can push this ahead of an actual crossing, peaking at 100%.
    - Decay is computed from real elapsed time, so the rate setting is independent of `pollFrequencies()`'s poll rate.
  - **RMS** (`getBandRmsPercent`/`getBinRmsPercent` + `Nonzero`) — root-mean-square of that bin's `normal`-stage history over a trailing 4s window, computed every poll (real `sqrtf()` cost on this FPU-less board, accepted by request). Represents overall energy rather than instantaneous peak, so broadband noise reads louder than a narrow tone at the same level — indistinguishable via max-of-bins alone.
- **`NewBandHit(band)`/`NewBinHit(bin)`**: one-shot edge trigger (band defaults to `BandFull`), true once per fresh crossing — consumption is scoped to that one specific instant in time (deduped by timestamp, not by "whoever asked first"), so it can never double-fire even if called more than once, or by more than one caller, while the getters are still pointed at that same instant. The band version consumes every covered bin regardless of which fired, so a pending flag can't leak through a different overlapping band.
- **AGC (auto-gain) modes** (`AGCMode`: `AGCoff`/`AGCband`/`AGCfull`, persisted, default **AGCfull**, subsetting 2d, encoder-cycled): decides how `normal` is boosted from floored `raw`. The boost itself (per bin, and the shared `AGCfull` equivalent) is a genuinely **stateful envelope**, not recomputed fresh from a snapshot each poll — classic fast-attack/slow-release compressor shape:
  - **Instant attack**: the moment a louder sample appears **anywhere in the whole 4s buffer** (see the unified history below) — including foresight-buffered *future* rows, not just what's currently displayed — boost snaps down immediately to keep that peak at 100%. This is what makes AGC genuinely **proactive** whenever foresight is nonzero: a loud moment already sitting in the buffer pulls boost down before it becomes the displayed instant, instead of reacting exactly on it.
  - **Continuous release**: otherwise, boost rises at a fixed rate calibrated so crossing its *entire* range (1.0x, no boost, up to `100/noiseFloorPercent_`, its natural ceiling — reachable because the smallest possible nonzero `clean` value is always just above the noise floor, so no separate clamp is needed) takes exactly one buffer-length (4s) — a real fixed-rate ramp, not a discrete "wait for the old peak to age out of a window" step.
  - **Off** — no gain, `normal == clean`.
  - **Band** — each bin runs its own independent envelope.
  - **Full** — every bin shares the SAME envelope, derived from the max across all 7 bins.
  - `getBandNormalPercentPreview(band, mode)` recomputes `normal` under an explicitly-passed mode instead of the committed one — lets the AGC-mode config screen preview a live, not-yet-committed mode choice without needing to mutate (and later revert) the real committed mode; cheap, since both envelopes are already tracked every poll regardless of which mode is actually active.
- **Auto-peak** (`AutoPeakMode`: `AutoPeakOff`/`AutoPeakFull`/`AutoPeakBin`, persisted, default **AutoPeakBin**, subsetting 2f, encoder-cycled, clamped like AGC mode): per-bin threshold scaling, replacing the old debounce-based design entirely.
  - **Inclusion is automatic, not show-configured**: a bin becomes "included" the instant ANY hit-family getter is called for it (`getBinHitPercent`/`getBandHitPercent`/their `Nonzero` forms/`NewBinHit`/`NewBandHit`) — a show's own normal sound-reactive code naturally registers exactly the bins/bands it actually uses, with zero explicit setup. `AudioBoard::clearAutoPeakInclusion()` resets to fully-excluded, called automatically on boot and on every show/variation change (a show may also call it explicitly).
  - **Off** — every bin uses the plain global threshold, nothing excluded.
  - **Excluded bins** (both other modes): `hit` is hard-pinned to 0, always — not a decay, an immediate zero, regardless of level.
  - **Full** — every INCLUDED bin shares ONE threshold: `globalThresh * (groupRms/100)`, where `groupRms` is the max RMS among currently-included bins — the whole included set treated like one shared band (mirrors how `AGCfull` derives one shared gain rather than differentiating per bin).
  - **Bin** — each included bin gets its OWN threshold: `globalThresh * (thisBinRms/maxIncludedRms)` (mirrors `AGCband`'s per-bin differentiation). A bin whose own RMS is below `AUTO_PEAK_MIN_INCLUSION_LEVEL` (compile-time, default 50%) of the group's max also reads as excluded for that instant — too quiet *right now* relative to the rest of the included group.
- **Simulated waveform** (`updateSimulatedBand()`/`resetSimulatedBand()`/`pollSimulated()`): synthetic signal through the same raw/normal/hit pipeline, for subsetting 2b's test screen.
- **Unified audio history, and the audioReadTime/lightsShowTime split** (`AudioBoard.h`'s big restructure — the actual mechanism behind foresight, prediction, and AGC's proactivity above): every poll writes one row into a single ring buffer (`audioData_[levelType][bin][slot]` + a shared `audioTimestamps_[slot]`), sized to hold a flat `AGC_WINDOW_MS` (4s) of history. Foresight eats into this same fixed budget rather than extending it — depth doesn't need to grow with the foresight setting, so `audioReadTime` always wraps cleanly with zero special-casing, and AGC's own scan covers the *entire* buffer every poll rather than needing a trailing-window carve-out (see above). Two pointers into that same buffer:
  - **`audioReadTime`** (internal only, no getter reaches it) — the write head, advances by 1 every poll, always the just-captured instant.
  - **`lightsShowTime`** — recomputed every poll as the row nearest to `audioReadTime`'s timestamp minus the live foresight setting **minus the current EMA loop time** (`LoopTime.h`, see "Loop timing" below) — the latter corrects for a separate, real sync-error contributor: the gap between "this poll computed `hit` for `lightsShowTime`" (top of `loop()`) and "the LEDs for that same iteration actually finished transmitting" (`FastLED.show()`, near the bottom) is essentially one loop iteration's own duration, present even at 0ms foresight. **Every external getter (`raw`/`normal`/`hit`/`RMS`, all of them) reads only from this pointer.**
- **Audio foresight** (0-700ms, default 0, subsetting 2c): the DSP pipeline "sees" a waveform before it reaches the speakers — this is how far `lightsShowTime` trails `audioReadTime`, i.e. how far ahead of the audible output the freshest capture currently sits. Default 0ms uses the freshest sample, keeping the DSP's free predictive lead; dialing up trades lead for tighter sync with true acoustic output.
- **Hit prediction style** (adjustable via the encoder on the Foresight subsetting 2c — no separate prediction-*distance* dial anymore, prediction always spans the whole live foresight range): the rows between `lightsShowTime` and `audioReadTime` are real future-relative samples not yet "displayed" — `computePredictedRamp_()` scans that span's `normal`-stage history (not `hit` — those rows haven't had AGC/hit processing applied yet, that only happens once `lightsShowTime` itself reaches them) for the earliest sample crossing the plain threshold, blending a rising lead-up into `hit` (whichever of normal tracking or predicted lead-up is higher). Three styles (`HitPredictionStyle`), default **Exponential rise**:
  - **Off** (`PredictDisabled`) — no lookahead, always returns 0.
  - **Exponential rise** (`PredictExponential`) — cheap-integer stand-in for an exponential curve asymptotic at the predicted hit: `ramp = 100*(foresightMs-timeUntilHit)/timeUntilHit`, clamped to 100.
  - **Machine gun** (`PredictPulseTrain`) — a drumroll of 30ms pulses (`shapePulse()` trapezoid) peaking at the live threshold level, fixed 300ms lead-in; spacing narrows 90ms→30ms into the hit.
  - **Drum intro** (`PredictTwoPulse`) — two 35ms pulses, 45ms apart, anchored at `min(foresightMs, 400ms)` before the hit.
  - **`shapePulse()`**: shared "rounded square wave" — linear ramp up, flat peak, linear ramp down, each a third of pulse width.
- **Silence detection**: fully derived from the same unified history now — `normal`-stage (`levelClean`) history across every bin is scanned over the same trailing `AGC_WINDOW_MS` window AGC/RMS already use; silent iff nothing anywhere in that window (any bin) has been above 0 (i.e. above the live noise floor). Only starts reporting silence once a full window of real history has actually accumulated (avoids a false "silent" reading right after boot). Internal state only — read via `AudioBoard::isSilent()`, not a public field.
- **Sound reactivity** (persisted, default **on**, subsetting 2e): global kill switch every `normal`/`hit` getter checks centrally. `raw`/`RMS` stay ungated.
- **Per-bin noise floor**, from measured self-noise: scoped with the phone muted, each bin's own baseline self-noise voltage differs consistently — bins 0-4 ~0.17V, bin 5 ~0.280V, bin 6 (highest freq) ~0.56V (`AudioBoard::BIN_NOISE_VOLTAGE[]`, hand-editable if re-measured). The global **noise floor** setting (0-100%, default **19.5%** — persisted as a whole-percent `uint8_t`, so the actual flashed default rounds to 20%) is the floor for whichever bin has the HIGHEST measured voltage (bin 6); every other bin's own floor scales down proportionally to its own voltage ratio against that max (`recomputeBinNoiseFloors_()`, re-run whenever the global setting changes). `clean` is an ALL-bins-or-nothing gate, not per-bin: only if EVERY bin's `raw` sits at/below its own per-bin floor does every bin's `clean` zero out this poll; the instant ANY bin is above its own floor, every bin's `clean` passes through as raw, unfiltered — the per-bin floors exist to correctly calibrate "is THIS bin real signal or just its own self-noise," and the all-below test is the right way to combine those per-bin tests into one "is anything real happening at all" global gate.
- **AGCfull is the default** at first boot; it, auto-peak (`AutoPeakBin`), noise floor, peak threshold (default **68%**), sound reactivity, hit decay rate, and audio foresight all persist (`Nvm.h`).
- **Peak threshold** (default **68%**) is the shared "this counts as a hit" level — see Configuration mode above. Auto-peak can raise the *effective* threshold above this per-bin, per-instant; never lowers it. Also adjustable live in normal show mode via **PressLeft/PressRight** — see "Encoder and button UI" above.
- **Loop timing** (`LoopTime.h`, new class, `LoopTime::update(nowMs)` called once at the very top of every `loop()` iteration, before `AudioBoard`'s own poll): tracks most-recent loop time, an EMA (750ms time constant — settles to a new steady-state within about a second of a real change like a show switch, without jittering frame-to-frame), the max-ever loop time (only recorded starting 2s after `setup()` returns, to exclude boot-time irregularities), and a trailing-2s windowed peak. The EMA feeds `lightsShowTime`'s own target computation directly, see above; the rest are general diagnostics, not currently surfaced anywhere else.
- **Outside this file, every level/hit is percent** (0-100), never raw 0-255 — `NightriderShow`/`FlameShow` convert back to their own internal 0-255 scale right at the call site, localized rather than threaded through their internals.

## Vehicle speed link and SpeedStripesShow

The Due's primary I2C bus (`Wire` — pins 20 (SDA)/21 (SCL) on the communication port header, confirmed against `framework-arduino-sam`'s `variant.h`; `Wire1`'s separate `SDA1`/`SCL1` pins are *not* used) runs as an I2C **slave** at address `0x69`, receiving vehicle telemetry from CANTroller2 (the sibling project, I2C master on its own `Wire` bus). Protocol (CANTroller2's `src/i2cbus.h`, `LightingBox`): no checksum/length byte, I2C START/STOP delimits each packet, top nibble of byte 1 is the command — `0x0`=status flags (ignored here), `0x1`=runmode (see "Low power mode" below), `0x2`=speed update (2 bytes: low nibble of byte 1 + byte 2 form a 12-bit hundredths-of-mph value).

CANTroller2 only ever transmits 12 bits of speed (top nibble of its 16-bit internal value discarded on send), so max representable speed is `0x0FFF` = 40.95 mph — a sending-side limitation, not fixable here.

`SpeedLink::getSpeedMph()` returns the last received value; `SpeedLink::isFresh()` reports whether a packet arrived recently (within 2s default), so a speed-reactive show can fall back to "stopped" instead of freezing on a stale value.

`SpeedLink` also tracks runmode packets (`0x1R`, sent only on change) via `getRunmode()`/`isLowPower()`. Runmode `1` is `LowPower` (CANTroller2's `globals.h`: `Basic=0, LowPower=1, Standby=2, Stall=3, Hold=4, Fly=5, Cruise=6, Cal=7`).

### Low power mode

Every show derives behavior from the live pot reading (`carpet->pot->readPercent()`), so rather than teaching each show about low power, the simulation lives one layer down in `LedControl::Potentiometer` (`LedController.h`) — every show gets it free, no show-file changes:

- **Entering low power** (`SpeedLink::isLowPower()` goes true): `Potentiometer::readPercent()` returns a synthetic value fading from the pot's reading *at the moment engaged* down to 0 over 20s, then holds — as if someone slowly turned the pot down and left it.
- **Manual override**: if the real pot moves ~2%+ from that captured start (during fade or zero-hold), the simulation cancels for this episode and `readPercent()` returns the true live value — a real turn always wins. Edge-triggered: the cancellation sticks despite continued `LowPower` reports; only a fresh rising edge (low power ends then re-engages) re-arms the fade.
- **Exiting low power**: any non-`LowPower` runmode cancels the simulation immediately, same as manual override, snapping to live tracking (no fade back up).

Only runs while `ModeShow` is active — config screens read the pot live via `Potentiometer::readLivePercent()`, so low-power state never affects brightness/audio/color adjustment.

**The raw ADC reading never leaves `Potentiometer`** — `readPercent()`/`readLivePercent()` are the only public accessors, both already 0-100%; `MAX_VOLTAGE` and `analogRead()` stay internal. Every consumer (soft-takeover checks across `CarpetLightLogic.cpp`/`LightShow.h`/`BumpingAudioShow.h`/`NightriderShow.h`) works in percent, never raw counts.

**`SpeedStripesShow`**: the two long side rope strips are the only fixtures running front-to-rear along the car's 16ft length (front/rear edges don't, stay off). Each side strip (352 LEDs) divides into 4 bands of 88 LEDs (4ft each) alternating lit/dark with a smoothed transition. The pattern scrolls front-to-rear at a rate proportional to live speed (`SpeedLink::mphToFtPerSec()`, real feet/sec — not a hand-tuned LED-count rate), giving a sense of motion.

**`VarZebra`** (2nd variation): deterministic rainbow-hue stripes with an optional periodic black-stripe overlay, pot-driven. The pot sets the black stripe's own width, 0ft (pot=0%, no stripes) to 40ft (pot=100%); the colored gap between consecutive black stripes is always exactly 2x that width, so the occurrence period (one black stripe's start to the next) is always exactly 3x the black width — e.g. a 2ft-wide stripe recurs every 6ft. Wider stripes therefore also come proportionally less often, rather than width and spacing scaling independently.

**Color**: each stripe (one lit band) gets its own 2-color gradient, leading to trailing edge, rather than one shared hue. Which pair a stripe shows is deterministic from its period-cycle (`floor(pos/period)` through a Knuth multiplicative hash), so a physical stripe keeps a stable identity while scrolling without per-instance state. The same slow desaturation cycle as `EqualizerShow` (100%→85%→100% over 30s) breathes on top.

**Performance note**: the SAM3X8E's Cortex-M3 has **no hardware FPU**, so `sinf()`/`tanhf()`/`atan2f()`/`cosf()` are expensive — and this show's `update()` runs unconditionally every loop, even behind a config screen. An earlier version called `atan2f()` per rope LED per frame (1016×, for a since-removed spin mode) and recomputed desaturation `cosf()` per LED — felt as general UI choppiness. The current version avoids float trig in the hot path entirely: `currentSatFraction()` computed once per `update()` and passed down, color hash is integer multiply/xor, brightness envelope uses table-based `sin8()` plus an integer contrast boost.

Every other fixture with a meaningful along-length position samples the same scrolling pattern at its own physical position, so a stripe's color matches everywhere it's visible (china, megabars, rope). The sampling function is periodic and defined beyond the car's own corners, so forward/rearward-pointed fixtures preview an approaching stripe or show it receding with no special-casing:

| Fixture(s) | Position sampled |
| --- | --- |
| Megabars `[11,0,1]` (front 3, incl. headlight) | 4ft ahead of the front corner — a preview, well before the stripe arrives |
| China `[1,2]` (aimed along the front edge) | 1ft ahead of the front corner — picks the same stripe up just as it's about to cross onto the carpet |
| Megabars `[5,6,7]` (rear 3) | 4ft behind the rear corner — mirrors the front, for the stripe that just left |
| China `[5,6]` (aimed along the rear edge) | 1ft behind the rear corner — mirrors the front |
| China `[0]`/`[3]` (front-right/front-left, aimed along a side edge) | 1/3 of the way rear from the front |
| China `[7]`/`[4]` (rear-right/rear-left, aimed along a side edge) | 2/3 of the way rear from the front |
| Megabars `[2,10]` / `[3,9]` / `[4,8]` (remaining 3 per side) | Same 1/3, 1/2, 2/3 partition by angle — `[2]`/`[10]` line up with the 1/3 china, `[4]`/`[8]` with the 2/3 china |

All of the above (16ft/12ft carpet dimensions, 4ft/1ft preview distances, fade steepness) are hardcoded per user-measured values in `SpeedStripesShow.h` — adjust there if dimensions change.

**Stopped mode**: at speed 0, or when `SpeedLink` hasn't heard from the vehicle in 4s (stricter than `isFresh()`'s 2s default), the pattern **freezes** — `scrollOffset_` stops advancing, so every fixture (all deriving position from it) holds exactly where it was. After a further **10s** stopped, each stripe's hue pair starts slowly meandering — one shared drifting offset (`meanderHue_`, `RandomWalk`-based, same technique as `LighthouseShow`/`FlameShow`) adds on top of each stripe's hash-derived base hue, so they evolve together while staying distinct. A fresh stop always restarts this from scratch.

## Timing convention

Any "how much time has passed since X" check should use `Utilities.h`'s `Timer` class (`.elapsed()`, `.expired()`, `.expireset()`) rather than hand-rolled `millis() - savedTimestamp` — clearer at the call site, and keeps rollover-safe subtraction in one place. Used throughout: `Potentiometer`'s low-power fade, `MagicCarpet::delayPolling()`, `ArmDmx.h`'s inter-send spacing, `AudioBoard`'s poll-interval and silence tracking, `EqualizerShow`'s strobe timing, and `RandomWalk` (duplicated in `FlameShow`/`LighthouseShow`).

Two spots deliberately still use raw `millis()` math, since `Timer` only models "elapsed since I was last reset to now":

- **`PushButtonEdge`'s edge queue / `PushButton::processEdge()`** (`LedController.h`) — edges are timestamped in an interrupt the instant they happen, then replayed later using that recorded (possibly stale) timestamp, not "now." A `Timer` can't be reset to an arbitrary past moment.
- **`AudioBoard`'s rolling-peak ring buffer** (`peakTimestamps_[]`) — independently-timestamped samples compared against the *current* time to find which are still inside the trailing 4s window, not a single running duration.

## Future direction

Light shows should eventually be defined in terms of human-meaningful physical zones/effects rather than raw array indices:

- **A "side" preset**: a run of megabars + china pair(s) washing that edge (e.g. "left side floods" = megabars `[3,4,5]` (addr 10,13,16) + china `[3,4]` (addr 55,61)).
- **A "sweep"**: driven by front-to-rear distance along the perimeter, applied symmetrically to both sides.
- **A "lighthouse" effect**: one rotating angle picking both the nearest megabar and a same-angle window of perimeter LEDs.
- **A rope-index/strand abstraction layer**: which physical strand (front/right/rear/left) a `ropeLeds[]` index belongs to, and its position along that strand, currently has to be re-derived by hand at every call site from raw `FRONT`/`RIGHT`/`REAR`/`LEFT`/`SIZEOF_*` constants (in firmware and visualizer independently) — every site is a fresh chance to get a strand boundary wrong or reach across it into a neighbor's logic (happened at least once — the Equalizer VU-meter bugfix above). Real functions (`stripFor(i)`, `posInStrip(i)`, plus megabar X/Y helpers, which have no position model in firmware today, only the visualizer) would make this impossible to get wrong by construction. Flagged, not yet built.

None of that abstraction exists in code yet — this doc records the raw hardware facts it would need to be built on.
