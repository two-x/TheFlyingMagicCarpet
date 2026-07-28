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

### Perimeter flash feedback

The rope perimeter flashes white as UI feedback (`MagicCarpet::flashRope()` — 55ms white padded by 15ms black on each side; 85ms gap between multiple flashes in one call; blocking, but polls the button throughout via `delayPolling()` rather than going fully blind — a flash sequence can run up to ~255ms, long enough to otherwise drop part of a rapid follow-up double-press):

- **1 flash**, live, the instant a hold crosses the medium-press threshold (300ms)
- **2 flashes**, live, the instant a hold crosses the long-press threshold (1500ms)
- **1 flash** when a double press commits a config setting/subsetting (not in normal mode, where double press is a no-op)
- **2 flashes** when a medium press advances to the next config setting (not in normal mode — a normal-mode medium press currently does nothing)

### Normal mode

- **Short press**: cycles to the next light show (Nightrider → Flame → Equalizer → SpeedStripes → ...). Persisted to flash immediately.
- **Extra-long press**: toggles the whole rig on/off. Always boots **on** — this one is intentionally *not* persisted.
- **Long press**: enters configuration mode (see below).
- **Encoder rotation**: per-show, selects that show's own variation (not global) — e.g. `NightriderShow` has 2 variations (pot picks a live hue pair, vs. hue auto-cycles at a pot-controlled rate), `FlameShow` has 2 (flames vs. waterflames palette, toggled per detent). Each show's current variation is persisted per-show, so switching shows and back recalls where you left it.
- **Pot**: meaning depends on the active show's current variation (e.g. live hue, animation speed, palette rate) — see each show's own code.

### Configuration mode

A 4-screen cycle, and each screen can have multiple **subsettings** (only Audio has more than one right now — 2: noise floor, auto-gain enable). Button roles inside config mode:

- **Medium press**: advances to the next screen (Global → Headlight → China → Audio → Global), no save. (This *replaces* medium press's old role of cancelling — a normal-mode medium press, and a config-mode medium press, no longer cancel anything.)
- **Double press**: commits whichever subsetting is currently showing, saves it to flash, prints it to the console, flashes once, and exits to normal mode.
- **Short press**: cycles to the next subsetting *within* the current screen (a no-op on the 3 screens that only have one).
- **Long press**: the *only* thing that cancels now — exits without saving, from any screen/subsetting. (Long press still also does double duty from normal mode: it's what enters config mode in the first place, always landing on screen 1.)

The active light show keeps running "invisibly" underneath the whole time config mode is open, so nothing jumps when you cancel.

**Soft takeover**: entering a screen *or* a subsetting doesn't snap the preview to wherever the pot/encoder physically happens to be. It holds at the already-committed value until the pot moves at least 2% of its full range, or the encoder is twisted at all, since entry — only then does it hand over to live tracking (`livePercentFor()` / `resetTakeoverState()` in `CarpetLightLogic.cpp`). This avoids a value jumping the instant you enter a screen or cycle a subsetting.

| # | Screen | Subsetting | Control → | Live preview |
| --- | --- | --- | --- | --- |
| 1 | Global brightness | (none) | Pot, 0-100% (top half only) | Rope + all megabars + china: solid red at the live value |
| 2 | Headlight brightness | (none) | Pot, 50-100% (full pot) | Rope + china off; other megabars solid red at the *committed* global brightness (reference); headlight solid red at global × live headlight |
| 3 | China brightness | (none) | Pot, 0-100% (full pot) | Rope off; megabars solid red at their normal effective brightness (committed global + headlight); china solid red at global × live china |
| 4a | Audio: noise floor | 0 | Pot, 0-100% (full pot) | See below |
| 4b | Audio: auto-gain enable | 1 | Encoder, right = on, left = off | See below |

**Audio screen visual** (`MagicCarpet::showAudioMeter()`, same for both subsettings): rope is otherwise entirely off, china is blanked; the front strip, both sides, and all megabars light up.

- **Front strip** (156 LEDs) splits into 3 equal 52-LED segments in array order: treble, mid, bass (`AudioBoard::getHigh()`/`getMid()`/`getLow()`, 0-255 each, silence-gated but never auto-gained — see "Audio processing" below). Each segment shows a dim (50%) reference gradient across its length — green → yellow/orange (center) → red — with a bright white fill from the segment's start up to the current level's position; a segment goes fully solid white at max level. (Fixed a real pre-existing bug in `AudioBoard::getHigh()` along the way — it was returning `bin_mid` instead of `bin_high`, a copy-paste error that also affected `NightriderShow` and `EqualizerShow`.)
- **Each side strip** (352 LEDs, back corner to front corner) shows a 10-LED window, always exactly 10 LEDs lit, colored by its own position along the side (red at back, green at front) rather than a fixed color. On subsetting 4a it tracks the live noise-floor percent (back corner = 0%, front corner = 100%). On subsetting 4b it just snaps to one end as an on/off indicator: back corner (0%) = auto-gain off, front corner (100%) = on. Left and right mirror each other.
- **All 12 megabars** glow blue, brightness proportional to `AudioBoard::getFullSpectrum()` (silence-gated, auto-gained if enabled) — a live readout of "how loud is it right now," independent of which subsetting is active. While adjusting subsetting 4b specifically, this reflects the *live* (not-yet-committed) auto-gain toggle state rather than the last-committed one, via `AudioBoard::getFullSpectrum(bool)`'s override parameter — this sidesteps needing to revert anything if you cancel out of that subsetting without committing.

Noise floor's live value *is* temporarily applied to `AudioBoard` while you're adjusting it (so the VU meter/megabar glow actually go quiet as live feedback while you raise it past the ambient level) — and explicitly reverted back to the committed value if you leave subsetting 4a any way other than committing (cycling away, advancing, or cancelling). Auto-gain doesn't need this treatment at all, per the override-parameter approach above.

## Audio processing (`AudioBoard.h`)

- **Full-spectrum level**: `getFullSpectrum()` is the max of all 7 raw hardware bins (not just the 3 curated into `bin_low`/`bin_mid`/`bin_high` for the VU meter), scaled to 0-255. This is the "how loud is it right now" signal auto-gain and the megabar glow are built on.
- **Rolling peak / auto-gain**: a sliding 4-second-window max (`rollingPeak_`, via a timestamped ring buffer, recomputed every ~30ms poll) sets a slow-moving gain (`255/rollingPeak_`) so quiet passages still reach full scale once enabled. Applying that gain to a sample louder than anything in the last 4s would push it past 255 — clamping to 255 achieves exactly "reduce the scaling factor so this instant peaks at max," just computed as a clamp rather than solving for a per-sample adjusted gain.
- **Rolling average / silence detection**: a separate EMA average (also ~4s time constant, computed with a `dt`-aware alpha so it stays consistent even though the actual poll interval isn't perfectly fixed) tracks a running level. If that average stays below the noise floor for a full 4 seconds continuously, audio is considered "off," and every level getter (`getLow`/`getMid`/`getHigh`/`getFullSpectrum`) returns 0 until the average rises back above the floor (which un-silences immediately, no delay on the way back up). The noise floor is compared against the **raw**, un-gained average specifically, so auto-gain can never mask silence by amplifying noise above the threshold.
- **Auto-gain is off by default** at first boot; both it and the noise floor persist to flash once configured (`Nvm.h`).
- Auto-gain deliberately only affects `getFullSpectrum()`, not the 3 curated VU-meter bins (`getLow`/`getMid`/`getHigh`) — those stay raw (but still silence-gated), so this doesn't retroactively change how `NightriderShow`/`FlameShow`/`EqualizerShow` already react to audio.

## Vehicle speed link (`SpeedLink.h`) and `SpeedStripesShow`

The Due's second I2C bus (`Wire1` — physically the `SDA1`/`SCL1` pins nearest the USB connectors, distinct from the primary `Wire` bus) runs as an I2C **slave** at address `0x69`, receiving vehicle telemetry from CANTroller2 (the sibling project, acting as I2C master over its own separate `Wire` bus). Protocol (see CANTroller2's `src/i2cbus.h`, `LightingBox`): no checksum/length byte, I2C START/STOP delimits each packet, top nibble of the first byte is a command code — `0x0`=status flags (1 byte, ignored here), `0x1`=runmode (1 byte, ignored here), `0x2`=speed update (2 bytes: low nibble of byte 1 + all of byte 2 form a 12-bit hundredths-of-mph value). Only the speed packet has any effect currently.

Note CANTroller2 only ever transmits 12 bits of speed (the top nibble of its 16-bit internal value is discarded on the sending side), so the max representable speed is `0x0FFF` = 40.95 mph — a limitation of the sending side, not something fixable from here.

`SpeedLink::getSpeedMph()` returns the last received value; `SpeedLink::isFresh()` reports whether a packet has arrived recently (within 2s by default), so a speed-reactive show can fall back to "stopped" if the link drops instead of freezing on a stale value.

**`SpeedStripesShow`**: only the two long side rope strips physically run front-to-back along the car's length (the front/back edges don't — see "Perimeter rope lights" above), so this show is confined to those two strips; megabars, china, and the front/back rope edges are blanked, same approach as the audio meter above. Each side strip (352 LEDs) is divided into 4 alternating lit/dark bands of 88 LEDs each — a quarter of the car's length wide. The band pattern continuously scrolls from front toward back at a rate proportional to the live speed reading (`LEDS_PER_MPH_PER_SEC` in the show, a to-taste constant), giving a sense of forward motion; at a standstill, or if the telemetry link is stale, the bands sit still. The pot picks the stripe color, same convention as `NightriderShow`'s variation 0.

## Future direction

Light shows should eventually be defined in terms of human-meaningful physical zones/effects rather than raw array indices — e.g.:

- **A "side" preset**: a run of megabars + whichever china pair(s) wash that same edge (e.g. "left side floods" = megabars `[3]`,`[4]`,`[5]` (addr 10,13,16) + china `[3]`,`[4]` (addr 55,61)).
- **A "sweep"**: driven by front-to-back distance along the perimeter, applied symmetrically to both sides at once.
- **A "lighthouse" effect**: driven by one rotating angle that picks both the nearest megabar and a same-angle window of perimeter LEDs at once.

None of that abstraction exists in code yet — this doc records the raw hardware facts it would need to be built on.
