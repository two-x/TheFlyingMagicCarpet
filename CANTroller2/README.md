# Flying Carpet Controller v3.1

The vehicle control box firmware ("CANTroller") for the Mule art car — an ESP32-S3 board reading pedal/steering/brake sensors and driving throttle/brake/steering actuators, plus an onboard TFT touchscreen UI. Talks over I2C to a sibling board, the **LightingBox** (= the `CarpetLightCode` sibling directory in this same repo), sending it runmode/speed/status updates.

**See also:** `CarpetLightCode/README.md` documents the LightingBox side of the I2C link this file describes from the control-box side. For Claude Code standing prompts that keep this file and commit messages in sync automatically, see [`claude_dev_prompts.md`](claude_dev_prompts.md).

## Contents

- [Setup](#setup)
  - [CPU board](#cpu-board)
  - [Install environment](#install-environment)
  - [Cloning the repo](#cloning-the-repo)
  - [Dev environment nice-to-haves](#dev-environment-nice-to-haves)
  - [Uploading](#uploading)
  - [Wifi](#wifi)
  - [JTAG debugging](#jtag-debugging)
- [Hardware inventory](#hardware-inventory)
- [Program flow](#program-flow)
- [Run mode state machine](#run-mode-state-machine)
- [Sensors](#sensors)
- [Motors](#motors)
- [Inputs: buttons, encoder, touchscreen](#inputs-buttons-encoder-touchscreen)
- [Display and UI](#display-and-ui)
- [I2C bus and the LightingBox link](#i2c-bus-and-the-lightingbox-link)
- [NeoPixel strip](#neopixel-strip)
- [Temperature monitoring](#temperature-monitoring)
- [Diagnostics and safety](#diagnostics-and-safety)
- [Persisted settings](#persisted-settings)
- [Known limitations and future work](#known-limitations-and-future-work)
- [Links](#links)

## Setup

### CPU board

Board is ESP32-S3-DevKitC-1-N8R8. Comes in v1 and v1.1, differing only in which pin the onboard NeoPixel is wired to (48 on v1, 38 on v1.1) — code assumes v1 (pin 48), since those are more available. [Buy here](https://www.amazon.com/gp/product/B0BSCXHB5S). For a breadboard dev setup, wire pins per the comments in `src/globals.h` (external capacitors/resistors are documented there too). Two micro-USB ports, "UART" and "USB" — either uploads code, but serial console output only comes out "UART". If upload fails on one port, try the other (upload runs at 1MBps, so a bad cable can also cause failures).

### Install environment

1. [Install VSCode](https://code.visualstudio.com/).
2. Extensions: PlatformIO IDE, C/C++, Clang-format, ESP-IDF, GitLens (optional: Vim, Rainbow CSV).
3. Clone the repo (below).
4. Create a PlatformIO project targeting board "ESP32-S3-DevkitC-..." pointed at the `CANTroller2` folder.

### Cloning the repo

```sh
ssh-keygen -t rsa -b 4096 -C <your-git-email>
# copy ~/.ssh/id_rsa.pub, add as an SSH key on GitHub (repo owner's account for push access)
mkdir -p ~/Documents/TheFlyingMagicCarpet
cd ~/Documents/TheFlyingMagicCarpet
git clone https://github.com/two-x/TheFlyingMagicCarpet .
# control system files: ~/Documents/TheFlyingMagicCarpet/CANTroller2
```

### Dev environment nice-to-haves

(iTerm + Mac + VSCode) [Oh My Zsh](https://ohmyz.sh/#install) for a more readable prompt plus plugin compatibility ([macOS plugin shortcuts](https://github.com/ohmyzsh/ohmyzsh/tree/master/plugins/macos#commands)); [zsh-autosuggestions](https://github.com/zsh-users/zsh-autosuggestions) (gray inline suggestions, right-arrow to accept) — needs installing plus `plugins=(git zsh-autosuggestions macos)` in `~/.zshrc`.

Alternate/manual PlatformIO setup, if the VSCode extension route doesn't work: `brew install platformio`; board JSON from [platform-espressif32](https://github.com/platformio/platform-espressif32/blob/master/boards/esp32-s3-devkitc-1.json) into `~/.platformio/boards/`; `clang-format` via `brew install clang-format`, then point the VSCode Clang-format extension's "Executable" setting at its path and "Assume Filename" at `.clang-format`.

### Uploading

Source is `CANTroller2/src`; runtime filesystem content is `CANTroller2/data`. Before first programming a board, erase flash once (alien head → Platform/Erase Flash) to partition correctly — only needed again if the partition map changes. Alien head → "Platform/Upload Filesystem Image" writes `data/` (compiled to `.pio/build/littlefs.bin`) over USB; alien head → "Upload" writes `src/` (compiled to `.pio/build/firmware.bin`). **OTA**: once a board already runs OTA-capable code, power it, join wifi `artcarpet`, browse to `192.168.1.69/update`, pick the OTA mode per file, then upload the two files above one at a time.

### Wifi

If the web server is enabled, the board acts as its own access point: SSID `artcarpet`, password `checkmate`, then browse to `192.168.1.69/`.

### JTAG debugging

[Espressif's JTAG guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/jtag-debugging/index.html), [a walkthrough video](https://www.youtube.com/watch?v=HGB9PI3IDL0), [Zadig](https://zadig.akeo.ie/) (Windows JTAG driver tool).

## Hardware inventory

**Sensors**: brake-fluid pressure (analog, psi), brake-actuator linear position (analog, inches), mule/car battery voltage (analog, 0-16V), potentiometer knob (analog, UI/sim input), rotary encoder + pushbutton (digital, UI), tachometer (hall-effect pulse train, engine RPM via crank magnet + external ÷8 frequency divider), speedometer (hall-effect pulse train, axle magnets ×2), air-velocity sensor (FS3000-1015, I2C), manifold-air-pressure sensor (SparkFun MicroPressure clone, I2C), 7× DS18B20 OneWire temp sensors (engine, ambient/control-box, 4× wheel, 1× brake motor — auto-address-detected, see [Temperature monitoring](#temperature-monitoring)), capacitive touchscreen (FT6X36-family, I2C), basic-mode toggle switch (repurposes the serial TX pin), onboard ESP "boot" button, HotRC radio receiver (4-channel PWM via the ESP32 RMT peripheral: Horz/Vert analog joystick + Ch3/Ch4 digital toggle buttons).

**Actuators**: throttle servo (PWM, angular degrees), brake DC motor (PWM, H-bridge-style "Jaguar" controller — 4 supported physical actuator models, auto-detected by which brake-motor temp-sensor address is found: Thomson, MotorFactoryStore, GoMotorWorld1/2), steering DC motor (PWM, same Jaguar-style driver, open-loop only — no feedback sensor), starter-motor relay (digital out), ignition relay (digital out), syspower switch (digital out, gates power to sensors/screen for sleep), cooling-fan output (defined but currently unused — `CoolingFan::update()` is a stub, not called from `loop()`).

**Display/UI**: 320×240 ILI9341 SPI TFT touchscreen, 10-pixel NeoPixel strip (WS281x/SK6812) for status/backlight/idiot-lights, rotary encoder + pushbutton, onboard ESP32 boot button.

**Pin map** (ESP32-S3-DevKitC-1, v1 board): full external-circuitry notes (pull-up/down/divider requirements per pin) live as comments in `src/globals.h`, alongside every pin's own reclaimability status — treat that file as the source of truth if this table and the code ever disagree.

| Pin | Role | Note |
| --- | --- | --- |
| 0 | Boot button | Strap pin, must be pulled high at boot |
| 1 | TFT DC | |
| 4 | Mule battery sense | 16V full-scale |
| 5 | Pot | |
| 6 | Brake position | |
| 7 | Brake pressure | |
| 8 / 9 | I2C SDA / SCL | Single shared bus: touch, airvelo, MAP, LightingBox |
| 10-13 | TFT CS, SPI MOSI/SCLK, spare MISO | |
| 14 / 15 | HotRC Ch2 (vert) / Ch1 (horz) | Analog joystick axes, PWM in |
| 16 / 17 / 18 | Gas / brake / steer PWM out | |
| 19 / 20 | Steering encoder A/B | Reserved, unused (or JTAG) |
| 21 | OneWire (temp sensors) | Doesn't work reliably above ~pin 35 |
| 35 | Speedo | ESP32-S3 errata 3.11: erroneous 80ns low glitch possible, avoid interrupts here |
| 36 | Starter relay | Same errata 3.11 caveat |
| 37 | Tach | |
| 38 | Encoder pushbutton | Also the onboard NeoPixel pin on v1.1 boards — documented conflict |
| 39 | Cooling fan (unused) | Same errata 3.11 caveat |
| 40 / 41 | HotRC Ch4 / Ch3 | Digital toggles: syspower/starter/cruise-toggle, and ignition |
| 42 | Encoder A | |
| 43 | Basic-mode switch | Doubles as serial TX — console is paused to sample it |
| 44 | Serial RX | |
| 45 / 46 | Ignition / syspower | Strap pins, must be pulled low at boot |
| 47 | Encoder B | |
| 48 | NeoPixel data | v1 boards (see note above) |

## Program flow

`cantroller2.cpp` is a thin 71-line orchestrator — `#include "objects.h"` pulls in the whole program (chained through every other header), then `#include "unittests.h"`. Everything besides `cantroller2.cpp` is a header-only, `#pragma once`, single-translation-unit design.

**`setup()`** order is deliberate and commented at each step: stray pins + basic-mode-switch read + Serial start, before display double-buffer semaphores, before I2C bus init, before touchscreen setup (shares the I2C mutex), before bringing up the TFT (so boot progress is visible ASAP on the on-screen console). `taskDraw`/`taskPush` (display bus-write/draw) spawn pinned to **the opposite core from `loop()`** — at task priority 4 vs. `loop()`'s priority 1, they'd otherwise preempt and stall it every ~1ms tick if left on the same core. Temp-sensor setup runs next; **which DS18B20 addresses it finds determines `running_on_devboard`** (see [Temperature monitoring](#temperature-monitoring)), which in turn flips several dev-only debug flags off via `set_board_defaults()`. `unittests.run_all()` (pure-logic self-tests, see below) runs before the ESP task watchdog is set up — which is currently **disabled** (`watchdog_enabled = false`; comment: "seems to mess with the hotrc"). Sensor/motor/lightbox/starter setup follows, then NeoPixel setup + its own task, then idiot-lights/diagnostics/ignition/runmode setup (each with an explicit ordering dependency on the previous), landing in `Standby` as the first boot runmode.

**`loop()`** owns one ESP32-S3 core exclusively (no `vTaskDelay` needed — nothing else shares that core) and runs, per iteration: watchdog → ignition → bootbutton → starter → encoder (~20µs) → pot (~400µs — the slowest single `analogRead`, oddly) → brake position (~120µs) → brake pressure (~50µs) → tach → speedo → mule battery → HotRC (~100µs) → `RunModeManager::update()` (~36µs idle — sets gas/brake/steer control targets) → gas/brake/steer motor updates → touch → tuner → diagnostics (~200µs) → console spam-decay → loop timer. `neo.update()` and `lightbox.update()` are deliberately **not** in this list — profiling showed they blocked too long, so both moved to their own tasks on the other core (`taskNeo`, `taskMAF` — the latter also drives `lightbox.update()` since it's already doing multi-ms I2C waits for the airflow/MAP sensors).

**Task/core split, in full**: `loop()` gets its own core; `taskDraw`, `taskPush`, `taskTemp`, `taskMAF`, `taskNeo` all run pinned to the other core, each with an inline comment justifying why (blocking I2C reads, blocking RMT NeoPixel writes, multi-ms display DMA). A single FreeRTOS mutex, `I2C::_busmutex`, now serializes every I2C transaction (touch, airvelo, MAP, LightingBox) across every task and core — replacing a formerly-separate LovyanGFX-internal I2C stack that used to fight the main `Wire` stack for the same physical bus. That contention is on record as the suspected root cause of several previously-mysterious bugs: bus stalls, touch-input corruption, and possibly the `EZReadConsole` hang/reboot described below.

## Run mode state machine

`enum runmode { Basic, LowPower, Standby, Stall, Hold, Fly, Cruise, Cal }`, driven by `RunModeManager::update()` every loop.

Global overrides run first, ahead of per-mode logic: `in_basicmode` forces `Basic`; just-booted or `!ignition.signal` forces `Standby`; `tach.stopped()` forces `Stall` (unless already `LowPower`/`Cal`). `Hold` is skipped entirely if the brake has no feedback sensor (`brake.feedback == _None`) or `simple_brake` is set — routing straight to the preferred drive mode instead. On any transition, `run_motor_ctrlmode[runmode][...]`/`run_motor_action[runmode][...]` (per-mode tables in `globals.h`) get re-applied to gas/brake/steer, and per-mode flags reset — except the Stall-mode 2-minute starter-disable timeout, which is deliberately **preserved across Stall↔Hold transitions** so tach noise can't reset that safety timer. Global safety guards run every loop regardless of mode (except `Cal`): if ignition is on/requested and the radio link is lost, untested, or a Ch3 toggle fires, ignition gets killed — via a **panic stop if the car is moving**, or a plain ignition-off if already stopped.

| Mode | Behavior |
| --- | --- |
| `Basic` | Manual pedal operation only, all motor PIDs disabled, steering still active. Ch4 (ignition off) → `LowPower`. |
| `LowPower` | Cuts `syspower` after a 350ms screen-blackout delay. Wakes on encoder short-press or Ch4, with a 900ms device-warmup delay before landing in Standby/Basic. |
| `Standby` | Shutdown choreography: auto-stop brake (6s timeout) → park motor (3s timeout) → halt. Screensaver arms after 17 min idle, LowPower sleep after 20 min. Ch3 → ignition on (`Stall`); Ch4 → sleep request. |
| `Stall` | Engine not running — gas is open-loop (no tach feedback yet), brake tracks the joystick directly. Ch4 triggers the starter (gated by the phantom-start mitigations below). Exits to `Hold` (or the preferred drive mode under `simple_brake`) once `!tach.stopped()`. |
| `Hold` | Waits for the trigger to be pulled after having centered at least once, then moves to the preferred drive mode. Ch4 toggles the preferred drive mode 500ms after the starter last ran. |
| `Fly` | Must keep pulling the trigger until the car actually moves, or it drops back to `Hold`. Once moving, releasing the trigger no longer exits `Fly` unless the car stops (and `!simple_brake`). |
| `Cruise` | Locks throttle target, adjustable via the trigger. Auto-drops to `Hold` if coasted to a stop (4s grace) or explicitly braked to a stop. Safety gesture: holding the trigger full-down for 500ms drops to `Fly` ("driver could be confused and panicking") — skipped if `cruise_brake` is enabled, since Cruise already brakes. |
| `Cal` | Deliberately hard to enter — allows unconstrained motor control for calibration. Gas/brake independently toggle into `CtrlCalibrate` via separate request flags. |

## Sensors

Class hierarchy (asserted at compile time in `unittests.h`):

```text
Device → Transducer → Sensor → I2CSensor  → AirVeloSensor, MAPSensor
                              → AnalogSensor → CarBattery, PressureSensor, BrakePositionSensor
                              → PulseSensor  → Tachometer, Speedometer
```

Plus standalone `Potentiometer`, `Param`, `Simulator`, `RMTInput`, `Hotrc`.

**`Param`**: a constrained float that also remembers its previous value, with min/max either owned internally or *shared by pointer* with another `Param` (letting one sensor's operating limits live-link to another's). `set()` reports whether the value changed **after** constraining — a NaN transition always counts as a change.

**`Device`/`Transducer`**: the shared base every sensor and actuator sits on. Each has a runtime-selectable `src` (Fixed/Pin/Sim/Pot) and three parallel value representations — `_native`, `_raw` (unfiltered, SI units), `_si` (filtered, SI units) — converted via `LinearMath`/`AbsLimMap`/`OpLimMap` and a `TransDir` (Fwd/Rev — e.g. brake position is `Rev` since a smaller inch reading means more brake applied).

**`I2CSensor`**: neither I2C sensor (MAP, AirVelo) has a hardware reset pin wired to the MCU, so a sensor that comes up "stuck" reporting NaN after a reflash typically needs a full power cycle, not just a reset (suspected internal sensor power-on race). `_stuck_recovery_timer` (10s) periodically retries re-`begin()`-ing it as a software-only mitigation.

- **`AirVeloSensor`** (FS3000-1015, I2C `0x28`): mph, op range 0-27.36 mph (from carburetor venturi geometry), EMA alpha 0.2.
- **`MAPSensor`** (SparkFun MicroPressure clone, I2C `0x18`): atm, op range 0.68-1.02 atm. Non-blocking read (12ms retry timeout) with a continuity filter that **rejects implausible steps** (>0.5 atm jump from the last good value) and requires even the *initial* seed reading to already be within the op range — a bad seed is otherwise effectively permanent, so this check is explicitly flagged in comments as load-bearing, not to be relaxed.

**`AnalogSensor`**: `_detected` is inferred at boot purely from whether the raw ADC reading falls inside `absmin_native`/`absmax_native`.

- **`CarBattery`**: 0-15.1V abs, 10.7-14.8V op; conversion factor `0.004075` (multimeter-calibrated against the real board).
- **`PressureSensor`** (brake fluid, psi): linear conversion derived from the sensor's 0.5-4.5V output spec against the 3.3V ADC ceiling (`m=0.202 psi/adc, b=-131.3 psi`). Extensive inline comments document a step-by-step recalibration wizard (zero point, op-max, EMA alpha) plus preserved historical calibration values for provenance.
- **`BrakePositionSensor`** (inches, reverse direction): abs range 1.22-7.26in, mapped from ADC 1885-2933, zero point 5.9in, margin 0.2in. Also carries an in-progress multi-step bench+car calibration procedure (as comments) for tuning each new brake-motor model.

**`PulseSensor`** (`Tachometer`/`Speedometer` base): hall-effect pulse timing via a FALLING-edge ISR with a debounce-reject threshold. Uses a **`scaling_ema_filt()`** whose effective alpha scales with actual elapsed time since the last call (`alpha = dt/(tau+dt)`), so filtering stays smooth across variable pulse rates, capped after long gaps. `stopped()` compares the filtered value to `opmin` within a margin.

- **`Tachometer`**: ÷8 external frequency divider, abs 0-4800rpm, op 0-3600rpm ("redline"), governed max = `opmax × governor/100`, idle 590rpm (680 cold, 500 hot), stop-timeout 1.0s (tuned past the ~960ms hot-idle pulse period).
- **`Speedometer`**: ×2 (two magnets/wheel rotation), 20-inch wheel diameter assumed in the mph conversion, abs 0-18mph, op 0-15mph, stop-timeout 1.3s.

**`Simulator`**: tracks which sensors can be simulated (`can_sim` bitmask) and whether via full simulator mode or the single-sensor pot-map (only one sensor can be pot-mapped at a time); both persist to flash.

**`RMTInput`**: wraps the ESP32 RMT peripheral in RX mode to read RC PWM pulse widths. Comment notes an unexplained hardware/driver quirk: if the code ever stops reading an RMT object, it floods the console with RMT failures — worked around by always reading.

**`Hotrc`**: the 4-channel RC transmitter/receiver interface (Horz/Vert analog joystick, Ch3/Ch4 digital toggles). Per-hardware-generation calibration tables live in comments (multiple receiver-module revisions, each with its own measured µs values). A custom 9-deep-ring-buffer **`spike_filter()`** rejects/interpolates transient PWM glitches the HotRC hardware occasionally produces, at the cost of ~9-reading input latency. Radio-lost failsafe logic distinguishes `_radiolost` (currently in failsafe range) from `_radiolost_untested` (failsafe condition never yet observed since boot, so detection itself is unverified) — `require_radiolost_test` refuses ignition-on until the latter clears. `_rc_ever_powered` latches once any channel reads above 1200µs, to distinguish "transmitter genuinely off" from the 1000µs boot-default reading every channel starts at.

## Motors

**`QPID`**: a heavily modified QuickPID (itself based on Arduino `PID_v1`). Beyond stock PID: proportional-on-error/measurement/blended modes, derivative-on-error/measurement, several anti-windup modes (conditional/clamp/off/round/round-conditional), a **centmode** feature (off/on/strict) that zeroes the integral and snaps the output sum back to a defined center whenever error crosses zero (used by the brake PIDs to stop integral windup carrying across a sign change), and an output rate limiter applied inside `compute()`. A comment flags a fixed historical bug: the conditional anti-windup math was previously missing the prospective P+I output's accumulated-integral term.

**`ServoMotor`** (base) → **`JagMotor`** (adds a bidirectional H-bridge-style model with a center `Stop` value; its `derive()` recomputes operating limits as a fraction of the *live measured battery voltage* — the abs range is literally `±mulebatt->val()`) → `ThrottleControl`, `BrakeControl`, `SteeringControl`.

- **`ThrottleControl`**: PID gains `kp=0.13, ki=0.50, kd=0.00` (separate gain pairs for Cruise mode depending on whether gas is itself open-loop or PID). Op range 66.3-165° servo angle, parked at 55°. Optional (off by default) idle-boost raises the idle target as engine temp reads colder — 680rpm cold idle to 500rpm hot, boosted up to 15% over a 60-80°F window. Throttle response can be exponent-linearized (default exponent 3.75). Two cruise-adjustment schemes: continuous rate-based adjustment while the trigger's off-center, or a one-shot adjustment proportional to peak trigger deflection.
- **`BrakeControl`** (the most complex motor class): two independent PID loops — position (`kp=22.0, ki=5.5`) and pressure (`kp=0.54, ki=0.10`) — blended via a **hybrid feedback scheme**: a sigmoid transition between 25% and 50% of full pressure range shifts control authority smoothly from position-dominant (low pressure, where position sensing is accurate) to pressure-dominant (high pressure, where position barely changes further). Motor-thermal protection disables the brake control mode above its operating temperature ceiling. An **auto-stop/auto-hold state machine** ramps brake pressure in increments (2.5%/tick normally, 4.0%/tick while panicking or already stopped) on a 250ms interval, with an 8s overall timeout. A positional-limit check halts the motor rather than let it drive the actuator past its mechanical stops. A fixed historical bug is documented here too: the `releasing` flag used to never get updated in this code path, leaving it stale — the on-screen icon showed "releasing" while the brake was actually being pressed. Open-loop modes: median-point hold (disallowed under `simple_brake`), auto-release-on-trigger-release, and auto-release-holdable (default — holds anywhere between full release and full press based on trigger position).
- **`SteeringControl`**: simplest motor — open-loop only, no PID, no feedback sensor. Has a documented safety feature, **speed-dependent steering-authority reduction**: at max vehicle speed, the steering endpoint is attenuated by 25% (linear with speed), preventing full-lock steering input while moving fast.

## Inputs: buttons, encoder, touchscreen

**`MomentarySwitch`**: generic debounced button with short/long press detection (300ms threshold); double-reads the pin to reject a documented ~70ns invalid-low glitch window.

**`Encoder`**: quadrature rotary encoder, ISR-driven, with **two alternate decode algorithms** selected by a compile-time define — two different cheap encoder hardware variants are in use across dev/vehicle boards, each needing different edge-interpretation logic. Computes a live spin-rate-based acceleration factor (up to 25×) for fast UI scrolling; a critical section around the shared delta counter avoids an ISR lost-update race.

**`Touchscreen`**: wraps SensorLib's `TouchDrvFT6X36` (Wire-based capacitive touch, I2C `0x38`). Previously LovyanGFX's own independent I2C driver stack handled touch, fighting the main `Wire` stack for the same physical bus — now unified onto one `Wire` stack serialized by `I2C::_busmutex` (see [Program flow](#program-flow)). Implements filtered touch state (6.5ms debounce, "needed for using through plastic box lid"), tap/double-tap (400ms)/long-press (550ms)/swipe (50px min)/drag/held detection, and an acceleration-ramping edit-delta that doubles every 400ms. `process_ui()` decodes touches into a 6×5 grid of touch boxes mapped to specific UI actions (page/select/+/-/screensaver, sim toggles, sensor nudges) — effectively the whole touchscreen menu dispatch table.

## Display and UI

**TFT**: ILI9341, 320×240, SPI (40MHz write / 16MHz read clock). Touch is deliberately *not* configured through LovyanGFX (see `Touchscreen` above, to avoid the dual-I2C-stack bus-contention bug). A 256-entry RGB332→RGB565 lookup table and 4 hand-encoded 8bpp icon bitmaps (arrows, a 145×74 "mule chassis" image) live in `tftsetup.h`.

**Layout**: a single fixed-layout dashboard, double-buffered across two tasks — `draw_task` paints into an off-screen sprite, `push_task` diffs it against the last-drawn sprite and DMAs only the changed pixel runs (`diffpush()`). 8 fixed telemetry rows (HotRC Vert/Horz %, Speed, Tach, Brake Sensor %, Throttle %, Brake Motor %, Steering Motor %) with bargraphs, needles, and targets are always visible. A scrollable 14-page tuning/telemetry dataset (run state, HotRC, sensors, pulse sensors, PWMs, idle, motors, brake PID, gas PID, cruise PID, temps, sim, diagnostics, UI settings) — up to 15 named, unit-labeled rows each, wired directly to live variables via pointer/`tune()` edits, functioning as an in-field engineering console. Side menu: PAG/SEL/+/-/ANI; top menu: CAL/SIM/CH4/IGN. An idiot-light row shows 12 icons across 3 rows (see [Diagnostics and safety](#diagnostics-and-safety)). A small "app panel" viewport hosts one of three mini-apps at a time (`EZReadUI` console / `MuleChassisUI` static diagram / `ScreensaverUI` full-screen animation). Custom 13×7 bitmap glyphs cover unit strings too wide for the tiny font (µs, %, Ω, °, mph, rpm, psi, atm, g/s, adc). A `Tuner` class mediates encoder/touch edits into live variable changes on whichever datapage row is selected.

**On-screen animations** (`animations.h`, TFT-only — distinct from both the NeoPixel strip and the carpet lights): `CollisionsSaver` (a bouncing-ball elastic-collision screensaver, up to 35 balls, slowly-rotating "meandering" gravity vector — its near-full-screen background repaint previously overwhelmed the draw/push tasks badly enough to trip the task watchdog and reboot the board; now relies on `draw_task`'s self-regulating backoff rather than a hard frame cap), `EraserSaver` (a pattern-drawing screensaver), `EZReadDrawer` (renders the console log into the app panel, snapshotting lines under mutex to avoid tearing), and `PanelAppManager` (arbitrates which mini-app owns the panel viewport, and draws touch buttons for manually poking simulated sensor values in sim mode).

## I2C bus and the LightingBox link

Single shared `Wire` bus (SDA 8 / SCL 9), every transaction serialized through `I2C::_busmutex` across all tasks/cores. Devices: touch controller (`0x38`), **LightingBox** (`0x69`, the CarpetLightCode firmware — see its own README), AirVelo sensor (`0x28`), MAP sensor (`0x18`). `I2C::setup()` scans addresses 1-126 at boot and logs found vs. expected devices; `Wire.setTimeOut(25)` (25ms max SCL hold, vs. the 50ms default).

**LightingBox protocol**, quoted from the source comment (also documented from the receiving side in `CarpetLightCode/README.md`):

> 1st nibble of 1st byte contains 4-bit command/request code. The 2nd nibble and any additional bytes contain data, as required by the code.
> code: `0x0F` = status flags. F bits: 0=syspower, 1=panicstop, 2=warning, 3=alarm
> code: `0x1R` = entered runmode given by R (no additional bytes)
> code: `0x2H,0xLL` = speedometer value update. 12-bit value contained in HLL is speed in hundredths-of-mph

`LightingBox::update()` sends at most one message per 250ms tick, prioritized status > runmode > speed. Status and runmode only transmit on change; **speed always transmits every tick**, doubling as the bus presence-detection heartbeat (a NACK on `Wire.endTransmission()` marks the device undetected). There's deliberately no separate periodic presence probe — an earlier attempt at one was reverted, since a NACK'd probe could trigger the ESP32 I2C driver's slow recovery path, stalling the whole bus for "the better part of a second" (this used to corrupt concurrent touch reads back when touch had its own independent I2C stack — fixed by the unification described in [Program flow](#program-flow)).

Also on this bus: **`SparkFun_MicroPressure`** (MAP sensor) is a vendored, modified copy of the SparkFun library (patched for non-blocking reads, avoiding a 6-7ms block per read); **`FS3000`** (air velocity) is likewise vendored, with a fixed bug: `readRaw()` now treats a raw count of exactly `0` as the I2C-failure sentinel `0xFFFF`, since an all-zeros buffer was silently passing the original library's checksum.

## NeoPixel strip

10 physical LEDs on the vehicle PCB (devboard has 2 fewer — a known quirk), `NeoPixelBus<NeoGrbFeature, NeoSk6812Method>`. Layout: pixel 0 = onboard ESP/box backlight, cosine-pulses in the current runmode's color (dimmed and slowed in LowPower); pixel 1 = PCBA backlight, sine-pulses the same color, phase-offset from pixel 0; pixel 2 = external-strip mode indicator, normally mirrors pixel 1, except in LowPower where it joins pixels 3-9 in a Cylon/Knight-Rider sweep instead; pixels 3-9 mirror the first 7 (true hazard-light subset) of the on-screen idiot lights, each capable of solid-on, multi-color error flashing, or a distinct critical-alert strobe (3 pulses/sec for 1s, then 2s off).

Every pixel write funnels through one gamma-correction choke point (`neoSetPixelColor()`/`neoSetPixelColorDimmed()`) — PWM duty cycle is linear but perceived brightness isn't, so this keeps the whole strip visually consistent; the "Dimmed" variant gamma-corrects *before* the extra linear dim, specifically to avoid collapsing a smooth fade into the gamma table's near-zero flat region. LowPower mode's Cylon sweep (pixels 2-9) runs continuously, independently drifting hue at each end of the strip (8-minute cycle one end, 6-minute the other, interpolated between) plus an oscillating saturation (5-minute period, 79-100%) — re-randomized fresh on each genuine entry into LowPower, not each ~2.75s sweep restart. A `flashdemo_ena()` test mode repurposes the 7 idiot-light pixels to demo 6 generic light-based value-encoding techniques (bitwise blink, duty-cycle, danger-strobe overlay, blackout-count, blink-rate, hue mapping) — a prototyping tool, not part of normal operation.

## Temperature monitoring

7 DS18B20 OneWire sensor locations: engine, ambient, 4× wheel (FL/FR/RL/RR), brake motor. Per-category °F limits (op min/max, alarm):

| Category | Op min | Op max | Alarm |
| --- | --- | --- | --- |
| Engine | 40 | 218 | 205 |
| Wheel (all 4) | 40 | 170 | 145 |
| Brake motor | 45 | 165 | 130 |
| Ambient | 40 | 135 | 120 |

Each physical DS18B20 has a fixed factory 64-bit address; the firmware carries a table of known addresses per location, including **4 possible brake-motor addresses corresponding to 4 different physical actuator models** (Thomson, MotorFactoryStore, GoMotorWorld1/2) — whichever is actually detected sets `brakemotor_type_detected`, auto-selecting the matching brake calibration. `vehicle_detected` is inferred from whether the ambient sensor (glued to the control box, always present on the real vehicle) was found — the same signal `running_on_devboard` (see [Program flow](#program-flow)) is derived from. Unrecognized sensor addresses auto-fill whichever known location slot is still empty. Non-blocking conversion state machine, one sensor read per ~1s tick, 11-bit resolution.

## Diagnostics and safety

**`LoopTimer`**: rolling 100-sample average + all-time-peak `loop()` iteration timing; warns above a 100ms max-allowable-loop-time.

**`DiagRuntime`** (`diag`): polled every 175ms, the central health-monitoring system. Registers ~24 telemetry channels (motor outputs, every sensor, HotRC channels, battery, temps), each classified into `ErrLost` (not responding), `ErrRange` (outside op range ± margin), or `ErrWarn` (soft warning). Dedicated per-loop checks include temperature failure (also computes alarm-strobe frequencies for the NeoPixel strip, scaling 1/3Hz→8.3Hz as temperature climbs from alarm threshold to op max, and enforces overtemp ignition/brake cutoffs), battery failure, speedo/tach failure (confirms the car actually accelerates under throttle and stays at zero when parked), and HotRC failure. Brake cross-sensor consistency checks exist but are **currently disabled** — they caused a false "brake position lost" error immediately on boot and need cleanup before re-enabling (a documented, known limitation, not an oversight). A large in-source design-notes block lists dozens of not-yet-implemented failure heuristics (brake chain slip, actuator stall detection, air-filter clog detection via MAF/throttle ratio drift, and others) as a roadmap.

**`BootMonitor`** (`watchdog`): an NVS-backed crash/postmortem system, currently more load-bearing than the disabled ESP hardware watchdog (see [Program flow](#program-flow)). Persists a coarse "what were we doing" status (booting/parked/stopped/driving/panicking/etc.), runmode, panic-stop state, and ignition state to flash on every meaningful change, plus boot/crash counters and rounded uptime. On boot it prints a human-readable postmortem (e.g. "bootcount: N (crashed), last lost power while driving in Cruise mode, after 12 min uptime"). An optional (off by default) setting can force a panic state on boot if the last recorded state was mid-drive.

**Idiot lights** (`idiots.h`): a 36-icon, 3-row × 12 on-screen warning panel. Row 1 (12 icons, the true hazard subset) is also mirrored onto the 7 physical NeoPixels: sensor-lost, sensor-range, radio-lost (tested/untested), panic-stop, brake auto-stopping/auto-holding, parking, releasing. Row 2: informational status (cruise-adjusting, car-hasn't-moved, starter-running, brake-PID-active, brake-no-feedback, speedo/tach activity + stopped states, touch-active, shutting-down, running-on-devboard). Row 3: one light per actuator/sensor error group from `diag.h` (throttle/brake/steer/HotRC/speedo/tach/pressure/position/temps/battery/GPIO/other).

**Safety interlocks summary** (this firmware drives real brakes/throttle/steering on a moving vehicle): ignition kill on radio loss, untested failsafe, or manual Ch3 toggle — via panic-stop braking rather than a plain power cut if the car is moving; the panic flag persists to flash so a post-crash reboot can detect it; starter motor gated behind brake-applied-and-confirmed-holding, a recent-input-activity requirement, an optional double-click, and a 2-minute stall-mode timeout, against "phantom start" risk; a `check_for_external_tampering()` check compares the starter's actual pin level against its tracked state every loop and panics if they diverge; overtemp shutoffs for engine/wheel/brake; brake positional-limit enforcement against driving the actuator past its mechanical stops; steering authority reduction at speed; `unittests.h` explicitly flags `constrain()` and PID output-clamping checks as safety-critical (colored red, not orange, in its output).

## Persisted settings

Via `Preferences prefs` (NVS partition "FlyByWire"): `pref_drivemode`, `cansim`, `potmap`, `codestatus`, `runmode`, `panicstop`, `ignition`, `bootcount`, `crashcount`, `uptime`, `dpage`. Together these let the system resume its preferred drive mode, remember simulator config, and report a boot-time crash postmortem.

## Known limitations and future work

- Brake cross-sensor consistency checks in `diag.h` are written but disabled (false-positive on boot; needs cleanup).
- `CoolingFan` is a defined but unused stub — no thermostat logic wired to `loop()` yet.
- SD card interface (integrated with the touchscreen module) is not yet implemented; intended for runtime data logging.
- `unittests.h` covers pure-logic/structural consistency only (compile-time asserts, math helpers, lookup-table completeness) — it does not exercise real hardware (I2C, PWM, encoder/touch) or run property/fuzz tests; both are flagged as future work in-source.
- `diag.h` carries an extensive in-source list of not-yet-implemented failure-detection heuristics (see [Diagnostics and safety](#diagnostics-and-safety)).

## Links

- [Flying Carpet Information Dumpster](https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA/edit#heading=h.uaks6l1vfqun)
- [Schematics, board layout, wiring diagrams](https://drive.google.com/drive/u/0/folders/1AAUnWQhdA940hJz0VnBCmHV5c0VyWric)
- [Bug list](https://docs.google.com/spreadsheets/d/1_FchfMjr4O9Q0fOcC0f2aJvmvfMjqayLLSsnorOU5c0/edit?gid=0#gid=0)
- SD card: 16GB, pre-formatted FAT32 (Windows, 32kB allocation unit) — macOS reads this as "MS-DOS (FAT32)". [SD card formatter](https://www.sdcard.org/downloads/formatter/) if reformatting.
- Image/color tooling: [free icon images](http://iconarchive.com/) (resize as needed, JPG with black background), [RGB565 converter](https://www.rinkydinkelectronics.com/t_imageconverter565.php), [RGB565 color picker](http://www.barth-dev.de/online/rgb565), [RGB332 color wheel](https://roger-random.github.io/RGB332_color_wheel_three.js), [named colors reference](https://wiki.tcl-lang.org/page/Colors+with+Names).
- Fonts: [font0 character map](https://learn.adafruit.com/assets/103682) (use the right-side map), [TomThumb font info](https://robey.lag.net/2010/01/23/tiny-monospace-font.html), [TomThumb character map](https://fontstruct.com/fontstructions/show/1656341/tom-thumb).
