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
4. Create a PlatformIO project targeting board `esp32-s3-devkitc-1` (`platformio.ini`'s actual `board` value) pointed at the `CANTroller2` folder.

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

Source is `CANTroller2/src`; runtime filesystem content is `CANTroller2/data` (compiles to a `littlefs` image per `platformio.ini`'s `board_build.filesystem = littlefs`, partitioned via `partitions.csv`). Before first programming a board, erase flash once (alien head → Platform/Erase Flash) to partition correctly — only needed again if the partition map changes. Alien head → "Platform/Upload Filesystem Image" writes `data/` (compiled to `.pio/build/littlefs.bin`) over USB; alien head → "Upload" writes `src/` (compiled to `.pio/build/firmware.bin`).

**No OTA/wifi upload path currently** — `platformio.ini` has WiFi support and `ElegantOTA` explicitly commented out (`-DWifiSupported=0`/`-DELEGANTOTA_USE_ASYNC_WEBSERVER=1`, both flagged "commenting due to removal of all radio features"), and no networking code remains anywhere in `src/` (only a dead, never-read `wifi_client_mode` bool survives). Upload over USB only until radio support is reinstated — see [Known limitations](#known-limitations-and-future-work).

### JTAG debugging

[Espressif's JTAG guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/jtag-debugging/index.html), [a walkthrough video](https://www.youtube.com/watch?v=HGB9PI3IDL0), [Zadig](https://zadig.akeo.ie/) (Windows JTAG driver tool).

## Hardware inventory

**Sensors**: brake-fluid pressure (analog, psi), brake-actuator linear position (analog, inches), mule/car battery voltage (analog, 0-16V), potentiometer knob (analog, UI/sim input), rotary encoder + pushbutton (digital, UI), tachometer (hall-effect pulse train, engine RPM via crank magnet + external ÷8 frequency divider), speedometer (hall-effect pulse train, axle magnets ×2), air-velocity sensor (FS3000-1015, I2C), manifold-air-pressure sensor (SparkFun MicroPressure clone, I2C), 7× DS18B20 OneWire temp sensors (engine, ambient/control-box, 4× wheel, 1× brake motor — auto-address-detected, see [Temperature monitoring](#temperature-monitoring)), capacitive touchscreen (FT6X36-family, I2C), basic-mode toggle switch (repurposes the serial TX pin), onboard ESP "boot" button, HotRC radio receiver (4-channel PWM via the ESP32 RMT peripheral: Horz/Vert analog joystick + Ch3/Ch4 digital toggle buttons).

**Actuators**: throttle servo (PWM, angular degrees), brake DC motor (PWM, H-bridge-style "Jaguar" controller — 4 supported physical actuator models, auto-detected by which brake-motor temp-sensor address is found: Thomson, MotorFactoryStore, GoMotorWorld1/2), steering DC motor (PWM, same Jaguar-style driver, open-loop only — no feedback sensor), starter-motor relay (digital out), ignition relay (digital out), syspower switch (digital out, gates power to sensors/screen for sleep), cooling-fan output (defined but currently unused — `CoolingFan::update()` is a stub, and both `fan.setup()`/`fan.update()` are commented out at their `cantroller2.cpp` call sites, not just internally empty).

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
| 35 | Speedo | |
| 36 | Starter relay | ESP32-S3 errata 3.11: erroneous ~80ns low glitch possible on this pin at RTC-peripheral power-up, avoid interrupts here |
| 37 | Tach | |
| 38 | Encoder pushbutton | Also the onboard NeoPixel pin on v1.1 boards — documented conflict |
| 39 | Cooling fan (unused) | Same errata 3.11 caveat as pin 36 |
| 40 / 41 | HotRC Ch4 / Ch3 | Digital toggles: syspower/starter/cruise-toggle, and ignition |
| 42 | Encoder A | |
| 43 | Basic-mode switch | Doubles as serial TX — console is paused to sample it |
| 44 | Serial RX | |
| 45 / 46 | Ignition / syspower | Strap pins, must be pulled low at boot |
| 47 | Encoder B | |
| 48 | NeoPixel data | v1 boards (see note above) |

## Program flow

`cantroller2.cpp` is a thin 71-line orchestrator — `#include "objects.h"` pulls in the whole program (chained through every other header), then `#include "unittests.h"`. Everything besides `cantroller2.cpp` is a header-only, `#pragma once`, single-translation-unit design.

**`setup()`** order is deliberate and commented at each step: stray pins + basic-mode-switch read + Serial start, before display double-buffer semaphores, before I2C bus init, before touchscreen setup (shares the I2C mutex), before bringing up the TFT (so boot progress is visible ASAP on the on-screen console). `taskDraw`/`taskPush` (display bus-write/draw) spawn pinned to **the opposite core from `loop()`** — at task priority 4 vs. `loop()`'s priority 1, they'd otherwise preempt and stall it every ~1ms tick if left on the same core. Temp-sensor setup runs next; **which DS18B20 addresses it finds determines `running_on_devboard`** (`cantroller2.cpp`: `running_on_devboard = !tempsens.setup()`, see [Temperature monitoring](#temperature-monitoring)), which in turn flips several dev-only debug flags off via `set_board_defaults()`. `unittests.run_all()` (pure-logic self-tests, see below) runs before the ESP task watchdog is set up — which is currently **disabled** (`watchdog_enabled = false`; comment: "disabled cuz it seems to mess with the hotrc (?)"). Sensor/motor/lightbox/starter setup follows, then NeoPixel setup + its own task, then idiot-lights/diagnostics/ignition/runmode setup (each with an explicit ordering dependency on the previous), landing in `Standby` as the first boot runmode.

**`loop()`** owns one ESP32-S3 core exclusively (no `vTaskDelay` needed — nothing else shares that core) and runs, per iteration: watchdog → ignition → bootbutton → starter → encoder (~20µs) → pot (~400µs — the slowest single `analogRead`, oddly) → brake position (~120µs) → brake pressure (~50µs) → tach → speedo → mule battery → HotRC (~100µs) → `RunModeManager::update()` (~36µs idle — sets gas/brake/steer control targets) → gas/brake/steer motor updates → touch → tuner → diagnostics (~200µs) → console spam-decay → loop timer. `neo.update()` and `lightbox.update()` are deliberately **not** in this list — profiling showed they blocked too long, so both moved to their own tasks on the other core (`taskNeo`, `taskMAF` — the latter also drives `lightbox.update()` since it's already doing multi-ms I2C waits for the airflow/MAP sensors).

**Task/core split, in full**: `loop()` gets its own core; `taskDraw`, `taskPush`, `taskTemp`, `taskMAF`, `taskNeo` all run pinned to the other core, each with an inline comment justifying why (blocking I2C reads, blocking RMT NeoPixel writes, multi-ms display DMA). `taskDraw`/`taskPush`/`taskMAF`/`taskNeo` share priority 4; **`taskTemp` runs at priority 6** (comment: "highest of the tasks sharing this core... doesn't affect `loop()`, which is alone on the other core; ESP-IDF internals run above 24") — so it wins scheduling contention against the other four whenever they'd otherwise compete. `taskMAF` additionally waits for `bootup_complete` before its first iteration, and self-throttles with an extra 300ms backoff whenever neither the AirVelo nor MAP sensor is I2C-detected. A single FreeRTOS mutex, `I2C::_busmutex`, now serializes every I2C transaction (touch, airvelo, MAP, LightingBox) across every task and core — replacing a formerly-separate LovyanGFX-internal I2C stack that used to fight the main `Wire` stack for the same physical bus. That contention is on record as the suspected root cause of several previously-mysterious bugs: bus stalls, touch-input corruption, and possibly the `EZReadConsole` hang/reboot (its own inline comments in `globals.h` cover this in more depth than fits here).

**Duplicate safety-guard logic, flagged as an open question in-source**: `Ignition::update()` independently re-implements the same radio-loss/panic-on-signal-loss guard `RunModeManager::update()` already runs (see [Run mode state machine](#run-mode-state-machine) below) — both check `hotrc.radiolost()`/`radiolost_untested()` and can independently set `panicreq`. The code carries its own dated `// TODO - is this redundant to code in run.update()?` next to the `Ignition` copy; nothing in the codebase currently resolves whether the two can double-fire or conflict.

## Run mode state machine

`enum runmode { Basic, LowPower, Standby, Stall, Hold, Fly, Cruise, Cal }`, driven by `RunModeManager::update()` every loop.

Global overrides run first, ahead of per-mode logic — all three wrapped in the same guard, not just the last one: `in_basicmode` forces `Basic`; just-booted or `!ignition.signal` forces `Standby`; `tach.stopped()` forces `Stall` (unless already `LowPower`/`Cal`). `Hold` is skipped whenever the brake has no feedback sensor (`brake.feedback == _None`) — where it routes depends on the mode being left: to `_preferred_drivemode` from `Stall`/`Standby`, to `Standby` from `Basic`/`Cal`/`LowPower`, or it just stays at `_oldmode` from any other (driving) mode, "so tach noise can't drop a moving car out of its current mode." Separately, entering `Hold` while `simple_brake` is set reverts to `_oldmode` outright — not to the preferred drive mode (in practice this looks like defensive dead code, since every real path that would set `runmode = Hold` is itself already gated against firing while `simple_brake` is true). On any transition, `run_motor_ctrlmode[runmode][...]`/`run_motor_action[runmode][...]` (per-mode tables in `globals.h`) get re-applied to gas/brake/steer, and per-mode flags reset — except the Stall-mode 2-minute starter-disable timeout, which is deliberately **preserved across Stall↔Hold transitions** so tach noise can't reset that safety timer. Global safety guards run every loop regardless of mode (except `Cal`): if ignition is on/requested and the radio link is lost, untested, or a Ch3 toggle fires, ignition gets killed — via a **panic stop if the car is moving**, or a plain ignition-off if already stopped.

| Mode | Behavior |
| --- | --- |
| `Basic` | Manual pedal operation only, all motor PIDs disabled, steering still active. Ch4 (ignition off) → `LowPower`. |
| `LowPower` | Cuts `syspower` after a 350ms screen-blackout delay. Wakes on encoder short-press or Ch4, with a 900ms device-warmup delay before landing in Standby/Basic. |
| `Standby` | Shutdown choreography: auto-stop brake (6s timeout) → park motor (3s timeout) → halt. Screensaver arms after 17 min idle, LowPower sleep after 20 min. Ch3 → ignition on (`Stall`) — gated on the radio-lost test having passed (`!radiolost() && !radiolost_untested()`); Ch4 → sleep request. |
| `Stall` | Engine not running — gas is open-loop (no tach feedback yet), brake tracks the joystick directly. Ch4 triggers the starter (gated by the phantom-start mitigations below). Exits to `Hold` (or the preferred drive mode under `simple_brake`) once `!tach.stopped()`. |
| `Hold` | Waits for the trigger to be pulled after having centered at least once, then moves to the preferred drive mode. Ch4 *can* toggle the preferred drive mode 500ms after the starter last ran, but only if `holdmode_ch4_drivetoggle` is set — it **defaults false**, so this is inert out of the box. |
| `Fly` | Must keep pulling the trigger until the car actually moves, or it drops back to `Hold`. Once moving, releasing the trigger no longer exits `Fly` unless the car stops (and `!simple_brake`). Ch4 unconditionally toggles straight to `Cruise` (no flag gate, unlike `Hold`'s toggle) whenever the starter isn't currently running. |
| `Cruise` | Locks throttle target, adjustable via the trigger. Auto-drops to `Hold` if coasted to a stop (4s grace) or explicitly braked to a stop — gated by `!simple_brake`, same as `Fly`'s equivalent auto-drop. Safety gesture: holding the trigger full-down for 500ms drops to `Fly` ("driver could be confused and panicking") — skipped if `cruise_brake` is enabled, since Cruise already brakes. Ch4 unconditionally toggles back to `Fly` (same ungated mechanism as `Fly`'s own Ch4 toggle) whenever the starter isn't running. |
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

**`I2CSensor`**: neither I2C sensor (MAP, AirVelo) has a hardware reset pin wired to the MCU, so a sensor that comes up "stuck" reporting NaN after a reflash typically needs a full power cycle, not just a reset (suspected internal sensor power-on race). `_stuck_recovery_timer` (10s) periodically retries re-`begin()`-ing it as a software-only mitigation. Both `AirVeloSensor::setup()`/`MAPSensor::setup()` trust the boot-time I2C bus scan over the sensor library's own `begin()` return value, since `begin()`'s internal ACK check can spuriously fail if the shared `Wire` bus was reinitialized by LovyanGFX (the touchscreen driver) between the scan and `begin()` — a previously-debugged real interaction with the touch subsystem.

- **`AirVeloSensor`** (FS3000-1015, I2C `0x28`): mph, op range 0-27.36 mph (from carburetor venturi geometry), EMA alpha 0.2. The vendored `FS3000` driver supports 2 calibration tables (`AIRFLOW_RANGE_7_MPS`, the default 0-7.23 m/s span, or `AIRFLOW_RANGE_15_MPS`, 0-15 m/s), each with its own raw-count lookup table.
- **`MAPSensor`** (SparkFun MicroPressure clone, I2C `0x18`): atm, op range 0.68-1.02 atm. Non-blocking read is a real 2-call state machine (`readPressure(units, blocking=false)`) — the first call issues the measurement command and immediately returns NaN with a `MapErrWaiting` flag; the caller must call again ≥8ms later (past the ~6.4ms max real conversion time) to get the actual value, reading 4 raw bytes directly rather than polling a busy flag (that path exists only as a defensive fallback). A continuity filter **rejects implausible steps** (>0.5 atm jump from the last good value) and requires even the *initial* seed reading to already be within the op range — a bad seed is otherwise effectively permanent, so this check is explicitly flagged in comments as load-bearing, not to be relaxed. A separate `_respect_integrity_errors` flag is permanently set `false`, with an in-source TODO calling it a workaround for "constant integrity flag errors" on the dev board that was never actually debugged.

**`AnalogSensor`**: `_detected` is inferred at boot purely from whether the raw ADC reading falls inside `absmin_native`/`absmax_native` — detection itself is boot-time-only and never re-evaluated, though reads continue afterward regardless ("in case sensor is intermittent after boot").

- **`CarBattery`**: 0-15.1V abs, 10.7-14.8V op; conversion factor `0.004075` (multimeter-calibrated against the real board).
- **`PressureSensor`** (brake fluid, psi): linear conversion derived from the sensor's 0.5-4.5V output spec against the 3.3V ADC ceiling (`m=0.202 psi/adc, b=-131.3 psi`). Extensive inline comments document a step-by-step recalibration wizard (zero point, op-max, EMA alpha) plus preserved historical calibration values for provenance.
- **`BrakePositionSensor`** (inches, reverse direction): abs range 1.22-7.26in, mapped from ADC 1885-2933, zero point 5.9in, margin 0.2in. Also carries an in-progress multi-step bench+car calibration procedure (as comments) for tuning each new brake-motor model.

**`PulseSensor`** (`Tachometer`/`Speedometer` base): hall-effect pulse timing via a FALLING-edge ISR with a debounce-reject threshold. Uses a **`scaling_ema_filt()`** whose effective alpha scales with actual elapsed time since the last call (`alpha = dt/(tau+dt)`), so filtering stays smooth across variable pulse rates, capped after long gaps. `stopped()` compares the filtered value to `opmin` within a margin.

- **`Tachometer`**: ÷8 external frequency divider, abs 0-4800rpm, op 0-3600rpm ("redline"), governed max = `opmax × governor/100`, idle 590rpm (680 cold, 500 hot), stop-timeout 1.0s (tuned past the ~960ms hot-idle pulse period). `Standby`-mode zero-speed confirmation is active for tach — its stopped-baseline check runs unconditionally, unlike speedo's (below).
- **`Speedometer`**: ×2 (two magnets/wheel rotation), 20-inch wheel diameter assumed in the mph conversion, abs 0-18mph, op 0-15mph, stop-timeout 1.3s. Its equivalent `Standby`-mode "stays at zero when parked" check exists in `SpeedoFailure()` but is **commented out** (reason unknown, per the code's own comment) — only the accelerates-under-throttle half of speedo's failure check is currently active, making it functionally asymmetric with tach's own equivalent check.

**`Simulator`**: tracks which sensors can be simulated (`can_sim` bitmask) and whether via full simulator mode or the single-sensor pot-map (only one sensor can be pot-mapped at a time); both persist to flash.

**`RMTInput`**: wraps the ESP32 RMT peripheral in RX mode to read RC PWM pulse widths. Comment notes an unexplained hardware/driver quirk: if the code ever stops reading an RMT object, it floods the console with RMT failures — worked around by always reading.

**`Hotrc`**: the 4-channel RC transmitter/receiver interface (Horz/Vert analog joystick, Ch3/Ch4 digital toggles). Per-hardware-generation calibration tables live in comments (multiple receiver-module revisions, each with its own measured µs values). A custom 9-deep-ring-buffer **`spike_filter()`** rejects/interpolates transient PWM glitches the HotRC hardware occasionally produces, at the cost of ~9-reading input latency. Radio-lost failsafe logic distinguishes `_radiolost` (currently in failsafe range) from `_radiolost_untested` (failsafe condition never yet observed since boot, so detection itself is unverified) — `require_radiolost_test` refuses ignition-on until the latter clears. `_rc_ever_powered` latches once any channel reads above 1200µs, to distinguish "transmitter genuinely off" from the 1000µs boot-default reading every channel starts at (the member's own declaration comment claims this also requires a verified failsafe, but the actual latch condition checks only the 1200µs threshold — the comment has drifted, not the behavior).

## Motors

**`QPID`**: a heavily modified QuickPID (itself based on Arduino `PID_v1`). Beyond stock PID: proportional-on-error/measurement/blended modes, derivative-on-error/measurement, several anti-windup modes (conditional/clamp/off/round/round-conditional), a **centmode** feature that zeroes the integral and snaps the output sum back to a defined center — `on` mode whenever error itself is near zero, `strict` mode *additionally* whenever error's sign flips versus the last reading (used by both brake PIDs, which run in `strict`, to stop integral windup carrying across a sign change) — and an output rate limiter applied inside `compute()`. A **second, independent** rate limiter also exists one layer up, on `ServoMotor` itself (`rate_limiter()`), used by `ThrottleControl` (non-PID mode), `BrakeControl::postprocess_out()` (explicitly to smooth "hard-stop transitions that bypass `compute()`'s internal limiter"), and `SteeringControl` — two distinct rate-limiting mechanisms at two different layers, not one. A comment flags a fixed historical bug: the conditional anti-windup math was previously missing the prospective P+I output's accumulated-integral term.

**`ServoMotor`** (base) → `ThrottleControl` directly; separately, **`ServoMotor` → `JagMotor`** (adds a bidirectional H-bridge-style model with a center `Stop` value; its `derive()` recomputes operating limits as a fraction of the *live measured battery voltage* — the abs range is literally `±mulebatt->val()`, except under `running_on_devboard`, where it substitutes a fixed `car_batt_fake_v` of 12.0V) → `BrakeControl`, `SteeringControl`. Throttle does **not** descend from `JagMotor` — it has no center-`Stop`/bidirectional concept, just `ServoMotor`'s plain linear mapping.

- **`ThrottleControl`**: PID gains `kp=0.13, ki=0.50, kd=0.00` (separate gain pairs for Cruise mode depending on whether gas is itself open-loop or PID). Op range 66.3-165° servo angle, parked at 55°. Optional (off by default) idle-boost raises the idle target as engine temp reads colder — 680rpm cold idle to 500rpm hot, boosted up to 15% over a 60-80°F window. Throttle response can be exponent-linearized (default exponent 3.75). Two cruise-adjustment schemes: continuous rate-based adjustment while the trigger's off-center, or a one-shot adjustment proportional to peak trigger deflection.
- **`BrakeControl`** (the most complex motor class): two independent PID loops — position (`kp=22.0, ki=5.5`) and pressure (`kp=0.54, ki=0.10`) — blended via a **hybrid feedback scheme**: a sigmoid transition between 25% and 50% of full pressure range shifts control authority smoothly from position-dominant (low pressure, where position sensing is accurate) to pressure-dominant (high pressure, where position barely changes further); `calc_hybrid_ratio_pc()` short-circuits when either sensor is absent, avoiding a `NaN × 0 = NaN` propagation. Motor-thermal protection disables the brake control mode above its operating temperature ceiling — a continuous `duty_pc` is also tracked from motor heat (`update_motorheat()`, exposed via `duty()`/`dutymin()`/`dutymax()`) but no code path in `motors.h` was found actually enforcing it against output, only computing/exposing it (possibly display-only; unconfirmed). Two structurally distinct automatic behaviors, not one shared state machine: **auto-stop** ramps pressure toward a full stop in increments (2.5%/tick normally, 4.0%/tick while panicking or already stopped, 250ms interval) against an 8s overall timeout that forces `ActionHalt` if reached; **auto-hold** has no timeout at all, holding indefinitely by continuously re-invoking the stop logic once its own 100% initial-pressure target is reached. A positional-limit check halts the motor rather than let it drive the actuator past its mechanical stops. A fixed historical bug is documented in **two** separate code paths here: the `releasing` flag used to never get updated, leaving it stale and showing "releasing" on-screen while the brake was actually being pressed — fixed once in the `simple_brake` branch and, separately, again in the full-PID (non-`simple_brake`) branch, which had the identical bug independently. The global `simple_brake` mode is broader than just an open-loop-mode restriction: it also disables `ActionAutoStop`/`ActionAutoHold` entirely and swaps `ActionManual`'s own logic for a simpler inline zeropoint-margin check with its own fixed-speed release path. Open-loop modes: median-point hold (disallowed under `simple_brake`), auto-release-on-trigger-release, and auto-release-holdable (default — holds anywhere between full release and full press based on trigger position).
- **`SteeringControl`**: simplest motor — open-loop only, no PID, no feedback sensor. Has a documented safety feature, **speed-dependent steering-authority reduction**: at max vehicle speed, the steering endpoint is attenuated by 25% (linear with speed), preventing full-lock steering input while moving fast.

## Inputs: buttons, encoder, touchscreen

**`MomentarySwitch`**: generic debounced button with short/long press detection (300ms threshold); double-reads the pin to reject a documented ~70ns invalid-low glitch window.

**`Encoder`**: quadrature rotary encoder, ISR-driven, with **two alternate decode algorithms** selected by a compile-time define — two different cheap encoder hardware variants are in use across dev/vehicle boards, each needing different edge-interpretation logic. Computes a live spin-rate-based acceleration factor (up to 25×) for fast UI scrolling; a critical section around the shared delta counter avoids an ISR lost-update race.

**`Touchscreen`**: wraps SensorLib's `TouchDrvFT6X36` (Wire-based capacitive touch, I2C `0x38`). Previously LovyanGFX's own independent I2C driver stack handled touch, fighting the main `Wire` stack for the same physical bus — now unified onto one `Wire` stack serialized by `I2C::_busmutex` (see [Program flow](#program-flow)). Two distinct timers are easy to conflate: `senseTimer` (6.5ms — how often the touch chip can even be re-polled) and the real debounce, `filterTimer` (**10ms** — touch/untouch events shorter than this are ignored, "needed for using through plastic box lid"). Implements tap/double-tap (400ms)/long-press (550ms)/swipe (50px min)/drag/held detection, and an acceleration-ramping edit-delta that doubles every 400ms. `process_ui()` decodes touches into a 6×5 grid of touch boxes mapped to specific UI actions (page/select/+/-/screensaver, sim toggles, sensor nudges) — effectively the whole touchscreen menu dispatch table.

## Display and UI

**TFT**: ILI9341, 320×240, SPI (40MHz write / 16MHz read clock). Touch is deliberately *not* configured through LovyanGFX (see `Touchscreen` above, to avoid the dual-I2C-stack bus-contention bug). A 256-entry RGB332→RGB565 lookup table and 4 hand-encoded 8bpp icon bitmaps (one directional arrow, reused rotated 0/90/180/270° for all 4 arrow directions; a 145×74 "mule chassis" image) live in `tftsetup.h`.

**Layout**: a single fixed-layout dashboard, double-buffered across two tasks — `draw_task` paints into an off-screen sprite, `push_task` diffs it against the last-drawn sprite and DMAs only the changed pixel runs (`diffpush()`). 8 fixed telemetry rows (HotRC Vert/Horz %, Speed, Tach, Brake Sensor %, Throttle %, Brake Motor %, Steering Motor %) with bargraphs, needles, and targets are always visible. A scrollable 14-page tuning/telemetry dataset (run state, HotRC, sensors, pulse sensors, PWMs, idle, motors, brake PID, gas PID, cruise PID, temps, sim, diagnostics, UI settings) — up to 15 named, unit-labeled rows each, wired directly to live variables via pointer/`tune()` edits, functioning as an in-field engineering console. Side menu: PAG/SEL/+/-/ANI; top menu: CAL/SIM/CH4/IGN. An idiot-light row shows **36 icons across 3 rows of 12** (see [Diagnostics and safety](#diagnostics-and-safety)). A small "app panel" viewport hosts one of three mini-apps at a time (`EZReadUI` console / `MuleChassisUI` static diagram / `ScreensaverUI` full-screen animation). Custom 13×7 bitmap glyphs (21 total, covering µs/%/Ω/°/mph/rpm/psi/atm/g/s/adc and others) cover unit strings too wide for the tiny font. A `Tuner` class mediates encoder/touch edits into live variable changes on whichever datapage row is selected.

**On-screen animations** (`animations.h`, TFT-only — distinct from both the NeoPixel strip and the carpet lights): `CollisionsSaver` (a bouncing-ball elastic-collision screensaver, up to 35 balls, slowly-rotating "meandering" gravity vector — its near-full-screen background repaint previously overwhelmed the draw/push tasks badly enough to trip the task watchdog and reboot the board; now relies on `draw_task`'s self-regulating backoff rather than a hard frame cap), `EraserSaver` (a pattern-drawing screensaver with 8 distinct pattern types — `Wedges/Dots/Rings/Ellipses/Boxes/Ascii/Worm/Rotate` — cycled between; a pot-based speed control, `pot_timing()`, exists but its only call site is currently commented out, making it dead code), `EZReadDrawer` (renders the console log into the app panel with its own scrollbar, snapshotting lines under mutex to avoid tearing), and `PanelAppManager` (arbitrates which mini-app owns the panel viewport; also draws touch buttons, in a 4×3 grid, for manually poking simulated sensor values in sim mode). Screensaver navigation, undocumented elsewhere: encoder rotation cycles between animations, a touch swipe left/right goes to the previous/next one, a double-tap exits back to Standby, and a boot-button short-press also cycles forward. An optional FPS counter overlay (`display_fps()`) can render over the fullscreen animation for perf debugging.

## I2C bus and the LightingBox link

Single shared `Wire` bus (SDA 8 / SCL 9), every transaction serialized through `I2C::_busmutex` across all tasks/cores. Devices: touch controller (`0x38`), **LightingBox** (`0x69`, the CarpetLightCode firmware — see its own README), AirVelo sensor (`0x28`), MAP sensor (`0x18`). `I2C::setup()` scans addresses 1-126 at boot and logs found vs. expected devices; `Wire.setTimeOut(25)` (25ms max SCL hold, vs. the 50ms default).

**LightingBox protocol**, quoted from the source comment (also documented from the receiving side in `CarpetLightCode/README.md`):

> 1st nibble of 1st byte contains 4-bit command/request code. The 2nd nibble and any additional bytes contain data, as required by the code.
> code: `0x0F` = status flags. F bits: 0=syspower, 1=panicstop, 2=warning, 3=alarm
> code: `0x1R` = entered runmode given by R (no additional bytes)
> code: `0x2H,0xLL` = speedometer value update. 12-bit value contained in HLL is speed in hundredths-of-mph

Of those 4 status bits, only **syspower and panicstop are actually live** — `sendstatus()` hardcodes both `warning` and `alarm` to `0`, since the `DiagRuntime* diag` pointer it would need to compute them from is commented out entirely in the `LightingBox` class. `LightingBox::update()` sends at most one message per tick, prioritized status > runmode > speed. Status and runmode only transmit on change; **speed always transmits every tick it's called**, doubling as the bus presence-detection heartbeat (a NACK on `Wire.endTransmission()` marks the device undetected). There's deliberately no separate periodic presence probe — an earlier attempt at one was reverted, since a NACK'd probe could trigger the ESP32 I2C driver's slow recovery path, stalling the whole bus for "the better part of a second" (this used to corrupt concurrent touch reads back when touch had its own independent I2C stack — fixed by the unification described in [Program flow](#program-flow)). That stall can no longer corrupt anything now that touch and Wire share one mutex-serialized driver stack, but it can still make any other i2c consumer waiting on `I2C::_busmutex` block for that same duration — and `Touchscreen::update()` takes that mutex every single `loop()` iteration. At the normal 250ms tick rate, a powered-off/disconnected lightbox was re-triggering that slow recovery path ~4x/sec, enough to make the whole main control loop visibly stutter or lock up. Fix: `LightingBox` now backs off to a 5-second retry interval whenever the last send wasn't ACK'd, snapping back to 250ms the moment a send succeeds again — same eventual-reconnect behavior, far fewer chances for the stall to recur.

Also on this bus: **`SparkFun_MicroPressure`** (MAP sensor) is a vendored, modified copy of the SparkFun library (patched for non-blocking reads, avoiding a 6-7ms block per read — see its 2-call state machine under [Sensors](#sensors)); **`FS3000`** (air velocity) is likewise vendored, with a fixed bug: `readRaw()` now treats a raw count of exactly `0` as the I2C-failure sentinel `0xFFFF`, since an all-zeros buffer was silently passing the original library's checksum.

## NeoPixel strip

10 physical LEDs on the vehicle PCB (devboard has 2 fewer — a known quirk), `NeoPixelBus<NeoGrbFeature, NeoSk6812Method>`. Layout: pixel 0 = onboard ESP/box backlight, cosine-pulses in the current runmode's color (dimmed and slowed in LowPower); pixel 1 = PCBA backlight, sine-pulses the same color, phase-offset from pixel 0; pixel 2 = external-strip mode indicator, normally mirrors pixel 1, except in LowPower where it joins pixels 3-9 in a Cylon/Knight-Rider sweep instead; pixels 3-9 mirror the **first 7** of the 12 on-screen row-1 idiot lights (sensor-lost, sensor-range, engine temp-alarm, brake-motor temp-alarm, wheel temp-alarm, radio-lost-tested, radio-lost-untested) — **not** the last 5 (panic-stop, brake auto-stopping/auto-holding, parking, releasing all stay screen-only), each mirrored light capable of solid-on, multi-color error flashing, or a distinct critical-alert strobe (3 pulses/sec for 1s, then 2s off).

Every pixel write funnels through one gamma-correction choke point (`neoSetPixelColor()`/`neoSetPixelColorDimmed()`) — PWM duty cycle is linear but perceived brightness isn't, so this keeps the whole strip visually consistent; the "Dimmed" variant gamma-corrects *before* the extra linear dim, specifically to avoid collapsing a smooth fade into the gamma table's near-zero flat region. LowPower mode's Cylon sweep (pixels 2-9) runs continuously, independently drifting hue at each end of the strip (8-minute cycle one end, 6-minute the other, interpolated between) plus an oscillating saturation (5-minute period, 79-100%) — re-randomized fresh on each genuine entry into LowPower, not each ~2.75s sweep restart.

**Temperature-alarm strobe** ties directly into the 3 mirrored temp-alarm pixels above: independent of the discrete `criticalAlertMode` pulse pattern, a continuously-variable white flash (60ms pulse) overlays pixels 2/3/4 (physical LEDs mapped to wheel/brake/engine) whenever that category is in alarm, at a frequency computed in `diag.h` (`eng_strobe_freq`/`brake_strobe_freq`/`wheel_strobe_freq`) that scales 1/3Hz → 8.3Hz as the reading climbs from alarm threshold to op max — see [Diagnostics and safety](#diagnostics-and-safety) for where that frequency is computed. A `flashdemo_ena()` test mode repurposes the 7 idiot-light pixels to demo **7** generic light-based value-encoding techniques (bitwise blink, duty-cycle, danger-strobe overlay, blackout-count, blink-rate urgency, dual-color alternating swap, color-temperature hue mapping) — a prototyping tool, not part of normal operation.

## Temperature monitoring

7 DS18B20 OneWire sensor locations: engine, ambient, 4× wheel (FL/FR/RL/RR), brake motor. Per-category °F limits (op min/max, alarm):

| Category | Op min | Op max | Alarm |
| --- | --- | --- | --- |
| Engine | 40 | 218 | 205 |
| Wheel (all 4) | 40 | 170 | 145 |
| Brake motor | 45 | 165 | 130 |
| Ambient | 40 | 135 | 120 |

(An internal `CatUnknown` category, for any sensor that never gets matched to a known location, silently mirrors the Ambient limits above.)

Each physical DS18B20 has a fixed factory 64-bit address; the firmware carries a table of known addresses per location, including **4 possible brake-motor addresses corresponding to 4 different physical actuator models** (Thomson, MotorFactoryStore, GoMotorWorld1/2) — whichever is actually detected sets `brakemotor_type_detected` and gets logged at boot, though no code path elsewhere in the firmware was found reading that variable back out to actually select a matching brake calibration, despite the name implying it should (worth a closer look if per-actuator-model calibration is expected to be live). `vehicle_detected` is inferred from whether the ambient sensor (glued to the control box, always present on the real vehicle) was found — the same signal `running_on_devboard` (`cantroller2.cpp`'s `running_on_devboard = !tempsens.setup()`) is derived from. Unrecognized sensor addresses auto-fill whichever known location slot is still empty. The whole task pauses entirely during `LowPower` (`while (runmode == LowPower) vTaskDelay(1000ms)`), otherwise runs a non-blocking conversion state machine, 11-bit resolution — the ~1s-per-tick figure describes the pacing between full 7-sensor rounds (bounded by the DS18B20's own ~375ms 11-bit conversion time via the vendored `DallasTemperature` library), not a fixed per-sensor read interval.

## Diagnostics and safety

**`LoopTimer`**: rolling 100-sample average + all-time-peak `loop()` iteration timing; warns above a 100ms max-allowable-loop-time.

**`DiagRuntime`** (`diag`): polled every 175ms, the central health-monitoring system. Registers **22** telemetry channels (`register_device()` calls in `setup()`: throttle, brake motor, steer motor, speedo, tach, brake pressure/position, HotRC's 4 channels, mule battery, AirVelo, MAP, pot, and the 7 temperature sensors) — each classified into `ErrLost` (not responding), `ErrRange` (outside op range ± margin), or `ErrWarn` (soft warning). `Ignition` and `Starter` have enum slots and `err_sens_card` entries but their `register_device()` calls are explicitly commented out (the referenced `register_bool_device()` helper doesn't even exist in this file), so neither is actually tracked. Dedicated per-loop checks include temperature failure (also computes the alarm-strobe frequencies described under [NeoPixel strip](#neopixel-strip), and enforces overtemp ignition/brake cutoffs), battery failure, speedo/tach failure (tach's own `Standby`-mode "stayed at zero" check is active; speedo's equivalent is commented out — see [Sensors](#sensors)), and HotRC failure. Brake cross-sensor consistency checks exist but are **currently disabled** — they caused a false "brake position lost" error immediately on boot and need cleanup before re-enabling (a documented, known limitation, not an oversight). A large in-source design-notes block lists dozens of not-yet-implemented failure heuristics (brake chain slip, actuator stall detection, air-filter clog detection via MAF/throttle ratio drift, and others) as a roadmap.

**`BootMonitor`** (`watchdog`): an NVS-backed crash/postmortem system, currently more load-bearing than the disabled ESP hardware watchdog (see [Program flow](#program-flow)). Persists a coarse "what were we doing" status (booting/parked/stopped/driving/panicking/etc.), runmode, panic-stop state, and ignition state to flash on every meaningful change, plus boot/crash counters and rounded uptime. On boot it prints a human-readable postmortem (e.g. "bootcount: N (crashed), last lost power while driving in Cruise mode, after 12 min uptime"). An optional (off by default) setting can force a panic state on boot if the last recorded state was mid-drive — gated in part by a `was_panicked` flag that's declared and read but never actually assigned `true` anywhere in `diag.h`, so that particular restart-after-crash log branch currently can't fire from what's visible in this file (it may be set elsewhere; unconfirmed).

**Idiot lights** (`idiots.h`): a 36-icon, 3-row × 12 on-screen warning panel. Row 1 (12 icons, the hazard-light superset) has its **first 7** mirrored onto the physical NeoPixels (see [NeoPixel strip](#neopixel-strip) for the exact list — not the last 5): sensor-lost, sensor-range, engine/brake/wheel temp-alarm, radio-lost tested/untested are on the strip; panic-stop, brake auto-stopping/auto-holding, parking, and releasing stay screen-only. Row 2: informational status (cruise-adjusting, car-hasn't-moved, starter-running, brake-PID-active, brake-no-feedback, speedo/tach activity + stopped states, touch-active, shutting-down, running-on-devboard). Row 3: one light per actuator/sensor error group from `diag.h` (throttle/brake/steer/HotRC/speedo/tach/pressure/position/temps/battery/GPIO/other).

**Safety interlocks summary** (this firmware drives real brakes/throttle/steering on a moving vehicle): ignition kill on radio loss, untested failsafe, or manual Ch3 toggle — via panic-stop braking rather than a plain power cut if the car is moving; the panic flag persists to flash so a post-crash reboot can detect it; the same guard exists in two places (`RunModeManager::update()` and, independently, `Ignition::update()` — see [Program flow](#program-flow)'s note on the open redundancy question); starter motor gated behind brake-applied-and-confirmed-holding, a recent-input-activity requirement, an optional double-click, and a 2-minute stall-mode timeout, against "phantom start" risk; a `check_for_external_tampering()` check compares the starter's actual pin level against its tracked state every loop and panics if they diverge; overtemp shutoffs for engine/wheel/brake; brake positional-limit enforcement against driving the actuator past its mechanical stops; steering authority reduction at speed; `unittests.h` explicitly flags `constrain()` and PID output-clamping checks as safety-critical (colored red, not orange, in its output).

**Undocumented debug instrumentation worth a look**: `BootButton::actions()`'s short-press handler (outside LowPower/screensaver) dumps the memory address of every telemetry-idiot array entry to the console, with a comment noting the developer was chasing a suspected address collision between `sensidiots[_MuleBatt]` and the global `nowtouch` bool. This is live, uncommented-out code, not a leftover no-op — worth confirming whether that suspected aliasing was ever actually resolved, given `sensidiots[]` feeds idiot-light/diagnostic state directly.

## Persisted settings

Via `Preferences prefs`/`myprefs` (NVS partition `"FlyByWire"`): `pref_drivemode`, `cansim`, `potmap`, `codestatus`, `runmode`, `panicstop`, `ignition`, `bootcount`, `crashcount`, `uptime`, `dpage`. Together these let the system resume its preferred drive mode, remember simulator config, and report a boot-time crash postmortem.

## Known limitations and future work

- **Wifi/OTA support has been removed**, not just disabled — `platformio.ini` comments both the `WifiSupported` and `ElegantOTA` build flags out with "removal of all radio features," and no networking code remains in `src/` (a `wifi_client_mode` bool is set but never read). Re-enabling OTA uploads would mean reinstating this from scratch, not flipping a flag.
- Brake cross-sensor consistency checks in `diag.h` are written but disabled (false-positive on boot; needs cleanup).
- `CoolingFan` is a defined but unused stub — no thermostat logic wired to `loop()` yet.
- SD card interface (integrated with the touchscreen module) is not yet implemented; intended for runtime data logging.
- `brakemotor_type_detected` is detected and logged but doesn't appear to be consumed anywhere to actually select a per-actuator-model brake calibration (see [Temperature monitoring](#temperature-monitoring)) — worth confirming whether this is intentional or a dangling feature.
- The duplicate radio-loss/panic safety-guard logic in both `RunModeManager::update()` and `Ignition::update()` (see [Program flow](#program-flow)) has an open, developer-authored question about whether it's redundant or could conflict — flagged, not resolved, in-source.
- `unittests.h` covers pure-logic/structural consistency only (compile-time asserts, math helpers, lookup-table completeness) — it does not exercise real hardware (I2C, PWM, encoder/touch) or run property/fuzz tests; both are flagged as future work in-source.
- `diag.h` carries an extensive in-source list of not-yet-implemented failure-detection heuristics (see [Diagnostics and safety](#diagnostics-and-safety)).

## Links

- [Flying Carpet Information Dumpster](https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA/edit#heading=h.uaks6l1vfqun)
- [Schematics, board layout, wiring diagrams](https://drive.google.com/drive/u/0/folders/1AAUnWQhdA940hJz0VnBCmHV5c0VyWric)
- [Bug list](https://docs.google.com/spreadsheets/d/1_FchfMjr4O9Q0fOcC0f2aJvmvfMjqayLLSsnorOU5c0/edit?gid=0#gid=0)
- SD card: 16GB, pre-formatted FAT32 (Windows, 32kB allocation unit) — macOS reads this as "MS-DOS (FAT32)". [SD card formatter](https://www.sdcard.org/downloads/formatter/) if reformatting.
- Image/color tooling: [free icon images](http://iconarchive.com/) (resize as needed, JPG with black background), [RGB565 converter](https://www.rinkydinkelectronics.com/t_imageconverter565.php), [RGB565 color picker](http://www.barth-dev.de/online/rgb565), [RGB332 color wheel](https://roger-random.github.io/RGB332_color_wheel_three.js), [named colors reference](https://wiki.tcl-lang.org/page/Colors+with+Names).
- Fonts: [font0 character map](https://learn.adafruit.com/assets/103682) (use the right-side map), [TomThumb font info](https://robey.lag.net/2010/01/23/tiny-monospace-font.html), [TomThumb character map](https://fontstruct.com/fontstructions/show/1656341/tom-thumb).
