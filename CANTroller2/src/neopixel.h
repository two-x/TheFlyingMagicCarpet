#pragma once
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include "idiots.h"
#define striplength 10  // car has 10 physical pixels (this was already correct); devboard has fewer (8) - see idiot_light_led_count/offset below for the actual missing-2-lights bug
#define idiot_light_led_offset 3  // Offset for idiot lights in the strip, after the heartbeat LEDs
#define runmode_lights_animation_duration_ms 6000
#define idiot_lights_animation_duration_ms 6500

// Declared outside of class because https://github.com/Makuna/NeoPixelBus/wiki/FAQ-%2311
inline NeoPixelBus<NeoGrbFeature, NeoSk6812Method> neoobj(striplength, neopixel_pin);  // NeoWs2812Method, NeoWs2812xMethod, NeoSk6812Method, NeoEsp32Rmt0Ws2812xMethod, NeoEsp32I2s1800KbpsMethod, NeoEsp32I2s1Sk6812Method,
inline NeoPixelAnimator neoanimator(3); // Channel 0 for runmode pulse, Channel 1 for idiot lights, Channel 2 for Cylon

// NeoPixelBus's standard gamma lookup table (NeoGammaTableMethod is its precomputed sRGB-ish table, cheaper at runtime
// than NeoGammaEquationMethod's powf() call). Human brightness perception is nonlinear but PWM duty cycle - what the LED
// driver actually controls - is linear, so an uncorrected 50% RGB value looks much brighter than 50% perceived brightness.
// Every pixel write in this file goes through this single choke point so the whole strip is gamma-correct everywhere.
inline NeoGamma<NeoGammaTableMethod> neoGammaCorrection;
inline void neoSetPixelColor(int index, RgbColor color) {
    neoobj.SetPixelColor(index, neoGammaCorrection.Correct(color));
}

static const RgbColor BLACK = RgbColor(0);
static const uint8_t idiot_light_colors[] = {0x63, 0xa3, 0xc2, 0xc0, 0xec, 0xf4, 0xd8};

class NeoIdiot {
private:
    bool _verbose = false;
public:
    uint8_t led;
    bool solidOnMode = false;
    RgbColor solidColor;
    RgbColor flashColors[NumTelemetryIdiots];
    uint8_t flashColorCount = 0;
    bool criticalAlertMode = false;

    NeoIdiot(uint8_t led, RgbColor solidColor)
        : led(led), solidColor(solidColor), flashColorCount(NumTelemetryIdiots) {
        for (uint8_t i = 0; i < NumTelemetryIdiots; i++) flashColors[i] = BLACK;
        if (_verbose) ezread.squintf("NeoIdiot created on LED %d with base color %d\n", led, solidColor);
    }
    void setFlashColor(uint8_t index, RgbColor color) {
        if (index < NumTelemetryIdiots) flashColors[index] = color;
    }
    void resetFlashColors() {
        flashColorCount = 0;
        for (uint8_t i = 0; i < NumTelemetryIdiots; i++) flashColors[i] = BLACK;
    }
    bool hasFlashColors() {
        for (uint8_t i = 0; i < NumTelemetryIdiots; i++) {
            if (flashColors[i].R != 0 || flashColors[i].G != 0 || flashColors[i].B != 0) return true;
        }
        return false;
    }
    RgbColor getFlashColor(float time) {
        // 6.5 second cycle, each erroring sensor flashes once
        float cycle_time = fmod(time, 6.5f);
        
        // Count non-black colors and build array of indices
        uint8_t nonBlackIndices[NumTelemetryIdiots];
        uint8_t nonBlackCount = 0;
        for (uint8_t i = 0; i < NumTelemetryIdiots; i++) {
            if (flashColors[i] != BLACK) {
                nonBlackIndices[nonBlackCount] = i;
                nonBlackCount++;
            }
        }
        if (nonBlackCount == 0) return solidColor;  // No flash colors, return solid color
        
        // Divide cycle into N equal slots, one per erroring sensor — exactly one flash each
        float pos_ms = cycle_time * 1000.0f;
        float slot_ms = 6500.0f / nonBlackCount;
        uint8_t slot_index = (uint8_t)(pos_ms / slot_ms) % nonBlackCount;
        float slot_pos_ms = fmod(pos_ms, slot_ms);

        // Slot structure: 22.5ms black / 435.94ms color / 22.5ms black / solid for remainder
        if (slot_pos_ms < 22.5f) return BLACK;
        if (slot_pos_ms < 458.4375f) return flashColors[nonBlackIndices[slot_index]];
        if (slot_pos_ms < 480.9375f) return BLACK;
        return solidColor;
    }
};
// Global idiot lights array
// When placed inside the NeopixelStrip class, its first member gets corrupted due to a static-initialization-order
// issue with NeoPixelAnimator channel allocation, so it lives at file scope. TODO: investigate root cause.
inline NeoIdiot neoidiots[idiot_light_led_count] = {
    NeoIdiot(0, RgbColor(0)), NeoIdiot(0, RgbColor(0)), NeoIdiot(0, RgbColor(0)),
    NeoIdiot(0, RgbColor(0)), NeoIdiot(0, RgbColor(0)), NeoIdiot(0, RgbColor(0)),
    NeoIdiot(0, RgbColor(0))
};
class NeopixelStrip {
private:
    enum anim { AnimRunmode=0, AnimIdiots=1, AnimCylon=2, NumAnims=3 };  // so humans can tell which animation we're talking about
    RgbColor runmode_color = RgbColor(0, 0, 0);
    bool _verbose = false;
    float progressAnim[(int)NumAnims] = { 0.0f, 0.0f, 0.0f };
    uint16_t _cylonHueStart = 0;       // randomized fresh each time Cylon (LowPower/"Knight Rider") genuinely restarts - see runmode-change block below - left-half starting hue
    uint16_t _cylonHueStartRight = 0;  // same, but for the right half of the strip, which cycles the opposite direction at a different period
    int64_t _cylonHueStartTime = 0;    // esp_timer_get_time() at that same moment - t=0 reference for both the hue and saturation cycles
    float _cylonSatPhaseStart = 0.0f;  // randomized fresh (0..1) alongside _cylonHueStart - starting phase within the saturation cycle

    RgbColor chg_pix_brightness(float new_brt) {
        return RgbColor(this->runmode_color.R * new_brt, this->runmode_color.G * new_brt, this->runmode_color.B * new_brt);
    }
    void startRunmodeAnimation() {
        // Start the runmode/backlight pulse animation
        neoanimator.StartAnimation(AnimRunmode, runmode_lights_animation_duration_ms, [this](const AnimationParam& param) {
            float lowpower_dimfactor = 0.15f;

            float wave_brightness = ((cos(param.progress * 2 * PI) + 1.0f) * 0.45f) + 0.1f;  // Cosine wave from 0.1 to 1 — PI is Arduino-specific; use M_PI for portability
            if (runmode == LowPower) wave_brightness *= lowpower_dimfactor;  // Keep controlbox dimmer when car is unattended
            RgbColor pulse_color = chg_pix_brightness(wave_brightness);
            
            neoSetPixelColor(0, pulse_color);  // Set the esp on-board & box backlight pixels to the cos wave (they are both pixel 0)

            wave_brightness = ((sin(param.progress * 2 * PI) + 1.0f) * 0.45f) + 0.1f;  // Sine wave from 0.1 to 1
            if (runmode == LowPower) wave_brightness *= lowpower_dimfactor;  // Keep controlbox dimmer when car is unattended
            pulse_color = chg_pix_brightness(wave_brightness);
            
            // Set the pcba backlight and external strip "mode" pixels (leftmost on the idiotlight strip) to the sin wave
            neoSetPixelColor(1, pulse_color);  // Set pcba backlight pixels to the sin wave 
            if (runmode != LowPower) neoSetPixelColor(2, pulse_color);  // in sleep mode this one does the cylon effect

            // Restarting the animation from inside its own callback caused bottomless recursion (stack overflow).
            // The update() function checks param.progress and restarts animations externally instead.
            this->progressAnim[AnimRunmode] = param.progress;
        });
    }

    void startNeoIdiotsAnimation() {
        // Start the idiot lights animation that loops through each light
        neoanimator.StartAnimation(AnimIdiots, idiot_lights_animation_duration_ms, [this](const AnimationParam& param) {
            if (flashdemo) {
                // demo_val oscillates 0→1→0 once per animation cycle — the "value being encoded"
                float t = param.progress * (idiot_lights_animation_duration_ms / 1000.0f);
                float demo_val = (sinf(param.progress * 2.0f * PI) + 1.0f) * 0.5f;

                // Light 0 — 4-bit bitwise data: 300ms start gap, 4×500ms bit slots (bright=1, dim flicker=0), 500ms end gap
                {
                    uint8_t bits = (uint8_t)(demo_val * 15.0f);
                    float sub = fmodf(t, 2.8f);
                    RgbColor c = BLACK;
                    if (sub >= 0.3f && sub < 2.3f) {
                        int bit_idx = (int)((sub - 0.3f) / 0.5f);
                        bool bit_on = (bits >> (3 - bit_idx)) & 1;
                        float bit_phase = fmodf(sub - 0.3f, 0.5f);
                        if (bit_on) c = neoidiots[0].solidColor;
                        else if (bit_phase < 0.12f) c = RgbColor(8, 20, 50);  // dim flicker marks a 0-bit
                    }
                    neoSetPixelColor(neoidiots[0].led, c);
                }

                // Light 1 — Duty-cycle value: fixed 4 Hz, on-fraction = demo_val (0%=min, 100%=max)
                {
                    bool on = fmodf(t * 4.0f, 1.0f) < demo_val;
                    neoSetPixelColor(neoidiots[1].led, on ? neoidiots[1].solidColor : BLACK);
                }

                // Light 2 — Danger strobe overlay: amber base + brief white flashes; rate 0.5→8 Hz as danger grows
                {
                    float freq = 0.5f + demo_val * 7.5f;
                    bool strobe = fmodf(t * freq, 1.0f) < 0.06f;
                    neoSetPixelColor(neoidiots[2].led, strobe ? RgbColor(255, 255, 255) : neoidiots[2].solidColor);
                }

                // Light 3 — Blackout count 1-4: steady violet with N brief black dips per 2s cycle
                {
                    int n_dips = 1 + (int)(demo_val * 3.999f);
                    float sub = fmodf(t, 2.0f);
                    RgbColor c = neoidiots[3].solidColor;
                    for (int n = 0; n < n_dips; n++) {
                        float gap_start = 0.15f + n * 0.22f;
                        if (sub >= gap_start && sub < gap_start + 0.08f) { c = BLACK; break; }
                    }
                    neoSetPixelColor(neoidiots[3].led, c);
                }

                // Light 4 — Blink-rate urgency: 50% duty, rate scales 0.3 Hz (calm) → 6 Hz (critical)
                {
                    float freq = 0.3f + demo_val * 5.7f;
                    bool on = fmodf(t * freq, 1.0f) < 0.5f;
                    neoSetPixelColor(neoidiots[4].led, on ? neoidiots[4].solidColor : BLACK);
                }

                // Light 5 — Dual-color alternating: cyan/orange swap at 1→5 Hz; fast rate = high urgency/ambiguity
                {
                    float freq = 1.0f + demo_val * 4.0f;
                    bool first = fmodf(t * freq, 1.0f) < 0.5f;
                    neoSetPixelColor(neoidiots[5].led, first ? RgbColor(0, 200, 200) : RgbColor(255, 80, 0));
                }

                // Light 6 — Color-temperature hue shift: green→yellow→red encodes value proximity to limit
                {
                    uint8_t r, g;
                    if (demo_val < 0.5f) {
                        float f = demo_val * 2.0f;
                        r = (uint8_t)(f * 210.0f);
                        g = 200;
                    } else {
                        float f = (demo_val - 0.5f) * 2.0f;
                        r = (uint8_t)(210.0f + f * 30.0f);
                        g = (uint8_t)((1.0f - f) * 200.0f);
                    }
                    neoSetPixelColor(neoidiots[6].led, RgbColor(r, g, 0));
                }

                this->progressAnim[AnimIdiots] = param.progress;
                return;
            }
            // Normal idiot-lights operation below
            for (int i = 0; i < idiot_light_led_count; i++) {
                if (neoidiots[i].criticalAlertMode) {
                    // Panic mode: 3 pulses in 1 second, then 2 second break (total 3 second cycle)
                    float cycle_time = fmod(param.progress * (idiot_lights_animation_duration_ms / 1000.0f), 3.0f);
                    
                    if (cycle_time < 1.0f) {
                        // First second: 3 rapid pulses
                        float pulse_phase = fmod(cycle_time * 3.0f, 1.0f);  // 3 pulses per second
                        bool flash_on = pulse_phase < 0.5f;  // 50% duty cycle per pulse
                        neoSetPixelColor(neoidiots[i].led, flash_on ? RgbColor(255, 255, 255) : neoidiots[i].solidColor);
                    } else {
                        // Next 2 seconds: break (off)
                        neoSetPixelColor(neoidiots[i].led, neoidiots[i].solidColor);
                    }
                } else if (neoidiots[i].solidOnMode) {
                    if (neoidiots[i].hasFlashColors()) {
                        // Flash mode: cycle through non-black colors with timing
                        float time_seconds = param.progress * (idiot_lights_animation_duration_ms / 1000.0f);
                        RgbColor flash_color = neoidiots[i].getFlashColor(time_seconds);
                        neoSetPixelColor(neoidiots[i].led, flash_color);
                    } else {
                        // Standard solid color mode
                        neoSetPixelColor(neoidiots[i].led, neoidiots[i].solidColor);
                    }
                } else {
                    // If not in warning mode, turn off the LED
                    neoSetPixelColor(neoidiots[i].led, BLACK);
                }
            }
            this->progressAnim[AnimIdiots] = param.progress;
        });
    }
    void startCylonAnimation() {
        // Smooth Cylon: asymmetric Gaussian centered on floating-point position.
        // Both sides have zero slope at the peak (C1 continuous) so there is no
        // perceptible step as the dot crosses each integer LED position.
        // Narrow sigma ahead for tight sub-pixel interpolation; wide sigma behind for the tail.
        neoanimator.StartAnimation(AnimCylon, 2750, [this](const AnimationParam& param) {
            int effect_offset = 2;
            int effect_length = striplength - effect_offset;

            float cycle_progress = param.progress * 2.0f;
            float position;
            bool moving_forward;
            if (cycle_progress <= 1.0f) {
                position = cycle_progress * (effect_length - 1);
                moving_forward = true;
            } else {
                position = (2.0f - cycle_progress) * (effect_length - 1);
                moving_forward = false;
            }

            // Hue drifts continuously with wall-clock time (independent of this sweep's own ~2.75s restart cycle, so the
            // color doesn't jump/reset every sweep) - the left END of the strip cycles forward around the spectrum every
            // 8 minutes, the right END cycles the opposite direction every 6 minutes, starting from _cylonHueStart /
            // _cylonHueStartRight respectively (randomized fresh on genuine Cylon start, see the runmode-change block below).
            // Pixels in between are hue-interpolated (see hue_delta below) for a continuous fade with no seam at the middle.
            constexpr int64_t hue_cycle_us_left = 480000000LL;   // 8 minutes per full hue cycle, left half
            constexpr int64_t hue_cycle_us_right = 360000000LL; // 6 minutes per full hue cycle, right half, opposite direction
            int64_t elapsed_us = esp_timer_get_time() - _cylonHueStartTime;
            uint16_t hue_left = (uint16_t)((_cylonHueStart + (uint32_t)((elapsed_us % hue_cycle_us_left) * 65536LL / hue_cycle_us_left)) % 65536);
            uint32_t right_delta = (uint32_t)((elapsed_us % hue_cycle_us_right) * 65536LL / hue_cycle_us_right);
            uint16_t hue_right = (uint16_t)(((uint32_t)_cylonHueStartRight - right_delta) % 65536);
            // Shortest signed hue distance from left to right (wraps correctly since 65536 == 2^16): reinterpreting the
            // wrapped uint16_t difference as int16_t picks whichever direction around the color wheel is the short way,
            // so pixels between the two ends fade continuously with no visible seam.
            int16_t hue_delta = (int16_t)(uint16_t)(hue_right - hue_left);

            // Saturation oscillates smoothly (sine wave, so no jump at the wrap point) between 79% and 100% of full,
            // independent of the hue cycle above - one full oscillation every 5 minutes, starting from a random phase
            // (_cylonSatPhaseStart, randomized fresh on genuine Cylon start, see the runmode-change block below).
            constexpr int64_t sat_cycle_us = 300000000LL;  // 5 minutes per full saturation oscillation
            constexpr float sat_min = 0.79f;
            constexpr float sat_max = 1.0f;
            float sat_phase = _cylonSatPhaseStart + (float)(elapsed_us % sat_cycle_us) / (float)sat_cycle_us;  // 0..~2
            float sat_frac = sat_min + (sat_max - sat_min) * (0.5f + 0.5f * sinf(sat_phase * 6.283185307f));
            uint8_t cylon_sat = (uint8_t)(255 * sat_frac);

            for (int i = 0; i < effect_length; i++) {
                float raw_d = (float)i - position;
                float d = moving_forward ? raw_d : -raw_d;  // d > 0: ahead, d < 0: trail
                float sigma = (d >= 0.0f) ? 0.65f : 1.5f;
                float brightness = expf(-(d * d) / (sigma * sigma));

                float t = (effect_length > 1) ? (float)i / (float)(effect_length - 1) : 0.0f;
                int32_t hue_raw = (int32_t)hue_left + (int32_t)lroundf(hue_delta * t);
                uint16_t hue = (uint16_t)(((hue_raw % 65536) + 65536) % 65536);
                neoSetPixelColor(i + effect_offset,
                    brightness > 0.004f ? color_to_neo(hsv_to_rgb<uint32_t>(hue, cylon_sat, (uint8_t)(brightness * 255.0f))) : BLACK);
            }
            this->progressAnim[AnimCylon] = param.progress;
        });
    }
    void clear() {
        for (int i = 0; i < striplength; i++) {
            neoSetPixelColor(i, RgbColor(0, 0, 0));
        }
        neoobj.Show();
    }
    void setPixelColor(int index, RgbColor color) {
        if (index >= 0 && index < striplength) {
            neoSetPixelColor(index, color);
        }
    }

public:
    NeopixelStrip() {}

    void setup() {
        neoobj.Begin();
        startRunmodeAnimation();
        startNeoIdiotsAnimation();
        for (int i = 0; i < idiot_light_led_count; i++) {
            uint8_t led = i + idiot_light_led_offset;  // Set LED index for each idiot light
            neoidiots[i].led = led;
            neoidiots[i].solidColor = color_to_neo(idiot_light_colors[i]);
            if (_verbose) ezread.squintf("NeoIdiot %d initialized with LED %d and color %d\n", i, neoidiots[i].led, neoidiots[i].solidColor);
            neoSetPixelColor(neoidiots[i].led, neoidiots[i].solidColor);
        }
        neoobj.Show();
        // test_pattern();   // removing bootup test pattern
        // demo mode for testing:
        // neoidiots[0].warningMode = true;
        // neoidiots[1].panicMode = true;
        // neoidiots[4].warningMode = true;
        // neoidiots[4].warningFlashColor = RgbColor(255, 0, 0);
    }
    void test_pattern() {
        for (int color = 0; color < 3; color++) {
            for (int i = 0; i < striplength; i++) {
                switch(color) {
                    case 0: neoSetPixelColor(i, RgbColor(255, 0, 0)); break;   // Red
                    case 1: neoSetPixelColor(i, RgbColor(0, 255, 0)); break;   // Green
                    case 2: neoSetPixelColor(i, RgbColor(0, 0, 255)); break;   // Blue
                }
            }
            neoobj.Show();
            delay(333);
        }
        for (int i = 0; i < striplength; i++) {
            neoSetPixelColor(i, RgbColor(0, 0, 0));
        }
        neoobj.Show();
    }
    void setNeoIdiotSolidOnMode(int idiot_index, bool on) {
        if (idiot_index < 0 || idiot_index >= idiot_light_led_count) return;
        neoidiots[idiot_index].solidOnMode = on;
        if (on == false) {
            setNeoIdiotResetFlashColors(idiot_index);
        }
    }
    void setNeoIdiotFlashColor(int idiot_index, uint8_t colorIndex, RgbColor color) {
        if (idiot_index >= 0 && idiot_index < idiot_light_led_count) {
            neoidiots[idiot_index].setFlashColor(colorIndex, color);
        }
    }
    void setNeoIdiotResetFlashColors(int idiot_index) {
        if (idiot_index >= 0 && idiot_index < idiot_light_led_count) {
            neoidiots[idiot_index].resetFlashColors();
        }
    }
    void setNeoIdiotCriticalAlertMode(int idiot_index, bool panic_mode) {
        if (idiot_index < 0 || idiot_index >= idiot_light_led_count) return;
        neoidiots[idiot_index].criticalAlertMode = panic_mode;
    }

    ~NeopixelStrip() {}  // Don't delete neoobj since it points to global neoobj

    void flashdemo_ena(bool ena) {
        flashdemo = ena;
        if (flashdemo) {
            for (int i = 0; i < idiot_light_led_count; i++) {
                neoidiots[i].solidOnMode = true;
                neoidiots[i].criticalAlertMode = false;
                neoidiots[i].resetFlashColors();
            }
            // Each light gets a base color suited to its demo style
            neoidiots[0].solidColor = RgbColor(  0,  90, 255);  // blue   — bitwise 4-bit data
            neoidiots[1].solidColor = RgbColor(  0, 200,  40);  // green  — duty-cycle value
            neoidiots[2].solidColor = RgbColor(220, 100,   0);  // amber  — danger strobe overlay
            neoidiots[3].solidColor = RgbColor(160,   0, 200);  // violet — blackout count 1-4
            neoidiots[4].solidColor = RgbColor(240,  20,   0);  // red    — blink-rate urgency
            neoidiots[5].solidColor = RgbColor(  0, 200, 200);  // cyan   — dual-color choice/urgency
            neoidiots[6].solidColor = RgbColor(  0, 200,  40);  // green (shifts to red) — color-temp value
        } else {
            for (int i = 0; i < idiot_light_led_count; i++) {
                neoidiots[i].criticalAlertMode = false;
                neoidiots[i].solidColor = color_to_neo(idiot_light_colors[i]);  // restore original colors
            }
        }
    }
    void update() {
        static Timer update_timer{20000}; // 20000us = 20ms update is 50 fps, fast enough to look smooth i think
        static int last_runmode = -1;

        if (update_timer.expireset()) {
            if (last_runmode == -1 || last_runmode != runmode) {
                // commented next line b/c for some reason _verbose becomes true and it runs (even tho it's set false)  wtf!
                // if (_verbose) ezread.squintf("neopixel2 detected runmode change: %d->%d\n", last_runmode, runmode);
                
                runmode_color = color_to_neo(colorcard[runmode]);
                last_runmode = runmode;
                
                if (runmode == LowPower) {
                    neoanimator.StopAnimation(AnimIdiots);
                    _cylonHueStart = (uint16_t)rn(65536);  // fresh random starting hue each genuine entry into Cylon (not on every ~2.75s sweep restart)
                    _cylonHueStartRight = (uint16_t)rn(65536);  // independent random starting hue for the right half
                    _cylonSatPhaseStart = (float)rn(65536) / 65536.0f;  // fresh random starting saturation phase, same occasion
                    _cylonHueStartTime = esp_timer_get_time();
                    startCylonAnimation();
                } else {
                    neoanimator.StopAnimation(AnimCylon);
                    startNeoIdiotsAnimation();
                }
            }       
            // restart animations when they finish so they cycle indefinitely
            // (this replaces the previous recursive approach where animations cycled themselves)
            //
            if (progressAnim[AnimRunmode] >= 1.0f) startRunmodeAnimation();
            if (runmode == LowPower) { if (progressAnim[AnimCylon] >= 1.0f) startCylonAnimation(); }
            else if (progressAnim[AnimIdiots] >= 1.0f) startNeoIdiotsAnimation();

            neoanimator.UpdateAnimations();
            // Directly apply basic on/off state each tick so pixels 3-9 always
            // mirror solidOnMode without relying solely on the animation cycle.
            // (Cylon owns pixels 3-9 in LowPower; animation still owns flash/critical cases.)
            if (runmode != LowPower && !flashdemo) {
                for (int i = 0; i < idiot_light_led_count; i++) {
                    if (!neoidiots[i].criticalAlertMode && !neoidiots[i].hasFlashColors()) {
                        float sfreq = (i == 2) ? diag.eng_strobe_freq
                                    : (i == 3) ? diag.brake_strobe_freq
                                    : (i == 4) ? diag.wheel_strobe_freq : 0.0f;
                        if (sfreq > 0.0f) {
                            uint32_t phase_ms = millis() % (uint32_t)(1000.0f / sfreq);
                            RgbColor base = neoidiots[i].solidOnMode ? neoidiots[i].solidColor : BLACK;
                            neoSetPixelColor(neoidiots[i].led, (phase_ms < 60) ? RgbColor(255, 255, 255) : base);
                        } else {
                            neoSetPixelColor(neoidiots[i].led,
                                neoidiots[i].solidOnMode ? neoidiots[i].solidColor : BLACK);
                        }
                    }
                }
            }
            neoobj.Show();
        }
    }
};