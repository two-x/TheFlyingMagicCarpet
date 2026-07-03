#pragma once
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include "idiots.h"
#define striplength 10
#define idiot_light_led_offset 3  // Offset for idiot lights in the strip, after the heartbeat LEDs
#define runmode_lights_animation_duration_ms 6000
#define idiot_lights_animation_duration_ms 6500

// Declared outside of class because https://github.com/Makuna/NeoPixelBus/wiki/FAQ-%2311
NeoPixelBus<NeoGrbFeature, NeoSk6812Method> neoobj(striplength, neopixel_pin);  // NeoWs2812Method, NeoWs2812xMethod, NeoSk6812Method, NeoEsp32Rmt0Ws2812xMethod, NeoEsp32I2s1800KbpsMethod, NeoEsp32I2s1Sk6812Method,
NeoPixelAnimator neoanimator(3); // Channel 0 for runmode pulse, Channel 1 for idiot lights, Channel 2 for Cylon

static const RgbColor BLACK = RgbColor(0);
static const uint8_t idiot_light_colors[] = {0x63, 0xa3, 0xc2, 0xc0, 0xec, 0xf4, 0xd8};


class IdiotLight {
private:
    bool _verbose = false;
public:
    uint8_t led;
    bool solidOnMode = false;
    RgbColor solidColor;
    RgbColor flashColors[NumTelemetryIdiots];
    uint8_t flashColorCount = 0;
    bool criticalAlertMode = false;

    IdiotLight(uint8_t led, RgbColor solidColor)
        : led(led), solidColor(solidColor), flashColorCount(NumTelemetryIdiots) {
        for (uint8_t i = 0; i < NumTelemetryIdiots; i++) {
            flashColors[i] = BLACK;
        }
        if (_verbose) ezread.squintf("IdiotLight created on LED %d with base color %d\n", led, solidColor);
    }

    void setFlashColor(uint8_t index, RgbColor color) {
        if (index < NumTelemetryIdiots) {
            flashColors[index] = color;
        }
    }

    void resetFlashColors() {
        flashColorCount = 0;
        for (uint8_t i = 0; i < NumTelemetryIdiots; i++) {
            flashColors[i] = BLACK;
        }
    }

    bool hasFlashColors() {
        for (uint8_t i = 0; i < NumTelemetryIdiots; i++) {
            if (flashColors[i].R != 0 || flashColors[i].G != 0 || flashColors[i].B != 0) {
                return true;
            }
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
        
        if (nonBlackCount == 0) {
            return solidColor;  // No flash colors, return solid color
        }
        
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
// When placed inside the NeopixelStrip class, it's first member gets corrupted wtf so it lives out here for now.
IdiotLight idiotlights[idiot_light_led_count] = {
    IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)),
    IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)),
    IdiotLight(0, RgbColor(0))
};

class NeopixelStrip {
private:
    enum anim { AnimRunmode=0, AnimIdiots=1, AnimCylon=2, NumAnims=3 };  // so humans can tell which animation we're talking about
    RgbColor runmode_color = RgbColor(0, 0, 0);
    bool _verbose = false;
    float progressAnim[(int)NumAnims] = { 0.0f, 0.0f, 0.0f };

    RgbColor chg_pix_brightness(float new_brt) {
        return RgbColor(this->runmode_color.R * new_brt, this->runmode_color.G * new_brt, this->runmode_color.B * new_brt);
    }
    void startRunmodeAnimation() {
        // Start the runmode/backlight pulse animation
        neoanimator.StartAnimation(AnimRunmode, runmode_lights_animation_duration_ms, [this](const AnimationParam& param) {
            float lowpower_dimfactor = 0.15f;

            float wave_brightness = ((cos(param.progress * 2 * PI) + 1.0f) * 0.45f) + 0.1f;  // Cosine wave from 0.1 to 1
            if (runmode == LowPower) wave_brightness *= lowpower_dimfactor;  // Keep controlbox dimmer when car is unattended
            RgbColor pulse_color = chg_pix_brightness(wave_brightness);
            
            neoobj.SetPixelColor(0, pulse_color);  // Set the esp on-board & box backlight pixels to the cos wave (they are both pixel 0)

            wave_brightness = ((sin(param.progress * 2 * PI) + 1.0f) * 0.45f) + 0.1f;  // Sine wave from 0.1 to 1
            if (runmode == LowPower) wave_brightness *= lowpower_dimfactor;  // Keep controlbox dimmer when car is unattended
            pulse_color = chg_pix_brightness(wave_brightness);
            
            // Set the pcba backlight and external strip "mode" pixels (leftmost on the idiotlight strip) to the sin wave
            neoobj.SetPixelColor(1, pulse_color);  // Set pcba backlight pixels to the sin wave 
            if (runmode != LowPower) neoobj.SetPixelColor(2, pulse_color);  // in sleep mode this one does the cylon effect

            // !! This creates a bottomless recursion which will cause a stack overflow! Removing
            //
            // // Restart animation when it completes for continuous loop
            // if (param.progress >= 1.0f) {
            //     this->startRunmodeAnimation();  
            // }
            //
            // Instead allow the update function to access the animation progress and restart accordingly
            //   Same change is also made to the other 2 animations below
            this->progressAnim[AnimRunmode] = param.progress;
        });
    }

    void startIdiotLightsAnimation() {
        // Start the idiot lights animation that loops through each light
        neoanimator.StartAnimation(AnimIdiots, idiot_lights_animation_duration_ms, [this](const AnimationParam& param) {
            // Loop through idiot lights - for now just iterate, do nothing
            for (int i = 0; i < idiot_light_led_count; i++) {
                if (idiotlights[i].criticalAlertMode) {
                    // Panic mode: 3 pulses in 1 second, then 2 second break (total 3 second cycle)
                    float cycle_time = fmod(param.progress * (idiot_lights_animation_duration_ms / 1000.0f), 3.0f);
                    
                    if (cycle_time < 1.0f) {
                        // First second: 3 rapid pulses
                        float pulse_phase = fmod(cycle_time * 3.0f, 1.0f);  // 3 pulses per second
                        bool flash_on = pulse_phase < 0.5f;  // 50% duty cycle per pulse
                        neoobj.SetPixelColor(idiotlights[i].led, flash_on ? RgbColor(255, 255, 255) : idiotlights[i].solidColor);
                    } else {
                        // Next 2 seconds: break (off)
                        neoobj.SetPixelColor(idiotlights[i].led, idiotlights[i].solidColor);
                    }
                } else if (idiotlights[i].solidOnMode) {
                    if (idiotlights[i].hasFlashColors()) {
                        // Flash mode: cycle through non-black colors with timing
                        float time_seconds = param.progress * (idiot_lights_animation_duration_ms / 1000.0f);
                        RgbColor flash_color = idiotlights[i].getFlashColor(time_seconds);
                        neoobj.SetPixelColor(idiotlights[i].led, flash_color);
                    } else {
                        // Standard solid color mode
                        neoobj.SetPixelColor(idiotlights[i].led, idiotlights[i].solidColor);
                    }
                } else {
                    // If not in warning mode, turn off the LED
                    neoobj.SetPixelColor(idiotlights[i].led, BLACK);
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
        neoanimator.StartAnimation(AnimCylon, 2000, [this](const AnimationParam& param) {
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

            for (int i = 0; i < effect_length; i++) {
                float raw_d = (float)i - position;
                float d = moving_forward ? raw_d : -raw_d;  // d > 0: ahead, d < 0: trail
                float sigma = (d >= 0.0f) ? 0.65f : 1.5f;
                float brightness = expf(-(d * d) / (sigma * sigma));

                neoobj.SetPixelColor(i + effect_offset,
                    brightness > 0.004f ? RgbColor((uint8_t)(brightness * 255.0f), 0, 0) : BLACK);
            }
            this->progressAnim[AnimCylon] = param.progress;
        });
    }

    void clear() {
        for (int i = 0; i < striplength; i++) {
            neoobj.SetPixelColor(i, RgbColor(0, 0, 0));
        }
        neoobj.Show();
    }


    void setPixelColor(int index, RgbColor color) {
        if (index >= 0 && index < striplength) {
            neoobj.SetPixelColor(index, color);
        }
    }

public:
    NeopixelStrip() {}

    void setup() {
        neoobj.Begin();
        startRunmodeAnimation();
        startIdiotLightsAnimation();
        for (int i = 0; i < idiot_light_led_count; i++) {
            uint8_t led = i + idiot_light_led_offset;  // Set LED index for each idiot light
            idiotlights[i].led = led;
            idiotlights[i].solidColor = color_to_neo(idiot_light_colors[i]);
            if (_verbose) ezread.squintf("IdiotLight %d initialized with LED %d and color %d\n", i, idiotlights[i].led, idiotlights[i].solidColor);
            neoobj.SetPixelColor(idiotlights[i].led, idiotlights[i].solidColor);
        }
        neoobj.Show();

        // test_pattern();   // removing bootup test pattern

        // demo mode for testing:
        // idiotlights[0].warningMode = true;
        // idiotlights[1].panicMode = true;
        // idiotlights[4].warningMode = true;
        // idiotlights[4].warningFlashColor = RgbColor(255, 0, 0);

    }

    void test_pattern() {
        for (int color = 0; color < 3; color++) {
            for (int i = 0; i < striplength; i++) {
                switch(color) {
                    case 0: neoobj.SetPixelColor(i, RgbColor(255, 0, 0)); break;   // Red
                    case 1: neoobj.SetPixelColor(i, RgbColor(0, 255, 0)); break;   // Green
                    case 2: neoobj.SetPixelColor(i, RgbColor(0, 0, 255)); break;   // Blue
                }
            }
            neoobj.Show();
            delay(333);
        }

        for (int i = 0; i < striplength; i++) {
            neoobj.SetPixelColor(i, RgbColor(0, 0, 0));
        }
        neoobj.Show();
    }

    void setIdiotLightSolidOnMode(int idiot_index, bool on) {
        if (idiot_index < 0 || idiot_index >= idiot_light_led_count) return;
        idiotlights[idiot_index].solidOnMode = on;
        if (on == false) {
            setIdiotLightResetFlashColors(idiot_index);
        }
    }

    void setIdiotLightFlashColor(int idiot_index, uint8_t colorIndex, RgbColor color) {
        if (idiot_index >= 0 && idiot_index < idiot_light_led_count) {
            idiotlights[idiot_index].setFlashColor(colorIndex, color);
        }
    }

    void setIdiotLightResetFlashColors(int idiot_index) {
        if (idiot_index >= 0 && idiot_index < idiot_light_led_count) {
            idiotlights[idiot_index].resetFlashColors();
        }
    }

  void setIdiotLightCriticalAlertMode(int idiot_index, bool panic_mode) {
        idiotlights[idiot_index].criticalAlertMode = panic_mode;
    }

    ~NeopixelStrip() {
        // Don't delete neoobj since it points to global neoobj
    }

    void flashdemo_ena(bool ena) {
        flashdemo = ena;
        if (flashdemo) {
            idiotlights[0].criticalAlertMode = true;
            idiotlights[1].criticalAlertMode = true;
            idiotlights[4].criticalAlertMode = true;
            // idiotlights[4].warningFlashColor = RgbColor(255, 0, 0);
            // for (int i=1; i<=6; i++) setflash(i, 3, 1, 2, 100, 0xffffff); // three super-quick bright white flashes
            // setflash(4, 8, 8, 8, 20);            // brightness toggle in a continuous squarewave
            // setflash(5, 3, 1, 2, 100, 0xffffff); // three super-quick bright white flashes
            // setflash(6, 2, 5, 5, 0, 0);          // two short black pulses
        }
        else {                                   // cancel any current blink programs on these leds
            idiotlights[0].criticalAlertMode = false;
            idiotlights[1].criticalAlertMode = false;
            idiotlights[4].criticalAlertMode = false;
            // for (int i=1; i<=6; i++) setflash(i, 0);
            // setflash(4, 0);
            // setflash(5, 0);
            // setflash(6, 0);
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
                    startCylonAnimation();
                } else {
                    neoanimator.StopAnimation(AnimCylon);
                    startIdiotLightsAnimation();
                }
            }
            
            // restart animations when they finish so they cycle indefinitely
            // (this replaces the previous recursive approach where animations cycled themselves)
            //
            if (progressAnim[AnimRunmode] >= 1.0f) startRunmodeAnimation();
            if (runmode == LowPower) { if (progressAnim[AnimCylon] >= 1.0f) startCylonAnimation(); }
            else if (progressAnim[AnimIdiots] >= 1.0f) startIdiotLightsAnimation();

            neoanimator.UpdateAnimations();
            // Directly apply basic on/off state each tick so pixels 3-9 always
            // mirror solidOnMode without relying solely on the animation cycle.
            // (Cylon owns pixels 3-9 in LowPower; animation still owns flash/critical cases.)
            if (runmode != LowPower) {
                for (int i = 0; i < idiot_light_led_count; i++) {
                    if (!idiotlights[i].criticalAlertMode && !idiotlights[i].hasFlashColors()) {
                        neoobj.SetPixelColor(idiotlights[i].led,
                            idiotlights[i].solidOnMode ? idiotlights[i].solidColor : BLACK);
                    }
                }
            }
            neoobj.Show();
        }
    }
};