#pragma once
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include "globals.h" 
#define neorgb_t RgbColor  // RgbwColor
#define striplength 10
#define idiot_light_led_offset 3  // Offset for idiot lights in the strip, after the heartbeat LEDs
#define idiot_light_led_count 7  // Number of idiot lights after the heartbeat leds
#define runmode_lights_animation_duration_ms 6000
#define idiot_lights_animation_duration_ms 6000

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
        // 6 second cycle with alternating pattern every 350ms
        float cycle_time = fmod(time, 6.0f);  // 6 second cycle
        
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
        
        // Pattern: t=0ms flash1, t=350ms solid, t=750ms flash2, t=1100ms solid, etc.
        // Each segment is 350ms, alternating between flash colors and solid
        uint32_t segment = (uint32_t)(cycle_time * 1000) / 350;  // Which 350ms segment (0-16 in 6s)
        
        if (segment % 2 == 0) {
            // Even segments (0, 2, 4, ...): show flash colors
            uint8_t flash_index = (segment / 2) % nonBlackCount;
            return flashColors[nonBlackIndices[flash_index]];
        } else {
            // Odd segments (1, 3, 5, ...): show solid color
            return solidColor;
        }
    }
};

// Global idiot lights array
// When placed inside the NeopixelStrip2 class, it's first member gets corrupted wtf so it lives out here for now.
IdiotLight idiotlights[idiot_light_led_count] = {
    IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)),
    IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)),
    IdiotLight(0, RgbColor(0))
};

#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>

class NeopixelStrip2 {
private:
    enum anim { AnimRunmode=0, AnimIdiots=1, AnimCylon=2, NumAnims=3 };  // so humans can tell which animation we're talking about
    RgbColor runmode_color = RgbColor(0, 0, 0);
    bool _verbose = false;
    float progressAnim[NumAnims] = { 0.0f, 0.0f, 0.0f };

    RgbColor chg_pix_brightness(float new_brt) {
        return RgbColor(this->runmode_color.R * new_brt, this->runmode_color.G * new_brt, this->runmode_color.B * new_brt);
    }
    void startRunmodeAnimation() {
        // Start the runmode/backlight pulse animation
        neoanimator.StartAnimation(AnimRunmode, runmode_lights_animation_duration_ms, [this](const AnimationParam& param) {
            float lowpower_dimfactor = 0.3f;

            float wave_brightness = (cos(param.progress * 2 * PI) + 1.1f) / 2.2f;  // Cosine wave from 0.1 to 1
            if (runmode == LowPower) wave_brightness *= lowpower_dimfactor;  // Keep controlbox dimmer when car is unattended
            RgbColor pulse_color = chg_pix_brightness(wave_brightness);
            
            neoobj.SetPixelColor(0, pulse_color);  // Set the esp on-board & box backlight pixels to the cos wave (they are both pixel 0)

            wave_brightness = (sin(param.progress * 2 * PI) + 1.1f) / 2.2f;  // Sine wave from 0.1 to 1
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
        // Cylon animation: red LED bouncing back and forth with trailing effect
        neoanimator.StartAnimation(AnimCylon, 2000, [this](const AnimationParam& param) {
            int effect_offset = 2;  // cylon should not extend to the pixels in the control box
            int effect_length = striplength - effect_offset;  // adjust the effective striplength accordingly

            // Clear all LEDs first
            for (int i = 0; i < effect_length; i++) {
                neoobj.SetPixelColor(i + effect_offset, BLACK);
            }
            
            // Calculate position (0 to striplength-1 and back)
            float cycle_progress = param.progress * 2.0f; // 0 to 2
            float position;
            
            if (cycle_progress <= 1.0f) {
                // Moving forward (0 to striplength-1)
                position = cycle_progress * (effect_length - 1);
            } else {
                // Moving backward (striplength-1 to 0)
                position = (2.0f - cycle_progress) * (effect_length - 1);
            }
            
            int main_pos = (int)position;
            
            // Main LED (brightest)
            if (main_pos >= 0 && main_pos < effect_length) {
                neoobj.SetPixelColor(main_pos + effect_offset, RgbColor(255, 0, 0));
            }
            
            // Trailing effect - dimmer LEDs behind the main one
            for (int trail = 1; trail <= 3; trail++) {
                int trail_brightness = 255 / (trail + 1);  // Diminishing brightness
                
                // Trail in direction opposite to movement
                int trail_pos;
                if (cycle_progress <= 1.0f) {
                    // Moving forward, trail behind
                    trail_pos = main_pos - trail;
                } else {
                    // Moving backward, trail behind
                    trail_pos = main_pos + trail;
                }
                
                if (trail_pos >= 0 && trail_pos < effect_length) {
                    neoobj.SetPixelColor(trail_pos + effect_offset, RgbColor(trail_brightness, 0, 0));
                }
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
    NeopixelStrip2() {}

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

    ~NeopixelStrip2() {
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
            neoobj.Show();
        }
    }
};
