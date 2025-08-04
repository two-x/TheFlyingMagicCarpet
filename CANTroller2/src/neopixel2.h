#pragma once
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include "globals.h" 
#define neorgb_t RgbColor  // RgbwColor
// #define striplength 10
#define striplength 9 // wtf dan's dev board has 9 LEDs, not 10
// #define idiot_light_offset 3  // Offset for idiot lights in the strip, after the heartbeat LEDs
#define idiot_light_led_offset 2  // For Dan's dev board neopixel, the first two LEDs are the heartbeat LEDs, then the idiot lights start
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
        ezread.squintf("IdiotLight created on LED %d with base color %d\n", led, solidColor);
    }

    void setFlashColor(uint8_t index, RgbColor color) {
        if (index < NumTelemetryIdiots) {
            flashColors[index] = color;
        }
    }

    void resetFlashColors() {
        flashColorCount = 0;
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
        // 6 second cycle, mostly solid color with brief flashes
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
        
        // Timeline: 0-1000ms: solid, 1001-1350ms: flash1, 1351-2350ms: solid, 2351-2700ms: flash2, etc.
        // Each flash: 1s wait + 350ms flash = 1350ms per flash color
        
        for (uint8_t i = 0; i < nonBlackCount; i++) {
            float start_time = 1.0f + (i * 1.35f);  // 1s + i*1.35s
            float end_time = start_time + 0.35f;    // +350ms flash duration
            
            if (cycle_time >= start_time && cycle_time < end_time) {
                return flashColors[nonBlackIndices[i]];
            }
        }
        
        // Default: show solid color
        return solidColor;
    }

    // // Getters
    // uint8_t getLed() const { return led; }
    // bool isWarningMode() const { return warningMode; }
    // bool isPanicMode() const { return panicMode; }
    // RgbColor getWarningSolidColor() const { return warningSolidColor; }
    // RgbColor getWarningFlashColor() const { return warningFlashColor; }
    //
    // // Setters
    // void setWarningMode(bool mode) { warningMode = mode; }
    // void setPanicMode(bool mode) { panicMode = mode; }
    // void setWarningFlashColor(RgbColor color) { warningFlashColor = color; }
};

// Global idiot lights array
// When placed inside the NeopixelStrip2 class, it's first member gets corrupted wtf so it lives out here for now.
IdiotLight idiotlights[idiot_light_led_count] = {
    IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)),
    IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)), IdiotLight(0, RgbColor(0)),
    IdiotLight(0, RgbColor(0))
};

class NeopixelStrip2 {
private:
    int last_runmode = -1;
    RgbColor runmode_color = RgbColor(0, 0, 0);

    void startRunmodeAnimation() {
        // Start the runmode/backlight pulse animation
        neoanimator.StartAnimation(0, runmode_lights_animation_duration_ms, [this](const AnimationParam& param) {
            // Calculate sine wave brightness (0.0 to 1.0)
            float sine_brightness = (sin(param.progress * 2 * PI) + 1.0f) / 2.0f;  // Sine wave from 0 to 1
            
            // Apply brightness to target color
            RgbColor pulse_color = RgbColor(
                this->runmode_color.R * sine_brightness,
                this->runmode_color.G * sine_brightness,
                this->runmode_color.B * sine_brightness
            );
            
            // Set pixels 0-2 to the pulsed color
            for (int i = 0; i < 3; i++) {
                neoobj.SetPixelColor(i, pulse_color);
            }
            
            // Restart animation when it completes for continuous loop
            if (param.progress >= 1.0f) {
                this->startRunmodeAnimation();
            }
        });
    }

    void startIdiotLightsAnimation() {
        // Start the idiot lights animation that loops through each light
        neoanimator.StartAnimation(1, idiot_lights_animation_duration_ms, [this](const AnimationParam& param) {
            // Loop through idiot lights - for now just iterate, do nothing
            for (int i = 0; i < idiot_light_led_count; i++) {
                if (idiotlights[i].criticalAlertMode) {
                    // Panic mode: 3 pulses in 1 second, then 2 second break (total 3 second cycle)
                    float cycle_time = fmod(param.progress * (idiot_lights_animation_duration_ms / 1000.0f), 3.0f);
                    
                    if (cycle_time < 1.0f) {
                        // First second: 3 rapid pulses
                        float pulse_phase = fmod(cycle_time * 3.0f, 1.0f);  // 3 pulses per second
                        bool flash_on = pulse_phase < 0.5f;  // 50% duty cycle per pulse
                        neoobj.SetPixelColor(idiotlights[i].led, flash_on ? RgbColor(255, 255, 255) : BLACK);
                    } else {
                        // Next 2 seconds: break (off)
                        neoobj.SetPixelColor(idiotlights[i].led, BLACK);
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
            
            // Restart animation when it completes for continuous loop
            if (param.progress >= 1.0f) {
                this->startIdiotLightsAnimation();
            }
        });
    }

    void startCylonAnimation() {
        // Cylon animation: red LED bouncing back and forth with trailing effect
        neoanimator.StartAnimation(2, 2000, [this](const AnimationParam& param) {
            // Clear all LEDs first
            for (int i = 0; i < striplength; i++) {
                neoobj.SetPixelColor(i, BLACK);
            }
            
            // Calculate position (0 to striplength-1 and back)
            float cycle_progress = param.progress * 2.0f; // 0 to 2
            float position;
            
            if (cycle_progress <= 1.0f) {
                // Moving forward (0 to striplength-1)
                position = cycle_progress * (striplength - 1);
            } else {
                // Moving backward (striplength-1 to 0)
                position = (2.0f - cycle_progress) * (striplength - 1);
            }
            
            int main_pos = (int)position;
            
            // Main LED (brightest)
            if (main_pos >= 0 && main_pos < striplength) {
                neoobj.SetPixelColor(main_pos, RgbColor(255, 0, 0));
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
                
                if (trail_pos >= 0 && trail_pos < striplength) {
                    neoobj.SetPixelColor(trail_pos, RgbColor(trail_brightness, 0, 0));
                }
            }
            
            // Restart animation when it completes for continuous loop
            if (param.progress >= 1.0f) {
                this->startCylonAnimation();
            }
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
            ezread.squintf("IdiotLight %d initialized with LED %d and color %d\n", i, idiotlights[i].led, idiotlights[i].solidColor);
            neoobj.SetPixelColor(idiotlights[i].led, idiotlights[i].solidColor);
        }
        neoobj.Show();

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
    }

    void setIdiotLightFlashColor(int idiot_index, uint8_t colorIndex, RgbColor color) {
        if (idiot_index >= 0 && idiot_index < idiot_light_led_count) {
            idiotlights[idiot_index].setFlashColor(colorIndex, color);
        }
    }

  void setIdiotLightCriticalAlertMode(int idiot_index, bool panic_mode) {
        idiotlights[idiot_index].criticalAlertMode = panic_mode;
    }

    ~NeopixelStrip2() {
        // Don't delete neoobj since it points to global neoobj
    }

    void update() {
        static Timer update_timer{20000}; // 20000us = 20ms update at most every 20ms.
        if (update_timer.expireset()) {
            if (last_runmode == -1 || last_runmode != runmode) {
                ezread.squintf("neopixel2 detected runmode change: %d->%d\n", last_runmode, runmode);
                last_runmode = runmode;
                runmode_color = color_to_neo(colorcard[last_runmode]);

                if (runmode == LowPower) {
                    neoanimator.StopAnimation(0);
                    neoanimator.StopAnimation(1);
                    startCylonAnimation();
                } else {
                    neoanimator.StopAnimation(2);
                    startRunmodeAnimation();
                    startIdiotLightsAnimation();
                }
            }

            neoanimator.UpdateAnimations();
            neoobj.Show();
        }
    }
};
