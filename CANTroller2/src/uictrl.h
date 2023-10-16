#pragma once
#include "utils.h"
#include "FunctionalInterrupt.h"
#include <NeoPixelBus.h>
#define colortype RgbColor  // RgbwColor

// Default for esp32 is dma via I2S bus 1 at 800kHz using RMT. Don't know difference between "Ws2812", "Ws2812x", and "Sk6812"
NeoPixelBus<NeoGrbFeature, NeoSk6812Method> neoobj(8, 48);  // NeoWs2812Method, NeoWs2812xMethod, NeoSk6812Method, NeoEsp32Rmt0Ws2812xMethod, NeoEsp32I2s1800KbpsMethod, NeoEsp32I2s1Sk6812Method, 

// Run neos in a task example: https://github.com/Makuna/NeoPixelBus/wiki/ESP32-and-RTOS-Tasks

// Potentiometer does an analog read from a pin and maps it to a percent (0%-100%). We filter the value to keep it smooth.
class Potentiometer {
    protected:
        static constexpr float adc_min = 300; // TUNED 230603 - Used only in determining theconversion factor
        static constexpr float adc_max = 4095; // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
        static constexpr float _ema_alpha = 0.1;
        static constexpr float _pc_min = 0.0;
        static constexpr float _pc_max = 100.0;
        uint8_t _pin;
        float _val;
    public:
        Potentiometer(uint8_t arg_pin) : _pin(arg_pin) {}
        Potentiometer() = delete; // must have a pin defined
        void setup() {
            set_pin(_pin, INPUT);
        }
        void update() {
            float new_val = map(static_cast<float>(analogRead(_pin)), adc_min, adc_max, _pc_min, _pc_max);
            new_val = constrain(new_val, _pc_min, _pc_max); // the lower limit of the adc reading isn't steady (it will dip below zero) so constrain it back in range
            _val = ema_filt(new_val, _val, _ema_alpha);
        }
        template<typename VAL_T>
        VAL_T mapToRange(VAL_T min, VAL_T max) {
            return static_cast<VAL_T>(map(_val, _pc_min, _pc_max, static_cast<float>(min), static_cast<float>(max)));
        }
        float val() { return _val; }
        float min() { return _pc_min; }
        float max() { return _pc_max; }
};

class Encoder {
    private:
        enum _inputs { ENC_A, ENC_B };

        // class vars
        //  ---- tunable ----
        // TODO: these are all currently private const, if we ever actually want to tune them live we would need to change this
        static const uint32_t _spinrate_min_us = 2500;  // Will reject spins faster than this as an attempt to debounce behavior
        static const uint32_t _accel_thresh_us = 100000;  // Spins faster than this will be accelerated
        static const int32_t _accel_max = 50;  // Maximum acceleration factor
        static const uint32_t _longPressTime = 350000;

        // instance vars
        volatile uint32_t _spinrate_isr_us = 100000;  // Time elapsed between last two detents
        volatile bool _a_stable = true;  //  Stores the value of encoder A pin as read during B pin transition (where A is stable)
        volatile int32_t _bounce_danger = ENC_B;  // Which of the encoder A or B inputs is currently untrustworthy due to bouncing 
        volatile int32_t _delta = 0;  // Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 

        uint8_t _a_pin;
        uint8_t _b_pin;
        uint8_t _sw_pin;
        int32_t _state = 0;
        int32_t _sw_action = NONE;  // Flag for encoder handler to know an encoder switch action needs to be handled
        uint32_t _spinrate_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
        uint32_t _spinrate_last_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
        uint32_t _spinrate_old_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
        bool _sw = false;  // Remember whether switch is being pressed
        bool _timer_active = false;  // Flag to prevent re-handling long presses if the sw is just kept down
        bool _suppress_click = false;  // Flag to prevent a short click on switch release after successful long press
        Timer _spinspeedTimer;  // Used to figure out how fast we're spinning the knob.  OK to not be volatile?
        //  ---- tunable ----
        Timer _longPressTimer;  // Used to time long button presses

        void IRAM_ATTR _a_isr() {
            if (_bounce_danger != Encoder::ENC_A) {
                if (!_a_stable) {
                    _spinrate_isr_us = _spinspeedTimer.elapsed();
                    _spinspeedTimer.reset();
                    _delta += digitalRead(_b_pin) ? -1 : 1;
                }
                _bounce_danger = Encoder::ENC_A;
            }
        }

        void IRAM_ATTR _b_isr() {
            if (_bounce_danger != Encoder::ENC_B) {
                _a_stable = digitalRead(_a_pin);
                _bounce_danger = Encoder::ENC_B;
            }
        }

    public:
        enum sw_presses { NONE, SHORT, LONG };

        Encoder(uint8_t a, uint8_t b, uint8_t sw) : _a_pin(a), _b_pin(b), _sw_pin(sw), _longPressTimer(_longPressTime){}
        Encoder() = delete; // must be instantiated with pins
        
        void setLongPressTimer(uint32_t t){
            _longPressTimer.set(t);
        }
    
        void setup() {
            set_pin(_a_pin, INPUT_PULLUP);
            set_pin(_b_pin, INPUT_PULLUP);
            set_pin(_sw_pin, INPUT_PULLUP);  // The esp32 pullup is too weak. Use resistor
            attachInterrupt(digitalPinToInterrupt(_a_pin), [this]{ _a_isr(); }, CHANGE); \
            attachInterrupt(digitalPinToInterrupt(_b_pin), [this]{ _b_isr(); }, CHANGE);
        }
        void update() {
            // Read and interpret encoder switch activity. Encoder rotation is handled in interrupt routine
            // Encoder handler routines should act whenever encoder_sw_action is SHORT or LONG, setting it back to
            // NONE once handled. When handling press, if encoder_long_clicked is nonzero then press is a long press
            if (!read_pin(_sw_pin)) {  // if encoder sw is being pressed (switch is active low)
                if (!_sw) {  // if the press just occurred
                    _longPressTimer.reset();  // start a press timer
                    _timer_active = true;  // flag to indicate timing for a possible long press
                }
                else if (_timer_active && _longPressTimer.expired()) {  // If press time exceeds long press threshold
                    _sw_action = LONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
                    _timer_active = false;  // Keeps us from entering this logic again until after next sw release (to prevent repeated long presses)
                    _suppress_click = true;  // Prevents the switch release after a long press from causing a short press
                }
                _sw = true;  // Remember a press is in effect
            }
            else {  // if encoder sw is not being pressed
                if (_sw && !_suppress_click) _sw_action = SHORT;  // if the switch was just released, a short press occurred, which must be handled
                _timer_active = false;  // Allows detection of next long press event
                _sw = false;  // Remember press is not in effect
                _suppress_click = false;  // End click suppression
            }
        }

        uint32_t handleSwitchAction() {
            uint32_t ret = _sw_action;
            _sw_action = NONE;
            return ret;
        }

        uint32_t handleSelection() {
            uint32_t d = 0;
            if (_delta) {  // Now handle any new rotations
                if (_spinrate_isr_us >= _spinrate_min_us) {  // Reject clicks coming in too fast as bounces
                    _spinrate_old_us = _spinrate_last_us;  // Store last few spin times for filtering purposes ...
                    _spinrate_last_us = _spinrate_us;  // ...
                    _spinrate_us = constrain (_spinrate_isr_us, _spinrate_min_us, _accel_thresh_us);
                    d = constrain (_delta, -1, 1);  // Only change one at a time when selecting or turning pages
                }
                _delta = 0;  // Our responsibility to reset this flag after handling events
            }
            return d;
        }

        uint32_t handleTuning() {
            uint32_t d = 0;
            if (_delta) {  // Handle any new rotations
                if (_spinrate_isr_us >= _spinrate_min_us) {  // Reject clicks coming in too fast as bounces
                    _spinrate_old_us = _spinrate_last_us;  // Store last few spin times for filtering purposes ...
                    _spinrate_last_us = _spinrate_us;  // ...
                    _spinrate_us = constrain(_spinrate_isr_us, _spinrate_min_us, _accel_thresh_us);
                    int32_t _temp = (_spinrate_old_us > _spinrate_last_us) ? _spinrate_old_us : _spinrate_last_us;  // Find the slowest of the last 3 detents ...
                    if (_temp < _spinrate_us) _temp = _spinrate_us;
                    _temp = map (_temp, _spinrate_min_us, _accel_thresh_us, _accel_max, 1);  // if turning faster than 100ms/det, proportionally accelerate the effect of each detent by up to 50x. encoder_temp variable repurposed here to hold # of edits per detent turned
                    d = _delta * _temp;  // If a tunable value is being edited, turning the encoder changes the value
                }
                _delta = 0;  // Our responsibility to reset this flag after handling events
            }
            return d;
        }
};

class neopixelStrip {
  private:
    enum brightness_presets { B_OFF, B_MIN, B_LO, B_MED, B_HI, B_EXT, B_MAX };
    uint8_t neo_wheelcounter = 0;
    enum brightness_contexts { NITE, DAY };  // Indoors = NITE
    uint8_t brightlev[2][7] = { { 0, 1,  6, 10, 17, 30,  50 },     // [NITE] [B_OFF/B_MIN/B_LOW/B_MED/B_HIGH/B_EXT/B_MAX]
                                { 0, 2, 16, 30, 45, 65, 100 }, };  // [DAY] [B_OFF/B_MIN/B_LOW/B_MED/B_HIGH/B_EXT/B_MAX]
    bool context = NITE;
    bool neo_heartbeat_variable_brightness = false;  // If false then brightness control only affect idiotlights
    uint8_t lobright = 1;
    uint8_t heartbright = 6;
    uint8_t hibright = 6;
    uint8_t heartlobright = 1;
    float desatlevel = 0.0;  // out of 10.0
    uint8_t neo_master_brightness = 0xff;
    float correction[3] = { 1.0, 0.9, 1.0 };  // Applied to brightness of rgb elements
    uint32_t neo_fade_timeout_us = 380000;
    Timer neoFadeTimer, neoHeartbeatTimer;
    bool neo_heartbeat = false;
    uint8_t pin = -1;
    uint8_t heartbeat_brightness; // brightness during fadeouts
    uint32_t neobright_last;
    int32_t heartbeat_state = 0;
    int32_t heartbeat_level = 0;
    int64_t heartbeat_ekg_us[4] = {250000, 200000, 620000, 2000000};  // {187500, 125000, 562500, 1250000};
    int32_t heartbeat_pulse = 255;
    static const uint8_t idiotCount = 7;
    static const uint8_t numpixels = 1 + idiotCount;  //  + extIdiotCount;  // 15 pixels = heartbeat RGB + 7 onboard RGB + 7 external RGBW
    bool idiotBoolState[idiotCount];  // For simple boolean idiot light, LOW will set urgency=1, and HIGH sets urgency=3.
    uint32_t idiotUrgency[idiotCount];  // urgency is from 1 to 10 level of freakout (0=off, 1=dim, 2=medium, 3=bright, 4-6=flash/black, 7-9=flash/white, 10=strobe) 
    uint8_t idiotBrightness[idiotCount];  //  + extIdiotCount];  //
    colortype neostrip[numpixels];
    colortype heartbeatColor, heartbeatNow, heartbeatColor_last;
    colortype neolast[numpixels];
    colortype idiotNormalColor[idiotCount];  // 
    colortype idiotEffectColor[idiotCount];  // 
    colortype idiotNowColor[idiotCount];  // 
    Timer debugtimer;
  public:
    neopixelStrip() {}

    void init(uint8_t argpin, bool viewcontext=NITE) {
        pin = argpin;
        // neoobj.NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod>(numpixels, pin);  // <NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> works! NeoGrbwFeature NeoWs2812xMethod NeoEsp32I2s1X8Sk6812Method  NeoEsp32I2s1X8Ws2812xMethod
        context = viewcontext;
        std::cout << "Neo init: add LEDs.. ";
        neoobj.Begin();
        heartbeat_brightness = brightlev[context][B_LO];
        neoHeartbeatTimer.set(heartbeat_ekg_us[3]);
        neoFadeTimer.set((int64_t)neo_fade_timeout_us);
        std::cout << "refresh strip.. ";
        refresh();
        std::cout << std::endl;
    }
    void setbright(int8_t newlev) {  // a way to specify brightness levels
        hibright = newlev;
        lobright = (hibright > 50) ? 3 : (hibright > 25) ? 2 : 1;
        if (neo_heartbeat_variable_brightness) {
            heartbright = hibright;  // (uint8_t)((float)hibright * 0.75);
            heartlobright = lobright;
        }
        updateAll();
    }
    void setdesaturation(float newlev) {  // a way to specify nite or daytime brightness levels
        desatlevel = newlev;
        updateAll();
    }
    void heartbeat(bool onoroff) {
        neo_heartbeat = onoroff;  // Start heart beating
    }
    void heartbeat_update(uint16_t runmode_color) {
        if (neo_heartbeat) {
            heartbeatColor = color_16b_to_Rgb(runmode_color);
            if (neoHeartbeatTimer.expired()) {
                heartbeat_pulse = !heartbeat_pulse;
                if (++heartbeat_state >= arraysize(heartbeat_ekg_us)) heartbeat_state -= arraysize(heartbeat_ekg_us);
                neoHeartbeatTimer.set(heartbeat_ekg_us[heartbeat_state]);
                if (heartbeat_pulse) heartbeat_brightness = heartbright;
                else neoFadeTimer.reset();
            }
            else if (!heartbeat_pulse) {
                if (neoFadeTimer.expired()) heartbeat_brightness = brightlev[context][B_MIN];
                else heartbeat_brightness = (int8_t)(heartlobright + max(1, (float)(heartbright - heartlobright) * (1.0 - ((heartbeat_state == 1) ? 1.5 : 1.0) * (float)neoFadeTimer.elapsed() / (float)neo_fade_timeout_us)));
            }
            if (heartbeatColor != heartbeatColor_last || heartbeat_brightness != neobright_last) {
                heartbeatNow = dimmer(heartbeatColor, heartbeat_brightness);  // heartbeatNow = dimmer(desaturate(heartbeatColor, desatlevel), heartbeat_brightness);
                debugtimer.set(4000000);
                heartbeatColor_last = heartbeatColor;
                neobright_last = heartbeat_brightness;
            }
        }
    }
    void colorfade_update() {
        if (neoFadeTimer.expireset()) {
            heartbeatNow = colorwheel(++neo_wheelcounter);
            neoobj.SetPixelColor(0, heartbeatNow);
        }
    }
    uint32_t neopixelsAvailable() {
        return idiotCount;
    }
    bool newIdiotLight(uint8_t idiot, uint16_t color565, bool startboolstate = 0) {
        idiotBoolState[idiot] = startboolstate;
        if (idiot > idiotCount-1) return false;
        idiotNormalColor[idiot] = color_16b_to_Rgb(color565);
        idiotEffectColor[idiot] = idiotNormalColor[idiot]; // desaturate(idiotNormalColor[idiot], desatlevel);
        setBoolState(idiot, idiotBoolState[idiot]);
        updateIdiot(idiot);
        // uint16_t reconv = color_Rgb_to_16b(idiotNormalColor[idiot]);
        // printf ("idiot#%d: 565: %04x (%02x, %02x, %02x) 32b: %06x Rgb.R: %02x Rgb.G: %02x Rgb.B: %02x reconv16b: %04x (%02x, %02x, %02x)\n", idiot, color565, (color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3, idiotNormalColor[idiot], idiotNormalColor[idiot].R, idiotNormalColor[idiot].G, idiotNormalColor[idiot].B, reconv, (reconv & 0xf800) >> 8, (reconv & 0x7e0) >> 3, (reconv & 0x1f) << 3);
        return true;
    }
    void setBoolState(uint8_t idiot, bool state) {
        if (idiot < idiotCount) {
            idiotBoolState[idiot] = state;
            idiotUrgency[idiot] = (state) ? 3 : 1;
        }
    }
    void updateAll() {
        for (int32_t idiot=0; idiot<idiotCount; idiot++) updateIdiot(idiot);
    }
    void updateIdiot(uint8_t idiot) {
        // idiotEffectColor[idiot] = idiotNormalColor[idiot];  // idiotEffectColor[idiot] = desaturate(idiotNormalColor[idiot], desatlevel);
        if (idiotUrgency[idiot] <= 0) {
            idiotNowColor[idiot] = colortype(0);  // Turn off the light
        }
        else if (idiotUrgency[idiot] == 1) idiotNowColor[idiot] = dimmer(idiotNormalColor[idiot], lobright);
        else if (idiotUrgency[idiot] == 2) idiotNowColor[idiot] = dimmer(idiotNormalColor[idiot], heartbright);
        else if (idiotUrgency[idiot] == 3) idiotNowColor[idiot] = dimmer(idiotNormalColor[idiot], hibright);
        else if (idiotUrgency[idiot] <= 6) {  // Flash alternating with black, at increasing frequency
            // todo : implement this effect
        }
        else if (idiotUrgency[idiot] <= 9) {  // Flash alternating with white, at increasing frequency
            // todo : implement this effect
        }
        else if (idiotUrgency[idiot] == 10) {  // Continuous white/color strobe 
            // todo : implement this effect
        }
        idiotNowColor[idiot] = desaturate(idiotNowColor[idiot], desatlevel);
        // neostrip[1+idiot] = idiotNowColor[idiot];
    }
    void refresh() {
        int32_t numledstowrite = (heartbeatNow != neostrip[0]);
        neostrip[0] = heartbeatNow;
        neoobj.SetPixelColor(0, heartbeatNow);
        for (int32_t idiot=0; idiot<idiotCount; idiot++) {
            if (idiotNowColor[idiot] != neostrip[idiot+1]) {
                neoobj.SetPixelColor (1+idiot, idiotNowColor[idiot]);
                neostrip[idiot + 1] = idiotNowColor[idiot];  // colortype(idiotNowColor[idiot]);
                numledstowrite = 2 + idiot;  // + idiotCount;
            }
        }
        if (numledstowrite) neoobj.Show(numledstowrite);  // This ability to exclude pixels at the end of the strip that haven't changed from the data write is an advantage of neopixelbus over adafruit
    }
  private:
    colortype colorwheel(uint8_t WheelPos) {
        WheelPos = 255 - WheelPos;
        int rgb[3];
        if (WheelPos < 85) {
            rgb[0] = 255 - WheelPos * 3; rgb[1] = 0; rgb[2] = WheelPos * 3;
        }
        else if (WheelPos < 170) {
            WheelPos -= 85;
            rgb[0] = 0; rgb[1] = WheelPos * 3; rgb[2] = 255 - WheelPos * 3;
        }
        else {
            WheelPos -= 170;
            rgb[0] = WheelPos * 3; rgb[1] = 255 - WheelPos * 3; rgb[2] = 0;
        }
        return colortype(rgb[0], rgb[1], rgb[2]);
    }
    struct hsv { float h; float s; float v; };
    // uint32_t rgb_to_hsv(uint32_t rgb) { return rgb_to_hsv((uint8_t)(rgb >> 16) | (uint8_t)((rgb & 0xff00 >> 8)) | (rgb & 0xff)); }
    // hsv rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b) {
    //     // convert RGB values to the range 0-1
    //     float ro = (float)r / 255.0;
    //     float go = (float)g / 255.0;
    //     float bo = (float)b / 255.0;
    //     float cmax = maxelement(ro, go, bo);
    //     float cmin = minelement(ro, go, bo);
    //     float delta = cmax - cmin;
    //     float h = 0.0;  // hue
    //     if (delta == 0.0) h = 0.0; // undefined, but typically set to 0
    //     else if (cmax == ro) h = 60 * fmod(((go - bo) / delta), 6);
    //     else if (cmax == go) h = 60 * (((bo - ro) / delta) + 2);
    //     else if (cmax == bo) h = 60 * (((ro - go) / delta) + 4);
    //     float s = (cmax == 0.0) ? 0.0 : delta / cmax;  // saturation
    //     float v = cmax;  // brightness
    //     return { h, s, v };
    // }
    uint32_t color_Rgb_to_32b(colortype color) {  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
        return (static_cast<uint32_t>(color.R) << 16) | (static_cast<uint32_t>(color.G) << 8) | static_cast<uint32_t>(color.B);  // (static_cast<uint32_t>(color.W) << 24) | 
    }
    uint32_t color_16b_to_32b(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
        return ((uint32_t)color565 & 0xf800) << 8 | ((uint32_t)color565 & 0x7e0) << 5 | ((uint32_t)color565 & 0x1f) << 3;
    }
    uint16_t color_32b_to_16b(uint32_t color) {  // Convert library color type to 5-6-5 encoded 16-bit color value
        return (int16_t)(((color & 0xf80000) >> 8) | ((color & 0xfc00) >> 5) | (color & 0xf8) >> 3);
    }
    colortype color_16b_to_Rgb(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
        return colortype((color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3);  // , 0);
    }
    uint16_t color_Rgb_to_16b(colortype color) {  // Convert library color type to 5-6-5 encoded 16-bit color value
        return ((static_cast<uint16_t>(color.R) & 0xf8) << 8) | ((static_cast<uint16_t>(color.G) & 0xfc) << 3) | (((static_cast<uint16_t>(color.B) & 0xf8) >> 3));
    }
    float maxelement(float r, float g, float b) {
        return (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
    }
    float midelement(float r, float g, float b) {
        return (r >= g) ? ((g >= b) ? g : ((r >= b) ? b : r)) : ((r >= b) ? r : ((b >= g) ? g : b));  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
    }
    float minelement(float r, float g, float b) {
        return (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
    }
    colortype dimmer(colortype color, int8_t bright_pc) {  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
        float rgb[3] = { static_cast<float>(color.R), static_cast<float>(color.G), static_cast<float>(color.B) };
        float fbright = (float)bright_pc * 2.55 / maxelement(rgb[0], rgb[1], rgb[2]);  // max(color.r, color.g, color.b);  // 2.55 = 0xff / 100
        float sat = 1;  // 1 - desatlevel * desatlevel / 100.0;
        float c[3] = { correction[0] * sat, correction[1] * sat, correction[2] * sat };
        for (int32_t element=0; element<3; element++)
            rgb[element] *= fbright * c[element];
        return colortype(rgb[0], rgb[1], rgb[2]);  // return CRGB((float)(color.r * fbright), (float)(color.g * fbright), (float)(color.b * fbright));
    }
    colortype desaturate(colortype color, int32_t desat_of_ten) {  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
        uint8_t rgb[3] = { static_cast<uint8_t>(color.R), static_cast<uint8_t>(color.G), static_cast<uint8_t>(color.B) };
        float dominant = maxelement(rgb[0], rgb[1], rgb[2]);
        if (desat_of_ten > 0) {
            for (int element=0; element<3; element++)
                rgb[element] = (uint8_t)(rgb[element] + ((float)desat_of_ten * (dominant - (float)(rgb[element])) / 20.0));
        }
        // else {
        //     // float mide = midelement(rgb[0], rgb[1], rgb[2]);
        //     dominant = minelement(rgb[0], rgb[1], rgb[2]);
        //     for (int element=0; element<3; element++)
        //         rgb[element] = (uint8_t)(rgb[element] - ((float)desat_of_ten * dominant / 10.0));
        // }
        return colortype(rgb[0], rgb[1], rgb[2]);
    }
};