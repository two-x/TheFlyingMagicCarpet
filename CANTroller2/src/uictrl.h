#pragma once
#include "utils.h"
#include "FunctionalInterrupt.h"
#include <NeoPixelBus.h>
#define colortype RgbColor  // CRGB
NeoPixelBus<NeoBgrFeature, NeoEsp32Rmt0Ws2812xMethod> neoobj(15, 48);  // NeoEsp32Rmt0Ws2812xMethod works! NeoWs2812xMethod NeoEsp32I2s1X8Sk6812Method  NeoEsp32I2s1X8Ws2812xMethod
// Run neos in a task example: https://github.com/Makuna/NeoPixelBus/wiki/ESP32-and-RTOS-Tasks

// Potentiometer does an analog read from a pin and maps it to a percent (0%-100%). We filter the value to keep it smooth.
class Potentiometer {
    protected:
        static constexpr float adc_min = 300; // TUNED 230603 - Used only in determining theconversion factor
        static constexpr float adc_max = 4095; // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
        static constexpr float _ema_alpha = 0.1;
        static constexpr float _percent_min = 0.0;
        static constexpr float _percent_max = 100.0;
        uint8_t _pin;
        float _val;
    public:
        Potentiometer(uint8_t arg_pin) : _pin(arg_pin) {}
        Potentiometer() = delete; // must have a pin defined
        void setup() {
            set_pin(_pin, INPUT);
        }
        void update() {
            float new_val = map(static_cast<float>(analogRead(_pin)), adc_min, adc_max, _percent_min, _percent_max);
            new_val = constrain(new_val, _percent_min, _percent_max); // the lower limit of the adc reading isn't steady (it will dip below zero) so constrain it back in range
            _val = ema_filt(new_val, _val, _ema_alpha);
        }
        template<typename VAL_T>
        VAL_T mapToRange(VAL_T min, VAL_T max) {
            return static_cast<VAL_T>(map(_val, _percent_min, _percent_max, static_cast<float>(min), static_cast<float>(max)));
        }
        float get() { return _val; }
        float min() { return _percent_min; }
        float max() { return _percent_max; }
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
            // Encoder - takes 10 us to read when no encoder activity
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
    uint8_t lobright = 1;
    uint8_t heartbright = 6;
    uint8_t hibright = 6;
    float desatlevel = 0.0;  // out of 10.0
    uint8_t neo_master_brightness = 0xff;
    float correction[3] = { 1.0, 0.9, 1.0 };  // Applied to brightness of rgb elements
    uint32_t neo_fade_timeout_us = 350000;
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
    static const uint8_t numpixels = 2 * idiotCount + 1;  // 15 pixels = heartbeat RGB + 7 onboard RGB + 7 external RGBW
    bool idiotBoolState[idiotCount];  // For simple boolean idiot light, LOW will set urgency=1, and HIGH sets urgency=3.
    uint32_t idiotUrgency[idiotCount];  // urgency is from 1 to 10 level of freakout (0=off, 1=dim, 2=medium, 3=bright, 4-6=flash/black, 7-9=flash/white, 10=strobe) 
    uint8_t idiotBrightness[idiotCount];  //
    colortype neostrip[numpixels];
    colortype heartbeatColor, heartbeatNow, heartbeatColor_last;
    colortype neolast[numpixels];
    colortype idiotNormalColor[idiotCount];  // 
    colortype idiotEffectColor[idiotCount];  // 
    colortype idiotNowColor[idiotCount];  // 
    //#ifdef use_neopixelbus
    //   NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> neoobj(15, 48);  // NeoEsp32I2s1X8Sk6812Method  NeoEsp32I2s1X8Ws2812xMethod
    //#else
    //#endif
  public:
    neopixelStrip() {}

    void init(uint8_t argpin, bool viewcontext=NITE) {
        pin = argpin;
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
        heartbright = hibright;  // (uint8_t)((float)hibright * 0.75);
        lobright = (hibright > 50) ? 3 : (hibright > 25) ? 2 : 1;
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
            heartbeatColor = color_16b_to_32b(runmode_color);
            if (neoHeartbeatTimer.expired()) {
                heartbeat_pulse = !heartbeat_pulse;
                if (++heartbeat_state >= arraysize(heartbeat_ekg_us)) heartbeat_state -= arraysize(heartbeat_ekg_us);
                neoHeartbeatTimer.set(heartbeat_ekg_us[heartbeat_state]);
                if (heartbeat_pulse) heartbeat_brightness = heartbright;
                else neoFadeTimer.reset();
            }
            else if (!heartbeat_pulse) {
                if (neoFadeTimer.expired()) heartbeat_brightness = brightlev[context][B_MIN];
                else heartbeat_brightness = (int8_t)(lobright + (float)(heartbright - lobright) * (1.0 - ((heartbeat_state == 1) ? 1.5 : 1.0) * (float)neoFadeTimer.elapsed() / (float)neo_fade_timeout_us));
                
            }
            if (heartbeatColor != heartbeatColor_last || heartbeat_brightness != neobright_last) {
                heartbeatNow = dimmer(heartbeatColor, heartbeat_brightness);  // heartbeatNow = dimmer(desaturate(heartbeatColor, desatlevel), heartbeat_brightness);
                neostrip[0] = heartbeatNow;  // neostrip->setPixelColor(0, heartbeatNow);
                neoobj.SetPixelColor(0, heartbeatNow);
                heartbeatColor_last = heartbeatColor;
                neobright_last = heartbeat_brightness;
            }
        }
    }
    void colorfade_update() {
        if (neoFadeTimer.expireset()) {
            neostrip[0] = colorwheel(++neo_wheelcounter);
            neoobj.SetPixelColor(0, colorwheel(++neo_wheelcounter));
        }
    }
    uint32_t neopixelsAvailable() {
        return idiotCount;
    }
    bool newIdiotLight(uint8_t idiot, uint16_t color565, bool startboolstate = 0) {
        idiotBoolState[idiot] = startboolstate;
        if (idiot > idiotCount-1) return false;
        idiotNormalColor[idiot] = color_16b_to_32b(color565);
        idiotEffectColor[idiot] = idiotNormalColor[idiot]; // desaturate(idiotNormalColor[idiot], desatlevel);
        setBoolState(idiot, idiotBoolState[idiot]);
        updateIdiot(idiot);
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
        idiotEffectColor[idiot] = idiotNormalColor[idiot];  // idiotEffectColor[idiot] = desaturate(idiotNormalColor[idiot], desatlevel);
        if (idiotUrgency[idiot] <= 0) {
            idiotNowColor[idiot] = RgbColor(0);  // Turn off the light
        }
        else if (idiotUrgency[idiot] == 1) idiotNowColor[idiot] = dimmer(idiotEffectColor[idiot], lobright);
        else if (idiotUrgency[idiot] == 2) idiotNowColor[idiot] = dimmer(idiotEffectColor[idiot], heartbright);
        else if (idiotUrgency[idiot] == 3) idiotNowColor[idiot] = dimmer(idiotEffectColor[idiot], hibright);
        else if (idiotUrgency[idiot] <= 6) {  // Flash alternating with black, at increasing frequency
            // todo : implement this effect
        }
        else if (idiotUrgency[idiot] <= 9) {  // Flash alternating with white, at increasing frequency
            // todo : implement this effect
        }
        else if (idiotUrgency[idiot] == 10) {  // Continuous white/color strobe 
            // todo : implement this effect
        }
        neostrip[1+idiot] = idiotNowColor[idiot];
        neoobj.SetPixelColor (1+idiot, idiotNowColor[idiot]);
    }
    void refresh() {
        int32_t numledstowrite = (heartbeatNow != neolast[0]);
        neolast[0] = heartbeatNow;
        neoobj.SetPixelColor(0, RgbColor(heartbeatNow));
        neostrip[0] = (heartbeatNow);
        for (int32_t idiot=0; idiot<idiotCount; idiot++) {
            if (idiotNowColor[idiot] != neolast[idiot+1]) {
                numledstowrite = idiot + idiotCount + 2;
                neolast[idiot + 1] = idiotNowColor[idiot];                
                neostrip[idiot + 1] = RgbColor(idiotNowColor[idiot]);
                neostrip[idiot + idiotCount + 1] = RgbColor(idiotNowColor[idiot]);  // RgbwColor
            }
        }
        if (numledstowrite) neoobj.Show(numledstowrite);
        // This ability to exclude pixels at the end of the strip that haven't changed from the data write is my whole point of using neopixelbus
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
        return RgbColor(rgb[0], rgb[1], rgb[2]);
    }
    colortype color_16b_to_32b(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to FastLED CRGB struct suitable for library
        return RgbColor((color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3);
    }
    uint16_t color_32b_to_16b(colortype color) {  // Convert 5-6-5 encoded 16-bit color value to FastLED CRGB struct suitable for library
        return ((static_cast<uint16_t>(color.R) & 0xf8) << 8) | ((static_cast<uint16_t>(color.G) & 0xfc) << 3) | (((static_cast<uint16_t>(color.B) & 0xf8) >> 3));
    }
    float maxelement(float r, float g, float b) {
        return (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
    }
    float minelement(float r, float g, float b) {
        return (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
    }
    colortype dimmer(colortype color, int8_t bright_percent) {  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
        float rgb[3] = { static_cast<float>(color.R), static_cast<float>(color.G), static_cast<float>(color.B) };
        float fbright = (float)bright_percent * 2.55 / maxelement(rgb[0], rgb[1], rgb[2]);  // max(color.r, color.g, color.b);  // 2.55 = 0xff / 100
        float sat = 1;  // 1 - desatlevel * desatlevel / 100.0;
        float c[3] = { correction[0] * sat, correction[1] * sat, correction[2] * sat };
        for (int32_t element=0; element<3; element++)
            rgb[element] *= fbright * c[element];
        return RgbColor(rgb[0], rgb[1], rgb[2]);  // return CRGB((float)(color.r * fbright), (float)(color.g * fbright), (float)(color.b * fbright));
    }
    colortype desaturate(colortype color, float desat_of_ten) {  // desat_percent=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
        int8_t desat_percent = desat_of_ten;  // * desat_of_ten / 2.0; // Makes this control exponential
        float rgb[3] = { static_cast<float>(color.R), static_cast<float>(color.G), static_cast<float>(color.B) };
        printf (" Desat: (%f) before: 0x%02x%02x%02x", desat_of_ten, rgb[0], rgb[1], rgb[2]);
        float dominant;
        if (desat_percent < 0) {
            desat_percent = -desat_percent;
            dominant = minelement(rgb[0], rgb[1], rgb[2]);  // max(color.r, color.g, color.b);
        }
        else dominant = maxelement(rgb[0], rgb[1], rgb[2]);  // max(color.r, color.g, color.b);
        for (int32_t element=0; element<3; element++)
            rgb[element] = (uint32_t)(rgb[element] + ((float)desat_percent * (dominant - (float)(rgb[element])) / 100.0));
        printf (" after: 0x%02x%02x%02x\n", rgb[0], rgb[1], rgb[2]);
        return RgbColor(rgb[0], rgb[1], rgb[2]);
    }
};