#pragma once
#include "utils.h"
#include "FunctionalInterrupt.h"
#include <Adafruit_NeoPixel.h> // Plan to allow control of neopixel LED onboard the esp32

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
    enum brightness_contexts { NITE, DAY };  // Indoors = NITE
    uint8_t neo_wheelcounter = 0;
    uint8_t brightlev[2][7] = { { 0, 1,  6, 10, 18, 30,  50 },     // [NITE] [B_OFF/B_MIN/B_LOW/B_MED/B_HIGH/B_EXT/B_MAX]
                                { 0, 2, 14, 25, 40, 60, 100 }, };  // [DAY] [B_OFF/B_MIN/B_LOW/B_MED/B_HIGH/B_EXT/B_MAX]
    uint8_t neo_master_brightness = 255;
    uint32_t neo_fade_timeout_us = 300000;
    Timer neoFadeTimer, neoHeartbeatTimer;
    bool neo_heartbeat = false;
    int32_t pin = -1;
    uint8_t heartbeat_brightness; // brightness during fadeouts
    uint32_t heartbeatColor, heartbeatNow, heartbeatColor_last, neobright_last;
    bool context = NITE;
    int32_t heartbeat_state = 0;
    int32_t heartbeat_level = 0;
    int64_t heartbeat_ekg_us[4] = {250000, 175000, 550000, 1750000};  // {187500, 125000, 562500, 1250000};
    int32_t heartbeat_pulse = 255;
    static const uint32_t pixelCount = 8;
    static const uint32_t idiotCount = pixelCount - 1;
    uint32_t color_last[pixelCount];
    bool idiotBoolState[idiotCount];  // For simple boolean idiot light, LOW will set urgency=1, and HIGH sets urgency=3.
    uint32_t idiotNormalColor[idiotCount];  // 
    uint32_t idiotNowColor[idiotCount];  // 
    uint32_t idiotUrgency[idiotCount];  // urgency is from 1 to 10 level of freakout (0=off, 1=dim, 2=medium, 3=bright, 4-6=flash/black, 7-9=flash/white, 10=strobe) 
    uint8_t idiotBrightness[idiotCount];  //
    Adafruit_NeoPixel* neostrip;
  public:
    neopixelStrip() {}

    void init(int32_t argpin, bool viewcontext=NITE) {
        pin = argpin;
        context = viewcontext;
        neostrip = new Adafruit_NeoPixel(pixelCount, pin, NEO_GRB + NEO_GRB + NEO_KHZ800);
        neostrip->begin();  // start datastream
        neostrip->setBrightness (neo_master_brightness);  // Truly these can get incredibly bright
        heartbeat_brightness = brightlev[context][B_MED];
        neoHeartbeatTimer.set(heartbeat_ekg_us[3]);
        neoFadeTimer.set((int64_t)neo_fade_timeout_us);
        for (int32_t idiot=0; idiot<idiotCount; idiot++) {
            setBoolState(idiot, 0);
            idiotNormalColor[idiot] = 0x808080;
            updateIdiot(idiot);
        }
        refresh();
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
                if (heartbeat_pulse) heartbeat_brightness = brightlev[context][B_MED];
                else neoFadeTimer.reset();
            }
            else if (!heartbeat_pulse && heartbeat_brightness > brightlev[context][B_MIN]) {
                if (neoFadeTimer.expired()) heartbeat_brightness = brightlev[context][B_MIN];
                else heartbeat_brightness = (int8_t)(brightlev[context][B_MIN] + (float)(brightlev[context][B_LO] - brightlev[context][B_MIN]) * (1 - (float)neoFadeTimer.elapsed() / (float)neo_fade_timeout_us));
                
            }
            if (heartbeatColor != heartbeatColor_last || heartbeat_brightness != neobright_last) {
                heartbeatNow = dimmer(heartbeatColor, heartbeat_brightness);
                neostrip->setPixelColor(0, heartbeatNow);
                heartbeatColor_last = heartbeatColor;
                neobright_last = heartbeat_brightness;
            }
        }
    }
    void colorfade_update() {
        if (neoFadeTimer.expireset()) {  // Rainbow fade
            neostrip->setPixelColor (0, colorwheel(++neo_wheelcounter));
            neostrip->show();
        }
    }
    uint32_t neopixelsAvailable() {
        return idiotCount;
    }
    bool newIdiotLight(uint32_t idiot, uint16_t color565, bool startboolstate = 0) {
        idiotBoolState[idiot] = startboolstate;
        if (idiot > idiotCount-1) return false;
        idiotNormalColor[idiot] = color_16b_to_32b(color565);
        setBoolState(idiot, idiotBoolState[idiot]);
        updateIdiot(idiot);
        return true;
    }
    void setBoolState(uint32_t idiot, bool state) {
        if (idiot < idiotCount) {
            idiotBoolState[idiot] = state;
            idiotUrgency[idiot] = (state) ? 3 : 1;
        }
    }
    void updateIdiot(uint32_t idiot) {
        if (idiotUrgency[idiot] <= 0) idiotNowColor[idiot] = brightlev[context][B_OFF];  // Turn off the light
        else if (idiotUrgency[idiot] == 1) idiotNowColor[idiot] = dimmer(desaturate(idiotNormalColor[idiot], 5), brightlev[context][B_MIN]);
        else if (idiotUrgency[idiot] == 2) idiotNowColor[idiot] = dimmer(desaturate(idiotNormalColor[idiot], 5), brightlev[context][B_LO]);
        else if (idiotUrgency[idiot] == 3) idiotNowColor[idiot] = dimmer(desaturate(idiotNormalColor[idiot], 5), brightlev[context][B_MED]);
        else if (idiotUrgency[idiot] <= 6) {  // Flash alternating with black, at increasing frequency
            // todo : implement this effect
        }
        else if (idiotUrgency[idiot] <= 9) {  // Flash alternating with white, at increasing frequency
            // todo : implement this effect
        }
        else if (idiotUrgency[idiot] == 10) {  // Continuous white/color strobe 
            // todo : implement this effect
        }
        neostrip->setPixelColor (1+idiot, idiotNowColor[idiot]);
    }
    void refresh() {
        bool something_changed = (heartbeatNow != color_last[0]);
        color_last[0] = heartbeatNow;
        for (int32_t idiot=0; idiot<idiotCount; idiot++) {
            if (idiotNowColor[idiot] != color_last[idiot+1]) something_changed = true;
            color_last[idiot+1] = idiotNowColor[idiot];
        }
        if (something_changed) neostrip->show();
    }
    // void assign_idiotlight(uint32_t index, bool idiotvalue, uint16_t idiotcolor) {
    //     for (int32_t idiot = 0; idiot <= min((uint32_t)arraysize(idiotlights), neopixelsAvailable()); idiot++) {
    //         neo.newIdiotLight(idiot, idiotcolors[idiot], *(idiotlights[idiot]));
    //     }
    // }
    // void draw_idiotlights (int32_t x, int32_t y, bool force = false) {
    //     for (int32_t ilite=0; ilite < arraysize(idiotlights); ilite++)
    //         if (force || (*(idiotlights[ilite]) ^ idiotlasts[ilite])) {
    //             draw_idiotlight (ilite, x + (2 * disp_font_width + 2) * ((ilite % disp_idiots_per_row) % disp_idiots_per_row), y + disp_idiot_row_height * (int32_t)(ilite / disp_idiots_per_row));
    //             if (ilite <= neo.neopixelsAvailable()) {
    //                 neo.setBoolState(ilite, *idiotlights[ilite]);
    //                 neo.updateIdiot(ilite);
    //             }
    //         }
    // }
  private:
    uint32_t dimmer(uint32_t color, int8_t bright_percent) {  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
        uint32_t rgb[3] = { color >> 16, (color & 0xff00) >> 8, color & 0xff };
        float fbright = (float)bright_percent * 2.55 / (float)max(rgb[0], rgb[1], rgb[2]);  // 2.55 = 0xff / 100
        return ((uint32_t)((float)rgb[0] * fbright) << 16) | ((uint32_t)((float)rgb[1] * fbright) << 8) | ((uint32_t)((float)rgb[2] * fbright));
    }
    uint32_t desaturate(uint32_t color, int8_t desat_percent) {  // desat_percent=0 has no effect, =100 desaturates all the way to greyscale, without change in brightness
        uint32_t rgb[3] = { color >> 16, (color & 0xff00) >> 8, color & 0xff };
        float dominant = (float)max(rgb[0], rgb[1], rgb[2]);
        for (int32_t element=0; element<3; element++) {
            rgb[element] = (uint32_t)(rgb[element] + ((float)desat_percent * (dominant - rgb[element]) / 100.0));
        }
        return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
    }
    uint32_t colorwheel(uint8_t WheelPos) {
        WheelPos = 255 - WheelPos;
        if (WheelPos < 85) return neostrip->Color(255 - WheelPos * 3, 0, WheelPos * 3);
        if (WheelPos < 170) {
            WheelPos -= 85;
            return neostrip->Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        WheelPos -= 170;
        return neostrip->Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    }
    uint32_t color_16b_to_32b(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to 32 bit BRG value suitable for neopixels
        return neostrip->Color((color565 & 0x1f) << 3, (color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3);
    }
};