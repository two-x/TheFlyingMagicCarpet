#ifndef UICTRL_H
#define UICTRL_H

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
    uint8_t neo_wheelcounter = 0;
    uint8_t neo_brightness_dim = 5;
    uint8_t neo_brightness_medium = 15;
    uint8_t neo_brightness_max = 30;
    uint32_t neo_timeout_us = 150000;
    uint32_t neo_heartbeat_timeout_us = 1000000;
    Timer neoTimer, heartbeatTimer;
    bool neo_heartbeat = false;
    uint8_t heartbeat_brightness = neo_brightness_medium; // brightness during fadeouts
    uint32_t heartbeatColor;
    int32_t heartbeat_state = 0;
    int32_t heartbeat_level = 0;
    uint32_t heartbeat_ekg_us[4] = {170000, 150000, 530000, 1100000};
    int32_t heartbeat_pulse = 255;
  protected:
    static const uint32_t pixelCount = 8;
    static const uint32_t idiotCount = pixelCount - 1;
    bool idiotBoolstate[idiotCount];  // For simple boolean idiot light, LOW will set urgency=1, and HIGH sets urgency=3.
    uint32_t idiotNormalColor[idiotCount];  // 
    uint32_t idiotNowColor[idiotCount];  // 
    uint32_t idiotUrgency[idiotCount];  // urgency is from 1 to 10 level of freakout (0=off, 1=dim, 2=medium, 3=bright, 4-6=flash/black, 7-9=flash/white, 10=strobe) 
    Adafruit_NeoPixel* neostrip;
  public:
    neopixelStrip(int32_t pin=-1) {  // , int32_t count=1
        // pixelCount = count;
        neostrip = new Adafruit_NeoPixel(pixelCount, pin, NEO_GRB + NEO_GRB + NEO_KHZ800);
        neoTimer.set((int64_t)neo_timeout_us);
        heartbeatTimer.set((int64_t)neo_heartbeat_timeout_us);
    }
    ~neopixelStrip() { delete neostrip; }

    void init() {
        for (int32_t idiot=0; idiot<=idiotCount; idiot++) {

        }
        neostrip->begin();  // start datastream
        neostrip->show();  // Turn off the pixel
        neostrip->setBrightness (neo_brightness_medium);  // It truly is incredibly bright
    }
    void heartbeat(bool onoroff) {
        neo_heartbeat = onoroff;  // Start heart beating
    }
  private:
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
  public:
    void heartbeat_update(uint16_t runmode_color) {
        if (neo_heartbeat) {
            heartbeatColor = color_16b_to_32b(runmode_color);
            if (heartbeatTimer.expired()) {
                heartbeat_pulse = !heartbeat_pulse;
                if (heartbeat_pulse) heartbeat_brightness = neo_brightness_medium;
                else neoTimer.reset();
                if (++heartbeat_state >= arraysize (heartbeat_ekg_us)) heartbeat_state -= arraysize (heartbeat_ekg_us);
                heartbeatTimer.set (heartbeat_ekg_us[heartbeat_state]);
            }
            else if (!heartbeat_pulse && heartbeat_brightness) {
                heartbeat_brightness = (int8_t)((float)neo_brightness_medium * (1 - (float)neoTimer.elapsed() / (float)neo_timeout_us));
                if (neoTimer.expired() || heartbeat_brightness < 1) heartbeat_brightness = 0;
            }
            int32_t heartbeatColor_last, neobright_last;
            if (heartbeatColor != heartbeatColor_last || heartbeat_brightness != neobright_last) {
                neostrip->setPixelColor (0, heartbeatColor);
                neostrip->setBrightness (heartbeat_brightness);
                neostrip->show();
                heartbeatColor_last = heartbeatColor;
                neobright_last = heartbeat_brightness;
            }
        }
    }
    void colorfade_update() {
        if (neoTimer.expireset()) {  // Rainbow fade
            neostrip->setPixelColor (0, colorwheel(++neo_wheelcounter));
            neostrip->show();
        }
    }
    bool newIdiotLight(uint32_t idiotIndex, uint16_t color565, bool startboolstate = 0) {
        idiotBoolstate[idiotIndex] = startboolstate;
        if (idiotIndex > idiotCount-1) return false;
        idiotNormalColor[idiotIndex] = color_16b_to_32b(color565);
        setBoolState(idiotIndex, idiotBoolstate[idiotIndex]);
        updateIdiot(idiotIndex);
        return true;
    }
    void setBoolState(uint32_t idiotIndex, bool state) {
        idiotUrgency[idiotIndex] = (state) ? 3 : 1;
    }
    void updateIdiot(uint32_t idiotIndex) {
        if (idiotUrgency[idiotIndex] == 1) {
            idiotNowColor[idiotIndex] = idiotNormalColor[idiotIndex];
        }
    }
};

#endif