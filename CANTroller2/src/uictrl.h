#pragma once
#include "FunctionalInterrupt.h"
// Potentiometer does an analog read from a pin and maps it to a percent (0%-100%). We filter the value to keep it smooth.
class Potentiometer {
  protected:
    static constexpr float adc_min = 300; // TUNED 230603 - Used only in determining theconversion factor
    static constexpr float adc_max = 4095; // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
    static constexpr float _ema_alpha = 0.35;
    static constexpr float _pc_min = 0.0;
    static constexpr float _pc_max = 100.0;
    static constexpr float _pc_activity_margin = 1.0;
    uint8_t _pin;
    float _val = 0.0, _activity_ref;
  public:
    Potentiometer(uint8_t arg_pin) : _pin(arg_pin) {}
    Potentiometer() = delete; // must have a pin defined
    void setup() {
        printf("Pot setup..\n");
        set_pin(_pin, INPUT);
        _activity_ref = _val;
    }
    void update() {
        float new_val = map(static_cast<float>(analogRead(_pin)), adc_min, adc_max, _pc_min, _pc_max);
        new_val = constrain(new_val, _pc_min, _pc_max); // the lower limit of the adc reading isn't steady (it will dip below zero) so constrain it back in range
        _val = ema_filt(new_val, _val, _ema_alpha);
        if (std::abs(_val - _activity_ref) > _pc_activity_margin) {
            kick_inactivity_timer(7);  // evidence of user activity
            _activity_ref = _val;
        }
    }
    template<typename VAL_T>
    VAL_T mapToRange(VAL_T min, VAL_T max) {
        return static_cast<VAL_T>(map(_val, _pc_min, _pc_max, static_cast<float>(min), static_cast<float>(max)));
    }
    float val() { return _val; }
    float min() { return _pc_min; }
    float max() { return _pc_max; }
};
class MomentaryButton {
  private:
    int32_t _sw_action = swNONE;  // Flag for encoder handler to know an encoder switch action needs to be handled
    bool _timer_active = false;  // Flag to prevent re-handling long presses if the sw is just kept down
    bool _suppress_click = false;  // Flag to prevent a short click on switch release after successful long press
    bool activity_timer_keepalive = true;  // will activity on this switch be considered that the user is active?
    Timer _spinspeedTimer;  // Used to figure out how fast we're spinning the knob.  OK to not be volatile?
    //  ---- tunable ----
    Timer _longPressTimer = Timer(375000);  // Used to time long button presses
  public:
    int _sw_pin = -1;
    bool now = false;  // Remember whether switch is being pressed
    MomentaryButton() {}
    MomentaryButton(int pin, bool _act_keepalive=true) : _sw_pin(pin), activity_timer_keepalive(_act_keepalive) {}
    void set_pin(int pin) {
        _sw_pin = pin;
    }
    void update() {
        // Read and interpret encoder switch activity. Encoder rotation is handled in interrupt routine
        // Encoder handler routines should act whenever encoder_sw_action is swSHORT or swLONG, setting it back to
        // swNONE once handled. When handling press, if encoder_long_clicked is nonzero then press is a long press
        bool myread = now;
        do {
            myread = digitalRead(_sw_pin);   // !value because electrical signal is active low
        } while (myread != digitalRead(_sw_pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure

        if (!myread) {  // if encoder sw is being pressed (switch is active low)
            if (!now) {  // if the press just occurred
                if (activity_timer_keepalive) kick_inactivity_timer(0);  // evidence of user activity
                _longPressTimer.reset();  // start a press timer
                _timer_active = true;  // flag to indicate timing for a possible long press
            }
            else if (_timer_active && _longPressTimer.expired()) {  // If press time exceeds long press threshold
                _sw_action = swLONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
                _timer_active = false;  // Keeps us from entering this logic again until after next sw release (to prevent repeated long presses)
                _suppress_click = true;  // Prevents the switch release after a long press from causing a short press
            }
            now = true;  // Remember a press is in effect
        }
        else {  // if encoder sw is not being pressed
            if (now) {
                if (activity_timer_keepalive) kick_inactivity_timer(1);  // evidence of user activity
                if(!_suppress_click) _sw_action = swSHORT;  // if the switch was just released, a short press occurred, which must be handled
            }
            _timer_active = false;  // Allows detection of next long press event
            now = false;  // Remember press is not in effect
            _suppress_click = false;  // End click suppression
        }
    }
    bool pressed() {
        return now;
    }
    uint32_t press_event(bool autoreset = true) {
        uint32_t ret = _sw_action;
        if (autoreset) _sw_action = swNONE;
        return ret;
    }
    void press_reset() {
        _sw_action = swNONE;
    }
    bool longpress() {  // code may call this to check for long press and if so act upon it. Resets the long press if asserted
        bool ret = (_sw_action == swLONG);
        if (ret) _sw_action = swNONE;
        return ret;
    }
    bool shortpress() {  // code may call this to check for short press and if so act upon it. Resets the long press if asserted
        bool ret = (_sw_action == swSHORT);
        if (ret) _sw_action = swNONE;
        return ret;
    }
    void setup(int pin = -1) {
        if (pin != -1) _sw_pin = pin;
        pinMode(_sw_pin, INPUT_PULLUP);
    }
    void setLongPressTimer(uint32_t t){
        _longPressTimer.set(t);
    }
};
class Encoder {
    private:
        enum _inputs { ENC_A, ENC_B };

        // class vars
        //  ---- tunable ----
        // TODO: these are all currently private const, if we ever actually want to tune them live we would need to change this
        static const uint32_t _spinrate_min_us = 2500;  // Will reject spins faster than this as an attempt to debounce behavior
        static const uint32_t _accel_thresh_us = 60000;  // Spins faster than this will be accelerated
        static const int32_t _accel_max = 15;  // Maximum acceleration factor

        // instance vars
        volatile uint32_t _spinrate_isr_us = 100000;  // Time elapsed between last two detents
        volatile int32_t _bounce_danger = ENC_B;  // Which of the encoder A or B inputs is currently untrustworthy due to bouncing 
        volatile int32_t _delta = 0;  // Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 
        int _a_pin, _b_pin, _sw_pin;
        int32_t _state = 0;
        uint32_t _spinrate_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
        Timer _spinspeedTimer;  // Used to figure out how fast we're spinning the knob.  OK to not be volatile?
        //  ---- tunable ----

        void IRAM_ATTR _a_isr() {
            if (_bounce_danger != Encoder::ENC_A) {
                if (!enc_a) {
                    _spinrate_isr_us = _spinspeedTimer.elapsed();
                    _spinspeedTimer.reset();
                    // _spinrate_isr_us = _spinspeedTimer.elapset();
                    enc_b = digitalRead(_b_pin);
                    _delta += enc_b ? -1 : 1;
                }
                _bounce_danger = Encoder::ENC_A;
            }
        }

        void IRAM_ATTR _b_isr() {
            if (_bounce_danger != Encoder::ENC_B) {
                enc_a = digitalRead(_a_pin);
                _bounce_danger = Encoder::ENC_B;
            }
        }

    public:
        MomentaryButton button;
        bool enc_a = 1;  // if initializing to 0 sets us up right after a reboot with a bit of a hair trigger which turns left at the slightest touch
        bool enc_b;
        Encoder(int a, int b, int sw) : _a_pin(a), _b_pin(b), _sw_pin(sw) {
            button.setup(_sw_pin);
        }
        Encoder() = delete; // must be instantiated with pins
        
        void setup() {
            printf("Encoder setup..\n");
            set_pin(_a_pin, INPUT_PULLUP);
            set_pin(_b_pin, INPUT_PULLUP);
            button.setup();
            // set_pin(_sw_pin, INPUT_PULLUP);  // The esp32 pullup is too weak. Use resistor
            attachInterrupt(digitalPinToInterrupt(_a_pin), [this]{ _a_isr(); }, CHANGE); \
            attachInterrupt(digitalPinToInterrupt(_b_pin), [this]{ _b_isr(); }, CHANGE);
        }
        void update() {
            button.update();
        }
        int32_t rotation(bool accel = false) {  // Returns detents spun since last call, accelerated by spin rate or not
            int32_t d = 0;
            if (_delta) {  // Now handle any new rotations
                kick_inactivity_timer(2);  // evidence of user activity
                if (_spinrate_isr_us >= _spinrate_min_us) {  // Reject clicks coming in too fast as bounces
                    if (accel) {
                        _spinrate_us = constrain (_spinrate_isr_us, _spinrate_min_us, _accel_thresh_us);
                        _spinrate_us = map (_spinrate_us, _spinrate_min_us, _accel_thresh_us, _accel_max, 1);  // if turning faster than 100ms/det, proportionally accelerate the effect of each detent by up to 50x. encoder_temp variable repurposed here to hold # of edits per detent turned
                        d = _delta * _spinrate_us;  // If a tunable value is being edited, turning the encoder changes the value
                    }
                    else d = constrain (_delta, -1, 1);  // Only change one at a time when selecting or turning pages
                }
                _delta = 0;  // Our responsibility to reset this flag after handling events
            }
            return d;
        }
};