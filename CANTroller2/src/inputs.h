#pragma once
#include "FunctionalInterrupt.h"
// Potentiometer does an analog read from a pin and maps it to a percent (0%-100%). We filter the value to keep it smooth.
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
#define touch_cell_v_pix 48  // When touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53  // When touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1  // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen
#define touch_reticle_offset 50  // Distance of center of each reticle to nearest screen edge
#define disp_tuning_lines 11  // Lines of dynamic variables/values in dataset pages 
class Touchscreen {
  private:
    LGFX* _tft;
    I2C* _i2c;
    int32_t corners[2][2][2] = { { { -25, -3549 }, { 185, 3839 } },  // [restouch][xx/yy][min/max]  // Read resistance values from upper-left and lower-right corners of screen, for calibration
                                 { { -100, 319 }, { 0, 174 } } };   // [captouch][xx/yy][min/max]  // Read resistance values from upper-left and lower-right corners of screen, for calibration
    bool touch_longpress_valid = true;
    bool landed_coordinates_valid = false;
    bool lasttouch = false;
    int tedit_exponent = 0;
    float tedit = (float)(1 << tedit_exponent);
    int touch_fudge = 0;
    int tedit_exponent_max = 8;
    int tlast_x, tlast_y;
    Timer touchHoldTimer{550000};  // Hold this long to count as a long press
    Timer touchAccelTimer{850000};
    Timer touchDoublePressTimer{40000};  // Won't allow a new press within this long after an old press (prevent accidental double clicks)
    Timer touchSenseTimer{15000};  // touch chip can't respond faster than some time period
    // debug printing
    bool touchPrintEnabled = true;
    unsigned long lastTouchPrintTime = 0;
    const unsigned long touchPrintInterval = 500; // Adjust this interval as needed (in milliseconds)
    enum touch_axis : int { xx, yy, zz };
    enum touch_lim : int { tsmin, tsmax };
    int trow, tcol, disp_size[2], touch_read[3], tft_touch[2], landed[2];  // landed are the initial coordinates of a touch event, unaffected by dragging
    // uint16_t touch_cal_data[5] = { 404, 3503, 460, 3313, 1 };  // Got from running TFT_eSPI/examples/Generic/Touch_calibrate/Touch_calibrate.ino
    // lcd.setTouch(touch_cal_data);
  public:
    static constexpr uint8_t addr = 0x38;  // i2c addr for captouch panel
    int idelta = 0;
    Touchscreen() {}
    void setup(LGFX* tft, I2C* i2c, int width, int height) {
        _tft = tft;
        _i2c = i2c;
        disp_size[HORZ] = width;
        disp_size[VERT] = height;
        captouch = (i2c->detected(i2c_touch));
        // _tft->touch_init();  // this points touch object to resistive or capacitive driver instance based on captouch
        Serial.printf("Touchscreen.. %s panel\n", (captouch) ? "detected captouch" : "using resistive");
    }
    bool touched() { return nowtouch; }
    int touch_x() { return tft_touch[xx]; }
    int touch_y() { return tft_touch[yy]; }
    int16_t touch_pt(uint8_t axis) { return tft_touch[axis]; }

    int read_touch() {
        if (captouch && _i2c->not_my_turn(i2c_touch)) return nowtouch;
        if (touchSenseTimer.expireset()) {
            uint8_t count = _tft->getTouch(&(touch_read[xx]), &(touch_read[yy]));
            // if (captouch) count = _tft->getTouch(&(touch_read[xx]), &(touch_read[yy]), &(touch_read[zz]));
            // Serial.printf("n%d rx:%d ry:%d ", nowtouch, touch_read[0], touch_read[1]);
            nowtouch = (count > 0);
            if (nowtouch) {
                for (int axis=0; axis<=1; axis++) {
                    // if (captouch) tft_touch[axis] = touch_read[axis];  // disp_width - 1 - touch_read[xx];
                    tft_touch[axis] = map(touch_read[axis], corners[captouch][axis][tsmin], corners[captouch][axis][tsmax], 0, disp_size[axis]);
                    tft_touch[axis] = constrain(tft_touch[axis], 0, disp_size[axis] - 1);
                    if (flip_the_screen) tft_touch[axis] = disp_size[axis] - tft_touch[axis];
                    if (!landed_coordinates_valid) {
                        landed[axis] = tft_touch[axis];
                        if (axis) landed_coordinates_valid = true;  // on 2nd time thru set this true
                    }
                }
            }
            else landed_coordinates_valid = false;
            // Serial.printf("tx:%d ty:%d\n", tft_touch[0], tft_touch[1]);
        }
        _i2c->pass_i2c_baton();
        return nowtouch;
    }
    void update() {
        read_touch();
        if (!nowtouch) {  // if not being touched
            idelta = 0;  // Stop changing the value
            if (lasttouch) touchDoublePressTimer.reset();  // Upon end of a touch, begin timer to reject any accidental double touches
            tedit_exponent = 0;
            tedit = (float)(1 << tedit_exponent); // Reset touch acceleration value to 1
            touchHoldTimer.reset();
            touch_longpress_valid = true;
        }
        else {
            // if (touchDoublePressTimer.expired()) {
            // Serial.printf("(%d,%d), landed(%d,%d)\n",tft_touch[xx],tft_touch[yy],landed[xx],landed[yy]);
            kick_inactivity_timer(4);  // evidence of user activity
            if (ui_context == DatapagesUI) process_ui();
        }
        lasttouch = nowtouch;
    }
    void process_ui() {
        tedit = (float)(1 << tedit_exponent);  // Determine value editing rate
        trow = constrain((landed[yy] + touch_fudge) / touch_cell_v_pix, 0, 4);
        tcol = constrain((landed[xx] - touch_margin_h_pix) / touch_cell_h_pix, 0, 5);
        if (tcol == 0 && trow == 0 && !lasttouch) {
            // if (!(landed[xx] == 0 && landed[yy] == 0)) touch_increment_datapage = true;  // Displayed dataset page can also be changed outside of simulator  // trying to prevent ghost touches we experience occasionally
            touch_increment_datapage = true;  // Displayed dataset page can also be changed outside of simulator  // trying to prevent ghost touches we experience occasionally
        }
        else if (tcol == 0 && trow == 1) {  // Long touch to enter/exit editing mode, if in editing mode, press to change the selection of the item to edit
            if (tunctrl == OFF) {
                sel_val = 0;  // If entering select mode from off mode, select the first variable
                if (touch_longpress_valid && touchHoldTimer.expired()) {
                    tunctrl = SELECT;
                    touch_longpress_valid = false;
                }
            }
            else if (tunctrl == EDIT && !lasttouch) {
                tunctrl = SELECT;  // Drop back to select mode
                sel_val++;  // Move to the next selection
            }
            else if (tunctrl == SELECT) {
                if (!lasttouch) sel_val = (sel_val + 1) % disp_tuning_lines;
                else if (touch_longpress_valid && touchHoldTimer.expired()) {
                    tunctrl = OFF;
                    touch_longpress_valid = false;
                }
            }
        }
        else if (tcol == 0 && trow == 2) {  // Pressed the increase value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // If just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta = (int)tedit;  // If in edit mode, increase the value
        }
        else if (tcol == 0 && trow == 3) {  // Pressed the decrease value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // If just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta = (int)(-tedit);  // If in edit mode, decrease the value
        }
        else if (tcol == 0 && trow == 4) {  // Pressed the simulation mode toggle. Needs long-press
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout()) {
                sim.toggle();
                touch_longpress_valid = false;
            }
        }
        else if (sim.enabled()) {
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout()) {
                if (tcol == 2 && trow == 0) calmode_request = true;
                else if (tcol == 4 && trow == 0) ignition_request = REQ_TOG;
                else if (tcol == 5 && trow == 0) sleep_request = REQ_TOG;  // sleep requests are handled by shutdown or asleep mode, otherwise will be ignored
                touch_longpress_valid = false;
            }
            else if (tcol == 3 && trow == 0 && sim.can_sim(sens::basicsw) && !lasttouch) basicmodesw = !basicmodesw;
            else if (tcol == 3 && trow == 1 && sim.can_sim(sens::pressure) && pressure.source() == src::TOUCH) pressure.add_human(tedit); // (+= 25) Pressed the increase brake pressure button
            else if (tcol == 3 && trow == 2 && sim.can_sim(sens::pressure) && pressure.source() == src::TOUCH) pressure.add_human(-tedit); // (-= 25) Pressed the decrease brake pressure button
            else if (tcol == 3 && trow == 3 && sim.can_sim(sens::brkpos) && brkpos.source() == src::TOUCH) brkpos.add_human(tedit); // (-= 25) Pressed the decrease brake pressure button
            else if (tcol == 3 && trow == 4 && sim.can_sim(sens::brkpos) && brkpos.source() == src::TOUCH) brkpos.add_human(-tedit); // (-= 25) Pressed the decrease brake pressure button
            else if (tcol == 4 && trow == 1 && sim.can_sim(sens::tach) && tach.source() == src::TOUCH) tach.add_human(tedit);
            else if (tcol == 4 && trow == 2 && sim.can_sim(sens::tach) && tach.source() == src::TOUCH) tach.add_human(-tedit);
            else if (tcol == 4 && trow == 3 && sim.can_sim(sens::joy)) adj_val(&hotrc.pc[VERT][FILT], tedit, hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
            else if (tcol == 4 && trow == 4 && sim.can_sim(sens::joy)) adj_val(&hotrc.pc[VERT][FILT], -tedit, hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
            else if (tcol == 5 && trow == 1 && sim.can_sim(sens::speedo) && speedo.source() == src::TOUCH) speedo.add_human(tedit);
            else if (tcol == 5 && trow == 2 && sim.can_sim(sens::speedo) && speedo.source() == src::TOUCH) speedo.add_human(-tedit);
            else if (tcol == 5 && trow == 3 && sim.can_sim(sens::joy)) adj_val(&hotrc.pc[HORZ][FILT], tedit, hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
            else if (tcol == 5 && trow == 4 && sim.can_sim(sens::joy)) adj_val(&hotrc.pc[HORZ][FILT], -tedit, hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
        }
        if (tedit_exponent < tedit_exponent_max && (touchHoldTimer.elapsed() > (tedit_exponent + 1) * touchAccelTimer.timeout())) {
            tedit_exponent++;
            tedit = (float)(1 << tedit_exponent); // Update the touch acceleration value
        }
    }
    void enableTouchPrint(bool enable) {
        touchPrintEnabled = enable;
    }
    void printTouchInfo() {
        if (touchPrintEnabled && touched()) {
            unsigned long currentTime = millis();
            if (currentTime - lastTouchPrintTime >= touchPrintInterval) {}
            Serial.printf("Touch %sdetected", (read_touch()) ? "" : "not ");
            if (nowtouch) Serial.printf(" x:%d y:%d\n", tft_touch[xx], tft_touch[yy]);
            else Serial.printf("\n");                // If available, you can print touch pressure as well (for capacitive touch)
            lastTouchPrintTime = currentTime;
        }
    }
};