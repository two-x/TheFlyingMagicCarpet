#pragma once
#include "FunctionalInterrupt.h"
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
        } while (myread != digitalRead(_sw_pin)); // some pins have a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure

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
    static const int _accel_max = 15;  // Maximum acceleration factor

    // instance vars
    volatile uint32_t _spinrate_isr_us = 100000;  // Time elapsed between last two detents
    volatile uint32_t isr_time_now;
    volatile uint32_t isr_time_last;
    volatile int _bounce_lock = ENC_B;  // Which of the encoder A or B inputs is currently untrustworthy due to bouncing 
    static const uint32_t _bounce_expire_us = 10000;  // need to let bounce lock expire to reliably catch turn events in either direction on direction reversals
    volatile int _delta = 0;  // Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 
    int _a_pin, _b_pin, _sw_pin;
    int _state = 0;
    uint32_t _spinrate_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
    //  ---- tunable ----

    // Encoder isr. note you gotta set the define EncoderPanasonicType to match your encoder type
    // If true, supports one type of cheap amazon black-pcb. 2024 one of these is mounted in the vehicle control enclosure
    //   * these guys hit a detent around each B transition, and A transitions in between detents
    //   * on an A transition, you're going CCW if the new value of A and the [now stable] B value are the same
    //   * if your encoder is the wrong kind for this algorithm, it'll act all squirrely, giving spurious double-actions etc., switch to the other algorithm
    // if false, supports a different type of cheap amazon black-pcb encoders
    //   * the A signal starts low at each detent, and goes high then back low before the next detent 
    //   * the value of the B signal on the falling edge of the A signal is high if going CW, otherwise low
    //   * if your encoder is the wrong kind for this algorithm, it'll behave all sluggish, like usually ignoring every other detent, switch to the other algorithm
    #if EncoderPanasonicType                              // eg the vehicle control box encoder
    void IRAM_ATTR _a_isr() {                             // A isr has been triggered by A signal rise or fall transition. A will now bounce for a while and can't be trusted
        isr_time_now = esp_timer_get_time();              // put some flavor in your flav
        uint32_t elapsed = isr_time_now - isr_time_last;  // note time elapsed since last valid event on A isr. might be invalid for use by outside code if this is a bounce   
        if (_bounce_lock == Encoder::ENC_A) {             // if A isr is bounce locked, B isr hasn't yet triggered since our last trigger
            if (elapsed <= _bounce_expire_us) return;     // if bouncing is still a thing then bail
            val_a_isr = digitalRead(_a_pin);              // otherwise since B isr hasn't triggered, we are reversing direction and must update our own value
        }
        else val_b_isr = digitalRead(_b_pin);             // get a stable reading of B signal, unless we know B hasn't changed then skip for speed
        _spinrate_isr_us = elapsed;                       // save elapsed time since last trigger, to calculate spin rate
        isr_time_last = isr_time_now;                     // reset spin rate timer for next time
        _delta += (val_a_isr == val_b_isr) ? -1 : 1;      // increment delta for each CW event and vice versa. handler in code may reset delta to 0 as it sees fit
        _bounce_lock = Encoder::ENC_A;                    // bounce lock this isr to avoid retriggering, and unlock the B isr
    }
    #else                                                    // eg soren's dev board encoder
    void IRAM_ATTR _a_isr() {                                // A isr has been triggered by A signal rise or fall transition. A will now bounce for a while and can't be trusted
        if (_bounce_lock == Encoder::ENC_A) return;          // if A isr is bounce locked then bail
        if (!val_a_isr) {                                    // if the A signal is now low (i.e. recent B isr read A pin as high)
            isr_time_now = esp_timer_get_time();             // put some flavor in your flav
            _spinrate_isr_us = isr_time_now - isr_time_last; // save elapsed time since last trigger, to calculate spin rate
            isr_time_last = isr_time_now;                    // reset spin rate timer for next time
            val_b_isr = !digitalRead(_b_pin);                // Get a clean reading of the B signal
            _delta += val_b_isr ? -1 : 1;                    // increment delta for each CW event and vice versa. handler in code may reset delta to 0 as it sees fit
        }
        _bounce_lock = Encoder::ENC_A;                       // bounce lock this isr to avoid retriggering, and unlock the B isr
    }
    #endif
    void IRAM_ATTR _b_isr() {                        // B isr has been triggered by B signal rise or fall transition. B will now bounce for a while and can't be trusted
        if (_bounce_lock == Encoder::ENC_B) return;  // if B isr is bounce locked then bail
        val_a_isr = !digitalRead(_a_pin);            // get a clean reading of A signal, forecasting it will invert
        _bounce_lock = Encoder::ENC_B;               // bounce lock this isr to avoid retriggering, and unlock the A isr
    }
  public:
    MomentaryButton button;
    bool enc_a, val_a_isr = LOW;  // if initializing HIGH sets us up right after a reboot with a bit of a hair trigger which turns left at the slightest touch
    bool enc_b, val_b_isr = HIGH;
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
        enc_a = !digitalRead(_a_pin);
        enc_b = !digitalRead(_b_pin);
        if (run.mode == LOWPOWER && !syspower) {
            if (button.shortpress()) sleep_request = REQ_OFF;
        }
        else if (run.mode == STANDBY) {
            if (button.shortpress()) autosaver_request = REQ_OFF;
        }
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
            // _delta = 0;  // [not] our responsibility to reset this flag after handling events
        }
        return d;
    }
    void rezero() { _delta = 0; }  // Handling code needs to call to rezero after reading rotations
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
    int tedit_exponent_max = 6;
    int tlast_x, tlast_y;
    Timer touchHoldTimer{550000};  // Hold this long to count as a long press
    Timer touchAccelTimer{400000};
    Timer touchDoublePressTimer{40000};  // Won't allow a new press within this long after an old press (prevent accidental double clicks)
    Timer touchSenseTimer{15000};  // touch chip can't respond faster than some time period
    bool touchPrintEnabled = true;
    unsigned long lastTouchPrintTime = 0;
    const unsigned long touchPrintInterval = 500; // Adjust this interval as needed (in milliseconds)
    enum touch_axis : int { xx, yy, zz };
    enum touch_lim : int { tsmin, tsmax };
    int trow, tcol, disp_size[2], touch_read[3], tft_touch[2], landed[2];  // landed are the initial coordinates of a touch event, unaffected by subsequent dragging
    uint16_t tquad;  // imagine screen divided into rows and columns as touch buttons. First byte encodes row, 2nd byte is column 
    // uint16_t touch_cal_data[5] = { 404, 3503, 460, 3313, 1 };  // Got from running TFT_eSPI/examples/Generic/Touch_calibrate/Touch_calibrate.ino
    // lcd.setTouch(touch_cal_data);
  public:
    static constexpr uint8_t addr = 0x38;  // i2c addr for captouch panel
    int idelta = 0;
    bool increment_datapage = false, increment_sel_val = false;
    Touchscreen() {}
    void setup(LGFX* tft, I2C* i2c, int width, int height) {
        if (!display_enabled) return;
        _tft = tft;
        _i2c = i2c;
        disp_size[HORZ] = width;
        disp_size[VERT] = height;
        captouch = (i2c->detected(i2c_touch));
        Serial.printf("Touchscreen.. %s panel\n", (captouch) ? "detected captouch" : "using resistive");
    }
    bool touched() { return nowtouch; }
    int touch_x() { return tft_touch[xx]; }
    int touch_y() { return tft_touch[yy]; }
    int16_t touch_pt(uint8_t axis) { return tft_touch[axis]; }
    bool ontouch() { return nowtouch && !lasttouch; }
    bool longpress() {
        bool retval = touch_longpress_valid && touchHoldTimer.expired();
        if (retval) touch_longpress_valid = false;
        return retval;
    }
    void update() {
        if (captouch && _i2c->not_my_turn(i2c_touch)) return;
        if (touchSenseTimer.expireset()) {
            uint8_t count = _tft->getTouch(&(touch_read[xx]), &(touch_read[yy]));
            // if (captouch) count = _tft->getTouch(&(touch_read[xx]), &(touch_read[yy]), &(touch_read[zz]));
            // Serial.printf("n%d rx:%d ry:%d ", nowtouch, touch_read[0], touch_read[1]);
            nowtouch = (count > 0);
            if (nowtouch) {
                kick_inactivity_timer(4);  // evidence of user activity
                for (int axis=0; axis<=1; axis++) {
                    // if (captouch) tft_touch[axis] = touch_read[axis];  // disp_width - 1 - touch_read[xx];
                    tft_touch[axis] = map(touch_read[axis], corners[captouch][axis][tsmin], corners[captouch][axis][tsmax], 0, disp_size[axis]);
                    tft_touch[axis] = constrain(tft_touch[axis], 0, disp_size[axis] - 1);
                    if (flip_the_screen) tft_touch[axis] = disp_size[axis] - tft_touch[axis];
                    if (!landed_coordinates_valid) {
                        landed[axis] = tft_touch[axis];
                        if (axis) {
                            landed_coordinates_valid = true;  // on 2nd time thru set this true
                        }
                    }
                }
                if (ui_context == DatapagesUI) {
                    if (touchHoldTimer.elapsed() > (tedit_exponent + 1) * touchAccelTimer.timeout()) {
                        tedit_exponent = constrain(tedit_exponent+1, 0, tedit_exponent_max);
                    }
                    tedit = (float)(1 << tedit_exponent); // Update the touch acceleration value
                    process_ui();
                }
            }
            else {  // if not being touched
                if (lasttouch) {
                    landed_coordinates_valid = false;
                    idelta = 0;  // Stop changing the value
                    // touchDoublePressTimer.reset();  // Upon end of a touch, begin timer to reject any accidental double touches
                    tedit_exponent = 0;
                    tedit = (float)(1 << tedit_exponent); // Reset touch acceleration value to 1
                }
                touchHoldTimer.reset();
                touch_longpress_valid = true;
            }
        }
        _i2c->pass_i2c_baton();
        lasttouch = nowtouch;
    }
    void process_ui() {
        if (!nowtouch) return;
        tquad = (constrain((landed[xx] - touch_margin_h_pix) / touch_cell_h_pix, 0, 5) << 4) | constrain((landed[yy] + touch_fudge) / touch_cell_v_pix, 0, 4);
        // Serial.printf("n%dl%dv%d q%02x tx:%3d ty:%3d e%d x%d\r", nowtouch, lasttouch, landed_coordinates_valid, tquad, tft_touch[0], tft_touch[1], tedit, (int)tedit_exponent);
        // std::cout << "n" << nowtouch << " e" << tedit << " x" << tedit_exponent << "\r";
        if (tquad == 0x00 && ontouch()) increment_datapage = true;  // Displayed dataset page can also be changed outside of simulator  // trying to prevent ghost touches we experience occasionally
        else if (tquad == 0x01) {  // Long touch to enter/exit editing mode, if in editing mode, press to change the selection of the item to edit
            if (tunctrl == OFF) {
                sel_val = 0;  // If entering select mode from off mode, select the first variable
                if (longpress()) tunctrl = SELECT;
            }
            else if (tunctrl == EDIT && ontouch()) {
                tunctrl = SELECT;  // Drop back to select mode
                increment_sel_val = true;  // Move to the next selection
            }
            else if (tunctrl == SELECT) {
                if (ontouch()) increment_sel_val = true;
                else if (longpress()) tunctrl = OFF;
            }
        }
        else if (tquad == 0x02) {  // Pressed the increase value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // If just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta = (int)tedit;  // If in edit mode, increase the value
        }
        else if (tquad == 0x03) {  // Pressed the decrease value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // If just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta = (int)(-tedit);  // If in edit mode, decrease the value
        }
        else if (tquad == 0x04 && longpress()) sim.toggle();  // Pressed the simulation mode toggle. Needs long-press
        else if (tquad == 0x20 && sim.enabled() && longpress()) calmode_request = true;
        else if (tquad == 0x40 && sim.enabled() && longpress()) ignition.request(REQ_TOG);
        else if (tquad == 0x50 && sim.enabled() && longpress()) sleep_request = REQ_TOG;  // sleep requests are handled by standby or lowpower mode, otherwise will be ignored
        else if (tquad == 0x30 && sim.simulating(sens::basicsw) && longpress()) basicmode_request = true;
        else if (tquad == 0x31 && sim.simulating(sens::pressure) && pressure.source() == src::TOUCH) pressure.add_human(tedit); // (+= 25) Pressed the increase brake pressure button
        else if (tquad == 0x32 && sim.simulating(sens::pressure) && pressure.source() == src::TOUCH) pressure.add_human(-tedit); // (-= 25) Pressed the decrease brake pressure button
        else if (tquad == 0x33 && sim.simulating(sens::brkpos) && brkpos.source() == src::TOUCH) brkpos.add_human(tedit); // (-= 25) Pressed the decrease brake pressure button
        else if (tquad == 0x34 && sim.simulating(sens::brkpos) && brkpos.source() == src::TOUCH) brkpos.add_human(-tedit); // (-= 25) Pressed the decrease brake pressure button
        else if (tquad == 0x41 && sim.simulating(sens::tach) && tach.source() == src::TOUCH) tach.add_human(tedit);
        else if (tquad == 0x42 && sim.simulating(sens::tach) && tach.source() == src::TOUCH) tach.add_human(-tedit);
        else if (tquad == 0x43 && sim.simulating(sens::joy)) adj_val(&hotrc.pc[VERT][FILT], tedit, hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
        else if (tquad == 0x44 && sim.simulating(sens::joy)) adj_val(&hotrc.pc[VERT][FILT], -tedit, hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
        else if (tquad == 0x51 && sim.simulating(sens::speedo) && speedo.source() == src::TOUCH) speedo.add_human(tedit);
        else if (tquad == 0x52 && sim.simulating(sens::speedo) && speedo.source() == src::TOUCH) speedo.add_human(-tedit);
        else if (tquad == 0x53 && sim.simulating(sens::joy)) adj_val(&hotrc.pc[HORZ][FILT], tedit, hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
        else if (tquad == 0x54 && sim.simulating(sens::joy)) adj_val(&hotrc.pc[HORZ][FILT], -tedit, hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
    }
    void enableTouchPrint(bool enable) {
        touchPrintEnabled = enable;
    }
    void printTouchInfo() {
        if (touchPrintEnabled && touched()) {
            unsigned long currentTime = millis();
            if (currentTime - lastTouchPrintTime >= touchPrintInterval) {}  // Serial.printf("Touch %sdetected", (read_touch()) ? "" : "not ");
            if (nowtouch) Serial.printf(" x:%d y:%d\n", tft_touch[xx], tft_touch[yy]);
            else Serial.printf("\n");                // If available, you can print touch pressure as well (for capacitive touch)
            lastTouchPrintTime = currentTime;
        }
    }
};