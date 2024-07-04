#pragma once
#include "FunctionalInterrupt.h"
class MomentarySwitch {
  private:
    int _sw_action = swNONE;  // Flag for encoder handler to know an encoder switch action needs to be handled
    bool _timer_active = false;  // Flag to prevent re-handling long presses if the sw is just kept down
    bool _suppress_click = false;  // Flag to prevent a short click on switch release after successful long press
    bool activity_timer_keepalive = true;  // will activity on this switch be considered that the user is active?
    Timer _longPressTimer{300000};  // Used to time long button presses
  public:
    int _pin = -1;
    bool _val = false;  // Remember whether switch is being pressed
    MomentarySwitch() {}
    MomentarySwitch(int pin, bool _act_keepalive=true) : _pin(pin), activity_timer_keepalive(_act_keepalive) {}
    void set_pin(int pin) { _pin = pin; }
    void update() {
        // Read and interpret encoder switch activity. Encoder rotation is handled in interrupt routine
        // Encoder handler routines should act whenever encoder_sw_action is swSHORT or swLONG, setting it back to
        // swNONE once handled. When handling press, if encoder_long_clicked is nonzero then press is a long press
        bool myread = _val;
        do {
            myread = digitalRead(_pin);   // !value because electrical signal is active low
        } while (myread != digitalRead(_pin)); // some pins have a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure

        if (!myread) {  // if encoder sw is being pressed (switch is active low)
            if (!_val) {  // if the press just occurred
                if (activity_timer_keepalive) kick_inactivity_timer(HUMomDown);  // evidence of user activity
                _longPressTimer.reset();  // start a press timer
                _timer_active = true;  // flag to indicate timing for a possible long press
            }
            else if (_timer_active && _longPressTimer.expired()) {  // If press time exceeds long press threshold
                _sw_action = swLONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
                _timer_active = false;  // Keeps us from entering this logic again until after next sw release (to prevent repeated long presses)
                _suppress_click = true;  // Prevents the switch release after a long press from causing a short press
            }
            _val = true;  // Remember a press is in effect
        }
        else {  // if encoder sw is not being pressed
            if (_val) {
                if (activity_timer_keepalive) kick_inactivity_timer(HUMomUp);  // evidence of user activity
                if(!_suppress_click) _sw_action = swSHORT;  // if the switch was just released, a short press occurred, which must be handled
            }
            _timer_active = false;  // Allows detection of next long press event
            _val = false;  // Remember press is not in effect
            _suppress_click = false;  // End click suppression
        }
    }
    int press_event(bool autoreset = true) {
        int ret = _sw_action;
        if (autoreset) _sw_action = swNONE;
        return ret;
    }
    bool longpress(bool autoreset=true) {  // code may call this to check for long press and if so act upon it. Resets the long press if asserted
        bool ret = (_sw_action == swLONG);
        if (ret && autoreset) _sw_action = swNONE;
        return ret;
    }
    bool shortpress(bool autoreset=true) {  // code may call this to check for short press and if so act upon it. Resets the long press if asserted
        bool ret = (_sw_action == swSHORT);
        if (ret && autoreset) _sw_action = swNONE;
        return ret;
    }
    void setup(int pin=-1) {
        if (pin != -1) _pin = pin;
        pinMode(_pin, INPUT_PULLUP);
    }
    void press_reset() { _sw_action = swNONE; }
    bool val() { return _val; }
    bool* ptr() { return &_val; }
    void setLongPressTimer(int t) { _longPressTimer.set(t); }
};
class Encoder {
  private:
    enum _inputs { ENC_A=0, ENC_B=1 };
    // using panasonic-type encoder (16 detents/spin, 1 transition per detent)
    // 800us: soren's best, maybe glitched, 15000us: uncomfortably fast, 40000us: regular twist, 800000us: eeeextra slow
    int _spintime_min_us = 6000;  // Will reject spins faster than this as an attempt to debounce behavior.
    volatile int _spintime_isr_us = 100000;  // Time elapsed between last two detents
    volatile int isr_time_now;
    volatile int isr_time_last;
    volatile int _bounce_lock = ENC_B;  // Which of the encoder A or B inputs is currently untrustworthy due to bouncing 
    static const int _bounce_expire_us = 10000;  // need to let bounce lock expire to reliably catch turn events in either direction on direction reversals
    volatile int _delta = 0;  // Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 
    int _a_pin, _b_pin, _sw_pin;
    int _state = 0;
    int _spintime_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess

    // Encoder isr. note you gotta set the define EncoderPanasonicType to match your encoder type
    // If true, supports one type of cheap amazon black-pcb. 2024 one of these is mounted in the vehicle control enclosure
    //   * these guys hit a detent around each B transition, and A transitions in between detents
    //   * on an A transition, you're going CCW if the new value of A and the [now stable] B value are the same
    //   * if your encoder is the wrong kind for this algorithm, it'll act all squirrely, giving spurious double-actions etc., switch to the other algorithm
    // if false, supports a different type of cheap amazon black-pcb encoders
    //   * the A signal starts low at each detent, and goes high then back low before the next detent 
    //   * the value of the B signal on the falling edge of the A signal is high if going CW, otherwise low
    //   * if your encoder is the wrong kind for this algorithm, it'll behave all sluggish, like usually ignoring every other detent, switch to the other algorithm
    #if EncoderPanasonicType                          // eg the vehicle control box encoder
    void IRAM_ATTR _a_isr() {                         // A isr has been triggered by A signal rise or fall transition. A will now bounce for a while and can't be trusted
        isr_time_now = esp_timer_get_time();          // put some flavor in your flav
        int elapsed = isr_time_now - isr_time_last;   // note time elapsed since last valid event on A isr. might be invalid for use by outside code if this is a bounce   
        if (_bounce_lock == Encoder::ENC_A) {         // if A isr is bounce locked, B isr hasn't yet triggered since our last trigger
            if (elapsed <= _bounce_expire_us) return; // if bouncing is still a thing then bail
            val_a_isr = digitalRead(_a_pin);          // otherwise since B isr hasn't triggered, we are reversing direction and must update our own value
        }
        else val_b_isr = digitalRead(_b_pin);         // get a stable reading of B signal, unless we know B hasn't changed then skip for speed
        _spintime_isr_us = elapsed;                   // save elapsed time since last trigger, to calculate spin rate
        isr_time_last = isr_time_now;                 // reset spin rate timer for next time
        _delta += (val_a_isr == val_b_isr) ? -1 : 1;  // increment delta for each CW event and vice versa. handler in code may reset delta to 0 as it sees fit
        _bounce_lock = Encoder::ENC_A;                // bounce lock this isr to avoid retriggering, and unlock the B isr
    }
    #else                                                    // eg soren's dev board encoder
    void IRAM_ATTR _a_isr() {                                // A isr has been triggered by A signal rise or fall transition. A will now bounce for a while and can't be trusted
        if (_bounce_lock == Encoder::ENC_A) return;          // if A isr is bounce locked then bail
        if (!val_a_isr) {                                    // if the A signal is now low (i.e. recent B isr read A pin as high)
            isr_time_now = esp_timer_get_time();             // put some flavor in your flav
            _spintime_isr_us = isr_time_now - isr_time_last; // save elapsed time since last trigger, to calculate spin rate
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
    Timer twisttimer{500000};  // set to the max amount of time you can reliably expect to see w/o any detents turned in between rapid twists
    float _spinrate, _spinrate_max;  // , spinrate_accel_thresh;  // in Hz
    int _accel_factor = 1;
  public:
    MomentarySwitch button;
    bool enc_a, val_a_isr = LOW;  // if initializing HIGH sets us up right after a reboot with a bit of a hair trigger which turns left at the slightest touch
    bool enc_b, val_b_isr = HIGH;
    float _accel_max = 25.0;  // Maximum acceleration factor    
    Encoder(int a, int b, int sw) : _a_pin(a), _b_pin(b), _sw_pin(sw) { button.setup(_sw_pin); }
    Encoder() = delete; // must be instantiated with pins
    
    void setup() {
        ezread.squintf("Encoder setup..\n");
        set_pin(_a_pin, INPUT_PULLUP);
        set_pin(_b_pin, INPUT_PULLUP);
        button.setup();
        // set_pin(_sw_pin, INPUT_PULLUP);  // The esp32 pullup is too weak. Use resistor
        attachInterrupt(digitalPinToInterrupt(_a_pin), [this]{ _a_isr(); }, CHANGE); \
        attachInterrupt(digitalPinToInterrupt(_b_pin), [this]{ _b_isr(); }, CHANGE);
        #if EncoderPanasonicType  // these encoders have half the transitions/interrupts for the same amount of turn 
            _spintime_min_us >> 1;  // so double the fastest spin threshold time
            // _accel_thresh_us *= 2;  // and double the acceleration threshold time
        #endif
        _spinrate_max = 1000000.0 / (float)_spintime_min_us;
        // spinrate_accel_thresh = 1000000.0 / (float)_accel_thresh_us;
        
    }
    float spinrate() { return _spinrate; }  // in Hz for display
    float spinrate_max() { return _spinrate_max; }  // in Hz for display
    int accel_factor() { return _accel_factor; }  // for display
    int accel_max() { return (int)_accel_max; }
    void update_spinrate() {
        static int best_time_this_twist;
        static int time_last;
        static bool twist_in_progress;
        static int div_time;
        if (_spintime_isr_us != time_last) {
            div_time = _spintime_isr_us;
            #if EncoderPanasonicType  // these encoders have half the transitions/interrupts for the same amount of turn 
            div_time >> 1;  // because it takes 2 detents to get 1 A interrupt w/ these encoders, each detent took half the time
            #endif
            if (twist_in_progress) {
                if (div_time < best_time_this_twist) best_time_this_twist = div_time;
            }
            else best_time_this_twist = div_time;
            twist_in_progress = true;
        }
        if (twisttimer.expired()) {
            twist_in_progress = false;
            _spinrate = 0.0;
        }
        time_last = _spintime_isr_us;
        if (best_time_this_twist != 0) _spinrate = 1000000.0 / (float)best_time_this_twist;  // convert to Hz
        // if (best_time_this_twist < _accel_thresh_us) _accel_factor = 1;
        // else _accel_factor = (int)(map(_spinrate, spinrate_accel_thresh, _spinrate_max, 1.0, 10.0));
        _accel_factor = (int)(map(_spinrate, 0.0, _spinrate_max, 1.0, _accel_max));
        // Serial.printf("dt:%d sr:%lf af:%d\n", div_time, _spinrate, _accel_factor);
    }
    void update() {
        button.update();
        enc_a = !digitalRead(_a_pin);
        enc_b = !digitalRead(_b_pin);
        if (runmode == LOWPOWER && !syspower) {
            if (button.shortpress(), false) sleep_request = REQ_OFF;
        }
        // else if (runmode == STANDBY) {
        //     if (button.shortpress(), false) autosaver_request = REQ_OFF;
        // }
        update_spinrate();
    }
    int rotation(bool accel=true) {  // Returns detents spun since last call, accelerated by spin rate or not
        int d = _delta;
        _delta = 0;  // our responsibility to reset this flag after handling events
        if (d) kick_inactivity_timer(HUEncTurn);  // evidence of user activity            
        if (accel) d *= _accel_factor;
        return d;
    }
    int rotdirection() {  // Returns 0 if unspun, or -1 or 1 if spun, depending on which direction
        int d = _delta;
        _delta = 0;  // our responsibility to reset this flag after handling events
        if (d) kick_inactivity_timer(HUEncTurn);  // evidence of user activity            
        return constrain(d, -1, 1);
    }
    // void rezero() { _delta = 0; }  // Handling code needs to call to rezero after reading rotations
};
// #define touch_cells_v 5  // how many cells vertically
// #define touch_cells_h 6  // how many cells across
// #define touch_cell_v_pix disp_height_pix / touch_cells_v  // 48 // When touchscreen gridded as buttons, height of each button
// #define touch_cell_h_pix disp_width_pix / touch_cells_h  // 53 // When touchscreen gridded as buttons, width of each button
// #define touch_margin_h_pix (disp_width_pix - (touch_cell_h_pix * touch_cells_h)) / 2  // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen
#define touch_cell_v_pix 48  // disp_height_pix / touch_cells_v  // 48 // When touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53  // disp_width_pix / touch_cells_h  // 53 // When touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1  // (disp_width_pix - (touch_cell_h_pix * touch_cells_h)) / 2  // 1 // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen
#define touch_reticle_offset 50  // Distance of center of each reticle to nearest screen edge
#define disp_tuning_lines 15  // Lines of dynamic variables/values in dataset pages 
class Touchscreen {
  private:
    LGFX* _tft;
    I2C* _i2c;
    int corners[2][2][2] = { { { -25, -3549 }, { 185, 3839 } },  // [restouch][xx/yy][min/max]  // Read resistance values from upper-left and lower-right corners of screen, for calibration
                             { { -100, 319 },  { 0, 174 } } };   // [captouch][xx/yy][min/max]  // Read resistance values from upper-left and lower-right corners of screen, for calibration
    bool touch_longpress_valid = true;
    bool landed_coordinates_valid = false;
    bool lasttouch = false;
    int fd_exponent = 0;
    float fd = (float)(1 << fd_exponent);  // float delta
    int touch_fudge = 0;
    int fd_exponent_max = 6;
    int tlast_x, tlast_y;
    Timer touchHoldTimer{550000};  // Hold this long to count as a long press
    Timer touchAccelTimer{400000};
    Timer touchSenseTimer{15000};  // touch chip can't respond faster than some time period
    Timer keyRepeatTimer{250000};  // for editing parameters with only a few values, auto repeat is this slow
    bool touchPrintEnabled = true;  // , nowtouch = false, nowtouch2 = false;
    unsigned long lastTouchPrintTime = 0;
    const unsigned long touchPrintInterval = 500; // Adjust this interval as needed (in milliseconds)
    enum touch_axis : int { xx, yy, zz };
    enum touch_lim : int { tsmin, tsmax };
    int trow, tcol, disp_size[2], touch_read[3], tft_touch[2], landed[2];  // landed are the initial coordinates of a touch event, unaffected by subsequent dragging
    // uint16_t touch_cal_data[5] = { 404, 3503, 460, 3313, 1 };  // Got from running TFT_eSPI/examples/Generic/Touch_calibrate/Touch_calibrate.ino
    // lcd.setTouch(touch_cal_data);
    void get_touch_debounced() {  // this rejects short spurious touch or un-touch blips
        static bool rejectiontimer_active;
        static Timer rejectiontimer;             // timer for required minimum duration of new presses, or removal of old presses
        uint8_t count = _tft->getTouch(&(touch_read[xx]), &(touch_read[yy]));
        bool touch_triggered = (count > 0);
        // Serial.printf("n:%d t:%d\n", nowtouch, touch_triggered);
        if (nowtouch != touch_triggered) {       // if the hardware returned opposite our current filtered state, get triggered
            if (!rejectiontimer_active) {        // if we're not already waiting for validity
                rejectiontimer.set(25000);       // reset the timer. the touch must stay triggered for this long (in us) for valid change in touch state
                rejectiontimer_active = true;    // remember we are now triggered and waiting for validity
            }
            else if (rejectiontimer.expired()) { // levels have held through entire validity wait timeout
                rejectiontimer_active = false;   // remember we're no longer triggered nor waiting for validity
                nowtouch = touch_triggered;      // we can now consider the change in touch state valid
            }
        }
        else rejectiontimer_active = false;      // cancel our trigger, and be ready to retrigger
    }
    // void edit_up() { id = (int)fd; }
    // void edit_dn() { id = -(int)fd; }
  public:
    static constexpr uint8_t addr = 0x38;  // i2c addr for captouch panel
    int idelta, id;  // id is edit amount as integer for locally made edits. idelta is edit amount as integer for passing off to tuner object.
    bool increment_datapage = false, increment_sel = false;
    Touchscreen() {}
    void setup(LGFX* tft, I2C* i2c) {
        if (!display_enabled) return;
        _tft = tft;
        _i2c = i2c;
        disp_size[HORZ] = disp_width_pix;
        disp_size[VERT] = disp_height_pix;
        captouch = (i2c->detected(i2c_touch));
        Serial.printf("Touchscreen.. %s panel\n", (captouch) ? "detected captouch" : "using resistive");
    }
    bool touched() { return nowtouch; }
    bool* nowtouch_ptr() { return &nowtouch; }
    int touch_x() { return tft_touch[xx]; }
    int touch_y() { return tft_touch[yy]; }
    int16_t touch_pt(uint8_t axis) { return tft_touch[axis]; }
    bool ontouch() { return nowtouch && !lasttouch; }
    bool onrepeat() {
        static bool press_in_effect = false;
        bool ret = false;
        if (!nowtouch) press_in_effect = false;
        if (press_in_effect) {
            if (keyRepeatTimer.expired()) {
                keyRepeatTimer.reset();
                ret = true;  // each time timer expires send true;
            }
        }
        else if (nowtouch) {
            keyRepeatTimer.reset();
            press_in_effect = true;
            ret = true;  // on first touch send true
        }
        else press_in_effect = ret = false;  // on loss of touch send false;
        return ret;
    }
    bool longpress() {
        bool retval = touch_longpress_valid && touchHoldTimer.expired();
        if (retval) touch_longpress_valid = false;
        return retval;
    }
    int get_delta() {
        int ret = idelta;
        idelta = 0;
        return ret;
    }
    void update() {
        bool myturn = !_i2c->not_my_turn(i2c_touch); // Serial.printf("c:%d m:%d n:%d\n", captouch, myturn, nowtouch);
        if (captouch && !myturn) return;             // if (captouch && _i2c->not_my_turn(i2c_touch)) return;
        if (touchSenseTimer.expireset()) {
            get_touch_debounced();
            if (nowtouch) {
                kick_inactivity_timer(HUTouch);  // register evidence of user activity to prevent going to sleep
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
                if (ui_context != ScreensaverUI) {
                    if (touchHoldTimer.elapsed() > (fd_exponent + 1) * touchAccelTimer.timeout()) {
                        fd_exponent = constrain(fd_exponent+1, 0, fd_exponent_max);
                    }
                    fd = (float)(1 << fd_exponent); // update the touch acceleration value
                    id = (int)fd;
                    process_ui();
                    fd = 0.0;
                }
            }
            else {                                        // if not being touched
                if (lasttouch) {                          // if touch was only just now removed
                    landed_coordinates_valid = false;     // indicate touch coordinates are stale
                    fd_exponent = 0;                   // reset acceleration factor
                    fd = (float)(1 << fd_exponent); // reset touch acceleration value to 1
                }
                touchHoldTimer.reset();
                touch_longpress_valid = true;             // allow for new longpress events
            }
            id = (int)fd;
        }
        _i2c->pass_i2c_baton();
        lasttouch = nowtouch;
    }  // Serial.printf("%s", nowtouch ? "+" : "-");
    void process_ui() {
        if (!nowtouch) return;
        bool menusafe = (runmode != FLY && runmode != HOLD && runmode != CRUISE);
        
        // tbox : section screen into 6x5 cells, with touched cell encoded as a hex byte with 1st nibble = col and 2nd nibble = row
        uint16_t tbox = (constrain((landed[xx] - touch_margin_h_pix) / touch_cell_h_pix, 0, 5) << 4) | constrain((landed[yy] + touch_fudge) / touch_cell_v_pix, 0, 4);
        
        // ezread.squintf("n%dl%dv%d q%02x tx:%3d ty:%3d e%d x%d\r", nowtouch, lasttouch, landed_coordinates_valid, tbox, tft_touch[0], tft_touch[1], fd, (int)fd_exponent);
        // std::cout << "n" << nowtouch << " e" << fd << " x" << fd_exponent << "\r";
        if (tbox == 0x00 && onrepeat()) increment_datapage = true;  // Displayed dataset page can also be changed outside of simulator  // trying to prevent ghost touches we experience occasionally
        else if (tbox == 0x01) {  // Long touch to enter/exit editing mode, if in editing mode, press to change the selection of the item to edit
            if (tunctrl == OFF) {
                sel = 0;  // If entering select mode from off mode, select the first variable
                if (longpress()) tunctrl = SELECT;
            }
            else if (tunctrl == EDIT && onrepeat()) {
                tunctrl = SELECT;  // Drop back to select mode
                increment_sel = true;  // Move to the next selection
            }
            else if (tunctrl == SELECT) {
                if (ontouch()) increment_sel = true;
                else if (longpress()) tunctrl = OFF;
            }
        }
        else if (tbox == 0x02) {  // Pressed the increase value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // If just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta = id;  // If in edit mode, increase the value
        }
        else if (tbox == 0x03) {  // Pressed the decrease value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // If just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta = -id;  // If in edit mode, decrease the value
        }
        else if (tbox == 0x04 && longpress()) sim.toggle();  // Pressed the simulation mode toggle. Needs long-press
        else if (tbox == 0x20 && menusafe && longpress()) calmode_request = true;
        else if (tbox == 0x21) ezread.lookback(tune(ezread.offset, id, 0, ezread.bufferSize));
        else if (tbox == 0x22 && onrepeat()) ezread.lookback(ezread.offset + 1);
        else if (tbox == 0x23 && onrepeat()) ezread.lookback(ezread.offset - 1);
        else if (tbox == 0x24) ezread.lookback(tune(ezread.offset, -id, 0, ezread.bufferSize));
        else if (tbox == 0x30 && menusafe && longpress()) fuelpump.request(REQ_TOG);
        else if (tbox == 0x31) pressure.sim_si(tune(pressure.val(), id, pressure.opmin(), pressure.opmax()));  // Pressed the increase brake pressure button
        else if (tbox == 0x32) pressure.sim_si(tune(pressure.val(), -id, pressure.opmin(), pressure.opmax()));
        else if (tbox == 0x33) brkpos.sim_si(tune(brkpos.val(), id, brkpos.opmin(), brkpos.opmax()));
        else if (tbox == 0x34) brkpos.sim_si(tune(brkpos.val(), -id, brkpos.opmin(), brkpos.opmax()));
        else if (tbox == 0x40 && menusafe && longpress()) ignition.request(REQ_TOG);
        else if (tbox == 0x41) tach.sim_si(tune(tach.val(), id, tach.opmin(), tach.opmax()));
        else if (tbox == 0x42) tach.sim_si(tune(tach.val(), -id, tach.opmin(), tach.opmax()));
        else if (tbox == 0x43 && sim.simulating(sens::joy)) tune(&hotrc.pc[VERT][FILT], id, hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
        else if (tbox == 0x44 && sim.simulating(sens::joy)) tune(&hotrc.pc[VERT][FILT], -id, hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
        else if (tbox == 0x50 && menusafe && longpress()) sleep_request = REQ_TOG;  // sleep requests are handled by standby or lowpower mode, otherwise will be ignored
        else if (tbox == 0x51) speedo.sim_si(tune(speedo.val(), id, speedo.opmin(), speedo.opmax()));
        else if (tbox == 0x52) speedo.sim_si(tune(speedo.val(), -id, speedo.opmin(), speedo.opmax()));
        else if (tbox == 0x53 && sim.simulating(sens::joy)) tune(&hotrc.pc[HORZ][FILT], id, hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
        else if (tbox == 0x54 && sim.simulating(sens::joy)) tune(&hotrc.pc[HORZ][FILT], -id, hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
    }
    void enableTouchPrint(bool enable) {
        touchPrintEnabled = enable;
    }
    void printTouchInfo() {
        if (touchPrintEnabled && touched()) {
            unsigned long currentTime = millis();
            if (currentTime - lastTouchPrintTime >= touchPrintInterval) {}  // ezread.squintf("Touch %sdetected", (read_touch()) ? "" : "not ");
            if (nowtouch) ezread.squintf(" x:%d y:%d\n", tft_touch[xx], tft_touch[yy]);
            else ezread.squintf("\n");                // If available, you can print touch pressure as well (for capacitive touch)
            lastTouchPrintTime = currentTime;
        }
    }
};