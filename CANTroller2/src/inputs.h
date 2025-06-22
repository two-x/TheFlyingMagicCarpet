#pragma once
#include "FunctionalInterrupt.h"
class MomentarySwitch {
  private:
    int _sw_action = swNONE;               // flag for encoder handler to know an encoder switch action needs to be handled
    bool _timer_active = false;            // flag to prevent re-handling long presses if the sw is just kept down
    bool _suppress_click = false;          // flag to prevent a short click on switch release after successful long press
    bool activity_timer_keepalive = true;  // will activity on this switch be considered that the user is active?
    Timer _longpressTimer{300000};         // used to time long button presses
  public:
    int _pin = -1;
    bool _val = false;                     // remember whether switch is being pressed
    MomentarySwitch() {}
    MomentarySwitch(int pin, bool _act_keepalive=true) : _pin(pin), activity_timer_keepalive(_act_keepalive) {}
    void set_pin(int pin) { _pin = pin; }
    void update() {
        // read and interpret encoder switch activity. encoder rotation is handled in interrupt routine
        // encoder handler routines should act whenever encoder_sw_action is swSHORT or swLONG, setting it back to
        // swNONE once handled. When handling press, if encoder_long_clicked is nonzero then press is a long press
        bool myread = _val;
        do {
            myread = !digitalRead(_pin);       // !value because electrical signal is active low
        } while (myread == digitalRead(_pin)); // some pins have a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure

        if (myread) {     // if encoder sw is being pressed (switch is active low)
            if (!_val) {  // if the press just occurred
                if (activity_timer_keepalive) kick_inactivity_timer(HUMomDown);  // evidence of user activity
                _longpressTimer.reset();  // start a press timer
                _timer_active = true;     // flag to indicate timing for a possible long press
            }
            else if (_timer_active && _longpressTimer.expired()) {  // if press time exceeds long press threshold
                _sw_action = swLONG;    // set flag to handle the long press event. note, routine handling press should clear this
                _timer_active = false;  // keeps us from entering this logic again until after next sw release (to prevent repeated long presses)
                _suppress_click = true; // prevents the switch release after a long press from causing a short press
            }
        }
        else {  // if encoder sw is not being pressed
            if (_val) {
                if (activity_timer_keepalive) kick_inactivity_timer(HUMomUp);  // evidence of user activity
                if(!_suppress_click) _sw_action = swSHORT;  // if the switch was just released, a short press occurred, which must be handled
            }
            _timer_active = false;   // allows detection of next long press event
            _suppress_click = false; // end click suppression
        }
        _val = myread;               // remember current state
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
    void setLongPressTimer(int t) { _longpressTimer.set(t); }
};
class Encoder {
  private:
    enum _inputs { ENC_A=0, ENC_B=1 };
    Timer activitytimer{300000};
    // using panasonic-type encoder (16 detents/spin, 1 transition per detent)
    // 800us: soren's best, maybe glitched, 15000us: uncomfortably fast, 40000us: regular twist, 800000us: eeeextra slow
    int _spintime_min_us = 6000;  // will reject spins faster than this as an attempt to debounce behavior.
    volatile int _spintime_isr_us = 100000;  // time elapsed between last two detents
    volatile int isr_time_now;
    volatile int isr_time_last;
    volatile int _bounce_lock = ENC_B;           // which of the encoder A or B inputs is currently untrustworthy due to bouncing 
    static const int _bounce_expire_us = 10000;  // need to let bounce lock expire to reliably catch turn events in either direction on direction reversals
    volatile int _delta = 0;                     // keeps track of un-handled rotary clicks of the encoder. positive for CW clicks, Negative for CCW. 
    int _a_pin, _b_pin, _sw_pin, _state = 0, _spintime_us = 1000000;  // how many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
    bool activity = false;
    // encoder isr. note you gotta set the define EncoderPanasonicType to match your encoder type
    // if true, supports one type of cheap amazon black-pcb. 2024 one of these is mounted in the vehicle control enclosure
    //   * these guys hit a detent around each B transition, and A transitions in between detents
    //   * on an A transition, you're going CCW if the new value of A and the [now stable] B value are the same
    //   * if your encoder is the wrong kind for this algorithm, it'll act all squirrely, giving spurious double-actions etc., switch to the other algorithm
    // if false, supports a different type of cheap amazon black-pcb encoders
    //   * the A signal starts low at each detent, and goes high then back low before the next detent 
    //   * the value of the B signal on the falling edge of the A signal is high if going CW, otherwise low
    //   * if your encoder is the wrong kind for this algorithm, it'll behave all sluggish, like usually ignoring every other detent, switch to the other algorithm
    #if EncoderPanasonicType                          // eg the vehicle control box encoder
    void IRAM_ATTR _a_isr() {                         // A isr has been triggered by A signal rise or fall transition. A will now be bouncing for a while and can't be trusted
        isr_time_now = esp_timer_get_time();          // put some flavor in your flav
        int elapsed = isr_time_now - isr_time_last;   // note time elapsed since last valid event on A isr. might be invalid for use by outside code if this is a bounce   
        if (_bounce_lock == Encoder::ENC_A) {         // if A isr is bounce locked, B isr hasn't yet triggered since our last trigger
            if (elapsed <= _bounce_expire_us) return; // if it's not yet been long enough since the initial transition we assume this is a bounce and just bail
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
    Timer twisttimer{500000};        // set to the max amount of time you can reliably expect to see w/o any detents turned in between rapid twists
    float _spinrate, _spinrate_max;  // , spinrate_accel_thresh;  // in Hz
    int _accel_factor = 1;
  public:
    MomentarySwitch button;
    bool enc_a, val_a_isr = LOW;  // initializing HIGH sets us up right after a reboot with a bit of a hair trigger which turns left at the slightest touch
    bool enc_b, val_b_isr = HIGH;
    float _accel_max = 25.0;      // maximum acceleration factor    
    Encoder(int a, int b, int sw) : _a_pin(a), _b_pin(b), _sw_pin(sw) { button.setup(_sw_pin); }
    Encoder() = delete;           // must be instantiated with pins
    
    void setup() {
        ezread.squintf("Encoder setup..\n");
        set_pin(_a_pin, INPUT_PULLUP);
        set_pin(_b_pin, INPUT_PULLUP);
        button.setup();
        // set_pin(_sw_pin, INPUT_PULLUP);  // the esp32 pullup is too weak. Use resistor
        attachInterrupt(digitalPinToInterrupt(_a_pin), [this]{ _a_isr(); }, CHANGE); \
        attachInterrupt(digitalPinToInterrupt(_b_pin), [this]{ _b_isr(); }, CHANGE);
        #if EncoderPanasonicType    // these encoders have half the transitions/interrupts for the same amount of turn 
            _spintime_min_us >> 1;  // so double the fastest spin threshold time
            // _accel_thresh_us *= 2;  // and double the acceleration threshold time
        #endif
        _spinrate_max = 1000000.0 / (float)_spintime_min_us;
        // spinrate_accel_thresh = 1000000.0 / (float)_accel_thresh_us;
        
    }
    float spinrate() { return _spinrate; }          // in Hz for display
    float spinrate_max() { return _spinrate_max; }  // in Hz for display
    int accel_factor() { return _accel_factor; }    // for display
    int accel_max() { return (int)_accel_max; }
    void update_spinrate() {
        static int best_time_this_twist, time_last, div_time;
        static bool twist_in_progress;
        if (_spintime_isr_us != time_last) {
            div_time = _spintime_isr_us;
            #if EncoderPanasonicType  // these encoders have half the transitions/interrupts for the same amount of turn 
            div_time >> 1;            // because it takes 2 detents to get 1 A interrupt w/ these encoders, ie each detent took half the time
            #endif
            if (!twist_in_progress || (div_time < best_time_this_twist)) best_time_this_twist = div_time;
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
        static int delta_last;
        button.update();
        enc_a = !digitalRead(_a_pin);
        enc_b = !digitalRead(_b_pin);
        // if (runmode == LOWPOWER || runmode == STANDBY) {
        //     if (button.shortpress(false)) {
        //         sleep_request = REQ_OFF;
        //         autosaver_request = REQ_OFF;
        //     }
        // }
        if (_delta != delta_last) activitytimer.reset();
        activity = !activitytimer.expired();
        delta_last = _delta;
        update_spinrate();
    }
    bool* activity_ptr() { return &activity; }
    int rotation(bool accel=true) {  // returns detents spun since last call, accelerated by spin rate or not
        int d = _delta;
        _delta = 0;  // our responsibility to reset this flag after queries
        if (d) kick_inactivity_timer(HUEncTurn);  // register evidence of user activity
        if (accel) d *= _accel_factor;
        return d;
    }
    int rotdirection() {  // returns 0 if unspun, -1 if spun CCW, or 1 if spun CW
        int d = _delta;
        _delta = 0;       // our responsibility to reset this flag after queries
        if (d) kick_inactivity_timer(HUEncTurn);  // register evidence of user activity
        return constrain(d, -1, 1);
    }
    // void rezero() { _delta = 0; }  // handling code needs to call to rezero after reading rotations
};
#define touch_cell_v_pix 48      // disp_height_pix / touch_cells_v  // 48 // when touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53      // disp_width_pix / touch_cells_h  // 53 // when touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1     // (disp_width_pix - (touch_cell_h_pix * touch_cells_h)) / 2  // 1 // on horizontal axis, we need an extra margin along both sides button sizes to fill the screen
#define touch_reticle_offset 50  // distance of center of each reticle to nearest screen edge
#define disp_tuning_lines 15     // lines of dynamic variables/values in dataset pages

// touchscreen driver:
// * panels supported: i2c capacitive or spi resistive touchscreen, autodetected based on i2c address of captouch panel
// * calibration: values of corners[][][] array must be edited for optimized location accuracy (for cap and res types)
// * filtering: rejects short spurious false touches or losses of touch (needed for use through plastic box lid)
// * external methods supported:
//    touched()     : returns true if a touch is in progress, otherwise false
//    touch_pt(xy)  : returns coordinate of the point currently or most recently touched, along the given axis.  value is held until the next touch event begins
//    landed_pt(xy) : returns coordinate of the initial touch point of the current or most recent touch event, along the given axis.  value is held until the next touch event begins
//    tap()         : returns true if a touch is removed quickly without significant drag, and then not quickly re-touched.  querying or staleness timeout resets to false
//    doubletap()   : returns true if a touch is removed quickly, then quickly re-touched and again removed quickly, without significant drag.  querying or staleness timeout resets to false
//    swipe()       : returns the dominant orthogonal direction of any valid swipe event detected, or otherwise DirNone.  a valid swipe is a touch that's dragged far enough then released soon enough.  querying or staleness timeout resets to DirNone 
//    held()        : returns true if an in-progress touch persists until valid taps and swipes are no longer possible, without experiencing any.  resets to false upon release
//    longpress()   : returns true if an in-progress touch persists long enough without significant drag.  resets to false upon release or query, and won't retrigger during the same touch event
//    drag()        : returns diagonal distance dragged in pixels, of the current or most recent touch event.  doesn't reset until the next touch event begins
//    drag(xy)      : returns orthogonal distance dragged in pixels along the given axis, of the current or most recent touch event.  doesn't reset until the next touch event begins
//    get_delta()   : (for editing)  returns a value which goes 0 -> 1 upon touch, and continues to increment at an exponentially increasing rate (up to a max) as long as the touch persists. value becomes 0 on release or after query (query preserves acceleration)
//    onrepeat()    : (for editing)  returns true periodically on fixed intervals while a touch event persists.  resets to false on release, or when queried (until the next interval)

class Touchscreen {
  private:
    I2C* _i2c;
    LGFX* _tft;
    int corners[2][2][2] = { { { -25, -3549 }, { 185, 3839 } },  // [restouch][HORZ/VERT][min/max]  // read resistance values from upper-left and lower-right corners of screen, for calibration
                             { { -100, 319 },  { 0, 174 } } };   // [captouch][HORZ/VERT][min/max]  // read resistance values from upper-left and lower-right corners of screen, for calibration
    bool longpress_possible = true, recent_tap = false, doubletap_possible = false;  // ts_tapped = false, ts_doubletapped = false, ts_longpressed = false;
    bool lasttouch = false, printEnabled = true, swipe_possible = true;  // , nowtouch = false, nowtouch2 = false;
    int fd_exponent = 0, fd_exponent_max = 6, tlast_x, tlast_y, keyrepeats = 0;
    float fd = (float)(1 << fd_exponent);  // float delta

    // timing requirements: 1) sense < filter < (twotap & repeat & accel).  2) (twotap & swipe) < longpress < press. 
    Timer senseTimer{6500};        // touch chip can't respond faster than some time period.
    Timer filterTimer{10000};       // touch or untouch events lasting less than this long are ignored. needed for using through plastic box lid
    Timer twotapTimer{180000};      // two tap events within this much time is a double tap
    Timer pressTimer{750000};       // taps/presses/swipes will expire if not queried within this timeframe after occurrence
    // below are all based on pressTimer
    int repeat_timeout = 250000;    // for editing parameters with only a few values, auto repeat is this slow
    int swipe_timeout = 300000;     // to count as a swipe, a drag must end before this long
    int accel_timeout = 400000;     // how long between each increase of edit acceleration level
    int longpress_timeout = 550000; // hold down this long to count as a long press

    enum touch_lim : int { tsmin, tsmax };
    int trow, tcol, disp_size[2], raw[3], tft_touch[2], landed[2];  // landed are the initial coordinates of a touch event, unaffected by subsequent dragging
    // uint16_t touch_cal_data[5] = { 404, 3503, 460, 3313, 1 };  // got from running TFT_eSPI/examples/Generic/Touch_calibrate/Touch_calibrate.ino
    // lcd.setTouch(touch_cal_data);
    void get_touch_debounced() {  // this updates nowtouch, rejecting any spurious touch or un-touch blips shorter than filterTimer timeout
        static bool filtertimer_active;
        uint8_t count = _tft->getTouch(&(raw[HORZ]), &(raw[VERT]));
        bool triggered = (count > 0);
        // Serial.printf("n:%d t:%d\n", nowtouch, touch_triggered);
        if (nowtouch != triggered) {            // if the hardware returned opposite our current filtered state, get triggered
            if (!filtertimer_active) {       // if we're not already waiting for validity
                filterTimer.reset();    // reset the timer. the touch must stay triggered for this long (in us) for valid change in touch state
                filtertimer_active = true;   // remember we are now triggered and waiting for validity
            }
            else if (filterTimer.expired()) { // levels have held through entire validity wait timeout
                filtertimer_active = false;   // remember we're no longer triggered nor waiting for validity
                nowtouch = triggered;            // we can now consider the change in touch state valid
            }
        }
        else filtertimer_active = false;      // cancel our trigger, and be ready to retrigger
    }
    int drag_axis(int axis) {  // for internal use only. returns pixels dragged along a given axis since start of current drag event. result is vertically flipped so up is positive
        int ret = tft_touch[axis] - landed[axis];
        if (axis == VERT) ret *= -1;  //  if (axis == (flip_the_screen ? HORZ : VERT)) ret *= -1;
        return ret;
    }
    bool ontouch() { return nowtouch && !lasttouch; }  // returns true only if touch is new for this update cycle. only reliable if called internally, otherwise use tap()
    bool onrelease() { return !nowtouch && lasttouch; }  // returns true only if release is new for this update cycle. only reliable if called internally, otherwise use tap()
    void update_swipe() {  // determines if there's a valid swipe, saved to variable
        int _dragged[2] = { drag(HORZ), drag(VERT) };
        int _axis = (std::abs(_dragged[HORZ]) > std::abs(_dragged[VERT])) ? HORZ : VERT;  // was swipe mostly horizontal or mostly vertical
        if (!swipe_possible || (std::abs(_dragged[_axis] < swipe_min))) ts_swipedir = DirNone;  // catch if swipe doesn't qualify
        else if (_axis == HORZ) ts_swipedir = (_dragged[HORZ] > 0) ? DirRight : DirLeft;
        else ts_swipedir = (_dragged[VERT] > 0) ? DirUp : DirDown;
        ts_swiped = (ts_swipedir != DirNone);  // for debug idiot light
    }
  public:
    static constexpr uint8_t addr = 0x38;  // i2c addr for captouch panel
    int drag_dist_min = 20;  // if a press changes location more than this many pixels, it is considered a drag not a press
    int swipe_min = 50;     // minimum travel in pixels to count as a swipe
    int idelta, id;         // id is edit amount as integer for locally made edits. idelta is edit amount as integer for passing off to tuner object.
    bool increment_datapage = false, increment_sel = false;
    Touchscreen() {}
    void setup(LGFX* tft, I2C* i2c) {
        if (!display_enabled) return;
        _tft = tft;
        _i2c = i2c;
        disp_size[HORZ] = disp_width_pix;
        disp_size[VERT] = disp_height_pix;
        captouch = (i2c->detected(i2c_touch));
        Serial.printf("Touchscreen.. %s panel", (captouch) ? "detected captouch" : "using resistive");
        if (   (senseTimer.timeout() >= filterTimer.timeout()) || (filterTimer.timeout() >= twotapTimer.timeout())  // checks all the timeouts are in length order
            || (filterTimer.timeout() >= repeat_timeout)       || (filterTimer.timeout() >= accel_timeout)
            || (twotapTimer.timeout() >= longpress_timeout)    || (swipe_timeout >= longpress_timeout)
            || (longpress_timeout >= pressTimer.timeout()) )   Serial.printf(" ERR in timing setup!");
        Serial.printf("\n");
    }
    // bool* touched_ptr() { return &nowtouch; }
    int touch_x() { return tft_touch[HORZ]; }
    int touch_y() { return tft_touch[VERT]; }
    int touch_pt(int axis) { return tft_touch[axis]; }  // returns coordinate along given axis of last-read touch point (in pixels)
    int landed_pt(int axis) { return landed[axis]; }    // returns coordinate along given axis of the initial touch point for latest touch event (in pixels)
    bool touched() { return nowtouch; }  // returns whether a touch is currently in progress
    bool held() { return (nowtouch && twotapTimer.expired() && !ts_tapped && !ts_doubletapped); } // true if press is held longer than the double-tap validity timeout
    bool onrepeat() {  // if touch is held down, this will return true periodically on consistent key repeat intervals like a pc keyboard
        if (ontouch()) return true;
        if (!nowtouch || (pressTimer.elapsed() / repeat_timeout <= keyrepeats)) return false;
        keyrepeats++;
        return true;
    }
    int get_delta(bool reset=true) {  // returns edit value (for tuning) based on length of press and acceleration factor
        int ret = idelta;
        if (reset) idelta = 0;
        return ret;
    }
    // w/o argument, returns diagonal distance of an in-progress drag since its start.
    // w/ a valid axis as argument, returns orthoganal distance dragged along the given axis. result is vertically flipped so up is positive
    int drag(int axis=-1) {
        if (!nowtouch) return 0;  // return 0 if not being touched. indistinguishable from fixed press
        if (axis == HORZ || axis == VERT) return drag_axis(axis);  // if a valid axis given
        float _dragged[2] = { (float)drag_axis(HORZ), (float)drag_axis(VERT) };
        return (int)(std::sqrt(_dragged[HORZ] * _dragged[HORZ] + _dragged[VERT] * _dragged[VERT]));  // pythagorean theorem
    }
    int swipe(bool reset=true) {  // returns direction of any valid orthogonal tinder-type swipe, otherwise 0 for no swipe
        int ret = ts_swipedir;
        if (reset) {
            ts_swipedir = DirNone;
            ts_swiped = false;
        }
        return ret;
    }
    bool tap(bool reset=true) {  // returns if a single-tap has occurred
        bool ret = ts_tapped;
        if (reset) ts_tapped = false;
        return ret;
    }
    bool doubletap(bool reset=true) {  // returns if a double-tap has occurred
        bool ret = ts_doubletapped;
        if (reset) ts_doubletapped = false;
        return ret;
    }
    bool longpress(bool reset=true) {  // returns true if a touch is held down longer than a timeout without any significant amount of drag
        bool ret = ts_longpressed;
        if (reset) ts_longpressed = false;
        return ret;    
    }    
    // bool* tap_ptr() { return &ts_tapped; }  // for idiot light
    // bool* doubletap_ptr() { return &ts_doubletapped; }  // for idiot light
    // bool* longpress_ptr() { return &ts_longpressed; }
    void update() {
        if (captouch && _i2c->not_my_turn(i2c_touch)) return;             // if (captouch && _i2c->not_my_turn(i2c_touch)) return;
        if (senseTimer.expireset()) {
            get_touch_debounced();
            if (nowtouch) process_touched();
            else process_untouched();
            if (pressTimer.expired()) {
                ts_tapped = ts_doubletapped = recent_tap = doubletap_possible = ts_swiped = false;
                ts_swipedir = DirNone;
            }
            id = (int)fd;
            lasttouch = nowtouch;
            // printTouchInfo();  // for debug
        }
        _i2c->pass_i2c_baton();
    }  // Serial.printf("%s", nowtouch ? "+" : "-");
  private:
    void process_touched() {  // executes on update whenever screen is being touched (nowtouch == true)
        kick_inactivity_timer(HUTouch);  // register evidence of user activity to prevent going to sleep
        for (int axis=HORZ; axis<=VERT; axis++) {
            // if (captouch) tft_touch[axis] = raw[axis];  // disp_width - 1 - raw[HORZ];
            tft_touch[axis] = map(raw[axis], corners[captouch][axis][tsmin], corners[captouch][axis][tsmax], 0, disp_size[axis]);
            tft_touch[axis] = constrain(tft_touch[axis], 0, disp_size[axis] - 1);
            if (flip_the_screen) tft_touch[axis] = disp_size[axis] - tft_touch[axis];
        }
        if (!lasttouch) {                      // if this touch only just now started
            for (int axis=HORZ; axis<=VERT; axis++) landed[axis] = tft_touch[axis];
            pressTimer.reset();  // start press timer for timing events and event stale-ness
            swipe_possible = longpress_possible = true;
            doubletap_possible = recent_tap;  // if there was just a tap, flag the current touch might be a double tap
        }
        else {            // if this touch is continuing from previous loop(s)
            if (longpress_possible && (pressTimer.elapsed() >= longpress_timeout)) {
                if (drag() <= drag_dist_min) ts_longpressed = true;
                longpress_possible = false;  // prevent a later longpress event in case drag returns to the original spot
            }
            if (pressTimer.elapsed() > swipe_timeout) swipe_possible = false;  // prevent meandering drags from being considered swipes
        }
        if (!auto_saver_enabled) {  // if (ui_context != ScreensaverUI)
            if (pressTimer.elapsed() > (fd_exponent + 1) * accel_timeout) {
                fd_exponent = constrain(fd_exponent+1, 0, fd_exponent_max);
            }
            fd = (float)(1 << fd_exponent); // update the touch acceleration value
            id = (int)fd;
            process_ui();
            fd = 0.0;
        }
    }
    void process_untouched() {  // executes on update whenever screen is not being touched (nowtouch == false)
        if (lasttouch) {                      // if touch was only just now released
            fd_exponent = 0;                  // reset acceleration factor
            fd = (float)(1 << fd_exponent);   // reset touch acceleration value to 1
            if (pressTimer.elapsed() < longpress_timeout) {      // if press duration was in short-press range
                if (doubletap_possible) ts_doubletapped = true;  // if there was a previous recent tap when pressed, we have a valid double tap
                else {
                    recent_tap = true;        // otherwise we have a potentially valid single tap
                    twotapTimer.reset();      // begin timeout again to wait for a second tap
                }
                update_swipe();
                swipe_possible = doubletap_possible = false;
            }
            ts_longpressed = longpress_possible = false;
            keyrepeats = 0;
        }
        if (twotapTimer.expired()) {
            ts_tapped = recent_tap;  // if no double tap happened in time, then recent short press becomes valid single tap
            recent_tap = doubletap_possible = false;
        }
        // for (int axis=HORZ; axis<=VERT; axis++) tft_touch[axis] = landed[axis] = -1;  // set coordinates to illegal value
    }
    void process_ui() {  // takes actions when screen objects are manipulated by touch
        // tbox : section screen into 6x5 cells, with touched cell encoded as a hex byte with 1st nibble = col and 2nd nibble = row, ie 0x32 means cell at 4th col and 3rd row
        uint16_t tbox = (constrain((landed[HORZ] - touch_margin_h_pix) / touch_cell_h_pix, 0, 5) << 4) | constrain((landed[VERT]) / touch_cell_v_pix, 0, 4);
        
        // ezread.squintf("n%dl%dv%d q%02x tx:%3d ty:%3d e%d x%d\r", nowtouch, lasttouch, landed_coordinates_valid, tbox, tft_touch[0], tft_touch[1], fd, (int)fd_exponent);
        // std::cout << "n" << nowtouch << " e" << fd << " x" << fd_exponent << "\r";
        if (tbox == 0x00 && onrepeat()) increment_datapage = true;  // displayed dataset page can also be changed outside of simulator  // trying to prevent ghost touches we experience occasionally
        else if (tbox == 0x01) {  // long touch to enter/exit editing mode, if in editing mode, press to change the selection of the item to edit
            if (tunctrl == OFF) {
                sel = 0;          // if entering select mode from off mode, select the first variable
                if (longpress()) tunctrl = SELECT;
            }
            else if (tunctrl == EDIT && onrepeat()) {
                tunctrl = SELECT;      // drop back to select mode
                increment_sel = true;  // move to the next selection
            }
            else if (tunctrl == SELECT) {
                if (ontouch()) increment_sel = true;
                else if (longpress()) tunctrl = OFF;
            }
        }
        else if (tbox == 0x02) {  // pressed the increase value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // if just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta = id;  // if in edit mode, increase the value
        }
        else if (tbox == 0x03) {  // pressed the decrease value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;   // if just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta = -id;  // if in edit mode, decrease the value
        }
        else if (tbox == 0x04 && longpress()) autosaver_request = REQ_ON;  // start fullscreen screensaver.  This button may be hijacked for a more useful function
        else if (tbox == 0x20 && longpress()) calmode_request = true;
        else if (tbox == 0x21) ezread.lookback(tune(ezread.offset, id, 0, ezread.bufferSize));  // fast scroll
        else if (tbox == 0x22 && onrepeat()) ezread.lookback(ezread.offset + 1);  // step scroll
        else if (tbox == 0x23 && onrepeat()) ezread.lookback(ezread.offset - 1);  // step scroll
        else if (tbox == 0x24) ezread.lookback(tune(ezread.offset, -id, 0, ezread.bufferSize));  // fast scroll
        else if (tbox == 0x30 && longpress()) sim.toggle();  // pressed the simulation mode toggle. Needs long-press
        else if (tbox == 0x31) pressure.sim_si(tune(pressure.val(), id, pressure.opmin(), pressure.opmax()));  // pressed the increase brake pressure button
        else if (tbox == 0x32) pressure.sim_si(tune(pressure.val(), -id, pressure.opmin(), pressure.opmax()));
        else if (tbox == 0x33) brkpos.sim_si(tune(brkpos.val(), id, brkpos.opmin(), brkpos.opmax()));
        else if (tbox == 0x34) brkpos.sim_si(tune(brkpos.val(), -id, brkpos.opmin(), brkpos.opmax()));
        else if (tbox == 0x40 && longpress()) hotrc.sim_button_press(CH4);  // sleep requests are handled by standby or lowpower mode, otherwise will be ignored
        else if (tbox == 0x41) tach.sim_si(tune(tach.val(), id, tach.opmin(), tach.opmax()));
        else if (tbox == 0x42) tach.sim_si(tune(tach.val(), -id, tach.opmin(), tach.opmax()));
        else if (tbox == 0x43 && sim.simulating(sens::joy)) tune(&hotrc.pc[VERT][FILT], id, hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
        else if (tbox == 0x44 && sim.simulating(sens::joy)) tune(&hotrc.pc[VERT][FILT], -id, hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
        else if (tbox == 0x50 && longpress()) ignition.request(REQ_TOG);
        else if (tbox == 0x51) speedo.sim_si(tune(speedo.val(), id, speedo.opmin(), speedo.opmax()));
        else if (tbox == 0x52) speedo.sim_si(tune(speedo.val(), -id, speedo.opmin(), speedo.opmax()));
        else if (tbox == 0x53 && sim.simulating(sens::joy)) tune(&hotrc.pc[HORZ][FILT], id, hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
        else if (tbox == 0x54 && sim.simulating(sens::joy)) tune(&hotrc.pc[HORZ][FILT], -id, hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
    }
    void printTouchInfo() {
        static bool last1tap, last2tap, lastlongtap;
        static int lastswipe, lastx, lasty, lastdragx, lastdragy;
        if (!last1tap && ts_tapped) ezread.squintf("ts: single-tap\n");
        if (!last2tap && ts_doubletapped) ezread.squintf("ts: double-tap\n");
        if (!lastlongtap && ts_longpressed) ezread.squintf("ts: longpress\n");
        if (lastswipe != ts_swipedir) ezread.squintf("ts: swipe=%d\n", ts_swipedir);
        if (lastdragx != drag(HORZ) || lastdragy != drag(VERT)) ezread.squintf("ts: drag %+3d,%+3d d:%+3d\n", drag(HORZ), drag(VERT), drag());
        last1tap = ts_tapped;
        last2tap = ts_doubletapped;
        lastlongtap = ts_longpressed;
        lastswipe = ts_swipedir;
        lastx = tft_touch[HORZ];
        lasty = tft_touch[VERT];
        lastdragx = drag(HORZ);
        lastdragy = drag(VERT);
    }
};