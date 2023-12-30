
#pragma once
#define touch_cell_v_pix 48  // When touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53  // When touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1  // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen
#define touch_reticle_offset 50  // Distance of center of each reticle to nearest screen edge
#define disp_tuning_lines 11  // Lines of dynamic variables/values in dataset pages 
class Touchscreen {
  private:
    LGFX* _tft;
    I2C* _i2c;
    int32_t corners[2][2] = { { -25, -3549 }, { 185, 3839 } };  // [xx/yy][min/max]  // Read resistance values from upper-left and lower-right corners of screen, for calibration
    bool touch_longpress_valid = true;
    bool landed_coordinates_valid = false;
    bool touch_now_touched = false;
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
    int trow, tcol, disp_size[2], touch_read[2], tft_touch[2], landed[2];  // landed are the initial coordinates of a touch event, unaffected by dragging
  public:
    static constexpr uint8_t addr = 0x38;  // i2c addr for captouch panel
    int idelta = 0;
    Touchscreen() {}
    void setup(LGFX* tft, I2C* i2c, int width, int height) {
        _tft = tft;
        _i2c = i2c;
        disp_size[HORZ] = width;
        disp_size[VERT] = height;
        captouch = (_i2c->detected(i2c_touch));
        _tft->touch_init();  // this points touch object to resistive or capacitive driver instance based on captouch
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
            nowtouch = count;
            if (nowtouch) {
                for (int axis=0; axis<=1; axis++) {
                    if (captouch) tft_touch[axis] = touch_read[axis];  // disp_width - 1 - touch_read[xx];
                    else tft_touch[axis] = map(touch_read[axis], corners[axis][tsmin], corners[axis][tsmax], 0, disp_size[axis]);  // translate resistance to pixels
                    tft_touch[axis] = constrain(tft_touch[axis], 0, disp_size[axis] - 1);
                    if (flip_the_screen) tft_touch[axis] = disp_size[axis] - tft_touch[axis];
                    if (!landed_coordinates_valid) {
                        landed[axis] = tft_touch[axis];
                        if (axis) landed_coordinates_valid = true;  // on 2nd time thru set this true
                    }
                }
            }
            else landed_coordinates_valid = false;
            // Serial.printf("n:%d c:%d rx:%d ry:%d tx:%d ty:%d\n", nowtouch, captouch, touch_read[0], touch_read[1], tft_touch[0], tft_touch[1]);
        }
        _i2c->pass_i2c_baton();
        return nowtouch;
    }
    void update() {
        read_touch();
        if (!nowtouch) {  // if not being touched
            idelta = 0;  // Stop changing the value
            if (touch_now_touched) touchDoublePressTimer.reset();  // Upon end of a touch, begin timer to reject any accidental double touches
            touch_now_touched = false;  // Remember the last touch state
            tedit_exponent = 0;
            tedit = (float)(1 << tedit_exponent); // Reset touch acceleration value to 1
            touchHoldTimer.reset();
            touch_longpress_valid = true;
            return;
        }
        // if (touchDoublePressTimer.expired()) {
        tedit = (float)(1 << tedit_exponent);
        //Serial.printf("(%d,%d), landed(%d,%d)\n",tft_touch[xx],tft_touch[yy],landed[xx],landed[yy]);
        sleep_inactivity_timer.reset();  // evidence of user activity
        tedit = (float)(1 << tedit_exponent);  // Determine value editing rate
        trow = constrain((landed[yy] + touch_fudge) / touch_cell_v_pix, 0, 4);
        tcol = constrain((landed[xx] - touch_margin_h_pix) / touch_cell_h_pix, 0, 5);
        if (tcol == 0 && trow == 0 && !touch_now_touched) {
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
            else if (tunctrl == EDIT && !touch_now_touched) {
                tunctrl = SELECT;  // Drop back to select mode
                sel_val++;  // Move to the next selection
            }
            else if (tunctrl == SELECT) {
                if (!touch_now_touched) sel_val = (sel_val + 1) % disp_tuning_lines;
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
            else if (tcol == 3 && trow == 0 && sim.can_sim(sens::basicsw) && !touch_now_touched) basicmodesw = !basicmodesw;
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
        touch_now_touched = true;
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