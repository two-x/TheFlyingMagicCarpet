
#pragma once
#include <XPT2046_Touchscreen.h>
class TouchScreen {
private:
    XPT2046_Touchscreen _ts;  // 3.5in resistive touch panel on tft lcd
    bool touch_longpress_valid = true;
    bool touch_now_touched = false;
    int32_t tedit_exponent = 0;
    int32_t tedit = 1 << tedit_exponent;
    int32_t touch_fudge = 0;
    int32_t tedit_exponent_max = 8;

    Timer touchHoldTimer{550000};  // Hold this long to count as a long press
    Timer touchAccelTimer{850000};
    Timer touchDoublePressTimer{40000};  // Won't allow a new press within this long after an old press (prevent accidental double clicks)

    // debug printing
    bool touchPrintEnabled = true;
    unsigned long lastTouchPrintTime = 0;
    const unsigned long touchPrintInterval = 500; // Adjust this interval as needed (in milliseconds)
    
    enum touch_axis { xx, yy, zz };
    enum touch_lim { tsmin, tsmax };
    int32_t trow, tcol;
    int32_t tft_touch[2];

    // These values need to be calibrated to each individual display panel for best accuracy
    int32_t corners[2][2] = { { 351, 3928 }, { 189, 3950 } };  // [xx/yy][min/max]
    // Soren's breadboard "" { { 351, 3933 }, { 189, 3950 } };  // [xx/yy][min/max]

    TS_Point touchpoint;
public:
    TouchScreen(uint8_t csPin, uint8_t irqPin = 255) : _ts(csPin, irqPin) {}
    
    void init() {
        printf("touchscreen..\n");
        _ts.begin();
        // _ts.setRotation(1); do we need to rotate?
    }

    bool touched() {
        return _ts.touched(); 
    }

    TS_Point getPoint() {
        return _ts.getPoint();
    }
    int16_t getX() {
        return getPoint().x;
    }

    int16_t getY() {
        return getPoint().y; 
    }
    
    int16_t touch_pt(uint8_t axis) { return tft_touch[axis]; }

    bool get_touchpoint() { 
        bool ret = false;
        if (touched() && touchDoublePressTimer.expired()) {
            ret = true;
            tedit = 1 << tedit_exponent;
            touchpoint = getPoint();
            tft_touch[xx] = map(touchpoint.x, corners[xx][tsmin], corners[xx][tsmax], 0, disp_width_pix);
            tft_touch[yy] = map(touchpoint.y, corners[yy][tsmin], corners[yy][tsmax], 0, disp_height_pix);
            tft_touch[xx] = constrain(tft_touch[xx], 0, disp_width_pix);
            tft_touch[yy] = constrain(tft_touch[yy], 0, disp_height_pix);
            if (!flip_the_screen) { 
                tft_touch[xx] = disp_width_pix - tft_touch[xx];
                tft_touch[yy] = disp_height_pix - tft_touch[yy];
            }
        }
        return ret;
    }
    void update() {
        if (!get_touchpoint()) { // If not being touched, put momentarily-set simulated button values back to default values
            idelta_touch = 0;  // Stop changing the value
            if (touch_now_touched) touchDoublePressTimer.reset();  // Upon end of a touch, begin timer to reject any accidental double touches
            touch_now_touched = false;  // Remember the last touch state
            tedit_exponent = 0;
            tedit = 1 << tedit_exponent; // Reset touch acceleration value to 1
            touchHoldTimer.reset();
            touch_longpress_valid = true;
            return;
        }
        tedit = 1 << tedit_exponent;  // Determine value editing rate
        trow = constrain((tft_touch[yy] + touch_fudge) / touch_cell_v_pix, 0, 4);
        tcol = (tft_touch[xx] - touch_margin_h_pix) / touch_cell_h_pix;
        // Take appropriate touchscreen actions depending on how we're being touched
        if (tcol == 0 && trow == 0 && !touch_now_touched) {
            if (++datapage >= arraysize(pagecard)) datapage -= arraysize(pagecard);  // Displayed dataset page can also be changed outside of simulator
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
                if (!touch_now_touched) sel_val = (sel_val + 1) % arraysize(datapage_names[datapage]);
                else if (touch_longpress_valid && touchHoldTimer.expired()) {
                    tunctrl = OFF;
                    touch_longpress_valid = false;
                }
            }
        }
        else if (tcol == 0 && trow == 2) {  // Pressed the increase value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // If just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta_touch = tedit;  // If in edit mode, increase the value
            // else adj_val(&idiot_hue_offset, idelta_touch, 0, 255);
            // set_idiotcolors();
        }
        else if (tcol == 0 && trow == 3) {  // Pressed the decrease value button, for real-time tuning of variables
            if (tunctrl == SELECT) tunctrl = EDIT;  // If just entering edit mode, don't change the value yet
            else if (tunctrl == EDIT) idelta_touch = -tedit;  // If in edit mode, decrease the value
            // else adj_val(&idiot_hue_offset -idelta_touch, 0, 255);
            // set_idiotcolors();
        }
        else if (tcol == 0 && trow == 4) {  // Pressed the simulation mode toggle. Needs long-press
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout()) {
                sim.toggle();
                touch_longpress_valid = false;
            }
        }
        if (tcol == 2 && trow == 0) {
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout()) {
                calmode_request = true;
                touch_longpress_valid = false;
            }
        }
        else if (sim.enabled()) {
            if (tcol == 4 && trow == 0) {
                if (touch_longpress_valid && touchHoldTimer.elapsed() > 2 * touchHoldTimer.timeout()) {  // Double hold time for some extra safety
                    ignition_request = req_tog;
                    touch_longpress_valid = false;
                }
            }
            else if (tcol == 5 && trow == 0) {
                if (touch_longpress_valid && touchHoldTimer.elapsed() > 2 * touchHoldTimer.timeout()) {  // Double hold time for some extra safety
                    sleep_request = req_on;  // sleep requests are handled by shutdown mode, otherwise will be ignored
                    touch_longpress_valid = false;
                }
            }
            else if (tcol == 3 && trow == 0 && sim.can_sim(sens::basicsw) && !touch_now_touched) basicmodesw = !basicmodesw;
            else if (tcol == 3 && trow == 1 && sim.can_sim(sens::pressure) && pressure.source() == src::TOUCH) pressure.add_human((float)tedit); // (+= 25) Pressed the increase brake pressure button
            else if (tcol == 3 && trow == 2 && sim.can_sim(sens::pressure) && pressure.source() == src::TOUCH) pressure.add_human((float)(-tedit)); // (-= 25) Pressed the decrease brake pressure button
            else if (tcol == 3 && trow == 4 && sim.can_sim(sens::joy)) adj_val(&hotrc.pc[horz][filt], -tedit, hotrc.pc[horz][opmin], hotrc.pc[horz][opmax]);
            else if (tcol == 4 && trow == 1 && sim.can_sim(sens::tach) && tach.source() == src::TOUCH) tach.add_human((float)tedit * 0.1);
            else if (tcol == 4 && trow == 2 && sim.can_sim(sens::tach) && tach.source() == src::TOUCH) tach.add_human((float)-tedit * 0.1);
            else if (tcol == 4 && trow == 3 && sim.can_sim(sens::joy)) adj_val(&hotrc.pc[vert][filt], tedit, hotrc.pc[vert][opmin], hotrc.pc[vert][opmax]);
            else if (tcol == 4 && trow == 4 && sim.can_sim(sens::joy)) adj_val(&hotrc.pc[vert][filt], -tedit, hotrc.pc[vert][opmin], hotrc.pc[vert][opmax]);
            else if (tcol == 5 && trow == 1 && sim.can_sim(sens::speedo) && speedo.source() == src::TOUCH) speedo.add_human((float)tedit * 0.05);
            else if (tcol == 5 && trow == 2 && sim.can_sim(sens::speedo) && speedo.source() == src::TOUCH) speedo.add_human((float)-tedit * 0.05);
            else if (tcol == 5 && trow == 4 && sim.can_sim(sens::joy)) adj_val(&hotrc.pc[horz][filt], tedit, hotrc.pc[horz][opmin], hotrc.pc[horz][opmax]);
        }
        // Update the tedit_exponent if needed
        if (tedit_exponent < tedit_exponent_max && (touchHoldTimer.elapsed() > (tedit_exponent + 1) * touchAccelTimer.timeout())) {
            tedit_exponent++;
            tedit = 1 << tedit_exponent; // Update the touch acceleration value
        }
        touch_now_touched = true;
    }
    void enableTouchPrint(bool enable) {
        touchPrintEnabled = enable;
    }
    void printTouchInfo() {
        if (touchPrintEnabled && touched()) {
            unsigned long currentTime = millis();
            if (currentTime - lastTouchPrintTime >= touchPrintInterval) {
                TS_Point touchpoint = getPoint();
                Serial.print("Touch detected at (x, y): ");
                Serial.print(touchpoint.x);
                Serial.print(", ");
                Serial.println(touchpoint.y);
                // If available, you can print touch pressure as well (for capacitive touch)
                lastTouchPrintTime = currentTime;
            }
        }
    }
};