
#pragma once
// #define CAPTOUCH  // #define CAPTOUCH if using IPS screen (black PCB), or #undef CAPTOUCH if using the red-PCB screen 
// #ifdef CAPTOUCH
  //#include <Adafruit_FT6206.h>
#define SENSITIVITY 40  // probably not needed for LGFX library
#define SIMPLETOUCH
// #else
//   #include <XPT2046_Touchscreen.h>
// #endif
#define touch_cell_v_pix 48  // When touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53  // When touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1  // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen
#define touch_reticle_offset 50  // Distance of center of each reticle to nearest screen edge
#define disp_tuning_lines 11  // Lines of dynamic variables/values in dataset pages 
class Touchscreen {
  private:
    //  #ifdef CAPTOUCH
    LGFX* _tft;
    I2C* _i2c;
    static constexpr uint8_t captouch_i2c_addr = 0x38;
    //  struct TS_Point {
    //      uint16_t x;
    //      uint16_t y;
    //  } touchpoint;
     // Adafruit_FT6206 _ts;
    // #else
    // XPT2046_Touchscreen _ts;  // 3.5in resistive touch panel on tft lcd
    // These values need to be calibrated to each individual display panel for best accuracy
    // int32_t corners[2][2] = { { 351, 3928 }, { 189, 3950 } };  // [xx/yy][min/max]
    // Soren's breadboard "" { { 351, 3933 }, { 189, 3950 } };  // [xx/yy][min/max]
    // TS_Point touchpoint;
    // #endif
    bool touch_longpress_valid = true, captouch;
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

    // debug printing
    bool touchPrintEnabled = true;
    unsigned long lastTouchPrintTime = 0;
    const unsigned long touchPrintInterval = 500; // Adjust this interval as needed (in milliseconds)
    
    enum touch_axis { xx, yy, zz };
    enum touch_lim { tsmin, tsmax };
    int32_t trow, tcol;
    int disp_width, disp_height;
    int touch_read[2], tft_touch[2], landed[2];  // landed are the initial coordinates of a touch event, unaffected by dragging
  public:
    int idelta = 0;

    Touchscreen() {}
    // #ifdef CAPTOUCH
    // Touchscreen(uint8_t csPin = 255, uint8_t irqPin = 255) {}
    // #else
    // Touchscreen(uint8_t csPin, uint8_t irqPin = 255) : _ts(csPin, irqPin) {}
    // #endif

    void setup(LGFX* tft, I2C* i2c, int width, int height) {
        _tft = tft;
        _i2c = i2c;
        disp_width = width;
        disp_height = height;
        captouch = (_i2c->device_detected(captouch_i2c_addr));
        Serial.printf("Touchscreen.. %s panel\n", (captouch) ? "detected captouch" : "using resistive");
        // #ifdef CAPTOUCH
            // _ts.begin(SENSITIVITY, &Wire);
            // _ts.setRotation(3);  // rotate -90 degrees to match IPS tft
        // #else
        //     _ts.begin();
        //     _ts.setRotation(3);  // rotate 90 degrees to match tft
        // #endif
    }

    bool touched() { return nowtouch; }
    int touch_x() { return tft_touch[xx]; }
    int touch_y() { return tft_touch[yy]; }

    // TS_Point getPoint() { return touchpoint; }
    int16_t touch_pt(uint8_t axis) { return tft_touch[axis]; }

    int read_touch() {
        // update touchpoint
        // #ifdef CAPTOUCH
        uint8_t count = _tft->getTouch(&(touch_read[xx]), &(touch_read[yy]));
        nowtouch = count;
        if (nowtouch) {
            if (captouch) {
                tft_touch[xx] = disp_width - touch_read[yy];
                tft_touch[yy] = disp_height - touch_read[xx];
            }
            else {
                tft_touch[xx] = disp_width;  // translate resistance to pixels
                tft_touch[yy] = disp_height;  // translate resistance to pixels            
            }
            // #else
            // else {
            //     nowtouch = _ts.touched();
            //     touchpoint = _ts.getPoint();
            //     tft_touch[xx] = map(touchpoint.x, corners[xx][tsmin], corners[xx][tsmax], 0, disp_width);  // translate resistance to pixels
            //     tft_touch[yy] = map(touchpoint.y, corners[yy][tsmin], corners[yy][tsmax], 0, disp_height);  // translate resistance to pixels                    
            // }
            // #endif
            // if (nowtouch) {
            // if (tft_touch[xx] != tlast_x || tft_touch[yy] != tlast_y) printf("x:%d y:%d\n", tft_touch[xx], tft_touch[yy]);
            // tlast_x = tft_touch[xx];
            // tlast_y = tft_touch[yy];
            tft_touch[xx] = constrain(tft_touch[xx], 0, disp_width);
            tft_touch[yy] = constrain(tft_touch[yy], 0, disp_height);
            // if (flip_the_screen) { 
            //     tft_touch[xx] = disp_width - tft_touch[xx];
            //     tft_touch[yy] = disp_height - tft_touch[yy];
            // }
            if (!landed_coordinates_valid) {
                landed[xx] = tft_touch[xx];
                landed[yy] = tft_touch[yy];
                landed_coordinates_valid = true;
            }
        }
        else landed_coordinates_valid = false;
        return nowtouch;
    }
    int read_touch_simple() {
        // update touchpoint
        // #ifdef CAPTOUCH
        nowtouch = (bool)(_tft->getTouch(&(touch_read[xx]), &(touch_read[yy])));
        if (nowtouch) {
            if (captouch) {
                tft_touch[xx] = disp_width - touch_read[yy];
                tft_touch[yy] = disp_height - touch_read[xx];
            }
            else {
                tft_touch[xx] = disp_width;  // translate resistance to pixels
                tft_touch[yy] = disp_height;  // translate resistance to pixels            
            }
        }
        return nowtouch;
    }
    void update() {
        #ifdef SIMPLETOUCH
            read_touch_simple();
        #else
            read_touch();
        #endif
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
        #ifdef SIMPLETOUCH
            trow = constrain((tft_touch[yy] + touch_fudge) / touch_cell_v_pix, 0, 4);
            tcol = constrain((tft_touch[xx] - touch_margin_h_pix) / touch_cell_h_pix, 0, 5);
        #else
            trow = constrain((landed[yy] + touch_fudge) / touch_cell_v_pix, 0, 4);
            tcol = constrain((landed[xx] - touch_margin_h_pix) / touch_cell_h_pix, 0, 5);
        #endif        
        // trow = constrain((landed[yy] + touch_fudge) / touch_cell_v_pix, 0, 4);
        // tcol = (landed[xx] - touch_margin_h_pix) / touch_cell_h_pix;
        // Serial.printf("touched trow: %d, tcol: %d, count: %d\n", trow, tcol, count); delay(1);
        // Take appropriate touchscreen actions depending on how we're being touched
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
        // Update the tedit_exponent if needed
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