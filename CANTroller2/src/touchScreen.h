#pragma once
// #ifdef CAP_TOUCH
//     #include <Adafruit_FT6206.h>  // For interfacing with the capacitive touchscreen controller chip
// #else
#include <XPT2046_Touchscreen.h>
// #endif

class TouchScreen {
private:
    #ifdef CAP_TOUCH
        #include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
        Adafruit_FT6206 _ts;  // 2.8in cap touch panel on tft lcd
    #else
        #include <XPT2046_Touchscreen.h>
        XPT2046_Touchscreen _ts;  // 3.2in resistive touch panel on tft lcd
        // XPT2046_Touchscreen ts (touch_cs_pin);  // 3.2in resistive touch panel on tft lcd
    #endif
    bool touch_longpress_valid = true;
    bool touch_now_touched = false;
    int32_t touch_accel_exponent = 0;
    int32_t touch_accel = 1 << touch_accel_exponent;
    int32_t touch_fudge = 0;
    int32_t touch_accel_exponent_max = 8;

    Timer touchHoldTimer{550000};  // Hold this long to count as a long press
    Timer touchAccelTimer{850000};
    Timer touchDoublePressTimer{40000};  // Won't allow a new press within this long after an old press (prevent accidental double clicks)

    // debug printing
    bool touchPrintEnabled = true;
    unsigned long lastTouchPrintTime = 0;
    const unsigned long touchPrintInterval = 500; // Adjust this interval as needed (in milliseconds)

public:
    #ifdef CAP_TOUCH
        TouchScreen() : _ts() {}
    #else
        TouchScreen(uint8_t csPin, uint8_t irqPin) : _ts(csPin, irqPin) {}
    #endif

    void init() {
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

    void handleTouch() {
        int32_t touch_x, touch_y, trow, tcol;
        // printf ("elaps:%8ld now:%d if:", touchDoublePressTimer.elapsed(), touch_now_touched);
        if (touched() && touchDoublePressTimer.expired()) { // Take actions if one touch is detected. This panel can read up to two simultaneous touchpoints
        //   printf ("touch\n");
          touch_accel = 1 << touch_accel_exponent;  // Determine value editing rate
            TS_Point touchpoint = getPoint();  // Retrieve a point
            #ifdef CAP_TOUCH
                // Rotate touch coordinates to match tft coordinates
                touchpoint.x = map(touchpoint.x, 0, disp_height_pix, disp_height_pix, 0);
                touchpoint.y = map(touchpoint.y, 0, disp_width_pix, disp_width_pix, 0);
                touch_y = disp_height_pix - touchpoint.x; // Touch point y coordinate in pixels, from the origin at the top-left corner
                touch_x = touchpoint.y; // Touch point x coordinate in pixels, from the origin at the top-left corner
            #else
                // Limit touch coordinates to a specific range
                touch_x = constrain(touchpoint.x, 355, 3968);
                touch_y = constrain(touchpoint.y, 230, 3930);
                touch_x = map(touch_x, 355, 3968, 0, disp_width_pix);
                touch_y = map(touch_y, 230, 3930, 0, disp_height_pix);

                if (!flip_the_screen) {
                    touch_x = disp_width_pix - touch_x;
                    touch_y = disp_height_pix - touch_y;
                }
            #endif

            trow = constrain((touch_y + touch_fudge) / touch_cell_v_pix, 0, 4);
            tcol = (touch_x - touch_margin_h_pix) / touch_cell_h_pix;

            // Take appropriate touchscreen actions depending on how we're being touched
            if (tcol == 0 && trow == 0 && !touch_now_touched) {
                if (++dataset_page >= arraysize(pagecard)) dataset_page -= arraysize(pagecard);  // Displayed dataset page can also be changed outside of simulator
            }
            else if (tcol == 0 && trow == 1) {  // Long touch to enter/exit editing mode, if in editing mode, press to change the selection of the item to edit
                if (tuning_ctrl == OFF) {
                    selected_value = 0;  // If entering select mode from off mode, select the first variable
                    if (touch_longpress_valid && touchHoldTimer.expired()) {
                        tuning_ctrl = SELECT;
                        touch_longpress_valid = false;
                    }
                }
                else if (tuning_ctrl == EDIT && !touch_now_touched) {
                    tuning_ctrl = SELECT;  // Drop back to select mode
                    selected_value++;  // Move to the next selection
                }
                else if (tuning_ctrl == SELECT) {
                    if (!touch_now_touched) selected_value = (selected_value + 1) % arraysize(dataset_page_names[dataset_page]);
                    else if (touch_longpress_valid && touchHoldTimer.expired()) {
                        tuning_ctrl = OFF;
                        touch_longpress_valid = false;
                    }
                }
            }
            else if (tcol == 0 && trow == 2) {  // Pressed the increase value button, for real-time tuning of variables
                if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If just entering edit mode, don't change the value yet
                else if (tuning_ctrl == EDIT) sim_edit_delta_touch = touch_accel;  // If in edit mode, increase the value
                // else adj_val(&idiot_hue_offset, sim_edit_delta_touch, 0, 255);
                // set_idiotcolors();
            }
            else if (tcol == 0 && trow == 3) {  // Pressed the decrease value button, for real-time tuning of variables
                if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If just entering edit mode, don't change the value yet
                else if (tuning_ctrl == EDIT) sim_edit_delta_touch = -touch_accel;  // If in edit mode, decrease the value
                // else adj_val(&idiot_hue_offset -sim_edit_delta_touch, 0, 255);
                // set_idiotcolors();
            }
            else if (tcol == 0 && trow == 4) {  // Pressed the simulation mode toggle. Needs long-press
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                    sim.toggle();
                    touch_longpress_valid = false;
                }
            }
            if (tcol == 2 && trow == 0 && (runmode == CAL || (runmode == SHUTDOWN && !shutdown_incomplete))) {
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                    calmode_request = true;
                    touch_longpress_valid = false;
                }
            }
            else if (tcol == 4 && trow == 0) {
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                    ignition_toggle_request = true;
                    touch_longpress_valid = false;
                }
            }
            else if (tcol == 5 && trow == 0) {
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                    syspower_toggle_request = true;
                    touch_longpress_valid = false;
                }
            }
            else if (sim.enabled()) {
                if (tcol == 3 && trow == 0 && sim.can_sim(sensor::basicsw) && !touch_now_touched) basicmodesw = !basicmodesw;
                else if (tcol == 3 && trow == 1 && sim.can_sim(sensor::pressure) && pressure_sensor.source() == ControllerMode::TOUCH) pressure_sensor.add_human((float)touch_accel); // (+= 25) Pressed the increase brake pressure button
                else if (tcol == 3 && trow == 2 && sim.can_sim(sensor::pressure) && pressure_sensor.source() == ControllerMode::TOUCH) pressure_sensor.add_human((float)(-touch_accel)); // (-= 25) Pressed the decrease brake pressure button
                else if (tcol == 3 && trow == 4 && sim.can_sim(sensor::joy)) adj_val(&hotrc_pc[HORZ][FILT], -touch_accel, hotrc_pc[HORZ][MIN], hotrc_pc[HORZ][MAX]);
                else if (tcol == 4 && trow == 1 && sim.can_sim(sensor::tach) && tachometer.source() == ControllerMode::TOUCH) tachometer.add_human((float)touch_accel * 0.1);
                else if (tcol == 4 && trow == 2 && sim.can_sim(sensor::tach) && tachometer.source() == ControllerMode::TOUCH) tachometer.add_human((float)-touch_accel * 0.1);
                else if (tcol == 4 && trow == 3 && sim.can_sim(sensor::joy)) adj_val(&hotrc_pc[VERT][FILT], touch_accel, hotrc_pc[VERT][MIN], hotrc_pc[VERT][MAX]);
                else if (tcol == 4 && trow == 4 && sim.can_sim(sensor::joy)) adj_val(&hotrc_pc[VERT][FILT], -touch_accel, hotrc_pc[VERT][MIN], hotrc_pc[VERT][MAX]);
                else if (tcol == 5 && trow == 1 && sim.can_sim(sensor::speedo) && speedometer.source() == ControllerMode::TOUCH) speedometer.add_human((float)touch_accel * 0.05);
                else if (tcol == 5 && trow == 2 && sim.can_sim(sensor::speedo) && speedometer.source() == ControllerMode::TOUCH) speedometer.add_human((float)-touch_accel * 0.05);
                else if (tcol == 5 && trow == 4 && sim.can_sim(sensor::joy)) adj_val(&hotrc_pc[HORZ][FILT], touch_accel, hotrc_pc[HORZ][MIN], hotrc_pc[HORZ][MAX]);
            }
            // Update the touch_accel_exponent if needed
            if (touch_accel_exponent < touch_accel_exponent_max && (touchHoldTimer.elapsed() > (touch_accel_exponent + 1) * touchAccelTimer.get_timeout())) {
                touch_accel_exponent++;
                touch_accel = 1 << touch_accel_exponent; // Update the touch acceleration value
            }
            
            touch_now_touched = true;
        } else { // If not being touched, put momentarily-set simulated button values back to default values
            sim_edit_delta_touch = 0;  // Stop changing the value
            if (touch_now_touched) touchDoublePressTimer.reset();  // Upon end of a touch, begin timer to reject any accidental double touches
            touch_now_touched = false;  // Remember the last touch state
            touch_accel_exponent = 0;
            touch_accel = 1 << touch_accel_exponent; // Reset touch acceleration value to 1
            touchHoldTimer.reset();
            touch_longpress_valid = true;
        }
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
                #ifdef CAP_TOUCH
                    Serial.print("Touch Pressure: ");
                    Serial.println(touchpoint.z);
                #endif

                // Add any other touch-related information you want to print
                // ...

                lastTouchPrintTime = currentTime;
            }
        }
    }
};