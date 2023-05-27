// Carpet CANTroller II  Source Code  - For Arduino Due with Adafruit 2.8inch Captouch TFT shield.
#include "globals.h"  // Contains variable and object initializations, and function definitions inculding setup function
#include "classes.h"  // Contains our data structures
#include "spid.h"  // A class for Soren's pid loop code
using namespace std;
void setup() {
    cantroller2_init();
};
// Main loop.  Each time through we do these eight steps:
//
// 0) Beginning-of-the-loop nonsense
// 1) Gather new telemetry and filter the signals
// 2) Check if our current runmode has been overridden by certain specific conditions
// 3) Read joystick horizontal and determine new steering setpoint
// 4) Do actions based on which runmode we are in (including gas & brake setpoint), and possibly change runmode 
// 5) Step the PID loops and update the actuation outputs
// 6) Service the user interface
// 7) Log to SD card
// 8) Do the control loop bookkeeping at the end of each loop
//
void loop() {
    // 0) Beginning-of-the-loop nonsense
    //
    if (serial_debugging && print_timestamps) {
        Serial.print("Loop# ");
        Serial.print(loopno);  Serial.print(": ");      
        loopzero = mycros();  // Start time for loop
    }
    // Update derived variable values in case they have changed
    ctrl_db_adc[VERT][BOT] = (adc_range_adc-ctrl_lims_adc[ctrl][VERT][DB])/2;  // Lower threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[VERT][TOP] = (adc_range_adc+ctrl_lims_adc[ctrl][VERT][DB])/2;  // Upper threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][BOT] = (adc_range_adc-ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Lower threshold of horz joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][TOP] = (adc_range_adc+ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Upper threshold of horz joy deadband (ADC count 0-4095)
    engine_govern_rpm = map(gas_governor_percent, 0, 100, 0, engine_redline_rpm);  // Create an artificially reduced maximum for the engine speed
    gas_pulse_govern_us = map(gas_governor_percent*(engine_redline_rpm-engine_idle_rpm)/engine_redline_rpm, 0, 100, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
    carspeed_govern_mmph = map(gas_governor_percent, 0, 100, 0, carspeed_redline_mmph);  // Governor must scale the top vehicle speed proportionally

    if (heartbeatTimer.expired()) {  // Heartbeat LED
        heartbeat_pulse = !heartbeat_pulse;
        if (++heartbeat_state >= arraysize(heartbeat_ekg)) heartbeat_state -= arraysize(heartbeat_ekg);
        heartbeatTimer.set(heartbeat_ekg[heartbeat_state]);
        digitalWrite(heartbeat_led_pin, heartbeat_pulse);
    }
    uint8_t neopixel_wheel_counter = 0;
    if (neopixelTimer.expired()) {
        neopixel_wheel_counter++;
        strip.setPixelColor(0, colorwheel(neopixel_wheel_counter));
        neopixelTimer.reset();
    }
    // Encoder
    // Read and interpret encoder switch activity. Encoder rotation is handled in interrupt routine
    // Encoder handler routines should act whenever encoder_sw_action is true, setting it back to false once handled.
    // When handling press, if encoder_long_clicked is nonzero then press is a long press
    if (!digitalRead(encoder_sw_pin)) {  // if encoder sw is being pressed (switch is active low)
        if (!encoder_sw) {  // if the press just occurred
            encoderLongPressTimer.reset();  // start a press timer
            encoder_timer_active = true;  // flag to indicate timing for a possible long press
        }
        else if (encoder_timer_active && encoderLongPressTimer.expired()) {  // If press time exceeds long press threshold
            encoder_sw_action = LONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
            encoder_timer_active = false;  // Keeps us from entering this logic again until after next sw release (to prevent repeated long presses)
            encoder_suppress_click = true;  // Prevents the switch release after a long press from causing a short press
        }
        encoder_sw = true;  // Remember a press is in effect
    }
    else {  // if encoder sw is not being pressed
        if (encoder_sw && !encoder_suppress_click) encoder_sw_action = SHORT;  // if the switch was just released, a short press occurred, which must be handled
        encoder_timer_active = false;  // Allows detection of next long press event
        encoder_sw = false;  // Remember press is not in effect
        encoder_suppress_click = false;  // End click suppression
    }

    // 1) Gather new telemetry and filter the signals
    //  
    pot_adc = 0;
    #ifndef ESP32
        pot_adc = analogRead(pot_wipe_pin);  // Potentiometer
    #endif
    pot_filt_adc = ema(pot_adc, pot_filt_adc, pot_ema_alpha);
    
    // Voltage of vehicle battery
    battery_adc = analogRead(battery_pin);
    battery_mv = (int32_t)(battery_max_mv*((double)battery_adc)/adc_range_adc);  // convert adc value read into mV    
    battery_filt_mv = ema(battery_mv, battery_filt_mv, battery_ema_alpha);  // Apply EMA filter
    
    // Read sensors
    if (!ui_simulating[GLOBAL] && !ui_simulating[BRKPOS]) {
        brake_pos_adc = analogRead(brake_pos_pin);
        brake_pos_filt_adc = ema(brake_pos_adc, brake_pos_filt_adc, brake_pos_ema_alpha);
    }
    else if (!ui_simulating[GLOBAL] && ui_simulating[BRKPOS] == TOUCH) brake_pos_filt_adc = (brake_pos_nom_lim_retract_adc + brake_pos_zeropoint_adc)/2;  // To keep brake position in legal range during simulation
    // Tach
    if (!ui_simulating[GLOBAL] || !ui_simulating[TACH]) {
        if (tachPulseTimer.elapsed() < engine_stop_timeout_us) engine_rpm = (int32_t)(60000000/(double)tach_delta_us);  // Tachometer magnets/us * 60000000 (1 rot/magnet * 1000000 us/sec * 60 sec/min) gives rpm
        else engine_rpm = 0;  // If timeout since last magnet is exceeded
        if (abs(engine_rpm-engine_old_rpm) > engine_lp_thresh_rpm || engine_rpm-engine_last_rpm < engine_spike_thresh_rpm) {  // Remove noise spikes from tach values, if otherwise in range
            engine_old_rpm = engine_last_rpm;
            engine_last_rpm = engine_rpm;
        }
        else engine_rpm = engine_last_rpm;  // Spike detected - ignore that sample
        if (engine_rpm) engine_filt_rpm = ema(engine_rpm, engine_filt_rpm, engine_ema_alpha);  // Sensor EMA filter
        else engine_filt_rpm = 0;
    }
    // Speedo
    if (!ui_simulating[GLOBAL] || !ui_simulating[SPEEDO]) { 
        if (speedoPulseTimer.elapsed() < car_stop_timeout_us) carspeed_mmph = (int32_t)(179757270/(double)speedo_delta_us); // Update car speed value  
        // magnets/us * 179757270 (1 rot/magnet * 1000000 us/sec * 3600 sec/hr * 1/19.85 gearing * 20*pi in/rot * 1/12 ft/in * 1000/5280 milli-mi/ft gives milli-mph  // * 1/1.15 knots/mph gives milliknots
        // Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
        else carspeed_mmph = 0;     
        if (abs(carspeed_mmph-carspeed_old_mmph) > carspeed_lp_thresh_mmph || carspeed_mmph-carspeed_last_mmph < carspeed_spike_thresh_mmph) {  // Remove noise spikes from speedo values, if otherwise in range
            carspeed_old_mmph = carspeed_last_mmph;
            carspeed_last_mmph = carspeed_mmph;
        }
        else carspeed_mmph = carspeed_last_mmph;  // Spike detected - ignore that sample
        if (carspeed_mmph)  carspeed_filt_mmph = ema(carspeed_mmph, carspeed_filt_mmph, carspeed_ema_alpha);  // Sensor EMA filter
        else carspeed_filt_mmph = 0;
    }

    if (!ui_simulating[GLOBAL] || !ui_simulating[PRESS]) {
        pressure_adc = analogRead(pressure_pin);  // Brake pressure.  Read sensor, then Remove noise spikes from brake feedback, if reading is otherwise in range
        if (abs(pressure_adc-pressure_old_adc) > pressure_lp_thresh_adc || pressure_adc-pressure_last_adc < pressure_spike_thresh_adc) {
            pressure_old_adc = pressure_last_adc;
            pressure_last_adc = pressure_adc;
        }
        else pressure_adc = pressure_last_adc;  // Spike detected - ignore that sample
        // pressure_psi = (int32_t)(1000*(double)(pressure_adc)/adc_range_adc);      // Convert pressure to units of psi
        pressure_filt_adc = ema(pressure_adc, pressure_filt_adc, pressure_ema_alpha);  // Sensor EMA filter
    }

   // 2) Read joystick then determine new steering setpoint, and handle digital pins
    //
    digitalWrite(led_tx_pin, !touch_now_touched);
    digitalWrite(led_rx_pin, (sim_edit_delta > 0) ? 0 : 1);  
    if (!ui_simulating[GLOBAL] || !ui_simulating[CTRL]) {  // If not simulating or not simulating joystick
        if (ctrl == HOTRC) {
            ctrl_pos_adc[VERT][RAW] = map(hotrc_vert_pulse_us, 2003, 1009, ctrl_lims_adc[ctrl][VERT][MAX], ctrl_lims_adc[ctrl][VERT][MIN]);
            ctrl_pos_adc[HORZ][RAW] = map(hotrc_horz_pulse_us, 2003, 1009, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
            ctrl_pos_adc[VERT][RAW] = constrain(ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
            ctrl_pos_adc[HORZ][RAW] = constrain(ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);   
        }
        else {
            ctrl_pos_adc[VERT][RAW] = analogRead(joy_vert_pin);  // Read joy vertical
            ctrl_pos_adc[HORZ][RAW] = analogRead(joy_horz_pin);  // Read joy horizontal
        }        
        if (ctrl_pos_adc[VERT][RAW] > ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][RAW] < ctrl_db_adc[VERT][TOP])  ctrl_pos_adc[VERT][FILT] = adc_midscale_adc;  // if joy vert is in the deadband, set joy_vert_filt to center value
        else ctrl_pos_adc[VERT][FILT] = ema(ctrl_pos_adc[VERT][RAW], ctrl_pos_adc[VERT][FILT], ctrl_ema_alpha[ctrl]);  // otherwise do ema filter to determine joy_vert_filt
        if (ctrl_pos_adc[HORZ][RAW] > ctrl_db_adc[HORZ][BOT] && ctrl_pos_adc[HORZ][RAW] < ctrl_db_adc[HORZ][TOP])  ctrl_pos_adc[HORZ][FILT] = adc_midscale_adc;  // if joy horz is in the deadband, set joy_horz_filt to center value
        else ctrl_pos_adc[HORZ][FILT] = ema(ctrl_pos_adc[HORZ][RAW], ctrl_pos_adc[HORZ][FILT], ctrl_ema_alpha[ctrl]);  // otherwise do ema filter to determine joy_horz_filt
    }
    if (!(runmode == SHUTDOWN && (!carspeed_filt_mmph || shutdown_complete)))  { // If not in shutdown mode with shutdown complete and car stopped
        if (ctrl_pos_adc[HORZ][FILT] >= ctrl_db_adc[HORZ][TOP]) {
            steer_pulse_safe_us = steer_pulse_stop_us + (int32_t)( (double)(steer_pulse_right_us - steer_pulse_stop_us) * (1 - ( (double)steer_safe_percent * carspeed_filt_mmph / ((double)carspeed_redline_mmph * 100) ) ) );
            steer_pulse_out_us = map(ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][TOP], ctrl_lims_adc[ctrl][HORZ][MAX], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the right of deadband
        }
        else if (ctrl_pos_adc[HORZ][FILT] <= ctrl_db_adc[HORZ][BOT]) {
            steer_pulse_safe_us = steer_pulse_stop_us - (int32_t)( (double)(steer_pulse_stop_us - steer_pulse_left_us) * (1 - ( (double)steer_safe_percent * carspeed_filt_mmph / ((double)carspeed_redline_mmph * 100) ) ) );
            steer_pulse_out_us = map(ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][BOT], ctrl_lims_adc[ctrl][HORZ][MIN], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the left of deadband
        }
        else steer_pulse_out_us = steer_pulse_stop_us;  // Stop the steering motor if inside the deadband
    }
    if (ctrl == HOTRC && hotrc_ch3_sw_event) {  // Turn on/off the vehicle ignition
        ignition = !ignition;
        hotrc_ch3_sw_event = false;
    }
    if (ctrl == HOTRC && hotrc_ch4_sw_event) {  // Toggle cruise/fly mode 
        if (runmode == FLY) runmode = CRUISE;
        else if (runmode == CRUISE) runmode = FLY;
        hotrc_ch4_sw_event = false;    
    }
    if (!ui_simulating[GLOBAL] || !ui_simulating[BASICSW]) basicmodesw = !digitalRead(basicmodesw_pin);   // 1-value because electrical signal is active low
    if (!ui_simulating[GLOBAL] || !ui_simulating[CRUISESW]) cruise_sw = digitalRead(cruise_sw_pin);

    // 3) Check if our current runmode has been overridden by certain specific conditions
    //
    if (basicmodesw) runmode = BASIC;    // if basicmode switch on --> Basic Mode
    else if (!ignition) runmode = SHUTDOWN;  //} && laboratory != true)  {  // otherwise if ignition off --> Shutdown Mode
    else if (!engine_filt_rpm)  runmode = STALL;    // otherwise if engine not running --> Stall Mode
    
    // 4) Do actions based on which runmode we are in (and set gas/brake setpoints), and possibly change runmode 
    //
    if (runmode == BASIC)  {  // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering stell works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        if ((!basicmodesw) && engine_filt_rpm)  runmode = HOLD;  // If we turned off the basic mode switch with engine running, go to Hold mode. If engine is not running, we'll end up in Stall Mode automatically
    }
    else if (runmode == SHUTDOWN)  { // In shutdown mode we stop the car if it's moving then park the motors.
        if (ignition) runmode = STALL;
        else if (we_just_switched_modes)  {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            engine_target_rpm = engine_idle_rpm;  //  Release the throttle 
            shutdown_complete = false;
            if (carspeed_filt_mmph)  {
                pressure_target_adc = (panic_stop) ? brake_panic_initial_adc : brake_hold_initial_adc;  // More brakes, etc. to stop the car
                brakeIntervalTimer.reset();
                sanityTimer.reset();
            }
        }
        if (!shutdown_complete)  {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (!carspeed_filt_mmph || sanityTimer.expired())  {  // If car has stopped, or timeout expires, then release the brake
                motorParkTimer.reset();  // Set a timer to timebox this effort
                park_the_motors = true;  // Flags the motor parking to happen
                if (pressure_filt_adc <= pressure_min_adc + pressure_margin_adc)  shutdown_complete = true;  // With this set, we will do nothing from here on out (until mode changes, i.e. ignition)
            }
            else if (brakeIntervalTimer.expired())  {
                pressure_target_adc += (panic_stop) ? brake_panic_increment_adc : brake_hold_increment_adc;  // Slowly add more brakes until car stops
                brakeIntervalTimer.reset();  
            }
            else if (!park_the_motors) shutdown_complete = true;
        }
    }
    else if (runmode == STALL)  {   // In stall mode, the gas doesn't have feedback
        if (engine_filt_rpm)  runmode = HOLD;  //  Enter Hold Mode if we started the car
        else {  // Actuators still respond and everything, even tho engine is turned off
            pressure_target_adc = pressure_min_adc;  // Default when joystick not pressed
            gas_pulse_out_us = gas_pulse_idle_us;  // Default when joystick not pressed
            if (ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][TOP])  { //  If we are pushing up
                // In stall mode there is no engine rpm for PID to use as feedback, so we bypass the PID and just set the engine_target_angle proportional to 
                // the joystick position.  This works whether there is normally a gas PID or not.
                gas_pulse_out_us = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], gas_pulse_idle_us, gas_pulse_govern_us);
            }
            else if (ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT])  {  // If we are pushing down
                pressure_target_adc = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][BOT], ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_adc, pressure_max_adc);  // Scale joystick value to pressure adc setpoint
            }
        }
    }
    else if (runmode == HOLD)  {
        if (we_just_switched_modes)  {  // Release throttle and push brake upon entering hold mode
            engine_target_rpm = engine_idle_rpm;  // Let off gas (if gas using PID mode)
            if (!carspeed_filt_mmph)  pressure_target_adc += brake_hold_increment_adc; // If the car is already stopped then just add a touch more pressure and then hold it.
            else pressure_target_adc = brake_hold_initial_adc;  //  Otherwise, these hippies need us to stop the car for them
            brakeIntervalTimer.reset();
            joy_centered = false;
        }
        else if (carspeed_filt_mmph && brakeIntervalTimer.expired())  { // Each interval the car is still moving, push harder
            pressure_target_adc += brake_hold_increment_adc;  // Slowly add more brakes until car stops
            brakeIntervalTimer.reset();
        }
        pressure_target_adc = constrain(pressure_target_adc, pressure_min_adc, pressure_max_adc);  // Just make sure we don't try to push harder than we can 
        if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) {
            joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        }
        if (joy_centered && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][TOP]) { 
            runmode = FLY; // Enter Fly Mode upon joystick movement from center to above center
        }
    }
    else if (runmode == FLY)  {
        if (we_just_switched_modes)  {
            gesture_progress = 0;
            gestureFlyTimer.reset(); // Initialize gesture timer to already-expired value
            cruise_sw_held = false;
            cruiseSwTimer.reset();
            hotrc_ch4_sw_event = false;
        }
        if (!carspeed_filt_mmph && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT])  runmode = HOLD;  // Go to Hold Mode if we have braked to a stop
        else  {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            engine_target_rpm = engine_idle_rpm;  // Default when joystick not pressed 
            pressure_target_adc = pressure_min_adc;  // Default when joystick not pressed   
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP])  {  // If we are trying to accelerate
                engine_target_rpm = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], engine_idle_rpm, engine_govern_rpm);
            }
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT])  {  // If we are trying to brake, scale joystick value to determine pressure adc setpoint
                pressure_target_adc = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][BOT], ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_adc, pressure_max_adc);
            }
        }
        // Cruise mode can be entered by pressing a physical momentary button, or by holding the brake on full for a half second. Which epends on the cruise_gesturing flag.
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (cruise_gesturing) {  // If we are configured to use joystick gestures to go to cruise mode, the gesture is 
            if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP])  { // Re-zero gesture timer for potential new gesture whenever joystick at center
                gestureFlyTimer.reset();
            }
            if (gestureFlyTimer.expired()) gesture_progress = 0; // If gesture timeout has expired, cancel any in-progress gesture
            else {  // Otherwise check for successful gesture motions
                if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_lims_adc[ctrl][VERT][MAX]-default_margin_adc)  { // If joystick quickly pushed to top, step 1 of gesture is successful
                    gesture_progress++;
                    gestureFlyTimer.reset();
                }
                else if (gesture_progress == 1 && ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN]+default_margin_adc)  { // If joystick then quickly pushed to bottom, step 2 succeeds
                    gesture_progress++;
                    gestureFlyTimer.reset();
                }
                else if (gesture_progress == 2 && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP]) { // If joystick then quickly returned to center, go to Cruise mode
                    runmode = CRUISE;
                }        
            }
        }
        if (!cruise_sw) {  // If button not currently pressed
            if (cruise_sw_held && cruiseSwTimer.expired())  runmode = CRUISE;  // If button was just held long enough, upon release enter Cruise mode
            cruise_sw_held = false;  // Cancel button held state
        }
        else if (!cruise_sw_held) {  // If button is being pressed, but we aren't in button held state
            cruiseSwTimer.reset(); // Start hold time timer
            cruise_sw_held = true;  // Get into that state
        }
    }
    else if (runmode == CRUISE)  {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            carspeed_target_mmph = carspeed_filt_mmph;  // Begin cruising with cruise set to current speed
            pressure_target_adc = pressure_min_adc;  // Let off the brake and keep it there till out of Cruise mode
            gestureFlyTimer.reset();  // reset gesture timer
            cruise_sw_held = false;
            hotrc_ch4_sw_event = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) {  // When joystick vert above center, increase the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            engine_target_rpm = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], engine_filt_rpm, engine_govern_rpm);
        }
        else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) {  // When joystick vert below center, decrease the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            engine_target_rpm = map(ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_db_adc[VERT][BOT], engine_idle_rpm, engine_filt_rpm);
        }
        else cruise_adjusting = false;  // if joystick at center
        if (cruise_adjusting) {
            carspeed_target_mmph = carspeed_filt_mmph;  // Upon return to center set speed target to current speed
            cruiseSPID.set_target((double)carspeed_target_mmph);
        }
        // This old gesture trigger drops to Fly mode if joystick moved quickly from center to bottom
        // if (ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN]+default_margin_adc && abs(mycros() - gesture_timer_us) < gesture_flytimeout_us)  runmode = FLY;  // If joystick quickly pushed to bottom 
        // printf("hotvpuls=%ld, hothpuls=%ld, joyvfilt=%ld, joyvmin+marg=%ld, timer=%ld\n", hotrc_vert_pulse_us, hotrc_horz_pulse_us, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc, gesture_timer_us);
        if (ctrl_pos_adc[VERT][RAW] > ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) runmode = FLY;  // New gesture to drop to fly mode is hold the brake all the way down for 500 ms
        if (cruise_sw)  cruise_sw_held = true;   // Pushing cruise button sets up return to fly mode
        else if (cruise_sw_held) { // Release of button drops us back to fly mode
            cruise_sw_held = false;
            runmode = FLY;
        }
        if (!carspeed_filt_mmph)  {  // In case we slam into a brick wall, get out of cruise mode
            if (serial_debugging) Serial.println(F("Error: Car stopped or taken out of gear in cruise mode"));  // , carspeed_filt_mmph, neutral
            runmode = HOLD;  // Back to Hold Mode  
        }
    }
    else { // Obviously this should never happen
        if (serial_debugging) Serial.println(F("Error: Invalid runmode entered"));  // ,  runmode
        runmode = HOLD;
    }

    // 5) Step the pids, update the actuator outputs  (at regular intervals)
    //
    if (pidTimer.expired() && !(runmode == SHUTDOWN && shutdown_complete))  {  // Recalculate pid and update outputs, at regular intervals
        steer_pulse_out_us = constrain(steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);  // Don't be out of range
        steer_servo.writeMicroseconds(steer_pulse_out_us);   // Write steering value to jaguar servo interface

        if (park_the_motors) {  // First check if we're in motor parking mode, if so park motors instead of running PIDs
            if ( ( abs(brake_pos_filt_adc - brake_pos_park_adc) <= default_margin_adc &&     // IF ( the brake motor is close enough to the park position AND
                abs(gas_pulse_out_us == gas_pulse_idle_us + gas_pulse_park_slack_us) )  //      so is the gas servo )
                || motorParkTimer.expired() ) {        //    OR the parking timeout has expired
                park_the_motors = false;                                                // THEN stop trying to park the motors
            }
            else {
                gas_pulse_out_us = gas_pulse_idle_us + gas_pulse_park_slack_us;
                gas_servo.writeMicroseconds(gas_pulse_out_us);
                if (brake_pos_filt_adc + brake_pos_margin_adc <= brake_pos_park_adc) brake_pulse_out_us = map (brake_pos_filt_adc, brake_pos_park_adc, brake_pos_nom_lim_retract_adc, brake_pulse_stop_us, brake_pulse_extend_us); // If brake is retracted from park point, extend toward park point, slowing as we approach
                if (brake_pos_filt_adc - brake_pos_margin_adc >= brake_pos_park_adc) brake_pulse_out_us = map (brake_pos_filt_adc, brake_pos_park_adc, brake_pos_nom_lim_extend_adc, brake_pulse_stop_us, brake_pulse_retract_us); // If brake is extended from park point, retract toward park point, slowing as we approach
                brake_pulse_out_us = constrain(brake_pulse_out_us, brake_pulse_retract_us, brake_pulse_extend_us);  // Send to the actuator. Refuse to exceed range    
                brake_servo.writeMicroseconds(brake_pulse_out_us);  // Write result to jaguar servo interface
            }
        }
        else if (runmode != BASIC) {  // Unless basicmode switch is turned on, we want brake and gas   
            pressure_target_adc = constrain(pressure_target_adc, pressure_min_adc, pressure_max_adc);  // Make sure pressure target is in range
            // printf("Brake PID rm=%-+4ld target=%-+9.4lf", runmode, (double)pressure_target_adc);
            brakeSPID.set_target((double)pressure_target_adc);
            brakeSPID.compute((double)pressure_filt_adc);
            brake_pulse_out_us = (int32_t)brakeSPID.get_output();
            // printf(" output = %-+9.4lf,  %+-4ld\n", brakeSPID.get_output(), brake_pulse_out_us);
            if ( ((brake_pos_filt_adc + brake_pos_margin_adc <= brake_pos_nom_lim_retract_adc) && (brake_pulse_out_us < brake_pulse_stop_us)) ||  // If the motor is at or past its position limit in the retract direction, and we're intending to retract more ...
                 ((brake_pos_filt_adc - brake_pos_margin_adc >= brake_pos_nom_lim_extend_adc) && (brake_pulse_out_us > brake_pulse_stop_us)) )  // ... or same thing in the extend direction ...
                brake_pulse_out_us = brake_pulse_stop_us;  // ... then stop the motor
                // Improve this by having the motor actively go back toward position range if position is beyond either limit             
            brake_servo.writeMicroseconds(brake_pulse_out_us);  // Write result to jaguar servo interface
            if (runmode == CRUISE && !cruise_adjusting) {  // Cruise loop updates gas rpm target to keep speed equal to cruise mmph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
                carspeed_target_mmph = constrain(carspeed_target_mmph, 0, carspeed_redline_mmph);
                // printf("Cruise PID rm= %+-4ld target=%-+9.4lf", runmode, (double)carspeed_target_mmph);
                cruiseSPID.compute((double)carspeed_filt_mmph);
                engine_target_rpm = (int32_t)cruiseSPID.get_output();
                // printf(" output = %-+9.4lf,  %+-4ld\n", cruiseSPID.get_output(), engine_target_rpm);
            }
            if (runmode != STALL) {  // Gas loop is effective in Fly or Cruise mode, we need to determine gas actuator output from rpm target
                engine_target_rpm = constrain(engine_target_rpm, engine_idle_rpm, engine_govern_rpm);  // Make sure desired rpm isn't out of range (due to crazy pid math, for example)
                if (gasSPID.get_open_loop()) gas_pulse_out_us = map(engine_target_rpm, engine_idle_rpm, engine_govern_rpm, gas_pulse_idle_us, gas_pulse_govern_us); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
                else {  // Do soren's quasi-pid math to determine gas_pulse_out_us from engine rpm error
                    // printf("Gas PID   rm= %+-4ld target=%-+9.4lf", runmode, (double)engine_target_rpm);
                    gasSPID.set_target((double)engine_target_rpm);
                    gasSPID.compute((double)engine_filt_rpm);
                    gas_pulse_out_us = (int32_t)gasSPID.get_output();
                    // printf(" output = %-+9.4lf,  %+-4ld\n", gasSPID.get_output(), gas_pulse_out_us);
                }
                gas_servo.writeMicroseconds(gas_pulse_out_us);  // Write result to servo
            }
        }
        pidTimer.reset();  // reset timer to trigger the next update
    }
    diagnostic();
    
    // 6) Service the user interface
    //
    int32_t touch_x, touch_y, touch_row, touch_col;
    if (touchPollTimer.expired()) {
        touchPollTimer.reset();
        if (touchpanel.touched()) { // Take actions upon being touched
            touch_accel = 1 << touch_accel_exponent;  // determine value editing rate
            TS_Point touchpoint = touchpanel.getPoint();  // Retreive a point
            touchpoint.x = map(touchpoint.x, 0, disp_height_pix, disp_height_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touchpoint.y = map(touchpoint.y, 0, disp_width_pix, disp_width_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touch_y = disp_height_pix-touchpoint.x; // touch point y coordinate in pixels, from origin at top left corner
            touch_x = touchpoint.y; // touch point x coordinate in pixels, from origin at top left corner
            touch_row = touch_y/touch_cell_v_pix;
            touch_col = (touch_x-touch_margin_h_pix)/touch_cell_h_pix;
            // Take appropriate touchscreen actions depending how we're being touched
            if (touch_col == 0 && touch_row == 0 && !touch_now_touched) {
                dataset_page += 1; // Displayed dataset page can also be changed outside of simulator
                if (dataset_page >= arraysize(pagecard)) dataset_page -= arraysize(pagecard);
            }
            else if (touch_col == 0 && touch_row == 1) {  // Long touch to enter/exit editing mode, if in editing mode, press to change selection of item to edit
                if (tuning_ctrl == OFF) {
                    selected_value = 0;  // if entering select mode from off mode, select first variable
                    if (touch_longpress_valid && touchHoldTimer.expired()) {
                        tuning_ctrl = SELECT;
                        touch_longpress_valid = false;
                    }
                }
                else if (tuning_ctrl == EDIT && !touch_now_touched) {
                    tuning_ctrl = SELECT;  // drop back to select mode
                    selected_value++;  // and move to next selection
                }
                else if (tuning_ctrl == SELECT) {
                    if (!touch_now_touched) {
                        if (++selected_value >= arraysize(dataset_page_names[dataset_page])) selected_value -= arraysize(dataset_page_names[dataset_page]);
                    }
                    else if (touch_longpress_valid && touchHoldTimer.expired()) {
                        tuning_ctrl = OFF;
                        touch_longpress_valid = false;
                    }
                }
            }
            else if (touch_col == 0 && touch_row == 2) {  // Pressed the increase value button, for real time tuning of variables
                if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If just entering edit mode, don't change value yet
                else if (tuning_ctrl == EDIT) sim_edit_delta_touch = touch_accel;  // If in edit mode, increase value
            }   
            else if (touch_col == 0 && touch_row == 3) {  // Pressed the decrease value button, for real time tuning of variables
                if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If just entering edit mode, don't change value yet
                else if (tuning_ctrl == EDIT) sim_edit_delta_touch = -touch_accel;  // If in edit mode, decrease value
            }
            else if (touch_col == 0 && touch_row == 4) {  // && touch_row == 0 . Pressed the simulation mode toggle. Needs long press
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout())  {
                    ui_simulating[GLOBAL] = (1 - ui_simulating[GLOBAL]);
                    touch_longpress_valid = false;
                }
            }
            else if (touch_col == 3 && touch_row == 0 && ui_simulating[GLOBAL] && ui_simulating[BASICSW] && !touch_now_touched) basicmodesw = !basicmodesw;  // Pressed the basic mode toggle button. Toggle value, only once per touch
            else if (touch_col == 3 && touch_row == 1 && ui_simulating[GLOBAL] && ui_simulating[PRESS]) adj_val(&pressure_filt_adc, touch_accel, pressure_min_adc, pressure_max_adc);  // (+= 25) Pressed the increase brake pressure button
            else if (touch_col == 3 && touch_row == 2 && ui_simulating[GLOBAL] && ui_simulating[PRESS]) adj_val(&pressure_filt_adc, -touch_accel, pressure_min_adc, pressure_max_adc);  // (-= 25) Pressed the decrease brake pressure button
            else if (touch_col == 3 && touch_row == 4 && ui_simulating[GLOBAL] && ui_simulating[CTRL]) adj_val(&ctrl_pos_adc[HORZ][FILT], -touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (-= 25) Pressed the joystick left button
            else if (touch_col == 4 && touch_row == 0 && ui_simulating[GLOBAL] && ui_simulating[IGN] && !touch_now_touched) ignition = !ignition; // Pressed the ignition switch toggle button. Toggle value, only once per touch
            else if (touch_col == 4 && touch_row == 1 && ui_simulating[GLOBAL] && ui_simulating[TACH]) adj_val(&engine_filt_rpm, touch_accel, 0, engine_redline_rpm);  // (+= 25) Pressed the increase engine rpm button
            else if (touch_col == 4 && touch_row == 2 && ui_simulating[GLOBAL] && ui_simulating[TACH]) adj_val(&engine_filt_rpm, -touch_accel, 0, engine_redline_rpm);  // (-= 25) Pressed the decrease engine rpm button
            else if (touch_col == 4 && touch_row == 3 && ui_simulating[GLOBAL] && ui_simulating[CTRL]) adj_val(&ctrl_pos_adc[VERT][FILT], touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (+= 25) Pressed the joystick up button
            else if (touch_col == 4 && touch_row == 4 && ui_simulating[GLOBAL] && ui_simulating[CTRL]) adj_val(&ctrl_pos_adc[VERT][FILT], -touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (-= 25) Pressed the joystick down button
            else if (touch_col == 5 && touch_row == 0 && ui_simulating[GLOBAL] && ui_simulating[CRUISESW]) cruise_sw = true;  // Pressed the cruise mode button. This is a momentary control, not a toggle. Value changes back upon release
            else if (touch_col == 5 && touch_row == 1 && ui_simulating[GLOBAL] && ui_simulating[SPEEDO]) adj_val(&carspeed_filt_mmph, touch_accel, 0, carspeed_redline_mmph);  // (+= 50) // Pressed the increase vehicle speed button
            else if (touch_col == 5 && touch_row == 2 && ui_simulating[GLOBAL] && ui_simulating[SPEEDO]) adj_val(&carspeed_filt_mmph, -touch_accel, 0, carspeed_redline_mmph);  // (-= 50) Pressed the decrease vehicle speed button
            else if (touch_col == 5 && touch_row == 4 && ui_simulating[GLOBAL] && ui_simulating[CTRL]) adj_val(&ctrl_pos_adc[HORZ][FILT], touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (+= 25) Pressed the joystick right button                           
            if (touch_accel_exponent < touch_accel_exponent_max && (touchHoldTimer.elapsed() > (touch_accel_exponent + 1) * touchAccelTimer.timeout())) touch_accel_exponent++; // If timer is > the shift time * exponent, and not already maxed, double the edit speed by incrementing the exponent
                
            touch_now_touched = true;
        }  // (if touchpanel reads a touch)
        else {  // If not being touched, put momentarily-set simulated button values back to default values
            if (ui_simulating[GLOBAL]) cruise_sw = false;  // // Makes this button effectively momentary
            sim_edit_delta_touch = 0;  // Stop changing value
            touch_now_touched = false;  // remember last touch state
            touch_accel_exponent = 0;
            touchHoldTimer.reset();
            touch_longpress_valid = true;
        }
    }
    // Encoder handling
    //
    if (encoder_sw_action != NONE) {  // First deal with any unhandled switch press events
        if (serial_debugging) Serial.println("in encoder sw handling function");
        if (encoder_sw_action == LONG) {
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;
            else if (tuning_ctrl == SELECT) tuning_ctrl = OFF;
            else if (tuning_ctrl == OFF && dataset_page != LOCK) tuning_ctrl = SELECT; 
        }
        else {  // if short press
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
        }
        encoder_sw_action = NONE; // Our responsibility to reset this flag after handling events
    }
    if (encoder_delta != 0) {  // Now handle any new rotations
        if (serial_debugging) { Serial.print("in encoder rotation handler. dataset_page = "); Serial.println(dataset_page); }
        // int32_t spinrate = (int32_t)((double)(encoderSpinspeedTimer.elapsed())/(double)abs(encoder_delta));
        if (encoder_spinrate_isr_us >= encoder_spinrate_min_us) {  // Attempt to reject clicks coming in too fast
            encoder_spinrate_old_us = encoder_spinrate_last_us;  // Store last few spin times for filtering purposes ...
            encoder_spinrate_last_us = encoder_spinrate_us;  // ...
            encoder_spinrate_us = constrain(encoder_spinrate_isr_us, encoder_spinrate_min_us, 100000);
            int32_t spinrate_temp = (encoder_spinrate_old_us > encoder_spinrate_last_us) ? encoder_spinrate_old_us : encoder_spinrate_last_us;  // Find the slowest of the last 3 detents ...
            spinrate_temp = (spinrate_temp > encoder_spinrate_us) ? spinrate_temp : encoder_spinrate_us;  // to prevent one ultrafast double-hit to jump it too far
            encoder_edits_per_det = map(spinrate_temp, encoder_spinrate_min_us, 100000, 50, 1);  // if turning faster than 100ms/det, proportionally accelerate the effect of each detent by up to 50x 
            // encoderSpinspeedTimer.reset();
            if (tuning_ctrl == EDIT) sim_edit_delta_encoder = encoder_delta * encoder_edits_per_det;  // If a tunable value is being edited, turning the encoder changes the value
            else encoder_delta = constrain(encoder_delta, -1, 1);  // Only change one at a time when selecting
            if (tuning_ctrl == SELECT) selected_value += encoder_delta;  // If overflow constrain will fix in general handler below
            else if (tuning_ctrl == OFF) dataset_page += encoder_delta;  // If overflow tconstrain will fix in general below
        }
        encoder_delta = 0;
    }
    
    // Implement effects of changes made by encoder or touchscreen to ui_simulating, dataset_page, selected_value, or tuning_ctrl
    //
    sim_edit_delta = sim_edit_delta_encoder + sim_edit_delta_touch;  // Allow edits using the encoder

    if (tuning_ctrl != tuning_ctrl_last || dataset_page != dataset_page_last || selected_value != selected_value_last || sim_edit_delta != 0) tuningCtrlTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
    if (tuning_ctrl != OFF && tuningCtrlTimer.expired()) tuning_ctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
    dataset_page = constrain(dataset_page, 0, arraysize(pagecard)-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
    if (dataset_page != dataset_page_last) {
        if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;
        dataset_page_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
    }
    if (tuning_ctrl == SELECT) {
        if (dataset_page >= 4) selected_value = constrain (selected_value, 5, 7);  // Skip unchangeable values for all PID modes
        else if (dataset_page == JOY) selected_value = constrain (selected_value, 2, 7);  // Skip unchangeable values for joy mode
        else if (dataset_page == LOCK) selected_value = constrain (selected_value, 3, 7);  // Skip unchangeable values for joy mode
        else selected_value = constrain(selected_value, 0, arraysize(dataset_page_names[dataset_page])-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
        if (selected_value != selected_value_last) selected_val_dirty = true;
    }
    if (tuning_ctrl != tuning_ctrl_last || dataset_page_dirty) selected_val_dirty = true;
    tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
    
    if (tuning_ctrl == EDIT && sim_edit_delta != 0) {  // Change tunable values when editing
        if (dataset_page == LOCK)  switch (selected_value) {
            // case 2:  brakeSPID.set_proportionality((sim_edit_delta > 0) ? ERROR_TERM : SENSED_INPUT);  break;
            case 3:  ui_simulating[CTRL] = (sim_edit_delta != 0) ? (sim_edit_delta > 0) : ui_simulating[CTRL];  break;
            case 4:  if (!sim_edit_delta) sim_source_change(&ui_pot_addrs, &ui_simulating, BRKPOS, (sim_edit_delta > 0));  break;
            case 5:  if (!sim_edit_delta) sim_source_change(&ui_pot_addrs, &ui_simulating, PRESS, (sim_edit_delta > 0));  break;
            case 6:  if (!sim_edit_delta) sim_source_change(&ui_pot_addrs, &ui_simulating, TACH, (sim_edit_delta > 0));  break;
            case 7:  if (!sim_edit_delta) sim_source_change(&ui_pot_addrs, &ui_simulating, SPEEDO, (sim_edit_delta > 0));  break;
        }
        else if (dataset_page == JOY)  switch (selected_value) {
            case 2:  adj_val(&ctrl_lims_adc[ctrl][HORZ][MIN], sim_edit_delta, 0, adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][DB] / 2 - 1);  break;
            case 3:  adj_val(&ctrl_lims_adc[ctrl][HORZ][MAX], sim_edit_delta, adc_midscale_adc + ctrl_lims_adc[ctrl][HORZ][DB] / 2 + 1, adc_range_adc);  break;
            case 4:  adj_val(&ctrl_lims_adc[ctrl][HORZ][DB], sim_edit_delta, 0, (adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adc_midscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adc_midscale_adc) : 2*(adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]));  break;
            case 5:  adj_val(&ctrl_lims_adc[ctrl][VERT][MIN], sim_edit_delta, 0, adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][DB] / 2 - 1);  break;
            case 6:  adj_val(&ctrl_lims_adc[ctrl][VERT][MAX], sim_edit_delta, adc_midscale_adc + ctrl_lims_adc[ctrl][VERT][DB] / 2 + 1, adc_range_adc);  break;
            case 7:  adj_val(&ctrl_lims_adc[ctrl][VERT][DB], sim_edit_delta, 0, (adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adc_midscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adc_midscale_adc) : 2*(adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]));  break;
        }
        else if (dataset_page == CAR)  switch (selected_value) {
            case 0:  adj_val(&gas_governor_percent, sim_edit_delta, 0, 100);  break;
            case 1:  adj_val(&engine_idle_rpm, sim_edit_delta, 0, engine_redline_rpm -1);  break;
            case 2:  adj_val(&engine_redline_rpm, sim_edit_delta, engine_idle_rpm, 8000);  break;
            case 3:  adj_val(&carspeed_idle_mmph, sim_edit_delta, 0, carspeed_redline_mmph - 1);  break;
            case 4:  adj_val(&carspeed_redline_mmph, sim_edit_delta, carspeed_idle_mmph, 30000);  break;
            case 5:  ctrl = (sim_edit_delta != 0) ? (sim_edit_delta > 0) : ctrl;  break;
            case 6:  gasSPID.set_open_loop((sim_edit_delta != 0) ? (sim_edit_delta > 0) : gasSPID.get_open_loop());  break;
            case 7:  adj_val(&brake_pos_zeropoint_adc, sim_edit_delta, brake_pos_nom_lim_retract_adc, brake_pos_nom_lim_extend_adc);  break;
        }
        else if (dataset_page == PWMS)  switch (selected_value) {
            case 0:  adj_val(&steer_pulse_left_us, sim_edit_delta, pwm_pulse_min_us, steer_pulse_stop_us - 1);  break;
            case 1:  adj_val(&steer_pulse_stop_us, sim_edit_delta, steer_pulse_left_us + 1, steer_pulse_right_us - 1);  break;
            case 2:  adj_val(&steer_pulse_right_us, sim_edit_delta, steer_pulse_stop_us + 1, pwm_pulse_max_us);  break;
            case 3:  adj_val(&brake_pulse_extend_us, sim_edit_delta, brake_pulse_stop_us + 1, pwm_pulse_max_us);  break;
            case 4:  adj_val(&brake_pulse_stop_us, sim_edit_delta, brake_pulse_retract_us + 1, brake_pulse_extend_us - 1);  break;
            case 5:  adj_val(&brake_pulse_retract_us, sim_edit_delta, pwm_pulse_min_us, brake_pulse_stop_us -1);  break;
            case 6:  adj_val(&gas_pulse_idle_us, sim_edit_delta, gas_pulse_redline_us + 1, pwm_pulse_max_us - gas_pulse_park_slack_us);  break;
            case 7:  adj_val(&gas_pulse_redline_us, sim_edit_delta, pwm_pulse_min_us, gas_pulse_idle_us - 1);  break;
        }
        else if (dataset_page == BPID) {
            if (selected_value == 5) brakeSPID.set_tunings( brakeSPID.get_kp_1k()+(double)sim_edit_delta, brakeSPID.get_ki_mhz(), brakeSPID.get_kd_ms() );
            if (selected_value == 6) brakeSPID.set_tunings( brakeSPID.get_kp_1k(), brakeSPID.get_ki_mhz()+(double)sim_edit_delta, brakeSPID.get_kd_ms() );
            if (selected_value == 7) brakeSPID.set_tunings( brakeSPID.get_kp_1k(), brakeSPID.get_ki_mhz(), brakeSPID.get_kd_ms()+(double)sim_edit_delta );
        }
        else if (dataset_page == GPID) {
            if (selected_value == 5) gasSPID.set_tunings( gasSPID.get_kp_1k()+(double)sim_edit_delta, gasSPID.get_ki_mhz(), gasSPID.get_kd_ms() );
            if (selected_value == 6) gasSPID.set_tunings( gasSPID.get_kp_1k(), gasSPID.get_ki_mhz()+(double)sim_edit_delta, gasSPID.get_kd_ms() );
            if (selected_value == 7) gasSPID.set_tunings( gasSPID.get_kp_1k(), gasSPID.get_ki_mhz(), gasSPID.get_kd_ms()+(double)sim_edit_delta );
        }
        else if (dataset_page == CPID) {
            if (selected_value == 5) cruiseSPID.set_tunings( cruiseSPID.get_kp_1k()+(double)sim_edit_delta, cruiseSPID.get_ki_mhz(), cruiseSPID.get_kd_ms() );
            if (selected_value == 6) cruiseSPID.set_tunings( cruiseSPID.get_kp_1k(), cruiseSPID.get_ki_mhz()+(double)sim_edit_delta, cruiseSPID.get_kd_ms() );
            if (selected_value == 7) cruiseSPID.set_tunings( cruiseSPID.get_kp_1k(), cruiseSPID.get_ki_mhz(), cruiseSPID.get_kd_ms()+(double)sim_edit_delta );
        }
    }

    // Do any needed actions due to changes made
    if (ignition != ignition_last) {  // Car was turned on or off
        digitalWrite(ignition_pin, ignition); // Make it real
        if (!ignition && carspeed_filt_mmph) panic_stop = true;
    }
    ignition_last = ignition; // Make sure this goes after the last comparison
    if (!carspeed_filt_mmph) panic_stop = false;

    // Update displayed telemetry values to the screen
    if (display_enabled)  {
        if (ui_simulating[GLOBAL] != ui_simulating[LAST]) draw_simbuttons(ui_simulating[GLOBAL]);  // if we just entered simulator draw the simulator buttons, or if we just left erase them
        ui_simulating[LAST] = ui_simulating[GLOBAL];
        if (dataset_page_dirty) {
            draw_fixed(true);
            draw_dynamic(0, 0, -1, -1, 2);
        }
        if (selected_val_dirty) draw_dynamic(0, 0, -1, -1, 3);
        dataset_page_dirty = false;
        selected_val_dirty = false;
        int32_t range;
        // draw_fixed();
        // if (ui_simulating) draw_touchgrid(); // Redraw only the at-risk content of the touch grid
        draw_dynamic(0, runmode, -1, -1, 1);
        draw_dyn_pid(1, carspeed_filt_mmph, 0, carspeed_redline_mmph, (int32_t)cruiseSPID.get_target(), 4);
        draw_dyn_pid(2, engine_filt_rpm, 0, engine_redline_rpm, (int32_t)gasSPID.get_target(), 4);
        draw_dyn_pid(3, pressure_filt_adc, pressure_min_adc, pressure_max_adc, (int32_t)brakeSPID.get_target(), 4);  // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.get_target() : pressure_target_adc, 4);
        draw_dynamic(4, ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX], 0);
        draw_dynamic(5, ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX], 0);
        draw_dynamic(6, (int32_t)cruiseSPID.get_target(), 0, carspeed_govern_mmph, 0);
        draw_dynamic(7, (int32_t)brakeSPID.get_target(), pressure_min_adc, pressure_max_adc, 0);
        draw_dynamic(8, (int32_t)gasSPID.get_target(), 0, engine_redline_rpm, 0);
        draw_dynamic(9, brake_pulse_out_us, brake_pulse_extend_us, brake_pulse_retract_us, 0);
        draw_dynamic(10, gas_pulse_out_us, gas_pulse_idle_us, gas_pulse_redline_us, 0);
        draw_dynamic(11, steer_pulse_out_us, steer_pulse_left_us, steer_pulse_right_us, 0);
        if (dataset_page == LOCK) {
            draw_dynamic(12, battery_filt_mv, 0, battery_max_mv, 0);
            draw_dynamic(13, brake_pos_filt_adc, brake_pos_nom_lim_retract_adc, brake_pos_nom_lim_extend_adc, 0);
            draw_dynamic(14, pot_filt_adc, pot_min_adc, pot_max_adc, 0);
            // draw_dynamic(14, brakeSPID.get_proportionality(), -1, -1, 0);
            draw_dynamic(15, ui_simulating[CTRL], -1, -1, 0);
            draw_dynamic(16, ui_simulating[BRKPOS], -1, -1, 0);
            draw_dynamic(17, ui_simulating[PRESS], -1, -1, 0);
            draw_dynamic(18, ui_simulating[TACH], -1, -1, 0);
            draw_dynamic(19, ui_simulating[SPEEDO], -1, -1, 0);
        }
        else if (dataset_page == JOY) {
            draw_dynamic(12, ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX], 0);
            draw_dynamic(13, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX], 0);
            draw_dynamic(14, ctrl_lims_adc[ctrl][HORZ][MIN], 0, (adc_range_adc-ctrl_lims_adc[ctrl][HORZ][MAX])/2, 0);
            draw_dynamic(15, ctrl_lims_adc[ctrl][HORZ][MAX], (ctrl_lims_adc[ctrl][HORZ][MIN]-adc_range_adc)/2, adc_range_adc, 0);
            draw_dynamic(16, ctrl_lims_adc[ctrl][HORZ][DB], 0, (adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adc_midscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adc_midscale_adc) : 2*(adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]), 0);
            draw_dynamic(17, ctrl_lims_adc[ctrl][VERT][MIN], 0, (adc_range_adc-ctrl_lims_adc[ctrl][VERT][MAX])/2, 0);
            draw_dynamic(18, ctrl_lims_adc[ctrl][VERT][MAX], (ctrl_lims_adc[ctrl][VERT][MIN]-adc_range_adc)/2, adc_range_adc, 0);
            draw_dynamic(19, ctrl_lims_adc[ctrl][VERT][DB], 0, (adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adc_midscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adc_midscale_adc) : 2*(adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]), 0);
        }
        else if (dataset_page == CAR) {
            draw_dynamic(12, gas_governor_percent, 0, 100, 0);
            draw_dynamic(13, engine_idle_rpm, 0, engine_redline_rpm, 0);
            draw_dynamic(14, engine_redline_rpm, 0, engine_max_rpm, 0);
            draw_dynamic(15, carspeed_idle_mmph, 0, carspeed_redline_mmph, 0);
            draw_dynamic(16, carspeed_redline_mmph, 0, carspeed_max_mmph, 0);
            draw_dynamic(17, ctrl, -1, -1, 0);  // 0 if hotrc
            draw_dynamic(18, gasSPID.get_open_loop(), -1, -1, 0);
            draw_dynamic(19, brake_pos_zeropoint_adc, brake_pos_nom_lim_retract_adc, brake_pos_nom_lim_extend_adc, 0);   
        }
        else if (dataset_page == PWMS) {
            draw_dynamic(12, steer_pulse_left_us, steer_pulse_left_max_us, steer_pulse_stop_us, 0);
            draw_dynamic(13, steer_pulse_stop_us, steer_pulse_left_us, steer_pulse_right_us, 0);
            draw_dynamic(14, steer_pulse_right_us, steer_pulse_stop_us, steer_pulse_right_max_us, 0);
            draw_dynamic(15, brake_pulse_extend_us, brake_pulse_stop_us, brake_pulse_extend_max_us, 0);
            draw_dynamic(16, brake_pulse_stop_us, brake_pulse_retract_us, brake_pulse_extend_us, 0);
            draw_dynamic(17, brake_pulse_retract_us, brake_pulse_retract_max_us, brake_pulse_stop_us, 0);
            draw_dynamic(18, gas_pulse_idle_us, gas_pulse_ccw_max_us, gas_pulse_cw_max_us, 0);
            draw_dynamic(19, gas_pulse_redline_us, gas_pulse_ccw_max_us, gas_pulse_cw_max_us, 0);
        }
        else if (dataset_page == BPID) {
            range = pressure_max_adc-pressure_min_adc;
            draw_dynamic(12, (int32_t)(brakeSPID.get_error()), -range, range, 0);
            draw_dynamic(13, (int32_t)(brakeSPID.get_p_term()), -range, range, 0);
            draw_dynamic(14, (int32_t)(brakeSPID.get_i_term()), -range, range, 0);
            draw_dynamic(15, (int32_t)(brakeSPID.get_d_term()), -range, range, 0);
            draw_dynamic(16, (int32_t)(brakeSPID.get_delta()), -range, range, 0);  // brake_carspeed_delta_adc, -range, range, 0);
            draw_dynamic(17, brakeSPID.disp_kp_1k, 0, 1000, 0);  // Real value is 1000 * smaller than displayed
            draw_dynamic(18, brakeSPID.disp_ki_mhz, 0, 1000, 0);
            draw_dynamic(19, brakeSPID.disp_kd_ms, 0, 1000, 0);
        }
        else if (dataset_page == GPID) {
            range = engine_govern_rpm-engine_idle_rpm;
            draw_dynamic(12, (int32_t)(gasSPID.get_error()), -range, range, 0);
            draw_dynamic(13, (int32_t)(gasSPID.get_p_term()), -range, range, 0);
            draw_dynamic(14, (int32_t)(gasSPID.get_i_term()), -range, range, 0);
            draw_dynamic(15, (int32_t)(gasSPID.get_d_term()), -range, range, 0);
            draw_dynamic(16, (int32_t)(gasSPID.get_delta()), -range, range, 0);  // gas_carspeed_delta_adc, -range, range, 0);
            draw_dynamic(17, gasSPID.disp_kp_1k, 0, 1000, 0);  // Real value is 1000 * smaller than displayed
            draw_dynamic(18, gasSPID.disp_ki_mhz, 0, 1000, 0);
            draw_dynamic(19, gasSPID.disp_kd_ms, 0, 1000, 0);
        }
        else if (dataset_page == CPID) {
            range = carspeed_govern_mmph-carspeed_idle_mmph;
            draw_dynamic(12, (int32_t)(cruiseSPID.get_error()), -range, range, 0);
            draw_dynamic(13, (int32_t)(cruiseSPID.get_p_term()), -range, range, 0);
            draw_dynamic(14, (int32_t)(cruiseSPID.get_i_term()), -range, range, 0);
            draw_dynamic(15, (int32_t)(cruiseSPID.get_d_term()), -range, range, 0);
            draw_dynamic(16, (int32_t)(cruiseSPID.get_delta()), -range, range, 0);  // cruise_carspeed_delta_adc, -range, range, 0);
            draw_dynamic(17, cruiseSPID.disp_kp_1k, 0, 1000, 0);  // Real value is 1000 * smaller than displayed
            draw_dynamic(18, cruiseSPID.disp_ki_mhz, 0, 1000, 0);
            draw_dynamic(19, cruiseSPID.disp_kd_ms, 0, 1000, 0);
        }
        draw_bool(basicmodesw, 0);
        draw_bool(ignition, 1);
        draw_bool(cruise_sw, 2);
    }
    dataset_page_last = dataset_page;
    selected_value_last = selected_value;
    
    // 8) Do the control loop bookkeeping at the end of each loop
    //
    sim_edit_delta_touch = 0;
    sim_edit_delta_encoder = 0;
    disp_redraw_all = false;
    loopno++;  // I like to count how many loops
    if (runmode != SHUTDOWN) shutdown_complete = false;
    if (runmode != oldmode) we_just_switched_modes = true;      // If changing runmode, set this so new mode logic can perform initial actions
    else we_just_switched_modes = false;    // Reset this variable
    oldmode = runmode;   // remember what mode we're in for next time
    loop_period_us = loopTimer.elapsed();  // abs is to handle when mycros() overflows back to 0
    if (!loop_period_us) loop_period_us++;  // ensure loop period is never zero since it gets divided by
    loop_freq_hz = (int32_t)(1000000/(double)loop_period_us);
    loopTimer.reset();
}