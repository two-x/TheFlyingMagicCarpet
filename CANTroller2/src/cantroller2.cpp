// Carpet CANTroller II  Source Code  - For ESP32-S3-DevKitC-1-N8
#include <SPI.h>  // SPI serial bus
#include <Adafruit_SleepyDog.h>  // Watchdog
#include <vector>
#include <iomanip>  // Formatting cout
#include "globals.h"
#include "display.h"
#include "uictrl.h"
#include "TouchScreen.h"
#include "RunModeManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

std::vector<std::string> loop_names(20);

void loop_savetime (uint32_t timesarray[], int32_t &index, std::vector<std::string> &names, bool dirty[], std::string loopname) {  // (int32_t timesarray[], int32_t index) {
    if (dirty[index]) {
        names[index] = loopname;  // names[index], name);
        dirty[index] = false;
    }
    timesarray[index] = esp_timer_get_time();
    index++;
}

HotrcManager hotrcHorzManager (6);
HotrcManager hotrcVertManager (6);
RunModeManager runModeManager;
Display screen;
ESP32PWM pwm;  // Object for timer pwm resources (servo outputs)


#ifdef CAP_TOUCH
TouchScreen ts;
#else
TouchScreen ts(touch_cs_pin, touch_irq_pin);
#endif
    
#define RUN_TESTS 0
#if RUN_TESTS
#include "tests.h"
void run_tests() {
        printf("Running tests...\n");
        delay(5000);
        test_Param();
        printf("Tests complete.\n");
        for(;;) {} // loop forever
}
#else
void run_tests() {}
#endif

void setup() {  // Setup just configures pins (and detects touchscreen type)
    if (RUN_TESTS) {
        run_tests();
    }    

    set_pin (tft_dc_pin, OUTPUT);
    set_pin (gas_pwm_pin, OUTPUT);
    set_pin (basicmodesw_pin, INPUT_PULLUP);
    set_pin (neopixel_pin, OUTPUT);
    set_pin (sdcard_cs_pin, OUTPUT);
    set_pin (tft_cs_pin, OUTPUT);
    set_pin (multibutton_pin, INPUT_PULLUP);
    set_pin (starter_pin, INPUT_PULLDOWN);
    set_pin (joy_horz_pin, INPUT);
    set_pin (joy_vert_pin, INPUT);
    set_pin (touch_irq_pin, INPUT_PULLUP);
    set_pin (ign_out_pin, OUTPUT);
    set_pin (syspower_pin, OUTPUT);  // Then set the put as an output as normal.
    #ifdef pwm_jaguars
        set_pin (brake_pwm_pin, OUTPUT);
        set_pin (steer_pwm_pin, OUTPUT);
    #else
        // Init serial port uart 1
    #endif
    analogReadResolution (adcbits);  // Set Arduino Due to 12-bit resolution (default is same as Mega=10bit)

    write_pin (tft_cs_pin, HIGH);   // Prevent bus contention
    write_pin (sdcard_cs_pin, HIGH);   // Prevent bus contention
    write_pin (tft_dc_pin, LOW);
    write_pin (ign_out_pin, LOW);
    write_pin (syspower_pin, syspower);
    
    // Calculate some derived variables
    calc_ctrl_lims();
    calc_governor();
    for (int32_t x=0; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;

    Serial.begin (115200);  // Open console serial port
    delay (800);  // This is needed to allow the uart to initialize and the screen board enough time after a cold boot
    printf ("Console started..\n");
    
    // Set up 4 RMT receivers, one per channel
    printf ("Init rmt for hotrc..\n");
    hotrc_vert.init();
    hotrc_horz.init();
    hotrc_ch3.init();
    hotrc_ch4.init();
    
    printf("Pot setup..\n");
    pot.setup();

    printf("Encoder setup..\n");
    encoder.setup();

    printf("Transducers setup..\n");
    pressure_sensor.setup();
    brkpos_sensor.setup();
    battery_sensor.setup();
    tachometer.setup();
    speedometer.setup();

    printf ("Init i2c and i2c-enabled devices..");
    i2c.init();
    airflow_sensor.setup(); // must be done after i2c is started
    map_sensor.setup();

    printf("Simulator setup..\n");
    simulator.register_device(SimOption::pressure, pressure_sensor, pressure_sensor.source());
    simulator.register_device(SimOption::brkpos, brkpos_sensor, brkpos_sensor.source());
    simulator.register_device(SimOption::airflow, airflow_sensor, airflow_sensor.source());
    simulator.register_device(SimOption::mapsens, map_sensor, map_sensor.source());
    simulator.register_device(SimOption::tach, tachometer, tachometer.source());
    simulator.register_device(SimOption::speedo, speedometer, speedometer.source());

    printf ("Configure timers for PWM out..\n");
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	brake_servo.setPeriodHertz(50);
    gas_servo.setPeriodHertz(49);
	steer_servo.setPeriodHertz(50);

    brake_servo.attach (brake_pwm_pin, brake_pulse_extend_us, brake_pulse_retract_us);  // Jag input PWM range default is 670us (full reverse) to 2330us (full fwd). Max range configurable is 500-2500us
    steer_servo.attach (steer_pwm_pin, steer_pulse_left_us, steer_pulse_right_us);  // Jag input PWM range default is 670us (full reverse) to 2330us (full fwd). Max range configurable is 500-2500us
    gas_servo.attach (gas_pwm_pin, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    // Servo() argument 2 is channel (0-15) of the esp timer (?). set to Servo::CHANNEL_NOT_ATTACHED to auto grab a channel
    // gas_servo.setup();
    // gas_servo.set_native_limits();  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)

    printf ("Init neopixel..\n");
    neo_heartbeat = (neopixel_pin >= 0);
    neostrip.begin();  // start datastream
    neostrip.show();  // Turn off the pixel
    neostrip.setBrightness (neo_brightness_max);  // It truly is incredibly bright
    

    temperature_sensor_manager.setup();  // Onewire bus and temp sensors
    
    throttle.setup(temperature_sensor_manager.get_sensor(sensor_location::ENGINE));
    // Create a new task that runs the update_temperature_sensors function
    xTaskCreate(update_temperature_sensors, "Update Temperature Sensors", 2048, NULL, 5, NULL);

    
    printf ("Init display..\n");
    if (display_enabled) {
        config.begin("FlyByWire", false);
        dataset_page = config.getUInt("dpage", PG_RUN);
        dataset_page_last = config.getUInt("dpage", PG_TEMP);
        screen.init();
        ts.init();
    }


    int32_t watchdog_time_ms = Watchdog.enable(2500);  // Start 2.5 sec watchdog
    printf ("Enable watchdog.. timer set to %ld ms\n", watchdog_time_ms);
    hotrcPanicTimer.reset();
    loopTimer.reset();  // start timer to measure the first loop
    booted = true;
    printf ("Setup done\n");
}

void loop() {
    loopindex = 0;  // reset at top of loop
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "top");
    // cout << "(top)) spd:" << speedo_filt_mph << " tach:" << tach_filt_rpm;

    // Update inputs.  Fresh sensor data, and filtering.
    //

    // ESP32 "boot" button. generates boot_button_action flags of LONG or SHORT presses which can be handled wherever. Handler must reset boot_button_action = NONE
    if (!read_pin (multibutton_pin)) {
        if (!boot_button) {  // If press just occurred
            dispResetButtonTimer.reset();  // Looks like someone just pushed the esp32 "boot" button
            boot_button_timer_active = true;  // flag to indicate timing for a possible long press
        }
        else if (boot_button_timer_active && dispResetButtonTimer.expired()) {
            boot_button_action = LONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
            boot_button_timer_active = false;  // Clear timer active flag
            boot_button_suppress_click = true;  // Prevents the switch release after a long press from causing a short press
        }
        boot_button = true;  // Store press is in effect
    }
    else {  // if button is not being pressed
        if (boot_button && !boot_button_suppress_click) boot_button_action = SHORT;  // if the button was just released, a short press occurred, which must be handled
        // else boot_button_action = NONE;  // This would auto-reset the button action flag but require it get handled in this loop. Otherwise the handler must set this
        boot_button_timer_active = false;  // Clear timer active flag
        boot_button = false;  // Store press is not in effect
        boot_button_suppress_click = false;  // End click suppression
    }
    
    // External digital signals - takes 11 us to read
    if (!simulator.simulating(SimOption::basicsw)) {
        do {
            basicmodesw = !digitalRead(basicmodesw_pin);   // 1-value because electrical signal is active low
        } while (basicmodesw != !digitalRead(basicmodesw_pin)); // basicmodesw pin has a tiny (39ns) window in which it can get invalid values, so read it twice to be sure
    }
    // if (ctrl == JOY && (!simulator.get_enabled() || !sim_cruisesw)) cruise_sw = digitalRead (joy_cruise_btn_pin);

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pre");


    encoder.update();  // Read encoder input signals

    pot.update();
    
    // Brake position - takes 70 us to read, convert, and filter
    brkpos_sensor.update();
    
    if (!starter_signal_support) starter = LOW;
    else if (!simulator.simulating(SimOption::starter)) starter = read_pin (starter_pin);

    // Tach - takes 22 us to read when no activity
    tachometer.update();
    throttle.push_tach_reading(tachometer.get_human(), tachometer.get_last_read_time());

    // Airflow sensor
    airflow_sensor.update();

    // MAP sensor
    map_sensor.update();

    // Speedo - takes 14 us to read when no activity
    speedometer.update();

    // Brake pressure - takes 72 us to read
    pressure_sensor.update();

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "inp");  //

    // Read the car ignition signal, and while we're at it measure the vehicle battery voltage off ign signal
    battery_sensor.update();
    ignition_sense = read_battery_ignition();  // Updates battery voltage reading and returns ignition status

    // Controller handling
    //
    if (ctrl == JOY) ignition = simulator.simulating(SimOption::ignition) ? ignition : ignition_sense;
    else if (ctrl == HOTRC) {
        hotrc_ch3_update();
        hotrc_ch4_update();
        if (hotrc_ch3_sw_event) {  // Turn on/off the vehicle ignition. If ign is turned off while the car is moving, this leads to panic stop
            if (hotrc_suppress_next_ch3_event) hotrc_suppress_next_ch3_event = false;
            else ignition = simulator.simulating(SimOption::ignition) ? ignition : !ignition;
            hotrc_ch3_sw_event = false;
        }
        if (hotrc_ch4_sw_event) {
            if (hotrc_suppress_next_ch4_event) hotrc_suppress_next_ch4_event = false;
            else if (runmode == STALL && remote_start_support) remote_start_toggle_request = true;
            else if (runmode == FLY || runmode == CRUISE) flycruise_toggle_request = true;
            else {}   // There's no reason pushing the ch4 button when in other modes can't do something different.  That would go here
            hotrc_ch4_sw_event = false;    
        }
        if (hotrc_vert_pulse_us > hotrc_pulse_failsafe_max_us) {
            hotrcPanicTimer.reset();
            hotrc_radio_detected = true;
            if (!ignition_output_enabled) ignition_output_enabled = true; // Ignition stays low until the hotrc is detected here, then output is allowed
            // set_pin (ignition_pin, OUTPUT);  // do NOT plug in the joystick when using the hotrc to avoid ign contention
        }
        else if (hotrc_radio_detected && hotrcPanicTimer.expired()) hotrc_radio_detected = false;
        // hotrc_suppress_next_ch3_event = true;  // reject spurious ch3 switch event upon next hotrc poweron
        // hotrc_suppress_next_ch4_event = true;  // reject spurious ch4 switch event upon next hotrc poweron
    }

    // Read horz and vert inputs, determine steering pwm output -  - takes 40 us to read. Then, takes 13 us to handle
    if (ctrl != JOY) {
        if (hotrc_source == ESP_RMT) {  // Read RMT pulse widths
            hotrc_horz_pulse_us = (int32_t)hotrc_horz.readPulseWidth();  
            hotrc_vert_pulse_us = (int32_t)hotrc_vert.readPulseWidth();
        }
        else {
            hotrc_horz_pulse_us = (int32_t)hotrc_horz_pulse_64_us;
            hotrc_vert_pulse_us = (int32_t)hotrc_vert_pulse_64_us;
        }
        hotrc_horz_pulse_us = hotrcHorzManager.spike_filter (hotrc_horz_pulse_us);
        hotrc_vert_pulse_us = hotrcVertManager.spike_filter (hotrc_vert_pulse_us);
        ema_filt (hotrc_vert_pulse_us, &hotrc_vert_pulse_filt_us, ctrl_ema_alpha[HOTRC]);  // Used to detect loss of radio
        // ema_filt (hotrc_horz_pulse_us, &hotrc_horz_pulse_filt_us, ctrl_ema_alpha[HOTRC]);  // Just here for debugging. Do not need filtered horz value
    }

    if (simulator.can_simulate(SimOption::joy) && simulator.get_pot_overload() == SimOption::joy) {
        ctrl_pos_adc[HORZ][FILT] = pot.mapToRange(steer_pulse_left_us, steer_pulse_right_us);
    } else if (!simulator.simulating(SimOption::joy)) {  // Handle HotRC button generated events and detect potential loss of radio signal
        if (ctrl == HOTRC) {
            if (hotrc_horz_pulse_us >= hotrc_pulse_lims_us[HORZ][CENT])  // Steering: Convert from pulse us to joystick adc equivalent, when pushing right, else pushing left
                ctrl_pos_adc[HORZ][RAW] = map (hotrc_horz_pulse_us, hotrc_pulse_lims_us[HORZ][CENT], hotrc_pulse_lims_us[HORZ][MAX], ctrl_lims_adc[ctrl][HORZ][CENT], ctrl_lims_adc[ctrl][HORZ][MAX]);
            else ctrl_pos_adc[HORZ][RAW] = map (hotrc_horz_pulse_us, hotrc_pulse_lims_us[HORZ][CENT], hotrc_pulse_lims_us[HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][CENT], ctrl_lims_adc[ctrl][HORZ][MIN]);
            if (hotrc_vert_pulse_us >= hotrc_pulse_lims_us[VERT][CENT])  // Trigger: Convert from pulse us to joystick adc equivalent, for trigger pull, else trigger push
                ctrl_pos_adc[VERT][RAW] = map (hotrc_vert_pulse_us, hotrc_pulse_lims_us[VERT][CENT], hotrc_pulse_lims_us[VERT][MAX], ctrl_lims_adc[ctrl][VERT][CENT], ctrl_lims_adc[ctrl][VERT][MAX]);
            else ctrl_pos_adc[VERT][RAW] = map (hotrc_vert_pulse_us, hotrc_pulse_lims_us[VERT][CENT], hotrc_pulse_lims_us[VERT][MIN], ctrl_lims_adc[ctrl][VERT][CENT], ctrl_lims_adc[ctrl][VERT][MIN]);  
        }
        else if (ctrl == JOY) {
            ctrl_pos_adc[VERT][RAW] = analogRead (joy_vert_pin);  // Read joy vertical
            ctrl_pos_adc[HORZ][RAW] = analogRead (joy_horz_pin);  // Read joy horizontal
        }

        // // If controller axis is wildly out of range, disregard reading
        // if (ctrl_pos_adc[HORZ][RAW] < ctrl_lims_adc[ctrl][HORZ][MIN] - ctrl_lims_adc[ctrl][HORZ][MARGIN] || 
        //     ctrl_pos_adc[HORZ][RAW] > ctrl_lims_adc[ctrl][HORZ][MAX] + ctrl_lims_adc[ctrl][HORZ][MARGIN]) {
        //     // Wildly out of range value maybe should set an error flag?  If hotrc, then this happens to HORZ axis when you turn it off.
        //     ctrl_pos_adc[HORZ][RAW] = ctrl_pos_adc[HORZ][FILT];  //  = ctrl_lims_adc[ctrl][HORZ][CENT];
        // }
        // if (ctrl_pos_adc[VERT][RAW] < ctrl_lims_adc[ctrl][VERT][MIN] - ctrl_lims_adc[ctrl][VERT][MARGIN] || 
        //     ctrl_pos_adc[VERT][RAW] > ctrl_lims_adc[ctrl][VERT][MAX] + ctrl_lims_adc[ctrl][VERT][MARGIN]) {
        //     ctrl_pos_adc[VERT][RAW] = ctrl_pos_adc[VERT][FILT];  //  = ctrl_lims_adc[ctrl][VERT][CENT];
        // }

        if (ctrl == HOTRC && !hotrc_radio_detected) {
            ctrl_pos_adc[HORZ][FILT] = ctrl_lims_adc[ctrl][HORZ][CENT];
            ctrl_pos_adc[VERT][FILT] = ctrl_lims_adc[ctrl][VERT][CENT];
        }
        else {
            ema_filt (ctrl_pos_adc[VERT][RAW], &ctrl_pos_adc[VERT][FILT], ctrl_ema_alpha[ctrl]);  // do ema filter to determine joy_vert_filt
            ema_filt (ctrl_pos_adc[HORZ][RAW], &ctrl_pos_adc[HORZ][FILT], ctrl_ema_alpha[ctrl]);  // do ema filter to determine joy_horz_filt
            ctrl_pos_adc[VERT][FILT] = constrain (ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
            ctrl_pos_adc[HORZ][FILT] = constrain (ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
        }

        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) {
            ctrl_pos_adc[VERT][FILT] = ctrl_lims_adc[ctrl][VERT][CENT];  // if joy vert is in the deadband, set joy_vert_filt to center value
        }
        if (ctrl_pos_adc[HORZ][FILT] > ctrl_db_adc[HORZ][BOT] && ctrl_pos_adc[HORZ][FILT] < ctrl_db_adc[HORZ][TOP]) {
            ctrl_pos_adc[HORZ][FILT] = ctrl_lims_adc[ctrl][HORZ][CENT];  // if joy horz is in the deadband, set joy_horz_filt to center value
        }
    }

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "joy");  //
    
    runmode = runModeManager.handle_runmode();  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "mod");  //

    // Update motor outputs - takes 185 us to handle every 30ms when the pid timer expires, otherwise 5 us
    //

    // Steering - Determine motor output and send to the motor
    if (steerPidTimer.expireset()) {
        if (runmode == SHUTDOWN && shutdown_complete) steer_out_percent = steer_stop_percent;  // Stop the steering motor if in shutdown mode and shutdown is complete
        else if (ctrl_pos_adc[HORZ][FILT] >= ctrl_db_adc[HORZ][TOP])  // If above the top edge of the deadband, turning right
            steer_out_percent = map ((float)ctrl_pos_adc[HORZ][FILT], (float)ctrl_db_adc[HORZ][TOP], (float)ctrl_lims_adc[ctrl][HORZ][MAX], steer_stop_percent, steer_safe (steer_right_percent));  // Figure out the steering setpoint if joy to the right of deadband
        else if (ctrl_pos_adc[HORZ][FILT] <= ctrl_db_adc[HORZ][BOT])  // If below the bottom edge of the deadband, turning left
            steer_out_percent = map ((float)ctrl_pos_adc[HORZ][FILT], (float)ctrl_db_adc[HORZ][BOT], (float)ctrl_lims_adc[ctrl][HORZ][MIN], steer_stop_percent, steer_safe (steer_left_percent));  // Figure out the steering setpoint if joy to the left of deadband
        else steer_out_percent = steer_stop_percent;  // Stop the steering motor if inside the deadband
        steer_out_percent = constrain (steer_out_percent, steer_left_percent, steer_right_percent);  // Don't be out of range
        
        if (steer_out_percent >= steer_stop_percent)
             steer_pulse_out_us = map (steer_out_percent, steer_stop_percent, steer_right_percent, steer_pulse_stop_us, steer_pulse_right_us);
        else steer_pulse_out_us = map (steer_out_percent, steer_stop_percent, steer_left_percent, steer_pulse_stop_us, steer_pulse_left_us);
        
        #ifdef pwm_jaguars
            steer_servo.writeMicroseconds ((int32_t)steer_pulse_out_us);   // Write steering value to jaguar servo interface
        #else
            // Send command over serial port
        #endif
    }
    // Brakes - Determine motor output and write it to motor
    if (brakePidTimer.expireset()) {

        // Step 1 : Determine motor percent value
        if (runmode == SHUTDOWN && shutdown_complete)
            brake_out_percent = brake_stop_percent; // if we're shutdown, stop the motor
        else if (runmode == CAL && cal_joyvert_brkmotor) {
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) brake_out_percent = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], brake_stop_percent, brake_retract_percent);
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) brake_out_percent = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_lims_adc[ctrl][VERT][MIN], (float)ctrl_db_adc[VERT][BOT], brake_extend_percent, brake_stop_percent);
            else brake_out_percent = (float)brake_stop_percent;
        }
        else if (park_the_motors) {
            if (brkpos_sensor.get_filtered_value() + BrakePositionSensor::margin_in <= BrakePositionSensor::park_in)  // If brake is retracted from park point, extend toward park point, slowing as we approach
                brake_out_percent = map (brkpos_sensor.get_filtered_value(), BrakePositionSensor::park_in, BrakePositionSensor::nom_lim_retract_in, brake_stop_percent, brake_extend_percent);
            else if (brkpos_sensor.get_filtered_value() - BrakePositionSensor::margin_in >= BrakePositionSensor::park_in)  // If brake is extended from park point, retract toward park point, slowing as we approach
                brake_out_percent = map (brkpos_sensor.get_filtered_value(), BrakePositionSensor::park_in, BrakePositionSensor::nom_lim_extend_in, brake_stop_percent, brake_retract_percent);
        }
        else if (runmode != BASIC) brakeQPID.Compute();  // Otherwise the pid control is active
        
        // Step 2 : Fix motor percent value if it's out of range or exceeding positional limits
        if (runmode == CAL && cal_joyvert_brkmotor)  // Constrain the motor to the operational range, unless calibrating (then constraint already performed above)
            brake_out_percent = constrain (brake_out_percent, brake_extend_min_percent, brake_retract_max_percent);  // Constrain to full potential range when calibrating. Caution don't break anything!
        else if ((brake_out_percent < brake_stop_percent && brkpos_sensor.get_filtered_value() > BrakePositionSensor::park_in - BrakePositionSensor::margin_in) || (brake_out_percent > brake_stop_percent && brkpos_sensor.get_filtered_value() < BrakePositionSensor::nom_lim_retract_in + BrakePositionSensor::margin_in))  // If brake is at position limits and we're tring to go further, stop the motor
            brake_out_percent = brake_stop_percent;
        else brake_out_percent = constrain (brake_out_percent, brake_extend_percent, brake_retract_percent);  // Send to the actuator. Refuse to exceed range

        // Step 3 : Convert motor percent value to pulse width
        if (brake_out_percent >= brake_stop_percent)
            // brake_pulse_out_us = map (brake_out_percent, brake_stop_percent, brake_extend_percent, brake_pulse_stop_us, brake_pulse_extend_us);
            brake_pulse_out_us = map (brake_out_percent, brake_stop_percent, brake_retract_percent, brake_pulse_stop_us, brake_pulse_retract_us);
         //else brake_pulse_out_us = map (brake_out_percent, brake_stop_percent, brake_retract_percent, brake_pulse_stop_us, brake_pulse_retract_us);
        else brake_pulse_out_us = map (brake_out_percent, brake_stop_percent, brake_extend_percent, brake_pulse_extend_us, brake_pulse_stop_us);

        // Step 4 : Write to motor
        if (runmode != BASIC || park_the_motors) {
            #ifdef pwm_jaguars
                brake_servo.writeMicroseconds ((int32_t)brake_pulse_out_us);  // Write result to jaguar servo interface
            #else
                // Send command over serial port
            #endif
        }
    }
    
    throttle.update();  // Allow idle control to mess with tach_target if necessary, or otherwise step in to prevent car from stalling

    // Cruise - Update gas target. Controls gas rpm target to keep speed equal to cruise mph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
    if (runmode == CRUISE && !cruise_adjusting && !cruise_fixed_throttle && cruisePidTimer.expireset()) {
        cruiseQPID.SetOutputLimits (throttle.get_idlespeed(), tach_govern_rpm);  // because cruise pid has internal variable for idlespeed which may need updating
        tach_target_rpm = throttle.get_target();
        cruiseQPID.Compute();
        throttle.set_target(tach_target_rpm);  // Need for pid math to be be read by gas control handler below
    }

    // Gas - Update servo output. Determine gas actuator output from rpm target.  PID loop is effective in Fly or Cruise mode.
    if (gasPidTimer.expireset()) {
        // Step 1 : Determine servo pulse width value
        if (park_the_motors || (runmode == SHUTDOWN && shutdown_complete))
            gas_pulse_out_us = gas_pulse_ccw_closed_us + gas_pulse_park_slack_us;
        else if (runmode == STALL) {  // Stall mode runs the gas servo directly proportional to joystick. This is truly open loop
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) gas_pulse_out_us = gas_pulse_ccw_closed_us;  // If in deadband or being pushed down, we want idle
            else gas_pulse_out_us = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], gas_pulse_ccw_closed_us, gas_pulse_govern_us);  // Actuators still respond and everything, even tho engine is turned off
        }
        else if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo)
            gas_pulse_out_us = map (pot.get(), pot.min(), pot.max(), gas_pulse_ccw_max_us, gas_pulse_cw_min_us);
        else if (runmode == CRUISE && cruise_fixed_throttle)
            gas_pulse_out_us = gas_pulse_cruise_us;
        else if (runmode != BASIC) {
            if (gasQPID.GetMode() == (uint8_t)QPID::Control::manual)  // This isn't really open loop, more like simple proportional control, with output set proportional to target 
                gas_pulse_out_us = map (throttle.get_target(), throttle.get_idlespeed(), tach_govern_rpm, gas_pulse_ccw_closed_us, gas_pulse_govern_us); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
            else {
                // if (boot_button) printf("gpid: gpo:%lf tt:%lf tf:%lf\n", gas_pulse_out_us, throttle.get_target(), tachometer.get_filtered_value());
                tach_target_rpm = throttle.get_target();
                gasQPID.Compute();  // Do proper pid math to determine gas_pulse_out_us from engine rpm error
            }
        }
        // Step 2 : Constrain if out of range
        if (runmode == BASIC || runmode == SHUTDOWN)
            gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_govern_us, gas_pulse_ccw_closed_us + gas_pulse_park_slack_us);
        else if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo)  // Constrain to operating limits. 
            gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
        else gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_govern_us, gas_pulse_ccw_closed_us);

        // Step 3 : Write to servo
        if (runmode != BASIC || park_the_motors) {
            gas_servo.writeMicroseconds ((int32_t)gas_pulse_out_us);  // Write result to servo
            // if (boot_button) printf (" Gas:%4ld\n", (int32_t)gas_pulse_out_us);
        }
    }

    if (park_the_motors) {  //  When parking motors, IF the timeout expires OR the brake and gas motors are both close enough to the park position, OR runmode has changed THEN stop trying to park the motors
        bool brake_parked = brkpos_sensor.parked();
        bool gas_parked = ((gas_pulse_out_us == gas_pulse_ccw_closed_us + gas_pulse_park_slack_us) && gasServoTimer.expired());
        if ((brake_parked && gas_parked) || motorParkTimer.expired() || (runmode != SHUTDOWN && runmode != BASIC))
            park_the_motors = false;
    }

    // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
    //
    // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
    // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
    // retreive with an OBD tool. Some checks are below, along with other possible things to check for:

    // Check: if engine is turning when ignition signal is off
    if (!ignition && !tachometer.engine_stopped()) {
        if (diag_ign_error_enabled) { // See if the engine is turning despite the ignition being off
            Serial.println (F("Detected engine rotation in the absense of ignition signal"));  // , tach_filt_rpm, ignition
            diag_ign_error_enabled = false;  // Prevents endless error reporting the same error
        }
    }
    else diag_ign_error_enabled = true;

    // Update warning idiot lights
    //
    bool check_wheels;
    check_wheels = false;
    TemperatureSensor * temp_fl = temperature_sensor_manager.get_sensor(sensor_location::WHEEL_FL);
    TemperatureSensor * temp_fr = temperature_sensor_manager.get_sensor(sensor_location::WHEEL_FR);
    TemperatureSensor * temp_rl = temperature_sensor_manager.get_sensor(sensor_location::WHEEL_RL);
    TemperatureSensor * temp_rr = temperature_sensor_manager.get_sensor(sensor_location::WHEEL_RR);
    if (temp_fl != nullptr && temp_fl->get_temperature() >= temp_lims_f[WHEEL][WARNING]) check_wheels = true;
    if (temp_fr != nullptr && temp_fr->get_temperature() >= temp_lims_f[WHEEL][WARNING]) check_wheels = true;
    if (temp_rl != nullptr && temp_rl->get_temperature() >= temp_lims_f[WHEEL][WARNING]) check_wheels = true;
    if (temp_rr != nullptr && temp_rr->get_temperature() >= temp_lims_f[WHEEL][WARNING]) check_wheels = true;
    err_temp_wheel = check_wheels;

    TemperatureSensor * temp_eng = temperature_sensor_manager.get_sensor(sensor_location::ENGINE);
    err_temp_engine = temp_eng != nullptr ? temp_eng->get_temperature() >= temp_lims_f[ENGINE][WARNING] : -999;
    
    
    //  Check: if any sensor is out of range    
    //  Check: if sensor readings aren't consistent with actuator outputs given. Like, if the brake motor is moving down, then either brake position should be decreasing or pressure increasing (for example)
    //  Check if the pressure response is characteristic of air being in the brake line.
    //  * Axle/brake drum may be going bad (increased engine RPM needed to achieve certain speedo)  (beware going up hill may look the same).
    //  * E-brake has been left on (much the same symptoms as above? (beware going up hill may look the same) 
    //  * Battery isn't charging, or just running low.
    //  * Carburetor not behaving (or air filter is clogged). (See above about engine deiseling - we can detect this!)
    //  * After increasing braking, the actuator position changes in the opposite direction, or vise versa.
    //  * Changing an actuator is not having the expected effect.
    //  * A tunable value suspected to be out of tune.
    //  * Check regularly for ridiculous shit. Compare our variable values against a predetermined list of conditions which shouldn't be possible or are at least very suspect. For example:
    //     A) Sensor reading is out of range, or has changed faster than it ever should.
    //     B) Stopping the car on entering hold/shutdown mode is taking longer than it ever should.
    //     C) Mule seems to be accelerating like a Tesla.
    //     D) Car is accelerating yet engine is at idle.
    //  * The control system has nonsensical values in its variables.
    //


    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pid");  //
        
    ts.handleTouch(); // Handle touch events and actions
    // ts.printTouchInfo(); 

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tch");  //

    // Encoder handling
    uint32_t encoder_sw_action = encoder.handleSwitchAction();
    if (encoder_sw_action != Encoder::NONE) {  // First deal with any unhandled switch press events
        if (encoder_sw_action == Encoder::SHORT)  {  // if short press
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
            else if (ctrl == JOY && !simulator.simulating(SimOption::cruisesw)) flycruise_toggle_request = true;  // Unless tuning, when using old joystick allow use of short encoder press to toggle fly/cruise modes
            // I envision pushing encoder switch while not tuning could switch desktops from our current analysis interface to a different runtime display 
        }
        else tuning_ctrl = (tuning_ctrl == OFF) ? SELECT : OFF;  // Long press starts/stops tuning
    }
    if (tuning_ctrl == EDIT) sim_edit_delta_encoder = encoder.handleTuning();
    else if (tuning_ctrl == SELECT) selected_value += encoder.handleSelection();  // If overflow constrain will fix in general handler below
    else if (tuning_ctrl == OFF) dataset_page += encoder.handleSelection();  // If overflow tconstrain will fix in general below

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "enc");  //

    // Tuning : implement effects of changes made by encoder or touchscreen to simulator, dataset_page, selected_value, or tuning_ctrl
    //
    sim_edit_delta += sim_edit_delta_encoder + sim_edit_delta_touch;  // Allow edits using the encoder or touchscreen
    sim_edit_delta_touch = 0;
    sim_edit_delta_encoder = 0;
    if (tuning_ctrl != tuning_ctrl_last || dataset_page != dataset_page_last || selected_value != selected_value_last || sim_edit_delta) tuningCtrlTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
    else if (tuning_ctrl != OFF && tuningCtrlTimer.expired()) tuning_ctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
    dataset_page = constrain (dataset_page, 0, arraysize(pagecard)-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
    if (dataset_page != dataset_page_last) {
        if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If page is flipped during edit, drop back to select mode
        disp_dataset_page_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
    }
    if (tuning_ctrl == SELECT) {
        selected_value = constrain (selected_value, tuning_first_editable_line[dataset_page], disp_tuning_lines-1);  // Skip unchangeable values for all PID modes
        if (selected_value != selected_value_last) disp_selected_val_dirty = true;
    }
    if (tuning_ctrl != tuning_ctrl_last || disp_dataset_page_dirty) disp_selected_val_dirty = true;
    bool adj;
    adj = false;
    if (tuning_ctrl == EDIT && sim_edit_delta != 0) {  // Change tunable values when editing
        if (dataset_page == PG_RUN) {
            if (selected_value == 9) adj = adj_val (&gas_governor_percent, sim_edit_delta, 0, 100);
            else if (selected_value == 10) adj = adj_val (&steer_safe_percent, sim_edit_delta, 0, 100);
        }
        else if (dataset_page == PG_JOY) {
            if (selected_value == 4) adj_val (&hotrc_pulse_failsafe_max_us, sim_edit_delta, hotrc_pulse_failsafe_min_us + 1, hotrc_pulse_lims_us[VERT][MIN] - 1);
            else if (selected_value == 5) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][MIN], sim_edit_delta, 0, ctrl_lims_adc[ctrl][HORZ][CENT] - ctrl_lims_adc[ctrl][HORZ][DB] / 2 - 1);
            else if (selected_value == 6) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][MAX], sim_edit_delta, ctrl_lims_adc[ctrl][HORZ][CENT] + ctrl_lims_adc[ctrl][HORZ][DB] / 2 + 1, ctrl_lims_adc[ctrl][HORZ][CENT]);
            else if (selected_value == 7) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][DB], sim_edit_delta, 0, (ctrl_lims_adc[ctrl][HORZ][CENT] - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - ctrl_lims_adc[ctrl][HORZ][CENT]) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - ctrl_lims_adc[ctrl][HORZ][CENT]) : 2*(ctrl_lims_adc[ctrl][HORZ][CENT] - ctrl_lims_adc[ctrl][HORZ][MIN]));
            else if (selected_value == 8) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][MIN], sim_edit_delta, 0, ctrl_lims_adc[ctrl][VERT][CENT] - ctrl_lims_adc[ctrl][VERT][DB] / 2 - 1);
            else if (selected_value == 9) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][MAX], sim_edit_delta, ctrl_lims_adc[ctrl][VERT][CENT] + ctrl_lims_adc[ctrl][VERT][DB] / 2 + 1, ctrl_lims_adc[ctrl][VERT][CENT]);
            else if (selected_value == 10) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][DB], sim_edit_delta, 0, (ctrl_lims_adc[ctrl][VERT][CENT] - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - ctrl_lims_adc[ctrl][VERT][CENT]) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - ctrl_lims_adc[ctrl][VERT][CENT]) : 2*(ctrl_lims_adc[ctrl][VERT][CENT] - ctrl_lims_adc[ctrl][VERT][MIN]));
            if (adj) calc_ctrl_lims();  // update derived variables relevant to changes made
        }
        else if (dataset_page == PG_CAR) {
            if (selected_value == 2) adj = adj_val (&tach_idle_hot_min_rpm, 0.1*(float)sim_edit_delta, tach_idle_abs_min_rpm, tach_idle_cold_max_rpm - 1);
            else if (selected_value == 3) adj = adj_val (&tach_idle_cold_max_rpm, 0.1*(float)sim_edit_delta, tach_idle_hot_min_rpm + 1, tach_idle_abs_max_rpm);
            else if (selected_value == 4) adj = adj_val (tachometer.get_redline_rpm_ptr().get(), 0.1*(float)sim_edit_delta, throttle.get_idlehigh(), tachometer.get_max_rpm());
            else if (selected_value == 5) adj_val (airflow_sensor.get_max_mph_ptr().get(), 0.01*(float)sim_edit_delta, 0, airflow_sensor.get_abs_max_mph());
            else if (selected_value == 6) adj_val (map_sensor.get_min_psi_ptr().get(), 0.1*(float)sim_edit_delta, map_sensor.get_abs_min_psi(), map_sensor.get_abs_max_psi());
            else if (selected_value == 6) adj_val (map_sensor.get_max_psi_ptr().get(), 0.1*(float)sim_edit_delta, map_sensor.get_abs_min_psi(), map_sensor.get_abs_max_psi());
            else if (selected_value == 8) adj_val (&speedo_idle_mph, 0.01*(float)sim_edit_delta, 0, speedometer.get_redline_mph() - 1);
            else if (selected_value == 9) adj_val (speedometer.get_redline_mph_ptr().get(), 0.01*(float)sim_edit_delta, speedo_idle_mph, 30);
            else if (selected_value == 10) adj_val (brkpos_sensor.get_zeropoint_ptr().get(), 0.001*(float)sim_edit_delta, BrakePositionSensor::nom_lim_retract_in, BrakePositionSensor::nom_lim_extend_in);
        }
        else if (dataset_page == PG_PWMS) {
            if (selected_value == 3) adj_val (&steer_pulse_left_us, sim_edit_delta, steer_pulse_left_min_us, steer_pulse_stop_us - 1);
            else if (selected_value == 4) adj_val (&steer_pulse_stop_us, sim_edit_delta, steer_pulse_left_us + 1, steer_pulse_right_us - 1);
            else if (selected_value == 5) adj_val (&steer_pulse_right_us, sim_edit_delta, steer_pulse_stop_us + 1, steer_pulse_right_max_us);
            else if (selected_value == 6) adj_val (&brake_pulse_extend_us, sim_edit_delta, brake_pulse_extend_min_us + 1, brake_pulse_stop_us);
            else if (selected_value == 7) adj_val (&brake_pulse_stop_us, sim_edit_delta, brake_pulse_extend_us + 1, brake_pulse_retract_us - 1);
            else if (selected_value == 8) adj_val (&brake_pulse_retract_us, sim_edit_delta, brake_pulse_stop_us, brake_pulse_retract_max_us -1);
            else if (selected_value == 9) adj_val (&gas_pulse_ccw_closed_us, sim_edit_delta, gas_pulse_cw_open_us + 1, gas_pulse_ccw_max_us - gas_pulse_park_slack_us);
            else if (selected_value == 10) adj_val (&gas_pulse_cw_open_us, sim_edit_delta, gas_pulse_cw_min_us, gas_pulse_ccw_closed_us - 1);
        }
        else if (dataset_page == PG_IDLE) {
            if (selected_value == 4) throttle.set_idlehigh (throttle.get_idlehigh(), (float)sim_edit_delta);
            else if (selected_value == 5) throttle.set_idlecold (throttle.get_idlecold(), (float)sim_edit_delta);
            else if (selected_value == 6) throttle.set_idlehot (throttle.get_idlehot(), (float)sim_edit_delta);
            else if (selected_value == 7) throttle.set_tempcold (throttle.get_tempcold(), (float)sim_edit_delta);
            else if (selected_value == 8) throttle.set_temphot (throttle.get_temphot(), (float)sim_edit_delta);
            else if (selected_value == 9) throttle.set_settlerate (throttle.get_settlerate() + sim_edit_delta);
            else if (selected_value == 10) throttle.cycle_idlemode (sim_edit_delta);
        }
        else if (dataset_page == PG_BPID) {
            if (selected_value == 8) brakeQPID.SetKp (brakeQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 9) brakeQPID.SetKi (brakeQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 10) brakeQPID.SetKd (brakeQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_GPID) {
            if (selected_value == 7) {
                adj_bool (&gas_open_loop, sim_edit_delta);
                gasQPID.SetMode (gas_open_loop ? QPID::Control::manual : QPID::Control::timer);
            }
            else if (selected_value == 8) gasQPID.SetKp (gasQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 9) gasQPID.SetKi (gasQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 10) gasQPID.SetKd (gasQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_CPID) {
            if (selected_value == 7) adj_bool (&cruise_fixed_throttle, sim_edit_delta);
            else if (selected_value == 8) cruiseQPID.SetKp (cruiseQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 9) cruiseQPID.SetKi (cruiseQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 10) cruiseQPID.SetKd (cruiseQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_TEMP) {
            if (selected_value == 8) simulator.set_can_simulate(SimOption::ignition, adj_bool(simulator.can_simulate(SimOption::ignition), sim_edit_delta));
            else if (selected_value == 9) simulator.set_can_simulate(SimOption::basicsw, adj_bool(simulator.can_simulate(SimOption::basicsw), sim_edit_delta));
            else if (selected_value == 10) simulator.set_can_simulate(SimOption::starter, adj_bool(simulator.can_simulate(SimOption::starter), sim_edit_delta));
         }
        else if (dataset_page == PG_SIM) {
            if (selected_value == 0) simulator.set_can_simulate(SimOption::syspower, adj_bool(simulator.can_simulate(SimOption::syspower), sim_edit_delta));
            else if (selected_value == 1) simulator.set_can_simulate(SimOption::joy, adj_bool(simulator.can_simulate(SimOption::joy), sim_edit_delta));
            else if (selected_value == 2) simulator.set_can_simulate(SimOption::pressure, adj_bool(simulator.can_simulate(SimOption::pressure), sim_edit_delta));
            else if (selected_value == 3) simulator.set_can_simulate(SimOption::brkpos, adj_bool(simulator.can_simulate(SimOption::brkpos), sim_edit_delta));
            else if (selected_value == 4) simulator.set_can_simulate(SimOption::tach, adj_bool(simulator.can_simulate(SimOption::tach), sim_edit_delta));
            else if (selected_value == 5) simulator.set_can_simulate(SimOption::airflow, adj_bool(simulator.can_simulate(SimOption::airflow), sim_edit_delta));
            else if (selected_value == 6) simulator.set_can_simulate(SimOption::mapsens, adj_bool(simulator.can_simulate(SimOption::mapsens), sim_edit_delta));
            else if (selected_value == 7) simulator.set_can_simulate(SimOption::speedo, adj_bool(simulator.can_simulate(SimOption::speedo), sim_edit_delta));
            else if (selected_value == 8) simulator.set_pot_overload(static_cast<SimOption>(adj_val(static_cast<int32_t>(simulator.get_pot_overload()), sim_edit_delta, 0, arraysize(sensorcard) - 1)));
            else if (selected_value == 9 && runmode == CAL) adj_bool (&cal_joyvert_brkmotor, sim_edit_delta);
            else if (selected_value == 10 && runmode == CAL) adj_bool (&cal_pot_gasservo, (sim_edit_delta < 0 || cal_pot_gas_ready) ? sim_edit_delta : -1);
        }
        sim_edit_delta = 0;
    }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tun");  //
    // Ignition & Panic stop logic and Update output signals
    if (!speedometer.car_stopped()) { // if we lose connection to the hotrc while driving, or the joystick ignition button was turned off, panic
        if (ctrl == HOTRC && !simulator.simulating(SimOption::joy) && !hotrc_radio_detected) panic_stop = true;  // panic_stop could also have been initiated by the user button   && hotrc_radio_detected_last 
        else if (ctrl == JOY && !ignition_sense) panic_stop = true;
        // else if (ctrl == JOY && !(simulator.get_enabled() && sim_joy) && !ignition && ignition_last) panic_stop = true;
    }
    else if (panic_stop) panic_stop = false;  // Cancel panic if car is stopped
    if (ctrl == HOTRC) {  // if ignition was turned off on HotRC, panic
        // When using joystick, ignition is controlled with button and we read it. With Hotrc, we control ignition
        hotrc_radio_detected_last = hotrc_radio_detected;
        if (!ignition && ignition_last && !speedometer.car_stopped()) panic_stop = true;
    }

    if (panic_stop) ignition = LOW;  // Kill car if panicking
    if ((ignition != ignition_last) /* && ignition_output_enabled */ ) {  // Whenever ignition state changes, assuming we're allowed to write to the pin
        write_pin (ign_out_pin, ignition);  // Turn car off or on (ign output is active high), ensuring to never turn on the ignition while panicking
        ignition_last = ignition;  // Make sure this goes after the last comparison
    }
    if (runmode == STALL && remote_start_toggle_request) {
        if (remote_starting) {
            set_pin (starter_pin, OUTPUT);
            write_pin (starter_pin, HIGH);
        }
        else {
            write_pin (starter_pin, LOW);
            set_pin (starter_pin, INPUT_PULLDOWN);
        }
        starter = remote_starting;
        remote_start_toggle_request = false;
    }
    if (syspower != syspower_last) {
        syspower = syspower_set (syspower);
        syspower_last = syspower;
    }
    // if (boot_button_action == LONG) {
    //     screen.tft_reset();
    //     boot_button_action = NONE;
    // }
    // if (!screen.get_reset_finished()) screen.tft_reset();  // If resetting tft, keep calling tft_reset until complete
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "ext");  //

    if (neopixel_pin >= 0) {  // Heartbeat led algorithm
        if (neo_heartbeat) {
            neo_heartcolor[N_RED] = (((runmode == SHUTDOWN) ? shutdown_color : colorcard[runmode]) & 0xf800) >> 8;
            neo_heartcolor[N_GRN] = (((runmode == SHUTDOWN) ? shutdown_color : colorcard[runmode]) & 0x7e0) >> 3;
            neo_heartcolor[N_BLU] = (((runmode == SHUTDOWN) ? shutdown_color : colorcard[runmode]) & 0x1f) << 3;
            int32_t neocolor = neostrip.Color (neo_heartcolor[N_BLU], neo_heartcolor[N_RED], neo_heartcolor[N_GRN]);
            if (heartbeatTimer.expired()) {
                heartbeat_pulse = !heartbeat_pulse;
                if (heartbeat_pulse) neo_brightness = neo_brightness_max;
                else neoTimer.reset();
                if (++heartbeat_state >= arraysize (heartbeat_ekg_us)) heartbeat_state -= arraysize (heartbeat_ekg_us);
                heartbeatTimer.set (heartbeat_ekg_us[heartbeat_state]);
            }
            else if (!heartbeat_pulse && neo_brightness) {
                neo_brightness = (int8_t)((float)neo_brightness_max * (1 - (float)neoTimer.elapsed() / (float)neo_timeout_us));
                if (neoTimer.expired() || neo_brightness < 1) neo_brightness = 0;
            }
            int32_t neocolor_last, neobright_last;
            if (neocolor != neocolor_last || neo_brightness != neobright_last) {
                neostrip.setPixelColor (0, neocolor);
                neostrip.setBrightness (neo_brightness);
                neostrip.show();
                neocolor_last = neocolor;
                neobright_last = neo_brightness;
            }
        }
        else if (neoTimer.expireset()) {  // Rainbow fade
            neostrip.setPixelColor (0, colorwheel(++neo_wheelcounter));
            neostrip.show();
        }
    }
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "hrt");
    
    // Display updates
    if (display_enabled) screen.update();
    else if (dataset_page_last != dataset_page) config.putUInt ("dpage", dataset_page);
    
    dataset_page_last = dataset_page;
    selected_value_last = selected_value;
    tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
    simulating_last = simulator.get_enabled();

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "dis");

    // Kick watchdogs
    Watchdog.reset();  // Kick the watchdog to keep us alive
    // if (display_enabled) screen.watchdog();
 
    // Do the control loop bookkeeping at the end of each loop
    loop_period_us = (uint32_t)loopTimer.elapsed();  // us since beginning of this loop
    loopTimer.reset();
    loop_freq_hz = 1000000.0 / ((loop_period_us) ? loop_period_us : 1);  // Prevent potential divide by zero
    loopno++;  // I like to count how many loops
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "end");    
    if (timestamp_loop) {
        // looptime_sum_us += loop_period_us;
        // if (loopno) looptime_avg_us = looptime_sum_us / loopno;
        std::cout <<"\rLp#" << loopno << " us:" << loop_period_us; // << " avg:" << looptime_avg_us;  //  " us:" << esp_timer_get_time() << 
        for (int32_t x=1; x<loopindex; x++) std::cout << " " << std::setw(3) << loop_names[x] << x << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1];
        if (loop_period_us > 20000) std::cout << std::endl;
    }
    loop_int_count = 0;
}
