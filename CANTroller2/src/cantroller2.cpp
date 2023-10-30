// Carpet CANTroller II  Source Code  - For ESP32-S3-DevKitC-1-N8
#include <SPI.h>  // SPI serial bus
#include <vector>
#include <iomanip>  // Formatting cout
#include "globals.h"
#include "display.h"
#include "touch.h"
#include "RunModeManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

HotrcManager hotrcManager[2] = { 6, 6 };  // [HORZ/VERT]
Display screen;
RunModeManager runmode(&screen, &encoder);
TouchScreen touch(touch_cs_pin, touch_irq_pin);
ESP32PWM pwm;  // Object for timer pwm resources (servo outputs)

#define RUN_TESTS 0
#if RUN_TESTS
    #include "unittests.h"
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

void get_touchpoint() { 
    touch_pt[0] = touch.touch_pt(0);
    touch_pt[1] = touch.touch_pt(1); 
    touch_pt[2] = touch.getX();
    touch_pt[3] = touch.getY(); 
}
void neo_idiots_update() {
    for (int32_t idiot = 0; idiot < arraysize(idiotlights); idiot++) {
        if (idiot <= neo.neopixelsAvailable())
            if (*(idiotlights[idiot]) != idiotlasts[idiot]) neo.setBoolState(idiot, *idiotlights[idiot]);
        if (idiot == LOST || idiot == RANGE) {
            if (highest_pri_failing_last[idiot] != highest_pri_failing_sensor[idiot]) {
                if (highest_pri_failing_sensor[idiot] == e_none) neo.setflash((int)idiot, 0);
                else neo.setflash((int)idiot, highest_pri_failing_sensor[idiot] + 1, 2, 6, 1, 0);
            }
            highest_pri_failing_last[idiot] = highest_pri_failing_sensor[idiot];
        }
    }
}
void setup() {  // Setup just configures pins (and detects touchscreen type)
    if (RUN_TESTS) run_tests();   
    set_pin(tft_dc_pin, OUTPUT);
    set_pin(gas_pwm_pin, OUTPUT);
    set_pin(basicmodesw_pin, INPUT_PULLUP);
    set_pin(neopixel_pin, OUTPUT);
    set_pin(sdcard_cs_pin, OUTPUT);
    set_pin(tft_cs_pin, OUTPUT);
    set_pin(starter_pin, INPUT_PULLDOWN);
    if (!usb_jtag) set_pin(steer_enc_a_pin, INPUT_PULLUP);
    if (!usb_jtag) set_pin(steer_enc_b_pin, INPUT_PULLUP);
    set_pin(ignition_pin, OUTPUT);
    set_pin(syspower_pin, OUTPUT);  // Then set the put as an output as normal.
    set_pin(brake_pwm_pin, OUTPUT);
    set_pin(steer_pwm_pin, OUTPUT);
    analogReadResolution(adcbits);  // Set Arduino Due to 12-bit resolution(default is same as Mega=10bit)
    write_pin(tft_cs_pin, HIGH);   // Prevent bus contention
    write_pin(sdcard_cs_pin, HIGH);   // Prevent bus contention
    write_pin(tft_dc_pin, LOW);
    write_pin(ignition_pin, LOW);
    write_pin(syspower_pin, syspower);
    calc_governor();
    set_pin(uart_tx_pin, INPUT);  // UART:  1st detect breadboard vs. vehicle PCB using TX pin pullup, then repurpose pin for UART and start UART 
    running_on_devboard =(read_pin(uart_tx_pin));
    set_board_defaults(running_on_devboard);
    set_pin(uart_tx_pin, OUTPUT);  // 
    Serial.begin(115200);  // Open console serial port
    delay(800);  // This is needed to allow the uart to initialize and the screen board enough time after a cold boot
    printf("Console started..\nUsing %s defaults..\n", (running_on_devboard) ? "dev-board" : "vehicle-pcb");
    printf("Init rmt for hotrc..\n");
    hotrc_calc_params();  // Need to establish derived parameter values
    for (int axis=HORZ; axis<=CH4; axis++) hotrc_rmt[axis].init();  // Set up 4 RMT receivers, one per channel
    printf("Pot setup..\n");
    pot.setup();             printf("Encoder setup..\n");
    encoder.setup();
    printf("Brake pressure sensor.. ");
    pressure.setup();
    printf("done\nBrake position sensor.. ");
    brakepos.setup();
    printf("done\nVehicle battery sense.. ");
    mulebatt.setup();
    printf("done\nLiPO cell sense.. ");
    lipobatt.setup();
    printf("done\nTachometer.. ");
    tach.setup();
    printf("done\nSpeedometer.. ");
    speedo.setup();
    printf("done..\nInit i2c and i2c-enabled devices.."); delay(1);  // Attempt to force print to happen before init
    i2c.init();
    airvelo.setup(); // must be done after i2c is started
    mapsens.setup();
    printf("Simulator setup..\n");
    sim.register_device(sensor::pressure, pressure, pressure.source());
    sim.register_device(sensor::brkpos, brakepos, brakepos.source());
    sim.register_device(sensor::airvelo, airvelo, airvelo.source());
    sim.register_device(sensor::mapsens, mapsens, mapsens.source());
    sim.register_device(sensor::tach, tach, tach.source());
    sim.register_device(sensor::speedo, speedo, speedo.source());
    printf("Configure timers for PWM out..\n");
    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	brakemotor.setPeriodHertz(50);
    gas_servo.setPeriodHertz(60);  // critically this pwm is a different frequency than the other two motors
	steermotor.setPeriodHertz(50);
    brakemotor.attach (brake_pwm_pin, brake_extend_us, brake_retract_us);  // Jag input PWM range default is 670us (full reverse) to 2330us (full fwd). Max range configurable is 500-2500us
    steermotor.attach (steer_pwm_pin, steer_left_us, steer_right_us);  // Jag input PWM range default is 670us (full reverse) to 2330us (full fwd). Max range configurable is 500-2500us
    gas_servo.attach (gas_pwm_pin, gas_cw_min_us, gas_ccw_max_us);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    tempsens.setup();  // Onewire bus and temp sensors
    xTaskCreate(update_temperature_sensors, "Update Temperature Sensors", 2048, NULL, 5, NULL);  // Create a new task that runs the update_temperature_sensors function
    throttle.setup(tempsens.get_sensor(location::engine));
    printf("Init screen.. ");
    if (display_enabled) {
        config.begin("FlyByWire", false);
        dataset_page = config.getUInt("dpage", PG_RUN);
        dataset_page_last = config.getUInt("dpage", PG_TEMP);
        screen.init();
        printf("TFT initialized.. ");
        touch.init();
        printf("touchscreen initialized..\n");
    }
    if (touch_reticles) screen.draw_reticles();
    std::cout << "Init neopixel.. ";
    neo_setup();
    int32_t idiots = min((uint32_t)arraysize(idiotlights), neo.neopixelsAvailable());
    for (int32_t idiot = 0; idiot < idiots; idiot++)
        neo.newIdiotLight(idiot, idiotcolors[idiot], *(idiotlights[idiot]));
    std::cout << "set up heartbeat led and " << idiots << " neopixel idiot lights" << std::endl;
    for (int32_t i=0; i<num_err_types; i++) for (int32_t j=0; j<e_num_sensors; j++) err_sensor[i][j] = false; // Initialize sensor error flags to false
    printf("Setup done%s\n", console_enabled ? "" : ". stopping console during runtime");
    if (!console_enabled) Serial.end();  // close serial console to prevent crashes due to error printing
    panicTimer.reset();
    looptime_init();
}

void loop() {
    // Update inputs.  Fresh sensor data, and filtering
    ignition_panic_update();  // handler for ignition pin output and panicstop status.
    bootbutton_update();
    basicsw_update();
    starter_update();  // Runs starter bidirectional handler
    encoder.update();  // Read encoder input signals
    pot.update();
    brakepos.update();  // Brake position
    tach.update();  // Tach
    throttle.push_tach_reading(tach.human(), tach.last_read_time());    
    speedo.update();  // Speedo
    pressure.update();  // Brake pressure
    mulebatt.update();
    lipobatt.update();
    airvelo.update();
    mapsens.update();  // MAP sensor  // takes 6800 us (!!)
    maf_ugps = massairflow();  // Recalculate intake mass airflow
    if (touch_reticles) get_touchpoint();

    // Controller handling
    // 1. Handle any toggle button events (ch3 and ch4)
    hotrc_events_update(runmode.mode());    
    // 2. Read horz and vert pulse inputs, spike filter, convert to percent, ema filter, constrain, and center if within deadband
    for (int8_t axis = HORZ; axis <= VERT; axis++) {
        hotrc_us[axis][RAW] = (int32_t)hotrc_rmt[axis].readPulseWidth();
        hotrc_us[axis][RAW] = hotrcManager[axis].spike_filter(hotrc_us[axis][RAW]);  // Not exactly "raw" any more after spike filter (not to mention really several readings in the past), but that's what we need
        ema_filt(hotrc_us[axis][RAW], &hotrc_ema_us[axis], hotrc_ema_alpha);  // Need unconstrained ema-filtered vertical for radio lost detection 
        if (!sim.simulating(sensor::joy)) {  // Handle HotRC button generated events and detect potential loss of radio signal
            if (hotrc_us[axis][RAW] >= hotrc_us[axis][CENT])  // hotrc_pc[axis][RAW] = hotrc_us_to_pc(axis, hotrc_us[axis][RAW]);
                hotrc_pc[axis][RAW] = map((float)hotrc_us[axis][RAW], (float)hotrc_us[axis][CENT], (float)hotrc_us[axis][MAX], hotrc_pc[axis][CENT], hotrc_pc[axis][MAX]);
            else hotrc_pc[axis][RAW] = map((float)hotrc_us[axis][RAW], (float)hotrc_us[axis][CENT], (float)hotrc_us[axis][MIN], hotrc_pc[axis][CENT], hotrc_pc[axis][MIN]);
            ema_filt(hotrc_pc[axis][RAW], &(hotrc_pc[axis][FILT]), hotrc_ema_alpha);  // do ema filter to determine joy_vert_filt
            hotrc_pc[axis][FILT] = constrain(hotrc_pc[axis][FILT], hotrc_pc[axis][MIN], hotrc_pc[axis][MAX]);
            if (hotrc_radio_lost || (hotrc_ema_us[axis] > hotrc_us[axis][DBBOT] && hotrc_ema_us[axis] < hotrc_us[axis][DBTOP]))
                hotrc_pc[axis][FILT] = hotrc_pc[axis][CENT];  // if within the deadband set joy_axis_filt to center value
        }
    }
    // 3. Pot map can rudely overwrite the horz value
    if (sim.potmapping(sensor::joy)) hotrc_pc[HORZ][FILT] = pot.mapToRange(steer_left_us, steer_right_us);
    // 4. Determine if the radio is lost
    if (hotrc_ema_us[VERT] > hotrc_failsafe_us + hotrc_failsafe_margin_us) {
        hotrcFailsafeTimer.reset();
        hotrc_radio_lost = false;
    }
    else if (!hotrc_radio_lost && hotrcFailsafeTimer.expired()) hotrc_radio_lost = true;

    runmode.run_runmode();  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    
    // Steering - Determine motor output and send to the motor
    if (steerPidTimer.expireset()) {
        if (runmode.mode() == SHUTDOWN && !shutdown_incomplete)
            steer_out_pc = steer_stop_pc;  // Stop the steering motor if in shutdown mode and shutdown is complete
        else {
            int8_t joydir = get_joydir(HORZ);
            if (joydir == joy_rt) steer_out_pc = map(hotrc_pc[HORZ][FILT], hotrc_pc[HORZ][DBTOP], hotrc_pc[HORZ][MAX], steer_stop_pc, steer_safe (steer_right_pc));  // if joy to the right of deadband
            else if (joydir == joy_lt) steer_out_pc = map(hotrc_pc[HORZ][FILT], hotrc_pc[HORZ][DBBOT], hotrc_pc[HORZ][MIN], steer_stop_pc, steer_safe (steer_left_pc));  // if joy to the left of deadband
            else steer_out_pc = steer_stop_pc;  // Stop the steering motor if inside the deadband
        }
        steer_out_pc = constrain(steer_out_pc, steer_left_pc, steer_right_pc);  // Don't be out of range
        if (steer_out_pc >= steer_stop_pc)
            steer_out_us = map(steer_out_pc, steer_stop_pc, steer_right_pc, steer_stop_us, steer_right_us);
        else steer_out_us = map(steer_out_pc, steer_stop_pc, steer_left_pc, steer_stop_us, steer_left_us);
        steermotor.writeMicroseconds ((int32_t)steer_out_us);   // Write steering value to jaguar servo interface
    }
    // Brakes - Determine motor output and write it to motor
    if (brakePidTimer.expireset()) {
        // Step 1 : Determine motor percent value
        if (runmode.mode() == SHUTDOWN && !shutdown_incomplete)
            brake_out_pc = brake_stop_pc; // if we're shutdown, stop the motor
        else if (runmode.mode() == CAL && cal_joyvert_brkmotor_mode) {
            if (hotrc_pc[VERT][FILT] > hotrc_pc[VERT][DBTOP])
                brake_out_pc = map(hotrc_pc[VERT][FILT], hotrc_pc[VERT][DBTOP], hotrc_pc[VERT][MAX], brake_stop_pc, brake_retract_pc);
            else if (hotrc_pc[VERT][FILT] < hotrc_pc[VERT][DBBOT])
                brake_out_pc = map(hotrc_pc[VERT][FILT], hotrc_pc[VERT][MIN], hotrc_pc[VERT][DBBOT], brake_extend_pc, brake_stop_pc);
            else brake_out_pc = (float)brake_stop_pc;
        }
        else if (park_the_motors) {
            if (brakepos.filt() + brakepos.margin() <= brakepos.parkpos())  // If brake is retracted from park point, extend toward park point, slowing as we approach
                brake_out_pc = map(brakepos.filt(), brakepos.parkpos(), brakepos.min_in(), brake_stop_pc, brake_extend_pc);
            else if (brakepos.filt() - brakepos.margin() >= brakepos.parkpos())  // If brake is extended from park point, retract toward park point, slowing as we approach
                brake_out_pc = map(brakepos.filt(), brakepos.parkpos(), brakepos.max_in(), brake_stop_pc, brake_retract_pc);
        }
        else if (runmode.mode() == CAL || runmode.mode() == BASIC || runmode.mode() == SHUTDOWN)
            brake_out_pc = (float)brake_stop_pc;
        else {  // First attenuate max power to avoid blowing out the motor like in bm2023, if retracting, as a proportion of position from zeropoint to fully retracted
            // To-Do Finish this brake governing calculation
            // brake_retract_effective_us = map(brakepos.filt(), brakepos.zeropoint(), BrakePositionSensor::abs_min_retract_in, )) {    
            // brake_motor_govern_duty_ratio = 0.25;  // 25% = Max motor duty cycle under load given by datasheet. Results in: 1500 + 0.25 * (2330 - 1500) = 1707.5 us max pulsewidth at position = minimum
            brake_retract_effective_max_us = brake_stop_us + brake_motor_govern_duty_pc * (brake_retract_us - brake_stop_us);  // Stores instantaneous calculated value of the effective maximum pulsewidth after attenuation
            brake_pid.compute();  // Otherwise the pid control is active
        }
        // Step 3 : Fix motor pc value if it's out of range or exceeding positional limits
        if (runmode.mode() == CAL && cal_joyvert_brkmotor_mode)  // Constrain the motor to the operational range, unless calibrating (then constraint already performed above)
            brake_out_pc = constrain(brake_out_pc, brake_extend_min_pc, brake_retract_max_pc);  // Constrain to full potential range when calibrating. Caution don't break anything!
        else if ((brake_out_pc < brake_stop_pc && brakepos.filt() > brakepos.parkpos() - brakepos.margin()) || (brake_out_pc > brake_stop_pc && brakepos.filt() < brakepos.min_in() + brakepos.margin()))  // If brake is at position limits and we're tring to go further, stop the motor
            brake_out_pc = brake_stop_pc;
        else brake_out_pc = constrain(brake_out_pc, brake_extend_pc, brake_retract_pc);  // Send to the actuator. Refuse to exceed range
        // Step 4 : Convert motor percent value to pulse width
        if (brake_out_pc >= brake_stop_pc)
            brake_out_us = map(brake_out_pc, brake_stop_pc, brake_retract_pc, brake_stop_us, brake_retract_us);
        else brake_out_us = map(brake_out_pc, brake_stop_pc, brake_extend_pc, brake_stop_us, brake_extend_us);
        // Step 5 : Write to motor
        if (!(runmode.mode() == BASIC && !park_the_motors) && !(runmode.mode() == CAL && !cal_joyvert_brkmotor_mode) && !(runmode.mode() == SHUTDOWN && !shutdown_incomplete)) {
            brakemotor.writeMicroseconds((int32_t)brake_out_us);  // Write result to jaguar servo interface
        }
    }
    throttle.update();  // Allow idle control to mess with tach_target if necessary, or otherwise step in to prevent car from stalling
    // Cruise - Update gas target. Controls gas rpm target to keep speed equal to cruise mph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
    if (runmode.mode() == CRUISE && (cruise_setpoint_mode == pid_suspend_fly) && cruisePidTimer.expireset()) {
        if (cruise_adjusting) speedo_target_mph = speedo.filt();
        else {
            cruise_pid.set_outlimits(throttle.idlespeed(), tach_govern_rpm);  // because cruise pid has internal variable for idlespeed which may need updating
            tach_target_rpm = throttle.target();  // tach_target_rpm is pointed to as the output of the cruise pid loop, need to update it before doing pid math
            cruise_pid.compute();  // cruise pid calculates new output (tach_target_rpm) based on input (speedmeter::human) and target (speedo_target_mph)
            throttle.set_target(tach_target_rpm);  // conversely to above, update throttle manager with new value from the pid calculation
        }
    }
    // Gas - Update servo output. Determine gas actuator output from rpm target.  PID loop is effective in Fly or Cruise mode.
    if (gasPidTimer.expireset()) {
        // Step 1 : Determine servo pulse width value
        if (park_the_motors || (runmode.mode() == SHUTDOWN && !shutdown_incomplete))
            gas_out_us = gas_ccw_closed_us + gas_park_slack_us;
        else if (runmode.mode() == STALL) {  // Stall mode runs the gas servo directly proportional to joystick. This is truly open loop
            if (hotrc_pc[VERT][FILT] < hotrc_pc[VERT][DBTOP]) 
                gas_out_us = gas_ccw_closed_us;  // If in deadband or being pushed down, we want idle
            else gas_out_us = map(hotrc_pc[VERT][FILT], hotrc_pc[VERT][DBTOP], hotrc_pc[VERT][MAX], gas_ccw_closed_us, gas_govern_us);  // actuators still respond even w/ engine turned off
        }
        else if (runmode.mode() == CAL && cal_pot_gasservo_mode)
            gas_out_us = map(pot.val(), pot.min(), pot.max(), gas_ccw_max_us, gas_cw_min_us);
        else if (runmode.mode() == CRUISE && (cruise_setpoint_mode != pid_suspend_fly))
            gas_out_us = gas_cruise_us;
        else if (runmode.mode() != BASIC) {
            tach_target_rpm = throttle.target();
            if (gas_open_loop)  // This isn't really open loop, more like simple proportional control, with output set proportional to target
                gas_out_us = map(throttle.target(), throttle.idlespeed(), tach_govern_rpm, gas_ccw_closed_us, gas_govern_us); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
            else gas_pid.compute();  // Do proper pid math to determine gas_out_us from engine rpm error
        }
        // Step 2 : Constrain if out of range
        if (runmode.mode() == BASIC || runmode.mode() == SHUTDOWN)
            gas_out_us = constrain(gas_out_us, gas_govern_us, gas_ccw_closed_us + gas_park_slack_us);
        else if (runmode.mode() == CAL && cal_pot_gasservo_mode)  // Constrain to operating limits. 
            gas_out_us = constrain(gas_out_us, gas_cw_min_us, gas_ccw_max_us);
        else gas_out_us = constrain(gas_out_us, gas_govern_us, gas_ccw_closed_us);
        // Step 3 : Write to servo
        if (!(runmode.mode() == BASIC && !park_the_motors) && !(runmode.mode() == CAL && !cal_pot_gasservo_mode) && !(runmode.mode() == SHUTDOWN && !shutdown_incomplete)) {
            if (reverse_gas_servo) gas_servo.writeMicroseconds((int32_t)(3000 - gas_out_us));
            else gas_servo.writeMicroseconds ((int32_t)gas_out_us);  // Write result to servo
            // if (boot_button) printf(" Gas:%4ld\n", (int32_t)gas_out_us);
        }
    }
    if (park_the_motors) {  //  When parking motors, IF the timeout expires OR the brake and gas motors are both close enough to the park position, OR runmode has changed THEN stop trying to park the motors
        bool brake_parked = brakepos.parked();
        bool gas_parked = ((gas_out_us == gas_ccw_closed_us + gas_park_slack_us) && gasServoTimer.expired());
        if ((brake_parked && gas_parked) || motorParkTimer.expired() || (runmode.mode() != SHUTDOWN && runmode.mode() != BASIC))
            park_the_motors = false;
    }

    // UI input handling : get any new action from the touchscreen or rotary encoder
    selected_value_last = selected_value;
    dataset_page_last = dataset_page;
    tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
    touch.update(); // Handle touch events and actions
    if (screensaver && touch.touched()) screen.saver_touch(touch.touch_pt(0), touch.touch_pt(1));
    uint32_t encoder_sw_action = encoder.press_event();  // true = autoreset the event if there is one
    if (encoder_sw_action != Encoder::NONE) {  // First deal with any unhandled switch press events
        if (encoder_sw_action == Encoder::SHORT)  {  // if short press
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
            // else ... I envision pushing encoder switch while not tuning could switch desktops from our current analysis interface to a different runtime display 
        }
        else tuning_ctrl = (tuning_ctrl == OFF) ? SELECT : OFF;  // Long press starts/stops tuning
    }
    if (tuning_ctrl == EDIT) idelta_encoder = encoder.rotation(true);  // true = include acceleration
    else if (tuning_ctrl == SELECT) selected_value += encoder.rotation();  // If overflow constrain will fix in general handler below
    else if (tuning_ctrl == OFF) dataset_page += encoder.rotation();  // If overflow tconstrain will fix in general below

    // Tuning : implement effects of changes made by encoder or touchscreen to simulator, dataset_page, selected_value, or tuning_ctrl
    idelta += idelta_encoder + idelta_touch;  // Allow edits using the encoder or touchscreen
    idelta_touch = idelta_encoder = 0;
    if (tuning_ctrl != tuning_ctrl_last || dataset_page != dataset_page_last || selected_value != selected_value_last || idelta) tuningCtrlTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
    else if (tuning_ctrl != OFF && tuningCtrlTimer.expired()) tuning_ctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
    dataset_page = constrain(dataset_page, 0, arraysize(pagecard)-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
    if (dataset_page != dataset_page_last) {
        if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If page is flipped during edit, drop back to select mode
        disp_dataset_page_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
    }
    if (tuning_ctrl == SELECT) {
        selected_value = constrain(selected_value, tuning_first_editable_line[dataset_page], disp_tuning_lines-1);  // Skip unchangeable values for all PID modes
        if (selected_value != selected_value_last) disp_selected_val_dirty = true;
    }
    if (tuning_ctrl != tuning_ctrl_last || disp_dataset_page_dirty) disp_selected_val_dirty = true;
    fdelta = (float)idelta;
    if (tuning_ctrl == EDIT && idelta != 0) {  // Change tunable values when editing
        if (dataset_page == PG_RUN) {
            if (selected_value == 9) {
                adj_val(&gas_governor_pc, idelta, 0, 100);
                calc_governor();
            }
            else if (selected_value == 10) adj_val(&steer_safe_pc, idelta, 0, 100);
        }
        else if (dataset_page == PG_JOY) {
            if (selected_value == 9) adj_val(&hotrc_failsafe_us, idelta, hotrc_absmin_us, hotrc_us[VERT][MIN] - hotrc_us[VERT][MARGIN]);
            else if (selected_value == 10) {
                adj_val(&hotrc_deadband_us, idelta, 0, 50);
                hotrc_calc_params();
            }
        }
        else if (dataset_page == PG_SENS) {
            if (selected_value == 2) throttle.add_idlehot(0.1 * fdelta);
            else if (selected_value == 3) throttle.add_idlecold(0.1 * fdelta);
            else if (selected_value == 4) adj_val(tach.redline_rpm_ptr(), 0.1 * fdelta, throttle.idlehigh(), tach.abs_max_rpm());
            else if (selected_value == 5) adj_val(airvelo.max_mph_ptr(), 0.01 * fdelta, 0, airvelo.abs_max_mph());
            else if (selected_value == 6) adj_val(mapsens.min_psi_ptr(), 0.1 * fdelta, mapsens.abs_min_psi(), mapsens.abs_max_psi());
            else if (selected_value == 6) adj_val(mapsens.max_psi_ptr(), 0.1 * fdelta, mapsens.abs_min_psi(), mapsens.abs_max_psi());
            else if (selected_value == 8) adj_val(&speedo_idle_mph, 0.01 * fdelta, 0, speedo.redline_mph() - 1);
            else if (selected_value == 9) adj_val(speedo.redline_mph_ptr(), 0.01 * fdelta, speedo_idle_mph, 20);
            else if (selected_value == 10) adj_val(brakepos.zeropoint_ptr(), 0.001 * fdelta, brakepos.op_min_in(), brakepos.op_max_in());
        }
        else if (dataset_page == PG_PWMS) {
            if (selected_value == 3) adj_val(&steer_left_us, idelta, steer_left_min_us, steer_stop_us - 1);
            else if (selected_value == 4) adj_val(&steer_stop_us, idelta, steer_left_us + 1, steer_right_us - 1);
            else if (selected_value == 5) adj_val(&steer_right_us, idelta, steer_stop_us + 1, steer_right_max_us);
            else if (selected_value == 6) adj_val(&brake_extend_us, idelta, brake_extend_min_us + 1, brake_stop_us);
            else if (selected_value == 7) adj_val(&brake_stop_us, idelta, brake_extend_us + 1, brake_retract_us - 1);
            else if (selected_value == 8) adj_val(&brake_retract_us, idelta, brake_stop_us, brake_retract_max_us -1);
            else if (selected_value == 9) adj_val(&gas_ccw_closed_us, idelta, gas_cw_open_us + 1, gas_ccw_max_us - gas_park_slack_us);
            else if (selected_value == 10) adj_val(&gas_cw_open_us, idelta, gas_cw_min_us, gas_ccw_closed_us - 1);
        }
        else if (dataset_page == PG_IDLE) {
            if (selected_value == 4) throttle.add_idlehigh(fdelta);
            else if (selected_value == 5) throttle.add_idlecold(fdelta);
            else if (selected_value == 6) throttle.add_idlehot(fdelta);
            else if (selected_value == 7) throttle.add_tempcold(fdelta);
            else if (selected_value == 8) throttle.add_temphot(fdelta);
            else if (selected_value == 9) throttle.add_settlerate(idelta);
            else if (selected_value == 10) throttle.cycle_idlemode(idelta);
        }
        else if (dataset_page == PG_BPID) {
            if (selected_value == 8) brake_pid.add_kp(0.001 * fdelta);
            else if (selected_value == 9) brake_pid.add_ki(0.001 * fdelta);
            else if (selected_value == 10) brake_pid.add_kd(0.001 * fdelta);
        }
        else if (dataset_page == PG_GPID) {
            if (selected_value == 7) { adj_bool(&gas_open_loop, idelta); }  // gas_pid.SetMode (gas_open_loop ? QPID::Control::manual : QPID::Control::automatic);
            else if (selected_value == 8) gas_pid.add_kp(0.001 * fdelta);
            else if (selected_value == 9) gas_pid.add_ki(0.001 * fdelta);
            else if (selected_value == 10) gas_pid.add_kd(0.001 * fdelta);
        }
        else if (dataset_page == PG_CPID) {
            if (selected_value == 7) adj_val(&cruise_delta_max_us_per_s, idelta, 1, 1000);
            else if (selected_value == 8) cruise_pid.add_kp(0.001 * fdelta);
            else if (selected_value == 9) cruise_pid.add_ki(0.001 * fdelta);
            else if (selected_value == 10) cruise_pid.add_kd(0.001 * fdelta);
        }
        else if (dataset_page == PG_TEMP) {
            if (selected_value == 10) adj_bool(&dont_take_temperatures, idelta);
        }
        else if (dataset_page == PG_SIM) {
            if (selected_value == 0) sim.set_can_sim(sensor::joy, idelta);
            else if (selected_value == 1) sim.set_can_sim(sensor::pressure, idelta);
            else if (selected_value == 2) sim.set_can_sim(sensor::brkpos, idelta);
            else if (selected_value == 3) sim.set_can_sim(sensor::speedo, idelta);
            else if (selected_value == 4) sim.set_can_sim(sensor::tach, idelta);
            else if (selected_value == 5) sim.set_can_sim(sensor::airvelo, idelta);
            else if (selected_value == 6) sim.set_can_sim(sensor::mapsens, idelta);
            // else if (selected_value == 7) sim.set_can_sim(sensor::starter, idelta);
            else if (selected_value == 7) sim.set_can_sim(sensor::basicsw, idelta);
            else if (selected_value == 8) sim.set_potmap((adj_val(sim.potmap(), idelta, 0, arraysize(sensorcard) - 4)));            
            else if (selected_value == 9 && runmode.mode() == CAL) adj_bool(&cal_joyvert_brkmotor_mode, idelta);
            else if (selected_value == 10 && runmode.mode() == CAL) adj_bool(&cal_pot_gasservo_mode, (idelta < 0 || cal_pot_gasservo_ready) ? idelta : -1);
        }
        else if (dataset_page == PG_UI) {
            if (selected_value == 7) {
                adj_bool(&flashdemo, idelta);
                enable_flashdemo(flashdemo);
            }
            // else if (selected_value == 7) { adj_val(&globalgamma, 0.01*fdelta, 0.1, 2.57); set_idiotcolors(); }
            else if (selected_value == 8) {
                adj_val(&neobright, idelta, 1, 100);
                neo.setbright(neobright);
            }
            else if (selected_value == 9) {
                adj_val(&neodesat, idelta, 0, 10);  // -10, 10);
                neo.setdesaturation(neodesat);
            }
            else if (selected_value == 10) adj_bool(&screensaver, idelta);
        }
        idelta = 0;
    }
    diag_update();  // Look for screwy conditions and update warning idiot lights
    neo_idiots_update();
    neo.set_heartcolor((runmode.mode() == SHUTDOWN) ? shutdown_color : colorcard[runmode.mode()]);
    neo.update(!syspower);
    looptime_mark("-");
    screen.update(runmode.mode());  // Display updates
    if (!display_enabled && dataset_page_last != dataset_page) config.putUInt("dpage", dataset_page);
    looptime_mark("dis");
    looptime_update();
}