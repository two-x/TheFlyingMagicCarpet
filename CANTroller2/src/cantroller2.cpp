// Carpet CANTroller II  Source Code  - For ESP32-S3-DevKitC-1-N8

#include <SPI.h>  // SPI serial bus needed to talk to the LCD and the SD card
#include <Adafruit_SleepyDog.h>  // Watchdog
#include <vector>
#include <iomanip>  // Formatting cout
#include "qpid.h"  // This is quickpid library except i have to edit some of it
#include "globals.h"
#include "display.h"
#include "uictrl.h"
#include "TouchScreen.h"
using namespace std;

std::vector<string> loop_names(20);

void loop_savetime (uint32_t timesarray[], int32_t &index, vector<string> &names, bool dirty[], string loopname) {  // (int32_t timesarray[], int32_t index) {
    if (dirty[index]) {
        names[index] = loopname;  // names[index], name);
        dirty[index] = false;
    }
    timesarray[index] = esp_timer_get_time();
    index++;
}

HotrcManager hotrcHorzManager (22);
HotrcManager hotrcVertManager (22);    
Display screen;
#ifdef CAP_TOUCH
TouchScreen ts;
#else
TouchScreen ts(touch_cs_pin, touch_irq_pin);
#endif
Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
    
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
    set_pin (tach_pulse_pin, INPUT_PULLUP);
    set_pin (speedo_pulse_pin, INPUT_PULLUP);
    set_pin (brake_pos_pin, INPUT);
    set_pin (neopixel_pin, OUTPUT);
    set_pin (sdcard_cs_pin, OUTPUT);
    set_pin (tft_cs_pin, OUTPUT);
    set_pin (pot_wipe_pin, INPUT);
    set_pin (button_pin, INPUT_PULLUP);
    set_pin (starter_pin, INPUT_PULLDOWN);
    set_pin (joy_horz_pin, INPUT);
    set_pin (joy_vert_pin, INPUT);
    set_pin (touch_irq_pin, INPUT_PULLUP);
    set_pin (ign_batt_pin, INPUT);
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
    
    printf ("Init display..\n");
    if (display_enabled) {
        config.begin("FlyByWire", false);
        dataset_page = config.getUInt("dpage", PG_RUN);
        dataset_page_last = config.getUInt("dpage", PG_TEMP);
        screen.init();
        ts.init();
    }

    printf("Encoder setup..\n");
    encoder.setup();

    printf("Device setup..\n");
    pressure_sensor.setup();

    // Set up our interrupts
    printf ("Attach interrupts..\n");
    attachInterrupt (digitalPinToInterrupt(tach_pulse_pin), tach_isr, RISING);
    attachInterrupt (digitalPinToInterrupt(speedo_pulse_pin), speedo_isr, RISING);

    printf ("Attach servos..\n");
    // Servo() argument 2 is channel (0-15) of the esp timer (?). set to Servo::CHANNEL_NOT_ATTACHED to auto grab a channel
    // gas_servo.setup();
    // gas_servo.set_native_limits();  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    gas_servo.attach (gas_pwm_pin, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    steer_servo.attach (steer_pwm_pin, steer_pulse_right_us, steer_pulse_left_us);  // Jag input PWM range default is 670us (full reverse) to 2330us (full fwd). Max range configurable is 500-2500us
    brake_servo.attach (brake_pwm_pin, brake_pulse_retract_us, brake_pulse_extend_us);  // Jag input PWM range default is 670us (full reverse) to 2330us (full fwd). Max range configurable is 500-2500us
    
    printf ("Init neopixel..");
    neo_heartbeat = (neopixel_pin >= 0);
    neostrip.begin();  // start datastream
    neostrip.show();  // Turn off the pixel
    neostrip.setBrightness (neo_brightness_max);  // It truly is incredibly bright
    
    printf ("Init i2c..");
    i2c_init (i2c_sda_pin, i2c_scl_pin);
    // printf ("done\n");
    for (int32_t i=0; i<i2c_devicecount; i++) if (i2c_addrs[i] == 0x28) airflow_detected = true;
    printf ("Airflow sensor.. %sdetected. sensor is", (airflow_detected) ? "" : "not ");
    if (airflow_detected) {
        if (airflow_sensor.begin() == false) printf (" not");  // Begin communication with air flow sensor) over I2C 
        airflow_sensor.setRange(AIRFLOW_RANGE_15_MPS);
        printf (" responding properly\n");
    }

    printf ("Temp sensors..");
    tempsensebus.setWaitForConversion (false);  // Whether to block during conversion process
    tempsensebus.setCheckForConversion (true);  // Do not listen to device for conversion result, instead we will wait the worst-case period
    tempsensebus.begin();
    temp_detected_device_ct = tempsensebus.getDeviceCount();
    printf (" detected %d devices, parasitic power is %s\n", temp_detected_device_ct, (tempsensebus.isParasitePowerMode()) ? "on" : "off");  // , DEC);
    int32_t temp_unknown_index = 0;
    for (int32_t index = 0; index < temp_detected_device_ct; index++) {  // for (int32_t x = 0; x < arraysize(temp_addrs); x++) {
        if (tempsensebus.getAddress (temp_temp_addr, index)) {
            for (int8_t addrbyte = 0; addrbyte < arraysize(temp_temp_addr); addrbyte++) {
                temp_addrs[index][addrbyte] = temp_temp_addr[addrbyte];
            }
            tempsensebus.setResolution (temp_temp_addr, temperature_precision);  // temp_addrs[x]
            printf ("  found sensor #%d, addr 0x%x\n", index, temp_temp_addr);  // temp_addrs[x]
        }
        else printf ("  ghost device #%d, addr unknown\n", index);  // printAddress (temp_addrs[x]);
    }  // Need algorithm to recognize addresses of detected devices in known vehicle locations
    
    // xTaskCreatePinnedToCore ( codeForTask1, "Task_1", 1000, NULL, 1, &Task1, 0);
    // if (ctrl == HOTRC) {  // Look for evidence of a normal (not failsafe) hotrc signal. If it's not yet powered on, we will ignore its spurious poweron ignition event
    //     int32_t temp = hotrc_vert_pulse_us;
    //     hotrc_radio_detected = ((ctrl_lims_adc[HOTRC][VERT][MIN] <= temp && temp < hotrc_pos_failsafe_min_us) || (hotrc_pos_failsafe_max_us < temp && temp <= ctrl_lims_adc[HOTRC][VERT][MAX]));
    //     for (int32_t x = 0; x < 4; x++) {
    //         delay (20);
    //         if (!((ctrl_lims_adc[HOTRC][VERT][MIN] < temp && temp < hotrc_pos_failsafe_min_us) || (hotrc_pos_failsafe_max_us < temp && temp < ctrl_lims_adc[HOTRC][VERT][MAX]))
    //             || (hotrcPulseTimer.elapsed() > (int32_t)(hotrc_pulse_period_us*2.5))) hotrc_radio_detected = false;
    //     }
    //     printf ("HotRC radio signal: %setected\n", (!hotrc_radio_detected) ? "Not d" : "D");
    // }
    
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
    if (!read_pin (button_pin)) {
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
    // printf ("it:%d ac:%ld lst:%d ta:%d sc:%d el:%ld\n", boot_button, boot_button_action, boot_button_last, boot_button_timer_active, boot_button_suppress_click, dispResetButtonTimer.elapsed());
    
    // External digital signals - takes 11 us to read
    if (!(simulating && sim_basicsw)) basicmodesw = !digitalRead (basicmodesw_pin);   // 1-value because electrical signal is active low
    // if (ctrl == JOY && (!simulating || !sim_cruisesw)) cruise_sw = digitalRead (joy_cruise_btn_pin);

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pre");

    if (take_temperatures) temp_soren();
    if (sim_coolant && pot_overload == coolant) temps_f[ENGINE] = map (pot_filt_percent, 0.0, 100.0, temp_sensor_min_f, temp_sensor_max_f);
    
    encoder.update();  // Read encoder input signals

    // Potentiometer - takes 400 us to read & convert (?!)
    int32_t pot_adc_last;
    pot_adc_last = pot_adc;
    pot_adc = analogRead (pot_wipe_pin);
    // pot_percent = convert_units ((float)pot_adc, pot_convert_percent_per_adc, pot_convert_invert, 0.0, pot_convert_offset);  // Potentiometer
    pot_percent = map ((float)pot_adc, (float)pot_min_adc, (float)pot_max_adc, pot_min_percent, pot_max_percent);
    pot_percent = constrain (pot_percent, pot_min_percent, pot_max_percent);
    ema_filt (pot_percent, &pot_filt_percent, pot_ema_alpha);
    // if (pot_adc != pot_adc_last) printf ("pot adc:%ld pot%%:%lf filt:%lf\n", pot_adc, pot_percent, pot_filt_percent); 
    
    // Brake position - takes 70 us to read, convert, and filter
    if (sim_brkpos && pot_overload == brkpos) brake_pos_filt_in = map (pot_filt_percent, 0.0, 100.0, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);
    else if (!(simulating && sim_brkpos)) {
        brake_pos_in = convert_units ((float)analogRead (brake_pos_pin), brake_pos_convert_in_per_adc, brake_pos_convert_invert);
        ema_filt (brake_pos_in, &brake_pos_filt_in, brake_pos_ema_alpha);
    }
    else brake_pos_filt_in = (brake_pos_nom_lim_retract_in + brake_pos_zeropoint_in)/2;  // To keep brake position in legal range during simulation
    
    if (!(simulating && sim_starter)) starter = read_pin (starter_pin);

    // Tach - takes 22 us to read when no activity
    if (sim_tach && pot_overload == tach) tach_filt_rpm = map (pot_filt_percent, 0.0, 100.0, 0.0, tach_govern_rpm);
    else if (!(simulating && sim_tach)) {
        tach_buf_us = (int32_t)tach_us;  // Copy delta value (in case another interrupt happens during handling)
        tach_us = 0;  // Indicates to isr we processed this value
        if (tach_buf_us) {  // If a valid rotation has happened since last time, delta will have a value
            tach_rpm = convert_units ((float)(tach_buf_us), tach_convert_rpm_per_rpus, tach_convert_invert);
            ema_filt (tach_rpm, &tach_filt_rpm, tach_ema_alpha);  // Sensor EMA filter
        }
        if (tach_rpm < tach_stop_thresh_rpm || (uint32_t)(tach_timer_read_us - tach_timer_start_us) >= tach_stop_timeout_us) {  // If time between pulses is long enough an engine can't run that slow
            tach_rpm = 0.0;  // If timeout since last magnet is exceeded
            tach_filt_rpm = 0.0;
        }        
    }
    // Airflow sensor
    if (sim_airflow && pot_overload == airflow) airflow_filt_mph = map (pot_filt_percent, 0.0, 100.0, 0.0, airflow_max_mph);
    else if (airflow_detected && !(simulating && sim_airflow)) {
        airflow_mph = airflow_sensor.readMilesPerHour(); // note, this returns a float from 0-33.55 for the FS3000-1015 
        ema_filt (airflow_mph, &airflow_filt_mph, airflow_ema_alpha);  // Sensor EMA filter
    }
    // Speedo - takes 14 us to read when no activity
    if (sim_speedo && pot_overload == speedo) speedo_filt_mph = map (pot_filt_percent, 0.0, 100.0, 0.0, speedo_govern_mph);
    else if (!(simulating && sim_speedo)) { 
        speedo_buf_us = (int32_t)speedo_us;  // Copy delta value (in case another interrupt happens during handling)
        speedo_us = 0;  // Indicates to isr we processed this value
        if (speedo_buf_us) {  // If a valid rotation has happened since last time, delta will have a value
            speedo_mph = convert_units ((float)(speedo_buf_us), speedo_convert_mph_per_rpus, speedo_convert_invert);  // Update car speed value  
            ema_filt (speedo_mph, &speedo_filt_mph, speedo_ema_alpha);  // Sensor EMA filter
        }
        if (speedo_mph < speedo_stop_thresh_mph || (uint32_t)(speedo_timer_read_us - speedo_timer_start_us) >= speedo_stop_timeout_us) {  // If time between pulses is long enough, consider the car is stopped
            speedo_mph = 0.0;
            speedo_filt_mph = 0.0;
        }
    }

    // Brake pressure - takes 72 us to read
    pressure_sensor.read();
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "inp");  //

    // Read the car ignition signal, and while we're at it measure the vehicle battery voltage off ign signal
    ignition_sense = read_battery_ignition();  // Updates battery voltage reading and returns ignition status
    if (sim_battery && pot_overload == coolant) battery_filt_v = map (pot_filt_percent, 0.0, 100.0, 0.0, battery_max_v);

    // Controller handling
    //

    // Read horz and vert inputs, determine steering pwm output -  - takes 40 us to read. Then, takes 13 us to handle
    if (ctrl != JOY) {
        if (hotrc_source == ESP_RMT) {  // Read RMT pulse widths
            hotrc_horz_pulse_us = (int32_t)hotrc_horz.readPulseWidth();  
            hotrc_vert_pulse_us = (int32_t)hotrc_vert.readPulseWidth();
        }
        else {
            hotrc_horz_pulse_us = hotrcHorzManager.spike_filter ((int32_t)hotrc_horz_pulse_64_us);
            hotrc_vert_pulse_us = hotrcVertManager.spike_filter ((int32_t)hotrc_vert_pulse_64_us);
        }
        ema_filt (hotrc_vert_pulse_us, &hotrc_vert_pulse_filt_us, ctrl_ema_alpha[HOTRC]);  // Used to detect loss of radio
        // if (boot_button) printf ("hrc H:%4ld V:%4ld\n", hotrc_horz_pulse_us, hotrc_vert_pulse_us);
        // ema_filt (hotrc_horz_pulse_us, &hotrc_horz_pulse_filt_us, ctrl_ema_alpha[HOTRC]);  // Just here for debugging. Do not need filtered horz value
        // if (boot_button) printf (" | ema H:%4ld V:%4ld", hotrc_horz_pulse_filt_us, hotrc_vert_pulse_filt_us);
    }
    if (!(simulating && sim_joy)) {  // Handle HotRC button generated events and detect potential loss of radio signal
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
        // if (boot_button) printf (" | Craw0 H:%4ld V:%4ld", ctrl_pos_adc[HORZ][RAW], ctrl_pos_adc[VERT][RAW]);
        // if (boot_button) printf (" | Cflt0 H:%4ld V:%4ld", ctrl_pos_adc[HORZ][FILT], ctrl_pos_adc[VERT][FILT]);
        ema_filt (ctrl_pos_adc[VERT][RAW], &ctrl_pos_adc[VERT][FILT], ctrl_ema_alpha[ctrl]);  // do ema filter to determine joy_vert_filt
        ema_filt (ctrl_pos_adc[HORZ][RAW], &ctrl_pos_adc[HORZ][FILT], ctrl_ema_alpha[ctrl]);  // do ema filter to determine joy_horz_filt
        ctrl_pos_adc[VERT][FILT] = constrain (ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
        ctrl_pos_adc[HORZ][FILT] = constrain (ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
        // if (boot_button) printf (" | Cflt1 H:%4ld V:%4ld", ctrl_pos_adc[HORZ][FILT], ctrl_pos_adc[VERT][FILT]);
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) {
            ctrl_pos_adc[VERT][FILT] = ctrl_lims_adc[ctrl][VERT][CENT];  // if joy vert is in the deadband, set joy_vert_filt to center value
        }
        if (ctrl_pos_adc[HORZ][FILT] > ctrl_db_adc[HORZ][BOT] && ctrl_pos_adc[HORZ][FILT] < ctrl_db_adc[HORZ][TOP]) {
            ctrl_pos_adc[HORZ][FILT] = ctrl_lims_adc[ctrl][HORZ][CENT];  // if joy horz is in the deadband, set joy_horz_filt to center value
        }
        // if (boot_button) printf (" | Cflt2 H:%4ld V:%4ld\n", ctrl_pos_adc[HORZ][FILT], ctrl_pos_adc[VERT][FILT]);
    }    
    // if (boot_button) printf ("hrz: %ld | saf: %ld | out %ld", ctrl_pos_adc[HORZ][FILT], steer_pulse_safe_us, steer_pulse_out_us);

    // Voltage of vehicle battery
    ignition_sense = read_battery_ignition();  // Updates battery voltage reading and returns ignition status
    if (sim_battery && pot_overload == battery) battery_filt_v = map (pot_filt_percent, 0.0, 100.0, 0.0, battery_max_v);

    if (ctrl == JOY) ignition = ignition_sense;
    else if (ctrl == HOTRC) {
        hotrc_ch3_update();
        hotrc_ch4_update();
        if (hotrc_ch3_sw_event) {  // Turn on/off the vehicle ignition. If ign is turned off while the car is moving, this leads to panic stop
            if (hotrc_suppress_next_ch3_event) hotrc_suppress_next_ch3_event = false;
            else ignition = !ignition;
            hotrc_ch3_sw_event = false;
        }
        if (hotrc_ch4_sw_event) {
            if (hotrc_suppress_next_ch4_event) hotrc_suppress_next_ch4_event = false;
            else if (runmode == FLY || runmode == CRUISE) flycruise_toggle_request = true;
            else if (runmode == STALL) remote_start_toggle_request = true;
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
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "joy");  //
    
    // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    //
    
    if (runmode == BASIC) {  // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        else if (!engine_stopped() && !basicmodesw) runmode = HOLD;  // If we turned off the basic mode switch with engine running, go to Hold mode. If engine is not running, we'll end up in Stall Mode automatically
    }
    else if (runmode == SHUTDOWN) { // In shutdown mode we stop the car if it's moving then park the motors.
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            tach_target_rpm = tach_idle_rpm;  //  Release the throttle 
            shutdown_complete = false;
            shutdown_color = LPNK;
            disp_runmode_dirty = true;
            calmode_request = false;
            park_the_motors = false;
            if (!car_stopped()) {
                if (panic_stop && pressure_target_psi < pressure_panic_initial_psi) pressure_target_psi = pressure_panic_initial_psi;
                else if (!panic_stop && pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = pressure_hold_initial_psi;
                brakeIntervalTimer.reset();
                stopcarTimer.reset();
            }
        }
        else if ((car_stopped() || !require_car_stopped_before_driving) && ignition && !panic_stop && !engine_stopped() && !starter)
            runmode = HOLD;  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
        if (!shutdown_complete) {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (car_stopped() || stopcarTimer.expired()) {  // If car has stopped, or timeout expires, then release the brake
                if (shutdown_color == LPNK) {  // On first time through here
                    park_the_motors = true;  // Flags the motor parking to happen, only once
                    gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
                    motorParkTimer.reset();  // Set a timer to timebox this effort
                    shutdown_color = DPNK;
                    disp_runmode_dirty = true;
                }
                else if (!park_the_motors) {  // When done parking the motors we can finish shutting down
                    shutdown_complete = true;
                    shutdown_color = colorcard[SHUTDOWN];
                    disp_runmode_dirty = true;
                    sleepInactivityTimer.reset();
                }
            }
            else if (brakeIntervalTimer.expireset())
                pressure_target_psi = pressure_target_psi + (panic_stop) ? pressure_panic_increment_psi : pressure_hold_increment_psi;  // Slowly add more brakes until car stops
        }
        else if (calmode_request) runmode = CAL;  // if fully shut down and cal mode requested, go to cal mode
        else if (sleepInactivityTimer.expired()) {
            syspower = LOW; // Power down devices to save battery
            // go to sleep, would happen here 
        }
    }
    else if (runmode == STALL) {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (we_just_switched_modes) {
            remote_starting = false;
            remote_start_toggle_request = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][BOT]) pressure_target_psi = pressure_sensor.get_min_human();  // If in deadband or being pushed up, no pressure target
        else pressure_target_psi = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][BOT], (float)ctrl_lims_adc[ctrl][VERT][MIN], pressure_sensor.get_min_human(), pressure_sensor.get_max_human());  // Scale joystick value to pressure adc setpoint
        if (!starter && !engine_stopped()) runmode = HOLD;  // If we started the car, enter hold mode once starter is released
    }
    else if (runmode == HOLD) {
        if (we_just_switched_modes) {  // Release throttle and push brake upon entering hold mode
            tach_target_rpm = tach_idle_rpm;  // Let off gas (if gas using PID mode)
            if (car_stopped()) pressure_target_psi = pressure_sensor.get_filtered_value() + pressure_hold_increment_psi; // If the car is already stopped then just add a touch more pressure and then hold it.
            else if (pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = pressure_hold_initial_psi;  //  These hippies need us to stop the car for them
            brakeIntervalTimer.reset();
            stopcarTimer.reset();
            joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
        }
        if (brakeIntervalTimer.expireset() && !car_stopped() && !stopcarTimer.expired()) {  // Each interval the car is still moving, push harder
            pressure_target_psi += pressure_hold_increment_psi;
        }
        if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && (ctrl == JOY || hotrc_radio_detected)) runmode = FLY; // Enter Fly Mode upon joystick movement from center to above center.
        // Possibly add "&& car_stopped()" to above check?
    }
    else if (runmode == FLY) {
        if (we_just_switched_modes) {
            gesture_progress = 0;
            gestureFlyTimer.set (gesture_flytimeout_us); // Initialize gesture timer to already-expired value
            cruise_sw_held = false;
            // cruiseSwTimer.reset();  // Needed if momentary cruise button is used to go to cruise mode
            flycruise_toggle_request = false;
            car_initially_moved = !car_stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
        }
        if (!car_initially_moved) {
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) runmode = HOLD;  // Must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!car_stopped()) car_initially_moved = true;  // Once car moves, we're allowed to stay in fly mode
        }
        else if (car_stopped()) runmode = HOLD;  // Go to Hold Mode if we have come to a stop after moving  // && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT]
        if (ctrl == HOTRC && !(simulating && sim_joy) && !hotrc_radio_detected) runmode = HOLD;  // Radio must be good to fly. This should already be handled elsewhere but another check can't hurt
        else {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP])  {  // If we are trying to accelerate, scale joystick value to determine gas setpoint
                tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], tach_idle_rpm, tach_govern_rpm);
            }
            else tach_target_rpm = tach_idle_rpm;  // Else let off gas (if gas using PID mode)
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT])  {  // If we are trying to brake, scale joystick value to determine brake pressure setpoint
                pressure_target_psi = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][BOT], (float)ctrl_lims_adc[ctrl][VERT][MIN], pressure_sensor.get_min_human(), pressure_sensor.get_max_human());
            }
            else pressure_target_psi = pressure_sensor.get_min_human();  // Else let off the brake   
        }
        // Cruise mode can be entered by pressing a controller button, or by holding the brake on full for a half second. Which epends on the cruise_gesturing flag.
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (flycruise_toggle_request) runmode = CRUISE;
        if (ctrl == JOY) {
            if (cruise_gesturing) {  // If we are configured to use joystick gestures to go to cruise mode, the gesture is 
                if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP]) {  // Re-zero gesture timer for potential new gesture whenever joystick at center
                    gestureFlyTimer.reset();
                }
                if (gestureFlyTimer.expired()) gesture_progress = 0; // If gesture timeout has expired, cancel any in-progress gesture
                else {  // Otherwise check for successful gesture motions
                    if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_lims_adc[ctrl][VERT][MAX] - flycruise_vert_margin_adc) {  // If joystick quickly pushed to top, step 1 of gesture is successful
                        gesture_progress++;
                        gestureFlyTimer.reset();
                    }
                    else if (gesture_progress == 1 && ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN] + flycruise_vert_margin_adc) {  // If joystick then quickly pushed to bottom, step 2 succeeds
                        gesture_progress++;
                        gestureFlyTimer.reset();
                    }
                    else if (gesture_progress == 2 && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP]) {  // If joystick then quickly returned to center, go to Cruise mode
                        runmode = CRUISE;
                    }        
                }
            }
            // This was when the thought was to add a momentary button to the joystick to toggle cruise <-> fly mode
            // if (!cruise_sw) {  // If button not currently pressed
            //     if (cruise_sw_held && cruiseSwTimer.expired()) runmode = CRUISE;  // After a long press of sufficient length, upon release enter Cruise mode
            //     cruise_sw_held = false;  // Cancel button held state
            // }
            // else if (!cruise_sw_held) {  // If the button just now got pressed
            //     cruiseSwTimer.reset(); // Start hold time timer
            //     cruise_sw_held = true;  // Get into button held state
            // }
        }
    }
    else if (runmode == CRUISE) {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            speedo_target_mph = speedo_filt_mph;
            pressure_target_psi = pressure_sensor.get_min_human();  // Let off the brake and keep it there till out of Cruise mode
            gestureFlyTimer.reset();  // reset gesture timer
            cruise_sw_held = false;
            cruise_adjusting = false;
            flycruise_toggle_request = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) {  // When joystick vert above center, increase the throttle target proportional to how far off center
            if (!cruise_adjusting) tach_adjustpoint_rpm = tach_filt_rpm;  // When beginning adjustment, save current tach value to use as adjustment low endpoint 
            cruise_adjusting = true;  // Suspend pid loop control of gas
            tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], tach_adjustpoint_rpm, tach_govern_rpm);
        }
        else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) {  // When joystick vert below center, decrease the throttle target proportional to how far off center
            if (!cruise_adjusting) tach_adjustpoint_rpm = tach_filt_rpm;  // When beginning adjustment, save current tach value to use as adjustment high endpoint 
            cruise_adjusting = true;  // Suspend pid loop control of gas
            tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_lims_adc[ctrl][VERT][MIN], (float)ctrl_db_adc[VERT][BOT], tach_idle_rpm, tach_adjustpoint_rpm);
        }
        else cruise_adjusting = false;  // When joystick at center, the target speed stays locked to the value it was when joystick goes to center
        
        if (!cruise_adjusting) cruiseAntiglitchTimer.reset();  // Anti-glitch timer attempts to keep very short joystick sensor glitches from going into adjust mode
        else if (cruiseAntiglitchTimer.expired()) speedo_target_mph = speedo_filt_mph;  // May be unneccesary now that our readings are stable.  Remove?  Anyway, need to review the logic

        if (flycruise_toggle_request) runmode = FLY;  // Go to fly mode if hotrc ch4 button pushed

        // If joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (ctrl_pos_adc[VERT][FILT] > ctrl_lims_adc[ctrl][VERT][MIN] + flycruise_vert_margin_adc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) runmode = FLY;  // New gesture to drop to fly mode is hold the brake all the way down for more than X ms
        
        if (car_stopped()) runmode = HOLD;  // In case we slam into a brick wall, get out of cruise mode
    }
    else if (runmode == CAL) {  // Calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration. Don't use it unless you know how.
        if (we_just_switched_modes) {  // Entering Cal mode: From fully shut down state, open simulator and long-press the Cal button. Each feature starts disabled but can be enabled with the tuner.
            calmode_request = false;
            cal_pot_gas_ready = false;
            cal_pot_gasservo = false;
            cal_joyvert_brkmotor = false;
        }
        else if (calmode_request) runmode = SHUTDOWN;
        
        if (!cal_pot_gas_ready) {
            float temp = map (pot_filt_percent, pot_min_percent, pot_max_percent, (float)gas_pulse_ccw_max_us, (float)gas_pulse_cw_min_us);
            if (temp <= (float)gas_pulse_idle_us && temp >= (float)gas_pulse_redline_us) cal_pot_gas_ready = true;
        }
    }
    else { // Obviously this should never happen
        if (serial_debugging) printf ("Error: Invalid runmode entered\n");
        runmode = SHUTDOWN;
    }
    
    if (basicmodesw) runmode = BASIC;  // if basicmode switch on --> Basic Mode
    else if (runmode != CAL && (panic_stop || !ignition)) runmode = SHUTDOWN;
    else if (runmode != CAL && (starter || engine_stopped())) runmode = STALL;  // otherwise if engine not running --> Stall Mode

    we_just_switched_modes = (runmode != oldmode);  // runmode should not be changed after this point in loop
    if (we_just_switched_modes) {
        disp_runmode_dirty = true;
        syspower = HIGH;
    }
    // cout << "rm:" << runmode << " om:" << oldmode << "vert:" << ctrl_pos_adc[VERT][FILT] << " up?" << (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) << " jc?" << joy_centered << "\n";
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "mod");  //

    // Update motor outputs - takes 185 us to handle every 30ms when the pid timer expires, otherwise 5 us
    //

    // Steering - Determine motor output and send to the motor
    if (steerPidTimer.expireset() && !(runmode == SHUTDOWN && shutdown_complete)) {
        if (ctrl_pos_adc[HORZ][FILT] >= ctrl_db_adc[HORZ][TOP])  // If above the top edge of the deadband, turning right
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
        // if (boot_button) printf ("JoyH:%4ld, safadj:%4.0lf out:%4.0lf puls:%4.0lf\n", ctrl_pos_adc[HORZ][FILT], steer_safe_adj_percent, steer_out_percent, steer_pulse_out_us);
    }
    // Brakes - Determine motor output and write it to motor
    if (brakePidTimer.expireset() && !(runmode == SHUTDOWN && shutdown_complete)) {
        if (runmode == CAL && cal_joyvert_brkmotor) {
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) brake_out_percent = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], brake_stop_percent, brake_retract_percent);
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) brake_out_percent = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_lims_adc[ctrl][VERT][MIN], (float)ctrl_db_adc[VERT][BOT], brake_extend_percent, brake_stop_percent);
            else brake_out_percent = (float)brake_stop_percent;
        }
        else if (park_the_motors) {
            if (brake_pos_filt_in + brake_pos_margin_in <= brake_pos_park_in)  // If brake is retracted from park point, extend toward park point, slowing as we approach
                brake_out_percent = map (brake_pos_filt_in, brake_pos_park_in, brake_pos_nom_lim_retract_in, brake_stop_percent, brake_extend_percent);
            else if (brake_pos_filt_in - brake_pos_margin_in >= brake_pos_park_in)  // If brake is extended from park point, retract toward park point, slowing as we approach
                brake_out_percent = map (brake_pos_filt_in, brake_pos_park_in, brake_pos_nom_lim_extend_in, brake_stop_percent, brake_retract_percent);
        }
        else if (runmode != BASIC) brakeQPID.Compute();  // Otherwise the pid control is active
        if (runmode != BASIC || park_the_motors) {
            if (((brake_pos_filt_in + brake_pos_margin_in <= brake_pos_nom_lim_retract_in) && brake_out_percent > brake_stop_percent) ||  // If the motor is at or past its position limit in the retract direction, and we're intending to retract more ...
                ((brake_pos_filt_in - brake_pos_margin_in >= brake_pos_nom_lim_extend_in) && brake_out_percent < brake_stop_percent))  // ... or same thing in the extend direction ...
                    brake_out_percent = brake_stop_percent;  // ... then stop the motor
            else if (runmode == CAL && cal_joyvert_brkmotor)  // Constrain the motor to the operational range, unless calibrating (then constraint already performed above)
                 brake_out_percent = constrain (brake_out_percent, brake_extend_min_percent, brake_retract_max_percent);  // Constrain to full potential range when calibrating. Caution don't break anything!
            else brake_out_percent = constrain (brake_out_percent, brake_extend_percent, brake_retract_percent);  // Send to the actuator. Refuse to exceed range    
            
            if (brake_out_percent >= brake_stop_percent)
                brake_pulse_out_us = map (brake_out_percent, brake_stop_percent, brake_retract_percent, brake_pulse_stop_us, brake_pulse_retract_us);
            else brake_pulse_out_us = map (brake_out_percent, brake_stop_percent, brake_extend_percent, brake_pulse_stop_us, brake_pulse_extend_us);
                    
            #ifdef pwm_jaguars
                brake_servo.writeMicroseconds ((int32_t)brake_pulse_out_us);  // Write result to jaguar servo interface
            #else
                // Send command over serial port
            #endif
            // if (boot_button) printf ("JoyH:%4ld, safadj:%4.0lf out:%4.0lf puls:%4.0lf", ctrl_pos_adc[HORZ][FILT], steer_safe_adj_percent, steer_out_percent, steer_pulse_out_us);
            // if (boot_button) printf (" Brk:%4ld", (int32_t)brake_pulse_out_us);
        }
    }
    // Cruise - Update gas target. Controls gas rpm target to keep speed equal to cruise mph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
    if (runmode == CRUISE && !cruise_adjusting && cruisePidTimer.expireset()) cruiseQPID.Compute();  // Cruise mode is simpler because it doesn't have to deal with an actuator. It's output is simply the target value for the gas PID
    // Gas - Update servo output. Determine gas actuator output from rpm target.  PID loop is effective in Fly or Cruise mode.
    if (gasPidTimer.expireset() && !(runmode == SHUTDOWN && shutdown_complete)) {
        if (park_the_motors) gas_pulse_out_us = gas_pulse_idle_us + gas_pulse_park_slack_us;
        else if (runmode == STALL) {  // Stall mode runs the gas servo directly proportional to joystick. This is truly open loop
            // if (starter) gas_pulse_out_us = gas_pulse_govern_us;  // Fully open throttle during starting engine
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) gas_pulse_out_us = gas_pulse_idle_us;  // If in deadband or being pushed down, we want idle
            else gas_pulse_out_us = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], gas_pulse_idle_us, gas_pulse_govern_us);  // Actuators still respond and everything, even tho engine is turned off
        }
        else if (runmode != BASIC) {
            if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo) {
                gas_pulse_out_us = map (pot_filt_percent, pot_min_percent, pot_max_percent, gas_pulse_ccw_max_us, gas_pulse_cw_min_us);
                gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
            }            
            else if (gasQPID.GetMode() == (uint8_t)QPID::Control::manual)  // This isn't really open loop, more like simple proportional control, with output set proportional to target 
                gas_pulse_out_us = map (tach_target_rpm, tach_idle_rpm, tach_govern_rpm, gas_pulse_idle_us, gas_pulse_govern_us); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
            else gasQPID.Compute();  // Do proper pid math to determine gas_pulse_out_us from engine rpm error
            // printf ("Gas PID   rm= %+-4ld target=%-+9.4lf", runmode, (float)tach_target_rpm);
            // printf (" output = %-+9.4lf,  %+-4ld\n", gasQPID.get_output(), gas_pulse_out_us);
        }
        if (runmode != BASIC || park_the_motors) {
            if (!(runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo))  // Constrain to operating limits. If calibrating constrain already happened above
                gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_govern_us, gas_pulse_idle_us);
            // printf (" output = %-+9.4lf,  %+-4ld\n", gasQPID.get_output(), gas_pulse_out_us);
            gas_servo.writeMicroseconds ((int32_t)gas_pulse_out_us);  // Write result to servo
            // if (boot_button) printf (" Gas:%4ld\n", (int32_t)gas_pulse_out_us);
        }
    }
    if (park_the_motors) {  //  When parking motors, IF the timeout expires OR the brake and gas motors are both close enough to the park position, OR runmode has changed THEN stop trying to park the motors
        bool brake_parked = (abs(brake_pos_filt_in - brake_pos_park_in) <= brake_pos_margin_in);
        bool gas_parked = ((gas_pulse_out_us == gas_pulse_idle_us + gas_pulse_park_slack_us) && gasServoTimer.expired());
        if ((brake_parked && gas_parked) || motorParkTimer.expired() || (runmode != SHUTDOWN && runmode != BASIC))
            park_the_motors = false;
    }

    // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
    //
    // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
    // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
    // retreive with an OBD tool. Eventually this should include functions allowing us to detect things like:
    //  1. A sensor or actuator is unplugged, movement blocked, missing magnetic pulses, etc.
    //  2. Air in the brake lines.
    //  3. Axle/brake drum may be going bad (increased engine RPM needed to achieve certain speedo)  (beware going up hill may look the same).
    //  4. E-brake has been left on (much the same symptoms as above? (beware going up hill may look the same) 
    //  5. Battery isn't charging, or just running low.
    //  6. Carburetor not behaving (or air filter is clogged). (See above about engine deiseling - we can detect this!)
    //  7. After increasing braking, the actuator position changes in the opposite direction, or vise versa.
    //  8. Changing an actuator is not having the expected effect.
    //  9. A tunable value suspected to be out of tune.
    //  10. Check regularly for ridiculous shit. Compare our variable values against a predetermined list of conditions which shouldn't be possible or are at least very suspect. For example:
    //     A) Sensor reading is out of range, or has changed faster than it ever should.
    //     B) Stopping the car on entering hold/shutdown mode is taking longer than it ever should.
    //     C) Mule seems to be accelerating like a Tesla.
    //     D) Car is accelerating yet engine is at idle.
    //  11. The control system has nonsensical values in its variables.
    //
    if (!ignition && !engine_stopped()) {
        if (diag_ign_error_enabled) { // See if the engine is turning despite the ignition being off
            Serial.println (F("Detected engine rotation in the absense of ignition signal"));  // , tach_filt_rpm, ignition
            diag_ign_error_enabled = false;  // Prevents endless error reporting the same error
        }
    }
    else diag_ign_error_enabled = true;
    // I don't think we really need to panic about this, but it does seem curious. Actually this will probably occur like when we're sliding
    // into camp after a ride, and kill the engine before we stop the car. For a fraction of a second the engine would keep turning anyway.
    // Or for that matter whenever the carb is out of tune and making the engine diesel after we kill the ign.

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pid");  //
        
    ts.handleTouch(); // Handle touch events and actions
    // ts.printTouchInfo(); 

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tch");  //

    // Encoder handling
    //
    uint32_t encoder_sw_action = encoder.handleSwitchAction();
    if (encoder_sw_action != Encoder::NONE) {  // First deal with any unhandled switch press events
        if (encoder_sw_action == Encoder::SHORT)  {  // if short press
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
            else if (ctrl == JOY && (!simulating || !sim_cruisesw)) flycruise_toggle_request = true;  // Unless tuning, when using old joystick allow use of short encoder press to toggle fly/cruise modes
            // I envision pushing encoder switch while not tuning could switch desktops from our current analysis interface to a different runtime display 
        }
        else tuning_ctrl = (tuning_ctrl == OFF) ? SELECT : OFF;  // Long press starts/stops tuning
    }
    if (tuning_ctrl == EDIT) sim_edit_delta_encoder = encoder.handleTuning();
    else if (tuning_ctrl == SELECT) selected_value += encoder.handleSelection();  // If overflow constrain will fix in general handler below
    else if (tuning_ctrl == OFF) dataset_page += encoder.handleSelection();  // If overflow tconstrain will fix in general below

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "enc");  //

    // Tuning : implement effects of changes made by encoder or touchscreen to simulating, dataset_page, selected_value, or tuning_ctrl
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
            if (selected_value == 4) adj_bool (&sim_joy, sim_edit_delta);
            // else if (selected_value == 5) adj_bool (&sim_pressure, sim_edit_delta);
            else if (selected_value == 5) {
                 adj_bool(&sim_pressure, sim_edit_delta);
                 if (pot_overload != pressure)
                    if (simulating && sim_pressure)
                        pressure_sensor.set_source(ControllerMode::TOUCH);
                    else
                        pressure_sensor.set_source(ControllerMode::PIN); 
            }
            else if (selected_value == 6) adj_bool (&sim_brkpos, sim_edit_delta);
            else if (selected_value == 7) adj_bool (&sim_tach, sim_edit_delta);
            else if (selected_value == 8) adj_bool (&sim_airflow, sim_edit_delta);
            else if (selected_value == 9) adj_bool (&sim_speedo, sim_edit_delta);
            else if (selected_value == 10) {
                adj_val(&pot_overload, sim_edit_delta, 0, arraysize(sensorcard)-1);
                if (pot_overload == pressure)
                    pressure_sensor.set_source(ControllerMode::POT);
                else
                    if (simulating && sim_pressure)
                        pressure_sensor.set_source(ControllerMode::TOUCH);
                    else
                        pressure_sensor.set_source(ControllerMode::PIN);
            }
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
            if (selected_value == 3) {
                adj = adj_val (&gas_governor_percent, sim_edit_delta, 0, 100);
                if (adj) calc_governor();  // update derived variables relevant to changes made
            }
            else if (selected_value == 4) {
                adj = adj_val (&steer_safe_percent, sim_edit_delta, 0, 100);
                if (adj) calc_ctrl_lims();
            }
            else if (selected_value == 5) adj_val (&airflow_max_mph, 0.01*(float)sim_edit_delta, 0, airflow_abs_max_mph);
            else if (selected_value == 6) adj_val (&tach_idle_rpm, 0.01*(float)sim_edit_delta, 0, tach_redline_rpm - 1);
            else if (selected_value == 7) adj_val (&tach_redline_rpm, 0.01*(float)sim_edit_delta, tach_idle_rpm, 8000);
            else if (selected_value == 8) adj_val (&speedo_idle_mph, 0.01*(float)sim_edit_delta, 0, speedo_redline_mph - 1);
            else if (selected_value == 9) adj_val (&speedo_redline_mph, 0.01*(float)sim_edit_delta, speedo_idle_mph, 30);
            else if (selected_value == 10) adj_val (&brake_pos_zeropoint_in, 0.001*sim_edit_delta, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);

      }
        else if (dataset_page == PG_PWMS) {
            if (selected_value == 3) adj_val (&steer_pulse_left_us, sim_edit_delta, steer_pulse_stop_us + 1, steer_pulse_left_max_us);
            else if (selected_value == 4) adj_val (&steer_pulse_stop_us, sim_edit_delta, steer_pulse_right_us + 1, steer_pulse_left_us - 1);
            else if (selected_value == 5) adj_val (&steer_pulse_right_us, sim_edit_delta, steer_pulse_right_min_us, steer_pulse_stop_us - 1);
            else if (selected_value == 6) adj_val (&brake_pulse_extend_us, sim_edit_delta, brake_pulse_stop_us + 1, brake_pulse_extend_max_us);
            else if (selected_value == 7) adj_val (&brake_pulse_stop_us, sim_edit_delta, brake_pulse_retract_us + 1, brake_pulse_extend_us - 1);
            else if (selected_value == 8) adj_val (&brake_pulse_retract_us, sim_edit_delta, brake_pulse_retract_min_us, brake_pulse_stop_us -1);
            else if (selected_value == 9) adj_val (&gas_pulse_idle_us, sim_edit_delta, gas_pulse_redline_us + 1, gas_pulse_ccw_max_us - gas_pulse_park_slack_us);
            else if (selected_value == 10) adj_val (&gas_pulse_redline_us, sim_edit_delta, gas_pulse_cw_min_us, gas_pulse_idle_us - 1);
        }
        else if (dataset_page == PG_BPID) {
            if (selected_value == 8) brakeQPID.SetKp (brakeQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 9) brakeQPID.SetKi (brakeQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 10) brakeQPID.SetKd (brakeQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_GPID) {
            if (selected_value == 7) adj_bool (&gas_open_loop, sim_edit_delta);
            else if (selected_value == 8) gasQPID.SetKp (gasQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 9) gasQPID.SetKi (gasQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 10) gasQPID.SetKd (gasQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_CPID) {
            if (selected_value == 8) cruiseQPID.SetKp (cruiseQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 9) cruiseQPID.SetKi (cruiseQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 10) cruiseQPID.SetKd (cruiseQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_TEMP) {        
            if (selected_value == 9 && runmode == CAL) adj_bool (&cal_joyvert_brkmotor, sim_edit_delta);
            else if (selected_value == 10 && runmode == CAL) adj_bool (&cal_pot_gasservo, (sim_edit_delta < 0 || cal_pot_gas_ready) ? sim_edit_delta : -1);
        }
        sim_edit_delta = 0;
    }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tun");  //

    // Ignition & Panic stop logic and Update output signals
    if (!car_stopped()) {
        if (ctrl == HOTRC && !(simulating && sim_joy) && !hotrc_radio_detected) panic_stop = true;  // panic_stop could also have been initiated by the user button   && hotrc_radio_detected_last 
        else if (ctrl == JOY && !ignition) panic_stop = true;
        // else if (ctrl == JOY && !(simulating && sim_joy) && !ignition && ignition_last) panic_stop = true;
    }
    else if (panic_stop) panic_stop = false;  // Cancel panic if car is stopped
    if (ctrl == HOTRC) {  // When using joystick, ignition is controlled with button and we read it. With Hotrc, we control ignition
        hotrc_radio_detected_last = hotrc_radio_detected;
        if (panic_stop) ignition = LOW;  // Kill car if panicking
        if ((ignition != ignition_last) && ignition_output_enabled) {  // Whenever ignition state changes, assuming we're allowed to write to the pin
            write_pin (ign_out_pin, !ignition);  // Turn car off or on (ign output is active low), ensuring to never turn on the ignition while panicking
            ignition_last = ignition;  // Make sure this goes after the last comparison
        }
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
    // cout << "starter:" << starter << " starting:" << remote_starting << endl;;

    if (syspower != syspower_last) {
        syspower = syspower_set (syspower);
        syspower_last = syspower;
    }
    if (boot_button_action == LONG) {
        screen.tft_reset();
        boot_button_action = NONE;
    }
    if (!screen.get_reset_finished()) screen.tft_reset();  // If resetting tft, keep calling tft_reset until complete
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
    
    oldmode = runmode;  // remember what mode we're in for next time
    dataset_page_last = dataset_page;
    selected_value_last = selected_value;
    tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
    simulating_last = simulating;

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "dis");

    // Kick watchdogs
    Watchdog.reset();  // Kick the watchdog to keep us alive
    // if (display_enabled) screen.watchdog();
 
    // Do the control loop bookkeeping at the end of each loop
    //
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