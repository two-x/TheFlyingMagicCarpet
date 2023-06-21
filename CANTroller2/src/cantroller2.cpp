// Carpet CANTroller II  Source Code  - For Arduino Due with Adafruit 2.8inch Captouch TFT shield.
#include <SPI.h>  // SPI serial bus needed to talk to the LCD and the SD card
#include <Wire.h>  // Contains I2C serial bus, needed to talk to touchscreen chip
#include <SdFat.h>  // SD card & FAT filesystem library
#include <Servo.h>  // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
#include <Adafruit_ILI9341.h>  // For interfacing with the TFT LCD controller chip
#ifdef DUE
    #include <LibPrintf.h>  // This works on Due but not ESP32
#endif
#include <Adafruit_NeoPixel.h>  // Plan to allow control of neopixel LED onboard the esp32
#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <vector>
#include <string>
#include <iomanip>
#include "classes.h"  // Contains our data structures
#include "spid.h"
#include "globals.h"
using namespace std;

double temp_min = -67.0;  // Minimum reading of sensor is -25 C = -67 F
double temp_max = 257.0;  // Maximum reading of sensor is 125 C = 257 F
double temp_room = 77.0;  // "Room" temperature is 25 C = 77 F

std::vector<string> loop_names(20);

void loop_savetime (uint32_t timesarray[], int32_t &index, vector<string> &names, bool dirty[], string loopname) {  // (int32_t timesarray[], int32_t index) {
    if (dirty[index]) {
        names[index] = loopname;  // names[index], name);
        dirty[index] = false;
    }
    timesarray[index] = micros();
    index++;
}

Hotrc hotrc ( (hotrc_pulse_vert_max_us+hotrc_pulse_vert_min_us)/2 );
    
void setup() {
    set_pin (heartbeat_led_pin, OUTPUT);
    set_pin (encoder_a_pin, INPUT_PULLUP);
    set_pin (encoder_b_pin, INPUT_PULLUP);
    set_pin (encoder_sw_pin, INPUT_PULLUP);  // The esp32 pullup is too weak. Use resistor
    set_pin (brake_pwm_pin, OUTPUT);
    set_pin (steer_pwm_pin, OUTPUT);
    set_pin (tft_dc_pin, OUTPUT);
    set_pin (gas_pwm_pin, OUTPUT);
    set_pin (ignition_pin, OUTPUT);  // drives relay to turn on/off car. Active high
    set_pin (basicmodesw_pin, INPUT_PULLUP);
    set_pin (tach_pulse_pin, INPUT_PULLUP);
    set_pin (speedo_pulse_pin, INPUT_PULLUP);
    set_pin (joy_horz_pin, INPUT);
    set_pin (joy_vert_pin, INPUT);
    set_pin (pressure_pin, INPUT);
    set_pin (brake_pos_pin, INPUT);
    set_pin (battery_pin, INPUT);
    set_pin (hotrc_horz_pin, INPUT);
    set_pin (hotrc_vert_pin, INPUT);
    set_pin (hotrc_ch3_pin, INPUT);
    set_pin (hotrc_ch4_pin, INPUT);
    set_pin (neopixel_pin, OUTPUT);
    set_pin (sdcard_cs_pin, OUTPUT);
    set_pin (tft_cs_pin, OUTPUT);
    set_pin (pot_wipe_pin, INPUT);
    set_pin (button_pin, INPUT_PULLUP);    
    set_pin (syspower_pin, OUTPUT);
    set_pin (cruise_sw_pin, INPUT_PULLUP);
    set_pin (tp_irq_pin, INPUT_PULLUP);
    set_pin (led_rx_pin, OUTPUT);
    // set_pin (led_tx_pin, OUTPUT);
    set_pin (encoder_pwr_pin, OUTPUT);
    // set_pin (tft_ledk_pin, OUTPUT);
    // set_pin (onewire_pin, OUTPUT);
    set_pin (tft_rst_pin, OUTPUT);

    write_pin (ignition_pin, ignition);
    write_pin (tft_cs_pin, HIGH);   // Prevent bus contention
    write_pin (sdcard_cs_pin, HIGH);   // Prevent bus contention
    write_pin (tft_dc_pin, LOW);
    write_pin (led_rx_pin, LOW);  // Light up
    // write_pin (led_tx_pin, HIGH);  // Off
    write_pin (syspower_pin, syspower);
    write_pin (encoder_pwr_pin, HIGH);
    write_pin (tft_rst_pin, HIGH);
    
    analogReadResolution (adcbits);  // Set Arduino Due to 12-bit resolution (default is same as Mega=10bit)
    Serial.begin (115200);  // Open serial port
    // printf("Serial port open\n");  // This works on Due but not ESP32
    
    for (int32_t x=0; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
    
    if (display_enabled) {
        delay (500); // This is needed to allow the screen board enough time after a cold boot before we start trying to talk to it.
        tft_init();
    }
    //     Serial.print (F("Init LCD... "));
    //     tft.begin();
    //     tft.setRotation (1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
    //     for (int32_t lineno=0; lineno <= arraysize (telemetry); lineno++)  {
    //         disp_age_quanta[lineno] = -1;
    //         memset (disp_values[lineno],0,strlen (disp_values[lineno]));
    //     }
    //     for (int32_t row=0; row<arraysize (disp_bool_values); row++) disp_bool_values[row] = 1;
    //     for (int32_t row=0; row<arraysize (disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
    //     for (int32_t row=0; row<arraysize (disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen

    //     tft.fillScreen (BLK);  // Black out the whole screen
    //     draw_fixed (false);
    //     draw_touchgrid (false);
    //     Serial.println (F("Success"));

    //     Serial.print(F("Captouch initialization... "));
    //     if (! touchpanel.begin(40)) {     // pass in 'sensitivity' coefficient
    //         Serial.println (F("Couldn't start FT6206 touchscreen controller"));
    //         // while (1);
    //     }
    //     else Serial.println (F("Capacitive touchscreen started"));
    // }
    neostrip.begin();  // start datastream
    neostrip.show();  // Turn off the pixel
    neostrip.setBrightness (neopixel_brightness);  // It truly is incredibly bright
    // 230417 removing sdcard init b/c boot is hanging here unless I insert this one precious SD card
    // Serial.print(F("Initializing filesystem...  "));  // SD card is pretty straightforward, a single call. 
    // if (! sd.begin(usd_cs_pin, SD_SCK_MHZ(25))) {   // ESP32 requires 25 mhz limit
    //     Serial.println(F("SD begin() failed"));
    //     for(;;); // Fatal error, do not continue
    // }
    // sd_init();
    // Serial.println(F("Filesystem started"));

    // Set up our interrupts
    Serial.print (F("Interrupts... "));
    attachInterrupt (digitalPinToInterrupt(tach_pulse_pin), tach_isr, RISING);
    attachInterrupt (digitalPinToInterrupt(speedo_pulse_pin), speedo_isr, RISING);
    // attachInterrupt (digitalPinToInterrupt(encoder_a_pin), encoder_a_rise_isr, RISING); // One type of encoder (e.g. Panasonic EVE-YBCAJ016B) needs Rising int on pin A only
    // attachInterrupt (digitalPinToInterrupt(encoder_a_pin), encoder_a_fall_isr, FALLING); // One type of encoder (e.g. Panasonic EVE-YBCAJ016B) needs Rising int on pin A only
    attachInterrupt (digitalPinToInterrupt(encoder_a_pin), encoder_a_isr, CHANGE);
    attachInterrupt (digitalPinToInterrupt(encoder_b_pin), encoder_b_isr, CHANGE);
    // attachInterrupt (digitalPinToInterrupt(hotrc_vert_pin), hotrc_vert_rise_isr, RISING);
    // attachInterrupt (digitalPinToInterrupt(hotrc_vert_pin), hotrc_vert_fall_isr, FALLING);
    attachInterrupt (digitalPinToInterrupt(hotrc_vert_pin), hotrc_vert_isr, CHANGE);
    attachInterrupt (digitalPinToInterrupt(hotrc_horz_pin), hotrc_horz_isr, FALLING);
    attachInterrupt (digitalPinToInterrupt(hotrc_ch3_pin), hotrc_ch3_isr, FALLING);
    attachInterrupt (digitalPinToInterrupt(hotrc_ch4_pin), hotrc_ch4_isr, FALLING);
    
    Serial.println (F("set up and enabled\n"));
    
    // Set up the soren pid loops
    brakeSPID.set_output_center ((double)brake_pulse_stop_us);  // Sets actuator centerpoint and puts pid loop in output centerpoint mode. Becasue actuator value is defined as a deviation from a centerpoint
    brakeSPID.set_input_limits (pressure_min_psi, pressure_max_psi);  // Make sure pressure target is in range
    brakeSPID.set_output_limits ((double)brake_pulse_retract_us, (double)brake_pulse_extend_us);
    gasSPID.set_input_limits (tach_idle_rpm, tach_govern_rpm);
    gasSPID.set_output_limits ((double)gas_pulse_govern_us, (double)gas_pulse_idle_us);
    cruiseSPID.set_input_limits (carspeed_idle_mmph, carspeed_govern_mmph);
    cruiseSPID.set_output_limits (tach_idle_rpm, tach_govern_rpm);
      
    steer_servo.attach (steer_pwm_pin);
    brake_servo.attach (brake_pwm_pin);
    gas_servo.attach (gas_pwm_pin);

    neopixel_heartbeat = (neopixel_pin >= 0);
    neostrip.begin();
    neostrip.show(); // Initialize all pixels to 'off'
    neostrip.setBrightness (neopixel_brightness);

    tempsensebus.setWaitForConversion (true);  // Do not block during conversion process
    tempsensebus.setCheckForConversion (true);  // Do not listen to device for conversion result, instead we will wait the worst-case period
    tempsensebus.begin();
    temp_detected_device_ct = tempsensebus.getDeviceCount();
    printf ("Temp sensors: Detected %d devices.\nParasitic power is: ", temp_detected_device_ct);  // , DEC);
    printf ((tempsensebus.isParasitePowerMode()) ? "On\n" : "Off\n");
    // for (int32_t x = 0; x < arraysize(temp_addrs); x++) {
    for (int32_t x = 0; x < temp_detected_device_ct; x++) {
        if (tempsensebus.getAddress (temp_temp_addr, x)) printf ("Found sensor device: index %d, addr %d\n", x, temp_temp_addr);  // temp_addrs[x]
        else printf ("Found ghost device : index %d, addr unknown\n", x);  // printAddress (temp_addrs[x]);
        tempsensebus.setResolution (temp_temp_addr, temperature_precision);  // temp_addrs[x]
    }

    // int32_t start = mycros();
    // tempsensebus.requestTemperatures();
    // int32_t mid = mycros();
    // double temp = tempsensebus.getTempCByIndex(0);
    // int32_t done = mycros();
    // printf ("Test blocking request: %ld us, received: %ld us, temp = %lf.\n", mid-start, done-mid, temp);
    // start = mycros();
    // tempsensebus.setWaitForConversion (false);  // makes it async
    // tempsensebus.requestTemperatures();
    // tempsensebus.setWaitForConversion (true);
    // mid = mycros();
    // delay (750 / (1 << (12 - temperature_precision)));
    // temp = tempsensebus.getTempCByIndex(0);
    // done = mycros();
    // printf (" Non-blocking request: %ld us, received: %ld us, temp = %lf.\n", mid-start, done-mid, temp);
    
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
    hotrcPanicTimer.reset();
    loopTimer.reset();  // start timer to measure the first loop
    Serial.println (F("Setup finished"));
}

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
    // Beginning-of-the-loop nonsense
    //
    loopindex = 0;  // reset at top of loop
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "top");

    // Update inputs.  Fresh sensor data, and filtering.
    //

    // Onboard devices - takes 12 us to read
    if (button_pin >= 0) {  // if encoder sw is being pressed (switch is active low)
        button_it = !digitalRead (button_pin);
        // if (button_it != button_last) Serial.println ("button\n");
    }

    // External digital signals - takes 11 us to read
    if (!simulating || !sim_basicsw) basicmodesw = !digitalRead (basicmodesw_pin);   // 1-value because electrical signal is active low
    if (!simulating || !sim_cruisesw) cruise_sw = digitalRead (cruise_sw_pin);

    // Temperature sensors
    // for (uint8_t x = 0; x < arraysize(temp_addrs); x++) {
    //     temps[x] = get_temp (temp_addrs[x]);
    // }

    int32_t temp_start, temp_mid, temp_done;
    if (tempTimer.expired()) {
        if (temp_status == IDLE) {
            wait_one_loop = true;
            if (++temp_current_index >= 2) temp_current_index -= 2;  // replace 1 with arraysize(temps)
            tempsensebus.setWaitForConversion (false);  // makes it async
            tempsensebus.requestTemperatures();
            tempsensebus.setWaitForConversion (true);
            tempTimer.set(180000);  // Give some time before reading temp
            temp_status = CONVERT;
        }
        else if (temp_status == CONVERT) {
            wait_one_loop = true;
            temps[temp_current_index] = tempsensebus.getTempFByIndex(temp_current_index);
            tempTimer.set(1500000);
            temp_status = DELAY;
        }
        else if (temp_status == DELAY) {
            // printf ("temps[%ld] = %lf F\n", temp_current_index, temps[temp_current_index]);
            tempTimer.set(60000);
            temp_status = IDLE;
        }
    }

    // Encoder - takes 10 us to read when no encoder activity
    // Read and interpret encoder switch activity. Encoder rotation is handled in interrupt routine
    // Encoder handler routines should act whenever encoder_sw_action is true, setting it back to false once handled.
    // When handling press, if encoder_long_clicked is nonzero then press is a long press
    if (!digitalRead (encoder_sw_pin)) {  // if encoder sw is being pressed (switch is active low)
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

    // Potentiometer - takes 400 us to read & convert (?!)
    pot_percent = convert_units ((double)analogRead (pot_wipe_pin), pot_convert_percent_per_adc, pot_convert_invert);  // Potentiometer
    ema_filt (pot_percent, &pot_filt_percent, pot_ema_alpha);
    
    // Voltage of vehicle battery - takes 70 us to read, convert, and filter
    battery_mv = convert_units ((double)analogRead (battery_pin), battery_convert_mv_per_adc, battery_convert_invert);
    ema_filt (battery_mv, &battery_filt_mv, battery_ema_alpha);  // Apply EMA filter

       // Brake position - takes 70 us to read, convert, and filter
    if (!simulating || !sim_brkpos) {
        brake_pos_thou = convert_units ((double)analogRead (brake_pos_pin), brake_pos_convert_thou_per_adc, brake_pos_convert_invert);
        ema_filt (brake_pos_thou, &brake_pos_filt_thou, brake_pos_ema_alpha);
    }
    else brake_pos_filt_thou = (brake_pos_nom_lim_retract_thou + brake_pos_zeropoint_thou)/2;  // To keep brake position in legal range during simulation
    
    // Tach - takes 22 us to read when no activity
    if (!simulating || !sim_tach) {
        if (tachPulseTimer.elapsed() < tach_stop_timeout_us) tach_rpm = convert_units ((double)(tach_delta_us), tach_convert_rpm_per_rpus, tach_convert_invert);
        else tach_rpm = 0;  // If timeout since last magnet is exceeded
        ema_filt (tach_rpm, &tach_filt_rpm, tach_ema_alpha);  // Sensor EMA filter
    }
    
    // Speedo - takes 14 us to read when no activity
    if (!simulating || !sim_speedo) { 
        if (speedoPulseTimer.elapsed() < speedo_stop_timeout_us) carspeed_mmph = convert_units ((double)(speedo_delta_us), speedo_convert_mmph_per_rpus, speedo_convert_invert);  // Update car speed value  
        else carspeed_mmph = 0;     
        ema_filt (carspeed_mmph, &carspeed_filt_mmph, carspeed_ema_alpha);  // Sensor EMA filter
    }

    // Brake pressure - takes 72 us to read
    if (!simulating || !sim_pressure) {
        pressure_psi = convert_units ((double)analogRead (pressure_pin), pressure_convert_psi_per_adc, pressure_convert_invert);  // Brake pressure.  Read sensor, then Remove noise spikes from brake feedback, if reading is otherwise in range
        ema_filt (pressure_psi, &pressure_filt_psi, pressure_ema_alpha);  // Sensor EMA filter
    }

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "inp");  //

    // Controller handling
    //
    // Read horz and vert inputs - takes 40 us to read
    if (!simulating || !sim_joy) {  // If not simulating or joystick simulation is disabled
        if (ctrl == HOTRC) {  // If using hotrc handle
            ctrl_pos_adc[VERT][RAW] = map (hotrc_vert_pulse_us, hotrc_pulse_vert_max_us, hotrc_pulse_vert_min_us, ctrl_lims_adc[ctrl][VERT][MAX], ctrl_lims_adc[ctrl][VERT][MIN]);
            ctrl_pos_adc[HORZ][RAW] = map (hotrc_horz_pulse_us, hotrc_pulse_horz_max_us, hotrc_pulse_horz_min_us, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
            ctrl_pos_adc[VERT][RAW] = constrain (ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
            ctrl_pos_adc[HORZ][RAW] = constrain (ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);   
        }
        else {  // If using old joystick
            ctrl_pos_adc[VERT][RAW] = analogRead (joy_vert_pin);  // Read joy vertical
            ctrl_pos_adc[HORZ][RAW] = analogRead (joy_horz_pin);  // Read joy horizontal
        }        
        if (ctrl_pos_adc[VERT][RAW] > ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][RAW] < ctrl_db_adc[VERT][TOP])  ctrl_pos_adc[VERT][FILT] = adcmidscale_adc;  // if joy vert is in the deadband, set joy_vert_filt to center value
        else ema_filt (ctrl_pos_adc[VERT][RAW], &ctrl_pos_adc[VERT][FILT], ctrl_ema_alpha[ctrl]);  // otherwise do ema filter to determine joy_vert_filt
        if (ctrl_pos_adc[HORZ][RAW] > ctrl_db_adc[HORZ][BOT] && ctrl_pos_adc[HORZ][RAW] < ctrl_db_adc[HORZ][TOP])  ctrl_pos_adc[HORZ][FILT] = adcmidscale_adc;  // if joy horz is in the deadband, set joy_horz_filt to center value
        else ema_filt (ctrl_pos_adc[HORZ][RAW], &ctrl_pos_adc[HORZ][FILT], ctrl_ema_alpha[ctrl]);  // otherwise do ema filter to determine joy_horz_filt
    }
 
    // Determine steering pwm output - takes 13 us to handle
    if (runmode != SHUTDOWN || !shutdown_complete)  { // Unless fully shut down at the moment, set the steering output
        if (ctrl_pos_adc[HORZ][FILT] >= ctrl_db_adc[HORZ][TOP]) {
            steer_pulse_safe_us = steer_pulse_stop_us + (int32_t)((double)(steer_pulse_right_us - steer_pulse_stop_us) * (1 - ((double)steer_safe_percent * carspeed_filt_mmph / ((double)carspeed_redline_mmph * 100) )));
            steer_pulse_out_us = map (ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][TOP], ctrl_lims_adc[ctrl][HORZ][MAX], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the right of deadband
        }
        else if (ctrl_pos_adc[HORZ][FILT] <= ctrl_db_adc[HORZ][BOT]) {
            steer_pulse_safe_us = steer_pulse_stop_us - (int32_t)((double)(steer_pulse_stop_us - steer_pulse_left_us) * (1 - ((double)steer_safe_percent * carspeed_filt_mmph / ((double)carspeed_redline_mmph * 100) )));
            steer_pulse_out_us = map (ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][BOT], ctrl_lims_adc[ctrl][HORZ][MIN], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the left of deadband
        }
        else steer_pulse_out_us = steer_pulse_stop_us;  // Stop the steering motor if inside the deadband
    }
    // Handle HotRC button generated events and detect potential loss of radio signal - takes 15 us to handle
    if (ctrl == HOTRC) {
        if (hotrc_ch3_sw_event) {  // Turn on/off the vehicle ignition
            if (hotrc_suppress_next_event) hotrc_suppress_next_event = false;
            else {
                ignition = !ignition;
                hotrc_ch3_sw_event = false;
            }
        }
        if (hotrc_ch4_sw_event && !hotrc_suppress_next_event) {  // Toggle cruise/fly mode 
            if (runmode == FLY) runmode = CRUISE;
            else if (runmode == CRUISE) runmode = FLY;
            hotrc_ch4_sw_event = false;    
        }
        // Detect loss of radio reception and panic stop
        if (ctrl_pos_adc[VERT][FILT] > hotrc_pos_failsafe_min_us && ctrl_pos_adc[VERT][FILT] < hotrc_pos_failsafe_max_us) {
            if (hotrcPanicTimer.expired()) {
                hotrc_radio_detected = false;
                hotrc_suppress_next_event = true;  // reject spurious ch3 switch event upon next hotrc poweron
            }
        }
        else {
            hotrcPanicTimer.reset();
            hotrc_radio_detected = true;
        }
    }

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "joy");  //
    
    // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    //
    // printf("mode: %d, panic: %d, vpos: %4ld, min: %4ld, max: %4ld, elaps: %6ld", runmode, panic_stop, ctrl_pos_adc[VERT][FILT], hotrc_pos_failsafe_min_us, hotrc_pos_failsafe_max_us, hotrcPanicTimer.elapsed());
    if (basicmodesw) runmode = BASIC;  // if basicmode switch on --> Basic Mode
    else if (runmode != CAL && (panic_stop || !ignition)) runmode = SHUTDOWN;
    else if (runmode != CAL && !tach_filt_rpm) runmode = STALL;  // otherwise if engine not running --> Stall Mode
    
    if (runmode == BASIC) {  // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering stell works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            // syspower = HIGH;  // Power up devices if not already
            gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        if (tach_filt_rpm && !basicmodesw) runmode = HOLD;  // If we turned off the basic mode switch with engine running, go to Hold mode. If engine is not running, we'll end up in Stall Mode automatically
    }
    else if (runmode == SHUTDOWN) { // In shutdown mode we stop the car if it's moving then park the motors.
        if (ignition && !panic_stop) {
            // syspower = HIGH;  // Power up devices if not already
            runmode = STALL;
        }
        else if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            gasSPID.set_target (tach_idle_rpm);  //  Release the throttle 
            shutdown_complete = false;
            shutdown_color = LPNK;
            calmode_request = false;
            if (carspeed_filt_mmph) {
                if (panic_stop && brakeSPID.get_target() < pressure_panic_initial_psi) brakeSPID.set_target (pressure_panic_initial_psi);
                if (!panic_stop && brakeSPID.get_target() < pressure_hold_initial_psi) brakeSPID.set_target (pressure_hold_initial_psi);
                brakeIntervalTimer.reset();
                sanityTimer.reset();
            }
        }
        if (!shutdown_complete) {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (!carspeed_filt_mmph || sanityTimer.expired()) {  // If car has stopped, or timeout expires, then release the brake
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
            else if (brakeIntervalTimer.expired()) {
                brakeSPID.set_target (brakeSPID.get_target() + (panic_stop) ? pressure_panic_increment_psi : pressure_hold_increment_psi);  // Slowly add more brakes until car stops
                brakeIntervalTimer.reset();  
            }
        }
        else if (calmode_request) {  // if fully shut down and cal mode requested
            // syspower = HIGH;  // Power up devices if not already
            runmode = CAL;
        }
        else if (sleepInactivityTimer.expired()) {
            // syspower = LOW;  // Power down devices
            // go to sleep?    
    }
    }
    else if (runmode == STALL) {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (tach_filt_rpm) runmode = HOLD;  // Enter Hold Mode if we started the car
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][BOT]) brakeSPID.set_target (pressure_min_psi);  // If in deadband or being pushed up, no pressure target
        else brakeSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_db_adc[VERT][BOT], (double)ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_psi, pressure_max_psi));  // Scale joystick value to pressure adc setpoint
        // Throttle behavior is handled in pid section
    }
    else if (runmode == HOLD) {
        if (we_just_switched_modes)  {  // Release throttle and push brake upon entering hold mode
            gasSPID.set_target (tach_idle_rpm);  // Let off gas (if gas using PID mode)
            if (!carspeed_filt_mmph) brakeSPID.set_target (pressure_filt_psi + pressure_hold_increment_psi); // If the car is already stopped then just add a touch more pressure and then hold it.
            else if (brakeSPID.get_target() < pressure_hold_initial_psi) brakeSPID.set_target (pressure_hold_initial_psi);  //  These hippies need us to stop the car for them
            brakeIntervalTimer.reset();
            sanityTimer.reset();
            joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
        }
        else if (brakeIntervalTimer.expired() && !sanityTimer.expired()) {  // Each interval the car is still moving, push harder
            if (carspeed_filt_mmph) brakeSPID.set_target (brakeSPID.get_target() + pressure_hold_increment_psi);
            brakeIntervalTimer.reset();
        }
        if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered) runmode = FLY; // Enter Fly Mode upon joystick movement from center to above center
        }
    else if (runmode == FLY) {
        if (we_just_switched_modes)  {
            gesture_progress = 0;
            gestureFlyTimer.set (gesture_flytimeout_us); // Initialize gesture timer to already-expired value
            cruise_sw_held = false;
            cruiseSwTimer.reset();
            hotrc_ch4_sw_event = false;
        }
        if (!carspeed_filt_mmph) runmode = HOLD;  // Go to Hold Mode if we have come to a stop  // && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT]
        else  {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP])  {  // If we are trying to accelerate
                gasSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_db_adc[VERT][TOP], (double)ctrl_lims_adc[ctrl][VERT][MAX], tach_idle_rpm, tach_govern_rpm));
            }
            else gasSPID.set_target (tach_idle_rpm);  // Let off gas (if gas using PID mode)
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT])  {  // If we are trying to brake, scale joystick value to determine pressure adc setpoint
                brakeSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_db_adc[VERT][BOT], (double)ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_psi, pressure_max_psi));
            }
            else brakeSPID.set_target (pressure_min_psi);  // Default when joystick not pressed   
        }
        // Cruise mode can be entered by pressing a physical momentary button, or by holding the brake on full for a half second. Which epends on the cruise_gesturing flag.
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (cruise_gesturing) {  // If we are configured to use joystick gestures to go to cruise mode, the gesture is 
            if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP])  { // Re-zero gesture timer for potential new gesture whenever joystick at center
                gestureFlyTimer.reset();
            }
            if (gestureFlyTimer.expired()) gesture_progress = 0; // If gesture timeout has expired, cancel any in-progress gesture
            else {  // Otherwise check for successful gesture motions
                if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_lims_adc[ctrl][VERT][MAX] - default_margin_adc)  { // If joystick quickly pushed to top, step 1 of gesture is successful
                    gesture_progress++;
                    gestureFlyTimer.reset();
                }
                else if (gesture_progress == 1 && ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc)  { // If joystick then quickly pushed to bottom, step 2 succeeds
                    gesture_progress++;
                    gestureFlyTimer.reset();
                }
                else if (gesture_progress == 2 && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP]) { // If joystick then quickly returned to center, go to Cruise mode
                    runmode = CRUISE;
                }        
            }
        }
        if (!cruise_sw) {  // If button not currently pressed
            if (cruise_sw_held && cruiseSwTimer.expired()) runmode = CRUISE;  // After a long press of sufficient length, upon release enter Cruise mode
            cruise_sw_held = false;  // Cancel button held state
        }
        else if (!cruise_sw_held) {  // If the button just now got pressed
            cruiseSwTimer.reset(); // Start hold time timer
            cruise_sw_held = true;  // Get into button held state
        }
    }
    else if (runmode == CRUISE)  {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            cruiseSPID.set_target (carspeed_filt_mmph);
            brakeSPID.set_target (pressure_min_psi);  // Let off the brake and keep it there till out of Cruise mode
            gestureFlyTimer.reset();  // reset gesture timer
            cruise_sw_held = false;
            hotrc_ch4_sw_event = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) {  // When joystick vert above center, increase the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            gasSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_db_adc[VERT][TOP], (double)ctrl_lims_adc[ctrl][VERT][MAX], tach_filt_rpm, tach_govern_rpm));
        }
        else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) {  // When joystick vert below center, decrease the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            gasSPID.set_target (map ((double)ctrl_pos_adc[VERT][FILT], (double)ctrl_lims_adc[ctrl][VERT][MIN], (double)ctrl_db_adc[VERT][BOT], tach_idle_rpm, tach_filt_rpm));
        }
        else cruise_adjusting = false;  // if joystick at center
        if (cruise_adjusting) cruiseSPID.set_target (carspeed_filt_mmph);
        
        // This old gesture trigger drops to Fly mode if joystick moved quickly from center to bottom
        // if (ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN]+default_margin_adc && abs(mycros() - gesture_timer_us) < gesture_flytimeout_us)  runmode = FLY;  // If joystick quickly pushed to bottom 
        // printf ("hotvpuls=%ld, hothpuls=%ld, joyvfilt=%ld, joyvmin+marg=%ld, timer=%ld\n", hotrc_vert_pulse_us, hotrc_horz_pulse_us, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc, gesture_timer_us);
        if (ctrl_pos_adc[VERT][FILT] > ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) runmode = FLY;  // New gesture to drop to fly mode is hold the brake all the way down for 500 ms
        if (cruise_sw)  cruise_sw_held = true;  // Pushing cruise button sets up return to fly mode
        else if (cruise_sw_held) {  // Release of button drops us back to fly mode
            cruise_sw_held = false;
            runmode = FLY;
        }
        if (!carspeed_filt_mmph) {  // In case we slam into a brick wall, get out of cruise mode
            if (serial_debugging) Serial.println (F("Error: Car stopped or taken out of gear in cruise mode"));  // , carspeed_filt_mmph, neutral
            runmode = HOLD;  // Back to Hold Mode  
        }
    }
    else if (runmode == CAL) {
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            calmode_request = false;
            cal_pot_gas_ready = false;
            cal_set_hotrc_failsafe_ready = false;
        }
        else if (calmode_request) runmode = SHUTDOWN;
        if (!cal_pot_gas_ready) {
            double temp = map (pot_filt_percent, pot_min_percent, pot_max_percent, (double)gas_pulse_ccw_max_us, (double)gas_pulse_cw_min_us);
            if (temp <= (double)gas_pulse_idle_us && temp >= (double)gas_pulse_redline_us) cal_pot_gas_ready = true;
        }
        if (!cal_set_hotrc_failsafe_ready) {
            if (button_it && !button_last) cal_set_hotrc_failsafe_ready = true;
        }
        else if (button_it) hotrc.print();
        else if (button_last) {
            hotrc_pos_failsafe_min_us = hotrc.get_min();
            hotrc_pos_failsafe_max_us = hotrc.get_max();
            cout << "\nHotrc failsafe range set!  Min: " << hotrc_pos_failsafe_min_us << "us, Max: " << hotrc_pos_failsafe_max_us << " us, including " << hotrc.get_pad() << " us slop both ways" << std::endl;
            cal_set_hotrc_failsafe_ready = false;
        }
    }
    else { // Obviously this should never happen
        if (serial_debugging) Serial.println (F("Error: Invalid runmode entered"));  // ,  runmode
        runmode = SHUTDOWN;
    }

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "mod");  //

    // Update motor outputs - takes 185 us to handle every 30ms when the pid timer expires, otherwise 5 us
    //
    if (pidTimer.expired() && !(runmode == SHUTDOWN && shutdown_complete)) {  // Recalculate pid and update outputs, at regular intervals
        
        pidTimer.reset();  // reset timer to trigger the next update
        if ( park_the_motors && ( motorParkTimer.expired() ||  //  When parking motors, IF the timeout expires OR the brake and gas motors are both close enough to the park position ...
            ( abs(brake_pos_filt_thou - brake_pos_park_thou) <= brake_pos_margin_thou && gasServoTimer.expired() && (gas_pulse_out_us == gas_pulse_idle_us + gas_pulse_park_slack_us) ) ) )
            park_the_motors = false;  // ... THEN stop trying to park the motors
        
        // Steering
        steer_pulse_out_us = constrain (steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);  // Don't be out of range
        steer_servo.writeMicroseconds (steer_pulse_out_us);   // Write steering value to jaguar servo interface

        // Brakes
        if (runmode == CAL && cal_joyvert_brkmotor) {
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) brake_pulse_out_us = map (ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], brake_pulse_stop_us, brake_pulse_extend_us);
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) brake_pulse_out_us = map (ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_db_adc[VERT][BOT], brake_pulse_retract_us, brake_pulse_stop_us);
            else brake_pulse_out_us = brake_pulse_stop_us;
        }
        else if (park_the_motors) {
            if (brake_pos_filt_thou + brake_pos_margin_thou <= brake_pos_park_thou) brake_pulse_out_us =  // If brake is retracted from park point, extend toward park point, slowing as we approach
                map ((int32_t)brake_pos_filt_thou, (int32_t)brake_pos_park_thou, (int32_t)brake_pos_nom_lim_retract_thou, brake_pulse_stop_us, brake_pulse_extend_us);
            else if (brake_pos_filt_thou - brake_pos_margin_thou >= brake_pos_park_thou) brake_pulse_out_us =  // If brake is extended from park point, retract toward park point, slowing as we approach
                map ((int32_t)brake_pos_filt_thou, (int32_t)brake_pos_park_thou, (int32_t)brake_pos_nom_lim_extend_thou, brake_pulse_stop_us, brake_pulse_retract_us);
        }
        else if (runmode != BASIC) brake_pulse_out_us = (int32_t)brakeSPID.compute (pressure_filt_psi);  // Otherwise the pid control is active
        // printf("Brake PID rm=%-+4ld target=%-+9.4lf", runmode, (double)pressure_target_psi);   
        // printf(" output = %-+9.4lf,  %+-4ld\n", brakeSPID.get_output(), brake_pulse_out_us); 
        if (runmode != BASIC || park_the_motors) {
            if (runmode == CAL && cal_joyvert_brkmotor)  // In Cal mode constrain the motor to its entire range, instead of to the calibrated limits
                brake_pulse_out_us = constrain (brake_pulse_out_us, brake_pulse_retract_min_us, brake_pulse_extend_max_us);  // Send to the actuator. Refuse to exceed range    
            else {  // Prevent any movement of motor which would exceed position limits. Improve this by having the motor actively go back toward position range if position is beyond either limit
                if ( ((brake_pos_filt_thou + brake_pos_margin_thou <= brake_pos_nom_lim_retract_thou) && (brake_pulse_out_us < brake_pulse_stop_us)) ||  // If the motor is at or past its position limit in the retract direction, and we're intending to retract more ...
                    ((brake_pos_filt_thou - brake_pos_margin_thou >= brake_pos_nom_lim_extend_thou) && (brake_pulse_out_us > brake_pulse_stop_us)) )  // ... or same thing in the extend direction ...
                    brake_pulse_out_us = brake_pulse_stop_us;  // ... then stop the motor
                brake_pulse_out_us = constrain (brake_pulse_out_us, brake_pulse_retract_us, brake_pulse_extend_us);  // Send to the actuator. Refuse to exceed range    
            } 
            brake_servo.writeMicroseconds (brake_pulse_out_us);  // Write result to jaguar servo interface
        }
        
        // Cruise.  Controls gas rpm target to keep speed equal to cruise mmph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
        if (runmode == CRUISE && !cruise_adjusting) gasSPID.set_target (cruiseSPID.compute (carspeed_filt_mmph));  // 
            
        // cruiseSPID.set_target carspeed_target_mmph = constrain (carspeed_target_mmph, 0, carspeed_redline_mmph);
        // printf ("Cruise PID rm= %+-4ld target=%-+9.4lf", runmode, (double)carspeed_target_mmph);
        // cruiseSPID.compute (carspeed_filt_mmph);
        // tach_target_rpm
        // printf (" output = %-+9.4lf,  %+-4ld\n", cruiseSPID.get_output(), tach_target_rpm);
        // }

        // Gas.  Determine gas actuator output from rpm target.  PID loop is effective in Fly or Cruise mode.
        if (park_the_motors) gas_pulse_out_us = gas_pulse_idle_us + gas_pulse_park_slack_us;
        else if (runmode == STALL) {  // Stall mode runs the gas servo directly proportional to joystick. This is truly open loop
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) gas_pulse_out_us = gas_pulse_idle_us;  // If in deadband or being pushed down, we want idle
            else gas_pulse_out_us = map (ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], gas_pulse_idle_us, gas_pulse_govern_us);  // Actuators still respond and everything, even tho engine is turned off
        }
        else if (runmode != BASIC) {
            if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo) 
                gas_pulse_out_us = (int32_t)map (pot_filt_percent, pot_min_percent, pot_max_percent, (double)gas_pulse_ccw_max_us, (double)gas_pulse_cw_min_us);
            else if (gasSPID.get_open_loop())  // This isn't really open loop, more like simple proportional control, with output set proportional to target 
                gas_pulse_out_us = (int32_t)map (gasSPID.get_target(), tach_idle_rpm, tach_govern_rpm, (double)gas_pulse_idle_us, (double)gas_pulse_govern_us); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
            else gas_pulse_out_us = (int32_t)gasSPID.compute (tach_filt_rpm);  // Do proper pid math to determine gas_pulse_out_us from engine rpm error
            // printf ("Gas PID   rm= %+-4ld target=%-+9.4lf", runmode, (double)tach_target_rpm);
            // printf (" output = %-+9.4lf,  %+-4ld\n", gasSPID.get_output(), gas_pulse_out_us);
        }
        if (runmode != BASIC || park_the_motors) {
            if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo)
                gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
            else gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_govern_us, gas_pulse_idle_us);
            gas_servo.writeMicroseconds (gas_pulse_out_us);  // Write result to servo
        }
        pidTimer.reset();  // reset timer to trigger the next update
    }

    // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
    //
    // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
    // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
    // retreive with an OBD tool. Eventually this should include functions allowing us to detect things like:
    //  1. A sensor or actuator is unplugged, movement blocked, missing magnetic pulses, etc.
    //  2. Air in the brake lines.
    //  3. Axle/brake drum may be going bad (increased engine RPM needed to achieve certain carspeed)  (beware going up hill may look the same).
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
    if (!ignition && ignition_last && tach_filt_rpm > 0) { // See if the engine is turning despite the ignition being off
        Serial.println (F("Detected engine rotation in the absense of ignition signal"));  // , tach_filt_rpm, ignition
        // I don't think we really need to panic about this, but it does seem curious. Actually this will probably occur when we're sliding
        // into camp after a ride, and kill the engine before we stop the car. For a fraction of a second the engine would keep turning anyway.
        // Or fopr that matter whenever the carb is out of tune and making the engine diesel after we kill the ign.
    }

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pid");  //
        
    // Touchscreen handling - takes 800 us to handle every 20ms when the touch timer expires, otherwise 20 us (includes touch timer + encoder handling w/o activity)
    //
    int32_t touch_x, touch_y, trow, tcol;
    // if (touchPollTimer.expired()) {
    // touchPollTimer.reset();
    if (touchpanel.touched() == 1 && !wait_one_loop) { // Take actions if one touch is detected. This panel can read up to two simultaneous touchpoints
        wait_one_loop = true;
        touch_accel = 1 << touch_accel_exponent;  // determine value editing rate
        TS_Point touchpoint = touchpanel.getPoint();  // Retreive a point
        touchpoint.x = map (touchpoint.x, 0, disp_height_pix, disp_height_pix, 0);  // Rotate touch coordinates to match tft coordinates
        touchpoint.y = map (touchpoint.y, 0, disp_width_pix, disp_width_pix, 0);  // Rotate touch coordinates to match tft coordinates
        touch_y = disp_height_pix-touchpoint.x; // touch point y coordinate in pixels, from origin at top left corner
        touch_x = touchpoint.y; // touch point x coordinate in pixels, from origin at top left corner
        trow = constrain((touch_y + touch_fudge)/touch_cell_v_pix, 0, 4);  // The -8 seems to be needed or the vertical touch seems off (?)
        tcol = (touch_x-touch_margin_h_pix)/touch_cell_h_pix;
        // else printf("Touch: x:%ld, y:%ld, row:%ld, col:%ld\n", touch_x, touch_y, trow, tcol);
        // Take appropriate touchscreen actions depending how we're being touched
        if (tcol==0 && trow==0 && !touch_now_touched) {
            if (++dataset_page >= arraysize(pagecard)) dataset_page -= arraysize(pagecard);  // Displayed dataset page can also be changed outside of simulator
        }
        else if (tcol==0 && trow==1) {  // Long touch to enter/exit editing mode, if in editing mode, press to change selection of item to edit
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
        else if (tcol==0 && trow==2) {  // Pressed the increase value button, for real time tuning of variables
            if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If just entering edit mode, don't change value yet
            else if (tuning_ctrl == EDIT) sim_edit_delta_touch = touch_accel;  // If in edit mode, increase value
        }   
        else if (tcol==0 && trow==3) {  // Pressed the decrease value button, for real time tuning of variables
            if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If just entering edit mode, don't change value yet
            else if (tuning_ctrl == EDIT) sim_edit_delta_touch = -touch_accel;  // If in edit mode, decrease value
        }
        else if (tcol==0 && trow==4) {  // && trow == 0 . Pressed the simulation mode toggle. Needs long press
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout()) {
                simulating = !simulating;
                touch_longpress_valid = false;
            }
        }
        else if (tcol==2 && trow==0 && (runmode == CAL || (runmode == SHUTDOWN && shutdown_complete))) {
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout()) {
                calmode_request = true;
                touch_longpress_valid = false;
            }  // Pressed the basic mode toggle button. Toggle value, only once per touch
        }
        else if (simulating) {
            if (tcol==3 && trow==0 && sim_basicsw && !touch_now_touched) basicmodesw = !basicmodesw;  // Pressed the basic mode toggle button. Toggle value, only once per touch
            else if (tcol==3 && trow==1 && sim_pressure) adj_val (&pressure_filt_psi, touch_accel, pressure_min_psi, pressure_max_psi);   // (+= 25) Pressed the increase brake pressure button
            else if (tcol==3 && trow==2 && sim_pressure) adj_val (&pressure_filt_psi, -touch_accel, pressure_min_psi, pressure_max_psi); // (-= 25) Pressed the decrease brake pressure button
            else if (tcol==3 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[HORZ][FILT], -touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (-= 25) Pressed the joystick left button
            else if (tcol==4 && trow==0 && sim_ign && !touch_now_touched) ignition = !ignition; // Pressed the ignition switch toggle button. Toggle value, only once per touch
            else if (tcol==4 && trow==1 && sim_tach) adj_val (&tach_filt_rpm, touch_accel, 0, tach_redline_rpm);  // (+= 25) Pressed the increase engine rpm button
            else if (tcol==4 && trow==2 && sim_tach) adj_val (&tach_filt_rpm, -touch_accel, 0, tach_redline_rpm);  // (-= 25) Pressed the decrease engine rpm button
            else if (tcol==4 && trow==3 && sim_joy) adj_val (&ctrl_pos_adc[VERT][FILT], touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (+= 25) Pressed the joystick up button
            else if (tcol==4 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[VERT][FILT], -touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (-= 25) Pressed the joystick down button
            else if (tcol==5 && trow==0 && sim_syspower) {  // You need to enable syspower simulation, be in sim mode and then long-press the syspower button to toggle it. (Hard to do by accident)/
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout()) {
                    syspower = !syspower;
                    touch_longpress_valid = false;
                }
            }
            else if (tcol==5 && trow==1 && sim_speedo) adj_val (&carspeed_filt_mmph, touch_accel, 0, carspeed_redline_mmph);  // (+= 50) // Pressed the increase vehicle speed button
            else if (tcol==5 && trow==2 && sim_speedo) adj_val (&carspeed_filt_mmph, -touch_accel, 0, carspeed_redline_mmph);  // (-= 50) Pressed the decrease vehicle speed button
            else if (tcol==5 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[HORZ][FILT], touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (+= 25) Pressed the joystick right button                           
        }
        if (touch_accel_exponent < touch_accel_exponent_max && (touchHoldTimer.elapsed() > (touch_accel_exponent + 1) * touchAccelTimer.timeout())) touch_accel_exponent++; // If timer is > the shift time * exponent, and not already maxed, double the edit speed by incrementing the exponent
        touch_now_touched = true;
    }  // (if touchpanel reads a touch)
    else {  // If not being touched, put momentarily-set simulated button values back to default values
        if (simulating) cruise_sw = false;  // // Makes this button effectively momentary
        sim_edit_delta_touch = 0;  // Stop changing value
        touch_now_touched = false;  // remember last touch state
        touch_accel_exponent = 0;
        touchHoldTimer.reset();
        touch_longpress_valid = true;
    }

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tch");  //

    // Encoder handling
    //
    if (encoder_sw_action != NONE) {  // First deal with any unhandled switch press events
        if (encoder_sw_action == SHORT)  {  // if short press
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
            else ;  // Unless tuning, short press now does nothing. I envision it should switch desktops from our current analysis interface to a different runtime display 
        }
        else tuning_ctrl = (tuning_ctrl == OFF) ? SELECT : OFF;  // Long press starts/stops tuning
        encoder_sw_action = NONE;  // Our responsibility to reset this flag after handling events
    }
    if (encoder_delta != 0) {  // Now handle any new rotations
        if (encoder_spinrate_isr_us >= encoder_spinrate_min_us) {  // Attempt to reject clicks coming in too fast
            encoder_spinrate_old_us = encoder_spinrate_last_us;  // Store last few spin times for filtering purposes ...
            encoder_spinrate_last_us = encoder_spinrate_us;  // ...
            encoder_spinrate_us = constrain (encoder_spinrate_isr_us, encoder_spinrate_min_us, encoder_accel_thresh_us);
            int32_t spinrate_temp = (encoder_spinrate_old_us > encoder_spinrate_last_us) ? encoder_spinrate_old_us : encoder_spinrate_last_us;  // Find the slowest of the last 3 detents ...
            if (spinrate_temp < encoder_spinrate_us) spinrate_temp = encoder_spinrate_us;
            encoder_edits_per_det = map (spinrate_temp, encoder_spinrate_min_us, encoder_accel_thresh_us, encoder_accel_max, 1);  // if turning faster than 100ms/det, proportionally accelerate the effect of each detent by up to 50x 
            // encoderSpinspeedTimer.reset();
            if (tuning_ctrl == EDIT) sim_edit_delta_encoder = encoder_delta * encoder_edits_per_det;  // If a tunable value is being edited, turning the encoder changes the value
            else encoder_delta = constrain (encoder_delta, -1, 1);  // Only change one at a time when selecting or turning pages
            if (tuning_ctrl == SELECT) selected_value += encoder_delta;  // If overflow constrain will fix in general handler below
            else if (tuning_ctrl == OFF) dataset_page += encoder_delta;  // If overflow tconstrain will fix in general below
        }
        encoder_delta = 0;  // Our responsibility to reset this flag after handling events
    }
    
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "enc");  //

    // Tuning : implement effects of changes made by encoder or touchscreen to simulating, dataset_page, selected_value, or tuning_ctrl
    //
    sim_edit_delta = sim_edit_delta_encoder + sim_edit_delta_touch;  // Allow edits using the encoder or touchscreen
    sim_edit_delta_touch = 0;
    sim_edit_delta_encoder = 0;
    if (tuning_ctrl != tuning_ctrl_last || dataset_page != dataset_page_last || selected_value != selected_value_last || sim_edit_delta != 0) tuningCtrlTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
    if (tuning_ctrl != OFF && tuningCtrlTimer.expired()) tuning_ctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
    dataset_page = constrain (dataset_page, 0, arraysize(pagecard)-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
    if (dataset_page != dataset_page_last) {
        if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If page is flipped during edit, drop back to select mode
        disp_dataset_page_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
    }
    if (tuning_ctrl == SELECT) {
        if (dataset_page >= 4) selected_value = constrain (selected_value, 5, 7);  // Skip unchangeable values for all PID modes
        else if (dataset_page == JOY) selected_value = constrain (selected_value, 2, 7);  // Skip unchangeable values for joy mode
        else if (dataset_page == RUN) selected_value = constrain (selected_value, 3, 7);  // Skip unchangeable values for run mode
        else if (dataset_page == TEMP) selected_value = constrain (selected_value, 6, 7);  // Skip unchangeable values for temp mode
        else selected_value = constrain (selected_value, 0, arraysize (dataset_page_names[dataset_page])-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
        if (selected_value != selected_value_last) disp_selected_val_dirty = true;
    }
    if (tuning_ctrl != tuning_ctrl_last || disp_dataset_page_dirty) disp_selected_val_dirty = true;
    if (tuning_ctrl == EDIT && sim_edit_delta != 0) {  // Change tunable values when editing
        if (dataset_page == RUN)  switch (selected_value) {
            // case 2:  brakeSPID.set_proportionality((sim_edit_delta > 0) ? ERROR_TERM : SENSED_INPUT);  break;
            case 3:  adj_bool (&sim_brkpos, sim_edit_delta);  break;
            case 4:  adj_bool (&sim_joy, sim_edit_delta);  break;
            case 5:  adj_bool (&sim_pressure, sim_edit_delta);  break;
            case 6:  adj_bool (&sim_tach, sim_edit_delta);  break;
            case 7:  adj_bool (&sim_speedo, sim_edit_delta);  break;
        }
        else if (dataset_page == JOY)  switch (selected_value) {
            case 2:  adj_val (&ctrl_lims_adc[ctrl][HORZ][MIN], sim_edit_delta, 0, adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][DB] / 2 - 1);  break;
            case 3:  adj_val (&ctrl_lims_adc[ctrl][HORZ][MAX], sim_edit_delta, adcmidscale_adc + ctrl_lims_adc[ctrl][HORZ][DB] / 2 + 1, adcrange_adc);  break;
            case 4:  adj_val (&ctrl_lims_adc[ctrl][HORZ][DB], sim_edit_delta, 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]));  break;
            case 5:  adj_val (&ctrl_lims_adc[ctrl][VERT][MIN], sim_edit_delta, 0, adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][DB] / 2 - 1);  break;
            case 6:  adj_val (&ctrl_lims_adc[ctrl][VERT][MAX], sim_edit_delta, adcmidscale_adc + ctrl_lims_adc[ctrl][VERT][DB] / 2 + 1, adcrange_adc);  break;
            case 7:  adj_val (&ctrl_lims_adc[ctrl][VERT][DB], sim_edit_delta, 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]));  break;
        }
        else if (dataset_page == CAR)  switch (selected_value) {
            case 0:  adj_val (&gas_governor_percent, sim_edit_delta, 0, 100);  break;
            case 1:  adj_val (&tach_idle_rpm, sim_edit_delta, 0, tach_redline_rpm -1);  break;
            case 2:  adj_val (&tach_redline_rpm, sim_edit_delta, tach_idle_rpm, 8000);  break;
            case 3:  adj_val (&carspeed_idle_mmph, sim_edit_delta, 0, carspeed_redline_mmph - 1);  break;
            case 4:  adj_val (&carspeed_redline_mmph, sim_edit_delta, carspeed_idle_mmph, 30000);  break;
            case 5:  adj_bool (&ctrl, sim_edit_delta);  break;
            case 6:  if (runmode == CAL) adj_bool (&cal_joyvert_brkmotor, sim_edit_delta);  break;
            case 7:  if (runmode == CAL) adj_bool (&cal_pot_gasservo, (sim_edit_delta < 0 || cal_pot_gas_ready) ? sim_edit_delta : -1);  break;
        }
        else if (dataset_page == PWMS)  switch (selected_value) {
            case 0:  adj_val (&steer_pulse_left_us, sim_edit_delta, steer_pulse_stop_us + 1, steer_pulse_left_max_us);  break;
            case 1:  adj_val (&steer_pulse_stop_us, sim_edit_delta, steer_pulse_right_us + 1, steer_pulse_left_us - 1);  break;
            case 2:  adj_val (&steer_pulse_right_us, sim_edit_delta, steer_pulse_right_min_us, steer_pulse_stop_us - 1);  break;
            case 3:  adj_val (&brake_pulse_extend_us, sim_edit_delta, brake_pulse_stop_us + 1, brake_pulse_extend_max_us);  break;
            case 4:  adj_val (&brake_pulse_stop_us, sim_edit_delta, brake_pulse_retract_us + 1, brake_pulse_extend_us - 1);  break;
            case 5:  adj_val (&brake_pulse_retract_us, sim_edit_delta, brake_pulse_retract_min_us, brake_pulse_stop_us -1);  break;
            case 6:  adj_val (&gas_pulse_idle_us, sim_edit_delta, gas_pulse_redline_us + 1, gas_pulse_ccw_max_us - gas_pulse_park_slack_us);  break;
            case 7:  adj_val (&gas_pulse_redline_us, sim_edit_delta, gas_pulse_cw_min_us, gas_pulse_idle_us - 1);  break;
        }
        else if (dataset_page == BPID) {
            if (selected_value == 5) brakeSPID.set_tunings (brakeSPID.get_kp()+0.001*(double)sim_edit_delta, brakeSPID.get_ki_hz(), brakeSPID.get_kd_s());
            if (selected_value == 6) brakeSPID.set_tunings (brakeSPID.get_kp(), brakeSPID.get_ki_hz()+0.001*(double)sim_edit_delta, brakeSPID.get_kd_s());
            if (selected_value == 7) brakeSPID.set_tunings (brakeSPID.get_kp(), brakeSPID.get_ki_hz(), brakeSPID.get_kd_s()+0.001*(double)sim_edit_delta);
        }
        else if (dataset_page == GPID) {
            if (selected_value == 5) gasSPID.set_tunings (gasSPID.get_kp()+0.001*(double)sim_edit_delta, gasSPID.get_ki_hz(), gasSPID.get_kd_s());
            if (selected_value == 6) gasSPID.set_tunings (gasSPID.get_kp(), gasSPID.get_ki_hz()+0.001*(double)sim_edit_delta, gasSPID.get_kd_s());
            if (selected_value == 7) gasSPID.set_tunings (gasSPID.get_kp(), gasSPID.get_ki_hz(), gasSPID.get_kd_s()+0.001*(double)sim_edit_delta);
        }
        else if (dataset_page == CPID) {
            if (selected_value == 5) cruiseSPID.set_tunings (cruiseSPID.get_kp()+0.001*(double)sim_edit_delta, cruiseSPID.get_ki_hz(), cruiseSPID.get_kd_s());
            if (selected_value == 6) cruiseSPID.set_tunings (cruiseSPID.get_kp(), cruiseSPID.get_ki_hz()+0.001*(double)sim_edit_delta, cruiseSPID.get_kd_s());
            if (selected_value == 7) cruiseSPID.set_tunings (cruiseSPID.get_kp(), cruiseSPID.get_ki_hz(), cruiseSPID.get_kd_s()+0.001*(double)sim_edit_delta);
        }
        else if (dataset_page == TEMP) {
            if (selected_value == 6) gasSPID.set_open_loop (sim_edit_delta > 0);
            if (selected_value == 7) adj_val (&brake_pos_zeropoint_thou, sim_edit_delta, brake_pos_nom_lim_retract_thou, brake_pos_nom_lim_extend_thou);
        }
    }
    
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tun");  //

    // Update derived variables values in case they have changed
    ctrl_db_adc[VERT][BOT] = (adcrange_adc-ctrl_lims_adc[ctrl][VERT][DB])/2;  // Lower threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[VERT][TOP] = (adcrange_adc+ctrl_lims_adc[ctrl][VERT][DB])/2;  // Upper threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][BOT] = (adcrange_adc-ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Lower threshold of horz joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][TOP] = (adcrange_adc+ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Upper threshold of horz joy deadband (ADC count 0-4095)
    tach_govern_rpm = map ((double)gas_governor_percent, 0.0, 100.0, 0.0, tach_redline_rpm);  // Create an artificially reduced maximum for the engine speed
    gas_pulse_govern_us = map ((int32_t)(gas_governor_percent*(tach_redline_rpm-tach_idle_rpm)/tach_redline_rpm), 0, 100, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
    carspeed_govern_mmph = map ((double)gas_governor_percent, 0.0, 100.0, 0.0, carspeed_redline_mmph);  // Governor must scale the top vehicle speed proportionally

    // Ignition & Panic stop logic and Update output signals
    if (panic_stop && !carspeed_filt_mmph) panic_stop = false;  //  Panic is over cuz car is stopped
    else if (ctrl == HOTRC && hotrc_radio_detected_last && !hotrc_radio_detected) panic_stop = true;  
    if (panic_stop) ignition = LOW;  // Kill car if panicking
    if ((ignition != ignition_last) && (!ignition || !panic_stop)) write_pin (ignition_pin, ignition);  // Turn car off or on, with some conditions
    if (syspower != syspower_last) syspower_set (syspower);
    hotrc_radio_detected_last = hotrc_radio_detected;
    syspower_last = syspower;
    ignition_last = ignition; // Make sure this goes after the last comparison
    
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "ext");  //

    // Heartbeat led algorithm
    if (neopixel_heartbeat) { // Make the neopixel into a beating heart, in the color of the current runmode 
        neopixel_heart_color[N_RED] = ((colorcard[runmode] & 0xf800) >> 11) << 3;
        neopixel_heart_color[N_GRN] = ((colorcard[runmode] & 0x7e0) >> 5) << 2;
        neopixel_heart_color[N_BLU] = (colorcard[runmode] & 0x1f) << 3;
        neostrip.setPixelColor (0, neostrip.Color (neopixel_heart_color[N_BLU], neopixel_heart_color[N_RED], neopixel_heart_color[N_GRN]));
        if (heartbeatTimer.expired()) {
            heartbeat_pulse = !heartbeat_pulse;
            if (heartbeat_pulse) neopixel_heart_fade = neopixel_brightness;
            else neopixelTimer.reset();
            if (++heartbeat_state >= arraysize (heartbeat_ekg)) heartbeat_state -= arraysize (heartbeat_ekg);
            heartbeatTimer.set (heartbeat_ekg[heartbeat_state]);
        }
        else if (!heartbeat_pulse && neopixel_heart_fade) {
            neopixel_heart_fade = (int8_t)((double)neopixel_brightness * (1 - (double)neopixelTimer.elapsed()/(double)neopixel_timeout));
            if (neopixel_heart_fade < 1) neopixel_heart_fade = 0;
        }
        if (neopixel_pin >= 0) {
            neostrip.setBrightness (neopixel_heart_fade);
            neostrip.show();
        }
    }
    else if (neopixelTimer.expired()) {  // Just make a heartbeat on the native board led
        heartbeat_pulse = !heartbeat_pulse;
        if (++heartbeat_state >= arraysize (heartbeat_ekg)) heartbeat_state -= arraysize (heartbeat_ekg);
        heartbeatTimer.set (heartbeat_ekg[heartbeat_state]);
        write_pin (heartbeat_led_pin, heartbeat_pulse);
        neostrip.setPixelColor (0, colorwheel (++neopixel_wheel_counter));
        neopixelTimer.reset();
    }
    write_pin (led_rx_pin, (sim_edit_delta <= 0));  // use these Due lights for whatever, here debugging the touchscreen
    
    // if (!neopixel_heartbeat && neopixelTimer.expired()) {  // Rainbow fade
    //     neopixel_wheel_counter++;
    //     neostrip.setPixelColor(0, colorwheel(neopixel_wheel_counter));
    //     neopixelTimer.reset();
    // }

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "hrt");
    
    // Display updates
    //
    if (display_enabled) {
        if (simulating != simulating_last) draw_simbuttons (simulating);  // if we just entered simulator draw the simulator buttons, or if we just left erase them
        if (disp_dataset_page_dirty || disp_redraw_all) draw_dataset_page (dataset_page, dataset_page_last);
        if (disp_selected_val_dirty || disp_redraw_all) draw_selected_name (tuning_ctrl, tuning_ctrl_last, selected_value, selected_value_last);
        if (disp_sidemenu_dirty || disp_redraw_all) draw_touchgrid (true);
        if (disp_runmode_dirty || runmode != oldmode || disp_redraw_all) draw_runmode (runmode, oldmode, (runmode == SHUTDOWN) ? shutdown_color : -1);
        disp_dataset_page_dirty = false;
        disp_selected_val_dirty = false;
        disp_sidemenu_dirty = false;
        disp_runmode_dirty = false;
        if (dispRefreshTimer.expired() || sim_edit_delta != 0 && !wait_one_loop) {
            wait_one_loop = true;
            dispRefreshTimer.reset();
            int32_t range; double drange;
            draw_dynamic(1, carspeed_filt_mmph, 0.0, carspeed_redline_mmph, cruiseSPID.get_target());
            draw_dynamic(2, tach_filt_rpm, 0.0, tach_redline_rpm, gasSPID.get_target());
            draw_dynamic(3, pressure_filt_psi, pressure_min_psi, pressure_max_psi, brakeSPID.get_target());  // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.get_target() : pressure_target_adc);
            draw_dynamic(4, ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
            draw_dynamic(5, ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
            draw_dynamic(6, cruiseSPID.get_target(), 0.0, carspeed_govern_mmph);
            draw_dynamic(7, brakeSPID.get_target(), pressure_min_psi, pressure_max_psi);
            draw_dynamic(8, gasSPID.get_target(), 0.0, tach_redline_rpm);
            draw_dynamic(9, brake_pulse_out_us, brake_pulse_retract_us, brake_pulse_extend_us);
            draw_dynamic(10, gas_pulse_out_us, gas_pulse_redline_us, gas_pulse_idle_us);
            draw_dynamic(11, steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);
            if (dataset_page == RUN) {
                draw_dynamic(12, battery_filt_mv, 0.0, battery_max_mv);
                draw_dynamic(13, brake_pos_filt_thou, brake_pos_nom_lim_retract_thou, brake_pos_nom_lim_extend_thou);
                draw_dynamic(14, pot_filt_percent, pot_min_percent, pot_max_percent);
                // draw_dynamic(14, brakeSPID.get_proportionality(), -1, -1);
                draw_dynamic(15, sim_brkpos, -1, -1);
                draw_dynamic(16, sim_joy, -1, -1);
                draw_dynamic(17, sim_pressure, -1, -1);
                draw_dynamic(18, sim_tach, -1, -1);
                draw_dynamic(19, sim_speedo, -1, -1);
            }
            else if (dataset_page == JOY) {
                draw_dynamic(12, ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
                draw_dynamic(13, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
                draw_dynamic(14, ctrl_lims_adc[ctrl][HORZ][MIN], 0, (adcrange_adc-ctrl_lims_adc[ctrl][HORZ][MAX])/2);
                draw_dynamic(15, ctrl_lims_adc[ctrl][HORZ][MAX], (ctrl_lims_adc[ctrl][HORZ][MIN]-adcrange_adc)/2, adcrange_adc);
                draw_dynamic(16, ctrl_lims_adc[ctrl][HORZ][DB], 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]));
                draw_dynamic(17, ctrl_lims_adc[ctrl][VERT][MIN], 0, (adcrange_adc-ctrl_lims_adc[ctrl][VERT][MAX])/2);
                draw_dynamic(18, ctrl_lims_adc[ctrl][VERT][MAX], (ctrl_lims_adc[ctrl][VERT][MIN]-adcrange_adc)/2, adcrange_adc);
                draw_dynamic(19, ctrl_lims_adc[ctrl][VERT][DB], 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]));
            }
            else if (dataset_page == CAR) {
                draw_dynamic(12, gas_governor_percent, 0, 100);
                draw_dynamic(13, tach_idle_rpm, 0.0, tach_redline_rpm);
                draw_dynamic(14, tach_redline_rpm, 0.0, tach_max_rpm);
                draw_dynamic(15, carspeed_idle_mmph, 0.0, carspeed_redline_mmph);
                draw_dynamic(16, carspeed_redline_mmph, 0.0, carspeed_max_mmph);
                draw_dynamic(17, ctrl, -1, -1);  // 0 if hotrc
                draw_dynamic(18, cal_joyvert_brkmotor, -1, -1);
                draw_dynamic(19, cal_pot_gasservo, -1, -1);
            }
            else if (dataset_page == PWMS) {
                draw_dynamic(12, steer_pulse_left_us, steer_pulse_stop_us, steer_pulse_left_max_us);
                draw_dynamic(13, steer_pulse_stop_us, steer_pulse_left_us, steer_pulse_right_us);
                draw_dynamic(14, steer_pulse_right_us, steer_pulse_right_min_us, steer_pulse_stop_us);
                draw_dynamic(15, brake_pulse_extend_us, brake_pulse_stop_us, brake_pulse_extend_max_us);
                draw_dynamic(16, brake_pulse_stop_us, brake_pulse_retract_us, brake_pulse_extend_us);
                draw_dynamic(17, brake_pulse_retract_us, brake_pulse_retract_min_us, brake_pulse_stop_us);
                draw_dynamic(18, gas_pulse_idle_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
                draw_dynamic(19, gas_pulse_redline_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
            }
            else if (dataset_page == BPID) {
                drange = pressure_max_psi-pressure_min_psi;
                draw_dynamic(12, (brakeSPID.get_error()), -drange, drange);
                draw_dynamic(13, (brakeSPID.get_p_term()), -drange, drange);
                draw_dynamic(14, (brakeSPID.get_i_term()), -drange, drange);
                draw_dynamic(15, (brakeSPID.get_d_term()), -drange, drange);
                draw_dynamic(16, (brakeSPID.get_output()), -drange, drange);  // brake_spid_carspeed_delta_adc, -range, range);
                draw_dynamic(17, brakeSPID.get_kp(), 0.0, 1.0);
                draw_dynamic(18, brakeSPID.get_ki_hz(), 0.0, 1.0);
                draw_dynamic(19, brakeSPID.get_kd_s(), 0.0, 1.0);
            }
            else if (dataset_page == GPID) {
                drange = tach_govern_rpm-tach_idle_rpm;
                draw_dynamic(12, (gasSPID.get_error()), -drange, drange);
                draw_dynamic(13, (gasSPID.get_p_term()), -drange, drange);
                draw_dynamic(14, (gasSPID.get_i_term()), -drange, drange);
                draw_dynamic(15, (gasSPID.get_d_term()), -drange, drange);
                draw_dynamic(16, (gasSPID.get_output()), -drange, drange);  // gas_spid_carspeed_delta_adc, -drange, drange);
                draw_dynamic(17, gasSPID.get_kp(), 0.0, 1.0);
                draw_dynamic(18, gasSPID.get_ki_hz(), 0.0, 1.0);
                draw_dynamic(19, gasSPID.get_kd_s(), 0.0, 1.0);
            }
            else if (dataset_page == CPID) {
                drange = carspeed_govern_mmph-carspeed_idle_mmph;
                draw_dynamic(12, (cruiseSPID.get_error()), -drange, drange);
                draw_dynamic(13, (cruiseSPID.get_p_term()), -drange, drange);
                draw_dynamic(14, (cruiseSPID.get_i_term()), -drange, drange);
                draw_dynamic(15, (cruiseSPID.get_d_term()), -drange, drange);
                draw_dynamic(16, (cruiseSPID.get_output()), -drange, drange);  // cruise_spid_carspeed_delta_adc, -drange, drange);
                draw_dynamic(17, cruiseSPID.get_kp(), 0.0, 1.0);
                draw_dynamic(18, cruiseSPID.get_ki_hz(), 0.0, 1.0);
                draw_dynamic(19, cruiseSPID.get_kd_s(), 0.0, 1.0);
            }
            else if (dataset_page == TEMP) {
                draw_dynamic(12, (temps[AMBIENT]), temp_min, temp_max);
                draw_dynamic(13, (temps[ENGINE]), temp_min, temp_max);
                draw_dynamic(14, (temps[WHEEL_FL]), temp_min, temp_max);
                draw_dynamic(15, (temps[WHEEL_FR]), temp_min, temp_max);
                draw_dynamic(16, (temps[WHEEL_RL]), temp_min, temp_max);
                draw_dynamic(17, (temps[WHEEL_RR]), temp_min, temp_max);
                draw_dynamic(18, gasSPID.get_open_loop(), -1, -1);
                draw_dynamic(19, brake_pos_zeropoint_thou, brake_pos_nom_lim_retract_thou, brake_pos_nom_lim_extend_thou);   
            }
            draw_bool ((runmode == CAL), 2);
            draw_bool (basicmodesw, 3);
            draw_bool (ignition, 4);
            draw_bool (syspower, 5);
        }
    }

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "dis");

    // Do the control loop bookkeeping at the end of each loop
    //
    
    wait_one_loop_last = wait_one_loop;
    wait_one_loop = false;
    simulating_last = simulating;
    tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
    dataset_page_last = dataset_page;
    selected_value_last = selected_value;
    disp_redraw_all = false;
    button_last = button_it;
    if (runmode != SHUTDOWN) shutdown_complete = false;
    if (runmode != oldmode) we_just_switched_modes = true;  // If changing runmode, set this so new mode logic can perform initial actions
    else we_just_switched_modes = false; // Reset this variable
    oldmode = runmode;  // remember what mode we're in for next time
    loop_period_us = loopTimer.elapsed();  // us since beginning of this loop
    if (!loop_period_us) loop_period_us++;  // ensure loop period is never zero since it gets divided by
    loop_freq_hz = 1000000.0/(double)loop_period_us;
    loopno++;  // I like to count how many loops
    tft_watchdog();
    if (timestamp_loop) {
        loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "end");  //
        printf ("\rRM:%ld Lp#%ld us:%5ld ", runmode, loopno, loop_period_us);
        // for (int32_t x=1; x<loopindex; x++) std::cout << ", " << std::setw(3) << loop_names[x] << x << ": " << std::setw(4) << looptimes_us[x]-looptimes_us[x-1];
        if (loop_period_us > 25000) printf ("\n");
    }
    int_counter = 0;
    loopTimer.reset();
}