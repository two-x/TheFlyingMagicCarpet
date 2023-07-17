// Carpet CANTroller II  Source Code  - For Arduino Due with Adafruit 2.8inch Captouch TFT shield.
#include <SPI.h>  // SPI serial bus needed to talk to the LCD and the SD card
#include <Wire.h>  // Contains I2C serial bus, needed to talk to touchscreen chip
#include "Arduino.h"
#include <Adafruit_SleepyDog.h>  // Watchdog
#include <vector>
#include <string>
#include <iomanip>  // Formatting cout
#include "classes.h"  // Contains our data structures
// #include "spid.h"
// #include <QuickPID.h>
#include "qpid.h"  // This is quickpid library except i have to edit some of it
#include "globals.h"
#include "uictrl.h"
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

Hotrc hotrc (&hotrc_vert_pulse_filt_us, hotrc_pulse_failsafe_min_us, hotrc_pulse_failsafe_max_us, hotrc_pulse_failsafe_pad_us);
    
Display screen(tft_cs_pin, tft_dc_pin);
    
// Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
MAKE_ENCODER(encoder, encoder_a_pin, encoder_b_pin, encoder_sw_pin);

void setup() {  // Setup just configures pins (and detects touchscreen type)
    set_pin (encoder_a_pin, INPUT_PULLUP);
    set_pin (encoder_b_pin, INPUT_PULLUP);
    set_pin (encoder_sw_pin, INPUT_PULLUP);  // The esp32 pullup is too weak. Use resistor
    set_pin (brake_pwm_pin, OUTPUT);
    set_pin (steer_pwm_pin, OUTPUT);
    set_pin (tft_dc_pin, OUTPUT);
    set_pin (gas_pwm_pin, OUTPUT);
    // set_pin (ignition_pin, OUTPUT);  // drives relay to turn on/off car. Active high
    set_pin (basicmodesw_pin, INPUT_PULLUP);
    set_pin (tach_pulse_pin, INPUT_PULLUP);
    set_pin (speedo_pulse_pin, INPUT_PULLUP);
    set_pin (pressure_pin, INPUT);
    set_pin (brake_pos_pin, INPUT);
    set_pin (ign_batt_pin, INPUT);
    set_pin (hotrc_ch1_horz_pin, INPUT);
    set_pin (hotrc_ch2_vert_pin, INPUT);
    set_pin (neopixel_pin, OUTPUT);
    set_pin (sdcard_cs_pin, OUTPUT);
    set_pin (tft_cs_pin, OUTPUT);
    set_pin (pot_wipe_pin, INPUT);
    set_pin (button_pin, INPUT_PULLUP);
    set_pin (starter_pin, INPUT_PULLDOWN);
    set_pin (hotrc_ch3_ign_pin, INPUT);
    set_pin (hotrc_ch4_cruise_pin, INPUT);

    set_pin (joy_horz_pin, INPUT);
    set_pin (joy_vert_pin, INPUT);
    
    set_pin (touch_irq_pin, INPUT_PULLUP);
    set_pin (tft_rst_pin, OUTPUT);

    write_pin (tft_cs_pin, HIGH);   // Prevent bus contention
    write_pin (sdcard_cs_pin, HIGH);   // Prevent bus contention
    write_pin (tft_dc_pin, LOW);
    write_pin (tft_rst_pin, HIGH);

    // This bit is here as a way of autdetecting the controller type. It starts HEADLESS. If HIGH is read here then JOY,
    // otherwise the pulldown R in the divider will enable HOTRC once radio is detected.
    set_pin (ign_batt_pin, INPUT);  
    // Temporarily use this pin to read a pullup/down resistor to detect controller type
    // ctrl = (read_pin (ignition_pin) ? JOY : HOTRC);  // HEADLESS
    // if (ctrl == HEADLESS) set_pin (ignition_pin, OUTPUT);  // Then set the put as an output as normal.
    // if (ctrl == HOTRC) { set_pin (ignition_pin, OUTPUT);  // Then set the put as an output as normal.
    // write_pin (ignition_pin, LOW); } // Initialize to ignition off
    set_pin (ign_out_pin, OUTPUT);
    write_pin (ign_out_pin, LOW);
    
    // This bit is here as a way of autdetecting soren's breadboard, since his LCD is wired upside-down.
    // Soren put a strong external pulldown on the pin, so it'll read low for autodetection. 
    
    // Disabling automatic screen flip detection due to it makes Anders board crash
    // set_pin (syspower_pin, INPUT);  // Temporarily use this pin to read a pullup/down resistor to configure screen flip
    // flip_the_screen = !(read_pin (syspower_pin));  // Will cause the LCD to be upside down
    
    set_pin (syspower_pin, OUTPUT);  // Then set the put as an output as normal.
    write_pin (syspower_pin, syspower);

    analogReadResolution (adcbits);  // Set Arduino Due to 12-bit resolution (default is same as Mega=10bit)
    Serial.begin (115200);  // Open serial port
    delay (800);  //  // This is needed to allow the uart to initialize and the screen board enough time after a cold boot
    // printf("Serial port open\n");  // This works on Due but not ESP32
    
    for (int32_t x=0; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
    
    if (display_enabled) {
        config.begin("FlyByWire", false);
        dataset_page = config.getUInt("dpage", PG_RUN);
        dataset_page_last = config.getUInt("dpage", PG_TEMP);
        screen.init();
    }
    neostrip.begin();  // start datastream
    neostrip.show();  // Turn off the pixel
    neostrip.setBrightness (neo_brightness_max);  // It truly is incredibly bright
    // 230417 removing sdcard init b/c boot is hanging here unless I insert this one precious SD card
    // Serial.print(F("Initializing filesystem...  "));  // SD card is pretty straightforward, a single call. 
    // if (! sd.begin(usd_cs_pin, SD_SCK_MHZ(25))) {   // ESP32 requires 25 mhz limit
    //     Serial.println(F("SD begin() failed"));
    //     for(;;); // Fatal error, do not continue
    // }
    // sd_init();
    // Serial.println(F("Filesystem started"));

    SETUP_ENCODER(encoder);

    // Configure MCPWM GPIOs
    //
    // The four channel inputs can be connected to any available GPIO pins on the ESP32, but the specific MCPWM unit and
    // output channels are used to configure the MCPWM GPIOs and input capture. Here's the mapping used in the code:
    // INPUT_PIN_1 is connected to MCPWM0A (MCPWM unit 0, output channel A).
    // INPUT_PIN_2 is connected to MCPWM1A (MCPWM unit 0, output channel B).
    // INPUT_PIN_3 is connected to MCPWM2A (MCPWM unit 0, output channel C).
    // INPUT_PIN_4 is connected to MCPWM3A (MCPWM unit 0, output channel D).
    // You can change these mappings according to your specific pin assignments. Ensure that the MCPWM GPIOs you select are compatible with input capture functionality.
    // Attempt to use MCPWM input capture pulse width timer unit to get precise hotrc readings
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/mcpwm.html#capture
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/kconfig.html#mcpwm-configuration
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/kconfig.html#mcpwm-configuration
    // // Configure MCPWM GPIOs
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, hotrc_ch1_horz_pin);
    // mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, hotrc_ch2_vert_pin);
    // // Configure MCPWM units 0 and 1
    // mcpwm_config_t pwm_config;
    // pwm_config.frequency = 0;  // Set frequency to 0 for input mode
    // pwm_config.cmpr_a = 0;  // Set duty cycle to 0 for input mode
    // pwm_config.counter_mode = MCPWM_UP_COUNTER;
    // pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    // mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    // // mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
    // // mcpwm_capture_enable(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
    // // mcpwm_capture_set_cb(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, hotrc_ch1_isr, NULL);
    // // mcpwm_capture_set_cb(MCPWM_UNIT_1, MCPWM_SELECT_CAP0, hotrc_ch2_isr, NULL);
    // Ch3 and Ch4 interrupts work with slower timers
    // attachInterrupt (digitalPinToInterrupt(hotrc_ch3_ign_pin), hotrc_ch3_isr, CHANGE);
    // attachInterrupt (digitalPinToInterrupt(hotrc_ch4_cruise_pin), hotrc_ch4_isr, FALLING);

    // Set up our interrupts
    attachInterrupt (digitalPinToInterrupt(tach_pulse_pin), tach_isr, RISING);
    attachInterrupt (digitalPinToInterrupt(speedo_pulse_pin), speedo_isr, RISING);
    
    printf ("ctrl=%ld\n");
    if (ctrl == HOTRC) {
        
        // hotrc_vert_timer = timerBegin(1, 80, true); // Use timer 0 for the pulse measurement
        // timerAttachInterrupt(hotrc_vert_timer, &hotrc_vert_isr, true);
        // timerAlarmWrite(hotrc_vert_timer, 0xFFFFFFFF, true); // Set a large initial alarm value
        // timerRestart(hotrc_vert_timer);
        // timerInputCaptureSingle(hotrc_vert_timer, hotrc_ch2_vert_pin, FALLING); // Capture on falling edge
        
        // ledcAttachPin(hotrc_ch2_vert_pin, 1); // Use LEDC channel 0 for input capture
        // ledcSetup(1, 1, 1); // Use LEDC channel 0 with a resolution of 1 bit
        // ledcAttachPinTone(hotrc_ch2_vert_pin, 1); // Capture on falling edge

        attachInterrupt (digitalPinToInterrupt (hotrc_ch2_vert_pin), hotrc_vert_isr, FALLING);
        attachInterrupt (digitalPinToInterrupt (hotrc_ch1_horz_pin), hotrc_horz_isr, FALLING);
        attachInterrupt (digitalPinToInterrupt (hotrc_ch3_ign_pin), hotrc_ch3_isr, FALLING);
        attachInterrupt (digitalPinToInterrupt (hotrc_ch4_cruise_pin), hotrc_ch4_isr, CHANGE);
    }
    Serial.println (F("Interrupts set up and enabled\n"));

    calc_deadbands();
    calc_governor();

    steer_servo.attach (steer_pwm_pin);
    brake_servo.attach (brake_pwm_pin);
    gas_servo.attach (gas_pwm_pin);

    neo_heartbeat = (neopixel_pin >= 0);
    neostrip.begin();
    neostrip.show(); // Initialize all pixels to 'off'
    neostrip.setBrightness (neo_brightness_max);

    // Peef setup
    // onewire.reset();
    // onewire.write(0xCC);        // All Devices present - Skip ROM ID
    // onewire.write(0x44);        // start conversion, with parasite power on at the end

    tempsensebus.setWaitForConversion (false);  // Whether to block during conversion process
    tempsensebus.setCheckForConversion (true);  // Do not listen to device for conversion result, instead we will wait the worst-case period
    tempsensebus.begin();
    temp_detected_device_ct = tempsensebus.getDeviceCount();
    printf ("Temp sensors: Detected %d devices.\nParasitic power is: ", temp_detected_device_ct);  // , DEC);
    printf ((tempsensebus.isParasitePowerMode()) ? "On\n" : "Off\n");
    // for (int32_t x = 0; x < arraysize(temp_addrs); x++) {
    for (int32_t index = 0; index < temp_detected_device_ct; index++) {
        if (tempsensebus.getAddress (temp_temp_addr, index)) {
            for (int8_t addrbyte = 0; addrbyte < arraysize(temp_temp_addr); addrbyte++) {
                temp_addrs[index][addrbyte] = temp_temp_addr[addrbyte];
            }
            tempsensebus.setResolution (temp_temp_addr, temperature_precision);  // temp_addrs[x]
            printf ("Found sensor device: index %d, addr 0x%x\n", index, temp_temp_addr);  // temp_addrs[x]
        }
        else printf ("Found ghost device : index %d, addr unknown\n", index);  // printAddress (temp_addrs[x]);
    }
    
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
    
    // pressure.set_convert ( 1000.0 * 3.3 / (adcrange_adc * (4.5 - 0.55)), 0.0, false);
    // pressure.set_names ("pressure", "adc ", "psi ");
    // pressure.set_limits (129.0, 452.0);
    
    int32_t watchdog_time_ms = Watchdog.enable(2500);  // Start 2.5 sec watchdog
    printf ("Watchdog enabled. Timer set to %ld ms.\n", watchdog_time_ms);
    hotrcPanicTimer.reset();
    loopTimer.reset();  // start timer to measure the first loop
    booted = true;
    Serial.println (F("Setup done"));
}

// void init (void) {     
// }

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
    loopindex = 0;  // reset at top of loop
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "top");
    // cout << "(top)) spd:" << speedo_filt_mph << " tach:" << tach_filt_rpm;

    // if (!booted) init();  // Initialize - If this is our first loop, initialize everything

    // Update inputs.  Fresh sensor data, and filtering.
    //

    // ESP32 "boot" button.  This is not working (?)
    button_last = button_it;
    if (!read_pin (button_pin)) {
        if (!button_it) {  // If press just occurred
            dispResetButtonTimer.reset();  // Looks like someone just pushed the esp32 "boot" button
            btn_press_timer_active = true;  // flag to indicate timing for a possible long press
        }
        else if (btn_press_timer_active && dispResetButtonTimer.expired()) {
            btn_press_action = LONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
            btn_press_timer_active = false;  // Clear timer active flag
            btn_press_suppress_click = true;  // Prevents the switch release after a long press from causing a short press
        }
        button_it = true;  // Store press is in effect
    }
    else {  // if button is not being pressed
        btn_press_action = NONE;  // Any button action handling needs to happen in the same loop or is lost
        if (button_it && !btn_press_suppress_click) btn_press_action = SHORT;  // if the button was just released, a short press occurred, which must be handled
        btn_press_timer_active = false;  // Clear timer active flag
        button_it = false;  // Store press is not in effect
        btn_press_suppress_click = false;  // End click suppression
    }
    // if (btn_press_action != NONE) 
    // printf ("it:%d ac:%ld lst:%d ta:%d sc:%d el:%ld\n", button_it, btn_press_action, button_last, btn_press_timer_active, btn_press_suppress_click, dispResetButtonTimer.elapsed());
    
    // External digital signals - takes 11 us to read
    if (!simulating || !sim_basicsw) basicmodesw = !digitalRead (basicmodesw_pin);   // 1-value because electrical signal is active low
    // if (ctrl == JOY && (!simulating || !sim_cruisesw)) cruise_sw = digitalRead (joy_cruise_btn_pin);

    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pre");

    // Temperature sensors
    // if (take_temperatures) {  // && tempTimer.expired()) {
    //     long tempread = temp_peef();
    //     if (tempread != 10000) temps[0] = (float)tempread;
    // }
    
    temp_soren();
    
    // float temps[temp_detected_device_ct];
    // uint32_t timecheck;
    // if (take_temperatures && tempTimer.expired()) {
    //     cout << endl << "loop# " << loopno << " stat0:" << temp_status;
    //     if (temp_status == IDLE) {
    //         wait_one_loop = true;
    //         if (++temp_current_index >= 2) temp_current_index -= 2;  // replace 1 with arraysize(temps)
    //         timecheck = micros();
    //         // tempsensebus.requestTemperaturesByIndex (temp_current_index);
    //         tempsensebus.setWaitForConversion (false);  // Do not block during conversion process
    //         tempsensebus.requestTemperatures();
    //         tempsensebus.setWaitForConversion (true);  // Do not listen to device for conversion result, instead we will wait the worst-case period
    //         cout << " my0:" << micros()-timecheck;
    //         //tempTimer.set (tempsensebus.millisToWaitForConversion (temperature_precision)*1000);
    //         tempTimer.set (800000);         
    //         temp_status = CONVERT;
    //     }
    //     else if (temp_status == CONVERT) {
    //         wait_one_loop = true;
    //         timecheck = micros();
    //         temps[temp_current_index] = tempsensebus.getTempFByIndex(temp_current_index);
    //         cout << " my1:" << micros()-timecheck;
    //         tempTimer.set(1500000);
    //         temp_status = DELAY;
    //     }
    //     else if (temp_status == DELAY) {
    //         //printf ("\n loop:%d temps[%ld] = %lf F\n", loopno, temp_current_index, temps[temp_current_index]);
    //         tempTimer.set(60000);
    //         if (++temp_current_index >= temp_detected_device_ct) temp_current_index -= temp_detected_device_ct;
    //         temp_status = IDLE;
    //     }
    //     cout << " stat1:" << temp_status << " id:"  << temp_current_index << " tmp:" << ((temp_status == IDLE) ? temps[temp_current_index] : -1) << endl;
    // }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "pst");

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
    if (!simulating || !sim_brkpos) {
        brake_pos_in = convert_units ((float)analogRead (brake_pos_pin), brake_pos_convert_in_per_adc, brake_pos_convert_invert);
        ema_filt (brake_pos_in, &brake_pos_filt_in, brake_pos_ema_alpha);
    }
    else brake_pos_filt_in = (brake_pos_nom_lim_retract_in + brake_pos_zeropoint_in)/2;  // To keep brake position in legal range during simulation
    
    if (!simulating || !sim_starter) starter = read_pin (starter_pin);

    // Tach - takes 22 us to read when no activity
    if (simulating && sim_tach && pot_overload == tach) tach_filt_rpm = map (pot_filt_percent, 0.0, 100.0, 0.0, tach_govern_rpm);
    else if (!simulating || !sim_tach) {
        tach_buf_us = (int32_t)tach_us;  // Copy delta value (in case another interrupt happens during handling)
        tach_us = 0;  // Indicates to isr we processed this value
        if (tach_buf_us) {  // If a valid rotation has happened since last time, delta will have a value
            tach_rpm = convert_units ((float)(tach_buf_us), tach_convert_rpm_per_rpus, tach_convert_invert);
            ema_filt (tach_rpm, &tach_filt_rpm, tach_ema_alpha);  // Sensor EMA filter
        }
        if (tach_rpm < tach_stop_thresh_rpm || (int32_t)(tach_timer_read_us - tach_timer_start_us) >= tach_stop_timeout_us) {  // If time between pulses is long enough an engine can't run that slow
            tach_rpm = 0.0;  // If timeout since last magnet is exceeded
            tach_filt_rpm = 0.0;
        }        
    }
    // Speedo - takes 14 us to read when no activity
    if (simulating && sim_speedo && pot_overload == speedo) speedo_filt_mph = map (pot_filt_percent, 0.0, 100.0, 0.0, speedo_govern_mph);
    else if (!simulating || !sim_speedo) { 
        speedo_buf_us = (int32_t)speedo_us;  // Copy delta value (in case another interrupt happens during handling)
        speedo_us = 0;  // Indicates to isr we processed this value
        if (speedo_buf_us) {  // If a valid rotation has happened since last time, delta will have a value
            speedo_mph = convert_units ((float)(speedo_buf_us), speedo_convert_mph_per_rpus, speedo_convert_invert);  // Update car speed value  
            ema_filt (speedo_mph, &speedo_filt_mph, speedo_ema_alpha);  // Sensor EMA filter
        }
        if (speedo_mph < speedo_stop_thresh_mph || (int32_t)(speedo_timer_read_us - speedo_timer_start_us) >= speedo_stop_timeout_us) {  // If time between pulses is long enough, consider the car is stopped
            speedo_mph = 0.0;
            speedo_filt_mph = 0.0;
        }
    }
    // Brake pressure - takes 72 us to read
    if (simulating && sim_pressure && pot_overload == pressure) pressure_filt_psi = map (pot_filt_percent, 0.0, 100.0, pressure_min_psi, pressure_max_psi);
    else if (!simulating && !sim_pressure) {
        pressure_adc = analogRead (pressure_pin);
        pressure_psi = convert_units ((float)pressure_adc, pressure_convert_psi_per_adc, pressure_convert_invert, (float)pressure_min_adc);
        ema_filt (pressure_psi, &pressure_filt_psi, pressure_ema_alpha);  // Sensor EMA filter
        // pressure.set_raw (analogRead (pressure_pin));
        // ema_filt (pressure.get_val(), &pressure_filt_psi, pressure_ema_alpha);  // Sensor EMA filter
    }
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "inp");  //

    // Controller handling
    //
    // Read horz and vert inputs, determine steering pwm output -  - takes 40 us to read. Then, takes 13 us to handle
    // std::cout << "164:" << hotrc_horz_pulse_us << " 264:" << hotrc_vert_pulse_us;
    if (ctrl != JOY) {
        hotrc_horz_pulse_us = (int32_t)hotrc_horz_pulse_64_us;
        hotrc_vert_pulse_us = (int32_t)hotrc_vert_pulse_64_us;  // replace with update history array
    }
    if (!simulating || !sim_joy) {  // Handle HotRC button generated events and detect potential loss of radio signal - takes 15 us to handle
        if (ctrl != JOY) {
            // printf (" 1R:%4ld 2R:%4ld 1A:%4ld 2A:%4ld", hotrc_horz_pulse_us, hotrc_vert_pulse_us, hotrc_horz_pulse_filt_us, hotrc_vert_pulse_filt_us);
            
            ema_filt (hotrc_horz_pulse_us, &hotrc_horz_pulse_filt_us, ctrl_ema_alpha[ctrl]);  // Used to detect loss of radio
            
            ema_filt (hotrc_vert_pulse_us, &hotrc_vert_pulse_filt_us, ctrl_ema_alpha[ctrl]);  // Used to detect loss of radio
            // printf (" 1B:%4ld 2B:%4ld", hotrc_horz_pulse_filt_us, hotrc_vert_pulse_filt_us);
        }
        if (ctrl == HOTRC) {
            // hotrc_horz_pulse_filt_us = constrain (hotrc_horz_pulse_filt_us, hotrc_pulse_lims_us[HORZ][MIN], hotrc_pulse_lims_us[HORZ][MAX]);
            // hotrc_vert_pulse_filt_us = constrain (hotrc_vert_pulse_filt_us, hotrc_pulse_lims_us[VERT][MIN], hotrc_pulse_lims_us[VERT][MAX]);
            // // ctrl_pos_adc[HORZ][FILT] = map (hotrc_horz_pulse_filt_us, hotrc_pulse_lims_us[HORZ][MIN], hotrc_pulse_lims_us[HORZ][MAX], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
            // // ctrl_pos_adc[VERT][FILT] = map (hotrc_vert_pulse_filt_us, hotrc_pulse_lims_us[VERT][MIN], hotrc_pulse_lims_us[VERT][MAX], ctrl_lims_adc[ctrl][VERT][MAX], ctrl_lims_adc[ctrl][VERT][MIN]);
            // if (hotrc_horz_pulse_filt_us >= hotrc_pulse_lims_us[HORZ][CENT])  // Steering: Convert from pulse us to joystick adc equivalent, when pushing left, or right
            //      ctrl_pos_adc[HORZ][FILT] = map (hotrc_horz_pulse_filt_us, hotrc_pulse_lims_us[HORZ][CENT], hotrc_pulse_lims_us[HORZ][MAX], ctrl_lims_adc[ctrl][HORZ][CENT], ctrl_lims_adc[ctrl][HORZ][MAX]);
            // else ctrl_pos_adc[HORZ][FILT] = map (hotrc_horz_pulse_filt_us, hotrc_pulse_lims_us[HORZ][CENT], hotrc_pulse_lims_us[HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][CENT], ctrl_lims_adc[ctrl][HORZ][MIN]);
            // if (hotrc_vert_pulse_filt_us >= hotrc_pulse_lims_us[VERT][CENT])  // Trigger: Convert from pulse us to joystick adc equivalent, when pushing down
            //      ctrl_pos_adc[VERT][FILT] = map (hotrc_vert_pulse_filt_us, hotrc_pulse_lims_us[VERT][CENT], hotrc_pulse_lims_us[VERT][MAX], ctrl_lims_adc[ctrl][VERT][CENT], ctrl_lims_adc[ctrl][VERT][MIN]);
            // else ctrl_pos_adc[VERT][FILT] = map (hotrc_vert_pulse_filt_us, hotrc_pulse_lims_us[VERT][CENT], hotrc_pulse_lims_us[VERT][MIN], ctrl_lims_adc[ctrl][VERT][CENT], ctrl_lims_adc[ctrl][VERT][MAX]);
            // ctrl_pos_adc[HORZ][RAW] = ctrl_pos_adc[HORZ][FILT];  // raw value isn't used I think but just in case
            // ctrl_pos_adc[VERT][RAW] = ctrl_pos_adc[VERT][FILT];  // raw value isn't used I think but just in case
            
            if (hotrc_horz_pulse_us >= hotrc_pulse_lims_us[HORZ][CENT])  // Steering: Convert from pulse us to joystick adc equivalent, when pushing left, or right
                 ctrl_pos_adc[HORZ][RAW] = map (hotrc_horz_pulse_us, hotrc_pulse_lims_us[HORZ][CENT], hotrc_pulse_lims_us[HORZ][MAX], ctrl_lims_adc[ctrl][HORZ][CENT], ctrl_lims_adc[ctrl][HORZ][MAX]);
            else ctrl_pos_adc[HORZ][RAW] = map (hotrc_horz_pulse_us, hotrc_pulse_lims_us[HORZ][CENT], hotrc_pulse_lims_us[HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][CENT], ctrl_lims_adc[ctrl][HORZ][MIN]);
            if (hotrc_vert_pulse_us >= hotrc_pulse_lims_us[VERT][CENT])  // Trigger: Convert from pulse us to joystick adc equivalent, when pushing down
                 ctrl_pos_adc[VERT][RAW] = map (hotrc_vert_pulse_us, hotrc_pulse_lims_us[VERT][CENT], hotrc_pulse_lims_us[VERT][MAX], ctrl_lims_adc[ctrl][VERT][CENT], ctrl_lims_adc[ctrl][VERT][MAX]);
            else ctrl_pos_adc[VERT][RAW] = map (hotrc_vert_pulse_us, hotrc_pulse_lims_us[VERT][CENT], hotrc_pulse_lims_us[VERT][MIN], ctrl_lims_adc[ctrl][VERT][CENT], ctrl_lims_adc[ctrl][VERT][MIN]);  
        }
        else if (ctrl == JOY) {
            ctrl_pos_adc[VERT][RAW] = analogRead (joy_vert_pin);  // Read joy vertical
            ctrl_pos_adc[HORZ][RAW] = analogRead (joy_horz_pin);  // Read joy horizontal
            // ema_filt (ctrl_pos_adc[VERT][RAW], &ctrl_pos_adc[VERT][FILT], ctrl_ema_alpha[ctrl]);  // do ema filter to determine joy_vert_filt
            // ema_filt (ctrl_pos_adc[HORZ][RAW], &ctrl_pos_adc[HORZ][FILT], ctrl_ema_alpha[ctrl]);  // do ema filter to determine joy_horz_filt

        }
        ema_filt (ctrl_pos_adc[VERT][RAW], &ctrl_pos_adc[VERT][FILT], ctrl_ema_alpha[ctrl]);  // do ema filter to determine joy_vert_filt
        ema_filt (ctrl_pos_adc[HORZ][RAW], &ctrl_pos_adc[HORZ][FILT], ctrl_ema_alpha[ctrl]);  // do ema filter to determine joy_horz_filt
        ctrl_pos_adc[VERT][FILT] = constrain (ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
        ctrl_pos_adc[HORZ][FILT] = constrain (ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);

        if (ctrl_pos_adc[VERT][RAW] > ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][RAW] < ctrl_db_adc[VERT][TOP]) {
            ctrl_pos_adc[VERT][FILT] = ctrl_pos_adc[VERT][CENT];  // if joy vert is in the deadband, set joy_vert_filt to center value
            if (ctrl == HOTRC) hotrc_vert_pulse_filt_us = hotrc_pulse_lims_us[VERT][CENT];
        }
        if (ctrl_pos_adc[HORZ][RAW] > ctrl_db_adc[HORZ][BOT] && ctrl_pos_adc[HORZ][RAW] < ctrl_db_adc[HORZ][TOP]) {
            ctrl_pos_adc[HORZ][FILT] = ctrl_pos_adc[HORZ][CENT];  // if joy horz is in the deadband, set joy_horz_filt to center value
            if (ctrl == HOTRC) hotrc_horz_pulse_filt_us = hotrc_pulse_lims_us[HORZ][CENT];
        }
    }
    if (runmode != SHUTDOWN || !shutdown_complete) { // Unless fully shut down at the moment, set the steering output
        if (ctrl_pos_adc[HORZ][FILT] >= ctrl_db_adc[HORZ][TOP]) {  // If above the top edge of the deadband, turning right
            steer_pulse_safe_us = steer_pulse_stop_us + (int32_t)((float)(steer_pulse_right_us - steer_pulse_stop_us) * (1 - ((float)steer_safe_percent * speedo_filt_mph / ((float)speedo_redline_mph * 100) )));
            steer_pulse_out_us = map (ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][TOP], ctrl_lims_adc[ctrl][HORZ][MAX], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the right of deadband
        }
        else if (ctrl_pos_adc[HORZ][FILT] <= ctrl_db_adc[HORZ][BOT]) {  // If below the bottom edge of the deadband, turning left
            steer_pulse_safe_us = steer_pulse_stop_us - (int32_t)((float)(steer_pulse_stop_us - steer_pulse_left_us) * (1 - ((float)steer_safe_percent * speedo_filt_mph / ((float)speedo_redline_mph * 100) )));
            steer_pulse_out_us = map (ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][BOT], ctrl_lims_adc[ctrl][HORZ][MIN], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the left of deadband
        }
        else steer_pulse_out_us = steer_pulse_stop_us;  // Stop the steering motor if inside the deadband
    }

    // Voltage of vehicle battery - takes 70 us to read, convert, and filter
    ignition_sense = read_battery_ignition();  // Updates battery voltage reading and returns ignition status
    // battery_v = convert_units ((float)analogRead (battery_pin), battery_convert_v_per_adc, battery_convert_invert);
    // ema_filt (battery_v, &battery_filt_v, battery_ema_alpha);  // Apply EMA filter

    if (ctrl == JOY) ignition = ignition_sense;
    else if (ctrl == HOTRC) {
        if (hotrc_ch3_sw_event) {  // Turn on/off the vehicle ignition
            if (hotrc_suppress_next_ch3_event) hotrc_suppress_next_ch3_event = false;
            else ignition = !ignition;
            hotrc_ch3_sw_event = false;
        }
        if (hotrc_ch4_sw_event) {
            if (hotrc_suppress_next_ch4_event) hotrc_suppress_next_ch4_event = false;
            else flycruise_toggle_request = true;
            hotrc_ch4_sw_event = false;    
        }
        hotrc.calc();  // Add latest vert pulse reading into history log and calculate avg value for detecting loss of radio reception
        // hotrc.print();
        if (hotrc_vert_pulse_us > hotrc.get_failsafe_min() && hotrc_vert_pulse_us < hotrc.get_failsafe_max()) {
            if (hotrc_radio_detected && hotrcPanicTimer.expired()) {
                hotrc_radio_detected = false;
                // hotrc_suppress_next_ch3_event = true;  // reject spurious ch3 switch event upon next hotrc poweron
                // hotrc_suppress_next_ch4_event = true;  // reject spurious ch4 switch event upon next hotrc poweron
            }
        }
        else {
            hotrcPanicTimer.reset();
            hotrc_radio_detected = true;
            if (!ignition_output_enabled) {  // Ignition stays low until the hotrc is detected here, then output is allowed
                ignition_output_enabled = true;
                // set_pin (ignition_pin, OUTPUT);  // do NOT plug in the joystick when using the hotrc to avoid ign contention
            }
        }
    }
    // printf (" 1C:%4ld 2C:%4ld 1ctR:%4ld 2ctR:%4ld 1ctF:%4ld 2ctF:%4ld c:%ld i:%ld \n", hotrc_horz_pulse_filt_us, hotrc_vert_pulse_filt_us, ctrl_pos_adc[HORZ][RAW], ctrl_pos_adc[VERT][RAW], ctrl_pos_adc[HORZ][FILT], ctrl_pos_adc[VERT][FILT], ctrl, intcount);
    // hotrc.calc();  // Add latest vert pulse reading into history log and calculate avg value for detecting loss of radio reception
    // hotrc.print();

    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "joy");  //
    
    // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    //
    // printf("mode: %d, panic: %d, vpos: %4ld, min: %4ld, max: %4ld, elaps: %6ld", runmode, panic_stop, ctrl_pos_adc[VERT][FILT], hotrc_pos_failsafe_min_us, hotrc_pos_failsafe_max_us, hotrcPanicTimer.elapsed());
    if (basicmodesw) runmode = BASIC;  // if basicmode switch on --> Basic Mode
    else if (runmode != CAL && (panic_stop || !ignition)) runmode = SHUTDOWN;
    else if (runmode != CAL && (starter || engine_stopped())) runmode = STALL;  // otherwise if engine not running --> Stall Mode
    
    if (runmode == BASIC) {  // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            // syspower = HIGH;  // Power up devices if not already
            // enable_pids (0, 0, 0);
            gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        if (!engine_stopped() && !basicmodesw) runmode = HOLD;  // If we turned off the basic mode switch with engine running, go to Hold mode. If engine is not running, we'll end up in Stall Mode automatically
    }
    else if (runmode == SHUTDOWN) { // In shutdown mode we stop the car if it's moving then park the motors.
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            // enable_pids (1, 1, 0);
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
        if (ignition && !panic_stop) {
            // syspower = HIGH;  // Power up devices if not already
            runmode = STALL;
        }
        if (!shutdown_complete) {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (car_stopped() || stopcarTimer.expired()) {  // If car has stopped, or timeout expires, then release the brake
                if (shutdown_color == LPNK) {  // On first time through here
                    // enable_pids (0, 0, 0);
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
                pressure_target_psi = pressure_target_psi + (panic_stop) ? pressure_panic_increment_psi : pressure_hold_increment_psi;  // Slowly add more brakes until car stops
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
        // if (we_just_switched_modes) enable_pids (0, 0, 0);
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][BOT]) pressure_target_psi = pressure_min_psi;  // If in deadband or being pushed up, no pressure target
        else pressure_target_psi = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][BOT], (float)ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_psi, pressure_max_psi);  // Scale joystick value to pressure adc setpoint
        if (!starter && !engine_stopped()) runmode = HOLD;  // Enter Hold Mode if we started the car
        // Throttle behavior is handled in pid section
    }
    else if (runmode == HOLD) {
        if (we_just_switched_modes) {  // Release throttle and push brake upon entering hold mode
            // enable_pids (1, 1, 0);
            tach_target_rpm = tach_idle_rpm;  // Let off gas (if gas using PID mode)
            if (car_stopped()) pressure_target_psi = pressure_filt_psi + pressure_hold_increment_psi; // If the car is already stopped then just add a touch more pressure and then hold it.
            else if (pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = pressure_hold_initial_psi;  //  These hippies need us to stop the car for them
            brakeIntervalTimer.reset();
            stopcarTimer.reset();
            joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
        }
        if (brakeIntervalTimer.expired() && !stopcarTimer.expired()) {  // Each interval the car is still moving, push harder
            if (!car_stopped()) pressure_target_psi = pressure_target_psi + pressure_hold_increment_psi;
            brakeIntervalTimer.reset();
        }
        if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && (ctrl == JOY || hotrc_radio_detected)) runmode = FLY; // Enter Fly Mode upon joystick movement from center to above center.
        // Possibly add "&& car_stopped()" to above check?
    }
    else if (runmode == FLY) {
        if (we_just_switched_modes) {
            // enable_pids (1, 1, 1);
            gesture_progress = 0;
            gestureFlyTimer.set (gesture_flytimeout_us); // Initialize gesture timer to already-expired value
            cruise_sw_held = false;
            cruiseSwTimer.reset();
            flycruise_toggle_request = false;
            car_initially_moved = !car_stopped();  // note whether car moving going into fly mode (usually not), this turns true once it has initially got moving
        }
        if (!car_initially_moved) {
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) runmode = HOLD;  // Must keep pulling trigger until car moves, or drop back to hold mode
            else if (!car_stopped()) car_initially_moved = true;  // Once car moves, we're allowed to stay in fly mode
        }
        else if (car_stopped()) runmode = HOLD;  // Go to Hold Mode if we have come to a stop  // && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT]
        if (ctrl == HOTRC && !(simulating && sim_joy) && !hotrc_radio_detected) runmode = HOLD;  // Radio must be good to fly. This should already be handled elsewhere but another check can't hurt
        else {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP])  {  // If we are trying to accelerate
                tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], tach_idle_rpm, tach_govern_rpm);
            }
            else tach_target_rpm = tach_idle_rpm;  // Let off gas (if gas using PID mode)
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT])  {  // If we are trying to brake, scale joystick value to determine pressure adc setpoint
                pressure_target_psi = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][BOT], (float)ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_psi, pressure_max_psi);
            }
            else pressure_target_psi = pressure_min_psi;  // Default when joystick not pressed   
        }
        // Cruise mode can be entered by pressing a controller button, or by holding the brake on full for a half second. Which epends on the cruise_gesturing flag.
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (flycruise_toggle_request) runmode = CRUISE;
        if (ctrl == JOY) {
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
            // if (!cruise_sw) {  // If button not currently pressed
            //     if (cruise_sw_held && cruiseSwTimer.expired()) runmode = CRUISE;  // After a long press of sufficient length, upon release enter Cruise mode
            //     cruise_sw_held = false;  // Cancel button held state
            // }
            // else if (!cruise_sw_held) {  // If the button just now got pressed
            //     cruiseSwTimer.reset(); // Start hold time timer
            //     cruise_sw_held = true;  // Get into button held state
            // }
        }
        // else if (flycruise_toggle_request) {
        //     flycruise_toggle_request = false;
        //     runmode = CRUISE;
        // }
    }
    else if (runmode == CRUISE) {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            // enable_pids (1, 1, 1);
            speedo_target_mph = speedo_filt_mph;
            pressure_target_psi = pressure_min_psi;  // Let off the brake and keep it there till out of Cruise mode
            gestureFlyTimer.reset();  // reset gesture timer
            cruise_sw_held = false;
            cruise_adjusting = false;
            flycruise_toggle_request = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) {  // When joystick vert above center, increase the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], tach_filt_rpm, tach_govern_rpm);
        }
        else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) {  // When joystick vert below center, decrease the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_lims_adc[ctrl][VERT][MIN], (float)ctrl_db_adc[VERT][BOT], tach_idle_rpm, tach_filt_rpm);
        }
        else cruise_adjusting = false;  // if joystick at center
        if (cruise_adjusting) speedo_target_mph = speedo_filt_mph;
        
        // This old gesture trigger drops to Fly mode if joystick moved quickly from center to bottom
        // if (ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN]+default_margin_adc && abs(mycros() - gesture_timer_us) < gesture_flytimeout_us)  runmode = FLY;  // If joystick quickly pushed to bottom 
        // printf ("hotvpuls=%ld, hothpuls=%ld, joyvfilt=%ld, joyvmin+marg=%ld, timer=%ld\n", hotrc_vert_pulse_us, hotrc_horz_pulse_us, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc, gesture_timer_us);
        if (flycruise_toggle_request) runmode = FLY;
        if (ctrl == JOY) {
            if (ctrl_pos_adc[VERT][FILT] > ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
            else if (gestureFlyTimer.expired()) runmode = FLY;  // New gesture to drop to fly mode is hold the brake all the way down for 500 ms
            // if (cruise_sw) cruise_sw_held = true;  // Pushing cruise button sets up return to fly mode
            // else if (cruise_sw_held) {  // Release of button drops us back to fly mode
            //     cruise_sw_held = false;
            //     runmode = FLY;
            // }
        }
        // else if (flycruise_toggle_request) {
        //     flycruise_toggle_request = false;
        //     runmode = FLY;
        // }
        if (car_stopped()) {  // In case we slam into a brick wall, get out of cruise mode
            if (serial_debugging) Serial.println (F("Error: Car stopped in cruise mode"));  // , speedo_filt_mph, neutral
            runmode = HOLD;  // Back to Hold Mode  
        }
    }
    else if (runmode == CAL) {
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            // enable_pids (0, 0, 0);
            calmode_request = false;
            cal_pot_gas_ready = false;
            cal_pot_gasservo = false;
            cal_joyvert_brkmotor = false;
            cal_set_hotrc_failsafe_ready = false;
        }
        else if (calmode_request) runmode = SHUTDOWN;
        if (!cal_pot_gas_ready) {
            float temp = map (pot_filt_percent, pot_min_percent, pot_max_percent, (float)gas_pulse_ccw_max_us, (float)gas_pulse_cw_min_us);
            if (temp <= (float)gas_pulse_idle_us && temp >= (float)gas_pulse_redline_us) cal_pot_gas_ready = true;
        }
        if (btn_press_action == SHORT) {
            hotrc.set_failsafe();
            hotrc.print();  // Also perhaps write values to flash
            std::cout << "\nHotrc failsafe range set!  Min: " << hotrc.get_failsafe_min() << "adc, Max: " << hotrc.get_failsafe_max() << " adc, including " << hotrc.get_pad() << " adc pad both ways" << std::endl;
            btn_press_action = NONE;
        }
    }
    else { // Obviously this should never happen
        if (serial_debugging) Serial.println (F("Error: Invalid runmode entered"));
        runmode = SHUTDOWN;
    }
    if (runmode != SHUTDOWN && runmode != BASIC) park_the_motors = false;
    if (runmode != oldmode) disp_runmode_dirty = true;  // runmode should not be changed after this point in loop
    we_just_switched_modes = (runmode != oldmode);
    
    // cout << "rm:" << runmode << " om:" << oldmode << "vert:" << ctrl_pos_adc[VERT][FILT] << " up?" << (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) << " jc?" << joy_centered << "\n";
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "mod");  //

    // Update motor outputs - takes 185 us to handle every 30ms when the pid timer expires, otherwise 5 us
    //
    
    // Steering - Update motor output
    if (steerPidTimer.expired()) {
        steerPidTimer.reset();
        steer_pulse_out_us = constrain (steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);  // Don't be out of range
        steer_servo.writeMicroseconds (steer_pulse_out_us);   // Write steering value to jaguar servo interface
    }
    // Brakes - Update motor output
    if (brakePidTimer.expired()) {
        brakePidTimer.reset();
        if (runmode == CAL && cal_joyvert_brkmotor) {
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) brake_pulse_out_us = (float)map (ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], brake_pulse_stop_us, brake_pulse_extend_us);
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) brake_pulse_out_us = (float)map (ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_db_adc[VERT][BOT], brake_pulse_retract_us, brake_pulse_stop_us);
            else brake_pulse_out_us = (float)brake_pulse_stop_us;
        }
        else if (park_the_motors) {
            if (brake_pos_filt_in + brake_pos_margin_in <= brake_pos_park_in)  // If brake is retracted from park point, extend toward park point, slowing as we approach
                brake_pulse_out_us = (float)map ((int32_t)brake_pos_filt_in, (int32_t)brake_pos_park_in, (int32_t)brake_pos_nom_lim_retract_in, brake_pulse_stop_us, brake_pulse_extend_us);
            else if (brake_pos_filt_in - brake_pos_margin_in >= brake_pos_park_in)  // If brake is extended from park point, retract toward park point, slowing as we approach
                brake_pulse_out_us = (float)map ((int32_t)brake_pos_filt_in, (int32_t)brake_pos_park_in, (int32_t)brake_pos_nom_lim_extend_in, brake_pulse_stop_us, brake_pulse_retract_us);
        }
        else if (runmode != BASIC) brakeQPID.Compute();  // Otherwise the pid control is active
        if (runmode != BASIC || park_the_motors) {
            if (runmode == CAL && cal_joyvert_brkmotor)  // In Cal mode constrain the motor to its entire range, instead of to the calibrated limits
                brake_pulse_out_us = constrain (brake_pulse_out_us, (float)brake_pulse_retract_min_us, (float)brake_pulse_extend_max_us);  // Send to the actuator. Refuse to exceed range    
            else {  // Prevent any movement of motor which would exceed position limits. Improve this by having the motor actively go back toward position range if position is beyond either limit
                if ( ((brake_pos_filt_in + brake_pos_margin_in <= brake_pos_nom_lim_retract_in) && ((int32_t)brake_pulse_out_us < brake_pulse_stop_us)) ||  // If the motor is at or past its position limit in the retract direction, and we're intending to retract more ...
                     ((brake_pos_filt_in - brake_pos_margin_in >= brake_pos_nom_lim_extend_in) && ((int32_t)brake_pulse_out_us > brake_pulse_stop_us)) )  // ... or same thing in the extend direction ...
                    brake_pulse_out_us = brake_pulse_stop_us;  // ... then stop the motor
                brake_pulse_out_us = constrain (brake_pulse_out_us, (float)brake_pulse_retract_us, (float)brake_pulse_extend_us);  // Send to the actuator. Refuse to exceed range    
            } 
            brake_servo.writeMicroseconds ((int32_t)brake_pulse_out_us);  // Write result to jaguar servo interface
        }
    }
    // Cruise - Update gas target. Controls gas rpm target to keep speed equal to cruise mph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
    if (cruisePidTimer.expired() && runmode == CRUISE && !cruise_adjusting) {
        cruisePidTimer.reset();
        cruiseQPID.Compute();  // 
    }
    // Gas - Update servo output. Determine gas actuator output from rpm target.  PID loop is effective in Fly or Cruise mode.
    if (gasPidTimer.expired()) {
        gasPidTimer.reset();
        if (park_the_motors) gas_pulse_out_us = gas_pulse_idle_us + gas_pulse_park_slack_us;
        else if (runmode == STALL) {  // Stall mode runs the gas servo directly proportional to joystick. This is truly open loop
            if (starter) gas_pulse_out_us = gas_pulse_govern_us;
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) gas_pulse_out_us = gas_pulse_idle_us;  // If in deadband or being pushed down, we want idle
            else gas_pulse_out_us = map (ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], gas_pulse_idle_us, gas_pulse_govern_us);  // Actuators still respond and everything, even tho engine is turned off
        }
        else if (runmode != BASIC) {
            if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo) 
                gas_pulse_out_us = (int32_t)(map (pot_filt_percent, pot_min_percent, pot_max_percent, (float)gas_pulse_ccw_max_us, (float)gas_pulse_cw_min_us));
            else if (gasQPID.GetMode() == (uint8_t)QPID::Control::manual)  // This isn't really open loop, more like simple proportional control, with output set proportional to target 
                gas_pulse_out_us = (int32_t)(map (tach_target_rpm, tach_idle_rpm, tach_govern_rpm, (float)gas_pulse_idle_us, (float)gas_pulse_govern_us)); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
            else gasQPID.Compute();  // Do proper pid math to determine gas_pulse_out_us from engine rpm error
            // printf ("Gas PID   rm= %+-4ld target=%-+9.4lf", runmode, (float)tach_target_rpm);
            // printf (" output = %-+9.4lf,  %+-4ld\n", gasQPID.get_output(), gas_pulse_out_us);
        }
        if (runmode != BASIC || park_the_motors) {
            if (runmode == CAL && cal_pot_gas_ready && cal_pot_gasservo)
                gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
            else gas_pulse_out_us = constrain (gas_pulse_out_us, gas_pulse_govern_us, gas_pulse_idle_us);
            // printf (" output = %-+9.4lf,  %+-4ld\n", gasQPID.get_output(), gas_pulse_out_us);
            gas_servo.writeMicroseconds (gas_pulse_out_us);  // Write result to servo
        }
    }
    if (park_the_motors) {  //  When parking motors, IF the timeout expires OR the brake and gas motors are both close enough to the park position THEN stop trying to park the motors
        bool brake_parked = (abs(brake_pos_filt_in - brake_pos_park_in) <= brake_pos_margin_in);
        bool gas_parked = ((gas_pulse_out_us == gas_pulse_idle_us + gas_pulse_park_slack_us) && gasServoTimer.expired());
        if ((brake_parked && gas_parked) || motorParkTimer.expired()) park_the_motors = false;
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
        
    // Touchscreen handling - takes 800 us to handle every 20ms when the touch timer expires, otherwise 20 us (includes touch timer + encoder handling w/o activity)
    //
    int32_t touch_x, touch_y, trow, tcol;
    bool get_touched;
    // if (screen.ts.touched() == 1 ) { // Take actions if one touch is detected. This panel can read up to two simultaneous touchpoints
    get_touched = (touch_irq_pin == 255) ? ts.touched() : ts.tirqTouched();
    if (get_touched) { // Take actions if one touch is detected. This panel can read up to two simultaneous touchpoints
        touch_accel = 1 << touch_accel_exponent;  // determine value editing rate
        // TS_Point touchpoint = screen.ts.getPoint();  // Retreive a point
        TS_Point touchpoint = ts.getPoint();  // Retreive a point
        #ifdef CAP_TOUCH
            touchpoint.x = map (touchpoint.x, 0, disp_height_pix, disp_height_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touchpoint.y = map (touchpoint.y, 0, disp_width_pix, disp_width_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touch_y = disp_height_pix-touchpoint.x; // touch point y coordinate in pixels, from origin at top left corner
            touch_x = touchpoint.y; // touch point x coordinate in pixels, from origin at top left corner
        #else
            touch_x = constrain (touchpoint.x, 355, 3968);
            touch_y = constrain (touchpoint.y, 230, 3930);
            touch_x = map (touch_x, 355, 3968, 0, disp_width_pix);
            touch_y = map (touch_y, 230, 3930, 0, disp_height_pix);
            if (!flip_the_screen) {
                touch_x = disp_width_pix - touch_x;
                touch_y = disp_height_pix - touch_y;
            }
        #endif
        // std::cout << "Touch: fs:" << flip_the_screen << " ptx:" << touchpoint.x << ", pty:" << touchpoint.y << ", ptz:" << touchpoint.z << " x:" << touch_x << ", y:" << touch_y << std::endl;
        // std::cout << "Touch: ptx:" << touchpoint.x << ", pty:" << touchpoint.y << " x:" << touch_x << ", y:" << touch_y << ", z:" << touchpoint.z << std::endl;
        trow = constrain((touch_y + touch_fudge)/touch_cell_v_pix, 0, 4);  // The -8 seems to be needed or the vertical touch seems off (?)
        tcol = (touch_x-touch_margin_h_pix)/touch_cell_h_pix;
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
                    if (++selected_value >= arraysize (dataset_page_names[dataset_page])) selected_value -= arraysize (dataset_page_names[dataset_page]);
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
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                simulating = !simulating;
                touch_longpress_valid = false;
            }
        }
        else if (tcol==2 && trow==0 && (runmode == CAL || (runmode == SHUTDOWN && shutdown_complete))) {
            if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                calmode_request = true;
                touch_longpress_valid = false;
            }  // Pressed the basic mode toggle button. Toggle value, only once per touch
        }
        else if (simulating) {
            if (tcol==3 && trow==0 && sim_basicsw && !touch_now_touched) basicmodesw = !basicmodesw;  // Pressed the basic mode toggle button. Toggle value, only once per touch
            else if (tcol==3 && trow==1 && sim_pressure) adj_val (&pressure_filt_psi, (float)touch_accel, pressure_min_psi, pressure_max_psi);   // (+= 25) Pressed the increase brake pressure button
            else if (tcol==3 && trow==2 && sim_pressure) adj_val (&pressure_filt_psi, (float)(-touch_accel), pressure_min_psi, pressure_max_psi); // (-= 25) Pressed the decrease brake pressure button
            else if (tcol==3 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[HORZ][FILT], -touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (-= 25) Pressed the joystick left button
            else if (tcol==4 && trow==0 && !touch_now_touched) ignition = !ignition; // Pressed the ignition switch toggle button. Toggle value, only once per touch
            else if (tcol==4 && trow==1 && sim_tach) adj_val (&tach_filt_rpm, (float)touch_accel, 0.0, tach_redline_rpm);  // (+= 25) Pressed the increase engine rpm button
            else if (tcol==4 && trow==2 && sim_tach) adj_val (&tach_filt_rpm, (float)(-touch_accel), 0.0, tach_redline_rpm);  // (-= 25) Pressed the decrease engine rpm button
            else if (tcol==4 && trow==3 && sim_joy) adj_val (&ctrl_pos_adc[VERT][FILT], touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (+= 25) Pressed the joystick up button
            else if (tcol==4 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[VERT][FILT], -touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (-= 25) Pressed the joystick down button
            else if (tcol==5 && trow==0 && sim_syspower) {  // You need to enable syspower simulation, be in sim mode and then long-press the syspower button to toggle it. (Hard to do by accident)/
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.get_timeout()) {
                    syspower = !syspower;
                    touch_longpress_valid = false;
                }
            }
            else if (tcol==5 && trow==1 && sim_speedo) adj_val (&speedo_filt_mph, 0.005*(float)touch_accel, 0.0, speedo_redline_mph);  // (+= 50) // Pressed the increase vehicle speed button
            else if (tcol==5 && trow==2 && sim_speedo) adj_val (&speedo_filt_mph, -0.005*(float)touch_accel, 0.0, speedo_redline_mph);  // (-= 50) Pressed the decrease vehicle speed button
            else if (tcol==5 && trow==4 && sim_joy) adj_val (&ctrl_pos_adc[HORZ][FILT], touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (+= 25) Pressed the joystick right button                           
        }
        if (touch_accel_exponent < touch_accel_exponent_max && (touchHoldTimer.elapsed() > (touch_accel_exponent + 1) * touchAccelTimer.get_timeout())) touch_accel_exponent++; // If timer is > the shift time * exponent, and not already maxed, float the edit speed by incrementing the exponent
        touch_now_touched = true;
    }  // (if screen.ts reads a touch)
    else {  // If not being touched, put momentarily-set simulated button values back to default values
        if (simulating) cruise_sw = false;  // // Makes this button effectively momentary
        sim_edit_delta_touch = 0;  // Stop changing value
        touch_now_touched = false;  // remember last touch state
        touch_accel_exponent = 0;
        touchHoldTimer.reset();
        touch_longpress_valid = true;
    }
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "tch");  //

    // Encoder handling
    //
    uint32_t encoder_sw_action = encoder.handleSwitchAction();
    if (encoder_sw_action != Encoder::NONE) {  // First deal with any unhandled switch press events
        if (encoder_sw_action == Encoder::SHORT)  {  // if short press
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
            else if (ctrl == JOY && (!simulating || !sim_cruisesw) && (runmode == FLY || runmode == CRUISE)) flycruise_toggle_request = true;
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
            if (selected_value == 3) adj_bool (&sim_brkpos, sim_edit_delta);
            else if (selected_value == 4) adj_bool (&sim_joy, sim_edit_delta);
            else if (selected_value == 5) adj_bool (&sim_pressure, sim_edit_delta);
            else if (selected_value == 6) adj_bool (&sim_tach, sim_edit_delta);
            else if (selected_value == 7) adj_bool (&sim_speedo, sim_edit_delta);
        }
        else if (dataset_page == PG_JOY) {
            if (selected_value == 2) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][MIN], sim_edit_delta, 0, ctrl_lims_adc[ctrl][HORZ][CENT] - ctrl_lims_adc[ctrl][HORZ][DB] / 2 - 1);
            else if (selected_value == 3) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][MAX], sim_edit_delta, ctrl_lims_adc[ctrl][HORZ][CENT] + ctrl_lims_adc[ctrl][HORZ][DB] / 2 + 1, ctrl_lims_adc[ctrl][HORZ][CENT]);
            else if (selected_value == 4) adj = adj_val (&ctrl_lims_adc[ctrl][HORZ][DB], sim_edit_delta, 0, (ctrl_lims_adc[ctrl][HORZ][CENT] - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - ctrl_lims_adc[ctrl][HORZ][CENT]) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - ctrl_lims_adc[ctrl][HORZ][CENT]) : 2*(ctrl_lims_adc[ctrl][HORZ][CENT] - ctrl_lims_adc[ctrl][HORZ][MIN]));
            else if (selected_value == 5) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][MIN], sim_edit_delta, 0, ctrl_lims_adc[ctrl][VERT][CENT] - ctrl_lims_adc[ctrl][VERT][DB] / 2 - 1);
            else if (selected_value == 6) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][MAX], sim_edit_delta, ctrl_lims_adc[ctrl][VERT][CENT] + ctrl_lims_adc[ctrl][VERT][DB] / 2 + 1, ctrl_lims_adc[ctrl][VERT][CENT]);
            else if (selected_value == 7) adj = adj_val (&ctrl_lims_adc[ctrl][VERT][DB], sim_edit_delta, 0, (ctrl_lims_adc[ctrl][VERT][CENT] - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - ctrl_lims_adc[ctrl][VERT][CENT]) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - ctrl_lims_adc[ctrl][VERT][CENT]) : 2*(ctrl_lims_adc[ctrl][VERT][CENT] - ctrl_lims_adc[ctrl][VERT][MIN]));
            if (adj) calc_deadbands();  // update derived variables relevant to changes made
        }
        else if (dataset_page == PG_CAR) {
            if (selected_value == 0) {
                adj = adj_val (&gas_governor_percent, sim_edit_delta, 0, 100);
                if (adj) calc_governor();  // update derived variables relevant to changes made
            }
            else if (selected_value == 1) adj_val (&tach_idle_rpm, 0.01*(float)sim_edit_delta, 0, tach_redline_rpm - 1);
            else if (selected_value == 2) adj_val (&tach_redline_rpm, 0.01*(float)sim_edit_delta, tach_idle_rpm, 8000);
            else if (selected_value == 3) adj_val (&speedo_idle_mph, 0.01*(float)sim_edit_delta, 0, speedo_redline_mph - 1);
            else if (selected_value == 4) adj_val (&speedo_redline_mph, 0.01*(float)sim_edit_delta, speedo_idle_mph, 30);
            else if (selected_value == 5) gas_open_loop = (sim_edit_delta > 0);
            else if (selected_value == 6 && runmode == CAL) adj_bool (&cal_joyvert_brkmotor, sim_edit_delta);
            else if (selected_value == 7 && runmode == CAL) adj_bool (&cal_pot_gasservo, (sim_edit_delta < 0 || cal_pot_gas_ready) ? sim_edit_delta : -1);
      }
        else if (dataset_page == PG_PWMS) {
            if (selected_value == 0) adj_val (&steer_pulse_left_us, sim_edit_delta, steer_pulse_stop_us + 1, steer_pulse_left_max_us);
            else if (selected_value == 1) adj_val (&steer_pulse_stop_us, sim_edit_delta, steer_pulse_right_us + 1, steer_pulse_left_us - 1);
            else if (selected_value == 2) adj_val (&steer_pulse_right_us, sim_edit_delta, steer_pulse_right_min_us, steer_pulse_stop_us - 1);
            else if (selected_value == 3) adj_val (&brake_pulse_extend_us, sim_edit_delta, brake_pulse_stop_us + 1, brake_pulse_extend_max_us);
            else if (selected_value == 4) adj_val (&brake_pulse_stop_us, sim_edit_delta, brake_pulse_retract_us + 1, brake_pulse_extend_us - 1);
            else if (selected_value == 5) adj_val (&brake_pulse_retract_us, sim_edit_delta, brake_pulse_retract_min_us, brake_pulse_stop_us -1);
            else if (selected_value == 6) adj_val (&gas_pulse_idle_us, sim_edit_delta, gas_pulse_redline_us + 1, gas_pulse_ccw_max_us - gas_pulse_park_slack_us);
            else if (selected_value == 7) adj_val (&gas_pulse_redline_us, sim_edit_delta, gas_pulse_cw_min_us, gas_pulse_idle_us - 1);
        }
        else if (dataset_page == PG_BPID) {
            if (selected_value == 5) brakeQPID.SetKp (brakeQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 6) brakeQPID.SetKi (brakeQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 7) brakeQPID.SetKd (brakeQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_GPID) {
            if (selected_value == 5) gasQPID.SetKp (gasQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 6) gasQPID.SetKi (gasQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 7) gasQPID.SetKd (gasQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_CPID) {
            if (selected_value == 5) cruiseQPID.SetKp (cruiseQPID.GetKp() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 6) cruiseQPID.SetKi (cruiseQPID.GetKi() + 0.001 * (float)sim_edit_delta);
            else if (selected_value == 7) cruiseQPID.SetKd (cruiseQPID.GetKd() + 0.001 * (float)sim_edit_delta);
        }
        else if (dataset_page == PG_TEMP) {        
            // if (selected_value == 4) 
            if (selected_value == 5) adj_val (&hotrc_pulse_failsafe_min_us, sim_edit_delta, 700, hotrc_pulse_failsafe_max_us - 1);
            else if (selected_value == 6) adj_val (&hotrc_pulse_failsafe_max_us, sim_edit_delta, hotrc_pulse_failsafe_min_us + 1, hotrc_pulse_lims_us[VERT][MIN] - 1);
            else if (selected_value == 4) adj_val (&pot_overload, sim_edit_delta, 0, 3);
            else if (selected_value == 7) adj_val (&pressure_adc, sim_edit_delta, pressure_min_adc, pressure_max_adc);
            // else if (selected_value == 7) adj_val (&brake_pos_zeropoint_in, 0.001*sim_edit_delta, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);
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
    if (syspower != syspower_last) {
        syspower_set (syspower);
        syspower_last = syspower;
    }
    if (btn_press_action == LONG) {
        screen.tft_reset();
        btn_press_action = NONE;
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
                if (++heartbeat_state >= arraysize (heartbeat_ekg)) heartbeat_state -= arraysize (heartbeat_ekg);
                heartbeatTimer.set (heartbeat_ekg[heartbeat_state]);
            }
            else if (!heartbeat_pulse && neo_brightness) {
                neo_brightness = (int8_t)((float)neo_brightness_max * (1 - (float)neoTimer.elapsed() / (float)neo_timeout));
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
        else if (neoTimer.expired()) {  // Rainbow fade
            neoTimer.reset();
            neostrip.setPixelColor (0, colorwheel(++neo_wheelcounter));
            neostrip.show();
        }
    }
    // else if (heartbeat_led_pin >= 0) {  // Just make a heartbeat on the native board led
    //     heartbeat_pulse = !heartbeat_pulse;
    //     if (++heartbeat_state >= arraysize (heartbeat_ekg)) heartbeat_state -= arraysize (heartbeat_ekg);
    //     heartbeatTimer.set (heartbeat_ekg[heartbeat_state]);
    //     write_pin (heartbeat_led_pin, heartbeat_pulse);
    // }
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "hrt");
    
    // Display updates
    if (display_enabled) screen.update();
    else {
        if (dataset_page_last != dataset_page) config.putUInt ("dpage", dataset_page);
        dataset_page_last = dataset_page;
        selected_value_last = selected_value;
        simulating_last = simulating;
        oldmode = runmode;
    }
    if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "dis");

    // Kick watchdogs
    Watchdog.reset();  // Kick the watchdog to keep us alive
    // if (display_enabled) screen.watchdog();
 
    // Do the control loop bookkeeping at the end of each loop
    //
    loop_period_us = loopTimer.elapsed();  // us since beginning of this loop
    loopTimer.reset();
    loop_freq_hz = 1000000.0 / ((loop_period_us) ? loop_period_us : 1);  // Prevent potential divide by zero
    loopno++;  // I like to count how many loops
    // if (timestamp_loop) loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "end");    
    if (timestamp_loop) {
        // loop_savetime (looptimes_us, loopindex, loop_names, loop_dirty, "end");  //
        if (loop_period_us > 25000) {
            std::cout << "RM:" << runmode << " us:" << esp_timer_get_time() << " Lp#" << loopno << " us:" << loop_period_us;
            for (int32_t x=1; x<loopindex; x++) std::cout << " " << std::setw(3) << loop_names[x] << x << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1];
            std::cout << std::endl;
        }
        // else {
        //     std::cout << loop_report;
        //     loop_report("");
        // }

        // std::cout << "\rRM:" << runmode << " us:" << esp_timer_get_time() << " Lp#" << loopno << " us:" << loop_period_us;
        // for (int32_t x=1; x<loopindex; x++) std::cout << " " << std::setw(3) << loop_names[x] << x << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1];
        // if (loop_period_us > 25000) printf ("\n");
    }
    loop_int_count = 0;
}