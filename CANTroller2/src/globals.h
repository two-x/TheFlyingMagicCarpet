#pragma once
// #include <SdFat.h>  // SD card & FAT filesystem library
#include <ESP32Servo.h>        // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <DallasTemperature.h>
#include <Wire.h>
#include <SparkFun_MicroPressure.h>
#include <Preferences.h>
#include <iostream>
// #include "freertos/FreeRTOS.h"  // MCPWM pulse measurement code
// #include "freertos/task.h"  // MCPWM pulse measurement code
// #include "driver/mcpwm.h"  // MCPWM pulse measurement code
#include "driver/rmt.h"
#include "RMT_Input.h"
#include "qpid.h" // This is quickpid library except i have to edit some of it
#include "utils.h"
#include "uictrl.h"
#include "devices.h"
#include "temperature.h"

// #define CAP_TOUCH
bool flip_the_screen = true;

// #define pinout_bm2023  // uncomment this to get pin assignments for old control box

#define bootbutton_pin 0        // (button0 / bootstrap high) - This is the "Boot" button on the esp32 board. Active low (existing onboard pullup)#define steer_enc_a_pin 1       // (adc) - Reserved for a steering quadrature encoder. Encoder "A" signal
#define steer_enc_a_pin 1       // (adc1ch0) - Reserved for a steering quadrature encoder. Encoder "A" signal
#define steer_enc_b_pin 2       // (adc1ch1) - Reserved for a steering quadrature encoder. Encoder "B" signal
#define tft_dc_pin 3            // (adc1ch2 / strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
#define mulebatt_pin 4          // (adc1ch3) - Analog input, battery voltage sense, full scale is 16V
#define pot_wipe_pin 5          // (adc1ch4) - Analog in from 20k pot
#define brake_pos_pin 6         // (adc1ch5) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#define pressure_pin 7          // (adc1ch6) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
#define i2c_sda_pin 8           // (i2c0 sda / adc1ch7) - i2c bus for airspeed/map sensors, lighting board, cap touchscreen
#define i2c_scl_pin 9           // (i2c0 scl / adc1ch8) - i2c bus for airspeed/map sensors, lighting board, cap touchscreen
#define tft_cs_pin 10           // (spi0 cs / adc1ch9) - Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
#define tft_mosi_pin 11         // (spi0 mosi / adc2ch0) - Used as spi interface data for touchscreen, sd card and tft screen
#define tft_sclk_pin 12         // (spi0 sclk / adc2ch1) - Used as spi interface clock for touchscreen, sd card and tft screen
#define tft_miso_pin 13         // (spi0 miso / adc2ch2) - Used as spi interface data from sd card and possibly (?) tft screen
#define hotrc_ch2_vert_pin 14   // (pwm0 / adc2ch3) - Hotrc Ch2 bidirectional trigger input
#define hotrc_ch1_horz_pin 15   // (pwm1 / adc2ch4) - Hotrc Ch1 thumb joystick input
#define gas_pwm_pin 16          // (pwm1 / adc2ch5) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
#define brake_pwm_pin 17        // (pwm0 / adc2ch6 / tx1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped)
#define steer_pwm_pin 18        // (pwm0 / adc2ch7 / rx1) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
#define onewire_pin 19          // (usb-otg / adc2ch8) - Onewire bus for temperature sensor data
#define hotrc_ch3_ign_pin 20    // (usb-otg / adc2ch9) - Ignition control, Hotrc Ch3 PWM toggle signal
#define hotrc_ch4_cruise_pin 21 // (pwm0) - Cruise control, Hotrc Ch4 PWM toggle signal
#define speedo_pulse_pin 35     // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. (Open collector sensors need pullup)
#define starter_pin 36          // (spi-ram / oct-spi) - Input, active high when vehicle starter is engaged (needs pulldown)
#define tach_pulse_pin 37       // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup) - Note: placed on p36 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
#define sdcard_cs_pin 38        // (spi-ram / oct-spi) - Output, chip select for SD card controller on SPI bus
#define basicmodesw_pin 39      // Input, asserted to tell us to run in basic mode, active low (has ext pullup) - Note: placed on p39 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
#define encoder_b_pin 40        // Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
#define encoder_a_pin 41        // Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
#define encoder_sw_pin 42       // Input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
#define uart_tx_pin 43          // "TX" (uart0 tx) - Needed for serial monitor
#define uart_rx_pin 44          // "RX" (uart0 rx) - Needed for serial monitor. In theory we could dual-purpose this for certain things, as we haven't yet needed to accept input over the serial monitor
#define ign_out_pin 45          // (bootstrap low) - Output for Hotrc to a relay to kill the car ignition. Note, Joystick ign button overrides this if connected and pressed
#define syspower_pin 46         // (bootstrap low) - Output, flips a relay to power all the tranducers. This is actually the neopixel pin on all v1.1 devkit boards.
#define touch_cs_pin 47         // Output, chip select for resistive touchscreen, active low
#define neopixel_pin 48         // (rgb led) - Data line to onboard Neopixel WS281x (on all v1 devkit boards)

#ifdef pinout_bm2023            // 231003: Swapped these 5 signals from pins below (bm2023) to above (bm2024)
    #define tach_pulse_pin 36   // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup) - Note: placed on p36 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
    #define ign_out_pin 37      // (spi-ram / oct-spi) - Output for Hotrc to a relay to kill the car ignition. Note, Joystick ign button overrides this if connected and pressed
    #define syspower_pin 38     // (spi-ram / oct-spi) - Output, flips a relay to power all the tranducers. This is actually the neopixel pin on all v1.1 devkit boards.
    #define starter_pin 45      // (bootstrap low) - Input, active high when vehicle starter is engaged (needs pulldown)
    #define sdcard_cs_pin 46    // (bootstrap low) - Output, chip select for SD card controller on SPI bus,
#endif

// External pullup/pulldown resistors:   (Note: "BB" = On dev breadboards only, "PCB" = On vehicle PCB only)
// 1. mulebatt_pin: Add 1M-ohm to 3.3V. Allows detecting broken connection.
// 2. brake_pos_pin, pressure_pin: Add 1M-ohm to GND. Allows detecting unconnected sensors or broken connections.
// 3. onewire_pin: Add 4.7k-ohm to 3.3V. Needed for open collector sensor output, to define logic-high voltage level.
// 4. tach_pulse_pin, speedo_pulse_pin: (PCB) Add 4.7k-ohm to 3.3V. For open collector sensor outputs. (BB) If no sensor is present: connect 4.7k-ohm to GND instead. Allows sensor detection.
// 5. neopixel_pin: (PCB) Add 300 ohm in series (between pin and the DataIn pin of the 1st pixel). (BB) Same, but this one is likely optional, e.g. mine works w/o it.  For signal integrity over long wires. 
// 6. uart_tx_pin: (PCB) Add 22k-ohm to GND. (BB) Connect 1M-ohm to 3.3V instead. For boot detection of vehicle PCB, so defaults are set appropriately.

// ESP32-S3 TRM: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#dma
// ESP32-S3 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
// ESP32-S3 has 5 DMA channels in each direction. We would use them for SPI data out to TFT, Neopixel data out, and possibly out to the 3 motor outputs and in from the 4 hotrc channels.
// DMA works with: RMT, I2S0, I2S1, SPI2, SPI3, ADC, internal RAM, external PSRAM, and a few others (see the TRM)
// Official pin capabilities: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html?highlight=devkitc#user-guide-s3-devkitc-1-v1-1-header-blocks
// ESP32 pins 34, 35, 36, 39 are input-only (applies to S3?).  ADC ch2 will not work if wifi is enabled
// Bootstrap pins: Pin 0 must be pulled high, and pins 45 and 46 pulled low during bootup
// ESP32 errata 3.11: Pin 36 and 39 will be pulled low for ~80ns when "certain RTC peripherals power up"
// ESP32 pullups/downs (~45k-ohm) details: https://www.esp32.com/viewtopic.php?f=12&t=34831

#define tft_ledk_pin -1   // Output, optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define touch_irq_pin 255 // Input, optional touch occurence interrupt signal (for resistive touchscreen, prevents spi bus delays) - Set to 255 if not used
#define tft_rst_pin -1    // TFT Reset allows us to reboot the screen hardware when it crashes. Otherwise connect screen reset line to esp reset pin

// Globals -------------------
bool serial_debugging = true;
bool take_temperatures = true;
bool keep_system_powered = false;    // Use true during development
bool allow_rolling_start = false;    // May be a smart prerequisite, may be us putting obstacles in our way
bool share_boot_button = false;      // Set true if joystick cruise button is in parallel with esp native "boot" button
bool remote_start_support = false;
bool starter_signal_support = true;
bool cruise_speed_lowerable = true;  // Allows use of trigger to adjust cruise speed target without leaving cruise mode.  Otherwise cruise button is a "lock" button, and trigger activity cancels lock
bool cruise_fixed_throttle = true;   // Cruise mode fixes the throttle angle rather than controlling for a target speed
bool autostop_disabled = true;       // Temporary measure to keep brake behaving until we get it debugged. Eventually should be false
bool timestamp_loop = false;         // Makes code write out timestamps throughout loop to serial port
uint32_t timestamp_loop_linefeed_threshold = 15000;  // Leaves prints of loops taking > this for analysis. Set to 0 prints every loop
#define pwm_jaguars true

// Persistent config storage
Preferences config;

// I2C related
I2C i2c(i2c_sda_pin, i2c_scl_pin);

// Declare Hotrc RMT Inputs in global scope
RMTInput hotrc_horz(RMT_CHANNEL_4, gpio_num_t(hotrc_ch1_horz_pin));
RMTInput hotrc_vert(RMT_CHANNEL_5, gpio_num_t(hotrc_ch2_vert_pin));
RMTInput hotrc_ch3(RMT_CHANNEL_6, gpio_num_t(hotrc_ch3_ign_pin));
RMTInput hotrc_ch4(RMT_CHANNEL_7, gpio_num_t(hotrc_ch4_cruise_pin));

// Globals -------------------
//

#ifdef pwm_jaguars
static Servo brake_servo;
static Servo steer_servo;
#else // jaguars controlled over asynchronous serial port
#include <HardwareSerial.h>
HardwareSerial jagPort(1); // Open serisl port to communicate with jaguar controllers for steering & brake motors
#endif

// run state machine related
enum runmodes { BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL };
runmodes runmode = SHUTDOWN;
runmodes disp_oldmode = SHUTDOWN;   // So we can tell when the mode has just changed. start as different to trigger_mode start algo
int32_t gesture_progress = 0;       // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
bool shutdown_incomplete = true;     // Shutdown mode has not completed its work and can't yet stop activity
bool we_just_switched_modes = true; // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;       // Indicates we should release the brake & gas so the pedals can be used manually without interference
bool car_hasnt_moved = false;         // Whether car has moved at all since entering fly mode
bool calmode_request = false;
bool joy_centered = false;
bool panic_stop = false;
bool remote_starting = false;
bool remote_starting_last = false;
bool remote_start_toggle_request = false;
float cruise_ctrl_extent_adc;       // During cruise adjustments, saves farthest trigger position read
float cruise_adjust_scaling_percent = 40;  // What ratio of full throttle range is the max available with each adjustment event?
bool cruise_trigger_released = false;
bool cruise_gesturing = false;          // Is cruise mode enabled by gesturing?  Otherwise by press of cruise button
bool cruise_sw_held = false;
bool cruise_adjusting = false;
bool flycruise_toggle_request = false;
int32_t flycruise_vert_margin_adc = 25; // Margin of error for determining hard brake value for dropping out of cruise mode
Timer gestureFlyTimer; // Used to keep track of time for gesturing for going in and out of fly/cruise modes
// Timer cruiseSwTimer;  // Was used to require a medium-length hold time pushing cruise button to switch modes
Timer sleepInactivityTimer(15000000);           // After shutdown how long to wait before powering down to sleep
Timer stopcarTimer(8000000);                    // Allows code to fail in a sensible way after a delay if nothing is happening
uint32_t motor_park_timeout_us = 4000000;       // If we can't park the motors faster than this, then give up.
uint32_t gesture_flytimeout_us = 400000;        // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
uint32_t cruise_sw_timeout_us = 500000;         // how long do you have to hold down the cruise button to start cruise mode (in us)
uint32_t cruise_antiglitch_timeout_us = 350000; // Target speed won't change until manual adjustment is outside deadboand for longer than this
Timer cruiseAntiglitchTimer(cruise_antiglitch_timeout_us);
Timer motorParkTimer(motor_park_timeout_us);
bool running_on_devboard;  // true = running on dev board, false = running on the car.  Requires pullup/pulldown resistors configured as above
int32_t neobright = 8.0;  // lets us dim/brighten the neopixels
float neodesat = 0.0;  // lets us de/saturate the neopixels

// potentiometer related
Potentiometer pot(pot_wipe_pin);

// simulator related
Simulator simulator(pot);
bool simulating_last = false;
Timer simTimer; // NOTE: unused
int32_t sim_edit_delta = 0;
int32_t sim_edit_delta_touch = 0;
int32_t sim_edit_delta_encoder = 0;

// calibration related
bool cal_joyvert_brkmotor_mode = false; // Allows direct control of brake motor using controller vert
bool cal_pot_gasservo_mode = false;     // Allows direct control of gas servo using pot. First requires pot to be in valid position before mode is entered
bool cal_pot_gasservo_ready = false;    // Whether pot is in valid range
bool cal_set_hotrc_failsafe_ready = false;

// diag/monitoring variables
Timer loopTimer(1000000); // how long the previous main loop took to run (in us)
uint32_t loop_period_us;
float looptime_sum_s;
float looptime_avg_ms;
float loop_freq_hz = 1;              // run loop real time frequency (in Hz)
volatile int32_t loop_int_count = 0; // counts interrupts per loop
int32_t loopno = 1;
uint32_t looptimes_us[20];
bool loop_dirty[20];
int32_t loopindex = 0;
int64_t looptime_cout_mark_us;
uint32_t looptime_cout_us;
bool booted = false;
bool diag_ign_error_enabled = true;

// pushbutton related
enum sw_presses { NONE, SHORT, LONG }; // used by encoder sw and button algorithms
bool boot_button_last = 0;
bool boot_button = 0;
bool boot_button_timer_active = false;
bool boot_button_suppress_click = false;
bool boot_button_action = NONE;

// external digital input and output signal related
bool ignition = LOW;
bool ignition_last = ignition;
bool ignition_output_enabled = false; // disallows configuration of ignition pin as an output until hotrc detected
bool ignition_sense = ignition;
float ignition_on_thresh_v = 2.0; // Below this voltage ignition is considered off
bool syspower = HIGH;
bool syspower_last = syspower;
bool basicmodesw = LOW;
bool cruise_sw = LOW;
bool starter = LOW;
bool starter_last = LOW;

enum temp_categories { AMBIENT = 0, ENGINE = 1, WHEEL = 2 };
enum temp_lims { DISP_MIN, NOM_MIN, NOM_MAX, WARNING, ALARM, DISP_MAX }; // Possible sources of gas, brake, steering commands
float temp_lims_f[3][6]{
    {0.0, 45.0, 115.0, 120.0, 130.0, 220.0},  // [AMBIENT][MIN/NOM_MIN/NOM_MAX/WARNING/ALARM]
    {0.0, 178.0, 198.0, 202.0, 205.0, 220.0}, // [ENGINE][MIN/NOM_MIN/NOM_MAX/WARNING/ALARM]
    {0.0, 50.0, 120.0, 130.0, 140.0, 220.0},
};                               // [WHEEL][MIN/NOM_MIN/NOM_MAX/WARNING/ALARM] (applies to all wheels)
float temp_room = 77.0;          // "Room" temperature is 25 C = 77 F  Who cares?
float temp_sensor_min_f = -67.0; // Minimum reading of sensor is -25 C = -67 F
float temp_sensor_max_f = 257.0; // Maximum reading of sensor is 125 C = 257 F

TemperatureSensorManager temperature_sensor_manager(onewire_pin);

// encoder related
Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);

// mule battery related
BatterySensor battery_sensor(mulebatt_pin);

// controller related
enum ctrls { HOTRC, OTHER, SIM, HEADLESS }; // Possible sources of gas, brake, steering commands
enum ctrl_axes { HORZ, VERT, CH3, CH4 };
enum ctrl_thresh { MIN, CENT, MAX, DB };
enum ctrl_edge { BOT, TOP };
enum ctrl_vals { RAW, FILT };
float ctrl_ema_alpha[2] = {0.2, 0.1};         // [HOTRC/OTHER] alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1).
int32_t ctrl_lims_adc[2][2][5] =               //   values as adc counts
    {{{0, adcmidscale_adc, adcrange_adc, 62, 100},  // [HOTRC][HORZ][MIN/CENT/MAX/DB/MARGIN]  // MARGIN is how much out of range the reading must be for axis to be completely ignored
      {0, adcmidscale_adc, adcrange_adc, 62, 100}}, // [HOTRC][VERT][MIN/CENT/MAX/DB/MARGIN]
     {{9, adcmidscale_adc, 4085, 50, 100},          // [OTHER][HORZ][MIN/CENT/MAX/DB/MARGIN]
      {9, adcmidscale_adc, 4085, 50, 100}}};        // [OTHER][VERT][MIN/CENT/MAX/DB/MARGIN]
int32_t ctrl_db_adc[2][2];                     // [HORZ/VERT] [BOT/TOP] - to store the top and bottom deadband values for each axis of selected controller
int32_t ctrl_pos_adc[2][2];                    // [HORZ/VERT] [RAW/FILT] - holds most current controller values
bool ctrl = HOTRC;                             // Use HotRC controller to drive instead of joystick?
int32_t hotrc_pulse_lims_us[4][3] = {{970 - 1, 1470 - 5, 1970 - 8},   // [HORZ] [MIN/CENT/MAX]
                                     {1080 - 1, 1580 - 5, 2080 - 8},  // [VERT] [MIN/CENT/MAX]
                                     {1200 - 1, 1500 - 5, 1800 - 8},  // [CH3] [MIN/CENT/MAX]
                                     {1300 - 1, 1500 - 5, 1700 - 8}}; // [CH4] [MIN/CENT/MAX]
int32_t hotrc_pulse_abs_min_us = 880;
int32_t hotrc_pulse_abs_max_us = 2080;
int32_t hotrc_pulse_margin_us = 20;
int32_t hotrc_spike_buffer[2][3];
bool hotrc_radio_lost = true;
bool hotrc_radio_lost_last = hotrc_radio_lost;
bool hotrc_suppress_next_ch3_event = true; // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
bool hotrc_suppress_next_ch4_event = true; // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
float hotrc_pulse_period_us = 1000000.0 / 50;
volatile int32_t hotrc_pulse_us[4];  // [HORZ/VERT/CH3/CH4]  // horz_pulse_us, hotrc_vert_pulse_us, hotrc_horz_pulse_filt_us, hotrc_vert_pulse_filt_us;
int32_t hotrc_pulse_vert_filt_us;  // Only needed for vert channel to detect radio
int32_t hotrc_pulse_failsafe_us = 880; // Hotrc must be configured per the instructions: search for "HotRC Setup Procedure"
int32_t hotrc_pulse_failsafe_margin_us = 100; // in the carpet dumpster file: https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA/edit
int32_t hotrc_pulse_failsafe_pad_us = 10;
uint32_t hotrc_panic_timeout_us = 500000; // how long to receive flameout-range signal from hotrc vertical before panic stopping
Timer hotrcPanicTimer(hotrc_panic_timeout_us);
volatile int64_t hotrc_timer_start;
volatile bool hotrc_ch3_sw, hotrc_ch4_sw, hotrc_ch3_sw_event, hotrc_ch4_sw_event, hotrc_ch3_sw_last, hotrc_ch4_sw_last;
volatile bool hotrc_isr_pin_preread = true;
volatile int64_t hotrc_horz_pulse_64_us = (int64_t)hotrc_pulse_lims_us[HORZ][CENT];
volatile int64_t hotrc_vert_pulse_64_us = (int64_t)hotrc_pulse_lims_us[VERT][CENT];
// volatile int32_t intcount = 0;

// steering related
float steer_safe_percent = 72.0; // Steering is slower at high speed. How strong is this effect
float steer_safe_ratio = steer_safe_percent / 100;
float speedo_safeline_mph;
//
float steer_out_percent, steer_safe_adj_percent;
float steer_right_max_percent = 100.0;
float steer_right_percent = 100.0;
float steer_stop_percent = 0.0;
float steer_left_percent = -100.0;
float steer_left_min_percent = -100.0;
float steer_margin_percent = 2.4;
// float steer_pulse_safe_us = 0;
float steer_pulse_left_min_us = 500;   // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float steer_pulse_left_us = 670;       // Steering pulsewidth corresponding to full-speed right steering (in us). Default setting for jaguar is max 670us
float steer_pulse_stop_us = 1500;      // Steering pulsewidth corresponding to zero steering motor movement (in us)
float steer_pulse_right_us = 2330;     // Steering pulsewidth corresponding to full-speed left steering (in us). Default setting for jaguar is max 2330us
float steer_pulse_right_max_us = 2500; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float steer_pulse_out_us = steer_pulse_stop_us;              // pid loop output to send to the actuator (steering)

// brake pressure related
PressureSensor pressure_sensor(pressure_pin);
float pressure_hold_initial_psi = 45;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
float pressure_hold_increment_psi = 3;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
float pressure_panic_initial_psi = 80; // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
float pressure_panic_increment_psi = 5; // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
float pressure_target_psi;

// brake actuator motor related
Timer brakeIntervalTimer(100000);             // How much time between increasing brake force during auto-stop if car still moving?
int32_t brake_increment_interval_us = 100000; // How often to apply increment during auto-stopping (in us)

float brake_out_percent;
float brake_retract_max_percent = 100.0; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float brake_retract_percent = 100.0;     // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us). Default setting for jaguar is max 670us
float brake_stop_percent = 0.0;          // Brake pulsewidth corresponding to center point where motor movement stops (in us)
float brake_extend_percent = -100.0;     // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us). Default setting for jaguar is max 2330us
float brake_extend_min_percent = -100.0; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float brake_margin_percent = 2.4;        // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated

float brake_pulse_extend_min_us = 670; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float brake_pulse_extend_us = 670;     // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us). Default setting for jaguar is max 670us
float brake_pulse_stop_us = 1500;       // Brake pulsewidth corresponding to center point where motor movement stops (in us)
float brake_pulse_retract_us = 2330;     // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us). Default setting for jaguar is max 2330us
float brake_pulse_retract_max_us = 2330; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float brake_pulse_out_us = brake_pulse_stop_us;               // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
// float brake_motor_govern_percent = 80;   // Artificial limit on how fully the brake motor is powered. Ie what percent of 12V (the spec maximum) shall we consider full power?

// brake actuator position related
BrakePositionSensor brkpos_sensor(brake_pos_pin);

// carspeed/speedo related
Speedometer speedometer(speedo_pulse_pin);
float speedo_target_mph;
float speedo_govern_mph;      // Governor must scale the top vehicle speed proportionally. This is given a value in the loop
float speedo_idle_mph = 4.50; // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)

// throttle servo related
bool reverse_gas_servo = true;
float gas_pulse_out_us = 1501;        // pid loop output to send to the actuator (gas)
float gas_pulse_govern_us = 1502;     // Governor must scale the pulse range proportionally. This is given a value in the loop
Timer gasServoTimer(500000);          // We expect the servo to find any new position within this time
float gas_governor_percent = 95;      // Software governor will only allow this percent of full-open throttle (percent 0-100)
float gas_pulse_cw_min_us = 500;      // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
float gas_pulse_cw_open_us = 718;     // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
float gas_pulse_ccw_closed_us = 2000; // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
float gas_pulse_ccw_max_us = 2500;    // Servo ccw limit pulsewidth. Hotrc controller ch1/2 min(lt/br) = 1000us, center = 1500us, max(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
float gas_pulse_park_slack_us = 30;   // Gas pulsewidth beyond gas_pulse_ccw_closed_us where to park the servo out of the way so we can drive manually (in us)
float gas_pulse_cruise_us = gas_pulse_ccw_closed_us;  // Gas pulsewidth value fixed by cruise mode when in fixed throttle mode 
float gas_pulse_adjustpoint_us;       // Used for adjusting cruise fixed throttle level

// tachometer related
Tachometer tachometer(tach_pulse_pin);
float tach_target_rpm, tach_adjustpoint_rpm;
float tach_govern_rpm;        // Software engine governor creates an artificially reduced maximum for the engine speed. This is given a value in calc_governor()
float tach_margin_rpm = 15.0; // Margin of error for checking engine rpm (in rpm)
// float tach_idle_rpm = 700.0; // Min value for engine hz, corresponding to low idle (in rpm) Note, this value is itself highly variable, dependent on engine temperature
float tach_idle_abs_min_rpm = 450.0;  // Low limit of idle speed adjustability
float tach_idle_hot_min_rpm = 550.0;  // Idle speed at nom_max engine temp
float tach_idle_cold_max_rpm = 775.0; // Idle speed at nom_min engine temp
float tach_idle_high_rpm = 950.0;     // Elevated rpm above idle guaranteed never to stall
float tach_idle_abs_max_rpm = 1000.0; // High limit of idle speed adjustability
Timer tachIdleTimer(5000000);         // How often to update tach idle value based on engine temperature

// airflow sensor related
AirflowSensor airflow_sensor(i2c);

// map sensor related
MAPSensor map_sensor(i2c);

// Motor control:
// Steering : Controls the steering motor proportionally based on the joystick
uint32_t steer_pid_period_us = 75000;    // (Not actually a pid) Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer steerPidTimer(steer_pid_period_us); // not actually tunable, just needs value above

// Brake : Controls the brake motor to achieve the desired brake fluid pressure
uint32_t brake_pid_period_us = 85000;    // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer brakePidTimer(brake_pid_period_us); // not actually tunable, just needs value above
// float brake_perc_per_us = (100.0 - (-100.0)) / (brake_pulse_extend_us - brake_pulse_retract_us);  // (100 - 0) percent / (us-max - us-min) us = 1/8.3 = 0.12 percent/us
float brake_spid_initial_kp = 0.323;                                                                         // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
float brake_spid_initial_ki_hz = 0.000;                                                                      // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
float brake_spid_initial_kd_s = 0.000;                                                                       // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
QPID brakeQPID(pressure_sensor.get_filtered_value_ptr().get(), &brake_out_percent, &pressure_target_psi,     // input, target, output variable references
               brake_extend_percent, brake_retract_percent,                                                  // output min, max
               brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s,                     // Kp, Ki, and Kd tuning constants
               QPID::pMode::pOnError, QPID::dMode::dOnError, QPID::iAwMode::iAwCondition, QPID::Action::direct,  // settings  // iAwRoundCond, iAwClamp
               brake_pid_period_us, QPID::Control::timer, QPID::centMode::center, brake_stop_percent); // period, more settings
               // QPID::pMode::pOnError, QPID::dMode::dOnError, QPID::iAwMode::iAwRound, QPID::Action::direct,  // settings  // iAwRoundCond, iAwClamp
               // brake_pid_period_us, QPID::Control::timer, QPID::centMode::centerStrict, brake_stop_percent); // period, more settings

// Gas : Controls the throttle to achieve the desired intake airflow and engine rpm

ThrottleControl throttle(tachometer.get_human_ptr().get(), tachometer.get_filtered_value_ptr().get(),
                         tach_idle_high_rpm, tach_idle_hot_min_rpm, tach_idle_cold_max_rpm,
                         temp_lims_f[ENGINE][NOM_MIN], temp_lims_f[ENGINE][WARNING],
                         50, ThrottleControl::idlemodes::control);
uint32_t gas_pid_period_us = 22500;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer gasPidTimer(gas_pid_period_us); // not actually tunable, just needs value above
float gas_spid_initial_kp = 0.206;    // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
float gas_spid_initial_ki_hz = 0.000; // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
float gas_spid_initial_kd_s = 0.000;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
bool gas_open_loop = true;
static Servo gas_servo;
QPID gasQPID(tachometer.get_filtered_value_ptr().get(), &gas_pulse_out_us, &tach_target_rpm,                            // input, target, output variable references
             gas_pulse_cw_open_us, gas_pulse_ccw_closed_us,                                                             // output min, max
             gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s,                                        // Kp, Ki, and Kd tuning constants
             QPID::pMode::pOnError, QPID::dMode::dOnError, QPID::iAwMode::iAwClamp, QPID::Action::reverse,              // settings
             gas_pid_period_us, (gas_open_loop) ? QPID::Control::manual : QPID::Control::timer, QPID::centMode::range); // period, more settings

// Cruise : is active on demand while driving. It controls the throttle target to achieve the desired vehicle speed
uint32_t cruise_pid_period_us = 110000;                                                                      // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer cruisePidTimer(cruise_pid_period_us);                                                                  // not actually tunable, just needs value above
float cruise_spid_initial_kp = 5.57;                                                                         // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
float cruise_spid_initial_ki_hz = 0.000;                                                                     // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
float cruise_spid_initial_kd_s = 0.000;                                                                      // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
QPID cruiseQPID(speedometer.get_filtered_value_ptr().get(), &tach_target_rpm, &speedo_target_mph,            // input, target, output variable references
                throttle.get_idlespeed(), tach_govern_rpm,                                                   // output min, max
                cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s,                 // Kp, Ki, and Kd tuning constants
                QPID::pMode::pOnError, QPID::dMode::dOnError, QPID::iAwMode::iAwRound, QPID::Action::direct, // settings
                cruise_pid_period_us, QPID::Control::timer, QPID::centMode::range);
// QPID::centMode::centerStrict, (tach_govern_rpm + tach_idle_rpm)/2);  // period, more settings

// Trouble codes
uint32_t err_timeout_us = 175000;
Timer errTimer((int64_t)err_timeout_us);
uint32_t err_margin_adc = 5;
bool err_temp_engine, err_temp_wheel;
// Sensor related trouble - this all should be moved to devices.h
enum err_types_sensor { LOST, RANGE, num_err_types };
enum err_sensors { e_hrchorz, e_hrcvert, e_hrcch3, e_hrcch4, e_pressure, e_brkpos, e_speedo, e_tach, e_airflow, e_mapsens, e_temps, e_battery, e_starter, e_basicsw, e_num_sensors };
// enum class SimOption : opt_t { none=0, joy, pressure, brkpos, speedo, tach, airflow, mapsens, engtemp, battery, ignition, basicsw, cruisesw, starter, syspower };  // , num_sensors, err_flag };

bool err_sensor_alarm[num_err_types] = { false, false };  // [LOST/RANGE]
bool err_sensor[num_err_types][e_num_sensors]; //  [LOST/RANGE] [e_hrchorz/e_hrcvert/e_hrcch3/e_hrcch4/e_pressure/e_brkpos/e_tach/e_speedo/e_airflow/e_mapsens/e_temps/e_battery/e_basicsw/e_starter]   // SimOption::opt_t::num_sensors]

// Soren: commenting out these pre-rmt horz/vert handler function relics (noticing also the horz function appears to operate on the vert value (?))
// void handle_hotrc_vert(int32_t pulse_width) { if (pulse_width > 0) hotrc_vert_pulse_64_us = pulse_width; }  // reads return 0 if the buffer is empty eg bc our loop is running faster than the rmt is getting pulses;
// void handle_hotrc_horz(int32_t pulse_width) { if (pulse_width > 0) hotrc_vert_pulse_64_us = pulse_width; }
void hotrc_ch3_update(void) {                                                            //
    hotrc_pulse_us[CH3] = hotrc_ch3.readPulseWidth(true);
    hotrc_ch3_sw = (hotrc_pulse_us[CH3] <= 1500); // Ch3 switch true if short pulse, otherwise false  hotrc_pulse_lims_us[CH3][CENT]
    if (hotrc_ch3_sw != hotrc_ch3_sw_last) hotrc_ch3_sw_event = true; // So a handler routine can be signaled. Handler must reset this to false
    hotrc_ch3_sw_last = hotrc_ch3_sw;
}
void hotrc_ch4_update(void) {                                                            //
    hotrc_pulse_us[CH4] = hotrc_ch4.readPulseWidth(true);
    hotrc_ch4_sw = (hotrc_pulse_us[CH4] <= 1500); // Ch4 switch true if short pulse, otherwise false  hotrc_pulse_lims_us[CH4][CENT]
    if (hotrc_ch4_sw != hotrc_ch4_sw_last) hotrc_ch4_sw_event = true; // So a handler routine can be signaled. Handler must reset this to false
    hotrc_ch4_sw_last = hotrc_ch4_sw;
}
void calc_ctrl_lims(void) {
    ctrl_db_adc[VERT][BOT] = ctrl_lims_adc[ctrl][VERT][CENT] - ctrl_lims_adc[ctrl][VERT][DB] / 2; // Lower threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[VERT][TOP] = ctrl_lims_adc[ctrl][VERT][CENT] + ctrl_lims_adc[ctrl][VERT][DB] / 2; // Upper threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][BOT] = ctrl_lims_adc[ctrl][HORZ][CENT] - ctrl_lims_adc[ctrl][HORZ][DB] / 2; // Lower threshold of horz joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][TOP] = ctrl_lims_adc[ctrl][HORZ][CENT] + ctrl_lims_adc[ctrl][HORZ][DB] / 2; // Upper threshold of horz joy deadband (ADC count 0-4095)
    steer_safe_ratio = steer_safe_percent / 100.0;
}
void calc_governor(void) {
    tach_govern_rpm = map(gas_governor_percent, 0.0, 100.0, 0.0, tachometer.get_redline_rpm()); // Create an artificially reduced maximum for the engine speed
    cruiseQPID.SetOutputLimits(throttle.get_idlespeed(), tach_govern_rpm);
    gas_pulse_govern_us = map(tach_govern_rpm, throttle.get_idlespeed(), tachometer.get_redline_rpm(), gas_pulse_ccw_closed_us, gas_pulse_cw_open_us); // Governor must scale the pulse range proportionally
    // gas_pulse_govern_us = map(gas_governor_percent * (tach_govern_rpm - throttle.get_idlespeed()) / tachometer.get_redline_rpm(), 0.0, 100.0, gas_pulse_ccw_closed_us, gas_pulse_cw_open_us); // Governor must scale the pulse range proportionally
    speedo_govern_mph = map(gas_governor_percent, 0.0, 100.0, 0.0, speedometer.get_redline_mph());                                                                                     // Governor must scale the top vehicle speed proportionally
}
float steer_safe(float endpoint) {
    return steer_stop_percent + (endpoint - steer_stop_percent) * (1 - steer_safe_ratio * speedometer.get_filtered_value() / speedometer.get_redline_mph());
}

// int* x is c++ style, int *x is c style
template <typename T>
T adj_val(T variable, T modify, T low_limit, T high_limit) {
    T oldval = variable;
    variable += modify;
    return variable < low_limit ? low_limit : (variable > high_limit ? high_limit : variable);
}
bool adj_val(int32_t *variable, int32_t modify, int32_t low_limit, int32_t high_limit) { // sets an int reference to new val constrained to given range
    int32_t oldval = *variable;
    *variable = adj_val(*variable, modify, low_limit, high_limit);
    return (*variable != oldval);
}
bool adj_val(float *variable, float modify, float low_limit, float high_limit) { // sets an int reference to new val constrained to given range
    float oldval = *variable;
    *variable = adj_val(*variable, modify, low_limit, high_limit);
    return (*variable != oldval);
}
// bool adj_val(uint8_t *variable, int8_t modify, uint8_t low_limit, uint8_t high_limit) { // sets an int reference to new val constrained to given range
//     float oldval = *variable;
//     *variable = adj_val(*variable, modify, low_limit, high_limit);
//     return (*variable != oldval);
// }
bool adj_bool(bool val, int32_t delta) { return delta != 0 ? delta > 0 : val; } // sets a bool reference to 1 on 1 delta or 0 on -1 delta
void adj_bool(bool *val, int32_t delta) { *val = adj_bool(*val, delta); }       // sets a bool reference to 1 on 1 delta or 0 on -1 delta

bool read_battery_ignition() { return battery_sensor.get_filtered_value() > ignition_on_thresh_v; }

bool syspower_set(bool val) {
    bool really_power = keep_system_powered | val;
    write_pin(syspower_pin, really_power); // delay (val * 500);
    return really_power;
}

// tasks for RTOS, this section is currently going to be where we keep sensor update loops.
// TODO move these to more sensible places

// This is the function that will be run in a separate task
void update_temperature_sensors(void *parameter) {
    while (true) {
        if (take_temperatures)
            temperature_sensor_manager.update_temperatures();
        if (simulator.can_simulate(SimOption::engtemp) && simulator.get_pot_overload() == SimOption::engtemp) {
            TemperatureSensor *engine_sensor = temperature_sensor_manager.get_sensor(sensor_location::ENGINE);
            if (engine_sensor != nullptr) {
                engine_sensor->set_temperature(pot.mapToRange(temp_sensor_min_f, temp_sensor_max_f));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for a second to avoid updating the sensors too frequently
    }
}
void set_devboard_defaults() {
    simulator.set_can_simulate(SimOption::pressure, adj_bool(simulator.can_simulate(SimOption::pressure), 1));
    simulator.set_can_simulate(SimOption::brkpos, adj_bool(simulator.can_simulate(SimOption::brkpos), 1));
    simulator.set_can_simulate(SimOption::tach, adj_bool(simulator.can_simulate(SimOption::tach), 1));
    simulator.set_can_simulate(SimOption::speedo, adj_bool(simulator.can_simulate(SimOption::speedo), 1));
    // simulator.set_can_simulate(SimOption::airflow, adj_bool(simulator.can_simulate(SimOption::airflow), 1));
    // simulator.set_can_simulate(SimOption::mapsens, adj_bool(simulator.can_simulate(SimOption::mapsens), 1));
}
