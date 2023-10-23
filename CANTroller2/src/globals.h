#pragma once
#include <ESP32Servo.h>        // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <DallasTemperature.h>
#include <Wire.h>
#include <Preferences.h>
#include <iostream>
#include <iomanip>  // For formatting console cout strings
#include "driver/rmt.h"
#include "RMT_Input.h"
#include "QPID.h" // This is quickpid library except i have to edit some of it
#include "utils.h"
#include "uictrl.h"
#include "neo.h"
#include "map.h"
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
#define spi_mosi_pin 11         // (spi0 mosi / adc2ch0) - Used as spi interface data for touchscreen, sd card and tft screen
#define spi_sclk_pin 12         // (spi0 sclk / adc2ch1) - Used as spi interface clock for touchscreen, sd card and tft screen
#define spi_miso_pin 13         // (spi0 miso / adc2ch2) - Used as spi interface data from sd card and possibly (?) tft screen
#define hotrc_ch2_vert_pin 14   // (pwm0 / adc2ch3) - Hotrc Ch2 bidirectional trigger input
#define hotrc_ch1_horz_pin 15   // (pwm1 / adc2ch4) - Hotrc Ch1 thumb joystick input
#define gas_pwm_pin 16          // (pwm1 / adc2ch5) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
#define brake_pwm_pin 17        // (pwm0 / adc2ch6 / tx1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped)
#define steer_pwm_pin 18        // (pwm0 / adc2ch7 / rx1) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
#define onewire_pin 19          // (usb-d- / adc2ch8) - Onewire bus for temperature sensor data
#define hotrc_ch3_pin 20        // (usb-d+ / adc2ch9) - Ignition control, Hotrc Ch3 PWM toggle signal
#define hotrc_ch4_pin 21        // (pwm0) - Cruise control, Hotrc Ch4 PWM toggle signal
#define speedo_pin 35           // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. (Open collector sensors need pullup)
#define starter_pin 36          // (spi-ram / oct-spi / glitch) - Input/Output (both active high), output when starter is being driven, otherwise input senses external starter activation
#define tach_pin 37             // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup) - Note: placed on p36 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
#define sdcard_cs_pin 38        // (spi-ram / oct-spi) - Output, chip select for SD card controller on SPI bus
#define basicmodesw_pin 39      // (glitch) Input, asserted to tell us to run in basic mode, active low (has ext pullup) - Note: placed on p39 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
#define encoder_b_pin 40        // Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
#define encoder_a_pin 41        // Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
#define encoder_sw_pin 42       // Input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
#define uart_tx_pin 43          // "TX" (uart0 tx) - Serial monitor data out. Also used to detect devboard vs. pcb at boot time (using pullup/pulldown, see below)
#define uart_rx_pin 44          // "RX" (uart0 rx) - Serial monitor data in. Maybe could repurpose during runtime since we only need outgoing console data?
#define ignition_pin 45          // (bootstrap low) - Output for Hotrc to a relay to kill the car ignition. Note, Joystick ign button overrides this if connected and pressed
#define syspower_pin 46         // (bootstrap low) - Output, flips a relay to power all the tranducers. This is actually the neopixel pin on all v1.1 devkit boards.
#define touch_cs_pin 47         // Output, chip select for resistive touchscreen, active low
#define neopixel_pin 48         // (rgb led) - Data line to onboard Neopixel WS281x (on all v1 devkit boards - pin 38 is used on v1.1 boards). Also used for onboard and external neopoxels

#ifdef pinout_bm2023            // Swapped these signals from pins below (bm2023) to above (bm2024)
    #define tach_pin 36   // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup) - Note: placed on p36 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
    #define ignition_pin 37      // (spi-ram / oct-spi) - Output for Hotrc to a relay to kill the car ignition. Note, Joystick ign button overrides this if connected and pressed
    #define syspower_pin 38     // (spi-ram / oct-spi) - Output, flips a relay to power all the tranducers. This is actually the neopixel pin on all v1.1 devkit boards.
    #define starter_pin 45      // (bootstrap low) - Input, active high when vehicle starter is engaged (needs pulldown)
    #define sdcard_cs_pin 46    // (bootstrap low) - Output, chip select for SD card controller on SPI bus,
#endif

// External components needed (pullup/pulldown resistors, capacitors, etc.): (Note: "BB" = On dev breadboards only, "PCB" = On vehicle PCB only)
// 1. brake_pos_pin: Add 1M-ohm to GND. Allows detecting unconnected sensor or broken connection.
// 2. onewire_pin: Add 4.7k-ohm to 3.3V. Needed for open collector sensor output, to define logic-high voltage level.
// 3. tach_pin, speedo_pin: (PCB) Add 4.7k-ohm to 3.3V. For open collector sensor outputs. (BB) If no sensor is present: connect 4.7k-ohm to GND instead. Allows sensor detection.
// 4. neopixel_pin: (PCB) Add 330 ohm in series (between pin and the DataIn pin of the 1st pixel). (BB) Same, but this one is likely optional, e.g. mine works w/o it.  For signal integrity over long wires. 
// 5. uart_tx_pin: (PCB) Add 22k-ohm to GND. (BB) Connect the 22k-ohm to 3.3V instead. For boot detection of vehicle PCB, so defaults are set appropriately.
// 6. ADC inputs (mulebatt_pin, pressure_pin, brake_pos_pin, pot_wipe_pin) should have 100nF cap to gnd, tho it works w/o it.
// 7. encoder_a_pin, encoder_b_pin, encoder_sw_pin: should have 10nF to gnd, tho it should work w/o it. Pullups to 3.3V (4.7k-ohm is good) are also necessary, but the encoder we're using includes these.
// 8. Resistor dividers are needed for these inputs: starter_pin (16V->3.3V), mulebatt_pin (16V->3.3V), and pressure_pin (5V->3.3V).
// 9. ignition_pin, syspower_pin, starter_pin: require pulldowns to gnd, this is provided by nfet gate pulldown.
// 10. gas_pwm_pin: should have a series ~680-ohm R going to the servo.

// ESP32-S3 TRM: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#dma
// ESP32-S3 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
// ESP32-S3 has 5 DMA channels in each direction. We would use them for SPI data out to TFT, Neopixel data out, and possibly out to the 3 motor outputs and in from the 4 hotrc channels.
// DMA works with: RMT, I2S0, I2S1, SPI2, SPI3, ADC, internal RAM, external PSRAM, and a few others (see the TRM)
// Official pin capabilities: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html?highlight=devkitc#user-guide-s3-devkitc-1-v1-1-header-blocks
// ADC ch2 will not work if wifi is enabled
// Bootstrap pins: Pin 0 must be pulled high, and pins 45 and 46 pulled low during bootup
// ESP32 errata 3.11: Pin 36 and 39 will be pulled low for ~80ns when "certain RTC peripherals power up"
// SPI bus page including DMA information: https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/spi_master.html

#define tft_ledk_pin -1   // Output, optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define touch_irq_pin 255 // Input, optional touch occurence interrupt signal (for resistive touchscreen, prevents spi bus delays) - Set to 255 if not used
#define tft_rst_pin -1    // TFT Reset allows us to reboot the screen hardware when it crashes. Otherwise connect screen reset line to esp reset pin
// Globals -------------------
bool starter_signal_support = true;
bool remote_start_support = true;
bool autostop_disabled = false;       // Temporary measure to keep brake behaving until we get it debugged. Eventually should be false
bool allow_rolling_start = false;    // May be a smart prerequisite, may be us putting obstacles in our way

// Dev-board-only options:
// Note these are ignored and set false at boot by set_board_defaults() unless running on a breadboard with a 22k-ohm pullup to 3.3V the TX pin
bool dont_take_temperatures = false;  // In case debugging dallas sensors or causing problems
bool gamma_correct_enabled = false;
bool console_enabled = true;         // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
bool keep_system_powered = false;    // Use true during development
bool screensaver = false;  // Can enable experiment with animated screen draws
bool timestamp_loop = false;         // Makes code write out timestamps throughout loop to serial port
uint32_t timestamp_loop_linefeed_threshold = 0;  // Leaves prints of loops taking > this for analysis. Set to 0 prints every loop

enum hotrc_axis { HORZ, VERT, CH3, CH4 };
enum hotrc_val { MIN, CENT, MAX, RAW, FILT, DBBOT, DBTOP, MARGIN };

// Persistent config storage
Preferences config;

// I2C related
I2C i2c(i2c_sda_pin, i2c_scl_pin);

// Declare Hotrc RMT Inputs in global scope
RMTInput hotrc_rmt[4] = {
    RMTInput(RMT_CHANNEL_4, gpio_num_t(hotrc_ch1_horz_pin)),  // hotrc[HORZ]
    RMTInput(RMT_CHANNEL_5, gpio_num_t(hotrc_ch2_vert_pin)),  // hotrc[VERT]
    RMTInput(RMT_CHANNEL_6, gpio_num_t(hotrc_ch3_pin)),  // hotrc[CH3]
    RMTInput(RMT_CHANNEL_7, gpio_num_t(hotrc_ch4_pin))  // hotrc[CH4]
};

// Globals -------------------
//

// In case we ever talk to jaguars over asynchronous serial port, replace with this:
// #include <HardwareSerial.h>
// HardwareSerial jagPort(1); // Open serisl port to communicate with jaguar controllers for steering & brake motors

// run state machine related
enum runmodes { BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL };
bool running_on_devboard = false;  // will overwrite with value read thru pull resistor on tx pin at boot
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
bool panic_stop_last = false;
Timer panicTimer(20000000);  // How long should a panic stop last?  We can't stay mad forever
// Timer cruiseSwTimer;  // Was used to require a medium-length hold time pushing cruise button to switch modes
Timer sleepInactivityTimer(15000000);           // After shutdown how long to wait before powering down to sleep
Timer stopcarTimer(8000000);                    // Allows code to fail in a sensible way after a delay if nothing is happening
uint32_t motor_park_timeout_us = 4000000;       // If we can't park the motors faster than this, then give up.
uint32_t gesture_flytimeout_us = 1250000;        // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
Timer gestureFlyTimer(gesture_flytimeout_us); // Used to keep track of time for gesturing for going in and out of fly/cruise modes
Timer motorParkTimer(motor_park_timeout_us);
int32_t neobright = 10;  // lets us dim/brighten the neopixels
int32_t neodesat = 0;  // lets us de/saturate the neopixels

// potentiometer related
Potentiometer pot(pot_wipe_pin);

// simulator related
Simulator sim(pot);
bool simulating_last = false;
Timer simTimer; // NOTE: unused
int32_t simdelta = 0;
int32_t simdelta_touch = 0;
int32_t simdelta_encoder = 0;

// calibration related
bool cal_joyvert_brkmotor_mode = false; // Allows direct control of brake motor using controller vert
bool cal_pot_gasservo_mode = false;     // Allows direct control of gas servo using pot. First requires pot to be in valid position before mode is entered
bool cal_pot_gasservo_ready = false;    // Whether pot is in valid range

// diag/monitoring variables
bool diag_ign_error_enabled = true;

// pushbutton related
enum sw_presses { NONE, SHORT, LONG }; // used by encoder sw and button algorithms
bool boot_button_last = 0;
bool boot_button = 0;
bool boot_button_timer_active = false;
bool boot_button_suppress_click = false;
bool boot_button_action = NONE;

// external digital input and output signal related
bool basicmodesw = LOW;

bool ignition = LOW;  // Set by handler only. Reflects current state of the signal
bool ignition_toggle_request = false;  // Setting to true tells handler to toggle the signal. Handler will set to false.
bool syspower = HIGH;  // Set by handler only. Reflects current state of the signal
bool syspower_toggle_request = false;  // Setting to true tells handler to toggle the signal. Handler will set to false.

enum starter_requests { st_nop = -1, st_off = 0, st_on = 1, st_tog = 2 };  // code can set starter_request to these things, handler will make starter do it
bool starter = LOW;  // Set by handler only. Reflects current state of starter signal (does not indicate source)
bool starter_drive = false;  // Set by handler only. High when we're driving starter, otherwise starter is an input
int8_t starter_request = st_nop;  // Code can set for handler to drive the pin. -1 = nop, LOW = turn off starter, HIGH = turn on starter
Timer starterTimer(5000000);  // If remotely-started starting event is left on for this long, end it automatically  

enum temp_categories { AMBIENT = 0, ENGINE = 1, WHEEL = 2 };
enum temp_lims { DISP_MIN, NOM_MIN, NOM_MAX, WARNING, ALARM, DISP_MAX }; // Possible sources of gas, brake, steering commands
float temp_lims_f[3][6]{
    {0.0, 45.0, 115.0, 120.0, 130.0, 220.0},  // [AMBIENT][DISP_MIN/NOM_MIN/NOM_MAX/WARNING/ALARM]
    {0.0, 178.0, 198.0, 202.0, 205.0, 220.0}, // [ENGINE][DISP_MIN/NOM_MIN/NOM_MAX/WARNING/ALARM]
    {0.0, 50.0, 120.0, 130.0, 140.0, 220.0},
};                               // [WHEEL][DISP_MIN/NOM_MIN/NOM_MAX/WARNING/ALARM] (applies to all wheels)
float temp_room = 77.0;          // "Room" temperature is 25 C = 77 F  Who cares?
float temp_sensor_min_f = -67.0; // Minimum reading of sensor is -25 C = -67 F
float temp_sensor_max_f = 257.0; // Maximum reading of sensor is 125 C = 257 F

TemperatureSensorManager tempsens(onewire_pin);

// encoder related
Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);

// mule battery related
BatterySensor mulebatt(mulebatt_pin);

static Servo brakemotor;
static Servo steermotor;
static Servo gas_servo;

// controller related
float hotrc_ema_alpha = 0.075;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1).
float hotrc_pc[2][8];           // values range from -100% to 100% are all derived or auto-assigned
int32_t hotrc_us[4][8] =
    { {  971, 1470, 1968, 0, 1500, 0, 0, 0 },     // 1000-30+1, 1500-30,  2000-30-2   // [HORZ] [MIN/CENT/MAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
      { 1081, 1580, 2078, 0, 1500, 0, 0, 0 },     // 1000+80+1, 1500+80,  2000+80-2,  // [VERT] [MIN/CENT/MAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
      { 1151, 1500, 1848, 0, 1500, 0, 0, 0 },     // 1000+150+1,   1500, 2000-150-2,  // [CH3] [MIN/CENT/MAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
      { 1251, 1500, 1748, 0, 1500, 0, 0, 0 }, };  // 1000+250+1,   1500, 2000-250-2,  // [CH4] [MIN/CENT/MAX/RAW/FILT/DBBOT/DBTOP/MARGIN]
float hotrc_ema_us[2] = { 1500.0, 1500.0 };  // [HORZ/VERT]
int32_t hotrc_absmin_us = 880;
int32_t hotrc_absmax_us = 2080;
int32_t hotrc_deadband_us = 13;  // All [DBBOT] and [DBTOP] values above are derived from this by calling hotrc_calc_params()
int32_t hotrc_margin_us = 20;  // All [MARGIN] values above are derived from this by calling hotrc_calc_params()
int32_t hotrc_failsafe_us = 880; // Hotrc must be configured per the instructions: search for "HotRC Setup Procedure"
int32_t hotrc_failsafe_margin_us = 100; // in the carpet dumpster file: https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA/edit
int32_t hotrc_failsafe_pad_us = 10;
Timer hotrcFailsafeTimer(15000);  // How long to receive failsafe pulse value continuously before recognizing radio is lost. To prevent false positives
bool hotrc_radio_lost = true;
bool hotrc_sw[4] = { 1, 1, 0, 0 };  // index[2]=CH3, index[3]=CH4 and using [0] and [1] indices for LAST values of ch3 and ch4 respectively
bool hotrc_sw_event[4];  // First 2 indices are unused.  What a tragic waste

// steering related
float steer_safe_pc = 72.0; // Steering is slower at high speed. How strong is this effect
float steer_out_pc, steer_safe_adj_pc;
float steer_right_max_pc = 100.0;
float steer_right_pc = 100.0;
float steer_stop_pc = 0.0;
float steer_left_pc = -100.0;
float steer_left_min_pc = -100.0;
float steer_margin_pc = 2.4;
float steer_left_min_us = 500;   // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float steer_left_us = 670;       // Steering pulsewidth corresponding to full-speed right steering (in us). Default setting for jaguar is max 670us
float steer_stop_us = 1500;      // Steering pulsewidth corresponding to zero steering motor movement (in us)
float steer_right_us = 2330;     // Steering pulsewidth corresponding to full-speed left steering (in us). Default setting for jaguar is max 2330us
float steer_right_max_us = 2500; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float steer_out_us = steer_stop_us;              // pid loop output to send to the actuator (steering)

// brake pressure related
PressureSensor pressure(pressure_pin);
float pressure_hold_initial_psi = 45;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
float pressure_hold_increment_psi = 3;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
float pressure_panic_initial_psi = 80; // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
float pressure_panic_increment_psi = 5; // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
float pressure_target_psi;

// brake actuator motor related
Timer brakeIntervalTimer(100000);             // How much time between increasing brake force during auto-stop if car still moving?
int32_t brake_increment_interval_us = 100000; // How often to apply increment during auto-stopping (in us)
float brake_out_pc;
float brake_retract_max_pc = 100.0; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float brake_retract_pc = 100.0;     // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us). Default setting for jaguar is max 670us
float brake_stop_pc = 0.0;          // Brake pulsewidth corresponding to center point where motor movement stops (in us)
float brake_extend_pc = -100.0;     // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us). Default setting for jaguar is max 2330us
float brake_extend_min_pc = -100.0; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float brake_margin_pc = 2.4;        // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated
float brake_extend_min_us = 670; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float brake_extend_us = 670;     // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us). Default setting for jaguar is max 670us
float brake_stop_us = 1500;       // Brake pulsewidth corresponding to center point where motor movement stops (in us)
float brake_retract_us = 2330;     // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us). Default setting for jaguar is max 2330us
float brake_retract_max_us = 2330; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float brake_out_us = brake_stop_us;               // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
float brake_motor_govern_duty_pc = 25;  // From motor datasheet
float brake_retract_effective_max_us;   // 

// brake actuator position related
BrakePositionSensor brakepos(brake_pos_pin);

// carspeed/speedo related
Speedometer speedo(speedo_pin);
float speedo_target_mph;
float speedo_govern_mph;      // Governor must scale the top vehicle speed proportionally. This is given a value in the loop
float speedo_idle_mph = 4.50; // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)

// throttle servo related
float gas_governor_pc = 95;      // Software governor will only allow this percent of full-open throttle (percent 0-100)
Timer gasServoTimer(500000);          // We expect the servo to find any new position within this time
float gas_adjustpoint_us;       // Used for adjusting cruise fixed throttle level

bool reverse_gas_servo = true;
float gas_out_us = 1501;        // pid loop output to send to the actuator (gas)
float gas_govern_us = 1502;     // Governor must scale the pulse range proportionally. This is given a value in the loop
float gas_cw_min_us = 500;      // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
float gas_cw_open_us = 718;     // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
float gas_ccw_closed_us = 2000; // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
float gas_ccw_max_us = 2500;    // Servo ccw limit pulsewidth. Hotrc controller ch1/2 min(lt/br) = 1000us, center = 1500us, max(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
float gas_park_slack_us = 30;   // Gas pulsewidth beyond gas_ccw_closed_us where to park the servo out of the way so we can drive manually (in us)

// Misguided beginnings of attempt to restructure pwm variables above
// enum pwm_val { OUT=3, GOV=4, ABSMIN=5, ABSMAX=6, PARK=7, num_pwmvals=8 };  // { MIN=0, CENT=1, MAX=2 }
// enum pwm_opts { REVERSED, CENTERED, GOVERNED, num_pwmopts };  // Forward is considered ccw < cw motion
// float gas_angle_pc[num_pwmvals];
// float gas_angle_us[num_pwmvals] = { 718, 1500, 2000, 2000, 718, 500, 2500, 2030 };
// bool gas_angle_opt[num_pwmopts] = { true, false, true };

// tach related
Tachometer tach(tach_pin);
float tach_target_rpm, tach_adjustpoint_rpm, tach_govern_rpm;        // Software engine governor creates an artificially reduced maximum for the engine speed. This is given a value in calc_governor()
float tach_margin_rpm = 15.0; // Margin of error for checking engine rpm (in rpm)
float tach_idle_abs_min_rpm = 450.0;  // Low limit of idle speed adjustability
float tach_idle_hot_min_rpm = 550.0;  // Idle speed at nom_max engine temp
float tach_idle_cold_max_rpm = 775.0; // Idle speed at nom_min engine temp
float tach_idle_high_rpm = 950.0;     // Elevated rpm above idle guaranteed never to stall
float tach_idle_abs_max_rpm = 1000.0; // High limit of idle speed adjustability
Timer tachIdleTimer(5000000);         // How often to update tach idle value based on engine temperature

// i2c sensor related
AirflowSensor airflow(i2c);
MAPSensor mapsens(i2c);  // map sensor related

// Motor control:
// Steering : Controls the steering motor proportionally based on the joystick
uint32_t steer_pid_period_us = 75000;    // (Not actually a pid) Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer steerPidTimer(steer_pid_period_us); // not actually tunable, just needs value above

// Brake : Controls the brake motor to achieve the desired brake fluid pressure
uint32_t brake_pid_period_us = 85000;    // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer brakePidTimer(brake_pid_period_us); // not actually tunable, just needs value above
// float brake_perc_per_us = (100.0 - (-100.0)) / (brake_extend_us - brake_retract_us);  // (100 - 0) percent / (us-max - us-min) us = 1/8.3 = 0.12 percent/us
float brake_spid_initial_kp = 0.323;                                                                         // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
float brake_spid_initial_ki_hz = 0.000;                                                                      // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
float brake_spid_initial_kd_s = 0.000;                                                                       // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)

QPID brake_pid(pressure.filt_ptr(), &brake_out_pc, &pressure_target_psi,     // input, target, output variable references
               brake_extend_pc, brake_retract_pc,                                                  // output min, max
               brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s,                     // Kp, Ki, and Kd tuning constants
               QPID::Pmode::onerr, QPID::Dmode::onerr, QPID::Awmode::cond, QPID::Dir::direct,  // settings  // roundcond, clamp
               brake_pid_period_us, QPID::Control::manual, QPID::Centmode::on, brake_stop_pc); // period, more settings
               // QPID::Pmode::onerr, QPID::Dmode::onerr, QPID::Awmode::round, QPID::Dir::direct,  // settings  // roundcond, clamp
               // brake_pid_period_us, QPID::Control::manual, QPID::Centmode::strict, brake_stop_pc); // period, more settings

// Gas : Controls the throttle to achieve the desired intake airflow and engine rpm

ThrottleControl throttle(tach.human_ptr(), tach.filt_ptr(),
                         tach_idle_high_rpm, tach_idle_hot_min_rpm, tach_idle_cold_max_rpm,
                         temp_lims_f[ENGINE][NOM_MIN], temp_lims_f[ENGINE][WARNING],
                         50, ThrottleControl::idlemodes::control);
uint32_t gas_pid_period_us = 22500;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer gasPidTimer(gas_pid_period_us); // not actually tunable, just needs value above
float gas_spid_initial_kp = 0.206;    // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
float gas_spid_initial_ki_hz = 0.000; // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
float gas_spid_initial_kd_s = 0.000;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
bool gas_open_loop = true;
QPID gas_pid(tach.filt_ptr(), &gas_out_us, &tach_target_rpm,                            // input, target, output variable references
             gas_cw_open_us, gas_ccw_closed_us,                                                             // output min, max
             gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s,                                        // Kp, Ki, and Kd tuning constants
             QPID::Pmode::onerr, QPID::Dmode::onerr, QPID::Awmode::clamp, QPID::Dir::reverse,              // settings
             gas_pid_period_us, (gas_open_loop) ? QPID::Control::manual : QPID::Control::manual, QPID::Centmode::off); // period, more settings

// Cruise : is active on demand while driving.
// Pick from 3 different styles of adjusting cruise setpoint. I prefer throttle_delta.
// pid_suspend_fly : (PID) Moving trigger from center disables pid and lets you adjust the rpm target directly like Fly mode does. Whatever speed you're at when trigger releases is new pid target  
// throttle_angle : Cruise locks throttle angle, instead of pid. Moving trigger from center adjusts setpoint proportional to how far you push it before releasing (and reduced by an attenuation factor)
// throttle_delta : Cruise locks throttle angle, instead of pid. Any non-center trigger position continuously adjusts setpoint proportional to how far it's pulled over time (up to a specified maximum rate)
enum cruise_modes { pid_suspend_fly, throttle_angle, throttle_delta };
cruise_modes cruise_setpoint_mode = throttle_delta;
bool cruise_speed_lowerable = true;  // Allows use of trigger to adjust cruise speed target without leaving cruise mode.  Otherwise cruise button is a "lock" button, and trigger activity cancels lock
bool cruise_adjusting = false;
int32_t cruise_delta_max_us_per_s = 200;  // (in throttle_delta mode) What's the fastest rate cruise adjustment can change pulse width (in us per second)
float cruise_angle_attenuator = 0.25;  // (in throttle_angle mode) Limits the change of each adjust trigger pull to this fraction of what's possible
bool flycruise_toggle_request = false;
float flycruise_vert_margin_pc = 0.3; // Margin of error for determining hard brake value for dropping out of cruise mode
float gas_cruise_us;  // Gas pulsewidth value fixed by cruise mode when in fixed throttle mode 

uint32_t cruise_pid_period_us = 85000;                                                                      // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer cruisePidTimer(cruise_pid_period_us);                                                                  // not actually tunable, just needs value above
float cruise_spid_initial_kp = 5.57;                                                                         // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
float cruise_spid_initial_ki_hz = 0.000;                                                                     // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
float cruise_spid_initial_kd_s = 0.000;                                                                      // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
QPID cruise_pid(speedo.filt_ptr(), &tach_target_rpm, &speedo_target_mph,            // input, target, output variable references
                throttle.idlespeed(), tach_govern_rpm,                                                   // output min, max
                cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s,                 // Kp, Ki, and Kd tuning constants
                QPID::Pmode::onerr, QPID::Dmode::onerr, QPID::Awmode::round, QPID::Dir::direct, // settings
                cruise_pid_period_us, QPID::Control::manual, QPID::Centmode::off);
// QPID::centmode::centerStrict, (tach_govern_rpm + tach_idle_rpm)/2);  // period, more settings

void hotrc_toggle_update(int8_t chan) {                                                            //
    hotrc_us[chan][RAW] = hotrc_rmt[chan].readPulseWidth(true);
    hotrc_sw[chan] = (hotrc_us[chan][RAW] <= hotrc_us[chan][CENT]); // Ch3 switch true if short pulse, otherwise false  hotrc_us[CH3][CENT]
    if (hotrc_sw[chan] != hotrc_sw[chan-2]) hotrc_sw_event[chan] = true; // So a handler routine can be signaled. Handler must reset this to false
    hotrc_sw[chan-2] = hotrc_sw[chan];  // chan-2 index being used to store previous values of index chan
}
float hotrc_us_to_pc(int32_t us) {  // float hotrc_us_to_pc(int8_t axis, int32_t us) {
    return (float)us * (hotrc_pc[VERT][MAX] - hotrc_pc[VERT][MIN]) / (float)(hotrc_us[VERT][MAX] - hotrc_us[VERT][MIN]);
    // if (us >= hotrc_us[axis][CENT])
    //     return map((float)us, (float)hotrc_us[axis][CENT], (float)hotrc_us[axis][MAX], hotrc_pc[axis][CENT], hotrc_pc[axis][MAX]);
    // else return map((float)us, (float)hotrc_us[axis][CENT], (float)hotrc_us[axis][MIN], hotrc_pc[axis][CENT], hotrc_pc[axis][MIN]);
}
void hotrc_calc_params() {
    for (int8_t axis=HORZ; axis<=VERT; axis++) {
        hotrc_us[axis][DBBOT] = hotrc_us[axis][CENT] - hotrc_deadband_us;
        hotrc_us[axis][DBTOP] = hotrc_us[axis][CENT] + hotrc_deadband_us;
        hotrc_us[axis][MARGIN] = hotrc_margin_us;
        hotrc_pc[axis][MIN] = -100.0;
        hotrc_pc[axis][CENT] = 0.0;
        hotrc_pc[axis][MAX] = 100.0;
        hotrc_pc[axis][DBBOT] = hotrc_pc[axis][CENT] - hotrc_us_to_pc(hotrc_deadband_us);  // hotrc_us_to_pc(axis, hotrc_deadband_us);
        hotrc_pc[axis][DBTOP] = hotrc_pc[axis][CENT] + hotrc_us_to_pc(hotrc_deadband_us);  // hotrc_us_to_pc(axis, hotrc_deadband_us);
        hotrc_pc[axis][MARGIN] = hotrc_us_to_pc(hotrc_margin_us);  // hotrc_us_to_pc(axis, hotrc_margin_us);
    }
}
void calc_governor(void) {
    tach_govern_rpm = map(gas_governor_pc, 0.0, 100.0, 0.0, tach.redline_rpm()); // Create an artificially reduced maximum for the engine speed
    cruise_pid.set_outlimits(throttle.idlespeed(), tach_govern_rpm);
    gas_govern_us = map(tach_govern_rpm, throttle.idlespeed(), tach.redline_rpm(), gas_ccw_closed_us, gas_cw_open_us); // Governor must scale the pulse range proportionally
    speedo_govern_mph = map(gas_governor_pc, 0.0, 100.0, 0.0, speedo.redline_mph());                                                                                     // Governor must scale the top vehicle speed proportionally
}
float steer_safe(float endpoint) {
    return steer_stop_pc + (endpoint - steer_stop_pc) * (1.0 - steer_safe_pc * speedo.filt() / (100.0 * speedo.redline_mph()));
}

enum joydirs { joy_down = -1, joy_cent = 0, joy_up = 1 };
joydirs get_joydir() {
    return (hotrc_pc[VERT][FILT] > hotrc_pc[VERT][DBTOP]) ? joy_up : ((hotrc_pc[VERT][FILT] < hotrc_pc[VERT][DBBOT]) ? joy_down : joy_cent);
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
bool adj_bool(bool val, int32_t delta) { return delta != 0 ? delta > 0 : val; } // returns 1 on delta=1, 0 on delta=-1, or val on delta=0
void adj_bool(bool *val, int32_t delta) { *val = adj_bool(*val, delta); }       // sets a bool reference to 1 on 1 delta or 0 on -1 delta

// tasks for RTOS, this section is currently going to be where we keep sensor update loops.
// TODO move these to more sensible places

// This is the function that will be run in a separate task
void update_temperature_sensors(void *parameter) {
    while (true) {
        if (!dont_take_temperatures)
            tempsens.update_temperatures();
        if (sim.potmapping(sensor::engtemp)) {
            TemperatureSensor *engine_sensor = tempsens.get_sensor(location::engine);
            if (engine_sensor != nullptr) {
                engine_sensor->set_temperature(pot.mapToRange(temp_sensor_min_f, temp_sensor_max_f));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for a second to avoid updating the sensors too frequently
    }
}

void set_board_defaults(bool devboard) {  // true for dev boards, false for printed board (on the car)
    if (devboard) {
        sim.set_can_sim(sensor::pressure, true);
        sim.set_can_sim(sensor::brkpos, true);
        sim.set_can_sim(sensor::tach, true);
        sim.set_can_sim(sensor::speedo, true);
    }
    else {
        gamma_correct_enabled = false;
        console_enabled = false;     // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
        keep_system_powered = false; // Use true during development
        screensaver = false;         // Can enable experiment with animated screen draws
        timestamp_loop = false;      // Makes code write out timestamps throughout loop to serial port
        dont_take_temperatures = false;
    }
}
// Starter bidirectional handler logic.  Outside code interacts with handler by setting starter_request = st_off, st_on, or st_tog    
void starter_update () {
    if (starter_signal_support) {
        if (starter_request == st_tog) starter_request = !starter_drive;  // translate toggle request to a drive request opposite to the current drive state
        if (starter_drive && ((starter_request == st_off) || starterTimer.expired())) {  // If we're driving the motor but need to stop
            starter_drive = false;
            set_pin (starter_pin, INPUT_PULLDOWN);  // we never assert low on the pin, just set pin as input and let the pulldown bring it low
        }
        if (!starter_drive && (starter_request != st_on) && !sim.simulating(sensor::starter)) {  // If we haven't been and shouldn't be driving, and not simulating
            do {
                starter = digitalRead(starter_pin);  // then read the pin, starter variable will store if starter is turned on externally
            } while (starter != digitalRead(starter_pin)); // starter pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
        }
        else if (!starter && (starter_request == st_on) && remote_start_support) {  // If we got a request to start the motor, and it's not already being driven externally
            starter_drive = true;
            starter = HIGH;
            set_pin (starter_pin, OUTPUT);  // then set pin to an output
            write_pin (starter_pin, starter);  // and start the motor
            starterTimer.reset();  // if left on the starter will turn off automatically after X seconds
        }
        starter_request = st_nop;  // we have serviced whatever requests
    }
    else starter = LOW;
}
void basicsw_update() {
    if (!sim.simulating(sensor::basicsw)) {  // Basic Mode switch
        do {
            basicmodesw = !digitalRead(basicmodesw_pin);   // !value because electrical signal is active low
        } while (basicmodesw != !digitalRead(basicmodesw_pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
    }
}
Timer dispResetButtonTimer (500000);  // How long to press esp32 "boot" button before screen will reset and redraw
void bootbutton_update() {
    // ESP32 "boot" button. generates boot_button_action flags of LONG or SHORT presses which can be handled wherever. Handler must reset boot_button_action = NONE
    if (!read_pin (bootbutton_pin)) {
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
}
Timer loopTimer(1000000); // how long the previous main loop took to run (in us)
float looptime_sum_s;
int32_t loopno = 1;
uint32_t looptimes_us[20];
bool loop_dirty[20];
int32_t loopindex = 0;
int64_t looptime_cout_mark_us;
uint32_t looptime_cout_us, looptime_peak_us;
uint32_t loop_maxloop_us = 100;
const uint32_t loop_history = 15;
uint32_t loop_periods_us[loop_history];
uint32_t loop_now = 0;
float looptime_avg_us, loopfreq_hz;
std::vector<std::string> loop_names(20);

void looptiming_init() {  // Run once at end of setup()
    if (timestamp_loop) {
        for (int32_t x=1; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
        loop_names[0] = std::string("top");
        loop_dirty[0] = false;
        loopindex = 1;
        looptimes_us[0] = esp_timer_get_time();
    }
    loopTimer.reset();  // start timer to measure the first loop
}
void loop_marktime(std::string loopname = std::string("")) {  // Add these throughout loop with short names
    if (timestamp_loop) {
        if (loop_dirty[loopindex]) {
            loop_names[loopindex] = loopname;  // names[index], name);
            loop_dirty[loopindex] = false;
        }
        looptimes_us[loopindex] = esp_timer_get_time();
        loopindex++;
    }
}
void loop_print_timing() {  // Call once each loop at the very end
    loop_periods_us[loop_now] = (uint32_t)loopTimer.elapsed();  // us since beginning of this loop
    loopTimer.reset();
    looptime_sum_s += (float)loop_periods_us[loop_now] / 1000000;
    ema_filt(loop_periods_us[loop_now], &looptime_avg_us, 0.05);
    if (looptime_avg_us > 1) loopfreq_hz = 1000000/looptime_avg_us;
    if (loopno > 10 && loop_periods_us[loop_now] > loop_maxloop_us) loop_maxloop_us = (float)loop_periods_us[loop_now];
    looptime_peak_us = 0;
    for (int8_t i=0; i<loop_history; i++) if (looptime_peak_us < loop_periods_us[i]) looptime_peak_us = loop_periods_us[i]; 
    if (timestamp_loop) {
        looptime_cout_mark_us = esp_timer_get_time();
        std::cout << std::fixed << std::setprecision(0);
        std::cout << "\r" << (uint32_t)looptime_sum_s << "s #" << loopno;  //  << " av:" << std::setw(5) << (int32_t)(looptime_avg_us);  //  << " av:" << std::setw(3) << looptime_avg_ms 
        std::cout << " : " << std::setw(5) << loop_periods_us[loop_now] << " (" << loop_periods_us[loop_now]-looptime_cout_us << ")us ";  // << " avg:" << looptime_avg_us;  //  " us:" << esp_timer_get_time() << 
        for (int32_t x=1; x<loopindex; x++)
            std::cout << std::setw(3) << loop_names[x] << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1] << " ";
        std::cout << " cout:" << std::setw(5) << looptime_cout_us;
        if (loop_periods_us[loop_now]-looptime_cout_us > timestamp_loop_linefeed_threshold || !timestamp_loop_linefeed_threshold) std::cout << std::endl;
        looptime_cout_us = (uint32_t)(esp_timer_get_time() - looptime_cout_mark_us);
        loopindex = 0;
        loop_marktime ("top");
    }
    ++loop_now %= loop_history;
    loopno++;  // I like to count how many loops
}

// Trouble codes
uint32_t err_timeout_us = 175000;
Timer errTimer((int64_t)err_timeout_us);
uint32_t err_margin_adc = 5;
bool err_temp_engine, err_temp_wheel;
// Sensor related trouble - this all should be moved to devices.h
enum err_type { LOST, RANGE, CALIB, WARN, CRIT, INFO, num_err_types };
enum err_sensor { e_hrcvert, e_hrcch3, e_pressure, e_brkpos, e_speedo, e_hrchorz, e_tach, e_temps, e_starter, e_hrcch4, e_basicsw, e_mulebatt, e_airflow, e_mapsens, e_num_sensors, e_none };  // these are in order of priority
char err_type_card[num_err_types][5] = { "Lost", "Rang", "Cal", "Warn", "Crit", "Info" };
char err_sensor_card[e_num_sensors+1][7] = { "HrcV", "HrcCh3", "BrPres", "BrkPos", "Speedo", "HrcH", "Tach", "Temps", "Startr", "HrcCh4", "Basic", "Batery", "Airflw", "MAP", "None" };
// enum class sensor : opt_t { none=0, joy, pressure, brkpos, speedo, tach, airflow, mapsens, engtemp, mulebatt, ignition, basicsw, cruisesw, starter, syspower };  // , num_sensors, err_flag };
bool err_sensor_alarm[num_err_types] = { false, false, false, false, false, false };
int8_t err_sensor_fails[num_err_types] = { 0, 0, 0, 0, 0, 0 };
bool err_sensor[num_err_types][e_num_sensors]; //  [LOST/RANGE] [e_hrchorz/e_hrcvert/e_hrcch3/e_hrcch4/e_pressure/e_brkpos/e_tach/e_speedo/e_airflow/e_mapsens/e_temps/e_mulebatt/e_basicsw/e_starter]   // sensor::opt_t::num_sensors]
uint8_t highest_pri_failing_sensor[num_err_types];
uint8_t highest_pri_failing_last[num_err_types];
void detect_errors() {
    if (errTimer.expireset()) {

        // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
        // this is one approach
        // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
        // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
        // retreive with an OBD tool. Some checks are below, along with other possible things to check for:
        if (!ignition && !tach.engine_stopped()) {  // Check: if engine is turning when ignition signal is off
            if (diag_ign_error_enabled) { // See if the engine is turning despite the ignition being off
                Serial.println (F("Detected engine rotation in the absense of ignition signal"));  // , tach_filt_rpm, ignition
                diag_ign_error_enabled = false;  // Prevents endless error reporting the same error
            }
        }
        else diag_ign_error_enabled = true;

        // different approach
        if (!tempsens.detected(location::ambient)) err_sensor[LOST][e_temps] = true;
        if (!tempsens.detected(location::engine)) err_sensor[LOST][e_temps] = true;
        else if (tempsens.val(location::engine) >= temp_lims_f[ENGINE][WARNING]) err_temp_engine = true;
        bool check_wheels;
        check_wheels = false;
        if (!tempsens.detected(location::wheel_fl)) err_sensor[LOST][e_temps] = true;
        else if (tempsens.val(location::wheel_fl) >= temp_lims_f[WHEEL][WARNING]) check_wheels = true;
        if (!tempsens.detected(location::wheel_fr)) err_sensor[LOST][e_temps] = true;
        else if (tempsens.val(location::wheel_fr) >= temp_lims_f[WHEEL][WARNING]) check_wheels = true;
        if (!tempsens.detected(location::wheel_rl)) err_sensor[LOST][e_temps] = true;
        else if (tempsens.val(location::wheel_rl) >= temp_lims_f[WHEEL][WARNING]) check_wheels = true;
        if (!tempsens.detected(location::wheel_rr)) err_sensor[LOST][e_temps] = true;
        else if (tempsens.val(location::wheel_rr) >= temp_lims_f[WHEEL][WARNING]) check_wheels = true;
        err_temp_wheel = check_wheels;

        // Detect sensors disconnected or giving out-of-range readings.
        // TODO : The logic of this for each sensor should be moved to devices.h objects
        uint32_t val;
        val = analogRead(brake_pos_pin);
        err_sensor[RANGE][e_brkpos] = (val < brakepos.min_native() || val > brakepos.max_native());
        err_sensor[LOST][e_brkpos] = (val < err_margin_adc);
        val = analogRead(pressure_pin);
        err_sensor[RANGE][e_pressure] = ((val && val < pressure.min_native()) || val > pressure.max_native());
        err_sensor[LOST][e_pressure] = (val < err_margin_adc);
        err_sensor[LOST][e_mulebatt] = (analogRead(mulebatt_pin) > mulebatt.max_native());
        for (int32_t ch = HORZ; ch <= CH4; ch++) {  // Hack: This loop depends on the indices for hotrc channel enums matching indices of hotrc sensor errors
            err_sensor[RANGE][ch] = !hotrc_radio_lost && ((hotrc_us[ch][RAW] < hotrc_us[ch][MIN] - (hotrc_us[ch][MARGIN] >> 1)) 
                                    || (hotrc_us[ch][RAW] > hotrc_us[ch][MAX] + (hotrc_us[ch][MARGIN] >> 1)));  // && ch != VERT
            err_sensor[LOST][ch] = !hotrc_radio_lost && ((hotrc_us[ch][RAW] < (hotrc_absmin_us - hotrc_us[ch][MARGIN]))
                                    || (hotrc_us[ch][RAW] > (hotrc_absmax_us + hotrc_us[ch][MARGIN])));
        }
        // err_sensor[RANGE][e_hrcvert] = (hotrc_us[VERT][RAW] < hotrc_failsafe_us - hotrc_us[ch][MARGIN])
        //     || ((hotrc_us[VERT][RAW] < hotrc_us[VERT][MIN] - halfmargin) && (hotrc_us[VERT][RAW] > hotrc_failsafe_us + hotrc_us[ch][MARGIN]));
        
        // Set sensor error idiot light flags
        // printf ("Sensors errors: ");
        
        // printf ("Sensor check: ");
        for (int32_t t=LOST; t<=RANGE; t++) {
            highest_pri_failing_sensor[t] = e_none;
            err_sensor_alarm[t] = false;
            err_sensor_fails[t] = 0;
            for (int32_t s=0; s<e_num_sensors; s++)
                if (err_sensor[t][s]) {
                    if (highest_pri_failing_sensor[t] = e_none) highest_pri_failing_sensor[t] = s;
                    err_sensor_alarm[t] = true;
                    err_sensor_fails[t]++;
                }
        }
        // printf ("\n");

        // Detectable transducer-related failures :: How we can detect them
        // Brakes:
        // * Pressure sensor, chain linkage, or vehicle brakes problem :: Motor retracted with position below zeropoint, but pressure did not increase.
        // * Pressure sensor zero point miscalibration (no force on pedal) :: Minimum pressure reading since startup has never reached 0 PSI or less (cal is too high), or, is more than a given margin below 0. * Note this can also be an auto-calibration approach
        // * Pressure sensor max point miscalibration (full force on pedal) :: When target set to max pressure, after motor moves to the point position isn't changing, the pressure reading deviates from max setting by more than a given margin. * Note this can also be an auto-calibration approach
        // * Position sensor problem :: When pressure is not near max, motor is driven more than X volt-seconds without position change (of the expected polarity).
        // * Brake motor problem :: When motor is driven more than X volt-seconds without any change (of the expected polarity) to either position or pressure.
        // * Brake calibration, idle high, or speedo sensor problem :: Motor retracted to near limit, with position decreased and pressure increased as expected, but speed doesn't settle toward 0.
        // * Pressure sensor problem :: If pressure reading is out of range, or ever changes in the unexpected direction during motor movement.
        // * Position sensor or limit switch problem :: If position reading is outside the range of the motor limit switches.
        // Steering:
        // * Chain derailment or motor or limit switch problem :: Motor told to drive for beyond X volt-seconds in one direction for > Y seconds.
        // Throttle/Engine:
        // * Airflow/MAP/tach sensor failure :: If any of these three sensor readings are out of range to the other two.
        // Tach/Speedo:
        // * Sensor read problem :: Derivative of consecutive readings (rate of change) spikes higher than it's possible for the physical rotation to change - (indicates missing pulses)
        // * Disconnected/problematic speed sensor :: ignition is on, tach is nonzero, and runmode = hold/fly/cruise, yet speed is zero. Or, throttle is at idle and brake pressure high for enough time, yet speed readings are nonzero
        // * Disconnected/problematic tach sensor :: runmode is hold/fly/cruise, ignition is on and speed increases, but tach is below idle speed 
        // Temperature:
        // * Engine temperature sensor problem :: Over X min elapsed with Ignition on and tach >= low_idle, but engine temp is below nominal warmup temp.
        // * Cooling system, coolant, fan, thermostat, or coolant sensor problem :: Engine temp stays over ~204 for >= X min without coolant temp dropping due to fan.
        // * Axle, brake, etc. wheel issue or wheel sensor problem :: The hottest wheel temp is >= X degF hotter than the 2nd hottest wheel.
        // * Axle, brake, etc. wheel issue or wheel/ambient sensor problem :: A wheel temp >= X degF higher than ambient temp.
        // * Ignition problem, fire alarm, or temp sensor problem :: Ignition is off but a non-ambient temp reading increases to above ambient temp.
        // Airflow:
        // * Air filter clogged, or carburetor problem :: Track ratio of airflow/throttle angle whenever throttle is constant. Then, if that ratio lowers over time by X below that level, indicates restricted air. 
        // Battery:
        // * Battery low :: Mulebatt readings average is below a given threshold
        // * Inadequate charging :: Mulebatt readings average has decreased over long time period
        // 
        // More ideas to define better and implement:
        // * Check if the pressure response is characteristic of air being in the brake line.
        // * Axle/brake drum may be going bad (increased engine RPM needed to achieve certain speedo)  (beware going up hill may look the same).
        // * E-brake has been left on (much the same symptoms as above? (beware going up hill may look the same) 
        // * Carburetor not behaving (or air filter is clogged). (See above about engine deiseling - we can detect this!)
        // * After increasing braking, the actuator position changes in the opposite direction, or vise versa.
        // * Changing an actuator is not having the expected effect.
        // * A tunable value suspected to be out of tune.
        // * Check regularly for ridiculous shit. Compare our variable values against a predetermined list of conditions which shouldn't be possible or are at least very suspect. For example:
        //   A) Sensor reading is out of range, or has changed faster than it ever should.
        //   B) Stopping the car on entering hold/shutdown mode is taking longer than it ever should.
        //   C) Mule seems to be accelerating like a Tesla.
        //   D) Car is accelerating yet engine is at idle.
        // * The control system has nonsensical values in its variables.
    }
}
void err_print_info() {
    for (int32_t t=LOST; t<=INFO; t++) {
        printf ("diag err: %s (%d): ", err_type_card[t], err_sensor_fails[t]);
        for (int32_t s=0; s<=e_num_sensors; s++) {
            if (s == e_num_sensors) s++;
            if (err_sensor[t][s]) printf ("%s, ", err_sensor_card[s]);
        }
        printf("\n");
    }
}
// Calculates massairflow in g/s using values passed in if present, otherwise it reads fresh values
float massairflow(float _map = NAN, float _airflow = NAN, float _ambient = NAN) {  // mdot (kg/s) = density (kg/m3) * v (m/s) * A (m2) .  And density = P/RT.  So,   mdot = v * A * P / (R * T)  in kg/s
    float temp = _ambient;
    if (std::isnan(_ambient)) {
        temp = tempsens.val(location::ambient);
        if (std::isnan(temp) && running_on_devboard) temp = tempsens.val(location::engine);
        if (std::isnan(temp)) return -1;  // Avoid crashing due to trying to read absent sensor
    }
    float T = 0.556 * (temp - 32.0) + 273.15;  // in K.  This converts from degF to K
    float R = 287.1;  // R (for air) in J/(kg·K) ( equivalent to 8.314 J/(mol·K) )  1 J = 1 kg*m2/s2
    float v = 0.447 * std::isnan(_airflow) ? airflow.filt() : _airflow; // in m/s   1609.34 m/mi * 1/3600 hr/s = 0.447
    float Ain2 = 3.1415926;  // in in2    1.0^2 in2 * pi  // will still need to divide by 1550 in2/m2
    float P = 6894.76 * std::isnan(_map) ? mapsens.filt() : _map;  // in Pa   6894.76 Pa/PSI  1 Pa = 1 J/m3
    return v * Ain2 * P * (1000000000.0 / (R * T * 1550));  // mass air flow in micrograms per second (ug/s)   (1B ug/kg * m/s * in2 * J/m3) / (J/(kg*K) * K * 1550 in2/m2) = ug/s
}
float maf_ugps;  // Manifold mass airflow in micrograms per second
float maf_min_ugps = 0.0;
float maf_max_ugps = massairflow(mapsens.max_psi(), airflow.max_mph(), temp_lims_f[AMBIENT][DISP_MIN]);
