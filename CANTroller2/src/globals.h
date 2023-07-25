#ifndef GLOBALS_H
#define GLOBALS_H
#include <SdFat.h>  // SD card & FAT filesystem library
#include <ESP32Servo.h>  // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <Adafruit_NeoPixel.h>  // Plan to allow control of neopixel LED onboard the esp32
#include <OneWire.h>
#include "temp.h"
#include <Wire.h>
#include <SparkFun_FS3000_Arduino_Library.h>  // For airflow sensor  http://librarymanager/All#SparkFun_FS3000
#include <Preferences.h>
#include <stdio.h>
#include <iostream>
// #include <DallasTemperature.h>
// #include <stdio.h>  // MCPWM pulse measurement code
// #include "freertos/FreeRTOS.h"  // MCPWM pulse measurement code
// #include "freertos/task.h"  // MCPWM pulse measurement code
// #include "driver/mcpwm.h"  // MCPWM pulse measurement code
#include "qpid.h"
#include "driver/rmt.h"
#include "RMT_Input.h"

// #define CAP_TOUCH
bool flip_the_screen = false;

// Here are the different runmodes documented
//
// ** Basic Mode **
// - Required: BasicMode switch On
// - Priority: 1 (Highest)
// The gas and brake don't do anything in Basic Mode. Just the steering works, so use the pedals.
// This mode is enabled with a toggle switch in the controller box.  When in Basic Mode, the only
// other valid mode is Shutdown Mode. Shutdown Mode may override Basic Mode.
// - Actions: Release and deactivate brake and gas actuators.  Steering PID keep active  
//
// ** Shutdown Mode **
// - Required: BasicMode switch Off & Ignition Off
// - Priority: 2
// This mode is active whenever the ignition is off.  In other words, whenever the
// little red pushbutton switch by the joystick is unclicked.  This happens before the
// ignition is pressed before driving, but it also may happen if the driver needs to
// panic and E-stop due to loss of control or any other reason.  The ignition will get cut
// independent of the controller, but we can help stop the car faster by applying the
// brakes. Once car is stopped, we release all actuators and then go idle.
// - Actions: 1. Release throttle. If car is moving AND BasicMode Off, apply brakes to stop car
// - Actions: 2: Release brakes and deactivate all actuators including steering
//
// ** Stall Mode **
// - Required: Engine stopped & BasicMode switch Off & Ignition On
// - Priority: 3
// This mode is active when the engine is not running.  If car is moving, then it presumably may
// coast to a stop.  The actuators are all enabled and work normally.  Starting the engine will 
// bring you into Hold Mode.  Shutdown Mode and Basic Mode both override Stall Mode. Note: This
// mode allows for driver to steer while being towed or pushed, or working on the car.
// - Actions: Enable all actuators
//
// ** Hold Mode **
// - Required: Engine running & JoyVert<=Center & BasicMode switch Off & Ignition On
// - Priority: 4
// This mode is entered from Stall Mode once engine is started, and also, whenever the car comes
// to a stop while driving around in Fly Mode.  This mode releases the throttle and will 
// continuously increase the brakes until the car is stopped, if it finds the car is moving. 
// Pushing up on the joystick from Hold mode releases the brakes & begins Fly Mode.
// Shutdown, Basic & Stall Modes override Hold Mode.
// # Actions: Close throttle, and Apply brake to stop car, continue to ensure it stays stopped.
//
// ** Fly Mode **
// - Required: (Car Moving OR JoyVert>Center) & In gear & Engine running & BasicMode Off & Ign On
// - Priority: 5
// This mode is for driving under manual control. In Fly Mode, vertical joystick positions
// result in a proportional level of gas or brake (AKA "Manual" control).  Fly Mode is
// only active when the car is moving - Once stopped or taken out of gear, we go back to Hold Mode.
// If the driver performs a special secret "cruise gesture" on the joystick, then go to Cruise Mode.
// Special cruise gesture might be: Pair of sudden full-throttle motions in rapid succession
// - Actions: Enable all actuators, Watch for gesture
//
// ** Cruise Mode **
// - Required: Car Moving & In gear & Engine running & BasicMode switch Off & Ignition On
// - Priority: 6 (Lowest)
// This mode is entered from Fly Mode by doing a special joystick gesture. In Cruise Mode,
// the brake is disabled, and the joystick vertical is different: If joyv at center, the
// throttle will actively maintain current car speed.  Up or down momentary joystick presses
// serve to adjust that target speed. A sharp, full-downward gesture will drop us back to 
// Fly Mode, promptly resulting in braking (if kept held down).
// - Actions: Release brake, Maintain car speed, Handle joyvert differently, Watch for gesture

// #ifdef CAP_TOUCH
// #else
//     #define touch_irq_pin 8  // (i2c0 sda / adc) - 
//     #define touch_cs_pin 9  // (i2c0 scl / adc) - Use as chip select for resistive touchscreen
// #endif

// Defines for all the GPIO pins we're using
#define button_pin 0  // (button0 / strap to 1) - This is the "Boot" button on the esp32 board
#define joy_horz_pin 1  // (adc) - Either analog left-right input (joystick)
#define joy_vert_pin 2  // (adc) - Either analog up-down input (joystick)
#define tft_dc_pin 3  // (adc* / strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
#define ign_batt_pin 4  // (adc) - Analog input, ignition signal and battery voltage sense, full scale is 15.638V
#define pot_wipe_pin 5  // (adc) - Analog in from 20k pot. Use 1% series R=22k to 5V on wipe=CW-0ohm side, and R=15k to gnd on wipe-CCW-0ohm side. Gives wipe range of 1.315V (CCW) to 3.070V (CW) with 80 uA draw.
#define brake_pos_pin 6  // (adc) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#define pressure_pin 7  // (adc) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
#define i2c_sda_pin 8  // (i2c0 sda / adc) - i2c bus for airspeed sensor, lighting board, cap touchscreen
#define i2c_scl_pin 9  // (i2c0 scl / adc) - i2c bus for airspeed sensor, lighting board, cap touchscreen
#define tft_cs_pin 10  // (spi0 cs / adc*) - Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
#define tft_mosi_pin 11  // (spi0 mosi / adc*) - Used as spi interface data to sd card and tft screen
#define tft_sclk_pin 12  // (spi0 sclk / adc*) - Used as spi interface clock for sd card and tft screen
#define tft_miso_pin 13  // (spi0 miso / adc*) - Used as spi interface data from sd card and possibly (?) tft screen
#define hotrc_ch2_vert_pin 14  // (pwm0 / adc*) - Hotrc Ch2 bidirectional trigger input
#define hotrc_ch1_horz_pin 15  // (pwm1 / adc*) - Hotrc Ch1 thumb joystick input
#define gas_pwm_pin 16  // (pwm1 / adc*) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
#define brake_pwm_pin 17  // (pwm0 / adc* / tx1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
#define steer_pwm_pin 18  // (pwm0 / adc* / rx1) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
#define onewire_pin 19  // (usb-otg / adc*) - Onewire bus for temperature sensor data
#define hotrc_ch3_ign_pin 20  // (usb-otg / adc*) - Ignition control, Hotrc Ch3 PWM toggle signal
#define hotrc_ch4_cruise_pin 21  // (pwm0) - Cruise control, Hotrc Ch4 PWM toggle signal
#define tach_pulse_pin 35  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
#define speedo_pulse_pin 36  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
#define ign_out_pin 37  // (spi-ram / oct-spi) - Output for Hotrc to a relay to kill the car ignition. Note, Joystick ign button overrides this if connected and pressed
#define syspower_pin 38  // (spi-ram / oct-spi) - Output, flips a relay to power all the tranducers. This is actually the neopixel pin on all v1.1 devkit boards.
#define touch_cs_pin 39  // Use as chip select for resistive touchscreen
#define encoder_b_pin 40  // Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
#define encoder_a_pin 41  // Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
#define encoder_sw_pin 42  // Input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
#define uart_tx_pin 43  // "TX" (uart0 tx) - Needed for serial monitor
#define uart_rx_pin 44  // "RX" (uart0 rx) - Needed for serial monitor. In theory we could dual-purpose this for certain things, as we haven't yet needed to accept input over the serial monitor
#define starter_pin 45  // (strap to 0) - Input, active high when vehicle starter is engaged (needs pulldown)
#define basicmodesw_pin 46  // (strap X) - Input, asserted to tell us to run in basic mode, active low (needs pullup)
#define sdcard_cs_pin 47  // Output, chip select allows SD card controller chip use of the SPI bus, active low
#define neopixel_pin 48  // (rgb led) - Data line to onboard Neopixel WS281x (on all v1 devkit boards)

#define tft_ledk_pin -1  // Output, optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define touch_irq_pin 255  // Input, optional touch occurence interrupt signal (for resistive touchscreen, prevents spi bus delays) - Set to 255 if not used
#define tft_rst_pin -1  // TFT Reset allows us to reboot the screen hardware when it crashes. Otherwise connect screen reset line to esp reset pin

#define adcbits 12
#define adcrange_adc 4095  // = 2^adcbits-1
#define adcmidscale_adc 2047  // = 2^(adcbits-1)-1

// Globals -------------------
bool serial_debugging = true; 
bool timestamp_loop = false;  // Makes code write out timestamps throughout loop to serial port
bool take_temperatures = true;

#define pwm_jaguars true

// Persistent config storage
Preferences config;

// Declare Hotrc RMT Inputs in global scope
RMTInput hotrc_horz(RMT_CHANNEL_4, gpio_num_t(hotrc_ch1_horz_pin)); 
RMTInput hotrc_vert(RMT_CHANNEL_5, gpio_num_t(hotrc_ch2_vert_pin)); 
RMTInput hotrc_ch3(RMT_CHANNEL_6, gpio_num_t(hotrc_ch3_ign_pin)); 
RMTInput hotrc_ch4(RMT_CHANNEL_7, gpio_num_t(hotrc_ch4_cruise_pin)); 

class Timer {  // 32 bit microsecond timer overflows after 71.5 minutes
  protected:
    volatile int64_t start_us = 0;
    volatile int64_t timeout_us = 0;
  public:
    Timer (void) { reset(); }
    Timer (uint32_t arg_timeout_us) { set ((int64_t)arg_timeout_us); }
    void IRAM_ATTR set (int64_t arg_timeout_us) { timeout_us = arg_timeout_us; reset(); }
    void IRAM_ATTR set (uint32_t arg_timeout_us) { set ((int64_t)arg_timeout_us); }
    void IRAM_ATTR reset (void) { start_us = esp_timer_get_time(); }
    bool IRAM_ATTR expired (void) { return (esp_timer_get_time() > start_us + timeout_us); }
    int64_t IRAM_ATTR elapsed (void) { return esp_timer_get_time() - start_us; }
    int64_t IRAM_ATTR get_timeout (void) { return timeout_us; }
};

float convert_units (float from_units, float convert_factor, bool invert, float in_offset = 0.0, float out_offset = 0.0) {
    if (!invert) return out_offset + convert_factor * (from_units - in_offset);
    if (from_units - in_offset) return out_offset + convert_factor / (from_units - in_offset);
    printf ("convert_units refused to divide by zero: %lf, %lf, %d, %lf, %lf", from_units, convert_factor, invert, in_offset, out_offset);
    return -1;
}

// Globals -------------------
//

#ifdef pwm_jaguars
    static Servo brake_servo;
    static Servo steer_servo;
#else  // jaguars controlled over asynchronous serial port
    #include <HardwareSerial.h>
    HardwareSerial jagPort(1);  // Open serisl port to communicate with jaguar controllers for steering & brake motors
#endif

// run state machine related
enum runmodes { BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL };
int32_t runmode = SHUTDOWN;
int32_t oldmode = SHUTDOWN;  // So we can tell when the mode has just changed. start as different to trigger_mode start algo
int32_t gesture_progress = 0;  // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
bool shutdown_complete = false;  // Shutdown mode has completed its work and can stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;  // Indicates we should release the brake & gas so the pedals can be used manually without interference
bool car_initially_moved = false;  // Whether car has moved at all since entering fly mode
bool calmode_request = false;
bool joy_centered = false;
bool panic_stop = false;
bool remote_starting = false;
bool remote_starting_last = false;
bool flycruise_toggle_request = false;
bool remote_start_toggle_request = false;
int32_t flycruise_vert_margin_adc = 25;  // Margin of error for determining hard brake value for dropping out of cruise mode
bool cruise_gesturing = false;  // Is cruise mode enabled by gesturing?  Otherwise by press of cruise button
bool cruise_sw_held = false;
bool cruise_adjusting = false;
Timer gestureFlyTimer;  // Used to keep track of time for gesturing for going in and out of fly/cruise modes
// Timer cruiseSwTimer;  // Was used to require a medium-length hold time pushing cruise button to switch modes
Timer sleepInactivityTimer (10000000);  // After shutdown how long to wait before powering down to sleep
Timer stopcarTimer (7000000);  // Allows code to fail in a sensible way after a delay if nothing is happening
uint32_t motor_park_timeout_us = 4000000;  // If we can't park the motors faster than this, then give up.
uint32_t gesture_flytimeout_us = 400000;  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
uint32_t cruise_sw_timeout_us = 500000;  // how long do you have to hold down the cruise button to start cruise mode (in us)
uint32_t cruise_antiglitch_timeout_us = 350000;  // Target speed won't change until manual adjustment is outside deadboand for longer than this 
Timer cruiseAntiglitchTimer(cruise_antiglitch_timeout_us);
Timer motorParkTimer(motor_park_timeout_us);

// simulator related
bool simulating_last = false;
Timer simTimer;
int32_t sim_edit_delta = 0;
int32_t sim_edit_delta_touch = 0;
int32_t sim_edit_delta_encoder = 0;
bool simulating = false;
enum pot_overload { none, pressure, brkpos, tach, airflow, speedo, battery, coolant };  // , joy, brkpos, pressure, basicsw, cruisesw, syspower }
int32_t pot_overload = speedo;  // Use the pot to simulate one of the sensors
bool sim_joy = false;
bool sim_tach = true;
bool sim_speedo = true;
bool sim_brkpos = true;
bool sim_basicsw = true;
bool sim_cruisesw = false;
bool sim_pressure = true;
bool sim_syspower = true;
bool sim_starter = true;
bool sim_ignition = true;
bool sim_airflow = false;
bool sim_battery = true;
bool sim_coolant = true;

// calibration related
bool cal_joyvert_brkmotor = false;  // Allows direct control of brake motor using controller vert
bool cal_pot_gasservo = false;  // Allows direct control of gas servo using pot
bool cal_pot_gas_ready = false;  // To avoid immediately overturning gas pot, first pot must be turned to valid range
bool cal_set_hotrc_failsafe_ready = false;  

// diag/monitoring variables
Timer loopTimer(1000000);  // how long the previous main loop took to run (in us)
uint32_t loop_period_us;
uint64_t looptime_sum_us;
uint32_t looptime_avg_us;
float loop_freq_hz = 1;  // run loop real time frequency (in Hz)
volatile int32_t loop_int_count = 0;  // counts interrupts per loop
int32_t loopno = 1;
uint32_t looptimes_us[20];
bool loop_dirty[20];
int32_t loopindex = 0;
bool booted = false;
bool diag_ign_error_enabled = true;

// neopixel and heartbeat related
uint8_t neo_wheelcounter = 0;
uint8_t neo_brightness_max = 15;
uint32_t neo_timeout_us = 150000;
Timer neoTimer (neo_timeout_us);
bool neo_heartbeat = (neopixel_pin >= 0);
uint8_t neo_brightness = neo_brightness_max;  // brightness during fadeouts
enum neo_colors { N_RED, N_GRN, N_BLU };
uint8_t neo_heartcolor[3] = { 0xff, 0xff, 0xff };
Timer heartbeatTimer (1000000);
int32_t heartbeat_state = 0;
int32_t heartbeat_level = 0;
uint32_t heartbeat_ekg_us[4] = { 170000, 150000, 530000, 1100000 };
int32_t heartbeat_pulse = 255;
static Adafruit_NeoPixel neostrip(1, neopixel_pin, NEO_GRB + NEO_GRB + NEO_KHZ800);

// pushbutton related
enum sw_presses { NONE, SHORT, LONG };  // used by encoder sw and button algorithms
bool boot_button_last = 0;
bool boot_button = 0;
bool btn_press_timer_active = false;
bool btn_press_suppress_click = false;
bool btn_press_action = NONE;

// external digital input and output signal related
bool ignition = LOW;
bool ignition_last = ignition;
bool ignition_output_enabled = false;  // disallows configuration of ignition pin as an output until hotrc detected
bool ignition_sense = ignition;
float ignition_on_thresh_v = 2.0;  // Below this voltage ignition is considered off
bool syspower = HIGH;
bool syspower_last = syspower;
bool basicmodesw = LOW;
bool cruise_sw = LOW;
bool starter = LOW;
bool starter_last = LOW;

// Temperature sensor related
long temp_temp, temp_last, temp_temp_addr_peef;  // peef variables
static int temp_secs = 0;  // peef variables
static byte temp_data[2];  // peef variables
static int16_t temp_raw;  // peef variables
enum temp_sensors { AMBIENT, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };
enum this_is_a_total_hack { WHEEL = 2 };
enum temp_lims { DISP_MIN, NOM_MIN, NOM_MAX, WARNING, ALARM, DISP_MAX };  // Possible sources of gas, brake, steering commands
float temp_lims_f[3][6] { { 0.0,  45.0, 115.0, 120.0, 130.0, 220.0 },  // [AMBIENT][NOM_MIN/NOM_MAX/WARNING/ALARM]
                          { 0.0, 178.0, 198.0, 202.0, 205.0, 220.0 },  // [ENGINE][NOM_MIN/NOM_MAX/WARNING/ALARM]
                          { 0.0,  50.0, 120.0, 130.0, 140.0, 220.0 }, };  // [WHEEL][NOM_MIN/NOM_MAX/WARNING/ALARM] (applies to all wheels)
float temp_sensor_min_f = -67.0;  // Minimum reading of sensor is -25 C = -67 F
float temp_sensor_max_f = 257.0;  // Maximum reading of sensor is 125 C = 257 F
// float temp_room = 77.0;  // "Room" temperature is 25 C = 77 F  Who cares?
float temps_f[6];  // Array stores most recent readings for all sensors
int32_t temp_detected_device_ct = 0;
int32_t temperature_precision = 12;  // 9-12 bit resolution
OneWire onewire (onewire_pin);
int32_t temp_current_index = 0;
enum temp_status : bool { CONVERT, READ };
temp_status temp_state = CONVERT;
uint32_t temp_times_us[2] = { 2000000, 10000 };  // Peef delay was 10000 (10ms)
uint32_t temp_timeout_us = 2000000;
Timer tempTimer (temp_timeout_us);
DeviceAddress temp_temp_addr;
DeviceAddress temp_addrs[6];  // Store the These are our finalHard code to the actual sensor addresses for the corresponding sense location on the car
// DeviceAddress temp_vehicle_addrs[6] =  // Hard code to the actual sensor addresses for the corresponding sense location on the car
//     { 0x0000000000000000,  // AMBIENT
//       0x0000000000000000,  // ENGINE
//       0x0000000000000000,  // WHEEL_FL
//       0x0000000000000000,  // WHEEL_FL
//       0x0000000000000000,  // WHEEL_RL
//       0x0000000000000000 };  // WHEEL_RR
DallasSensor tempsensebus (&onewire);

// mule battery related
float battery_adc = adcmidscale_adc;
float battery_v = 10.0;
float battery_filt_v = 10.0;
float battery_max_v = 16.0;  // The max vehicle voltage we can sense. Design resistor divider to match. Must exceed max V possible.
float battery_convert_v_per_adc = battery_max_v/adcrange_adc;
bool battery_convert_invert = false;
int32_t battery_convert_polarity = 1;  // Forward
float battery_ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 

// potentiometer related
float pot_percent = 50;
float pot_filt_percent = pot_percent;
float pot_min_percent = 0;  //
float pot_max_percent = 100;  //
int32_t pot_adc = adcmidscale_adc;
float pot_min_adc = 300;  // TUNED 230603 - Used only in determining theconversion factor
float pot_max_adc = 4095;  // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
float pot_convert_percent_per_adc = (pot_max_percent - pot_min_percent)/(pot_max_adc - pot_min_adc);  // 100 % / (3996 adc - 0 adc) = 0.025 %/adc
bool pot_convert_invert = false;
float pot_convert_offset = -0.08;
int32_t pot_convert_polarity = 1;  // Forward
float pot_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 

// controller related
enum ctrls { HOTRC, JOY, SIM, HEADLESS };  // Possible sources of gas, brake, steering commands
enum ctrl_axes { HORZ, VERT, CH3, CH4 };
enum ctrl_thresh { MIN, CENT, MAX, DB };
enum ctrl_edge { BOT, TOP };
enum ctrl_vals { RAW, FILT };
enum hotrc_sources { MICROS, ESP_RMT };
float ctrl_ema_alpha[2] = { 0.05, 0.1 };  // [HOTRC/JOY] alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t ctrl_lims_adc[2][2][4] = { { { 0, adcmidscale_adc, adcrange_adc, 42 }, { 0, adcmidscale_adc, adcrange_adc, 42 } }, { { 9, adcmidscale_adc, 4085, 50 }, { 9, adcmidscale_adc, 4085, 50 } } }; // [HOTRC/JOY] [HORZ/VERT], [MIN/CENT/MAX/DB] values as microseconds (hotrc) or adc counts (joystick)
int32_t ctrl_db_adc[2][2];  // [HORZ/VERT] [BOT/TOP] - to store the top and bottom deadband values for each axis of selected controller
int32_t ctrl_pos_adc[2][2];  // [HORZ/VERT] [RAW/FILT] - holds most current controller values
bool ctrl = HOTRC;  // Use HotRC controller to drive instead of joystick?
int32_t hotrc_source = ESP_RMT;

int32_t hotrc_pulse_lims_us[4][3] = { { 970-1, 1470-3, 1970-3 }, { 1080-1, 1580-3, 2080-3 }, { 1200-1, 1500-2, 1800-3 }, { 1300-1, 1500-2, 1700-3 } };  // [HORZ/VERT/CH3/CH4] [MIN/CENT/MAX]  // These are the l
int32_t hotrc_spike_buffer[2][3];
bool hotrc_radio_detected = false;
bool hotrc_radio_detected_last = hotrc_radio_detected;
bool hotrc_suppress_next_ch3_event = true;  // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
bool hotrc_suppress_next_ch4_event = true;  // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
float hotrc_pulse_period_us = 1000000.0 / 50;
int32_t hotrc_horz_pulse_us, hotrc_vert_pulse_us, hotrc_horz_pulse_filt_us, hotrc_vert_pulse_filt_us;
int32_t hotrc_pulse_failsafe_min_us = 780;  // Hotrc must be configured per the instructions: search for "HotRC Setup Procedure"
int32_t hotrc_pulse_failsafe_max_us = 980;  // in the carpet dumpster file: https://docs.google.com/document/d/1VsAMAy2v4jEO3QGt3vowFyfUuK1FoZYbwQ3TZ1XJbTA/edit
int32_t hotrc_pulse_failsafe_pad_us = 10;
uint32_t hotrc_panic_timeout_us = 500000;  // how long to receive flameout-range signal from hotrc vertical before panic stopping
Timer hotrcPanicTimer (hotrc_panic_timeout_us);
volatile int64_t hotrc_timer_start;
volatile bool hotrc_ch3_sw, hotrc_ch4_sw, hotrc_ch3_sw_event, hotrc_ch4_sw_event, hotrc_ch3_sw_last, hotrc_ch4_sw_last;
volatile bool hotrc_isr_pin_preread = true;
volatile int64_t hotrc_horz_pulse_64_us = (int64_t)hotrc_pulse_lims_us[HORZ][CENT];
volatile int64_t hotrc_vert_pulse_64_us = (int64_t)hotrc_pulse_lims_us[VERT][CENT];
// volatile int32_t intcount = 0;

// steering related
float steer_safe_percent = 72.0;  // Steering is slower at high speed. How strong is this effect 
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
float steer_pulse_out_us;  // pid loop output to send to the actuator (steering)
float steer_pulse_right_min_us = 500;  // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float steer_pulse_right_us = 670;  // Steering pulsewidth corresponding to full-speed right steering (in us). Default setting for jaguar is max 670us
float steer_pulse_stop_us = 1500;  // Steering pulsewidth corresponding to zero steering motor movement (in us)
float steer_pulse_left_us = 2330;  // Steering pulsewidth corresponding to full-speed left steering (in us). Default setting for jaguar is max 2330us
float steer_pulse_left_max_us = 2500;  // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us

// brake pressure related
int32_t pressure_adc;
int32_t pressure_min_adc = 658; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
int32_t pressure_sensor_max_adc = adcrange_adc; // Sensor reading max, limited by adc Vmax. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as chris can push (wimp)
int32_t pressure_max_adc = 2080; // Sensor measured maximum reading. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as [wimp] chris can push
float pressure_convert_psi_per_adc = 1000.0 * (3.3 - 0.554) / ( (pressure_sensor_max_adc - pressure_min_adc) * (4.5 - 0.554) );  // 1000 psi * (adc_max v - v_min v) / ((4095 adc - 658 adc) * (v-max v - v-min v)) = 0.2 psi/adc 
bool pressure_convert_invert = false;
float pressure_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
float pressure_margin_psi = 2.5;  // Margin of error when comparing brake pressure adc values (psi)
float pressure_min_psi = 0.0;  // TUNED 230602 - Brake pressure when brakes are effectively off. Sensor min = 0.5V, scaled by 3.3/4.5V is 0.36V of 3.3V (ADC count 0-4095). 
float pressure_max_psi = convert_units (pressure_max_adc - pressure_min_adc, pressure_convert_psi_per_adc, pressure_convert_invert);  // TUNED 230602 - Highest possible pressure achievable by the actuator 
float pressure_hold_initial_psi = 150;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
float pressure_hold_increment_psi = 15;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
float pressure_panic_initial_psi = 250;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
float pressure_panic_increment_psi = 25;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
float pressure_psi = (pressure_min_psi+pressure_max_psi)/2;
float pressure_filt_psi = pressure_psi;  // Stores new setpoint to give to the pid loop (brake)
float pressure_target_psi;

// brake actuator motor related
Timer brakeIntervalTimer (500000);  // How much time between increasing brake force during auto-stop if car still moving?
int32_t brake_increment_interval_us = 500000;  // How often to apply increment during auto-stopping (in us)

float brake_out_percent;
float brake_retract_max_percent = 100.0;  // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float brake_retract_percent = 100.0;  // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us). Default setting for jaguar is max 670us
float brake_stop_percent = 0.0;  // Brake pulsewidth corresponding to center point where motor movement stops (in us)
float brake_extend_percent = -100.0;  // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us). Default setting for jaguar is max 2330us
float brake_extend_min_percent = -100.0;  // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float brake_margin_percent = 2.4; // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated 

float brake_pulse_out_us;  // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
float brake_pulse_retract_min_us = 670;  // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float brake_pulse_retract_us = 670;  // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us). Default setting for jaguar is max 670us
float brake_pulse_stop_us = 1500;  // Brake pulsewidth corresponding to center point where motor movement stops (in us)
float brake_pulse_extend_us = 2330;  // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us). Default setting for jaguar is max 2330us
float brake_pulse_extend_max_us = 2330;  // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
// float brake_pulse_margin_us = 40; // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated 
// don't convert, just map the brake.  Using conversion ratio only to scale pid constants
// //  (1000 psi * (adc_max v - v_min v) / ((4095 adc - 658 adc) * (v-max v - v-min v)) = 0.2 psi/adc 
// bool brake_motor_convert_invert = false;
// int32_t brake_motor_convert_polarity = -1;  // Reverse

// brake actuator position related
float brake_pos_in;
float brake_pos_filt_in;
float brake_pos_convert_in_per_adc = 3.3 * 10000.0 / (5.0 * adcrange_adc * 557);  // 3.3 v * 10k ohm * 1/5 1/v * 1/4095 1/adc * 1/557 in/ohm = 0.0029 in/adc
bool brake_pos_convert_invert = false;
int32_t brake_pos_convert_polarity = 1;  // Forward
float brake_pos_ema_alpha = 0.25;
float brake_pos_abs_min_retract_in = 0.335;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. ("in"sandths of an inch)
float brake_pos_nom_lim_retract_in = 0.506;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (in)
float brake_pos_zeropoint_in = 3.179;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (in)
float brake_pos_park_in = 4.234;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (in)
float brake_pos_nom_lim_extend_in = 4.624;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (in)
float brake_pos_abs_max_extend_in = 8.300;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (in)
float brake_pos_margin_in = .029;  //

// carspeed/speedo related
float speedo_target_mph;
float speedo_govern_mph;  // Governor must scale the top vehicle speed proportionally. This is given a value in the loop
float speedo_mph = 1.01;  // Current car speed, raw as sensed (in mph)
float speedo_filt_mph = 1.02;  // Current car speed, filtered (in mph)
volatile int64_t speedo_us = 0;
int32_t speedo_buf_us = 0;
volatile int64_t speedo_timer_start_us = 0;
volatile int64_t speedo_time_us;
volatile int64_t speedo_timer_read_us = 0;
float speedo_convert_mph_per_rpus = 1000000.0 * 3600.0 * 20 * 3.14159 / (19.85 * 12 * 5280);  // 1 pulrot/us * 1000000 us/sec * 3600 sec/hr * 1/19.85 whlrot/pulrot * 20*pi in/whlrot * 1/12 ft/in * 1/5280 mi/ft = 179757 mi/hr (mph)
    // 5280 ft/mi * 12 in/ft * 1/(20*pi) whlrot/in * 19.85 pulrot/whlrot = 20017 pulrot/mile
    // Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
bool speedo_convert_invert = true;
int32_t speedo_convert_polarity = 1;  // Forward
float speedo_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
float speedo_idle_mph = 4.50;  // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)
float speedo_redline_mph = 15.0;  // What is our steady state speed at redline? Pulley rotation frequency (in milli-mph)
float speedo_max_mph = 25.0;  // What is max speed car can ever go
float speedo_stop_thresh_mph = 0.1;  // Below which the car is considered stopped
uint32_t speedo_stop_timeout_us = 600000;  // Time after last magnet pulse when we can assume the car is stopped (in us)
int64_t speedo_delta_abs_min_us = 4500;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers

// throttle servo related
float gas_pulse_out_us = 1501;  // pid loop output to send to the actuator (gas)
float gas_pulse_govern_us = 1502;  // Governor must scale the pulse range proportionally. This is given a value in the loop
Timer gasServoTimer (500000);  // We expect the servo to find any new position within this time
float gas_governor_percent = 95;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
float gas_pulse_cw_min_us = 500;  // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
float gas_pulse_redline_us = 1400;  // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
float gas_pulse_idle_us = 1800;  // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
float gas_pulse_ccw_max_us = 2500;  // Servo ccw limit pulsewidth. Hotrc controller ch1/2 min(lt/br) = 1000us, center = 1500us, max(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
float gas_pulse_park_slack_us = 30;  // Gas pulsewidth beyond gas_pulse_idle_us where to park the servo out of the way so we can drive manually (in us)

// tachometer related
volatile int64_t tach_us = 0;
int32_t tach_buf_us = 0;
volatile int64_t tach_timer_start_us = 0;
volatile int64_t tach_time_us;
volatile int64_t tach_timer_read_us = 0;
float tach_target_rpm, tach_adjustpoint_rpm;
float tach_rpm = 50.0;  // Current engine speed, raw value converted to rpm (in rpm)
float tach_filt_rpm = 50.0;  // Current engine speed, filtered (in rpm)
float tach_govern_rpm;  // Software engine governor creates an artificially reduced maximum for the engine speed. This is given a value in calc_governor()
float tach_convert_rpm_per_rpus = 60.0 * 1000000.0;  // 1 rot/us * 60 sec/min * 1000000 us/sec = 60000000 rot/min (rpm)
bool tach_convert_invert = true;
int32_t tach_convert_polarity = 1;  // Forward      
float tach_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
float tach_idle_rpm = 700.0;  // Min value for engine hz, corresponding to low idle (in rpm) Note, this value is itself highly variable, dependent on engine temperature
float tach_max_rpm = 7000.0;  // Max possible engine rotation speed
float tach_redline_rpm = 5000.0;  // Max value for tach_rpm, pedal to the metal (in rpm). 20000 rotations/mile * 15 mi/hr * 1/60 hr/min = 5000 rpm
float tach_margin_rpm = 15.0;  // Margin of error for checking engine rpm (in rpm)
float tach_stop_thresh_rpm = 0.1;  // Below which the engine is considered stopped - this is redundant,
uint32_t tach_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
int64_t tach_delta_abs_min_us = 6500;  // 6500 us corresponds to about 10000 rpm, which isn't possible. Use to reject retriggers
float tach_idle_min_rpm = 475;  // Idle speed at nom_max engine temp
float tach_idle_max_rpm = 800;  // Idle speed at nom_min engine temp

// airflow related
bool airflow_detected = false;
float airflow_mph = 0.0;
float airflow_filt_mph = airflow_mph;
float airflow_target_mph = airflow_mph;
float airflow_min_mph = 0.0;
float airflow_max_mph = 33.5;  // 620/2 cm3/rot * 5000 rot/min (max) * 60 min/hr * 1/(pi * (2.85 / 2)^2) 1/cm2 * 1/160934 mi/cm = 90.58 mi/hr (mph) (?!)
float airflow_idle_mph = airflow_max_mph * tach_idle_rpm / tach_redline_rpm;
// What diameter intake hose will reduce airspeed to abs max?  2.7 times the xsectional area. Current area is 6.38 cm2. New diameter = 4.68 cm (min). So, need to adapt to 2.5in + tube
float airflow_abs_max_mph = 33.55;
float airflow_ema_alpha = 0.2;
FS3000 airflow_sensor;
            
// Motor control:
// Steering : Controls the steering motor proportionally based on the joystick
uint32_t steer_pid_period_us = 185000;  // (Not actually a pid) Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer steerPidTimer (steer_pid_period_us);  // not actually tunable, just needs value above

// Brake : Controls the brake motor to achieve the desired brake fluid pressure
uint32_t brake_pid_period_us = 185000;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer brakePidTimer (brake_pid_period_us);  // not actually tunable, just needs value above

// float brake_perc_per_us = (100.0 - (-100.0)) / (brake_pulse_extend_us - brake_pulse_retract_us);  // (100 - 0) percent / (us-max - us-min) us = 1/8.3 = 0.12 percent/us
float brake_spid_initial_kp = 2.110 * 0.12;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
float brake_spid_initial_ki_hz = 0.473 * 0.12;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
float brake_spid_initial_kd_s = 0.830 * 0.12;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
QPID brakeQPID (&pressure_filt_psi, &brake_out_percent, &pressure_target_psi,  // input, target, output variable references
    brake_extend_percent, brake_retract_percent,  // output min, max
    brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s,  // Kp, Ki, and Kd tuning constants
    QPID::pMode::pOnError, QPID::dMode::dOnError, QPID::iAwMode::iAwRound, QPID::Action::direct,  // settings  // iAwRoundCond, iAwClamp
    brake_pid_period_us, QPID::Control::timer, QPID::centMode::centerStrict, brake_stop_percent);  // period, more settings
// float brake_spid_initial_kp = 2.110;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
// float brake_spid_initial_ki_hz = 0.873;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
// float brake_spid_initial_kd_s = 1.130;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
// QPID brakeQPID (&pressure_filt_psi, &brake_pulse_out_us, &pressure_target_psi,  // input, target, output variable references
//     brake_pulse_retract_us, brake_pulse_extend_us,  // output min, max
//     brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s,  // Kp, Ki, and Kd tuning constants
//     QPID::pMode::pOnError, QPID::dMode::dOnError, QPID::iAwMode::iAwClamp, QPID::Action::reverse,  // settings
//     brake_pid_period_us, QPID::Control::timer, QPID::centMode::centerStrict, brake_pulse_stop_us);  // period, more settings
    
// Gas : Controls the throttle to achieve the desired intake airflow and engine rpm
uint32_t gas_pid_period_us = 225000;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer gasPidTimer (gas_pid_period_us);  // not actually tunable, just needs value above
float gas_spid_initial_kp = 0.256;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
float gas_spid_initial_ki_hz = 0.022;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
float gas_spid_initial_kd_s = 0.091;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
bool gas_open_loop = false;
static Servo gas_servo;
QPID gasQPID (&tach_filt_rpm, &gas_pulse_out_us, &tach_target_rpm,  // input, target, output variable references
    (float)gas_pulse_redline_us, (float)gas_pulse_idle_us,  // output min, max
    gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s,  // Kp, Ki, and Kd tuning constants
    QPID::pMode::pOnErrorMeas, QPID::dMode::dOnMeas, QPID::iAwMode::iAwRound, QPID::Action::reverse,  // settings
    gas_pid_period_us, QPID::Control::timer, QPID::centMode::range);  // period, more settings

// Cruise : is active on demand while driving. It controls the throttle target to achieve the desired vehicle speed
uint32_t cruise_pid_period_us = 300000;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer cruisePidTimer (cruise_pid_period_us);  // not actually tunable, just needs value above
float cruise_spid_initial_kp = 0.157;  // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
float cruise_spid_initial_ki_hz = 0.035;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
float cruise_spid_initial_kd_s = 0.044;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
QPID cruiseQPID (&speedo_filt_mph, &tach_target_rpm, &speedo_target_mph,  // input, target, output variable references
    (float)tach_idle_rpm, (float)tach_govern_rpm,  // output min, max
    cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s,  // Kp, Ki, and Kd tuning constants
    QPID::pMode::pOnError, QPID::dMode::dOnError, QPID::iAwMode::iAwRound, QPID::Action::direct,  // settings
    cruise_pid_period_us, QPID::Control::timer, QPID::centMode::range);  // period, more settings

SdFat sd;  // SD card filesystem
#define approot "cantroller2020"
#define logfile "log.txt"
#define error(msg) sd.errorHalt(F(msg))  // Error messages stored in flash.
SdFile root;  // Directory file.
SdFile file;  // Use for file creation in folders.

// Interrupt service routines
//
// The tach and speed use a hall sensor being triggered by a passing magnet once per pulley turn. These ISRs call mycros()
// on every pulse to know the time since the previous pulse. I tested this on the bench up to about 0.750 mph which is as 
// fast as I can move the magnet with my hand, and it works. Update: Janky bench test appeared to work up to 11000 rpm.
void IRAM_ATTR tach_isr (void) {  // The tach and speedo isrs get the period of the vehicle pulley rotations.
    tach_timer_read_us = esp_timer_get_time();
    tach_time_us = tach_timer_read_us - tach_timer_start_us;
    if (tach_time_us > tach_delta_abs_min_us) {  // ignore spurious triggers or bounces
        tach_timer_start_us = tach_timer_read_us;
        tach_us = tach_time_us;
    }
}
void IRAM_ATTR speedo_isr (void) {  //  Handler can get the most recent rotation time at speedo_us
    speedo_timer_read_us = esp_timer_get_time();
    speedo_time_us = speedo_timer_read_us - speedo_timer_start_us;
    if (speedo_time_us > speedo_delta_abs_min_us) {  // ignore spurious triggers or bounces
        speedo_timer_start_us = speedo_timer_read_us;
        speedo_us = speedo_time_us;
    }
}
void handle_hotrc_vert(int32_t pulse_width) {
    // reads return 0 if the buffer is empty eg bc our loop is running faster than the rmt is getting pulses
    if (pulse_width > 0) {
        hotrc_vert_pulse_64_us = pulse_width;
    }
}
void handle_hotrc_horz(int32_t pulse_width) {
    if (pulse_width > 0) {
        hotrc_vert_pulse_64_us = pulse_width;
    }
}
void hotrc_ch3_update (void) {  // 
    hotrc_ch3_sw = (hotrc_ch3.readPulseWidth(true) <= 1500);  // Ch3 switch true if short pulse, otherwise false  hotrc_pulse_lims_us[CH3][CENT]
    if (hotrc_ch3_sw != hotrc_ch3_sw_last) hotrc_ch3_sw_event = true;  // So a handler routine can be signaled. Handler must reset this to false
    hotrc_ch3_sw_last = hotrc_ch3_sw;
}
void hotrc_ch4_update (void) {  // 
    hotrc_ch4_sw = (hotrc_ch4.readPulseWidth(true) <= 1500);  // Ch3 switch true if short pulse, otherwise false  hotrc_pulse_lims_us[CH3][CENT]
    if (hotrc_ch4_sw != hotrc_ch4_sw_last) hotrc_ch4_sw_event = true;  // So a handler routine can be signaled. Handler must reset this to false
    hotrc_ch4_sw_last = hotrc_ch4_sw;
}

// TODO handle hotrc_ch3 and ch4
// void handle_hotrc_ch3(int32_t pulse_width) {
//     // handle here
// }

// void IRAM_ATTR hotrc_ch3_isr (int32_t pulse_width) {  // On falling edge, records high pulse width to determine ch3 button toggle state
//     hotrc_ch3_sw = (esp_timer_get_time() - hotrc_timer_start <= 1500);  // Ch3 switch true if short pulse, otherwise false  hotrc_pulse_lims_us[CH3][CENT]
//     if (hotrc_ch3_sw != hotrc_ch3_sw_last) hotrc_ch3_sw_event = true;  // So a handler routine can be signaled. Handler must reset this to false
//     hotrc_ch3_sw_last = hotrc_ch3_sw;
// }
// void IRAM_ATTR hotrc_ch4_isr (void) {  // Triggers on both edges. Sets timer on rising edge (for all channels) and reads it on falling to determine ch4 button toggle state
//     if (hotrc_isr_pin_preread) hotrc_timer_start = esp_timer_get_time();  // hotrcPulseTimer.reset();
//     else {
//         hotrc_ch4_sw = (esp_timer_get_time() - hotrc_timer_start <= 1500);  // Ch4 switch true if short pulse, otherwise false  hotrc_pulse_lims_us[CH4][CENT]
//         if (hotrc_ch4_sw != hotrc_ch4_sw_last) hotrc_ch4_sw_event = true;  // So a handler routine can be signaled. Handler must reset this to false
//         hotrc_ch4_sw_last = hotrc_ch4_sw;
//     }
//     hotrc_isr_pin_preread = !(digitalRead (hotrc_ch4_cruise_pin));  // Read pin after timer operations to maximize clocking accuracy
// }  // intcount++;

// Attempt to use MCPWM input capture pulse width timer unit to get precise hotrc readings
// int32_t hotrc_ch3_pulse_us, hotrc_ch4_pulse_us;
// uint32_t mcpwm_unit0_capture, mcpwm_unit1_capture, mcpwm_unit2_capture;
// uint32_t mcpwm_unit0_capture_last, mcpwm_unit1_capture_last, mcpwm_unit2_capture_last;
// int32_t hotrc_ch3_preread;
// void IRAM_ATTR hotrc_isr (void) {
//     mcpwm_unit0_capture = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
//     mcpwm_unit1_capture = mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP0);
//     hotrc_horz_pulse_us = (int32_t)(mcpwm_unit0_capture - mcpwm_unit0_capture_last);
//     hotrc_vert_pulse_us = (int32_t)(mcpwm_unit1_capture - mcpwm_unit1_capture_last);
//     mcpwm_unit0_capture_last = mcpwm_unit0_capture;
//     mcpwm_unit1_capture_last = mcpwm_unit1_capture;
// }
// // // Separate attempt to use timers to measure pulses
// // void IRAM_ATTR hotrc_ch1_isr (void) {
// //     mcpwm_unit0_capture = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
// //     hotrc_horz_pulse_us = (int32_t)(mcpwm_unit0_capture - mcpwm_unit0_capture_last);
// //     mcpwm_unit0_capture_last = mcpwm_unit0_capture;
// // }
// // void IRAM_ATTR hotrc_ch2_isr (void) {
// //     mcpwm_unit1_capture = mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP0);
// //     hotrc_vert_pulse_us = (int32_t)(mcpwm_unit1_capture - mcpwm_unit1_capture_last);
// //     mcpwm_unit1_capture_last = mcpwm_unit1_capture;
// // }
    // if (hotrc_vert_preread) hotrc_timer_start = esp_timer_get_time();  // hotrcPulseTimer.reset();
    // else hotrc_vert_pulse_us = esp_timer_get_time() - hotrc_timer_start;  // hotrcPulseTimer.elapsed();
    // hotrc_vert_preread = !(digitalRead (hotrc_ch2_horz_pin));  // Read pin after timer operations to maximize clocking accuracy

    // intcount++;
    // // hotrc_horz_pulse_us = esp_timer_get_time() - hotrc_timer_start;

// void IRAM_ATTR hotrc_vert_isr() {
//   hotrc_vert_pulse_us = (uint32_t)timerRead(hotrc_vert_timer);
//   timerStop(hotrc_vert_timer);
//   timerAlarmWrite(hotrc_vert_timer, 0xFFFFFFFF, true);  // Reset the alarm value
//   timerRestart(hotrc_vert_timer);
// }

// Utility functions
#define arraysize(x) ((int32_t)(sizeof(x) / sizeof((x)[0])))  // A macro function to determine the length of string arrays
#define floor(amt, lim) ((amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)
#undef max
inline float max (float a, float b) { return (a > b) ? a : b; }
inline int32_t max (int32_t a, int32_t b) { return (a > b) ? a : b; }
#undef min
inline float min (float a, float b) { return (a < b) ? a : b; }
inline int32_t min (int32_t a, int32_t b) { return (a < b) ? a : b; }
#undef constrain
inline float constrain (float amt, float low, float high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline int32_t constrain (int32_t amt, int32_t low, int32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline uint32_t constrain (uint32_t amt, uint32_t low, uint32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
#undef map
inline float map (float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
inline int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
bool rounding = true;
float dround (float val, int32_t digits) { return (rounding) ? (std::round(val * std::pow (10, digits)) / std::pow (10, digits)) : val; }

bool inline car_stopped (void) { return (speedo_filt_mph < speedo_stop_thresh_mph); }  // Moved logic that was here to the main loop
bool inline engine_stopped (void) { return (tach_filt_rpm < tach_stop_thresh_rpm); }  // Note due to weird float math stuff, can not just check if tach == 0.0

uint32_t colorwheel (uint8_t WheelPos) {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) return neostrip.Color (255 - WheelPos * 3, 0, WheelPos * 3);
    if (WheelPos < 170) {
        WheelPos -= 85;
        return neostrip.Color (0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return neostrip.Color (WheelPos * 3, 255 - WheelPos * 3, 0);
}
void calc_ctrl_lims (void) {
    ctrl_db_adc[VERT][BOT] = ctrl_lims_adc[ctrl][VERT][CENT]-ctrl_lims_adc[ctrl][VERT][DB]/2;  // Lower threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[VERT][TOP] = ctrl_lims_adc[ctrl][VERT][CENT]+ctrl_lims_adc[ctrl][VERT][DB]/2;  // Upper threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][BOT] = ctrl_lims_adc[ctrl][HORZ][CENT]-ctrl_lims_adc[ctrl][HORZ][DB]/2;  // Lower threshold of horz joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][TOP] = ctrl_lims_adc[ctrl][HORZ][CENT]+ctrl_lims_adc[ctrl][HORZ][DB]/2;  // Upper threshold of horz joy deadband (ADC count 0-4095)
    steer_safe_ratio = steer_safe_percent/100.0;
}
void calc_governor (void) {
    tach_govern_rpm = map ((float)gas_governor_percent, 0.0, 100.0, 0.0, tach_redline_rpm);  // Create an artificially reduced maximum for the engine speed
    gas_pulse_govern_us = map (gas_governor_percent*(tach_redline_rpm-tach_idle_rpm)/tach_redline_rpm, 0.0, 100.0, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
    speedo_govern_mph = map ((float)gas_governor_percent, 0.0, 100.0, 0.0, speedo_redline_mph);  // Governor must scale the top vehicle speed proportionally
}
float steer_safe (float endpoint) {
    return steer_stop_percent + (endpoint - steer_stop_percent) * (1 - steer_safe_ratio * speedo_filt_mph / speedo_redline_mph);
    // return steer_pulse_stop_us + (endpoint - steer_pulse_stop_us) * map (speedo_filt_mph, 0.0, speedo_redline_mph, 1.0, steer_safe_ratio);
    // return steer_stop_percent + (endpoint - steer_stop_percent) * (1 - steer_safe_ratio * speedo_filt_mph / speedo_redline_mph);
}

// Exponential Moving Average filter : Smooth out noise on inputs. 0 < alpha < 1 where lower = smoother and higher = more responsive
// Pass in a fresh raw value, address of filtered value, and alpha factor, filtered value will get updated
void ema_filt (float raw, float* filt, float alpha) {
    // if (!raw) *filt = 0.0; else
    *filt = alpha * raw + (1 - alpha) * (*filt);
}
void ema_filt (int32_t raw, float* filt, float alpha) {
    ema_filt ((float)raw, filt, alpha);
}
void ema_filt (int32_t raw, int32_t* filt, float alpha) {
    *filt = (int32_t)(alpha * (float)raw + (1 - alpha) * (float)(*filt) + 0.5);  // (?) Adding 0.5 to compensate for the average loss due to int casting roundoff
}

void sd_init() {
    if (!sd.begin (sdcard_cs_pin, SD_SCK_MHZ (50))) sd.initErrorHalt();  // Initialize at highest supported speed that is not over 50 mhz. Go lower if errors.
    if (!root.open ("/")) error("open root failed");
    if (!sd.exists (approot)) { 
        if (sd.mkdir (approot)) Serial.println (F("Created approot directory\n"));  // cout << F("Created approot directory\n");
        else error("Create approot failed");
    }
    // Change volume working directory to Folder1.
    // if (sd.chdir(approot)) {
    //    cout << F("\nList of files in appdir:\n");
    //    char *apppath = (char*)malloc((arraysize(appdir)+2)*sizeof(char));
    //        sd.ls(strcat("/",approot, LS_R);
    // }
    // else {
    //     error("Chdir approot failed\n");
    // }    
    // if (!file.open(logfile, O_WRONLY | O_CREAT)) {
    //     error("Open logfile failed\n");
    // }
    // file.close();
    // Serial.println(F("Filesystem init finished\n"));  // cout << F("Filesystem init finished\n");
    // for (byte a = 10; a >= 1; a--) {
    //     char fileName[12];
    //     sprintf(fileName, "%d.txt", a);
    //     file = sd.open(fileName, FILE_WRITE); //create file
    // }
}

// int* x is c++ style, int *x is c style
bool adj_val (int32_t* variable, int32_t modify, int32_t low_limit, int32_t high_limit) {  // sets an int reference to new val constrained to given range
    int32_t oldval = *variable;
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify;
    return (*variable != oldval);
}
bool adj_val (float* variable, float modify, float low_limit, float high_limit) {  // sets an int reference to new val constrained to given range
    float oldval = *variable;
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
    return (*variable != oldval);
}
void adj_bool (bool* val, int32_t delta) { if (delta != 0) *val = (delta > 0); }  // sets a bool reference to 1 on 1 delta or 0 on -1 delta 

// pin operations that first check if pin exists for the current board
void set_pin (int32_t pin, int32_t mode) { if (pin >= 0 && pin != 255) pinMode (pin, mode); }
void write_pin (int32_t pin, int32_t val) {  if (pin >= 0 && pin != 255) digitalWrite (pin, val); }
int32_t read_pin (int32_t pin) { return (pin >= 0 && pin != 255) ? digitalRead (pin) : -1; }

// battery_v = convert_units ((float)analogRead (battery_pin), battery_convert_v_per_adc, battery_convert_invert);
// ema_filt (battery_v, &battery_filt_v, battery_ema_alpha);  // Apply EMA filter
bool read_battery_ignition (void) {  //Updates battery voltage and returns ignition on/off
    battery_adc = analogRead (ign_batt_pin);
    battery_v = convert_units (battery_adc, battery_convert_v_per_adc, battery_convert_invert);
    ema_filt (battery_v, &battery_filt_v, battery_ema_alpha);  // Apply EMA filter
    return (battery_filt_v > ignition_on_thresh_v);
}
void syspower_set (bool val) {
    if (digitalRead (syspower_pin) != val) {
        write_pin (syspower_pin, val);
        // delay (val * 500);
    }
}
long temp_peef (void) {  // Peef's algorithm somewhat modified by Soren
    long temp_temp;
    if (tempTimer.expired()) {  // if (millis() % 1000 == 0)
        if (temp_state == READ) {
            // delay(10);
            if (abs(temp_last-temp_temp) >= 20 && temp_last != 0) {
                // Serial.println("Bad-T-" + String(f));
                temp_temp = temp_last;
            }
            onewire.reset();
            onewire.write(0xCC);  // All Devices present - Skip ROM ID
            onewire.write(0x44);  // start conversion, with parasite power on at the end
            // printf ("\nTemp: %s.%s °F\n", String(temp/10), String(temp%10));
            tempTimer.set (temp_times_us[temp_state]);
            temp_state = CONVERT;
            return temp_temp;
        }  // else CONVERT
        onewire.reset();
        onewire.write(0xCC);  // All Devices present - Skip ROM ID
        onewire.write(0xBE);  // Read Scratchpad
        temp_data[0] = onewire.read();
        temp_data[1] = onewire.read();
        temp_raw = (temp_data[1] << 8) | temp_data[0];
        temp_last = temp_temp;
        temp_temp_addr_peef = ((long)temp_raw * 180 / 16 + 3205) / 10;
        temp_secs++;
        tempTimer.set (temp_times_us[temp_state]);
        temp_state = READ;
    }
    return 9930;  // Otherwise just return the temperature of the surface of the sun
}

void temp_soren (void) {
    if (temp_detected_device_ct && tempTimer.expired()) {
        if (temp_state == CONVERT) {
            tempsensebus.requestTemperatures();
            tempTimer.set (tempsensebus.microsToWaitForConversion(temperature_precision));  // 50 us . / (1 << (12 - temperature_precision)));  // Give some time before reading temp
            temp_state = READ;
        }
        else if (temp_state == READ) {
            temps_f[temp_current_index] = tempsensebus.getTempF(temp_addrs[temp_current_index]);  // 12800 us
            tempTimer.set (temp_timeout_us);
            temp_state = CONVERT;
            ++temp_current_index %= temp_detected_device_ct;
        }
    }
}

// I2C related
int32_t i2c_devicecount = 0;
uint8_t i2c_addrs[10];

void i2c_init (int32_t sda, int32_t scl) {
    printf ("I2C driver ");
    Wire.begin (sda, scl);  // I2c bus needed for airflow sensor
    byte error, address;
    printf (" scanning ...");
    i2c_devicecount = 0;
    for (address = 1; address < 127; address++ ) {
        Wire.beginTransmission (address);
        error = Wire.endTransmission();
        if (error == 0) {
            printf (" found addr: 0x%s%x", (address < 16) ? "0" : "", address);
            i2c_addrs[i2c_devicecount++] = address;
        }
        else if (error==4) printf (" error addr: 0x%s%x", (address < 16) ? "0" : "", address);
    }
    if (i2c_devicecount == 0) printf (" no devices found\n");
    else printf (" done\n");
}

// TaskHandle_t Task1;
// void codeForTask1 (void * parameter) {
//     for(;;) {
//         if (tempTimer.expired()) {
//             int32_t start = mycros();
//             tempsensebus.setWaitForConversion (false);  // makes it async
//             tempsensebus.requestTemperatures();
//             int32_t mid = mycros();
//             temps[0] = tempsensebus.getTempCByIndex(0);
//             int32_t done = mycros();
//             printf ("Temp: %lf, took %ld + %ld = %ld us.\n", temps[0], mid-start, done-mid, done-start);
//             tempTimer.reset();
//         }
//     }
// }
#endif  // GLOBALS_H