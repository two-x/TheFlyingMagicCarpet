#ifndef GLOBALS_H
#define GLOBALS_H
#include <SdFat.h>  // SD card & FAT filesystem library
#include <Servo.h>  // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <Adafruit_NeoPixel.h>  // Plan to allow control of neopixel LED onboard the esp32
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"
#include <Preferences.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <string>
#include <iomanip>
// #include <stdio.h>  // MCPWM pulse measurement code
// #include "freertos/FreeRTOS.h"  // MCPWM pulse measurement code
// #include "freertos/task.h"  // MCPWM pulse measurement code
// #include "driver/mcpwm.h"  // MCPWM pulse measurement code

// #include "classes.h"
#include "qpid.h"
// #include "spid.h"
// #include "disp.h"

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

// Defines for all the GPIO pins we're using
#define button_pin 0  // (button0 / strap to 1) - This is the left "Boot" button on the esp32 board
#define joy_horz_pin 1  // (adc) - Either analog left-right input (joystick)
#define joy_vert_pin 2  // (adc) - Either analog up-down input (joystick)
#define tft_dc_pin 3  // (strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
#define battery_pin 4  // (adc) -  Analog input, mule battery voltage level, full scale is 15.638V
#define pot_wipe_pin 5  // (adc) - Analog in from 20k pot. Use 1% series R=22k to 5V on wipe=CW-0ohm side, and R=15k to gnd on wipe-CCW-0ohm side. Gives wipe range of 1.315V (CCW) to 3.070V (CW) with 80 uA draw.
#define brake_pos_pin 6  // (adc) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#define pressure_pin 7  // (adc) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
#ifdef CAP_TOUCH
    #define i2c_sda_pin 8  // (i2c0 sda / adc) - Hijack these pins for the touchscreen and micro-sd i2c bus
    #define i2c_scl_pin 9  // (i2c0 scl / adc) - Hijack these pins for the touchscreen and micro-sd i2c bus
#else
    #define touch_irq_pin 8  // (i2c0 scl / adc) - With resistive touchscreen this pin is freed up
    #define touch_cs_pin 9  // (i2c0 scl / adc) - Use as chip select for resistive touchscreen
#endif
#define tft_cs_pin 10  // (spi0 cs) -  Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
#define tft_mosi_pin 11  // (spi0 mosi) - Used as spi interface data to sd card and tft screen
#define tft_sclk_pin 12  // (spi0 sclk) - Used as spi interface clock for sd card and tft screen
#define tft_miso_pin 13  // (spi0 miso) - Used as spi interface data from sd card and possibly (?) tft screen
#define steer_pwm_pin 14  // (pwm0) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
#define brake_pwm_pin 15  // (pwm1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
#define gas_pwm_pin 16  // (pwm1) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
#define hotrc_ch1_horz_pin 17  // (pwm0 / tx1) - Hotrc Ch1 thumb joystick input.
#define hotrc_ch2_vert_pin 18  // (pwm0 / rx1) - Hotrc Ch2 bidirectional trigger input
#define onewire_pin 19  // (usb-otg) - Onewire bus for temperature sensor data
#define hotrc_ch3_ign_pin 20  // (usb-otg) - Ignition control, Hotrc Ch3 PWM toggle signal
#define hotrc_ch4_cruise_pin 21  // (pwm0) - Cruise control, Hotrc Ch4 PWM toggle signal
#define tach_pulse_pin 35  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
#define speedo_pulse_pin 36  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
#define ignition_pin 37  // (spi-ram / oct-spi) - Output flips a relay to kill the car ignition, active high (no pullup)
#define syspower_pin 38  // (spi-ram / oct-spi) - Output, flips a relay to power all the tranducers
#define tft_rst_pin 39  // TFT Reset allows us to reboot the screen when it crashes
#define encoder_b_pin 40  // Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
#define encoder_a_pin 41  // Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
#define encoder_sw_pin 42  // Input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
#define joy_ign_btn_pin 43  // (uart0 tx) - Joystick ignition button. Reserve for possible jaguar interface
#define joy_cruise_btn_pin 44  // (uart0 rx) - Joystick cruise button. Reserve for possible jaguar interface
#define starter_pin 45  // (strap to 0) - Input, active high when vehicle starter is engaged (needs pulldown)
#define basicmodesw_pin 46  // (strap X) - Input, asserted to tell us to run in basic mode, active low (needs pullup)
#define sdcard_cs_pin 47  // Output, chip select allows SD card controller chip use of the SPI bus, active low
#define neopixel_pin 48  // (rgb led) - Data line to onboard Neopixel WS281x

// #define ctrl_horz_ch1_pin 1  // (adc) - Either analog left-right input (joystick), or Hotrc Ch1 thumb joystick PWM signal.
// #define ctrl_vert_ch2_pin 2  // (adc) - Either analog up-down input (joystick), or Hotrc Ch2 bidirectional trigger signal.
// #define unused 17  // (pwm0 / tx1) - 
// #define unused 18  // (pwm0 / rx1) -  

#define tp_irq_pin -1  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
#define tft_ledk_pin -1  // (spi-ram / oct-spi) - Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define encoder_pwr_pin -1
#define led_rx_pin -1  // Unused on esp32
#define led_tx_pin -1  // Unused on esp32
#define heartbeat_led_pin -1

#define adcbits 12
#define adcrange_adc 4095  // = 2^adcbits-1
#define adcmidscale_adc 2047  // = 2^(adcbits-1)-1

// Global settings
bool serial_debugging = true; 
bool timestamp_loop = false;  // Makes code write out timestamps throughout loop to serial port
bool take_temperatures = true;

// Persistent config storage
Preferences config;
    
// Readily available possibilities we could wire up if we want
//
// * Status LEDs (digital out)
// * Control of steering or brake motor coast vs. brake
// * CAN bus as a superior interface to brake and steering Jaguars (only on Due I think?)
// * Steering limit switches left and right, handle here instead of in Jaguar (digital in)
// * Engine temperature module overheat panic input (digital in)
// * Remote E-Stop panic inputs (digital in)
// * Serial interface to the lighting controller (if we can think of a reason)
// * Mule starter (digital out)
// * E-brake handle position (digital in)

// Globals -------------------
//
class Timer {  // 32 bit microsecond timer overflows after 71.5 minutes
  protected:
    volatile uint32_t start_us = 0;
    volatile uint32_t timeout_us = 0;
    volatile uint32_t remain_us;
    volatile bool enabled = true;
  public:
    Timer (void) { reset(); }
    Timer (uint32_t arg_timeout_us) { set (arg_timeout_us); }
    IRAM_ATTR void set (uint32_t arg_timeout_us) { timeout_us = arg_timeout_us; reset(); }
    IRAM_ATTR void reset (void) { start_us = esp_timer_get_time(); remain_us = timeout_us; }
    IRAM_ATTR void pause (void) { remain_us = remain(); enabled = false; }
    IRAM_ATTR void resume (void) { start_us = esp_timer_get_time() - remain_us; enabled = true; }
    IRAM_ATTR bool expired (void) { return (enabled) ? (esp_timer_get_time() >= start_us + timeout_us): false; }
    IRAM_ATTR uint32_t elapsed (void) { return (enabled) ? (esp_timer_get_time() - start_us) : (timeout_us - remain_us); }
    IRAM_ATTR uint32_t remain (void) { return (enabled) ? ((start_us + timeout_us) - esp_timer_get_time()) : remain_us; }
    IRAM_ATTR uint32_t get_timeout (void) { return timeout_us; }
    IRAM_ATTR bool get_enabled (void) { return enabled; }
};

double convert_units (double from_units, double convert_factor, bool invert, double in_offset = 0.0, double out_offset = 0.0) {
    if (!invert) return out_offset + convert_factor * (from_units - in_offset);
    if (from_units - in_offset) return out_offset + convert_factor / (from_units - in_offset);
    printf ("convert_units refused to divide by zero: %lf, %lf, %d, %lf, %lf", from_units, convert_factor, invert, in_offset, out_offset);
    return -1;
}

// run state globals
enum runmodes { BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL };
int32_t runmode = SHUTDOWN;
int32_t oldmode = BASIC;  // So we can tell when the mode has just changed. start as different to trigger_mode start algo
int32_t gesture_progress = 0;  // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
bool shutdown_complete = false;  // Shutdown mode has completed its work and can stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;  // Indicates we should release the brake & gas so the pedals can be used manually without interference
bool calmode_request = false;
bool panic_stop = false;
bool cruise_gesturing = false;  // Is cruise mode enabled by gesturing?  Otherwise by press of cruise button
bool cruise_sw_held = false;
bool cruise_adjusting = false;
Timer gestureFlyTimer;  // Used to keep track of time for gesturing for going in and out of fly/cruise modes
Timer cruiseSwTimer;
Timer sleepInactivityTimer (10000000);  // After shutdown how long to wait before powering down to sleep
Timer stopcarTimer (7000000);  // Allows code to fail in a sensible way after a delay if nothing is happening
//  ---- tunable ----
uint32_t motor_park_timeout_us = 4000000;  // If we can't park the motors faster than this, then give up.
uint32_t gesture_flytimeout_us = 500000;  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
uint32_t cruise_sw_timeout_us = 500000;  // how long do you have to hold down the cruise button to start cruise mode (in us)
Timer motorParkTimer(motor_park_timeout_us);

// calibration related
bool cal_joyvert_brkmotor = false;  // Allows direct control of brake motor using controller vert
bool cal_pot_gasservo = false;  // Allows direct control of gas servo using pot
bool cal_pot_gas_ready = false;  // To avoid immediately overturning gas pot, first pot must be turned to valid range
bool cal_set_hotrc_failsafe_ready = false;  

// generic values
//  ---- tunable ----
int32_t default_margin_adc = 12;  // Default margin of error for comparisons of adc values (ADC count 0-4095)

// pid related globals
//  ---- tunable ----
uint32_t steer_pid_period_ms = 185;  // (Not actually a pid) Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer steerPidTimer (steer_pid_period_ms*1000);  // not actually tunable, just needs value above
uint32_t brake_pid_period_ms = 185;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer brakePidTimer (brake_pid_period_ms*1000);  // not actually tunable, just needs value above
// int32_t brake_spid_ctrl_dir = SPID::REV;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value
double brake_spid_initial_kp = 2.18;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
double brake_spid_initial_ki_hz = 0.215;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
double brake_spid_initial_kd_s = 1.130;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
uint32_t cruise_pid_period_ms = 300;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer cruisePidTimer (cruise_pid_period_ms*1000);  // not actually tunable, just needs value above
double cruise_spid_initial_kp = 0.157;  // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
double cruise_spid_initial_ki_hz = 0.035;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
double cruise_spid_initial_kd_s = 0.044;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
// int32_t cruise_spid_ctrl_dir = SPID::FWD;  // 1 = fwd, 0 = rev.
uint32_t gas_pid_period_ms = 225;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer gasPidTimer (gas_pid_period_ms*1000);  // not actually tunable, just needs value above
double gas_spid_initial_kp = 0.245;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
double gas_spid_initial_ki_hz = 0.015;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
double gas_spid_initial_kd_s = 0.022;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
// int32_t gas_spid_ctrl_dir = SPID::REV;  // 0 = fwd, 1 = rev.
bool gas_open_loop = true;
// starter related
bool starter = LOW;
bool starter_last = LOW;
bool sim_starter = false;

// mule battery related
double battery_adc = adcmidscale_adc;
double battery_v = 10.0;
double battery_filt_v = 10.0;
//  ---- tunable ----
double battery_max_v = 16.0;  // The max vehicle voltage we can sense. Design resistor divider to match. Must exceed max V possible.
double battery_convert_v_per_adc = battery_max_v/adcrange_adc;
bool battery_convert_invert = false;
int32_t battery_convert_polarity = 1;  // Forward
double battery_ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 

// potentiometer related
double pot_percent = 50;
double pot_filt_percent = pot_percent;
double pot_min_percent = 0;  //
double pot_max_percent = 100;  //
//  ---- tunable ----
double pot_min_adc = 0;  // TUNED 230603 - Used only in determining theconversion factor
double pot_max_adc = 4090;  // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
double pot_convert_percent_per_adc = (pot_max_percent - pot_min_percent)/(pot_max_adc - pot_min_adc);  // 100 % / (3996 adc - 0 adc) = 0.025 %/adc
bool pot_convert_invert = false;
double pot_convert_offset = -0.08;
int32_t pot_convert_polarity = 1;  // Forward
double pot_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 

// controller related
enum ctrls { HOTRC, JOY, SIM };  // Possible sources of gas, brake, steering commands
enum ctrl_axes { HORZ, VERT };
enum ctrl_thresh { MIN, DB, MAX };
enum ctrl_edge { BOT, TOP };
enum raw_filt { RAW, FILT };
bool joy_centered = false;
int32_t ctrl_db_adc[2][2];  // [HORZ/VERT] [BOT/TOP] - to store the top and bottom deadband values for each axis of selected controller
int32_t ctrl_pos_adc[2][2] = { { adcmidscale_adc, adcmidscale_adc }, { adcmidscale_adc, adcmidscale_adc} };  // [HORZ/VERT] [RAW/FILT]
volatile bool hotrc_vert_preread = 0;
volatile bool hotrc_ch3_sw, hotrc_ch4_sw, hotrc_ch3_sw_event, hotrc_ch4_sw_event, hotrc_ch3_sw_last, hotrc_ch4_sw_last;
volatile int32_t hotrc_horz_pulse_us = 1500;
volatile int32_t hotrc_vert_pulse_us = 1500;
Timer hotrcPulseTimer;  // OK to not be volatile?
// Merging these into Hotrc class
bool hotrc_radio_detected = false;
bool hotrc_radio_detected_last = hotrc_radio_detected;
bool hotrc_suppress_next_ch3_event = true;  // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
bool hotrc_suppress_next_ch4_event = true;  // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
//  ---- tunable ----
double hotrc_pulse_period_us = 1000000.0 / 50;
double ctrl_ema_alpha[2] = { 0.01, 0.1 };  // [HOTRC/JOY] alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t ctrl_lims_adc[2][2][3] = { { { 3, 375, 4092 }, { 3, 375, 4092 } }, { { 9, 200, 4085 }, { 9, 200, 4085 } }, }; // [HOTRC/JOY] [HORZ/VERT], [MIN/DEADBAND/MAX] values as ADC counts
bool ctrl = HOTRC;  // Use HotRC controller to drive instead of joystick?
// Limits of what pulsewidth the hotrc receiver puts out
// For some funky reason I was unable to initialize these in an array format !?!?!
// int32_t hotrc_pulse_lims_us[2][2];  // = { { 1009, 2003 }, { 1009, 2003 } };  // [HORZ/VERT] [MIN/MAX]  // These are the limits of hotrc vert and horz high pulse
int32_t hotrc_pulse_vert_min_us = 990;  // 1009;
int32_t hotrc_pulse_vert_max_us = 1990;  // 2003;
int32_t hotrc_pulse_horz_min_us = 990;  // 1009;
int32_t hotrc_pulse_horz_max_us = 1990;  // 2003;

// Maybe merging these into Hotrc class
int32_t hotrc_pos_failsafe_min_adc = 140;  // The failsafe setting in the hotrc must be set to a trigger level equal to max amount of trim upward from trigger released.
int32_t hotrc_pos_failsafe_max_adc = 320;
int32_t hotrc_pos_failsafe_pad_adc = 10;
uint32_t hotrc_panic_timeout = 1000000;  // how long to receive flameout-range signal from hotrc vertical before panic stopping
Timer hotrcPanicTimer(hotrc_panic_timeout);

// steering related
int32_t steer_pulse_safe_us = 0;
int32_t steer_pulse_out_us;  // pid loop output to send to the actuator (steering)
//  ---- tunable ----
int32_t steer_pulse_right_min_us = 500;  // Smallest pulsewidth acceptable to jaguar
int32_t steer_pulse_right_us = 800;  // Steering pulsewidth corresponding to full-speed right steering (in us)
int32_t steer_pulse_stop_us = 1500;  // Steering pulsewidth corresponding to zero steering motor movement (in us)
int32_t steer_pulse_left_us = 2200;  // Steering pulsewidth corresponding to full-speed left steering (in us)
int32_t steer_pulse_left_max_us = 2500;  // Longest pulsewidth acceptable to jaguar
int32_t steer_safe_percent = 72;  // Sterring is slower at high speed. How strong is this effect 

// brake pressure related
int32_t pressure_adc;
// AnalogSensor pressure (&pressure_adc, "Pressure:", "adc ", 658, 2100);
//  ---- tunable ----
int32_t pressure_min_adc = 658; // Sensor reading when brake fully released.  230430 measured 658 adc (0.554V) = no brakes
int32_t pressure_sensor_max_adc = adcrange_adc; // Sensor reading max, limited by adc Vmax. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as chris can push (wimp)
int32_t pressure_max_adc = 2080; // Sensor measured maximum reading. (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as [wimp] chris can push
double pressure_convert_psi_per_adc = 1000.0 * (3.3 - 0.554) / ( (pressure_sensor_max_adc - pressure_min_adc) * (4.5 - 0.554) );  // 1000 psi * (adc_max v - v_min v) / ((4095 adc - 658 adc) * (v-max v - v-min v)) = 0.2 psi/adc 
bool pressure_convert_invert = false;
// int32_t pressure_convert_polarity = SPID::FWD;
double pressure_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double pressure_margin_psi = 2.5;  // Margin of error when comparing brake pressure adc values (psi)
double pressure_min_psi = 0.0;  // TUNED 230602 - Brake pressure when brakes are effectively off. Sensor min = 0.5V, scaled by 3.3/4.5V is 0.36V of 3.3V (ADC count 0-4095). 
double pressure_max_psi = convert_units (pressure_max_adc - pressure_min_adc, pressure_convert_psi_per_adc, pressure_convert_invert);  // TUNED 230602 - Highest possible pressure achievable by the actuator 
double pressure_hold_initial_psi = 150;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
double pressure_hold_increment_psi = 15;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
double pressure_panic_initial_psi = 250;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
double pressure_panic_increment_psi = 25;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
// max pedal bent 1154
double pressure_psi = (pressure_min_psi+pressure_max_psi)/2;
double pressure_filt_psi = pressure_psi;  // Stores new setpoint to give to the pid loop (brake)
double pressure_target_psi;

// brake actuator motor related
double brake_pulse_out_us;  // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
//  ---- tunable ----
Timer brakeIntervalTimer (500000);  // How much time between increasing brake force during auto-stop if car still moving?
int32_t brake_increment_interval_us = 500000;  // How often to apply increment during auto-stopping (in us)
int32_t brake_pulse_retract_min_us = 500;  // Smallest pulsewidth acceptable to jaguar
int32_t brake_pulse_retract_us = 650;  // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us)
int32_t brake_pulse_stop_us = 1500;  // Brake pulsewidth corresponding to center point where motor movement stops (in us)
int32_t brake_pulse_extend_us = 2350;  // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us)
int32_t brake_pulse_extend_max_us = 2500;  // Longest pulsewidth acceptable to jaguar
int32_t brake_pulse_margin_us = 40; // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated 

// brake actuator position related
double brake_pos_in;
double brake_pos_filt_in;
//  ---- tunable ----
double brake_pos_convert_in_per_adc = 3.3 * 10000.0 / (5.0 * adcrange_adc * 557);  // 3.3 v * 10k ohm / (5 v * 4095 adc * 557 ohm/in) = 0.0029 in/adc = 2.89 m-in/adc 
bool brake_pos_convert_invert = false;
int32_t brake_pos_convert_polarity = 1;  // Forward
double brake_pos_ema_alpha = 0.25;
double brake_pos_abs_min_retract_in = 0.335;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. ("in"sandths of an inch)
double brake_pos_nom_lim_retract_in = 0.506;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (in)
double brake_pos_zeropoint_in = 3.179;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (in)
double brake_pos_park_in = 4.234;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (in)
double brake_pos_nom_lim_extend_in = 4.624;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (in)
double brake_pos_abs_max_extend_in = 8.300;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (in)
double brake_pos_margin_in = .029;  //
// int32_t brake_pos_abs_min_retract_adc = 116;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. (ADC count 0-4095)
// int32_t brake_pos_nom_lim_retract_adc = 175;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
// int32_t brake_pos_zeropoint_adc = 1100;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (ADC count 0-4095)
// int32_t brake_pos_park_adc = 1465;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (ADC count 0-4095)
// int32_t brake_pos_nom_lim_extend_adc = 1600;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
// int32_t brake_pos_abs_max_extend_adc = 2872;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (ADC count 0-4095)
// int32_t brake_pos_margin_adc = 10;  //    

// throttle servo related
int32_t gas_pulse_out_us = 1501;  // pid loop output to send to the actuator (gas)
int32_t gas_pulse_govern_us = 1502;  // Governor must scale the pulse range proportionally. This is given a value in the loop
//  ---- tunable ----
Timer gasServoTimer (500000);  // We expect the servo to find any new position within this time
int32_t gas_governor_percent = 95;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
int32_t gas_pulse_cw_min_us = 1000;  // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
int32_t gas_pulse_redline_us = 1400;  // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
int32_t gas_pulse_idle_us = 1800;  // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
int32_t gas_pulse_ccw_max_us = 2000;  // Servo ccw limit pulsewidth. Hotrc controller ch1/2 min(lt/br) = 1000us, center = 1500us, max(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
int32_t gas_pulse_park_slack_us = 30;  // Gas pulsewidth beyond gas_pulse_idle_us where to park the servo out of the way so we can drive manually (in us)

// tachometer related
Timer tachPulseTimer;  // OK to not be volatile?
volatile int32_t tach_delta_us = 0;
volatile int32_t tach_buf_delta_us = 0;
volatile uint32_t tach_time_us;
double tach_target_rpm;
double tach_rpm = 50.0;  // Current engine speed, raw value converted to rpm (in rpm)
double tach_filt_rpm = 50.0;  // Current engine speed, filtered (in rpm)
double tach_govern_rpm;  // Software engine governor creates an artificially reduced maximum for the engine speed. This is given a value in calc_governor()
//  ---- tunable ----
double tach_convert_rpm_per_rpus = 60.0 * 1000000.0;  // 1 rot/us * 60 sec/min * 1000000 us/sec = 60000000 rot/min
bool tach_convert_invert = true;
int32_t tach_convert_polarity = 1;  // Forward      
double tach_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double tach_idle_rpm = 700.0;  // Min value for engine hz, corresponding to low idle (in rpm)
double tach_max_rpm = 6000.0;  // Max possible engine rotation speed
double tach_redline_rpm = 4000.0;  // Max value for tach_rpm, pedal to the metal (in rpm)
double tach_margin_rpm = 15.0;  // Margin of error for checking engine rpm (in rpm)
double tach_stop_thresh_rpm = 0.01;  // Below which the engine is considered stopped - this is redundant,
int32_t tach_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
int32_t tach_delta_abs_min_us = 6500;  // 6500 us corresponds to about 10000 rpm, which isn't possible. Use to reject retriggers

// carspeed/speedo related
double speedo_target_mph;
double speedo_govern_mph;  // Governor must scale the top vehicle speed proportionally. This is given a value in the loop
double speedo_mph = 1.01;  // Current car speed, raw as sensed (in mph)
double speedo_filt_mph = 1.02;  // Current car speed, filtered (in mph)
Timer speedoPulseTimer;  // OK to not be volatile?
volatile int32_t speedo_delta_us = 0;
volatile int32_t speedo_buf_delta_us = 0;
volatile uint32_t speedo_time_us;
//  ---- tunable ----
double speedo_convert_mph_per_rpus = 1000000.0 * 3600.0 * 20 * 3.14159 / (19.85 * 12 * 5280);  // 1 rot/us * 1000000 us/sec * 3600 sec/hr * 1/19.85 gearing * 20*pi in/rot * 1/12 ft/in * 1/5280 mi/ft = 179757 mi/hr (mph)
// Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
bool speedo_convert_invert = true;
int32_t speedo_convert_polarity = 1;  // Forward      
double speedo_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double speedo_idle_mph = 4.50;  // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)
double speedo_redline_mph = 15.0;  // What is our steady state speed at redline? Pulley rotation frequency (in milli-mph)
double speedo_max_mph = 25.0;  // What is max speed car can ever go
double speedo_stop_thresh_mph = 0.01;  // Below which the car is considered stopped
uint32_t speedo_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the car is stopped (in us)
int32_t speedo_delta_abs_min_us = 4500;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers
            
// neopixel and heartbeat related
uint8_t neo_wheelcounter = 0;
uint8_t neo_brightness_max = 21;
uint32_t neo_timeout = 150000;
Timer neoTimer (neo_timeout);
bool neo_heartbeat = (neopixel_pin >= 0);
uint8_t neo_brightness = neo_brightness_max;  // brightness during fadeouts
enum neo_colors { N_RED, N_GRN, N_BLU };
uint8_t neo_heartcolor[3] = { 0xff, 0xff, 0xff };
Timer heartbeatTimer (1000000);
int32_t heartbeat_state = 0;
int32_t heartbeat_level = 0;
int32_t heartbeat_ekg[4] = { 170000, 150000, 530000, 1100000 };
int32_t heartbeat_pulse = 255;

// diag/monitoring variables
Timer loopTimer (1000000);  // how long the previous main loop took to run (in us)
uint32_t loop_period_us = 10000;
double loop_freq_hz = 1;  // run loop real time frequency (in Hz)
volatile int32_t loop_int_count = 0;  // counts interrupts per loop
int32_t loopno = 1;
uint32_t looptimes_us[20];
bool loop_dirty[20];
int32_t loopindex = 0;
bool booted = false;
bool diag_ign_error_enabled = true;

// pushbutton related
enum sw_presses { NONE, SHORT, LONG };  // used by encoder sw and button algorithms
bool button_last = 0;
bool button_it = 0;
bool btn_press_timer_active = false;
bool btn_press_suppress_click = false;
bool btn_press_action = NONE;

// external signal related
bool ignition = LOW;
bool ignition_last = ignition;
bool ignition_output_enabled = false;  // disallows configuration of ignition pin as an output until hotrc detected
bool syspower = HIGH;
bool syspower_last = syspower;
bool basicmodesw = LOW;
bool cruise_sw = LOW;

// simulator related
bool simulating_last = false;
Timer simTimer;
int32_t sim_edit_delta = 0;
int32_t sim_edit_delta_touch = 0;
int32_t sim_edit_delta_encoder = 0;
//  ---- tunable ----
bool simulating = false;
bool sim_joy = false;
bool sim_tach = true;
bool sim_speedo = true;
bool sim_brkpos = false;
bool sim_basicsw = true;
bool sim_cruisesw = true;
bool sim_pressure = true;
bool sim_syspower = true;
bool pot_pressure = true;  // Use the pot to simulate the brake pressure

SdFat sd;  // SD card filesystem
#define approot "cantroller2020"
#define logfile "log.txt"
#define error(msg) sd.errorHalt(F(msg))  // Error messages stored in flash.
SdFile root;  // Directory file.
SdFile file;  // Use for file creation in folders.

// SPID brakeSPID (&pressure_filt_psi, brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s, brake_spid_ctrl_dir, brake_pid_period_ms);
// SPID gasSPID (&tach_filt_rpm, gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s, gas_spid_ctrl_dir, gas_pid_period_ms);
// SPID cruiseSPID (&speedo_filt_mph, cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s, cruise_spid_ctrl_dir, cruise_pid_period_ms);

QPID brakeQPID (&pressure_filt_psi, &brake_pulse_out_us, &pressure_target_psi);
QPID gasQPID (&tach_filt_rpm, &gas_pulse_out_us, &tach_target_rpm);
QPID cruiseQPID (&speedo_filt_mph, &tach_target_rpm, &speedo_target_mph);

// brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s, brake_spid_ctrl_dir, brake_pid_period_ms);

// SPID gasSPID (&tach_filt_rpm, gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s, gas_spid_ctrl_dir, gas_pid_period_ms);
// SPID cruiseSPID (&speedo_filt_mph, cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s, cruise_spid_ctrl_dir, cruise_pid_period_ms);

// Servo library lets us set pwm outputs given an on-time pulse width in us
static Servo steer_servo;
static Servo brake_servo;
static Servo gas_servo;
static Adafruit_NeoPixel neostrip(1, neopixel_pin, NEO_GRB + NEO_GRB + NEO_KHZ800);

// Temperature sensor related
double temp_min = -67.0;  // Minimum reading of sensor is -25 C = -67 F
double temp_max = 257.0;  // Maximum reading of sensor is 125 C = 257 F
double temp_room = 77.0;  // "Room" temperature is 25 C = 77 F
enum temp_sensors { AMBIENT, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };
Timer tempTimer (2000000);
enum temp_status { IDLE, CONVERT, DELAY };
int32_t temp_status = IDLE;
double temps[6];
int32_t temp_detected_device_ct = 0;
int32_t temperature_precision = 12;  // 9-12 bit resolution
OneWire onewire (onewire_pin);
DallasTemperature tempsensebus (&onewire);
DeviceAddress temp_temp_addr;
int32_t temp_current_index = 0;
DeviceAddress temp_addrs[6];

// Interrupt service routines
//
// The tach and speed use a hall sensor being triggered by a passing magnet once per pulley turn. These ISRs call mycros()
// on every pulse to know the time since the previous pulse. I tested this on the bench up to about 0.750 mph which is as 
// fast as I can move the magnet with my hand, and it works. Update: Janky bench test appeared to work up to 11000 rpm.
void IRAM_ATTR tach_isr (void) {  // The tach and speedo isrs get the period of the vehicle pulley rotations.
    tach_time_us = tachPulseTimer.elapsed();
    if (tach_time_us > tach_delta_abs_min_us) {  // ignore spurious triggers or bounces
        tachPulseTimer.reset();
        tach_delta_us = tach_time_us;
    }
}
void IRAM_ATTR speedo_isr (void) {  //  Handler can get the most recent rotation time at speedo_delta_us
    speedo_time_us = speedoPulseTimer.elapsed();
    if (speedo_time_us > speedo_delta_abs_min_us) {  // ignore spurious triggers or bounces
        speedoPulseTimer.reset();
        speedo_delta_us = speedo_time_us;    
    }
}

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

void IRAM_ATTR hotrc_vert_isr (void) {  // On falling edge, records high pulse width to determine ch2 steering slider position
    if (hotrc_vert_preread) hotrcPulseTimer.reset();
    else hotrc_vert_pulse_us = hotrcPulseTimer.elapsed();
    hotrc_vert_preread = !(digitalRead (hotrc_ch2_vert_pin));  // Read pin after timer operations to maximize clocking accuracy
}
void IRAM_ATTR hotrc_horz_isr (void) {  // On falling edge, records high pulse width to determine ch2 steering slider position
    hotrc_horz_pulse_us = hotrcPulseTimer.elapsed();
}
void IRAM_ATTR hotrc_ch3_isr (void) {  // On falling edge, records high pulse width to determine ch3 button toggle state
    hotrc_ch3_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch3 switch true if short pulse, otherwise false
    if (hotrc_ch3_sw != hotrc_ch3_sw_last) hotrc_ch3_sw_event = true;  // So a handler routine can be signaled. Handler must reset this to false
    hotrc_ch3_sw_last = hotrc_ch3_sw;
}
void IRAM_ATTR hotrc_ch4_isr (void) {  // On falling edge, records high pulse width to determine ch4 button toggle state
    hotrc_ch4_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch4 switch true if short pulse, otherwise false
    if (hotrc_ch4_sw != hotrc_ch4_sw_last) hotrc_ch4_sw_event = true;  // So a handler routine can be signaled. Handler must reset this to false
    hotrc_ch4_sw_last = hotrc_ch4_sw;
}

// Utility functions
#define arraysize(x) ((int32_t)(sizeof(x) / sizeof((x)[0])))  // A macro function to determine the length of string arrays
#define floor(amt, lim) ((amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)
// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
// #define min(a, b) ( (a <= b) ? a : b)
// #define max(a, b) ( (a >= b) ? a : b)
#undef max
inline double max (double a, double b) { return (a > b) ? a : b; }
inline int32_t max (int32_t a, int32_t b) { return (a > b) ? a : b; }
#undef min
inline double min (double a, double b) { return (a < b) ? a : b; }
inline int32_t min (int32_t a, int32_t b) { return (a < b) ? a : b; }
#undef constrain
inline double constrain (double amt, double low, double high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline int32_t constrain (int32_t amt, int32_t low, int32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline uint32_t constrain (uint32_t amt, uint32_t low, uint32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
#undef map
inline double map (double x, double in_min, double in_max, double out_min, double out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
inline int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
bool rounding = true;
double dround (double val, int32_t digits) { return (rounding) ? (std::round(val * std::pow (10, digits)) / std::pow (10, digits)) : val; }

bool car_stopped (void) { return (speedo_filt_mph < speedo_stop_thresh_mph); }
bool engine_stopped (void) { return (tach_filt_rpm < tach_stop_thresh_rpm); }

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
void calc_deadbands (void) {
    ctrl_db_adc[VERT][BOT] = (adcrange_adc-ctrl_lims_adc[ctrl][VERT][DB])/2;  // Lower threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[VERT][TOP] = (adcrange_adc+ctrl_lims_adc[ctrl][VERT][DB])/2;  // Upper threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][BOT] = (adcrange_adc-ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Lower threshold of horz joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][TOP] = (adcrange_adc+ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Upper threshold of horz joy deadband (ADC count 0-4095)
}
void calc_governor (void) {
    tach_govern_rpm = map ((double)gas_governor_percent, 0.0, 100.0, 0.0, tach_redline_rpm);  // Create an artificially reduced maximum for the engine speed
    gas_pulse_govern_us = map ((int32_t)(gas_governor_percent*(tach_redline_rpm-tach_idle_rpm)/tach_redline_rpm), 0, 100, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
    speedo_govern_mph = map ((double)gas_governor_percent, 0.0, 100.0, 0.0, speedo_redline_mph);  // Governor must scale the top vehicle speed proportionally
}
// Exponential Moving Average filter : Smooth out noise on inputs. 0 < alpha < 1 where lower = smoother and higher = more responsive
// Pass in a fresh raw value, address of filtered value, and alpha factor, filtered value will get updated
void ema_filt (double raw, double* filt, double alpha) {
    // if (!raw) *filt = 0.0; else
    *filt = alpha * raw + (1 - alpha) * (*filt);
}
void ema_filt (int32_t raw, double* filt, double alpha) {
    ema_filt ((double)raw, filt, alpha);
}
void ema_filt (int32_t raw, int32_t* filt, double alpha) {
    *filt = (int32_t)(alpha * (double)raw + (1 - alpha) * (double)(*filt));
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
bool adj_val (double* variable, int32_t modify, double low_limit, double high_limit) {  // sets an int reference to new val constrained to given range
    double oldval = *variable;
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
    return (*variable != oldval);
}
bool adj_val (double* variable, double modify, double low_limit, double high_limit) {  // sets an int reference to new val constrained to given range
    double oldval = *variable;
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
    return (*variable != oldval);
}

void adj_bool (bool* val, int32_t delta) { if (delta != 0) *val = (delta > 0); }  // sets a bool reference to 1 on 1 delta or 0 on -1 delta 

// pin operations that first check if pin exists for the current board
void set_pin (int32_t pin, int32_t mode) { if (pin >= 0) pinMode (pin, mode); }
void write_pin (int32_t pin, int32_t val) {  if (pin >= 0) digitalWrite (pin, val); }
int32_t read_pin (int32_t pin) { return (pin >= 0) ? digitalRead (pin) : -1; }

// void enable_pids (int32_t en_brake, int32_t en_gas, int32_t en_cruise) {  // pass in 0 (disable), 1 (enable), or -1 (leave it alone) for each pid loop
//     if (en_brake != -1) brakeSPID.set_enable ((bool)en_brake);
//     if (en_gas != -1) gasSPID.set_enable ((bool)en_gas);
//     if (en_cruise != -1) cruiseSPID.set_enable ((bool)en_cruise);
// }

void syspower_set (bool val) {
    if (digitalRead (syspower_pin) != val) {
        write_pin (syspower_pin, val);
        // delay (val * 500);
    }
}

// double get_temp (DeviceAddress arg_addr) {  // function to print the temperature for a device
//     double tempF = tempsensebus.getTempF (arg_addr);
//     // if (tempF == DEVICE_DISCONNECTED_C) printf ("Error: Could not read temperature\n");
//     return tempF;
// }

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