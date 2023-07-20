#ifndef GLOBALS_H
#define GLOBALS_H
#include <SdFat.h>  // SD card & FAT filesystem library
#include <ESP32Servo.h>  // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <Adafruit_NeoPixel.h>  // Plan to allow control of neopixel LED onboard the esp32
#include <OneWire.h>
#include "temp.h"
// #include <DallasTemperature.h>
#include <Wire.h>
#include <SparkFun_FS3000_Arduino_Library.h>  // For airflow sensor  http://librarymanager/All#SparkFun_FS3000
#include "Arduino.h"
#include <Preferences.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <string>
// #include <sstream.h>  // For saving error strings to print on emptier loops
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
extern bool flip_the_screen;

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

// Global settings
extern bool serial_debugging;
extern bool timestamp_loop;
extern bool take_temperatures;

// Persistent config storage
extern Preferences config;
    
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
class Timer {
  protected:
    volatile int64_t start_us;
    volatile int64_t timeout_us;
    
  public:
    Timer();
    Timer(uint32_t arg_timeout_us);
    
    void set(int64_t arg_timeout_us);
    void set(uint32_t arg_timeout_us);
    
    void reset();
    
    bool expired();
    
    int64_t elapsed();
    int64_t get_timeout();
};

extern float convert_units(float from_units, float convert_factor, bool invert, float in_offset = 0.0, float out_offset = 0.0);

// run state globals
enum runmodes { BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL };

extern int32_t runmode;
extern int32_t oldmode;  

extern int32_t gesture_progress;

extern bool shutdown_complete; 
extern bool we_just_switched_modes;
extern bool park_the_motors;
extern bool car_initially_moved;
extern bool calmode_request;
extern bool panic_stop;
extern bool flycruise_toggle_request;

extern int32_t flycruise_vert_margin_adc;

extern bool cruise_gesturing;
extern bool cruise_sw_held;
extern bool cruise_adjusting;

extern Timer gestureFlyTimer;
extern Timer sleepInactivityTimer;
extern Timer stopcarTimer;

// ---- tunable ----  
extern uint32_t motor_park_timeout_us;
extern uint32_t gesture_flytimeout_us;
extern uint32_t cruise_sw_timeout_us;  
extern uint32_t cruise_antiglitch_timeout_us;

extern Timer cruiseAntiglitchTimer;
extern Timer motorParkTimer;

// calibration related
extern bool cal_joyvert_brkmotor;  
extern bool cal_pot_gasservo;
extern bool cal_pot_gas_ready;
extern bool cal_set_hotrc_failsafe_ready;

// pid related globals
// ---- tunable ----
extern uint32_t steer_pid_period_ms;  
extern Timer steerPidTimer;
extern uint32_t brake_pid_period_ms;
extern Timer brakePidTimer;
// extern int32_t brake_spid_ctrl_dir;
extern float brake_spid_initial_kp;
extern float brake_spid_initial_ki_hz;
extern float brake_spid_initial_kd_s;
extern uint32_t cruise_pid_period_ms;
extern Timer cruisePidTimer;
extern float cruise_spid_initial_kp;
extern float cruise_spid_initial_ki_hz;
extern float cruise_spid_initial_kd_s;
// extern int32_t cruise_spid_ctrl_dir;
extern uint32_t gas_pid_period_ms; 
extern Timer gasPidTimer;
extern float gas_spid_initial_kp;
extern float gas_spid_initial_ki_hz;
extern float gas_spid_initial_kd_s;
// extern int32_t gas_spid_ctrl_dir;
extern bool gas_open_loop;

// starter related
extern bool starter;
extern bool starter_last;

// extern bool sim_starter;


// mule battery related
extern float battery_adc; 
extern float battery_v;
extern float battery_filt_v;

// tunable
extern float battery_max_v;  
extern float battery_convert_v_per_adc;
extern bool battery_convert_invert;
extern int32_t battery_convert_polarity;
extern float battery_ema_alpha;

// potentiometer related  
extern float pot_percent;
extern float pot_filt_percent;
extern float pot_min_percent;
extern float pot_max_percent;

// tunable
extern int32_t pot_adc;
extern float pot_min_adc;
extern float pot_max_adc;
extern float pot_convert_percent_per_adc;
extern bool pot_convert_invert;
extern float pot_convert_offset; 
extern int32_t pot_convert_polarity;
extern float pot_ema_alpha;

// controller related
enum ctrls { HOTRC, JOY, SIM, HEADLESS };  // Possible sources of gas, brake, steering commands
enum ctrl_axes { HORZ, VERT, CH3, CH4 };
enum ctrl_thresh { MIN, CENT, MAX, DB };
enum ctrl_edge { BOT, TOP };
enum raw_filt { RAW, FILT };

extern bool joy_centered;

// extern Timer hotrcPulseTimer;  

extern bool hotrc_radio_detected;
extern bool hotrc_radio_detected_last;
extern bool hotrc_suppress_next_ch3_event;  
extern bool hotrc_suppress_next_ch4_event;

// tunable
extern float hotrc_pulse_period_us;
extern float ctrl_ema_alpha[2];  

extern int32_t ctrl_lims_adc[2][2][4];

extern bool ctrl;  

// extern int32_t hotrc_pulse_lims_us[2][3];

extern int32_t hotrc_pulse_lims_us[4][3];

extern int32_t hotrc_spike_buffer[2][3];

// extern float hotrc_mapratio[2][3];


extern volatile int64_t hotrc_timer_start;
extern volatile bool hotrc_ch3_sw, hotrc_ch4_sw, hotrc_ch3_sw_event, hotrc_ch4_sw_event, hotrc_ch3_sw_last, hotrc_ch4_sw_last;
extern volatile bool hotrc_isr_pin_preread;
extern volatile int64_t hotrc_horz_pulse_64_us;
extern volatile int64_t hotrc_vert_pulse_64_us;

extern int32_t hotrc_horz_pulse_us;
extern int32_t hotrc_vert_pulse_us;  

extern int32_t hotrc_horz_pulse_filt_us;
extern int32_t hotrc_vert_pulse_filt_us;

extern int32_t intcount;
extern int32_t ctrl_db_adc[2][2];  // [HORZ/VERT] [BOT/TOP] - to store the top and bottom deadband values for each axis of selected controller
extern int32_t ctrl_pos_adc[2][2];  // [HORZ/VERT] [RAW/FILT] - holds most current controller values

extern int32_t hotrc_pulse_failsafe_min_us;
extern int32_t hotrc_pulse_failsafe_max_us;

extern int32_t hotrc_pulse_failsafe_pad_us;  
extern uint32_t hotrc_panic_timeout_us;

extern Timer hotrcPanicTimer;


// hw_timer_t *hotrc_vert_timer = NULL;
// volatile uint32_t hotrc_vert_width_us = 0; 
// volatile bool hotrc_vert_pulse_started = false;
// volatile uint64_t hotrc_vert_pulse_start_us = 0;

// volatile int64_t hotrc_vert_pulse_us = (int64_t)hotrc_pulse_lims_us[VERT][CENT];
// int32_t hotrc_vert_pulse_filt_us = (int32_t)hotrc_vert_pulse_us;

// Maybe merging these into Hotrc class

// I2C related  
extern int32_t i2c_devicecount;
extern uint8_t i2c_addrs[10];

// airflow related
extern float airflow_mph;
extern float airflow_filt_mph;
extern float airflow_min_mph;
extern float airflow_max_mph;
extern float airflow_abs_max_mph;
extern float airflow_ema_alpha;
extern FS3000 airflow_sensor;

// steering related
extern int32_t steer_pulse_safe_us;
extern int32_t steer_pulse_out_us;  

// tunable
extern int32_t steer_pulse_right_min_us;
extern int32_t steer_pulse_right_us;
extern int32_t steer_pulse_stop_us;
extern int32_t steer_pulse_left_us; 
extern int32_t steer_pulse_left_max_us;
extern int32_t steer_safe_percent;

// brake pressure related
extern int32_t pressure_adc;

// tunable  
extern int32_t pressure_min_adc;
extern int32_t pressure_sensor_max_adc; 
extern int32_t pressure_max_adc;
extern float pressure_convert_psi_per_adc;
extern bool pressure_convert_invert;
extern float pressure_ema_alpha;
extern float pressure_margin_psi;
extern float pressure_min_psi;
extern float pressure_max_psi;
extern float pressure_hold_initial_psi;
extern float pressure_hold_increment_psi;
extern float pressure_panic_initial_psi;
extern float pressure_panic_increment_psi;

extern float pressure_psi;
extern float pressure_filt_psi;  
extern float pressure_target_psi;

// brake actuator motor related
extern float brake_pulse_out_us;  

// tunable
extern Timer brakeIntervalTimer;
extern int32_t brake_increment_interval_us;
extern int32_t brake_pulse_retract_min_us;
extern int32_t brake_pulse_retract_us;
extern int32_t brake_pulse_stop_us;
extern int32_t brake_pulse_extend_us;
extern int32_t brake_pulse_extend_max_us;
extern int32_t brake_pulse_margin_us;

// brake actuator position related
extern float brake_pos_in;
extern float brake_pos_filt_in;

// tunable
extern float brake_pos_convert_in_per_adc;  
extern bool brake_pos_convert_invert;
extern int32_t brake_pos_convert_polarity;
extern float brake_pos_ema_alpha;
extern float brake_pos_abs_min_retract_in;
extern float brake_pos_nom_lim_retract_in;
extern float brake_pos_zeropoint_in;
extern float brake_pos_park_in;
extern float brake_pos_nom_lim_extend_in;
extern float brake_pos_abs_max_extend_in;
extern float brake_pos_margin_in;

// throttle servo related
extern int32_t gas_pulse_out_us;
extern int32_t gas_pulse_govern_us;

// tunable
extern Timer gasServoTimer;
extern int32_t gas_governor_percent;  
extern int32_t gas_pulse_cw_min_us;
extern int32_t gas_pulse_redline_us;
extern int32_t gas_pulse_idle_us;
extern int32_t gas_pulse_ccw_max_us;
extern int32_t gas_pulse_park_slack_us;

// tachometer related
extern volatile int64_t tach_us;
extern int32_t tach_buf_us;
extern volatile int64_t tach_timer_start_us;
extern volatile int64_t tach_time_us;  
extern volatile int64_t tach_timer_read_us;
extern float tach_target_rpm;
extern float tach_rpm;
extern float tach_filt_rpm;
extern float tach_govern_rpm;

// tunable
extern float tach_convert_rpm_per_rpus;
extern bool tach_convert_invert;
extern int32_t tach_convert_polarity;
extern float tach_ema_alpha;
extern float tach_idle_rpm;
extern float tach_max_rpm;
extern float tach_redline_rpm;
extern float tach_margin_rpm;  
extern float tach_stop_thresh_rpm;
extern uint32_t tach_stop_timeout_us;
extern int64_t tach_delta_abs_min_us;

// carspeed/speedo related 
extern float speedo_target_mph;
extern float speedo_govern_mph;
extern float speedo_mph;
extern float speedo_filt_mph;

extern volatile int64_t speedo_us;
extern int32_t speedo_buf_us;
extern volatile int64_t speedo_timer_start_us;
extern volatile int64_t speedo_time_us;
extern volatile int64_t speedo_timer_read_us;

// tunable
extern float speedo_convert_mph_per_rpus;
extern bool speedo_convert_invert;
extern int32_t speedo_convert_polarity;  
extern float speedo_ema_alpha;
extern float speedo_idle_mph;
extern float speedo_redline_mph;
extern float speedo_max_mph;
extern float speedo_stop_thresh_mph;
extern uint32_t speedo_stop_timeout_us;
extern int64_t speedo_delta_abs_min_us;
            
// neopixel and heartbeat related
extern uint8_t neo_wheelcounter;
extern uint8_t neo_brightness_max;
extern uint32_t neo_timeout_us;
extern Timer neoTimer;
extern bool neo_heartbeat;
extern uint8_t neo_brightness;
enum neo_colors { N_RED, N_GRN, N_BLU };
extern uint8_t neo_heartcolor[3];
extern Timer heartbeatTimer;  
extern int32_t heartbeat_state;
extern int32_t heartbeat_level;
extern uint32_t heartbeat_ekg_us[4];
extern int32_t heartbeat_pulse;

// diag/monitoring variables
extern Timer loopTimer;
extern uint32_t loop_period_us;
extern float loop_freq_hz;
extern volatile int32_t loop_int_count;
extern int32_t loopno;
extern uint32_t looptimes_us[20];
extern bool loop_dirty[20]; 
extern int32_t loopindex;
extern bool booted;
extern bool diag_ign_error_enabled;

// pushbutton related  
enum sw_presses { NONE, SHORT, LONG }; // used by encoder sw and button algorithms
extern bool button_last;
extern bool button_it;
extern bool btn_press_timer_active;
extern bool btn_press_suppress_click;
extern bool btn_press_action;

// external signal related
extern bool ignition;
extern bool ignition_last;
extern bool ignition_output_enabled;  
extern bool ignition_sense;
extern float ignition_on_thresh_v;
extern bool syspower;
extern bool syspower_last;
extern bool basicmodesw;
extern bool cruise_sw;

// simulator related
extern bool simulating_last;
extern Timer simTimer;
extern int32_t sim_edit_delta;
extern int32_t sim_edit_delta_touch;
extern int32_t sim_edit_delta_encoder;

// tunable
extern bool simulating;
// enum sources { _PIN, _TOUCH, _POT };
enum pot_overload { none, pressure, tach, speedo, brkpos, airflow };  // , joy, brkpos, pressure, basicsw, cruisesw, syspower }
extern int32_t pot_overload;
extern bool sim_joy;
extern bool sim_tach;
extern bool sim_speedo;
extern bool sim_brkpos;
extern bool sim_basicsw;
extern bool sim_cruisesw;
extern bool sim_pressure; 
extern bool sim_syspower;
extern bool sim_starter;
extern bool sim_ignition;
extern bool sim_airflow;

extern SdFat sd;
#define approot "cantroller2020"
#define logfile "log.txt"
#define error(msg) sd.errorHalt(F(msg))  
extern SdFile root;
extern SdFile file;


extern QPID brakeQPID;

extern QPID gasQPID;

extern QPID cruiseQPID;

extern Servo gas_servo;
extern Servo brake_servo;
extern Servo steer_servo;
extern Adafruit_NeoPixel neostrip;

// Temperature sensor related  
extern long temp;
extern long temp_last;
// static int temp_secs; // static variables only for within file not global -bobby
// static byte temp_data[2];
// static int16_t temp_raw;


// Temperature related
extern float temp_min;  
extern float temp_max;
extern float temp_room;
enum temp_sensors { AMBIENT, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };
extern float temps[6];
extern int32_t temp_detected_device_ct;
extern int32_t temperature_precision;
extern OneWire onewire; 
extern int32_t temp_current_index;
enum temp_status {CONVERT, READ};
extern temp_status temp_state;
extern uint32_t temp_times_us[2];
extern uint32_t temp_timeout_us;
extern Timer tempTimer;

extern DeviceAddress temp_temp_addr;
extern DeviceAddress temp_addrs[6];  
extern DallasSensor tempsensebus;

// Interrupt handlers
extern void IRAM_ATTR tach_isr(void);
extern void IRAM_ATTR speedo_isr(void);

extern void IRAM_ATTR hotrc_horz_isr(void);
extern void IRAM_ATTR hotrc_vert_isr(void);  
extern void IRAM_ATTR hotrc_ch3_isr(void);
extern void IRAM_ATTR hotrc_ch4_isr(void);


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
// inline float mapfast (float x, float in_min, float out_min, float range_ratio) {
//     return out_min + (x - in_min) * range_ratio;
// }
// inline int32_t mapfast (int32_t x, int32_t in_min, int32_t out_min, float range_ratio) {
//     return (int32_t)(out_min + (x - in_min) * range_ratio);
// }
extern bool rounding;
extern float dround (float val, int32_t digits);

bool inline car_stopped (void) { return (speedo_filt_mph < speedo_stop_thresh_mph); }  // Moved logic that was here to the main loop
bool inline engine_stopped (void) { return (tach_filt_rpm < tach_stop_thresh_rpm); }  // Note due to weird float math stuff, can not just check if tach == 0.0

extern uint32_t colorwheel (uint8_t WheelPos);
extern void calc_ctrl_lims (void);

extern void calc_governor(void);
extern void ema_filt(float raw, float* filt, float alpha);  
extern void ema_filt(int32_t raw, float* filt, float alpha);
extern void ema_filt(int32_t raw, int32_t* filt, float alpha);
extern void sd_init();

extern bool adj_val(int32_t* variable, int32_t modify, int32_t low_limit, int32_t high_limit);
extern bool adj_val(float* variable, float modify, float low_limit, float high_limit);
extern void adj_bool(bool* val, int32_t delta);

extern void set_pin(int32_t pin, int32_t mode);
extern void write_pin(int32_t pin, int32_t val);  
extern int32_t read_pin(int32_t pin);

// void enable_pids(int32_t en_brake, int32_t en_gas, int32_t en_cruise);

extern bool read_battery_ignition(void);
extern void syspower_set(bool val);
extern long temp_peef(void);


extern void temp_soren(void);

extern void i2c_init(int32_t sda, int32_t scl);

#endif  // GLOBALS_H