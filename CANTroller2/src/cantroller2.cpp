// Carpet CANTroller II  Source Code  - For Arduino Due with Adafruit 2.8inch Captouch TFT shield.

// Libraries to include.  Note all these have example code when installed into arduino ide
//
#include <SPI.h>  // SPI serial bus needed to talk to the LCD and the SD card
#include <Wire.h>  // Contains I2C serial bus, needed to talk to touchscreen chip
#include <SdFat.h>  // SD card & FAT filesystem library
#include <Servo.h>  // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
#include <Adafruit_ILI9341.h>  // For interfacing with the TFT LCD controller chip
// #include <Adafruit_GFX.h>  // For drawing pictures & text on the screen

/*
# Here are the different runmodes documented
#
# ** Basic Mode **
# - Required: BasicMode switch On
# - Priority: 1 (Highest)
# The gas and brake don't do anything in Basic Mode. Just the steering works, so use the pedals.
# This mode is enabled with a toggle switch in the controller box.  When in Basic Mode, the only
# other valid mode is Shutdown Mode. Shutdown Mode may override Basic Mode.
# - Actions: Release and deactivate brake and gas actuators.  Steering PID keep active 
#
# ** Shutdown Mode **
# - Required: BasicMode switch Off & Ignition Off
# - Priority: 2
# This mode is active whenever the ignition is off.  In other words, whenever the
# little red pushbutton switch by the joystick is unclicked.  This happens before the
# ignition is pressed before driving, but it also may happen if the driver needs to
# panic and E-stop due to loss of control or any other reason.  The ignition will get cut
# independent of the controller, but we can help stop the car faster by applying the
# brakes. Once car is stopped, we release all actuators and then go idle.
# - Actions: 1. Release throttle. If car is moving AND BasicMode Off, apply brakes to stop car
# - Actions: 2: Release brakes and deactivate all actuators including steering
#
# ** Stall Mode **
# - Required: Engine stopped & BasicMode switch Off & Ignition On
# - Priority: 3
# This mode is active when the engine is not running.  If car is moving, then it presumably may
# coast to a stop.  The actuators are all enabled and work normally.  Starting the engine will 
# bring you into Hold Mode.  Shutdown Mode and Basic Mode both override Stall Mode. Note: This
# mode allows for driver to steer while being towed or pushed, or working on the car.
# - Actions: Enable all actuators
#
# ** Hold Mode **
# - Required: Engine running & JoyVert<=Center & BasicMode switch Off & Ignition On
# - Priority: 4
# This mode is entered from Stall Mode once engine is started, and also, whenever the car comes
# to a stop while driving around in Fly Mode.  This mode releases the throttle and will 
# continuously increase the brakes until the car is stopped, if it finds the car is moving. 
# Pushing up on the joystick from Hold mode releases the brakes & begins Fly Mode.
# Shutdown, Basic & Stall Modes override Hold Mode.
# # Actions: Close throttle, and Apply brake to stop car, continue to ensure it stays stopped.
#
# ** Fly Mode **
# - Required: (Car Moving OR JoyVert>Center) & In gear & Engine running & BasicMode Off & Ign On
# - Priority: 5
# This mode is for driving under manual control. In Fly Mode, vertical joystick positions
# result in a proportional level of gas or brake (AKA "Manual" control).  Fly Mode is
# only active when the car is moving - Once stopped or taken out of gear, we go back to Hold Mode.
# If the driver performs a special secret "cruise gesture" on the joystick, then go to Cruise Mode.
# Special cruise gesture might be: Pair of sudden full-throttle motions in rapid succession
# - Actions: Enable all actuators, Watch for gesture
#
# ** Cruise Mode **
# - Required: Car Moving & In gear & Engine running & BasicMode switch Off & Ignition On
# - Priority: 6 (Lowest)
# This mode is entered from Fly Mode by doing a special joystick gesture. In Cruise Mode,
# the brake is disabled, and the joystick vertical is different: If joyv at center, the
# throttle will actively maintain current car speed.  Up or down momentary joystick presses
# serve to adjust that target speed. A sharp, full-downward gesture will drop us back to 
# Fly Mode, promptly resulting in braking (if kept held down).
# - Actions: Release brake, Maintain car speed, Handle joyvert differently, Watch for gesture
*/

// Some human readable integers
//
#define SHUTDOWN 0  // for runmodes
#define BASIC 1  // for runmodes
#define STALL 2  // for runmodes
#define HOLD 3  // for runmodes
#define FLY 4  // for runmodes
#define CRUISE 5  // for runmodes
#define LOCK 0  // for dataset pages
#define JOY 1  // for dataset pages
#define CAR 2  // for dataset pages
#define PWMS 3  // for dataset pages
#define BPID 4  // for dataset pages
#define GPID 5  // for dataset pages
#define CPID 6  // for dataset pages
#define CW 1  // for encoder rotation
#define CCW -1  // for encoder rotation
#define OFF 0  // for UI tuning state
#define SELECT 1  // for UI tuning state
#define EDIT 2  // for UI tuning state

#define arraysize(x)  (sizeof(x) / sizeof((x)[0]))  // To determine the length of string arrays
// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))  // Copied from arduino constrain, just so I can easily refer to the the source code (made a function instead)

// LCD is 2.8in diagonal, 240x320 pixels
// LCD supports 18-bit color, but GFX library uses 16-bit color, organized (MSB) 5b-red, 6b-green, 5b-blue (LSB)
// Since the RGB don't line up with the nibble boundaries, it's tricky to quantify a color, here are some colors:
// Color picker websites: http://www.barth-dev.de/online/rgb565 , https://chrishewett.com/blog/true-rgb565-colour-picker/
#define BLK 0x0000
#define BLU 0x001F
#define RED 0xF800
#define GRN 0x07E0
#define CYN 0x07FF  // 00000 111 111 11111 
#define DCYN 0x0575  //
#define MGT 0xF81F
#define ORG 0xFCA0 
#define YEL 0xFFE0 
#define WHT 0xFFFF
#define GRY1 0x8410  // 10000 100 000 10000 = 84 10  dark grey
#define GRY2 0xC618  // 11000 110 000 11000 = C6 18  light grey
#define PNK 0xFC1F  // Pink is the best color
#define DPNK 0xBAD7  // We need all shades of pink
#define LPNK 0xFE1F  // Especially light pink, the champagne of pinks

// Defines for all the GPIO pins we're using
#define usd_cs_pin 4  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
#define tft_ledk_pin 5  // Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define tp_irq_pin 7  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
#define tft_dc_pin 9  // Output, Assert when sending data to display chip to indicate commands vs. screen data
#define tft_cs_pin 10  // Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
#define led_pin 13  // Output, This is the LED labeled "L" onboard the arduino due.  Active high.
#define encoder_sw_pin 18  // Int input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
#define encoder_b_pin 19  // Int input, The B pin (aka DT pin) of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
#define encoder_a_pin 21  // Int input, The A pin (aka CLK pin) of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
                          // The other kind of encoder: When A toggles, if B is equal to A, then turn is CCW, else CW.  (needs pullup)
#define pot_pwr_pin 24  // Output, Lets us supply the optional external potentiometer with 3.3V power
// #define sim_pulse_pin 26  // Output, For testing interrupts and stuff
#define steer_pwm_pin 29  // Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
#define speedo_pulse_pin 30  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
#define tach_pulse_pin 32  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
#define brake_pwm_pin 35  // Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
#define ignition_pin 43  // Input tells us if ignition signal is on or off, active high (no pullup)
#define cruise_sw_pin 41  // Input, momentary button low pulse >500ms in fly mode means start cruise mode. Any pulse in cruise mode goes to fly mode. Active low. (needs pullup)
#define basicmodesw_pin 47  // Input, asserted to tell us to run in basic mode.   (needs pullup)
// #define neutral_pin 49  // Input, asserted when car is in neutral, i.e. out of gear. Active low. (needs pullup)
#define gas_pwm_pin 67  // Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
#define pot_wipe_pin A6  // Analog input, tells us position of attached potentiometer (useful for debug, etc.)
#define battery_pin A7  // Analog input, mule battery voltage level, full scale is 15.638V
#define joy_horz_pin A8  // Analog input, tells us left-right position of joystick. Take complement of ADC value gives:  Low values for left, High values for right.
#define joy_vert_pin A9  // Analog input, tells us up-down position of joystick. Take complement of ADC value gives:  Low values for down, High values for up.
#define pressure_pin A10  // Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
#define brake_pos_pin A11  // Analog input, tells us linear position of brake actuator 
#define sim_halfass false  // Don't sim the joystick or encoder or tach
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

// Fixed parameters
//
#define disp_width_pix 320  // Horizontal resolution in pixels (held landscape)
#define disp_height_pix 240  // Vertical resolution in pixels (held landscape)
#define disp_lines 20  // Max lines of text displayable at line height = disp_line_height_pix
#define disp_fixed_lines 12  // Lines of static variables/values always displayed
#define disp_tuning_lines 8  // Lines of dynamic variables/values in dataset pages 
#define disp_line_height_pix 12  // Pixel height of each text line. Screen can fit 16x 15-pixel or 20x 12-pixel lines
#define disp_vshift_pix 2  // Unknown
#define touch_rows 4  // When touchscreen gridded as buttons, how many rows of buttons
#define touch_cols 5  // When touchscreen gridded as buttons, how many columns of buttons
#define touch_cell_width_pix 64  // When touchscreen gridded as buttons, width of each button
#define touch_cell_height_pix 60  // When touchscreen gridded as buttons, height of each button

#define adc_bits 12
#define adc_range_adc 4096    // = 2^12
#define adc_midscale_adc 2048
#define serial_debugging false
#define print_timestamps false  // Makes code write out timestamps throughout loop to serial port
// #define sim_dataset_page_count 7  // How many dataset pages

char telemetry[disp_fixed_lines][12] = {  
    "Flightmode:",
    " Air Speed:",
    " Engine #1:",
    "Hydraulics:",   
    "Stick Horz:",
    "Stick Vert:",
    " Steer PWM:",
    "Cruise Tgt:",
    "Gas Target:",
    "   Gas PWM:",
    "PresTarget:",
    " Brake PWM:"
};
char pagecard[7][5] = { "Run ", "Joy ", "Car ", "PWMs", "Bpid", "Gpid", "Cpid" };
char dataset_pages[arraysize(pagecard)][disp_tuning_lines][12] = {
    {   "   Battery:",  // LOCK
        " Brake Pos:",
        "  Pot Filt:",
        "       Pot:",
        " Encoder_A:",
        " Encoder_B:",
        " Enc State:",
        " EnCounter:" },
    {   "  Horz Raw:",  // JOY
        "  Vert Raw:",
        "  Horz Min:",
        "  Horz Max:",
        " Horz Dead:",
        "  Vert Min:",
        "  Vert Max:",
        " Vert Dead:" },
    {   "  Governor:",  // CAR
        "  Eng Idle:",
        "Eng Redlin:",
        "Speed Idle:",
        "Spd Redlin:",
        "   Gas PID:",
        " Gesturing:",
        "BrakePosZP:" },
    {   "  Steer Lt:",  // PWMS
        "Steer Stop:",
        "  Steer Rt:",
        " Brake Ext:",
        "Brake Stop:",
        "Brake Retr:",
        "  Gas Idle:",
        "Gas Redlin:" },
    {   "Pres Error:",  // BPID
        "    P Term:",
        "    I Term:",
        "    D Term:",
        "Pres Delta:",
        "    Kc (P):",
        "    Fi (I):",
        "    Td (D):" },
    {   " Eng Error:",  // GPID
        "    P Term:",
        "    I Term:",
        "    D Term:",
        " Eng Delta:",
        "    Kc (P):",
        "    Fi (I):",
        "    Td (D):" },
    {   " Spd Error:",  // CPID
        "    P Term:",
        "    I Term:",
        "    D Term:",
        " Spd Delta:",
        "    Kc (P):",
        "    Fi (I):",
        "    Td (D):" },
};
char units[disp_fixed_lines][5] = {"    ", "mmph", "rpm ", "adc ", "adc ", "adc ", "us  ", "mmph", "rpm ", "us  ", "adc ", "us  " };
char tuneunits[arraysize(pagecard)][disp_tuning_lines][5] = {
    { "mV  ", "adc ", "adc ", "adc ", "    ", "    ", "Hz  ", "    " },  // LOCK
    { "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc " },  // JOY
    { "%   ", "rpm ", "rpm ", "mmph", "mmph", "    ", "    ", "adc " },  // CAR
    { "us  ", "us  ", "us  ", "us  ", "us  ", "us  ", "us  ", "us  " },  // PWM
    { "adc ", "adc ", "adc ", "adc ", "adc ", "*1k ", "Hz  ", "ns  " },  // BPID
    { "mmph", "mmph", "mmph", "mmph", "mmph", "*1k ", "Hz  ", "ns  " },  // GPID
    { "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "*1k ", "Hz  ", "ns  " }   // CPID
};
char simgrid[touch_rows][touch_cols][6] = {
    { "     ", "     ", "pres+", "rpm+ ", "car+ " },
    { "     ", "  B  ", "pres-", "rpm- ", "car- " },
    { "     ", "  I  ", "(-)  ", "joy ^", "(+)  " },
    { "     ", "  C  ", "< joy", "joy v", "joy >" }
};    
char modecard[6][7] = { "Shutdn", "Basic", "Stall", "Hold", "Fly", "Cruise" };
int32_t colorcard[arraysize(modecard)] = { RED, MGT, ORG, YEL, GRN, CYN };

// Settable calibration values and control parameters
//
// When setting time values in us, consider each loop completes in around 65000us (or 200us without screen writes)
bool laboratory = true;  // Indicates we're not live on a real car. Allows launch of simulation interface by touching upper left corner
bool gas_pid = true;  // Are we using pid to get gas pulse output from desired engine rpm in fly mode, or just setting proportional
bool display_enabled = true;  // Should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool cruise_gesturing = false;  // Is cruise mode enabled by gesturing?  Otherwise by press of cruise button
float brake_pid_kc = 0.8;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
float brake_pid_fi_mhz = 0.0;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
float brake_pid_td_us = 0.0;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
float brake_pid_pos_kx = 0.0;  // Extra brake actuator position influence. This kicks in when the actuator is below the pressure zeropoint, to bring it up  (unitless range 0-1)
float cruise_pid_kc = 0.9;  // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
float cruise_pid_fi_mhz = 0.0;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
float cruise_pid_td_us = 0.0;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
float gas_pid_kc = 0.85;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
float gas_pid_fi_mhz = 0.0;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
float gas_pid_td_us = 0.0;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
float joy_ema_alpha = 0.05;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
float pot_ema_alpha = 0.2;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
float battery_ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
float pressure_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
float carspeed_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
float engine_rpm_ema_alpha = 0.005;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t joy_vert_min_adc = 9;  // ADC count of furthest joy position in down direction (ADC count 0-4095)
int32_t joy_vert_max_adc = 4095;  // (3728 at 3.3V VDD) ADC count of furthest joy position in up direction (ADC count 0-4095)
int32_t joy_horz_min_adc = 9;  // ADC count of furthest joy position in left direction (ADC count 0-4095)
int32_t joy_horz_max_adc = 4095;  // (3728 at 3.3V VDD) ADC count of furthest joy position in right direction (ADC count 0-4095)
int32_t joy_vert_deadband_adc = 220;  // Width of inert readings around center which we should treat as center (vert) (ADC count 0-4095)
int32_t joy_horz_deadband_adc = 220;  // Width of inert readings around center which we should treat as center (horz) (ADC count 0-4095)
int32_t pressure_min_adc = 455;  // Brake pressure when brakes are effectively off. Sensor min = 0.5V, scaled by 3.3/4.5V is 0.36V of 3.3V (ADC count 0-4095)
int32_t pressure_max_adc = 2048;  // Highest possible pressure achievable by the actuator (ADC count 0-4095)  658 adc no brakes     2080 adc as hard as chris can push  0.554V = no push, 1.89V = as hard as chris can push
int32_t pressure_margin_adc = 12;  // Margin of error when comparing brake pressure adc values (ADC count 0-4095)
int32_t pressure_spike_thresh_adc = 60;  // min pressure delta between two readings considered a spike to ignore (ADC count 0-4095)
int32_t pressure_lp_thresh_adc = 1200;   // max delta acceptable over three consecutive readings (ADC count 0-4095)
int32_t brake_hold_initial_adc = 1200;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
int32_t brake_hold_increment_adc = 10;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
int32_t brake_increment_interval_us = 500000;  // How often to apply increment during auto-stopping (in us)
int32_t brake_pos_retracted_adc = 0;  // Brake position value corresponding to retract limit of actuator (ADC count 0-4095)
int32_t brake_pos_zeropoint_adc = 1900;  // Brake position value corresponding to the point where fluid PSI hits zero (ADC count 0-4095)
int32_t brake_pos_park_adc = 2000;  // Best position to park the actuator out of the way so we can use the pedal (ADC count 0-4095)
int32_t brake_pos_extended_adc = 4095;  // Brake position value corresponding to ur max extension limit of actuator (ADC count 0-4095)
int32_t gesture_flytimeout_us = 250000;  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
int32_t sanity_timeout_us = 7000000;  // Gives certain loops an eventual way out if failures prevent normal completion (in us)
int32_t car_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the car is stopped (in us)
int32_t engine_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
int32_t engine_idle_rpm = 700;  // Min value for engine hz, corresponding to low idle (in rpm)
int32_t engine_redline_rpm = 4000;  // Max value for engine_rpm, pedal to the metal (in rpm)
int32_t engine_margin_rpm = 15;  // Margin of error for checking engine rpm (in rpm)
int32_t engine_spike_thresh_rpm = 500;  // min pressure delta between two readings considered a spike to ignore (in rpm)
int32_t engine_lp_thresh_rpm = 1000;   // max delta acceptable over three consecutive readings (in rpm)
int32_t carspeed_spike_thresh_mmph = 1500;  // min pressure delta between two readings considered a spike to ignore (in milli-mph)
int32_t carspeed_lp_thresh_mmph = 3000;   // max delta acceptable over three consecutive readings (in milli-mph)
int32_t carspeed_idle_mmph = 3000;  // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)
int32_t carspeed_redline_mmph = 20000;  // What is our steady state speed at redline? Pulley rotation frequency (in milli-mph)
int32_t cruise_max_change_mmph = 500;  // What's the max car cruise speed change from a single speed adjustment? (in milli-mph)
int32_t cruise_adj_period_us = 250000;  // how often cruise mode applies speed adjustments based on joystick vert position (in us)
int32_t cruise_sw_timeout_us = 500000;  // how long do you have to hold down the cruise button to start cruise mode (in us)
int32_t gas_governor_percent = 85;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
// int32_t gas_pulse_idle_us = 1761;  // Gas pulsewidth corresponding to fully closed throttle (in us)
// int32_t gas_pulse_redline_us = 1544;  // Gas pulsewidth corresponding to full open throttle (in us)
int32_t gas_pulse_redline_us = 1700;  // Gas pulsewidth corresponding to full open throttle (in us)
int32_t gas_pulse_idle_us = 2300;  // Gas pulsewidth corresponding to fully closed throttle (in us)
int32_t gas_pulse_park_us = gas_pulse_idle_us + 30;  // Gas pulsewidth where to park the servo out of the way so we can drive manually (in us)
int32_t steer_pulse_right_us = 840;  // Steering pulsewidth corresponding to full-speed right steering (in us)
int32_t steer_pulse_stop_us = 1500;  // Steering pulsewidth corresponding to zero steering motor movement (in us)
int32_t steer_pulse_left_us = 2200;  // Steering pulsewidth corresponding to full-speed left steering (in us)
int32_t default_pulse_margin_us = 30;  // Default margin of error for comparisons of pulsewidths (in us)
int32_t brake_pulse_retract_us = 650;  // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us)
int32_t brake_pulse_stop_us = 1500;  // Brake pulsewidth corresponding to center point where motor movement stops (in us)
int32_t brake_pulse_extend_us = 2350;  // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us)
int32_t default_margin_adc = 20;  // Default margin of error for comparisons of adc values (ADC count 0-4095)
int32_t sim_modify_period_us = 150000;  // How fast to change variable values when holding modify button down in simulator (in us)
int32_t battery_max_mv = 16000;  // The max vehicle voltage we can sense. Design resistor divider to match. Must exceed max V possible.
int32_t pid_period_us = 20000;    // time period between output updates. Reciprocal of pid frequency (in us)
int32_t motor_park_timeout_us = 3000000;  // If we can't park the motors faster than this, then give up.

// Non-settable variables
//
int32_t sim_dataset_page = LOCK;  // Which of the six 8-value dataset pages is currently displayed, and available to edit
int32_t sim_dataset_page_last = sim_dataset_page;
int32_t sim_selected_value = 0;  // In the real time tuning UI, which of the editable values (0-7) is selected. -1 for none 
int32_t sim_selected_value_last = 0;
float gas_pid_p_term_rpm = 0.0;
float gas_pid_i_term_rpm = 0.0;
float gas_pid_d_term_rpm = 0.0;
float gas_pid_derivative_rpmperus = 0.0;
float cruise_pid_p_term_mmph = 0.0;
float cruise_pid_i_term_mmph = 0.0;
float cruise_pid_d_term_mmph = 0.0;
float cruise_pid_derivative_mmphperus = 0.0;
float brake_pid_p_term_adc = 0.0;
float brake_pid_i_term_adc = 0.0;
float brake_pid_d_term_adc = 0.0;
float brake_pid_pos_term_adc = 0.0;
float brake_pid_derivative_adcperus = 0.0;
int32_t engine_rpm = 0;  // Current engine speed, raw as sensed (in rpm)
int32_t engine_filt_rpm = 0;  // Current engine speed, filtered (in rpm)
int32_t engine_last_rpm = 0;  // Engine speed from previous loop (in rpm)
int32_t engine_old_rpm = 0; // Engine speed from two loops back (in rpm)
int32_t carspeed_mmph = 0;  // Current car speed, raw as sensed (in mmph)
int32_t carspeed_filt_mmph = 0;  // Current car speed, filtered (in mmph)
int32_t carspeed_last_mmph = 0;  // Car speed from previous loop (in mmph)
int32_t carspeed_old_mmph = 0;  // Car speed from two loops back (in mmph)
int32_t battery_mv = 10000;
int32_t battery_filt_mv = 10000;
int32_t pot_filt_adc = adc_midscale_adc;
int32_t joy_vert_adc = adc_midscale_adc;
int32_t joy_horz_adc = adc_midscale_adc;
int32_t joy_vert_filt_adc = adc_midscale_adc;
int32_t joy_horz_filt_adc = adc_midscale_adc;
int32_t steer_pulse_out_us = steer_pulse_stop_us;  // pid loop output to send to the actuator (steering)
int32_t brake_pulse_out_us = brake_pulse_stop_us;  // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
int32_t brake_timer_us = 0;  // Timer used to control braking increments
int32_t brake_pid_error_adc = 0;
int32_t brake_pid_error_last_adc = 0;
int32_t brake_pid_integral_adcus = 0;
int32_t brake_pid_pos_error_adc = 0;
int32_t pressure_adc = 0;
int32_t pressure_delta_adc = 0;
int32_t pressure_target_adc = 0;  // Stores new setpoint to give to the pid loop (brake)
int32_t pressure_filt_adc = 0;  // Stores new setpoint to give to the pid loop (brake)
int32_t pressure_error_polarity = 0;  // Which direction the motor is going
int32_t pressure_error_polarity_last = 0;  // Allows detection of change of direction, to implement hysteresis
int32_t pressure_hysteresis_adc = 100 ;  // 100 = 2.5% of ADC range. Keeps motor from constantly moving once target is hit
bool pressure_hysteresis_hold = false;
int32_t pressure_last_adc = adc_midscale_adc;  // Some pressure reading history for noise handling (-1)
int32_t pressure_old_adc  = adc_midscale_adc;  // Some pressure reading history for noise handling (-2)
int32_t gas_target_rpm = 0;  // Stores new setpoint to give to the pid loop (gas)
int32_t gas_pid_error_rpm = 0;
int32_t gas_delta_rpm = 0;
int32_t gas_pid_error_last_rpm = 0;
int32_t gas_pid_integral_rpmus = 0;
int32_t gas_pulse_delta_us;
int32_t gas_pulse_out_us = gas_pulse_idle_us;  // pid loop output to send to the actuator (gas)
int32_t sanity_timer_us;  // Allows code to fail in a sensible way in certain circumstances
int32_t gesture_timer_us = 0;  // Used to keep track of time for gesturing
int32_t cruise_engine_delta_rpm = 0; //
int32_t cruise_pid_error_mmph = 0;
int32_t cruise_pid_error_last_mmph = 0;
int32_t cruise_pid_integral_mmphus = 0;
bool cruise_adjusting = false;
int32_t carspeed_delta_mmph = 0;  // 
int32_t carspeed_target_mmph = 0.0;  // Stores new setpoint to give to the pid loop (cruise) in milli-mph
int32_t gesture_progress = 0;  // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
int32_t runmode = SHUTDOWN;  // Variable to store what mode we're in
int32_t oldmode = SHUTDOWN;  // So we can tell when the mode has just changed
// bool neutral = true;
bool ignition = false;
bool disp_redraw_all = true;
bool basicmodesw = false;
bool cruise_sw = false;
bool cruise_sw_held = false;
bool shutdown_complete = true;  // Shutdown mode has completed its work and can stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;  // Indicates we should release the brake & gas so the pedals can be used manually without interference
int32_t motor_park_timer_us = micros();
bool sim_out = LOW;
bool simulate = false;
bool simulate_last = false; 
char disp_draw_buffer[8];  // Used to convert integers to ascii for purposes of displaying on screen
char disp_values[disp_lines][8];
int32_t disp_age_quanta[disp_lines];
int32_t disp_ages_us[disp_lines];
#define disp_nobools 4
bool disp_bool_values[disp_nobools];
char disp_bool_buffer;
int32_t old_tach_time_us;
int32_t old_speedo_time_us;
int32_t cruise_sw_timer_us = 0;
int32_t now_us = micros();
int32_t pid_timer_us = micros();
int32_t sim_timer_us = micros();
int32_t sim_modify_polarity = 0;
int32_t sim_modify_polarity_touch = 0;
int32_t sim_modify_polarity_encoder = 0;
int32_t sim_modify_timer_us = micros();
int32_t sim_tuning_ctrl = OFF;
int32_t sim_tuning_ctrl_last = OFF;
int32_t touch_timer_us = micros();  // Timer for regular touchscreen sampling
int32_t touch_period_us = 35000;  // Time between consecutive touchscreen readings. Taps won't cause responses faster than this
int32_t touch_start_marker_us;  // Start time of current touch event
bool touch_now_touched = false;  // Is a touch event in progress
int32_t touch_accel_exponent = 0;  // Will edit values by +/- 2^touch_accel_exponent per touch_period interval
int32_t touch_accel = 1 << touch_accel_exponent;  // Touch acceleration level, which increases the longer you hold. Each edit update chages value by this
int32_t touch_accel_exponent_max = 8;  // Never edit values faster than this. 2^8 = 256 change in value per update
int32_t touch_accel_shift_period_us = 600000;  // Touch hold time per left shift (doubling) of touch_accel
int32_t loop_timer_us = micros();  // used to determine loop_period_us
int32_t loop_period_us = 1000000;  // how long the previous main loop took to run (in us)
int32_t loop_freq_hz = 1;  // run loop real time frequency (in Hz)
int32_t loopno = 1;    
int32_t loopzero = 0;  

// int32_t pressure_min_psi = 0;  // Brake pressure when brakes are effectively off (psi 0-1000)
// int32_t pressure_max_psi = 500;  // Highest possible pressure achievable by the actuator (psi 0-1000)

// Testing magnet sensors
int32_t tach_magnet_count = 0;  // remove this after tach sensor bench testing (all references)

// Volatile variables  - for variables set inside ISRs
//
volatile int32_t tach_timer_us = micros();
volatile int32_t tach_last_us = tach_timer_us;
volatile int32_t speedo_timer_us = micros();
volatile int32_t tach_delta_us = 0;
volatile int32_t speedo_last_us = speedo_timer_us;
volatile int32_t speedo_delta_us = 0;
volatile int32_t encoder_delta = 0;  //  Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 
// volatile int32_t encoder_debounce_timer_us = micros();  // Used to prevent bouncing and noise generated interrupts
// volatile bool encoder_sw_isr_flag = false;  // flag for fresh interrupt (push down or up)
volatile bool led_state = LOW;
// volatile int32_t int_count = 0;
// volatile int32_t* pwm[] = { &OCR5A, &OCR5B, &OCR5C }; // &OCR1A, &OCR1B, &OCR1C, &OCR3A, &OCR3B, &OCR3C, &OCR4A, &OCR4B, &OCR4C,   // Store the addresses of the PWM timer compare (duty-cycle) registers:

// int32_t encoder_invalid_timeout_us = 1000;  // Used to prevent bouncing and noise generated interrupts (in us)
int32_t encoder_spinspeed_timer_us = micros();  // Used to figure out how fast we're spinning the knob
int32_t encoder_press_timer_us = micros();  // Used to time long button presses
int32_t encoder_press_time_us = 0;  // Holds the duration of a long press
int32_t encoder_spinrate_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_spinrate_min_us = 2500;  // Will reject spins faster than this as an attempt to debounce behavior
int32_t encoder_spinrate_last_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_spinrate_old_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_edits_per_det = 1;  // How many edits per detent. How much change happens per rotation detent
int32_t encoder_long_press_us = 800000;  // How long pushing switch counts as a long press
bool encoder_sw = false;  // Remember whether switch is being pressed
bool encoder_sw_action = true;  // Flag for encoder handler to know an encoder switch action needs to be handled
bool encoder_timer_on = false;  // Flag to prevent re-handling long presses if the sw is just kept down
bool encoder_suppress_click = false;  // Flag to prevent a short click on switch release after successful long press
bool encoder_a_bounce_danger = false;
bool encoder_b_bounce_danger = false;
bool encoder_b_raw = digitalRead(encoder_b_pin);  // To store value of encoder pin value
bool encoder_a_raw = digitalRead(encoder_a_pin);
int32_t encoder_state = 0;
int32_t encoder_counter = 0;

// Instantiate objects 
Adafruit_FT6206 touchpanel = Adafruit_FT6206(); // Touch panel
Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_cs_pin, tft_dc_pin);  // LCD screen

SdFat sd;  // SD card filesystem
#define approot "cantroller2020"
#define logfile "log.txt"
#define error(msg) sd.errorHalt(F(msg))  // Error messages stored in flash.
SdFile root;  // Directory file.
SdFile file;  // Use for file creation in folders.

// char cinBuf[40];  // Buffer for Serial input.
// ArduinoOutStream cout(Serial);  // Create a Serial output stream.
// ArduinoInStream cin(Serial, cinBuf, sizeof(cinBuf));  // Create a serial input stream.

// Instantiate PID loops
//
// Steering:  Motor direction and velocity are controlled with PWM, proportional to joystick horizontal direction and magnitude
//   Setpoint Value: Proportional to Joystick Horz ADC value.  0V = Full Left, 2.5V = Stop, 5V = Full Right
//   Measured Value: We have no feedback, other than the joystick current horizontal position
//   Actuator Output Value: PWM square wave sent to Jaguar, w/ high pulse width of: ~2.2ms = Full Left, 1.5ms = Stop, ~800us = Full Right
//   Limits: Reed switch limit signals for left and right may be handled by us, or by the jaguar controller
//   Setpoint scaling: Kp/Ki/Kd values should decrease appropriately as a function of vehicle speed (safety) 
//
//   Notes: The steering has no feedback sensing, other than two digital limit switches at the ends of travel.  
//   So just consider the error to be the difference between the joystick position and the last output value.
//
// Brakes:  Motor direction & velocity are controlled with PWM until brake pressure matches pressure setpoint
//   Setpoint Value: * Default: Pressure setpoint proportional to Joystick Vert distance from center when below center.
//       * In Hold Mode: Brake adjusts automatically to keep car stopped, as long as joystick below center
//       * In Cruise Mode: Brake is kept released 
//   Measured Value: Analog voltage from brake fluid pressure sensor. 0-3.3V proportional to 0-1000psi
//   Actuator Output Value: PWM signal to Brake Jaguar unit.
//       0% duty = Full speed extend (less brake), 50% = Stop, 100% = Full speed Retract (more brake)
//   Position: Analog 0-3.3V proportional to the travel length of the actuator (not used as feedback)
//
// Gas:  Servo angle is adjusted with PWM until engine rpm matches rpm target setpoint
//   Setpoint Value: * Default: RPM Setpoint proportional to Joystick Vert distance from center when above center.
//       * In Cruise Mode: Upward or downward joy vert motions modify vehicle speed setpoint
//                          Gas pid setppoints are output from cruise pid
//   Measured Value: * Default: Engine speed determined from tach pulses
//   Actuator Output Value: PWM signal to throttle servo
//       0% duty = Fully close throttle.  This will idle.  100% duty = Fully open throttle.
//
// Cruise:
//   Setpoint Value: * Default: Set to the current vehicle speed when mode is entered.
//       * In Cruise Mode: Upward or downward joy vert motions suspend loop and accelerate or decelerate,
//                         upon return to center loop resumes with new speed target set to vehicle speed when released.
//   Measured Value: * Vehicle speed determined from pulley sensor pulses
//   Actuator Output Value: Cruise PID output values become setpoint values for the Gas PID above
//       0% duty = Car stopped.  100% duty = Car max speed.
//
// One way to tune a PID loop:
// 1) Set Kp = 0, Ki = 0, and Kd = 0
// 2) Increase Kp until output starts to oscillate
// 3) Increase Ki until oscillation stops
// 4) If improved response time is needed, increase Kd slightly and go back to step 2
//
// Ziegler-Nichols method:
// 1) Set Kp = 0, Ki = 0, and Kd = 0
// 2) Increase Kp until output starts to oscillate.
// 3) Record Kc = critical value of Kp, and Pc = period of oscillations
// 4) Set Kp=0.6*Kc and Ki=1.2*Kc/Pc and Kd=Kc*Pc/13.33  (Or for P only:  Kp=Kc/2)  (Or for PI:  Kp=0.45*Kc and Ki=0.54*Kc/Pc)

// PID brake_pid(&pressure, &brake_pwm, &brake_target, brake_pid_kp, brake_pid_ki_mhz, brake_pid_kd_us, DIRECT);
// FastPID brake_pid(brake_pid_kp, brake_pid_ki_mhz, brake_pid_kd_us, pid_freq_hz, 8, false);
// FastPID cruise_pid(cruise_pid_kp, cruise_pid_ki_mhz, cruise_pid_kd_us, pid_freq_hz, 8, false);

// Servo library lets us set pwm outputs given an on-time pulse width in us
static Servo steer_servo;
static Servo brake_servo;
// static Servo gas_servo;

// General use functions
// long map(long x, long in_min, long in_max, long out_min, long out_max) {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }
// template<class T>
// const T& constrain(const T& x, const T& xmin, const T& xmax) {
//     if(x < xmin) {
//         return xmin;
//     }
//     else if(xmax < x) {
//         return xmax;
//     }
//     else
//         return x;
// }

// Interrupt service routines
//
// void encoder_a_isr(void) {  // When A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
//     if (!encoder_bounce_danger) {
//         encoder_delta = digitalRead(encoder_b_pin) ? -1 : 1;  // CCW turn decreases things
//         encoder_counter += encoder_delta;
//         encoder_bounce_danger = true;
//     }
// }
// void encoder_b_isr(void) {  // When A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
//     encoder_bounce_danger = false;
// }
void encoder_a_isr(void) {  // When A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
    if (!encoder_a_bounce_danger) {
        if (digitalRead(encoder_a_pin)) {
            encoder_delta = digitalRead(encoder_b_pin) ? -1 : 1;  // CCW turn decreases things
            encoder_counter += encoder_delta;
        }
        encoder_a_bounce_danger = true;
        encoder_b_bounce_danger = false;
    }
}
void encoder_b_isr(void) {  // On B rising or falling edge, When B A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
    if (!encoder_b_bounce_danger) {
        encoder_a_bounce_danger = false;
        encoder_b_bounce_danger = true;
    }
}
// void encoder_a_isr(void) {  // When A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
//     if (!encoder_a_bounce_danger && digitalRead(encoder_a_pin)) {
//         encoder_delta = digitalRead(encoder_b_pin) ? -1 : 1;  // CCW turn decreases things
//         encoder_counter += encoder_delta;
//     }
//     encoder_a_bounce_danger = true;
//     encoder_b_bounce_danger = false;
// }
// void encoder_b_isr(void) {  // On B rising or falling edge, When B A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
//     encoder_a_bounce_danger = false;
//     encoder_b_bounce_danger = true;
// }

    // if (digitalRead(encoder_b_pin) encoder_delta = -1;
    // else encoder_delta = 1;  // CW turn increases things

    // if (digitalRead(encoder_b_pin)) encoder_delta--;
    // else encoder_delta++;
 
    // if (encoder_b_raw) encoder_delta--;
    // led_state = !led_state;
    // digitalWrite(led_pin, led_state);

// void encoder_b_isr(void) {  // When A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
//     encoder_b_raw = digitalRead(encoder_b_pin);
// }
// void encoder_sw_isr(void) {  
//     encoder_button = !digitalRead(encoder_sw_pin);
//     encoder_sw_isr_flag = true;
// }

// The tach and speed use a hall sensor being triggered by a passing magnet once per pulley turn. These ISRs call micros()
// on every pulse to know the time since the previous pulse. I tested this on the bench up to about 750 mmph which is as 
// fast as I can move the magnet with my hand, and it works. It would be cleaner to just increment a counter here in the ISR
// then call micros() in the main loop and compare with a timer to calculate mmph.

void tach_isr(void) {  // The tach and speedo isrs compare value returned from the micros() function with the value from the last interrupt to determine period, to get frequency of the vehicle pulley rotations.
    
    // Method 1: This works but this is a sloppy excuse for an ISR. we shouldn't be calling micros() inside an ISR
    tach_timer_us = micros();  // This might screw up things.  Anders would remember
    tach_delta_us = abs(tach_timer_us-tach_last_us);
    tach_last_us = tach_timer_us;
    //
    tach_magnet_count++; // this is temporary. remove after sensor tests (remove all copies)
    
    // // Method 2: Makes this ISR squeaky clean like the asshole of a fetus
    // // This aint working for some reason ... I don't have time to debug it now. Ugh!
    // tach_magnet_count++;  // add one to the pulses sensed since the last sensor value update (in main loop) 
}
void speedo_isr(void) {  //  A better approach would be to read, reset, and restart a hardware interval timer in the isr.  Better still would be to regularly read a hardware binary counter clocked by the pin - no isr.
    speedo_timer_us = micros();  // This might screw up things.  Anders would remember
    speedo_delta_us = abs(speedo_timer_us-speedo_last_us);
    speedo_last_us = speedo_timer_us;
}

// ema_filt : pass in a fresh raw value, the previously filtered value, and alpha factor, returns new filtered value
int32_t ema_filt(int32_t raw_value, int32_t filt_value, float alpha) {
    return (int32_t)((alpha*(float)raw_value) + ((1-alpha)*(float)filt_value));
}
    
// Functions to write to the screen efficiently
//
// draw_text displays 20 rows of text strings with variable names. and also a column of text indicating units, all in grey.
void draw_text(bool redraw_tuning_corner) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
    tft.setTextColor(GRY2);
    tft.setTextSize(1);    
    if (redraw_tuning_corner)  tft.fillRect(0,146,138,94,BLK);
    else {
        tft.fillScreen(BLK);
        for (int32_t lineno=0; lineno < (int32_t)arraysize(telemetry); lineno++)  {
            tft.setCursor(2, lineno*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
            tft.println(telemetry[lineno]);
            tft.setCursor(114, lineno*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
            tft.println(units[lineno]);
        }
    }
    for (int32_t lineno=0; lineno < (int32_t)arraysize(dataset_pages[sim_dataset_page]); lineno++)  {
        tft.setCursor(2, (lineno+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
        tft.println(dataset_pages[sim_dataset_page][lineno]);
        tft.setCursor(114, (lineno+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
        tft.println(tuneunits[sim_dataset_page][lineno]);
    }   
}
// draw_value  normally draws a given value on a given line (0-19) to the screen if it has changed since last draw.
void draw_value(int32_t lineno, int32_t value, int32_t modeflag) {
    int32_t age_us = (int32_t)((float)(abs(now_us - disp_ages_us[lineno]))/2500000); // Divide by us per color gradient quantum
    memset(disp_draw_buffer,0,strlen(disp_draw_buffer));
    if (modeflag == 0) itoa(value, disp_draw_buffer, 10);  // Modeflag 0 is for writing numeric values for variables in the active data column at a given line
    else if (modeflag == 1)  strcpy(disp_draw_buffer, modecard[runmode]); // Modeflag 1 is used for writing the runmode. Each mode has a custom color which doesn't get stale
    if (modeflag == 3) {  // Modeflag 3 is for highlighting a variable name when its value may be changed
        if ( (sim_tuning_ctrl == SELECT && sim_selected_value != sim_selected_value_last) || // IF the selected tuning variable has changed, OR
             (sim_tuning_ctrl != SELECT && sim_tuning_ctrl_last == SELECT) ) {  // We just stopped selecting values altogether
            tft.setCursor(2, (sim_selected_value_last+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
            tft.setTextColor(GRY2);
            tft.print(dataset_pages[sim_dataset_page][sim_selected_value_last]);  // Grey out the old highlighted variable
        }
        if (sim_tuning_ctrl == EDIT && sim_tuning_ctrl_last != EDIT) {  // If we just started editing the variable
            tft.setCursor(2, (sim_selected_value+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
            tft.setTextColor(GRN);
            tft.print(dataset_pages[sim_dataset_page][sim_selected_value]);  // Highlight selected value in blue
        }
        else if ( sim_tuning_ctrl == SELECT && (sim_selected_value != sim_selected_value_last || sim_tuning_ctrl_last != SELECT) ) { // If just entered selecting mode or selected value changed
            tft.setCursor(2, (sim_selected_value+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
            tft.setTextColor(YEL);
            tft.print(dataset_pages[sim_dataset_page][sim_selected_value]);  // Highlight selected value in white
        }
    }
    // if (modeflag == 3 && sim_selected_value != sim_selected_value_last) {  // Modeflag 3 is for highlighting a variable name when its value may be changed
    //     tft.setCursor(2, (sim_selected_value_last+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
    //     tft.setTextColor(GRY2);  
    //     // if (sim_dataset_page != sim_dataset_page_last) tft.print(dataset_pages[sim_dataset_page_last][sim_selected_value_last]);
    //     // else
    //     tft.print(dataset_pages[sim_dataset_page][sim_selected_value_last]);
    //     // if (sim_dataset_page != LOCK && sim_selected_value >= 0) {
    //         tft.setCursor(2, (sim_selected_value+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
    //         if (sim_tuning_ctrl == EDIT) tft.setTextColor(BLU);
    //         else tft.setTextColor(WHT);
    //         tft.print(dataset_pages[sim_dataset_page][sim_selected_value]);
    //     // }
    //     // sim_selected_value_last = sim_selected_value;
    // }
    else if (modeflag == 2 && sim_dataset_page != sim_dataset_page_last) {  // Modeflag 2 is used for displaying which set of tuning variables is being displayed. Text next to the runmode
        tft.setCursor(112, disp_vshift_pix);  // +disp_line_height_pix/2
        tft.setTextColor(BLK);  // Rewriting old value in black over visible value is efficient way to erase
        tft.print(pagecard[sim_dataset_page_last]);
        tft.setCursor(112, disp_vshift_pix);  // +disp_line_height_pix/2
        tft.setTextColor(CYN);  
        tft.print(pagecard[sim_dataset_page]);
        // sim_dataset_page_last = sim_dataset_page;
    }
    if (modeflag < 2 && (strcmp(disp_values[lineno], disp_draw_buffer) || disp_redraw_all))  {  // If value differs, Erase old value and write new
        tft.setCursor(70, (lineno)*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
        tft.setTextColor(BLK);  // Rewriting old value in black over visible value is efficient way to erase
        tft.print(disp_values[lineno]);
        tft.setCursor(70, (lineno)*disp_line_height_pix+disp_vshift_pix);  // +disp_line_height_pix/2
        if (modeflag == 1)  tft.setTextColor(colorcard[runmode]);
        else tft.setTextColor(GRN);
        tft.print(disp_draw_buffer);
        strcpy(disp_values[lineno], disp_draw_buffer);
        disp_ages_us[lineno] = now_us;
        disp_age_quanta[lineno] = 0;
    }
    else if (modeflag == 0 && age_us > disp_age_quanta[lineno] && age_us < 11)  {  // As readings age, redraw in new color
        int32_t color;
        if (age_us < 8) color = 0x1FE0+age_us*0x2000;  // Base of green with red added as you age
        else color = 0xFFE0;  // Yellow is achieved
        if (age_us > 8) color -= (age_us-8)*0x100;  // Then lose green as you age further
        tft.setTextColor(color);
        tft.setCursor(70, (lineno)*disp_line_height_pix+disp_vshift_pix); // +disp_line_height_pix/2
        tft.print(disp_values[lineno]);
        disp_age_quanta[lineno] = age_us;   
    } // Else don't draw anything, because we already did.  Logic is 100s of times cheaper than screen drawing.
}
void draw_bool(bool value, int32_t row) {
    if ((disp_bool_values[row] != value) || disp_redraw_all) {  // If value differs, Erase old value and write new
        if (value) {
            tft.setCursor(disp_width_pix/touch_cols+touch_cell_width_pix/2+11, row*disp_height_pix/touch_rows+touch_cell_height_pix/2-disp_line_height_pix);
            tft.setTextColor(BLK);
            tft.println("0");
            tft.setCursor(disp_width_pix/touch_cols+touch_cell_width_pix/2+11, row*disp_height_pix/touch_rows+touch_cell_height_pix/2-disp_line_height_pix);
            tft.setTextColor(CYN);
            tft.println("1");
            disp_bool_values[row] = 1;
        }
        else {
            tft.setCursor(disp_width_pix/touch_cols+touch_cell_width_pix/2+11, row*disp_height_pix/touch_rows+touch_cell_height_pix/2-disp_line_height_pix);
            tft.setTextColor(BLK);
            tft.println("1");
            tft.setCursor(disp_width_pix/touch_cols+touch_cell_width_pix/2+11, row*disp_height_pix/touch_rows+touch_cell_height_pix/2-disp_line_height_pix);
            tft.setTextColor(DCYN);
            tft.println("0");
            disp_bool_values[row] = 0;
        }
    }
}

void draw_touchgrid(bool update) {  // if update is true, will only redraw the at-risk elements of the grid
    // if (!update) {
        for (int32_t row = 0; row < touch_rows; row++) tft.drawFastHLine(0, row*disp_height_pix/touch_rows, disp_width_pix, DPNK);  // x, y, length, color
        for (int32_t col = 0; col < touch_cols; col++) tft.drawFastVLine(col*disp_width_pix/touch_cols, 0, disp_height_pix, DPNK);  // faster than tft.drawLine(0, 80, 320, 80, GRY1);
        tft.drawFastHLine(0, disp_height_pix-1, disp_width_pix, DPNK);  // Draw line along very bottom. (x, y, length, color)
        tft.drawFastVLine(disp_width_pix-1, 0, disp_height_pix, DPNK);  // Draw line along far right edge. Faster than tft.drawLine(0, 80, 320, 80, GRY1);
    // }
    tft.setTextColor(LPNK);
    tft.setTextSize(1);
    for (int32_t row = 0; row < touch_rows; row++) {
        for (int32_t col = 0; col < ((touch_cols-1) * !update) + 1; col++) {
            tft.setCursor(col*disp_width_pix/touch_cols+touch_cell_width_pix/2-7, row*disp_height_pix/touch_rows+touch_cell_height_pix/2-disp_line_height_pix);
            tft.println(simgrid[row][col]);
        }
    }
    // if (update) {
    //     for (int32_t row = 0; row < touch_rows; row++) {
    //         tft.setCursor(disp_width_pix/touch_cols+touch_cell_width_pix/2-7, row*disp_height_pix/touch_rows+touch_cell_height_pix/2-disp_line_height_pix);
    //         tft.println(simgrid[row][1]);
    //     }
    // }
    // else {
    //     for (int32_t row = 0; row < touch_rows; row++) {
    //         for (int32_t col = 0; col < touch_cols; col++) {
    //             tft.setCursor(col*disp_width_pix/touch_cols+touch_cell_width_pix/2-7, row*disp_height_pix/touch_rows+touch_cell_height_pix/2-disp_line_height_pix);
    //             tft.println(simgrid[row][col]);
    //         }
    //     }
    // }
}

void sd_init() {
    if (!sd.begin(usd_cs_pin, SD_SCK_MHZ(50))) {  // Initialize at highest supported speed that is not over 50 mhz. Go lower if errors.
        sd.initErrorHalt();
    }
    if (!root.open("/")) {
        error("open root failed");
    }
    if (!sd.exists(approot)) { 
        if (sd.mkdir(approot)) {
            Serial.println(F("Created approot directory\n"));  // cout << F("Created approot directory\n");
        }
        else {
            error("Create approot failed");
        }
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
    Serial.println(F("Filesystem init finished\n"));  // cout << F("Filesystem init finished\n");

    // for (byte a = 10; a >= 1; a--)
    // {
    //     char fileName[12];
    //     sprintf(fileName, "%d.txt", a);
    //     file = sd.open(fileName, FILE_WRITE); //create file
    // }
}

int32_t sim_edit(int32_t variable, int32_t modify) {
        if (modify < 0 && (int32_t)variable < -modify) return 0;
        else return variable + modify; 
        // return (variable + modify < 0) ? 0 : variable + modify; 
    }

void setup() {
    pinMode(led_pin, OUTPUT);
    pinMode(encoder_a_pin, INPUT_PULLUP);
    pinMode(encoder_b_pin, INPUT_PULLUP);
    // pinMode(sim_pulse_pin, OUTPUT);
    pinMode(brake_pwm_pin, OUTPUT);
    pinMode(steer_pwm_pin, OUTPUT);
    pinMode(tft_dc_pin, OUTPUT);
    pinMode(encoder_sw_pin, INPUT_PULLUP);
    pinMode(gas_pwm_pin, OUTPUT);
    pinMode(ignition_pin, INPUT);
    // pinMode(neutral_pin, INPUT_PULLUP);
    pinMode(basicmodesw_pin, INPUT_PULLUP);
    pinMode(cruise_sw_pin, INPUT_PULLUP);
    pinMode(tach_pulse_pin, INPUT_PULLUP);
    pinMode(speedo_pulse_pin, INPUT_PULLUP);
    pinMode(joy_horz_pin, INPUT);
    pinMode(joy_vert_pin, INPUT);
    pinMode(pressure_pin, INPUT);
    pinMode(brake_pos_pin, INPUT);
    pinMode(battery_pin, INPUT);
    pinMode(usd_cs_pin, OUTPUT);
    pinMode(tft_cs_pin, OUTPUT);
    pinMode(pot_wipe_pin, INPUT);
    pinMode(tp_irq_pin, INPUT);
    // pinMode(tft_ledk_pin, OUTPUT);

    // Set all outputs to known sensible values
    digitalWrite(tft_cs_pin, HIGH);   // Prevent bus contention
    digitalWrite(usd_cs_pin, HIGH);   // Prevent bus contention
    digitalWrite(tft_dc_pin, LOW);
    // digitalWrite(sim_pulse_pin, LOW);
    digitalWrite(pot_pwr_pin, HIGH);  // Power up the potentiometer
    digitalWrite(led_pin, HIGH);  // Light on

    analogReadResolution(adc_bits);  // Set Arduino Due to 12-bit resolution (default is same as Mega=10bit)

    // Arduino Due timer research:
    //
    // TCLK0 (PB26, Peripheral B) is external clock for Timer/Counter0, Ch0, Enabled by rtegister TC0XC0S
    // TCLK6 (PC27, Peripheral B) is external clock for Timer/Counter2, Ch0, Enabled by rtegister TC2XC0S (?)
    // Peripheral ID for programming interrupt for TC0: 27.  For TC2: 29
    // Enabling TC requires configuration of PMC (Power Mgmt Controller) to enable TC clock
    // QDEC must be disabled (default) for TC0 1 and 2 to be independent
    // Each 32bit TC channel has a 32bit counter, increments on rising edges of clock. Counter Value in TC_CV reg,  Overflow sets COVFS bit in TC_SR
    // Configure clock source for each channel with TCCLKS bit of TC_BMR.  External clk signals XC0, XC2.  Can invert with CLKI bit in TC_CMR, or configure burst (BURST in TC_CMR)
    // External clock freq must be 2.5x less than peripheral clock.  100 Hz x 2.5 = 250 Hz.  No problem, yeah?
    // Enable/disable clock with CLKEN and CLKDIS in TC_CCR.  See pg 862-863 for config of stop, disable, trigger events
    // Set mode to capture or waveform (PWM) using WAVE bit of TC_CMR
    // See waveform mode setup to configure PWM.  See Quadrature encoder setup on pg 875 to decode rotary encoders.
    // TC0XC0S = 0  B.26 D22  
    // Can sync up the three PWM outputs by setting SYNCx bits of PWM_SCM.  Synced channels get CPREx, CPRDx, and CALG0 fields from those of ch 0 instead of their own channel. Pg 985

    // PWM setup (for gas):    (found here: https://forum.arduino.cc/index.php?topic=386981.0)
    // Output 50Hz PWM at a resolution of 14 bits on pin DAC1 (D67) (pin: PB16, peripheral: PWML0)
    // The PWM channels can be multiplexed onto a number of different pins on the Due. Which PWM pins are available is specified by the mutliplexing tables in the SAM3X8E's datasheet.
    // The SAM3X8E has four tables for each of its 32-bit ports A, B, C and D. Most pins having two peripheral functions: "Peripheral A" and "Peripheral B". 
    // It's possible for each PWM channel to control two complementary (opposite of each other) square wave outputs, labelled in the table as PWMLx (low) and PWMHx (high).
    // It's first necessary to enable the PWM controller and multiplex the PWM controller's output to DAC1, which is pin PB16, using the REG_PMC_PCER1, REG_PIOB_ABSR and REG_PIOB_PDR reg.
    // The PWM controller's clock register (REG_PWM_CLK) allows you to either divide down the master clock (84MHz), either by using a prescaler, or clock divisor, or both. 
    // There are two clock divisor and prescaler registers: A and B, so you can generate up to two base frequencies for your 8 channels.
    // The channel mode register (REG_PWM_CMR0) connects the divided clock (2MHz) to channel 0 and selects the PWM mode of operation, in this case dual slope PWM, (center aligned).
    // REG_PWM_CPRD0 and REG_PWM_CDTY0 determine the period (frequency) and duty cycle (phase) respectively for a given channel. It's also necessary to enable ch 0 with the REG_PWM_ENA reg.
    // By the way, if you plan on changing the period or duty cycle during PWM operation, you'll need to use the REG_PWM_CPRDUPDx and REG_PWM_CDTYUPDx update registers for the given channel.
    // The equation for calculating the PWM frequency is also contained in the SAM3X8E's datasheet. For dual slope PWM:  PWMFrequency = MCLK/(2*CPRD*DIVA) = 84MHz/(2*20000*42) = 50Hz
    REG_PMC_PCER1 |= PMC_PCER1_PID36;                     // Enable PWM
    REG_PIOB_ABSR |= PIO_ABSR_P16;                        // Set PWM pin perhipheral type A or B, in this case B
    REG_PIOB_PDR |= PIO_PDR_P16;                          // Set PWM pin to an output
    REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(42);     // Set the PWM clock rate to 2MHz (84MHz/42)
    REG_PWM_CMR0 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;      // Enable dual slope PWM and set the clock source as CLKA
    REG_PWM_CPRD0 = pid_period_us;                        // Set the PWM frequency 2MHz/(2 * 20000) = 50Hz
    REG_PWM_CDTY0 = gas_pulse_idle_us;                    // Set the PWM duty cycle to nominal angle
    REG_PWM_ENA = PWM_ENA_CHID0;                          // Enable the PWM channel     
    // To change duty cycle:  CW limit: REG_PWM_CDTYUPD0 = 1000;      Center: REG_PWM_CDTYUPD0 = 1500;      CCW limit: REG_PWM_CDTYUPD0 = 2000;
    
    // analogWrite(steer_pwm_pin, steer_stop_pwm);   // Write values range from 0 to 255
    // analogWrite(gas_pwm_pin, gas_min_pwm);
    // analogWrite(brake_pwm_pin, brake_stop_pwm);
    
    // while (!Serial);     // needed for debugging?!
    Serial.begin(115200);  // Open serial port
    
    delay(500); // This is needed to allow the screen board enough time after a cold boot before we start trying to talk to it.
    if (display_enabled) {
        Serial.print(F("Init LCD... "));
        tft.begin();
        tft.setRotation(1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
        // tft.setFont(&FreeSans12pt7b);  // Use tft.setFont() to return to fixed font      
        for (int32_t lineno=0; lineno <= (int32_t)arraysize(telemetry); lineno++)  {
            disp_age_quanta[lineno] = -1;
            disp_ages_us[lineno] = 0;
            memset(disp_values[lineno],0,strlen(disp_values[lineno]));
        }
        for (int32_t row=0; row<disp_nobools; row++)  {
            disp_bool_values[row] = 0;
        }
        draw_text(false);
        Serial.println(F("Success"));

        Serial.print(F("Captouch initialization... "));
        if (! touchpanel.begin(40)) {     // pass in 'sensitivity' coefficient
            Serial.println(F("Couldn't start FT6206 touchscreen controller"));
            // while (1);
        }
        else Serial.println(F("Capacitive touchscreen started"));
    }
    /*
    while (1) {  // Useful to uncomment and move around the code to find points of crashing
        Serial.print(F("Alive, "));
        Serial.println(micros());
        delay(250);
    }
    */
    
    // 230417 removing sdcard init b/c boot is hanging here unless I insert this one precious SD card
    // Serial.print(F("Initializing filesystem...  "));  // SD card is pretty straightforward, a single call. 
    // if (! sd.begin(usd_cs_pin, SD_SCK_MHZ(25))) {   // ESP32 requires 25 mhz limit
    //     Serial.println(F("SD begin() failed"));
    //     for(;;); // Fatal error, do not continue
    // }
    // sd_init();
    // Serial.println(F("Filesystem started"));
    
    // Set up our interrupts
    Serial.print(F("Interrupts... "));
    attachInterrupt(digitalPinToInterrupt(tach_pulse_pin), tach_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(speedo_pulse_pin), speedo_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder_a_pin), encoder_a_isr, CHANGE); // One type of encoder (e.g. Panasonic EVE-YBCAJ016B) needs Rising int on pin A only
    attachInterrupt(digitalPinToInterrupt(encoder_b_pin), encoder_b_isr, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(encoder_sw_pin), encoder_sw_isr, CHANGE);
    Serial.println(F("Set up and enabled"));
    
    steer_servo.attach(steer_pwm_pin);
    brake_servo.attach(brake_pwm_pin);
    // gas_servo.attach(gas_pwm_pin);
    
    // steer_pid.setOutputRange(steer_min_pwm, steer_max_pwm);
    // gas_pid.setOutputRange(gas_min_pwm, gas_max_pwm);

    // brake_pid.setOutputRange(brake_pwm_retract_pwm, brake_pwm_extend_pwm);  // Lines only useful with FastPID library
    // cruise_pid.setOutputRange(engine_idle_rpm, engine_redline_rpm);  // Lines only useful with FastPID library

    loop_timer_us = micros();  // start timer to measure the first loop

    Serial.println(F("Setup finished"));
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

    // 0) Beginning-of-the-loop nonsense
    //
    if (serial_debugging && print_timestamps) {
        Serial.print("Loop# ");
        Serial.print(loopno);  Serial.print(": ");      
        loopzero = micros();  // Start time for loop
    }
    // Update derived variable values in case they have changed
    float gas_pid_ki_mhz = gas_pid_kc*gas_pid_fi_mhz;  // Convert dependent-form PID coefficients to independent term for each of the influences
    float gas_pid_kd_us = gas_pid_kc*gas_pid_td_us;  // Convert dependent-form PID coefficients to independent term for each of the influences
    float brake_pid_ki_mhz = brake_pid_kc*brake_pid_fi_mhz;  // Convert dependent-form PID coefficients to independent term for each of the influences
    float brake_pid_kd_us = brake_pid_kc*brake_pid_td_us;  // Convert dependent-form PID coefficients to independent term for each of the influences
    float brake_pid_pos_kp = brake_pid_kc*brake_pid_pos_kx;  // Convert dependent-form PID coefficients to independent term for each of the influences
    float cruise_pid_ki_mhz = cruise_pid_kc*cruise_pid_fi_mhz;  // Convert dependent-form PID coefficients to independent term for each of the influences
    float cruise_pid_kd_us = cruise_pid_kc*cruise_pid_td_us;  // Convert dependent-form PID coefficients to independent term for each of the influences
    int32_t joy_vert_deadband_bot_adc = (adc_range_adc-joy_vert_deadband_adc)/2;  // Lower threshold of vert joy deadband (ADC count 0-4095)
    int32_t joy_vert_deadband_top_adc = (adc_range_adc+joy_vert_deadband_adc)/2;  // Upper threshold of vert joy deadband (ADC count 0-4095)
    int32_t joy_horz_deadband_bot_adc = (adc_range_adc-joy_horz_deadband_adc)/2;  // Lower threshold of horz joy deadband (ADC count 0-4095)
    int32_t joy_horz_deadband_top_adc = (adc_range_adc+joy_horz_deadband_adc)/2;  // Upper threshold of horz joy deadband (ADC count 0-4095)
    int32_t engine_govern_rpm = map(gas_governor_percent, 0, 100, 0, engine_redline_rpm);  // Create an artificially reduced maximum for the engine speed
    int32_t gas_pulse_govern_us = map(gas_governor_percent*(engine_redline_rpm-engine_idle_rpm)/engine_redline_rpm, 0, 100, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
    int32_t carspeed_govern_mmph = map(gas_governor_percent, 0, 100, 0, carspeed_redline_mmph);  // Governor must scale the top vehicle speed proportionally
     
    now_us = micros();
    if (serial_debugging && print_timestamps) {
        // Serial.print(now_us-loopzero);  Serial.print(" ");
    }
    // 1) Gather new telemetry and filter the signals
    //
    
    // Serial.print("Point0: ");  Serial.println(carspeed_target_mmph);
    
    // Read misc input signals
    int32_t brake_pos_adc = analogRead(brake_pos_pin);

    // Potentiometer
    int32_t pot_adc = analogRead(pot_wipe_pin);
    pot_filt_adc = (int32_t)((pot_ema_alpha*(float)pot_adc) + ((1-pot_ema_alpha)*(float)pot_filt_adc));     // Apply EMA filter

    // Encoder
    //
    // Read and interpret encoder switch activity. Encoder rotation is handled in interrupt routine
    // Encoder handler routines should act whenever encoder_sw_action is true, setting it back to false once handled.
    // When handling press, if encoder_press_time is nonzero then press is a long press

    // encoder_a_activity <<< 1;
    // encoder_a_activity <<< 1;
    // encoder_a_raw = digitalRead(encoder_a_pin);
    // encoder_b_raw = digitalRead(encoder_b_pin);

    // encoder_state = (encoder_state << 1) | digitalRead(encoder_a_pin) | 0xe000;
    // if (encoder_state == 0xf000) {
    //    encoder_state = 0x0000;
    //    encoder_delta = digitalRead(encoder_b_pin) ? 1 : -1;
    //    encoder_counter += encoder_delta;
    // }

    if (!digitalRead(encoder_sw_pin)) {  // if encoder sw is being pressed (switch is active low)
        if (!encoder_sw) {  // if the press just occurred
            encoder_press_timer_us = now_us;  // start a press timer
            encoder_timer_on = true;  // flag to indicate timing for a possible long press
        }
        else if (encoder_timer_on && abs(now_us - encoder_press_timer_us) > encoder_long_press_us) {  // If press time exceeds long press threshold
            encoder_press_time_us = abs(now_us - encoder_press_timer_us);  // Store the press duration
            encoder_sw_action = true;  // Set flag to handle the long press event. Note, routine handling press should clear this
            encoder_timer_on = false;  // Keeps us from entering this logic again until after next sw release (to prevent repeated long presses)
            encoder_suppress_click = true;  // Prevents the switch release after a long press from causing a short press
        }
        encoder_sw = true;  // Remember a press is in effect
    }
    else {  // if encoder sw is not being pressed
        if (!encoder_sw_action) encoder_press_time_us = 0; // If actions have been handled, clear the previous stored press duration
        if (encoder_sw && !encoder_suppress_click) encoder_sw_action = true;  // if the switch was just released, a short press occurred, which must be handled
        encoder_timer_on = false;  // Allows detection of next long press event
        encoder_sw = false;  // Remember press is not in effect
        encoder_suppress_click = false;  // End click suppression
    }
    
    // Voltage of vehicle battery
    int32_t battery_adc = analogRead(battery_pin);
    battery_mv = (int32_t)(battery_max_mv*((float)battery_adc)/adc_range_adc);  // convert adc value read into mV
    battery_filt_mv = (int32_t)((battery_ema_alpha*(float)battery_mv) + ((1-battery_ema_alpha)*(float)battery_filt_mv));     // Apply EMA filter
    
    // Read sensors
    if (simulate) {  // Any sensor data which should be mucked with or overridden automatically and continuously, during simulation should get set in here
        if (brake_pos_adc < brake_pos_zeropoint_adc) brake_pos_adc = brake_pos_zeropoint_adc;  // To keep brake position in legal range during simulation
    }
    else {    // When not simulating, read real sensors and filter them.  Just those that would get taken over by the simulator go in here.
        ignition = digitalRead(ignition_pin);
        basicmodesw = !digitalRead(basicmodesw_pin);   // 1-value because electrical signal is active low
        // neutral = 1-digitalRead(neutral_pin);           // 1-value because electrical signal is active low
        cruise_sw = !digitalRead(cruise_sw_pin);       // 1-value because electrical signal is active low

        // Tach
        
        // Method 1: This works with the ISR implementation having micros() call inside the ISR
        if (abs(now_us - tach_timer_us) < engine_stop_timeout_us)  engine_rpm = (int32_t)(60000000/(float)tach_delta_us);  // Tachometer magnets/us * 60000000 (1 rot/magnet * 1000000 us/sec * 60 sec/min) gives rpm
        else engine_rpm = 0;  // If timeout since last magnet is exceeded
        if (abs(engine_rpm-engine_old_rpm) > engine_lp_thresh_rpm || engine_rpm-engine_last_rpm < engine_spike_thresh_rpm) {  // Remove noise spikes from tach values, if otherwise in range
            engine_old_rpm = engine_last_rpm;
            engine_last_rpm = engine_rpm;
        }
        else engine_rpm = engine_last_rpm;  // Spike detected - ignore that sample
        if (engine_rpm)  engine_filt_rpm = (int32_t)((engine_rpm_ema_alpha*(float)engine_rpm) + ((1-engine_rpm_ema_alpha)*(float)engine_filt_rpm));     // Sensor EMA filter
        else engine_filt_rpm = 0;    

        // Method 2: This should work also, perhaps better at high speeds, and uses a squeaky clean ISR implementation.  But it doesn't :(        
        // if (tach_magnet_count) {
        //     engine_rpm = (int32_t)(60000000 * (float)tach_magnet_count/(float)loop_period_us);  // Tachometer (magnets/loop) / (us/loop) * 60000000 (1 rot/magnet * 1000000 us/sec * 60 sec/min) gives rpm
        //     tach_timer_us = micros();
        //     tach_magnet_count = 0;
        // }
        // else if (now_us-tach_timer_us < engine_stop_timeout_us) engine_rpm = 0;  // If the vehicle stop timeout is exceeded, consider rpm is zero
        // if (engine_rpm)  engine_filt_rpm = (int32_t)((engine_rpm_ema_alpha*(float)engine_rpm) + ((1-engine_rpm_ema_alpha)*(float)engine_filt_rpm));     // Sensor EMA filter
        // else engine_filt_rpm = 0;    

        // Speedo    
        if (abs(now_us - speedo_timer_us) < car_stop_timeout_us)  carspeed_mmph = (int32_t)(179757270/(float)speedo_delta_us); // Update car speed value  
            // magnets/us * 179757270 (1 rot/magnet * 1000000 us/sec * 3600 sec/hr * 1/19.85 gearing * 20*pi in/rot * 1/12 ft/in * 1000/5280 milli-mi/ft gives milli-mph  // * 1/1.15 knots/mph gives milliknots
            // Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
        else carspeed_mmph = 0;     
        if (abs(carspeed_mmph-carspeed_old_mmph) > carspeed_lp_thresh_mmph || carspeed_mmph-carspeed_last_mmph < carspeed_spike_thresh_mmph) {  // Remove noise spikes from speedo values, if otherwise in range
            carspeed_old_mmph = carspeed_last_mmph;
            carspeed_last_mmph = carspeed_mmph;
        }
        else carspeed_mmph = carspeed_last_mmph;  // Spike detected - ignore that sample
        if (carspeed_mmph)  carspeed_filt_mmph = (int32_t)((carspeed_ema_alpha*(float)carspeed_mmph) + ((1-carspeed_ema_alpha)*(float)carspeed_filt_mmph));     // Sensor EMA filter
        else carspeed_filt_mmph = 0;
    
        // Brake pressure.  Read sensor, then Remove noise spikes from brake feedback, if reading is otherwise in range
        int32_t pressure_adc = analogRead(pressure_pin);
        if (abs(pressure_adc-pressure_old_adc) > pressure_lp_thresh_adc || pressure_adc-pressure_last_adc < pressure_spike_thresh_adc) {
            pressure_old_adc = pressure_last_adc;
            pressure_last_adc = pressure_adc;
        }
        else pressure_adc = pressure_last_adc;  // Spike detected - ignore that sample
        // pressure_psi = (int32_t)(1000*(float)(pressure_adc)/adc_range_adc);      // Convert pressure to units of psi
        pressure_filt_adc = (int32_t)((pressure_ema_alpha*(float)pressure_adc) + ((1-pressure_ema_alpha)*(float)pressure_filt_adc));     // Sensor EMA filter
    }

    now_us = micros();
    // if (print_timestamps) Serial.print(now_us-loopzero);  Serial.print(" ");

    // 2) Read joystick then determine new steering setpoint
    //
    joy_vert_adc = analogRead(joy_vert_pin);  // Read joy vertical
    joy_horz_adc = analogRead(joy_horz_pin);  // Read joy horizontal

    if (!simulate || sim_halfass) {  // If not fully simulating 
        if (joy_vert_adc > joy_vert_deadband_bot_adc && joy_vert_adc < joy_vert_deadband_top_adc)  joy_vert_filt_adc = adc_midscale_adc;  // if joy vert is in the deadband, set joy_vert_filt to center value
        else joy_vert_filt_adc = (int32_t)(joy_ema_alpha*joy_vert_adc + (1-joy_ema_alpha)*joy_vert_filt_adc);  // otherwise do ema filter to determine joy_vert_filt

        // Serial.print(joy_vert_deadband_top_adc); // joy_horz_deadband_top_adc, joy_horz_max_adc, steer_pulse_stop_us, steer_pulse_right_us ");

        if (joy_horz_adc > joy_horz_deadband_bot_adc && joy_horz_adc < joy_horz_deadband_top_adc)  joy_horz_filt_adc = adc_midscale_adc;  // if joy horz is in the deadband, set joy_horz_filt to center value
        else joy_horz_filt_adc = (int32_t)(joy_ema_alpha*joy_horz_adc + (1-joy_ema_alpha)*joy_horz_filt_adc);  // otherwise do ema filter to determine joy_horz_filt
    }
    if (!(runmode == SHUTDOWN && (!carspeed_filt_mmph || shutdown_complete)))  { // If not in shutdown mode with shutdown complete and car stopped
        if (joy_horz_filt_adc >= joy_horz_deadband_top_adc) steer_pulse_out_us = map(joy_horz_filt_adc, joy_horz_deadband_top_adc, joy_horz_max_adc, steer_pulse_stop_us, steer_pulse_right_us);  // Figure out the steering setpoint if joy to the right of deadband
        else if (joy_horz_filt_adc <= joy_horz_deadband_bot_adc) steer_pulse_out_us = map(joy_horz_filt_adc, joy_horz_deadband_bot_adc, joy_horz_min_adc, steer_pulse_stop_us, steer_pulse_left_us);  // Figure out the steering setpoint if joy to the left of deadband
        else steer_pulse_out_us = steer_pulse_stop_us;  // Stop the steering motor if inside the deadband
        // Serial.print("joy_horz_filt_adc, joy_horz_deadband_top_adc, joy_horz_max_adc, steer_pulse_stop_us, steer_pulse_right_us ");
        // Serial.print(joy_horz_filt_adc);  Serial.print(" "); Serial.print(joy_horz_deadband_top_adc);  Serial.print(" "); Serial.print(joy_horz_max_adc);  Serial.print(" "); Serial.print(steer_pulse_stop_us); Serial.print(" "); Serial.println(steer_pulse_right_us);
        // Serial.print("joy V/H: ");  Serial.print(joy_vert_adc); Serial.print(" ");  Serial.print(joy_vert_filt_adc); Serial.print(" ");  Serial.print(joy_horz_adc); Serial.print(" ");  Serial.println(joy_horz_filt_adc);
    }

    // Serial.print("Point1: ");  Serial.println(carspeed_target_mmph);
    
    // 3) Check if our current runmode has been overridden by certain specific conditions
    //
    if (basicmodesw) runmode = BASIC;    // if basicmode switch on --> Basic Mode
    else if (!ignition) runmode = SHUTDOWN;  //} && laboratory != true)  {  // otherwise if ignition off --> Shutdown Mode
    else if (!engine_filt_rpm)  runmode = STALL;    // otherwise if engine not running --> Stall Mode
    
    now_us = micros();
    if (print_timestamps) {
        // Serial.print(now_us-loopzero);  Serial.print(" ");
    }

    // 4) Do actions based on which runmode we are in (and set gas/brake setpoints), and possibly change runmode 
    //
    if (runmode == BASIC)  {  // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering stell works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            motor_park_timer_us = now_us;  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        if ((!basicmodesw) && engine_filt_rpm)  runmode = HOLD;  // If we turned off the basic mode switch with engine running, go to Hold mode. If engine is not running, we'll end up in Stall Mode automatically
    }
    else if (runmode == SHUTDOWN)  { // In shutdown mode we stop the car if it's moving then park the motors.
        if (ignition) runmode = STALL;
        // if (basicmodesw)  shutdown_complete = true;    // If basic mode switch is enabled then stop any in-progress shutdown sequence
        else if (we_just_switched_modes)  {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            gas_target_rpm = engine_idle_rpm;  //  Release the throttle 
            shutdown_complete = false;
            if (carspeed_filt_mmph)  {
                pressure_target_adc = brake_hold_initial_adc;  // More brakes, etc. to stop the car
                brake_timer_us = now_us;
                sanity_timer_us = now_us;
            }
        }
        if (!shutdown_complete)  {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (!carspeed_filt_mmph || abs(now_us - sanity_timer_us) > sanity_timeout_us)  {  // If car has stopped, or timeout expires, then release the brake
                motor_park_timer_us = now_us;  // Set a timer to timebox this effort
                park_the_motors = true;  // Flags the motor parking to happen
                // pressure_target_adc = pressure_min_adc;  // Start to Fully release brakes
                // if (pressure_filt_adc <= pressure_min_adc + pressure_margin_adc)  shutdown_complete = true;  // With this set, we will do nothing from here on out (until mode changes, i.e. ignition)
                if (pressure_filt_adc <= pressure_min_adc + pressure_margin_adc)  shutdown_complete = true;  // With this set, we will do nothing from here on out (until mode changes, i.e. ignition)
            }
            else if (abs(brake_timer_us - now_us) > brake_increment_interval_us)  {
                pressure_target_adc += brake_hold_increment_adc;  // Slowly add more brakes until car stops
                brake_timer_us = now_us;  
            }
            else if (!park_the_motors) shutdown_complete = true;
        }
    }
    else if (runmode == STALL)  {   // In stall mode, the gas doesn't have feedback
        if (engine_filt_rpm)  runmode = HOLD;  //  Enter Hold Mode if we started the car
        else {  // Actuators still respond and everything, even tho engine is turned off
            pressure_target_adc = pressure_min_adc;  // Default when joystick not pressed
            gas_pulse_out_us = gas_pulse_idle_us;  // Default when joystick not pressed
            if (joy_vert_filt_adc >= joy_vert_deadband_top_adc)  { //  If we are pushing up
                // In stall mode there is no engine rom for PID to use as feedback, so we bypass the PID and just set the gas_target_angle proportional to 
                // the joystick position.  This works whether there is normally a gas PID or not.
                gas_pulse_out_us = map(joy_vert_filt_adc, joy_vert_deadband_top_adc, joy_vert_max_adc, gas_pulse_idle_us, gas_pulse_govern_us);
                // gas_target_rpm = engine_idle_rpm+engine_range_rpm*((float)joy_vert_filt_adc-(float)joy_vert_deadband_top_adc)/(float)joy_vert_deadband_bot_adc;
            }
            else if (joy_vert_filt_adc <= joy_vert_deadband_bot_adc)  {  // If we are pushing down
                // Scale joystick value to pressure adc setpoint
                // pressure_target_adc = scalefloat((float)joy_vert_filt_adcpot_filtered_adc, 0.0, 4096.0, (float)pressure_min_adc, (float)pressure_max_adc);
                pressure_target_adc = map(joy_vert_filt_adc, joy_vert_deadband_bot_adc, joy_vert_min_adc, pressure_min_adc, pressure_max_adc);
                // Serial.print(joy_vert_filt_adc);  Serial.print(" ");
                // Serial.print(joy_vert_deadband_bot_adc);  Serial.print(" ");
                // Serial.print(joy_vert_min_adc);  Serial.print(" ");
                // Serial.print(pressure_min_adc);  Serial.print(" ");
                // Serial.print(pressure_min_adc);  Serial.print(" ");
                // Serial.println(pressure_target_adc);  //Serial.print(" ");
                // pressure_min_adc+(pressure_max_adc-pressure_min_adc)*((float)joy_vert_deadband_bot_adc-(float)joy_vert_filt_adc)/(float)joy_vert_deadband_bot_adc;
            }
        }
    }
    else if (runmode == HOLD)  {
        // Serial.println("Welcome to Hold mode");   Serial.print(" ");
        // Serial.print(joy_vert_filt_adc);   Serial.print(" ");
        // Serial.print(joy_vert_deadband_top_adc);   Serial.print(" ");
        // Serial.print(neutral);   Serial.print(" ");
        if (joy_vert_filt_adc >= joy_vert_deadband_top_adc)  runmode = FLY; // Enter Fly Mode if joystick is pushed up
        else if (we_just_switched_modes)  {  // Release throttle and push brake upon entering hold mode
            gas_target_rpm = engine_idle_rpm;  // Let off gas (if gas using PID mode)
            if (!carspeed_filt_mmph)  pressure_target_adc += brake_hold_increment_adc; // If the car is already stopped then just add a touch more pressure and then hold it.
            else pressure_target_adc = brake_hold_initial_adc;  //  Otherwise, these hippies need us to stop the car for them
            brake_timer_us = now_us;
        }
        else if (carspeed_filt_mmph && abs(brake_timer_us - now_us) > brake_increment_interval_us)  { // Each interval the car is still moving, push harder
            pressure_target_adc += brake_hold_increment_adc;  // Slowly add more brakes until car stops
            brake_timer_us = now_us;
        }
        pressure_target_adc = constrain(pressure_target_adc, pressure_min_adc, pressure_max_adc);  // Just make sure we don't try to push harder than we can 
        //Serial.print("runmode: ");   Serial.println(runmode);
    }
    else if (runmode == FLY)  {
        // Serial.println("Welcome to Fly mode");   Serial.print(" ");
        if (we_just_switched_modes)  {
            gesture_progress = 0;
            gesture_timer_us = abs(now_us - (gesture_flytimeout_us+1)); // Initialize gesture timer to already-expired value
            cruise_sw_held = false;
            cruise_sw_timer_us = now_us;
        }
        if (!carspeed_filt_mmph && joy_vert_filt_adc <= joy_vert_deadband_bot_adc)  runmode = HOLD;  // Go to Hold Mode if we have braked to a stop
        else  {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            gas_target_rpm = engine_idle_rpm;  // Default when joystick not pressed 
            pressure_target_adc = pressure_min_adc;  // Default when joystick not pressed   
            if (joy_vert_filt_adc > joy_vert_deadband_top_adc)  {  // If we are trying to accelerate
                gas_target_rpm = map(joy_vert_filt_adc, joy_vert_deadband_top_adc, joy_vert_max_adc, engine_idle_rpm, engine_govern_rpm);
            }
            else if (joy_vert_filt_adc < joy_vert_deadband_bot_adc)  {  // If we are trying to brake, scale joystick value to determine pressure adc setpoint
                pressure_target_adc = map(joy_vert_filt_adc, joy_vert_deadband_bot_adc, joy_vert_min_adc, pressure_min_adc, pressure_max_adc);
            }
        }
        // Cruise mode can be entered by pressing a physical momentary button, or by performing a specific joystick gesture, depending on the cruise_gesturing flag.
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (cruise_gesturing) {  // If we are configured to use joystick gestures to go to cruise mode, the gesture is 
            if (!gesture_progress && joy_vert_filt_adc >= joy_vert_deadband_bot_adc && joy_vert_filt_adc <= joy_vert_deadband_top_adc)  { // Re-zero gesture timer for potential new gesture whenever joystick at center
                gesture_timer_us = now_us;
            }
            if (abs(now_us - gesture_timer_us) >= gesture_flytimeout_us) gesture_progress = 0; // If gesture timeout has expired, cancel any in-progress gesture
            else {  // Otherwise check for successful gesture motions
                if (!gesture_progress && joy_vert_filt_adc >= joy_vert_max_adc-default_margin_adc)  { // If joystick quickly pushed to top, step 1 of gesture is successful
                    gesture_progress++;
                    gesture_timer_us = now_us;
                }
                else if (gesture_progress == 1 && joy_vert_filt_adc <= joy_vert_min_adc+default_margin_adc)  { // If joystick then quickly pushed to bottom, step 2 succeeds
                    gesture_progress++;
                    gesture_timer_us = now_us;
                }
                else if (gesture_progress == 2 && joy_vert_filt_adc >= joy_vert_deadband_bot_adc && joy_vert_filt_adc <= joy_vert_deadband_top_adc) { // If joystick then quickly returned to center, go to Cruise mode
                    runmode = CRUISE;
                }        
            }
        }
        else {  // If cruise mode is entered by long press of a cruise button
            if (!cruise_sw) {  // If button not currently pressed
                if (cruise_sw_held && abs(now_us - cruise_sw_timer_us) > cruise_sw_timeout_us)  runmode = CRUISE;  // If button was just held long enough, upon release enter Cruise mode
                cruise_sw_held = false;  // Cancel button held state
            }
            else if (!cruise_sw_held) {  // If button is being pressed, but we aren't in button held state
                cruise_sw_timer_us = now_us; // Start hold time timer
                cruise_sw_held = true;  // Get into that state
            }
        }         
    }
    else if (runmode == CRUISE)  {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            carspeed_target_mmph = carspeed_filt_mmph;  // Begin cruising with cruise set to current speed
            pressure_target_adc = pressure_min_adc;  // Let off the brake and keep it there till out of Cruise mode
            gesture_timer_us = now_us;
            cruise_sw_held = false;
        }
        if (joy_vert_filt_adc > joy_vert_deadband_top_adc) {  // When joystick vert above center, increase the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            gas_target_rpm = map(joy_vert_filt_adc, joy_vert_deadband_top_adc, joy_vert_max_adc, engine_filt_rpm, engine_govern_rpm);
        }
        else if (joy_vert_filt_adc < joy_vert_deadband_bot_adc) {  // When joystick vert below center, decrease the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            gas_target_rpm = map(joy_vert_filt_adc, joy_vert_min_adc, joy_vert_deadband_bot_adc, engine_idle_rpm, engine_filt_rpm);
        }
        else {  // if joystick at center
            if (cruise_adjusting) carspeed_target_mmph = carspeed_filt_mmph;  // Upon return to center set speed target to current speed
            cruise_adjusting = false;
            gesture_timer_us = now_us;  // reset gesture timer
        }
        if (joy_vert_filt_adc <= joy_vert_min_adc+default_margin_adc && abs(now_us - gesture_timer_us) < gesture_flytimeout_us)  runmode = FLY;  // If joystick quickly pushed to bottom 
        if (cruise_sw)  cruise_sw_held = true;   // Pushing cruise button sets up return to fly mode
        else if (cruise_sw_held) { // Release of button drops us back to fly mode
            cruise_sw_held = false;
            runmode = FLY;
        }
        if (!carspeed_filt_mmph)  {  // In case we slam into a brick wall, get out of cruise mode
            if (serial_debugging) Serial.println(F("Error: Car stopped or taken out of gear in cruise mode"));  // , carspeed_filt_mmph, neutral
            runmode = HOLD;  // Back to Hold Mode  
        }
    }
    else { // Obviously this should never happen
        if (serial_debugging) Serial.println(F("Error: Invalid runmode entered"));  // ,  runmode
        runmode = HOLD;
    }

    now_us = micros();
    if (serial_debugging && print_timestamps) {
        // Serial.print(now_us-loopzero);  Serial.print(" ");
    }
    
    // Serial.print("Point3: ");  Serial.println(carspeed_target_mmph);

    // 5) Step the pids, update the actuator outputs  (at regular intervals)
    //
    if (now_us-pid_timer_us > pid_period_us && !(runmode == SHUTDOWN && shutdown_complete))  {  // Recalculate pid and update outputs, at regular intervals
        steer_pulse_out_us = constrain(steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);  // Don't be out of range
        steer_servo.writeMicroseconds(steer_pulse_out_us);   // Write steering value to jaguar servo interface
        
        int32_t brake_pos_max_temp_adc = brake_pos_zeropoint_adc;;
        if (park_the_motors) {  // First check if we're in motor parking mode, if so park motors instead of running PIDs
            if ( ( abs(brake_pos_adc - brake_pos_park_adc) <= default_margin_adc &&  // IF ( the brake motor is close enough to the park position AND
                abs(gas_pulse_out_us == gas_pulse_park_us) )                         //      so is the gas servo )
                || abs(now_us - motor_park_timer_us) > motor_park_timeout_us ) {     //      OR the parking timeout has expired
                park_the_motors = false;                                             // THEN stop trying to park the motors
            }
            else brake_pos_max_temp_adc = brake_pos_park_adc;
        }
        else if (runmode != BASIC) {  // Unless basicmode switch is turned on, we want brake and gas
            // Here is the brake PID math
            pressure_target_adc = constrain(pressure_target_adc, pressure_min_adc, pressure_max_adc);  // Make sure pressure target is in range
            brake_pid_error_adc = pressure_target_adc - pressure_filt_adc;  // Determine the error in pressure
            brake_pid_p_term_adc = (int32_t)(brake_pid_kc*(float)brake_pid_error_adc);
            brake_pid_integral_adcus += brake_pid_error_adc*pid_period_us;  // Calculate pressure integral
            brake_pid_i_term_adc = constrain((int32_t)(brake_pid_ki_mhz*(float)brake_pid_integral_adcus), pressure_min_adc-pressure_max_adc, pressure_max_adc-pressure_min_adc);  // limit integral to 2x the full range of the input
            brake_pid_derivative_adcperus = (float)((brake_pid_error_adc - brake_pid_error_last_adc))/(float)pid_period_us;  // Calculate pressure derivative
            brake_pid_d_term_adc = brake_pid_kd_us*(float)brake_pid_derivative_adcperus;
            if (brake_pos_adc < brake_pos_zeropoint_adc) brake_pid_pos_error_adc = brake_pos_zeropoint_adc-brake_pos_adc; // Additional position influence to ensure actuator position doesn't go below the zero pressure point
            else brake_pid_pos_error_adc = 0;
            brake_pid_pos_term_adc = brake_pid_pos_kp*(float)brake_pid_pos_error_adc;
            pressure_delta_adc = brake_pid_p_term_adc + brake_pid_i_term_adc + brake_pid_d_term_adc + brake_pid_pos_term_adc;  // Add all the terms and scale to get delta in adc counts
            
            if (pressure_delta_adc > 0) brake_pulse_out_us = map(pressure_filt_adc + pressure_delta_adc, pressure_filt_adc, pressure_max_adc, brake_pulse_stop_us, brake_pulse_retract_us);
            else if (pressure_delta_adc < 0) brake_pulse_out_us = map(pressure_filt_adc + pressure_delta_adc, pressure_filt_adc, pressure_min_adc, brake_pulse_stop_us, brake_pulse_extend_us);
            else brake_pulse_out_us = brake_pulse_stop_us;

            brake_pid_error_last_adc = brake_pid_error_adc;  // For use next time in pressure derivative calculation and hysteresis behavior            
        }
        // Override pid for brake if position is out of allowed range, correct that instead
        if (brake_pos_adc + default_margin_adc <= brake_pos_max_temp_adc) brake_pulse_out_us = map (brake_pos_adc, brake_pos_max_temp_adc, brake_pos_retracted_adc, brake_pulse_stop_us, brake_pulse_extend_us); // If brake is too far retracted, extend toward park point, slowing as we approach
        if (brake_pos_adc - default_margin_adc >= brake_pos_max_temp_adc) brake_pulse_out_us = map (brake_pos_adc, brake_pos_max_temp_adc, brake_pos_extended_adc, brake_pulse_stop_us, brake_pulse_retract_us); // If brake is too far extended, retract toward park point, slowing as we approach
        // Send to the actuator
        brake_pulse_out_us = constrain(brake_pulse_out_us, brake_pulse_retract_us, brake_pulse_extend_us);  // Refuse to exceed range
        brake_servo.writeMicroseconds(brake_pulse_out_us);  // Write result to jaguar servo interface

        // brake_pulse_out_us = map(pressure_delta_adc+pressure_filt_adc, pressure_min_adc, pressure_max_adc, brake_pulse_extend_us, brake_pulse_retract_us);  // Scale pressure adc value to range of PWM pulse on-time
        
        // if (carspeed_filt_mmph == 0 && abs(pressure_filt_adc - pressure_delta_adc) < pressure_hysteresis_adc) {  // If desired change is less than hysteresis threshold
        //     if (pressure_hysteresis_hold) {  // If we are holding due to already having met the target
        //         brake_pulse_out_us = brake_pulse_stop_us;  // Keep the motor stopped. Keeps from constantly jittering when error is small
        //     }
        // }
        // else pressure_hysteresis_hold = false;  // Error is past hysteresis threshold, stop holding the motor
        // if (brake_pid_error_adc * brake_pid_error_last_adc < 0) pressure_hysteresis_hold = true;  // If error crossed zero in last two readings, stay at zero point
        
        // if (abs(pressure_filt_adc - pressure_delta_adc) < pressure_hysteresis_adc) {  // If desired change is less than hysteresis threshold
        //     if (pressure_hyseresis_hold) {  // If we are holding due to already having met the target
        //         brake_pulse_out_us = brake_pulse_stop_us;  // Keep the motor stopped. Keeps from constantly jittering when error is small
        //     }
        //     else pressure_hysteresis_hold = false;  // Error is past hysteresis threshold, stop holding the motor
        // }
        // if (brake_pid_error_adc * brake_pid_error_last_adc < 0) pressure_hysteresis_hold = true;  // If error crossed zero in last two readings, stay at zero point
        // brake_pid_error_last_adc = brake_pid_error_adc;  // For use next time in pressure derivative calculation and hysteresis behavior
        
        // Serial.print("Brake: ");                                 Serial.print(" ");  //
        // Serial.print(brake_pos_adc);                             Serial.print(" ");  // Brake position current adc value
        // Serial.print(brake_pos_zeropoint_adc);                   Serial.print(" ");  // Brake position zero point
        // Serial.print(brake_pid_pos_error_adc);                   Serial.print(" ");  // Brake position error
        // Serial.print(pressure_filt_adc);                         Serial.print(" ");  // Pressure sensor feedback value (adc)
        // Serial.print(pressure_target_adc);                       Serial.print(" ");  // Pressure target setpoint (adc count)
        // Serial.print(brake_pid_kc*brake_pid_error_adc);          Serial.print(" ");  // Proportional component
        // Serial.print(brake_pid_i_term_adc);                      Serial.print(" ");  // Integral component
        // Serial.print((int32_t)(brake_pid_ki_mhz*(float)brake_pid_integral_adcus)); Serial.print(" ");  //        
        // Serial.print(brake_pid_d_term_adc);                      Serial.print(" ");  // Derivative component
        // Serial.print(brake_pid_pos_kp*brake_pid_pos_error_adc);  Serial.print(" ");  // Derivative component
        // Serial.print(pressure_filt_adc);                         Serial.print(" ");  // Filtered sensor feedback value
        // Serial.print(pressure_adc);                              Serial.print(" ");  // Raw sensor feedback value
        // Serial.print(pressure_delta_adc);                        Serial.print(" ");  // Raw sensor feedback value
        // Serial.println(brake_pulse_out_us);                      // Serial.print(" ");  // Raw sensor feedback value
        // brake_pwm_out_pwm = brake_pid.step(pressure_target_adc, pressure_adc);  // Old attempt to use library PID loop
            
        if (runmode != BASIC) {  // Unless basicmode switch is turned on, we want brake and gas

            if (runmode == CRUISE && !cruise_adjusting) {  // Cruise loop updates gas rpm target to keep speed equal to cruise mmph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
                // gas_target_rpm = cruise_pid.step(carspeed_target_mmph, carspeed_filt_mmph);
                carspeed_target_mmph = constrain(carspeed_target_mmph, 0, carspeed_redline_mmph);
                cruise_pid_error_mmph = carspeed_target_mmph - carspeed_filt_mmph;  // Determine the mmph error
                cruise_pid_p_term_mmph = (int32_t)(cruise_pid_kc*(float)cruise_pid_error_mmph);
                cruise_pid_integral_mmphus += cruise_pid_error_mmph*pid_period_us;  // Calculate mmph integral
                cruise_pid_i_term_mmph = constrain((int32_t)(cruise_pid_ki_mhz*(float)cruise_pid_integral_mmphus), carspeed_idle_mmph-carspeed_redline_mmph, carspeed_redline_mmph-carspeed_idle_mmph);  // limit integral to 2x the full range of the input
                cruise_pid_derivative_mmphperus = (float)((cruise_pid_error_mmph - cruise_pid_error_last_mmph))/(float)pid_period_us;  // Calculate mmph derivative
                cruise_pid_error_last_mmph = cruise_pid_error_mmph;  // For use next time in mmph derivative calculation
                cruise_pid_d_term_mmph = cruise_pid_kd_us*(float)cruise_pid_derivative_mmphperus;
                carspeed_delta_mmph = cruise_pid_p_term_mmph + cruise_pid_i_term_mmph + cruise_pid_d_term_mmph;  // Add all the terms and scale to get delta from center in mmph
                if (carspeed_delta_mmph > 0) gas_target_rpm = map(carspeed_delta_mmph+carspeed_filt_mmph, carspeed_filt_mmph, carspeed_govern_mmph, engine_filt_rpm, engine_govern_rpm);  // Scale up rpm target based on mmph delta
                else gas_target_rpm = map(carspeed_delta_mmph+carspeed_filt_mmph, carspeed_idle_mmph, carspeed_filt_mmph, engine_idle_rpm, engine_filt_rpm);  // Scale down rpm target based on mmph delta
                // Serial.print("Cruise: ");                                                     Serial.print(" ");  //
                // Serial.print(carspeed_target_mmph);                                           Serial.print(" ");  // Raw sensor feedback value
                // Serial.print(cruise_pid_kc*cruise_pid_error_mmph);                            Serial.print(" ");  // Proportional component
                // Serial.print(cruise_pid_kc*cruise_pid_fi_mhz*cruise_pid_integral_mmphus);     Serial.print(" ");  // Integral component
                // Serial.print(cruise_pid_kc*cruise_pid_td_us*cruise_pid_derivative_mmphperus); Serial.print(" ");  // Derivative component
                // Serial.print(carspeed_filt_mmph);                                             Serial.print(" ");  // Filtered sensor feedback value
                // Serial.print(carspeed_delta_mmph);                                            Serial.print(" ");  // Raw sensor feedback value
                // Serial.print(carspeed_delta_mmph+carspeed_idle_mmph);                         Serial.print(" ");  // Raw sensor feedback value
                // Serial.println(gas_target_rpm);                                               // Serial.print(" ");  // Raw sensor feedback value
            }

            if (park_the_motors) gas_pulse_out_us = gas_pulse_park_us;
            else if (runmode != STALL) {  // If Hold, Fly or Cruise mode, then we need to determine gas actuator output from rpm target
                gas_target_rpm = constrain(gas_target_rpm, engine_idle_rpm, engine_govern_rpm);  // Make sure desired rpm isn't out of range (due to crazy pid math, for example)

                if (gas_pid) {  // If use of gas pid is enabled, calculate pid to get pulse output from rpm target
                    gas_pid_error_rpm = gas_target_rpm - engine_filt_rpm;  // Determine the rpm error
                    gas_pid_p_term_rpm = (int32_t)(gas_pid_kc*(float)gas_pid_error_rpm);
                    gas_pid_integral_rpmus += gas_pid_error_rpm*pid_period_us;  // Calculate rpm integral
                    gas_pid_i_term_rpm = constrain((int32_t)(gas_pid_ki_mhz*(float)gas_pid_integral_rpmus), engine_idle_rpm-engine_govern_rpm, engine_govern_rpm-engine_idle_rpm);  // Prevent integral runaway by limiting it to 2x the full range of the input
                    gas_pid_derivative_rpmperus = (float)((gas_pid_error_rpm - gas_pid_error_last_rpm))/(float)pid_period_us;  // Calculate rpm derivative
                    gas_pid_error_last_rpm = gas_pid_error_rpm;  // For use next time in rpm derivative calculation
                    gas_pid_d_term_rpm = gas_pid_kd_us*(float)gas_pid_derivative_rpmperus;
                    gas_delta_rpm = gas_pid_p_term_rpm + gas_pid_i_term_rpm + gas_pid_d_term_rpm;  // Add all the terms and scale to get delta from center in rpm
                    gas_pulse_out_us = map(gas_delta_rpm+engine_idle_rpm, engine_idle_rpm, engine_govern_rpm, gas_pulse_idle_us, gas_pulse_govern_us);  // Scale rpm alue to range of PWM pulse on-time
                    // Serial.print("Gas: ");                                              Serial.print(" ");  //
                    // Serial.print(gas_target_rpm);                                       Serial.print(" ");  // Raw sensor feedback value
                    // Serial.print(gas_pid_kc*gas_pid_error_rpm);                         Serial.print(" ");  // Proportional component
                    // Serial.print(gas_pid_kc*gas_pid_fi_mhz*gas_pid_integral_rpmus);     Serial.print(" ");  // Integral component
                    // Serial.print(gas_pid_kc*gas_pid_td_us*gas_pid_derivative_rpmperus); Serial.print(" ");  // Derivative component
                    // Serial.print(engine_filt_rpm);                                      Serial.print(" ");  // Filtered sensor feedback value
                    // Serial.print(gas_delta_rpm);                                        Serial.print(" ");  // Raw sensor feedback value
                    // Serial.print(gas_delta_rpm+engine_idle_rpm);                        Serial.print(" ");  // Raw sensor feedback value
                    // Serial.println(gas_pulse_out_us);                                   // Serial.print(" ");  // Raw sensor feedback value
                }
                else {  // With open-loop gas control, throttle angle is directly proportional to target value
                    gas_pulse_out_us = map(gas_target_rpm, engine_idle_rpm, engine_govern_rpm, gas_pulse_idle_us, gas_pulse_govern_us); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
                }
            }
            gas_pulse_out_us = constrain(gas_pulse_out_us, gas_pulse_govern_us, gas_pulse_idle_us);  // Make sure pulse time is in range
            // Serial.println(gas_pulse_out_us);
            REG_PWM_CDTYUPD0 = gas_pulse_out_us;  // Update the pin duty cycle.
            // gas_servo.writeMicroseconds(gas_pulse_out_us);  // Update gas pwm output (with gas pid or servo mode)
        }
        pid_timer_us = now_us; // reset timer to trigger the next update
    }
    
    // 5.5) Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
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
    if (ignition && engine_filt_rpm > 0) { // See if the engine is turning despite the ignition being off
        Serial.println(F("Detected engine rotation in the absense of ignition signal"));  // , engine_filt_rpm, ignition
        // I don't think we really need to panic about this, but it does seem curious. Actually this will probably occur when we're sliding
        // into camp after a ride, and kill the engine before we stop the car. For a fraction of a second the engine would keep turning anyway.
        // Or fopr that matter whenever the carb is out of tune and making the engine diesel after we kill the ign.
    }

    // Serial.print("Point4: ");  Serial.println(carspeed_target_mmph);

    now_us = micros();
    if (print_timestamps) {
        // Serial.print(now_us-loopzero);  Serial.print(" ");
    }
    
    // 6) Service the user interface
    //
    if (abs(now_us - touch_timer_us) > touch_period_us) {
        touch_timer_us = now_us;
        if (touchpanel.touched()) { // Take actions upon being touched
            TS_Point touchpoint = touchpanel.getPoint();   // Retreive a point
            touchpoint.x = map(touchpoint.x, 0, disp_height_pix, disp_height_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touchpoint.y = map(touchpoint.y, 0, disp_width_pix, disp_width_pix, 0);  // Rotate touch coordinates to match tft coordinates
            int32_t touch_y = tft.height()-touchpoint.x; // touch point y coordinate in pixels, from origin at top left corner
            int32_t touch_x = touchpoint.y; // touch point x coordinate in pixels, from origin at top left corner
            int32_t touch_row = (int32_t)((float)touch_y/touch_cell_height_pix); // which of our 4 rows of touch buttons was touched?
            int32_t touch_col = (int32_t)((float)touch_x/touch_cell_width_pix); // which of our 5 columns of touch buttons was touched?
            // Take appropriate touchscreen actions depending how we're being touched
            touch_accel = 1 << touch_accel_exponent;  // determine value editing rate
            if (touch_row == 0 && touch_col == 0 && laboratory && !touch_now_touched) simulate = !simulate; // If the top left touch button is pressed (and laboratory variable is set)
            else if (touch_row == 0 && touch_col == 1 && !touch_now_touched) {
                sim_dataset_page += 1; // Displayed dataset page can also be changed outside of simulator
                if (sim_dataset_page >= (int32_t)arraysize(pagecard)) sim_dataset_page -= arraysize(pagecard);
            }
            else if (simulate) {  // if we are currenttly using the our built-in vehicle simulator tool,
                if (touch_row == 0 && touch_col == 2) pressure_filt_adc += touch_accel;  // (+= 25) Pressed the increase brake pressure button
                else if (touch_row == 0 && touch_col == 3) engine_filt_rpm += touch_accel;  // (+= 25) Pressed the increase engine rpm button
                else if (touch_row == 0 && touch_col == 4) carspeed_filt_mmph += touch_accel;  // (+= 50) // Pressed the increase vehicle speed button
                else if (touch_row == 1 && touch_col == 0 && !touch_now_touched) {  // Pressed the select value button, for real time tuning of variables. Only take action once per touch
                    if (sim_tuning_ctrl == OFF) sim_selected_value = 0;  // if entering select mode from off mode, select first variable
                    if (sim_tuning_ctrl != SELECT) sim_tuning_ctrl = SELECT;  // if now entering select mode, don't change selection
                    else {
                        sim_selected_value += 1;  // Otherwise select the next variable
                        if (sim_selected_value >= (int32_t)arraysize(dataset_pages[sim_dataset_page])) sim_selected_value -= arraysize(dataset_pages[sim_dataset_page]);
                        if (sim_dataset_page >= 4) sim_selected_value = constrain (sim_selected_value, 5, 7);  // Skip unchangeable values for all PID modes
                        else if (sim_dataset_page == JOY) sim_selected_value = constrain (sim_selected_value, 2, 7);  // Skip unchangeable values for joy mode
                    }
                }
                else if (touch_row == 1 && touch_col == 1 && !touch_now_touched) basicmodesw = !basicmodesw;  // Pressed the basic mode toggle button. Toggle value, only once per touch
                else if (touch_row == 1 && touch_col == 2) pressure_filt_adc -= touch_accel;  // (-= 25) Pressed the decrease brake pressure button
                else if (touch_row == 1 && touch_col == 3) engine_filt_rpm -= touch_accel;  // (-= 25) Pressed the decrease engine rpm button
                else if (touch_row == 1 && touch_col == 4) carspeed_filt_mmph -= touch_accel;  // (-= 50) Pressed the decrease vehicle speed button
                else if (touch_row == 2 && touch_col == 0);  // Pressed a button that doesn't do anything
                else if (touch_row == 2 && touch_col == 1 && !touch_now_touched) ignition = !ignition; // Pressed the ignition switch toggle button. Toggle value, only once per touch
                else if (touch_row == 2 && touch_col == 2) {  // Pressed the decrease value button, for real time tuning of variables
                    if (sim_tuning_ctrl == SELECT) sim_tuning_ctrl = EDIT;  // If just entering edit mode, don't change value yet
                    else if (sim_tuning_ctrl == EDIT) sim_modify_polarity_touch = -touch_accel;  // If in edit mode, decrease value
                }
                else if (touch_row == 2 && touch_col == 3) joy_vert_filt_adc += touch_accel;  // (+= 25) Pressed the joystick up button
                else if (touch_row == 2 && touch_col == 4) {  // Pressed the increase value button, for real time tuning of variables
                    if (sim_tuning_ctrl == SELECT) sim_tuning_ctrl = EDIT;  // If just entering edit mode, don't change value yet
                    else if (sim_tuning_ctrl == EDIT) sim_modify_polarity_touch = touch_accel;  // If in edit mode, decrease value
                }   
                else if (touch_row == 3 && touch_col == 0);  // Pressed a button that doesn't do anything
                else if (touch_row == 3 && touch_col == 1) cruise_sw = true;  // Pressed the cruise mode button. This is a momentary control, not a toggle. Value changes back upon release
                else if (touch_row == 3 && touch_col == 2) joy_horz_filt_adc -= touch_accel;  // (-= 25) Pressed the joystick left button
                else if (touch_row == 3 && touch_col == 3) joy_vert_filt_adc -= touch_accel;  // (-= 25) Pressed the joystick down button
                else if (touch_row == 3 && touch_col == 4) joy_horz_filt_adc += touch_accel;  // (+= 25) Pressed the joystick right button   
                // Make sure any edits made are kept within valid boundaries
                joy_horz_filt_adc = constrain(joy_horz_filt_adc, joy_horz_min_adc, joy_horz_max_adc); 
                                  
                // Serial.print("joy_vert_filt before = "); Serial.print(joy_vert_filt_adc);
                joy_vert_filt_adc = constrain(joy_vert_filt_adc, joy_vert_min_adc, joy_vert_max_adc);                    
                // Serial.print(" after = "); Serial.println(joy_vert_filt_adc);
                
                pressure_filt_adc = constrain(pressure_filt_adc, pressure_min_adc, pressure_max_adc);                    
                engine_filt_rpm = constrain(engine_filt_rpm, 0, engine_redline_rpm);
                carspeed_filt_mmph = constrain(carspeed_filt_mmph, 0, carspeed_redline_mmph);
            }  // (if simulate)
            if (touch_accel_exponent < touch_accel_exponent_max &&  // If timer is > the shift time * exponent, and not already maxed, double the edit speed by
                abs(now_us - touch_start_marker_us) > (touch_accel_exponent + 1) * touch_accel_shift_period_us) touch_accel_exponent++;  // increasing the exponent
            touch_now_touched = true;
        }  // (if touchpanel reads a touch)
        else {  // If not being touched, put momentarily-set simulated button values back to default values
            if (simulate) {
                cruise_sw = false;  // Makes this button effectively momentary
                sim_modify_polarity_touch = 0;  // Stop changing value
            }
            touch_now_touched = false;  // remember last touch state
            touch_start_marker_us = now_us;
            touch_accel_exponent = 0;

        }
    }
    // Encoder handling
    //
    if (encoder_sw_action) {  // First deal with any unhandled switch press events
        if (serial_debugging) Serial.println("in encoder sw handling function");
        if (encoder_press_time_us) {  // If the press is a long press
            if (sim_tuning_ctrl == EDIT) sim_tuning_ctrl = SELECT;
            else if (sim_tuning_ctrl == SELECT) sim_tuning_ctrl = OFF;
            else simulate = !simulate;
        }
        else {  // if short press
            if (sim_tuning_ctrl == EDIT) sim_tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (sim_tuning_ctrl == SELECT) sim_tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
            else if (simulate) {
                sim_tuning_ctrl = SELECT;  // Let us select a value to edit
                sim_selected_value = arraysize(dataset_pages[sim_dataset_page]-1);  // Highlight the bottom value
            }
            //  else do any action is appropriate on encoder short click in context of not simulating
        }
        encoder_sw_action = false; // Our responsibility to reset this flag after handling events
    }
    if (encoder_delta != 0) {  // Now handle any new rotations
        if (serial_debugging) { Serial.print("in encoder rotation handler. dataset_page = "); Serial.println(sim_dataset_page); }
        int32_t encoder_spinrate_temp_us = (int32_t)(float(abs(now_us - encoder_spinspeed_timer_us))/(float)abs(encoder_delta));
        if (encoder_spinrate_temp_us >= encoder_spinrate_min_us) {  // Attempt to reject clicks coming in too fast
            encoder_spinrate_old_us = encoder_spinrate_last_us;
            encoder_spinrate_last_us = encoder_spinrate_us;
            encoder_spinrate_us = constrain(encoder_spinrate_temp_us, encoder_spinrate_min_us, 100000);
            encoder_spinrate_temp_us = (encoder_spinrate_old_us > encoder_spinrate_last_us) ? encoder_spinrate_old_us : encoder_spinrate_last_us;
            encoder_spinrate_temp_us = (encoder_spinrate_temp_us > encoder_spinrate_us) ? encoder_spinrate_temp_us : encoder_spinrate_us;
            encoder_edits_per_det = map(encoder_spinrate_temp_us, encoder_spinrate_min_us, 100000, 50, 1);  // if turning faster than 100ms/det, accelerate the effect of each detent
            encoder_spinspeed_timer_us = micros();
            if (sim_tuning_ctrl == EDIT) sim_modify_polarity_encoder = encoder_delta * encoder_edits_per_det;  // If a tunable value is being edited, turning the encoder changes the value
            else if (sim_tuning_ctrl == SELECT) {
                if (!sim_selected_value && encoder_delta < 0) encoder_delta = 0;
                sim_selected_value += encoder_delta;
                if (sim_dataset_page >= 4) sim_selected_value = constrain (sim_selected_value, 5, 7);  // Skip unchangeable values for all PID modes
                else if (sim_dataset_page == JOY) sim_selected_value = constrain (sim_selected_value, 2, 7);  // Skip unchangeable values for joy mode
                else sim_selected_value = constrain(sim_selected_value, 0, (int32_t)arraysize(dataset_pages[sim_dataset_page])-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
            }
            else if (sim_tuning_ctrl == OFF) {
                if (!sim_dataset_page && encoder_delta < 0) encoder_delta = 0;
                sim_dataset_page += encoder_delta;
                sim_dataset_page = constrain(sim_dataset_page, 0, (int32_t)arraysize(pagecard)-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
            }
        }
        encoder_delta = 0;
    }

    // Implement effects of changes made by encoder or touchscreen to simulate, sim_dataset_page, sim_selected_value, or sim_tuning_ctrl
    //
    if (sim_dataset_page != sim_dataset_page_last) {  // If we switched the displayed dataset page
        sim_tuning_ctrl = OFF;  // This should get taken care of elsewhere, but can't hurt to be sure?
        if (simulate && display_enabled) draw_touchgrid(true);  // If we are in the simulator, redraw missing touch grid since the corner will have been botched
        if (display_enabled) draw_text(true);  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
    }
    if (simulate) {  // if simulating
        if (sim_dataset_page == LOCK) sim_tuning_ctrl = OFF;  // Can not select or edit when on LOCK page
        if (!simulate_last && display_enabled) draw_touchgrid(false); // if we just entered simulator draw the touch grid over the display, in its entirety
    }
    else { // if not simulating
        sim_tuning_ctrl = OFF;
        if (simulate_last && display_enabled) {  // if we just quit the simulator
            disp_redraw_all = true;  // Signal drawing functions to redraw everything (shouldn't need this)
            draw_text(false);  // Reset the screen, completely, to get rid of button grid
        }
    }
    
    // Change tunable values when editing
    sim_modify_polarity = sim_modify_polarity_encoder;  // Allow edits using the encoder
    if (abs(sim_modify_timer_us - now_us) > sim_modify_period_us) {  // Also, on regular intervals,
        sim_modify_polarity += sim_modify_polarity_touch;  // allow edits using the touchscreen
        sim_modify_timer_us=now_us;  // reset the timer
    }
    if (sim_tuning_ctrl == EDIT && sim_modify_polarity != 0) {
        if (sim_dataset_page == JOY)  switch (sim_selected_value) {
            case 2:  joy_horz_min_adc = sim_edit(joy_horz_min_adc, sim_modify_polarity);  break;
            case 3:  joy_horz_max_adc = sim_edit(joy_horz_max_adc, sim_modify_polarity);  break;
            case 4:  joy_horz_deadband_adc = sim_edit(joy_horz_deadband_adc, sim_modify_polarity);  break;
            case 5:  joy_vert_min_adc = sim_edit(joy_vert_min_adc, sim_modify_polarity);  break;
            case 6:  joy_vert_max_adc = sim_edit(joy_vert_max_adc, sim_modify_polarity);  break;
            case 7:  joy_vert_deadband_adc = sim_edit(joy_vert_deadband_adc, sim_modify_polarity);  break;
            // case 2:  joy_horz_min_adc += sim_modify_polarity;  break;
            // case 3:  joy_horz_max_adc += sim_modify_polarity;  break;
            // case 4:  joy_horz_deadband_adc += sim_modify_polarity;  break;
            // case 5:  joy_vert_min_adc += sim_modify_polarity;  break;
            // case 6:  joy_vert_max_adc += sim_modify_polarity;  break;
            // case 7:  joy_vert_deadband_adc += sim_modify_polarity;  break;
        }
        else if (sim_dataset_page == CAR)  switch (sim_selected_value) {
            case 0:  gas_governor_percent += sim_modify_polarity;  break;
            case 1:  engine_idle_rpm += sim_modify_polarity;  break;
            case 2:  engine_redline_rpm += sim_modify_polarity;  break;
            case 3:  carspeed_idle_mmph += sim_modify_polarity;  break;
            case 4:  carspeed_redline_mmph += sim_modify_polarity;  break;
            case 5:  gas_pid = (sim_modify_polarity+1)/2;  break;
            case 6:  cruise_gesturing = (sim_modify_polarity+1)/2;  break;
            case 7:  brake_pos_zeropoint_adc += sim_modify_polarity;  break;
        }
        else if (sim_dataset_page == PWMS)  switch (sim_selected_value) {
            case 0:  steer_pulse_left_us += sim_modify_polarity;  break;
            case 1:  steer_pulse_stop_us += sim_modify_polarity;  break;
            case 2:  steer_pulse_right_us += sim_modify_polarity;  break;
            case 3:  brake_pulse_extend_us += sim_modify_polarity;  break;
            case 4:  brake_pulse_stop_us += sim_modify_polarity;  break;
            case 5:  brake_pulse_retract_us += sim_modify_polarity;  break;
            case 6:  gas_pulse_idle_us += sim_modify_polarity;  break;
            case 7:  gas_pulse_redline_us += sim_modify_polarity;  break;
        }
        else if (sim_dataset_page == BPID)  switch (sim_selected_value) {
            case 5:  brake_pid_kc += 0.001*(float)sim_modify_polarity;  break;
            case 6:  brake_pid_fi_mhz += 0.001*(float)sim_modify_polarity;  break;
            case 7:  brake_pid_td_us += 0.001*(float)sim_modify_polarity;  break;
        }
        else if (sim_dataset_page == GPID)  switch (sim_selected_value) {
            case 5:  gas_pid_kc += 0.001*(float)sim_modify_polarity;  break;
            case 6:  gas_pid_fi_mhz += 0.001*(float)sim_modify_polarity;  break;
            case 7:  gas_pid_td_us += 0.001*(float)sim_modify_polarity;  break;
        }
        else if (sim_dataset_page == CPID)  switch (sim_selected_value) {
            case 5:  cruise_pid_kc += 0.001*(float)sim_modify_polarity;  break;
            case 6:  cruise_pid_fi_mhz += 0.001*(float)sim_modify_polarity;  break;
            case 7:  cruise_pid_td_us += 0.001*(float)sim_modify_polarity;  break;
        }
    }
    
    // Update displayed telemetry values to the screen
    if (display_enabled)  {
        draw_value(0, 0, 2);
        draw_value(0, 0, 3);
        if (simulate) draw_touchgrid(true); // Redraw only the at-risk content of the touch grid
        draw_value(0, runmode, 1);
        draw_value(1, carspeed_filt_mmph, 0);
        draw_value(2, engine_filt_rpm, 0);
        draw_value(3, pressure_filt_adc, 0);
        draw_value(4, joy_horz_filt_adc, 0);
        draw_value(5, joy_vert_filt_adc, 0);
        draw_value(6, steer_pulse_out_us, 0);
        draw_value(7, carspeed_target_mmph, 0);
        draw_value(8, gas_target_rpm, 0);
        draw_value(9, gas_pulse_out_us, 0);
        draw_value(10, pressure_target_adc, 0);
        draw_value(11, brake_pulse_out_us, 0);
        if (sim_dataset_page == LOCK) {
            draw_value(12, battery_filt_mv, 0);
            draw_value(13, brake_pos_adc, 0);
            draw_value(14, pot_filt_adc, 0);
            draw_value(15, pot_adc, 0);
            // draw_value(16, digitalRead(tach_pulse_pin), 0);
            // draw_value(17, tach_magnet_count, 0);
            // draw_value(18, loop_freq_hz, 0);
            draw_value(16, encoder_a_raw, 0);
            draw_value(17, encoder_b_raw, 0);
            draw_value(18, encoder_state, 0);
            draw_value(19, encoder_counter, 0);
        }
        else if (sim_dataset_page == JOY) {
            draw_value(12, joy_horz_adc, 0);
            draw_value(13, joy_vert_adc, 0);
            draw_value(14, joy_horz_min_adc, 0);
            draw_value(15, joy_horz_max_adc, 0);
            draw_value(16, joy_horz_deadband_adc, 0);
            draw_value(17, joy_vert_min_adc, 0);
            draw_value(18, joy_vert_max_adc, 0);
            draw_value(19, joy_vert_deadband_adc, 0);
        }
        else if (sim_dataset_page == CAR) {
            draw_value(12, gas_governor_percent, 0);
            draw_value(13, engine_idle_rpm, 0);
            draw_value(14, engine_redline_rpm, 0);
            draw_value(15, carspeed_idle_mmph, 0);
            draw_value(16, carspeed_redline_mmph, 0);
            draw_value(17, gas_pid, 0);
            draw_value(18, cruise_gesturing, 0);
            draw_value(19, brake_pos_zeropoint_adc, 0);   
        }
        else if (sim_dataset_page == PWMS) {
            draw_value(12, steer_pulse_left_us, 0);
            draw_value(13, steer_pulse_stop_us, 0);
            draw_value(14, steer_pulse_right_us, 0);
            draw_value(15, brake_pulse_extend_us, 0);
            draw_value(16, brake_pulse_stop_us, 0);
            draw_value(17, brake_pulse_retract_us, 0);
            draw_value(18, gas_pulse_idle_us, 0);
            draw_value(19, gas_pulse_redline_us, 0);
        }
        else if (sim_dataset_page == BPID) {
            draw_value(12, brake_pid_error_adc, 0);
            draw_value(13, (int32_t)(brake_pid_kc*(float)brake_pid_error_adc), 0);
            draw_value(14, brake_pid_i_term_adc, 0);
            draw_value(15, brake_pid_d_term_adc, 0);
            draw_value(16, pressure_delta_adc, 0);
            draw_value(17, (int32_t)(1000*brake_pid_kc), 0);
            draw_value(18, (int32_t)(1000000*brake_pid_fi_mhz), 0);
            draw_value(19, (int32_t)(1000*brake_pid_td_us), 0);
        }
        else if (sim_dataset_page == GPID) {
            draw_value(12, gas_pid_error_rpm, 0);
            draw_value(13, (int32_t)(gas_pid_kc*(float)gas_pid_error_rpm), 0);
            draw_value(14, gas_pid_i_term_rpm, 0);
            draw_value(15, gas_pid_d_term_rpm, 0);
            draw_value(16, gas_delta_rpm, 0);
            draw_value(17, (int32_t)(1000*gas_pid_kc), 0);
            draw_value(18, (int32_t)(1000000*gas_pid_fi_mhz), 0);
            draw_value(19, (int32_t)(1000*gas_pid_td_us), 0);
        }
        else if (sim_dataset_page == CPID) {
            draw_value(12, cruise_pid_error_mmph, 0);
            draw_value(13, (int32_t)(cruise_pid_kc*(float)cruise_pid_error_mmph), 0);
            draw_value(14, cruise_pid_i_term_mmph, 0);
            draw_value(15, cruise_pid_d_term_mmph, 0);
            draw_value(16, carspeed_delta_mmph, 0);
            draw_value(17, (int32_t)(1000*cruise_pid_kc), 0);
            draw_value(18, (int32_t)(1000000*cruise_pid_fi_mhz), 0);
            draw_value(19, (int32_t)(1000*cruise_pid_td_us), 0);    
        }
        draw_bool(basicmodesw, 1);
        draw_bool(ignition, 2);
        // draw_bool(neutral, 2);
        draw_bool(cruise_sw, 3);
    }
    // Update memory of current state for the next loop
    sim_modify_polarity_touch = 0;
    sim_modify_polarity_encoder = 0;
    disp_redraw_all = false;
    sim_dataset_page_last = sim_dataset_page;
    sim_selected_value_last = sim_selected_value;
    simulate_last = simulate;
    sim_tuning_ctrl_last = sim_tuning_ctrl;    

    // 7) SD card
    //
    // [Do card storage operations]

    // 8) Do the control loop bookkeeping at the end of each loop
    //
    now_us = micros();
    if (serial_debugging && print_timestamps) {
        // Serial.print(now_us-loopzero);
        // Serial.print(".  Loop in ");
        Serial.print((float)(abs(now_us-loopzero)/1000));
        Serial.print(" ms, ");
        Serial.print(1000000/((float)(abs(now_us-loopzero))));
        Serial.println(" Hz");
    }
    loopno++;  // I like to count how many loops
    if (runmode != SHUTDOWN) shutdown_complete = false;
    if (runmode != oldmode) we_just_switched_modes = true;      // If changing runmode, set this so new mode logic can perform initial actions
    else we_just_switched_modes = false;    // Reset this variable
    oldmode = runmode;   // remember what mode we're in for next time
    loop_period_us = abs(micros() - loop_timer_us);  // abs is to handle when micros() overflows back to 0
    if (!loop_period_us) loop_period_us++;  // ensure loop period is never zero since it gets divided by
    loop_freq_hz = (int32_t)(1000000/(float)loop_period_us);
    loop_timer_us = micros();
}