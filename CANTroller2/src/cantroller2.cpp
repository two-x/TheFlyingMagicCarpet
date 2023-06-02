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
// #include <Adafruit_GFX.h>  // For drawing pictures & text on the screen
#include "classes.h"  // Contains our data structures
#include "spid.h"

using namespace std;
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

#define arraysize(x)  ((int32_t)(sizeof(x) / sizeof((x)[0])))  // A macro function to determine the length of string arrays
#define floor(amt, lim) ( (amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)
// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
// #define min(a, b) ( (a <= b) ? a : b)
// #define max(a, b) ( (a >= b) ? a : b)

// LCD is 2.8in diagonal, 240x320 pixels
// LCD supports 18-bit color, but GFX library uses 16-bit color, organized (MSB) 5b-red, 6b-green, 5b-blue (LSB)
// Since the RGB don't line up with the nibble boundaries, it's tricky to quantify a color, here are some colors:
// Color picker websites: http://www.barth-dev.de/online/rgb565 , https://chrishewett.com/blog/true-rgb565-colour-picker/
#define BLK  0x0000
#define BLU  0x001f
#define RED  0xf800
#define DRED 0xb000
#define GRN  0x07e0
#define CYN  0x07ff  // 00000 111 111 11111 
#define DCYN 0x0575  //
#define MGT  0xf81f
#define ORG  0xfca0 
#define YEL  0xffe0
#define LYEL 0xfff8
#define WHT  0xffff
#define DGRY 0x39c7  // very dark grey
#define GRY1 0x8410  // 10000 100 000 10000 = 84 10  dark grey
#define GRY2 0xc618  // 11000 110 000 11000 = C6 18  light grey
#define LGRY 0xd6ba  // very light grey
#define PNK  0xfc1f  // Pink is the best color
#define DPNK 0xbad7  // We need all shades of pink
#define LPNK 0xfe1f  // Especially light pink, the champagne of pinks

// Defines for all the GPIO pins we're using
#ifdef ESP32_SX_DEVKIT
    #define button_pin 0  // (button0) - This is one of the buttons on the esp32 board
    #define joy_horz_pin 1  // (adc) - Analog input, tells us left-right position of joystick. Take complement of ADC value gives:  Low values for left, High values for right.
    #define joy_vert_pin 2  // (adc) - Analog input, tells us up-down position of joystick. Take complement of ADC value gives:  Low values for down, High values for up.
    #define tft_dc_pin 3  // Output, Assert when sending data to display chip to indicate commands vs. screen data
    #define battery_pin 4  // (adc) -  Analog input, mule battery voltage level, full scale is 15.638V
    #define pot_wipe_pin 5  // (adc) - Analog input, tells us position of attached potentiometer (useful for debug, etc.)
    #define brake_pos_pin 6  // (adc) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
    #define pressure_pin 7  // (adc) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
    #define i2c_sda_pin 8  // (i2c0 sda / adc) - Hijack these pins for the touchscreen and micro-sd i2c bus
    #define i2c_scl_pin 9  // (i2c0 scl / adc) -  Hijack these pins for the touchscreen and micro-sd i2c bus
    #define tft_cs_pin 10  // (spi0 cs) -  Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
    #define tft_mosi_pin 11  // (spi0 mosi) - Used as spi interface to tft screen
    #define tft_sclk_pin 12  // (spi0 sclk) - Used as spi interface to tft screen
    #define tft_miso_pin 13  // (spi0 miso) - Used as spi interface to tft screen
    #define steer_pwm_pin 14  // (pwm0) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
    #define brake_pwm_pin 15  // (pwm1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
    #define gas_pwm_pin 16  // (pwm1) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
    #define hotrc_horz_pin 17  // (pwm0) - Hotrc thumb joystick input.
    #define hotrc_vert_pin 18  // (pwm0) - Hotrc bidirectional trigger input
    #define tp_irq_pin 19  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
    #define hotrc_ch4_pin 20  // Hotrc Ch3 toggle output, used to toggle cruise mode
    #define hotrc_ch3_pin 21  // (pwm0) - Hotrc Ch4 toggle output, used to panic stop & kill ignition
    #define tach_pulse_pin 35  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
    #define speedo_pulse_pin 36  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
    #define ignition_pin 37  // Output flips a relay to kill the car ignition, active high (no pullup)
    #define basicmodesw_pin 38  // Input, asserted to tell us to run in basic mode.   (needs pullup)
    #define encoder_sw_pin 39  // Int input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
    #define encoder_b_pin 40  // Int input, The B pin (aka DT pin) of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
    #define encoder_a_pin 41  // Int input, The A pin (aka CLK pin) of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
    #define heartbeat_led_pin 42  // There is not actually an LED here, so this is basically a free pin
    #define uart0_tx_pin 43  // (uart0 tx) - Reserve for possible jaguar interface
    #define uart0_rx_pin 44  // (uart0 rx) - Reserve for possible jaguar interface
    #define cruise_sw_pin 45  // Input, momentary button low pulse >500ms in fly mode means start cruise mode. Any pulse in cruise mode goes to fly mode. Active low. (needs pullup)
    #define tft_ledk_pin 46  // Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
    #define usd_cs_pin 47  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
    #define neopixel_pin 48  // (rgb led) - Data line to onboard Neopixel WS281x

    #define pot_pwr_pin -1  // Unused on esp32
    #define led_rx_pin -1  // Unused on esp32
    #define led_tx_pin -1  // Unused on esp32

#else  // Applies to Due
    #define usd_cs_pin 4  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
    #define tft_ledk_pin 5  // Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
    #define tp_irq_pin 7  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
    #define tft_dc_pin 9  // Output, Assert when sending data to display chip to indicate commands vs. screen data
    #define tft_cs_pin 10  // Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
    #define heartbeat_led_pin 13  // Output, This is the LED labeled "L" onboard the arduino due.  Active high.
    #define encoder_sw_pin 18  // Int input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
    #define encoder_b_pin 19  // Int input, The B pin (aka DT pin) of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
    #define encoder_a_pin 21  // Int input, The A pin (aka CLK pin) of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
    #define speedo_pulse_pin 23  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
    #define tach_pulse_pin 25  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
    #define pot_pwr_pin 27  // Output, Lets us supply the optional external potentiometer with 3.3V power
    #define steer_pwm_pin 29  // Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
    #define neopixel_pin 31  // Output, no neopixel for due
    #define hotrc_horz_pin 35
    #define hotrc_vert_pin 37
    #define hotrc_ch3_pin 39
    #define hotrc_ch4_pin 41
    #define brake_pwm_pin 43  // Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
    #define gas_pwm_pin 45  // Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
    #define basicmodesw_pin 47  // Input, asserted to tell us to run in basic mode.   (needs pullup)
    #define ignition_pin 49  // Output flips a relay to kill the car ignition, active high (no pullup)
    #define cruise_sw_pin 51  // Input, momentary button low pulse >500ms in fly mode means start cruise mode. Any pulse in cruise mode goes to fly mode. Active low. (needs pullup)
    #define led_rx_pin 72  // Another on-board led (Due only)
    #define led_tx_pin 73  // Another on-board led (Due only)
    #define pot_wipe_pin A6  // Analog input, tells us position of attached potentiometer (useful for debug, etc.)
    #define battery_pin A7  // Analog input, mule battery voltage level, full scale is 15.638V
    #define joy_horz_pin A8  // Analog input, tells us left-right position of joystick. Take complement of ADC value gives:  Low values for left, High values for right.
    #define joy_vert_pin A9  // Analog input, tells us up-down position of joystick. Take complement of ADC value gives:  Low values for down, High values for up.
    #define pressure_pin A10  // Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
    #define brake_pos_pin A11  // Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.

    #define button_pin -1
#endif

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
#define disp_fixed_lines 11  // Lines of static variables/values always displayed
#define disp_tuning_lines 8  // Lines of dynamic variables/values in dataset pages 
#define disp_line_height_pix 12  // Pixel height of each text line. Screen can fit 16x 15-pixel or 20x 12-pixel lines
#define disp_vshift_pix 2  // Unknown.  Note: At smallest text size, characters are 5x7 pix + pad on rt and bot for 6x8 pix.
#define disp_font_height 8
#define disp_font_width 6
#define disp_bargraph_width 40
#define disp_bargraph_squeeze 1
#define touch_cell_v_pix 48  // When touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53  // When touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1  // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen
bool display_enabled = true;  // Should we run 325x slower in order to get bombarded with tiny numbers?  Probably.

enum dataset_pages {LOCK, JOY, CAR, PWMS, BPID, GPID, CPID};

char telemetry[disp_fixed_lines][10] = {  
    "   Speed:",
    "    Tach:",
    "Brk Pres:",   
    "Joy Horz:",
    "Joy Vert:",
    "CruisTgt:",
    " Brk Tgt:",
    " Gas Tgt:",
    " Brk PWM:",
    " Gas PWM:",
    "SteerPWM:",
};
char pagecard[7][5] = { "Run ", "Joy ", "Car ", "PWMs", "Bpid", "Gpid", "Cpid" };
char dataset_page_names[arraysize(pagecard)][disp_tuning_lines][12] = {
    {   " Battery:",  // LOCK
        " Brk Pos:",
        "  P-on-E:",
        "SimBkPos:",
        " Sim Joy:",
        "Sim Pres:",
        "Sim Tach:",
        " Sim Spd:", },
    {   "Horz Raw:",  // JOY
        "Vert Raw:",
        "Horz Min:",
        "Horz Max:",
        " Horz DZ:",
        "Vert Min:",
        "Vert Max:",
        " Vert DZ:", },
    {   "Governor:",  // CAR
        "Eng Idle:",
        "Eng RedL:",
        "Spd Idle:",
        "Spd RedL:",
        "Joystck?:",
        "GasOpenL:",
        "BrkPosZP:", },
    {   "Str Left:",  // PWMS
        "Str Stop:",
        "Str Rght:",
        "Brk Extd:",
        "Brk Stop:",
        "Brk Retr:",
        "Gas Idle:",
        "Gas RedL:", },
    {   "Pres Err:",  // BPID
        "  P Term:",
        "  I Term:",
        "  D Term:",
        " PID Out:",
        "  Kp (P):",
        "  Ki (I):",
        "  Kd (D):", },
    {   " Eng Err:",  // GPID
        "  P Term:",
        "  I Term:",
        "  D Term:",
        " PID Out:",
        "  Kp (P):",
        "  Ki (I):",
        "  Kd (D):" },
    {   " Spd Err:",  // CPID
        "  P Term:",
        "  I Term:",
        "  D Term:",
        " PID Out:",
        "  Kp (P):",
        "  Ki (I):",
        "  Kd (D):", },
};
char units[disp_fixed_lines][5] = {"mmph", "rpm ", "adc ", "adc ", "adc ", "mmph", "adc ", "rpm ", "\xe5s  ", "\xe5s  ", "\xe5s  " };
char tuneunits[arraysize(pagecard)][disp_tuning_lines][5] = {
    { "mV  ", "adc ", "adc ", "    ", "    ", "    ", "    ", "    " },  // LOCK
    { "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc " },  // JOY
    { "%   ", "rpm ", "rpm ", "mmph", "mmph", "    ", "    ", "adc " },  // CAR
    { "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  " },  // PWM
    { "adc ", "adc ", "adc ", "adc ", "adc ", "/1k ", "mHz ", "ms  " },  // BPID
    { "mmph", "mmph", "mmph", "mmph", "mmph", "/1k ", "mHz ", "ms  " },  // GPID
    { "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "/1k ", "mHz ", "ms  " },  // CPID
};
char simgrid[4][3][5] = {
    { "prs+", "rpm+", "car+" },
    { "prs-", "rpm-", "car-" },
    { "    ", " \x1e  ", "    " },
    { " \x11  ", " \x1f  ", "  \x10 " },
};
char modecard[6][7] = { "Basic", "Shutdn", "Stall", "Hold", "Fly", "Cruise" };
char side_menu_buttons[5][4] = { "PG ", "SEL", "+  ", "-  ", "SIM" };  // Pad shorter names with spaces on the right
char top_menu_buttons[3][6] = { "BASIC", " IGN ", "CRUIS" };  // Pad shorter names with spaces to center

int32_t colorcard[arraysize(modecard)] = { MGT, RED, ORG, YEL, GRN, CYN };

enum tuning_ctrl_states {OFF, SELECT, EDIT};
int32_t tuning_ctrl = OFF;
int32_t tuning_ctrl_last = OFF;
Timer tuningCtrlTimer(25000000);  // This times out edit mode after a a long period of inactivity

#define adc_bits 12
#define adc_range_adc 4095    // = 2^12-1
#define adc_midscale_adc 2047
#define serial_debugging false
#define print_timestamps false  // Makes code write out timestamps throughout loop to serial port

enum runmodes {BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE};
int32_t runmode = SHUTDOWN;
int32_t oldmode = runmode;  // So we can tell when the mode has just changed

// Settable calibration values and control parameters
//
enum ctrls { HOTRC };  // This is a bad hack. Since JOY is already enum'd as 1 for dataset pages
bool ctrl = HOTRC;  // Use HotRC controller to drive instead of joystick?
bool cruise_gesturing = false;  // Is cruise mode enabled by gesturing?  Otherwise by press of cruise button

int32_t brake_spid_ctrl_dir = FWD;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value
double brake_spid_initial_kp = 0.588;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
double brake_spid_initial_ki_hz = 0.193;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
double brake_spid_initial_kd_s = 0.252;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
double cruise_spid_initial_kp = 0.157;  // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
double cruise_spid_initial_ki_hz = 0.035;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
double cruise_spid_initial_kd_s = 0.044;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
int32_t cruise_spid_ctrl_dir = FWD;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value
double gas_spid_initial_kp = 0.064;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
double gas_spid_initial_ki_hz = 0.015;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
double gas_spid_initial_kd_s = 0.022;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
int32_t gas_spid_ctrl_dir = REV;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value
double ctrl_ema_alpha[2] = { 0.2, 0.1 };  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double pot_ema_alpha = 0.2;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double battery_ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double pressure_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double carspeed_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double engine_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double brake_pos_ema_alpha = 0.25;
int32_t pwm_pulse_min_us = 500;
int32_t pwm_pulse_center_us = 1500;
int32_t pwm_pulse_max_us = 2500;
enum ctrl_axes { HORZ, VERT };
enum ctrl_thresh { MIN, DB, MAX };
enum ctrl_edge { BOT, TOP };
int32_t ctrl_lims_adc[2][2][3] = { { { 3,  50, 4092 }, { 3,  75, 4092 } }, { { 9, 200, 4085 }, { 9, 200, 4085 } }, }; // [HOTRC, JOY] [HORZ, VERT], [MIN, DEADBAND, MAX] values as ADC counts
int32_t ctrl_db_adc[2][2];  // To store the top and bottom deadband values for each axis of selected controller
int32_t pressure_min_adc = 658;  // Brake pressure when brakes are effectively off. Sensor min = 0.5V, scaled by 3.3/4.5V is 0.36V of 3.3V (ADC count 0-4095). 230430 measured 658 adc (0.554V) = no brakes
int32_t pressure_max_adc = 2100;  // Highest possible pressure achievable by the actuator (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as chris can push (wimp)
int32_t pressure_margin_adc = 12;  // Margin of error when comparing brake pressure adc values (ADC count 0-4095)
int32_t pressure_spike_thresh_adc = 60;  // min pressure delta between two readings considered a spike to ignore (ADC count 0-4095)
int32_t pressure_lp_thresh_adc = 1200;   // max delta acceptable over three consecutive readings (ADC count 0-4095)
int32_t brake_hold_initial_adc = 1200;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
int32_t brake_hold_increment_adc = 25;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
bool panic_stop = false;
int32_t brake_panic_initial_adc = 1800;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
int32_t brake_panic_increment_adc = 50;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
Timer brakeIntervalTimer(500000);  // How much time between increasing brake force during auto-stop if car still moving?
int32_t brake_increment_interval_us = 500000;  // How often to apply increment during auto-stopping (in us)
int32_t brake_pos_nom_lim_retract_adc = 153;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
int32_t brake_pos_abs_min_retract_adc = 175;  // Retract value corresponding with the absolute minimum retract actuator is capable of. (ADC count 0-4095)
int32_t brake_pos_zeropoint_adc = 1500;  // ++ Brake position value corresponding to the point where fluid PSI hits zero (ADC count 0-4095)
int32_t brake_pos_park_adc = 1750;  // Best position to park the actuator out of the way so we can use the pedal (ADC count 0-4095)
int32_t brake_pos_nom_lim_extend_adc = 2500;  // Extend limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
int32_t brake_pos_abs_max_extend_adc = 3076;  // Extend value corresponding with the absolute max extension actuator is capable of. (ADC count 0-4095)
int32_t brake_pos_margin_adc = 10;  //
int32_t brake_pos_adc = brake_pos_park_adc;
int32_t brake_pos_filt_adc = brake_pos_adc;
int32_t gesture_flytimeout_us = 500000;  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
int32_t car_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the car is stopped (in us)
int32_t engine_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
int32_t engine_idle_rpm = 700;  // Min value for engine hz, corresponding to low idle (in rpm)
int32_t engine_max_rpm = 6000;  // Max possible engine rotation speed
int32_t engine_redline_rpm = 4000;  // Max value for engine_rpm, pedal to the metal (in rpm)
int32_t engine_margin_rpm = 15;  // Margin of error for checking engine rpm (in rpm)
int32_t engine_spike_thresh_rpm = 500;  // min pressure delta between two readings considered a spike to ignore (in rpm)
int32_t engine_lp_thresh_rpm = 1000;   // max delta acceptable over three consecutive readings (in rpm)
int32_t carspeed_spike_thresh_mmph = 1500;  // min pressure delta between two readings considered a spike to ignore (in milli-mph)
int32_t carspeed_lp_thresh_mmph = 3000;   // max delta acceptable over three consecutive readings (in milli-mph)
int32_t carspeed_idle_mmph = 4500;  // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)
int32_t carspeed_redline_mmph = 15000;  // What is our steady state speed at redline? Pulley rotation frequency (in milli-mph)
int32_t carspeed_max_mmph = 25000;  // What is max speed car can ever go
int32_t cruise_sw_timeout_us = 500000;  // how long do you have to hold down the cruise button to start cruise mode (in us)
int32_t gas_governor_percent = 95;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
int32_t gas_pulse_ccw_max_us = 2000;  // Servo ccw limit pulsewidth. Hotrc controller ch1/2 min(lt/br) = 1000us, center = 1500us, max(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
int32_t gas_pulse_cw_max_us = 1000;  // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
int32_t gas_pulse_redline_us = 1400;  // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
int32_t gas_pulse_idle_us = 1800;  // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
int32_t gas_pulse_park_slack_us = 30;  // Gas pulsewidth beyond gas_pulse_idle_us where to park the servo out of the way so we can drive manually (in us)
int32_t steer_pulse_right_max_us = 500;  // Smallest pulsewidth acceptable to jaguar
int32_t steer_pulse_left_max_us = 2500;  // Longest pulsewidth acceptable to jaguar
int32_t steer_pulse_right_us = 800;  // Steering pulsewidth corresponding to full-speed right steering (in us)
int32_t steer_pulse_stop_us = pwm_pulse_center_us;  // Steering pulsewidth corresponding to zero steering motor movement (in us)
int32_t steer_pulse_left_us = 2200;  // Steering pulsewidth corresponding to full-speed left steering (in us)
int32_t steer_safe_percent = 72;  // Sterring is slower at high speed. How strong is this effect 
int32_t default_pulse_margin_us = 22;  // Default margin of error for comparisons of pulsewidths (in us)
int32_t brake_pulse_retract_max_us = 500;  // Smallest pulsewidth acceptable to jaguar
int32_t brake_pulse_extend_max_us = 2500;  // Longest pulsewidth acceptable to jaguar
int32_t brake_pulse_retract_us = 650;  // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us)
int32_t brake_pulse_stop_us = pwm_pulse_center_us;  // Brake pulsewidth corresponding to center point where motor movement stops (in us)
int32_t brake_pulse_extend_us = 2350;  // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us)
int32_t brake_pulse_margin_us = 40; // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated 
int32_t default_margin_adc = 12;  // Default margin of error for comparisons of adc values (ADC count 0-4095)
int32_t battery_max_mv = 16000;  // The max vehicle voltage we can sense. Design resistor divider to match. Must exceed max V possible.
int32_t pid_period_us = 50000;    // time period between output updates. Reciprocal of pid frequency (in us)
int32_t pid_period_ms = pid_period_us/1000;
Timer pidTimer(pid_period_us);
int32_t motor_park_timeout_us = 3000000;  // If we can't park the motors faster than this, then give up.
int32_t pot_min_adc = 100;
int32_t pot_max_adc = adc_range_adc-100;

// Non-settable variables
//
int32_t dataset_page = LOCK;  // Which of the six 8-value dataset pages is currently displayed, and available to edit
int32_t dataset_page_last = dataset_page;
int32_t selected_value = 0;  // In the real time tuning UI, which of the editable values (0-7) is selected. -1 for none 
int32_t selected_value_last = 0;
int32_t steer_pulse_safe_us = 0;
double gas_spid_derivative_rpmperus = 0.0;
bool gas_spid_saturated = false;
double cruise_spid_derivative_mmphperus = 0.0;
bool cruise_spid_saturated = false;
double brake_spid_derivative_adcperus = 0.0;
bool brake_spid_saturated = false;
int32_t engine_rpm = 50;  // Current engine speed, raw as sensed (in rpm)
int32_t engine_filt_rpm = 50;  // Current engine speed, filtered (in rpm)
int32_t engine_last_rpm = 50;  // Engine speed from previous loop (in rpm)
int32_t engine_old_rpm = 50; // Engine speed from two loops back (in rpm)
int32_t engine_govern_rpm = map(gas_governor_percent, 0, 100, 0, engine_redline_rpm);  // Create an artificially reduced maximum for the engine speed
int32_t gas_pulse_govern_us = map(gas_governor_percent*(engine_redline_rpm-engine_idle_rpm)/engine_redline_rpm, 0, 100, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
int32_t carspeed_govern_mmph = map(gas_governor_percent, 0, 100, 0, carspeed_redline_mmph);  // Governor must scale the top vehicle speed proportionally
int32_t carspeed_mmph = 10;  // Current car speed, raw as sensed (in mmph)
int32_t carspeed_filt_mmph = 10;  // Current car speed, filtered (in mmph)
int32_t carspeed_last_mmph = 10;  // Car speed from previous loop (in mmph)
int32_t carspeed_old_mmph = 10;  // Car speed from two loops back (in mmph)
int32_t battery_adc = 0;
int32_t battery_mv = 10000;
int32_t battery_filt_mv = 10000;
int32_t pot_adc = 0;
int32_t pot_filt_adc = adc_midscale_adc;
enum raw_filt { RAW, FILT};
int32_t ctrl_pos_adc[2][2] = { { adc_midscale_adc, adc_midscale_adc }, { adc_midscale_adc, adc_midscale_adc} };  // [HORZ/VERT] [RAW/FILT]
int32_t steer_pulse_out_us = steer_pulse_stop_us;  // pid loop output to send to the actuator (steering)
int32_t brake_pulse_out_us = brake_pulse_stop_us;  // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
double d_brake_pulse_out_us = (double)brake_pulse_out_us;
int32_t brake_spid_pressure_error_adc = 0;
int32_t brake_spid_integral_adcus = 0;
int32_t brake_spid_pos_error_adc = 0;
int32_t pressure_adc = 0;
int32_t brake_spid_carspeed_delta_adc = 0;
int32_t pressure_target_adc = 0;  // Stores new setpoint to give to the pid loop (brake)
double d_pressure_target_adc = (double)pressure_target_adc;
int32_t pressure_filt_adc = 0;  // Stores new setpoint to give to the pid loop (brake)
double d_pressure_filt_adc = (double)pressure_filt_adc;
int32_t pressure_last_adc = adc_midscale_adc;  // Some pressure reading history for noise handling (-1)
int32_t pressure_old_adc  = adc_midscale_adc;  // Some pressure reading history for noise handling (-2)
int32_t engine_target_rpm = 0;  // Stores new setpoint to give to the pid loop (gas)
int32_t gas_spid_engine_error_rpm = 0;
int32_t gas_spid_engine_delta_rpm = 0;
int32_t gas_spid_integral_rpmus = 0;
int32_t gas_pulse_delta_us;
int32_t gas_pulse_out_us = gas_pulse_idle_us;  // pid loop output to send to the actuator (gas)
Timer sanityTimer(7000000);  // Allows code to fail in a sensible way after a delay if nothing is happening
Timer gestureFlyTimer(gesture_flytimeout_us);  // Used to keep track of time for gesturing for going in and out of fly/cruise modes
int32_t cruise_spid_carspeed_error_mmph = 0;
int32_t cruise_spid_integral_mmphus = 0;
bool cruise_adjusting = false;
int32_t cruise_spid_carspeed_delta_mmph = 0;  // 
int32_t carspeed_target_mmph = 0.0;  // Stores new setpoint to give to the pid loop (cruise) in milli-mph
int32_t gesture_progress = 0;  // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
bool ignition = LOW;
bool ignition_last = ignition;
bool disp_redraw_all = true;
bool basicmodesw = LOW;
bool cruise_sw = LOW;
bool cruise_sw_held = false;
bool shutdown_complete = true;  // Shutdown mode has completed its work and can stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;  // Indicates we should release the brake & gas so the pedals can be used manually without interference
bool joy_centered = false;
Timer motorParkTimer;
bool simulating = false;
bool simulating_last = false;
bool sim_halfass = true;  // Don't sim the joystick or encoder or tach
bool sim_joy = false;
bool sim_press = true;
bool sim_tach = true;
bool sim_speedo = true;
bool sim_brkpos = true;
bool sim_basicsw = true;
bool sim_ign = true;
bool sim_cruisesw = true;
char disp_draw_buffer[8];  // Used to convert integers to ascii for purposes of displaying on screen
char disp_draw_buffer2[8];  // Used to convert integers to ascii for purposes of displaying on screen
char disp_values[disp_lines][8];
int32_t disp_needles[disp_lines];
int32_t disp_targets[disp_lines];
int32_t disp_age_quanta[disp_lines];
Timer dispAgeTimer[disp_lines];  // int32_t disp_age_timer_us[disp_lines];
bool disp_bool_values[6];
char disp_bool_buffer;
bool selected_val_dirty = true;
bool dataset_page_dirty = true;
int32_t old_tach_time_us;
int32_t old_speedo_time_us;
Timer cruiseSwTimer;
Timer simTimer;
int32_t sim_edit_delta = 0;
int32_t sim_edit_delta_touch = 0;
int32_t sim_edit_delta_encoder = 0;
Timer touchPollTimer(35000);  // Timer for regular touchscreen sampling
Timer touchHoldTimer(1000000);  // For timing touch long presses
Timer touchAccelTimer(850000);  // Touch hold time per left shift (doubling) of touch_accel
bool touch_now_touched = false;  // Is a touch event in progress
int32_t touch_accel_exponent = 0;  // Will edit values by +/- 2^touch_accel_exponent per touch_period interval
int32_t touch_accel = 1 << touch_accel_exponent;  // Touch acceleration level, which increases the longer you hold. Each edit update chages value by this
int32_t touch_accel_exponent_max = 8;  // Never edit values faster than this. 2^8 = 256 change in value per update
bool touch_longpress_valid = true;
int32_t touch_fudge = 0;  // -8
Timer loopTimer(1000000);  // how long the previous main loop took to run (in us)
int32_t loop_period_us = 100000;
int32_t loop_freq_hz = 1;  // run loop real time frequency (in Hz)
int32_t loopno = 1;    
int32_t loopzero = 0;  
Timer heartbeatTimer(1000000);
int32_t heartbeat_state = 0;
int32_t heartbeat_ekg[4] = { 150000, 100000, 430000, 1100000 };
bool heartbeat_pulse = HIGH;
bool button_last = 0;
bool button_it = 0;
// int32_t pressure_min_psi = 0;  // Brake pressure when brakes are effectively off (psi 0-1000)
// int32_t pressure_max_psi = 500;  // Highest possible pressure achievable by the actuator (psi 0-1000)

// Testing magnet sensors
// int32_t tach_magnet_count = 0;  // remove this after tach sensor bench testing (all references)

// Volatile variables  - for variables set inside ISRs
//
enum encoder_inputs {A, B, SW};
// volatile int32_t int_count = 0;
// volatile int32_t* pwm[] = { &OCR5A, &OCR5B, &OCR5C }; // &OCR1A, &OCR1B, &OCR1C, &OCR3A, &OCR3B, &OCR3C, &OCR4A, &OCR4B, &OCR4C,   // Store the addresses of the PWM timer compare (duty-cycle) registers:
volatile int32_t encoder_bounce_danger = B;  // Which of the encoder A or B inputs is currently untrustworthy due to bouncing 
volatile int32_t encoder_delta = 0;  // Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 
volatile bool encoder_a_stable = true;  //  Stores the value of encoder A pin as read during B pin transition (where A is stable)
volatile int32_t encoder_spinrate_isr_us = 100000;  // Time elapsed between last two detents
volatile int32_t tach_delta_us = 0;
volatile int32_t speedo_delta_us = 0;
volatile int32_t hotrc_horz_pulse_us = 1500;
volatile int32_t hotrc_vert_pulse_us = 1500;
volatile bool hotrc_ch3_sw, hotrc_ch4_sw, hotrc_ch3_sw_event, hotrc_ch4_sw_event, hotrc_ch3_sw_last, hotrc_ch4_sw_last;

Timer encoderSpinspeedTimer;  // Used to figure out how fast we're spinning the knob.  OK to not be volatile?
Timer tachPulseTimer;  // OK to not be volatile?
Timer speedoPulseTimer;  // OK to not be volatile?
Timer hotrcPulseTimer;  // OK to not be volatile?

int32_t tach_delta_impossible_us = 6500;  // 6500 us corresponds to about 10000 rpm, which isn't possible. Use to reject retriggers
int32_t speedo_delta_impossible_us = 4500;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers

enum encodersw_presses { NONE, SHORT, LONG };
Timer encoderLongPressTimer(800000);  // Used to time long button presses
int32_t encoder_spinrate_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_spinrate_min_us = 2500;  // Will reject spins faster than this as an attempt to debounce behavior
int32_t encoder_spinrate_last_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_spinrate_old_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_edits_per_det = 1;  // How many edits per detent. How much change happens per rotation detent
bool encoder_sw = false;  // Remember whether switch is being pressed
int32_t encoder_sw_action = NONE;  // Flag for encoder handler to know an encoder switch action needs to be handled
bool encoder_timer_active = false;  // Flag to prevent re-handling long presses if the sw is just kept down
bool encoder_suppress_click = false;  // Flag to prevent a short click on switch release after successful long press
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

SPID brakeSPID(brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s, brake_spid_ctrl_dir, pid_period_ms);
SPID gasSPID(gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s, gas_spid_ctrl_dir, pid_period_ms);
SPID cruiseSPID(cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s, cruise_spid_ctrl_dir, pid_period_ms);

// Servo library lets us set pwm outputs given an on-time pulse width in us
static Servo steer_servo;
static Servo brake_servo;
static Servo gas_servo;
static Adafruit_NeoPixel strip(1, neopixel_pin, NEO_GRB + NEO_GRB + NEO_KHZ800);

// Interrupt service routines
//
void encoder_a_isr(void) {  // When A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
    if (encoder_bounce_danger != A) {  // Prevents taking action on any re-triggers after a valid trigger due to bouncing
        if (!encoder_a_stable) {  // Since A just transitioned, if a_stable is low, this is a rising edge = time to register a turn 
            encoder_delta += digitalRead(encoder_b_pin) ? -1 : 1;  // Create turn event to be handled later. If B=0, delta=-1 (CCW) turn decreases things
            encoder_counter += encoder_delta;  // Just used to aid in debugging this isr
            encoder_spinrate_isr_us = encoderSpinspeedTimer.elapsed();
            encoderSpinspeedTimer.reset();
        }
        encoder_bounce_danger = A;  // Set to reject A retriggers and enable B trigger
    }
}
void encoder_b_isr(void) {  // On B rising or falling edge, A should have stabilized by now, so don't ignore next A transition
    if (encoder_bounce_danger != B) {  // Prevents taking action on any re-triggers after a valid trigger due to bouncing
        encoder_a_stable = digitalRead(encoder_a_pin);  // Input A is stable by the time B changes, so read A value here
        
        encoder_bounce_danger = B;  // Set to reject B retriggers and enable A trigger
    }
}
// The tach and speed use a hall sensor being triggered by a passing magnet once per pulley turn. These ISRs call mycros()
// on every pulse to know the time since the previous pulse. I tested this on the bench up to about 750 mmph which is as 
// fast as I can move the magnet with my hand, and it works. It would be cleaner to just increment a counter here in the ISR
// then call mycros() in the main loop and compare with a timer to calculate mmph.
void tach_isr(void) {  // The tach and speedo isrs compare value returned from the mycros() function with the value from the last interrupt to determine period, to get frequency of the vehicle pulley rotations.
    int32_t temp_us = tachPulseTimer.elapsed();
    if (temp_us > tach_delta_impossible_us) {
        tach_delta_us = temp_us;    
        tachPulseTimer.reset();
    }
}
void speedo_isr(void) {  //  A better approach would be to read, reset, and restart a hardware interval timer in the isr.  Better still would be to regularly read a hardware binary counter clocked by the pin - no isr.
    int32_t temp_us = speedoPulseTimer.elapsed();
    if (temp_us > speedo_delta_impossible_us) {
        speedo_delta_us = temp_us;    
        speedoPulseTimer.reset();
    }
}
void hotrc_horz_isr(void) {  // Reads ranged PWM signal on an input pin to determine control position. This ISR sets timer for all hotrc isrs on hi-going edge
    if (digitalRead(hotrc_horz_pin)) hotrcPulseTimer.reset();
    else hotrc_horz_pulse_us = hotrcPulseTimer.elapsed();
}
void hotrc_vert_isr(void) {  // On falling edge, reads ranged PWM signal on an input pin to determine control position
    hotrc_vert_pulse_us = hotrcPulseTimer.elapsed();
}
void hotrc_ch3_isr(void) {  // Reads a binary switch encoded as PWM on an input pin to determine button toggle state
    hotrc_ch3_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch3 switch true if short pulse, otherwise false
    if (hotrc_ch3_sw != hotrc_ch3_sw_last) hotrc_ch3_sw_event = true;  // So a handler routine can be signaled
    hotrc_ch3_sw_last = hotrc_ch3_sw;
}
void hotrc_ch4_isr(void) {  // Reads PWM signal on an input pin to determine control position
    hotrc_ch4_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch4 switch true if short pulse, otherwise false
    if (hotrc_ch4_sw != hotrc_ch4_sw_last) hotrc_ch4_sw_event = true;  // So a handler routine can be signaled
    hotrc_ch4_sw_last = hotrc_ch4_sw;
}
uint32_t neopixel_wheel_counter = 0;
Timer neopixelTimer(100000);
uint32_t colorwheel(int32_t WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    if(WheelPos < 170) {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// ema : pass in a fresh raw value, the previously filtered value, and alpha factor, returns new filtered value
int32_t ema(int32_t raw_value, int32_t filt_value, double alpha) {
    return (int32_t)((alpha*(double)raw_value) + ((1-alpha)*(double)filt_value));
}
double ema(int32_t raw_value, double filt_value, double alpha) {
    return (int32_t)((alpha*(double)raw_value) + ((1-alpha)*filt_value));
}

// Functions to write to the screen efficiently
//
void draw_bargraph_base(int32_t corner_x, int32_t corner_y, int32_t width) {  // draws a horizontal bargraph scale.  124, y, 40
    tft.drawFastHLine(corner_x+disp_bargraph_squeeze, corner_y, width-disp_bargraph_squeeze*2, GRY1);
    for (int32_t offset=0; offset<=2; offset++) tft.drawFastVLine((corner_x+disp_bargraph_squeeze)+offset*(width/2 - disp_bargraph_squeeze), corner_y-1, 3, WHT);
}
void draw_needle_shape(int32_t pos_x, int32_t pos_y, int32_t color) {  // draws a cute little pointy needle
    tft.drawFastVLine(pos_x-1, pos_y, 2, color);
    tft.drawFastVLine(pos_x, pos_y, 4, color);
    tft.drawFastVLine(pos_x+1, pos_y, 2, color);
}
void draw_target_shape(int32_t pos_x, int32_t pos_y, int32_t t_color, int32_t r_color) {  // draws a cute little target symbol
    tft.drawFastVLine(pos_x, pos_y+6, 3, t_color);
    tft.drawFastHLine(pos_x-1, pos_y+9, 3, t_color);
}
void draw_bargraph_needle(int32_t n_pos_x, int32_t old_n_pos_x, int32_t pos_y, int32_t n_color) {  // draws a cute little pointy needle
    draw_needle_shape(old_n_pos_x, pos_y, BLK);
    draw_needle_shape(n_pos_x, pos_y, n_color);
}
void draw_bargraph_needle_target(int32_t n_pos_x, int32_t old_n_pos_x, int32_t t_pos_x, int32_t old_t_pos_x, int32_t pos_y, int32_t n_color, int32_t t_color, int32_t r_color) {  // draws a needle and target
    draw_needle_shape(old_n_pos_x, pos_y, BLK);
    draw_target_shape(old_t_pos_x, pos_y, BLK, BLK);
    draw_target_shape(t_pos_x, pos_y, t_color, r_color);
    draw_needle_shape(n_pos_x, pos_y, n_color);
}
void draw_string(int32_t x, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
    tft.setCursor(x, y);
    tft.setTextColor(bgcolor);
    tft.print(oldtext);  // Erase the old content
    tft.setCursor(x, y);
    tft.setTextColor(color);
    tft.print(text);  // Draw the new content
}
void draw_mmph(int32_t x, int32_t y, int32_t color) {  // This is my cheesy pixel-drawn "mmph" compressed horizontally to 3-char width
    tft.setCursor(x, y);
    tft.setTextColor(color);
    tft.print("m");
    tft.setCursor(x+4, y);
    tft.print("m");
    tft.drawPixel(x+3, y+2, color);
    tft.drawPixel(x+11, y+2, color);
    tft.drawPixel(x+11, y+6, color);
    tft.drawPixel(x+15, y+2, color);
    tft.drawFastVLine(x+10, y+2, 6, color);
    tft.drawFastVLine(x+12, y+3, 3, color);
    tft.drawFastVLine(x+14, y, 7, color);
    tft.drawFastVLine(x+16, y+3, 4, color);
}
void draw_string_units(int32_t x, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
    if (!strcmp(oldtext, "mmph")) {  // Handle "mmph" different than other units so all fit in 3-char width
        draw_mmph(x, y, bgcolor);
        oldtext = "";
    }
    if (!strcmp(text, "mmph")) draw_mmph(x, y, color);
    else draw_string(x, y, text, oldtext, color, bgcolor);
}
// draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
void draw_fixed(bool redraw_tuning_corner) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
    tft.setTextColor(GRY2);
    tft.setTextSize(1);    
    if (redraw_tuning_corner) tft.fillRect(10, 145, 154, 95, BLK); // tft.fillRect(0,145,167,95,BLK);  // Erase old dataset page area
    else {
        for (int32_t lineno=0; lineno < arraysize(telemetry); lineno++)  {  // Step thru lines of fixed telemetry data
            draw_string(12, (lineno+1)*disp_line_height_pix+disp_vshift_pix, telemetry[lineno], "", GRY2, BLK);
            draw_string_units(104, (lineno+1)*disp_line_height_pix+disp_vshift_pix, units[lineno], "", GRY2, BLK);
            draw_bargraph_base(124, (lineno+1)*disp_line_height_pix+disp_vshift_pix+7, disp_bargraph_width);
        }
    }
    for (int32_t lineno=0; lineno < arraysize(dataset_page_names[dataset_page]); lineno++)  {  // Step thru lines of dataset page data
        draw_string(12, (lineno+1+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][lineno], dataset_page_names[dataset_page_last][lineno], GRY2, BLK);
        draw_string_units(104, (lineno+1+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix, tuneunits[dataset_page][lineno], tuneunits[dataset_page_last][lineno], GRY2, BLK);
        draw_bargraph_base(124, (lineno+1+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix+7, disp_bargraph_width);
    }
}
// draw_dynamic  normally draws a given value on a given line (0-19) to the screen if it has changed since last draw.
void draw_dyn_pid(int32_t lineno, int32_t value, int32_t lowlim, int32_t hilim, int32_t target, int32_t modeflag) {
    if (modeflag == 0 || modeflag == 4) {  // Modeflag 0 draws a numeric value and a bargraph needle. Modeflag 4 also draws a target
        bool dirty = false;
        int32_t age_us = (int32_t)(( (double)(dispAgeTimer[lineno].elapsed()) / 2500000)); // Divide by us per color gradient quantum
        memset(disp_draw_buffer,0,strlen(disp_draw_buffer));
        itoa(value, disp_draw_buffer, 10);  // Modeflag 0 is for writing numeric values for variables in the active data column at a given line
        if ( strcmp(disp_values[lineno], disp_draw_buffer) || disp_redraw_all ) dirty = true;
        if (dirty) {  // If value differs, Erase old value and write new
            draw_string(66, lineno*disp_line_height_pix+disp_vshift_pix, disp_draw_buffer, disp_values[lineno], GRN, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            strcpy(disp_values[lineno], disp_draw_buffer);
            dispAgeTimer[lineno].reset();
            disp_age_quanta[lineno] = 0;
        }
        if ( lowlim != -1 && (dirty || modeflag == 4) )  {  // If value differs, Erase old value and write new
            int32_t corner_x = 124;
            int32_t corner_y = lineno*disp_line_height_pix+disp_vshift_pix-1;
            int32_t n_pos = map(value, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
            int32_t ncolor = (n_pos > disp_bargraph_width-disp_bargraph_squeeze || n_pos < disp_bargraph_squeeze) ? RED : GRN;
            n_pos = corner_x + constrain(n_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
            draw_bargraph_needle(n_pos, disp_needles[lineno], corner_y, ncolor);  // Let's draw a needle
            if (modeflag == 4) {
                int32_t t_pos = map(target, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                int32_t tcolor = (t_pos > disp_bargraph_width-disp_bargraph_squeeze || n_pos < disp_bargraph_squeeze) ? RED : ( (t_pos != n_pos) ? YEL : GRN );
                t_pos = corner_x + constrain(t_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                // int32_t rcolor = (t_pos != n_pos) ? RED : GRN;
                draw_target_shape(disp_targets[lineno], corner_y, BLK, -1);
                draw_bargraph_base(124, (lineno)*disp_line_height_pix+disp_vshift_pix+7, disp_bargraph_width);
                draw_target_shape(t_pos, corner_y, tcolor, -1);
                // draw_bargraph_needle_target(n_pos, disp_needles[lineno], t_pos, disp_targets[lineno], corner_y, ncolor, tcolor, rcolor);  // draws a needle and target
                disp_targets[lineno] = t_pos;
            }
            disp_needles[lineno] = n_pos;
        }
        else if (age_us > disp_age_quanta[lineno] && age_us < 11)  {  // As readings age, redraw in new color
            int32_t color;
            if (age_us < 8) color = 0x1fe0 + age_us*0x2000;  // Base of green with red added as you age
            else color = 0xffe0 - (age_us-8)*0x100;  // Until yellow is achieved, then lose green as you age further
            draw_string(66, (lineno)*disp_line_height_pix+disp_vshift_pix, disp_values[lineno], "", color, BLK);
           disp_age_quanta[lineno] = age_us;
        } // Else don't draw anything, because we already did.  Logic is 100s of times cheaper than screen drawing.)
    }
    else if (modeflag == 1 && (strcmp(disp_values[lineno], modecard[runmode]) || disp_redraw_all)) {  // Modeflag 1 is for drawing the runmode in the upper left corner
        draw_string(11+6, lineno*disp_line_height_pix+disp_vshift_pix, disp_values[lineno], "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        draw_string(11+6*(2+strlen(disp_values[lineno])), lineno*disp_line_height_pix+disp_vshift_pix, "Mode", "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        draw_string(11+6, lineno*disp_line_height_pix+disp_vshift_pix, modecard[runmode], "", colorcard[runmode], BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        draw_string(11+6*(2+strlen(modecard[runmode])), lineno*disp_line_height_pix+disp_vshift_pix, "Mode", "", colorcard[runmode], BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        strcpy(disp_values[lineno], modecard[runmode]);
    }
    else if (modeflag == 2 && dataset_page != dataset_page_last) {  // Modeflag 2 is used for displaying which set of tuning variables is being displayed. Text next to the runmode
        draw_string(122, disp_vshift_pix, pagecard[dataset_page], pagecard[dataset_page_last], CYN, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
    }
    else if (modeflag == 3) {  // Modeflag 3 is for highlighting a variable name when its value may be changed
        int32_t color = GRY2;
        if (tuning_ctrl == SELECT) color = YEL;
        else if (tuning_ctrl == EDIT) color = GRN;
        if (tuning_ctrl == SELECT && selected_value != selected_value_last) {
            draw_string(12, 12+(selected_value+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_value], "", color, BLK);
            draw_string(12, 12+(selected_value_last+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_value_last], "", GRY2, BLK);
        }
        else if (tuning_ctrl != tuning_ctrl_last) {
            draw_string(12, 12+(selected_value+arraysize(telemetry))*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_value], "", color, BLK);
        }
    }
}
void draw_dynamic(int32_t lineno, int32_t value, int32_t lowlim, int32_t hilim, int32_t modeflag) {
    if (modeflag != 4) draw_dyn_pid(lineno, value, lowlim, hilim, -1, modeflag);
}
void draw_bool(bool value, int32_t col) {  // Draws values of boolean data
    if ((disp_bool_values[col] != value) || disp_redraw_all) {  // If value differs, Erase old value and write new
        draw_string(touch_margin_h_pix + touch_cell_h_pix*(col+3) + (touch_cell_h_pix>>1) - arraysize(top_menu_buttons[col]-1)*(disp_font_width>>1) - 2, 0, top_menu_buttons[col], "", (value) ? GRN : LGRY, DGRY);
        disp_bool_values[col] = value;
    }
}
void draw_simbuttons(bool create) {  // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
    tft.setTextColor(LYEL);
    for (int32_t row = 0; row < arraysize(simgrid); row++) {
        for (int32_t col = 0; col < arraysize(simgrid[row]); col++) {
            int32_t cntr_x = touch_margin_h_pix + touch_cell_h_pix*(col+3) + (touch_cell_h_pix>>1) +2;
            int32_t cntr_y = touch_cell_v_pix*(row+1) + (touch_cell_v_pix>>1);
            if ( strcmp( simgrid[row][col], "    " ) ) {
                tft.fillCircle(cntr_x, cntr_y, 19, create ? DGRY : BLK);
                if (create) {
                    tft.drawCircle(cntr_x, cntr_y, 19, LYEL);
                    draw_string(cntr_x-(arraysize(simgrid[row][col])-1)*(disp_font_width>>1), cntr_y-(disp_font_height>>1), simgrid[row][col], "", LYEL, DGRY);
                }
            }
        }     
    }
}
void draw_touchgrid(bool replace_names) {  // draws edge buttons with names in 'em. If replace_names, just updates names
    int32_t namelen = 0;
    if (replace_names) strcpy(side_menu_buttons[0], pagecard[dataset_page]);
    tft.setTextColor(WHT);
    for (int32_t row = 0; row < ((replace_names) ? 1 : arraysize(side_menu_buttons)); row++) {  // Step thru all rows to draw buttons along the left edge
        tft.fillRoundRect(-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, DGRY);
        tft.drawRoundRect(-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, LYEL);
        namelen = 0;
        for (uint32_t x = 0 ; x < arraysize(side_menu_buttons[row]) ; x++ ) {
            if (side_menu_buttons[row][x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
        }
        for (int32_t letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
            tft.setCursor( 1, ( touch_cell_v_pix*row) + (touch_cell_v_pix/2) - (int32_t)(4.5*((double)namelen-1)) + (disp_font_height+1)*letter ); // adjusts vertical offset depending how many letters in the button name and which letter we're on
            tft.println( side_menu_buttons[row][letter] );  // Writes each letter such that the whole name is centered vertically on the button
        }
    }
    for (int32_t btn = 0; btn < 3; btn++) {  // Step thru all cols to draw buttons across the top edge
        tft.fillRoundRect(touch_margin_h_pix + touch_cell_h_pix*(btn+3) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
        tft.drawRoundRect(touch_margin_h_pix + touch_cell_h_pix*(btn+3) + 3, -9, touch_cell_h_pix-6, 18, 8, LYEL);  // tft.width()-9, 3, 18, (tft.height()/5)-6, 8, LYEL);
        // draw_bool(top_menu_buttons[btn], btn+3);
    }
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

// int* x is c++ style, int *x is c style
void adj_val (int32_t* variable, int32_t modify, int32_t low_limit, int32_t high_limit) {
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
}
void set_pin (int32_t pin, int32_t mode) {
    if (pin >= 0) pinMode (pin, mode);
}
void write_pin (int32_t pin, int32_t val) {
    if (pin >= 0) digitalWrite (pin, val);
}

void setup() {
    set_pin(heartbeat_led_pin, OUTPUT);
    set_pin(encoder_a_pin, INPUT_PULLUP);
    set_pin(encoder_b_pin, INPUT_PULLUP);
    set_pin(brake_pwm_pin, OUTPUT);
    set_pin(steer_pwm_pin, OUTPUT);
    set_pin(tft_dc_pin, OUTPUT);
    set_pin(encoder_sw_pin, INPUT_PULLUP);
    set_pin(gas_pwm_pin, OUTPUT);
    set_pin(ignition_pin, OUTPUT);  // drives relay to turn on/off car. Active high
    set_pin(basicmodesw_pin, INPUT_PULLUP);
    set_pin(tach_pulse_pin, INPUT_PULLUP);
    set_pin(speedo_pulse_pin, INPUT_PULLUP);
    set_pin(joy_horz_pin, INPUT);
    set_pin(joy_vert_pin, INPUT);
    set_pin(pressure_pin, INPUT);
    set_pin(brake_pos_pin, INPUT);
    set_pin(battery_pin, INPUT);
    set_pin(hotrc_horz_pin, INPUT);
    set_pin(hotrc_vert_pin, INPUT);
    set_pin(hotrc_ch3_pin, INPUT);
    set_pin(hotrc_ch4_pin, INPUT);
    set_pin(neopixel_pin, OUTPUT);
    set_pin(usd_cs_pin, OUTPUT);
    set_pin(tft_cs_pin, OUTPUT);
    set_pin(pot_wipe_pin, INPUT);
    set_pin(button_pin, INPUT_PULLUP);    
    set_pin(pot_pwr_pin, OUTPUT);
    set_pin(pot_pwr_pin, OUTPUT);
    set_pin(cruise_sw_pin, INPUT_PULLUP);
    set_pin(tp_irq_pin, INPUT_PULLUP);
    set_pin(led_rx_pin, OUTPUT);
    set_pin(led_tx_pin, OUTPUT);
    // if ((int32_t)tft_ledk_pin >= 0) set_pin(tft_ledk_pin, OUTPUT);
    
    write_pin(ignition_pin, ignition);
    write_pin(tft_cs_pin, HIGH);   // Prevent bus contention
    write_pin(usd_cs_pin, HIGH);   // Prevent bus contention
    write_pin(tft_dc_pin, LOW);
    write_pin(led_rx_pin, LOW);  // Light up
    write_pin(led_tx_pin, HIGH);  // Off
    write_pin(pot_pwr_pin, HIGH);

    analogReadResolution(adc_bits);  // Set Arduino Due to 12-bit resolution (default is same as Mega=10bit)
    Serial.begin(115200);  // Open serial port
    // printf("Serial port open\n");  // This works on Due but not ESP32
    
    delay(500); // This is needed to allow the screen board enough time after a cold boot before we start trying to talk to it.
    if (display_enabled) {
        Serial.print(F("Init LCD... "));
        tft.begin();
        tft.setRotation(1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
        for (int32_t lineno=0; lineno <= arraysize(telemetry); lineno++)  {
            disp_age_quanta[lineno] = -1;
            memset(disp_values[lineno],0,strlen(disp_values[lineno]));
        }
        for (int32_t row=0; row<arraysize(disp_bool_values); row++) disp_bool_values[row] = 1;
        for (int32_t row=0; row<arraysize(disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
        for (int32_t row=0; row<arraysize(disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen

        tft.fillScreen(BLK);  // Black out the whole screen
        draw_fixed(false);
        draw_touchgrid(false);
        Serial.println(F("Success"));

        Serial.print(F("Captouch initialization... "));
        if (! touchpanel.begin(40)) {     // pass in 'sensitivity' coefficient
            Serial.println(F("Couldn't start FT6206 touchscreen controller"));
            // while (1);
        }
        else Serial.println(F("Capacitive touchscreen started"));
    }
    strip.begin();  // start datastream
    strip.show();  // Turn off the pixel
    strip.setBrightness(50);  // It truly is incredibly bright
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
    attachInterrupt(digitalPinToInterrupt(hotrc_horz_pin), hotrc_horz_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hotrc_vert_pin), hotrc_vert_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(hotrc_ch3_pin), hotrc_ch3_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(hotrc_ch4_pin), hotrc_ch4_isr, FALLING);
    
    Serial.println(F("set up and enabled\n"));
    
    // Set up the soren pid loops
    brakeSPID.set_output_center(brake_pulse_stop_us);  // Sets actuator centerpoint and puts pid loop in output centerpoint mode. Becasue actuator value is defined as a deviation from a centerpoint
    brakeSPID.set_input_limits((double)pressure_min_adc, (double)pressure_max_adc);  // Make sure pressure target is in range
    brakeSPID.set_output_limits((double)brake_pulse_retract_us , (double)brake_pulse_extend_us);
    gasSPID.set_input_limits((double)engine_idle_rpm, (double)engine_govern_rpm);
    gasSPID.set_output_limits((double)gas_pulse_redline_us, (double)gas_pulse_idle_us);
    cruiseSPID.set_input_limits((double)carspeed_idle_mmph, (double)carspeed_redline_mmph);
    cruiseSPID.set_output_limits((double)engine_idle_rpm, (double)engine_govern_rpm);
      
    steer_servo.attach(steer_pwm_pin);
    brake_servo.attach(brake_pwm_pin);
    gas_servo.attach(gas_pwm_pin);

    loopTimer.reset();  // start timer to measure the first loop
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
    if (button_it != button_last) Serial.println("button");
    button_last = button_it;
    if (serial_debugging && print_timestamps) {
        Serial.print("Loop# ");
        Serial.print(loopno);  Serial.print(": ");      
        loopzero = mycros();  // Start time for loop
    }
    // Update derived variable values in case they have changed
    ctrl_db_adc[VERT][BOT] = (adc_range_adc-ctrl_lims_adc[ctrl][VERT][DB])/2;  // Lower threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[VERT][TOP] = (adc_range_adc+ctrl_lims_adc[ctrl][VERT][DB])/2;  // Upper threshold of vert joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][BOT] = (adc_range_adc-ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Lower threshold of horz joy deadband (ADC count 0-4095)
    ctrl_db_adc[HORZ][TOP] = (adc_range_adc+ctrl_lims_adc[ctrl][HORZ][DB])/2;  // Upper threshold of horz joy deadband (ADC count 0-4095)
    engine_govern_rpm = map(gas_governor_percent, 0, 100, 0, engine_redline_rpm);  // Create an artificially reduced maximum for the engine speed
    gas_pulse_govern_us = map(gas_governor_percent*(engine_redline_rpm-engine_idle_rpm)/engine_redline_rpm, 0, 100, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
    carspeed_govern_mmph = map(gas_governor_percent, 0, 100, 0, carspeed_redline_mmph);  // Governor must scale the top vehicle speed proportionally

    if (serial_debugging && print_timestamps) printf ("%ld ", mycros()-loopzero);
    
    if (heartbeat_led_pin >= 0 && heartbeatTimer.expired()) {  // Heartbeat LED
        heartbeat_pulse = !heartbeat_pulse;
        if (++heartbeat_state >= arraysize(heartbeat_ekg)) heartbeat_state -= arraysize(heartbeat_ekg);
        heartbeatTimer.set(heartbeat_ekg[heartbeat_state]);
        digitalWrite(heartbeat_led_pin, heartbeat_pulse);
    }
    if (neopixel_pin >= 0 && neopixelTimer.expired()) {
        neopixel_wheel_counter++;
        strip.setPixelColor(0, colorwheel(neopixel_wheel_counter));
        neopixelTimer.reset();
    }
    // Encoder
    //
    // Read and interpret encoder switch activity. Encoder rotation is handled in interrupt routine
    // Encoder handler routines should act whenever encoder_sw_action is true, setting it back to false once handled.
    // When handling press, if encoder_long_clicked is nonzero then press is a long press
    if (!digitalRead(encoder_sw_pin)) {  // if encoder sw is being pressed (switch is active low)
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

    // 1) Gather new telemetry and filter the signals
    //  
    pot_adc = 0;
    #ifndef ESP32
        pot_adc = analogRead(pot_wipe_pin);  // Potentiometer
    #endif
    pot_filt_adc = ema(pot_adc, pot_filt_adc, pot_ema_alpha);
    
    // Voltage of vehicle battery
    battery_adc = analogRead(battery_pin);
    battery_mv = (int32_t)(battery_max_mv*((double)battery_adc)/adc_range_adc);  // convert adc value read into mV    
    battery_filt_mv = ema(battery_mv, battery_filt_mv, battery_ema_alpha);  // Apply EMA filter
    
    // Read sensors
    if (!simulating && !sim_brkpos) {
        brake_pos_adc = analogRead(brake_pos_pin);
        brake_pos_filt_adc = ema(brake_pos_adc, brake_pos_filt_adc, brake_pos_ema_alpha);
    }
    else brake_pos_filt_adc = (brake_pos_nom_lim_retract_adc + brake_pos_zeropoint_adc)/2;  // To keep brake position in legal range during simulation
    // Tach
    if (!simulating || !sim_tach) {
        if (tachPulseTimer.elapsed() < engine_stop_timeout_us) engine_rpm = (int32_t)(60000000/(double)tach_delta_us);  // Tachometer magnets/us * 60000000 (1 rot/magnet * 1000000 us/sec * 60 sec/min) gives rpm
        else engine_rpm = 0;  // If timeout since last magnet is exceeded
        if (abs(engine_rpm-engine_old_rpm) > engine_lp_thresh_rpm || engine_rpm-engine_last_rpm < engine_spike_thresh_rpm) {  // Remove noise spikes from tach values, if otherwise in range
            engine_old_rpm = engine_last_rpm;
            engine_last_rpm = engine_rpm;
        }
        else engine_rpm = engine_last_rpm;  // Spike detected - ignore that sample
        if (engine_rpm) engine_filt_rpm = ema(engine_rpm, engine_filt_rpm, engine_ema_alpha);  // Sensor EMA filter
        else engine_filt_rpm = 0;
    }
    // Speedo
    if (!simulating || !sim_speedo) { 
        if (speedoPulseTimer.elapsed() < car_stop_timeout_us) carspeed_mmph = (int32_t)(179757270/(double)speedo_delta_us); // Update car speed value  
        // magnets/us * 179757270 (1 rot/magnet * 1000000 us/sec * 3600 sec/hr * 1/19.85 gearing * 20*pi in/rot * 1/12 ft/in * 1000/5280 milli-mi/ft gives milli-mph  // * 1/1.15 knots/mph gives milliknots
        // Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
        else carspeed_mmph = 0;     
        if (abs(carspeed_mmph-carspeed_old_mmph) > carspeed_lp_thresh_mmph || carspeed_mmph-carspeed_last_mmph < carspeed_spike_thresh_mmph) {  // Remove noise spikes from speedo values, if otherwise in range
            carspeed_old_mmph = carspeed_last_mmph;
            carspeed_last_mmph = carspeed_mmph;
        }
        else carspeed_mmph = carspeed_last_mmph;  // Spike detected - ignore that sample
        if (carspeed_mmph)  carspeed_filt_mmph = ema(carspeed_mmph, carspeed_filt_mmph, carspeed_ema_alpha);  // Sensor EMA filter
        else carspeed_filt_mmph = 0;
    }

    if (!simulating || !sim_press) {
        pressure_adc = analogRead(pressure_pin);  // Brake pressure.  Read sensor, then Remove noise spikes from brake feedback, if reading is otherwise in range
        if (abs(pressure_adc-pressure_old_adc) > pressure_lp_thresh_adc || pressure_adc-pressure_last_adc < pressure_spike_thresh_adc) {
            pressure_old_adc = pressure_last_adc;
            pressure_last_adc = pressure_adc;
        }
        else pressure_adc = pressure_last_adc;  // Spike detected - ignore that sample
        // pressure_psi = (int32_t)(1000*(double)(pressure_adc)/adc_range_adc);      // Convert pressure to units of psi
        pressure_filt_adc = ema(pressure_adc, pressure_filt_adc, pressure_ema_alpha);  // Sensor EMA filter
    }

   // 2) Read joystick then determine new steering setpoint, and handle digital pins
    //
    if (led_tx_pin >= 0) digitalWrite(led_tx_pin, !touch_now_touched);
    if (led_rx_pin >= 0) digitalWrite(led_rx_pin, (sim_edit_delta > 0) ? 0 : 1);  
    if (!simulating || !sim_joy) {  // If not simulating or not simulating joystick
        if (ctrl == HOTRC) {
            ctrl_pos_adc[VERT][RAW] = map(hotrc_vert_pulse_us, 2003, 1009, ctrl_lims_adc[ctrl][VERT][MAX], ctrl_lims_adc[ctrl][VERT][MIN]);
            ctrl_pos_adc[HORZ][RAW] = map(hotrc_horz_pulse_us, 2003, 1009, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
            ctrl_pos_adc[VERT][RAW] = constrain(ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
            ctrl_pos_adc[HORZ][RAW] = constrain(ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);   
        }
        else {
            ctrl_pos_adc[VERT][RAW] = analogRead(joy_vert_pin);  // Read joy vertical
            ctrl_pos_adc[HORZ][RAW] = analogRead(joy_horz_pin);  // Read joy horizontal
        }        
        if (ctrl_pos_adc[VERT][RAW] > ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][RAW] < ctrl_db_adc[VERT][TOP])  ctrl_pos_adc[VERT][FILT] = adc_midscale_adc;  // if joy vert is in the deadband, set joy_vert_filt to center value
        else ctrl_pos_adc[VERT][FILT] = ema(ctrl_pos_adc[VERT][RAW], ctrl_pos_adc[VERT][FILT], ctrl_ema_alpha[ctrl]);  // otherwise do ema filter to determine joy_vert_filt
        if (ctrl_pos_adc[HORZ][RAW] > ctrl_db_adc[HORZ][BOT] && ctrl_pos_adc[HORZ][RAW] < ctrl_db_adc[HORZ][TOP])  ctrl_pos_adc[HORZ][FILT] = adc_midscale_adc;  // if joy horz is in the deadband, set joy_horz_filt to center value
        else ctrl_pos_adc[HORZ][FILT] = ema(ctrl_pos_adc[HORZ][RAW], ctrl_pos_adc[HORZ][FILT], ctrl_ema_alpha[ctrl]);  // otherwise do ema filter to determine joy_horz_filt
    }
    if (!(runmode == SHUTDOWN && (!carspeed_filt_mmph || shutdown_complete)))  { // If not in shutdown mode with shutdown complete and car stopped
        if (ctrl_pos_adc[HORZ][FILT] >= ctrl_db_adc[HORZ][TOP]) {
            steer_pulse_safe_us = steer_pulse_stop_us + (int32_t)( (double)(steer_pulse_right_us - steer_pulse_stop_us) * (1 - ( (double)steer_safe_percent * carspeed_filt_mmph / ((double)carspeed_redline_mmph * 100) ) ) );
            steer_pulse_out_us = map(ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][TOP], ctrl_lims_adc[ctrl][HORZ][MAX], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the right of deadband
        }
        else if (ctrl_pos_adc[HORZ][FILT] <= ctrl_db_adc[HORZ][BOT]) {
            steer_pulse_safe_us = steer_pulse_stop_us - (int32_t)( (double)(steer_pulse_stop_us - steer_pulse_left_us) * (1 - ( (double)steer_safe_percent * carspeed_filt_mmph / ((double)carspeed_redline_mmph * 100) ) ) );
            steer_pulse_out_us = map(ctrl_pos_adc[HORZ][FILT], ctrl_db_adc[HORZ][BOT], ctrl_lims_adc[ctrl][HORZ][MIN], steer_pulse_stop_us, steer_pulse_safe_us);  // Figure out the steering setpoint if joy to the left of deadband
        }
        else steer_pulse_out_us = steer_pulse_stop_us;  // Stop the steering motor if inside the deadband
    }
    if (ctrl == HOTRC && hotrc_ch3_sw_event) {  // Turn on/off the vehicle ignition
        ignition = !ignition;
        hotrc_ch3_sw_event = false;
    }
    if (ctrl == HOTRC && hotrc_ch4_sw_event) {  // Toggle cruise/fly mode 
        if (runmode == FLY) runmode = CRUISE;
        else if (runmode == CRUISE) runmode = FLY;
        hotrc_ch4_sw_event = false;    
    }
    if (!simulating || !sim_basicsw) basicmodesw = !digitalRead(basicmodesw_pin);   // 1-value because electrical signal is active low
    if (!simulating || !sim_cruisesw) cruise_sw = digitalRead(cruise_sw_pin);
    
    if (serial_debugging && print_timestamps) printf ("%ld ", mycros()-loopzero);

    // 3) Check if our current runmode has been overridden by certain specific conditions
    //
    if (basicmodesw) runmode = BASIC;    // if basicmode switch on --> Basic Mode
    else if (!ignition) runmode = SHUTDOWN;  //} && laboratory != true)  {  // otherwise if ignition off --> Shutdown Mode
    else if (!engine_filt_rpm)  runmode = STALL;    // otherwise if engine not running --> Stall Mode
    
    // 4) Do actions based on which runmode we are in (and set gas/brake setpoints), and possibly change runmode 
    //
    if (runmode == BASIC)  {  // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering stell works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        if ((!basicmodesw) && engine_filt_rpm)  runmode = HOLD;  // If we turned off the basic mode switch with engine running, go to Hold mode. If engine is not running, we'll end up in Stall Mode automatically
    }
    else if (runmode == SHUTDOWN)  { // In shutdown mode we stop the car if it's moving then park the motors.
        if (ignition) runmode = STALL;
        else if (we_just_switched_modes)  {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            engine_target_rpm = engine_idle_rpm;  //  Release the throttle 
            shutdown_complete = false;
            if (carspeed_filt_mmph)  {
                pressure_target_adc = (panic_stop) ? brake_panic_initial_adc : brake_hold_initial_adc;  // More brakes, etc. to stop the car
                brakeIntervalTimer.reset();
                sanityTimer.reset();
            }
        }
        if (!shutdown_complete)  {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (!carspeed_filt_mmph || sanityTimer.expired())  {  // If car has stopped, or timeout expires, then release the brake
                motorParkTimer.reset();  // Set a timer to timebox this effort
                park_the_motors = true;  // Flags the motor parking to happen
                if (pressure_filt_adc <= pressure_min_adc + pressure_margin_adc)  shutdown_complete = true;  // With this set, we will do nothing from here on out (until mode changes, i.e. ignition)
            }
            else if (brakeIntervalTimer.expired())  {
                pressure_target_adc += (panic_stop) ? brake_panic_increment_adc : brake_hold_increment_adc;  // Slowly add more brakes until car stops
                brakeIntervalTimer.reset();  
            }
            else if (!park_the_motors) shutdown_complete = true;
        }
    }
    else if (runmode == STALL)  {   // In stall mode, the gas doesn't have feedback
        if (engine_filt_rpm)  runmode = HOLD;  //  Enter Hold Mode if we started the car
        else {  // Actuators still respond and everything, even tho engine is turned off
            pressure_target_adc = pressure_min_adc;  // Default when joystick not pressed
            gas_pulse_out_us = gas_pulse_idle_us;  // Default when joystick not pressed
            if (ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][TOP])  { //  If we are pushing up
                // In stall mode there is no engine rpm for PID to use as feedback, so we bypass the PID and just set the engine_target_angle proportional to 
                // the joystick position.  This works whether there is normally a gas PID or not.
                gas_pulse_out_us = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], gas_pulse_idle_us, gas_pulse_govern_us);
            }
            else if (ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT])  {  // If we are pushing down
                pressure_target_adc = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][BOT], ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_adc, pressure_max_adc);  // Scale joystick value to pressure adc setpoint
            }
        }
    }
    else if (runmode == HOLD)  {
        if (we_just_switched_modes)  {  // Release throttle and push brake upon entering hold mode
            engine_target_rpm = engine_idle_rpm;  // Let off gas (if gas using PID mode)
            if (!carspeed_filt_mmph)  pressure_target_adc += brake_hold_increment_adc; // If the car is already stopped then just add a touch more pressure and then hold it.
            else pressure_target_adc = brake_hold_initial_adc;  //  Otherwise, these hippies need us to stop the car for them
            brakeIntervalTimer.reset();
            joy_centered = false;
        }
        else if (carspeed_filt_mmph && brakeIntervalTimer.expired())  { // Each interval the car is still moving, push harder
            pressure_target_adc += brake_hold_increment_adc;  // Slowly add more brakes until car stops
            brakeIntervalTimer.reset();
        }
        pressure_target_adc = constrain(pressure_target_adc, pressure_min_adc, pressure_max_adc);  // Just make sure we don't try to push harder than we can 
        if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) {
            joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        }
        if (joy_centered && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][TOP]) { 
            runmode = FLY; // Enter Fly Mode upon joystick movement from center to above center
        }
    }
    else if (runmode == FLY)  {
        if (we_just_switched_modes)  {
            gesture_progress = 0;
            gestureFlyTimer.reset(); // Initialize gesture timer to already-expired value
            cruise_sw_held = false;
            cruiseSwTimer.reset();
            hotrc_ch4_sw_event = false;
        }
        if (!carspeed_filt_mmph && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT])  runmode = HOLD;  // Go to Hold Mode if we have braked to a stop
        else  {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            engine_target_rpm = engine_idle_rpm;  // Default when joystick not pressed 
            pressure_target_adc = pressure_min_adc;  // Default when joystick not pressed   
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP])  {  // If we are trying to accelerate
                engine_target_rpm = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], engine_idle_rpm, engine_govern_rpm);
            }
            else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT])  {  // If we are trying to brake, scale joystick value to determine pressure adc setpoint
                pressure_target_adc = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][BOT], ctrl_lims_adc[ctrl][VERT][MIN], pressure_min_adc, pressure_max_adc);
            }
        }
        // Cruise mode can be entered by pressing a physical momentary button, or by holding the brake on full for a half second. Which epends on the cruise_gesturing flag.
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (cruise_gesturing) {  // If we are configured to use joystick gestures to go to cruise mode, the gesture is 
            if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP])  { // Re-zero gesture timer for potential new gesture whenever joystick at center
                gestureFlyTimer.reset();
            }
            if (gestureFlyTimer.expired()) gesture_progress = 0; // If gesture timeout has expired, cancel any in-progress gesture
            else {  // Otherwise check for successful gesture motions
                if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_lims_adc[ctrl][VERT][MAX]-default_margin_adc)  { // If joystick quickly pushed to top, step 1 of gesture is successful
                    gesture_progress++;
                    gestureFlyTimer.reset();
                }
                else if (gesture_progress == 1 && ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN]+default_margin_adc)  { // If joystick then quickly pushed to bottom, step 2 succeeds
                    gesture_progress++;
                    gestureFlyTimer.reset();
                }
                else if (gesture_progress == 2 && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP]) { // If joystick then quickly returned to center, go to Cruise mode
                    runmode = CRUISE;
                }        
            }
        }
        if (!cruise_sw) {  // If button not currently pressed
            if (cruise_sw_held && cruiseSwTimer.expired())  runmode = CRUISE;  // If button was just held long enough, upon release enter Cruise mode
            cruise_sw_held = false;  // Cancel button held state
        }
        else if (!cruise_sw_held) {  // If button is being pressed, but we aren't in button held state
            cruiseSwTimer.reset(); // Start hold time timer
            cruise_sw_held = true;  // Get into that state
        }
    }
    else if (runmode == CRUISE)  {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            carspeed_target_mmph = carspeed_filt_mmph;  // Begin cruising with cruise set to current speed
            pressure_target_adc = pressure_min_adc;  // Let off the brake and keep it there till out of Cruise mode
            gestureFlyTimer.reset();  // reset gesture timer
            cruise_sw_held = false;
            hotrc_ch4_sw_event = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) {  // When joystick vert above center, increase the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            engine_target_rpm = map(ctrl_pos_adc[VERT][FILT], ctrl_db_adc[VERT][TOP], ctrl_lims_adc[ctrl][VERT][MAX], engine_filt_rpm, engine_govern_rpm);
        }
        else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) {  // When joystick vert below center, decrease the throttle target proportional to how far off center
            cruise_adjusting = true;  // Suspend pid loop control of gas
            engine_target_rpm = map(ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_db_adc[VERT][BOT], engine_idle_rpm, engine_filt_rpm);
        }
        else cruise_adjusting = false;  // if joystick at center
        if (cruise_adjusting) {
            carspeed_target_mmph = carspeed_filt_mmph;  // Upon return to center set speed target to current speed
            cruiseSPID.set_target((double)carspeed_target_mmph);
        }
        // This old gesture trigger drops to Fly mode if joystick moved quickly from center to bottom
        // if (ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN]+default_margin_adc && abs(mycros() - gesture_timer_us) < gesture_flytimeout_us)  runmode = FLY;  // If joystick quickly pushed to bottom 
        // printf("hotvpuls=%ld, hothpuls=%ld, joyvfilt=%ld, joyvmin+marg=%ld, timer=%ld\n", hotrc_vert_pulse_us, hotrc_horz_pulse_us, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc, gesture_timer_us);
        if (ctrl_pos_adc[VERT][RAW] > ctrl_lims_adc[ctrl][VERT][MIN] + default_margin_adc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) runmode = FLY;  // New gesture to drop to fly mode is hold the brake all the way down for 500 ms
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

    // 5) Step the pids, update the actuator outputs  (at regular intervals)
    //
    if (pidTimer.expired() && !(runmode == SHUTDOWN && shutdown_complete))  {  // Recalculate pid and update outputs, at regular intervals
        steer_pulse_out_us = constrain(steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);  // Don't be out of range
        steer_servo.writeMicroseconds(steer_pulse_out_us);   // Write steering value to jaguar servo interface

        if (park_the_motors) {  // First check if we're in motor parking mode, if so park motors instead of running PIDs
            if ( ( abs(brake_pos_filt_adc - brake_pos_park_adc) <= default_margin_adc &&     // IF ( the brake motor is close enough to the park position AND
                abs(gas_pulse_out_us == gas_pulse_idle_us + gas_pulse_park_slack_us) )  //      so is the gas servo )
                || motorParkTimer.expired() ) {        //    OR the parking timeout has expired
                park_the_motors = false;                                                // THEN stop trying to park the motors
            }
            else {
                gas_pulse_out_us = gas_pulse_idle_us + gas_pulse_park_slack_us;
                gas_servo.writeMicroseconds(gas_pulse_out_us);
                if (brake_pos_filt_adc + brake_pos_margin_adc <= brake_pos_park_adc) brake_pulse_out_us = map (brake_pos_filt_adc, brake_pos_park_adc, brake_pos_nom_lim_retract_adc, brake_pulse_stop_us, brake_pulse_extend_us); // If brake is retracted from park point, extend toward park point, slowing as we approach
                if (brake_pos_filt_adc - brake_pos_margin_adc >= brake_pos_park_adc) brake_pulse_out_us = map (brake_pos_filt_adc, brake_pos_park_adc, brake_pos_nom_lim_extend_adc, brake_pulse_stop_us, brake_pulse_retract_us); // If brake is extended from park point, retract toward park point, slowing as we approach
                brake_pulse_out_us = constrain(brake_pulse_out_us, brake_pulse_retract_us, brake_pulse_extend_us);  // Send to the actuator. Refuse to exceed range    
                brake_servo.writeMicroseconds(brake_pulse_out_us);  // Write result to jaguar servo interface
            }
        }
        else if (runmode != BASIC) {  // Unless basicmode switch is turned on, we want brake and gas   
            pressure_target_adc = constrain(pressure_target_adc, pressure_min_adc, pressure_max_adc);  // Make sure pressure target is in range
            // printf("Brake PID rm=%-+4ld target=%-+9.4lf", runmode, (double)pressure_target_adc);
            brakeSPID.set_target((double)pressure_target_adc);
            brakeSPID.compute((double)pressure_filt_adc);
            brake_pulse_out_us = (int32_t)brakeSPID.get_output();
            // printf(" output = %-+9.4lf,  %+-4ld\n", brakeSPID.get_output(), brake_pulse_out_us);
            if ( ((brake_pos_filt_adc + brake_pos_margin_adc <= brake_pos_nom_lim_retract_adc) && (brake_pulse_out_us < brake_pulse_stop_us)) ||  // If the motor is at or past its position limit in the retract direction, and we're intending to retract more ...
                 ((brake_pos_filt_adc - brake_pos_margin_adc >= brake_pos_nom_lim_extend_adc) && (brake_pulse_out_us > brake_pulse_stop_us)) )  // ... or same thing in the extend direction ...
                brake_pulse_out_us = brake_pulse_stop_us;  // ... then stop the motor
                // Improve this by having the motor actively go back toward position range if position is beyond either limit             
            brake_servo.writeMicroseconds(brake_pulse_out_us);  // Write result to jaguar servo interface
            if (runmode == CRUISE && !cruise_adjusting) {  // Cruise loop updates gas rpm target to keep speed equal to cruise mmph target, except during cruise target adjustment, gas target is determined in cruise mode logic.
                carspeed_target_mmph = constrain(carspeed_target_mmph, 0, carspeed_redline_mmph);
                // printf("Cruise PID rm= %+-4ld target=%-+9.4lf", runmode, (double)carspeed_target_mmph);
                cruiseSPID.compute((double)carspeed_filt_mmph);
                engine_target_rpm = (int32_t)cruiseSPID.get_output();
                // printf(" output = %-+9.4lf,  %+-4ld\n", cruiseSPID.get_output(), engine_target_rpm);
            }
            if (runmode != STALL) {  // Gas loop is effective in Fly or Cruise mode, we need to determine gas actuator output from rpm target
                engine_target_rpm = constrain(engine_target_rpm, engine_idle_rpm, engine_govern_rpm);  // Make sure desired rpm isn't out of range (due to crazy pid math, for example)
                if (gasSPID.get_open_loop()) gas_pulse_out_us = map(engine_target_rpm, engine_idle_rpm, engine_govern_rpm, gas_pulse_idle_us, gas_pulse_govern_us); // scale gas rpm target onto gas pulsewidth target (unless already set in stall mode logic)
                else {  // Do soren's quasi-pid math to determine gas_pulse_out_us from engine rpm error
                    // printf("Gas PID   rm= %+-4ld target=%-+9.4lf", runmode, (double)engine_target_rpm);
                    gasSPID.set_target((double)engine_target_rpm);
                    gasSPID.compute((double)engine_filt_rpm);
                    gas_pulse_out_us = (int32_t)gasSPID.get_output();
                    // printf(" output = %-+9.4lf,  %+-4ld\n", gasSPID.get_output(), gas_pulse_out_us);
                }
                gas_servo.writeMicroseconds(gas_pulse_out_us);  // Write result to servo
            }
        }
        pidTimer.reset();  // reset timer to trigger the next update
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
    if (!ignition && engine_filt_rpm > 0) { // See if the engine is turning despite the ignition being off
        Serial.println(F("Detected engine rotation in the absense of ignition signal"));  // , engine_filt_rpm, ignition
        // I don't think we really need to panic about this, but it does seem curious. Actually this will probably occur when we're sliding
        // into camp after a ride, and kill the engine before we stop the car. For a fraction of a second the engine would keep turning anyway.
        // Or fopr that matter whenever the carb is out of tune and making the engine diesel after we kill the ign.
    }

    // 6) Service the user interface
    //
    int32_t touch_x, touch_y, trow, tcol;
    if (touchPollTimer.expired()) {
        touchPollTimer.reset();
        if (touchpanel.touched()) { // Take actions upon being touched
            touch_accel = 1 << touch_accel_exponent;  // determine value editing rate
            TS_Point touchpoint = touchpanel.getPoint();  // Retreive a point
            touchpoint.x = map(touchpoint.x, 0, disp_height_pix, disp_height_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touchpoint.y = map(touchpoint.y, 0, disp_width_pix, disp_width_pix, 0);  // Rotate touch coordinates to match tft coordinates
            touch_y = disp_height_pix-touchpoint.x; // touch point y coordinate in pixels, from origin at top left corner
            touch_x = touchpoint.y; // touch point x coordinate in pixels, from origin at top left corner
            trow = constrain((touch_y + touch_fudge)/touch_cell_v_pix, 0, 4);  // The -8 seems to be needed or the vertical touch seems off (?)
            tcol = (touch_x-touch_margin_h_pix)/touch_cell_h_pix;
            // else printf("Touch: x:%ld, y:%ld, row:%ld, col:%ld\n", touch_x, touch_y, trow, tcol);
            // Take appropriate touchscreen actions depending how we're being touched
            if (tcol==0 && trow==0 && !touch_now_touched) {
                dataset_page += 1; // Displayed dataset page can also be changed outside of simulator
                if (dataset_page >= arraysize(pagecard)) dataset_page -= arraysize(pagecard);
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
                if (touch_longpress_valid && touchHoldTimer.elapsed() > touchHoldTimer.timeout())  {
                    simulating = !simulating;
                    touch_longpress_valid = false;
                }
            }
            else if (tcol==3 && trow==0 && simulating && sim_basicsw && !touch_now_touched) basicmodesw = !basicmodesw;  // Pressed the basic mode toggle button. Toggle value, only once per touch
            else if (tcol==3 && trow==1 && simulating && sim_press) adj_val(&pressure_filt_adc, touch_accel, pressure_min_adc, pressure_max_adc);  // (+= 25) Pressed the increase brake pressure button
            else if (tcol==3 && trow==2 && simulating && sim_press) adj_val(&pressure_filt_adc, -touch_accel, pressure_min_adc, pressure_max_adc);  // (-= 25) Pressed the decrease brake pressure button
            else if (tcol==3 && trow==4 && simulating && sim_joy) adj_val(&ctrl_pos_adc[HORZ][FILT], -touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (-= 25) Pressed the joystick left button
            else if (tcol==4 && trow==0 && simulating && sim_ign && !touch_now_touched) ignition = !ignition; // Pressed the ignition switch toggle button. Toggle value, only once per touch
            else if (tcol==4 && trow==1 && simulating && sim_tach) adj_val(&engine_filt_rpm, touch_accel, 0, engine_redline_rpm);  // (+= 25) Pressed the increase engine rpm button
            else if (tcol==4 && trow==2 && simulating && sim_tach) adj_val(&engine_filt_rpm, -touch_accel, 0, engine_redline_rpm);  // (-= 25) Pressed the decrease engine rpm button
            else if (tcol==4 && trow==3 && simulating && sim_joy) adj_val(&ctrl_pos_adc[VERT][FILT], touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (+= 25) Pressed the joystick up button
            else if (tcol==4 && trow==4 && simulating && sim_joy) adj_val(&ctrl_pos_adc[VERT][FILT], -touch_accel, ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);  // (-= 25) Pressed the joystick down button
            else if (tcol==5 && trow==0 && simulating && sim_cruisesw) cruise_sw = true;  // Pressed the cruise mode button. This is a momentary control, not a toggle. Value changes back upon release
            else if (tcol==5 && trow==1 && simulating && sim_speedo) adj_val(&carspeed_filt_mmph, touch_accel, 0, carspeed_redline_mmph);  // (+= 50) // Pressed the increase vehicle speed button
            else if (tcol==5 && trow==2 && simulating && sim_speedo) adj_val(&carspeed_filt_mmph, -touch_accel, 0, carspeed_redline_mmph);  // (-= 50) Pressed the decrease vehicle speed button
            else if (tcol==5 && trow==4 && simulating && sim_joy) adj_val(&ctrl_pos_adc[HORZ][FILT], touch_accel, ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);  // (+= 25) Pressed the joystick right button                           
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
    }
    // Encoder handling
    //
    if (encoder_sw_action != NONE) {  // First deal with any unhandled switch press events
        if (serial_debugging) Serial.println("in encoder sw handling function");
        if (encoder_sw_action == LONG) {
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;
            else if (tuning_ctrl == SELECT) tuning_ctrl = OFF;
            else if (tuning_ctrl == OFF && dataset_page != LOCK) tuning_ctrl = SELECT; 
        }
        else {  // if short press
            if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tuning_ctrl == SELECT) tuning_ctrl = EDIT;  // If we were selecting a variable start editing its value
        }
        encoder_sw_action = NONE; // Our responsibility to reset this flag after handling events
    }
    if (encoder_delta != 0) {  // Now handle any new rotations
        if (serial_debugging) { Serial.print("in encoder rotation handler. dataset_page = "); Serial.println(dataset_page); }
        // int32_t spinrate = (int32_t)((double)(encoderSpinspeedTimer.elapsed())/(double)abs(encoder_delta));
        if (encoder_spinrate_isr_us >= encoder_spinrate_min_us) {  // Attempt to reject clicks coming in too fast
            encoder_spinrate_old_us = encoder_spinrate_last_us;  // Store last few spin times for filtering purposes ...
            encoder_spinrate_last_us = encoder_spinrate_us;  // ...
            encoder_spinrate_us = constrain(encoder_spinrate_isr_us, encoder_spinrate_min_us, 100000);
            int32_t spinrate_temp = (encoder_spinrate_old_us > encoder_spinrate_last_us) ? encoder_spinrate_old_us : encoder_spinrate_last_us;  // Find the slowest of the last 3 detents ...
            spinrate_temp = (spinrate_temp > encoder_spinrate_us) ? spinrate_temp : encoder_spinrate_us;  // to prevent one ultrafast double-hit to jump it too far
            encoder_edits_per_det = map(spinrate_temp, encoder_spinrate_min_us, 100000, 50, 1);  // if turning faster than 100ms/det, proportionally accelerate the effect of each detent by up to 50x 
            // encoderSpinspeedTimer.reset();
            if (tuning_ctrl == EDIT) sim_edit_delta_encoder = encoder_delta * encoder_edits_per_det;  // If a tunable value is being edited, turning the encoder changes the value
            else encoder_delta = constrain(encoder_delta, -1, 1);  // Only change one at a time when selecting
            if (tuning_ctrl == SELECT) selected_value += encoder_delta;  // If overflow constrain will fix in general handler below
            else if (tuning_ctrl == OFF) dataset_page += encoder_delta;  // If overflow tconstrain will fix in general below
        }
        encoder_delta = 0;
    }
    
    // Implement effects of changes made by encoder or touchscreen to simulating, dataset_page, selected_value, or tuning_ctrl
    //
    sim_edit_delta = sim_edit_delta_encoder + sim_edit_delta_touch;  // Allow edits using the encoder

    if (tuning_ctrl != tuning_ctrl_last || dataset_page != dataset_page_last || selected_value != selected_value_last || sim_edit_delta != 0) tuningCtrlTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
    if (tuning_ctrl != OFF && tuningCtrlTimer.expired()) tuning_ctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
    dataset_page = constrain(dataset_page, 0, arraysize(pagecard)-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
    if (dataset_page != dataset_page_last) {
        if (tuning_ctrl == EDIT) tuning_ctrl = SELECT;
        dataset_page_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
    }
    if (tuning_ctrl == SELECT) {
        if (dataset_page >= 4) selected_value = constrain (selected_value, 5, 7);  // Skip unchangeable values for all PID modes
        else if (dataset_page == JOY) selected_value = constrain (selected_value, 2, 7);  // Skip unchangeable values for joy mode
        else if (dataset_page == LOCK) selected_value = constrain (selected_value, 2, 7);  // Skip unchangeable values for joy mode
        else selected_value = constrain(selected_value, 0, arraysize(dataset_page_names[dataset_page])-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
        if (selected_value != selected_value_last) selected_val_dirty = true;
    }
    if (tuning_ctrl != tuning_ctrl_last || dataset_page_dirty) selected_val_dirty = true;
    tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
    
    if (tuning_ctrl == EDIT && sim_edit_delta != 0) {  // Change tunable values when editing
        if (dataset_page == LOCK)  switch (selected_value) {
            case 2:  brakeSPID.set_proportionality((sim_edit_delta > 0) ? ERROR_TERM : SENSED_INPUT);  break;
            case 3:  sim_brkpos = (sim_edit_delta != 0) ? (sim_edit_delta > 0) : sim_brkpos;  break;
            case 4:  sim_joy = (sim_edit_delta != 0) ? (sim_edit_delta > 0) : sim_joy;  break;
            case 5:  sim_press = (sim_edit_delta != 0) ? (sim_edit_delta > 0) : sim_press;  break;
            case 6:  sim_tach = (sim_edit_delta != 0) ? (sim_edit_delta > 0) : sim_tach;  break;
            case 7:  sim_speedo = (sim_edit_delta != 0) ? (sim_edit_delta > 0) : sim_speedo;  break;
        }
        else if (dataset_page == JOY)  switch (selected_value) {
            case 2:  adj_val(&ctrl_lims_adc[ctrl][HORZ][MIN], sim_edit_delta, 0, adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][DB] / 2 - 1);  break;
            case 3:  adj_val(&ctrl_lims_adc[ctrl][HORZ][MAX], sim_edit_delta, adc_midscale_adc + ctrl_lims_adc[ctrl][HORZ][DB] / 2 + 1, adc_range_adc);  break;
            case 4:  adj_val(&ctrl_lims_adc[ctrl][HORZ][DB], sim_edit_delta, 0, (adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adc_midscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adc_midscale_adc) : 2*(adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]));  break;
            case 5:  adj_val(&ctrl_lims_adc[ctrl][VERT][MIN], sim_edit_delta, 0, adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][DB] / 2 - 1);  break;
            case 6:  adj_val(&ctrl_lims_adc[ctrl][VERT][MAX], sim_edit_delta, adc_midscale_adc + ctrl_lims_adc[ctrl][VERT][DB] / 2 + 1, adc_range_adc);  break;
            case 7:  adj_val(&ctrl_lims_adc[ctrl][VERT][DB], sim_edit_delta, 0, (adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adc_midscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adc_midscale_adc) : 2*(adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]));  break;
        }
        else if (dataset_page == CAR)  switch (selected_value) {
            case 0:  adj_val(&gas_governor_percent, sim_edit_delta, 0, 100);  break;
            case 1:  adj_val(&engine_idle_rpm, sim_edit_delta, 0, engine_redline_rpm -1);  break;
            case 2:  adj_val(&engine_redline_rpm, sim_edit_delta, engine_idle_rpm, 8000);  break;
            case 3:  adj_val(&carspeed_idle_mmph, sim_edit_delta, 0, carspeed_redline_mmph - 1);  break;
            case 4:  adj_val(&carspeed_redline_mmph, sim_edit_delta, carspeed_idle_mmph, 30000);  break;
            case 5:  ctrl = (sim_edit_delta != 0) ? (sim_edit_delta > 0) : ctrl;  break;
            case 6:  gasSPID.set_open_loop((sim_edit_delta != 0) ? (sim_edit_delta > 0) : gasSPID.get_open_loop());  break;
            case 7:  adj_val(&brake_pos_zeropoint_adc, sim_edit_delta, brake_pos_nom_lim_retract_adc, brake_pos_nom_lim_extend_adc);  break;
        }
        else if (dataset_page == PWMS)  switch (selected_value) {
            case 0:  adj_val(&steer_pulse_left_us, sim_edit_delta, pwm_pulse_min_us, steer_pulse_stop_us - 1);  break;
            case 1:  adj_val(&steer_pulse_stop_us, sim_edit_delta, steer_pulse_left_us + 1, steer_pulse_right_us - 1);  break;
            case 2:  adj_val(&steer_pulse_right_us, sim_edit_delta, steer_pulse_stop_us + 1, pwm_pulse_max_us);  break;
            case 3:  adj_val(&brake_pulse_extend_us, sim_edit_delta, brake_pulse_stop_us + 1, pwm_pulse_max_us);  break;
            case 4:  adj_val(&brake_pulse_stop_us, sim_edit_delta, brake_pulse_retract_us + 1, brake_pulse_extend_us - 1);  break;
            case 5:  adj_val(&brake_pulse_retract_us, sim_edit_delta, pwm_pulse_min_us, brake_pulse_stop_us -1);  break;
            case 6:  adj_val(&gas_pulse_idle_us, sim_edit_delta, gas_pulse_redline_us + 1, pwm_pulse_max_us - gas_pulse_park_slack_us);  break;
            case 7:  adj_val(&gas_pulse_redline_us, sim_edit_delta, pwm_pulse_min_us, gas_pulse_idle_us - 1);  break;
        }
        else if (dataset_page == BPID) {
            if (selected_value == 5) brakeSPID.set_tunings( brakeSPID.get_kp()+0.001*(double)sim_edit_delta, brakeSPID.get_ki_hz(), brakeSPID.get_kd_s() );
            if (selected_value == 6) brakeSPID.set_tunings( brakeSPID.get_kp(), brakeSPID.get_ki_hz()+0.001*(double)sim_edit_delta, brakeSPID.get_kd_s() );
            if (selected_value == 7) brakeSPID.set_tunings( brakeSPID.get_kp(), brakeSPID.get_ki_hz(), brakeSPID.get_kd_s()+0.001*(double)sim_edit_delta );
        }
        else if (dataset_page == GPID) {
            if (selected_value == 5) gasSPID.set_tunings( gasSPID.get_kp()+0.001*(double)sim_edit_delta, gasSPID.get_ki_hz(), gasSPID.get_kd_s() );
            if (selected_value == 6) gasSPID.set_tunings( gasSPID.get_kp(), gasSPID.get_ki_hz()+0.001*(double)sim_edit_delta, gasSPID.get_kd_s() );
            if (selected_value == 7) gasSPID.set_tunings( gasSPID.get_kp(), gasSPID.get_ki_hz(), gasSPID.get_kd_s()+0.001*(double)sim_edit_delta );
        }
        else if (dataset_page == CPID) {
            if (selected_value == 5) cruiseSPID.set_tunings( cruiseSPID.get_kp()+0.001*(double)sim_edit_delta, cruiseSPID.get_ki_hz(), cruiseSPID.get_kd_s() );
            if (selected_value == 6) cruiseSPID.set_tunings( cruiseSPID.get_kp(), cruiseSPID.get_ki_hz()+0.001*(double)sim_edit_delta, cruiseSPID.get_kd_s() );
            if (selected_value == 7) cruiseSPID.set_tunings( cruiseSPID.get_kp(), cruiseSPID.get_ki_hz(), cruiseSPID.get_kd_s()+0.001*(double)sim_edit_delta );
        }
    }

    // Do any needed actions due to changes made
    if (ignition != ignition_last) {  // Car was turned on or off
        digitalWrite(ignition_pin, ignition); // Make it real
        if (!ignition && carspeed_filt_mmph) panic_stop = true;
    }
    ignition_last = ignition; // Make sure this goes after the last comparison
    if (!carspeed_filt_mmph) panic_stop = false;

    // Update displayed telemetry values to the screen
    if (display_enabled)  {
        if (simulating != simulating_last) draw_simbuttons(simulating);  // if we just entered simulator draw the simulator buttons, or if we just left erase them
        simulating_last = simulating;
        if (dataset_page_dirty) {
            draw_fixed(true);
            draw_dynamic(0, 0, -1, -1, 2);
        }
        if (selected_val_dirty) draw_dynamic(0, 0, -1, -1, 3);
        dataset_page_dirty = false;
        selected_val_dirty = false;
        int32_t range;
        // draw_fixed();
        // if (simulating) draw_touchgrid(); // Redraw only the at-risk content of the touch grid
        draw_dynamic(0, runmode, -1, -1, 1);
        draw_dyn_pid(1, carspeed_filt_mmph, 0, carspeed_redline_mmph, (int32_t)cruiseSPID.get_target(), 4);
        draw_dyn_pid(2, engine_filt_rpm, 0, engine_redline_rpm, (int32_t)gasSPID.get_target(), 4);
        draw_dyn_pid(3, pressure_filt_adc, pressure_min_adc, pressure_max_adc, (int32_t)brakeSPID.get_target(), 4);  // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.get_target() : pressure_target_adc, 4);
        draw_dynamic(4, ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX], 0);
        draw_dynamic(5, ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX], 0);
        draw_dynamic(6, (int32_t)cruiseSPID.get_target(), 0, carspeed_govern_mmph, 0);
        draw_dynamic(7, (int32_t)brakeSPID.get_target(), pressure_min_adc, pressure_max_adc, 0);
        draw_dynamic(8, (int32_t)gasSPID.get_target(), 0, engine_redline_rpm, 0);
        draw_dynamic(9, brake_pulse_out_us, brake_pulse_extend_us, brake_pulse_retract_us, 0);
        draw_dynamic(10, gas_pulse_out_us, gas_pulse_idle_us, gas_pulse_redline_us, 0);
        draw_dynamic(11, steer_pulse_out_us, steer_pulse_left_us, steer_pulse_right_us, 0);
        if (dataset_page == LOCK) {
            draw_dynamic(12, battery_filt_mv, 0, battery_max_mv, 0);
            draw_dynamic(13, brake_pos_filt_adc, brake_pos_nom_lim_retract_adc, brake_pos_nom_lim_extend_adc, 0);
            draw_dynamic(14, brakeSPID.get_proportionality(), -1, -1, 0);
            draw_dynamic(15, sim_brkpos, -1, -1, 0);
            draw_dynamic(16, sim_joy, -1, -1, 0);
            draw_dynamic(17, sim_press, -1, -1, 0);
            draw_dynamic(18, sim_tach, -1, -1, 0);
            draw_dynamic(19, sim_speedo, -1, -1, 0);
        }
        else if (dataset_page == JOY) {
            draw_dynamic(12, ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX], 0);
            draw_dynamic(13, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX], 0);
            draw_dynamic(14, ctrl_lims_adc[ctrl][HORZ][MIN], 0, (adc_range_adc-ctrl_lims_adc[ctrl][HORZ][MAX])/2, 0);
            draw_dynamic(15, ctrl_lims_adc[ctrl][HORZ][MAX], (ctrl_lims_adc[ctrl][HORZ][MIN]-adc_range_adc)/2, adc_range_adc, 0);
            draw_dynamic(16, ctrl_lims_adc[ctrl][HORZ][DB], 0, (adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adc_midscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adc_midscale_adc) : 2*(adc_midscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]), 0);
            draw_dynamic(17, ctrl_lims_adc[ctrl][VERT][MIN], 0, (adc_range_adc-ctrl_lims_adc[ctrl][VERT][MAX])/2, 0);
            draw_dynamic(18, ctrl_lims_adc[ctrl][VERT][MAX], (ctrl_lims_adc[ctrl][VERT][MIN]-adc_range_adc)/2, adc_range_adc, 0);
            draw_dynamic(19, ctrl_lims_adc[ctrl][VERT][DB], 0, (adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adc_midscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adc_midscale_adc) : 2*(adc_midscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]), 0);
        }
        else if (dataset_page == CAR) {
            draw_dynamic(12, gas_governor_percent, 0, 100, 0);
            draw_dynamic(13, engine_idle_rpm, 0, engine_redline_rpm, 0);
            draw_dynamic(14, engine_redline_rpm, 0, engine_max_rpm, 0);
            draw_dynamic(15, carspeed_idle_mmph, 0, carspeed_redline_mmph, 0);
            draw_dynamic(16, carspeed_redline_mmph, 0, carspeed_max_mmph, 0);
            draw_dynamic(17, ctrl, -1, -1, 0);  // 0 if hotrc
            draw_dynamic(18, gasSPID.get_open_loop(), -1, -1, 0);
            draw_dynamic(19, brake_pos_zeropoint_adc, brake_pos_nom_lim_retract_adc, brake_pos_nom_lim_extend_adc, 0);   
        }
        else if (dataset_page == PWMS) {
            draw_dynamic(12, steer_pulse_left_us, steer_pulse_left_max_us, steer_pulse_stop_us, 0);
            draw_dynamic(13, steer_pulse_stop_us, steer_pulse_left_us, steer_pulse_right_us, 0);
            draw_dynamic(14, steer_pulse_right_us, steer_pulse_stop_us, steer_pulse_right_max_us, 0);
            draw_dynamic(15, brake_pulse_extend_us, brake_pulse_stop_us, brake_pulse_extend_max_us, 0);
            draw_dynamic(16, brake_pulse_stop_us, brake_pulse_retract_us, brake_pulse_extend_us, 0);
            draw_dynamic(17, brake_pulse_retract_us, brake_pulse_retract_max_us, brake_pulse_stop_us, 0);
            draw_dynamic(18, gas_pulse_idle_us, gas_pulse_ccw_max_us, gas_pulse_cw_max_us, 0);
            draw_dynamic(19, gas_pulse_redline_us, gas_pulse_ccw_max_us, gas_pulse_cw_max_us, 0);
        }
        else if (dataset_page == BPID) {
            range = pressure_max_adc-pressure_min_adc;
            draw_dynamic(12, (int32_t)(brakeSPID.get_error()), -range, range, 0);
            draw_dynamic(13, (int32_t)(brakeSPID.get_p_term()), -range, range, 0);
            draw_dynamic(14, (int32_t)(brakeSPID.get_i_term()), -range, range, 0);
            draw_dynamic(15, (int32_t)(brakeSPID.get_d_term()), -range, range, 0);
            draw_dynamic(16, (int32_t)(brakeSPID.get_output()), -range, range, 0);  // brake_spid_carspeed_delta_adc, -range, range, 0);
            draw_dynamic(17, brakeSPID.get_disp_kp_1k(), 0, 1000, 0);  // Real value is 1000 * smaller than displayed
            draw_dynamic(18, brakeSPID.get_disp_ki_mhz(), 0, 1000, 0);
            draw_dynamic(19, brakeSPID.get_disp_kd_ms(), 0, 1000, 0);
        }
        else if (dataset_page == GPID) {
            range = engine_govern_rpm-engine_idle_rpm;
            draw_dynamic(12, (int32_t)(gasSPID.get_error()), -range, range, 0);
            draw_dynamic(13, (int32_t)(gasSPID.get_p_term()), -range, range, 0);
            draw_dynamic(14, (int32_t)(gasSPID.get_i_term()), -range, range, 0);
            draw_dynamic(15, (int32_t)(gasSPID.get_d_term()), -range, range, 0);
            draw_dynamic(16, (int32_t)(gasSPID.get_output()), -range, range, 0);  // gas_spid_carspeed_delta_adc, -range, range, 0);
            draw_dynamic(17, gasSPID.get_disp_kp_1k(), 0, 1000, 0);  // Real value is 1000 * smaller than displayed
            draw_dynamic(18, gasSPID.get_disp_ki_mhz(), 0, 1000, 0);
            draw_dynamic(19, gasSPID.get_disp_kd_ms(), 0, 1000, 0);
        }
        else if (dataset_page == CPID) {
            range = carspeed_govern_mmph-carspeed_idle_mmph;
            draw_dynamic(12, (int32_t)(cruiseSPID.get_error()), -range, range, 0);
            draw_dynamic(13, (int32_t)(cruiseSPID.get_p_term()), -range, range, 0);
            draw_dynamic(14, (int32_t)(cruiseSPID.get_i_term()), -range, range, 0);
            draw_dynamic(15, (int32_t)(cruiseSPID.get_d_term()), -range, range, 0);
            draw_dynamic(16, (int32_t)(cruiseSPID.get_output()), -range, range, 0);  // cruise_spid_carspeed_delta_adc, -range, range, 0);
            draw_dynamic(17, cruiseSPID.get_disp_kp_1k(), 0, 1000, 0);  // Real value is 1000 * smaller than displayed
            draw_dynamic(18, cruiseSPID.get_disp_ki_mhz(), 0, 1000, 0);
            draw_dynamic(19, cruiseSPID.get_disp_kd_ms(), 0, 1000, 0);
        }
        draw_bool(basicmodesw, 0);
        draw_bool(ignition, 1);
        draw_bool(cruise_sw, 2);
    }
    dataset_page_last = dataset_page;
    selected_value_last = selected_value;
    
    if (serial_debugging && print_timestamps) printf("%ld ms, %ld Hz\n", (int32_t)((double)(abs(mycros()-loopzero)/1000)), (int32_t)(1000000/((double)(abs(mycros()-loopzero)))));

    // 8) Do the control loop bookkeeping at the end of each loop
    //
    sim_edit_delta_touch = 0;
    sim_edit_delta_encoder = 0;
    disp_redraw_all = false;
    loopno++;  // I like to count how many loops
    if (runmode != SHUTDOWN) shutdown_complete = false;
    if (runmode != oldmode) we_just_switched_modes = true;      // If changing runmode, set this so new mode logic can perform initial actions
    else we_just_switched_modes = false;    // Reset this variable
    oldmode = runmode;   // remember what mode we're in for next time
    loop_period_us = loopTimer.elapsed();  // abs is to handle when mycros() overflows back to 0
    if (!loop_period_us) loop_period_us++;  // ensure loop period is never zero since it gets divided by
    loop_freq_hz = (int32_t)(1000000/(double)loop_period_us);
    loopTimer.reset();
}