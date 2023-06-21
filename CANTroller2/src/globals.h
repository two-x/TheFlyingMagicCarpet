#ifndef GLOBALS_H
#define GLOBALS_H
#undef min
#undef max
#undef map
#undef constrain
#ifdef DUE
    #include <LibPrintf.h>  // This works on Due but not ESP32
#endif
#include "Arduino.h"
#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <string>
#include <iomanip>
// #include "classes.h"
#include "spid.h"
// #include "disp.h"

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

#define arraysize(x) ((int32_t)(sizeof(x) / sizeof((x)[0])))  // A macro function to determine the length of string arrays
#define floor(amt, lim) ((amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)
// #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
// #define min(a, b) ( (a <= b) ? a : b)
// #define max(a, b) ( (a >= b) ? a : b)

// Defines for all the GPIO pins we're using
#ifdef ESP32_SX_DEVKIT
    #define button_pin 0  // (button0 / strap to 1) - This is one of the buttons on the esp32 board
    #define joy_horz_pin 1  // (adc) - Analog input, tells us left-right position of joystick. Take complement of ADC value gives:  Low values for left, High values for right.
    #define joy_vert_pin 2  // (adc) - Analog input, tells us up-down position of joystick. Take complement of ADC value gives:  Low values for down, High values for up.
    #define tft_dc_pin 3  // (strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
    #define battery_pin 4  // (adc) -  Analog input, mule battery voltage level, full scale is 15.638V
    #define pot_wipe_pin 5  // (adc) - Analog in from 20k pot. Use 1% series R=22k to 5V on wipe=CW-0ohm side, and R=15k to gnd on wipe-CCW-0ohm side. Gives wipe range of 1.315V (CCW) to 3.070V (CW) with 80 uA draw.
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
    #define hotrc_horz_pin 17  // (pwm0 / tx1) - Hotrc thumb joystick input.
    #define hotrc_vert_pin 18  // (pwm0 / rx1) - Hotrc bidirectional trigger input
    #define syspower_pin 38  // (usb-otg) - Output, flips a relay to power all the tranducers
    #define hotrc_ch4_pin 20  // (usb-otg) - Hotrc Ch3 toggle output, used to toggle cruise mode
    #define hotrc_ch3_pin 21  // (pwm0) - Hotrc Ch4 toggle output, used to panic stop & kill ignition
    #define tach_pulse_pin 35  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
    #define speedo_pulse_pin 36  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
    #define ignition_pin 37  // (spi-ram / oct-spi) - Output flips a relay to kill the car ignition, active high (no pullup)
    #define onewire_pin 19  // // (spi-ram / oct-spi) - Onewire bus for temperature sensor data
    #define tft_rst_pin 39  // TFT Reset
    #define encoder_b_pin 40  // Int input, The B pin (aka DT pin) of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
    #define encoder_a_pin 41  // Int input, The A pin (aka CLK pin) of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
    #define encoder_sw_pin 42  // Int input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
    #define uart0_tx_pin 43  // (uart0 tx) - Reserve for possible jaguar interface
    #define uart0_rx_pin 44  // (uart0 rx) - Reserve for possible jaguar interface
    #define cruise_sw_pin 45  // (strap to 0) - Input, momentary button low pulse >500ms in fly mode means start cruise mode. Any pulse in cruise mode goes to fly mode. Active low. (needs pullup)
    #define basicmodesw_pin 46  // (strap X) - Input, asserted to tell us to run in basic mode.   (needs pullup)
    #define sdcard_cs_pin 47  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
    #define neopixel_pin 48  // (rgb led) - Data line to onboard Neopixel WS281x

    #define tp_irq_pin -1  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
    #define tft_ledk_pin -1  // (spi-ram / oct-spi) - Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
    #define encoder_pwr_pin -1
    #define led_rx_pin -1  // Unused on esp32
    #define led_tx_pin -1  // Unused on esp32
    #define heartbeat_led_pin -1
#else  // Applies to Due
    #define sdcard_cs_pin 4  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
    #define tft_ledk_pin 5  // Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
    #define tp_irq_pin 7  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
    #define tft_dc_pin 9  // Output, Assert when sending data to display chip to indicate commands vs. screen data
    #define tft_cs_pin 10  // Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
    #define tft_sclk_pin 11  // (spi0 sclk) - Used as spi interface to tft screen
    #define tft_miso_pin 12  // (spi0 miso) - Used as spi interface to tft screen
    #define tft_mosi_pin 13  // (spi0 mosi) - Used as spi interface to tft screen
    // #define heartbeat_led_pin 13  // Output, This is the LED labeled "L" onboard the arduino due.  Active high.
    #define encoder_sw_pin 18  // Int input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
    #define encoder_b_pin 19  // Int input, The B pin (aka DT pin) of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
    #define encoder_pwr_pin 20  // Provide 3.3V for encoder (testing)
    #define encoder_a_pin 21  // Int input, The A pin (aka CLK pin) of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
    #define speedo_pulse_pin 23  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
    #define tach_pulse_pin 25  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
    #define syspower_pin 27  // Output, flips a relay to power all the tranducers
    #define steer_pwm_pin 29  // Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
    #define onewire_pin 31  // For temperature sensors
    #define fun_pin 33  // Available
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
    #define heartbeat_led_pin 73  // Another on-board led (Due only)
    #define pot_wipe_pin A6  // Analog in from 20k pot. Use 1% series R=22k to 5V on wipe=CW-0ohm side, and R=15k to gnd on wipe-CCW-0ohm side. Gives wipe range of 1.315V (CCW) to 3.070V (CW) with 80 uA draw.
    #define battery_pin A7  // Analog input, mule battery voltage level, full scale is 15.638V
    #define joy_horz_pin A8  // Analog input, tells us left-right position of joystick. Take complement of ADC value gives:  Low values for left, High values for right.
    #define joy_vert_pin A9  // Analog input, tells us up-down position of joystick. Take complement of ADC value gives:  Low values for down, High values for up.
    #define pressure_pin A10  // Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
    #define brake_pos_pin A11  // Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.

    #define neopixel_pin -1
    #define button_pin -1
#endif

#define adcbits 12
#define adcrange_adc 4095  // = 2^adcbits-1
#define adcmidscale_adc 2047  // = 2^(adcbits-1)-1
#define serial_debugging true
#define timestamp_loop false  // Makes code write out timestamps throughout loop to serial port

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

// LCD is 2.8in diagonal, 240x320 pixels
// LCD supports 18-bit color, but GFX library uses 16-bit color, organized (MSB) 5b-red, 6b-green, 5b-blue (LSB)
// Since the RGB don't line up with the nibble boundaries, it's tricky to quantify a color, here are some colors:
// Color picker websites: http://www.barth-dev.de/online/rgb565 , https://chrishewett.com/blog/true-rgb565-colour-picker/

// Globals -------------------
//

int32_t mycros(void) {  // This is "my" micros() function that returns signed int32
    uint32_t temp = micros();
    return (int32_t)(temp &= 0x7fffffff);  // Note this overflows every 35 min due to being only 31 bits. 
}
// Timer
// For controlling event timing. Communicates in signed int32 values but uses uint32 under the hood to
// last 72 minutes between overflows. However you can only time for 36 minutes max.
// Note: experimenting with use of micros() instead of mycros(), check for errors on overflow and possibly revert
class Timer {
  protected:
    volatile uint32_t start_us;  // start time in us
    int32_t timeout_us = 0;  // in us
    int32_t paused_us;
    bool enabled = true;
  public:
    Timer (void) { start_us = micros(); };
    Timer (int32_t arg1) { set(arg1); };
    void reset()  { start_us = micros(); }
    bool expired()  { return (enabled) ? abs((int32_t)(micros() - start_us)) > timeout_us : false; }
    int32_t elapsed()  { return (enabled) ? abs((int32_t)(micros() - start_us)) : 0; }
    int32_t timeout()  { return (int32_t)timeout_us; }
    void set (int32_t arg1)  {
        timeout_us = arg1;
        start_us = micros();
    }
    int32_t remain()  { 
        uint32_t temp = abs((int32_t)(micros() - start_us));
        return (timeout_us - (int32_t)temp);
    }
    void disable() { enabled = false; }  // int32_t pause()
    void enable() {  // int32_t resume()
        enabled = true;
        reset();
    }
};

// display related globals
#define BLK  0x0000
#define BLU  0x001f
#define MBLU 0x009f  // midnight blue. b/c true blue too dark to see over black
#define RBLU 0x043f  // royal blue
#define RED  0xf800
#define DRED 0xb000
#define GRN  0x07e0
#define CYN  0x07ff  // 00000 111 111 11111 
#define DCYN 0x0575  //
#define MGT  0xf81f
#define ORG  0xfca0
#define DORG 0xfa40  // Dark orange
#define YEL  0xffe0
#define LYEL 0xfff8
#define WHT  0xffff
#define DGRY 0x39c7  // very dark grey
#define GRY1 0x8410  // 10000 100 000 10000 = 84 10  dark grey
#define GRY2 0xc618  // 11000 110 000 11000 = C6 18  light grey
#define LGRY 0xd6ba  // very light grey
#define PNK  0xfcf3  // Pink is the best color
#define DPNK 0xfa8a  // We need all shades of pink
#define LPNK 0xfe18  // Especially light pink, the champagne of pinks

enum dataset_pages { RUN, JOY, CAR, PWMS, BPID, GPID, CPID, TEMP };
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
#define disp_maxlength 6  // How many characters fit between the ":" and the units string
#define touch_cell_v_pix 48  // When touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53  // When touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1  // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen

char telemetry[disp_fixed_lines][9] = {  
    "   Speed",
    "    Tach",
    "Brk Pres",   
    "Joy Horz",
    "Joy Vert",
    "CruisTgt",
    " Brk Tgt",
    " Gas Tgt",
    " Brk PWM",
    " Gas PWM",
    "SteerPWM",
};
char pagecard[8][5] = { "Run ", "Joy ", "Car ", "PWMs", "Bpid", "Gpid", "Cpid", "Temp" };
char dataset_page_names[arraysize(pagecard)][disp_tuning_lines][9] = {
    {   " Battery",  // RUN
        " Brk Pos",
        "     Pot",
        "SimBkPos",
        " Sim Joy",
        "Sim Pres",
        "Sim Tach",
        " Sim Spd", },
    {   "Horz Raw",  // JOY
        "Vert Raw",
        "Horz Min",
        "Horz Max",
        " Horz DZ",
        "Vert Min",
        "Vert Max",
        " Vert DZ", },
    {   "Governor",  // CAR
        "Eng Idle",
        "Eng RedL",
        "Spd Idle",
        "Spd RedL",
        "Joystck?",
        " Cal Brk",
        " Cal Gas", },
    {   "Str Left",  // PWMS
        "Str Stop",
        "Str Rght",
        "Brk Extd",
        "Brk Stop",
        "Brk Retr",
        "Gas Idle",
        "Gas RedL", },
    {   "Pres Err",  // BPID
        "  P Term",
        "  I Term",
        "  D Term",
        " PID Out",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)", },
    {   " Eng Err",  // GPID
        "  P Term",
        "  I Term",
        "  D Term",
        " PID Out",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)" },
    {   " Spd Err",  // CPID
        "  P Term",
        "  I Term",
        "  D Term",
        " PID Out",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)", },
    {   " Tmp Amb",  // TEMP
        " Tmp Eng",
        "Tmp WhFL",
        "Tmp WhFR",
        "Tmp WhRL",
        "Tmp WhRR",
        "GasOpnLp",
        "BrkZeroP", },
};
char units[disp_fixed_lines][5] = { "mmph", "rpm ", "psi ", "adc ", "adc ", "mmph", "adc ", "rpm ", "\xe5s  ", "\xe5s  ", "\xe5s  " };
char tuneunits[arraysize(pagecard)][disp_tuning_lines][5] = {
    { "mV  ", "thou", "%   ", "    ", "    ", "    ", "    ", "    " },  // RUN
    { "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc " },  // JOY
    { "%   ", "rpm ", "rpm ", "mmph", "mmph", "    ", "    ", "    " },  // CAR
    { "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  " },  // PWM
    { "psi ", "psi ", "psi ", "psi ", "psi ", "/1k ", "mHz ", "ms  " },  // BPID
    { "mmph", "mmph", "mmph", "mmph", "mmph", "/1k ", "mHz ", "ms  " },  // GPID
    { "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "/1k ", "mHz ", "ms  " },  // CPID
    { "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "    ", "thou" },  // TEMP
    // { "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "    ", "    " },  // TEMP
};
char simgrid[4][3][5] = {
    { "prs\x18", "rpm\x18", "car\x18" },
    { "prs\x19", "rpm\x19", "car\x19" },
    { "    ", " \x1e  ", "    " },
    { " \x11  ", " \x1f  ", "  \x10 " },  // Font special characters map:  https://learn.adafruit.com/assets/103682
};
char modecard[7][7] = { "Basic", "Shutdn", "Stall", "Hold", "Fly", "Cruise", "Cal" };
char side_menu_buttons[5][4] = { "PAG", "SEL", "\x18  ", "\x19  ", "SIM" };  // Pad shorter names with spaces on the right
char top_menu_buttons[4][6] = { " CAL ", "BASIC", " IGN ", "POWER" };  // Pad shorter names with spaces to center
char disp_values[disp_lines][disp_maxlength+1];  // Holds previously drawn value strings for each line
bool disp_polarities[disp_lines];  // Holds sign of previously drawn values
bool display_enabled = true;  // Should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool disp_redraw_all = true;
bool disp_bool_values[6];
bool disp_selected_val_dirty = true;
bool disp_dataset_page_dirty = true;
bool disp_sidemenu_dirty = true;
bool disp_runmode_dirty = true;
int32_t colorcard[arraysize(modecard)] = { MGT, RED, ORG, YEL, GRN, CYN, MBLU };
int32_t disp_needles[disp_lines];
int32_t disp_targets[disp_lines];
int32_t disp_age_quanta[disp_lines];
Timer dispAgeTimer[disp_lines];  // int32_t disp_age_timer_us[disp_lines];
Timer dispRefreshTimer (100000);  // Don't refresh screen faster than this (16667us = 60fps, 33333us = 30fps, 66666us = 15fps)

// tuning-ui related globals
enum tuning_ctrl_states { OFF, SELECT, EDIT };
int32_t tuning_ctrl = OFF;
int32_t tuning_ctrl_last = OFF;
int32_t dataset_page = RUN;  // Which of the six 8-value dataset pages is currently displayed, and available to edit
int32_t dataset_page_last = dataset_page;
int32_t selected_value = 0;  // In the real time tuning UI, which of the editable values (0-7) is selected. -1 for none 
int32_t selected_value_last = 0;
//  ---- tunable ----
Timer tuningCtrlTimer (25000000);  // This times out edit mode after a a long period of inactivity

// run state globals
enum runmodes { BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL };
int32_t runmode = SHUTDOWN;
int32_t oldmode = BASIC;  // So we can tell when the mode has just changed. start as different to trigger_mode start algo
int32_t gesture_progress = 0;  // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
int32_t shutdown_color = colorcard[SHUTDOWN];
bool shutdown_complete = false;  // Shutdown mode has completed its work and can stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;  // Indicates we should release the brake & gas so the pedals can be used manually without interference
bool calmode_request = false;
bool panic_stop = false;
bool cruise_gesturing = false;  // Is cruise mode enabled by gesturing?  Otherwise by press of cruise button
bool cruise_sw_held = false;
bool cruise_adjusting = false;
Timer motorParkTimer;
Timer gestureFlyTimer;  // Used to keep track of time for gesturing for going in and out of fly/cruise modes
Timer cruiseSwTimer;
Timer sleepInactivityTimer (10000000);  // After shutdown how long to wait before powering down to sleep
//  ---- tunable ----
int32_t motor_park_timeout_us = 3000000;  // If we can't park the motors faster than this, then give up.
int32_t gesture_flytimeout_us = 500000;  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
int32_t cruise_sw_timeout_us = 500000;  // how long do you have to hold down the cruise button to start cruise mode (in us)

// calibration related
bool cal_joyvert_brkmotor = false;  // Allows direct control of brake motor using controller vert
bool cal_pot_gasservo = false;  // Allows direct control of gas servo using pot
bool cal_pot_gas_ready = false;  // To avoid immediately overturning gas pot, first pot must be turned to valid range
bool cal_set_hotrc_failsafe_ready = false;  

// generic values
//  ---- tunable ----
int32_t default_margin_adc = 12;  // Default margin of error for comparisons of adc values (ADC count 0-4095)
Timer sanityTimer (7000000);  // Allows code to fail in a sensible way after a delay if nothing is happening

// pid related globals
//  ---- tunable ----
int32_t pid_period_ms = 50;
Timer pidTimer (pid_period_ms*1000);  // not actually tunable, just needs value above
int32_t brake_spid_ctrl_dir = SPID::FWD;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value
double brake_spid_initial_kp = 0.588;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
double brake_spid_initial_ki_hz = 0.193;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
double brake_spid_initial_kd_s = 0.252;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
double cruise_spid_initial_kp = 0.157;  // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
double cruise_spid_initial_ki_hz = 0.035;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
double cruise_spid_initial_kd_s = 0.044;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
int32_t cruise_spid_ctrl_dir = SPID::FWD;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value
double gas_spid_initial_kp = 0.064;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
double gas_spid_initial_ki_hz = 0.015;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
double gas_spid_initial_kd_s = 0.022;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
int32_t gas_spid_ctrl_dir = SPID::REV;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value

// mule battery related
double battery_adc = adcmidscale_adc;
double battery_mv = 10000;
double battery_filt_mv = 10000;
//  ---- tunable ----
double battery_max_mv = 16000;  // The max vehicle voltage we can sense. Design resistor divider to match. Must exceed max V possible.
double battery_convert_mv_per_adc = battery_max_mv/adcrange_adc;
bool battery_convert_invert = false;
int32_t battery_convert_polarity = SPID::FWD;
double battery_ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 

// potentiometer related
double pot_percent = 50;
double pot_filt_percent = pot_percent;
double pot_min_percent = 0;  //
double pot_max_percent = 100;  //
//  ---- tunable ----
double pot_min_adc = 0;  // TUNED 230603 - Used only in determining theconversion factor
double pot_max_adc = 4005;  // TUNED 230613 - adc max measured = ?, or 9x.? % of adc_range. Used only in determining theconversion factor
double pot_convert_percent_per_adc = 100/(pot_max_adc - pot_min_adc);  // 100 % / (3996 adc - 0 adc) = 0.025 %/adc
bool pot_convert_invert = false;
int32_t pot_convert_polarity = SPID::FWD;
double pot_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 

// controller related
enum ctrls { HOTRC };  // This is a bad hack. Since JOY is already enum'd as 1 for dataset pages
enum ctrl_axes { HORZ, VERT };
enum ctrl_thresh { MIN, DB, MAX };
enum ctrl_edge { BOT, TOP };
enum raw_filt { RAW, FILT };
int32_t ctrl_db_adc[2][2];  // [HORZ/VERT] [BOT, TOP] - to store the top and bottom deadband values for each axis of selected controller
int32_t ctrl_pos_adc[2][2] = { { adcmidscale_adc, adcmidscale_adc }, { adcmidscale_adc, adcmidscale_adc} };  // [HORZ/VERT] [RAW/FILT]
volatile bool hotrc_ch3_sw, hotrc_ch4_sw, hotrc_ch3_sw_event, hotrc_ch4_sw_event, hotrc_ch3_sw_last, hotrc_ch4_sw_last;
volatile int32_t hotrc_horz_pulse_us = 1500;
volatile int32_t hotrc_vert_pulse_us = 1500;
bool joy_centered = false;

// Merging these into Hotrc class
bool hotrc_radio_detected = false;
bool hotrc_radio_detected_last = hotrc_radio_detected;
bool hotrc_suppress_next_event = true;  // When powered up, the hotrc will trigger a Ch3 and Ch4 event we should ignore
Timer hotrcPulseTimer;  // OK to not be volatile?
//  ---- tunable ----
double hotrc_pulse_period_us = 1000000.0 / 50;


double ctrl_ema_alpha[2] = { 0.05, 0.1 };  // [HOTRC, JOY] alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t ctrl_lims_adc[2][2][3] = { { { 3,  50, 4092 }, { 3,  75, 4092 } }, { { 9, 200, 4085 }, { 9, 200, 4085 } }, }; // [HOTRC, JOY] [HORZ, VERT], [MIN, DEADBAND, MAX] values as ADC counts
bool ctrl = HOTRC;  // Use HotRC controller to drive instead of joystick?
// Limits of what pulsewidth the hotrc receiver puts out
// For some funky reason I was unable to initialize these in an array format !?!?!
// int32_t hotrc_pulse_lims_us[2][2];  // = { { 1009, 2003 }, { 1009, 2003 } };  // [HORZ/VERT] [MIN/MAX]  // These are the limits of hotrc vert and horz high pulse
int32_t hotrc_pulse_vert_min_us = 990;  // 1009;
int32_t hotrc_pulse_vert_max_us = 1990;  // 2003;
int32_t hotrc_pulse_horz_min_us = 990;  // 1009;
int32_t hotrc_pulse_horz_max_us = 1990;  // 2003;

// Maybe merging these into Hotrc class
int32_t hotrc_pos_failsafe_min_us = 450;  // The failsafe setting in the hotrc must be set to a trigger level equal to max amount of trim upward from trigger released.
int32_t hotrc_pos_failsafe_max_us = 530;
int32_t hotrc_panic_timeout = 1000000;  // how long to receive flameout-range signal from hotrc vertical before panic stopping
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
double pressure_psi;
double pressure_filt_psi;  // Stores new setpoint to give to the pid loop (brake)
// Param pressure (&pressure_adc, "Pressure:", "adc ", 658, 2100);
//  ---- tunable ----
double pressure_convert_psi_per_adc = 1000.0 * 3.3 / ( adcrange_adc * (4.5 - 0.5) );  // 1000 psi * 3.3 v / (4095 adc * (v-max v - v-min v)) = 0.2 psi/adc 
bool pressure_convert_invert = false;
int32_t pressure_convert_polarity = SPID::FWD;
double pressure_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double pressure_margin_psi = 2.5;  // Margin of error when comparing brake pressure adc values (psi)
double pressure_min_psi = 129;  // TUNED 230602 - Brake pressure when brakes are effectively off. Sensor min = 0.5V, scaled by 3.3/4.5V is 0.36V of 3.3V (ADC count 0-4095). 230430 measured 658 adc (0.554V) = no brakes
double pressure_max_psi = 452;  // TUNED 230602 - Highest possible pressure achievable by the actuator (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as chris can push (wimp)
double pressure_hold_initial_psi = 200;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
double pressure_hold_increment_psi = 10;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
double pressure_panic_initial_psi = 300;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
double pressure_panic_increment_psi = 25;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
// max pedal bent 1154

// brake actuator motor related
int32_t brake_pulse_out_us;  // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
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
double brake_pos_thou;
double brake_pos_filt_thou;
//  ---- tunable ----
double brake_pos_convert_thou_per_adc = 3.3 * 10000.0 * 1000.0 / (5.0 * adcrange_adc * 557);  // 3.3 v * 10k ohm * 1k m-in/in / (5 v * 4095 adc * 557 ohm/in) = 0.0029 in/adc = 2.89 m-in/adc 
bool brake_pos_convert_invert = false;
int32_t brake_pos_convert_polarity = SPID::FWD;
double brake_pos_ema_alpha = 0.25;
double brake_pos_abs_min_retract_thou = 335;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. ("thou"sandths of an inch)
double brake_pos_nom_lim_retract_thou = 506;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (thou)
double brake_pos_zeropoint_thou = 3179;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (thou)
double brake_pos_park_thou = 4234;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (thou)
double brake_pos_nom_lim_extend_thou = 4624;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (thou)
double brake_pos_abs_max_extend_thou = 8300;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (thou)
double brake_pos_margin_thou = 29;  //
// int32_t brake_pos_abs_min_retract_adc = 116;  // TUNED 230602 - Retract value corresponding with the absolute minimum retract actuator is capable of. (ADC count 0-4095)
// int32_t brake_pos_nom_lim_retract_adc = 175;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
// int32_t brake_pos_zeropoint_adc = 1100;  // TUNED 230602 - Brake position value corresponding to the point where fluid PSI hits zero (ADC count 0-4095)
// int32_t brake_pos_park_adc = 1465;  // TUNED 230602 - Best position to park the actuator out of the way so we can use the pedal (ADC count 0-4095)
// int32_t brake_pos_nom_lim_extend_adc = 1600;  // TUNED 230602 - Extend limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
// int32_t brake_pos_abs_max_extend_adc = 2872;  // TUNED 230602 - Extend value corresponding with the absolute max extension actuator is capable of. (ADC count 0-4095)
// int32_t brake_pos_margin_adc = 10;  //    

// throttle servo related
int32_t gas_pulse_out_us;  // pid loop output to send to the actuator (gas)
int32_t gas_pulse_govern_us;  // Governor must scale the pulse range proportionally. This is given a value in the loop
//  ---- tunable ----
Timer gasServoTimer (500000);  // We expect the servo to find any new position within this time
int32_t gas_governor_percent = 95;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
int32_t gas_pulse_cw_min_us = 1000;  // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
int32_t gas_pulse_redline_us = 1400;  // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
int32_t gas_pulse_idle_us = 1800;  // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
int32_t gas_pulse_ccw_max_us = 2000;  // Servo ccw limit pulsewidth. Hotrc controller ch1/2 min(lt/br) = 1000us, center = 1500us, max(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
int32_t gas_pulse_park_slack_us = 30;  // Gas pulsewidth beyond gas_pulse_idle_us where to park the servo out of the way so we can drive manually (in us)

// tachometer related
volatile int32_t tach_delta_us = 0;
Timer tachPulseTimer;  // OK to not be volatile?
double tach_rpm = 50;  // Current engine speed, raw as sensed (in rpm)
double tach_filt_rpm = 50;  // Current engine speed, filtered (in rpm)
double tach_govern_rpm;  // Create an artificially reduced maximum for the engine speed. This is given a value in the loop
//  ---- tunable ----
double tach_convert_rpm_per_rpus = 60.0 * 1000000.0;  // 1 rot/us * 60 sec/min * 1000000 us/sec = 60000000 rot/min
bool tach_convert_invert = true;
int32_t tach_convert_polarity = SPID::FWD;      
double tach_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double tach_idle_rpm = 700;  // Min value for engine hz, corresponding to low idle (in rpm)
double tach_max_rpm = 6000;  // Max possible engine rotation speed
double tach_redline_rpm = 4000;  // Max value for tach_rpm, pedal to the metal (in rpm)
double tach_margin_rpm = 15;  // Margin of error for checking engine rpm (in rpm)
int32_t tach_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
int32_t tach_delta_abs_min_us = 6500;  // 6500 us corresponds to about 10000 rpm, which isn't possible. Use to reject retriggers

// carspeed/speedo related
volatile int32_t speedo_delta_us = 0;
double carspeed_govern_mmph;  // Governor must scale the top vehicle speed proportionally. This is given a value in the loop
double carspeed_mmph;  // Current car speed, raw as sensed (in mmph)
double carspeed_filt_mmph;  // Current car speed, filtered (in mmph)
Timer speedoPulseTimer;  // OK to not be volatile?
//  ---- tunable ----
double speedo_convert_mmph_per_rpus = 1000000.0 * 3600.0 * 20 * 3.14159 * 1000.0 / (19.85 * 12 * 5280);  // 1 rot/us * 1000000 us/sec * 3600 sec/hr * 1/19.85 gearing * 20*pi in/rot * 1/12 ft/in * 1000/5280 milli-mi/ft = 179757270 milli-mi/hr (mmph)
// Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
bool speedo_convert_invert = true;
int32_t speedo_convert_polarity = SPID::FWD;      
double carspeed_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
double carspeed_idle_mmph = 4500;  // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)
double carspeed_redline_mmph = 15000;  // What is our steady state speed at redline? Pulley rotation frequency (in milli-mph)
double carspeed_max_mmph = 25000;  // What is max speed car can ever go
int32_t speedo_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the car is stopped (in us)
int32_t speedo_delta_abs_min_us = 4500;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers
            
// neopixel related
uint8_t neopixel_wheel_counter = 0;
int8_t neopixel_brightness = 30;
int32_t neopixel_timeout = 150000;
Timer neopixelTimer (neopixel_timeout);

// diag/monitoring variables
Timer loopTimer (1000000);  // how long the previous main loop took to run (in us)
int32_t loop_period_us = 100000;
double loop_freq_hz = 1;  // run loop real time frequency (in Hz)
volatile int32_t int_counter = 0;  // counts interrupts per loop
bool wait_one_loop = false;
bool wait_one_loop_last = false;
int32_t loopno = 1;
uint32_t looptimes_us[20];
bool loop_dirty[20];
int32_t loopindex = 0;
Timer tftResetTimer (100000);
Timer tftDelayTimer (3000000);
int32_t timing_tft_reset = 0; 
Timer heartbeatTimer (1000000);
int32_t heartbeat_state = 0;
int32_t heartbeat_level = 0;
int32_t heartbeat_ekg[4] = { 170000, 150000, 530000, 1100000 };
int32_t heartbeat_pulse = 255;
bool neopixel_heartbeat;
int8_t neopixel_heart_fade = neopixel_brightness;  // brightness during fadeouts
enum neo_colors { N_RED, N_GRN, N_BLU };
uint8_t neopixel_heart_color[3] = { 0xff, 0xff, 0xff };

// pushbutton related
bool button_last = 0;
bool button_it = 0;

// external signal related
bool ignition = LOW;
bool ignition_last = ignition;
bool syspower = HIGH;
bool syspower_last = syspower;
bool basicmodesw = LOW;
bool cruise_sw = LOW;

// touchscreen related
bool touch_now_touched = false;  // Is a touch event in progress
bool touch_longpress_valid = true;
int32_t touch_accel_exponent = 0;  // Will edit values by +/- 2^touch_accel_exponent per touch_period interval
int32_t touch_accel = 1 << touch_accel_exponent;  // Touch acceleration level, which increases the longer you hold. Each edit update chages value by this
int32_t touch_fudge = 0;  // -8
//  ---- tunable ----
int32_t touch_accel_exponent_max = 8;  // Never edit values faster than this. 2^8 = 256 change in value per update
Timer touchPollTimer (35000);  // Timer for regular touchscreen sampling
Timer touchHoldTimer (1000000);  // For timing touch long presses
Timer touchAccelTimer (850000);  // Touch hold time per left shift (doubling) of touch_accel

// rotary encoder related
enum encodersw_presses { NONE, SHORT, LONG };
enum encoder_inputs { A, B };
Timer encoderSpinspeedTimer;  // Used to figure out how fast we're spinning the knob.  OK to not be volatile?
int32_t encoder_state = 0;
int32_t encoder_spinrate_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_spinrate_last_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_spinrate_old_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_edits_per_det = 1;  // How many edits per detent. How much change happens per rotation detent
int32_t encoder_sw_action = NONE;  // Flag for encoder handler to know an encoder switch action needs to be handled
bool encoder_sw = false;  // Remember whether switch is being pressed
bool encoder_timer_active = false;  // Flag to prevent re-handling long presses if the sw is just kept down
bool encoder_suppress_click = false;  // Flag to prevent a short click on switch release after successful long press
bool encoder_b_raw = digitalRead (encoder_b_pin);  // To store value of encoder pin value
bool encoder_a_raw = digitalRead (encoder_a_pin);
volatile bool encoder_a_stable = true;  //  Stores the value of encoder A pin as read during B pin transition (where A is stable)
volatile int32_t encoder_spinrate_isr_us = 100000;  // Time elapsed between last two detents
volatile int32_t encoder_bounce_danger = B;  // Which of the encoder A or B inputs is currently untrustworthy due to bouncing 
volatile int32_t encoder_delta = 0;  // Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 
//  ---- tunable ----
Timer encoderLongPressTimer(800000);  // Used to time long button presses
int32_t encoder_spinrate_min_us = 2500;  // Will reject spins faster than this as an attempt to debounce behavior
int32_t encoder_accel_thresh_us = 100000;  // Spins faster than this will be accelerated
int32_t encoder_accel_max = 50;  // Maximum acceleration factor

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
bool sim_ign = true;
bool sim_cruisesw = true;
bool sim_pressure = false;
bool sim_syspower = true;

// Instantiate objects 
Adafruit_FT6206 touchpanel = Adafruit_FT6206(); // Touch panel
Adafruit_ILI9341 tft = Adafruit_ILI9341 (tft_cs_pin, tft_dc_pin);  // LCD screen

SdFat sd;  // SD card filesystem
#define approot "cantroller2020"
#define logfile "log.txt"
#define error(msg) sd.errorHalt(F(msg))  // Error messages stored in flash.
SdFile root;  // Directory file.
SdFile file;  // Use for file creation in folders.

SPID brakeSPID (brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s, brake_spid_ctrl_dir, pid_period_ms);
SPID gasSPID (gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s, gas_spid_ctrl_dir, pid_period_ms);
SPID cruiseSPID (cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s, cruise_spid_ctrl_dir, pid_period_ms);

// Servo library lets us set pwm outputs given an on-time pulse width in us
static Servo steer_servo;
static Servo brake_servo;
static Servo gas_servo;
static Adafruit_NeoPixel neostrip(1, neopixel_pin, NEO_GRB + NEO_GRB + NEO_KHZ800);

// Temperature sensor related
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
void encoder_a_isr (void) {  // When A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
    if (encoder_bounce_danger != A) {  // Prevents taking action on any re-triggers after a valid trigger due to bouncing
        if (!encoder_a_stable) {  // Since A just transitioned, if a_stable is low, this is a rising edge = time to register a turn 
            encoder_spinrate_isr_us = encoderSpinspeedTimer.elapsed();
            encoderSpinspeedTimer.reset();
            encoder_delta += digitalRead (encoder_b_pin) ? -1 : 1;  // Create turn event to be handled later. If B=0, delta=-1 (CCW) turn decreases things
        }
        encoder_bounce_danger = A;  // Set to reject A retriggers and enable B trigger
    }
}
void encoder_b_isr (void) {  // On B rising or falling edge, A should have stabilized by now, so don't ignore next A transition
    if (encoder_bounce_danger != B) {  // Prevents taking action on any re-triggers after a valid trigger due to bouncing
        encoder_a_stable = digitalRead (encoder_a_pin);  // Input A is stable by the time B changes, so read A value here
        encoder_bounce_danger = B;  // Set to reject B retriggers and enable A trigger
    }
}
// void encoder_a_rise_isr (void) {  // When A goes high if B is low, we are CW, otherwise we are CCW
//     if (encoder_bounce_danger != A) {  // Prevents taking action on any re-triggers after a valid trigger due to bouncing
//         encoder_spinrate_isr_us = encoderSpinspeedTimer.elapsed();
//         encoderSpinspeedTimer.reset();
//         encoder_delta += digitalRead (encoder_b_pin) ? -1 : 1;  // Create turn event to be handled later. If B=0, delta=-1 (CCW) turn decreases things        
//     }  // Input B is stable by the time A changes, so read B value here
//     encoder_bounce_danger = A;  // Set to reject A retriggers and enable B trigger
// }
// void encoder_a_fall_isr (void) {  // This ISR disables A triggering on A falling edge to prevent trigger during the subsequent ringing
//     encoder_bounce_danger = A;  // Set to enable B trigger
// }
// void encoder_b_isr (void) {  // On B rising or falling edge, input A should be stable
//     encoder_bounce_danger = B; // Set to enable A trigger
// }
// The tach and speed use a hall sensor being triggered by a passing magnet once per pulley turn. These ISRs call mycros()
// on every pulse to know the time since the previous pulse. I tested this on the bench up to about 750 mmph which is as 
// fast as I can move the magnet with my hand, and it works. It would be cleaner to just increment a counter here in the ISR
// then call mycros() in the main loop and compare with a timer to calculate mmph.
void tach_isr (void) {  // The tach and speedo isrs compare value returned from the mycros() function with the value from the last interrupt to determine period, to get frequency of the vehicle pulley rotations.
    int32_t temp_us = tachPulseTimer.elapsed();
    if (temp_us > tach_delta_abs_min_us) {
        tachPulseTimer.reset();
        tach_delta_us = temp_us;    
    }
}
void speedo_isr (void) {  //  A better approach would be to read, reset, and restart a hardware interval timer in the isr.  Better still would be to regularly read a hardware binary counter clocked by the pin - no isr.
    int32_t temp_us = speedoPulseTimer.elapsed();
    if (temp_us > speedo_delta_abs_min_us) {
        speedoPulseTimer.reset();
        speedo_delta_us = temp_us;    
    }
}
// void hotrc_vert_rise_isr (void) {  // On rising edge of ch1-vert, this ISR zeros the timer used as reference for all hotrc falling-edge isrs
//     hotrcPulseTimer.reset();
// }
// void hotrc_vert_fall_isr (void) {  //  On falling edge, records high pulse width to determine ch1 trigger position
//     hotrc_vert_pulse_us = hotrcPulseTimer.elapsed();
// }
void hotrc_vert_isr (void) {  // On falling edge, records high pulse width to determine ch2 steering slider position
    if (digitalRead (hotrc_vert_pin)) hotrcPulseTimer.reset();
    else hotrc_vert_pulse_us = hotrcPulseTimer.elapsed();
}
void hotrc_horz_isr (void) {  // On falling edge, records high pulse width to determine ch2 steering slider position
    hotrc_horz_pulse_us = hotrcPulseTimer.elapsed();
}
void hotrc_ch3_isr (void) {  // On falling edge, records high pulse width to determine ch3 button toggle state
    hotrc_ch3_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch3 switch true if short pulse, otherwise false
    if (hotrc_ch3_sw != hotrc_ch3_sw_last) hotrc_ch3_sw_event = true;  // So a handler routine can be signaled
    hotrc_ch3_sw_last = hotrc_ch3_sw;
}
void hotrc_ch4_isr (void) {  // On falling edge, records high pulse width to determine ch4 button toggle state
    hotrc_ch4_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch4 switch true if short pulse, otherwise false
    if (hotrc_ch4_sw != hotrc_ch4_sw_last) hotrc_ch4_sw_event = true;  // So a handler routine can be signaled
    hotrc_ch4_sw_last = hotrc_ch4_sw;
}

inline double max (double a, double b) { return (a > b) ? a : b; }
inline double min (double a, double b) { return (a < b) ? a : b; }
inline double constrain (double amt, double low, double high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline double map (double x, double in_min, double in_max, double out_min, double out_max) {
    if (in_max != in_min) return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return out_max;
}
inline int32_t max (int32_t a, int32_t b) { return (a > b) ? a : b; }
inline int32_t min (int32_t a, int32_t b) { return (a < b) ? a : b; }
inline int32_t constrain (int32_t amt, int32_t low, int32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max != in_min) return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return out_max;
}

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
    // if (!raw) *filt = 0; else
    *filt = (int32_t)(alpha * (double)raw + (1-alpha) * (double)(*filt));
}

// Functions to write to the screen efficiently
//
void draw_bargraph_base (int32_t corner_x, int32_t corner_y, int32_t width) {  // draws a horizontal bargraph scale.  124, y, 40
    tft.drawFastHLine (corner_x+disp_bargraph_squeeze, corner_y, width-disp_bargraph_squeeze*2, GRY1);
    for (int32_t offset=0; offset<=2; offset++) tft.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(width/2 - disp_bargraph_squeeze), corner_y-1, 3, WHT);
}
void draw_needle_shape (int32_t pos_x, int32_t pos_y, int32_t color) {  // draws a cute little pointy needle
    tft.drawFastVLine (pos_x-1, pos_y, 2, color);
    tft.drawFastVLine (pos_x, pos_y, 4, color);
    tft.drawFastVLine (pos_x+1, pos_y, 2, color);
}
void draw_target_shape (int32_t pos_x, int32_t pos_y, int32_t t_color, int32_t r_color) {  // draws a cute little target symbol
    tft.drawFastVLine (pos_x-1, pos_y+7, 2, t_color);
    tft.drawFastVLine (pos_x, pos_y+5, 4, t_color);
    tft.drawFastVLine (pos_x+1, pos_y+7, 2, t_color);
}
void draw_bargraph_needle (int32_t n_pos_x, int32_t old_n_pos_x, int32_t pos_y, int32_t n_color) {  // draws a cute little pointy needle
    draw_needle_shape (old_n_pos_x, pos_y, BLK);
    draw_needle_shape (n_pos_x, pos_y, n_color);
}
// void draw_bargraph_needle_target (int32_t n_pos_x, int32_t old_n_pos_x, int32_t t_pos_x, int32_t old_t_pos_x, int32_t pos_y, int32_t n_color, int32_t t_color, int32_t r_color) {  // draws a needle and target
//     draw_needle_shape (old_n_pos_x, pos_y, BLK);
//     draw_target_shape (old_t_pos_x, pos_y, BLK, BLK);
//     draw_target_shape (t_pos_x, pos_y, t_color, r_color);
//     draw_needle_shape (n_pos_x, pos_y, n_color);
// }  // This function was for when the needle could overlap the target
void draw_string (int32_t x_new, int32_t x_old, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
    tft.setCursor (x_old, y);
    tft.setTextColor (bgcolor);
    tft.print (oldtext);  // Erase the old content
    tft.setCursor (x_new, y);
    tft.setTextColor (color);
    tft.print (text);  // Draw the new content
}
void draw_mmph (int32_t x, int32_t y, int32_t color) {  // This is my cheesy pixel-drawn "mmph" compressed horizontally to 3-char width
    tft.setTextColor (color);
    tft.setCursor (x, y);
    tft.print ("m");
    tft.setCursor (x+4, y);
    tft.print ("m");  // Overlapping 'mm' complete (x = 0-8)
    tft.drawFastVLine (x+10, y+2, 6, color);
    tft.drawPixel (x+11, y+2, color);
    tft.drawPixel (x+11, y+6, color);
    tft.drawFastVLine (x+12, y+3, 3, color);  // 'p' complete (x = 10-12)
    tft.drawFastVLine (x+14, y, 7, color);
    tft.drawPixel (x+15, y+2, color);
    tft.drawFastVLine (x+16, y+3, 4, color);  // 'h' complete (x = 14-16)
}
void draw_thou (int32_t x, int32_t y, int32_t color) {  // This is my cheesy pixel-drawn "thou" compressed horizontally to 3-char width
    tft.drawFastVLine (x+1, y+1, 5, color);
    tft.drawFastHLine (x, y+2, 3, color);
    tft.drawPixel (x+2, y+6, color);  // 't' complete (x = 0-2)
    tft.drawFastVLine (x+4, y, 7, color);
    tft.drawPixel (x+5, y+3, color);
    tft.drawPixel (x+6, y+2, color);
    tft.drawFastVLine (x+7, y+3, 4, color);  // 'h' complete (x = 4-7)
    tft.drawFastVLine (x+9, y+3, 3, color);
    tft.drawFastHLine (x+10, y+2, 2, color);
    tft.drawFastHLine (x+10, y+6, 2, color);
    tft.drawFastVLine (x+12, y+3, 3, color);  // 'o' complete (x = 9-12)
    tft.drawFastVLine (x+14, y+2, 4, color);
    tft.drawPixel (x+15, y+6, color);
    tft.drawFastVLine (x+16, y+2, 5, color);  // 'u' complete (x = 14-16)
}
void draw_string_units (int32_t x, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
    if (!strcmp (oldtext, "mmph")) draw_mmph(x, y, bgcolor);
    else if (!strcmp (oldtext, "thou")) draw_thou (x, y, bgcolor);
    else {
        tft.setCursor (x, y);
        tft.setTextColor (bgcolor);
        tft.print (oldtext);  // Erase the old content
    }
    if (!strcmp (text, "mmph")) draw_mmph(x, y, color);
    else if (!strcmp (text, "thou")) draw_thou (x, y, color);
    else {
        tft.setCursor (x, y);
        tft.setTextColor (color);
        tft.print (text);  // Erase the old content
    }
}
void draw_colons (int32_t x_pos, int32_t first, int32_t last, int32_t color) {
    for (int32_t lineno=first; lineno <= last; lineno++) {
        tft.drawPixel (x_pos, lineno*disp_line_height_pix+3, color);
        tft.drawPixel (x_pos, lineno*disp_line_height_pix+7, color);
        // tft.fillRect (x_pos, (lineno+1)*disp_line_height_pix+3, 2, 2, color);
        // tft.fillRect (x_pos, (lineno+1)*disp_line_height_pix+7, 2, 2, color);
    }
}
// draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
void draw_fixed (int32_t page, int32_t page_last, bool redraw_tuning_corner) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
    tft.setTextColor (GRY2);
    tft.setTextSize (1);
    // if (redraw_tuning_corner) tft.fillRect(10, 145, 154, 95, BLK); // tft.fillRect(0,145,167,95,BLK);  // Erase old dataset page area - This line alone uses 15 ms
    int32_t y_pos;
    if (!redraw_tuning_corner) {
        for (int32_t lineno=0; lineno < disp_fixed_lines; lineno++)  {  // Step thru lines of fixed telemetry data
            y_pos = (lineno+1)*disp_line_height_pix+disp_vshift_pix;
            draw_string (12, 12, y_pos, telemetry[lineno], "", GRY2, BLK);
            draw_string_units (104, y_pos, units[lineno], "", GRY2, BLK);
            draw_bargraph_base (124, y_pos+7, disp_bargraph_width);
        }
        // draw_colons(7+disp_font_width*arraysize(telemetry[0]), 1, disp_fixed_lines+disp_tuning_lines, GRY1);
    }
    for (int32_t lineno=0; lineno < disp_tuning_lines; lineno++)  {  // Step thru lines of dataset page data
        draw_string(12, 12, (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[page][lineno], dataset_page_names[page_last][lineno], GRY2, BLK);
        draw_string_units(104, (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix, tuneunits[page][lineno], tuneunits[page_last][lineno], GRY2, BLK);
        if (redraw_tuning_corner) {
            int32_t corner_y = (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix+7;  // lineno*disp_line_height_pix+disp_vshift_pix-1;
            draw_bargraph_base (124, corner_y, disp_bargraph_width);
            if (disp_needles[lineno] >= 0) draw_bargraph_needle (-1, disp_needles[lineno], corner_y-6, BLK);  // Let's draw a needle
        }
    }
}
// Font character \xfa is little 3-pixel wide hyphen, use for negative numbers 
void draw_hyphen (int32_t x_pos, int32_t y_pos, int32_t color) {
    tft.drawFastHLine (x_pos+2, y_pos+3, 3, color);
}
void draw_dynamic (int32_t lineno, char const* disp_string, int32_t value, int32_t lowlim, int32_t hilim, int32_t target) {
    int32_t age_us = (int32_t)((double)(dispAgeTimer[lineno].elapsed()) / 2500000); // Divide by us per color gradient quantum
    int32_t x_base = 59;
    bool polarity = (value >= 0);  // polarity 0=negative, 1=positive
    if (strcmp(disp_values[lineno], disp_string) || disp_redraw_all) {  // If value differs, Erase old value and write new
        int32_t y_pos = lineno*disp_line_height_pix+disp_vshift_pix;
        if (polarity != disp_polarities[lineno]) draw_hyphen (x_base, y_pos, (!polarity) ? GRN : BLK);
        draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_string, disp_values[lineno], GRN, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        strcpy (disp_values[lineno], disp_string);
        disp_polarities[lineno] = polarity;
        dispAgeTimer[lineno].reset();
        disp_age_quanta[lineno] = 0;
    }
    else if (age_us > disp_age_quanta[lineno] && age_us < 11)  {  // As readings age, redraw in new color. This may fail and redraw when the timer overflows? 
        int32_t color;
        if (age_us < 8) color = 0x1fe0 + age_us*0x2000;  // Base of green with red added as you age, until yellow is achieved
        else color = 0xffe0 - (age_us-8) * 0x100;  // Then lose green as you age further
        int32_t y_pos = (lineno)*disp_line_height_pix+disp_vshift_pix;
        if (!polarity) draw_hyphen (x_base, y_pos, color);
        draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_values[lineno], "", color, BLK);
        disp_age_quanta[lineno] = age_us;
    }
    if (lowlim < hilim) {  // Any value having a given range deserves a bargraph gauge with a needle
        int32_t corner_x = 124;    
        int32_t corner_y = lineno*disp_line_height_pix+disp_vshift_pix-1;
        int32_t n_pos = map (value, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
        int32_t ncolor = (n_pos > disp_bargraph_width-disp_bargraph_squeeze || n_pos < disp_bargraph_squeeze) ? DORG : GRN;
        n_pos = corner_x + constrain (n_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
        if (target != -1) {  // If target value is given, draw a target on the bargraph too
            int32_t t_pos = map (target, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
            int32_t tcolor = (t_pos > disp_bargraph_width-disp_bargraph_squeeze || t_pos < disp_bargraph_squeeze) ? DORG : ( (t_pos != n_pos) ? YEL : GRN );
            t_pos = corner_x + constrain (t_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
            if (t_pos != disp_targets[lineno] || (t_pos == n_pos)^(disp_needles[lineno] != disp_targets[lineno]) || disp_redraw_all) {
                draw_target_shape (disp_targets[lineno], corner_y, BLK, -1);  // Erase old target
                tft.drawFastHLine (disp_targets[lineno]-(disp_targets[lineno] != corner_x+disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+7, 2+(disp_targets[lineno] != corner_x+disp_bargraph_width-disp_bargraph_squeeze), GRY1);  // Patch bargraph line where old target got erased
                for (int32_t offset=0; offset<=2; offset++) tft.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(disp_bargraph_width/2 - disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+6, 3, WHT);  // Redraw bargraph graduations in case one got corrupted by target erasure
                draw_target_shape (t_pos, corner_y, tcolor, -1);  // Draw the new target
                disp_targets[lineno] = t_pos;  // Remember position of target
            }
        }
        if (n_pos != disp_needles[lineno] || disp_redraw_all) {
            draw_bargraph_needle (n_pos, disp_needles[lineno], corner_y, ncolor);  // Let's draw a needle
            disp_needles[lineno] = n_pos;  // Remember position of needle
        }
    }
    else if (disp_needles[lineno] >= 0) {  // If value having no range is drawn over one that did ...
        draw_bargraph_needle (-1, disp_needles[lineno], lineno*disp_line_height_pix+disp_vshift_pix-1, BLK);  // Erase the old needle
        disp_needles[lineno] = -1;  // Flag for no needle
    }
}
int32_t significant_place (double value) {
    int32_t place = 0;
    if (value >= 1) { // int32_t vallog = std::log10(value);  // Can be sped up
        place = 1;
        while (value >= 10) {
            value /= 10;
            place++;
        }
    }
    else if (value) {  // checking (value) rather than (value != 0.0) can help avoid precision errors caused by digital representation of floating numbers
        while (value < 1) {
            value *= 10;
            place--;
        }
    }
    return place;
}
std::string abs_itoa (int32_t value, int32_t maxlength) {
    value = abs (value);  // This function disregards sign
    if (significant_place(value) <= maxlength) return std::to_string (value);  // If value is short enough, return it
    std::string result;
    int32_t magnitude = std::log10 (value);
    double scaledValue = value / std::pow (10, magnitude + 1 - maxlength);  // was (10, magnitude - 5);
    if (scaledValue >= 1.0 && scaledValue < 10.0) result = std::to_string (static_cast<int>(scaledValue));
    else result = std::to_string (scaledValue);
    if (magnitude >= maxlength) result += "e" + std::to_string (magnitude);
    return result;
}
std::string abs_ftoa (double value, int32_t maxlength, int32_t sigdig) {  // sigdig limits number of significant digits  
    value = abs (value);  // This function disregards sign
    int32_t place = significant_place (value);  // Learn decimal place of the most significant digit in value
    if (place >= sigdig && place <= maxlength) {  // Then we want simple cast to an integer w/o decimal point (eg 123456, 12345, 1234)
        std::string result (std::to_string ((int32_t)value));
        return result;
    }
    if (place >= 0 && place < maxlength) {  // Then we want float formatted with enough nonzero digits after the decimal point for 4 significant digits (eg 123.4, 12.34, 1.234, 0)
        int32_t length = min (sigdig+1, maxlength);
        char buffer[length+1];
        std::snprintf (buffer, length + 1, "%.*f", length - 1, value);
        std::string result (buffer);  // copy buffer to result
        return result;
    }
    if (place >= 3-maxlength && place < maxlength) {  // Then we want decimal w/o initial '0' limited to 3 significant digits (eg .123, .0123, .00123)
        std::string result (std::to_string(value));
        size_t decimalPos = result.find('.');  // Remove any digits to the left of the decimal point
        if (decimalPos != std::string::npos) result = result.substr(decimalPos);
        if (result.length() > sigdig) result.resize(sigdig);  // Limit the string length to the desired number of significant digits
        return result;
    }  // Otherwise we want scientific notation with precision removed as needed to respect maxlength (eg 1.23e4, 1.23e5, but using long e character not e for negative exponents
    char buffer[maxlength+1];  // Allocate buffer with the maximum required size
    snprintf(buffer, sizeof(buffer), "%.*e", min (sigdig, maxlength-4), value);
    std::string result (buffer);  // copy buffer to result
    if (result.find ("e+0") != std::string::npos) result.replace (result.find ("e+0"), 3, "e");
    else if (result.find ("e-0") != std::string::npos) result.replace (result.find ("e-0"), 3, "\x88");  // Font character \x88 is phoenetic long e, using for exponent to negative power  // if (result.find ("e-0") != std::string::npos) 
    return result;    
}
void draw_dynamic (int32_t lineno, int32_t value, int32_t lowlim, int32_t hilim, int target) {
    std::string val_string = abs_itoa (value, (int32_t)disp_maxlength);
    // std::cout << "Int: " << value << " -> " << val_string << ", " << ((value >= 0) ? 1 : -1) << std::endl;
    draw_dynamic (lineno, val_string.c_str(), value, lowlim, hilim, (int32_t)target);
}
void draw_dynamic (int32_t lineno, int32_t value, int32_t lowlim, int32_t hilim) {
    draw_dynamic (lineno, value, lowlim, hilim, -1);
}
void draw_dynamic (int32_t lineno, double value, double lowlim, double hilim, int32_t target) {
    std::string val_string = abs_ftoa (value, (int32_t)disp_maxlength, 4);
    // std::cout << "Flt: " << value << " -> " << val_string << ", " << ((value >= 0) ? 1 : -1) << std::endl;
    draw_dynamic (lineno, val_string.c_str(), (int32_t)value, (int32_t)lowlim, (int32_t)hilim, target);
}
void draw_dynamic (int32_t lineno, double value, double lowlim, double hilim) {
    draw_dynamic (lineno, value, lowlim, hilim, -1);
}
void draw_runmode (int32_t runmode, int32_t oldmode, int32_t color_override) {  // color_override = -1 uses default color
    int32_t color = (color_override == -1) ? colorcard[runmode] : color_override;
    int32_t x_new = 8+6*(2+strlen (modecard[runmode]))-3;
    int32_t x_old = 8+6*(2+strlen (modecard[oldmode]))-3;
    draw_string (8+6, 8+6, disp_vshift_pix, modecard[oldmode], "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
    draw_string (x_old, x_old, disp_vshift_pix, "Mode", "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
    draw_string (8+6, 8+6, disp_vshift_pix, modecard[runmode], "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
    draw_string (x_new, x_new, disp_vshift_pix, "Mode", "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
}
void draw_dataset_page (int32_t page, int32_t page_last) {
    draw_fixed (page, page_last, true);  // Erase and redraw dynamic data corner of screen with names, units etc.
    // for (int32_t lineno=0; lineno<disp_lines; lineno++) draw_hyphen (59, lineno*disp_line_height_pix+disp_vshift_pix, BLK);
    draw_string (83, 83, disp_vshift_pix, pagecard[page], pagecard[page_last], RBLU, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
}
void draw_selected_name (int32_t tun_ctrl, int32_t tun_ctrl_last, int32_t selected_val, int32_t selected_last) {
    if (selected_val != selected_last) draw_string (12, 12, 12+(selected_last+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_last], "", GRY2, BLK);
    draw_string (12, 12, 12+(selected_val+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_val], "", (tun_ctrl == EDIT) ? GRN : ((tun_ctrl == SELECT) ? YEL : GRY2), BLK);
}
void draw_bool (bool value, int32_t col) {  // Draws values of boolean data
    if ((disp_bool_values[col-2] != value) || disp_redraw_all) {  // If value differs, Erase old value and write new
        int32_t x_mod = touch_margin_h_pix + touch_cell_h_pix*(col) + (touch_cell_h_pix>>1) - arraysize (top_menu_buttons[col-2]-1)*(disp_font_width>>1) - 2;
        draw_string (x_mod, x_mod, 0, top_menu_buttons[col-2], "", (value) ? GRN : LGRY, DGRY);
        disp_bool_values[col-2] = value;
    }
}
void draw_simbuttons (bool create) {  // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
    tft.setTextColor (LYEL);
    for (int32_t row = 0; row < arraysize(simgrid); row++) {
        for (int32_t col = 0; col < arraysize(simgrid[row]); col++) {
            int32_t cntr_x = touch_margin_h_pix + touch_cell_h_pix*(col+3) + (touch_cell_h_pix>>1) +2;
            int32_t cntr_y = touch_cell_v_pix*(row+1) + (touch_cell_v_pix>>1);
            if (strcmp (simgrid[row][col], "    " )) {
                tft.fillCircle (cntr_x, cntr_y, 19, create ? DGRY : BLK);
                if (create) {
                    tft.drawCircle (cntr_x, cntr_y, 19, LYEL);
                    int32_t x_mod = cntr_x-(arraysize (simgrid[row][col])-1)*(disp_font_width>>1);
                    draw_string (x_mod, x_mod, cntr_y-(disp_font_height>>1), simgrid[row][col], "", LYEL, DGRY);
                }
            }
        }     
    }
}
void draw_touchgrid (bool side_only) {  // draws edge buttons with names in 'em. If replace_names, just updates names
    int32_t namelen = 0;
    tft.setTextColor (WHT);
    for (int32_t row = 0; row < arraysize (side_menu_buttons); row++) {  // Step thru all rows to draw buttons along the left edge
        tft.fillRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, DGRY);
        tft.drawRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, LYEL);
        namelen = 0;
        for (uint32_t x = 0 ; x < arraysize (side_menu_buttons[row]) ; x++ ) {
            if (side_menu_buttons[row][x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
        }
        for (int32_t letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
            tft.setCursor (1, ( touch_cell_v_pix*row) + (touch_cell_v_pix/2) - (int32_t)(4.5*((double)namelen-1)) + (disp_font_height+1)*letter); // adjusts vertical offset depending how many letters in the button name and which letter we're on
            tft.println (side_menu_buttons[row][letter]);  // Writes each letter such that the whole name is centered vertically on the button
        }
    }
    if (!side_only) {
        for (int32_t col = 2; col <= 5; col++) {  // Step thru all cols to draw buttons across the top edge
            tft.fillRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
            tft.drawRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, LYEL);  // tft.width()-9, 3, 18, (tft.height()/5)-6, 8, LYEL);
            // draw_bool (top_menu_buttons[btn], btn+3);
        }
    }
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
void adj_val (int32_t* variable, int32_t modify, int32_t low_limit, int32_t high_limit) {  // sets an int reference to new val constrained to given range
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
}
void adj_val (double* variable, int32_t modify, double low_limit, double high_limit) {  // sets an int reference to new val constrained to given range
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
}

void adj_bool (bool* val, int32_t delta) { if (delta != 0) *val = (delta > 0); }  // sets a bool reference to 1 on 1 delta or 0 on -1 delta 

void set_pin (int32_t pin, int32_t mode) {  // configures a pin on the condition it exists for the current board
    if (pin >= 0) pinMode (pin, mode);
}
void write_pin (int32_t pin, int32_t val) {  // writes a digital value to a pin on the condition it exists for the current board
    if (pin >= 0) digitalWrite (pin, val);
}
int32_t read_pin (int32_t pin) {  // reads a digital value from a pin on the condition it exists for the current board
    if (pin >= 0) return digitalRead (pin);
    return -1;
}
void syspower_set (bool val) {
    if (digitalRead (syspower_pin) != val) {
        write_pin (syspower_pin, val);
        // delay (val * 500);
    }
}

// double get_temp (DeviceAddress arg_addr) {  // function to print the temperature for a device
//     float tempF = tempsensebus.getTempF (arg_addr);
//     // if (tempF == DEVICE_DISCONNECTED_C) printf ("Error: Could not read temperature\n");
//     return tempF;
// }

double convert_units (double from_units, double convert_factor, bool invert) {
    return ((invert) ? 1/from_units : from_units) * convert_factor;
}

void tft_init (void) {
    if (display_enabled) {
        printf ("Init LCD... ");
        // delay (500); // This is needed to allow the screen board enough time after a cold boot before we start trying to talk to it.
        tft.begin();
        tft.setRotation (1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
        for (int32_t lineno=0; lineno <= disp_fixed_lines; lineno++)  {
            disp_age_quanta[lineno] = -1;
            memset (disp_values[lineno],0,strlen (disp_values[lineno]));
            disp_polarities[lineno] = 1;
        }
        for (int32_t row=0; row<arraysize (disp_bool_values); row++) disp_bool_values[row] = 1;
        for (int32_t row=0; row<arraysize (disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
        for (int32_t row=0; row<arraysize (disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen
        tft.fillScreen (BLK);  // Black out the whole screen
        draw_fixed (dataset_page, dataset_page_last, false);
        draw_touchgrid (false);
        printf ("Success.\nCaptouch initialization... ");
        if (! touchpanel.begin(40)) printf ("Couldn't start FT6206 touchscreen controller");  // pass in 'sensitivity' coefficient
        else printf ("Capacitive touchscreen started\n");
    }
}
void tft_watchdog (void) {
    if (display_enabled) {
        if (loop_period_us > 70000 && timing_tft_reset == 0) timing_tft_reset = 1;
        if (timing_tft_reset == 0) tftDelayTimer.reset();
        else if (!tftDelayTimer.expired()) tftResetTimer.reset();
        else if (timing_tft_reset == 1) {
            write_pin (tft_rst_pin, LOW);
            timing_tft_reset = 2;
        }
        else if (timing_tft_reset == 2 && tftResetTimer.expired()) {
            write_pin (tft_rst_pin, HIGH);
            tft_init();
            timing_tft_reset = 0;
        }
    }
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