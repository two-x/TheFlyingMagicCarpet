/* Contains code for the LCD touchscreen */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_ILI9341.h>
#include "globals.h"

#ifdef CAP_TOUCH
    #include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
    Adafruit_FT6206 ts;  // 2.8in cap touch panel on tft lcd
    bool cap_touch = true;
#else
    #include <XPT2046_Touchscreen.h>
    XPT2046_Touchscreen ts (touch_cs_pin, touch_irq_pin);  // 3.2in resistive touch panel on tft lcd
    // XPT2046_Touchscreen ts (touch_cs_pin);  // 3.2in resistive touch panel on tft lcd
    bool cap_touch = false;
#endif // CAP_TOUCH

class Display
{
private:
    Adafruit_ILI9341 _tft;
    Timer _tftResetTimer = Timer(100000);
    Timer _tftDelayTimer = Timer(3000000);
    int32_t _timing_tft_reset = 0;
    bool _procrastinate = false;
    bool reset_finished = false;
    bool _disp_redraw_all = true;

public:
    Display(int8_t cs_pin, int8_t dc_pin);
    void init();
    bool tft_reset(); // call to begin a tft reset, and continue to call every loop until returns true (or get_reset_finished() returns true), then stop
    void watchdog();  // Call in every loop to perform a reset upon detection of blocked loops and
    bool get_reset_finished();
    void draw_bargraph_base(int32_t corner_x, int32_t corner_y, int32_t width); // draws a horizontal bargraph scale.  124, y, 40
    void draw_needle_shape(int32_t pos_x, int32_t pos_y, int32_t color);        // draws a cute little pointy needle
    void draw_target_shape(int32_t pos_x, int32_t pos_y, int32_t t_color, int32_t r_color);
    void draw_bargraph_needle(int32_t n_pos_x, int32_t old_n_pos_x, int32_t pos_y, int32_t n_color);
    void draw_string(int32_t x_new, int32_t x_old, int32_t y, const char *text, const char *oldtext,
                     int32_t color, int32_t bgcolor, bool forced = false);
    void draw_mmph(int32_t x, int32_t y, int32_t color);
    void draw_thou(int32_t x, int32_t y, int32_t color);
    void draw_string_units(int32_t x, int32_t y, const char *text, const char *oldtext,
                           int32_t color, int32_t bgcolor);
    void draw_colons(int32_t x_pos, int32_t first, int32_t last, int32_t color);
    void draw_fixed(int32_t page, int32_t page_last, bool redraw_tuning_corner, bool forced = false);
    void draw_hyphen(int32_t x_pos, int32_t y_pos, int32_t color);
    void draw_dynamic(int32_t lineno, char const *disp_string, int32_t value, int32_t lowlim, int32_t hilim, int32_t target = -1);
    int32_t significant_place(float value);
    std::string abs_itoa(int32_t value, int32_t maxlength);
    std::string abs_ftoa(float value, int32_t maxlength, int32_t sigdig);
    void draw_dynamic(int32_t lineno, int32_t value, int32_t lowlim, int32_t hilim, int32_t target = -1);
    void draw_dynamic(int32_t lineno, float value, float lowlim, float hilim, int32_t target = -1);
    void draw_dynamic(int32_t lineno, float value, float lowlim, float hilim, float target);
    void draw_dynamic(int32_t lineno, int32_t erasure);
    void draw_runmode(int32_t runmode, int32_t oldmode, int32_t color_override = -1);
    void draw_dataset_page(int32_t page, int32_t page_last, bool forced = false);
    void draw_selected_name(int32_t tun_ctrl, int32_t tun_ctrl_last, int32_t selected_val, int32_t selected_last);
    void draw_bool(bool value, int32_t col);
    void draw_simbuttons(bool create);
    void draw_touchgrid(bool side_only);
    void update();
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
#define DORG 0xfa40  // Dark orange aka brown
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

// LCD supports 18-bit color, but GFX library uses 16-bit color, organized (MSB) 5b-red, 6b-green, 5b-blue (LSB)
// Since the RGB don't line up with the nibble boundaries, it's tricky to quantify a color, here are some colors:
// Color picker websites: http://www.barth-dev.de/online/rgb565 , https://chrishewett.com/blog/true-rgb565-colour-picker/
// LCD is 2.8in diagonal, 240x320 pixels

#define disp_width_pix 320  // Horizontal resolution in pixels (held landscape)
#define disp_height_pix 240  // Vertical resolution in pixels (held landscape)
#define disp_lines 20  // Max lines of text displayable at line height = disp_line_height_pix
#define disp_fixed_lines 8  // Lines of static variables/values always displayed
#define disp_tuning_lines 11  // Lines of dynamic variables/values in dataset pages 
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

// string* pagecard = new string[8];  // How we might allocate on the heap instead of in the stack
// string* modecard = new string[7];

char pagecard[8][5] = { "Run ", "Joy ", "Car ", "PWMs", "Bpid", "Gpid", "Cpid", "Temp" };
char modecard[7][7] = { "Basic", "Shutdn", "Stall", "Hold", "Fly", "Cruise", "Cal" };
int32_t colorcard[arraysize(modecard)] = { MGT, RED, ORG, YEL, GRN, CYN, MBLU };
enum dataset_pages { PG_RUN, PG_JOY, PG_CAR, PG_PWMS, PG_BPID, PG_GPID, PG_CPID, PG_TEMP };

char telemetry[disp_fixed_lines][9] = {  
    "Joy Vert",
    "   Speed",
    "    Tach",
    " Gas PWM",
    "Brk Pres",   
    " Brk PWM",
    "Joy Horz",
    "SteerPWM",
}; 
char dataset_page_names[arraysize(pagecard)][disp_tuning_lines][9] = {
    {   " Airflow",  // PG_RUN
        " Brk Pos",
        " Battery",
        "     Pot",
        "SimAirFl",
        "SimBkPos",
        " Sim Joy",
        "Sim Pres",
        "Sim Tach",
        "SimSpeed",
        "SimW/Pot", },
    {   "Horz Raw",  // PG_JOY
        "Vert Raw",
        "HRC Horz", 
        "HRC Vert",
        "HNoRadio",
        "Horz Min",
        "Horz Max",
        " Horz DB",
        "Vert Min",
        "Vert Max",
        " Vert DB", },
    {   "Pres ADC",  // PG_CAR
        "      - ",
        "      - ",
        "Governor",
        "Str Safe",
        "AirFlMax",
        "Eng Idle",
        "Eng RedL",
        "Spd Idle",
        "Spd RedL",
        "BrkPos0P", },
    {   "      - ",  // PG_PWMS
        "      - ",
        "      - ",
        "Steer Lt",
        "SteerStp",
        "Steer Rt",
        "Brk Extd",
        "Brk Stop",
        "Brk Retr",
        "Gas Idle",
        "Gas RdLn", },
    {   "Pres Tgt",  // PG_BPID
        "Pres Err",
        "  P Term",
        "  I Term",
        "  D Term",
        "Integral",
        "      - ",
        "      - ",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)", },
    {   "Tach Tgt",  // PG_GPID
        "Tach Err",
        "  P Term",
        "  I Term",
        "  D Term",
        "Integral",
        "      - ",
        "OpenLoop",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)" },
    {   "SpeedTgt",  // PG_CPID
        "SpeedErr",
        "  P Term",
        "  I Term",
        "  D Term",
        "Integral",
        "Tach Tgt",
        "      - ",
        "  Kp (P)",
        "  Ki (I)",
        "  Kd (D)", },
    {   "Temp Amb",  // PG_TEMP
        "Temp Eng",
        "TempWhFL",
        "TempWhFR",
        "TempWhRL",
        "TempWhRR",
        "      - ",
        "      - ",
        "      - ",
        " Cal Brk",
        " Cal Gas", },
};
int32_t tuning_first_editable_line[disp_tuning_lines] = { 4, 4, 3, 3, 8, 7, 8, 9 };  // first value in each dataset page that's editable. All values after this must also be editable
char units[disp_fixed_lines][5] = { "adc ", "mph ", "rpm ", "\xe5s  ", "psi ", "\xe5s  ", "adc ", "\xe5s  " };

char tuneunits[arraysize(pagecard)][disp_tuning_lines][5] = {
    { "mph ", "in  ", "V   ", "%   ", "    ", "    ", "    ", "    ", "    ", "    ", "    " },  // PG_RUN
    { "adc ", "adc ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc " },  // PG_JOY
    { "adc ", "    ", "    ", "%   ", "%   ", "mph ", "rpm ", "rpm ", "mph ", "mph ", "in  " },  // PG_CAR
    { "adc ", "    ", "    ", "    ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  " },  // PG_PWMS
    { "psi ", "psi ", "psi ", "psi ", "psi ", "psi ", "    ", "    ", "    ", "Hz  ", "s " },  // PG_BPID
    { "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "    ", "    ", "    ", "Hz  ", "s " },  // PG_GPID
    { "mph ", "mph ", "mph ", "mph ", "mph ", "mph ", "rpm ", "    ", "    ", "Hz  ", "s " },  // PG_CPID
    { "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "    ", "    ", "    ", "    ", "    " },  // PG_TEMP
};
char simgrid[4][3][5] = {
    { "prs\x18", "rpm\x18", "car\x18" },
    { "prs\x19", "rpm\x19", "car\x19" },
    { "    ", " \x1e  ", "    " },
    { " \x11  ", " \x1f  ", "  \x10 " },  // Font special characters map:  https://learn.adafruit.com/assets/103682
};
char side_menu_buttons[5][4] = { "PAG", "SEL", "+  ", "-  ", "SIM" };  // Pad shorter names with spaces on the right
char top_menu_buttons[4][6] = { " CAL ", "BASIC", " IGN ", "POWER" };  // Pad shorter names with spaces to center
char disp_values[disp_lines][disp_maxlength+1];  // Holds previously drawn value strings for each line
bool disp_polarities[disp_lines];  // Holds sign of previously drawn values
bool display_enabled = true;  // Should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool disp_bool_values[6];
bool disp_selected_val_dirty = true;
bool disp_dataset_page_dirty = true;
bool disp_sidemenu_dirty = true;
bool disp_runmode_dirty = true;
int32_t disp_needles[disp_lines];
int32_t disp_targets[disp_lines];
int32_t disp_age_quanta[disp_lines];
Timer dispAgeTimer[disp_lines];  // int32_t disp_age_timer_us[disp_lines];
Timer dispRefreshTimer (100000);  // Don't refresh screen faster than this (16667us = 60fps, 33333us = 30fps, 66666us = 15fps)
Timer dispResetButtonTimer (500000);  // How long to press esp32 "boot" button before screen will reset and redraw
uint32_t tft_watchdog_timeout_us = 100000;

// tuning-ui related globals
enum disp_draw { ERASE = -1 };
enum tuning_ctrl_states { OFF, SELECT, EDIT };
int32_t tuning_ctrl = OFF;
int32_t tuning_ctrl_last = OFF;
int32_t dataset_page = PG_RUN;  // Which of the six 8-value dataset pages is currently displayed, and available to edit
int32_t dataset_page_last = PG_TEMP;
int32_t selected_value = 0;  // In the real time tuning UI, which of the editable values (0-7) is selected. -1 for none 
int32_t selected_value_last = 0;
//  ---- tunable ----
Timer tuningCtrlTimer (25000000);  // This times out edit mode after a a long period of inactivity

// touchscreen related
bool touch_now_touched = false;  // Is a touch event in progress
bool touch_longpress_valid = true;
int32_t touch_accel_exponent = 0;  // Will edit values by +/- 2^touch_accel_exponent per touch_period interval
int32_t touch_accel = 1 << touch_accel_exponent;  // Touch acceleration level, which increases the longer you hold. Each edit update chages value by this
int32_t touch_fudge = 0;  // -8
//  ---- tunable ----
int32_t touch_accel_exponent_max = 8;  // Never edit values faster than this. 2^8 = 256 change in value per update
// Timer touchPollTimer (35000);  // Timer for regular touchscreen sampling
Timer touchHoldTimer (800000);  // For timing touch long presses
Timer touchAccelTimer (850000);  // Touch hold time per left shift (doubling) of touch_accel

// run state globals
int32_t shutdown_color = colorcard[SHUTDOWN];

#endif // DISPLAY_H