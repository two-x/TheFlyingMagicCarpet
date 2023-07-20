/* Contains code for the LCD touchscreen */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_ILI9341.h>
#include "globals.h"

#ifdef CAP_TOUCH
    #include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
    Adafruit_FT6206 ts;  // 2.8in cap touch panel on tft lcd
    extern bool cap_touch;
#else
    #include <XPT2046_Touchscreen.h>
    extern XPT2046_Touchscreen ts;  // 3.2in resistive touch panel on tft lcd
    //extern XPT2046_Touchscreen ts (touch_cs_pin);  // 3.2in resistive touch panel on tft lcd
    extern bool cap_touch;
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

enum dataset_pages { PG_RUN, PG_JOY, PG_CAR, PG_PWMS, PG_BPID, PG_GPID, PG_CPID, PG_TEMP };

extern const char pagecard[8][5];
extern const char modecard[7][7];  
extern const int32_t colorcard[7];
extern const char telemetry[8][9];
extern const char dataset_page_names[8][11][9]; 
extern const int32_t tuning_first_editable_line[11];
extern const char units[8][5];
extern const char tuneunits[8][11][5];
extern const char simgrid[4][3][5];

#define disp_fixed_lines 8
#define disp_tuning_lines 11

extern char side_menu_buttons[5][4];  
extern char top_menu_buttons[4][6];
extern char disp_values[disp_lines][disp_maxlength+1];
extern bool disp_polarities[disp_lines];  
extern bool display_enabled;
extern bool disp_bool_values[6];
extern bool disp_selected_val_dirty;
extern bool disp_dataset_page_dirty;
extern bool disp_sidemenu_dirty;
extern bool disp_runmode_dirty;
extern int32_t disp_needles[disp_lines];
extern int32_t disp_targets[disp_lines];
extern int32_t disp_age_quanta[disp_lines];
extern Timer dispAgeTimer[disp_lines];
extern Timer dispRefreshTimer;
extern Timer dispResetButtonTimer;
extern uint32_t tft_watchdog_timeout_us;

enum disp_draw { ERASE = -1 };
enum tuning_ctrl_states { OFF, SELECT, EDIT };
extern int32_t tuning_ctrl;
extern int32_t tuning_ctrl_last;
extern int32_t dataset_page;
extern int32_t dataset_page_last;  
extern int32_t selected_value;
extern int32_t selected_value_last;
extern Timer tuningCtrlTimer;

// Booleans
extern bool touch_now_touched;  
extern bool touch_longpress_valid;

// Integers
extern int32_t touch_accel_exponent;
extern int32_t touch_accel;
extern int32_t touch_fudge;
extern int32_t touch_accel_exponent_max;

// Timers  
extern Timer touchHoldTimer;
extern Timer touchAccelTimer;

// Integer arrays
extern int32_t shutdown_color;

#endif // DISPLAY_H