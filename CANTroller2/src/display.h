/* Contains code for the LCD touchscreen */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
#include <Adafruit_ILI9341.h>  // For interfacing with the TFT LCD controller chip
#include "globals.h"

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

// string* pagecard = new string[8];  // How we might allocate on the heap instead of in the stack
// string* modecard = new string[7];

char pagecard[8][5] = { "Run ", "Joy ", "Car ", "PWMs", "Bpid", "Gpid", "Cpid", "Temp" };
char modecard[7][7] = { "Basic", "Shutdn", "Stall", "Hold", "Fly", "Cruise", "Cal" };
int32_t colorcard[arraysize(modecard)] = { MGT, RED, ORG, YEL, GRN, CYN, MBLU };
enum dataset_pages { RUN, JOY, CAR, PWMS, BPID, GPID, CPID, TEMP };

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
int32_t tuning_first_editable_line[disp_tuning_lines] = { 3, 2, 0, 0, 5, 5, 5, 6 };  // first value in each dataset page that's editable. All values after this must also be editable
char units[disp_fixed_lines][5] = { "mph ", "rpm ", "psi ", "adc ", "adc ", "mph ", "psi ", "rpm ", "\xe5s  ", "\xe5s  ", "\xe5s  " };
char tuneunits[arraysize(pagecard)][disp_tuning_lines][5] = {
    { "V   ", "in  ", "%   ", "    ", "    ", "    ", "    ", "    " },  // RUN
    { "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc ", "adc " },  // JOY
    { "%   ", "rpm ", "rpm ", "mph ", "mph ", "    ", "    ", "    " },  // CAR
    { "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  ", "\xe5s  " },  // PWM
    { "psi ", "psi ", "psi ", "psi ", "psi ", "    ", "Hz  ", "sec " },  // BPID
    { "mph ", "mph ", "mph ", "mph ", "mph ", "    ", "Hz  ", "sec " },  // GPID
    { "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "    ", "Hz  ", "sec " },  // CPID
    { "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "\x09""F  ", "    ", "in  " },  // TEMP
    // { "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "\x09 F ", "    ", "    " },  // TEMP
};
char simgrid[4][3][5] = {
    { "prs\x18", "rpm\x18", "car\x18" },
    { "prs\x19", "rpm\x19", "car\x19" },
    { "    ", " \x1e  ", "    " },
    { " \x11  ", " \x1f  ", "  \x10 " },  // Font special characters map:  https://learn.adafruit.com/assets/103682
};
char side_menu_buttons[5][4] = { "PAG", "SEL", "\x18  ", "\x19  ", "SIM" };  // Pad shorter names with spaces on the right
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
uint32_t tft_watchdog_timeout = 100000;

// tuning-ui related globals
enum tuning_ctrl_states { OFF, SELECT, EDIT };
int32_t tuning_ctrl = OFF;
int32_t tuning_ctrl_last = OFF;
int32_t dataset_page = RUN;  // Which of the six 8-value dataset pages is currently displayed, and available to edit
int32_t dataset_page_last = TEMP;
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
Timer touchPollTimer (35000);  // Timer for regular touchscreen sampling
Timer touchHoldTimer (1000000);  // For timing touch long presses
Timer touchAccelTimer (850000);  // Touch hold time per left shift (doubling) of touch_accel

// run state globals
int32_t shutdown_color = colorcard[SHUTDOWN];

class Display {
    private:
        Adafruit_ILI9341 _tft; // LCD screen
        Timer _tftResetTimer;
        Timer _tftDelayTimer;
        int32_t _timing_tft_reset;
        bool _procrastinate = false;
        bool _disp_redraw_all = true;
    public:
        Adafruit_FT6206 touchpanel; // Touch panel

        Display(int8_t cs_pin, int8_t dc_pin) : _tft(cs_pin, dc_pin), _tftResetTimer(100000), _tftDelayTimer(3000000), _timing_tft_reset(0){}

        void init() {
            printf ("Init LCD... ");
            _tft.begin();
            _tft.setRotation(1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
            for (int32_t lineno=0; lineno <= disp_fixed_lines; lineno++)  {
                disp_age_quanta[lineno] = -1;
                memset (disp_values[lineno], 0, strlen (disp_values[lineno]));
                disp_polarities[lineno] = 1;
            }
            for (int32_t row=0; row<arraysize (disp_bool_values); row++) disp_bool_values[row] = 1;
            for (int32_t row=0; row<arraysize (disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
            for (int32_t row=0; row<arraysize (disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen
            _tft.fillScreen (BLK);  // Black out the whole screen
            draw_touchgrid (false);
            draw_fixed (dataset_page, dataset_page_last, false);
            _disp_redraw_all = true;
            printf ("Success.\nCaptouch initialization... ");
            if (!touchpanel.begin(40)) printf ("Couldn't start FT6206 touchscreen controller");  // pass in 'sensitivity' coefficient
            else printf ("Capacitive touchscreen started\n");
        }

        void watchdog() {
            if (loop_period_us > tft_watchdog_timeout && _timing_tft_reset == 0) _timing_tft_reset = 1;
            if (_timing_tft_reset == 0) _tftDelayTimer.reset();
            else if (!_tftDelayTimer.expired()) _tftResetTimer.reset();
            else if (_timing_tft_reset == 1) {
                write_pin (tft_rst_pin, LOW);
                _timing_tft_reset = 2;
            }
            else if (_timing_tft_reset == 2 && _tftResetTimer.expired()) {
                write_pin (tft_rst_pin, HIGH);
                init();
                _timing_tft_reset = 0;
            }
        }

        // Functions to write to the screen efficiently
        //
        void draw_bargraph_base (int32_t corner_x, int32_t corner_y, int32_t width) {  // draws a horizontal bargraph scale.  124, y, 40
            _tft.drawFastHLine (corner_x+disp_bargraph_squeeze, corner_y, width-disp_bargraph_squeeze*2, GRY1);
            for (int32_t offset=0; offset<=2; offset++) _tft.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(width/2 - disp_bargraph_squeeze), corner_y-1, 3, WHT);
        }
        void draw_needle_shape (int32_t pos_x, int32_t pos_y, int32_t color) {  // draws a cute little pointy needle
            _tft.drawFastVLine (pos_x-1, pos_y, 2, color);
            _tft.drawFastVLine (pos_x, pos_y, 4, color);
            _tft.drawFastVLine (pos_x+1, pos_y, 2, color);
        }
        void draw_target_shape (int32_t pos_x, int32_t pos_y, int32_t t_color, int32_t r_color) {  // draws a cute little target symbol
            _tft.drawFastVLine (pos_x-1, pos_y+7, 2, t_color);
            _tft.drawFastVLine (pos_x, pos_y+5, 4, t_color);
            _tft.drawFastVLine (pos_x+1, pos_y+7, 2, t_color);
        }
        void draw_bargraph_needle (int32_t n_pos_x, int32_t old_n_pos_x, int32_t pos_y, int32_t n_color) {  // draws a cute little pointy needle
            draw_needle_shape (old_n_pos_x, pos_y, BLK);
            draw_needle_shape (n_pos_x, pos_y, n_color);
        }
        void draw_string (int32_t x_new, int32_t x_old, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor, bool forced=false) {  // Send in "" for oldtext if erase isn't needed
            int32_t oldlen = strlen(oldtext);
            int32_t newlen = strlen(text);
            _tft.setTextColor (bgcolor);  
            for (int32_t letter=0; letter < oldlen; letter++) {
                if (newlen - letter < 1) {
                    _tft.setCursor (x_old+disp_font_width*letter, y);
                    _tft.print (oldtext[letter]);
                }
                else if (oldtext[letter] != text[letter]) {
                    _tft.setCursor (x_old+disp_font_width*letter, y);
                    _tft.print (oldtext[letter]);
                }
            }
            _tft.setTextColor (color);  
            for (int32_t letter=0; letter < newlen; letter++) {
                if (oldlen - letter < 1) {
                    _tft.setCursor (x_new+disp_font_width*letter, y);
                    _tft.print (text[letter]);
                }
                else if (oldtext[letter] != text[letter] || forced) {
                    _tft.setCursor (x_new+disp_font_width*letter, y);
                    _tft.print (text[letter]);
                }
            }
        }
        void draw_mmph (int32_t x, int32_t y, int32_t color) {  // This is my cheesy pixel-drawn "mmph" compressed horizontally to 3-char width
            _tft.setTextColor (color);
            _tft.setCursor (x, y);
            _tft.print ("m");
            _tft.setCursor (x+4, y);
            _tft.print ("m");  // Overlapping 'mm' complete (x = 0-8)
            _tft.drawFastVLine (x+10, y+2, 6, color);
            _tft.drawPixel (x+11, y+2, color);
            _tft.drawPixel (x+11, y+6, color);
            _tft.drawFastVLine (x+12, y+3, 3, color);  // 'p' complete (x = 10-12)
            _tft.drawFastVLine (x+14, y, 7, color);
            _tft.drawPixel (x+15, y+2, color);
            _tft.drawFastVLine (x+16, y+3, 4, color);  // 'h' complete (x = 14-16)
        }
        void draw_thou (int32_t x, int32_t y, int32_t color) {  // This is my cheesy pixel-drawn "thou" compressed horizontally to 3-char width
            _tft.drawFastVLine (x+1, y+1, 5, color);
            _tft.drawFastHLine (x, y+2, 3, color);
            _tft.drawPixel (x+2, y+6, color);  // 't' complete (x = 0-2)
            _tft.drawFastVLine (x+4, y, 7, color);
            _tft.drawPixel (x+5, y+3, color);
            _tft.drawPixel (x+6, y+2, color);
            _tft.drawFastVLine (x+7, y+3, 4, color);  // 'h' complete (x = 4-7)
            _tft.drawFastVLine (x+9, y+3, 3, color);
            _tft.drawFastHLine (x+10, y+2, 2, color);
            _tft.drawFastHLine (x+10, y+6, 2, color);
            _tft.drawFastVLine (x+12, y+3, 3, color);  // 'o' complete (x = 9-12)
            _tft.drawFastVLine (x+14, y+2, 4, color);
            _tft.drawPixel (x+15, y+6, color);
            _tft.drawFastVLine (x+16, y+2, 5, color);  // 'u' complete (x = 14-16)
        }
        void draw_string_units (int32_t x, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
            _tft.setCursor (x, y);
            _tft.setTextColor (bgcolor);
            _tft.print (oldtext);  // Erase the old content
            _tft.setCursor (x, y);
            _tft.setTextColor (color);
            _tft.print (text);  // Erase the old content
        }
        void draw_colons (int32_t x_pos, int32_t first, int32_t last, int32_t color) {
            for (int32_t lineno=first; lineno <= last; lineno++) {
                _tft.drawPixel (x_pos, lineno*disp_line_height_pix+3, color);  // Tiny microscopic colon dots
                _tft.drawPixel (x_pos, lineno*disp_line_height_pix+7, color);  // Tiny microscopic colon dots
                // _tft.fillRect (x_pos, (lineno+1)*disp_line_height_pix+3, 2, 2, color);  // Big goofy looking colon dots
                // _tft.fillRect (x_pos, (lineno+1)*disp_line_height_pix+7, 2, 2, color);  // Big goofy looking colon dots
            }
        }
        // draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
        void draw_fixed (int32_t page, int32_t page_last, bool redraw_tuning_corner, bool forced=false) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
            yield();  // experiment
            _tft.setTextColor (GRY2);
            _tft.setTextSize (1);
            // if (redraw_tuning_corner) _tft.fillRect(10, 145, 154, 95, BLK); // _tft.fillRect(0,145,167,95,BLK);  // Erase old dataset page area - This line alone uses 15 ms
            int32_t y_pos;
            if (!redraw_tuning_corner) {
                for (int32_t lineno=0; lineno < disp_fixed_lines; lineno++)  {  // Step thru lines of fixed telemetry data
                    y_pos = (lineno+1)*disp_line_height_pix+disp_vshift_pix;
                    draw_string (12, 12, y_pos, telemetry[lineno], "", GRY2, BLK, forced);
                    draw_string_units (104, y_pos, units[lineno], "", GRY2, BLK);
                    draw_bargraph_base (124, y_pos+7, disp_bargraph_width);
                }
                // draw_colons(7+disp_font_width*arraysize(telemetry[0]), 1, disp_fixed_lines+disp_tuning_lines, GRY1);  // I can't decide if I like the colons or not
            }
            for (int32_t lineno=0; lineno < disp_tuning_lines; lineno++)  {  // Step thru lines of dataset page data
                yield();  // experiment
                draw_string(12, 12, (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[page][lineno], dataset_page_names[page_last][lineno], GRY2, BLK, forced);
                draw_string_units(104, (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix, tuneunits[page][lineno], tuneunits[page_last][lineno], GRY2, BLK);
                if (redraw_tuning_corner) {
                    int32_t corner_y = (lineno+disp_fixed_lines+1)*disp_line_height_pix+disp_vshift_pix+7;  // lineno*disp_line_height_pix+disp_vshift_pix-1;
                    draw_bargraph_base (124, corner_y, disp_bargraph_width);
                    if (disp_needles[lineno] >= 0) draw_bargraph_needle (-1, disp_needles[lineno], corner_y-6, BLK);  // Let's draw a needle
                }
            }
        }
        void draw_hyphen (int32_t x_pos, int32_t y_pos, int32_t color) {  // Draw minus sign in front of negative numbers
            _tft.drawFastHLine (x_pos+2, y_pos+3, 3, color);
        }
        void draw_dynamic (int32_t lineno, char const* disp_string, int32_t value, int32_t lowlim, int32_t hilim, int32_t target=-1) {
            yield();  // experiment
            int32_t age_us = (int32_t)((double)(dispAgeTimer[lineno].elapsed()) / 2500000); // Divide by us per color gradient quantum
            int32_t x_base = 59;
            bool polarity = (value >= 0);  // polarity 0=negative, 1=positive
            if (strcmp(disp_values[lineno], disp_string) || _disp_redraw_all) {  // If value differs, Erase old value and write new
                int32_t y_pos = lineno*disp_line_height_pix+disp_vshift_pix;
                if (polarity != disp_polarities[lineno]) draw_hyphen (x_base, y_pos, (!polarity) ? GRN : BLK);
                draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_string, disp_values[lineno], GRN, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
                strcpy (disp_values[lineno], disp_string);
                disp_polarities[lineno] = polarity;
                dispAgeTimer[lineno].reset();
                disp_age_quanta[lineno] = 0;
            }  // to-do: Fix failure to freshen aged coloration of unchanged characters of changed values
            else if (age_us > disp_age_quanta[lineno] && age_us < 11)  {  // As readings age, redraw in new color. This may fail and redraw when the timer overflows? 
                int32_t color;
                if (age_us < 8) color = 0x1fe0 + age_us*0x2000;  // Base of green with red added as you age, until yellow is achieved
                else color = 0xffe0 - (age_us-8) * 0x100;  // Then lose green as you age further
                int32_t y_pos = (lineno)*disp_line_height_pix+disp_vshift_pix;
                if (!polarity) draw_hyphen (x_base, y_pos, color);
                draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_values[lineno], "", color, BLK);
                disp_age_quanta[lineno] = age_us;
            }
            yield();  // experiment
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
                    if (t_pos != disp_targets[lineno] || (t_pos == n_pos)^(disp_needles[lineno] != disp_targets[lineno]) || _disp_redraw_all) {
                        draw_target_shape (disp_targets[lineno], corner_y, BLK, -1);  // Erase old target
                        _tft.drawFastHLine (disp_targets[lineno]-(disp_targets[lineno] != corner_x+disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+7, 2+(disp_targets[lineno] != corner_x+disp_bargraph_width-disp_bargraph_squeeze), GRY1);  // Patch bargraph line where old target got erased
                        for (int32_t offset=0; offset<=2; offset++) _tft.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(disp_bargraph_width/2 - disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+6, 3, WHT);  // Redraw bargraph graduations in case one got corrupted by target erasure
                        draw_target_shape (t_pos, corner_y, tcolor, -1);  // Draw the new target
                        disp_targets[lineno] = t_pos;  // Remember position of target
                    }
                }
                if (n_pos != disp_needles[lineno] || _disp_redraw_all) {
                    draw_bargraph_needle (n_pos, disp_needles[lineno], corner_y, ncolor);  // Let's draw a needle
                    disp_needles[lineno] = n_pos;  // Remember position of needle
                }
            }
            else if (disp_needles[lineno] >= 0) {  // If value having no range is drawn over one that did ...
                draw_bargraph_needle (-1, disp_needles[lineno], lineno*disp_line_height_pix+disp_vshift_pix-1, BLK);  // Erase the old needle
                disp_needles[lineno] = -1;  // Flag for no needle
            }
        }
        int32_t significant_place (double value) {  // Returns the decimal place of the most significant digit of a given float value, without relying on logarithm math
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
        std::string abs_itoa (int32_t value, int32_t maxlength) {  // returns an ascii string representation of a given integer value, using scientific notation if necessary to fit within given width constraint
            value = abs (value);  // This function disregards sign
            if (significant_place(value) <= maxlength) return std::to_string (value);  // If value is short enough, return it
            std::string result;
            int32_t magnitude = std::log10 (value);  // check how slow is log() function? Compare performance vs. multiple divides ( see abs_ftoa() )
            double scaledValue = value / std::pow (10, magnitude + 1 - maxlength);  // was (10, magnitude - 5);
            if (scaledValue >= 1.0 && scaledValue < 10.0) result = std::to_string (static_cast<int>(scaledValue));
            else result = std::to_string (scaledValue);
            if (magnitude >= maxlength) result += "e" + std::to_string (magnitude);
            return result;
        }
        std::string abs_ftoa (double value, int32_t maxlength, int32_t sigdig) {  // returns an ascii string representation of a given double value, formatted to efficiently fit withinthe given width constraint
            value = abs (value);  // This function disregards sign
            int32_t place = significant_place (value);  // Learn decimal place of the most significant digit in value
            if (place >= sigdig && place <= maxlength) {  // Then we want simple cast to an integer w/o decimal point (eg 123456, 12345, 1234)
                std::string result (std::to_string ((int32_t)value));
                return result;
            }
            if (place >= 0 && place < maxlength) {  // Then we want float formatted with enough nonzero digits after the decimal point for 4 significant digits (eg 123.4, 12.34, 1.234, 0)
                int32_t length = min (sigdig+1, maxlength);
                char buffer[length+1];
                std::snprintf (buffer, length + 1, "%.*g", length - 1, value);
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
            if (place <= -10) return std::string ("~0");  // Ridiculously small values just indicate basically zero
            char buffer[maxlength+1];  // Allocate buffer with the maximum required size
            snprintf(buffer, sizeof(buffer), "%*.*f%*d", maxlength-sigdig-1, sigdig-1, value, maxlength-1, 0);
            std::string result (buffer);  // copy buffer to result
            if (result.find ("e+0") != std::string::npos) result.replace (result.find ("e+0"), 3, "e");  // Remove useless "+0" from exponent
            else if (result.find ("e-0") != std::string::npos) result.replace (result.find ("e-0"), 3, "\x88");  // For very small scientific notation values, replace the "e-0" with a phoenetic long e character, to indicate negative power  // if (result.find ("e-0") != std::string::npos) 
            else if (result.find ("e+") != std::string::npos) result.replace (result.find ("e+"), 3, "e");  // For ridiculously large values
            return result;    
        }
        void draw_dynamic (int32_t lineno, int32_t value, int32_t lowlim, int32_t hilim, int32_t target=-1) {
            std::string val_string = abs_itoa (value, (int32_t)disp_maxlength);
            // std::cout << "Int: " << value << " -> " << val_string << ", " << ((value >= 0) ? 1 : -1) << std::endl;
            draw_dynamic (lineno, val_string.c_str(), value, lowlim, hilim, (int32_t)target);
        }
        void draw_dynamic (int32_t lineno, double value, double lowlim, double hilim, int32_t target=-1) {
            std::string val_string = abs_ftoa (value, (int32_t)disp_maxlength, 4);
            // std::cout << "Flt: " << value << " -> " << val_string << ", " << ((value >= 0) ? 1 : -1) << std::endl;
            draw_dynamic (lineno, val_string.c_str(), (int32_t)value, (int32_t)lowlim, (int32_t)hilim, target);
        }
        void draw_dynamic (int32_t lineno, double value, double lowlim, double hilim, double target) {
            draw_dynamic (lineno, value, lowlim, hilim, (int32_t)target);
        }
        void draw_runmode (int32_t runmode, int32_t oldmode, int32_t color_override=-1) {  // color_override = -1 uses default color
            yield();
            int32_t color = (color_override == -1) ? colorcard[runmode] : color_override;
            int32_t x_new = 8+6*(2+strlen (modecard[runmode]))-3;
            int32_t x_old = 8+6*(2+strlen (modecard[oldmode]))-3;
            draw_string (8+6, 8+6, disp_vshift_pix, modecard[oldmode], "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (x_old, x_old, disp_vshift_pix, "Mode", "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (8+6, 8+6, disp_vshift_pix, modecard[runmode], "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (x_new, x_new, disp_vshift_pix, "Mode", "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        }
        void draw_dataset_page (int32_t page, int32_t page_last, bool forced=false) {
            draw_fixed (page, page_last, true, forced);  // Erase and redraw dynamic data corner of screen with names, units etc.
            // for (int32_t lineno=0; lineno<disp_lines; lineno++) draw_hyphen (59, lineno*disp_line_height_pix+disp_vshift_pix, BLK);
            yield();
            draw_string (83, 83, disp_vshift_pix, pagecard[page], pagecard[page_last], RBLU, BLK, forced); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        }
        void draw_selected_name (int32_t tun_ctrl, int32_t tun_ctrl_last, int32_t selected_val, int32_t selected_last) {
            yield();
            if (selected_val != selected_last) draw_string (12, 12, 12+(selected_last+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_last], "", GRY2, BLK);
            draw_string (12, 12, 12+(selected_val+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, dataset_page_names[dataset_page][selected_val], "", (tun_ctrl == EDIT) ? GRN : ((tun_ctrl == SELECT) ? YEL : GRY2), BLK);
        }
        void draw_bool (bool value, int32_t col) {  // Draws values of boolean data
            if ((disp_bool_values[col-2] != value) || _disp_redraw_all) {  // If value differs, Erase old value and write new
                int32_t x_mod = touch_margin_h_pix + touch_cell_h_pix*(col) + (touch_cell_h_pix>>1) - arraysize (top_menu_buttons[col-2]-1)*(disp_font_width>>1) - 2;
                draw_string (x_mod, x_mod, 0, top_menu_buttons[col-2], "", (value) ? GRN : LGRY, DGRY);
                disp_bool_values[col-2] = value;
            }
        }
        void draw_simbuttons (bool create) {  // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
            _tft.setTextColor (LYEL);
            for (int32_t row = 0; row < arraysize(simgrid); row++) {
                for (int32_t col = 0; col < arraysize(simgrid[row]); col++) {
                    yield();
                    int32_t cntr_x = touch_margin_h_pix + touch_cell_h_pix*(col+3) + (touch_cell_h_pix>>1) +2;
                    int32_t cntr_y = touch_cell_v_pix*(row+1) + (touch_cell_v_pix>>1);
                    if (strcmp (simgrid[row][col], "    " )) {
                        _tft.fillCircle (cntr_x, cntr_y, 19, create ? DGRY : BLK);
                        if (create) {
                            _tft.drawCircle (cntr_x, cntr_y, 19, LYEL);
                            int32_t x_mod = cntr_x-(arraysize (simgrid[row][col])-1)*(disp_font_width>>1);
                            draw_string (x_mod, x_mod, cntr_y-(disp_font_height>>1), simgrid[row][col], "", LYEL, DGRY);
                        }
                    }
                }     
            }
        }
        void draw_touchgrid (bool side_only) {  // draws edge buttons with names in 'em. If replace_names, just updates names
            int32_t namelen = 0;
            _tft.setTextColor (WHT);
            for (int32_t row = 0; row < arraysize (side_menu_buttons); row++) {  // Step thru all rows to draw buttons along the left edge
                yield();
                _tft.fillRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, DGRY);
                _tft.drawRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, LYEL);
                namelen = 0;
                for (uint32_t x = 0 ; x < arraysize (side_menu_buttons[row]) ; x++ ) {
                    if (side_menu_buttons[row][x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
                }
                for (int32_t letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
                    yield();
                    _tft.setCursor (1, ( touch_cell_v_pix*row) + (touch_cell_v_pix/2) - (int32_t)(4.5*((double)namelen-1)) + (disp_font_height+1)*letter); // adjusts vertical offset depending how many letters in the button name and which letter we're on
                    _tft.println (side_menu_buttons[row][letter]);  // Writes each letter such that the whole name is centered vertically on the button
                }
            }
            if (!side_only) {
                for (int32_t col = 2; col <= 5; col++) {  // Step thru all cols to draw buttons across the top edge
                    yield();
                    _tft.fillRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
                    _tft.drawRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, LYEL);  // _tft.width()-9, 3, 18, (_tft.height()/5)-6, 8, LYEL);
                    // draw_bool (top_menu_buttons[btn], btn+3);
                }
            }
        }

        void update() {
            if (simulating != simulating_last || _disp_redraw_all) {
                draw_simbuttons(simulating);  // if we just entered simulator draw the simulator buttons, or if we just left erase them
                simulating_last = simulating;
                _procrastinate = true;  // Waits till next loop to draw changed values
            }
            if ((disp_dataset_page_dirty || _disp_redraw_all)) {
                static bool first = true;
                draw_dataset_page(dataset_page, dataset_page_last, first);
                first = false;
                disp_dataset_page_dirty = false;
                if (dataset_page_last != dataset_page) config.putUInt("dpage", dataset_page);
                dataset_page_last = dataset_page;
                _procrastinate = true;  // Waits till next loop to draw changed values
            }
            if ((disp_sidemenu_dirty || _disp_redraw_all)) {
                draw_touchgrid(true);
                disp_sidemenu_dirty = false;
                _procrastinate = true;  // Waits till next loop to draw changed values
            }
            if (disp_selected_val_dirty || _disp_redraw_all) {
                draw_selected_name(tuning_ctrl, tuning_ctrl_last, selected_value, selected_value_last);
                disp_selected_val_dirty = false;
                selected_value_last = selected_value;
                tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
            }
            if (disp_runmode_dirty || _disp_redraw_all) {
                draw_runmode(runmode, oldmode, (runmode == SHUTDOWN) ? shutdown_color : -1);
                disp_runmode_dirty = false;
                oldmode = runmode;  // remember what mode we're in for next time
            }
            if ((dispRefreshTimer.expired() && !_procrastinate) || _disp_redraw_all) {
                dispRefreshTimer.reset();
                double drange;
                draw_dynamic(1, carspeed_filt_mph, 0.0, carspeed_redline_mph, cruiseSPID.get_target());
                draw_dynamic(2, tach_filt_rpm, 0.0, tach_redline_rpm, gasSPID.get_target());
                draw_dynamic(3, pressure_filt_psi, pressure_min_psi, pressure_max_psi, brakeSPID.get_target());  // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.get_target() : pressure_target_adc);
                draw_dynamic(4, ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
                draw_dynamic(5, ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
                draw_dynamic(6, cruiseSPID.get_target(), 0.0, carspeed_govern_mph);
                draw_dynamic(7, brakeSPID.get_target(), pressure_min_psi, pressure_max_psi);
                draw_dynamic(8, gasSPID.get_target(), 0.0, tach_redline_rpm);
                draw_dynamic(9, (int32_t)brake_pulse_out_us, brake_pulse_retract_us, brake_pulse_extend_us);
                draw_dynamic(10, gas_pulse_out_us, gas_pulse_redline_us, gas_pulse_idle_us);
                draw_dynamic(11, steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);
                if (dataset_page == RUN) {
                    draw_dynamic(12, battery_filt_v, 0.0, battery_max_v);
                    draw_dynamic(13, brake_pos_filt_in, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);
                    draw_dynamic(14, pot_filt_percent, pot_min_percent, pot_max_percent);
                    // draw_dynamic (14, brakeSPID.get_proportionality(), -1, -1);
                    draw_dynamic(15, sim_brkpos, -1, -1);
                    draw_dynamic(16, sim_joy, -1, -1);
                    draw_dynamic(17, sim_pressure, -1, -1);
                    draw_dynamic(18, sim_tach, -1, -1);
                    draw_dynamic(19, sim_speedo, -1, -1);
                }
                else if (dataset_page == JOY) {
                    draw_dynamic(12, ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
                    draw_dynamic(13, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
                    draw_dynamic(14, ctrl_lims_adc[ctrl][HORZ][MIN], 0, (adcrange_adc-ctrl_lims_adc[ctrl][HORZ][MAX])/2);
                    draw_dynamic(15, ctrl_lims_adc[ctrl][HORZ][MAX], (ctrl_lims_adc[ctrl][HORZ][MIN]-adcrange_adc)/2, adcrange_adc);
                    draw_dynamic(16, ctrl_lims_adc[ctrl][HORZ][DB], 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN] > ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][HORZ][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][HORZ][MIN]));
                    draw_dynamic(17, ctrl_lims_adc[ctrl][VERT][MIN], 0, (adcrange_adc-ctrl_lims_adc[ctrl][VERT][MAX])/2);
                    draw_dynamic(18, ctrl_lims_adc[ctrl][VERT][MAX], (ctrl_lims_adc[ctrl][VERT][MIN]-adcrange_adc)/2, adcrange_adc);
                    draw_dynamic(19, ctrl_lims_adc[ctrl][VERT][DB], 0, (adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN] > ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) ? 2*(ctrl_lims_adc[ctrl][VERT][MAX] - adcmidscale_adc) : 2*(adcmidscale_adc - ctrl_lims_adc[ctrl][VERT][MIN]));
                }
                else if (dataset_page == CAR) {
                    draw_dynamic(12, gas_governor_percent, 0, 100);
                    draw_dynamic(13, tach_idle_rpm, 0.0, tach_redline_rpm);
                    draw_dynamic(14, tach_redline_rpm, 0.0, tach_max_rpm);
                    draw_dynamic(15, carspeed_idle_mph, 0.0, carspeed_redline_mph);
                    draw_dynamic(16, carspeed_redline_mph, 0.0, carspeed_max_mph);
                    draw_dynamic(17, ctrl, -1, -1);  // 0 if hotrc
                    draw_dynamic(18, cal_joyvert_brkmotor, -1, -1);
                    draw_dynamic(19, cal_pot_gasservo, -1, -1);
                }
                else if (dataset_page == PWMS) {
                    draw_dynamic(12, steer_pulse_left_us, steer_pulse_stop_us, steer_pulse_left_max_us);
                    draw_dynamic(13, steer_pulse_stop_us, steer_pulse_left_us, steer_pulse_right_us);
                    draw_dynamic(14, steer_pulse_right_us, steer_pulse_right_min_us, steer_pulse_stop_us);
                    draw_dynamic(15, brake_pulse_extend_us, brake_pulse_stop_us, brake_pulse_extend_max_us);
                    draw_dynamic(16, brake_pulse_stop_us, brake_pulse_retract_us, brake_pulse_extend_us);
                    draw_dynamic(17, brake_pulse_retract_us, brake_pulse_retract_min_us, brake_pulse_stop_us);
                    draw_dynamic(18, gas_pulse_idle_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
                    draw_dynamic(19, gas_pulse_redline_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
                }
                else if (dataset_page == BPID) {
                    drange = pressure_max_psi-pressure_min_psi;
                    draw_dynamic(12, brakeSPID.get_error(), -drange, drange);
                    draw_dynamic(13, brakeSPID.get_p_term(), -drange, drange);
                    draw_dynamic(14, brakeSPID.get_i_term(), -drange, drange);
                    draw_dynamic(15, brakeSPID.get_d_term(), -drange, drange);
                    draw_dynamic(16, brakeSPID.get_output(), -drange, drange);  // brake_spid_carspeed_delta_adc, -range, range);
                    draw_dynamic(17, brakeSPID.get_kp(), 0.0, 2.0);
                    draw_dynamic(18, brakeSPID.get_ki_hz(), 0.0, 2.0);
                    draw_dynamic(19, brakeSPID.get_kd_s(), 0.0, 2.0);
                }
                else if (dataset_page == GPID) {
                    drange = tach_govern_rpm-tach_idle_rpm;
                    draw_dynamic(12, gasSPID.get_error(), -drange, drange);
                    draw_dynamic(13, gasSPID.get_p_term(), -drange, drange);
                    draw_dynamic(14, gasSPID.get_i_term(), -drange, drange);
                    draw_dynamic(15, gasSPID.get_d_term(), -drange, drange);
                    draw_dynamic(16, gasSPID.get_output(), -drange, drange);  // gas_spid_carspeed_delta_adc, -drange, drange);
                    draw_dynamic(17, gasSPID.get_kp(), 0.0, 2.0);
                    draw_dynamic(18, gasSPID.get_ki_hz(), 0.0, 2.0);
                    draw_dynamic(19, gasSPID.get_kd_s(), 0.0, 2.0);
                }
                else if (dataset_page == CPID) {
                    drange = carspeed_govern_mph-carspeed_idle_mph;
                    draw_dynamic(12, cruiseSPID.get_error(), -drange, drange);
                    draw_dynamic(13, cruiseSPID.get_p_term(), -drange, drange);
                    draw_dynamic(14, cruiseSPID.get_i_term(), -drange, drange);
                    draw_dynamic(15, cruiseSPID.get_d_term(), -drange, drange);
                    draw_dynamic(16, cruiseSPID.get_output(), -drange, drange);  // cruise_spid_carspeed_delta_adc, -drange, drange);
                    draw_dynamic(17, cruiseSPID.get_kp(), 0.0, 2.0);
                    draw_dynamic(18, cruiseSPID.get_ki_hz(), 0.0, 2.0);
                    draw_dynamic(19, cruiseSPID.get_kd_s(), 0.0, 2.0);
                }
                else if (dataset_page == TEMP) {
                    draw_dynamic(12, temps[AMBIENT], temp_min, temp_max);
                    draw_dynamic(13, temps[ENGINE], temp_min, temp_max);
                    draw_dynamic(14, temps[WHEEL_FL], temp_min, temp_max);
                    draw_dynamic(15, temps[WHEEL_FR], temp_min, temp_max);
                    draw_dynamic(16, temps[WHEEL_RL], temp_min, temp_max);
                    draw_dynamic(17, temps[WHEEL_RR], temp_min, temp_max);
                    draw_dynamic(18, gasSPID.get_open_loop(), -1, -1);
                    draw_dynamic(19, brake_pos_zeropoint_in, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);   
                }
                draw_bool((runmode == CAL), 2);
                draw_bool((runmode == BASIC), 3);
                draw_bool(ignition, 4);
                draw_bool(syspower, 5);
            }
            _procrastinate = false;
            _disp_redraw_all = false;
        }
};

#endif