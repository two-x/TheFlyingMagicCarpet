#ifndef UI_H
#define UI_H
#include <stdio.h>
#include <SPI.h>  // SPI serial bus needed to talk to the LCD and the SD card
#include <Wire.h>  // Contains I2C serial bus, needed to talk to touchscreen chip
#include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
#include <Adafruit_ILI9341.h>  // For interfacing with the TFT LCD controller chip
#include <SDfat.h>
#ifdef DUE
    #include <LibPrintf.h>  // This works on Due but not ESP32
#endif
#include <Adafruit_NeoPixel.h>  // Plan to allow control of neopixel LED onboard the esp32
#include "Arduino.h"  // #include <Adafruit_GFX.h>  // For drawing pictures & text on the screen
#include "devices.h"
#include "globals.h"
#include "spid.h"

static Adafruit_NeoPixel strip(1, neopixel_pin, NEO_GRB + NEO_GRB + NEO_KHZ800);
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

SdFat sd;  // SD card filesystem
#define approot "cantroller2020"
#define logfile "log.txt"
#define error(msg) sd.errorHalt(F(msg))  // Error messages stored in flash.
SdFile root;  // Directory file.
SdFile file;  // Use for file creation in folders.

// LCD is 2.8in diagonal, 240x320 pixels
// LCD supports 18-bit color, but GFX library uses 16-bit color, organized (MSB) 5b-red, 6b-green, 5b-blue (LSB)
// Since the RGB don't line up with the nibble boundaries, it's tricky to quantify a color, here are some colors:
// Color picker websites: http://www.barth-dev.de/online/rgb565 , https://chrishewett.com/blog/true-rgb565-colour-picker/
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
        "     Pot:",
        " Sim Joy:",
        "SimBkPos:",
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
        "   Delta:",
        "      kP:",
        "      kI:",
        "      kD:", },
    {   " Eng Err:",  // GPID
        "  P Term:",
        "  I Term:",
        "  D Term:",
        "   Delta:",
        "      kP:",
        "      kI:",
        "      kD:", },
    {   " Spd Err:",  // CPID
        "  P Term:",
        "  I Term:",
        "  D Term:",
        "   Delta:",
        "      kP:",
        "      kI:",
        "      kD:", },
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


Wire.begin(i2c_sda_pin, i2c_scl_pin);
static Adafruit_FT6206 touchpanel = Adafruit_FT6206(); // Touch panel
static Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_cs_pin, tft_dc_pin);  // LCD screen

bool display_enabled = true;  // Should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool disp_redraw_all = true;
char disp_draw_buffer[7];  // Used to convert integers to ascii for purposes of displaying on screen
char disp_draw_buffer2[7];  // Used to convert integers to ascii for purposes of displaying on screen
char disp_values[disp_lines][7];
int32_t disp_needles[disp_lines];
int32_t disp_targets[disp_lines];
int32_t disp_age_quanta[disp_lines];
Timer dispAgeTimer[disp_lines];  // int32_t disp_age_timer_us[disp_lines];
bool disp_bool_values[6];
char disp_bool_buffer;
bool selected_val_dirty = true;
bool dataset_page_dirty = true;
int32_t dataset_page = LOCK;  // Which of the six 8-value dataset pages is currently displayed, and available to edit
int32_t dataset_page_last = dataset_page;
int32_t selected_value = 0;  // In the real time tuning UI, which of the editable values (0-7) is selected. -1 for none 
int32_t selected_value_last = 0;
Timer simTimer;
int32_t sim_edit_delta = 0;
int32_t sim_edit_delta_touch = 0;
int32_t sim_edit_delta_encoder = 0;
int32_t tuning_ctrl = OFF;
int32_t tuning_ctrl_last = OFF;
Timer tuningCtrlTimer(25000000);  // This times out edit mode after a a long period of inactivity

Timer touchPollTimer(35000);  // Timer for regular touchscreen sampling
Timer touchHoldTimer(1000000);  // For timing touch long presses
Timer touchAccelTimer(850000);  // Touch hold time per left shift (doubling) of touch_accel
bool touch_now_touched = false;  // Is a touch event in progress
int32_t touch_accel_exponent = 0;  // Will edit values by +/- 2^touch_accel_exponent per touch_period interval
int32_t touch_accel = 1 << touch_accel_exponent;  // Touch acceleration level, which increases the longer you hold. Each edit update chages value by this
int32_t touch_accel_exponent_max = 8;  // Never edit values faster than this. 2^8 = 256 change in value per update
bool touch_longpress_valid = true;

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
        bool dirty = false, str_overflow = false;
        int32_t age_us = (int32_t)(( (double)(dispAgeTimer[lineno].elapsed()) / 2500000)); // Divide by us per color gradient quantum
        int32_t val_length = strlen(itoa(value, disp_draw_buffer, 10));
        if (val_length > 6) str_overflow = true;
        memset(disp_draw_buffer, 0, 7);
        itoa(value, disp_draw_buffer, 10);  // Modeflag 0 is for writing numeric values for variables in the active data column at a given line
        if (str_overflow) disp_draw_buffer[6] = '\x10';
        // disp_draw_buffer[7] = 0;
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
            if (modeflag == 4)  {  // If modeflag == 4
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

// Device::Encoder is a class for rotary encoder input devices
class RotaryEncoder  {
  public:
};

#endif  // UI_H