#pragma once
#include <TFT_eSPI.h>
#include <iostream>  // for gamma correction
#include <cmath>  // for gamma correction
// LCD supports 18-bit color, but GFX library uses 16-bit color, organized (MSB) 5b-red, 6b-green, 5b-blue (LSB)
// Since the RGB don't line up with the nibble boundaries, it's tricky to quantify a color, here are some colors:
#define BLK  0x0000  // greyscale: full black (RGB elements off)
#define HGRY 0x2104  // greyscale: hella dark grey
#define DGRY 0x39c7  // greyscale: very dark grey
#define GRY1 0x8410  // greyscale: dark grey  (10000)(100 000)(10000) = 84 10
#define GRY2 0xc618  // greyscale: light grey  (11000)(110 000)(11000) = C6 18
#define LGRY 0xd6ba  // greyscale: very light grey
#define WHT  0xffff  // greyscale: full white (RGB elements full on)
#define RED  0xf800  // primary red (R element full on)
#define YEL  0xffe0  // Secondary yellow (RG elements full on)
#define GRN  0x07e0  // primary green (G element full on)
#define CYN  0x07ff  // secondary cyan (GB elements full on)  (00000)(111 111)(11111) = 07 ff
#define BLU  0x001f  // primary red (B element full on)
#define MGT  0xf81f  // secondary magenta (RB elements full on)
#define DRED 0xb000  // dark red
#define BORG 0xfa00  // blood orange (very reddish orange)
#define BRN  0xfa40  // dark orange aka brown
#define ORG  0xfca0  // 
#define LYEL 0xfff8  // 
#define GGRN 0x5cac  // a low saturation greyish pastel green
#define TEAL 0x07f9  // this teal is barely distinguishable from cyan
#define STBL 0x767d  // steel blue is desaturated light blue
#define DCYN 0x0575  // dark cyan
#define RBLU 0x043f  // royal blue
#define MBLU 0x009f  // midnight blue
#define INDG 0x601f  // indigo (deep blue with a hint of purple)
#define ORCD 0xb81f  // orchid (lighter and less saturated purple)
#define PUR  0x881f  // 
#define GPUR 0x8c15  // a low saturation greyish pastel purple
#define LPUR 0xc59f  // a light pastel purple
#define PNK  0xfcf3  // pink is the best color
#define DPNK 0xfa8a  // we need all shades of pink
#define LPNK 0xfe18  // especially light pink, the champagne of pinks
// 5-6-5 color picker site: http://www.barth-dev.de/online/rgb565  // named colors: https://wiki.tcl-lang.org/page/Colors+with+Names

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
#define disp_default_float_precision 3  // Significant digits displayed for float values. Higher causes more screen draws
#define disp_datapage_names_x 12
#define disp_datapage_values_x 59
#define disp_datapage_units_x 104        
#define disp_bargraphs_x 123
#define disp_runmode_text_x 8
#define disp_datapage_title_x 83
#define disp_simbutton_radius_pix 19
#define disp_idiot_corner_x 165
#define disp_idiot_corner_y 13
#define disp_idiots_per_row 11
#define disp_idiot_row_height 10
#define touch_cell_v_pix 48  // When touchscreen gridded as buttons, height of each button
#define touch_cell_h_pix 53  // When touchscreen gridded as buttons, width of each button
#define touch_margin_h_pix 1  // On horizontal axis, we need an extra margin along both sides button sizes to fill the screen
#define disp_simbuttons_x 165
#define disp_simbuttons_y 48
#define disp_saver_width 155
#define disp_saver_height 192

float globalgamma = 2.2;  // Standard gamma is 2.2
char side_menu_buttons[5][4] = { "PAG", "SEL", "+  ", "-  ", "SIM" };  // Pad shorter names with spaces on the right
char top_menu_buttons[4][6] = { " CAL ", "BASIC", " IGN ", "POWER" };  // Pad shorter names with spaces to center
char disp_values[disp_lines][disp_maxlength+1];  // Holds previously drawn value strings for each line
bool disp_polarities[disp_lines];  // Holds sign of previously drawn values
bool display_enabled = true;  // Should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool disp_bool_values[6];
bool disp_selected_val_dirty, disp_datapage_dirty, disp_data_dirty, disp_sidemenu_dirty, disp_runmode_dirty, disp_simbuttons_dirty, disp_idiots_dirty;
int32_t disp_needles[disp_lines];
int32_t disp_targets[disp_lines];
int32_t disp_age_quanta[disp_lines];
Timer dispAgeTimer[disp_lines];  // int32_t disp_age_timer_us[disp_lines];
Timer dispRefreshTimer (100000);  // Don't refresh screen faster than this (16667us = 60fps, 33333us = 30fps, 66666us = 15fps)
uint32_t tft_watchdog_timeout_us = 100000;

uint16_t gamma_x(uint16_t element565, float gamma = globalgamma) {  // (31.0 * (float)(bits * 2 - 10)
    float gamma565 = static_cast<float>(element565) / 31.0;  // Convert the original 5-6-5 bit value to the range 0-1
    gamma565 = std::pow(gamma565, gamma);  // Apply gamma correction
    return static_cast<uint16_t>(gamma565 * 31.0);  // Convert the corrected value back to 5-6-5 bit range
}
uint16_t gamma16(uint16_t element565, float gamma = globalgamma) {
    uint16_t r = (element565 >> 11) & 0x1f;
    uint16_t g = (element565 >> 5) & 0x3f;
    uint16_t b = element565 & 0x1f;
    return (gamma_x(r, gamma) << 11) | (gamma_x(g, gamma) << 5) | gamma_x(b, gamma);
}
uint32_t color_16b_to_uint32(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to uint32 in format 0x00RRGGBB
    return (((uint32_t)color565 & 0xf800) << 8) | (((uint32_t)color565 & 0x7e0) << 5) | (((uint32_t)color565 & 0x1f) << 3);
}
uint16_t color_uint32_to_16b(uint32_t color32b) {  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
    return (uint16_t)(((color32b & 0xf80000) >> 8) | ((color32b & 0xfc00) >> 5) | ((color32b & 0xf8) >> 3));
}
// hue: 0,255 = red, 85 = grn, 170 = blu | sat: 0 = saturated up to greyscale, 255 = pure color | bright: 0 = blk, 255 = "full" | bright_flat: if =1, "full" brightness varies w/ hue for consistent luminance, otherwise "full" always ranges to 255 (mixed-element colors are brighter) | blu_boost: adds blu_boost/255 desaturation as a ratio of blu dominance
template <typename T>
T hsv_to_rgb(uint8_t hue, uint8_t sat = 255, uint8_t bright = 255, bool bright_flat = 1, uint8_t blu_boost = 0) {  // returns uint32 color in format 0x00RRGGBB
    uint32_t rgb[3] = { 255 - 3 * (uint32_t)((255 - hue) % 85), 0, 3 * (uint32_t)((255 - hue) % 85) };
    float maxc = (float)((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]);
    if (hue <= 85) { rgb[1] = rgb[0]; rgb[0] = rgb[2]; rgb[2] = 0; }
    else if (hue <= 170) { rgb[1] = rgb[2]; rgb[2] = rgb[0]; rgb[0] = 0; }
    float brightener = (float)bright / (bright_flat ? 255.0 : maxc);
    float blu_booster = 1 + (float)(blu_boost * rgb[2]) / (float)(255.0 * (rgb[0] + rgb[1] + rgb[2]));
    for (int led=0; led<=2; led++) 
        rgb[led] = brightener * ((float)rgb[led] + blu_booster * (255.0 - sat) * (float)(maxc - rgb[led]) / 255.0);
    if (std::is_same<T, uint16_t>::value) return (T)((rgb[0] & 0xf8) << 8) | ((rgb[1] & 0xfc) << 5) | (rgb[2] >> 3);
    else if (std::is_same<T, uint32_t>::value) return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
}
uint16_t hue_to_rgb16(uint8_t hue) {  // returns uint16 color
    if (hue <= 85) return (((3 * ((255 - hue) % 85)) & 0xf8) << 8) | (((255 - 3 * ((255 - hue) % 85)) & 0xfc) << 3);
    if (hue <= 170) return (((3 * ((255 - hue) % 85)) & 0xfc) << 3) | ((255 - 3 * ((255 - hue) % 85)) >> 3);
    return (((255 - 3 * ((255 - hue) % 85)) & 0xf8) << 8) | ((3 * ((255 - hue) % 85)) >> 3);
}

char modecard[8][7] = { "Basic", "Asleep", "Shutdn", "Stall", "Hold", "Fly", "Cruise", "Cal" };
int32_t colorcard[arraysize(modecard)] = { MGT, MBLU, RED, ORG, YEL, GRN, TEAL, LPNK };

char sensorcard[14][7] = { "none", "joy", "bkpres", "brkpos", "speedo", "tach", "airflw", "mapsns", "engtmp", "batery", "startr", "basic", "ign", "syspwr" };

char idlemodecard[3][7] = { "direct", "cntrol", "minimz" };
char idlestatecard[Throttle::targetstates::num_states][7] = { "todriv", "drving", "toidle", "tolow", "idling", "minimz" };

// These defines are just a convenience to keep the below datapage strings array initializations aligned in neat rows & cols for legibility
#define stEr "St\x88r"
#define brAk "Br\x83k"
#define spEd "Sp\x88""d"
#define b1nary "  \xa7 "
#define scroll "\x12   "
#define degree "\xf7   "
#define degreF "\xf7""F  "
#define ______ "    "
#define __________ "      \xf9 "
#define neo_bright "NeoBr\x8dgt"
#define maxadjrate "MaxAjR\x83t"
#define horfailsaf "HFails\x83""f"

char telemetry[disp_fixed_lines][9] = { "TriggerV", "   Speed", "    Tach", "Throttle", brAk"Pres", brAk"Motr", "JoysticH", stEr"Motr", };  // Fixed rows
char units[disp_fixed_lines][5] = { "%   ", "mph ", "rpm ", "%   ", "psi ", "%   ", "%   ", "%   " };  // Fixed rows

enum datapages { PG_RUN, PG_JOY, PG_SENS, PG_PWMS, PG_IDLE, PG_BPID, PG_GPID, PG_CPID, PG_TEMP, PG_SIM, PG_UI, num_datapages };
char pagecard[datapages::num_datapages][5] = { "Run ", "Joy ", "Sens", "PWMs", "Idle", "Bpid", "Gpid", "Cpid", "Temp", "Sim ", "UI  " };
int32_t tuning_first_editable_line[datapages::num_datapages] = { 9, 9, 5, 7, 4, 8, 7, 7, 10, 0, 7 };  // first value in each dataset page that's editable. All values after this must also be editable

char datapage_names[datapages::num_datapages][disp_tuning_lines][9] = {
    { brAk"Posn", "MuleBatt", "LiPoBatt", "     Pot", "Air Velo", "     MAP", "MasAirFl", __________, __________, "Governor", stEr"Safe", },  // PG_RUN
    { "HRc Horz", "HRc Vert", "HotRcCh3", "HotRcCh4", "TrigVRaw", "JoyH Raw", __________, __________, __________, horfailsaf, "Deadband", },  // PG_JOY
    { "PressRaw", "BkPosRaw", __________, __________, __________, "AirSpMax", " MAP Min", " MAP Max", spEd"Idle", spEd"RedL", "BkPos0Pt", },  // PG_SENS
    { "Throttle", "Throttle", brAk"Motr", brAk"Motr", stEr"Motr", stEr"Motr", __________, "ThrotCls", "ThrotOpn", brAk"Stop", brAk"Duty", },  // PG_PWMS
    { "IdlState", "Tach Tgt", "StallIdl", "Low Idle", "HighIdle", "ColdIdle", "Hot Idle", "ColdTemp", "Hot Temp", "SetlRate", "IdleMode", },  // PG_IDLE
    { "PresTarg", "Pres Err", "  P Term", "  I Term", "  D Term", "Integral", brAk"Motr", brAk"Pres", "Brake Kp", "Brake Ki", "Brake Kd", },  // PG_BPID
    { "TachTarg", "Tach Err", "  P Term", "  I Term", "  D Term", "Integral", __________, "OpenLoop", "  Gas Kp", "  Gas Ki", "  Gas Kd", },  // PG_GPID
    { spEd"Targ", "SpeedErr", "  P Term", "  I Term", "  D Term", "Integral", "ThrotSet", maxadjrate, "Cruis Kp", "Cruis Ki", "Cruis Kd", },  // PG_CPID
    { " Ambient", "  Engine", "AxleFrLt", "AxleFrRt", "AxleRrLt", "AxleRrRt", __________, __________, __________, __________, "No Temps", },  // PG_TEMP
    { "Joystick", brAk"Pres", brAk"Posn", "  Speedo", "    Tach", "AirSpeed", "     MAP", "Basic Sw", " Pot Map", "CalBrake", " Cal Gas", },  // PG_SIM
    { "LoopFreq", "Loop Avg", "LoopPeak", " Touch X", " Touch Y", " Touch X", " Touch Y", "BlnkDemo", neo_bright, "NeoDesat", "ScrSaver", },  // PG_UI      // "   Gamma"
};
char tuneunits[datapages::num_datapages][disp_tuning_lines][5] = {
    { "in  ", "V   ", "V   ", "%   ", "mph ", "psi ", "g/s ", ______, ______, "%   ", "%   ", },  // PG_RUN
    { "us  ", "us  ", "us  ", "us  ", "%   ", "%   ", ______, ______, ______, "us  ", "us  ", },  // PG_JOY
    { "adc ", "adc ", ______, ______, ______, "mph ", "psi ", "psi ", "mph ", "mph ", "in  ", },  // PG_SENS
    { degree, "us  ", "V   ", "us  ", "V   ", "us  ", ______, degree, degree, "us  ", "%   ", },  // PG_PWMS
    { scroll, "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", degreF, degreF, "rpms", scroll, },  // PG_IDLE
    { "psi ", "psi ", "%   ", "%   ", "%   ", "%   ", "us  ", "adc ", ______, "Hz  ", "s   ", },  // PG_BPID
    { "rpm ", "rpm ", "%   ", "%   ", "%   ", "%   ", ______, b1nary, ______, "Hz  ", "s   ", },  // PG_GPID
    { "mph ", "mph ", "rpm ", "rpm ", "rpm ", "rpm ", "%   ", "%/s ", ______, "Hz  ", "s   ", },  // PG_CPID
    { degreF, degreF, degreF, degreF, degreF, degreF, ______, ______, ______, ______, b1nary, },  // PG_TEMP
    { b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, scroll, b1nary, b1nary, },  // PG_SIM
    { "Hz  ", "us  ", "us  ", "pix ", "pix ", "ohm ", "ohm ", b1nary, "%   ", "/10 ", b1nary, },  // PG_UI
};
char simgrid[4][3][5] = {
    { "psi\x18", "rpm\x18", "mph\x18" },
    { "psi\x19", "rpm\x19", "mph\x19" },
    { ______, " \x1e  ", ______ },
    { " \x11  ", " \x1f  ", "  \x10 " },  // Font special characters is the right-side map:  https://learn.adafruit.com/assets/103682
};  // The greek mu character we used for microseconds no longer works after switching from Adafruit to tft_espi library. So I switched em to "us" :(

char unitmapnames[8][5] = { "usps", "us  ", "rpms", scroll, b1nary, "%   ", "ohm ", "eyes" };  // unit strings matching these will get replaced by the corresponding bitmap graphic below
uint8_t unitmaps[8][17] = {  // 17x7-pixel bitmaps for where units use symbols not present in the font, are longer than 3 characters, or are just special
    { 0x7e, 0x20, 0x20, 0x3c, 0x00, 0x24, 0x2a, 0x2a, 0x12, 0x00, 0x70, 0x0e, 0x00, 0x24, 0x2a, 0x2a, 0x12, },  // usps - microseconds per second
    { 0x40, 0x7e, 0x20, 0x20, 0x1c, 0x20, 0x00, 0x24, 0x2a, 0x2a, 0x2a, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, },  // us - b/c the font's "mu" character doesn't work
    { 0x1f, 0x01, 0x00, 0x3f, 0x09, 0x0e, 0x00, 0x0f, 0x01, 0x0e, 0x01, 0x0e, 0x60, 0x1c, 0x00, 0x58, 0x74, },  // rpm/s (or rot/m*s) - rate of change of engine rpm
    { 0x04, 0x02, 0x7f, 0x02, 0x04, 0x00, 0x10, 0x20, 0x7f, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // scroll arrows - to indicate multiple choice
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x1c, 0x22, 0x22, 0x1c, 0x00, 0x00, },  // 0/1 - to indicate binary value
    { 0x02, 0x45, 0x25, 0x12, 0x08, 0x24, 0x52, 0x51, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // % - just because the font one is feeble
    { 0x4e, 0x51, 0x61, 0x01, 0x61, 0x51, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // Capital omega - for ohms
    { 0x08, 0x1c, 0x2a, 0x08, 0x00, 0x3e, 0x49, 0x5d, 0x49, 0x41, 0x3e, 0x49, 0x5d, 0x49, 0x41, 0x41, 0x3e, },  // Googly eyes, to point out new features
};  // These bitmaps are in the same format as the idiot light bitmaps, described below
//  { 0x7e, 0x20, 0x3e, 0x20, 0x00, 0x0c, 0x52, 0x4a, 0x3c, 0x00, 0x60, 0x18, 0x06, 0x00, 0x2c, 0x2a, 0x32, },  // ug/s - for manifold mass airflow
bool* idiotlights[16] = {
    &(err_sensor_alarm[LOST]), &(err_sensor_alarm[RANGE]), &(temp_err[ENGINE]), &(temp_err[WHEEL]), &panicstop, 
    hotrc.radiolost_ptr(), &shutdown_incomplete, &park_the_motors, &autostopping, &cruise_adjusting,
    &car_hasnt_moved, &starter, &boot_button, sim.enabled_ptr(), &running_on_devboard, 
    &powering_up,
};
char idiotchars[arraysize(idiotlights)][3] = {
    "SL", "SR", "\xf7""E", "\xf7""W", "P\x13", 
    "RC", "SI", "Pk", "AS", "Aj",
    "HM", "St", "BB", "Sm", "DB",
    "PU",
};  // "c3", "c4" };
uint16_t idiotcolors[arraysize(idiotlights)];
uint8_t idiot_saturation = 225;  // 170-195 makes nice bright yet distinguishable colors
uint8_t idiot_hue_offset = 240;
bool idiotlasts[arraysize(idiotlights)];
// 11x7 pixel idiot light bitmaps.  Format: Each byte is one pixel column (left->right) with LSB->MSB within each byte being the top->bottom pixels in that column
// The high bit (bottom pixel) of every byte is 0 due to 7-pixel height. Set high bit of first byte to 1 to skip bitmap and use letters instead
uint8_t idiotmaps[arraysize(idiotlights)][11] = {
    { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x3e, 0x71, 0x59, 0x4d, 0x47, 0x3e, },     // 0 = "S" w/ crossout symbol
    { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x78, 0x70, 0x58, 0x0d, 0x07, 0x0f, },     // 1 = "S" w/ double arrow
    { 0x7f, 0x7f, 0x6b, 0x6b, 0x00, 0x70, 0x12, 0x17, 0x75, 0x67, 0x02, },     // 2 = "En" w/ degree symbol
    { 0x7f, 0x30, 0x18, 0x30, 0x7f, 0x10, 0x12, 0x77, 0x65, 0x07, 0x02, },     // 3 = "Wh" w/ degree symbol
    { 0x7f, 0x7f, 0x09, 0x0f, 0x76, 0x10, 0x70, 0x60, 0x00, 0x6f, 0x6f, },     // 4 = "Pn!"
    { 0x63, 0x36, 0x1c, 0x36, 0x63, 0x14, 0x08, 0x22, 0x1c, 0x41, 0x3e, },     // 5 = wifi symbol w/ X
    { 0x16, 0x15, 0x0d, 0x60, 0x6f, 0x04, 0x6f, 0x60, 0x0f, 0x69, 0x66, },     // 6 = "SHD..."
    { 0x3e, 0x63, 0x41, 0x7d, 0x7d, 0x55, 0x55, 0x5d, 0x49, 0x63, 0x3e, },     // 7 = circle-"P"
    { 0x50, 0x40, 0x40, 0x1c, 0x36, 0x7b, 0x4d, 0x47, 0x42, 0x66, 0x3c, },     // 8 = skid wheel w/ brake caliper
    { 0x08, 0x1c, 0x36, 0x00, 0x3e, 0x63, 0x63, 0x00, 0x36, 0x1c, 0x08, },     // 9 = "<C>"
    { 0x1d, 0x23, 0x47, 0x00, 0x3e, 0x63, 0x55, 0x49, 0x55, 0x63, 0x3e, },     // 10 = rotation arrow w/ X wheel
    { 0x3e, 0x41, 0x7f, 0x79, 0x71, 0x73, 0x3e, 0x1c, 0x7f, 0x55, 0x7f, },     // 11 = motor w/ spur gear
    { 0x01, 0x7f, 0x7f, 0x7f, 0x3f, 0x38, 0x74, 0x70, 0x70, 0x70, 0x60, },     // 12 = boot
    { 0x6e, 0x6b, 0x3b, 0x00, 0x7f, 0x00, 0x7f, 0x06, 0x1c, 0x06, 0x7f, },     // 13 = "SIM"
    { 0x7f, 0x63, 0x3e, 0x00, 0x7f, 0x6b, 0x6b, 0x00, 0x7f, 0x30, 0x1f, },     // 14 = "DEV"
    { 0x00, 0x3e, 0x63, 0x41, 0x40, 0x4f, 0x40, 0x41, 0x63, 0x3e, 0x00, },     // 15 = power symbol
};
//  { 0x56, 0x5d, 0x42, 0x5f, 0x42, 0x4c, 0x52, 0x4c, 0x5e, 0x4a, 0x44, },     // 8 = underline "Stop"
//  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, }, };  // N/A (no idiot light bitmap)
void set_idiotcolors() {
    for (int32_t idiot=0; idiot<arraysize(idiotlights); idiot++) {
        int division = disp_idiots_per_row;
        uint32_t color32 = hsv_to_rgb<uint32_t>((int8_t)(255 * (idiot % division) / division + idiot_hue_offset), idiot_saturation, 255, 0, 220);
        idiotcolors[idiot] = color_uint32_to_16b(color32);  // 5957 = 2^16/11
        if (gamma_correct_enabled) idiotcolors[idiot] = gamma16(idiotcolors[idiot]);
        disp_idiots_dirty = true;
    }
}

// tuning-ui related globals
enum disp_draw { ERASE = -1 };
enum tunctrls { OFF, SELECT, EDIT };
int32_t tunctrl = OFF, tunctrl_last = OFF;
int32_t datapage = PG_RUN, datapage_last = PG_TEMP;  // Which of the six 8-value dataset pages is currently displayed, and available to edit
int32_t sel_val = 0, sel_val_last = 0;  // In the real time tuning UI, which of the editable values (0-7) is selected. -1 for none 
Timer tuningCtrlTimer (25000000);  // This times out edit mode after a a long period of inactivity

class Display {
    private:
        TFT_eSPI _tft = TFT_eSPI();
        TFT_eSprite _saver = TFT_eSprite(&_tft);  // Declare screensaver sprite object with pointer to tft object
        uint16_t touch_cal_data[5] = { 404, 3503, 460, 3313, 1 };  // Got from running TFT_eSPI/examples/Generic/Touch_calibrate/Touch_calibrate.ino
        Timer _tftResetTimer, _tftDelayTimer;
        int32_t _timing_tft_reset;
        bool _procrastinate = false, reset_finished = false, saver_lotto = false, simulating_last;        
        // For screensaver sprite
        long sx0, sy0;
        long touchpoint_x = -1, touchpoint_y = -1, eraser_rad = 14, eraser_rad_min = 9, eraser_rad_max = 27, eraser_velo_min = 4, eraser_velo_max = 10;
        long eraser_pos[2] = { 0, 0 };
        long eraser_velo[2] = { random(eraser_velo_max), random(eraser_velo_max) };
        long eraser_pos_max[2] = { disp_saver_width / 2 - eraser_rad, disp_saver_height / 2 - eraser_rad }; 
        long eraser_velo_sign[2] = { 1, 1 };
        uint8_t saver_illicit_prob = 15, penhue = 0, spothue = 255;
        float pensat = 200.0;
        uint16_t pencolor = RED;
        uint32_t pentimeout = 700000;
        int savernumcycles, savershape_last;
        int savershapes = 5, savercycle = 1, savtouch_last_x = -1, savtouch_last_y = -1, savtouch_last_w = 2, savershape = random(savershapes);
        uint32_t saver_cycletime_us = 38000000, saver_refresh_us = 45000;
        Timer saverRefreshTimer, saverCycleTimer, pentimer;
        uint32_t disp_oldmode = SHUTDOWN;   // So we can tell when the mode has just changed. start as different to trigger_mode start algo
    public:
        Display (int8_t cs_pin, int8_t dc_pin) : _tft(cs_pin, dc_pin), _tftResetTimer(100000), _tftDelayTimer(3000000), _timing_tft_reset(0) {}
        Display () : _tft(), _tftResetTimer(100000), _tftDelayTimer(3000000), _timing_tft_reset(0) {}
        void init() {
            yield();
            _tft.begin();
            _tft.setRotation((flip_the_screen) ? 3 : 1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
            _tft.setTouch(touch_cal_data);
            for (int32_t lineno=0; lineno <= disp_fixed_lines; lineno++)  {
                disp_age_quanta[lineno] = -1;
                memset (disp_values[lineno], 0, strlen (disp_values[lineno]));
                disp_polarities[lineno] = 1;
            }
            for (int32_t row=0; row<arraysize (disp_bool_values); row++) disp_bool_values[row] = 1;
            for (int32_t row=0; row<arraysize (disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
            for (int32_t row=0; row<arraysize (disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen
            yield();
            // set_runmodecolors();
            _tft.fillScreen (BLK);  // Black out the whole screen
            draw_touchgrid (false);
            draw_fixed (datapage, datapage_last, false);
            yield();
            set_idiotcolors();
            draw_idiotlights(disp_idiot_corner_x, disp_idiot_corner_y, true);
            all_dirty();
            saver_setup();
        }
        bool tft_reset() {  // call to begin a tft reset, and continue to call every loop until returns true (or get_reset_finished() returns true), then stop
            if (reset_finished) {
                reset_finished = false;
                _timing_tft_reset = 1;
             }
            if (_timing_tft_reset == 1) {
                write_pin (tft_rst_pin, LOW);
                _timing_tft_reset = 2;
            }
            else if (_timing_tft_reset == 2 && _tftResetTimer.expired()) {
                write_pin (tft_rst_pin, HIGH);
                init();
                _timing_tft_reset = 0;
                reset_finished = true;
            }
            return reset_finished;
        }
        void all_dirty() {
            disp_idiots_dirty = true;
            disp_data_dirty = true;
            disp_selected_val_dirty = true;
            disp_datapage_dirty = true;
            disp_sidemenu_dirty = true;
            disp_runmode_dirty = true;
            disp_simbuttons_dirty = true;
            screensaver = false;
        }
        void watchdog() {  // Call in every loop to perform a reset upon detection of blocked loops and 
            if (loop_periods_us[loop_now] > tft_watchdog_timeout_us && _timing_tft_reset == 0) _timing_tft_reset = 1;
            if (_timing_tft_reset == 0 || !_tftDelayTimer.expired()) _tftDelayTimer.reset();
            else tft_reset();
        }
        bool get_reset_finished() { return reset_finished; }
        void set_runmodecolors() {
            uint8_t saturat = 255;  uint8_t hue_offset = 0;
            for (int32_t rm=0; rm<arraysize(colorcard); rm++) {
                int division = num_runmodes;
                uint32_t color32 = hsv_to_rgb<uint32_t>((int8_t)(255 * (rm % division) / division + hue_offset), saturat, 255, 0, 220);
                colorcard[rm] = color_uint32_to_16b(color32);  // 5957 = 2^16/11
                if (gamma_correct_enabled) colorcard[rm] = gamma16(colorcard[rm]);
                disp_runmode_dirty = true;
            }
        }
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
        void draw_unitmap (int8_t index, int32_t x, int32_t y, uint16_t color) {
            for (int32_t xo = 0; xo < disp_font_width * 3 - 1; xo++)
                for (int32_t yo = 0; yo < disp_font_height - 1; yo++)
                    if ((unitmaps[index][xo] >> yo) & 1) _tft.drawPixel (x + xo, y + yo, color);
        }
        void draw_string_units (int32_t x, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
            bool drawn = false;
            for (int8_t i = 0; i<arraysize(unitmaps); i++)
                if (!strcmp(unitmapnames[i], oldtext)) {
                    draw_unitmap(i, x, y, bgcolor);
                    drawn = true;
                }
            if (!drawn) {
                _tft.setCursor (x, y);
                _tft.setTextColor (bgcolor);
                _tft.print (oldtext);  // Erase the old content
            }
            for (int8_t i = 0; i<arraysize(unitmaps); i++)
                if (!strcmp(unitmapnames[i], text)) {
                    draw_unitmap(i, x, y, color);
                    return;
                }
            _tft.setCursor (x, y);
            _tft.setTextColor (color);
            _tft.print (text);  // Erase the old content
        }
        // draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
        void draw_fixed (int32_t page, int32_t page_last, bool redraw_tuning_corner, bool forced=false) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
            _tft.setTextColor (GRY2);
            _tft.setTextSize (1);
            int32_t y_pos;
            if (!redraw_tuning_corner) {
                for (int32_t lineno = 0; lineno < disp_fixed_lines; lineno++)  {  // Step thru lines of fixed telemetry data
                    y_pos = (lineno + 1) * disp_line_height_pix + disp_vshift_pix;
                    draw_string (disp_datapage_names_x, disp_datapage_names_x, y_pos, telemetry[lineno], "", GRY2, BLK, forced);
                    draw_string_units (disp_datapage_units_x, y_pos, units[lineno], "", GRY2, BLK);
                    draw_bargraph_base (disp_bargraphs_x, y_pos + 7, disp_bargraph_width);
                }
            }
            for (int32_t lineno=0; lineno < disp_tuning_lines; lineno++)  {  // Step thru lines of dataset page data
                draw_string (disp_datapage_names_x, disp_datapage_names_x, (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix, datapage_names[page][lineno], datapage_names[page_last][lineno], GRY2, BLK, forced);
                draw_string_units (disp_datapage_units_x, (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix, tuneunits[page][lineno], tuneunits[page_last][lineno], GRY2, BLK);
                if (redraw_tuning_corner) {
                    int32_t corner_y = (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix + 7;  // lineno*disp_line_height_pix+disp_vshift_pix-1;
                    draw_bargraph_base (disp_bargraphs_x, corner_y, disp_bargraph_width);
                    if (disp_needles[lineno] >= 0) draw_bargraph_needle (-1, disp_needles[lineno], corner_y - 6, BLK);  // Let's draw a needle
                }
            }
        }
        void draw_hyphen (int32_t x_pos, int32_t y_pos, int32_t color) {  // Draw minus sign in front of negative numbers
            _tft.drawFastHLine (x_pos+2, y_pos+3, 3, color);
        }
        void draw_dynamic (int32_t lineno, char const* disp_string, int32_t value, int32_t lowlim, int32_t hilim, int32_t target=-1, int32_t color=-1) {
            int32_t age_us = (color >= 0) ? 11 : (int32_t)((float)(dispAgeTimer[lineno].elapsed()) / 2500000); // Divide by us per color gradient quantum
            int32_t x_base = disp_datapage_values_x;
            bool polarity = (value >= 0);  // polarity 0=negative, 1=positive
            if (strcmp(disp_values[lineno], disp_string) || value == 1234567 || disp_data_dirty) {  // If value differs, Erase old value and write new
                if (color == -1) color = GRN;
                int32_t y_pos = lineno*disp_line_height_pix+disp_vshift_pix;
                if (polarity != disp_polarities[lineno]) draw_hyphen (x_base, y_pos, (!polarity) ? color : BLK);
                draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_string, disp_values[lineno], color, BLK); // +6*(arraysize(modecard[run.mode()])+4-namelen)/2
                strcpy (disp_values[lineno], disp_string);
                disp_polarities[lineno] = polarity;
                dispAgeTimer[lineno].reset();
                disp_age_quanta[lineno] = 0;
            }  // to-do: Fix failure to freshen aged coloration of unchanged characters of changed values
            else if (age_us > disp_age_quanta[lineno] && age_us < 11)  {  // As readings age, redraw in new color. This may fail and redraw when the timer overflows? 
                if (age_us < 8) color = 0x1fe0 + age_us*0x2000;  // Base of green with red added as you age, until yellow is achieved
                else color = 0xffe0 - (age_us-8) * 0x100;  // Then lose green as you age further
                int32_t y_pos = (lineno)*disp_line_height_pix+disp_vshift_pix;
                if (!polarity) draw_hyphen (x_base, y_pos, color);
                draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_values[lineno], "", color, BLK);
                disp_age_quanta[lineno] = age_us;
            }
            if (lowlim < hilim) {  // Any value having a given range deserves a bargraph gauge with a needle
                int32_t corner_x = disp_bargraphs_x;    
                int32_t corner_y = lineno*disp_line_height_pix+disp_vshift_pix-1;
                int32_t n_pos = map (value, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                int32_t ncolor = (n_pos > disp_bargraph_width-disp_bargraph_squeeze || n_pos < disp_bargraph_squeeze) ? BRN : GRN;
                n_pos = corner_x + constrain (n_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                if (target != -1) {  // If target value is given, draw a target on the bargraph too
                    int32_t t_pos = map (target, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                    int32_t tcolor = (t_pos > disp_bargraph_width-disp_bargraph_squeeze || t_pos < disp_bargraph_squeeze) ? BRN : ( (t_pos != n_pos) ? YEL : GRN );
                    t_pos = corner_x + constrain (t_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                    if (t_pos != disp_targets[lineno] || (t_pos == n_pos)^(disp_needles[lineno] != disp_targets[lineno]) || disp_data_dirty) {
                        draw_target_shape (disp_targets[lineno], corner_y, BLK, -1);  // Erase old target
                        _tft.drawFastHLine (disp_targets[lineno]-(disp_targets[lineno] != corner_x+disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+7, 2+(disp_targets[lineno] != corner_x+disp_bargraph_width-disp_bargraph_squeeze), GRY1);  // Patch bargraph line where old target got erased
                        for (int32_t offset=0; offset<=2; offset++) _tft.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(disp_bargraph_width/2 - disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+6, 3, WHT);  // Redraw bargraph graduations in case one got corrupted by target erasure
                        draw_target_shape (t_pos, corner_y, tcolor, -1);  // Draw the new target
                        disp_targets[lineno] = t_pos;  // Remember position of target
                    }
                }
                if (n_pos != disp_needles[lineno] || disp_data_dirty) {
                    draw_bargraph_needle (n_pos, disp_needles[lineno], corner_y, ncolor);  // Let's draw a needle
                    disp_needles[lineno] = n_pos;  // Remember position of needle
                }
            }
            else if (disp_needles[lineno] >= 0) {  // If value having no range is drawn over one that did ...
                draw_bargraph_needle (-1, disp_needles[lineno], lineno*disp_line_height_pix+disp_vshift_pix-1, BLK);  // Erase the old needle
                disp_needles[lineno] = -1;  // Flag for no needle
            }
        }
        int32_t significant_place (float value) {  // Returns the decimal place of the most significant digit of a given float value, without relying on logarithm math
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
        int32_t significant_place (int32_t value) {  // Returns the decimal place of the most significant digit of a given float value, without relying on logarithm math
            int32_t place = 1;
            while (value >= 10) {
                value /= 10;
                place++;
            }
            return place;
        }
        std::string abs_itoa (int32_t value, int32_t maxlength) {  // returns an ascii string representation of a given integer value, using scientific notation if necessary to fit within given width constraint
            value = abs (value);  // This function disregards sign
            int32_t magnitude = significant_place (value);  // check how slow is log() function? Compare performance vs. multiple divides ( see abs_ftoa() )
            if (magnitude <= maxlength) return std::to_string (value);  // If value is short enough, return it
            else return std::to_string ((float)value / (float)magnitude) + "e" + std::to_string (magnitude);
        }
        std::string abs_ftoa (float value, int32_t maxlength, int32_t sigdig) {  // returns an ascii string representation of a given float value, formatted to efficiently fit withinthe given width constraint
            value = abs (value);  // This function disregards sign
            int32_t place = significant_place (value);  // Learn decimal place of the most significant digit in value
            if (place >= sigdig && place <= maxlength) {  // Then we want simple cast to an integer w/o decimal point (eg 123456, 12345, 1234)
                std::string result (std::to_string ((int32_t)value));
                return result;
            }
            if (place >= 0 && place < maxlength) {  // Then we want float formatted with enough nonzero digits after the decimal point for 4 significant digits (eg 123.4, 12.34, 1.234, 0.000)
                int32_t length = smin(sigdig+1, maxlength);
                char buffer[length+1];
                std::snprintf (buffer, length + 1, "%.*g", length - 1, value);  // (buf, chars incl. end, %.*g = floats formatted in shortest form, length-1 digits after decimal, val)
                std::string result (buffer);  // copy buffer to result
                return result;
            }
            if (place >= 3-maxlength && place < maxlength) {  // Then we want decimal w/o initial '0' limited to 3 significant digits (eg .123, .0123, .00123)
                std::string result (std::to_string(value));
                size_t decimalPos = result.find ('.');  // Remove any digits to the left of the decimal point
                if (decimalPos != std::string::npos) result = result.substr (decimalPos);
                if (result.length() > sigdig) result.resize (sigdig+1);  // Limit the string length to the desired number of significant digits
                return result;
            }  // Otherwise we want scientific notation with precision removed as needed to respect maxlength (eg 1.23e4, 1.23e5, but using long e character not e for negative exponents
            char buffer[maxlength+1];  // Allocate buffer with the maximum required size
            int32_t sigdigless = sigdig - 1 - (place <= -10);  // was: if (place <= -10) return std::string ("~0");  // Ridiculously small values just indicate basically zero
            snprintf (buffer, sizeof (buffer), "%*.*f%*d", maxlength-sigdigless, sigdigless, value, maxlength-1, 0);
            std::string result (buffer);  // copy buffer to result
            if (result.find ("e+0") != std::string::npos) result.replace (result.find ("e+0"), 3, "e");  // Remove useless "+0" from exponent
            else if (result.find ("e-0") != std::string::npos) result.replace (result.find ("e-0"), 3, "\x88");  // For very small scientific notation values, replace the "e-0" with a phoenetic long e character, to indicate negative power  // if (result.find ("e-0") != std::string::npos) 
            else if (result.find ("e+") != std::string::npos) result.replace (result.find ("e+"), 3, "e");  // For ridiculously large values
            else if (result.find ("e-") != std::string::npos) result.replace (result.find ("e-"), 3, "\x88");  // For ridiculously small values
            return result;    
        }
        void draw_dynamic (int32_t lineno, int32_t value, int32_t lowlim=-1, int32_t hilim=-1, int32_t target=-1) {
            std::string val_string = abs_itoa (value, (int32_t)disp_maxlength);
            draw_dynamic (lineno, val_string.c_str(), value, lowlim, hilim, (int32_t)target);
        }
        void draw_dynamic (int32_t lineno, float value, float lowlim=-1, float hilim=-1, int32_t target=-1, int32_t precision = disp_default_float_precision) {
            std::string val_string = abs_ftoa (value, (int32_t)disp_maxlength, precision);
            draw_dynamic (lineno, val_string.c_str(), (int32_t)value, (int32_t)lowlim, (int32_t)hilim, target);
        }
        void draw_dynamic (int32_t lineno, float value, float lowlim, float hilim, float target, int32_t precision = disp_default_float_precision) {
            draw_dynamic (lineno, value, lowlim, hilim, (int32_t)target, precision);
        }
        void draw_eraseval (int32_t lineno) {
            draw_dynamic (lineno, "", 1234567, -1, -1, -1);
        }
        void draw_asciiname (int32_t lineno, String name) {
            draw_dynamic (lineno, name.c_str(), 1, -1, -1, -1, CYN);
        }
        void draw_truth (int32_t lineno, bool truthy, int32_t styl=2) {  // 0:on/off, 1:yes/no, 2:true/false .
            draw_dynamic (lineno, (truthy) ? ((styl==0) ? " on" : ((styl==1) ? "yes" : "true")) : ((styl==0) ? "off" : ((styl==1) ? "no" : "false")), 1, -1, -1, -1, (truthy) ? LPUR : GPUR);
        }
        void draw_runmode (int32_t _nowmode, int32_t _oldmode, int32_t color_override=-1) {  // color_override = -1 uses default color
            int32_t color = (color_override == -1) ? colorcard[_nowmode] : color_override;
            int32_t x_new = disp_runmode_text_x + disp_font_width * (2 + strlen (modecard[_nowmode])) - 3;
            int32_t x_old = disp_runmode_text_x + disp_font_width * (2 + strlen (modecard[_oldmode])) - 3;
            draw_string (disp_runmode_text_x + disp_font_width, disp_runmode_text_x + disp_font_width, disp_vshift_pix, modecard[_oldmode], "", BLK, BLK); // +6*(arraysize(modecard[_nowmode])+4-namelen)/2
            draw_string (x_old, x_old, disp_vshift_pix, "Mode", "", BLK, BLK); // +6*(arraysize(modecard[_nowmode])+4-namelen)/2
            draw_string (disp_runmode_text_x + disp_font_width, disp_runmode_text_x + disp_font_width, disp_vshift_pix, modecard[_nowmode], "", color, BLK); // +6*(arraysize(modecard[_nowmode])+4-namelen)/2
            draw_string (x_new, x_new, disp_vshift_pix, "Mode", "", color, BLK); // +6*(arraysize(modecard[_nowmode])+4-namelen)/2
        }
        void draw_datapage (int32_t page, int32_t page_last, bool forced=false) {
            draw_fixed (page, page_last, true, forced);  // Erase and redraw dynamic data corner of screen with names, units etc.
            draw_string (disp_datapage_title_x, disp_datapage_title_x, disp_vshift_pix, pagecard[page], pagecard[page_last], STBL, BLK, forced); // +6*(arraysize(modecard[_runmode.mode()])+4-namelen)/2
        }
        void draw_selected_name (int32_t tun_ctrl, int32_t tun_ctrl_last, int32_t selected_val, int32_t selected_last) {
            if (selected_val != selected_last) draw_string (12, 12, 12+(selected_last+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, datapage_names[datapage][selected_last], "", GRY2, BLK);
            draw_string (12, 12, 12+(selected_val+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, datapage_names[datapage][selected_val], "", (tun_ctrl == EDIT) ? GRN : ((tun_ctrl == SELECT) ? YEL : GRY2), BLK);
        }
        void draw_bool (bool value, int32_t col) {  // Draws values of boolean data
            if ((disp_bool_values[col-2] != value) || disp_data_dirty) {  // If value differs, Erase old value and write new
                int32_t x_mod = touch_margin_h_pix + touch_cell_h_pix*(col) + (touch_cell_h_pix>>1) - arraysize (top_menu_buttons[col-2]-1)*(disp_font_width>>1) - 2;
                draw_string (x_mod, x_mod, 0, top_menu_buttons[col-2], "", (value) ? GRN : LGRY, DGRY);
                disp_bool_values[col-2] = value;
            }
        }
        void draw_simbuttons (bool create) {  // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
            _tft.fillRect(disp_simbuttons_x, disp_simbuttons_y, disp_saver_width, disp_saver_height, BLK);
            _tft.setTextColor (LYEL);
            for (int32_t row = 0; row < arraysize(simgrid); row++) {
                for (int32_t col = 0; col < arraysize(simgrid[row]); col++) {
                    int32_t cntr_x = touch_margin_h_pix + touch_cell_h_pix*(col+3) + (touch_cell_h_pix>>1) +2;
                    int32_t cntr_y = touch_cell_v_pix*(row+1) + (touch_cell_v_pix>>1);
                    if (strcmp (simgrid[row][col], ______ )) {
                        _tft.fillCircle (cntr_x, cntr_y, disp_simbutton_radius_pix, create ? DGRY : BLK);
                        _tft.drawCircle (cntr_x, cntr_y, 19, create ? LYEL : BLK);
                        if (create) {
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
                _tft.fillRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, DGRY);
                _tft.drawRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, LYEL);
                namelen = 0;
                for (uint32_t x = 0 ; x < arraysize (side_menu_buttons[row]) ; x++ ) {
                    if (side_menu_buttons[row][x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
                }
                for (int32_t letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
                    _tft.setCursor (1, ( touch_cell_v_pix*row) + (touch_cell_v_pix/2) - (int32_t)(4.5*((float)namelen-1)) + (disp_font_height+1)*letter); // adjusts vertical offset depending how many letters in the button name and which letter we're on
                    _tft.println (side_menu_buttons[row][letter]);  // Writes each letter such that the whole name is centered vertically on the button
                }
            }
            if (!side_only) {
                for (int32_t col = 2; col <= 5; col++) {  // Step thru all cols to draw buttons across the top edge
                    _tft.fillRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
                    _tft.drawRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, LYEL);  // _tft.width()-9, 3, 18, (_tft.height()/5)-6, 8, LYEL);
                }
            }
        }
        void draw_reticle(uint32_t x, uint32_t y) {
            _tft.drawFastHLine (x - 2, y, 5, DGRY);
            _tft.drawFastVLine (x, y - 2, 5, DGRY);
        }
        void draw_reticles() {
            draw_reticle(260, 50);
            draw_reticle(60, 50);
            draw_reticle(60, 189);
            draw_reticle(260, 189);
        }
        uint16_t darken_color (uint16_t color, int32_t halvings = 1) {  // halves each of r, g, and b of a 5-6-5 formatted 16-bit color value either once or twice
            if (halvings == 1) return ((color & 0xf000) | (color & 0x7c0) | (color & 0x1e)) >> 1;
            else return ((color & 0xe000) | (color & 0x780) | (color & 0x1c)) >> 2;
        }
        void draw_idiotbitmap (int32_t idiot, int32_t x, int32_t y) {
            uint16_t bg = (*idiotlights[idiot]) ? idiotcolors[idiot] : BLK;
            uint16_t color = (*idiotlights[idiot]) ? BLK : darken_color(idiotcolors[idiot]);
            _tft.drawRoundRect (x, y, 2 * disp_font_width + 1, disp_font_height + 1, 1, bg);
            for (int32_t xo = 0; xo < (2 * disp_font_width - 1); xo++)
                for (int32_t yo = 0; yo < disp_font_height - 1; yo++)
                    _tft.drawPixel (x + xo + 1, y + yo + 1, ((idiotmaps[idiot][xo] >> yo) & 1) ? color : bg);
        }
        void draw_idiotlight (int32_t idiot, int32_t x, int32_t y) {
            if (idiotmaps[idiot][0] >= 0x80) {
                _tft.fillRoundRect (x, y, 2 * disp_font_width + 1, disp_font_height + 1, 2, (*(idiotlights[idiot])) ? idiotcolors[idiot] : BLK);  // GRY1);
                _tft.setTextColor ((*(idiotlights[idiot])) ? BLK : darken_color(idiotcolors[idiot]));  // darken_color ((*(idiotlights[index])) ? BLK : DGRY)
                _tft.setCursor (x+1, y+1);
                _tft.print (idiotchars[idiot]);
            }
            else draw_idiotbitmap(idiot, x, y);
            idiotlasts[idiot] = *(idiotlights[idiot]);
        }
        void draw_idiotlights (int32_t x, int32_t y, bool force = false) {
            for (int32_t ilite=0; ilite < arraysize(idiotlights); ilite++)
                if (force || (*(idiotlights[ilite]) ^ idiotlasts[ilite]))
                    draw_idiotlight (ilite, x + (2 * disp_font_width + 2) * ((ilite % disp_idiots_per_row) % disp_idiots_per_row), y + disp_idiot_row_height * (int32_t)(ilite / disp_idiots_per_row));
        }
        void draw_temperature(loc location, int draw_index) {
            if (!tempsens.detected(location)) draw_eraseval(draw_index);
            else draw_dynamic(draw_index, tempsens.val(location), temp_lims_f[tempsens.errclass(location)][DISP_opmin], temp_lims_f[tempsens.errclass(location)][DISP_MAX]);
        }
        void update_idiots(bool force = false) {
            draw_idiotlights(disp_idiot_corner_x, disp_idiot_corner_y, force);
        }
        void update(runmode _nowmode) {
            if (!display_enabled) return;
            update_idiots(disp_idiots_dirty);
            disp_idiots_dirty = false;
            if (disp_simbuttons_dirty || (sim.enabled() != simulating_last)) {
                draw_simbuttons(sim.enabled());  // if we just entered simulator draw the simulator buttons, or if we just left erase them
                disp_simbuttons_dirty = false;
                simulating_last = sim.enabled();
            }
            if ((disp_datapage_dirty)) {
                static bool first = true;
                draw_datapage(datapage, datapage_last, first);
                first = false;
                disp_datapage_dirty = false;
                if (datapage_last != datapage) config.putUInt("dpage", datapage);
            }
            if ((disp_sidemenu_dirty)) {
                draw_touchgrid(true);
                disp_sidemenu_dirty = false;
            }
            if (disp_selected_val_dirty) {
                draw_selected_name(tunctrl, tunctrl_last, sel_val, sel_val_last);
                disp_selected_val_dirty = false;
            }
            if (disp_runmode_dirty) {
                draw_runmode(_nowmode, disp_oldmode, -1);
                disp_oldmode = _nowmode;
                disp_runmode_dirty = false;
            }
            if (dispRefreshTimer.expired()) {
                dispRefreshTimer.reset();
                float drange;
                draw_dynamic(1, hotrc.pc[vert][filt], hotrc.pc[vert][opmin], hotrc.pc[vert][opmax]);
                draw_dynamic(2, speedo.filt(), 0.0, speedo.redline_mph(), gas.cruisepid.target());
                draw_dynamic(3, tach.filt(), 0.0, tach.redline_rpm(), gas.mypid.target());
                draw_dynamic(4, gas.pc[out], gas.pc[opmin], gas.pc[opmax]);
                draw_dynamic(5, pressure.filt(), pressure.min_human(), pressure.max_human(), brake.mypid.target());  // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.targ() : pressure_target_adc);
                draw_dynamic(6, brake.pc[out], brake.pc[opmin], brake.pc[opmax]);
                draw_dynamic(7, hotrc.pc[horz][filt], hotrc.pc[horz][opmin], hotrc.pc[horz][opmax]);
                draw_dynamic(8, steer.pc[out], steer.pc[opmin], steer.pc[opmax]);
                if (datapage == PG_RUN) {
                    draw_dynamic(9, brakepos.filt(), brakepos.op_min_in(), brakepos.op_max_in());
                    draw_dynamic(10, mulebatt.filt(), mulebatt.op_min_v(), mulebatt.op_max_v());
                    draw_dynamic(11, lipobatt.filt(), lipobatt.op_min_v(), lipobatt.op_max_v());
                    draw_dynamic(12, pot.val(), pot.min(), pot.max());
                    draw_dynamic(13, airvelo.filt(), airvelo.min_mph(), airvelo.max_mph());
                    draw_dynamic(14, mapsens.filt(), mapsens.min_psi(), mapsens.max_psi());
                    draw_dynamic(15, maf_gps, maf_min_gps, maf_max_gps);
                    for (int line=16; line<=17; line++) draw_eraseval(line);
                    draw_dynamic(18, gas.governor, 0.0, 100.0);
                    draw_dynamic(19, steer.steer_safe_pc, 0.0, 100.0);
                }
                else if (datapage == PG_JOY) {
                    draw_dynamic(9, hotrc.us[horz][raw], hotrc.us[horz][opmin], hotrc.us[horz][opmax]);
                    draw_dynamic(10, hotrc.us[vert][raw], hotrc.us[vert][opmin], hotrc.us[vert][opmax]);
                    draw_dynamic(11, hotrc.us[ch3][raw], hotrc.us[ch3][opmin], hotrc.us[ch3][opmax]);
                    draw_dynamic(12, hotrc.us[ch4][raw], hotrc.us[ch4][opmin], hotrc.us[ch4][opmax]);
                    draw_dynamic(13, hotrc.pc[horz][raw], hotrc.pc[horz][opmin], hotrc.pc[horz][opmax]);
                    draw_dynamic(14, hotrc.pc[vert][raw], hotrc.pc[vert][opmin], hotrc.pc[vert][opmax]);
                    for (int line=15; line<=17; line++) draw_eraseval(line);
                    draw_dynamic(18, hotrc.failsafe_us, hotrc.absmin_us, hotrc.us[vert][opmin] - hotrc.us[vert][margin]);
                    draw_dynamic(19, hotrc.deadband_us, 0, 100);
                }
                else if (datapage == PG_SENS) {
                    draw_dynamic(9, pressure.raw(), pressure.min_native(), pressure.max_native());                    
                    draw_dynamic(10, brakepos.raw(), brakepos.min_native(), brakepos.max_native());                    
                    for (int line=11; line<=13; line++) draw_eraseval(13);
                    draw_dynamic(14, airvelo.max_mph(), 0.0, airvelo.abs_max_mph());
                    draw_dynamic(15, mapsens.min_psi(), mapsens.abs_min_psi(), mapsens.abs_max_psi());
                    draw_dynamic(16, mapsens.max_psi(), mapsens.abs_min_psi(), mapsens.abs_max_psi());
                    draw_dynamic(17, speedo.idle_mph(), 0.0, speedo.redline_mph());
                    draw_dynamic(18, speedo.redline_mph(), 0.0, speedo.max_human());
                    draw_dynamic(19, brakepos.zeropoint(), brakepos.min_human(), brakepos.max_human());  // BrakePositionSensor::abs_min_retract_in, BrakePositionSensor::abs_max_extend_in);
                }
                else if (datapage == PG_PWMS) {
                    draw_dynamic(9, gas.deg[out], gas.deg[opmin], gas.deg[opmax]);
                    draw_dynamic(10, gas.us[out], gas.us[absmin], gas.us[absmax]);
                    draw_dynamic(11, brake.volt[out], brake.volt[opmin], brake.volt[opmax]);
                    draw_dynamic(12, brake.us[out], brake.us[absmin], brake.us[absmax]);
                    draw_dynamic(13, steer.volt[out], steer.volt[opmin], steer.volt[opmax]);
                    draw_dynamic(14, steer.us[out], steer.us[absmin], steer.us[absmax]);
                    draw_eraseval(15);
                    draw_dynamic(16, gas.deg[opmin], gas.deg[absmax], gas.deg[absmax]);
                    draw_dynamic(17, gas.deg[opmax], gas.deg[absmax], gas.deg[absmax]);
                    draw_dynamic(18, brake.us[stop], brake.us[absmin], brake.us[absmax]);
                    draw_dynamic(19, brake.duty_pc, 0.0, 100.0);
                }
                else if (datapage == PG_IDLE) {
                    draw_asciiname(9, idlestatecard[throttle.targetstate]);
                    draw_dynamic(10, gas.mypid.target(), 0.0, tach.redline_rpm());
                    draw_dynamic(11, throttle.stallpoint, throttle.idle_absmin, throttle.idle_absmax);
                    draw_dynamic(12, throttle.idle_rpm, throttle.idle_absmin, throttle.idle_absmax);  // throttle.idlehot(), throttle.idlecold());
                    draw_dynamic(13, throttle.idlehigh, throttle.idle_absmin, throttle.idle_absmax);
                    draw_dynamic(14, throttle.idlecold, throttle.idle_absmin, throttle.idle_absmax, -1, 4);
                    draw_dynamic(15, throttle.idlehot, throttle.idle_absmin, throttle.idle_absmax, -1, 4);
                    draw_dynamic(16, throttle.tempcold, temp_lims_f[ENGINE][DISP_opmin], temp_lims_f[ENGINE][DISP_MAX]);
                    draw_dynamic(17, throttle.temphot, temp_lims_f[ENGINE][DISP_opmin], temp_lims_f[ENGINE][DISP_MAX]);
                    draw_dynamic(18, (int32_t)throttle.settlerate_rpmps, 0, 500);
                    draw_asciiname(19, idlemodecard[(int32_t)throttle.idlemode]);
                }
                else if (datapage == PG_BPID) {
                    drange = brake.us[absmin]-brake.us[absmax];
                    draw_dynamic(9, brake.mypid.target(), pressure.min_human(), pressure.max_human());
                    draw_dynamic(10, brake.mypid.err(), pressure.min_human()-pressure.max_human(), pressure.max_human()-pressure.min_human());
                    draw_dynamic(11, brake.mypid.pterm(), -drange, drange);
                    draw_dynamic(12, brake.mypid.iterm(), -drange, drange);
                    draw_dynamic(13, brake.mypid.dterm(), -drange, drange);
                    draw_dynamic(14, brake.mypid.outsum(), -brake.mypid.outrange(), brake.mypid.outrange());  // brake_spid_speedo_delta_adc, -range, range);
                    draw_dynamic(15, brake.us[out], brake.us[absmin], brake.us[absmax]);
                    draw_dynamic(16, pressure.native(), pressure.min_native(), pressure.max_native());
                    draw_dynamic(17, brake.mypid.kp(), 0.0, 2.0);
                    draw_dynamic(18, brake.mypid.ki(), 0.0, 2.0);
                    draw_dynamic(19, brake.mypid.kd(), 0.0, 2.0);
                }
                else if (datapage == PG_GPID) {
                    draw_dynamic(9, gas.mypid.target(), 0.0, tach.redline_rpm());
                    draw_dynamic(10, gas.mypid.err(), throttle.idle_rpm - tach.govern_rpm(), tach.govern_rpm() - throttle.idle_rpm);
                    draw_dynamic(11, gas.mypid.pterm(), -100.0, 100.0);
                    draw_dynamic(12, gas.mypid.iterm(), -100.0, 100.0);
                    draw_dynamic(13, gas.mypid.dterm(), -100.0, 100.0);
                    draw_dynamic(14, gas.mypid.outsum(), -gas.mypid.outrange(), gas.mypid.outrange());
                    draw_eraseval(15);
                    draw_truth(16, gas.open_loop, 1);
                    draw_dynamic(17, gas.mypid.kp(), 0.0, 1.0);
                    draw_dynamic(18, gas.mypid.ki(), 0.0, 1.0);
                    draw_dynamic(19, gas.mypid.kd(), 0.0, 1.0);
                }
                else if (datapage == PG_CPID) {
                    drange = tach.govern_rpm() - throttle.idle_rpm;
                    draw_dynamic(9, gas.cruisepid.target(), 0.0, speedo.govern_mph());
                    draw_dynamic(10, gas.cruisepid.err(), speedo.idle_mph()-speedo.govern_mph(), speedo.govern_mph()-speedo.idle_mph());
                    draw_dynamic(11, gas.cruisepid.pterm(), -drange, drange);
                    draw_dynamic(12, gas.cruisepid.iterm(), -drange, drange);
                    draw_dynamic(13, gas.cruisepid.dterm(), -drange, drange);
                    draw_dynamic(14, gas.cruisepid.outsum(), -gas.cruisepid.outrange(), gas.cruisepid.outrange());  // cruise_spid_speedo_delta_adc, -drange, drange);
                    draw_dynamic(15, gas.cruise_target_pc, 0.0, 100.0);
                    draw_dynamic(16, cruise_delta_max_pc_per_s, 1, 1000);
                    draw_dynamic(17, gas.cruisepid.kp(), 0.0, 10.0);
                    draw_dynamic(18, gas.cruisepid.ki(), 0.0, 10.0);
                    draw_dynamic(19, gas.cruisepid.kd(), 0.0, 10.0);
                }
                else if (datapage == PG_TEMP) {
                    draw_temperature(loc::ambient, 9);
                    draw_temperature(loc::engine, 10);
                    draw_temperature(loc::wheel_fl, 11);
                    draw_temperature(loc::wheel_fr, 12);
                    draw_temperature(loc::wheel_rl, 13);
                    draw_temperature(loc::wheel_rr, 14);
                    for (int line=15; line<=18; line++) draw_eraseval(line);
                    draw_truth(19, dont_take_temperatures, 2);
                }
                else if (datapage == PG_SIM) {
                    draw_truth(9, sim.can_sim(sens::joy), 0);
                    draw_truth(10, sim.can_sim(sens::pressure), 0);
                    draw_truth(11, sim.can_sim(sens::brkpos), 0);
                    draw_truth(12, sim.can_sim(sens::speedo), 0);
                    draw_truth(13, sim.can_sim(sens::tach), 0);
                    draw_truth(14, sim.can_sim(sens::airvelo), 0);
                    draw_truth(15, sim.can_sim(sens::mapsens), 0);
                    draw_truth(16, sim.can_sim(sens::basicsw), 0);                    
                    draw_asciiname(17, sensorcard[sim.potmap()]);
                    draw_truth(18, cal_joyvert_brkmotor_mode, 0);
                    draw_truth(19, cal_pot_gasservo_mode, 0);
                }
                else if (datapage == PG_UI) {
                    draw_dynamic(9, loopfreq_hz);
                    draw_dynamic(10, (int32_t)loop_avg_us, loop_scale_min_us, loop_scale_avg_max_us);
                    draw_dynamic(11, loop_peak_us, loop_scale_min_us, loop_scale_peak_max_us);
                    draw_dynamic(12, touch_pt[0], 0, disp_width_pix);
                    draw_dynamic(13, touch_pt[1], 0, disp_height_pix);
                    draw_dynamic(14, touch_pt[2], 340, 3980);
                    draw_dynamic(15, touch_pt[3], 180, 3980);
                    draw_truth(16, flashdemo, 0);  // draw_dynamic(16, globalgamma, 0.1, 2.57, -1, 3);
                    draw_dynamic(17, neobright, 1.0, 100.0, -1, 3);
                    draw_dynamic(18, neodesat, 0, 10, -1, 2);  // -10, 10, -1, 2);
                    draw_truth(19, screensaver, 0);
                }
                draw_bool((_nowmode == CAL), 2);
                draw_bool((_nowmode == BASIC), 3);
                draw_bool(ignition, 4);
                draw_bool(syspower, 5);
                disp_data_dirty = false;
                _procrastinate = true;  // don't do anything else in this same loop
            }
            if (screensaver && !sim.enabled() && !_procrastinate) saver_update();
            _procrastinate = false;
        }
        void saver_touch(int16_t x, int16_t y) {
            if (x >= disp_simbuttons_x && y >= disp_simbuttons_y) {
                x -= disp_simbuttons_x; y -= disp_simbuttons_y;
                if (savtouch_last_x == -1) savtouch_last_x = x;
                if (savtouch_last_y == -1) savtouch_last_y = y;
                if (pentimer.expireset()) {
                    pensat += 1.5;
                    if (pensat > 255.0) pensat = 100.0;
                    pencolor = (savercycle == 1) ? random(0x10000) : hsv_to_rgb<uint16_t>(++penhue, (uint8_t)pensat, 200+random(56));
                }
                _saver.drawWedgeLine(savtouch_last_x, savtouch_last_y, x, y, 4, 4, pencolor, pencolor);  // savtouch_last_w, w, pencolor, pencolor);
                savtouch_last_x = x; savtouch_last_y = y;  // savtouch_last_w = w;
            }
        }
        void saver_setup() {
            // _saver.setColorDepth(8);  // Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
            _saver.createSprite(disp_saver_width, disp_saver_height);  // Create a sprite of defined size
            _saver.fillSprite(TFT_BLACK);
            sx0 = random(disp_saver_width);        // Random x coordinate
            sy0 = random(disp_saver_height);       // Random y coordinate
            for (int16_t axis=0; axis<=1; axis++) eraser_velo_sign[axis] = (random(1)) ? 1 : -1;
            _saver.setTextDatum(MC_DATUM);
            _saver.setTextColor(BLK); 
            _saver.setTextSize(1); 
            saverRefreshTimer.set(saver_refresh_us);
            saverCycleTimer.set((int64_t)saver_cycletime_us);
            pentimer.set(pentimeout);
        }
        void saver_update() {
            if (saverRefreshTimer.expireset()) {
                if (saverCycleTimer.expired()) {
                    savernumcycles++;
                    if (--savercycle < 1) savercycle = 3;
                    if (savercycle == 1) ++savershape %= savershapes;
                    saver_lotto = !random(saver_illicit_prob);
                    saverCycleTimer.set(saver_cycletime_us / ((savercycle == 2) ? 3 : 1));
                }
                long sx1 = random(disp_saver_width);        // Random x coordinate
                long sy1 = random(disp_saver_height);       // Random y coordinate
                if (savercycle != 2) {
                    spothue--;
                    if (!savershape) _saver.drawWedgeLine(sx0, sy0, sx1, sy1, 1+random(3), 1, hsv_to_rgb<uint16_t>(random(256), 63+(spothue>>1)+(spothue>>2), 150+random(106)), BLK);
                    else if (savershape == 1) {
                        uint8_t d1 = 10+random(30);
                        uint8_t d2 = 10+random(30);
                        uint8_t hue = random(255);
                        uint8_t sat = (spothue < 128) ? 2*spothue : 2*(255-spothue);
                        uint8_t brt = 180+random(76);
                        for (int i=0; i<(3 * 3+random(10)); i+=3) _saver.drawEllipse(sx1, sy1, d1 - i, d2 + i, hsv_to_rgb<uint16_t>(hue+2*i, sat, brt));
                    }
                    else if (savershape == 4) _saver.drawSmoothCircle(sx1, sy1, random(25), hsv_to_rgb<uint16_t>(spothue+127*random(1), random(128)+(spothue>>1), 150+random(106)), BLK);
                    else for (int star=0; star<10; star++) {
                        if (savershape == 2) _saver.drawSpot(random(disp_saver_width), random(disp_saver_height), 2+random(1), hsv_to_rgb<uint16_t>((spothue>>1)*(1+random(2)), 255, 210+random(46)), BLK);  // hue_to_rgb16(random(255)), BLK);
                        else if (savershape == 3) {
                            _saver.setTextColor(hsv_to_rgb<uint16_t>(sx0 + sy0 + (spothue>>2), 63+(spothue>>1), 200+random(56)), BLK);
                            char letter = (char)(1 + random(0xbe));
                            _saver.setCursor(sx1, sy1);
                            _saver.print((String)letter);
                        }
                    }
                    _saver.setTextColor(BLK);
                }
                if (savercycle == 3 && saver_lotto) _saver.drawString("do drugs", disp_saver_width / 2, disp_saver_height / 2, 4);
                else if (savercycle != 1) {
                    for (int axis=0; axis<=1; axis++) {
                        eraser_pos[axis] += eraser_velo[axis] * eraser_velo_sign[axis];
                        if (eraser_pos[axis] * eraser_velo_sign[axis] >= eraser_pos_max[axis]) {
                            eraser_pos[axis] = eraser_velo_sign[axis] * eraser_pos_max[axis];
                            eraser_velo[axis] = (eraser_velo_min + random(eraser_velo_max - eraser_velo_min)) >> (savershape == 3);
                            eraser_velo[!axis] = (eraser_velo_min + random(eraser_velo_max - eraser_velo_min)) >> (savershape == 3);
                            eraser_velo_sign[axis] *= -1;
                            eraser_rad = constrain(eraser_rad + random(5) - 2, eraser_rad_min, eraser_rad_max);
                        }
                    }
                    _saver.fillCircle((disp_saver_width / 2) + eraser_pos[0], (disp_saver_height / 2) + eraser_pos[1], eraser_rad, BLK);
                } 
                sx0 = sx1;
                sy0 = sy1;
                yield();
                _saver.pushSprite(disp_simbuttons_x, disp_simbuttons_y);
            }
        }
};
// The following project draws a nice looking gauge cluster, very apropos to our needs and code is given.
// See this video: https://www.youtube.com/watch?v=U4jOFLFNZBI&ab_channel=VolosProjects
// Rinkydink home page: http://www.rinkydinkelectronics.com
// moving transparent arrow sprite over background: https://www.youtube.com/watch?v=U4jOFLFNZBI&ab_channel=VolosProjects
// bar graphs: https://www.youtube.com/watch?v=g4jlj_T-nRw&ab_channel=VolosProjects
