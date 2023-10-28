#pragma once
#include <TFT_eSPI.h>
#include "globals.h"
#include <iostream>  // for gamma correction
#include <cmath>  // for gamma correction

#undef USE_DMA_TO_TFT
#ifdef USE_DMA_TO_TFT
    #define disp _tft[_sprsel]
#else
    #define disp _tft
#endif

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
#define STBL 0x9e1f  // steel blue is desaturated light blue
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
uint32_t hsv_to_rgb32(uint8_t hue, uint8_t sat = 255, uint8_t bright = 255, bool bright_flat = 1, uint8_t blu_boost = 0) {  // returns uint32 color in format 0x00RRGGBB
    uint32_t rgb[3] = { 255 - 3 * (uint32_t)((255 - hue) % 85), 0, 3 * (uint32_t)((255 - hue) % 85) };
    float maxc = (float)((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]);
    if (hue <= 85) { rgb[1] = rgb[0]; rgb[0] = rgb[2]; rgb[2] = 0; }
    else if (hue <= 170) { rgb[1] = rgb[2]; rgb[2] = rgb[0]; rgb[0] = 0; }
    float brightener = (float)bright / (bright_flat ? 255.0 : maxc);
    float blu_booster = 1 + (float)(blu_boost * rgb[2]) / (float)(255.0 * (rgb[0] + rgb[1] + rgb[2]));
    for (int led=0; led<=2; led++) 
        rgb[led] = brightener * ((float)rgb[led] + blu_booster * (255.0 - sat) * (float)(maxc - rgb[led]) / 255.0);
    return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
}
uint32_t hue_to_rgb16(uint8_t hue) {  // returns uint16 color
    if (hue <= 85) return (((3 * ((255 - hue) % 85)) & 0xf8) << 8) | (((255 - 3 * ((255 - hue) % 85)) & 0xfc) << 3);
    if (hue <= 170) return (((3 * ((255 - hue) % 85)) & 0xfc) << 3) | ((255 - 3 * ((255 - hue) % 85)) >> 3);
    return (((255 - 3 * ((255 - hue) % 85)) & 0xf8) << 8) | ((3 * ((255 - hue) % 85)) >> 3);
}

char modecard[8][7] = { "Basic", "Asleep", "Shutdn", "Stall", "Hold", "Fly", "Cruise", "Cal" };
int32_t colorcard[arraysize(modecard)] = { MGT, PUR, RED, ORG, YEL, GRN, TEAL, MBLU };

char sensorcard[14][7] = { "none", "joy", "bkpres", "brkpos", "speedo", "tach", "airflw", "mapsns", "engtmp", "batery", "startr", "basic", "ign", "syspwr" };

char idlemodecard[3][7] = { "direct", "cntrol", "minimz" };
char idlestatecard[ThrottleControl::targetstates::num_states][7] = { "todriv", "drving", "toidle", "tolow", "idling", "minimz" };

// These defines are just a convenience to keep the below datapage strings array initializations aligned in neat rows & cols for legibility
#define stEr "St\x88r"
#define brAk "Br\x83k"
#define spEd "Sp\x88""d"
#define b1nary "  \xa7 "
#define scroll "\x12   "
#define degreF "\xf7""F  "  // "\x09""F  "
#define ______ "    "
#define __________ "      \xf9 "
#define neo_bright "NeoBr\x8dgt"
#define maxadjrate "MaxAjR\x83t"
#define horfailsaf "HFails\x83""f"

char telemetry[disp_fixed_lines][9] = { "TriggerV", "   Speed", "    Tach", "ThrotPWM", brAk"Pres", brAk"Motr", "JoysticH", stEr"Motr", };  // Fixed rows
char units[disp_fixed_lines][5] = { "%   ", "mph ", "rpm ", "us  ", "psi ", "%   ", "%   ", "%   " };  // Fixed rows

enum dataset_pages { PG_RUN, PG_JOY, PG_CAR, PG_PWMS, PG_IDLE, PG_BPID, PG_GPID, PG_CPID, PG_TEMP, PG_SIM, PG_UI, num_datapages };
char pagecard[dataset_pages::num_datapages][5] = { "Run ", "Joy ", "Car ", "PWMs", "Idle", "Bpid", "Gpid", "Cpid", "Temp", "Sim ", "UI  " };
int32_t tuning_first_editable_line[dataset_pages::num_datapages] = { 9, 9, 5, 3, 4, 8, 7, 7, 10, 0, 7 };  // first value in each dataset page that's editable. All values after this must also be editable

char dataset_page_names[dataset_pages::num_datapages][disp_tuning_lines][9] = {
    { brAk"Posn", "MuleBatt", "LiPOBatt", "     Pot", "Air Velo", "     MAP", "MasAirFl", __________, __________, "Governor", stEr"Safe", },  // PG_RUN
    { "HRc Horz", "HRc Vert", "HotRcCh3", "HotRcCh4", "TrigVRaw", "JoyH Raw", __________, __________, __________, horfailsaf, "Deadband", },  // PG_JOY
    { "Pres ADC", __________, __________, __________, __________, "AirSpMax", " MAP Min", " MAP Max", spEd"Idle", spEd"RedL", "BkPos0Pt", },  // PG_CAR
    { "BrakePWM", "SteerPWM", __________, stEr"Left", stEr"Stop", stEr"Rigt", brAk"Extd", brAk"Stop", brAk"Retr", "ThrotCls", "ThrotOpn", },  // PG_PWMS
    { "IdlState", "Tach Tgt", "StallIdl", "Low Idle", "HighIdle", "ColdIdle", "Hot Idle", "ColdTemp", "Hot Temp", "SetlRate", "IdleMode", },  // PG_IDLE
    { "PresTarg", "Pres Err", "  P Term", "  I Term", "  D Term", "Integral", brAk"Motr", brAk"Pres", "Brake Kp", "Brake Ki", "Brake Kd", },  // PG_BPID
    { "TachTarg", "Tach Err", "  P Term", "  I Term", "  D Term", "Integral", __________, "OpenLoop", "  Gas Kp", "  Gas Ki", "  Gas Kd", },  // PG_GPID
    { spEd"Targ", "SpeedErr", "  P Term", "  I Term", "  D Term", "Integral", "ThrotSet", maxadjrate, "Cruis Kp", "Cruis Ki", "Cruis Kd", },  // PG_CPID
    { " Ambient", "  Engine", "AxleFrLt", "AxleFrRt", "AxleRrLt", "AxleRrRt", __________, __________, __________, __________, "No Temps", },  // PG_TEMP
    { "Joystick", brAk"Pres", brAk"Posn", "  Speedo", "    Tach", "AirSpeed", "     MAP", "Basic Sw", " Pot Map", "CalBrake", " Cal Gas", },  // PG_SIM
    { "LoopFreq", "Loop Avg", "LoopPeak", " Touch X", " Touch Y", " Touch X", " Touch Y", "BlnkDemo", neo_bright, "NeoDesat", "ScrSaver", },  // PG_UI      // "   Gamma"
};
char tuneunits[dataset_pages::num_datapages][disp_tuning_lines][5] = {
    { "in  ", "V   ", "V   ", "%   ", "mph ", "psi ", "g/s ", ______, ______, "%   ", "%   ", },  // PG_RUN
    { "us  ", "us  ", "us  ", "us  ", "%   ", "%   ", ______, ______, ______, "us  ", "us  ", },  // PG_JOY
    { "adc ", ______, ______, ______, ______, "mph ", "psi ", "psi ", "mph ", "mph ", "in  ", },  // PG_CAR
    { "us  ", "us  ", ______, "us  ", "us  ", "us  ", "us  ", "us  ", "us  ", "us  ", "us  ", },  // PG_PWMS
    { scroll, "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", degreF, degreF, "rpms", scroll, },  // PG_IDLE
    { "psi ", "psi ", "%   ", "%   ", "%   ", "%   ", "us  ", "adc ", ______, "Hz  ", "s   ", },  // PG_BPID
    { "rpm ", "rpm ", "us  ", "us  ", "us  ", "us  ", ______, b1nary, ______, "Hz  ", "s   ", },  // PG_GPID
    { "mph ", "mph ", "rpm ", "rpm ", "rpm ", "rpm ", "us  ", "usps", ______, "Hz  ", "s   ", },  // PG_CPID
    { degreF, degreF, degreF, degreF, degreF, degreF, ______, ______, ______, ______, b1nary, },  // PG_TEMP
    { b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, scroll, b1nary, b1nary, },  // PG_SIM
    { "Hz  ", "us  ", "us  ", "pix ", "pix ", "ohm ", "ohm ", "eyes", "%   ", "/10 ", "eyes", },  // PG_UI
};
char simgrid[4][3][5] = {
    { "psi\x18", "rpm\x18", "mph\x18" },
    { "psi\x19", "rpm\x19", "mph\x19" },
    { ______, " \x1e  ", ______ },
    { " \x11  ", " \x1f  ", "  \x10 " },  // Font special characters is the left-side map:  https://learn.adafruit.com/assets/103682
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
//  { 0x3e, 0x49, 0x5d, 0x49, 0x41, 0x41, 0x3e, 0x49, 0x5d, 0x49, 0x41, 0x41, 0x3e, 0x00, 0x00, 0x6e, 0x6f, },  // Googly eyes, to point out new features
//  { 0x7e, 0x20, 0x3e, 0x20, 0x00, 0x0c, 0x52, 0x4a, 0x3c, 0x00, 0x60, 0x18, 0x06, 0x00, 0x2c, 0x2a, 0x32, },  // ug/s - for manifold mass airflow
bool* idiotlights[15] = {&(err_sensor_alarm[LOST]), &(err_sensor_alarm[RANGE]), &(temp_err[ENGINE]), &(temp_err[WHEEL]), &panicstop, &hotrc_radio_lost, &shutdown_incomplete, &park_the_motors, &cruise_adjusting, &car_hasnt_moved, &starter, &boot_button, sim.enabled_ptr(), &running_on_devboard, &powering_up };
char idiotchars[arraysize(idiotlights)][3] = {"SL", "SR", "\xf7""E", "\xf7""W", "P\x13", "RC", "SI", "Pk", "Aj", "HM", "St", "BB", "Sm", "DB", "PU" };  // "c3", "c4" };
uint16_t idiotcolors[arraysize(idiotlights)];
uint8_t idiot_saturation = 225;  // 170-195 makes nice bright yet distinguishable colors
uint8_t idiot_hue_offset = 240;
bool idiots_dirty = true;
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
    { 0x08, 0x1c, 0x36, 0x00, 0x3e, 0x63, 0x63, 0x00, 0x36, 0x1c, 0x08, },     // 8 = "<C>"
    { 0x1d, 0x23, 0x47, 0x00, 0x3e, 0x63, 0x55, 0x49, 0x55, 0x63, 0x3e, },     // 9 = rotation arrow w/ X wheel
    { 0x3e, 0x41, 0x7f, 0x41, 0x41, 0x63, 0x3e, 0x08, 0x7f, 0x55, 0x7f, },     // 10 = motor w/ spur gear
    { 0x01, 0x7f, 0x7f, 0x7f, 0x3f, 0x38, 0x74, 0x70, 0x70, 0x70, 0x60, },     // 11 = boot
    { 0x6e, 0x6b, 0x3b, 0x00, 0x7f, 0x00, 0x7f, 0x06, 0x1c, 0x06, 0x7f, },     // 12 = "SIM"
    { 0x7f, 0x63, 0x3e, 0x00, 0x7f, 0x6b, 0x6b, 0x00, 0x7f, 0x30, 0x1f, },     // 13 = "DEV"
    { 0x00, 0x00, 0x3e, 0x41, 0x40, 0x4f, 0x40, 0x41, 0x3e, 0x00, 0x00, }, };  // 14 = need bitmap for powering up
//  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, }, };  // 14 = N/A (no idiot light bitmap)
void set_idiotcolors() {
    for (int32_t idiot=0; idiot<arraysize(idiotlights); idiot++) {
        int division = disp_idiots_per_row;
        uint32_t color32 = hsv_to_rgb32((int8_t)(255 * (idiot % division) / division + idiot_hue_offset), idiot_saturation, 255, 0, 220);
        idiotcolors[idiot] = color_uint32_to_16b(color32);  // 5957 = 2^16/11
        if (gamma_correct_enabled) idiotcolors[idiot] = gamma16(idiotcolors[idiot]);
        idiots_dirty = true;
    }
}

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
Timer tuningCtrlTimer (25000000);  // This times out edit mode after a a long period of inactivity

// run state globals
int32_t shutdown_color = colorcard[SHUTDOWN];

class Display {
    private:
        #ifdef USE_DMA_TO_TFT
            TFT_eSPI _panel = TFT_eSPI();
            TFT_eSprite _tft[2] = {TFT_eSprite(&_panel), TFT_eSprite(&_panel) };
            bool _sprsel = 0;
            uint16_t* _sprptr[2];
            // TFT_eSprite _tft = TFT_eSprite(&_panel); // LCD screen
        #else
            TFT_eSPI _tft = TFT_eSPI();
            TFT_eSprite _saver = TFT_eSprite(&_tft);  // Declare screensaver sprite object with pointer to tft object
        #endif
        Timer _tftResetTimer;
        Timer _tftDelayTimer;
        int32_t _timing_tft_reset;
        bool _procrastinate = false, reset_finished = false;
        bool _disp_redraw_all = true;
        
        // For screensaver sprite
        long star_x0, star_y0;
        long touchpoint_x = -1; long touchpoint_y = -1;
        long eraser_rad = 14;
        long eraser_velo_min = 4;
        long eraser_velo_max = 10;
        long eraser_pos[2] = { 0, 0 };
        long eraser_velo[2] = { random(eraser_velo_max), random(eraser_velo_max) };
        long eraser_pos_max[2] = { disp_saver_width / 2 - eraser_rad, disp_saver_height / 2 - eraser_rad }; 
        long eraser_velo_sign[2] = { 1, 1 };
        uint8_t penhue = 0;
        uint16_t pencolor = RED;
        uint32_t pentimeout = 700000;
        Timer pentimer;
        int savernumcycles; int savercycle = 1; int savershape_last;
        int savershapes = 3;  // 4
        int savershape = random(savershapes);  // 3
        uint32_t saver_cycletime_us = 45000000;
        Timer saverRefreshTimer, saverCycleTimer;
        int16_t saver_lines_mode = 0;  // 0 = eraser, 1 = do drugs
        uint32_t disp_oldmode = SHUTDOWN;   // So we can tell when the mode has just changed. start as different to trigger_mode start algo
    public:

        #ifdef USE_DMA_TO_TFT
            Display (int8_t cs_pin, int8_t dc_pin) : _panel(cs_pin, dc_pin), _tftResetTimer(100000), _tftDelayTimer(3000000), _timing_tft_reset(0){}
            Display () : _panel(), _tftResetTimer(100000), _tftDelayTimer(3000000), _timing_tft_reset(0){}
        #else
            Display (int8_t cs_pin, int8_t dc_pin) : _tft(cs_pin, dc_pin), _tftResetTimer(100000), _tftDelayTimer(3000000), _timing_tft_reset(0){}
            Display () : _tft(), _tftResetTimer(100000), _tftDelayTimer(3000000), _timing_tft_reset(0){}
        #endif

        void init() {
            yield();
            #ifdef USE_DMA_TO_TFT
                _panel.begin();
                _panel.setRotation((flip_the_screen) ? 3 : 1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
                _tft[0].setColorDepth(16);
                _tft[1].setColorDepth(16);
                _sprptr[0] = (uint16_t*)_tft[0].createSprite(disp_width_pix, disp_height_pix);
                _sprptr[1] = (uint16_t*)_tft[1].createSprite(disp_width_pix, disp_height_pix);
                _panel.initDMA(); // Initialise the DMA engine (tested with STM32F446 and STM32F767)
            #else
                _tft.begin();
                _tft.setRotation((flip_the_screen) ? 3 : 1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
            #endif
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
            disp.fillScreen (BLK);  // Black out the whole screen
            draw_touchgrid (false);
            draw_fixed (dataset_page, dataset_page_last, false);
            yield();
            set_idiotcolors();
            draw_idiotlights(disp_idiot_corner_x, disp_idiot_corner_y, true);
            _disp_redraw_all = true;
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
                uint32_t color32 = hsv_to_rgb32((int8_t)(255 * (rm % division) / division + hue_offset), saturat, 255, 0, 220);
                colorcard[rm] = color_uint32_to_16b(color32);  // 5957 = 2^16/11
                if (gamma_correct_enabled) colorcard[rm] = gamma16(colorcard[rm]);
                disp_runmode_dirty = true;
            }
        }
        void draw_bargraph_base (int32_t corner_x, int32_t corner_y, int32_t width) {  // draws a horizontal bargraph scale.  124, y, 40
            disp.drawFastHLine (corner_x+disp_bargraph_squeeze, corner_y, width-disp_bargraph_squeeze*2, GRY1);
            for (int32_t offset=0; offset<=2; offset++) disp.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(width/2 - disp_bargraph_squeeze), corner_y-1, 3, WHT);
        }
        void draw_needle_shape (int32_t pos_x, int32_t pos_y, int32_t color) {  // draws a cute little pointy needle
            disp.drawFastVLine (pos_x-1, pos_y, 2, color);
            disp.drawFastVLine (pos_x, pos_y, 4, color);
            disp.drawFastVLine (pos_x+1, pos_y, 2, color);
        }
        void draw_target_shape (int32_t pos_x, int32_t pos_y, int32_t t_color, int32_t r_color) {  // draws a cute little target symbol
            disp.drawFastVLine (pos_x-1, pos_y+7, 2, t_color);
            disp.drawFastVLine (pos_x, pos_y+5, 4, t_color);
            disp.drawFastVLine (pos_x+1, pos_y+7, 2, t_color);
        }
        void draw_bargraph_needle (int32_t n_pos_x, int32_t old_n_pos_x, int32_t pos_y, int32_t n_color) {  // draws a cute little pointy needle
            draw_needle_shape (old_n_pos_x, pos_y, BLK);
            draw_needle_shape (n_pos_x, pos_y, n_color);
        }
        void draw_string (int32_t x_new, int32_t x_old, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor, bool forced=false) {  // Send in "" for oldtext if erase isn't needed
            int32_t oldlen = strlen(oldtext);
            int32_t newlen = strlen(text);
            disp.setTextColor (bgcolor);  
            for (int32_t letter=0; letter < oldlen; letter++) {
                if (newlen - letter < 1) {
                    disp.setCursor (x_old+disp_font_width*letter, y);
                    disp.print (oldtext[letter]);
                }
                else if (oldtext[letter] != text[letter]) {
                    disp.setCursor (x_old+disp_font_width*letter, y);
                    disp.print (oldtext[letter]);
                }
            }
            disp.setTextColor (color);  
            for (int32_t letter=0; letter < newlen; letter++) {
                if (oldlen - letter < 1) {
                    disp.setCursor (x_new+disp_font_width*letter, y);
                    disp.print (text[letter]);
                }
                else if (oldtext[letter] != text[letter] || forced) {
                    disp.setCursor (x_new+disp_font_width*letter, y);
                    disp.print (text[letter]);
                }
            }
        }
        void draw_unitmap (int8_t index, int32_t x, int32_t y, uint16_t color) {
            for (int32_t xo = 0; xo < disp_font_width * 3 - 1; xo++)
                for (int32_t yo = 0; yo < disp_font_height - 1; yo++)
                    if ((unitmaps[index][xo] >> yo) & 1) disp.drawPixel (x + xo + 1, y + yo + 1, color);
        }
        void draw_string_units (int32_t x, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
            bool drawn = false;
            for (int8_t i = 0; i<arraysize(unitmaps); i++)
                if (!strcmp(unitmapnames[i], oldtext)) {
                    draw_unitmap(i, x, y, bgcolor);
                    drawn = true;
                }
            if (!drawn) {
                disp.setCursor (x, y);
                disp.setTextColor (bgcolor);
                disp.print (oldtext);  // Erase the old content
            }
            for (int8_t i = 0; i<arraysize(unitmaps); i++)
                if (!strcmp(unitmapnames[i], text)) {
                    draw_unitmap(i, x, y, color);
                    return;
                }
            disp.setCursor (x, y);
            disp.setTextColor (color);
            disp.print (text);  // Erase the old content
        }
        // draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
        void draw_fixed (int32_t page, int32_t page_last, bool redraw_tuning_corner, bool forced=false) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
            disp.setTextColor (GRY2);
            disp.setTextSize (1);
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
                draw_string (disp_datapage_names_x, disp_datapage_names_x, (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix, dataset_page_names[page][lineno], dataset_page_names[page_last][lineno], GRY2, BLK, forced);
                draw_string_units (disp_datapage_units_x, (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix, tuneunits[page][lineno], tuneunits[page_last][lineno], GRY2, BLK);
                if (redraw_tuning_corner) {
                    int32_t corner_y = (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix + 7;  // lineno*disp_line_height_pix+disp_vshift_pix-1;
                    draw_bargraph_base (disp_bargraphs_x, corner_y, disp_bargraph_width);
                    if (disp_needles[lineno] >= 0) draw_bargraph_needle (-1, disp_needles[lineno], corner_y - 6, BLK);  // Let's draw a needle
                }
            }
        }
        void draw_hyphen (int32_t x_pos, int32_t y_pos, int32_t color) {  // Draw minus sign in front of negative numbers
            disp.drawFastHLine (x_pos+2, y_pos+3, 3, color);
        }
        void draw_dynamic (int32_t lineno, char const* disp_string, int32_t value, int32_t lowlim, int32_t hilim, int32_t target=-1, int32_t color=-1) {
            int32_t age_us = (color >= 0) ? 11 : (int32_t)((float)(dispAgeTimer[lineno].elapsed()) / 2500000); // Divide by us per color gradient quantum
            int32_t x_base = disp_datapage_values_x;
            bool polarity = (value >= 0);  // polarity 0=negative, 1=positive
            if (strcmp(disp_values[lineno], disp_string) || value == 1234567 || _disp_redraw_all) {  // If value differs, Erase old value and write new
                if (color == -1) color = GRN;
                int32_t y_pos = lineno*disp_line_height_pix+disp_vshift_pix;
                if (polarity != disp_polarities[lineno]) draw_hyphen (x_base, y_pos, (!polarity) ? color : BLK);
                draw_string (x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_string, disp_values[lineno], color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
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
                    if (t_pos != disp_targets[lineno] || (t_pos == n_pos)^(disp_needles[lineno] != disp_targets[lineno]) || _disp_redraw_all) {
                        draw_target_shape (disp_targets[lineno], corner_y, BLK, -1);  // Erase old target
                        disp.drawFastHLine (disp_targets[lineno]-(disp_targets[lineno] != corner_x+disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+7, 2+(disp_targets[lineno] != corner_x+disp_bargraph_width-disp_bargraph_squeeze), GRY1);  // Patch bargraph line where old target got erased
                        for (int32_t offset=0; offset<=2; offset++) disp.drawFastVLine ((corner_x+disp_bargraph_squeeze)+offset*(disp_bargraph_width/2 - disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+6, 3, WHT);  // Redraw bargraph graduations in case one got corrupted by target erasure
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
                int32_t length = min (sigdig+1, maxlength);
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
        void draw_runmode (int32_t runmode, int32_t oldmode, int32_t color_override=-1) {  // color_override = -1 uses default color
            int32_t color = (color_override == -1) ? colorcard[runmode] : color_override;
            int32_t x_new = disp_runmode_text_x + disp_font_width * (2 + strlen (modecard[runmode])) - 3;
            int32_t x_old = disp_runmode_text_x + disp_font_width * (2 + strlen (modecard[oldmode])) - 3;
            draw_string (disp_runmode_text_x + disp_font_width, disp_runmode_text_x + disp_font_width, disp_vshift_pix, modecard[oldmode], "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (x_old, x_old, disp_vshift_pix, "Mode", "", BLK, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (disp_runmode_text_x + disp_font_width, disp_runmode_text_x + disp_font_width, disp_vshift_pix, modecard[runmode], "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            draw_string (x_new, x_new, disp_vshift_pix, "Mode", "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        }
        void draw_dataset_page (int32_t page, int32_t page_last, bool forced=false) {
            draw_fixed (page, page_last, true, forced);  // Erase and redraw dynamic data corner of screen with names, units etc.
            draw_string (disp_datapage_title_x, disp_datapage_title_x, disp_vshift_pix, pagecard[page], pagecard[page_last], STBL, BLK, forced); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        }
        void draw_selected_name (int32_t tun_ctrl, int32_t tun_ctrl_last, int32_t selected_val, int32_t selected_last) {
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
            disp.fillRect(disp_simbuttons_x, disp_simbuttons_y, disp_saver_width, disp_saver_height, BLK);
            disp.setTextColor (LYEL);
            for (int32_t row = 0; row < arraysize(simgrid); row++) {
                for (int32_t col = 0; col < arraysize(simgrid[row]); col++) {
                    int32_t cntr_x = touch_margin_h_pix + touch_cell_h_pix*(col+3) + (touch_cell_h_pix>>1) +2;
                    int32_t cntr_y = touch_cell_v_pix*(row+1) + (touch_cell_v_pix>>1);
                    if (strcmp (simgrid[row][col], ______ )) {
                        disp.fillCircle (cntr_x, cntr_y, disp_simbutton_radius_pix, create ? DGRY : BLK);
                        disp.drawCircle (cntr_x, cntr_y, 19, create ? LYEL : BLK);
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
            disp.setTextColor (WHT);
            for (int32_t row = 0; row < arraysize (side_menu_buttons); row++) {  // Step thru all rows to draw buttons along the left edge
                disp.fillRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, DGRY);
                disp.drawRoundRect (-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, LYEL);
                namelen = 0;
                for (uint32_t x = 0 ; x < arraysize (side_menu_buttons[row]) ; x++ ) {
                    if (side_menu_buttons[row][x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
                }
                for (int32_t letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
                    disp.setCursor (1, ( touch_cell_v_pix*row) + (touch_cell_v_pix/2) - (int32_t)(4.5*((float)namelen-1)) + (disp_font_height+1)*letter); // adjusts vertical offset depending how many letters in the button name and which letter we're on
                    disp.println (side_menu_buttons[row][letter]);  // Writes each letter such that the whole name is centered vertically on the button
                }
            }
            if (!side_only) {
                for (int32_t col = 2; col <= 5; col++) {  // Step thru all cols to draw buttons across the top edge
                    disp.fillRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
                    disp.drawRoundRect (touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, LYEL);  // disp.width()-9, 3, 18, (disp.height()/5)-6, 8, LYEL);
                }
            }
        }
        void draw_reticle(uint32_t x, uint32_t y) {
            disp.drawFastHLine (x - 2, y, 5, DGRY);
            disp.drawFastVLine (x, y - 2, 5, DGRY);
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
            disp.drawRoundRect (x, y, 2 * disp_font_width + 1, disp_font_height + 1, 1, bg);
            for (int32_t xo = 0; xo < (2 * disp_font_width - 1); xo++)
                for (int32_t yo = 0; yo < disp_font_height - 1; yo++)
                    disp.drawPixel (x + xo + 1, y + yo + 1, ((idiotmaps[idiot][xo] >> yo) & 1) ? color : bg);
        }
        void draw_idiotlight (int32_t idiot, int32_t x, int32_t y) {
            if (idiotmaps[idiot][0] >= 0x80) {
                disp.fillRoundRect (x, y, 2 * disp_font_width + 1, disp_font_height + 1, 2, (*(idiotlights[idiot])) ? idiotcolors[idiot] : BLK);  // GRY1);
                disp.setTextColor ((*(idiotlights[idiot])) ? BLK : darken_color(idiotcolors[idiot]));  // darken_color ((*(idiotlights[index])) ? BLK : DGRY)
                disp.setCursor (x+1, y+1);
                disp.print (idiotchars[idiot]);
            }
            else draw_idiotbitmap(idiot, x, y);
            idiotlasts[idiot] = *(idiotlights[idiot]);
        }
        void draw_idiotlights (int32_t x, int32_t y, bool force = false) {
            for (int32_t ilite=0; ilite < arraysize(idiotlights); ilite++)
                if (force || (*(idiotlights[ilite]) ^ idiotlasts[ilite]))
                    draw_idiotlight (ilite, x + (2 * disp_font_width + 2) * ((ilite % disp_idiots_per_row) % disp_idiots_per_row), y + disp_idiot_row_height * (int32_t)(ilite / disp_idiots_per_row));
        }
        void draw_temperature(location location, int draw_index) {
            if (!tempsens.detected(location)) draw_eraseval(draw_index);
            else draw_dynamic(draw_index, tempsens.val(location), temp_lims_f[tempsens.errclass(location)][DISP_MIN], temp_lims_f[tempsens.errclass(location)][DISP_MAX]);
        }
        void update_idiots(bool force = false) {
            draw_idiotlights(disp_idiot_corner_x, disp_idiot_corner_y, force);
        }
        void update() {
            if (!display_enabled) return;
            update_idiots(_disp_redraw_all || idiots_dirty);
            idiots_dirty = false;
            if (sim.enabled()) {
                if (!simulating_last || _disp_redraw_all) {
                    draw_simbuttons(sim.enabled());  // if we just entered simulator draw the simulator buttons, or if we just left erase them
                    _procrastinate = true;  // Waits till next loop to draw changed values
                }
            }
            else if (simulating_last) draw_simbuttons(sim.enabled());
            simulating_last = sim.enabled();
            if ((disp_dataset_page_dirty || _disp_redraw_all)) {
                static bool first = true;
                draw_dataset_page(dataset_page, dataset_page_last, first);
                first = false;
                disp_dataset_page_dirty = false;
                if (dataset_page_last != dataset_page) config.putUInt("dpage", dataset_page);
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
            }
            if (disp_runmode_dirty || _disp_redraw_all) {
                draw_runmode(runmode, disp_oldmode, (runmode == SHUTDOWN) ? shutdown_color : -1);
                disp_oldmode = runmode;
                disp_runmode_dirty = false;
            }
            if ((dispRefreshTimer.expired() && !_procrastinate) || _disp_redraw_all) {
                dispRefreshTimer.reset();
                float drange;
                draw_dynamic(1, hotrc_pc[VERT][FILT], hotrc_pc[VERT][MIN], hotrc_pc[VERT][MAX]);
                draw_dynamic(2, speedo.filt(), 0.0, speedo.redline_mph(), speedo_target_mph);
                draw_dynamic(3, tach.filt(), 0.0, tach.redline_rpm(), tach_target_rpm);
                draw_dynamic(4, gas_out_us, gas_cw_open_us, gas_ccw_closed_us);
                draw_dynamic(5, pressure.filt(), pressure.min_human(), pressure.max_human(), pressure_target_psi);  // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.targ() : pressure_target_adc);
                draw_dynamic(6, brake_out_pc, brake_extend_pc, brake_retract_pc);
                draw_dynamic(7, hotrc_pc[HORZ][FILT], hotrc_pc[HORZ][MIN], hotrc_pc[HORZ][MAX]);
                draw_dynamic(8, steer_out_pc, steer_left_pc, steer_right_pc);
                if (dataset_page == PG_RUN) {
                    draw_dynamic(9, brakepos.filt(), brakepos.op_min_in(), brakepos.op_max_in());
                    draw_dynamic(10, mulebatt.filt(), mulebatt.op_min_v(), mulebatt.op_max_v());
                    draw_dynamic(11, lipobatt.filt(), lipobatt.op_min_v(), lipobatt.op_max_v());
                    draw_dynamic(12, pot.val(), pot.min(), pot.max());
                    draw_dynamic(13, airvelo.filt(), airvelo.min_mph(), airvelo.max_mph());
                    draw_dynamic(14, mapsens.filt(), mapsens.min_psi(), mapsens.max_psi());
                    draw_dynamic(15, maf_ugps, maf_min_ugps, maf_max_ugps);
                    draw_eraseval(16);
                    draw_eraseval(17);
                    draw_dynamic(18, gas_governor_pc, 0.0, 100.0);
                    draw_dynamic(19, steer_safe_pc, 0.0, 100.0);
                }
                else if (dataset_page == PG_JOY) {
                    draw_dynamic(9, hotrc_us[HORZ][RAW], hotrc_us[HORZ][MIN], hotrc_us[HORZ][MAX]);
                    draw_dynamic(10, hotrc_us[VERT][RAW], hotrc_us[VERT][MIN], hotrc_us[VERT][MAX]);
                    draw_dynamic(11, hotrc_us[CH3][RAW], hotrc_us[CH3][MIN], hotrc_us[CH3][MAX]);
                    draw_dynamic(12, hotrc_us[CH4][RAW], hotrc_us[CH4][MIN], hotrc_us[CH4][MAX]);
                    draw_dynamic(13, hotrc_pc[HORZ][RAW], hotrc_pc[HORZ][MIN], hotrc_pc[HORZ][MAX]);
                    draw_dynamic(14, hotrc_pc[VERT][RAW], hotrc_pc[VERT][MIN], hotrc_pc[VERT][MAX]);
                    draw_eraseval(15);
                    draw_eraseval(16);
                    draw_eraseval(17);
                    draw_dynamic(18, hotrc_failsafe_us, hotrc_absmin_us, hotrc_us[VERT][MIN] - hotrc_us[VERT][MARGIN]);
                    draw_dynamic(19, hotrc_deadband_us, 0, 100);
                }
                else if (dataset_page == PG_CAR) {
                    draw_dynamic(9, pressure.native(), pressure.min_native(), pressure.max_native());                    
                    for (int line=10; line<=13; line++) draw_eraseval(line);
                    draw_dynamic(14, airvelo.max_mph(), 0.0, airvelo.abs_max_mph());
                    draw_dynamic(15, mapsens.min_psi(), mapsens.abs_min_psi(), mapsens.abs_max_psi());
                    draw_dynamic(16, mapsens.max_psi(), mapsens.abs_min_psi(), mapsens.abs_max_psi());
                    draw_dynamic(17, speedo_idle_mph, 0.0, speedo.redline_mph());
                    draw_dynamic(18, speedo.redline_mph(), 0.0, speedo.max_human());
                    draw_dynamic(19, brakepos.zeropoint(), brakepos.min_human(), brakepos.max_human());  // BrakePositionSensor::abs_min_retract_in, BrakePositionSensor::abs_max_extend_in);
                }
                else if (dataset_page == PG_PWMS) {
                    draw_dynamic(9, brake_out_us, brake_retract_us, brake_extend_us);
                    draw_dynamic(10, steer_out_us, steer_left_us, steer_right_us);
                    draw_eraseval(11);
                    draw_dynamic(12, steer_left_us, steer_left_min_us, steer_stop_us);
                    draw_dynamic(13, steer_stop_us, steer_left_us, steer_right_us);
                    draw_dynamic(14, steer_right_us, steer_stop_us, steer_right_max_us);
                    draw_dynamic(15, brake_extend_us, brake_extend_min_us, brake_stop_us);
                    draw_dynamic(16, brake_stop_us, brake_extend_us, brake_retract_us);
                    draw_dynamic(17, brake_retract_us, brake_stop_us, brake_retract_max_us);
                    draw_dynamic(18, gas_ccw_closed_us, gas_cw_min_us, gas_ccw_max_us);
                    draw_dynamic(19, gas_cw_open_us, gas_cw_min_us, gas_ccw_max_us);
                }
                else if (dataset_page == PG_IDLE) {
                    draw_asciiname(9, idlestatecard[throttle.targetstate()]);
                    draw_dynamic(10, tach_target_rpm, 0.0, tach.redline_rpm());
                    draw_dynamic(11, throttle.stallpoint(), tach_idle_abs_min_rpm, tach_idle_abs_max_rpm);
                    draw_dynamic(12, throttle.idlespeed(), tach_idle_abs_min_rpm, tach_idle_abs_max_rpm);  // throttle.idlehot(), throttle.idlecold());
                    draw_dynamic(13, throttle.idlehigh(), tach_idle_abs_min_rpm, tach_idle_abs_max_rpm);
                    draw_dynamic(14, throttle.idlecold(), tach_idle_abs_min_rpm, tach_idle_abs_max_rpm, -1, 4);
                    draw_dynamic(15, throttle.idlehot(), tach_idle_abs_min_rpm, tach_idle_abs_max_rpm, -1, 4);
                    draw_dynamic(16, throttle.tempcold(), temp_lims_f[ENGINE][DISP_MIN], temp_lims_f[ENGINE][DISP_MAX]);
                    draw_dynamic(17, throttle.temphot(), temp_lims_f[ENGINE][DISP_MIN], temp_lims_f[ENGINE][DISP_MAX]);
                    draw_dynamic(18, (int32_t)throttle.settlerate(), 0, 500);
                    draw_asciiname(19, idlemodecard[(int32_t)throttle.idlemode()]);
                }
                else if (dataset_page == PG_BPID) {
                    drange = brake_extend_us-brake_retract_us;
                    draw_dynamic(9, pressure_target_psi, pressure.min_human(), pressure.max_human());
                    draw_dynamic(10, brake_pid.err(), pressure.min_human()-pressure.max_human(), pressure.max_human()-pressure.min_human());
                    draw_dynamic(11, brake_pid.pterm(), -drange, drange);
                    draw_dynamic(12, brake_pid.iterm(), -drange, drange);
                    draw_dynamic(13, brake_pid.dterm(), -drange, drange);
                    draw_dynamic(14, brake_pid.outsum(), -brake_pid.outrange(), brake_pid.outrange());  // brake_spid_speedo_delta_adc, -range, range);
                    draw_dynamic(15, brake_out_us, brake_extend_us, brake_retract_us);
                    draw_dynamic(16, pressure.native(), pressure.min_native(), pressure.max_native());
                    draw_dynamic(17, brake_pid.kp(), 0.0, 2.0);
                    draw_dynamic(18, brake_pid.ki(), 0.0, 2.0);
                    draw_dynamic(19, brake_pid.kd(), 0.0, 2.0);
                }
                else if (dataset_page == PG_GPID) {
                    drange = gas_ccw_closed_us-gas_govern_us;
                    draw_dynamic(9, tach_target_rpm, 0.0, tach.redline_rpm());
                    draw_dynamic(10, gas_pid.err(), throttle.idlespeed() - tach_govern_rpm, tach_govern_rpm - throttle.idlespeed());
                    draw_dynamic(11, gas_pid.pterm(), -drange, drange);
                    draw_dynamic(12, gas_pid.iterm(), -drange, drange);
                    draw_dynamic(13, gas_pid.dterm(), -drange, drange);
                    draw_dynamic(14, gas_pid.outsum(), -gas_pid.outrange(), gas_pid.outrange());
                    draw_eraseval(15);
                    draw_truth(16, gas_open_loop, 1);
                    draw_dynamic(17, gas_pid.kp(), 0.0, 2.0);
                    draw_dynamic(18, gas_pid.ki(), 0.0, 2.0);
                    draw_dynamic(19, gas_pid.kd(), 0.0, 2.0);
                }
                else if (dataset_page == PG_CPID) {
                    drange = tach_govern_rpm - throttle.idlespeed();
                    draw_dynamic(9, speedo_target_mph, 0.0, speedo_govern_mph);
                    draw_dynamic(10, cruise_pid.err(), speedo_idle_mph-speedo_govern_mph, speedo_govern_mph-speedo_idle_mph);
                    draw_dynamic(11, cruise_pid.pterm(), -drange, drange);
                    draw_dynamic(12, cruise_pid.iterm(), -drange, drange);
                    draw_dynamic(13, cruise_pid.dterm(), -drange, drange);
                    draw_dynamic(14, cruise_pid.outsum(), -cruise_pid.outrange(), cruise_pid.outrange());  // cruise_spid_speedo_delta_adc, -drange, drange);
                    draw_dynamic(15, gas_cruise_us, gas_cw_open_us, gas_ccw_closed_us);
                    draw_dynamic(16, cruise_delta_max_us_per_s, 1, 1000);
                    draw_dynamic(17, cruise_pid.kp(), 0.0, 10.0);
                    draw_dynamic(18, cruise_pid.ki(), 0.0, 10.0);
                    draw_dynamic(19, cruise_pid.kd(), 0.0, 10.0);
                }
                else if (dataset_page == PG_TEMP) {
                    draw_temperature(location::ambient, 9);
                    draw_temperature(location::engine, 10);
                    draw_temperature(location::wheel_fl, 11);
                    draw_temperature(location::wheel_fr, 12);
                    draw_temperature(location::wheel_rl, 13);
                    draw_temperature(location::wheel_rr, 14);
                    for (int line=15; line<=18; line++) draw_eraseval(line);
                    draw_truth(19, dont_take_temperatures, 2);
                }
                else if (dataset_page == PG_SIM) {
                    draw_truth(9, sim.can_sim(sensor::joy), 0);
                    draw_truth(10, sim.can_sim(sensor::pressure), 0);
                    draw_truth(11, sim.can_sim(sensor::brkpos), 0);
                    draw_truth(12, sim.can_sim(sensor::speedo), 0);
                    draw_truth(13, sim.can_sim(sensor::tach), 0);
                    draw_truth(14, sim.can_sim(sensor::airvelo), 0);
                    draw_truth(15, sim.can_sim(sensor::mapsens), 0);
                    draw_truth(16, sim.can_sim(sensor::basicsw), 0);                    
                    draw_asciiname(17, sensorcard[sim.potmap()]);
                    draw_truth(18, cal_joyvert_brkmotor_mode, 0);
                    draw_truth(19, cal_pot_gasservo_mode, 0);
                }
                else if (dataset_page == PG_UI) {
                    draw_dynamic(9, loopfreq_hz);
                    draw_dynamic(10, (int32_t)looptime_avg_us, 0, loop_maxloop_us);
                    draw_dynamic(11, looptime_peak_us, 0, loop_maxloop_us);
                    draw_dynamic(12, touch_pt[0], 0, disp_width_pix);
                    draw_dynamic(13, touch_pt[1], 0, disp_height_pix);
                    draw_dynamic(14, touch_pt[2], 340, 3980);
                    draw_dynamic(15, touch_pt[3], 180, 3980);

                    draw_truth(16, flashdemo, 0);
                    // draw_dynamic(16, globalgamma, 0.1, 2.57, -1, 3);
                    draw_dynamic(17, neobright, 1.0, 100.0, -1, 3);
                    draw_dynamic(18, neodesat, 0, 10, -1, 2);  // -10, 10, -1, 2);
                    draw_truth(19, screensaver, 0);
                }
                draw_bool((runmode == CAL), 2);
                draw_bool((runmode == BASIC), 3);
                draw_bool(ignition, 4);
                draw_bool(syspower, 5);
                _procrastinate = true;
            }
            if (screensaver && !sim.enabled() && !_procrastinate) {
                saver_update();
                _procrastinate = true;
            }
            #ifdef USE_DMA_TO_TFT
                // if (_panel.dmaBusy()) prime_max++; // Increase processing load until just not busy
                _panel.pushImageDMA(0, 0, disp_width_pix, disp_height_pix, _sprptr[_sprsel]);
                _sprsel = !_sprsel;
                // _panel.endWrite();
            // #else
            //     _tft.pushSprite(0, 0);
            #endif
            _procrastinate = false;
            _disp_redraw_all = false;

        }
        void saver_touch(int16_t x, int16_t y) {
            touchpoint_x = x;
            touchpoint_y = y;
            if (touchpoint_x >= disp_simbuttons_x && touchpoint_y >= disp_simbuttons_y) {
                if (pentimer.expireset()) {
                    if (savercycle == 1) pencolor = random(0x10000);
                    else pencolor = hue_to_rgb16(++penhue);
                }
                _saver.fillCircle(touchpoint_x-disp_simbuttons_x, touchpoint_y-disp_simbuttons_y, 4, pencolor);
                touchpoint_x = -1;
                touchpoint_y = -1;
            }
        }
        void saver_setup() {
            // _saver.setColorDepth(8);  // Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
            _saver.createSprite(disp_saver_width, disp_saver_height);  // Create a sprite of defined size
            _saver.fillSprite(TFT_BLACK);
            star_x0 = random(disp_saver_width);        // Random x coordinate
            star_y0 = random(disp_saver_height);       // Random y coordinate
            if (saver_lines_mode == 1) savershape = 1;
            for (int16_t axis=0; axis<=1; axis++) { eraser_velo_sign[axis] = (random(1)) ? 1 : -1; }
            _saver.setTextDatum(MC_DATUM);
            _saver.setTextColor(BLK);
            saverRefreshTimer.set(50000);
            saverCycleTimer.set((int64_t)saver_cycletime_us);
            pentimer.set(pentimeout);
        }
        void saver_update() {
            if (saverRefreshTimer.expireset()) {
                if (saverCycleTimer.expireset()) {
                    savernumcycles++;
                    if (saver_lines_mode == 1) {
                        if (!savercycle) _saver.fillSprite(BLK);
                        savercycle = !savercycle;
                    }
                    else if (saver_lines_mode == 0) {
                        if (--savercycle < 0b01) savercycle = 0b11;
                        if (!(savernumcycles % savershapes)) {
                            savershape_last = savershape;
                            while (savershape == savershape_last) savershape = random(savershapes);
                        }
                    }
                }
                uint16_t color = hue_to_rgb16(random(255)); // Returns colour 0 - 0xFFFF
                if (gamma_correct_enabled) color = gamma16(color);
                long star_x1 = random(disp_saver_width);        // Random x coordinate
                long star_y1 = random(disp_saver_height);       // Random y coordinate
                if (saver_lines_mode || (savercycle != 2)) {
                    if (!savershape) {
                        _saver.drawLine(star_x0, star_y0, star_x1, star_y1, color); 
                        if (savercycle) _saver.drawLine(star_x0+1, star_y0+1, star_x1+1, star_y1+1, color);
                    }
                    else if (savershape == 1) {
                        int rad = random(24);
                        _saver.drawCircle(star_x1, star_y1, rad, color);
                        _saver.drawCircle(star_x1, star_y1, rad+1, color);
                    }
                    else if (savershape == 2)      // Draw pixels in sprite
                        for (int star=0; star<10; star++) 
                            _saver.drawRect(random(disp_saver_width), random(disp_saver_height), 2, 2, hue_to_rgb16(random(255)));      
                }  // _saver.drawPixel(random(disp_saver_width), random(disp_saver_height), gamma(random(0x10000)));
                if (saver_lines_mode) _saver.drawString("do drugs", disp_saver_width / 2, disp_saver_height / 2, 4);
                else if (savercycle != 1) {
                    for (int axis=0; axis<=1; axis++) {
                        eraser_pos[axis] += eraser_velo[axis] * eraser_velo_sign[axis];
                        if (eraser_pos[axis] * eraser_velo_sign[axis] >= eraser_pos_max[axis]) {
                            eraser_pos[axis] = eraser_pos_max[axis] * eraser_velo_sign[axis];
                            eraser_velo[axis] = eraser_velo_min + random(eraser_velo_max - eraser_velo_min);
                            eraser_velo[!axis] = eraser_velo_min + random(eraser_velo_max - eraser_velo_min);
                            eraser_velo_sign[axis] *= -1;
                        }
                    }
                    _saver.fillCircle((disp_saver_width / 2) + eraser_pos[0], (disp_saver_height / 2) + eraser_pos[1], eraser_rad, BLK);
                } 
                star_x0 = star_x1;
                star_y0 = star_y1;
                _saver.pushSprite(disp_simbuttons_x, disp_simbuttons_y);
            }
        }
};