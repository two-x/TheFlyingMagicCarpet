#pragma once
#define disp_width_pix 320  // Horizontal resolution in pixels (held landscape)
#define disp_height_pix 240  // Vertical resolution in pixels (held landscape)
// #include "tft.h"
#include "lgfx.h"
#include "neopixel.h"
#include "touch.h"
#include "images.h"
#include "animations.h"
// #define disp_width_pix 320  // Horizontal resolution in pixels (held landscape)
// #define disp_height_pix 240  // Vertical resolution in pixels (held landscape)
#define disp_vshift_pix 2  // Unknown.  Note: At smallest text size, characters are 5x7 pix + pad on rt and bot for 6x8 pix.
#define disp_runmode_text_x 8
int32_t colorcard[NUM_RUNMODES] = { MGT, WHT, RED, ORG, YEL, GRN, TEAL, PUR };
char modecard[NUM_RUNMODES][7] = { "Basic", "Asleep", "Shutdn", "Stall", "Hold", "Fly", "Cruise", "Cal" };
char side_menu_buttons[5][4] = { "PAG", "SEL", "+  ", "-  ", "SIM" };  // Pad shorter names with spaces on the right
char top_menu_buttons[4][6]  = { " CAL ", "BASIC", " IGN ", "POWER" };  // Pad shorter names with spaces to center
char idlemodecard[IdleControl::idlemodes::NUM_IDLEMODES][7] = { "direct", "cntrol", "minimz" };
char idlestatecard[IdleControl::targetstates::NUM_STATES][7] = { "todriv", "drving", "toidle", "tolow", "idling", "minimz" };
char sensorcard[14][7] = { "none", "joy", "bkpres", "brkpos", "speedo", "tach", "airflw", "mapsns", "engtmp", "batery", "startr", "basic", "ign", "syspwr" };

uint32_t color_16b_to_uint32(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to uint32 in format 0x00RRGGBB
    return (((uint32_t)color565 & 0xf800) << 8) | (((uint32_t)color565 & 0x7e0) << 5) | (((uint32_t)color565 & 0x1f) << 3);
}
uint16_t color_uint32_to_16b(uint32_t color32b) {  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
    return (uint16_t)(((color32b & 0xf80000) >> 8) | ((color32b & 0xfc00) >> 5) | ((color32b & 0xf8) >> 3));
}
uint16_t color_16b_to_8b(uint16_t color565) {  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
    return (uint8_t)(((color565 & 0xe000) >> 8) | ((color565 & 0x700) >> 6) | ((color565 & 0x18) >> 3));
}
class IdiotLight {  // defunct: currently not using individual instances for each idiot light. i couldn't get it to work
    public:
    bool* val = nullptr;
    char letters[3] = "--";
    uint8_t bitmap[11] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    int16_t color = DGRY;
    bool last;  // = 0;
    IdiotLight(bool* _val, uint8_t* _map) : val(_val) {
        for (int i=0; i<11; i++) bitmap[i] = _map[i];
        last = *val;
    } 
};
class IdiotLights {
  public:
    static constexpr int row_count = 11;
    static constexpr int row_height = 11;
    static constexpr int iconcount = 33;  // number of boolean values included on the screen panel (not the neopixels) 
    bool* vals[iconcount] = {
        &(diag.err_sens_alarm[LOST]), &(diag.err_sens_alarm[RANGE]), &(diag.temp_err[ENGINE]), &(diag.temp_err[WHEEL]), &panicstop, 
        hotrc.radiolost_ptr(), &shutdown_incomplete, &park_the_motors, &autostopping, &cruise_adjusting, &car_hasnt_moved, &starter, &bootbutton.now, 
        sim.enabled_ptr(), &running_on_devboard, &powering_up, &(brake.posn_pid_active), &(encoder.enc_a), &(encoder.enc_b), &nowtouch,
        &screensaver, &web_disabled,  // these are just placeholders
        &(sensidiots[0]), &(sensidiots[1]), &(sensidiots[2]), &(sensidiots[3]), &(sensidiots[4]), &(sensidiots[5]), &(sensidiots[6]), 
        &(sensidiots[7]), &(sensidiots[8]), &(sensidiots[9]), &(sensidiots[10]),
    };
    uint8_t icon[iconcount][11] = {
        { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x3e, 0x71, 0x59, 0x4d, 0x47, 0x3e, },  // 0 = "S" w/ crossout symbol
        { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x78, 0x70, 0x58, 0x4d, 0x07, 0x0f, },  // 1 = "S" w/ double arrow
        { 0x7f, 0x7f, 0x6b, 0x6b, 0x00, 0x70, 0x10, 0x10, 0x77, 0x65, 0x07, },  // 2 = "En" w/ degree symbol
        { 0x7f, 0x30, 0x18, 0x30, 0x7f, 0x00, 0x7e, 0x10, 0x77, 0x65, 0x07, },  // 3 = "Wh" w/ degree symbol
        { 0x7f, 0x7f, 0x29, 0x0b, 0x76, 0x10, 0x70, 0x60, 0x00, 0x6f, 0x6f, },  // 4 = "Pn!"
        { 0x7a, 0x7f, 0x4f, 0x17, 0x06, 0x00, 0x63, 0x36, 0x1c, 0x36, 0x63, },  // 5 = hotrc w/ X
        { 0x16, 0x15, 0x0d, 0x60, 0x6f, 0x04, 0x6f, 0x60, 0x0f, 0x69, 0x66, },  // 6 = "SHD..."
        { 0x3e, 0x63, 0x41, 0x7d, 0x7d, 0x55, 0x55, 0x5d, 0x49, 0x63, 0x3e, },  // 7 = circle-"P"
        { 0x3e, 0x49, 0x08, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x08, 0x49, 0x3e, },  // 8 = brake assembly, or tie fighter
        { 0x08, 0x1c, 0x36, 0x00, 0x3e, 0x63, 0x63, 0x00, 0x36, 0x1c, 0x08, },  // 9 = "<C>"
        { 0x1d, 0x23, 0x47, 0x00, 0x3e, 0x63, 0x55, 0x49, 0x55, 0x63, 0x3e, },  // 10 = rotation arrow w/ X wheel
        { 0x3e, 0x41, 0x7f, 0x7b, 0x7b, 0x7b, 0x3e, 0x1c, 0x7f, 0x55, 0x7f, },  // 11 = motor w/ spur gear
        { 0x01, 0x7f, 0x7f, 0x7f, 0x3f, 0x38, 0x74, 0x70, 0x70, 0x70, 0x60, },  // 12 = work boot
        { 0x6e, 0x6b, 0x3b, 0x00, 0x7f, 0x00, 0x7f, 0x06, 0x1c, 0x06, 0x7f, },  // 13 = "SIM"
        { 0x7f, 0x63, 0x3e, 0x00, 0x7f, 0x6b, 0x6b, 0x00, 0x7f, 0x30, 0x1f, },  // 14 = "DEV"
        { 0x00, 0x3e, 0x63, 0x41, 0x40, 0x4f, 0x40, 0x41, 0x63, 0x3e, 0x00, },  // 15 = power symbol
        { 0x7c, 0x46, 0x7f, 0x7f, 0x33, 0x12, 0x12, 0x12, 0x1e, 0x12, 0x0c, },  // 16 = linear actuator, or schlong
        { 0x0e, 0x1d, 0x7d, 0x1d, 0x0e, 0x00, 0x7e, 0x0b, 0x09, 0x0b, 0x7e, },  // 17 = knob "A"
        { 0x0e, 0x1d, 0x7d, 0x1d, 0x0e, 0x00, 0x7f, 0x49, 0x49, 0x7f, 0x36, },  // 18 = knob "B"
        { 0x78, 0x7c, 0x7f, 0x7f, 0x7c, 0x7c, 0x1c, 0x0c, 0x0c, 0x0c, 0x0c, },  // 19 = index finger
        { 0x02, 0x07, 0x32, 0x78, 0x78, 0x30, 0x00, 0x06, 0x0f, 0x0f, 0x06, },  // 20 = bouncy balls
        { 0x36, 0x1c, 0x36, 0x00, 0x22, 0x1c, 0x00, 0x22, 0x1c, 0x41, 0x3e, },  // 21 = wifi symbol w/ X
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, },  // sensors
    };
     // { 0x3e, 0x41, 0x1c, 0x22, 0x08, 0x7c, 0x08, 0x22, 0x1c, 0x41, 0x3e, },  // 21 = X w/ waves
     // { 0x63, 0x36, 0x1c, 0x36, 0x63, 0x14, 0x08, 0x22, 0x1c, 0x41, 0x3e, },  // 21 = wifi symbol w/ X
    char letters[iconcount][3] = {
        "SL", "SR", "\xf7""E", "\xf7""W", "P\x13", "RC", "SI", "Pk", "AS", "Aj", "HM",
        "St", "BB", "Sm", "DB", "PU", "BP", "eA", "eB", "NT",
        "An", "ww",
        "Th", "Br", "St", "RC", "Sp", "Tc", "Pr", "Ps", "Tm", "Ot", "IO",
    };
    uint16_t color[iconcount];
    bool last[iconcount];
    uint8_t idiot_saturation = 225;  // 170-195 makes nice bright yet distinguishable colors
    uint8_t idiot_hue_offset = 240;
    IdiotLights() {
        for (int i=0; i<iconcount; i++) last[i] = *(vals[i]);
        set_colors();
    }
    void setup(NeopixelStrip* _neo) {
        myneo = _neo;
        // int n = new_idiot(&(err_sens_alarm[LOST]), "SL", { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x3e, 0x71, 0x59, 0x4d, 0x47, 0x3e })
        for (int32_t i=0; i<iconcount; i++) myneo->newIdiotLight(i, color[i], val(i));
        std::cout << "Idiot lights.. set up " << iconcount << " toggle icons & " << myneo->idiotcount << " neopixel hazard lights" << std::endl;
    }
    bool val(int index) { return *(vals[index]); }
    bool* ptr(int index) { return vals[index]; }
  private:
    NeopixelStrip* myneo;
    void set_colors() {
        for (int32_t i=0; i<iconcount; i++) {
            int division = row_count;
            uint32_t color32 = hsv_to_rgb<uint32_t>((65535 * (uint16_t)(i % division) / division + idiot_hue_offset), idiot_saturation, 255);  // , 0, 220);
            color[i] = color_uint32_to_16b(color32);  // 5957 = 2^16/11
        }
    }
};
// class SimPanel {};
// class DataPage {};
#define disp_lines 20  // Max lines of text displayable at line height = disp_line_height_pix
#define disp_fixed_lines 8  // Lines of static variables/values always displayed
#define disp_line_height_pix 12  // Pixel height of each text line. Screen can fit 16x 15-pixel or 20x 12-pixel lines
#define disp_font_height 8
#define disp_font_width 6
#define disp_bargraph_width 40
#define disp_bargraph_squeeze 1
#define disp_maxlength 6  // How many characters fit between the ":" and the units string
#define disp_default_float_precision 3  // Significant digits displayed for float values. Higher causes more screen draws
#define disp_datapage_names_x 12
#define disp_datapage_values_x 59
#define disp_datapage_units_x 103        
#define disp_bargraphs_x 122
#define disp_datapage_title_x 83
char disp_values[disp_lines][disp_maxlength+1];  // Holds previously drawn value strings for each line
bool disp_polarities[disp_lines];  // Holds sign of previously drawn values
bool disp_bool_values[6];
bool disp_selected_val_dirty, disp_datapage_dirty, disp_data_dirty, disp_sidemenu_dirty, disp_runmode_dirty, disp_simbuttons_dirty, disp_idiots_dirty;
int32_t disp_needles[disp_lines];
int32_t disp_targets[disp_lines];
int32_t disp_age_quanta[disp_lines];
uint16_t disp_val_colors[disp_lines];
Timer dispAgeTimer[disp_lines];  // int32_t disp_age_timer_us[disp_lines];

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
static constexpr char telemetry[disp_fixed_lines][9] = { "TriggerV", "   Speed", "    Tach", "Throttle", brAk"Pres", brAk"Motr", "JoysticH", stEr"Motr", };  // Fixed rows
static constexpr char units[disp_fixed_lines][5] = { "%   ", "mph ", "rpm ", "%   ", "psi ", "%   ", "%   ", "%   " };  // Fixed rows
static constexpr char brake_pid_card[2][7] = { "presur", "positn" };
static constexpr char pagecard[datapages::NUM_DATAPAGES][5] = { "Run ", "Joy ", "Sens", "PWMs", "Idle", "Bpid", "Gpid", "Cpid", "Temp", "Sim ", "UI  " };
static constexpr int32_t tuning_first_editable_line[datapages::NUM_DATAPAGES] = { 9, 9, 5, 7, 4, 8, 7, 7, 9, 0, 7 };  // first value in each dataset page that's editable. All values after this must also be editable
static constexpr char datapage_names[datapages::NUM_DATAPAGES][disp_tuning_lines][9] = {
    { brAk"Posn", "MuleBatt", "     Pot", "Air Velo", "     MAP", "MasAirFl", __________, __________, __________, "Governor", stEr"Safe", },  // PG_RUN
    { "HRc Horz", "HRc Vert", "HotRcCh3", "HotRcCh4", "TrigVRaw", "JoyH Raw", __________, __________, __________, horfailsaf, "Deadband", },  // PG_JOY
    { "PressRaw", "BkPosRaw", __________, __________, __________, "AirV Max", " MAP Min", " MAP Max", spEd"Idle", spEd"RedL", "BkPos0Pt", },  // PG_SENS
    { "Throttle", "Throttle", brAk"Motr", brAk"Motr", stEr"Motr", stEr"Motr", __________, "ThrotCls", "ThrotOpn", brAk"Stop", brAk"Duty", },  // PG_PWMS
    { "IdlState", "Tach Tgt", "StallIdl", "Low Idle", "HighIdle", "ColdIdle", "Hot Idle", "ColdTemp", "Hot Temp", "SetlRate", "IdleMode", },  // PG_IDLE
    { brAk"Targ", "Pn|PrErr", "  P Term", "  I Term", "  D Term", brAk"Posn", "OutRatio", "MotrDuty", "Brake Kp", "Brake Ki", "Brake Kd", },  // PG_BPID
    { "TachTarg", "Tach Err", "  P Term", "  I Term", "  D Term", "Integral", __________, "OpenLoop", "  Gas Kp", "  Gas Ki", "  Gas Kd", },  // PG_GPID
    { spEd"Targ", "SpeedErr", "  P Term", "  I Term", "  D Term", "Integral", "ThrotSet", maxadjrate, "Cruis Kp", "Cruis Ki", "Cruis Kd", },  // PG_CPID
    { " Ambient", "  Engine", "AxleFrLt", "AxleFrRt", "AxleRrLt", "AxleRrRt", " Touch X", " Touch Y", __________, "Webservr", "No Temps", },  // PG_TEMP
    { "Joystick", brAk"Pres", brAk"Posn", "  Speedo", "    Tach", "AirSpeed", "     MAP", "Basic Sw", " Pot Map", "CalBrake", " Cal Gas", },  // PG_SIM
    { "Loop Avg", "LoopPeak", "LoopFreq", "FramRate", "Draw Clk", "Push Clk", "Idle Clk", "BlnkDemo", neo_bright, "NeoDesat", "Animaton", },  // PG_UI
};
static constexpr char tuneunits[datapages::NUM_DATAPAGES][disp_tuning_lines][5] = {
    { "in  ", "V   ", "%   ", "mph ", "atm ", "g/s ", ______, ______, ______, "%   ", "%   ", },  // PG_RUN
    { "us  ", "us  ", "us  ", "us  ", "%   ", "%   ", ______, ______, ______, "us  ", "us  ", },  // PG_JOY
    { "adc ", "adc ", ______, ______, ______, "mph ", "atm ", "atm ", "mph ", "mph ", "in  ", },  // PG_SENS
    { degree, "us  ", "V   ", "us  ", "V   ", "us  ", ______, degree, degree, "us  ", "%   ", },  // PG_PWMS
    { scroll, "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", "rpm ", degreF, degreF, "rpms", scroll, },  // PG_IDLE
    { "%   ", "psin", "%   ", "%   ", "%   ", "in  ", "%   ", "%   ", ______, "Hz  ", "s   ", },  // PG_BPID
    { "rpm ", "rpm ", "%   ", "%   ", "%   ", "%   ", ______, b1nary, ______, "Hz  ", "s   ", },  // PG_GPID
    { "mph ", "mph ", "rpm ", "rpm ", "rpm ", "rpm ", "%   ", "%/s ", ______, "Hz  ", "s   ", },  // PG_CPID
    { degreF, degreF, degreF, degreF, degreF, degreF, "pix ", "pix ", ______, b1nary, b1nary, },  // PG_TEMP
    { b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, scroll, b1nary, b1nary, },  // PG_SIM
    { "us  ", "us  ", "Hz  ", "fps ", "us  ", "us  ", "us  ", b1nary, "%   ", "/10 ", "eyes", },  // PG_UI
};
static constexpr char unitmapnames[9][5] = { "usps", "us  ", "rpms", scroll, b1nary, "%   ", "ohm ", "eyes", "psin", };  // unit strings matching these will get replaced by the corresponding bitmap graphic below
static constexpr uint8_t unitmaps[9][17] = {  // 17x7-pixel bitmaps for where units use symbols not present in the font, are longer than 3 characters, or are just special
    { 0x7e, 0x20, 0x20, 0x3c, 0x00, 0x24, 0x2a, 0x2a, 0x12, 0x00, 0x70, 0x0e, 0x00, 0x24, 0x2a, 0x2a, 0x12, },  // usps - microseconds per second
    { 0x40, 0x7e, 0x20, 0x20, 0x1c, 0x20, 0x00, 0x24, 0x2a, 0x2a, 0x2a, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, },  // us - b/c the font's "mu" character doesn't work
    { 0x1f, 0x01, 0x00, 0x3f, 0x09, 0x0e, 0x00, 0x0f, 0x01, 0x0e, 0x01, 0x0e, 0x60, 0x1c, 0x00, 0x58, 0x74, },  // rpm/s (or rot/m*s) - rate of change of engine rpm
    { 0x04, 0x02, 0x7f, 0x02, 0x04, 0x00, 0x10, 0x20, 0x7f, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // scroll arrows - to indicate multiple choice
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x1c, 0x22, 0x22, 0x1c, 0x00, 0x00, },  // 0/1 - to indicate binary value
    { 0x02, 0x45, 0x25, 0x12, 0x08, 0x24, 0x52, 0x51, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // % - just because the font one is feeble
    { 0x4e, 0x51, 0x61, 0x01, 0x61, 0x51, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // capital omega - for ohms
    { 0x08, 0x1c, 0x2a, 0x08, 0x00, 0x3e, 0x63, 0x63, 0x77, 0x7f, 0x41, 0x3e, 0x63, 0x63, 0x77, 0x7f, 0x3e, },  // googly eyes, are as goofy as they are stupid
    { 0x3d, 0x00, 0x3e, 0x02, 0x3c, 0x00, 0x7f, 0x00, 0x7e, 0x22, 0x1c, 0x00, 0x2c, 0x2a, 0x1a, 0x00, 0x3d, },  // inches or psi "in|psi"
};  // These bitmaps are in the same format as the idiot light bitmaps, described below
//  { 0x7e, 0x20, 0x3e, 0x20, 0x00, 0x0c, 0x52, 0x4a, 0x3c, 0x00, 0x60, 0x18, 0x06, 0x00, 0x2c, 0x2a, 0x32, },  // ug/s - for manifold mass airflow
class TunerPanel {
  public:
    TunerPanel() {};
    void setup() {};
  private:
    // DataPage[NUM_DATAPAGES];
};
AnimationManager animations;
FlexPanel flexpanel;
volatile float fps = 0.0;
volatile bool is_pushing = 0;
volatile bool is_drawing = 0;
volatile bool pushtime = 0;
volatile int32_t pushclock;
volatile int32_t drawclock;
volatile int32_t idleclock;
// static void push_task(void*) {
#ifdef VIDEO_TASKS
static void push_task(void *parameter) {
    while (true) {
        while (!(screensaver || sim.enabled()) || !pushtime || !(screenRefreshTimer.expired() || screensaver_max_refresh))  // taskYIELD(); || sim.enabled()
            vTaskDelay(pdMS_TO_TICKS(1));
        screenRefreshTimer.reset();
        is_pushing = true;
        if (fullscreen_screensaver_test) flexpanel->diffpush(&flexpanel_sp[flip], &flexpanel_sp[!flip]);
        else flexpanel->diffpush(&flexpanel_sp[flip], &flexpanel_sp[!flip], disp_simbuttons_x, disp_simbuttons_y, disp_simbuttons_w, disp_simbuttons_h);
        pushclock = (int32_t)screenRefreshTimer.elapsed();
        flip = !flip;
        is_pushing = pushtime = false;  // drawn = 
    }
    // vTaskDelete(NULL);
}
static void draw_task(void *parameter) {
    while (true) {
        while (!(screensaver || sim.enabled()) || pushtime) vTaskDelay(pdMS_TO_TICKS(1));  //   || sim.enabled()
        is_drawing = true;
        int32_t mark = (int32_t)screenRefreshTimer.elapsed();
        if (screensaver) fps = animations.update();
        // if (sim.enabled()) draw_simbuttons();
        drawclock = (int32_t)screenRefreshTimer.elapsed() - mark;
        idleclock = refresh_limit - pushclock - drawclock;
        is_drawing = false;  // pushed = false;
        pushtime = true;
    }
}
#else
void push_task() {
    is_pushing = true;
    if (fullscreen_screensaver_test) flexpanel->diffpush(&flexpanel_sp[flip], &flexpanel_sp[!flip]);
    else flexpanel->diffpush(&flexpanel_sp[flip], &flexpanel_sp[!flip], disp_simbuttons_x, disp_simbuttons_y, disp_simbuttons_w, disp_simbuttons_h);
    flip = !flip;
    is_pushing = pushtime = false;  // drawn = 
}
void draw_task() {
    is_drawing = true;
    int32_t mark = (int32_t)screenRefreshTimer.elapsed();
    flexpanel_sp[flip].setAddrWindow(disp_simbuttons_x, disp_simbuttons_y, disp_simbuttons_w, disp_simbuttons_h))
    flexpanel_sp[flip].setClipRect(disp_simbuttons_x, disp_simbuttons_y, disp_simbuttons_w, disp_simbuttons_h))
    fps = animations.update();
    flexpanel_sp[flip].clearClipRect();
    flexpanel_sp[flip].setAddrWindow(0, 0, disp_width_pix, disp_height_pix);
    drawclock = (int32_t)screenRefreshTimer.elapsed() - mark;
    idleclock = refresh_limit - pushclock - drawclock;
    is_drawing = false;  // pushed = false;
    pushtime = true;
}
#endif
class Display {
  private:
    LGFX _tft = LGFX();
    NeopixelStrip* neo;
    Touchscreen* touch;
    Simulator* sim;
    TunerPanel tuner;
    IdiotLights* idiots;
    Timer valuesRefreshTimer = Timer(160000);  // Don't refresh screen faster than this (16667us = 60fps, 33333us = 30fps, 66666us = 15fps)
    uint16_t touch_cal_data[5] = { 404, 3503, 460, 3313, 1 };  // Got from running TFT_eSPI/examples/Generic/Touch_calibrate/Touch_calibrate.ino
    bool _procrastinate = false, reset_finished = false, simulating_last;
    int disp_oldmode = SHUTDOWN;   // So we can tell when  the mode has just changed. start as different to trigger_mode start algo    
    uint8_t palettesize = 2;
    uint16_t palette[256] = { TFT_BLACK, TFT_WHITE };
    static constexpr int runOnCore = CONFIG_ARDUINO_RUNNING_CORE == 0 ? 1 : 0;    
  public:
    static constexpr int idiots_corner_x = 165;
    static constexpr int idiots_corner_y = 13;
    Display(NeopixelStrip* _neo, Touchscreen* _touch, IdiotLights* _idiots, Simulator* _sim)
        : _tft(), neo(_neo), touch(_touch), idiots(_idiots), sim(_sim) {
        // if (fullscreen_screensaver_test)
            flexpanel.init(&_tft, touch, 0, 0, disp_width_pix, disp_height_pix);
        // else flexpanel.init(&_tft, touch, disp_simbuttons_x, disp_simbuttons_y, disp_simbuttons_w, disp_simbuttons_h);
        animations.init(&flexpanel, sim);
    }
    Display(int8_t cs_pin, int8_t dc_pin, NeopixelStrip* _neo, Touchscreen* _touch, IdiotLights* _idiots, Simulator* _sim) 
        : _tft(), neo(_neo), touch(_touch), idiots(_idiots), sim(_sim) {
        Display(_neo, _touch, _idiots, _sim);
    }
    LGFX* get_tft() {
        return &_tft;
    }
    void init_tasks() {
        #ifdef VIDEO_TASKS
        TaskHandle_t pushTaskHandle = nullptr;
        xTaskCreatePinnedToCore(push_task, "taskPush", 8192, NULL, 2, &pushTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);  // 16384
        TaskHandle_t drawTaskHandle = nullptr;
        xTaskCreatePinnedToCore(draw_task, "taskDraw", 4096, NULL, 3, &drawTaskHandle, runOnCore);
        #endif

        // TaskHandle_t drawTaskHandle = nullptr;
        // xTaskCreateUniversal([](void*) {
        //     while(true) {
        //         while (!screensaver || sim->enabled() || pushtime) taskYIELD();
        //         // if (screensaver && !sim->enabled() && !pushtime)
        //         draw_task();
        //         // delayMicroseconds(25); // allow for wifi etc
        //     }
        // }, "drawTask", 4096 , NULL, 1, &drawTaskHandle, runOnCore);  //  8192
        //
        // TaskHandle_t pushTaskHandle = nullptr;
        // xTaskCreateUniversal([](void*) {
        //     while(true) {
        //         while (!screensaver || sim->enabled() || !pushtime || !(screenRefreshTimer.expired() || screensaver_max_refresh))
        //         // if (screensaver && !sim->enabled() && pushtime && (screenRefreshTimer.expired() || screensaver_max_refresh)) {
        //             taskYIELD();
        //         screenRefreshTimer.reset();
        //         push_task();
        //         // taskYIELD();
        //         // delayMicroseconds(25); // allow for wifi etc
        //     }
        // }, "pushTask", 16384 , NULL, 5, &pushTaskHandle, runOnCore);  //  8192
    }
    void make_sprites() {

    }
    void setup() {
        Serial.printf("Display..");  //
        _tft.init();
        // _tft.setAttribute(PSRAM_ENABLE, true);  // enable use of PSRAM
        _tft.setColorDepth(8);
        Serial.printf(" ..");  //

        _tft.begin();  // _tft.begin();
        Serial.printf(" ..");  //

        _tft.initDMA();
        Serial.printf(" ..");  //
        // _tft.setRotation((flip_the_screen) ? 3 : 1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
        if (_tft.width() < _tft.height()) _tft.setRotation(_tft.getRotation() ^ 1);
        // _tft.setTouch(touch_cal_data);
        _tft.setSwapBytes(true);  // rearranges color ordering of 16bit colors when displaying image files
        for (int32_t lineno=0; lineno <= disp_fixed_lines; lineno++)  {
            disp_age_quanta[lineno] = -1;
            memset(disp_values[lineno], 0, strlen(disp_values[lineno]));
            disp_polarities[lineno] = 1;
        }
        for (int32_t row=0; row<arraysize(disp_bool_values); row++) disp_bool_values[row] = 1;
        for (int32_t row=0; row<arraysize(disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
        for (int32_t row=0; row<arraysize(disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen
        yield();
        Serial.printf(" ..");  //
        _tft.fillScreen(TFT_BLACK);  // Black out the whole screen
        if (!fullscreen_screensaver_test) {
            Serial.printf(" ..");  //
            draw_touchgrid(false);
            draw_fixed(datapage, datapage_last, false);
            draw_idiotlights(idiots_corner_x, idiots_corner_y, true);
            all_dirty();
        }
        animations.setup();
        init_tasks();
        Serial.printf(" initialized\n");
    }
    // uint8_t add_palette(uint16_t color) {
    //     for (uint8_t i=0; i<palettesize; i++) if (_tft.getPaletteColor(i) == color) return i;
    //     _tft.setPaletteColor(palettesize++, color);
    //     return palettesize;
    // }
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
    void set_runmodecolors() {
        uint8_t saturat = 255;  uint8_t hue_offset = 0;
        for (int32_t rm=0; rm<NUM_RUNMODES; rm++) {
            int division = NUM_RUNMODES;
            uint32_t color32 = hsv_to_rgb<uint32_t>((65536 * (uint16_t)(rm % division) / division + hue_offset), saturat, 255);  // , 0, 220);
            colorcard[rm] = color_uint32_to_16b(color32);  // 5957 = 2^16/11
            disp_runmode_dirty = true;
        }
    }
    void tiny_text() {
        _tft.setTextDatum(textdatum_t::top_left);
        _tft.setFont(&fonts::Font0);
    }
    uint16_t darken_color(uint16_t color, int32_t halvings = 1) {  // halves each of r, g, and b of a 5-6-5 formatted 16-bit color value either once or twice
        if (halvings == 1) return ((color & 0xf000) | (color & 0x7c0) | (color & 0x1e)) >> 1;
        else return ((color & 0xe000) | (color & 0x780) | (color & 0x1c)) >> 2;
    }
  private:
    void draw_bargraph_base(int32_t corner_x, int32_t corner_y, int32_t width) {  // draws a horizontal bargraph scale.  124, y, 40
        _tft.drawFastHLine(corner_x+disp_bargraph_squeeze, corner_y, width-disp_bargraph_squeeze*2, GRY1);
        for (int32_t offset=0; offset<=2; offset++) _tft.drawFastVLine((corner_x+disp_bargraph_squeeze)+offset*(width/2 - disp_bargraph_squeeze), corner_y-1, 3, WHT);
    }
    void draw_needle_shape(int32_t pos_x, int32_t pos_y, int32_t color) {  // draws a cute little pointy needle
        _tft.drawFastVLine(pos_x-1, pos_y, 2, color);
        _tft.drawFastVLine(pos_x, pos_y, 4, color);
        _tft.drawFastVLine(pos_x+1, pos_y, 2, color);
    }
    void draw_target_shape(int32_t pos_x, int32_t pos_y, int32_t t_color, int32_t r_color) {  // draws a cute little target symbol
        _tft.drawFastVLine(pos_x-1, pos_y+7, 2, t_color);
        _tft.drawFastVLine(pos_x, pos_y+5, 4, t_color);
        _tft.drawFastVLine(pos_x+1, pos_y+7, 2, t_color);
    }
    void draw_bargraph_needle(int32_t n_pos_x, int32_t old_n_pos_x, int32_t pos_y, int32_t n_color) {  // draws a cute little pointy needle
        draw_needle_shape(old_n_pos_x, pos_y, TFT_BLACK);
        draw_needle_shape(n_pos_x, pos_y, n_color);
    }
    void draw_string(int32_t x_new, int32_t x_old, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor, bool forced=false) {  // Send in "" for oldtext if erase isn't needed
        int32_t oldlen = strlen(oldtext);
        int32_t newlen = strlen(text);
        _tft.setTextColor(bgcolor);  
        for (int32_t letter=0; letter < oldlen; letter++) {
            if (newlen - letter < 1) {
                _tft.setCursor(x_old+disp_font_width*letter, y);
                _tft.print(oldtext[letter]);
            }
            else if (oldtext[letter] != text[letter]) {
                _tft.setCursor(x_old+disp_font_width*letter, y);
                _tft.print(oldtext[letter]);
            }
        }
        _tft.setTextColor(color);  
        for (int32_t letter=0; letter < newlen; letter++) {
            if (oldlen - letter < 1) {
                _tft.setCursor(x_new+disp_font_width*letter, y);
                _tft.print(text[letter]);
            }
            else if (oldtext[letter] != text[letter] || forced) {
                _tft.setCursor(x_new+disp_font_width*letter, y);
                _tft.print(text[letter]);
            }
        }
    }
    void draw_unitmap(int8_t index, int32_t x, int32_t y, uint16_t color) {
        for (int32_t xo = 0; xo < disp_font_width * 3 - 1; xo++)
            for (int32_t yo = 0; yo < disp_font_height - 1; yo++)
                if ((unitmaps[index][xo] >> yo) & 1) _tft.drawPixel(x + xo, y + yo, color);
    }
    void draw_string_units(int32_t x, int32_t y, const char* text, const char* oldtext, int32_t color, int32_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
        bool drawn = false;
        for (int8_t i = 0; i<arraysize(unitmaps); i++)
            if (!strcmp(unitmapnames[i], oldtext)) {
                draw_unitmap(i, x, y, bgcolor);
                drawn = true;
            }
        if (!drawn) {
            _tft.setCursor(x, y);
            _tft.setTextColor(bgcolor);
            _tft.print(oldtext);  // Erase the old content
        }
        for (int8_t i = 0; i<arraysize(unitmaps); i++)
            if (!strcmp(unitmapnames[i], text)) {
                draw_unitmap(i, x, y, color);
                return;
            }
        _tft.setCursor(x, y);
        _tft.setTextColor(color);
        _tft.print(text);  // Erase the old content
    }
    // draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
    void draw_fixed(int32_t page, int32_t page_last, bool redraw_tuning_corner, bool forced=false) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
        _tft.setTextColor(GRY2);
        _tft.setTextSize(1);
        int32_t y_pos;
        if (!redraw_tuning_corner) {
            for (int32_t lineno = 0; lineno < disp_fixed_lines; lineno++)  {  // Step thru lines of fixed telemetry data
                y_pos = (lineno + 1) * disp_line_height_pix + disp_vshift_pix;
                draw_string(disp_datapage_names_x, disp_datapage_names_x, y_pos, telemetry[lineno], "", GRY2, TFT_BLACK, forced);
                draw_string_units(disp_datapage_units_x, y_pos, units[lineno], "", GRY2, TFT_BLACK);
                draw_bargraph_base(disp_bargraphs_x, y_pos + 7, disp_bargraph_width);
            }
        }
        for (int32_t lineno=0; lineno < disp_tuning_lines; lineno++)  {  // Step thru lines of dataset page data
            draw_string(disp_datapage_names_x, disp_datapage_names_x, (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix, datapage_names[page][lineno], datapage_names[page_last][lineno], GRY2, TFT_BLACK, forced);
            draw_string_units(disp_datapage_units_x, (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix, tuneunits[page][lineno], tuneunits[page_last][lineno], GRY2, TFT_BLACK);
            if (redraw_tuning_corner) {
                int32_t corner_y = (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix + 7;  // lineno*disp_line_height_pix+disp_vshift_pix-1;
                draw_bargraph_base(disp_bargraphs_x, corner_y, disp_bargraph_width);
                if (disp_needles[lineno] >= 0) draw_bargraph_needle(-1, disp_needles[lineno], corner_y - 6, TFT_BLACK);  // Let's draw a needle
            }
        }
    }
    void draw_hyphen(int32_t x_pos, int32_t y_pos, int32_t color) {  // Draw minus sign in front of negative numbers
        _tft.drawFastHLine(x_pos+2, y_pos+3, 3, color);
    }
    void draw_dynamic(int32_t lineno, char const* disp_string, int32_t value, int32_t lowlim, int32_t hilim, int32_t target=-1, int32_t color=-1) {
        int32_t age_us = (color >= 0) ? 11 : (int32_t)((float)(dispAgeTimer[lineno].elapsed()) / 2500000); // Divide by us per color gradient quantum
        int32_t x_base = disp_datapage_values_x;
        bool polarity = (value >= 0);  // polarity 0=negative, 1=positive
        if (strcmp(disp_values[lineno], disp_string) || value == 1234567 || disp_data_dirty) {  // If value differs, Erase old value and write new
            if (color == -1) color = GRN;
            int32_t y_pos = lineno*disp_line_height_pix+disp_vshift_pix;
            if (polarity != disp_polarities[lineno]) draw_hyphen(x_base, y_pos, (!polarity) ? color : TFT_BLACK);
            draw_string(x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_string, disp_values[lineno], color, TFT_BLACK, (color != disp_val_colors[lineno])); // +6*(arraysize(modecard[run.mode])+4-namelen)/2
            strcpy(disp_values[lineno], disp_string);
            disp_polarities[lineno] = polarity;
            disp_val_colors[lineno] = color;
            dispAgeTimer[lineno].reset();
            disp_age_quanta[lineno] = 0;
        }  // to-do: Fix failure to freshen aged coloration of unchanged characters of changed values
        else if (age_us > disp_age_quanta[lineno] && age_us < 11)  {  // As readings age, redraw in new color. This may fail and redraw when the timer overflows? 
            if (age_us < 8) color = 0x1fe0 + age_us*0x2000;  // Base of green with red added as you age, until yellow is achieved
            else color = 0xffe0 - (age_us-8) * 0x100;  // Then lose green as you age further
            int32_t y_pos = (lineno)*disp_line_height_pix+disp_vshift_pix;
            if (!polarity) draw_hyphen(x_base, y_pos, color);
            draw_string(x_base+disp_font_width, x_base+disp_font_width, y_pos, disp_values[lineno], "", color, TFT_BLACK);
            disp_age_quanta[lineno] = age_us;
            disp_val_colors[lineno] = color;
        }
        if (lowlim < hilim) {  // Any value having a given range deserves a bargraph gauge with a needle
            int32_t corner_x = disp_bargraphs_x;    
            int32_t corner_y = lineno*disp_line_height_pix+disp_vshift_pix-1;
            int32_t n_pos = map(value, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
            int32_t ncolor = (n_pos > disp_bargraph_width-disp_bargraph_squeeze || n_pos < disp_bargraph_squeeze) ? BRN : GRN;
            n_pos = corner_x + constrain(n_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
            if (target != -1) {  // If target value is given, draw a target on the bargraph too
                int32_t t_pos = map(target, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                int32_t tcolor = (t_pos > disp_bargraph_width-disp_bargraph_squeeze || t_pos < disp_bargraph_squeeze) ? BRN : ( (t_pos != n_pos) ? YEL : GRN );
                t_pos = corner_x + constrain(t_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                if (t_pos != disp_targets[lineno] || (t_pos == n_pos)^(disp_needles[lineno] != disp_targets[lineno]) || disp_data_dirty) {
                    draw_target_shape(disp_targets[lineno], corner_y, TFT_BLACK, -1);  // Erase old target
                    _tft.drawFastHLine(disp_targets[lineno]-(disp_targets[lineno] != corner_x+disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+7, 2+(disp_targets[lineno] != corner_x+disp_bargraph_width-disp_bargraph_squeeze), GRY1);  // Patch bargraph line where old target got erased
                    for (int32_t offset=0; offset<=2; offset++) _tft.drawFastVLine((corner_x+disp_bargraph_squeeze)+offset*(disp_bargraph_width/2 - disp_bargraph_squeeze), lineno*disp_line_height_pix+disp_vshift_pix+6, 3, WHT);  // Redraw bargraph graduations in case one got corrupted by target erasure
                    draw_target_shape(t_pos, corner_y, tcolor, -1);  // Draw the new target
                    disp_targets[lineno] = t_pos;  // Remember position of target
                }
            }
            if (n_pos != disp_needles[lineno] || disp_data_dirty) {
                draw_bargraph_needle(n_pos, disp_needles[lineno], corner_y, ncolor);  // Let's draw a needle
                disp_needles[lineno] = n_pos;  // Remember position of needle
            }
        }
        else if (disp_needles[lineno] >= 0) {  // If value having no range is drawn over one that did ...
            draw_bargraph_needle(-1, disp_needles[lineno], lineno*disp_line_height_pix+disp_vshift_pix-1, TFT_BLACK);  // Erase the old needle
            disp_needles[lineno] = -1;  // Flag for no needle
        }
    }
    int32_t significant_place(float value) {  // Returns the decimal place of the most significant digit of a positive float value, without relying on logarithm math
        int32_t place = 1;
        if (value >= 1) { // int32_t vallog = std::log10(value);  // Can be sped up
            while (value >= 10) {
                value /= 10;
                place++;  // ex. 100.34 -> 3
            }
        }
        else if (value) {  // checking (value) rather than (value != 0.0) can help avoid precision errors caused by digital representation of floating numbers
            place = 0;
            while (value < 1) {
                value *= 10;
                place--;  // ex. 0.00334 -> -3
            }
        }
        return place;
    }
    int32_t significant_place(int32_t value) {  // Returns the length in digits of a positive integer value
        int32_t place = 1;
        while (value >= 10) {
            value /= 10;
            place++;
        }
        return place;
    }
    std::string num2string(int32_t value, int32_t maxlength) {  // returns an ascii string representation of a given integer value, using scientific notation if necessary to fit within given width constraint
        value = abs(value);  // This function disregards sign
        int32_t place = significant_place(value);  // check how slow is log() function? Compare performance vs. multiple divides ( see num2string() )
        if (place <= maxlength) return std::to_string(value);  // If value is short enough, return it
        char buffer[maxlength+1];  // Allocate buffer with the maximum required size
        std::snprintf(buffer, sizeof(buffer), "%.*e", maxlength - 4 - (int)(place >= 10), (float)value);
        std::string result(buffer);  // copy buffer to result
        return result.substr(0, result.find('e') + 1) + std::to_string(place);
    }
    std::string num2string(float value, int32_t maxlength, int32_t sigdig, bool chop_zeroes = true) {  // returns an ascii string representation of a given float value, formatted efficiently. It will not exceed maxlength. fractional digits will be removed respecting given number of significant digits
        value = abs(value);  // This function disregards sign
        int32_t place = significant_place(value);  // Learn decimal place of the most significant digit in value
        if (place >= sigdig && place <= maxlength) {  // Then we want simple cast to an integer w/o decimal point (eg 123456, 12345, 1234)
            std::string result(std::to_string((int32_t)value));
            return result;
        }
        if (place >= 0 && place < maxlength) {  // Then we want float formatted with enough nonzero digits after the decimal point for given significant digits (eg 123.4, 12.34, 1.234, 0.000)
            int32_t length = smin(sigdig+1, maxlength);
            char buffer[length+1];
            std::snprintf(buffer, length + 1, (chop_zeroes) ? "%.*g" : "%.*f", length - 1, value);  // (buf, letters incl. end, %.*g = floats formatted in shortest form, length-1 digits after decimal, val)
            std::string result(buffer);  // copy buffer to result            
            if (value != 0.0 && chop_zeroes && result.find('.') != std::string::npos) result = result.substr(0, result.find_last_not_of('0') + 1);
            if (result.back() == '.') result.pop_back();
            return result;
        }
        if (place < 0 && sigdig - place <= maxlength) {  // Then we want decimal w/o initial '0' limited to given significant digits (eg .123, .0123, .00123)
            std::string result (std::to_string(value));  // sd=3,  0.1234  d=1 l=6    0.00123
            size_t decimalPos = result.find('.');  // decimalPos will always be 1 (?)
            if (decimalPos != std::string::npos) result = result.substr(decimalPos, smin(sigdig-place, maxlength));  // Remove any digits to the left of the decimal point
            return result;
        }  // Otherwise we want scientific notation with precision removed as needed to respect maxlength (eg 1.23e4, 1.23e5, but using long e character not e for negative exponents
        char buffer[maxlength+1];  // Allocate buffer with the maximum required size
        int32_t truncit = smin(sigdig - 1, maxlength - 4 - (int)(place <= -10 || place >= 10));
        std::snprintf(buffer, sizeof(buffer), "%.*e", truncit, value);
        std::string result(buffer);  // copy buffer to result
        if (result.find("e+0") != std::string::npos) result.replace(result.find("e+0"), 3, "e");  // Remove useless "+0" from exponent
        else if (result.find("e-0") != std::string::npos) result.replace(result.find("e-0"), 3, "\x88");  // For very small scientific notation values, replace the "e-0" with a phoenetic long e character, to indicate negative power  // if (result.find("e-0") != std::string::npos) 
        else if (result.find("e+") != std::string::npos) result.replace(result.find("e+"), 2, "e");  // For ridiculously large values
        else if (result.find("e-") != std::string::npos) result.replace(result.find("e-"), 2, "\x88");  // For ridiculously small values
        return result;
    }
    void draw_dynamic(int32_t lineno, int32_t value, int32_t lowlim=-1, int32_t hilim=-1, int32_t target=-1) {
        std::string val_string = num2string(value, (int32_t)disp_maxlength);
        draw_dynamic(lineno, val_string.c_str(), value, lowlim, hilim, (int32_t)target);
    }
    void draw_dynamic(int32_t lineno, float value, float lowlim=-1, float hilim=-1, int32_t target=-1, int32_t precision = disp_default_float_precision) {
        std::string val_string = num2string(value, (int32_t)disp_maxlength, precision);
        draw_dynamic(lineno, val_string.c_str(), (int32_t)value, (int32_t)lowlim, (int32_t)hilim, target);
    }
    void draw_dynamic(int32_t lineno, float value, float lowlim, float hilim, float target, int32_t precision = disp_default_float_precision) {
        draw_dynamic(lineno, value, lowlim, hilim, (int32_t)target, precision);
    }
    void draw_eraseval(int32_t lineno) {
        draw_dynamic(lineno, "", 1234567, -1, -1, -1);
    }
    void draw_asciiname(int32_t lineno, String name) {
        draw_dynamic(lineno, name.c_str(), 1, -1, -1, -1, CYN);
    }
    void draw_truth(int32_t lineno, bool truthy, int32_t styl=2) {  // 0:on/off, 1:yes/no, 2:true/false .
        draw_dynamic(lineno, (truthy) ? ((styl==0) ? "on" : ((styl==1) ? "yes" : "true")) : ((styl==0) ? "off" : ((styl==1) ? "no" : "false")), 1, -1, -1, -1, (truthy) ? LPUR : GPUR);
    }
    void draw_runmode(int32_t _nowmode, int32_t _oldmode, int32_t color_override=-1) {  // color_override = -1 uses default color
        int32_t color = (color_override == -1) ? colorcard[_nowmode] : color_override;
        int32_t x_new = disp_runmode_text_x + disp_font_width * (2 + strlen(modecard[_nowmode])) - 3;
        int32_t x_old = disp_runmode_text_x + disp_font_width * (2 + strlen(modecard[_oldmode])) - 3;
        draw_string(disp_runmode_text_x + disp_font_width, disp_runmode_text_x + disp_font_width, disp_vshift_pix, modecard[_oldmode], "", TFT_BLACK, TFT_BLACK); // +6*(arraysize(modecard[_nowmode])+4-namelen)/2
        draw_string(x_old, x_old, disp_vshift_pix, "Mode", "", TFT_BLACK, TFT_BLACK); // +6*(arraysize(modecard[_nowmode])+4-namelen)/2
        draw_string(disp_runmode_text_x + disp_font_width, disp_runmode_text_x + disp_font_width, disp_vshift_pix, modecard[_nowmode], "", color, TFT_BLACK); // +6*(arraysize(modecard[_nowmode])+4-namelen)/2
        draw_string(x_new, x_new, disp_vshift_pix, "Mode", "", color, TFT_BLACK); // +6*(arraysize(modecard[_nowmode])+4-namelen)/2
    }
    void draw_datapage(int32_t page, int32_t page_last, bool forced=false) {
        draw_fixed(page, page_last, true, forced);  // Erase and redraw dynamic data corner of screen with names, units etc.
        draw_string(disp_datapage_title_x, disp_datapage_title_x, disp_vshift_pix, pagecard[page], pagecard[page_last], STBL, TFT_BLACK, forced); // +6*(arraysize(modecard[_runmode.mode()])+4-namelen)/2
    }
    void draw_selected_name(int32_t tun_ctrl, int32_t tun_ctrl_last, int32_t selected_val, int32_t selected_last) {
        if (selected_val != selected_last) draw_string(12, 12, 12+(selected_last+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, datapage_names[datapage][selected_last], "", GRY2, TFT_BLACK);
        draw_string(12, 12, 12+(selected_val+disp_fixed_lines)*disp_line_height_pix+disp_vshift_pix, datapage_names[datapage][selected_val], "", (tun_ctrl == EDIT) ? GRN : ((tun_ctrl == SELECT) ? YEL : GRY2), TFT_BLACK);
    }
    void draw_bool(bool value, int32_t col) {  // Draws values of boolean data
        if ((disp_bool_values[col-2] != value) || disp_data_dirty) {  // If value differs, Erase old value and write new
            int32_t x_mod = touch_margin_h_pix + touch_cell_h_pix*(col) + (touch_cell_h_pix>>1) - arraysize(top_menu_buttons[col-2]-1)*(disp_font_width>>1) - 2;
            draw_string(x_mod, x_mod, 0, top_menu_buttons[col-2], "", (value) ? GRN : LGRY, DGRY);
            disp_bool_values[col-2] = value;
        }
    }
    void draw_touchgrid(bool side_only) {  // draws edge buttons with names in 'em. If replace_names, just updates names
        int32_t namelen = 0;
        _tft.setTextColor(WHT);
        for (int32_t row = 0; row < arraysize(side_menu_buttons); row++) {  // Step thru all rows to draw buttons along the left edge
            _tft.fillRoundRect(-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, DGRY);
            _tft.drawRoundRect(-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, LYEL);
            namelen = 0;
            for (uint32_t x = 0 ; x < arraysize(side_menu_buttons[row]) ; x++ ) {
                if (side_menu_buttons[row][x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
            }
            for (int32_t letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
                _tft.setCursor(1, ( touch_cell_v_pix*row) + (touch_cell_v_pix/2) - (int32_t)(4.5*((float)namelen-1)) + (disp_font_height+1)*letter); // adjusts vertical offset depending how many letters in the button name and which letter we're on
                _tft.println(side_menu_buttons[row][letter]);  // Writes each letter such that the whole name is centered vertically on the button
            }
        }
        if (!side_only) {
            for (int32_t col = 2; col <= 5; col++) {  // Step thru all cols to draw buttons across the top edge
                _tft.fillRoundRect(touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
                _tft.drawRoundRect(touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, LYEL);  // _tft.width()-9, 3, 18, (_tft.height()/5)-6, 8, LYEL);
            }
        }
    }
    void draw_reticle(LGFX_Sprite* spr, uint32_t x, uint32_t y) {
        spr->drawFastHLine(x - 2, y, 5, DGRY);
        spr->drawFastVLine(x, y - 2, 5, DGRY);
    }
    void draw_reticles(LGFX_Sprite* spr) {
        if (touch_reticles) {
            draw_reticle(spr, disp_width_pix-touch_reticle_offset, touch_reticle_offset);
            draw_reticle(spr, touch_reticle_offset, touch_reticle_offset);
            draw_reticle(spr, touch_reticle_offset, disp_height_pix-touch_reticle_offset);
            draw_reticle(spr, disp_width_pix-touch_reticle_offset, disp_height_pix-touch_reticle_offset);
        }
    }
    void draw_idiotbitmap(int i, int32_t x, int32_t y) {
        uint16_t bg = idiots->val(i) ? idiots->color[i] : TFT_BLACK;
        uint16_t color = idiots->val(i) ? TFT_BLACK : darken_color(idiots->color[i]);
        _tft.drawRoundRect(x, y, 2 * disp_font_width + 1, disp_font_height + 1, 1, bg);
        for (int xo = 0; xo < (2 * disp_font_width - 1); xo++)
            for (int yo = 0; yo < disp_font_height - 1; yo++)
                _tft.drawPixel(x + xo + 1, y + yo + 1, ((idiots->icon[i][xo] >> yo) & 1) ? color : bg);
    }
    void draw_idiotlight(int32_t i, int32_t x, int32_t y) {
        if (idiots->icon[i][0] == 0xff) {  // 0xff in the first byte will draw 2-letter string instead of bitmap
            _tft.fillRoundRect(x, y, 2 * disp_font_width + 1, disp_font_height + 1, 2, (idiots->val(i) ? idiots->color[i] : TFT_BLACK));  // GRY1);
            _tft.setTextColor(idiots->val(i) ? TFT_BLACK : darken_color(idiots->color[i]));  // darken_color((*(idiots->lights[index])) ? TFT_BLACK : DGRY)
            _tft.setCursor(x+1, y+1);
            _tft.print(idiots->letters[i]);
        }
        else if (idiots->icon[i][0] != 0x88) draw_idiotbitmap(i, x, y);  // 0x88 in the first byte will skip a space
        idiots->last[i] = idiots->val(i);
    }
    void draw_idiotlights(int32_t x, int32_t y, bool force = false) {
        for (int i=0; i < idiots->iconcount; i++)
            if (force || (idiots->val(i) ^ idiots->last[i]))
                draw_idiotlight(i, x + (2 * disp_font_width + 2) * (i % idiots->row_count), y + idiots->row_height * (int32_t)(i / idiots->row_count));
    }
    void draw_temperature(loc location, int draw_index) {
        if (!tempsens.detected(location)) draw_eraseval(draw_index);
        else draw_dynamic(draw_index, tempsens.val(location), temp_lims_f[tempsens.errclass(location)][DISP_MIN], temp_lims_f[tempsens.errclass(location)][DISP_MAX]);
    }
    void update_idiots(bool force = false) {
        for (int i = 0; i < idiots->iconcount; i++) {
            if (i <= neo->neopixelsAvailable())
                if (idiots->val(i) != idiots->last[i]) neo->setBoolState(i, idiots->val(i));
            if (i == LOST || i == RANGE) {
                if (diag.most_critical_last[i] != diag.most_critical_sensor[i]) {
                    if (diag.most_critical_sensor[i] == _None) neo->setflash((int)i, 0);
                    else neo->setflash((int)i, diag.most_critical_sensor[i] + 1, 2, 6, 1, 0);
                }
                diag.most_critical_last[i] = diag.most_critical_sensor[i];
            }
        }
        if (display_enabled) draw_idiotlights(idiots_corner_x, idiots_corner_y, force);
    }
  public:
    void update_flexpanel() {
        #ifndef VIDEO_TASKS
        if (!is_pushing && !is_drawing) {  // && (sim->enabled() || screensaver)
            if (!pushtime) draw_task();
            else if (screenRefreshTimer.expired() || screensaver_max_refresh || fullscreen_screensaver_test) { // taskYIELD(); 
                screenRefreshTimer.reset();
                push_task();
                pushclock = (int32_t)screenRefreshTimer.elapsed();
            }
        }
        #endif
    }
    void update(int _nowmode) {
        if (fullscreen_screensaver_test) {
            update_flexpanel();
            return;
        }
        tiny_text();
        _tft.startWrite();
        update_idiots(disp_idiots_dirty);
        _tft.endWrite();
        disp_idiots_dirty = false;
        if (!display_enabled) return;
        _tft.startWrite();
        if (disp_datapage_dirty) {
            static bool first = true;
            draw_datapage(datapage, datapage_last, first);
            first = false;
            disp_datapage_dirty = false;
            if (datapage_last != datapage) prefs.putUInt("dpage", datapage);
        }
        if (disp_sidemenu_dirty) {
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
        if (valuesRefreshTimer.expireset()) {
            float drange;
            draw_dynamic(1, hotrc.pc[VERT][FILT], hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
            draw_dynamic(2, speedo.filt(), 0.0, speedo.redline_mph(), gas.cruisepid.target());
            draw_dynamic(3, tach.filt(), 0.0, tach.redline_rpm(), gas.pid.target());
            draw_dynamic(4, gas.pc[OUT], gas.pc[OPMIN], gas.pc[OPMAX]);
            draw_dynamic(5, pressure.filt(), pressure.min_human(), pressure.max_human(), brake.pids[PRESPID].target());  // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.targ() : pressure_target_adc);
            draw_dynamic(6, brake.pc[OUT], brake.pc[OPMIN], brake.pc[OPMAX]);
            draw_dynamic(7, hotrc.pc[HORZ][FILT], hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
            draw_dynamic(8, steer.pc[OUT], steer.pc[OPMIN], steer.pc[OPMAX]);
            if (datapage == PG_RUN) {
                draw_dynamic(9, brkpos.filt(), brkpos.op_min_in(), brkpos.op_max_in());
                draw_dynamic(10, mulebatt.filt(), mulebatt.op_min_v(), mulebatt.op_max_v());
                draw_dynamic(11, pot.val(), pot.min(), pot.max());
                draw_dynamic(12, airvelo.human(), airvelo.min_mph(), airvelo.max_mph());
                draw_dynamic(13, mapsens.human(), mapsens.min_atm(), mapsens.max_atm());
                draw_dynamic(14, maf_gps, maf_min_gps, maf_max_gps);
                for (int line=15; line<=17; line++) draw_eraseval(line);
                draw_dynamic(18, gas.governor, 0.0, 100.0);
                draw_dynamic(19, steer.steer_safe_pc, 0.0, 100.0);
            }
            else if (datapage == PG_JOY) {
                draw_dynamic(9, hotrc.us[HORZ][RAW], hotrc.us[HORZ][OPMIN], hotrc.us[HORZ][OPMAX]);
                draw_dynamic(10, hotrc.us[VERT][RAW], hotrc.us[VERT][OPMIN], hotrc.us[VERT][OPMAX]);
                draw_dynamic(11, hotrc.us[CH3][RAW], hotrc.us[CH3][OPMIN], hotrc.us[CH3][OPMAX]);
                draw_dynamic(12, hotrc.us[CH4][RAW], hotrc.us[CH4][OPMIN], hotrc.us[CH4][OPMAX]);
                draw_dynamic(13, hotrc.pc[HORZ][RAW], hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
                draw_dynamic(14, hotrc.pc[VERT][RAW], hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
                for (int line=15; line<=17; line++) draw_eraseval(line);
                draw_dynamic(18, hotrc.failsafe_us, hotrc.absmin_us, hotrc.us[VERT][OPMIN] - hotrc.us[VERT][MARGIN]);
                draw_dynamic(19, hotrc.deadband_us, 0, 100);
            }
            else if (datapage == PG_SENS) {
                draw_dynamic(9, pressure.raw(), pressure.min_native(), pressure.max_native());                    
                draw_dynamic(10, brkpos.raw(), brkpos.min_native(), brkpos.max_native());                    
                for (int line=11; line<=13; line++) draw_eraseval(line);
                draw_dynamic(14, airvelo.max_mph(), 0.0, airvelo.abs_max_mph());
                draw_dynamic(15, mapsens.min_atm(), mapsens.abs_min_atm(), mapsens.abs_max_atm());
                draw_dynamic(16, mapsens.max_atm(), mapsens.abs_min_atm(), mapsens.abs_max_atm());
                draw_dynamic(17, speedo.idle_mph(), 0.0, speedo.redline_mph());
                draw_dynamic(18, speedo.redline_mph(), 0.0, speedo.max_human());
                draw_dynamic(19, brkpos.zeropoint(), brkpos.min_human(), brkpos.max_human());  // BrakePositionSensor::abs_min_retract_in, BrakePositionSensor::abs_max_extend_in);
            }
            else if (datapage == PG_PWMS) {
                draw_dynamic(9, gas.deg[OUT], gas.deg[OPMIN], gas.deg[OPMAX]);
                draw_dynamic(10, gas.us[OUT], gas.us[ABSMIN], gas.us[ABSMAX]);
                draw_dynamic(11, brake.volt[OUT], brake.volt[OPMIN], brake.volt[OPMAX]);
                draw_dynamic(12, brake.us[OUT], brake.us[ABSMIN], brake.us[ABSMAX]);
                draw_dynamic(13, steer.volt[OUT], steer.volt[OPMIN], steer.volt[OPMAX]);
                draw_dynamic(14, steer.us[OUT], steer.us[ABSMIN], steer.us[ABSMAX]);
                draw_eraseval(15);
                draw_dynamic(16, gas.deg[OPMIN], gas.deg[ABSMAX], gas.deg[ABSMAX]);
                draw_dynamic(17, gas.deg[OPMAX], gas.deg[ABSMAX], gas.deg[ABSMAX]);
                draw_dynamic(18, brake.us[STOP], brake.us[ABSMIN], brake.us[ABSMAX]);
                draw_dynamic(19, brake.duty_fwd_pc, 0.0, 100.0);
            }
            else if (datapage == PG_IDLE) {
                draw_asciiname(9, idlestatecard[gas.idlectrl.targetstate]);
                draw_dynamic(10, gas.pid.target(), 0.0, tach.redline_rpm());
                draw_dynamic(11, gas.idlectrl.stallpoint, gas.idlectrl.idle_absmin, gas.idlectrl.idle_absmax);
                draw_dynamic(12, gas.idlectrl.idle_rpm, gas.idlectrl.idle_absmin, gas.idlectrl.idle_absmax);  // gas.idlectrl.idlehot(), gas.idlectrl.idlecold());
                draw_dynamic(13, gas.idlectrl.idlehigh, gas.idlectrl.idle_absmin, gas.idlectrl.idle_absmax);
                draw_dynamic(14, gas.idlectrl.idlecold, gas.idlectrl.idle_absmin, gas.idlectrl.idle_absmax, -1, 4);
                draw_dynamic(15, gas.idlectrl.idlehot, gas.idlectrl.idle_absmin, gas.idlectrl.idle_absmax, -1, 4);
                draw_dynamic(16, gas.idlectrl.tempcold, temp_lims_f[ENGINE][DISP_MIN], temp_lims_f[ENGINE][DISP_MAX]);
                draw_dynamic(17, gas.idlectrl.temphot, temp_lims_f[ENGINE][DISP_MIN], temp_lims_f[ENGINE][DISP_MAX]);
                draw_dynamic(18, (int32_t)gas.idlectrl.settlerate_rpmps, 0, 500);
                draw_asciiname(19, idlemodecard[(int32_t)gas.idlectrl.idlemode]);
            }
            else if (datapage == PG_BPID) {
                drange = brake.us[ABSMIN]-brake.us[ABSMAX];
                draw_dynamic(9, brake.pid_dom->target(), 0.0, 100.0);  // brake.pid_dom->outmin(), brake.pid_dom->outmax());
                draw_dynamic(10, brake.pid_dom->err(), brake.pid_dom->outmin(), brake.pid_dom->outmax());
                draw_dynamic(11, brake.pid_dom->pterm(), -drange, drange);
                draw_dynamic(12, brake.pid_dom->iterm(), -drange, drange);
                draw_dynamic(13, brake.pid_dom->dterm(), -drange, drange);
                draw_dynamic(14, brkpos.filt(), brkpos.op_min_in(), brkpos.op_max_in(), brake.pids[POSNPID].target());
                draw_dynamic(15, brake.hybrid_out_ratio_pc, 0.0, 100.0);  // brake_spid_speedo_delta_adc, -range, range);
                draw_dynamic(16, brake.duty_continuous, 0.0, 100.0);  // brake_spid_speedo_delta_adc, -range, range);
                // draw_eraseval(16);
                draw_dynamic(17, brake.pid_dom->kp(), 0.0, 8.0);
                draw_dynamic(18, brake.pid_dom->ki(), 0.0, 8.0);
                draw_dynamic(19, brake.pid_dom->kd(), 0.0, 8.0);
            }
            else if (datapage == PG_GPID) {
                draw_dynamic(9, gas.pid.target(), 0.0, tach.redline_rpm());
                draw_dynamic(10, gas.pid.err(), gas.idlectrl.idle_rpm - tach.govern_rpm(), tach.govern_rpm() - gas.idlectrl.idle_rpm);
                draw_dynamic(11, gas.pid.pterm(), -100.0, 100.0);
                draw_dynamic(12, gas.pid.iterm(), -100.0, 100.0);
                draw_dynamic(13, gas.pid.dterm(), -100.0, 100.0);
                draw_dynamic(14, gas.pid.outsum(), -gas.pid.outrange(), gas.pid.outrange());
                draw_eraseval(15);
                draw_truth(16, gas.openloop, 1);
                draw_dynamic(17, gas.pid.kp(), 0.0, 1.0);
                draw_dynamic(18, gas.pid.ki(), 0.0, 1.0);
                draw_dynamic(19, gas.pid.kd(), 0.0, 1.0);
            }
            else if (datapage == PG_CPID) {
                drange = tach.govern_rpm() - gas.idlectrl.idle_rpm;
                draw_dynamic(9, gas.cruisepid.target(), 0.0, speedo.govern_mph());
                draw_dynamic(10, gas.cruisepid.err(), speedo.idle_mph()-speedo.govern_mph(), speedo.govern_mph()-speedo.idle_mph());
                draw_dynamic(11, gas.cruisepid.pterm(), -drange, drange);
                draw_dynamic(12, gas.cruisepid.iterm(), -drange, drange);
                draw_dynamic(13, gas.cruisepid.dterm(), -drange, drange);
                draw_dynamic(14, gas.cruisepid.outsum(), -gas.cruisepid.outrange(), gas.cruisepid.outrange());  // cruise_spid_speedo_delta_adc, -drange, drange);
                draw_dynamic(15, gas.cruise_target_pc, 0.0, 100.0);
                draw_dynamic(16, cruise_delta_max_pc_per_s, 1, 35);
                draw_dynamic(17, gas.cruisepid.kp(), 0.0, 10.0);
                draw_dynamic(18, gas.cruisepid.ki(), 0.0, 10.0);
                draw_dynamic(19, gas.cruisepid.kd(), 0.0, 10.0);
            }
            else if (datapage == PG_TEMP) {
                draw_temperature(loc::AMBIENT, 9);
                draw_temperature(loc::ENGINE, 10);
                draw_temperature(loc::WHEEL_FL, 11);
                draw_temperature(loc::WHEEL_FR, 12);
                draw_temperature(loc::WHEEL_RL, 13);
                draw_temperature(loc::WHEEL_RR, 14);
                draw_dynamic(15, touch->touch_pt(0), 0, disp_width_pix);
                draw_dynamic(16, touch->touch_pt(1), 0, disp_height_pix);
                draw_eraseval(17);
                draw_truth(18, !web_disabled, 0);  // note this value is inverse to how it's displayed, same for the tuner entry
                draw_truth(19, dont_take_temperatures, 2);
            }
            else if (datapage == PG_SIM) {
                draw_truth(9, sim->can_sim(sens::joy), 0);
                draw_truth(10, sim->can_sim(sens::pressure), 0);
                draw_truth(11, sim->can_sim(sens::brkpos), 0);
                draw_truth(12, sim->can_sim(sens::speedo), 0);
                draw_truth(13, sim->can_sim(sens::tach), 0);
                draw_truth(14, sim->can_sim(sens::airvelo), 0);
                draw_truth(15, sim->can_sim(sens::mapsens), 0);
                draw_truth(16, sim->can_sim(sens::basicsw), 0);                    
                draw_asciiname(17, sensorcard[sim->potmap()]);
                draw_truth(18, cal_brakemode, 0);
                draw_truth(19, cal_gasmode, 0);
            }
            else if (datapage == PG_UI) {
                draw_dynamic(9, (int32_t)loop_avg_us, looptimer.loop_scale_min_us, looptimer.loop_scale_avg_max_us);
                draw_dynamic(10, looptimer.loop_peak_us, looptimer.loop_scale_min_us, looptimer.loop_scale_peak_max_us);
                draw_dynamic(11, (int32_t)looptimer.loopfreq_hz, 0.0, 2000.0);
                draw_dynamic(12, fps, 0.0, 600.0);
                draw_dynamic(13, drawclock, 0, refresh_limit);
                draw_dynamic(14, pushclock, 0, refresh_limit);
                draw_dynamic(15, idleclock, 0, refresh_limit);
                draw_truth(16, flashdemo, 0);
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
        _tft.endWrite();
        if (!_procrastinate) update_flexpanel();
        _procrastinate = false;
    }
};
class Tuner {
  private:
    NeopixelStrip* neo;
    Touchscreen* touch;
    Timer tuningCtrlTimer = Timer(25000000);  // This times out edit mode after a a long period of inactivity
  public:
    // Tuner(NeopixelStrip* _neo, Touchscreen* _touch) : neo(_neo), touch(_touch) {}
    Tuner(NeopixelStrip* _neo, Touchscreen* _touch) {
        neo = _neo;
        touch = _touch;
    }
    int32_t idelta = 0, idelta_encoder = 0;
    void update(int rmode) {
        process_inputs();
        edit_values(rmode);
    }
  private:
    void process_inputs() {
        sel_val_last = sel_val;
        datapage_last = datapage;
        tunctrl_last = tunctrl; // Make sure this goes after the last comparison
        uint32_t encoder_sw_action = encoder.button.press_event();  // true = autoreset the event if there is one
        if (encoder_sw_action != swNONE) {  // First deal with any unhandled switch press events
            if (encoder_sw_action == swSHORT)  {  // if short press
                if (tunctrl == EDIT) tunctrl = SELECT;  // If we were editing a value drop back to select mode
                else if (tunctrl == SELECT) tunctrl = EDIT;  // If we were selecting a variable start editing its value
                else if (button_test_heartbeat_color) heartbeat_override_color = random(0x10000);  // temporary!! to test heartbeat color override feature
            }
            else tunctrl = (tunctrl == OFF) ? SELECT : OFF;  // Long press starts/stops tuning
        }
        if (tunctrl == EDIT) idelta_encoder = encoder.rotation(true);  // true = include acceleration
        else if (tunctrl == SELECT) sel_val += encoder.rotation();  // If overflow constrain will fix in general handler below
        else if (tunctrl == OFF) datapage += encoder.rotation();  // If overflow tconstrain will fix in general below
        if (touch_increment_datapage) ++datapage %= NUM_DATAPAGES;
        touch_increment_datapage = false;
        idelta += idelta_encoder + touch->idelta;  // Allow edits using the encoder or touchscreen
        touch->idelta = idelta_encoder = 0;
        if (tunctrl != tunctrl_last || datapage != datapage_last || sel_val != sel_val_last || idelta) tuningCtrlTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
        else if (tuningCtrlTimer.expired()) tunctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
        datapage = constrain(datapage, 0, datapages::NUM_DATAPAGES-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
        if (datapage != datapage_last) {
            if (tunctrl == EDIT) tunctrl = SELECT;  // If page is flipped during edit, drop back to select mode
            disp_datapage_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
        }
        if (tunctrl == SELECT) {
            sel_val = constrain(sel_val, tuning_first_editable_line[datapage], disp_tuning_lines-1);  // Skip unchangeable values for all PID modes
            if (sel_val != sel_val_last) disp_selected_val_dirty = true;
        }
        if (tunctrl != tunctrl_last || disp_datapage_dirty) disp_selected_val_dirty = true;
    }
    void edit_values(int rmode) {
        float fdelta = (float)idelta;
        if (tunctrl == EDIT && idelta) {  // Change tunable values when editing
            if (datapage == PG_RUN) {
                if (sel_val == 9) { adj_val(&(gas.governor), idelta, 0, 100); gas.derive(); }
                else if (sel_val == 10) adj_val(&(steer.steer_safe_pc), idelta, 0, 100);
            }
            else if (datapage == PG_JOY) {
                if (sel_val == 9) adj_val(&hotrc.failsafe_us, idelta, hotrc.absmin_us, hotrc.us[VERT][OPMIN] - hotrc.us[VERT][MARGIN]);
                else if (sel_val == 10) { adj_val(&hotrc.deadband_us, idelta, 0, 50); hotrc.calc_params(); }
            }
            else if (datapage == PG_SENS) {
                if (sel_val == 5) adj_val(airvelo.max_mph_ptr(), fdelta, 0, airvelo.abs_max_mph());
                else if (sel_val == 6) adj_val(mapsens.min_atm_ptr(), fdelta, mapsens.abs_min_atm(), mapsens.abs_max_atm());
                else if (sel_val == 6) adj_val(mapsens.max_atm_ptr(), fdelta, mapsens.abs_min_atm(), mapsens.abs_max_atm());
                else if (sel_val == 8) adj_val(speedo.idle_mph_ptr(), fdelta, 0, speedo.redline_mph() - 1);
                else if (sel_val == 9) adj_val(speedo.redline_mph_ptr(), fdelta, speedo.idle_mph(), 20);
                else if (sel_val == 10) adj_val(brkpos.zeropoint_ptr(), fdelta, brkpos.op_min_in(), brkpos.op_max_in());
            }
            else if (datapage == PG_PWMS) {
                if (sel_val == 7) { adj_val(&(gas.si[OPMIN]), fdelta, gas.si[PARKED] + 1, gas.si[OPMAX] - 1); gas.derive(); }
                else if (sel_val == 8) { adj_val(&(gas.si[OPMAX]), fdelta, gas.si[OPMIN] + 1, 180.0); gas.derive(); }
                else if (sel_val == 9) { adj_val(&(brake.us[STOP]), fdelta, brake.us[OPMIN] + 1, brake.us[OPMAX] - 1); brake.derive(); }
                else if (sel_val == 10) { adj_val(&(brake.duty_fwd_pc), fdelta, 0.0, 100.0); brake.derive(); }
            }
            else if (datapage == PG_IDLE) {
                if (sel_val == 4) gas.idlectrl.add_idlehigh(fdelta);
                else if (sel_val == 5) gas.idlectrl.add_idlecold(fdelta);
                else if (sel_val == 6) gas.idlectrl.add_idlehot(fdelta);
                else if (sel_val == 7) gas.idlectrl.add_tempcold(fdelta);
                else if (sel_val == 8) gas.idlectrl.add_temphot(fdelta);
                else if (sel_val == 9) gas.idlectrl.add_settlerate(idelta);
                else if (sel_val == 10) gas.idlectrl.cycle_idlemode(idelta);
            }
            else if (datapage == PG_BPID) {
                if (sel_val == 8) brake.pid_dom->add_kp(0.001 * fdelta);
                else if (sel_val == 9) brake.pid_dom->add_ki(0.001 * fdelta);
                else if (sel_val == 10) brake.pid_dom->add_kd(0.001 * fdelta);
            }
            else if (datapage == PG_GPID) {
                if (sel_val == 7) { adj_bool(&(gas.openloop), idelta); }  // gas_pid.SetMode(gas_open_loop ? QPID::ctrl::manual : QPID::ctrl::automatic);
                else if (sel_val == 8) gas.pid.add_kp(0.001 * fdelta);
                else if (sel_val == 9) gas.pid.add_ki(0.001 * fdelta);
                else if (sel_val == 10) gas.pid.add_kd(0.001 * fdelta);
            }
            else if (datapage == PG_CPID) {
                if (sel_val == 7) adj_val(&cruise_delta_max_pc_per_s, idelta, 1, 35);
                else if (sel_val == 8) gas.cruisepid.add_kp(0.001 * fdelta);
                else if (sel_val == 9) gas.cruisepid.add_ki(0.001 * fdelta);
                else if (sel_val == 10) gas.cruisepid.add_kd(0.001 * fdelta);
            }
            else if (datapage == PG_TEMP) {
                if (sel_val == 9) { adj_bool(&web_disabled, -1 * idelta); }  // note this value is inverse to how it's displayed, same for the value display entry
                else if (sel_val == 10) adj_bool(&dont_take_temperatures, idelta);
            }
            else if (datapage == PG_SIM) {
                if (sel_val == 0) sim.set_can_sim(sens::joy, idelta);
                else if (sel_val == 1) sim.set_can_sim(sens::pressure, idelta);
                else if (sel_val == 2) sim.set_can_sim(sens::brkpos, idelta);
                else if (sel_val == 3) sim.set_can_sim(sens::speedo, idelta);
                else if (sel_val == 4) sim.set_can_sim(sens::tach, idelta);
                else if (sel_val == 5) sim.set_can_sim(sens::airvelo, idelta);
                else if (sel_val == 6) sim.set_can_sim(sens::mapsens, idelta);  // else if (sel_val == 7) sim.set_can_sim(sens::starter, idelta);
                else if (sel_val == 7) sim.set_can_sim(sens::basicsw, idelta);
                else if (sel_val == 8) { sim.set_potmap((adj_val(sim.potmap(), idelta, 0, (int)(sens::starter) - 1))); prefs.putUInt("potmap", sim.potmap()); }
                else if (sel_val == 9 && rmode == CAL) adj_bool(&(cal_brakemode), idelta);
                else if (sel_val == 10 && rmode == CAL) adj_bool(&(cal_gasmode_request), idelta);
            }
            else if (datapage == PG_UI) {
                if (sel_val == 7) { adj_bool(&flashdemo, idelta); neo->enable_flashdemo(flashdemo); }
                else if (sel_val == 8) { adj_val(&neobright, idelta, 1, 100); neo->setbright(neobright); }
                else if (sel_val == 9) { adj_val(&neodesat, idelta, 0, 10); neo->setdesaturation(neodesat); }
                else if (sel_val == 10) adj_bool(&screensaver, idelta);
            }
            idelta = 0;
        }
    }
};
// The following project draws a nice looking gauge cluster, very apropos to our needs and the code is given.
// See this video: https://www.youtube.com/watch?v=U4jOFLFNZBI&ab_channel=VolosProjects
// Rinkydink home page: http://www.rinkydinkelectronics.com
// moving transparent arrow sprite over background: https://www.youtube.com/watch?v=U4jOFLFNZBI&ab_channel=VolosProjects
// bar graphs: https://www.youtube.com/watch?v=g4jlj_T-nRw&ab_channel=VolosProjects