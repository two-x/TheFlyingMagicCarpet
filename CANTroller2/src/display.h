#pragma once

#define disp_runmode_text_x 12
#define disp_lines 24  // Max lines of text displayable at line height = disp_line_height_pix
#define disp_fixed_lines 8  // Lines of static variables/values always displayed
#define disp_line_height_pix 10  // Pixel height of each text line. Screen can fit 16x 15-pixel or 20x 12-pixel lines
#define disp_bargraph_width 38
#define disp_maxlength 5  // How many characters is max data value
#define disp_default_float_sig_dig 3  // Significant digits displayed for float values. Higher causes more screen draws
#define disp_datapage_names_x 12
#define disp_datapage_values_x 59
#define disp_datapage_units_x 96  // 103        
#define disp_bargraphs_x 111  // 122
#define disp_datapage_title_x 83
#define disp_value_dimsteps 2  // or 3 for multiple levels of dimness
std::string side_menu_buttons[5] = { "PAG", "SEL", "+  ", "-  ", "ANI" };  // Pad shorter names with spaces on the right
std::string top_menu_buttons[4]  = { "CAL", "SIM", "CH4", "IGN" };
std::string ch4_menu_buttons[NUM_RUNMODES] = { "CH4", "WAKE", "SLEEP", "START", "START", "CRUIS", "FLY", "CH4" }; // Basic, LowPwr, Stndby, Stall, Hold, Fly, Cruise, Cal
std::string sensorcard[14] = { "none", "joy", "bkpres", "brkpos", "speedo", "tach", "airflw", "mapsns", "engtmp", "batery", "startr", "basic", "ign", "syspwr" };
std::string uicontextcard[NumContextsUI] = { "ezread", "chasis", "animat" };
#define stEr "St\x88r"     // These defines are just a convenience to keep the below datapage strings
#define brAk "Br\x83k"     //   array initializations aligned in neat rows & cols for legibility
#define spEd "Sp\x88""d"
#define b1nary "  \xa7"
#define scroll "\x12"
#define degree "\xf7"
#define degreF "\xf7""F"
#define degsec "\xf7/s"
#define ______ ""
#define __________ "      \xf9"
#define neo_bright "NeoBr\x8dgt"
#define maxadjrate "MaxAjR\x83t"
#define horfailsaf "HFails\x83""f"
static std::string telemetry[disp_fixed_lines] = { "Hot Vert", "Hot Horz", "   Speed", "    Tach", brAk"Sens", "Throttle", brAk"Motr", stEr"Motr" };  // Fixed rows
static std::string units[disp_fixed_lines] = { "%", "%", "mph", "rpm", "%", "%", "%", "%" };  // Fixed rows
static std::string pagecard[datapages::NUM_DATAPAGES] = { "Run ", "Joy ", "Sens", "Puls", "PWMs", "Idle", "Motr", "Bpid", "Gpid", "Cpid", "Temp", "Sim ", "UI  " };
static constexpr int tuning_first_editable_line[datapages::NUM_DATAPAGES] = { 13, 10, 10, 10, 11, 11, 5, 11, 11, 11, 12, 4, 8 };  // first value in each dataset page that's editable. All values after this must also be editable
static std::string datapage_names[datapages::NUM_DATAPAGES][disp_tuning_lines] = {
    { brAk"Pres", brAk"Posn", "MuleBatt", "     Pot", " AirVelo", "     MAP", "MasAirFl", "Gas Mode", brAk"Mode", stEr"Mode", "  Uptime", __________, __________, "Governor", stEr"Safe", },  // PG_RUN
    { "FiltHorz", "FiltVert", "Raw Horz", "Raw Vert", " Raw Ch3", " Raw Ch4", "Raw Horz", "Raw Vert", __________, __________, "AirVOMax", "MAP OMin", "MAP OMax", horfailsaf, "Deadband", },  // PG_JOY
    { " Pot Raw", brAk"Posn", brAk"Posn", brAk"Posn", "Pressure", "Pressure", "Pressure", "MuleBatt", "MuleBatt", __________, "PresOmin", "PresOmax", "BPosOmin", "BPosOmax", "BPosZero", },  // PG_SENS
    { "TachPuls", "Tach Raw", "Tach Raw", spEd"Puls", "SpeedRaw", "SpeedRaw", "   Speed", __________, __________, __________, "TachOMin", "TachOMax", spEd"OMin", spEd"OMax", spEd"Idle", },  // PG_PULS
    { "Throttle", "Throttle", brAk"Motr", brAk"Motr", stEr"Motr", stEr"Motr", __________, __________, __________, __________, __________, "ThrotCls", "ThrotOpn", brAk"Stop", brAk"Duty", },  // PG_PWMS
    { "Gas Mode", "Tach Tgt", "IdlBoost", "    Idle", "    Idle", "    Idle", "FuelPump", __________, __________, __________, __________, "StartGas", "MaxBoost", "ColdTemp", "Hot Temp", },  // PG_IDLE
    { "Brk Heat", "HybBrake", __________, __________, __________, "BkEnaPID", "BkFeedbk", "BOpnMode", "BkPosLim", "BkMaxChg", "GasEnPID", "CrEnaPID", "CrAdjMod", "CrusBrak", "DrivMode", },  // PG_MOTR    
    { "MotrMode", "Pressure", "Pres Tgt", "Position", "Posn Tgt", "Hyb Targ", "OutRatio", "  P Term", "Integral", "  I Term", "  D Term", "SamplTim", "Brake Kp", "Brake Ki", "Brake Kd", },  // PG_BPID
    { "MotrMode", "AngleTgt", "TachTarg", "Tach Err", "  P Term", "  I Term", "  D Term", __________, __________, __________, __________, "AnglVelo", "  Gas Kp", "  Gas Ki", "  Gas Kd", },  // PG_GPID
    { spEd"Targ", "SpeedErr", "  P Term", "  I Term", "  D Term", "ThrotSet", __________, __________, __________, __________, __________, maxadjrate, "Cruis Kp", "Cruis Ki", "Cruis Kd", },  // PG_CPID
    { " Ambient", "  Engine", "Wheel FL", "Wheel FR", "Wheel RL", "Wheel RR", "BrkMotor", __________, __________, __________, __________, __________, "WhTmpDif", "No Temps", "StopWifi", },  // PG_TEMP
    { __________, __________, __________, __________, "Joystick", brAk"Pres", brAk"Posn", "  Speedo", "    Tach", "AirSpeed", "     MAP", "Basic Sw", " Pot Map", "CalBrake", " Cal Gas", },  // PG_SIM
    { "Loop Avg", "LoopPeak", "FramRate", "HumanAct", " Touch X", " Touch Y", "SpinRate", "   Accel", "EZScroll", "PcbaGlow", "BlnkDemo", "NiteRidr", neo_bright, "NeoSatur", "PanelApp", },  // PG_UI
};
static std::string tuneunits[datapages::NUM_DATAPAGES][disp_tuning_lines] = {
    { "psi",  "in",   "V",    "%",    "mph",  "atm",  "g/s",  scroll, scroll, scroll, "min",  ______, ______, "%",    "%",    },  // PG_RUN
    { "us",   "us",   "us",   "us",   "us",   "us",   "%",    "%",    ______, ______, "mph",  "atm",  "atm",  "us",   "us",   },  // PG_JOY
    { "adc",  "adc",  "in",   "%",    "adc",  "psi",  "%",    "adc",  "V",    ______, "psi",  "psi",  "in",   "in",   "in",   },  // PG_SENS
    { "ms",   "Hz",   "rpm",  "ms",   "Hz",   "mph",  "%",    ______, ______, ______, "rpm",  "rpm",  "mph",  "mph",  "mph",  },  // PG_PULS
    { degree, "us",   "V",    "us",   "V",    "us",   ______, ______, ______, ______, ______, degree, degree, "us",   "%",    },  // PG_PWMS
    { scroll, "rpm",  "%",    "%",    degree, "rpm",  "V",    ______, ______, ______, ______, "%",    "%",    degreF, degreF, },  // PG_IDLE
    { degreF, "%",    ______, ______, ______, b1nary, scroll, scroll, b1nary, "%/s",  b1nary, b1nary, scroll, b1nary, scroll, },  // PG_MOTR
    { scroll, "%",    "psi",  "%",    "in",   "%",    "%",    "%",    "%",    "%",    "%",    "us",   ______, "Hz",   "s",    },  // PG_BPID
    { scroll, "%",    "rpm",  "rpm",  "%",    "%",    "%",    ______, ______, ______, ______, degsec, ______, "Hz",   "s",    },  // PG_GPID
    { "mph",  "mph",  "rpm",  "rpm",  "rpm",  "%",    ______, ______, ______, ______, ______, "%/s",  ______, "Hz",   "s",    },  // PG_CPID
    { degreF, degreF, degreF, degreF, degreF, degreF, degreF, ______, ______, ______, ______, ______, degreF, b1nary, b1nary, },  // PG_TEMP
    { ______, ______, ______, ______, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, scroll, b1nary, b1nary, },  // PG_SIM
    { "us",   "us",   "fps",  scroll, "pix",  "pix",  "Hz",   "x",    "lin",  scroll, b1nary, "eyes", "%",    "%",    scroll, },  // PG_UI
};
static std::string unitmapnames[20] = { "us", scroll, b1nary, "%", "ohm", "eyes", degree, degreF, "mph", "rpm", "psi", "atm", "g/s", "adc", "pix", "min", "%/s", degsec, "fps", "lin" };  // unit strings matching these will get replaced by the corresponding bitmap graphic below
static constexpr uint8_t unitmaps[20][13] = {  // now 13x7-pixel bitmaps for unit strings. required when string is over 2 characters
    { 0x40, 0x7e, 0x20, 0x20, 0x1c, 0x20, 0x00, 0x24, 0x2a, 0x2a, 0x2a, 0x12, 0x00, },  // us - b/c the font's lowercase mu character doesn't work
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x7f, 0x00, 0x7f, 0x02, 0x04, },  // up/down arrows to indicate multiple choices. this one is right-aligned one char over to allow longer names
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x75, 0x75, 0x22, 0x00, },  // binary vertical on/off dots
    { 0x02, 0x45, 0x25, 0x12, 0x08, 0x24, 0x52, 0x51, 0x20, 0x00, 0x00, 0x00, 0x00, },  // % - we use this a lot and the font % looks feeble
    { 0x4e, 0x51, 0x61, 0x01, 0x61, 0x51, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // capital omega - for ohms
    { 0x00, 0x3e, 0x63, 0x63, 0x77, 0x7f, 0x41, 0x3e, 0x63, 0x63, 0x77, 0x7f, 0x3e, },  // googly eyes, are as goofy as they are stupid
    { 0x06, 0x09, 0x49, 0x66, 0x50, 0x48, 0x4c, 0x72, 0x41, 0x40, 0x40, 0x00, 0x00, },  // angular degrees
    { 0x06, 0x09, 0x09, 0x06, 0x00, 0x7f, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00, 0x00, },  // degrees F
    { 0x3e, 0x02, 0x3c, 0x02, 0x3c, 0x00, 0x7e, 0x12, 0x0c, 0x00, 0x3f, 0x04, 0x38, },  // mph
    { 0x3e, 0x02, 0x04, 0x00, 0x7e, 0x12, 0x0c, 0x00, 0x3e, 0x02, 0x3c, 0x02, 0x3c, },  // rpm
    { 0x7e, 0x12, 0x12, 0x0c, 0x00, 0x2c, 0x2a, 0x2a, 0x12, 0x00, 0x24, 0x3d, 0x20, },  // psi
    { 0x10, 0x2a, 0x2a, 0x3c, 0x20, 0x1f, 0x22, 0x00, 0x3e, 0x02, 0x3c, 0x02, 0x3c, },  // atm
    { 0x4c, 0x52, 0x52, 0x3e, 0x00, 0x30, 0x0c, 0x03, 0x00, 0x2c, 0x2a, 0x2a, 0x1a, },  // g/s
    { 0x10, 0x2a, 0x2a, 0x3c, 0x20, 0x1c, 0x24, 0x24, 0x3f, 0x00, 0x1c, 0x22, 0x22, },  // adc
    { 0x7e, 0x12, 0x12, 0x0c, 0x00, 0x04, 0x3d, 0x00, 0x22, 0x14, 0x08, 0x14, 0x22, },  // pix
    { 0x7c, 0x04, 0x78, 0x04, 0x78, 0x00, 0x44, 0x7d, 0x40, 0x00, 0x7c, 0x04, 0x78, },  // min
    { 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x60, 0x18, 0x06, 0x48, 0x54, 0x54, 0x24, },  // %/s
    { 0x06, 0x09, 0x09, 0x06, 0x60, 0x18, 0x06, 0x00, 0x48, 0x54, 0x54, 0x24, 0x00, },  // deg/s  // 0x06, 0x0f, 0x09, 0x0f, 0x06,
    { 0x08, 0x7e, 0x09, 0x02, 0x00, 0x7e, 0x12, 0x12, 0x0c, 0x00, 0x2c, 0x2a, 0x12, },  // fps
    { 0x41, 0x7f, 0x40, 0x00, 0x44, 0x7d, 0x40, 0x00, 0x7c, 0x04, 0x04, 0x78, 0x00, },  // lin
};  // These bitmaps are in the same format as the idiot light bitmaps, described in neopixel.h
    // { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x22, 0x41, 0x22, 0x14, },  // up/down arrows to indicate multiple choices. this one is right-aligned one char over to allow longer names
    // { 0x00, 0x00, 0x00, 0x00, 0x3e, 0x41, 0x45, 0x49, 0x51, 0x49, 0x45, 0x41, 0x3e, },  // dropdown arrow to indicate multiple choices. this one is right-aligned one char over to allow longer names
    // { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x7e, 0x3f, 0x00, 0x7e, 0x3f, 0x10, },  // thick-ass scroll arrows from the 80s
    // { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x7e, 0x07, 0x00, 0x70, 0x3f, 0x18, },  // scroll arrows - curvy and sorta disfigured
    // { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x66, 0x7f, 0x66, 0x60, 0x00, },  // 0/1 - to indicate binary value. right-aligned to maybe distinguish it from actual units

static EZReadDrawer ezdraw(&ezread);
static PanelAppManager panel(&ezdraw);
volatile float fps = 0.0;
// int pushclock, drawclock, idleclock;
volatile bool reset_request = false;

SemaphoreHandle_t pushbuf_sem;  // StaticSemaphore_t push_semaphorebuf_sem;
SemaphoreHandle_t drawbuf_sem;  // StaticSemaphore_t draw_semaphorebuf_sem;
static void push_task(void *parameter);
static void draw_task(void *parameter);
void semaphore_setup() {
    ezread.squintf("Semaphores..");
    pushbuf_sem = xSemaphoreCreateBinary();  // StaticSemaphore_t push_semaphorebuf_sem;
    drawbuf_sem = xSemaphoreCreateBinary();  // StaticSemaphore_t draw_semaphorebuf_sem;
    if (pushbuf_sem == NULL || drawbuf_sem == NULL) ezread.squintf(" creation failed");
    else {
        xSemaphoreGive(pushbuf_sem);
        xSemaphoreGive(drawbuf_sem);
    }
    ezread.squintf("\n");
}
LGFX_Sprite* sprptr;
std::string nulstr = "";
std::string* nulstrptr = &nulstr;

class Display {
  private:
    NeopixelStrip* neo;
    Touchscreen* touch;
    Simulator* sim;
    IdiotLights* idiots;
    Timer valuesRefreshTimer{160000};  // Don't refresh screen faster than this (16667us = 60fps, 33333us = 30fps, 66666us = 15fps)
    uint8_t palettesize = 2;
    // uint16_t palette[256] = { BLK, WHT };
    static constexpr int runOnCore = CONFIG_ARDUINO_RUNNING_CORE == 0 ? 1 : 0;
    Timer dispAgeTimer[disp_lines];  // int disp_age_timer_us[disp_lines];
    static constexpr int idiots_corner_x = disp_apppanel_x + 2;
    static constexpr int idiots_corner_y = 13;
    static constexpr int idiots_spacing_x = 1;
    bool sim_last = false, fullscreen_last = false;
    int runmode_last = -1;
  public:
    std::string disp_values[disp_lines];  // Holds previously drawn value strings for each line
    uint8_t disp_val_colors[disp_lines];
    bool disp_bool_values[6], disp_bargraphs[disp_lines], disp_polarities[disp_lines];  // Holds sign of previously drawn values
    int disp_datapage_last, disp_needles[disp_lines], disp_targets[disp_lines], disp_age_quanta[disp_lines];
    bool disp_selection_dirty, disp_datapage_dirty, disp_values_dirty, disp_data_dirty[disp_lines], disp_menutoggles_dirty;
    bool disp_menus_dirty, disp_runmode_dirty, disp_simbuttons_dirty,disp_idiots_dirty, disp_units_dirty;
    Display(NeopixelStrip* _neo, Touchscreen* _touch, IdiotLights* _idiots, Simulator* _sim)
      : neo(_neo), touch(_touch), idiots(_idiots), sim(_sim) {}
    void init_framebuffers(int _sprwidth, int _sprheight) {
        int sprsize[2] = { _sprwidth, _sprheight };
        ezread.squintf(" %dx buffers ", num_bufs);
        lcd.setColorDepth(sprite_color_depth);
        for (int i = 0; i < num_bufs; i++) framebuf[i].setColorDepth(8);  // color_depth_t::rgb332_1Byte = 8  Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
        auto framewidth = sprsize[HORZ];
        auto frameheight = sprsize[VERT];
        bool fail = false;
        bool using_psram = false;
        for (int i = 0; !fail && i < num_bufs; ++i) {
            framebuf[i].setPsram(false);
            fail = !framebuf[i].createSprite(framewidth, frameheight);
        }
        if (fail) {
            fail = false;
            for (int i = 0; !fail && i < num_bufs; ++i) {
                framebuf[i].setPsram(true);
                fail = !framebuf[i].createSprite(framewidth, frameheight);
            }
            if (fail) {
                fail = false;
                if (framewidth >= 320) framewidth = 180;
                if (frameheight >= 240) frameheight = 180;
                for (int i = 0; !fail && i < num_bufs; ++i) {
                    fail = !framebuf[i].createSprite(framewidth, frameheight);
                }
                if (fail) {
                    lcd.print("failed\n");
                }
                else using_psram = true;
            }
            else using_psram = true;
        }
        ezread.squintf("(%dx%d) in %sram\n", framewidth, frameheight, using_psram ? "ps" : "native ");
    }
    void init() {  // init() is necessary after any power interruption
        lcd.setColorDepth(8);
        lcd.begin();  // lcd.begin();
        lcd.initDMA();
        // lcd.setRotation((flip_the_screen) ? 3 : 1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
        if (lcd.width() < lcd.height()) lcd.setRotation(lcd.getRotation() ^ 1);
        lcd.setSwapBytes(true);  // rearranges color ordering of 16bit colors when displaying image files
        reset_request = true;
    }    
    void setup() {  // setup() only happens once at system boot
        if (!display_enabled) return;
        ezread.squintf("Display..");  //
        lcd.init();
        #ifdef BOARD_HAS_PSRAM
        // lcd.setAttribute(PSRAM_ENABLE, true);  // enable use of PSRAM - (this is only relevant for TFT_eSPI display library)
        #endif
        init();
        for (int lineno=0; lineno <= disp_fixed_lines; lineno++)  {
            disp_age_quanta[lineno] = -1;
            disp_values[lineno] = "";
            disp_polarities[lineno] = 1;
        }
        for (int row=0; row<arraysize(disp_bool_values); row++) disp_bool_values[row] = 1;
        for (int row=0; row<arraysize(disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
        for (int row=0; row<arraysize(disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen
        datapage = prefs.getUInt("dpage", PG_RUN);
        init_framebuffers(disp_width_pix, disp_height_pix);
        panel.setup(&lcd, sim, touch, disp_apppanel_x, disp_apppanel_y, disp_apppanel_w, disp_apppanel_h);
        sprptr = &framebuf[flip];
        // ezread.squintf("  display initialized\n");
    }
    void reset() {
        blackout();
        all_dirty();
        reset_request = false;
    }
    // uint8_t add_palette(uint8_t color) {
    //     for (uint8_t i=0; i<palettesize; i++) if (lcd.getPaletteColor(i) == color) return i;
    //     lcd.setPaletteColor(palettesize++, color);
    //     return palettesize;
    // }
    void all_dirty() {
        disp_idiots_dirty = true;
        for (int i=0; i<disp_lines; i++) {
            disp_data_dirty[i] = true;
            disp_bargraphs[i] = false;
        }
        disp_menutoggles_dirty = disp_selection_dirty = disp_datapage_dirty = disp_menus_dirty = true;
        disp_runmode_dirty = disp_simbuttons_dirty = disp_values_dirty = true;
        ui_context = ui_default;
    }
    void blackout() {
        sprptr->fillSprite(BLK);
        // std::uint32_t* s;
        // for (int f=0; f<2; f++) {
        //     s = (std::uint32_t*)(*spr).getBuffer();
        //     for (int w=0; w<(spr->width() * spr->height() / 4); w++) s[w] = 0x00000000;
        // }
    }
    void set_runmodecolors() {
        uint8_t saturat = 255;  uint8_t hue_offset = 0;
        for (int rm=0; rm<NUM_RUNMODES; rm++) {
            int division = NUM_RUNMODES;
            uint32_t color32 = hsv_to_rgb<uint32_t>((65536 * (uint16_t)(rm % division) / division + hue_offset), saturat, 255);  // , 0, 220);
            colorcard[rm] = color_to_332(color32);  // 5957 = 2^16/11
            disp_runmode_dirty = true;
        }
    }
    void tiny_text() {
        sprptr->setTextDatum(textdatum_t::top_left);
        sprptr->setFont(&fonts::Font0);
    }
    uint8_t darken_color(uint8_t color) {  // halves each of r, g, and b of a 5-6-5 formatted 16-bit color value either once or twice
        return ((color & 0xc0) | (color & 0x18) | (color & 0x2));
    }
    uint16_t darken_color(uint16_t color, int halvings = 1) {  // halves each of r, g, and b of a 5-6-5 formatted 16-bit color value either once or twice
        if (halvings == 1) return ((color & 0xf000) | (color & 0x7c0) | (color & 0x1e)) >> 1;
        else return ((color & 0xe000) | (color & 0x780) | (color & 0x1c)) >> 2;
    }
  private:
    void drawWideLine(int x0, int y0, int x1, int y1, float wd) {  // took from http://members.chello.at/~easyfilter/bresenham.html
        int dx = std::abs(x1-x0), sx = x0 < x1 ? 1 : -1; 
        int dy = std::abs(y1-y0), sy = y0 < y1 ? 1 : -1; 
        int err = dx-dy, e2, x2, y2;                          /* error value e_xy */
        float ed = dx+dy == 0 ? 1 : sqrt((float)dx*dx+(float)dy*dy);

        for (wd = (wd+1)/2; ; ) {                                   /* pixel loop */
            sprptr->drawPixel(x0, y0, std::max(0.0f, 255*(std::abs(err-dx+dy)/ed-wd+1)));
            e2 = err; x2 = x0;
            if (2*e2 >= -dx) {                                           /* x step */
                for (e2 += dy, y2 = y0; e2 < ed*wd && (y1 != y2 || dx > dy); e2 += dx)
                   sprptr->drawPixel(x0, y2 += sy, std::max(0.0f, 255*(abs(e2)/ed-wd+1)));
                if (x0 == x1) break;
                e2 = err; err -= dy; x0 += sx; 
            } 
            if (2*e2 <= dy) {                                            /* y step */
                for (e2 = dx-e2; e2 < ed*wd && (x1 != x2 || dx < dy); e2 += dy)
                    sprptr->drawPixel(x2 += sx, y0, std::max(0.0f, 255*(abs(e2)/ed-wd+1)));
                if (y0 == y1) break;
                err += dx; y0 += sy; 
            }
        }
    }
    void draw_bargraph_base(int corner_x, int corner_y, int width) {  // draws a horizontal bargraph scale.  124, y, 40
        sprptr->drawFastHLine(corner_x + 1, corner_y, width - 2, MGRY);  // base line
        sprptr->drawFastVLine(corner_x + width/2, corner_y-1, 2, WHT);  // centerpoint gradient line
        for (int offset=0; offset<=2; offset+=2) sprptr->drawFastVLine((corner_x + 1) + offset * (width/2 - 1), corner_y-2, 3, WHT);  // endpoint gradient lines
    }
    void draw_needle_shape(int pos_x, int pos_y, uint8_t color) {  // draws a cute little pointy needle
        sprptr->drawFastVLine(pos_x-1, pos_y, 2, color);
        sprptr->drawFastVLine(pos_x, pos_y, 4, color);
        sprptr->drawFastVLine(pos_x+1, pos_y, 2, color);
    }
    void draw_target_shape(int pos_x, int pos_y, uint8_t t_color, uint8_t r_color) {  // draws a cute little target symbol
        sprptr->drawFastHLine(pos_x-1, pos_y, 3, t_color);
        sprptr->drawFastVLine(pos_x, pos_y, 4, t_color);
    }
    void draw_bargraph_needle(int n_pos_x, int old_n_pos_x, int pos_y, uint8_t n_color) {  // draws a cute little pointy needle
        draw_needle_shape(old_n_pos_x, pos_y, BLK);
        draw_needle_shape(n_pos_x, pos_y, n_color);
    }
    void draw_string_core(int x, int y, std::string text, uint8_t color) {  // Send in "" for oldtext if erase isn't needed
        sprptr->setTextColor(color);
        sprptr->setCursor(x, y);
        sprptr->print(text.c_str());    
    }
    void draw_string(int x, int y, std::string text, std::string oldtext, uint8_t color, uint8_t bgcolor, bool forced=false) {  // Send in bgcolor=NON if erase isn't needed
        if ((text == oldtext) && !forced) return;
        if (bgcolor != NON) draw_string_core(x, y, oldtext, bgcolor);  // sprptr->fillRect(x_old, y, oldtext.length() * disp_font_width, disp_font_height, bgcolor);
        draw_string_core(x, y, text, color);
    }
    void draw_unitmap(int index, int x, int y, uint8_t color) {
        for (int xo = 0; xo < arraysize(unitmaps[0]); xo++)
            for (int yo = 0; yo < disp_font_height - 1; yo++)
                if ((unitmaps[index][xo] >> yo) & 1) sprptr->drawPixel(x + xo, y + yo, color);
    }
    void draw_erase_units(int x, int y, uint8_t bgcolor) {
        sprptr->fillRect(x, y, arraysize(unitmaps[0]), disp_font_height, bgcolor);
    }
    void draw_string_units(int x, int y, std::string text, std::string oldtext, uint8_t color, uint8_t bgcolor) {  // Send in bgcolor=NON if erase isn't needed
        if (bgcolor != NON) draw_erase_units(x, y, bgcolor);
        for (int i = 0; i<arraysize(unitmaps); i++) {
            if (unitmapnames[i] == text) {
                draw_unitmap(i, x, y, color);
                return;
            }
        }
        sprptr->setCursor(x, y);
        sprptr->setTextColor(color);  // text units strings must be 2 chars max
        sprptr->print(text.c_str());
    }
    // draw_fixed displays rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
    void draw_fixed(int page, int page_last, bool redraw_all=true, bool forced=false) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
        static int fixed_page_last;
        sprptr->setTextColor(LGRY);
        sprptr->setTextSize(1);
        int y_pos;
        for (int lineno = 0; lineno < disp_fixed_lines; lineno++) {  // step thru lines of fixed telemetry data
            y_pos = (lineno + 1) * disp_line_height_pix;
            draw_string(disp_datapage_names_x, y_pos, telemetry[lineno], nulstr, LGRY, BLK, forced);
            draw_string_units(disp_datapage_units_x, y_pos, units[lineno], nulstr, LGRY, BLK);
        }
        if (redraw_all) {
            for (int lineno = 0; lineno < disp_tuning_lines; lineno++) {  // step thru lines of fixed telemetry data
                y_pos = (lineno + disp_fixed_lines + 1) * disp_line_height_pix;
                // int index = lineno - disp_fixed_lines - 1;
                // ezread.squintf("drawing line:%d x:%d y:%d text:%s\n", index, disp_datapage_names_x, y_pos, datapage_names[page][index].c_str() );
                draw_string(disp_datapage_names_x, y_pos, datapage_names[page_last][lineno], datapage_names[page_last][lineno], BLK, NON, forced);  // erase old value
                draw_string_units(disp_datapage_units_x, y_pos, tuneunits[page_last][lineno], tuneunits[page_last][lineno], BLK, NON);  // erase unit string here in case new long value string overlaps old units string. draw new unit string after values are drawn
                draw_string(disp_datapage_names_x, y_pos, datapage_names[page][lineno], datapage_names[page][lineno], LGRY, NON, forced);  //draw new value
                disp_bargraphs[lineno] = false;
            }
        }
    }
    void draw_hyphen(int x_pos, int y_pos, uint8_t color) {  // draw minus sign in front of negative numbers
        sprptr->drawFastHLine(x_pos+2, y_pos+3, 3, color);
    }
    void drawval_core(int lineno, std::string disp_string, float value, float lowlim, float hilim, float target=NAN, uint8_t color=NON) {
        int age_us = (color != NON) ? disp_value_dimsteps : (int)((float)(dispAgeTimer[lineno].elapsed()) / 25000000); // Divide by us per color gradient quantum
        bool outofrange = (value > hilim || value < lowlim);
        int x_base = disp_datapage_values_x;
        bool polarity = (value > -float_zero) || (std::isnan(value));  // polarity 0=negative, 1=positive
        bool force = std::isnan(value) || disp_data_dirty[lineno];
        if ((disp_values[lineno] != disp_string) || force) {  // if value differs, Erase old value and write new
            if (color == NON) color = (outofrange) ? 0xf8 : 0x3c;
            int y_pos = lineno*disp_line_height_pix;
            if (polarity != disp_polarities[lineno]) draw_hyphen(x_base, y_pos, (!polarity) ? color : BLK);
            draw_string(x_base+disp_font_width, y_pos, disp_string, disp_values[lineno], color, BLK, force || (color != disp_val_colors[lineno])); // +6*(arraysize(modecard[runmode])+4-namelen)/2
            disp_values[lineno] = disp_string;
            disp_polarities[lineno] = polarity;
            disp_val_colors[lineno] = color;
            dispAgeTimer[lineno].reset();
            disp_age_quanta[lineno] = 0;
        }
        else if (age_us > disp_age_quanta[lineno] && age_us < disp_value_dimsteps)  {  // as readings age, redraw in new color. This may fail and redraw when the timer overflows? 
            if (outofrange) color = 0xf8 - (age_us * 0x24);  // if out of range, yellow color loses brightness with age
            else color = 0x3c - (age_us << 2);  // in range green color loses brightness with age
            // if (age_us < 8) color = 0x1c + (age_us << 5);  // base of green with red added as you age, until yellow is achieved
            // else color = 0xfc - ((age_us-8) << 2);  // then lose green as you age further
            int y_pos = (lineno)*disp_line_height_pix;
            if (!polarity) draw_hyphen(x_base, y_pos, color);
            draw_string(x_base+disp_font_width, y_pos, disp_values[lineno], "", color, BLK);
            disp_age_quanta[lineno] = age_us;
            disp_val_colors[lineno] = color;
        }
        bool delete_bargraph = false;
        int corner_x = disp_bargraphs_x;
        int corner_y = lineno*disp_line_height_pix;
        if (!std::isnan(lowlim) && !std::isnan(hilim)) {  // any value having a given range deserves a bargraph gauge with a needle
            int n_pos = (int)(map(value, lowlim, hilim, 1.0f, (float)(disp_bargraph_width-1)));
            uint8_t ncolor = (n_pos > disp_bargraph_width-1 || n_pos < 1) ? RED : GRN;
            n_pos = corner_x + constrain(n_pos, 1, disp_bargraph_width-1);
            if (!disp_bargraphs[lineno]) draw_bargraph_base(corner_x, corner_y + 8, disp_bargraph_width);
            disp_bargraphs[lineno] = true;
            draw_target_shape(disp_targets[lineno], corner_y, BLK, NON);  // erase old target
            draw_bargraph_needle(n_pos, disp_needles[lineno], corner_y, ncolor);  // let's draw a needle
            disp_needles[lineno] = n_pos;  // remember position of needle
            if (!std::isnan(target)) {  // if target value is given, draw a target on the bargraph too
                int t_pos = (int)(map(target, lowlim, hilim, 1.0f, (float)(disp_bargraph_width-1)));
                uint8_t tcolor = (t_pos > disp_bargraph_width-1 || t_pos < 1) ? RED : ( (t_pos != n_pos) ? YEL : GRN );
                t_pos = corner_x + constrain(t_pos, 1, disp_bargraph_width-1);
                draw_target_shape(t_pos, corner_y, tcolor, NON);  // draw the new target
                disp_targets[lineno] = t_pos;  // remember position of target
            }
        }
        else delete_bargraph = true;
        if (delete_bargraph || value == NAN) {
            sprptr->fillRect(corner_x - 1, corner_y, disp_bargraph_width + 2, disp_line_height_pix, BLK);
            disp_bargraphs[lineno] = false;
        }
        disp_data_dirty[lineno] = false;
    }
    void drawval(int lineno, int value, int lowlim=-1, int hilim=-1, int target=-1) {
        std::string val_string = num2string(value, (int)disp_maxlength);
        float lo, hi, targ;
        if (lowlim == -1) lo = NAN;
        else lo = (float)lowlim;
        if (hilim == -1) hi = NAN;
        else hi = (float)hilim;
        if (target == -1) targ = NAN;
        else targ = (float)target;        
        drawval_core(lineno, val_string, (float)value, lo, hi, targ);
    }
    void drawval(int lineno, float value, float lowlim, float hilim, float target=NAN, int sig_places = disp_default_float_sig_dig) {
        std::string val_string;
        if (std::isnan(value)) val_string = "nan";
        else val_string = num2string(value, (int)disp_maxlength, sig_places);
        drawval_core(lineno, val_string, value, lowlim, hilim, target);
    }
    void drawval(int lineno, float value, int sig_places) {
        std::string val_string;
        if (std::isnan(value)) val_string = "nan";
        else val_string = num2string(value, (int)disp_maxlength, sig_places);
        drawval_core(lineno, val_string, value, NAN, NAN, NAN);
    }
    void drawval(int lineno, float value) {
        drawval(lineno, value, NAN, NAN, NAN);
    }
    void draw_eraseval(int lineno) {
        drawval_core(lineno, "", NAN, NAN, NAN, NAN);
    }
    void draw_ascii(int lineno, std::string name) {
        drawval_core(lineno, name, 1, NAN, NAN, NAN, CYN);
    }
    void draw_truth(int lineno, bool truthy, int styl=2) {  // 0:on/off, 1:yes/no, 2:true/false .
        drawval_core(lineno, (truthy) ? ((styl==0) ? "on" : ((styl==1) ? "yes" : "true")) : ((styl==0) ? "off" : ((styl==1) ? "no" : "false")), 1, NAN, NAN, NAN, (truthy) ? LPUR : ORCD);
    }    
    std::string num2string(int value, int maxlength) {  // returns an ascii string representation of a given integer value, using scientific notation if necessary to fit within given width constraint
        value = abs(value);  // This function disregards sign
        int place = significant_place(value);  // check how slow is log() function? Compare performance vs. multiple divides ( see num2string() )
        if (place <= maxlength) return std::to_string(value);  // If value is short enough, return it
        char buffer[maxlength+1];  // Allocate buffer with the maximum required size
        std::snprintf(buffer, sizeof(buffer), "%.*e", maxlength - 4 - (int)(place >= 10), (float)value);
        std::string result(buffer);  // copy buffer to result
        return result.substr(0, result.find('e') + 1) + std::to_string(place);
    }
    std::string num2string(float value, int maxlength, int sig_places=-1, bool chop_zeroes=true) {  // returns an ascii string representation of a given float value, formatted efficiently. It will not exceed maxlength. fractional digits will be removed respecting given number of significant digits
        if (sig_places == -1) sig_places = disp_default_float_sig_dig;
        value = abs(value);  // This function disregards sign
        int place = significant_place(value);  // Learn decimal place of the most significant digit in value
        if (place >= sig_places && place <= maxlength) {  // Then we want simple cast to an integer w/o decimal point (eg 123456, 12345, 1234)
            std::string result(std::to_string((int)value));
            return result;
        }
        if (place >= 0 && place < maxlength) {  // Then we want float formatted with enough nonzero digits after the decimal point for given significant digits (eg 123.4, 12.34, 1.234, 0.000)
            int length = std::min(sig_places+1, maxlength);
            char buffer[length+1];
            std::snprintf(buffer, length + 1, (chop_zeroes) ? "%.*g" : "%.*f", length - 1, value);  // (buf, letters incl. end, %.*g = floats formatted in shortest form, length-1 digits after decimal, val)
            std::string result(buffer);  // copy buffer to result            
            if (value != 0.0 && chop_zeroes && result.find('.') != std::string::npos) result = result.substr(0, result.find_last_not_of('0') + 1);
            if (result.back() == '.') result.pop_back();
            return result;
        }
        if (place < 0 && sig_places - place <= maxlength) {  // Then we want decimal w/o initial '0' limited to given significant digits (eg .123, .0123, .00123)
            std::string result (std::to_string(value));  // sd=3,  0.1234  d=1 l=6    0.00123
            size_t decimalPos = result.find('.');  // decimalPos will always be 1 (?)
            if (decimalPos != std::string::npos) result = result.substr(decimalPos, std::min(sig_places-place, maxlength));  // Remove any digits to the left of the decimal point
            if (value != 0.0 && chop_zeroes && result.find('.') != std::string::npos) result = result.substr(0, result.find_last_not_of('0') + 1);
            return result;
        }  // Otherwise we want scientific notation with precision removed as needed to respect maxlength (eg 1.23e4, 1.23e5, but using long e character not e for negative exponents
        char buffer[maxlength+1];  // Allocate buffer with the maximum required size
        int truncit = std::min(sig_places - 1, maxlength - 4 - (int)(place <= -10 || place >= 10));
        std::snprintf(buffer, sizeof(buffer), "%.*e", truncit, value);
        std::string result(buffer);  // copy buffer to result
        result += std::to_string(std::abs(place));  // put exponent on
        if (result.find("e+0") != std::string::npos) result.replace(result.find("e+0"), 3, "e");  // Remove useless "+0" from exponent
        else if (result.find("e-0") != std::string::npos) result.replace(result.find("e-0"), 3, "\x88");  // For very small scientific notation values, replace the "e-0" with a phoenetic long e character, to indicate negative power  // if (result.find("e-0") != std::string::npos) 
        else if (result.find("e+") != std::string::npos) result.replace(result.find("e+"), 2, "e");  // For ridiculously large values
        else if (result.find("e-") != std::string::npos) result.replace(result.find("e-"), 2, "\x88");  // For ridiculously small values
        return result;
    }
    void draw_runmode(int _nowmode, uint8_t color_override=NON) {  // color_override = -1 uses default color
        sprptr->setTextDatum(textdatum_t::top_left);
        sprptr->fillRect(disp_runmode_text_x, 0, (modecard[runmode_last].length() + 5) * disp_font_width, disp_font_height, BLK);
        sprptr->setTextColor((color_override == NON) ? colorcard[_nowmode] : color_override);  
        sprptr->setCursor(disp_runmode_text_x, 0);
        sprptr->print(modecard[_nowmode].c_str());
        sprptr->print(" Mode");
        disp_runmode_dirty = false;
        runmode_last = _nowmode;
    }
    void draw_datapage(int page, bool forced=false) {
        if (forced) {
            for (int i = disp_fixed_lines; i < disp_lines; i++) {
                disp_age_quanta[i] = 0;
                dispAgeTimer[i].reset();
                disp_data_dirty[i] = true;
            }
            disp_values_dirty = true;
        }
        draw_fixed(page, disp_datapage_last, true, forced);  // Erase and redraw variable names and units for data on this page
        draw_string(disp_datapage_title_x, 0, pagecard[page], pagecard[disp_datapage_last], STBL, BLK, forced); // +6*(arraysize(modecard[_runmode.mode()])+4-namelen)/2
        disp_datapage_last = page;
        disp_units_dirty = true;
        disp_datapage_dirty = false;
        prefs.putUInt("dpage", (uint32_t)page);
    }
    void draw_unitvals(int page) {
        for (int lineno = 0; lineno < disp_tuning_lines; lineno++) {  // step thru lines of fixed telemetry data
            int y_pos = (lineno + disp_fixed_lines + 1) * disp_line_height_pix;
            draw_string_units(disp_datapage_units_x, y_pos, tuneunits[page][lineno], tuneunits[page][lineno], LGRY, NON);  // erase value first (above) in case new long value string overlaps old units string
        }
        disp_units_dirty = false;
    }
    void draw_selected_name(int tun_ctrl, int selection, int selected_last, int selected_last_last) {
        static int last_selected; 
        uint8_t color = LGRY;
        if (tun_ctrl == EDIT) color = GRN;
        else if (tun_ctrl == SELECT) color = YEL;
        draw_string(12, (last_selected + disp_fixed_lines + 1) * disp_line_height_pix, datapage_names[datapage][last_selected], nulstr, LGRY, BLK, true);
        draw_string(12, (selection + disp_fixed_lines + 1) * disp_line_height_pix, datapage_names[datapage][selection], nulstr, color, BLK, true);
        last_selected = selection;
        disp_selection_dirty = false;    
    }
    void draw_menu_toggle(bool value, int col, bool force=false) {  // Draws values of boolean data
        if ((disp_bool_values[col - 2] != value) || force) {  // If value differs, Erase old value and write new
            std::string drawme = top_menu_buttons[col-2];
            if (drawme == "CAL" && !(runmode == STANDBY || runmode == CAL)) drawme = "NA";
            else if (drawme == "CH4") drawme = ch4_menu_buttons[runmode];
            int x_mod = touch_margin_h_pix + touch_cell_h_pix * (col) + (touch_cell_h_pix >> 1);
            sprptr->fillRect(x_mod - 5 * (disp_font_width >> 1) + 1, 0, 5 * disp_font_width, disp_font_height, DGRY);
            sprptr->setTextDatum(textdatum_t::top_left);
            sprptr->setFont(&fonts::Font0);
            sprptr->setTextColor((value) ? YEL : LGRY);  
            sprptr->drawString(drawme.c_str(), x_mod - drawme.length() * (disp_font_width >> 1) + 1, 0);
            disp_bool_values[col - 2] = value;
        }
    }
    void draw_menus(bool side_only = false) {  // draws edge buttons with names in 'em. If replace_names, just updates names
        sprptr->setTextDatum(textdatum_t::top_left);
        int namelen = 0;
        sprptr->setTextColor(LGRY);
        for (int row = 0; row < arraysize(side_menu_buttons); row++) {  // Step thru all rows to draw buttons along the left edge
            std::string drawme = side_menu_buttons[row];
            if (drawme == "ANI" && runmode != STANDBY) drawme = "NA";
            sprptr->fillRoundRect(-9, touch_cell_v_pix * row + 3, 18, touch_cell_v_pix - 6, 8, DGRY);
            sprptr->drawRoundRect(-9, touch_cell_v_pix * row + 3, 18, touch_cell_v_pix - 6, 8, LGRY);
            namelen = 0;
            for (int x = 0 ; x < drawme.length() ; x++ ) {
                if (drawme[x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
            }
            for (int letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
                sprptr->setCursor(1, (touch_cell_v_pix * row) + (touch_cell_v_pix / 2) + (disp_font_height + 1) * (letter - (namelen >> 1)) - 3); // adjusts vertical offset depending how many letters in the button name and which letter we're on
                sprptr->print(drawme[letter]);  // Writes each letter such that the whole name is centered vertically on the button
            }
        }
        if (!side_only) {
            for (int col = 2; col <= 5; col++) {  // Step thru all cols to draw buttons across the top edge
                sprptr->fillRoundRect(touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
                sprptr->drawRoundRect(touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, LGRY);  // sprptr->width()-9, 3, 18, (sprptr->height()/5)-6, 8, LGRY);
            }
        }
        disp_menus_dirty = false;
    }
    void draw_reticle(LGFX_Sprite* spr, int x, int y) {
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
    void draw_temp(loc location, int draw_index) {
        if (!tempsens.detected(location)) draw_eraseval(draw_index);
        else drawval(draw_index, tempsens.val(location), tempsens.opmin(location), tempsens.opmax(location));  //temp_lims_f[tempsens.errclass(location)][DISP_MIN], temp_lims_f[tempsens.errclass(location)][DISP_MAX]);
    }
    void draw_idiotbitmap(int i, int x, int y) {
        uint8_t bg = idiots->val(i) ? idiots->color[ON][i] : BLK;
        uint8_t color = idiots->val(i) ? BLK : idiots->color[OFF][i];
        sprptr->drawRoundRect(x, y, 2 * disp_font_width + 1, disp_font_height + 1, 1, bg);
        for (int xo = 0; xo < (2 * disp_font_width - 1); xo++)
            for (int yo = 0; yo < disp_font_height - 1; yo++)
                sprptr->drawPixel(x + xo + 1, y + yo + 1, ((idiots->icon[i][xo] >> yo) & 1) ? color : bg);
    }
    void draw_idiotlight(int i, int x, int y) {
        if (!idiots->val(i)) sprptr->fillRect(x, y, 2 * disp_font_width + 1, disp_font_height + 1, BLK);  // erase rectangle when turning off. need to test if this is necessary
        if (idiots->icon[i][0] == 0xff) {  // 0xff in the first byte will draw 2-letter string instead of bitmap
            sprptr->fillRoundRect(x, y, 2 * disp_font_width + 1, disp_font_height + 1, 1, (idiots->val(i)) ? idiots->color[ON][i] : BLK);
            sprptr->setTextColor(idiots->val(i) ? BLK : idiots->color[OFF][i]);  // darken_color((*(idiots->lights[index])) ? BLK : DGRY)
            sprptr->setCursor(x+1, y+1);
            sprptr->print(idiots->letters[i].c_str());
        }
        else if (idiots->icon[i][0] != 0x88) draw_idiotbitmap(i, x, y);  // 0x88 in the first byte will skip a space
        idiots->last[i] = idiots->val(i);
    }
    void update_idiots(bool force = false) {
        for (int i = 0; i < idiots->num_idiots(); i++) {
            if (i < neo->num_neo_idiots()) {
                if (idiots->val(i) != idiots->last[i]) neo->setBoolState(i, idiots->val(i));
                if (idiots->ptr(i) == &panicstop || idiots->ptr(i) == &diag.err_sens[RANGE][_TempEng] || idiots->ptr(i) == &wheeltemperr) neo->setflash(i, 3, 1, 2, 100, 0xffffff);  // add a brilliant flash to the more critical idiot lights
                else if (idiots->ptr(i) == &diag.err_sens_alarm[LOST] || idiots->ptr(i) == &diag.err_sens_alarm[RANGE]) neo->setflash(i, diag.errorcount(i), 2, 6, 1, 0);  // encode number of errored sensors with black blinks
                else neo->setflash(i, 0);
            }
            if (force || (idiots->val(i) ^ idiots->last[i])) {
                draw_idiotlight(i, idiots_corner_x + (2 * disp_font_width + idiots_spacing_x + 1) * (i % idiots->row_count), idiots_corner_y + idiots->row_height * (int)(i / idiots->row_count));
            }
        }
        disp_idiots_dirty = false;
    }
    void disp_menu_bools() {
        draw_menu_toggle((runmode == CAL), 2, disp_menutoggles_dirty);
        draw_menu_toggle(sim->simulating(), 3, disp_menutoggles_dirty);  // fuelpump.status()
        draw_menu_toggle(starter.motor, 4, disp_menutoggles_dirty);
        draw_menu_toggle(ignition.signal, 5, disp_menutoggles_dirty);
        disp_menutoggles_dirty = false;
    }
    // value rendering options:  for [optional] arguments use -1 (for int) or NAN (for float) to get the default, same as not including. Default sig_places for floats is 3
    // * drawval (int_line, float_value)  // for floats
    // * drawval (int_line, float_value, [int_sig_places])  // for floats. sig_places is how many digits after decimal (if they fit) 
    // * drawval (int_line, float_value, [float_min], [float_max], [float_targ], [int_sig_places])  // for floats. if min & max are given it draws a bargraph of that range is drawn. If targ value is given then bargraph will include a target pointer. sig_places is how many digits after decimal (if they fit) 
    // * drawval (int_line, int_value, [int_min], [int_max], [int_targ])  // for ints. if min & max are given it draws a bargraph of that range. If targ value is given then bargraph will include a target pointer. 
    // * draw_truth (int_line, bool_value, [int_style])  // for bools. styles: 0 (on/off), 1 (yes/no), 2 (true/false) (default)
    // * draw_temp (int_line, sensor_location)  // for drawing temperatures
    // * draw_ascii (int_line, string)  // string must be length 6 max
    // * draw_eraseval (int_line)  // leaves the entry blank. use for every line not containing a value
    void disp_datapage_values() {
        float drange;
        drawval(1, hotrc.pc[VERT][FILT], hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
        drawval(2, hotrc.pc[HORZ][FILT], hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
        drawval(3, speedo.val(), 0.0f, speedo.opmax(), gas.cruisepid.target());
        drawval(4, tach.val(), 0.0f, tach.opmax(), gas.pid.target());
        drawval(5, brake.combined_read_pc, 0.0, 100.0, brake.target_pc);  // (brake_active_pid == S_PID) ? (int)brakeSPID.targ() : pressure_target_adc);
        drawval(6, gas.pc[OUT], gas.pc[OPMIN], gas.pc[OPMAX], gas.throttle_target_pc);
        drawval(7, brake.pc[OUT], brake.pc[OPMIN], brake.pc[OPMAX]);
        drawval(8, steer.pc[OUT], steer.pc[OPMIN], steer.pc[OPMAX]);
        if (datapage == PG_RUN) {
            drawval(9, pressure.val(), pressure.opmin(), pressure.opmax());
            drawval(10, brkpos.val(), brkpos.opmin(), brkpos.opmax());
            drawval(11, mulebatt.val(), mulebatt.opmin(), mulebatt.opmax(), NAN, 4);
            drawval(12, pot.val(), pot.opmin(), pot.opmax());
            drawval(13, airvelo.val(), airvelo.opmin(), airvelo.opmax());
            drawval(14, mapsens.val(), mapsens.opmin(), mapsens.opmax());
            drawval(15, maf_gps, maf_min_gps, maf_max_gps);
            draw_ascii(16, motormodecard[gas.motormode]);
            draw_ascii(17, motormodecard[brake.motormode]);
            draw_ascii(18, motormodecard[steer.motormode]);
            drawval(19, looptimer.uptime());
            for (int line=20; line<=21; line++) draw_eraseval(line);
            drawval(22, gas.governor, 0.0f, 100.0f, NAN, 1);
            drawval(23, steer.steer_safe_pc, 0.0f, 100.0f, NAN, 1);
        }
        else if (datapage == PG_JOY) {
            drawval(9, hotrc.us[HORZ][FILT], hotrc.us[HORZ][OPMIN], hotrc.us[HORZ][OPMAX]);
            drawval(10, hotrc.us[VERT][FILT], hotrc.us[VERT][OPMIN], hotrc.us[VERT][OPMAX]);
            drawval(11, hotrc.us[HORZ][RAW], hotrc.us[HORZ][OPMIN], hotrc.us[HORZ][OPMAX]);
            drawval(12, hotrc.us[VERT][RAW], hotrc.us[VERT][OPMIN], hotrc.us[VERT][OPMAX]);
            drawval(13, hotrc.us[CH3][RAW], hotrc.us[CH3][OPMIN], hotrc.us[CH3][OPMAX]);
            drawval(14, hotrc.us[CH4][RAW], hotrc.us[CH4][OPMIN], hotrc.us[CH4][OPMAX]);
            drawval(15, hotrc.pc[HORZ][RAW], hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
            drawval(16, hotrc.pc[VERT][RAW], hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
            for (int line=17; line<=18; line++) draw_eraseval(line);
            drawval(19, airvelo.opmax(), airvelo.absmin(), airvelo.absmax());
            drawval(20, mapsens.opmin(), mapsens.absmin(), mapsens.absmax());
            drawval(21, mapsens.opmax(), mapsens.absmin(), mapsens.absmax());
            drawval(22, hotrc.failsafe_us, hotrc.absmin_us, hotrc.us[VERT][OPMIN] - hotrc.us[VERT][MARGIN]);
            drawval(23, hotrc.deadband_us, 0.0, 100.0);
        }
        else if (datapage == PG_SENS) {
            drawval(9, pot.native(), pot.absmin_native(), pot.absmax_native());
            drawval(10, brkpos.native(), brkpos.opmin_native(), brkpos.opmax_native());
            drawval(11, brkpos.val(), brkpos.opmin(), brkpos.opmax());
            drawval(12, brkpos.pc(), 0.0, 100.0);
            drawval(13, pressure.native(), pressure.opmin_native(), pressure.opmax_native());
            drawval(14, pressure.val(), pressure.opmin(), pressure.opmax());
            drawval(15, pressure.pc(), 0.0, 100.0);
            // drawval(11, brkpos.raw(), brkpos.opmin(), brkpos.opmax());
            // drawval(15, pressure.raw(), pressure.opmin(), pressure.opmax());
            drawval(16, mulebatt.native(), mulebatt.opmin_native(), mulebatt.opmax_native());
            drawval(17, mulebatt.val(), mulebatt.opmin(), mulebatt.opmax(), NAN, 4);
            draw_eraseval(18);
            drawval(19, pressure.opmin(), pressure.absmin(), pressure.absmax());
            drawval(20, pressure.opmax(), pressure.absmin(), pressure.absmax());
            drawval(21, brkpos.opmin(), brkpos.absmin(), brkpos.absmax());
            drawval(22, brkpos.opmax(), brkpos.absmin(), brkpos.absmax());
            drawval(23, brkpos.zeropoint(), brkpos.opmin(), brkpos.opmax());  // BrakePositionSensor::absmin_retract_in, BrakePositionSensor::absmax_extend_in);
        }
        else if (datapage == PG_PULS) {
            drawval(9, tach.ms(), tach.absmin_ms(), tach.absmax_ms());
            drawval(10, tach.native(), tach.opmin_native(), tach.opmax_native());
            drawval(11, tach.raw(), tach.opmin(), tach.opmax());
            drawval(12, speedo.ms(), speedo.absmin_ms(), speedo.absmax_ms());
            drawval(13, speedo.native(), speedo.opmin_native(), speedo.opmax_native());
            drawval(14, speedo.raw(), speedo.opmin(), speedo.opmax());
            drawval(15, speedo.pc(), 0.0, 100.0);
            for (int line=16; line<=18; line++) draw_eraseval(line);
            drawval(19, tach.opmin(), tach.absmin(), tach.absmax());
            drawval(20, tach.opmax(), tach.absmin(), tach.absmax());
            drawval(11, speedo.opmin(), speedo.absmin(), speedo.absmax());
            drawval(22, speedo.opmax(), speedo.absmin(), speedo.absmax());
            drawval(23, speedo.idle(), speedo.opmin(), speedo.opmax());
        }
        else if (datapage == PG_PWMS) {
            drawval(9, gas.si[OUT], gas.si[OPMIN], gas.si[OPMAX]);
            drawval(10, gas.us[OUT], gas.us[ABSMIN], gas.us[ABSMAX]);
            drawval(11, brake.volt[OUT], brake.volt[OPMIN], brake.volt[OPMAX]);
            drawval(12, brake.us[OUT], brake.us[ABSMIN], brake.us[ABSMAX]);
            drawval(13, steer.volt[OUT], steer.volt[OPMIN], steer.volt[OPMAX]);
            drawval(14, steer.us[OUT], steer.us[ABSMIN], steer.us[ABSMAX]);
            for (int line=15; line<=19; line++) draw_eraseval(line);
            drawval(20, gas.si[OPMIN], gas.si[ABSMIN], gas.si[ABSMAX]);
            drawval(21, gas.si[OPMAX], gas.si[ABSMIN], gas.si[ABSMAX]);
            drawval(22, brake.us[STOP], brake.us[ABSMIN], brake.us[ABSMAX]);
            drawval(23, brake.duty_fwd_pc, 0.0f, 100.0f);
        }
        else if (datapage == PG_IDLE) {
            draw_ascii(9, motormodecard[gas.motormode]);
            drawval(10, gas.pid.target(), tach.opmin(), tach.opmax());
            drawval(11, gas.idle_boost_pc, 0.0f, gas.idle_max_boost_pc);
            drawval(12, gas.idle_pc(), gas.pc[OPMIN], gas.pc[OPMAX]);
            drawval(13, gas.idle_si(), gas.si[OPMIN], gas.si[OPMAX]);
            drawval(14, tach.idle(), tach.opmin(), tach.opmax());
            drawval(15, fuelpump.volts(), fuelpump.off_v, fuelpump.on_max_v);
            for (int line=16; line<=19; line++) draw_eraseval(line);
            drawval(20, gas.starting_pc, gas.pc[OPMIN], gas.pc[OPMAX]);
            drawval(21, gas.idle_max_boost_pc);
            drawval(22, gas.idle_temp_lim_f[LOW], tempsens.opmin(loc::ENGINE), tempsens.opmax(loc::ENGINE));
            drawval(23, gas.idle_temp_lim_f[HIGH], tempsens.opmin(loc::ENGINE), tempsens.opmax(loc::ENGINE));
        }
        else if (datapage == PG_MOTR) {
            drawval(9, brake.motorheat(), brake.motorheatmin(), brake.motorheatmax());  // brake_spid_speedo_delta_adc, -range, range);
            drawval(10, brake.combined_read_pc, 0.0, 100.0);  // brake_spid_speedo_delta_adc, -range, range);
            for (int myline=11; myline<=13; myline++) draw_eraseval(myline);
            draw_truth(14, brake.pid_enabled, 1);
            draw_ascii(15, brakefeedbackcard[brake.feedback]);
            draw_ascii(16, openloopmodecard[brake.openloop_mode]);
            draw_truth(17, brake.enforce_positional_limits, 1);
            drawval(18, brake.max_out_change_rate_pcps);
            draw_truth(19, gas.pid_enabled, 1);
            draw_truth(20, gas.cruise_pid_enabled, 1);
            draw_ascii(21, cruiseschemecard[gas.cruise_adjust_scheme]);
            draw_truth(22, cruise_brake, 1);
            draw_ascii(23, modecard[drive_mode]);
        }
        else if (datapage == PG_BPID) {
            drange = brake.us[ABSMIN]-brake.us[ABSMAX];
            draw_ascii(9, motormodecard[brake.motormode]);
            drawval(10, pressure.pc(), 0.0, 100.0, brake.pids[PressureFB].target());
            drawval(11, brake.pids[PressureFB].target(), pressure.opmin(), pressure.opmax());
            drawval(12, brkpos.pc(), 0.0, 100.0, brake.pids[PositionFB].target());
            drawval(13, brake.pids[PositionFB].target(), brkpos.opmin(), brkpos.opmax());
            drawval(14, brake.target_pc, 0.0, 100.0);
            // draw_ascii(9, motormodecard[brake.motormode]);
            // draw_ascii(10, brakefeedbackcard[brake.feedback]);
            // drawval(12, brake.pid_dom->err(), -brake.sensmax(), brake.sensmax());
            // drawval(13, brake.target[PressureFB], 0.0f, 100.0f);  // brake.pid_dom->outmin(), brake.pid_dom->outmax());
            // drawval(14, brake.target[PositionFB], 0.0f, 100.0f);  // brake.pid_dom->outmin(), brake.pid_dom->outmax());
            drawval(15, brake.hybrid_out_ratio_pc, 0.0f, 100.0f);  // brake_spid_speedo_delta_adc, -range, range);
            drawval(16, brake.pid_dom->pterm(), -drange, drange);
            drawval(17, brake.pid_dom->outsum(), -drange, drange);
            drawval(18, brake.pid_dom->iterm(), -drange, drange);
            drawval(19, brake.pid_dom->dterm(), -drange, drange);
            drawval(20, brake.pid_dom->sampletime());
            drawval(21, brake.pid_dom->kp());
            drawval(22, brake.pid_dom->ki());
            drawval(23, brake.pid_dom->kd());
        }
        else if (datapage == PG_GPID) {
            draw_ascii(9, motormodecard[gas.motormode]);
            drawval(10, gas.throttle_target_pc, tach.opmin(), tach.opmax());
            drawval(11, gas.pid.target(), tach.opmin(), tach.opmax());
            drawval(12, gas.pid.err(), tach.idle() - tach.opmax(), tach.opmax() - tach.idle());
            drawval(13, gas.pid.pterm(), -100.0f, 100.0f);
            drawval(14, gas.pid.iterm(), -100.0f, 100.0f);
            drawval(15, gas.pid.dterm(), -100.0f, 100.0f);
            // drawval(15, gas.pid.outsum(), -gas.pid.outrange(), gas.pid.outrange());
            for (int line=16; line<=19; line++) draw_eraseval(line);
            drawval(20, gas.max_throttle_angular_velocity_degps, 0.0f, 360.0f);
            drawval(21, gas.pid.kp());
            drawval(22, gas.pid.ki());
            drawval(23, gas.pid.kd());
        }
        else if (datapage == PG_CPID) {
            drange = tach.opmax() - tach.idle();
            drawval(9, gas.cruisepid.target(), speedo.idle(), speedo.opmax());
            drawval(10, gas.cruisepid.err(), speedo.idle()-speedo.opmax(), speedo.opmax()-speedo.idle());
            drawval(11, gas.cruisepid.pterm(), -drange, drange);
            drawval(12, gas.cruisepid.iterm(), -drange, drange);
            drawval(13, gas.cruisepid.dterm(), -drange, drange);
            // drawval(14, gas.cruisepid.outsum(), -gas.cruisepid.outrange(), gas.cruisepid.outrange());  // cruise_spid_speedo_delta_adc, -drange, drange);
            drawval(14, gas.throttle_target_pc, 0.0f, 100.0f);
            for (int line=15; line<=19; line++) draw_eraseval(line);
            drawval(20, cruise_delta_max_pc_per_s, 1, 35);
            drawval(21, gas.cruisepid.kp());
            drawval(22, gas.cruisepid.ki());
            drawval(23, gas.cruisepid.kd());
        }
        else if (datapage == PG_TEMP) {
            draw_temp(loc::AMBIENT, 9);
            draw_temp(loc::ENGINE, 10);
            draw_temp(loc::WHEEL_FL, 11);
            draw_temp(loc::WHEEL_FR, 12);
            draw_temp(loc::WHEEL_RL, 13);
            draw_temp(loc::WHEEL_RR, 14);
            draw_temp(loc::BRAKE, 15);
            for (int line=16; line<=20; line++) draw_eraseval(line);
            drawval(21, wheeldifferr);
            draw_truth(22, dont_take_temperatures, 2);
            draw_truth(23, web_disabled, 2);
        }
        else if (datapage == PG_SIM) {
            for (int line=9; line<=12; line++) draw_eraseval(line);
            draw_truth(13, sim->can_sim(sens::joy), 0);
            draw_truth(14, sim->can_sim(sens::pressure), 0);
            draw_truth(15, sim->can_sim(sens::brkpos), 0);
            draw_truth(16, sim->can_sim(sens::speedo), 0);
            draw_truth(17, sim->can_sim(sens::tach), 0);
            draw_truth(18, sim->can_sim(sens::airvelo), 0);
            draw_truth(19, sim->can_sim(sens::mapsens), 0);
            draw_truth(20, sim->can_sim(sens::basicsw), 0);                    
            draw_ascii(21, sensorcard[sim->potmap()]);
            draw_truth(22, cal_brakemode, 0);
            draw_truth(23, cal_gasmode, 0);
        }
        else if (datapage == PG_UI) {
            drawval(9, loop_avg_us);
            drawval(10, looptimer.loop_peak_us);
            // drawval(11, (int)looptimer.loopfreq_hz);
            drawval(11, fps);
            draw_ascii(12, activitiescard[last_activity]);
            drawval(13, touch->touch_pt(0), 0, disp_width_pix);
            drawval(14, touch->touch_pt(1), 0, disp_height_pix);
            drawval(15, encoder.spinrate(), 0.0, encoder.spinrate_max());
            drawval(16, encoder.accel_factor(), 1, encoder.accel_max());
            drawval(17, ezread.offset, 0, ezread.bufferSize);  //  - ezread.num_lines);
            draw_ascii(18, neo->pcbaglowcard[neo->pcbaglow]);
            draw_truth(19, flashdemo, 0);
            draw_truth(20, neo->sleepmode, 0);
            drawval(21, neobright, 0.0, 100.0);  // drawval(22, neobright, 1.0, 100.0f, -1, 3);
            drawval(22, neosat, 1.0, 100.0);  // drawval(22, neobright, 1.0, 100.0f, -1, 3);
            draw_ascii(23, uicontextcard[ui_context]);
        }
        disp_values_dirty = false;
    }
  public:
    bool draw_all(LGFX_Sprite* spr) {
        static bool starter_last = false;
        if (!display_enabled) return false;
        if (reset_request) reset();
        auto_saver();
        if (!auto_saver_enabled) {
            tiny_text();
            update_idiots(disp_idiots_dirty);
            if (disp_datapage_dirty) draw_datapage(datapage, true);
            if (disp_menus_dirty) draw_menus(false);
            if (disp_selection_dirty) draw_selected_name(tunctrl, sel, sel_last, sel_last_last);
            if (runmode != runmode_last) disp_runmode_dirty = true;
            if (disp_values_dirty || disp_runmode_dirty || valuesRefreshTimer.expireset()) {
                disp_datapage_values();
                disp_menutoggles_dirty = true;
            }
            if (starter.motor != starter_last) disp_menutoggles_dirty = true;
            starter_last = starter.motor;
            if (disp_menutoggles_dirty) disp_menu_bools();
            if (disp_units_dirty) draw_unitvals(datapage);
            if (disp_runmode_dirty) draw_runmode(runmode, NON);
        }
        if (sim->enabled() != sim_last) disp_simbuttons_dirty = true;
        sim_last = sim->enabled();
        fps = panel.update(spr, disp_simbuttons_dirty);
        disp_simbuttons_dirty = false;
        return true;
    }
    void do_push() {
        if (print_framebuffers) {  // warning this *severely* slows everything down, ~.25 sec/loop. consider disabling word wrap in terminal output
            ezread.squintf("flip=%d\n", flip);
            printframebufs(2);
        }
        diffpush(&framebuf[flip], &framebuf[!flip]);
        flip = !flip;
        sprptr = &framebuf[flip];
        // pushclock = (int)(esp_timer_get_time() - screen_refresh_time);
    }
    void do_draw() {
        int mark = (int)esp_timer_get_time();
        draw_all(&framebuf[flip]);
        // drawclock = (int)(esp_timer_get_time() - mark);
        // idleclock = refresh_limit - pushclock - drawclock;
    }
    void diffpush(LGFX_Sprite* source, LGFX_Sprite* ref) {
        union {  // source
            std::uint32_t* s32;
            std::uint8_t* s;
        };
        union {  // reference
            std::uint32_t* r32;
            std::uint8_t* r;
        };
        s32 = (std::uint32_t*)source->getBuffer();
        r32 = (std::uint32_t*)ref->getBuffer();
        auto sprwidth = source->width();
        auto sprheight = source->height();
        auto w32 = (sprwidth + 3) >> 2;
        std::int32_t y = 0;
        lcd.startWrite();
        do {
            std::int32_t x32 = 0;
            do {
                while (s32[x32] == r32[x32] && ++x32 < w32);
                if (x32 == w32) break;
                std::int32_t xs = x32 << 2;
                while (s[xs] == r[xs]) ++xs;
                while (++x32 < w32 && s32[x32] != r32[x32]);
                std::int32_t xe = (x32 << 2) - 1;
                if (xe >= sprwidth) xe = sprwidth - 1;
                while (s[xe] == r[xe]) --xe;
                lcd.pushImageDMA(xs, y, xe - xs + 1, 1, &s[xs]);
                memcpy(&r[xs], &s[xs], sizeof(s[0])*(xe - xs + 1));
            } while (x32 < w32);
            s32 += w32;
            r32 += w32;
        } while (++y < sprheight);
        lcd.endWrite();   // lcd->display();
    }
    void auto_saver() {
        static int last_context;
        if (autosaver_request == REQ_TOG) autosaver_request = auto_saver_enabled ? REQ_OFF : REQ_ON;  // (int)(!auto_saver_enabled);
        if (runmode != STANDBY) {
            if (autosaver_request == REQ_ON) autosaver_request = REQ_NA;
            if (auto_saver_enabled) autosaver_request = REQ_OFF;
        }
        if (autosaver_request == (int)(auto_saver_enabled)) autosaver_request = REQ_NA;
        static bool was_simulating;
        if (autosaver_request == REQ_ON) {
            was_simulating = sim->enabled();
            sim->disable();
            panel.set_vp(0, 0, disp_width_pix, disp_height_pix);
            auto_saver_enabled = true;
            last_context = ui_context;
            ui_context = ScreensaverUI;
            panel.anim_reset_request = true;
        }
        else if (autosaver_request == REQ_OFF) {
            auto_saver_enabled = false;
            ui_context = last_context;
            panel.set_vp(disp_apppanel_x, disp_apppanel_y, disp_apppanel_w, disp_apppanel_h);
            reset_request = true;
            if (was_simulating) sim->enable();
        }
        autosaver_request = REQ_NA;
    }
    void printframebufs(int reduce=2, bool ascii=false) {  // reduce is how many times to shrink the screen by half (0, 1, 2, 3, or 4). ascii=true gives ascii art output
        std::string brites[16] = {" ", ".", ",", ":", ";", "+", "=", ">", "%", "#", "*", "$", "@", "&", "M", "W"};
        int found;
        uint8_t* s;
        for (int f=0; f<2; f++) {
            s = (uint8_t*)(framebuf[f]).getBuffer();
            for (int y=0; y<disp_height_pix >> reduce; y++) {
                Serial.printf("%d ", f);
                for (int x=0; x<disp_width_pix >> reduce; x++) {
                    found = 0; 
                    for (int sy=0; sy<reduce; sy++) {
                        for (int sx=0; sx<reduce; sx++) {
                            if (s[((y << reduce) + sy)*disp_width_pix + (x << reduce) + sx] != 0x00) found++;
                        }
                    }
                    if (ascii) Serial.printf("%s", brites[(reduce >= 2) ? (found >> (reduce - 2)) : (found << (2 - reduce))].c_str());
                    else Serial.printf("%s", (found > 0) ? "*" : ".");
                }
                Serial.printf("\n");
            }
            Serial.printf("\n");
        }
    }
};
class Tuner {
  private:
    Display* screen;
    NeopixelStrip* neo;
    Touchscreen* touch;
    Timer tuningAbandonmentTimer{45000000};  // This times out edit mode after a a long period of inactivity
    Timer tuningEditTimer{50000};  // Control frequency of polling for new edits
    int datapage_last;
  public:
    Tuner(Display* _screen, NeopixelStrip* _neo, Touchscreen* _touch) : screen(_screen), neo(_neo), touch(_touch) {}
    int id, id_encoder = 0;  // idelta is integer edit value accelerated, and is used for all tuning edits regardless if int float or bool
    int rdelta_encoder = 0;  // rdelta is raw (unaccelerated) integer edit value, idelta is integer edit value accelerated
    void update() {
        if (runmode == LOWPOWER) return;
        process_inputs();
        edit_values();
    }
  private:
    void process_inputs() {
        if (!tuningEditTimer.expired() || auto_saver_enabled) return;
        tuningEditTimer.reset();
        if (!screen->disp_selection_dirty) {
            sel_last = sel;
            tunctrl_last = tunctrl;
        }
        if (!screen->disp_datapage_dirty) datapage_last = datapage;
        int encoder_sw_action = encoder.button.press_event();  // true = autoreset the event if there is one
        if (encoder_sw_action != swNONE) {  // First deal with any unhandled switch press events
            if (encoder_sw_action == swSHORT)  {  // if short press
                if (tunctrl == EDIT) tunctrl = SELECT;  // If we were editing a value drop back to select mode
                else if (tunctrl == SELECT) tunctrl = EDIT;  // If we were selecting a variable start editing its value
                else if (button_test_heartbeat_color) heartbeat_override_color = rando_color();  // temporary!! to test heartbeat color override feature
            }
            else tunctrl = (tunctrl == OFF) ? SELECT : OFF;  // Long press starts/stops tuning
        }
        id_encoder = encoder.rotation(true);   // accelerated
        rdelta_encoder = constrain(id_encoder, -1, 1);
        if (tunctrl == SELECT) sel += rdelta_encoder;  // If overflow constrain will fix in general handler below
        else if (tunctrl == OFF) datapage += rdelta_encoder;  // If overflow tconstrain will fix in general below
        if (touch->increment_sel) ++sel %= disp_tuning_lines;
        if (touch->increment_datapage) ++datapage %= NUM_DATAPAGES;
        touch->increment_sel = touch->increment_datapage = false;
        id = id_encoder + touch->get_delta();  // Allow edits using the encoder or touchscreen
        if (pot_tuner_acceleration && !sim.potmapping()) id = constrain(id * (int)(map(pot.val(), 0.0, 100.0, 1.0, 10.0)), 1, (int)encoder._accel_max); // {  // use pot to control level of acceleration
        if (tunctrl != tunctrl_last || datapage != datapage_last || sel != sel_last || id) tuningAbandonmentTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
        else if (tuningAbandonmentTimer.expired()) tunctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
        datapage = constrain(datapage, 0, datapages::NUM_DATAPAGES-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
        if (datapage != datapage_last) {
            if (tunctrl == EDIT) tunctrl = SELECT;  // If page is flipped during edit, drop back to select mode
            screen->disp_datapage_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
        }
        if (tunctrl == SELECT) {
            sel = constrain(sel, tuning_first_editable_line[datapage], disp_tuning_lines-1);  // Skip unchangeable values for all PID modes
            if (sel != sel_last) screen->disp_selection_dirty = true;
        }
        if (tunctrl != tunctrl_last || screen->disp_datapage_dirty) screen->disp_selection_dirty = true;
    }
    void edit_values() {  // Change tunable values when editing
        if (tunctrl != EDIT || !id) return;
        if (datapage == PG_RUN) {
            if (sel == 13) gas.set(&gas.governor, tune(gas.governor, id, 0.0f, 100.0f));
            else if (sel == 14) tune(&steer.steer_safe_pc, id, 0.0f, 100.0f);
        }
        else if (datapage == PG_JOY) {
            if (sel == 10) airvelo.set_oplim(NAN, tune(airvelo.opmax(), id, airvelo.opmin(), airvelo.absmax()));
            else if (sel == 11) mapsens.set_oplim(tune(mapsens.opmin(), id, mapsens.absmin(), mapsens.opmax()), NAN);
            else if (sel == 12) mapsens.set_oplim(NAN, tune(mapsens.opmax(), id, mapsens.opmin(), mapsens.absmax()));
            else if (sel == 13) hotrc.set(&hotrc.failsafe_us, tune(hotrc.failsafe_us, id, hotrc.absmin_us, hotrc.us[VERT][OPMIN] - hotrc.us[VERT][MARGIN]));
            else if (sel == 14) hotrc.set(&hotrc.deadband_us, tune(hotrc.deadband_us, id, 0.0f, 100.0f));  // use hotrc.set function to force derive
        }
        else if (datapage == PG_SENS) {
            if (sel == 10) pressure.set_oplim(tune(pressure.opmin(), id, pressure.absmin(), pressure.opmax()), NAN);
            else if (sel == 11) pressure.set_oplim(NAN, tune(pressure.opmax(), id, pressure.opmin(), pressure.absmax()));
            else if (sel == 12) brkpos.set_oplim(tune(brkpos.opmin(), id, brkpos.absmin(), brkpos.opmax()), NAN);
            else if (sel == 13) brkpos.set_oplim(NAN, tune(brkpos.opmax(), id, brkpos.opmin(), brkpos.absmax()));
            else if (sel == 14) tune(brkpos.zeropoint_ptr(), id, brkpos.opmin(), brkpos.opmax());
            // else if (sel == 14) mulebatt.set_conversions(tune(mulebatt.mfactor(), id, 0.001, 0.009, -5), NAN);  // this works till you try to tune it, any edit causes conversion failures (showing nan result) followed by crash
        }
        else if (datapage == PG_PULS) {
            if (sel == 10) tach.set_oplim(tune(tach.opmin(), id, tach.absmin(), tach.opmax()), NAN);
            else if (sel == 11) tach.set_oplim(NAN, tune(tach.opmax(), id, tach.opmin(), tach.absmax()));
            else if (sel == 12) speedo.set_oplim(tune(speedo.opmin(), id, speedo.absmin(), speedo.opmax()), NAN);
            else if (sel == 13) speedo.set_oplim(NAN, tune(speedo.opmax(), id, speedo.opmin(), speedo.absmax()));
            else if (sel == 14) tune(speedo.idle_ptr(), id, speedo.opmin(), speedo.opmax());
        }                
        else if (datapage == PG_PWMS) {
            if (sel == 11) gas.set(&gas.si[OPMIN], tune(gas.si[OPMIN], id, gas.si[ABSMIN], gas.si[OPMAX]));
            else if (sel == 12) gas.set(&gas.si[OPMAX], tune(gas.si[OPMAX], id, gas.si[OPMIN], gas.si[ABSMAX]));
            else if (sel == 13) brake.set(&brake.us[STOP], tune(brake.us[STOP], id, brake.us[OPMIN], brake.us[OPMAX]));
            else if (sel == 14) brake.set(&brake.duty_fwd_pc, tune(brake.duty_fwd_pc, id, 0.0f, 100.0f));
        }
        else if (datapage == PG_IDLE) {
            if (sel == 11) tune(&gas.starting_pc, id, gas.pc[OPMIN], gas.pc[OPMAX]);
            else if (sel == 12) tune(&gas.idle_max_boost_pc, 0.0f, 100.0f);
            else if (sel == 13) tune(&gas.idle_temp_lim_f[LOW], id, tempsens.opmin(loc::ENGINE), gas.idle_temp_lim_f[HIGH]);
            else if (sel == 14) tune(&gas.idle_temp_lim_f[HIGH], id, gas.idle_temp_lim_f[LOW], tempsens.opmax(loc::ENGINE));
        }
        else if (datapage == PG_MOTR) {
            if (sel == 5) brake.update_ctrl_config((int)tune(id));
            else if (sel == 6) brake.update_ctrl_config(-1, tune(brake.feedback, id, 0, NumBrakeFB-1, true));
            else if (sel == 7) brake.update_ctrl_config(-1, -1, tune(brake.openloop_mode, id, 0, NumOpenLoopModes-1, true));
            else if (sel == 8) brake.enforce_positional_limits = tune(id);
            else if (sel == 9) tune(&brake.max_out_change_rate_pcps, id, 0.0f, 1000.0f);
            else if (sel == 10) gas.update_ctrl_config((int)tune(id));
            else if (sel == 11) gas.update_cruise_ctrl_config((int)tune(id));
            else if (sel == 12) gas.set_cruise_scheme(tune(gas.cruise_adjust_scheme, id, 0, NumCruiseSchemes-1, true));
            else if (sel == 13) tune(&cruise_brake, id);
            else if (sel == 14) tune(&drive_mode, id, FLY, CRUISE, true);
        }
        else if (datapage == PG_BPID) {
            if (sel == 11) brake.pid_dom->set_sampletime(tune(brake.pid_dom->sampletime(), id, 1000));
            else if (sel == 12) brake.pid_dom->set_kp(tune(brake.pid_dom->kp(), id, 0.0f, NAN));
            else if (sel == 13) brake.pid_dom->set_ki(tune(brake.pid_dom->ki(), id, 0.0f, NAN));
            else if (sel == 14) brake.pid_dom->set_kd(tune(brake.pid_dom->kd(), id, 0.0f, NAN));
        }
        else if (datapage == PG_GPID) {
            if (sel == 11) gas.set_changerate_deg(tune(gas.max_throttle_angular_velocity_degps, id, 0.0f, 180.0f));
            else if (sel == 12) gas.pid.set_kp(tune(gas.pid.kp(), id, 0.0f, NAN));
            else if (sel == 13) gas.pid.set_ki(tune(gas.pid.ki(), id, 0.0f, NAN));
            else if (sel == 14) gas.pid.set_kd(tune(gas.pid.kd(), id, 0.0f, NAN));
        }
        else if (datapage == PG_CPID) {
            if (sel == 11) tune(&cruise_delta_max_pc_per_s, id, 1.0, 35.0);
            else if (sel == 12) gas.cruisepid.set_kp(tune(gas.cruisepid.kp(), id, 0.0f, NAN));
            else if (sel == 13) gas.cruisepid.set_ki(tune(gas.cruisepid.ki(), id, 0.0f, NAN));
            else if (sel == 14) gas.cruisepid.set_kd(tune(gas.cruisepid.kd(), id, 0.0f, NAN));
        }
        else if (datapage == PG_TEMP) {
            if (sel == 12) tune(&wheeldifferr, id);
            else if (sel == 13) dont_take_temperatures = tune(id);
            else if (sel == 14) web_disabled = tune(id);
        }
        else if (datapage == PG_SIM) {
            screen->disp_simbuttons_dirty = true;  // any of the following will necessitate a redraw of the simbuttons
            if (sel == 4) sim.set_can_sim(sens::joy, tune(id));
            else if (sel == 5) sim.set_can_sim(sens::pressure, tune(id));
            else if (sel == 6) sim.set_can_sim(sens::brkpos, tune(id));
            else if (sel == 7) sim.set_can_sim(sens::speedo, tune(id));
            else if (sel == 8) sim.set_can_sim(sens::tach, tune(id));
            else if (sel == 9) sim.set_can_sim(sens::airvelo, tune(id));
            else if (sel == 10) sim.set_can_sim(sens::mapsens, tune(id));
            else if (sel == 11) sim.set_can_sim(sens::basicsw, tune(id));
            else if (sel == 12) sim.set_potmap((sens)(tune((int)sim.potmap(), id, 0, (int)(sens::starter) - 1, true)));
            else if (sel == 13) cal_brakemode_request = tune(id);
            else if (sel == 14) cal_gasmode_request = tune(id);
        }
        else if (datapage == PG_UI) {
            if (sel == 8) ezread.lookback(tune(ezread.offset, id, 0, ezread.bufferSize));
            else if (sel == 9) neo->set_pcba_glow(tune(neo->pcbaglow, id, 0, GlowNumModes - 1, true));
            else if (sel == 10) neo->flashdemo_ena(tune(id));  //  neo->enable_flashdemo(flashdemo); }
            else if (sel == 11) neo->sleepmode_ena(tune(id));  //  neo->enable_flashdemo(flashdemo); }
            else if (sel == 12) neo->setbright(tune(neobright, id, 0.0f, 100.0f));  //  neo->setbright(neobright); }
            else if (sel == 13) neo->setsat(tune(neosat, id, 0.0f, 100.0f));  //  neo->setbright(neobright); }
            else if (sel == 14) tune(&ui_context, id, 0, NumContextsUI-1, true);
        }
        id = 0;
    }
};
static NeopixelStrip neo(neopixel_pin);
static IdiotLights idiots;
static Touchscreen touch;
static Display screen(&neo, &touch, &idiots, &sim);
static Tuner tuner(&screen, &neo, &touch);
bool take_two_semaphores(SemaphoreHandle_t* sem1, SemaphoreHandle_t* sem2, TickType_t waittime=portMAX_DELAY) {   // pdMS_TO_TICKS(1)
    if (xSemaphoreTake(*sem1, waittime) == pdTRUE) {
        if (xSemaphoreTake(*sem2, waittime) == pdTRUE) return pdTRUE;
        xSemaphoreGive(*sem1);
    }
    return pdFALSE;
}
static void push_task(void *parameter) {
    static int lastmode;
    static bool lastpush = false;
    while (true) {
        if (runmode == LOWPOWER) {
            if (lastmode != LOWPOWER) {
                vTaskDelay(portMAX_DELAY * 2);
                lastpush = true;
            }
            else vTaskDelay(pdMS_TO_TICKS(1000));
            lastmode = runmode;
        }
        if ((runmode != LOWPOWER) || lastpush) {
            if (take_two_semaphores(&pushbuf_sem, &drawbuf_sem, portMAX_DELAY) == pdTRUE) {
                screen.do_push();
                xSemaphoreGive(pushbuf_sem);
                xSemaphoreGive(drawbuf_sem);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // vTaskDelete(NULL);
        lastpush = false;
        lastmode = runmode;
    }
}
static void draw_task(void *parameter) {
    static int lastmode;
    while (true) {
        while (runmode == LOWPOWER) {
            if (lastmode != LOWPOWER) {
                screen.blackout();
                reset_request = true;
            }
            else vTaskDelay(pdMS_TO_TICKS(1000));
            lastmode = runmode;
        }
        // if (runmode != LOWPOWER && lastmode == LOWPOWER) screen.reset();
        // if ((esp_timer_get_time() - screen_refresh_time > refresh_limit) || always_max_refresh || auto_saver_enabled) {
        if (xSemaphoreTake(drawbuf_sem, portMAX_DELAY) == pdTRUE) {
            screen_refresh_time = esp_timer_get_time();
            screen.do_draw();
            xSemaphoreGive(drawbuf_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(1));  //   || sim.enabled()
        vTaskDelay(pdMS_TO_TICKS((int)(refresh_limit / 1000 - 1)));  //   || sim.enabled()
        // if (!always_max_refresh && !auto_saver_enabled) vTaskDelay(pdMS_TO_TICKS((int)(refresh_limit / 1000 - 1)));  //   || sim.enabled()
        lastmode = runmode;
    }
}
// The following project draws a nice looking gauge cluster, very apropos to our needs and the code is given.
// See this video: https://www.youtube.com/watch?v=U4jOFLFNZBI&ab_channel=VolosProjects
// Rinkydink home page: http://www.rinkydinkelectronics.com
// moving transparent arrow sprite over background: https://www.youtube.com/watch?v=U4jOFLFNZBI&ab_channel=VolosProjects
// bar graphs: https://www.youtube.com/watch?v=g4jlj_T-nRw&ab_channel=VolosProjects