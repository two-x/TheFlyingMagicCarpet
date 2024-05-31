#pragma once
#include "animations.h"
#include "neopixel.h"

// #define disp_vshift_pix 2  // Unknown.  Note: At smallest text size, characters are 5x7 pix + pad on rt and bot for 6x8 pix.
#define disp_runmode_text_x 12
#define disp_lines 24  // Max lines of text displayable at line height = disp_line_height_pix
#define disp_fixed_lines 8  // Lines of static variables/values always displayed
#define disp_line_height_pix 10  // Pixel height of each text line. Screen can fit 16x 15-pixel or 20x 12-pixel lines
#define disp_bargraph_width 40
#define disp_bargraph_squeeze 1
#define disp_maxlength 6  // How many characters fit between the ":" and the units string
#define disp_default_float_precision 3  // Significant digits displayed for float values. Higher causes more screen draws
#define disp_datapage_names_x 12
#define disp_datapage_values_x 59
#define disp_datapage_units_x 103        
#define disp_bargraphs_x 122
#define disp_datapage_title_x 83
uint8_t colorcard[NUM_RUNMODES] = { MGT, WHT, RED, ORG, YEL, GRN, TEAL, PUR };
std::string modecard[NUM_RUNMODES] = { "Basic", "LowPwr", "Stndby", "Stall", "Hold", "Fly", "Cruise", "Cal" };
std::string side_menu_buttons[5] = { "PAG", "SEL", "+  ", "-  ", "SIM" };  // Pad shorter names with spaces on the right
std::string top_menu_buttons[4]  = { " CAL ", "BASIC", " IGN ", "POWER" };  // Pad shorter names with spaces to center
std::string sensorcard[14] = { "none", "joy", "bkpres", "brkpos", "speedo", "tach", "airflw", "mapsns", "engtmp", "batery", "startr", "basic", "ign", "syspwr" };
std::string uicontextcard[NumContextsUI] = { "ezread", "chasis", "animat" };
// These defines are just a convenience to keep the below datapage strings array initializations aligned in neat rows & cols for legibility
#define stEr "St\x88r"
#define brAk "Br\x83k"
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
static std::string telemetry[disp_fixed_lines] = { "TriggerV", "JoysticH", "   Speed", "    Tach", brAk"Pres", "Throttle", brAk"Motr", stEr"Motr", };  // Fixed rows
static std::string units[disp_fixed_lines] = { "%", "%", "mph", "rpm", "psi", "%", "%", "%" };  // Fixed rows
static std::string pagecard[datapages::NUM_DATAPAGES] = { "Run ", "Joy ", "Sens", "Puls", "PWMs", "Idle", "Motr", "Bpid", "Gpid", "Cpid", "Temp", "Sim ", "UI  " };
static constexpr int tuning_first_editable_line[datapages::NUM_DATAPAGES] = { 13, 10, 10, 10, 11, 10, 9, 12, 11, 11, 13, 4, 11 };  // first value in each dataset page that's editable. All values after this must also be editable
static std::string datapage_names[datapages::NUM_DATAPAGES][disp_tuning_lines] = {
    { brAk"Posn", "MuleBatt", "     Pot", " AirVelo", "     MAP", "MasAirFl", "Gas Mode", brAk"Mode", stEr"Mode", __________, __________, __________, __________, "Governor", stEr"Safe", },  // PG_RUN
    { "HRc Horz", "HRc Vert", "HotRcCh3", "HotRcCh4", "TrigVRaw", "JoyH Raw", __________, __________, __________, __________, "AirVOMax", "MAP OMin", "MAP OMax", horfailsaf, "Deadband", },  // PG_JOY
    { " Pot Raw", "BkPosRaw", "BkPosRaw", brAk"Posn", brAk"Posn", "PressRaw", "PressRaw", "Pressure", "Pressure", __________, "PresOmin", "PresOmax", "BPosOmin", "BPosOmax", "BPosZero", },  // PG_SENS
    { "TachPuls", "Tach Raw", "Tach Raw", spEd"Puls", "SpeedRaw", "SpeedRaw", "   Speed", "   Speed", spEd"AMin", spEd"AMax", "TachOMin", "TachOMax", spEd"OMin", spEd"OMax", spEd"Idle", },  // PG_PULS
    { "Throttle", "Throttle", brAk"Motr", brAk"Motr", stEr"Motr", stEr"Motr", __________, __________, __________, __________, __________, "ThrotCls", "ThrotOpn", brAk"Stop", brAk"Duty", },  // PG_PWMS
    { "Gas Mode", "Tach Tgt", "    Idle", "    Idle", "    Idle", "FuelPump", __________, __________, __________, __________, "StartGas", "ColdIdle", "Hot Idle", "ColdTemp", "Hot Temp", },  // PG_IDLE
    { "Brk Heat", "HybBrake", __________, __________, __________, __________, __________, __________, __________, "BkEnaPID", "BkFeedbk", "BkPosLim", "GasEnPID", "CrEnaPID", "CrAdjMod", },  // PG_MOTR
    { "MotrMode", "Feedback", brAk"Posn", "Pn|PrErr", "Pres Tgt", "Posn Tgt", "HybrdTgt", "OutRatio", "  P Term", "  I Term", "  D Term", __________, "Brake Kp", "Brake Ki", "Brake Kd", },  // PG_BPID
    { "MotrMode", "AngleTgt", "TachTarg", "Tach Err", "  P Term", "  I Term", "  D Term", __________, __________, __________, __________, "AnglVelo", "  Gas Kp", "  Gas Ki", "  Gas Kd", },  // PG_GPID
    { spEd"Targ", "SpeedErr", "  P Term", "  I Term", "  D Term", "ThrotSet", __________, __________, __________, __________, __________, maxadjrate, "Cruis Kp", "Cruis Ki", "Cruis Kd", },  // PG_CPID
    { " Ambient", "  Engine", "Wheel FL", "Wheel FR", "Wheel RL", "Wheel RR", "BrkMotor", __________, __________, __________, __________, __________, __________, "No Temps", "StopWifi", },  // PG_TEMP
    { __________, __________, __________, __________, "Joystick", brAk"Pres", brAk"Posn", "  Speedo", "    Tach", "AirSpeed", "     MAP", "Basic Sw", " Pot Map", "CalBrake", " Cal Gas", },  // PG_SIM
    { "Loop Avg", "LoopPeak", "LoopFreq", "FramRate", "HumanAct", " Touch X", " Touch Y", "  Uptime", __________, __________, __________, "BlnkDemo", neo_bright, "NeoDesat", "PanelApp", },  // PG_UI
};
static std::string tuneunits[datapages::NUM_DATAPAGES][disp_tuning_lines] = {
    { "in",   "V",    "%",    "mph",  "atm",  "g/s",  scroll, scroll, scroll, ______, ______, ______, ______, "%",    "%",    },  // PG_RUN
    { "us",   "us",   "us",   "us",   "%",    "%",    ______, ______, ______, ______, "mph",  "atm",  "atm",  "us",   "us",   },  // PG_JOY
    { "adc",  "adc",  "in",   "in",   "%",    "adc",  "psi",  "psi",  "%",    ______, "psi",  "psi",  "in",   "in",   "in",   },  // PG_SENS
    { "ms",   "Hz",   "psi",  "ms",   "Hz",   "mph",  "mph",  "%",    "ms",   "ms",   "rpm",  "rpm",  "mph",  "mph",  "mph",  },  // PG_PULS
    { degree, "us",   "V",    "us",   "V",    "us",   ______, ______, ______, ______, ______, degree, degree, "us",   "%",    },  // PG_PWMS
    { scroll, "rpm",  "%",    degree, "rpm",  "V",    ______, ______, ______, ______, "%",    degree, degree, degreF, degreF, },  // PG_IDLE
    { degreF, "%",    ______, ______, ______, ______, ______, ______, ______, b1nary, scroll, b1nary, b1nary, b1nary, scroll, },  // PG_MOTR
    { scroll, scroll, "in",   "psin", "psin", "%",    "%",    "%",    "%",    "%",    "%",    ______, ______, "Hz",   "s",    },  // PG_BPID
    { scroll, "%",    "rpm",  "rpm",  "%",    "%",    "%",    ______, ______, ______, ______, degsec, ______, "Hz",   "s",    },  // PG_GPID
    { "mph",  "mph",  "rpm",  "rpm",  "rpm",  "%",    ______, ______, ______, ______, ______, "%/s",  ______, "Hz",   "s",    },  // PG_CPID
    { degreF, degreF, degreF, degreF, degreF, degreF, degreF, ______, ______, ______, ______, ______, ______, b1nary, b1nary, },  // PG_TEMP
    { ______, ______, ______, ______, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, b1nary, scroll, b1nary, b1nary, },  // PG_SIM
    { "us",   "us",   "Hz",   "fps",  scroll, "pix",  "pix",  "min",  ______, ______, ______, "eyes", "%",    "of10", scroll, },  // PG_UI
};
static std::string unitmapnames[11] = { "usps", "us", "rpms", scroll, b1nary, "%", "ohm", "eyes", "psin", degree, "of10" };  // unit strings matching these will get replaced by the corresponding bitmap graphic below
static constexpr uint8_t unitmaps[11][17] = {  // 17x7-pixel bitmaps for exceptions where some unit strings can't be well represented by any set of 3 font characters
    { 0x7e, 0x20, 0x20, 0x3c, 0x00, 0x24, 0x2a, 0x2a, 0x12, 0x00, 0x70, 0x0e, 0x00, 0x24, 0x2a, 0x2a, 0x12, },  // usps - microseconds per second
    { 0x40, 0x7e, 0x20, 0x20, 0x1c, 0x20, 0x00, 0x24, 0x2a, 0x2a, 0x2a, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, },  // us - b/c the font's lowercase mu character doesn't work
    { 0x1f, 0x01, 0x00, 0x3f, 0x09, 0x0e, 0x00, 0x0f, 0x01, 0x0e, 0x01, 0x0e, 0x60, 0x1c, 0x00, 0x58, 0x74, },  // rpm/s (or rot/m*s) - rate of change of engine rpm
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x7f, 0x02, 0x04, 0x00, 0x10, 0x20, 0x7f, 0x20, 0x10, 0x00, },  // scroll arrows - to indicate multiple choices
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x1c, 0x22, 0x22, 0x1c, 0x00, 0x00, },  // 0/1 - to indicate binary value
    { 0x02, 0x45, 0x25, 0x12, 0x08, 0x24, 0x52, 0x51, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // % - we use this a lot and the font % looks feeble
    { 0x4e, 0x51, 0x61, 0x01, 0x61, 0x51, 0x4e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },  // capital omega - for ohms
    { 0x41, 0x22, 0x22, 0x00, 0x00, 0x3e, 0x63, 0x63, 0x77, 0x7f, 0x41, 0x3e, 0x63, 0x63, 0x77, 0x7f, 0x3e, },  // googly eyes, are as goofy as they are stupid
    { 0x3d, 0x00, 0x3e, 0x02, 0x3c, 0x00, 0x7f, 0x00, 0x3e, 0x12, 0x0c, 0x00, 0x2c, 0x2a, 0x1a, 0x00, 0x3d, },  // inches or psi "in|psi" - that's right SIX characters in a 3 font-width space, haha.  get a microscope
    { 0x06, 0x0f, 0x09, 0x4f, 0x66, 0x50, 0x48, 0x4c, 0x72, 0x41, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, },  // angular degrees
    { 0x30, 0x48, 0x48, 0x30, 0x00, 0x10, 0x7c, 0x12, 0x04, 0x00, 0x02, 0x7f, 0x00, 0x3e, 0x51, 0x49, 0x3e, },  // of 10
};  // These bitmaps are in the same format as the idiot light bitmaps, described in neopixel.h
//  { 0x7e, 0x20, 0x3e, 0x20, 0x00, 0x0c, 0x52, 0x4a, 0x3c, 0x00, 0x60, 0x18, 0x06, 0x00, 0x2c, 0x2a, 0x32, },  // ug/s - for manifold mass airflow
static EZReadDrawer ezdraw(&ezread);
static PanelAppManager panel(&ezdraw);
volatile float fps = 0.0;
volatile bool is_pushing = 0;
volatile bool is_drawing = 0;
volatile int pushclock;
volatile int drawclock;
volatile int idleclock;
volatile bool reset_request = false;
volatile bool pushtime = 0;
volatile bool drawn = false;
volatile bool pushed = true;

SemaphoreHandle_t pushbuf_sem;  // StaticSemaphore_t push_semaphorebuf_sem;
SemaphoreHandle_t drawbuf_sem;  // StaticSemaphore_t draw_semaphorebuf_sem;
static void push_task_wrapper(void *parameter);
static void draw_task_wrapper(void *parameter);
void semaphore_setup() {
    Serial.printf("Semaphores..");
    pushbuf_sem = xSemaphoreCreateBinary();  // StaticSemaphore_t push_semaphorebuf_sem;
    drawbuf_sem = xSemaphoreCreateBinary();  // StaticSemaphore_t draw_semaphorebuf_sem;
    if (pushbuf_sem == NULL || drawbuf_sem == NULL) Serial.printf(" creation failed");
    else {
        xSemaphoreGive(pushbuf_sem);
        xSemaphoreGive(drawbuf_sem);
    }
    Serial.printf("\n");
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
    Timer valuesRefreshTimer = Timer(160000);  // Don't refresh screen faster than this (16667us = 60fps, 33333us = 30fps, 66666us = 15fps)
    uint8_t palettesize = 2;
    // uint16_t palette[256] = { BLK, WHT };
    static constexpr int runOnCore = CONFIG_ARDUINO_RUNNING_CORE == 0 ? 1 : 0;
    Timer dispAgeTimer[disp_lines];  // int disp_age_timer_us[disp_lines];
    static constexpr int idiots_corner_x = 165;
    static constexpr int idiots_corner_y = 13;
    bool sim_last = false, fullscreen_last = false;
    int runmode_last = -1;
  public:
    std::string disp_values[disp_lines];  // Holds previously drawn value strings for each line
    volatile bool disp_bool_values[6];
    int disp_datapage_last;
    volatile bool disp_bargraphs[disp_lines];
    volatile bool disp_polarities[disp_lines];  // Holds sign of previously drawn values
    volatile int disp_needles[disp_lines];
    volatile int disp_targets[disp_lines];
    volatile int disp_age_quanta[disp_lines];
    volatile uint8_t disp_val_colors[disp_lines];
    volatile bool disp_selected_val_dirty;
    volatile bool disp_datapage_dirty;
    volatile bool disp_values_dirty;
    volatile bool disp_data_dirty[disp_lines];
    volatile bool disp_bools_dirty;
    volatile bool disp_sidemenu_dirty;
    volatile bool disp_runmode_dirty;
    volatile bool disp_simbuttons_dirty;
    volatile bool disp_idiots_dirty;
    Display(NeopixelStrip* _neo, Touchscreen* _touch, IdiotLights* _idiots, Simulator* _sim)
      : neo(_neo), touch(_touch), idiots(_idiots), sim(_sim) {}
    void init_framebuffers(int _sprwidth, int _sprheight) {
        int sprsize[2] = { _sprwidth, _sprheight };
        Serial.printf("  create frame buffers.. ");
        lcd.setColorDepth(sprite_color_depth);
        for (int i = 0; i <= 1; i++) framebuf[i].setColorDepth(8);  // color_depth_t::rgb332_1Byte = 8  Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
        auto framewidth = sprsize[HORZ];
        auto frameheight = sprsize[VERT];
        bool fail = false;
        bool using_psram = false;
        for (std::uint32_t i = 0; !fail && i < 2; ++i) {
            framebuf[i].setPsram(false);
            fail = !framebuf[i].createSprite(framewidth, frameheight);
        }
        if (fail) {
            fail = false;
            for (std::uint32_t i = 0; !fail && i < 2; ++i) {
                framebuf[i].setPsram(true);
                fail = !framebuf[i].createSprite(framewidth, frameheight);
            }
            if (fail) {
                fail = false;
                if (framewidth >= 320) framewidth = 180;
                if (frameheight >= 240) frameheight = 180;
                for (std::uint32_t i = 0; !fail && i < 2; ++i) {
                    fail = !framebuf[i].createSprite(framewidth, frameheight);
                }
                if (fail) {
                    lcd.print("createSprite fail\n");
                }
                else using_psram = true;
            }
            else using_psram = true;
        }
        Serial.printf(" made 2x %dx%d sprites in %sram\n", framewidth, frameheight, using_psram ? "ps" : "native ");
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
        Serial.printf("Display..");  //
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
        panel.setup(&lcd, sim, touch, disp_simbuttons_x, disp_simbuttons_y, disp_simbuttons_w, disp_simbuttons_h);
        sprptr = &framebuf[flip];
        Serial.printf("  display initialized\n");
    }
    void reset(LGFX_Sprite* spr) {
        blackout(spr);
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
        disp_bools_dirty = disp_selected_val_dirty = disp_datapage_dirty = disp_sidemenu_dirty = true;
        disp_runmode_dirty = disp_simbuttons_dirty = disp_values_dirty = true;
        ui_context = ui_default;
    }
    void blackout(LGFX_Sprite* spr) {
        spr->fillSprite(BLK);
        // std::uint32_t* s;
        // for (int f=0; f<2; f++) {
        //     s = (std::uint32_t*)(*spr).getBuffer();
        //     for (int w=0; w<(spr->width() * spr->height() / 4); w++) s[w] = 0x00000000;
        // }
    }
    void set_runmodecolors() {
        uint8_t saturat = 255;  uint8_t hue_offset = 0;
        for (int32_t rm=0; rm<NUM_RUNMODES; rm++) {
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
        sprptr->drawFastHLine(corner_x + disp_bargraph_squeeze, corner_y, width - disp_bargraph_squeeze*2, MGRY);  // base line
        sprptr->drawFastVLine(corner_x + width/2, corner_y-1, 2, WHT);  // centerpoint gradient line
        for (int offset=0; offset<=2; offset+=2) sprptr->drawFastVLine((corner_x + disp_bargraph_squeeze) + offset * (width/2 - disp_bargraph_squeeze), corner_y-2, 3, WHT);  // endpoint gradient lines
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
    void draw_string(int x, int y, std::string text, std::string oldtext, uint8_t color, uint8_t bgcolor, bool forced=false) {  // Send in "" for oldtext if erase isn't needed
        if ((text == oldtext) && !forced) return; 
        // sprptr->fillRect(x_old, y, oldtext.length() * disp_font_width, disp_font_height, bgcolor);
        sprptr->setTextColor(bgcolor);
        sprptr->setCursor(x, y);
        sprptr->print(oldtext.c_str());
        sprptr->setTextColor(color);
        sprptr->setCursor(x, y);
        sprptr->print(text.c_str());
    }
    void draw_unitmap(int index, int x, int y, uint8_t color) {
        for (int xo = 0; xo < disp_font_width * 3 - 1; xo++)
            for (int yo = 0; yo < disp_font_height - 1; yo++)
                if ((unitmaps[index][xo] >> yo) & 1) sprptr->drawPixel(x + xo, y + yo, color);
    }
    void draw_string_units(int x, int y, std::string text, std::string oldtext, uint8_t color, uint8_t bgcolor) {  // Send in "" for oldtext if erase isn't needed
        sprptr->fillRect(x, y, 3 * disp_font_width, disp_font_height, bgcolor);
        for (int i = 0; i<arraysize(unitmaps); i++) {
            if (unitmapnames[i] == text) {
                draw_unitmap(i, x, y, color);
                return;
            }
        }
        sprptr->setCursor(x, y);
        sprptr->setTextColor(color);
        sprptr->print(text.c_str());
    }
    // draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
    void draw_fixed(int page, int page_last, bool redraw_all, bool forced=false) {  // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
        static int fixed_page_last;
        sprptr->setTextColor(LGRY);
        sprptr->setTextSize(1);
        int y_pos;
        for (int lineno = 0; lineno < disp_fixed_lines; lineno++) {  // Step thru lines of fixed telemetry data
            y_pos = (lineno + 1) * disp_line_height_pix;
            draw_string(disp_datapage_names_x, y_pos, telemetry[lineno], nulstr, LGRY, BLK, forced);
            draw_string_units(disp_datapage_units_x, y_pos, units[lineno], nulstr, LGRY, BLK);
        }
        if (redraw_all) {
            for (int lineno = 0; lineno < disp_tuning_lines; lineno++) {  // Step thru lines of fixed telemetry data
                y_pos = (lineno + disp_fixed_lines + 1) * disp_line_height_pix;
                // int index = lineno - disp_fixed_lines - 1;
                // Serial.printf("drawing line:%d x:%d y:%d text:%s\n", index, disp_datapage_names_x, y_pos, datapage_names[page][index].c_str() );
                draw_string(disp_datapage_names_x, y_pos, datapage_names[page][lineno], datapage_names[page_last][lineno], LGRY, BLK, forced);
                draw_string_units(disp_datapage_units_x, y_pos, tuneunits[page][lineno], tuneunits[page_last][lineno], LGRY, BLK);
                
                // commenting these two lines doesn't seem to mess up the rendering of the bargraphs when i flip thru the datapages ... so what are they for?!
                // sprptr->fillRect(disp_bargraphs_x - 1, (lineno + disp_fixed_lines + 1) * disp_line_height_pix, disp_bargraph_width + 2, 4, BLK);
                // if (disp_needles[index] >= 0) draw_bargraph_needle(-1, disp_needles[index], y_pos + 1, BLK);  // Let's draw a needle
                disp_bargraphs[lineno] = false;
            }
        }
    }
    void draw_hyphen(int x_pos, int y_pos, uint8_t color) {  // Draw minus sign in front of negative numbers
        sprptr->drawFastHLine(x_pos+2, y_pos+3, 3, color);
    }
    void draw_dynamic(int lineno, std::string disp_string, int value, int lowlim, int hilim, int target=-1, uint8_t color=NON) {
        int age_us = (color != NON) ? 11 : (int)((float)(dispAgeTimer[lineno].elapsed()) / 2500000); // Divide by us per color gradient quantum
        int x_base = disp_datapage_values_x;
        bool polarity = (value >= 0);  // polarity 0=negative, 1=positive
        bool force = (value == 1234567) || disp_data_dirty[lineno];
        if ((disp_values[lineno] != disp_string) || force) {  // If value differs, Erase old value and write new
            if (color == NON) color = GRN;
            int y_pos = lineno*disp_line_height_pix;
            if (polarity != disp_polarities[lineno]) draw_hyphen(x_base, y_pos, (!polarity) ? color : BLK);
            draw_string(x_base+disp_font_width, y_pos, disp_string, disp_values[lineno], color, BLK, force || (color != disp_val_colors[lineno])); // +6*(arraysize(modecard[run.mode])+4-namelen)/2
            disp_values[lineno] = disp_string;
            disp_polarities[lineno] = polarity;
            disp_val_colors[lineno] = color;
            dispAgeTimer[lineno].reset();
            disp_age_quanta[lineno] = 0;
        }
        else if (age_us > disp_age_quanta[lineno] && age_us < 11)  {  // As readings age, redraw in new color. This may fail and redraw when the timer overflows? 
            if (age_us < 8) color = 0x1c + (age_us << 5);  // Base of green with red added as you age, until yellow is achieved
            else color = 0xfc - ((age_us-8) << 2);  // Then lose green as you age further
            int y_pos = (lineno)*disp_line_height_pix;
            if (!polarity) draw_hyphen(x_base, y_pos, color);
            draw_string(x_base+disp_font_width, y_pos, disp_values[lineno], "", color, BLK);
            disp_age_quanta[lineno] = age_us;
            disp_val_colors[lineno] = color;
        }
        bool delete_bargraph = false;
        int corner_x = disp_bargraphs_x;
        int corner_y = lineno*disp_line_height_pix;
        if (lowlim < hilim) {  // Any value having a given range deserves a bargraph gauge with a needle
            int n_pos = map(value, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
            uint8_t ncolor = (n_pos > disp_bargraph_width-disp_bargraph_squeeze || n_pos < disp_bargraph_squeeze) ? RED : GRN;
            n_pos = corner_x + constrain(n_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
            if (!disp_bargraphs[lineno]) draw_bargraph_base(corner_x, corner_y + 8, disp_bargraph_width);
            disp_bargraphs[lineno] = true;
            draw_target_shape(disp_targets[lineno], corner_y, BLK, NON);  // Erase old target
            draw_bargraph_needle(n_pos, disp_needles[lineno], corner_y, ncolor);  // Let's draw a needle
            disp_needles[lineno] = n_pos;  // Remember position of needle
            if (target != -1) {  // If target value is given, draw a target on the bargraph too
                int t_pos = map(target, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                uint8_t tcolor = (t_pos > disp_bargraph_width-disp_bargraph_squeeze || t_pos < disp_bargraph_squeeze) ? RED : ( (t_pos != n_pos) ? YEL : GRN );
                t_pos = corner_x + constrain(t_pos, disp_bargraph_squeeze, disp_bargraph_width-disp_bargraph_squeeze);
                draw_target_shape(t_pos, corner_y, tcolor, NON);  // Draw the new target
                disp_targets[lineno] = t_pos;  // Remember position of target
            }
        }
        else delete_bargraph = true;
        if (delete_bargraph || value == 1234567) {
            sprptr->fillRect(corner_x - 1, corner_y, disp_bargraph_width + 2, disp_line_height_pix, BLK);
            disp_bargraphs[lineno] = false;
        }
        disp_data_dirty[lineno] = false;
    }
    void draw_dynamic(int lineno, int value, int lowlim=-1, int hilim=-1, int target=-1) {
        std::string val_string = num2string(value, (int)disp_maxlength);
        draw_dynamic(lineno, val_string.c_str(), value, lowlim, hilim, (int)target);
    }
    void draw_dynamic(int lineno, float value, float lowlim=-1, float hilim=-1, int target=-1, int precision = disp_default_float_precision) {
        std::string val_string = num2string(value, (int)disp_maxlength, precision);
        draw_dynamic(lineno, val_string.c_str(), (int)value, (int)lowlim, (int)hilim, target);
    }
    void draw_dynamic(int lineno, float value, float lowlim, float hilim, float target, int precision = disp_default_float_precision) {
        draw_dynamic(lineno, value, lowlim, hilim, (int)target, precision);
    }
    void draw_eraseval(int lineno) {
        draw_dynamic(lineno, "", 1234567, -1, -1, -1);
    }
    void draw_asciiname(int lineno, std::string name) {
        draw_dynamic(lineno, name.c_str(), 1, -1, -1, -1, CYN);
    }
    void draw_truth(int lineno, bool truthy, int styl=2) {  // 0:on/off, 1:yes/no, 2:true/false .
        draw_dynamic(lineno, (truthy) ? ((styl==0) ? "on" : ((styl==1) ? "yes" : "true")) : ((styl==0) ? "off" : ((styl==1) ? "no" : "false")), 1, -1, -1, -1, (truthy) ? LPUR : GPUR);
    }
    int significant_place(float value) {  // Returns the decimal place of the most significant digit of a positive float value, without relying on logarithm math
        int place = 1;
        if (value >= 1) { // int vallog = std::log10(value);  // Can be sped up
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
    int significant_place(int value) {  // Returns the length in digits of a positive integer value
        int place = 1;
        while (value >= 10) {
            value /= 10;
            place++;
        }
        return place;
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
    std::string num2string(float value, int maxlength, int sigdig, bool chop_zeroes = true) {  // returns an ascii string representation of a given float value, formatted efficiently. It will not exceed maxlength. fractional digits will be removed respecting given number of significant digits
        value = abs(value);  // This function disregards sign
        int place = significant_place(value);  // Learn decimal place of the most significant digit in value
        if (place >= sigdig && place <= maxlength) {  // Then we want simple cast to an integer w/o decimal point (eg 123456, 12345, 1234)
            std::string result(std::to_string((int)value));
            return result;
        }
        if (place >= 0 && place < maxlength) {  // Then we want float formatted with enough nonzero digits after the decimal point for given significant digits (eg 123.4, 12.34, 1.234, 0.000)
            int length = std::min(sigdig+1, maxlength);
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
            if (decimalPos != std::string::npos) result = result.substr(decimalPos, std::min(sigdig-place, maxlength));  // Remove any digits to the left of the decimal point
            return result;
        }  // Otherwise we want scientific notation with precision removed as needed to respect maxlength (eg 1.23e4, 1.23e5, but using long e character not e for negative exponents
        char buffer[maxlength+1];  // Allocate buffer with the maximum required size
        int truncit = std::min(sigdig - 1, maxlength - 4 - (int)(place <= -10 || place >= 10));
        std::snprintf(buffer, sizeof(buffer), "%.*e", truncit, value);
        std::string result(buffer);  // copy buffer to result
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
        disp_datapage_dirty = false;
        prefs.putUInt("dpage", (uint32_t)page);
    }
    void draw_selected_name(int tun_ctrl, int selected_val, int selected_last, int selected_last_last) {
        static int last_selected; 
        uint8_t color = LGRY;
        if (tun_ctrl == EDIT) color = GRN;
        else if (tun_ctrl == SELECT) color = YEL;
        draw_string(12, (last_selected + disp_fixed_lines + 1) * disp_line_height_pix, datapage_names[datapage][last_selected], nulstr, LGRY, BLK, true);
        draw_string(12, (selected_val + disp_fixed_lines + 1) * disp_line_height_pix, datapage_names[datapage][selected_val], nulstr, color, BLK, true);
        last_selected = selected_val;
        disp_selected_val_dirty = false;    
    }
    void draw_bool(bool value, int col, bool force=false) {  // Draws values of boolean data
        if ((disp_bool_values[col-2] != value) || force) {  // If value differs, Erase old value and write new
            int x_mod = touch_margin_h_pix + touch_cell_h_pix*(col) + (touch_cell_h_pix>>1) - top_menu_buttons[col-2].length()*(disp_font_width>>1) + 1;
            sprptr->setTextDatum(textdatum_t::top_left);
            sprptr->setFont(&fonts::Font0);
            sprptr->setTextColor((value) ? YEL : LGRY);  
            sprptr->drawString(top_menu_buttons[col-2].c_str(), x_mod, 0);
            disp_bool_values[col-2] = value;
        }
    }
    void draw_touchgrid(bool side_only = false) {  // draws edge buttons with names in 'em. If replace_names, just updates names
        sprptr->setTextDatum(textdatum_t::top_left);
        int namelen = 0;
        sprptr->setTextColor(LGRY);
        for (int row = 0; row < arraysize(side_menu_buttons); row++) {  // Step thru all rows to draw buttons along the left edge
            sprptr->fillRoundRect(-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, DGRY);
            sprptr->drawRoundRect(-9, touch_cell_v_pix*row+3, 18, touch_cell_v_pix-6, 8, LGRY);
            namelen = 0;
            for (int x = 0 ; x < side_menu_buttons[row].length() ; x++ ) {
                if (side_menu_buttons[row][x] != ' ') namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters 
            }
            for (int letter = 0; letter < namelen; letter++) {  // Going letter by letter thru each button name so we can write vertically 
                sprptr->setCursor(1, (touch_cell_v_pix*row) + (touch_cell_v_pix/2) + (disp_font_height + 1) * (letter - (namelen >> 1)) - 3); // adjusts vertical offset depending how many letters in the button name and which letter we're on
                sprptr->print(side_menu_buttons[row][letter]);  // Writes each letter such that the whole name is centered vertically on the button
            }
        }
        if (!side_only) {
            for (int col = 2; col <= 5; col++) {  // Step thru all cols to draw buttons across the top edge
                sprptr->fillRoundRect(touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, DGRY);
                sprptr->drawRoundRect(touch_margin_h_pix + touch_cell_h_pix*(col) + 3, -9, touch_cell_h_pix-6, 18, 8, LGRY);  // sprptr->width()-9, 3, 18, (sprptr->height()/5)-6, 8, LGRY);
            }
        }
        disp_sidemenu_dirty = false;
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
    void draw_idiotbitmap(int i, int x, int y) {
        uint8_t bg = idiots->val(i) ? idiots->color[ON][i] : BLK;
        uint8_t color = idiots->val(i) ? BLK : idiots->color[OFF][i];
        sprptr->drawRoundRect(x, y, 2 * disp_font_width + 1, disp_font_height + 1, 1, bg);
        for (int xo = 0; xo < (2 * disp_font_width - 1); xo++)
            for (int yo = 0; yo < disp_font_height - 1; yo++)
                sprptr->drawPixel(x + xo + 1, y + yo + 1, ((idiots->icon[i][xo] >> yo) & 1) ? color : bg);
    }
    void draw_temperature(loc location, int draw_index) {
        if (!tempsens.detected(location)) draw_eraseval(draw_index);
        else draw_dynamic(draw_index, tempsens.val(location), temp_lims_f[tempsens.errclass(location)][DISP_MIN], temp_lims_f[tempsens.errclass(location)][DISP_MAX]);
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
        for (int i = 0; i < idiots->iconcount; i++) {
            if (i <= neo->neopixelsAvailable()) {
                if (idiots->val(i) != idiots->last[i]) neo->setBoolState(i, idiots->val(i));
            }
            if (i == LOST || i == RANGE) {
                if (diag.most_critical_last[i] != diag.most_critical_sensor[i]) {
                    if (diag.most_critical_sensor[i] == _None) neo->setflash((int)i, 0);
                    else neo->setflash((int)i, diag.most_critical_sensor[i] + 1, 2, 6, 1, 0);
                }
                diag.most_critical_last[i] = diag.most_critical_sensor[i];
            }
            if (force || (idiots->val(i) ^ idiots->last[i])) {
                draw_idiotlight(i, idiots_corner_x + (2 * disp_font_width + 2) * (i % idiots->row_count), idiots_corner_y + idiots->row_height * (int)(i / idiots->row_count));
            }
        }
        disp_idiots_dirty = false;
    }
    void disp_menu_bools() {
        // if (!disp_bools_dirty) return;
        draw_bool((run.mode == CAL), 2, disp_bools_dirty);
        draw_bool((run.mode == BASIC), 3, disp_bools_dirty);
        draw_bool(ignition.signal, 4, disp_bools_dirty);
        draw_bool(syspower, 5, disp_bools_dirty);
        disp_bools_dirty = false;
    }
    void disp_datapage_values() {
        // if (!disp_values_dirty) return;
        float drange;
        draw_dynamic(1, hotrc.pc[VERT][FILT], hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
        draw_dynamic(2, hotrc.pc[HORZ][FILT], hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
        draw_dynamic(3, speedo.val(), 0.0f, speedo.opmax(), gas.cruisepid.target());
        draw_dynamic(4, tach.val(), 0.0f, tach.opmax(), gas.pid.target());
        draw_dynamic(5, pressure.val(), pressure.opmin(), pressure.opmax(), brake.pids[PressureFB].target());  // (brake_active_pid == S_PID) ? (int)brakeSPID.targ() : pressure_target_adc);
        draw_dynamic(6, gas.pc[OUT], gas.pc[OPMIN], gas.pc[OPMAX], gas.throttle_target_pc);
        draw_dynamic(7, brake.pc[OUT], brake.pc[OPMIN], brake.pc[OPMAX]);
        draw_dynamic(8, steer.pc[OUT], steer.pc[OPMIN], steer.pc[OPMAX]);
        if (datapage == PG_RUN) {
            draw_dynamic(9, brkpos.val(), brkpos.opmin(), brkpos.opmax());
            draw_dynamic(10, mulebatt.val(), mulebatt.opmin(), mulebatt.opmax());
            draw_dynamic(11, pot.val(), pot.opmin(), pot.opmax());
            draw_dynamic(12, airvelo.val(), airvelo.opmin(), airvelo.opmax());
            draw_dynamic(13, mapsens.val(), mapsens.opmin(), mapsens.opmax());
            draw_dynamic(14, maf_gps, maf_min_gps, maf_max_gps);
            draw_asciiname(15, motormodecard[gas.motormode]);
            draw_asciiname(16, motormodecard[brake.motormode]);
            draw_asciiname(17, motormodecard[steer.motormode]);
            for (int line=18; line<=21; line++) draw_eraseval(line);
            draw_dynamic(22, gas.governor, 0.0f, 100.0f);
            draw_dynamic(23, steer.steer_safe_pc, 0.0f, 100.0f);
        }
        else if (datapage == PG_JOY) {
            draw_dynamic(9, hotrc.us[HORZ][RAW], hotrc.us[HORZ][OPMIN], hotrc.us[HORZ][OPMAX]);
            draw_dynamic(10, hotrc.us[VERT][RAW], hotrc.us[VERT][OPMIN], hotrc.us[VERT][OPMAX]);
            draw_dynamic(11, hotrc.us[CH3][RAW], hotrc.us[CH3][OPMIN], hotrc.us[CH3][OPMAX]);
            draw_dynamic(12, hotrc.us[CH4][RAW], hotrc.us[CH4][OPMIN], hotrc.us[CH4][OPMAX]);
            draw_dynamic(13, hotrc.pc[HORZ][RAW], hotrc.pc[HORZ][OPMIN], hotrc.pc[HORZ][OPMAX]);
            draw_dynamic(14, hotrc.pc[VERT][RAW], hotrc.pc[VERT][OPMIN], hotrc.pc[VERT][OPMAX]);
            for (int line=15; line<=18; line++) draw_eraseval(line);
            draw_dynamic(19, airvelo.opmax(), airvelo.absmin(), airvelo.absmax());
            draw_dynamic(20, mapsens.opmin(), mapsens.absmin(), mapsens.absmax());
            draw_dynamic(21, mapsens.opmax(), mapsens.absmin(), mapsens.absmax());
            draw_dynamic(22, hotrc.failsafe_us, hotrc.absmin_us, hotrc.us[VERT][OPMIN] - hotrc.us[VERT][MARGIN]);
            draw_dynamic(23, hotrc.deadband_us, 0, 100);
        }
        else if (datapage == PG_SENS) {
            draw_dynamic(9, pot.native(), pot.opmin_native(), pot.opmax_native());
            draw_dynamic(10, brkpos.native(), brkpos.opmin_native(), brkpos.opmax_native());
            draw_dynamic(11, brkpos.raw(), brkpos.opmin(), brkpos.opmax());
            draw_dynamic(12, brkpos.val(), brkpos.opmin(), brkpos.opmax());
            draw_dynamic(13, brkpos.pc(), 0.0, 100.0);
            draw_dynamic(14, pressure.native(), pressure.opmin_native(), pressure.opmax_native());
            draw_dynamic(15, pressure.raw(), pressure.opmin(), pressure.opmax());
            draw_dynamic(16, pressure.val(), pressure.opmin(), pressure.opmax());
            draw_dynamic(17, pressure.pc(), 0.0, 100.0);
            draw_eraseval(18);
            draw_dynamic(19, pressure.opmin(), pressure.absmin(), pressure.absmax());
            draw_dynamic(20, pressure.opmax(), pressure.absmin(), pressure.absmax());
            draw_dynamic(21, brkpos.opmin(), brkpos.absmin(), brkpos.absmax());
            draw_dynamic(22, brkpos.opmax(), brkpos.absmin(), brkpos.absmax());
            draw_dynamic(23, brkpos.zeropoint(), brkpos.opmin(), brkpos.opmax());  // BrakePositionSensor::absmin_retract_in, BrakePositionSensor::absmax_extend_in);
        }
        else if (datapage == PG_PULS) {
            draw_dynamic(9, tach.ms(), tach.absmin_ms(), tach.absmax_ms());
            draw_dynamic(10, tach.native(), tach.opmin_native(), tach.opmax_native());
            draw_dynamic(11, tach.raw(), tach.opmin(), tach.opmax());
            draw_dynamic(12, speedo.ms(), speedo.absmin_ms(), speedo.absmax_ms());
            draw_dynamic(13, speedo.native(), speedo.opmin_native(), speedo.opmax_native());
            draw_dynamic(14, speedo.raw(), speedo.opmin(), speedo.opmax());
            draw_dynamic(15, speedo.val(), speedo.opmin(), speedo.opmax());
            draw_dynamic(16, speedo.pc(), 0.0, 100.0);
            draw_dynamic(17, speedo.absmin_ms());
            draw_dynamic(18, speedo.absmax_ms());
            draw_dynamic(19, tach.opmin(), tach.absmin(), tach.absmax());
            draw_dynamic(20, tach.opmax(), tach.absmin(), tach.absmax());
            draw_dynamic(11, speedo.opmin(), speedo.absmin(), speedo.absmax());
            draw_dynamic(22, speedo.opmax(), speedo.absmin(), speedo.absmax());
            draw_dynamic(23, speedo.idle(), speedo.opmin(), speedo.opmax());
            // for (int myline=9; myline<=18; myline++) draw_eraseval(myline);
        }
        else if (datapage == PG_PWMS) {
            draw_dynamic(9, gas.deg[OUT], gas.deg[OPMIN], gas.deg[OPMAX]);
            draw_dynamic(10, gas.us[OUT], gas.us[ABSMIN], gas.us[ABSMAX]);
            draw_dynamic(11, brake.volt[OUT], brake.volt[OPMIN], brake.volt[OPMAX]);
            draw_dynamic(12, brake.us[OUT], brake.us[ABSMIN], brake.us[ABSMAX]);
            draw_dynamic(13, steer.volt[OUT], steer.volt[OPMIN], steer.volt[OPMAX]);
            draw_dynamic(14, steer.us[OUT], steer.us[ABSMIN], steer.us[ABSMAX]);
            for (int line=15; line<=19; line++) draw_eraseval(line);
            draw_dynamic(20, gas.deg[OPMIN], gas.deg[ABSMAX], gas.deg[ABSMAX]);
            draw_dynamic(21, gas.deg[OPMAX], gas.deg[ABSMAX], gas.deg[ABSMAX]);
            draw_dynamic(22, brake.us[STOP], brake.us[ABSMIN], brake.us[ABSMAX]);
            draw_dynamic(23, brake.duty_fwd_pc, 0.0f, 100.0f);
        }
        else if (datapage == PG_IDLE) {
            draw_asciiname(9, motormodecard[gas.motormode]);
            draw_dynamic(10, gas.pid.target(), tach.opmin(), tach.opmax());
            draw_dynamic(11, gas.idle_pc, gas.pc[OPMIN], gas.pc[OPMAX]);
            draw_dynamic(12, gas.idle_si[OUT], gas.si[OPMIN], gas.si[OPMAX]);
            draw_dynamic(13, tach.idle(), tach.opmin(), tach.opmax());
            draw_dynamic(14, fuelpump.volts, fuelpump.off_v, fuelpump.on_max_v);
            for (int line=15; line<=18; line++) draw_eraseval(line);
            draw_dynamic(19, gas.starting_pc, gas.pc[OPMIN], gas.pc[OPMAX]);
            draw_dynamic(20, gas.idle_si[OPMAX], gas.idle_si[ABSMIN], gas.idle_si[ABSMAX], -1, 4);
            draw_dynamic(21, gas.idle_si[OPMIN], gas.idle_si[ABSMIN], gas.idle_si[ABSMAX], -1, 4);
            draw_dynamic(22, gas.idletemp_f[OPMIN], gas.idletemp_f[ABSMIN], gas.idletemp_f[ABSMAX]); //  gas.idletemp_f[ABSMIN], gas.idletemp_f[ABSMAX], -1, 4);
            draw_dynamic(23, gas.idletemp_f[OPMAX], gas.idletemp_f[ABSMIN], gas.idletemp_f[ABSMAX]); // gas.idletemp_f[ABSMIN], gas.idletemp_f[ABSMAX], -1, 4); 
        }
        else if (datapage == PG_MOTR) {
            draw_dynamic(9, brake.motorheat(), brake.motorheatmin(), brake.motorheatmax());  // brake_spid_speedo_delta_adc, -range, range);
            draw_dynamic(10, brake.combined_read_pc, 0.0, 100.0);  // brake_spid_speedo_delta_adc, -range, range);
            for (int myline=11; myline<=17; myline++) draw_eraseval(myline);
            draw_truth(18, brake.pid_enabled, 1);
            draw_asciiname(19, brakefeedbackcard[brake.feedback]);
            draw_truth(20, brake.enforce_positional_limits, 1);
            draw_truth(21, gas.pid_enabled, 1);
            draw_truth(22, gas.cruise_pid_enabled, 1);
            draw_asciiname(23, cruiseschemecard[gas.cruise_adjust_scheme]);
        }
        else if (datapage == PG_BPID) {
            drange = brake.us[ABSMIN]-brake.us[ABSMAX];
            draw_asciiname(9, motormodecard[brake.motormode]);
            draw_asciiname(10, brakefeedbackcard[brake.feedback]);
            draw_dynamic(11, brkpos.val(), brkpos.opmin(), brkpos.opmax(), brake.pids[_BrakePosn].target());
            draw_dynamic(12, brake.pid_dom->err(), -brake.sensmax(), brake.sensmax());
            draw_dynamic(13, brake.target[PressureFB], 0.0f, 100.0f);  // brake.pid_dom->outmin(), brake.pid_dom->outmax());
            draw_dynamic(14, brake.target[PositionFB], 0.0f, 100.0f);  // brake.pid_dom->outmin(), brake.pid_dom->outmax());
            draw_dynamic(15, brake.target_pc, 0.0f, 100.0f);  // brake.pid_dom->outmin(), brake.pid_dom->outmax());
            draw_dynamic(16, brake.hybrid_out_ratio_pc, 0.0f, 100.0f);  // brake_spid_speedo_delta_adc, -range, range);
            draw_dynamic(17, brake.pid_dom->pterm(), -drange, drange);
            draw_dynamic(18, brake.pid_dom->iterm(), -drange, drange);
            draw_dynamic(19, brake.pid_dom->dterm(), -drange, drange);
            draw_eraseval(20);
            draw_dynamic(21, brake.pid_dom->kp(), 0.0f, 8.0);
            draw_dynamic(22, brake.pid_dom->ki(), 0.0f, 8.0);
            draw_dynamic(23, brake.pid_dom->kd(), 0.0f, 8.0);
        }
        else if (datapage == PG_GPID) {
            draw_asciiname(9, motormodecard[gas.motormode]);
            draw_dynamic(10, gas.throttle_target_pc, tach.opmin(), tach.opmax());
            draw_dynamic(11, gas.pid.target(), tach.opmin(), tach.opmax());
            draw_dynamic(12, gas.pid.err(), tach.idle() - tach.opmax(), tach.opmax() - tach.idle());
            draw_dynamic(13, gas.pid.pterm(), -100.0f, 100.0f);
            draw_dynamic(14, gas.pid.iterm(), -100.0f, 100.0f);
            draw_dynamic(15, gas.pid.dterm(), -100.0f, 100.0f);
            // draw_dynamic(15, gas.pid.outsum(), -gas.pid.outrange(), gas.pid.outrange());
            for (int line=16; line<=19; line++) draw_eraseval(line);
            draw_dynamic(20, gas.max_throttle_angular_velocity_degps, 0.0f, 360.0f);
            draw_dynamic(21, gas.pid.kp(), 0.0f, 1.0);
            draw_dynamic(22, gas.pid.ki(), 0.0f, 1.0);
            draw_dynamic(23, gas.pid.kd(), 0.0f, 1.0);
        }
        else if (datapage == PG_CPID) {
            drange = tach.opmax() - tach.idle();
            // draw_truth(9, gas.cruise_pid_enabled, 1);
            draw_dynamic(9, gas.cruisepid.target(), speedo.idle(), speedo.opmax());
            draw_dynamic(10, gas.cruisepid.err(), speedo.idle()-speedo.opmax(), speedo.opmax()-speedo.idle());
            draw_dynamic(11, gas.cruisepid.pterm(), -drange, drange);
            draw_dynamic(12, gas.cruisepid.iterm(), -drange, drange);
            draw_dynamic(13, gas.cruisepid.dterm(), -drange, drange);
            // draw_dynamic(14, gas.cruisepid.outsum(), -gas.cruisepid.outrange(), gas.cruisepid.outrange());  // cruise_spid_speedo_delta_adc, -drange, drange);
            // Serial.printf("min:%lf max:%lf", gas.pc[OPMIN], gas.pc[OPMAX]);
            // Serial.printf(" gmin():%lf gmax():%lf", gas.pid.outmin(), gas.pid.outmax());
            // Serial.printf(" cmin():%lf cmax():%lf", gas.cruisepid.outmin(), gas.cruisepid.outmax());
            draw_dynamic(14, gas.throttle_target_pc, 0.0f, 100.0f);
            for (int line=15; line<=19; line++) draw_eraseval(line);
            draw_dynamic(20, cruise_delta_max_pc_per_s, 1, 35);
            draw_dynamic(21, gas.cruisepid.kp(), 0.0f, 10.0f);
            draw_dynamic(22, gas.cruisepid.ki(), 0.0f, 10.0f);
            draw_dynamic(23, gas.cruisepid.kd(), 0.0f, 10.0f);
        }
        else if (datapage == PG_TEMP) {
            draw_temperature(loc::AMBIENT, 9);
            draw_temperature(loc::ENGINE, 10);
            draw_temperature(loc::WHEEL_FL, 11);
            draw_temperature(loc::WHEEL_FR, 12);
            draw_temperature(loc::WHEEL_RL, 13);
            draw_temperature(loc::WHEEL_RR, 14);
            draw_temperature(loc::BRAKE, 15);
            for (int line=16; line<=21; line++) draw_eraseval(line);
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
            draw_asciiname(21, sensorcard[sim->potmap()]);
            draw_truth(22, cal_brakemode, 0);
            draw_truth(23, cal_gasmode, 0);
        }
        else if (datapage == PG_UI) {
            draw_dynamic(9, (int)loop_avg_us, -1, -1, -1);
            draw_dynamic(10, looptimer.loop_peak_us, -1, -1, -1);
            draw_dynamic(11, (int)looptimer.loopfreq_hz, -1, -1, -1);
            draw_dynamic(12, fps);
            draw_asciiname(13, activitiescard[last_activity]);
            draw_dynamic(14, touch->touch_pt(0), 0, disp_width_pix);
            draw_dynamic(15, touch->touch_pt(1), 0, disp_height_pix);
            draw_dynamic(16, looptimer.uptime());
            // draw_dynamic(13, drawclock, 0, refresh_limit);
            // draw_dynamic(14, pushclock, 0, refresh_limit);
            // draw_dynamic(15, idleclock, 0, refresh_limit);
            for (int line=17; line<=19; line++) draw_eraseval(line);
            draw_truth(20, flashdemo, 0);
            draw_dynamic(21, neobright, 1.0, 100.0f, -1, 3);
            draw_dynamic(22, neodesat, 0, 10, -1, 2);  // -10, 10, -1, 2);
            draw_asciiname(23, uicontextcard[ui_context]);
            // draw_truth(19, (ui_context == ScreensaverUI), 0);
        }
        disp_values_dirty = false;
    }
  public:
    bool draw_all(LGFX_Sprite* spr) {
        if (!display_enabled) return false;
        if (reset_request) reset(spr);
        auto_saver();
            // if (run.display_reset_requested) init();
            // run.display_reset_requested = false;
        if (!auto_saver_enabled) {
            tiny_text();
            update_idiots(disp_idiots_dirty);
            if (disp_datapage_dirty) draw_datapage(datapage, true);
            if (disp_sidemenu_dirty) draw_touchgrid(false);
            if (disp_selected_val_dirty) draw_selected_name(tunctrl, sel_val, sel_val_last, sel_val_last_last);
            
            if (run.mode != runmode_last) disp_runmode_dirty = true;
            if (disp_values_dirty || disp_runmode_dirty || valuesRefreshTimer.expireset()) {
                disp_menu_bools();
                disp_datapage_values();
            }
            if (disp_runmode_dirty) draw_runmode(run.mode, NON);
        }
        if (sim->enabled() != sim_last) disp_simbuttons_dirty = true;
        sim_last = sim->enabled();
        fps = panel.update(spr, disp_simbuttons_dirty);
        disp_simbuttons_dirty = false;
        return true;
    }
    void push_task() {
        // Serial.printf("f%d push@ 0x%08x vs 0x%08x\n", flip, &framebuf[flip], &framebuf[!flip]);
        if (print_framebuffers) {  // warning this *severely* slows everything down, ~.25 sec/loop. consider disabling word wrap in terminal output
            Serial.printf("flip=%d\n", flip);
            printframebufs(2);
        }
        diffpush(&framebuf[flip], &framebuf[!flip]);
        flip = !flip;
        sprptr = &framebuf[flip];
        pushclock = (int)(esp_timer_get_time() - screen_refresh_time);
    }
    void draw_task() {
        int mark = (int)esp_timer_get_time();
        // Serial.printf("f%d draw@ 0x%08x\n", flip, &framebuf[flip]);
        draw_all(&framebuf[flip]);
        drawclock = (int)(esp_timer_get_time() - mark);
        idleclock = refresh_limit - pushclock - drawclock;
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
        if (autosaver_request == REQ_NA) return;
        if (autosaver_request == REQ_TOG) autosaver_request = (int)(!auto_saver_enabled);
        if (autosaver_request == (int)(auto_saver_enabled)) {
            autosaver_request = REQ_NA;
            return;
        }
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
            panel.set_vp(disp_simbuttons_x, disp_simbuttons_y, disp_simbuttons_w, disp_simbuttons_h);
            reset_request = true;
            if (was_simulating) sim->enable();
            // ui_context = DatapagesUI;
        }
        autosaver_request = REQ_NA;
    }
    void printframebufs(int reduce = 2, bool ascii = false) {  // reduce is how many times to shrink the screen by half (0, 1, 2, 3, or 4). ascii=true gives ascii art output
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
    Timer tuningAbandonmentTimer{25000000};  // This times out edit mode after a a long period of inactivity
    Timer tuningEditTimer{50000};  // Control frequency of polling for new edits
    int datapage_last;
  public:
    Tuner(Display* _screen, NeopixelStrip* _neo, Touchscreen* _touch) : screen(_screen), neo(_neo), touch(_touch) {}
    int rdelta = 0, rdelta_encoder = 0, idelta = 0, idelta_encoder = 0;  // rdelta is raw (unaccelerated) integer edit value, idelta is integer edit value accelerated
    float fdelta = 0.0;  // fdelta is float edit value accelerated
    void update(int rmode) {
        process_inputs();
        edit_values(rmode);
    }
  private:
    void process_inputs() {
        if (!tuningEditTimer.expired()) return;
        tuningEditTimer.reset();
        if (!screen->disp_selected_val_dirty) {
            sel_val_last = sel_val;
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
        // rdelta_encoder = encoder.rotation(false);  // unaccelerated encoder turn
        idelta_encoder = encoder.rotation();   // accelerated
        rdelta_encoder = constrain(idelta_encoder, -1, 1);
        // encoder.rezero();
        // if (tunctrl == EDIT) idelta_encoder = encoder.rotation(true);  // true = include acceleration
        if (tunctrl == SELECT) sel_val += rdelta_encoder;  // If overflow constrain will fix in general handler below
        else if (tunctrl == OFF) datapage += rdelta_encoder;  // If overflow tconstrain will fix in general below
        if (touch->increment_sel_val) ++sel_val %= disp_tuning_lines;
        if (touch->increment_datapage) ++datapage %= NUM_DATAPAGES;
        touch->increment_sel_val = touch->increment_datapage = false;
        idelta = idelta_encoder + touch->get_delta();  // Allow edits using the encoder or touchscreen
        fdelta = float(idelta);
        rdelta = constrain(idelta, -1, 1);  // combine unaccelerated values
        if (pot_tuner_acceleration && !sim.potmapping()) {  // use pot to control level of acceleration
            if (pot.val() < 50.0) fdelta /= map(pot.val(), 50.0, 0.0, 1.0, 5.0);
            else fdelta *= map(pot.val(), 50.0, 100.0, 1.0, 25.0);
        }
        idelta = (int)fdelta;
        if (tunctrl != tunctrl_last || datapage != datapage_last || sel_val != sel_val_last || idelta) tuningAbandonmentTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
        else if (tuningAbandonmentTimer.expired()) tunctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
        datapage = constrain(datapage, 0, datapages::NUM_DATAPAGES-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
        if (datapage != datapage_last) {
            if (tunctrl == EDIT) tunctrl = SELECT;  // If page is flipped during edit, drßop back to select mode
            screen->disp_datapage_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
        }
        if (tunctrl == SELECT) {
            sel_val = constrain(sel_val, tuning_first_editable_line[datapage], disp_tuning_lines-1);  // Skip unchangeable values for all PID modes
            if (sel_val != sel_val_last) screen->disp_selected_val_dirty = true;
        }
        if (tunctrl != tunctrl_last || screen->disp_datapage_dirty) screen->disp_selected_val_dirty = true;
    }
    void adj_potmap() {  // potmap scroll select custom adjust function. includes potentially needed refresh of sim buttons content
        sim.set_potmap((sens)(adj_val(sim.potmap(), rdelta, 0, (int)(sens::starter) - 1)));
        screen->disp_simbuttons_dirty = true;
    }
    void adj_brake_feedback(int _rdelta) {  // active brake sensors scroll select custom adjust function. allows scroll select of valid values even tho they are not contiguous in the enum
        int t = brake.feedback;
        adj_val(&t, _rdelta, 0, NumBrakeFB-1);
        brake.update_ctrl_config(brake.pid_enabled, t);
    }
    void adj_cruise_scheme() {
        int t = gas.cruise_adjust_scheme;
        adj_val(&t, rdelta, 0, NumCruiseSchemes-1);
        gas.set_cruise_scheme(t);
    }
    void edit_values(int rmode) {
        if (tunctrl == EDIT && idelta) {  // Change tunable values when editing
            if (datapage == PG_RUN) {
                if (sel_val == 13) { adj_val(&(gas.governor), idelta, 0, 100); gas.derive(); }
                else if (sel_val == 14) adj_val(&(steer.steer_safe_pc), idelta, 0, 100);
            }
            else if (datapage == PG_JOY) {
                if (sel_val == 10) airvelo.set_oplim(NAN, airvelo.opmax() + fdelta);
                else if (sel_val == 11) mapsens.set_oplim(mapsens.opmin() + fdelta, NAN);
                else if (sel_val == 12) mapsens.set_oplim(NAN, mapsens.opmax() + fdelta);
                else if (sel_val == 13) adj_val(&hotrc.failsafe_us, idelta, hotrc.absmin_us, hotrc.us[VERT][OPMIN] - hotrc.us[VERT][MARGIN]);
                else if (sel_val == 14) { adj_val(&hotrc.deadband_us, idelta, 0, 50); hotrc.calc_params(); }
            }
            else if (datapage == PG_SENS) {
                if (sel_val == 10) pressure.set_oplim(pressure.opmin() + fdelta, NAN);
                else if (sel_val == 11) pressure.set_oplim(NAN, pressure.opmax() + fdelta);
                else if (sel_val == 12) brkpos.set_oplim(brkpos.opmin() + fdelta, NAN);
                else if (sel_val == 13) brkpos.set_oplim(NAN, brkpos.opmax() + fdelta);
                else if (sel_val == 14) adj_val(brkpos.zeropoint_ptr(), fdelta, brkpos.opmin(), brkpos.opmax());
                // if (sel_val == 11) adj_val(airvelo.opmax_ptr(), fdelta, airvelo.opmin(), airvelo.absmax());
                // else if (sel_val == 12) adj_val(mapsens.opmin_ptr(), fdelta, mapsens.absmin(), mapsens.opmax());
                // else if (sel_val == 13) adj_val(mapsens.opmax_ptr(), fdelta, mapsens.opmin(), mapsens.absmax());
                // else if (sel_val == 14) adj_val(brkpos.zeropoint_ptr(), fdelta, brkpos.opmin(), brkpos.opmax());
            }
            else if (datapage == PG_PULS) {
                if (sel_val == 10) tach.set_oplim(tach.opmin() + fdelta, NAN);
                else if (sel_val == 11) tach.set_oplim(NAN, tach.opmax() + fdelta);
                else if (sel_val == 12) speedo.set_oplim(speedo.opmin() + fdelta, NAN);
                else if (sel_val == 13) speedo.set_oplim(NAN, speedo.opmax() + fdelta);
                else if (sel_val == 14) adj_val(speedo.idle_ptr(), fdelta, speedo.opmin(), speedo.opmax());
                // if (sel_val == 10) adj_bool(&web_disabled, -1 * rdelta);  // note this value is inverse to how it's displayed, same for the value display entry
            }                
            else if (datapage == PG_PWMS) {
                if (sel_val == 11) { adj_val(&(gas.si[OPMIN]), fdelta, gas.si[PARKED] + 1, gas.si[OPMAX] - 1); gas.derive(); }
                else if (sel_val == 12) { adj_val(&(gas.si[OPMAX]), fdelta, gas.si[OPMIN] + 1, 180.0f); gas.derive(); }
                else if (sel_val == 13) { adj_val(&(brake.us[STOP]), fdelta, brake.us[OPMIN] + 1, brake.us[OPMAX] - 1); brake.derive(); }
                else if (sel_val == 14) { adj_val(&(brake.duty_fwd_pc), fdelta, 0.0f, 100.0f); brake.derive(); }
            }
            else if (datapage == PG_IDLE) {
                if (sel_val == 10) adj_val(&gas.starting_pc, fdelta, gas.pc[OPMIN], gas.pc[OPMAX]);
                else if (sel_val == 11) gas.add_idlecold(fdelta);
                else if (sel_val == 12) gas.add_idlehot(fdelta);
                else if (sel_val == 13) gas.add_tempcold(fdelta);
                else if (sel_val == 14) gas.add_temphot(fdelta);
            }
            else if (datapage == PG_MOTR) {
                if (sel_val == 9) brake.update_ctrl_config((int)(rdelta>0));
                else if (sel_val == 10) adj_brake_feedback(rdelta);
                else if (sel_val == 11) adj_bool(&brake.enforce_positional_limits, rdelta);
                else if (sel_val == 12) gas.update_ctrl_config((int)(rdelta>0));
                else if (sel_val == 13) gas.update_cruise_ctrl_config((int)(rdelta>0));
                else if (sel_val == 14) adj_cruise_scheme();
            }
            else if (datapage == PG_BPID) {
                if (sel_val == 12) brake.pid_dom->add_kp(0.001f * fdelta);
                else if (sel_val == 13) brake.pid_dom->add_ki(0.001f * fdelta);
                else if (sel_val == 14) brake.pid_dom->add_kd(0.001f * fdelta);
            }
            else if (datapage == PG_GPID) {
                if (sel_val == 11) adj_val(&(gas.max_throttle_angular_velocity_degps), fdelta, 0.0f, 180.0f);
                else if (sel_val == 12) gas.pid.add_kp(0.001f * fdelta);
                else if (sel_val == 13) gas.pid.add_ki(0.001f * fdelta);
                else if (sel_val == 14) gas.pid.add_kd(0.001f * fdelta);
            }
            else if (datapage == PG_CPID) {
                if (sel_val == 11) adj_val(&cruise_delta_max_pc_per_s, idelta, 1, 35);
                else if (sel_val == 12) gas.cruisepid.add_kp(0.001f * fdelta);
                else if (sel_val == 13) gas.cruisepid.add_ki(0.001f * fdelta);
                else if (sel_val == 14) gas.cruisepid.add_kd(0.001f * fdelta);
            }
            else if (datapage == PG_TEMP) {
                if (sel_val == 13) adj_bool(&dont_take_temperatures, rdelta);
                else if (sel_val == 14) adj_bool(&web_disabled, rdelta);
            }
            else if (datapage == PG_SIM) {
                if (sel_val == 4) { sim.set_can_sim(sens::joy, rdelta); screen->disp_simbuttons_dirty = true; }
                else if (sel_val == 5) { sim.set_can_sim(sens::pressure, rdelta); screen->disp_simbuttons_dirty = true; }
                else if (sel_val == 6) { sim.set_can_sim(sens::brkpos, rdelta); screen->disp_simbuttons_dirty = true; }
                else if (sel_val == 7) { sim.set_can_sim(sens::speedo, rdelta); screen->disp_simbuttons_dirty = true; }
                else if (sel_val == 8) { sim.set_can_sim(sens::tach, rdelta); screen->disp_simbuttons_dirty = true; }
                else if (sel_val == 9) { sim.set_can_sim(sens::airvelo, rdelta); screen->disp_simbuttons_dirty = true; }
                else if (sel_val == 10) { sim.set_can_sim(sens::mapsens, rdelta); screen->disp_simbuttons_dirty = true; } // else if (sel_val == 7) sim.set_can_sim(sens::starter, idelta);
                else if (sel_val == 11) { sim.set_can_sim(sens::basicsw, rdelta); screen->disp_simbuttons_dirty = true; }
                else if (sel_val == 12) adj_potmap();
                else if (sel_val == 13) adj_bool(&cal_brakemode_request, rdelta);
                else if (sel_val == 14) adj_bool(&cal_gasmode_request, rdelta);
            }
            else if (datapage == PG_UI) {
                if (sel_val == 11) { adj_bool(&flashdemo, rdelta); neo->enable_flashdemo(flashdemo); }
                else if (sel_val == 12) { adj_val(&neobright, rdelta, 1, 100); neo->setbright(neobright); }
                else if (sel_val == 13) { adj_val(&neodesat, rdelta, 0, 10); neo->setdesaturation(neodesat); }
                else if (sel_val == 14) adj_val(&ui_context, rdelta, 0, NumContextsUI-1);
            }
            idelta = 0;
        }
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
// pushbuf_sem = xSemaphoreCreateMutexStatic(&push_semaphorebuf_sem);  // xSemaphoreCreateBinaryStatic(&push_semaphorebuf_sem);
// drawbuf_sem = xSemaphoreCreateMutexStatic(&draw_semaphorebuf_sem);  // xSemaphoreCreateBinaryStatic(&draw_semaphorebuf_sem);
static void push_task_wrapper(void *parameter) {
    while (true) {
        if (take_two_semaphores(&pushbuf_sem, &drawbuf_sem, portMAX_DELAY) == pdTRUE) {
            screen.push_task();
            xSemaphoreGive(pushbuf_sem);
            xSemaphoreGive(drawbuf_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(2)); 
        // vTaskDelete(NULL);
    }
}
static void draw_task_wrapper(void *parameter) {
    while (true) {
        // if ((esp_timer_get_time() - screen_refresh_time > refresh_limit) || always_max_refresh || auto_saver_enabled) {
        if (xSemaphoreTake(drawbuf_sem, portMAX_DELAY) == pdTRUE) {
            screen_refresh_time = esp_timer_get_time();
            screen.draw_task();
            xSemaphoreGive(drawbuf_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(1));  //   || sim.enabled()
        if (!always_max_refresh && !auto_saver_enabled) vTaskDelay(pdMS_TO_TICKS((int)(refresh_limit / 1000 - 1)));  //   || sim.enabled()
    }
}
// The following project draws a nice looking gauge cluster, very apropos to our needs and the code is given.
// See this video: https://www.youtube.com/watch?v=U4jOFLFNZBI&ab_channel=VolosProjects
// Rinkydink home page: http://www.rinkydinkelectronics.com
// moving transparent arrow sprite over background: https://www.youtube.com/watch?v=U4jOFLFNZBI&ab_channel=VolosProjects
// bar graphs: https://www.youtube.com/watch?v=g4jlj_T-nRw&ab_channel=VolosProjects