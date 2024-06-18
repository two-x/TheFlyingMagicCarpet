#pragma once
#include <NeoPixelBus.h>
#define neorgb_t RgbColor  // RgbwColor
#define striplength 8
NeoPixelBus<NeoGrbFeature, NeoSk6812Method> neoobj(striplength, neopixel_pin);  // NeoWs2812Method, NeoWs2812xMethod, NeoSk6812Method, NeoEsp32Rmt0Ws2812xMethod, NeoEsp32I2s1800KbpsMethod, NeoEsp32I2s1Sk6812Method, 
// Default for esp32 is dma via I2S bus 1 at 800kHz using RMT. Don't know the protocol difference between "Ws2812", "Ws2812x", and "Sk6812"
// Run neos in a task example: https://github.com/Makuna/NeoPixelBus/wiki/ESP32-and-RTOS-Tasks

// i made a set of functions to convert colors between any of four relevant formats. 
uint32_t color_to_888(uint16_t color565) { return (static_cast<uint32_t>(color565 & 0xf800) << 8) | (static_cast<uint32_t>(color565 & 0x7e0) << 5) | (static_cast<uint32_t>(color565 & 0x1f) << 3); } // Convert 5-6-5 encoded 16-bit color value to uint32 in format 0x00RRGGBB
uint32_t color_to_888(uint8_t color332) { return (static_cast<uint32_t>(color332 & 0xe0) << 16) | (static_cast<uint32_t>(color332 & 0x1c) << 11) | (static_cast<uint32_t>(color332 & 0x3) << 6); }
uint32_t color_to_888(neorgb_t colorneo) { return (static_cast<uint32_t>(colorneo.R) << 16) | (static_cast<uint32_t>(colorneo.G) << 8) | static_cast<uint32_t>(colorneo.B); } // (static_cast<uint32_t>(color.W) << 24)
uint16_t color_to_565(uint32_t color888) { return static_cast<uint16_t>(((color888 & 0xf80000) >> 8) | ((color888 & 0xfc00) >> 5) | ((color888 & 0xf8) >> 3)); }  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
uint16_t color_to_565(uint8_t color332) { return ((static_cast<uint16_t>(color332) & 0xe0) << 8) | ((static_cast<uint16_t>(color332) & 0x1c) << 6) | ((static_cast<uint16_t>(color332) & 0x3) << 3); }  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
uint16_t color_to_565(neorgb_t colorneo) { return ((static_cast<uint16_t>(colorneo.R) & 0xf8) << 8) | ((static_cast<uint16_t>(colorneo.G) & 0xfc) << 3) | (((static_cast<uint16_t>(colorneo.B) & 0xf8) >> 3)); }
uint8_t color_to_332(uint16_t color565) { return static_cast<uint8_t>(((color565 & 0xe000) >> 8) | ((color565 & 0x700) >> 6) | ((color565 & 0x18) >> 3)); }  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
uint8_t color_to_332(uint32_t color888) { return static_cast<uint8_t>(((color888 & 0xe00000) >> 16) | ((color888 & 0xe000) >> 11) | ((color888 & 0xc0) >> 6)); }
uint8_t color_to_332(neorgb_t colorneo) { return (colorneo.R & 0xe0) | ((colorneo.G & 0xe0) >> 3) | ((colorneo.B & 0xc0) >> 6); }
neorgb_t color_to_neo(uint32_t color888) { return neorgb_t((color888 >> 16) & 0xff, (color888 >> 8) & 0xff, color888 & 0xff); }  // (static_cast<uint32_t>(color.W) << 24) |
neorgb_t color_to_neo(uint16_t color565) { return neorgb_t((color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3); }  // , 0);
neorgb_t color_to_neo(uint8_t color332) { return neorgb_t(color332 & 0xe0, (color332 & 0x1c) << 3, (color332 & 0x3) << 6); }  // , 0);
class NeopixelStrip {
  public:
    static const int idiotcount = 7;
  private:
    enum brightness_presets : int { B_OFF, B_MIN, B_LO, B_MED, B_HI, B_EXT, B_MAX };
    enum ledset : int { onoff, fcount, fpulseh, fpulsel, fonbrit, fnumset };  // just a bunch of int variables needed for each of the neo idiot lights
    enum ledcolor : int { cnow, clast, cnormal, coff, con, cflash, cnumcolors };
    enum brightness_contexts { NITE, DAY };  // Indoors = NITE
    uint8_t brightlev[2][7] = { { 0, 2,  6, 10, 17, 30,  50 },     // [NITE] [B_OFF/B_MIN/B_LOW/B_MED/B_HIGH/B_EXT/B_MAX]
                                { 0, 3, 16, 30, 45, 65, 100 }, };  // [DAY] [B_OFF/B_MIN/B_LOW/B_MED/B_HIGH/B_EXT/B_MAX]
    bool context = NITE;
    bool neo_heartbeat_variable_brightness = false;  // If false then brightness control only affect idiotlights
    uint8_t lobright;
    uint8_t heartbright = 22;
    uint8_t hibright = 35;
    uint8_t heartlobright = 2;
    int lomultiplier = 3;  // lobright is hibright divided by this twice
    uint8_t neo_master_brightness = 0xff;
    float correction[3] = { 1.0, 0.9, 1.0 };  // Applied to brightness of rgb elements
    static constexpr int neo_fade_timeout_us = 380000;
    Timer neoFadeTimer = Timer(neo_fade_timeout_us), neoHeartbeatTimer;
    bool neo_heartbeat = false;
    bool heartcolor_change = true;  // , heartcolor_overridden = false;
    int pin = -1;
    uint8_t heartbeat_brightness, heartbeat_brightness_last; // brightness during fadeouts
    int neobright_last;
    int heartbeat_state = 0;
    int heartbeat_level = 0;
    int64_t heartbeat_ekg_us[4] = {250000, 240000, 620000, 2000000};  // {187500, 125000, 562500, 1250000};
    int heartbeat_pulse = 255;
    static const int numpixels = 1 + idiotcount;  //  + extIdiotCount;  // 15 pixels = heartbeat RGB + 7 onboard RGB + 7 external RGBW
    neorgb_t neostrip[numpixels];
    neorgb_t heartbeatColor, heartbeatNow;
    uint8_t heartcolor16 = 0x00;  // blackened heart
    bool breadboard = false;
    int fset[idiotcount][fnumset];
    neorgb_t cidiot[idiotcount][cnumcolors];
    int fquantum_us = 50000;  // time resolution of flashes
    static const int fevresolution = 128;
    static const int fevpages = 4;  // fevresolution / 32;  <- but if I use that math instead, seg fault
    int fevents[idiotcount][fevpages];
    int fevcurrpage = 0; int fevfilled = 0; int nowtime_us, nowepoch;  // , fevmask_master;
    Timer flashtimer;
    float maxelement(float r, float g, float b);
    float midelement(float r, float g, float b);
    float minelement(float r, float g, float b);
    neorgb_t dimmer(neorgb_t color, uint8_t bright);  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
    neorgb_t desaturate(neorgb_t color, int _desat_of_ten);  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
    void recolor_idiots(int argidiot = -1);
    bool fevpop(int _idiot, int pop_off);
    void fevpush(int _idiot, int push_off, bool push_val);
    void update_idiot(int _idiot);
    void calc_lobright();
  public:
    NeopixelStrip(int argpin) { pin = argpin; }
    void refresh();
    void setup(bool viewcontext=NITE);
    void setbright(int bright_pc);
    void setdesaturation(float _desat_of_ten);  // a way to specify nite or daytime brightness levels
    void heartbeat_ena(bool onoroff);
    void set_heartcolor(uint8_t newcolor);
    void heartcolor_override(uint8_t color);
    void heartbeat_update();
    void colorfade_update();
    int neopixelsAvailable();
    bool newIdiotLight(int _idiot, uint8_t color332, bool startboolstate = 0);
    void setBoolState(int _idiot, bool state);
    void setflash(int _idiot, int count, int pulseh=1, int pulsel=1, int onbrit=-1, uint32_t color=0xffffff);
    void update(uint16_t heart_color);
    void enable_flashdemo(bool ena);
    uint32_t idiot_neo_color(int _idiot);
};

float NeopixelStrip::maxelement(float r, float g, float b) {
    return (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //std::max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
float NeopixelStrip::midelement(float r, float g, float b) {
    return (r >= g) ? ((g >= b) ? g : ((r >= b) ? b : r)) : ((r >= b) ? r : ((b >= g) ? g : b));  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //std::max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
float NeopixelStrip::minelement(float r, float g, float b) {
    return (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //std::max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
neorgb_t NeopixelStrip::dimmer(neorgb_t color, uint8_t bright) {  // brightness 0 is off, 255 is max brightness while retaining same hue and saturation
    int ret[3];
    float rgb[3] = { static_cast<float>(color.R), static_cast<float>(color.G), static_cast<float>(color.B) };
    float fbright = (float)bright / maxelement(rgb[0], rgb[1], rgb[2]);  // std::max(color.r, color.g, color.b);  // 2.55 = 0xff / 100
    for (int element=0; element<3; element++) ret[element] = constrain((int)(rgb[element] * fbright), 0, 255);
    // printf("D br:%d  inc:%06x  outc:%02x%02x%02x\n", bright, color_to_888(color), (uint8_t)ret[0], (uint8_t)ret[1], (uint8_t)ret[2]);
    return neorgb_t(ret[0], ret[1], ret[2]);  // return CRGB((float)(color.r * fbright), (float)(color.g * fbright), (float)(color.b * fbright));
}
neorgb_t NeopixelStrip::desaturate(neorgb_t color, int _desat_of_ten) {  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
    uint8_t rgb[3] = { static_cast<uint8_t>(color.R), static_cast<uint8_t>(color.G), static_cast<uint8_t>(color.B) };
    float dominant = maxelement(rgb[0], rgb[1], rgb[2]);
    if (_desat_of_ten > 0)
        for (int element=0; element<3; element++)
            rgb[element] = (uint8_t)(rgb[element] + ((float)_desat_of_ten * (dominant - (float)(rgb[element])) / 20.0));
    // else {
    //     // float mide = midelement(rgb[0], rgb[1], rgb[2]);
    //     dominant = minelement(rgb[0], rgb[1], rgb[2]);
    //     for (int element=0; element<3; element++)
    //         rgb[element] = (uint8_t)(rgb[element] - ((float)desat_of_ten * dominant / 10.0));
    // }
    return neorgb_t(rgb[0], rgb[1], rgb[2]);
}
void NeopixelStrip::recolor_idiots(int _idiot) {  // pass in -1 to recolor all idiots
    int start = (_idiot >= 0) ? _idiot : 0;
    int end = (_idiot >= 0) ? _idiot + 1 : idiotcount;
    for (int i = start; i < end; i++) {
        cidiot[i][con] = dimmer(cidiot[i][cnormal], hibright);
        cidiot[i][con] = desaturate(cidiot[i][con], desat_of_ten);
        cidiot[i][coff] = dimmer(cidiot[i][cnormal], lobright);
        cidiot[i][coff] = desaturate(cidiot[i][coff], desat_of_ten);
    }
}
void NeopixelStrip::refresh() {
    int numledstowrite = (heartbeatNow != neostrip[0]);
    neostrip[0] = heartbeatNow;
    neoobj.SetPixelColor(0, heartbeatNow);
    for (int i=0; i<idiotcount; i++) {
        if (cidiot[i][cnow] != neostrip[i+1]) {
            neoobj.SetPixelColor (1+i, cidiot[i][cnow]);
            neostrip[i + 1] = cidiot[i][cnow];  // color_to_neo(cidiot[i][cnow]);
            numledstowrite = 2 + i;  // + idiotCount;
        }
    }
    if (numledstowrite) neoobj.Show(numledstowrite);  // This ability to exclude pixels at the end of the strip that haven't changed from the data write is an advantage of neopixelbus over adafruit
}
void NeopixelStrip::setup(bool viewcontext) {
    std::cout << "Neopixels.. ";
    breadboard = running_on_devboard;
    context = viewcontext;
    calc_lobright();
    neoobj.Begin();
    heartbeat_brightness = brightlev[context][B_LO];
    neoHeartbeatTimer.set(heartbeat_ekg_us[3]);
    neoFadeTimer.reset();
    setbright(neobright);
    setdesaturation(desat_of_ten);
    heartbeat_ena(true);
    std::cout << "refresh.. ";
    flashtimer.set(fquantum_us * fevresolution);
    refresh();
    std::cout << std::endl;
}
void NeopixelStrip::calc_lobright() {
    lobright = std::max(3, (hibright / lomultiplier) / lomultiplier);
}
void NeopixelStrip::setbright(int bright_pc) {  // a way to specify brightness level as a percent
    neobright = bright_pc;
    hibright = (uint8_t)((255.0 * (float)neobright) / 100.0);
    calc_lobright();
    if (neo_heartbeat_variable_brightness) {
        heartbright = hibright;  // (uint8_t)((float)hibright * 0.75);
        heartlobright = lobright;
    }
    recolor_idiots();
}
void NeopixelStrip::setdesaturation(float _desat_of_ten) {  // a way to specify nite or daytime brightness levels
    desat_of_ten = _desat_of_ten;
    recolor_idiots();
}
void NeopixelStrip::heartbeat_ena(bool onoroff) {
    neo_heartbeat = onoroff;  // Start heart beating
}
void NeopixelStrip::set_heartcolor(uint8_t _newcolor) {
    uint8_t newcolor = _newcolor;
    if (heartbeat_override_color != 0x00) newcolor = heartbeat_override_color;
    if (heartcolor16 != newcolor) {
        heartbeatColor = color_to_neo(newcolor);
        heartcolor_change = true;
        heartcolor16 = newcolor;
    }
}
void NeopixelStrip::heartbeat_update() {
    if (!neo_heartbeat) return;
    if (neoHeartbeatTimer.expired()) {
        heartbeat_pulse = !heartbeat_pulse;
        ++heartbeat_state %= arraysize(heartbeat_ekg_us);
        neoHeartbeatTimer.set(heartbeat_ekg_us[heartbeat_state]);
        if (heartbeat_pulse) heartbeat_brightness = heartbright;
        else neoFadeTimer.reset();
    }
    else if (!heartbeat_pulse) {
        heartbeat_brightness_last = heartbeat_brightness;
        if (neoFadeTimer.expired()) heartbeat_brightness = brightlev[context][B_MIN];
        else heartbeat_brightness = (uint8_t)(heartlobright + std::max((double)0, (float)(heartbright - heartlobright) * (1.0 - ((heartbeat_state == 1) ? 1.5 : 1.0) * (float)neoFadeTimer.elapsed() / (float)neo_fade_timeout_us)));
        if (heartbeat_brightness > heartbeat_brightness_last) heartbeat_brightness = heartbeat_brightness_last;
    }
    if (heartcolor_change || heartbeat_brightness != neobright_last) {
        heartbeatNow = dimmer(heartbeatColor, heartbeat_brightness);  // heartbeatNow = dimmer(desaturate(heartbeatColor, desat_of_ten), heartbeat_brightness);
        heartcolor_change = false;
        neobright_last = heartbeat_brightness;
    }
}
int NeopixelStrip::neopixelsAvailable() {
    return idiotcount;
}
bool NeopixelStrip::newIdiotLight(int _idiot, uint8_t color332, bool startboolstate) {
    if (_idiot > idiotcount-1) return false;
    fset[_idiot][onoff] = startboolstate;
    cidiot[_idiot][cnormal] = color_to_neo(color332);
    cidiot[_idiot][clast] = color_to_neo((uint32_t)0);
    fset[_idiot][fcount] = 0;
    setBoolState(_idiot, startboolstate);
    for (int pg = 0; pg < fevpages; pg++) fevents[_idiot][pg] = 0;
    recolor_idiots(_idiot);
    return true;
}
void NeopixelStrip::setBoolState(int _idiot, bool state) {
    fset[_idiot][onoff] = state;
}
bool NeopixelStrip::fevpop(int _idiot, int pop_off) {
    int page = pop_off / 32;
    return (fevents[_idiot][page] >> (pop_off - 32 * page)) & 1;  // pop a bit off the stack and return it
}
void NeopixelStrip::fevpush(int _idiot, int push_off, bool push_val) {
    int page = push_off / 32;
    fevents[_idiot][page] |= (push_val << (push_off - 32 * page));
}
// setflash() : Call this to add a blink sequence to one of the idiot lights which will repeat indefinitely in [up to 6.4 sec] cycles
//   _idiot = which idiot light
//   count  = number of blinks per cycle. Use 0 to cancel a previously applied blink pattern
//   pulseh = high pulse width of each blink, in increments of 50 ms (of which there are max 128 per cycle)
//   pulsel = low pulse width of each blink, in increments of 50 ms (of which there are max 128 per cycle)
//   onbrit = percent brightness to apply during high pulses (use -1 to avoid)
//   color  = alternate color to apply during high pulses (use -1 to avoid)
void NeopixelStrip::setflash(int _idiot, int count, int pulseh, int pulsel, int onbrit, uint32_t color) {
    fset[_idiot][fcount] = count;
    fset[_idiot][fpulseh] = std::max((int)pulseh, 1);
    fset[_idiot][fpulsel] = std::max((int)pulsel, 1);
    fset[_idiot][fonbrit] = (onbrit == -1) ? hibright : (int)onbrit;
    cidiot[_idiot][cflash] = dimmer((color == -1) ? cidiot[_idiot][cnormal] : color_to_neo((uint32_t)color), hibright);
    cidiot[_idiot][cflash] = desaturate(dimmer(cidiot[_idiot][cflash], fset[_idiot][fonbrit]), desat_of_ten);
    for (int pg = 0; pg < fevpages; pg++) fevents[_idiot][pg] = 0;
    int filled = 0;
    int lstop;
    int patternlen = fset[_idiot][fcount] * (fset[_idiot][fpulseh] + fset[_idiot][fpulsel]);
    int reps = 1 + (patternlen < fevresolution / 3);  // For shorter flash patterns repeat them multiple times in each cycle
    for (int rep = 1; rep <= reps; rep++) {        
        lstop = std::min((int)fevresolution, filled + patternlen);
        while (filled < lstop) {
            for (int hbit = 0; hbit < fset[_idiot][fpulseh]; hbit++)
                if (filled < lstop) fevpush(_idiot, filled++, 1);
            for (int lbit = 0; lbit < fset[_idiot][fpulsel]; lbit++)
                if (filled < lstop) fevpush(_idiot, filled++, 0); 
        }
        filled = fevresolution / reps;
    }
}
uint32_t NeopixelStrip::idiot_neo_color(int _idiot) { 
    return color_to_888(cidiot[_idiot][cnow]);
}
void NeopixelStrip::enable_flashdemo(bool ena) {
    flashdemo = ena;
    if (flashdemo) {
        setflash(4, 8, 8, 8, 20, -1);  // brightness toggle in a continuous squarewave
        setflash(5, 3, 1, 2, 85);      // three super-quick bright white flashes
        setflash(6, 2, 5, 5, 0, 0);    // two short black pulses
    }
    else {
        setflash(4, 0);
        setflash(5, 0);
        setflash(6, 0);
    }
}
void NeopixelStrip::update_idiot(int _idiot) {
    cidiot[_idiot][clast] = cidiot[_idiot][cnow];
    neorgb_t newnow = (fset[_idiot][onoff]) ? cidiot[_idiot][con] : cidiot[_idiot][coff];
    if (!fset[_idiot][fcount]) cidiot[_idiot][cnow] = newnow;
    else cidiot[_idiot][cnow] = fevpop(_idiot, nowepoch) ? cidiot[_idiot][cflash] : newnow;
}
void NeopixelStrip::update(uint16_t heart_color) {
    set_heartcolor(heart_color);
    heartbeat_update();  // Update our beating heart
    nowtime_us = (int)flashtimer.elapsed();
    nowepoch = nowtime_us / fquantum_us;
    for (int i=0; i<idiotcount; i++) {
        if (!syspower) cidiot[i][cnow] = neorgb_t(0);
        else update_idiot(i);
    }
    refresh();
    flashtimer.expireset();
}

class IdiotLight {  // defunct: currently not using individual instances for each idiot light. i couldn't get it to work
    public:
    bool* val = nullptr;
    char letters[3] = "--";
    uint8_t bitmap[11] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    uint8_t color = DGRY;
    bool last;  // = 0;
    IdiotLight(bool* _val, uint8_t* _map) : val(_val) {
        for (int i=0; i<11; i++) bitmap[i] = _map[i];
        last = *val;
    } 
};
class IdiotLights {
  public:
    static constexpr int row_count = 12;
    static constexpr int row_height = 11;
    static constexpr int iconcount = 36;  // number of boolean values included on the screen panel (not the neopixels) 
    bool* vals[iconcount] = {  // 6 per line
        &diag.err_sens_alarm[LOST], &diag.err_sens_alarm[RANGE], &diag.err_sens[RANGE][_TempEng], &diag.err_sens[RANGE][_TempWheel], hotrc.radiolost_ptr(), &panicstop,
        &standby_incomplete, &parking, &brake.autostopping, &brake.autoholding, &cruise_adjusting, &car_hasnt_moved, 
        &starter.motor, &fuelpump.status_inverse, &brake.posn_pid_active, &brake.no_feedback, speedo.pin_inactive_ptr(), tach.pin_inactive_ptr(),
        &bootbutton.now, &nowtouch, &encoder.enc_a, sim.enabled_ptr(), &running_on_devboard, &not_syspower,
        &sensidiots[_Throttle], &sensidiots[_BrakeMotor], &sensidiots[_SteerMotor], &sensidiots[_HotRC], &sensidiots[_Speedo], &sensidiots[_Tach],
        &sensidiots[_BrakePres], &sensidiots[_BrakePosn], &sensidiots[_Temps], &sensidiots[_MuleBatt], &sensidiots[_Other], &sensidiots[_GPIO],
    };  // , &encoder.enc_b, &starter.req_active, &web_disabled, &powering_up
    uint8_t icon[iconcount][11] = {
        { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x3e, 0x71, 0x59, 0x4d, 0x47, 0x3e, },  // "S" w/ crossout symbol
        { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x78, 0x70, 0x59, 0x4d, 0x07, 0x0f, },  // "S" w/ double arrow
        { 0x7f, 0x7f, 0x6b, 0x6b, 0x00, 0x70, 0x10, 0x10, 0x77, 0x65, 0x07, },  // "En" w/ degree symbol
        { 0x7f, 0x30, 0x18, 0x30, 0x7f, 0x00, 0x7e, 0x10, 0x77, 0x65, 0x07, },  // "Wh" w/ degree symbol
        { 0x7a, 0x7f, 0x4f, 0x17, 0x06, 0x00, 0x22, 0x1c, 0x00, 0x41, 0x3e, },  // hotrc w/ radio waves
        { 0x7f, 0x7f, 0x09, 0x09, 0x77, 0x16, 0x70, 0x60, 0x00, 0x6f, 0x6f, },  // "Pn!"
        { 0x16, 0x15, 0x0d, 0x60, 0x6f, 0x04, 0x6f, 0x60, 0x0f, 0x69, 0x66, },  // "SHD..."
        { 0x3e, 0x63, 0x41, 0x7d, 0x7d, 0x55, 0x55, 0x5d, 0x49, 0x63, 0x3e, },  // circle-"P"
        { 0x3e, 0x49, 0x1c, 0x00, 0x6e, 0x6b, 0x3b, 0x00, 0x1c, 0x49, 0x3e, },  // brake assembly w/ "S"
        { 0x3e, 0x49, 0x1c, 0x00, 0x7f, 0x18, 0x7f, 0x00, 0x1c, 0x49, 0x3e, },  // brake assembly w/ "H"
        { 0x08, 0x1c, 0x36, 0x00, 0x3e, 0x63, 0x63, 0x00, 0x36, 0x1c, 0x08, },  // "<C>"
        { 0x3d, 0x43, 0x07, 0x00, 0x3e, 0x63, 0x55, 0x49, 0x55, 0x63, 0x3e, },  // rotation arrow w/ X wheel
        { 0x3e, 0x41, 0x7f, 0x7b, 0x7b, 0x7b, 0x3e, 0x1c, 0x7f, 0x55, 0x7f, },  // motor w/ spur gear
        { 0x40, 0x7e, 0x79, 0x79, 0x79, 0x7e, 0x48, 0x38, 0x61, 0x3f, 0x86, },  // fuel
        { 0x7c, 0x46, 0x7f, 0x7f, 0x33, 0x12, 0x12, 0x12, 0x1e, 0x12, 0x0c, },  // linear actuator or choad
        { 0x3e, 0x63, 0x41, 0x33, 0x36, 0x1c, 0x1c, 0x36, 0x22, 0x63, 0x41, },  // open loop
        { 0x63, 0x63, 0x63, 0x63, 0x77, 0x3e, 0x1c, 0x00, 0x6e, 0x6b, 0x3b, },  // "S" magnet w/ zap
        { 0x63, 0x63, 0x63, 0x63, 0x77, 0x3e, 0x1c, 0x00, 0x06, 0x7e, 0x06, },  // "T" magnet w/ zap
        { 0x01, 0x7f, 0x7f, 0x7f, 0x3f, 0x38, 0x74, 0x70, 0x70, 0x70, 0x60, },  // boot
        { 0x78, 0x7c, 0x7f, 0x7f, 0x7c, 0x7c, 0x1c, 0x0c, 0x0c, 0x0c, 0x0c, },  // finger
        { 0x0e, 0x1d, 0x7d, 0x7d, 0x1d, 0x0e, 0x00, 0x7f, 0x6b, 0x6b, 0x63, },  // encoder "E"
        { 0x6e, 0x6b, 0x3b, 0x00, 0x7f, 0x00, 0x7f, 0x06, 0x1c, 0x06, 0x7f, },  // "SIM"
        { 0x7f, 0x63, 0x3e, 0x00, 0x7f, 0x6b, 0x6b, 0x00, 0x7f, 0x30, 0x1f, },  // "DEV"
        { 0x00, 0x3e, 0x63, 0x41, 0x40, 0x4f, 0x40, 0x41, 0x63, 0x3e, 0x00, },  // power symbol
        { 0x3e, 0x63, 0x7b, 0x00, 0x7e, 0x13, 0x7e, 0x00, 0x6e, 0x6b, 0x3b, },  // "GAS"
        { 0x3e, 0x49, 0x08, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x08, 0x49, 0x3e, },  // brakes or tie fighter
        { 0x00, 0x1c, 0x26, 0x45, 0x49, 0x79, 0x49, 0x45, 0x26, 0x1c, 0x00, },  // steering wheel
        { 0x00, 0x00, 0x00, 0x72, 0x7f, 0x7f, 0x4f, 0x03, 0x06, 0x00, 0x00, },  // hotrc
        { 0x60, 0x66, 0x06, 0x10, 0x30, 0x63, 0x43, 0x00, 0x6e, 0x6b, 0x3b, },  // gauge "S"
        { 0x60, 0x66, 0x06, 0x10, 0x30, 0x63, 0x43, 0x00, 0x06, 0x7e, 0x06, },  // gauge "T"
        { 0x42, 0x24, 0x2f, 0x44, 0x42, 0x20, 0x22, 0x44, 0x4f, 0x24, 0x22, },  // pressure waves
        { 0x7e, 0x42, 0x46, 0x42, 0x4e, 0x42, 0x46, 0x42, 0x4e, 0x42, 0x7e, },  // ruler
        { 0x02, 0x07, 0x35, 0x77, 0x7a, 0x1c, 0x0e, 0x1f, 0x13, 0x06, 0x04, },  // thermometer or schlong
        { 0x7e, 0x77, 0x63, 0x77, 0x7e, 0x7e, 0x7e, 0x77, 0x77, 0x77, 0x7e, },  // battery
        { 0x7f, 0x6b, 0x6b, 0x00, 0x03, 0x7f, 0x03, 0x00, 0x3e, 0x63, 0x63, },  // "ETC"
        { 0x2a, 0x2a, 0x2a, 0x7f, 0x7d, 0x7f, 0x7f, 0x7f, 0x2a, 0x2a, 0x2a, },  // chip
    };
     // { 0x4c, 0x2a, 0x19, 0x00, 0x63, 0x63, 0x63, 0x63, 0x77, 0x3e, 0x1c, },  // [original] magnet w/ zap
     // { 0x36, 0x1c, 0x36, 0x00, 0x22, 0x1c, 0x00, 0x22, 0x1c, 0x41, 0x3e, },  // wifi symbol w/ X
     // { 0x7f, 0x19, 0x6e, 0x00, 0x7f, 0x6b, 0x6b, 0x00, 0x3e, 0x23, 0x7e, },  // "REQ"
     // { 0x7a, 0x7f, 0x4f, 0x17, 0x06, 0x00, 0x02, 0x5b, 0x59, 0x0f, 0x06, },  // hotrc w/ "?"
     // { 0x10, 0x18, 0x68, 0x78, 0x10, 0x78, 0x68, 0x18, 0x12, 0x07, 0x7f, },  // carpet at stop sign
     // { 0x2a, 0x2a, 0x7f, 0x61, 0x7f, 0x43, 0x5b, 0x43, 0x7f, 0x2a, 0x2a, },  // sensor - chip w/ "IO"
     // { 0x0f, 0x05, 0x75, 0x15, 0x35, 0x15, 0x37, 0x15, 0x32, 0x10, 0x30, },  // sensor - piston w/ ruler
     // { 0x08, 0x00, 0x3a, 0x18, 0x29, 0x40, 0x01, 0x00, 0x5a, 0x54, 0x64, },  // sensor - gauge "S"
     // { 0x08, 0x00, 0x3a, 0x18, 0x29, 0x40, 0x01, 0x00, 0x0a, 0x78, 0x08, },  // sensor - gauge "T"
     // { 0x08, 0x00, 0x3a, 0x18, 0x29, 0x40, 0x71, 0x10, 0x22, 0x10, 0x78, },  // gauge "M"
     // { 0x0e, 0x1d, 0x7d, 0x1d, 0x0e, 0x00, 0x7e, 0x0b, 0x09, 0x0b, 0x7e, },  // encoder "A"
     // { 0x0e, 0x1d, 0x7d, 0x1d, 0x0e, 0x00, 0x7f, 0x49, 0x49, 0x7f, 0x36, },  // encoder "B"
     // { 0x02, 0x07, 0x32, 0x78, 0x78, 0x30, 0x00, 0x06, 0x0f, 0x0f, 0x06, },  // balls
     // { 0x3e, 0x41, 0x1c, 0x22, 0x08, 0x7c, 0x08, 0x22, 0x1c, 0x41, 0x3e, },  // X w/ waves
     // { 0x63, 0x36, 0x1c, 0x36, 0x63, 0x14, 0x08, 0x22, 0x1c, 0x41, 0x3e, },  // wifi symbol w/ X
    std::string letters[iconcount] = {
        "SL", "SR", "\xf7""E", "\xf7""W", "RC", "P\x13", "SI", "Pk", "AS", "AH", "Aj", "HM",  // 12 per row 
        "St", "FP",      "Pn",   "NF",     "SM", "TM", "Bt", "NT",   "eA", "WD", "Dv", "Pw",
        "Th", "Br", "St",      "RC",      "Sp", "Tc",    "Pr", "Ps", "Tm", "Bt", "Ot", "IO",
    };
    uint8_t color[2][iconcount] = {
     // Colors gradiated for 12 per row
        { 0x82, 0xa2, 0xc1, 0xcc, 0xd0, 0xb4, 0x74, 0x14, 0x1a, 0x16, 0x0e, 0x0a,
          0x82, 0xa2, 0xc1, 0xcc, 0xd0, 0xb4, 0x74, 0x14, 0x1a, 0x16, 0x0e, 0x0a,
          0x82, 0xa2, 0xc1, 0xcc, 0xd0, 0xb4, 0x74, 0x14, 0x1a, 0x16, 0x0e, 0x0a, },
        { 0xa3, 0xc2, 0xe1, 0xec, 0xf4, 0xfc, 0x9c, 0x1c, 0x1e, 0x1f, 0x13, 0x0b,  
          0xa3, 0xc2, 0xe1, 0xec, 0xf4, 0xfc, 0x9c, 0x1c, 0x1e, 0x1f, 0x13, 0x0b, 
          0xa3, 0xc2, 0xe1, 0xec, 0xf4, 0xfc, 0x9c, 0x1c, 0x1e, 0x1f, 0x13, 0x0b, }
    };
    //  // Colors gradiated for 11 per row
    //  // { 0xa9, 0xad, 0xb1, 0xb5, 0x95, 0x55, 0x5a, 0x32, 0x2a, 0x66, 0xa6,  // these are brighter dim versions
    //     { 0x60, 0x64, 0x68, 0x6c, 0x2c, 0x51, 0x2d, 0x09, 0x06, 0x45, 0x65,
    //       0x60, 0x64, 0x68, 0x6c, 0x2c, 0x51, 0x2d, 0x09, 0x06, 0x45, 0x65,
    //       0x60, 0x64, 0x68, 0x6c, 0x2c, 0x51, 0x2d, 0x09, 0x06, 0x45, 0x65, },
    //     { 0xe9, 0xf1, 0xf9, 0xfd, 0xbd, 0x5d, 0x5e, 0x5b, 0x53, 0x8b, 0xeb,
    //       0xe9, 0xf1, 0xf9, 0xfd, 0xbd, 0x5d, 0x5e, 0x5b, 0x53, 0x8b, 0xeb,
    //       0xe9, 0xf1, 0xf9, 0xfd, 0xbd, 0x5d, 0x5e, 0x5b, 0x53, 0x8b, 0xeb, }
    // };
    bool last[iconcount];
    uint8_t idiot_saturation = 225;  // 170-195 makes nice bright yet distinguishable colors
    uint8_t idiot_hue_offset = 240;
    IdiotLights() {
        for (int i=0; i<iconcount; i++) last[i] = *(vals[i]);
        // set_colors();
    }
    void setup(NeopixelStrip* _neo) {
        myneo = _neo;
        // int n = new_idiot(&(err_sens_alarm[LOST]), "SL", { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x3e, 0x71, 0x59, 0x4d, 0x47, 0x3e })
        for (int i=0; i<iconcount; i++) myneo->newIdiotLight(i, color[ON][i], val(i));
        std::cout << "Idiot lights.. set up " << iconcount << " toggle icons & " << myneo->idiotcount << " neopixel hazard lights" << std::endl;
    }
    bool val(int index) { return *(vals[index]); }
    bool* ptr(int index) { return vals[index]; }
  private:
    NeopixelStrip* myneo;
    void set_colors() {
        for (int i=0; i<iconcount; i++) {
            int division = row_count;
            uint32_t color32 = hsv_to_rgb<uint32_t>((65535 * (uint16_t)(i % division) / division + idiot_hue_offset), idiot_saturation, 255);  // , 0, 220);
            color[ON][i] = color_to_332(color32);  // 5957 = 2^16/11
        }
    }
};