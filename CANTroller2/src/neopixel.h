#pragma once
#include <NeoPixelBus.h>
#define colortype RgbColor  // RgbwColor
#define striplength 8
NeoPixelBus<NeoGrbFeature, NeoSk6812Method> neoobj(striplength, neopixel_pin);  // NeoWs2812Method, NeoWs2812xMethod, NeoSk6812Method, NeoEsp32Rmt0Ws2812xMethod, NeoEsp32I2s1800KbpsMethod, NeoEsp32I2s1Sk6812Method, 
// Default for esp32 is dma via I2S bus 1 at 800kHz using RMT. Don't know the protocol difference between "Ws2812", "Ws2812x", and "Sk6812"
// Run neos in a task example: https://github.com/Makuna/NeoPixelBus/wiki/ESP32-and-RTOS-Tasks
class neopixelstrip {
  private:
    enum brightness_presets { B_OFF, B_MIN, B_LO, B_MED, B_HI, B_EXT, B_MAX };
    uint8_t neo_wheelcounter = 0;
    enum brightness_contexts { NITE, DAY };  // Indoors = NITE
    uint8_t brightlev[2][7] = { { 0, 1,  6, 10, 17, 30,  50 },     // [NITE] [B_OFF/B_MIN/B_LOW/B_MED/B_HIGH/B_EXT/B_MAX]
                                { 0, 2, 16, 30, 45, 65, 100 }, };  // [DAY] [B_OFF/B_MIN/B_LOW/B_MED/B_HIGH/B_EXT/B_MAX]
    bool context = NITE;
    bool neo_heartbeat_variable_brightness = false;  // If false then brightness control only affect idiotlights
    uint8_t lobright;
    uint8_t heartbright = 22;
    uint8_t hibright = 35;
    uint8_t heartlobright = 2;
    int8_t lomultiplier = 3;  // lobright is hibright divided by this twice
    float desat_of_ten = 0.0;  // out of 10.0
    uint8_t neo_master_brightness = 0xff;
    float correction[3] = { 1.0, 0.9, 1.0 };  // Applied to brightness of rgb elements
    uint32_t neo_fade_timeout_us = 380000;
    Timer neoFadeTimer, neoHeartbeatTimer;
    bool neo_heartbeat = false;
    bool heartcolor_change = true;
    uint8_t pin = -1;
    uint8_t heartbeat_brightness, heartbeat_brightness_last; // brightness during fadeouts
    uint32_t neobright_last;
    int32_t heartbeat_state = 0;
    int32_t heartbeat_level = 0;
    int64_t heartbeat_ekg_us[4] = {250000, 240000, 620000, 2000000};  // {187500, 125000, 562500, 1250000};
    int32_t heartbeat_pulse = 255;
    static const uint8_t idiotcount = 7;
    static const uint8_t numpixels = 1 + idiotcount;  //  + extIdiotCount;  // 15 pixels = heartbeat RGB + 7 onboard RGB + 7 external RGBW
    colortype neostrip[numpixels];
    colortype heartbeatColor, heartbeatNow;
    uint16_t heartcolor16 = 0x0000;  // blackened heart
    bool breadboard = false;
    enum ledset { onoff, fcount, fpulseh, fpulsel, fonbrit, fnumset };  // just a bunch of int variables needed for each of the neo idiot lights
    enum ledcolor { cnow, clast, cnormal, coff, con, cflash, cnumcolors };
    uint8_t fset[idiotcount][fnumset];
    colortype cidiot[idiotcount][cnumcolors];
    uint32_t fquantum_us = 50000;  // time resolution of flashes
    static const uint8_t fevresolution = 128;
    static const uint8_t fevpages = 4;  // fevresolution / 32;  <- but if I use that math instead, seg fault
    uint32_t fevents[idiotcount][fevpages];
    uint8_t fevcurrpage = 0; uint8_t fevfilled = 0; uint32_t nowtime_us, nowepoch;  // , fevmask_master;
    Timer flashtimer;
    uint32_t color_to_32b(colortype color);  // Convert library color type to 32b 0xRRGGBB format
    uint32_t color_to_32b(uint16_t color565);  // Convert 5-6-5 encoded 16-bit color value to 32b 0xRRGGBB format
    colortype color_to_Rgb(uint16_t color565);  // Convert 5-6-5 encoded 16-bit color value to library color type
    colortype color_to_Rgb(uint32_t color);  // Convert 32b 0xRRGGBB format color value to library color type
    uint16_t color_to_16b(uint32_t color);  // Convert 32b 0xRRGGBB format color to 5-6-5 encoded 16-bit color value
    uint16_t color_to_16b(colortype color);  // Convert library color type to 5-6-5 encoded 16-bit color value
    float maxelement(float r, float g, float b);
    float midelement(float r, float g, float b);
    float minelement(float r, float g, float b);
    colortype dimmer(colortype color, uint8_t bright);  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
    colortype desaturate(colortype color, int32_t _desat_of_ten);  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
    colortype hue_to_rgb(uint8_t hue);
    void recolor_idiots(int8_t argidiot = -1);
    bool fevpop(uint8_t idiot, int8_t pop_off);
    void fevpush(uint8_t idiot, int8_t push_off, bool push_val);
    void update_idiot(uint32_t idiot);
    void calc_lobright();
  public:
    neopixelstrip() {}
    void refresh();
    void init(uint8_t argpin, bool argbreadboard=false, bool viewcontext=NITE);
    void setbright(uint8_t bright_pc);
    void setdesaturation(float _desat_of_ten);  // a way to specify nite or daytime brightness levels
    void heartbeat(bool onoroff);
    void set_heartcolor(uint16_t newcolor);
    void heartbeat_update();
    void colorfade_update();
    uint32_t neopixelsAvailable();
    bool newIdiotLight(uint8_t idiot, uint16_t color565, bool startboolstate = 0);
    void setBoolState(uint8_t idiot, bool state);
    void setflash(uint8_t idiot, uint8_t count, uint8_t pulseh=1, uint8_t pulsel=1, int32_t onbrit=-1, int32_t color=0xffffff);
    void update(bool blackout=false);
    uint32_t idiot_neo_color(uint8_t idiot);
};

uint32_t neopixelstrip::color_to_32b(colortype color) {  // Convert library color type to 32b 0xRRGGBB format
    return (static_cast<uint32_t>(color.R) << 16) | (static_cast<uint32_t>(color.G) << 8) | static_cast<uint32_t>(color.B);  // (static_cast<uint32_t>(color.W) << 24) | 
}
uint32_t neopixelstrip::color_to_32b(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to 32b 0xRRGGBB format
    return ((uint32_t)color565 & 0xf800) << 8 | ((uint32_t)color565 & 0x7e0) << 5 | ((uint32_t)color565 & 0x1f) << 3;
}
colortype neopixelstrip::color_to_Rgb(uint32_t color) {  // Convert 32b 0xRRGGBB format color value to library color type
    return colortype((color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff);  // (static_cast<uint32_t>(color.W) << 24) | 
}
colortype neopixelstrip::color_to_Rgb(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to library color type
    return colortype((color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3);  // , 0);
}
uint16_t neopixelstrip::color_to_16b(uint32_t color) {  // Convert 32b 0xRRGGBB format color to 5-6-5 encoded 16-bit color value
    return (int16_t)(((color & 0xf80000) >> 8) | ((color & 0xfc00) >> 5) | (color & 0xf8) >> 3);
}
uint16_t neopixelstrip::color_to_16b(colortype color) {  // Convert library color type to 5-6-5 encoded 16-bit color value
    return ((static_cast<uint16_t>(color.R) & 0xf8) << 8) | ((static_cast<uint16_t>(color.G) & 0xfc) << 3) | (((static_cast<uint16_t>(color.B) & 0xf8) >> 3));
}
float neopixelstrip::maxelement(float r, float g, float b) {
    return (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //smax(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
float neopixelstrip::midelement(float r, float g, float b) {
    return (r >= g) ? ((g >= b) ? g : ((r >= b) ? b : r)) : ((r >= b) ? r : ((b >= g) ? g : b));  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //smax(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
float neopixelstrip::minelement(float r, float g, float b) {
    return (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //smax(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
colortype neopixelstrip::dimmer(colortype color, uint8_t bright) {  // brightness 0 is off, 255 is max brightness while retaining same hue and saturation
    int32_t ret[3];
    float rgb[3] = { static_cast<float>(color.R), static_cast<float>(color.G), static_cast<float>(color.B) };
    float fbright = (float)bright / maxelement(rgb[0], rgb[1], rgb[2]);  // smax(color.r, color.g, color.b);  // 2.55 = 0xff / 100
    for (int32_t element=0; element<3; element++) ret[element] = constrain((int32_t)(rgb[element] * fbright), 0, 255);
    // printf("D br:%d  inc:%06x  outc:%02x%02x%02x\n", bright, color_to_32b(color), (uint8_t)ret[0], (uint8_t)ret[1], (uint8_t)ret[2]);
    return colortype(ret[0], ret[1], ret[2]);  // return CRGB((float)(color.r * fbright), (float)(color.g * fbright), (float)(color.b * fbright));
}
colortype neopixelstrip::desaturate(colortype color, int32_t _desat_of_ten) {  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
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
    return colortype(rgb[0], rgb[1], rgb[2]);
}
colortype neopixelstrip::hue_to_rgb(uint8_t hue) {
    uint32_t rgb[3] = { 255 - 3 * (uint32_t)((255 - hue) % 85), 0, 3 * (uint32_t)((255 - hue) % 85) };
    if (hue <= 85) { rgb[1] = rgb[0]; rgb[0] = rgb[2]; rgb[2] = 0; }
    else if (hue <= 170) { rgb[1] = rgb[2]; rgb[2] = rgb[0]; rgb[0] = 0; }
    return colortype(rgb[0], rgb[1], rgb[2]);
}
void neopixelstrip::recolor_idiots(int8_t argidiot) {  // pass in -1 to recolor all idiots
    int8_t start = (argidiot >= 0) ? argidiot : 0;
    int8_t end = (argidiot >= 0) ? argidiot + 1 : idiotcount;
    for (int8_t idiot = start; idiot < end; idiot++) {
        cidiot[idiot][con] = dimmer(cidiot[idiot][cnormal], hibright);
        cidiot[idiot][con] = desaturate(cidiot[idiot][con], desat_of_ten);
        cidiot[idiot][coff] = dimmer(cidiot[idiot][cnormal], lobright);
        cidiot[idiot][coff] = desaturate(cidiot[idiot][coff], desat_of_ten);
    }
}
void neopixelstrip::refresh() {
    int32_t numledstowrite = (heartbeatNow != neostrip[0]);
    neostrip[0] = heartbeatNow;
    neoobj.SetPixelColor(0, heartbeatNow);
    for (int32_t idiot=0; idiot<idiotcount; idiot++) {
        if (cidiot[idiot][cnow] != neostrip[idiot+1]) {
            neoobj.SetPixelColor (1+idiot, cidiot[idiot][cnow]);
            neostrip[idiot + 1] = cidiot[idiot][cnow];  // color_to_Rgb(cidiot[idiot][cnow]);
            numledstowrite = 2 + idiot;  // + idiotCount;
        }
    }
    if (numledstowrite) neoobj.Show(numledstowrite);  // This ability to exclude pixels at the end of the strip that haven't changed from the data write is an advantage of neopixelbus over adafruit
}
void neopixelstrip::init(uint8_t argpin, bool argbreadboard, bool viewcontext) {
    pin = argpin;
    breadboard = argbreadboard;
    context = viewcontext;
    calc_lobright();
    std::cout << "Neo init: add LEDs.. ";
    neoobj.Begin();
    heartbeat_brightness = brightlev[context][B_LO];
    neoHeartbeatTimer.set(heartbeat_ekg_us[3]);
    neoFadeTimer.set((int64_t)neo_fade_timeout_us);
    std::cout << "refresh strip.. ";
    flashtimer.set(fquantum_us * fevresolution);
    refresh();
    std::cout << std::endl;
}
void neopixelstrip::calc_lobright() {
    lobright = smax(3, (hibright / lomultiplier) / lomultiplier);
}
void neopixelstrip::setbright(uint8_t bright_pc) {  // a way to specify brightness level as a percent
    hibright = (uint8_t)((255.0 * (float)bright_pc) / 100.0);
    calc_lobright();
    if (neo_heartbeat_variable_brightness) {
        heartbright = hibright;  // (uint8_t)((float)hibright * 0.75);
        heartlobright = lobright;
    }
    recolor_idiots();
}
void neopixelstrip::setdesaturation(float _desat_of_ten) {  // a way to specify nite or daytime brightness levels
    desat_of_ten = _desat_of_ten;
    recolor_idiots();
}
void neopixelstrip::heartbeat(bool onoroff) {
    neo_heartbeat = onoroff;  // Start heart beating
}
void neopixelstrip::set_heartcolor(uint16_t newcolor) {
    if (heartcolor16 != newcolor) {
        heartbeatColor = color_to_Rgb(newcolor);
        heartcolor_change = true;
        heartcolor16 = newcolor;
    }
}
void neopixelstrip::heartbeat_update() {
    if (!neo_heartbeat) return;
    if (neoHeartbeatTimer.expired()) {
        heartbeat_pulse = !heartbeat_pulse;
        if (++heartbeat_state >= arraysize(heartbeat_ekg_us)) heartbeat_state -= arraysize(heartbeat_ekg_us);
        neoHeartbeatTimer.set(heartbeat_ekg_us[heartbeat_state]);
        if (heartbeat_pulse) heartbeat_brightness = heartbright;
        else neoFadeTimer.reset();
    }
    else if (!heartbeat_pulse) {
        heartbeat_brightness_last = heartbeat_brightness;
        if (neoFadeTimer.expired()) heartbeat_brightness = brightlev[context][B_MIN];
        else heartbeat_brightness = (int8_t)(heartlobright + smax(0, (float)(heartbright - heartlobright) * (1.0 - ((heartbeat_state == 1) ? 1.5 : 1.0) * (float)neoFadeTimer.elapsed() / (float)neo_fade_timeout_us)));
        if (heartbeat_brightness > heartbeat_brightness_last) heartbeat_brightness = heartbeat_brightness_last;
    }
    if (heartcolor_change || heartbeat_brightness != neobright_last) {
        heartbeatNow = dimmer(heartbeatColor, heartbeat_brightness);  // heartbeatNow = dimmer(desaturate(heartbeatColor, desat_of_ten), heartbeat_brightness);
        heartcolor_change = false;
        neobright_last = heartbeat_brightness;
    }
}
void neopixelstrip::colorfade_update() {
    if (neoFadeTimer.expireset()) {
        heartbeatNow = hue_to_rgb(++neo_wheelcounter);
        neoobj.SetPixelColor(0, heartbeatNow);
    }
}
uint32_t neopixelstrip::neopixelsAvailable() {
    return idiotcount;
}
bool neopixelstrip::newIdiotLight(uint8_t idiot, uint16_t color565, bool startboolstate) {
    if (idiot > idiotcount-1) return false;
    fset[idiot][onoff] = startboolstate;
    cidiot[idiot][cnormal] = color_to_Rgb(color565);
    cidiot[idiot][clast] = color_to_Rgb((uint32_t)0);
    fset[idiot][fcount] = 0;
    setBoolState(idiot, startboolstate);
    for (uint8_t pg = 0; pg < fevpages; pg++) fevents[idiot][pg] = 0;
    recolor_idiots(idiot);
    return true;
}
void neopixelstrip::setBoolState(uint8_t idiot, bool state) {
    fset[idiot][onoff] = state;
}
bool neopixelstrip::fevpop(uint8_t idiot, int8_t pop_off) {
    int8_t page = pop_off / 32;
    return (fevents[idiot][page] >> (pop_off - 32 * page)) & 1;  // pop a bit off the stack and return it
}
void neopixelstrip::fevpush(uint8_t idiot, int8_t push_off, bool push_val) {
    int8_t page = push_off / 32;
    fevents[idiot][page] |= (push_val << (push_off - 32 * page));
}
void neopixelstrip::setflash(uint8_t idiot, uint8_t count, uint8_t pulseh, uint8_t pulsel, int32_t onbrit, int32_t color) {
    fset[idiot][fcount] = count;
    fset[idiot][fpulseh] = smax(pulseh, 1);
    fset[idiot][fpulsel] = smax(pulsel, 1);
    fset[idiot][fonbrit] = (onbrit == -1) ? hibright : (uint8_t)onbrit;
    cidiot[idiot][cflash] = dimmer((color == -1) ? cidiot[idiot][cnormal] : color_to_Rgb((uint32_t)color), hibright);
    cidiot[idiot][cflash] = desaturate(dimmer(cidiot[idiot][cflash], fset[idiot][fonbrit]), desat_of_ten);
    for (uint8_t pg = 0; pg < fevpages; pg++) fevents[idiot][pg] = 0;
    uint8_t filled = 0;
    uint8_t lstop;
    uint8_t patternlen = fset[idiot][fcount] * (fset[idiot][fpulseh] + fset[idiot][fpulsel]);
    uint8_t reps = 1 + (patternlen < fevresolution / 3);  // For shorter flash patterns repeat them multiple times in each cycle
    for (uint8_t rep = 1; rep <= reps; rep++) {        
        lstop = smin(fevresolution, filled + patternlen);
        while (filled < lstop) {
            for (uint8_t hbit = 0; hbit < fset[idiot][fpulseh]; hbit++)
                if (filled < lstop) fevpush(idiot, filled++, 1);
            for (uint8_t lbit = 0; lbit < fset[idiot][fpulsel]; lbit++)
                if (filled < lstop) fevpush(idiot, filled++, 0); 
        }
        filled = fevresolution / reps;
    }
}
uint32_t neopixelstrip::idiot_neo_color(uint8_t idiot) { 
    return color_to_32b(cidiot[idiot][cnow]);
}
void neopixelstrip::update_idiot(uint32_t idiot) {
    cidiot[idiot][clast] = cidiot[idiot][cnow];
    colortype newnow = (fset[idiot][onoff]) ? cidiot[idiot][con] : cidiot[idiot][coff];
    if (!fset[idiot][fcount]) cidiot[idiot][cnow] = newnow;
    else cidiot[idiot][cnow] = fevpop(idiot, nowepoch) ? cidiot[idiot][cflash] : newnow;
}
void neopixelstrip::update(bool blackout) {
    heartbeat_update();  // Update our beating heart
    nowtime_us = (uint32_t)flashtimer.elapsed();
    nowepoch = nowtime_us / fquantum_us;
    for (int32_t idiot=0; idiot<idiotcount; idiot++) {
        if (blackout) cidiot[idiot][cnow] = colortype(0);
        else update_idiot(idiot);
    }
    refresh();
    flashtimer.expireset();
}