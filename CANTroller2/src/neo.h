#pragma once
#include "utils.h"
#include <NeoPixelBus.h>
#define colortype RgbColor  // RgbwColor
NeoPixelBus<NeoGrbFeature, NeoSk6812Method> neoobj(8, 48);  // NeoWs2812Method, NeoWs2812xMethod, NeoSk6812Method, NeoEsp32Rmt0Ws2812xMethod, NeoEsp32I2s1800KbpsMethod, NeoEsp32I2s1Sk6812Method, 
// Default for esp32 is dma via I2S bus 1 at 800kHz using RMT. Don't know difference between "Ws2812", "Ws2812x", and "Sk6812"
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
    uint8_t heartbright = 18;
    uint8_t hibright = 25;
    uint8_t heartlobright = 1;
    int8_t lomultiplier = 3;  // lobright is hibright divided by this twice
    float desat_of_ten = 0.0;  // out of 10.0
    uint8_t neo_master_brightness = 0xff;
    float correction[3] = { 1.0, 0.9, 1.0 };  // Applied to brightness of rgb elements
    uint32_t neo_fade_timeout_us = 380000;
    Timer neoFadeTimer, neoHeartbeatTimer;
    bool neo_heartbeat = false;
    uint8_t pin = -1;
    uint8_t heartbeat_brightness, heartbeat_brightness_last; // brightness during fadeouts
    uint32_t neobright_last;
    int32_t heartbeat_state = 0;
    int32_t heartbeat_level = 0;
    int64_t heartbeat_ekg_us[4] = {250000, 200000, 620000, 2000000};  // {187500, 125000, 562500, 1250000};
    int32_t heartbeat_pulse = 255;
    static const uint8_t idiotcount = 7;
    static const uint8_t numpixels = 1 + idiotcount;  //  + extIdiotCount;  // 15 pixels = heartbeat RGB + 7 onboard RGB + 7 external RGBW
    colortype neostrip[numpixels];
    colortype heartbeatColor, heartbeatNow, heartbeatColor_last;
    Timer debugtimer;
    bool breadboard = false;
    enum ledset { onoff, fcount, fpulseh, fpulsel, foffbrit, fonbrit, fnumset };  // just a bunch of int variables needed for each of the neo idiot lights
    enum ledcolor { cnow, clast, cnormal, coff, con, cflash, cnumcolors };
    uint8_t fset[idiotcount][fnumset];
    colortype cidiot[idiotcount][cnumcolors];
    uint32_t fquantum_us = 75000;  // time resolution of flashes
    // uint32_t fevmask[idiotcount]; 
    uint32_t fevents[idiotcount];
    uint32_t nowtime_us, nowepoch;  // , fevmask_master;
    Timer flashtimer;
    // setflash(idiot, fcount, fperiod=1, fpulsew=1, foffbrit=0, fonbrit=0): set args, cflashoff, cflash, fevents, fevmask. calc/set cflashoff & cflash
    // update(): update() idiots, refresh
    // update_idiot(idiot): calc nowtime, if fcount=0, set now=coff/con else lookup in fevents/fevmasks and set to cflash/cflashoff

    uint32_t color_Rgb_to_32b(colortype color);  // Convert library color type to 32b 0xRRGGBB format
    colortype color_32b_to_Rgb(uint32_t color);  // Convert 32b 0xRRGGBB format color value to library color type
    uint32_t color_16b_to_32b(uint16_t color565);  // Convert 5-6-5 encoded 16-bit color value to 32b 0xRRGGBB format
    uint16_t color_32b_to_16b(uint32_t color);  // Convert 32b 0xRRGGBB format color to 5-6-5 encoded 16-bit color value
    colortype color_16b_to_Rgb(uint16_t color565);  // Convert 5-6-5 encoded 16-bit color value to library color type
    uint16_t color_Rgb_to_16b(colortype color);  // Convert library color type to 5-6-5 encoded 16-bit color value
    float maxelement(float r, float g, float b);
    float midelement(float r, float g, float b);
    float minelement(float r, float g, float b);
    colortype dimmer(colortype color, uint8_t bright);  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
    colortype desaturate(colortype color, int32_t _desat_of_ten);  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
    colortype hue_to_rgb(uint8_t hue);
    void recolor_idiots(int8_t argidiot = -1);
    // struct hsv { float h; float s; float v; };
    // uint32_t rgb_to_hsv(uint32_t rgb);
    // hsv rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b);
  public:
    neopixelstrip() {}
    void refresh();
    void init(uint8_t argpin, bool argbreadboard=false, bool viewcontext=NITE);
    void calc_lobright();
    void setbright(uint8_t bright_pc);
    void setdesaturation(float _desat_of_ten);  // a way to specify nite or daytime brightness levels
    void heartbeat(bool onoroff);
    void heartbeat_update(uint16_t runmode_color);
    void colorfade_update();
    uint32_t neopixelsAvailable();
    bool newIdiotLight(uint8_t idiot, uint16_t color565, bool startboolstate = 0);
    void setBoolState(uint8_t idiot, bool state);
    void setflash(uint8_t idiot, uint8_t count, uint8_t pulseh=1, uint8_t pulsel=1, uint32_t color=0xffffff, uint8_t onbrit=0);
    // void setflash2(uint8_t idiot, uint8_t count, uint8_t pulseh=1, uint8_t pulsel=1, uint32_t color=0xffffff, uint8_t onbrit=0, uint8_t offbrit=255);
    void update();
    uint32_t idiot_neo_color(uint8_t idiot);
};

uint32_t neopixelstrip::color_Rgb_to_32b(colortype color) {  // Convert library color type to 32b 0xRRGGBB format
    return (static_cast<uint32_t>(color.R) << 16) | (static_cast<uint32_t>(color.G) << 8) | static_cast<uint32_t>(color.B);  // (static_cast<uint32_t>(color.W) << 24) | 
}
colortype neopixelstrip::color_32b_to_Rgb(uint32_t color) {  // Convert 32b 0xRRGGBB format color value to library color type
    return colortype((color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff);  // (static_cast<uint32_t>(color.W) << 24) | 
}
uint32_t neopixelstrip::color_16b_to_32b(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to 32b 0xRRGGBB format
    return ((uint32_t)color565 & 0xf800) << 8 | ((uint32_t)color565 & 0x7e0) << 5 | ((uint32_t)color565 & 0x1f) << 3;
}
uint16_t neopixelstrip::color_32b_to_16b(uint32_t color) {  // Convert 32b 0xRRGGBB format color to 5-6-5 encoded 16-bit color value
    return (int16_t)(((color & 0xf80000) >> 8) | ((color & 0xfc00) >> 5) | (color & 0xf8) >> 3);
}
colortype neopixelstrip::color_16b_to_Rgb(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to library color type
    return colortype((color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3);  // , 0);
}
uint16_t neopixelstrip::color_Rgb_to_16b(colortype color) {  // Convert library color type to 5-6-5 encoded 16-bit color value
    return ((static_cast<uint16_t>(color.R) & 0xf8) << 8) | ((static_cast<uint16_t>(color.G) & 0xfc) << 3) | (((static_cast<uint16_t>(color.B) & 0xf8) >> 3));
}
float neopixelstrip::maxelement(float r, float g, float b) {
    return (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
float neopixelstrip::midelement(float r, float g, float b) {
    return (r >= g) ? ((g >= b) ? g : ((r >= b) ? b : r)) : ((r >= b) ? r : ((b >= g) ? g : b));  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
float neopixelstrip::minelement(float r, float g, float b) {
    return (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
colortype neopixelstrip::dimmer(colortype color, uint8_t bright) {  // brightness 0 is off, 255 is max brightness while retaining same hue and saturation
    int32_t ret[3];
    float rgb[3] = { static_cast<float>(color.R), static_cast<float>(color.G), static_cast<float>(color.B) };
    float fbright = (float)bright / maxelement(rgb[0], rgb[1], rgb[2]);  // max(color.r, color.g, color.b);  // 2.55 = 0xff / 100
    for (int32_t element=0; element<3; element++) ret[element] = constrain((int32_t)(rgb[element] * fbright), 0, 255);
    // printf("D br:%d  inc:%06x  outc:%02x%02x%02x\n", bright, color_Rgb_to_32b(color), (uint8_t)ret[0], (uint8_t)ret[1], (uint8_t)ret[2]);

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
// uint32_t neopixelstrip::rgb_to_hsv(uint32_t rgb) { return rgb_to_hsv((uint8_t)(rgb >> 16) | (uint8_t)((rgb & 0xff00 >> 8)) | (rgb & 0xff)); }
// hsv neopixelstrip::rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b) {
//     // convert RGB values to the range 0-1
//     float ro = (float)r / 255.0;
//     float go = (float)g / 255.0;
//     float bo = (float)b / 255.0;
//     float cmax = maxelement(ro, go, bo);
//     float cmin = minelement(ro, go, bo);
//     float delta = cmax - cmin;
//     float h = 0.0;  // hue
//     if (delta == 0.0) h = 0.0; // undefined, but typically set to 0
//     else if (cmax == ro) h = 60 * fmod(((go - bo) / delta), 6);
//     else if (cmax == go) h = 60 * (((bo - ro) / delta) + 2);
//     else if (cmax == bo) h = 60 * (((ro - go) / delta) + 4);
//     float s = (cmax == 0.0) ? 0.0 : delta / cmax;  // saturation
//     float v = cmax;  // brightness
//     return { h, s, v };
// }
void neopixelstrip::refresh() {
    int32_t numledstowrite = (heartbeatNow != neostrip[0]);
    neostrip[0] = heartbeatNow;
    neoobj.SetPixelColor(0, heartbeatNow);
    for (int32_t idiot=0; idiot<idiotcount; idiot++) {
        if (cidiot[idiot][cnow] != neostrip[idiot+1]) {
            neoobj.SetPixelColor (1+idiot, cidiot[idiot][cnow]);
            neostrip[idiot + 1] = cidiot[idiot][cnow];  // color_32b_to_Rgb(cidiot[idiot][cnow]);
            numledstowrite = 2 + idiot;  // + idiotCount;
        }
    }
    if (numledstowrite) neoobj.Show(numledstowrite);  // This ability to exclude pixels at the end of the strip that haven't changed from the data write is an advantage of neopixelbus over adafruit
}
void neopixelstrip::init(uint8_t argpin, bool argbreadboard, bool viewcontext) {
    pin = argpin;
    // neoobj.NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod>(numpixels, pin);  // <NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> works! NeoGrbwFeature NeoWs2812xMethod NeoEsp32I2s1X8Sk6812Method  NeoEsp32I2s1X8Ws2812xMethod
    breadboard = argbreadboard;
    context = viewcontext;
    calc_lobright();
    std::cout << "Neo init: add LEDs.. ";
    neoobj.Begin();
    heartbeat_brightness = brightlev[context][B_LO];
    neoHeartbeatTimer.set(heartbeat_ekg_us[3]);
    neoFadeTimer.set((int64_t)neo_fade_timeout_us);
    std::cout << "refresh strip.. ";
    flashtimer.set(fquantum_us * 32);
    refresh();
    std::cout << std::endl;
}
void neopixelstrip::calc_lobright() {
    lobright = max(3, (hibright / lomultiplier) / lomultiplier);
}
void neopixelstrip::setbright(uint8_t bright_pc) {  // a way to specify brightness level as a percent
    hibright = (uint8_t)((255.0 * (float)bright_pc) / 100.0);
    calc_lobright();
    if (neo_heartbeat_variable_brightness) {
        heartbright = hibright;  // (uint8_t)((float)hibright * 0.75);
        heartlobright = lobright;
    }
    recolor_idiots();
    // printf("BR br:%d  hi:%d  lo:%d\n", bright_pc, hibright, lobright);

}
void neopixelstrip::setdesaturation(float _desat_of_ten) {  // a way to specify nite or daytime brightness levels
    desat_of_ten = _desat_of_ten;
    recolor_idiots();
}
void neopixelstrip::heartbeat(bool onoroff) {
    neo_heartbeat = onoroff;  // Start heart beating
}
void neopixelstrip::heartbeat_update(uint16_t runmode_color) {
    if (!neo_heartbeat) return;
    heartbeatColor = color_16b_to_Rgb(runmode_color);
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
        else heartbeat_brightness = (int8_t)(heartlobright + max(0, (float)(heartbright - heartlobright) * (1.0 - ((heartbeat_state == 1) ? 1.5 : 1.0) * (float)neoFadeTimer.elapsed() / (float)neo_fade_timeout_us)));
        if (heartbeat_brightness > heartbeat_brightness_last) heartbeat_brightness = heartbeat_brightness_last;
    }
    if (heartbeatColor != heartbeatColor_last || heartbeat_brightness != neobright_last) {
        heartbeatNow = dimmer(heartbeatColor, heartbeat_brightness);  // heartbeatNow = dimmer(desaturate(heartbeatColor, desat_of_ten), heartbeat_brightness);
        debugtimer.set(4000000);
        heartbeatColor_last = heartbeatColor;
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
    cidiot[idiot][cnormal] = color_16b_to_Rgb(color565);
    cidiot[idiot][clast] = color_32b_to_Rgb(0);
    fset[idiot][fcount] = 0;
    setBoolState(idiot, startboolstate);
    fevents[idiot] = 0;
    recolor_idiots(idiot);
    // updateidiot(idiot);
    // printf ("N i:%d v:%d nt:%ld ne:%d cn:0x%06x cw:0x%06x cl:0x%06x cf1:0x%06x cf0:0x%06x ev:0x%08x ph:%d pl:%d\n", idiot, fset[idiot][onoff], 0, 0,
    //     color_Rgb_to_32b(cidiot[idiot][cnormal]), color_Rgb_to_32b(cidiot[idiot][cnow]), color_Rgb_to_32b(cidiot[idiot][clast]), color_Rgb_to_32b(cidiot[idiot][cflash]), color_Rgb_to_32b(cidiot[idiot][cflashoff]), fevents[idiot], fset[idiot][fpulseh], fset[idiot][fpulsel] );
    return true;
}
void neopixelstrip::setBoolState(uint8_t idiot, bool state) {
    fset[idiot][onoff] = state;
    // cidiot[idiot][cflashoff] = (fset[idiot][onoff]) ? cidiot[idiot][con], cidiot[idiot][coff];
}
void neopixelstrip::setflash(uint8_t idiot, uint8_t count, uint8_t pulseh, uint8_t pulsel, uint32_t color, uint8_t onbrit) {
    fset[idiot][fcount] = count;
    fset[idiot][fpulseh] = min(pulseh, 1);
    fset[idiot][fpulsel] = min(pulsel, 1);
    fset[idiot][fonbrit] = onbrit;
    fset[idiot][foffbrit] = hibright;
    // cidiot[idiot][cflash] = desaturate(dimmer(color_32b_to_Rgb(color), hibright + (fset[idiot][fonbrit] * (255 - hibright))/255), desat_of_ten);
    cidiot[idiot][cflash] = dimmer(color_32b_to_Rgb(color), hibright);
    cidiot[idiot][cflash] = desaturate(dimmer(cidiot[idiot][cflash], fset[idiot][fonbrit]), desat_of_ten);
    // cidiot[idiot][cflashoff] = (fset[idiot][onoff]) ? cidiot[idiot][con], cidiot[idiot][coff];
    fevents[idiot] = 0;
    uint32_t filled = 0;
    while (filled < fset[idiot][fcount] * (fset[idiot][fpulseh] + fset[idiot][fpulsel])) {    
        for (uint8_t bt = 0; bt < fset[idiot][fpulseh]; bt++) {
            fevents[idiot] |= (1 << (bt + filled));
            filled++;
        }
        filled += fset[idiot][fpulsel];
    }
    // printf ("S i:%d v:%d nt:%ld ne:%d cn:0x%06x cw:0x%06x cl:0x%06x cf1:0x%06x cf0:0x%06x ev:0x%08x ph:%d pl:%d\n", idiot, fset[idiot][onoff], 0, 0,
    //     color_Rgb_to_32b(cidiot[idiot][cnormal]), color_Rgb_to_32b(cidiot[idiot][cnow]), color_Rgb_to_32b(cidiot[idiot][clast]), color_Rgb_to_32b(cidiot[idiot][cflash]), color_Rgb_to_32b(cidiot[idiot][cflashoff]),
    //     fevents[idiot], fset[idiot][fpulseh], fset[idiot][fpulsel] );
}
// void neopixelstrip::setflash2(uint8_t idiot, uint8_t count, uint8_t pulseh, uint8_t pulsel, uint32_t color, uint8_t onbrit, uint8_t offbrit) {
//     setflash(idiot, count, pulseh, pulsel, color, onbrit);
//     fset[idiot][foffbrit] = dimmer(cidiot[idiot][cnormal], offbrit);
//     cidiot[idiot][cflashoff] = desaturate(dimmer(cidiot[idiot][cnormal], fset[idiot][foffbrit]), desat_of_ten);
// }
void neopixelstrip::update() {
    nowtime_us = (uint32_t)flashtimer.elapsed();
    nowepoch = nowtime_us / fquantum_us;
    for (int32_t idiot=0; idiot<idiotcount; idiot++) {
        cidiot[idiot][clast] = cidiot[idiot][cnow];
        colortype newnow = (fset[idiot][onoff]) ? cidiot[idiot][con] : cidiot[idiot][coff];
        if (!fset[idiot][fcount]) cidiot[idiot][cnow] = newnow;
        else cidiot[idiot][cnow] = ((fevents[idiot] >> nowepoch) & 1) ? cidiot[idiot][cflash] : newnow;
        // if (idiot == 4 && (cidiot[idiot][clast] != cidiot[idiot][cnow])) {
        //     printf ("U i:%d 01:%d flv:%d ct:%d, nt:%ld ne:%d cn:0x%06x cw:0x%06x cl:0x%06x cf1:0x%06x c1:0x%06x c0:0x%06x ev:0x%08x ph:%d pl:%d\n", idiot, fset[idiot][onoff], fset[idiot][fcount], ((fevents[idiot] >> nowepoch) & 1), nowtime_us, nowepoch,
        //         color_Rgb_to_32b(cidiot[idiot][cnormal]), color_Rgb_to_32b(cidiot[idiot][cnow]), color_Rgb_to_32b(cidiot[idiot][clast]), color_Rgb_to_32b(cidiot[idiot][cflash]), color_Rgb_to_32b(cidiot[idiot][con]), color_Rgb_to_32b(cidiot[idiot][coff]),
        //         fevents[idiot], fset[idiot][fpulseh], fset[idiot][fpulsel] );
        // }
    }
    refresh();
        // for (int32_t idiot=0; idiot<idiotcount; idiot++) 
        //     printf ("U2 i:%d v:%d nt:%ld ne:%d\n",idiot, ((fevents[idiot] >> nowepoch) & 1), nowtime_us, nowepoch);
    flashtimer.expireset();
}
uint32_t neopixelstrip::idiot_neo_color(uint8_t idiot) { return color_Rgb_to_32b(cidiot[idiot][cnow]); }