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
    uint8_t lobright = 1;
    uint8_t heartbright = 6;
    uint8_t hibright = 6;
    uint8_t heartlobright = 0;
    float desatlevel = 0.0;  // out of 10.0
    uint8_t neo_master_brightness = 0xff;
    float correction[3] = { 1.0, 0.9, 1.0 };  // Applied to brightness of rgb elements
    uint32_t neo_fade_timeout_us = 380000;
    Timer neoFadeTimer, neoHeartbeatTimer;
    bool neo_heartbeat = false;
    uint8_t pin = -1;
    uint8_t heartbeat_brightness; // brightness during fadeouts
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

    enum ledset { onoff, fcount, fperiod, fpulsew, foffbrit, fonbritadd, fnumset };  // just a bunch of int variables needed for each of the neo idiot lights
    enum ledcolor { cnow, clast, cnormal, coff, con, cflashoff, cflashon, cnumcolors };
    uint8_t fset[idiotcount][fnumset];
    colortype cidiot[idiotcount][cnumcolors];
    uint32_t fquantum_us = 50000;  // time resolution of flashes
    // uint32_t fevmask[idiotcount]; 
    uint32_t fevents[idiotcount];
    uint32_t nowtime_us, nowepoch;  // , fevmask_master;
    Timer flashtimer;
    // setflash(idiot, fcount, fperiod=1, fpulsew=1, foffbrit=0, fonbritadd=0): set args, cflashoff, cflashon, fevents, fevmask. calc/set cflashoff & cflashon
    // update(): update() idiots, refresh
    // update_idiot(idiot): calc nowtime, if fcount=0, set now=coff/con else lookup in fevents/fevmasks and set to cflashon/cflashoff

    uint32_t color_Rgb_to_32b(colortype color);  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
    uint32_t color_16b_to_32b(uint16_t color565);  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
    uint16_t color_32b_to_16b(uint32_t color);  // Convert library color type to 5-6-5 encoded 16-bit color value
    colortype color_16b_to_Rgb(uint16_t color565);  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
    uint16_t color_Rgb_to_16b(colortype color);  // Convert library color type to 5-6-5 encoded 16-bit color value
    float maxelement(float r, float g, float b);
    float midelement(float r, float g, float b);
    float minelement(float r, float g, float b);
    colortype dimmer(colortype color, int8_t bright_pc);  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
    colortype desaturate(colortype color, int32_t desat_of_ten);  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
    colortype colorwheel(uint8_t WheelPos);
    void recolor_idiot(int8_t argidiot = -1);
    void recolor_idiots();
    // struct hsv { float h; float s; float v; };
    // uint32_t rgb_to_hsv(uint32_t rgb);
    // hsv rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b);
  public:
    neopixelstrip() {}
    void refresh();
    void init(uint8_t argpin, bool argbreadboard=false, bool viewcontext=NITE);
    void setbright(int8_t newlev);
    void setdesaturation(float newlev);  // a way to specify nite or daytime brightness levels
    void heartbeat(bool onoroff);
    void heartbeat_update(uint16_t runmode_color);
    void colorfade_update();
    uint32_t neopixelsAvailable();
    bool newIdiotLight(uint8_t idiot, uint16_t color565, bool startboolstate = 0);
    void setBoolState(uint8_t idiot, bool state);
    void setflash(uint8_t idiot, uint8_t count, uint8_t period, uint8_t pulsew, uint32_t color, uint8_t offbrit, uint8_t onbritadd);
    void update();
    uint32_t idiot_neo_color(uint8_t idiot);
};

uint32_t neopixelstrip::color_Rgb_to_32b(colortype color) {  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
    return (static_cast<uint32_t>(color.R) << 16) | (static_cast<uint32_t>(color.G) << 8) | static_cast<uint32_t>(color.B);  // (static_cast<uint32_t>(color.W) << 24) | 
}
uint32_t neopixelstrip::color_16b_to_32b(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
    return ((uint32_t)color565 & 0xf800) << 8 | ((uint32_t)color565 & 0x7e0) << 5 | ((uint32_t)color565 & 0x1f) << 3;
}
uint16_t neopixelstrip::color_32b_to_16b(uint32_t color) {  // Convert library color type to 5-6-5 encoded 16-bit color value
    return (int16_t)(((color & 0xf80000) >> 8) | ((color & 0xfc00) >> 5) | (color & 0xf8) >> 3);
}
colortype neopixelstrip::color_16b_to_Rgb(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to type suitable for library
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
colortype neopixelstrip::dimmer(colortype color, int8_t bright_pc) {  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
    float rgb[3] = { static_cast<float>(color.R), static_cast<float>(color.G), static_cast<float>(color.B) };
    float fbright = (float)bright_pc * 2.55 / maxelement(rgb[0], rgb[1], rgb[2]);  // max(color.r, color.g, color.b);  // 2.55 = 0xff / 100
    float sat = 1;  // 1 - desatlevel * desatlevel / 100.0;
    float c[3] = { correction[0] * sat, correction[1] * sat, correction[2] * sat };
    for (int32_t element=0; element<3; element++)
        rgb[element] *= fbright * c[element];
    return colortype(rgb[0], rgb[1], rgb[2]);  // return CRGB((float)(color.r * fbright), (float)(color.g * fbright), (float)(color.b * fbright));
}
colortype neopixelstrip::desaturate(colortype color, int32_t desat_of_ten) {  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
    uint8_t rgb[3] = { static_cast<uint8_t>(color.R), static_cast<uint8_t>(color.G), static_cast<uint8_t>(color.B) };
    float dominant = maxelement(rgb[0], rgb[1], rgb[2]);
    if (desat_of_ten > 0) {
        for (int element=0; element<3; element++)
            rgb[element] = (uint8_t)(rgb[element] + ((float)desat_of_ten * (dominant - (float)(rgb[element])) / 20.0));
    }
    // else {
    //     // float mide = midelement(rgb[0], rgb[1], rgb[2]);
    //     dominant = minelement(rgb[0], rgb[1], rgb[2]);
    //     for (int element=0; element<3; element++)
    //         rgb[element] = (uint8_t)(rgb[element] - ((float)desat_of_ten * dominant / 10.0));
    // }
    return colortype(rgb[0], rgb[1], rgb[2]);
}
colortype neopixelstrip::colorwheel(uint8_t WheelPos) {
    WheelPos = 255 - WheelPos;
    int rgb[3];
    if (WheelPos < 85) {
        rgb[0] = 255 - WheelPos * 3; rgb[1] = 0; rgb[2] = WheelPos * 3;
    }
    else if (WheelPos < 170) {
        WheelPos -= 85;
        rgb[0] = 0; rgb[1] = WheelPos * 3; rgb[2] = 255 - WheelPos * 3;
    }
    else {
        WheelPos -= 170;
        rgb[0] = WheelPos * 3; rgb[1] = 255 - WheelPos * 3; rgb[2] = 0;
    }
    return colortype(rgb[0], rgb[1], rgb[2]);
}
void neopixelstrip::recolor_idiot(int8_t argidiot) {  // pass in -1 to recolor all idiots
    int8_t start = (argidiot >= 0) ? argidiot : 0;
    int8_t end = (argidiot >= 0) ? argidiot + 1 : idiotcount;
    for (int8_t idiot = start; idiot < end; idiot++) {
        cidiot[idiot][con] = dimmer(cidiot[idiot][cnormal], hibright);
        cidiot[idiot][con] = desaturate(cidiot[idiot][con], desatlevel);
        cidiot[idiot][coff] = dimmer(cidiot[idiot][cnormal], lobright);
        cidiot[idiot][coff] = desaturate(cidiot[idiot][con], desatlevel);
    }
}
void neopixelstrip::recolor_idiots() {
    for (int32_t idiot=0; idiot<idiotcount; idiot++) recolor_idiot(idiot);
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
            neostrip[idiot + 1] = cidiot[idiot][cnow];  // colortype(cidiot[idiot][cnow]);
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
void neopixelstrip::setbright(int8_t newlev) {  // a way to specify brightness levels
    hibright = newlev;
    lobright = (hibright > 50) ? 3 : (hibright > 25) ? 2 : 1;
    if (neo_heartbeat_variable_brightness) {
        heartbright = hibright;  // (uint8_t)((float)hibright * 0.75);
        heartlobright = lobright;
    }
    recolor_idiots();
}
void neopixelstrip::setdesaturation(float newlev) {  // a way to specify nite or daytime brightness levels
    desatlevel = newlev;
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
        if (neoFadeTimer.expired()) heartbeat_brightness = brightlev[context][B_MIN];
        else heartbeat_brightness = (int8_t)(heartlobright + max(1, (float)(heartbright - heartlobright) * (1.0 - ((heartbeat_state == 1) ? 1.5 : 1.0) * (float)neoFadeTimer.elapsed() / (float)neo_fade_timeout_us)));
    }
    if (heartbeatColor != heartbeatColor_last || heartbeat_brightness != neobright_last) {
        heartbeatNow = dimmer(heartbeatColor, heartbeat_brightness);  // heartbeatNow = dimmer(desaturate(heartbeatColor, desatlevel), heartbeat_brightness);
        debugtimer.set(4000000);
        heartbeatColor_last = heartbeatColor;
        neobright_last = heartbeat_brightness;
    }
}
void neopixelstrip::colorfade_update() {
    if (neoFadeTimer.expireset()) {
        heartbeatNow = colorwheel(++neo_wheelcounter);
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
    cidiot[idiot][clast] = colortype(0x00, 0x00, 0x00);
    fset[idiot][fcount] = 0;
    setBoolState(idiot, startboolstate);
    recolor_idiot(idiot);
    // updateidiot(idiot);
    return true;
}
void neopixelstrip::setBoolState(uint8_t idiot, bool state) {
    fset[idiot][onoff] = state;
}
void neopixelstrip::setflash(uint8_t idiot, uint8_t count, uint8_t pulseh, uint8_t pulsel, uint32_t color, uint8_t offbritadd, uint8_t offbrit) {
    fset[idiot][fcount] = count;
    fset[idiot][fpulseh] = pulseh;
    fset[idiot][fpulsel] = pulsel;
    fset[idiot][fonbritadd] = onbritadd;
    fset[idiot][foffbrit] = offbrit;
    cidiot[idiot][cflashoff] = desaturate(dimmer(cidiot[idiot][cnormal], fset[idiot][foffbrit]), desatlevel);
    cidiot[idiot][cflashon] = desaturate(dimmer(colortype(color), hibright + (fset[idiot][fonbritadd] * (255 - hibright))/255), desatlevel);
    fevents[idiot] = 0;
    // fevmask[idiot] = 0;
    uint32_t nowval = 0;
    for (uint32_t epoch = 0; epoch < 32; epoch++) {
        if ((epoch < fset[idiot][fcount] * 2 * fset[idiot][period]) && (epoch % period == 0)) {
            nowval = !nowval;
            // fevmask[idiot] = fevmask[idiot] | (1 << epoch);
        }
        fevents[idiot] = fevents[idiot] | (nowval << epoch);
    }
}
void neopixelstrip::update() {
    for (int32_t idiot=0; idiot<idiotcount; idiot++) {
        nowtime_us = (uint32_t)flashtimer.elapsed();
        cidiot[idiot][clast] = cidiot[idiot][cnow];
        if (!fset[idiot][fcount])
            cidiot[idiot][cnow] = (fset[idiot][onoff]) ? cidiot[idiot][con] : cidiot[idiot][coff];
        else cidiot[idiot][cnow] == ((fevents[idiot] >> nowepoch) & 1) ? cidiot[idiot][cflashon] : cidiot[idiot][cflashoff];
    }
    refresh();
    flashtimer.expireset();
}
uint32_t neopixelstrip::idiot_neo_color(uint8_t idiot) { return color_Rgb_to_32b(cidiot[idiot][cnow]); }