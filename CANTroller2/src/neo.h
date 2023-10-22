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
    enum ledstate { posts, flashes, nowflash, nowpost, nowphase, onoff, bright, num_states };  // just a bunch of int variables needed for each of the neo idiot lights
    enum cycles { steady = 2, num_cycles = 3 };  // borrowing posts=0 and flashes=1 from above
    enum ledcolor { normal, effect, now, last, num_colors };  // colortype-typed colors specific to each neo idiot light
    uint8_t idiotstate[idiotcount][num_states];  // flash pattern data for all idiot lights
    colortype idiotcolor[idiotcount][num_colors];
    int32_t pulsetime_us[2][2] = { { 45000, 55000 }, { 175000, 74000 } };  // [flashes/posts][flashcolor/normalcolor] 
    int32_t flashtimeout;
    Timer flashtimer[idiotcount];
    bool pulsecycle = false; bool breadboard = false;
    int8_t cycle = flashes; int8_t cycle_last;
    int8_t idiotpulses[idiotcount][2];
    int8_t idiotnowpulse[idiotcount][2];
    colortype pulsecolor[2] = { colortype(0xff, 0xff, 0xff), colortype(0x00, 0x00, 0x00) };
    Timer pulseSeqTimer;
    colortype neostrip[numpixels];
    colortype heartbeatColor, heartbeatNow, heartbeatColor_last;
    Timer debugtimer;
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
    void updateIdiot(uint8_t idiot, bool switchit = false);
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
    void setPosts(uint8_t idiot, uint8_t argposts);
    void setFlashes(uint8_t idiot, uint8_t argflashes);
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
        idiotstate[idiot][bright] = (idiotpulses[idiot][flashes] > 0 || idiotstate[idiot][onoff] > 0) ? hibright : lobright;  // If flashing the led is always bright
        idiotcolor[idiot][effect] = dimmer(idiotcolor[idiot][normal], idiotstate[idiot][bright]);
        idiotcolor[idiot][effect] = desaturate(idiotcolor[idiot][effect], desatlevel);
    }
}
void neopixelstrip::updateIdiot(uint8_t idiot, bool switchit) {
    idiotcolor[idiot][last] = idiotcolor[idiot][now];
    if (idiotpulses[idiot][flashes] == 4) {  // Flashes=4 is code for constant strobe, so hack our logic below
        switchit = true;  // spoof this to get into loop
        cycle = flashes;  // steal the fast blink rate of the flasher
        // idiotstate[idiot][bright] = higher // finish this if desired (brighten colors)
    }
    recolor_idiot(idiot);  // shouldn't have to do this every update ...
    if (switchit && (cycle != steady)) idiotnowpulse[idiot][cycle] = (idiotpulses[idiot][cycle] > 0);
    if (cycle == steady || (idiotpulses[idiot][flashes] == 0 && idiotpulses[idiot][posts] == 0))
            idiotcolor[idiot][now] = idiotcolor[idiot][effect];  // we aren't flashing or posting, so just stay lit
    else if (switchit || flashtimer[idiot].expired()) {  // figure out flashing/posts situation
        flashtimeout = 2000000;
        if ((idiotpulses[idiot][cycle]) && idiotnowpulse[idiot][cycle] <= idiotpulses[idiot][cycle]) { // now flashing
            if (idiotstate[idiot][nowphase]) idiotcolor[idiot][now] = idiotcolor[idiot][effect];
            else idiotcolor[idiot][now] = pulsecolor[cycle];
            if (idiotnowpulse[idiot][cycle] <= idiotpulses[idiot][cycle])
                flashtimeout = pulsetime_us[cycle][idiotstate[idiot][nowphase]];
            if (idiotstate[idiot][nowphase]) idiotnowpulse[idiot][cycle]++;
            idiotstate[idiot][nowphase] = !idiotstate[idiot][nowphase];
            // if (idiot <= 1) printf ("i%d: s:%d", idiot, (uint8_t)flashseq);
            // flashtimeout = postflash_ekg_us[0] / ((idiotstate[idiot][posts] > 0) + (idiotpulses[idiot][flashes] > 0)) - idiotpulses[idiot][cycle] * (postflash_ekg_us[1] + postflash_ekg_us[2]);  // end of these pulses             
        }
        flashtimer[idiot].set(flashtimeout);
        if (idiotcolor[idiot][last] != idiotcolor[idiot][now]) {
            if (idiot == 0) printf ("i:%d cyc:%d f:%d/%d ph:%d ", idiot, cycle, idiotnowpulse[idiot][cycle], idiotpulses[idiot][cycle], idiotstate[idiot][nowphase]);
            if (idiot == 0) std::cout << " s:" << cycle << " c:" << color_Rgb_to_32b(idiotcolor[idiot][now]) << " t:" << flashtimeout << std::endl;
        }
    }
    // neostrip[1+idiot] = idiotcolor[idiot][now];
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
        if (idiotcolor[idiot][now] != neostrip[idiot+1]) {
            neoobj.SetPixelColor (1+idiot, idiotcolor[idiot][now]);
            neostrip[idiot + 1] = idiotcolor[idiot][now];  // colortype(idiotcolor[idiot][now]);
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
    pulseSeqTimer.set(1000000);
    pulsecolor[flashes] = dimmer(colortype(0xff, 0xff, 0xff), (255 + hibright)/2);
    std::cout << "refresh strip.. ";
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
    pulsecolor[flashes] = dimmer(colortype(0xff, 0xff, 0xff), (255 + hibright)/2);
    // recolor_idiot(idiot);
}
void neopixelstrip::setdesaturation(float newlev) {  // a way to specify nite or daytime brightness levels
    desatlevel = newlev;
    // recolor_idiot(idiot);
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
    idiotstate[idiot][onoff] = startboolstate;
    idiotcolor[idiot][normal] = color_16b_to_Rgb(color565);
    idiotcolor[idiot][last] = colortype(0x00, 0x00, 0x00);
    for (int i=flashes; i<=posts; i++) {
        idiotpulses[idiot][i] = 0;
        idiotnowpulse[idiot][i] = 0;
    }
    setBoolState(idiot, startboolstate);
    flashtimer[idiot].set(2000000);
    updateIdiot(idiot);
    return true;
}
void neopixelstrip::setBoolState(uint8_t idiot, bool state) {
    idiotstate[idiot][onoff] = state;
    // recolor_idiot(idiot);
}
void neopixelstrip::setPosts(uint8_t idiot, uint8_t argposts) {
    idiotpulses[idiot][posts] = min(4, argposts);
    idiotnowpulse[idiot][posts] = (idiotpulses[idiot][posts] > 0);
}
void neopixelstrip::setFlashes(uint8_t idiot, uint8_t argflashes) {
    idiotpulses[idiot][flashes] = min(4, argflashes);
    idiotnowpulse[idiot][flashes] = (idiotpulses[idiot][flashes] > 0);
    // recolor_idiot(idiot);
}
void neopixelstrip::update() {
    cycle_last = cycle;
    if (pulseSeqTimer.expireset()) ++cycle %= num_cycles; // figure out flashing/posts situation
    for (int32_t idiot=0; idiot<idiotcount; idiot++) updateIdiot(idiot, (cycle != cycle_last));
    refresh();
}
uint32_t neopixelstrip::idiot_neo_color(uint8_t idiot) { return color_Rgb_to_32b(idiotcolor[idiot][now]); }