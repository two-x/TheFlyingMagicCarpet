#pragma once
#include <NeoPixelBus.h>
#define colortype RgbColor  // RgbwColor
#define striplength 8
NeoPixelBus<NeoGrbFeature, NeoSk6812Method> neoobj(striplength, neopixel_pin);  // NeoWs2812Method, NeoWs2812xMethod, NeoSk6812Method, NeoEsp32Rmt0Ws2812xMethod, NeoEsp32I2s1800KbpsMethod, NeoEsp32I2s1Sk6812Method, 
// Default for esp32 is dma via I2S bus 1 at 800kHz using RMT. Don't know the protocol difference between "Ws2812", "Ws2812x", and "Sk6812"
// Run neos in a task example: https://github.com/Makuna/NeoPixelBus/wiki/ESP32-and-RTOS-Tasks
uint32_t color_to_888(uint16_t color565) {  // Convert 5-6-5 encoded 16-bit color value to uint32 in format 0x00RRGGBB
    return (static_cast<uint32_t>(color565 & 0xf800) << 8) | (static_cast<uint32_t>(color565 & 0x7e0) << 5) | (static_cast<uint32_t>(color565 & 0x1f) << 3);
}
uint32_t color_to_888(uint8_t color332) {  
    return (static_cast<uint32_t>(color332 & 0xe0) << 16) | (static_cast<uint32_t>(color332 & 0x1c) << 11) | (static_cast<uint32_t>(color332 & 0x3) << 6);
}
uint32_t color_to_888(colortype colorneo) {  
    return (static_cast<uint32_t>(colorneo.R) << 16) | (static_cast<uint32_t>(colorneo.G) << 8) | static_cast<uint32_t>(colorneo.B);  // (static_cast<uint32_t>(color.W) << 24) | 
}
uint16_t color_to_565(uint32_t color888) {  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
    return static_cast<uint16_t>(((color888 & 0xf80000) >> 8) | ((color888 & 0xfc00) >> 5) | ((color888 & 0xf8) >> 3));
}
uint16_t color_to_565(uint8_t color332) {  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
    return ((static_cast<uint16_t>(color332) & 0xe0) << 8) | ((static_cast<uint16_t>(color332) & 0x1c) << 6) | ((static_cast<uint16_t>(color332) & 0x3) << 3);
}
uint16_t color_to_565(colortype colorneo) { 
    return ((static_cast<uint16_t>(colorneo.R) & 0xf8) << 8) | ((static_cast<uint16_t>(colorneo.G) & 0xfc) << 3) | (((static_cast<uint16_t>(colorneo.B) & 0xf8) >> 3));
}
uint8_t color_to_332(uint16_t color565) {  // Convert uint32 color in format 0x00RRGGBB to uint16 5-6-5 encoded color value suitable for screen
    return static_cast<uint8_t>(((color565 & 0xe000) >> 8) | ((color565 & 0x700) >> 6) | ((color565 & 0x18) >> 3));
}
uint8_t color_to_332(uint32_t color888) {  // 
    return static_cast<uint8_t>(((color888 & 0xe00000) >> 16) | ((color888 & 0xe000) >> 11) | ((color888 & 0xc0) >> 6));
}
uint8_t color_to_332(colortype colorneo) {  
    return (colorneo.R & 0xe0) | ((colorneo.G & 0xe0) >> 3) | ((colorneo.B & 0xc0) >> 6);
}
colortype color_to_neo(uint32_t color888) {  // 
    return colortype((color888 >> 16) & 0xff, (color888 >> 8) & 0xff, color888 & 0xff);  // (static_cast<uint32_t>(color.W) << 24) | 
}
colortype color_to_neo(uint16_t color565) {  // 
    return colortype((color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3);  // , 0);
}
colortype color_to_neo(uint8_t color332) {  // 
    return colortype(color332 & 0xe0, (color332 & 0x1c) << 3, (color332 & 0x3) << 6);  // , 0);
}
class NeopixelStrip {
  public:
    static const uint idiotcount = 7;
  private:
    enum brightness_presets : int { B_OFF, B_MIN, B_LO, B_MED, B_HI, B_EXT, B_MAX };
    enum ledset : int { onoff, fcount, fpulseh, fpulsel, fonbrit, fnumset };  // just a bunch of int variables needed for each of the neo idiot lights
    enum ledcolor : int { cnow, clast, cnormal, coff, con, cflash, cnumcolors };
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
    static constexpr uint32_t neo_fade_timeout_us = 380000;
    Timer neoFadeTimer = Timer(neo_fade_timeout_us), neoHeartbeatTimer;
    bool neo_heartbeat = false;
    bool heartcolor_change = true;  // , heartcolor_overridden = false;
    int pin = -1;
    uint8_t heartbeat_brightness, heartbeat_brightness_last; // brightness during fadeouts
    uint32_t neobright_last;
    int32_t heartbeat_state = 0;
    int32_t heartbeat_level = 0;
    int64_t heartbeat_ekg_us[4] = {250000, 240000, 620000, 2000000};  // {187500, 125000, 562500, 1250000};
    int32_t heartbeat_pulse = 255;
    static const uint8_t numpixels = 1 + idiotcount;  //  + extIdiotCount;  // 15 pixels = heartbeat RGB + 7 onboard RGB + 7 external RGBW
    colortype neostrip[numpixels];
    colortype heartbeatColor, heartbeatNow;
    uint8_t heartcolor16 = 0x00;  // blackened heart
    bool breadboard = false;
    uint8_t fset[idiotcount][fnumset];
    colortype cidiot[idiotcount][cnumcolors];
    uint32_t fquantum_us = 50000;  // time resolution of flashes
    static const uint8_t fevresolution = 128;
    static const uint8_t fevpages = 4;  // fevresolution / 32;  <- but if I use that math instead, seg fault
    uint32_t fevents[idiotcount][fevpages];
    uint8_t fevcurrpage = 0; uint8_t fevfilled = 0; uint32_t nowtime_us, nowepoch;  // , fevmask_master;
    Timer flashtimer;
    float maxelement(float r, float g, float b);
    float midelement(float r, float g, float b);
    float minelement(float r, float g, float b);
    colortype dimmer(colortype color, uint8_t bright);  // brightness 0 is off, 100 is max brightness while retaining same hue and saturation
    colortype desaturate(colortype color, int32_t _desat_of_ten);  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
    colortype hue_to_rgb(uint8_t hue);
    void recolor_idiots(int8_t argidiot = -1);
    bool fevpop(uint _idiot, int8_t pop_off);
    void fevpush(uint _idiot, int8_t push_off, bool push_val);
    void update_idiot(uint _idiot);
    void calc_lobright();
  public:
    NeopixelStrip(int argpin) { pin = argpin; }
    void refresh();
    void setup(bool viewcontext=NITE);
    void setbright(uint8_t bright_pc);
    void setdesaturation(float _desat_of_ten);  // a way to specify nite or daytime brightness levels
    void heartbeat_ena(bool onoroff);
    void set_heartcolor(uint8_t newcolor);
    void heartcolor_override(uint8_t color);
    void heartbeat_update();
    void colorfade_update();
    uint32_t neopixelsAvailable();
    bool newIdiotLight(uint _idiot, uint8_t color332, bool startboolstate = 0);
    void setBoolState(uint _idiot, bool state);
    void setflash(uint _idiot, uint8_t count, uint8_t pulseh=1, uint8_t pulsel=1, int32_t onbrit=-1, int32_t color=0xffffff);
    void update(int16_t heart_color);
    void enable_flashdemo(bool ena);
    uint32_t idiot_neo_color(uint _idiot);
};

float NeopixelStrip::maxelement(float r, float g, float b) {
    return (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //smax(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
float NeopixelStrip::midelement(float r, float g, float b) {
    return (r >= g) ? ((g >= b) ? g : ((r >= b) ? b : r)) : ((r >= b) ? r : ((b >= g) ? g : b));  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //smax(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
float NeopixelStrip::minelement(float r, float g, float b) {
    return (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //smax(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
}
colortype NeopixelStrip::dimmer(colortype color, uint8_t bright) {  // brightness 0 is off, 255 is max brightness while retaining same hue and saturation
    int32_t ret[3];
    float rgb[3] = { static_cast<float>(color.R), static_cast<float>(color.G), static_cast<float>(color.B) };
    float fbright = (float)bright / maxelement(rgb[0], rgb[1], rgb[2]);  // smax(color.r, color.g, color.b);  // 2.55 = 0xff / 100
    for (int32_t element=0; element<3; element++) ret[element] = constrain((int32_t)(rgb[element] * fbright), 0, 255);
    // printf("D br:%d  inc:%06x  outc:%02x%02x%02x\n", bright, color_to_888(color), (uint8_t)ret[0], (uint8_t)ret[1], (uint8_t)ret[2]);
    return colortype(ret[0], ret[1], ret[2]);  // return CRGB((float)(color.r * fbright), (float)(color.g * fbright), (float)(color.b * fbright));
}
colortype NeopixelStrip::desaturate(colortype color, int32_t _desat_of_ten) {  // desat_pc=0 has no effect, =10 desaturates all the way to greyscale, =-99 saturates to max. without change in brightness
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
colortype NeopixelStrip::hue_to_rgb(uint8_t hue) {
    uint32_t rgb[3] = { 255 - 3 * (uint32_t)((255 - hue) % 85), 0, 3 * (uint32_t)((255 - hue) % 85) };
    if (hue <= 85) { rgb[1] = rgb[0]; rgb[0] = rgb[2]; rgb[2] = 0; }
    else if (hue <= 170) { rgb[1] = rgb[2]; rgb[2] = rgb[0]; rgb[0] = 0; }
    return colortype(rgb[0], rgb[1], rgb[2]);
}
void NeopixelStrip::recolor_idiots(int8_t _idiot) {  // pass in -1 to recolor all idiots
    int8_t start = (_idiot >= 0) ? _idiot : 0;
    int8_t end = (_idiot >= 0) ? _idiot + 1 : idiotcount;
    for (int i = start; i < end; i++) {
        cidiot[i][con] = dimmer(cidiot[i][cnormal], hibright);
        cidiot[i][con] = desaturate(cidiot[i][con], desat_of_ten);
        cidiot[i][coff] = dimmer(cidiot[i][cnormal], lobright);
        cidiot[i][coff] = desaturate(cidiot[i][coff], desat_of_ten);
    }
}
void NeopixelStrip::refresh() {
    int32_t numledstowrite = (heartbeatNow != neostrip[0]);
    neostrip[0] = heartbeatNow;
    neoobj.SetPixelColor(0, heartbeatNow);
    for (int32_t i=0; i<idiotcount; i++) {
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
    setdesaturation(neodesat);
    heartbeat_ena(true);
    std::cout << "refresh.. ";
    flashtimer.set(fquantum_us * fevresolution);
    refresh();
    std::cout << std::endl;
}
void NeopixelStrip::calc_lobright() {
    lobright = smax(3, (hibright / lomultiplier) / lomultiplier);
}
void NeopixelStrip::setbright(uint8_t bright_pc) {  // a way to specify brightness level as a percent
    hibright = (uint8_t)((255.0 * (float)bright_pc) / 100.0);
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
void NeopixelStrip::colorfade_update() {
    if (neoFadeTimer.expireset()) {
        heartbeatNow = hue_to_rgb(++neo_wheelcounter);
        neoobj.SetPixelColor(0, heartbeatNow);
    }
}
uint32_t NeopixelStrip::neopixelsAvailable() {
    return idiotcount;
}
bool NeopixelStrip::newIdiotLight(uint _idiot, uint8_t color332, bool startboolstate) {
    if (_idiot > idiotcount-1) return false;
    fset[_idiot][onoff] = startboolstate;
    cidiot[_idiot][cnormal] = color_to_neo(color332);
    cidiot[_idiot][clast] = color_to_neo((uint32_t)0);
    fset[_idiot][fcount] = 0;
    setBoolState(_idiot, startboolstate);
    for (uint8_t pg = 0; pg < fevpages; pg++) fevents[_idiot][pg] = 0;
    recolor_idiots(_idiot);
    return true;
}
void NeopixelStrip::setBoolState(uint _idiot, bool state) {
    fset[_idiot][onoff] = state;
}
bool NeopixelStrip::fevpop(uint _idiot, int8_t pop_off) {
    int8_t page = pop_off / 32;
    return (fevents[_idiot][page] >> (pop_off - 32 * page)) & 1;  // pop a bit off the stack and return it
}
void NeopixelStrip::fevpush(uint _idiot, int8_t push_off, bool push_val) {
    int8_t page = push_off / 32;
    fevents[_idiot][page] |= (push_val << (push_off - 32 * page));
}
// setflash() : Call this to add a blink sequence to one of the idiot lights which will repeat indefinitely in [up to 6.4 sec] cycles
//   _idiot = which idiot light
//   count  = number of blinks per cycle. Use 0 to cancel a previously applied blink pattern
//   pulseh = high pulse width of each blink, in increments of 50 ms (of which there are max 128 per cycle)
//   pulsel = low pulse width of each blink, in increments of 50 ms (of which there are max 128 per cycle)
//   onbrit = percent brightness to apply during high pulses (use -1 to avoid)
//   color  = alternate color to apply during high pulses (use -1 to avoid)
void NeopixelStrip::setflash(uint _idiot, uint8_t count, uint8_t pulseh, uint8_t pulsel, int32_t onbrit, int32_t color) {
    fset[_idiot][fcount] = count;
    fset[_idiot][fpulseh] = smax(pulseh, 1);
    fset[_idiot][fpulsel] = smax(pulsel, 1);
    fset[_idiot][fonbrit] = (onbrit == -1) ? hibright : (uint8_t)onbrit;
    cidiot[_idiot][cflash] = dimmer((color == -1) ? cidiot[_idiot][cnormal] : color_to_neo((uint32_t)color), hibright);
    cidiot[_idiot][cflash] = desaturate(dimmer(cidiot[_idiot][cflash], fset[_idiot][fonbrit]), desat_of_ten);
    for (uint8_t pg = 0; pg < fevpages; pg++) fevents[_idiot][pg] = 0;
    uint8_t filled = 0;
    uint8_t lstop;
    uint8_t patternlen = fset[_idiot][fcount] * (fset[_idiot][fpulseh] + fset[_idiot][fpulsel]);
    uint8_t reps = 1 + (patternlen < fevresolution / 3);  // For shorter flash patterns repeat them multiple times in each cycle
    for (uint8_t rep = 1; rep <= reps; rep++) {        
        lstop = smin(fevresolution, filled + patternlen);
        while (filled < lstop) {
            for (uint8_t hbit = 0; hbit < fset[_idiot][fpulseh]; hbit++)
                if (filled < lstop) fevpush(_idiot, filled++, 1);
            for (uint8_t lbit = 0; lbit < fset[_idiot][fpulsel]; lbit++)
                if (filled < lstop) fevpush(_idiot, filled++, 0); 
        }
        filled = fevresolution / reps;
    }
}
uint32_t NeopixelStrip::idiot_neo_color(uint _idiot) { 
    return color_to_888(cidiot[_idiot][cnow]);
}
void NeopixelStrip::enable_flashdemo(bool ena) {
    if (ena) {
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
void NeopixelStrip::update_idiot(uint32_t _idiot) {
    cidiot[_idiot][clast] = cidiot[_idiot][cnow];
    colortype newnow = (fset[_idiot][onoff]) ? cidiot[_idiot][con] : cidiot[_idiot][coff];
    if (!fset[_idiot][fcount]) cidiot[_idiot][cnow] = newnow;
    else cidiot[_idiot][cnow] = fevpop(_idiot, nowepoch) ? cidiot[_idiot][cflash] : newnow;
}
void NeopixelStrip::update(int16_t heart_color) {
    set_heartcolor(heart_color);
    heartbeat_update();  // Update our beating heart
    nowtime_us = (uint32_t)flashtimer.elapsed();
    nowepoch = nowtime_us / fquantum_us;
    for (int32_t i=0; i<idiotcount; i++) {
        if (!syspower) cidiot[i][cnow] = colortype(0);
        else update_idiot(i);
    }
    refresh();
    flashtimer.expireset();
}