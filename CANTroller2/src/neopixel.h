#pragma once
#include <NeoPixelBus.h>
#define neorgb_t RgbColor  // RgbwColor
#define striplength 10
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
uint8_t color_to_332(uint16_t color565) { return static_cast<uint8_t>(((color565 & 0xe000) >> 8) | ((color565 & 0x700) >> 6) | ((color565 & 0x18) >> 3)); }  // Convert uint32 color in format 0x00RRGGBB to uint8 3-3-2 encoded color value suitable for frame buffers
uint8_t color_to_332(uint32_t color888) { return static_cast<uint8_t>(((color888 & 0xe00000) >> 16) | ((color888 & 0xe000) >> 11) | ((color888 & 0xc0) >> 6)); }
uint8_t color_to_332(neorgb_t colorneo) { return (colorneo.R & 0xe0) | ((colorneo.G & 0xe0) >> 3) | ((colorneo.B & 0xc0) >> 6); }
neorgb_t color_to_neo(uint32_t color888) { return neorgb_t((color888 >> 16) & 0xff, (color888 >> 8) & 0xff, color888 & 0xff); }  // (static_cast<uint32_t>(color.W) << 24) |
neorgb_t color_to_neo(uint16_t color565) { return neorgb_t((color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3); }  // , 0);
neorgb_t color_to_neo(uint8_t color332) { return neorgb_t(color332 & 0xe0, (color332 & 0x1c) << 3, (color332 & 0x3) << 6); }  // , 0);

class NeopixelStrip {
  private:
    enum ledset : int { onoff, fcount, fpulseh, fpulsel, fonbrit, fnumset };  // just a bunch of int variables needed for each of the neo idiot lights
    enum ledcolor : int { cnow, clast, cnormal, coff, con, cflash, cnumcolors };
  public:
    bool sleepmode = false;
    int pcbaglow = GlowSimple;
    static const int heartcount = 2;  // number of heartbeat leds in the strip before the idiotlights
    static const int idiotcount = striplength - heartcount - 1;  // number of idiotlights after the heartbeat leds
    NeopixelStrip(int argpin) : pin(argpin) {}
    void setup();  // (bool viewcontext=NITE);
    void setbright(float bright_pc);
    void setsat(float sat_pc);
    void set_heartcolor(uint8_t newcolor);
    void heartcolor_override(uint8_t color);
    void setlogic(int _idiot, bool state);
    void setflash(int _idiot, int count, int pulseh=1, int pulsel=1, int onbrit=-1, uint32_t color=0);
    void update();
    void flashdemo_ena(bool ena);
    uint32_t idiot_neo_color(int _idiot);
    bool newIdiotLight(int _idiot, uint8_t color332, bool startboolstate = 0);
    void sleepmode_ena(bool ena);
    void set_pcba_glow(int mode=-1);
    int num_neo_idiots();
  private:
    static const uint fevresolution = 128;
    static const uint fevpages = 4;  // fevresolution / 32;  <- but if I use that math instead, seg fault
    bool heartcolor_change = true;
    uint8_t lobright_idiot = 0, hibright_idiot = 100, csat[idiotcount];  // idiot brightnesses for each logic level lo and hi
    uint8_t hibright_heart[2] = { 100, 100 }, lobright_heart[2] = { 25, 25 };  // [LED0/LED1] min and max brightness for each heartbeat led
    uint32_t fcbase[idiotcount];
    float neobright_last[2];
    Timer heartbeatTimer{10000}, flashTimer;
    int pin = -1, fevcurrpage = 0, fevfilled = 0, nowtime_us, nowepoch, fquantum_us = 50000;  // time resolution of flashes
    int fevents[idiotcount][fevpages], fset[idiotcount][fnumset];
    neorgb_t neostrip[striplength], cidiot[idiotcount][cnumcolors], heartcolorbase, heartcolornow[2];
    uint16_t chue[idiotcount];
    // private methods
    void recolor_idiots(int argidiot=-1);
    uint32_t recolor(uint32_t index, float bright_pc, float sat_pc);
    neorgb_t recolor(neorgb_t index, float bright_pc, float sat_pc);
    void set_fcolor(int _idiot);
    bool fevpop(int _idiot, uint pop_off);
    void fevpush(int _idiot, uint push_off, bool push_val);
    void update_idiot(int _idiot);
    void update_pcba_glow();
    uint16_t get_hue(RgbColor rgb); // Function to convert RGB888 to 16-bit hue value
    uint8_t get_sat(RgbColor rgb);
    uint8_t get_brite(RgbColor rgb);
    void refresh(bool force=false);
    void heartbeat_update();
    void heartxfade_update();
    void heartsine_update();
    void knightrider();
};
void NeopixelStrip::setlogic(int _idiot, bool state) { fset[_idiot][onoff] = state; }
void NeopixelStrip::setsat(float sat_pc) { neosat = sat_pc; recolor_idiots(-1); }
uint32_t NeopixelStrip::idiot_neo_color(int _idiot) { return color_to_888(cidiot[_idiot][cnow]); }
uint16_t NeopixelStrip::get_hue(neorgb_t rgb) { return static_cast<uint16_t>(((HslColor)rgb).H * 65535); }  // Convert hue from float [0.0, 1.0] to 16-bit integer [0, 65535]
uint8_t NeopixelStrip::get_sat(neorgb_t rgb) { return static_cast<uint8_t>(((HslColor)rgb).S * 255); } // Convert saturation from float [0.0, 1.0] to 16-bit integer [0, 65535]
uint8_t NeopixelStrip::get_brite(neorgb_t rgb) { return static_cast<uint8_t>(((HslColor)rgb).L * 255); } // Convert saturation from float [0.0, 1.0] to 16-bit integer [0, 65535]

// recolor sets overall brightness and reduces saturation to match the percents given, and returns the modified color. pass in 100.0 to leave either brightness or sat unchanged
neorgb_t NeopixelStrip::recolor(neorgb_t orig, float bright_pc=100.0, float sat_pc=100.0) {
    uint8_t newbrite = (uint8_t)((float)(get_brite(orig)) * bright_pc / 100.0);  // uint8_t newbrite = (uint8_t)(bright_pc / 100.0);
    uint8_t newsat = (uint8_t)((float)(get_sat(orig)) * sat_pc / 100.0);
    return color_to_neo(hsv_to_rgb<uint32_t>(get_hue(orig), newsat, newbrite));
}
uint32_t NeopixelStrip::recolor(uint32_t orig888, float bright_pc=100.0, float sat_pc=100.0) {
    neorgb_t orig = color_to_neo(orig888);
    uint8_t newbrite = (uint8_t)((float)(get_brite(orig)) * bright_pc / 100.0);  // uint8_t newbrite = (uint8_t)(bright_pc / 100.0);
    uint8_t newsat = (uint8_t)((float)(get_sat(orig)) * sat_pc / 100.0);
    return hsv_to_rgb<uint32_t>(get_hue(orig), newsat, newbrite);
}
void NeopixelStrip::recolor_idiots(int _idiot) {  // call w/ -1 to recolor all idiots
    int start = ((_idiot >= 0) ? _idiot : 0) + heartcount;
    int end = ((_idiot >= 0) ? _idiot + 1 : idiotcount) + heartcount;
    for (int i = start; i < end; i++) {  // uint8_t sat = std::min((uint8_t)(csat[_idiot] * 256.0 / 100.0), (uint8_t)(neosat * 256.0 / 100.0));
        cidiot[i][con] = recolor(cidiot[i][cnormal], (float)hibright_idiot, neosat);
        cidiot[i][coff] = recolor(cidiot[i][cnormal], (float)lobright_idiot, neosat);
        if (fset[i][fcount]) set_fcolor(i);
    }
}
void NeopixelStrip::refresh(bool force) {
    neoobj.SetPixelColor(0, heartcolornow[0]);
    neoobj.SetPixelColor(1, heartcolornow[1]);
    neoobj.SetPixelColor(2, heartcolornow[0]);
    for (int i=0; i<idiotcount; i++)
        neoobj.SetPixelColor(i + heartcount + 1, recolor(cidiot[i][cnormal], (float)(fset[i][onoff] ? hibright_idiot : lobright_idiot), neosat));
    neoobj.Show();
}
void NeopixelStrip::setup() {
    ezread.squintf("Neopixels.. ");
    set_pcba_glow(GlowSine);
    neoobj.Begin();
    if (!running_on_devboard) setbright(100.0);
    ezread.squintf("refresh.. ");
    flashTimer.set(fquantum_us * (int)fevresolution);
    refresh();
    ezread.squintf("\n");
}
void NeopixelStrip::setbright(float bright_pc) {  // a way to specify brightness level as a percent
    static int lomultiplier = 3;
    bool neo_heartbeat_variable_brightness = false;  // If false then brightness control only affect idiotlights
    neobright = bright_pc;
    hibright_idiot = neobright;  // hibright_idiot = (uint8_t)((255.0 * neobright) / 100.0);
    // lobright_idiot = std::max(3, (hibright_idiot / lomultiplier) / lomultiplier);
    if (neo_heartbeat_variable_brightness) {  // note this is broken, it will always make both heartbeat leds be same brightness
        for (int i=0; i<heartcount; i++) {
            hibright_heart[0] = hibright_idiot;  // (uint8_t)((float)hibright * 0.75);
            lobright_heart[0] = lobright_idiot;
        }
    }
    recolor_idiots(-1);
}
void NeopixelStrip::set_heartcolor(uint8_t _newcolor) {
    uint8_t newcolor = _newcolor;
    static uint16_t heartcolor332 = 0x00;
    if (heartbeat_override_color != 0x00) newcolor = heartbeat_override_color;
    if (heartcolor332 != newcolor) {
        heartcolor_change = true;
        heartcolor332 = newcolor;
        heartcolorbase = color_to_neo(newcolor);
    }
}
void NeopixelStrip::set_pcba_glow(int mode) {  // call with -1 argument to set glow mode to flashed value
    int newmode = mode;
    if (newmode == -1) newmode = watchdog.flash_read("glowmode", GlowSimple);
    if (newmode == -1) newmode = GlowSimple;
    if (newmode == pcbaglow) return;
    pcbaglow = newmode;
    if (mode != -1) watchdog.flash_forcewrite("glowmode", pcbaglow);
    heartcolornow[0] = heartcolornow[1] = color_to_neo(BLK);  // set both leds to black to start
}
void NeopixelStrip::heartbeat_update() {
    static int state = 0, brt[2], brt_last[2], ekg[4] = {250000, 240000, 620000, 2000000};
    static bool highpulse = true;
    static Timer fadeTimer{38000};
    if (heartbeatTimer.expired()) {
        highpulse = !highpulse;
        ++state %= arraysize(ekg);
        heartbeatTimer.set(ekg[state]);
        if (highpulse) for (int i=0; i<heartcount; i++) brt[i] = hibright_heart[i];
        else fadeTimer.reset();
    }
    else if (!highpulse) {
        for (int i=0; i<heartcount; i++) {
            brt_last[i] = brt[i];
            if (fadeTimer.expired()) brt[i] = lobright_heart[i];
            else brt[i] = (uint8_t)(lobright_heart[i] + std::max((double)0, (float)(hibright_heart[i] - lobright_heart[i]) * (1.0 - ((state == 1) ? 1.5 : 1.0) * (float)fadeTimer.elapsed() / (float)fadeTimer.timeout())));
            if (brt[i] > brt_last[i]) brt[i] = brt_last[i];
        }
    }
    for (int i=0; i<heartcount; i++) {
        if (heartcolor_change || (std::abs(brt[i] - (int)neobright_last[i]) > 1)) {
            heartcolornow[i] = recolor(heartcolorbase, (float)brt[i]);  // heartcolor = dimmer(desaturate(heartcolorbase, desat_of_ten), brt);
            heartcolor_change = false;
            neobright_last[i] = (float)brt[i];
        }
    }
}
void NeopixelStrip::heartxfade_update() {}
void NeopixelStrip::heartsine_update() {
    // static const float min_brightness = 50.0f;  // in percent
    float phase = M_PI * 2.0f * (float)heartbeatTimer.elapsed() / 2850000.0f;  // denominator here is the period of the pulse in us, adjustable
    heartcolornow[0] = recolor(heartcolorbase, lobright_heart[0] + (hibright_heart[0] - lobright_heart[0]) * (1.0f + (float)std::cos(phase)) / 2.0f);
    heartcolornow[1] = recolor(heartcolorbase, lobright_heart[1] + (hibright_heart[1] - lobright_heart[1]) * (1.0f + (float)std::sin(phase)) / 2.0f);
}

bool NeopixelStrip::newIdiotLight(int _idiot, uint8_t color332, bool startboolstate) {
    if (_idiot >= idiotcount) return false;
    fset[_idiot][onoff] = startboolstate;
    fset[_idiot][fcount] = 0;
    cidiot[_idiot][clast] = color_to_neo((uint32_t)0);
    if (use_tft_colors_for_neo) cidiot[_idiot][cnormal] = color_to_neo(color332);
    else cidiot[_idiot][cnormal] = color_to_neo((hsv_to_rgb<uint32_t>((uint16_t)(_idiot * 65563.0 / idiotcount), 255, 255)));
    setlogic(_idiot, startboolstate);
    for (int pg = 0; pg < fevpages; pg++) fevents[_idiot][pg] = 0;
    recolor_idiots(_idiot);
    return true;
}

// setflash() : Call this to add a blink sequence to one of the idiot lights which will repeat indefinitely in cycles
//              Cycles are chopped into 128 definable segments of 50 ms each, for a total of 6.4 sec
//   _idiot = which idiot light
//   count  = number of blinks per cycle. Use 0 to cancel a previously applied blink pattern
//   pulseh = high pulse width of each blink, in increments of 50 ms (of which there are max 128 per cycle)
//   pulsel = low pulse width of each blink, here led will be its normal on or off color. also 50 ms each
//   onbrit = percent brightness to apply during high pulses (use -1 to avoid)
//   color  = alternate color to apply during high pulses (defaults to normal idiot light color)
void NeopixelStrip::setflash(int _idiot, int count, int pulseh, int pulsel, int onbrit, uint32_t color) {
    fset[_idiot][fcount] = count;
    fset[_idiot][fpulseh] = std::max((int)pulseh, 1);
    fset[_idiot][fpulsel] = std::max((int)pulsel, 1);
    fset[_idiot][fonbrit] = onbrit;
    fcbase[_idiot] = color;
    set_fcolor(_idiot);
    for (int pg = 0; pg < fevpages; pg++) fevents[_idiot][pg] = 0;
    uint lstop, filled = 0;
    uint patternlen = fset[_idiot][fcount] * (fset[_idiot][fpulseh] + fset[_idiot][fpulsel]);
    // uint reps = 1 + (patternlen < fevresolution / 3);  // For shorter flash patterns repeat them multiple times in each cycle
    // for (int rep = 1; rep <= reps; rep++) {        
    lstop = std::min(fevresolution, filled + patternlen);
    while (filled < lstop) {
        for (int hbit = 0; hbit < fset[_idiot][fpulseh]; hbit++) if (filled < lstop) fevpush(_idiot, filled++, 1);
        for (int lbit = 0; lbit < fset[_idiot][fpulsel]; lbit++) if (filled < lstop) fevpush(_idiot, filled++, 0); 
    }
    filled = fevresolution;  // filled = fevresolution / reps; }
}
void NeopixelStrip::set_fcolor(int _idiot) {  // flashing event : recolor idiotlight to flash pattern value
    int brite = (fset[_idiot][fonbrit] >= 0) ? fset[_idiot][fonbrit] : hibright_idiot;  // 
    if (!fcbase[_idiot]) cidiot[_idiot][cflash] = cidiot[_idiot][cnormal];  // set the flash color to the normal base color if the flash base color is black,
    else cidiot[_idiot][cflash] = fcbase[_idiot];                           // otherwise set it to the flash base color
    cidiot[_idiot][cflash] = recolor(cidiot[_idiot][cflash], (float)brite, neosat);
}
bool NeopixelStrip::fevpop(int _idiot, uint pop_off) {  // flashing event : pop a flash sequence bit out from the data page
    int page = pop_off / 32;  // divide by 32
    return (fevents[_idiot][page] >> (pop_off - 32 * page)) & 1;
}
void NeopixelStrip::fevpush(int _idiot, uint push_off, bool push_val) {  // flashing event : push a flash sequence bit into the data page
    int page = push_off / 32;  // divide by 32
    fevents[_idiot][page] |= (push_val << (push_off - 32 * page));
}
void NeopixelStrip::flashdemo_ena(bool ena) {
    flashdemo = ena;
    if (flashdemo) {
        setflash(4, 8, 8, 8, 20);            // brightness toggle in a continuous squarewave
        setflash(5, 3, 1, 2, 100, 0xffffff); // three super-quick bright white flashes
        setflash(6, 2, 5, 5, 0, 0);          // two short black pulses
    }
    else {                                   // cancel any current blink programs on these leds
        setflash(4, 0);
        setflash(5, 0);
        setflash(6, 0);
    }
}
void NeopixelStrip::update_idiot(int _idiot) {
    cidiot[_idiot][clast] = cidiot[_idiot][cnow];                                                          // remember previous color
    cidiot[_idiot][cnow] = fset[_idiot][onoff] ? cidiot[_idiot][con] : cidiot[_idiot][coff];               // set color to con or coff depending on state of idiotlight
    if (fset[_idiot][fcount]) if (fevpop(_idiot, nowepoch)) cidiot[_idiot][cnow] = cidiot[_idiot][cflash]; // if flashing, override with the flash color
}
void NeopixelStrip::update_pcba_glow() {
    if (pcbaglow == GlowOff) return;
    set_heartcolor(colorcard[runmode]);
    if (pcbaglow == GlowSimple) heartcolornow[0] = heartcolornow[1] = heartcolorbase;
    else if (pcbaglow == GlowHeart) heartbeat_update();  // Update our beating heart
    else if (pcbaglow == GlowXFade) heartxfade_update();
    else if (pcbaglow == GlowSine) heartsine_update();
}
void NeopixelStrip::update() {
    static int runmode_last;
    if (runmode != runmode_last) sleepmode_ena(runmode == LOWPOWER);
    runmode_last = runmode;
    if (sleepmode) {
        knightrider();
        return;
    }
    update_pcba_glow();
    nowtime_us = (int)flashTimer.elapsed();
    nowepoch = nowtime_us / fquantum_us;
    for (int i=0; i<idiotcount; i++) update_idiot(i);
    refresh();
    flashTimer.expireset();
}
int NeopixelStrip::num_neo_idiots() { return idiotcount; }
void NeopixelStrip::sleepmode_ena(bool ena) {
    if (!ena && sleepmode) refresh(true);
    sleepmode = ena;
}
void NeopixelStrip::knightrider() {
    static Timer knighttimer{150000};
    static int tail = 4, posn = heartcount - 1, first = heartcount - 1, dir = 1; // dir=1 for right, dir=-1 for left
    static const float maxspeed = 20.0, minspeed = 100.0;  // fastest and slowest speed (milliseconds per step)
    static const uint32_t color = 0xff0000;                // red color
    float speed = maxspeed + (minspeed - maxspeed) * (float)posn / (striplength - 1); // Calculate the speed based on the position (decelerates as it moves)
    if (knighttimer.expireset()) {      // if (knighttimer.elapsed((int)moveInterval)) {
        posn += dir;                    // move the bright point
        if (posn <= first || posn >= striplength - 1) dir = -dir;   // reverse direction at the ends
        posn = constrain(posn, first, striplength - 1);             // clamp position within bounds
    }
    for (int i=first; i<striplength; i++) neoobj.SetPixelColor(i, color_to_neo(BLK));  // clear the strip   
    neoobj.SetPixelColor(posn, color_to_neo(recolor(color, neobright)));         // set the bright point
    for (int i=1; i<=tail; i++) {   // set the trailing effect
        int tailpos = posn - i * dir;
        if (tailpos >= first && tailpos < striplength)
            neoobj.SetPixelColor(tailpos, color_to_neo(recolor(color, neobright * (100.0 * (tail - i) / tail) / 100.0)));
    }
    neoobj.Show(); // Show the updated strip
}

//     for (int i = 1; i <= tail; i++) {  // Set the tailing effect
//         int tailposn = posn - i * direction;
//         if (tailposn >= 0 && tailposn < striplength) {
//             neoobj.SetPixelColor(tailposn, color_to_neo(recolor(basecolor, neobright * (tail - i) / trail)));
//         }
//     }

// void NeopixelStrip::knightrider() {
//     static Timer knighttimer{15000};
//     static float posn = 0.0f;
//     static int direction = 1;        // 1 for right, -1 for left
//     const int tail = 4;             // Length of the fading tail
//     const uint32_t basecolor = 0xff0000;  // Red color
//     const float maxspeed = 20.0;     // Fastest speed (ms per step)
//     const float minspeed = 100.0;    // Slowest speed (ms per step)
//     float speed = 1000.0 * maxspeed + (minspeed - maxspeed) * (float)posn / (striplength - 1);  // Calculate the speed based on the position (decelerates as it moves) in us
//     if (knighttimer.expired()) {   // if (knighttimer.elapsed((int)moveInterval)) {
//         posn += direction * 0.1f;    // Move the bright point fractionally
//         if (posn <= 0 || posn >= striplength) {
//             direction = -direction;  // Reverse direction at the ends
//             posn = constrain(posn, 0.0, (float)(striplength - 1));
//         }
//         knighttimer.set(speed);
//         // Serial.printf("%.1lf ", posn);
//         // for (int i = 1; i < striplength; i++) {
//         //     // int x = map((int)(neoobj.GetPixelColor(i).R), 0, 255, 0, 9);
//         //     uint32_t x = (color_to_888(neoobj.GetPixelColor(i))) >> 16;
//         //     // Serial.printf("%d", x);
//         //     Serial.printf("%02x ", x);
//         // }
//         // Serial.printf("\n");
//     }
//     neoobj.ClearTo(0);
//     int iposn = (int)posn;
//     float frac = posn - iposn;
//     if (direction) frac = 1.0 - frac;
//     uint32_t brightcolor = recolor(basecolor, neobright);
//     uint32_t dimcolor = recolor(basecolor, neobright * (1.0 - frac));
//     // Serial.printf("p:%.1lf i:%d f:%.1lf b:%06x d:%06x\n", posn, iposn, frac, brightcolor, dimcolor);
//     // else recolor(basecolor, neobright * (1.0 - (posn - iposn)));
//     neoobj.SetPixelColor(iposn, color_to_neo(brightcolor));  // Set the bright point
//     if (iposn + direction >= 0 && iposn + direction < striplength) {
//         neoobj.SetPixelColor(iposn + direction, color_to_neo(dimcolor));
//     }
//     for (int i = 1; i <= tail; i++) {  // Set the tailing effect
//         int tailposn = posn - i * direction;
//         if (tailposn >= 0 && tailposn < striplength) {
//             neoobj.SetPixelColor(tailposn, color_to_neo(recolor(basecolor, neobright * (tail - i) / trail)));
//         }
//     }
//     neoobj.Show(); // Show the updated strip
// }
//
// float NeopixelStrip::maxelement(float r, float g, float b) { return (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b); }  // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //std::max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
// float NeopixelStrip::midelement(float r, float g, float b) { return (r >= g) ? ((g >= b) ? g : ((r >= b) ? b : r)) : ((r >= b) ? r : ((b >= g) ? g : b)); } // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //std::max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);
// float NeopixelStrip::minelement(float r, float g, float b) { return (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b); } // (rgb[0] > rgb[1]) ? ((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]) : ((rgb[1] > rgb[2]) ? rgb[1] : rgb[2]);  //std::max(rgb[0], rgb[1], rgb[2]);  // (color.r > color.g) ? ((color.r > color.b) ? color.r : color.b) : ((color.g > color.b) ? color.g : color.b);

class IdiotLights {
//   private: bool fixed_one = true, fixed_zero = false; bool* _one = &fixed_one; bool* _zero = &fixed_zero;
  public:
    static constexpr int row_count = 12;
    static constexpr int row_height = 11;
    static constexpr int iconcount = 36;  // number of boolean values included on the screen panel (not the neopixels) 
    bool* vals[iconcount] = {  // 6 per line
        &diag.err_sens_alarm[LOST], &diag.err_sens_alarm[RANGE], &diag.err_sens[RANGE][_TempEng], &wheeltemperr, hotrc.radiolost_ptr(), &panicstop, // &diag.err_sens[RANGE][_TempWheel]
        &shutting_down, &parking, &brake.autostopping, &brake.autoholding, &cruise_adjusting, &car_hasnt_moved, 
        &starter.motor, fuelpump.status_inverse_ptr(), &brake.posn_pid_active, &brake.no_feedback, speedo.pin_level_ptr(), tach.pin_level_ptr(), // fuelpump.status_inverse_ptr(), speedo.pin_inactive_ptr(), tach.pin_inactive_ptr(),
        &nowtouch, &ts_tapped, &ts_doubletapped, encoder.activity_ptr(), &running_on_devboard, &not_syspower,  //  touch.tap_ptr(), touch.doubletap_ptr(), bootbutton.ptr(), sim.enabled_ptr(), &encoder.enc_a, 
        &sensidiots[_Throttle], &sensidiots[_BrakeMotor], &sensidiots[_SteerMotor], &sensidiots[_HotRC], &sensidiots[_Speedo], &sensidiots[_Tach],
        &sensidiots[_BrakePres], &sensidiots[_BrakePosn], &sensidiots[_Temps], &diag.battrangeerr, &sensidiots[_Other], &sensidiots[_GPIO],  // &sensidiots[_MuleBatt]
    };  // , &encoder.enc_b, &starter.req_active, &web_disabled, &powering_up
        // _one, _one, _one, _one, _one, _one, _one, _one, _one, _one, _one, _one, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero, _zero,
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
        { 0x7c, 0x46, 0x7f, 0x7f, 0x33, 0x12, 0x12, 0x12, 0x1e, 0x12, 0x0c, },  // linear actuator or schlong
        { 0x3e, 0x63, 0x41, 0x33, 0x36, 0x1c, 0x1c, 0x36, 0x22, 0x63, 0x41, },  // open loop
        { 0x63, 0x63, 0x63, 0x63, 0x77, 0x3e, 0x1c, 0x00, 0x6e, 0x6b, 0x3b, },  // "S" magnet w/ zap
        { 0x63, 0x63, 0x63, 0x63, 0x77, 0x3e, 0x1c, 0x00, 0x06, 0x7e, 0x06, },  // "T" magnet w/ zap
        { 0x78, 0x7c, 0x7f, 0x7f, 0x7c, 0x7c, 0x1c, 0x0c, 0x0c, 0x0c, 0x0c, },  // finger
        { 0x00, 0x00, 0x00, 0x1c, 0x3e, 0x3e, 0x3e, 0x1c, 0x00, 0x00, 0x00, },  // one dot
        { 0x1c, 0x3e, 0x3e, 0x3e, 0x1c, 0x00, 0x1c, 0x3e, 0x3e, 0x3e, 0x1c, },  // two dots
        { 0x0e, 0x1d, 0x7d, 0x7d, 0x1d, 0x0e, 0x00, 0x7f, 0x6b, 0x6b, 0x63, },  // encoder "E"
        { 0x7f, 0x63, 0x3e, 0x00, 0x7f, 0x6b, 0x6b, 0x00, 0x7f, 0x30, 0x1f, },  // "DEV"
        { 0x00, 0x3e, 0x63, 0x41, 0x40, 0x4f, 0x40, 0x41, 0x63, 0x3e, 0x00, },  // power on/off symbol
        { 0x3e, 0x63, 0x7b, 0x00, 0x7e, 0x13, 0x7e, 0x00, 0x6e, 0x6b, 0x3b, },  // "GAS"
        { 0x3e, 0x49, 0x08, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x08, 0x49, 0x3e, },  // brakes or tie fighter
        { 0x00, 0x1c, 0x26, 0x45, 0x49, 0x79, 0x49, 0x45, 0x26, 0x1c, 0x00, },  // steering wheel
        { 0x00, 0x00, 0x00, 0x72, 0x7f, 0x7f, 0x4f, 0x03, 0x06, 0x00, 0x00, },  // hotrc
        { 0x60, 0x66, 0x06, 0x10, 0x30, 0x63, 0x43, 0x00, 0x6e, 0x6b, 0x3b, },  // gauge "S"
        { 0x60, 0x66, 0x06, 0x10, 0x30, 0x63, 0x43, 0x00, 0x06, 0x7e, 0x06, },  // gauge "T"
        { 0x42, 0x24, 0x2f, 0x44, 0x42, 0x20, 0x22, 0x44, 0x4f, 0x24, 0x22, },  // pressure waves
        { 0x7e, 0x42, 0x46, 0x42, 0x4e, 0x42, 0x46, 0x42, 0x4e, 0x42, 0x7e, },  // ruler
        { 0x02, 0x07, 0x35, 0x77, 0x7a, 0x1c, 0x0e, 0x1f, 0x13, 0x06, 0x04, },  // thermometer
        { 0x7e, 0x77, 0x63, 0x77, 0x7e, 0x7e, 0x7e, 0x77, 0x77, 0x77, 0x7e, },  // car battery
        { 0x7f, 0x6b, 0x6b, 0x00, 0x03, 0x7f, 0x03, 0x00, 0x3e, 0x63, 0x63, },  // "ETC"
        { 0x2a, 0x2a, 0x2a, 0x7f, 0x7d, 0x7f, 0x7f, 0x7f, 0x2a, 0x2a, 0x2a, },  // chip
    };
     // { 0x01, 0x7f, 0x7f, 0x7f, 0x3f, 0x38, 0x74, 0x70, 0x70, 0x70, 0x60, },  // boot
     // { 0x6e, 0x6b, 0x3b, 0x00, 0x7f, 0x00, 0x7f, 0x06, 0x1c, 0x06, 0x7f, },  // "SIM"
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
        "St", "FP", "Pn", "NF", "SM", "TM", "Bt", "NT", "eA", "WD", "Dv", "Pw",
        "Th", "Br", "St", "RC", "Sp", "Tc", "Pr", "Ps", "Tm", "Bt", "Ot", "IO",
    };
    uint8_t color[2][iconcount] = {
     // Colors gradiated for 12 per row, avoiding the dimmest stretch of the blue spectrum
        // { 0x82, 0xa2, 0xc1, 0xcc, 0xd0, 0xb4, 0x74, 0x14, 0x1a, 0x16, 0x0e, 0x0a,  // these are brighter dim versions
        { 0x21, 0x41, 0x61, 0x40, 0x44, 0x68, 0x48, 0x28, 0x08, 0x09, 0x05, 0x01,
          0x21, 0x41, 0x61, 0x40, 0x44, 0x68, 0x48, 0x28, 0x08, 0x09, 0x05, 0x01,
          0x21, 0x41, 0x61, 0x40, 0x44, 0x68, 0x48, 0x28, 0x08, 0x09, 0x05, 0x01, },
        { 0x63, 0xa3, 0xc2, 0xc0, 0xec, 0xf4, 0xd8, 0x9c, 0x1d, 0x1b, 0x13, 0x0b,  
          0x63, 0xa3, 0xc2, 0xc0, 0xec, 0xf4, 0xd8, 0x9c, 0x1d, 0x1b, 0x13, 0x0b, 
          0x63, 0xa3, 0xc2, 0xc0, 0xec, 0xf4, 0xd8, 0x9c, 0x1d, 0x1b, 0x13, 0x0b, }
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
    }
    void setup(NeopixelStrip* _neo) {
        myneo = _neo;
        // int n = new_idiot(&(err_sens_alarm[LOST]), "SL", { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x3e, 0x71, 0x59, 0x4d, 0x47, 0x3e })
        for (int i=0; i<iconcount; i++) myneo->newIdiotLight(i, color[ON][i], val(i));
        ezread.squintf("Idiot lights set up %d icons & %d neopix hazards\n", iconcount, myneo->idiotcount);
    }
    int num_idiots() { return iconcount; }
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

// class IdiotLight {  // defunct: currently not using individual instances for each idiot light. i couldn't get it to work
//     public:
//     bool* val = nullptr;
//     char letters[3] = "--";
//     uint8_t bitmap[11] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
//     uint8_t color = DGRY;
//     bool last;  // = 0;
//     IdiotLight(bool* _val, uint8_t* _map) : val(_val) {
//         for (int i=0; i<11; i++) bitmap[i] = _map[i];
//         last = *val;
//     } 
// };