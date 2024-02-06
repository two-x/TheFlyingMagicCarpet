#pragma once
#include <Arduino.h>
// #define CONFIG_IDF_TARGET_ESP32
#define touch_simbutton 38
#define disp_simbuttons_x 164
#define disp_simbuttons_y 48
#define disp_simbuttons_w (disp_width_pix - disp_simbuttons_x)  // 156
#define disp_simbuttons_h (disp_height_pix - disp_simbuttons_y)  // 192
const uint8_t BLK  = 0x00;  // greyscale: full black (RGB elements off)
const uint8_t DGRY = 0x49;  // pseudo-greyscale: very dark grey (blueish)
const uint8_t MGRY = 0x6d;  // pseudo-greyscale: medium grey (yellowish)
const uint8_t LGRY = 0xb6;  // greyscale: very light grey
const uint8_t WHT  = 0xff;  // greyscale: full white (RGB elements full on)
const uint8_t RED  = 0xe0;  // primary red (R element full on)
const uint8_t YEL  = 0xfc;  // Secondary yellow (RG elements full on)
const uint8_t GRN  = 0x1c;  // primary green (G element full on)
const uint8_t CYN  = 0x1f;  // secondary cyan (GB elements full on)  (00000)(111 111)(11111) = 07 ff
const uint8_t BLU  = 0x03;  // primary blue (B element full on)
const uint8_t MGT  = 0xe2;  // secondary magenta (RB elements full on)
const uint8_t DRED = 0x80;  // dark red
const uint8_t BORG = 0xe8;  // blood orange (very reddish orange)
const uint8_t BRN  = 0x88;  // dark orange aka brown
const uint8_t DBRN = 0x44;  // dark orange aka brown
const uint8_t ORG  = 0xf0;  // 
const uint8_t LYEL = 0xfe;  // 
const uint8_t GGRN = 0x9e;  // a low saturation greyish pastel green
const uint8_t TEAL = 0x1e;  // this teal is barely distinguishable from cyan
const uint8_t STBL = 0x9b;  // steel blue is desaturated light blue
const uint8_t DCYN = 0x12;  // dark cyan
const uint8_t RBLU = 0x0b;  // royal blue
const uint8_t MBLU = 0x02;  // midnight blue
const uint8_t INDG = 0x43;  // indigo (deep blue with a hint of purple)
const uint8_t ORCD = 0x8f;  // orchid (lighter and less saturated purple)
const uint8_t VIO  = 0x83;  // 
const uint8_t PUR  = 0x63;
const uint8_t GPUR = 0x6a;  // a low saturation greyish pastel purple
const uint8_t LPUR = 0xb3;  // a light pastel purple
const uint8_t PNK  = 0xe3;  // pink is the best color
const uint8_t MPNK = 0xeb;  // we need all shades of pink
const uint8_t LPNK = 0xf3;  // especially light pink, the champagne of pinks
const uint8_t NON  = 0x45;  // used as default value when color is unspecified
static constexpr int simgriddir[4][3] = {
    { JOY_PLUS,  JOY_PLUS,  JOY_PLUS,  },
    { JOY_MINUS, JOY_MINUS, JOY_MINUS, },
    { JOY_PLUS,  JOY_UP,    JOY_RT,    },
    { JOY_MINUS, JOY_DN,    JOY_LT,    },
};
static constexpr char simgrid[4][3][4] = {
    { "psi", "rpm", "mph" },
    { "psi", "rpm", "mph" },
    { "pos", "   ", "   " },
    { "pos", "   ", "   " },
};  // The greek mu character we used for microseconds no longer works after switching from Adafruit to tft_espi library. So I switched em to "us" :(

volatile bool _is_running;
// volatile std::uint32_t _draw_count;
volatile std::uint32_t _loop_count;
static constexpr std::uint32_t SHIFTSIZE = 8;
volatile bool flip = 0;
volatile int32_t refresh_limit = 16666; // 16666; // = 60 Hz
Timer screenRefreshTimer = Timer((int64_t)refresh_limit);
LGFX lcd;
LGFX_Sprite framebuf[2];  // , datapage_sp[2], bargraph_sp[2], idiots_sp[2];
struct viewport {
    int32_t x;
    int32_t y;
    int32_t w;
    int32_t h;
};

// volatile int PushSp = 1;
// volatile int DrawSp = 0;
// volatile bool pushed[2];
// volatile bool drawn[2];
class FlexPanel {
  public:
    viewport vp;
    LGFX_Sprite* nowspr;
    LGFX* lcd;
    int touchp[2];
    int corner[2], sprsize[2];
    Touchscreen* _touch;
    // std::size_t flip = 0;
    std::uint32_t sec, psec, _width, _height, _myfps = 0, myfps = 0, frame_count = 0;
    FlexPanel() {}
    // int setflip(bool clear) {  // clear=true blacks out the sprite before drawing on it
    //     bool flipit = false;
    //     if (drawn[flip] && pushed[!flip]) flipit = true;
    //     if (flipit) flip = !flip;
    //     nowspr = &(sp[flip]);
    //     if (flipit && clear) nowspr->clear();
    //     return flip;
    // }
    
    void init(LGFX* _lcd, Touchscreen* touch, int _cornerx, int _cornery, int _sprwidth, int _sprheight) {
        lcd = _lcd;
        _touch = touch;
        set_vp(_cornerx, _cornery, _sprwidth, _sprheight);
        _width = vp.w << SHIFTSIZE;
        _height = vp.h << SHIFTSIZE;
    }
    void set_vp(int _cornerx, int _cornery, int _sprwidth, int _sprheight) {
        vp.x = _cornerx;
        vp.y = _cornery;
        vp.w = _sprwidth;
        vp.h = _sprheight;
    }
    bool touched() {
        if (_touch->touched()) {
            // for (int axis=HORZ; axis<=VERT; axis++)
                touchp[HORZ] = _touch->touch_pt(HORZ) - vp.x;
                touchp[VERT] = _touch->touch_pt(VERT) - vp.y;
            return true;
        }
        return false;
    }
    int touch_pt(int axis) {
        return touchp[axis];
    }
};
class CollisionsSaver {
  public:
    // int flip;
    // int* draw_count_ptr;
    struct ball_info_t {
        int32_t x;
        int32_t y;
        int32_t dx;
        int32_t dy;
        int32_t r;
        int32_t m;
        uint8_t color;
    };
    viewport* vp;
    uint8_t sqrme, slices = 8;
    ball_info_t* balls;
    ball_info_t* a;
    LGFX_Sprite* sprite;
    static constexpr std::uint32_t BALL_MAX = 45;  // 256
    ball_info_t _balls[2][BALL_MAX];
    std::uint32_t _ball_count = 0, _myfps = 0;
    std::uint32_t ball_thismax, ball_count = 0;
    int _width, _height;
    std::uint32_t sec, psec, ball_create_rate = 3200;
    std::uint32_t myfps = 0, frame_count = 0;
    // uint32_t ball_radius_pix_per_10ksqpix = 5;
    float ball_radius_base = 5.0 / 235.0;  // 7 pixels radius / 125x100 sprite = about 5 pix per 235 sides sum
    float ball_radius_modifier = 3.0 / 235.0;  // 4 pixels radius / 125x100 sprite = about 3 pix per...
    uint8_t ball_redoubler_rate = 0x18;  // originally 0x07
    uint8_t ball_gravity = 16;  // originally 0 with suggestion of 4
    volatile bool _is_running;
    volatile std::uint32_t _loop_count = 0;
    CollisionsSaver() {}
    void drawfunc() {
        auto sprwidth = vp->w;
        auto sprheight = vp->h;
        // flip = _draw_count & 1;
        balls = &_balls[flip][0];
        // sprite = &(framebuf[flip]);
        sprite->clear();
        // for (int i = 1; i < slices; i++) {
        //     sprite->drawGradientVLine(0, (i * sprwidth) / slices, sprheight - 1, (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)(i * 65535 / sprwidth)+25*_loop_count, 255, 200) >> 8), (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200) >> 8));
        //     sprite->drawGradientHLine(0, (i * sprheight) / slices, sprwidth - 1, (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)(i * 65535 / sprheight)+25*_loop_count, 255, 200) >> 8), (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200) >> 8));
        // }
        // for (int i = 0; i <= 1; i++) {
        //     sprite->drawGradientVLine(0, i * (sprwidth - 1), sprheight - 1, (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200) >> 8), (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200) >> 8));
        //     sprite->drawGradientHLine(0, i * (sprheight - 1), sprwidth - 1, (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200) >> 8), (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200) >> 8));
        // }
        for (float i = 0.125; i < 1.0; i += 0.125) {
            sprite->drawGradientVLine((int)(i * (vp->w - 1)) + vp->x, vp->y, vp->h, (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200) >> 8), (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200) >> 8));
            sprite->drawGradientHLine(vp->x, (int)(i * (vp->h - 1)) + vp->y, vp->w, (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200) >> 8), (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200) >> 8));
        }
        for (float i = 0.0; i <= 1.0; i += 1.0) {
            sprite->drawGradientVLine((int)(i * (vp->w - 1)) + vp->x, vp->y, vp->h, (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200) >> 8), (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200) >> 8));
            sprite->drawGradientHLine(vp->x, (int)(i * (vp->h - 1)) + vp->y, vp->w, (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200) >> 8), (uint8_t)(hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200) >> 8));
        }

        for (std::uint32_t i = 0; i < _ball_count; i++) {
            a = &balls[i];
            sprite->fillCircle((a->x >> SHIFTSIZE) + vp->x, (a->y >> SHIFTSIZE) + vp->y, a->r >> SHIFTSIZE, (uint8_t)(a->color));
        }
        // _draw_count++;
    }
    void new_ball(int ballno) {
        auto a = &_balls[_loop_count & 1][ballno];
        a->color = lgfx::color332(100 + (rand() % 155), 100 + (rand() % 155), 100 + (rand() % 155));
        // Serial.printf("%02x ", a->color);
        a->x = _width * rn(2);
        a->dx = (rand() & (5 << SHIFTSIZE)) + 1;
        a->dy = (rand() & (5 << SHIFTSIZE)) + 1;
        // a->color = lgfx::color888(100 + (rand() % 155), 100 + (rand() % 155), 100 + (rand() % 155));
        // Serial.printf("%02x ", a->color);
        sqrme = (uint8_t)(ball_radius_base * (float)(vp->w + vp->h));
        sqrme += rn((int)(ball_radius_modifier * (float)(vp->w + vp->h)));
        for (int i=0; i<=2; i++) if (!rn(ball_redoubler_rate)) sqrme *=2;
        a->r = sqrme << SHIFTSIZE;  // (sqrme * sqrme)));
        a->m = 4 + (ball_count & 0x07);
        // a->r = (4 + (i & 0x07)) << SHIFTSIZE;
        // a->m = 4 + (i & 0x07);
    }
    bool mainfunc(void) {
        bool new_round = false;
        static constexpr float e = 0.999;  // Coefficient of friction
        sec = lgfx::millis() / ball_create_rate;
        if (psec != sec) {
            psec = sec;
            myfps = frame_count;
            frame_count = 0;
            if (++ball_count >= ball_thismax) {
                new_round = true;
                ball_count = 1;
            }
            new_ball(ball_count - 1);
        }
        frame_count++;
        _loop_count++;
        ball_info_t *a, *b, *balls;
        int32_t rr, len, vx2vy2;
        float vx, vy, distance, t;
        size_t f = _loop_count & 1;
        balls = a = &_balls[f][0];
        b = &_balls[!f][0];
        memcpy(a, b, sizeof(ball_info_t) * ball_count);
        for (int i = 0; i != ball_count; i++) {
            a = &balls[i];
            a->dy += ball_gravity; // gravity
            a->x += a->dx;
            if (a->x < a->r) {
                a->x = a->r;
                if (a->dx < 0) a->dx = -a->dx * e;
            }
            else if (a->x >= _width - a->r) {
                a->x = _width - a->r - 1;
                if (a->dx > 0) a->dx = -a->dx * e;
            }
            a->y += a->dy;
            if (a->y < a->r) {
                a->y = a->r;
                if (a->dy < 0) a->dy = -a->dy * e;
            }
            else if (a->y >= _height - a->r) {
                a->y = _height - a->r - 1;
                if (a->dy > 0) a->dy = -a->dy * e;
            }
            for (int j = i + 1; j != ball_count; j++) {
                b = &balls[j];
                rr = a->r + b->r;
                vx = a->x - b->x;
                if (abs(vx) > rr) continue;
                vy = a->y - b->y;
                if (abs(vy) > rr) continue;
                len = sqrt(vx * vx + vy * vy);
                if (len >= rr) continue;
                if (len == 0.0) continue;
                distance = (rr - len) >> 1;
                vx *= distance / len;
                vy *= distance / len;
                a->x += vx;
                b->x -= vx;
                vx = b->x - a->x;
                a->y += vy;
                b->y -= vy;
                vy = b->y - a->y;
                vx2vy2 = vx * vx + vy * vy;
                t = -(vx * a->dx + vy * a->dy) / vx2vy2;
                float arx = a->dx + vx * t;
                float ary = a->dy + vy * t;
                t = -(-vy * a->dx + vx * a->dy) / vx2vy2;
                float amx = a->dx - vy * t;
                float amy = a->dy + vx * t;
                t = -(vx * b->dx + vy * b->dy) / vx2vy2;
                float brx = b->dx + vx * t;
                float bry = b->dy + vy * t;
                t = -(-vy * b->dx + vx * b->dy) / vx2vy2;
                float bmx = b->dx - vy * t;
                float bmy = b->dy + vx * t;
                float adx = (a->m * amx + b->m * bmx + bmx * e * b->m - amx * e * b->m) / (a->m + b->m);
                float bdx = -e * (bmx - amx) + adx;
                float ady = (a->m * amy + b->m * bmy + bmy * e * b->m - amy * e * b->m) / (a->m + b->m);
                float bdy = -e * (bmy - amy) + ady;
                a->dx = roundf(adx + arx);
                a->dy = roundf(ady + ary);
                b->dx = roundf(bdx + brx);
                b->dy = roundf(bdy + bry);      
            }
        }
        _myfps = myfps;
        _ball_count = ball_count;
        return new_round;
    }
    // #if defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32) || defined(ESP_PLATFORM)
    //     void taskDraw(void*) {
    //         while (_is_running) {
    //             while (_loop_count == _draw_count) {
    //                 taskYIELD();
    //             }
    //             drawfunc();
    //         }
    //         vTaskDelete(NULL);
    //     }
    // #endif

    void setup(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        _width = vp->w << SHIFTSIZE;
        _height = vp->h << SHIFTSIZE;
        // reset();
    }
    void reset(LGFX_Sprite* sp0, LGFX_Sprite* sp1, viewport* _vp) {
        vp = _vp;
        _width = vp->w << SHIFTSIZE;
        _height = vp->h << SHIFTSIZE;
        LGFX_Sprite* spp[2] = { sp0, sp1 };
        for (int i = 0; i <= 1; i++) {
            spp[i]->setBaseColor(BLK);
            spp[i]->clear();
            spp[i]->setTextSize(1);
            // sprite->setFont(&fonts::Font4);
            spp[i]->setTextDatum(textdatum_t::top_left);
        }
        ball_thismax = BALL_MAX - rn(25);
        for (std::uint32_t i = 0; i < ball_count; ++i) new_ball(i);
        refresh_limit = 20000;  // 50 Hz limit
        screenRefreshTimer.set(refresh_limit);
        _is_running = true;
        // _draw_count = 0;
        // _loop_count = 0;
        #if defined(CONFIG_IDF_TARGET_ESP32)
            xTaskCreate(taskDraw, "taskDraw", 2048, NULL, 0, NULL);
        #endif
    }
    int update(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        bool round_over = mainfunc();
        // #if defined(CONFIG_IDF_TARGET_ESP32)
        //     while (_loop_count != _draw_count) { taskYIELD(); }
        // #else
        drawfunc();
        // #endif
        return !round_over;  // not done yet
    }
    void saver_touch(int, int) {};  // unused
};
class EraserSaver {  // draws colorful patterns to exercise
    enum savershapes : int { Wedges, Dots, Rings, Ellipses, Boxes, Ascii, Rotate, NumSaverShapes };
 private:
    LGFX_Sprite* sprite;
    viewport* vp;
    int sprsize[2], rotate = -1, scaler = 1;
    int point[2], plast[2], er[2];
    int eraser_rad = 14, eraser_rad_min = 22, eraser_rad_max = 40, eraser_velo_min = 3, eraser_velo_max = 7, touch_w_last = 2;
    int erpos[2] = {0, 0}, eraser_velo_sign[2] = {1, 1}, boxsize[2], now = 0;
    int eraser_velo[2] = {rn(eraser_velo_max), rn(eraser_velo_max)}, shapes_per_run = 5, shapes_done = 0;
    int erpos_max[2];
    uint8_t saver_illicit_prob = 12, wclast, pencolor = RED;
    float pensat = 200.0;
    uint16_t spothue = 65535, slowhue = 0, penhue;
    int num_cycles = 3, cycle = 0, boxrad, boxminsize, boxmaxarea = 1200, shape = rn(Rotate);
    static constexpr uint32_t saver_cycletime_us = 18000000;
    Timer saverCycleTimer, pentimer = Timer(1500000);
    bool saver_lotto = false, has_eraser = true;
 public:
    EraserSaver() {}
    void setup(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        // Serial.printf("es: x%d y%d w%d h%d\n", vp->x, vp->y, vp->w, vp->h);
        erpos_max[HORZ] = (int32_t)vp->w / 2 - eraser_rad;
        erpos_max[VERT] = (int32_t)vp->h / 2 - eraser_rad;
        point[HORZ] = rn(vp->w);
        point[VERT] = rn(vp->h);
        for (int axis = 0; axis <= 1; axis++) eraser_velo_sign[axis] = (rn(1)) ? 1 : -1;
        // reset();
    }
    void reset(LGFX_Sprite* sp0, LGFX_Sprite* sp1, viewport* _vp) {
        vp = _vp;
        LGFX_Sprite* spp[2] = { sp0, sp1 };
        shapes_done = cycle = 0;
        for (int i = 0; i <= 1; i++) {
            spp[i]->setBaseColor(BLK);
            spp[i]->setTextSize(1);
            spp[i]->fillSprite(BLK);
            spp[i]->setTextDatum(textdatum_t::middle_center);
            spp[i]->setTextColor(BLK);
            // spp[i]->setFont(&fonts::Font4);
            spp[i]->setCursor(vp->w / 2 + vp->x, vp->h / 2 + vp->y);
        }
        change_pattern(-2);  // randomize new pattern whenever turned off and on
        saverCycleTimer.set(saver_cycletime_us);
        refresh_limit = 11111;  // 90 Hz limit
        screenRefreshTimer.set(refresh_limit);
        scaler = std::max(1, (vp->w + vp->h)/200);
        // _draw_count = _loop_count = 0;
        _is_running = true;
    }
    // void saver_touch(int16_t x, int16_t y) {  // you can draw colorful lines on the screensaver
    //     int tp[2] = {(int32_t)x - (int32_t)corner[HORZ], (int32_t)y - (int32_t)corner[VERT]};
    //     if (tp[HORZ] < 0 || tp[VERT] < 0) return;
    //     for (int axis = HORZ; axis <= VERT; axis++)
    //         if (touchlast[axis] == -1) touchlast[axis] = tp[axis];
    //     if (pentimer.expireset()) {
    //         pensat += 1.5;
    //         if (pensat > 255.0) pensat = 100.0;
    //         pencolor = (cycle == 1) ? rando_color() : hsv_to_rgb<uint8_t>(++penhue, (uint8_t)pensat, 200 + rn(56));
    //     }
    //     sprite->fillCircle(touchlast[HORZ], touchlast[VERT], 20, pencolor);
    //     // sprite->drawWedgeLine(touchlast[HORZ], touchlast[VERT], tp[HORZ], tp[VERT], 4, 4, pencolor, pencolor);  // savtouch_last_w, w, pencolor, pencolor);
    //     // for (int i=-7; i<=8; i++)
    //     //     sprite->drawLine(touchlast[HORZ]+i, touchlast[VERT]+i, tp[HORZ]+i, tp[VERT]+i, pencolor);  // savtouch_last_w, w, pencolor, pencolor);
    //     for (int axis = HORZ; axis <= VERT; axis++) touchlast[axis] = tp[axis];
    // }
    void saver_touch(LGFX_Sprite* spr, int x, int y) {  // you can draw colorful lines on the screensaver
        if (pentimer.expireset()) {
            pensat += 1.5;
            if (pensat > 255.0) pensat = 100.0;
            penhue += 150;
            pencolor = (cycle == 1) ? rando_color() : hsv_to_rgb<uint8_t>(penhue, (uint8_t)pensat, 200 + rn(56));
        }
        spr->fillCircle(x + vp->x, y + vp->y, 20 * scaler, pencolor);
    }
    int update(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        if (saverCycleTimer.expired()) {
            ++cycle %= num_cycles;
            if (cycle == 0) change_pattern(-1);
            saverCycleTimer.set(saver_cycletime_us / ((cycle == 2) ? 5 : 1));
        }
        drawsprite();
        return shapes_done;
    }
  private:
    void drawsprite() {
        // Serial.printf("\r%d,%d,%d ", shape, shapes_done, cycle);
        point[HORZ] = rn(vp->w);
        point[VERT] = rn(vp->h);
        if (shape == Rotate) ++rotate %= NumSaverShapes;
        else rotate = shape;
        if ((cycle != 2) || !has_eraser) {
            spothue -= 10;
            if (!rn(20)) spothue = rn(65535);
            if (spothue & 1) slowhue += 13;
            if (rotate == Wedges) {
                uint8_t wc = hsv_to_rgb<uint8_t>(rn(65536), 127 + (spothue >> 9));
                float im = 0;
                if (plast[VERT] != point[VERT]) im = (float)(plast[HORZ] - point[HORZ]) / (float)(plast[VERT] - point[VERT]);
                sprite->fillCircle(plast[HORZ] + vp->x, plast[VERT] + vp->y, 3, wc);
                // sprite->drawCircle(point[HORZ], point[VERT], 3, BLK);
                for (int h=-4; h<=4; h++)
                    sprite->drawGradientLine(point[HORZ] + vp->x, point[VERT] + vp->y, plast[HORZ]  + vp->x + (int)(h / ((std::abs(im) > 1.0) ? im : 1)), plast[VERT] + vp->y + (int)(h * ((std::abs(im) > 1.0) ? 1 : im)), wclast, wc);
                wclast = wc;
            }
            else if (rotate == Ellipses) {
                int d[2] = {10 + rn(30), 10 + rn(30)};
                uint8_t sat, brt;
                uint16_t mult = rn(2000), hue = spothue + rn(3000);
                sat = 100 + rn(156);
                brt = 120 + rn(136);
                // Serial.printf("\n%d/%d: ", mult, hue);
                for (int i = 0; i < 6 + rn(20); i++) { 
                    sprite->drawEllipse(point[HORZ] + vp->x, point[VERT] + vp->y, scaler * d[0] - i, scaler * d[1] + i, hsv_to_rgb<uint8_t>(hue + mult * i, sat, brt));
                    // Serial.printf("%d ", hue + mult * i);
                }

            }
            else if (rotate == Rings) {
                int d = 8 + rn(25);
                uint16_t hue = spothue + 32768 * rn(2);
                uint8_t sat = rn(128) + (spothue >> 9);
                uint8_t brt = 180 + rn(76);
                uint8_t c = hsv_to_rgb<uint8_t>(hue, sat, brt);
                uint8_t c2 = hsv_to_rgb<uint8_t>(hue, sat, brt-10);
                // Serial.printf("%3.0f%3.0f%3.0f (%3.0f%3.0f%3.0f) (%3.0f%3.0f%3.0f)\n", (float)(hue/655.35), (float)(sat/2.56), (float)(brt/2.56), 100*(float)((c >> 11) & 0x1f)/(float)0x1f, 100*(float)((c >> 5) & 0x3f)/(float)0x3f, 100*(float)(c & 0x1f)/(float)0x1f, 100*(float)((c2 >> 11) & 0x1f)/(float)0x1f, 100*(float)((c2 >> 5) & 0x3f)/(float)0x3f, 100*(float)(c2 & 0x1f)/(float)0x1f);
                for (int xo = -1; xo <= 1; xo += 2) {
                    sprite->drawCircle(point[HORZ] + vp->x, point[VERT] + vp->y + xo, d * scaler, c);
                    sprite->drawCircle(point[HORZ] + vp->x + xo, point[VERT] + vp->y, d * scaler, c);
                }
                for (int edge = -1; edge <= 1; edge += 2)
                    sprite->drawCircle(point[HORZ] + vp->x, point[VERT] + vp->y, d * scaler + edge, c2);
            }
            else if (rotate == Dots) {
                for (int star = 0; star < 12; star++)
                    sprite->fillCircle(rn(vp->w) + vp->x, rn(vp->h) + vp->y, scaler * (2 + rn(2)), hsv_to_rgb<uint8_t>((uint16_t)(spothue + (spothue >> 2) * rn(3)), 128 + rn(128), 160 + rn(96)));  // hue_to_rgb16(rn(255)), BLK);
            }
            else if (rotate == Boxes) {
                boxrad = 2 + rn(2);
                boxminsize = 2 * boxrad + 10;
                int longer = rn(2);
                boxsize[longer] = boxminsize + rn(vp->w - boxminsize);
                boxsize[!longer] = boxminsize + rn(smax(0, boxmaxarea / boxsize[longer] - boxminsize));
                for (int dim = 0; dim <= 1; dim++) point[dim] = -boxsize[dim] / 2 + rn(sprsize[dim]);
                sprite->fillSmoothRoundRect(point[HORZ] + vp->x, point[VERT] + vp->y, boxsize[HORZ], boxsize[VERT], boxrad, rando_color());  // Change colors as needed
            }
            else if (rotate == Ascii) {
                // sprite->setFont(&fonts::Font4);
                sprite->setTextSize(1);
                sprite->setFont(&fonts::Font4);
                for (int star = 0; star < 4; star++) {
                    point[HORZ] = rn(vp->w);
                    point[VERT] = rn(vp->h);
                    uint16_t hue = point[VERT] * 65535 / vp->h;
                    uint8_t sat = point[HORZ] * 255 / vp->w;
                    String letter = (String)((char)(0x21 + rn(0x5d)));
                    uint8_t c = hsv_to_rgb<uint8_t>(hue, sat, 100 + 100 * (spothue > 32767) + rn(56));
                    sprite->drawString(letter, point[HORZ]+1 + vp->x, point[VERT]+1 + vp->y);
                    sprite->drawString(letter, point[HORZ]-1 + vp->x, point[VERT]-1 + vp->y);
                    sprite->setTextColor(hsv_to_rgb<uint8_t>(hue, sat, 100 + 100 * (spothue > 32767) + rn(56)));
                    sprite->drawString(letter, point[HORZ] + vp->x, point[VERT] + vp->y);
                    sprite->setTextColor(BLK);  // allows subliminal messaging
                }
                // sprite->setTextSize(0);
                sprite->setFont(&fonts::Font0);
            }
        }
        if ((cycle != 0) && has_eraser) {
            for (int axis = HORZ; axis <= VERT; axis++) {
                erpos[axis] += eraser_velo[axis] * eraser_velo_sign[axis];
                if (erpos[axis] * eraser_velo_sign[axis] >= erpos_max[axis]) {
                    // Serial.printf(" w:%3ld h:%3ld l:%3ld m:%3ld ", vp->w, vp->h, erpos[axis] * eraser_velo_sign[axis], erpos_max[axis]);
                    erpos[axis] = eraser_velo_sign[axis] * erpos_max[axis];
                    eraser_velo[axis] = eraser_velo_min + rn(eraser_velo_max - eraser_velo_min);
                    eraser_velo[!axis] = eraser_velo_min + rn(eraser_velo_max - eraser_velo_min);
                    eraser_velo_sign[axis] *= -1;
                    eraser_rad = constrain((int)(eraser_rad + rn(5) - 2), eraser_rad_min, eraser_rad_max);
                }
            }
            // Serial.printf(" e %3d,%3d,%3d,%3d,%3d\n", (vp->w / 2) + erpos[HORZ], (vp->h / 2) + erpos[VERT], eraser_rad, eraser_velo[HORZ], eraser_velo[VERT]);
            // sprite->fillCircle((vp->w / 2) + erpos[HORZ], (vp->h / 2) + erpos[VERT], 20, pencolor);
            sprite->fillCircle((vp->w / 2) + erpos[HORZ] + vp->x, (vp->h / 2) + erpos[VERT] + vp->y, eraser_rad * scaler, (uint8_t)BLK);
        }
        if (saver_lotto) {
            sprite->setFont(&fonts::Font4);
            sprite->drawString("do drugs", vp->w / 2 + vp->x, vp->h / 2 + vp->y);
            sprite->setFont(&fonts::Font0);
        }
        for (int axis = HORZ; axis <= VERT; axis++) plast[axis] = point[axis];  // erlast[axis] = erpos[axis];
        // _draw_count++;
    }
    void change_pattern(int newpat = -1) {  // pass non-negative value for a specific pattern, or  -1 for cycle, -2 for random
        ++shapes_done %= 5;
        int last_pat = shape;
        saver_lotto = !rn(saver_illicit_prob);
        has_eraser = !rn(2);
        if (0 <= newpat && newpat < NumSaverShapes) shape = newpat;  //
        else {
            if (newpat == -1) ++shape %= Rotate;
            else if (newpat == -2) while (last_pat == shape) shape = rn(Rotate);
            if (!rn(25)) shape = Rotate;
        }
        // for (int i = 0; i <= 1; i++)
        //     if (shape == Ascii) framebuf[i].setFont(&fonts::Font4);
        //     else framebuf[i].setFont(&fonts::Font4);
    }
};
class AnimationManager {
  private:
    enum saverchoices : int { Eraser, Collisions, NumSaverMenu, Blank };
    int nowsaver = Collisions, still_running = 0;
    LGFX* mylcd;
    LGFX_Sprite* nowspr_ptr;
    LGFX_Sprite arrowspr;
    FlexPanel* panel;
    EraserSaver eSaver;
    CollisionsSaver cSaver;
    Simulator* sim;
    // Timer saverRefreshTimer = Timer(16666);
    Timer fps_timer;
    float myfps = 0.0;
    viewport* vp;
    int64_t fps_mark;
    bool screensaver_last = false, simulating_last = false, mule_drawn = false;
    void change_saver() {  // pass non-negative value for a specific pattern, -1 for cycle, -2 for random
        ++nowsaver %= NumSaverMenu;
        reset();
    }
  public:
    AnimationManager() {}
    void init(FlexPanel* _panel, Simulator* _sim) {
        vp = &(_panel->vp);
        panel = _panel;
        sim = _sim;
    }
    void setup() {
        // int flip = panel->setflip(true);
        eSaver.setup(&framebuf[flip], vp);
        cSaver.setup(&framebuf[flip], vp);
    }
    void reset() {
        // int flip = panel->setflip(true);
        if (nowsaver == Eraser) eSaver.reset(&framebuf[flip], &framebuf[!flip], vp);
        else if (nowsaver == Collisions) cSaver.reset(&framebuf[flip], &framebuf[!flip], vp);
    }
    // void redraw() {
    //     // if (!is_drawing) is_pushing = false;
    //     panel->diffpush(&framebuf[flip], &framebuf[!flip]);
    // }
    void draw_simbutton(LGFX_Sprite* spr, int cntr_x, int cntr_y, int dir, uint8_t color) {
        if (dir == JOY_PLUS)  spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_plus_32x32x8, BLK);
        else if (dir == JOY_MINUS) spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_minus_32x32x8, BLK);
        else if (dir == JOY_UP) spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == JOY_DN) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 180, 1, 1, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == JOY_LT) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 270, 1, 1, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == JOY_RT) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 90, 1, 1, 32, 32, blue_up_32x32x8, BLK);

    // void pushImageRotateZoom(float dst_x, float dst_y, float src_x, float src_y, float angle, float zoom_x, float zoom_y, int32_t w, int32_t h, const void* data, uint32_t transparent, color_depth_t depth, const T* palette)
    }

    void draw_simbuttons (LGFX_Sprite* spr, bool create) {  // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
        if (!create) {
            spr->fillSprite(BLK);
            return;
        }
        spr->setTextDatum(textdatum_t::middle_center);
        spr->setFont(&fonts::Font2);
        for (int32_t row = 0; row < arraysize(simgrid); row++) {
            for (int32_t col = 0; col < arraysize(simgrid[row]); col++) {
                int32_t cntr_x = touch_cell_h_pix*col + (touch_cell_h_pix>>1) + 2 + vp->x - 5;
                int32_t cntr_y = touch_cell_v_pix*row + (touch_cell_v_pix>>1) + vp->y;
                if (strcmp(simgrid[row][col], "    ")) {
                    draw_simbutton(spr, cntr_x + 2, cntr_y - 1, simgriddir[row][col], YEL);  // for 3d look
                    draw_simbutton(spr, cntr_x, cntr_y, simgriddir[row][col], DGRY);
                    // spr->fillRoundRect(cntr_x - 20, cntr_y - touch_cell_v_pix/2 - 10, 40, 20, 5, DGRY);
                    // spr->drawRoundRect(cntr_x - 20, cntr_y - touch_cell_v_pix/2 - 10, 40, 20, 5, BLK);
                    if (row % 2) {
                        spr->setFont(&fonts::FreeSans9pt7b);
                        spr->setTextColor(BLK);
                        spr->drawString(simgrid[row][col], cntr_x - 1, cntr_y - touch_cell_v_pix/2 + 5 - 1);
                        spr->drawString(simgrid[row][col], cntr_x + 1, cntr_y - touch_cell_v_pix/2 + 5 + 1);
                        spr->setTextColor(LYEL);
                        spr->drawString(simgrid[row][col], cntr_x, cntr_y - touch_cell_v_pix/2 + 5);
                    }
                }
            }     
        }
        // draw_reticles(spr);
        // spr->setTextDatum(textdatum_t::top_left);
        // spr->setFont(&fonts::Font0);
        spr->setTextColor(BLK);
    }
    void calc_fps() {
        int64_t now = fps_timer.elapsed();
        myfps = (float)(now - fps_mark);
        if (myfps > 0.001) myfps = 1000000 / myfps;
        fps_mark = now;
    }
    float update() {
        nowspr_ptr = &framebuf[flip];
        nowspr_ptr->setClipRect(vp->x, vp->y, vp->w, vp->h);
        if (!screensaver_last && screensaver) change_saver();  // ptrsaver->reset();
        screensaver_last = screensaver;
        if (screensaver) {        // With timer == 16666 drawing dots, avg=8k, peak=17k.  balls, avg 2.7k, peak 9k after 20sec
            mule_drawn = false;
            // With max refresh drawing dots, avg=14k, peak=28k.  balls, avg 6k, peak 8k after 20sec
            if (nowsaver == Eraser) still_running = eSaver.update(nowspr_ptr, vp);
            else if (nowsaver == Collisions) still_running = cSaver.update(nowspr_ptr, vp);
            if (panel->touched()) eSaver.saver_touch(nowspr_ptr, panel->touch_pt(HORZ), panel->touch_pt(VERT));
            if (!still_running) change_saver();
        }
        else if (!mule_drawn) {
            nowspr_ptr->fillSprite(BLK);
            nowspr_ptr->pushImageRotateZoom(85 + vp->x , 85 + vp->y, 82, 37, 0, 1, 1, 145, 74, mulechassis_145x74x8, BLK);
            mule_drawn = true;
        }
        // if (fullscreen_screensaver_test) {
        //     nowspr_ptr->setCursor(0,0);
        //     nowspr_ptr->printf("fps:%03d", myfps);
        // }
        if (sim->enabled()) {
            draw_simbuttons(nowspr_ptr, sim->enabled());  // if we just entered simulator draw the simulator buttons, or if we just left erase them
        }
        else if (simulating_last) {
            nowspr_ptr->fillSprite(BLK);
            mule_drawn = false;
        }
        simulating_last = sim->enabled();
        // if (disp_simbuttons_dirty || sim->enabled()) {
        //     draw_simbuttons(sim->enabled());  // if we just entered simulator draw the simulator buttons, or if we just left erase them
        //     disp_simbuttons_dirty = false;
        //     simulating_last = sim->enabled();
        //     _procrastinate = true;
        // }
        calc_fps();
        nowspr_ptr->clearClipRect();
        return myfps;
    }
};
class DiagConsole {
  private:
    LGFX* mylcd;
    LGFX_Sprite* nowspr_ptr;
    FlexPanel* panel;
    static constexpr int num_lines = 16;
    std::string textlines[num_lines];
    int usedlines = 0;
  public:
    DiagConsole() {}
    void init(FlexPanel* _panel) {
        panel = _panel;
    }
    void setup() {}
    // void redraw() {
    //     panel->diffpush(&framebuf[flip], &framebuf[!flip]);
    // }
    void add_errorline(std::string type, std::string item) {
        std::string newerr = type + ": " + item;
        if (newerr.length() > 15) newerr = newerr.substr(0, 15);
        textlines[usedlines++] = newerr;
    }
    void update() {
        // int flip = panel->setflip(false);
        // nowspr_ptr = &(framebuf[flip]);
        // panel->diffpush(&framebuf[flip], &framebuf[!flip]);
    }
};

#ifdef CONVERT_IMAGE
#define IMAGE_ARRAY mulechassis_145x74
#define IMAGE_WIDTH 145
void convert_565_to_332_image() {
    Serial.printf("const uint8_t %sx8[%ld] PROGMEM = {\n\t", String(IMAGE_ARRAY).c_str(), arraysize(IMAGE_ARRAY));
    for (int i=0; i<arraysize(IMAGE_ARRAY); i++) {
        Serial.printf("0x%02x, ", color_16b_to_8b(IMAGE_ARRAY[i]);
        if (!(i % IMAGE_WIDTH)) Serial.printf("  // pixel# %ld\n%s", pixcount, (i < arraysize(IMAGE_ARRAY) - 1) ? "\t" : "");
    }
    Serial.printf("};\n");
}
#endif
