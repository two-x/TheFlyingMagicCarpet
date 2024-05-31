#pragma once
#include <Arduino.h>
#include "tftsetup.h"
#include "inputs.h"
static Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
static MomentaryButton bootbutton(boot_sw_pin, false);

// #define touch_simbutton 38
#define disp_simbuttons_x 164
#define disp_simbuttons_y 48
#define disp_simbuttons_w (disp_width_pix - disp_simbuttons_x)  // 156
#define disp_simbuttons_h (disp_height_pix - disp_simbuttons_y)  // 192
#define disp_font_height 8
#define disp_font_width 6
int simgriddir[4][3] = {
    { JOY_PLUS,  JOY_PLUS,  JOY_PLUS,  },
    { JOY_MINUS, JOY_MINUS, JOY_MINUS, },
    { JOY_PLUS,  JOY_UP,    JOY_RT,    },
    { JOY_MINUS, JOY_DN,    JOY_LT,    },
};
std::string simgrid[4][3] = {
    { "psi", "rpm", "mph" },
    { "psi", "rpm", "mph" },
    { "pos", "joy", "joy" },
    { "pos", "joy", "joy" },
};  // The greek mu character we used for microseconds no longer works after switching from Adafruit to tft_espi library. So I switched em to "us" :(

volatile bool _is_running;
volatile std::uint32_t _loop_count;
static constexpr std::uint32_t SHIFTSIZE = 8;
volatile bool flip = 0;
volatile int32_t refresh_limit = 11111; // 16666; // = 60 Hz,   11111 = 90 Hz
volatile bool auto_saver_enabled = false;
volatile int32_t screen_refresh_time;
// Timer screenRefreshTimer = Timer((int64_t)refresh_limit);
LGFX lcd;
LGFX_Sprite framebuf[2];  // , datapage_sp[2], bargraph_sp[2], idiots_sp[2];
struct viewport {
    int32_t x;
    int32_t y;
    int32_t w;
    int32_t h;
};
class CollisionsSaver {
  public:
    struct ball_info_t {
        int32_t x;
        int32_t y;
        int32_t dx;
        int32_t dy;
        int32_t r;
        int32_t m;
        uint8_t color;
    };
    bool touchnow = false, touchlast;
    bool touchball_invisible = true;
    viewport* vp;
    uint8_t sqrme, slices = 8;
    ball_info_t* balls;
    ball_info_t* a;
    static constexpr int touchball_r = 15;
    ball_info_t touchball = { 100, 100, 0, 0, touchball_r << SHIFTSIZE, 10, GRN };
    LGFX_Sprite* sprite;
    static constexpr std::uint32_t BALL_MAX = 35;  // 256
    ball_info_t _balls[2][BALL_MAX];
    std::uint32_t _ball_count = 0, _myfps = 0;
    std::uint32_t ball_thismax, ball_count = 0;
    int _width, _height, touchx, touchy, lastx, lasty;
    std::uint32_t sec, psec, ball_create_rate = 3200;
    std::uint32_t myfps = 0, frame_count = 0;
    float ball_radius_base = 4.5 / 235.0;  // 7 pixels radius / 125x100 sprite = about 5 pix per 235 sides sum
    float ball_radius_modifier = 2.6 / 235.0;  // 4 pixels radius / 125x100 sprite = about 3 pix per...
    uint8_t ball_redoubler_rate = 0x18;  // originally 0x07
    uint8_t ball_gravity = 16;  // originally 0 with suggestion of 4
    volatile bool _is_running;
    volatile std::uint32_t _loop_count = 0;
    CollisionsSaver() {}
    void drawfunc() {
        auto sprwidth = vp->w;
        auto sprheight = vp->h;
        balls = &_balls[flip][0];
        sprite->fillSprite(BLK);
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
        if (touchnow && !touchball_invisible) sprite->fillCircle((touchball.x >> SHIFTSIZE) + vp->x, (touchball.y >> SHIFTSIZE) + vp->y, touchball.r >> SHIFTSIZE, touchball.color);
        touchnow = false;
    }
    void new_ball(int ballno) {
        auto a = &_balls[_loop_count & 1][ballno];
        a->color = lgfx::color332(100 + (rand() % 155), 100 + (rand() % 155), 100 + (rand() % 155));
        a->x = _width * rn(2);
        a->dx = (rand() & (5 << SHIFTSIZE)) + 1;
        a->dy = (rand() & (5 << SHIFTSIZE)) + 1;
        sqrme = (uint8_t)(ball_radius_base * (float)(vp->w + vp->h));
        sqrme += rn((int)(ball_radius_modifier * (float)(vp->w + vp->h)));
        for (int i=0; i<2; i++) if (!rn(ball_redoubler_rate)) sqrme *=2;
        a->r = sqrme << SHIFTSIZE;  // (sqrme * sqrme)));
        a->m = 4 + (ball_count & 0x07);
    }
    void saver_touch(LGFX_Sprite* spr, int x, int y) {  // you can draw colorful lines on the screensaver
        // pencolor = (cycle == 1) ? rando_color() : hsv_to_rgb<uint8_t>(penhue, (uint8_t)pensat, 200 + rn(56));
        lastx = touchball.x;
        lasty = touchball.y;
        touchx = x;
        touchy = y;
        touchnow = true;
        touchball.x = constrain(touchx, touchball_r, vp->w - touchball_r) << SHIFTSIZE;
        touchball.y = constrain(touchy, touchball_r, vp->h - touchball_r) << SHIFTSIZE;
        touchball.dx = (touchball.x - lastx) / 2;
        touchball.dy = (touchball.y - lasty) / 2;
    }
    // ball_info_t temp_ball(int32_t x, int32_t y, int32_t dx, int32_t dy, int32_t r, int32_t m, uint8_t color) {
    //     return { x, y, dx, dy, r, m, color }; // Uniform initialization
    // }
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
        for (int i = 0; i < ball_count + touchnow; i++) {
            a = (i == ball_count) ? &touchball : &balls[i];
            if (i < ball_count) {
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
            }
            for (int j = i + 1; j < ball_count + touchnow; j++) {
                b = (j == ball_count) ? &touchball : &balls[j];
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
                if (j < ball_count) b->x -= vx;
                vx = b->x - a->x;
                a->y += vy;
                if (j < ball_count) b->y -= vy;
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
                if (j < ball_count) {
                    b->dx = roundf(bdx + brx);
                    b->dy = roundf(bdy + bry);
                }
            }
        }
        _myfps = myfps;
        _ball_count = ball_count;
        return new_round;
    }
    void setup(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        _width = vp->w << SHIFTSIZE;
        _height = vp->h << SHIFTSIZE;
    }
    void reset(LGFX_Sprite* sp0, LGFX_Sprite* sp1, viewport* _vp) {
        vp = _vp;
        _width = vp->w << SHIFTSIZE;
        _height = vp->h << SHIFTSIZE;
        LGFX_Sprite* spp[2] = { sp0, sp1 };
        // spp[0]->fillSprite(BLK);
        for (int i = 0; i <= 1; i++) {
            // spp[i]->setBaseColor(BLK);
            spp[i]->setTextSize(1);
            spp[i]->setTextDatum(textdatum_t::top_left);
        }
        ball_thismax = BALL_MAX - rn(25);
        for (std::uint32_t i = 0; i < ball_count; ++i) new_ball(i);
        // screenRefreshTimer.set(refresh_limit);
        _is_running = true;
    }
    int update(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        bool round_over = mainfunc();
        drawfunc();
        return !round_over;  // not done yet
    }
    void saver_touch(int, int) {};  // unused
};
class EraserSaver {  // draws colorful patterns to exercise
    enum savershapes : int { Wedges, Dots, Rings, Ellipses, Boxes, Ascii, Worm, Rotate, NumSaverShapes };
 private:
    LGFX_Sprite* sprite;
    viewport* vp;
    int wormpos[2] = {0, 0}, wormvel[2] = {0, 0}, wormsign[2] = {1, 1}, wormd[2] = {20, 20};
    int shifter = 2, wormdmin = 8, wormdmax = 50, wormvelmax = 512, wormsat = 128;
    uint32_t boxsize[2], huebase = 0;
    int sprsize[2], rotate = -1, scaler = 1, season = 0, numseasons = 4;
    int point[2], plast[2], er[2], erpos_max[2], wormstripe = 2;
    int eraser_rad = 14, eraser_rad_min = 22, eraser_rad_max = 40, eraser_velo_min = 3, eraser_velo_max = 7, touch_w_last = 2;
    int erpos[2] = {0, 0}, eraser_velo_sign[2] = {1, 1}, now = 0;
    int eraser_velo[2] = {rn(eraser_velo_max), rn(eraser_velo_max)}, shapes_per_run = 5, shapes_done = 0;
    uint8_t wclast, pencolor = RED;
    float pensat = 200.0;
    uint16_t spothue = 65535, slowhue = 0, penhue = rn(65535);
    int num_cycles = 3, cycle = 0, boxrad, boxminsize, boxmaxarea = 200, shape = rn(Rotate), pensatdir = 1;
    static constexpr uint32_t saver_cycletime_us = 18000000;
    Timer saverCycleTimer, pentimer = Timer(70000), lucktimer, seasontimer;
    Timer wormmovetimer = Timer(20000), wormtimer = Timer(1000000), wormstripetimer = Timer(2850000);
    bool saver_lotto = false, has_eraser = true;
 public:
    EraserSaver() {}
    void setup(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        lucktimer.set((2 + rn(5)) * 10000000);
    }
    void reset(LGFX_Sprite* sp0, LGFX_Sprite* sp1, viewport* _vp) {
        vp = _vp;
        LGFX_Sprite* spp[2] = { sp0, sp1 };
        shapes_done = cycle = 0;
        // spp[0]->fillSprite(BLK);
        for (int i = 0; i <= 1; i++) {
            // spp[i]->setBaseColor(BLK);
            spp[i]->setTextSize(1);
            spp[i]->setTextDatum(textdatum_t::middle_center);
            spp[i]->setTextColor(BLK);
            spp[i]->setCursor(vp->w / 2 + vp->x, vp->h / 2 + vp->y);
        }
        change_pattern(-2);  // randomize new pattern whenever turned off and on
        saverCycleTimer.set(saver_cycletime_us);
        // refresh_limit = 11111;  // 90 Hz limit
        // screenRefreshTimer.set(refresh_limit);
        seasontimer.set(3000000);
        scaler = std::max(1, (vp->w + vp->h)/200);
        erpos_max[HORZ] = (int32_t)vp->w / 2 - eraser_rad;
        erpos_max[VERT] = (int32_t)vp->h / 2 - eraser_rad;
        point[HORZ] = rn(vp->w);
        point[VERT] = rn(vp->h);
        for (int axis = 0; axis <= 1; axis++) eraser_velo_sign[axis] = (rn(1)) ? 1 : -1;
        _is_running = true;
    }
    void saver_touch(LGFX_Sprite* spr, int x, int y) {  // you can draw colorful lines on the screensaver
        // pencolor = (cycle == 1) ? rando_color() : hsv_to_rgb<uint8_t>(penhue, (uint8_t)pensat, 200 + rn(56));
        spr->fillCircle(x + vp->x, y + vp->y, 20 * scaler, pencolor);
    }
    int update(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        if (saverCycleTimer.expired()) {
            ++cycle %= num_cycles;
            if (cycle == 0) change_pattern(-1);
            saverCycleTimer.set((saver_cycletime_us / ((cycle == 2) ? 5 : 1)) << (shape == Worm));
        }
        if (seasontimer.expireset()) {
            ++season %= numseasons;
            seasontimer.set(2000000 * (1 + rn(4)));
        }
        drawsprite();
        return shapes_done;
    }
  private:
    void drawsprite() {
        point[HORZ] = rn(vp->w);
        point[VERT] = rn(vp->h);
        if (shape == Rotate) ++rotate %= NumSaverShapes;
        else rotate = shape;
        if ((cycle != 2) || !has_eraser) {
            // if (!rn(35)) huebase = rn(1 << 21);
            // else ++huebase %= (1 << 21);
            // spothue = (uint16_t)(huebase >> 5);
            if (pentimer.expireset()) {
                if (season <= 1) {
                    pensat += (float)pensatdir * 1.5;
                    if (pensat > 255.0) {
                        pensat = 255;
                        pensatdir = -1;
                    }
                    else if (pensat < 100.0) {
                        pensat = 100;
                        pensatdir = 1;
                    };
                }
                else penhue += 255;
                pencolor = hsv_to_rgb<uint8_t>(penhue, (uint8_t)pensat, 200 + rn(56));
            }
            spothue -= 2;
            if (!rn(20)) spothue = rn(65535);
            if (spothue & 1) slowhue += 13;
            if (rotate == Wedges) {
                uint8_t wcball, wctip;
                uint16_t hue = rn(65536);
                uint8_t brt = 156 + rn(100);
                if (season == 1) {
                    wctip = hsv_to_rgb<uint8_t>(hue, 0, brt);
                    wcball = hsv_to_rgb<uint8_t>(hue, 64, brt);
                }
                else if (season == 3) {
                    wctip = hsv_to_rgb<uint8_t>(hue, 0, brt);
                    wcball = hsv_to_rgb<uint8_t>(hue, 64, rn(64));
                }
                else {
                    wcball = hsv_to_rgb<uint8_t>(hue, 127 + (spothue >> 9), 200 + rn(56));
                    wctip = wclast;
                }
                float im = 0;
                if (plast[VERT] != point[VERT]) im = (float)(plast[HORZ] - point[HORZ]) / (float)(plast[VERT] - point[VERT]);
                sprite->fillCircle(plast[HORZ] + vp->x, plast[VERT] + vp->y, 3, wcball);
                // sprite->drawCircle(point[HORZ], point[VERT], 3, BLK);
                for (int h=-4; h<=4; h++)
                    sprite->drawGradientLine(point[HORZ] + vp->x, point[VERT] + vp->y, plast[HORZ]  + vp->x + (int)(h / ((std::abs(im) > 1.0) ? im : 1)), plast[VERT] + vp->y + (int)(h * ((std::abs(im) > 1.0) ? 1 : im)), wctip, wcball);
                wclast = wcball;
            }
            else if (rotate == Ellipses) {
                int d[2] = {10 + rn(30), 10 + rn(30)};
                uint8_t sat, brt;
                uint16_t mult = rn(2000), hue = spothue + rn(3000);
                sat = 100 + rn(156);
                brt = 120 + rn(136);
                for (int i = 0; i < 6 + rn(20); i++) { 
                    sprite->drawEllipse(point[HORZ] + vp->x, point[VERT] + vp->y, scaler * d[0] - i, scaler * d[1] + i, hsv_to_rgb<uint8_t>(hue + mult * i, sat, brt));
                }
            }
            else if (rotate == Rings) {
                int d = 6 + rn(45);
                uint8_t sat, brt, c, c2;
                uint16_t hue = spothue + 32768 * rn(2);
                sat = 255 - ((uint8_t)(spothue >> (8+season)));  // + rn(63) 
                brt = 175 + rn(56);
                if (season == 0) { sat = 100; brt = 200; }
                if (season == 3) { sat = 255; brt = 255; }
                c = hsv_to_rgb<uint8_t>(hue, sat, brt);
                c2 = hsv_to_rgb<uint8_t>(hue, sat, std::abs(brt-10));
                // Serial.printf("%3.0f%3.0f%3.0f (%3.0f%3.0f%3.0f) (%3.0f%3.0f%3.0f)\n", (float)(hue/655.35), (float)(sat/2.56), (float)(brt/2.56), 100*(float)((c >> 11) & 0x1f)/(float)0x1f, 100*(float)((c >> 5) & 0x3f)/(float)0x3f, 100*(float)(c & 0x1f)/(float)0x1f, 100*(float)((c2 >> 11) & 0x1f)/(float)0x1f, 100*(float)((c2 >> 5) & 0x3f)/(float)0x3f, 100*(float)(c2 & 0x1f)/(float)0x1f);
                for (int xo = -1; xo <= 1; xo += 2) {
                    sprite->drawCircle(point[HORZ] + vp->x, point[VERT] + vp->y, d * scaler, c);
                    sprite->drawCircle(point[HORZ] + vp->x, point[VERT] + vp->y + xo, d * scaler, c);
                    sprite->drawCircle(point[HORZ] + vp->x + xo, point[VERT] + vp->y, d * scaler, c);
                }
                for (int edge = -1; edge <= 1; edge += 2)
                    sprite->drawCircle(point[HORZ] + vp->x, point[VERT] + vp->y, d * scaler + edge, c2);
            }
            else if (rotate == Dots) {
                uint8_t sat = (30 * season) + rn(256 - 30 * season);
                int bigger = 1 + rn(3);
                for (int star = 0; star < 12; star++)
                    sprite->fillCircle(rn(vp->w) + vp->x, rn(vp->h) + vp->y, scaler * (2 + rn(1 + bigger * season)), hsv_to_rgb<uint8_t>((uint16_t)(spothue + (spothue >> 2) * rn(3)), sat, 130 + rn(126)));  // hue_to_rgb16(rn(255)), BLK);
            }
            else if (rotate == Boxes) {
                boxrad = 2 + rn(2 + 4 * season);
                boxminsize = 2 * boxrad + 5;
                int longer = rn(2);
                boxsize[longer] = boxminsize + rn(vp->w - boxminsize);
                boxsize[!longer] = boxminsize + rn(boxsize[longer] >> 2);  // cheesy attempt to work around crazy-values bug
                point[HORZ] = rn(vp->w) - (boxsize[HORZ] >> 1);
                point[VERT] = rn(vp->h) - (boxsize[VERT] >> 1);
                for (int axis=HORZ; axis<=VERT; axis++) {
                    if (point[axis] < 0) {
                        boxsize[axis] += point[axis];
                        point[axis] = -boxrad;
                    }
                }
                if (point[HORZ] + boxsize[HORZ] > vp->w) boxsize[HORZ] = (vp->w + boxrad - point[HORZ]);
                if (point[VERT] + boxsize[VERT] > vp->h) boxsize[VERT] = (vp->h + boxrad - point[VERT]);
                // std::cout << "px" << point[HORZ] << " py" << point[VERT] << " bx" << boxsize[HORZ] << " by" << boxsize[VERT] << "\n";
                sprite->fillSmoothRoundRect(point[HORZ] + vp->x, point[VERT] + vp->y, boxsize[HORZ], boxsize[VERT], boxrad, rando_color());  // Change colors as needed
            }
            else if (rotate == Ascii) {
                sprite->setTextSize(1);
                sprite->setTextDatum(textdatum_t::middle_center);
                sprite->setFont(&fonts::Font4);
                for (int star = 0; star < 4; star++) {
                    point[HORZ] = rn(vp->w);
                    point[VERT] = rn(vp->h);
                    uint16_t hue = point[VERT] * 65535 / vp->h;
                    uint8_t sat = point[HORZ] * 255 / vp->w;
                    String letter = (String)((char)(0x21 + rn(0x5d)));
                    uint8_t c = hsv_to_rgb<uint8_t>(hue, sat, 100 + 100 * (spothue < (32767 / (season+1))) + rn(56));
                    sprite->drawString(letter, point[HORZ]+1 + vp->x, point[VERT]+1 + vp->y);
                    sprite->drawString(letter, point[HORZ]-1 + vp->x, point[VERT]-1 + vp->y);
                    sprite->setTextColor(hsv_to_rgb<uint8_t>(hue, sat, 100 + 100 * (spothue > 32767) + rn(56)));
                    sprite->drawString(letter, point[HORZ] + vp->x, point[VERT] + vp->y);
                    sprite->setTextColor(BLK);  // allows subliminal messaging
                }
                sprite->setFont(&fonts::Font0);
            }
            else if (rotate == Worm) {
                has_eraser = saver_lotto = false;
                lucktimer.reset();
                // uint8_t sat = spothue >> 8;
                // uint8_t brt = 129 + 42 * (1 + std::abs(season-1));
                uint8_t c = (!wormstripe) ? BLK : pencolor;
                if (wormstripetimer.expired()) {
                    ++wormstripe %= 2;
                    wormstripetimer.set(300000 * ((!wormstripe) ? 1 : 4));
                }
                // if ((spothue >> 5) & 3 == 3) {
                //     if (season == 3) c = BLK;
                //     else if (season ==1) c = WHT;
                // }
                int wormposmax[2] = {(vp->w - wormd[HORZ]) / 2, (vp->h - wormd[VERT]) / 2};
                if (wormmovetimer.expireset()) {
                    for (int axis = HORZ; axis <= VERT; axis++) {
                        wormpos[axis] += (wormvel[axis] >> 6) * wormsign[axis];
                        if ((wormpos[axis] * wormsign[axis]) >> shifter >= wormposmax[axis] + 2) {
                            wormpos[axis] = (wormsign[axis] * wormposmax[axis]) << shifter;
                            wormsign[axis] *= -1;
                        }
                    }
                    // wormsat = constrain(wormsat + rn(3) - 1, 96, 255);
                }
                for (int axis = HORZ; axis <= VERT; axis++) {
                    if (!rn(3)) wormd[axis] = constrain(wormd[axis] + rn(3) - 1, wormdmin, wormdmax);
                }
                if (wormtimer.expireset()) {
                    for (int axis = HORZ; axis <= VERT; axis++) {
                        wormvel[axis] = constrain(wormvel[axis] + rn(255) - 127, 0, wormvelmax);
                        // if (wormvel[axis] == 0) wormsign[axis] = (rn(2) << 1) - 1;
                    }
                }
                // sprite->fillEllipse((vp->w / 2) + (wormpos[HORZ] >> shifter) + vp->x, (vp->h / 2) + (wormpos[VERT] >> shifter) + vp->y, wormd[HORZ] * scaler, wormd[VERT] * scaler, BLK);
                sprite->drawEllipse((vp->w / 2) + (wormpos[HORZ] >> shifter) + vp->x, (vp->h / 2) + (wormpos[VERT] >> shifter) + vp->y, wormd[HORZ] * scaler, wormd[VERT] * scaler, c);
                sprite->drawEllipse((vp->w / 2) + (wormpos[HORZ] >> shifter) + vp->x + 1, (vp->h / 2) + (wormpos[VERT] >> shifter) + vp->y, wormd[HORZ] * scaler, wormd[VERT] * scaler, c);
                sprite->drawEllipse((vp->w / 2) + (wormpos[HORZ] >> shifter) + vp->x - 1, (vp->h / 2) + (wormpos[VERT] >> shifter) + vp->y, wormd[HORZ] * scaler, wormd[VERT] * scaler, c);
                sprite->drawEllipse((vp->w / 2) + (wormpos[HORZ] >> shifter) + vp->x, (vp->h / 2) + (wormpos[VERT] >> shifter) + vp->y + 1, wormd[HORZ] * scaler, wormd[VERT] * scaler, c);
                sprite->drawEllipse((vp->w / 2) + (wormpos[HORZ] >> shifter) + vp->x, (vp->h / 2) + (wormpos[VERT] >> shifter) + vp->y - 1, wormd[HORZ] * scaler, wormd[VERT] * scaler, c);
            }
        }
        if ((cycle != 0) && has_eraser) {
            int erpos_max[2] = {(vp->w - eraser_rad) / 2, (vp->h - eraser_rad) / 2};
            for (int axis = HORZ; axis <= VERT; axis++) {
                erpos[axis] += eraser_velo[axis] * eraser_velo_sign[axis];
                if (erpos[axis] * eraser_velo_sign[axis] >= erpos_max[axis] + 5) {
                    erpos[axis] = eraser_velo_sign[axis] * erpos_max[axis];
                    eraser_velo[axis] = eraser_velo_min + rn(eraser_velo_max - eraser_velo_min);
                    eraser_velo[!axis] = eraser_velo_min + rn(eraser_velo_max - eraser_velo_min);
                    eraser_velo_sign[axis] *= -1;
                    eraser_rad = constrain((int)(eraser_rad + rn(5) - 2), eraser_rad_min, eraser_rad_max);
                }
            }
            sprite->fillCircle((vp->w / 2) + erpos[HORZ] + vp->x, (vp->h / 2) + erpos[VERT] + vp->y, eraser_rad * scaler, BLK);
        }
        if (lucktimer.expired())  {
            saver_lotto = !saver_lotto;
            lucktimer.set(3200000 + !saver_lotto * (5 + rn(350)) * 4000000);
        } 
        if (saver_lotto) {
            sprite->setTextDatum(textdatum_t::middle_center);
            sprite->setFont(&fonts::Font4);
            sprite->setTextColor(BLK);
            sprite->drawString("do drugs", vp->w / 2 + vp->x, vp->h / 2 + vp->y);
            sprite->setFont(&fonts::Font0);
            sprite->setTextDatum(textdatum_t::top_left);
        }
        for (int axis = HORZ; axis <= VERT; axis++) plast[axis] = point[axis];  // erlast[axis] = erpos[axis];
    }
    void change_pattern(int newpat = -1) {  // pass non-negative value for a specific pattern, or  -1 for cycle, -2 for random
        ++shapes_done %= 5;
        int last_pat = shape;
        has_eraser = !rn(2);
        if (0 <= newpat && newpat < NumSaverShapes) shape = newpat;  //
        else {
            if (newpat == -1) ++shape %= Worm;
            else if (newpat == -2) while (last_pat == shape) shape = rn(Rotate);
            if (rn(25) == 13) shape = Rotate;
        }
    }
};
#include <iostream>
#include <string>
#include <sstream>
#include <stdarg.h>
// #include "Org_01.h"
class EZReadConsole {  // never has any terminal solution been easier on the eyes
  public:
    bool dirty = true;
    static constexpr int num_lines = 24;
    // std::string textlines[num_lines];
    int usedlines = 0, last_drawn = -1, newest_content = -1, next_index = 0, linelength = 50, pix_margin = 2;
    int font_height = 6, highlighted_lines = 0;
    std::string textlines[num_lines], drawnow; // Ring buffer array
    uint8_t linecolors[num_lines], defaultcolor = MYEL, sadcolor = SALM, happycolor = LGRN, usecolor;    // std::vector<std::string> textlines; // Ring buffer array
    int bufferSize = num_lines; // size_t bufferSize; // Size of the ring buffer
  private:
    LGFX* mylcd;
    LGFX_Sprite* nowspr_ptr;
    viewport* vp;
    Timer messagetimer{1000000};  // messages delivered within this close of each other will get highlighted together
    // size_t nextIndex = 0; // Index for the next insertion
    // Function to retrieve the stored strings
    std::string getBufferElement(int index) { // size_t 
        if (index < bufferSize) {
            return textlines[index];
        } else {
            return ""; // Return empty string if index is out of range
        }
    }
    std::string remove_nonprintable(const std::string& victim) {
        std::string result;
        for (char ch : victim) {
            if (isprint(static_cast<unsigned char>(ch))) result += ch;
        }
        return result;
    }
    void dprintf_impl(uint8_t color, const char* format, va_list args) {  // this is not called directly but by one ots overloads below
        if (messagetimer.expireset()) highlighted_lines = next_index;  // mark the target index as 1st to be highlighted
        char temp[100]; // Assuming maximum length of output string
        vsnprintf(temp, sizeof(temp), format, args);
        textlines[next_index] = remove_nonprintable(std::string(temp)); // Store formatted output into buffer
        linecolors[next_index] = color;
        newest_content = next_index;
        ++next_index %= bufferSize; // Update next insertion index
        dirty = true;
    }
    int chars_to_fit_pix(LGFX_Sprite* spr, std::string& str, int pix) {
        int totalwidth = 0, charcount = 0;
        for (size_t i = 0; i < str.length(); ++i) {
            std::string ch(1, str[i]); // Create a string with the single character
            int charwidth = spr->textWidth(ch.c_str()); // Measure the width of the next character
            if (totalwidth + charwidth > pix) break; // Stop if adding the next character exceeds the maximum width
            totalwidth += charwidth;
            charcount++;
        }
        return charcount - 1;
    }
    void draw(LGFX_Sprite* spr) {
        spr->fillSprite(BLK);
        // int strsize = std::min((int)linelength, (int)textlines[nowindex].length());
        spr->setFont(&fonts::Font0);  // spr->setFont(&fonts::Org_01);
        spr->setTextDatum(textdatum_t::top_left);
        spr->setTextColor(linecolors[newest_content]);
        std::string nowline = textlines[newest_content];
        int chopit = chars_to_fit_pix(spr, nowline, vp->w);
        bool toobig = (chopit < nowline.length());
        if (toobig) {
            spr->setCursor(vp->x + pix_margin, vp->y + vp->h - 18);
            nowline = textlines[newest_content].substr(0, chopit);
            spr->print(nowline.c_str());
            nowline = textlines[newest_content].substr(chopit);
        }
        spr->setCursor(vp->x + pix_margin, vp->y + vp->h - 9);
        spr->print(nowline.c_str());
        int bottom_extent = vp->y + vp->h - 9 * (1 + (int)toobig);
        // spr->drawFastHLine(vp->x + 3, bottom_extent, vp->w - 6, LGRY);  // separator for the newest line at the bottom will be printed larger and span 2 lines
        spr->setFont(&fonts::TomThumb);
        for (int line=1; line<num_lines; line++) {
            int backindex = (newest_content + bufferSize - line) % bufferSize;
            spr->setCursor(vp->x + pix_margin, bottom_extent - line * (font_height + pix_margin));
            // if (nowindex >= highlighted_lines) spr->setTextColor(MYEL);
            int strsize = std::min((int)linelength, (int)textlines[backindex].length());
            spr->setTextColor(linecolors[backindex]);
            spr->print(textlines[backindex].c_str());
        }
        dirty = false;
    }
  public:
    void dprintf(const char* format, ...) {  // for if we're called with same arguments as printf would take
        va_list args;
        va_start(args, format);
        dprintf_impl(defaultcolor, format, args);  // Use default color
        va_end(args);
    }
    void dprintf(uint8_t color, const char* format, ...) {  // otherwise you can insert a custom color as the first argument
        va_list args;
        va_start(args, format);
        dprintf_impl(color, format, args);  // Use provided color
        va_end(args);
    }
    void setup(viewport* _vp) {
        vp = _vp;
        std::string blank = "";
        for (int i=0; i<num_lines; i++) {
            // linecolors[i] = MGRY;
            this->dprintf("%s", blank.c_str());
            linecolors[i] = MYEL;
        }
        linelength = (int)(vp->w / disp_font_width);
        dprintf("welcome to EZ-Read console");
        dirty = true;
    }
    void update(LGFX_Sprite* spr, bool force=false) {
        if (dirty || force) draw(spr);
        dirty = false;
    }
    EZReadConsole() {}
};
class PanelAppManager {
  private:
    enum saverchoices : int { Eraser, Collisions, NumSaverMenu, Blank };
    int nowsaver = Eraser, still_running = 0;
    LGFX* mylcd;
    LGFX_Sprite* nowspr_ptr;
    viewport vp;
    EraserSaver eSaver;
    CollisionsSaver cSaver;
    Simulator* sim;
    Touchscreen* touch;
    int touchp[2], dispfps;
    int corner[2], sprsize[2];
    Timer fps_timer, fps_timer2{250000};
    float myfps = 0.0;
    int oldfps = 0;
    int64_t fps_mark;
    bool simulating_last = false, mule_drawn = false, dirty = true;
    int ui_context_last = MuleChassisUI;
  public:
    EZReadConsole* ezread; // Initialize serial buffer with size 5
    std::uint32_t sec, psec, _width, _height, _myfps = 0, frame_count = 0;
    bool anim_reset_request = false;
    PanelAppManager(EZReadConsole* _ez) : ezread(_ez) {}
    void change_saver() {  // pass non-negative value for a specific pattern, -1 for cycle, -2 for random
        ++nowsaver %= NumSaverMenu;
        anim_reset_request = true;
    }
    void setup(LGFX* _lgfx, Simulator* _sim, Touchscreen* _touch, int _cornerx, int _cornery, int _sprwidth, int _sprheight) {
        Serial.printf("  panel app manager init ..");
        mylcd = _lgfx;
        sim = _sim;
        touch = _touch;
        set_vp(_cornerx, _cornery, _sprwidth, _sprheight);
        _width = vp.w << SHIFTSIZE;
        _height = vp.h << SHIFTSIZE;
        Serial.printf(" screensavers & diag console .. ");
        eSaver.setup(&framebuf[flip], &vp);
        cSaver.setup(&framebuf[flip], &vp);
        ezread->setup(&vp);
        Serial.printf("set up\n");
    }
    void reset() {
        if (nowsaver == Eraser) eSaver.reset(&framebuf[flip], &framebuf[!flip], &vp);
        else if (nowsaver == Collisions) cSaver.reset(&framebuf[flip], &framebuf[!flip], &vp);
        anim_reset_request = false;
    }
    void draw_simbutton(LGFX_Sprite* spr, int cntr_x, int cntr_y, int dir, uint8_t color) {
        if (dir == JOY_PLUS)  spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_plus_32x32x8, BLK);
        else if (dir == JOY_MINUS) spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_minus_32x32x8, BLK);
        else if (dir == JOY_UP) spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == JOY_DN) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 180, 1, 1, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == JOY_LT) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 270, 1, 1, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == JOY_RT) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 90, 1, 1, 32, 32, blue_up_32x32x8, BLK);
    }
    void draw_simbuttons (LGFX_Sprite* spr, bool create) {  // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
        if (!create) {
            spr->fillSprite(BLK);
            return;
        }
        spr->setTextDatum(textdatum_t::middle_center);
        bool do_draw;
        for (int32_t row = 0; row < arraysize(simgrid); row++) {
            for (int32_t col = 0; col < arraysize(simgrid[row]); col++) {
                do_draw = true;
                if ((simgrid[row][col].find("pos") != std::string::npos) && (!sim->can_sim(sens::brkpos) || sim->potmapping(sens::brkpos))) do_draw = false;
                if ((simgrid[row][col].find("psi") != std::string::npos) && (!sim->can_sim(sens::pressure) || sim->potmapping(sens::pressure))) do_draw = false;
                if ((simgrid[row][col].find("mph") != std::string::npos) && (!sim->can_sim(sens::speedo) || sim->potmapping(sens::speedo))) do_draw = false;
                if ((simgrid[row][col].find("rpm") != std::string::npos) && (!sim->can_sim(sens::tach) || sim->potmapping(sens::tach))) do_draw = false;
                if (simgrid[row][col].find("joy") != std::string::npos) {
                    if (sim->potmapping(sens::joy) && (col == 2)) do_draw = false;
                    if (!sim->can_sim(sens::joy)) do_draw = false;
                }
                if (do_draw) {
                    int32_t cntr_x = touch_cell_h_pix*col + (touch_cell_h_pix>>1) + 2 + vp.x - 5 + 2;
                    int32_t cntr_y = touch_cell_v_pix*row + (touch_cell_v_pix>>1) + vp.y - 1;
                    if (simgrid[row][col] != "    ") {
                        draw_simbutton(spr, cntr_x + 2, cntr_y - 1, simgriddir[row][col], YEL);  // for 3d look
                        draw_simbutton(spr, cntr_x, cntr_y, simgriddir[row][col], DGRY);
                        // spr->fillRoundRect(cntr_x - 20, cntr_y - touch_cell_v_pix/2 - 10, 40, 20, 5, DGRY);
                        // spr->drawRoundRect(cntr_x - 20, cntr_y - touch_cell_v_pix/2 - 10, 40, 20, 5, BLK);
                        if ((row % 2) && (simgrid[row][col].find("joy") == std::string::npos)) {
                            spr->setFont(&fonts::FreeSans9pt7b);
                            spr->setTextColor(BLK);
                            spr->drawString(simgrid[row][col].c_str(), cntr_x - 1, cntr_y - touch_cell_v_pix/2 + 5 - 1);
                            spr->drawString(simgrid[row][col].c_str(), cntr_x + 1, cntr_y - touch_cell_v_pix/2 + 5 + 1);
                            spr->setTextColor(LYEL);
                            spr->drawString(simgrid[row][col].c_str(), cntr_x, cntr_y - touch_cell_v_pix/2 + 5);
                        }
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
    void display_fps(LGFX_Sprite* spr) {
        if (auto_saver_enabled && autosaver_display_fps) {
            if (fps_timer2.expireset()) dispfps = (int)myfps;
            spr->setFont(&fonts::Font0);
            spr->setTextDatum(textdatum_t::top_left);
            spr->setTextColor(BLK);
            spr->setCursor(9, 9);
            spr->print(std::to_string(oldfps).c_str());
            spr->setCursor(11, 11);
            spr->print(std::to_string(oldfps).c_str());
            spr->setCursor(10, 10);
            spr->print(std::to_string(oldfps).c_str());
            // for (int x=9; x<=11; x++) {
            //     spr->setCursor(x, x);
            //     spr->print(std::to_string(oldfps).c_str());
            // }
            spr->setTextColor(LGRY);
            spr->setCursor(10, 10);
            spr->print(std::to_string(dispfps).c_str());
            oldfps = dispfps;
        }
    }
    float update(LGFX_Sprite* spr, bool argdirty=false) {
        if ((ui_context_last != ScreensaverUI) && (ui_context == ScreensaverUI)) change_saver();  // ptrsaver->reset();
        if (ui_context_last != ui_context) dirty = true;
        if (argdirty) dirty = true;
        ui_context_last = ui_context;
        if (anim_reset_request) reset();
        spr->setClipRect(vp.x, vp.y, vp.w, vp.h);
        if (dirty) {
            spr->fillSprite(BLK);
            mule_drawn = false;
            ezread->dirty = true;
        }
        if (ui_context == ScreensaverUI) {  // With timer == 16666 drawing dots, avg=8k, peak=17k.  balls, avg 2.7k, peak 9k after 20sec
            // mule_drawn = false;  // With max refresh drawing dots, avg=14k, peak=28k.  balls, avg 6k, peak 8k after 20sec
            if (nowsaver == Eraser) {
                still_running = eSaver.update(spr, &vp);
                if (touched()) eSaver.saver_touch(spr, touch_pt(HORZ), touch_pt(VERT));
                display_fps(spr);
            }
            else if (nowsaver == Collisions) {
                if (touched()) cSaver.saver_touch(spr, touch_pt(HORZ), touch_pt(VERT));
                still_running = cSaver.update(spr, &vp);
                display_fps(spr);
            }
            if (!still_running) change_saver();
        }
        else if (ui_context == EZReadUI) {
            ezread->update(spr);
        }
        else if (ui_context == MuleChassisUI) {
            if (!mule_drawn) {
                spr->fillSprite(BLK);
                spr->pushImageRotateZoom(85 + vp.x, 85 + vp.y, 82, 37, 0, 1, 1, 145, 74, mulechassis_145x74x8, BLK);
                mule_drawn = true;
            }
        }
        spr->clearClipRect();
        if (sim->enabled()) {
            draw_simbuttons(spr, sim->enabled());  // if we just entered simulator draw the simulator buttons, or if we just left erase them
        }
        else if (simulating_last) {
            spr->fillRect(vp.x, vp.y, vp.w, vp.h, BLK);
            dirty = true;
            mule_drawn = false;
        }
        simulating_last = sim->enabled();
        dirty = false;
        calc_fps();
        return myfps;
    }
    void set_vp(int _cornerx, int _cornery, int _sprwidth, int _sprheight) {
        vp.x = _cornerx;
        vp.y = _cornery;
        vp.w = _sprwidth;
        vp.h = _sprheight;
    }
    bool touched() {
        if (touch->touched()) {
            touchp[HORZ] = touch->touch_pt(HORZ) - vp.x;
            touchp[VERT] = touch->touch_pt(VERT) - vp.y;
            return true;
        }
        return false;
    }
    int touch_pt(int axis) {
        return touchp[axis];
    }
};
#ifdef CONVERT_IMAGE 
#define IMAGE_ARRAY mulechassis_145x74 
#define IMAGE_WIDTH 145
void convert_565_to_332_image() {
    Serial.printf("const uint8_t %sx8[%ld] PROGMEM = {\n\t", String(IMAGE_ARRAY).c_str(), arraysize(IMAGE_ARRAY));
    for (int i=0; i<arraysize(IMAGE_ARRAY); i++) {
        Serial.printf("0x%02x, ", color_16b_to_8b(IMAGE_ARRAY[i]));
        if (!(i % IMAGE_WIDTH)) Serial.printf("  // pixel# %ld\n%s", pixcount, (i < arraysize(IMAGE_ARRAY) - 1) ? "\t" : "");
    }
    Serial.printf("};\n");
}
#endif