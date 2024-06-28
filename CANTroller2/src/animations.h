#pragma once
#include <Arduino.h>
#include "tftsetup.h"
#include "inputs.h"
static Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
static MomentaryButton bootbutton(boot_sw_pin, false);

// #define touch_simbutton 38
#define disp_apppanel_x 150
#define disp_apppanel_y 48
#define disp_apppanel_w (disp_width_pix - disp_apppanel_x)  // 156
#define disp_apppanel_h (disp_height_pix - disp_apppanel_y)  // 192
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
volatile int _loop_count;
static constexpr int SHIFTSIZE = 8;
volatile bool flip = 0;
volatile int refresh_limit = 11111; // 16666; // = 60 Hz,   11111 = 90 Hz
// volatile bool auto_saver_enabled = false;
volatile int screen_refresh_time;
// Timer screenRefreshTimer = Timer((int64_t)refresh_limit);
LGFX lcd;
static constexpr int num_bufs = 2;
LGFX_Sprite framebuf[num_bufs];  // , datapage_sp[2], bargraph_sp[2], idiots_sp[2];
struct viewport {
    int x;
    int y;
    int w;
    int h;
};
class CollisionsSaver {
  public:
    struct ball_info_t {
        int x;
        int y;
        int dx;
        int dy;
        int r;
        int m;
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
    static constexpr int BALL_MAX = 35;  // 256
    ball_info_t _balls[2][BALL_MAX];
    int _ball_count = 0, _myfps = 0;
    int ball_thismax, ball_count = 0;
    int _width, _height, touchx, touchy, lastx, lasty;
    int sec, psec, ball_create_rate = 3200;
    int myfps = 0, frame_count = 0;
    float ball_radius_base = 4.5 / 235.0;  // 7 pixels radius / 125x100 sprite = about 5 pix per 235 sides sum
    float ball_radius_modifier = 2.6 / 235.0;  // 4 pixels radius / 125x100 sprite = about 3 pix per...
    uint8_t ball_redoubler_rate = 0x18;  // originally 0x07
    int8_t ball_gravity_x = 0, ball_gravity_y = 16;  // ball_gravity = 16;  // originally 0 with suggestion of 4
    volatile bool _is_running;
    volatile int _loop_count = 0;
    Timer gravtimer;
    CollisionsSaver() {}
  private:
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
        for (int i = 0; i < _ball_count; i++) {
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
        for (int i=0; i<2; i++) if (!(bool)rn(ball_redoubler_rate)) sqrme *=2;
        a->r = sqrme << SHIFTSIZE;  // (sqrme * sqrme)));
        a->m = 4 + (ball_count & 0x07);
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
        int rr, len, vx2vy2;
        float vx, vy, distance, t;
        size_t f = _loop_count & 1;
        balls = a = &_balls[f][0];
        b = &_balls[!f][0];
        memcpy(a, b, sizeof(ball_info_t) * ball_count);
        for (int i = 0; i < ball_count + touchnow; i++) {
            a = (i == ball_count) ? &touchball : &balls[i];
            if (i < ball_count) {
                a->dx += ball_gravity_x; // gravity horz
                a->dy += ball_gravity_y; // gravity vert
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
  public:
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
        for (int i = 0; i < ball_count; ++i) new_ball(i);
        // screenRefreshTimer.set(refresh_limit);
        _is_running = true;
    }
    void meandering_gravity() {
        if (gravtimer.expired()) {
            float radius = 10.0, cosinc = 0.985, sininc = 0.174; // precomputed trig values  // angleinc = 10.0 * M_PI / 180.0; // Increment in radians
            static float x = 0, y = radius, mag = 0.5;
            mag = constrain((float)(mag + 0.01 * (float)(rn(70) - 35)), 0.0f, 1.0f);
            x = x * cosinc - y * sininc; // apply rotation and get new coordinates
            y = x * sininc + y * cosinc;
            ball_gravity_x = (int)(x * radius * mag);  // ball_gravity_x = constrain((ball_gravity_x + rn(6) - 3), -18, 28);
            ball_gravity_y = (int)(y * radius * mag);  // ball_gravity_y = constrain((ball_gravity_y + rn(6) - 3), -18, 28);
            gravtimer.set(500000 * (3 + rn(3)));
        }
    }
    int update(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        bool round_over = mainfunc();
        meandering_gravity();
        drawfunc();
        return !round_over;  // not done yet
    }
};
class EraserSaver {  // draws colorful patterns to exercise
 private:
    enum savershapes { Wedges, Dots, Rings, Ellipses, Boxes, Ascii, Worm, Rotate, NumSaverShapes };
    LGFX_Sprite* sprite;
    viewport* vp;
    int wormpos[2] = {0, 0}, wormvel[2] = {0, 0}, wormsign[2] = {1, 1}, wormd[2] = {20, 20};
    int shifter = 2, wormdmin = 8, wormdmax = 38, wormvelmax = 400, wormsat = 128, boxsize[2], mindot = 4, adddot = 4;
    int sprsize[2], rotate = -1, scaler = 1, season = rn(4), last_season = 0, precession = rn(10), last_precession = 3, numseasons = 4;
    int point[2], plast[2], er[2], erpos_max[2], wormstripe = 2;
    int eraser_rad = 14, eraser_rad_min = 22, eraser_rad_max = 40, eraser_velo_min = 3, eraser_velo_max = 7, touch_w_last = 2;
    int erpos[2] = {0, 0}, eraser_velo_sign[2] = {1, 1}, now = 0;
    int eraser_velo[2] = {rn(eraser_velo_max), rn(eraser_velo_max)}, shapes_per_run = 5, shapes_done = 0;
    uint8_t wclast, pencolor = RED;
    float pensat = 200.0;
    uint16_t spothue = 65535, penhue = rn(65535);
    int spotrate = 300, huebase = 0;
    int num_cycles = 3, cycle = 0, boxrad, boxminsize, boxmaxarea = 200, shape = rn(Rotate), pensatdir = 1;
    static constexpr int saver_cycletime_us = 35000000;
    Timer saverCycleTimer, pentimer{70000}, lucktimer, seasontimer, spottimer{2000000};
    Timer wormmovetimer{20000}, wormtimer{1000000}, extraeffectstimer{2850000};
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
        for (int i = 0; i <= 1; i++) {
            spp[i]->setTextSize(1);
            spp[i]->setTextDatum(textdatum_t::middle_center);
            spp[i]->setTextColor(BLK);
            spp[i]->setCursor(vp->w / 2 + vp->x, vp->h / 2 + vp->y);
        }
        change_pattern(-2);  // randomize new pattern whenever turned off and on
        saverCycleTimer.set(saver_cycletime_us);
        seasontimer.set(3000000);
        scaler = std::max(1, (vp->w + vp->h)/200);
        erpos_max[HORZ] = (int)vp->w / 2 - eraser_rad;
        erpos_max[VERT] = (int)vp->h / 2 - eraser_rad;
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
            last_season = season;
            ++season %= numseasons;
            seasontimer.set(3200000 * (1 + rn(4)));
            last_precession = precession;
            precession += season;
            precession %= 10;
            Serial.printf("p:%d s:%d\n", precession, season);
        }
        drawsprite();
        return shapes_done;
    }
    void change_pattern(int newpat = -1) {  // pass non-negative value for a specific pattern, or  -1 for cycle, -2 for random, -3 for cycle backwards  // XX , -4 for autocycle (retains saver timeout)
        ++shapes_done %= 7;
        int last_pat = shape;
        has_eraser = !(bool)rn(3);
        if (0 <= newpat && newpat < NumSaverShapes) shape = newpat;  //
        else {
            if (newpat == -1) ++shape %= Rotate;
            if (newpat == -3) shape = (shape - 1 + Rotate) % Rotate;
            else if (newpat == -2) while (last_pat == shape) shape = rn(Rotate);
            if (rn(25) == 13) shape = Rotate;
        }
    }
  private:
    void update_pen() {
        if (cycle != 2) {
            if (pentimer.expireset()) {
                if (season == 1 && season == 3) {
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
                else penhue += 500;
                if (!(bool)rn(30)) penhue = rn(65536);
                pencolor = hsv_to_rgb<uint8_t>(penhue, (uint8_t)pensat, 200 + rn(56));
            }
        }
        spothue = (uint16_t)(spothue + (spotrate >> 2) % sizeof(spothue));
        if (!(bool)rn(20)) spothue = rn(65535);
    }
    void run_wedges() {
        uint8_t wcball, wctip;
        uint16_t hue = rn(65536);
        uint8_t brt = 156 + rn(100);
        if (season == 1) {
            wctip = hsv_to_rgb<uint8_t>(hue, 0, brt);
            wcball = hsv_to_rgb<uint8_t>(hue, 64, brt);
        }
        else if (season == 3) {
            wctip = hsv_to_rgb<uint8_t>(hue, rn(256), brt);
            wcball = hsv_to_rgb<uint8_t>(hue, 64, rn(64));
        }
        else {
            wcball = hsv_to_rgb<uint8_t>(hue, 127 + (spothue >> (season + 5)), 200 + rn(56));
            wctip = wclast;
        }
        float im = 0;
        if (plast[VERT] != point[VERT]) im = (float)(plast[HORZ] - point[HORZ]) / (float)(plast[VERT] - point[VERT]);
        sprite->fillCircle(plast[HORZ] + vp->x, plast[VERT] + vp->y, 3, wcball);
        for (int h=-4; h<=4; h++)
            sprite->drawGradientLine(point[HORZ] + vp->x, point[VERT] + vp->y, plast[HORZ] + vp->x + (int)(h / ((std::abs(im) > 1.0) ? im : 1)), plast[VERT] + vp->y + (int)(h * ((std::abs(im) > 1.0) ? 1 : im)), wctip, wcball);
        wclast = wcball;
    }
    void run_ellipses() {
        int d[2] = {10 + rn(30), 10 + rn(30)};  //  this crashes after 10 seconds:  int d[2] = { 5 + rn(20 * (int)(season / 2)), 5 + rn(20 * (int(season / 2))) };  // 
        uint8_t sat, brt;
        uint16_t mult = rn(2000), hue = spothue + rn(3000);
        spotrate = (int)((season * 200) + rn(200));
        sat = 100 + rn(156);
        brt = 120 + rn(136);
        for (int i = 0; i < 6 + rn(20); i++) {
            uint8_t c = hsv_to_rgb<uint8_t>(hue + mult * i, sat, brt);
            sprite->drawEllipse(point[HORZ] + vp->x, point[VERT] + vp->y, scaler * d[0] - i, scaler * d[1] + i, c);
        }
    }
    void run_rings() {
        if (extraeffectstimer.expired()) {
            spotrate = (int)(200 + rn(800));
            extraeffectstimer.set(500000 * (1 + rn(4)));
        }
        int d = 3 + rn(14 + (precession - 4) * 2);
        uint8_t sat, brt, c, c2;
        uint16_t hue;
        uint16_t hue_common = hue = spothue + 32768 * rn(2) + rn(1500);
        if (season == 0) {
            hue = hue_common;
            sat = (!rn(4)) ? 200 + rn(50) : 75 + 100 * rn(2);
            brt = 150 + rn(56);
        }
        else if (season == 1) {
            hue = (penhue + rn(2) * 32781) % 65563;
            sat = pensat + rn(100) - 100;
            brt = 150 + rn(106);
        }
        else if (season == 2) {
            hue = hue_common;
            sat = 255 - (uint8_t)(spothue >> (6 + rn(3) + season));
            brt = 175 + rn(80);
        }
        else if (season == 3) {
            hue = hue_common;
            sat = 175 + 75 * rn(2);
            brt = 200 + rn(56);
        }
        c = hsv_to_rgb<uint8_t>(hue, sat, brt);
        if (!(precession % 3)) c2 = c;  // hsv_to_rgb<uint8_t>(hue, sat, std::abs(brt-10));
        else if (!(precession % 2)) c2 = hsv_to_rgb<uint8_t>(hue, sat, std::abs(brt-10));
        else c2 = BLK;  // 
        // Serial.printf("%3.0f%3.0f%3.0f (%3.0f%3.0f%3.0f) (%3.0f%3.0f%3.0f)\n", (float)(hue/655.35), (float)(sat/2.56), (float)(brt/2.56), 100*(float)((c >> 11) & 0x1f)/(float)0x1f, 100*(float)((c >> 5) & 0x3f)/(float)0x3f, 100*(float)(c & 0x1f)/(float)0x1f, 100*(float)((c2 >> 11) & 0x1f)/(float)0x1f, 100*(float)((c2 >> 5) & 0x3f)/(float)0x3f, 100*(float)(c2 & 0x1f)/(float)0x1f);
        for (int xo = -1; xo <= 1; xo += 2) {
            sprite->drawCircle(point[HORZ] + vp->x, point[VERT] + vp->y, d * scaler, c);
            sprite->drawCircle(point[HORZ] + vp->x, point[VERT] + vp->y + xo, d * scaler, c);
            sprite->drawCircle(point[HORZ] + vp->x + xo, point[VERT] + vp->y, d * scaler, c);
        }
        for (int edge = -1; edge <= 1; edge += 2)
            sprite->drawCircle(point[HORZ] + vp->x, point[VERT] + vp->y, d * scaler + edge, c2);
    }
    void run_dots() {
        static int punches_left;
        static bool punchdelay;
        static bool was_eraser;
        spotrate = (int)(rn(900));
        uint8_t c, sat = (30 * season) + rn(256 - 30 * season);
        static bool oldproc;
        if (precession < oldproc) {  // on leap year we slam them with a few bigger punches
            was_eraser = has_eraser;
            has_eraser = false;
            punchdelay = true;
            punches_left = 8;
        }
        else if (!punchdelay) punches_left = 0;
        oldproc = precession;
        if (season != last_season) {
            mindot = constrain(mindot + rn(4) - 2, 1, 8);
            adddot = constrain(adddot + rn(4) - 2, 2, 11 - mindot);
        }
        int r, myshape = rn(season + precession);
        if (myshape < 2) myshape = 0;
        else if (myshape < 6) myshape = 1;
        else myshape = 2; 
        // int squarechance = constrain(precession - 2, 0, 5);
        // if (squarechance == 5) squarechance = 1;
        // else if (squarechance) squarechance = (int)(rn(squarechance) > 1);
        // for upright equilateral triangle with origin at 0,0 and peak at 0,1 the bottom right corner is at sqrt(3)/3,-1/3
        float rt3over3 = 0.6;
        int stars = 13 - (adddot / 2) - (mindot + adddot > 8) ? 3 : 0;
        if (punches_left > 0) {
            if (extraeffectstimer.expired()) {
                extraeffectstimer.set(30000 * (9 - punches_left--));
                r = scaler * (25 + rn(15));
                int p[2] = { rn(vp->w) + vp->x, rn(vp->h) + vp->y };
                c = hsv_to_rgb<uint8_t>((uint16_t)(spothue + (spothue >> 2) * rn(3)), sat, 130 + rn(126));
                if (myshape == 0) sprite->fillCircle(p[0], p[1], r, c);  // hue_to_rgb16(rn(255)), BLK);
                else if (myshape == 1) sprite->fillRect(p[0] - r/2, p[1] - r/2, r * 2, r * 2, c); 
                else {
                    int pt2[2] = { p[0] - (int)(rt3over3 * (float)r), p[1] - int(0.33 * (float)r) };
                    int pt3[2] = { p[0] + (int)(rt3over3 * (float)r), pt2[1] };                    
                    sprite->fillTriangle(p[0], p[1] - r, pt2[0], pt2[1], pt3[0], pt3[1], c);
                }
                if (punches_left <= 0) {
                    has_eraser = was_eraser;
                    punchdelay = false;
                }
            }
        }
        else for (int star = 0; star < stars; star++) {
            r = scaler * (mindot + rn(adddot));
            int p[2] = { rn(vp->w) + vp->x, rn(vp->h) + vp->y };
            c = hsv_to_rgb<uint8_t>((uint16_t)(spothue + (spothue >> 2) * rn(3)), sat, 130 + rn(126));
            if (myshape == 0) sprite->fillCircle(p[0], p[1], r, c);
            else if (myshape == 1) sprite->fillRect(p[0] - r/2, p[1] - r/2, r * 2, r * 2, c);
            else {
                int pt2[2] = { p[0] - (int)(rt3over3 * (float)r), p[1] - int(0.33 * (float)r) };
                int pt3[2] = { p[0] + (int)(rt3over3 * (float)r), pt2[1] };                    
                sprite->fillTriangle(p[0], p[1] - r, pt2[0], pt2[1], pt3[0], pt3[1], c);
            }
        }
    }
    void run_boxes() {
        uint8_t boxcolor;
        boxrad = rn(1 + rn(2) * season);  // note this will crash us!  ->  boxrad = rn(5 * season);
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
        int shells = 1 + (!(bool)rn(5)) ? 1 + rn(4) : 0;
        int steps[2] = { boxsize[HORZ] / (shells+1), boxsize[VERT] / (shells+1) };
        for (int mat=0; mat<shells; mat++) {
            if (season == 0) boxcolor = rando_color();
            else if (season == 1) boxcolor = hsv_to_rgb<uint8_t>((uint16_t)rn(65535), rn(256), rn(256));
            else if (season == 2) boxcolor = hsv_to_rgb<uint8_t>((uint16_t)((spothue + rn(2) * 32767 + rn(512)) % 65535), 100 + rn(106), 255);
            else if (season == 3) boxcolor = hsv_to_rgb<uint8_t>((uint16_t)((spothue + rn(1024)) % 65535), 150 + rn(56), 255);
            for (int axis=HORZ; axis<=VERT; axis++) {
                boxsize[axis] -= steps[axis];
                point[axis] += (steps[axis] >> 1);
            }
            sprite->fillSmoothRoundRect(point[HORZ] + vp->x, point[VERT] + vp->y, boxsize[HORZ], boxsize[VERT], boxrad, boxcolor);  // Change colors as needed
        }
    }
    void run_ascii() {
        static float offset[2];
        static int final[2];
        uint16_t hue;
        uint8_t sat;
        sprite->setTextSize(1);
        sprite->setTextDatum(textdatum_t::middle_center);
        sprite->setFont(&fonts::Font4);
        for (int star = 0; star < 4; star++) {
            point[HORZ] = rn(vp->w);
            point[VERT] = rn(vp->h);
            if (precession > 4) {
                hue = (vp->h * (int)(season > 1)) - point[VERT] * 65535 / vp->h;
                sat = point[HORZ] * -255 / vp->w;
            }
            else {
                hue = (vp->w * (int)(season == 0 || season == 2)) - point[HORZ] * 65535 / vp->w;
                sat = point[VERT] * -255 / vp->h;
            }
            for (int axis=HORZ; axis <= VERT; axis++) offset[axis] += (float)rn(100) / 100;
            final[HORZ] = (point[HORZ] + ((season < 3) ? (int)(offset[HORZ]) : 0)) % vp->w + vp->x;
            final[VERT] = (point[VERT] + ((season > 0) ? (int)(offset[VERT]) : 0)) % vp->h + vp->y;
            std::string letter = std::string(1, static_cast<char>(0x21 + rn(0x5d)));
            uint8_t c = hsv_to_rgb<uint8_t>(hue, sat, 150 + 50 * (spothue < (32767 / (season+1))) + rn(56));
            sprite->setTextColor(BLK);  // allows subliminal messaging
            sprite->drawString(letter.c_str(), final[HORZ] + 1, final[VERT] + 1);  // these will not work at extreme sides
            sprite->drawString(letter.c_str(), final[HORZ] - 1, final[VERT] - 1);  // these will not work at extreme sides
            sprite->setTextColor(hsv_to_rgb<uint8_t>(hue, sat, 150 + 50 * (spothue > 32767) + rn(56)));
            sprite->drawString(letter.c_str(), final[HORZ], final[VERT]);
        }
        sprite->setTextColor(BLK);  // allows subliminal messaging
        sprite->setFont(&fonts::Font0);
    }
    void run_worm() {
        int point[2];
        has_eraser = saver_lotto = false;
        lucktimer.reset();
        uint8_t sat;
        if (season == 0 || season == 2) sat = pensat;
        else sat = 105 + (int)(150.0 * (float)(((precession > 5) ? 10 - precession : precession)) / 5.0); 
        uint8_t c = (!wormstripe) ? BLK : hsv_to_rgb<uint8_t>(penhue, sat, 200 + rn(56));;
        if (extraeffectstimer.expired()) {
            ++wormstripe %= 2;
            extraeffectstimer.set(300000 * ((!wormstripe) ? 1 : 4));
        }
        int wormposmax[2] = {(vp->w - wormd[HORZ]) / 2, (vp->h - wormd[VERT]) / 2};
        if (wormmovetimer.expireset()) {
            for (int axis = HORZ; axis <= VERT; axis++) {
                wormpos[axis] += (wormvel[axis] >> 6) * wormsign[axis];
                if ((wormpos[axis] * wormsign[axis]) >> shifter >= wormposmax[axis] + 2) {
                    wormpos[axis] = (wormsign[axis] * wormposmax[axis]) << shifter;
                    wormsign[axis] *= -1;
                    if (wormvel[1-axis] < (1 << 6)) wormvel[1-axis] = wormvel[1-axis] << 1;
                }
            }
        }
        for (int axis = HORZ; axis <= VERT; axis++) {
            if (!(bool)rn(3)) wormd[axis] = constrain(wormd[axis] + rn(3) - 1, wormdmin, wormdmax);
        }
        if (wormtimer.expireset()) {
            for (int axis = HORZ; axis <= VERT; axis++) {
                wormvel[axis] = constrain(wormvel[axis] + rn(255) - 127, 0, wormvelmax);
            }
        }
        for (int xo1 = -1; xo1 <= 1; xo1 += 2) {
            point[HORZ] = (vp->w / 2) + (wormpos[HORZ] >> shifter) + vp->x + xo1;
            point[VERT] = (vp->h / 2) + (wormpos[VERT] >> shifter) + vp->y + xo1;
            for (int xo2 = -1; xo2 <= 1; xo2 += 2) {
                sprite->drawEllipse(point[HORZ], point[VERT], wormd[HORZ] * scaler, wormd[VERT] * scaler, c);
                sprite->drawEllipse(point[HORZ] + xo2, point[VERT] + xo2, wormd[HORZ] * scaler, wormd[VERT] * scaler, c);
                sprite->drawEllipse(point[HORZ] + xo2, point[VERT], wormd[HORZ] * scaler, wormd[VERT] * scaler, c);
            }
        }
    }
    void the_eraser() {
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
            lucktimer.set(1900000 + !saver_lotto * (200 + 3 * rn(255)) * 4000000);
        } 
        if (saver_lotto) {
            sprite->setTextDatum(textdatum_t::middle_center);
            sprite->setFont(&fonts::Font4);
            sprite->setTextColor(BLK);
            sprite->drawString("\x64\x6f\x20\x64\x72\x75\x67\x73", vp->w / 2 + vp->x, vp->h / 2 + vp->y);
            sprite->setFont(&fonts::Font0);
            sprite->setTextDatum(textdatum_t::top_left);
        }
    }
    void drawsprite() {
        point[HORZ] = rn(vp->w);
        point[VERT] = rn(vp->h);
        if (shape == Rotate) ++rotate %= NumSaverShapes;
        else rotate = shape;
        update_pen();
        if (rotate == Wedges) run_wedges();
        else if (rotate == Ellipses) run_ellipses();
        else if (rotate == Rings) run_rings();
        else if (rotate == Dots) run_dots();
        else if (rotate == Boxes) run_boxes();
        else if (rotate == Ascii) run_ascii();
        else if (rotate == Worm) run_worm();
        the_eraser();
        for (int axis = HORZ; axis <= VERT; axis++) plast[axis] = point[axis];  // erlast[axis] = erpos[axis];
    }
};
class EZReadDrawer {  // never has any terminal solution been easier on the eyes
  public:
    bool dirty = true;
    std::string drawnow; // Ring buffer array
  private:
    int _main_x, scrollbar_width = 3, pix_margin = 2;
    int font_height = 6, linelength;
    LGFX* mylcd;
    // LGFX_Sprite* spr;
    LGFX_Sprite* nowspr_ptr;
    viewport* vp;
    EZReadConsole* ez;
    int chars_to_fit_pix(LGFX_Sprite* spr, std::string& str, int pix) {
        int totalwidth = 0, charcount = 0;
        for (size_t i = 0; i < str.length(); ++i) {
            std::string ch(1, str[i]); // Create a string with the single character
            int charwidth = spr->textWidth(ch.c_str()); // Measure the width of the next character
            if (totalwidth + charwidth > pix) break; // Stop if adding the next character exceeds the maximum width
            totalwidth += charwidth;
            charcount++;
        }
        return charcount;
    }
    void draw_scrollbar(LGFX_Sprite* spr, uint8_t color) {
        int cent = (int)((float)vp->h * 0.125) - 6;
        for (int i=0; i<3; i++) spr->drawFastVLine(vp->x + i, cent + 12 - (i + 1) * 4, (i + 1) * 4, color);
        cent = (int)((float)vp->h * 0.375) - 6;
        for (int arrow=0; arrow<3; arrow++) {
            int my_y = cent + arrow * 5 - 4;    
            for (int i=0; i<3; i++) spr->drawFastVLine(vp->x + i, my_y + i, i + 1, color);
        }
        cent = (int)((float)vp->h * 0.625) - 6;
        for (int arrow=0; arrow<3; arrow++) {
            int my_y = cent + arrow * 5 - 4;    
            for (int i=0; i<3; i++) spr->drawFastVLine(vp->x + i, my_y, i + 1, color);
        }
        // int cent = vp->h / 8 - 6;
        // for (int i=0; i<3; i++) spr->drawFastVLine(vp->x + i, cent + 12 - (i + 1) * 4, (i + 1) * 4, color);
    }
    void draw(LGFX_Sprite* spr) {
        draw_scrollbar(spr, LGRY);
        int botline = (ez->current_index - ez->offset - (int)ez->textlines[ez->current_index].empty() + ez->bufferSize) % ez->bufferSize;
        spr->fillSprite(BLK);
        spr->setTextWrap(false);        // 右端到達時のカーソル折り返しを禁止
        // int strsize = std::min((int)linelength, (int)textlines[nowindex].length());
        spr->setFont(&fonts::Font0);  // spr->setFont(&fonts::Org_01);
        spr->setTextDatum(textdatum_t::top_left);
        spr->setTextColor(ez->linecolors[botline]);
        std::string nowline = ez->textlines[botline];
        int chopit = chars_to_fit_pix(spr, nowline, vp->w);
        bool toobig = (chopit < nowline.length());
        if (toobig) {
            spr->setCursor(_main_x, vp->y + vp->h - 18);
            nowline = ez->textlines[botline].substr(0, chopit);
            spr->print(nowline.c_str());
            nowline = ez->textlines[botline].substr(chopit);
        }
        spr->setCursor(_main_x, vp->y + vp->h - 9);
        spr->print(nowline.c_str());
        int bottom_extent = vp->y + vp->h - 9 * (1 + (int)toobig);
        // spr->drawFastHLine(vp->x + 3, bottom_extent, vp->w - 6, LGRY);  // separator for the newest line at the bottom will be printed larger and span 2 lines
        spr->setFont(&fonts::TomThumb);
        for (int line=1; line<ez->num_lines; line++) {
            int backindex = (botline + ez->bufferSize - line) % ez->bufferSize;
            spr->setCursor(_main_x, bottom_extent - line * (font_height + pix_margin));
            // if (nowindex >= highlighted_lines) spr->setTextColor(MYEL);
            int strsize = std::min((int)linelength, (int)ez->textlines[backindex].length());
            spr->setTextColor(ez->linecolors[backindex]);
            spr->print(ez->textlines[backindex].c_str());
        }
        dirty = false;
    }
  public:
    void setup(viewport* _vp) {
        ez->setup();
        vp = _vp;
        _main_x = _vp->x + scrollbar_width + pix_margin;
        linelength = (int)(vp->w / disp_font_width);
    }
    void update(LGFX_Sprite* spr, bool force=false) {
        if (ez->offsettimer.expired()) {
            ez->offset = 0;
            dirty = true;
        }
        if (dirty || force || ez->dirty) draw(spr);
        dirty = ez->dirty = false;
    }
    EZReadDrawer(EZReadConsole* _ez) : ez(_ez) {}
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
    EZReadDrawer* ezdraw;
    int touchp[2], dispfps;
    int corner[2], sprsize[2];
    Timer fps_timer, fps_timer2{250000};
    float myfps = 0.0;
    int oldfps = 0;
    int64_t fps_mark;
    bool simulating_last = false, mule_drawn = false, dirty = true;
    int ui_context_last = MuleChassisUI;
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
        for (int row = 0; row < arraysize(simgrid); row++) {
            for (int col = 0; col < arraysize(simgrid[row]); col++) {
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
                    int cntr_x = touch_cell_h_pix*col + (touch_cell_h_pix>>1) + 2 + disp_simbuttons_x - 5 + 2;
                    int cntr_y = touch_cell_v_pix*row + (touch_cell_v_pix>>1) + disp_simbuttons_y - 1;
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
            spr->setFont(&fonts::TomThumb);
            spr->setTextDatum(textdatum_t::top_left);
            spr->setTextColor(BLK);
            spr->setCursor(10, 10);
            spr->print(std::to_string(oldfps).c_str());
            spr->setCursor(9, 9);
            spr->print(std::to_string(dispfps).c_str());
            spr->setCursor(11, 11);
            spr->print(std::to_string(dispfps).c_str());
            spr->setTextColor(LGRY);
            spr->setCursor(10, 10);
            spr->print(std::to_string(dispfps).c_str());
            oldfps = dispfps;
        }
    }
    bool touched() {
        if (touch->touched()) {
            touchp[HORZ] = touch->touch_pt(HORZ) - vp.x;
            touchp[VERT] = touch->touch_pt(VERT) - vp.y;
            return true;
        }
        return false;
    }
    int touch_pt(int axis) { return touchp[axis]; }
  public:
    int sec, psec, _width, _height, _myfps = 0, frame_count = 0;
    bool anim_reset_request = false;
    PanelAppManager(EZReadDrawer* _ez) : ezdraw(_ez) {}
    void set_vp(int _cornerx, int _cornery, int _sprwidth, int _sprheight) {
        vp.x = _cornerx;
        vp.y = _cornery;
        vp.w = _sprwidth;
        vp.h = _sprheight;
    }
    void change_saver() {  // pass non-negative value for a specific pattern, -1 for cycle, -2 for random
        ++nowsaver %= NumSaverMenu;
        anim_reset_request = true;
        still_running = 1;
    }
    void setup(LGFX* _lgfx, Simulator* _sim, Touchscreen* _touch, int _cornerx, int _cornery, int _sprwidth, int _sprheight) {
        Serial.printf("  panel app manager init ..");
        mylcd = _lgfx;
        sim = _sim;
        touch = _touch;
        set_vp(_cornerx, _cornery, _sprwidth, _sprheight);
        _width = vp.w << SHIFTSIZE;
        _height = vp.h << SHIFTSIZE;
        Serial.printf(" screensavers & ezread console .. ");
        eSaver.setup(&framebuf[flip], &vp);
        cSaver.setup(&framebuf[flip], &vp);
        ezdraw->setup(&vp);
        Serial.printf("set up\n");
    }
    void reset() {
        if (nowsaver == Eraser) eSaver.reset(&framebuf[flip], &framebuf[!flip], &vp);
        else if (nowsaver == Collisions) cSaver.reset(&framebuf[flip], &framebuf[!flip], &vp);
        anim_reset_request = false;
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
            ezdraw->dirty = true;
        }
        if (ui_context == ScreensaverUI) {  // With timer == 16666 drawing dots, avg=8k, peak=17k.  balls, avg 2.7k, peak 9k after 20sec
            // mule_drawn = false;  // With max refresh drawing dots, avg=14k, peak=28k.  balls, avg 6k, peak 8k after 20sec
            if (nowsaver == Eraser) {
                still_running = eSaver.update(spr, &vp);
                if (touched()) eSaver.saver_touch(spr, touch_pt(HORZ), touch_pt(VERT));
                if (auto_saver_enabled) {
                    int changeit = encoder.rotdirection();
                    if (changeit > 0) eSaver.change_pattern(-1);
                    else if (changeit < 0) eSaver.change_pattern(-3);  // still_running = 0;
                }
                display_fps(spr);
            }
            else if (nowsaver == Collisions) {
                if (touched()) cSaver.saver_touch(spr, touch_pt(HORZ), touch_pt(VERT));
                if (auto_saver_enabled) {
                    int changeit = encoder.rotdirection();
                    if (changeit != 0) still_running = 0;
                }
                if ((bool)still_running) still_running = cSaver.update(spr, &vp);
                display_fps(spr);
            }
            if (!(bool)still_running) change_saver();
            if (bootbutton.shortpress()) change_saver();
        }
        else if (ui_context == EZReadUI) ezdraw->update(spr);
        else if (ui_context == MuleChassisUI) {
            if (!mule_drawn) {
                spr->fillSprite(BLK);
                float w = 145.0, h = 74.0;
                // spr->pushImageRotateZoom(vp.x, vp.y, w / 2, h / 2, 0, vp.w / w, vp.h / h, w, h, mulechassis_145x74x8, BLK);
                // spr->pushImageRotateZoom(vp.x + (vp.w - 145) / 2, vp.y + (vp.h - 74) / 2, 82, 37, 0, vp.w / 145, vp.h / 74, 145, 74, mulechassis_145x74x8, BLK);
                spr->pushImageRotateZoom(85 + vp.x, 85 + vp.y, w / 2, h / 2, 0, 1, 1, w, h, mulechassis_145x74x8, BLK);
                mule_drawn = true;
            }
        }
        spr->clearClipRect();
        if (sim->enabled()) draw_simbuttons(spr, sim->enabled());  // if we just entered simulator draw the simulator buttons, or if we just left erase them
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
};
#ifdef CONVERT_IMAGE 
#define IMAGE_ARRAY mulechassis_145x74 
#define IMAGE_WIDTH 145
void convert_565_to_332_image() {
    Serial.printf("const uint8_t %sx8[%ld] PROGMEM = {\n\t", String(IMAGE_ARRAY).c_str(), arraysize(IMAGE_ARRAY));
    for (int i=0; i<arraysize(IMAGE_ARRAY); i++) {
        Serial.printf("0x%02x, ", color_16b_to_8b(IMAGE_ARRAY[i]));
        if (!(bool)(i % IMAGE_WIDTH)) Serial.printf("  // pixel# %ld\n%s", pixcount, (i < arraysize(IMAGE_ARRAY) - 1) ? "\t" : "");
    }
    Serial.printf("};\n");
}
#endif