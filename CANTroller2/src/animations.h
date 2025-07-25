#pragma once
#include <Arduino.h>

#define disp_apppanel_x 150
#define disp_apppanel_y 43  // was 48
#define disp_apppanel_w (disp_width_pix - disp_apppanel_x)  // 156
#define disp_apppanel_h (disp_height_pix - disp_apppanel_y)  // 197  // was 192
#define disp_simbuttons_x 164
#define disp_simbuttons_y 48
#define disp_simbuttons_w (disp_width_pix - disp_simbuttons_x)  // 156
#define disp_simbuttons_h (disp_height_pix - disp_simbuttons_y)  // 192
#define disp_font_height 8
#define disp_font_width 6
int simgriddir[4][3] = {
    { HrcPlus,  HrcPlus,  HrcPlus,  },
    { HrcMinus, HrcMinus, HrcMinus, },
    { HrcPlus,  HrcUp,    HrcRt,    },
    { HrcMinus, HrcDn,    HrcLt,    },
};
std::string simgrid[4][3] = {
    { "psi", "rpm", "mph" },
    { "psi", "rpm", "mph" },
    { "pos", "joy", "joy" },
    { "pos", "joy", "joy" },
};
volatile bool _is_running;
static constexpr int SHIFTSIZE = 8;
volatile bool flip = 0;
volatile int screen_refresh_time;
LGFX lcd;
static constexpr int num_bufs = 2;
LGFX_Sprite framebuf[num_bufs];
struct viewport { int x; int y; int w; int h; };
class CollisionsSaver {
  public:
    struct ball_info_t { int x; int y; int dx; int dy; int r; int m; uint8_t color; };
    bool touchnow = false, touchlast;
    viewport* vp;
    ball_info_t* balls;
    ball_info_t* a;
    static constexpr bool touchball_invisible = false;
    static constexpr int touchball_r = 15;
    static constexpr int BALL_MAX = 35;  // 256
    LGFX_Sprite* sprite;
    ball_info_t _balls[2][BALL_MAX], touchball = { 100, 100, 0, 0, touchball_r << SHIFTSIZE, 10, GRN };
    int _ball_count = 0, _myfps = 0, ball_thismax, ball_count = 0, myfps = 0, frame_count = 0;
    int _width, _height, touchx, touchy, lastx, lasty, sec, psec, ball_create_rate = 3200, _loop_count = 0;
    float ball_radius_base = 4.5f / 235.0f;  // 7 pixels radius / 125x100 sprite = about 5 pix per 235 sides sum
    float ball_radius_modifier = 2.6f / 235.0f;  // 4 pixels radius / 125x100 sprite = about 3 pix per...
    uint8_t ball_redoubler_rate = 0x18;  // originally 0x07
    int8_t sqrme, slices = 8, ball_gravity_x = 0, ball_gravity_y = 16;  // ball_gravity = 16;  // originally 0 with suggestion of 4
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
        a->color = lgfx::color332(100 + (std::rand() % 155), 100 + (std::rand() % 155), 100 + (std::rand() % 155));
        a->x = _width * rn(2);
        a->dx = (std::rand() & (5 << SHIFTSIZE)) + 1;
        a->dy = (std::rand() & (5 << SHIFTSIZE)) + 1;
        sqrme = (uint8_t)(ball_radius_base * (float)(vp->w + vp->h));
        sqrme += rn((int)(ball_radius_modifier * (float)(vp->w + vp->h)));
        for (int i=0; i<2; i++) if (!(bool)rn(ball_redoubler_rate)) sqrme *=2;
        a->r = sqrme << SHIFTSIZE;  // (sqrme * sqrme)));
        a->m = 4 + (ball_count & 0x07);
    }
    bool mainfunc(void) {
        bool new_round = false;
        static constexpr float e = 0.999;  // 0.999 coefficient of friction
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
                if (std::abs(vx) > rr) continue;
                vy = a->y - b->y;
                if (std::abs(vy) > rr) continue;
                len = std::sqrt(vx * vx + vy * vy);
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
                a->dx = std::roundf(adx + arx);
                a->dy = std::roundf(ady + ary);
                if (j < ball_count) {
                    b->dx = std::roundf(bdx + brx);
                    b->dy = std::roundf(bdy + bry);
                }
            }
        }
        _myfps = myfps;
        _ball_count = ball_count;
        return new_round;
    }
  public:
    void touch(LGFX_Sprite* spr, int x, int y) {  // you can create a new phantom ball and bash around the other balls w/ it
        touchnow = true;
        touchx = map(x, vp->x, vp->x + vp->w, 0, vp->w);
        touchy = map(y, vp->y, vp->y + vp->h, 0, vp->h);
        lastx = touchball.x;
        lasty = touchball.y;
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
        static Timer gravtimer;
        if (gravtimer.expired()) {
            float radius = 10.0, cosinc = 0.985, sininc = 0.174; // precomputed trig values  // angleinc = 10.0 * M_PI / 180.0; // Increment in radians
            static float x = 0, y = radius, mag = 0.5;
            mag = constrain((float)(mag + 0.01 * (float)(rn(70) - 35)), 0.0f, 1.0f);
            x = x * cosinc - y * sininc; // apply rotation and get new coordinates
            y = x * sininc + y * cosinc;
            ball_gravity_x = (int)(x * radius * mag);  // ball_gravity_x = constrain((ball_gravity_x + rn(6) - 3), -18, 28);
            ball_gravity_y = (int)(y * radius * mag);  // ball_gravity_y = constrain((ball_gravity_y + rn(6) - 3), -18, 28);
            gravtimer.set(200000 * (3 + rn(3)));
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
class EraserSaver {  // draws colorful patterns to exercise video buffering performance
 private:
    enum savershapes { Wedges, Dots, Rings, Ellipses, Boxes, Ascii, Worm, Rotate, NumSaverShapes };
    LGFX_Sprite* sprite;
    viewport* vp;
    float pensat = 200.0;
    uint8_t pencolor = RED, sat = rn(256), brt = rn(256), c = rando_color();
    uint16_t spothue = 65535, penhue = rn(65535), hue = rn(65535);
    std::string sub = "\x64\x6f\x20\x64\x72\x75\x67\x73";
    int season = rn(4), precess = rn(10), shdone = 0, point[2], plast[2], spotrate = 300;
    int cycle = 0, scaler = 1, cycletime = 50000000, seasontime = 3000000, cycle_season_ratio = 16;
    Timer extratimer{2850000}, lucktimer, seasontimer, cycletimer;
    bool lotto = false, has_eraser = true;
 public:
    int shape, num_shapes = NumSaverShapes;
    EraserSaver() {}
    void setup(LGFX_Sprite* _nowspr, viewport* _vp) {
        sprite = _nowspr;
        vp = _vp;
        lucktimer.set((2 + rn(5)) * 10000000);
        cycletimer.set(cycletime);
        seasontimer.set(seasontime);
        shape = rn(Rotate);
    }
    void reset(LGFX_Sprite* sp0, LGFX_Sprite* sp1, viewport* _vp) {
        vp = _vp;
        change_pattern(-2);  // randomize new pattern whenever turned off and on
        scaler = std::max(1, (vp->w + vp->h)/200);
        _is_running = true;
    }
    void touchwrite(LGFX_Sprite* spr, int x, int y) {  // you can draw colorful lines on the screensaver
        // pencolor = (cycle == 1) ? rando_color() : hsv_to_rgb<uint8_t>(penhue, (uint8_t)pensat, 200 + rn(56));
        spr->fillCircle(x, y, 20 * scaler, pencolor);
        // spr->fillCircle(x + vp->x, y + vp->y, 20 * scaler, pencolor);
    }
    void touchadjust(int x, int y) {  // you can draw colorful lines on the screensaver
        season = map(y, vp->y, vp->y + vp->h, 0, 3);
        precess = map(x, vp->x, vp->x + vp->w, 0, 9);
    }
    void pot_timing() {
        if (!pot_controls_animation_timeout) return;
        static int potval, lastpot;
        int lasttime, maxcycletime = 1800000000, mincycletime = 3000000;
        lastpot = potval;
        potval = (int)(pot.val());
        if (std::abs(potval - lastpot) > 1) {
            cycletime = constrain(map(potval, 0, 100, maxcycletime, mincycletime), mincycletime, maxcycletime);
            seasontime = cycletime / cycle_season_ratio;
            cycletimer.set(cycletime);
            seasontimer.set(seasontime);
        }
    }
    int update(LGFX_Sprite* _nowspr, viewport* _vp) {
        static int last_season = 0, last_precess = 3, num_cycles = 3;
        int numseasons = 4;
        sprite = _nowspr;
        vp = _vp;
        // pot_timing();
        if (cycletimer.expired()) {
            ++cycle %= num_cycles;
            if (cycle == 0) change_pattern(-1);
            cycletimer.set((cycletime / ((cycle == 2) ? 5 : 1)) << (shape == Worm));
        }  // Serial.printf("[c%d] ", cycle);
        if (seasontimer.expired()) {
            last_season = season;
            ++season %= numseasons;
            last_precess = precess;
            precess = (precess + 9 - season - rn(2)) % 10;  // Serial.printf("(p%d s%d %ldms) ", precess, season, (int)seasontimer.elapsed()/1000);
            seasontimer.set(3200000 * (2 + rn(5)));
        }
        drawsprite();
        return shdone;
    }
    int change_pattern(int newpat = -1) {  // pass non-negative value for a specific pattern, or  -1 for cycle, -2 for random, -3 for cycle backwards  // XX , -4 for autocycle (retains saver timeout)
        ++shdone %= 7;
        int last_pat = shape;
        has_eraser = !(bool)rn(3);
        if (0 <= newpat && newpat < NumSaverShapes) shape = newpat;  //
        else {
            if (newpat == -1) ++shape %= Rotate;
            if (newpat == -3) shape = (shape - 1 + Rotate) % Rotate;
            else if (newpat == -2) while (last_pat == shape) shape = rn(Rotate);
            if (rn(25) == 13) shape = Rotate;
        }  // Serial.printf("\ns%d %ldms: ", shape, (int)cycletimer.elapsed()/1000);
        return shape;
    }
  private:
    void update_pen() {
        static int pensatdir = 1;
        static Timer pentimer{110000};
        if (cycle != 2) {
            if (pentimer.expireset()) {
                if (season == 1 || season == 3) {
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
        static uint16_t myhue[2] = { 0, 32767 };
        static uint8_t wclast;
        uint8_t wcball, wctip;  // static uint16_t hue = rn(65535);
        bool flipper = (bool)rn(2);
        float im = 0;
        hue = rn(65536);
        for (int i=0; i<2; i++) myhue[i] += rn(511) - 255;
        if (season == 1) {
            brt = 156 + rn(100);
            wctip = hsv_to_rgb<uint8_t>(hue, 0, brt);
            wcball = hsv_to_rgb<uint8_t>(hue, 64, brt);
        }
        else if (season == 2) {
            wcball = hsv_to_rgb<uint8_t>((flipper) ? myhue[0] : myhue[1], pensat, 200 + rn(56));
            wctip = hsv_to_rgb<uint8_t>((flipper) ? myhue[1] : myhue[0], pensat, 200 + rn(56));
        }
        else if (season == 3) {
            wctip = hsv_to_rgb<uint8_t>((flipper) ? myhue[0] : myhue[1], rn(256), 156 + rn(100));
            wcball = hsv_to_rgb<uint8_t>((flipper) ? myhue[1] : myhue[0], 88, 32 + rn(64));
        }
        else {
            wcball = hsv_to_rgb<uint8_t>(hue, 127 + (spothue >> (season + 5)), 200 + rn(56));
            wctip = wclast;
        }
        if (plast[Vert] != point[Vert]) im = (float)(plast[Horz] - point[Horz]) / (float)(plast[Vert] - point[Vert]);
        sprite->fillCircle(plast[Horz] + vp->x, plast[Vert] + vp->y, 3, wcball);
        for (int h=-4; h<=4; h++)
            sprite->drawGradientLine(point[Horz] + vp->x, point[Vert] + vp->y, plast[Horz] + vp->x + (int)(h / ((std::abs(im) > 1.0) ? im : 1)), plast[Vert] + vp->y + (int)(h * ((std::abs(im) > 1.0) ? 1 : im)), wctip, wcball);
        wclast = wcball;
    }
    void run_ellipses() {
        int d[2] = {10 + rn(30), 10 + rn(30)};  //  this crashes after 10 seconds:  int d[2] = { 5 + rn(20 * (int)(season / 2)), 5 + rn(20 * (int(season / 2))) };  // 
        uint16_t mult = rn(2000);
        hue = spothue + rn(3000);
        spotrate = (int)((season * 200) + rn(200));
        sat = 96 + precess * 7 + rn(20 + season * 32);
        brt = 120 + rn(136);
        for (int i = 0; i < 6 + rn(20); i++) {
            c = hsv_to_rgb<uint8_t>(hue + mult * i, sat, brt);
            sprite->drawEllipse(point[Horz] + vp->x, point[Vert] + vp->y, scaler * d[0] - i, scaler * d[1] + i, c);
        }
    }
    void run_rings() {
        int d = 3 + rn(14 + (precess - 4) * 2);
        uint8_t c2;
        uint16_t hue_common = hue = spothue + 32768 * rn(2) + rn(1500);
        if (extratimer.expired()) {
            spotrate = (int)(200 + rn(800));
            extratimer.set(500000 * (1 + rn(4)));
        }
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
        if (!(precess % 3)) c2 = c;  // hsv_to_rgb<uint8_t>(hue, sat, std::abs(brt-10));
        else if (!(precess % 2)) c2 = hsv_to_rgb<uint8_t>(hue, sat, std::abs(brt-10));
        else c2 = BLK;  // Serial.printf("%3.0f%3.0f%3.0f (%3.0f%3.0f%3.0f) (%3.0f%3.0f%3.0f)\n", (float)(hue/655.35), (float)(sat/2.56), (float)(brt/2.56), 100*(float)((c >> 11) & 0x1f)/(float)0x1f, 100*(float)((c >> 5) & 0x3f)/(float)0x3f, 100*(float)(c & 0x1f)/(float)0x1f, 100*(float)((c2 >> 11) & 0x1f)/(float)0x1f, 100*(float)((c2 >> 5) & 0x3f)/(float)0x3f, 100*(float)(c2 & 0x1f)/(float)0x1f);
        for (int xo = -1; xo <= 1; xo += 2) {
            sprite->drawCircle(point[Horz] + vp->x, point[Vert] + vp->y, d * scaler, c);
            sprite->drawCircle(point[Horz] + vp->x, point[Vert] + vp->y + xo, d * scaler, c);
            sprite->drawCircle(point[Horz] + vp->x + xo, point[Vert] + vp->y, d * scaler, c);
        }
        for (int edge = -1; edge <= 1; edge += 2)
            sprite->drawCircle(point[Horz] + vp->x, point[Vert] + vp->y, d * scaler + edge, c2);
    }
    void run_dots() {
        int total_punches = 12, p[2], stars = 0, r, myshape = rn(season + precess);
        static int punches_left, oldseason, oldprec, mindot = 4, adddot = 4;
        static bool was_eraser = has_eraser, flipper; 
        static uint16_t myhue[2] = { 0, 32767 };
        spotrate = (int)(rn(900));
        if ((precess > oldprec) && !rn(2)) {  // sometimes on leap year we slam them with a few bigger punches
            has_eraser = false;
            punches_left = total_punches;
        }
        oldprec = precess;
        if (season != oldseason) {
            mindot = constrain(mindot + rn(3) - 1, 1 + (cycle >> 1), 2 + cycle);
            adddot = constrain(adddot + rn(3) - 1, 1, 3 + (cycle >> 1));
        }
        oldseason = season;
        if (myshape < 2) myshape = 0;
        else if (myshape < 6) myshape = 1;
        else myshape = 2;
        if (!punches_left) {
            stars = 13 - (adddot / 2) - (mindot + adddot > 8) ? 3 : 0;
            has_eraser = was_eraser;
        }
        else if (extratimer.expired()) stars = 1;
        for (int i=0; i<2; i++) myhue[i] += rn(511) - 255;
        for (int star = 0; star < stars; star++) {
            p[0] = vp->x + (punches_left > 0) ? vp->w >> 1 : rn(vp->w);
            p[1] = vp->y + (punches_left > 0) ? vp->h >> 1 : rn(vp->h);
            if (!punches_left) r = scaler * (mindot + rn(adddot));
            else r = scaler * (int)((float)vp->h * 0.45 * (1.0 - ((float)punches_left / (float)total_punches)));
            if (precess < 6) {
                c = hsv_to_rgb<uint8_t>(myhue[flipper], pensat, 200 + rn(56));
                flipper = !flipper;
            }
            else {
                sat = (20 * season) + rn(256 - 30 * season);
                hue = (myshape == 2) ? spothue : myhue[(bool)myshape];
                c = hsv_to_rgb<uint8_t>(hue, sat, 130 + rn(126));
            }
            if (myshape == 0) sprite->fillCircle(p[0], p[1], r, c);
            else if (myshape == 1) sprite->fillRect(p[0] - r, p[1] - r, r * 2, r * 2, c);
            else {
                int pt2[2] = { p[0] - (int)(1.4 * (float)r), p[1] + int(1.05 * (float)r) };
                int pt3[2] = { p[0] + (int)(1.4 * (float)r), pt2[1] };                    
                p[1] -= (int)(1.35 * (float)r);
                sprite->fillTriangle(p[0], p[1], pt2[0], pt2[1], pt3[0], pt3[1], c);
            }
            if (punches_left) extratimer.set(((punches_left < 3) ? 60000 : 9000) * (total_punches + 1 - punches_left--));
        }
    }
    void run_boxes() {
        static int boxsize[2], boxrad, boxminsize;
        int longer = rn(2);
        uint8_t boxcolor;
        has_eraser = false;
        boxrad = rn(1 + rn(2) * season);  // note this will crash us!  ->  boxrad = rn(5 * season);
        boxminsize = 2 * boxrad + 5;
        boxsize[longer] = boxminsize + rn(vp->w - boxminsize);
        boxsize[!longer] = boxminsize + rn(boxsize[longer] >> 2);  // cheesy attempt to work around crazy-values bug
        point[Horz] = rn(vp->w) - (boxsize[Horz] >> 1);
        point[Vert] = rn(vp->h) - (boxsize[Vert] >> 1);
        for (int axis=Horz; axis<=Vert; axis++) {
            if (point[axis] < 0) {
                boxsize[axis] += point[axis];
                point[axis] = -boxrad;
            }
        }
        if (point[Horz] + boxsize[Horz] > vp->w) boxsize[Horz] = (vp->w + boxrad - point[Horz]);
        if (point[Vert] + boxsize[Vert] > vp->h) boxsize[Vert] = (vp->h + boxrad - point[Vert]);
        int shells = 1 + (rn(5) != 0) ? 1 + rn(4) : 0;
        int steps[2] = { boxsize[Horz] / (shells + 1), boxsize[Vert] / (shells + 1) };
        // this crashes it
        // if (precess > 5 && !rn(2)) shells = boxsize[!longer] >> 1;
        // else shells = (int)(rn(5) > 0);
        // int steps[2] = { boxsize[Horz] / shells, boxsize[Vert] / shells };
        for (int mat=0; mat<shells; mat++) {
            if (season == 0) boxcolor = rando_color();
            else if (season == 1) boxcolor = hsv_to_rgb<uint8_t>((uint16_t)rn(65535), rn(256), rn(256));
            else if (season == 2) boxcolor = hsv_to_rgb<uint8_t>((uint16_t)((spothue + rn(2) * 32767 + rn(512)) % 65535), 100 + rn(106), 255);
            else if (season == 3) boxcolor = hsv_to_rgb<uint8_t>((uint16_t)((spothue + rn(1024)) % 65535), 150 + rn(56), 255);
            for (int axis=Horz; axis<=Vert; axis++) {
                boxsize[axis] -= steps[axis];
                point[axis] += (steps[axis] >> 1);
            }
            sprite->fillSmoothRoundRect(point[Horz] + vp->x, point[Vert] + vp->y, boxsize[Horz], boxsize[Vert], boxrad, boxcolor);  // Change colors as needed
        }
    }
    void run_ascii() {
        static float offset[2];
        static int final[2];
        sprite->setTextSize(1);
        sprite->setTextDatum(textdatum_t::middle_center);
        sprite->setFont(&fonts::Font4);
        for (int star = 0; star < 4; star++) {
            point[Horz] = rn(vp->w);
            point[Vert] = rn(vp->h);
            if (precess > 4) {
                hue = (vp->h * (int)(season > 1)) - point[Vert] * 65535 / vp->h;
                sat = point[Horz] * -255 / vp->w;
            }
            else {
                hue = (vp->w * (int)(season == 0 || season == 2)) - point[Horz] * 65535 / vp->w;
                sat = point[Vert] * -255 / vp->h;
            }
            sat = map(sat, 0, 255, 20, 255);
            for (int axis=Horz; axis <= Vert; axis++) offset[axis] += (float)rn(100) / 100;
            final[Horz] = (point[Horz] + ((season < 3) ? (int)(offset[Horz]) : 0)) % vp->w + vp->x;
            final[Vert] = (point[Vert] + ((season > 0) ? (int)(offset[Vert]) : 0)) % vp->h + vp->y;
            std::string letter = std::string(1, static_cast<char>(0x21 + rn(0x5d)));
            uint8_t c = hsv_to_rgb<uint8_t>(hue, sat, 150 + 50 * (spothue < (32767 / (season+1))) + rn(56));
            sprite->setTextColor(BLK);  // allows subliminal messaging
            sprite->drawString(letter.c_str(), final[Horz] + 1, final[Vert] + 1);  // these will not work at extreme sides
            sprite->drawString(letter.c_str(), final[Horz] - 1, final[Vert] - 1);  // these will not work at extreme sides
            sprite->setTextColor(hsv_to_rgb<uint8_t>(hue, sat, 150 + 50 * (spothue > 32767) + rn(56)));
            sprite->drawString(letter.c_str(), final[Horz], final[Vert]);
        }
    }
    void run_worm() {
        static int wormpos[2] = { 0, 0 }, wormvel[2] = { 0, 0 }, wormsign[2] = { 1, 1 }, wormd[2] = { 20, 20 };
        static int shifter = 2, wormdmin = 8, wormdmax = 38, wormvelmax = 400, wormsat = 128, brightmod = rn(66);
        static Timer movetimer{20000}, wormtimer{1000000};
        static bool wormstripe = false, fading = false;
        int colorslices = 11, stripeslices = 1, fadeslices = 5, slicetime = 75000;
        has_eraser = lotto = false;
        lucktimer.reset();
        if (season == 0 || season == 2) sat = pensat;
        else sat = 105 + (int)(150.0 * (float)(((precess > 5) ? 10 - precess : precess)) / 5.0);
        brightmod = constrain(brightmod + rn(5) - 2, 0, 65);
        if (extratimer.expired()) {
            fading = !fading;
            if (!fading) wormstripe = !wormstripe;  // wormstripe changes at the end of fade, in either direction
            extratimer.set(slicetime * (fading ? fadeslices : (wormstripe ? stripeslices : colorslices)));
        }
        brt = 190 + brightmod;
        if (fading) {
            if (wormstripe) brt = brt * (int)extratimer.elapsed() / (slicetime * fadeslices);
            else brt -= brt * (int)extratimer.elapsed() / (slicetime * fadeslices);
        }
        else if (wormstripe) brt = 0;
        c = hsv_to_rgb<uint8_t>(penhue, sat, brt);
        int wormposmax[2] = { (vp->w - wormd[Horz]) / 2, (vp->h - wormd[Vert]) / 2 };
        if (movetimer.expireset()) {
            for (int axis = Horz; axis <= Vert; axis++) {
                wormpos[axis] += (wormvel[axis] >> 6) * wormsign[axis];
                if ((wormpos[axis] * wormsign[axis]) >> shifter >= wormposmax[axis] + 2) {
                    wormpos[axis] = (wormsign[axis] * wormposmax[axis]) << shifter;
                    wormsign[axis] *= -1;
                    wormvel[1-axis] = rn(wormvelmax);
                    if (wormvel[1-axis] < (1 << 6)) wormvel[1-axis] = wormvel[1-axis] << 1;
                }
            }
        }
        for (int axis = Horz; axis <= Vert; axis++)
            if (rn(3) == 0) wormd[axis] = constrain(wormd[axis] + rn(3) - 1, wormdmin, wormdmax);
        if (wormtimer.expireset())
            for (int axis = Horz; axis <= Vert; axis++)
                wormvel[axis] = constrain(wormvel[axis] + rn(255) - 127, 0, wormvelmax);
        for (int xo1 = -1; xo1 <= 1; xo1 += 2) {
            point[Horz] = (vp->w / 2) + (wormpos[Horz] >> shifter) + vp->x + xo1;
            point[Vert] = (vp->h / 2) + (wormpos[Vert] >> shifter) + vp->y + xo1;
            for (int xo2 = -1; xo2 <= 1; xo2 += 2) {
                sprite->drawEllipse(point[Horz], point[Vert], wormd[Horz] * scaler, wormd[Vert] * scaler, c);
                sprite->drawEllipse(point[Horz] + xo2, point[Vert] + xo2, wormd[Horz] * scaler, wormd[Vert] * scaler, c);
                sprite->drawEllipse(point[Horz] + xo2, point[Vert], wormd[Horz] * scaler, wormd[Vert] * scaler, c);
            }
        }
    }
    void the_eraser() {
        static bool seizure = false;
        static Timer seizuretimer{10000000};
        int eraser_velo_min = 3, eraser_velo_max = 7;
        static int erpos[2] = { 0, 0 }, eraser_velo_sign[2] = { 1, 1 }, eraser_rad = 14;
        static int eraser_velo[2] = { rn(eraser_velo_max), rn(eraser_velo_max) };
        if ((cycle != 0) && has_eraser && !seizure) {
            int erpos_max[2] = {(vp->w - eraser_rad) / 2, (vp->h - eraser_rad) / 2};
            for (int axis = Horz; axis <= Vert; axis++) {
                erpos[axis] += eraser_velo[axis] * eraser_velo_sign[axis];
                if (erpos[axis] * eraser_velo_sign[axis] >= erpos_max[axis] + 5) {
                    erpos[axis] = eraser_velo_sign[axis] * erpos_max[axis];
                    eraser_velo[axis] = eraser_velo_min + rn(eraser_velo_max - eraser_velo_min);
                    eraser_velo[!axis] = eraser_velo_min + rn(eraser_velo_max - eraser_velo_min);
                    eraser_velo_sign[axis] *= -1;
                    eraser_rad = constrain((int)(eraser_rad + rn(5) - 2), 22, 40);
                }
            }
            sprite->fillCircle((vp->w / 2) + erpos[Horz] + vp->x, (vp->h / 2) + erpos[Vert] + vp->y, eraser_rad * scaler, BLK);
        }
        if (seizuretimer.expired()) {
            seizure = !seizure;
            seizuretimer.set(200000 + 300000 * rn(seizure ? 4 : 10));
        }
        if (lucktimer.expired())  {
            lotto = !lotto;
            lucktimer.set(1900000 + (int)(!lotto) * (200 + 3 * rn(255)) * 4000000);
        } 
        if (lotto) {
            sprite->setTextDatum(textdatum_t::middle_center);
            sprite->setFont(&fonts::Font4);
            sprite->setTextColor(BLK);
            sprite->drawString(sub.c_str(), vp->w / 2 + vp->x, vp->h / 2 + vp->y);
        }
    }
    void drawsprite() {
        static int rotate = -1;
        point[Horz] = rn(vp->w);
        point[Vert] = rn(vp->h);
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
        for (int axis = Horz; axis <= Vert; axis++) plast[axis] = point[axis];  // erlast[axis] = erpos[axis];
    }
};
class EZReadDrawer {  // never has any terminal application been easier on the eyes
  public:
    bool dirty = true;
    std::string drawnow; // Ring buffer array
  private:
    static const int row_padding_pix = 1;
    int _main_x, pix_margin = 2, linelength, scrollbar_width = 3, font_height = 6 + row_padding_pix;  // add one extra line between lines for readability
    LGFX* mylcd;
    LGFX_Sprite* nowspr_ptr;
    viewport* vp;
    EZReadConsole* ez;
    void draw_scrollbar(LGFX_Sprite* spr, uint8_t color) {  // this runs but is not finished and doesn't do anything
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
        draw_scrollbar(spr, LGRY);  // currently does not draw anything
        spr->fillSprite(BLK);
        spr->setTextWrap(false);
        spr->setFont(&fonts::TomThumb);
        spr->setTextDatum(textdatum_t::top_left);
        int max_lines = (vp->h + 1 + row_padding_pix) / font_height;  // add 1-2 to vp->h b/c do not need extra spacing between rows on last row
        if (max_lines <= 0) return;
        int max_offset = ez->has_wrapped ? ez->bufferSize - 1 : ez->current_index;  // clamp offset to available history
        int safe_offset = constrain(ez->offset, 0, max_offset);
        int newest_line = (ez->current_index - safe_offset + ez->bufferSize) % ez->bufferSize; // start from the newest visible line (working backward)
        for (int i = max_lines - 1; i >= 0; --i) {
            int idx = (newest_line - (max_lines - 1 - i) + ez->bufferSize) % ez->bufferSize;
            std::string& nowline = ez->textlines[idx];
            if (nowline.empty()) continue;
            int y = vp->y + i * font_height;
            spr->setTextColor(ez->linecolors[idx]);
            spr->setCursor(_main_x, y);
            spr->print(nowline.c_str());
        }
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
    LGFX* mylcd;
    LGFX_Sprite* nowspr_ptr;
    viewport vp;
    EraserSaver eSaver;
    CollisionsSaver cSaver;
    Simulator* sim;
    Touchscreen* touch;
    EZReadDrawer* ezdraw;
    int nowsaver = Eraser, still_running = 0, touchp[2], dispfps, corner[2], oldfps = 0, ui_app_last = MuleChassisUI;
    Timer fps_timer, fps_timer2{250000};
    float myfps = 0.0;
    int64_t fps_mark;
    bool simulating_last = false, mule_drawn = false, dirty = true;
    void draw_simbutton(LGFX_Sprite* spr, int cntr_x, int cntr_y, int dir, uint8_t color) {
        if (dir == HrcPlus)  spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_plus_32x32x8, BLK);
        else if (dir == HrcMinus) spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_minus_32x32x8, BLK);
        else if (dir == HrcUp) spr->pushImage(cntr_x-16, cntr_y-16, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == HrcDn) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 180, 1, 1, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == HrcLt) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 270, 1, 1, 32, 32, blue_up_32x32x8, BLK);
        else if (dir == HrcRt) spr->pushImageRotateZoom(cntr_x, cntr_y, 16, 16, 90, 1, 1, 32, 32, blue_up_32x32x8, BLK);
    }
    void draw_simbuttons(LGFX_Sprite* spr) {  // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
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
                    int cntr_x = touch_cell_h_pix * col + (touch_cell_h_pix >> 1) + 2 + disp_simbuttons_x - 5 + 2;
                    int cntr_y = touch_cell_v_pix * row + (touch_cell_v_pix >> 1) + disp_simbuttons_y - 1;
                    if (simgrid[row][col] != "    ") {
                        draw_simbutton(spr, cntr_x + 2, cntr_y - 1, simgriddir[row][col], YEL);  // for 3d look
                        draw_simbutton(spr, cntr_x, cntr_y, simgriddir[row][col], DGRY);
                        // spr->fillRoundRect(cntr_x - 20, cntr_y - touch_cell_v_pix/2 - 10, 40, 20, 5, DGRY);
                        // spr->drawRoundRect(cntr_x - 20, cntr_y - touch_cell_v_pix/2 - 10, 40, 20, 5, BLK);
                        if ((row % 2) && (simgrid[row][col].find("joy") == std::string::npos)) {
                            spr->setFont(&fonts::FreeSans9pt7b);
                            spr->setTextColor(BLK);
                            spr->drawString(simgrid[row][col].c_str(), cntr_x - 1, cntr_y - touch_cell_v_pix / 2 + 5 - 1);
                            spr->drawString(simgrid[row][col].c_str(), cntr_x + 1, cntr_y - touch_cell_v_pix / 2 + 5 + 1);
                            spr->setTextColor(LYEL);
                            spr->drawString(simgrid[row][col].c_str(), cntr_x, cntr_y - touch_cell_v_pix / 2 + 5);
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
    void draw_mule(LGFX_Sprite* spr) {
        if (mule_drawn) return;
        spr->fillSprite(BLK);
        float w = 145.0, h = 74.0;
        spr->pushImageRotateZoom(85 + vp.x, 85 + vp.y, w / 2, h / 2, 0, 1, 1, w, h, mulechassis_145x74x8, BLK);
        mule_drawn = true;
    }
    int check_cycle_req() {  // check for user input wanting to cycle the animation (in fullscreen animation mode)
        if (!auto_saver_enabled) return DirNone;
        int changeit = encoder.rotdirection();
        int swipe = touch->swipe();  // check for touchscreen swipe event
        if (swipe == DirLeft) changeit--;  // swipe left for previous animation
        else if (swipe == DirRight) changeit++;  // swipe right for next animation

        if (touch->doubletap()) { // temporary alternate touch method since swiping is broken
            if (touch->touch_pt(Horz) >= (disp_width_pix >> 1)) changeit++; // doubletap on right half to cycle fwd
            else changeit--;                                                // or on left half to cycle back
        }
        if (bootbutton.shortpress()) changeit++;  // boot button also cycles, always forward
        return changeit;
    }
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
    void cycle_anim(int changeit=DirFwd) {  // cycle the animation
        if (changeit == DirNone) return;    // do nothing
        changeit = constrain(changeit, DirRev, DirFwd);
        if (nowsaver == Collisions) {  // if on ball saver
            change_saver();  // switch to eraser saver
            eSaver.change_pattern((changeit == DirFwd) ? 0 : eSaver.num_shapes - 1);  // go to first or last erasersaver pattern
            return;
        }
        if (changeit == DirFwd) {  // go forward one animation
            if (eSaver.shape == eSaver.num_shapes - 1) change_saver();  // going past the last erasersaver pattern changes to ball saver
            else eSaver.change_pattern(-1);  // erasersaver goes to next pattern
            return;
        }  // else go back one animation (below)
        if (eSaver.shape == 0) change_saver();  // requesting to go below 1st erasersaver pattern switches to ball saver
        else eSaver.change_pattern(-3);  // erasersaver goes back one pattern
    }
    void change_saver() {  // switch back and forth between ball saver / eraser saver
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
        if ((ui_app_last != ScreensaverUI) && (ui_app == ScreensaverUI)) cycle_anim(DirFwd);  // next saver.  ptrsaver->reset();
        if (ui_app_last != ui_app) dirty = true;
        if (argdirty) dirty = true;
        ui_app_last = ui_app;
        if (anim_reset_request) reset();
        spr->setClipRect(vp.x, vp.y, vp.w, vp.h);
        if (!sim->enabled() && simulating_last) dirty = true;  // if we just left the simulator erase everything to get rid of the simulator buttons
        if (dirty) {  // clear the screen
            spr->fillSprite(BLK);
            mule_drawn = false;
            ezdraw->dirty = true;
        }
        bool touch_valid = (touch->touched() && (touch->landed_pt(Horz) >= vp.x) && (touch->landed_pt(Vert) >= vp.y) && 
                           (touch->landed_pt(Horz) < vp.x + vp.w) && (touch->landed_pt(Vert) < vp.y + vp.h));
        if (ui_app == EZReadUI) ezdraw->update(spr);
        else if (ui_app == MuleChassisUI) draw_mule(spr);
        else if (ui_app == ScreensaverUI) {  // With timer == 16666 drawing dots, avg=8k, peak=17k.  balls, avg 2.7k, peak 9k after 20sec
            // mule_drawn = false;  // With max refresh drawing dots, avg=14k, peak=28k.  balls, avg 6k, peak 8k after 20sec
            if (nowsaver == Eraser) {
                still_running = eSaver.update(spr, &vp);
                if (touch_valid && touch->tap()) eSaver.touchadjust(touch->touch_pt(Horz), touch->touch_pt(Vert));
                else if (touch_valid && touch->held()) eSaver.touchwrite(spr, touch->touch_pt(Horz), touch->touch_pt(Vert)); 
            }
            else if (nowsaver == Collisions) {
                still_running = cSaver.update(spr, &vp);  // if ((bool)still_running) 
                if (touch_valid && touch->held()) cSaver.touch(spr, touch->touch_pt(Horz), touch->touch_pt(Vert));
            }
            if (!still_running) change_saver();
            else cycle_anim(check_cycle_req());
            display_fps(spr);
        }
        spr->clearClipRect();        
        if (sim->enabled()) draw_simbuttons(spr);  // if simulating draw the buttons over the bg content
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