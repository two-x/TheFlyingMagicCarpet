#pragma once
#include <Arduino.h>
// #define CONFIG_IDF_TARGET_ESP32
enum saverchoices : int { Eraser, Collisions, NumSaverMenu, Blank };
static LGFX_Sprite sp[2];
static LGFX_Sprite* nowspr;
static LGFX* lcd;
static Touchscreen* _touch;
static uint32_t corner[2], sprwidth, sprheight;
static std::size_t flip = 0;
static constexpr std::uint32_t SHIFTSIZE = 8;
static std::uint32_t sec, psec, _width, _height, _fps = 0, fps = 0, frame_count = 0;
volatile bool _is_running;
volatile std::uint32_t _draw_count;
volatile std::uint32_t _draw_count_last;
volatile std::uint32_t _loop_count;
volatile bool now_drawing = false;
static bool round_over = false;
struct ball_info_t {
    int32_t x;
    int32_t y;
    int32_t dx;
    int32_t dy;
    int32_t r;
    int32_t m;
    uint32_t color;
};
static constexpr std::uint32_t BALL_MAX = 64;  // 256
static std::uint32_t ball_count = 0;
static ball_info_t _balls[2][BALL_MAX];
static std::uint32_t _ball_count = 0;

void diffDraw() {
    union {
        std::uint32_t* s32;
        std::uint8_t* s;
    };
    union {
        std::uint32_t* p32;
        std::uint8_t* p;
    };
    s32 = (std::uint32_t*)sp[flip].getBuffer();
    p32 = (std::uint32_t*)sp[!flip].getBuffer();
    auto sprwidth = sp[flip].width();
    auto sprheight = sp[flip].height();
    auto w32 = (sprwidth + 3) >> 2;
    std::int32_t y = 0;
    // lcd->startWrite();
    do {
        std::int32_t x32 = 0;
        do {
            while (s32[x32] == p32[x32] && ++x32 < w32);
            if (x32 == w32) break;
            std::int32_t xs = x32 << 2;
            while (s[xs] == p[xs]) ++xs;
            while (++x32 < w32 && s32[x32] != p32[x32]);
            std::int32_t xe = (x32 << 2) - 1;
            if (xe >= sprwidth) xe = sprwidth - 1;
            while (s[xe] == p[xe]) --xe;
            lcd->pushImage(xs + corner[HORZ], y + corner[VERT], xe - xs + 1, 1, &s[xs]);
            // lcd->pushImage(xs + corner[HORZ], y + corner[VERT], xe - xs + 1, 1, &s[xs]);
        } while (x32 < w32);
        s32 += w32;
        p32 += w32;
    } while (++y < sprheight);
    lcd->display();
    // lcd->endWrite();
}
void setflip(bool clear=true) {  // clear=true blacks out the sprite before drawing on it
    flip = _draw_count & 1;
    nowspr = &(sp[flip]);
    if (clear) nowspr->clear();
}
void drawfunc(void) {
    now_drawing = true;
    ball_info_t* balls;
    ball_info_t* a;
    auto sprwidth = sp[0].width();
    auto sprheight = sp[0].height();
    setflip();
    balls = &_balls[flip][0];
    for (int32_t i = 8; i < sprwidth; i += 16) nowspr->drawFastVLine(i, 0, sprheight, 0x1F);
    for (int32_t i = 8; i < sprheight; i += 16) nowspr->drawFastHLine(0, i, sprwidth, 0x1F);
    for (std::uint32_t i = 0; i < _ball_count; i++) {
        a = &balls[i];
        nowspr->fillCircle(a->x >> SHIFTSIZE, a->y >> SHIFTSIZE, a->r >> SHIFTSIZE, a->color);
    }
    diffDraw();
    ++_draw_count;
    now_drawing = false;
}
bool mainfunc(void) {
    bool new_round = false;
    static constexpr float e = 0.999;  // Coefficient of friction
    sec = lgfx::millis() / 1000;
    if (psec != sec) {
        psec = sec;
        fps = frame_count;
        frame_count = 0;
        if (++ball_count >= BALL_MAX) {
            new_round = true;
            ball_count = 1;
        }
        auto a = &_balls[_loop_count & 1][ball_count - 1];
        a->color = lgfx::color888(100 + (rand() % 155), 100 + (rand() % 155), 100 + (rand() % 155));
        a->x = 0;
        a->y = 0;
        a->dx = (rand() & (5 << SHIFTSIZE)) + 1;  // was (3 << SHIFTSIZE)) for slower balls
        a->dy = (rand() & (5 << SHIFTSIZE)) + 1;  // was (3 << SHIFTSIZE)) for slower balls
        a->r = (4 + (ball_count & 0x07)) << SHIFTSIZE;
        a->m = 4 + (ball_count & 0x07);
        #if defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32) || defined(ESP_PLATFORM)
            vTaskDelay(1);
        #endif
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
        //  a->dy += 4; // gravity
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
    _fps = fps;
    _ball_count = ball_count;
    return new_round;
}

class Animation {
 public:
    static void init(LGFX* _lcd, Touchscreen* touch, uint32_t _cornerx, uint32_t _cornery, uint32_t _sprwidth, uint32_t _sprheight) {
        lcd = _lcd;
        _touch = touch;
        corner[HORZ] = _cornerx;
        corner[VERT] = _cornery;
        sprwidth = _sprwidth;
        sprheight = _sprheight;
        lcd->startWrite();
        lcd->setColorDepth(16);
        if (lcd->width() < lcd->height()) lcd->setRotation(lcd->getRotation() ^ 1);
        for (int i = 0; i <= 1; i++) sp[i].setColorDepth(8);  // Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
        auto framewidth = sprwidth;
        auto frameheight = sprheight;
        bool fail = false;
        for (std::uint32_t i = 0; !fail && i < 2; ++i)
            fail = !sp[i].createSprite(framewidth, frameheight);
        if (fail) {
            fail = false;
            for (std::uint32_t i = 0; !fail && i < 2; ++i) {
                sp[i].setPsram(true);
                fail = !sp[i].createSprite(framewidth, frameheight);
            }
            if (fail) {
                fail = false;
                if (framewidth > 320) framewidth = 320;
                if (frameheight > 240) frameheight = 240;
                for (std::uint32_t i = 0; !fail && i < 2; ++i) {
                    sp[i].setPsram(true);
                    fail = !sp[i].createSprite(framewidth, frameheight);
                }
                if (fail) {
                    lcd->print("createSprite fail...");
                    // lgfx::delay(3000);
                }
            }
        }
        for (int i=0; i<=1; i++) sp[i].clear(TFT_BLACK);
        // sp[0].pushImage() draw(corner[HORZ], corner[VERT]);
        // lcd->display();
        lcd->endWrite();
        _width = framewidth << SHIFTSIZE;
        _height = frameheight << SHIFTSIZE;
    }
    virtual void setup() = 0;
    virtual void reset() = 0;
    virtual void saver_touch(int16_t, int16_t) = 0;
    virtual int update() = 0;  //{ return 0; };
};
#if defined (ESP32) || defined (CONFIG_IDF_TARGET_ESP32) || defined (ESP_PLATFORM)
static void taskDraw(void*) {
    // while (true) {
        // Serial.printf("s%d n%d i%d ", screensaver, (anim_mgr->nowsaver == Collisions), _is_running);
        // while (screensaver && (anim_mgr->nowsaver == Collisions) && _is_running) {

        
        while (_is_running) {
            // Serial.printf("n%d l%d d%d ", now_drawing, _loop_count, _draw_count);
        
            // while (!anim_mgr->updated) taskYIELD();
            while ((_loop_count == _draw_count)){  // } || bus_busy) {
                taskYIELD();
            }
            bus_busy = true;
            // vTaskDelay(1);
            // anim_mgr->draw(); // 
            drawfunc();
            bus_busy = false;

            // vTaskDelay(1);
            // anim_mgr->updated = false;
            // Serial.printf("n%d l%d d%d\n", now_drawing, _loop_count, _draw_count);

        
        }
        // vTaskDelay(1);

    // }
    vTaskDelete(NULL);
}
#endif

class CollisionsSaver : public Animation {
  public:
    // volatile bool _is_running;
    CollisionsSaver() {}
    virtual void setup() override { reset(); }
    virtual void reset() override {
        for (int i = 0; i <= 1; i++) {
            sp[i].fillScreen(TFT_BLACK);
            sp[i].setTextSize(1);
            sp[i].setTextDatum(textdatum_t::top_left);
        }
        for (std::uint32_t i = 0; i < ball_count; ++i) {
            auto a = &_balls[_loop_count & 1][i];
            a->color = lgfx::color888(100 + (rand() % 155), 100 + (rand() % 155), 100 + (rand() % 155));
            a->x = 0;
            a->y = 0;
            a->dx = (rand() & (3 << SHIFTSIZE)) + 1;
            a->dy = (rand() & (3 << SHIFTSIZE)) + 1;
            a->r = (4 + (i & 0x07)) << SHIFTSIZE;
            a->m = 4 + (i & 0x07);
        }
        _is_running = screensaver;
        _draw_count = 0;
        _loop_count = 0;
        #if defined (CONFIG_IDF_TARGET_ESP32)
            xTaskCreate(taskDraw, "taskDraw", 2048, NULL, 0, NULL);
        #endif
    }
    virtual int update() override {
        // Serial.printf("n%d l%d d%d dl%d\n", now_drawing, _loop_count, _draw_count, _draw_count_last);

        if ((_loop_count == _draw_count) && !now_drawing) round_over = mainfunc();
        // if ((_loop_count == _draw_count) && (_draw_count != _draw_count_last)) round_over = mainfunc();
        // _draw_count_last = _draw_count;
        // #if defined(CONFIG_IDF_TARGET_ESP32)
        // else taskYIELD();
             while (_loop_count != _draw_count) { taskYIELD(); }
        // #else
        //     drawfunc();
        // #endif
        return !round_over;  // not done yet
    }
    virtual void saver_touch(int16_t, int16_t) override{};  // unused
};
class EraserSaver : public Animation {  // draws colorful patterns to exercise
    enum savershapes : int { Wedges, Dots, Rings, Ellipses, Boxes, NumSaverShapes, FocusRing, Ascii };

 private:
    uint32_t sprsize[2];
    int point[2], plast[2], er[2], touchlast[2] = {-1, -1}, touchpoint[2] = {-1, -1};
    int eraser_rad = 14, eraser_rad_min = 9, eraser_rad_max = 26, eraser_velo_min = 4, eraser_velo_max = 10, touch_w_last = 2;
    int erpos[2] = {0, 0}, eraser_velo_sign[2] = {1, 1}, boxsize[2], now = 0;
    int eraser_velo[2] = {random(eraser_velo_max), random(eraser_velo_max)}, shapes_per_run = 5, shapes_done = 0;
    int32_t erpos_max[2];
    uint8_t saver_illicit_prob = 12, penhue = 0, spothue = 255, slowhue = 0;
    float pensat = 200.0;
    uint16_t pencolor = TFT_RED;
    int num_cycles = 3, cycle = 0, boxrad, boxminsize, boxmaxarea = 1500, shape = random(NumSaverShapes);
    static constexpr uint32_t saver_cycletime_us = 24000000;
    Timer saverCycleTimer, pentimer = Timer(700000);
    bool saver_lotto = false;

 public:
    EraserSaver() {}
    virtual void setup() override {
        sprsize[HORZ] = sprwidth;
        sprsize[VERT] = sprheight;
        erpos_max[HORZ] = (int32_t)sprwidth / 2 - eraser_rad;
        erpos_max[VERT] = (int32_t)sprheight / 2 - eraser_rad;
        for (int axis = 0; axis <= 1; axis++) {
            point[axis] = random(sprsize[axis]);
            eraser_velo_sign[axis] = (random(1)) ? 1 : -1;
        }
        reset();
    }
    virtual void reset() override {
        shapes_done = cycle = 0;
        for (int i = 0; i <= 1; i++) {
            sp[i].setTextSize(1);
            sp[i].fillSprite(TFT_BLACK);
            sp[i].setTextDatum(textdatum_t::middle_center);
            sp[i].setTextColor(TFT_BLACK);
            sp[i].setFont(&fonts::Font4);
            sp[i].setCursor(sprsize[HORZ] / 2, sprsize[VERT] / 2);
        }
        change_pattern(-2);  // randomize new pattern whenever turned off and on
        saverCycleTimer.set(saver_cycletime_us);
    }
    virtual void saver_touch(int16_t x, int16_t y)  override {  // you can draw colorful lines on the screensaver
        int tp[2] = {(int32_t)x - (int32_t)corner[HORZ], (int32_t)y - (int32_t)corner[VERT]};
        if (tp[HORZ] < 0 || tp[VERT] < 0) return;
        for (int axis = HORZ; axis <= VERT; axis++)
            if (touchlast[axis] == -1) touchlast[axis] = tp[axis];
        if (pentimer.expireset()) {
            pensat += 1.5;
            if (pensat > 255.0) pensat = 100.0;
            pencolor = (cycle == 1) ? random(0x10000) : hsv_to_rgb<uint16_t>(++penhue, (uint8_t)pensat, 200 + random(56));
        }
        // sp[now].drawWedgeLine(touchlast[HORZ], touchlast[VERT], tp[HORZ],
        // tp[VERT], 4, 4, pencolor, pencolor);  // savtouch_last_w, w, pencolor,
        // pencolor);
        for (int i=-7; i<=8; i++)
            sp[now].drawLine(touchlast[HORZ]+i, touchlast[VERT]+i, tp[HORZ]+i, tp[VERT]+i, pencolor);  // savtouch_last_w, w, pencolor, pencolor);
        for (int axis = HORZ; axis <= VERT; axis++) touchlast[axis] = tp[axis];
    }
    virtual int update() override {
        if (saverCycleTimer.expired()) {
            ++cycle %= num_cycles;
            if (cycle == 2) change_pattern(-1);
            saverCycleTimer.set(saver_cycletime_us / ((cycle == 2) ? 5 : 1));
        }
        drawsprite();
        return shapes_done;
    }
  private:
    void drawsprite() {
        // Serial.printf("\r%d,%d,%d ", shape, shapes_done, cycle);
        for (int axis = 0; axis <= 1; axis++) point[axis] = random(sprsize[axis]);
        if (cycle != 2) {
            spothue--;
            if (!(spothue % 4)) slowhue += random(4);
            if (shape == Wedges) {
                uint16_t c[2] = {
                    hsv_to_rgb<uint16_t>(random(256), 127 + (spothue >> 1)),
                    hsv_to_rgb<uint16_t>(random(256), 127 + (spothue >> 1))
                };
                float im = 0;
                if (plast[VERT] != point[VERT])
                    im = (float)(plast[HORZ] - point[HORZ]) / (float)(plast[VERT] - point[VERT]);
                for (int g = -4; g <= 4; g++) {
                    sp[now].fillCircle(plast[HORZ], plast[VERT], 2, c[0]);
                    if (std::abs(im) > 1.0)
                        sp[now].drawGradientLine(plast[HORZ] + (int)(g / im), plast[VERT] + g, point[HORZ], point[VERT], c[0], c[1]);
                    else sp[now].drawGradientLine(plast[HORZ] + g, plast[VERT] + (int)(g * im), point[HORZ], point[VERT], c[0], c[1]);
                }
            }
            else if (shape == Ellipses) {
                int d[2] = {10 + random(30), 10 + random(30)};
                uint8_t sat = 100 + random(155);
                uint8_t hue = slowhue;
                uint8_t brt = 50 + random(206);
                if (hue > 50 && hue < 150) slowhue++;
                for (int i = 0; i < (3 + random(10)); i++)
                    sp[now].drawEllipse(point[HORZ], point[VERT], d[0] - 2 * i, d[1] + 2 * i, hsv_to_rgb<uint16_t>(hue + 2 * i, sat, brt));
            }
            else if (shape == Rings) {
                int d = 8 + random(25);
                uint16_t c[2] = {hsv_to_rgb<uint16_t>(spothue + 127 * random(1), random(128) + (spothue >> 1), 150 + random(106)), (uint16_t)random(0x10000)};
                for (int r = d; r >= (d - 4); r--) sp[now].drawCircle(point[HORZ], point[VERT], r, c[r % 2]);
            }
            else if (shape == Dots)
                for (int star = 0; star < 4; star++)
                    sp[now].fillCircle(random(sprwidth), random(sprheight), 2 + random(2), hsv_to_rgb<uint16_t>((spothue >> 1) * (1 + random(2)), 128 + (slowhue >> 1), 110 + random(146)));  // hue_to_rgb16(random(255)), TFT_BLACK);
            else if (shape == Ascii)
                for (int star = 0; star < (shape * 5); star++) {
                    sp[now].setTextColor(hsv_to_rgb<uint16_t>(plast[HORZ] + plast[VERT] + (spothue >> 2), 63 + (spothue >> 1), 200 + random(56)), TFT_BLACK);
                    char letter = (char)(1 + random(0xbe));
                    sp[now].setCursor(point[HORZ], point[VERT]);
                    sp[now].print((String)letter);
                }
            else if (shape == Boxes) {
                boxrad = 5 + random(5);
                boxminsize = 2 * boxrad + 10;
                int longer = random(2);
                boxsize[longer] = boxminsize + random(sprwidth - boxminsize);
                boxsize[!longer] = boxminsize + random(smax(0, boxmaxarea / boxsize[longer] - boxminsize));
                for (int dim = 0; dim <= 1; dim++) point[dim] = -boxsize[dim] / 2 + random(sprsize[dim]);
                sp[now].fillSmoothRoundRect(point[HORZ], point[VERT], boxsize[HORZ], boxsize[VERT], boxrad, random(0x10000));  // Change colors as needed
            }
            // else if (shape == FocusRing) {
            // hsv_to_rgb<uint16_t>(random(256), 63+(spothue>>1)+(spothue>>2),
            // 150+random(106)), TFT_BLACK)
            // }
            sp[now].setTextColor(TFT_BLACK);  // allows subliminal messaging
        }
        // if (cycle != 0) {
            for (int axis = HORZ; axis <= VERT; axis++) {
                erpos[axis] += eraser_velo[axis] * eraser_velo_sign[axis];
                if (erpos[axis] * eraser_velo_sign[axis] >= erpos_max[axis]) {
                    // Serial.printf(" w:%3ld h:%3ld l:%3ld m:%3ld ", sprwidth, sprheight, erpos[axis] * eraser_velo_sign[axis], erpos_max[axis]);
                    erpos[axis] = eraser_velo_sign[axis] * erpos_max[axis];
                    eraser_velo[axis] = eraser_velo_min + random(eraser_velo_max - eraser_velo_min);
                    eraser_velo[!axis] = eraser_velo_min + random(eraser_velo_max - eraser_velo_min);
                    eraser_velo_sign[axis] *= -1;
                    eraser_rad = constrain((int)(eraser_rad + random(5) - 2), eraser_rad_min, eraser_rad_max);
                }
            }
            // Serial.printf(" e %3d,%3d,%3d,%3d,%3d\n", (sprwidth / 2) + erpos[HORZ], (sprheight / 2) + erpos[VERT], eraser_rad, eraser_velo[HORZ], eraser_velo[VERT]);
            sp[now].fillCircle((sprwidth / 2) + erpos[HORZ], (sprheight / 2) + erpos[VERT], eraser_rad, 0x0000);
        // }
        if (saver_lotto) sp[now].drawString("do drugs", sprwidth / 2, sprheight / 2);
        for (int axis = HORZ; axis <= VERT; axis++)
            plast[axis] = point[axis];  // erlast[axis] = erpos[axis];
    }
    void change_pattern(
        int newpat = -1) {  // pass non-negative value for a specific pattern, or  -1 for cycle, -2 for random
        if (++shapes_done > 4) shapes_done = 0;
        else {
            int last_pat = shape;
            saver_lotto = !random(saver_illicit_prob);
            if (0 <= newpat && newpat < NumSaverShapes) shape = newpat;  //
            else if (newpat == -1) ++shape %= NumSaverShapes;
            else if (newpat == -2)
                while (last_pat == shape) shape = random(NumSaverShapes);
        }
    }
};
class BlankSaver : public Animation {
  public: 
    BlankSaver() {}
    void setup() {}
    void reset() { for (int i = 0; i <= 1; i++) sp[i].clear(TFT_BLACK); }
    int update() { return true; }
    virtual void saver_touch(int16_t, int16_t) override{};  // unused
};

class AnimationManager {
  private:
    uint32_t cornerx, cornery, sizex, sizey;
    LGFX* mylcd;
    BlankSaver bSaver;
    EraserSaver eSaver;
    CollisionsSaver cSaver;
    Animation* ptrsaver = &cSaver;
    Touchscreen* mytouch;
    Timer saverRefreshTimer = Timer(16666);
    Timer fps_timer;
    float fps = 0.0;
    int64_t fps_mark;
    bool screensaver_last = false;
    void change_saver() {  // pass non-negative value for a specific pattern, -1 for cycle, -2 for random
        ++nowsaver %= NumSaverMenu;
        if (nowsaver == Eraser) ptrsaver = &eSaver;
        else if (nowsaver == Collisions) ptrsaver = &cSaver;
        else ptrsaver = &bSaver;
        // ptrsaver = &(savers[nowsaver]);
        ptrsaver->reset();
    }
  public:
    bool updated;
    int nowsaver = Eraser, still_running = 0;
    AnimationManager(LGFX* _lcd, Touchscreen* touch, uint32_t _cornerx, uint32_t _cornery, uint32_t _sizex, uint32_t _sizey)
        : mylcd(_lcd), mytouch(touch), cornerx(_cornerx), cornery(_cornery), sizex(_sizex), sizey(_sizey) {}
    void setup();
    void reset() { ptrsaver->reset(); }
    void redraw() { diffDraw(); }
    void calc_fps() {
        int64_t now = fps_timer.elapsed();
        fps = (float)(now - fps_mark);
        if (fps > 0.001) fps = 1000000 / fps;
        fps_mark = now;
    }
    float update() {
        if (!screensaver_last && screensaver) {
            // if (nowsaver == Eraser) ptrsaver->reset();  else
            change_saver();  // ptrsaver->reset();
        }
        screensaver_last = screensaver;
        if (!screensaver) return NAN;
        // if (_touch->touched()) ptrsaver->saver_touch(_touch->touch_pt(HORZ), _touch->touch_pt(VERT));
        // // With timer == 16666 drawing dots, avg=8k, peak=17k.  balls, avg 2.7k, peak 9k after 20sec
        // // With max refresh drawing dots, avg=14k, peak=28k.  balls, avg 6k, peak 8k after 20sec
        if (saverRefreshTimer.expireset() || screensaver_max_refresh) {
        //     // Serial.printf("s%d n%d i%d ", screensaver, (nowsaver == Collisions), _is_running);
            mainfunc();
            #if defined (CONFIG_IDF_TARGET_ESP32)
                while (_loop_count != _draw_count) { taskYIELD(); }
            #else
                drawfunc();
            #endif

             calc_fps();
        }
        //     ptrsaver->setflip((nowsaver == Collisions));
        //     still_running = ptrsaver->update();
        //     // if (still_running) updated = true;
        //     if (still_running) {
        //         if (nowsaver == Eraser) diffDraw();
        //         // else if (nowsaver == Collisions) cSaver.mainfunc();
        //     }
        //     else change_saver();
        //     // Serial.printf("u%d\n ", updated);
        // }
        return fps;
    }
    // void draw() {
    //     cSaver.drawfunc();
    // }
};
AnimationManager* anim_mgr;

void AnimationManager::setup() {
    Animation::init(mylcd, mytouch, cornerx, cornery, sizex, sizey);
    eSaver.setup();
    cSaver.setup();
    // xTaskCreate(taskDraw, "taskDraw", 4096, NULL, 0, NULL);
}
