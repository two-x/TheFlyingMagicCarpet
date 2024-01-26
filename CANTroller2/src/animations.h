#pragma once
#include <Arduino.h>
// #define CONFIG_IDF_TARGET_ESP32
volatile bool _is_running;
// volatile std::uint32_t _draw_count;
volatile std::uint32_t _loop_count;
static constexpr std::uint32_t SHIFTSIZE = 8;
volatile bool flip = 0;
volatile int32_t refresh_limit = 10000; // 16666; // = 60 Hz
Timer screenRefreshTimer = Timer((int64_t)refresh_limit);


// volatile int PushSp = 1;
// volatile int DrawSp = 0;
// volatile bool pushed[2];
// volatile bool drawn[2];

class FlexPanel {
  public:
    LGFX_Sprite sp[2];
    LGFX_Sprite* nowspr;
    LGFX* lcd;
    int touchp[2];
    int corner[2], sprsize[2];
    Touchscreen* _touch;
    // std::size_t flip = 0;
    std::uint32_t sec, psec, _width, _height, _myfps = 0, myfps = 0, frame_count = 0;
    FlexPanel() {}
    void init(LGFX* _lcd, Touchscreen* touch, int _cornerx, int _cornery, int _sprwidth, int _sprheight) {
        lcd = _lcd;
        _touch = touch;
        corner[HORZ] = _cornerx;
        corner[VERT] = _cornery;
        sprsize[HORZ] = _sprwidth;
        sprsize[VERT] = _sprheight;
        Serial.printf("  multi purpose panel init.. ");
        lcd->startWrite();
        lcd->setColorDepth(16);
        if (lcd->width() < lcd->height()) lcd->setRotation(lcd->getRotation() ^ 1);
        for (int i = 0; i <= 1; i++) sp[i].setColorDepth(8);  // Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
        auto framewidth = sprsize[HORZ];
        auto frameheight = sprsize[VERT];
        bool fail = false;
        bool using_psram = false;
        for (std::uint32_t i = 0; !fail && i < 2; ++i) {
            sp[i].setPsram(false);
            fail = !sp[i].createSprite(framewidth, frameheight);
        }
        if (fail) {
            fail = false;
            for (std::uint32_t i = 0; !fail && i < 2; ++i) {
                sp[i].setPsram(true);
                fail = !sp[i].createSprite(framewidth, frameheight);
            }
            if (fail) {
                fail = false;
                if (framewidth >= 320) framewidth = 180;
                if (frameheight >= 240) frameheight = 180;
                for (std::uint32_t i = 0; !fail && i < 2; ++i) {
                    fail = !sp[i].createSprite(framewidth, frameheight);
                }
                if (fail) {
                    lcd->print("createSprite fail\n");
                    // lgfx::delay(3000);
                }
                else using_psram = true;
            }
            else using_psram = true;
        }
        Serial.printf(" made 2x %dx%d sprites in %sram\n", framewidth, frameheight, using_psram ? "ps" : "native ");
        for (int i=0; i<=1; i++) sp[i].clear();
        // sp[0].pushImageDMA() draw(corner[HORZ], corner[VERT]);
        // lcd->display();
        lcd->endWrite();
        _width = framewidth << SHIFTSIZE;
        _height = frameheight << SHIFTSIZE;
    }
    // int setflip(bool clear) {  // clear=true blacks out the sprite before drawing on it
    //     bool flipit = false;
    //     if (drawn[flip] && pushed[!flip]) flipit = true;
    //     if (flipit) flip = !flip;
    //     nowspr = &(sp[flip]);
    //     if (flipit && clear) nowspr->clear();
    //     return flip;
    // }
    void diffpush(LGFX_Sprite* source, LGFX_Sprite* ref) {
        union {  // source
            std::uint32_t* s32;
            std::uint8_t* s;
        };
        union {  // reference
            std::uint32_t* r32;
            std::uint8_t* r;
        };
        s32 = (std::uint32_t*)source->getBuffer();
        r32 = (std::uint32_t*)ref->getBuffer();
        auto sprwidth = source->width();
        auto sprheight = source->height();
        auto w32 = (sprwidth + 3) >> 2;
        std::int32_t y = 0;
        lcd->startWrite();
        do {
            std::int32_t x32 = 0;
            do {
                while (s32[x32] == r32[x32] && ++x32 < w32);
                if (x32 == w32) break;
                std::int32_t xs = x32 << 2;
                while (s[xs] == r[xs]) ++xs;
                while (++x32 < w32 && s32[x32] != r32[x32]);
                std::int32_t xe = (x32 << 2) - 1;
                if (xe >= sprwidth) xe = sprwidth - 1;
                while (s[xe] == r[xe]) --xe;
                lcd->pushImageDMA(xs + corner[HORZ], y + corner[VERT], xe - xs + 1, 1, &s[xs]);
                memcpy(&r[xs], &s[xs], sizeof(s[0])*(xe - xs + 1));
            } while (x32 < w32);
            s32 += w32;
            r32 += w32;
        } while (++y < sprheight);
        // lcd->display();
        lcd->endWrite();
        // pushed[flip] = true;
        // drawn[!flip] = false;
    }
    bool touched() {
        if (_touch->touched()) {
            for (int axis=HORZ; axis<=VERT; axis++)
                touchp[axis] = _touch->touch_pt(axis) - corner[axis];
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
        uint32_t color;
    };
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
    uint8_t ball_radius_base = 7;  // originally 4
    uint8_t ball_radius_modifier = 4;  // originally 4
    uint8_t ball_redoubler_rate = 25;  // originally 0x07
    uint8_t ball_gravity = 16;  // originally 0 with suggestion of 4
    volatile bool _is_running;
    volatile std::uint32_t _loop_count = 0;
    CollisionsSaver() {}
    void drawfunc() {
        auto sprwidth = sprite->width();
        auto sprheight = sprite->height();
        // flip = _draw_count & 1;
        balls = &_balls[flip][0];
        // sprite = &(sp[flip]);
        sprite->clear();
        for (float i = 0.125; i < 1.0; i += 0.125) sprite->drawGradientVLine((int)(i * (sprwidth-1)), 0, sprheight, hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200), hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200));
        for (float i = 0.125; i < 1.0; i += 0.125) sprite->drawGradientHLine(0, (int)(i * (sprheight-1)), sprwidth, hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200), hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200));
        for (float i = 0.0; i <= 1.0; i += 1.0) sprite->drawGradientVLine((int)(i * (sprwidth-1)), 0, sprheight, hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200), hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200));
        for (float i = 0.0; i <= 1.0; i += 1.0) sprite->drawGradientHLine(0, (int)(i * (sprheight-1)), sprwidth, hsv_to_rgb<uint16_t>((uint16_t)(i * 65535)+25*_loop_count, 255, 200), hsv_to_rgb<uint16_t>((uint16_t)((1.0-i) * 65535)+25*_loop_count, 255, 200));
        for (std::uint32_t i = 0; i < _ball_count; i++) {
            a = &balls[i];
            sprite->fillCircle(a->x >> SHIFTSIZE, a->y >> SHIFTSIZE, a->r >> SHIFTSIZE, a->color);
        }
        // _draw_count++;
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
            auto a = &_balls[_loop_count & 1][ball_count - 1];
            a->color = lgfx::color888(100 + (rand() % 155), 100 + (rand() % 155), 100 + (rand() % 155));
            a->x = _width * random(2);
            a->y = 0;
            a->dx = (rand() & (5 << SHIFTSIZE)) + 1;  // was (3 << SHIFTSIZE)) for slower balls
            a->dy = (rand() & (5 << SHIFTSIZE)) + 1;  // was (3 << SHIFTSIZE)) for slower balls
            uint8_t sqrme = ball_radius_base + random(ball_radius_modifier);
            for (int i=0; i<=2; i++) if (!random(ball_redoubler_rate)) sqrme *= 2;
            a->r = sqrme << SHIFTSIZE;  // (sqrme * sqrme)));
            a->m = 4 + (ball_count & 0x07);
            // vTaskDelay(1);
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

    void setup(LGFX_Sprite* _nowspr) {
        sprite = _nowspr;
        _width = sprite->width() << SHIFTSIZE;
        _height = sprite->height() << SHIFTSIZE;
        // reset();
    }
    void reset(LGFX_Sprite* sp0, LGFX_Sprite* sp1) {
        LGFX_Sprite* spp[2] = { sp0, sp1 };
        for (int i = 0; i <= 1; i++) {
            spp[i]->setBaseColor(TFT_BLACK);
            spp[i]->clear();
            spp[i]->setTextSize(1);
            spp[i]->setTextDatum(textdatum_t::top_left);
        }
        ball_thismax = BALL_MAX - random(25);
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
        refresh_limit = 20000;  // 50 Hz limit
        screenRefreshTimer.set(refresh_limit);
        _is_running = true;
        // _draw_count = 0;
        // _loop_count = 0;
        #if defined(CONFIG_IDF_TARGET_ESP32)
            xTaskCreate(taskDraw, "taskDraw", 2048, NULL, 0, NULL);
        #endif
    }
    int update(LGFX_Sprite* _nowspr) {
        sprite = _nowspr;
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
    int sprsize[2], rotate = -1;
    int point[2], plast[2], er[2];
    int eraser_rad = 14, eraser_rad_min = 22, eraser_rad_max = 40, eraser_velo_min = 3, eraser_velo_max = 7, touch_w_last = 2;
    int erpos[2] = {0, 0}, eraser_velo_sign[2] = {1, 1}, boxsize[2], now = 0;
    int eraser_velo[2] = {random(eraser_velo_max), random(eraser_velo_max)}, shapes_per_run = 5, shapes_done = 0;
    int erpos_max[2];
    uint8_t saver_illicit_prob = 12;
    float pensat = 200.0;
    uint16_t pencolor = TFT_RED, wclast, spothue = 65535, slowhue = 0, penhue;
    int num_cycles = 3, cycle = 0, boxrad, boxminsize, boxmaxarea = 1200, shape = random(Rotate);
    static constexpr uint32_t saver_cycletime_us = 18000000;
    Timer saverCycleTimer, pentimer = Timer(1500000);
    bool saver_lotto = false, has_eraser = true;
 public:
    EraserSaver() {}
    void setup(LGFX_Sprite* _nowspr) {
        sprite = _nowspr;
        sprsize[HORZ] = sprite->width();
        sprsize[VERT] = sprite->height();
        erpos_max[HORZ] = (int32_t)sprsize[HORZ] / 2 - eraser_rad;
        erpos_max[VERT] = (int32_t)sprsize[VERT] / 2 - eraser_rad;
        for (int axis = 0; axis <= 1; axis++) {
            point[axis] = random(sprsize[axis]);
            eraser_velo_sign[axis] = (random(1)) ? 1 : -1;
        }
        // reset();
    }
    void reset(LGFX_Sprite* sp0, LGFX_Sprite* sp1) {
        LGFX_Sprite* spp[2] = { sp0, sp1 };
        shapes_done = cycle = 0;
        for (int i = 0; i <= 1; i++) {
            spp[i]->setBaseColor(TFT_BLACK);
            spp[i]->setTextSize(1);
            spp[i]->fillSprite(TFT_BLACK);
            spp[i]->setTextDatum(textdatum_t::middle_center);
            spp[i]->setTextColor(TFT_BLACK);
            spp[i]->setFont(&fonts::Font4);
            spp[i]->setCursor(sprsize[HORZ] / 2, sprsize[VERT] / 2);
        }
        change_pattern(-2);  // randomize new pattern whenever turned off and on
        saverCycleTimer.set(saver_cycletime_us);
        refresh_limit = 11111;  // 90 Hz limit
        screenRefreshTimer.set(refresh_limit);
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
    //         pencolor = (cycle == 1) ? rando_color() : hsv_to_rgb<uint16_t>(++penhue, (uint8_t)pensat, 200 + random(56));
    //     }
    //     sprite->fillCircle(touchlast[HORZ], touchlast[VERT], 20, pencolor);
    //     // sprite->drawWedgeLine(touchlast[HORZ], touchlast[VERT], tp[HORZ], tp[VERT], 4, 4, pencolor, pencolor);  // savtouch_last_w, w, pencolor, pencolor);
    //     // for (int i=-7; i<=8; i++)
    //     //     sprite->drawLine(touchlast[HORZ]+i, touchlast[VERT]+i, tp[HORZ]+i, tp[VERT]+i, pencolor);  // savtouch_last_w, w, pencolor, pencolor);
    //     for (int axis = HORZ; axis <= VERT; axis++) touchlast[axis] = tp[axis];
    // }
    void saver_touch(int x, int y) {  // you can draw colorful lines on the screensaver
        if (pentimer.expireset()) {
            pensat += 1.5;
            if (pensat > 255.0) pensat = 100.0;
            pencolor = (cycle == 1) ? rando_color() : hsv_to_rgb<uint16_t>(++penhue, (uint8_t)pensat, 200 + random(56));
        }
        sprite->fillCircle(x, y, 20, pencolor);
    }
    int update(LGFX_Sprite* _nowspr) {
        sprite = _nowspr;
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
        for (int axis = 0; axis <= 1; axis++) point[axis] = random(sprsize[axis]);
        if (shape == Rotate) ++rotate %= Ascii;
        else rotate = shape;
        if ((cycle != 2) || !has_eraser) {
            spothue -= 10;
            if (!random(20)) spothue = random(65535);
            if (spothue & 1) slowhue += 13;
            if (rotate == Wedges) {
                uint16_t wc = hsv_to_rgb<uint16_t>(random(65536), 127 + (spothue >> 9));
                float im = 0;
                if (plast[VERT] != point[VERT]) im = (float)(plast[HORZ] - point[HORZ]) / (float)(plast[VERT] - point[VERT]);
                sprite->fillCircle(plast[HORZ], plast[VERT], 3, wc);
                // sprite->drawCircle(point[HORZ], point[VERT], 3, TFT_BLACK);
                for (int h=-4; h<=4; h++)
                    sprite->drawGradientLine(point[HORZ], point[VERT], plast[HORZ] + (int)(h / ((std::abs(im) > 1.0) ? im : 1)), plast[VERT] + (int)(h * ((std::abs(im) > 1.0) ? 1 : im)), wc, wclast);
                wclast = wc;
            }
            else if (rotate == Ellipses) {
                int d[2] = {10 + random(30), 10 + random(30)};
                uint8_t sat, brt, mult = random(4);
                for (int i = 0; i < 6 + random(20); i++) {
                    sat = 100 + random(156);
                    brt = 90 + random(166);
                    sprite->drawEllipse(point[HORZ], point[VERT], d[0] - i, d[1] + i, hsv_to_rgb<uint16_t>(spothue + ((mult * i) >> 1), sat, brt));
                }
            }
            else if (rotate == Rings) {
                int d = 8 + random(25);
                uint16_t hue = spothue + 32768 * random(1);
                uint8_t sat = random(128) + (spothue >> 9);
                uint8_t brt = 180 + random(76);
                uint8_t c = hsv_to_rgb<uint8_t>(hue, sat, brt);
                uint8_t c2 = hsv_to_rgb<uint8_t>(hue, sat, brt-10);
                // Serial.printf("%3.0f%3.0f%3.0f (%3.0f%3.0f%3.0f) (%3.0f%3.0f%3.0f)\n", (float)(hue/655.35), (float)(sat/2.56), (float)(brt/2.56), 100*(float)((c >> 11) & 0x1f)/(float)0x1f, 100*(float)((c >> 5) & 0x3f)/(float)0x3f, 100*(float)(c & 0x1f)/(float)0x1f, 100*(float)((c2 >> 11) & 0x1f)/(float)0x1f, 100*(float)((c2 >> 5) & 0x3f)/(float)0x3f, 100*(float)(c2 & 0x1f)/(float)0x1f);
                for (int xo = -1; xo <= 1; xo += 2) {
                    sprite->drawCircle(point[HORZ], point[VERT] + xo, d, c);
                    sprite->drawCircle(point[HORZ] + xo, point[VERT], d, c);
                }
                for (int edge = -1; edge <= 1; edge += 2)
                    sprite->drawCircle(point[HORZ], point[VERT], d + edge, c2);
            }
            else if (rotate == Dots)
                for (int star = 0; star < 12; star++)
                    sprite->fillCircle(random(sprsize[HORZ]), random(sprsize[VERT]), 2 + random(2), hsv_to_rgb<uint8_t>((uint16_t)((spothue >> 1) * (1 + random(2))), 128 + random(128), 160 + random(96)));  // hue_to_rgb16(random(255)), TFT_BLACK);
            else if (rotate == Boxes) {
                boxrad = 2 + random(2);
                boxminsize = 2 * boxrad + 10;
                int longer = random(2);
                boxsize[longer] = boxminsize + random(sprsize[HORZ] - boxminsize);
                boxsize[!longer] = boxminsize + random(smax(0, boxmaxarea / boxsize[longer] - boxminsize));
                for (int dim = 0; dim <= 1; dim++) point[dim] = -boxsize[dim] / 2 + random(sprsize[dim]);
                sprite->fillSmoothRoundRect(point[HORZ], point[VERT], boxsize[HORZ], boxsize[VERT], boxrad, rando_color());  // Change colors as needed
            }
            else if (rotate == Ascii)
                for (int star = 0; star < 4; star++) {
                    point[HORZ] = random(sprite->width());
                    point[VERT] = random(sprite->height());
                    uint16_t hue = point[VERT] * 65535 / sprite->height();
                    uint8_t sat = point[HORZ] * 255 / sprite->width();
                    String letter = (String)((char)(0x21 + random(0x5d)));
                    uint16_t c = hsv_to_rgb<uint16_t>(hue, sat, 100 + 100 * (spothue > 32767) + random(56));
                    sprite->drawString(letter, point[HORZ]+1, point[VERT]+1);
                    sprite->drawString(letter, point[HORZ]-1, point[VERT]-1);
                    sprite->setTextColor(hsv_to_rgb<uint16_t>(hue, sat, 100 + 100 * (spothue > 32767) + random(56)));
                    sprite->drawString(letter, point[HORZ], point[VERT]);
                    sprite->setTextColor(TFT_BLACK);  // allows subliminal messaging
                }
        }
        if ((cycle != 0) && has_eraser) {
            for (int axis = HORZ; axis <= VERT; axis++) {
                erpos[axis] += eraser_velo[axis] * eraser_velo_sign[axis];
                if (erpos[axis] * eraser_velo_sign[axis] >= erpos_max[axis]) {
                    // Serial.printf(" w:%3ld h:%3ld l:%3ld m:%3ld ", sprsize[HORZ], sprsize[VERT], erpos[axis] * eraser_velo_sign[axis], erpos_max[axis]);
                    erpos[axis] = eraser_velo_sign[axis] * erpos_max[axis];
                    eraser_velo[axis] = eraser_velo_min + random(eraser_velo_max - eraser_velo_min);
                    eraser_velo[!axis] = eraser_velo_min + random(eraser_velo_max - eraser_velo_min);
                    eraser_velo_sign[axis] *= -1;
                    eraser_rad = constrain((int)(eraser_rad + random(5) - 2), eraser_rad_min, eraser_rad_max);
                }
            }
            // Serial.printf(" e %3d,%3d,%3d,%3d,%3d\n", (sprsize[HORZ] / 2) + erpos[HORZ], (sprsize[VERT] / 2) + erpos[VERT], eraser_rad, eraser_velo[HORZ], eraser_velo[VERT]);
            // sprite->fillCircle((sprsize[HORZ] / 2) + erpos[HORZ], (sprsize[VERT] / 2) + erpos[VERT], 20, pencolor);
            sprite->fillCircle((sprsize[HORZ] / 2) + erpos[HORZ], (sprsize[VERT] / 2) + erpos[VERT], eraser_rad, (uint8_t)TFT_BLACK);
        }
        if (saver_lotto) sprite->drawString("do drugs", sprsize[HORZ] / 2, sprsize[VERT] / 2);
        for (int axis = HORZ; axis <= VERT; axis++) plast[axis] = point[axis];  // erlast[axis] = erpos[axis];
        // _draw_count++;
    }
    void change_pattern(int newpat = -1) {  // pass non-negative value for a specific pattern, or  -1 for cycle, -2 for random
        ++shapes_done %= 5;
        int last_pat = shape;
        saver_lotto = !random(saver_illicit_prob);
        has_eraser = !random(2);
        if (0 <= newpat && newpat < NumSaverShapes) shape = newpat;  //
        else {
            if (newpat == -1) ++shape %= Rotate;
            else if (newpat == -2) while (last_pat == shape) shape = random(Rotate);
            if (!random(25)) shape = Rotate;
        }
        // for (int i = 0; i <= 1; i++)
        //     if (shape == Ascii) sp[i].setFont(&fonts::Font4);
        //     else sp[i].setFont(&fonts::Font4);
    }
};
class AnimationManager {
  private:
    enum saverchoices : int { Eraser, Collisions, NumSaverMenu, Blank };
    int nowsaver = Eraser, still_running = 0;
    LGFX* mylcd;
    LGFX_Sprite* nowspr_ptr;
    FlexPanel* panel;
    EraserSaver eSaver;
    CollisionsSaver cSaver;
    // Timer saverRefreshTimer = Timer(16666);
    Timer fps_timer;
    float myfps = 0.0;
    int64_t fps_mark;
    bool screensaver_last = false;
    void change_saver() {  // pass non-negative value for a specific pattern, -1 for cycle, -2 for random
        ++nowsaver %= NumSaverMenu;
        reset();
    }
  public:
    AnimationManager() {}
    void init(FlexPanel* _panel) {
        panel = _panel;
    }
    void setup() {
        // int flip = panel->setflip(true);
        eSaver.setup(&panel->sp[flip]);
        cSaver.setup(&panel->sp[flip]);
    }
    void reset() {
        // int flip = panel->setflip(true);
        if (nowsaver == Eraser) eSaver.reset(&panel->sp[flip], &panel->sp[!flip]);
        else if (nowsaver == Collisions) cSaver.reset(&panel->sp[flip], &panel->sp[!flip]);
    }
    // void redraw() {
    //     // if (!is_drawing) is_pushing = false;
    //     panel->diffpush(&panel->sp[flip], &panel->sp[!flip]);
    // }
    void calc_fps() {
        int64_t now = fps_timer.elapsed();
        myfps = (float)(now - fps_mark);
        if (myfps > 0.001) myfps = 1000000 / myfps;
        fps_mark = now;
    }
    float update() {
        if (!screensaver_last && screensaver) change_saver();  // ptrsaver->reset();
        Serial.printf(",%d\n", nowsaver);
        screensaver_last = screensaver;
        if (!screensaver) return NAN;        // With timer == 16666 drawing dots, avg=8k, peak=17k.  balls, avg 2.7k, peak 9k after 20sec
        // With max refresh drawing dots, avg=14k, peak=28k.  balls, avg 6k, peak 8k after 20sec
        nowspr_ptr = &(panel->sp[flip]);
        if (nowsaver == Eraser) {
            still_running = eSaver.update(nowspr_ptr);
            if (panel->touched()) eSaver.saver_touch(panel->touch_pt(HORZ), panel->touch_pt(VERT));
        }
        else if (nowsaver == Collisions) still_running = cSaver.update(nowspr_ptr);
        if (!still_running) change_saver();
        calc_fps();
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
    void redraw() {
        panel->diffpush(&panel->sp[flip], &panel->sp[!flip]);
    }
    void add_errorline(std::string type, std::string item) {
        std::string newerr = type + ": " + item;
        if (newerr.length() > 15) newerr = newerr.substr(0, 15);
        textlines[usedlines++] = newerr;
    }
    void update() {
        // int flip = panel->setflip(false);
        // nowspr_ptr = &(panel->sp[flip]);
        // panel->diffpush(&panel->sp[flip], &panel->sp[!flip]);
    }
};