#pragma once
#include <Arduino.h>
// #define LGFX_USE_V1
// #include "lgfxsetup.h"
// // #define CONFIG_IDF_TARGET_ESP32
class Animation {
  public:
    LGFX_Sprite sp[2];
    LGFX_Sprite* sprite;
    LGFX* lcd;
    int corner[2];
    int framesize[2], lcdsize[2];;
    volatile std::uint32_t _draw_count;
    std::size_t flip = 0;

    void setup(LGFX* _lcd, int cornerx, int cornery) {
        lcd = _lcd;
        corner[HORZ] = cornerx;
        corner[VERT] = cornery;
        create_sprites();
        framesize[HORZ] = sp[0].width();
        framesize[VERT] = sp[0].height();
        setflip()
    }
    void setflip() {
        flip = _draw_count & 1;
        sprite = &(sp[flip]);
        sprite->clear();
    }
    void create_sprites() {  // make a sprite to draw on, and another to draw on next. only data that differs will be drawn to screen
        for (int i=0; i<=1; i++) {
            sp[i].setColorDepth(8);  // Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
            sp[i].setTextSize(1);
        }
        bool fail = false;
        auto resx = framesize[HORZ];
        auto resy = framesize[VERT];

        for (std::uint32_t i = 0; !fail && i < 2; ++i) fail = !sp[i].createSprite(resx, resy);
        if (fail) {
            fail = false;
            for (std::uint32_t i = 0; !fail && i < 2; ++i) {
                sp[i].setPsram(true);
                fail = !sp[i].createSprite(resx, resy);
            }
            if (fail) {
                fail = false;
                if (resx > 320) resx = 320;
                if (resy > 240) resy = 240;
                for (std::uint32_t i = 0; !fail && i < 2; ++i) {
                    sp[i].setPsram(true);
                    fail = !sp[i].createSprite(resx, resy);
                }
                if (fail) {
                    lcd->print("createSprite fail...");
                    lgfx::delay(3000);
                }
            }
        }
    }
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

        auto resx = framesize[HORZ];
        auto resy = framesize[VERT];

        // auto width = view_w;
        // auto height = view_h;

        auto w32 = (resx+3) >> 2;
        std::int32_t y = 0;
        do {
            std::int32_t x32 = 0;
            do {
                while (s32[x32] == p32[x32] && ++x32 < w32);
                if (x32 == w32) break;

                std::int32_t xs = x32 << 2;
                while (s[xs] == p[xs]) ++xs;

                while (++x32 < w32 && s32[x32] != p32[x32]);

                std::int32_t xe = (x32 << 2) - 1;
                if (xe >= resx) xe = resx - 1;
                while (s[xe] == p[xe]) --xe;

                lcd->pushImage(xs + corner[HORZ], y + corner[VERT], xe - xs + 1, 1, &s[xs]);
                // lcd->pushImage(xs + corner[HORZ], y + corner[VERT], xe - xs + 1, 1, &s[xs]);
            } while (x32 < w32);
            
            s32 += w32;
            p32 += w32;
        } while (++y < resy);
        
        lcd->display();
    }

};

class CollisionsSaver : public Animation {
  public:
    struct ball_info_t {
        int32_t x, y, dx, dy, r, m;
        uint32_t color;
    };

    static constexpr std::uint32_t SHIFTSIZE = 8;  // 8
    static constexpr std::uint32_t BALL_MAX = 128;  // 256

    ball_info_t _balls[2][BALL_MAX];
    std::uint32_t _ball_count = 0, _fps = 0;
    std::uint32_t ball_count = 0;
    std::uint32_t sec, psec;
    std::uint32_t fps = 0, frame_count = 0;

    volatile bool _is_running;
    volatile std::uint32_t _loop_count;

    CollisionsSaver() {}

    void drawfunc(void) {
        ball_info_t* balls;
        ball_info_t* a;
        LGFX_Sprite* sprite;

        flip = _draw_count & 1;
        balls = &_balls[flip][0];

        sprite = &(sp[flip]);
        sprite->clear();

        for (int32_t i = 8; i < width; i += 16) sprite->drawFastVLine(i, 0, resy, 0x1F);
        for (int32_t i = 8; i < resy; i += 16) sprite->drawFastHLine(0, i, width, 0x1F);
        for (std::uint32_t i = 0; i < _ball_count; i++) {
            a = &balls[i];
            sprite->fillCircle( a->x >> SHIFTSIZE, a->y >> SHIFTSIZE, a->r >> SHIFTSIZE, a->color);
        }

        sprite->setCursor(1,1);
        sprite->setTextColor(TFT_BLACK);
        sprite->printf("obj:%d fps:%d", _ball_count, _fps);
        sprite->setCursor(0,0);
        sprite->setTextColor(TFT_WHITE);
        sprite->printf("obj:%d fps:%d", _ball_count, _fps);
        diffDraw();
        ++_draw_count;
    }

    bool mainfunc(void) {
        bool new_round = false;
        static constexpr float e = 0.999; // Coefficient of friction

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
            a->color = lgfx::color888(100+(rand()%155), 100+(rand()%155), 100+(rand()%155));
            a->x = 0;
            a->y = 0;
            a->dx = (rand() & (3 << SHIFTSIZE)) + 1;
            a->dy = (rand() & (3 << SHIFTSIZE)) + 1;
            a->r = (4 + (ball_count & 0x07)) << SHIFTSIZE;
            a->m =  4 + (ball_count & 0x07);
            #if defined (ESP32) || defined (CONFIG_IDF_TARGET_ESP32) || defined (ESP_PLATFORM)
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
                if (a->dx < 0) a->dx = - a->dx*e;
            }
            else if (a->x >= lcdsize[HORZ] - a->r) {
                a->x = lcdsize[HORZ] - a->r -1;
                if (a->dx > 0) a->dx = - a->dx*e;
            }
            a->y += a->dy;
            if (a->y < a->r) {
                a->y = a->r;
                if (a->dy < 0) a->dy = - a->dy*e;
            }
            else if (a->y >= _resy - a->r) {
                a->y = _height - a->r -1;
                if (a->dy > 0) a->dy = - a->dy*e;
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
                float bdx = - e * (bmx - amx) + adx;
                float ady = (a->m * amy + b->m * bmy + bmy * e * b->m - amy * e * b->m) / (a->m + b->m);
                float bdy = - e * (bmy - amy) + ady;

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

    #if defined (ESP32) || defined (CONFIG_IDF_TARGET_ESP32) || defined (ESP_PLATFORM)
        void taskDraw(void*) {
            while ( _is_running ) {
                while (_loop_count == _draw_count) { taskYIELD(); }
                drawfunc();
            }
            vTaskDelete(NULL);
        }
    #endif

    void setup(LGFX* ptr_lcd, int cornerx, int cornery) {
        Animation.setup(ptr_lcd, int cornerx, int cornery);
        // lcd->begin();
        // lcd->startWrite();
        // lcd->setColorDepth(8);
        // if (lcd->width() < lcd->height()) lcd->setRotation(lcd->getRotation() ^ 1);

        auto lcdsize[HORZ] = sp[0].width();  // lcd->width();
        auto height = sp[0].height();  // lcd->height();

        for (std::uint32_t i = 0; i < 2; ++i) {
            sp[i].setTextSize(1);
            sp[i].setColorDepth(8);
        }
        // <create_sprites();>
        lcdsize[HORZ] = _wid << SHIFTSIZE;
        _height = _height << SHIFTSIZE;
        reset();
    }
    void reset() {
        for (std::uint32_t i = 0; i < ball_count; ++i) {
            auto a = &_balls[_loop_count & 1][i];
            a->color = lgfx::color888(100+(rand()%155), 100+(rand()%155), 100+(rand()%155));
            a->x = 0;
            a->y = 0;
            a->dx = (rand() & (3 << SHIFTSIZE)) + 1;
            a->dy = (rand() & (3 << SHIFTSIZE)) + 1;
            a->r = (4 + (i & 0x07)) << SHIFTSIZE;
            a->m =  4 + (i & 0x07);
        }
        _is_running = true;
        _draw_count = 0;
        _loop_count = 0;

        #if defined (CONFIG_IDF_TARGET_ESP32)
            xTaskCreate(taskDraw, "taskDraw", 2048, NULL, 0, NULL);
        #endif
    }

    int update(void) {
        bool round_over = mainfunc();
        #if defined (CONFIG_IDF_TARGET_ESP32)
        while (_loop_count != _draw_count) { taskYIELD(); }
        #else
        drawfunc();
        #endif
        return round_over;  // not done yet
    }
};
class LibDrawDemo : public Animation {  // draws colorful patterns to exercise screen draw capabilities
  public:
    enum savermenu : int { Eraser, Collisions, NumSaverMenu };
    enum savershapes : int { Wedges, Dots, Rings, Ellipses, Boxes, NumSaverShapes, FocusRing, Ascii };
  private:
    CollisionsSaver collisions;
    TouchScreen* touch;
    int point[2], plast[2], er[2], touchlast[2] = { -1, -1 }, touchpoint[2] = { -1, -1 };
    int eraser_rad = 14, eraser_rad_min = 9, eraser_rad_max = 26, eraser_velo_min = 4, eraser_velo_max = 10, touch_w_last = 2;
    int erpos[2] = { 0, 0 }, eraser_velo_sign[2] = { 1, 1 }, boxsize[2], now = 0, savermenu = random(NumSaverMenu);
    int eraser_velo[2] = { random(eraser_velo_max), random(eraser_velo_max) }, shapes_per_run = 5, shapes_done = 0;
    int erpos_max[2] = { resx / 2 - eraser_rad, resy / 2 - eraser_rad }; 
    uint8_t saver_illicit_prob = 12, penhue = 0, spothue = 255, slowhue = 0;
    float pensat = 200.0;
    uint16_t pencolor = TFT_RED;
    int num_cycles = 3, cycle = 0, boxrad, boxminsize, boxmaxarea = 1500, shape = random(NumSaverShapes);
    static constexpr uint32_t saver_cycletime_us = 34000000;
    Timer saverRefreshTimer = Timer(45000), saverCycleTimer, pentimer = Timer(700000);
    bool saver_lotto = false, screensaver_last = false, done_yet = false;
  public:
    LibDrawDemo() {}
    void setup(LGFX* ptr_lcd, TouchScreen* arg_touch, int argcornerx, int argcornery) {
        Animation.setup(ptr_lcd, int argcornerx, int argcornery);
        touch = arg_touch;
        collisions.setup();
        for (int axis=0; axis<=1; axis++) {
            point[axis] = random(res[axis]);
            eraser_velo_sign[axis] = (random(1)) ? 1 : -1;
        }
        saverCycleTimer.set(saver_cycletime_us);
    }
    void saver_reset() {
        sp[now].fillSprite(TFT_BLACK);
        saver_pattern(-2);  // randomize new pattern whenever turned off and on
        cycle = 0;
        saverCycleTimer.reset();
    }
    void saver_touch(int16_t x, int16_t y) {  // you can draw colorful lines on the screensaver
        int tp[2] = { x - corner[HORZ], y - corner[VERT] };
        if (tp[HORZ] < 0 || tp[VERT] < 0) return;
        for (int axis=HORZ; axis<=VERT; axis++) if (touchlast[axis] == -1) touchlast[axis] = tp[axis];
        if (pentimer.expireset()) {
            pensat += 1.5;
            if (pensat > 255.0) pensat = 100.0;
            pencolor = (cycle == 1) ? random(0x10000) : hsv_to_rgb<uint16_t>(++penhue, (uint8_t)pensat, 200+random(56));
        }
        // sp[now].drawWedgeLine(touchlast[HORZ], touchlast[VERT], tp[HORZ], tp[VERT], 4, 4, pencolor, pencolor);  // savtouch_last_w, w, pencolor, pencolor);
        sp[now].drawLine(touchlast[HORZ], touchlast[VERT], tp[HORZ], tp[VERT], pencolor);  // savtouch_last_w, w, pencolor, pencolor);
        for (int axis=HORZ; axis<=VERT; axis++) touchlast[axis] = tp[axis];
    }
    void update() {
        if (savermenu == Eraser && touch->touched()) saver_touch(touch->touch_pt(HORZ), touch->touch_pt(VERT));
        if (!screensaver_last && screensaver) saver_reset();
        screensaver_last = screensaver;
        if (!screensaver) return;
        if (savermenu == Collisions) run_collisions();
        else run_eraser();
    }
    void run_collisions() {
        done_yet = collisions.update();
        if (done_yet) saver_pattern();
    }
    void run_eraser() {
        if (saverCycleTimer.expired()) {
            ++cycle %= num_cycles;
            if (cycle == 2) saver_pattern(-1);
            saverCycleTimer.set(saver_cycletime_us / ((cycle == 2) ? 5 : 1));
        }
        else if (saverRefreshTimer.expireset()) {
            setflip();
            for (int axis=0; axis<=1; axis++) point[axis] = random(res[axis]);
            if (cycle != 2) {
                spothue--;
                if (!(spothue % 4)) slowhue++;
                if (shape == Wedges) {
                    uint16_t c[2] = { hsv_to_rgb<uint16_t>(random(256), 127+(spothue>>1)), hsv_to_rgb<uint16_t>(random(256), 127+(spothue>>1)) };
                    float im = 0;
                    if (plast[VERT] != point[VERT]) im = (float)(plast[HORZ]-point[HORZ]) / (float)(plast[VERT]-point[VERT]);
                    for (int g=-4; g<=4; g++) {
                        sp[now].fillCircle(plast[HORZ], plast[VERT], 2, c[0]);
                        if (std::abs(im) > 1.0) sp[now].drawGradientLine(plast[HORZ]+(int)(g/im), plast[VERT]+g, point[HORZ], point[VERT], c[0], c[1]);
                        else sp[now].drawGradientLine(plast[HORZ]+g, plast[VERT]+(int)(g*im), point[HORZ], point[VERT], c[0], c[1]);
                    }                                        
                }
                else if (shape == Ellipses) {
                    int d[2] = { 10+random(30), 10+random(30) };
                    uint8_t sat = 100+random(155);
                    uint8_t hue = slowhue;
                    uint8_t brt = 50+random(206);
                    if (hue > 50 && hue < 150) slowhue++;
                    for (int i=0; i<(3+random(10)); i++) sp[now].drawEllipse(point[HORZ], point[VERT], d[0] - 2*i, d[1] + 2*i, hsv_to_rgb<uint16_t>(hue+2*i, sat, brt));
                }
                else if (shape == Rings) {
                    int d = 8 + random(25);
                    uint16_t c = hsv_to_rgb<uint16_t>(spothue+127*random(1), random(128)+(spothue>>1), 150+random(106));
                    for (int r=d; r>=(d-4); r--) sp[now].drawCircle(point[HORZ], point[VERT], r, c);
                }
                else if (shape == Dots) 
                    for (int star=0; star<(shape*5); star++) 
                        sp[now].fillCircle(random(resx), random(resy), 2+random(3), hsv_to_rgb<uint16_t>((spothue>>1)*(1+random(2)), 255, 210+random(46)));  // hue_to_rgb16(random(255)), TFT_BLACK);
                else if (shape == Ascii)
                    for (int star=0; star<(shape*5); star++) {                
                        sp[now].setTextColor(hsv_to_rgb<uint16_t>(plast[HORZ] + plast[VERT] + (spothue>>2), 63+(spothue>>1), 200+random(56)), TFT_BLACK);
                        char letter = (char)(1 + random(0xbe));
                        sp[now].setCursor(point[HORZ], point[VERT]);
                        sp[now].print((String)letter);
                    }
                else if (shape == Boxes) {
                    boxrad = 5 + random(5);
                    boxminsize = 2 * boxrad + 10;
                    int longer = random(2);
                    boxsize[longer] = boxminsize + random(resx - boxminsize);
                    boxsize[!longer] = boxminsize + random(smax(0, boxmaxarea / boxsize[longer] - boxminsize));
                    for (int dim=0; dim<=1; dim++) point[dim] = -boxsize[dim] / 2 + random(res[dim]);
                    sp[now].fillSmoothRoundRect(point[HORZ], point[VERT], boxsize[HORZ], boxsize[VERT], boxrad, random(0x10000)); // Change colors as needed                    
                }
                // else if (shape == FocusRing) {
                    // hsv_to_rgb<uint16_t>(random(256), 63+(spothue>>1)+(spothue>>2), 150+random(106)), TFT_BLACK)
                // }
                sp[now].setTextColor(TFT_BLACK);  // allows subliminal messaging
            }
            if (cycle != 0) {
                for (int axis=HORZ; axis<=VERT; axis++) {
                    erpos[axis] += eraser_velo[axis] * eraser_velo_sign[axis];
                    if (erpos[axis] * eraser_velo_sign[axis] >= erpos_max[axis]) {
                        erpos[axis] = eraser_velo_sign[axis] * erpos_max[axis];
                        eraser_velo[axis] = eraser_velo_min + random(eraser_velo_max - eraser_velo_min);
                        eraser_velo[!axis] = eraser_velo_min + random(eraser_velo_max - eraser_velo_min);
                        eraser_velo_sign[axis] *= -1;
                        eraser_rad = constrain((int)(eraser_rad + random(5) - 2), eraser_rad_min, eraser_rad_max);
                    }
                }
                sp[now].fillCircle((resx/2) + erpos[HORZ], (resy/2) + erpos[VERT], eraser_rad, TFT_BLACK);
            }
            if (saver_lotto) sp[now].drawString("do drugs", resx/2, resy/2);
            for (int axis=HORZ; axis<=VERT; axis++) plast[axis] = point[axis];  // erlast[axis] = erpos[axis];
            push();
        }
    }
    void push() {
        yield();
        sp[now].pushSprite(corner[HORZ], corner[VERT]);
    }
  private:
    void eraser_init() {
        sp[now].fillSprite(TFT_BLACK);
        sp[now].setTextDatum(textdatum_t::middle_center);
        sp[now].setTextColor(TFT_BLACK); 
        sp[now].setFont(&fonts::Font4);
        sp[now].setCursor(resx/2, resy/2);
    }
    void saver_pattern(int newpat=-1) {  // pass non-negative value for a specific pattern, or -1 for cycle, -2 for random
        if (savermenu == Eraser) {
            if (shapes_done > 5) {
                shapes_done = 0;
                done_yet = false;
                savermenu = Collisions;
                collisions.reset();
            }
            else {
                int last_pat = shape;
                saver_lotto = !random(saver_illicit_prob);
                if (0 <= newpat && newpat < NumSaverShapes) shape = newpat;  // 
                else if (newpat == -1) ++shape %= NumSaverShapes;
                else if (newpat == -2) while (last_pat == shape) shape = random(NumSaverShapes);
                shapes_done++;
            }
        }
        else {
            savermenu = Eraser;
            eraser_init();
        }
    }
};