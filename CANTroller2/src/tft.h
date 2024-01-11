#pragma once
// tft.h - started life as some examples in the tft_espi library, including one called SpriteRotatingCube.ino
#include <TFT_eSPI.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>

#define USE_SECOND_CORE 1
#define USE_DIFFDRAW 0
#define FULLSCREEN_SPRITES 0
#define disp_width_pix 240
#define disp_height_pix 320
// enum dirs : int { HORZ, VERT };

static constexpr int NumSp = 2;
volatile int DrawSp = 0, RefSp = 1;
#if USE_DIFFDRAW
    volatile int& PushSp = DrawSp;
#else
    volatile int& PushSp = RefSp;
#endif

#if USE_SECOND_CORE
    #include <atomic>
    std::atomic<bool> sprite_locked[NumSp];
    #define LOCK_SPRITE(n) while(sprite_locked[n]) delay(1); \
        sprite_locked[n] = true;
    #define UNLOCK_SPRITE(n) sprite_locked[n] = false;
#else
    #define   LOCK_SPRITE(n)
    #define UNLOCK_SPRITE(n)
#endif
// enum displaysprites : int { DrawSp, RefSp, PushSp, NumSp };
    volatile bool ready_to_push = false;
    volatile int draw_count;
    volatile int push_count;
    int push_wait_us, draw_wait_us;
    volatile bool pushed[NumSp];
    volatile bool drawn[NumSp];
    // int corner[2] = { 0, 0 };
    int tft_w, tft_h;
    uint32_t drawtime, pushtime;
    static constexpr uint32_t color_depth = 16;
    static constexpr int iwidth = 128;  // 128x128 for a 16-bit colour Sprite (32Kbytes RAM)
    static constexpr int iheight = 128;  // Maximum is 181x181 (64Kbytes) for DMA -  restricted by processor design
    TFT_eSPI tft = TFT_eSPI();  // Library instance Declare object "tft"
    // #if USE_DIFFDRAW
    //     TFT_eSprite spr[NumSp] = { TFT_eSprite(&tft), TFT_eSprite(&tft), TFT_eSprite(&tft) };
    // #else
        TFT_eSprite spr[NumSp] = { TFT_eSprite(&tft), TFT_eSprite(&tft) };
    // #endif
    uint16_t* sprPtr[NumSp];  // Pointers to start of Sprites in RAM
        int16_t xpos = 0, ypos = 0;  // Sprite draw position

    uint16_t* CubeSpPtr;
    uint16_t counter = 0;  // Used for fps measuring
    long startMillis = millis();
    uint16_t interval = 100;
    String fps = "0fps";  // Frames per second
    static constexpr int prime_number_processor_load = 0;  // 491;  // 241 = 50% CPU load for 128 * 128 and STM32F466 Nucleo 64, 491 = 50% CPU load for 128 * 128 and STM32F767 Nucleo 144
    int prime_max = 2;  // Prime number initial value
    TFT_eSprite CubeSp{&tft};
    Timer refreshtimer{25000};

class DisplayDriver {
  public:
    // void pushsprites(TFT_eSprite* source, int spx, int spy, int spw, int sph);  // uint_fast8_t flip
    // void pushdiffs(TFT_eSprite* source, int spx, int spy, int spw, int sph);

    DisplayDriver() {}

    // https://en.wikipedia.org/wiki/Direct_memory_access
    // The rotating cube is drawn into a 128 x 128 Sprite and then this is
    // rendered to screen. The Sprite need 32Kbytes of RAM and DMA buffer the same
    // so processors with at least >64Kbytes RAM free will be required.
    // (Tested with Nucleo 64 STM32F446RE and Nucleo 144 STM32F767ZI)
    // STM32F767 27MHz SPI 50% processor load: Non DMA  52 fps, with DMA 101 fps
    // STM32F767 27MHz SPI  0% processor load: Non DMA  97 fps, with DMA 102 fps
    // ESP32     27MHz SPI  0% processor load: Non DMA  90 fps, with DMA 101 fps
    // ESP32     40MHz SPI  0% processor load: Non DMA 127 fps, with DMA 145 fps

    // related to graphics driver
    uint32_t computePrimeNumbers(int32_t n) {  // This is to provide a processing load to see the improvement DMA gives
        if (n<2) return 1;
        int32_t i, fact, j, p = 0;  //Serial.print("\nPrime Numbers are: \n");
        for (i = 1; i <= n; i++) {
            fact = 0;
            for (j = 1; j <= n; j++) if (i % j == 0) fact++;
            if (fact == 2) p = i;  //Serial.print(i); Serial.print(", ");
        }
        return p; // Biggest   //Serial.println();
    }

    void pushsprites(TFT_eSprite* source, int spx, int spy, int spw, int sph) {  // uint_fast8_t flip
        // Method 1 (BouncyBoxSprite example) : draws the next frame on one sprite while pushing the last, swapping back and forth
        push_wait_us = 0;
        while (!drawn[PushSp] || pushed[PushSp]) {
            delayMicroseconds(25);
            push_wait_us += 25;
        }
        // Serial.printf("p%d %d s:%d,%d dp:%d%d,%d%d t:%ld\n", push_wait_us, counter, DrawSp, RefSp, drawn[0], pushed[0], drawn[1], pushed[1], pushtime);
        int64_t begin = esp_timer_get_time();
        // Serial.printf("p%d %d s:%d,%d,%d dp:%d%d,%d%d,%d%d\n", push_wait_us, counter, DrawSp, RefSp, PushSp, drawn[0], pushed[0], drawn[1], pushed[1], drawn[2], pushed[2]);
        LOCK_SPRITE(PushSp);
        // LOCK_SPRITE(RefSp);
        tft.startWrite();  // Start SPI transaction and drop TFT_CS - avoids transaction overhead in loop
        // if (tft.dmaBusy() && prime_number_processor_load) prime_max++; // Increase processing load until just not busy
        // while (tft.dmaBusy() || (draw_count <= push_count)) delayMicroseconds(200);  // Hang out till dma can be used
        // #if FULLSCREEN_SPRITES
        // #else
        // tft.fillScreen(TFT_BLACK);
        // #endif
        tft.pushImageDMA(spx, spy, spw, sph, (uint16_t*)source->getPointer());
        tft.endWrite();
        counter++;
        // UNLOCK_SPRITE(RefSp);
        UNLOCK_SPRITE(PushSp);
        pushed[PushSp] = true;
        drawn[PushSp] = false;
        ++PushSp %= NumSp;
        push_count++;
        pushtime = esp_timer_get_time()-begin;
        // // Method 2 (BouncyCircles example) : draws screen in alternating halves. Modified with option to use 2nd core
        // tft.pushImageDMA(0, flip * disp_height_pix / 2, disp_width_pix, disp_height_pix / 2, sprPtr[flip]);
    }
    void pushdiffs(TFT_eSprite* source, int spx, int spy, int spw, int sph) {
        push_wait_us = 0;
        // while (!drawn[PushSp] || pushed[PushSp]) {
        while (!drawn[PushSp] || pushed[PushSp]) {
            delayMicroseconds(25);
            push_wait_us += 25;
        }
        pushed[PushSp] = false;

        // Serial.printf("p%d %d s:%d,%d dp:%d%d,%d%d t:%ld\n", push_wait_us, counter, DrawSp, RefSp, drawn[0], pushed[0], drawn[1], pushed[1], pushtime);
        int64_t begin = esp_timer_get_time();
        // Serial.printf("p%d s:%d,%d,%d dp:%d%d,%d%d,%d%d\n", push_wait_us, DrawSp, RefSp, PushSp, drawn[0], pushed[0], drawn[1], pushed[1], drawn[2], pushed[2]);
        LOCK_SPRITE(PushSp);
        LOCK_SPRITE(RefSp);
        union {
            std::uint32_t* s32;
            std::uint16_t* s16;
            // std::uint8_t* s8;
        };
        union {
            std::uint32_t* p32;
            std::uint16_t* p16;
            // std::uint8_t* p8;
        };
        s32 = (std::uint32_t*)spr[PushSp].getPointer();  // In lgfx library was    getBuffer();
        p32 = (std::uint32_t*)spr[RefSp].getPointer();
        // p32 = (std::uint32_t*)source->getPointer();
        
        // auto sprwidth = spr[RefSp].width();
        // auto sprheight = spr[RefSp].height();
        // auto destwidth = spr[PushSp].width();
        auto sprwidth = spw;
        auto sprheight = sph;
        auto destwidth = spr[PushSp].width();
        
        auto w32 = (sprwidth + 1) >> 1;  // (color_depth == 8) ? 3 : 1) >> ((color_depth == 8) ? 2 : 1);
        std::int32_t si, y = 0;
        tft.startWrite();  // Start SPI transaction and drop TFT_CS - avoids transaction overhead in loop
        do {
            std::int32_t x32 = 0;
            std::int32_t xs, xsi, xe, xei;
            #if FULLSCREEN_SPRITES
                // si = spx + (spy + y) * destwidth;
                do {
                    while (s32[x32] == p32[x32] && ++x32 < w32);
                    if (x32 == w32) break;
                    xs = x32 << 1;
                    while (s16[xs] == p16[xs]) ++xs;
                    while (++x32 < w32 && s32[x32] != p32[x32]);
                    xe = (x32 << 1) - 1;
                    if (xe >= sprwidth) xe = sprwidth - 1;
                    while (s16[xe] == p16[xe]) --xe;
                    // tft.pushPixelsDMA(&(s[xs]), xe - xs + 1);
                    tft.pushImageDMA(xs, y, xe - xs + 1, 1, &s16[xs]);
                } while (x32 < w32);
            #else
                si = spx + (spy + y) * destwidth;
                do {
                    while (s32[x32] == p32[x32] && ++x32 < w32);
                    if (x32 == w32) break;
                    xs = x32 << 1;
                    xsi = si << 1;
                    while (s16[xsi] == p16[xs]) {
                        ++xs;
                        ++xsi;
                    }
                    while (++x32 < w32 && s32[++si] != p32[x32]);
                    xe = (x32 << 1) - 1;
                    xei = (si << 1) - 1;
                    if (xe >= sprwidth) xe = sprwidth - 1;
                    if (xei >= destwidth) xei = destwidth - 1;
                    while (s16[xe] == p16[xei]) {
                        --xe;
                        --xei;
                    }
                    tft.pushImageDMA(xs + xpos, y + ypos, xe - xs + 1, 1, &s16[xs]);
                } while (x32 < w32);
            #endif
            s32 += w32;
            p32 += w32;
        } while (++y < sprheight);
        // tft.display();
        tft.endWrite();
        counter++;
        UNLOCK_SPRITE(RefSp);
        UNLOCK_SPRITE(PushSp);
        // pushed[PushSp] = true;
        // drawn[RefSp] = false;
        pushed[PushSp] = true;
        drawn[RefSp] = false;
        // ++PushSp %= NumSp;
        ++DrawSp %= NumSp;
        ++RefSp %= NumSp;
        push_count++;
        pushtime = esp_timer_get_time()-begin;
    }
   void setup() {
        #if USE_LGFX_TFT
            return;
        #endif
        // Serial.begin(115200);
        // delay(1000);
        tft.init();
        tft.fillScreen(TFT_BLACK);
        tft.setRotation(2);
        xpos = 0;
        ypos = (tft.height() - iheight) / 2;
        tft_w = tft.width();
        tft_h = tft.height();
        drawn[DrawSp] = false;
        pushed[DrawSp] = true;
        // drawn[DrawSp] = pushed[PushSp] = false;
        // pushed[DrawSp] = drawn[PushSp] = true;
        drawn[RefSp] = true;
        #if USE_DIFFDRAW
            pushed[RefSp] = true;
            // pushed[RefSp] = drawn[RefSp] = true;
        #else
            pushed[PushSp] = false;
        #endif
         #if FULLSCREEN_SPRITES
            for (int i=0; i<NumSp; i++) {
                spr[i].setColorDepth(color_depth);  // Color depth has to be 16 bits if DMA is used to render image
                #if FULLSCREEN_SPRITES
                    spr[i].createSprite(tft_w, tft_h);
                    sprPtr[i] = (uint16_t*)(spr[i].getPointer());
                // #else
                //     sprPtr[i] = (uint16_t*)(spr[i].createSprite(iwidth, iheight));
                #endif
                spr[i].setTextColor(TFT_WHITE);
                spr[i].setTextDatum(MC_DATUM);
            }
        #else
        #endif
        tft.initDMA(); // Initialise the DMA engine (tested with STM32F446 and STM32F767)- should work with ESP32, STM32F2xx/F4xx/F7xx processors  >>>>>> DMA IS FOR SPI DISPLAYS ONLY <<<<<<
        startMillis = millis();  // Animation control timer
    }
    // void update_framebuffer(bool flip) {}
};
DisplayDriver disp;

class SpinnyCube {
  public:
    TFT_eSPI* mytft;
    // TFT_eSprite CubeSp;
    // uint16_t* CubeSpPtr;
    void cube_setup(TFT_eSPI* _tft) {
        mytft = _tft;
        if (random(2)) dx = -1;  // Random movement direction
        if (random(2)) dy = -1;  // Random movement direction
        CubeSpPtr = (uint16_t*)(CubeSp.createSprite(iwidth, iheight));

    }
    SpinnyCube() { 
        #if USE_LGFX_TFT
        #else
        cube_setup(&tft);
        #endif
    }
    // TFT_eSPI* tft;
    static constexpr int cubesize = 358;  // Size of cube image. 358 is max for 128x128 sprite, too big and pixel trails are drawn...
    uint16_t palette[6] = { TFT_WHITE, TFT_GREENYELLOW, TFT_YELLOW, TFT_PINK, TFT_MAGENTA, TFT_CYAN }; // Define the cube face colors
    float d = 15;  // size / 2 of cube edge
    bool spinX = true, spinY = true, spinZ = true;  // 3 axis spin control
    int xmin, ymin, xmax, ymax;  // Min and max of cube edges, "int" type used for compatibility with original sketch min() function
    float px[8] = { -d,  d,  d, -d, -d,  d,  d, -d };
    float py[8] = { -d, -d,  d,  d, -d, -d,  d,  d };
    float pz[8] = { -d, -d, -d, -d,  d,  d,  d,  d };
    float p2x[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // mapped coordinates on screen
    float p2y[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    float r[3] = { 0, 0, 0 };  // rotation angle in radians
    int faces[12][3] = {  // Define the triangles. The order of the vertices MUST be CCW or the shoelace method won't work to detect visible edges
        {0, 1, 4}, {1, 5, 4}, {1, 2, 5}, {2, 6, 5}, {5, 7, 4}, {6, 7, 5},
        {3, 4, 7}, {4, 3, 0}, {0, 3, 1}, {1, 3, 2}, {2, 3, 6}, {6, 3, 7},
    };
    uint32_t updateTime = 0;  // time for next update
    int wait = 0;  //random (20);
    bool bounce = false;
    int dx = 1, dy = 1;

    // Detected visible triangles. If calculated area > 0 the triangle is rendered facing towards the viewer, 
    // since the vertices are CCW. If the area is negative the triangle is CW and thus facing away from us.
    int shoelace(int x1, int y1, int x2, int y2, int x3, int y3) {  // (x1y2 - y1x2) + (x2y3 - y2x3)
        return x1 * y2 - y1 * x2 + x2 * y3 - y2 * x3 + x3 * y1 - y3 * x1;
    }
    void rendercube() { // Rotates and renders the cube.
        double speed = 90;
        if (spinX) r[0] = r[0] + PI / speed; // Add a degree
        if (spinY) r[1] = r[1] + PI / speed; // Add a degree
        if (spinZ) r[2] = r[2] + PI / speed; // Add a degree
        if (r[0] >= 360.0 * PI / 90.0) r[0] = 0;
        if (r[1] >= 360.0 * PI / 90.0) r[1] = 0;
        if (r[2] >= 360.0 * PI / 90.0) r[2] = 0;
        float ax[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        float ay[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        float az[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 8; i++) {  // Calculate all vertices of the cube
            float px2 = px[i];
            float py2 = cos(r[0]) * py[i] - sin(r[0]) * pz[i];
            float pz2 = sin(r[0]) * py[i] + cos(r[0]) * pz[i];
            float px3 = cos(r[1]) * px2 + sin(r[1]) * pz2;
            float py3 = py2;
            float pz3 = -sin(r[1]) * px2 + cos(r[1]) * pz2;
            ax[i] = cos(r[2]) * px3 - sin(r[2]) * py3;
            ay[i] = sin(r[2]) * px3 + cos(r[2]) * py3;
            az[i] = pz3 - 150;
            p2x[i] = iwidth / 2 + ax[i] * cubesize / az[i];
            p2y[i] = iheight / 2 + ay[i] * cubesize / az[i];
        }
        #if FULLSCREEN_SPRITES
            spr[DrawSp].fillSprite(TFT_BLACK);  // Fill the buffer with colour 0 (Black)
            // spr[DrawSp].fillRect(xpos, ypos, iwidth, iheight, TFT_BLACK);  // Fill the buffer with colour 0 (Black)
        
        #else
            CubeSp.fillSprite(TFT_BLACK);  // Fill the buffer with colour 0 (Black)
        #endif
        for (int i = 0; i < 12; i++) {
            if (shoelace(p2x[faces[i][0]], p2y[faces[i][0]], p2x[faces[i][1]], p2y[faces[i][1]], p2x[faces[i][2]], p2y[faces[i][2]]) > 0) {
                int x0 = p2x[faces[i][0]];
                int y0 = p2y[faces[i][0]];
                int x1 = p2x[faces[i][1]];
                int y1 = p2y[faces[i][1]];
                int x2 = p2x[faces[i][2]];
                int y2 = p2y[faces[i][2]];
                xmin = min(xmin, x0);
                ymin = min(ymin, y0);
                xmin = min(xmin, x1);
                ymin = min(ymin, y1);
                xmin = min(xmin, x2);
                ymin = min(ymin, y2);
                xmax = max(xmax, x0);
                ymax = max(ymax, y0);
                xmax = max(xmax, x1);
                ymax = max(ymax, y1);
                xmax = max(xmax, x2);
                ymax = max(ymax, y2);
                #if FULLSCREEN_SPRITES
                    spr[DrawSp].fillTriangle(x0 + xpos, y0 + ypos, x1 + xpos, y1 + ypos, x2 + xpos, y2 + ypos, palette[i / 2]);
                #else
                    CubeSp.fillTriangle(x0, y0, x1, y1, x2, y2, palette[i / 2]);
                #endif
                if (i % 2) {
                    int avX = 0;
                    int avY = 0;
                    for (int v = 0; v < 3; v++) {
                        avX += p2x[faces[i][v]];
                        avY += p2y[faces[i][v]];
                    }
                    avX = avX / 3;
                    avY = avY / 3;
                }
            }
            spr[DrawSp].drawString(fps, 0, 0, 0);
        }
        //spr[flip].drawString(fps, iwidth / 2, iheight / 2, 4);
        //delay(100);
    }

    void drawcubeloop() {
        while (true) {  // Loop forever
        }
    }
    void printmem() {
        size_t freeHeapSize = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        size_t freeHeapSizePS = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        size_t freeHeapSizePSRAM = ESP.getFreePsram();
        size_t largestFreeBlockSize = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        size_t largestFreeBlockSizePSRAM = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        Serial.printf("Free Heap: %uB, %uB (%u)\n", freeHeapSize, freeHeapSizePS, freeHeapSizePSRAM);
        Serial.printf("Largest Block: %uB, %uB\n", largestFreeBlockSize, largestFreeBlockSizePSRAM);
    }
    void drawcube() {  // should not 
        draw_wait_us = 0;
        while ((drawn[DrawSp] || !pushed[DrawSp]) && !refreshtimer.expired()) {
        // while (drawn[DrawSp] || !pushed[DrawSp]) {
            delayMicroseconds(10);
            draw_wait_us += 10;
        }
        refreshtimer.reset();
        drawn[DrawSp] = pushed[DrawSp] = false;
        // Serial.printf("d%d %d s:%d,%d dp:%d%d,%d%d t:%ld\n", draw_wait_us, counter, DrawSp, RefSp, drawn[0], pushed[0], drawn[1], pushed[1], drawtime);
        int64_t begin = esp_timer_get_time();
        // Serial.printf("d%d %d s:%d,%d,%d dp:%d%d,%d%d,%d%d\n", draw_wait_us, counter, DrawSp, RefSp, PushSp, drawn[0], pushed[0], drawn[1], pushed[1], drawn[2], pushed[2]);
        // Serial.printf("draw f%d\n", flip);
        LOCK_SPRITE(DrawSp);
        if (xpos >= mytft->width() - xmax) { bounce = true; dx = -1; }  // Pull it back onto screen if it wanders off
        else if (xpos < -xmin) { bounce = true; dx = 1; }
        if (ypos >= mytft->height() - ymax) { bounce = true; dy = -1; }
        else if (ypos < -ymin) { bounce = true; dy = 1; }
        if (bounce) {  // Randomise spin
            if (random(2)) spinX = true;
            else spinX = false;
            if (random(2)) spinY = true;
            else spinY = false;
            if (random(2)) spinZ = true;
            else spinZ = false;
            bounce = false;
            //wait = random (20);
        }
        // if (updateTime <= millis()) {  // Use time delay so sprtie does not move fast when not all on screen
        updateTime = millis() + wait;
        xmin = iwidth / 2; xmax = iwidth / 2; ymin = iheight / 2; ymax = iheight / 2;
        rendercube();  // draws newest image onto sprite
        UNLOCK_SPRITE(DrawSp);
        // #if USE_SECOND_CORE
        // #else    
        //     #if FULLSCREEN_SPRITES
        //         #if USE_DIFFDRAW
        //             pushdiffs(&spr[RefSp], 0, 0, disp_width_pix, disp_height_pix);
        //         #else
        //             pushsprites(&spr[PushSp], 0, 0, disp_width_pix, disp_height_pix);
        //         #endif
        //     #else
        //         pushsprites(&CubeSp, xpos, ypos, iwidth, iheight);
        //     #endif
        // #endif
        if (counter % interval == 0) {  // only calculate the fps every <interval> iterations.
            long millisSinceUpdate = millis() - startMillis;
            fps = String((int)(interval * 1000.0 / (millisSinceUpdate))) + " fps";
            Serial.printf("\n%s\n", fps);
            startMillis = millis();
            printmem();
        }
        if (prime_number_processor_load) {  // Add a processor task
            uint32_t pr = disp.computePrimeNumbers(prime_max);
            Serial.printf("\rbig=%d    ", pr);
        }
        xpos += dx;  // Change coord for next loop
        ypos += dy;

        // }
        // mytft->endWrite();  // Release exclusive use of SPI bus ( here as a reminder... forever loop prevents execution)
        // drawn[DrawSp] =  true;
        // pushed[DrawSp] = false;
        drawn[DrawSp] = true;
        pushed[DrawSp] = false;
        #if USE_DIFFDRAW
        #else
            ++DrawSp %= NumSp;
        #endif
        draw_count++;
        drawtime = esp_timer_get_time()-begin;
    }
};
SpinnyCube spinnycube;
void draw_task() {
    spinnycube.drawcube();
}
void push_task() {
    // Serial.printf("push f%d\n", pushflip);
    #if USE_DIFFDRAW
        #if FULLSCREEN_SPRITES
            disp.pushdiffs(&spr[RefSp], 0, 0, disp_width_pix, disp_height_pix);
        #else
            disp.pushdiffs(&CubeSp, xpos, ypos, iwidth, iheight);
        #endif
    #else
        #if FULLSCREEN_SPRITES
            disp.pushsprites(&spr[PushSp], 0, 0, disp_width_pix, disp_height_pix);
            // pushsprites(&(spr[PushSp]), xpos, ypos, iwidth, iheight);
        #else
            disp.pushsprites(&CubeSp, xpos, ypos, iwidth, iheight);
        #endif
    #endif
    esp_task_wdt_reset();
}

void displaydriver_setup() {
    disp.setup();
    spinnycube.cube_setup(&tft);
    spinnycube.printmem();
    Serial.printf("t:%dx%d d:%dx%d r:%dx%d f:%dx%d\n", tft.width(), tft.height(), spr[DrawSp].width(), spr[DrawSp].height(), spr[RefSp].width(), spr[RefSp].height(), disp_width_pix, disp_height_pix);
    #if USE_SECOND_CORE
        TaskHandle_t drawTaskHandle = nullptr;
        xTaskCreateUniversal([](void*) {
            while(true) {
                draw_task();
                // vTaskDelay(pdMS_TO_TICKS(2)); // Delay for 20ms, hopefully that's fast enough
                // taskYIELD();
                // push_task();
                // taskYIELD();
                // // vTaskDelay(pdMS_TO_TICKS(2)); // Delay for 20ms, hopefully that's fast enough
                // delayMicroseconds(25); // allow for wifi etc
            }
        }, "drawTask", 4096 , NULL, 1, &drawTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);  //  8192
    #endif
    constexpr int runOnCore = CONFIG_ARDUINO_RUNNING_CORE == 0 ? 1 : 0;
    TaskHandle_t pushTaskHandle = nullptr;
    xTaskCreateUniversal([](void*) {
        while(true) {
            #if USE_SECOND_CORE
            #else
                // delayMicroseconds(25); // allow for wifi etc
                draw_task();
            #endif
            delayMicroseconds(25); // allow for wifi etc
            push_task();
            // delayMicroseconds(25); // allow for wifi etc
        }
    }, "pushTask", 4096 , NULL, 1, &pushTaskHandle, runOnCore);  //  8192

    // #if USE_SECOND_CORE    
    //     TaskHandle_t pushTaskHandle = nullptr;
    //     xTaskCreateUniversal([](void*) {
    //         while(true) {
    //             push_task();
    //             delayMicroseconds(100); // allow for wifi etc
    //         }
    //     }, "pushTask", 8192 , NULL, 1, &pushTaskHandle, runOnCore);  //  8192
    // #endif

} 
// void testes() {
//     Serial.printf("Testes ...\n");
//     delay(10);
//     uint16_t a[4000];
//     int x = 0;
//     for (int i=0; i<4000; i++) a[i] = i;
//     uint16_t* aptr0;
//     uint16_t* aptr1;
//     aptr0 = &a[0];
//     aptr1 = &a[2000];
//     Serial.printf("0:%d 1:%d\n", *aptr0, *aptr1);
//     while (true) delay(10);
// }