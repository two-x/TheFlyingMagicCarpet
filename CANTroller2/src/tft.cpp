// #pragma once
// tft.h - started life as some examples in the tft_espi library, including one called SpriteRotatingCube.ino
#include <TFT_eSPI.h>
#include <esp_task_wdt.h>
#define USE_SECOND_CORE 1
#define USE_DIFFDRAW 0
#define USE_DMA 1
#define FULLSCREEN_SPRITES 0
#define disp_width_pix 320
#define disp_height_pix 240

// enum displaysprites : int { DrawSp, RefSp, PushSp, NumSp };
enum dirs : int { HORZ, VERT };

#if USE_DIFFDRAW
    volatile int PushSp = 2;
    static constexpr int NumSp = 3;
#else
    volatile int PushSp = 1;
    static constexpr int NumSp = 2;
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
static constexpr int prime_number_processor_load = 0;  // 491;  // 241 = 50% CPU load for 128 * 128 and STM32F466 Nucleo 64, 491 = 50% CPU load for 128 * 128 and STM32F767 Nucleo 144
int prime_max = 2;  // Prime number initial value
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
volatile bool ready_to_push = false;
volatile int draw_count;
volatile int push_count;
int push_wait_us, draw_wait_us;
volatile int DrawSp = 0, RefSp = 1;
volatile bool pushed[NumSp];
volatile bool drawn[NumSp];
int corner[2] = { 0, 0 };
int tft_w, tft_h;
static constexpr uint32_t color_depth = 16;
static constexpr int iwidth = 128;  // 128x128 for a 16-bit colour Sprite (32Kbytes RAM)
static constexpr int iheight = 128;  // Maximum is 181x181 (64Kbytes) for DMA -  restricted by processor design
TFT_eSPI tft = TFT_eSPI();  // Library instance Declare object "tft"
#if USE_DIFFDRAW
    TFT_eSprite spr[NumSp] = { TFT_eSprite(&tft), TFT_eSprite(&tft), TFT_eSprite(&tft) };
#else
    TFT_eSprite spr[NumSp] = { TFT_eSprite(&tft), TFT_eSprite(&tft) };
#endif
uint16_t* sprPtr[NumSp];  // Pointers to start of Sprites in RAM
uint16_t counter = 0;  // Used for fps measuring
long startMillis = millis();
uint16_t interval = 100;
String fps = "0fps";  // Frames per second
int16_t xpos = 0, ypos = 0;  // Sprite draw position

class SpinnyCube {
  public:
    SpinnyCube() { 
        cube_setup();
    }
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
        spr[DrawSp].fillSprite(TFT_BLACK);  // Fill the buffer with colour 0 (Black)
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
                    spr[DrawSp].fillTriangle(x0, y0, x1, y1, x2, y2, palette[i / 2]);
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

    void cube_setup() {
        if (random(2)) dx = -1;  // Random movement direction
        if (random(2)) dy = -1;  // Random movement direction
    }

    void drawcube() {  // should not 
        while (true) {  // Loop forever
            draw_wait_us = 0;
            while (drawn[DrawSp] || !pushed[DrawSp]) {
                delayMicroseconds(10);
                draw_wait_us += 10;
            }
            // Serial.printf("d%d s:%d,%d,%d dp:%d%d,%d%d,%d%d\n", draw_wait_us, DrawSp, RefSp, PushSp, drawn[0], pushed[0], drawn[1], pushed[1], drawn[2], pushed[2]);
            drawn[DrawSp] = false;
            // Serial.printf("draw f%d\n", flip);
            LOCK_SPRITE(DrawSp);
            if (xpos >= tft.width() - xmax) { bounce = true; dx = -1; }  // Pull it back onto screen if it wanders off
            else if (xpos < -xmin) { bounce = true; dx = 1; }
            if (ypos >= tft.height() - ymax) { bounce = true; dy = -1; }
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
            #if USE_SECOND_CORE
            #else    
                push_task();
            #endif
            if (counter % interval == 0) {  // only calculate the fps every <interval> iterations.
                long millisSinceUpdate = millis() - startMillis;
                fps = String((int)(interval * 1000.0 / (millisSinceUpdate))) + " fps";
                Serial.printf("\n%s\n", fps);
                startMillis = millis();
            }
            if (prime_number_processor_load) {  // Add a processor task
                uint32_t pr = computePrimeNumbers(prime_max);
                Serial.printf("\rbig=%d    ", pr);
            }
            xpos += dx;  // Change coord for next loop
            ypos += dy;

            // }
            // tft.endWrite();  // Release exclusive use of SPI bus ( here as a reminder... forever loop prevents execution)
            drawn[DrawSp] = true;
            pushed[DrawSp] = false;
            ++DrawSp %= NumSp;
            draw_count++;
        }
    }
};
SpinnyCube spinnycube;

void pushsprites() {  // uint_fast8_t flip
    // Method 1 (BouncyBoxSprite example) : draws the next frame on one sprite while pushing the last, swapping back and forth
    push_wait_us = 0;
    while (!drawn[PushSp] || pushed[PushSp]) {
        delayMicroseconds(10);
        push_wait_us += 10;
    }
    // Serial.printf("p%d s:%d,%d,%d dp:%d%d,%d%d,%d%d\n", push_wait_us, DrawSp, RefSp, PushSp, drawn[0], pushed[0], drawn[1], pushed[1], drawn[2], pushed[2]);
    LOCK_SPRITE(PushSp);
    // LOCK_SPRITE(RefSp);
    #if USE_DMA
        // if (tft.dmaBusy() && prime_number_processor_load) prime_max++; // Increase processing load until just not busy
        // while (tft.dmaBusy() || (draw_count <= push_count)) delayMicroseconds(200);  // Hang out till dma can be used
        tft.startWrite();  // Start SPI transaction and drop TFT_CS - avoids transaction overhead in loop
        
        tft.pushImageDMA(xpos, ypos, iwidth, iheight, sprPtr[PushSp]);
        
        tft.endWrite();
    #else
        if (prime_number_processor_load) prime_max = prime_number_processor_load;
        spr[PushSp].pushSprite(xpos, ypos); // Blocking write (no DMA) 115fps
    #endif
    counter++;
    UNLOCK_SPRITE(PushSp);
    pushed[PushSp] = true;
    drawn[PushSp] = false;
    ++PushSp %= NumSp;
    push_count++;
    // // Method 2 (BouncyCircles example) : draws screen in alternating halves. Modified with option to use 2nd core
    // #if USE_DMA
    //     tft.pushImageDMA(0, flip * disp_height_pix / 2, disp_width_pix, disp_height_pix / 2, sprPtr[flip]);
    // #else
    //     tft.pushImage(0, flip * disp_height_pix / 2, disp_width_pix, disp_height_pix / 2, sprPtr[flip]);
    // #endif
}
void pushdiffs() {
    push_wait_us = 0;
    while (!drawn[PushSp] || pushed[PushSp]) {
        delayMicroseconds(10);
        push_wait_us += 10;
    }
    // Serial.printf("p%d s:%d,%d,%d dp:%d%d,%d%d,%d%d\n", push_wait_us, DrawSp, RefSp, PushSp, drawn[0], pushed[0], drawn[1], pushed[1], drawn[2], pushed[2]);
    LOCK_SPRITE(PushSp);
    // LOCK_SPRITE(RefSp);
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
    auto sprwidth = spr[PushSp].width();
    auto sprheight = spr[PushSp].height();
    auto w32 = (sprwidth + 1) >> 1;  // (color_depth == 8) ? 3 : 1) >> ((color_depth == 8) ? 2 : 1);
    std::int32_t y = 0;
    tft.startWrite();  // Start SPI transaction and drop TFT_CS - avoids transaction overhead in loop
    do {
        std::int32_t x32 = 0;
        std::int32_t xs, xe;
        do {
            while (s32[x32] == p32[x32] && ++x32 < w32);
            if (x32 == w32) break;
            // if (color_depth == 8) {
            //     xs = x32 << 2;
            //     while (s8[xs] == p8[xs]) ++xs;
            //     while (++x32 < w32 && s32[x32] != p32[x32]);
            //     xe = (x32 << 2) - 1;
            //     if (xe >= sprwidth) xe = sprwidth - 1;
            //     while (s8[xe] == p8[xe]) --xe;
            // }
            // else if (color_depth == 16) {
                xs = x32 << 1;
                while (s16[xs] == p16[xs]) ++xs;
                while (++x32 < w32 && s32[x32] != p32[x32]);
                xe = (x32 << 1) - 1;
                if (xe >= sprwidth) xe = sprwidth - 1;
                while (s16[xe] == p16[xe]) --xe;
            // }
            
            #if USE_DMA
                // tft.pushPixelsDMA(&(s[xs]), xe - xs + 1);
                #if FULLSCREEN_SPRITES
                    // if (color_depth == 8) tft.pushImageDMA(xs, y, xe - xs + 1, 1, &s8[xs]);
                    // else if (color_depth == 16) 
                    // tft.setAddrWindow(xs, y, xe - xs + 1, 1);   // Set window area to pour pixels into
                    // tft.pushPixelsDMA(&renderbuf[bufIdx][0], xe - xs + 1); // Push line to screen        
                    tft.pushImageDMA(xs, y, xe - xs + 1, 1, &s16[xs]);
                #else
                    // if (color_depth == 8) tft.pushImageDMA(xs + xpos, y + ypos, xe - xs + 1, 1, &s8[xs]);
                    // else if (color_depth == 16) 
                    tft.pushImageDMA(xs + xpos, y + ypos, xe - xs + 1, 1, &s16[xs]);
                #endif
            #else
                #if FULLSCREEN_SPRITES
                    if (color_depth == 8) tft.pushImage(xs, y, xe - xs + 1, 1, &s8[xs]);
                    else if (color_depth == 16) tft.pushImage(xs, y, xe - xs + 1, 1, &s16[xs]);
                #else
                    if (color_depth == 8) tft.pushImage(xs + xpos, y + ypos, xe - xs + 1, 1, &s8[xs]);
                    else if (color_depth == 16) tft.pushImage(xs + xpos, y + ypos, xe - xs + 1, 1, &s16[xs]);
                #endif
            #endif
            // lcd->pushImage(xs + corner[HORZ], y + corner[VERT], xe - xs + 1, 1, &s[xs]);
        } while (x32 < w32);
        s32 += w32;
        p32 += w32;
    } while (++y < sprheight);
    // tft.display();
    tft.endWrite();
    counter++;
    // UNLOCK_SPRITE(RefSp);
    UNLOCK_SPRITE(PushSp);
    pushed[PushSp] = true;
    drawn[RefSp] = false;
    ++PushSp %= NumSp;
    ++RefSp %= NumSp;
    push_count++;
}
void push_task() {
    // Serial.printf("push f%d\n", pushflip);
    #if USE_DIFFDRAW
        pushdiffs();
    #else
        pushsprites();
    #endif
    esp_task_wdt_reset();
}
void gfx_setup() {
    Serial.begin(115200);
    delay(1000);
    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(2);
    xpos = 0;
    ypos = (tft.height() - iheight) / 2;
    tft_w = tft.width();
    tft_h = tft.height();
    drawn[DrawSp] = pushed[PushSp] = false;
    pushed[DrawSp] = drawn[PushSp] = true;
    #if USE_DIFFDRAW
        pushed[RefSp] = drawn[RefSp] = true;
    #endif
    for (int i=0; i<NumSp; i++) {
            spr[i].setColorDepth(color_depth);  // Color depth has to be 16 bits if DMA is used to render image
        #if FULLSCREEN_SPRITES
            spr[i].createSprite(tft_w, tft_h/2);
            sprPtr[i] = (uint16_t*)(spr[i].getPointer());
        #else
            sprPtr[i] = (uint16_t*)(spr[i].createSprite(iwidth, iheight));
        #endif
        spr[i].setTextColor(TFT_WHITE);
        spr[i].setTextDatum(MC_DATUM);
    }
    #if USE_DMA
        tft.initDMA(); // Initialise the DMA engine (tested with STM32F446 and STM32F767)- should work with ESP32, STM32F2xx/F4xx/F7xx processors  >>>>>> DMA IS FOR SPI DISPLAYS ONLY <<<<<<
    #endif
    startMillis = millis();  // Animation control timer
    #if USE_SECOND_CORE    
        constexpr int runOnCore = CONFIG_ARDUINO_RUNNING_CORE == 0 ? 1 : 0;
        TaskHandle_t pushTaskHandle = nullptr;
        xTaskCreateUniversal([](void*) {
            while(true) {
                push_task();
                delayMicroseconds(200); // allow for wifi etc
            }
        }, "pushTask", 16384 , NULL, 1, &pushTaskHandle, runOnCore);  //  8192
    #endif
}
// void update_framebuffer(bool flip) {}


void setup() {
    gfx_setup();
    spinnycube.cube_setup();
} 

void loop() { spinnycube.drawcube(); } 
