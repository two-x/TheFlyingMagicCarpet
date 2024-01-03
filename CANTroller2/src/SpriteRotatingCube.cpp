// #pragma once
#include <TFT_eSPI.h>
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
static constexpr int prime_number_processor_load = 491;  // 241 = 50% CPU load for 128 * 128 and STM32F466 Nucleo 64, 491 = 50% CPU load for 128 * 128 and STM32F767 Nucleo 144
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

static constexpr int iwidth = 128;  // 128x128 for a 16-bit colour Sprite (32Kbytes RAM)
static constexpr int iheight = 128;  // Maximum is 181x181 (64Kbytes) for DMA -  restricted by processor design
TFT_eSPI tft = TFT_eSPI();  // Library instance Declare object "tft"
TFT_eSprite spr[2] = { TFT_eSprite(&tft), TFT_eSprite(&tft) };  // Create two sprites for a DMA toggle buffer
bool sprSel = 0;  // Toggle buffer selection
uint16_t* sprPtr[2];  // Pointers to start of Sprites in RAM
uint16_t counter = 0;  // Used for fps measuring
long startMillis = millis();
uint16_t interval = 100;
String fps = "0fps";  // Frames per second
int16_t xpos = 0, ypos = 0;  // Sprite draw position
void setup() {
    Serial.begin(115200);
    tft.init();
    tft.fillScreen(TFT_BLACK);
    xpos = 0;
    ypos = (tft.height() - iheight) / 2;
    for (int i=0; i<=1; i++) {
        spr[i].setColorDepth(16);  // Color depth has to be 16 bits if DMA is used to render image
        sprPtr[i] = (uint16_t*)spr[i].createSprite(iwidth, iheight);
        spr[i].setTextColor(TFT_BLACK);
        spr[i].setTextDatum(MC_DATUM);
    }
    tft.initDMA(); // Initialise the DMA engine (tested with STM32F446 and STM32F767)- should work with ESP32, STM32F2xx/F4xx/F7xx processors  >>>>>> DMA IS FOR SPI DISPLAYS ONLY <<<<<<
    startMillis = millis();  // Animation control timer
}

// related to bouncy cube
static constexpr int cubesize = 358;  // Size of cube image. 358 is max for 128x128 sprite, too big and pixel trails are drawn...
uint16_t palette[] = { TFT_WHITE, TFT_GREENYELLOW, TFT_YELLOW, TFT_PINK, TFT_MAGENTA, TFT_CYAN }; // Define the cube face colors
float d = 15;  // size / 2 of cube edge
bool spinX = true, spinY = true, spinZ = true;  // 3 axis spin control
int xmin, ymin, xmax, ymax;  // Min and max of cube edges, "int" type used for compatibility with original sketch min() function
float px[] = { -d,  d,  d, -d, -d,  d,  d, -d };
float py[] = { -d, -d,  d,  d, -d, -d,  d,  d };
float pz[] = { -d, -d, -d, -d,  d,  d,  d,  d };
float p2x[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // mapped coordinates on screen
float p2y[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float r[] = { 0, 0, 0 };  // rotation angle in radians
int faces[12][3] = {  // Define the triangles. The order of the vertices MUST be CCW or the shoelace method won't work to detect visible edges
    {0, 1, 4}, {1, 5, 4}, {1, 2, 5}, {2, 6, 5}, {5, 7, 4}, {6, 7, 5},
    {3, 4, 7}, {4, 3, 0}, {0, 3, 1}, {1, 3, 2}, {2, 3, 6}, {6, 3, 7},
};
// Detected visible triangles. If calculated area > 0 the triangle is rendered facing towards the viewer, 
// since the vertices are CCW. If the area is negative the triangle is CW and thus facing away from us.
int shoelace(int x1, int y1, int x2, int y2, int x3, int y3) {  // (x1y2 - y1x2) + (x2y3 - y2x3)
    return x1 * y2 - y1 * x2 + x2 * y3 - y2 * x3 + x3 * y1 - y3 * x1;
}
void drawCube() { // Rotates and renders the cube.
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
    spr[sprSel].fillSprite(TFT_BLACK);  // Fill the buffer with colour 0 (Black)
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
            spr[sprSel].fillTriangle(x0, y0, x1, y1, x2, y2, palette[i / 2]);
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
    }
    //spr[sprSel].drawString(fps, iwidth / 2, iheight / 2, 4);
    //delay(100);
}

void loop() {
    uint32_t updateTime = 0;  // time for next update
    int wait = 0;  //random (20);
    tft.startWrite();  // Grab exclusive use of the SPI bus. due to endless loop endWrite() will never get called

    bool bounce = false;
    int dx = 1, dy = 1;
    if (random(2)) dx = -1;  // Random movement direction
    if (random(2)) dy = -1;  // Random movement direction
    while (true) {  // Loop forever
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

        if (updateTime <= millis()) {  // Use time delay so sprtie does not move fast when not all on screen
            updateTime = millis() + wait;
            xmin = iwidth / 2; xmax = iwidth / 2; ymin = iheight / 2; ymax = iheight / 2;
            drawCube();
            if (tft.dmaBusy()) prime_max++; // Increase processing load until just not busy
            tft.pushImageDMA(xpos, ypos, iwidth, iheight, sprPtr[sprSel]);
            sprSel = !sprSel;
            counter++;
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
        }
    }  // End of forever loop
    tft.endWrite();  // Release exclusive use of SPI bus ( here as a reminder... forever loop prevents execution)
}