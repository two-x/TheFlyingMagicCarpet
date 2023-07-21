#include "lvgl_display.h"

// #define TFT_CS 10 
// #define TFT_DC 3 
// #define TFT_RST -1
#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 320

// Initialize the Adafruit ILI9341 display
// Adafruit_ILI9341 display = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Create an instance of the graphics library
TFT_eSPI tft = TFT_eSPI(); 

// Buffer for LittlevGL to draw
static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * LV_VER_RES_MAX * 10]; // Declare a buffer for 10 lines

void lvgl_init() {
    // Initialize LVGL
    lv_init();

    // Initialize your display buffer
    lv_disp_draw_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

    // Initialize a display driver
    lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.hor_res = 240;
    disp_drv.ver_res = 320;
    disp_drv.flush_cb = disp_flush;    /*Set your driver function*/
    disp_drv.draw_buf = &disp_buf;
    // disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

    // Initialize TFT
    tft.begin();
    tft.setRotation(3); // Use landscape orientation
}

// Called by the LittlevGL library to write to the display // eSPI
void disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)color_p, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
    // Serial.println("disp_flush finished");
}

void create_hello_world_label() {
    lv_obj_t *label = lv_label_create(lv_scr_act()); // create a label on the default (active) screen
    lv_label_set_text(label, "Hello, world!"); // set the label text
    lv_obj_center(label); // center the label on the screen
    Serial.println("hello world created");
}


void toggle_screen_colors() {
    /* change the color of the screen every 500ms */
    static uint32_t prevMillis = 0;
    if (millis() - prevMillis >= 500) {
        prevMillis = millis();
        static bool red = true;

        if (red) {
        tft.fillScreen(TFT_RED); /* Fill the screen with red */
        } else {
        tft.fillScreen(TFT_BLUE); /* Fill the screen with blue */
        }

        red = !red; /* Toggle the color */
    }
}

// // This function will be called by LVGL to write to the display // ADAFRUIT
// void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
// {
//     uint16_t c;

//     display.startWrite();
//     display.setAddrWindow(area->x1, area->y1, area->x2, area->y2);
//     for (int16_t h = area->y1; h <= area->y2; h++) {
//         for (int16_t w = area->x1; w <= area->x2; w++) {
//             c = color_p->full;
//             display.drawPixel(w, h, c);
//             color_p++;
//         }
//     }
//     display.endWrite();

//     lv_disp_flush_ready(disp);
// }
