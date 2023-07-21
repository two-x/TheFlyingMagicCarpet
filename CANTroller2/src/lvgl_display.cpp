#include "lvgl_display.h"

#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 320

TFT_eSPI tft = TFT_eSPI(); 

// Buffer for LittlevGL to draw
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10]; // Declare a buffer for 10 lines

void lvgl_init() {
    // Initialize LVGL
    lv_init();

    // Initialize your display buffer
    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

    // Initialize a display driver
    lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.hor_res = 240;
    disp_drv.ver_res = 320;
    disp_drv.flush_cb = disp_flush;    /*Set your driver function*/
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
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
    lv_obj_t *label = lv_label_create(lv_scr_act(), NULL); // create a label on the default (active) screen
    lv_label_set_text(label, "Hello, world!"); // set the label text
    lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0); // center the label on the screen
    // Serial.println("hello world created");
}
