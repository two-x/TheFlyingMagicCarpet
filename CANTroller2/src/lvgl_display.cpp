#include "lvgl_display.h"

#define TFT_CS 10 
#define TFT_DC 3 
#define TFT_RST -1

// Initialize the Adafruit ILI9341 display
Adafruit_ILI9341 display = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// This function will be called by LVGL to write to the display
void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint16_t c;

    display.startWrite();
    display.setAddrWindow(area->x1, area->y1, area->x2, area->y2);
    for (int16_t h = area->y1; h <= area->y2; h++) {
        for (int16_t w = area->x1; w <= area->x2; w++) {
            c = color_p->full;
            display.drawPixel(w, h, c);
            color_p++;
        }
    }
    display.endWrite();

    lv_disp_flush_ready(disp);
}

void lvgl_init() {
    lv_init();

    static lv_disp_draw_buf_t disp_buf;
    static lv_color_t buf[240 * 10]; // Replace 240 with your display's horizontal resolution

    lv_disp_draw_buf_init(&disp_buf, buf, NULL, 240 * 10); // Again, replace 240 with your display's horizontal resolution

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = 240; // And replace these with your display's resolution
    disp_drv.ver_res = 320;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);
}

void create_hello_world_label() {
    lv_obj_t *label = lv_label_create(lv_scr_act()); // create a label on the default (active) screen
    lv_label_set_text(label, "Hello, world!"); // set the label text
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0); // align the label to the center of the screen
} 