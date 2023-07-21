#ifndef LVGL_DISPLAY_H
#define LVGL_DISPLAY_H

#include <Adafruit_ILI9341.h>
#include <lvgl.h>

void disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p);
void lvgl_init();
void create_hello_world_label();

#endif
