#ifndef LVGL_DISPLAY_H
#define LVGL_DISPLAY_H

// #include <Adafruit_ILI9341.h>
#include <TFT_eSPI.h> // Include the graphics library
#include <lvgl.h>

#include <Wire.h>
#include "Adafruit_GFX.h"

extern lv_obj_t *chart;
extern lv_chart_series_t *ser1;

void disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p);
void lvgl_init();
void create_hello_world_screen();

#endif
