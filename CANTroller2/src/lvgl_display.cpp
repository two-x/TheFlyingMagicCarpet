#include "lvgl_display.h"
#include "lvgl.h"

TFT_eSPI tft = TFT_eSPI(); 

// Buffer for LittlevGL to draw
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10]; // Declare a buffer for 10 lines

lv_obj_t *chart = NULL;
lv_chart_series_t *ser1 = NULL;
lv_chart_series_t *ser2 = NULL;
lv_chart_series_t *ser3 = NULL;

void lvgl_init() {
    // Initialize LVGL
    lv_init();

    // Initialize your display buffer
    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

    // Initialize a display driver
    lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.hor_res = 320;
    disp_drv.ver_res = 240;
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
}

void create_hello_world_screen() {
    // Create and style a label with a larger blue box around it
    lv_obj_t *label_bg = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(label_bg, 200, 50);
    lv_obj_set_style_local_bg_color(label_bg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_obj_set_style_local_radius(label_bg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 10);
    lv_obj_set_style_local_value_font(label_bg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_14);

    lv_obj_t *label = lv_label_create(label_bg, NULL);
    lv_label_set_text(label, "Hello, world!");
    lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);

    // Align label to bottom third of screen
    lv_obj_align(label_bg, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);

    // Create a chart
    chart = lv_chart_create(lv_scr_act(), NULL);
    lv_obj_set_size(chart, 200, 150);
    lv_obj_align(chart, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_COLUMN);
    lv_chart_set_div_line_count(chart, 5, 5);
    lv_chart_set_range(chart, 0, 100);
    // Set the point count
    lv_chart_set_point_count(chart, 1);

    // Add series to the chart
    ser1 = lv_chart_add_series(chart, LV_COLOR_RED);

    // Set the data for the series
    // lv_chart_set_next(chart, ser1, 35);

    // Add a title to the chart
    lv_obj_t *chart_label = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_text(chart_label, "Dirty PIDs");
    lv_obj_align(chart_label, chart, LV_ALIGN_OUT_TOP_MID, 0, 30);
}



