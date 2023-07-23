#include "lvgl.h"
#include <TFT_eSPI.h> // Include the graphics library

// #define PI 3.14159265
double t = 0;
lv_obj_t *chart = NULL;
lv_chart_series_t *ser1 = NULL;

TFT_eSPI tft = TFT_eSPI(); 

// Called by the LVGL library to write to the display // eSPI
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

// Buffer for LVGL to draw
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
    // disp_drv.hor_res = 320;
    // disp_drv.ver_res = 240;
    disp_drv.flush_cb = disp_flush;    /*Set your driver function*/
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

    // Initialize TFT
    tft.begin();
    tft.setRotation(3); // Use landscape orientation
}

lv_obj_t *label; // global
lv_obj_t *label_bg; 
void create_hello_world_label() {
    // Create and style a label with a larger blue box around it
    label_bg = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(label_bg, 200, 50);
    lv_obj_set_style_local_bg_color(label_bg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_BLUE);
    lv_obj_set_style_local_radius(label_bg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 10);
    lv_obj_set_style_local_value_font(label_bg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_14);

    label = lv_label_create(label_bg, NULL);

    // Align label to center
    lv_obj_align(label_bg, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
}


void create_chart() {
    // Create a chart
    chart = lv_chart_create(lv_scr_act(), NULL);
    lv_obj_set_size(chart, 200, 150);
    lv_obj_align(chart, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);
    lv_chart_set_type(chart, LV_CHART_TYPE_COLUMN);
    lv_chart_set_div_line_count(chart, 5, 5);
    lv_chart_set_range(chart, 0, 100);
    lv_chart_set_point_count(chart, 1);

    // Add series to the chart
    ser1 = lv_chart_add_series(chart, LV_COLOR_RED);
}

void setup() {
    Serial.begin (115200);  // Open serial port
    
    // Initialize the display
    lvgl_init();

    create_hello_world_label(); 
    create_chart();
}

// Add an array of words to cycle through
const char* words[] = {"Hello", "World", "ESP32", "Arduino", "LVGL"};
const int num_words = sizeof(words) / sizeof(words[0]);  // Calculate the number of words
int word_index = 0;  // Start with the first word

void update_hello_world_label(const char* word) {
    lv_label_set_text(label, word);  // Update the label text
    lv_obj_align(label, label_bg, LV_ALIGN_CENTER, 0, 0); // Re-center the label on the screen
    // Serial.printf("Updating lable to %s\n", word);
}

double generate_data_sine() {
    double result = 50.0 * (sin(t * PI / 180.0) + 1); // results vary between 0 and 100
    t += 10; // Change this to change the speed of the sine wave
    if(t > 360) t -= 360; // Reset the time after a full circle to keep it in the range of valid inputs for sine
    // Serial.println(result); // Print the result for debugging
    return result;
}

void update_chart() {
  double data = generate_data_sine();
  lv_chart_set_next(chart, ser1, lv_coord_t(data));
//   lv_obj_invalidate(chart);
}

// Track last time LVGL was updated
unsigned long lastUpdateTime = millis(); 

// Track current time for LVGL update
unsigned long currentUpdateTime;

// Track last time serial text was updated  
unsigned long lastSerialTime = millis();

// Track current time for serial text update
unsigned long currentSerialTime;

unsigned long lastChartTime = millis();
unsigned long currentChartTime;

unsigned long lastLabelTime = millis();
unsigned long currentLabelTime;

const unsigned long chartUpdateInterval = 10;
const unsigned long labelUpdateInterval = 500;

void loop() {
    currentUpdateTime = millis();
    lv_tick_inc(currentUpdateTime - lastUpdateTime);
    lastUpdateTime = currentUpdateTime;

    unsigned long start_time, end_time, elapsed_time;

    start_time = micros();  // Get the time before lv_task_handler
    // Update the disp
    lv_task_handler();
    end_time = micros();  // Get the time after lv_task_handler

    elapsed_time = end_time - start_time;  // Calculate the elapsed time
    if (elapsed_time/1000 > 1){
        Serial.printf("task handler took %d ms \n", elapsed_time/1000);  // Print the elapsed time
    }

    // Check if it's time to update the chart
    currentChartTime = millis();
    if (currentChartTime - lastChartTime >= chartUpdateInterval) {
        lastChartTime = currentChartTime;
        update_chart();
    }

    // Check if it's time to update the label
    currentLabelTime = millis();
    if (currentLabelTime - lastLabelTime >= labelUpdateInterval) {
        lastLabelTime = currentLabelTime;
        // Update the hello world text 
        word_index = (word_index + 1) % num_words;  // Cycle through the words
        update_hello_world_label(words[word_index]);
    }

    // Serial.printf("Loop duration: %d\n", millis() - currentUpdateTime);
    delay(5);
}








