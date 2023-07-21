#include <TFT_eSPI.h> // Include the graphics library
#include <lvgl.h>

#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 320

TFT_eSPI tft = TFT_eSPI(); /* Create instance of the TFT_eSPI class */

// Buffer for LittlevGL to draw
static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * LV_VER_RES_MAX / 10]; // Declare a buffer for 10 lines

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

void lvgl_init() {
    // Initialize LVGL
    lv_init();

    // Initialize your display buffer
    lv_disp_draw_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

    // Initialize a display driver
    lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;
    disp_drv.flush_cb = disp_flush;    /*Set your driver function*/
    disp_drv.draw_buf = &disp_buf;
    // disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

    // Initialize TFT
    tft.begin();
    tft.setRotation(3); // Use landscape orientation
}

lv_obj_t *label; // global
void create_hello_world_label() {
    label = lv_label_create(lv_scr_act()); // create a label on the default (active) screen
    lv_label_set_text(label, "Hello, world!"); // set the label text
    lv_obj_center(label); // center the label on the screen
}

void setup() {
    Serial.begin (115200);  // Open serial port
    // Initialize the display
    lvgl_init();

    // create_hello_world_label();
}


// Add an array of words to cycle through
const char* words[] = {"Hello", "World", "ESP32", "Arduino", "LVGL"};
const int num_words = sizeof(words) / sizeof(words[0]);  // Calculate the number of words
int word_index = 0;  // Start with the first word

void update_hello_world_label(const char* word) {
    lv_label_set_text(label, word);  // Update the label text
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0); // Re-center the label on the screen
}

unsigned long last_serial_time = millis();
unsigned long current_serial_time = last_serial_time;
unsigned long last_time = millis();
unsigned long current_time = last_time;

void loop() {
    current_time = millis();
    lv_tick_inc(current_time - last_time);
    last_time = current_time;
    // Update the display
    lv_task_handler();
    current_serial_time = millis();
    if (current_serial_time - last_serial_time == 1000) {
        last_serial_time = current_serial_time;
        Serial.printf("hi!\n");
        // Update the hello world text every second
        word_index = (word_index + 1) % num_words;  // Cycle through the words
        update_hello_world_label(words[word_index]);  // Update the label with the new word
    }
    
    delay(1); // Allow some delay for system tasks
}

