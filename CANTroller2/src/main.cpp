#include <Arduino.h>
#include "lvgl_display.h"
#include "lvgl.h"

// TFT_eSPI tft = TFT_eSPI(); /* Create instance of the TFT_eSPI class */

void setup() {
    Serial.begin (115200);  // Open serial port
    Serial.println("setup complete");
    // Initialize the display
    lvgl_init();

    // Fill the screen with black color.
    // tft.fillScreen(TFT_BLACK); 
    create_hello_world_label();
    
    /*Change the active screen's background color*/
    // lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xa3108b), LV_PART_MAIN);
}

void loop() {
    Serial.println("in loop");
    delay(5); // Allow some delay for system tasks
    
    // toggle_screen_colors();
    // Update the display
    lv_task_handler();
}
