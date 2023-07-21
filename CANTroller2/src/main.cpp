#include <Arduino.h>
#include "lvgl_display.h"
#include "lvgl.h"

// TFT_eSPI tft = TFT_eSPI(); /* Create instance of the TFT_eSPI class */

void setup() {
    Serial.begin (115200);  // Open serial port
    Serial.println("setup complete");
    
    // Initialize the display
    lvgl_init();

    create_hello_world_label();
}

void loop() {
    Serial.println("in loop");
    delay(5); // Allow some delay for system tasks
    
    // Update the display
    lv_task_handler();
}
