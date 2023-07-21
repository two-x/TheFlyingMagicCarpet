#include "lvgl_display.h"
#include "lvgl.h"
double t = 0;

#define LV_CHART_POINT_NUM 10

void setup() {
    Serial.begin (115200);  // Open serial port
    
    // Initialize the display
    lvgl_init();

    create_hello_world_screen(); 
}


double generate_data_sine() {
    double result = 50.0 * (sin(t * PI / 180.0) + 1); // results vary between 0 and 100
    t += 10; // Change this to change the speed of the sine wave
    if(t > 360) t -= 360; // Reset the time after a full circle to keep it in the range of valid inputs for sine
    return result;
}

void update_chart() {
    // Generate some data
    float data = generate_data_sine();
    Serial.print("Updating chart with data: ");
    Serial.println(data);

    // Update the chart
    lv_chart_set_next(chart, ser1, (lv_coord_t)data);
    
    // Refresh chart
    // lv_chart_refresh(chart);  // doesn't work
    // lv_refr_now(NULL);        // works but is forcing a whole screen redraw
}

void loop() {
    // Update the display
    lv_task_handler();
    delay(5); // Allow some delay for system tasks
    static uint32_t lastTime = 0; // Keep track of the last time you updated the chart
    uint32_t now = millis(); // Get the current time

    if (now - lastTime > 50) { // If it's been more than 1000 milliseconds (1 second)
        update_chart(); // Update the chart
        lastTime = now; // Remember the current time   
    }
    
}
