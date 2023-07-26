#ifndef TASKS_H
#define TASKS_H 

#include <Arduino.h>
#include <FreeRTOS.h>
#include "globals.h"
#include "RunModeManager.h"

// Task priorities 
#define INPUT_TASK_PRI 1
#define OUTPUT_TASK_PRI 1
#define CONTROL_TASK_PRI 2
#define DISPLAY_TASK_PRI 5
// The ESP32 FreeRTOS supports priorities from 0 to 24, with 0 being the highest priority.
// Some guidelines:
// Time critical tasks like sensors: Priorities 1-5
// User interface tasks: Priorities 5-10
// Background communication tasks: Priorities 15-20


// Task handles  
TaskHandle_t inputTaskHandle;
TaskHandle_t outputTaskHandle;  
TaskHandle_t controlTaskHandle;
TaskHandle_t modeControlTaskHandle;
TaskHandle_t displayTaskHandle;

RunModeManager runModeManager;

// Input task 
void inputTask(void* params){

  while(1){
    // Update inputs.  Fresh sensor data, and filtering.
    vTaskDelay(10); // 10ms
  }
}

// Mode Control Task
void modeControlTask(void* params) {
  while(1){
    // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    runmode = runModeManager.handle_runmode();
    vTaskDelay(10); // 10ms
  }
}

// Control task could be for getting the values from the PID loops to send to the servos
void controlTask(void* params){

  while(1){
  
    // Read latest shared data
    
    // Update motor outputs - takes 185 us to handle every 30ms when the pid timer expires, otherwise 5 us
    //
      
    vTaskDelay(10); // 10ms
  }
}

// Output task could be for sending outputs to the servos
void outputTask(void* params){

  while(1){
  
    // Write actuator outputs  
    vTaskDelay(20); // 20ms
  }
}

// Display task could be for drawing the display
void displayTask(void* params){

  while(1){
  
    // draw the screen
    vTaskDelay(20); // 20ms
  }
}

#endif // TASKS_H
