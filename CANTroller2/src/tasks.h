#ifndef TASKS_H
#define TASKS_H 

// Create tasks
// xTaskCreate(sensorTask, "Sense", 2048, NULL, 1, NULL);
// xTaskCreate(controlTask, "Ctrl", 2048, NULL, 2, NULL);
// xTaskCreate(displayTask, "Disp", 2048, NULL, 5, NULL);
// // Create tasks
// xTaskCreatePinnedToCore(sensorTask, "Sense", 2048, NULL, 1, NULL, 0);
// xTaskCreatePinnedToCore(controlTask, "Ctrl", 2048, NULL, 2, NULL, 0); 
// xTaskCreatePinnedToCore(displayTask, "Disp", 2048, NULL, 5, NULL, 0);

// The ESP32 FreeRTOS supports priorities from 0 to 24, with 0 being the highest priority.

// Some guidelines:

// Time critical tasks like sensors: Priorities 1-5
// User interface tasks: Priorities 5-10
// Background communication tasks: Priorities 15-20


#include <Arduino.h>
#include <FreeRTOS.h>

// Task priorities 
#define MAIN_TASK_PRI 1
#define SENSOR_READ_TASK_PRI 2
#define MOTOR_CONTROL_TASK_PRI 3

// Task handles
TaskHandle_t mainTaskHandle;
TaskHandle_t sensorReadTaskHandle;
TaskHandle_t motorControlTaskHandle;

// Main task
void main_task(void* params) {
  // Overall system management
  for(;;) {
    // Your task code here
  }
}

// Sensor reading task 
void sensor_read_task(void* params) {
  // Read sensors
  // Pre-process sensor data
  // Send sensor data to queue
  for(;;) {
    // Your task code here
  }
}

// Motor control task
void motor_control_task(void* params) {
  // Receive sensor data from queue
  // Run control algorithms
  // Send output commands
  for(;;) {
    // Your task code here
  }
}

// Function to create tasks
void create_tasks() {
  xTaskCreate(main_task, "Main", 2048, NULL, MAIN_TASK_PRI, &mainTaskHandle);
  xTaskCreate(sensor_read_task, "Sensors", 2048, NULL, SENSOR_READ_TASK_PRI, &sensorReadTaskHandle);
  xTaskCreate(motor_control_task, "Motors", 2048, NULL, MOTOR_CONTROL_TASK_PRI, &motorControlTaskHandle);
}

#endif // TASKS_H
