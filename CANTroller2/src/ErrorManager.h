#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <Arduino.h>
#include <string>

enum class ErrorType {
    SENSOR_DISCONNECTED,
    LOW_BATTERY,
    // Add more error types as needed
};

class ErrorManager {
public:
    void log_error(ErrorType error, const std::string& source, const std::string& message) {
        // Log the error with the source, message, and error type
        Serial.printf("Error: %s, Source: %s, Message: %s\n", error, source.c_str(), message.c_str());
    }

    void handle_error(ErrorType error, bool error_state) {
        switch (error) {
            case ErrorType::SENSOR_DISCONNECTED:
                // Log error, display error, take action
                if (error_state) {
                    log_error(error, "Main Loop", "Sensor disconnected");
                    // Turn on the idiot light for sensor error
                } else {
                    log_error(error, "Main Loop", "Sensor reconnected");
                    // Turn off the idiot light for sensor error
                }
                break;
            // Handle other error types
        }
    }
};

#endif  // ERROR_MANAGER_H