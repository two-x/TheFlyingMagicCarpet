#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <Arduino.h>
#include <string>
#include <SD.h>

enum class ErrorType {
    SENSOR_DISCONNECTED,
    TEMPERATURE_WARNING,
    TEMPERATURE_ALARM,
    // Add more error types as needed
};

class ErrorManager {
public:
    void log_error(ErrorType error, const std::string& source, const std::string& message) {
        // Log the error with the source, message, and error type
        unsigned long timeSinceBoot = millis();
        Serial.printf("Time: %lu, Error: %s, Source: %s, Message: %s\n", timeSinceBoot, error, source.c_str(), message.c_str());
        log_error_to_sd_card(error, source, message, timeSinceBoot);
    }

   void log_error_to_sd_card(ErrorType error, const std::string& source, const std::string& message, unsigned long timeSinceBoot) {
        static bool firstLog = true;
        static String logFileName;

        // create a new log file for each time we boot up
        if (firstLog) {
            unsigned long bootTime = millis();
            logFileName = "log_" + String(bootTime) + ".txt";
            firstLog = false;
        }

        File logFile = SD.open(logFileName.c_str(), FILE_WRITE);
        if (logFile) {
            logFile.printf("Time: %lu, Error: %s, Source: %s, Message: %s\n", timeSinceBoot, errorTypeToString(error).c_str(), source.c_str(), message.c_str());
            logFile.close();
        } else {
            Serial.println("Failed to open log file");
        }
    }

    void handle_error(ErrorType error, bool error_state) {
        static bool last_error_state = false; // only log when the state changes
        if (last_error_state != error_state) {
            last_error_state = error_state;
            switch (error) {
                case ErrorType::SENSOR_DISCONNECTED:
                    // Log error, display error, take action
                    if (error_state) {
                        log_error(error, "Main Loop", "Sensor disconnected");
                    } else {
                        log_error(error, "Main Loop", "Sensor reconnected");
                    }
                    break;
                case ErrorType::TEMPERATURE_WARNING:
                    if (error_state) {
                        log_error(error, "Main Loop", "Temperature warning");
                    } else {
                        log_error(error, "Main Loop", "Temperature warning resolved");   
                    }
                    break;
                case ErrorType::TEMPERATURE_ALARM:
                    if (error_state) {
                        log_error(error, "Main Loop", "Temperature alarm");
                    } else {
                        log_error(error, "Main Loop", "Temperature alarm resolved");
                    }
                    break;
                // Handle other error types
            }
        }
    }
private:
    std::string errorTypeToString(ErrorType error) {
        switch (error) {
            case ErrorType::SENSOR_DISCONNECTED: return "SENSOR_DISCONNECTED";
            case ErrorType::TEMPERATURE_WARNING: return "TEMPERATURE_WARNING";
            case ErrorType::TEMPERATURE_ALARM: return "TEMPERATURE_ALARM";
            // Add more cases as needed
            default: return "UNKNOWN_ERROR";
        }
    }
};

#endif  // ERROR_MANAGER_H