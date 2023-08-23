#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include <array>
#include <string>
#include <map>
#include "temp.h"
#include "TemperatureLimits.h"

class TemperatureSensor {
public:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;

    enum class location { AMBIENT, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };  // , SOREN_DEV0, SOREN_DEV1, num_known_ };
    enum class SensorType {AMBIENT, ENGINE, WHEEL}; // SensorType is here to avoid redundant code, as we group the 4 wheels together a lot
    static constexpr float ROOM_TEMPERATURE_F = 77.0;  // "Room" temperature is 25 C = 77 F
    static constexpr float SENSOR_MIN_READING_F = -67.0;  // Minimum reading of sensor is -25 C = -67 F
    static constexpr float SENSOR_MAX_READING_F = 257.0;  // Maximum reading of sensor is 125 C = 257 F

private:
    location _location;
    DeviceAddress _address;
    float _temperature;
    DallasSensor* _tempsensebus;
    SensorType _type;
    TemperatureLimits _limits;
    float temp_room = 77.0;  // "Room" temperature is 25 C = 77 F  Who cares?
    float temp_sensor_min_f = -67.0;  // Minimum reading of sensor is -25 C = -67 F
    float temp_sensor_max_f = 257.0;  // Maximum reading of sensor is 125 C = 257 F

    // Initialize the static map of temperature limits
    static inline const std::map<SensorType, TemperatureLimits> temp_limits_map = {
        {SensorType::AMBIENT, TemperatureLimits(0.0,  45.0, 115.0, 120.0, 130.0, 220.0)},
        {SensorType::ENGINE,  TemperatureLimits(0.0, 178.0, 198.0, 202.0, 205.0, 220.0)},
        {SensorType::WHEEL,   TemperatureLimits(0.0,  50.0, 120.0, 130.0, 140.0, 220.0)}
    };

public:
    TemperatureSensor(location location, const DeviceAddress& address, DallasSensor* tempsensebus, SensorType type)
   : _location(location), _address(address), _tempsensebus(tempsensebus), _temperature(-999), _type(type), _limits(temp_limits_map.at(type)) {}

    TemperatureSensor() = delete; // always create with a pointer to the tempsensorbus

    void request_temperature() {
        // Request temperature from sensor
        if (!_tempsensebus->requestTemperaturesByAddress(_address.data())) {
            Serial.printf("Error: Failed to request temperature from sensor at address ");
            print_address();
            Serial.println();
        }
    }

    bool has_valid_address() const {
        // Check if the sensor has a valid address
        // This could involve checking if the address is not equal to a default or invalid address
        return _address != DeviceAddress{};
    }

    float read_temperature() {
        float temp = _tempsensebus->getTempF(_address.data());
        if (temp == DEVICE_DISCONNECTED_F) {
            Serial.printf("Error: Device at location %s with address ", location_to_string(_location));
            print_address();
            Serial.println(" is disconnected");
            return DEVICE_DISCONNECTED_F;
        } 
        _temperature = temp;
        return _temperature;
    }

    // getters
    location get_location() const { return _location; }
    float get_temperature() const { return _temperature; }
    const DeviceAddress& get_address() const { return _address; }
    TemperatureLimits get_limits() const { return _limits; }

    // setters
    void set_location(location location) { _location = location; }
    void set_temperature(float temperature) { _temperature = temperature; }
    void set_address(DeviceAddress address) { _address = address; }
    void set_limits(TemperatureLimits limits) {_limits = limits; }
    
    void print_address() const {
        for(uint8_t i = 0; i < _address.size(); i++) {
            if(_address[i] < 0x10) Serial.print("0");
            Serial.print(_address[i], HEX);
        }
        Serial.println();
    }

    void print_sensor_info() const {
        Serial.printf("Location: %s, Assigned Address: ", location_to_string(_location).c_str());
        print_address();
        Serial.println();
    }

    static std::string location_to_string(location location) {
        switch(location) {
            case location::AMBIENT: return "AMBIENT";
            case location::ENGINE: return "ENGINE";
            case location::WHEEL_FL: return "WHEEL_FL";
            case location::WHEEL_FR: return "WHEEL_FR";
            case location::WHEEL_RL: return "WHEEL_RL";
            case location::WHEEL_RR: return "WHEEL_RR";
            default: return "UNKNOWN";
        }
    }
};

#endif // TEMPERATURESENSOR_H