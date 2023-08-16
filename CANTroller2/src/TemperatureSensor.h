#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include <array>
#include <string>
#include "temp.h"

enum class sensor_location { AMBIENT, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };  // , SOREN_DEV0, SOREN_DEV1, num_known_ };

class TemperatureSensor {
public:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;

    enum class State {
        CONVERTING,
        READY_TO_READ,
        WAITING_FOR_NEXT_CONVERSION
    };

private:
    sensor_location _location;
    DeviceAddress _address;
    float _temperature;
    State _state;
    DallasSensor* _tempsensebus;
    bool temperature_read = false;

public:
    TemperatureSensor(sensor_location location, const DeviceAddress& address, DallasSensor* tempsensebus)
   : _location(location), _address(address), _tempsensebus(tempsensebus), _state(State::CONVERTING), _temperature(-999) {}

    TemperatureSensor() = delete; // always create with a pointer to the tempsensorbus

     void request_temperature() {
        // Request temperature from sensor
        if (!_tempsensebus->requestTemperaturesByAddress(_address.data())) {
            Serial.printf("Error: Failed to request temperature from sensor at address ");
            print_address();
            Serial.println();
        }
        _state = State::CONVERTING;
    }

     bool is_ready() {
        // Check if sensor is ready
        return _tempsensebus->isConversionComplete();
    }

    bool has_valid_address() const {
        // Check if the sensor has a valid address
        // This could involve checking if the address is not equal to a default or invalid address
        return _address != DeviceAddress{};
    }

    float read_temperature() {
        if (is_ready() && _state == State::READY_TO_READ) {
            int temp = _tempsensebus->getTempF(_address.data());
            if (temp == DEVICE_DISCONNECTED_F) {
                Serial.printf("Error: Device at location %s with address ", sensor_location_to_string(_location));
                print_address();
                Serial.println(" is disconnected");
                return DEVICE_DISCONNECTED_F;
            } 
            _temperature = temp;
            _state = State::WAITING_FOR_NEXT_CONVERSION;
            return _temperature;
        }
        return -999; // error value, TODO we could make this a named thing or something more helpful
    }

    // getters
    sensor_location get_location() const { return _location; }
    float get_temperature() const { return _temperature; }
    const DeviceAddress& get_address() const { return _address; }
    State get_state() const { return _state; }
    
    // setters
    void set_location(sensor_location location) { _location = location; }
    void set_temperature(float temperature) { _temperature = temperature; }
    void set_address(DeviceAddress address) {
        _address = address;
    }
    void set_state(State state) {
        _state = state;
    }

    void print_address() const {
        for(uint8_t i = 0; i < _address.size(); i++) {
            if(_address[i] < 0x10) Serial.print("0");
            Serial.print(_address[i], HEX);
        }
        Serial.println();
    }

    void print_sensor_info() const {
        Serial.printf("Location: %s, Assigned Address: ", sensor_location_to_string(_location).c_str());
        print_address();
        Serial.println();
    }

    static std::string sensor_location_to_string(sensor_location location) {
        switch(location) {
            case sensor_location::AMBIENT: return "AMBIENT";
            case sensor_location::ENGINE: return "ENGINE";
            case sensor_location::WHEEL_FL: return "WHEEL_FL";
            case sensor_location::WHEEL_FR: return "WHEEL_FR";
            case sensor_location::WHEEL_RL: return "WHEEL_RL";
            case sensor_location::WHEEL_RR: return "WHEEL_RR";
            default: return "UNKNOWN";
        }
    }
};

#endif // TEMPERATURESENSOR_H