#pragma once
#include <vector>
#include <map>
#include <array>
#include <algorithm>
#include <string>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "utils.h"

enum class sensor_location { AMBIENT, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR };  // , SOREN_DEV0, SOREN_DEV1, num_known_ };

class TemperatureSensor {
public:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;

private:
    sensor_location _location;
    DeviceAddress _address;
    float _temperature;
    DallasTemperature* _tempsensebus;

public:
    TemperatureSensor(sensor_location location, const DeviceAddress& address, DallasTemperature* tempsensebus)
   : _location(location), _address(address), _tempsensebus(tempsensebus), _temperature(-999) {}

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
            Serial.printf("Error: Device at location %s with address ", sensor_location_to_string(_location));
            print_address();
            Serial.println(" is disconnected");
            return DEVICE_DISCONNECTED_F;
        } 
        _temperature = temp;
        return _temperature;
    }

    // getters
    sensor_location get_location() const { return _location; }
    float get_temperature() const { return _temperature; }
    const DeviceAddress& get_address() const { return _address; }
    
    // setters
    void set_location(sensor_location location) { _location = location; }
    void set_temperature(float temperature) { _temperature = temperature; }
    void set_address(DeviceAddress address) {
        _address = address;
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

// Class to manage OneWire temperature sensors
class TemperatureSensorManager {
public:
    enum class State {
        CONVERTING,
        READY_TO_READ
    };

private:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;

    int32_t temperature_precision = 11;  // 9-12 bit resolution
    int detected_devices_ct = 0;
    unsigned long last_read_request_time;
    int sensor_index;
    State _state;
    
    OneWire one_wire_bus;
    DallasTemperature tempsensebus;

    std::vector<DeviceAddress> detected_addresses;
    std::vector<sensor_location> all_locations = {
        sensor_location::ENGINE,
        sensor_location::AMBIENT,
        sensor_location::WHEEL_FL,
        sensor_location::WHEEL_FR,
        sensor_location::WHEEL_RL,
        sensor_location::WHEEL_RR
        };
    std::map<sensor_location, DeviceAddress> known_addresses = {
        {sensor_location::ENGINE, {0x28, 0x1a, 0x27, 0x90, 0x5c, 0x21, 0x01, 0x59}},
        {sensor_location::AMBIENT, {0x28, 0x3c, 0xf3, 0xa7, 0xc1, 0x21, 0x06, 0x69}},
        {sensor_location::WHEEL_FL, {0x28, 0x55, 0x42, 0x8f, 0x5c, 0x21, 0x01, 0x69}},
        {sensor_location::WHEEL_FR, {0x28, 0x70, 0x73, 0xb3, 0x5c, 0x21, 0x01, 0x27}},
        {sensor_location::WHEEL_RL, {0x28, 0x54, 0xfb, 0x88, 0x5c, 0x21, 0x01, 0x64}},
        {sensor_location::WHEEL_RR, {0x28, 0x6f, 0xcd, 0xba, 0x5c, 0x21, 0x01, 0x26}}
    };

    std::map<sensor_location, TemperatureSensor> sensors;

    // Assigns known addresses to Sensors. The sensors will have locations like ENGINE or AMBIENT
    void assign_known_addresses() {
        for (auto& known_address : known_addresses) {
            // check to see if we have a known address that wasn't detected, print a warning if yes
            auto detected_address_it = std::find_if(detected_addresses.begin(), detected_addresses.end(), [&](const DeviceAddress& detected_address) {
                return std::equal(detected_address.begin(), detected_address.end(), known_address.second.begin());
            });

            if (detected_address_it != detected_addresses.end()) {
                // The known address was detected, so assign it to the corresponding sensor
                auto sensor_it = sensors.find(known_address.first);
                if (sensor_it == sensors.end()) {
                    // The sensor doesn't exist yet, so create it and add it to the map
                    sensors.emplace(known_address.first, TemperatureSensor(known_address.first, *detected_address_it, &tempsensebus));
                    // Print the sensor address for debugging purposes
                    Serial.println("Assigned known sensor address:");
                    sensors.at(known_address.first).print_address();
                    
                } else {
                    // The sensor already exists, so just update its address
                    sensor_it->second.set_address(*detected_address_it);
                    // Print the updated sensor address for debugging purposes
                    Serial.println("Updated sensor address:");
                    sensor_it->second.print_address();
                    
                }
            } else {
                // The known address was not detected, so log a warning message
                Serial.printf("Warning: Known sensor at location %s was not detected\n", TemperatureSensor::sensor_location_to_string(known_address.first).c_str());
            }
        }
    }

    // Assign remaining addresses to any unassigned locations, in order of the sensor_locations enum
    void assign_remaining_addresses() {
        auto it = all_locations.begin();
        for (auto& detected_address : detected_addresses) {
            if (std::find_if(sensors.begin(), sensors.end(), [&](const std::pair<const sensor_location, TemperatureSensor>& pair) {
                return std::equal(pair.second.get_address().begin(), pair.second.get_address().end(), detected_address.begin());
            }) == sensors.end()) {
                while (it != all_locations.end() && sensors.find(*it) != sensors.end()) {
                    ++it;
                }
                if (it != all_locations.end()) {
                    // The sensor doesn't exist yet, so create it and add it to the map
                    sensors.emplace(*it, TemperatureSensor(*it, detected_address, &tempsensebus));
                    // Print the sensor address for debugging purposes
                    sensors.at(*it).print_address();
                }
            }
        }
    }

public:
    TemperatureSensorManager(uint8_t _onewire_pin) : one_wire_bus(_onewire_pin), tempsensebus(&one_wire_bus),  last_read_request_time(0), sensor_index(0), _state(State::CONVERTING) {}
    void setup() {
        Serial.println("Setting up Temperature Sensors...");
        
        tempsensebus.setWaitForConversion(false);
        tempsensebus.setCheckForConversion(true);
        tempsensebus.begin();
        detected_devices_ct = tempsensebus.getDeviceCount();
        detected_addresses.resize(detected_devices_ct);
        Serial.printf (" detected %d devices, parasitic power is %s\n", detected_devices_ct, (tempsensebus.isParasitePowerMode()) ? "on" : "off");  // , DEC);
        if (detected_devices_ct == 0) {
            Serial.println("No temperature sensor devices detected");
            return;
        }

        // find all detected devices and set their temperature precision
        for (int i = 0; i < detected_devices_ct; i++) {
            if (tempsensebus.getAddress(detected_addresses[i].data(), i)) {
                tempsensebus.setResolution(detected_addresses[i].data(), temperature_precision);
            }
        }

        // Assign known addresses to the sensors they belong to
        assign_known_addresses();

        // Assign remaining addresses to the sensors in order using all_locations
        assign_remaining_addresses();

        // Request temperature for each sensor, this will make the is_ready() method work
        request_temperatures();
    }

    // gets the state of the sensors (still converting values, or ready to read)
    State get_state() const {
        return _state;
    }

    void set_state(State state) {
        _state = state;
    }

    // checks if all of the sensors on the device are ready to be read from, runs every 
    void update_state() {
        if (tempsensebus.isConversionComplete()) {
            _state = State::READY_TO_READ;
        } else {
            _state = State::CONVERTING;
        }
    }

    // previously called temp_soren()
    void update_temperatures() {
        if (sensors.size() == 0) return; // if there are no sensors connected, bail
        // Check if conversions are complete
        if (get_state() == State::READY_TO_READ) {
            // Read temperature from one sensor
            auto sensor_it = std::next(sensors.begin(), sensor_index);
            sensor_it->second.read_temperature();

            // Move to the next sensor
            sensor_index = (sensor_index + 1) % sensors.size();

            // If we've read from all sensors, start the next round of conversions
            if (sensor_index == 0) {
                request_temperatures();
                set_state(State::CONVERTING);
                last_read_request_time = millis();
            }
        } else if (millis() - last_read_request_time >= tempsensebus.millisToWaitForConversion(tempsensebus.getResolution())) {
            // Enough time has passed since the last conversion request, so check if conversions are complete
            update_state();
        }
    }

    int get_detected_device_count() {
        return detected_devices_ct;
    }

    // Method to get a sensor by its location
    TemperatureSensor* get_sensor(sensor_location location) {
        auto it = sensors.find(location);
        if (it != sensors.end()) {
            // The sensor exists in the map, so return it
            return &it->second;
        } else {
            // The sensor doesn't exist in the map
            return nullptr;
        }
    }

    void request_temperatures() {
        // todo consider removing this, as it's non blocking and we risk it not finishing before is_ready runs
        tempsensebus.requestTemperatures();
        // Set state to converting until the sensors are ready to be read from
        set_state(State::CONVERTING);
        // Record the time we made this request so we only check if the sensors are ready every so often
        last_read_request_time = millis();
    }

    int get_micros_to_wait_for_conversion() {
        return 1000 * tempsensebus.millisToWaitForConversion(tempsensebus.getResolution());
    }
    // This looks wrong so I modified above. Here is the original. I don't think it's used anywhere
    // int get_micros_to_wait_for_conversion(int microseconds) {
    //     return 1000 * tempsensebus.millisToWaitForConversion(microseconds);
    // }

};