#ifndef TEMPERATURESENSORMANAGER_H
#define TEMPERATURESENSORMANAGER_H

#include <vector>
#include <map>
#include <array>
#include <algorithm>

#include <OneWire.h>
#include "temp.h"
#include "utils.h"
#include "TemperatureSensor.h"

// Class to manage OneWire temperature sensors
class TemperatureSensorManager {
private:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;

    int32_t temperature_precision = 12;  // 9-12 bit resolution
    int detected_devices_ct = 0;
    
    OneWire one_wire_bus;
    DallasSensor tempsensebus;

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
        {sensor_location::ENGINE, {0x28, 0x3c, 0xf3, 0xa7, 0x00, 0x00, 0x00, 0x00}},
        {sensor_location::AMBIENT, {0x28, 0x14, 0xb3, 0xbf, 0x00, 0x00, 0x00, 0x00}}
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
    TemperatureSensorManager(uint8_t _onewire_pin) : one_wire_bus(_onewire_pin), tempsensebus(&one_wire_bus) {}
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

    // previously called temp_soren()
    void update_temperatures() {
        bool all_sensors_ready = true;

        // Iterate through the sensors that have valid addresses, check if they're ready, and update
        for (auto& sensor : sensors) {
            if (sensor.second.has_valid_address() && sensor.second.get_state() != TemperatureSensor::State::WAITING_FOR_NEXT_CONVERSION) {
                if (sensor.second.is_ready()) {
                    // The sensor is ready, so set the state to READY_TO_READ
                    sensor.second.set_state(TemperatureSensor::State::READY_TO_READ);
                    // Then read the temperature
                    sensor.second.read_temperature();
                } else {
                    // At least one sensor is not ready
                    all_sensors_ready = false;
                }
            }
        }

        // If all sensors are ready, start new temperature conversions
        if (all_sensors_ready) {
            request_temperatures();
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
            // Serial.printf("Error: Sensor at location %s not found\n", TemperatureSensor::sensor_location_to_string(location).c_str());
            return nullptr;
            // throw std::runtime_error("Sensor not found"); // we could also just panic if this is not ok
        }
    }

    void request_temperatures() {
        // todo consider removing this, as it's non blocking and we risk it not finishing before is_ready runs
        tempsensebus.requestTemperatures();
        // Set all sensors to the CONVERT state
        for (auto& sensor : sensors) {
            sensor.second.set_state(TemperatureSensor::State::CONVERTING);
        }
    }

    int get_micros_to_wait_for_conversion(int microseconds) {
        return tempsensebus.microsToWaitForConversion(microseconds);
    }

};

#endif // TEMPERATURESENSORMANAGER_H