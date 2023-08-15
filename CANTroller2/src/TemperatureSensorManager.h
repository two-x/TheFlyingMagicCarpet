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
#include "globals.h"

// Class to manage OneWire temperature sensors
class TemperatureSensorManager {
private:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;

    static constexpr uint8_t ONEWIRE_PIN = 19; // todo read from config in the future
    int32_t temperature_precision = 12;  // 9-12 bit resolution
    int32_t current_index = 0;
    int detected_devices_ct = 0;
    
    OneWire one_wire_bus;
    DallasSensor tempsensebus;

    std::vector<std::string> all_locations = {"ENGINE", "AMBIENT", "WHEEL_FL", "WHEEL_FR", "WHEEL_RL", "WHEEL_RR"};
    std::vector<DeviceAddress> detected_addresses;
    std::map<std::string, DeviceAddress> known_addresses = {
        {"ENGINE", {0x28, 0x3c, 0xf3, 0xa7, 0x00, 0x00, 0x00, 0x00}},
        {"AMBIENT", {0x28, 0x14, 0xb3, 0xbf, 0x00, 0x00, 0x00, 0x00}}
    };
    std::map<std::string, TemperatureSensor> sensors;

    // Assigns known addresses to Sensors. The sensors will have names like ENGINE or AMBIENT
    void assign_known_addresses() {
        for (auto& detected_address : detected_addresses) {
            for (auto& known_address : known_addresses) {
                if (std::equal(detected_address.begin(), detected_address.end(), known_address.second.begin())) {
                    sensors[known_address.first].set_address(detected_address);
                    break; // Break the inner loop once a match is found
                }
            }
        }
    }

    // Assign remaining addresses to any unassigned locations
    void assign_remaining_addresses() {
        auto it = all_locations.begin();
        for (auto& detected_address : detected_addresses) {
            if (std::find_if(sensors.begin(), sensors.end(), [&](const std::pair<const std::string, TemperatureSensor>& pair) {
                return std::equal(pair.second.get_address().begin(), pair.second.get_address().end(), detected_address.begin());
            }) == sensors.end()) {
                while (it != all_locations.end() && sensors[*it].get_address() != DeviceAddress{}) {
                    ++it;
                }
                if (it != all_locations.end()) {
                    sensors[*it].set_address(detected_address);
                }
            }
        }
    }
    

public:
    TemperatureSensorManager() : one_wire_bus(ONEWIRE_PIN), tempsensebus(&one_wire_bus) {}

    void setup() {
        Serial.println("Setting up Temperature Sensors...");
        
        // Instantiate TemperatureSensor objects with their names
        for (const auto& location : all_locations) {
            sensors[location] = TemperatureSensor(location, {});
        }
        
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
        for (int i = 0; i < detected_devices_ct; i++) {
            // getAddress returns true if it gets an address when it checks at index i
            // if it does find an address, it stores it in detected_addresses[i] and sets precision
            // the .data() returns a pointer that getAddress uses to populate detected-addresses
            if (tempsensebus.getAddress(detected_addresses[i].data(), i)) {
                tempsensebus.setResolution(detected_addresses[i].data(), temperature_precision);
            }
        }

        // Assign known addresses to the sensors they belong to
        assign_known_addresses();

        // Assign remaining addresses to the sensors in order using all_locations
        assign_remaining_addresses();

        // List the assigned devices
        print_assigned_addresses();
    }

    // Method to read temperature from a specified location
    float read_temperature(const std::string& location) {
        if (sensors.find(location) != sensors.end()) {
            return tempsensebus.getTempF(sensors[location].get_address().data());
        }
        return -999; // Error value, adjust as needed
    }

    float read_temperature_by_index(int device_index) {
        if (device_index < detected_devices_ct) {
            return tempsensebus.getTempF(detected_addresses[device_index].data());
        }
        return -999; // Error value, adjust as needed
    }

    float get_last_known_temperature(const std::string& location) {
        if (sensors.find(location) != sensors.end()) {
            return sensors[location].get_current_temperature();
        }
        return -999; // Error value, adjust as needed
    }

    // previously called temp_soren()
    void update_temperatures() {
        if (get_device_count() && tempTimer.expired()) {
            if (temp_state == CONVERT) {
                request_temperatures();
                tempTimer.set(micros_to_wait_for_conversion(temperature_precision));
                temp_state = READ;
            }
            else if (temp_state == READ) {
                temps_f[current_index] = read_temperature_by_index(current_index);
                tempTimer.set(temp_timeout_us);
                temp_state = CONVERT;
                ++current_index %= get_device_count();
            }
        }
    }

    // Method to print assigned addresses
    void print_assigned_addresses() {
        for (const auto& pair : sensors) {
            const auto& location = pair.first;
            const auto& sensor = pair.second;
            Serial.print("Location: ");
            Serial.print(location.c_str());
            Serial.print(", Assigned Address: ");
            sensor.print_address(); // Use the print_address function from utils.h
            Serial.println();
        }
    }

    int get_device_count() {
        return detected_devices_ct; // todo make sure we want to iterate through all addresses vs assigned
    }

    void request_temperatures() {
        tempsensebus.requestTemperatures();
    }

    int micros_to_wait_for_conversion(int precision) {
        return tempsensebus.microsToWaitForConversion(precision);
    }

};

#endif // TEMPERATURESENSORMANAGER_H