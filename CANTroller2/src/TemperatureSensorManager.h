#ifndef TEMPERATURESENSORS_H
#define TEMPERATURESENSORS_H

#include <vector>
#include <map>
#include <array>
#include <algorithm>

#include <OneWire.h>
#include "temp.h"
#include "globals.h"
#include "utils.h"

// Class to manage OneWire temperature sensors
class TemperatureSensorManager {
private:
    using DeviceAddress = std::array<uint8_t, 8>;

    static constexpr uint8_t ONEWIRE_PIN = onewire_pin; // todo read from config in the future
    OneWire one_wire_bus;
    DallasSensor tempsensebus;

    int temp_detected_device_ct = 0;
    std::vector<DeviceAddress> detected_addresses;
    std::map<std::string, DeviceAddress> known_addresses = {
        {"ENGINE", {0x28, 0x3c, 0xf3, 0xa7, 0x00, 0x00, 0x00, 0x00}},
        {"AMBIENT", {0x28, 0x14, 0xb3, 0xbf, 0x00, 0x00, 0x00, 0x00}}
    };
    std::map<std::string, DeviceAddress> assigned_addresses;

    // Assign known addresses to detected devices
    void assign_known_addresses() {
        for (auto& detected_address : detected_addresses) {
            for (auto& known_address : known_addresses) {
                if (std::equal(detected_address.begin(), detected_address.end(), known_address.second.begin())) {
                    assigned_addresses[known_address.first] = detected_address;
                    break; // Break the inner loop once a match is found
                }
            }
        }
    }

    // Assign remaining addresses to any unassigned locations
    void assign_remaining_addresses() {
        for (auto& detected_address : detected_addresses) {
            if (std::find_if(assigned_addresses.begin(), assigned_addresses.end(), [&](const std::pair<const std::string, DeviceAddress>& pair) {
                return std::equal(pair.second.begin(), pair.second.end(), detected_address.begin());
            }) == assigned_addresses.end()) {
                for (auto& known_address : known_addresses) {
                    if (assigned_addresses.find(known_address.first) == assigned_addresses.end()) {
                        assigned_addresses[known_address.first] = detected_address;
                        break;
                    }
                }
            }
        }
    }

public:
    TemperatureSensorManager() : one_wire_bus(ONEWIRE_PIN), tempsensebus(&one_wire_bus) {}

    void setup() {
        Serial.println("Setting up Temperature Sensors...");
        tempsensebus.setWaitForConversion(false);
        tempsensebus.setCheckForConversion(true);
        tempsensebus.begin();
        temp_detected_device_ct = tempsensebus.getDeviceCount();
        detected_addresses.resize(temp_detected_device_ct);
        Serial.printf (" detected %d devices, parasitic power is %s\n", temp_detected_device_ct, (tempsensebus.isParasitePowerMode()) ? "on" : "off");  // , DEC);
        for (int i = 0; i < temp_detected_device_ct; i++) {
            if (tempsensebus.getAddress(detected_addresses[i].data(), i)) {
                tempsensebus.setResolution(detected_addresses[i].data(), temperature_precision);
            }
        }

        assign_known_addresses();
        assign_remaining_addresses();
        
        // List the assigned devices
        print_assigned_addresses();
    }

    // Method to read temperature from a specified location
    float read_temperature(const std::string& location) {
        if (assigned_addresses.find(location) != assigned_addresses.end()) {
            return tempsensebus.getTempF(assigned_addresses[location].data());
        }
        return -999; // Error value, adjust as needed
    }

    float read_temperature_by_index(int device_index) {
        if (device_index < temp_detected_device_ct) {
            return tempsensebus.getTempF(detected_addresses[device_index].data());
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
                temps_f[temp_current_index] = read_temperature_by_index(temp_current_index);
                tempTimer.set(temp_timeout_us);
                temp_state = CONVERT;
                ++temp_current_index %= get_device_count();
            }
        }
    }

    // Method to print assigned addresses
    void print_assigned_addresses() {
        for (const auto& pair : assigned_addresses) {
            const auto& location = pair.first;
            const auto& address = pair.second;
            Serial.print("Location: ");
            Serial.print(location.c_str());
            Serial.print(", Assigned Address: ");
            print_address(address.data()); // Use the print_address function from utils.h
            Serial.println();
        }
    }

    int get_device_count() {
        return temp_detected_device_ct; // todo make sure we want to iterate through all addresses vs assigned
    }

    void request_temperatures() {
        tempsensebus.requestTemperatures();
    }

    int micros_to_wait_for_conversion(int precision) {
        return tempsensebus.microsToWaitForConversion(precision);
    }

};

#endif // TEMPERATURESENSORS_H