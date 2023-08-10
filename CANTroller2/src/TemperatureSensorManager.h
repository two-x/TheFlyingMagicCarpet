// temperaturesensors.h

#ifndef TEMPERATURESENSORS_H
#define TEMPERATURESENSORS_H

#include <vector>
#include <map>

#include <OneWire.h>
#include "temp.h"


class TemperatureSensorManager {
private:
    static constexpr uint8_t ONEWIRE_PIN = 19;
    OneWire one_wire_bus;
    DallasTemperature tempsensebus;

    int temp_detected_device_ct = 0;
    std::vector<DeviceAddress> detected_addresses;
    std::map<std::string, DeviceAddress> known_addresses = {
        {"ENGINE", {0x28, 0x3c, 0xf3, 0xa7, 0x00, 0x00, 0x00, 0x00}},
        {"AMBIENT", {0x28, 0x14, 0xb3, 0xbf, 0x00, 0x00, 0x00, 0x00}}
    };
    std::map<std::string, DeviceAddress> assigned_addresses;
    int temp_current_index = 0;

    void print_address(DeviceAddress device_address) {
        for(uint8_t i = 0; i < sizeof(device_address); i++) {
            if(device_address[i] < 0x10) Serial.print("0");
            Serial.print(device_address[i], HEX);
        }
        Serial.println();
    }

public:
    TemperatureSensorManager() : one_wire_bus(ONEWIRE_PIN), tempsensebus(&one_wire_bus) {}

    void init() {
        tempsensebus.setWaitForConversion(false);
        tempsensebus.setCheckForConversion(true);
        tempsensebus.begin();
        temp_detected_device_ct = tempsensebus.getDeviceCount();
        detected_addresses.resize(temp_detected_device_ct);
        Serial.print("Temp sensors.. detected ");
        Serial.print(temp_detected_device_ct);
        Serial.println(" devices.");
        for (int i = 0; i < temp_detected_device_ct; i++) {
            if (tempsensebus.getAddress(detected_addresses[i], i)) {
                tempsensebus.setResolution(detected_addresses[i], temperature_precision);
                Serial.print("  found sensor #");
                Serial.print(i);
                Serial.print(", addr 0x");
                print_address(detected_addresses[i]);
            } else {
                Serial.print("  ghost device #");
                Serial.println(i);
            }
        }
        match_known_addresses();
        assign_unmatched_sensors();
    }

    float read_temperature(const std::string& location) {
        if (assigned_addresses.find(location) != assigned_addresses.end()) {
            return tempsensebus.getTempF(assigned_addresses[location]);
        }
        return -999; // Error value, adjust as needed
    }

    void update_temperature_readings() {
        if (temp_detected_device_ct) {
            tempsensebus.requestTemperaturesByAddress(detected_addresses[temp_current_index]);
            ++temp_current_index %= temp_detected_device_ct;
        }
    }

private:
    void match_known_addresses() {
        for (const auto& [location, addr] : known_addresses) {
            for (const auto& detected_addr : detected_addresses) {
                if (std::equal(std::begin(addr), std::end(addr), std::begin(detected_addr))) {
                    assigned_addresses[location] = detected_addr;
                }
            }
        }
    }

    void assign_unmatched_sensors() {
        for (const auto& detected_addr : detected_addresses) {
            bool is_assigned = false;
            for (const auto& [_, assigned_addr] : assigned_addresses) {
                if (std::equal(std::begin(assigned_addr), std::end(assigned_addr), std::begin(detected_addr))) {
                    is_assigned = true;
                    break;
                }
            }
            if (!is_assigned) {
                // Assign to the next available known location
                for (const auto& [location, _] : known_addresses) {
                    if (assigned_addresses.find(location) == assigned_addresses.end()) {
                        assigned_addresses[location] = detected_addr;
                        break;
                    }
                }
            }
        }
    }
};

#endif // TEMPERATURESENSORS_H