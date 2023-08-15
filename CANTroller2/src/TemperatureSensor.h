#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include <array>
#include <string>

class TemperatureSensor {
public:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;

private:
    std::string _name;
    DeviceAddress _address;
    float _current_temperature;

public:
    TemperatureSensor(const std::string& name, const DeviceAddress& address)
    : _name(name), _address(address), _current_temperature(-999) {}

    const std::string& get_name() const { return _name; }
    const DeviceAddress& get_address() const { return _address; }
    float get_current_temperature() const { return _current_temperature; }

    void set_current_temperature(float temperature) { _current_temperature = temperature; }
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
};

#endif // TEMPERATURESENSOR_H