#pragma once
#include <vector>
#include <string>  // used when printing out addresses
#include <iostream>  // used when printing out addresses
#include <DallasTemperature.h>

enum class loc { TempAmbient=0, TempEngine, TempWheelFL, TempWheelFR, TempWheelRL, TempWheelRR, TempBrake, NumTempLocations };  // , SorenDev0, SorenDev1, };
enum temp_categories { CatUnknown=0, CatAmbient=1, CatEngine=2, CatWheel=3, CatBrake=4, NumTempCategories=5 };  // 
enum brakemotor_types { Nil=-1, Thomson=0, MotorFactoryStore=1, GoMotorWorld=2, NumBrakeMotorTypes=3 };
    
float temp_lims_f[NumTempCategories][NumMotorVals] {
    // changed opmin values all to 40 to avoid idiot lights. engine opmin was 125, wheel was 50, brake was 45
    {  40.0,  77.0, 120.0, 135.0, NAN, -67.0, 257.0, 2.0 },  // [CatUnknown] [OpMin/Cent/OpMax/Alarm/Filt/AbsMin/AbsMax/Margin]
    {  40.0,  77.0, 120.0, 135.0, NAN, -67.0, 257.0, 2.0 },  // [CatAmbient] [OpMin/Cent/OpMax/Alarm/Filt/AbsMin/AbsMax/Margin]
    {  40.0, 178.0, 205.0, 218.0, NAN, -67.0, 257.0, 2.0 },  //  [CatEngine] [OpMin/Cent/OpMax/Alarm/Filt/AbsMin/AbsMax/Margin]
    {  40.0,  77.0, 170.0, 145.0, NAN, -67.0, 257.0, 2.0 },  //   [CatWheel] [OpMin/Cent/OpMax/Alarm/Filt/AbsMin/AbsMax/Margin] (applies to all wheels)
    {  45.0,  77.0, 115.0, 125.0, NAN, -67.0, 257.0, 2.0 },  //   [CatBrake] [OpMin/Cent/OpMax/Alarm/Filt/AbsMin/AbsMax/Margin]
};  // float* degf[(int)loc::NumTempLocatio  ns][NumMotorVals];

class TemperatureSensor {
public:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;
private:
    loc _location;
    DeviceAddress _address;
    float _temperature;
    DallasTemperature* _tempsensebus;
    int category = CatUnknown;
public:
    TemperatureSensor(loc location, const DeviceAddress& address, DallasTemperature* tempsensebus)
   : _location(location), _address(address), _tempsensebus(tempsensebus), _temperature(-999) {}

    TemperatureSensor() = delete; // always create with a pointer to the tempsensorbus
    float* degf[NumMotorVals] = { nanptr, nanptr, nanptr, nanptr, nanptr, nanptr, nanptr, nanptr };

    void request_temperature() {
        // Request temperature from sensor
        if (!_tempsensebus->requestTemperaturesByAddress(_address.data())) {
            ezread.squintf(ezread.madcolor, "err: temp fail request from 0x%s\n", addr_hex_string().c_str());
            // print_address();
            // ezread.squintf("\n");
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
            // ezread.squintf("  disconnected device %s w/ addr:\n", location_to_string(_location));
            ezread.squintf(ezread.madcolor, "err: disconnected temp sensor %s\n  at addr: 0x%s\n", location_to_string(_location), addr_hex_string().c_str());
            // print_address();
            // ezread.squintf("\n");
            return DEVICE_DISCONNECTED_F;
        } 
        _temperature = temp;
        // *degf[Filt] = _temperature;
        return _temperature;
    }

    // getters
    loc get_location() const { return _location; }
    float get_temperature() const { return _temperature; }
    const DeviceAddress& get_address() const { return _address; }
    
    // setters
    void set_location(loc location) { _location = location; }
    void set_temperature(float temperature) { _temperature = temperature; }
    void set_address(DeviceAddress address) {
        _address = address;
    }
    void set_lims() {
        if (_location == loc::TempAmbient) category = CatAmbient;
        else if (_location == loc::TempEngine) category = CatEngine;
        else if (_location == loc::TempBrake) category = CatBrake;
        else if (_location == loc::TempWheelFL || _location == loc::TempWheelFR || _location == loc::TempWheelRL || _location == loc::TempWheelRR ) category = CatWheel;
        else category = CatUnknown;
        for (int i=0; i<NumMotorVals; i++) if (i != Filt) degf[i] = &temp_lims_f[category][i];  // degf[(int)sens][Filt] = &_temperature;
        degf[Filt] = &_temperature;
    }
    std::string addr_hex_string() const {
        std::string str;
        for (uint8_t i = 0; i < _address.size(); i++) {  // str += std::format("{:02x}", _address[i]);  // ezread.squintf("%02x", _address[i]);
            char buf[3];
            snprintf(buf, sizeof(buf), "%02x", _address[i]);
            str += buf;
        }
        return str;
    }

    void print_sensor_info() const {
        ezread.squintf("  assigned %s to 0x%s\n", location_to_string(_location).c_str(), addr_hex_string().c_str());
        // print_address();
        // ezread.squintf("\n");
    }
    static std::string location_to_string(loc location) {
        switch(location) {
            case loc::TempAmbient: return "ambient";
            case loc::TempEngine: return "engine";
            case loc::TempWheelFL: return "wheel_fl";
            case loc::TempWheelFR: return "wheel_fr";
            case loc::TempWheelRL: return "wheel_rl";
            case loc::TempWheelRR: return "wheel_rr";
            case loc::TempBrake: return "brakemotor";
            default: return "unknown";
        }
    }
    float val() { return _temperature; }
    float opmin() { return *degf[OpMin]; }
    float opmax() { return *degf[OpMax]; }
    float absmin() { return *degf[AbsMin]; }
    float absmax() { return *degf[AbsMax]; }
    float alarm() { return *degf[Alarm]; }
    float margin() { return *degf[Margin]; }
    float cent() { return *degf[Cent]; }
    float* ptr() { return &_temperature; }
    float* opmin_ptr() { return degf[OpMin]; }
    float* opmax_ptr() { return degf[OpMax]; }
    float* absmin_ptr() { return degf[AbsMin]; }
    float* absmax_ptr() { return degf[AbsMax]; }
    float* alarm_ptr() { return degf[Alarm]; }
    float* margin_ptr() { return degf[Margin]; }
    float* cent_ptr() { return degf[Cent]; }
};


// Class to manage OneWire temperature sensors
class TemperatureSensorManager {
public:
    enum class State {
        CONVERTING,
        READY_TO_READ
    };
    bool vehicle_detected = true;
    int brakemotor_type_detected = Nil;  // default value
    int temperature_precision = 11;  // 9-12 bit resolution
    int detected_devices_ct = 0;
private:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;
    bool brake_assigned;
    unsigned long last_read_request_time;
    int sensor_index;
    int lost_sensors = 0;
    State _state;
    
    OneWire one_wire_bus;
    DallasTemperature tempsensebus;

    std::vector<DeviceAddress> detected_addresses;
    std::vector<loc> all_locations = {
        loc::TempEngine,
        loc::TempAmbient,
        loc::TempWheelFL,
        loc::TempWheelFR,
        loc::TempWheelRL,
        loc::TempWheelRR,
        loc::TempBrake,
    };
    std::map<loc, DeviceAddress> known_addresses = {  // these are the (default?) sensor addresses
        {loc::TempEngine, {0x28, 0x1a, 0x27, 0x90, 0x5c, 0x21, 0x01, 0x59}},  // mule engine sensor
        {loc::TempAmbient, {0x28, 0x53, 0x57, 0xad, 0x5c, 0x21, 0x01, 0x02}},  // sensor glued to the the control box
        {loc::TempWheelFL, {0x28, 0x55, 0x42, 0x8f, 0x5c, 0x21, 0x01, 0x69}},  // these are the sensors on the car
        {loc::TempWheelFR, {0x28, 0x70, 0x73, 0xb3, 0x5c, 0x21, 0x01, 0x27}},  // these are the sensors on the car
        {loc::TempWheelRL, {0x28, 0x54, 0xfb, 0x88, 0x5c, 0x21, 0x01, 0x64}},  // these are the sensors on the car
        {loc::TempWheelRR, {0x28, 0x6f, 0xcd, 0xba, 0x5c, 0x21, 0x01, 0x26}},  // these are the sensors on the car
        {loc::TempBrake, {0x28, 0xb5, 0x1d, 0x9c, 0x4b, 0x20, 0x01, 0xfe}},  // ?
        // {loc::TempBrake, {0x28, 0x6b, 0x0f, 0x84, 0x4b, 0x20, 0x01, 0xf2}},  // ?
        
        // {0x28, 0x53, 0x57, 0xad, 0x5c, 0x21, 0x01, 0x02}  // sensor glued to the the control box
        // {0x28, 0x09, 0xe0, 0xd7, 0x5c, 0x21, 0x01, 0x4e}  // sensor on soren's breadboard
        // {0x28, 0x6b, 0x0f, 0x84, 0x4b, 0x20, 0x01, 0xf2}  // Thomson (2023) motor (?) confirm this
        // {0x28, 0xce, 0x10, 0x8b, 0x4b, 0x20, 0x01, 0xcc}  // MotorFactoryStore (2024) motor
        // {0x28, 0xf0, 0x03, 0xb6, 0x5c, 0x21, 0x01, 0x21}  // GoMotorWorld (2025) motor

        // A different sensor is glued to each motor, so address depends which one is installed.
        // Use this fact to autodetect the motor, assign it to the loc::TempBrake location ...
        // so we can load the appropriate calibrations, etc.
    };

    std::map<loc, TemperatureSensor> sensors;

    // Assigns known addresses to Sensors. The sensors will have locations like engine or ambient
    void assign_known_addresses() {
        DeviceAddress thomson_brake_address = {0x28, 0x6b, 0x0f, 0x84, 0x4b, 0x20, 0x01, 0xf2};
        DeviceAddress mfs_brake_address = {0x28, 0xce, 0x10, 0x8b, 0x4b, 0x20, 0x01, 0xcc};
        DeviceAddress gmw_brake_address = {0x28, 0xf0, 0x03, 0xb6, 0x5c, 0x21, 0x01, 0x21};
        
        bool brake_assigned = false;

        // First handle brake sensors
        for (auto& detected_address : detected_addresses) {
            if (std::equal(detected_address.begin(), detected_address.end(), thomson_brake_address.begin())) {
                 brakemotor_type_detected = Thomson;
            }
            else if (std::equal(detected_address.begin(), detected_address.end(), mfs_brake_address.begin())) {
                brakemotor_type_detected = MotorFactoryStore;
            }
            else if (std::equal(detected_address.begin(), detected_address.end(), gmw_brake_address.begin())) {
                brakemotor_type_detected = GoMotorWorld;
            }
            if (!brake_assigned && (brakemotor_type_detected != Nil)) {
                sensors.emplace(loc::TempBrake, TemperatureSensor(loc::TempBrake, detected_address, &tempsensebus));
                ezread.squintf("  detected %s brake at 0x%s\n", brakemotor_type_to_string(brakemotor_type_detected).c_str(), sensors.at(loc::TempBrake).addr_hex_string().c_str());
                // sensors.at(loc::TempBrake).print_address();
                // ezread.squintf("\n");
                brake_assigned = true;
            }
            continue;
        }

        // Then handle other sensors
        for (auto& known_address : known_addresses) {
            if (known_address.first == loc::TempBrake) continue; // Skip brake because it's already handled

            // check to see if we have a known address that wasn't detected, print a warning if yes
            auto detected_address_it = std::find_if(detected_addresses.begin(), detected_addresses.end(), [&](const DeviceAddress& detected_address) {
                return std::equal(detected_address.begin(), detected_address.end(), known_address.second.begin());
            });

            if (detected_address_it != detected_addresses.end()) {
                // The known address was detected, so assign it to the corresponding sensor
                loc location = known_address.first;
                
                auto sensor_it = sensors.find(location);
                if (sensor_it == sensors.end()) {
                    // The sensor doesn't exist yet, so create it and add it to the map
                    sensors.emplace(location, TemperatureSensor(location, *detected_address_it, &tempsensebus));
                    // Print the sensor address for debugging purposes
                    ezread.squintf("  known %s at 0x%s\n", TemperatureSensor::location_to_string(known_address.first).c_str(), sensors.at(known_address.first).addr_hex_string().c_str());
                    // sensors.at(known_address.first).print_address();
                    // ezread.squintf("\n");
                }
                else {
                    // The sensor already exists, so just update its address
                    sensor_it->second.set_address(*detected_address_it);
                    // Print the updated sensor address for debugging purposes
                    ezread.squintf("  updated addr 0x%s\n", sensor_it->second.addr_hex_string().c_str());
                    // sensor_it->second.print_address();
                    // sensor_it->second.set_lims();
                }
            }
            else {
                lost_sensors++;
                // The known address was not detected, so log a warning message
                // ezread.squintf("  known sensor %s not detected\n", TemperatureSensor::location_to_string(known_address.first).c_str());
            }
        }
        vehicle_detected = detected(loc::TempAmbient);
    }

    // Assign remaining addresses to any unassigned locations, in order of the locations enum
    void assign_remaining_addresses() {
        auto it = all_locations.begin();
        for (auto& detected_address : detected_addresses) {
            if (std::find_if(sensors.begin(), sensors.end(), [&](const std::pair<const loc, TemperatureSensor>& pair) {
                return std::equal(pair.second.get_address().begin(), pair.second.get_address().end(), detected_address.begin());
            }) == sensors.end()) {
                while (it != all_locations.end() && sensors.find(*it) != sensors.end()) {
                    ++it;
                }
                if (it != all_locations.end()) {
                    // The sensor doesn't exist yet, so create it and add it to the map and print the sensor address
                    sensors.emplace(*it, TemperatureSensor(*it, detected_address, &tempsensebus));
                    ezread.squintf("  unknown sensor at 0x%s ..\n", sensors.at(*it).addr_hex_string().c_str());  // ezread.squintf("  unknown addr: ");
                    ezread.squintf("    assigned as %s sensor.\n", TemperatureSensor::location_to_string(*it).c_str());
                    // sensors.at(*it).print_address();
                    // ezread.squintf("\n");
                }
            }
        }
    }
    // void assign_category(*TemperatureSensor sens, int temp_category) {
    //     auto sensor_it = sensors.find(sens);
    //     if (sensor_it == sensors.end()) {
    //             for (int i=0; i<NumMotorVals; i++) {  // if (i != Filt) degf[(int)sens][i] = &temp_lims_f[temp_category][i];
    //         if (i != Filt) degf[i] = &temp_lims_f[temp_category][i];  // degf[(int)sens][Filt] = &_temperature;
    //     }
    //     degf[Filt] = &_temperature;
    //     }
    // }
    // void assign_categories() {
    //     for (loc i=TempAmbient; i<NumTempLocations; i = (loc)((int)i + 1)) {
    //         auto it = sensors.find(i);
    //         if (it != sensors.end()) {
    //             if (i == TempAmbient) assign_category(&it->second, CatAmbient);
    //             else if (i == TempEngine) assign_category(&it->second, CatEngine);
    //             else if (i == TempBrake) assign_category(&it->second, CatBrake);
    //             else if (i == TempWheelFL || i == TempWheelFR || i == TempWheelRL || i == TempWheelRR ) assign_category(&it->second, CatWheel);
    //             else assign_category(&it->second, CatUnknown);
    //         }                
    //         else {
    //             // The sensor doesn't exist in the map
    //             return nullptr;
    //         }            
    //     }
    // }
    static std::string brakemotor_type_to_string(int motortype) {
        switch(motortype) {
            case Nil: return "undetected";
            case Thomson: return "Thomson";
            case MotorFactoryStore: return "MFS";
            case GoMotorWorld: return "GMW";
            default: return "undetected";
        }
    }

public:
    TemperatureSensorManager(uint8_t _onewire_pin) : one_wire_bus(_onewire_pin), tempsensebus(&one_wire_bus),  last_read_request_time(0), sensor_index(0), _state(State::CONVERTING) {}
    bool setup() {
        tempsensebus.setWaitForConversion(false);
        tempsensebus.setCheckForConversion(true);
        tempsensebus.begin();
        detected_devices_ct = tempsensebus.getDeviceCount();
        detected_addresses.resize(detected_devices_ct);
        ezread.squintf(ezread.highlightcolor, "Temp sensors (p%d).. found %d devices:\n", onewire_pin, detected_devices_ct);  // ezread.squintf(" parasitic %s.", (tempsensebus.isParasitePowerMode()) ? "on" : "off");
        if (detected_devices_ct == 0) vehicle_detected = false;
        else {
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

            // assign_categories();
            if (lost_sensors) ezread.squintf("  did not detect %d known sensor(s)\n", lost_sensors);
            // Request temperature for each sensor, this will make the is_ready() method work
            request_temperatures();
        }
        ezread.squintf("  vehicle %sdetected. using %s config.\n", vehicle_detected ? "" : "not ", vehicle_detected ? "on-car" : "devboard");
        return vehicle_detected;
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
        }
        else if (millis() - last_read_request_time >= tempsensebus.millisToWaitForConversion(tempsensebus.getResolution())) {
            // Enough time has passed since the last conversion request, so check if conversions are complete
            update_state();
        }
    }

    int get_detected_device_count() {
        return detected_devices_ct;
    }

    // Method to get a sensor by its location
    TemperatureSensor* get_sensor(loc location) {
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

    // Soren: I made this function for code to easily get most recently read temp value from a location
    float val(loc locat) {
        TemperatureSensor* sens = get_sensor(locat);  // ambient
        if (!sens) return NAN;  // avoid crashing if undetected location is indicated
        return sens->get_temperature();
    }
    bool detected(loc locat) {
        TemperatureSensor* sens = get_sensor(locat);  // ambient
        return (bool)sens; 
    }
    int locint(loc locat = loc::NumTempLocations) {
        return static_cast<int>(locat);
    }
    int errclass(loc locat) {
        if (locat == loc::TempAmbient || locat == loc::TempEngine) return static_cast<int>(locat);
        return static_cast<int>(loc::TempWheelFL);  // All wheels use this error class
    }
    int errclass(int locat) { return errclass(static_cast<loc>(locat)); }
    float val(int locat) { return val(static_cast<loc>(locat)); }
    bool detected(int locat) { return detected(static_cast<loc>(locat)); }
    src source() { return src::Pin; }
    int brake_type() { return brakemotor_type_detected; }

    float opmin(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return NAN; return sens->opmin(); }
    float opmax(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return NAN; return sens->opmax(); }
    float absmin(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return NAN; return sens->absmin(); }
    float absmax(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return NAN; return sens->absmax(); }
    float alarm(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return NAN; return sens->alarm(); }
    float cent(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return NAN; return sens->cent(); }
    float margin(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return NAN; return sens->margin(); }

    float* ptr(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return nanptr; return sens->ptr(); }
    float* opmin_ptr(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return nanptr; return sens->opmin_ptr(); }
    float* opmax_ptr(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return nanptr; return sens->opmax_ptr(); }
    float* absmin_ptr(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return nanptr; return sens->absmin_ptr(); }
    float* absmax_ptr(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return nanptr; return sens->absmax_ptr(); }
    float* alarm_ptr(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return nanptr; return sens->alarm_ptr(); }
    float* cent_ptr(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return nanptr; return sens->cent_ptr(); }
    float* margin_ptr(loc locat) { TemperatureSensor* sens = get_sensor(locat); if (!sens) return nanptr; return sens->margin_ptr(); }
};