#pragma once
#include <vector>
#include <DallasTemperature.h>

enum class loc { AMBIENT=0, ENGINE, WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR, BRAKE, NUM_LOCATIONS };  // , SOREN_DEV0, SOREN_DEV1, };
enum temp_categories { CatUnknown=0, CatAmbient=1, CatEngine=2, CatWheel=3, CatBrake=4, NumTempCategories=5 };  // 
enum brakemotor_types { NIL=-1, Thomson=0, LAE=1 };
    
float temp_lims_f[NumTempCategories][NUM_MOTORVALS] {
    // changed opmin values all to 40 to avoid idiot lights. engine opmin was 125, wheel was 50, brake was 45
    {  40.0,  77.0, 120.0, 135.0, NAN, -67.0, 257.0, 2.0 },  // [CatUnknown] [OPMIN/CENT/OPMAX/ALARM/FILT/ABSMIN/ABSMAX/MARGIN]
    {  40.0,  77.0, 120.0, 135.0, NAN, -67.0, 257.0, 2.0 },  // [CatAmbient] [OPMIN/CENT/OPMAX/ALARM/FILT/ABSMIN/ABSMAX/MARGIN]
    {  40.0, 178.0, 205.0, 218.0, NAN, -67.0, 257.0, 2.0 },  //  [CatEngine] [OPMIN/CENT/OPMAX/ALARM/FILT/ABSMIN/ABSMAX/MARGIN]
    {  40.0,  77.0, 170.0, 145.0, NAN, -67.0, 257.0, 2.0 },  //   [CatWheel] [OPMIN/CENT/OPMAX/ALARM/FILT/ABSMIN/ABSMAX/MARGIN] (applies to all wheels)
    {  45.0,  77.0, 125.0, 135.0, NAN, -67.0, 257.0, 2.0 },  //   [CatBrake] [OPMIN/CENT/OPMAX/ALARM/FILT/ABSMIN/ABSMAX/MARGIN]
};  // float* degf[(int)loc::NUM_LOCATIONS][NUM_MOTORVALS];

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
    float* degf[NUM_MOTORVALS] = { nanptr, nanptr, nanptr, nanptr, nanptr, nanptr, nanptr, nanptr };

    void request_temperature() {
        // Request temperature from sensor
        if (!_tempsensebus->requestTemperaturesByAddress(_address.data())) {
            ezread.squintf("  failed temp request from sensor addr ");
            print_address();
            ezread.squintf("\n");
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
            ezread.squintf("  disconnected device %s w/ addr: ", location_to_string(_location));
            print_address();
            ezread.squintf("\n");
            return DEVICE_DISCONNECTED_F;
        } 
        _temperature = temp;
        // *degf[FILT] = _temperature;
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
        if (_location == loc::AMBIENT) category = CatAmbient;
        else if (_location == loc::ENGINE) category = CatEngine;
        else if (_location == loc::BRAKE) category = CatBrake;
        else if (_location == loc::WHEEL_FL || _location == loc::WHEEL_FR || _location == loc::WHEEL_RL || _location == loc::WHEEL_RR ) category = CatWheel;
        else category = CatUnknown;
        for (int i=0; i<NUM_MOTORVALS; i++) if (i != FILT) degf[i] = &temp_lims_f[category][i];  // degf[(int)sens][FILT] = &_temperature;
        degf[FILT] = &_temperature;
    }
    
    void print_address() const {
        ezread.squintf("0x");
        for(uint8_t i = 0; i < _address.size(); i++) ezread.squintf("%02x", _address[i]);
        ezread.squintf("\n");
    }

    void print_sensor_info() const {
        ezread.squintf("  loc: %s, assigned addr ", location_to_string(_location).c_str());
        print_address();
        ezread.squintf("\n");
    }

    static std::string location_to_string(loc location) {
        switch(location) {
            case loc::AMBIENT: return "ambient";
            case loc::ENGINE: return "engine";
            case loc::WHEEL_FL: return "wheel_fl";
            case loc::WHEEL_FR: return "wheel_fr";
            case loc::WHEEL_RL: return "wheel_rl";
            case loc::WHEEL_RR: return "wheel_rr";
            case loc::BRAKE: return "brakemotor";
            default: return "unknown";
        }
    }
    float val() { return _temperature; }
    float opmin() { return *degf[OPMIN]; }
    float opmax() { return *degf[OPMAX]; }
    float absmin() { return *degf[ABSMIN]; }
    float absmax() { return *degf[ABSMAX]; }
    float alarm() { return *degf[ALARM]; }
    float margin() { return *degf[MARGIN]; }
    float cent() { return *degf[CENT]; }
    float* ptr() { return &_temperature; }
    float* opmin_ptr() { return degf[OPMIN]; }
    float* opmax_ptr() { return degf[OPMAX]; }
    float* absmin_ptr() { return degf[ABSMIN]; }
    float* absmax_ptr() { return degf[ABSMAX]; }
    float* alarm_ptr() { return degf[ALARM]; }
    float* margin_ptr() { return degf[MARGIN]; }
    float* cent_ptr() { return degf[CENT]; }
};


// Class to manage OneWire temperature sensors
class TemperatureSensorManager {
public:
    enum class State {
        CONVERTING,
        READY_TO_READ
    };
    bool vehicle_detected = true;
    int brakemotor_type_detected = NIL;  // default value
    int temperature_precision = 11;  // 9-12 bit resolution
    int detected_devices_ct = 0;
private:
    // Replace DeviceAddress with std::array<uint8_t, 8>
    using DeviceAddress = std::array<uint8_t, 8>;
    bool brake_assigned;
    unsigned long last_read_request_time;
    int sensor_index;
    State _state;
    
    OneWire one_wire_bus;
    DallasTemperature tempsensebus;

    std::vector<DeviceAddress> detected_addresses;
    std::vector<loc> all_locations = {
        loc::ENGINE,
        loc::AMBIENT,
        loc::WHEEL_FL,
        loc::WHEEL_FR,
        loc::WHEEL_RL,
        loc::WHEEL_RR,
        loc::BRAKE,
    };
    std::map<loc, DeviceAddress> known_addresses = {
        {loc::ENGINE, {0x28, 0x1a, 0x27, 0x90, 0x5c, 0x21, 0x01, 0x59}},
        {loc::AMBIENT, {0x28, 0x53, 0x57, 0xad, 0x5c, 0x21, 0x01, 0x02}},
        {loc::WHEEL_FL, {0x28, 0x55, 0x42, 0x8f, 0x5c, 0x21, 0x01, 0x69}},
        {loc::WHEEL_FR, {0x28, 0x70, 0x73, 0xb3, 0x5c, 0x21, 0x01, 0x27}},
        {loc::WHEEL_RL, {0x28, 0x54, 0xfb, 0x88, 0x5c, 0x21, 0x01, 0x64}},
        {loc::WHEEL_RR, {0x28, 0x6f, 0xcd, 0xba, 0x5c, 0x21, 0x01, 0x26}},
        {loc::BRAKE, {0x28, 0x6b, 0x0f, 0x84, 0x4b, 0x20, 0x01, 0xf2}},
        
        // Attn Bobby!
        // Todo: The BRAKE sensor above might be either of two sensor addresses, depending on which motor is installed.
        // Actually we'd like to use this fact to autodetect which motor is installed, that can inform other parts of 
        // the code. But whichever of the two is detected should be assigned to the loc::BRAKE location.
        // Here are the two addresses:
        // Thomson Motor: {0x28, 0x09, 0xe0, 0xd7, 0x5c, 0x21, 0x01, 0x4e}
        //     LAE Motor: {0x28, 0xce, 0x10, 0x8b, 0x4b, 0x20, 0x01, 0xcc}
    };

    std::map<loc, TemperatureSensor> sensors;

    // Assigns known addresses to Sensors. The sensors will have locations like engine or ambient
    void assign_known_addresses() {
        int lost_sensors = 0;
        DeviceAddress thomson_brake_address = {0x28, 0x09, 0xe0, 0xd7, 0x5c, 0x21, 0x01, 0x4e};
        DeviceAddress lae_brake_address = {0x28, 0xce, 0x10, 0x8b, 0x4b, 0x20, 0x01, 0xcc};
        bool brake_assigned = false;

        // First handle brake sensors
        for (auto& detected_address : detected_addresses) {
            if (std::equal(detected_address.begin(), detected_address.end(), thomson_brake_address.begin())) {
                 brakemotor_type_detected = Thomson;  // default value
            }
            else if (std::equal(detected_address.begin(), detected_address.end(), lae_brake_address.begin())) {
                brakemotor_type_detected = LAE;  // default value
            }
            if (!brake_assigned && (brakemotor_type_detected != NIL)) {
                sensors.emplace(loc::BRAKE, TemperatureSensor(loc::BRAKE, detected_address, &tempsensebus));
                Serial.printf("  detected %s brake sensor addr: ", brakemotor_type_to_string(brakemotor_type_detected).c_str());
                ezread.printf("  %s brake addr: ", brakemotor_type_to_string(brakemotor_type_detected).c_str());
                sensors.at(loc::BRAKE).print_address();
                brake_assigned = true;
            }
            continue;
        }

        // Then handle other sensors
        for (auto& known_address : known_addresses) {
            if (known_address.first == loc::BRAKE) continue; // Skip brake because it's already handled

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
                    Serial.printf("  known sensor %s addr: ", TemperatureSensor::location_to_string(known_address.first).c_str());
                    ezread.printf("  known %s @ addr: ", TemperatureSensor::location_to_string(known_address.first).c_str());
                    sensors.at(known_address.first).print_address();
                } else {
                    // The sensor already exists, so just update its address
                    sensor_it->second.set_address(*detected_address_it);
                    // Print the updated sensor address for debugging purposes
                    ezread.squintf("  updated sensor addr: ");
                    sensor_it->second.print_address();

                    sensor_it->second.set_lims();

                }
            } else {
                lost_sensors++;
                // The known address was not detected, so log a warning message
                // ezread.squintf("  known sensor %s not detected\n", TemperatureSensor::location_to_string(known_address.first).c_str());
            }
        }
        if (lost_sensors) ezread.squintf("  did not detect %d known sensor(s)\n", lost_sensors);
        vehicle_detected = detected(loc::AMBIENT);
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
                    Serial.printf("  detected unknown sensor addr: ");
                    ezread.printf("  unknown addr: ");
                    sensors.emplace(*it, TemperatureSensor(*it, detected_address, &tempsensebus));
                    sensors.at(*it).print_address();
                    ezread.squintf("\n");
                }
            }
        }
    }
    // void assign_category(*TemperatureSensor sens, int temp_category) {
    //     auto sensor_it = sensors.find(sens);
    //     if (sensor_it == sensors.end()) {
    //             for (int i=0; i<NUM_MOTORVALS; i++) {  // if (i != FILT) degf[(int)sens][i] = &temp_lims_f[temp_category][i];
    //         if (i != FILT) degf[i] = &temp_lims_f[temp_category][i];  // degf[(int)sens][FILT] = &_temperature;
    //     }
    //     degf[FILT] = &_temperature;
    //     }
    // }
    // void assign_categories() {
    //     for (loc i=AMBIENT; i<NUM_LOCATIONS; i = (loc)((int)i + 1)) {
    //         auto it = sensors.find(i);
    //         if (it != sensors.end()) {
    //             if (i == AMBIENT) assign_category(&it->second, CatAmbient);
    //             else if (i == ENGINE) assign_category(&it->second, CatEngine);
    //             else if (i == BRAKE) assign_category(&it->second, CatBrake);
    //             else if (i == WHEEL_FL || i == WHEEL_FR || i == WHEEL_RL || i == WHEEL_RR ) assign_category(&it->second, CatWheel);
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
            case NIL: return "undetected";
            case Thomson: return "Thomson";
            case LAE: return "LAE";
            default: return "undetected";
        }
    }

public:
    TemperatureSensorManager(uint8_t _onewire_pin) : one_wire_bus(_onewire_pin), tempsensebus(&one_wire_bus),  last_read_request_time(0), sensor_index(0), _state(State::CONVERTING) {}
    bool setup() {
        ezread.squintf("Temp sensors:");
        
        tempsensebus.setWaitForConversion(false);
        tempsensebus.setCheckForConversion(true);
        tempsensebus.begin();
        detected_devices_ct = tempsensebus.getDeviceCount();
        detected_addresses.resize(detected_devices_ct);
        ezread.squintf(" parasitic %s. found %d device(s):\n", (tempsensebus.isParasitePowerMode()) ? "on" : "off", detected_devices_ct);  // , DEC);
        if (detected_devices_ct == 0) {
            // ezread.squintf("  no devices detected\n");
            vehicle_detected = false;
        }
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

            // Request temperature for each sensor, this will make the is_ready() method work
            request_temperatures();
        }
        ezread.squintf("  detected %s context\n", vehicle_detected ? "on-vehicle" : "dev-board");
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
        } else if (millis() - last_read_request_time >= tempsensebus.millisToWaitForConversion(tempsensebus.getResolution())) {
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
    int locint(loc locat = loc::NUM_LOCATIONS) {
        return static_cast<int>(locat);
    }
    int errclass(loc locat) {
        if (locat == loc::AMBIENT || locat == loc::ENGINE) return static_cast<int>(locat);
        return static_cast<int>(loc::WHEEL_FL);  // All wheels use this error class
    }
    int errclass(int locat) { return errclass(static_cast<loc>(locat)); }
    float val(int locat) { return val(static_cast<loc>(locat)); }
    bool detected(int locat) { return detected(static_cast<loc>(locat)); }
    src source() { return src::PIN; }
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