#ifndef IDIOT_LIGHT_MANAGER_H
#define IDIOT_LIGHT_MANAGER_H

#include <map>
#include "IdiotLight.h"
#include "devices.h"
#include "TemperatureSensorManager.h"
#include "colors.h"
#include "IdiotLightParams.h" // todo remove this hack, it's for circular dependencies

class IdiotLightManager {
private:
    std::map<IdiotLightName, IdiotLight> idiot_lights;
    bool* starter;
    bool* remote_starting;
    bool* ignition_sense;
    bool* ignition;
    bool* syspower;
    bool* shutdown_complete;
    Simulator* simulator;
    bool* hotrc_radio_detected;
    bool* panic_stop;
    bool* park_the_motors;
    bool* cruise_adjusting;
    TemperatureSensorManager* temperature_sensor_manager;

public:
    IdiotLightManager(IdiotLightParams params) : 
        starter(params.starter), 
        remote_starting(params.remote_starting), 
        ignition_sense(params.ignition_sense), 
        ignition(params.ignition), 
        syspower(params.syspower), 
        shutdown_complete(params.shutdown_complete), 
        simulator(params.simulator), 
        hotrc_radio_detected(params.hotrc_radio_detected), 
        panic_stop(params.panic_stop), 
        park_the_motors(params.park_the_motors), 
        cruise_adjusting(params.cruise_adjusting),
        temperature_sensor_manager(params.temperature_sensor_manager) {}

    void setup() {
        idiot_lights.emplace(IdiotLightName::START, IdiotLight([this](){return *(this->starter);}, GRN, "St"));
        idiot_lights.emplace(IdiotLightName::REMOTE_START, IdiotLight([this](){return *(this->remote_starting);}, TEAL, "RS"));
        idiot_lights.emplace(IdiotLightName::IGNITION_SENSE, IdiotLight([this](){return *(this->ignition_sense);}, ORG, "Is"));
        idiot_lights.emplace(IdiotLightName::IGNITION, IdiotLight([this](){return *(this->ignition);}, YEL, "IG"));
        idiot_lights.emplace(IdiotLightName::SYSPOWER, IdiotLight([this](){return *(this->syspower);}, GRN, "Pw"));
        idiot_lights.emplace(IdiotLightName::SHUTDOWN_COMPLETE, IdiotLight([this](){return *(this->shutdown_complete);}, ORG, "Sh"));
        idiot_lights.emplace(IdiotLightName::SIMULATOR_ENABLED, IdiotLight([this](){return this->simulator->get_enabled();}, PNK, "Sm"));
        idiot_lights.emplace(IdiotLightName::RADIO_DETECTED, IdiotLight([this](){return *(this->hotrc_radio_detected);}, GRN, "RC"));
        idiot_lights.emplace(IdiotLightName::PANIC_STOP, IdiotLight([this](){return *(this->panic_stop);}, RED, "Pn"));
        idiot_lights.emplace(IdiotLightName::MOTORS_PARKED, IdiotLight([this](){return *(this->park_the_motors);}, DPNK, "Pk"));
        idiot_lights.emplace(IdiotLightName::CRUISE_ADJUSTING, IdiotLight([this](){return *(this->cruise_adjusting);}, CYN, "Aj"));
        idiot_lights.emplace(IdiotLightName::ENGINE_TEMP_WARN, IdiotLight([this](){return this->check_engine_temp_warn();}, ORG, "Eg"));
        idiot_lights.emplace(IdiotLightName::ENGINE_TEMP_ALARM, IdiotLight([this](){return this->check_engine_temp_alarm();}, RED, "Eg"));
        idiot_lights.emplace(IdiotLightName::WHEEL_TEMP_WARN, IdiotLight([this](){return this->check_wheel_temp_warn();}, ORG, "Wh"));
        idiot_lights.emplace(IdiotLightName::WHEEL_TEMP_ALARM, IdiotLight([this](){return this->check_wheel_temp_alarm();}, RED, "Wh"));
    }

    IdiotLight get(IdiotLightName name) {
        auto it = idiot_lights.find(name);
        if (it != idiot_lights.end()) {
            return it->second;
        } else {
            // Handle the case where `name` doesn't exist in the map.
            // You can throw an exception, return a dummy `IdiotLight` object, etc.
        }
    }

    std::map<IdiotLightName, IdiotLight> get_all_lights() {
        return idiot_lights;
    }


private:
    // we can put all of the more complicated state functions here to determine if the light should be on or not. 
    bool check_engine_temp_warn() {
        TemperatureSensor* engine_sensor = temperature_sensor_manager->get_sensor(TemperatureSensor::location::ENGINE);
        if (engine_sensor == nullptr) {
            return false; // Return false if the sensor isn't connected
        }
        return engine_sensor->get_temperature() >= engine_sensor->get_limits().get_warning();
    }

    bool check_engine_temp_alarm() {
        TemperatureSensor* engine_sensor = temperature_sensor_manager->get_sensor(TemperatureSensor::location::ENGINE);
        if (engine_sensor == nullptr) {
            return false; // Return false if the sensor isn't connected
        }
        return engine_sensor->get_temperature() >= engine_sensor->get_limits().get_alarm();
    }

    bool check_wheel_temp_warn() {
        TemperatureSensor* wheel_sensor = temperature_sensor_manager->get_sensor(TemperatureSensor::location::WHEEL_FL);
        if (wheel_sensor == nullptr) {
            return false; // Return false if the sensor isn't connected
        }
        return wheel_sensor->get_temperature() >= wheel_sensor->get_limits().get_warning();
    }

    bool check_wheel_temp_alarm() {
        TemperatureSensor* wheel_sensor = temperature_sensor_manager->get_sensor(TemperatureSensor::location::WHEEL_FL);
        if (wheel_sensor == nullptr) {
            return false; // Return false if the sensor isn't connected
        }
        return wheel_sensor->get_temperature() >= wheel_sensor->get_limits().get_alarm();
    }
};

#endif // IDIOT_LIGHT_MANAGER_H