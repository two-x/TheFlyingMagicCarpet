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
    std::map<IdiotLight::Name, IdiotLight> idiot_lights;
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
        idiot_lights.emplace(IdiotLight::Name::START, IdiotLight([this](IdiotLight& light){return *(this->starter);}, GRN, "St"));
        idiot_lights.emplace(IdiotLight::Name::REMOTE_START, IdiotLight([this](IdiotLight& light){return *(this->remote_starting);}, TEAL, "RS"));
        idiot_lights.emplace(IdiotLight::Name::IGNITION_SENSE, IdiotLight([this](IdiotLight& light){return *(this->ignition_sense);}, ORG, "Is"));
        idiot_lights.emplace(IdiotLight::Name::IGNITION, IdiotLight([this](IdiotLight& light){return *(this->ignition);}, YEL, "IG"));
        idiot_lights.emplace(IdiotLight::Name::SYSPOWER, IdiotLight([this](IdiotLight& light){return *(this->syspower);}, GRN, "Pw"));
        idiot_lights.emplace(IdiotLight::Name::SHUTDOWN_COMPLETE, IdiotLight([this](IdiotLight& light){return *(this->shutdown_complete);}, ORG, "Sh"));
        idiot_lights.emplace(IdiotLight::Name::SIMULATOR_ENABLED, IdiotLight([this](IdiotLight& light){return this->simulator->get_enabled();}, PNK, "Sm"));
        idiot_lights.emplace(IdiotLight::Name::RADIO_DETECTED, IdiotLight([this](IdiotLight& light){return *(this->hotrc_radio_detected);}, GRN, "RC"));
        idiot_lights.emplace(IdiotLight::Name::PANIC_STOP, IdiotLight([this](IdiotLight& light){return *(this->panic_stop);}, RED, "Pn"));
        idiot_lights.emplace(IdiotLight::Name::MOTORS_PARKED, IdiotLight([this](IdiotLight& light){return *(this->park_the_motors);}, DPNK, "Pk"));
        idiot_lights.emplace(IdiotLight::Name::CRUISE_ADJUSTING, IdiotLight([this](IdiotLight& light){return *(this->cruise_adjusting);}, CYN, "Aj"));
        idiot_lights.emplace(IdiotLight::Name::ENGINE_TEMP, IdiotLight([this](IdiotLight& light){return this->check_engine_temp(light);}, ORG, "Eg"));
        idiot_lights.emplace(IdiotLight::Name::WHEEL_TEMP, IdiotLight([this](IdiotLight& light){return this->check_wheel_temp(light);}, ORG, "Wh"));
    }

    IdiotLight get(IdiotLight::Name name) {
        auto it = idiot_lights.find(name);
        if (it != idiot_lights.end()) {
            return it->second;
        } else {
            // Handle the case where `name` doesn't exist in the map.
            // You can throw an exception, return a dummy `IdiotLight` object, etc.
        }
    }

    std::map<IdiotLight::Name, IdiotLight> get_all_lights() {
        return idiot_lights;
    }

    static void update_idiot_lights_wrapper(void *parameter) {
        // Cast the parameter back to an IdiotLightManager instance and call update_idiot_lights
        static_cast<IdiotLightManager*>(parameter)->update_idiot_lights();
    }

    void update_idiot_lights() {
        for (auto& pair : idiot_lights) {
            Serial.printf("Updating idiot light\n");
            pair.second.update();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for a 100ms to avoid updating the sensors too frequently
    }

private:
    // we can put all of the more complicated state functions here to determine if the light should be on or not. 
    bool check_engine_temp(IdiotLight& light) {
        TemperatureSensor* engine_sensor = temperature_sensor_manager->get_sensor(TemperatureSensor::location::ENGINE);
        if (engine_sensor == nullptr) {
            return false; // Return false if the sensor isn't connected
        }

        double current_temp = engine_sensor->get_temperature();
        if (current_temp >= engine_sensor->get_limits().get_alarm()) {
            light.set_severity(IdiotLight::Severity::ALARM);
            return true;
        } else if (current_temp >= engine_sensor->get_limits().get_warning()) {
            light.set_severity(IdiotLight::Severity::WARN);
            return true;
        }

        light.set_severity(IdiotLight::Severity::NONE);
        return false;
    }

    bool check_wheel_temp(IdiotLight& light) {
        TemperatureSensor* wheel_sensor = temperature_sensor_manager->get_sensor(TemperatureSensor::location::WHEEL_FL);
        if (wheel_sensor == nullptr) {
            return false; // Return false if the sensor isn't connected
        }

        double current_temp = wheel_sensor->get_temperature();
        if (current_temp >= wheel_sensor->get_limits().get_alarm()) {
            light.set_severity(IdiotLight::Severity::ALARM);
            return true;
        } else if (current_temp >= wheel_sensor->get_limits().get_warning()) {
            light.set_severity(IdiotLight::Severity::WARN);
            return true;
        }

        light.set_severity(IdiotLight::Severity::NONE);
        return false;
    }
};

#endif // IDIOT_LIGHT_MANAGER_H