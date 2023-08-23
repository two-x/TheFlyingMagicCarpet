#ifndef IDIOT_LIGHT_PARAMS_H
#define IDIOT_LIGHT_PARAMS_H

#include "devices.h"
#include "TemperatureSensorManager.h"

struct IdiotLightParams {
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
};

#endif // IDIOT_LIGHT_PARAMS_H