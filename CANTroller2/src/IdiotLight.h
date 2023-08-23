#ifndef IDIOT_LIGHT_H
#define IDIOT_LIGHT_H

#include <functional> 

enum class IdiotLightName {
    START, REMOTE_START, IGNITION_SENSE, IGNITION, SYSPOWER, SHUTDOWN_COMPLETE, MODE_SWITCH, SIMULATOR_ENABLED, RADIO_DETECTED, PANIC_STOP, MOTORS_PARKED, CRUISE_ADJUSTING, ENGINE_TEMP_WARN, ENGINE_TEMP_ALARM, WHEEL_TEMP_WARN, WHEEL_TEMP_ALARM };

class IdiotLight {
public:
    using StateFunction = std::function<bool()>;

    IdiotLight(StateFunction state_func, uint16_t color, const char* label)
        : state_func(state_func), color(color), label(label), last_state(state_func()) {}

    bool has_changed() const {
        return state_func() != last_state;
    }

    void update() {
        last_state = state_func();
    }

    bool get_state() const {
        return state_func();
    }

    uint16_t get_color() const {
        return color;
    }

    const char* get_label() const {
        return label;
    }

private:
    StateFunction state_func;
    uint16_t color;
    const char* label;
    bool last_state;
};

#endif // IDIOT_LIGHT_H