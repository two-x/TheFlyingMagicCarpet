#ifndef IDIOT_LIGHT_H
#define IDIOT_LIGHT_H

#include <functional> 

class IdiotLight {
public:
    using StateFunction = std::function<bool(IdiotLight&)>;
    enum class Severity {NONE, WARN, ALARM};
    enum class Name {
    START, REMOTE_START, IGNITION_SENSE, IGNITION, SYSPOWER, SHUTDOWN_COMPLETE, MODE_SWITCH, SIMULATOR_ENABLED, RADIO_DETECTED, PANIC_STOP, MOTORS_PARKED, CRUISE_ADJUSTING, ENGINE_TEMP, WHEEL_TEMP };

    IdiotLight(StateFunction state_func, uint16_t color, const char* label)
        : state_func(state_func), color(color), severity(Severity::NONE), label(label), last_state(false) {}

    IdiotLight() = delete; // should always create a light with settings
    
    bool has_changed() const {
        return state_func(*const_cast<IdiotLight*>(this)) != last_state;
    }

    void update() {
        last_state = state_func(*this);
    }

    bool get_state() const {
        return last_state;
    }

    uint16_t get_color() const {
        return color;
    }

    const char* get_label() const {
        return label;
    }

    void set_severity(Severity severity) {
        this->severity = severity;
    }

    Severity get_severity() {
        return severity;
    }

private:
    StateFunction state_func;
    uint16_t color;
    const char* label;
    bool last_state;
    Severity severity;
};

#endif // IDIOT_LIGHT_H