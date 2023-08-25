#ifndef IDIOT_LIGHT_H
#define IDIOT_LIGHT_H

#include <functional> 
#include "colors.h"

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
        if (severity == Severity::WARN) {
            set_color(ORG); // Orange
        } else if (severity == Severity::ALARM) {
            set_color(RED); // Red
        }
    }

    bool get_state() const {
        return last_state;
    }

    uint16_t get_color() const {
        return color;
    }
    
    void set_color(uint16_t color) {
        this->color = color;
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

    void print_details() const {
        printf("Label: %s ", label);
        printf("Color: %s ", color_name(color));
        printf("Last State: %s\n", last_state ? "True" : "False");
    }

    const char* color_name(uint16_t color) const {
        switch(color) {
            case RED: return "Red";
            case ORG: return "Orange";
            case BLK: return "Black";
            case BLU: return "Blue";
            case MBLU: return "Midnight Blue";
            case RBLU: return "Royal Blue";
            case DRED: return "Dark Red";
            case GRN: return "Green";
            case CYN: return "Cyan";
            case DCYN: return "Dark Cyan";
            case MGT: return "Magenta";
            case DORG: return "Dark Orange";
            case YEL: return "Yellow";
            case LYEL: return "Light Yellow";
            case WHT: return "White";
            case DGRY: return "Very Dark Grey";
            case GRY1: return "Dark Grey";
            case GRY2: return "Light Grey";
            case LGRY: return "Very Light Grey";
            case PNK: return "Pink";
            case DPNK: return "Dark Pink";
            case LPNK: return "Light Pink";
            case TEAL: return "Teal";
            case PUR: return "Purple";
            case LPUR: return "Light Pastel Purple";
            case GPUR: return "Greyish Pastel Purple";
        }
    }
    

private:
    StateFunction state_func;
    uint16_t color;
    const char* label;
    bool last_state;
    Severity severity;
};

#endif // IDIOT_LIGHT_H