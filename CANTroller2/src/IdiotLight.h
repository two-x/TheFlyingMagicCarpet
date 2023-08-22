#ifndef IDIOT_LIGHT_H
#define IDIOT_LIGHT_H

#include "globals.h"

class IdiotLight {
public:
    IdiotLight(bool* state, uint16_t color, const char* label)
        : state(state), color(color), label(label), last_state(*state) {}

    bool has_changed() const {
        return *state != last_state;
    }

    void update() {
        last_state = *state;
    }

    bool get_state() const {
        return *state;
    }

    uint16_t get_color() const {
        return color;
    }

    const char* get_label() const {
        return label;
    }

private:
    bool* state;
    uint16_t color;
    const char* label;
    bool last_state;
};

IdiotLight idiot_lights[] = {
    IdiotLight(&starter, GRN, "St"),
    IdiotLight(&remote_starting, TEAL, "RS"),
    IdiotLight(&ignition_sense, ORG, "Is"),
    IdiotLight(&ignition, YEL, "IG"),
    IdiotLight(&syspower, GRN, "Pw"),
    IdiotLight(&shutdown_complete, ORG, "Sh"),
    IdiotLight(&we_just_switched_modes, LYEL, "We"),
    IdiotLight(simulator.get_enabled_ptr(), PNK, "Sm"),
    IdiotLight(&hotrc_radio_detected, GRN, "RC"),
    IdiotLight(&panic_stop, RED, "Pn"),
    IdiotLight(&park_the_motors, DPNK, "Pk"),
    IdiotLight(&cruise_adjusting, CYN, "Aj")
};

#endif // IDIOT_LIGHT_H