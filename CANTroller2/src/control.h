// control.h : centralized wrapper classes for all braking or throttle activity, as holistic coordination between sensors, actuator, and intent is critical
#pragma once
#include <ESP32Servo.h>  // Eventually move Servos into new ServoPWM objects then remove this
#include "common.h"
#include "devices.h"
#include "qpid.h"
// MotorManager - a parent class providing common elements for motor manager child classes. These subclasses each
// coordinate all aspects of one motor, including related sensors & pid, having a standard control interface
class MotorManager {
  protected:
    Hotrc* hotrc;
    qpid* mypid;
    bool change_state, faultsnow, faultslast;
    void do_autostop(), do_park(), do_cal(), do_control(), do_autocal(), do_release(), do_halt(bool immediate = false);
    bool fault_detect();
    int job_last = na;
    int8_t motor_pin;
    float initial_spid_kp = 0.323;     // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float initial_spid_ki_hz = 0.000;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float initial_spid_kd_s = 0.000;   // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    uint32_t pid_period_us = 85000;    // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    Timer out_timer, pid_timer;
  public:
    int mode = pid, job = na;
    void init(Hotrc* _hotrc) { this->hotrc = _hotrc; }
    int set_job(int _cmd) {
        if ((_cmd != na) && (_cmd != this->job)) {
            this->job_last = this->job;
            this->job = _cmd;
            this->change_state = true;
        }
        return this->job;
    }
    bool isactive() { return (this->job != na && this->job != halt); }
    void set_mode(int _newmode) { this->mode = _newmode; }
    int update(int _cmd, bool vip) {
        set_job(_cmd);
        if (this->out_timer.expireset() || this->change_state || vip) {
            if (this->job == halt) do_halt();
            else if (this->job == pid) do_control();
            else if (this->job == park) do_park();
            else if (this->job == release) do_release();
            else if (this->job == autostop) do_autostop();
            else if (this->job == cal) do_cal();
            else if (this->job == autocal) do_autocal();
            if (fault_detect()) {}
            this->change_state = false;
        }
        return this->job;
    }
};

class Brake : public MotorManager {
  public:
    Brake();  // Brake(int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin);
    void init(Hotrc* _hotrc, Servo* _motor, qpid* _brakepid, PressureSensor* _pressure, BrakePositionSensor* _brakepos);
  private:
    BrakePositionSensor* brakepos;
    PressureSensor* pressure;
    Servo* motor;
    int8_t press_pin, posn_pin;
    uint32_t out_timeout = 85000;
    Timer interval_timer;  // Interval: How much time between increasing brake force during auto-stop if car still moving?
    void do_autostop(), do_park(), do_cal(), do_control(), do_autocal(), do_release(), do_halt(bool immediate = false);
    bool fault_detect() {
        // Todo: Detect system faults, such as:
        // 1. The brake chain is not connected (evidenced by change in brake position without expected pressure changes)
        // 2. Obstruction, motor failure, or inaccurate position. Evidenced by motor instructed to move but position not changing even when pressure is low.
        // 3. Brake hydraulics failure or inaccurate pressure. Evidenced by normal positional change not causing expected increase in pressure.

        // To-Do Finish this brake governing calculation
        // brake_retract_effective_us = map(brakepos.filt(), brakepos.zeropoint(), BrakePositionSensor::abs_min_retract_in, )) {    
        // brake_motor_govern_duty_ratio = 0.25;  // 25% = Max motor duty cycle under load given by datasheet. Results in: 1500 + 0.25 * (2330 - 1500) = 1707.5 us max pulsewidth at position = minimum
        retract_effective_max_us = stop_us + duty_pc * (retract_max_us - stop_us);  // Stores instantaneous calculated value of the effective maximum pulsewidth after attenuation

        faultslast = faultsnow;
        return faultsnow;
    }
  public:
    uint32_t interval_timeout = 1000000;  // How often to apply increment during auto-stopping (in us)
    float duty_pc = 25.0;  // From motor datasheet
    float extend_absmin_pc = -100.0; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
    float stop_pc = 0.0;          // Brake pulsewidth corresponding to center point where motor movement stops (in us)
    float retract_absmax_pc = 100.0; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
    float margin_pc = 1.8;        // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated
    float extend_absmin_us = 670; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
    float stop_us = 1500;       // Brake pulsewidth corresponding to center point where motor movement stops (in us)
    float retract_absmax_us = 2330; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
    float out_pc = stop_pc;
    float out_us = stop_us;
    float retract_effective_max_us;   // 
    float extend_min_pc = extend_absmin_pc * duty_pc / 100.0;
    float retract_max_pc = retract_absmax_pc * duty_pc / 100.0;
    float extend_min_us = stop_us - duty_pc * (stop_us - extend_absmin_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained retraction of brake actuator (in us). Default setting for jaguar is max 670us
    float retract_max_us = stop_us - duty_pc * (stop_us - retract_absmax_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained extension of brake actuator (in us). Default setting for jaguar is max 2330us
    // float pc_per_us = (100.0 - (-100.0)) / (extend_min_us - retractmx_us);  // (100 - 0) percent / (us-max - us-min) us = 1/8.3 = 0.12 percent/us
};

Brake::Brake() {}
void Brake::init(Hotrc* _hotrc, Servo* _motor, qpid* _brakepid, PressureSensor* _pressure, BrakePositionSensor* _brakepos) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
    hotrc = _hotrc;
    motor = _motor;        // motor_pin = _motor_pin;
    pressure = _pressure;  // press_pin = _press_pin;
    brakepos = _brakepos;  // posn_pin = _posn_pin;
    mypid = _brakepid;
    pid_timer.set(pid_period_us);
    interval_timer.set(interval_timeout);
    out_timer.set(out_timeout);
}
void Brake::do_release() {
}
void Brake::do_halt(bool immediate) {
    if (immediate) out_pc = stop_pc;
    else {

    }
}
void Brake::do_autostop() {
    if (change_state) {
    }
    // bool autostop(req _cmd = req_na) {  // call this regularly during autostop event
    //     req cmd = _cmd;
    //     if (autostop_disabled) autostopping = false;
    //     else {
    //         if (autostopping) {
    //             if (brakeIntervalTimer.expireset())
    //                 brake_pid.set_target(smin(brake_pid.target() + (panicstop ? pressure_panic_increment_psi : pressure_hold_increment_psi), pressure.max_human()));
    //             if (speedo.car_stopped() || stopcarTimer.expired()) cmd = req_off; 
    //         }
    //         if (cmd == req_tog) cmd = (req)(!autostopping);
    //         if (autostopping && cmd == req_off) {
    //             brake_pid.set_target(pressure.min_psi());
    //             autostopping = false;
    //         }
    //         else if (!autostopping && cmd == req_on && !speedo.car_stopped()) {
    //             throttle.goto_idle();  // Keep target updated to possibly changing idle value
    //             brake_pid.set_target(smax(pressure.filt(), (panicstop ? pressure_panic_initial_psi : pressure_hold_initial_psi)));
    //             brakeIntervalTimer.reset();
    //             stopcarTimer.reset();
    //             autostopping = true;
    //         }
    //     }
    //     return autostopping;
    // }

    // if () job = autoadding;
    // else if () job = holding;
}
void Brake::do_park() {
    if (change_state) job = park;
    if (brakepos->filt() + brakepos->margin() <= brakepos->parkpos())  // If brake is retracted from park point, extend toward park point, slowing as we approach
        out_pc = map(brakepos->filt(), brakepos->parkpos(), brakepos->min_in(), stop_pc, extend_min_pc);
    else if (brakepos->filt() - brakepos->margin() >= brakepos->parkpos())  // If brake is extended from park point, retract toward park point, slowing as we approach
        out_pc = map(brakepos->filt(), brakepos->parkpos(), brakepos->max_in(), stop_pc, retract_max_pc);
    else set_job(halt);

    // bool park_motors(req _cmd = req_na) {  // call this regularly during motor parking event
    //     req cmd = _cmd;
    //     if (park_the_motors) {
    //         bool brake_parked = brakepos->parked();
    //         bool gas_parked = ((std::abs(gas_out_us - gas_ccw_parked_us) < 1) && gasServoTimer.expired());
    //         if ((brake_parked && gas_parked) || motorParkTimer.expired()) cmd = req_off;
    //     }
    //     if (cmd == req_tog) cmd = (req)(!park_the_motors);
    //     if (park_the_motors && cmd == req_off)  park_the_motors = false;
    //     else if (!park_the_motors && cmd == req_on) {
    //         gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
    //         motorParkTimer.reset();  // Set a timer to timebox this effort
    //         park_the_motors = true;
    //     }
    //     return park_the_motors;
    // }
}
void Brake::do_control() {
    if (mode == pid) {

    }
    else if (mode == manual) {

    }
}
void Brake::do_cal() {
    if (hotrc->pc[VERT][FILT] > hotrc->pc[VERT][DBTOP]) out_pc = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBTOP], hotrc->pc[VERT][MAX], stop_pc, retract_max_pc);
    else if (hotrc->pc[VERT][FILT] < hotrc->pc[VERT][DBBOT]) out_pc = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][MIN], hotrc->pc[VERT][DBBOT], extend_min_pc, stop_pc);
    else out_pc = stop_pc;
}
void Brake::do_autocal() {
    if (change_state);
}
// bool Brake::fault_detect() {
//     // Todo: Detect system faults, such as:
//     // 1. The brake chain is not connected (evidenced by change in brake position without expected pressure changes)
//     // 2. Obstruction, motor failure, or inaccurate position. Evidenced by motor instructed to move but position not changing even when pressure is low.
//     // 3. Brake hydraulics failure or inaccurate pressure. Evidenced by normal positional change not causing expected increase in pressure.

//     // To-Do Finish this brake governing calculation
//     // brake_retract_effective_us = map(brakepos.filt(), brakepos.zeropoint(), BrakePositionSensor::abs_min_retract_in, )) {    
//     // brake_motor_govern_duty_ratio = 0.25;  // 25% = Max motor duty cycle under load given by datasheet. Results in: 1500 + 0.25 * (2330 - 1500) = 1707.5 us max pulsewidth at position = minimum
//     retract_effective_max_us = stop_us + duty_pc * (retract_max_us - stop_us);  // Stores instantaneous calculated value of the effective maximum pulsewidth after attenuation

//     faultslast = faultsnow;
//     return faultsnow;
// }