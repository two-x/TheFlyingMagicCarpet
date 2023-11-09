// control.h : centralized wrapper classes for all braking or throttle activity, as holistic coordination between sensors, actuator, and intent is critical
#pragma once
#include <ESP32Servo.h>  // Eventually move Servos into new ServoPWM objects then remove this
#include "common.h"
#include "devices.h"
#include "qpid.h"
// MotorManager - a parent class providing common elements for motor manager child classes. These subclasses each
// coordinate all aspects of one motor, including related sensors & pid, having a standard control interface
class ServoMotor {
  protected:
    Hotrc* hotrc;
    qpid* mypid;
    Servo* motor;  // ServoPWM* motor;
    bool job_hop, faultsnow, faultslast;
    int job_last = na;
    int8_t motor_pin;
    float initial_spid_kp, initial_spid_ki_hz, initial_spid_kd_s;
    uint32_t pid_period_us = 100000;    // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    uint32_t out_timeout = 85000;
    Timer out_timer, pid_timer;
    void do_autostop() {}
    void do_park() {}
    void do_cal() {}
    void do_control() {}
    void do_autocal() {}
    void do_release() {}
    void do_halt(bool immediate = false) {}
    bool fault_filter() { return false; }
  public:
    int mode = pid, job = na;
    float duty_pc = 100, pc_per_nat;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1).
    float pc[num_valus] = { 0, 0, 100, 0, 0, -100, 100, 2 };  // percent values [opmin/stop/opmax/out/parked/absmin/absmax/margin]  values range from -100% to 100% are all derived or auto-assigned
    float nat[num_valus] = { 0, 90, 180, 90, 90, 0, 180, 1.8 };  // native-unit values [opmin/stop/opmax/out/parked/absmin/absmax/margin]
    float us[num_valus] = { 500, 1500, 2500, 1500, 1500, 500, 2500, 20 };  // us pulsewidth values [opmin/stop/opmax/out/parked/absmin/absmax/margin]
    bool motor_reversed = true;
    void init(Hotrc* _hotrc) {
        this->hotrc = _hotrc;
        this->derive();
    }
    int set_job(int _cmd) {
        if ((_cmd != na) && (_cmd != this->job)) {
            this->job_last = this->job;
            this->job = _cmd;
            this->job_hop = true;
        }
        return this->job;
    }
    bool isactive() { return (this->job != na && this->job != halt); }
    void set_mode(int _newmode) { this->mode = _newmode; }
    int update(int _cmd = na, bool vip = false) {
        this->set_job(_cmd);
        if (this->out_timer.expireset() || this->job_hop || vip) {
            if (this->job == halt) this->do_halt();
            else if (this->job == pid) this->do_control();
            else if (this->job == park) this->do_park();
            else if (this->job == release) this->do_release();
            else if (this->job == autostop) this->do_autostop();
            else if (this->job == cal) this->do_cal();
            else if (this->job == autocal) this->do_autocal();
            if (this->fault_detect()) {}
            this->constrain_pc();
            this->write();
            this->job_hop = false;
        }
        return this->job;
    }
    void derive() {  // calc derived limit values for all units based on tuned values for each motor
        this->pc[absmin] = map(this->nat[absmin], this->nat[opmin], this->nat[opmax], this->pc[opmin], this->pc[opmax]);
        this->pc[absmax] = map(this->nat[absmax], this->nat[opmin], this->nat[opmax], this->pc[opmin], this->pc[opmax]);
        this->pc[parked] = map(this->nat[parked], this->nat[opmin], this->nat[opmax], this->pc[opmin], this->pc[opmax]);
        
        this->pc[opmax] = this->pc[absmax] * this->duty_pc / 100.0;
        this->nat[opmin] = this->nat[stop] - this->duty_pc * (this->nat[stop] - this->nat[absmin]) / 100.0;  // Brake pulsewidth corresponding to duty-constrained retraction of brake actuator (in us). Default setting for jaguar is max 670us
        this->nat[opmax] = this->nat[stop] - this->duty_pc * (this->nat[stop] - this->nat[absmax]) / 100.0;  // Brake pulsewidth corresponding to duty-constrained extension of brake actuator (in us). Default setting for jaguar is max 2330us
        this->pc_per_nat = (this->pc[absmax] - this->pc[absmin]) / (this->nat[opmin] - this->nat[opmax]);  // (100 - 0) percent / (nat-max - nat-min) nat = 1/8.3 = 0.12 percent/nat
    }
    float pc_to_nat(float _pc) {  // Eventually this should be linearized
        // return this->nat[absmin] + _pc * (this->nat[absmax] - this->nat[absmin]) / 100.0;  // gas closed is min
        return map(_pc, this->pc[absmin], this->pc[absmax], this->nat[absmin], this->nat[absmax]);
    }
    float nat_to_pc(float _nat) {  // Eventually this should be linearized
        // return this->pc[absmax] * (_nat - this->nat[absmin]) / (this->nat[absmax] - this->nat[absmin]);
        return map(_nat, this->nat[absmin], this->nat[absmax], this->pc[absmin], this->pc[absmax]);
    }
    float nat_to_us(float _nat) {  // works for motor without stop value
        return map(_nat, this->nat[absmin], this->nat[absmax], this->motor_reversed ? this->us[absmax] : this->us[absmin], this->motor_reversed ? this->us[absmin] : this->us[absmax]);
    }
    float pc_to_us(float _pc) {  // works for motor without stop value
        return map(_pc, this->pc[absmin], this->pc[absmax], this->motor_reversed ? this->us[absmax] : this->us[absmin], this->motor_reversed ? this->us[absmin] : this->us[opmax]);
    }
    void to_us() {
        this->nat[out] = this->pc_to_nat(this->pc[out]);
        this->us[out] = this->pc_to_us(this->pc[out]);        
    }
  protected:
    void constrain_pc() {
        constrain(this->pc[out], this->pc[opmin], this->pc[opmax]);
    }
    void write() {  // Write result to jaguar servo interface
        this->to_us();
        this->motor->writeMicroseconds((int32_t)this->us[out]);
    }
};
// void brake_calc_duty(float duty) {  // call from setup and whenever changing duty cycle
//     brake_extend_min_pc = brake_extend_absmin_pc * duty / 100.0;     // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us). Default setting for jaguar is max 2330us
//     brake_retract_max_pc = brake_retract_absmax_pc * duty / 100.0;     // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us). Default setting for jaguar is max 670us
//     brake_extend_min_us = brake_stop_us - duty * (brake_stop_us - brake_extend_absmin_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained retraction of brake actuator (in us). Default setting for jaguar is max 670us
//     brake_retract_max_us = brake_stop_us - duty * (brake_stop_us - brake_retract_absmax_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained extension of brake actuator (in us). Default setting for jaguar is max 2330us
// }
class JagMotor : public ServoMotor {
  public:
    float pc[num_valus] = { -100, 0, 100, 0, 0, -100, 100, 2 };  // percent values [opmin/stop/opmax/out/parked/absmin/absmax/margin]  values range from -100% to 100% are all derived or auto-assigned
    float nat[num_valus] = { -12, 0, 12, 0, 0, -12.0, 12.0, 0.24 };  // native-unit values [opmin/stop/opmax/out/parked/absmin/absmax/margin]
    float us[num_valus] = { -1, 1500, -1, 1500, 1500, 670, 2330, 16.6 };  // us pulsewidth values [opmin/stop/opmax/out/parked/absmin/absmax/margin]
    float (&volt)[arraysize(nat)] = nat;  // our "native" value is volts
    float &pc_per_volt = pc_per_nat;
    bool motor_reversed = false;
  protected:
    void derive() {  // calc pc and voltage op limits from volt and us abs limits 
        this->pc[opmin] = this->pc[absmin] * this->duty_pc / 100.0;
        this->pc[opmax] = this->pc[absmax] * this->duty_pc / 100.0;
        this->volt[opmin] = this->volt[stop] - this->duty_pc * (this->volt[stop] - this->volt[absmin]) / 100.0;  // Brake pulsewidth corresponding to duty-constrained retraction of brake actuator (in us). Default setting for jaguar is max 670us
        this->volt[opmax] = this->volt[stop] - this->duty_pc * (this->volt[stop] - this->volt[absmax]) / 100.0;  // Brake pulsewidth corresponding to duty-constrained extension of brake actuator (in us). Default setting for jaguar is max 2330us
        this->pc_per_volt = (this->pc[absmax] - this->pc[absmin]) / (this->volt[opmin] - this->volt[opmax]);  // (100 - 0) percent / (nat-max - nat-min) nat = 1/8.3 = 0.12 percent/nat
    }
    float pc_to_nat(float _pc) {  // Override these conversion functions
        if (_pc > this->pc[stop]) return map(_pc, this->pc[stop], this->pc[absmax], this->nat[stop], this->nat[absmax]);
        if (_pc < this->pc[stop]) return map(_pc, this->pc[stop], this->pc[absmin], this->nat[stop], this->nat[absmin]);
        return this->nat[stop];
    }
    float nat_to_pc(float _nat) {  // Eventually this should be linearized
        if (_nat > this->nat[stop]) return map(_nat, this->nat[stop], this->nat[absmax], this->pc[stop], this->pc[absmax]);
        if (_nat < this->nat[stop]) return map(_nat, this->nat[stop], this->nat[absmin], this->pc[stop], this->pc[absmin]);
        return this->pc[stop];
    }
    float nat_to_us(float _nat) {  // works for motor without stop value
        if (_nat > this->nat[stop]) return map(_nat, this->nat[stop], this->nat[absmax], this->us[stop], this->motor_reversed ? this->us[absmin] : this->us[absmax]);
        if (_nat < this->nat[stop]) return map(_nat, this->nat[stop], this->nat[absmin], this->us[stop], this->motor_reversed ? this->us[absmax] : this->us[absmin]);
        return this->us[stop];
    }
    float pc_to_us(float _pc) {  // works for motor without stop value
        if (_pc > this->pc[stop]) return map(_pc, this->pc[stop], this->pc[absmax], this->us[stop], this->motor_reversed ? this->us[absmin] : this->us[absmax]);
        if (_pc < this->pc[stop]) return map(_pc, this->pc[stop], this->pc[absmin], this->us[stop], this->motor_reversed ? this->us[absmax] : this->us[absmin]);
        return this->us[stop];
    }
};
class Gas : public ServoMotor {
  public:
    Gas() {};  // Brake(int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin); 
    void init(Hotrc* _hotrc, Servo* _motor, qpid* _pid, Tachometer* _tach, Potentiometer* _pot) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        hotrc = _hotrc;
        motor = _motor;        // motor_pin = _motor_pin;
        tach = _tach;  // press_pin = _press_pin;
        pot = _pot;
        mypid = _pid;
        derive();
        pid_timer.set(pid_period_us);
        out_timer.set(out_timeout);
    }
  private:
    float (&deg)[arraysize(nat)] = nat;  // our "native" value is degrees of rotation "deg"
    Tachometer* tach;
    Potentiometer* pot;
    void do_halt(bool immediate) {}
    void do_control() {
        if (mode == pid) {
            pc[out] = mypid->compute();
        }
        else if (mode == manual) {  // pc[opmin] is idle speed, pc[opmax] is governor
            if (joydir() != joy_up) pc[out] = pc[opmin];  // If in deadband or being pushed down, we want idle
            else pc[out] = map(hotrc->pc[vert][filt], hotrc->pc[vert][dbtop], hotrc->pc[vert][opmax], 0.0, pc[opmax]);  // actuators still respond even w/ engine turned off
        }
    }
    void do_park() {

    }
    void do_release() {

    }
    void do_autostop() {
        if (job_hop && !autostop_disabled) throttle.goto_idle();  // Keep target updated to possibly changing idle value
        else set_mode(na);
    }
    void do_cal() {
        float temp = pot->mapToRange(0.0, 180.0);
    }
    void do_autocal() {}
};
class Brake : public JagMotor {
  public:
    Brake() {}  // Brake(int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin); 
    bool autostopping = false;
    static const uint32_t interval_timeout = 1000000;  // How often to apply increment during auto-stopping (in us)
    static const uint32_t stopcar_timeout = 8000000;  // How often to apply increment during auto-stopping (in us)
    Timer brakeIntervalTimer, stopcarTimer;             // How much time between increasing brake force during auto-stop if car still moving?    // How long before giving up on trying to stop car?
    void init(Hotrc* _hotrc, Servo* _motor, qpid* _pid, PressureSensor* _pressure, BrakePositionSensor* _brakepos, Speedometer* _speedo) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        hotrc = _hotrc;
        motor = _motor;        // motor_pin = _motor_pin;
        pressure = _pressure;  // press_pin = _press_pin;
        brakepos = _brakepos;  // posn_pin = _posn_pin;
        mypid = _pid;
        speedo = _speedo;
        duty_pc = 25.0;  // From motor datasheet
        derive();
        pid_timer.set(pid_period_us);
        interval_timer.set(interval_timeout);
        out_timer.set(out_timeout);
    }
  private:
    BrakePositionSensor* brakepos;
    PressureSensor* pressure;
    Speedometer* speedo;
    Timer interval_timer;  // Interval: How much time between increasing brake force during auto-stop if car still moving?
    float retract_effective_max_us;   // 
    uint32_t pid_period_us = 85000;    // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    uint32_t out_timeout = 85000;
    float initial_spid_kp = 0.323;     // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float initial_spid_ki_hz = 0.000;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float initial_spid_kd_s = 0.000;   // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    // void do_autostop(), do_park(), do_cal(), do_control(), do_autocal(), do_release(), do_halt(bool immediate = false);
    void constrain_out() {
        if ((job == cal) || (job == autocal)) pc[out] = constrain(pc[out], pc[absmin], pc[absmax]);  // Constrain to full potential range when calibrating. Caution don't break anything!
        else pc[out] = constrain(pc[out], pc[opmin], pc[opmax]);
        volt[out] = pc_to_nat(pc[out]);
    }
    void write() {
        us[out] = pc_to_us(pc[out])
        motor->writeMicroseconds((int32_t)us[out]);  // Write result to jaguar servo interface
    }
    void do_release() {
        mypid->set_target(pressure->min_human());
    }
    void do_halt(bool immediate) {
        if (immediate) pc[out] = pc[stop];
        else {

        }
    }
    void do_autostop() {
        if (job_hop) {
            if (!speedo->car_stopped() && !autostop_disabled) {
                throttle.goto_idle();  // Keep target updated to possibly changing idle value
                mypid->set_target(smax(pressure->filt(), (panicstop ? pressure->panic_initial_psi : pressure->hold_initial_psi)));
                brakeIntervalTimer.reset();
                stopcarTimer.reset();
                autostopping = true;
            }
            else set_job(na);
            return;
        }
        if (autostopping) {
            if (brakeIntervalTimer.expireset())
                brake_pid.set_target(smin(brake_pid.target() + (panicstop ? pressure->panic_increment_psi : pressure->hold_increment_psi), pressure->max_human()));
            if (speedo->car_stopped() || stopcarTimer.expired()) set_job(na);
            if (job != autostop) {
                mypid->set_target(pressure->min_psi());
                autostopping = false;
            }
        }
    }
    void do_park() {
        if (brakepos->filt() + brakepos->margin() <= brakepos->parkpos())  // If brake is retracted from park point, extend toward park point, slowing as we approach
            pc[out] = map(brakepos->filt(), brakepos->parkpos(), brakepos->min_in(), pc[stop], pc[opmin]);
        else if (brakepos->filt() - brakepos->margin() >= brakepos->parkpos())  // If brake is extended from park point, retract toward park point, slowing as we approach
            pc[out] = map(brakepos->filt(), brakepos->parkpos(), brakepos->max_in(), pc[stop], pc[opmax]);
        else set_job(halt);
    }
    void do_control() {
        if (mode == pid) {
            mypid->set_target(map (hotrc->pc[vert][filt], hotrc->pc[vert][dbbot], hotrc->pc[vert][opmin], pressure->min_human(), pressure->max_human()));  // Scale joystick value to pressure adc setpoint
            pc[out] = mypid->compute();
        }
        else if (mode == manual) {
        }
    }
    void do_cal() {
        if (hotrc->pc[vert][filt] > hotrc->pc[vert][dbtop]) pc[out] = map(hotrc->pc[vert][filt], hotrc->pc[vert][dbtop], hotrc->pc[vert][opmax], pc[stop], pc[opmax]);
        else if (hotrc->pc[vert][filt] < hotrc->pc[vert][dbbot]) pc[out] = map(hotrc->pc[vert][filt], hotrc->pc[vert][opmin], hotrc->pc[vert][dbbot], pc[opmin], pc[stop]);
        else pc[out] = pc[stop];
    }
    void do_autocal() {
        if (job_hop);
    }
    bool fault_filter() {
        faultslast = faultsnow;
        faultsnow = (pc[out] < pc[stop] && brakepos->filt() > brakepos->parkpos() - brakepos->margin())
            || (pc[out] > pc[stop] && brakepos->filt() < brakepos->min_in() + brakepos->margin());  // If brake is at position limits and we're tring to go further, stop the motor
        if (faultsnow) pc[out] = pc[stop];
        return faultsnow;
        // 1. Detect  brake chain is not connected (evidenced by change in brake position without expected pressure changes)
        // 2. Detect obstruction, motor failure, or inaccurate position. Evidenced by motor instructed to move but position not changing even when pressure is low.
        // 3. Detet brake hydraulics failure or inaccurate pressure. Evidenced by normal positional change not causing expected increase in pressure.
        // retract_effective_max_us = volt[stop] + duty_pc * (volt[opmax] - volt[stop]);  // Stores instantaneous calculated value of the effective maximum pulsewidth after attenuation
    }
  
};


// float duty_pc = 25.0;  // From motor datasheet
// float extend_absmin_pc = -100.0; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
// float stop_pc = 0.0;          // Brake pulsewidth corresponding to center point where motor movement stops (in us)
// float retract_absmax_pc = 100.0; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
// float margin_pc = 1.8;        // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated
// float extend_absmin_us = 670; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
// float stop_us = 1500;       // Brake pulsewidth corresponding to center point where motor movement stops (in us)
// float retract_absmax_us = 2330; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
// float out_pc = stop_pc;
// float out_us = stop_us;
// float retract_effective_max_us;   // 
// float extend_min_pc = extend_absmin_pc * duty_pc / 100.0;
// float retract_max_pc = retract_absmax_pc * duty_pc / 100.0;
// float extend_min_us = stop_us - duty_pc * (stop_us - extend_absmin_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained retraction of brake actuator (in us). Default setting for jaguar is max 670us
// float retract_max_us = stop_us - duty_pc * (stop_us - retract_absmax_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained extension of brake actuator (in us). Default setting for jaguar is max 2330us
// float pc_per_us = (100.0 - (-100.0)) / (extend_min_us - retractmx_us);  // (100 - 0) percent / (us-max - us-min) us = 1/8.3 = 0.12 percent/us