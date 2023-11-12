// motors.h : centralized wrapper classes for all braking or throttle activity, as holistic coordination between sensors, actuator, and intent is critical
#pragma once
#include <ESP32Servo.h>  // Eventually move Servos into new ServoPWM objects then remove this
#include "qpid.h"

class Throttle {  // Soren - To allow creative control of PID targets in case your engine idle problems need that.
  public:
    enum class idlemodes : uint32_t { direct, control, minimize, num_idlemodes };  // direct: disable idle management.  control: soft landing to idle rpm.  minimize: attempt to minimize idle to edge of instability
    enum targetstates : uint32_t { todrive, driving, droptohigh, droptolow, idling, minimizing, num_states };
  protected:
    // String modenames[3] = { "direct", "cntrol", "minimz" };
    // String statenames[4] = { "drivng", "tohigh", "tolow", "tostal" };
    targetstates runstate, nextstate;
    idlemodes _idlemode;
    float targetlast_rpm, idle_rpm, idlehigh_rpm, idlehot_rpm, idlecold_rpm, stallpoint_rpm, dynamic_rpm, temphot_f, tempcold_f, idle_slope_rpmps;
    float margin_rpm = 10; float idle_absmax_rpm = 1000.0;  // High limit of idle speed adjustability
    float* target_rpm; float* measraw_rpm; float* measfilt_rpm; float engine_temp_f;
    TemperatureSensor* engine_sensor = nullptr;
    bool we_just_changed_states = true; bool target_externally_set = false; // bool now_trying_to_idle = false;
    uint32_t settlerate_rpmps, index_now, index_last;
    uint32_t stallrate_rpmps = 400;  // Engine rpm drops exceeding this much per second are considered a stall in progress
    float recovery_boost_rpm = 5;  // How much to increase rpm target in response to detection of stall slope
    // The following are for detecting arhythmic period in tach pulses, which isn't implemented yet
    uint32_t history_depth = 100;
    int32_t tach_history_rpm[100];  // Why can't I use [history_depth] here instead of [20] in this instantiation?  c++ is a pain in my ass
    uint32_t timestamps_us[100];
    Timer settleTimer, tachHistoryTimer;
    int64_t readtime_last;
  public:
    Throttle() {}
    void init (float* target, float* measraw, float* measfilt,  // Variable references: idle target, rpm raw, rpm filt
      float idlehigh, float idlecold, float idlehot,  // Values for: high-idle rpm (will not stall), hot idle nominal rpm, cold idle nominal rpm 
      TemperatureSensor* engine_sensor_ptr,  // Rate to lower idle from high point to low point (in rpm per second)
      float tempcold, float temphot, int32_t settlerate = 100,  // Values for: engine operational temp cold (min) and temp hot (max) in degrees-f
      idlemodes myidlemode = idlemodes::control) {  // Configure idle control to just soft land or also attempt to minimize idle
        target_rpm = target;
        measraw_rpm = measraw;
        measfilt_rpm = measfilt;
        *target_rpm = *measfilt_rpm;
        set_idlehigh (idlehigh);
        idlehot_rpm = constrain (idlehot, 0.0, idlehigh_rpm);
        stallpoint_rpm = idlehot_rpm - 1;  // Just to give a sane initial value
        set_idlecold (idlecold);
        set_temphot (temphot);
        set_tempcold (tempcold);        
        calc_idlespeed();
        targetlast_rpm = *target_rpm;
        settlerate_rpmps = settlerate;
        settleTimer.reset();
        _idlemode = myidlemode;
        runstate = driving;
        if (engine_sensor_ptr == nullptr) {
            Serial.println("engine_sensor_ptr is nullptr");
            return;
        }
        engine_sensor = engine_sensor_ptr;
    }
    void update (void) {  // this should be called to update idle and throttle target values before throttle-related control loop outputs are calculated
        // update engine temp if it's ready
        if (engine_sensor != nullptr) {
            engine_temp_f = engine_sensor->get_temperature();
        }
        antistall();
        calc_idlespeed();  // determine our appropriate idle speed, based on latest engine temperature reading
        runstate_changer();  // if runstate was changed, prepare to run any initial actions upon processing our new runstate algorithm
        if (runstate == todrive) process_todrive();  // Target is above idle, but currently engine is still idling 
        else if (runstate == driving) process_driving();  // while throttle is open when driving, we don't mess with the rpm target value
        else if (runstate == droptohigh) process_droptohigh();  // once the metaphorical foot is taken off the gas, first we let the carb close quickly to a high-idle rpm level (that won't stall)
        else if (runstate == droptolow) process_droptolow();  // after the rpm hits the high-idle level, we slowly close the throttle further until we reach the correct low-idle speed for the current engine temperature
        else if (runstate == idling) process_idling();  // maintain the low-idle level, adjusting to track temperature cchanges as appropriate
        else if (runstate == minimizing) process_minimizing();  // if _idlemode == minimize, we then further allow the idle to drop, until we begin to sense irregularity in our rpm sensor pulses
    }
    void goto_idle (void) {  // The gods request the engine should idle now
        if (runstate == driving) nextstate = (_idlemode == idlemodes::direct) ? droptolow : droptohigh;
        // now_trying_to_idle = true;
    }
    void push_tach_reading (int32_t reading, int64_t readtime) {  // Add a new rpm reading to a small LIFO ring buffer. We will use this to detect arhythmic rpm
        if (readtime == readtime_last) return;  // Ignore new tach values unless rpm has changed
        index_last = index_now;
        index_now = (index_now + 1) % history_depth;
        tach_history_rpm[index_now] = reading;
        timestamps_us[index_now] = (uint32_t)(readtime-readtime_last);  // (uint32_t)tachHistoryTimer.elapsed();
        // tachHistoryTimer.reset();
        readtime_last = readtime;
    }
    void set_target (float argtarget) {
        if ((int32_t)(*target_rpm) != (int32_t)argtarget) {
            target_externally_set = true;
            set_target_internal (argtarget);
        }
    }
    void set_engine_sensor (TemperatureSensor* sensor) {
      engine_sensor = sensor;
    }
  protected:
    void set_target_internal (float argtarget) {
        if ((int32_t)(*target_rpm) != (int32_t)argtarget) {
            targetlast_rpm = *target_rpm;
            *target_rpm = argtarget;
        }
    }
    void calc_idlespeed (void) {
        idle_rpm = map (engine_temp_f, tempcold_f, temphot_f, idlecold_rpm, idlehot_rpm);
        idle_rpm = constrain (idle_rpm, idlehot_rpm, idlecold_rpm);
    }
    void runstate_changer (void) {  // If nextstate was changed during last update, or someone externally changed the target, change our runstate
        if (target_externally_set) {  // If the target has been changed externally, then determine our appropriate runstate based on target value
            if (*target_rpm > idle_rpm + margin_rpm) nextstate = (*measfilt_rpm > idlehigh_rpm) ? driving : todrive;
            // else nextstate = (_idlemode == idlemodes::minimize) ? minimizing : idling;
        }
        target_externally_set = false;
        we_just_changed_states = (nextstate != runstate);
        runstate = nextstate;
    }
    void process_todrive (void) {
        if (we_just_changed_states);  // { printf("todriv "); }
        else if (*measfilt_rpm > idlehigh_rpm) nextstate = driving;
    }
    void process_driving (void) {
        // if (we_just_changed_states) { printf("driving "); }
    }
    void process_droptohigh (void) {
        if (we_just_changed_states) { set_target_internal (idlehigh_rpm); }  // printf("droptohigh "); }
        else if (*measfilt_rpm <= idlehigh_rpm + margin_rpm) nextstate = droptolow;  // Done dropping to high idle, next continue dropping to low idle
    }
    void process_droptolow (void) {
        if (we_just_changed_states) { settleTimer.reset(); }  // printf("droptolow "); }
        if (*measfilt_rpm <= idle_rpm + margin_rpm) nextstate = (_idlemode == idlemodes::minimize) ? minimizing : idling;  // Done dropping to low idle, next proceed to a steady state
        else {  // Drop from current rpm toward low idle speed at configured rate
            set_target_internal (*measfilt_rpm - settlerate_rpmps * (float)settleTimer.elapsed()/1000000);  // Need to review the dynamics of this considering update frequency and motor latency 
            settleTimer.reset();
        }
    }
    void process_idling (void) {  // If we aren't set to attempt to minimize idle speed, then we end up here
        // if (we_just_changed_states) {       printf("idling "); }
        if (_idlemode == idlemodes::minimize) nextstate = minimizing;  // in case _idlemode is changed while in idling state
        set_target_internal (idle_rpm);  // We're done dropping to the idle point, but keep tracking as idle speed may change
    }
    void process_minimizing (void) {
        if (we_just_changed_states) { stallpoint_rpm = idle_rpm; }  // printf("minimizing "); }
        else if (_idlemode != idlemodes::minimize) nextstate = idling;  // in case _idlemode is changed while in stallpoint state
        // else if (*measfilt_rpm > )
        // Soren finish writing this
    }
    void antistall (void) {
        idle_slope_rpmps = (float)(tach_history_rpm[index_now] - tach_history_rpm[index_last]) * 1000000 / timestamps_us[index_now];
        if (*measfilt_rpm <= idlehigh_rpm && idle_slope_rpmps < stallrate_rpmps) set_target_internal (idle_rpm + recovery_boost_rpm);
        // Soren:  This is rather arbitrary and unlikely to work. Need to determine anti-stall strategy
    }
    // String get_modename (void) { return modenames[(int32_t)_idlemode].c_str(); }
    // String get_statename (void) { return statenames[runstate].c_str(); }
  public:
    void cycle_idlemode (int32_t cycledir) {  // Cycldir positive or negative
        if (cycledir) _idlemode = (idlemodes)(constrain ((int32_t)_idlemode + constrain (cycledir, -1, 1), 0, (int32_t)idlemodes::num_idlemodes - 1));
    }
    void set_idlemode (idlemodes _idlemode) {} 
    void set_idlehigh (float idlehigh) { idlehigh_rpm = constrain (idlehigh, idlecold_rpm + 1, idle_absmax_rpm); }
    void add_idlehigh (float add) { set_idlehigh(idlehigh_rpm + add); }
    
    void set_idlehot (float idlehot) { 
        idlehot_rpm = constrain (idlehot, stallpoint_rpm, idlecold_rpm - 1);
        calc_idlespeed();
    }
    void add_idlehot (float add) { set_idlehot(idlehot_rpm + add); }

    void set_idlecold (float idlecold) { 
        idlecold_rpm = constrain (idlecold, idlehot_rpm + 1, idlehigh_rpm - 1);
        calc_idlespeed();
    }
    void add_idlecold (float add) { set_idlecold(idlecold_rpm + add); }

    void set_temphot (float temphot) { 
        if (temphot > tempcold_f) temphot_f = temphot;
        calc_idlespeed();
    }
    void add_temphot (float add) { set_temphot(temphot_f + add); }

    void set_tempcold (float tempcold) { 
        if (tempcold < temphot_f) tempcold_f = tempcold;
        calc_idlespeed();
    }
    void add_tempcold (float add) { set_tempcold(tempcold_f + add); }

    void set_settlerate (uint32_t settlerate) { if (settlerate) settlerate_rpmps = settlerate; }
    void add_settlerate (int32_t add) { set_settlerate(settlerate_rpmps + add); }

    void set_margin (float margin) { margin_rpm = margin; }
    void set_target_ptr (float* __ptr) { target_rpm = __ptr; }
    // Getter functions
    targetstates targetstate (void) { return runstate; } 
    idlemodes idlemode (void) { return _idlemode; } 
    uint32_t settlerate (void) { return settlerate_rpmps; }
    float idlehigh (void) { return idlehigh_rpm; }
    float idlehot (void) { return idlehot_rpm; }
    float idlecold (void) { return idlecold_rpm; }
    float temphot (void) { return temphot_f; }
    float tempcold (void) { return tempcold_f; }
    float idlespeed (void) { return idle_rpm; }
    float margin (void) { return margin_rpm; }
    float stallpoint (void) { return stallpoint_rpm; }
    float target (void) { return *target_rpm; }
    float* target_ptr (void) { return target_rpm; }
};
// ServoMotor - a parent class providing common elements for motor manager child classes. These subclasses each
// coordinate all aspects of one motor, including related sensors & pid, having a standard control interface
class ServoMotor {
  protected:
    Hotrc* hotrc;
    Servo motor;  // ServoPWM* motor;
    bool job_hop, faultsnow, faultslast, motor_reversed = true;
    int job_last = na, pin;
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
    float duty_pc = 100, governor = 95, pc_per_nat;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1).
    float pc[num_allvals] = { 0, 0, 100, 0, 0, -100, 100, 2, 100 };  // percent values [opmin/stop/opmax/out/parked/absmin/absmax/margin/govern]  values range from -100% to 100% are all derived or auto-assigned
    float nat[num_allvals] = { 0, 90, 180, 90, 90, 0, 180, 1.8, 180 };  // native-unit values [opmin/stop/opmax/out/parked/absmin/absmax/margin/govern]
    float us[num_valus] = { 500, 1500, 2500, 1500, 1500, 500, 2500, 20 };  // us pulsewidth values [opmin/stop/opmax/out/parked/absmin/absmax/margin]
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
    void update(int _runmode) {}
    // int update(int _cmd = na, bool vip = false) { return this->job; }
    // int update(int _cmd = na, bool vip = false) {
    //     this->set_job(_cmd);
    //     if (this->out_timer.expireset() || this->job_hop || vip) {
    //         if (this->job == halt) this->do_halt();
    //         else if (this->job == pid) this->do_control();
    //         else if (this->job == park) this->do_park();
    //         else if (this->job == release) this->do_release();
    //         else if (this->job == autostop) this->do_autostop();
    //         else if (this->job == cal) this->do_cal();
    //         else if (this->job == autocal) this->do_autocal();
    //         if (this->fault_detect()) {}
    //         this->constrain_pc();
    //         this->write();
    //         this->job_hop = false;
    //     }
    //     return this->job;
    // }
    void derive() {  // calc derived limit values for all units based on tuned values for each motor
        this->pc[absmin] = map(this->nat[absmin], this->nat[opmin], this->nat[opmax], this->pc[opmin], this->pc[opmax]);
        this->pc[absmax] = map(this->nat[absmax], this->nat[opmin], this->nat[opmax], this->pc[opmin], this->pc[opmax]);
        this->pc[parked] = map(this->nat[parked], this->nat[opmin], this->nat[opmax], this->pc[opmin], this->pc[opmax]);
        this->pc[opmax] = this->pc[absmax] * this->duty_pc / 100.0;
        this->pc_per_nat = (this->pc[absmax] - this->pc[absmin]) / (this->nat[absmax] - this->nat[absmin]);  // (100 - 0) percent / (nat-max - nat-min) nat = 1/8.3 = 0.12 percent/nat
        this->pc[govern] = governor * this->pc[opmax] / 100.0;        
        this->nat[govern] = governor * this->nat[opmax] / 100.0;
        // cruisepid.set_outlimits(throttle->idlespeed(), nat[govern]);
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
        this->motor.writeMicroseconds((int32_t)this->us[out]);
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
    float pc[num_allvals] = { -100, 0, 100, 0, 0, -100, 100, 2, 100 };  // percent values [opmin/stop/opmax/out/parked/absmin/absmax/margin/govern]  values range from -100% to 100% are all derived or auto-assigned
    float nat[num_allvals] = { -12, 0, 12, 0, 0, -12.0, 12.0, 0.24, 12 };  // native-unit values [opmin/stop/opmax/out/parked/absmin/absmax/margin/govern]
    float us[num_valus] = { -1, 1500, -1, 1500, 1500, 670, 2330, 16.6 };  // us pulsewidth values [opmin/stop/opmax/out/parked/absmin/absmax/margin]
    float (&volt)[arraysize(nat)] = nat;  // our "native" value is volts
    float &pc_per_volt = pc_per_nat;
    bool motor_reversed = false;
    void derive() {  // calc pc and voltage op limits from volt and us abs limits 
        this->pc[opmin] = this->pc[absmin] * this->duty_pc / 100.0;
        this->pc[opmax] = this->pc[absmax] * this->duty_pc / 100.0;
        this->volt[opmin] = this->volt[stop] - this->duty_pc * (this->volt[stop] - this->volt[absmin]) / 100.0;  // Brake pulsewidth corresponding to duty-constrained retraction of brake actuator (in us). Default setting for jaguar is max 670us
        this->volt[opmax] = this->volt[stop] - this->duty_pc * (this->volt[stop] - this->volt[absmax]) / 100.0;  // Brake pulsewidth corresponding to duty-constrained extension of brake actuator (in us). Default setting for jaguar is max 2330us
        this->pc_per_volt = (this->pc[absmax] - this->pc[absmin]) / (this->volt[opmin] - this->volt[opmax]);  // (100 - 0) percent / (nat-max - nat-min) nat = 1/8.3 = 0.12 percent/nat
        this->pc[govern] = governor * this->pc[opmax] / 100.0;        
        this->nat[govern] = governor * this->nat[opmax] / 100.0;
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
    float (&deg)[arraysize(nat)] = nat;  // our "native" value is degrees of rotation "deg"
    qpid mypid;
    qpid cruisepid;
    float cruise_target_pc;
    Timer servo_delay_timer;    // We expect the servo to find any new position within this time
    float governor = 95;     // Software governor will only allow this percent of full-open throttle (percent 0-100)
    bool open_loop = true;
  private:
    Tachometer* tach;
    Potentiometer* pot;
    Speedometer* speedo;
    Throttle* throttle;
    uint32_t pid_period_us = 85000, cruise_pid_period_us = 85000, servo_delay_us = 500000;                                                                      // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    float cruise_initial_kp = 5.57;                                                                         // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
    float cruise_initial_ki = 0.000;                                                                     // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
    float cruise_initial_kd = 0.000;                                                                      // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
    float gas_initial_kp = 0.013;    // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
    float gas_initial_ki = 0.000; // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
    float gas_initial_kd = 0.000;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
  public:
    void init(int _pin, int _freq, Hotrc* _hotrc, Tachometer* _tach, Speedometer* _speedo, Potentiometer* _pot, Throttle* _throttle) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        hotrc = _hotrc;
        tach = _tach;  // press_pin = _press_pin;
        speedo = _speedo;
        pot = _pot;
        pin = _pin;
        throttle = _throttle;
        deg[opmin] = 45.0;
        deg[opmax] = 168.2;
        deg[parked] = 43.0;
        motor.setPeriodHertz(_freq);
        motor.attach(pin, motor_reversed ? us[absmax] : us[absmin], motor_reversed ? us[absmin] : us[absmax]);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)    
        mypid.init(tach->filt_ptr(), 0.0, 100.0, gas_initial_kp, gas_initial_ki, gas_initial_kd, qpid::pmod::onerr, qpid::dmod::onerr, 
            qpid::awmod::clamp, qpid::cdir::direct, pid_period_us, (open_loop) ? qpid::ctrl::manual : qpid::ctrl::automatic, qpid::centmod::off);
        cruisepid.init(speedo->filt_ptr(), throttle->idlespeed(), tach->govern_rpm, cruise_initial_kp, cruise_initial_ki, cruise_initial_kd,
            qpid::pmod::onerr, qpid::dmod::onerr, qpid::awmod::round, qpid::cdir::direct, cruise_pid_period_us, qpid::ctrl::manual, qpid::centmod::off);
        derive();
        pid_timer.set(pid_period_us);
        out_timer.set(out_timeout);
        servo_delay_timer.set(servo_delay_us);
    }
    // int update(int _cmd = na, bool vip = false) {  // Gas - Update servo output. Determine gas actuator output from rpm target.  PID loop is effective in Fly or Cruise mode.
    void update(int _runmode) {
        if (pid_timer.expireset()) {
            // Step 1 : update throttle target from idle control or cruise mode pid, if applicable (on the same timer as gas pid)
            throttle->update();  // Allow idle control to mess with tach_target if necessary, or otherwise step in to prevent car from stalling
            if (_runmode == CRUISE && (cruise_setpoint_mode == pid_suspend_fly) && !cruise_adjusting) {
                cruisepid.set_outlimits(throttle->idlespeed(), tach->govern_rpm);  // because cruise pid has internal variable for idlespeed which may need updating
                mypid.set_target(cruisepid.compute());  // cruise pid calculates new output (tach_target_rpm) based on input (speedmeter::human) and target (speedo_target_mph)
            }
            // Step 2 : Determine servo pulse width value
            if (park_the_motors || (_runmode == SHUTDOWN && !shutdown_incomplete) || _runmode == ASLEEP)
                pc[out] = pc[parked];
            else if (_runmode == CAL && cal_pot_gasservo_mode)
                pc[out] = nat_to_pc(map(pot->val(), pot->min(), pot->max(), deg[absmin], deg[absmax]));  // gas_ccw_max_us, gas_cw_min_us
            else if (_runmode == CRUISE && (cruise_setpoint_mode != pid_suspend_fly))
                pc[out] = cruise_target_pc;
            else if (_runmode == STALL || (open_loop && _runmode != BASIC)) {  // Stall mode runs the gas servo directly proportional to joystick. This is truly open loop
                if (hotrc->joydir() != joy_up) pc[out] = pc[opmin];  // If in deadband or being pushed down, we want idle
                else pc[out] = map(hotrc->pc[vert][filt], hotrc->pc[vert][dbtop], hotrc->pc[vert][opmax], pc[opmin], pc[govern]);  // actuators still respond even w/ engine turned off
            }
            else if (_runmode != BASIC) {
                pc[out] = mypid.compute();  // Do proper pid math to determine gas_out_us from engine rpm error
            }
            // Step 3 : Convert to degrees and constrain if out of range
            deg[out] = pc_to_nat(pc[out]);  // convert to degrees
            if (_runmode == BASIC || _runmode == SHUTDOWN)
                deg[out] = constrain(deg[out], deg[parked], nat[govern]);
            else if (_runmode == CAL && cal_pot_gasservo_mode)  // Constrain to operating limits. 
                deg[out] = constrain(deg[out], deg[absmin], deg[absmax]);
            else deg[out] = constrain(deg[out], deg[opmin], deg[govern]);
            pc[out] = constrain(pc[out], pc[parked], pc[govern]);
            // Step 4 : Write to servo
            us[out] = nat_to_us(deg[out]);
            if (!((_runmode == BASIC && !park_the_motors) || (_runmode == CAL && !cal_pot_gasservo_mode) || (_runmode == SHUTDOWN && !shutdown_incomplete) || (_runmode == ASLEEP))) {
                motor.writeMicroseconds((int32_t)us[out]);
                // printf("wrote %d\n", (int32_t)us[out]);
            }
        }
    }
  private:
    void do_halt(bool immediate) {}
    void do_control() {
        if (mode == pid) {
            pc[out] = mypid.compute();
        }
        else if (mode == manual) {  // pc[opmin] is idle speed, pc[opmax] is governor
            if (hotrc->joydir() != joy_up) pc[out] = pc[opmin];  // If in deadband or being pushed down, we want idle
            else pc[out] = map(hotrc->pc[vert][filt], hotrc->pc[vert][dbtop], hotrc->pc[vert][opmax], 0.0, pc[opmax]);  // actuators still respond even w/ engine turned off
        }
    }
    void do_park() {}
    void do_release() {}
    void do_autostop() {
        if (job_hop && !autostop_disabled) throttle->goto_idle();  // Keep target updated to possibly changing idle value
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
    qpid mypid;
  private:
    BrakePositionSensor* brakepos;
    PressureSensor* pressure;
    Speedometer* speedo;
    float initial_kp = 0.323;     // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float initial_ki = 0.000;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float initial_kd = 0.000;   // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    uint32_t pid_period_us = 85000;    // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    Timer interval_timer;  // Interval: How much time between increasing brake force during auto-stop if car still moving?
    float retract_effective_max_us;   // 
    uint32_t out_timeout = 85000;
  public:
    void init(int _pin, int _freq, Hotrc* _hotrc, PressureSensor* _pressure, BrakePositionSensor* _brakepos, Speedometer* _speedo) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        hotrc = _hotrc;
        pressure = _pressure;  // press_pin = _press_pin;
        brakepos = _brakepos;  // posn_pin = _posn_pin;
        pin = _pin;
        motor.setPeriodHertz(_freq);
        motor.attach(pin, motor_reversed ? us[absmax] : us[absmin], motor_reversed ? us[absmin] : us[absmax]);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)    
        mypid.init(pressure->filt_ptr(), pc[opmin], pc[opmax], initial_kp, initial_ki, initial_kd, qpid::pmod::onerr, qpid::dmod::onerr,
            qpid::awmod::cond, qpid::cdir::direct, pid_period_us, qpid::ctrl::manual, qpid::centmod::on, pc[stop]);
        speedo = _speedo;
        duty_pc = 25.0;  // From motor datasheet
        derive();
        pid_timer.set(pid_period_us);
        interval_timer.set(interval_timeout);
        out_timer.set(out_timeout);
    }
    // int update(int _cmd = na, bool vip = false) {  // Gas - Update servo output. Determine gas actuator output from rpm target.  PID loop is effective in Fly or Cruise mode.
    void update(int _runmode) {
        // Brakes - Determine motor output and write it to motor
        if (pid_timer.expireset()) {
            // Step 1 : Determine motor percent value
            if (park_the_motors) {
                // if (park_the_motors) brake.do_park();
                if (brakepos->filt() + brakepos->margin() <= brakepos->parkpos())  // If brake is retracted from park point, extend toward park point, slowing as we approach
                    pc[out] = map(brakepos->filt(), brakepos->parkpos(), brakepos->min_in(), pc[stop], pc[opmin]);
                else if (brakepos->filt() - brakepos->margin() >= brakepos->parkpos())  // If brake is extended from park point, retract toward park point, slowing as we approach
                    pc[out] = map(brakepos->filt(), brakepos->parkpos(), brakepos->max_in(), pc[stop], pc[opmax]);
            }
            else if (_runmode == CAL && cal_joyvert_brkmotor_mode) {
                int _joydir = hotrc->joydir();
                if (_joydir == joy_up) pc[out] = map(hotrc->pc[vert][filt], hotrc->pc[vert][dbtop], hotrc->pc[vert][opmax], pc[stop], pc[opmax]);
                else if (_joydir == joy_down) pc[out] = map(hotrc->pc[vert][filt], hotrc->pc[vert][opmin], hotrc->pc[vert][dbbot], pc[opmin], pc[stop]);
                else pc[out] = pc[stop];
            }
            else if (_runmode == CAL || _runmode == BASIC || _runmode == ASLEEP || (_runmode == SHUTDOWN && !shutdown_incomplete))
                pc[out] = pc[stop];
            else {  // First attenuate max power to avoid blowing out the motor like in bm2023, if retracting, as a proportion of position from zeropoint to fully retracted
                pc[out] = mypid.compute();  // Otherwise the pid control is active
            }
            // Step 2 : Fix motor pc value if it's out of range or exceeding positional limits
            if (_runmode == CAL && cal_joyvert_brkmotor_mode)  // Constrain the motor to the operational range, unless calibrating (then constraint already performed above)
                pc[out] = constrain(pc[out], pc[opmin], pc[opmax]);  // Constrain to full potential range when calibrating. Caution don't break anything!
            else if ((pc[out] < pc[stop] && brakepos->filt() > brakepos->parkpos() - brakepos->margin()) || (pc[out] > pc[stop] && brakepos->filt() < brakepos->min_in() + brakepos->margin()))  // If brake is at position limits and we're tring to go further, stop the motor
                pc[out] = pc[stop];
            else pc[out] = constrain(pc[out], pc[opmin], pc[opmax]);  // Send to the actuator. Refuse to exceed range
            // Step 3 : Convert motor percent value to pulse width
            us[out] = pc_to_us(pc[out]);
            // Step 4 : Write to motor
            if (!(_runmode == BASIC && !park_the_motors) && !(_runmode == CAL && !cal_joyvert_brkmotor_mode) && !(_runmode == SHUTDOWN && !shutdown_incomplete) && !(_runmode == ASLEEP)) {
                motor.writeMicroseconds((int32_t)us[out]);  // Write result to jaguar servo interface
            }
        }
    }
  private:
    // void do_autostop(), do_park(), do_cal(), do_control(), do_autocal(), do_release(), do_halt(bool immediate = false);
    void constrain_out() {
        if ((job == cal) || (job == autocal)) pc[out] = constrain(pc[out], pc[absmin], pc[absmax]);  // Constrain to full potential range when calibrating. Caution don't break anything!
        else pc[out] = constrain(pc[out], pc[opmin], pc[opmax]);
        volt[out] = pc_to_nat(pc[out]);
    }
    void write() {
        us[out] = pc_to_us(pc[out]);
        motor.writeMicroseconds((int32_t)us[out]);  // Write result to jaguar servo interface
    }
    void do_release() {
        mypid.set_target(pressure->min_human());
    }
    void do_halt(bool immediate) {
        if (immediate) pc[out] = pc[stop];
        else {}
    }
    void do_autostop() {
        if (job_hop) {
            if (!speedo->car_stopped() && !autostop_disabled) {
                mypid.set_target(smax(pressure->filt(), (panicstop ? pressure->panic_initial_psi : pressure->hold_initial_psi)));
                brakeIntervalTimer.reset();
                stopcarTimer.reset();
                autostopping = true;
            }
            else set_job(na);
            return;
        }
        if (autostopping) {
            if (brakeIntervalTimer.expireset())
                mypid.set_target(smin(mypid.target() + (panicstop ? pressure->panic_increment_psi : pressure->hold_increment_psi), pressure->max_human()));
            if (speedo->car_stopped() || stopcarTimer.expired()) set_job(na);
            if (job != autostop) {
                mypid.set_target(pressure->min_psi());
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
            mypid.set_target(map (hotrc->pc[vert][filt], hotrc->pc[vert][dbbot], hotrc->pc[vert][opmin], pressure->min_human(), pressure->max_human()));  // Scale joystick value to pressure adc setpoint
            pc[out] = mypid.compute();
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