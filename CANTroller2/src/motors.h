// motors.h : centralized wrapper classes for all braking or throttle activity, as holistic coordination between sensors, actuator, and intent is critical
#pragma once
#include <ESP32Servo.h>  // Eventually move Servos into new ServoPWM objects then remove this
#include "qpid.h"
class Throttle {  // Soren - To allow creative control of PID targets in case your engine idle problems need that.
  public:
    enum class idlemodes : uint32_t { direct, control, minimize, num_idlemodes };  // direct: disable idle management.  control: soft landing to idle rpm.  minimize: attempt to minimize idle to edge of instability
    enum targetstates : uint32_t { todrive, driving, droptohigh, droptolow, idling, minimizing, num_states };
    float idlehot = 550.0, idlecold = 775.0, idlehigh = 950.0;  // Idle speed at op_max and op_min engine temps, and elevated rpm above idle guaranteed never to stall
    float margin = 10, idle_absmin = 450.0, idle_absmax = 1000.0;  // High limit of idle speed adjustability
    float idle_rpm, stallpoint, dynamic_rpm, temphot, tempcold, idle_slope_rpmps;
    uint32_t settlerate_rpmps, stallrate_rpmps = 400;  // Engine rpm drops exceeding this much per second are considered a stall in progress
    idlemodes idlemode;
    targetstates targetstate, nextstate;
  protected:
    // String modenames[3] = { "direct", "cntrol", "minimz" };
    // String statenames[4] = { "drivng", "tohigh", "tolow", "tostal" };
    float* target_rpm; float* measraw_rpm; float* measfilt_rpm; float engine_temp_f;
    TemperatureSensor* engine_sensor = nullptr;
    bool we_just_changed_states = true, target_externally_set = false; // bool now_trying_to_idle = false;
    uint32_t index_now, index_last;  // Engine rpm drops exceeding this much per second are considered a stall in progress
    float targetlast_rpm, recovery_boost_rpm = 5;  // How much to increase rpm target in response to detection of stall slope
    // The following are for detecting arhythmic period in tach pulses, which isn't implemented yet
    uint32_t history_depth = 100, tach_idle_timeout_us = 5000000;
    int32_t tach_history_rpm[100];  // Why can't I use [history_depth] here instead of [20] in this instantiation?  c++ is a pain in my ass
    uint32_t timestamps_us[100];
    Timer settleTimer, tachHistoryTimer, tachIdleTimer;  // tachIdleTimer = How often to update tach idle value based on engine temperature
    int64_t readtime_last;
  public:
    Throttle() {}
    void init (float* target, float* measraw, float* measfilt,  // Variable references: idle target, rpm raw, rpm filt
      TemperatureSensor* engine_sensor_ptr,  // Rate to lower idle from high point to low point (in rpm per second)
      float tempcold, float temphot, int32_t settlerate = 100,  // Values for: engine operational temp cold (min) and temp hot (max) in degrees-f
      idlemodes myidlemode = idlemodes::control) {  // Configure idle control to just soft land or also attempt to minimize idle
        target_rpm = target;
        measraw_rpm = measraw;
        measfilt_rpm = measfilt;
        *target_rpm = *measfilt_rpm;
        set_idlehigh (idlehigh);
        idlehot = constrain (idlehot, 0.0, idlehigh);
        stallpoint = idlehot - 1;  // Just to give a sane initial value
        set_idlecold (idlecold);
        set_temphot (temphot);
        set_tempcold (tempcold);        
        calc_idlespeed();
        targetlast_rpm = *target_rpm;
        settlerate_rpmps = settlerate;
        settleTimer.reset();
        idlemode = myidlemode;
        targetstate = driving;
        if (engine_sensor_ptr == nullptr) {
            Serial.println("engine_sensor_ptr is nullptr");
            return;
        }
        engine_sensor = engine_sensor_ptr;
        tachIdleTimer.set(tach_idle_timeout_us);
    }
    void update (void) {  // this should be called to update idle and throttle target values before throttle-related control loop outputs are calculated
        // update engine temp if it's ready
        if (engine_sensor != nullptr) {
            engine_temp_f = engine_sensor->get_temperature();
        }
        antistall();
        calc_idlespeed();  // determine our appropriate idle speed, based on latest engine temperature reading
        targetstate_changer();  // if targetstate was changed, prepare to run any initial actions upon processing our new targetstate algorithm
        if (targetstate == todrive) process_todrive();  // Target is above idle, but currently engine is still idling 
        else if (targetstate == driving) process_driving();  // while throttle is open when driving, we don't mess with the rpm target value
        else if (targetstate == droptohigh) process_droptohigh();  // once the metaphorical foot is taken off the gas, first we let the carb close quickly to a high-idle rpm level (that won't stall)
        else if (targetstate == droptolow) process_droptolow();  // after the rpm hits the high-idle level, we slowly close the throttle further until we reach the correct low-idle speed for the current engine temperature
        else if (targetstate == idling) process_idling();  // maintain the low-idle level, adjusting to track temperature cchanges as appropriate
        else if (targetstate == minimizing) process_minimizing();  // if idlemode == minimize, we then further allow the idle to drop, until we begin to sense irregularity in our rpm sensor pulses
    }
    void goto_idle (void) {  // The gods request the engine should idle now
        if (targetstate == driving) nextstate = (idlemode == idlemodes::direct) ? droptolow : droptohigh;
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
        idle_rpm = map (engine_temp_f, tempcold, temphot, idlecold, idlehot);
        idle_rpm = constrain (idle_rpm, idlehot, idlecold);
    }
    void targetstate_changer (void) {  // If nextstate was changed during last update, or someone externally changed the target, change our targetstate
        if (target_externally_set) {  // If the target has been changed externally, then determine our appropriate targetstate based on target value
            if (*target_rpm > idle_rpm + margin) nextstate = (*measfilt_rpm > idlehigh) ? driving : todrive;
            // else nextstate = (idlemode == idlemodes::minimize) ? minimizing : idling;
        }
        target_externally_set = false;
        we_just_changed_states = (nextstate != targetstate);
        targetstate = nextstate;
    }
    void process_todrive (void) {
        if (we_just_changed_states);  // { printf("todriv "); }
        else if (*measfilt_rpm > idlehigh) nextstate = driving;
    }
    void process_driving (void) {
        // if (we_just_changed_states) { printf("driving "); }
    }
    void process_droptohigh (void) {
        if (we_just_changed_states) { set_target_internal (idlehigh); }  // printf("droptohigh "); }
        else if (*measfilt_rpm <= idlehigh + margin) nextstate = droptolow;  // Done dropping to high idle, next continue dropping to low idle
    }
    void process_droptolow (void) {
        if (we_just_changed_states) { settleTimer.reset(); }  // printf("droptolow "); }
        if (*measfilt_rpm <= idle_rpm + margin) nextstate = (idlemode == idlemodes::minimize) ? minimizing : idling;  // Done dropping to low idle, next proceed to a steady state
        else {  // Drop from current rpm toward low idle speed at configured rate
            set_target_internal (*measfilt_rpm - settlerate_rpmps * (float)settleTimer.elapsed()/1000000);  // Need to review the dynamics of this considering update frequency and motor latency 
            settleTimer.reset();
        }
    }
    void process_idling (void) {  // If we aren't set to attempt to minimize idle speed, then we end up here
        // if (we_just_changed_states) {       printf("idling "); }
        if (idlemode == idlemodes::minimize) nextstate = minimizing;  // in case idlemode is changed while in idling state
        set_target_internal (idle_rpm);  // We're done dropping to the idle point, but keep tracking as idle speed may change
    }
    void process_minimizing (void) {
        if (we_just_changed_states) { stallpoint = idle_rpm; }  // printf("minimizing "); }
        else if (idlemode != idlemodes::minimize) nextstate = idling;  // in case idlemode is changed while in stallpoint state
        // else if (*measfilt_rpm > )
        // Soren finish writing this
    }
    void antistall (void) {
        idle_slope_rpmps = (float)(tach_history_rpm[index_now] - tach_history_rpm[index_last]) * 1000000 / timestamps_us[index_now];
        if (*measfilt_rpm <= idlehigh && idle_slope_rpmps < stallrate_rpmps) set_target_internal (idle_rpm + recovery_boost_rpm);
        // Soren:  This is rather arbitrary and unlikely to work. Need to determine anti-stall strategy
    }
    // String get_modename (void) { return modenames[(int32_t)idlemode].c_str(); }
    // String get_statename (void) { return statenames[targetstate].c_str(); }
  public:
    void cycle_idlemode (int32_t cycledir) {  // Cycldir positive or negative
        if (cycledir) idlemode = (idlemodes)(constrain ((int32_t)idlemode + constrain (cycledir, -1, 1), 0, (int32_t)idlemodes::num_idlemodes - 1));
    }
    void set_idlehigh (float newidlehigh) { idlehigh = constrain (newidlehigh, idlecold + 1, idle_absmax); }
    void add_idlehigh (float add) { idlehigh += add; }
    void add_idlehot (float add) { idlehot += add; }
    void add_idlecold (float add) { idlecold += add; }
    void add_temphot (float add) { temphot += add; }
    void add_tempcold (float add) { tempcold += add; }
    void add_settlerate (int32_t add) { settlerate_rpmps += add; }
    void set_target_ptr (float* __ptr) { target_rpm = __ptr; }
    void set_idlehot (float newidlehot) { 
        idlehot = constrain (newidlehot, stallpoint, idlecold - 1);
        calc_idlespeed();
    }
    void set_idlecold (float newidlecold) { 
        idlecold = constrain (newidlecold, idlehot + 1, idlehigh - 1);
        calc_idlespeed();
    }
    void set_temphot (float newtemphot) { 
        if (newtemphot > tempcold) temphot = newtemphot;
        calc_idlespeed();
    }
    void set_tempcold (float newtempcold) { 
        if (newtempcold < temphot) tempcold = newtempcold;
        calc_idlespeed();
    }
    // Getter functions
    float target (void) { return *target_rpm; }
    float* target_ptr (void) { return target_rpm; }
};
// ServoMotor - a parent class providing common elements for motor manager child classes. These subclasses each
// coordinate all aspects of one motor, including related sensors & pid, having a standard control interface
class ServoMotor {
  protected:
    Hotrc* hotrc;
    Speedometer* speedo;
    Servo motor;
    Timer pid_timer;
    int pin;
    uint32_t pid_period_us = 85000;
  public:
    bool open_loop = true, motor_reversed = false;
    float pc[num_motorvals] = { 0, NAN, 100, NAN, NAN, NAN, NAN };  // percent values [opmin/parked/opmax/out/govern/absmin/absmax]  values range from -100% to 100% are all derived or auto-assigned
    float nat[num_motorvals] = { 45.0, 43.0, 168.2, 45.0, NAN, 0, 180 };  // native-unit values [opmin/parked/opmax/out/govern/absmin/absmax]
    float us[num_motorvals] = { NAN, 1500, NAN, NAN, NAN, 500, 2500 };  // us pulsewidth values [-/cent/-/out/-/absmin/absmax]
    void init(int _pin, int _freq, Hotrc* _hotrc, Speedometer* _speedo) {
        this->hotrc = _hotrc;
        this->speedo = _speedo;
        this->pin = _pin;
        this->motor.setPeriodHertz(_freq);
        this->motor.attach(this->pin, this->us[absmin], this->us[absmax]);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
        this->pid_timer.set(this->pid_period_us);
    }
    float pc_to_nat(float _pc) {  // Eventually this should be linearized
        return map(_pc, this->pc[absmin], this->pc[absmax], this->nat[absmin], this->nat[absmax]); 
    }
    float nat_to_pc(float _nat) {  // Eventually this should be linearized
        return map(_nat, this->nat[absmin], this->nat[absmax], this->pc[absmin], this->pc[absmax]);
    }
    float nat_to_us(float _nat) {  // works for motor with or without stop value
        return map(_nat, this->nat[absmin], this->nat[absmax], this->motor_reversed ? this->us[absmax] : this->us[absmin], this->motor_reversed ? this->us[absmin] : this->us[absmax]);
    }
    float pc_to_us(float _pc) {  // works for motor with or without stop value
        return map(_pc, this->pc[absmin], this->pc[absmax], this->motor_reversed ? this->us[absmax] : this->us[absmin], this->motor_reversed ? this->us[absmin] : this->us[opmax]);
    }
    void write_motor() {
        this->motor.writeMicroseconds((int32_t)(this->us[out]));
    }
};
class GasServo : public ServoMotor {
  private:
    Tachometer* tach;
    Potentiometer* pot;
    Throttle* throttle;
    uint32_t servo_delay_us = 500000; // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    float cruise_initial_kp = 5.57;   // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
    float cruise_initial_ki = 0.000;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
    float cruise_initial_kd = 0.000;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
    float initial_kp = 0.013;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
    float initial_ki = 0.000;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
    float initial_kd = 0.000;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
  public:
    float (&deg)[arraysize(nat)] = nat;  // our "native" value is degrees of rotation "deg"
    qpid pid, cruisepid;
    float cruise_target_pc, governor = 95;     // Software governor will only allow this percent of full-open throttle (percent 0-100)
    Timer servo_delay_timer;    // We expect the servo to find any new position within this time
    GasServo() {};  // Brake(int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin); 
    void derive() {  // calc derived limit values for all units based on tuned values for each motor
        pc[absmin] = map(nat[absmin], nat[opmin], nat[opmax], pc[opmin], pc[opmax]);
        pc[absmax] = map(nat[absmax], nat[opmin], nat[opmax], pc[opmin], pc[opmax]);
        pc[parked] = map(nat[parked], nat[opmin], nat[opmax], pc[opmin], pc[opmax]);
        pc[govern] = governor * pc[opmax] / 100.0;        
        nat[govern] = map(pc[govern], pc[opmin], pc[opmax], nat[opmin], nat[opmax]);
    }
    void init(int _pin, int _freq, Hotrc* _hotrc, Speedometer* _speedo, Tachometer* _tach, Potentiometer* _pot, Throttle* _throttle) {
        ServoMotor::init(_pin, _freq, _hotrc, _speedo);
        tach = _tach;
        pot = _pot;
        throttle = _throttle;
        pid.init(tach->filt_ptr(), 0.0, 100.0, initial_kp, initial_ki, initial_kd, qpid::pmod::onerr, qpid::dmod::onerr, 
            qpid::awmod::clamp, qpid::cdir::direct, pid_period_us);
        cruisepid.init(speedo->filt_ptr(), throttle->idle_rpm, tach->govern_rpm(), cruise_initial_kp, cruise_initial_ki, cruise_initial_kd,
            qpid::pmod::onerr, qpid::dmod::onerr, qpid::awmod::round, qpid::cdir::direct, pid_period_us);
        servo_delay_timer.set(servo_delay_us);
        derive();
    }
    void update(int _runmode) {
        if (pid_timer.expireset()) {
            // Step 1 : update throttle target from idle control or cruise mode pid, if applicable (on the same timer as gas pid)
            throttle->update();  // Allow idle control to mess with tach_target if necessary, or otherwise step in to prevent car from stalling
            if (_runmode == CRUISE && (cruise_setpoint_mode == pid_suspend_fly) && !cruise_adjusting) {
                cruisepid.set_outlimits(throttle->idle_rpm, tach->govern_rpm());  // because cruise pid has internal variable for idlespeed which may need updating
                pid.set_target(cruisepid.compute());  // cruise pid calculates new output (tach_target_rpm) based on input (speedmeter::human) and target (speedo_target_mph)
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
                pc[out] = pid.compute();  // Do proper pid math to determine gas_out_us from engine rpm error
            }
            // Step 3 : Convert to degrees and constrain if out of range
            deg[out] = pc_to_nat(pc[out]);  // convert to degrees
            if (_runmode == CAL && cal_pot_gasservo_mode)  // Constrain to operating limits. 
                deg[out] = constrain(deg[out], deg[absmin], deg[absmax]);
            else if (_runmode == BASIC || _runmode == SHUTDOWN)
                deg[out] = constrain(deg[out], deg[parked], nat[govern]);
            else deg[out] = constrain(deg[out], deg[opmin], deg[govern]);
            pc[out] = nat_to_pc(deg[out]);
            // Step 4 : Write to servo
            us[out] = nat_to_us(deg[out]);
            if (!((_runmode == BASIC && !park_the_motors) || (_runmode == CAL && !cal_pot_gasservo_mode) 
               || (_runmode == SHUTDOWN && !shutdown_incomplete) || (_runmode == ASLEEP)))
                write_motor();
        }
    }
};
class JagMotor : public ServoMotor {
  protected:
    CarBattery* mulebatt;
    static constexpr float car_batt_fake_v = 12.0;
    uint32_t volt_check_period_us = 3500000;
    Timer volt_check_timer;
  public:
    float duty_pc = 100;
    float pc[num_motorvals] = { NAN, 0, NAN, NAN, NAN, -100, 100 };  // percent values [opmin/stop/opmax/out/-/absmin/absmax]  values range from -100% to 100% are all derived or auto-assigned
    float nat[num_motorvals] = { NAN, 0, NAN, NAN, NAN, NAN, NAN };  // native-unit values [opmin/stop/opmax/out/-/absmin/absmax]
    float us[num_motorvals] = { NAN, 1500, NAN, NAN, NAN, 670, 2330 };  // us pulsewidth values [-/cent/-/out/-/absmin/absmax]
    float (&volt)[arraysize(nat)] = this->nat;  // our "native" value is volts
    bool motor_reversed = false;
    void derive() {  // calc pc and voltage op limits from volt and us abs limits 
        this->nat[absmax] = running_on_devboard ? this->car_batt_fake_v : this->mulebatt->v();
        this->nat[absmin] = -(this->nat[absmax]);
        this->pc[opmin] = this->pc[absmin] * this->duty_pc / 100.0;
        this->pc[opmax] = this->pc[absmax] * this->duty_pc / 100.0;
        this->nat[opmin] = map(this->pc[opmin], this->pc[stop], this->pc[absmin], this->nat[stop], this->nat[absmin]);
        this->nat[opmax] = map(this->pc[opmax], this->pc[stop], this->pc[absmax], this->nat[stop], this->nat[absmax]);
    }
    void init(int _pin, int _freq, Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {
        ServoMotor::init(_pin, _freq, _hotrc, _speedo);
        this->mulebatt = _batt;
        this->volt_check_timer.set(this->volt_check_period_us);
        this->derive();
    }
    float pc_to_nat(float _pc) {  // Eventually this should be linearized
        if (_pc > this->pc[stop]) return map(_pc, this->pc[stop], this->pc[absmax], this->nat[stop], this->nat[absmax]);
        if (_pc < this->pc[stop]) return map(_pc, this->pc[stop], this->pc[absmin], this->nat[stop], this->nat[absmin]);
        return this->nat[stop];
    }
    float nat_to_pc(float _nat) {  // Eventually this should be linearized
        if (_nat > this->nat[stop]) return map(_nat, this->nat[stop], this->nat[absmax], this->pc[stop], this->pc[absmax]);
        if (_nat < this->nat[stop]) return map(_nat, this->nat[stop], this->nat[absmin], this->pc[stop], this->pc[absmin]);
        return this->pc[stop];
    }
    float nat_to_us(float _nat) {  // works for motor with or without stop value
        if (_nat > this->nat[stop]) return map(_nat, this->nat[stop], this->nat[absmax], this->us[stop], this->motor_reversed ? this->us[absmin] : this->us[absmax]);
        if (_nat < this->nat[stop]) return map(_nat, this->nat[stop], this->nat[absmin], this->us[stop], this->motor_reversed ? this->us[absmax] : this->us[absmin]);
        return this->us[stop];
    }
    float pc_to_us(float _pc) {  // works for motor with or without stop value
        if (_pc > this->pc[stop]) return map(_pc, this->pc[stop], this->pc[absmax], this->us[stop], this->motor_reversed ? this->us[absmin] : this->us[absmax]);
        if (_pc < this->pc[stop]) return map(_pc, this->pc[stop], this->pc[absmin], this->us[stop], this->motor_reversed ? this->us[absmax] : this->us[absmin]);
        return this->us[stop];
    }
    void write_motor() {
        this->motor.writeMicroseconds((int32_t)(this->us[out]));
    }
};
class BrakeMotor : public JagMotor {
  public:
    bool autostopping = false;
    float duty_pc = 25.0;
    Timer stopcar_timer, interval_timer;  // How much time between increasing brake force during auto-stop if car still moving?    // How long before giving up on trying to stop car?
    qpid pid;
    BrakeMotor() {}  // Brake(int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin); 
    void init(int _pin, int _freq, Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt, PressureSensor* _pressure, BrakePositionSensor* _brakepos) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        JagMotor::init(_pin, _freq, _hotrc, _speedo, _batt);
        pressure = _pressure;  // press_pin = _press_pin;
        brakepos = _brakepos;  // posn_pin = _posn_pin;
        pid.init(pressure->filt_ptr(), pc[opmin], pc[opmax], initial_kp, initial_ki, initial_kd, qpid::pmod::onerr, qpid::dmod::onerr,
            qpid::awmod::cond, qpid::cdir::direct, pid_period_us, qpid::ctrl::manual, qpid::centmod::on, pc[stop]);
        interval_timer.set(interval_timeout);
        stopcar_timer.set(stopcar_timeout);
    }
    void update(int _runmode) {
        // Brakes - Determine motor output and write it to motor
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            // Step 1 : Determine motor percent value
            if (park_the_motors) {
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
                pc[out] = pid.compute();  // Otherwise the pid control is active
            }
            // Step 2 : Fix motor pc value if it's out of range or exceeding positional limits
            if (_runmode == CAL && cal_joyvert_brkmotor_mode)  // Constrain the motor to the operational range, unless calibrating (then constraint already performed above)
                pc[out] = constrain(pc[out], pc[opmin], pc[opmax]);  // Constrain to full potential range when calibrating. Caution don't break anything!
            else if ((pc[out] < pc[stop] && brakepos->filt() > brakepos->parkpos() - brakepos->margin()) 
                  || (pc[out] > pc[stop] && brakepos->filt() < brakepos->min_in() + brakepos->margin()))  // If brake is at position limits and we're tring to go further, stop the motor
                pc[out] = pc[stop];
            else pc[out] = constrain(pc[out], pc[opmin], pc[opmax]);  // Send to the actuator. Refuse to exceed range
            // Step 3 : Convert motor percent value to pulse width for motor, and to volts for display
            us[out] = pc_to_us(pc[out]);
            volt[out] = pc_to_nat(pc[out]);
            // Step 4 : Write to motor
            if (!((_runmode == BASIC && !park_the_motors) || (_runmode == CAL && !cal_pot_gasservo_mode) 
               || (_runmode == SHUTDOWN && !shutdown_incomplete) || (_runmode == ASLEEP)))
                write_motor();
        }
    }
  private:
    BrakePositionSensor* brakepos;
    PressureSensor* pressure;
    float initial_kp = 0.323;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float initial_ki = 0.000;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float initial_kd = 0.000;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    uint32_t pid_period_us = 85000;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    static const uint32_t interval_timeout = 1000000;  // How often to apply increment during auto-stopping (in us)
    static const uint32_t stopcar_timeout = 8000000;  // How often to apply increment during auto-stopping (in us)
    void fault_filter() {
        // 1. Detect  brake chain is not connected (evidenced by change in brake position without expected pressure changes)
        // 2. Detect obstruction, motor failure, or inaccurate position. Evidenced by motor instructed to move but position not changing even when pressure is low.
        // 3. Detet brake hydraulics failure or inaccurate pressure. Evidenced by normal positional change not causing expected increase in pressure.
        // retract_effective_max_us = volt[stop] + duty_pc * (volt[opmax] - volt[stop]);  // Stores instantaneous calculated value of the effective maximum pulsewidth after attenuation
    }
};
class SteerMotor : public JagMotor {
  public:
    float steer_safe_pc = 72.0;
    SteerMotor() {}
    void init(int _pin, int _freq, Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        JagMotor::init(_pin, _freq, _hotrc, _speedo, _batt);
    }
    void update(int _runmode) {
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            if (_runmode == ASLEEP || (_runmode == SHUTDOWN && !shutdown_incomplete)) pc[out] = pc[stop];  // Stop the steering motor if in shutdown mode and shutdown is complete
            else {
                int _joydir = hotrc->joydir(horz);
                if (_joydir == joy_rt) pc[out] = map(hotrc->pc[horz][filt], hotrc->pc[horz][dbtop], hotrc->pc[horz][opmax], pc[stop], steer_safe (pc[opmax]));  // if joy to the right of deadband
                else if (_joydir == joy_lt) pc[out] = map(hotrc->pc[horz][filt], hotrc->pc[horz][dbbot], hotrc->pc[horz][opmin], pc[stop], steer_safe (pc[opmin]));  // if joy to the left of deadband
                else pc[out] = pc[stop];  // Stop the steering motor if inside the deadband
            }
            pc[out] = constrain(pc[out], pc[opmin], pc[opmax]);  // Don't be out of range
            us[out] = pc_to_us(pc[out]);
            volt[out] = pc_to_nat(pc[out]);
            write_motor();
        }
    }
  private:
    float steer_safe(float endpoint) {
        return pc[stop] + (endpoint - pc[stop]) * (1.0 - steer_safe_pc * speedo->filt() / (100.0 * speedo->redline_mph()));
    }
};