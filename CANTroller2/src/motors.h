// motors.h : centralized wrapper classes for all braking or throttle activity, as holistic coordination between sensors, actuator, and intent is critical
#pragma once
#include <ESP32Servo.h>  // Eventually move Servos into new ServoPWM objects then remove this
#include "qpid.h"
#include "temperature.h"
#include <cmath>
class IdleControl {  // Soren - To allow creative control of PID targets in case your engine idle problems need that.
  public:
    enum idlemodes : int { DIRECT, CONTROL, MINIMIZE, NUM_IDLEMODES };  // DIRECT: disable idle management.  CONTROL: soft landing to idle rpm.  MINIMIZE: attempt to minimize idle to edge of instability
    enum targetstates : int { TODRIVE, DRIVING, DROPTOHIGH, DROPTOLOW, IDLING, MINIMIZING, NUM_STATES };
    float idlehot = 550.0, idlecold = 775.0, idlehigh = 950.0;  // Idle speed at op_max and op_min engine temps, and elevated rpm above idle guaranteed never to stall
    float margin = 10, idle_absmin = 450.0, idle_absmax = 1000.0;  // High limit of idle speed adjustability
    float idle_rpm, stallpoint, dynamic_rpm, temphot, tempcold, idle_slope_rpmps;
    uint32_t settlerate_rpmps, stallrate_rpmps = 400;  // Engine rpm drops exceeding this much per second are considered a stall in progress
    int idlemode, targetstate, nextstate;
  protected:
    // String modenames[3] = { "direct", "cntrol", "minimz" };
    // String statenames[4] = { "drivng", "tohigh", "tolow", "tostal" };
    float* target_rpm; float* measraw_rpm; float* measfilt_rpm; float engine_temp_f;
    TemperatureSensor* engine_sensor = nullptr;   // Rate to lower idle from high point to low point (in rpm per second)
    bool we_just_changed_states = true, target_externally_set = false; // bool now_trying_to_idle = false;
    uint32_t index_now, index_last;  // Engine rpm drops exceeding this much per second are considered a stall in progress
    float targetlast_rpm, recovery_boost_rpm = 5;  // How much to increase rpm target in response to detection of stall slope
    // The following are for detecting arhythmic period in tach pulses, which isn't implemented yet
    uint32_t history_depth = 100;
    int32_t tach_history_rpm[100];  // Why can't I use [history_depth] here instead of [20] in this instantiation?  c++ is a pain in my ass
    uint32_t timestamps_us[100];
    Timer settleTimer, tachHistoryTimer, tachIdleTimer = Timer(5000000);  // tachIdleTimer = How often to update tach idle value based on engine temperature
    int64_t readtime_last;
  public:
    IdleControl() {}
void setup(float* target, float* measraw, float* measfilt,  // Variable references: idle target, rpm raw, rpm filt
        TemperatureSensor* engine_sensor_ptr,
        float tempcold, float temphot, int32_t settlerate = 100,  // Values for: engine operational temp cold (min) and temp hot (max) in degrees-f
        int myidlemode = CONTROL) {  // Configure idle control to just soft land or also attempt to minimize idle
        Serial.printf("Idle control..");
        target_rpm = target;
        measraw_rpm = measraw;
        measfilt_rpm = measfilt;
        *target_rpm = *measfilt_rpm;
        set_idlehigh(idlehigh);
        idlehot = constrain(idlehot, 0.0, idlehigh);
        stallpoint = idlehot - 1;  // Just to give a sane initial value
        set_idlecold(idlecold);
        set_temphot(temphot);
        set_tempcold(tempcold);        
        calc_idlespeed();
        targetlast_rpm = *target_rpm;
        settlerate_rpmps = settlerate;
        settleTimer.reset();
        idlemode = myidlemode;
        targetstate = DRIVING;
        if (engine_sensor_ptr == nullptr) Serial.printf(" err: no temp sensor");
        else engine_sensor = engine_sensor_ptr;
        tachIdleTimer.reset();
        Serial.printf("\n");
    }
    void update(void) {  // this should be called to update idle and throttle target values before throttle-related control loop outputs are calculated
        // update engine temp if it's ready
        if (engine_sensor != nullptr) engine_temp_f = engine_sensor->get_temperature();
        antistall();
        calc_idlespeed();  // determine our appropriate idle speed, based on latest engine temperature reading
        targetstate_changer();  // if targetstate was changed, prepare to run any initial actions upon processing our new targetstate algorithm
        if (targetstate == TODRIVE) process_todrive();  // Target is above idle, but currently engine is still idling 
        else if (targetstate == DRIVING) process_driving();  // while throttle is open when driving, we don't mess with the rpm target value
        else if (targetstate == DROPTOHIGH) process_droptohigh();  // once the metaphorical foot is taken off the gas, first we let the carb close quickly to a high-idle rpm level (that won't stall)
        else if (targetstate == DROPTOLOW) process_droptolow();  // after the rpm hits the high-idle level, we slowly close the throttle further until we reach the correct low-idle speed for the current engine temperature
        else if (targetstate == IDLING) process_idling();  // maintain the low-idle level, adjusting to track temperature cchanges as appropriate
        else if (targetstate == MINIMIZING) process_minimizing();  // if idlemode == MINIMIZE, we then further allow the idle to drop, until we begin to sense irregularity in our rpm sensor pulses
    }
    void goto_idle(void) {  // The gods request the engine should idle now
        if (targetstate == DRIVING) nextstate = (idlemode == DIRECT) ? DROPTOLOW : DROPTOHIGH;
        // now_trying_to_idle = true;
    }
    void push_tach_reading(int32_t reading, int64_t readtime) {  // Add a new rpm reading to a small LIFO ring buffer. We will use this to detect arhythmic rpm
        if (readtime == readtime_last) return;  // Ignore new tach values unless rpm has changed
        index_last = index_now;
        index_now = (index_now + 1) % history_depth;
        tach_history_rpm[index_now] = reading;
        timestamps_us[index_now] = (uint32_t)(readtime-readtime_last);  // (uint32_t)tachHistoryTimer.elapsed();
        // tachHistoryTimer.reset();
        readtime_last = readtime;
    }
    void set_target(float argtarget) {
        if ((int32_t)(*target_rpm) != (int32_t)argtarget) {
            target_externally_set = true;
            set_target_internal(argtarget);
        }
    }
    void set_engine_sensor(TemperatureSensor* sensor) { engine_sensor = sensor; }
  protected:
    void set_target_internal(float argtarget) {
        if ((int32_t)(*target_rpm) != (int32_t)argtarget) {
            targetlast_rpm = *target_rpm;
            *target_rpm = argtarget;
        }
    }
    void calc_idlespeed(void) {
        idle_rpm = map(engine_temp_f, tempcold, temphot, idlecold, idlehot);
        idle_rpm = constrain(idle_rpm, idlehot, idlecold);
    }
    void targetstate_changer(void) {  // If nextstate was changed during last update, or someone externally changed the target, change our targetstate
        if (target_externally_set) {  // If the target has been changed externally, then determine our appropriate targetstate based on target value
            if (*target_rpm > idle_rpm + margin) nextstate = (*measfilt_rpm > idlehigh) ? DRIVING : TODRIVE;
            // else nextstate = (idlemode == MINIMIZE) ? MINIMIZING : IDLING;
        }
        target_externally_set = false;
        we_just_changed_states = (nextstate != targetstate);
        targetstate = nextstate;
    }
    void process_todrive(void) {
        if (we_just_changed_states);  // { printf("todriv "); }
        else if (*measfilt_rpm > idlehigh) nextstate = DRIVING;
    }
    void process_driving(void) {}  // if (we_just_changed_states) { printf("driving "); }
    void process_droptohigh(void) {
        if (we_just_changed_states) { set_target_internal(idlehigh); }  // printf("droptohigh "); }
        else if (*measfilt_rpm <= idlehigh + margin) nextstate = DROPTOLOW;  // Done dropping to high idle, next continue dropping to low idle
    }
    void process_droptolow(void) {
        if (we_just_changed_states) { settleTimer.reset(); }  // printf("droptolow "); }
        if (*measfilt_rpm <= idle_rpm + margin) nextstate = (idlemode == MINIMIZE) ? MINIMIZING : IDLING;  // Done dropping to low idle, next proceed to a steady state
        else {  // Drop from current rpm toward low idle speed at configured rate
            set_target_internal(*measfilt_rpm - settlerate_rpmps * (float)settleTimer.elapsed()/1000000);  // Need to review the dynamics of this considering update frequency and motor latency 
            settleTimer.reset();
        }
    }
    void process_idling(void) {  // If we aren't set to attempt to minimize idle speed, then we end up here
        // if (we_just_changed_states) {       printf("idling "); }
        if (idlemode == MINIMIZE) nextstate = MINIMIZING;  // in case idlemode is changed while in idling state
        set_target_internal(idle_rpm);  // We're done dropping to the idle point, but keep tracking as idle speed may change
    }
    void process_minimizing(void) {
        if (we_just_changed_states) { stallpoint = idle_rpm; }  // printf("minimizing "); }
        else if (idlemode != MINIMIZE) nextstate = IDLING;  // in case idlemode is changed while in stallpoint state
        // else if (*measfilt_rpm > )
        // Soren finish writing this
    }
    void antistall(void) {  // Soren:  This is rather arbitrary and unlikely to work. Need to determine anti-stall strategy
        idle_slope_rpmps = (float)(tach_history_rpm[index_now] - tach_history_rpm[index_last]) * 1000000 / timestamps_us[index_now];
        if (*measfilt_rpm <= idlehigh && idle_slope_rpmps < stallrate_rpmps) set_target_internal(idle_rpm + recovery_boost_rpm);
    }
    // String get_modename(void) { return modenames[(int32_t)idlemode].c_str(); }
    // String get_statename(void) { return statenames[targetstate].c_str(); }
  public:
    void cycle_idlemode(int32_t cycledir) {  // Cycldir positive or negative
        if(cycledir) idlemode = constrain(idlemode + constrain(cycledir, -1, 1), 0, NUM_IDLEMODES - 1);
    }
    void set_target_ptr(float* __ptr) { target_rpm = __ptr; }
    void set_idlehot(float newidlehot) { idlehot = constrain(newidlehot, stallpoint, idlecold - 1); calc_idlespeed(); }
    void set_idlecold(float newidlecold) { idlecold = constrain(newidlecold, idlehot + 1, idlehigh - 1); calc_idlespeed(); }
    void set_temphot(float newtemphot) { if (newtemphot > tempcold) temphot = newtemphot; calc_idlespeed(); }
    void set_tempcold(float newtempcold) { if (newtempcold < temphot) tempcold = newtempcold; calc_idlespeed(); }
    void set_idlehigh(float newidlehigh) { idlehigh = constrain(newidlehigh, idlecold + 1, idle_absmax); }
    void set_settlerate(int32_t newrate) { settlerate_rpmps = newrate; }
    void add_idlehigh(float add) { set_idlehigh(idlehigh + add); }
    void add_idlehot(float add) { set_idlehot(idlehot + add); }
    void add_idlecold(float add) { set_idlecold(idlecold + add); }
    void add_temphot(float add) { set_temphot(temphot + add); }
    void add_tempcold(float add) { set_tempcold(tempcold + add); }
    void add_settlerate(int32_t add) { set_settlerate(settlerate_rpmps + add); }
    // Getter functions
    float target(void) { return *target_rpm; }
    float* target_ptr(void) { return target_rpm; }
    float* idle_rpm_ptr(void) { return &idle_rpm; }
};
// ServoMotor - a parent class providing common elements for motor manager child classes. These subclasses each
// coordinate all aspects of one motor, including related sensors & pid, having a standard control interface
class ServoMotor {
  protected:
    Hotrc* hotrc;
    Speedometer* speedo;
    Servo motor;
    static const uint32_t pid_timeout = 85000;
    Timer pid_timer = Timer(pid_timeout);
    int pin, freq;
  public:
    bool openloop = false, reverse = false;  // defaults. subclasses override as necessary
    float pc[NUM_MOTORVALS] = { 0, NAN, 100, NAN, NAN, NAN, NAN };  // percent values [OPMIN/PARKED/OPMAX/OUT/GOVERN/ABSMIN/ABSMAX]  values range from -100% to 100% are all derived or auto-assigned
    float si[NUM_MOTORVALS] = { 45.0, 43.0, 168.2, 45.0, NAN, 0, 180 };  // standard si-unit values [OPMIN/PARKED/OPMAX/OUT/GOVERN/ABSMIN/ABSMAX]
    float us[NUM_MOTORVALS] = { NAN, 1500, NAN, NAN, NAN, 500, 2500 };  // us pulsewidth values [-/CENT/-/OUT/-/ABSMIN/ABSMAX]
    ServoMotor(int _pin, int _freq) { pin = _pin; freq = _freq; }
    void setup(Hotrc* _hotrc, Speedometer* _speedo) {
        hotrc = _hotrc;
        speedo = _speedo;
        motor.setPeriodHertz(freq);
        motor.attach(pin, us[ABSMIN], us[ABSMAX]);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    }
    float pc_to_si(float _pc) {  // Eventually this should be linearized
        return map(_pc, pc[ABSMIN], pc[ABSMAX], si[ABSMIN], si[ABSMAX]); 
    }
    float si_to_pc(float _si) {  // Eventually this should be linearized
        return map(_si, si[ABSMIN], si[ABSMAX], pc[ABSMIN], pc[ABSMAX]);
    }
    float si_to_us(float _si) {  // works for motor with or without stop value
        return map(_si, si[ABSMIN], si[ABSMAX], reverse ? us[ABSMAX] : us[ABSMIN], reverse ? us[ABSMIN] : us[ABSMAX]);
    }
    float pc_to_us(float _pc) {  // works for motor with or without stop value
        return map(_pc, pc[ABSMIN], pc[ABSMAX], reverse ? us[ABSMAX] : us[ABSMIN], reverse ? us[ABSMIN] : us[ABSMAX]);
    }
    void write_motor() { motor.writeMicroseconds((int32_t)(us[OUT])); }
};
class JagMotor : public ServoMotor {
  protected:
    CarBattery* mulebatt;
    float car_batt_fake_v = 12.0;
    Timer volt_check_timer = Timer(3500000);
  public:
    using ServoMotor::ServoMotor;
    float duty_fwd_pc = 100;  // default. subclasses override as necessary
    float duty_rev_pc = 100;  // default. subclasses override as necessary
    float pc[NUM_MOTORVALS] = { NAN, 0, NAN, NAN, NAN, -100, 100 };  // percent values [OPMIN/STOP/OPMAX/OUT/-/ABSMIN/ABSMAX]  values range from -100% to 100% are all derived or auto-assigned
    float si[NUM_MOTORVALS] = { NAN, 0, NAN, NAN, NAN, NAN, NAN };  // standard si-unit values [OPMIN/STOP/OPMAX/OUT/-/ABSMIN/ABSMAX]
    float us[NUM_MOTORVALS] = { NAN, 1500, NAN, NAN, NAN, 670, 2330 };  // us pulsewidth values [-/CENT/-/OUT/-/ABSMIN/ABSMAX]
    float (&volt)[arraysize(si)] = si;  // our standard si value is volts. Create reference so si and volt are interchangeable
    // JagMotor(int _pin, int _freq) : ServoMotor(_pin, _freq) {}
    void derive() {  // calc pc and voltage op limits from volt and us abs limits 
        si[ABSMAX] = running_on_devboard ? car_batt_fake_v : mulebatt->v();
        si[ABSMIN] = -(si[ABSMAX]);
        pc[OPMIN] = pc[ABSMIN] * duty_rev_pc / 100.0;
        pc[OPMAX] = pc[ABSMAX] * duty_fwd_pc / 100.0;
        si[OPMIN] = map(pc[OPMIN], pc[STOP], pc[ABSMIN], si[STOP], si[ABSMIN]);
        si[OPMAX] = map(pc[OPMAX], pc[STOP], pc[ABSMAX], si[STOP], si[ABSMAX]);
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {
        ServoMotor::setup(_hotrc, _speedo);
        mulebatt = _batt;
        derive();
    }
    float pc_to_si(float _pc) {  // Eventually this should be linearized
        if (_pc > pc[STOP]) return map(_pc, pc[STOP], pc[ABSMAX], si[STOP], si[ABSMAX]);
        if (_pc < pc[STOP]) return map(_pc, pc[STOP], pc[ABSMIN], si[STOP], si[ABSMIN]);
        return si[STOP];
    }
    float si_to_pc(float _si) {  // Eventually this should be linearized
        if (_si > si[STOP]) return map(_si, si[STOP], si[ABSMAX], pc[STOP], pc[ABSMAX]);
        if (_si < si[STOP]) return map(_si, si[STOP], si[ABSMIN], pc[STOP], pc[ABSMIN]);
        return pc[STOP];
    }
    float si_to_us(float _si) {  // works for motor with center stop value
        if (_si > si[STOP]) return map(_si, si[STOP], si[ABSMAX], us[STOP], reverse ? us[ABSMIN] : us[ABSMAX]);
        if (_si < si[STOP]) return map(_si, si[STOP], si[ABSMIN], us[STOP], reverse ? us[ABSMAX] : us[ABSMIN]);
        return us[STOP];
    }
    float pc_to_us(float _pc) {  // works for motor with center stop value
        if (_pc > pc[STOP]) return map(_pc, pc[STOP], pc[ABSMAX], us[STOP], reverse ? us[ABSMIN] : us[ABSMAX]);
        if (_pc < pc[STOP]) return map(_pc, pc[STOP], pc[ABSMIN], us[STOP], reverse ? us[ABSMAX] : us[ABSMIN]);
        return us[STOP];
    }
    void write_motor() { motor.writeMicroseconds((int32_t)(us[OUT])); }
};
class GasServo : public ServoMotor {
  private:
    Tachometer* tach;
    Potentiometer* pot;
    TemperatureSensorManager* tempsens;
    float cruise_initial_kp = 5.57;   // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
    float cruise_initial_ki = 0.000;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
    float cruise_initial_kd = 0.000;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
    float initial_kp = 0.013;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
    float initial_ki = 0.000;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
    float initial_kd = 0.000;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
  public:
    using ServoMotor::ServoMotor;
    IdleControl idlectrl;
    QPID pid, cruisepid;
    bool openloop = true, reverse = false;  // if servo higher pulsewidth turns ccw, then do reverse=true
    float (&deg)[arraysize(si)] = si;  // our standard si value is degrees of rotation "deg". Create reference so si and deg are interchangeable
    float tach_last, cruise_target_pc, governor = 95;     // Software governor will only allow this percent of full-open throttle (percent 0-100)
    Timer servo_delay_timer = Timer(500000);    // We expect the servo to find any new position within this time
    void derive() {  // calc derived limit values for all units based on tuned values for each motor
        pc[ABSMIN] = map(si[ABSMIN], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[ABSMAX] = map(si[ABSMAX], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[PARKED] = map(si[PARKED], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[GOVERN] = map(governor, 0.0, 100.0, pc[OPMIN], pc[OPMAX]);  // pc[GOVERN] = pc[OPMIN] + governor * (pc[OPMAX] - pc[OPMIN]) / 100.0;      
        si[GOVERN] = map(pc[GOVERN], pc[OPMIN], pc[OPMAX], si[OPMIN], si[OPMAX]);
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, Tachometer* _tach, Potentiometer* _pot, TemperatureSensorManager* _temp) {
        printf("Gas servo..\n");
        ServoMotor::setup(_hotrc, _speedo);
        tach = _tach;
        pot = _pot;
        tempsens = _temp;
        derive();
        idlectrl.setup(pid.target_ptr(), tach->human_ptr(), tach->filt_ptr(), tempsens->get_sensor(loc::ENGINE), 
            temp_lims_f[ENGINE][OPMIN], temp_lims_f[ENGINE][WARNING], 50, IdleControl::idlemodes::CONTROL);
        pid.init(tach->filt_ptr(), &(pc[OPMIN]), &(pc[OPMAX]), initial_kp, initial_ki, initial_kd, QPID::pmod::onerr, 
            QPID::dmod::onerr, QPID::awmod::clamp, QPID::cdir::direct, pid_timeout);
        cruisepid.init(speedo->filt_ptr(), idlectrl.idle_rpm_ptr(), tach->govern_rpm_ptr(), cruise_initial_kp, cruise_initial_ki,
            cruise_initial_kd, QPID::pmod::onerr, QPID::dmod::onerr, QPID::awmod::round, QPID::cdir::direct, pid_timeout);
    }
    void update(int runmode) {
        float tach_now = tach->human();
        if (tach_now != tach_last) idlectrl.push_tach_reading(tach_now, tach->last_read_time());    
        tach_last = tach_now;
        if (pid_timer.expireset()) {
            // Step 1 : update throttle target from idle control or cruise mode pid, if applicable (on the same timer as gas pid)
            idlectrl.update();  // Allow idle control to mess with tach_target if necessary, or otherwise step in to prevent car from stalling
            if (runmode == CRUISE && (cruise_setpoint_mode == PID_SUSPEND_FLY) && !cruise_adjusting)
                pid.set_target(cruisepid.compute());  // cruise pid calculates new output (tach_target_rpm) based on input (speedmeter::human) and target (speedo_target_mph)
            // Step 2 : Determine servo pulse width value
            if (park_the_motors)
                pc[OUT] = pc[PARKED];
            else if (runmode == CAL && cal_gasmode)
                pc[OUT] = si_to_pc(map(pot->val(), pot->min(), pot->max(), deg[ABSMIN], deg[ABSMAX]));  // gas_ccw_max_us, gas_cw_min_us
            else if (runmode == CRUISE && (cruise_setpoint_mode != PID_SUSPEND_FLY))
                pc[OUT] = cruise_target_pc;
            else if (runmode != BASIC && runmode != CAL && runmode != ASLEEP && (runmode != SHUTDOWN || shutdown_incomplete)) {
                if (runmode == STALL || openloop) {  // Stall mode runs the gas servo directly proportional to joystick. This is truly open loop
                    if (hotrc->joydir() != JOY_UP) pc[OUT] = pc[OPMIN];  // If in deadband or being pushed down, we want idle
                    else pc[OUT] = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBTOP], hotrc->pc[VERT][OPMAX], pc[OPMIN], pc[GOVERN]);  // actuators still respond even w/ engine turned off
                }
                else pc[OUT] = pid.compute();  // Do proper pid math to determine gas_out_us from engine rpm error
            }
            // Step 3 : Convert to degrees and constrain if out of range
            deg[OUT] = pc_to_si(pc[OUT]);  // convert to degrees
            if (runmode == CAL && cal_gasmode)  // Constrain to operating limits. 
                deg[OUT] = constrain(deg[OUT], deg[ABSMIN], deg[ABSMAX]);
            else if (runmode == BASIC || runmode == SHUTDOWN || runmode == ASLEEP)
                deg[OUT] = constrain(deg[OUT], deg[PARKED], deg[GOVERN]);
            else deg[OUT] = constrain(deg[OUT], deg[OPMIN], deg[GOVERN]);
            pc[OUT] = si_to_pc(deg[OUT]);
            // Step 4 : Write to servo
            us[OUT] = si_to_us(deg[OUT]);
            write_motor();
        }
    }
};
// Brake uses two PID loops: one based on pressure (accurate for high brake pressures), and one based on position (accurate when pedal is more released)
// Note as pedal is pressed down, position decreases as pressure increases. PID output is percent motor power.
class BrakeMotor : public JagMotor {
  private:
    BrakePositionSensor* brkpos;
    PressureSensor* pressure;
    float brakemotor_max_duty_pc = 40.0;  // In order to not exceed spec and overheat the actuator, limit brake presses when under pressure and adding pressure
    float initial_kp = 0.142;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float initial_ki = 0.000;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float initial_kd = 0.000;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    float posn_initial_kp = 6.5;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float posn_initial_ki = 0.000;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float posn_initial_kd = 0.000;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    static constexpr uint32_t pid_timeout = 85000;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    static constexpr uint32_t interval_timeout = 1000000;  // How often to apply increment during auto-stopping (in us)
    static constexpr uint32_t stopcar_timeout = 8000000;  // How often to apply increment during auto-stopping (in us)
    float pres_out, posn_out, pc_out_last, posn_last, pres_last;
    static constexpr uint32_t duty_integration_period = 300000000;  // in us. note every 5 minutes uses 1% of our RAM for history buffer
    static constexpr int history_depth = duty_integration_period / pid_timeout;
    uint8_t history[history_depth];  // stores past motor work done. cast to uint8_t to save ram
    float duty_integral = sens_ratio[PRESPID] * hybrid_out_ratio;
    int duty_index = 0, duty_index_last; 
    // float posn_inflect, pres_inflect, pc_inflect;
    void activate_pid(int _pid) {
        dominantpid = _pid;
        pid_dom = &(pids[dominantpid]);
        posn_pid_active = (dominantpid == POSNPID);
    }
  public:
    using JagMotor::JagMotor;
    QPID pids[NUM_BRAKEPIDS];  // brake changes from pressure target to position target as pressures decrease, and vice versa
    QPID* pid_dom = &(pids[PRESPID]);  // AnalogSensor sensed[NUM_BRAKEPIDS];
    Timer stopcar_timer = Timer(stopcar_timeout);
    Timer interval_timer = Timer(interval_timeout);
    float brake_pid_trans_threshold_lo = 0.25;  // tunable. At what fraction of full brake pressure will motor control begin to transition from posn control to pressure control
    float brake_pid_trans_threshold_hi = 0.50;  // tunable. At what fraction of full brake pressure will motor control be fully transitioned to pressure control
    bool autostopping = false, reverse = false, openloop = false, duty_tracker_load_sensitive = true;
    // float duty_pc = 25;
    int dominantpid = brake_default_pid, dominantpid_last = brake_default_pid;
    bool posn_pid_active = (dominantpid == POSNPID);
    float panic_initial_pc = 60, hold_initial_pc = 40, panic_increment_pc = 4, hold_increment_pc = 2, pid_targ_pc, pid_err_pc, pid_final_out;
    float sens_ratio[NUM_BRAKEPIDS], sensnow[NUM_BRAKEPIDS], outnow[NUM_BRAKEPIDS], hybrid_math_offset, hybrid_math_coeff;  // , senslast[NUM_BRAKEPIDS], d_ratio[NUM_BRAKEPIDS], 
    float hybrid_sens_ratio, hybrid_sens_ratio_pc, hybrid_out_ratio = 1.0, hybrid_out_ratio_pc = 100.0;
    float duty_integral_sum, duty_instant, duty_continuous;
    void derive() {
        JagMotor::derive();
        hybrid_math_offset = 0.5 * (brake_pid_trans_threshold_lo + brake_pid_trans_threshold_hi);
        hybrid_math_coeff = M_PI / (brake_pid_trans_threshold_hi - brake_pid_trans_threshold_lo);
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt, PressureSensor* _pressure, BrakePositionSensor* _brkpos) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        printf("Brake motor..\n");
        JagMotor::setup(_hotrc, _speedo, _batt);
        pressure = _pressure;  // press_pin = _press_pin;  // sensed[PRESPID] = _pressure;
        brkpos = _brkpos;  // posn_pin = _posn_pin;  // sensed[POSNPID] = _brkpos;
        duty_fwd_pc = brakemotor_max_duty_pc;  
        pres_last = pressure->human();
        posn_last = brkpos->human();
        activate_pid(brake_default_pid);
        derive();
        if (brake_hybrid_pid) calc_hybrid_ratio();
        pids[PRESPID].init(pressure->filt_ptr(), &(pc[OPMIN]), &(pc[OPMAX]), initial_kp, initial_ki, initial_kd, QPID::pmod::onerr,
            QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout, QPID::ctrl::manual, QPID::centmod::on, pc[STOP]);
        pids[POSNPID].init(brkpos->filt_ptr(), &(pc[OPMIN]), &(pc[OPMAX]), posn_initial_kp, posn_initial_ki, posn_initial_kd, QPID::pmod::onerr, 
            QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::reverse, pid_timeout, QPID::ctrl::manual, QPID::centmod::on, pc[STOP]);
    }
    void set_pidtarg(float targ_pc) {
        pid_targ_pc = targ_pc;
        pids[PRESPID].set_target(pressure->min_human() + pid_targ_pc * (pressure->max_human() - pressure->min_human()) / 100.0);
        pids[POSNPID].set_target(brkpos->min_human() + (100.0 - pid_targ_pc) * (brkpos->max_human() - brkpos->min_human()) / 100.0);
    }
  private:
    // The influence on the final output from the pressure and position pids is weighted based on the pressure reading
    // Which PID is dominant is the pid having more than 50% influence
    // PID outputs are weighted based on where in the pedal press we are. Near the bottom is pressure control, and near the top is position control.
    void calc_hybrid_ratio() {
        if (sens_ratio[PRESPID] >= brake_pid_trans_threshold_hi) hybrid_out_ratio = 1.0;  // at pressure above hi threshold, pressure has 100% influence
        else if (sens_ratio[PRESPID] <= brake_pid_trans_threshold_lo) hybrid_out_ratio = 0.0;  // at pressure below lo threshold, position has 100% influence
        else hybrid_out_ratio = 0.5 + 0.5 * sin(hybrid_math_coeff * (sens_ratio[PRESPID] - hybrid_math_offset));  // in between we make a steep but smooth transition during which both have some influence
        hybrid_out_ratio_pc = 100.0 * hybrid_out_ratio;  // for display
        activate_pid((int)(sens_ratio[PRESPID] > 0.5 * (brake_pid_trans_threshold_lo + brake_pid_trans_threshold_hi)));  // pressure pid == 1. sets the "dominant" pid
    }
    float pid_out() {  // feedback brake pid with position or pressure, whichever is changing more quickly
        float range;
        for (int p = POSNPID; p <= PRESPID; p++) {
            range = (p == PRESPID) ? (pressure->max_human() - pressure->min_human()) : (brkpos->max_human() - brkpos->min_human());
            sensnow[p] = (p == PRESPID) ? pressure->human() : brkpos->human();  // latest sensed value
            sens_ratio[p] = (sensnow[p] - ((p == PRESPID) ? pressure->min_human() : brkpos->min_human())) / range;  // calculate ratio of output to range
            outnow[p] = pids[p].compute();  // get each pid calculated output
        }
        if (brake_hybrid_pid) {
            calc_hybrid_ratio();  // calculate pressure vs. position multiplier based on the sensed values
            pid_final_out = hybrid_out_ratio * outnow[PRESPID] + (1.0 - hybrid_out_ratio) * outnow[POSNPID];  // combine pid outputs weighted by the multiplier
            for (int p = POSNPID; p <= PRESPID; p++) pids[p].set_output(pid_final_out);  // Feed the final value back into the pids
        }
        else pid_final_out = outnow[dominantpid];
        return pid_final_out;
    }
    void fault_filter() {
        // 1. Detect  brake chain is not connected (evidenced by change in brake position without expected pressure changes)
        // 2. Detect obstruction, motor failure, or inaccurate position. Evidenced by motor instructed to move but position not changing even when pressure is low.
        // 3. Detet brake hydraulics failure or inaccurate pressure. Evidenced by normal positional change not causing expected increase in pressure.
        // retract_effective_max_us = volt[STOP] + duty_pc * (volt[OPMAX] - volt[STOP]);  // Stores instantaneous calculated value of the effective maximum pulsewidth after attenuation
    }
    // Duty tracking: we keep a buffer of motor work done
    void calc_motor_duty() {
        ++duty_index %= history_depth;  // advance ring buffer index, this is the oldest value in the buffer
        duty_integral_sum -= (float)history[duty_index] / 2.5;  // subtract oldest value from our running sum.
        int duty_instant = std::abs(pc[OUT]);  // what is current motor drive percent
        if (duty_tracker_load_sensitive) {  // if we are being fancy
            if (pc[OUT] < 0) duty_instant = 0.0;  // ignore motion in the extend direction due to low load
            else duty_instant *= sens_ratio[PRESPID];  // scale load with brake pressure
        }        
        history[duty_index] = (uint8_t)(duty_instant * 2.5);  // write over current indexed value in the history ring buffer. scale to optimize for uint8
        duty_integral_sum += duty_instant;  // add value to our running sum
        duty_continuous = duty_integral_sum / history_depth;  // divide ring buffer total by its size to get average value
        if (duty_continuous < 0.001) duty_continuous = 0.0;  // otherwise displays "5.67e-" (?)
        // now we know our continuous duty, need to limit motor output based on this
    }
  public:
    void autostop_initial(bool panic) { set_pidtarg(smax(panic ? panic_initial_pc : hold_initial_pc, pid_dom->target())); }
    void autostop_increment(bool panic) { set_pidtarg(smin(pid_dom->target() + panic ? panic_increment_pc : hold_increment_pc, 100.0)); }
    void update(int runmode) {
        // Brakes - Determine motor output and write it to motor
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            calc_motor_duty();
            // Step 1 : Determine motor percent value
            if (park_the_motors) {
                if (brkpos->filt() + brkpos->margin() <= brkpos->parkpos())  // If brake is retracted from park point, extend toward park point, slowing as we approach
                    pc[OUT] = map(brkpos->filt(), brkpos->parkpos(), brkpos->min_in(), pc[STOP], pc[OPMIN]);
                else if (brkpos->filt() - brkpos->margin() >= brkpos->parkpos())  // If brake is extended from park point, retract toward park point, slowing as we approach
                    pc[OUT] = map(brkpos->filt(), brkpos->parkpos(), brkpos->max_in(), pc[STOP], pc[OPMAX]);
            }
            else if (runmode == CAL && cal_brakemode) {
                int _joydir = hotrc->joydir();
                if (_joydir == JOY_UP) pc[OUT] = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBTOP], hotrc->pc[VERT][OPMAX], pc[STOP], pc[OPMAX]);
                else if (_joydir == JOY_DN) pc[OUT] = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][OPMIN], hotrc->pc[VERT][DBBOT], pc[OPMIN], pc[STOP]);
                else pc[OUT] = pc[STOP];
            }
            else if (runmode == CAL || runmode == BASIC || runmode == ASLEEP || (runmode == SHUTDOWN && !shutdown_incomplete))
                pc[OUT] = pc[STOP];
            else pc[OUT] = pid_out(); // Otherwise the pid control is active  // First attenuate max power to avoid blowing out the motor like in bm2023, if retracting, as a proportion of position from zeropoint to fully retracted
            // Step 2 : Fix motor pc value if it's out of range or exceeding positional limits
            if (runmode == CAL && cal_brakemode)  // Constrain the motor to the operational range, unless calibrating (then constraint already performed above)
                pc[OUT] = constrain(pc[OUT], pc[ABSMIN], pc[ABSMAX]);  // Constrain to full potential range when calibrating. Caution don't break anything!
            else if ((pc[OUT] < pc[STOP] && brkpos->filt() > brkpos->parkpos() - brkpos->margin()) 
                  || (pc[OUT] > pc[STOP] && brkpos->filt() < brkpos->min_in() + brkpos->margin()))  // If brake is at position limits and we're tring to go further, stop the motor
                pc[OUT] = pc[STOP];
            else pc[OUT] = constrain(pc[OUT], pc[OPMIN], pc[OPMAX]);  // Send to the actuator. Refuse to exceed range
            // Step 3 : Convert motor percent value to pulse width for motor, and to volts for display
            us[OUT] = pc_to_us(pc[OUT]);
            volt[OUT] = pc_to_si(pc[OUT]);
            // Step 4 : Write to motor
            write_motor();
        }
    }
};
class SteerMotor : public JagMotor {
  public:
    using JagMotor::JagMotor;
    float steer_safe_pc = 72.0;  // this percent otaken off full steering power when driving full speed (linearly applied)
    bool reverse = false, openloop = true;
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        printf("Steering motor..\n");
        JagMotor::setup(_hotrc, _speedo, _batt);
    }
    void update(int runmode) {
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            if (runmode == ASLEEP || (runmode == SHUTDOWN && !shutdown_incomplete)) pc[OUT] = pc[STOP];  // Stop the steering motor if in shutdown mode and shutdown is complete
            else {
                int _joydir = hotrc->joydir(HORZ);
                if (_joydir == JOY_RT) pc[OUT] = map(hotrc->pc[HORZ][FILT], hotrc->pc[HORZ][DBTOP], hotrc->pc[HORZ][OPMAX], pc[STOP], steer_safe(pc[OPMAX]));  // if joy to the right of deadband
                else if (_joydir == JOY_LT) pc[OUT] = map(hotrc->pc[HORZ][FILT], hotrc->pc[HORZ][DBBOT], hotrc->pc[HORZ][OPMIN], pc[STOP], steer_safe(pc[OPMIN]));  // if joy to the left of deadband
                else pc[OUT] = pc[STOP];  // Stop the steering motor if inside the deadband
            }
            pc[OUT] = constrain(pc[OUT], pc[OPMIN], pc[OPMAX]);  // Don't be out of range
            us[OUT] = pc_to_us(pc[OUT]);
            volt[OUT] = pc_to_si(pc[OUT]);
            write_motor();
        }
    }
  private:
    float steer_safe(float endpoint) {
        return pc[STOP] + (endpoint - pc[STOP]) * (1.0 - steer_safe_pc * speedo->filt() / (100.0 * speedo->redline_mph()));
    }
};