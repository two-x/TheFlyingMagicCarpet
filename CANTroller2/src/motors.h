// motors.h : centralized wrapper classes for all braking or throttle activity, as holistic coordination between sensors, actuator, and intent is critical
#pragma once
#include <ESP32Servo.h>  // Eventually move Servos into new ServoPWM objects then remove this
// #include "qpid.h"
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
// I stole this library and modified it heavily to our purpposes - Soren
// QPID Library for Arduino - Version 3.1.9 by dlloydev https://github.com/Dlloydev/QPID
// Based on the Arduino PID_v1 Library. Licensed under the MIT License.
class QPID {
  public:
    enum class ctrl : int {manual, automatic, toggle};            // controller mode
    enum class cdir : int {reverse=-1, direct=1};                      // controller direction
    enum class pmod : int {onerr, onmeas, onerrmeas};             // proportional mode
    enum class dmod : int {onerr, onmeas};                        // derivative mode
    enum class awmod : int {cond, clamp, off, round, roundcond};  // integral anti-windup mode  // Soren edit
    enum class centmod : int {off, on, strict};                   // Soren - Allows a defined output zero point
  private:
    float dispkp = 0; float dispki = 0; float dispkd = 0;
    float _pterm, _iterm, _dterm, _kp, _ki, _kd, _err, lasterr, lastin, _cent, _outsum, _target, _output;
    float *myin;     // Pointers to the input, output, and target variables. This  creates a
    float *_outmin;
    float *_outmax;
    ctrl _mode = ctrl::manual;
    cdir _dir = cdir::direct;
    pmod _pmode = pmod::onerr;
    dmod _dmode = dmod::onmeas;
    awmod _awmode = awmod::cond;
    centmod _centmode = centmod::off;  // Soren
    uint32_t sampletime, lasttime;
  public:
    QPID() {}  // Default constructor
    QPID(float* a_in, float* a_min, float* a_max, float a_kp = 0, float a_ki = 0, float a_kd = 0,  // Soren edit
      pmod a_pmode = pmod::onerr, dmod a_dmode = dmod::onmeas, awmod a_awmode = awmod::cond, cdir a_dir = cdir::direct,
      uint32_t a_sampletime = 100000, ctrl a_mode = ctrl::manual, centmod a_centmode = centmod::off, float a_cent = NAN) {
        init(a_in, a_min, a_max, a_kp, a_ki, a_kd, a_pmode, a_dmode, a_awmode, a_dir, a_sampletime, a_mode, a_centmode, a_cent);
    }
    void init(float* a_in, float* a_min, float* a_max, float a_kp = 0, float a_ki = 0, float a_kd = 0,  // Soren edit
      pmod a_pmode = pmod::onerr, dmod a_dmode = dmod::onmeas, awmod a_awmode = awmod::cond, cdir a_dir = cdir::direct,
      uint32_t a_sampletime = 100000, ctrl a_mode = ctrl::manual, centmod a_centmode = centmod::off, float a_cent = NAN) {
        myin = a_in;
        _mode = a_mode;
        _outmin = a_min;
        _outmax = a_max;
        _output = constrain(_output, *_outmin, *_outmax);
        _outsum = constrain(_outsum, *_outmin, *_outmax);
        set_centmode(a_centmode);  // Soren
        if (_centmode != centmod::off && !std::isnan(a_cent)) {  // Soren
            set_cent(a_cent);  // Soren
            _outsum = _cent;
        }
        else set_cent(*_outmin);  // Soren
        sampletime = a_sampletime;              // Soren edit
        set_dir(a_dir);
        set_tunings(a_kp, a_ki, a_kd, _pmode, _dmode, _awmode);
        lasttime = micros() - sampletime;  // Soren edit

    }
    void init(float preload_output = NAN) {  // Ensure a bumpless transfer from manual to automatic mode
        if (!std::isnan(preload_output)) _output = preload_output;
        _outsum = _output;  // Soren
        lastin = *myin;
        _outsum = constrain(_outsum, *_outmin, *_outmax);
    }
    // This function should be called every time "void loop()" executes. The function will decide whether a new 
    // PID output needs to be computed. Returns true when the output is computed, false when nothing has been done.
    float compute() {
        uint32_t now = micros();  // Soren edit
        uint32_t timechange = (now - lasttime);
        if (_mode == ctrl::automatic && timechange < sampletime) return _output;  // If class is handling the timing and this time was a nop

        float in = *myin;
        float din = in - lastin;
        if (_dir == cdir::reverse) din = -din;  // Soren

        _err = _target - in;
        if (_dir == cdir::reverse) _err = -_err;  // Soren
        float derr = _err - lasterr;

        float peterm = _kp * _err;
        float pmterm = _kp * din;
        if (_pmode == pmod::onerr) pmterm = 0;
        else if (_pmode == pmod::onmeas) peterm = 0;
        else { //onerrmeas
            peterm *= 0.5f;
            pmterm *= 0.5f;
        }
        _pterm = peterm - pmterm;
        _iterm = _ki * _err;
        if (_dmode == dmod::onerr) _dterm = _kd * derr;
        else _dterm = -_kd * din; // onmeas

        if (_awmode == awmod::cond || _awmode == awmod::roundcond) {  // condition anti-windup (default)
            bool aw = false;
            float _itermout = (peterm - pmterm) + _ki * (_iterm + _err);
            if (_itermout > *_outmax && derr > 0) aw = true;
            else if (_itermout < *_outmin && derr < 0) aw = true;
            if (aw && _ki) _iterm = constrain(_itermout, -(*_outmax), *_outmax);
        }
        else if ((_awmode == awmod::round || _awmode == awmod::roundcond) && _err < 0.001 && _err > -0.001) {
            _err = 0.0;
            if (_centmode == centmod::on || _centmode == centmod::strict) _outsum = _cent;     
        }
        if (_centmode == centmod::strict && _err * lasterr < 0) _outsum = _cent;  // Soren - Recenters any old integral when error crosses zero

        _outsum += _iterm - pmterm;  // by default, compute output as per PID_v1    // include integral amount and pmterm
        if (_awmode != awmod::off) _outsum = constrain(_outsum, *_outmin, *_outmax);  // Clamp

        _output = constrain(_outsum + peterm + _dterm, *_outmin, *_outmax);  // include _dterm, clamp and drive output

        lasterr = _err;  // Soren
        lastin = in;
        lasttime = now;
        return _output;
    }
    // set_tunings  This function allows the controller's dynamic performance to be adjusted. It's called 
    // automatically from the constructor, but tunings can also be adjusted on the fly during normal operation.
    void set_tunings(float a_kp, float a_ki, float a_kd, pmod a_pmode = pmod::onerr,
      dmod a_dmode = dmod::onmeas, awmod a_awmode = awmod::cond) {
        if (a_kp < 0 || a_ki < 0 || a_kd < 0 || !sampletime) return;  // Soren - added divide by zero protection
        if (a_ki == 0) _outsum = 0;
        _pmode = a_pmode; _dmode = a_dmode; _awmode = a_awmode;
        dispkp = a_kp; dispki = a_ki; dispkd = a_kd;
        float sampletime_sec = (float)sampletime / 1000000;
        _kp = a_kp;
        _ki = a_ki * sampletime_sec;
        _kd = a_kd / sampletime_sec;
    }
    // set_tunings  Set Tunings using the last remembered pmode, dmode and awmode settings.
    void set_tunings(float a_kp, float a_ki, float a_kd) {
        set_tunings(a_kp, a_ki, a_kd, _pmode, _dmode, _awmode);
    }
    // set_sampletime  Sets the period, in microseconds, at which the calculation is performed.
    void set_sampletime(uint32_t a_sampletime) {
        if (a_sampletime > 0 && sampletime) {  // Soren - added more divide by zero protection
            float ratio  = (float)a_sampletime / (float)sampletime;
            _ki *= ratio;
            _kd /= ratio;
            sampletime = a_sampletime;
        }
    }
    // set_mode Sets the controller mode to manual (0), automatic (1) or timer (2) when the transition 
    // from manual to automatic or timer occurs, the controller is automatically initialized.
    void set_mode(ctrl a_mode) {
        if (_mode == ctrl::manual && a_mode != ctrl::manual) { // just went from manual to automatic
            _mode = ctrl::automatic;
            init();
        }
        else if (_mode == ctrl::automatic && a_mode != ctrl::automatic) {
            _mode = ctrl::manual;
        }
    }
    void set_mode(int a_mode) { set_mode((ctrl)a_mode); }
    // Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.
    void reset() {
        lasttime = micros() - sampletime;  // Soren edit
        lastin = 0; _outsum = 0;
        _pterm = 0; _iterm = 0; _dterm = 0;
    }
    void set_kp(float a_kp) { set_tunings(a_kp, dispki, dispkd, _pmode, _dmode, _awmode); }  // Soren
    void set_ki(float a_ki) { set_tunings(dispkp, a_ki, dispkd, _pmode, _dmode, _awmode); }  // Soren
    void set_kd(float a_kd) { set_tunings(dispkp, dispki, a_kd, _pmode, _dmode, _awmode); }  // Soren
    void add_kp(float add) { set_kp(_kp + add); }  // Soren
    void add_ki(float add) { set_ki(_ki + add); }  // Soren
    void add_kd(float add) { set_kd(_kd + add); }  // Soren
    void set_centmode(centmod a_centmode) { _centmode = a_centmode; }  // Soren
    void set_centmode(int a_centmode) { _centmode = (centmod)a_centmode; }  // Soren
    void set_cent(float a_cent) { if (*_outmin <= a_cent && *_outmax >= a_cent) _cent = a_cent; }  // Soren
    void set_target(float a_target) { _target = a_target; }
    void set_output(float a_output) { _output = constrain(a_output, *_outmin, *_outmax); }
    // The PID will either be connected to a direct acting process (+output leads to +input) or a reverse acting process(+output leads to -input).
    void set_dir(cdir a_dir) { _dir = a_dir; }
    void set_dir(int a_dir) { _dir = (cdir)a_dir; }
    // Sets the computation method for the proportional term, to compute based either on error (default), on measurement, or the average of both.
    void set_pmode(pmod a_pmode) { _pmode = a_pmode; }
    void set_pmode(int a_pmode) { _pmode = (pmod)a_pmode; }
    // Sets the computation method for the derivative term, to compute based either on error or on measurement (default).
    void set_dmode(dmod a_dmode) { _dmode = a_dmode; }
    void set_dmode(int a_dmode) { _dmode = (dmod)a_dmode; }
    // Sets the integral anti-windup mode to one of clamp, which clamps the output after adding integral and proportional (on measurement) terms,
    // or cond (default), which provides some integral correction, prevents deep saturation and reduces overshoot. Option off disables anti-windup altogether.
    void set_awmode(awmod a_awmode) { _awmode = a_awmode; }
    void set_awmode(int a_awmode) { _awmode = (awmod)a_awmode; }
    // sets the output summation value
    void set_outsum(float a_outsum) { _outsum = a_outsum; }
    // Getter functions
    float err() { return _err; }  // Soren
    float kp() { return dispkp; }
    float ki() { return dispki; }
    float kd() { return dispkd; }
    float pterm() { return _pterm; }
    float iterm() { return _iterm; }
    float dterm() { return _dterm; }
    float outsum() { return _outsum; }
    float outmin() { return *_outmin; }  // Soren
    float outmax() { return *_outmax; }  // Soren
    float outrange() { return *_outmax - *_outmin; }  // Soren
    float cent() { return _cent; }  // Soren
    float target() { return _target; }  // Soren
    float output() { return _output; }  // Soren
    int mode() { return static_cast<int>(_mode); }
    int dir() { return static_cast<int>(_dir); }
    int pmode() { return static_cast<int>(_pmode); }
    int dmode() { return static_cast<int>(_dmode); }
    int awmode() { return static_cast<int>(_awmode); }
    int centmode() { return static_cast<int>(_centmode); }  // Soren
    float* target_ptr() { return &_target; }
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
    bool reverse = false;  // defaults. subclasses override as necessary
    float pc[NUM_MOTORVALS] = { 0, NAN, 100, NAN, NAN, NAN, NAN, NAN };  // percent values [OPMIN/PARKED/OPMAX/OUT/GOVERN/ABSMIN/ABSMAX/MARGIN]  values range from -100% to 100% are all derived or auto-assigned
    float si[NUM_MOTORVALS] = { 45.0, 43.0, 168.2, 45.0, NAN, 0, 180, 1.0 };  // standard si-unit values [OPMIN/PARKED/OPMAX/OUT/GOVERN/ABSMIN/ABSMAX/MARGIN]
    float us[NUM_MOTORVALS] = { NAN, 1500, NAN, NAN, NAN, 500, 2500, NAN };  // us pulsewidth values [-/CENT/-/OUT/-/ABSMIN/ABSMAX/-]
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
    float pc[NUM_MOTORVALS] = { NAN, 0, NAN, NAN, NAN, -100, 100, 2 };  // percent values [OPMIN/STOP/OPMAX/OUT/-/ABSMIN/ABSMAX/MARGIN]  values range from -100% to 100% are all derived or auto-assigned
    float si[NUM_MOTORVALS] = { NAN, 0, NAN, NAN, NAN, NAN, NAN, NAN };  // standard si-unit values [OPMIN/STOP/OPMAX/OUT/-/ABSMIN/ABSMAX/MARGIN]
    float us[NUM_MOTORVALS] = { NAN, 1500, NAN, NAN, NAN, 670, 2330, NAN };  // us pulsewidth values [-/CENT/-/OUT/-/ABSMIN/ABSMAX/-]
    float (&volt)[arraysize(si)] = si;  // our standard si value is volts. Create reference so si and volt are interchangeable
    // JagMotor(int _pin, int _freq) : ServoMotor(_pin, _freq) {}
    void derive() {  // calc pc and voltage op limits from volt and us abs limits 
        si[ABSMAX] = running_on_devboard ? car_batt_fake_v : mulebatt->v();
        si[ABSMIN] = -(si[ABSMAX]);
        pc[OPMIN] = pc[ABSMIN] * duty_rev_pc / 100.0;
        pc[OPMAX] = pc[ABSMAX] * duty_fwd_pc / 100.0;
        si[OPMIN] = map(pc[OPMIN], pc[STOP], pc[ABSMIN], si[STOP], si[ABSMIN]);
        si[OPMAX] = map(pc[OPMAX], pc[STOP], pc[ABSMAX], si[STOP], si[ABSMAX]);
        si[MARGIN] = map(pc[MARGIN], pc[ABSMIN], pc[ABSMAX], si[ABSMIN], si[ABSMAX]);
        // us[MARGIN] = map(pc[MARGIN], pc[ABSMIN], pc[ABSMAX], us[ABSMIN], us[ABSMAX]);
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
    float cruise_pidgas_initial_kp = 5.57;   // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
    float cruise_pidgas_initial_ki = 0.000;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
    float cruise_pidgas_initial_kd = 0.000;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
    float cruise_opengas_initial_kp = 7.00;   // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
    float cruise_opengas_initial_ki = 0.000;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
    float cruise_opengas_initial_kd = 0.000;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
    float gas_initial_kp = 0.013;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
    float gas_initial_ki = 0.000;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
    float gas_initial_kd = 0.000;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
    float cruise_ctrl_extent_pc, adjustpoint;       // During cruise adjustments, saves farthest trigger position read
    bool cruise_trigger_released = false;
    Timer cruiseDeltaTimer;
  public:
    using ServoMotor::ServoMotor;
    IdleControl idlectrl;
    QPID pid, cruisepid;
    int motormode = Idle, oldmode = Idle;
    int mode_request = NA;
    bool mode_busy = false;
    bool reverse = false;  // if servo higher pulsewidth turns ccw, then do reverse=true
    float (&deg)[arraysize(si)] = si;  // our standard si value is degrees of rotation "deg". Create reference so si and deg are interchangeable
    float tach_last, cruise_target_pc, governor = 95;     // Software governor will only allow this percent of full-open throttle (percent 0-100)
    void derive() {  // calc derived limit values for all units based on tuned values for each motor
        pc[ABSMIN] = map(si[ABSMIN], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[ABSMAX] = map(si[ABSMAX], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[PARKED] = map(si[PARKED], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[GOVERN] = map(governor, 0.0, 100.0, pc[OPMIN], pc[OPMAX]);  // pc[GOVERN] = pc[OPMIN] + governor * (pc[OPMAX] - pc[OPMIN]) / 100.0;      
        si[GOVERN] = map(pc[GOVERN], pc[OPMIN], pc[OPMAX], si[OPMIN], si[OPMAX]);
        pc[MARGIN] = map(si[MARGIN], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
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
        pid.init(tach->filt_ptr(), &(pc[OPMIN]), &(pc[OPMAX]), gas_initial_kp, gas_initial_ki, gas_initial_kd, QPID::pmod::onerr, 
            QPID::dmod::onerr, QPID::awmod::clamp, QPID::cdir::direct, pid_timeout);
        if (throttle_ctrl_mode == OpenLoop) {
            cruisepid.init(speedo->filt_ptr(), &(pc[OPMIN]), &(pc[OPMAX]), cruise_opengas_initial_kp, cruise_opengas_initial_ki,
                cruise_opengas_initial_kd, QPID::pmod::onerr, QPID::dmod::onerr, QPID::awmod::round, QPID::cdir::direct, pid_timeout);
        }
        else if (throttle_ctrl_mode == ActivePID) {
            cruisepid.init(speedo->filt_ptr(), idlectrl.idle_rpm_ptr(), tach->govern_rpm_ptr(), cruise_pidgas_initial_kp, cruise_pidgas_initial_ki,
                cruise_pidgas_initial_kd, QPID::pmod::onerr, QPID::dmod::onerr, QPID::awmod::round, QPID::cdir::direct, pid_timeout);
        }
    }
  private:
    // throttle_ctrl_mode : (compile time option)
    //    OpenLoop : Servo angle is simply proportional to trigger pull. This is our tested default
    //    ActivePID : Servo angle determined by PID calculation designed to converge engine rpm to an rpm target value which is based on trigger
    // cruise_setpoint_scheme : (compile time option) Pick from 3 different styles for adjustment of cruise setpoint. I prefer THROTTLE_DELTA.
    //    THROTTLE_ANGLE : Cruise locks servo angle (cruise_target_pc), instead of pid. Moving trigger from center adjusts setpoint proportional to how far you push it before releasing (and reduced by an attenuation factor)
    //    THROTTLE_DELTA : Cruise locks servo angle (cruise_target_pc), instead of pid. Any non-center trigger position continuously adjusts setpoint proportional to how far it's pulled over time (up to a specified maximum rate)
    //    PID_SUSPEND_FLY : Cruise uses its own pid targeting a speed value. The PID output is either a servo angle or an rpm target for the gas pid, depending on throttle_ctrl_mode setting above. Whatever speed you're at when trigger releases is new cruise target  
    void calc_cruise_target() {
        int joydir = hotrc->joydir(VERT);
        if (joydir == JOY_CENT) {
            cruise_adjusting = false;
            cruise_trigger_released = true;
            cruise_ctrl_extent_pc = hotrc->pc[VERT][CENT];  // After an adjustment, need this to prevent setpoint from following the trigger back to center as you release it
            if (cruise_setpoint_scheme == PID_SUSPEND_FLY) {
                cruise_target_pc = cruisepid.compute();
                if (throttle_ctrl_mode == ActivePID) pid.set_target(cruise_target_pc);  // this is a bit misleading use of variables, because this argument value is actually an rpm target, not cruise percent, if gas is on pid
            }
        }
        else if ((joydir == JOY_UP || (joydir == JOY_DN && cruise_speed_lowerable)) && cruise_trigger_released) {  // adjustments disabled until trigger has been to center at least once since going to cruise mode
            float ctrlratio = (std::abs(hotrc->pc[VERT][FILT]) - hotrc->pc[VERT][DBTOP]) / (hotrc->pc[VERT][OPMAX] - hotrc->pc[VERT][DBTOP]);
            if (cruise_setpoint_scheme == THROTTLE_DELTA) {
                if (cruise_adjusting) cruise_target_pc += joydir * ctrlratio * cruise_delta_max_pc_per_s * cruiseDeltaTimer.elapsed() / 1000000.0;
                cruiseDeltaTimer.reset(); 
            }
            else if (cruise_setpoint_scheme == THROTTLE_ANGLE && std::abs(hotrc->pc[VERT][FILT]) >= cruise_ctrl_extent_pc) {  // to avoid the adjustments following the trigger back to center when released
                if (!cruise_adjusting) adjustpoint = cruise_target_pc;  // When beginning adjustment, save current throttle pulse value to use as adjustment endpoint
                cruise_target_pc = adjustpoint + ctrlratio * cruise_angle_attenuator * (((joydir == JOY_UP) ? 100.0 : 0.0) - adjustpoint);
                cruise_ctrl_extent_pc = std::abs(hotrc->pc[VERT][FILT]);
            }
            else if (cruise_setpoint_scheme == PID_SUSPEND_FLY) {
                if (throttle_ctrl_mode == OpenLoop) {
                    if (!cruise_adjusting) adjustpoint = pc[OUT];
                    cruise_target_pc = adjustpoint + ctrlratio * (((joydir == JOY_UP) ? pc[OPMAX] : pc[OPMIN]) - adjustpoint);;
                }
                else if (throttle_ctrl_mode == ActivePID) {
                    if (!cruise_adjusting) adjustpoint = tach->filt();
                    pid.set_target(adjustpoint + ctrlratio * (((joydir == JOY_UP) ? tach->govern_rpm() : idlectrl.idle_rpm) - adjustpoint));
                }
                cruisepid.set_target(speedo->filt());
            }
            cruise_target_pc = constrain(cruise_target_pc, 0.0, 100.0);
            cruise_adjusting = true;
        }
    }
    void set_output() { // services any requests for change in brake mode
        cal_gasmode = false;
        if (motormode == Idle) {
            idlectrl.goto_idle();
        }
        else if (motormode == Calibrate) {
            cal_gasmode = true;
            pc[OUT] = si_to_pc(map(pot->val(), pot->min(), pot->max(), deg[ABSMIN], deg[ABSMAX]));  // gas_ccw_max_us, gas_cw_min_us
        }
        else if (motormode == Cruise) {
            calc_cruise_target();  // cruise mode just got too big to be nested in this if-else clause
            if (throttle_ctrl_mode == ActivePID) pc[OUT] = pid.compute();
            else if (throttle_ctrl_mode == OpenLoop) pc[OUT] = cruise_target_pc;
        }
        else if (motormode == ParkMotor) {
            mode_busy = true;
            pc[OUT] = pc[PARKED];
        }
        else if (motormode == ActivePID) {
            if (hotrc->joydir(VERT) == JOY_UP)  // If we are trying to accelerate, scale joystick value to determine gas setpoint
                pid.set_target(map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBTOP], hotrc->pc[VERT][OPMAX], idlectrl.idle_rpm, tach->govern_rpm()));
            else idlectrl.goto_idle();  // Else let off gas (if gas using PID mode)
            pc[OUT] = pid.compute();  // Do proper pid math to determine gas_out_us from engine rpm error
        }
        else if (motormode == OpenLoop) {
            if (hotrc->joydir() != JOY_UP) pc[OUT] = pc[OPMIN];  // If in deadband or being pushed down, we want idle
            else pc[OUT] = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBTOP], hotrc->pc[VERT][OPMAX], pc[OPMIN], pc[GOVERN]);  // actuators still respond even w/ engine turned off
        }
    }
  public:
    void setmode(int _mode=NA) {
        mode_request = _mode;
        mode_busy = false;
        if (mode_request == Calibrate) {
            float temp = pot->mapToRange(0.0, 180.0);
            if (temp >= si[PARKED] && temp <= si[OPMAX]) motormode = Calibrate;
        }
        else if (mode_request == Cruise) {
            cruisepid.set_target(speedo->filt());  // set pid loop speed target to current speed  (for PID_SUSPEND_FLY mode)
            pid.set_target(tach->filt());  // initialize pid output (rpm target) to current rpm  (for PID_SUSPEND_FLY mode)
            cruise_target_pc = pc[OUT];  //  set target throttle angle to current throttle angle  (for THROTTLE_ANGLE/THROTTLE_DELTA modes)
            cruise_adjusting = cruise_trigger_released = false;  // in case trigger is being pulled as cruise mode is entered, the ability to adjust is only unlocked after the trigger is subsequently released to the center
            motormode = Cruise;
        }
        else if (mode_request != NA) motormode = mode_request;
        mode_request = NA;
    }
    int parked() {
        return (std::abs(pc_to_si(pc[OUT]) - si[PARKED]) < 1);
    }
    void update(int runmode) {
        float tach_now = tach->human();
        if (tach_now != tach_last) idlectrl.push_tach_reading(tach_now, tach->last_read_time());    
        tach_last = tach_now;
        if (pid_timer.expireset()) {
            // Step 1 : update throttle target from idle control or cruise mode pid, if applicable (on the same timer as gas pid)
            idlectrl.update();  // Allow idle control to mess with tach_target if necessary, or otherwise step in to prevent car from stalling
            set_output();
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
    IdleControl* throttle;
    float brakemotor_max_duty_pc = 40.0;  // In order to not exceed spec and overheat the actuator, limit brake presses when under pressure and adding pressure
    float press_initial_kp = 0.142;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float press_initial_ki = 0.000;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float press_initial_kd = 0.000;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
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
    float duty_integral = sens_ratio[PressurePID] * hybrid_out_ratio;
    int duty_index = 0, duty_index_last; 
    // float posn_inflect, pres_inflect, pc_inflect;
    void activate_pid(int _pid) {
        dominantpid = _pid;
        pid_dom = &(pids[dominantpid]);
        posn_pid_active = (dominantpid == PositionPID);
    }
  public:
    using JagMotor::JagMotor;
    int motormode = Halt, oldmode = Halt;
    int mode_request = NA;
    bool mode_busy = false;
    int pid_status = HybridPID;
    QPID pids[2];  // brake changes from pressure target to position target as pressures decrease, and vice versa
    QPID* pid_dom = &(pids[PressurePID]);  // AnalogSensor sensed[2];
    Timer stopcar_timer{stopcar_timeout};
    Timer interval_timer{interval_timeout};
    Timer motor_park_timer{4000000};
    float brake_pid_trans_threshold_lo = 0.25;  // tunable. At what fraction of full brake pressure will motor control begin to transition from posn control to pressure control
    float brake_pid_trans_threshold_hi = 0.50;  // tunable. At what fraction of full brake pressure will motor control be fully transitioned to pressure control
    bool autostopping = false, autoholding = false, reverse = false, duty_tracker_load_sensitive = true;
    // float duty_pc = 25;
    int dominantpid = brake_default_pid, dominantpid_last = brake_default_pid;
    bool posn_pid_active = (dominantpid == PositionPID);
    float panic_initial_pc = 60, hold_initial_pc = 40, panic_increment_pc = 4, hold_increment_pc = 2, pid_targ_pc, pid_err_pc, pid_final_out;
    float sens_ratio[2], sensnow[2], outnow[2], hybrid_math_offset, hybrid_math_coeff;  // , senslast[2], d_ratio[2], 
    float hybrid_sens_ratio, hybrid_sens_ratio_pc, hybrid_out_ratio = 1.0, hybrid_out_ratio_pc = 100.0;
    float duty_integral_sum, duty_instant, duty_continuous;
    void derive() {
        JagMotor::derive();
        hybrid_math_offset = 0.5 * (brake_pid_trans_threshold_lo + brake_pid_trans_threshold_hi);
        hybrid_math_coeff = M_PI / (brake_pid_trans_threshold_hi - brake_pid_trans_threshold_lo);
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt, PressureSensor* _pressure, BrakePositionSensor* _brkpos, IdleControl* _throttle) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        printf("Brake motor..\n");
        JagMotor::setup(_hotrc, _speedo, _batt);
        pressure = _pressure;  // press_pin = _press_pin;  // sensed[PressurePID] = _pressure;
        brkpos = _brkpos;  // posn_pin = _posn_pin;  // sensed[PositionPID] = _brkpos;
        throttle = _throttle;
        duty_fwd_pc = brakemotor_max_duty_pc;  
        pres_last = pressure->human();
        posn_last = brkpos->human();
        activate_pid(brake_default_pid);
        derive();
        if (brake_hybrid_pid) calc_hybrid_ratio();
        pids[PressurePID].init(pressure->filt_ptr(), &(pc[OPMIN]), &(pc[OPMAX]), press_initial_kp, press_initial_ki, press_initial_kd, QPID::pmod::onerr,
            QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout, QPID::ctrl::manual, QPID::centmod::on, pc[STOP]);
        pids[PositionPID].init(brkpos->filt_ptr(), &(pc[OPMIN]), &(pc[OPMAX]), posn_initial_kp, posn_initial_ki, posn_initial_kd, QPID::pmod::onerr, 
            QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::reverse, pid_timeout, QPID::ctrl::manual, QPID::centmod::on, pc[STOP]);
    }
    void set_pidtarg(float targ_pc) {
        pid_targ_pc = targ_pc;
        pids[PressurePID].set_target(pressure->min_human() + pid_targ_pc * (pressure->max_human() - pressure->min_human()) / 100.0);
        pids[PositionPID].set_target(brkpos->min_human() + (100.0 - pid_targ_pc) * (brkpos->max_human() - brkpos->min_human()) / 100.0);
    }
  private:
    // The influence on the final output from the pressure and position pids is weighted based on the pressure reading
    // Definition: "dominant" PID = The PID loop (pressure or position) that has the majority influence over the motor
    // PID outputs are weighted based on where in the pedal press we are. Near the bottom is 100% pressure control, and near the top is 100% position control.
    void calc_hybrid_ratio() {
        if (pid_status == HybridPID) {
            if (sens_ratio[PressurePID] >= brake_pid_trans_threshold_hi) hybrid_out_ratio = 1.0;  // at pressure above hi threshold, pressure has 100% influence
            else if (sens_ratio[PressurePID] <= brake_pid_trans_threshold_lo) hybrid_out_ratio = 0.0;  // at pressure below lo threshold, position has 100% influence
            else hybrid_out_ratio = 0.5 + 0.5 * sin(hybrid_math_coeff * (sens_ratio[PressurePID] - hybrid_math_offset));  // in between we make a steep but smooth transition during which both have some influence
            activate_pid((int)(sens_ratio[PressurePID] > 0.5 * (brake_pid_trans_threshold_lo + brake_pid_trans_threshold_hi)));  // pressure pid == 1. sets the "dominant" pid
        }
        else {
            hybrid_out_ratio = (float)pid_status;
            activate_pid(pid_status);
        }    
        hybrid_out_ratio_pc = 100.0 * hybrid_out_ratio;  // for display
    }
    float pid_out() {  // feedback brake pid with position or pressure, whichever is changing more quickly
        float range;
        for (int p = PositionPID; p <= PressurePID; p++) {
            range = (p == PressurePID) ? (pressure->max_human() - pressure->min_human()) : (brkpos->max_human() - brkpos->min_human());
            sensnow[p] = (p == PressurePID) ? pressure->human() : brkpos->human();  // latest sensed value
            sens_ratio[p] = (sensnow[p] - ((p == PressurePID) ? pressure->min_human() : brkpos->min_human())) / range;  // calculate ratio of output to range
            outnow[p] = pids[p].compute();  // get each pid calculated output
        }
        calc_hybrid_ratio();  // calculate pressure vs. position multiplier based on the sensed values
        pid_final_out = hybrid_out_ratio * outnow[PressurePID] + (1.0 - hybrid_out_ratio) * outnow[PositionPID];  // combine pid outputs weighted by the multiplier
        for (int p = PositionPID; p <= PressurePID; p++) pids[p].set_output(pid_final_out);  // Feed the final value back into the pids
        pid_final_out = outnow[dominantpid];
        return pid_final_out;
    }
    // Duty tracking: we keep a buffer of motor work done
    void calc_motor_duty() {
        ++duty_index %= history_depth;  // advance ring buffer index, this is the oldest value in the buffer
        duty_integral_sum -= (float)history[duty_index] / 2.5;  // subtract oldest value from our running sum.
        int duty_instant = std::abs(pc[OUT]);  // what is current motor drive percent
        if (duty_tracker_load_sensitive) {  // if we are being fancy
            if (pc[OUT] < 0) duty_instant = 0.0;  // ignore motion in the extend direction due to low load
            else duty_instant *= sens_ratio[PressurePID];  // scale load with brake pressure
        }        
        history[duty_index] = (uint8_t)(duty_instant * 2.5);  // write over current indexed value in the history ring buffer. scale to optimize for uint8
        duty_integral_sum += duty_instant;  // add value to our running sum
        duty_continuous = duty_integral_sum / history_depth;  // divide ring buffer total by its size to get average value
        if (duty_continuous < 0.001) duty_continuous = 0.0;  // otherwise displays "5.67e-" (?)
        // now we know our continuous duty, need to limit motor output based on this
    }
    // autostop: if car is moving, apply initial pressure plus incremental pressure every few seconds until it stops or timeout expires, then stop motor and cancel mode
    // autohold: apply initial moderate brake pressure, and incrementally more if car is moving. If car stops, then stop motor but continue to monitor car speed indefinitely, adding brake as needed
    void set_output() { // services any requests for change in brake mode
        autostopping = autoholding = false;
        if (motormode == AutoHold) {
            pid_status = HybridPID;
            set_pidtarg(std::max(hold_initial_pc, pid_dom->target()));  // Autohold always applies the brake somewhat, even if already stopped
            autoholding = speedo->car_stopped() && (pc[OUT] >= hold_initial_pc - pc[MARGIN]);  // this needs to be tested
            if (!speedo->car_stopped()) {
                autostopping = true;
                if (interval_timer.expireset()) set_pidtarg(std::min(pid_dom->target() + hold_increment_pc, 100.0f));
                pc[OUT] = pid_out();
            }
            else pc[OUT] = pc[STOP];
        }
        else if (motormode == AutoStop) {
            pid_status = HybridPID;
            if (panicstop) throttle->goto_idle();  // Stop pushing the gas, will help us stop the car better
            if (speedo->car_stopped() || stopcar_timer.expired()) {
                mode_request = Halt;  // After AutoStop mode stops the car or times out, then stop driving the motor
                pc[OUT] = pc[STOP];
            }
            else {
                autostopping = true;
                if (interval_timer.expireset()) set_pidtarg(std::min(pid_dom->target() + panicstop ? panic_increment_pc : hold_increment_pc, 100.0f));
                else set_pidtarg(std::max(panicstop ? panic_initial_pc : hold_initial_pc, pid_dom->target()));
                pc[OUT] = pid_out();
            }
        }
        else if (motormode == Halt) {
            pid_status = HybridPID;
            pc[OUT] = pc[STOP];
        }
        else if (motormode == Calibrate) {
            pid_status = HybridPID;
            cal_brakemode = true;
            int _joydir = hotrc->joydir(); 
            if (_joydir == JOY_UP) pc[OUT] = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBTOP], hotrc->pc[VERT][OPMAX], pc[STOP], pc[OPMAX]);
            else if (_joydir == JOY_DN) pc[OUT] = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][OPMIN], hotrc->pc[VERT][DBBOT], pc[OPMIN], pc[STOP]);
            else pc[OUT] = pc[STOP];
        }
        else if (motormode == Release) {
            pid_status = PositionPID;
            mode_busy = true;
            if (released() || motor_park_timer.expired()) {
                mode_request = Halt;
                mode_busy = false;
            }
            else set_pidtarg(map(brkpos->zeropoint(), brkpos->min_human(), brkpos->max_human(), 100.0, 0.0));  // Flipped to 100-value because function argument subtracts back for position pid
            pc[OUT] = pid_out();
        }
        else if (motormode == ParkMotor) {
            pid_status = PositionPID;
            parking = mode_busy = true;
            if (parked() || motor_park_timer.expired()) {
                mode_request = Halt;
                parking = mode_busy = false;
            }
            else set_pidtarg(map(brkpos->parkpos(), brkpos->min_human(), brkpos->max_human(), 100.0, 0.0));  // Flipped to 100-value because function argument subtracts back for position pid
            pc[OUT] = pid_out();
        }
        else if (motormode == ActivePID) {
            pid_status = HybridPID;
            if (hotrc->joydir(VERT) != JOY_DN) set_pidtarg(0);  // let off the brake
            else set_pidtarg(map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBBOT], hotrc->pc[VERT][OPMIN], 0.0, 100.0));  // If we are trying to brake, scale joystick value to determine brake pressure setpoint
            pc[OUT] = pid_out();
        }
        mode_busy = autostopping || parking;
    }
  public:
    void setmode(int _mode=NA) {
        mode_request = _mode;
        mode_busy = false;
        if ((mode_request == AutoStop || mode_request == AutoHold) && mode_request != motormode) {
            interval_timer.reset();
            stopcar_timer.reset();
        }
        else if ((mode_request == Release || mode_request == ParkMotor) && mode_request != motormode) {
            motor_park_timer.reset();
        }
        else if (mode_request != NA) motormode = mode_request;
        mode_request = NA;
        // if (motormode != oldmode) stopcar_timer.reset();    
    }
    int parked() {
        return (std::abs(brkpos->filt() - brkpos->parkpos()) <= brkpos->margin());   // (brkpos->filt() + brkpos->margin() > brkpos->parkpos());
    }
    int released() {
        return (std::abs(brkpos->filt() - brkpos->zeropoint()) <= brkpos->margin());   // (brkpos->filt() + brkpos->margin() > brkpos->parkpos());
    }
    void update(int runmode) {
        // Brakes - Determine motor output and write it to motor
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            calc_motor_duty();
            // Step 1 : Determine motor percent value
            set_output();
            // Step 2 : Fix motor pc value if it's out of range or exceeding positional limits
            if (motormode == Calibrate)  // Constrain the motor to the operational range, unless calibrating (then constraint already performed above)
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
    int motormode = Halt, oldmode = Halt, mode_request;
    float steer_safe_pc = 72.0;  // this percent is taken off full steering power when driving full speed (linearly applied)
    bool reverse = false;
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        printf("Steering motor..\n");
        JagMotor::setup(_hotrc, _speedo, _batt);
    }
    void update(int runmode) {
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            set_output();
            pc[OUT] = constrain(pc[OUT], pc[OPMIN], pc[OPMAX]);  // Don't be out of range
            us[OUT] = pc_to_us(pc[OUT]);
            volt[OUT] = pc_to_si(pc[OUT]);
            write_motor();
        }
    }
    void setmode(int _mode=NA) { mode_request = _mode; }
  private:
    void set_output() {
        if (mode_request != NA) motormode = mode_request;
        mode_request = NA;
        if (motormode == Halt) {
            pc[OUT] = pc[STOP];  // Stop the steering motor if in shutdown mode and shutdown is complete
        }
        else if (motormode == OpenLoop) {
            int _joydir = hotrc->joydir(HORZ);
            if (_joydir == JOY_RT) pc[OUT] = map(hotrc->pc[HORZ][FILT], hotrc->pc[HORZ][DBTOP], hotrc->pc[HORZ][OPMAX], pc[STOP], steer_safe(pc[OPMAX]));  // if joy to the right of deadband
            else if (_joydir == JOY_LT) pc[OUT] = map(hotrc->pc[HORZ][FILT], hotrc->pc[HORZ][DBBOT], hotrc->pc[HORZ][OPMIN], pc[STOP], steer_safe(pc[OPMIN]));  // if joy to the left of deadband
            else pc[OUT] = pc[STOP];  // Stop the steering motor if inside the deadband
        }
    }
    float steer_safe(float endpoint) {
        return pc[STOP] + (endpoint - pc[STOP]) * (1.0 - steer_safe_pc * speedo->filt() / (100.0 * speedo->redline_mph()));
    }
};