#pragma once
#include <stdint.h>
#include "utils.h"
#include "temperature.h"

// I stole this library and modified it heavily to our purpposes - Soren
// QPID Library for Arduino - Version 3.1.9 by dlloydev https://github.com/Dlloydev/QPID
// Based on the Arduino PID_v1 Library. Licensed under the MIT License.

class qpid {
  public:
    enum class ctrl : uint8_t {manual, automatic, toggle};            // controller mode
    enum class cdir : uint8_t {direct, reverse};                      // controller direction
    enum class pmod : uint8_t {onerr, onmeas, onerrmeas};             // proportional mode
    enum class dmod : uint8_t {onerr, onmeas};                        // derivative mode
    enum class awmod : uint8_t {cond, clamp, off, round, roundcond};  // integral anti-windup mode  // Soren edit
    enum class centmod : uint8_t {off, on, strict};                   // Soren - Allows a defined output zero point
  private:
    float dispkp = 0; float dispki = 0; float dispkd = 0;
    float _pterm, _iterm, _dterm, _kp, _ki, _kd, _outmin, _outmax, _err, lasterr, lastin, _cent, _outsum;
    float *myin;     // Pointers to the input, output, and target variables. This creates a
    float *myout;    // hard link between the variables and the PID, freeing the user from having
    float *mytarg;  // to constantly tell us what these values are. With pointers we'll just know.
    ctrl _mode = ctrl::manual;
    cdir _dir = cdir::direct;
    pmod _pmode = pmod::onerr;
    dmod _dmode = dmod::onmeas;
    awmod _awmode = awmod::cond;
    centmod _centmode = centmod::off;  // Soren
    uint32_t sampletime, lasttime;
  public:
    qpid();  // Default constructor
    // Constructor. Links the PID to input, output, target, initial tuning parameters and control modes.
    qpid(float *arg_in, float *arg_output, float *arg_targ, float arg_min, float arg_max, float arg_kp, float arg_ki, float arg_kd,  // Soren edit
        pmod arg_pmode, dmod arg_dmode, awmod arg_awmode, cdir arg_dir, uint32_t arg_sampletime, ctrl arg_mode, centmod arg_centmode, float arg_cent);  // Soren edit
    // qpid(float *in, float *out, float *targ, float Kp, float Ki, float Kd, dir dir);
    // qpid(float *in, float *out, float *targ);
    void set_mode(ctrl arg_mode);
    void set_mode(uint8_t arg_mode);
    bool compute();  // Performs the PID calculation
    void set_outlimits(float arg_min, float arg_max);  // Sets and clamps the output to a specific range (0-255 by default).
    void set_centmode(centmod arg_centmode);  // Soren
    void set_centmode(uint8_t arg_centmode);  // Soren
    void set_cent(float arg_cent);  // Soren
    void set_tunings(float arg_kp, float arg_ki, float arg_kd);
    void set_tunings(float arg_kp, float arg_ki, float arg_kd, pmod arg_pmode, dmod arg_dmode, awmod arg_awmode);
    void set_kp(float arg_kp);
    void set_ki(float arg_ki);
    void set_kd(float arg_kd);
    void add_kp(float arg_kp);
    void add_ki(float arg_ki);
    void add_kd(float arg_kd);
    void set_dir(cdir arg_dir);
    void set_dir(uint8_t arg_dir);
    void set_sampletime(uint32_t arg_sampletime);  // in microseconds
    void set_pmode(pmod arg_pmode);  // Sets the computation method for the proportional term, to compute based either on error (default), on measurement, or the average of both.
    void set_pmode(uint8_t arg_pmode);
    void set_dmode(dmod arg_dmode);  // Sets the computation method for the derivative term, to compute based either on error or measurement (default).
    void set_dmode(uint8_t arg_dmode);
    void set_awmode(awmod arg_awmode);  // Sets the integral anti-windup mode to one of awClamp, which clamps the output after adding integral and proportional (on measurement) terms, or awcond (default), which provides some integral correction, prevents deep saturation and reduces overshoot. Option awOff disables anti-windup altogether.
    void set_awmode(uint8_t arg_awmode);
    void set_outsum(float arg_outsum);  // sets the output summation value
    void init();        // Ensure a bumpless transfer from manual to automatic mode
    void reset();             // Clears _pterm, _iterm, _dterm and _outsum values
    // Getter functions
    float err() { return _err; }  // Soren
    float kp() { return dispkp; }
    float ki() { return dispki; }
    float kd() { return dispkd; }
    float pterm() { return _pterm; }
    float iterm() { return _iterm; }
    float dterm() { return _dterm; }
    float outsum() { return _outsum; }
    float outmin() { return _outmin; }  // Soren
    float outmax() { return _outmax; }  // Soren
    float outrange() { return _outmax - _outmin; }  // Soren
    float cent() { return _cent; }  // Soren
    uint8_t mode() { return static_cast<uint8_t>(_mode); }
    uint8_t dir() { return static_cast<uint8_t>(_dir); }
    uint8_t pmode() { return static_cast<uint8_t>(_pmode); }
    uint8_t dmode() { return static_cast<uint8_t>(_dmode); }
    uint8_t awmode() { return static_cast<uint8_t>(_awmode); }
    uint8_t centmode() { return static_cast<uint8_t>(_centmode); }  // Soren
};

qpid::qpid() {}

// Constructor that allows all parameters to get set
qpid::qpid(float* arg_in, float* arg_out, float* arg_targ, float arg_min, float arg_max,
    float arg_kp = 0, float arg_ki = 0, float arg_kd = 0,  // Soren edit
    pmod arg_pmode = pmod::onerr, dmod arg_dmode = dmod::onmeas,
    awmod arg_awmode = awmod::cond, cdir arg_dir = cdir::direct,
    uint32_t arg_sampletime = 100000, ctrl arg_mode = ctrl::manual, 
    centmod arg_centmode = centmod::off, float arg_cent = NAN) {  // Soren edit
    myout = arg_out;
    myin = arg_in;
    mytarg = arg_targ;
    _mode = arg_mode;
    qpid::set_outlimits(arg_min, arg_max);  // same default as Arduino PWM limit - Soren edit
    qpid::set_centmode(arg_centmode);  // Soren
    if (_centmode != centmod::off && !std::isnan(arg_cent)) {  // Soren
        set_cent(arg_cent);  // Soren
        _outsum = _cent;
    }
    else set_cent(_outmin);  // Soren
    sampletime = arg_sampletime;              // Soren edit
    qpid::set_dir(arg_dir);
    qpid::set_tunings(arg_kp, arg_ki, arg_kd, _pmode, _dmode, _awmode);
    lasttime = micros() - sampletime;  // Soren edit
}

// This function should be called every time "void loop()" executes. The function will decide whether a new 
// PID output needs to be computed. Returns true when the output is computed, false when nothing has been done.
bool qpid::compute() {
    uint32_t now = micros();  // Soren edit
    uint32_t timechange = (now - lasttime);
    if (_mode == ctrl::automatic && timechange < sampletime) return false;  // If class is handling the timing and this time was a nop

    float in = *myin;
    float din = in - lastin;
    if (_dir == cdir::reverse) din = -din;  // Soren

    _err = *mytarg - in;
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
        if (_itermout > _outmax && derr > 0) aw = true;
        else if (_itermout < _outmin && derr < 0) aw = true;
        if (aw && _ki) _iterm = constrain(_itermout, -_outmax, _outmax);
    }
    else if ((_awmode == awmod::round || _awmode == awmod::roundcond) && _err < 0.001 && _err > -0.001) {
        _err = 0.0;
        if (_centmode == centmod::on || _centmode == centmod::strict) _outsum = _cent;     
    }
    if (_centmode == centmod::strict && _err * lasterr < 0) _outsum = _cent;  // Soren - Recenters any old integral when error crosses zero

    _outsum += _iterm - pmterm;  // by default, compute output as per PID_v1    // include integral amount and pmterm
    if (_awmode != awmod::off) _outsum = constrain(_outsum, _outmin, _outmax);  // Clamp

    *myout = constrain(_outsum + peterm + _dterm, _outmin, _outmax);  // include _dterm, clamp and drive output

    lasterr = _err;  // Soren
    lastin = in;
    lasttime = now;
    return true;
}

// set_tunings  This function allows the controller's dynamic performance to be adjusted. It's called 
// automatically from the constructor, but tunings can also be adjusted on the fly during normal operation.
void qpid::set_tunings(float arg_kp, float arg_ki, float arg_kd, pmod arg_pmode = pmod::onerr,
    dmod arg_dmode = dmod::onmeas, awmod arg_awmode = awmod::cond) {
    if (arg_kp < 0 || arg_ki < 0 || arg_kd < 0 || !sampletime) return;  // Soren - added divide by zero protection
    if (arg_ki == 0) _outsum = 0;
    _pmode = arg_pmode; _dmode = arg_dmode; _awmode = arg_awmode;
    dispkp = arg_kp; dispki = arg_ki; dispkd = arg_kd;
    float sampletime_sec = (float)sampletime / 1000000;
    _kp = arg_kp;
    _ki = arg_ki * sampletime_sec;
    _kd = arg_kd / sampletime_sec;
}

// set_tunings  Set Tunings using the last remembered pmode, dmode and awmode settings.
void qpid::set_tunings(float arg_kp, float arg_ki, float arg_kd) {
    set_tunings(arg_kp, arg_ki, arg_kd, _pmode, _dmode, _awmode);
}

// Soren: I wrote these to facilitate changing only one tuning parameter at a time
void qpid::set_kp(float arg_kp) { set_tunings(arg_kp, dispki, dispkd, _pmode, _dmode, _awmode); }  // Soren
void qpid::set_ki(float arg_ki) { set_tunings(dispkp, arg_ki, dispkd, _pmode, _dmode, _awmode); }  // Soren
void qpid::set_kd(float arg_kd) { set_tunings(dispkp, dispki, arg_kd, _pmode, _dmode, _awmode); }  // Soren

void qpid::add_kp(float add) { set_kp(_kp + add); }  // Soren
void qpid::add_ki(float add) { set_ki(_ki + add); }  // Soren
void qpid::add_kd(float add) { set_kd(_kd + add); }  // Soren

// set_sampletime  Sets the period, in microseconds, at which the calculation is performed.
void qpid::set_sampletime(uint32_t arg_sampletime) {
    if (arg_sampletime > 0 && sampletime) {  // Soren - added more divide by zero protection
        float ratio  = (float)arg_sampletime / (float)sampletime;
        _ki *= ratio;
        _kd /= ratio;
        sampletime = arg_sampletime;
    }
}

// set_outLimits  The PID controller is designed to vary its output within a given range, default 0-255
void qpid::set_outlimits(float arg_min, float arg_max) {
    if (arg_min >= arg_max) return;
    _outmin = arg_min; _outmax = arg_max;

    // if (_mode != ctrl::manual) {
    *myout = constrain(*myout, _outmin, _outmax);  // Soren
    _outsum = constrain(_outsum, _outmin, _outmax);
}

void qpid::set_centmode(centmod arg_centmode) { _centmode = arg_centmode; }  // Soren
void qpid::set_centmode(uint8_t arg_centmode) { _centmode = (centmod)arg_centmode; }  // Soren
void qpid::set_cent(float arg_cent) { if (_outmin <= arg_cent && _outmax >= arg_cent) _cent = arg_cent; }  // Soren
// if (_centmode == centmod::off) _centmode = centmod::strict;  // Soren - does definition of center imply to use center mode?

// set_mode Sets the controller mode to manual (0), automatic (1) or timer (2) when the transition 
// from manual to automatic or timer occurs, the controller is automatically initialized.
void qpid::set_mode(ctrl arg_mode) {
    if (_mode == ctrl::manual && arg_mode != ctrl::manual) { // just went from manual to automatic
        _mode = ctrl::automatic;
        qpid::init();
    }
    else if (_mode == ctrl::automatic && arg_mode != ctrl::automatic) {
        _mode = ctrl::manual;
    }
}
void qpid::set_mode(uint8_t arg_mode) { set_mode((ctrl)arg_mode); }

// Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.
void qpid::init() {
    _outsum = (float)*myout;  // Soren
    lastin = *myin;
    _outsum = constrain(_outsum, _outmin, _outmax);
}

// The PID will either be connected to a direct acting process (+output leads to +input) or a reverse acting process(+output leads to -input).
void qpid::set_dir(cdir arg_dir) { _dir = arg_dir; }
void qpid::set_dir(uint8_t arg_dir) { _dir = (cdir)arg_dir; }

// Sets the computation method for the proportional term, to compute based either on error (default), on measurement, or the average of both.
void qpid::set_pmode(pmod arg_pmode) { _pmode = arg_pmode; }
void qpid::set_pmode(uint8_t arg_pmode) { _pmode = (pmod)arg_pmode; }

// Sets the computation method for the derivative term, to compute based either on error or on measurement (default).
void qpid::set_dmode(dmod arg_dmode) { _dmode = arg_dmode; }
void qpid::set_dmode(uint8_t arg_dmode) { _dmode = (dmod)arg_dmode; }

// Sets the integral anti-windup mode to one of clamp, which clamps the output after adding integral and proportional (on measurement) terms,
// or cond (default), which provides some integral correction, prevents deep saturation and reduces overshoot. Option off disables anti-windup altogether.
void qpid::set_awmode(awmod arg_awmode) { _awmode = arg_awmode; }
void qpid::set_awmode(uint8_t arg_awmode) { _awmode = (awmod)arg_awmode; }

void qpid::reset() {
    lasttime = micros() - sampletime;  // Soren edit
    lastin = 0; _outsum = 0;
    _pterm = 0; _iterm = 0; _dterm = 0;
}

// sets the output summation value
void qpid::set_outsum(float arg_outsum) { _outsum = arg_outsum; }


class ThrottleControl {  // Soren - To allow creative control of PID targets in case your engine idle problems need that.
  public:
    enum class idlemodes : uint32_t { direct, control, minimize, num_idlemodes };  // direct: disable idle management.  control: soft landing to idle rpm.  minimize: attempt to minimize idle to edge of instability
    enum targetstates : uint32_t { todrive, driving, droptohigh, droptolow, idling, minimizing, num_states };
  protected:
    // String modenames[3] = { "direct", "cntrol", "minimz" };
    // String statenames[4] = { "drivng", "tohigh", "tolow", "tostal" };
    targetstates runstate, nextstate;
    idlemodes _idlemode;
    float target_rpm, targetlast_rpm, idle_rpm, idlehigh_rpm, idlehot_rpm, idlecold_rpm, stallpoint_rpm, dynamic_rpm, temphot_f, tempcold_f, idle_slope_rpmps;
    float margin_rpm = 10; float idle_absmax_rpm = 1000.0;  // High limit of idle speed adjustability
    float* measraw_rpm; float* measfilt_rpm; float engine_temp_f;
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
    ThrottleControl (float* measraw, float* measfilt, // Variable references: idle target, rpm raw, rpm filt
      float idlehigh, float idlehot, float idlecold,  // Values for: high-idle rpm (will not stall), hot idle nominal rpm, cold idle nominal rpm 
      float tempcold, float temphot,  // Values for: engine operational temp cold (min) and temp hot (max) in degrees-f
      int32_t settlerate = 100,  // Rate to lower idle from high point to low point (in rpm per second)
      idlemodes myidlemode = idlemodes::control) {  // Configure idle control to just soft land or also attempt to minimize idle
        measraw_rpm = measraw;
        measfilt_rpm = measfilt;
        target_rpm = *measfilt_rpm;
        set_idlehigh (idlehigh);
        idlehot_rpm = constrain (idlehot, 0.0, idlehigh_rpm);
        stallpoint_rpm = idlehot_rpm - 1;  // Just to give a sane initial value
        set_idlecold (idlecold);
        set_temphot (temphot);
        set_tempcold (tempcold);        
        calc_idlespeed();
        targetlast_rpm = target_rpm;
        settlerate_rpmps = settlerate;
        settleTimer.reset();
        _idlemode = myidlemode;
        runstate = driving;
    }
    void setup(TemperatureSensor* engine_sensor_ptr) {
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
        if ((int32_t)target_rpm != (int32_t)argtarget) {
            target_externally_set = true;
            set_target_internal (argtarget);
        }
    }
    void set_engine_sensor (TemperatureSensor* sensor) {
      engine_sensor = sensor;
    }
  protected:
    void set_target_internal (float argtarget) {
        if ((int32_t)target_rpm != (int32_t)argtarget) {
            targetlast_rpm = target_rpm;
            target_rpm = argtarget;
        }
    }
    void calc_idlespeed (void) {
        idle_rpm = map (engine_temp_f, tempcold_f, temphot_f, idlecold_rpm, idlehot_rpm);
        idle_rpm = constrain (idle_rpm, idlehot_rpm, idlecold_rpm);
    }
    void runstate_changer (void) {  // If nextstate was changed during last update, or someone externally changed the target, change our runstate
        if (target_externally_set) {  // If the target has been changed externally, then determine our appropriate runstate based on target value
            if (target_rpm > idle_rpm + margin_rpm) nextstate = (*measfilt_rpm > idlehigh_rpm) ? driving : todrive;
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
    float target (void) { return target_rpm; }
};