// motors.h : centralized wrapper classes for all braking, throttle, and steering activity, as holistic coordination between sensors, actuator, and intent is critical
#pragma once
#include <ESP32Servo.h>  // Eventually move Servos into new ServoPWM objects then remove this
#include "temperature.h"
#include <cmath>

// I stole this library and modified it heavily to our purposes - Soren
// QPID Library for Arduino - Version 3.1.9 by dlloydev https://github.com/Dlloydev/QPID
// Based on the Arduino PID_v1 Library. Licensed under the MIT License.
// some general pid information:
// https://www.youtube.com/watch?v=6OH-wOsVVjg&ab_channel=SiieeFPV
// https://www.youtube.com/watch?v=qKy98Cbcltw&ab_channel=Horizon4electronics
class QPID {
  public:
    enum class ctrl : int {manual, automatic, toggle};            // controller mode
    enum class cdir : int {reverse=-1, direct=1};                 // controller direction
    enum class pmod : int {onerr, onmeas, onerrmeas};             // proportional mode
    enum class dmod : int {onerr, onmeas};                        // derivative mode
    enum class awmod : int {cond, clamp, off, round, roundcond};  // integral anti-windup mode
    enum class centmod : int {off, on, strict};                   // Allows a defined output zero point
  private:
    float dispkp = 0, dispki = 0, dispkd = 0, _iaw_cond_thresh = 1.0;  // set what fraction of output range beyond which applies integral anti-windup in cond mode
    float _pterm, _iterm, _dterm, _kp, _ki, _kd, _err, lasterr, lastin, _cent, _outsum, _target;
    float *myin;     // Pointers to the input, output, and target variables. This  creates a
    float *_outmin;
    float *_outmax;
    float _output;
    bool pause_diff;
    ctrl _mode = ctrl::manual;
    cdir _dir = cdir::direct;
    pmod _pmode = pmod::onerr;
    dmod _dmode = dmod::onmeas;
    awmod _awmode = awmod::cond;
    centmod _centmode = centmod::off;
    int _sampletime, lasttime;
  public:
    QPID() {}  // Default constructor
    QPID(float* a_in, float* a_min, float* a_max, float a_kp = 0, float a_ki = 0, float a_kd = 0,
      pmod a_pmode = pmod::onerr, dmod a_dmode = dmod::onmeas, awmod a_awmode = awmod::cond, cdir a_dir = cdir::direct,
      int a_sampletime = 100000, ctrl a_mode = ctrl::manual, centmod a_centmode = centmod::off, float a_cent = NAN) {
        init(a_in, a_min, a_max, a_kp, a_ki, a_kd, a_pmode, a_dmode, a_awmode, a_dir, a_sampletime, a_mode, a_centmode, a_cent);
    }
    void init(float* a_in, float* a_min, float* a_max, float a_kp = 0, float a_ki = 0, float a_kd = 0,
      pmod a_pmode = pmod::onerr, dmod a_dmode = dmod::onmeas, awmod a_awmode = awmod::cond, cdir a_dir = cdir::direct,
      int a_sampletime = 100000, ctrl a_mode = ctrl::manual, centmod a_centmode = centmod::off, float a_cent = NAN) {
        myin = a_in;
        _mode = a_mode;
        _outmin = a_min;
        _outmax = a_max;
        _output = constrain(_output, *_outmin, *_outmax);
        _outsum = constrain(_outsum, *_outmin, *_outmax);
        set_centmode(a_centmode);
        if (_centmode != centmod::off && !std::isnan(a_cent)) {
            set_cent(a_cent);
            _outsum = _cent;
        }
        else set_cent(*_outmin);
        _sampletime = a_sampletime;
        set_dir(a_dir);
        set_tunings(a_kp, a_ki, a_kd, _pmode, _dmode, _awmode);
        lasttime = micros() - _sampletime;
    }
    void init(float preload_output = NAN) {  // Ensure a bumpless transfer from manual to automatic mode
        if (!std::isnan(preload_output)) _output = preload_output;
        _outsum = _output;
        lastin = *myin;
        _outsum = constrain(_outsum, *_outmin, *_outmax);
    }
    // This function should be called every time "void loop()" executes. The function will decide whether a new 
    // PID output needs to be computed. Returns true when the output is computed, false when nothing has been done.
    float compute() {
        int now = micros();
        int timechange = (now - lasttime);
        if (_mode == ctrl::automatic && timechange < _sampletime) return _output;  // If class is handling the timing and this time was a nop

        float in = *myin;
        float din = in - lastin;
        if (_dir == cdir::reverse) din = -din;

        _err = _target - in;
        if (_dir == cdir::reverse) _err = -_err;
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
            if (_itermout > (*_outmax * _iaw_cond_thresh) && derr > 0) aw = true;
            else if (_itermout < (*_outmin * _iaw_cond_thresh) && derr < 0) aw = true;
            if (aw && _ki) _iterm = constrain(_itermout, -(*_outmax * _iaw_cond_thresh), *_outmax * _iaw_cond_thresh);
        }
        if ((_awmode == awmod::round || _awmode == awmod::roundcond) && _err < 0.001 && _err > -0.001) {
            _err = 0.0;
            if (_centmode == centmod::on || _centmode == centmod::strict) _outsum = _cent;     
        }
        if (_centmode == centmod::strict && _err * lasterr < 0) _outsum = _cent;  // Recenters any old integral when error crosses zero
        
        if (pause_diff) _dterm = 0.0;  // avoid differential effect on this cycle due to recent external setting of output
        pause_diff = false;

        _outsum += _iterm;  // by default, compute output as per PID_v1    // include integral amount and pmterm
        if (_awmode == awmod::off) _outsum -= pmterm;
        else _outsum = constrain(_outsum - pmterm, *_outmin, *_outmax);  // Clamp
        _output = constrain(_outsum + peterm + _dterm, *_outmin, *_outmax);  // include _dterm, clamp and drive output
        
        lasterr = _err;
        lastin = in;
        lasttime = now;
        return _output;
    }
    // set_tunings  This function allows the controller's dynamic performance to be adjusted. It's called 
    // automatically from the constructor, but tunings can also be adjusted on the fly during normal operation.
    // void set_tunings(float a_kp, float a_ki, float a_kd, pmod a_pmode = pmod::onerr, dmod a_dmode = dmod::onmeas, awmod a_awmode = awmod::cond) {
    void set_tunings(float a_kp, float a_ki, float a_kd, pmod a_pmode, dmod a_dmode, awmod a_awmode) {
        if (a_kp < 0 || a_ki < 0 || a_kd < 0 || !_sampletime) return;  // added divide by zero protection
        if (a_ki == 0) _outsum = 0;
        _pmode = a_pmode; _dmode = a_dmode; _awmode = a_awmode;
        if (std::abs(a_kp) < float_conversion_zero) a_kp = 0.0;
        if (std::abs(a_ki) < float_conversion_zero) a_ki = 0.0;
        if (std::abs(a_kd) < float_conversion_zero) a_kd = 0.0;
        dispkp = a_kp; dispki = a_ki; dispkd = a_kd;
        float sampletime_sec = (float)_sampletime / 1000000;
        _kp = a_kp;
        _ki = a_ki * sampletime_sec;
        _kd = a_kd / sampletime_sec;
    }
    // set_tunings  Set Tunings using the last remembered pmode, dmode and awmode settings.
    // void set_tunings(float a_kp, float a_ki, float a_kd) {
    void set_tunings(float a_kp, float a_ki, float a_kd) {
        set_tunings(a_kp, a_ki, a_kd, _pmode, _dmode, _awmode);
    }
    // set_sampletime  Sets the period, in microseconds, at which the calculation is performed.
    void set_sampletime(int a_sampletime) {
        if (a_sampletime > 0 && _sampletime) {  // added more divide by zero protection
            float ratio  = (float)a_sampletime / (float)_sampletime;
            _ki *= ratio;
            _kd /= ratio;
            _sampletime = a_sampletime;
        }
    }
    // set_mode Sets the controller mode to manual (0), automatic (1) or timer (2) when the transition 
    // from manual to automatic or timer occurs, the controller is automatically initialized.
    void set_mode(ctrl a_mode) {
        if (_mode == ctrl::manual && a_mode != ctrl::manual) { // just went from manual to automatic
            _mode = ctrl::automatic;
            init();
        }
        else if (_mode == ctrl::automatic && a_mode != ctrl::automatic) _mode = ctrl::manual;
    }
    void set_mode(int a_mode) { set_mode((ctrl)a_mode); }
    // Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.
    void reset() {
        lasttime = micros() - _sampletime;
        lastin = 0; _outsum = 0;
        _pterm = 0; _iterm = 0; _dterm = 0;
    }
    void set_kp(float a_kp) { set_tunings(std::max(0.0f, a_kp), dispki, dispkd, _pmode, _dmode, _awmode); }
    void set_ki(float a_ki) { set_tunings(dispkp, std::max(0.0f, a_ki), dispkd, _pmode, _dmode, _awmode); }
    void set_kd(float a_kd) { set_tunings(dispkp, dispki, std::max(0.0f, a_kd), _pmode, _dmode, _awmode); }
    // void add_kp(float add) { set_kp(dispkp + add); }
    // void add_ki(float add) { set_ki(dispki + add); }
    // void add_kd(float add) { set_kd(dispkd + add); }
    // void add_sampletime(int add) { set_sampletime(_sampletime + add);}
    void set_centmode(centmod a_centmode) { _centmode = a_centmode; }
    void set_centmode(int a_centmode) { _centmode = (centmod)a_centmode; }
    void set_cent(float a_cent) { if (*_outmin <= a_cent && *_outmax >= a_cent) _cent = a_cent; }
    void set_target(float a_target) { _target = a_target; }
    void set_output(float a_output) { 
        a_output = constrain(a_output, *_outmin, *_outmax);
        if (std::abs(_output) > float_zero) {
            float ratio = a_output / _output;
            _output *= ratio;
            _outsum *= ratio;
        }
        else _output = _outsum = a_output;
        pause_diff = true;
    }
    void set_iaw_cond_thresh(float thresh) { _iaw_cond_thresh = std::max(0.0f, thresh); }  // place the integral windup limit to be this fraction of the output range 
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
    void set_limits(float* a_min, float* a_max) {
        _outmin = a_min;
        _outmax = a_max;
        _output = constrain(_output, *_outmin, *_outmax);
        _outsum = constrain(_outsum, *_outmin, *_outmax);
    }
    // Getter functions
    float err() { return _err; }
    float kp() { return dispkp; }
    float ki() { return dispki; }
    float kd() { return dispkd; }
    int sampletime() { return _sampletime; }  // provides sampletime in ms
    float pterm() { return _pterm; }
    float iterm() { return _iterm; }
    float dterm() { return _dterm; }
    float outsum() { return _outsum; }
    float outmin() { return *_outmin; }
    float outmax() { return *_outmax; }
    float outrange() { return *_outmax - *_outmin; }
    float cent() { return _cent; }
    float target() { return _target; }
    float output() { return _output; }
    int mode() { return static_cast<int>(_mode); }
    int dir() { return static_cast<int>(_dir); }
    int pmode() { return static_cast<int>(_pmode); }
    int dmode() { return static_cast<int>(_dmode); }
    int awmode() { return static_cast<int>(_awmode); }
    int centmode() { return static_cast<int>(_centmode); }
    float* target_ptr() { return &_target; }
};

static std::string motormodecard[NumMotorModes] = { "NA", "Halt", "Idle", "Releas", "OpLoop", "Thresh", "ActPID", "AuStop", "AuHold", "Park", "Cruise", "Calib", "Start" };
static std::string cruiseschemecard[NumCruiseSchemes] = { "FlyPID", "TrPull", "TrHold" };
static std::string brakefeedbackcard[NumBrakeFB] = { "BkPosn", "BkPres", "Hybrid", "None" };
static std::string openloopmodecard[NumOpenLoopModes] = { "Median", "AutRel", "ARHold" };

// ServoMotor - a parent class providing common elements for motor manager child classes. These subclasses each
// coordinate all aspects of one motor, including related sensors & pid, having a standard control interface
class ServoMotor {
  protected:
    Hotrc* hotrc;
    Speedometer* speedo;
    Servo motor;
    static const int pid_timeout = 85000;
    float lastoutput;
    Timer pid_timer{pid_timeout}, outchangetimer;
    int pin, freq;
  public:
    bool reverse = false;  // defaults. subclasses override as necessary
    float pc[NUM_MOTORVALS] = { 0, NAN, 100, 0, NAN, NAN, NAN, NAN };  // percent values [OPMIN/PARKED/OPMAX/OUT/GOVERN/ABSMIN/ABSMAX/MARGIN]  values range from -100% to 100% are all derived or auto-assigned
    float si[NUM_MOTORVALS] = { 45.0, 43.0, 168.2, 45.0, NAN, 0, 180, 1.0 };  // standard si-unit values [OPMIN/PARKED/OPMAX/OUT/GOVERN/ABSMIN/ABSMAX/MARGIN]
    float us[NUM_MOTORVALS] = { NAN, 1500, NAN, NAN, NAN, 500, 2500, NAN };  // us pulsewidth values [-/CENT/-/OUT/-/ABSMIN/ABSMAX/-]
    float max_out_change_rate_pcps = 800.0;
    ServoMotor(int _pin, int _freq) { pin = _pin; freq = _freq; }
    void setup(Hotrc* _hotrc, Speedometer* _speedo) {
        hotrc = _hotrc;
        speedo = _speedo;
        motor.setPeriodHertz(freq);
        motor.attach(pin, us[ABSMIN], us[ABSMAX]);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    }
    float out_pc_to_si(float _pc) {  // Eventually this should be linearized
        return map(_pc, pc[ABSMIN], pc[ABSMAX], si[ABSMIN], si[ABSMAX]); 
    }
    float out_si_to_pc(float _si) {  // Eventually this should be linearized
        return map(_si, si[ABSMIN], si[ABSMAX], pc[ABSMIN], pc[ABSMAX]);
    }
    float out_si_to_us(float _si) {  // works for motor with or without stop value
        return map(_si, si[ABSMIN], si[ABSMAX], reverse ? us[ABSMAX] : us[ABSMIN], reverse ? us[ABSMIN] : us[ABSMAX]);
    }
    float out_pc_to_us(float _pc) {  // works for motor with or without stop value
        return map(_pc, pc[ABSMIN], pc[ABSMAX], reverse ? us[ABSMAX] : us[ABSMIN], reverse ? us[ABSMIN] : us[ABSMAX]);
    }
    void write_motor() {
        if (!std::isnan(us[OUT])) motor.writeMicroseconds((int32_t)(us[OUT]));
        lastoutput = pc[OUT];    
    }
  protected:
    void changerate_limiter() {
        float max_out_change_pc = max_out_change_rate_pcps * outchangetimer.elapsed() / 1000000.0;
        outchangetimer.reset();
        pc[OUT] = constrain(pc[OUT], lastoutput - max_out_change_pc, lastoutput + max_out_change_pc);
        // lastoutput = pc[OUT];  // NOTE you must set lastoutput = pc[OUT]
    }
};
class JagMotor : public ServoMotor {
  protected:
    CarBattery* mulebatt;
    float car_batt_fake_v = 12.0;
    Timer volt_check_timer{3500000};
  public:
    using ServoMotor::ServoMotor;
    float duty_fwd_pc = 100;  // default. subclasses override as necessary
    float duty_rev_pc = 100;  // default. subclasses override as necessary
    float pc[NUM_MOTORVALS] = { NAN, 0, NAN, 0, NAN, -100, 100, 2 };  // percent values [OPMIN/STOP/OPMAX/OUT/-/ABSMIN/ABSMAX/MARGIN]  values range from -100% to 100% are all derived or auto-assigned
    float si[NUM_MOTORVALS] = { NAN, 0, NAN, 0, NAN, NAN, NAN, NAN };  // standard si-unit values [OPMIN/STOP/OPMAX/OUT/-/ABSMIN/ABSMAX/MARGIN]
    float us[NUM_MOTORVALS] = { NAN, 1500, NAN, NAN, NAN, 670, 2330, NAN };  // us pulsewidth values [-/CENT/-/OUT/-/ABSMIN/ABSMAX/-]
    float (&volt)[arraysize(si)] = si;  // our standard si value is volts. Create reference so si and volt are interchangeable
    #if !BrakeThomson
    // set opmin to avoid driving motor with under 8%
    #endif
    // JagMotor(int _pin, int _freq) : ServoMotor(_pin, _freq) {}
    void derive() {  // calc pc and voltage op limits from volt and us abs limits 
        si[ABSMAX] = running_on_devboard ? car_batt_fake_v : mulebatt->val();
        si[ABSMIN] = -(si[ABSMAX]);
        pc[OPMIN] = pc[ABSMIN] * duty_rev_pc / 100.0;
        pc[OPMAX] = pc[ABSMAX] * duty_fwd_pc / 100.0;
        si[OPMIN] = map(pc[OPMIN], pc[STOP], pc[ABSMIN], si[STOP], si[ABSMIN]);
        si[OPMAX] = map(pc[OPMAX], pc[STOP], pc[ABSMAX], si[STOP], si[ABSMAX]);
        si[MARGIN] = map(pc[MARGIN], pc[ABSMIN], pc[ABSMAX], si[ABSMIN], si[ABSMAX]);
        // us[MARGIN] = map(pc[MARGIN], pc[ABSMIN], pc[ABSMAX], us[ABSMIN], us[ABSMAX]);
    }
    float out_pc_to_si(float _pc) {  // Eventually this should be linearized
        if (_pc > pc[STOP]) return map(_pc, pc[STOP], pc[ABSMAX], si[STOP], si[ABSMAX]);
        if (_pc < pc[STOP]) return map(_pc, pc[STOP], pc[ABSMIN], si[STOP], si[ABSMIN]);
        return si[STOP];
    }
    float out_si_to_pc(float _si) {  // Eventually this should be linearized
        if (_si > si[STOP]) return map(_si, si[STOP], si[ABSMAX], pc[STOP], pc[ABSMAX]);
        if (_si < si[STOP]) return map(_si, si[STOP], si[ABSMIN], pc[STOP], pc[ABSMIN]);
        return pc[STOP];
    }
    float out_si_to_us(float _si) {  // works for motor with center stop value
        if (_si > si[STOP]) return map(_si, si[STOP], si[ABSMAX], us[STOP], reverse ? us[ABSMIN] : us[ABSMAX]);
        if (_si < si[STOP]) return map(_si, si[STOP], si[ABSMIN], us[STOP], reverse ? us[ABSMAX] : us[ABSMIN]);
        return us[STOP];
    }
    float out_pc_to_us(float _pc) {  // works for motor with center stop value
        if (_pc > pc[STOP]) return map(_pc, pc[STOP], pc[ABSMAX], us[STOP], reverse ? us[ABSMIN] : us[ABSMAX]);
        if (_pc < pc[STOP]) return map(_pc, pc[STOP], pc[ABSMIN], us[STOP], reverse ? us[ABSMAX] : us[ABSMIN]);
        return us[STOP];
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {
        ServoMotor::setup(_hotrc, _speedo);
        mulebatt = _batt;
        derive();
        pc[OUT] = out_si_to_pc(si[OUT]);
        us[OUT] = out_si_to_us(si[OUT]);
    }
    void write_motor() { if (!std::isnan(us[OUT])) motor.writeMicroseconds((int32_t)(us[OUT])); }
};
// pid_config : (configurable default)
//    OpenLoop : Servo angle is simply proportional to trigger pull. This is our tested default
//    ActivePID : Servo angle is determined by PID calculation designed to converge engine rpm to a target value set proportional to trigger pull
// cruise_adjust_scheme : (compile time option) Pick from 3 different styles for adjustment of cruise setpoint. I prefer TriggerHold.
//    TriggerPull : Cruise locks servo angle (throttle_target_pc), instead of pid. Moving trigger from center adjusts setpoint proportional to how far you push it before releasing (and reduced by an attenuation factor)
//    TriggerHold : Cruise locks servo angle (throttle_target_pc), instead of pid. Any non-center trigger position continuously adjusts setpoint proportional to how far it's pulled over time (up to a specified maximum rate)
//    SuspendFly : Cruise uses its own pid targeting a speed value. The PID output is either a servo angle or an rpm target for the gas pid, depending on pid_config setting above. Whatever speed you're at when trigger releases is new cruise target  
class ThrottleControl : public ServoMotor {
// replace with class ThrottleControl {
  private:
    Tachometer* tach;
    Potentiometer* pot;
    TemperatureSensorManager* tempsens;
    float cruise_pidgas_kp = 5.57;    // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
    float cruise_pidgas_ki = 0.000;   // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
    float cruise_pidgas_kd = 0.000;   // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
    float cruise_opengas_kp = 7.00;   // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
    float cruise_opengas_ki = 0.000;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
    float cruise_opengas_kd = 0.000;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
    float gas_kp = 0.013;             // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
    float gas_ki = 0.000;             // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
    float gas_kd = 0.000;             // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
    float cruise_ctrl_extent_pc, adjustpoint, ctrlratio;  // During cruise adjustments, saves farthest trigger position read
    Timer cruiseDeltaTimer, throttleRateTimer;
    bool pid_ena_last = false, cruise_pid_ena_last = false;
  public:
    // replace
    using ServoMotor::ServoMotor;
    float tach_last, throttle_target_pc, governor = 95, max_throttle_angular_velocity_pcps;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
    float idle_si[NUM_MOTORVALS] = { 45.0, NAN, 60.0, 58.0, NAN, 43.0, 75.0, 1.0 };          // in angular degrees [OPMIN(hot)/-/OPMAX(cold)/OUT/-/ABSMIN/ABSMAX/MARGIN]
    float idletemp_f[NUM_MOTORVALS] = { 60.0, NAN, 205.0, 75.0, NAN, 40.0, 225.0, 1.5};      // in degrees F [OPMIN/-/OPMAX/OUT/-/ABSMIN/ABSMAX/MARGIN]
    float idle_pc = 11.3;                              // idle percent is derived from the si (degrees) value
    float starting_pc = 25.0;                          // percent throttle to open to while starting the car
    // with:
    // ThrottleServo servo;
    // ThrottleControl(int pin, int freq) {
    //     servo = ThrottleServo(pin, freq);
    // }
    // float throttle_target_pc, tach_last;

    QPID pid, cruisepid;
    int throttle_ctrl_mode = OpenLoop;   // set default ctrl behavior. should gas servo use the rpm-sensing pid? values: ActivePID or OpenLoop
    bool pid_enabled = false, cruise_pid_enabled = false;
    int cruise_adjust_scheme = TriggerHold;  // edit these to be the defaults on boot
    int motormode = Idle;  // not tunable  // pid_status = OpenLoop, cruise_pid_status = OpenLoop,
    bool cruise_trigger_released = false, reverse = false;  // if servo higher pulsewidth turns ccw, then do reverse=true
    float (&deg)[arraysize(si)] = si;                  // our standard si value is degrees of rotation "deg". Create reference so si and deg are interchangeable
    float max_throttle_angular_velocity_degps = 65.0;  // deg/sec How quickly can the throttle change angle?  too low is unresponsive, too high can cause engine hesitations (going up) or stalls (going down)
    float pc_to_rpm(float _pc) {
        return map(_pc, 0.0, 100.0, tach->idle(), tach->opmax());
    }
    float rpm_to_pc(float _rpm) {
        return map(_rpm, tach->idle(), tach->opmax(), 0.0, 100.0);
    }
    void derive() {  // calc derived limit values for all units based on tuned values for each motor
        pc[ABSMIN] = map(si[ABSMIN], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[ABSMAX] = map(si[ABSMAX], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[PARKED] = map(si[PARKED], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        pc[GOVERN] = map(governor, 0.0, 100.0, pc[OPMIN], pc[OPMAX]);  // pc[GOVERN] = pc[OPMIN] + governor * (pc[OPMAX] - pc[OPMIN]) / 100.0;      
        si[GOVERN] = map(pc[GOVERN], pc[OPMIN], pc[OPMAX], si[OPMIN], si[OPMAX]);
        pc[MARGIN] = map(si[MARGIN], si[OPMIN], si[OPMAX], pc[OPMIN], pc[OPMAX]);
        max_throttle_angular_velocity_pcps = 100.0 * max_throttle_angular_velocity_degps / (si[OPMAX] - si[OPMIN]);
    }
    void linearize_throttle() {  // todo : compensate for at least 3 known sources of non-linearity in the throttle
        // 1. The coin-shaped butterfly valve blocks the cylindrical carb intake passage. The engine power is a function
        // of the airflow which is a function of the elliptical cross-section of the valve as it rotates. Here's math:
        // d = throttle intake / valve diameter (about 1 inch)
        // g = angle of valve, with 90 deg = full closed and 0 deg = full open
        // Open Area:  A = pi/4 * d^2 - pi * d^2 * cos(g)    // in squared units of d
        // Open Ratio: R = 1 - 4 * cos(g)                    // as a ratio to full-open (range 0-1)
        //
        // 2. Due to the mount and pull angles, the servo opens the throttle more degrees per degree of its own rotation 
        // when the throttle is closed vs. when it's open. Need to do the math for this, but I imagine it'll be another 
        // curve from 1 to 0, perhaps another cosine, to multiply by depending on current servo angle. Note it's possible
        // to mechanically nullify this pull angle non-linearity by having a rotational linkage instead of servo horns,
        // for example like a belt, or maybe pulleys on both servo and valve with a string wrapped around them.
        //
        // 3. The engine will characteristically produce power as a nonlinear function of carb airflow. Data for this would
        // be nice. [Research into typical curves] < [Specific performance data for this engine] < [Do empirical testing].
        //
        // It's a bit too complex (I'm too ignorant) to predict which of the above might serve to cancel/exacerbate the 
        // others. None of this matters if we use the throttle PID, only if we run open loop
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, Tachometer* _tach, Potentiometer* _pot, TemperatureSensorManager* _temp) {
        tach = _tach;  pot = _pot;  tempsens = _temp;
        ezread.squintf("Throttle servo.. pid %s\n", pid_enabled ? "enabled" : "disabled");
        ezread.squintf("  Cruise pid %s, using %s adj scheme\n", cruise_pid_enabled ? "enabled" : "disabled", cruiseschemecard[cruise_adjust_scheme].c_str());
        ServoMotor::setup(_hotrc, _speedo);
        throttleRateTimer.reset();
        derive();
        pid.init(tach->ptr(), &pc[OPMIN], &pc[OPMAX], gas_kp, gas_ki, gas_kd, QPID::pmod::onerr, 
            QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout);
        if (pid_enabled) {  // to-do need to dynamically recreate pid if changed during runtime
            cruisepid.init(speedo->ptr(), tach->idle_ptr(), tach->opmax_ptr(), cruise_pidgas_kp, cruise_pidgas_ki,
              cruise_pidgas_kd, QPID::pmod::onerr, QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout);
        }
        else {  // if OpenLoop
            cruisepid.init(speedo->ptr(), &pc[OPMIN], &pc[OPMAX], cruise_opengas_kp, cruise_opengas_ki,
              cruise_opengas_kd, QPID::pmod::onerr, QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout);
        }
    }
    void update_idlespeed() {
        // ezread.squintf("idle");
        if (!std::isnan(tempsens->val(loc::ENGINE))) idletemp_f[OUT] = tempsens->val(loc::ENGINE);
        if (std::isnan(idletemp_f[OUT])) return;
        idle_si[OUT] = map(idletemp_f[OUT], idletemp_f[OPMIN], idletemp_f[OPMAX], idle_si[OPMAX], idle_si[OPMIN]);
        idle_si[OUT] = constrain(idle_si[OUT], idle_si[OPMIN], idle_si[OPMAX]);
        idle_pc = out_si_to_pc(idle_si[OUT]);
        tach->set_idle(map(idletemp_f[OUT], idletemp_f[OPMIN], idletemp_f[OPMAX], tach->idle_cold(), tach->idle_hot()));
        // ezread.squintf(" si:%lf pc:%lf\n", idle_si[OUT], idle_pc);
    }
  private:
    void cruise_adjust(int joydir) {
        if (joydir == JOY_UP) ctrlratio = (hotrc->pc[VERT][FILT] - hotrc->pc[VERT][DBTOP]) / (hotrc->pc[VERT][OPMAX] - hotrc->pc[VERT][DBTOP]);
        else ctrlratio = (hotrc->pc[VERT][FILT] - hotrc->pc[VERT][DBBOT]) / (hotrc->pc[VERT][OPMIN] - hotrc->pc[VERT][DBBOT]);
        if (cruise_adjust_scheme == TriggerHold) {
            if (cruise_adjusting) throttle_target_pc += joydir * ctrlratio * cruise_delta_max_pc_per_s * cruiseDeltaTimer.elapsed() / 1000000.0;
            cruiseDeltaTimer.reset(); 
        }
        else if (cruise_adjust_scheme == TriggerPull && std::abs(hotrc->pc[VERT][FILT]) >= cruise_ctrl_extent_pc) {  // to avoid the adjustments following the trigger back to center when released
            if (!cruise_adjusting) adjustpoint = throttle_target_pc;  // When beginning adjustment, save current throttle pulse value to use as adjustment endpoint
            throttle_target_pc = adjustpoint + ctrlratio * cruise_angle_attenuator * (((joydir == JOY_UP) ? 100.0 : 0.0) - adjustpoint);
            cruise_ctrl_extent_pc = std::abs(hotrc->pc[VERT][FILT]);
        }
        else if (cruise_adjust_scheme == SuspendFly) {
            if (!cruise_adjusting) adjustpoint = (cruise_pid_enabled) ? rpm_to_pc(tach->val()) : pc[OUT];
            throttle_target_pc = adjustpoint + ctrlratio * (((joydir == JOY_UP) ? pc[GOVERN] : idle_pc) - adjustpoint);
        }
        if (cruise_pid_enabled) cruisepid.set_target(speedo->val());  // while adjusting, constantly update cruise pid target to whatever the current speed is
        cruise_adjusting = true;
    }
    void cruise_logic() {
        int joydir = hotrc->joydir(VERT);
        if ((joydir == JOY_UP || (joydir == JOY_DN && cruise_speed_lowerable)) && cruise_trigger_released) {  // adjustments disabled until trigger has been to center at least once since going to cruise mode
            cruise_adjust(joydir);
            return;
        }
        if (joydir != JOY_CENT) return;  // if trigger is being pulled under any other conditions we do nothing. below assume trigger is released
        cruise_adjusting = false;
        cruise_trigger_released = true;
        cruise_ctrl_extent_pc = hotrc->pc[VERT][CENT];  // After an adjustment, need this to prevent setpoint from following the trigger back to center as you release it
        if (cruise_pid_enabled) throttle_target_pc = cruisepid.compute();  // if cruise is using pid, it's only active when trigger is released
    }
    float rate_limiter(float val) {  // give it where you would want the throttle (%), it gives you back where you're allowed to put it, respecting max angular velocity
        float max_change = (float)throttleRateTimer.elapsed() * max_throttle_angular_velocity_pcps / 1000000.0;
        throttleRateTimer.reset();  // ezread.squintf(" new:%lf pc0:%lf mx:%lf", throttle_target_pc, pc[OUT], max_change);
        return constrain(val, pc[OUT] - max_change, pc[OUT] + max_change);  // ezread.squintf(" tgt:%lf pc1:%lf\n", throttle_target_pc, pc[OUT]);
    }
    void set_output() {
        // ezread.squintf("mode");
        if (motormode == Idle) throttle_target_pc = idle_pc;
        else if (motormode == Starting) throttle_target_pc = starting_pc;
        else if (motormode == Cruise) cruise_logic();  // cruise mode just got too big to be nested in this if-else clause
        else if (motormode == ParkMotor) throttle_target_pc = pc[PARKED];
        else if (motormode == OpenLoop || motormode == ActivePID) {
            if (hotrc->joydir() != JOY_UP) throttle_target_pc = idle_pc;  // If in deadband or being pushed down, we want idle
            else throttle_target_pc = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBTOP], hotrc->pc[VERT][OPMAX], idle_pc, pc[GOVERN]);  // actuators still respond even w/ engine turned off
        }
        else if (motormode == Calibrate) {
            cal_gasmode = true;
            pc[OUT] = out_si_to_pc(map(pot->val(), pot->opmin(), pot->opmax(), si[ABSMIN], si[ABSMAX]));  // gas_ccw_max_us, gas_cw_min_us
            return;  // cal mode sets the output directly, skipping the post processing below
        }  // ezread.squintf(":%d tgt:%lf pk:%lf idl:%lf\n", motormode, throttle_target_pc, pc[PARKED], idle_pc);
        float new_out;
        throttle_target_pc = constrain(throttle_target_pc, pc[PARKED], pc[OPMAX]);
        if (motormode == ActivePID) {
            pid.set_target(pc_to_rpm(throttle_target_pc));
            new_out = pid.compute();
        }
        else new_out = throttle_target_pc;  // ezread.squintf(" ela:%ld pcps:%lf", throttleRateTimer.elapsed(), max_throttle_angular_velocity_pcps);
        pc[OUT] = rate_limiter(new_out);
    }
    void postprocessing() {
        if (motormode == Calibrate) pc[OUT] = constrain(pc[OUT], pc[ABSMIN], pc[ABSMAX]);
        else if (motormode == ParkMotor || motormode == Halt) pc[OUT] = constrain(pc[OUT], pc[PARKED], pc[GOVERN]);
        else pc[OUT] = constrain(pc[OUT], idle_pc, pc[GOVERN]);
        if (motormode == ActivePID) pid.set_output(pc[OUT]);  // feed possibly-modified output value back into pid
        if ((motormode == Cruise) && cruise_pid_enabled) cruisepid.set_output(throttle_target_pc);  // feed possibly-modified output value back into pid
    }
  public:
    void setmode(int _mode) {
        if (!pid_enabled && _mode == ActivePID) _mode = OpenLoop;
        if (_mode == motormode) return;
        cal_gasmode = false;
        throttleRateTimer.reset();
        if (_mode == Cruise) {
            cruisepid.set_target(speedo->val());  // set pid loop speed target to current speed  (for SuspendFly mode)
            pid.set_target(tach->val());  // initialize pid output (rpm target) to current rpm  (for SuspendFly mode)
            throttle_target_pc = pc[OUT];  //  set target throttle angle to current throttle angle  (for TriggerPull/TriggerHold modes)
            cruise_adjusting = cruise_trigger_released = false;  // in case trigger is being pulled as cruise mode is entered, the ability to adjust is only unlocked after the trigger is subsequently released to the center
        }
        if (_mode == Calibrate) {
            float temp = pot->mapToRange(0.0, 180.0);
            if (temp < si[PARKED] || temp > si[OPMAX]) return;  // do not change to cal mode if attempting to enter while pot is out of range
        }
        motormode = _mode;
    }
    void update_ctrl_config(int new_pid_ena=-5) {   // run w/o arguments each loop to enforce configuration limitations, or call with an argument to enable/disable pid and update. do not change pid_enabled anywhere else!
        if (new_pid_ena != -5) pid_enabled = (bool)new_pid_ena;     // receive new pid enable setting (ON/OFF) if given
        if (pid_enabled != pid_ena_last) {
            if (pid_enabled) {
                cruisepid.set_limits(tach->idle_ptr(), tach->opmax_ptr());  // switch cruise pid parameters to feed valid throttle angle values to openloop gas
                cruisepid.set_tunings(cruise_pidgas_kp, cruise_pidgas_ki, cruise_pidgas_kd);
            }
            else {
                cruisepid.set_limits(&pc[OPMIN], &pc[OPMAX]);  // switch cruise pid parameters to feed valid rpm targets to gas pid
                cruisepid.set_tunings(cruise_opengas_kp, cruise_opengas_ki, cruise_opengas_kd);
                setmode(motormode);  // ensure current motor mode is consistent with configs set here
            }
            ezread.squintf("throttle pid %s\n", pid_enabled ? "enabled" : "disabled");
        }
        pid_ena_last = pid_enabled;
    }
    void update_cruise_ctrl_config(int new_pid_ena=-5) {  // pass in OFF or ON (for compatibility with brake function)
        if (new_pid_ena != -5) cruise_pid_enabled = (bool)new_pid_ena;     // otherwise receive new pid enable setting (ON/OFF) if given
        if (cruise_pid_enabled != cruise_pid_ena_last) ezread.squintf("cruise pid %s\n", cruise_pid_enabled ? "enabled" : "disabled");
        cruise_pid_ena_last = cruise_pid_enabled;
    }
    void set_cruise_scheme(int newscheme) {
        // if (cruise_pid_enabled) cruise_adjust_scheme = SuspendFly;  else // in pid mode cruise must use SuspendFly adjustment scheme (not sure if this restriction is necessary?)
        cruise_adjust_scheme = newscheme;  // otherwise anything goes
        ezread.squintf("cruise using %s adj scheme\n", cruiseschemecard[cruise_adjust_scheme].c_str());
    }
    int parked() {
        return (std::abs(out_pc_to_si(pc[OUT]) - si[PARKED]) < 1);
    }
    void update() {
        if (pid_timer.expireset()) {
            update_idlespeed();                // Step 1 : do any idle speed management needed          
            update_ctrl_config();
            set_output();                      // Step 2 : determine motor output value. updates throttle target from idle control or cruise mode pid, if applicable (on the same timer as gas pid). allows idle control to mess with tach_target if necessary, or otherwise step in to prevent car from stalling
            postprocessing();                // Step 3 : fix output to ensure it's in range
            us[OUT] = out_pc_to_us(pc[OUT]);   // Step 4 : convert motor value to pulsewidth time
            deg[OUT] = out_pc_to_si(pc[OUT]);
            // ezread.squintf("out pc:%lf us:%lf\n", pc[OUT], us[OUT]);
            write_motor();                     // Step 5 : write to servo
        }
    }
    float idlehot() {
        if (pid_enabled) return tach->idle_hot();
        else return idle_si[OPMIN];
    }
    float idlecold() {
        if (pid_enabled) return tach->idle_cold();
        else return idle_si[OPMAX];
    }
    void set_idlehot(float newidlehot) {
        if (pid_enabled) tach->set_idlehot(constrain(newidlehot, tach->opmin(), tach->idle_cold() - 1.0));
        else idle_si[OPMIN] = constrain(newidlehot, idle_si[ABSMIN], idle_si[OPMAX] - 1.0);
    }
    void set_idlecold(float newidlecold) {
        if (pid_enabled) tach->set_idlecold(constrain(newidlecold, tach->idle_hot() + 1.0, tach->opmax()));
        else idle_si[OPMAX] = constrain(newidlecold, idle_si[OPMIN] + 1.0, idle_si[ABSMAX]);
    }
    void set_temphot(float newtemphot) { idletemp_f[OPMAX] = constrain(newtemphot, idletemp_f[OPMIN] + 1.0, idletemp_f[ABSMAX]); }
    void set_tempcold(float newtempcold) { idletemp_f[OPMIN] = constrain(newtempcold, idletemp_f[ABSMIN], idletemp_f[OPMAX] - 1.0); }
};
class ChokeControl : public ServoMotor {
  private:
};
// Brake uses two PID loops: one based on pressure (accurate for high brake pressures), and one based on position (accurate when pedal is more released)
// Note as pedal is pressed down, position decreases as pressure increases. PID output is percent motor power.
class BrakeControl : public JagMotor {
  private:
    BrakePositionSensor* brkpos;
    PressureSensor* pressure;
    ThrottleControl* throttle;
    TemperatureSensorManager* tempsens;
    float brakemotor_duty_spec_pc = 25.0;  // In order to not exceed spec and overheat the actuator, limit brake presses when under pressure and adding pressure
    float press_kp = 0.142;        // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float press_ki = 0.04;        // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float press_kd = 0.000;        // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    float posn_kp = 42.0;          // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float posn_ki = 5.5;         // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float posn_kd = 22.0;         // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    static constexpr int pid_timeout = 40000;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
    float pres_out, posn_out, pc_out_last, posn_last, pres_last;
    int dominantsens_last = PositionFB, preforce_request;    // float posn_inflect, pres_inflect, pc_inflect;
    float heat_math_offset, motor_heat_min = 75.0, motor_heat_max = 200.0;
    Timer stopcar_timer{10000000}, interval_timer{1000000}, motor_park_timer{4000000}, motorheat_timer{500000}, blindaction_timer{3000000};
    bool stopped_last = false;
    bool feedback_enabled[NumBrakeSens];
    void set_dominant_sensor(float _hybrid_ratio) {
        if (feedback == PressureFB || feedback == PositionFB) dominantsens = feedback;
        else if (std::isnan(_hybrid_ratio)) return;
        else dominantsens = (int)(_hybrid_ratio + 0.5) ? PressureFB : PositionFB;  // round to 0 or 1, the indices of posn and pres respectively
        if (dominantsens == PositionFB) pid_dom = &(pids[PositionFB]);
        else pid_dom = &(pids[PressureFB]);
        posn_pid_active = (dominantsens == PositionFB);  // for display
    }
    float pressure_pc_to_si(float pc) {
        return map(pc, 0.0, 100.0, pressure->opmin(), pressure->opmax());
    }
  public:
    using JagMotor::JagMotor;
    bool pid_enabled = true, pid_ena_last = true, enforce_positional_limits = true, no_feedback = false;    // default for use of pid allowed
    int feedback = HybridFB, feedback_last = HybridFB;  // this is the default for sensors to use as feedback
    int dominantsens, motormode = Halt, oldmode = Halt;  // not tunable
    int openloop_mode = AutoRelHoldable;  // if true, when in openloop the brakes release when trigger released. otherwise, control with thrigger using halfway point scheme
    bool brake_tempsens_exists = false, posn_pid_active = (dominantsens == PositionFB);
    QPID pids[NumBrakeSens];  // brake changes from pressure target to position target as pressures decrease, and vice versa
    QPID* pid_dom = &(pids[PositionFB]);  // AnalogSensor sensed[2];
    float brake_pid_trans_threshold_lo = 0.25;  // tunable. At what fraction of full brake pressure will motor control begin to transition from posn control to pressure control
    float brake_pid_trans_threshold_hi = 0.50;  // tunable. At what fraction of full brake pressure will motor control be fully transitioned to pressure control
    bool autostopping = false, autoholding = false, reverse = false;
    float target_si[NumBrakeSens];  // this value is the posn and pressure (and hybrid combined) target settings, or fed into pid to calculate setting if pid enabled, in si units appropriate to each sensor
    float hybrid_math_offset, hybrid_math_coeff, hybrid_sens_ratio, hybrid_sens_ratio_pc, target_pc, pid_err_pc;
    float combined_read_pc, hybrid_out_ratio = 1.0, hybrid_out_ratio_pc = 100.0;
    float motor_heat = NAN, motor_heatloss_rate = 3.0, motor_max_loaded_heatup_rate = 1.5, motor_max_unloaded_heatup_rate = 0.3;  // deg F per timer timeout
    float open_loop_attenuation_pc = 75.0, thresh_loop_attenuation_pc = 65.0, thresh_loop_hysteresis_pc = 2.0;  // when driving blind i.e. w/o any sensors, what's the max motor speed as a percent
    float max_out_change_pcps = 800.0;  // what percent of motor overall range should be the most its value can change within 1 second?
    void derive() {  // to-do below: need stop/hold values for posn only operation!
        JagMotor::derive();
        // pc[MARGIN] = 100.0 * pressure->margin() / (pressure->opmax() - pressure->opmin());
        hybrid_math_offset = 0.5 * (brake_pid_trans_threshold_hi + brake_pid_trans_threshold_lo);  // pre-do some of the math to speed up hybrid calculations
        hybrid_math_coeff = M_PI / (brake_pid_trans_threshold_hi - brake_pid_trans_threshold_lo);  // pre-do some of the math to speed up hybrid calculations
    }
    bool detect_tempsens() {
        float trytemp = tempsens->val(loc::BRAKE);
        brake_tempsens_exists = !std::isnan(trytemp);
        ezread.squintf(" using heat %s sensor\n", brake_tempsens_exists ? "readings from detected" : "estimates in lieu of");
        return brake_tempsens_exists;
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt, PressureSensor* _pressure, BrakePositionSensor* _brkpos, ThrottleControl* _throttle, TemperatureSensorManager* _tempsens) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        ezread.squintf("Brake pid %s, feedback: %s\n", pid_enabled ? "enabled" : "disabled",  brakefeedbackcard[feedback].c_str());
        JagMotor::setup(_hotrc, _speedo, _batt);
        pressure = _pressure;  brkpos = _brkpos;  throttle = _throttle;  throttle = _throttle;  tempsens = _tempsens; 
        // duty_fwd_pc = brakemotor_duty_spec_pc;
        pres_last = pressure->val();
        posn_last = brkpos->val();
        // set_dominant_sensor(brake_default_pid);
        detect_tempsens();
        if (!std::isnan(tempsens->val(loc::AMBIENT))) motor_heat_min = tempsens->val(loc::AMBIENT);
        derive();
        update_ctrl_config();
        pids[PressureFB].init(pressure->ptr(), &(pc[OPMIN]), &(pc[OPMAX]), press_kp, press_ki, press_kd, QPID::pmod::onerr,
            QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout, QPID::ctrl::manual, QPID::centmod::off);  // QPID::centmod::on, pc[STOP]);
        pids[PositionFB].init(brkpos->ptr(), &(pc[OPMIN]), &(pc[OPMAX]), posn_kp, posn_ki, posn_kd, QPID::pmod::onerr, 
            QPID::dmod::onerr, QPID::awmod::off, QPID::cdir::reverse, pid_timeout, QPID::ctrl::manual, QPID::centmod::off);  // QPID::centmod::on, pc[STOP]);
        // pids[PressureFB].set_iaw_cond_thresh(0.25);  // set the fraction of the output range within which integration works at all
        // pids[PositionFB].set_iaw_cond_thresh(0.25);
        max_out_change_rate_pcps = 200.0;  // LAE actuator stutters and stalls when changing direction suddenly
    }
  private:
    void update_motorheat() {  // i am probably going to scrap all this nonsense and just put another temp sensor on the motor
        float added_heat, nowtemp, out_ratio;
        if (motorheat_timer.expireset()) {
            out_ratio = pc[OUT] / 100.0;
            if (brake_tempsens_exists) motor_heat = tempsens->val(loc::BRAKE);
            else {
                nowtemp = tempsens->val(loc::AMBIENT);
                if (std::isnan(motor_heat) && !std::isnan(nowtemp)) motor_heat = nowtemp;  // ezread.squintf("Actively forecasting brake motor heat generation\n");
                else {
                    if (std::isnan(nowtemp)) added_heat = motor_heatloss_rate / -4.0;
                    else {
                        motor_heat_min = nowtemp;
                        if (motor_heat > nowtemp) added_heat = -motor_heatloss_rate * (motor_heat - nowtemp) / nowtemp;
                    }
                    if (pc[OUT] <= brakemotor_duty_spec_pc + pc[MARGIN]) added_heat += map(pc[OUT], 0.0, -100.0, 0.0, motor_max_unloaded_heatup_rate);
                    else if (pc[OUT] > brakemotor_duty_spec_pc - pc[MARGIN]) added_heat += map(pc[OUT], brakemotor_duty_spec_pc, 100.0, 0.0, motor_max_loaded_heatup_rate);
                    motor_heat += added_heat;
                }
            }
            motor_heat = constrain(motor_heat, motor_heat_min, motor_heat_max);
        }
        // that's great to have some idea whether the motor is hot. but we need to take some actions in response
    }
    // the brake can be controlled by a pid or open loop. either way it will use all enabled sensors as     // the influence on the final output from the pressure and position pids is weighted based on the pressure reading
    // as the brakes are pressurized beyond X psi, use pressure reading, otherwise use position as feedback, because
    // near either extreme only one of the two sensors is accurate. So at lower psi, control is 100% position-based, fade to 100% pressure-based at mid/high pressures
    // "dominant" PID means the PID loop (pressure or position) that has the majority influence over the motor
    float calc_hybrid_ratio(float pressure_val) {  // returns pressure influence as a ratio (from 0.0 to 1.0) for a given pressure level in percent 
        if (feedback == NoneFB) return NAN;             // this should not happen, maybe print an error message
        if (feedback == PressureFB) return 1.0;
        if (feedback == PositionFB) return 0.0;
        float pressure_ratio = pressure->pc() / 100.0;  // (pressure_val - pressure->opmin()) / (pressure->opmax() - pressure->opmin());  // calculate ratio of output to range
        if (pressure_ratio >= brake_pid_trans_threshold_hi) return 1.0;  // at pressure above hi threshold, pressure has 100% influence
        if (pressure_ratio <= brake_pid_trans_threshold_lo) return 0.0;  // at pressure below lo threshold, position has 100% influence
        return 0.5 + 0.5 * sin(hybrid_math_coeff * (pressure_ratio - hybrid_math_offset));  // in between we make a steep but smooth transition during which both have some influence
    }
    void set_hybrid_ratio() {
        float temp = calc_hybrid_ratio(pressure->val());
        if (!isnan(temp)) {
            hybrid_out_ratio = temp;  // calculate pressure vs. position multiplier based on the sensed values
            hybrid_out_ratio_pc = 100.0 * hybrid_out_ratio;  // for display
        }
        else hybrid_out_ratio = hybrid_out_ratio_pc = NAN;
        set_dominant_sensor(hybrid_out_ratio);  // round to 0 (posn pid) or 1 (pressure pid). this is for idiot light display
    }
    float get_hybrid_brake_pc(float _given_pres, float _given_posn) {  // uses current hybrid_ratio to calculate a combined brake percentage value from given pres and posn values
        return hybrid_out_ratio * _given_pres + (1.0 - hybrid_out_ratio) * _given_posn;  // combine pid outputs weighted by the multiplier
    }
    void set_target(float targ_pc) {  // sets brake target percent. if hybrid, will set pressue and posn targets per current hybrid ratio, and set both pid targets in case pids are used
        target_pc = targ_pc;
        if (feedback == NoneFB) return;  // target_pc is overall, (or desired or combined ?) value is used if running openloop
        target_si[PressureFB] = pressure->from_pc(target_pc);  // feed this into pid if enabled, (otherwise could use as a motor cutoff threshold? this mode not implemented)
        target_si[PositionFB] = brkpos->from_pc(target_pc);  // (saved into variables for display)
        for (int mypid=PositionFB; mypid<=PressureFB; mypid++) pids[mypid].set_target(target_si[mypid]);  // feed target vals to pid loops. this is harmless if pids disabled, it will have no effect
    }
    void read_sensors() {  // calculates and saves combined brake value percent, based on readings of the feedback sensors. harmless to call even if running openloop
        if (feedback == PositionFB) combined_read_pc = brkpos->pc();
        else if (feedback == PressureFB) combined_read_pc = pressure->pc();
        else if (feedback == HybridFB) combined_read_pc = get_hybrid_brake_pc(pressure->pc(), brkpos->pc());
        else combined_read_pc = NAN;
    }
    float calc_thresh_loop_out() {  // scheme to drive using sensors but without pid, just uses target as a threshold value, always driving motor at a fixed speed toward it
        float err = (combined_read_pc - target_pc);
        if (std::abs(err) < thresh_loop_hysteresis_pc) return pc[STOP];
        if (err > 0.0) return thresh_loop_attenuation_pc * pc[OPMIN];
        return thresh_loop_attenuation_pc * pc[OPMAX];
    }
    float calc_loop_out() {  // returns motor output percent calculated based on current target_pc or target[] values, in a way consistent w/ current config
        if (motormode == ThreshLoop) return calc_thresh_loop_out();
        else if (motormode == ActivePID) return get_hybrid_brake_pc(pids[PressureFB].compute(), pids[PositionFB].compute());  // combine pid outputs weighted by the multiplier
        else return pc[STOP];  // this should not happen, maybe print an error message
    }
    float calc_open_out() {
        // this line releases brake whenever trigger is released, and presses proportional to trigger push. must hold trigger steady to stop brakes where they are
        if (openloop_mode == AutoRelHoldable) {
            if (hotrc->pc[VERT][FILT] <= hotrc->pc[VERT][OPMIN] + hotrc->pc[VERT][MARGIN])
                set_target(open_loop_attenuation_pc * pc[OPMAX] / 100.0);
            else if (hotrc->joydir() == JOY_DN) set_target(pc[STOP]);
            else set_target(pc[OPMIN]);
        }
        else if (openloop_mode == AutoRelease) {
            if (hotrc->joydir() == JOY_DN) set_target(open_loop_attenuation_pc * map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBBOT], hotrc->pc[VERT][OPMIN], pc[STOP], pc[OPMAX]) / 100.0);
            else set_target(pc[OPMIN]);
        }
        else {  // if openloopmode == Median
            if (hotrc->joydir() == JOY_DN) set_target(open_loop_attenuation_pc * map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBBOT], hotrc->pc[VERT][OPMIN], pc[OPMIN], pc[OPMAX]) / 100.0);
            else set_target(pc[STOP]);
        }
        return target_pc;  // this is openloop (blind trigger) control scheme
    }
    void carstop(bool panic_support=true) {  // autogenerates motor target values with the goal of stopping the car
        bool panic = panic_support && panicstop;
        bool stopped_now = speedo->stopped();
        if (stopped_last && !stopped_now) stopcar_timer.reset();
        stopped_last = stopped_now;
        if (feedback == NoneFB) {  // to panic stop without any feedback, we just push as hard as we can for a couple seconds then stop
            autostopping = !stopped_now && panic && !blindaction_timer.expired();
            if (!autostopping) return;
            pc[OUT] = pc[OPMAX];  // note this is dangerous in that it could mechanically break stuff. if running open loop avoid panicstops
            return;
        }
        autostopping = !stopped_now && !stopcar_timer.expired();
        if (!autostopping) return;
        else if (interval_timer.expireset()) set_target(std::min(100.0f, target_pc + panic ? pressure->panic_increment_pc() : pressure->hold_increment_pc()));
        else set_target(std::max(target_pc, panic ? pressure->panic_initial_pc() : pressure->hold_initial_pc()));
        pc[OUT] = calc_loop_out();
    }
    bool goto_fixed_point(float tgt_point, bool at_position) {  // goes to a fixed position (hopefully) or pressure (if posn is unavailable) then stops.  useful for parking and releasing modes
        // active_sensor = (enabled_sensor == PressureFB) ? PressureFB : PositionFB;  // use posn sensor for this unless we are specifically forcing pressure only
        bool in_progress;
        if (feedback == NoneFB) {  // if running w/o feedback, let's blindly release the brake for a few seconds then halt it
            in_progress = (!blindaction_timer.expired());
            if (in_progress) pc[OUT] = pc[OPMIN];
            else setmode(Halt, false); 
            return in_progress;
        }
        in_progress = (!at_position && !motor_park_timer.expired());
        if (in_progress) set_target(tgt_point);
        else setmode(Halt, false);
        pc[OUT] = calc_loop_out();
        return in_progress;
    }
    void set_output() {  // sets pc[OUT] in whatever way is right for the current motor mode
        if (motormode == OpenLoop) {  // in open loop, push trigger out less than 1/2way, motor extends (release brake), or more than halfway to retract (push brake), w/ speed proportional to distance from halfway point 
            pc[OUT] = calc_open_out();
            return;  // open loop should skip calculating hybrid ratio, so just return
        }
        set_hybrid_ratio();
        read_sensors();

        if (motormode == AutoHold) {  // autohold: apply initial moderate brake pressure, and incrementally more if car is moving. If car stops, then stop motor but continue to monitor car speed indefinitely, adding brake as needed
            carstop(false);
            autoholding = !autostopping && (pressure->val() >= pressure->hold_initial - pressure->margin());  // this needs to be tested  // if (!speedo->stopped()) {            
            // ezread.squintf("as:%d ah:%d f:%lf, h:%lf, m:%lf\n", autostopping, autoholding, pressure->val(), pressure->hold_initial_psi, pressure->margin_psi);
            if (autoholding) pc[OUT] = pc[STOP];
            else if (!autostopping) {
                set_target(std::max(target_pc, pressure->hold_initial_pc()));
                pc[OUT] = calc_loop_out();
            }
        }
        else if (motormode == AutoStop) {  // autostop: if car is moving, apply initial pressure plus incremental pressure every few seconds until it stops or timeout expires, then stop motor and cancel mode
            throttle->setmode(Idle);  // Stop pushing the gas, will help us stop the car better
            carstop(true);
            if (!autostopping) setmode(Halt, false);  // After AutoStop mode stops the car or times out, then stop driving the motor
        }
        else if (motormode == Halt) {
            pc[OUT] = pc[STOP];
        }
        else if (motormode == Calibrate) {
            cal_brakemode = true;  // for sunmode state machine
            int _joydir = hotrc->joydir(); 
            if (_joydir == JOY_UP) pc[OUT] = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBTOP], hotrc->pc[VERT][OPMAX], pc[STOP], pc[OPMAX]);
            else if (_joydir == JOY_DN) pc[OUT] = map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][OPMIN], hotrc->pc[VERT][DBBOT], pc[OPMIN], pc[STOP]);
            else pc[OUT] = pc[STOP];
        }
        else if (motormode == Release) {
            releasing = goto_fixed_point(brkpos->zeropoint_pc(), released());
        }
        else if (motormode == ParkMotor) {
            parking = goto_fixed_point(0.0, parked());
        }
        else if ((motormode == ThreshLoop) || (motormode == ActivePID)) {
            if (hotrc->joydir(VERT) != JOY_DN) set_target(pc[STOP]);  // let off the brake
            else set_target(map(hotrc->pc[VERT][FILT], hotrc->pc[VERT][DBBOT], hotrc->pc[VERT][OPMIN], 0.0, 100.0));  // If we are trying to brake, scale joystick value to determine brake pressure setpoint
            pc[OUT] = calc_loop_out();
        }
    }
    void postprocessing() {  // keep within the operational range, or to full absolute range if calibrating (caution don't break anything!)
        if (motormode == Calibrate) pc[OUT] = constrain(pc[OUT], pc[ABSMIN], pc[ABSMAX]);
        else if (enforce_positional_limits                                                 // IF we're not exceed actuator position limits
          && ((pc[OUT] < pc[STOP] && brkpos->val() > brkpos->opmax() - brkpos->margin())   //   AND ( we're trying to extend while already at the extension limit
          || (pc[OUT] > pc[STOP] && brkpos->val() < brkpos->opmin() + brkpos->margin())))  //     OR trying to retract while already at the retraction limit )
            pc[OUT] = pc[STOP];                                                            // THEN stop the motor
        else pc[OUT] = constrain(pc[OUT], pc[OPMIN], pc[OPMAX]);                     // constrain motor value to operational range
        changerate_limiter();
        if (std::abs(pc[OUT]) < 0.01) pc[OUT] = 0.0;                                 // prevent stupidly small values which i was seeing
        for (int p = PositionFB; p <= PressureFB; p++) pids[p].set_output(pc[OUT]);  // feed the final value back into the pids
    }
  public:
    void setmode(int _mode, bool external_request=true) {  // external_request set to false when calling from inside the class (to remember what the outside world wants the mode to be)   
        bool mode_forced = false;                                                 // note whether requested mode was overriden
        int save_mode_request = _mode;
        if (feedback == NoneFB) {                                                 // if there is no feedback
            if (_mode == ActivePID || _mode == ThreshLoop) { 
                // preforce_drivemode = _mode;                                       // remember our requested drive mode before demoting to openloop mode, so we can restore it if settings changed
                mode_forced = true;
                _mode = OpenLoop;      // can't use loops, drop to openloop instead
            }
            else if (_mode == AutoHold || (_mode == AutoStop && !panicstop)) {    // todo: rethink these scenarios 
                Serial.print("Warn: autobrake unavailable in openloop\n");
                return;  // keep current mode
            }
            else if (_mode == AutoStop) Serial.print("Warn: performing blind panic stop maneuver\n");
            else if (_mode == Release) {
                _mode = Halt;
                mode_forced = true;
            }
            // else if (_mode == Release || _mode == ParkMotor) _mode = Halt;   // in openloop we lose some safeties, only use if necessary 
        }
        else if (!pid_enabled) {                         // if we have feedback but pid is disabled in config
            if (_mode == ActivePID) _mode = ThreshLoop;  // drop to simple threshold-based loop scheme
        }
        if (_mode == motormode) return;
        if (external_request) preforce_request = save_mode_request;  // remember what the outside world actually asked for, in case we override it but later wish to go back
        autostopping = autoholding = cal_brakemode = parking = releasing = false;        
        interval_timer.reset();
        stopcar_timer.reset();
        blindaction_timer.reset();
        motor_park_timer.reset();
        motormode = _mode;
        // ezread.squintf("brakemode: %d\n",motormode);
    }
    void update_ctrl_config(int new_pid_ena=-1, int new_feedback=-1, int new_openloop_mode=-1) {   // run w/o arguments each loop to enforce configuration limitations, or call with argument(s) to change config and update. do not change feedback or pid_enabled anywhere else!
        if (new_feedback >= 0 && new_feedback <= NumBrakeFB) feedback = new_feedback;                 // receive new feedback sensors setting (NoneFB/PressureFB/PositionFB/_Hybrid) if given
        if (feedback == NoneFB) pid_enabled = false;                      // if there are no feedback sensors then we must force disable pid
        else if (new_pid_ena == 0 || new_pid_ena == 1) pid_enabled = (bool)new_pid_ena;     // otherwise receive new pid enable setting (ON/OFF) if given
        feedback_enabled[PositionFB] = ((feedback == PositionFB) || (feedback == HybridFB));  // set sensor enable consistent with feedback config
        feedback_enabled[PressureFB] = ((feedback == PressureFB) || (feedback == HybridFB));  // set sensor enable consistent with feedback config
        no_feedback = (feedback == NoneFB);                              // for idiot light display
        if ((feedback != feedback_last) || (pid_enabled != pid_ena_last)) {
            setmode(preforce_request, false);                            // ensure current motor mode is consistent with configs set here
            derive();  // on change need to recalculate some values
            ezread.squintf("brake pid %s feedback: %s\n", pid_enabled ? "enabled" : "disabled", brakefeedbackcard[feedback].c_str());        
        }
        if (new_openloop_mode >= 0 && new_openloop_mode <= NumOpenLoopModes) openloop_mode = new_openloop_mode;
        feedback_last = feedback;
        pid_ena_last = pid_enabled;
    }
    void update() {  // Brakes - Determine motor output and write it to motor
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            update_motorheat();                // Step 1 : Continuously estimate motor heat to prevent exceeding duty limitation requirement
            update_ctrl_config();              // Step 1.5 : Catch any change in configuration affecting motor mode
            set_output();                      // Step 2 : Determine motor percent value
            postprocessing();                // Step 3 : Fix motor pc value if it's out of range or threatening to exceed positional limits
            us[OUT] = out_pc_to_us(pc[OUT]);   // Step 4 : Convert motor percent value to pulse width for motor, and to volts for display
            volt[OUT] = out_pc_to_si(pc[OUT]);
            write_motor();  // Step 5 : Write to motor
        }
    }
    bool parked() {
        if (feedback_enabled[PositionFB]) return brkpos->parked();
        if (feedback_enabled[PressureFB]) return pressure->released();  // pressure doesn't have a parked() function yet
        return false;  // really without sensors we have no idea if we're parked. Print an error message
    }
    bool released() {
        if (feedback_enabled[PositionFB]) return brkpos->released();
        if (feedback_enabled[PressureFB]) return pressure->released();  // pressure doesn't have a parked() function yet
        return false;  // really without sensors we have no idea if we're released. Print an error message
    }
    float sensmin() { return (dominantsens == PressureFB) ? pressure->opmin() : brkpos->opmin(); }
    float sensmax() { return (dominantsens == PositionFB) ? pressure->opmax() : brkpos->opmax(); }
    float motorheat() { return motor_heat; }
    float motorheatmin() { return motor_heat_min; }
    float motorheatmax() { return motor_heat_max; }
};
class SteeringControl : public JagMotor {
  public:
    using JagMotor::JagMotor;
    int motormode = Halt, oldmode = Halt;
    float steer_safe_pc = 72.0;  // this percent is taken off full steering power when driving full speed (linearly applied)
    bool reverse = false;
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        max_out_change_rate_pcps = 350.0;
        ezread.squintf("Steering motor..\n");
        JagMotor::setup(_hotrc, _speedo, _batt);
    }
    void update() {
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            set_output();                     // Step 1 : Determine motor percent value
            postprocessing();               // Step 2 : Fix motor pc value if it's out of range
            us[OUT] = out_pc_to_us(pc[OUT]);  // Step 3 : Convert motor percent value to pulse width for motor, and to volts for display
            volt[OUT] = out_pc_to_si(pc[OUT]);
            write_motor();  // Step 4 : Write to motor
        }
    }
    void setmode(int _mode) { motormode = _mode; }
  private:
    void set_output() {
        if (motormode == Halt) pc[OUT] = pc[STOP];  // Stop the steering motor if in standby mode and standby is complete
        else if (motormode == OpenLoop) {
            int _joydir = hotrc->joydir(HORZ);
            if (_joydir == JOY_RT)
                pc[OUT] = map(hotrc->pc[HORZ][FILT], hotrc->pc[HORZ][DBTOP], hotrc->pc[HORZ][OPMAX], pc[STOP], steer_safe(pc[OPMAX]));  // if joy to the right of deadband
            else if (_joydir == JOY_LT)
                pc[OUT] = map(hotrc->pc[HORZ][FILT], hotrc->pc[HORZ][DBBOT], hotrc->pc[HORZ][OPMIN], pc[STOP], steer_safe(pc[OPMIN]));  // if joy to the left of deadband
            else
                pc[OUT] = pc[STOP];  // Stop the steering motor if inside the deadband
        }
    }
    void postprocessing() {
        changerate_limiter();
        pc[OUT] = constrain(pc[OUT], pc[OPMIN], pc[OPMAX]);
        if (std::abs(pc[OUT]) < 0.01) pc[OUT] = 0.0;
    }
    float steer_safe(float endpoint) {
        return pc[STOP] + (endpoint - pc[STOP]) * (1.0 - steer_safe_pc * speedo->val() / (100.0 * speedo->opmax()));
    }
};