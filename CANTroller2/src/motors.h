// motors.h : centralized wrapper classes for all braking, throttle, and steering activity, as holistic coordination between sensors, actuator, and intent is critical
#pragma once
#include <ESP32Servo.h>  // Eventually move Servos into new ServoPWM objects then remove this

// I stole this library and modified it heavily to our purposes - Soren
// QuickPID Library for Arduino - Version 3.1.9 by dlloydev https://github.com/Dlloydev/QuickPID
// Based on the Arduino PID_v1 Library. Licensed under the MIT License.
// some general pid information:
// https://www.youtube.com/watch?v=6OH-wOsVVjg&ab_channel=SiieeFPV
// https://www.youtube.com/watch?v=qKy98Cbcltw&ab_channel=Horizon4electronics
class QPID {
  public:
    // ctrl: sets the controller mode to manual (0), automatic (1) or timer (2) when the transition from manual to automatic or timer occurs, the controller is automatically initialized.
    // cdir: the PID will either be connected to a direct acting process (+output leads to +input) or a reverse acting process(+output leads to -input)
    // pmod: sets the computation method for the proportional term, to compute based either on error (default), on measurement, or the average of both
    // dmod: sets the computation method for the derivative term, to compute based either on error or on measurement (default)
    // awmode: sets the integral anti-windup mode to one of clamp, which clamps the output after adding integral and proportional (on measurement) terms,
    //   or cond (default), which provides some integral correction, prevents deep saturation and reduces overshoot. Option off disables anti-windup altogether
    enum class ctrl { manual, automatic, toggle };            // controller mode
    enum class cdir { reverse=-1, direct=1 };                 // controller direction
    enum class pmod { onerr, onmeas, onerrmeas };             // proportional mode
    enum class dmod { onerr, onmeas };                        // derivative mode
    enum class awmod { cond, clamp, off, round, roundcond };  // integral anti-windup mode
    enum class centmod { off, on, strict };                   // Allows a defined output zero point
  private:
    float dispkp = 0.0f, dispki = 0.0f, dispkd = 0.0f, _iaw_cond_thresh = 1.0f;  // set what fraction of output range beyond which applies integral anti-windup in cond mode
    float _pterm, _iterm, _dterm, _kp, _ki, _kd, _err, lasterr, lastin, lastout, _cent, _outsum, _target, _output;
    float *myin;     // Pointers to the input, output, and target variables. This  creates a
    float *_outmin;
    float *_outmax;
    float _out_changerate_ps = 0.0f;  // how much output is changing per sec (differential of output) 
    float _max_out_changerate_ps = 0.0f;  // set to impose limits on how fast the output can change. set to 0.0 to disable
    bool pause_diff;
    ctrl _mode = ctrl::manual;
    cdir _dir = cdir::direct;
    pmod _pmode = pmod::onerr;
    dmod _dmode = dmod::onmeas;
    awmod _awmode = awmod::cond;
    centmod _centmode = centmod::off;
    int _sampletime;  // time in us between each output calculation
    int64_t lasttime; // need 64 bit value to hold 54 bit esp hardware timer value (ensures no overflow after a few hours like micros())
  public:
    QPID() {}  // Default constructor
    QPID(float* a_in, float* a_min, float* a_max, float a_kp = 0.0f, float a_ki = 0.0f, float a_kd = 0.0f,
      pmod a_pmode = pmod::onerr, dmod a_dmode = dmod::onmeas, awmod a_awmode = awmod::cond, cdir a_dir = cdir::direct,
      int a_sampletime = 100000, ctrl a_mode = ctrl::manual, centmod a_centmode = centmod::off, float a_cent = NAN) {
        init(a_in, a_min, a_max, a_kp, a_ki, a_kd, a_pmode, a_dmode, a_awmode, a_dir, a_sampletime, a_mode, a_centmode, a_cent);
    }
    void init(float* a_in, float* a_min, float* a_max, float a_kp = 0.0f, float a_ki = 0.0f, float a_kd = 0.0f,
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
        lasttime = esp_timer_get_time() - _sampletime;
    }
    void init(float preload_output = NAN) {  // ensure a bumpless transfer from manual to automatic mode
        if (!std::isnan(preload_output)) _output = preload_output;
        _outsum = _output;
        lastin = *myin;
        _outsum = constrain(_outsum, *_outmin, *_outmax);
    }
    // This function should be called every time "void loop()" executes. The function will decide whether a new 
    // PID output needs to be computed. Returns true when the output is computed, false when nothing has been done.
    float compute() {
        int64_t now = esp_timer_get_time();
        int timechange = (int)(now - lasttime);
        if (_mode == ctrl::automatic && timechange < _sampletime) return _output;  // if class is handling the timing and this time was a nop

        float in = *myin;
        float din = in - lastin;
        if (_dir == cdir::reverse) din = -din;

        _err = _target - in;
        if (_dir == cdir::reverse) _err = -_err;
        float derr = _err - lasterr;

        float peterm = _kp * _err;
        float pmterm = _kp * din;
        if (_pmode == pmod::onerr) pmterm = 0.0f;
        else if (_pmode == pmod::onmeas) peterm = 0.0f;
        else { // onerrmeas
            peterm *= 0.5f;
            pmterm *= 0.5f;
        }
        _pterm = peterm - pmterm;  // used by getter function
        _iterm = _ki * _err;
        if (_dmode == dmod::onerr) _dterm = _kd * derr;
        else _dterm = -_kd * din; // onmeas

        if (_awmode == awmod::cond || _awmode == awmod::roundcond) {  // condition anti-windup (default)
            bool aw = false;
            float _itermout = (peterm - pmterm) + _ki * (_iterm + _err);
            if (_itermout > (*_outmax * _iaw_cond_thresh) && derr > 0.0f) aw = true;
            else if (_itermout < (*_outmin * _iaw_cond_thresh) && derr < 0.0f) aw = true;
            if (aw && _ki) _iterm = constrain(_itermout, -(*_outmax * _iaw_cond_thresh), *_outmax * _iaw_cond_thresh);
        }
        if ((_awmode == awmod::round || _awmode == awmod::roundcond) && _err < 0.001f && _err > -0.001f) {
            _err = 0.0f;
            if (_centmode == centmod::on || _centmode == centmod::strict) _outsum = _cent;     
        }
        if (_centmode == centmod::strict && _err * lasterr < 0.0f) _outsum = _cent;  // Recenters any old integral when error crosses zero
        
        if (pause_diff) _dterm = 0.0f;  // avoid differential effect on this cycle due to recent external setting of output
        pause_diff = false;

        _outsum += _iterm;  // by default, compute output as per PID_v1    // include integral amount and pmterm
        if (_awmode == awmod::off) _outsum -= pmterm;
        else _outsum = constrain(_outsum - pmterm, *_outmin, *_outmax);  // clamp
        
        lastout = _output;
        _output = constrain(_outsum + peterm + _dterm, *_outmin, *_outmax);  // include _dterm, clamp and drive output
        
        if (!iszero(_max_out_changerate_ps)) {  // unless rate limiter is bypassed by having value of 0.0
            float max_out_change = _max_out_changerate_ps * (float)timechange / 1000000.0f;  // calculate max allowed output value change since the last calculate
            _output = constrain(_output, lastout - max_out_change, lastout + max_out_change);  // constrain output to comply
        }
        _out_changerate_ps = !timechange ? NAN : std::abs(_output - lastout) * 1000000.0f / (float)timechange;  // for external query
        
        lasterr = _err;
        lastin = in;
        lasttime = now;
        return _output;
    }
    // set_tunings  This function allows the controller's dynamic performance to be adjusted. It's called 
    // automatically from the constructor, but tunings can also be adjusted on the fly during normal operation.
    // void set_tunings(float a_kp, float a_ki, float a_kd, pmod a_pmode = pmod::onerr, dmod a_dmode = dmod::onmeas, awmod a_awmode = awmod::cond) {
    void set_tunings(float a_kp, float a_ki, float a_kd, pmod a_pmode, dmod a_dmode, awmod a_awmode) {
        if (a_kp < 0.0f || a_ki < 0.0f || a_kd < 0.0f || !_sampletime) return;  // added divide by zero protection
        if (a_ki == 0.0f) _outsum = 0.0f;
        _pmode = a_pmode; _dmode = a_dmode; _awmode = a_awmode;
        if (std::abs(a_kp) < float_conversion_zero) a_kp = 0.0f;
        if (std::abs(a_ki) < float_conversion_zero) a_ki = 0.0f;
        if (std::abs(a_kd) < float_conversion_zero) a_kd = 0.0f;
        dispkp = a_kp; dispki = a_ki; dispkd = a_kd;
        float sampletime_sec = (float)_sampletime / 1000000.0f;
        _kp = a_kp;
        _ki = a_ki * sampletime_sec;
        _kd = a_kd / sampletime_sec;
    }
    void set_tunings(float a_kp, float a_ki, float a_kd) {  // set Tunings using the last remembered pmode, dmode and awmode settings.
        set_tunings(a_kp, a_ki, a_kd, _pmode, _dmode, _awmode);
    }
    void set_sampletime(int a_sampletime) {  // set_sampletime  Sets the period, in microseconds, at which the calculation is performed.
        if (a_sampletime > 0.0f && _sampletime) {  // added more divide by zero protection
            float ratio  = (float)a_sampletime / (float)_sampletime;
            _ki *= ratio;
            _kd /= ratio;
            _sampletime = a_sampletime;
        }
    }
    void set_mode(ctrl a_mode) {  // set control mode
        if (_mode == ctrl::manual && a_mode != ctrl::manual) { // just went from manual to automatic
            _mode = ctrl::automatic;
            init();
        }
        else if (_mode == ctrl::automatic && a_mode != ctrl::automatic) _mode = ctrl::manual;
    }
    void set_mode(int a_mode) { set_mode((ctrl)a_mode); }
    void reset() {  // Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.
        lasttime = esp_timer_get_time() - _sampletime;
        lastin = _outsum = _pterm = _iterm = _dterm = 0.0f;
    }
    void set_output(float a_output) {
        float oldout = _output;
        if (std::isnan(a_output)) return;
        a_output = constrain(a_output, *_outmin, *_outmax);
        if (!iszero(_output)) {
            float ratio = a_output / _output;
            _output *= ratio;
            _outsum *= ratio;
        }
        else _output = _outsum = a_output;
        if (std::abs(oldout - _output) > _dterm) pause_diff = true;  // will skip differential effect on net compute cycle if output changed significantly
    }
    void set_limits(float* a_min, float* a_max) {
        _outmin = a_min;
        _outmax = a_max;
        _output = constrain(_output, *_outmin, *_outmax);
        _outsum = constrain(_outsum, *_outmin, *_outmax);
    }
    void set_max_out_changerate(float a_max_rate) { _max_out_changerate_ps = a_max_rate; }  // to set value of imposed maximum output change rate (in output units per second). if 0.0 disables limits
    void set_kp(float a_kp) { set_tunings(std::max(0.0f, a_kp), dispki, dispkd, _pmode, _dmode, _awmode); }
    void set_ki(float a_ki) { set_tunings(dispkp, std::max(0.0f, a_ki), dispkd, _pmode, _dmode, _awmode); }
    void set_kd(float a_kd) { set_tunings(dispkp, dispki, std::max(0.0f, a_kd), _pmode, _dmode, _awmode); }
    void set_centmode(centmod a_centmode) { _centmode = a_centmode; }
    void set_centmode(int a_centmode) { _centmode = (centmod)a_centmode; }
    void set_cent(float a_cent) { if (*_outmin <= a_cent && *_outmax >= a_cent) _cent = a_cent; }
    void set_target(float a_target) { _target = a_target; }
    void set_iaw_cond_thresh(float thresh) { _iaw_cond_thresh = std::max(0.0f, thresh); }  // place the integral windup limit to be this fraction of the output range 
    void set_dir(cdir a_dir) { _dir = a_dir; }
    void set_dir(int a_dir) { _dir = (cdir)a_dir; }
    void set_pmode(pmod a_pmode) { _pmode = a_pmode; }
    void set_pmode(int a_pmode) { _pmode = (pmod)a_pmode; }
    void set_dmode(dmod a_dmode) { _dmode = a_dmode; }
    void set_dmode(int a_dmode) { _dmode = (dmod)a_dmode; }
    void set_awmode(awmod a_awmode) { _awmode = a_awmode; }
    void set_awmode(int a_awmode) { _awmode = (awmod)a_awmode; }
    void set_outsum(float a_outsum) { _outsum = a_outsum; }  // sets the value of the integral
    int sampletime() { return _sampletime; }  // provides sampletime in ms
    int mode() { return static_cast<int>(_mode); }
    int dir() { return static_cast<int>(_dir); }
    int pmode() { return static_cast<int>(_pmode); }
    int dmode() { return static_cast<int>(_dmode); }
    int awmode() { return static_cast<int>(_awmode); }
    int centmode() { return static_cast<int>(_centmode); }
    float err() { return _err; }
    float kp() { return dispkp; }
    float ki() { return dispki; }
    float kd() { return dispkd; }
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
    float max_out_changerate_ps() { return _max_out_changerate_ps; }  // returns current value of imposed max output change rate (per second). if 0.0 disables limits
    float out_changerate_ps() { return _out_changerate_ps; }  // returns current rate of output change per sec
    float* target_ptr() { return &_target; }
};

static std::string motormodecard[NumMotorModes] = { "NA", "Halt", "Idle", "Releas", "OpLoop", "PropLp", "ActPID", "AuStop", "AuHold", "Park", "Cruise", "Calib", "Start", "AuPID" };
static std::string cruiseschemecard[NumCruiseSchemes] = { "OnePul", "HoldTm" };
static std::string brakefeedbackcard[NumBrakeFB] = { "BkPosn", "BkPres", "Hybrid", "None" };
static std::string openloopmodecard[NumOpenLoopModes] = { "Median", "AutRel", "ARHold" };

// ServoMotor - a parent class providing common elements for motor manager child classes. These subclasses each
// coordinate all aspects of one motor, including related sensors & pid, having a standard control interface
class ServoMotor {
  protected:
    Hotrc* hotrc;
    Speedometer* speedo;
    Servo* motor;
    int pid_timeout = 50000;  // if too high, servo performance is choppy
    float lastoutput;
    Timer pid_timer, outchangetimer;
    int pin, freq, timerno;
  public:
    bool reverse = false;  // defaults. subclasses override as necessary
    float pc[NumMotorVals] = { 0.0f, NAN, 100.0f, 0.0f, NAN, NAN, NAN, NAN };  // percent values [OpMin/Parked/OpMax/Out/Govern/AbsMin/AbsMax/Margin]  values range from -100% to 100% are all derived or auto-assigned
    float si[NumMotorVals] = { 45.0f, NAN, 90.0f, NAN, 0.0f, 180.0f, 1.0f };  // standard si-unit values [OpMin/Parked/OpMax/Out/Govern/AbsMin/AbsMax/Margin]
    float us[NumMotorVals] = { NAN, 1500.0f, NAN, NAN, NAN, 500.0f, 2500.0f, NAN };  // us pulsewidth values [-/Cent/-/Out/-/AbsMin/AbsMax/-]
    float max_out_changerate_pcps = 200.0f;  // max rate of change of motor output as a percent of motor range per second. set to 0 to disable limitation
    ServoMotor(int _pin, int _timerno, int _freq) : pin(_pin), timerno(_timerno), freq(_freq) {}
    void setup(Hotrc* _hotrc, Speedometer* _speedo) {
        hotrc = _hotrc;
        speedo = _speedo;
        ESP32PWM::allocateTimer(timerno);
        motor = new Servo();
        motor->setPeriodHertz(freq);
        pid_timer.set(pid_timeout);
        motor->attach(pin, us[AbsMin], us[AbsMax]);  // Servo goes from 500us (+90deg CW) to 2500us (-90deg CCW)
    }
    float out_pc_to_si(float _pc) {
        return map(_pc, pc[AbsMin], pc[AbsMax], si[AbsMin], si[AbsMax]); 
    }
    float out_si_to_pc(float _si) {
        return map(_si, si[AbsMin], si[AbsMax], pc[AbsMin], pc[AbsMax]);
    }
    float out_si_to_us(float _si, bool rev=false) {  // works for motor with or without stop value
        return map(_si, si[AbsMin], si[AbsMax], rev ? us[AbsMax] : us[AbsMin], rev ? us[AbsMin] : us[AbsMax]);
    }
    float out_pc_to_us(float _pc, bool rev=false) {  // works for motor with or without stop value
        return map(_pc, pc[AbsMin], pc[AbsMax], rev ? us[AbsMax] : us[AbsMin], rev ? us[AbsMin] : us[AbsMax]);
    }
    void write_motor() {
        if (!std::isnan(us[Out])) motor->writeMicroseconds((int32_t)(us[Out]));
        lastoutput = pc[Out];    
    }
    void set(float* member, float val) {  // generic setter for any member floats. basically makes sure to rerun derive() after
        *member = val;
        derive();
    }
  protected:
    float rate_limiter(float a_outval) {
        if (iszero(max_out_changerate_pcps)) return a_outval;  // bypass rate limiter feature by setting max rate to 0
        float max_out_change_pc = max_out_changerate_pcps * outchangetimer.elapsed() / 1000000.0f;
        outchangetimer.reset();
        return constrain(a_outval, lastoutput - max_out_change_pc, lastoutput + max_out_change_pc);
    }
    void derive() {};  // child overload this
};
class JagMotor : public ServoMotor {
  protected:
    CarBattery* mulebatt;
    float car_batt_fake_v = 12.0f;
    Timer volt_check_timer{3500000};
  public:
    using ServoMotor::ServoMotor;
    float duty_fwd_pc = 100.0f;  // default. subclasses override as necessary
    float duty_rev_pc = 100.0f;  // default. subclasses override as necessary
    float* volt = si;  // apparently not legal:  float (&volt)[arraysize(si)] = si;  // our standard si value is volts. Create reference so si and volt are interchangeable
    #if !BrakeThomson
    // set opmin to avoid driving motor with under 8%
    #endif
    JagMotor(int _pin, int _timerno, int _freq) : ServoMotor(_pin, _timerno, _freq) {}
    void derive() {  // calc pc and voltage op limits from volt and us abs limits 
        si[AbsMax] = running_on_devboard ? car_batt_fake_v : mulebatt->val();
        si[AbsMin] = -(si[AbsMax]);
        pc[OpMin] = pc[AbsMin] * duty_rev_pc / 100.0f;
        pc[OpMax] = pc[AbsMax] * duty_fwd_pc / 100.0f;
        si[OpMin] = map(pc[OpMin], pc[Stop], pc[AbsMin], si[Stop], si[AbsMin]);
        si[OpMax] = map(pc[OpMax], pc[Stop], pc[AbsMax], si[Stop], si[AbsMax]);
        si[Margin] = map(pc[Margin], pc[AbsMin], pc[AbsMax], si[AbsMin], si[AbsMax]);  // us[Margin] = map(pc[Margin], pc[AbsMin], pc[AbsMax], us[AbsMin], us[AbsMax]);
    }
    float out_pc_to_si(float _pc) {
        if (_pc > pc[Stop]) return map(_pc, pc[Stop], pc[AbsMax], si[Stop], si[AbsMax]);
        if (_pc < pc[Stop]) return map(_pc, pc[Stop], pc[AbsMin], si[Stop], si[AbsMin]);
        return si[Stop];
    }
    float out_si_to_pc(float _si) {
        if (_si > si[Stop]) return map(_si, si[Stop], si[AbsMax], pc[Stop], pc[AbsMax]);
        if (_si < si[Stop]) return map(_si, si[Stop], si[AbsMin], pc[Stop], pc[AbsMin]);
        return pc[Stop];
    }
    float out_si_to_us(float _si, bool rev=false) {  // works for motor with center stop value
        if (_si > si[Stop]) return map(_si, si[Stop], si[AbsMax], us[Stop], rev ? us[AbsMin] : us[AbsMax]);
        if (_si < si[Stop]) return map(_si, si[Stop], si[AbsMin], us[Stop], rev ? us[AbsMax] : us[AbsMin]);
        return us[Stop];
    }
    float out_pc_to_us(float _pc, bool rev=false) {  // works for motor with center stop value
        if (_pc > pc[Stop]) return map(_pc, pc[Stop], pc[AbsMax], us[Stop], rev ? us[AbsMin] : us[AbsMax]);
        if (_pc < pc[Stop]) return map(_pc, pc[Stop], pc[AbsMin], us[Stop], rev ? us[AbsMax] : us[AbsMin]);
        return us[Stop];
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {
        us[Out] = pc[OpMin] = pc[OpMax] = si[OpMin] = si[OpMax] = NAN;
        pc[Out] = pc[Stop] = si[Out] = si[Stop] = 0.0f;
        pc[AbsMin] = -100.0f;
        pc[AbsMax] = 100.0f;
        pc[Margin] = 2.0f;
        us[Cent] = 1500.0f;
        us[AbsMin] = 670.0f;
        us[AbsMax] = 2330.0f;
        ServoMotor::setup(_hotrc, _speedo);
        mulebatt = _batt;
        derive();
        pc[Out] = out_si_to_pc(si[Out]);
        us[Out] = out_si_to_us(si[Out]);
    }
};
// pid_config : (configurable default)
//    OpenLoop : Servo angle is simply proportional to trigger pull. This is our tested default
//    ActivePID : Servo angle is determined by PID calculation designed to converge engine rpm to a target value set proportional to trigger pull
// cruise_adjust_scheme : (compile time option) Pick from different styles for adjustment of cruise setpoint. I prefer HoldTime.
//    OnePull : Cruise locks servo angle (throttle_target_pc), instead of pid. Moving trigger from center adjusts setpoint proportional to how far you push it before releasing (and reduced by an attenuation factor)
//    HoldTime : Cruise locks servo angle (throttle_target_pc), instead of pid. Any non-center trigger position continuously adjusts setpoint proportional to how far it's pulled over time (up to a specified maximum rate)
class ThrottleControl : public ServoMotor {
// replace with class ThrottleControl {
  private:
    Tachometer* tach;
    Potentiometer* pot;
    TemperatureSensorManager* tempsens;
    float gas_kp = 0.13f;             // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
    float gas_ki = 0.50f;             // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
    float gas_kd = 0.00f;             // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
    float cruise_kp[2] = { 2.33f,  5.57f }; // [GasOpen/GasPID] PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
    float cruise_ki[2] = { 1.06f, 11.00f }; // [GasOpen/GasPID] PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
    float cruise_kd[2] = { 0.00f,  0.00f }; // [GasOpen/GasPID] PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
    float* cruise_outmin_ptr[2] = { &pc[OpMin],  tach->idle_ptr() };      // [GasOpen/GasPID] 
    float* cruise_outmax_ptr[2] = { &pc[Govern], tach->governmax_ptr() }; // [GasOpen/GasPID] 
    bool pid_ena_last = false, cruise_pid_ena_last = false;
  public:
    // replace this: ...
    using ServoMotor::ServoMotor;
    // ThrottleControl(int _pin, int _freq) { pin = _pin; freq = _freq; };
    float tach_last, throttle_target_pc; //, max_throttle_angular_velocity_pcps;
    float linearizer_exponent = 3.75f;
    float cruise_linearizer_exponent = 1.05f;
    float trigger_vert_pc = 0.0f;
    float idle_max_boost_pc = 15.0f;  // max amount (in percent) to boost idle if the engine is too cold
    float idle_temp_lim_f[2] = { 60.0f, 80.0f };  // [LOW]/HIGH] temperature range (in F) over which to apply idle boost. max boost at temp=LOW or less, and no boost at temp=HIGH or more 
    // float idle_si[NumMotorVals] = { 58.0f, NAN, 65.0f, NAN, NAN, 0.0f, 180.0f, 1.0f };          // in angular degrees [OpMin(hot)/-/OpMax(cold)/Out/-/AbsMin/AbsMax/Margin]
    // float idletemp_f[NumMotorVals] = { 60.0f, NAN, 205.0f, 75.0f, NAN, 40.0f, 225.0f, 1.5f};      // in degrees F [OpMin/-/OpMax/Out/-/AbsMin/AbsMax/Margin]
    float _idle_pc = 0.0f, idle_boost_pc = 0.0f; // 11.3f;                // idle percent is derived from the si (degrees) value
    float starting_pc = 25.0f;                          // percent throttle to open to while starting the car
    float idle_temp_f = NAN;  // temperature determined for purpose of calculating boost. will prefer engine temp sensor but has fallback options 
    QPID pid, cruisepid;
    int throttle_ctrl_mode = OpenLoop;   // set default ctrl behavior. should gas servo use the rpm-sensing pid? values: ActivePID or OpenLoop
    bool pid_enabled, cruise_pid_enabled;  // if servo higher pulsewidth turns ccw, then do reverse=true   // cruise_trigger_released = false; 
    bool just_started_cruise_session = true;
    int cruise_adjust_scheme = HoldTime, motormode = Parked;  // not tunable  // pid_status = OpenLoop, cruise_pid_status = OpenLoop,
    float* deg = si;             // our standard si value is degrees of rotation "deg". Create reference so si and deg are interchangeable
    float pc_to_rpm(float _pc) { return map(_pc, 0.0f, 100.0f, tach->idle(), tach->opmax()); }
    float rpm_to_pc(float _rpm) { return map(_rpm, tach->idle(), tach->opmax(), 0.0f, 100.0f); }
    void derive() {  // calc derived limit values for all units based on tuned values for each motor
        pc[AbsMin] = map(si[AbsMin], si[OpMin], si[OpMax], pc[OpMin], pc[OpMax]);
        pc[AbsMax] = map(si[AbsMax], si[OpMin], si[OpMax], pc[OpMin], pc[OpMax]);
        pc[Parked] = map(si[Parked], si[OpMin], si[OpMax], pc[OpMin], pc[OpMax]);
        pc[Govern] = map(governor, 0.0f, 100.0f, pc[OpMin], pc[OpMax]);  // pc[Govern] = pc[OpMin] + governor * (pc[OpMax] - pc[OpMin]) / 100.0f;      
        si[Govern] = map(pc[Govern], pc[OpMin], pc[OpMax], si[OpMin], si[OpMax]);
        pc[Margin] = map(si[Margin], si[OpMin], si[OpMax], pc[OpMin], pc[OpMax]);
    }   // max_throttle_angular_velocity_pcps = 100.0f * max_throttle_angular_velocity_degps / (si[OpMax] - si[OpMin]);
    void set_out_changerate_pcps(float newrate) {
        max_out_changerate_pcps = newrate;  // max_out_changerate_pcps = 100.0f * max_out_changerate_degps / (si[OpMax] - si[OpMin]);
        pid.set_max_out_changerate(max_out_changerate_pcps);
    }
    void set_out_changerate_degps(float newrate_deg) {
        if (iszero(si[OpMax] - si[OpMin])) return;  // no divide by zero
        set_out_changerate_pcps(newrate_deg * (pc[OpMax] - pc[OpMin]) / (si[OpMax] - si[OpMin]));
    }
    float get_max_out_changerate_degps() {
        if (iszero(pc[OpMax] - pc[OpMin])) return NAN;  // no divide by zero
        return max_out_changerate_pcps * (si[OpMax] - si[OpMin]) / (pc[OpMax] - pc[OpMin]);
    }
    void init_pid() {
        pid.init(tach->ptr(), &pc[OpMin], &pc[Govern], gas_kp, gas_ki, gas_kd, QPID::pmod::onerr, QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout);
    }
    void init_cruise_pid() {
        cruisepid.init(speedo->ptr(), cruise_outmin_ptr[pid_enabled], cruise_outmax_ptr[pid_enabled],
            cruise_kp[pid_enabled], cruise_ki[pid_enabled], cruise_kd[pid_enabled],
            QPID::pmod::onerr, QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout);
        update_cruise_pid();  // conditionally correct the cruise pid settings above
    }
    // Throttle calibration notes:
    // * with servo at center position (1500 us pulsewidth) mount the servo horn inline with center of the servo body's long dimension. this is 90 deg. CW from here are higher angles and CCW are lower angles
    // * for best linearity, adjust the servo-carb linkage length so the carb throttle is at halfway point when the servo is at the 90 degree center position
    // * calibration:
    //   1. mount hardware as described above, go to cal mode and enable the pot gas control. view the PWMs datapage
    //   2. turn servo using the pot to find the us pulsewidth values for opmin (closed), opmax (open), and parked angles
    //   3. use these values to initialize the us[OpMin], us[OpMax], and us[Parked] values
    //   4. if increasing pulsewidths moves servo CCW then set reverse = true
    //   5. note also the reported angular value to see how accurate they are. if not, fix the conversions (currently a map statement)
    // * set operational angular range (OpMin/OpMax), margin (Margin) and parking angle (Parked) in setup() function, in degree units (si)
    //
    // (old) us[OpMin] = 1365.0f;  us[Parked] = 1340.0f;  us[OpMax] = 2475.0f;  us[Margin] = 30.0f;
    // (old) float si[NumMotorVals] = { 88.8, NAN, 5.0f, 69.0f, NAN, 0.0f, 180, 1.0f };  // standard si-unit values [OpMin/Parked/OpMax/Out/Govern/AbsMin/AbsMax/Margin]
    // (old) float us[NumMotorVals] = { 1365.0f, 1340.0f, 2475.0f, NAN, NAN, 0.0f, 180, 1.0f };  // standard si-unit values [OpMin/Parked/OpMax/Out/Govern/AbsMin/AbsMax/Margin]

    void setup(Hotrc* _hotrc, Speedometer* _speedo, Tachometer* _tach, Potentiometer* _pot, TemperatureSensorManager* _temp) {
        tach = _tach;  pot = _pot;  tempsens = _temp;
        ezread.squintf(ezread.highlightcolor, "Throttle servo (p%d), pid %s\n", pin, pid_enabled ? "ena" : "dis");
        si[OpMin] = si[Out] = 66.3f;
        si[Parked] = 55.0f;
        si[OpMax] = 165.0f;
        si[Govern] = NAN;
        si[AbsMin] = 0.0f;
        si[AbsMax] = 180.0f;
        si[Margin] = 1.0f;
        reverse = false;
        ServoMotor::setup(_hotrc, _speedo);
        derive();
        init_pid();
        init_cruise_pid();
        set_pid_ena(throttle_pid_default);
        set_cruise_pid_ena(cruise_pid_default);
        set_out_changerate_degps(65.0f);  // don't turn faster than 65 degrees per sec
        ezread.squintf("  rev=%d 0%% = %dus, 100%% = %dus\n", reverse, (int)out_pc_to_us(0.0f, reverse), (int)out_pc_to_us(100.0f, reverse));
        ezread.squintf("  cruise pid %sabled, w/ %s adj scheme\n", cruise_pid_enabled ? "en" : "dis", cruiseschemecard[cruise_adjust_scheme].c_str());
    }
    float idle_pc() { return _idle_pc; }
    float idle_si() { return out_pc_to_si(_idle_pc); }
    float idle_us() { return out_pc_to_us(_idle_pc); }    
  private:
    void update_idle() {  // updates _idle_pc based on temperature, ranging from pc[OpMin] (when warm) to full boost% applied (when cold)
        if (!use_idle_boost) return;
        idle_temp_f = tempsens->val(loc::TempEngine);
        if (std::isnan(idle_temp_f)) idle_boost_pc = 0.0f;  // without valid temp reading do not boost
        else idle_boost_pc = map(idle_temp_f, idle_temp_lim_f[LOW], idle_temp_lim_f[HIGH], idle_max_boost_pc, 0.0f);
        idle_boost_pc = constrain(idle_boost_pc, 0.0f, idle_max_boost_pc);
        tach->set_idle(map(idle_boost_pc, 0.0f, idle_max_boost_pc, tach->idle_hot(), tach->idle_cold()));
        _idle_pc = pc[OpMin] + idle_boost_pc;
    }  // ezread.squintf(" si:%lf pc:%lf\n", idle_si[Out], idle_pc);
    float cruise_adjust(int joydir, float thr_targ) {
        if (cruise_adjust_scheme == HoldTime) {  // this scheme continually adjusts the current thr_targ based on how far from center (in either dir) you hold the trigger
            static Timer deltatimer;             // timer allows algorithm to know time elapsed since last function call
            if (cruise_adjusting) thr_targ += trigger_vert_pc * cruise_holdtime_attenuator_pc * deltatimer.elapsed() / (100.0f * 1000000.0f); // if this isn't our first time through (b/c timer hasn't been reset), calculate our adjustment since last loop
            deltatimer.reset();                  // reset timer so we can measure elapsed loop time the next time through
        }
        else if (cruise_adjust_scheme == OnePull) {  // this scheme makes a single adjustment with each departure from trigger center, based on how far you push it (in either dir).
            static float running_max_pc;             // stores the furthest the trigger has been from center since start of the adjustment event
            if (!cruise_adjusting) running_max_pc = hotrc->pc[Vert][Cent];  // on first time thru, recenter our max-trigger-value-so-far value back to center
            if (std::abs(trigger_vert_pc) >= std::abs(running_max_pc)) {  // if new trigger read beats our record so far (to avoid continuing adjustments when trigger hasn't increased)
                thr_targ += (trigger_vert_pc - running_max_pc) * cruise_onepull_attenuator_pc / 100.0f;  // make an adjustment proportional to the incremental trigger difference
                running_max_pc = trigger_vert_pc;    // update our value for the furthest trigger read thusfar during the push
            }
        }
        else ezread.printf(ezread.madcolor, "err: invalid cruise scheme=%d\n", cruise_adjust_scheme);  // leaving thr_targ unchanged
        return thr_targ;
    }
    float cruise_logic(float thr_targ) {
        int joydir = hotrc->joydir(Vert);                           // read vertical joystick direction
        if (just_started_cruise_session) {                          // run once at start of each session to set cruise control values
            cruise_adjusting = false;                               // initialize value of cruise_adjusting for this session
            if (cruise_pid_enabled) cruisepid.set_output(thr_targ);  // if this is the end of an adjustment, update pid out to current gas tgt
            if (cruise_pid_enabled) thr_targ = cruisepid.compute(); // if running on pid and not adjusting, use the pid math to drive thr_targ
            just_started_cruise_session = false;                    // if (joydir == HrcCent) cruise_trigger_released = true; // if joystick is at center, flag that it has been to center since first entering cruise mode
        }
        if (joydir == HrcCent) {     // if joystick is at center, or this is the first loop of our session  //  || !cruise_trigger_released
            if (cruise_pid_enabled && cruise_adjusting) cruisepid.set_output(thr_targ);  // if this is the end of an adjustment, update pid out to current gas tgt
            cruise_adjusting = false;                               // flag that no cruise adjustment is in progress. (resumes pid ctrl of thrtarg, if running pid)
            if (cruise_pid_enabled) thr_targ = cruisepid.compute(); // if running on pid and not adjusting, use the pid math to drive thr_targ
        }                                                           // note, so far thr_targ has been left unchanged, ie the basic point of cruise ctrl after all
        else {                                                      // if trigger is off center (and not b/c it's returning to center after first entering cruise)
            thr_targ = cruise_adjust(joydir, thr_targ);             // we are adjusting, so do adjustment & get new thr_targ. must come before cruise_adjusting=true
            cruise_adjusting = true;                                // flag the current adjust event is initialized & in progress (suspends pid ctrl of thrtarg)
            if (cruise_pid_enabled) cruisepid.set_target(speedo->val());  // during adjustment, constantly update the pid speed target to the current speed
        }
        return thr_targ;
    }
    void set_output() {
        // static int last_mode = Calibrate;
        float new_out, out_lowlim = _idle_pc;
        if (motormode == Halt) return;  // a servo by its nature will hold its current angle whenever its pwm input pulsewidth isn't changing
        else if (motormode == Calibrate) {  // allows adjustment of gas using potmap, with its operational limits temporarily disabled (to facilitate calibrating those limits)
            cal_gasmode = true;  // flag that we have officially entered cal mode, as the potmapped gas was within the incumbent operational limits when cal mode started
            new_out = map(pot->val(), pot->opmin(), pot->opmax(), si[AbsMin], si[AbsMax]);  // translate pot value to a gas value
            pc[Out] = constrain(out_si_to_pc(new_out), pc[AbsMin], pc[AbsMax]);  // make doubly sure the resulting gas value is within its *absolute* limits 
            return;  // cal mode sets the output directly, ignoring the normal operational process at the bottom of this function
        }  // ezread.squintf(":%d tgt:%lf pk:%lf idl:%lf\n", motormode, throttle_target_pc, pc[Parked], _idle_pc);
        
        trigger_vert_pc = hotrc->pc[Vert][Filt];  // copy current trigger value to internal value, in case we linearize it
        if (motormode == Idle) throttle_target_pc = _idle_pc;
        else if (motormode == Starting) throttle_target_pc = starting_pc;
        else if (motormode == ParkMotor) throttle_target_pc = out_lowlim = pc[Parked];
        else if (motormode == CruiseMode) {
            if (throttle_linearize_cruise) linearizer(&trigger_vert_pc, cruise_linearizer_exponent);  // we now linearize the trigger value not the output value
            throttle_target_pc = cruise_logic(throttle_target_pc);  // cruise mode just got too big to be nested in this if-else clause
        }
        else if (motormode == OpenLoop || motormode == ActivePID) {  // combine these into one "Driving" mode
            if (throttle_linearize_trigger) linearizer(&trigger_vert_pc, linearizer_exponent);  // we now linearize the trigger value not the output value
            if (hotrc->joydir() == HrcUp) throttle_target_pc = map(trigger_vert_pc, hotrc->pc[Vert][Cent], hotrc->pc[Vert][OpMax], _idle_pc, pc[Govern]);  // actuators still respond even w/ engine turned off
            else throttle_target_pc = _idle_pc;  // If in deadband or being pushed down, we want idle
        }
        else {
            ezread.squintf(ezread.madcolor, "err: invalid gas motor mode=%d\n", motormode);  // ignition.panic_request(ReqOn);
            return;
        }
        
        throttle_target_pc = constrain(throttle_target_pc, pc[Parked], pc[OpMax]);
        if (pid_enabled) {
            pid.set_target(pc_to_rpm(throttle_target_pc));  // revisit this line, it may not be this simple to convert to rpm
            new_out = pid.compute();
        }
        else new_out = throttle_target_pc;
        if (!pid_enabled) new_out = rate_limiter(new_out);  // the pid should already be set to limit rate if needed.   pc[Out] = rate_limiter(new_out);  max_out_change_rate_pcps
        pc[Out] = constrain(new_out, out_lowlim, pc[Govern]);   // pid should constrain on its own, do not want to go editing its output
        // if (last_mode != motormode) ezread.printf("gas mode changed %s -> %s\n", motormodecard[last_mode].c_str(), motormodecard[motormode].c_str());
        // last_mode = motormode;
    }
  public:
    void setmode(int _mode) {
        int lastmode = motormode;  // remember incumbent mode
        if (_mode == AutoPID) _mode = throttle_pid_default ? ActivePID : OpenLoop;  // translate "AutoPID" (a fake mode) into ActivePID or OpenLoop mode, depending on current configuration
        if (_mode == motormode) return;  // if mode isn't being changed, then do nothing and ditch out
        cal_gasmode = false;  // 
        if (_mode == Calibrate) {
            float temp = pot->mapToRange(0.0f, 180.0f);
            if (temp < si[Parked] || temp > si[OpMax]) return;  // do not change to cal mode if attempting to enter while pot is out of range
        }
        else if (_mode == ActivePID) set_pid_ena(true);
        else if (_mode == OpenLoop) set_pid_ena(false);
        else if (_mode == CruiseMode) {           // upon entering cruise mode ...
            just_started_cruise_session = true;  // flag that the trigger could be off center upon entering cruise mode, so we don't interpret that as an adjustment in progress // cruise_trigger_released = false;  
            update_cruise_pid();  // update the cruise pid adjustments to the current settings, and its variables to the current state, for a smooth transition into cruise mode
            throttle_target_pc = pid_enabled ? tach->val() : pc[Out]; // set throttle target to its current state, for a smooth transition into cruise mode
        }
        motormode = _mode;  // actually change the motor mode
        // ezread.printf("gas mode set to %s\n", motormodecard[motormode].c_str());
    }
    void update_pid() {  // must call whenever governor or tach limits are changed
        if (pid_enabled) pid.set_limits(&pc[OpMin], &pc[Govern]);  // pid.reset();
        pid.set_output(pid_enabled ? tach->val() : pc[Out]); // set pid output to current value appropriate to current gas pid status
    }
    void set_cruise_tunings(float a_kp=NAN, float a_ki=NAN, float a_kd=NAN) {  // for external tuner to calibrate the cruise pid coefficients. helper function needed to remember the edited settings independently for whether gas pid is open or closed loop
        if (!std::isnan(a_kp)) cruise_kp[pid_enabled] = a_kp;  // remember these calibrated settings as specific to the current state of the gas pid 
        if (!std::isnan(a_ki)) cruise_ki[pid_enabled] = a_ki;
        if (!std::isnan(a_kd)) cruise_kd[pid_enabled] = a_kd;
        cruisepid.set_tunings(cruise_kp[pid_enabled], cruise_ki[pid_enabled], cruise_kd[pid_enabled]);
    }
    void update_cruise_pid(bool limits_only=false) {  // always and only call whenever either pid enable is changed. send in true if only adjusting limits
        if (!cruise_pid_enabled) return;
        cruisepid.set_limits(cruise_outmin_ptr[pid_enabled], cruise_outmax_ptr[pid_enabled]); // switch cruise pid limits to those appropriate for current gas pid status
        if (limits_only) return;
        cruisepid.set_target(speedo->val()); // set pid loop speed target to current speed
        cruisepid.set_output(pid_enabled ? tach->val() : pc[Out]); // set pid output to current value appropriate to current gas pid status
        cruisepid.set_tunings(cruise_kp[pid_enabled], cruise_ki[pid_enabled], cruise_kd[pid_enabled]);
        // cruisepid.reset();  // causes problems ... check into what this actually does
    }
    void set_cruise_pid_ena(bool new_cruise_pid_ena) {   // run w/o arguments each loop to enforce configuration limitations, or call with an argument to enable/disable pid and update. do not change pid_enabled anywhere else!
        bool cruise_pid_ena_last = cruise_pid_enabled;
        cruise_pid_enabled = new_cruise_pid_ena;
        if (cruise_pid_ena_last != cruise_pid_enabled) {
            update_cruise_pid();
            // ezread.squintf("cruis pid %s %s linz\n", cruise_pid_enabled ? "enab" : "disab", throttle_linearize_cruise ? "w/" : "w/o");
        }
    }    
    void set_pid_ena(bool new_pid_ena) {   // run w/o arguments each loop to enforce configuration limitations, or call with an argument to enable/disable pid and update. do not change pid_enabled anywhere else!
        bool pid_ena_last = pid_enabled;
        pid_enabled = new_pid_ena;
        if (pid_enabled != pid_ena_last) {
            if (motormode == OpenLoop && pid_enabled) motormode = ActivePID; 
            else if (motormode == ActivePID && !pid_enabled) motormode = OpenLoop; 
            update_pid();
            update_cruise_pid();
            // if (pid_ena_last != pid_enabled) ezread.squintf("throt pid %s %s linz\n", pid_enabled ? "enab" : "disab", throttle_linearize_trigger ? "w/" : "w/o");
        }
    }
    void set_cruise_scheme(int newscheme) {
        // if (cruise_pid_enabled) cruise_adjust_scheme = SuspendFly;  else // in pid mode cruise must use SuspendFly adjustment scheme (not sure if this restriction is necessary?)
        cruise_adjust_scheme = newscheme;  // otherwise anything goes
        // ezread.squintf("cruise using %s adj scheme\n", cruiseschemecard[cruise_adjust_scheme].c_str());
    }
    bool parked() { return (std::abs(out_pc_to_si(pc[Out]) - si[Parked]) < si[Margin]); }
    void set_governor_pc(float a_newgov) {
        governor = a_newgov;
        derive();
        update_pid();
        update_cruise_pid(true);  // update but to set limits only
    }
    void update() {
        if (runmode == LowPower) return;
        if (pid_timer.expireset()) {
            // with recent changes to tune() I had to move the setter function for these here, instead of inline in the tuner like it was, to make edit acceleration work
            // static float gov_last;                                // attempt to fix ui tuning acceleration of governor value
            // if (governor != gov_last) set_governor_pc(governor);  // attempt to fix ui tuning acceleration of governor value
            // gov_last = governor;                                  // attempt to fix ui tuning acceleration of governor value
            // update_ctrl_config();
            update_idle();                // Step 1 : do any idle speed management needed          
            set_output();                      // Step 2 : determine motor output value. updates throttle target from idle control or cruise mode pid, if applicable (on the same timer as gas pid). allows idle control to mess with tach_target if necessary, or otherwise step in to prevent car from stalling
            us[Out] = out_pc_to_us(pc[Out], reverse);   // Step 4 : convert motor value to pulsewidth time
            deg[Out] = out_pc_to_si(pc[Out]);
            write_motor();                     // Step 5 : write to servo
        }
    }  // ezread.squintf("out pc:%lf us:%lf\n", pc[Out], us[Out]);
};
// Brake uses two PID loops: one based on pressure (accurate for high brake pressures), and one based on position (accurate when pedal is more released)
// Note as pedal is pressed down, position decreases as pressure increases. PID output is percent motor power.
class BrakeControl : public JagMotor {
  private:
    BrakePositionSensor* brkpos;
    PressureSensor* pressure;
    ThrottleControl* throttle;
    TemperatureSensorManager* tempsens;

    // Notes on tuning coefficients:
    // these apply 33 times per second. so scale to have reasonable change versus entire range
    // full range:  pos: about 5 inch    press: about 1500 psi
    // arbitrary decide we expect a full range action to take max 3 seconds, then that's about 100 updates
    // full range averages (max):  pos = .05 in/update  press:  15 psi/update
    // double these due to each sensor covers its whole range mostly during when it's dominant:  pos: 0.1 in/update  press: 30 psi/update
    // start point for tuning: all coefficients should add to about this value for expected sane performance   
    float press_kp = 0.54f;        // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float press_ki = 0.22f;        // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float press_kd = 0.0f;        // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    float posn_kp = 30.3f;          // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
    float posn_ki = 8.0f;         // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
    float posn_kd = 0.0f;         // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)
    float _autostop_smooth_initial_pc = 60.0f;  // default initial applied braking to auto-stop or auto-hold the car (in percent of op range)
    float _autostop_smooth_increment_pc = 2.5f;  // default additional braking added periodically when auto stopping (in percent of op range)
    float _autostop_fast_initial_pc = 65.0f; // same as above but for fast-braking (during panic or when stopped)
    float _autostop_fast_increment_pc = 4.0f; // same as above but for fast-braking (during panic or when stopped)
    float _autohold_initial_pc = 100.0f;  // braking to apply when autoholding if car is stopped 
    float brakemotor_duty_spec_pc = 10.0f;  // = 25.0; // In order to not exceed spec and overheat the actuator, limit brake presses when under pressure and adding pressure
    float pres_out, posn_out, pc_out_last, posn_last, pres_last;
    float heat_math_offset, motor_heat_min = 75.0f, motor_heat_max = 200.0f;
    int dominantsens_last = HybridFB, last_external_mode_request = Halt;
    Timer stopcar_timer{8000000}, interval_timer{250000}, motor_park_timer{4000000}, motorheat_timer{500000}, blindaction_timer{3000000};
    void set_dominant_sensor(float _hybrid_ratio) {
        if (feedback == PressureFB || feedback == PositionFB) dominantsens = feedback;
        else if (std::isnan(_hybrid_ratio)) return;  // dominantsens = NoneFB;
        else dominantsens = (int)(_hybrid_ratio + 0.5f) ? PressureFB : PositionFB;  // round to 0 or 1, the indices of posn and pres respectively
        if (dominantsens == PositionFB) pid_dom = &(pids[PositionFB]);
        else pid_dom = &(pids[PressureFB]);
        posn_pid_active = (dominantsens == PositionFB);  // for display
    }
    float pressure_pc_to_si(float pc) {
        return map(pc, 0.0f, 100.0f, pressure->opmin(), pressure->opmax());
    }
  public:
    using JagMotor::JagMotor;
    bool pid_enabled = true, pid_ena_last = true, enforce_positional_limits = true, no_feedback = false;    // default for use of pid allowed
    int feedback = HybridFB, feedback_last = HybridFB;  // this is the default for sensors to use as feedback
    bool feedback_enabled[NumBrakeSens];
    int dominantsens, motormode = Halt, oldmode = Halt;  // not tunable
    int openloop_mode = AutoRelHoldable;  // if true, when in openloop the brakes release when trigger released. otherwise, control with thrigger using halfway point scheme
    int _fixed_target_mode = NA;  // allows brake in loop modes to stay in loop when releasing or parking motor
    bool brake_tempsens_exists = false, posn_pid_active = (dominantsens == PositionFB);
    QPID pids[NumBrakeSens];  // brake changes from pressure target to position target as pressures decrease, and vice versa
    QPID* pid_dom = &(pids[PositionFB]);  // AnalogSensor sensed[2];
    float duty_pc = 0.0f;  // our continuous estimate of current duty level of actuator (in pc)
    float brake_pid_trans_threshold_lo = 0.25f;  // tunable. At what fraction of full brake pressure will motor control begin to transition from posn control to pressure control
    float brake_pid_trans_threshold_hi = 0.50f;  // tunable. At what fraction of full brake pressure will motor control be fully transitioned to pressure control
    bool autostopping = false, autoholding = false;
    float target_si[NumBrakeSens];  // this value is the posn and pressure (and hybrid combined) target settings, or fed into pid to calculate setting if pid enabled, in si units appropriate to each sensor
    float hybrid_math_offset, hybrid_math_coeff, hybrid_sens_ratio, hybrid_sens_ratio_pc, target_pc, pid_err_pc;
    float combined_read_pc, target_margin_pc = 2.0f, hybrid_out_ratio = 1.0f, hybrid_out_ratio_pc = 100.0f;
    float _fixed_release_speed = -50.0f;  // rate an openloop pedal release will travel
    float motor_heat = NAN, motor_heatloss_rate = 3.0f, motor_max_loaded_heatup_rate = 1.5f, motor_max_unloaded_heatup_rate = 0.3f;  // deg F per timer timeout
    float open_loop_attenuation_pc = 75.0f, thresh_loop_attenuation_pc = 45.0f, thresh_loop_hysteresis_pc = 2.0f;  // when driving blind i.e. w/o any sensors, what's the max motor speed as a percent
    void derive() {  // to-do below: need stop/hold values for posn only operation!
        JagMotor::derive();
        // pc[Margin] = 100.0 * pressure->margin() / (pressure->opmax() - pressure->opmin());
        hybrid_math_offset = 0.5f * (brake_pid_trans_threshold_hi + brake_pid_trans_threshold_lo);  // pre-do some of the math to speed up hybrid calculations
        hybrid_math_coeff = M_PI / (brake_pid_trans_threshold_hi - brake_pid_trans_threshold_lo);  // pre-do some of the math to speed up hybrid calculations
    }
    bool detect_tempsens() {
        float trytemp = tempsens->val(loc::TempBrake);
        brake_tempsens_exists = !std::isnan(trytemp);
        ezread.squintf("  using heat %s sensor\n", brake_tempsens_exists ? "readings from detected" : "estimates in lieu of");
        return brake_tempsens_exists;
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt, PressureSensor* _pressure, BrakePositionSensor* _brkpos, ThrottleControl* _throttle, TemperatureSensorManager* _tempsens) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        ezread.squintf(ezread.highlightcolor, "Brake (p%d) pid %s, feedback: %s\n", pin, pid_enabled ? "enabled" : "disabled",  brakefeedbackcard[feedback].c_str());
        pressure = _pressure;  brkpos = _brkpos;  throttle = _throttle;  throttle = _throttle;  tempsens = _tempsens; 
        pid_timeout = 40000;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
        JagMotor::setup(_hotrc, _speedo, _batt);
        pres_last = pressure->val();
        posn_last = brkpos->val();
        detect_tempsens();
        if (!std::isnan(tempsens->val(loc::TempAmbient))) motor_heat_min = tempsens->val(loc::TempAmbient) - 2.0f;
        derive();
        update_ctrl_config();
        pids[PressureFB].init(pressure->ptr(), &(pc[OpMin]), &(pc[OpMax]), press_kp, press_ki, press_kd, QPID::pmod::onerr,
            QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::direct, pid_timeout, QPID::ctrl::manual, QPID::centmod::strict, pc[Stop]);  // QPID::centmod::off);
        pids[PositionFB].init(brkpos->ptr(), &(pc[OpMin]), &(pc[OpMax]), posn_kp, posn_ki, posn_kd, QPID::pmod::onerr, 
            QPID::dmod::onerr, QPID::awmod::cond, QPID::cdir::reverse, pid_timeout, QPID::ctrl::manual, QPID::centmod::strict, pc[Stop]);  // QPID::centmod::off);
        // pids[PressureFB].set_iaw_cond_thresh(0.25);  // set the fraction of the output range within which integration works at all
        // pids[PositionFB].set_iaw_cond_thresh(0.25);
        set_out_changerate_pcps(400.0);  // MotorFactoryStore actuator stutters and stalls when changing direction suddenly, when power is low
    }
    void set_out_changerate_pcps(float newrate) {
        max_out_changerate_pcps = newrate;
        // max_out_changerate_pcps = 100.0 * max_out_changerate_degps / (si[OpMax] - si[OpMin]);
        for (int mypid=PositionFB; mypid<=PressureFB; mypid++) pids[mypid].set_max_out_changerate(max_out_changerate_pcps);
    }
  private:
    void update_motorheat() {  // i am probably going to scrap all this nonsense and just put another temp sensor on the motor
        float added_heat, nowtemp, out_ratio;
        static bool printed_error = false;
        if (motorheat_timer.expireset()) {
            out_ratio = pc[Out] / 100.0f;
            if (brake_tempsens_exists) motor_heat = tempsens->val(loc::TempBrake);
            else {
                nowtemp = tempsens->val(loc::TempAmbient);
                if (std::isnan(motor_heat) && !std::isnan(nowtemp)) motor_heat = nowtemp;  // ezread.squintf("Actively forecasting brake motor heat generation\n");
                else {
                    if (std::isnan(nowtemp)) added_heat = motor_heatloss_rate / -4.0f;
                    else {
                        motor_heat_min = nowtemp;
                        if (motor_heat > nowtemp) added_heat = -motor_heatloss_rate * (motor_heat - nowtemp) / nowtemp;
                    }
                    if (pc[Out] <= brakemotor_duty_spec_pc + pc[Margin])
                        added_heat += map(pc[Out], 0.0f, -100.0f, 0.0f, motor_max_unloaded_heatup_rate);
                    else if (pc[Out] > brakemotor_duty_spec_pc - pc[Margin])
                        added_heat += map(pc[Out], brakemotor_duty_spec_pc, 100.0f, 0.0f, motor_max_loaded_heatup_rate);
                    motor_heat += added_heat;
                }
            }        
            // duty_pc is intended for us to estimate the current duty of the actuator, as a percent. for now is proportional to temp reading
            if (std::isnan(motor_heat)) {
                duty_pc = NAN;
                if (!printed_error) ezread.squintf(ezread.madcolor, "err: brake motor_heat value == NAN\n");
                printed_error = true;        
            }
            else {
                motor_heat = constrain(motor_heat, tempsens->absmin(loc::TempBrake), tempsens->absmax(loc::TempBrake));            
                duty_pc = map(motor_heat, motor_heat_min, motor_heat_max, 0.0f, brakemotor_duty_spec_pc);  // replace this w/ ongoing estimate
                if (overtemp_shutoff_brake) {  // here the brakemotor is shut off if overtemp. also in diag class the engine is stopped
                    if (motor_heat > tempsens->opmax(loc::TempBrake)) {
                        if (!printed_error) ezread.squintf(ezread.madcolor, "err: brake motor overheating. stop motor\n");
                        printed_error = true;
                        // setmode(Halt, false);  // commented b/c is done in diag (it didn't seem to work here)       // stop the brake motor // pc[Out] = pc[Stop];  // setmode(ParkMotor, false);
                        // ignition.request(ReqOff);  // request kill ignition  // commented this out b/c is already in diag brake check
                    }
                    else printed_error = false;
                }
            }
        }  // that's great to have some idea whether the motor is hot. but we need to take some actions in response
    }
    // the brake can be controlled by a pid or open loop. either way it will use all enabled sensors. The influence on
    // the final output from the pressure and position pids is weighted based on the pressure reading as the brakes
    // are pressurized beyond X psi, use pressure reading, and otherwise use position as feedback. This is because 
    // near either pressure extreme only one of the two sensors is accurate. So at lower psi, control is 100% 
    // position-based, fade to 100% pressure-based at mid/high pressures.
    // "dominant" PID means the PID loop (pressure or position) that has the majority influence over the motor
    float calc_hybrid_ratio(float pressure_val) {  // returns pressure influence as a ratio (from 0.0 to 1.0) for a given pressure level in percent 
        if (feedback == NoneFB) return NAN;
        if (feedback == PressureFB) return 1.0f;
        if (feedback == PositionFB) return 0.0f;
        float pressure_ratio = pressure_val / 100.0f;  // (pressure_val - pressure->opmin()) / (pressure->opmax() - pressure->opmin());  // calculate ratio of output to range
        if (pressure_ratio >= brake_pid_trans_threshold_hi) return 1.0f;  // at pressure above hi threshold, pressure has 100% influence
        if (pressure_ratio <= brake_pid_trans_threshold_lo) return 0.0f;  // at pressure below lo threshold, position has 100% influence
        return 0.5f + 0.5f * (float)std::sin(hybrid_math_coeff * (pressure_ratio - hybrid_math_offset));  // in between we make a steep but smooth transition during which both have some influence
        // TODO (251028) hybrid brake value can decrease as pressure increases (w/ position fixed) during transition from 100% pressure <-> 100% position, i noticed while simulating both on breadboard
    }
    void set_hybrid_ratio() {
        float temp = calc_hybrid_ratio(pressure->pc());
        if (!isnan(temp)) {
            hybrid_out_ratio = temp;  // calculate pressure vs. position multiplier based on the sensed values
            hybrid_out_ratio_pc = 100.0f * hybrid_out_ratio;  // for display
        }
        else hybrid_out_ratio = hybrid_out_ratio_pc = NAN;
        set_dominant_sensor(hybrid_out_ratio);  // round to 0 (posn pid) or 1 (pressure pid). this is for idiot light display
    }
    float get_hybrid_brake_pc(float _given_pres, float _given_posn) {  // uses current hybrid_ratio to calculate a combined brake percentage value from given pres and posn values
        return hybrid_out_ratio * _given_pres + (1.0f - hybrid_out_ratio) * _given_posn;  // combine pid outputs weighted by the multiplier
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
    float calc_loop_out() {  // returns motor output percent calculated based on current target_pc or target[] values, in a way consistent w/ current config
        if (motormode == PropLoop) {  // scheme to drive using sensors but without pid, just uses target as a threshold value, always driving motor at a fixed speed toward it
            float err = (combined_read_pc - target_pc);
            if (std::abs(err) < thresh_loop_hysteresis_pc) return pc[Stop];
            if (err > 0.0f) return thresh_loop_attenuation_pc * throttle->idle_pc();
            return thresh_loop_attenuation_pc * pc[OpMax];
        }
        else {
            return get_hybrid_brake_pc(pids[PressureFB].compute(), pids[PositionFB].compute());  // if (motormode == ActivePID) combine pid outputs weighted by the multiplier
        }
    }
    float calc_open_out() {
        float new_out = pc[Out];
        if (openloop_mode == AutoRelHoldable) {  // AutoRelHoldable: releases brake whenever trigger is released, and presses brake when trigger pulled to max. anywhere in between stops brake movement
            if (hotrc->pc[Vert][Filt] <= hotrc->pc[Vert][OpMin] + hotrc->pc[Vert][Margin])
                new_out = open_loop_attenuation_pc * pc[OpMax] / 100.0f;
            else if (hotrc->joydir() == HrcDn) new_out = pc[Stop];
            else new_out = pc[OpMin];
        }
        else if (openloop_mode == AutoRelease) {  // AutoRelease: releases brake whenever trigger is released, and presses brake proportional to trigger push. must hold trigger steady to stop brakes where they are
            if (hotrc->joydir() == HrcDn) new_out = open_loop_attenuation_pc * map(hotrc->pc[Vert][Filt], hotrc->pc[Vert][Cent], hotrc->pc[Vert][OpMin], pc[Stop], pc[OpMax]) / 100.0f;
            else new_out = pc[OpMin];
        }
        else if (openloop_mode == MedianPoint) {  // Median: holds brake when trigger is released, or if pulled to exactly halfway point. trigger pulls over or under halfway either proportionally apply or release the brake
            if (hotrc->joydir() == HrcDn) new_out = open_loop_attenuation_pc * map(hotrc->pc[Vert][Filt], hotrc->pc[Vert][Cent], hotrc->pc[Vert][OpMin], pc[OpMin], pc[OpMax]) / 100.0f;
            else new_out = pc[Stop];
        }
        return new_out;  // this is openloop (blind trigger) control scheme
    }
    void carstop(bool panic_support=true) {  // autogenerates motor target values with the goal of stopping the car
        static bool stopped_last = false, autostopping_last = false;
        bool panic = panic_support && panicstop;
        bool stopped_now = speedo->stopped();
        if (stopped_last && !stopped_now) stopcar_timer.reset();  // if car just started moving, start a timer to timebox our autostop attempt
        stopped_last = stopped_now;
        
        // First determine if we really ought to be stopping the car, and set the autostopping flag
        autostopping = !stopped_now && !stopcar_timer.expired();  // if car is moving and we are trying to stop it, this status flag is set
        if (!pid_enabled) autostopping = autostopping && panic && !blindaction_timer.expired();  // to panic stop without any feedback, we just push as hard as we can for a couple seconds then stop
        // ezread.squintf("as:%d st:%d ex:%d\n", autostopping, stopped_now, stopcar_timer.expired());
        
        // If indeed autostopping, apply ever-increasing brakes until stopped
        if (autostopping) {  // if car is stopped or our timer expired, skip autostop action below
            if (!pid_enabled) pc[Out] = pc[OpMax];  // note this is dangerous in that it could mechanically break stuff. if running open loop avoid panicstops
            else {  // if autostopping while using brake feedback of any kind
                bool fast = panic || stopped_now;
                if (interval_timer.expireset()) set_target(std::min(pressure->opmax(), target_pc + fast ? _autostop_fast_increment_pc : _autostop_smooth_increment_pc));  // gradually increase brake pressure on regular intervals until car stops
                else set_target(std::max(target_pc, fast ? _autostop_fast_initial_pc : _autostop_smooth_initial_pc));  // initially set pressure target to baseline value, then hold it constant in between intervals
            }
        }
        else if (autostopping_last) {
            if (!pid_enabled) pc[Out] = pc[Stop];
            else {
                set_target(pc[Stop]);  // when autostopping effort ends, stop the motor
                ezread.squintf("autostop session ended, stopping brake motor");
            }
        }
        // if (pid_enabled) pc[Out] = calc_loop_out();
        autostopping_last = autostopping;
    }
    bool goto_fixed_position(float tgt_position, bool at_position) {  // goes to a fixed position (hopefully) or pressure (if posn is unavailable) then stops.  useful for parking and releasing modes
        // active_sensor = (enabled_sensor == PressureFB) ? PressureFB : PositionFB;  // use posn sensor for this unless we are specifically forcing pressure only
        if (motormode == PropLoop || motormode == ActivePID) {
            set_target(tgt_position);
            return (std::abs(target_pc - combined_read_pc <= target_margin_pc));
        }
        bool in_progress;
        if (feedback == NoneFB) in_progress = (!blindaction_timer.expired());  // if running w/o feedback, let's blindly release the brake for a few seconds then halt it
        else in_progress = ((brkpos->pc() < tgt_position) && !motor_park_timer.expired());
        if (in_progress) pc[Out] = _fixed_release_speed;
        else setmode(Halt, false); 
        return in_progress;
    }
    void set_output() {  // sets pc[Out] in whatever way is right for the current motor mode
        if (!(motormode == OpenLoop || feedback == NoneFB)) {  // open loop should skip calculating hybrid ratio
            set_hybrid_ratio();
            read_sensors();
        }

        // AutoHold will initially stop the car (if moving) or apply decent brake pressure (if not), then thereafter ensure it stays stopped while in this mode
        if (motormode == AutoHold) {  // autohold: apply initial moderate brake pressure, and incrementally more if car is moving. If car stops, then stop motor but continue to monitor car speed indefinitely, adding brake as needed
            carstop(false);  // stop the car if moving (will set autostopping flag correspondingly). this may increase the pressure target, but not decrease
            if (!autostopping) set_target(std::max(target_pc, _autohold_initial_pc));  // set target to a decent amount of pressure
            autoholding = !autostopping && (get_hybrid_brake_pc(pressure->pc(), brkpos->pc()) >= _autohold_initial_pc);  // set flag
            // TODO !! May need better logic to overpower back-force slip of new motor!

            // ezread.squintf("as:%d ah:%d f:%lf, h:%lf, m:%lf\n", autostopping, autoholding, pressure->val(), pressure->hold_initial_psi, pressure->margin_psi);            
            // if (feedback == NoneFB) pc[Out] = target_pc;  else
            pc[Out] = calc_loop_out();   // if (autoholding) pc[Out] = pc[Stop];  // kills power to motor while car is stopped. 
        }

        // AutoStop will try to stop the car by closing throttle and applying steadily increasing brakes. When stopped or timeout, mode changes to  then ensure it stays stopped while in this mode
        else if (motormode == AutoStop) {  // autostop: if car is moving, apply initial pressure plus incremental pressure every few seconds until it stops or timeout expires, then stop motor and cancel mode
            // ezread.printf("gas debug: mmode=%s, eval=%d\n", motormodecard[motormode].c_str(), (int)((run_motor_mode[runmode][_Throttle] == ParkMotor)));
            throttle->setmode((run_motor_mode[runmode][_Throttle] == ParkMotor) ? ParkMotor : Idle);  // Stop pushing the gas, will help us stop the car better
            carstop(true);  // this will set the autostopping flag as appropriate, and set increasing brake pressure target if so
            if (!autostopping) setmode(Halt, false);  // After AutoStop mode stops the car or times out, then stop driving the motor
            pc[Out] = calc_loop_out();
        }
        else if (motormode == Halt) {
            pc[Out] = pc[Stop];
        }
        else if (motormode == Calibrate) {
            cal_brakemode = true;  // for sunmode state machine
            int _joydir = hotrc->joydir(); 
            if (_joydir == HrcUp) pc[Out] = map(hotrc->pc[Vert][Filt], hotrc->pc[Vert][Cent], hotrc->pc[Vert][OpMax], pc[Stop], pc[OpMax]);
            else if (_joydir == HrcDn) pc[Out] = map(hotrc->pc[Vert][Filt], hotrc->pc[Vert][OpMin], hotrc->pc[Vert][Cent], pc[OpMin], pc[Stop]);
            else pc[Out] = pc[Stop];
        }
        else if (motormode == Release) {  // this will not get entered if using loop control. instead target is set in setmode function
            releasing = goto_fixed_position(brkpos->zeropoint_pc(), brkpos->released());
            // if (feedback == NoneFB) pc[Out] = target_pc; else
            // pc[Out] = calc_loop_out();   // if (autoholding) pc[Out] = pc[Stop];  // kills power to motor while car is stopped. 
        }
        else if (motormode == ParkMotor) {  // this will not get entered if using loop control. instead target is set in setmode function
            parking = goto_fixed_position(brkpos->parkpos_pc(), brkpos->parked());  // will set target to parked position
            // if (feedback == NoneFB) pc[Out] = target_pc;  else
            // pc[Out] = calc_loop_out();   // if (autoholding) pc[Out] = pc[Stop];  // kills power to motor while car is stopped. 
        }
        else if (motormode == OpenLoop) {  // in open loop, push trigger out less than 1/2way, motor extends (release brake), or more than halfway to retract (push brake), w/ speed proportional to distance from halfway point 
            pc[Out] = calc_open_out();
        }
        else if ((motormode == PropLoop) || (motormode == ActivePID)) {
            if (_fixed_target_mode == Release) releasing = goto_fixed_position(brkpos->zeropoint_pc(), brkpos->released());
            else if (_fixed_target_mode == ParkMotor) parking = goto_fixed_position(brkpos->parkpos_pc(), brkpos->parked());
            else {  // when dynamically driving
                if (hotrc->joydir(Vert) != HrcDn) set_target(pc[Stop]);  // let off the brake
                else set_target(map(hotrc->pc[Vert][Filt], hotrc->pc[Vert][Cent], hotrc->pc[Vert][OpMin], 0.0f, 100.0f));  // If we are trying to brake, scale joystick value to determine brake pressure setpoint
            }
            pc[Out] = calc_loop_out();
        }
    }
    void postprocessing() {  // keep within the operational range, or to full absolute range if calibrating (caution don't break anything!)
        float new_out = pc[Out];
        if (motormode == ActivePID);  // don't mess with the output value that's being controlled internally to a pid. except for an artificial cleanzero unbeknownst to the pid
        else if (motormode == Calibrate) new_out = constrain(new_out, pc[AbsMin], pc[AbsMax]);
        else if (enforce_positional_limits                                // IF we're not to exceed actuator position limits
          && ((new_out < pc[Stop] && brkpos->val() > brkpos->opmax())     // AND ( trying to extend while at extension limit OR trying to retract while at retraction limit )
          || (new_out > pc[Stop] && brkpos->val() < brkpos->opmin())))                              //  - brkpos->margin() //  + brkpos->margin()
            new_out = pc[Stop];                                           // THEN stop the motor
        else new_out = constrain(new_out, pc[OpMin], pc[OpMax]);          // constrain motor value to operational range (in pid mode pids should manage this)
        cleanzero(&new_out, 0.01f);  // if (std::abs(pc[Out]) < 0.01) pc[Out] = 0.0;                                 // prevent stupidly small values which i was seeing
        pc[Out] = rate_limiter(new_out);  // changerate_limiter();
        // for (int p = PositionFB; p <= PressureFB; p++) if (feedback_enabled[p]) pids[p].set_output(pc[Out]);  // feed the final value back into the pids
    }
  public:
    void setmode(int _mode, bool external_request=true) {  // external_request set to false when calling from inside the class (to remember what the outside world wants the mode to be)   
        bool mode_forced = false;                                                 // note whether requested mode was overriden
        int save_mode_request = _mode;
        if ((motormode == PropLoop) || (motormode == ActivePID)) {
            if (_mode == Release || _mode == ParkMotor) {
                _fixed_target_mode = _mode;  // tells loop to target a fixed braking lavel instead of user control. Avoids stopping pid
                return;  // skip the mode change, we are just using new targets
            }
            else if (_mode != motormode || _fixed_target_mode == NA) _fixed_target_mode = motormode;  // remove special target mode, back to dynamic control
        }
        if (feedback == NoneFB) {                                                 // if there is no feedback
            if (_mode == AutoStop && panicstop) ezread.squintf(ezread.sadcolor, "warn: performing blind panic stop maneuver\n");
            else if (_mode == AutoHold || _mode == AutoStop) {    // todo: rethink these scenarios 
                ezread.squintf(ezread.sadcolor, "warn: autobrake unavailable in openloop\n");
                return;  // keep current mode
            }
            else if (_mode == Release || _mode == ParkMotor) {
                _mode = Halt;  // TODO: should have a timer way to release/park w/o sensors
                if (_mode == Release) releasing = true;
                if (_mode == ParkMotor) parking = true;
                mode_forced = true;
            }
            if (_mode == ActivePID || _mode == PropLoop) { 
                // preforce_drivemode = _mode;                                       // remember our requested drive mode before demoting to openloop mode, so we can restore it if settings changed
                mode_forced = true;
                _mode = OpenLoop;      // can't use loops, drop to openloop instead
            }
            // else if (_mode == Release || _mode == ParkMotor) _mode = Halt;   // in openloop we lose some safeties, only use if necessary 
        }
        else if (!pid_enabled) {                         // if we have feedback but pid is disabled in config
            if (_mode == ActivePID) _mode = PropLoop;  // drop to simple threshold-based loop scheme
        }
        if (_mode == motormode) return;
        if (external_request) last_external_mode_request = save_mode_request;  // remember what the outside world actually asked for, in case we override it but later wish to go back
        autostopping = autoholding = cal_brakemode = parking = releasing = false;        
        if ((motormode == PropLoop) || (motormode == ActivePID)) _fixed_target_mode = _mode;  // cancel any fixed target condition
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
        else if (new_pid_ena == 0 || new_pid_ena == 1) pid_enabled = (bool)new_pid_ena;     // otherwise receive new pid enable setting (On/Off) if given
        feedback_enabled[PositionFB] = ((feedback == PositionFB) || (feedback == HybridFB));  // set sensor enable consistent with feedback config
        feedback_enabled[PressureFB] = ((feedback == PressureFB) || (feedback == HybridFB));  // set sensor enable consistent with feedback config
        no_feedback = (feedback == NoneFB);                              // for idiot light display
        if ((feedback != feedback_last) || (pid_enabled != pid_ena_last)) {
            setmode(last_external_mode_request, false);                            // ensure current motor mode is consistent with configs set here
            derive();  // on change need to recalculate some values
            ezread.squintf("brake pid %s feedback: %s\n", pid_enabled ? "enabled" : "disabled", brakefeedbackcard[feedback].c_str());        
        }
        if (new_openloop_mode >= 0 && new_openloop_mode <= NumOpenLoopModes) openloop_mode = new_openloop_mode;
        feedback_last = feedback;
        pid_ena_last = pid_enabled;
    }
    void update() {                 // brakes - determine motor output and write it to motor
        if (runmode == LowPower) return;
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            update_ctrl_config();   // catch any change in configuration affecting motor mode
            update_motorheat();     // update motor heat management, will halt the motor and panic if overheated
            set_output();           // determine motor percent value
            postprocessing();       // fix motor pc value if it's out of range or threatening to exceed positional limits
            us[Out] = out_pc_to_us(pc[Out], reverse); // convert motor percent value to pulse width for motor
            volt[Out] = out_pc_to_si(pc[Out]);        // convert to volts for display
            write_motor();                            // write to motor
        }
    }
    bool parked() {
        if (!feedback_enabled[PressureFB]) return brkpos->parked();
        else if (!feedback_enabled[PositionFB]) return pressure->released();  // pressure doesn't have a parked() function yet
        else return (combined_read_pc <= sensmin());  // TODO - sensmin() value is untested and arbitrary - replace with a calibrated value?
        ezread.squintf(ezread.sadcolor, "warn: brake unaware if parked w/o sensors\n");
        return false;  // really without sensors we have no idea if we're parked. Print an error message
    }
    bool released() {
        if (!feedback_enabled[PressureFB]) return brkpos->released();
        else if (!feedback_enabled[PositionFB]) return pressure->released();  // pressure doesn't have a parked() function yet
        else return (combined_read_pc <= sensmin());  // TODO - sensmin() value is untested and arbitrary - replace with a calibrated value?
        ezread.squintf(ezread.sadcolor, "warn: brake unaware if released w/o sensors\n");
        return false;  // really without sensors we have no idea if we're released. Print an error message
    }
    float sensmin() { return (dominantsens == PressureFB) ? pressure->opmin() : brkpos->opmin(); }
    float sensmax() { return (dominantsens == PressureFB) ? pressure->opmax() : brkpos->opmax(); }
    float duty() { return duty_pc; }
    float dutymin() { return 0.0f; }
    float dutymax() { return brakemotor_duty_spec_pc; }
    float motorheat() { return motor_heat; }
    float motorheatmin() { return motor_heat_min; }
    float motorheatmax() { return motor_heat_max; }
};
class SteeringControl : public JagMotor {
  public:
    using JagMotor::JagMotor;
    int motormode = Halt, oldmode = Halt;
    float steer_safe_pc = 25.0f;  // this percent is taken off full steering power when driving full speed (linearly applied)
    void set_out_changerate_pcps(float newrate) {
        max_out_changerate_pcps = newrate;
        // max_out_changerate_pcps = 100.0 * max_out_changerate_degps / (si[OpMax] - si[OpMin]);
        // pid.set_max_out_changerate(max_out_changerate_pcps); // if there ever is a pid
    }
    void setup(Hotrc* _hotrc, Speedometer* _speedo, CarBattery* _batt) {  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)
        set_out_changerate_pcps(350.0f);
        ezread.squintf(ezread.highlightcolor, "Steering motor {p%d)\n", pin);
        JagMotor::setup(_hotrc, _speedo, _batt);
    }
    void update() {
        if (runmode == LowPower) return;
        if (volt_check_timer.expireset()) derive();
        if (pid_timer.expireset()) {
            set_output();                     // Step 1 : Determine motor percent value
            us[Out] = out_pc_to_us(pc[Out], reverse);  // Step 3 : Convert motor percent value to pulse width for motor, and to volts for display
            volt[Out] = out_pc_to_si(pc[Out]);
            // static int count;
            // if (++count == 50)  ezread.squintf("to steering motor: %lf, %lf\n", pc[Out], us[Out]);
            // count %= 50;
            write_motor();  // Step 4 : Write to motor
        }
    }
    void setmode(int _mode) { motormode = _mode; }
  private:
      float steer_safe(float endpoint) {
        return pc[Stop] + (endpoint - pc[Stop]) * (1.0f - steer_safe_pc * speedo->val() / (100.0f * speedo->opmax()));
    }
    void set_output() {
        float new_out = pc[Out];
        if (motormode == Halt) new_out = pc[Stop];  // Stop the steering motor if in standby mode and standby is complete
        else if (motormode == OpenLoop) {
            int _joydir = hotrc->joydir(Horz);
            if (_joydir == HrcRt)
                new_out = map(hotrc->pc[Horz][Filt], hotrc->pc[Horz][Cent], hotrc->pc[Horz][OpMax], pc[Stop], steer_safe(pc[OpMax]));  // if joy to the right of deadband
            else if (_joydir == HrcLt)
                new_out = map(hotrc->pc[Horz][Filt], hotrc->pc[Horz][Cent], hotrc->pc[Horz][OpMin], pc[Stop], steer_safe(pc[OpMin]));  // if joy to the left of deadband
            else
                new_out = pc[Stop];  // Stop the steering motor if inside the deadband
        }
        new_out = rate_limiter(new_out);  // changerate_limiter();
        cleanzero(&new_out, 0.01f);
        pc[Out] = constrain(new_out, pc[OpMin], pc[OpMax]);
    }
};