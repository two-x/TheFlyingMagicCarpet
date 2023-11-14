#pragma once
// #include <stdint.h>
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
    float _pterm, _iterm, _dterm, _kp, _ki, _kd, _outmin, _outmax, _err, lasterr, lastin, _cent, _outsum, _target, _output;
    float *myin;     // Pointers to the input, output, and target variables. This creates a
    ctrl _mode = ctrl::manual;
    cdir _dir = cdir::direct;
    pmod _pmode = pmod::onerr;
    dmod _dmode = dmod::onmeas;
    awmod _awmode = awmod::cond;
    centmod _centmode = centmod::off;  // Soren
    uint32_t sampletime, lasttime;
  public:
    qpid() {}  // Default constructor
    qpid(float* __in, float __min, float __max, float __kp = 0, float __ki = 0, float __kd = 0,  // Soren edit
      pmod __pmode = pmod::onerr, dmod __dmode = dmod::onmeas, awmod __awmode = awmod::cond, cdir __dir = cdir::direct,
      uint32_t __sampletime = 100000, ctrl __mode = ctrl::manual, centmod __centmode = centmod::off, float __cent = NAN) {
        init(__in, __min, __max, __kp, __ki, __kd, __pmode, __dmode, __awmode, __dir, __sampletime, __mode, __centmode, __cent);
    }
    void init(float* __in, float __min, float __max, float __kp = 0, float __ki = 0, float __kd = 0,  // Soren edit
        pmod __pmode = pmod::onerr, dmod __dmode = dmod::onmeas, awmod __awmode = awmod::cond, cdir __dir = cdir::direct,
        uint32_t __sampletime = 100000, ctrl __mode = ctrl::manual, centmod __centmode = centmod::off, float __cent = NAN);
    void set_mode(ctrl __mode);
    void set_mode(uint8_t __mode);
    float compute();  // Performs the PID calculation
    void set_outlimits(float __min, float __max);  // Sets and clamps the output to a specific range (0-255 by default).
    void set_centmode(centmod __centmode);  // Soren
    void set_centmode(uint8_t __centmode);  // Soren
    void set_cent(float __cent);  // Soren
    void set_tunings(float __kp, float __ki, float __kd);
    void set_tunings(float __kp, float __ki, float __kd, pmod __pmode, dmod __dmode, awmod __awmode);
    void set_kp(float __kp);
    void set_ki(float __ki);
    void set_kd(float __kd);
    void add_kp(float __kp);
    void add_ki(float __ki);
    void add_kd(float __kd);
    void set_dir(cdir __dir);
    void set_dir(uint8_t __dir);
    void set_sampletime(uint32_t __sampletime);  // in microseconds
    void set_pmode(pmod __pmode);  // Sets the computation method for the proportional term, to compute based either on error (default), on measurement, or the average of both.
    void set_pmode(uint8_t __pmode);
    void set_dmode(dmod __dmode);  // Sets the computation method for the derivative term, to compute based either on error or measurement (default).
    void set_dmode(uint8_t __dmode);
    void set_awmode(awmod __awmode);  // Sets the integral anti-windup mode to one of awClamp, which clamps the output after adding integral and proportional (on measurement) terms, or awcond (default), which provides some integral correction, prevents deep saturation and reduces overshoot. Option awOff disables anti-windup altogether.
    void set_awmode(uint8_t __awmode);
    void set_outsum(float __outsum);  // sets the output summation value
    void set_target(float __target);
    void set_output(float __output);
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
    float target() { return _target; }  // Soren
    float output() { return _output; }  // Soren
    uint8_t mode() { return static_cast<uint8_t>(_mode); }
    uint8_t dir() { return static_cast<uint8_t>(_dir); }
    uint8_t pmode() { return static_cast<uint8_t>(_pmode); }
    uint8_t dmode() { return static_cast<uint8_t>(_dmode); }
    uint8_t awmode() { return static_cast<uint8_t>(_awmode); }
    uint8_t centmode() { return static_cast<uint8_t>(_centmode); }  // Soren
    float* target_ptr() { return &_target; }
};
void qpid::init(float* __in, float __min, float __max, float __kp, float __ki, float __kd, pmod __pmode, dmod __dmode,
  awmod __awmode, cdir __dir, uint32_t __sampletime, ctrl __mode, centmod __centmode, float __cent) {  // Soren edit
    myin = __in;
    _mode = __mode;
    qpid::set_outlimits(__min, __max);  // same default as Arduino PWM limit - Soren edit
    qpid::set_centmode(__centmode);  // Soren
    if (_centmode != centmod::off && !std::isnan(__cent)) {  // Soren
        set_cent(__cent);  // Soren
        _outsum = _cent;
    }
    else set_cent(_outmin);  // Soren
    sampletime = __sampletime;              // Soren edit
    qpid::set_dir(__dir);
    qpid::set_tunings(__kp, __ki, __kd, _pmode, _dmode, _awmode);
    lasttime = micros() - sampletime;  // Soren edit
}

// This function should be called every time "void loop()" executes. The function will decide whether a new 
// PID output needs to be computed. Returns true when the output is computed, false when nothing has been done.
float qpid::compute() {
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

    _output = constrain(_outsum + peterm + _dterm, _outmin, _outmax);  // include _dterm, clamp and drive output

    lasterr = _err;  // Soren
    lastin = in;
    lasttime = now;
    return _output;
}

// set_tunings  This function allows the controller's dynamic performance to be adjusted. It's called 
// automatically from the constructor, but tunings can also be adjusted on the fly during normal operation.
void qpid::set_tunings(float __kp, float __ki, float __kd, pmod __pmode = pmod::onerr,
    dmod __dmode = dmod::onmeas, awmod __awmode = awmod::cond) {
    if (__kp < 0 || __ki < 0 || __kd < 0 || !sampletime) return;  // Soren - added divide by zero protection
    if (__ki == 0) _outsum = 0;
    _pmode = __pmode; _dmode = __dmode; _awmode = __awmode;
    dispkp = __kp; dispki = __ki; dispkd = __kd;
    float sampletime_sec = (float)sampletime / 1000000;
    _kp = __kp;
    _ki = __ki * sampletime_sec;
    _kd = __kd / sampletime_sec;
}

// set_tunings  Set Tunings using the last remembered pmode, dmode and awmode settings.
void qpid::set_tunings(float __kp, float __ki, float __kd) {
    set_tunings(__kp, __ki, __kd, _pmode, _dmode, _awmode);
}

// Soren: I wrote these to facilitate changing only one tuning parameter at a time
void qpid::set_kp(float __kp) { set_tunings(__kp, dispki, dispkd, _pmode, _dmode, _awmode); }  // Soren
void qpid::set_ki(float __ki) { set_tunings(dispkp, __ki, dispkd, _pmode, _dmode, _awmode); }  // Soren
void qpid::set_kd(float __kd) { set_tunings(dispkp, dispki, __kd, _pmode, _dmode, _awmode); }  // Soren

void qpid::add_kp(float add) { set_kp(_kp + add); }  // Soren
void qpid::add_ki(float add) { set_ki(_ki + add); }  // Soren
void qpid::add_kd(float add) { set_kd(_kd + add); }  // Soren

// set_sampletime  Sets the period, in microseconds, at which the calculation is performed.
void qpid::set_sampletime(uint32_t __sampletime) {
    if (__sampletime > 0 && sampletime) {  // Soren - added more divide by zero protection
        float ratio  = (float)__sampletime / (float)sampletime;
        _ki *= ratio;
        _kd /= ratio;
        sampletime = __sampletime;
    }
}

// set_outLimits  The PID controller is designed to vary its output within a given range, default 0-255
void qpid::set_outlimits(float __min, float __max) {
    if (__min >= __max) return;
    _outmin = __min; _outmax = __max;

    // if (_mode != ctrl::manual) {
    _output = constrain(_output, _outmin, _outmax);  // Soren
    _outsum = constrain(_outsum, _outmin, _outmax);
}

void qpid::set_centmode(centmod __centmode) { _centmode = __centmode; }  // Soren
void qpid::set_centmode(uint8_t __centmode) { _centmode = (centmod)__centmode; }  // Soren
void qpid::set_cent(float __cent) { if (_outmin <= __cent && _outmax >= __cent) _cent = __cent; }  // Soren
void qpid::set_target(float __target) { _target = __target; }
void qpid::set_output(float __output) { _output = constrain(__output, _outmin, _outmax); }

// set_mode Sets the controller mode to manual (0), automatic (1) or timer (2) when the transition 
// from manual to automatic or timer occurs, the controller is automatically initialized.
void qpid::set_mode(ctrl __mode) {
    if (_mode == ctrl::manual && __mode != ctrl::manual) { // just went from manual to automatic
        _mode = ctrl::automatic;
        qpid::init();
    }
    else if (_mode == ctrl::automatic && __mode != ctrl::automatic) {
        _mode = ctrl::manual;
    }
}
void qpid::set_mode(uint8_t __mode) { set_mode((ctrl)__mode); }

// Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.
void qpid::init() {
    _outsum = _output;  // Soren
    lastin = *myin;
    _outsum = constrain(_outsum, _outmin, _outmax);
}

// The PID will either be connected to a direct acting process (+output leads to +input) or a reverse acting process(+output leads to -input).
void qpid::set_dir(cdir __dir) { _dir = __dir; }
void qpid::set_dir(uint8_t __dir) { _dir = (cdir)__dir; }

// Sets the computation method for the proportional term, to compute based either on error (default), on measurement, or the average of both.
void qpid::set_pmode(pmod __pmode) { _pmode = __pmode; }
void qpid::set_pmode(uint8_t __pmode) { _pmode = (pmod)__pmode; }

// Sets the computation method for the derivative term, to compute based either on error or on measurement (default).
void qpid::set_dmode(dmod __dmode) { _dmode = __dmode; }
void qpid::set_dmode(uint8_t __dmode) { _dmode = (dmod)__dmode; }

// Sets the integral anti-windup mode to one of clamp, which clamps the output after adding integral and proportional (on measurement) terms,
// or cond (default), which provides some integral correction, prevents deep saturation and reduces overshoot. Option off disables anti-windup altogether.
void qpid::set_awmode(awmod __awmode) { _awmode = __awmode; }
void qpid::set_awmode(uint8_t __awmode) { _awmode = (awmod)__awmode; }

void qpid::reset() {
    lasttime = micros() - sampletime;  // Soren edit
    lastin = 0; _outsum = 0;
    _pterm = 0; _iterm = 0; _dterm = 0;
}
// sets the output summation value
void qpid::set_outsum(float __outsum) { _outsum = __outsum; }
