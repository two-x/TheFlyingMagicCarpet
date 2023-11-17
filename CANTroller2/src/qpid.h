#pragma once
// #include <stdint.h>
// I stole this library and modified it heavily to our purpposes - Soren
// QPID Library for Arduino - Version 3.1.9 by dlloydev https://github.com/Dlloydev/QPID
// Based on the Arduino PID_v1 Library. Licensed under the MIT License.
class QPID {
  public:
    enum class ctrl : int {manual, automatic, toggle};            // controller mode
    enum class cdir : int {direct, reverse};                      // controller direction
    enum class pmod : int {onerr, onmeas, onerrmeas};             // proportional mode
    enum class dmod : int {onerr, onmeas};                        // derivative mode
    enum class awmod : int {cond, clamp, off, round, roundcond};  // integral anti-windup mode  // Soren edit
    enum class centmod : int {off, on, strict};                   // Soren - Allows a defined output zero point
  private:
    float dispkp = 0; float dispki = 0; float dispkd = 0;
    float _pterm, _iterm, _dterm, _kp, _ki, _kd, _err, lasterr, lastin, _cent, _outsum, _target, _output;
    float *myin;     // Pointers to the input, output, and target variables. This creates a
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
        uint32_t a_sampletime = 100000, ctrl a_mode = ctrl::manual, centmod a_centmode = centmod::off, float a_cent = NAN);
    void set_mode(ctrl a_mode);
    void set_mode(int a_mode);
    float compute();  // Performs the PID calculation
    void set_tunings(float a_kp, float a_ki, float a_kd);
    void set_tunings(float a_kp, float a_ki, float a_kd, pmod a_pmode, dmod a_dmode, awmod a_awmode);
    void set_sampletime(uint32_t a_sampletime);  // in microseconds
    void init();        // Ensure a bumpless transfer from manual to automatic mode
    void reset();             // Clears _pterm, _iterm, _dterm and _outsum values
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
void QPID::init(float* a_in, float* a_min, float* a_max, float a_kp, float a_ki, float a_kd, pmod a_pmode, dmod a_dmode,
  awmod a_awmode, cdir a_dir, uint32_t a_sampletime, ctrl a_mode, centmod a_centmode, float a_cent) {  // Soren edit
    myin = a_in;
    _mode = a_mode;
    _outmin = a_min;
    _outmax = a_max;
    _output = constrain(_output, *_outmin, *_outmax);
    _outsum = constrain(_outsum, *_outmin, *_outmax);
    QPID::set_centmode(a_centmode);  // Soren
    if (_centmode != centmod::off && !std::isnan(a_cent)) {  // Soren
        set_cent(a_cent);  // Soren
        _outsum = _cent;
    }
    else set_cent(*_outmin);  // Soren
    sampletime = a_sampletime;              // Soren edit
    QPID::set_dir(a_dir);
    QPID::set_tunings(a_kp, a_ki, a_kd, _pmode, _dmode, _awmode);
    lasttime = micros() - sampletime;  // Soren edit
}
// This function should be called every time "void loop()" executes. The function will decide whether a new 
// PID output needs to be computed. Returns true when the output is computed, false when nothing has been done.
float QPID::compute() {
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
void QPID::set_tunings(float a_kp, float a_ki, float a_kd, pmod a_pmode = pmod::onerr,
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
void QPID::set_tunings(float a_kp, float a_ki, float a_kd) {
    set_tunings(a_kp, a_ki, a_kd, _pmode, _dmode, _awmode);
}
// set_sampletime  Sets the period, in microseconds, at which the calculation is performed.
void QPID::set_sampletime(uint32_t a_sampletime) {
    if (a_sampletime > 0 && sampletime) {  // Soren - added more divide by zero protection
        float ratio  = (float)a_sampletime / (float)sampletime;
        _ki *= ratio;
        _kd /= ratio;
        sampletime = a_sampletime;
    }
}
// set_mode Sets the controller mode to manual (0), automatic (1) or timer (2) when the transition 
// from manual to automatic or timer occurs, the controller is automatically initialized.
void QPID::set_mode(ctrl a_mode) {
    if (_mode == ctrl::manual && a_mode != ctrl::manual) { // just went from manual to automatic
        _mode = ctrl::automatic;
        QPID::init();
    }
    else if (_mode == ctrl::automatic && a_mode != ctrl::automatic) {
        _mode = ctrl::manual;
    }
}
void QPID::set_mode(int a_mode) { set_mode((ctrl)a_mode); }
// Does all the things that need to happen to ensure a bumpless transfer from manual to automatic mode.
void QPID::init() {
    _outsum = _output;  // Soren
    lastin = *myin;
    _outsum = constrain(_outsum, *_outmin, *_outmax);
}
void QPID::reset() {
    lasttime = micros() - sampletime;  // Soren edit
    lastin = 0; _outsum = 0;
    _pterm = 0; _iterm = 0; _dterm = 0;
}