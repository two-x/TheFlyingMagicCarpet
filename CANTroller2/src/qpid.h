#pragma once
#ifndef QPID_h
#define QPID_h
#include <stdint.h>
#include "Arduino.h"
#include "utils.h"

class QPID {

  public:

    enum class Control : uint8_t {manual, automatic, timer, toggle};  // controller mode
    enum class Action : uint8_t {direct, reverse};                    // controller action
    enum class pMode : uint8_t {pOnError, pOnMeas, pOnErrorMeas};     // proportional mode
    enum class dMode : uint8_t {dOnError, dOnMeas};                   // derivative mode
    enum class iAwMode : uint8_t {iAwCondition, iAwClamp, iAwOff, iAwRound, iAwRoundCond};    // integral anti-windup mode  // Soren edit
    enum class centMode : uint8_t {range, center, centerStrict};    // Soren - Allows a defined output zero point
    // enum class targSet : uint8_t {immediate, autoMin};    // Soren - Allows us to control target intelligently for throttle idle

    // commonly used functions ************************************************************************************

    // Default constructor
    QPID();

    // Constructor. Links the PID to Input, Output, Setpoint, initial tuning parameters and control modes.
    QPID(float *Input, float *Output, float *Setpoint, float Min, float Max, float Kp, float Ki, float Kd,  // Soren edit
         pMode pMode, dMode dMode, iAwMode iAwMode, Action Action, uint32_t SampleTimeUs, Control Mode, centMode CentMode, float Center);  // Soren edit

    // Constructor allowing use of integer instead of float output value. Soren
    QPID(float *Input, int32_t *Output, float *Setpoint, float Min, float Max, float Kp, float Ki, float Kd,  // Soren
         pMode pMode, dMode dMode, iAwMode iAwMode, Action Action, uint32_t SampleTimeUs, Control Mode, centMode CentMode, float Center);  // Soren

    // Overload constructor links the PID to Input, Output, Setpoint, tuning parameters and control Action.
    // Uses defaults for remaining parameters.
    // QPID(float *Input, float *Output, float *Setpoint, float Kp, float Ki, float Kd, Action Action);

    // Simplified constructor which uses defaults for remaining parameters.
    // QPID(float *Input, float *Output, float *Setpoint);
    
    // Sets PID mode to manual (0), automatic (1), timer (2) or toggle manual/automatic (3).
    void SetMode(Control Mode);
    void SetMode(uint8_t Mode);

    // Performs the PID calculation. It should be called every time loop() cycles ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    bool Compute();

    // Sets and clamps the output to a specific range (0-255 by default).
    void SetOutputLimits(float Min, float Max);

    void SetCentMode(centMode CentMode);  // Soren
    void SetCenter(float Center);  // Soren

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(float Kp, float Ki, float Kd);

    // Overload for specifying proportional ratio.
    void SetTunings(float Kp, float Ki, float Kd, pMode pMode, dMode dMode, iAwMode iAwMode);

    // Soren: I wrote these to facilitate changing only one tuning parameter at a time
    void SetKp(float Kp);
    void SetKi(float Ki);
    void SetKd(float Kd);

    // Sets the controller direction or action. Direct means the output will increase when the error is positive.
    // Reverse means the output will decrease when the error is positive.
    void SetControllerDirection(Action Action);
    void SetControllerDirection(uint8_t Direction);

    // Sets the sample time in microseconds with which each PID calculation is performed. Default is 100000 µs.
    void SetSampleTimeUs(uint32_t NewSampleTimeUs);

    // Sets the computation method for the proportional term, to compute based either on error (default),
    // on measurement, or the average of both.
    void SetProportionalMode(pMode pMode);
    void SetProportionalMode(uint8_t Pmode);

    // Sets the computation method for the derivative term, to compute based either on error or measurement (default).
    void SetDerivativeMode(dMode dMode);
    void SetDerivativeMode(uint8_t Dmode);

    // Sets the integral anti-windup mode to one of iAwClamp, which clamps the output after
    // adding integral and proportional (on measurement) terms, or iAwCondition (default), which
    // provides some integral correction, prevents deep saturation and reduces overshoot.
    // Option iAwOff disables anti-windup altogether.
    void SetAntiWindupMode(iAwMode iAwMode);
    void SetAntiWindupMode(uint8_t IawMode);

    // sets the output summation value
    void SetOutputSum(float sum);

    void Initialize();        // Ensure a bumpless transfer from manual to automatic mode
    void Reset();             // Clears pTerm, iTerm, dTerm and outputSum values

    // PID Query functions ****************************************************************************************
    float GetError();  // Soren added
    float GetKp();            // proportional gain
    float GetKi();            // integral gain
    float GetKd();            // derivative gain
    float GetPterm();         // proportional component of output
    float GetIterm();         // integral component of output
    float GetDterm();         // derivative component of output
    float GetOutputSum();     // summation of all pid term components
    float GetOutputRange();   // Soren - range of output
    float GetOutputMin();     // Soren
    float GetOutputMax();     // Soren
    uint8_t GetMode();        // manual (0), automatic (1), timer (2) or toggle manual/automatic (3)
    uint8_t GetDirection();   // direct (0), reverse (1)
    uint8_t GetPmode();       // pOnError (0), pOnMeas (1), pOnErrorMeas (2)
    uint8_t GetDmode();       // dOnError (0), dOnMeas (1)
    uint8_t GetAwMode();      // iAwCondition (0, iAwClamp (1), iAwOff (2)
    uint8_t GetCentMode();  // Soren
    float GetCenter();  // Soren
    float outputSum;          // Internal integral sum
  private:
    float dispKp = 0;   // for defaults and display
    float dispKi = 0;
    float dispKd = 0;
    float pTerm;
    float iTerm;
    float dTerm;
    float kp;           // (P)roportional Tuning Parameter
    float ki;           // (I)ntegral Tuning Parameter
    float kd;           // (D)erivative Tuning Parameter
    float *myInput;     // Pointers to the Input, Output, and Setpoint variables. This creates a
    float *myOutput;    // hard link between the variables and the PID, freeing the user from having
    float *mySetpoint;  // to constantly tell us what these values are. With pointers we'll just know.
    Control mode = Control::manual;
    Action action = Action::direct;
    pMode pmode = pMode::pOnError;
    dMode dmode = dMode::dOnMeas;
    iAwMode iawmode = iAwMode::iAwCondition;
    centMode centmode = centMode::range;  // Soren
    uint32_t sampleTimeUs, lastTime;
    float outMin, outMax, error, lastError, lastInput;
    float center;  // Soren
    int32_t *myIntOutput;  // Soren
    bool int32_output = false;  // Soren
};  // End of quickpid.h library file. From here down was originally quickpid.cpp

/**********************************************************************************
   QPID Library for Arduino - Version 3.1.9
   by dlloydev https://github.com/Dlloydev/QPID
   Based on the Arduino PID_v1 Library. Licensed under the MIT License.
 **********************************************************************************/

QPID::QPID() {}

// Constructor that allows all parameters to get set
QPID::QPID(float* Input, float* Output, float* Setpoint,
                   float Min, float Max,
                   float Kp = 0, float Ki = 0, float Kd = 0,
                   pMode pMode = pMode::pOnError,
                   dMode dMode = dMode::dOnMeas,
                   iAwMode iAwMode = iAwMode::iAwCondition,
                   Action Action = Action::direct,
                   uint32_t SampleTimeUs = 100000,
                   Control Mode = Control::manual,
                   centMode CentMode = centMode::range, float Center = 1234567) {  // Soren
  int32_output = false;  // Soren
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  mode = Mode;
  QPID::SetOutputLimits(Min, Max);  // same default as Arduino PWM limit - Soren edit
  QPID::SetCentMode(CentMode);  // Soren
  if (centmode != centMode::range && Center != 1234567) {  // Soren
    SetCenter(Center);  // Soren
    outputSum = Center;
  }
  else SetCenter(outMin);  // Soren
  sampleTimeUs = SampleTimeUs;              // Soren edit
  QPID::SetControllerDirection(Action);
  QPID::SetTunings(Kp, Ki, Kd, pMode, dMode, iAwMode);
  lastTime = micros() - sampleTimeUs;  // Soren edit
}

// Constructor allowing use of integer instead of float output value. Soren
QPID::QPID(float* Input, int32_t* IntOutput, float* Setpoint,  // Soren
                   float Min, float Max,
                   float Kp = 0, float Ki = 0, float Kd = 0,
                   pMode pMode = pMode::pOnError,
                   dMode dMode = dMode::dOnMeas,
                   iAwMode iAwMode = iAwMode::iAwCondition,
                   Action Action = Action::direct,
                   uint32_t SampleTimeUs = 100000,
                   Control Mode = Control::manual,
                   centMode CentMode = centMode::range, float Center = 1234567) {  // Soren
  int32_output = true;  // Soren
  myIntOutput = IntOutput;  // Soren
  myInput = Input;
  mySetpoint = Setpoint;
  mode = Mode;
  QPID::SetOutputLimits(Min, Max);  // same default as Arduino PWM limit - Soren edit
  QPID::SetCentMode(CentMode);  // Soren
  if (centmode != centMode::range && Center != 1234567) {  // Soren
    SetCenter(Center);  // Soren
    outputSum = Center;
  }
  else SetCenter(outMin);  // Soren
  sampleTimeUs = SampleTimeUs;              // Soren edit
  QPID::SetControllerDirection(Action);
  QPID::SetTunings(Kp, Ki, Kd, pMode, dMode, iAwMode);
  lastTime = micros() - sampleTimeUs;  // Soren edit
}
/* Compute() ***********************************************************************
   This function should be called every time "void loop()" executes. The function
   will decide whether a new PID Output needs to be computed. Returns true
   when the output is computed, false when nothing has been done.
 **********************************************************************************/
bool QPID::Compute() {
  if (mode == Control::manual) return false;
  uint32_t now = micros();  // Soren edit
  uint32_t timeChange = (now - lastTime);
  if (mode == Control::timer || timeChange >= sampleTimeUs) {

    float input = *myInput;
    float dInput = input - lastInput;
    if (action == Action::reverse) dInput = -dInput;  // Soren

    error = *mySetpoint - input;
    if (action == Action::reverse) error = -error;  // Soren
    float dError = error - lastError;

    float peTerm = kp * error;
    float pmTerm = kp * dInput;
    if (pmode == pMode::pOnError) pmTerm = 0;
    else if (pmode == pMode::pOnMeas) peTerm = 0;
    else { //pOnErrorMeas
      peTerm *= 0.5f;
      pmTerm *= 0.5f;
    }
    pTerm = peTerm - pmTerm;
    iTerm = ki * error;
    if (dmode == dMode::dOnError) dTerm = kd * dError;
    else dTerm = -kd * dInput; // dOnMeas

    if (iawmode == iAwMode::iAwCondition || iawmode == iAwMode::iAwRoundCond) {  // condition anti-windup (default)
      bool aw = false;
      float iTermOut = (peTerm - pmTerm) + ki * (iTerm + error);
      if (iTermOut > outMax && dError > 0) aw = true;
      else if (iTermOut < outMin && dError < 0) aw = true;
      if (aw && ki) iTerm = constrain(iTermOut, -outMax, outMax);
    }
    else if ((iawmode == iAwMode::iAwRound || iawmode == iAwMode::iAwRoundCond) && error < 0.001 && error > -0.001) {
        error = 0.0;
        if (centmode == centMode::centerStrict) outputSum = center;     
    }
    if (centmode == centMode::centerStrict && error * lastError < 0) outputSum = center;  // Soren - Recenters any old integral when error crosses zero
    
    outputSum += iTerm;  // by default, compute output as per PID_v1    // include integral amount
    if (iawmode == iAwMode::iAwOff) outputSum -= pmTerm;                // include pmTerm (no anti-windup)
    else outputSum = constrain(outputSum - pmTerm, outMin, outMax);     // include pmTerm and clamp
    
    if (int32_output) *myIntOutput = (int32_t)(constrain(outputSum + peTerm + dTerm, outMin, outMax));  // Soren
    else *myOutput = constrain(outputSum + peTerm + dTerm, outMin, outMax);  // include dTerm, clamp and drive output

    lastError = error;  // Soren
    lastInput = input;
    lastTime = now;
    return true;
  }
  else return false;  // If class is handling the timing and this time was a nop
}

/* SetTunings(....)************************************************************
  This function allows the controller's dynamic performance to be adjusted.
  it's called automatically from the constructor, but tunings can also
  be adjusted on the fly during normal operation.
******************************************************************************/
void QPID::SetTunings(float Kp, float Ki, float Kd,
                          pMode pMode = pMode::pOnError,
                          dMode dMode = dMode::dOnMeas,
                          iAwMode iAwMode = iAwMode::iAwCondition) {

  if (Kp < 0 || Ki < 0 || Kd < 0 || !sampleTimeUs) return;  // Soren - added divide by zero protection
  if (Ki == 0) outputSum = 0;
  pmode = pMode; dmode = dMode; iawmode = iAwMode;
  dispKp = Kp; dispKi = Ki; dispKd = Kd;
  float SampleTimeSec = (float)sampleTimeUs / 1000000;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
}

/* SetTunings(...)************************************************************
  Set Tunings using the last remembered pMode, dMode and iAwMode settings.
******************************************************************************/
void QPID::SetTunings(float Kp, float Ki, float Kd) {
  SetTunings(Kp, Ki, Kd, pmode, dmode, iawmode);
}

// Soren: I wrote these to facilitate changing only one tuning parameter at a time
void QPID::SetKp(float Kp) { SetTunings(Kp, dispKi, dispKd, pmode, dmode, iawmode); }  // Soren
void QPID::SetKi(float Ki) { SetTunings(dispKp, Ki, dispKd, pmode, dmode, iawmode); }  // Soren
void QPID::SetKd(float Kd) { SetTunings(dispKp, dispKi, Kd, pmode, dmode, iawmode); }  // Soren

/* SetSampleTime(.)***********************************************************
  Sets the period, in microseconds, at which the calculation is performed.
******************************************************************************/
void QPID::SetSampleTimeUs(uint32_t NewSampleTimeUs) {
  if (NewSampleTimeUs > 0 && sampleTimeUs) {  // Soren - added more divide by zero protection
    float ratio  = (float)NewSampleTimeUs / (float)sampleTimeUs;
    ki *= ratio;
    kd /= ratio;
    sampleTimeUs = NewSampleTimeUs;
  }
}

/* SetOutputLimits(..)********************************************************
  The PID controller is designed to vary its output within a given range.
  By default this range is 0-255, the Arduino PWM range.
******************************************************************************/
void QPID::SetOutputLimits(float Min, float Max) {
  if (Min >= Max) return;
  outMin = Min; outMax = Max;

  if (mode != Control::manual) {
    if (int32_output) *myIntOutput = (int32_t)constrain((float)*myIntOutput, outMin, outMax);  // Soren
    else *myOutput = constrain(*myOutput, outMin, outMax);  // Soren
    outputSum = constrain(outputSum, outMin, outMax);
  }
}

void QPID::SetCentMode(centMode CentMode) { centmode = CentMode; }  // Soren
void QPID::SetCenter(float Center) { if (outMin <= Center && outMax >= Center) center = Center; }  // Soren
// if (centmode == centMode::range) centmode = centMode::centerStrict;  // Soren - does definition of center imply to use center mode?

/* SetMode(.)*****************************************************************
  Sets the controller mode to manual (0), automatic (1) or timer (2)
  when the transition from manual to automatic or timer occurs, the
  controller is automatically initialized.
******************************************************************************/
void QPID::SetMode(Control Mode) {
  if (mode == Control::manual && Mode != Control::manual) { // just went from manual to automatic, timer or toggle
    QPID::Initialize();
  }
  if (Mode == Control::toggle) {
    mode = (mode == Control::manual) ? Control::automatic : Control::manual;
  } else  mode = Mode;
}
void QPID::SetMode(uint8_t Mode) {
  if (mode == Control::manual && Mode != 0) { // just went from manual to automatic or timer
    QPID::Initialize();
  }
  if (Mode == 3) { // toggle
    mode = (mode == Control::manual) ? Control::automatic : Control::manual;
  } else  mode = (Control)Mode;
}

/* Initialize()****************************************************************
  Does all the things that need to happen to ensure a bumpless transfer
  from manual to automatic mode.
******************************************************************************/
void QPID::Initialize() {
  outputSum = (float)*myOutput;  // Soren
  lastInput = *myInput;
  outputSum = constrain(outputSum, outMin, outMax);
}

/* SetControllerDirection(.)**************************************************
  The PID will either be connected to a direct acting process (+Output leads
  to +Input) or a reverse acting process(+Output leads to -Input).
******************************************************************************/
void QPID::SetControllerDirection(Action Action) {
  action = Action;
}
void QPID::SetControllerDirection(uint8_t Direction) {
  action = (Action)Direction;
}

/* SetProportionalMode(.)*****************************************************
  Sets the computation method for the proportional term, to compute based
  either on error (default), on measurement, or the average of both.
******************************************************************************/
void QPID::SetProportionalMode(pMode pMode) {
  pmode = pMode;
}
void QPID::SetProportionalMode(uint8_t Pmode) {
  pmode = (pMode)Pmode;
}

/* SetDerivativeMode(.)*******************************************************
  Sets the computation method for the derivative term, to compute based
  either on error or on measurement (default).
******************************************************************************/
void QPID::SetDerivativeMode(dMode dMode) {
  dmode = dMode;
}
void QPID::SetDerivativeMode(uint8_t Dmode) {
  dmode = (dMode)Dmode;
}

/* SetAntiWindupMode(.)*******************************************************
  Sets the integral anti-windup mode to one of iAwClamp, which clamps
  the output after adding integral and proportional (on measurement) terms,
  or iAwCondition (default), which provides some integral correction, prevents
  deep saturation and reduces overshoot.
  Option iAwOff disables anti-windup altogether.
******************************************************************************/
void QPID::SetAntiWindupMode(iAwMode iAwMode) {
  iawmode = iAwMode;
}
void QPID::SetAntiWindupMode(uint8_t IawMode) {
  iawmode = (iAwMode)IawMode;
}

void QPID::Reset() {
  lastTime = micros() - sampleTimeUs;  // Soren edit
  lastInput = 0;
  outputSum = 0;
  pTerm = 0;
  iTerm = 0;
  dTerm = 0;
}

// sets the output summation value
void QPID::SetOutputSum(float sum) {
  outputSum = sum;
}

/* Status Functions************************************************************
  These functions query the internal state of the PID.
******************************************************************************/
float QPID::GetError() { return error; }  // Soren
float QPID::GetKp() { return dispKp; }
float QPID::GetKi() { return dispKi; }
float QPID::GetKd() { return dispKd; }
float QPID::GetPterm() { return pTerm; }
float QPID::GetIterm() { return iTerm; }
float QPID::GetDterm() { return dTerm; }
float QPID::GetOutputSum() { return outputSum; }
float QPID::GetOutputMin() { return outMin; }  // Soren
float QPID::GetOutputMax() { return outMax; }  // Soren
float QPID::GetOutputRange() { return outMax - outMin; }  // Soren
float QPID::GetCenter() { return center; }  // Soren
uint8_t QPID::GetMode() { return static_cast<uint8_t>(mode); }
uint8_t QPID::GetDirection() { return static_cast<uint8_t>(action); }
uint8_t QPID::GetPmode() { return static_cast<uint8_t>(pmode); }
uint8_t QPID::GetDmode() { return static_cast<uint8_t>(dmode); }
uint8_t QPID::GetAwMode() { return static_cast<uint8_t>(iawmode); }
uint8_t QPID::GetCentMode() { return static_cast<uint8_t>(centmode); }  // Soren

class IdleControl {  // Soren - To allow creative control of PID targets in case your engine idle problems need that.
  public:
    enum class idlemodes : uint32_t { direct, control, minimize };  // direct: disable idle management.  control: soft landing to idle rpm.  minimize: attempt to minimize idle to edge of instability
  protected:
    enum targetstates : uint32_t { driving, droptohigh, droptolow, stallpoint };
    // String modenames[3] = { "direct", "cntrol", "minimz" };
    // String statenames[4] = { "drivng", "tohigh", "tolow", "tostal" };
    targetstates runstate, laststate;
    idlemodes idlemode;
    float targetlast_rpm, idle_rpm, idlehigh_rpm, idlehot_rpm, idlecold_rpm, stallpoint_rpm, dynamic_rpm, temphot_f, tempcold_f;
    float margin_rpm = 10; float* target_rpm; float* measraw_rpm; float* measfilt_rpm; float* coolant_f;
    float idle_absmax_rpm = 1000.0;  // High limit of idle speed adjustability
    bool we_just_changed_states = true;
    uint32_t settletime_us;
    Timer settleTimer;
  public:
    IdleControl (float* target_rpm, float* measraw_rpm, float* measfilt_rpm, float* coolant_f,  // Variable references: idle target, rpm raw, rpm filt, Engine temp
      float idlehigh, float idlehot, float idlecold,  // Values for: high-idle rpm (will not stall), hot idle nominal rpm, cold idle nominal rpm 
      float tempcold, float temphot,  // Values for: engine operational temp cold (min) and temp hot (max) in degrees-f
      uint32_t settletime_us = 2000000,  // Period over which the idle will be lowered from high-idle to final idle
      idlemodes idlemode = idlemodes::control) {  // Configure idle control to just soft land or also attempt to minimize idle
        set_idlehigh (idlehigh);
        set_idlehot (idlehot);
        stallpoint_rpm = idlehot_rpm;
        set_idlecold (idlecold);
        set_temphot (temphot);
        set_tempcold (tempcold);        
        calc_idlespeed();
        targetlast_rpm = *target_rpm;
        settleTimer.set ((int64_t)settletime_us);  // Time period over which the idle will subsequently drop further to its lower temperature-dependent target value
        runstate = driving;
    }
    void update (void) {  // this should be called to update idle and throttle target values before throttle-related control loop outputs are calculated
        calc_idlespeed();  // determine our appropriate idle speed, based on latest engine temperature reading
        if (runstate == driving) process_driving();  // while throttle is open when driving, we don't mess with the rpm target value
        else if (runstate == droptohigh) process_droptohigh();  // once the metaphorical foot is taken off the gas, first we let the carb close quickly to a high-idle rpm level (that won't stall)
        else if (runstate == droptolow) process_droptolow();  // after the rpm hits the high-idle level, we slowly close the throttle further until we reach the correct low-idle speed for the current engine temperature
        else if (runstate == stallpoint) process_stallpoint();  // if idlemode == activemin, we then further allow the idle to drop, until we begin to sense irregularity in our rpm sensor pulses
        runstate_changer();  // if runstate was changed, prepare to run any initial actions upon processing our new runstate algorithm
    }
    void calc_idlespeed (void) {
        idle_rpm = map (*coolant_f, tempcold_f, temphot_f, idlecold_rpm, idlehot_rpm);
        idle_rpm = constrain (idle_rpm, idlehot_rpm, idlecold_rpm);
    }
    void goto_idle (void) {  // The gods request the engine should idle now
        if (runstate == driving && idlemode != idlemodes::) runstate = droptohigh;
    }
    void runstate_changer (void) {
        we_just_changed_states = (runstate != laststate);
        laststate = runstate;
    }
    void process_driving (void) {
        if (*target_rpm < idle_rpm) *target_rpm = idle_rpm;
    }
    void process_droptohigh (void) {
        if (*target_rpm > idlehigh_rpm) runstate = driving;
        else if (*measfilt_rpm > idlehigh_rpm + margin_rpm) *target_rpm = idlehigh_rpm;
        else runstate = droptolow;
    }
    void process_droptolow (void) {
        if (we_just_changed_states) {
            dynamic_rpm = *measfilt_rpm - idle_rpm;
            settleTimer.reset();
        }
        if (*target_rpm > idlehigh_rpm) runstate = driving;
        else if (*measfilt_rpm <= idle_rpm + margin_rpm && idlemode == idlemodes::minimize) runstate = stallpoint;
        else if (settleTimer.expired()) *target_rpm = idle_rpm;
        else {
            float addtarg_rpm = dynamic_rpm * (1 - (float)settleTimer.elapsed()/(float)settletime_us);
            *target_rpm = idle_rpm + (addtarg_rpm > 0.0) ? addtarg_rpm : 0.0;  // because for some dumb reason max() doesn't do jack smack
        }
    }
    void process_stallpoint (void) {
        if (we_just_changed_states) {
            stallpoint_rpm = idle_rpm;
            dynamic_rpm = stallpoint_rpm;
        }
        if (*target_rpm > idlehigh_rpm) runstate = driving;
        // else if (*measfilt_rpm > )
        // Soren finish writing this
    }
    void calc_idle_stability (void) {
        // This function does bullocks Soren
    }
    // String get_modename (void) { return modenames[(int32_t)idlemode].c_str(); }
    // String get_statename (void) { return statenames[runstate].c_str(); }
    void set_settletime (uint32_t new_settletime_us) {
        if (new_settletime_us) settletime_us = new_settletime_us;
        settleTimer.set ((int64_t)settletime_us);
    }
    void next_idlemode (void) { idlemode = (idlemode == direct) ? control : (idlemode == control) ? minimize : direct; }
    void set_idlemode (idlemodes idlemode) {} 
    void set_idlehigh (float idlehigh_rpm, float add = 0.0) { 
        idlehigh_rpm = constrain (idlehigh_rpm + add, idlecold_rpm + 0.1, idle_absmax_rpm);
    }
    void set_idlehot (float idlehot_rpm, float add = 0.0) { 
        idlehot_rpm = constrain (idlehot_rpm + add, stallpoint_rpm, idlecold_rpm - 0.1);
        calc_idlespeed();
    }
    void set_idlecold (float idlecold_rpm, float add = 0.0) { 
        idlecold_rpm = constrain (idlecold_rpm + add, idlehot_rpm + 0.1, idlehigh_rpm - 0.1);
        calc_idlespeed();
    }
    void set_temphot (float temphot_f, float add = 0.0) { 
        if (temphot_f + add > tempcold_f) temphot_f += add;
        calc_idlespeed();
    }
    void set_tempcold (float tempcold, float add = 0.0) { 
        if (tempcold_f + add < temphot_f) tempcold_f += add;
        calc_idlespeed();
    }
    void set_margin (float margin_rpm) {}
    targetstates get_targetstate (void) { return runstate; } 
    idlemodes get_idlemode (void) { return idlemode; } 
    uint32_t get_settletime (void) { return settletime_us; }
    float get_idlehigh (void) { return idlehigh_rpm; }
    float get_idlehot (void) { return idlehot_rpm; }
    float get_idlecold (void) { return idlecold_rpm; }
    float get_temphot (void) { return temphot_f; }
    float get_tempcold (void) { return tempcold_f; }
    float get_idlespeed (void) { return idle_rpm; }
    float get_margin (void) { return margin_rpm; }
    float get_stallpoint (void) { return stallpoint_rpm; }
};
#endif // QPID.h