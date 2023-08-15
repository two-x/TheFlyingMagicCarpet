#pragma once
#ifndef QPID_h
#define QPID_h
#include <stdint.h>
#include "Arduino.h"
#include "utils.h"
#include "TemperatureSensor.h"

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
    
    outputSum += iTerm - pmTerm;  // by default, compute output as per PID_v1    // include integral amount and pmTerm
    if (iawmode != iAwMode::iAwOff) outputSum = constrain(outputSum, outMin, outMax);  // Clamp
    
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

class ThrottleControl {  // Soren - To allow creative control of PID targets in case your engine idle problems need that.
  public:
    enum class idlemodes : uint32_t { direct, control, minimize, num_idlemodes };  // direct: disable idle management.  control: soft landing to idle rpm.  minimize: attempt to minimize idle to edge of instability
    enum targetstates : uint32_t { todrive, driving, droptohigh, droptolow, idling, minimizing, num_states };
  protected:
    // String modenames[3] = { "direct", "cntrol", "minimz" };
    // String statenames[4] = { "drivng", "tohigh", "tolow", "tostal" };
    targetstates runstate, nextstate;
    idlemodes idlemode;
    float target_rpm, targetlast_rpm, idle_rpm, idlehigh_rpm, idlehot_rpm, idlecold_rpm, stallpoint_rpm, dynamic_rpm, temphot_f, tempcold_f, idle_slope_rpmps;
    float margin_rpm = 10; float idle_absmax_rpm = 1000.0;  // High limit of idle speed adjustability
    float* measraw_rpm; float* measfilt_rpm; float engine_temp_f;
    TemperatureSensor* engine_sensor = nullptr;
    bool we_just_changed_states = true; bool target_externally_set = false; // bool now_trying_to_idle = false;
    uint32_t settlerate_rpmps, index_now, index_last;
    uint32_t stallrate_rpmps = 400;  // Engine rpm drops exceeding this much per second are considered a stall in progress
    uint32_t history_depth = 20;
    int32_t tach_history_rpm[20];  // Why can't I use [history_depth] here instead of [20] in this instantiation?  c++ is a pain in my ass
    uint32_t timestamps_us[20];
    Timer settleTimer, tachHistoryTimer;
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
        idlemode = myidlemode;
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
        if (engine_sensor) {
            engine_temp_f = engine_sensor->get_temperature();
        }
        calc_tach_stability();
        calc_idlespeed();  // determine our appropriate idle speed, based on latest engine temperature reading
        runstate_changer();  // if runstate was changed, prepare to run any initial actions upon processing our new runstate algorithm
        if (runstate == todrive) process_todrive();  // Target is above idle, but currently engine is still idling 
        else if (runstate == driving) process_driving();  // while throttle is open when driving, we don't mess with the rpm target value
        else if (runstate == droptohigh) process_droptohigh();  // once the metaphorical foot is taken off the gas, first we let the carb close quickly to a high-idle rpm level (that won't stall)
        else if (runstate == droptolow) process_droptolow();  // after the rpm hits the high-idle level, we slowly close the throttle further until we reach the correct low-idle speed for the current engine temperature
        else if (runstate == idling) process_idling();  // maintain the low-idle level, adjusting to track temperature cchanges as appropriate
        else if (runstate == minimizing) process_minimizing();  // if idlemode == minimize, we then further allow the idle to drop, until we begin to sense irregularity in our rpm sensor pulses
    }
    void goto_idle (void) {  // The gods request the engine should idle now
        if (runstate == driving) nextstate = (idlemode == idlemodes::direct) ? droptolow : droptohigh;
        // now_trying_to_idle = true;
    }
    void push_tach_reading (int32_t reading) {  // Add a new rpm reading to a small LIFO ring buffer. We will use this to detect arhythmic rpm
        if (reading == tach_history_rpm[index_now]) return;  // Ignore new tach values unless rpm has changed
        index_last = index_now;
        index_now = (index_now + 1) % history_depth;
        tach_history_rpm[index_now] = reading;
        timestamps_us[index_now] = (uint32_t)tachHistoryTimer.elapsed();
        tachHistoryTimer.reset();
    }
    void set_target (float argtarget) {
        if ((int32_t)target_rpm != (int32_t)argtarget) {
            target_externally_set = true;
            targetlast_rpm = target_rpm;
            target_rpm = argtarget;
        }
    }
    void set_engine_sensor (TemperatureSensor& sensor) {
      engine_sensor = &sensor;
    }
  protected:
    void set_target_internal (float argtarget) {
        if ((int32_t)target_rpm != (int32_t)argtarget) {
            target_externally_set = false;
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
            if (target_rpm > idlehigh_rpm) nextstate = (*measfilt_rpm > idlehigh_rpm) ? driving : todrive;
            else if (target_rpm > idle_rpm + margin_rpm) nextstate = (idlemode == idlemodes::minimize) ? minimizing : idling;
            // now_trying_to_idle = false;
        }
        target_externally_set = false;
        we_just_changed_states = (nextstate != runstate);
        runstate = nextstate;
    }
    void process_todrive (void) {
        if (we_just_changed_states) { printf("todriv "); }
        else if (*measfilt_rpm > idlehigh_rpm) nextstate = driving;
    }
    void process_driving (void) {
        if (we_just_changed_states) { printf("driving "); }
    }
    void process_droptohigh (void) {
        if (we_just_changed_states) { set_target_internal (idlehigh_rpm); printf("droptohigh "); }
        else if (*measfilt_rpm <= idlehigh_rpm + margin_rpm) nextstate = droptolow;  // Done dropping to high idle, next continue dropping to low idle
    }
    void process_droptolow (void) {
        if (we_just_changed_states) { settleTimer.reset(); printf("droptolow "); }
        if (*measfilt_rpm <= idle_rpm + margin_rpm) nextstate = (idlemode == idlemodes::minimize) ? minimizing : idling;  // Done dropping to low idle, next proceed to a steady state
        else {  // Drop from current rpm toward low idle speed at configured rate
            set_target_internal (*measfilt_rpm - settlerate_rpmps * (float)settleTimer.elapsed()/1000000);  // Need to review the dynamics of this considering update frequency and motor latency 
            settleTimer.reset();
        }
    }
    void process_idling (void) {  // If we aren't set to attempt to minimize idle speed, then we end up here
            if (we_just_changed_states) {       printf("idling "); }

        if (idlemode == idlemodes::minimize) nextstate = minimizing;  // In case idlemode is changed while in idling state
        set_target_internal (idle_rpm);  // We're done dropping to the idle point, but keep tracking as idle speed may change
    }
    void process_minimizing (void) {
        if (we_just_changed_states) { stallpoint_rpm = idle_rpm; printf("minimizing "); }
        else if (idlemode != idlemodes::minimize) nextstate = idling;  // In case idlemode is changed while in stallpoint state
        // else if (*measfilt_rpm > )
        // Soren finish writing this
    }
    void calc_tach_stability (void) {
        idle_slope_rpmps = (float)(tach_history_rpm[index_now] - tach_history_rpm[index_last]) * 1000000 / timestamps_us[index_now];
        // if (idle_slope_rpmps < stallrate_rpmps) 
        // Soren finish writing this.  So close!
    }
    // String get_modename (void) { return modenames[(int32_t)idlemode].c_str(); }
    // String get_statename (void) { return statenames[runstate].c_str(); }
  public:
    void cycle_idlemode (int32_t cycledir) {  // Cycldir positive or negative
        if (cycledir) idlemode = (idlemodes)(constrain ((int32_t)idlemode + constrain (cycledir, -1, 1), 0, (int32_t)idlemodes::num_idlemodes - 1));
    }
    void set_idlemode (idlemodes idlemode) {} 
    void set_idlehigh (float idlehigh, float add = 0.0) { idlehigh_rpm = constrain (idlehigh + add, idlecold_rpm + 1, idle_absmax_rpm); }
    void set_idlehot (float idlehot, float add = 0.0) { 
        idlehot_rpm = constrain (idlehot + add, stallpoint_rpm, idlecold_rpm - 1);
        calc_idlespeed();
    }
    void set_idlecold (float idlecold, float add = 0.0) { 
        idlecold_rpm = constrain (idlecold + add, idlehot_rpm + 1, idlehigh_rpm - 1);
        calc_idlespeed();
    }
    void set_temphot (float temphot, float add = 0.0) { 
        if (temphot + add > tempcold_f) temphot_f = temphot + add;
        calc_idlespeed();
    }
    void set_tempcold (float tempcold, float add = 0.0) { 
        if (tempcold + add < temphot_f) tempcold_f = tempcold + add;
        calc_idlespeed();
    }
    void set_settlerate (uint32_t settlerate) { if (settlerate) settlerate_rpmps = settlerate; }
    void set_margin (float margin) { margin_rpm = margin; }
    targetstates get_targetstate (void) { return runstate; } 
    idlemodes get_idlemode (void) { return idlemode; } 
    uint32_t get_settlerate (void) { return settlerate_rpmps; }
    float get_idlehigh (void) { return idlehigh_rpm; }
    float get_idlehot (void) { return idlehot_rpm; }
    float get_idlecold (void) { return idlecold_rpm; }
    float get_temphot (void) { return temphot_f; }
    float get_tempcold (void) { return tempcold_f; }
    float get_idlespeed (void) { return idle_rpm; }
    float get_margin (void) { return margin_rpm; }
    float get_stallpoint (void) { return stallpoint_rpm; }
    float get_target (void) { return target_rpm; }
};
#endif // QPID.h