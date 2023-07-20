#include "qpid.h"
#include <Arduino.h>

/**********************************************************************************
   QPID Library for Arduino - Version 3.1.9
   by dlloydev https://github.com/Dlloydev/QPID
   Based on the Arduino PID_v1 Library. Licensed under the MIT License.
 **********************************************************************************/

QPID::QPID() {}

// Constructor that allows all parameters to get set
QPID::QPID(float* Input, float* Output, float* Setpoint,
           float Min, float Max,  
           float Kp, float Ki, float Kd,
           pMode pMode, 
           dMode dMode,
           iAwMode iAwMode,
           Action Action,
           uint32_t SampleTimeUs,
           Control Mode,
           centMode CentMode, float Center) {  // Soren
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
QPID::QPID(float* Input, int32_t* IntOutput, float* Setpoint,  
           float Min, float Max,
           float Kp, float Ki, float Kd,
           pMode pMode, 
           dMode dMode,
           iAwMode iAwMode,
           Action Action,
           uint32_t SampleTimeUs,
           Control Mode,
           centMode CentMode, float Center) {  // Soren
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
    if (action == Action::reverse) dInput = -dInput;

    error = *mySetpoint - input;
    if (action == Action::reverse) error = -error;
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

    //condition anti-windup (default)
    if (iawmode == iAwMode::iAwCondition) {
      bool aw = false;
      float iTermOut = (peTerm - pmTerm) + ki * (iTerm + error);
      if (iTermOut > outMax && dError > 0) aw = true;
      else if (iTermOut < outMin && dError < 0) aw = true;
      if (aw && ki) iTerm = constrain(iTermOut, -outMax, outMax);
    }

    if (centmode == centMode::centerStrict && error * lastError < 0) outputSum = center;  // Soren - Recenters any old integral when error crosses zero
    
    // by default, compute output as per PID_v1
    outputSum += iTerm;                                                 // include integral amount
    if (iawmode == iAwMode::iAwOff) outputSum -= pmTerm;                // include pmTerm (no anti-windup)
    else outputSum = constrain(outputSum - pmTerm, outMin, outMax);     // include pmTerm and clamp
    
    if (int32_output) *myIntOutput = (int32_t)(constrain(outputSum + peTerm + dTerm, outMin, outMax));  // Soren
    else *myOutput = constrain(outputSum + peTerm + dTerm, outMin, outMax);  // include dTerm, clamp and drive output

    lastError = error;
    lastInput = input;
    lastTime = now;
    return true;
  }
  else return false;
}

/* SetTunings(....)************************************************************
  This function allows the controller's dynamic performance to be adjusted.
  it's called automatically from the constructor, but tunings can also
  be adjusted on the fly during normal operation.
******************************************************************************/
void QPID::SetTunings(float Kp, float Ki, float Kd, 
                      pMode pMode, 
                      dMode dMode,
                      iAwMode iAwMode) { // Soren

  if (Kp < 0 || Ki < 0 || Kd < 0) return;
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
  if (NewSampleTimeUs > 0) {
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
  outMin = Min;
  outMax = Max;

  if (mode != Control::manual) {
    if (int32_output) *myIntOutput = (int32_t)constrain((float)*myIntOutput, outMin, outMax);  // Soren
    else *myOutput = constrain(*myOutput, outMin, outMax);  // Soren
    outputSum = constrain(outputSum, outMin, outMax);
  }
}

void QPID::SetCentMode(centMode CentMode) {  // Soren
  centmode = CentMode;  // Soren
}

void QPID::SetCenter(float Center) {  // Soren
  if (outMin <= Center && outMax >= Center) {  // Soren
    center = Center;  // Soren
    if (centmode == centMode::range) centmode = centMode::centerStrict;  // Soren
  }
}


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
uint8_t QPID::GetMode() { return static_cast<uint8_t>(mode); }
uint8_t QPID::GetDirection() { return static_cast<uint8_t>(action); }
uint8_t QPID::GetPmode() { return static_cast<uint8_t>(pmode); }
uint8_t QPID::GetDmode() { return static_cast<uint8_t>(dmode); }
uint8_t QPID::GetAwMode() { return static_cast<uint8_t>(iawmode); }
uint8_t QPID::GetCentMode() { return static_cast<uint8_t>(centmode); }  // Soren
float QPID::GetCenter() { return center; }  // Soren