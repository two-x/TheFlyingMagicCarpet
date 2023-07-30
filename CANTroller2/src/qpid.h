#pragma once
#include <stdint.h>
#ifndef QPID_h
#define QPID_h
#include "Arduino.h"

class TargetControl {  // Soren - To allow creative control of PID targets in case your engine idle problems need that.
  public:
    enum class targetMode : uint32_t { softLanding, activeMin };  // After soft landing at minimum (which it will always do), Can be set to continuously further minimize target until irregular pulses detected. 
  protected:
    enum targetState : uint32_t { onTarget, gotoHighIdle, Timer, minimize };
    float initialTarget, lastTarget;
    float* target;
    float* outRaw;
    float* outFilt;
    Timer setTimer;
    targetMode targetmode;
  public:
    enum class targetMode : uint32_t { softLanding, activeMin };  // After soft landing at minimum (which it will always do), Can be set to continuously further minimize target until irregular pulses detected. 
    TargetControl (float* argTarget, float* argOutRaw, float* argOutFilt, float initial, uint32_t settime, targetMode targmode = targetMode::softLanding) {
        target = argTarget;
        outRaw = argOutRaw;
        outFilt = argOutFilt;
        lastTarget = *target;
        setInitial (initial);
        setTimer.set ((int64_t)settime);

    }
    void setInitial (float initial) { initialTarget = initial; }
    void update (float deviationRatio = 1.0) {  // deviationRatio tells us how unstable our output is being (0=stable to 1=unstable)

    }
};
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
//===================================================================================

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
uint8_t QPID::GetMode() { return static_cast<uint8_t>(mode); }
uint8_t QPID::GetDirection() { return static_cast<uint8_t>(action); }
uint8_t QPID::GetPmode() { return static_cast<uint8_t>(pmode); }
uint8_t QPID::GetDmode() { return static_cast<uint8_t>(dmode); }
uint8_t QPID::GetAwMode() { return static_cast<uint8_t>(iawmode); }
uint8_t QPID::GetCentMode() { return static_cast<uint8_t>(centmode); }  // Soren
float QPID::GetCenter() { return center; }  // Soren

#endif // QPID.h