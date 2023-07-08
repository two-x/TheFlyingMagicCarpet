#pragma once
#include <stdint.h>
#ifndef QPID_h
#define QPID_h

class QPID {

  public:

    enum class Control : uint8_t {manual, automatic, timer, toggle};  // controller mode
    enum class Action : uint8_t {direct, reverse};                    // controller action
    enum class pMode : uint8_t {pOnError, pOnMeas, pOnErrorMeas};     // proportional mode
    enum class dMode : uint8_t {dOnError, dOnMeas};                   // derivative mode
    enum class iAwMode : uint8_t {iAwCondition, iAwClamp, iAwOff};    // integral anti-windup mode

    // commonly used functions ************************************************************************************

    // Default constructor
    QPID();

    // Constructor. Links the PID to Input, Output, Setpoint, initial tuning parameters and control modes.
    QPID(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd,
             pMode pMode, dMode dMode, iAwMode iAwMode, Action Action);

    // Overload constructor links the PID to Input, Output, Setpoint, tuning parameters and control Action.
    // Uses defaults for remaining parameters.
    QPID(double *Input, double *Output, double *Setpoint, double Kp, double Ki, double Kd, Action Action);

    // Simplified constructor which uses defaults for remaining parameters.
    // QPID(double *Input, double *Output, double *Setpoint);
    
    // Simplified constructor which uses defaults for remaining parameters.
    QPID(double *Input, int32_t *Output, double *Setpoint);

    // Sets PID mode to manual (0), automatic (1), timer (2) or toggle manual/automatic (3).
    void SetMode(Control Mode);
    void SetMode(uint8_t Mode);

    // Performs the PID calculation. It should be called every time loop() cycles ON/OFF and calculation frequency
    // can be set using SetMode and SetSampleTime respectively.
    bool Compute();

    // Sets and clamps the output to a specific range (0-255 by default).
    void SetOutputLimits(double Min, double Max);

    // available but not commonly used functions ******************************************************************

    // While most users will set the tunings once in the constructor, this function gives the user the option of
    // changing tunings during runtime for Adaptive control.
    void SetTunings(double Kp, double Ki, double Kd);

    // Overload for specifying proportional ratio.
    void SetTunings(double Kp, double Ki, double Kd, pMode pMode, dMode dMode, iAwMode iAwMode);

    // Soren: I wrote these to facilitate changing only one tuning parameter at a time
    void SetKp(double Kp);
    void SetKi(double Ki);
    void SetKd(double Kd);

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
    void SetOutputSum(double sum);

    void Initialize();        // Ensure a bumpless transfer from manual to automatic mode
    void Reset();             // Clears pTerm, iTerm, dTerm and outputSum values

    // PID Query functions ****************************************************************************************
    double GetError();  // Soren added
    double GetKp();            // proportional gain
    double GetKi();            // integral gain
    double GetKd();            // derivative gain
    double GetPterm();         // proportional component of output
    double GetIterm();         // integral component of output
    double GetDterm();         // derivative component of output
    double GetOutputSum();     // summation of all pid term components
    uint8_t GetMode();        // manual (0), automatic (1), timer (2) or toggle manual/automatic (3)
    uint8_t GetDirection();   // direct (0), reverse (1)
    uint8_t GetPmode();       // pOnError (0), pOnMeas (1), pOnErrorMeas (2)
    uint8_t GetDmode();       // dOnError (0), dOnMeas (1)
    uint8_t GetAwMode();      // iAwCondition (0, iAwClamp (1), iAwOff (2)

    double outputSum;          // Internal integral sum

  private:

    double dispKp = 0;   // for defaults and display
    double dispKi = 0;
    double dispKd = 0;
    double pTerm;
    double iTerm;
    double dTerm;

    double kp;           // (P)roportional Tuning Parameter
    double ki;           // (I)ntegral Tuning Parameter
    double kd;           // (D)erivative Tuning Parameter

    double *myInput;     // Pointers to the Input, Output, and Setpoint variables. This creates a
    double *myOutput;    // hard link between the variables and the PID, freeing the user from having
    double *mySetpoint;  // to constantly tell us what these values are. With pointers we'll just know.

    Control mode = Control::manual;
    Action action = Action::direct;
    pMode pmode = pMode::pOnError;
    dMode dmode = dMode::dOnMeas;
    iAwMode iawmode = iAwMode::iAwCondition;

    uint32_t sampleTimeUs, lastTime;
    double outMin, outMax, error, lastError, lastInput;
    
    int32_t *myIntOutput;  // SC
    bool int32_output = false;  // SC

}; // class QPID

// End of .h file, start of .cpp file

/**********************************************************************************
   QPID Library for Arduino - Version 3.1.9
   by dlloydev https://github.com/Dlloydev/QPID
   Based on the Arduino PID_v1 Library. Licensed under the MIT License.
 **********************************************************************************/

// #if ARDUINO >= 100
// #include "Arduino.h"
// #else
// #include "WProgram.h"
// #endif

// #include "QPID.h"

QPID::QPID() {}

/* Constructor ********************************************************************
   The parameters specified here are those for for which we can't set up
   reliable defaults, so we need to have the user set them.
 **********************************************************************************/

QPID::QPID(double* Input, double* Output, double* Setpoint,
                   double Kp = 0, double Ki = 0, double Kd = 0,
                   pMode pMode = pMode::pOnError,
                   dMode dMode = dMode::dOnMeas,
                   iAwMode iAwMode = iAwMode::iAwCondition,
                   Action Action = Action::direct) {

  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  mode = Control::manual;

  QPID::SetOutputLimits(0, 255);  // same default as Arduino PWM limit
  sampleTimeUs = 100000;              // 0.1 sec default
  QPID::SetControllerDirection(Action);
  QPID::SetTunings(Kp, Ki, Kd, pMode, dMode, iAwMode);

  lastTime = micros() - sampleTimeUs;

  int32_output = false;
}

/* Constructor *********************************************************************
   To allow using pOnError, dOnMeas and iAwCondition without explicitly saying so.
 **********************************************************************************/
QPID::QPID(double* Input, double* Output, double* Setpoint,
                   double Kp, double Ki, double Kd, Action Action)
  : QPID::QPID(Input, Output, Setpoint, Kp, Ki, Kd,
                       pmode = pMode::pOnError,
                       dmode = dMode::dOnMeas,
                       iawmode = iAwMode::iAwCondition,
                       action = Action) {
}

/* Constructor *********************************************************************
   Simplified constructor which uses defaults for remaining parameters.
 **********************************************************************************/
// QPID::QPID(double* Input, double* Output, double* Setpoint)
//   : QPID::QPID(Input, Output, Setpoint,
//                        dispKp = 0,
//                        dispKi = 0,
//                        dispKd = 0,
//                        pmode = pMode::pOnError,
//                        dmode = dMode::dOnMeas,
//                        iawmode = iAwMode::iAwCondition,
//                        action = Action::direct) {
// }

QPID::QPID(double* Input, int32_t* IntOutput, double* Setpoint) {  // SC
  int32_output = true;  // SC

  double Kp = 0, Ki = 0, Kd = 0;
  pMode pMode = pMode::pOnError;
  dMode dMode = dMode::dOnMeas;
  iAwMode iAwMode = iAwMode::iAwCondition;
  Action Action = Action::direct;

  myIntOutput = IntOutput;
  myInput = Input;
  mySetpoint = Setpoint;
  mode = Control::manual;

  QPID::SetOutputLimits(0, 255);  // same default as Arduino PWM limit
  sampleTimeUs = 100000;              // 0.1 sec default
  QPID::SetControllerDirection(Action);
  QPID::SetTunings(Kp, Ki, Kd, pMode, dMode, iAwMode);

  lastTime = micros() - sampleTimeUs;
}

/* Compute() ***********************************************************************
   This function should be called every time "void loop()" executes. The function
   will decide whether a new PID Output needs to be computed. Returns true
   when the output is computed, false when nothing has been done.
 **********************************************************************************/
bool QPID::Compute() {
  if (mode == Control::manual) return false;
  uint32_t now = micros();
  uint32_t timeChange = (now - lastTime);
  if (mode == Control::timer || timeChange >= sampleTimeUs) {

    double input = *myInput;
    double dInput = input - lastInput;
    if (action == Action::reverse) dInput = -dInput;

    error = *mySetpoint - input;
    if (action == Action::reverse) error = -error;
    double dError = error - lastError;

    double peTerm = kp * error;
    double pmTerm = kp * dInput;
    if (pmode == pMode::pOnError) pmTerm = 0;
    else if (pmode == pMode::pOnMeas) peTerm = 0;
    else { //pOnErrorMeas
      peTerm *= 0.5f;
      pmTerm *= 0.5f;
    }
    pTerm = peTerm - pmTerm; // used by GetDterm()
    iTerm = ki  * error;
    if (dmode == dMode::dOnError) dTerm = kd * dError;
    else dTerm = -kd * dInput; // dOnMeas

    //condition anti-windup (default)
    if (iawmode == iAwMode::iAwCondition) {
      bool aw = false;
      double iTermOut = (peTerm - pmTerm) + ki * (iTerm + error);
      if (iTermOut > outMax && dError > 0) aw = true;
      else if (iTermOut < outMin && dError < 0) aw = true;
      if (aw && ki) iTerm = constrain(iTermOut, -outMax, outMax);
    }

    // by default, compute output as per PID_v1
    outputSum += iTerm;                                                 // include integral amount
    if (iawmode == iAwMode::iAwOff) outputSum -= pmTerm;                // include pmTerm (no anti-windup)
    else outputSum = constrain(outputSum - pmTerm, outMin, outMax);     // include pmTerm and clamp
    
    if (int32_output) *myIntOutput = (int32_t)(constrain(outputSum + peTerm + dTerm, outMin, outMax));  // SC
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
void QPID::SetTunings(double Kp, double Ki, double Kd,
                          pMode pMode = pMode::pOnError,
                          dMode dMode = dMode::dOnMeas,
                          iAwMode iAwMode = iAwMode::iAwCondition) {

  if (Kp < 0 || Ki < 0 || Kd < 0) return;
  if (Ki == 0) outputSum = 0;
  pmode = pMode; dmode = dMode; iawmode = iAwMode;
  dispKp = Kp; dispKi = Ki; dispKd = Kd;
  double SampleTimeSec = (double)sampleTimeUs / 1000000;
  kp = Kp;
  ki = Ki * SampleTimeSec;
  kd = Kd / SampleTimeSec;
}

/* SetTunings(...)************************************************************
  Set Tunings using the last remembered pMode, dMode and iAwMode settings.
******************************************************************************/
void QPID::SetTunings(double Kp, double Ki, double Kd) {
  SetTunings(Kp, Ki, Kd, pmode, dmode, iawmode);
}

// Soren: I wrote these to facilitate changing only one tuning parameter at a time
void QPID::SetKp(double Kp) { SetTunings(Kp, dispKi, dispKd, pmode, dmode, iawmode); }
void QPID::SetKi(double Ki) { SetTunings(dispKp, Ki, dispKd, pmode, dmode, iawmode); }
void QPID::SetKd(double Kd) { SetTunings(dispKp, dispKi, Kd, pmode, dmode, iawmode); }

/* SetSampleTime(.)***********************************************************
  Sets the period, in microseconds, at which the calculation is performed.
******************************************************************************/
void QPID::SetSampleTimeUs(uint32_t NewSampleTimeUs) {
  if (NewSampleTimeUs > 0) {
    double ratio  = (double)NewSampleTimeUs / (double)sampleTimeUs;
    ki *= ratio;
    kd /= ratio;
    sampleTimeUs = NewSampleTimeUs;
  }
}

/* SetOutputLimits(..)********************************************************
  The PID controller is designed to vary its output within a given range.
  By default this range is 0-255, the Arduino PWM range.
******************************************************************************/
void QPID::SetOutputLimits(double Min, double Max) {
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (mode != Control::manual) {
    if (int32_output) *myIntOutput = (int32_t)constrain((double)*myIntOutput, outMin, outMax);  // SC
    else *myOutput = constrain(*myOutput, outMin, outMax);  // SC
    outputSum = constrain(outputSum, outMin, outMax);
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
  outputSum = (double)*myOutput;  // SC
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
  lastTime = micros() - sampleTimeUs;
  lastInput = 0;
  outputSum = 0;
  pTerm = 0;
  iTerm = 0;
  dTerm = 0;
}

// sets the output summation value
void QPID::SetOutputSum(double sum) {
  outputSum = sum;
}

/* Status Functions************************************************************
  These functions query the internal state of the PID.
******************************************************************************/
double QPID::GetError() { return error; }  // Soren added
double QPID::GetKp() { return dispKp; }
double QPID::GetKi() { return dispKi; }
double QPID::GetKd() { return dispKd; }
double QPID::GetPterm() { return pTerm; }
double QPID::GetIterm() { return iTerm; }
double QPID::GetDterm() { return dTerm; }
double QPID::GetOutputSum() { return outputSum; }
uint8_t QPID::GetMode() { return static_cast<uint8_t>(mode); }
uint8_t QPID::GetDirection() { return static_cast<uint8_t>(action); }
uint8_t QPID::GetPmode() { return static_cast<uint8_t>(pmode); }
uint8_t QPID::GetDmode() { return static_cast<uint8_t>(dmode); }
uint8_t QPID::GetAwMode() { return static_cast<uint8_t>(iawmode); }

#endif // QPID.h
