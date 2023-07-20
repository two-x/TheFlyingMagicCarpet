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
    enum class centMode : uint8_t {range, center, centerStrict};    // Soren - Allows a defined output zero point

    // commonly used functions ************************************************************************************

    // Default constructor
    QPID();

    // Constructor with default values
    QPID(float* Input, float* Output, float* Setpoint,
        float Min, float Max,  
        float Kp = 0, 
        float Ki = 0, 
        float Kd = 0,
        pMode pMode = pMode::pOnError,
        dMode dMode = dMode::dOnMeas,
        iAwMode iAwMode = iAwMode::iAwCondition,  
        Action Action = Action::direct,
        uint32_t SampleTimeUs = 100000,
        Control Mode = Control::manual,
        centMode CentMode = centMode::range, 
        float Center = 1234567);

    // Constructor. Links the PID to Input, Output, Setpoint, initial tuning parameters and control modes.
    // QPID(float *Input, float *Output, float *Setpoint, 
    // float Min, float Max, 
    // float Kp, float Ki, float Kd,                                // Soren edit
    //      pMode pMode, dMode dMode, iAwMode iAwMode, Action Action, uint32_t SampleTimeUs, Control Mode, centMode CentMode, float Center); // Soren edit
    // Bobby edit- Duplicate of the constructor above. Calls will use defaults if they don't send in args

    // Constructor allowing use of integer instead of float output value. Soren
    QPID(float* Input, int32_t* IntOutput, float* Setpoint,  
           float Min, float Max,
           float Kp = 0, 
           float Ki = 0,  
           float Kd = 0,
           pMode pMode = pMode::pOnError,
           dMode dMode = dMode::dOnMeas,
           iAwMode iAwMode = iAwMode::iAwCondition,
           Action Action = Action::direct,  
           uint32_t SampleTimeUs = 100000,
           Control Mode = Control::manual,
           centMode CentMode = centMode::range, 
           float Center = 1234567); // soren

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
    void SetTunings(float Kp, float Ki, float Kd, 
                  pMode pMode = pMode::pOnError,
                  dMode dMode = dMode::dOnMeas,
                  iAwMode iAwMode = iAwMode::iAwCondition); // soren

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

}; // class QPID


#endif // QPID.h
