#ifndef LPID_H
#define LPID_H
#ifdef DUE
#include <LibPrintf.h>  // This works on Due but not ESP32
#endif
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class LPID {  // Soren's bastardized version of the Arduino "PID" library https://github.com/br3ttb/Arduino-PID-Library/tree/master
    private:
        double dispKp, dispKi, dispKd;  // Tuning parameters in passed-in units 
        double kp, ki, kd;  // Tuning parameters scaled to sample time
        double *myInput, *myOutput, *mySetpoint;  // Pointers to the Input, Output, and Setpoint variables. Frees the user from having to constantly tell us what these values are
        double outputSum, lastInput, outMin, outMax;
        double error, p_term, i_term, d_term, delta;  // Soren added these
        uint32_t lastTime, SampleTime;
        int32_t controllerDirection, pOn;
        bool inAuto, pOnE;

        void Initialize() {
            outputSum = *myOutput;
            lastInput = *myInput;
            if(outputSum > outMax) outputSum = outMax;
            else if(outputSum < outMin) outputSum = outMin;
        }
    public:
        #define AUTOMATIC 1
        #define MANUAL 0
        #define DIRECT 0
        #define REVERSE 1
        #define P_ON_M 0
        #define P_ON_E 1

        LPID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int32_t sampleTime, int32_t POn, int32_t ControllerDirection) {
            myOutput = Output;
            myInput = Input;
            mySetpoint = Setpoint;
            inAuto = false;
            
            SetOutputLimits(0, 255);				//default output limit corresponds to
                                                        //the arduino pwm limits
            SampleTime = (uint32_t)sampleTime;							//default Controller Sample Time is 0.1 seconds
            SetControllerDirection(ControllerDirection);
            SetTunings(Kp, Ki, Kd);
            SetPropMode(POn);
            
            lastTime = millis()-SampleTime;
        }
        
        bool Compute() {
            if(!inAuto) return false;
            double input = *myInput;
            error = *mySetpoint - input;
            double dInput = (input - lastInput);
            double output;
            double output_last = outputSum;
            
            outputSum += (ki * error);
            i_term = outputSum;

            if(!pOnE) {  // Add Proportional on Measurement, of P_ON_M is specified
                p_term = -1 * kp * dInput;
                outputSum += p_term;
                output = 0;
            }
            else {  // Add Proportional on Error, if P_ON_E is specified
                p_term = kp * error;
                output = p_term;
            }
            if (outputSum > outMax) outputSum = outMax;
            else if (outputSum < outMin) outputSum = outMin;

            d_term = -1 * kd * dInput;
            output += outputSum + d_term;

            if (output > outMax) output = outMax;
            else if (output < outMin) output = outMin;
            *myOutput = output;

            delta = output - output_last;

            printf(", setp=%10.4lf, err=%10.4lf, pt=%10.4lf, it=%10.4lf, dt=%10.4lf, delt=%10.4lf, out=%10.4lf, kp=%10.4lf, ki=%10.4lf, kd=%10.4lf", *mySetpoint, error, p_term, i_term, d_term, delta, *myOutput, kp, ki, kd);

            lastInput = input;  // Remember some variables for next time
            lastTime = millis();
            return true;
        }
        void SetTunings(double Kp, double Ki, double Kd) {
            if (Kp < 0 || Ki < 0 || Kd < 0) return;
            dispKp = Kp; dispKi = Ki; dispKd = Kd;

            double SampleTimeInSec = ((double)SampleTime)/1000;
            kp = Kp;
            ki = Ki * SampleTimeInSec;
            kd = Kd / SampleTimeInSec;

            if (controllerDirection ==REVERSE) {
                kp = (0 - kp);
                ki = (0 - ki);
                kd = (0 - kd);
            }
        }
        void SetPropMode(int32_t POn) {  // Allows changing parameters any time, or setting proportional mode
            pOn = POn;
            pOnE = (POn == P_ON_E);
        }
        void SetSampleTime(int32_t NewSampleTime) {
            if (NewSampleTime > 0) {
                double ratio  = (double)NewSampleTime / (double)SampleTime;
                ki *= ratio;
                kd /= ratio;
                SampleTime = (unsigned long)NewSampleTime;
            }
        }
        void SetOutputLimits(double Min, double Max) {
            if (Min >= Max) return;
            outMin = Min;
            outMax = Max;
            if (inAuto) {
                if (*myOutput > outMax) *myOutput = outMax;
                else if (*myOutput < outMin) *myOutput = outMin;

                if (outputSum > outMax) outputSum = outMax;
                else if (outputSum < outMin) outputSum = outMin;
            }
        }
        void SetMode(int32_t Mode) {
            bool newAuto = (Mode == AUTOMATIC);
            if (newAuto && !inAuto) Initialize();  /*we just went from manual to auto*/
            inAuto = newAuto;
        }
        void SetControllerDirection(int32_t Direction) {
            if (inAuto && Direction != controllerDirection) {
                kp = (0 - kp);
                ki = (0 - ki);
                kd = (0 - kd);
            }
            controllerDirection = Direction;
        }
        double GetKp() { return  dispKp; }
        double GetKi() { return  dispKi; }
        double GetKd() { return  dispKd; }
        int32_t GetPropMode() { return pOn; }
        int32_t GetMode() { return inAuto ? AUTOMATIC : MANUAL; }
        int32_t GetDirection() { return controllerDirection; }
        double GetError() { return error; }
        double GetDelta() { return delta; }
        double GetPterm() { return p_term; }
        double GetIterm() { return i_term; }
        double GetDterm() { return d_term; }
};

#endif