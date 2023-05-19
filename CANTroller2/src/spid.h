#ifndef SPID_H
#define SPID_H

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

using namespace std;

class PID {
    public:
        //Constants used in some of the functions below
        #define AUTOMATIC	1
        #define MANUAL	0
        #define DIRECT  0
        #define REVERSE  1
        #define P_ON_M 0
        #define P_ON_E 1

        // Constructors.  links the PID to the Input, Output, and Setpoint.  Initial tuning parameters are also set here. (overload for specifying proportional mode)
        PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int32_t POn, int32_t ControllerDirection) {
            myOutput = Output;
            myInput = Input;
            mySetpoint = Setpoint;
            inAuto = false;
            PID::SetOutputLimits(0, 255);				//default output limit corresponds to
                                                        //the arduino pwm limits
            SampleTime = 100;							//default Controller Sample Time is 0.1 seconds
            PID::SetControllerDirection(ControllerDirection);
            PID::SetTunings(Kp, Ki, Kd, POn);
            lastTime = millis()-SampleTime;
        }
        PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int32_t ControllerDirection) { }
        
        bool Compute() {
            if(!inAuto) return false;
            unsigned long now = millis();
            unsigned long timeChange = (now - lastTime);
            if(timeChange>=SampleTime) {
                /*Compute all the working error variables*/
                double input = *myInput;
                double error = *mySetpoint - input;
                double dInput = (input - lastInput);
                outputSum+= (ki * error);                
                if(!pOnE) outputSum-= kp * dInput;  /*Add Proportional on Measurement, if P_ON_M is specified*/

                if(outputSum > outMax) outputSum= outMax;
                else if(outputSum < outMin) outputSum= outMin;

                /*Add Proportional on Error, if P_ON_E is specified*/
                double output;
                if(pOnE) output = kp * error;
                else output = 0;

                /*Compute Rest of PID Output*/
                output += outputSum - kd * dInput;

                *myOutput = constrain(output, outMin, outMax);
                // if(output > outMax) output = outMax;
                // else if(output < outMin) output = outMin;
                // *myOutput = output;

                /*Remember some variables for next time*/
                lastInput = input;
                lastTime = now;
                return true;
            }
            else return false;
        }
        void SetTunings(double Kp, double Ki, double Kd){
            SetTunings(Kp, Ki, Kd, pOn); 
        }
        void SetTunings(double Kp, double Ki, double Kd, int POn) {  // Allows changing parameters any time, or setting proportional mode
            if (Kp<0 || Ki<0 || Kd<0) return;
            pOn = POn;
            pOnE = POn == P_ON_E;
            dispKp = Kp; dispKi = Ki; dispKd = Kd;

            double SampleTimeInSec = ((double)SampleTime)/1000;
            kp = Kp;
            ki = Ki * SampleTimeInSec;
            kd = Kd / SampleTimeInSec;

            if(controllerDirection ==REVERSE) {
                kp = (0 - kp);
                ki = (0 - ki);
                kd = (0 - kd);
            }
        }
        void SetSampleTime(int NewSampleTime) {
            if (NewSampleTime > 0) {
                double ratio  = (double)NewSampleTime / (double)SampleTime;
                ki *= ratio;
                kd /= ratio;
                SampleTime = (unsigned long)NewSampleTime;
            }
        }
        void SetOutputLimits(double Min, double Max) {
            if(Min >= Max) return;
            outMin = Min;
            outMax = Max;
            if(inAuto) {
                if(*myOutput > outMax) *myOutput = outMax;
                else if(*myOutput < outMin) *myOutput = outMin;

                if(outputSum > outMax) outputSum= outMax;
                else if(outputSum < outMin) outputSum= outMin;
            }
        }
        void SetMode(int Mode) {
            bool newAuto = (Mode == AUTOMATIC);
            if(newAuto && !inAuto) {  /*we just went from manual to auto*/
                PID::Initialize();
            }
            inAuto = newAuto;
        }
        void SetControllerDirection(int Direction) {
            if(inAuto && Direction !=controllerDirection) {
                kp = (0 - kp);
                ki = (0 - ki);
                kd = (0 - kd);
            }
            controllerDirection = Direction;
        }
        double GetKp(){ return  dispKp; }
        double GetKi(){ return  dispKi;}
        double GetKd(){ return  dispKd;}
        int GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
        int GetDirection(){ return controllerDirection;}

    private:
        double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
        double dispKi;				//   format for display purposes
        double dispKd;				//
        double kp;                  // * (P)roportional Tuning Parameter
        double ki;                  // * (I)ntegral Tuning Parameter
        double kd;                  // * (D)erivative Tuning Parameter
        int controllerDirection;
        int pOn;
        double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
        double *myOutput;             //   This creates a hard link between the variables and the 
        double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                        //   what these values are.  with pointers we'll just know.    
        unsigned long lastTime;
        double outputSum, lastInput;
        unsigned long SampleTime;
        double outMin, outMax;
        bool inAuto, pOnE;

        void Initialize() {
            outputSum = *myOutput;
            lastInput = *myInput;
            if(outputSum > outMax) outputSum = outMax;
            else if(outputSum < outMin) outputSum = outMin;
        }
};

#endif