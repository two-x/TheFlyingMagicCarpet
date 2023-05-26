#ifndef SPID_H
#define SPID_H
#ifdef DUE
#include <LibPrintf.h>  // This works on Due but not ESP32
#endif
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "math.h"  // Just using for signbit() function. Note signbit returns true for negative signed argument

// Here is the brake PID math
// Our target is the desired amount of the measured value. The error is what we must add to ourt current value to get there
// We make 3 terms P I and D which add to become our Delta which goes to the actuator.
// P term scales proportionally to error, however it can never reach the setpoint 
// I term steadily grows the longer the error is the same sign, adds a boost to P. I coefficient is   
// D term counteracts fast changes from P (and I), serving to prevent overshooting target.
// Error = Setpoint - ProcessValue
// Output  =  P + I + D  =  (K * Error) + (K / Tau_I) + (Error - LastError)

class SPID {  // Soren's home-made pid loop
    public:
        #define ABSOLUTE 0  // assign to outCenterMode if pid output just spans a range, rather than deviating from a centerpoint 
        #define RELATIVE 1  // assign to outCenterMode if pid output deviates from a centerpoint 
        #define ERROR_TERM 0  // What the proportional term is proportional_to
        #define SENSED_INPUT 1  // What the proportional term is proportional_to
        #define FWD 1  // Actuator_direction influences sensed value in the same direction
        #define REV -1  // Actuator_direction influences sensed value in the opposite direction
    private:
        double kp_coeff = 0, ki_coeff = 0, kd_coeff = 0, kp, ki_hz, kd_s;
        int32_t actuator_direction;
        uint32_t sample_period_ms;
        double target = 0, error = 0, error_last = 0, p_term = 0, i_term = 0, d_term = 0;
        double near_target_error_thresh_percent = 0.005;  // Fraction of the input range where if the error has not exceeded in either dir since last zero crossing, it is considered zero (to prevent endless microadjustments) 
        double output = 0, input = 0, input_last = 0, near_target_error_thresh = 0, near_target_lock = (in_max-in_min)/2;
        double in_min = 0, in_max = 4095, out_min = 0, out_max = 4095, in_center = 2047, out_center = 2047;
        bool out_center_mode = RELATIVE, in_center_mode = RELATIVE, proportional_to = ERROR_TERM, saturated = false, output_hold = false;
    public:
        int32_t disp_kp_1k, disp_ki_mhz, disp_kd_ms;  // disp_* are integer version of tuning variables scaled up by *1000, suitable for screen display

        SPID(double arg_kp, double arg_ki_hz, double arg_kd_s, int32_t direction, int32_t arg_sample_period_ms) {  // , bool arg_in_center_mode, bool arg_out_center_mode
            set_tunings(arg_kp, arg_ki_hz, arg_kd_s);
            set_actuator_direction(direction);
            // set_center_modes(arg_in_center_mode, arg_out_center_mode);
            sample_period_ms = (uint32_t)arg_sample_period_ms;
        }
        void compute(double arg_input) {
            input_last = input;
            input = arg_input;
            if (input < in_min) input = in_min;
            else if (input > in_max) input = in_max;
            error_last = error;
            error = target - input;

            printf(" in=%-+9.4lf err=%-+9.4lf errlast=%-+9.4lf ntet=%-+9.4lf kp_co=%-+9.4lf ki_co=%-+9.4lf kd_co=%-+9.4lf kds=%-+9.4lf kd_disp=%ld", input, error, error_last, near_target_error_thresh, kp_coeff, ki_coeff, kd_coeff, kd_s, disp_kd_ms);

            // Add a layer of history here, with an additional condition that output only holds if change in error has been small for X time period
            // Also add code to hold output (below in this function) if output_hold = true
            if (signbit(error_last) != signbit(error) && abs(error_last - error) < near_target_error_thresh) {
                near_target_lock = target;
                output_hold = true;
            }
            else if (abs(near_target_lock - input) > near_target_error_thresh) output_hold = false;

            // Add handling for RELATIVE controller!!  (our brake)

            if (proportional_to == ERROR_TERM) p_term = kp_coeff * error;  // If proportional_to Error (default)
            else p_term = -kp_coeff * (input - input_last);  // If proportional_to Input (default)

            if (saturated && (output * error * actuator_direction >= 0)) i_term = 0;  // Delete intregal windup if windup is occurring
            else i_term += ki_coeff * error;  // Update integral

            d_term = kd_coeff * (input - input_last);
            
            double delta = p_term + i_term - d_term;

            output = out_center + delta;

            printf(" ntl2=%-+9.4lf sat=%1d outh=%1d pterm=%-+9.4lf iterm=%-+9.4lf dterm=%-+9.4lf out=%-+9.4lf", near_target_lock, saturated, output_hold, p_term, i_term, d_term, output);

            if (output < out_min) {  // need to include a margin
                output = out_min;
                saturated = true;
            }
            else if (output > out_max) {  // need to include a margin
                output = out_max;
                saturated = true;
            }
            else saturated = false;
        }
        // void set_input(double arg_input) {
        //     input_last = input;
        //     input = arg_input;
        // }
        // void set_saturated(bool arg_saturated) {
        //     saturated = arg_saturated;
        // }
        void set_target(double arg_target) {
            // printf("SPID::set_target():  received arg_target=%-+9.4lf, in_min=%-+9.4lf, in_max=%-+9.4lf\n", arg_target, in_min, in_max);
            if (arg_target < in_min || arg_target > in_max) {
                printf ("Warning: SPID::set_target() ignored request to set target beyond input range.\n");
                return;
            }
            target = arg_target;
            error = target - input;
        }
        void set_tunings(double arg_kp, double arg_ki_hz, double arg_kd_s) {  // Set tuning parameters to negative values if higher actuator values causes lower sensor values and vice versa 
            if (arg_kp < 0 || arg_ki_hz < 0 || arg_kd_s < 0) {  // || ( arg_kp <= 0 && arg_ki_hz <= 0 && arg_kd_s <= 0 ) ) ) {
                printf ("Warning: SPID::set_tunings() ignored request to set tuning parameters to negative values.\n");
                return;
            }
            double sample_period_s = ((double)sample_period_ms)/1000;
            kp = arg_kp;  ki_hz = arg_ki_hz;  kd_s = arg_kd_s;
            kp_coeff = arg_kp;
            ki_coeff = arg_ki_hz * sample_period_s;
            
            // printf("dispkds=%ld", disp_kd_ms);
            // printf(", kds=%lf", kd_s);
            // printf(", speriodms=%ld", sample_period_ms);
            // printf(", speriods=%lf", sample_period_s);
            // printf(", argkds=%lf", arg_kd_s);

            kd_coeff = arg_kd_s;  // / sample_period_s;
            // if (arg_kd_s != 0) kd_coeff = arg_kd_s /(((double)sample_period_ms)/1000);  // ??? yes, all those parentheses are needed or kd_coeff is infinite (due to div/0?)
            // else kd_coeff = 0;
            
            // printf(", kd_coeff=%lf", kd_coeff);
            // printf("\n");

            disp_kp_1k = (int32_t)(arg_kp * 1000);  // Scaled integer version of tuning parameters suitable for screen display
            disp_ki_mhz = (int32_t)(arg_ki_hz * 1000);
            disp_kd_ms = (int32_t)(arg_kd_s * 1000);
        }
        void set_actuator_direction(int32_t direction) {
            if (direction != actuator_direction) {
                kp_coeff = (0 - kp_coeff);
                ki_coeff = (0 - ki_coeff);
                kd_coeff = (0 - kd_coeff);
                actuator_direction = direction;
            }
        }
        void set_input_limits(double arg_min, double arg_max) {
            // printf("SPID::set_input_limits(arg,arg):  received arg_min=%-+9.4lf, arg_max=%-+9.4lf, nttp=%-+9.4lf\n", arg_min, arg_max, near_target_error_thresh_percent);
            if (arg_min >= arg_max) {
                printf ("Warning: SPID::set_input_limits() ignored request to set input minimum limit > maximum limit./n");
                return;
            }
            in_min = arg_min;  in_max = arg_max;
            if (input < in_min) input = in_min;
            else if (input > in_max) input = in_max;
            near_target_error_thresh = (in_max - in_min) * near_target_error_thresh_percent;
        }
        void set_input_limits(double arg_min, double arg_max, double arg_near_target_thresh_percent) {
            // printf("SPID::set_input_limits(arg,arg,arg):  received arg_min=%-+9.4lf, arg_max=%-+9.4lf, arg_nttp=%-+9.4lf\n", arg_min, arg_max, arg_near_target_thresh_percent);
            if (arg_near_target_thresh_percent > 1) {
                printf ("Warning: SPID::set_input_limits() ignored request to set near target error threshold to over 100%%\n");
                return;
            }
            near_target_error_thresh_percent = arg_near_target_thresh_percent;
            set_input_limits(arg_min, arg_max);
        }
        void set_output_limits(double arg_min, double arg_max) {
            if (arg_min >= arg_max) {
                printf ("Warning: SPID::set_output_limits() ignored request to set output minimum limit > maximum limit./n");
                return;
            }
            out_min = arg_min;  out_max = arg_max;
            out_center = (out_min + out_max)/2;
            if (output < out_min) output = out_min;
            else if (output > out_max) output = out_max;
            // return output;                
        }
            // if (*myOutput > outMax) *myOutput = outMax;
            // else if (*myOutput < outMin) *myOutput = outMin;
            // if (outputSum > outMax) outputSum = outMax;
            // else if (outputSum < outMin) outputSum = outMin;
            // if (*myInput > inMax) *myInput = inMax;
            // else if (*myInput < inMin) *myInput = inMin;
            // if (inputSum > inMax) inputSum = inMax;
            // else if (inputSum < inMin) inputSum = inMin;
        void set_input_center(void) { in_center_mode = ABSOLUTE; }  // Call w/o arguments to set input to absolute mode
        void set_input_center(double arg_in_center) {  // Sets input to relative (centerpoint) mode and takes value of center point. 
            if (arg_in_center < in_min || arg_in_center > in_max) {
                printf ("Warning: SPID::set_input_center() ignored request to set input centerpoint outside input range.\n");
                return;
            }
            in_center_mode = RELATIVE;
            in_center = arg_in_center;
        }
        void set_output_center(void) { out_center_mode = ABSOLUTE; }  // Call w/o arguments to set output to absolute mode
        void set_output_center(double arg_out_center) {  // Sets output to relative (centerpoint) mode and takes value of center point. 
            if (arg_out_center < out_min || arg_out_center > out_max) {
                printf ("Warning: SPID::set_output_center() ignored request to set output centerpoint outside output range.\n");
                return;
            }
            out_center_mode = RELATIVE;
            out_center = arg_out_center;
        }
        void set_proportionality(bool arg_prop_to) { 
            proportional_to = arg_prop_to;
        }
        double get_kp_coeff() { return kp_coeff; }
        double get_ki_coeff() { return ki_coeff; }
        double get_kd_coeff() { return kd_coeff; }
        double get_kp() { return kp; }
        double get_ki_hz() { return ki_hz; }
        double get_kd_s() { return kd_s; }
        double get_disp_kp_1k() { return disp_kp_1k; }
        double get_disp_ki_mhz() { return disp_ki_mhz; }
        double get_disp_kd_ms() { return disp_kd_ms; }
        bool get_out_center_mode() { return out_center_mode; }
        bool get_in_center_mode() { return in_center_mode; }
        bool get_proportionality() { return proportional_to; }
        bool get_actuator_direction() { return actuator_direction; }
        double get_near_target_error_thresh_percent() { return near_target_error_thresh_percent; }
        double get_near_target_error_thresh() { return near_target_error_thresh; }
        bool get_out_center() { return out_center; }
        bool get_in_center() { return in_center; }
        bool get_saturated() { return saturated; }
        double get_p_term() { return p_term; }
        double get_i_term() { return i_term; }
        double get_d_term() { return d_term; }
        double get_error() { return error; }
        double get_target() { return target; }
        double get_output() { return output; }       
};
// If output constrain prevents us from exceeding the limits of the actuator. But we need to know two things as we constrain, to preevent "Windup", a condition where the I term exploded due to an extended error when maybe the motor couldn't meet the target. Once back to normal, don't want I term wound up.
// So improve this Clamp to check for A. Is it saturating? I.e. was constrain necessary or not? and B. Is the sign (polarity) of the output the same as that of the error?  If both are true, we have Integrator Windup.  So when we detect this, we can temporarily "Clamp" the I-term to 0 until we are "recovered".
// Recovered can be either of these conditions:  1. We are no longer saturated (constrain is doing nothing)m or, 2. The error changes sign. (then reconnect I term.)
// When determining saturation or not, add a margin.

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
            if (!inAuto) return false;
            uint32_t now = micros();
            double input = *myInput;
            error = *mySetpoint - input;
            double dInput = (input - lastInput);
            
            outputSum += (ki * error);
            i_term = outputSum;  // sc

            if (!pOnE) {  // Add Proportional on Measurement, of P_ON_M is specified
                p_term = -1 * kp * dInput;  // sc
                outputSum += p_term;
            }
            
            if (outputSum > outMax) outputSum = outMax;
            else if (outputSum < outMin) outputSum = outMin;

            double output;
            if (pOnE) {
                p_term = kp * error;  // sc
                output = p_term;  // Add Proportional on Error, if P_ON_E is specified
            }
            else output = 0;

            d_term = kd * dInput;  // sc
            output += outputSum - d_term;

            if (output > outMax) output = outMax;
            else if (output < outMin) output = outMin;
            *myOutput = output;

            // printf(", setp=%10.4lf, err=%10.4lf, pt=%10.4lf, it=%10.4lf, dt=%10.4lf, delt=%10.4lf, out=%10.4lf, kp=%10.4lf, ki=%10.4lf, kd=%10.4lf", *mySetpoint, error, p_term, i_term, d_term, delta, *myOutput, kp, ki, kd);

            lastInput = input;  // Remember some variables for next time
            lastTime = now;
            return true;
        }
        void SetTunings(double Kp, double Ki, double Kd) {
            if (Kp < 0 || Ki < 0 || Kd < 0) return;
            dispKp = Kp; dispKi = Ki; dispKd = Kd;

            double SampleTimeInSec = ((double)SampleTime)/1000000;  // sc changed millis() to micros() and 1000 to 1000000 here

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