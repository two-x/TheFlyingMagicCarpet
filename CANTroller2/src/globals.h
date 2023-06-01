#ifndef GLOBALS_H
#define GLOBALS_H
#include <stdio.h>
#ifdef DUE
    #include <LibPrintf.h>  // This works on Due but not ESP32
#endif
#include "Arduino.h"
#include "devices.h"
#include "spid.h"
#include "ui.h"

#define arraysize(x)  ((int32_t)(sizeof(x) / sizeof((x)[0])))  // A macro function to determine the length of string arrays
#define floor(amt, lim) ( (amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)
//define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//define min(a, b) ( (a <= b) ? a : b)
//define max(a, b) ( (a >= b) ? a : b)

enum dataset_pages {LOCK, JOY, CAR, PWMS, BPID, GPID, CPID};
enum runmodes {BASIC, SHUTDOWN, STALL, HOLD, FLY, CRUISE};
enum tuning_ctrl_states {OFF, SELECT, EDIT};
enum ctrls { HOTRC };  // This is a bad hack. Since JOY is already enum'd as 1 for dataset pages
enum ctrl_axes { HORZ, VERT };
enum ctrl_thresh { MIN, DB, MAX };
enum ctrl_edge { BOT, TOP };
enum raw_filt { RAW, FILT };
enum encoder_inputs {A, B};
enum encodersw_presses { NONE, SHORT, LONG };
// enum sensor_sources { SENSOR, TOUCH, POT, LAST };

#define adc_bits 12
#define adc_range_adc 4095    // = 2^12-1
#define adc_midscale_adc 2047
#define serial_debugging false
#define print_timestamps false  // Makes code write out timestamps throughout loop to serial port

// Code monitoring/debug
Timer loopTimer(1000000);  // how long the previous main loop took to run (in us)
int32_t loop_period_us = 100000;
int32_t loop_freq_hz = 1;  // run loop real time frequency (in Hz)
int32_t loopno = 1;    
int32_t loopzero = 0;
Timer heartbeatTimer(1000000);
int32_t heartbeat_state = 0;
int32_t heartbeat_ekg[4] = { 150000, 100000, 430000, 1100000 };
bool heartbeat_pulse = HIGH;

// Flight mode state machine related
int32_t runmode = SHUTDOWN;
int32_t oldmode = runmode;  // So we can tell when the mode has just changed
bool shutdown_complete = true;  // Shutdown mode has completed its work and can stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
int32_t motor_park_timeout_us = 3000000;  // If we can't park the motors faster than this, then give up.
bool park_the_motors = false;  // Indicates we should release the brake & gas so the pedals can be used manually without interference
Timer motorParkTimer;
bool panic_stop = false;

// External signals
bool ignition = LOW;
bool ignition_last = ignition;
bool basicmodesw = LOW;

// Cruise mode activation
Timer cruiseSwTimer;
bool cruise_sw = LOW;
bool cruise_sw_held = false;
bool cruise_gesturing = false;  // Is cruise mode enabled by gesturing?  Otherwise by press of cruise button
bool cruise_adjusting = false;
int32_t gesture_progress = 0;  // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
int32_t gesture_flytimeout_us = 500000;  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
Timer gestureFlyTimer(gesture_flytimeout_us);  // Used to keep track of time for gesturing for going in and out of fly/cruise modes
int32_t cruise_sw_timeout_us = 500000;  // how long do you have to hold down the cruise button to start cruise mode (in us)

// Potentiometer related
double pot_ema_alpha = 0.2;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t pot_min_adc = 100;
int32_t pot_max_adc = adc_range_adc-100;
int32_t pot_adc = 0;
int32_t pot_filt_adc = adc_midscale_adc;

// Telemetry
double battery_ema_alpha = 0.01;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t battery_max_mv = 16000;  // The max vehicle voltage we can sense. Design resistor divider to match. Must exceed max V possible.
int32_t battery_adc = 0;
int32_t battery_mv = 10000;
int32_t battery_filt_mv = 10000;

// Generic parameters
Timer sanityTimer(7000000);  // Allows code to fail in a sensible way after a delay if nothing is happening
int32_t default_pulse_margin_us = 22;  // Default margin of error for comparisons of pulsewidths (in us)
int32_t default_margin_adc = 12;  // Default margin of error for comparisons of adc values (ADC count 0-4095)
int32_t pwm_pulse_min_us = 500;
int32_t pwm_pulse_center_us = 1500;
int32_t pwm_pulse_max_us = 2500;

// Controller related
bool ctrl = HOTRC;  // Use HotRC controller to drive instead of joystick?
double ctrl_ema_alpha[2] = { 0.2, 0.1 };  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t ctrl_lims_adc[2][2][3] = { { { 3,  50, 4092 }, { 3,  75, 4092 } }, { { 9, 200, 4085 }, { 9, 200, 4085 } }, }; // [HOTRC, JOY] [HORZ, VERT], [MIN, DEADBAND, MAX] values as ADC counts
int32_t ctrl_db_adc[2][2];  // To store the top and bottom deadband values for each axis of selected controller
int32_t ctrl_pos_adc[2][2] = { { adc_midscale_adc, adc_midscale_adc }, { adc_midscale_adc, adc_midscale_adc} };  // [HORZ/VERT] [RAW/FILT]
bool joy_centered = false;
volatile int32_t hotrc_horz_pulse_us = 1500;
volatile int32_t hotrc_vert_pulse_us = 1500;
volatile bool hotrc_ch3_sw, hotrc_ch4_sw, hotrc_ch3_sw_event, hotrc_ch4_sw_event, hotrc_ch3_sw_last, hotrc_ch4_sw_last;
Timer hotrcPulseTimer;  // OK to not be volatile?

// Tachometer related
int32_t old_tach_time_us;
volatile int32_t tach_delta_us = 0;
int32_t tach_delta_impossible_us = 6500;  // 6500 us corresponds to about 10000 rpm, which isn't possible. Use to reject retriggers
Timer tachPulseTimer;  // OK to not be volatile?
int32_t gas_governor_percent = 95;  // Software governor will only allow this percent of full-open throttle (percent 0-100)
double engine_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t engine_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the engine is stopped (in us)
int32_t engine_idle_rpm = 700;  // Min value for engine hz, corresponding to low idle (in rpm)
int32_t engine_max_rpm = 6000;  // Max possible engine rotation speed
int32_t engine_redline_rpm = 4000;  // Max value for engine_rpm, pedal to the metal (in rpm)
int32_t engine_margin_rpm = 15;  // Margin of error for checking engine rpm (in rpm)
int32_t engine_spike_thresh_rpm = 500;  // min pressure delta between two readings considered a spike to ignore (in rpm)
int32_t engine_lp_thresh_rpm = 1000;   // max delta acceptable over three consecutive readings (in rpm)
int32_t engine_rpm = 50;  // Current engine speed, raw as sensed (in rpm)
int32_t engine_filt_rpm = 50;  // Current engine speed, filtered (in rpm)
int32_t engine_last_rpm = 50;  // Engine speed from previous loop (in rpm)
int32_t engine_old_rpm = 50; // Engine speed from two loops back (in rpm)
int32_t engine_govern_rpm = map(gas_governor_percent, 0, 100, 0, engine_redline_rpm);  // Create an artificially reduced maximum for the engine speed
int32_t engine_target_rpm = 0;  // Stores new setpoint to give to the pid loop (gas)

// Throttle related
int32_t gas_pulse_delta_us;
int32_t gas_pulse_out_us = gas_pulse_idle_us;  // pid loop output to send to the actuator (gas)
int32_t gas_pulse_govern_us = map(gas_governor_percent*(engine_redline_rpm-engine_idle_rpm)/engine_redline_rpm, 0, 100, gas_pulse_idle_us, gas_pulse_redline_us);  // Governor must scale the pulse range proportionally
int32_t gas_pulse_ccw_max_us = 2000;  // Servo ccw limit pulsewidth. Hotrc controller ch1/2 min(lt/br) = 1000us, center = 1500us, max(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
int32_t gas_pulse_cw_max_us = 1000;  // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
int32_t gas_pulse_redline_us = 1400;  // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
int32_t gas_pulse_idle_us = 1800;  // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
int32_t gas_pulse_park_slack_us = 30;  // Gas pulsewidth beyond gas_pulse_idle_us where to park the servo out of the way so we can drive manually (in us)
double gas_kp_1k = 64;  // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
double gas_kp_mhz = 15;  // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
double gas_kd_ms = 22;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
int32_t gas_pid_dir = REV;  // 0 = fwd, 1 = rev. Because a higher value on the gas servo pulsewidth causes a decrease in engine rpm

// PID loop common parameters
int32_t pid_period_us = 100000;  // time period between output updates. Reciprocal of pid frequency (in us)
Timer pidTimer(pid_period_us);

// Steering related
int32_t steer_pulse_right_max_us = 500;  // Smallest pulsewidth acceptable to jaguar
int32_t steer_pulse_left_max_us = 2500;  // Longest pulsewidth acceptable to jaguar
int32_t steer_pulse_right_us = 800;  // Steering pulsewidth corresponding to full-speed right steering (in us)
int32_t steer_pulse_stop_us = pwm_pulse_center_us;  // Steering pulsewidth corresponding to zero steering motor movement (in us)
int32_t steer_pulse_left_us = 2200;  // Steering pulsewidth corresponding to full-speed left steering (in us)
int32_t steer_safe_percent = 72;  // Sterring is slower at high speed. How strong is this effect 
int32_t steer_pulse_safe_us = 0;
int32_t steer_pulse_out_us = steer_pulse_stop_us;  // pid loop output to send to the actuator (steering)

// Rotary encoder related
Timer encoderSpinspeedTimer;  // Used to figure out how fast we're spinning the knob.  OK to not be volatile?
volatile int32_t encoder_bounce_danger = B;  // Which of the encoder A or B inputs is currently untrustworthy due to bouncing 
volatile int32_t encoder_delta = 0;  // Keeps track of un-handled rotary clicks of the encoder.  Positive for CW clicks, Negative for CCW. 
volatile bool encoder_a_stable = true;  //  Stores the value of encoder A pin as read during B pin transition (where A is stable)
volatile int32_t encoder_spinrate_isr_us = 100000;  // Time elapsed between last two detents
Timer encoderLongPressTimer(800000);  // Used to time long button presses
int32_t encoder_spinrate_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_spinrate_min_us = 2500;  // Will reject spins faster than this as an attempt to debounce behavior
int32_t encoder_spinrate_last_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_spinrate_old_us = 1000000;  // How many us elapsed between the last two encoder detents? realistic range while spinning is 5 to 100 ms I'd guess
int32_t encoder_edits_per_det = 1;  // How many edits per detent. How much change happens per rotation detent
bool encoder_sw = false;  // Remember whether switch is being pressed
int32_t encoder_sw_action = NONE;  // Flag for encoder handler to know an encoder switch action needs to be handled
bool encoder_timer_active = false;  // Flag to prevent re-handling long presses if the sw is just kept down
bool encoder_suppress_click = false;  // Flag to prevent a short click on switch release after successful long press
int32_t encoder_state = 0;
int32_t encoder_counter = 0;
bool encoder_b_raw = digitalRead(encoder_b_pin);  // To store value of encoder pin value
bool encoder_a_raw = digitalRead(encoder_a_pin);

// Brake position related
int32_t brake_hold_initial_adc = 1200;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
int32_t brake_hold_increment_adc = 25;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
int32_t brake_panic_initial_adc = 1800;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
int32_t brake_panic_increment_adc = 50;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
Timer brakeIntervalTimer(500000);  // How much time between increasing brake force during auto-stop if car still moving?
int32_t brake_increment_interval_us = 500000;  // How often to apply increment during auto-stopping (in us)
double brake_pos_ema_alpha = 0.25;
int32_t brake_pos_nom_lim_retract_adc = 153;  // Retract limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
int32_t brake_pos_abs_min_retract_adc = 175;  // Retract value corresponding with the absolute minimum retract actuator is capable of. (ADC count 0-4095)
int32_t brake_pos_zeropoint_adc = 1500;  // ++ Brake position value corresponding to the point where fluid PSI hits zero (ADC count 0-4095)
int32_t brake_pos_park_adc = 1750;  // Best position to park the actuator out of the way so we can use the pedal (ADC count 0-4095)
int32_t brake_pos_nom_lim_extend_adc = 2500;  // Extend limit during nominal operation. Brake motor is prevented from pushing past this. (ADC count 0-4095)
int32_t brake_pos_abs_max_extend_adc = 3076;  // Extend value corresponding with the absolute max extension actuator is capable of. (ADC count 0-4095)
int32_t brake_pos_margin_adc = 10;  //
int32_t brake_pos_adc = brake_pos_park_adc;
int32_t brake_pos_filt_adc = brake_pos_adc;
int32_t brake_pid_dir = REV;  // 0 = fwd, 1 = rev. Because a higher value on the brake actuator pulsewidth causes a decrease in pressure value

// Brake pressure related
int32_t pressure_target_adc = 0;  // Stores new setpoint to give to the pid loop (brake)
double d_pressure_target_adc = (double)pressure_target_adc;
int32_t pressure_filt_adc = 0;  // Stores new setpoint to give to the pid loop (brake)
double d_pressure_filt_adc = (double)pressure_filt_adc;
int32_t pressure_old_adc  = adc_midscale_adc;  // Some pressure reading history for noise handling (-2)
double pressure_ema_alpha = 0.1;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t d_pressure_min_adc = 658;  // Brake pressure when brakes are effectively off. Sensor min = 0.5V, scaled by 3.3/4.5V is 0.36V of 3.3V (ADC count 0-4095). 230430 measured 658 adc (0.554V) = no brakes
int32_t d_pressure_max_adc = 2100;  // Highest possible pressure achievable by the actuator (ADC count 0-4095). 230430 measured 2080 adc (1.89V) is as hard as chris can push (wimp)
int32_t pressure_margin_adc = 12;  // Margin of error when comparing brake pressure adc values (ADC count 0-4095)
int32_t pressure_spike_thresh_adc = 60;  // min pressure delta between two readings considered a spike to ignore (ADC count 0-4095)
int32_t pressure_lp_thresh_adc = 1200;   // max delta acceptable over three consecutive readings (ADC count 0-4095)

// Brake actuator related
int32_t brake_pulse_out_us = brake_pulse_stop_us;  // sets the pulse on-time of the brake control signal. about 1500us is stop, higher is fwd, lower is rev
double d_brake_pulse_out_us = (double)brake_pulse_out_us;
int32_t brake_pulse_retract_max_us = 500;  // Smallest pulsewidth acceptable to jaguar
int32_t brake_pulse_extend_max_us = 2500;  // Longest pulsewidth acceptable to jaguar
int32_t brake_pulse_retract_us = 650;  // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us)
int32_t brake_pulse_stop_us = pwm_pulse_center_us;  // Brake pulsewidth corresponding to center point where motor movement stops (in us)
int32_t brake_pulse_extend_us = 2350;  // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us)
int32_t brake_pulse_margin_us = 40; // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated 
double brake_kp_1k = 588;  // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
double brake_kp_mhz = 135;  // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
double brake_kd_ms = 91;  // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)


// Carspeed related
double speedo_delta_impossible_us = 4500;  // 4500 us corresponds to about 40 mph, which isn't possible. Use to reject retriggers
double car_stop_timeout_us = 400000;  // Time after last magnet pulse when we can assume the car is stopped (in us)
int32_t carspeed_spike_thresh_mmph = 1500;  // min pressure delta between two readings considered a spike to ignore (in milli-mph)
int32_t carspeed_lp_thresh_mmph = 3000;   // max delta acceptable over three consecutive readings (in milli-mph)
int32_t carspeed_idle_mmph = 4500;  // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)
double carspeed_ema_alpha = 0.015;  // alpha value for ema filtering, lower is more continuous, higher is more responsive (0-1). 
int32_t carspeed_redline_mmph = 15000;  // What is our steady state speed at redline? Pulley rotation frequency (in milli-mph)
int32_t carspeed_max_mmph = 25000;  // What is max speed car can ever go
int32_t carspeed_govern_mmph = map(gas_governor_percent, 0, 100, 0, carspeed_redline_mmph);  // Governor must scale the top vehicle speed proportionally
int32_t carspeed_target_mmph = 0.0;  // Stores new setpoint to give to the pid loop (cruise) in milli-mph
int32_t carspeed_mmph = 10;  // Current car speed, raw as sensed (in mmph)
int32_t carspeed_filt_mmph = 10;  // Current car speed, filtered (in mmph)
int32_t carspeed_last_mmph = 10;  // Car speed from previous loop (in mmph)
int32_t carspeed_old_mmph = 10;  // Car speed from two loops back (in mmph)
double cruise_kp_1k = 157;  // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
double cruise_kp_mhz = 35;  // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
double cruise_kd_ms = 44;  // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)
int32_t cruise_pid_dir = FWD;  // 0 = fwd,. Because a higher val 1 = revue on the engine rpm causes an increase in car speed
// volatile int32_t speedo_delta_us = 0;
// Timer speedoPulseTimer;  // OK to not be volatile?

AnalogSensor Pressure(d_pressure_filt_adc);
PulseSensor Speedo(speedo_pulse_pin, speedo_delta_impossible_us, car_stop_timeout_us); //  pin, impossible, stop_timeout);
PulseSensor Tach(tach_pulse_pin, tach_delta_impossible_us, engine_stop_timeout_us); //  pin, impossible, stop_timeout);

SPID brakeSPID(brake_kp_1k, brake_kp_mhz, brake_kd_ms, brake_pid_dir, pid_period_us);
SPID gasSPID(gas_kp_1k, gas_kp_mhz, gas_kd_ms, gas_pid_dir, pid_period_us);
// SPID cruiseSPID(cruise_kp_1k, cruise_kp_mhz, cruise_kd_ms, cruise_pid_dir, pid_period_us);

// AnalogSensor MuleBatt();
// AnalogSensor BrakePos();
// AnalogSensor Pot();
// RotaryEncoder Encoder();
// InToggle BasicSw();
// OutToggle Ignition();
// Outtoggle SysPower();
// Outtoggle LedRx();
// Outtoggle LedTx();
// Outtoggle LedHeart();
// Outtoggle SysPower();

PulseSensor Tach(tach_pulse_pin, tach_delta_impossible_us, tach_stop_timeout_us);
PulseSensor Speedo(speedo_pulse_pin, speedo_delta_impossible_us, speedo_stop_timeout_us);
ServoPWM gasServo(gas_pwm_pin, gas_pulse_idle_us, gas_pulse_redline_us);
ServoPWM HotrcCh3();
ServoPWM HotrcCh4();
JagMotor steerMotor();
JagMotor BrakeMotor();

// ControlLoop Steering();
// ControlLoop Braking();
// ControlLoop Throttle();
// ControlLoop Cruise();

// static Servo gas_servo;
// static Servo steer_servo;  // Lets us set pwm outputs given an on-time pulse width in us

// ema : pass in a fresh raw value, the previously filtered value, and alpha factor, returns new filtered value
int32_t ema(int32_t raw_value, int32_t filt_value, double alpha) {
    return (int32_t)((alpha*(double)raw_value) + ((1-alpha)*(double)filt_value));
}
double ema(int32_t raw_value, double filt_value, double alpha) {
    return (int32_t)((alpha*(double)raw_value) + ((1-alpha)*filt_value));
}

// int* x is c++ style, int *x is c style
void adj_val(int32_t* variable, int32_t modify, int32_t low_limit, int32_t high_limit) {
    if (*variable + modify < low_limit) *variable = low_limit;
    else if (*variable + modify > high_limit) *variable = high_limit;
    else *variable += modify; 
}

enum simulatable { GLOBAL, LAST, POT, CTRL, BRKPOS, PRESS, TACH, SPEEDO, BASICSW, IGN, CRUISESW, };
// enum io_list { POT, ENCODER, JOY, HOTRC, BRKPOS, PRESS, TACH, SPEEDO, BASICSW, IGN, CRUISESW, STEERMTR, BRKMTR, GASSERVO};
// Device* devs[14];
int32_t sim_en[11] = { 0, 0, 0,   1,     1,    1,      1,       1,   1,        1 };
// int32_t ui_pot_addrs[11];
// int32_t *ui_sim_pot = &pot_adc;

// AnalogSens Pot("Pot", )

void sim_source_change(int32_t* sensarray, int32_t* simarray, int32_t index, int32_t adj_dir) {
    if (adj_dir) simarray[index] = constrain (simarray[index] + (adj_dir > 0), 0, 1);


    // int32_t max_val;
    // if (ui_pot_addrs[index] == &pot_adc) max_val = 1;
    // else max_val = 2;
    

    // ui_sim_pot = ui_pot_addrs[index];
    // if (ui_sim_pot == nullptr) *ui_sim_pot = ui_pot_addrs[POT];

    // if (sensor >=3 && sensor <= 6) max_val = POT;
    // ui_sim_pot
    // max_val == TOUCH;
    // if (sensor >=3 && sensor <= 6) max_val = POT;
    // *sensor_array[sensor] += constrain(adj_dir, -1, 1);

}
int32_t mycros(void) {  // This is "my" micros() function that returns signed int32
    uint32_t temp = micros();
    return (int32_t)(temp &= 0x7fffffff);  // Note this overflows every 35 min due to being only 31 bits. 
}
// Interrupt service routines
//
void encoder_a_isr(void) {  // When A goes high if B is low, we are CW, otherwise we are CCW -- This ISR intended for encoders like the one on the tan proto board
    if (encoder_bounce_danger != A) {  // Prevents taking action on any re-triggers after a valid trigger due to bouncing
        if (!encoder_a_stable) {  // Since A just transitioned, if a_stable is low, this is a rising edge = time to register a turn 
            encoder_delta += digitalRead(encoder_b_pin) ? -1 : 1;  // Create turn event to be handled later. If B=0, delta=-1 (CCW) turn decreases things
            encoder_counter += encoder_delta;  // Just used to aid in debugging this isr
            encoder_spinrate_isr_us = encoderSpinspeedTimer.elapsed();
            encoderSpinspeedTimer.reset();
        }
        encoder_bounce_danger = A;  // Set to reject A retriggers and enable B trigger
    }
}
void encoder_b_isr(void) {  // On B rising or falling edge, A should have stabilized by now, so don't ignore next A transition
    if (encoder_bounce_danger != B) {  // Prevents taking action on any re-triggers after a valid trigger due to bouncing
        encoder_a_stable = digitalRead(encoder_a_pin);  // Input A is stable by the time B changes, so read A value here
        
        encoder_bounce_danger = B;  // Set to reject B retriggers and enable A trigger
    }
}
// The tach and speed use a hall sensor being triggered by a passing magnet once per pulley turn. These ISRs call mycros()
// on every pulse to know the time since the previous pulse. I tested this on the bench up to about 750 mmph which is as 
// fast as I can move the magnet with my hand, and it works. It would be cleaner to just increment a counter here in the ISR
// then call mycros() in the main loop and compare with a timer to calculate mmph.
void tach_isr(void) {  // The tach and speedo isrs compare value returned from the mycros() function with the value from the last interrupt to determine period, to get frequency of the vehicle pulley rotations.
    int32_t temp_us = tachPulseTimer.elapsed();
    if (temp_us > tach_delta_impossible_us) {
        tach_delta_us = temp_us;    
        tachPulseTimer.reset();
    }
}
void speedo_isr(void) {  //  A better approach would be to read, reset, and restart a hardware interval timer in the isr.  Better still would be to regularly read a hardware binary counter clocked by the pin - no isr.
    Speedo.isr();
    // int32_t temp_us = speedoPulseTimer.elapsed();
    // if (temp_us > speedo_delta_impossible_us) {
    //     speedo_delta_us = temp_us;    
    //     speedoPulseTimer.reset();
    // }
}
void hotrc_horz_isr(void) {  // Reads ranged PWM signal on an input pin to determine control position. This ISR sets timer for all hotrc isrs on hi-going edge
    if (digitalRead(hotrc_horz_pin)) hotrcPulseTimer.reset();
    else hotrc_horz_pulse_us = hotrcPulseTimer.elapsed();
}
void hotrc_vert_isr(void) {  // On falling edge, reads ranged PWM signal on an input pin to determine control position
    hotrc_vert_pulse_us = hotrcPulseTimer.elapsed();
}
void hotrc_ch3_isr(void) {  // Reads a binary switch encoded as PWM on an input pin to determine button toggle state
    hotrc_ch3_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch3 switch true if short pulse, otherwise false
    if (hotrc_ch3_sw != hotrc_ch3_sw_last) hotrc_ch3_sw_event = true;  // So a handler routine can be signaled
    hotrc_ch3_sw_last = hotrc_ch3_sw;
}
void hotrc_ch4_isr(void) {  // Reads PWM signal on an input pin to determine control position
    hotrc_ch4_sw = (hotrcPulseTimer.elapsed() <= 1500);  // Ch4 switch true if short pulse, otherwise false
    if (hotrc_ch4_sw != hotrc_ch4_sw_last) hotrc_ch4_sw_event = true;  // So a handler routine can be signaled
    hotrc_ch4_sw_last = hotrc_ch4_sw;
}
// diagnostic()  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
//
// This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
// informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
// retreive with an OBD tool. Eventually this should include functions allowing us to detect things like:
//  1. A sensor or actuator is unplugged, movement blocked, missing magnetic pulses, etc.
//  2. Air in the brake lines.
//  3. Axle/brake drum may be going bad (increased engine RPM needed to achieve certain carspeed)  (beware going up hill may look the same).
//  4. E-brake has been left on (much the same symptoms as above? (beware going up hill may look the same) 
//  5. Battery isn't charging, or just running low.
//  6. Carburetor not behaving (or air filter is clogged). (See above about engine deiseling - we can detect this!)
//  7. After increasing braking, the actuator position changes in the opposite direction, or vise versa.
//  8. Changing an actuator is not having the expected effect.
//  9. A tunable value suspected to be out of tune.
//  10. Check regularly for ridiculous shit. Compare our variable values against a predetermined list of conditions which shouldn't be possible or are at least very suspect. For example:
//     A) Sensor reading is out of range, or has changed faster than it ever should.
//     B) Stopping the car on entering hold/shutdown mode is taking longer than it ever should.
//     C) Mule seems to be accelerating like a Tesla.
//     D) Car is accelerating yet engine is at idle.
//  11. The control system has nonsensical values in its variables.
//
void diagnostic() {
    if (!ignition && engine_filt_rpm > 0) { // See if the engine is turning despite the ignition being off
        Serial.println(F("Detected engine rotation in the absense of ignition signal"));  // , engine_filt_rpm, ignition
        // I don't think we really need to panic about this, but it does seem curious. Actually this will probably occur when we're sliding
        // into camp after a ride, and kill the engine before we stop the car. For a fraction of a second the engine would keep turning anyway.
        // Or fopr that matter whenever the carb is out of tune and making the engine diesel after we kill the ign.
    }
}

// Here are the different runmodes documented
//
// ** Basic Mode **
// - Required: BasicMode switch On
// - Priority: 1 (Highest)
// The gas and brake don't do anything in Basic Mode. Just the steering works, so use the pedals.
// This mode is enabled with a toggle switch in the controller box.  When in Basic Mode, the only
// other valid mode is Shutdown Mode. Shutdown Mode may override Basic Mode.
// - Actions: Release and deactivate brake and gas actuators.  Steering PID keep active 
//
// ** Shutdown Mode **
// - Required: BasicMode switch Off & Ignition Off
// - Priority: 2
// This mode is active whenever the ignition is off.  In other words, whenever the
// little red pushbutton switch by the joystick is unclicked.  This happens before the
// ignition is pressed before driving, but it also may happen if the driver needs to
// panic and E-stop due to loss of control or any other reason.  The ignition will get cut
// independent of the controller, but we can help stop the car faster by applying the
// brakes. Once car is stopped, we release all actuators and then go idle.
// - Actions: 1. Release throttle. If car is moving AND BasicMode Off, apply brakes to stop car
// - Actions: 2: Release brakes and deactivate all actuators including steering
//
// ** Stall Mode **
// - Required: Engine stopped & BasicMode switch Off & Ignition On
// - Priority: 3
// This mode is active when the engine is not running.  If car is moving, then it presumably may
// coast to a stop.  The actuators are all enabled and work normally.  Starting the engine will 
// bring you into Hold Mode.  Shutdown Mode and Basic Mode both override Stall Mode. Note: This
// mode allows for driver to steer while being towed or pushed, or working on the car.
// - Actions: Enable all actuators
//
// ** Hold Mode **
// - Required: Engine running & JoyVert<=Center & BasicMode switch Off & Ignition On
// - Priority: 4
// This mode is entered from Stall Mode once engine is started, and also, whenever the car comes
// to a stop while driving around in Fly Mode.  This mode releases the throttle and will 
// continuously increase the brakes until the car is stopped, if it finds the car is moving. 
// Pushing up on the joystick from Hold mode releases the brakes & begins Fly Mode.
// Shutdown, Basic & Stall Modes override Hold Mode.
// Actions: Close throttle, and Apply brake to stop car, continue to ensure it stays stopped.
//
// ** Fly Mode **
// - Required: (Car Moving OR JoyVert>Center) & In gear & Engine running & BasicMode Off & Ign On
// - Priority: 5
// This mode is for driving under manual control. In Fly Mode, vertical joystick positions
// result in a proportional level of gas or brake (AKA "Manual" control).  Fly Mode is
// only active when the car is moving - Once stopped or taken out of gear, we go back to Hold Mode.
// If the driver performs a special secret "cruise gesture" on the joystick, then go to Cruise Mode.
// Special cruise gesture might be: Pair of sudden full-throttle motions in rapid succession
// - Actions: Enable all actuators, Watch for gesture
//
// ** Cruise Mode **
// - Required: Car Moving & In gear & Engine running & BasicMode switch Off & Ignition On
// - Priority: 6 (Lowest)
// This mode is entered from Fly Mode by doing a special joystick gesture. In Cruise Mode,
// the brake is disabled, and the joystick vertical is different: If joyv at center, the
// throttle will actively maintain current car speed.  Up or down momentary joystick presses
// serve to adjust that target speed. A sharp, full-downward gesture will drop us back to 
// Fly Mode, promptly resulting in braking (if kept held down).
// - Actions: Release brake, Maintain car speed, Handle joyvert differently, Watch for gesture

#endif  // GLOBALS_H