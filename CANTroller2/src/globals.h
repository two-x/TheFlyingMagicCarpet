#pragma once
#include <ESP32Servo.h>        // Makes PWM output to control motors (for rudimentary control of our gas and steering)
#include <DallasTemperature.h>
#include <Wire.h>
#include <Preferences.h>
#include <iostream>
#include <iomanip>  // For formatting console cout strings
#include "utils.h"
#include "qpid.h" // This is quickpid library except i have to edit some of it
#include "uictrl.h"
#include "neo.h"
#include "devices.h"
#include "temperature.h"

bool starter_signal_support = true;
bool remote_start_support = true;
bool autostop_disabled = false;       // Temporary measure to keep brake behaving until we get it debugged. Eventually should be false
bool allow_rolling_start = false;    // May be a smart prerequisite, may be us putting obstacles in our way
bool flip_the_screen = true;
// Dev-board-only options:  Note these are ignored and set false at boot by set_board_defaults() unless running on a breadboard with a 22k-ohm pullup to 3.3V the TX pin
bool usb_jtag = true;  // If you will need the usb otg port for jtag debugging (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/jtag-debugging/configure-builtin-jtag.html)
bool dont_take_temperatures = false;  // In case debugging dallas sensors or causing problems
bool gamma_correct_enabled = false;
bool console_enabled = true;         // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
bool keep_system_powered = false;    // Use true during development
bool screensaver = false;  // Can enable experiment with animated screen draws
bool looptime_print = false;         // Makes code write out timestamps throughout loop to serial port
bool touch_reticles = false;
uint32_t looptime_linefeed_threshold = 0;  // Leaves prints of loops taking > this for analysis. Set to 0 prints every loop

// In case we ever talk to jaguars over asynchronous serial port, replace with this:
// #include <HardwareSerial.h>
// HardwareSerial jagPort(1); // Open serisl port to communicate with jaguar controllers for steering & brake motors

// run state machine related
bool running_on_devboard = false;    // will overwrite with value read thru pull resistor on tx pin at boot
bool shutdown_incomplete = true;     // minor state variable for shutdown mode - Shutdown mode has not completed its work and can't yet stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;        // Indicates we should release the brake & gas so the pedals can be used manually without interference
bool autostopping = false, autostopping_last = false;           // true when in process of stopping the car (hold or shutdown modes)
bool car_hasnt_moved = false;        // minor state variable for fly mode - Whether car has moved at all since entering fly mode
bool calmode_request = false;
bool joy_centered = false;  // minor state variable for hold mode
bool powering_up = false;  // minor state variable for asleep mode
// Timer cruiseSwTimer;  // Was used to require a medium-length hold time pushing cruise button to switch modes
Timer sleepInactivityTimer(180000000);           // After shutdown how long to wait before powering down to sleep
Timer stopcarTimer(8000000);                    // Allows code to fail in a sensible way after a delay if nothing is happening
uint32_t motor_park_timeout_us = 4000000;       // If we can't park the motors faster than this, then give up.
uint32_t gesture_flytimeout_us = 1250000;        // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
Timer gestureFlyTimer(gesture_flytimeout_us); // Used to keep track of time for gesturing for going in and out of fly/cruise modes
Timer motorParkTimer(motor_park_timeout_us);
bool simulating_last = false;
int32_t idelta = 0, idelta_touch = 0, idelta_encoder = 0;
float fdelta = 0.0;

// calibration related
bool cal_joyvert_brkmotor_mode = false; // Allows direct control of brake motor using controller vert
bool cal_pot_gasservo_mode = false;     // Allows direct control of gas servo using pot. First requires pot to be in valid position before mode is entered
bool cal_pot_gasservo_ready = false;    // Whether pot is in valid range

enum temp_categories { AMBIENT = 0, ENGINE = 1, WHEEL = 2, num_temp_categories };
enum temp_lims { DISP_MIN, OP_MIN, OP_MAX, WARNING, ALARM, DISP_MAX }; // Possible sources of gas, brake, steering commands
float temp_lims_f[3][6]{
    {0.0, 45.0, 115.0, 120.0, 130.0, 220.0},  // [AMBIENT][DISP_MIN/OP_MIN/OP_MAX/WARNING/ALARM]
    {0.0, 178.0, 198.0, 202.0, 205.0, 220.0}, // [ENGINE][DISP_MIN/OP_MIN/OP_MAX/WARNING/ALARM]
    {0.0, 50.0, 120.0, 130.0, 140.0, 220.0},  // [WHEEL][DISP_MIN/OP_MIN/OP_MAX/WARNING/ALARM] (applies to all wheels)
};
float temp_room = 77.0;          // "Room" temperature is 25 C = 77 F  Who cares?
float temp_sensor_min_f = -67.0; // Minimum reading of sensor is -25 C = -67 F
float temp_sensor_max_f = 257.0; // Maximum reading of sensor is 125 C = 257 F
bool temp_err[num_temp_categories];  // [AMBIENT/ENGINE/WHEEL]

// steering related
float steer_safe_pc = 72.0; // Steering is slower at high speed. How strong is this effect
float steer_out_pc, steer_safe_adj_pc;
float steer_right_max_pc = 100.0;
float steer_right_pc = 100.0;
float steer_stop_pc = 0.0;
float steer_left_pc = -100.0;
float steer_left_min_pc = -100.0;
float steer_margin_pc = 2.4;
float steer_left_min_us = 670;   // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float steer_left_us = 670;       // Steering pulsewidth corresponding to full-speed right steering (in us). Default setting for jaguar is max 670us
float steer_stop_us = 1500;      // Steering pulsewidth corresponding to zero steering motor movement (in us)
float steer_right_us = 2330;     // Steering pulsewidth corresponding to full-speed left steering (in us). Default setting for jaguar is max 2330us
float steer_right_max_us = 2330; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float steer_out_us = steer_stop_us;              // pid loop output to send to the actuator (steering)
uint32_t steer_pid_period_us = 75000;    // (Not actually a pid) Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer steerPidTimer(steer_pid_period_us); // not actually tunable, just needs value above

// brake pressure related
float pressure_hold_initial_psi = 45;  // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
float pressure_hold_increment_psi = 3;  // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
float pressure_panic_initial_psi = 80; // Pressure initially applied when brakes are hit to auto-stop the car (ADC count 0-4095)
float pressure_panic_increment_psi = 5; // Incremental pressure added periodically when auto stopping (ADC count 0-4095)
float pressure_target_psi;

// brake actuator motor related
Timer brakeIntervalTimer(1000000);             // How much time between increasing brake force during auto-stop if car still moving?
int32_t brake_increment_interval_us = 1000000; // How often to apply increment during auto-stopping (in us)
float brake_duty_pc = 25.0;  // From motor datasheet
float brake_extend_absmin_pc = -100.0; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float brake_stop_pc = 0.0;          // Brake pulsewidth corresponding to center point where motor movement stops (in us)
float brake_retract_absmax_pc = 100.0; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float brake_margin_pc = 1.8;        // If pid pulse calculation exceeds pulse limit, how far beyond the limit is considered saturated
float brake_extend_absmin_us = 670; // Smallest pulsewidth acceptable to jaguar (if recalibrated) is 500us
float brake_stop_us = 1500;       // Brake pulsewidth corresponding to center point where motor movement stops (in us)
float brake_retract_absmax_us = 2330; // Longest pulsewidth acceptable to jaguar (if recalibrated) is 2500us
float brake_out_pc = brake_stop_pc;
float brake_out_us = brake_stop_us;
float brake_retract_effective_max_us;   // 
float brake_extend_min_pc = brake_extend_absmin_pc * brake_duty_pc / 100.0;
float brake_retract_max_pc = brake_retract_absmax_pc * brake_duty_pc / 100.0;
float brake_extend_min_us = brake_stop_us - brake_duty_pc * (brake_stop_us - brake_extend_absmin_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained retraction of brake actuator (in us). Default setting for jaguar is max 670us
float brake_retract_max_us = brake_stop_us - brake_duty_pc * (brake_stop_us - brake_retract_absmax_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained extension of brake actuator (in us). Default setting for jaguar is max 2330us
uint32_t brake_pid_period_us = 85000;    // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer brakePidTimer(brake_pid_period_us); // not actually tunable, just needs value above
// float brake_perc_per_us = (100.0 - (-100.0)) / (brake_extend_min_us - brake_retractmx_us);  // (100 - 0) percent / (us-max - us-min) us = 1/8.3 = 0.12 percent/us
float brake_spid_initial_kp = 0.323;                                                                         // PID proportional coefficient (brake). How hard to push for each unit of difference between measured and desired pressure (unitless range 0-1)
float brake_spid_initial_ki_hz = 0.000;                                                                      // PID integral frequency factor (brake). How much harder to push for each unit time trying to reach desired pressure  (in 1/us (mhz), range 0-1)
float brake_spid_initial_kd_s = 0.000;                                                                       // PID derivative time factor (brake). How much to dampen sudden braking changes due to P and I infuences (in us, range 0-1)

// throttle servo related
float gas_governor_pc = 95;     // Software governor will only allow this percent of full-open throttle (percent 0-100)
Timer gasServoTimer(500000);    // We expect the servo to find any new position within this time
float gas_out_pc, gas_out_deg, gas_govern_deg;   // pid loop output to send to the actuator (gas)
bool gas_servo_reversed = false;  // true if higher pulsewidth goes ccw
float gas_absmax_us = 2500;       // Servo cw limit pulsewidth. Servo: full ccw = 2500us, center = 1500us , full cw = 500us
float gas_absmin_us = 500;     // Servo ccw limit pulsewidth. Hotrc controller ch1/2 smin(lt/br) = 1000us, center = 1500us, smax(rt/th) = 2000us (with scaling knob at max).  ch4 off = 1000us, on = 2000us
float gas_open_max_deg = 168.2;      // Gas pulsewidth corresponding to full open throttle with 180-degree servo (in us)
float gas_closed_min_deg = 45.0;  // Gas pulsewidth corresponding to fully closed throttle with 180-degree servo (in us)
float gas_parked_deg = 43.0;  // Gas pulsewidth beyond gas_ccw_closed_us where to park the servo out of the way so we can drive manually (in us)
uint32_t gas_pid_period_us = 22500;  // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
Timer gasPidTimer(gas_pid_period_us); // not actually tunable, just needs value above
float gas_spid_initial_kp = 0.013;    // PID proportional coefficient (gas) How much to open throttle for each unit of difference between measured and desired RPM  (unitless range 0-1)
float gas_spid_initial_ki_hz = 0.000; // PID integral frequency factor (gas). How much more to open throttle for each unit time trying to reach desired RPM  (in 1/us (mhz), range 0-1)
float gas_spid_initial_kd_s = 0.000;  // PID derivative time factor (gas). How much to dampen sudden throttle changes due to P and I infuences (in us, range 0-1)
bool gas_open_loop = true;

// carspeed/speedo related
float speedo_target_mph;
float speedo_govern_mph;       // Governor must scale the top vehicle speed proportionally. This is given a value in the loop
float speedo_idle_mph = 4.50;  // What is our steady state speed at engine idle? Pulley rotation frequency (in milli-mph)

// tach related
float tach_target_rpm, tach_govern_rpm;        // Software engine governor creates an artificially reduced maximum for the engine speed. This is given a value in calc_governor()
float tach_margin_rpm = 15.0; // Margin of error for checking engine rpm (in rpm)
float tach_idle_abs_min_rpm = 450.0;  // Low limit of idle speed adjustability
float tach_idle_hot_min_rpm = 550.0;  // Idle speed at op_max engine temp
float tach_idle_cold_max_rpm = 775.0; // Idle speed at op_min engine temp
float tach_idle_high_rpm = 950.0;     // Elevated rpm above idle guaranteed never to stall
float tach_idle_abs_max_rpm = 1000.0; // High limit of idle speed adjustability
Timer tachIdleTimer(5000000);         // How often to update tach idle value based on engine temperature

// Cruise : is active on demand while driving.
// Pick from 3 different styles of adjusting cruise setpoint. I prefer throttle_delta.
// pid_suspend_fly : (PID) Moving trigger from center disables pid and lets you adjust the rpm target directly like Fly mode does. Whatever speed you're at when trigger releases is new pid target  
// throttle_angle : Cruise locks throttle angle, instead of pid. Moving trigger from center adjusts setpoint proportional to how far you push it before releasing (and reduced by an attenuation factor)
// throttle_delta : Cruise locks throttle angle, instead of pid. Any non-center trigger position continuously adjusts setpoint proportional to how far it's pulled over time (up to a specified maximum rate)
enum cruise_modes { pid_suspend_fly, throttle_angle, throttle_delta };
cruise_modes cruise_setpoint_mode = throttle_delta;
bool cruise_speed_lowerable = true;  // Allows use of trigger to adjust cruise speed target without leaving cruise mode.  Otherwise cruise button is a "lock" button, and trigger activity cancels lock
bool cruise_adjusting = false;
int32_t cruise_delta_max_pc_per_s = 16;  // (in throttle_delta mode) What's the fastest rate cruise adjustment can change pulse width (in us per second)
// float cruise_angle_attenuator = 0.25;  // (in throttle_angle mode) Limits the change of each adjust trigger pull to this fraction of what's possible
float cruise_angle_attenuator = 0.016;  // (in throttle_angle mode) Limits the change of each adjust trigger pull to this fraction of what's possible
bool flycruise_toggle_request = false;
float flycruise_vert_margin_pc = 0.3; // Margin of error for determining hard brake value for dropping out of cruise mode
float gas_cruise_pc;  // Gas pulsewidth value fixed by cruise mode when in fixed throttle mode 
uint32_t cruise_pid_period_us = 85000;                                                                      // Needs to be long enough for motor to cause change in measurement, but higher means less responsive
float cruise_spid_initial_kp = 5.57;                                                                         // PID proportional coefficient (cruise) How many RPM for each unit of difference between measured and desired car speed  (unitless range 0-1)
float cruise_spid_initial_ki_hz = 0.000;                                                                     // PID integral frequency factor (cruise). How many more RPM for each unit time trying to reach desired car speed  (in 1/us (mhz), range 0-1)
float cruise_spid_initial_kd_s = 0.000;                                                                      // PID derivative time factor (cruise). How much to dampen sudden RPM changes due to P and I infuences (in us, range 0-1)

bool starter = LOW;  // Set by handler only. Reflects current state of starter signal (does not indicate source)
bool starter_drive = false;  // Set by handler only. High when we're driving starter, otherwise starter is an input
req starter_request = req_na;
Timer starterTimer(5000000);  // If remotely-started starting event is left on for this long, end it automatically  
bool syspower = HIGH;  // Set by handler only. Reflects current state of the signal
bool ignition = LOW;  // Set by handler only. Reflects current state of the signal
req ignition_request = req_na;
bool panicstop = false;
req panicstop_request = req_on;  // On powerup we assume the code just crashed during a drive, because it could have
Timer panicTimer(20000000);  // How long should a panic stop last?  We can't stay mad forever
bool basicmodesw = LOW;
enum sw_presses { NONE, SHORT, LONG }; // used by encoder sw and button algorithms
int8_t sleep_request = req_na;

// Instantiate objects
Preferences config;  // Persistent config storage
I2C i2c(i2c_sda_pin, i2c_scl_pin);
Hotrc hotrc;
Potentiometer pot(pot_wipe_pin);
Simulator sim(pot);
TemperatureSensorManager tempsens(onewire_pin);
Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
CarBattery mulebatt(mulebatt_pin);
LiPOBatt lipobatt(lipobatt_pin);
static Servo brakemotor, steermotor, gas_servo;
PressureSensor pressure(pressure_pin);
BrakePositionSensor brakepos(brake_pos_pin);
Speedometer speedo(speedo_pin);
Tachometer tach(tach_pin);
AirVeloSensor airvelo(i2c);
MAPSensor mapsens(i2c);  // map sensor related
ThrottleControl throttle(tach.human_ptr(), tach.filt_ptr(), tach_idle_high_rpm, tach_idle_hot_min_rpm, tach_idle_cold_max_rpm,
    temp_lims_f[ENGINE][OP_MIN], temp_lims_f[ENGINE][WARNING], 50, ThrottleControl::idlemodes::control);

qpid brake_pid(pressure.filt_ptr(), &brake_out_pc, &pressure_target_psi, brake_extend_min_pc, brake_retract_max_pc,  // input, target, output, output min, max variable references
    brake_spid_initial_kp, brake_spid_initial_ki_hz, brake_spid_initial_kd_s,                     // Kp, Ki, and Kd tuning constants
    qpid::pmod::onerr, qpid::dmod::onerr, qpid::awmod::cond, qpid::cdir::direct,  // settings  // roundcond, clamp
    brake_pid_period_us, qpid::ctrl::manual, qpid::centmod::on, brake_stop_pc); // period, more settings

qpid gas_pid(tach.filt_ptr(), &gas_out_pc, &tach_target_rpm, 0.0, 100.0,  // input, target, output variable references
    gas_spid_initial_kp, gas_spid_initial_ki_hz, gas_spid_initial_kd_s,  // Kp, Ki, and Kd tuning constants
    qpid::pmod::onerr, qpid::dmod::onerr, qpid::awmod::clamp, qpid::cdir::direct,              // settings
    gas_pid_period_us, (gas_open_loop) ? qpid::ctrl::manual : qpid::ctrl::manual, qpid::centmod::off); // period, more settings

qpid cruise_pid(speedo.filt_ptr(), &tach_target_rpm, &speedo_target_mph, throttle.idlespeed(), tach_govern_rpm,  // input, target, output variable references
    cruise_spid_initial_kp, cruise_spid_initial_ki_hz, cruise_spid_initial_kd_s,                 // Kp, Ki, and Kd tuning constants
    qpid::pmod::onerr, qpid::dmod::onerr, qpid::awmod::round, qpid::cdir::direct, // settings
    cruise_pid_period_us, qpid::ctrl::manual, qpid::centmod::off);

Brake brake;  // (int8_t _motor_pin, int8_t _press_pin, int8_t _posn_pin)

joydirs joydir(int8_t axis = VERT) {
    if (axis == VERT) return ((hotrc.pc[axis][FILT] > hotrc.pc[axis][DBTOP]) ? joy_up : ((hotrc.pc[axis][FILT] < hotrc.pc[axis][DBBOT]) ? joy_down : joy_cent));
    return ((hotrc.pc[axis][FILT] > hotrc.pc[axis][DBTOP]) ? joy_rt : ((hotrc.pc[axis][FILT] < hotrc.pc[axis][DBBOT]) ? joy_lt : joy_cent));
    // return (pc[axis][FILT] > pc[axis][DBTOP]) ? ((axis == VERT) ? joy_up : joy_rt) : (pc[axis][FILT] < pc[axis][DBBOT]) ? ((axis == VERT) ? joy_down : joy_lt) : joy_cent;
}

float gas_pc_to_deg(float _pc) {  // Eventually this should be linearized
    return gas_closed_min_deg + _pc * (gas_open_max_deg - gas_closed_min_deg) / 100.0;
}
float gas_deg_to_pc(float _deg) {  // Eventually this should be linearized
    return 100.0 * (_deg - gas_closed_min_deg) / (gas_open_max_deg - gas_closed_min_deg);
}
float gas_deg_to_us(float _deg) {
    return map(_deg, 0.0, 180.0, gas_servo_reversed ? gas_absmax_us : gas_absmin_us, gas_servo_reversed ? gas_absmin_us : gas_absmax_us);
}
float gas_pc_to_us(float _pc) {
    return gas_deg_to_us(gas_pc_to_deg(_pc));
}
void brake_calc_duty(float duty) {  // call from setup and whenever changing duty cycle
    brake_extend_min_pc = brake_extend_absmin_pc * duty / 100.0;     // Brake pulsewidth corresponding to full-speed extension of brake actuator (in us). Default setting for jaguar is max 2330us
    brake_retract_max_pc = brake_retract_absmax_pc * duty / 100.0;     // Brake pulsewidth corresponding to full-speed retraction of brake actuator (in us). Default setting for jaguar is max 670us
    brake_extend_min_us = brake_stop_us - duty * (brake_stop_us - brake_extend_absmin_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained retraction of brake actuator (in us). Default setting for jaguar is max 670us
    brake_retract_max_us = brake_stop_us - duty * (brake_stop_us - brake_retract_absmax_us) / 100.0;  // Brake pulsewidth corresponding to duty-constrained extension of brake actuator (in us). Default setting for jaguar is max 2330us
}
void calc_governor(void) {
    tach_govern_rpm = map(gas_governor_pc, 0.0, 100.0, 0.0, tach.redline_rpm()); // Create an artificially reduced maximum for the engine speed
    cruise_pid.set_outlimits(throttle.idlespeed(), tach_govern_rpm);
    gas_govern_deg = gas_pc_to_deg(gas_governor_pc);
    speedo_govern_mph = map(gas_governor_pc, 0.0, 100.0, 0.0, speedo.redline_mph());                                                                                     // Governor must scale the top vehicle speed proportionally
}
float steer_safe(float endpoint) {
    return steer_stop_pc + (endpoint - steer_stop_pc) * (1.0 - steer_safe_pc * speedo.filt() / (100.0 * speedo.redline_mph()));
}

// int* x is c++ style, int *x is c style
template <typename T>
T adj_val(T variable, T modify, T low_limit, T high_limit) {
    T oldval = variable;
    variable += modify;
    return variable < low_limit ? low_limit : (variable > high_limit ? high_limit : variable);
}
bool adj_val(int32_t *variable, int32_t modify, int32_t low_limit, int32_t high_limit) { // sets an int reference to new val constrained to given range
    int32_t oldval = *variable;
    *variable = adj_val(*variable, modify, low_limit, high_limit);
    return (*variable != oldval);
}
bool adj_val(float *variable, float modify, float low_limit, float high_limit) { // sets an int reference to new val constrained to given range
    float oldval = *variable;
    *variable = adj_val(*variable, modify, low_limit, high_limit);
    return (*variable != oldval);
}
bool adj_bool(bool val, int32_t delta) { return delta != 0 ? delta > 0 : val; } // returns 1 on delta=1, 0 on delta=-1, or val on delta=0
void adj_bool(bool *val, int32_t delta) { *val = adj_bool(*val, delta); }       // sets a bool reference to 1 on 1 delta or 0 on -1 delta

// tasks for RTOS, this section is currently going to be where we keep sensor update loops.
// TODO move these to more sensible places

// This is the function that will be run in a separate task
void update_temperature_sensors(void *parameter) {
    while (true) {
        if (!dont_take_temperatures)
            tempsens.update_temperatures();
        if (sim.potmapping(sens::engtemp)) {
            TemperatureSensor *engine_sensor = tempsens.get_sensor(loc::engine);
            if (engine_sensor != nullptr) {
                engine_sensor->set_temperature(pot.mapToRange(temp_sensor_min_f, temp_sensor_max_f));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for a second to avoid updating the sensors too frequently
    }
}

void set_board_defaults(bool devboard) {  // true for dev boards, false for printed board (on the car)
    if (devboard) {
        sim.set_can_sim(sens::pressure, true);
        sim.set_can_sim(sens::brkpos, true);
        sim.set_can_sim(sens::tach, true);
        sim.set_can_sim(sens::speedo, true);
        sim.set_can_sim(sens::mapsens, true);
        sim.set_can_sim(sens::airvelo, true);
        sim.set_can_sim(sens::basicsw, true);
        sim.set_potmap(sens::pressure);
    }
    else {  // override settings if running on the real car
        usb_jtag = false;
        gamma_correct_enabled = false;
        console_enabled = false;     // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
        keep_system_powered = false; // Use true during development
        screensaver = false;         // Can enable experiment with animated screen draws
        looptime_print = false;      // Makes code write out timestamps throughout loop to serial port
        dont_take_temperatures = false;
        touch_reticles = false;
    }
}
void starter_update () {  // Starter bidirectional handler logic.  Outside code interacts with handler by setting starter_request = req_off, req_on, or req_tog
    if (starter_signal_support) {
        if (starter_request == req_tog) starter_request = (req)(!starter_drive);  // translate toggle request to a drive request opposite to the current drive state
        if (starter_drive && ((starter_request == req_off) || starterTimer.expired())) {  // If we're driving the motor but need to stop
            starter_drive = false;
            set_pin (starter_pin, INPUT_PULLDOWN);  // we never assert low on the pin, just set pin as input and let the pulldown bring it low
        }
        if (!starter_drive && (starter_request != req_on) && !sim.simulating(sens::starter)) {  // If we haven't been and shouldn't be driving, and not simulating
            do {
                starter = digitalRead(starter_pin);  // then read the pin, starter variable will store if starter is turned on externally
            } while (starter != digitalRead(starter_pin)); // starter pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
        }
        else if (!starter && (starter_request == req_on) && remote_start_support) {  // If we got a request to start the motor, and it's not already being driven externally
            starter_drive = true;
            starter = HIGH;
            set_pin (starter_pin, OUTPUT);  // then set pin to an output
            write_pin (starter_pin, starter);  // and start the motor
            starterTimer.reset();  // if left on the starter will turn off automatically after X seconds
        }
        starter_request = req_na;  // we have serviced whatever requests
    }
    else starter = LOW;
}
void ignition_panic_update() {  // Run once each main loop, directly before panicstop_update()
    if (panicstop_request == req_tog) panicstop_request = (req)(!panicstop);
    if (ignition_request == req_tog) ignition_request = (req)(!ignition);
    // else if (ignition_request == ignition) ignition_request = req_na;  // With this line, it ignores requests to go to state it's already in, i.e. won't do unnecessary pin write
    if (speedo.car_stopped() || panicTimer.expired()) panicstop_request = req_off;  // Cancel panic stop if car is stopped
    if (!speedo.car_stopped()) {
        if (ignition && ignition_request == req_off) panicstop_request = req_on;  // ignition cut causes panic stop
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) panicstop_request = req_on;
    }
    bool paniclast = panicstop;
    if (panicstop_request != req_na) {
        panicstop = (bool)panicstop_request;
        if (panicstop && !paniclast) panicTimer.reset();
    }
    panicstop_request = req_na;
    if (panicstop) ignition_request = req_off;  // panic stop causes ignition cut
    if (ignition_request != req_na) {
        ignition = (bool)ignition_request;
        write_pin (ignition_pin, ignition);  // Turn car off or on (ign output is active high), ensuring to never turn on the ignition while panicking
    }
    ignition_request = req_na;  // Make sure this goes after the last comparison
}
void basicsw_update() {
    if (!sim.simulating(sens::basicsw)) {  // Basic Mode switch
        do {
            basicmodesw = !digitalRead(basicmodesw_pin);   // !value because electrical signal is active low
        } while (basicmodesw != !digitalRead(basicmodesw_pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
    }
}
void set_syspower(bool setting) {
    // if (!setting && runmode.mode() != ASLEEP) return;
    syspower = setting;
    if (keep_system_powered) syspower = HIGH;
    printf("syspower -> %d\n", syspower);
    write_pin(syspower_pin, syspower);
}
// pushbutton related
bool boot_button_last = 0;
bool boot_button = 0;
bool boot_button_timer_active = false;
bool boot_button_suppress_click = false;
bool boot_button_action = NONE;
Timer boot_button_timer(400000);
//
void bootbutton_update() {
    // ESP32 "boot" button. generates boot_button_action flags of LONG or SHORT presses which can be handled wherever. Handler must reset boot_button_action = NONE
    if (bootbutton_pin < 0) return;
    // if (boot_button_action == SHORT) {
    //     syspower_request = req_on;
    //     boot_button_action == NONE;
    // }
    if (!read_pin (bootbutton_pin)) {
        if (!boot_button) {  // If press just occurred
            boot_button_timer.reset();  // Looks like someone just pushed the esp32 "boot" button
            boot_button_timer_active = true;  // flag to indicate timing for a possible long press
        }
        else if (boot_button_timer_active && boot_button_timer.expired()) {
            boot_button_action = LONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
            boot_button_timer_active = false;  // Clear timer active flag
            boot_button_suppress_click = true;  // Prevents the switch release after a long press from causing a short press
        }
        boot_button = true;  // Store press is in effect
    }
    else {  // if button is not being pressed
        if (boot_button && !boot_button_suppress_click) boot_button_action = SHORT;  // if the button was just released, a short press occurred, which must be handled
        // else boot_button_action = NONE;  // This would auto-reset the button action flag but require it get handled in this loop. Otherwise the handler must set this
        boot_button_timer_active = false;  // Clear timer active flag
        boot_button = false;  // Store press is not in effect
        boot_button_suppress_click = false;  // End click suppression
    }
}
// int8_t syspower_request = req_na;
// //
// void syspower_update() {  // Soren: A lot of duplicate code with ignition/panicstop and syspower routines here ...
//     if (syspower_request == req_tog) syspower_request = (int8_t)(!syspower);
//     // else if (syspower_request == syspower) syspower_request = req_na;  // With this line, it ignores requests to go to state it's already in, i.e. won't do unnecessary pin write
//     if (syspower_request == req_off && !speedo.car_stopped()) syspower_request = req_na;
//     if (!syspower && keep_system_powered) syspower_request = req_on;
//     if (syspower_request != req_na) {
//         syspower = syspower_request;
//         write_pin(syspower_pin, syspower);
//     }
//     syspower_request = req_na;
// }
// Loop timing related
Timer loopTimer(1000000); // how long the previous main loop took to run (in us)
float loop_sum_s, loop_avg_us, loopfreq_hz;
uint32_t looptimes_us[20];
bool loop_dirty[20];
int32_t loopno = 1, loopindex = 0, loop_recentsum = 0, loop_scale_min_us = 0, loop_scale_avg_max_us = 2500, loop_scale_peak_max_us = 25000;
int64_t loop_cout_mark_us;
uint32_t loop_cout_us = 0, loop_peak_us = 0, loop_now = 0;;
const uint32_t loop_history = 100;
uint32_t loop_periods_us[loop_history];
std::vector<std::string> loop_names(20);
//
void looptime_init() {  // Run once at end of setup()
    if (looptime_print) {
        for (int32_t x=1; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
        loop_names[0] = std::string("top");
        loop_dirty[0] = false;
        loopindex = 1;
        looptimes_us[0] = esp_timer_get_time();
    }
    loopTimer.reset();  // start timer to measure the first loop
}
void looptime_mark(std::string loopname = std::string("")) {  // Add marks wherever you want in the main loop, set looptime_print true, will report times between all adjacent marks
    if (looptime_print) {
        if (loop_dirty[loopindex]) {
            loop_names[loopindex] = loopname;  // names[index], name);
            loop_dirty[loopindex] = false;
        }
        looptimes_us[loopindex] = esp_timer_get_time();
        loopindex++;
    }
}
float loop_calc_avg(uint32_t _loop_now, uint32_t _thisloop) {
    if (_loop_now == loop_history + 2) {
        loop_recentsum = _thisloop;
        for (int l = 0; l <= loop_history; l++)
            loop_recentsum += loop_periods_us[(_loop_now + l) % loop_history];
    }
    else loop_recentsum += _thisloop - loop_periods_us[loop_now];
    return (float)loop_recentsum/(float)loop_history;
}
void looptime_update() {  // Call once each loop at the very end
    uint32_t thisloop = (uint32_t)loopTimer.elapsed();
    loop_avg_us = loop_calc_avg(loop_now, thisloop);
    loop_periods_us[loop_now] = thisloop;  // us since beginning of this loop
    loopTimer.reset();
    loop_sum_s += (float)loop_periods_us[loop_now] / 1000000;
    // ema_filt(loop_periods_us[loop_now], &loop_avg_us, 0.01);
    if (loop_avg_us > 1) loopfreq_hz = 1000000/loop_avg_us;
    loop_peak_us = 0;
    for (int8_t i=0; i<loop_history; i++) if (loop_peak_us < loop_periods_us[i]) loop_peak_us = loop_periods_us[i]; 
    if (looptime_print) {
        loop_cout_mark_us = esp_timer_get_time();
        std::cout << std::fixed << std::setprecision(0);
        std::cout << "\r" << (uint32_t)loop_sum_s << "s #" << loopno;  //  << " av:" << std::setw(5) << (int32_t)(loop_avg_us);  //  << " av:" << std::setw(3) << loop_avg_ms 
        std::cout << " : " << std::setw(5) << loop_periods_us[loop_now] << " (" << loop_periods_us[loop_now]-loop_cout_us << ")us ";  // << " avg:" << loop_avg_us;  //  " us:" << esp_timer_get_time() << 
        for (int32_t x=1; x<loopindex; x++)
            std::cout << std::setw(3) << loop_names[x] << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1] << " ";
        std::cout << " cout:" << std::setw(5) << loop_cout_us;
        if (loop_periods_us[loop_now]-loop_cout_us > looptime_linefeed_threshold || !looptime_linefeed_threshold) std::cout << std::endl;
        loop_cout_us = (uint32_t)(esp_timer_get_time() - loop_cout_mark_us);
        loopindex = 0;
        looptime_mark ("top");
    }
    ++loop_now %= loop_history;
    loopno++;  // I like to count how many loops
}
// Diag / trouble codes
uint32_t err_timeout_us = 175000;
Timer errTimer((int64_t)err_timeout_us);
uint32_t err_margin_adc = 5;
// Sensor related trouble - this all should be moved to devices.h
enum err_type { LOST, RANGE, CALIB, WARN, CRIT, INFO, num_err_types };
enum err_sens { e_hrcvert, e_hrcch3, e_pressure, e_brkpos, e_speedo, e_hrchorz, e_tach, e_temps, e_starter, e_hrcch4, e_basicsw, e_mulebatt, e_lipobatt, e_airvelo, e_mapsens, e_num_sensors, e_none };  // these are in order of priority
char err_type_card[num_err_types][5] = { "Lost", "Rang", "Cal", "Warn", "Crit", "Info" };
char err_sensor_card[e_num_sensors+1][7] = { "HrcV", "HrcCh3", "BrPres", "BrkPos", "Speedo", "HrcH", "Tach", "Temps", "Startr", "HrcCh4", "Basic", "MulBat", "LiPO", "Airflw", "MAP", "None" };
// enum class sensor : opt_t { none=0, joy, pressure, brkpos, speedo, tach, airvelo, mapsens, engtemp, mulebatt, ignition, basicsw, cruisesw, starter, syspower };  // , num_sensors, err_flag };
bool err_sensor_alarm[num_err_types] = { false, false, false, false, false, false };
int8_t err_sensor_fails[num_err_types] = { 0, 0, 0, 0, 0, 0 };
bool err_sensor[num_err_types][e_num_sensors]; //  [LOST/RANGE] [e_hrchorz/e_hrcvert/e_hrcch3/e_hrcch4/e_pressure/e_brkpos/e_tach/e_speedo/e_airvelo/e_mapsens/e_temps/e_mulebatt/e_lipobatt/e_basicsw/e_starter]   // sens::opt_t::num_sensors]
uint8_t highest_pri_failing_sensor[num_err_types];
uint8_t highest_pri_failing_last[num_err_types];
bool diag_ign_error_enabled = true;

void diag_update() {
    if (errTimer.expireset()) {

        // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
        // this is one approach
        // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
        // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
        // retreive with an OBD tool. Some checks are below, along with other possible things to check for:
        if (!ignition && !tach.engine_stopped()) {  // Check: if engine is turning when ignition signal is off
            if (diag_ign_error_enabled) { // See if the engine is turning despite the ignition being off
                Serial.println (F("Detected engine rotation in the absense of ignition signal"));  // , tach_filt_rpm, ignition
                diag_ign_error_enabled = false;  // Prevents endless error reporting the same error
            }
        }
        else diag_ign_error_enabled = true;

        // different approach
        bool not_detected;
        not_detected = false;  // first reset
        for (int cat = 0; cat < num_temp_categories; cat++) temp_err[cat] = false;  // first reset
        for (int loc = 0; loc < tempsens.locint(loc::num_locations); loc++) {
            if (!tempsens.detected(loc)) not_detected = true;
            else if (tempsens.val(loc) >= temp_lims_f[tempsens.errclass(loc)][WARNING]) temp_err[tempsens.errclass(loc)] = true;
        }
        err_sensor[LOST][e_temps] = not_detected;

        // Detect sensors disconnected or giving out-of-range readings.
        // TODO : The logic of this for each sensor should be moved to devices.h objects
        err_sensor[RANGE][e_brkpos] = (brakepos.in() < brakepos.op_min_in() || brakepos.in() > brakepos.op_max_in());
        err_sensor[LOST][e_brkpos] = (brakepos.raw() < err_margin_adc);
        err_sensor[RANGE][e_pressure] = (pressure.psi() < pressure.op_min_psi() || pressure.psi() > pressure.op_max_psi());
        err_sensor[LOST][e_pressure] = (pressure.raw() < err_margin_adc);
        err_sensor[RANGE][e_mulebatt] = (mulebatt.v() < mulebatt.op_min_v() || mulebatt.v() > mulebatt.op_max_v());
        for (int32_t ch = HORZ; ch <= CH4; ch++) {  // Hack: This loop depends on the indices for hotrc channel enums matching indices of hotrc sensor errors
            err_sensor[RANGE][ch] = !hotrc.radiolost() && ((hotrc.us[ch][RAW] < hotrc.us[ch][MIN] - (hotrc.us[ch][MARGIN] >> 1)) 
                                    || (hotrc.us[ch][RAW] > hotrc.us[ch][MAX] + (hotrc.us[ch][MARGIN] >> 1)));  // && ch != VERT
            err_sensor[LOST][ch] = !hotrc.radiolost() && ((hotrc.us[ch][RAW] < (hotrc.absmin_us - hotrc.us[ch][MARGIN]))
                                    || (hotrc.us[ch][RAW] > (hotrc.absmax_us + hotrc.us[ch][MARGIN])));
        }
        // err_sensor[RANGE][e_hrcvert] = (hotrc.us[VERT][RAW] < hotrc.failsafe_us - hotrc.us[ch][MARGIN])
        //     || ((hotrc.us[VERT][RAW] < hotrc.us[VERT][MIN] - halfmargin) && (hotrc.us[VERT][RAW] > hotrc.failsafe_us + hotrc.us[ch][MARGIN]));
        
        // Set sensor error idiot light flags
        // printf ("Sensors errors: ");
        
        // printf ("Sensor check: ");
        for (int32_t t=LOST; t<=RANGE; t++) {
            highest_pri_failing_sensor[t] = e_none;
            err_sensor_alarm[t] = false;
            err_sensor_fails[t] = 0;
            for (int32_t s=0; s<e_num_sensors; s++)
                if (err_sensor[t][s]) {
                    if (highest_pri_failing_sensor[t] = e_none) highest_pri_failing_sensor[t] = s;
                    err_sensor_alarm[t] = true;
                    err_sensor_fails[t]++;
                }
        }
        // printf ("\n");

        // Detectable transducer-related failures :: How we can detect them
        // Brakes:
        // * Pressure sensor, chain linkage, or vehicle brakes problem :: Motor retracted with position below zeropoint, but pressure did not increase.
        // * Pressure sensor zero point miscalibration (no force on pedal) :: Minimum pressure reading since startup has never reached 0 PSI or less (cal is too high), or, is more than a given margin below 0. * Note this can also be an auto-calibration approach
        // * Pressure sensor max point miscalibration (full force on pedal) :: When target set to max pressure, after motor moves to the point position isn't changing, the pressure reading deviates from max setting by more than a given margin. * Note this can also be an auto-calibration approach
        // * Position sensor problem :: When pressure is not near max, motor is driven more than X volt-seconds without position change (of the expected polarity).
        // * Brake motor problem :: When motor is driven more than X volt-seconds without any change (of the expected polarity) to either position or pressure.
        // * Brake calibration, idle high, or speedo sensor problem :: Motor retracted to near limit, with position decreased and pressure increased as expected, but speed doesn't settle toward 0.
        // * Pressure sensor problem :: If pressure reading is out of range, or ever changes in the unexpected direction during motor movement.
        // * Position sensor or limit switch problem :: If position reading is outside the range of the motor limit switches.
        // Steering:
        // * Chain derailment or motor or limit switch problem :: Motor told to drive for beyond X volt-seconds in one direction for > Y seconds.
        // Throttle/Engine:
        // * AirVelo/MAP/tach sensor failure :: If any of these three sensor readings are out of range to the other two.
        // Tach/Speedo:
        // * Sensor read problem :: Derivative of consecutive readings (rate of change) spikes higher than it's possible for the physical rotation to change - (indicates missing pulses)
        // * Disconnected/problematic speed sensor :: ignition is on, tach is nonzero, and runmode = hold/fly/cruise, yet speed is zero. Or, throttle is at idle and brake pressure high for enough time, yet speed readings are nonzero
        // * Disconnected/problematic tach sensor :: runmode is hold/fly/cruise, ignition is on and speed increases, but tach is below idle speed 
        // Temperature:
        // * Engine temperature sensor problem :: Over X min elapsed with Ignition on and tach >= low_idle, but engine temp is below nominal warmup temp.
        // * Cooling system, coolant, fan, thermostat, or coolant sensor problem :: Engine temp stays over ~204 for >= X min without coolant temp dropping due to fan.
        // * Axle, brake, etc. wheel issue or wheel sensor problem :: The hottest wheel temp is >= X degF hotter than the 2nd hottest wheel.
        // * Axle, brake, etc. wheel issue or wheel/ambient sensor problem :: A wheel temp >= X degF higher than ambient temp.
        // * Ignition problem, fire alarm, or temp sensor problem :: Ignition is off but a non-ambient temp reading increases to above ambient temp.
        // AirVelo:
        // * Air filter clogged, or carburetor problem :: Track ratio of massairflow/throttle angle whenever throttle is constant. Then, if that ratio lowers over time by X below that level, indicates restricted air. 
        // Battery:
        // * Battery low :: Mulebatt readings average is below a given threshold
        // * Inadequate charging :: Mulebatt readings average has decreased over long time period
        // 
        // More ideas to define better and implement:
        // * Check if the pressure response is characteristic of air being in the brake line.
        // * Axle/brake drum may be going bad (increased engine RPM needed to achieve certain speedo)  (beware going up hill may look the same).
        // * E-brake has been left on (much the same symptoms as above? (beware going up hill may look the same) 
        // * Carburetor not behaving (or air filter is clogged). (See above about engine deiseling - we can detect this!)
        // * After increasing braking, the actuator position changes in the opposite direction, or vise versa.
        // * Changing an actuator is not having the expected effect.
        // * A tunable value suspected to be out of tune.
        // * Check regularly for ridiculous shit. Compare our variable values against a predetermined list of conditions which shouldn't be possible or are at least very suspect. For example:
        //   A) Sensor reading is out of range, or has changed faster than it ever should.
        //   B) Stopping the car on entering hold/shutdown mode is taking longer than it ever should.
        //   C) Mule seems to be accelerating like a Tesla.
        //   D) Car is accelerating yet engine is at idle.
        // * The control system has nonsensical values in its variables.
    }
}
void err_print_info() {
    for (int32_t t=LOST; t<=INFO; t++) {
        printf ("diag err: %s (%d): ", err_type_card[t], err_sensor_fails[t]);
        for (int32_t s=0; s<=e_num_sensors; s++) {
            if (s == e_num_sensors) s++;
            if (err_sensor[t][s]) printf ("%s, ", err_sensor_card[s]);
        }
        printf("\n");
    }
}
int16_t touch_pt[4] = { 160, 120, 2230, 2130 };

// Neopixel stuff
neopixelstrip neo;
int32_t neobright = 10;  // lets us dim/brighten the neopixels
int32_t neodesat = 0;  // lets us de/saturate the neopixels

void neo_setup() {
    neo.init((uint8_t)neopixel_pin, running_on_devboard, 1);
    // neo.init((uint8_t)neopixel_pin, !running_on_devboard);
    neo.setbright(neobright);
    neo.setdesaturation(neodesat);
    neo.heartbeat(neopixel_pin >= 0);
}
void enable_flashdemo(bool ena) {
    if (ena) {
        neo.setflash(4, 8, 8, 8, 20, -1);  // brightness toggle in a continuous squarewave
        neo.setflash(5, 3, 1, 2, 85);      // three super-quick bright white flashes
        neo.setflash(6, 2, 5, 5, 0, 0);    // two short black pulses
    }
    else {
        neo.setflash(4, 0);
        neo.setflash(5, 0);
        neo.setflash(6, 0);
    }
}
// Calculates massairflow in g/s using values passed in if present, otherwise it reads fresh values
float massairflow(float _map = NAN, float _airvelo = NAN, float _ambient = NAN) {  // mdot (kg/s) = density (kg/m3) * v (m/s) * A (m2) .  And density = P/RT.  So,   mdot = v * A * P / (R * T)  in kg/s
    float temp = _ambient;
    if (std::isnan(_ambient)) {
        temp = tempsens.val(loc::ambient);
        if (std::isnan(temp) && running_on_devboard) temp = tempsens.val(loc::engine);
        if (std::isnan(temp)) return -1;  // Avoid crashing due to trying to read absent sensor
    }
    float T = 0.556 * (temp - 32.0) + 273.15;  // in K.  This converts from degF to K
    float R = 287.1;  // R (for air) in J/(kg·K) ( equivalent to 8.314 J/(mol·K) )  1 J = 1 kg*m2/s2
    float v = 0.447 * (std::isnan(_airvelo) ? airvelo.filt() : _airvelo); // in m/s   1609.34 m/mi * 1/3600 hr/s = 0.447
    float Ain2 = 3.1415926;  // in in2    1.0^2 in2 * pi  // will still need to divide by 1550 in2/m2
    float P = 6894.76 * (std::isnan(_map) ? mapsens.filt() : _map);  // in Pa   6894.76 Pa/PSI  1 Pa = 1 J/m3
    return v * Ain2 * P * 1000.0 / (R * T * 1550);  // mass air flow in grams per second (ug/s)   (1k g/kg * m/s * in2 * J/m3) / (J/(kg*K) * K * 1550 in2/m2) = g/s
}
float maf_gps;  // Manifold mass airflow in grams per second
float maf_min_gps = 0.0;
float maf_max_gps = massairflow(mapsens.max_psi(), airvelo.max_mph(), temp_lims_f[AMBIENT][DISP_MIN]);
bool flashdemo = false;