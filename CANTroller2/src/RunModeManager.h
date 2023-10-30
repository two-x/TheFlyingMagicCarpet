#pragma once
#include "display.h"
#include "uictrl.h"  // for encoder button to wake up
#include "esp_sleep.h"  // for deep sleep

class RunModeManager {
private:
    enum joydirs { joy_rt = -2, joy_down = -1, joy_cent = 0, joy_up = 1, joy_lt = 2 };
    joydirs joydir;
    float cruise_ctrl_extent_pc;       // During cruise adjustments, saves farthest trigger position read
    bool cruise_trigger_released = false;
    static const uint32_t gesture_flytimeout_us = 2500000;        // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
    Timer gestureFlyTimer, cruiseDeltaTimer, pwrup_timer;
    runmodes _currentMode = SHUTDOWN; // note these are more here in caseA we eventually don't use the globals
    runmodes _oldMode;
    uint32_t pwrup_timeout = 500000;
    Encoder* encoder;
    Display* display;
public:
    // Call this function in the main loop to manage run modes
    // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    RunModeManager(Display* _display, Encoder* _encoder) { encoder = _encoder; display = _display; }
    runmodes go_to(runmodes newmode) {
        _currentMode = newmode;
        return newmode;
    }
    runmodes mode() { return _currentMode; }

    runmodes run_runmode() {
        updateMode(); // Update the current mode if needed, this also sets we_just_switched_modes
        if (_currentMode == BASIC) run_basicMode(); // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        else if (_currentMode == ASLEEP) run_asleepMode();
        else if (_currentMode == SHUTDOWN) run_shutdownMode();
        else if (_currentMode == STALL) run_stallMode();
        else if (_currentMode == HOLD) run_holdMode();
        else if (_currentMode == FLY) run_flyMode();
        else if (_currentMode == CRUISE) run_cruiseMode();
        else if (_currentMode == CAL) run_calMode();
        else {  // Obviously this should never happen
            Serial.println (F("Error: Invalid runmode entered"));
            go_to(SHUTDOWN);
        }
        return _currentMode;
    }
private:
    joydirs get_joydir(uint8_t axis = VERT) {
        if (axis == VERT) return (hotrc_pc[axis][FILT] > hotrc_pc[axis][DBTOP]) ? joy_up : ((hotrc_pc[axis][FILT] < hotrc_pc[axis][DBBOT]) ? joy_down : joy_cent);
        return (hotrc_pc[axis][FILT] > hotrc_pc[axis][DBTOP]) ? joy_rt : ((hotrc_pc[axis][FILT] < hotrc_pc[axis][DBBOT]) ? joy_lt : joy_cent);
        // return (hotrc_pc[axis][FILT] > hotrc_pc[axis][DBTOP]) ? ((axis == VERT) ? joy_up : joy_rt) : (hotrc_pc[axis][FILT] < hotrc_pc[axis][DBBOT]) ? ((axis == VERT) ? joy_down : joy_lt) : joy_cent;
    }
    // void sleep_for(uint32_t wakeup_us) {
    //     esp_sleep_enable_timer_wakeup(wakeup_us);
    //     esp_deep_sleep_start();
    // }
    void updateMode() {
        // if (!syspower) _currentMode = ASLEEP;
        if (basicmodesw) _currentMode = BASIC;  // if basicmode switch on --> Basic Mode
        else if ((_currentMode != CAL) && (_currentMode != ASLEEP) && (panicstop || !ignition)) _currentMode = SHUTDOWN;
        else if ((_currentMode != CAL) && (_currentMode != ASLEEP) && tach.engine_stopped()) _currentMode = STALL;;  // otherwise if engine not running --> Stall Mode
        we_just_switched_modes = (_currentMode != _oldMode);  // currentMode should not be changed after this point in loop
        if (we_just_switched_modes) {
            disp_runmode_dirty = true;
            cleanup_state_variables();
        }
        _oldMode = _currentMode;
    }
    void cleanup_state_variables()  {
        if (_oldMode == ASLEEP) powering_up = false;
        else if (_oldMode == SHUTDOWN) {
            shutdown_color = colorcard[SHUTDOWN];
            shutdown_incomplete = false;
        }
        else if (_oldMode == STALL);
        else if (_oldMode == HOLD) {
            joy_centered = false;
            starter_request = req_off;  // Stop any in-progress startings
        }
        else if (_oldMode == FLY) car_hasnt_moved = false;
        else if (_oldMode == CRUISE) cruise_adjusting = false;
        else if (_oldMode == CAL) {
            cal_pot_gasservo_ready = false;
            cal_pot_gasservo_mode = false;
            cal_joyvert_brkmotor_mode = false;
        }
    }
    void run_basicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        else if (!basicmodesw && !tach.engine_stopped()) go_to( speedo.car_stopped() ? HOLD : FLY );  // If we turned off the basic mode switch with engine running, change modes. If engine is not running, we'll end up in Stall Mode automatically
    }
    void run_asleepMode() {
        if (we_just_switched_modes) {
            set_syspower(LOW); // Power down devices to save battery
            sleep_request = req_na;
            powering_up = false;
        }
        if ((*encoder).pressed() || sleep_request == req_off) {
            set_syspower(HIGH);
            sleep_request = req_na;
            pwrup_timer.set(pwrup_timeout);
            powering_up = true;
        }
        if (powering_up && pwrup_timer.expired()) {
            display->all_dirty();
            go_to(SHUTDOWN);
        }
    }
    void run_shutdownMode() { // In shutdown mode we stop the car if it's moving then park the motors.
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            throttle.goto_idle();  //  Release the throttle 
            shutdown_incomplete = true;
            shutdown_color = LPNK;
            disp_runmode_dirty = true;
            calmode_request = false;
            sleep_request = req_na;
            if (!speedo.car_stopped() && !autostop_disabled) {
                if (panicstop && pressure_target_psi < pressure_panic_initial_psi) pressure_target_psi = pressure_panic_initial_psi;
                else if (!panicstop && pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = pressure_hold_initial_psi;
                brakeIntervalTimer.reset();
                stopcarTimer.reset();
                park_the_motors = false;
            }
        }
        else if ((speedo.car_stopped() || allow_rolling_start || autostop_disabled) && ignition && !panicstop && !tach.engine_stopped()) go_to(HOLD);  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
        if (shutdown_incomplete) {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (speedo.car_stopped() || stopcarTimer.expired() || autostop_disabled) {  // If car has stopped, or timeout expires, then release the brake
                if (shutdown_color == LPNK) {  // On first time through here
                    park_the_motors = true;  // Flags the motor parking to happen, only once
                    gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
                    motorParkTimer.reset();  // Set a timer to timebox this effort
                    shutdown_color = DPNK;
                    disp_runmode_dirty = true;
                }
                else if (!park_the_motors) {  // When done parking the motors we can finish shutting down
                    shutdown_incomplete = false;
                    shutdown_color = colorcard[SHUTDOWN];
                    disp_runmode_dirty = true;
                    sleepInactivityTimer.reset();
                }
            }
            else if (brakeIntervalTimer.expireset()) {
                if (!autostop_disabled) pressure_target_psi = pressure_target_psi + (panicstop ? pressure_panic_increment_psi : pressure_hold_increment_psi);  // Slowly add more brakes until car stops
                throttle.goto_idle();  // Keep target updated to possibly changing idle value
            }
        }
        else if (calmode_request) go_to(CAL);  // if fully shut down and cal mode requested, go to cal mode
        else if (sleepInactivityTimer.expired() || sleep_request == req_on) go_to(ASLEEP);
        sleep_request == req_na;
    }
    void run_stallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (get_joydir(VERT) != joy_down) pressure_target_psi = pressure.min_human();  // If in deadband or being pushed up, no pressure target
        else pressure_target_psi = map (hotrc_pc[VERT][FILT], hotrc_pc[VERT][DBBOT], hotrc_pc[VERT][MIN], pressure.min_human(), pressure.max_human());  // Scale joystick value to pressure adc setpoint
        if (!tach.engine_stopped()) go_to(HOLD);  // If we started the car, enter hold mode once starter is released
        if (starter || !tach.engine_stopped()) go_to(HOLD);  // If we started the car, enter hold mode once starter is released
    }
    void run_holdMode() {
        if (we_just_switched_modes) {  // Release throttle and push brake upon entering hold mode
            if (!autostop_disabled) {
                if (speedo.car_stopped()) pressure_target_psi = pressure.filt() + (starter ? pressure_panic_increment_psi : pressure_hold_increment_psi); // If the car is already stopped then just add a touch more pressure and then hold it.
                else if (pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = starter ? pressure_panic_initial_psi : pressure_hold_initial_psi;  //  These hippies need us to stop the car for them
            }
            brakeIntervalTimer.reset();
            stopcarTimer.reset();
            joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
        }
        throttle.goto_idle();  // Let off gas (if gas using PID mode) and keep target updated to possibly changing idle value
        if (brakeIntervalTimer.expireset() && !speedo.car_stopped() && !stopcarTimer.expired() && !autostop_disabled)
            pressure_target_psi = min (pressure_target_psi + (starter ? pressure_panic_increment_psi : pressure_hold_increment_psi), pressure.max_human());  // If the car is still moving, push harder
        if (get_joydir(VERT) != joy_up) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && !starter && !hotrc_radio_lost) go_to(FLY); // Enter Fly Mode upon joystick movement from center to above center  // Possibly add "&& car_stopped()" to above check?
    }
    void run_flyMode() {
        if (we_just_switched_modes) car_hasnt_moved = speedo.car_stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
        joydir = get_joydir(VERT);
        if (car_hasnt_moved) {
            if (joydir != joy_up) go_to(HOLD);  // Must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.car_stopped()) car_hasnt_moved = false;  // Once car moves, we're allowed to release the trigger without falling out of fly mode
        }
        else if (speedo.car_stopped()) go_to(HOLD);  // Go to Hold Mode if we have come to a stop after moving  // && hotrc_pc[VERT][FILT] <= hotrc_pc[VERT][DBBOT]

        if (!sim.simulating(sensor::joy) && hotrc_radio_lost) go_to(HOLD);  // Radio must be good to fly. This should already be handled elsewhere but another check can't hurt
        else {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            if (joydir == joy_up)  // If we are trying to accelerate, scale joystick value to determine gas setpoint
                throttle.set_target (map (hotrc_pc[VERT][FILT], hotrc_pc[VERT][DBTOP], hotrc_pc[VERT][MAX], throttle.idlespeed(), tach_govern_rpm));
            else throttle.goto_idle();  // Else let off gas (if gas using PID mode)
            
            if (joydir == joy_down)  // If we are trying to brake, scale joystick value to determine brake pressure setpoint
                pressure_target_psi = map (hotrc_pc[VERT][FILT], hotrc_pc[VERT][DBBOT], hotrc_pc[VERT][MIN], pressure.min_human(), pressure.max_human());
            else pressure_target_psi = pressure.min_human();  // Else let off the brake   
        }
        // Cruise mode can be entered/exited by pressing a controller button, or exited by holding the brake on full for a half second
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (flycruise_toggle_request) go_to(CRUISE);
        flycruise_toggle_request = false;
    }
    void run_cruiseMode() {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            speedo_target_mph = speedo.filt();
            pressure_target_psi = pressure.min_human();  // Let off the brake and keep it there till out of Cruise mode
            throttle.set_target(tach.filt());  // Start off with target set to current tach value
            gestureFlyTimer.set(gesture_flytimeout_us);  // reset gesture timer
            cruise_trigger_released = false;  // in case trigger is being pulled as cruise mode is entered, the ability to adjust is only unlocked after the trigger is subsequently released to the center
            gas_cruise_us = gas_out_us;  //  if cruise_fixed throttle is true, this variable stores the setpoint of throttle angle
            gas_adjustpoint_us = gas_cruise_us;  // Pull of trigger away from center in either direction starts a setpoint adjustment, scaled from *your current setpoint* (not from the center value) to the relevant min or max extreme 
            cruise_ctrl_extent_pc = hotrc_pc[VERT][CENT];  // After an adjustment, need this to prevent setpoint from following the trigger back to center as you release it
            cruise_adjusting = false;
        }
        joydir = get_joydir(VERT);
        if (joydir == joy_cent) {
            cruise_trigger_released = true;
            cruise_ctrl_extent_pc = hotrc_pc[VERT][CENT];
            cruise_adjusting = false;
        }
        else if (joydir == joy_down && !cruise_speed_lowerable) go_to(FLY);
        else if (cruise_trigger_released) {  // adjustments disabled until trigger has been to center at least once since going to cruise mode
            float ctrlratio = (std::abs(hotrc_pc[VERT][FILT]) - hotrc_pc[VERT][DBTOP]) / (hotrc_pc[VERT][MAX] - hotrc_pc[VERT][DBTOP]);
            if (cruise_setpoint_mode == throttle_delta) {
                if (cruise_adjusting) gas_cruise_us -= joydir * ctrlratio * cruise_delta_max_us_per_s * cruiseDeltaTimer.elapsed() / 1000000.0;
                cruiseDeltaTimer.reset(); 
            }
            else if (std::abs(hotrc_pc[VERT][FILT]) >= cruise_ctrl_extent_pc) {  // to avoid the adjustments following the trigger back to center when released
                if (cruise_setpoint_mode == throttle_angle) {
                    if (!cruise_adjusting) gas_adjustpoint_us = gas_cruise_us;  // When beginning adjustment, save current throttle pulse value to use as adjustment endpoint
                    gas_cruise_us = gas_adjustpoint_us + ctrlratio * cruise_angle_attenuator * (((joydir == joy_up) ? gas_govern_us : gas_ccw_closed_us) - gas_adjustpoint_us);
                }
                else if (cruise_setpoint_mode == pid_suspend_fly) {
                    if (!cruise_adjusting) tach_adjustpoint_rpm = tach.filt();
                    throttle.set_target(tach_adjustpoint_rpm + ctrlratio * (((joydir == joy_up) ? tach_govern_rpm : throttle.idlespeed()) - tach_adjustpoint_rpm));
                }
                cruise_ctrl_extent_pc = std::abs(hotrc_pc[VERT][FILT]);
            }
            gas_cruise_us = constrain(gas_cruise_us, gas_govern_us, gas_ccw_closed_us);
            cruise_adjusting = true;
        }
        if (flycruise_toggle_request) go_to(FLY);  // Go to fly mode if hotrc ch4 button pushed
        flycruise_toggle_request = false;
        // If joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (hotrc_pc[VERT][FILT] > hotrc_pc[VERT][MIN] + flycruise_vert_margin_pc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) go_to(FLY);  // New gesture to drop to fly mode is hold the brake all the way down for more than X ms
        if (speedo.car_stopped()) go_to(HOLD);  // In case we slam into camp Q woofer stack, get out of cruise mode
    }
    void run_calMode() {  // Calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration. Don't use it unless you know how.
        if (we_just_switched_modes) {  // Entering Cal mode: From fully shut down state, open simulator and long-press the Cal button. Each feature starts disabled but can be enabled with the tuner.
            calmode_request = false;
            cal_pot_gasservo_mode = false;
            cal_pot_gasservo_ready = false;
            cal_joyvert_brkmotor_mode = false;
        }
        else if (calmode_request) go_to(SHUTDOWN);
        float temp = pot.mapToRange(gas_ccw_max_us, gas_cw_min_us);
        cal_pot_gasservo_ready = (temp <= (float)gas_ccw_closed_us && temp >= (float)gas_cw_open_us);
    }
};
// Here are the different runmodes documented
//
// ** Basic Mode **
// - Required: BasicMode switch On
// At startup it attempts to park the gas servo and brake motor, to release all tension on gas or brake
// pedals so you can drive w/ your feet. Thereafter the gas and brake don't do anything. Only the steering
// works, so use the pedals. This mode is enabled by a switch on the controller box. The only way to 
// leave Basic Mode is by turning off the basic switch, or turning off the syspower signal.
//
// ** Asleep Mode **
// - Required: Request with hotrc ch4 button when shut down
// Turns off power to the system. This includes all sensors/actuators and the screen, but not the hotrc receiver.
// When requested to power up (same ch4 hotrc button), it re-powers everything and goes to shutdown mode.
//
// ** Shutdown Mode **
// - Required: BasicMode switch Off & Ignition Off
// This mode is active at boot, or whenever the ignition is off or when panic stopping. If the car is
// moving, then like hold mode, shutdown mode will try to stop the car. Once stopped, then like basic mode,
// it will park the motors out of the way and all systems stop. After a timeout it will go to asleep mode.
//
// ** Stall Mode **
// - Required: Engine stopped & BasicMode switch Off & Ignition On
// This mode is active when the engine is not running.  If car is moving, then it presumably may
// coast to a stop.  The actuators are all enabled and work, but the gas drops to open-loop control.
// The starter may be turned on or off freely here by hitting the hotrc ch4 button. If the engine 
// turns, then it'll go to hold mode. May be useful if beoing pushed or towed, or when servicing.
//
// ** Hold Mode **
// - Required: Engine running & JoyVert<=Center & BasicMode switch Off & Ignition On
// This mode ensures the car is stopped and stays stopped until you pull the trigger to give it gas, at which
// point it goes to fly mode. This mode is entered from fly mode if the car comes to a stop, or from Stall Mode if
// the engine starts turning. The starter can possibly be on through that transition, and thereafter it may be 
// turned off but not on from hold mode.
//
// ** Fly Mode **
// - Required: JoyVert>Center & Engine running & BasicMode Off & Ign On
// This mode is for driving under manual control. This mode is entered from hold mode by pulling the gas trigger.
// If the trigger is released again before the car moves, it's back to hold mode though. Trigger pull controls throttle
// and trigger push controls brake, either/or. Whenever the car stops, then back to hold mode. Cruise mode may be 
// entered from fly mode by pressing the cruise toggle button (ch4).
//
// ** Cruise Mode **
// - Required: Car Moving & Engine running & BasicMode switch Off & Ignition On
// This mode is entered from Fly Mode by pushing the cruise toggle button, and pushing it again will take you back.
// There are no brakes in cruise mode, and pushing away on the trigger instead allows decreasing the cruise setpoint
// (or pull it to increase the setpoint) Cruise can be run in three modes which adjust differently. The default
// "throttle_delta_mode" holds a fixed throttle servo position as long as the trigger is centered, and if not,
// it adjusts the setpoint up or down proportional to how far and how long you hold the trigger away from center.
// If you panic and push full brake for over 500ms, it will drop to fly mode and then push brakes.
//
// ** Cal Mode **
// This mode allows direct control of some actuators without respecting limits of motion, for purpose of
// calibrating those very limits. It can be entered from shutdown mode with simulator on by long-pressing the CAL
// button. Be careful with it.