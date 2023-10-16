#pragma once
#include "display.h"

class RunModeManager {
private:
    enum joydirs { joy_down = -1, joy_cent = 0, joy_up = 1 };
    joydirs joydir;
    float cruise_ctrl_extent_pc;       // During cruise adjustments, saves farthest trigger position read
    bool cruise_trigger_released = false;
    Timer cruiseDeltaTimer;
public:

    RunModeManager() : _currentMode(SHUTDOWN), _oldMode(SHUTDOWN) {}

    // Call this function in the main loop to manage run modes
    // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    runmodes handle_runmode() {
        updateMode(runmodes(runmode)); // Update the current mode if needed, this also sets we_just_switched_modes
        
        if (_currentMode == BASIC) {
            handleBasicMode(); // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        } else if (_currentMode == SHUTDOWN) {
            handleShutdownMode();
        } else if (_currentMode == STALL) {
            handleStallMode();
        } else if (_currentMode == HOLD) {
            handleHoldMode();
        } else if (_currentMode == FLY) {
            handleFlyMode();
        } else if (_currentMode == CRUISE) {
            handleCruiseMode();
        } else if (_currentMode == CAL) {
            handleCalMode();
        } else {
            // Obviously this should never happen
            Serial.println (F("Error: Invalid runmode entered"));
            updateMode(SHUTDOWN);
        }
        return modeChanger();
    }

private:
    runmodes _currentMode; // note these are more here in case we eventually don't use the globals
    runmodes _oldMode;

    joydirs get_joydir() {
        return (hotrc_pc[VERT][FILT] > hotrc_pc[VERT][DBTOP]) ? joy_up : ((hotrc_pc[VERT][FILT] < hotrc_pc[VERT][DBBOT]) ? joy_down : joy_cent);
    }
    
    void updateMode(runmodes newmode) { _currentMode = newmode; }

    runmodes modeChanger() {
        if (basicmodesw) _currentMode = BASIC;  // if basicmode switch on --> Basic Mode
        else if ((_currentMode != CAL) && (panic_stop || !ignition)) _currentMode = SHUTDOWN;
        else if ((_currentMode != CAL) && tachometer.engine_stopped()) _currentMode = STALL;;  // otherwise if engine not running --> Stall Mode
        we_just_switched_modes = (_currentMode != _oldMode);  // currentMode should not be changed after this point in loop
        if (we_just_switched_modes) {
            disp_runmode_dirty = true;
            cleanup_state_variables();
            syspower = HIGH;
        }
        _oldMode = _currentMode;
        return _currentMode;
    }
    void cleanup_state_variables()  {
        if (_oldMode == SHUTDOWN) {
            shutdown_color = colorcard[SHUTDOWN];
            shutdown_incomplete = false;
        }
        else if (_oldMode == STALL) starter_request = st_off;  // Stop any in-progress startings
        else if (_oldMode == HOLD) joy_centered = false;
        else if (_oldMode == FLY) car_hasnt_moved = false;
        else if (_oldMode == CRUISE) cruise_adjusting = false;
        else if (_oldMode == CAL) {
            cal_pot_gasservo_ready = false;
            cal_pot_gasservo_mode = false;
            cal_joyvert_brkmotor_mode = false;
        }
    }
    void handleBasicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        else if (!basicmodesw && !tachometer.engine_stopped()) updateMode( speedometer.car_stopped() ? HOLD : FLY );  // If we turned off the basic mode switch with engine running, change modes. If engine is not running, we'll end up in Stall Mode automatically
    }
    void handleShutdownMode() { // In shutdown mode we stop the car if it's moving then park the motors.
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            throttle.goto_idle();  //  Release the throttle 
            shutdown_incomplete = true;
            shutdown_color = LPNK;
            disp_runmode_dirty = true;
            calmode_request = false;
            if (!speedometer.car_stopped() && !autostop_disabled) {
                if (panic_stop && pressure_target_psi < pressure_panic_initial_psi) pressure_target_psi = pressure_panic_initial_psi;
                else if (!panic_stop && pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = pressure_hold_initial_psi;
                brakeIntervalTimer.reset();
                stopcarTimer.reset();
                park_the_motors = false;
            }
        }
        // else if (ignition && engine_stopped()) updateMode(STALL);  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
        else if ((speedometer.car_stopped() || allow_rolling_start || autostop_disabled) && ignition && !panic_stop && !tachometer.engine_stopped()) updateMode(HOLD);  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
        if (shutdown_incomplete) {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (speedometer.car_stopped() || stopcarTimer.expired() || autostop_disabled) {  // If car has stopped, or timeout expires, then release the brake
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
                if (!autostop_disabled) pressure_target_psi = pressure_target_psi + (panic_stop) ? pressure_panic_increment_psi : pressure_hold_increment_psi;  // Slowly add more brakes until car stops
                throttle.goto_idle();  // Keep target updated to possibly changing idle value
            }
        }
        else if (calmode_request) updateMode(CAL);  // if fully shut down and cal mode requested, go to cal mode
        else if (sleepInactivityTimer.expired()) {
            syspower = LOW; // Power down devices to save battery
            // go to sleep, would happen here 
        }
    }
    void handleStallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (get_joydir() != joy_down) pressure_target_psi = pressure_sensor.min_human();  // If in deadband or being pushed up, no pressure target
        else pressure_target_psi = map (hotrc_pc[VERT][FILT], hotrc_pc[VERT][DBBOT], hotrc_pc[VERT][MIN], pressure_sensor.min_human(), pressure_sensor.max_human());  // Scale joystick value to pressure adc setpoint
        if (!tachometer.engine_stopped()) updateMode(HOLD);  // If we started the car, enter hold mode once starter is released
        if (starter || !tachometer.engine_stopped()) updateMode(HOLD);  // If we started the car, enter hold mode once starter is released
    }
    void handleHoldMode() {
        if (we_just_switched_modes) {  // Release throttle and push brake upon entering hold mode
            if (!autostop_disabled) {
                if (speedometer.car_stopped()) pressure_target_psi = pressure_sensor.filt() + starter ? pressure_panic_increment_psi : pressure_hold_increment_psi; // If the car is already stopped then just add a touch more pressure and then hold it.
                else if (pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = starter ? pressure_panic_initial_psi : pressure_hold_initial_psi;  //  These hippies need us to stop the car for them
            }
            brakeIntervalTimer.reset();
            stopcarTimer.reset();
            joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
        }
        throttle.goto_idle();  // Let off gas (if gas using PID mode) and keep target updated to possibly changing idle value
        if (brakeIntervalTimer.expireset() && !speedometer.car_stopped() && !stopcarTimer.expired() && !autostop_disabled)
            pressure_target_psi = min (pressure_target_psi + starter ? pressure_panic_increment_psi : pressure_hold_increment_psi, pressure_sensor.max_human());  // If the car is still moving, push harder
        if (get_joydir() != joy_up) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && !starter && !hotrc_radio_lost) updateMode(FLY); // Enter Fly Mode upon joystick movement from center to above center  // Possibly add "&& car_stopped()" to above check?
    }
    void handleFlyMode() {
        if (we_just_switched_modes) car_hasnt_moved = speedometer.car_stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
        joydir = get_joydir();
        if (car_hasnt_moved) {
            if (joydir != joy_up) updateMode(HOLD);  // Must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedometer.car_stopped()) car_hasnt_moved = false;  // Once car moves, we're allowed to release the trigger without falling out of fly mode
        }
        else if (speedometer.car_stopped()) updateMode(HOLD);  // Go to Hold Mode if we have come to a stop after moving  // && hotrc_pc[VERT][FILT] <= hotrc_pc[VERT][DBBOT]

        if (!sim.simulating(sensor::joy) && hotrc_radio_lost) updateMode(HOLD);  // Radio must be good to fly. This should already be handled elsewhere but another check can't hurt
        else {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            if (joydir == joy_up)  // If we are trying to accelerate, scale joystick value to determine gas setpoint
                throttle.set_target (map (hotrc_pc[VERT][FILT], hotrc_pc[VERT][DBTOP], hotrc_pc[VERT][MAX], throttle.idlespeed(), tach_govern_rpm));
            else throttle.goto_idle();  // Else let off gas (if gas using PID mode)
            
            if (joydir == joy_down)  // If we are trying to brake, scale joystick value to determine brake pressure setpoint
                pressure_target_psi = map (hotrc_pc[VERT][FILT], hotrc_pc[VERT][DBBOT], hotrc_pc[VERT][MIN], pressure_sensor.min_human(), pressure_sensor.max_human());
            else pressure_target_psi = pressure_sensor.min_human();  // Else let off the brake   
        }
        // Cruise mode can be entered/exited by pressing a controller button, or exited by holding the brake on full for a half second
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (flycruise_toggle_request) updateMode(CRUISE);
        flycruise_toggle_request = false;
    }
    void handleCruiseMode() {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            speedo_target_mph = speedometer.filt();
            pressure_target_psi = pressure_sensor.min_human();  // Let off the brake and keep it there till out of Cruise mode
            throttle.set_target(tachometer.filt());  // Start off with target set to current tach value
            gestureFlyTimer.reset();  // reset gesture timer
            cruise_trigger_released = false;  // in case trigger is being pulled as cruise mode is entered, the ability to adjust is only unlocked after the trigger is subsequently released to the center
            gas_pulse_cruise_us = gas_pulse_out_us;  //  if cruise_fixed throttle is true, this variable stores the setpoint of throttle angle
            gas_pulse_adjustpoint_us = gas_pulse_cruise_us;  // Pull of trigger away from center in either direction starts a setpoint adjustment, scaled from *your current setpoint* (not from the center value) to the relevant min or max extreme 
            cruise_ctrl_extent_pc = hotrc_pc[VERT][CENT];  // After an adjustment, need this to prevent setpoint from following the trigger back to center as you release it
            cruise_adjusting = false;
        }
        joydir = get_joydir();
        if (joydir == joy_cent) {
            cruise_trigger_released = true;
            cruise_ctrl_extent_pc = hotrc_pc[VERT][CENT];
            cruise_adjusting = false;
        }
        else if (joydir == joy_down && !cruise_speed_lowerable) updateMode(FLY);
        else if (cruise_trigger_released) {  // adjustments disabled until trigger has been to center at least once since going to cruise mode
            float ctrlratio = (std::abs(hotrc_pc[VERT][FILT]) - hotrc_pc[VERT][DBTOP]) / (hotrc_pc[VERT][MAX] - hotrc_pc[VERT][DBTOP]);
            if (cruise_setpoint_mode == throttle_delta) {
                if (cruise_adjusting) gas_pulse_cruise_us -= joydir * ctrlratio * cruise_delta_max_us_per_s * cruiseDeltaTimer.elapsed() / 1000000.0;
                cruiseDeltaTimer.reset(); 
            }
            else if (std::abs(hotrc_pc[VERT][FILT]) >= cruise_ctrl_extent_pc) {  // to avoid the adjustments following the trigger back to center when released
                if (cruise_setpoint_mode == throttle_angle) {
                    if (!cruise_adjusting) gas_pulse_adjustpoint_us = gas_pulse_cruise_us;  // When beginning adjustment, save current throttle pulse value to use as adjustment endpoint
                    gas_pulse_cruise_us = gas_pulse_adjustpoint_us + ctrlratio * cruise_angle_attenuator * (((joydir == joy_up) ? gas_pulse_govern_us : gas_pulse_ccw_closed_us) - gas_pulse_adjustpoint_us);
                }
                else if (cruise_setpoint_mode == pid_suspend_fly) {
                    if (!cruise_adjusting) tach_adjustpoint_rpm = tachometer.filt();
                    throttle.set_target(tach_adjustpoint_rpm + ctrlratio * (((joydir == joy_up) ? tach_govern_rpm : throttle.idlespeed()) - tach_adjustpoint_rpm));
                }
                cruise_ctrl_extent_pc = std::abs(hotrc_pc[VERT][FILT]);
            }
            gas_pulse_cruise_us = constrain(gas_pulse_cruise_us, gas_pulse_govern_us, gas_pulse_ccw_closed_us);
            cruise_adjusting = true;
        }
        if (flycruise_toggle_request) updateMode(FLY);  // Go to fly mode if hotrc ch4 button pushed
        flycruise_toggle_request = false;
        // If joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (hotrc_pc[VERT][FILT] > hotrc_pc[VERT][MIN] + flycruise_vert_margin_pc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) updateMode(FLY);  // New gesture to drop to fly mode is hold the brake all the way down for more than X ms
        if (speedometer.car_stopped()) updateMode(HOLD);  // In case we slam into camp Q woofer stack, get out of cruise mode
    }

    void handleCalMode() {  // Calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration. Don't use it unless you know how.
        if (we_just_switched_modes) {  // Entering Cal mode: From fully shut down state, open simulator and long-press the Cal button. Each feature starts disabled but can be enabled with the tuner.
            calmode_request = false;
            cal_pot_gasservo_mode = false;
            cal_pot_gasservo_ready = false;
            cal_joyvert_brkmotor_mode = false;
        }
        else if (calmode_request) updateMode(SHUTDOWN);
        float temp = pot.mapToRange(gas_pulse_ccw_max_us, gas_pulse_cw_min_us);
        cal_pot_gasservo_ready = (temp <= (float)gas_pulse_ccw_closed_us && temp >= (float)gas_pulse_cw_open_us);
    }
};
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
// # Actions: Close throttle, and Apply brake to stop car, continue to ensure it stays stopped.
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