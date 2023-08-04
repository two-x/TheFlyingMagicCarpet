#pragma once
#ifndef RUNMODEMANAGER_H
#define RUNMODEMANAGER_H

#include "display.h"

class RunModeManager {
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
            if (serial_debugging) Serial.println (F("Error: Invalid runmode entered"));
            updateMode(SHUTDOWN);
        }
        return modeChanger();
    }

private:
    runmodes _currentMode; // note these are more here in case we eventually don't use the globals
    runmodes _oldMode;
    
    void updateMode(runmodes newmode) { _currentMode = newmode; }

    runmodes modeChanger() {
        if (basicmodesw) _currentMode = BASIC;  // if basicmode switch on --> Basic Mode
        else if (_currentMode != CAL && (panic_stop || !ignition)) _currentMode = SHUTDOWN;
        else if (_currentMode != CAL && (starter || engine_stopped())) _currentMode = STALL;;  // otherwise if engine not running --> Stall Mode
        we_just_switched_modes = (_currentMode != _oldMode);  // currentMode should not be changed after this point in loop
        if (we_just_switched_modes) {
            disp_runmode_dirty = true;
            syspower = HIGH;
        }
        _oldMode = _currentMode;
        return _currentMode;
    }

    void handleBasicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
            motorParkTimer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;  // Flags the motor parking to happen
        }
        else if (!engine_stopped() && !basicmodesw) updateMode(HOLD);  // If we turned off the basic mode switch with engine running, go to Hold mode. If engine is not running, we'll end up in Stall Mode automatically
    }

    void handleShutdownMode() { // In shutdown mode we stop the car if it's moving then park the motors.
        if (we_just_switched_modes) {  // If basic switch is off, we need to stop the car and release brakes and gas before shutting down                
            idler.goto_idle();  //  Release the throttle 
            shutdown_complete = false;
            shutdown_color = LPNK;
            disp_runmode_dirty = true;
            calmode_request = false;
            park_the_motors = false;
            if (!car_stopped()) {
                if (panic_stop && pressure_target_psi < pressure_panic_initial_psi) pressure_target_psi = pressure_panic_initial_psi;
                else if (!panic_stop && pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = pressure_hold_initial_psi;
                brakeIntervalTimer.reset();
                stopcarTimer.reset();
            }
        }
        else if ((car_stopped() || allow_rolling_start) && ignition && !panic_stop && !engine_stopped() && !starter) updateMode(HOLD);  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
        if (!shutdown_complete) {  // If we haven't yet stopped the car and then released the brakes and gas all the way
            if (car_stopped() || stopcarTimer.expired()) {  // If car has stopped, or timeout expires, then release the brake
                if (shutdown_color == LPNK) {  // On first time through here
                    park_the_motors = true;  // Flags the motor parking to happen, only once
                    gasServoTimer.reset();  // Ensure we give the servo enough time to move to position
                    motorParkTimer.reset();  // Set a timer to timebox this effort
                    shutdown_color = DPNK;
                    disp_runmode_dirty = true;
                }
                else if (!park_the_motors) {  // When done parking the motors we can finish shutting down
                    shutdown_complete = true;
                    shutdown_color = colorcard[SHUTDOWN];
                    disp_runmode_dirty = true;
                    sleepInactivityTimer.reset();
                }
            }
            else if (brakeIntervalTimer.expireset()) {
                pressure_target_psi = pressure_target_psi + (panic_stop) ? pressure_panic_increment_psi : pressure_hold_increment_psi;  // Slowly add more brakes until car stops
                tach_target_rpm = tach_idle_rpm;  // Keep target updated to possibly changing idle value
            }
        }
        else if (calmode_request) updateMode(CAL);  // if fully shut down and cal mode requested, go to cal mode
        else if (sleepInactivityTimer.expired()) {
            syspower = LOW; // Power down devices to save battery
            // go to sleep, would happen here 
        }
    }

    void handleStallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (we_just_switched_modes) {
            remote_starting = false;
            remote_start_toggle_request = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][BOT]) pressure_target_psi = pressure_sensor.get_min_human();  // If in deadband or being pushed up, no pressure target
        else pressure_target_psi = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][BOT], (float)ctrl_lims_adc[ctrl][VERT][MIN], pressure_sensor.get_min_human(), pressure_sensor.get_max_human());  // Scale joystick value to pressure adc setpoint
        if (!starter && !engine_stopped()) updateMode(HOLD);  // If we started the car, enter hold mode once starter is released
    }

    void handleHoldMode() {
        if (we_just_switched_modes) {  // Release throttle and push brake upon entering hold mode
            tach_target_rpm = tach_idle_rpm;  // Let off gas (if gas using PID mode)
            if (car_stopped()) pressure_target_psi = pressure_sensor.get_filtered_value() + pressure_hold_increment_psi; // If the car is already stopped then just add a touch more pressure and then hold it.
            else if (pressure_target_psi < pressure_hold_initial_psi) pressure_target_psi = pressure_hold_initial_psi;  //  These hippies need us to stop the car for them
            brakeIntervalTimer.reset();
            stopcarTimer.reset();
            joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
        }
        if (brakeIntervalTimer.expireset()) {  // On an interval ...
            tach_target_rpm = tach_idle_rpm;  // Keep target updated to possibly changing idle value
            if (!car_stopped() && !stopcarTimer.expired()) pressure_target_psi = min (pressure_target_psi + pressure_hold_increment_psi, pressure_sensor.get_max_human());  // If the car is still moving, push harder
        }
        if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && (ctrl == JOY || hotrc_radio_detected)) updateMode(FLY); // Enter Fly Mode upon joystick movement from center to above center  // Possibly add "&& car_stopped()" to above check?
    }

     void handleFlyMode() {
        if (we_just_switched_modes) {
            gesture_progress = 0;
            gestureFlyTimer.set (gesture_flytimeout_us); // Initialize gesture timer to already-expired value
            cruise_sw_held = false;
            // cruiseSwTimer.reset();  // Needed if momentary cruise button is used to go to cruise mode
            flycruise_toggle_request = false;
            car_initially_moved = !car_stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
        }
        if (!car_initially_moved) {
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][TOP]) updateMode(HOLD);  // Must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!car_stopped()) car_initially_moved = true;  // Once car moves, we're allowed to stay in fly mode
        }
        else if (car_stopped()) updateMode(HOLD);  // Go to Hold Mode if we have come to a stop after moving  // && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][BOT]
        if (ctrl == HOTRC && !simulator.simulating(SimOption::joy) && !hotrc_radio_detected) updateMode(HOLD);  // Radio must be good to fly. This should already be handled elsewhere but another check can't hurt
        else {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP])  {  // If we are trying to accelerate, scale joystick value to determine gas setpoint
                tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], tach_idle_rpm, tach_govern_rpm);
            }
            else tach_target_rpm = tach_idle_rpm;  // Else let off gas (if gas using PID mode)
            if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT])  {  // If we are trying to brake, scale joystick value to determine brake pressure setpoint
                pressure_target_psi = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][BOT], (float)ctrl_lims_adc[ctrl][VERT][MIN], pressure_sensor.get_min_human(), pressure_sensor.get_max_human());
            }
            else pressure_target_psi = pressure_sensor.get_min_human();  // Else let off the brake   
        }
        // Cruise mode can be entered by pressing a controller button, or by holding the brake on full for a half second. Which epends on the cruise_gesturing flag.
        // The gesture involves pushing the joystick from the center to the top, then to the bottom, then back to center, quickly enough.
        if (flycruise_toggle_request) updateMode(CRUISE);
        flycruise_toggle_request = false;  // Reset the toggle request
        if (ctrl == JOY) {
            if (cruise_gesturing) {  // If we are configured to use joystick gestures to go to cruise mode, the gesture is 
                if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP]) {  // Re-zero gesture timer for potential new gesture whenever joystick at center
                    gestureFlyTimer.reset();
                }
                if (gestureFlyTimer.expired()) gesture_progress = 0; // If gesture timeout has expired, cancel any in-progress gesture
                else {  // Otherwise check for successful gesture motions
                    if (!gesture_progress && ctrl_pos_adc[VERT][FILT] >= ctrl_lims_adc[ctrl][VERT][MAX] - flycruise_vert_margin_adc) {  // If joystick quickly pushed to top, step 1 of gesture is successful
                        gesture_progress++;
                        gestureFlyTimer.reset();
                    }
                    else if (gesture_progress == 1 && ctrl_pos_adc[VERT][FILT] <= ctrl_lims_adc[ctrl][VERT][MIN] + flycruise_vert_margin_adc) {  // If joystick then quickly pushed to bottom, step 2 succeeds
                        gesture_progress++;
                        gestureFlyTimer.reset();
                    }
                    else if (gesture_progress == 2 && ctrl_pos_adc[VERT][FILT] >= ctrl_db_adc[VERT][BOT] && ctrl_pos_adc[VERT][FILT] <= ctrl_db_adc[VERT][TOP]) {  // If joystick then quickly returned to center, go to Cruise mode
                        updateMode(CRUISE);
                    }        
                }
            }
            // This was when the thought was to add a momentary button to the joystick to toggle cruise <-> fly mode
            // if (!cruise_sw) {  // If button not currently pressed
            //     if (cruise_sw_held && cruiseSwTimer.expired()) runmode = CRUISE;  // After a long press of sufficient length, upon release enter Cruise mode
            //     cruise_sw_held = false;  // Cancel button held state
            // }
            // else if (!cruise_sw_held) {  // If the button just now got pressed
            //     cruiseSwTimer.reset(); // Start hold time timer
            //     cruise_sw_held = true;  // Get into button held state
            // }
        }
    }

    void handleCruiseMode() {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            speedo_target_mph = speedo_filt_mph;
            pressure_target_psi = pressure_sensor.get_min_human();  // Let off the brake and keep it there till out of Cruise mode
            tach_target_rpm = tach_filt_rpm;  // Start off with target set to current tach value
            // cruiseQPID.SetCenter (tach_filt_rpm);
            gestureFlyTimer.reset();  // reset gesture timer
            cruise_sw_held = false;
            cruise_adjusting = false;
            flycruise_toggle_request = false;
        }
        if (ctrl_pos_adc[VERT][FILT] > ctrl_db_adc[VERT][TOP]) {  // When joystick vert above center, increase the throttle target proportional to how far off center
            if (!cruise_adjusting) tach_adjustpoint_rpm = tach_filt_rpm;  // When beginning adjustment, save current tach value to use as adjustment low endpoint 
            cruise_adjusting = true;  // Suspend pid loop control of gas
            tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_db_adc[VERT][TOP], (float)ctrl_lims_adc[ctrl][VERT][MAX], tach_adjustpoint_rpm, tach_govern_rpm);
        }
        else if (ctrl_pos_adc[VERT][FILT] < ctrl_db_adc[VERT][BOT]) {  // When joystick vert below center, decrease the throttle target proportional to how far off center
            if (!cruise_adjusting) tach_adjustpoint_rpm = tach_filt_rpm;  // When beginning adjustment, save current tach value to use as adjustment high endpoint 
            cruise_adjusting = true;  // Suspend pid loop control of gas
            tach_target_rpm = map ((float)ctrl_pos_adc[VERT][FILT], (float)ctrl_lims_adc[ctrl][VERT][MIN], (float)ctrl_db_adc[VERT][BOT], tach_idle_rpm, tach_adjustpoint_rpm);
        }
        else if (cruise_adjusting) {  // When joystick at center, the target speed stays locked to the value it was when joystick goes to center
            tach_target_rpm = tach_filt_rpm;
            // cruiseQPID.SetCenter (tach_filt_rpm);
            cruise_adjusting = false;
        }
        if (!cruise_adjusting) cruiseAntiglitchTimer.reset();  // Anti-glitch timer attempts to keep very short joystick sensor glitches from going into adjust mode
        else if (cruiseAntiglitchTimer.expired()) speedo_target_mph = speedo_filt_mph;  // May be unneccesary now that our readings are stable.  Remove?  Anyway, need to review the logic

        if (flycruise_toggle_request) updateMode(FLY);  // Go to fly mode if hotrc ch4 button pushed
        flycruise_toggle_request = false;  // Reset the toggle request

        // If joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (ctrl_pos_adc[VERT][FILT] > ctrl_lims_adc[ctrl][VERT][MIN] + flycruise_vert_margin_adc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) updateMode(FLY);  // New gesture to drop to fly mode is hold the brake all the way down for more than X ms
        
        if (car_stopped()) updateMode(HOLD);  // In case we slam into a brick wall, get out of cruise mode
    }

    void handleCalMode() {  // Calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration. Don't use it unless you know how.
        if (we_just_switched_modes) {  // Entering Cal mode: From fully shut down state, open simulator and long-press the Cal button. Each feature starts disabled but can be enabled with the tuner.
            calmode_request = false;
            cal_pot_gas_ready = false;
            cal_pot_gasservo = false;
            cal_joyvert_brkmotor = false;
        }
        else if (calmode_request) updateMode(SHUTDOWN);
        
        if (!cal_pot_gas_ready) {
            float temp = pot.mapToRange(gas_pulse_ccw_max_us, gas_pulse_cw_min_us);
            if (temp <= (float)gas_pulse_ccw_closed_us && temp >= (float)gas_pulse_cw_open_us) cal_pot_gas_ready = true;
        }
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
#endif  // RUNMODEMANAGER.H