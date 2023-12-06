#pragma once
class RunModeManager {  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
  private:
    int _joydir;
    float cruise_ctrl_extent_pc, adjustpoint;       // During cruise adjustments, saves farthest trigger position read
    bool cruise_trigger_released = false;
    static const uint32_t gesture_flytimeout_us = 1250000;         // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
    static const uint32_t motor_park_timeout_us = 4000000;         // Timeout when parking motors if they don't park for whatever reason (in us)
    static const uint32_t sleep_inactivity_timeout_us = 180000000; // How long to wait around in shutdown mode before going to asleep mode (in us)
    Timer gestureFlyTimer, cruiseDeltaTimer, pwrup_timer, motor_park_timer; 
    uint32_t pwrup_timeout = 500000;
    Encoder* encoder;
    Display* display;
    int oldmode;
    bool autostopping_last = false;
  public:
    int mode = SHUTDOWN;
    bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
    bool joy_centered = false;  // minor state variable for hold mode
    RunModeManager(Display* _display, Encoder* _encoder) {
        display = _display;
        encoder = _encoder;
        motor_park_timer.set(motor_park_timeout_us);
        sleep_inactivity_timer.set(sleep_inactivity_timeout_us);
    }
    int mode_logic() {
        updateMode(); // Update the current mode if needed, this also sets we_just_switched_modes
        if (mode == BASIC) run_basicMode(); // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        else if (mode == ASLEEP) run_asleepMode();
        else if (mode == SHUTDOWN) run_shutdownMode();
        else if (mode == STALL) run_stallMode();
        else if (mode == HOLD) run_holdMode();
        else if (mode == FLY) run_flyMode();
        else if (mode == CRUISE) run_cruiseMode();
        else if (mode == CAL) run_calMode();
        else {  // Obviously this should never happen
            Serial.println (F("Error: Invalid runmode entered"));
            mode = SHUTDOWN;
        }
        return mode;
    }
    void kick() { sleep_inactivity_timer.reset(); }  // call when user activity is detected, to prevent shutdown mode timeout to asleep mode
  private:
    bool autostop(int _cmd = REQ_NA) {
        int cmd = _cmd;
        if (autostop_disabled) autostopping = false;
        else {
            if (autostopping) {
                if (speedo.car_stopped() || brake.stopcar_timer.expired()) cmd = REQ_OFF; 
                else if (brake.interval_timer.expireset()) brake.autostop_increment(panicstop);
            }
            if (cmd == REQ_TOG) cmd = !autostopping;
            if (autostopping && cmd == REQ_OFF) {
                brake.set_pidtarg(0);
                autostopping = false;
            }
            else if (!autostopping && cmd == REQ_ON && !speedo.car_stopped()) {
                gas.idlectrl.goto_idle();  // Keep target updated to possibly changing idle value
                brake.autostop_initial(panicstop);
                brake.interval_timer.reset();
                brake.stopcar_timer.reset();
                autostopping = true;
            }
        }
        return autostopping;
    }
    bool park_motors(int _cmd = REQ_NA) {
        int cmd = _cmd;
        if (park_the_motors) {
            bool brake_parked = brkpos.parked();
            bool gas_parked = ((std::abs(gas.pc_to_nat(gas.pc[OUT]) - gas.nat[PARKED]) < 1) && gas.servo_delay_timer.expired());
            if ((brake_parked && gas_parked) || motor_park_timer.expired()) cmd = REQ_OFF;
        }
        if (cmd == REQ_TOG) cmd = !park_the_motors;
        if (park_the_motors && cmd == REQ_OFF) park_the_motors = false;
        else if (!park_the_motors && cmd == REQ_ON) {
            gas.servo_delay_timer.reset();  // Ensure we give the servo enough time to move to position
            motor_park_timer.reset();  // Set a timer to timebox this effort
            park_the_motors = true;
        }
        return park_the_motors;
    }
    // void sleep_for(uint32_t wakeup_us) {
    //     esp_sleep_enable_timer_wakeup(wakeup_us);
    //     esp_deep_sleep_start();
    // }
    void updateMode() {
        if (basicmodesw) mode = BASIC;  // if basicmode switch on --> Basic Mode
        else if ((mode != CAL) && (mode != ASLEEP)) {
            if (panicstop || !ignition) mode = SHUTDOWN;
            else if (tach.engine_stopped()) mode = STALL;;  // otherwise if engine not running --> Stall Mode
        }
        we_just_switched_modes = (mode != oldmode);  // currentMode should not be changed after this point in loop
        if (we_just_switched_modes) {
            disp_runmode_dirty = true;
            cleanup_state_variables();
        }
        oldmode = mode;
    }
    void cleanup_state_variables() {
        if (oldmode == BASIC) park_motors(REQ_OFF);
        else if (oldmode == ASLEEP);
        else if (oldmode == SHUTDOWN) {
            autostop(REQ_OFF);
            park_motors(REQ_OFF);
            shutdown_incomplete = false;
        }
        else if (oldmode == STALL);
        else if (oldmode == HOLD) {
            autostop(REQ_OFF);
            joy_centered = false;
            starter_request = REQ_OFF;  // Stop any in-progress startings
        }
        else if (oldmode == FLY) car_hasnt_moved = false;
        else if (oldmode == CRUISE) cruise_adjusting = false;
        else if (oldmode == CAL) cal_gasmode = cal_gasmode_ready = cal_gasmode_request = cal_brakemode = false;
    }
    void run_basicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {
            park_motors(REQ_ON);  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            powering_up = false;  // to cover unlikely edge case where basic mode switch is enabled during wakeup from asleep mode
        }
        else if (park_the_motors) park_motors();  // Update motor parking until finished
        if (!basicmodesw && !tach.engine_stopped()) mode = speedo.car_stopped() ? HOLD : FLY;  // If we turned off the basic mode switch with engine running, change modes. If engine is not running, we'll end up in Stall Mode automatically
    }
    void run_asleepMode() {  // turns off syspower and just idles. sleep_request are handled here or in shutdown mode below
        if (we_just_switched_modes) {
            set_syspower(LOW); // Power down devices to save battery
            sleep_request = REQ_NA;
            powering_up = false;
        }
        if (encoder->pressed() || sleep_request == REQ_OFF || sleep_request == REQ_TOG) {
            set_syspower(HIGH);
            sleep_request = REQ_NA;
            pwrup_timer.set(pwrup_timeout);
            powering_up = true;
        }
        if (powering_up && pwrup_timer.expired()) {
            display->all_dirty();  // tells display to redraw everything. display must set back to false
            mode = SHUTDOWN;
        }
    }
    void run_shutdownMode() { // In shutdown mode we stop the car if it's moving, park the motors, go idle for a while and eventually sleep.
        if (we_just_switched_modes) {              
            gas.idlectrl.goto_idle();  //  Release the throttle 
            shutdown_incomplete = !(powering_up);
            powering_up = calmode_request = false;
            sleep_request = REQ_NA;
            park_motors(REQ_OFF);  // stop any in-progress parking
            if (!speedo.car_stopped()) autostop(REQ_ON);  // if car is moving begin autostopping
        }
        else if (shutdown_incomplete) {  // first we need to stop the car and release brakes and gas before shutting down  
            if (park_the_motors) shutdown_incomplete = park_motors(); // update any in-progress motor parking. shutdown is complete once parked
            else if (!autostop()) park_motors(REQ_ON);  // start parking motors directly after final autostop update
            if (!shutdown_incomplete) sleep_inactivity_timer.reset();  // upon shutdown completion, start the sleep timer
        }
        else {  // if shutdown is complete
            if (calmode_request) mode = CAL;  // if fully shut down and cal mode requested, go to cal mode
            if (sleep_inactivity_timer.expired() || sleep_request == REQ_ON || sleep_request == REQ_TOG) mode = ASLEEP;
        }
        sleep_request == REQ_NA;
        if ((speedo.car_stopped() || allow_rolling_start) && ignition && !panicstop && !tach.engine_stopped()) mode = HOLD;  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
    }
    void run_stallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (hotrc.joydir(VERT) != JOY_DN) brake.set_pidtarg(0);  // If in deadband or being pushed up, no pressure target
        else brake.set_pidtarg(map (hotrc.pc[VERT][FILT], hotrc.pc[VERT][DBBOT], hotrc.pc[VERT][OPMIN], 0.0, 100.0));  // Scale joystick value to pressure adc setpoint
        if (starter || !tach.engine_stopped()) mode = HOLD;  // If we started the car, enter hold mode once starter is released
    }
    void run_holdMode() {
        if (we_just_switched_modes) joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
        if (!speedo.car_stopped()) autostop(REQ_ON);
        gas.idlectrl.goto_idle();  // Let off gas (if gas using PID mode) and keep target updated to possibly changing idle value
        if (hotrc.joydir(VERT) != JOY_UP) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && !starter && !hotrc.radiolost()) mode = FLY; // Enter Fly Mode upon joystick movement from center to above center  // Possibly add "&& car_stopped()" to above check?
    }
    void run_flyMode() {
        if (we_just_switched_modes) car_hasnt_moved = speedo.car_stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
        _joydir = hotrc.joydir(VERT);
        if (car_hasnt_moved) {
            if (_joydir != JOY_UP) mode = HOLD;  // Must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.car_stopped()) car_hasnt_moved = false;  // Once car moves, we're allowed to release the trigger without falling out of fly mode
        }
        else if (speedo.car_stopped() && hotrc.joydir() != JOY_UP) mode = HOLD;  // Go to Hold Mode if we have come to a stop after moving  // && hotrc.pc[VERT][FILT] <= hotrc.pc[VERT][DBBOT]
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) mode = HOLD;  // Radio must be good to fly. This should already be handled elsewhere but another check can't hurt
        else {  // Update the gas and brake targets based on joystick position, for the PIDs to drive
            if (_joydir == JOY_UP)  // If we are trying to accelerate, scale joystick value to determine gas setpoint
                gas.pid.set_target(map(hotrc.pc[VERT][FILT], hotrc.pc[VERT][DBTOP], hotrc.pc[VERT][OPMAX], gas.idlectrl.idle_rpm, tach.govern_rpm()));
            else gas.idlectrl.goto_idle();  // Else let off gas (if gas using PID mode)
            
            if (_joydir == JOY_DN)  // If we are trying to brake, scale joystick value to determine brake pressure setpoint
                brake.set_pidtarg(map(hotrc.pc[VERT][FILT], hotrc.pc[VERT][DBBOT], hotrc.pc[VERT][OPMIN], 0.0, 100.0));
            else brake.set_pidtarg(0);  // Else let off the brake   
        }
        if (flycruise_toggle_request) mode = CRUISE;  // enter cruise mode by pressing hrc ch4 button
        flycruise_toggle_request = false;
    }
    void run_cruiseMode() {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            brake.set_pidtarg(0);  // Let off the brake and keep it there till out of Cruise mode
            gas.cruisepid.set_target(speedo.filt());  // set pid loop speed target to current speed  (for PID_SUSPEND_FLY mode)
            gas.pid.set_target(tach.filt());  // initialize pid output (rpm target) to current rpm  (for PID_SUSPEND_FLY mode)
            gas.cruise_target_pc = gas.pc[OUT];  //  set target throttle angle to current throttle angle  (for THROTTLE_ANGLE/THROTTLE_DELTA modes)
            cruise_adjusting = cruise_trigger_released = false;  // in case trigger is being pulled as cruise mode is entered, the ability to adjust is only unlocked after the trigger is subsequently released to the center
            gestureFlyTimer.set(gesture_flytimeout_us);  // initialize brake-trigger timer
        }
        _joydir = hotrc.joydir(VERT);
        if (_joydir == JOY_CENT) {
            if (cruise_adjusting) gas.cruisepid.set_target(speedo.filt());
            cruise_adjusting = false;
            cruise_trigger_released = true;
            cruise_ctrl_extent_pc = hotrc.pc[VERT][CENT];  // After an adjustment, need this to prevent setpoint from following the trigger back to center as you release it
        }
        else if (_joydir == JOY_DN && !cruise_speed_lowerable) mode = FLY;
        else if (cruise_trigger_released) {  // adjustments disabled until trigger has been to center at least once since going to cruise mode
            float ctrlratio = (std::abs(hotrc.pc[VERT][FILT]) - hotrc.pc[VERT][DBTOP]) / (hotrc.pc[VERT][OPMAX] - hotrc.pc[VERT][DBTOP]);
            if (cruise_setpoint_mode == THROTTLE_DELTA) {
                if (cruise_adjusting) gas.cruise_target_pc += _joydir * ctrlratio * cruise_delta_max_pc_per_s * cruiseDeltaTimer.elapsed() / 1000000.0;
                cruiseDeltaTimer.reset(); 
            }
            else if (std::abs(hotrc.pc[VERT][FILT]) >= cruise_ctrl_extent_pc) {  // to avoid the adjustments following the trigger back to center when released
                if (cruise_setpoint_mode == THROTTLE_ANGLE) {
                    if (!cruise_adjusting) adjustpoint = gas.cruise_target_pc;  // When beginning adjustment, save current throttle pulse value to use as adjustment endpoint
                    gas.cruise_target_pc = adjustpoint + ctrlratio * cruise_angle_attenuator * (((_joydir == JOY_UP) ? 100.0 : 0.0) - adjustpoint);
                }
                else if (cruise_setpoint_mode == PID_SUSPEND_FLY) {
                    if (!cruise_adjusting) adjustpoint = tach.filt();
                    gas.pid.set_target(adjustpoint + ctrlratio * (((_joydir == JOY_UP) ? tach.govern_rpm() : gas.idlectrl.idle_rpm) - adjustpoint));
                }
                cruise_ctrl_extent_pc = std::abs(hotrc.pc[VERT][FILT]);
            }
            gas.cruise_target_pc = constrain(gas.cruise_target_pc, 0.0, 100.0);
            cruise_adjusting = true;
        }
        if (flycruise_toggle_request) mode = FLY;  // Go to fly mode if hotrc ch4 button pushed
        flycruise_toggle_request = false;
        // If joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (hotrc.pc[VERT][FILT] > hotrc.pc[VERT][OPMIN] + flycruise_vert_margin_pc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) mode = FLY;  // New gesture to drop to fly mode is hold the brake all the way down for more than X ms
        if (speedo.car_stopped()) mode = (_joydir == JOY_UP) ? FLY : HOLD;  // In case we slam into camp Q woofer stack, get out of cruise mode.
    }
    void run_calMode() {  // Calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration. Don't use it unless you know how.
        if (we_just_switched_modes) calmode_request = cal_gasmode = cal_gasmode_ready = cal_brakemode = cal_gasmode_request = false;
        else if (calmode_request) mode = SHUTDOWN;
        float temp = pot.mapToRange(0.0, 180.0);
        cal_gasmode_ready = (temp >= gas.nat[PARKED] && temp <= gas.nat[OPMAX]);
        if (!cal_gasmode_request) cal_gasmode = false;
        else if (cal_gasmode_ready) cal_gasmode = true;
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
// THROTTLE_DELTA mode holds a fixed throttle servo position as long as the trigger is centered, and if not,
// it adjusts the setpoint up or down proportional to how far and how long you hold the trigger away from center.
// If you panic and push full brake for over 500ms, it will drop to fly mode and then push brakes.
// Cruise modes : Pick from 3 different styles for adjustment of cruise setpoint. I prefer THROTTLE_DELTA.
//    PID_SUSPEND_FLY : (PID) Moving trigger from center pauses the pid and lets you adjust the rpm target directly like Fly mode does. Whatever speed you're at when trigger releases is new pid target  
//    THROTTLE_ANGLE : Cruise locks throttle angle, instead of pid. Moving trigger from center adjusts setpoint proportional to how far you push it before releasing (and reduced by an attenuation factor)
//    THROTTLE_DELTA : Cruise locks throttle angle, instead of pid. Any non-center trigger position continuously adjusts setpoint proportional to how far it's pulled over time (up to a specified maximum rate)
//
// ** Cal Mode **
// This mode allows direct control of some actuators without respecting limits of motion, for purpose of
// calibrating those very limits. It can be entered from shutdown mode with simulator on by long-pressing the CAL
// button. Be careful with it.