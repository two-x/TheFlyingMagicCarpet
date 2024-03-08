#pragma once
class RunModeManager {  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
  private:
    int _joydir;
    Timer gestureFlyTimer{1250000};  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
    Timer pwrup_timer{500000};  // Timeout when parking motors if they don't park for whatever reason (in us)
    Timer shutdown_timer{5000000};
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
  private:
    void updateMode() {
        if (basicmodesw) mode = BASIC;  // if basicmode switch on --> Basic Mode
        else if ((mode != CAL) && (mode != ASLEEP)) {
            if (panicstop || !ignition) mode = SHUTDOWN;
            else if (tach.engine_stopped()) mode = STALL;;  // otherwise if engine not running --> Stall Mode
        }
        we_just_switched_modes = (mode != oldmode);  // currentMode should not be changed after this point in loop
        if (we_just_switched_modes) {
            display->disp_runmode_dirty = true;
            cleanup_state_variables();
        }
        oldmode = mode;
    }
    void cleanup_state_variables() {
        if (oldmode == BASIC);
        else if (oldmode == ASLEEP);
        else if (oldmode == SHUTDOWN) shutdown_incomplete = false;
        else if (oldmode == STALL);
        else if (oldmode == HOLD) {
            joy_centered = false;
            starter_request = REQ_OFF;  // Stop any in-progress startings
        }
        else if (oldmode == FLY) car_hasnt_moved = false;
        else if (oldmode == CRUISE) cruise_adjusting = false;
        else if (oldmode == CAL) cal_gasmode = cal_gasmode_request = cal_brakemode = false;
    }
    void run_basicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {
            gas.setmode(ParkMotor);  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            brake.setmode(ParkMotor);
            steer.setmode(OpenLoop);
            powering_up = false;  // to cover unlikely edge case where basic mode switch is enabled during wakeup from asleep mode
            watchdog.set_codemode(Parked);
        }
        if (!basicmodesw && !tach.engine_stopped()) mode = speedo.car_stopped() ? HOLD : FLY;  // If we turned off the basic mode switch with engine running, change modes. If engine is not running, we'll end up in Stall Mode automatically
    }
    void run_asleepMode() {  // turns off syspower and just idles. sleep_request are handled here or in shutdown mode below
        if (we_just_switched_modes) {
            if (saver_on_sleep) display->auto_saver(true);
            sleep_request = REQ_NA;
            powering_up = false;
            brake.setmode(Halt);
            steer.setmode(Halt);
            set_syspower(LOW); // Power down devices to save battery
        }
        if (hotrc.sw_event(CH4)) sleep_request = REQ_OFF; 
        if (encoder->button.pressed() || sleep_request == REQ_OFF || sleep_request == REQ_TOG) {
            set_syspower(HIGH);
            sleep_request = REQ_NA;
            pwrup_timer.set(pwrup_timeout);
            powering_up = true;
            display->auto_saver(false);
        }
        if (powering_up && pwrup_timer.expired()) mode = SHUTDOWN;  // display->all_dirty();  // tells display to redraw everything. display must set back to false
    }
    void run_shutdownMode() { // In shutdown mode we stop the car if it's moving, park the motors, go idle for a while and eventually sleep.
        if (we_just_switched_modes) {              
            shutdown_incomplete = !(powering_up);
            powering_up = calmode_request = false;
            sleep_request = REQ_NA;
            gas.setmode(ParkMotor);  // if car is moving begin autostopping
            brake.setmode(AutoStop);  // if car is moving begin autostopping
            steer.setmode(Halt);
            shutdown_timer.reset();
        }
        else if (shutdown_incomplete) {  // first we need to stop the car and release brakes and gas before shutting down
            if (shutdown_timer.expired()) shutdown_incomplete = false;
            if (brake.motormode != AutoStop) {
                if (brake.parked()) shutdown_incomplete = false;
                else brake.setmode(ParkMotor);
            }
        }
        else {  // if shutdown is complete
            brake.setmode(Halt);
            watchdog.set_codemode(Parked);
            if (hotrc.sw_event(CH4)) sleep_request = REQ_ON;
            if (sleep_inactivity_timer.expired() || sleep_request == REQ_ON || sleep_request == REQ_TOG) mode = ASLEEP;
        }
        sleep_request = REQ_NA;
        if (calmode_request) mode = CAL;  // if fully shut down and cal mode requested, go to cal mode
        if ((speedo.car_stopped() || allow_rolling_start) && ignition && !panicstop && !tach.engine_stopped()) mode = HOLD;  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
    }
    void run_stallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (we_just_switched_modes) {
            gas.setmode(OpenLoop);
            brake.setmode(ActivePID);
            steer.setmode(OpenLoop);
        }
        if (hotrc.sw_event(CH4)) starter_request = REQ_TOG;
        if (brake.motormode == Halt) brake.setmode(ActivePID);  // Can happen if starter attempt fails to autohold the brakes
        if (starter || !tach.engine_stopped()) mode = HOLD;  // If we started the car, enter hold mode once starter is released
        Serial.printf("%d/%d ", starter_request, starter);
    }
    void run_holdMode() {
        if (we_just_switched_modes) {
            joy_centered = false;  // Fly mode will be locked until the joystick first is put at or below center
            watchdog.set_codemode(Driving);
            gas.setmode(OpenLoop);
            brake.setmode(AutoHold);
            steer.setmode(OpenLoop);
        }
        if (hotrc.sw_event(CH4)) starter_request = REQ_OFF;
        if (hotrc.joydir(VERT) != JOY_UP) joy_centered = true; // Mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && !starter && !hotrc.radiolost()) mode = FLY; // Enter Fly Mode upon joystick movement from center to above center  // Possibly add "&& car_stopped()" to above check?
    }
    void run_flyMode() {
        if (we_just_switched_modes) {
            car_hasnt_moved = speedo.car_stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
            gas.setmode(OpenLoop);
            brake.setmode(ActivePID);
            steer.setmode(OpenLoop);
        }
        if (car_hasnt_moved) {
            if (hotrc.joydir(VERT) != JOY_UP) mode = HOLD;  // Must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.car_stopped()) car_hasnt_moved = false;  // Once car moves, we're allowed to release the trigger without falling out of fly mode
        }
        else if (speedo.car_stopped() && hotrc.joydir() != JOY_UP) mode = HOLD;  // Go to Hold Mode if we have come to a stop after moving  // && hotrc.pc[VERT][FILT] <= hotrc.pc[VERT][DBBOT]
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) mode = HOLD;  // Radio must be good to fly. This should already be handled elsewhere but another check can't hurt
        if (hotrc.sw_event(CH4)) mode = CRUISE;  // enter cruise mode by pressing hrc ch4 button
    }
    void run_cruiseMode() {
        if (we_just_switched_modes) {  // Upon first entering cruise mode, initialize things
            gas.setmode(Cruise);
            brake.setmode(Release);
            steer.setmode(OpenLoop);
            gestureFlyTimer.reset();  // initialize brake-trigger timer
        }
        if (hotrc.joydir(VERT) == JOY_DN && !cruise_speed_lowerable) mode = FLY;
        if (hotrc.sw_event(CH4)) mode = FLY;  // Go to fly mode if hotrc ch4 button pushed
        // If joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (hotrc.pc[VERT][FILT] > hotrc.pc[VERT][OPMIN] + flycruise_vert_margin_pc) gestureFlyTimer.reset();  // Keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) mode = FLY;  // New gesture to drop to fly mode is hold the brake all the way down for more than X ms
        if (speedo.car_stopped()) mode = (_joydir == JOY_UP) ? FLY : HOLD;  // In case we slam into camp Q woofer stack, get out of cruise mode.
    }
    void run_calMode() {  // Calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration. Don't use it unless you know how.
        if (we_just_switched_modes) {
            calmode_request = cal_gasmode_request = cal_brakemode = false;
            gas.setmode(Idle);
            brake.setmode(Halt);
            steer.setmode(Halt);
        }
        else if (calmode_request) mode = SHUTDOWN;
        if (cal_brakemode) brake.setmode(Calibrate);
        else if (brake.motormode == Calibrate) brake.setmode(Halt);
        if (cal_gasmode_request) gas.setmode(Calibrate);
        else gas.setmode(Idle);
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