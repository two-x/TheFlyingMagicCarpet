#pragma once
class RunModeManager {  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in standby mode with no activity
  private:
    int _joydir;
    int64_t lowpower_delay = 9000000;  // Time of inactivity after entering standby mode before going to lowpower mode
    int64_t screensaver_delay = 3000000;  // Time of inactivity after entering standby mode before starting screensaver turns on
    Timer gestureFlyTimer{1250000};  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
    Timer pwrup_timer{3000000};  // Timeout when parking motors if they don't park for whatever reason (in us)
    Timer standby_timer{5000000};
    int oldmode = LOWPOWER;
    bool still_interactive = true;
    uint32_t initial_inactivity;
  public:
    int mode = STANDBY;
    bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
    bool joy_centered = false;
    bool autosaver_requested = false, display_reset_requested = false;  // set these for the display to poll and take action, since we don't have access to that object, but it has access to us
    RunModeManager() {}
    void setup() { mode = watchdog.boot_to_runmode; }  // we don't really need to set up anything, unless we need to recover to a specific runmode after crash
    int mode_logic() {
        if (mode != LOWPOWER && mode != CAL) {
            if (basicsw.val) mode = BASIC;  // if basicmode switch on --> Basic Mode
            else if (!ignition.signal) mode = STANDBY;
            else if (tach.engine_stopped()) mode = STALL;  // otherwise if engine not running --> Stall Mode
        }
        if (mode == HOLD && (brake.feedback == _None)) mode = FLY;  // we can not autohold the brake when running brake open loop, so go directly to fly mode
        we_just_switched_modes = (mode != oldmode);  // currentMode should not be changed after this point in loop
        if (we_just_switched_modes) cleanup_state_variables();
        oldmode = mode;        
        // common to almost all the modes, so i put it here
        if (mode != LOWPOWER) {
            autosaver_requested = false;
            if (hotrc.sw_event(CH3)) ignition.request(REQ_TOG);  // Turn on/off the vehicle ignition. if ign is turned off while the car is moving, this leads to panic stop
        }
        if (mode == BASIC) run_basicMode(); // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        else if (mode == LOWPOWER) run_lowpowerMode();
        else if (mode == STANDBY) run_standbyMode();
        else if (mode == STALL) run_stallMode();
        else if (mode == HOLD) run_holdMode();
        else if (mode == FLY) run_flyMode();
        else if (mode == CRUISE) run_cruiseMode();
        else if (mode == CAL) run_calMode();
        else Serial.println(F("Error: Invalid runmode entered"));  // Obviously this should never happen
        return mode;
    }
  private:
    void cleanup_state_variables() {
        if (oldmode == BASIC);
        else if (oldmode == LOWPOWER);
        else if (oldmode == STANDBY) standby_incomplete = false;
        else if (oldmode == STALL);
        else if (oldmode == HOLD) joy_centered = false;  // starter.request(REQ_OFF);  // Stop any in-progress startings
        else if (oldmode == FLY) car_hasnt_moved = false;
        else if (oldmode == CRUISE) cruise_adjusting = false;
        else if (oldmode == CAL) cal_gasmode = cal_brakemode = cal_gasmode_request = cal_brakemode_request = false;
    }
    void run_basicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) {
            gas.setmode(ParkMotor);  // Upon entering basic mode, the brake and gas actuators need to be parked out of the way so the pedals can be used.
            brake.setmode(ParkMotor);
            steer.setmode(OpenLoop);
            powering_up = basicmode_request = false;  // to cover unlikely edge case where basic mode switch is enabled during wakeup from lowpower mode
            watchdog.set_codestatus(Parked);
        }
        if (hotrc.sw_event(CH4) && !ignition.signal) mode = LOWPOWER;
        if (!basicsw.val && !tach.engine_stopped()) mode = speedo.car_stopped() ? HOLD : FLY;  // If we turned off the basic mode switch with engine running, change modes. If engine is not running, we'll end up in Stall Mode automatically
        if (basicmode_request) mode = STANDBY;  // if fully shut down and cal mode requested, go to cal mode
    }
    void run_lowpowerMode() {  // turns off syspower and just idles. sleep_request are handled here or in standby mode below
        if (we_just_switched_modes) {
            sleep_request = REQ_NA;
            powering_up = false;
            brake.setmode(Halt);
            steer.setmode(Halt);
            set_syspower(LOW);     // Power down devices to save battery
        }
        if (hotrc.sw_event(CH4) || sleep_request == REQ_TOG || sleep_request == REQ_OFF) {  // start powering up
            set_syspower(HIGH);    // switch on control system devices
            pwrup_timer.reset();   // stay in lowpower mode for a delay to allow devices to power up
            powering_up = true;
            autosaver_requested = false;
        }
        if (powering_up && pwrup_timer.expired()) mode = (basicsw.val) ? BASIC : STANDBY;  // finish powering up . display->all_dirty();  // tells display to redraw everything. display must set back to false
        sleep_request = REQ_NA;
    }
    void run_standbyMode() { // In standby mode we stop the car if it's moving, park the motors, go idle for a while and eventually sleep.
        if (we_just_switched_modes) {              
            standby_incomplete = !powering_up;   // if waking up from sleep standby is already complete
            powering_up = calmode_request = basicmode_request = autosaver_requested = false;
            gas.setmode(ParkMotor);                 // carburetor parked 
            brake.setmode(AutoStop);                // if car is moving begin autostopping
            standby_timer.reset();
            sleep_request = REQ_NA;
        }
        else if (standby_incomplete) {  // first we need to stop the car and release brakes and gas before shutting down
            if (standby_timer.expired()) standby_incomplete = false;
            if (brake.motormode != AutoStop) {  // brake autostop mode will have dropped to Halt mode once complete, check for that
                if (brake.parked()) standby_incomplete = false;
                else brake.setmode(ParkMotor);
            }
        }
        else {  // if standby is complete
            steer.setmode(Halt);  // disable steering, in case it was left on while we were panic stopping
            brake.setmode(Halt);
            watchdog.set_codestatus(Parked);  // write to flash we are in an appropriate place to lose power, so we can detect crashes on boot
            if (hotrc.sw_event(CH4) || (user_inactivity_timer.elapsed() > lowpower_delay) || sleep_request == REQ_TOG || sleep_request == REQ_ON) mode = LOWPOWER;
            if (calmode_request) mode = CAL;  // if fully shut down and cal mode requested, go to cal mode
            if (basicmode_request) mode = BASIC;  // if fully shut down and basic mode requested, go to basic mode
            if (user_inactivity_timer.elapsed() > screensaver_delay) autosaver_requested = true;
            // else if (encoder.button.shortpress()) autosaver_requested = false;
        }
        if ((speedo.car_stopped() || allow_rolling_start) && ignition.signal && !panicstop && !tach.engine_stopped()) mode = HOLD;  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
        sleep_request = REQ_NA;
    }
    void run_stallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (we_just_switched_modes) {
            gas.setmode(OpenLoop);     // throttle always runs open loop in stall mode, b/c there's no rpm for the pid to measure anyway
            brake.setmode(ActivePID);
            steer.setmode(OpenLoop);
        }
        if (hotrc.sw_event(CH4)) starter.request(REQ_TOG);  // Serial.printf("stall: req=%d\n", REQ_TOG);
        if (starter.motor || !tach.engine_stopped()) mode = HOLD;  // If we started the car, enter hold mode once starter is released
        // Serial.printf("%d/%d ", starter_request, starter);
    }
    void run_holdMode(bool recovering=false) {
        if (we_just_switched_modes) {
            joy_centered = recovering;  // Fly mode will be locked until the joystick first is put at or below center
            watchdog.set_codestatus(Stopped);  // write to flash we are NOT in an appropriate place to lose power, so we can detect crashes on boot
            gas.setmode(throttle_ctrl_mode);
            brake.setmode(AutoHold);
            steer.setmode(OpenLoop);
        }
        if (hotrc.sw_event(CH4)) starter.request(REQ_OFF);
        if (hotrc.joydir(VERT) != JOY_UP) joy_centered = true;  // mark joystick at or below center, now pushing up will go to fly mode
        else if (joy_centered && !starter.motor && !hotrc.radiolost()) mode = FLY;  // Enter Fly Mode upon joystick movement from center to above center  // Possibly add "&& car_stopped()" to above check?
    }
    void run_flyMode() {
        if (we_just_switched_modes) {
            car_hasnt_moved = speedo.car_stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
            gas.setmode(throttle_ctrl_mode);
            brake.setmode(ActivePID);
            steer.setmode(OpenLoop);
        }
        if (car_hasnt_moved) {
            if (hotrc.joydir(VERT) != JOY_UP) mode = HOLD;            // must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.car_stopped()) car_hasnt_moved = false;  // once car moves, we're allowed to release the trigger without falling out of fly mode
        }
        else {
            watchdog.set_codestatus(Driving);  // write to flash we are NOT in an appropriate place to lose power, so we can detect crashes on boot
            if (speedo.car_stopped() && hotrc.joydir() != JOY_UP) mode = HOLD;  // go to Hold Mode if we have come to a stop after moving  // && hotrc.pc[VERT][FILT] <= hotrc.pc[VERT][DBBOT]
        }
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) mode = HOLD;        // radio must be good to fly, this should already be handled elsewhere but another check can't hurt
        if (hotrc.sw_event(CH4)) mode = CRUISE;                                     // enter fly mode by pressing hrc ch4 button
    }
    void run_cruiseMode() {
        if (we_just_switched_modes) {  // upon first entering cruise mode, initialize things
            gas.setmode(Cruise);
            brake.setmode(Release);
            gestureFlyTimer.reset();  // initialize brake-trigger timer
        }
        if (hotrc.joydir(VERT) == JOY_DN && !cruise_speed_lowerable) mode = FLY;
        if (hotrc.sw_event(CH4)) mode = FLY;                  // go to fly mode if hotrc ch4 button pushed
        // if joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (hotrc.pc[VERT][FILT] > hotrc.pc[VERT][OPMIN] + flycruise_vert_margin_pc) gestureFlyTimer.reset();  // keep resetting timer if joystick not at bottom
        else if (gestureFlyTimer.expired()) mode = FLY;  // new gesture to drop to fly mode is hold the brake all the way down for more than X ms
        if (speedo.car_stopped()) mode = (hotrc.joydir(VERT) == JOY_UP) ? FLY : HOLD;  // in case we slam into camp Q woofer stack, get out of cruise mode.
    }
    void run_calMode() {  // calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration - don't use it unless you know how.
        if (we_just_switched_modes) {
            calmode_request = cal_gasmode_request = cal_brakemode_request = false;
            gas.setmode(Idle);
            brake.setmode(Halt);
            steer.setmode(Halt);
        }
        else if (calmode_request) mode = STANDBY;
        if (cal_gasmode_request && gas.motormode != Calibrate) gas.setmode(Calibrate);  // Serial.printf("req:%d, cal:%d\n", cal_brakemode_request, cal_brakemode);
        else if (!cal_gasmode_request && gas.motormode == Calibrate) gas.setmode(Idle);
        if (cal_brakemode_request && brake.motormode != Calibrate) brake.setmode(Calibrate);  // Serial.printf("req:%d, cal:%d\n", cal_brakemode_request, cal_brakemode);
        else if (!cal_brakemode_request && brake.motormode == Calibrate) brake.setmode(Halt);
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
// ** Lowpower Mode **
// - Required: Request with hotrc ch4 button when shut down
// Turns off power to the system. This includes all sensors/actuators and the screen, but not the hotrc receiver.
// When requested to power up (same ch4 hotrc button), it re-powers everything and goes to standby mode.
//
// ** Standby Mode **
// - Required: BasicMode switch Off & Ignition Off
// This mode is active at boot, or whenever the ignition is off or when panic stopping. If the car is
// moving, then like hold mode, standby mode will try to stop the car. Once stopped, then like basic mode,
// it will park the motors out of the way and all systems stop. After a timeout it will go to lowpower mode.
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
//
// ** Cal Mode **
// This mode allows direct control of some actuators without respecting limits of motion, for purpose of
// calibrating those very limits. It can be entered from standby mode with simulator on by long-pressing the CAL
// button. Be careful with it.