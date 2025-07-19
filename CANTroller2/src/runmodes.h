#pragma once
class RunModeManager {  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in standby mode with no activity
  private:
    int lowpower_delay_sec = 1500;  // Time of inactivity after entering standby mode before going to lowpower mode.  900sec = 15min
    int screensaver_delay_sec = 600;  // Time of inactivity after entering standby mode before starting screensaver turns on.  300sec = 5min
    Timer gestureFlyTimer{500000};  // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
    Timer pwrchange_timer{500000};  // Timeout to allow powerup of system devices during wakeup. delays entry to standby mode (in us)
    Timer standby_timer{5000000};
    Timer stall_timer;
    int _joydir, oldmode = LowPower;
  public:
    bool joy_centered = false, we_just_switched_modes = true, stoppedholdtimer_active = false;  // For mode logic to set things up upon first entry into mode
    bool display_reset_requested = false;  // set these for the display to poll and take action, since we don't have access to that object, but it has access to us
    RunModeManager() {}
    void setup() {
        ezread.squintf("Runmode state machine init\n");
        runmode = Standby;  // our first mode upon boot  // disabling ability to recover to previous runmode after crash:  runmode = watchdog.boot_to_runmode;
    }  // we don't really need to set up anything, unless we need to recover to a specific runmode after crash
    int mode_logic() {
        if (runmode != LowPower && runmode != Cal) {
            if (in_basicmode) runmode = Basic;  // basicsw.val() if basicmode switch was on at boot time --> Basic Mode
            else if (!ignition.signal) runmode = Standby;
            else if (tach.stopped()) runmode = Stall;  // otherwise if engine not running --> Stall Mode
        }
        if ((runmode == Hold) && (brake.feedback == _None)) {  // if we have no brake feedback then hold mode must be skipped...
            if (oldmode == Stall || oldmode == Standby) runmode = default_drive_mode;  // skip hold mode when starting up
            else if (oldmode == Basic || oldmode == Cal || oldmode == LowPower) runmode = Standby;  // just to cover all possibilities
            else runmode = oldmode;  // don't drop to hold mode from other (driving) modes
        }
        we_just_switched_modes = (runmode != oldmode);  // currentMode should not be changed after this point in loop
        if (we_just_switched_modes) {
            if (runmode != Standby) autosaver_request = ReqOff;
            if (starter.motor && runmode != Hold && oldmode != Stall) starter.request(ReqOff, StartRunmode); // the only mode transition the starter motor may remain running thru is stall -> hold
            gas.setmode(run_motor_mode[runmode][_Throttle]);
            brake.setmode(run_motor_mode[runmode][_BrakeMotor]);
            steer.setmode(run_motor_mode[runmode][_SteerMotor]);
            watchdog.set_codestatus();
            cleanup_state_variables();
        }
        oldmode = runmode;
         // common to almost all the modes, so i put it here
        if (runmode != LowPower) {
            if (ignition.signal && hotrc.sw_event_unfilt(Ch3)) ignition.request(ReqOff);  // turn ignition off if any Ch3 event occurred. if ign is turned off while the car is moving, this leads to panic stop
            else if (!ignition.signal && hotrc.sw_event_filt(Ch3)) ignition.request(ReqOn);  // turn ignition on if a steady filtered Ch3 event occurred
        }
        if (runmode == Basic) run_basicMode(); // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        else if (runmode == LowPower) run_lowpowerMode();
        else if (runmode == Standby) run_standbyMode();
        else if (runmode == Stall) run_stallMode();
        else if (runmode == Hold) run_holdMode();
        else if (runmode == Fly) run_flyMode();
        else if (runmode == Cruise) run_cruiseMode();
        else if (runmode == Cal) run_calMode();
        else ezread.squintf(RED, "err: invalid runmode=%d entered\n", runmode);  // Obviously this should never happen
        return runmode;
    }
  private:
    void cleanup_state_variables() {
        if (oldmode == Basic);
        else if (oldmode == LowPower);
        else if (oldmode == Standby) shutting_down = false;
        else if (oldmode == Stall);
        else if (oldmode == Hold) joy_centered = false;  // starter.request(ReqOff);  // Stop any in-progress startings
        else if (oldmode == Fly) car_hasnt_moved = false;
        else if (oldmode == Cruise) cruise_adjusting = car_hasnt_moved = stoppedholdtimer_active = false;
        else if (oldmode == Cal) cal_gasmode = cal_brakemode = cal_gasmode_request = cal_brakemode_request = false;
    }
    void run_basicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (we_just_switched_modes) powering_up = false;  // basicmode_request =  to cover unlikely edge case where basic mode switch is enabled during wakeup from lowpower mode
        if (hotrc.sw_event_filt(Ch4) && !ignition.signal) runmode = LowPower;
        if (!in_basicmode && !tach.stopped()) runmode = speedo.stopped() ? Hold : Fly;  // basicsw.val()  If we turned off the basic mode switch with engine running, change modes. If engine is not running, we'll end up in Stall Mode automatically
    }
    void run_lowpowerMode() {  // turns off syspower and just idles. sleep_request are handled here or in standby mode below
        if (we_just_switched_modes) {
            sleep_request = ReqNA;
            powering_up = false;  // three state variables to track entry/exit phases of lowpower mode
            powering_down = true; // during this time we blackout the screen (should be done in display.h)
            autosaver_request = ReqOff;
            pwrchange_timer.reset();  // give some time for screen to blackout
        }
        else if (powering_down && pwrchange_timer.expired()) {  // blackout time is over, now go to sleep
            syspower.set(LOW);  // Power down devices to save battery
            powering_down = false;
        }
        else if (powering_up && pwrchange_timer.expired()) {  // by now sensors etc. have got powered up, so switch runmode
            powering_up = false;
            runmode = (in_basicmode) ? Basic : Standby;  // basicsw.val()  finish powering up . display->all_dirty();  // tells display to redraw everything. display must set back to false
        }
        else {
            if (encoder.button.shortpress()) sleep_request = ReqOff;
            if (!hotrc.radiolost() && hotrc.sw_event_filt(Ch4)) sleep_request = ReqOff;
            if (sleep_request == ReqTog || sleep_request == ReqOff) {  // start powering up
                syspower.set(HIGH);        // switch on control system devices
                pwrchange_timer.reset();   // stay in lowpower mode for a delay to allow devices to power up
                powering_up = true;
                autosaver_request = ReqOff;
            }
        }
        sleep_request = ReqNA;
    }
    void run_standbyMode() { // In standby mode we stop the car if it's moving, park the motors, go idle for a while and eventually sleep.
        if (we_just_switched_modes) {              
            shutting_down = !powering_up;   // if waking up from sleep standby is already complete
            powering_up = false;
            ignition.request(ReqOff);
            calmode_request = autosaver_request = ReqOff;  // = basicmode_request 
            standby_timer.reset();
            sleep_request = ReqNA;
            user_inactivity_timer.set(lowpower_delay_sec * 1000000);
        }
        else if (shutting_down) {  // first we need to stop the car and release brakes and gas before shutting down
            if (standby_timer.expired()) shutting_down = false;
            if (brake.motormode != AutoStop) {  // brake autostop mode will have dropped to Halt mode once complete, check for that
                if (brake.parked()) shutting_down = false;
                else brake.setmode(ParkMotor);
            }
        }
        else {  // if standby is complete
            steer.setmode(Halt);  // disable steering, in case it was left on while we were panic stopping
            brake.setmode(Halt);
            if (hotrc.sw_event_filt(Ch4) || user_inactivity_timer.expired() || sleep_request == ReqTog || sleep_request == ReqOn) runmode = LowPower;
            if (calmode_request) runmode = Cal;  // if fully shut down and cal mode requested, go to cal mode
            if (auto_saver_enabled) if (encoder.button.shortpress()) autosaver_request = ReqOff;
            if (user_inactivity_timer.elapsed() > screensaver_delay_sec * 1000000) autosaver_request = ReqOn;
        }
        if ((speedo.stopped() || allow_rolling_start) && ignition.signal && !panicstop && !tach.stopped()) runmode = Hold;  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
        sleep_request = ReqNA;
    }
    void run_stallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (!starter.motor && (starter.now_req != ReqOn)) {  // if starter is not running, or in-progress request to start running
            if (brake.motormode != run_motor_mode[runmode][_BrakeMotor]) brake.setmode(run_motor_mode[runmode][_BrakeMotor]); // take back the brake after failed attempt to start
            if (gas.motormode != run_motor_mode[runmode][_Throttle]) gas.setmode(run_motor_mode[runmode][_Throttle]); // take back the gas after failed attempt to start
        }
        // if (stall_mode_timeout) {}      // should stall mode time out after a while, to mitigate potential safety issues w/ ghost starter bug
        if (starter.motor && hotrc.sw_event_unfilt(Ch4)) starter.request(ReqOff, hotrc.last_ch4_source());  // turn off starter if any Ch4 event occurred
        else if (!starter.motor && hotrc.sw_event_filt(Ch4)) starter.request(ReqOn, hotrc.last_ch4_source());  // turn on starter if a stable, filtered Ch4 event occurred
        // if (starter.motor || !tach.stopped()) runmode = Hold;  // If we started the car, enter hold mode once starter is released
        if (!tach.stopped()) runmode = Hold;  // If we started the car, enter hold mode once starter is released
    }
    void run_holdMode(bool recovering=false) {
        if (we_just_switched_modes) joy_centered = recovering;  // Fly mode will be locked until the joystick first is put at or below center
            // if (!starter.motor || (gas.motormode != Starting)) gas.setmode(AutoPID);  // change gas mode unless starter has overridden it
        if (starter.motor && hotrc.sw_event_unfilt(Ch4)) starter.request(ReqOff, hotrc.last_ch4_source());  // turn off starter if any Ch4 event occurred
        if (hotrc.joydir(Vert) != HrcUp) joy_centered = true;  // mark joystick at or below center, now pushing up will go to fly mode
        else {  // else user is pushing up and trying to start driving
            bool allowed_to_fly = joy_centered && !starter.motor;
            bool radio_problem = false;
            if (!sim.simulating(sens::joy)) {  // when using the hotrc remote there are a few checks we gotta make for safety
                if (hotrc.radiolost_untested() && require_hotrc_powercycle) radio_problem = true;
                if (hotrc.radiolost()) radio_problem = true;
            }
            if (allowed_to_fly) {
                if (!radio_problem) runmode = default_drive_mode;  // Enter Fly or Cruise Mode upon joystick movement from center to above center  // Possibly add "&& stopped()" to above check?
                else ezread.squintf(ORG, "warn: need working & tested radio to fly\n");
            }
        }
    }
    void run_flyMode() {
        if (we_just_switched_modes) car_hasnt_moved = speedo.stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
        if (car_hasnt_moved) {
            if (hotrc.joydir(Vert) != HrcUp) runmode = Hold;      // must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.stopped()) car_hasnt_moved = false;  // once car moves, we're allowed to release the trigger without falling out of fly mode
        }
        else if (speedo.stopped() && hotrc.joydir() != HrcUp) runmode = Hold;  // go to Hold Mode if we have come to a stop after moving  // && hotrc.pc[Vert][Filt] <= hotrc.pc[Vert][Cent]
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) runmode = Hold;   // radio must be good to fly, this should already be handled elsewhere but another check can't hurt
        if (hotrc.sw_event_filt(Ch4)) runmode = Cruise;                        // enter cruise mode by pressing hrc ch4 button
    }
    void run_cruiseMode() {
        static Timer stoppedholdtimer{4000000};  // how long after coming to a stop should we drop to hold mode?
        if (we_just_switched_modes) {  // upon first entering cruise mode, initialize things
            car_hasnt_moved = speedo.stopped();  // note whether car is moving going into the mode, this turns true once it has initially got moving
            if (!cruise_brake) brake.setmode(Release);  // override the mode set by run_motor_mode[][] array which assumes cruise_brake == true
            gestureFlyTimer.reset();  // initialize brake-trigger timer
        }
        if (car_hasnt_moved) {
            if (hotrc.joydir(Vert) != HrcUp) runmode = Hold;            // must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.stopped()) car_hasnt_moved = false;  // once car moves, we're allowed to release the trigger without falling out of the mode
        }
        else if (speedo.stopped()) {
            if (hotrc.joydir() == HrcDn) runmode = Hold;  // go to Hold Mode if we have slowed to a stop after previously moving  // && hotrc.pc[Vert][Filt] <= hotrc.pc[Vert][Cent]
            else if (hotrc.joydir() != HrcUp) {  // unless attempting to increase cruise speed, when stopped drop to hold after a short timeout
                if (!stoppedholdtimer_active) {
                    stoppedholdtimer.reset();
                    stoppedholdtimer_active = true;
                }
                else if (stoppedholdtimer.expired()) runmode = Hold;
            }
        }  // if (hotrc.joydir(Vert) == HrcDn && !cruise_speed_lowerable) runmode = Fly;
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) runmode = Hold;        // radio must be good to fly, this should already be handled elsewhere but another check can't hurt
        if (hotrc.sw_event_filt(Ch4)) runmode = Fly;                  // go to fly mode if hotrc ch4 button pushed
        
        // // if joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (!cruise_brake) {  // no need for this feature if cruise includes braking
            if (hotrc.pc[Vert][Filt] > hotrc.pc[Vert][OpMin] + flycruise_vert_margin_pc) gestureFlyTimer.reset();  // keep resetting timer if joystick not at bottom
            else if (gestureFlyTimer.expired()) runmode = Fly;  // new gesture to drop to fly mode is hold the brake all the way down for more than X ms
        }
        // removing requirement for car to be moving to stay in cruise mode 2024bm
        // if (speedo.stopped()) runmode = (hotrc.joydir(Vert) == HrcUp) ? Fly : Hold;  // in case we slam into camp Q woofer stack, get out of cruise mode.
    }
    void run_calMode() {  // calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration - don't use it unless you know how.
        if (we_just_switched_modes) calmode_request = cal_gasmode_request = cal_brakemode_request = false;
        else if (calmode_request) runmode = Standby;
        if (cal_gasmode_request && gas.motormode != Calibrate) gas.setmode(Calibrate);  // ezread.squintf("req:%d, cal:%d\n", cal_brakemode_request, cal_brakemode);
        else if (!cal_gasmode_request && gas.motormode == Calibrate) gas.setmode(Idle);
        if (cal_brakemode_request && brake.motormode != Calibrate) brake.setmode(Calibrate);  // ezread.squintf("req:%d, cal:%d\n", cal_brakemode_request, cal_brakemode);
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
// - Required: Engine running & HrcVert<=Center & BasicMode switch Off & Ignition On
// This mode ensures the car is stopped and stays stopped until you pull the trigger to give it gas, at which
// point it goes to fly mode. This mode is entered from fly mode if the car comes to a stop, or from Stall Mode if
// the engine starts turning. The starter can possibly be on through that transition, and thereafter it may be 
// turned off but not on from hold mode.
//
// ** Fly Mode **
// - Required: HrcVert>Center & Engine running & BasicMode Off & Ign On
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