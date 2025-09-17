#pragma once
class RunModeManager {  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in standby mode with no activity
  private:
    int _preferred_drivemode = Cruise; // from hold mode, should we enter cruise or fly mode?
    int _lowpower_delay_min = 20;      // Time of inactivity after entering standby mode before going to lowpower mode.  900sec = 15min
    int _screensaver_delay_min = 17;   // Time of inactivity after entering standby mode before starting screensaver turns on.  300sec = 5min
    int _joydir, _oldmode = LowPower;
    bool _joy_has_been_centered = false, _we_just_switched_modes = true, _stoppedholdtimer_active = false;
    Timer _gestureFlyTimer{500000};    // Time allowed for joy mode-change gesture motions (Fly mode <==> Cruise mode) (in us)
  public:
    bool display_reset_requested = false;  // set these for the display to poll and take action, since we don't have access to that object, but it has access to us
    RunModeManager() {}
    void setup() {
        ezread.squintf(ezread.highlightcolor, "Runmode state machine init\n");
        set_preferred_drivemode();        // read preferred drivemode from flash
        runmode = Standby;  // our first mode upon boot  // disabling ability to recover to previous runmode after crash:  runmode = watchdog.boot_to_runmode;
    }  // we don't really need to set up anything, unless we need to recover to a specific runmode after crash
    int update() {
        if (runmode != LowPower && runmode != Cal) {
            if (in_basicmode) runmode = Basic;  // basicsw.val() if basicmode switch was on at boot time --> Basic Mode
            else if (!ignition.signal) runmode = Standby;
            else if (tach.stopped()) runmode = Stall;  // otherwise if engine not running --> Stall Mode
        }
        if ((runmode == Hold) && (brake.feedback == _None)) {  // if we have no brake feedback then hold mode must be skipped...
            if (_oldmode == Stall || _oldmode == Standby) runmode = _preferred_drivemode;  // skip hold mode when starting up
            else if (_oldmode == Basic || _oldmode == Cal || _oldmode == LowPower) runmode = Standby;  // just to cover all possibilities
            else runmode = _oldmode;  // don't drop to hold mode from other (driving) modes
        }
        _we_just_switched_modes = (runmode != _oldmode);  // has our runmode been changed?
        if (_we_just_switched_modes) {
            calmode_request = autosaver_request = ReqOff;
            if (starter.motor && runmode != Hold && _oldmode != Stall) starter.request(ReqOff, StartRunmode); // the only mode transition the starter motor may remain running thru is stall -> hold
            gas.setmode(run_motor_mode[runmode][_Throttle]);
            brake.setmode(run_motor_mode[runmode][_BrakeMotor]);
            steer.setmode(run_motor_mode[runmode][_SteerMotor]);
            watchdog.set_codestatus();
            shutting_down = _joy_has_been_centered = car_hasnt_moved = cruise_adjusting = false;  // clean up previous runmode values
            _stoppedholdtimer_active = cal_gasmode = cal_brakemode = cal_gasmode_request = cal_brakemode_request = false;  // clean up previous runmode values
            // ezread.squintf("runmode %s -> %s\n", modecard[_oldmode].c_str(), modecard[runmode].c_str());
        }
        _oldmode = runmode;
        if (runmode != LowPower) {  // common to almost all the modes, so i put it here
            if (ignition.signal) {  // if ignition is on
                if (untested_hotrc_kills_ign && hotrc.radiolost_untested()) ignition.request(ReqOff);
                if (hotrc.sw_event_unfilt(Ch3)) ignition.request(ReqOff);  // any Ch3 event turns it off. if ign is turned off while the car is moving, this leads to panic stop. Note keep this if separate, as it will reset the sw event
            }
            else {  // if ignition is off
                if (hotrc.sw_event_filt(Ch3)) ignition.request(ReqOn);  // a steady filtered Ch3 event turns it on. Note keep this if separate, as it will reset the sw event
            }
        }
        if (runmode == Basic) run_basicMode(); // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        else if (runmode == LowPower) run_lowpowerMode();
        else if (runmode == Standby) run_standbyMode();
        else if (runmode == Stall) run_stallMode();
        else if (runmode == Hold) run_holdMode();
        else if (runmode == Fly) run_flyMode();
        else if (runmode == Cruise) run_cruiseMode();
        else if (runmode == Cal) run_calMode();
        else ezread.squintf(ezread.madcolor, "err: invalid runmode=%d entered\n", runmode);  // Obviously this should never happen
        return runmode;
    }
    int preferred_drivemode() { return _preferred_drivemode; }       // returns the current preferred drivemode
    int* preferred_drivemode_ptr() { return &_preferred_drivemode; } // returns pointer to the current preferred drivemode
    void tog_preferred_drivemode() { set_preferred_drivemode((_preferred_drivemode == Cruise) ? Fly : Cruise); } // toggles the preferred drivemode Fly <-> Cruise
    void set_preferred_drivemode(int new_drivemode=-1) {          // sets the preferred drivemode to the given mode (Fly or Cruise). Run w/o arguments to set to value stored in flash
        int old_drivemode = _preferred_drivemode;                 // used to help us avoid unnecessary flash writes
        if (new_drivemode == -1) {                                // if no new drivemode was given
            int newread = watchdog.flash_read("pref_drivemode");  // get stored flash value if present, otherwise -1
            if (newread == -1) old_drivemode = -1;                // if no flash content found, we will force a flash write
            else new_drivemode = newread;
        }
        if ((new_drivemode == Cruise) || (new_drivemode == Fly)) _preferred_drivemode = new_drivemode;  // if valid then make official
        if (_preferred_drivemode != old_drivemode) watchdog.flash_forcewrite("pref_drivemode", (uint32_t)_preferred_drivemode);  // if value changed or flash read failed, write new value to flash
    }
  private:
    void run_basicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (_we_just_switched_modes) powering_up = false;  // basicmode_request =  to cover unlikely edge case where basic mode switch is enabled during wakeup from lowpower mode
        if (!ignition.signal) {
            if (hotrc.sw_event_filt(Ch4)) runmode = LowPower;  // Note keep this if separate, as it will reset the sw event
        }
        if (!in_basicmode && !tach.stopped()) runmode = speedo.stopped() ? Hold : Fly;  // basicsw.val()  If we turned off the basic mode switch with engine running, change modes. If engine is not running, we'll end up in Stall Mode automatically
    }
    void run_lowpowerMode() {  // turns off syspower and just idles. sleep_request are handled here or in standby mode below
        static Timer pwrchange_timer{300000}; // time allowed to power up/down system devices during wakeup/sleeping. delays entry to standby mode (in us)
        if (_we_just_switched_modes) {
            sleep_request = ReqNA;
            powering_up = false;  // three state variables to track entry/exit phases of lowpower mode
            powering_down = true; // during this time we blackout the screen (should be done in display.h)
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
            if (!hotrc.radiolost()) {
                if (hotrc.sw_event_filt(Ch4)) sleep_request = ReqOff;  // Note keep this if separate, as it will reset the sw event
            }
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
        static Timer stopcar_timer{5000000}; // spend this long trying to stop the car and parking motors before halting actuators 
        static Timer parkmotors_timer{2000000}; // spend this long trying to park the motors before halting them
        static bool stopcar_phase;
        if (_we_just_switched_modes) {              
            shutting_down = !powering_up;   // if waking up from sleep standby is already complete
            stopcar_phase = true;  // !speedo.stopped();
            ignition.request(ReqOff);
            stopcar_timer.reset();
            sleep_request = ReqNA;
            user_inactivity_timer.set(_lowpower_delay_min * 60 * 1000000);
        }
        if (shutting_down) {
            if (stopcar_phase) {
                if (speedo.stopped() || stopcar_timer.expired()) {  // first we need to stop the car and release brakes and gas before shutting down
                    if (stopcar_timer.expired()) ezread.squintf(ezread.sadcolor, "warn: standby mode unable to stop car\n");
                    stopcar_phase = false;  // move on to parkmotor phase
                    parkmotors_timer.reset();
                }
                else if (brake.motormode != AutoStop) brake.setmode(AutoStop);
            }
            else {  // we are in park motor phase
                if (brkpos.parked() || parkmotors_timer.expired()) {  // first we need to stop the car and release brakes and gas before shutting down
                    if (parkmotors_timer.expired()) ezread.squintf(ezread.sadcolor, "warn: standby mode unable to park brake\n");
                    shutting_down = false;  // done shutting down
                    brake.setmode(Halt);
                }
                else if (brake.motormode != ParkMotor) brake.setmode(ParkMotor);
            }
        }
        else {
            if (steer.motormode != Halt) steer.setmode(Halt);
            if (brake.motormode != Halt) brake.setmode(Halt);
            if (hotrc.sw_event_filt(Ch4) || user_inactivity_timer.expired() || sleep_request == ReqTog || sleep_request == ReqOn) runmode = LowPower;
            if (calmode_request) runmode = Cal;  // if fully shut down and cal mode requested, go to cal mode
            if (auto_saver_enabled) if (encoder.button.shortpress()) autosaver_request = ReqOff;
            if (user_inactivity_timer.elapsed() > _screensaver_delay_min * 60 * 1000000) autosaver_request = ReqOn;
        }
        if ((speedo.stopped() || allow_rolling_start) && ignition.signal && !panicstop && !tach.stopped()) runmode = Hold;  // If we started the car, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
        sleep_request = ReqNA;
    }
    void run_stallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        if (!starter.motor && (starter.now_req != ReqOn)) {  // if starter is not running, or in-progress request to start running
            if (brake.motormode != run_motor_mode[runmode][_BrakeMotor]) brake.setmode(run_motor_mode[runmode][_BrakeMotor]); // take back the brake after failed attempt to start
            if (gas.motormode != run_motor_mode[runmode][_Throttle]) gas.setmode(run_motor_mode[runmode][_Throttle]); // take back the gas after failed attempt to start
        }
        if (starter.motor) {  // if starter is running
            if (hotrc.sw_event_unfilt(Ch4)) starter.request(ReqOff, hotrc.last_ch4_source());  // turn off starter if any Ch4 event occurred. Note keep this if separate, as it will reset the sw event
        } 
        else {  // if starter is not running
            if (hotrc.sw_event_filt(Ch4)) starter.request(ReqOn, hotrc.last_ch4_source());  // turn on starter if a stable, filtered Ch4 event occurred. Note keep this if separate, as it will reset the sw event
        }
        if (!tach.stopped()) runmode = Hold;  // If we started the car, enter hold mode once starter is released
    }
    void run_holdMode(bool recovering=false) {  // recovering argument is only used by the [experimental & optional] boot monitor feature to resume previous drive state after a system crash
        static Timer ch4_function_timer{500000};  // this long after starter motor has stopped, ch4 button function becomes toggling between preferred drivemode, rather than stopping the running starter
        if (_we_just_switched_modes) _joy_has_been_centered = recovering;  // Fly mode will be locked until the joystick first is put at or below center (to avoid acting on whatever initial trigger value upon Hold mode entry)
        if (starter.motor) {  // if starter motor is running
            ch4_function_timer.reset();  // constantly reset timer to prevent Ch4 buttonn from changing drivemode until after starter motor is stopped
            if (hotrc.sw_event_unfilt(Ch4)) starter.request(ReqOff, hotrc.last_ch4_source());  // a Ch4 event turns off the starter.  Note keep this if separate, as it will reset the sw event
        }
        if (holdmode_ch4_drivetoggle && ch4_function_timer.expired()) {  // if the starter motor has been off for long enough
            if (hotrc.sw_event_filt(Ch4)) tog_preferred_drivemode();  // a Ch4 event will select the preferred drivemode.  Note keep this if separate, as it will reset the sw event
        }
        if (hotrc.joydir(Vert) != HrcUp) _joy_has_been_centered = true;  // if not pushing up, set this flag. now pushing up will go to fly mode
        else if (_joy_has_been_centered && !starter.motor) {  // if pushing up (trying to drive), and starter not running, and trigger was previously centered (prerequisites to drive)
            bool radio_problem = false;
            if (!sim.simulating(sens::joy)) {  // when using the hotrc remote there are a few checks we gotta make for safety
                if (hotrc.radiolost_untested() && require_hotrc_powercycle) radio_problem = true;
                if (hotrc.radiolost()) radio_problem = true;
            }
            if (!radio_problem) runmode = _preferred_drivemode;  // Enter Fly or Cruise Mode upon joystick movement from center to above center  // Possibly add "&& stopped()" to above check?
            else ezread.squintf(ezread.sadcolor, "warn: need working & tested radio to fly\n");
        }
    }
    void run_flyMode() {
        if (_we_just_switched_modes) car_hasnt_moved = speedo.stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
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
        if (_we_just_switched_modes) {  // upon first entering cruise mode, initialize things
            car_hasnt_moved = speedo.stopped();  // note whether car is moving going into the mode, this turns true once it has initially got moving
            if (!cruise_brake) brake.setmode(Release);  // override the mode set by run_motor_mode[][] array which assumes cruise_brake == true
            _gestureFlyTimer.reset();  // initialize brake-trigger timer
        }
        if (car_hasnt_moved) {
            if (hotrc.joydir(Vert) != HrcUp) runmode = Hold;            // must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.stopped()) car_hasnt_moved = false;  // once car moves, we're allowed to release the trigger without falling out of the mode
        }
        else if (speedo.stopped()) {
            if (hotrc.joydir() == HrcDn) runmode = Hold;  // go to Hold Mode if we have slowed to a stop after previously moving  // && hotrc.pc[Vert][Filt] <= hotrc.pc[Vert][Cent]
            else if (hotrc.joydir() != HrcUp) {  // unless attempting to increase cruise speed, when stopped drop to hold after a short timeout
                if (!_stoppedholdtimer_active) {
                    stoppedholdtimer.reset();
                    _stoppedholdtimer_active = true;
                }
                else if (stoppedholdtimer.expired()) runmode = Hold;
            }
        }  // if (hotrc.joydir(Vert) == HrcDn && !cruise_speed_lowerable) runmode = Fly;
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) runmode = Hold;        // radio must be good to fly, this should already be handled elsewhere but another check can't hurt
        if (hotrc.sw_event_filt(Ch4)) runmode = Fly;                  // go to fly mode if hotrc ch4 button pushed
        // // if joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (!cruise_brake) {  // no need for this feature if cruise includes braking
            if (hotrc.pc[Vert][Filt] > hotrc.pc[Vert][OpMin] + flycruise_vert_margin_pc) _gestureFlyTimer.reset();  // keep resetting timer if joystick not at bottom
            else if (_gestureFlyTimer.expired()) runmode = Fly;  // new gesture to drop to fly mode is hold the brake all the way down for more than X ms
        }
        // removing requirement for car to be moving to stay in cruise mode 2024bm
        // if (speedo.stopped()) runmode = (hotrc.joydir(Vert) == HrcUp) ? Fly : Hold;  // in case we slam into camp Q woofer stack, get out of cruise mode.
    }
    void run_calMode() {  // calibration mode is purposely difficult to get into, because it allows control of motors without constraints for purposes of calibration - don't use it unless you know how.
        if (_we_just_switched_modes) calmode_request = cal_gasmode_request = cal_brakemode_request = false;
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