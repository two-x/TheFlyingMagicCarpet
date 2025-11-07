#pragma once
class RunModeManager {  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in standby mode with no activity
  private:
    bool _verbose = true;              // causes console prints for debug purposes
    int _preferred_drivemode = Cruise; // from hold mode, should we enter cruise or fly mode?
    int _lowpower_delay_min = 20;      // Time of inactivity after entering standby mode before going to lowpower mode.  900sec = 15min
    int _screensaver_delay_min = 17;   // Time of inactivity after entering standby mode before starting screensaver turns on.  300sec = 5min
    int _joydir, _oldmode = LowPower;
    bool _joy_has_been_centered = false, _we_just_switched_modes = true, _stopped_hold_timer_active = false, _stall_ch4start_timed_out = false;
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
            shutting_down = _joy_has_been_centered = car_hasnt_moved = cruise_adjusting = _stall_ch4start_timed_out = false;  // clean up previous runmode values
            _stopped_hold_timer_active = cal_gasmode = cal_brakemode = cal_gasmode_request = cal_brakemode_request = false;  // clean up previous runmode values
            if (_verbose) ezread.squintf("run: mode change %s->%s\n", modecard[_oldmode].c_str(), modecard[runmode].c_str());
        }
        _oldmode = runmode;

        if (runmode == Basic) run_basicMode(); // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        else if (runmode == LowPower) run_lowpowerMode();
        else if (runmode == Standby) run_standbyMode();
        else if (runmode == Stall) run_stallMode();
        else if (runmode == Hold) run_holdMode();
        else if (runmode == Fly) run_flyMode();
        else if (runmode == Cruise) run_cruiseMode();
        else if (runmode == Cal) run_calMode();
        else ezread.squintf(ezread.madcolor, "err: invalid runmode=%d entered\n", runmode);  // Obviously this should never happen
        
        if (ignition.signal && runmode != Cal) {  // guarantee execution of ignition-kill features, in all modes except Cal
            bool kill_ign = false;
            if (!(sim.simulating(sens::joy) && sim.enabled())) {  // when using the radio to control joy vert (rather than simulator)
                if (hotrc.radiolost_untested() && require_radiolost_test) kill_ign = true;
                if (hotrc.radiolost()) kill_ign = true;
                if (kill_ign) ezread.squintf(ezread.madcolor, "run: ignition kill due to radio error\n");
            }
            if (hotrc.sw_event_unfilt(Ch3)) kill_ign = true; // any Ch3 event turns ign off
            if (kill_ign) {
                if (!speedo.stopped()) {
                    if (_verbose) ezread.squintf("panic state requested by run.update()\n");
                    ignition.panic_request(ReqOn); // if moving, panic will kill ignition and apply brakes
                }
                else ignition.request(ReqOff);                        // otherwise just kill the ignition
            }
        }
        return runmode;
    }
    int preferred_drivemode() { return _preferred_drivemode; }       // returns the current preferred drivemode
    int* preferred_drivemode_ptr() { return &_preferred_drivemode; } // returns pointer to the current preferred drivemode
    void tog_preferred_drivemode() { set_preferred_drivemode((_preferred_drivemode == Cruise) ? Fly : Cruise); } // toggles the preferred drivemode Fly <-> Cruise
    void set_preferred_drivemode(int new_drivemode=-1) { // set the pref drivemode to given mode, otherwise to mode stored in flash
        int old_drivemode = _preferred_drivemode;                 // used to help us avoid unnecessary flash writes
        if (new_drivemode == -1) {                                // if no new drivemode was given
            int newread = watchdog.flash_read("pref_drivemode");  // get stored flash value if present, otherwise -1
            if (newread == -1) old_drivemode = -1;                // if no flash content found, we will force a flash write
            else new_drivemode = newread;
        }
        if ((new_drivemode == Cruise) || (new_drivemode == Fly)) _preferred_drivemode = new_drivemode;  // if valid then make official
        if (_preferred_drivemode != old_drivemode) watchdog.flash_forcewrite("pref_drivemode", (uint32_t)_preferred_drivemode);  // if value changed or flash read failed, write new value to flash
    }
    void tog_current_drivemode() {
        tog_preferred_drivemode();  // this ensures the drivemode change will stick beyond future runmode changes
        runmode = _preferred_drivemode;
    }
  private:
    void run_basicMode() { // Basic mode is for when we want to operate the pedals manually. All PIDs stop, only steering still works.
        if (_we_just_switched_modes) powering_up = false;  // to cover unlikely edge case where basic mode switch is enabled during wakeup from lowpower mode
        if (!ignition.signal) {
            if (hotrc.sw_event_filt(Ch4)) runmode = LowPower;  // Note keep this if separate, as it will reset the sw event
        }
        if (!in_basicmode) runmode = Standby;  // if we turned off the basic mode switch, go to standby mode
    }
    void run_lowpowerMode() {  // turns off syspower and just waits. sleep_request are handled here or in standby mode below
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
            if (hotrc.sw_event_filt(Ch4)) sleep_request = ReqOff;  // Note keep this if separate, as it will reset the sw event
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
        static Timer parkmotors_timer{2500000}; // spend this long trying to park the motors before halting them
        static bool stopcar_phase;
        if (_we_just_switched_modes) {              
            shutting_down = !powering_up;   // if waking up from sleep standby is already complete
            stopcar_phase = true;  // !speedo.stopped();
            ignition.request(ReqOff);  // ezread.squintf("temp: ignition OFF in standby\n");
            stopcar_timer.reset();
            sleep_request = ReqNA;
            user_inactivity_timer.set(_lowpower_delay_min * 60 * 1000000);
        }
        if (shutting_down) {
            if (stopcar_phase) {
                if (speedo.stopped() || stopcar_timer.expired()) {  // first we need to stop the car and release brakes and gas before shutting down
                    if (stopcar_timer.expired()) ezread.squintf(ezread.sadcolor, "warn: standby mode unable to stop car\n");
                    stopcar_phase = false;  // move on to parkmotor phase
                    brake.setmode(ParkMotor);
                    parkmotors_timer.reset();
                }
                else if (brake.motormode != AutoStop) brake.setmode(AutoStop);
            }
            else {  // we are in park motor phase
                if (brake.parked() || parkmotors_timer.expired()) {  // first we need to stop the car and release brakes and gas before shutting down
                    if (!brake.parked()) ezread.squintf(ezread.sadcolor, "warn: standby mode unable to park brake\n");
                    shutting_down = false;  // done shutting down
                    brake.setmode(Halt);
                }
                // else if (brake.motormode != ParkMotor) brake.setmode(ParkMotor);
            }
        }
        else {
            if (steer.motormode != Halt) steer.setmode(Halt);
            if (brake.motormode != Halt) brake.setmode(Halt);
            if (hotrc.sw_event_filt(Ch4)) sleep_request = ReqOn;  // ch4 press puts system to sleep.  ensure switch event check is in its own if statement
            if (hotrc.sw_event_filt(Ch3)) { 
                if (!hotrc.radiolost_untested() && !hotrc.radiolost()) ignition.request(ReqOn); // turn on ignition, will land us in stall mode
                else ezread.squintf(ezread.sadcolor, "warn: ignition requires tested radio\n");
            }
            if (calmode_request) runmode = Cal;  // if fully shut down and cal mode requested, go to cal mode
            if (auto_saver_enabled) if (encoder.button.shortpress()) autosaver_request = ReqOff;
            if (user_inactivity_timer.elapsed() > _screensaver_delay_min * 60 * 1000000) autosaver_request = ReqOn;
            if (user_inactivity_timer.expired() || sleep_request == ReqTog || sleep_request == ReqOn) runmode = LowPower;
        }
        // TODO - review whether going to hold mode directly from standby is ever necessary or wise ... what is the use case where we want this?
        if ((speedo.stopped() || allow_rolling_start) && ignition.signal && !panicstop && !tach.stopped())
            runmode = Hold;  // If the car is already running, go to Hold mode. If ignition is on w/o engine running, we'll end up in Stall Mode automatically
    }
    void run_stallMode() {  // In stall mode, the gas doesn't have feedback, so runs open loop, and brake pressure target proportional to joystick
        static Timer ch4start_disable_timer{3 * 60 * 1000000};  // set an X minute timer to disable ch4 start function, reducing risk of phantom starter bug
        if (_we_just_switched_modes) ch4start_disable_timer.reset();
        if (starter.motor) {  // if starter is running,
            if (hotrc.sw_event_unfilt(Ch4)) starter.request(ReqOff, hotrc.last_ch4_source());  // turn off starter if any Ch4 event occurred. Note keep this if separate, as it will reset the sw event
        } 
        else {  // if starter is not running,
            if (starter.now_req != ReqOn) {  // if there's no in-progress request to start the starter
                if (brake.motormode != run_motor_mode[runmode][_BrakeMotor]) // if brake mode was taken over by a failed start attempt,
                    brake.setmode(run_motor_mode[runmode][_BrakeMotor]);     // put it back to default
                if (gas.motormode != run_motor_mode[runmode][_Throttle])     // if gas mode was taken over by a failed start attempt,
                    gas.setmode(run_motor_mode[runmode][_Throttle]);         // put it back to default
            }
            if (!_stall_ch4start_timed_out) {  // if ch3-start functionality hasn't timed out,
                if (hotrc.sw_event_filt(Ch4)) starter.request(ReqOn, hotrc.last_ch4_source());  // turn on starter if a stable Ch4 event occurred. Note keep this if separate, as it will reset the sw event
            }
        }
        if (stall_ch4start_fn_timeout && !_stall_ch4start_timed_out && ch4start_disable_timer.expired()) {
            _stall_ch4start_timed_out = true;
            ezread.squintf("stall mode %dmin ch3-start fn timed out\n", ch4start_disable_timer.timeout() / (60 * 1000000));
            // ignition.request(ReqOff);  // ignition kill will result in fall back to Standby mode
        }
        if (!tach.stopped()) runmode = Hold;  // If we started the car, enter hold mode
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
        else if (_joy_has_been_centered && !starter.motor) runmode = _preferred_drivemode;  // if pushing up (and starter is off) then drive
    }
    void run_flyMode() {
        if (_we_just_switched_modes) car_hasnt_moved = speedo.stopped();  // note whether car is moving going into fly mode (probably not), this turns true once it has initially got moving
        if (car_hasnt_moved) {
            if (hotrc.joydir(Vert) != HrcUp) runmode = Hold;      // must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.stopped()) car_hasnt_moved = false;  // once car moves, we're allowed to release the trigger without falling out of fly mode
        }
        else if (speedo.stopped() && hotrc.joydir() != HrcUp) runmode = Hold;  // go to Hold Mode if we have come to a stop after moving  // && hotrc.pc[Vert][Filt] <= hotrc.pc[Vert][Cent]
        if (hotrc.sw_event_filt(Ch4)) tog_current_drivemode();                 // hrc ch4 button press switches drivemodes
    }
    void run_cruiseMode() {
        static Timer stopped_hold_timer{4000000};  // how long after coming to a stop should we drop to hold mode (to give driver a chance to get it moving again)?
        static Timer gesture_fly_timer{500000};    // how long a full-down press is held before auto-drop to fly mode
        if (_we_just_switched_modes) {  // upon first entering cruise mode, initialize things
            car_hasnt_moved = speedo.stopped();  // note whether car is moving going into the mode, this turns true once it has initially got moving
            if (!cruise_brake) brake.setmode(Release);  // override the mode set by run_motor_mode[][] array which assumes cruise_brake == true
            gesture_fly_timer.reset();  // initialize brake-trigger timer
        }
        if (car_hasnt_moved) {  // if car has not yet moved
            if (hotrc.joydir(Vert) != HrcUp) runmode = Hold;            // must keep pulling trigger until car moves, or it drops back to hold mode
            else if (!speedo.stopped()) car_hasnt_moved = false;  // once car moves, we're allowed to release the trigger without falling out of the mode
        }
        else if (speedo.stopped()) {  // if car has become stopped after previously moving
            if (hotrc.joydir() == HrcDn) runmode = Hold;  // if we have purposely braked to a stop, go to hold mode
            else if (hotrc.joydir() == HrcUp) _stopped_hold_timer_active = false;  // if trying to get moving again, keep car in cruise mode
            else {  // if we coasted to a stop and there's no immediate attempt to get moving, drop to hold mode. this is for safety, b/c car could easily resume movement, while driver assumes it's safe
                if (!_stopped_hold_timer_active) {
                    stopped_hold_timer.reset();
                    _stopped_hold_timer_active = true;
                }
                else if (stopped_hold_timer.expired()) runmode = Hold;
            }
        }
        else _stopped_hold_timer_active = false;  // if car is moving, cancel any impending holdmode timeout

        // if joystick is held full-brake for more than X, driver could be confused & panicking, drop to fly mode so fly mode will push the brakes
        if (!cruise_brake) {  // no need for this feature if cruise includes braking
            if (hotrc.pc[Vert][Filt] > hotrc.pc[Vert][OpMin] + hotrc.pc[Vert][Margin]) gesture_fly_timer.reset();  // keep resetting timer if joystick not at bottom
            else if (gesture_fly_timer.expired()) runmode = Fly;
        }
        if (hotrc.sw_event_filt(Ch4)) tog_current_drivemode();                 // hrc ch4 button press switches drivemodes
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