// objects.h : contains instantiations of major system components, and global functions
#pragma once
#include "globals.h"

class SysPower {
  public:
    int _pin;
    bool _val = HIGH, _notval = LOW;  // _notval is meant for idiot light
    SysPower(int pin) : _pin(pin) { set_pin(_pin, OUTPUT, _val); }
    void set(bool val) {
        _val = val || keep_system_powered;
        _notval = !_val;
        write_pin(_pin, _val);
    }
    void print_bootstatus() { ezread.squintf(ezread.highlightcolor, "Syspower (p%d) is: %s\n", _pin, _val ? "on" : "off"); }
    bool val() { return _val; }
    bool notval() { return _notval; }
    bool* val_ptr() { return &_val; }
    bool* notval_ptr() { return &_notval; }
};
static SysPower syspower(syspower_pin);
#include "i2cbus.h"
#include "sensors.h"
static Preferences prefs;  // Persistent config storage
static Potentiometer pot(pot_pin);
static Simulator sim(pot, &prefs);
static Hotrc hotrc(&sim, &pot);
#include "temperature.h"
#include "motors.h"
static TemperatureSensorManager tempsens(onewire_pin);
static CarBattery mulebatt(mulebatt_pin);
static PressureSensor pressure(pressure_pin);
static BrakePositionSensor brkpos(brake_pos_pin);
static Speedometer speedo(speedo_pin, 2.0f);  // mult-by-2 because there are 2 magnets per turn
static Tachometer tach(tach_pin, 0.125f);  // divide-by-8 due to external frequency divider
static I2C i2c(i2c_sda_pin, i2c_scl_pin);
static AirVeloSensor airvelo(&i2c);
static MAPSensor mapsens(&i2c);
static ThrottleControl gas(gas_pwm_pin, 0, 50);
static BrakeControl brake(brake_pwm_pin, 1, 50);
static SteeringControl steer(steer_pwm_pin, 2, 50);
static LightingBox lightbox(&i2c);  // lightbox(&diag);

void set_board_defaults() {          // true for dev boards, false for printed board (on the car)
    // ezread.squintf("  using %s defaults..\n", (running_on_devboard) ? "dev-board" : "vehicle-pcb");  // already printed during detection in temperature class
    if (running_on_devboard) return;      // override settings if running on the real car
    looptime_print = false;         // Makes code write out timestamps throughout loop to serial port
    touch_reticles = false;
    // console_enabled = false;     // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
    wifi_client_mode = false;       // Should wifi be in client or access point mode?
    keep_system_powered = false;    // Use true during development
    dont_take_temperatures = false;
    button_test_heartbeat_color = false;
    print_framebuffers = false;
    print_task_stack_usage = false;
}
void sim_setup() {
    sim.register_device(sens::pressure, pressure, pressure.source());
    sim.register_device(sens::brkpos, brkpos, brkpos.source());
    sim.register_device(sens::airvelo, airvelo, airvelo.source());
    sim.register_device(sens::mapsens, mapsens, mapsens.source());
    sim.register_device(sens::tach, tach, tach.source());
    sim.register_device(sens::speedo, speedo, speedo.source());
    sim.recall_cansim();
    sim.set_potmap();
    // sim.register_device(sens::engtemp, temp, temp.source());
    // sim.register_device(sens::starter, starter, starter.source());
    // for (sens sen=sens::pressure; sen<=sens::mulebatt; sen=(sens)((int)sen+1))
    // sim.set_can_sim(sens::joy, running_on_devboard);    
    // sim.set_can_sim(sens::pressure, running_on_devboard);
    // sim.set_can_sim(sens::brkpos, running_on_devboard);
    // sim.set_can_sim(sens::speedo, running_on_devboard);
    // sim.set_can_sim(sens::tach, running_on_devboard);
    // sim.set_can_sim(sens::airvelo, running_on_devboard);
    // sim.set_can_sim(sens::mapsens, running_on_devboard);
    // sim.set_can_sim(sens::engtemp, running_on_devboard);
    // sim.set_can_sim(sens::mulebatt, running_on_devboard);
    // // sim.set_can_sim(sens::starter, running_on_devboard);
    // sim.set_can_sim(sens::basicsw, running_on_devboard);
    // for (sens sen=sens::engtemp; sen<sens::basicsw; sen=(sens)((int)sen+1)) sim.set_can_sim(sen, false);
    // sim.set_potmap(sens::none);        
    ezread.squintf(ezread.highlightcolor, "Simulator: registered %d devices\n", sim.registered_device_count());
}
// RTOS task that updates temp sensors in a separate task
void tempsens_task(void *parameter) {
    while (true) {
        while (runmode == LowPower) vTaskDelay(pdMS_TO_TICKS(1000));
        if (!dont_take_temperatures) tempsens.update_temperatures();
        if (sim.potmapping(sens::engtemp)) {
            TemperatureSensor *engine_sensor = tempsens.get_sensor(loc::TempEngine);
            if (engine_sensor != nullptr) {
                engine_sensor->set_temperature(pot.mapToRange(tempsens.opmin(loc::TempEngine), tempsens.opmax(loc::TempEngine)));  // temp_sensor_min_f, temp_sensor_max_f));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for a second to avoid updating the sensors too frequently
    }
}

// Calculates massairflow in g/s using values passed in if present, otherwise it reads fresh values
float massairflow(float _map=NAN, float _airvelo=NAN, float _ambient=NAN) {  // mdot (kg/s) = density (kg/m3) * v (m/s) * A (m2) .  And density = P/RT.  So,   mdot = v * A * P / (R * T)  in kg/s
    static float maf_map_last;
    static float maf_velo_last;
    float temp = _ambient;
    // if (!airvelo.enabled() || !mapsens.enabled()) return NAN;
    float new_velo = airvelo.val();
    float new_map = mapsens.val();
    if (std::isnan(_ambient)) {
        if (new_velo == maf_velo_last && new_map == maf_map_last) return maf_gps;  // if no new sensor readings, don't recalculate the same value
        temp = tempsens.val(loc::TempAmbient);
        if (std::isnan(temp) && running_on_devboard) temp = tempsens.val(loc::TempBrake);
        if (std::isnan(temp)) return NAN;  // Avoid crashing due to trying to read absent sensor
    }
    maf_velo_last = new_velo;
    maf_map_last = new_map;
    float T = 0.556 * (temp - 32.0) + 273.15;  // in K.  This converts from degF to K
    float R = 287.1;  // R (for air) in J/(kg·K) ( equivalent to 8.314 J/(mol·K) )  1 J = 1 kg*m2/s2
    float v = 0.447 * (std::isnan(_airvelo) ? new_velo : _airvelo); // in m/s   1609.34 m/mi * 1/3600 hr/s = 0.447
    float Ain2 = M_PI;  // in in2    1.0^2 in2 * pi  // will still need to divide by 1550 in2/m2
    float P = 101325.0 * (std::isnan(_map) ? new_map : _map);  // in Pa   101325 Pa/atm  1 Pa = 1 J/m3
    float maf = v * Ain2 * P * 1000.0 / (R * T * 1550);  // mas\\s air flow in grams per second (g/s)   (1000 g/kg * m/s * in2 * J/m3) / (J/(kg*K) * K * 1550 in2/m2) = g/s
    if (std::abs(maf) < 0.001) maf = 0;
    // ezread.squintf("maf: %.3lf\n", maf);
    return maf;
}
// RTOS task that updates map and airflow sensors, and mass airflow calculation
void maf_task(void *parameter) {
    while (true) {
        if (i2c.detected(I2CMAP)) mapsens.update();          // manifold air pressure sensor  // 70 us + 2ms every 9 loops
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow other tasks to do stuff
        if (i2c.detected(I2CAirVelo)) airvelo.update();          // manifold air velocity sensor  // 20us + 900us every 4 loops
        maf_gps = massairflow();   // calculate grams/sec of air molecules entering the engine (Mass Air Flow) using velocity, pressure, and temperature of manifold air 
        if (!i2c.detected(I2CAirVelo) && !i2c.detected(I2CMAP)) vTaskDelay(pdMS_TO_TICKS(100));  // Delay for a second to avoid updating the sensors too frequently
    }
}
class ToggleSwitch {
  public:
    bool val = LOW;  // pin low means val high
  protected:
    int pin;
    bool last = 0;
    sens attached_sensor = sens::none;
    void readswpin() {
        last = val;
        do val = digitalRead(pin);   // !value because electrical signal is active low
        while (val == digitalRead(pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
    }
  public:
    ToggleSwitch(int _pin, sens _sens=sens::none) : pin(_pin), attached_sensor(_sens) {}
    void setup() {
        set_pin(pin, INPUT_PULLUP);
        readswpin();
    }
    void update() {
        if ((attached_sensor == sens::none) || !sim.simulating(attached_sensor)) readswpin();
        if (last != val) kick_inactivity_timer(HuTogSw);
    }
};
// the basic mode switch puts either a pullup or pulldown onto the serial tx pin. so to read it we must turn off the serial console, read the pin, then turn the console back on
// to limit interruptions in the console, we only read every few seconds, and only when not in a driving mode
class BasicModeSwitch : public ToggleSwitch {
  public:
    BasicModeSwitch(int _pin) : ToggleSwitch(_pin, sens::basicsw) {}
    void read() {  // ensure console is not active when calling this
        set_pin(pin, INPUT);
        val = digitalRead(pin);
        in_basicmode = val;
        if (last != val) kick_inactivity_timer(HuTogSw);
    }
    void print_bootstatus() { ezread.squintf(ezread.highlightcolor, "Basic switch (p%d) read: %s\n", pin, in_basicmode ? "high" : "low"); }  // can't print during setup() due to sharing pin w/ serial console
    void reread() {
        if (sim.simulating(attached_sensor)) return;
        if (runmode == Fly || runmode == Hold || runmode == Cruise) return;
        if (console_enabled) {
            // delay(200);  // give time for serial to print everything in its buffer
            Serial.flush();  // empty serial buffer before closing port
            Serial.end();  // close serial console to prevent crashes due to error printing
        }
        read();
        if (console_enabled) {
            Serial.begin(serial_monitor_baudrate);  // 9600/19200/28800/57600/115200/230400/460800/921600;  // restart serial console to prevent crashes due to error printing
            // Serial.begin(115200);  // restart serial console to prevent crashes due to error printing
            // delay(1500);  // note we will miss console messages for a bit surrounding a read, unless we add back these delays
        }
    }
};
static BasicModeSwitch basicsw(tx_basic_pin);

void test_console_throughput() {
    int step = 0, bits = 0;
    Serial.printf("  test: ");
    Timer testtimer{1000000};  //, chartimer{100000};
    while (!testtimer.expired()) {
        Serial.printf("\b \b \b \b \b%s", (step == 0) ? "-" : ((step == 1) ? "\\" : "/"));  // ezread console can not yet support backspaces
        std::fflush(stdout);  // ensure immediate output
        ++step %= 3;
        bits += 10 * 8;
    }
    Serial.printf("\b%ld baud\n", bits);
    ezread.printf("  tested %ld baud\n", bits);
}

#ifdef GIT_ENABLED
static inline void print_git_info() {
    if (build_git_error()) ezread.squintf(ezread.madcolor, "err: git data unknown due to git errors\n");
    else ezread.squintf("Git sha=%s (%s) branch=%s\n", kBuildSha.c_str(), (build_dirty()) ? "dirty" : " clean", kBuildBranch.c_str());
    ezread.squintf("  utc: %s\n", kBuildBuiltUtc.c_str());
    
    // std::string line = "Git sha=" + kBuildSha +
    //     " dirty=" + std::to_string(kBuildDirty ? 1 : 0) +
    //     " branch=" + kBuildBranch +
    //     " built=" + kBuildBuiltUtc;  // + " err=" + std::to_string(kBuildGitError ? 1 : 0);
    // ezread.squintf("%s\n", line.c_str());  // out.println(line.c_str());
}
#endif

void initialize_boot() {                        // set up those straggler pins which aren't taken care of inside class objects
    set_pin(sdcard_cs_pin, OUTPUT, HIGH);                   // deasserting unused cs line ensures available spi bus
    set_pin(free_pin, INPUT_PULLUP);                       // avoid undefined inputs
    if (!USB_JTAG) set_pin(steer_enc_a_pin, INPUT_PULLUP);  // avoid voltage level contention
    if (!USB_JTAG) set_pin(steer_enc_b_pin, INPUT_PULLUP);  // avoid voltage level contention
    Serial.flush();       // ensure serial buffer is fully printed out before closing the port
    Serial.end();         // close serial console port. serial doesn't work w/o this. maybe b/c the esp bootloader uses 115200 & we don't?
    basicsw.read();       // read the basic switch. the serial port must be fully stopped
    Serial.begin(serial_monitor_baudrate); // 9600/19200/28800/57600/115200/230400/460800/921600 // open console serial port (will reassign tx pin as output)
    delay(3000);   //  3000 is enough at 921600. Any less of a delay causes us to miss the first few lines of output
    ezread.setup();        // start the onscreen terminal
    ezread.squintf(ezread.announcecolor, "Magic carpet setup begin ..\n");
    ezread.squintf(ezread.highlightcolor, "EZread and serial consoles are started\n");  //  Serial.printf("Serial console is started\n");
    test_console_throughput();
    #ifdef GIT_ENABLED
    print_git_info();
    #endif
    syspower.print_bootstatus();
    basicsw.print_bootstatus();
}
void finalize_boot() {
    ezread.squintf("%s", console_enabled ? "" : "Stopping console during runtime\n");  // ezread.squintf(ezread.announcecolor, "** Setup done **\n");
    std::fflush(stdout);  // ensure immediate output
    if (!console_enabled) {
        delay(200);           // give time for serial to print everything in its buffer
        Serial.flush();       // ensure serial buffer is fully printed out before closing the port
        Serial.end();         // close serial console to prevent crashes due to error printing
    }
    ezread.printf(ezread.announcecolor, "Magic carpet is booted!\n");
    ezread.end_bootgraceperiod();
    bootup_complete = true;
}
class Ignition {  // the ignition object controls the car ignition signal and the panicstop state based on external requests
  private:
    int _pin, _ign_req = ReqNA, _panic_req = ReqNA;  // bool _verbose = true;
    Timer panicTimer{10000000};  // how long should a panic stop last?  we can't stay mad forever
    void set_ignition(bool newval) {
        _ign_req = ReqNA;
        signal = newval;
        write_pin(_pin, signal);  // turn car off or on (ign output is active high), ensuring to never turn on the ignition while panicking
    }
    void set_panicstop(bool newval) {
        _panic_req = ReqNA;                 // no matter what happens we will service the request
        if (!newval && !panicstop) return;  // no need to turn panic off if it's already off, so bail
        if (newval != panicstop) prefs.putUInt("panicstop", (uint32_t)newval);  // whenever panic state changes, update flash. this is read at boot, see diag.h
        if (newval && !panicstop) panicTimer.reset();  // start chillout timer when turning panic state on
        panicstop = newval;
    }
  public:
    bool signal = LOW;                    // set by handler only. Reflects current state of the signal
    Ignition(int pin) : _pin(pin) {}
    void setup() { ezread.squintf(ezread.highlightcolor, "Ignition (p%d) handler init\n", _pin); }
    void request(int req) { _ign_req = req; }  // ezread.squintf("ign req %s\n", requestcard[req].c_str());
    void panic_request(int req) { _panic_req = req; }  // if (_verbose) ezread.squintf("ign2: ign=%d ir=%s pan=%d pr=%s\n", (int)signal, requestcard[ign_req].c_str(), (int)panicstop, requestcard[panic_req].c_str()); delay(1);
    void update() {  // Run once each main loop
        if (runmode == LowPower) return;
        if (_panic_req == ReqTog) _panic_req = panicstop ? ReqOff : ReqOn;
        if (_ign_req == ReqTog) _ign_req = signal ? ReqOff : ReqOn;
        if (!speedo.stopped()) {  //  && (runmode == Fly || runmode == Cruise || runmode == Hold)
            if (signal && _ign_req == ReqOff) _panic_req = ReqOn;  // ignition cut while driving causes panic stop
            if (!sim.simulating(sens::joy) && hotrc.radiolost()) _panic_req = ReqOn;
        }
        if (_panic_req != ReqNA) set_panicstop((_panic_req == ReqOn) ? true : false);    // ezread.squintf("panic=%d\n", panicstop);
        if (panicstop) _ign_req = ReqOff;  // panic stop causes ignition cut
        if (speedo.stopped() || panicTimer.expired()) set_panicstop(false);  // Cancel panic stop if car is stopped
        if (_ign_req != ReqNA) set_ignition((_ign_req == ReqOn) ? HIGH : LOW);
    }
};
static Ignition ignition(ignition_pin);

// TODO - review to ensure the starter class acts right if the vehicle starter is turned on/off using external switch
class Starter {
  private:
    std::string startreqcard[NumStartReq] = { "unknwn", "class", "hotrc", "touch", "runmod" };
    int _pin, _requestor = StartClass;
    bool _verbose = false;
    Timer _starterTimer;  // if remotely-started starting event is left on for this long, end it automatically

    void turnon(int code=-1) {  // function to start the motor.  code argument is so we can determine which internal function call got us here
        static int req_source_timeout_ms = 5000;  // request to turn on starter will fail if no activity occurred within this time on any valid ReqOn source
        
        bool ok_to_start = false;  // prevent us from starting until we first check for requestor or call code anomalies
        if ((_requestor == StartHotrc) && (hotrc.ch4_button_last_ms() > req_source_timeout_ms))  // don't start without recent activity on the particular requestor source used
            ezread.squintf(ezread.madcolor, "err: starter reqOn by Hrc.%d, %dms ago\n", code, hotrc.ch4_button_last_ms());
        else if ((_requestor == StartTouch) && (hotrc.sim_button_last_ms() > req_source_timeout_ms))  // don't start without recent activity on the particular requestor source used
            ezread.squintf(ezread.madcolor, "err: starter reqOn by Sim.%d, %dms ago\n", code, hotrc.sim_button_last_ms());
        else if ((_requestor != StartHotrc) && (_requestor != StartTouch)) ezread.squintf(ezread.madcolor, "err: bad start requestor %s (code=%d)\n", startreqcard[_requestor].c_str(), code); // don't start if origin of request is invalid
        else if (code == -1) ezread.squintf(ezread.madcolor, "err: starter ReqOn w/o given code call\n");  // don't start if a code wasn't given when called
        else ok_to_start = true;  // if no error checks were triggered then return that no anomalies were detected

        if (ok_to_start) {
            ezread.squintf("starter turnon by %s.%d\n", startreqcard[_requestor].c_str(), code);  // maybe use ezread.squintf instead? (prints to both screen and console)
            if (push_gas_when_starting && check_brake_before_starting) gas.setmode(Starting);  // give it some gas if we're allowed to, unless brake wasn't checked (due to risk of lurching forward)
            _starterTimer.set((int64_t)(run_timeout * 1000000.0));  // if left on the starter will turn off automatically after X seconds
            motor = HIGH;                                           // ensure starter variable always reflects the starter status regardless who is driving it
            write_pin(_pin, motor);                                 // and start the motor
        }
        request(ReqNA, StartClass);  // cancel the starter on request which we have serviced (or properly ignored if it was erroneous)
    }
    void turnoff() {                        // function to stop the motor
        if (_verbose) ezread.squintf("starter turnoff by %s\n", startreqcard[_requestor].c_str());
        motor = LOW;                        // we will turn it off
        write_pin(_pin, motor);             // begin driving the pin low voltage
        request(ReqNA, StartClass);         // cancel current request
    }
    void check_for_external_tampering() {   // in case an external bug could be turning on the starter instead of us    
        bool pin_now = read_pin(_pin);      // get current value of pin to do the following check
        if (motor != pin_now) {             // check if someone changed the motor value or started driving our pin
            ezread.squintf(ezread.madcolor, "err: starter pin/pointer abuse! p:%d != m:%d\n", (int)pin_now, (int)motor); // how do we not miss this message?
            if (motor) ignition.panic_request(ReqOn);  // in case motor did start up, request panic will kill the ignition & stop car 
            request(ReqOff, StartClass);               // request to stop motor
        }
    }
    void verify_double_click() {
        static Timer twoclicktimer{2000000};
        static bool one_click_done = false;
        static int last_req_2click = ReqNA;
        if (_requestor != StartHotrc) return;  // 2 click requirement only applies to hotrc requests
        if (now_req == ReqOn && last_req_2click != ReqOn) {  // if we got a new on request
            if (!one_click_done) {                            // if this is the 1st click
                twoclicktimer.reset();                        // start a timer for opportunity to accept 2nd click
                request(ReqNA, StartClass);                   // cancel the turnon request
            }                                                 // otherwise if 2nd click then the turnon request remains active
            one_click_done = !one_click_done;                 // toggle next click will be the opposite of this one
        }
        if (twoclicktimer.expired()) {
            if (one_click_done) ezread.squintf(ezread.sadcolor, "warn: starter requires 2-clicks\n");
            one_click_done = false;  // cancel 2click sequence if too much time elapsed since last click
        }
        last_req_2click = now_req;  // allows us to detect when request first goes to on
    }
  public:
    int now_req = ReqNA;
    float run_timeout = 3.5, run_lolimit = 1.0, run_hilimit = 10.0;;  // timeout for motor to shut off automatically, in seconds
    bool req_active = false;  // for idiotlight display
    bool motor = LOW;         // motor is the current state of starter voltage. set in this class only

    Starter(int pin) : _pin(pin) { set_pin(_pin, OUTPUT); }
    void setup() { ezread.squintf(ezread.highlightcolor, "Starter (p%d) handler init\n", _pin); }

    void request(int req, int a_requestor=StartUnknown) {  // this is the only valid way to change the current request now_req, internally or externally
        static int last_req = ReqNA;  // for detecting external settings of now_req
        if (now_req != last_req) ezread.squintf(ezread.madcolor, "err: detected starter req value abuse!\n"); // report now_req was apparently set by somewhere besides this function
        else if ((a_requestor < 0) || (a_requestor >= NumStartReq)) ezread.squintf(ezread.madcolor, "err: invalid start requestor=%d\n", a_requestor); // prevent crash if requestor value passed is out of range
        else {  // if no errors happened then accept the request
            if (_verbose && ((req != last_req) || (a_requestor != StartClass))) ezread.squintf("new starter %s request from %s\n", requestcard[req].c_str(), startreqcard[a_requestor].c_str());  // report all request activity
            if (req == ReqTog) req = motor ? ReqOff : ReqOn;  // translate a toggle request to a drive request opposite to the current drive state
            now_req = req;
            _requestor = a_requestor;
            last_req = now_req;
        }
    }
    void update() {  // starter drive handler logic.  Outside code interacts with handler by calling request(XX) = ReqOff, ReqOn, or ReqTog
        static Timer brakeTimer{4000000};
        if (runmode == LowPower) return;  // bypass all this processing and sensor reads, etc. when we're in powerdown mode
        check_for_external_tampering();   // change request to ReqOff if pin value somehow differs from motor variable
        req_active = (now_req != ReqNA); // just for idiot light display

        if (motor && _starterTimer.expired()) request(ReqOff, StartClass); // request stop if motor was left on too long
        if (motor && (now_req == ReqOff)) turnoff();  // stop the motor. also will revert request to ReqNA

        if (two_click_starter) verify_double_click();
        if (motor || now_req != ReqOn) {              // if starter is already being driven, or we aren't being tasked to drive it
            if (now_req != ReqNA) request(ReqNA, StartClass);  // cancel any requests
            return;                                             // we're done here
        }
        // hereafter we know the motor is off and we are supposed to turn it on

        if (brake.autoholding) {  // if brake is successfully holding
            turnon(0);            // start the car
            return;               // finished servicing request
        }
        if (!brake_before_starting && !check_brake_before_starting) {  // if we don't need to apply the brake or even check the brake
            turnon(1);            // start the car
            return;               // finished servicing request
        }
        if (brake_before_starting) {        // if we must apply brakes before starting
            if (brake.feedback == _None) {  // check if brake is running in openloop mode (we can't control an autohold)
                ezread.squintf(ezread.sadcolor, "warn: starter can't use openloop brake\n");
                request(ReqNA, StartClass); // cancel turn on request
                return;                     // finished servicing request
            }
            else if (brake.motormode != AutoHold) {  // if we haven't yet told the brake to hold down
                ezread.squintf("starter autobraking..\n");
                brake.setmode(AutoHold);             // tell the brake to hold
                brakeTimer.reset();                  // start a timer to time box the application of brake
                return;                              // ditch out and wait for brake to push, leaving on request active
            }
        }
        // hereafter we are waiting for the brake push timer to expire, with ReqOn request active

        if (brakeTimer.expired()) {                      // waited long enough for the brake to push
            if (!check_brake_before_starting) turnon(2); // if no need to check whether brake succeeded, then start the car
            else {                                       // if we were supposed to apply the brakes and also check they got pushed
                ezread.squintf(ezread.sadcolor, "warn: cant start, brake not autoholding\n");
                request(ReqNA, StartClass);              // cancel the starter-on request, we can't drive the starter cuz the car might lurch forward
            }
        }  // otherwise we're still waiting for the brake to push, leaving ReqOn request active
    }
};
static Starter starter(starter_pin);
#include "diag.h"
static LoopTimer looptimer;
static BootMonitor watchdog(&prefs, &looptimer);
static DiagRuntime diag(&hotrc, &tempsens, &pressure, &brkpos, &tach, &speedo, &gas, &brake, &steer, &mulebatt, &airvelo, &mapsens, &pot, &ignition, &looptimer);
#include "tftsetup.h"
#include "inputs.h"
static Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
#include "runmodes.h"
static RunModeManager run;

// to avoid dependency conflicts, the BootButton class' actions() function is only prototyped 
// here, then its content is fleshed out below, after including the code that uses the object 
class BootButton : public MomentarySwitch {
  protected:
    void actions();  // function prototyhpe. see full definition below
    // int dummyprintcount = 0;  // useful for debugging ezread spam suppression feature
  public:
    BootButton(int apin) : MomentarySwitch(apin, false) {}
    void update() {
        MomentarySwitch::update();
        bootbutton_val = val();
        actions();
    }
};
static BootButton bootbutton(boot_sw_pin);
#include "animations.h"
#include "neopixel.h"
#include "display.h"

void BootButton::actions() {  // temporary (?) functionality added for development convenience
    // if (val()) ezread.printf("long ass print %d\n", ++dummyprintcount); return;  // useful for debugging ezread spam suppression feature
    if (longpress()) autosaver_request = ReqTog;  // screen.auto_saver(!auto_saver_enabled);
    if (shortpress()) {
        if (runmode == LowPower) sleep_request = ReqOff;
        else if (auto_saver_enabled) panel.change_saver();
        else {
            sim.toggle();
            // below trying to debug mulebatt error bit having same addr as nowtouch
            ezread.squintf("addr: batt:%08X touch:%08X\n", &sensidiots[_MuleBatt], &nowtouch);
            Timer printtimer;
            for (int i=0; i<NumTelemetryFull; i++) {
                ezread.squintf("%02d: %08X %s\n", i, &sensidiots[i], diag.ascii_name(i).c_str());            
            }
            ezread.squintf("printed in: ");
            ezread.squintf("%ld us\n", printtimer.elapsed());
        }  // ezread.printf("%s:%.2lf%s=%.2lf%s=%.2lf%%", pressure._short_name.c_str(), pressure.val(), pressure._si_units.c_str(), pressure.native(), pressure._native_units.c_str(), pressure.pc());
    }
}
class CoolingFan {  // new class to serve as thermostat for vehicle radiator fan, in the inevitable eventuality that our current Engine Guardian system finally breaks
  private:
    int _pin = -1;  // uses the pin and transistor circuit existing onboard originally intended for the vehicle fuel pump
  public:
    bool signal = LOW;  // reflects current state of the fan output pin
    CoolingFan(int pin) : _pin(pin) { set_pin(_pin, OUTPUT); }
    void setup() { ezread.squintf(ezread.highlightcolor, "Cooling fan: init vehicle engine thermostat\n"); }
    void update() {
        // read engine temp
        // turn on cooling fan as needed
    }
};
static CoolingFan fan(fan_tp_cs_pin);  // uses the pin and transistor circuit existing onboard originally intended for the vehicle fuel pump