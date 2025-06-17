// objects.h : contains instantiations of major system components, and global functions
#pragma once
#include "globals.h"
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
static Speedometer speedo(speedo_pin, 0.5);  // 0.5x because there are 2 magnets
static Tachometer tach(tach_pin, 8.0);  // 8.0x frequency divider
static I2C i2c(i2c_sda_pin, i2c_scl_pin);
static AirVeloSensor airvelo(&i2c);
static MAPSensor mapsens(&i2c);
static ThrottleControl gas(gas_pwm_pin, 52);
static BrakeControl brake(brake_pwm_pin, 50);
static SteeringControl steer(steer_pwm_pin, 50);
static LightingBox lightbox(&i2c);  // lightbox(&diag);

void set_board_defaults() {          // true for dev boards, false for printed board (on the car)
    ezread.squintf("Using %s defaults..\n", (running_on_devboard) ? "dev-board" : "vehicle-pcb");
    if (running_on_devboard) return;      // override settings if running on the real car
    looptime_print = false;         // Makes code write out timestamps throughout loop to serial port
    encoder_reverse = true;
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
}
void set_syspower(bool setting) {
    syspower = setting | keep_system_powered;
    not_syspower = !syspower;
    write_pin(syspower_pin, syspower);
}
// RTOS task that updates temp sensors in a separate task
void tempsens_task(void *parameter) {
    while (true) {
        while (runmode == LOWPOWER) vTaskDelay(pdMS_TO_TICKS(1000));
        if (!dont_take_temperatures) tempsens.update_temperatures();
        if (sim.potmapping(sens::engtemp)) {
            TemperatureSensor *engine_sensor = tempsens.get_sensor(loc::ENGINE);
            if (engine_sensor != nullptr) {
                engine_sensor->set_temperature(pot.mapToRange(tempsens.opmin(loc::ENGINE), tempsens.opmax(loc::ENGINE)));  // temp_sensor_min_f, temp_sensor_max_f));
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
        temp = tempsens.val(loc::AMBIENT);
        if (std::isnan(temp) && running_on_devboard) temp = tempsens.val(loc::BRAKE);
        if (std::isnan(temp)) return NAN;  // Avoid crashing due to trying to read absent sensor
    }
    maf_velo_last = new_velo;
    maf_map_last = new_map;
    float T = 0.556 * (temp - 32.0) + 273.15;  // in K.  This converts from degF to K
    float R = 287.1;  // R (for air) in J/(kg·K) ( equivalent to 8.314 J/(mol·K) )  1 J = 1 kg*m2/s2
    float v = 0.447 * (std::isnan(_airvelo) ? new_velo : _airvelo); // in m/s   1609.34 m/mi * 1/3600 hr/s = 0.447
    float Ain2 = M_PI;  // in in2    1.0^2 in2 * pi  // will still need to divide by 1550 in2/m2
    float P = 101325.0 * (std::isnan(_map) ? new_map : _map);  // in Pa   101325 Pa/atm  1 Pa = 1 J/m3
    float maf = v * Ain2 * P * 1000.0 / (R * T * 1550);  // mass air flow in grams per second (g/s)   (1000 g/kg * m/s * in2 * J/m3) / (J/(kg*K) * K * 1550 in2/m2) = g/s
    if (std::abs(maf) < 0.001) maf = 0;
    // ezread.squintf("maf: %.3lf\n", maf);
    return maf;
}
// RTOS task that updates map and airflow sensors, and mass airflow calculation
void maf_task(void *parameter) {
    while (true) {
        mapsens.update();          // manifold air pressure sensor  // 70 us + 2ms every 9 loops
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay to allow other tasks to do stuff
        airvelo.update();          // manifold air velocity sensor  // 20us + 900us every 4 loops
        maf_gps = massairflow();   // calculate grams/sec of air molecules entering the engine (Mass Air Flow) using velocity, pressure, and temperature of manifold air 
        vTaskDelay(pdMS_TO_TICKS(95)); // Delay for a second to avoid updating the sensors too frequently
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
        do {
            val = digitalRead(pin);   // !value because electrical signal is active low
        } while (val == digitalRead(pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
    }
  public:
    ToggleSwitch(int _pin, sens _sens=sens::none) : pin(_pin), attached_sensor(_sens) {}
    void setup() {
        set_pin(pin, INPUT_PULLUP);
        readswpin();
    }
    void update() {
        if ((attached_sensor == sens::none) || !sim.simulating(attached_sensor)) readswpin();
        if (last != val) kick_inactivity_timer(HUTogSw);
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
        // readswpin();
        if (last != val) kick_inactivity_timer(HUTogSw);
    }
    void reread() {
        if (sim.simulating(attached_sensor)) return;
        if (runmode == FLY || runmode == HOLD || runmode == CRUISE) return;
        if (console_enabled) {
            // delay(200);  // give time for serial to print everything in its buffer
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
    ezread.squintf("Speed test:  ");
    Timer testtimer{1000000};  //, chartimer{100000};
    while (!testtimer.expired()) {
        Serial.printf("\b \b \b \b \b%s", (step == 0) ? "-" : ((step == 1) ? "\\" : "/"));  // ezread console can not yet support backspaces
        std::fflush(stdout);  // ensure immediate output
        ++step %= 3;
        bits += 10 * 8;
    }
    ezread.squintf("\b%ld baud\n", bits);
}
void initialize_pins_and_console() {                        // set up those straggler pins which aren't taken care of inside class objects
    set_pin(sdcard_cs_pin, OUTPUT, HIGH);                   // deasserting unused cs line ensures available spi bus
    set_pin(syspower_pin, OUTPUT, syspower);
    set_pin(free_pin, INPUT_PULLUP);                       // avoid undefined inputs
    if (!USB_JTAG) set_pin(steer_enc_a_pin, INPUT_PULLUP);  // avoid voltage level contention
    if (!USB_JTAG) set_pin(steer_enc_b_pin, INPUT_PULLUP);  // avoid voltage level contention
    basicsw.read();
    Serial.begin(921600); // 9600/19200/28800/57600/115200/230400/460800/921600 // open console serial port (will reassign tx pin as output)
    delay(1200);          // 1200 use for 115200 baud // this is needed to allow the uart to initialize and the screen board enough time after a cold boot
    ezread.squintf(LPUR, "\n** Setup begin **\n");
    ezread.squintf("  Serial console started. ");
    test_console_throughput();
    ezread.squintf("Syspower is %s, basicsw read: %s\n", syspower ? "on" : "off", in_basicmode ? "high" : "low");    
}
void stop_console() {
    ezread.squintf("%s", console_enabled ? "" : "Stopping console during runtime\n");
    ezread.squintf(LPUR, "** Setup done **\n");
    std::fflush(stdout);  // ensure immediate output
    if (!console_enabled) {
        delay(200);  // give time for serial to print everything in its buffer
        Serial.end();  // close serial console to prevent crashes due to error printing
    }
    ezread.printf(DCYN, "magic carpet is booted\n");
}
class Ignition {
  private:
    int ign_req = REQ_NA, panic_req = REQ_NA, pin;
    bool paniclast, booted = false;
    Timer panicTimer{15000000};  // how long should a panic stop last?  we can't stay mad forever
  public:
    bool signal = LOW;                    // set by handler only. Reflects current state of the signal
    // bool panicstop = false;                 // initialize NOT in panic, but with an active panic request, this puts us in panic mode with timer set properly etc.
    Ignition(int _pin) : pin(_pin) {}
    void setup() {  // must run after diag recovery function, to ensure initial ign value is asserted correctly
        bool pin_initial_val = LOW;
        if (!booted) {
            if (ign_req == REQ_ON) pin_initial_val = HIGH;
            else pin_initial_val = LOW;        
            set_pin(pin, OUTPUT, pin_initial_val);
        }
        booted = true;
        ign_req = REQ_NA;
    }
    void request(int req) { ign_req = req; }
    void panic_request(int req) { panic_req = req; }
    void update() {  // Run once each main loop
        if (runmode == LOWPOWER) return;
        if (panic_req == REQ_TOG) panic_req = !panicstop;
        if (ign_req == REQ_TOG) ign_req = !signal;
        // else if (request == signal) request = REQ_NA;  // With this line, it ignores requests to go to state it's already in, i.e. won't do unnecessary pin write
        if (speedo.stopped() || panicTimer.expired()) panic_req = REQ_OFF;  // Cancel panic stop if car is stopped
        if (!speedo.stopped() && (runmode == FLY || runmode == CRUISE || runmode == HOLD)) {
            if (signal && ign_req == REQ_OFF) panic_req = REQ_ON;  // ignition cut while driving causes panic stop
            if (!sim.simulating(sens::joy) && hotrc.radiolost()) panic_req = REQ_ON;
        }
        if (panic_req != REQ_NA) {
            panicstop = (panic_req == REQ_ON) ? true : false;    // ezread.squintf("panic=%d\n", panicstop);
            if (panicstop != paniclast) {
                prefs.putUInt("panicstop", (uint32_t)panicstop);  // this is read at boot, see diag.h
                if (panicstop) panicTimer.reset();
            }
        }
        paniclast = panicstop;
        if (panicstop) ign_req = REQ_OFF;  // panic stop causes ignition cut
        if (ign_req != REQ_NA && runmode != LOWPOWER) {
            signal = (bool)ign_req;
            write_pin(pin, signal);  // turn car off or on (ign output is active high), ensuring to never turn on the ignition while panicking
        }
        panic_req = ign_req = REQ_NA;  // cancel outstanding requests
    }
};
class Starter {
  private:
    int lastbrakemode, lastgasmode, pin, turnoff_timeout = 100000;
    Timer starterTimer, twoclicktimer{2000000}, brakeTimer{(int64_t)4000000};  // if remotely-started starting event is left on for this long, end it automatically
  public:
    Starter(int _pin) : pin(_pin) {}
    int now_req = REQ_NA, last_req = REQ_NA;
    bool req_active = false, one_click_done = false, motor = LOW;    // motor is the current state of starter voltage. set in this class only
    float run_timeout = 3.5, run_lolimit = 1.0, run_hilimit = 10.0;  // in seconds
    void setup() {
        ezread.squintf("Starter.. output-only supported\n");
        set_pin(pin, OUTPUT);                                  // set pin as output
    }
    void request(int req) { now_req = req; }                   // squintf("r:%d n:%d\n", req, now_req);}
    void turnon() {                                            // function to start the motor
        ezread.printf("starter turnon\n");                     // maybe use ezread.squintf instead? (prints to both screen and console)
        lastgasmode = gas.motormode;                           // remember incumbent gas setting
        if (push_gas_when_starting && check_brake_before_starting) gas.setmode(Starting);  // give it some gas, unless there's risk of lurching forward
        starterTimer.set((int64_t)(run_timeout * 1000000.0));  // if left on the starter will turn off automatically after X seconds
        motor = HIGH;                                          // ensure starter variable always reflects the starter status regardless who is driving it
        write_pin(pin, motor);                                 // and start the motor
        now_req = REQ_NA;                                      // we have serviced starter-on request, so cancel it
    }
    void turnoff() {                                              // function to stop the motor
        ezread.printf("starter turnoff\n");
        motor = LOW;                                              // we will turn it off
        write_pin(pin, motor);                                    // begin driving the pin low voltage
        if (gas.motormode == Starting) gas.setmode(lastgasmode);  // put the throttle back to doing whatever it was doing before
        if (brake.motormode == AutoHold && runmode != HOLD) brake.setmode(lastbrakemode);  // unless successfully into hold mode, put the brake back to doing whatever it was doing before
        now_req = REQ_NA;                                         // cancel current request
    }  // if (sim.simulating(sens::starter)) motor = pin_outputting;  // if simulating starter, there's no external influence
    void set_runtimeout(float newtime) { if (newtime <= run_hilimit && newtime >= run_lolimit) run_timeout = newtime; }
    void update() {  // starter drive handler logic.  Outside code interacts with handler by calling request(XX) = REQ_OFF, REQ_ON, or REQ_TOG
        if (runmode == LOWPOWER) return;                             // starter isn't supported during powerdown
        if (now_req == REQ_TOG) now_req = motor ? REQ_OFF : REQ_ON;  // translate a toggle request to a drive request opposite to the current drive state
        req_active = (now_req != REQ_NA);                   // for idiot light display
        if (two_click_starter) {                            // if 2 clicks are required to start
            if (now_req == REQ_ON && last_req != REQ_ON) {  // if we got a new on request
                if (!one_click_done) {                      // if this is the 1st click
                    twoclicktimer.reset();                  // start a timer for opportunity to accept 2nd click
                    now_req = REQ_NA;                       // cancel the turnon request
                }                                           // otherwise if 2nd click then the turnon request remains active
                one_click_done = !one_click_done;           // toggle next click will be the opposite of this one
            }
            if (twoclicktimer.expired()) one_click_done = false;  // cancel 2click sequence if too much time elapsed since last click
            last_req = now_req;                                   // allows us to detect when request first goes to on
        }
        if (motor && ((now_req == REQ_OFF) || starterTimer.expired())) turnoff();  // stop the motor if we're being asked to, or if it was left on too long
        if (motor || now_req != REQ_ON) {  // if starter is already being driven, or we aren't being tasked to drive it
            now_req = REQ_NA;              // cancel any requests
            return;                        // and ditch
        }  // from here on, we can assume the starter is off and we are supposed to turn it on
        if (brake.autoholding) {           // if brake is successfully holding
            turnon();                      // start the car
            return;                        // and then ditch out
        }
        if (brake_before_starting) {        // if we must apply brakes before starting
            if (brake.feedback == _None) {  // check if brake is running in openloop mode (we can't control an autohold)
                ezread.printf("warn: starter can't use openloop brake\n");
                now_req = REQ_NA;           // cancel turn on request
                return;                     // and then ditch out
            }
            else if (brake.motormode != AutoHold) {  // if we haven't yet told the brake to hold down
                ezread.printf("starter autobraking..\n");
                lastbrakemode = brake.motormode;     // remember incumbent brake setting
                brake.setmode(AutoHold);             // tell the brake to hold
                brakeTimer.reset();                  // start a timer to time box the application of brake
                return;                              // ditch out and wait for brake to push, leaving on request active
            }
        }
        else if (!check_brake_before_starting) {  // if we don't need to apply the brake nor even check for it
            turnon();    // start the car
            return;      // and then ditch out
        }
        if (brakeTimer.expired()) {                      // waited long enough for the brake to push
            if (!check_brake_before_starting) turnon();  // if no need to check whether brake succeeded, then start the car
            else {                                       // if we were supposed to apply the brakes and also check they got pushed
                ezread.printf("warn: starter cancel - no brake\n");
                now_req = REQ_NA;                        // cancel the starter-on request, we can't drive the starter cuz the car might lurch forward
                if (brake.motormode == AutoHold && runmode != HOLD) brake.setmode(lastbrakemode); // restore prev brake mode, unless it's already been changed
            }
        }  // otherwise we're still waiting for the brake to push, meanwhile the starter turn-on request remains intact
    }
};
static Ignition ignition(ignition_pin);
static Starter starter(starter_pin);
#include "diag.h"
static LoopTimer looptimer;
static BootMonitor watchdog(&prefs, &looptimer);
static DiagRuntime diag(&hotrc, &tempsens, &pressure, &brkpos, &tach, &speedo, &gas, &brake, &steer, &mulebatt, &airvelo, &mapsens, &pot, &ignition);
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
  public:
    BootButton(int apin) : MomentarySwitch(apin, false) {}
    void update() {
        MomentarySwitch::update();
        actions();
    }
};
static BootButton bootbutton(boot_sw_pin);
#include "animations.h"
#include "neopixel.h"
#include "display.h"

void BootButton::actions() {  // temporary (?) functionality added for development convenience
    if (longpress()) autosaver_request = REQ_TOG;  // screen.auto_saver(!auto_saver_enabled);
    if (shortpress()) {
        if (auto_saver_enabled) panel.change_saver();
        else {
            sim.toggle();
            Timer printtimer;
            ezread.squintf("addr: batt:%08X touch:%08X\n", &sensidiots[_MuleBatt], &nowtouch);
            for (int i=0; i<NumTelemetryFull; i++) {
                ezread.squintf("%02d: %08X %s\n", i, &sensidiots[i], diag.err_sens_card[i].c_str());            
            }
            ezread.squintf("printed in: ");
            ezread.squintf("%ld us\n", printtimer.elapsed());
        }  // ezread.printf("%s:%.2lf%s=%.2lf%s=%.2lf%%", pressure._short_name.c_str(), pressure.val(), pressure._si_units.c_str(), pressure.native(), pressure._native_units.c_str(), pressure.pc());
    }
}