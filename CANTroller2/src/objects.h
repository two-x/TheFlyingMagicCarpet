// objects.h : contains instantiations of major system components, and global functions
#pragma once
#include "freertos/FreeRTOS.h"  // for semaphores
#include "freertos/semphr.h"  // for semaphores
#include <esp_partition.h>
#include <random>
#include "globals.h"
#include "sensors.h"  // includes uictrl.h, i2cbus.h
#include "motors.h"  // includes qpid.h, temperature.h

// Instantiate objects
std::random_device rd;
std::mt19937 gen(rd());  // randomizer

static Preferences prefs;  // Persistent config storage
static Potentiometer pot(pot_pin);
static Simulator sim(pot, &prefs);
static Hotrc hotrc(&sim, &pot);
static TemperatureSensorManager tempsens(onewire_pin);
static CarBattery mulebatt(mulebatt_pin);
static PressureSensor pressure(pressure_pin);
static BrakePositionSensor brkpos(brake_pos_pin);
static Speedometer speedo(speedo_pin, 0.5);  // 0.5x because there are 2 magnets
static Tachometer tach(tach_pin, 8.0);  // 8.0x frequency divider
static I2C i2c(i2c_sda_pin, i2c_scl_pin);
static AirVeloSensor airvelo(&i2c);
static MAPSensor mapsens(&i2c);
static GasServo gas(gas_pwm_pin, 52);
static BrakeMotor brake(brake_pwm_pin, 50);
static SteerMotor steer(steer_pwm_pin, 50);
static LightingBox lightbox(&i2c);  // lightbox(&diag);

int rn(int values=256) {  // Generate a random number between 0 and values-1
    std::uniform_int_distribution<> dis(0, values - 1);
    return dis(gen);
}
void initialize_pins_and_console() {  // set up those straggler pins which aren't taken care of inside class objects
    set_pin(sdcard_cs_pin, OUTPUT, HIGH);    // deasserting unused cs line ensures available spi bus
    set_pin(syspower_pin, OUTPUT, syspower);
    if (!USB_JTAG) set_pin(steer_enc_a_pin, INPUT_PULLUP);         // avoid voltage level contention
    if (!USB_JTAG) set_pin(steer_enc_b_pin, INPUT_PULLUP);         // avoid voltage level contention
    set_pin(uart_tx_pin, INPUT);             // UART:  1st detect breadboard vs. vehicle PCB using TX pin pullup, then repurpose pin for UART and start UART 
    fun_flag = (read_pin(uart_tx_pin));       // detect bit at boot, can be used for any hardware devation we might need
    Serial.begin(115200);                     // open console serial port (will reassign tx pin as output)
    delay(1200);                              // This is needed to allow the uart to initialize and the screen board enough time after a cold boot
    Serial.printf("** Setup begin..\nSerial console started..\n");
    Serial.printf("Syspower turned %s. fun_flag is %s.\n", syspower ? "on" : "off", fun_flag ? "high" : "low");    
}
void set_board_defaults() {          // true for dev boards, false for printed board (on the car)
    sim.set_can_sim(sens::joy, false);
    for (sens sen=sens::pressure; sen<=sens::mulebatt; sen=(sens)((int)sen+1)) sim.set_can_sim(sen, running_on_devboard);
    // for (sens sen=sens::engtemp; sen<sens::basicsw; sen=(sens)((int)sen+1)) sim.set_can_sim(sen, false);
    // sim.set_can_sim(sens::basicsw, running_on_devboard);
    // sim.set_can_sim(sens::starter, running_on_devboard);
    // sim.set_can_sim(sens::mulebatt, running_on_devboard);
    if (!running_on_devboard) {      // override settings if running on the real car
        sim.set_potmap(sens::none);        
        looptime_print = false;      // Makes code write out timestamps throughout loop to serial port
        touch_reticles = false;
        // console_enabled = false;     // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
        wifi_client_mode = false;       // Should wifi be in client or access point mode?
        keep_system_powered = false; // Use true during development
        dont_take_temperatures = false;
        button_test_heartbeat_color = false;
        print_framebuffers = false;
        print_task_stack_usage = false;
    }
    printf("Using %s defaults..\n", (running_on_devboard) ? "dev-board" : "vehicle-pcb");
}
void print_partition_table() {
    printf("\nPartition Typ SubT  Address SizeByte   kB\n");
    esp_partition_iterator_t iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    const esp_partition_t *partition;
    while ((partition = esp_partition_get(iterator)) != NULL) {
        printf(" %8s %3d 0x%02x 0x%06x 0x%06x %4d\n", partition->label, partition->type, partition->subtype, partition->address, partition->size, (partition->size)/1024);
        if (!strcmp(partition->label, "coredump")) break;
        iterator = esp_partition_next(iterator);
    }
    esp_partition_iterator_release(iterator);
}
void sim_setup() {
    sim.register_device(sens::pressure, pressure, pressure.source());
    sim.register_device(sens::brkpos, brkpos, brkpos.source());
    sim.register_device(sens::airvelo, airvelo, airvelo.source());
    sim.register_device(sens::mapsens, mapsens, mapsens.source());
    sim.register_device(sens::tach, tach, tach.source());
    sim.register_device(sens::speedo, speedo, speedo.source());
    // sim.register_device(sens::engtemp, temp, temp.source());
    // sim.register_device(sens::starter, starter, starter.source());
    sim.set_potmap();
}
// RTOS task that updates temp sensors in a separate task
void update_temperature_sensors(void *parameter) {
    while (true) {
        if (!dont_take_temperatures)
            tempsens.update_temperatures();
        if (sim.potmapping(sens::engtemp)) {
            TemperatureSensor *engine_sensor = tempsens.get_sensor(loc::ENGINE);
            if (engine_sensor != nullptr) {
                engine_sensor->set_temperature(pot.mapToRange(temp_sensor_min_f, temp_sensor_max_f));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for a second to avoid updating the sensors too frequently
    }
}
void set_syspower(bool setting) {
    syspower = setting | keep_system_powered;
    write_pin(syspower_pin, syspower);
}
// Calculates massairflow in g/s using values passed in if present, otherwise it reads fresh values
float massairflow(float _map = NAN, float _airvelo = NAN, float _ambient = NAN) {  // mdot (kg/s) = density (kg/m3) * v (m/s) * A (m2) .  And density = P/RT.  So,   mdot = v * A * P / (R * T)  in kg/s
    static float maf_map_last;
    static float maf_velo_last;
    float temp = _ambient;
    float new_velo = airvelo.val();
    float new_map = mapsens.val();
    if (std::isnan(_ambient)) {
        if (new_velo == maf_velo_last && new_map == maf_map_last) return maf_gps;  // if no new sensor readings, don't recalculate the same value
        temp = tempsens.val(loc::AMBIENT);
        if (std::isnan(temp) && running_on_devboard) temp = tempsens.val(loc::ENGINE);
        if (std::isnan(temp)) return -1;  // Avoid crashing due to trying to read absent sensor
    }
    maf_velo_last = new_velo;
    maf_map_last = new_map;
    float T = 0.556 * (temp - 32.0) + 273.15;  // in K.  This converts from degF to K
    float R = 287.1;  // R (for air) in J/(kg·K) ( equivalent to 8.314 J/(mol·K) )  1 J = 1 kg*m2/s2
    float v = 0.447 * (std::isnan(_airvelo) ? new_velo : _airvelo); // in m/s   1609.34 m/mi * 1/3600 hr/s = 0.447
    float Ain2 = 3.1415926;  // in in2    1.0^2 in2 * pi  // will still need to divide by 1550 in2/m2
    float P = 101325.0 * (std::isnan(_map) ? new_map : _map);  // in Pa   101325 Pa/atm  1 Pa = 1 J/m3
    float maf = v * Ain2 * P * 1000.0 / (R * T * 1550);  // mass air flow in grams per second (g/s)   (1000 g/kg * m/s * in2 * J/m3) / (J/(kg*K) * K * 1550 in2/m2) = g/s
    if (std::abs(maf) < 0.001) maf = 0;
    return maf;
}
void psram_setup() {  // see https://www.upesy.com/blogs/tutorials/get-more-ram-on-esp32-with-psram#
    Serial.printf("PSRAM..");
    #ifndef BOARD_HAS_PSRAM
    Serial.printf(" support is currently disabled\n");
    return;
    #endif
    if (psramInit()) Serial.printf(" is correctly initialized, ");
    else Serial.printf(" is not available, ");
    // int available_PSRAM_size = ESP.getFreePsram();
    // Serial.println((String)"  PSRAM Size available (bytes): " + available_PSRAM_size);
    // int *array_int = (int *) ps_malloc(1000 * sizeof(int)); // Create an integer array of 1000
    // array_int[0] = 42;
    // array_int[999] = 42; //We access array values like classic array
    // int available_PSRAM_size_after = ESP.getFreePsram();
    // Serial.println((String)"  PSRAM Size available (bytes): " + available_PSRAM_size_after); // Free memory space has decreased
    // int array_size = available_PSRAM_size - available_PSRAM_size_after;
    // Serial.println((String)"Array size in PSRAM in bytes: " + array_size);
    // // free(array_int); //The allocated memory is freed.
    Serial.println((String)"size (B): " +ESP.getFreePsram());
}
class ToggleSwitch {
  public:
    bool pin_val = HIGH, val = LOW;  // pin low means val high
  private:
    int pin;
    bool last = 0;
    sens attached_sensor = sens::none;
    void readswpin() {
        last = val;
        do {
            pin_val = digitalRead(pin);   // !value because electrical signal is active low
        } while (pin_val != digitalRead(pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
        val = !pin_val;  // pin low means switch value is high
    }
  public:
    ToggleSwitch(int _pin, sens _sens=sens::none) : pin(_pin), attached_sensor(_sens) {
        set_pin(pin, INPUT_PULLUP);
        readswpin();
    }
    void update() {
        if ((attached_sensor == sens::none) || !sim.simulating(attached_sensor)) readswpin();
        if (last != val) kick_inactivity_timer(8);
    }
};
class Ignition {
  private:
    int ign_req = REQ_NA, panic_req = REQ_NA, pin;
    bool paniclast, booted = false;
    Timer panicTimer{15000000};  // How long should a panic stop last?  we can't stay mad forever
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
    void update(int runmode) {  // Run once each main loop
        if (panic_req == REQ_TOG) panic_req = !panicstop;
        if (ign_req == REQ_TOG) ign_req = !signal;
        // else if (request == signal) request = REQ_NA;  // With this line, it ignores requests to go to state it's already in, i.e. won't do unnecessary pin write
        if (speedo.stopped() || panicTimer.expired()) panic_req = REQ_OFF;  // Cancel panic stop if car is stopped
        if (!speedo.stopped() && (runmode == FLY || runmode == CRUISE || runmode == HOLD)) {
            if (signal && ign_req == REQ_OFF) panic_req = REQ_ON;  // ignition cut while driving causes panic stop
            if (!sim.simulating(sens::joy) && hotrc.radiolost()) panic_req = REQ_ON;
        }
        if (panic_req != REQ_NA) {
            panicstop = (panic_req == REQ_ON) ? true : false;    // printf("panic=%d\n", panicstop);
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
    uint32_t pushbrake_timeout = 6000000;
    uint32_t run_timeout = 5000000;
    uint32_t turnoff_timeout = 100000;
    Timer starterTimer;  // If remotely-started starting event is left on for this long, end it automatically
    int lastbrakemode, lastgasmode, pin;
  public:
    Starter(int _pin) : pin(_pin) {}
    bool motor = LOW;             // set by handler only. Reflects current state of starter signal (does not indicate source)
    int now_req = REQ_NA;
    bool req_active = false;
    void setup() {
        Serial.printf("Starter.. output-only supported\n");
        set_pin(pin, OUTPUT);  // set pin as output
    }
    void request(int req) { now_req = req; }  // Serial.printf("r:%d n:%d\n", req, now_req);}
    void update() {  // starter drive handler logic.  Outside code interacts with handler by calling request(XX) = REQ_OFF, REQ_ON, or REQ_TOG
        // if (now_req != NA) Serial.printf("m:%d r:%d\n", motor, now_req);
        if (now_req == REQ_TOG) now_req = motor ? REQ_OFF : REQ_ON;  // translate a toggle request to a drive request opposite to the current drive state
        if ((brake.feedback == _None) && (now_req == REQ_ON)) now_req = REQ_NA;  // never run the starter if brake is in openloop mode
        req_active = (now_req != REQ_NA);                   // for display
        if (motor && ((now_req == REQ_OFF) || starterTimer.expired()))  {  // if we're driving the motor but need to stop or in the process of stopping
            motor = LOW;             // we will turn it off
            write_pin (pin, motor);  // begin driving the pin low voltage
            if (gas.motormode == Starting) gas.setmode(lastgasmode);  // put the throttle back to doing whatever it was doing before
            now_req = REQ_NA;
            return;                  // ditch out, leaving the motor-off request intact. we'll check on the timer next time
        }  // now, we have stopped driving the starter if we were supposed to stop
        // if (sim.simulating(sens::starter)) motor = pin_outputting;  // if simulating starter, there's no external influence
        if (motor || now_req != REQ_ON) {  // if starter is already being driven, or we aren't being tasked to drive it
            now_req = REQ_NA;          // cancel any requests
            return;                    // and ditch
        }  // from here on, we can assume the starter is off and we are supposed to turn it on
        if (brake.autoholding || !brake_before_starting) {  // if the brake is being held down, or if we don't care whether it is
            lastgasmode = gas.motormode;      // remember incumbent gas setting
            gas.setmode(Starting);            // give it some gas
            starterTimer.set((int64_t)run_timeout);  // if left on the starter will turn off automatically after X seconds
            motor = HIGH;    // ensure starter variable always reflects the starter status regardless who is driving it
            write_pin(pin, motor);           // and start the motor
            now_req = REQ_NA;                 // we have serviced starter-on request, so cancel it
            return;                           // if the brake was right we have started driving the starter
        }  // from here on, we can assume the brake isn't being held on, which is in the way of our task to begin driving the starter
        if (brake.motormode != AutoHold) {   // if we haven't yet told the brake to hold down
            lastbrakemode = brake.motormode; // remember incumbent brake setting
            brake.setmode(AutoHold);         // tell the brake to hold
            starterTimer.set((int64_t)pushbrake_timeout);   // start a timer to time box that action
            return;  // we told the brake to hold down, leaving the request to turn the starter on intact, so we'll be back to check
        }  // at this point the brake has been told to hold but isn't holding yet
        if (starterTimer.expired()) {  // if we've waited long enough for the damn brake
            if (brake.motormode == AutoHold) brake.setmode(lastbrakemode);  // put the brake back to doing whatever it was doing before, unless it's already been changed
            now_req = REQ_NA;  // cancel the starter-on request, we can't drive the starter cuz the car might lurch forward
        }  // otherwise we're still waiting for the brake to push. the starter turn-on request remains intact
    }
    // src source() { return pin_outputting ? src::CALC : src::PIN; }
};

static ToggleSwitch basicsw(basicsw_pin, sens::basicsw);
static Ignition ignition(ignition_pin);
static Starter starter(starter_pin);

class FuelPump {  // drives power to the fuel pump when the engine is turning
  public:
    float off_v = 0.0;
    float on_min_v = 8.0;
    float on_max_v = 12.0;
    float volts = 0.0;
    float turnon_rpm = 50.0;
    float duty, pwm_period = 25000;  // used for software pwm timing
    int adc = 0;
    bool status = LOW, status_inverse = HIGH, pump_last = LOW, sw_pwm_out_now = LOW;
  private:
    bool variable_speed_output = false;  // this interferes with the gas servo pwm when enabled
    bool use_software_pwm = true;  // avoid using hardware resources for variable output, we can fake it with a timer
    Timer fuelTimer;
    int timeout = 25000;  // not tunable
    int pin, ledc_channel, pwm_frequency = 42, pwm_resolution = 8;  // for if hardware pwm is used
    void writepin() {
        if (variable_speed_output) {
            if (use_software_pwm) {
                if (duty > 99.5) write_pin(pin, HIGH);
                else if (duty < 0.5) write_pin(pin, LOW);
                else if (fuelTimer.expired()) {
                    sw_pwm_out_now = !sw_pwm_out_now;
                    timeout = (int)(duty * pwm_period / 100.0);
                    if (!sw_pwm_out_now) timeout = (int)pwm_period - timeout;
                    write_pin(pin, sw_pwm_out_now);
                    fuelTimer.set(timeout);
                }
            }
            else ledcWrite(ledc_channel, adc);
        }
        else write_pin(pin, status);
    }
  public:
    FuelPump(int _pin) : pin(_pin) {}
    void update() {
        if (!fuelpump_supported || !captouch) return;
        float tachnow = tach.val();
        pump_last = status;
        if (starter.motor || (ignition.signal && (tachnow >= turnon_rpm))) {
            volts = map(gas.pc[OUT], gas.pc[OPMIN], gas.pc[OPMAX], on_min_v, on_max_v);
            volts = constrain(volts, on_min_v, on_max_v);
            adc = map((int)volts, 0, (int)on_max_v, 0, 255);
            duty = 100.0 * volts / on_max_v;
            status = HIGH;
        }
        else {
            volts = off_v;
            adc = 0;
            duty = 0.0;
            status = LOW;
        }
        status_inverse = !status;  // for idiot light
        writepin();
    }
    void setup() {
        if (!fuelpump_supported || !captouch) return;  // if resistive touchscreen, then pin is needed for chip select
        Serial.printf("Fuel pump.. ");
        if (variable_speed_output) {
            if (use_software_pwm) {
                set_pin(pin, OUTPUT);  // initialize_pin
                fuelTimer.set(timeout);  // start the timer in case we are doing software pwm
                Serial.printf("using software pwm\n");
            }
            else {
                int ledc_channel = analogGetChannel(pin);
                if (ledcSetup(ledc_channel, pwm_frequency, pwm_resolution) == 0) Serial.printf("failed to configure ");
                else {
                    Serial.printf("using ");
                    ledcAttachPin(pin, ledc_channel);
                }
                Serial.printf("ledc ch %d, %d bit at %d Hz\n", ledc_channel, pwm_resolution, pwm_frequency);
            }
        }
        else {
            set_pin(pin, OUTPUT);  // initialize_pin
            Serial.printf("using digital drive\n");
        }
    }
};

static FuelPump fuelpump(tp_cs_fuel_pin);

#include "diag.h"
static LoopTimer looptimer;
static BootMonitor watchdog(&prefs, &looptimer);
static DiagRuntime diag(&hotrc, &tempsens, &pressure, &brkpos, &tach, &speedo, &gas, &brake, &steer, &mulebatt, &airvelo, &mapsens, &pot, &ignition);

#include "web.h"
static WebManager web(&looptimer);

#include "runmodes.h"
static RunModeManager run;

#include "display.h"  // includes neopixel.h, touch.h

void update_web(void *parameter) {
    while (true) {
        web.update();
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 20ms, hopefully that's fast enough
    }
}
void stop_console() {
    printf("** Setup done%s\n", console_enabled ? "" : ". stopping console during runtime");
    if (!console_enabled) {
        delay(200);  // give time for serial to print everything in its buffer
        Serial.end();  // close serial console to prevent crashes due to error printing
    }
    diagconsole.dprintf(CYN, "magic carpet is booted");
    diagconsole.dprintf("welcome to diag console");
}
void dump_errorcode_update() {
    static uint16_t status_last[NUM_ERR_TYPES];
    bool do_print = false;
    uint8_t color = NON;
    uint16_t now, was;
    int diff = 0;
    for (int e=0; e<NUM_ERR_TYPES; e++) {
        if (diag.errstatus[e] != status_last[e]) {
            do_print = true;
            now = diag.errstatus[e];
            was = status_last[e];
            while (now || was) {
                if ((now & 1) > (was & 1)) {
                    diff++;  // there's a new error
                    if (color == LGRN) color = diagconsole.defaultcolor;  // but also another error got cleared, so use default color
                    else if (color == NON) color = SALM;  // use the sad red bad-news color
                }
                else if ((now & 1) < (was & 1)) {
                    diff--;  // an error got cleared
                    if (color == SALM) color = diagconsole.defaultcolor;  // but there was also a new error, so use default color
                    else if (color == NON) color = LGRN;  // use the happy green color            
                }
                now >>= 1;
                was >>= 1;
            }
        }
        status_last[e] = diag.errstatus[e];
    }
    if (color == NON) color = diagconsole.defaultcolor;
    if (do_print) diagconsole.dprintf(color, "flag L:%04x R:%04x W:%04x", diag.errstatus[LOST], diag.errstatus[RANGE], diag.errstatus[WARN]);
}
void bootbutton_actions() {  // temporary (?) functionality added for development convenience
    if (bootbutton.longpress()) autosaver_request = REQ_TOG;  // screen.auto_saver(!auto_saver_enabled);
    if (bootbutton.shortpress()) {
        if (auto_saver_enabled) panel.change_saver();
        // else sim.toggle();
        else {
            sim.toggle();
            pressure.print_config(true);
            brkpos.print_config(true);
            speedo.print_config(true);
            tach.print_config(true);
            mulebatt.print_config(true);
            // diagconsole.dprintf("%s:%.2lf%s=%.2lf%s=%.2lf%%", pressure._short_name.c_str(), pressure.val(), pressure._si_units.c_str(), pressure.native(), pressure._native_units.c_str(), pressure.pc());
        }
    }
}