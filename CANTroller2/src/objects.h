// objects.h : contains instantiations of major system components, and global functions
#pragma once
#include <esp_partition.h>
#include <random>
#include <esp_task_wdt.h>
#include <iostream>
#include <iomanip>  // For formatting console loop timing string output
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
static Speedometer speedo(speedo_pin);
static Tachometer tach(tach_pin);
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
void initialize_pins() {
    set_pin(ignition_pin, OUTPUT, LOW);
    set_pin(sdcard_cs_pin, OUTPUT, HIGH);    // deasserting unused cs line ensures available spi bus
    set_pin(syspower_pin, OUTPUT, syspower);
    set_pin(basicmodesw_pin, INPUT_PULLUP);
    if (!USB_JTAG) set_pin(steer_enc_a_pin, INPUT_PULLUP);         // avoid voltage level contention
    if (!USB_JTAG) set_pin(steer_enc_b_pin, INPUT_PULLUP);         // avoid voltage level contention
    set_pin(uart_tx_pin, INPUT);             // UART:  1st detect breadboard vs. vehicle PCB using TX pin pullup, then repurpose pin for UART and start UART 
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
        console_enabled = false;     // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
        wifi_client_mode = false;       // Should wifi be in client or access point mode?
        keep_system_powered = false; // Use true during development
        dont_take_temperatures = false;
        button_test_heartbeat_color = false;
        print_framebuffers = false;
    }
    printf("Using %s defaults..\n", (running_on_devboard) ? "dev-board" : "vehicle-pcb");
}
void partition_table() {
    printf("\n** Setup begin\nPartition Typ SubT  Address SizeByte   kB\n");
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
void ignition_panic_update(int runmode) {  // Run once each main loop
    if (panicstop_request == REQ_TOG) panicstop_request = !panicstop;
    if (ignition_request == REQ_TOG) ignition_request = !ignition;
    // else if (ignition_request == ignition) ignition_request = REQ_NA;  // With this line, it ignores requests to go to state it's already in, i.e. won't do unnecessary pin write
    if (speedo.car_stopped() || panicTimer.expired()) panicstop_request = REQ_OFF;  // Cancel panic stop if car is stopped
    if (!speedo.car_stopped() && (runmode == FLY || runmode == CRUISE || runmode == HOLD)) {
        if (ignition && ignition_request == REQ_OFF) panicstop_request = REQ_ON;  // ignition cut while driving causes panic stop
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) panicstop_request = REQ_ON;
    }
    bool paniclast = panicstop;
    if (panicstop_request != REQ_NA) {
        panicstop = (bool)panicstop_request;    // printf("panic=%d\n", panicstop);
        if (panicstop && !paniclast) panicTimer.reset();
    }
    if (panicstop) ignition_request = REQ_OFF;  // panic stop causes ignition cut
    if (ignition_request != REQ_NA) {
        ignition = (bool)ignition_request;
        write_pin (ignition_pin, ignition);  // turn car off or on (ign output is active high), ensuring to never turn on the ignition while panicking
    }
    panicstop_request = ignition_request = REQ_NA;  // cancel outstanding requests
}
void basicsw_update() {
    bool last_val = basicmodesw;
    if (!sim.simulating(sens::basicsw)) {  // Basic Mode switch
        do {
            basicmodesw = !digitalRead(basicmodesw_pin);   // !value because electrical signal is active low
        } while (basicmodesw != !digitalRead(basicmodesw_pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
    }
    if (last_val != basicmodesw) kick_inactivity_timer(8);
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
    float new_velo = airvelo.human();
    float new_map = mapsens.human();
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
class Starter {
  private:
    uint32_t pushbrake_timeout = 6000000;
    uint32_t run_timeout = 5000000;
    uint32_t turnoff_timeout = 100000;
    Timer starterTimer;  // If remotely-started starting event is left on for this long, end it automatically
    int lastbrakemode, lastgasmode, pin;
    bool pin_outputting = false;   // set by handler only. High when we're driving starter, otherwise starter is an input
  public:
    Starter(int _pin) : pin(_pin) {}
    bool motor = LOW;             // set by handler only. Reflects current state of starter signal (does not indicate source)
    int now_req = REQ_NA;
    bool req_active = false;
    void setup() {
        Serial.printf("Starter..\n");
        set_pin (pin, INPUT_PULLDOWN);  // set pin as input
    }
    void request(int req) { now_req = req; }
    void update() {  // starter bidirectional handler logic.  Outside code interacts with handler by calling request(XX) = REQ_OFF, REQ_ON, or REQ_TOG
        // Serial.printf("m:%d o:%d r:%d\n", motor, pin_outputting, now_req);
        if (!starter_signal_support) {  // if we don't get involved with or care about the car's starter
            motor = LOW;                // arbitrarily give this variable a fixed value
            now_req = REQ_NA;           // cancel any requests which we are ignoring anyway
            return;                     // no action
        }  // from here on, we can assume starter signal is supported
        if (now_req == REQ_TOG) now_req = !pin_outputting;  // translate a toggle request to a drive request opposite to the current drive state
        req_active = (now_req != REQ_NA);                   // for display
        if (pin_outputting && (!motor || (now_req == REQ_OFF) || starterTimer.expired())) {  // if we're driving the motor but need to stop or in the process of stopping
            if (motor) {                 // if motor is currently on
                motor = LOW;             // we will turn it off
                write_pin (pin, motor);  // begin driving the pin low voltage
                starterTimer.set((int64_t)turnoff_timeout);               // start timer to control length of low output
                if (gas.motormode == Starting) gas.setmode(lastgasmode);  // put the throttle back to doing whatever it was doing before
                return;                  // ditch out, leaving the motor-off request intact. we'll check on the timer next time
            }
            else if (starterTimer.expired()) {  // if it's been long enough since turning off the motor circuit ...
                set_pin (pin, INPUT_PULLDOWN);  // set pin as input
                pin_outputting = false;         // we are no longer driving the pin
                now_req = REQ_NA;               // reset the request line
            }
        }  // now, we have stopped driving the starter if we were supposed to stop
        if (sim.simulating(sens::starter)) motor = pin_outputting;  // if simulating starter, there's no external influence
        else if (!pin_outputting) {               // otherwise if we aren't driving the starter ...
            do {
                motor = digitalRead(pin);         // then let's read the pin, and starter variable will reflect whether starter has been turned on externally
            } while (motor != digitalRead(pin));  // due to a chip glitch, starter pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
            if (now_req != REQ_ON) now_req = REQ_NA;
        }  // now, if we aren't driving the starter, we've read the pin and know its status
        if (motor || now_req != REQ_ON || !remote_start_support) {  // if starter is already being driven by us or externally, or we aren't being tasked to drive it, or we don't even support driving it
            now_req = REQ_NA;          // cancel any requests
            return;                    // and ditch
        }  // from here on, we can assume the starter is off and we are supposed to turn it on
        if (brake.autoholding || !brake_before_starting) {  // if the brake is being held down, or if we don't care whether it is
            lastgasmode = gas.motormode;      // remember incumbent gas setting
            gas.setmode(Starting);            // give it some gas
            starterTimer.set((int64_t)run_timeout);  // if left on the starter will turn off automatically after X seconds
            pin_outputting = motor = HIGH;    // ensure starter variable always reflects the starter status regardless who is driving it
            set_pin (pin, OUTPUT);            // then set pin to an output
            write_pin (pin, motor);           // and start the motor
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
    src source() { return pin_outputting ? src::CALC : src::PIN; }
};
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
        float tachnow = tach.filt();
        pump_last = status;
        if (starter.motor || (ignition && (tachnow >= turnon_rpm))) {
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

class LoopTimer {
  public:
    LoopTimer() {}
    // Loop timing related
    Timer loop_timer = Timer(1000000);  // how long the previous main loop took to run (in us)
    int32_t loopno = 1, loopindex = 0, loop_recentsum = 0, loop_scale_min_us = 0, loop_scale_avg_max_us = 2500, loop_scale_peak_max_us = 25000;
    float loop_sum_s, loopfreq_hz;
    uint32_t looptimes_us[20];
    bool loop_dirty[20];
    int64_t loop_cout_mark_us, boot_mark;
    uint32_t loop_cout_us = 0, loop_peak_us = 0, loop_now = 0;;
    static constexpr uint32_t loop_history = 100;
    uint32_t loop_periods_us[loop_history];
    // std::vector<std::string> loop_names(20);
    std::string loop_names[20];
    void setup() {  // Run once at end of setup()
        boot_mark = esp_timer_get_time();
        if (looptime_print) {
            for (int32_t x=1; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
            loop_names[0] = std::string("top");
            loop_dirty[0] = false;
            loopindex = 1;
            looptimes_us[0] = esp_timer_get_time();
        }
        loop_timer.reset();  // start timer to measure the first loop
    }
    void mark(std::string loopname = std::string("")) {  // Add marks wherever you want in the main loop, set looptime_print true, will report times between all adjacent marks
        if (looptime_print) {
            if (loop_dirty[loopindex]) {
                loop_names[loopindex] = loopname;  // names[index], name);
                loop_dirty[loopindex] = false;
            }
            looptimes_us[loopindex] = esp_timer_get_time();
            loopindex++;
        }
    }
    float calc_avg(uint32_t _loop_now, uint32_t _thisloop) {
        if (_loop_now == loop_history + 2) {
            loop_recentsum = _thisloop;
            for (int l = 0; l <= loop_history; l++)
                loop_recentsum += loop_periods_us[(_loop_now + l) % loop_history];
        }
        else loop_recentsum += _thisloop - loop_periods_us[loop_now];
        return (float)loop_recentsum/(float)loop_history;
    }
    void update() {  // Call once each loop at the very end
        uint32_t thisloop = (uint32_t)loop_timer.elapsed();
        loop_avg_us = calc_avg(loop_now, thisloop);
        loop_periods_us[loop_now] = thisloop;  // us since beginning of this loop
        loop_timer.reset();
        loop_sum_s += (float)loop_periods_us[loop_now] / 1000000;
        // ema_filt(loop_periods_us[loop_now], &loop_avg_us, 0.01);
        if (loop_avg_us > 1) loopfreq_hz = 1000000/loop_avg_us;
        loop_peak_us = 0;
        for (int8_t i=0; i<loop_history; i++) if (loop_peak_us < loop_periods_us[i]) loop_peak_us = loop_periods_us[i]; 
        if (looptime_print) {
            loop_cout_mark_us = esp_timer_get_time();
            std::cout << std::fixed << std::setprecision(0);
            std::cout << "\r" << (uint32_t)loop_sum_s << "s #" << loopno;  //  << " av:" << std::setw(5) << (int32_t)(loop_avg_us);  //  << " av:" << std::setw(3) << loop_avg_ms 
            std::cout << " : " << std::setw(5) << loop_periods_us[loop_now] << " (" << std::setw(5) << loop_periods_us[loop_now]-loop_cout_us << ")us ";  // << " avg:" << loop_avg_us;  //  " us:" << esp_timer_get_time() << 
            for (int32_t x=1; x<loopindex; x++)
                std::cout << std::setw(3) << loop_names[x] << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1] << " ";
            std::cout << " cout:" << std::setw(5) << loop_cout_us;
            if (loop_periods_us[loop_now]-loop_cout_us > looptime_linefeed_threshold || !looptime_linefeed_threshold) std::cout << std::endl;
            loop_cout_us = (uint32_t)(esp_timer_get_time() - loop_cout_mark_us);
            loopindex = 0;
            mark ("top");
        }
        ++loop_now %= loop_history;
        loopno++;  // I like to count how many loops
    }
    float uptime() {  // returns uptime since last reset in minutes
        return (float)((esp_timer_get_time() - boot_mark)) / (60.0 * 1000000.0);
    }
};
class BootMonitor {
  private:
    int timeout_sec = 10;
    uint32_t uptime_recorded = -1, uptime_rounding = 5;
    Preferences* myprefs;
    LoopTimer* myloop;
    int codemode_last = 50000, crashcount = 0;
    uint32_t bootcount;                         // variable to track total number of boots of this code build
    uint32_t codemode_postmortem;
    std::string codemodecard[4] = { "confused", "booting", "parked", "driving" };
  public:
    BootMonitor(Preferences* _prefs, LoopTimer* _loop) : myprefs(_prefs), myloop(_loop) {}
    void bootcounter() {
        bootcount = myprefs->getUInt("bootcount", 0) + 1;
        myprefs->putUInt("bootcount", bootcount);
        codemode_postmortem = myprefs->getUInt("codemode", Confused);
        crashcount = myprefs->getUInt("crashcount", 0);
        if (codemode_postmortem != Parked) crashcount++;
        myprefs->putUInt("crashcount", crashcount);
        Serial.printf("Boot count: %d (%d/%d). Last lost power while %s after ", bootcount, bootcount-crashcount, crashcount, codemodecard[codemode_postmortem].c_str());
        uptime_postmortem();
    }
    void set_codemode(int _mode) {
        codemode = _mode;
        if (codemode_last != codemode) myprefs->putUInt("codemode", codemode);
        codemode_last = codemode;
    }
    void setup(int sec = -1) {
        if (sec >= 0) timeout_sec = sec;
        myprefs->begin("FlyByWire", false);
        bootcounter();
        set_codemode(Booting);
        if (!watchdog_enabled) return;
        Serial.printf("Boot manager.. \n");
        esp_task_wdt_init(timeout_sec, true);  // see https://github.com/espressif/esp-idf/blob/master/examples/system/task_watchdog/main/task_watchdog_example_main.c
        esp_task_wdt_add(NULL);
    }
    void add(TaskHandle_t taskh) {
        if (!watchdog_enabled) return;
        esp_task_wdt_add(taskh);
    }
    void write_uptime() {
        float get_uptime = myloop->uptime();
        uint32_t myround = std::min((uint32_t)get_uptime, uptime_rounding);
        uint32_t uptime_new = (uint32_t)(get_uptime / (float)myround) * myround;
        if (uptime_new == uptime_recorded) return;
        myprefs->putUInt("uptime", uptime_new);
        uptime_recorded = uptime_new;
    }
    void uptime_postmortem() {
        uint32_t last_uptime = myprefs->getUInt("uptime", 0);
        if (last_uptime > 0) {
            Serial.printf("just over %d min uptime\n", last_uptime);
            write_uptime();
        }
        else Serial.printf("under 1 min uptime\n");
    }
    void pet() {
        if (!watchdog_enabled) return;
        esp_task_wdt_reset();
    }
    void update() {
        pet();
        if (codemode == Booting) set_codemode(Confused);
        write_uptime();
    }
};
static LoopTimer looptimer;
static BootMonitor watchdog(&prefs, &looptimer);
#include "web.h"
static WebManager web(&looptimer);

void update_web(void *parameter) {
    while (true) {
        web.update();
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 20ms, hopefully that's fast enough
    }
}
