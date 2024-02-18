// objects.h : contains instantiations of major system components, and global functions
#pragma once
#include <Preferences.h>  // Functions for writing to flash, i think
#include <esp_partition.h>
#include <random>
#include "globals.h"
#include "sensors.h"  // includes uictrl.h, i2cbus.h
#include "motors.h"  // includes qpid.h, temperature.h
#include "diag.h"
#include "web.h"
// #include <HardwareSerial.h>  // In case we ever talk to jaguars over asynchronous serial port, uncomment:
// HardwareSerial jagPort(1); // Open serisl port to communicate with jaguar controllers for steering & brake motors

// Instantiate objects
std::random_device rd;
std::mt19937 gen(rd());  // randomizer
static Preferences prefs;  // Persistent config storage
static Potentiometer pot(pot_pin);
static Simulator sim(pot);
static Hotrc hotrc(&sim);
static TemperatureSensorManager tempsens(onewire_pin);
static Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
static MomentaryButton bootbutton(boot_sw_pin, false);
static CarBattery mulebatt(mulebatt_pin);
static PressureSensor pressure(pressure_pin);
static BrakePositionSensor brkpos(brake_pos_pin);
static Speedometer speedo(speedo_pin);
static Tachometer tach(tach_pin);
static I2C i2c(i2c_sda_pin, i2c_scl_pin);
static AirVeloSensor airvelo(&i2c);
static MAPSensor mapsens(&i2c);
static GasServo gas(gas_pwm_pin, 60);
static BrakeMotor brake(brake_pwm_pin, 50);
static SteerMotor steer(steer_pwm_pin, 50);
static LoopTimer looptimer;
static WebManager web(&looptimer);
static DiagRuntime diag(&hotrc, &tempsens, &pressure, &brkpos, &tach, &speedo, &gas, &brake, &steer, &mulebatt, &airvelo, &mapsens, &pot, &maf_gps, &ignition);
static LightingBox lightbox(&i2c);  // lightbox(&diag);

int rn(int values=256) {  // Generate a random number between 0 and values-1
    std::uniform_int_distribution<> dis(0, values - 1);
    return dis(gen);
}
void initialize_pins() {
    set_pin(ignition_pin, OUTPUT, LOW);
    set_pin(sdcard_cs_pin, OUTPUT, HIGH);    // deasserting unused cs line ensures available spi bus
    set_pin(touch_cs_pin, OUTPUT, HIGH);     // deasserting touch cs line in case i2c captouch screen is used
    set_pin(syspower_pin, OUTPUT, syspower);
    set_pin(basicmodesw_pin, INPUT_PULLUP);
    set_pin(starter_pin, INPUT_PULLDOWN);
    // set_pin(free_pin, INPUT_PULLUP);         // ensure defined voltage level is present for unused pin
    set_pin(uart_tx_pin, INPUT);             // UART:  1st detect breadboard vs. vehicle PCB using TX pin pullup, then repurpose pin for UART and start UART 
    if (!usb_jtag) set_pin(steer_enc_a_pin, INPUT_PULLUP);  // assign stable defined behavior to currently unused pin
    if (!usb_jtag) set_pin(steer_enc_b_pin, INPUT_PULLUP);  // assign stable defined behavior to currently unused pin
}
void set_board_defaults() {          // true for dev boards, false for printed board (on the car)
    sim.set_can_sim(sens::joy, false);
    for (sens sen=sens::pressure; sen<=sens::mapsens; sen=(sens)((int)sen+1)) sim.set_can_sim(sen, running_on_devboard);
    for (sens sen=sens::engtemp; sen<sens::basicsw; sen=(sens)((int)sen+1)) sim.set_can_sim(sen, false);
    sim.set_can_sim(sens::basicsw, running_on_devboard);
    sim.set_can_sim(sens::mulebatt, running_on_devboard);
    if (!running_on_devboard) {      // override settings if running on the real car
        sim.set_potmap(sens::none);        
        usb_jtag = false;
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
    sim.set_potmap(prefs.getUInt("potmap", 2));  // 2 = sens::pressure
}
void update_web(void *parameter) {
    while (true) {
        web.update();
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 20ms, hopefully that's fast enough
    }
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
void starter_update () {  // Starter bidirectional handler logic.  Outside code interacts with handler by setting starter_request = REQ_OFF, REQ_ON, or REQ_TOG
    if (starter_signal_support) {
        if (starter_request == REQ_TOG) starter_request = !starter_drive;  // translate toggle request to a drive request opposite to the current drive state
        if (starter_drive && ((starter_request == REQ_OFF) || starterTimer.expired())) {  // If we're driving the motor but need to stop
            starter_drive = false;
            set_pin (starter_pin, INPUT_PULLDOWN);  // we never assert low on the pin, just set pin as input and let the pulldown bring it low
        }
        if (!starter_drive && (starter_request != REQ_ON) && !sim.simulating(sens::starter)) {  // If we haven't been and shouldn't be driving, and not simulating
            do {
                starter = digitalRead(starter_pin);  // then read the pin, and starter variable will reflect whether starter has been turned on externally
            } while (starter != digitalRead(starter_pin)); // due to a chip glitch, starter pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
        }
        else if (!starter && (starter_request == REQ_ON) && remote_start_support) {  // If we got a request to start the motor, and it's not already being driven externally
            starter_drive = true;
            starter = HIGH;  // ensure starter variable always reflects the starter status regardless who is driving it
            set_pin (starter_pin, OUTPUT);  // then set pin to an output
            write_pin (starter_pin, starter);  // and start the motor
            starterTimer.reset();  // if left on the starter will turn off automatically after X seconds
        }
        starter_request = REQ_NA;  // we have serviced whatever requests
    }
    else starter = LOW;
}
void ignition_panic_update() {  // Run once each main loop
    if (panicstop_request == REQ_TOG) panicstop_request = !panicstop;
    if (ignition_request == REQ_TOG) ignition_request = !ignition;
    // else if (ignition_request == ignition) ignition_request = REQ_NA;  // With this line, it ignores requests to go to state it's already in, i.e. won't do unnecessary pin write
    if (speedo.car_stopped() || panicTimer.expired()) panicstop_request = REQ_OFF;  // Cancel panic stop if car is stopped
    if (!speedo.car_stopped()) {
        if (ignition && ignition_request == REQ_OFF) panicstop_request = REQ_ON;  // ignition cut causes panic stop
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) panicstop_request = REQ_ON;
    }
    bool paniclast = panicstop;
    if (panicstop_request != REQ_NA) {
        panicstop = (bool)panicstop_request;  // printf("panic=%d\n", panicstop);
        if (panicstop && !paniclast) panicTimer.reset();
    }
    if (panicstop) ignition_request = REQ_OFF;  // panic stop causes ignition cut
    if (ignition_request != REQ_NA) {
        ignition = (bool)ignition_request;
        write_pin (ignition_pin, ignition);  // Turn car off or on (ign output is active high), ensuring to never turn on the ignition while panicking
    }
    panicstop_request = REQ_NA;
    ignition_request = REQ_NA;  // Make sure this goes after the last comparison
}
void basicsw_update() {
    bool last_val = basicmodesw;
    if (!sim.simulating(sens::basicsw)) {  // Basic Mode switch
        do {
            basicmodesw = !digitalRead(basicmodesw_pin);   // !value because electrical signal is active low
        } while (basicmodesw != !digitalRead(basicmodesw_pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
    }
    if (last_val != basicmodesw) kick_inactivity_timer(7);
}
void set_syspower(bool setting) {
    syspower = setting | keep_system_powered;
    write_pin(syspower_pin, syspower);
}
void hotrc_events(int runmode) {
    if (hotrc.sw_event(CH3) && runmode != ASLEEP) ignition_request = REQ_TOG;  // Turn on/off the vehicle ignition. If ign is turned off while the car is moving, this leads to panic stop
    if (hotrc.sw_event(CH4)) {
        if (runmode == FLY || runmode == CRUISE) flycruise_toggle_request = true;
        else if (runmode == STALL) starter_request = REQ_TOG;
        else if (runmode == HOLD) starter_request = REQ_OFF;
        else if (runmode == SHUTDOWN && !shutdown_incomplete) sleep_request = REQ_ON;
        else if (runmode == ASLEEP) sleep_request = REQ_OFF; 
    }
    // hotrc.toggles_reset();
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