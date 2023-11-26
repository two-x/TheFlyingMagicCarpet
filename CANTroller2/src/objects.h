#pragma once
#include <Preferences.h>  // Functions for writing to flash, i think
#include <iomanip>  // For formatting console loop timing string output
#include <vector>  // used to group loop times with string labels
// #include "neopixel.h"
#include "web.h"
// #include <HardwareSerial.h>  // In case we ever talk to jaguars over asynchronous serial port, uncomment:
// HardwareSerial jagPort(1); // Open serisl port to communicate with jaguar controllers for steering & brake motors

// Instantiate objects
static Preferences prefs;  // Persistent config storage
static Hotrc hotrc;
static Potentiometer pot(pot_pin);
static Simulator sim(pot);
static TemperatureSensorManager tempsens(onewire_pin);
static Encoder encoder(encoder_a_pin, encoder_b_pin, encoder_sw_pin);
static CarBattery mulebatt(mulebatt_pin);
static LiPoBatt lipobatt(lipobatt_pin);
static PressureSensor pressure(pressure_pin);
static BrakePositionSensor brkpos(brake_pos_pin);
static Speedometer speedo(speedo_pin);
static Tachometer tach(tach_pin);
static I2C i2c(i2c_sda_pin, i2c_scl_pin);
static AirVeloSensor airvelo(i2c);
static MAPSensor mapsens(i2c);
static LightingBox lightbox;
static IdleControl idlectrl;
static GasServo gas;
static BrakeMotor brake;
static SteerMotor steer;
static WebManager web;

void update_web(void *parameter) {
    while (true) {
        web.update();
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 20ms, hopefully that's fast enough
    }
}

// RTOS task that updates temp sensors in a separate task
bool temp_err[NUM_TEMP_CATEGORIES];  // [AMBIENT/ENGINE/WHEEL]
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
void set_board_defaults() {  // true for dev boards, false for printed board (on the car)
    sim.set_can_sim(sens::pressure, running_on_devboard);
    sim.set_can_sim(sens::brkpos, running_on_devboard);
    sim.set_can_sim(sens::tach, running_on_devboard);
    sim.set_can_sim(sens::speedo, running_on_devboard);
    sim.set_can_sim(sens::mapsens, running_on_devboard);
    sim.set_can_sim(sens::airvelo, running_on_devboard);
    sim.set_can_sim(sens::basicsw, running_on_devboard);
    if (!running_on_devboard) {  // override settings if running on the real car
        sim.set_potmap(sens::none);        
        usb_jtag = false;
        console_enabled = false;     // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
        keep_system_powered = false; // Use true during development
        screensaver = false;         // Can enable experiment with animated screen draws
        looptime_print = false;      // Makes code write out timestamps throughout loop to serial port
        dont_take_temperatures = false;
        touch_reticles = false;
    }
    printf("Using %s defaults..\n", (running_on_devboard) ? "dev-board" : "vehicle-pcb");
}
void sim_setup() {
    printf("Simulator setup..\n");
    sim.register_device(sens::pressure, pressure, pressure.source());
    sim.register_device(sens::brkpos, brkpos, brkpos.source());
    sim.register_device(sens::airvelo, airvelo, airvelo.source());
    sim.register_device(sens::mapsens, mapsens, mapsens.source());
    sim.register_device(sens::tach, tach, tach.source());
    sim.register_device(sens::speedo, speedo, speedo.source());
    sim.set_potmap(prefs.getUInt("potmap", 2));  // 2 = sens::pressure
}
bool starter = LOW;  // Set by handler only. Reflects current state of starter signal (does not indicate source)
bool starter_drive = false;  // Set by handler only. High when we're driving starter, otherwise starter is an input
int starter_request = REQ_NA;
Timer starterTimer(starter_timeout_us);  // If remotely-started starting event is left on for this long, end it automatically  
void starter_update () {  // Starter bidirectional handler logic.  Outside code interacts with handler by setting starter_request = REQ_OFF, REQ_ON, or REQ_TOG
    if (starter_signal_support) {
        if (starter_request == REQ_TOG) starter_request = !starter_drive;  // translate toggle request to a drive request opposite to the current drive state
        if (starter_drive && ((starter_request == REQ_OFF) || starterTimer.expired())) {  // If we're driving the motor but need to stop
            starter_drive = false;
            set_pin (starter_pin, INPUT_PULLDOWN);  // we never assert low on the pin, just set pin as input and let the pulldown bring it low
        }
        if (!starter_drive && (starter_request != REQ_ON) && !sim.simulating(sens::starter)) {  // If we haven't been and shouldn't be driving, and not simulating
            do {
                starter = digitalRead(starter_pin);  // then read the pin, starter variable will store if starter is turned on externally
            } while (starter != digitalRead(starter_pin)); // starter pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
        }
        else if (!starter && (starter_request == REQ_ON) && remote_start_support) {  // If we got a request to start the motor, and it's not already being driven externally
            starter_drive = true;
            starter = HIGH;
            set_pin (starter_pin, OUTPUT);  // then set pin to an output
            write_pin (starter_pin, starter);  // and start the motor
            starterTimer.reset();  // if left on the starter will turn off automatically after X seconds
        }
        starter_request = REQ_NA;  // we have serviced whatever requests
    }
    else starter = LOW;
}
bool ignition = LOW;  // Set by handler only. Reflects current state of the signal
int ignition_request = REQ_NA;
bool panicstop = true;  // initialize in panic, because we could have just crashed and reset. If car is stopped, handler will clear it
int panicstop_request = REQ_ON;  // On powerup we assume the code just crashed during a drive, because it could have
Timer panicTimer(panic_relax_timeout_us);  // How long should a panic stop last?  We can't stay mad forever
void ignition_panic_update() {  // Run once each main loop, directly before panicstop_update()
    if (panicstop_request == REQ_TOG) panicstop_request = (req)(!panicstop);
    if (ignition_request == REQ_TOG) ignition_request = (req)(!ignition);
    // else if (ignition_request == ignition) ignition_request = REQ_NA;  // With this line, it ignores requests to go to state it's already in, i.e. won't do unnecessary pin write
    if (speedo.car_stopped() || panicTimer.expired()) panicstop_request = REQ_OFF;  // Cancel panic stop if car is stopped
    if (!speedo.car_stopped()) {
        if (ignition && ignition_request == REQ_OFF) panicstop_request = REQ_ON;  // ignition cut causes panic stop
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) panicstop_request = REQ_ON;
    }
    bool paniclast = panicstop;
    if (panicstop_request != REQ_NA) {
        panicstop = (bool)panicstop_request;
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
bool basicmodesw = LOW;
void basicsw_update() {
    if (!sim.simulating(sens::basicsw)) {  // Basic Mode switch
        do {
            basicmodesw = !digitalRead(basicmodesw_pin);   // !value because electrical signal is active low
        } while (basicmodesw != !digitalRead(basicmodesw_pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
    }
}
bool syspower = HIGH;  // Set by handler only. Reflects current state of the signal
void set_syspower(bool setting) {
    syspower = setting | keep_system_powered;
    write_pin(syspower_pin, syspower);
}
void hotrc_events_update(int runmode) {
    if (hotrc.sw_event(CH3)) ignition_request = REQ_TOG;  // Turn on/off the vehicle ignition. If ign is turned off while the car is moving, this leads to panic stop
    if (hotrc.sw_event(CH4)) {
        if (runmode == FLY || runmode == CRUISE) flycruise_toggle_request = true;
        else if (runmode == STALL) starter_request = REQ_TOG;
        else if (runmode == HOLD) starter_request = REQ_OFF;
        else if (runmode == SHUTDOWN && !shutdown_incomplete) sleep_request = REQ_ON;
        else if (runmode == ASLEEP) sleep_request = REQ_OFF; 
    }
    hotrc.toggles_reset();
}
// Calculates massairflow in g/s using values passed in if present, otherwise it reads fresh values
float maf_gps;  // Manifold mass airflow in grams per second
float massairflow(float _map = NAN, float _airvelo = NAN, float _ambient = NAN) {  // mdot (kg/s) = density (kg/m3) * v (m/s) * A (m2) .  And density = P/RT.  So,   mdot = v * A * P / (R * T)  in kg/s
    float temp = _ambient;
    if (std::isnan(_ambient)) {
        temp = tempsens.val(loc::AMBIENT);
        if (std::isnan(temp) && running_on_devboard) temp = tempsens.val(loc::ENGINE);
        if (std::isnan(temp)) return -1;  // Avoid crashing due to trying to read absent sensor
    }
    float T = 0.556 * (temp - 32.0) + 273.15;  // in K.  This converts from degF to K
    float R = 287.1;  // R (for air) in J/(kg·K) ( equivalent to 8.314 J/(mol·K) )  1 J = 1 kg*m2/s2
    float v = 0.447 * (std::isnan(_airvelo) ? airvelo.filt() : _airvelo); // in m/s   1609.34 m/mi * 1/3600 hr/s = 0.447
    float Ain2 = 3.1415926;  // in in2    1.0^2 in2 * pi  // will still need to divide by 1550 in2/m2
    float P = 6894.76 * (std::isnan(_map) ? mapsens.filt() : _map);  // in Pa   6894.76 Pa/PSI  1 Pa = 1 J/m3
    return v * Ain2 * P * 1000.0 / (R * T * 1550);  // mass air flow in grams per second (ug/s)   (1k g/kg * m/s * in2 * J/m3) / (J/(kg*K) * K * 1550 in2/m2) = g/s
}
// Loop timing related
Timer loopTimer(1000000); // how long the previous main loop took to run (in us)
float loop_sum_s, loop_avg_us, loopfreq_hz;
uint32_t looptimes_us[20];
bool loop_dirty[20];
int32_t loopno = 1, loopindex = 0, loop_recentsum = 0, loop_scale_min_us = 0, loop_scale_avg_max_us = 2500, loop_scale_peak_max_us = 25000;
int64_t loop_cout_mark_us;
uint32_t loop_cout_us = 0, loop_peak_us = 0, loop_now = 0;;
const uint32_t loop_history = 100;
uint32_t loop_periods_us[loop_history];
std::vector<std::string> loop_names(20);
void looptime_setup() {  // Run once at end of setup()
    if (looptime_print) {
        for (int32_t x=1; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
        loop_names[0] = std::string("top");
        loop_dirty[0] = false;
        loopindex = 1;
        looptimes_us[0] = esp_timer_get_time();
    }
    loopTimer.reset();  // start timer to measure the first loop
}
void looptime_mark(std::string loopname = std::string("")) {  // Add marks wherever you want in the main loop, set looptime_print true, will report times between all adjacent marks
    if (looptime_print) {
        if (loop_dirty[loopindex]) {
            loop_names[loopindex] = loopname;  // names[index], name);
            loop_dirty[loopindex] = false;
        }
        looptimes_us[loopindex] = esp_timer_get_time();
        loopindex++;
    }
}
float loop_calc_avg(uint32_t _loop_now, uint32_t _thisloop) {
    if (_loop_now == loop_history + 2) {
        loop_recentsum = _thisloop;
        for (int l = 0; l <= loop_history; l++)
            loop_recentsum += loop_periods_us[(_loop_now + l) % loop_history];
    }
    else loop_recentsum += _thisloop - loop_periods_us[loop_now];
    return (float)loop_recentsum/(float)loop_history;
}
void looptime_update() {  // Call once each loop at the very end
    uint32_t thisloop = (uint32_t)loopTimer.elapsed();
    loop_avg_us = loop_calc_avg(loop_now, thisloop);
    loop_periods_us[loop_now] = thisloop;  // us since beginning of this loop
    loopTimer.reset();
    loop_sum_s += (float)loop_periods_us[loop_now] / 1000000;
    // ema_filt(loop_periods_us[loop_now], &loop_avg_us, 0.01);
    if (loop_avg_us > 1) loopfreq_hz = 1000000/loop_avg_us;
    loop_peak_us = 0;
    for (int8_t i=0; i<loop_history; i++) if (loop_peak_us < loop_periods_us[i]) loop_peak_us = loop_periods_us[i]; 
    if (looptime_print) {
        loop_cout_mark_us = esp_timer_get_time();
        std::cout << std::fixed << std::setprecision(0);
        std::cout << "\r" << (uint32_t)loop_sum_s << "s #" << loopno;  //  << " av:" << std::setw(5) << (int32_t)(loop_avg_us);  //  << " av:" << std::setw(3) << loop_avg_ms 
        std::cout << " : " << std::setw(5) << loop_periods_us[loop_now] << " (" << loop_periods_us[loop_now]-loop_cout_us << ")us ";  // << " avg:" << loop_avg_us;  //  " us:" << esp_timer_get_time() << 
        for (int32_t x=1; x<loopindex; x++)
            std::cout << std::setw(3) << loop_names[x] << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1] << " ";
        std::cout << " cout:" << std::setw(5) << loop_cout_us;
        if (loop_periods_us[loop_now]-loop_cout_us > looptime_linefeed_threshold || !looptime_linefeed_threshold) std::cout << std::endl;
        loop_cout_us = (uint32_t)(esp_timer_get_time() - loop_cout_mark_us);
        loopindex = 0;
        looptime_mark ("top");
    }
    ++loop_now %= loop_history;
    loopno++;  // I like to count how many loops
}
// diag/error-checking routine. this should be turned into a class probably
enum err_type : int { LOST, RANGE, CALIB, WARN, CRIT, INFO, NUM_ERR_TYPES };
enum err_sens : int { e_hrcvert, e_hrcch3, e_pressure, e_brkpos, e_speedo, e_hrchorz, e_tach, e_temps, e_starter, e_hrcch4, e_basicsw, e_mulebatt, e_lipobatt, e_airvelo, e_mapsens, E_NUM_SENSORS, e_none };  // these are in order of priority
// diag tunable values
uint32_t err_timeout_us = 175000;
uint32_t err_margin_adc = 5;
char err_type_card[NUM_ERR_TYPES][5] = { "Lost", "Rang", "Cal", "Warn", "Crit", "Info" };
char err_sensor_card[E_NUM_SENSORS+1][7] = { "HrcV", "HrcCh3", "BrPres", "BrkPos", "Speedo", "HrcH", "Tach", "Temps", "Startr", "HrcCh4", "Basic", "MulBat", "LiPo", "Airflw", "MAP", "None" };
bool diag_ign_error_enabled = true;
// diag non-tunable values
Timer errTimer(err_timeout_us);
bool err_sensor_alarm[NUM_ERR_TYPES] = { false, false, false, false, false, false };
int8_t err_sensor_fails[NUM_ERR_TYPES] = { 0, 0, 0, 0, 0, 0 };
bool err_sensor[NUM_ERR_TYPES][E_NUM_SENSORS]; //  [LOST/RANGE] [e_hrchorz/e_hrcvert/e_hrcch3/e_hrcch4/e_pressure/e_brkpos/e_tach/e_speedo/e_airvelo/e_mapsens/e_temps/e_mulebatt/e_lipobatt/e_basicsw/e_starter]   // sens::opt_t::NUM_SENSORS]
uint8_t highest_pri_failing_sensor[NUM_ERR_TYPES];
uint8_t highest_pri_failing_last[NUM_ERR_TYPES];
void diag_update() {
    if (errTimer.expireset()) {
        // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
        // this is one approach
        // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
        // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
        // retreive with an OBD tool. Some checks are below, along with other possible things to check for:
        if (!ignition && !tach.engine_stopped()) {  // Check: if engine is turning when ignition signal is off
            if (diag_ign_error_enabled) { // See if the engine is turning despite the ignition being off
                Serial.println (F("Detected engine rotation in the absense of ignition signal"));  // , tach_filt_rpm, ignition
                diag_ign_error_enabled = false;  // Prevents endless error reporting the same error
            }
        }
        else diag_ign_error_enabled = true;
        // different approach
        bool not_detected;
        not_detected = false;  // first reset
        for (int cat = 0; cat < NUM_TEMP_CATEGORIES; cat++) temp_err[cat] = false;  // first reset
        for (int loc = 0; loc < tempsens.locint(loc::NUM_LOCATIONS); loc++) {
            if (!tempsens.detected(loc)) not_detected = true;
            else if (tempsens.val(loc) >= temp_lims_f[tempsens.errclass(loc)][WARNING]) temp_err[tempsens.errclass(loc)] = true;
        }
        err_sensor[LOST][e_temps] = not_detected;

        // Detect sensors disconnected or giving out-of-range readings.
        // TODO : The logic of this for each sensor should be moved to devices.h objects
        err_sensor[RANGE][e_brkpos] = (brkpos.in() < brkpos.op_min_in() || brkpos.in() > brkpos.op_max_in());
        err_sensor[LOST][e_brkpos] = (brkpos.raw() < err_margin_adc);
        err_sensor[RANGE][e_pressure] = (pressure.psi() < pressure.op_min_psi() || pressure.psi() > pressure.op_max_psi());
        err_sensor[LOST][e_pressure] = (pressure.raw() < err_margin_adc);
        err_sensor[RANGE][e_mulebatt] = (mulebatt.v() < mulebatt.op_min_v() || mulebatt.v() > mulebatt.op_max_v());
        for (int32_t ch = HORZ; ch <= CH4; ch++) {  // Hack: This loop depends on the indices for hotrc channel enums matching indices of hotrc sensor errors
            err_sensor[RANGE][ch] = !hotrc.radiolost() && ((hotrc.us[ch][RAW] < hotrc.us[ch][OPMIN] - (hotrc.us[ch][MARGIN] >> 1)) 
                                    || (hotrc.us[ch][RAW] > hotrc.us[ch][OPMAX] + (hotrc.us[ch][MARGIN] >> 1)));  // && ch != VERT
            err_sensor[LOST][ch] = !hotrc.radiolost() && ((hotrc.us[ch][RAW] < (hotrc.absmin_us - hotrc.us[ch][MARGIN]))
                                    || (hotrc.us[ch][RAW] > (hotrc.absmax_us + hotrc.us[ch][MARGIN])));
        }
        // err_sensor[RANGE][e_hrcvert] = (hotrc.us[VERT][RAW] < hotrc.failsafe_us - hotrc.us[ch][MARGIN])
        //     || ((hotrc.us[VERT][RAW] < hotrc.us[VERT][OPMIN] - halfMARGIN) && (hotrc.us[VERT][RAW] > hotrc.failsafe_us + hotrc.us[ch][MARGIN]));
        
        // Set sensor error idiot light flags
        // printf ("Sensors errors: ");
        
        // printf ("Sensor check: ");
        for (int32_t t=LOST; t<=RANGE; t++) {
            highest_pri_failing_sensor[t] = e_none;
            err_sensor_alarm[t] = false;
            err_sensor_fails[t] = 0;
            for (int32_t s=0; s<E_NUM_SENSORS; s++)
                if (err_sensor[t][s]) {
                    if (highest_pri_failing_sensor[t] = e_none) highest_pri_failing_sensor[t] = s;
                    err_sensor_alarm[t] = true;
                    err_sensor_fails[t]++;
                }
        }
        // printf ("\n");

        // Detectable transducer-related failures :: How we can detect them
        // Brakes:
        // * Pressure sensor, chain linkage, or vehicle brakes problem :: Motor retracted with position below zeropoint, but pressure did not increase.
        // * Pressure sensor zero point miscalibration (no force on pedal) :: Minimum pressure reading since startup has never reached 0 PSI or less (cal is too high), or, is more than a given margin below 0. * Note this can also be an auto-calibration approach
        // * Pressure sensor max point miscalibration (full force on pedal) :: When target set to max pressure, after motor moves to the point position isn't changing, the pressure reading deviates from max setting by more than a given margin. * Note this can also be an auto-calibration approach
        // * Position sensor problem :: When pressure is not near max, motor is driven more than X volt-seconds without position change (of the expected polarity).
        // * Brake motor problem :: When motor is driven more than X volt-seconds without any change (of the expected polarity) to either position or pressure.
        // * Brake calibration, idle high, or speedo sensor problem :: Motor retracted to near limit, with position decreased and pressure increased as expected, but speed doesn't settle toward 0.
        // * Pressure sensor problem :: If pressure reading is out of range, or ever changes in the unexpected direction during motor movement.
        // * Position sensor or limit switch problem :: If position reading is outside the range of the motor limit switches.
        // Steering:
        // * Chain derailment or motor or limit switch problem :: Motor told to drive for beyond X volt-seconds in one direction for > Y seconds.
        // Throttle/Engine:
        // * AirVelo/MAP/tach sensor failure :: If any of these three sensor readings are out of range to the other two.
        // Tach/Speedo:
        // * Sensor read problem :: Derivative of consecutive readings (rate of change) spikes higher than it's possible for the physical rotation to change - (indicates missing pulses)
        // * Disconnected/problematic speed sensor :: ignition is on, tach is nonzero, and runmode = hold/fly/cruise, yet speed is zero. Or, throttle is at idle and brake pressure high for enough time, yet speed readings are nonzero
        // * Disconnected/problematic tach sensor :: runmode is hold/fly/cruise, ignition is on and speed increases, but tach is below idle speed 
        // Temperature:
        // * Engine temperature sensor problem :: Over X min elapsed with Ignition on and tach >= low_idle, but engine temp is below nominal warmup temp.
        // * Cooling system, coolant, fan, thermostat, or coolant sensor problem :: Engine temp stays over ~204 for >= X min without coolant temp dropping due to fan.
        // * Axle, brake, etc. wheel issue or wheel sensor problem :: The hottest wheel temp is >= X degF hotter than the 2nd hottest wheel.
        // * Axle, brake, etc. wheel issue or wheel/ambient sensor problem :: A wheel temp >= X degF higher than ambient temp.
        // * Ignition problem, fire alarm, or temp sensor problem :: Ignition is off but a non-ambient temp reading increases to above ambient temp.
        // AirVelo:
        // * Air filter clogged, or carburetor problem :: Track ratio of massairflow/throttle angle whenever throttle is constant. Then, if that ratio lowers over time by X below that level, indicates restricted air. 
        // Battery:
        // * Battery low :: Mulebatt readings average is below a given threshold
        // * Inadequate charging :: Mulebatt readings average has decreased over long time period
        // 
        // More ideas to define better and implement:
        // * Check if the pressure response is characteristic of air being in the brake line.
        // * Axle/brake drum may be going bad (increased engine RPM needed to achieve certain speedo)  (beware going up hill may look the same).
        // * E-brake has been left on (much the same symptoms as above? (beware going up hill may look the same) 
        // * Carburetor not behaving (or air filter is clogged). (See above about engine deiseling - we can detect this!)
        // * After increasing braking, the actuator position changes in the opposite direction, or vise versa.
        // * Changing an actuator is not having the expected effect.
        // * A tunable value suspected to be out of tune.
        // * Check regularly for ridiculous shit. Compare our variable values against a predetermined list of conditions which shouldn't be possible or are at least very suspect. For example:
        //   A) Sensor reading is out of range, or has changed faster than it ever should.
        //   B) Stopping the car on entering hold/shutdown mode is taking longer than it ever should.
        //   C) Mule seems to be accelerating like a Tesla.
        //   D) Car is accelerating yet engine is at idle.
        // * The control system has nonsensical values in its variables.
    }
}
void err_print_info() {
    for (int32_t t=LOST; t<=INFO; t++) {
        printf ("diag err: %s (%d): ", err_type_card[t], err_sensor_fails[t]);
        for (int32_t s=0; s<=E_NUM_SENSORS; s++) {
            if (s == E_NUM_SENSORS) s++;
            if (err_sensor[t][s]) printf ("%s, ", err_sensor_card[s]);
        }
        printf("\n");
    }
}
#define RUN_TESTS 0
#if RUN_TESTS
    #include "unittests.h"
    void run_tests() {
        printf("Running tests...\n");
        delay(5000);
        test_Param();
        printf("Tests complete.\n");
        for(;;); // loop forever
    }
#else
    void run_tests() {}
#endif
