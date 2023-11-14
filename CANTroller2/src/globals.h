#pragma once
#include <Preferences.h>  // Functions for writing to flash, i think
#include <iomanip>  // For formatting console loop timing string output
#include <vector>  // used to group loop times with string labels
// #include <HardwareSerial.h>  // In case we ever talk to jaguars over asynchronous serial port, uncomment:
// HardwareSerial jagPort(1); // Open serisl port to communicate with jaguar controllers for steering & brake motors

enum temp_categories { AMBIENT = 0, ENGINE = 1, WHEEL = 2, num_temp_categories };
enum temp_lims { DISP_MIN, OP_MIN, OP_MAX, WARNING, ALARM, DISP_MAX }; // Possible sources of gas, brake, steering commands
float temp_lims_f[3][6]{
    {0.0, 45.0, 115.0, 120.0, 130.0, 220.0},  // [AMBIENT][DISP_MIN/OP_MIN/OP_MAX/WARNING/ALARM]
    {0.0, 178.0, 198.0, 202.0, 205.0, 220.0}, // [ENGINE][DISP_MIN/OP_MIN/OP_MAX/WARNING/ALARM]
    {0.0, 50.0, 120.0, 130.0, 140.0, 220.0},  // [WHEEL][DISP_MIN/OP_MIN/OP_MAX/WARNING/ALARM] (applies to all wheels)
};
float temp_room = 77.0;          // "Room" temperature is 25 C = 77 F  Who cares?
float temp_sensor_min_f = -67.0; // Minimum reading of sensor is -25 C = -67 F
float temp_sensor_max_f = 257.0; // Maximum reading of sensor is 125 C = 257 F
bool temp_err[num_temp_categories];  // [AMBIENT/ENGINE/WHEEL]

// Instantiate objects
static Preferences config;  // Persistent config storage
static Hotrc hotrc;
static Potentiometer pot(pot_wipe_pin);
static Simulator sim(pot);
static TemperatureSensorManager tempsens(onewire_pin);
static Encoder encoder(encoder_a_pin, encoder_b_pin, button_pin);
static CarBattery mulebatt(mulebatt_pin);
static LiPoBatt lipobatt(lipobatt_pin);
static PressureSensor pressure(pressure_pin);
static BrakePositionSensor brakepos(brake_pos_pin);
static Speedometer speedo(speedo_pin);
static Tachometer tach(tach_pin);
static I2C i2c(i2c_sda_pin, i2c_scl_pin);
static AirVeloSensor airvelo(i2c);
static MAPSensor mapsens(i2c);
static Throttle throttle;
static GasServo gas;
static BrakeMotor brake;
static SteerMotor steer;
static neopixelstrip neo;

// RTOS task that updates temp sensors in a separate task
void update_temperature_sensors(void *parameter) {
    while (true) {
        if (!dont_take_temperatures)
            tempsens.update_temperatures();
        if (sim.potmapping(sens::engtemp)) {
            TemperatureSensor *engine_sensor = tempsens.get_sensor(loc::engine);
            if (engine_sensor != nullptr) {
                engine_sensor->set_temperature(pot.mapToRange(temp_sensor_min_f, temp_sensor_max_f));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for a second to avoid updating the sensors too frequently
    }
}

void set_board_defaults(bool devboard) {  // true for dev boards, false for printed board (on the car)
    if (devboard) {
        sim.set_can_sim(sens::pressure, true);
        sim.set_can_sim(sens::brkpos, true);
        sim.set_can_sim(sens::tach, true);
        sim.set_can_sim(sens::speedo, true);
        sim.set_can_sim(sens::mapsens, true);
        sim.set_can_sim(sens::airvelo, true);
        sim.set_can_sim(sens::basicsw, true);
        sim.set_potmap(sens::pressure);
    }
    else {  // override settings if running on the real car
        usb_jtag = false;
        console_enabled = false;     // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
        keep_system_powered = false; // Use true during development
        screensaver = false;         // Can enable experiment with animated screen draws
        looptime_print = false;      // Makes code write out timestamps throughout loop to serial port
        dont_take_temperatures = false;
        touch_reticles = false;
    }
}

Timer starterTimer(5000000);  // If remotely-started starting event is left on for this long, end it automatically  
Timer panicTimer(20000000);  // How long should a panic stop last?  We can't stay mad forever

void starter_update () {  // Starter bidirectional handler logic.  Outside code interacts with handler by setting starter_request = req_off, req_on, or req_tog
    if (starter_signal_support) {
        if (starter_request == req_tog) starter_request = (req)(!starter_drive);  // translate toggle request to a drive request opposite to the current drive state
        if (starter_drive && ((starter_request == req_off) || starterTimer.expired())) {  // If we're driving the motor but need to stop
            starter_drive = false;
            set_pin (starter_pin, INPUT_PULLDOWN);  // we never assert low on the pin, just set pin as input and let the pulldown bring it low
        }
        if (!starter_drive && (starter_request != req_on) && !sim.simulating(sens::starter)) {  // If we haven't been and shouldn't be driving, and not simulating
            do {
                starter = digitalRead(starter_pin);  // then read the pin, starter variable will store if starter is turned on externally
            } while (starter != digitalRead(starter_pin)); // starter pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
        }
        else if (!starter && (starter_request == req_on) && remote_start_support) {  // If we got a request to start the motor, and it's not already being driven externally
            starter_drive = true;
            starter = HIGH;
            set_pin (starter_pin, OUTPUT);  // then set pin to an output
            write_pin (starter_pin, starter);  // and start the motor
            starterTimer.reset();  // if left on the starter will turn off automatically after X seconds
        }
        starter_request = req_na;  // we have serviced whatever requests
    }
    else starter = LOW;
}
void ignition_panic_update() {  // Run once each main loop, directly before panicstop_update()
    if (panicstop_request == req_tog) panicstop_request = (req)(!panicstop);
    if (ignition_request == req_tog) ignition_request = (req)(!ignition);
    // else if (ignition_request == ignition) ignition_request = req_na;  // With this line, it ignores requests to go to state it's already in, i.e. won't do unnecessary pin write
    if (speedo.car_stopped() || panicTimer.expired()) panicstop_request = req_off;  // Cancel panic stop if car is stopped
    if (!speedo.car_stopped()) {
        if (ignition && ignition_request == req_off) panicstop_request = req_on;  // ignition cut causes panic stop
        if (!sim.simulating(sens::joy) && hotrc.radiolost()) panicstop_request = req_on;
    }
    bool paniclast = panicstop;
    if (panicstop_request != req_na) {
        panicstop = (bool)panicstop_request;
        if (panicstop && !paniclast) panicTimer.reset();
    }
    panicstop_request = req_na;
    if (panicstop) ignition_request = req_off;  // panic stop causes ignition cut
    if (ignition_request != req_na) {
        ignition = (bool)ignition_request;
        write_pin (ignition_pin, ignition);  // Turn car off or on (ign output is active high), ensuring to never turn on the ignition while panicking
    }
    ignition_request = req_na;  // Make sure this goes after the last comparison
}
void basicsw_update() {
    if (!sim.simulating(sens::basicsw)) {  // Basic Mode switch
        do {
            basicmodesw = !digitalRead(basicmodesw_pin);   // !value because electrical signal is active low
        } while (basicmodesw != !digitalRead(basicmodesw_pin)); // basicmodesw pin has a tiny (70ns) window in which it could get invalid low values, so read it twice to be sure
    }
}
void set_syspower(bool setting) {
    syspower = setting;
    if (keep_system_powered) syspower = HIGH;
    write_pin(syspower_pin, syspower);
}
bool boot_button_last = 0;
bool boot_button = 0;
bool boot_button_timer_active = false;
bool boot_button_suppress_click = false;
bool boot_button_action = NONE;
Timer boot_button_timer(400000);
//
void bootbutton_update() {
    // ESP32 "boot" button. generates boot_button_action flags of LONG or SHORT presses which can be handled wherever. Handler must reset boot_button_action = NONE
    if (button_pin < 0) return;
    // if (boot_button_action == SHORT) {
    //     syspower_request = req_on;
    //     boot_button_action == NONE;
    // }
    if (!read_pin (button_pin)) {
        if (!boot_button) {  // If press just occurred
            boot_button_timer.reset();  // Looks like someone just pushed the esp32 "boot" button
            boot_button_timer_active = true;  // flag to indicate timing for a possible long press
        }
        else if (boot_button_timer_active && boot_button_timer.expired()) {
            boot_button_action = LONG;  // Set flag to handle the long press event. Note, routine handling press should clear this
            boot_button_timer_active = false;  // Clear timer active flag
            boot_button_suppress_click = true;  // Prevents the switch release after a long press from causing a short press
        }
        boot_button = true;  // Store press is in effect
    }
    else {  // if button is not being pressed
        if (boot_button && !boot_button_suppress_click) boot_button_action = SHORT;  // if the button was just released, a short press occurred, which must be handled
        // else boot_button_action = NONE;  // This would auto-reset the button action flag but require it get handled in this loop. Otherwise the handler must set this
        boot_button_timer_active = false;  // Clear timer active flag
        boot_button = false;  // Store press is not in effect
        boot_button_suppress_click = false;  // End click suppression
    }
}
// Loop timing related
uint32_t looptime_linefeed_threshold = 0;  // Leaves prints of loops taking > this for analysis. Set to 0 prints every loop
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
//
void looptime_init() {  // Run once at end of setup()
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
// Diag / trouble codes
uint32_t err_timeout_us = 175000;
Timer errTimer((int64_t)err_timeout_us);
uint32_t err_margin_adc = 5;
// Sensor related trouble - this all should be moved to devices.h
enum err_type { LOST, RANGE, CALIB, WARN, CRIT, INFO, num_err_types };
enum err_sens { e_hrcvert, e_hrcch3, e_pressure, e_brkpos, e_speedo, e_hrchorz, e_tach, e_temps, e_starter, e_hrcch4, e_basicsw, e_mulebatt, e_lipobatt, e_airvelo, e_mapsens, e_num_sensors, e_none };  // these are in order of priority
char err_type_card[num_err_types][5] = { "Lost", "Rang", "Cal", "Warn", "Crit", "Info" };
char err_sensor_card[e_num_sensors+1][7] = { "HrcV", "HrcCh3", "BrPres", "BrkPos", "Speedo", "HrcH", "Tach", "Temps", "Startr", "HrcCh4", "Basic", "MulBat", "LiPo", "Airflw", "MAP", "None" };
// enum class sensor : opt_t { none=0, joy, pressure, brkpos, speedo, tach, airvelo, mapsens, engtemp, mulebatt, ignition, basicsw, cruisesw, starter, syspower };  // , num_sensors, err_flag };
bool err_sensor_alarm[num_err_types] = { false, false, false, false, false, false };
int8_t err_sensor_fails[num_err_types] = { 0, 0, 0, 0, 0, 0 };
bool err_sensor[num_err_types][e_num_sensors]; //  [LOST/RANGE] [e_hrchorz/e_hrcvert/e_hrcch3/e_hrcch4/e_pressure/e_brkpos/e_tach/e_speedo/e_airvelo/e_mapsens/e_temps/e_mulebatt/e_lipobatt/e_basicsw/e_starter]   // sens::opt_t::num_sensors]
uint8_t highest_pri_failing_sensor[num_err_types];
uint8_t highest_pri_failing_last[num_err_types];
bool diag_ign_error_enabled = true;

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
        for (int cat = 0; cat < num_temp_categories; cat++) temp_err[cat] = false;  // first reset
        for (int loc = 0; loc < tempsens.locint(loc::num_locations); loc++) {
            if (!tempsens.detected(loc)) not_detected = true;
            else if (tempsens.val(loc) >= temp_lims_f[tempsens.errclass(loc)][WARNING]) temp_err[tempsens.errclass(loc)] = true;
        }
        err_sensor[LOST][e_temps] = not_detected;

        // Detect sensors disconnected or giving out-of-range readings.
        // TODO : The logic of this for each sensor should be moved to devices.h objects
        err_sensor[RANGE][e_brkpos] = (brakepos.in() < brakepos.op_min_in() || brakepos.in() > brakepos.op_max_in());
        err_sensor[LOST][e_brkpos] = (brakepos.raw() < err_margin_adc);
        err_sensor[RANGE][e_pressure] = (pressure.psi() < pressure.op_min_psi() || pressure.psi() > pressure.op_max_psi());
        err_sensor[LOST][e_pressure] = (pressure.raw() < err_margin_adc);
        err_sensor[RANGE][e_mulebatt] = (mulebatt.v() < mulebatt.op_min_v() || mulebatt.v() > mulebatt.op_max_v());
        for (int32_t ch = horz; ch <= ch4; ch++) {  // Hack: This loop depends on the indices for hotrc channel enums matching indices of hotrc sensor errors
            err_sensor[RANGE][ch] = !hotrc.radiolost() && ((hotrc.us[ch][raw] < hotrc.us[ch][opmin] - (hotrc.us[ch][margin] >> 1)) 
                                    || (hotrc.us[ch][raw] > hotrc.us[ch][opmax] + (hotrc.us[ch][margin] >> 1)));  // && ch != vert
            err_sensor[LOST][ch] = !hotrc.radiolost() && ((hotrc.us[ch][raw] < (hotrc.absmin_us - hotrc.us[ch][margin]))
                                    || (hotrc.us[ch][raw] > (hotrc.absmax_us + hotrc.us[ch][margin])));
        }
        // err_sensor[RANGE][e_hrcvert] = (hotrc.us[vert][raw] < hotrc.failsafe_us - hotrc.us[ch][margin])
        //     || ((hotrc.us[vert][raw] < hotrc.us[vert][opmin] - halfmargin) && (hotrc.us[vert][raw] > hotrc.failsafe_us + hotrc.us[ch][margin]));
        
        // Set sensor error idiot light flags
        // printf ("Sensors errors: ");
        
        // printf ("Sensor check: ");
        for (int32_t t=LOST; t<=RANGE; t++) {
            highest_pri_failing_sensor[t] = e_none;
            err_sensor_alarm[t] = false;
            err_sensor_fails[t] = 0;
            for (int32_t s=0; s<e_num_sensors; s++)
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
        for (int32_t s=0; s<=e_num_sensors; s++) {
            if (s == e_num_sensors) s++;
            if (err_sensor[t][s]) printf ("%s, ", err_sensor_card[s]);
        }
        printf("\n");
    }
}
int16_t touch_pt[4] = { 160, 120, 2230, 2130 };

// Neopixel stuff
int32_t neobright = 10;  // lets us dim/brighten the neopixels
int32_t neodesat = 0;  // lets us de/saturate the neopixels

void neo_setup() {
    neo.init((uint8_t)neopixel_pin, running_on_devboard, 1);
    // neo.init((uint8_t)neopixel_pin, !running_on_devboard);
    neo.setbright(neobright);
    neo.setdesaturation(neodesat);
    neo.heartbeat(neopixel_pin >= 0);
}
void enable_flashdemo(bool ena) {
    if (ena) {
        neo.setflash(4, 8, 8, 8, 20, -1);  // brightness toggle in a continuous squarewave
        neo.setflash(5, 3, 1, 2, 85);      // three super-quick bright white flashes
        neo.setflash(6, 2, 5, 5, 0, 0);    // two short black pulses
    }
    else {
        neo.setflash(4, 0);
        neo.setflash(5, 0);
        neo.setflash(6, 0);
    }
}
// Calculates massairflow in g/s using values passed in if present, otherwise it reads fresh values
float massairflow(float _map = NAN, float _airvelo = NAN, float _ambient = NAN) {  // mdot (kg/s) = density (kg/m3) * v (m/s) * A (m2) .  And density = P/RT.  So,   mdot = v * A * P / (R * T)  in kg/s
    float temp = _ambient;
    if (std::isnan(_ambient)) {
        temp = tempsens.val(loc::ambient);
        if (std::isnan(temp) && running_on_devboard) temp = tempsens.val(loc::engine);
        if (std::isnan(temp)) return -1;  // Avoid crashing due to trying to read absent sensor
    }
    float T = 0.556 * (temp - 32.0) + 273.15;  // in K.  This converts from degF to K
    float R = 287.1;  // R (for air) in J/(kg·K) ( equivalent to 8.314 J/(mol·K) )  1 J = 1 kg*m2/s2
    float v = 0.447 * (std::isnan(_airvelo) ? airvelo.filt() : _airvelo); // in m/s   1609.34 m/mi * 1/3600 hr/s = 0.447
    float Ain2 = 3.1415926;  // in in2    1.0^2 in2 * pi  // will still need to divide by 1550 in2/m2
    float P = 6894.76 * (std::isnan(_map) ? mapsens.filt() : _map);  // in Pa   6894.76 Pa/PSI  1 Pa = 1 J/m3
    return v * Ain2 * P * 1000.0 / (R * T * 1550);  // mass air flow in grams per second (ug/s)   (1k g/kg * m/s * in2 * J/m3) / (J/(kg*K) * K * 1550 in2/m2) = g/s
}
float maf_gps;  // Manifold mass airflow in grams per second
float maf_min_gps = 0.0;
float maf_max_gps = massairflow(mapsens.max_psi(), airvelo.max_mph(), temp_lims_f[AMBIENT][DISP_MIN]);

#define RUN_TESTS 0
#if RUN_TESTS
    #include "unittests.h"
    void run_tests() {
        printf("Running tests...\n");
        delay(5000);
        test_Param();
        printf("Tests complete.\n");
        for(;;) {} // loop forever
    }
#else
    void run_tests() {}
#endif
