#pragma once
#include "Arduino.h"
#include <iostream>
#include <iomanip>  // For formatting console loop timing string output
#include <vector>  // used to group loop times with string labels
enum err_type : int { LOST, RANGE, CALIB, WARN, CRIT, INFO, NUM_ERR_TYPES };
enum err_sens : int {  // these are in order of priority 
    e_hrcvert, e_hrcch3, e_pressure, e_brkpos, e_speedo, 
    e_hrchorz, e_tach, e_temps, e_starter, e_hrcch4, 
    e_basicsw, e_mulebatt, e_airvelo, e_mapsens, E_NUM_SENSORS,
    e_none 
};
class DiagRuntime {
  private:
    Hotrc* hotrc;
    TemperatureSensorManager* tempsens;
    PressureSensor* pressure;
    BrakePositionSensor* brkpos;
    Tachometer* tach;
    Speedometer* speedo;
    GasServo* gas;
    BrakeMotor* brake;
    SteerMotor* steer;
    CarBattery* mulebatt;
    float* maf;
    bool* ignition;
  public:
    enum telemetry_dictionary_float : int { 
        _GasServo, _BrakeMotor, _SteerMotor, _HotRCHorz, _HotRCVert,
        _Pressure, _BrakePos, _Speedo, _Tach,  _MuleBatt,
        _TempEng, _TempWhFL, _TempWhFR, _TempWhRL, _TempWhRR,
        _TempAmb, _AirVelo, _MAP, _MAF, NumTelemetryFloats
    };
    enum telemetry_dictionary_bool : int { 
        _Ignition, _PanicStop, _SysPower, _HotRCCh3, _StarterDr, 
        _StarterExt, _HotRCCh4, _BasicSw, NumTelemetryBools
    };
    DiagRuntime (Hotrc* a_hotrc, TemperatureSensorManager* a_temp, PressureSensor* a_pressure, BrakePositionSensor* a_brkpos,
        Tachometer* a_tach, Speedometer* a_speedo, GasServo* a_gas, BrakeMotor* a_brake, SteerMotor* a_steer, 
        CarBattery* a_mulebatt, float* a_maf, bool* a_ignition)
        : hotrc(a_hotrc), tempsens(a_temp), pressure(a_pressure), brkpos(a_brkpos), tach(a_tach), speedo(a_speedo), gas(a_gas),
          brake(a_brake), steer(a_steer), mulebatt(a_mulebatt), maf(a_maf), ignition(a_ignition) {}
    // diag tunable values
    uint32_t err_timeout_us = 175000;
    uint32_t err_margin_adc = 5;
    char err_type_card[NUM_ERR_TYPES][5] = { "Lost", "Rang", "Cal", "Warn", "Crit", "Info" };
    char err_sens_card[E_NUM_SENSORS+1][7] = { "HrcV", "HrcCh3", "BrPres", "BrkPos", "Speedo", "HrcH", "Tach", "Temps", "Startr", "HrcCh4", "Basic", "MulBat", "Airflw", "MAP", "None" };
    bool diag_ign_error_enabled = true;
    // diag non-tunable values
    bool temp_err[NUM_TEMP_CATEGORIES];  // [AMBIENT/ENGINE/WHEEL]
    Timer errTimer;
    bool err_sens_alarm[NUM_ERR_TYPES] = { false, false, false, false, false, false };
    int8_t err_sens_fails[NUM_ERR_TYPES] = { 0, 0, 0, 0, 0, 0 };
    bool err_sens[NUM_ERR_TYPES][E_NUM_SENSORS]; //  [LOST/RANGE] [e_hrchorz/e_hrcvert/e_hrcch3/e_hrcch4/e_pressure/e_brkpos/e_tach/e_speedo/e_airvelo/e_mapsens/e_temps/e_mulebatt/e_basicsw/e_starter]   // sens::opt_t::NUM_SENSORS]
    uint8_t highest_pri_failing_sensor[NUM_ERR_TYPES];
    uint8_t highest_pri_failing_last[NUM_ERR_TYPES];
    
    void setup() {
        for (int32_t i=0; i<NUM_ERR_TYPES; i++)
            for (int32_t j=0; j<E_NUM_SENSORS; j++)
                err_sens[i][j] = false; // Initialize sensor error flags to false
        errTimer.set(err_timeout_us);
    }
    void update() {
        if (errTimer.expireset()) {
            // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
            // this is one approach
            // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
            // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
            // retreive with an OBD tool. Some checks are below, along with other possible things to check for:
            if (!ignition && !tach->engine_stopped()) {  // Check: if engine is turning when ignition signal is off
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
            for (int l = 0; l < tempsens->locint(); l++) {
                if (!tempsens->detected(l)) not_detected = true;
                else if (tempsens->val(l) >= temp_lims_f[tempsens->errclass(l)][WARNING]) temp_err[tempsens->errclass(l)] = true;
            }
            err_sens[LOST][e_temps] = not_detected;

            // Detect sensors disconnected or giving out-of-range readings.
            // TODO : The logic of this for each sensor should be moved to devices.h objects
            err_sens[RANGE][e_brkpos] = (brkpos->in() < brkpos->op_min_in() || brkpos->in() > brkpos->op_max_in());
            err_sens[LOST][e_brkpos] = (brkpos->raw() < err_margin_adc);
            err_sens[RANGE][e_pressure] = (pressure->psi() < pressure->op_min_psi() || pressure->psi() > pressure->op_max_psi());
            err_sens[LOST][e_pressure] = (pressure->raw() < err_margin_adc);
            err_sens[RANGE][e_mulebatt] = (mulebatt->v() < mulebatt->op_min_v() || mulebatt->v() > mulebatt->op_max_v());
            for (int32_t ch = HORZ; ch <= CH4; ch++) {  // Hack: This loop depends on the indices for hotrc channel enums matching indices of hotrc sensor errors
                err_sens[RANGE][ch] = !hotrc->radiolost() && ((hotrc->us[ch][RAW] < hotrc->us[ch][OPMIN] - (hotrc->us[ch][MARGIN] >> 1)) 
                                        || (hotrc->us[ch][RAW] > hotrc->us[ch][OPMAX] + (hotrc->us[ch][MARGIN] >> 1)));  // && ch != VERT
                err_sens[LOST][ch] = !hotrc->radiolost() && ((hotrc->us[ch][RAW] < (hotrc->absmin_us - hotrc->us[ch][MARGIN]))
                                        || (hotrc->us[ch][RAW] > (hotrc->absmax_us + hotrc->us[ch][MARGIN])));
            }
            // err_sens[RANGE][e_hrcvert] = (hotrc->us[VERT][RAW] < hotrc->failsafe_us - hotrc->us[ch][MARGIN])
            //     || ((hotrc->us[VERT][RAW] < hotrc->us[VERT][OPMIN] - halfMARGIN) && (hotrc->us[VERT][RAW] > hotrc->failsafe_us + hotrc->us[ch][MARGIN]));
            
            // Set sensor error idiot light flags
            // printf ("Sensors errors: ");
            
            // printf ("Sensor check: ");
            for (int32_t t=LOST; t<=RANGE; t++) {
                highest_pri_failing_sensor[t] = e_none;
                err_sens_alarm[t] = false;
                err_sens_fails[t] = 0;
                for (int32_t s=0; s<E_NUM_SENSORS; s++)
                    if (err_sens[t][s]) {
                        if (highest_pri_failing_sensor[t] = e_none) highest_pri_failing_sensor[t] = s;
                        err_sens_alarm[t] = true;
                        err_sens_fails[t]++;
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
    void print() {
        for (int32_t t=LOST; t<=INFO; t++) {
            printf ("diag err: %s (%d): ", err_type_card[t], err_sens_fails[t]);
            for (int32_t s=0; s<=E_NUM_SENSORS; s++) {
                if (s == E_NUM_SENSORS) s++;
                if (err_sens[t][s]) printf ("%s, ", err_sens_card[s]);
            }
            printf("\n");
        }
    }
};
class LoopTimer {
  public:
    LoopTimer() {}
    // Loop timing related
    Timer loop_timer;
    static constexpr uint32_t looptimeout = 1000000; // how long the previous main loop took to run (in us)
    int32_t loopno = 1, loopindex = 0, loop_recentsum = 0, loop_scale_min_us = 0, loop_scale_avg_max_us = 2500, loop_scale_peak_max_us = 25000;
    float loop_sum_s, loop_avg_us, loopfreq_hz;
    uint32_t looptimes_us[20];
    bool loop_dirty[20];
    int64_t loop_cout_mark_us;
    uint32_t loop_cout_us = 0, loop_peak_us = 0, loop_now = 0;;
    static constexpr uint32_t loop_history = 100;
    uint32_t loop_periods_us[loop_history];
    // std::vector<std::string> loop_names(20);
    std::string loop_names[20];
    void setup() {  // Run once at end of setup()
        if (looptime_print) {
            for (int32_t x=1; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
            loop_names[0] = std::string("top");
            loop_dirty[0] = false;
            loopindex = 1;
            looptimes_us[0] = esp_timer_get_time();
        }
        loop_timer.set(looptimeout);  // start timer to measure the first loop
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
};
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