#pragma once
#include "Arduino.h"
#include <esp_task_wdt.h>
#include <iostream>
#include <iomanip>  // For formatting console loop timing string output
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
    AirVeloSensor* airvelo;
    MAPSensor* mapsens;
    Potentiometer* pot;
    float* maf;
    bool* ignition;
    static constexpr int entries = 100;  // size of log buffers
    int64_t times[2][entries];
    // two sets of large arrays for storage of log data. when one fills up it jumps to the other, so the first might be written to an sd card
    float tel[2][NumTelemetryFull][entries];  // array for telemetry of all sensors for given timestamp
    bool bools[2][NumTelemetryBool][entries];  // boolean control values
    int index = 0, dic = 0;  // start with dictionary 0
    Timer logTimer = Timer(100000);  // microseconds per logged reading
    Timer errTimer = Timer(175000);
  public:
    // diag tunable values
    uint32_t err_margin_adc = 5;
    char err_type_card[NUM_ERR_TYPES][5] = { "Lost", "Rang", "Cal", "Warn", "Crit", "Info" };
    char err_sens_card[NumTelemetryFull+2][7] = { 
        "Throtl", "BkMotr", "Steer", "HotRC", "Speedo", "Tach", "BkPres", "BkPosn", "Temps", "Other", "GPIO", 
        "AirVel", "MAP", "Batery", "Pot", "MAF", "HrcHrz", "HrcVrt", "TmpEng", "TmpWFL", "TmpWFR", "TmpWRL", "TmpWRR", "TmpAmb",
        "NA", "None"
    };

    bool diag_ign_error_enabled = true;
    // diag non-tunable values
    bool temp_err[NUM_TEMP_CATEGORIES];  // [AMBIENT/ENGINE/WHEEL]
    bool err_sens_alarm[NUM_ERR_TYPES] = { false, false, false, false, false, false };
    int8_t err_sens_fails[NUM_ERR_TYPES] = { 0, 0, 0, 0, 0, 0 };
    bool err_sens[NUM_ERR_TYPES][NumTelemetryFull]; //  [LOST/RANGE] [_HotRCHorz/_HotRCVert/_HotRCCh3/_HotRCCh4/_Pressure/_BrkPos/_Tach/_Speedo/_AirVelo/_MAP/_TempEng/_MuleBatt/_BasicSw/_Starter]   // sens::opt_t::NUM_SENSORS]
    uint8_t most_critical_sensor[NUM_ERR_TYPES];
    uint8_t most_critical_last[NUM_ERR_TYPES];
    DiagRuntime (Hotrc* a_hotrc, TemperatureSensorManager* a_temp, PressureSensor* a_pressure, BrakePositionSensor* a_brkpos,
        Tachometer* a_tach, Speedometer* a_speedo, GasServo* a_gas, BrakeMotor* a_brake, SteerMotor* a_steer, 
        CarBattery* a_mulebatt, AirVeloSensor* a_airvelo, MAPSensor* a_mapsens, Potentiometer* a_pot, float* a_maf, bool* a_ignition)
        : hotrc(a_hotrc), tempsens(a_temp), pressure(a_pressure), brkpos(a_brkpos), tach(a_tach), speedo(a_speedo), gas(a_gas), brake(a_brake), 
          steer(a_steer), mulebatt(a_mulebatt), airvelo(a_airvelo), mapsens(a_mapsens), pot(a_pot), maf(a_maf), ignition(a_ignition) {}

    void setup() {
        for (int32_t i=0; i<NUM_ERR_TYPES; i++)
            for (int32_t j=0; j<NumTelemetryFull; j++)
                err_sens[i][j] = false; // Initialize sensor error flags to false
    }
    void make_log_entry() {
        if (logTimer.expireset()) {
            times[dic][index] = esp_timer_get_time();
            tel[dic][_GasServo][index] = gas->pc[OUT];
            tel[dic][_BrakeMotor][index] = brake->pc[OUT];
            tel[dic][_SteerMotor][index] = steer->pc[OUT];
            tel[dic][_BrakePres][index] = pressure->filt();
            tel[dic][_BrakePosn][index] = brkpos->filt();
            tel[dic][_Speedo][index] = speedo->filt();
            tel[dic][_Tach][index] = tach->filt();
            // tel[dic][_HotRCHorz][index] = hotrc->pc[HORZ][FILT];
            // tel[dic][_HotRCVert][index] = hotrc->pc[VERT][FILT];
            // tel[dic][_MuleBatt][index] = mulebatt->filt();
            // tel[dic][_AirVelo][index] = airvelo->filt(); 
            // tel[dic][_MAP][index] = mapsens->filt();
            // tel[dic][_MAF][index] = *maf;
            // tel[dic][_Pot][index] = pot->val();
            // bools[dic][_Ignition][index] = *ignition;
            // tel[dic][_TempEng][index] = 
            // tel[dic][_TempWhFL][index] = 
            // tel[dic][_TempWhFR][index] = 
            // tel[dic][_TempWhRL][index] = 
            // tel[dic][_TempWhRR][index] = 
            // tel[dic][_TempAmb][index] = 
            ++index %= entries;
            // printf(".");
            if (!index) {
                dic = !dic;
                // printf("Filled dic %d\n", dic);
            }
        }
    }
    void set_sensidiots() {
        for (int err=0; err<=_GPIO; err++) {
            sensidiots[err] = false;
            for (int typ=LOST; typ<=VALUE; typ++) {
                if (err == _HotRC)
                    for (int ch = HORZ; ch <= CH4; ch++) 
                        sensidiots[err] = sensidiots[err] || err_sens[typ][ch];
                else if (err == _Temps)
                    for (int sens = _TempEng; sens <= _TempAmb; sens++)
                        sensidiots[err] = sensidiots[err] || err_sens[typ][sens];
                else if (err == _Other) {
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_MuleBatt];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_AirVelo];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_MAP];
                }
                else if (err == _GPIO) {
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_Ignition];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_SysPower];
                }
                else sensidiots[err] = err_sens[typ][err];
            }
        }
    }
    void update() {
        if (errTimer.expireset()) {
            // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
            // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
            // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
            // retreive with an OBD tool. Some checks are below, along with other possible things to check for:
            bool not_detected = false;  // first reset
            for (int cat = 0; cat < NUM_TEMP_CATEGORIES; cat++) temp_err[cat] = false;  // first reset
            for (int l = 0; l < tempsens->locint(); l++) {
                if (!tempsens->detected(l)) not_detected = true;
                else if (tempsens->val(l) >= temp_lims_f[tempsens->errclass(l)][WARNING]) temp_err[tempsens->errclass(l)] = true;
            }
            err_sens[LOST][_TempEng] = not_detected;

            // Detect sensors disconnected or giving out-of-range readings.
            // TODO : The logic of this for each sensor should be moved to devices.h objects
            err_sens[RANGE][_BrakeMotor] = (brake->pc[OUT] < brake->pc[OPMIN] || brake->pc[OUT] > brake->pc[OPMAX]);
            err_sens[RANGE][_GasServo] = (gas->pc[OUT] < gas->pc[OPMIN] || gas->pc[OUT] > gas->pc[OPMAX]);
            err_sens[RANGE][_BrakeMotor] = (steer->pc[OUT] < steer->pc[OPMIN] || steer->pc[OUT] > steer->pc[OPMAX]);
            err_sens[RANGE][_BrakePosn] = (brkpos->in() < brkpos->op_min() || brkpos->in() > brkpos->op_max());
            err_sens[LOST][_BrakePosn] = (brkpos->raw() < err_margin_adc);
            err_sens[RANGE][_BrakePres] = (pressure->psi() < pressure->op_min() || pressure->psi() > pressure->op_max());
            err_sens[LOST][_BrakePres] = (pressure->raw() < err_margin_adc);
            err_sens[RANGE][_MuleBatt] = (mulebatt->v() < mulebatt->op_min_v() || mulebatt->v() > mulebatt->op_max_v());
            for (int32_t ch = HORZ; ch <= CH4; ch++) {  // Hack: This loop depends on the indices for hotrc channel enums matching indices of hotrc sensor errors
                err_sens[RANGE][ch] = !hotrc->radiolost() && ((hotrc->us[ch][RAW] < hotrc->us[ch][OPMIN] - (hotrc->us[ch][MARGIN] >> 1)) 
                                        || (hotrc->us[ch][RAW] > hotrc->us[ch][OPMAX] + (hotrc->us[ch][MARGIN] >> 1)));  // && ch != VERT
                err_sens[LOST][ch] = !hotrc->radiolost() && ((hotrc->us[ch][RAW] < (hotrc->absmin_us - hotrc->us[ch][MARGIN]))
                                        || (hotrc->us[ch][RAW] > (hotrc->absmax_us + hotrc->us[ch][MARGIN])));
            }
            err_sens[VALUE][_Ignition] = (!ignition && !tach->engine_stopped());
            // err_sens[VALUE][_SysPower] = (!syspower && (run.mode != ASLEEP));
            set_sensidiots();

            // err_sens[RANGE][_HotRCVert] = (hotrc->us[VERT][RAW] < hotrc->failsafe_us - hotrc->us[ch][MARGIN])
            //     || ((hotrc->us[VERT][RAW] < hotrc->us[VERT][OPMIN] - halfMARGIN) && (hotrc->us[VERT][RAW] > hotrc->failsafe_us + hotrc->us[ch][MARGIN]));
            
            // Set sensor error idiot light flags
            // printf ("Sensors errors: ");
            
            // printf ("Sensor check: ");
            for (int32_t t=LOST; t<=RANGE; t++) {
                most_critical_sensor[t] = _None;
                err_sens_alarm[t] = false;
                err_sens_fails[t] = 0;
                for (int32_t s=0; s<NumTelemetryShort; s++)
                    if (err_sens[t][s]) {
                        if (most_critical_sensor[t] = _None) most_critical_sensor[t] = s;
                        err_sens_alarm[t] = true;
                        err_sens_fails[t]++;
                    }
            }
            // printf ("\n");
            make_log_entry();
        }
    }
    int worst_sensor(int type) {
        return most_critical_sensor[type];  // for global awareness
    }
    void print() {
        for (int32_t t=LOST; t<=INFO; t++) {
            printf ("diag err: %s (%d): ", err_type_card[t], err_sens_fails[t]);
            for (int32_t s=0; s<=NumTelemetryFull; s++) {
                if (s == NumTelemetryFull) s++;
                if (err_sens[t][s]) printf ("%s, ", err_sens_card[s]);
            }
            printf("\n");
        }
    }
};
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

// more notes on brake error detection ideas:
// 1. Detect  brake chain is not connected (evidenced by change in brake position without expected pressure changes)
// 2. Detect obstruction, motor failure, or inaccurate position. Evidenced by motor instructed to move but position not changing even when pressure is low.
// 3. Detet brake hydraulics failure or inaccurate pressure. Evidenced by normal positional change not causing expected increase in pressure.
// retract_effective_max_us = volt[STOP] + duty_pc * (volt[OPMAX] - volt[STOP]);  // Stores instantaneous calculated value of the effective maximum pulsewidth after attenuation

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
class Watchdog {
  private:
    int timeout_sec = 10;
    Preferences* myprefs;
    int codemode_last = 50000, crashcount = 0;
    std::string codemodecard[4] = { "confused", "booting", "parked", "driving" };
  public:
    Watchdog(Preferences* _prefs) : myprefs(_prefs) {}
    void bootcounter() {
        bootcount = myprefs->getUInt("bootcount", 0) + 1;
        myprefs->putUInt("bootcount", bootcount);
        codemode_postmortem = myprefs->getUInt("codemode", Confused);
        crashcount = myprefs->getUInt("crashcount", 0);
        if (codemode_postmortem != Parked) crashcount++;
        myprefs->putUInt("crashcount", crashcount);
        Serial.printf("Boot count: %d (%d/%d). Last lost power while %s\n", bootcount, bootcount-crashcount, crashcount, codemodecard[codemode_postmortem].c_str());
    }
    void set_codemode(int _mode) {
        codemode = _mode;
        if (codemode_last != codemode) myprefs->putUInt("codemode", codemode);
        codemode_last = codemode;
    }
    void setup(int sec = -1) {
        if (sec >= 0) timeout_sec = sec;
        bootcounter();
        if (!watchdog_enabled) return;
        Serial.printf("Watchdog timer.. \n");
        esp_task_wdt_init(timeout_sec, true);  // see https://github.com/espressif/esp-idf/blob/master/examples/system/task_watchdog/main/task_watchdog_example_main.c
        esp_task_wdt_add(NULL);
    }
    void add(TaskHandle_t taskh) {
        if (!watchdog_enabled) return;
        esp_task_wdt_add(taskh);
    }
    void pet() {
        if (!watchdog_enabled) return;
        esp_task_wdt_reset();
    }
};

#define RUN_TESTS 0  // set this to 0 for the car
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