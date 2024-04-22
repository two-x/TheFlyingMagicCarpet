#pragma once
#include "Arduino.h"
class DiagRuntime {
  private:
    bool report_error_changes = true;
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
    bool* ignition;
    static constexpr int entries = 100;  // size of log buffers
    int64_t times[2][entries];
    // two sets of large arrays for storage of log data. when one fills up it jumps to the other, so the first might be written to an sd card
    float tel[2][NumTelemetryFull][entries];  // array for telemetry of all sensors for given timestamp
    int index = 0, dic = 0, runmode;  // start with dictionary 0
    Timer logTimer{100000};  // microseconds per logged reading
    Timer errTimer{175000};
    Timer speedoTimer{2500000}, tachTimer{2500000};  // how much time within which we expect the car will move after hitting the gas
  public:
    // diag tunable values
    uint32_t err_margin_adc = 5;
    char err_type_card[NUM_ERR_TYPES][5] = { "Lost", "Rang" };  // this needs to match err_type enum   // , "Cal", "Warn", "Crit", "Info" };
    char err_sens_card[NumTelemetryFull+2][7] = {  // this needs to match telemetry_idiots and telemetry_full enums, with NA and None tacked on the end
        "Throtl", "BkMotr", "Steer", "HotRC", "Speedo", "Tach", "BkPres", "BkPosn", "Temps", "Other", "GPIO", 
        "HrcHrz", "HrcVrt", "HrcCh3", "HrcCh4", "Batery", "AirVel", "MAP", "Pot", "TmpEng", "TmpWFL", "TmpWFR",
        "TmpWRL", "TmpWRR", "TmpAmb", "Ign", "Start", "BasicS", "FuelP",
        "NA", "None"
    };

    // diag non-tunable values
    bool temp_err[NUM_TEMP_CATEGORIES];  // [AMBIENT/ENGINE/WHEEL]
    bool err_sens_alarm[NUM_ERR_TYPES] = { false, false };
    int8_t err_sens_fails[NUM_ERR_TYPES] = { 0, 0 };
    bool err_sens[NUM_ERR_TYPES][NumTelemetryFull]; //  [LOST/RANGE] [_HotRCHorz/_HotRCVert/_HotRCCh3/_HotRCCh4/_Pressure/_BrkPos/_Tach/_Speedo/_AirVelo/_MAP/_TempEng/_MuleBatt/_BasicSw/_Starter]   // sens::opt_t::NUM_SENSORS]
    bool err_last[NUM_ERR_TYPES][NumTelemetryFull]; //  [LOST/RANGE] [_HotRCHorz/_HotRCVert/_HotRCCh3/_HotRCCh4/_Pressure/_BrkPos/_Tach/_Speedo/_AirVelo/_MAP/_TempEng/_MuleBatt/_BasicSw/_Starter]   // sens::opt_t::NUM_SENSORS]
    uint8_t most_critical_sensor[NUM_ERR_TYPES];
    uint8_t most_critical_last[NUM_ERR_TYPES];
    DiagRuntime (Hotrc* a_hotrc, TemperatureSensorManager* a_temp, PressureSensor* a_pressure, BrakePositionSensor* a_brkpos,
        Tachometer* a_tach, Speedometer* a_speedo, GasServo* a_gas, BrakeMotor* a_brake, SteerMotor* a_steer, 
        CarBattery* a_mulebatt, AirVeloSensor* a_airvelo, MAPSensor* a_mapsens, Potentiometer* a_pot, bool* a_ignition)
        : hotrc(a_hotrc), tempsens(a_temp), pressure(a_pressure), brkpos(a_brkpos), tach(a_tach), speedo(a_speedo), gas(a_gas), brake(a_brake), 
          steer(a_steer), mulebatt(a_mulebatt), airvelo(a_airvelo), mapsens(a_mapsens), pot(a_pot), ignition(a_ignition) {}

    void setup() {
        for (int32_t i=0; i<NUM_ERR_TYPES; i++)
            for (int32_t j=0; j<NumTelemetryFull; j++)
                err_sens[i][j] = err_last[i][j] = false; // Initialize sensor error flags to false
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
            for (int typ=0; typ<=NUM_ERR_TYPES; typ++) {
                if (err == _HotRC)  // error with any hotrc channel triggers the hotrc idiot light
                    for (int ch = HORZ; ch <= CH4; ch++) 
                        sensidiots[err] = sensidiots[err] || err_sens[typ][ch];
                else if (err == _Temps)  // error with any temp sensor triggers the temp idiot light
                    for (int sens = _TempEng; sens <= _TempAmb; sens++)
                        sensidiots[err] = sensidiots[err] || err_sens[typ][sens];
                else if (err == _GPIO) {  // error with any digital signal triggers the GPIO idiot light
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_Ignition];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_BasicSw];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_FuelPump];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_Starter];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_FuelPump];
                }
                else if (err == _Other) {  // error with any other sensor not having its own idiot light triggers the "Other" (aka "ETC") idiot light
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_MuleBatt];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_AirVelo];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_MAP];
                    sensidiots[err] = sensidiots[err] || err_sens[typ][_Pot];
                }
                else sensidiots[err] = sensidiots[err] || err_sens[typ][err];
            }
        }
    }
    void update(int _runmode) {
        runmode = _runmode;
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
                int errindex;
                if (ch == HORZ) errindex = _HotRCHorz;
                else if (ch == VERT) errindex = _HotRCVert;
                else if (ch == CH3) errindex = _HotRCCh3;
                else if (ch == CH4) errindex = _HotRCCh4;
                err_sens[RANGE][errindex] = !hotrc->radiolost() && ((hotrc->us[ch][RAW] < hotrc->us[ch][OPMIN] - (hotrc->us[ch][MARGIN] >> 1)) 
                                        || (hotrc->us[ch][RAW] > hotrc->us[ch][OPMAX] + (hotrc->us[ch][MARGIN] >> 1)));  // && ch != VERT
                err_sens[LOST][errindex] = !hotrc->radiolost() && ((hotrc->us[ch][RAW] < (hotrc->absmin_us - hotrc->us[ch][MARGIN]))
                                        || (hotrc->us[ch][RAW] > (hotrc->absmax_us + hotrc->us[ch][MARGIN])));
            }
            err_sens[LOST][_Ignition] = (!ignition && !tach->engine_stopped());  // Not really "LOST", but lost isn't meaningful for ignition really anyway
            err_sens[LOST][_Speedo] = SpeedoFailure();
            err_sens[LOST][_Tach] = TachFailure();
            err_sens[RANGE][_Speedo] = (speedo->mph() < speedo->min_human() || speedo->mph() > speedo->max_human());;
            err_sens[RANGE][_Tach] = (tach->rpm() < tach->min_human() || tach->rpm() > tach->max_human());
            
            // err_sens[VALUE][_SysPower] = (!syspower && (run.mode != ASLEEP));
            set_sensidiots();

            // err_sens[RANGE][_HotRCVert] = (hotrc->us[VERT][RAW] < hotrc->failsafe_us - hotrc->us[ch][MARGIN])
            //     || ((hotrc->us[VERT][RAW] < hotrc->us[VERT][OPMIN] - halfMARGIN) && (hotrc->us[VERT][RAW] > hotrc->failsafe_us + hotrc->us[ch][MARGIN]));
            
            // Set sensor error idiot light flags
            // printf ("Sensors errors: ");
            
            set_idiot_blinks();
            // detect and report changes in any error values
            report_changes();

            for (int32_t i=0; i<NUM_ERR_TYPES; i++)
                for (int32_t j=0; j<NumTelemetryFull; j++)

            // printf ("\n");
            make_log_entry();
        }
    }
    void set_idiot_blinks() {  // adds blink code to lost and range err neopixels corresponing to the lowest numbered failing sensor
        for (int32_t t=LOST; t<=RANGE; t++) {
            most_critical_sensor[t] = _None;
            err_sens_alarm[t] = false;
            err_sens_fails[t] = 0;
            for (int32_t s=0; s<NumTelemetryIdiots; s++)
                if (err_sens[t][s]) {
                    if (most_critical_sensor[t] = _None) most_critical_sensor[t] = s;
                    err_sens_alarm[t] = true;
                    err_sens_fails[t]++;
                }
        }
    }
    void report_changes() {
        for (int32_t i=0; i<NUM_ERR_TYPES; i++) {
            for (int32_t j=0; j<NumTelemetryFull; j++) {
                if (report_error_changes) {
                    if (err_sens[i][j] && !err_last[i][j])
                        Serial.printf("!diag: %s %s err\n", err_sens_card[j], err_type_card[i]);
                    else if (!err_sens[i][j] && err_last[i][j])
                        Serial.printf("!diag: %s %s ok\n", err_sens_card[j], err_type_card[i]);
                }
                err_last[i][j] = err_sens[i][j];
            }
        }    
    }
    int worst_sensor(int type) {
        return most_critical_sensor[type];  // for global awareness
    }
    void print() {
        for (int32_t t=0; t<=NUM_ERR_TYPES; t++) {
            printf ("diag err: %s (%d): ", err_type_card[t], err_sens_fails[t]);
            for (int32_t s=0; s<=NumTelemetryFull; s++) {
                if (s == NumTelemetryFull) s++;
                if (err_sens[t][s]) printf ("%s, ", err_sens_card[s]);
            }
            printf("\n");
        }
    }
    bool SpeedoFailure() {  // checks if speedo isn't zero when stopped, or doesn't increase when we drive
        static bool gunning_it, gunning_last = true;
        static float baseline_speed;
        bool fail = false;
        if (runmode == SHUTDOWN) {  // || runmode == HOLD  // check that the speed is zero when stopped
            // if (gunning_last) speedoTimer.reset();       // if we just stopped driving, allow time for car to stop
            // else if (speedoTimer.expired()) {            // if it has been enough time since entering shutdown, we should be stopped
            fail = (baseline_speed > speedo->margin_mph());  // when stopped the speedo reading should be zero, otherwise fail
            baseline_speed = speedo->filt();         // store the speed value when we are stopped
            // }
        }
        gunning_it = (gas->pc[OUT] > 20.0 && (runmode == FLY || runmode == CRUISE));
        if (gunning_it) {                                                             // if we're attempting to drive
            if (!gunning_last) speedoTimer.reset();                     // delay our speed comparison so car can accelerate
            else if (speedoTimer.expired()) fail = (speedo->filt() - baseline_speed < speedo->margin_mph());  // the car should be moving by now
        }
        gunning_last = gunning_it;
        return fail;
    }
    bool TachFailure() {  // checks if tach isn't low when throttle is released, or doesn't increase when we gun it
        static bool running_it, running_last = true;
        static float baseline_rpm;
        bool fail = false;
        if (runmode == SHUTDOWN) {  // || runmode == STALL  // check that the speed is zero when stopped
            // if (running_last) tachTimer.reset();       // if we just stopped driving, allow time for car to stop
            // else if (tachTimer.expired()) {            // if it has been enough time since entering shutdown, we should be stopped
            fail = (baseline_rpm > tach->margin_rpm());  // when stopped the speedo reading should be zero, otherwise fail
            baseline_rpm = tach->filt();         // store the speed value when we are stopped
            // }
        }
        running_it = (gas->pc[OUT] > 20.0 && (runmode == FLY || runmode == CRUISE));
        if (running_it) {                                               // if we're attempting to drive
            if (!running_last) tachTimer.reset();                     // delay our rpm comparison so car can respond
            else if (tachTimer.expired()) fail = (tach->filt() - baseline_rpm < tach->margin_rpm());  // the car should be moving by now
        }
        running_last = running_it;
        return fail;
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


// Moved sdcard code into diag.h to reduce file count. these includes go with it
#include <iostream>
#include <string>
#include <iomanip>
#include <SD.h>
#include <FFat.h>
#include <LovyanGFX.hpp>

#define FORMAT_FFAT true  // You only need to format FFat the first time you use a specific card

class SdCard {
  private:
    // LGFX* lcd;
    Timer capture_timer{5000000};
    int capcount = 0, capmax = 100;
    char filenamebase[11] = "/screencap";
    char extension[5] = ".bmp";
    #ifndef SDCARD_SPI
        #define SDCARD_SPI SPI
    #endif
    std::string filename_seq(int num) {
        std::string writefile = filenamebase + std::string(2 - std::to_string(num).length(), '0'); // Add leading zeros
        writefile += std::to_string(num) + extension;
        return writefile;
    }
    std::string saveToSD(int num) {
        std::string filename = filename_seq(num);
        bool result = false;
        File file = SD.open(filename.c_str(), "w");
        if (file) {
            int width = 320; //  = lcd->width();
            int height = 240; // = lcd->height();
            int rowSize = (2 * width + 3) & ~ 3;  // int rowSize = (3 * width + 3) & ~ 3;  // 24-bit version
            lgfx::bitmap_header_t bmpheader;
            bmpheader.bfType = 0x4D42;
            bmpheader.bfSize = rowSize * height + sizeof(bmpheader);
            bmpheader.bfOffBits = sizeof(bmpheader);
            bmpheader.biSize = 40;
            bmpheader.biWidth = width;
            bmpheader.biHeight = height;
            bmpheader.biPlanes = 1;
            bmpheader.biBitCount = 16;  // bmpheader.biBitCount = 24;  // 24-bit version
            bmpheader.biCompression = 3;  // bmpheader.biCompression = 0;  // 24-bit version
            file.write((std::uint8_t*)&bmpheader, sizeof(bmpheader));
            std::uint8_t buffer[rowSize];
            memset(&buffer[rowSize - 4], 0, 4);
            for (int y = 240 - 1; y >= 0; y--) {
                // lcd->readRect(0, y, lcd->width(), 1, (lgfx::rgb565_t*)buffer);
                // // lcd->readRect(0, y, lcd->width(), 1, (lgfx::rgb888_t*)buffer);  // 24 bit color version
                file.write(buffer, rowSize);
            }
            file.close();
            result = true;
        }
        else Serial.print("error:file open failure\n");
        return filename;
    }
    void draw_bmp_file(void) {
        ++capcount %= capmax;
        std::string wfile = filename_seq(capcount);

        // lcd->drawBmpFile(SD, wfile.c_str(), random(-20,20), random(-20, 20));  // drawBmpFile(fs, path, x, y, maxWidth, maxHeight, offX, offY, scale_x, scale_y, datum);

        Serial.printf("wrote: %s\n", wfile.c_str());
    }
  public:
    // SdCard(LGFX* _lcd\\) : lcd(_lcd) {}
    SdCard() {}

    void setup(void) {
        // lcd->init();
        // Serial.begin(115200);
        // lcd->setColorDepth(16);
        // lcd->setColor(TFT_WHITE);

        // lcd->startWrite();
        // lcd->setAddrWindow(0, 0, lcd->width(), lcd->height());
        // for (int y = 0; y < lcd->height(); ++y) 
        //     for (int x = 0; x < lcd->width(); ++x) 
        //         lcd->writeColor( lcd->color888(x << 1, x + y, y << 1), 1);
        // Serial.printf("BMP save test\n");
        // lcd->endWrite();
        
        // for (int i=0; i<capmax; i++) {
        // int i = 0;
        // std::string filename = filename_seq(i);
        // do {
        //     SD.end();
        //     delay(1000);
        //     SD.begin(sdcard_cs_pin, SDCARD_SPI, 25000000);
        // } while (!saveToSD(filename));
        // Serial.printf("BMP save %s success\n", filename.c_str());
        
        // lcd->setAddrWindow(0, 0, 320, 240);

    }

    void update() {
        if (capture_timer.expireset()) {
            ++capcount %= capmax;
            std::string wfile = saveToSD(capcount);
            Serial.printf("wrote: %s\n", wfile.c_str());
        }
    }
};
class FatFs {
  public:
    // This file should be compiled with 'Partition Scheme' (in Tools menu)
    // set to 'Default with ffat' if you have a 4MB ESP32 dev module or
    // set to '16M Fat' if you have a 16MB ESP32 dev module.
    FatFs() {}
    void setup() {
        // format: allocation unit size must be a power of 2, w/ min=sector_size, max=128*sector_size. setting to 0 will result in allocation unit set to the sector size. larger is faster but with more wasted space when files are small
        // sector size is always 512 B. For wear levelling, sector size is determined by CONFIG_WL_SECTOR_SIZE option of esp_vfs_fat_mount_config_t.
        Serial.setDebugOutput(true);
        if (FORMAT_FFAT) FFat.format();
        if (!FFat.begin()){
            Serial.println("fatfs mount failed\n");
            return;
        }
        Serial.println("fatfs mounted on /sd\n");
    }
    void fattest() {
        Serial.printf("Total space: %10u\n", FFat.totalBytes());
        Serial.printf("Free space: %10u\n", FFat.freeBytes());
        listDir(FFat, "/", 0);
        writeFile(FFat, "/hello.txt", "Hello ");
        appendFile(FFat, "/hello.txt", "World!\r\n");
        readFile(FFat, "/hello.txt");
        renameFile(FFat, "/hello.txt", "/foo.txt");
        readFile(FFat, "/foo.txt");
        deleteFile(FFat, "/foo.txt");
        testFileIO(FFat, "/test.txt");
        Serial.printf("Free space: %10u\n", FFat.freeBytes());
        deleteFile(FFat, "/test.txt");
        Serial.println( "Test complete" );
    }
    void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
        Serial.printf("Listing directory: %s\r\n", dirname);

        File root = fs.open(dirname);
        if (!root){
            Serial.println("- failed to open directory");
            return;
        }
        if (!root.isDirectory()){
            Serial.println(" - not a directory");
            return;
        }

        File file = root.openNextFile();
        while(file){
            if (file.isDirectory()){
                Serial.print("  DIR : ");
                Serial.println(file.name());
                if (levels) listDir(fs, file.path(), levels -1);
            }
            else {
                Serial.print("  FILE: ");
                Serial.print(file.name());
                Serial.print("\tSIZE: ");
                Serial.println(file.size());
            }
            file = root.openNextFile();
        }
    }

    void readFile(fs::FS &fs, const char * path) {
        Serial.printf("Reading file: %s\r\n", path);

        File file = fs.open(path);
        if (!file || file.isDirectory()){
            Serial.println("- failed to open file for reading");
            return;
        }

        Serial.println("- read from file:");
        while (file.available()) Serial.write(file.read());
        file.close();
    }

    void writeFile(fs::FS &fs, const char * path, const char * message) {
        Serial.printf("Writing file: %s\r\n", path);

        File file = fs.open(path, FILE_WRITE);
        if (!file){
            Serial.println("- failed to open file for writing");
            return;
        }
        if (file.print(message)) Serial.println("- file written");
        else Serial.println("- write failed");
        file.close();
    }

    void appendFile(fs::FS &fs, const char * path, const char * message) {
        Serial.printf("Appending to file: %s\r\n", path);

        File file = fs.open(path, FILE_APPEND);
        if (!file){
            Serial.println("- failed to open file for appending");
            return;
        }
        if (file.print(message)) Serial.println("- message appended");
        else Serial.println("- append failed");
        file.close();
    }
    void renameFile(fs::FS &fs, const char * path1, const char * path2) {
        Serial.printf("Renaming file %s to %s\r\n", path1, path2);
        if (fs.rename(path1, path2)) Serial.println("- file renamed");
        else Serial.println("- rename failed");
    }
    void deleteFile(fs::FS &fs, const char * path) {
        Serial.printf("Deleting file: %s\r\n", path);
        if (fs.remove(path)) Serial.println("- file deleted");
        else Serial.println("- delete failed");
    }
    void testFileIO(fs::FS &fs, const char * path) {
        Serial.printf("Testing file I/O with %s\r\n", path);

        static uint8_t buf[512];
        size_t len = 0;
        File file = fs.open(path, FILE_WRITE);
        if (!file) {
            Serial.println("- failed to open file for writing");
            return;
        }
        size_t i;
        Serial.print("- writing");
        uint32_t start = millis();
        for (i=0; i<2048; i++) {
            if ((i & 0x001F) == 0x001F) Serial.print(".");
            file.write(buf, 512);
        }
        Serial.println("");
        uint32_t end = millis() - start;
        Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
        file.close();

        file = fs.open(path);
        start = millis();
        end = start;
        i = 0;
        if (file && !file.isDirectory()) {
            len = file.size();
            size_t flen = len;
            start = millis();
            Serial.print("- reading" );
            while (len){
                size_t toRead = len;
                if (toRead > 512) toRead = 512;
                file.read(buf, toRead);
                if ((i++ & 0x001F) == 0x001F) Serial.print(".");
                len -= toRead;
            }
            Serial.println("");
            end = millis() - start;
            Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
            file.close();
        }
        else Serial.println("- failed to open file for reading");
    }
};
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