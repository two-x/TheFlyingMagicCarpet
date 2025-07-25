#pragma once
#include "Arduino.h"
#include <esp_task_wdt.h>
#include <esp_partition.h>
#include <iostream>
#include <iomanip>  // For formatting console loop timing string output

class DiagRuntime {
  private:
    bool report_error_changes = true, first_boot = true;
    Hotrc* hotrc;
    TemperatureSensorManager* tempsens;
    PressureSensor* pressure;
    BrakePositionSensor* brkpos;
    Tachometer* tach;
    Speedometer* speedo;
    ThrottleControl* gas;
    BrakeControl* brake;
    SteeringControl* steer;
    CarBattery* mulebatt;
    AirVeloSensor* airvelo;
    MAPSensor* mapsens;
    Potentiometer* pot;
    Ignition* ignition;
    static constexpr int entries = 100;  // size of log buffers
    int64_t times[2][entries];
    // two sets of large arrays for storage of log data. when one fills up it jumps to the other, so the first might be written to an sd card
    float tel[2][NumTelemetryFull][entries];  // array for telemetry of all sensors for given timestamp
    int index = 0, dic = 0, total_registered = 0, runmode;  // start with dictionary 0
    Timer logTimer{100000};  // microseconds per logged reading
    Timer errTimer{175000};
    Timer speedoTimer{2500000}, tachTimer{2500000};  // how much time within which we expect the car will move after hitting the gas
  public:
    // diag tunable values
    int err_margin_adc = 5;
    uint errstatus[NumErrTypes] = { 0x00, 0x00, 0x00 };  // keeps current error status in efficient hex words
    std::string err_type_card[NumErrTypes] = { "Lost", "Rang", "Warn" };  // this needs to match err_type enum   // , "Lost", "Range", "Warn"
    std::string err_sens_card[NumTelemetryFull+3] = {  // this needs to match telemetry_idiots and telemetry_full enums, with NA, None, and Hybrid tacked on the end.  access these names using ascii_name() function
        "Throtl", "BkMotr", "Steer", "Speedo", "Tach", "BkPres", "BkPosn", "HotRC",
        "Temps", "Other", "GPIO", "HrcHrz", "HrcVrt", "HrcCh3", "HrcCh4", "Batery",
        "AirVel", "MAP", "Pot", "TmpEng", "TmpWFL", "TmpWFR", "TmpWRL", "TmpWRR",
        "TmpBrk", "TmpAmb", "Ign", "Start", "BasicS", "TmpWhl",
        "NA", "None", "Hybrid",
    };
    // "Throtl", "BkMotr", "Steer", "Speedo", "Tach", "BkPres", "BkPosn", "HrcHrz", "HrcVrt", "HrcCh3",
    // "HrcCh4", "Batery", "AirVel", "MAP", "Pot", "TmpEng", "TmpWFL", "TmpWFR", "TmpWRL", "TmpWRR",
    // "TmpBrk", "TmpAmb", "Ign", "Start", "BasicS",
    // "NA", "None", "Hybrid",
    std::string ascii_name(int sensor) {
        if (sensor == _NA) return err_sens_card[NumTelemetryFull];
        if (sensor == _None) return err_sens_card[NumTelemetryFull + 1];
        if (sensor == _Hybrid) return err_sens_card[NumTelemetryFull + 2];
        return err_sens_card[NumTelemetryFull];        
    }
    // diag non-tunable values
    // bool temp_err[NumTempCategories];  // [CatAmbient/CatEngine/CatWheel/CatBrake]
    bool err_sens_alarm[NumErrTypes] = { false, false, false, };  //  [ErrLost/ErrRange/ErrWarn]
    int err_sens_fails[NumErrTypes] = { 0, 0, 0, };
    bool err_sens[NumErrTypes][NumTelemetryFull]; //  [ErrLost/ErrRange/ErrWarn] [_HotRCHorz/_HotRCVert/_HotRCCh3/_HotRCCh4/_Pressure/_BrkPos/_Tach/_Speedo/_AirVelo/_MAP/_TempEng/_MuleBatt/_BasicSw/_Starter]   // sens::opt_t::NUM_SENSORS]
    bool err_last[NumErrTypes][NumTelemetryFull]; //  [ErrLost/ErrRange/ErrWarn] [_HotRCHorz/_HotRCVert/_HotRCCh3/_HotRCCh4/_Pressure/_BrkPos/_Tach/_Speedo/_AirVelo/_MAP/_TempEng/_MuleBatt/_BasicSw/_Starter]   // sens::opt_t::NUM_SENSORS]
    // Device* devices[NumTelemetryFull];
    int registered_errcount[NumErrTypes];
    float* devices[NumTelemetryFull][NumDiagVals];  //
    bool registered[NumTelemetryFull];
    float violating_value[NumTelemetryFull];
    uint8_t most_critical_sensor[NumErrTypes];
    uint8_t most_critical_last[NumErrTypes];
    DiagRuntime (Hotrc* a_hotrc, TemperatureSensorManager* a_temp, PressureSensor* a_pressure, BrakePositionSensor* a_brkpos,
        Tachometer* a_tach, Speedometer* a_speedo, ThrottleControl* a_gas, BrakeControl* a_brake, SteeringControl* a_steer, 
        CarBattery* a_mulebatt, AirVeloSensor* a_airvelo, MAPSensor* a_mapsens, Potentiometer* a_pot, Ignition* a_ignition)
        : hotrc(a_hotrc), tempsens(a_temp), pressure(a_pressure), brkpos(a_brkpos), tach(a_tach), speedo(a_speedo), gas(a_gas), brake(a_brake), 
          steer(a_steer), mulebatt(a_mulebatt), airvelo(a_airvelo), mapsens(a_mapsens), pot(a_pot), ignition(a_ignition) {}
    void setup() {
        ezread.squintf("Diagnostic engine:");
        for (int i=0; i<NumErrTypes; i++)
            for (int j=0; j<NumTelemetryFull; j++) {
                err_sens[i][j] = err_last[i][j] = false; // Initialize sensor error flags to false
            }
        for (int j=0; j<NumTelemetryFull; j++) registered[j] = false;
        register_device(_Throttle, &gas->si[Out], &gas->si[Parked], &gas->si[OpMax], &gas->si[Margin]);
        register_device(_BrakeMotor, &brake->si[Out], &brake->si[OpMin], &brake->si[OpMax], &brake->si[Margin]);
        register_device(_SteerMotor, &steer->si[Out], &steer->si[OpMin], &steer->si[OpMax], &steer->si[Margin]);
        register_device(_Speedo, speedo->ptr(), speedo->opmin_ptr(), speedo->opmax_ptr(), speedo->margin_ptr());
        register_device(_Tach, tach->ptr(), tach->opmin_ptr(), tach->opmax_ptr(), tach->margin_ptr());
        register_device(_BrakePres, pressure->ptr(), pressure->opmin_ptr(), pressure->opmax_ptr(), pressure->margin_ptr());
        register_device(_BrakePosn, brkpos->ptr(), brkpos->opmin_ptr(), brkpos->opmax_ptr(), brkpos->margin_ptr());
        register_device(_HotRCHorz, &hotrc->us[Horz][Filt], &hotrc->us[Horz][OpMin], &hotrc->us[Horz][OpMax], &hotrc->us[Horz][Margin]);
        register_device(_HotRCVert, &hotrc->us[Vert][Filt], &hotrc->us[Vert][OpMin], &hotrc->us[Vert][OpMax], &hotrc->us[Vert][Margin]);
        register_device(_HotRCCh3, &hotrc->us[Ch3][Raw], &hotrc->us[Ch3][OpMin], &hotrc->us[Ch3][OpMax], &hotrc->us[Ch3][Margin]);
        register_device(_HotRCCh4, &hotrc->us[Ch4][Raw], &hotrc->us[Ch4][OpMin], &hotrc->us[Ch4][OpMax], &hotrc->us[Ch4][Margin]);
        register_device(_MuleBatt, mulebatt->ptr(), mulebatt->opmin_ptr(), mulebatt->opmax_ptr(), mulebatt->margin_ptr());
        register_device(_AirVelo, airvelo->ptr(), airvelo->opmin_ptr(), airvelo->opmax_ptr(), airvelo->margin_ptr());
        register_device(_MAP, mapsens->ptr(), mapsens->opmin_ptr(), mapsens->opmax_ptr(), mapsens->margin_ptr());
        register_device(_Pot, pot->ptr(), pot->opmin_ptr(), pot->opmax_ptr(), pot->margin_ptr());
        register_device(_TempEng, tempsens->ptr(loc::TempEngine), tempsens->opmin_ptr(loc::TempEngine), tempsens->opmax_ptr(loc::TempEngine), tempsens->margin_ptr(loc::TempEngine));
        register_device(_TempAmb, tempsens->ptr(loc::TempAmbient), tempsens->opmin_ptr(loc::TempAmbient), tempsens->opmax_ptr(loc::TempAmbient), tempsens->margin_ptr(loc::TempAmbient));
        register_device(_TempBrake, tempsens->ptr(loc::TempBrake), tempsens->opmin_ptr(loc::TempBrake), tempsens->opmax_ptr(loc::TempBrake), tempsens->margin_ptr(loc::TempBrake));
        register_device(_TempWhFL, tempsens->ptr(loc::TempWheelFL), tempsens->opmin_ptr(loc::TempWheelFL), tempsens->opmax_ptr(loc::TempWheelFL), tempsens->margin_ptr(loc::TempWheelFL));
        register_device(_TempWhFR, tempsens->ptr(loc::TempWheelFR), tempsens->opmin_ptr(loc::TempWheelFR), tempsens->opmax_ptr(loc::TempWheelFR), tempsens->margin_ptr(loc::TempWheelFR));
        register_device(_TempWhRL, tempsens->ptr(loc::TempWheelRL), tempsens->opmin_ptr(loc::TempWheelRL), tempsens->opmax_ptr(loc::TempWheelRL), tempsens->margin_ptr(loc::TempWheelRL));
        register_device(_TempWhRR, tempsens->ptr(loc::TempWheelRR), tempsens->opmin_ptr(loc::TempWheelRR), tempsens->opmax_ptr(loc::TempWheelRR), tempsens->margin_ptr(loc::TempWheelRR));
        // register_bool_device(_Ignition, ignition->signal_ptr());
        // register_bool_device(_Starter, starter->signal_ptr());
        ezread.squintf(" monitoring %d devices\n", total_registered);
    }
    void update() {
        if (first_boot) {  // don't run too soon before sensors get initialized etc.
            first_boot = false;
            errTimer.reset();
            return;
        }
        if (runmode == LowPower) return;
        if (errTimer.expireset()) {
            // Auto-Diagnostic  :   Check for worrisome oddities and dubious circumstances. Report any suspicious findings
            // This section should become a real time self-diagnostic system, to look for anything that doesn't seem right and display an
            // informed trouble code. Much like the engine computer in cars nowadays, which keep track of any detectable failures for you to
            // retreive with an OBD tool. Some checks are below, along with other possible things to check for:
            // TODO : The logic of this for each sensor should be moved to devices.h objects
            TempFailure();

            checkrange(_Pot);
            checkrange(_Throttle);
            checkrange(_SteerMotor);
            // setflag(_Throttle, ErrRange, gas->pc[Out] < gas->pc[Parked] || gas->pc[Out] > gas->pc[OpMax]);
            // setflag(_SteerMotor, ErrRange, steer->pc[Out] < steer->pc[OpMin] || steer->pc[Out] > steer->pc[OpMax]);
            // setflag(_Ignition, ErrLost, !ignition->signal && !tach->stopped());  // Not really "ErrLost", but lost isn't meaningful for ignition really anyway
            BatteryFailure();            
            BrakeFailure();            
            HotRCFailure();
            SpeedoFailure();
            TachFailure();
            // err_sens[VALUE][_SysPower] = (!syspower && (run.mode != LowPower));
            set_sensorgroups();
            set_sensidiots();

            count_errors();
            set_idiot_blinks();
            report_changes();  // detect and report changes in any error values
            dump_errorcode_update();
            // for (int i=0; i<NumErrTypes; i++)
            //     for (int j=0; j<NumTelemetryFull; j++)
            // // ezread.squintf ("\n");
            // make_log_entry();
        }
    }
    int errorcount(int errtype) { return registered_errcount[errtype]; }
    bool battrangeerr;
  private:
    void register_device(int _enumname, float* _value, float* _min, float* _max, float* _margin) {  // registers devices that are children of ServoMotor class
        devices[_enumname][DiagVal] = _value;
        devices[_enumname][DiagMin] = _min;
        devices[_enumname][DiagMax] = _max;
        devices[_enumname][DiagMargin] = _margin;
        registered[_enumname] = true;
        total_registered++;
    }
    void update_status_words() {}
    void setflag(int device, int errtype, bool stat) {  // this sets the error flag in err_sens[type] array, and if device is registered, also in the status words
        err_sens[errtype][device] = stat;
        errstatus[errtype] = (errstatus[errtype] & ~(1 << device)) | ((stat && registered[device]) << device);  // replaces the device's error bit in status word with updated value
    }
    void checkrange(int _sens, bool extra_condition = true) {
        bool last_rangerr = err_sens[ErrRange][_sens];
        bool rangerr = extra_condition && ((*devices[_sens][DiagVal] < *devices[_sens][DiagMin] - *devices[_sens][DiagMargin]) || (*devices[_sens][DiagVal] > *devices[_sens][DiagMax] + *devices[_sens][DiagMargin]));
        if (rangerr != last_rangerr) {
            setflag(_sens, ErrRange, rangerr);
            violating_value[_sens] = *devices[_sens][DiagVal];  
        }
    }
    void set_sensorgroups() {
        for (int typ=0; typ<NumErrTypes; typ++) {
            setflag(_HotRC, typ, err_sens[typ][_HotRCHorz] || err_sens[typ][_HotRCVert] || err_sens[typ][_HotRCCh3] || err_sens[typ][_HotRCCh4]);
            setflag(_GPIO, typ, err_sens[typ][_Ignition] || err_sens[typ][_BasicSw] || err_sens[typ][_Starter]);
            setflag(_Other, typ, err_sens[typ][_AirVelo] || err_sens[typ][_MAP] || err_sens[typ][_Pot]);  // || err_sens[typ][_MuleBatt]
            setflag(_TempWheel, typ, err_sens[typ][_TempWhFL] || err_sens[typ][_TempWhFR] || err_sens[typ][_TempWhRL] || err_sens[typ][_TempWhRR]);
            setflag(_Temps, typ, err_sens[typ][_TempEng] || err_sens[typ][_TempWheel] || err_sens[typ][_TempBrake] || err_sens[typ][_TempAmb]);
        }
        // adding this workaround for the mulebatt errors, which should have been included in the _Other group above,
        // but as you can see above I had to comment it out b/c of the confounding fact that err_sens[ErrRange][_MuleBatt]
        // for some reason has the same mem address as the global nowtouch flag (!!)
        setflag(_Other, ErrWarn, err_sens[ErrWarn][_Other] || err_sens[ErrWarn][_MuleBatt]);
        setflag(_Other, ErrLost, err_sens[ErrLost][_Other] || err_sens[ErrLost][_MuleBatt]);
        setflag(_Other, ErrRange, err_sens[ErrRange][_Other] || battrangeerr);
        // end of cheesy workaround
    }
    void set_sensidiots() {
        for (int err=0; err<=_GPIO; err++) {
            sensidiots[err] = false;
            for (int typ=0; typ<NumErrTypes; typ++) sensidiots[err] |= err_sens[typ][err];
        }
    }
    void count_errors() {
        for (int t=ErrLost; t<=ErrWarn; t++) {
            registered_errcount[t] = 0;
            for (int j=0; j<NumTelemetryFull; j++)
                if (registered[j] && err_sens[t][j])
                    registered_errcount[t]++;
        }
    }
    void set_idiot_blinks() {  // adds blink code to lost and range err neopixels corresponing to the lowest numbered failing sensor
        for (int t=ErrLost; t<=ErrRange; t++) {
            most_critical_sensor[t] = _None;
            err_sens_alarm[t] = false;
            err_sens_fails[t] = 0;
            for (int s=0; s<NumTelemetryIdiots; s++)
                if (err_sens[t][s]) {
                    if (most_critical_sensor[t] == _None) most_critical_sensor[t] = s;
                    err_sens_alarm[t] = true;
                    err_sens_fails[t]++;
                }
        }
    }
    void report_changes() {
        if (!print_error_changes) return;
        for (int i=0; i<NumErrTypes; i++) {
            bool printheader1 = false, printheader2 = false, printed = false;
            for (int j=0; j<NumTelemetryFull; j++) {
                if (report_error_changes && (err_sens[i][j] && !err_last[i][j]) && registered[j]) {
                    if (!printheader1) {
                        ezread.squintf("!%s:", err_type_card[i].c_str());
                        printheader1 = true;
                    }
                    Serial.printf(" %s (%.2lf)", err_sens_card[j].c_str(), violating_value[j]);
                    ezread.printf(" %s (%.0lf)", err_sens_card[j].c_str(), violating_value[j]);
                    printed = true;
                }
            }
            for (int j=0; j<NumTelemetryFull; j++) {
                if (report_error_changes && (!err_sens[i][j] && err_last[i][j]) && registered[j]) {
                    if (!printheader2) {
                        if (!printheader1) ezread.squintf("!%s:", err_type_card[i].c_str());
                        ezread.squintf(" OK:");
                        printheader2 = true;
                    }
                    Serial.printf(" %s (%.2lf)", err_sens_card[j].c_str(), violating_value[j]);
                    ezread.printf(" %s (%.0lf)", err_sens_card[j].c_str(), violating_value[j]);
                    printed = true;
                }
            }
            for (int j=0; j<NumTelemetryFull; j++) err_last[i][j] = err_sens[i][j];
            if (printed) ezread.squintf("\n");
        }    
    }
    int worst_sensor(int type) {
        return most_critical_sensor[type];  // for global awareness
    }
    void BatteryFailure() {
        bool last_rangerr = battrangeerr;
        int _sens = _MuleBatt;
        battrangeerr = ((*devices[_sens][DiagVal] < *devices[_sens][DiagMin] - *devices[_sens][DiagMargin]) || (*devices[_sens][DiagVal] > *devices[_sens][DiagMax] + *devices[_sens][DiagMargin]));
        if (battrangeerr != last_rangerr) {
            violating_value[_sens] = *devices[_sens][DiagVal];  
        }
        // checkrange(_MuleBatt);
    }
    void TempFailure() {
        static bool printed_error_wheel = false, printed_error_eng = false, printed_error_brake = false;
        for (int sen=_TempEng; sen<=_TempAmb; sen++) {
            checkrange(sen);
            // bypassing ErrLost check because it's giving false positive for all 7 sensors on the car, even tho they are reading fine. FIX!
            // setflag(i, ErrLost, !tempsens->detected(i));
        }
        bool wheel_err_range = false, wheel_err_other = false;
        int minwheel = 400, maxwheel = -400;
        for (int sen=_TempWhFL; sen<=_TempWhRR; sen++) {
            if (*devices[sen][DiagVal] < minwheel) minwheel = *devices[sen][DiagVal];
            if (*devices[sen][DiagVal] > maxwheel) maxwheel = *devices[sen][DiagVal];
            if (err_sens[ErrRange][sen]) wheel_err_range = true;
            for (int typ=0; typ<NumErrTypes; typ++)
                if ((typ != ErrRange) && err_sens[typ][sen]) wheel_err_other = true;
            
        }
        wheeltemperr = wheel_err_range || wheel_err_other || ((maxwheel - minwheel) > wheeldifferr);  // set global wheel temp idiot light to include all wheel temp errors
        if (overtemp_shutoff_wheel) {
            if (wheel_err_range) {
                ignition->request(ReqOff);  // ignition->panic_request(ReqOff);  // not sure if this was intentional? commenting out
                if (!printed_error_wheel) ezread.squintf(RED, "err: wheel temp out of range. stop engine\n");
                printed_error_wheel = true;
            }
            else printed_error_wheel = false;
        }
        if (overtemp_shutoff_engine) {
            if (err_sens[ErrRange][_TempEng]) {
                ignition->request(ReqOff);
                if (!printed_error_eng) ezread.squintf(RED, "err: engine temp out of range. stop engine\n");
                printed_error_eng = true;
            }
            else printed_error_eng = false;
        }
        if (overtemp_shutoff_brake) {  // here the engine is shut off. also in brake motor class the brake motor is stopped
            if (err_sens[ErrRange][_TempBrake]) {
                // brake->setmode(Halt);        // stop the brake actuator  // commented b/c is done in brake class
                ignition->request(ReqOff);  // kill the engine
                if (!printed_error_brake) ezread.squintf(RED, "err: brakemotor temp out of range. stop engine\n");
                printed_error_brake = true;
            }
            else printed_error_brake = false;
        }
    }
    // Brakes:   checks if any sensor the brake is expecting to use in its current mode are posting errors
    void BrakeFailure() {  // checks if posn is not changing while pressure is changing and motor is moving (assuming not near max brake)
        static float pressure_last_pc;
        static float brkpos_last_pc;
        static float motor_last_pc;
        if ((std::abs(brake->pc[Out]) > brake->pc[Margin]) && (std::abs(motor_last_pc) > brake->pc[Margin]) && (signbit(brake->pc[Out]) == signbit(motor_last_pc))) {  // if brake motor is moving
            if (pressure->pc() <= 80.0) {  // if not near the max pressure (where position changes very little)
                setflag(_BrakePosn, ErrLost, ((std::abs(pressure->pc() - pressure_last_pc) > pressure->margin_pc()) && (std::abs(brkpos->pc() - brkpos_last_pc) < brkpos->margin_pc())));  // if pressure value is changing but position isn't, then set flag otherwise clear
                
                // Skipping to avoid annoying warnings. check this!
                // setflag(_BrakePosn, ErrWarn, (brake->feedback == HybridFB) && (signbit(brkpos->pc()) != signbit(pressure->pc())) && (signbit(brkpos->pc()) != signbit(brake->pc[Out])));  // if motor and pressure are consistent with moving one direction but position is changing the opposite way  // rewmove the (brake->feedback == HybridFB) && term?
            }
            if (brkpos->pc() >= 20.0) {  // if not near the min braking full extension (where pressure changes very little) 
                setflag(_BrakePres, ErrLost, ((std::abs(brkpos->pc() - brkpos_last_pc) > brkpos->margin_pc()) && (std::abs(pressure->pc() - pressure_last_pc) < pressure->margin_pc())));  // if position value is changing but pressure isn't, then set flag otherwise clear
                setflag(_BrakePres, ErrWarn, (brake->feedback == HybridFB) && (signbit(pressure->pc()) != signbit(brkpos->pc())) && (signbit(pressure->pc()) != signbit(brake->pc[Out])));  // if motor and position are consistent with moving one direction but pressure is changing the opposite way
            }
            
            // Skipping this b/c it is triggering and I don't care. Really should figure out what's going on
            // setflag(_BrakeMotor, ErrLost, ((std::abs(pressure->pc() - pressure_last_pc) < pressure->margin_pc()) && (std::abs(brkpos->pc() - brkpos_last_pc) < brkpos->margin_pc())));  // if neither sensor is changing, set motor lost flag
        }
        checkrange(_BrakeMotor);
        checkrange(_BrakePosn);
        checkrange(_BrakePres);
        // setflag(_BrakeMotor, ErrRange, brake->pc[Out] < brake->pc[OpMin] - brake->pc[Margin] || brake->pc[Out] > brake->pc[OpMax] + brake->pc[Margin]);
        // setflag(_BrakePosn, ErrRange, (brkpos->pc() > 100.0 + brkpos->margin_pc()) || (brkpos->pc() < 0.0 - brkpos->margin_pc()));  // if position reading is outside oprange, set flag
        // setflag(_BrakePres, ErrRange, (pressure->pc() > 100.0 + pressure->margin_pc()) || (pressure->pc() < 0.0 - pressure->margin_pc()));  // if pressure reading is outside oprange, set flag
        bool found_err = false;
        if (brake->feedback_enabled[PressureFB] && (err_sens[ErrLost][_BrakePosn] || err_sens[ErrRange][_BrakePosn])) found_err = true;
        if (brake->feedback_enabled[PositionFB] && (err_sens[ErrLost][_BrakePres] || err_sens[ErrRange][_BrakePres])) found_err = true;
        setflag(_BrakeMotor, ErrWarn, found_err);
        pressure_last_pc = pressure->pc();
        brkpos_last_pc = brkpos->pc();
        motor_last_pc = brake->pc[Out];
    }
    void SpeedoFailure() {  // checks if speedo isn't zero when stopped, or doesn't increase when we drive
        static bool gunning_it, gunning_last = true;
        static float baseline_speed;
        bool fail = false;
        // commented this b/c 
        // if (runmode == Standby) {  // || runmode == Hold  // check that the speed is zero when stopped
        //     // if (gunning_last) speedoTimer.reset();       // if we just stopped driving, allow time for car to stop
        //     // else if (speedoTimer.expired()) {            // if it has been enough time since entering standby, we should be stopped
        //     fail = (baseline_speed > speedo->margin());  // when stopped the speedo reading should be zero, otherwise fail
        //     baseline_speed = speedo->val();         // store the speed value when we are stopped
        //     // }
        // }
        gunning_it = (gas->pc[Out] > 20.0 && (runmode == Fly || runmode == Cruise));
        if (gunning_it) {                                                             // if we're attempting to drive
            if (!gunning_last) speedoTimer.reset();                     // delay our speed comparison so car can accelerate
            else if (speedoTimer.expired()) fail = (speedo->val() - baseline_speed < speedo->margin());  // the car should be moving by now
        }
        gunning_last = gunning_it;
        setflag(_Speedo, ErrLost, fail);
        checkrange(_Speedo);
        // setflag(_Speedo, ErrRange, speedo->val() < speedo->opmin() || speedo->val() > speedo->opmax());
    }
    void TachFailure() {  // checks if tach isn't low when throttle is released, or doesn't increase when we gun it
        static bool running_it, running_last = true;
        static float baseline_rpm;
        bool fail = false;
        if (runmode == Standby) {  // || runmode == Stall  // check that the speed is zero when stopped
            // if (running_last) tachTimer.reset();       // if we just stopped driving, allow time for car to stop
            // else if (tachTimer.expired()) {            // if it has been enough time since entering standby, we should be stopped
            fail = (baseline_rpm > tach->margin());  // when stopped the speedo reading should be zero, otherwise fail
            baseline_rpm = tach->val();         // store the speed value when we are stopped
            // }
        }
        running_it = (gas->pc[Out] > 20.0 && (runmode == Fly || runmode == Cruise));
        if (running_it) {                                               // if we're attempting to drive
            if (!running_last) tachTimer.reset();                     // delay our rpm comparison so car can respond
            else if (tachTimer.expired()) fail = (tach->val() - baseline_rpm < tach->margin());  // the car should be moving by now
        }
        running_last = running_it;
        setflag(_Tach, ErrLost, fail);
        checkrange(_Tach);
        // setflag(_Tach, ErrRange, tach->val() < tach->opmin() || tach->val() > tach->opmax());
    }
    void HotRCFailure() {
        for (int ch = Horz; ch <= Ch4; ch++) {
            int errindex;
            if (ch == Horz)      { errindex = _HotRCHorz; checkrange(_HotRCHorz, !hotrc->radiolost()); }
            else if (ch == Vert) { errindex = _HotRCVert; checkrange(_HotRCVert, !hotrc->radiolost()); }
            else if (ch == Ch3)  { errindex = _HotRCCh3;  checkrange(_HotRCCh3, !hotrc->radiolost());  }
            else if (ch == Ch4)  { errindex = _HotRCCh4;  checkrange(_HotRCCh4, !hotrc->radiolost());  }
            // setflag(errindex, ErrRange, !hotrc->radiolost() && ((hotrc->us[ch][Filt] < hotrc->us[ch][OpMin] - hotrc->us[ch][Margin]) 
            //                         || (hotrc->us[ch][Filt] > hotrc->us[ch][OpMax] + hotrc->us[ch][Margin])));  // && ch != Vert
            setflag(errindex, ErrLost, !hotrc->radiolost() && ((hotrc->us[ch][Filt] < hotrc->absmin_us - hotrc->us[ch][Margin])
                                    || (hotrc->us[ch][Filt] > hotrc->absmax_us + hotrc->us[ch][Margin])));
            // err_sens[ErrRange][_HotRCVert] = (hotrc->us[Vert][RAW] < hotrc->failsafe_us - hotrc->us[ch][Margin])
            //     || ((hotrc->us[Vert][RAW] < hotrc->us[Vert][OpMin] - halfMargin) && (hotrc->us[Vert][RAW] > hotrc->failsafe_us + hotrc->us[ch][Margin]));
        }
    }
    void dump_errorcode_update() {
        if (!print_error_changes) return;
        static uint32_t status_last[NumErrTypes];
        bool do_print = false;
        uint8_t color = NON;
        uint32_t now, was;
        int errdiffs = 0, errtotal = 0;
        for (int e=0; e<NumErrTypes; e++) {
            if (errstatus[e] != status_last[e]) do_print = true;
            now = errstatus[e];
            was = status_last[e];
            while (now || was) {
                if (now & 1) {
                    errtotal++;
                    if (!(was & 1)) {
                        errdiffs++;  // there's a new error
                        if (color == ezread.happycolor) color = ezread.defaultcolor;  // but also another error got cleared, so use default color
                        else if (color == NON) color = ezread.sadcolor;  // use the bad news color
                    }
                }
                else if ((now & 1) < (was & 1)) {
                    errdiffs--;  // an error got cleared
                    if (color == ezread.sadcolor) color = ezread.defaultcolor;  // but there was also a new error, so use default color
                    else if (color == NON) color = ezread.happycolor;  // use the good news color
                }
                now >>= 1;
                was >>= 1;
            }
            status_last[e] = errstatus[e];
        }
        if (color == NON) {
            color = ezread.defaultcolor;
            if (!errdiffs) do_print = false;
        }
        if (do_print) {
            ezread.squintf(color, "errs %d(%+d) L%x R%x W%x\n", errtotal, errdiffs, errstatus[ErrLost], errstatus[ErrRange], errstatus[ErrWarn]);
        }
    }
    void make_log_entry() {
        if (logTimer.expireset()) {
            times[dic][index] = esp_timer_get_time();
            tel[dic][_Throttle][index] = gas->pc[Out];
            tel[dic][_BrakeMotor][index] = brake->pc[Out];
            tel[dic][_SteerMotor][index] = steer->pc[Out];
            tel[dic][_BrakePres][index] = pressure->val();
            tel[dic][_BrakePosn][index] = brkpos->val();
            tel[dic][_Speedo][index] = speedo->val();
            tel[dic][_Tach][index] = tach->val();
            // tel[dic][_HotRCHorz][index] = hotrc->pc[Horz][Filt];
            // tel[dic][_HotRCVert][index] = hotrc->pc[Vert][Filt];
            // tel[dic][_MuleBatt][index] = mulebatt->val();
            // tel[dic][_AirVelo][index] = airvelo->val(); 
            // tel[dic][_MAP][index] = mapsens->val();
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
            // ezread.squintf(".");
            if (!index) {
                dic = !dic;
                // ezread.squintf("Filled dic %d\n", dic);
            }
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
//   B) Stopping the car on entering hold/standby mode is taking longer than it ever should.
//   C) Mule seems to be accelerating like a Tesla.
//   D) Car is accelerating yet engine is at idle.
// * The control system has nonsensical values in its variables.

// more notes on brake error detection ideas:
// 1. Detect  brake chain is not connected (evidenced by change in brake position without expected pressure changes)
// 2. Detect obstruction, motor failure, or inaccurate position. Evidenced by motor instructed to move but position not changing even when pressure is low.
// 3. Detet brake hydraulics failure or inaccurate pressure. Evidenced by normal positional change not causing expected increase in pressure.
// retract_effective_max_us = volt[Stop] + duty_pc * (volt[OpMax] - volt[Stop]);  // Stores instantaneous calculated value of the effective maximum pulsewidth after attenuation

class LoopTimer {
  public:
    LoopTimer() { setup(); }
    // Loop timing related
    Timer loop_timer{1000000};  // how long the previous main loop took to run (in us)
    int loopno = 1, loopindex = 0, loop_recentsum = 0, loop_scale_min_us = 0, loop_scale_avg_max_us = 2500, loop_scale_peak_max_us = 25000;
    float loop_sum_s, loopfreq_hz;
    int looptimes_us[20];
    bool loop_dirty[20];
    int64_t loop_cout_mark_us, boot_mark_us;
    int loop_cout_us = 0, loop_peak_us = 0, loop_now = 0;;
    static constexpr int loop_history = 100;
    int loop_periods_us[loop_history];
    // std::vector<std::string> loop_names(20);
    std::string loop_names[20];
    void setup() {  // Run once at end of setup()
        boot_mark_us = esp_timer_get_time();
        if (looptime_print) {
            for (int x=1; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
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
    float calc_avg(int _loop_now, int _thisloop) {
        if (_loop_now == loop_history + 2) {
            loop_recentsum = _thisloop;
            for (int l = 0; l <= loop_history; l++)
                loop_recentsum += loop_periods_us[(_loop_now + l) % loop_history];
        }
        else loop_recentsum += _thisloop - loop_periods_us[loop_now];
        return (float)loop_recentsum/(float)loop_history;
    }
    void update() {  // Call once each loop at the very end
        if (runmode == LowPower) return;
        int thisloop = (int)loop_timer.elapsed();
        loop_avg_us = calc_avg(loop_now, thisloop);
        loop_periods_us[loop_now] = thisloop;  // us since beginning of this loop
        loop_timer.reset();
        loop_sum_s += (float)loop_periods_us[loop_now] / 1000000;
        // ema_filt(loop_periods_us[loop_now], &loop_avg_us, 0.01);
        if (loop_avg_us > 1) loopfreq_hz = 1000000/loop_avg_us;
        loop_peak_us = 0;
        for (int i=0; i<loop_history; i++) if (loop_peak_us < loop_periods_us[i]) loop_peak_us = loop_periods_us[i]; 
        if (looptime_print) {  // note: convert cout to ezread.squintf
            loop_cout_mark_us = esp_timer_get_time();
            std::cout << std::fixed << std::setprecision(0);
            std::cout << "\r" << (int)loop_sum_s << "s #" << loopno;  //  << " av:" << std::setw(5) << (int)(loop_avg_us);  //  << " av:" << std::setw(3) << loop_avg_ms 
            std::cout << " : " << std::setw(5) << loop_periods_us[loop_now] << " (" << std::setw(5) << loop_periods_us[loop_now]-loop_cout_us << ")us ";  // << " avg:" << loop_avg_us;  //  " us:" << esp_timer_get_time() << 
            for (int x=1; x<loopindex; x++)
                std::cout << std::setw(3) << loop_names[x] << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1] << " ";
            std::cout << " cout:" << std::setw(5) << loop_cout_us;
            if (loop_periods_us[loop_now]-loop_cout_us > looptime_linefeed_threshold || !looptime_linefeed_threshold) std::cout << std::endl;
            loop_cout_us = (int)(esp_timer_get_time() - loop_cout_mark_us);
            loopindex = 0;
            mark("top");
        }
        ++loop_now %= loop_history;
        loopno++;  // I like to count how many loops
    }
    int uptime_us() {  // returns uptime since last reset in microseconds
        return esp_timer_get_time() - boot_mark_us;
    }
    float uptime() {  // returns uptime since last reset in minutes
        return (float)((esp_timer_get_time() - boot_mark_us)) / (60.0 * 1000000.0);
    }
};
class BootMonitor {
  private:
    int timeout_sec = 10, runmode_now, wrote_runmode = 500, wrote_status = 500, crashcount = 0, bootcount;     // variable to track total number of boots of this code build
    int runmode_postmortem, codestatus_postmortem, ign_postmortem, panic_postmortem;
    uint32_t uptime_recorded = -1, uptime_rounding = 5;
    Preferences* myprefs;
    LoopTimer* myloop;
    std::string codestatuscard[NumCodeStatuses] = { "confused", "asleep", "booting", "parked", "stopped", "panicking", "in basicmode", "driving" };
    Timer highWaterTimer{30000000};
    TaskHandle_t* task1; TaskHandle_t* task2; TaskHandle_t* task3; TaskHandle_t* task4;
    UBaseType_t highWaterBytes;
    bool wrote_panic, wrote_ign, was_panicked = false;
    void flash_codestatus(int _stat) {
        codestatus = _stat;
        if (wrote_status != codestatus) myprefs->putUInt("codestatus", (uint32_t)_stat);
        wrote_status = _stat;
    }
    void flash_runmode() {
        runmode_now = runmode;
        if (wrote_runmode != runmode_now) myprefs->putUInt("runmode", (uint32_t)runmode_now);
        wrote_runmode = runmode_now;
    }
    void flash_panicstop() {
        if (wrote_panic != panicstop) myprefs->putUInt("panicstop", (uint32_t)panicstop);
        wrote_panic = panicstop;
    }
    void flash_ignition() {
        if (wrote_ign != ignition.signal) myprefs->putUInt("ignition", (uint32_t)ignition.signal);
        wrote_ign = ignition.signal;
    }
  public:
    // int boot_to_runmode = Standby;    // disabling ability to recover to previous runmode after crash
    BootMonitor(Preferences* _prefs, LoopTimer* _loop) : myprefs(_prefs), myloop(_loop) {}
    void set_codestatus() {
        flash_runmode();
        flash_panicstop();
        flash_ignition();
        int dowrite = StConfused;
        if (panicstop) dowrite = StPanicking;
        if (runmode == Basic) dowrite = StInBasic;
        else if (runmode == LowPower) dowrite = StAsleep;
        else if (runmode == Fly || runmode == Cruise) dowrite = StDriving;
        else if (runmode == Hold || runmode == Stall) dowrite = StStopped;
        else if (runmode == Standby) dowrite = StParked;
        flash_codestatus(dowrite);  // write to flash we are in an appropriate place to lose power, so we can detect crashes on boot
    }
    void setup(TaskHandle_t* t1, TaskHandle_t* t2, TaskHandle_t* t3, TaskHandle_t* t4, int sec = -1) {
        task1 = t1;  task2 = t2;  task3 = t3;  task4 = t4;
        if (sec >= 0) timeout_sec = sec;
        psram_setup();
        myprefs->begin("FlyByWire", false);
        read_bootstatus();
        flash_codestatus(StBooting);
        print_postmortem();
        panic_on_crash();
        print_chip_info();
        // print_partition_table();
        if (!watchdog_enabled) return;
        ezread.squintf("Boot manager.. \n");
        esp_task_wdt_init(timeout_sec, true);  // see https://github.com/espressif/esp-idf/blob/master/examples/system/task_watchdog/main/task_watchdog_example_main.c
        esp_task_wdt_add(NULL);
    }
    void pet() {
        if (!watchdog_enabled) return;
        esp_task_wdt_reset();
    }
    void add(TaskHandle_t taskh) {
        if (!watchdog_enabled) return;
        esp_task_wdt_add(taskh);
    }
    void update() {
        pet();
        if (codestatus == StBooting) set_codestatus();  // we are not booting any more
        write_uptime();
        print_high_water(task1, task2, task3, task4);
    }
    void flash_forcewrite(const char* flashid, uint32_t val) {  // force writes a uint value to indicated flashed value. external code must be responsible here to not write unnecessarily!
        myprefs->putUInt(flashid, val);
    }
    int flash_read(std::string flashid, uint32_t def=1234567) {  // reads a uint value at the indicated flash slot. returns -1 on error
        uint32_t ret = (int)myprefs->getUInt(flashid.c_str(), def);
        if (ret == 1234567) {
            ezread.squintf("Err: no flash id \"%s\"\n", flashid.c_str());
            return -1;
        }
        return (int)ret;
    }
  private:
    void read_bootstatus() {
        bootcount = (int)myprefs->getUInt("bootcount", 0) + 1;
        myprefs->putUInt("bootcount", (uint32_t)bootcount);
        codestatus_postmortem = (int)myprefs->getUInt("codestatus", StConfused);
        ign_postmortem = (int)myprefs->getUInt("ignition", (uint32_t)HIGH);
        panic_postmortem = (int)myprefs->getUInt("panicstop", (uint32_t)HIGH);
        runmode_postmortem = (int)myprefs->getUInt("runmode", (uint32_t)Fly);
        crashcount = (int)myprefs->getUInt("crashcount", 0);
        if (codestatus_postmortem != Parked) crashcount++;
        myprefs->putUInt("crashcount", (uint32_t)crashcount);
        wrote_panic = (codestatus_postmortem == StPanicking);
    }
    void write_uptime() {
        float get_uptime = myloop->uptime();
        float myround = std::min(get_uptime, (float)uptime_rounding);
        int uptime_new = (int)(get_uptime / myround) * (int)myround;
        if (uptime_new == uptime_recorded) return;
        myprefs->putUInt("uptime", (uint32_t)uptime_new);
        uptime_recorded = uptime_new;
    }
    void print_postmortem() {
        ezread.squintf("Bootcount: %d (%d/%d). Last lost power\n  while %s%s in %s mode,\n  after ", 
          bootcount, bootcount-crashcount, crashcount, codestatuscard[codestatus_postmortem].c_str(), panic_postmortem ? " and panicking" : "", modecard[runmode_postmortem].c_str());
        int last_uptime = (int)myprefs->getUInt("uptime", 0);
        if (last_uptime > 0) {
            ezread.squintf("just over %d min uptime\n", last_uptime);
            write_uptime();
        }
        else ezread.squintf("under 1 min uptime\n");
    }
    void print_high_water(xTaskHandle* t1, xTaskHandle* t2, xTaskHandle* t3, xTaskHandle* t4) {
        if (print_task_stack_usage && highWaterTimer.expireset()) {
            ezread.squintf("mem minfree(B): heap:%d", xPortGetMinimumEverFreeHeapSize());            
            highWaterBytes = uxTaskGetStackHighWaterMark(*t1) * sizeof(StackType_t);
            ezread.squintf(" temptask:%d", highWaterBytes);
            highWaterBytes = uxTaskGetStackHighWaterMark(*t2) * sizeof(StackType_t);
            ezread.squintf(", drawtask:%d", highWaterBytes);
            highWaterBytes = uxTaskGetStackHighWaterMark(*t3) * sizeof(StackType_t);
            ezread.squintf(", pushtask:%d", highWaterBytes);
            highWaterBytes = uxTaskGetStackHighWaterMark(*t4) * sizeof(StackType_t);
            ezread.squintf(", maftask:%d\n", highWaterBytes);
        }
    }
    void panic_on_crash() {
        // if ((codestatus_postmortem != StDriving && codestatus_postmortem != StStopped)|| !crash_driving_recovery) return;  // crash_driving_recovery
        if (!panic_on_boot_after_crash || ((codestatus_postmortem != StDriving) && !was_panicked)) return;
        ezread.squintf(ORG, "warn: panic condition %sstarted after crash detect\n", was_panicked ? "re" : "");
        ignition.panic_request(ReqOn);
        // boot_to_runmode = runmode_postmortem;  // disabling ability to recover to previous runmode after crash
        // if (runmode_postmortem == Cruise) boot_to_runmode = Fly; // disabling ability to recover to previous runmode after crash
        // else if (runmode_postmortem == Cal) boot_to_runmode = Standby; // disabling ability to recover to previous runmode after crash
        // ezread.squintf("  Resuming %s run mode w/ ignition %s..\n", modecard[boot_to_runmode], ign_postmortem ? "on" : "off"); // disabling ability to recover to previous runmode after crash
        // ignition.request(ign_postmortem ? ReqOn : ReqOff);  // this is ludicrous! commented out!
        // gas.(brake.pc[Stop]);  // brake.pid_targ_pc(brake.pc[Stop]);
    }
    void psram_setup() {  // see https://www.upesy.com/blogs/tutorials/get-more-ram-on-esp32-with-psram#
        ezread.squintf("PSRAM.. ");
        #ifndef BOARD_HAS_PSRAM
        ezread.squintf("support is currently disabled\n");
        return;
        #endif
        ezread.squintf("is %s, size %d B\n", psramInit() ? "correctly initialized" : "not available", ESP.getFreePsram());
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
    }
    void print_partition_table() {  // warning: this could hang/crash if no "coredump" labeled partition is found
        if (!running_on_devboard) return;
        ezread.squintf("Partition Typ SubT  Address SizeByte   kB\n");
        esp_partition_iterator_t iterator = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
        const esp_partition_t *partition;
        while ((partition = esp_partition_get(iterator)) != NULL) {
            ezread.squintf(" %8s %3d 0x%02x 0x%06x 0x%06x %4d\n", partition->label, partition->type, partition->subtype, partition->address, partition->size, (partition->size)/1024);
            if (!strcmp(partition->label, "coredump")) break;
            iterator = esp_partition_next(iterator);
        }
        esp_partition_iterator_release(iterator);
    }
    void print_chip_info() {
        esp_chip_info_t chip_info;
        uint32_t flash_size;
        esp_chip_info(&chip_info);
        unsigned major_rev = chip_info.revision / 100;
        unsigned minor_rev = chip_info.revision % 100;
        ezread.squintf("MCU: detect %s v%d.%d %d-core, ", CONFIG_IDF_TARGET, major_rev, minor_rev, chip_info.cores);
        if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) ezread.squintf("fail flash detect\n");
        else ezread.squintf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
          (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "emb" : "ext");  // embedded or external
        ezread.squintf("  w/ %s%s%s%s.", 
          (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
          (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
          (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
          (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");
        ezread.squintf(" heap: %" PRIu32 " B free (min)\n", esp_get_minimum_free_heap_size());
    }
};
#if RUN_TESTS
    #include "unittests.h"
    void run_tests() {
        Serial.printf("Tests running..");
        delay(5000);
        test_Param();
        Serial.printf("Tests complete\n");
        for(;;); // loop forever
    }
#else
    void run_tests() {}
#endif
// // Moved sdcard code into diag.h to reduce file count. these includes go with it
// #include <iostream>
// #include <string>
// #include <iomanip>
// #include <SD.h>
// #include <FFat.h>
// #include <LovyanGFX.hpp>

// #define FORMAT_FFAT true  // You only need to format FFat the first time you use a specific card

// class SdCard {
//   private:
//     // LGFX* lcd;
//     Timer capture_timer{5000000};
//     int capcount = 0, capmax = 100;
//     char filenamebase[11] = "/screencap";
//     char extension[5] = ".bmp";
//     #ifndef SDCARD_SPI
//         #define SDCARD_SPI SPI
//     #endif
//     std::string filename_seq(int num) {
//         std::string writefile = filenamebase + std::string(2 - std::to_string(num).length(), '0'); // Add leading zeros
//         writefile += std::to_string(num) + extension;
//         return writefile;
//     }
//     std::string saveToSD(int num) {
//         std::string filename = filename_seq(num);
//         bool result = false;
//         File file = SD.open(filename.c_str(), "w");
//         if (file) {
//             int width = 320; //  = lcd->width();
//             int height = 240; // = lcd->height();
//             int rowSize = (2 * width + 3) & ~ 3;  // int rowSize = (3 * width + 3) & ~ 3;  // 24-bit version
//             lgfx::bitmap_header_t bmpheader;
//             bmpheader.bfType = 0x4D42;
//             bmpheader.bfSize = rowSize * height + sizeof(bmpheader);
//             bmpheader.bfOffBits = sizeof(bmpheader);
//             bmpheader.biSize = 40;
//             bmpheader.biWidth = width;
//             bmpheader.biHeight = height;
//             bmpheader.biPlanes = 1;
//             bmpheader.biBitCount = 16;  // bmpheader.biBitCount = 24;  // 24-bit version
//             bmpheader.biCompression = 3;  // bmpheader.biCompression = 0;  // 24-bit version
//             file.write((std::uint8_t*)&bmpheader, sizeof(bmpheader));
//             std::uint8_t buffer[rowSize];
//             memset(&buffer[rowSize - 4], 0, 4);
//             for (int y = 240 - 1; y >= 0; y--) {
//                 // lcd->readRect(0, y, lcd->width(), 1, (lgfx::rgb565_t*)buffer);
//                 // // lcd->readRect(0, y, lcd->width(), 1, (lgfx::rgb888_t*)buffer);  // 24 bit color version
//                 file.write(buffer, rowSize);
//             }
//             file.close();
//             result = true;
//         }
//         else Serial.print("error:file open failure\n");
//         return filename;
//     }
//     void draw_bmp_file(void) {
//         ++capcount %= capmax;
//         std::string wfile = filename_seq(capcount);

//         // lcd->drawBmpFile(SD, wfile.c_str(), random(-20,20), random(-20, 20));  // drawBmpFile(fs, path, x, y, maxWidth, maxHeight, offX, offY, scale_x, scale_y, datum);

//         ezread.squintf("wrote: %s\n", wfile.c_str());
//     }
//   public:
//     // SdCard(LGFX* _lcd\\) : lcd(_lcd) {}
//     SdCard() {}

//     void setup(void) {
//         // lcd->init();
//         // Serial.begin(115200);
//         // lcd->setColorDepth(16);
//         // lcd->setColor(TFT_WHITE);

//         // lcd->startWrite();
//         // lcd->setAddrWindow(0, 0, lcd->width(), lcd->height());
//         // for (int y = 0; y < lcd->height(); ++y) 
//         //     for (int x = 0; x < lcd->width(); ++x) 
//         //         lcd->writeColor( lcd->color888(x << 1, x + y, y << 1), 1);
//         // ezread.squintf("BMP save test\n");
//         // lcd->endWrite();
        
//         // for (int i=0; i<capmax; i++) {
//         // int i = 0;
//         // std::string filename = filename_seq(i);
//         // do {
//         //     SD.end();
//         //     delay(1000);
//         //     SD.begin(sdcard_cs_pin, SDCARD_SPI, 25000000);
//         // } while (!saveToSD(filename));
//         // ezread.squintf("BMP save %s success\n", filename.c_str());
        
//         // lcd->setAddrWindow(0, 0, 320, 240);

//     }

//     void update() {
//         if (capture_timer.expireset()) {
//             ++capcount %= capmax;
//             std::string wfile = saveToSD(capcount);
//             ezread.squintf("wrote: %s\n", wfile.c_str());
//         }
//     }
// };
// class FatFs {
//   public:
//     // This file should be compiled with 'Partition Scheme' (in Tools menu)
//     // set to 'Default with ffat' if you have a 4MB ESP32 dev module or
//     // set to '16M Fat' if you have a 16MB ESP32 dev module.
//     FatFs() {}
//     void setup() {
//         // format: allocation unit size must be a power of 2, w/ min=sector_size, max=128*sector_size. setting to 0 will result in allocation unit set to the sector size. larger is faster but with more wasted space when files are small
//         // sector size is always 512 B. For wear levelling, sector size is determined by CONFIG_WL_SECTOR_SIZE option of esp_vfs_fat_mount_config_t.
//         Serial.setDebugOutput(true);
//         if (FORMAT_FFAT) FFat.format();
//         if (!FFat.begin()){
//             Serial.println("fatfs mount failed\n");
//             return;
//         }
//         Serial.println("fatfs mounted on /sd\n");
//     }
//     void fattest() {
//         ezread.squintf("Total space: %10u\n", FFat.totalBytes());
//         ezread.squintf("Free space: %10u\n", FFat.freeBytes());
//         listDir(FFat, "/", 0);
//         writeFile(FFat, "/hello.txt", "Hello ");
//         appendFile(FFat, "/hello.txt", "World!\r\n");
//         readFile(FFat, "/hello.txt");
//         renameFile(FFat, "/hello.txt", "/foo.txt");
//         readFile(FFat, "/foo.txt");
//         deleteFile(FFat, "/foo.txt");
//         testFileIO(FFat, "/test.txt");
//         ezread.squintf("Free space: %10u\n", FFat.freeBytes());
//         deleteFile(FFat, "/test.txt");
//         Serial.println( "Test complete" );
//     }
//     void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
//         ezread.squintf("Listing directory: %s\r\n", dirname);

//         File root = fs.open(dirname);
//         if (!root){
//             Serial.println("- failed to open directory");
//             return;
//         }
//         if (!root.isDirectory()){
//             Serial.println(" - not a directory");
//             return;
//         }

//         File file = root.openNextFile();
//         while(file){
//             if (file.isDirectory()){
//                 Serial.print("  DIR : ");
//                 Serial.println(file.name());
//                 if (levels) listDir(fs, file.path(), levels -1);
//             }
//             else {
//                 Serial.print("  FILE: ");
//                 Serial.print(file.name());
//                 Serial.print("\tSIZE: ");
//                 Serial.println(file.size());
//             }
//             file = root.openNextFile();
//         }
//     }

//     void readFile(fs::FS &fs, const char * path) {
//         ezread.squintf("Reading file: %s\r\n", path);

//         File file = fs.open(path);
//         if (!file || file.isDirectory()){
//             Serial.println("- failed to open file for reading");
//             return;
//         }

//         Serial.println("- read from file:");
//         while (file.available()) Serial.write(file.read());
//         file.close();
//     }

//     void writeFile(fs::FS &fs, const char * path, const char * message) {
//         ezread.squintf("Writing file: %s\r\n", path);

//         File file = fs.open(path, FILE_WRITE);
//         if (!file){
//             Serial.println("- failed to open file for writing");
//             return;
//         }
//         if (file.print(message)) Serial.println("- file written");
//         else Serial.println("- write failed");
//         file.close();
//     }

//     void appendFile(fs::FS &fs, const char * path, const char * message) {
//         ezread.squintf("Appending to file: %s\r\n", path);

//         File file = fs.open(path, FILE_APPEND);
//         if (!file){
//             Serial.println("- failed to open file for appending");
//             return;
//         }
//         if (file.print(message)) Serial.println("- message appended");
//         else Serial.println("- append failed");
//         file.close();
//     }
//     void renameFile(fs::FS &fs, const char * path1, const char * path2) {
//         ezread.squintf("Renaming file %s to %s\r\n", path1, path2);
//         if (fs.rename(path1, path2)) Serial.println("- file renamed");
//         else Serial.println("- rename failed");
//     }
//     void deleteFile(fs::FS &fs, const char * path) {
//         ezread.squintf("Deleting file: %s\r\n", path);
//         if (fs.remove(path)) Serial.println("- file deleted");
//         else Serial.println("- delete failed");
//     }
//     void testFileIO(fs::FS &fs, const char * path) {
//         ezread.squintf("Testing file I/O with %s\r\n", path);

//         static uint8_t buf[512];
//         size_t len = 0;
//         File file = fs.open(path, FILE_WRITE);
//         if (!file) {
//             Serial.println("- failed to open file for writing");
//             return;
//         }
//         size_t i;
//         Serial.print("- writing");
//         uint32_t start = millis();
//         for (i=0; i<2048; i++) {
//             if ((i & 0x001F) == 0x001F) Serial.print(".");
//             file.write(buf, 512);
//         }
//         Serial.println("");
//         uint32_t end = millis() - start;
//         ezread.squintf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
//         file.close();

//         file = fs.open(path);
//         start = millis();
//         end = start;
//         i = 0;
//         if (file && !file.isDirectory()) {
//             len = file.size();
//             size_t flen = len;
//             start = millis();
//             Serial.print("- reading" );
//             while (len){
//                 size_t toRead = len;
//                 if (toRead > 512) toRead = 512;
//                 file.read(buf, toRead);
//                 if ((i++ & 0x001F) == 0x001F) Serial.print(".");
//                 len -= toRead;
//             }
//             Serial.println("");
//             end = millis() - start;
//             ezread.squintf("- %u bytes read in %u ms\r\n", flen, end);
//             file.close();
//         }
//         else Serial.println("- failed to open file for reading");
//     }
// };