// Carpet CANTroller II  Source Code  - For ESP32-S3-DevKitC-1-N8
#include <SPI.h>  // SPI serial bus
#include "common.h"
#include "uictrl.h"
#include "mapsens.h"
#include "sensors.h"
#include "temperature.h"
#include "motors.h"
#include "neopixel.h"
#include "globals.h"
#include "display.h"
#include "touch.h"
#include "RunModeManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
Display screen;
TouchScreen touch(touch_cs_pin);
RunModeManager run(&screen, &encoder);
ESP32PWM pwm;  // Object for timer pwm resources (servo outputs)
void get_touchpoint() { 
    touch_pt[0] = touch.touch_pt(0);
    touch_pt[1] = touch.touch_pt(1); 
    touch_pt[2] = touch.getX();
    touch_pt[3] = touch.getY(); 
}
void neo_idiots_update() {
    for (int32_t idiot = 0; idiot < arraysize(idiotlights); idiot++) {
        if (idiot <= neo.neopixelsAvailable())
            if (*(idiotlights[idiot]) != idiotlasts[idiot]) neo.setBoolState(idiot, *idiotlights[idiot]);
        if (idiot == LOST || idiot == RANGE) {
            if (highest_pri_failing_last[idiot] != highest_pri_failing_sensor[idiot]) {
                if (highest_pri_failing_sensor[idiot] == e_none) neo.setflash((int)idiot, 0);
                else neo.setflash((int)idiot, highest_pri_failing_sensor[idiot] + 1, 2, 6, 1, 0);
            }
            highest_pri_failing_last[idiot] = highest_pri_failing_sensor[idiot];
        }
    }
}
void hotrc_events_update(int8_t _rm) {
    hotrc.toggles_update();
    if (hotrc.sw_event(ch3)) ignition_request = req_tog;  // Turn on/off the vehicle ignition. If ign is turned off while the car is moving, this leads to panic stop
    if (hotrc.sw_event(ch4)) {
        if (_rm == FLY || _rm == CRUISE) flycruise_toggle_request = true;
        else if (_rm == STALL) starter_request = req_tog;
        else if (_rm == HOLD) starter_request = req_off;
        else if (_rm == SHUTDOWN && !shutdown_incomplete) sleep_request = req_on;
        else if (_rm == ASLEEP) sleep_request = req_off; 
    }
    hotrc.toggles_reset();
}
void setup() {  // Setup just configures pins (and detects touchscreen type)
    if (RUN_TESTS) run_tests();
    analogReadResolution(adcbits);  // Set ADC to 12-bit resolution
    set_pin(starter_pin, INPUT_PULLDOWN);
    set_pin(basicmodesw_pin, INPUT_PULLUP);
    if (!usb_jtag) set_pin(steer_enc_a_pin, INPUT_PULLUP);
    if (!usb_jtag) set_pin(steer_enc_b_pin, INPUT_PULLUP);
    set_pin(neopixel_pin, OUTPUT);
    set_pin(sdcard_cs_pin, OUTPUT);
    set_pin(touch_cs_pin, OUTPUT);
    set_pin(tft_cs_pin, OUTPUT);
    set_pin(tft_dc_pin, OUTPUT);
    set_pin(ignition_pin, OUTPUT);
    set_pin(syspower_pin, OUTPUT);  // Then set the put as an output as normal.
    set_pin(brake_pwm_pin, OUTPUT);
    set_pin(steer_pwm_pin, OUTPUT);
    set_pin(gas_pwm_pin, OUTPUT);
    write_pin(tft_cs_pin, HIGH);   // Prevent bus contention
    write_pin(sdcard_cs_pin, HIGH);   // Prevent bus contention
    write_pin(touch_cs_pin, HIGH);   // Prevent bus contention
    write_pin(ignition_pin, LOW);
    write_pin(syspower_pin, syspower);
    set_pin(uart_tx_pin, INPUT);  // UART:  1st detect breadboard vs. vehicle PCB using TX pin pullup, then repurpose pin for UART and start UART 
    running_on_devboard =(read_pin(uart_tx_pin));
    set_board_defaults(running_on_devboard);
    set_pin(uart_tx_pin, OUTPUT);  // 
    Serial.begin(115200);  // Open console serial port
    delay(800);  // This is needed to allow the uart to initialize and the screen board enough time after a cold boot
    printf("Console started..\nUsing %s defaults..\n", (running_on_devboard) ? "dev-board" : "vehicle-pcb");
    printf("Init rmt for hotrc..\n");
    hotrc.init();
    printf("Pot setup..\n");
    pot.setup();
    printf("Encoder setup..\n");
    encoder.setup();
    printf("Brake pressure sensor.. ");
    pressure.setup();
    printf("done\nBrake position sensor.. ");
    brakepos.setup();
    printf("done\nVehicle battery sense.. ");
    mulebatt.setup();
    printf("done\nLiPo cell sense.. ");
    lipobatt.setup();
    printf("done\nTachometer.. ");
    tach.setup();
    printf("done\nSpeedometer.. ");
    speedo.setup();
    printf("done..\nInit i2c and i2c-enabled devices.."); delay(1);  // Attempt to force print to happen before init
    i2c.init();
    airvelo.setup(); // must be done after i2c is started
    mapsens.setup();
    tempsens.setup();  // Onewire bus and temp sensors
    xTaskCreate(update_temperature_sensors, "Update Temperature Sensors", 2048, NULL, 5, NULL);  // Create a new task that runs the update_temperature_sensors function
    throttle.init(gas.mypid.target_ptr(), tach.human_ptr(), tach.filt_ptr(),
        tempsens.get_sensor(loc::engine), temp_lims_f[ENGINE][OP_MIN], temp_lims_f[ENGINE][WARNING], 50, Throttle::idlemodes::control);
    printf("Simulator setup..\n");
    sim.register_device(sens::pressure, pressure, pressure.source());
    sim.register_device(sens::brkpos, brakepos, brakepos.source());
    sim.register_device(sens::airvelo, airvelo, airvelo.source());
    sim.register_device(sens::mapsens, mapsens, mapsens.source());
    sim.register_device(sens::tach, tach, tach.source());
    sim.register_device(sens::speedo, speedo, speedo.source());
    printf("Configure timers for PWM out..\n");
    for (int ch=0; ch<4; ch++) ESP32PWM::allocateTimer(ch);
    gas.init(gas_pwm_pin, 60, &hotrc, &speedo, &tach, &pot, &throttle);
    brake.init(brake_pwm_pin, 50, &hotrc, &speedo, &pressure, &brakepos);
    steer.init(steer_pwm_pin, 50, &hotrc, &speedo);
    printf("Init screen.. ");
    if (display_enabled) {
        config.begin("FlyByWire", false);
        datapage = config.getUInt("dpage", PG_RUN);
        datapage_last = config.getUInt("dpage", PG_TEMP);
        printf("tft.. ");
        screen.init();
        printf("touchscreen..\n");
        touch.init();
    }
    if (touch_reticles) screen.draw_reticles();
    std::cout << "Init neopixels.. ";
    neo_setup();
    int32_t idiots = smin((uint32_t)arraysize(idiotlights), neo.neopixelsAvailable());
    for (int32_t idiot = 0; idiot < idiots; idiot++)
        neo.newIdiotLight(idiot, idiotcolors[idiot], *(idiotlights[idiot]));
    std::cout << "set up heartbeat led and " << idiots << " neopixel idiot lights" << std::endl;
    for (int32_t i=0; i<num_err_types; i++) for (int32_t j=0; j<e_num_sensors; j++) err_sensor[i][j] = false; // Initialize sensor error flags to false
    printf("Setup done%s\n", console_enabled ? "" : ". stopping console during runtime");
    if (!console_enabled) Serial.end();  // close serial console to prevent crashes due to error printing
    panicTimer.reset();
    looptime_init();
}
void loop() {
    ignition_panic_update();  // handler for ignition pin output and panicstop status.
    bootbutton_update();
    basicsw_update();
    starter_update();  // Runs starter bidirectional handler
    encoder.update();  // Read encoder input signals
    pot.update();
    brakepos.update();  // Brake position
    tach.update();  // Tach
    throttle.push_tach_reading(tach.human(), tach.last_read_time());    
    speedo.update();  // Speedo
    pressure.update();  // Brake pressure
    mulebatt.update();
    lipobatt.update();
    airvelo.update();
    mapsens.update();  // MAP sensor  // takes 6800 us (!!)
    maf_gps = massairflow();  // Recalculate intake mass airflow
    if (touch_reticles) get_touchpoint();
    hotrc_events_update(run.mode());
    hotrc.update();
    run.run_runmode();  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    gas.update(run.mode());
    brake.update(run.mode());
    steer.update(run.mode());
    if (sim.potmapping(sens::joy)) hotrc.set_pc(horz, filt, pot.mapToRange(steer.pc_to_us(steer.pc[opmin]), steer.pc_to_us(steer.pc[opmax])));
    
    // debugging stupid gas stuck on full bug
    float gas_last, brake_last, steer_last;
    if (gas.us[out] != gas_last || brake.us[out] != brake_last || steer.us[out] != steer_last)
        printf("gas %d brake %d steer %d\n", (int32_t)(gas.us[out]), (int32_t)(brake.us[out]), (int32_t)(steer.us[out]));
    gas_last = gas.us[out]; brake_last = brake.us[out]; steer_last = steer.us[out];

    sel_val_last = sel_val;
    datapage_last = datapage;
    tunctrl_last = tunctrl; // Make sure this goes after the last comparison
    touch.update(); // Handle touch events and actions
    if (screensaver && touch.touched()) screen.saver_touch(touch.touch_pt(0), touch.touch_pt(1));
    uint32_t encoder_sw_action = encoder.press_event();  // true = autoreset the event if there is one
    if (encoder_sw_action != Encoder::NONE) {  // First deal with any unhandled switch press events
        if (encoder_sw_action == Encoder::SHORT)  {  // if short press
            if (tunctrl == EDIT) tunctrl = SELECT;  // If we were editing a value drop back to select mode
            else if (tunctrl == SELECT) tunctrl = EDIT;  // If we were selecting a variable start editing its value
        }
        else tunctrl = (tunctrl == OFF) ? SELECT : OFF;  // Long press starts/stops tuning
    }
    if (tunctrl == EDIT) idelta_encoder = encoder.rotation(true);  // true = include acceleration
    else if (tunctrl == SELECT) sel_val += encoder.rotation();  // If overflow constrain will fix in general handler below
    else if (tunctrl == OFF) datapage += encoder.rotation();  // If overflow tconstrain will fix in general below
    idelta += idelta_encoder + idelta_touch;  // Allow edits using the encoder or touchscreen
    idelta_touch = idelta_encoder = 0;
    if (tunctrl != tunctrl_last || datapage != datapage_last || sel_val != sel_val_last || idelta) tuningCtrlTimer.reset();  // If just switched tuning mode or any tuning activity, reset the timer
    else if (tunctrl != OFF && tuningCtrlTimer.expired()) tunctrl = OFF;  // If the timer expired, go to OFF and redraw the tuning corner
    datapage = constrain(datapage, 0, arraysize(pagecard)-1);  // select next or prev only 1 at a time, avoiding over/underflows, and without giving any int negative value
    if (datapage != datapage_last) {
        if (tunctrl == EDIT) tunctrl = SELECT;  // If page is flipped during edit, drop back to select mode
        disp_datapage_dirty = true;  // Redraw the fixed text in the tuning corner of the screen with data from the new dataset page
    }
    if (tunctrl == SELECT) {
        sel_val = constrain(sel_val, tuning_first_editable_line[datapage], disp_tuning_lines-1);  // Skip unchangeable values for all PID modes
        if (sel_val != sel_val_last) disp_selected_val_dirty = true;
    }
    if (tunctrl != tunctrl_last || disp_datapage_dirty) disp_selected_val_dirty = true;
    float fdelta = (float)idelta;
    if (tunctrl == EDIT && idelta != 0) {  // Change tunable values when editing
        if (datapage == PG_RUN) {
            if (sel_val == 9) { adj_val(&(gas.pc[govern]), idelta, 0, 100); gas.derive(); }
            else if (sel_val == 10) adj_val(&(steer.steer_safe_pc), idelta, 0, 100);
        }
        else if (datapage == PG_JOY) {
            if (sel_val == 9) adj_val(&hotrc.failsafe_us, idelta, hotrc.absmin_us, hotrc.us[vert][opmin] - hotrc.us[vert][margin]);
            else if (sel_val == 10) { adj_val(&hotrc.deadband_us, idelta, 0, 50); hotrc.calc_params(); }
        }
        else if (datapage == PG_SENS) {
            if (sel_val == 2) throttle.add_idlehot(0.1 * fdelta);
            else if (sel_val == 3) throttle.add_idlecold(0.1 * fdelta);
            else if (sel_val == 4) adj_val(tach.redline_rpm_ptr(), 0.1 * fdelta, throttle.idlehigh, tach.abs_max_rpm());
            else if (sel_val == 5) adj_val(airvelo.max_mph_ptr(), 0.01 * fdelta, 0, airvelo.abs_max_mph());
            else if (sel_val == 6) adj_val(mapsens.min_psi_ptr(), 0.1 * fdelta, mapsens.abs_min_psi(), mapsens.abs_max_psi());
            else if (sel_val == 6) adj_val(mapsens.max_psi_ptr(), 0.1 * fdelta, mapsens.abs_min_psi(), mapsens.abs_max_psi());
            else if (sel_val == 8) adj_val(&speedo_idle_mph, 0.01 * fdelta, 0, speedo.redline_mph() - 1);
            else if (sel_val == 9) adj_val(speedo.redline_mph_ptr(), 0.01 * fdelta, speedo_idle_mph, 20);
            else if (sel_val == 10) adj_val(brakepos.zeropoint_ptr(), 0.001 * fdelta, brakepos.op_min_in(), brakepos.op_max_in());
        }
        else if (datapage == PG_PWMS) {
            if (sel_val == 7) { adj_val(&(gas.nat[opmin]), fdelta, gas.nat[parked] + 1, gas.nat[opmax] - 1); gas.derive(); }
            else if (sel_val == 8) { adj_val(&(gas.nat[opmax]), fdelta, gas.nat[opmin] + 1, 180.0); gas.derive(); }
            else if (sel_val == 9) { adj_val(&(brake.us[stop]), fdelta, brake.us[opmin] + 1, brake.us[opmax] - 1); brake.derive(); }
            else if (sel_val == 10) { adj_val(&(brake.duty_pc), fdelta, 0.0, 100.0); brake.derive(); }
        }
        else if (datapage == PG_IDLE) {
            if (sel_val == 4) throttle.add_idlehigh(fdelta);
            else if (sel_val == 5) throttle.add_idlecold(fdelta);
            else if (sel_val == 6) throttle.add_idlehot(fdelta);
            else if (sel_val == 7) throttle.add_tempcold(fdelta);
            else if (sel_val == 8) throttle.add_temphot(fdelta);
            else if (sel_val == 9) throttle.add_settlerate(idelta);
            else if (sel_val == 10) throttle.cycle_idlemode(idelta);
        }
        else if (datapage == PG_BPID) {
            if (sel_val == 8) brake.mypid.add_kp(0.001 * fdelta);
            else if (sel_val == 9) brake.mypid.add_ki(0.001 * fdelta);
            else if (sel_val == 10) brake.mypid.add_kd(0.001 * fdelta);
        }
        else if (datapage == PG_GPID) {
            if (sel_val == 7) { adj_bool(&(gas.open_loop), idelta); }  // gas_pid.SetMode (gas_open_loop ? qpid::ctrl::manual : qpid::ctrl::automatic);
            else if (sel_val == 8) gas.mypid.add_kp(0.001 * fdelta);
            else if (sel_val == 9) gas.mypid.add_ki(0.001 * fdelta);
            else if (sel_val == 10) gas.mypid.add_kd(0.001 * fdelta);
        }
        else if (datapage == PG_CPID) {
            if (sel_val == 7) adj_val(&cruise_delta_max_pc_per_s, idelta, 1, 35);
            else if (sel_val == 8) gas.cruisepid.add_kp(0.001 * fdelta);
            else if (sel_val == 9) gas.cruisepid.add_ki(0.001 * fdelta);
            else if (sel_val == 10) gas.cruisepid.add_kd(0.001 * fdelta);
        }
        else if (datapage == PG_TEMP) {
            if (sel_val == 10) adj_bool(&dont_take_temperatures, idelta);
        }
        else if (datapage == PG_SIM) {
            if (sel_val == 0) sim.set_can_sim(sens::joy, idelta);
            else if (sel_val == 1) sim.set_can_sim(sens::pressure, idelta);
            else if (sel_val == 2) sim.set_can_sim(sens::brkpos, idelta);
            else if (sel_val == 3) sim.set_can_sim(sens::speedo, idelta);
            else if (sel_val == 4) sim.set_can_sim(sens::tach, idelta);
            else if (sel_val == 5) sim.set_can_sim(sens::airvelo, idelta);
            else if (sel_val == 6) sim.set_can_sim(sens::mapsens, idelta);  // else if (sel_val == 7) sim.set_can_sim(sens::starter, idelta);
            else if (sel_val == 7) sim.set_can_sim(sens::basicsw, idelta);
            else if (sel_val == 8) sim.set_potmap((adj_val(sim.potmap(), idelta, 0, arraysize(sensorcard) - 4)));            
            else if (sel_val == 9 && run.mode() == CAL) adj_bool(&cal_joyvert_brkmotor_mode, idelta);
            else if (sel_val == 10 && run.mode() == CAL) adj_bool(&cal_pot_gasservo_mode, (idelta < 0 || cal_pot_gasservo_ready) ? idelta : -1);
        }
        else if (datapage == PG_UI) {
            if (sel_val == 7) { adj_bool(&flashdemo, idelta); enable_flashdemo(flashdemo); }
            else if (sel_val == 8) { adj_val(&neobright, idelta, 1, 100); neo.setbright(neobright); }
            else if (sel_val == 9) { adj_val(&neodesat, idelta, 0, 10); neo.setdesaturation(neodesat); }
            else if (sel_val == 10) adj_bool(&screensaver, idelta);
        }
        idelta = 0;
    }
    diag_update();  // notice any screwy conditions or suspicious shenigans
    neo_idiots_update();
    neo.set_heartcolor(colorcard[run.mode()]);
    neo.update(!syspower);
    looptime_mark("-");
    screen.update(run.mode());  // Display updates
    if (!display_enabled && datapage_last != datapage) config.putUInt("dpage", datapage);
    looptime_mark("dis");
    looptime_update();
}