// Carpet CANTroller II  Source Code  - For ESP32-S3-DevKitC-1-N8
#include "globals.h"
#include "uictrl.h"
#include "mapsens.h"
#include "sensors.h"
#include "temperature.h"
#include "motors.h"
#include "neopixel.h"
#include "objects.h"
#include "display.h"
#include "touch.h"
#include "RunModeManager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
static Display screen;
static TouchScreen touch(touch_cs_pin);
static RunModeManager run(&screen, &encoder);
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
void setup() {  // Setup just configures pins (and detects touchscreen type)
    if (RUN_TESTS) run_tests();
    analogReadResolution(adcbits);  // Set ADC to 12-bit resolutqion
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
    set_board_defaults();
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
    printf("Simulator setup..\n");
    sim.register_device(sens::pressure, pressure, pressure.source());
    sim.register_device(sens::brkpos, brakepos, brakepos.source());
    sim.register_device(sens::airvelo, airvelo, airvelo.source());
    sim.register_device(sens::mapsens, mapsens, mapsens.source());
    sim.register_device(sens::tach, tach, tach.source());
    sim.register_device(sens::speedo, speedo, speedo.source());
    printf("Configure motors..\n");
    throttle.init(gas.pid.target_ptr(), tach.human_ptr(), tach.filt_ptr(),
        tempsens.get_sensor(loc::engine), temp_lims_f[ENGINE][OP_MIN], temp_lims_f[ENGINE][WARNING], 50, Throttle::idlemodes::control);
    for (int ch=0; ch<4; ch++) ESP32PWM::allocateTimer(ch);
    gas.init(gas_pwm_pin, 60, &hotrc, &speedo, &tach, &pot, &throttle);
    brake.init(brake_pwm_pin, 50, &hotrc, &speedo, &mulebatt, &pressure, &brakepos);
    steer.init(steer_pwm_pin, 50, &hotrc, &speedo, &mulebatt);
    printf("Init display.. ");
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
    neo_setup();
    int32_t idiots = smin((uint32_t)arraysize(idiotlights), neo.neopixelsAvailable());
    for (int32_t idiot = 0; idiot < idiots; idiot++) neo.newIdiotLight(idiot, idiotcolors[idiot], *(idiotlights[idiot]));
    std::cout << "set up heartbeat led and " << idiots << " neopixel idiot lights" << std::endl;
    for (int32_t i=0; i<num_err_types; i++) for (int32_t j=0; j<e_num_sensors; j++) err_sensor[i][j] = false; // Initialize sensor error flags to false
    printf("Setup done%s\n", console_enabled ? "" : ". stopping console during runtime");
    if (!console_enabled) Serial.end();  // close serial console to prevent crashes due to error printing
    panicTimer.reset();
    looptime_init();
}
void loop() {
    ignition_panic_update();  // handler for ignition pin output and panicstop status.
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
    hotrc_events_update(run.mode);
    hotrc.update();
    run.run_runmode();  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    gas.update(run.mode);
    brake.update(run.mode);
    steer.update(run.mode);
    if (sim.potmapping(sens::joy)) hotrc.set_pc(horz, filt, pot.mapToRange(steer.pc_to_us(steer.pc[opmin]), steer.pc_to_us(steer.pc[opmax])));
    touch.update(); // Handle touch events and actions
    if (screensaver && touch.touched()) screen.saver_touch(touch.touch_pt(0), touch.touch_pt(1));
    tuner_update(run.mode);
    diag_update();  // notice any screwy conditions or suspicious shenigans
    neo_idiots_update();
    neo.set_heartcolor(colorcard[run.mode]);
    neo.update(!syspower);
    looptime_mark("-");
    screen.update(run.mode);  // Display updates
    if (!display_enabled && datapage_last != datapage) config.putUInt("dpage", datapage);
    looptime_mark("dis");
    looptime_update();
}