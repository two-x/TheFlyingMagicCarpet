// Carpet CANTroller II  Source Code  - For ESP32-S3-DevKitC-1-N8
#include "globals.h"
#include "uictrl.h"
#include "i2cbus.h"
#include "sensors.h"
#include "temperature.h"
#include "motors.h"  // qpid.h is included from within motors.h
#include "neopixel.h"
#include "web.h"
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
    set_pin(starter_pin, INPUT_PULLDOWN);
    set_pin(basicmodesw_pin, INPUT_PULLUP);
    if (!usb_jtag) set_pin(steer_enc_a_pin, INPUT_PULLUP);
    if (!usb_jtag) set_pin(steer_enc_b_pin, INPUT_PULLUP);
    set_pin(sdcard_cs_pin, OUTPUT, HIGH);  // Prevent bus contention
    set_pin(ignition_pin, OUTPUT, LOW);
    set_pin(syspower_pin, OUTPUT, syspower);  // Then set the put as an output as normal.
    set_pin(uart_tx_pin, INPUT);  // UART:  1st detect breadboard vs. vehicle PCB using TX pin pullup, then repurpose pin for UART and start UART 
    running_on_devboard = (read_pin(uart_tx_pin));
    Serial.begin(115200);  // Open console serial port
    delay(800);  // This is needed to allow the uart to initialize and the screen board enough time after a cold boot
    printf("Console started..\nUsing %s defaults..\n", (running_on_devboard) ? "dev-board" : "vehicle-pcb");
    hotrc.init();
    pot.setup();
    encoder.setup();
    printf("Brake pressure sensor..\n");
    pressure.setup();
    printf("Brake position sensor..\n");
    brakepos.setup();
    printf("Vehicle battery sense..\n");
    mulebatt.setup();
    printf("LiPo cell sense..\n");
    lipobatt.setup();
    printf("Tachometer..\n");
    tach.setup();
    printf("Speedometer..\n");
    speedo.setup();
    i2c.init();
    airvelo.setup(); // must be done after i2c is started
    mapsens.setup();
    lightbox.init();
    tempsens.setup();  // Onewire bus and temp sensors
    xTaskCreate(update_temperature_sensors, "Update Temperature Sensors", 2048, NULL, 5, NULL);  // Create a new task that runs the update_temperature_sensors function
    printf("Simulator setup..\n");
    sim.register_device(sens::pressure, pressure, pressure.source());
    sim.register_device(sens::brkpos, brakepos, brakepos.source());
    sim.register_device(sens::airvelo, airvelo, airvelo.source());
    sim.register_device(sens::mapsens, mapsens, mapsens.source());
    sim.register_device(sens::tach, tach, tach.source());
    sim.register_device(sens::speedo, speedo, speedo.source());
    idlectrl.init(gas.pid.target_ptr(), tach.human_ptr(), tach.filt_ptr(),
        tempsens.get_sensor(loc::ENGINE), temp_lims_f[ENGINE][OP_MIN], temp_lims_f[ENGINE][WARNING], 50, IdleControl::idlemodes::CONTROL);
    for (int ch=0; ch<4; ch++) ESP32PWM::allocateTimer(ch);
    gas.init(gas_pwm_pin, 60, &hotrc, &speedo, &tach, &pot, &idlectrl);
    brake.init(brake_pwm_pin, 50, &hotrc, &speedo, &mulebatt, &pressure, &brakepos);
    steer.init(steer_pwm_pin, 50, &hotrc, &speedo, &mulebatt);
    config.begin("FlyByWire", false);
    datapage = config.getUInt("dpage", PG_RUN);
    datapage_last = config.getUInt("dpage", PG_TEMP);
    sim.set_potmap(config.getUInt("potmap", 2));  // 2 = sens::pressure
    set_board_defaults();
    if (display_enabled) {
        screen.init();
        touch.init();
    }
    neo_setup();
    int32_t idiots = smin((uint32_t)arraysize(idiotlights), neo.neopixelsAvailable());
    for (int32_t idiot = 0; idiot < idiots; idiot++) neo.newIdiotLight(idiot, idiotcolors[idiot], *(idiotlights[idiot]));
    std::cout << "set up heartbeat led and " << idiots << " neopixel idiot lights" << std::endl;
    for (int32_t i=0; i<NUM_ERR_TYPES; i++) for (int32_t j=0; j<E_NUM_SENSORS; j++) err_sensor[i][j] = false; // Initialize sensor error flags to false
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
    speedo.update();  // Speedo
    // lightbox.update(run.mode, speedo.human());
    pressure.update();  // Brake pressure
    mulebatt.update();
    lipobatt.update();
    airvelo.update();
    mapsens.update();  // MAP sensor  // takes 6800 us (!!)
    maf_gps = massairflow();  // Recalculate intake mass airflow
    if (touch_reticles) get_touchpoint();
    hotrc_events_update(run.mode);
    hotrc.update();
    if (sim.potmapping(sens::joy)) hotrc.set_pc(HORZ, FILT, pot.mapToRange(steer.pc_to_us(steer.pc[OPMIN]), steer.pc_to_us(steer.pc[OPMAX])));
    run.mode_logic();  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    gas.update(run.mode);
    brake.update(run.mode);
    steer.update(run.mode);
    touch.update(); // Handle touch events and actions
    if (screensaver && touch.touched()) screen.saver_touch(touch.touch_pt(0), touch.touch_pt(1));
    tuner_update(run.mode);
    diag_update();  // notice any screwy conditions or suspicious shenigans
    neo_idiots_update();
    neo.update(colorcard[run.mode], !syspower);
    looptime_mark("-");
    screen.update(run.mode);  // Display updates
    looptime_mark("dis");
    web.update();
    looptime_update();
}