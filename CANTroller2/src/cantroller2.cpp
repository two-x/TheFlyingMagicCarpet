// Carpet CANTroller II  Source Code  - For ESP32-S3-DevKitC-1-N8
#include "globals.h"
#include "sensors.h"  // includes uictrl.h, i2cbus.h
#include "motors.h"  // includes qpid.h, temperature.h
#include "objects.h"  // includes web.h
#include "display.h"  // includes touch.h, neopixel.h
#include "RunModeManager.h"
static NeopixelStrip neo(neopixel_pin);
static TouchScreen touch(touch_cs_pin);
static Display screen(&neo, &touch);
static Tuner tuner(&neo, &touch);
static RunModeManager run(&screen, &encoder);
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
    set_board_defaults();
    hotrc.setup();
    pot.setup();
    encoder.setup();
    pressure.setup();
    brkpos.setup();
    mulebatt.setup();
    lipobatt.setup();
    tach.setup();
    speedo.setup();
    i2c.setup();
    airvelo.setup(); // must be done after i2c is started
    mapsens.setup();
    lightbox.setup();
    tempsens.setup();  // Onewire bus and temp sensors
    xTaskCreate(update_temperature_sensors, "Update Temperature Sensors", 2048, NULL, 5, NULL);  // Temperature sensors task
    sim_setup();  // simulator initialize devices and pot map
    idlectrl.setup(gas.pid.target_ptr(), tach.human_ptr(), tach.filt_ptr(), tempsens.get_sensor(loc::ENGINE), temp_lims_f[ENGINE][OP_MIN], temp_lims_f[ENGINE][WARNING], 50, IdleControl::idlemodes::CONTROL);
    for (int ch=0; ch<4; ch++) ESP32PWM::allocateTimer(ch);
    gas.setup(gas_pwm_pin, 60, &hotrc, &speedo, &tach, &pot, &idlectrl);
    brake.setup(brake_pwm_pin, 50, &hotrc, &speedo, &mulebatt, &pressure, &brkpos);
    steer.setup(steer_pwm_pin, 50, &hotrc, &speedo, &mulebatt);
    prefs.begin("FlyByWire", false);
    datapage = prefs.getUInt("dpage", PG_RUN);
    datapage_last = prefs.getUInt("dpage", PG_TEMP);
    if (display_enabled) screen.setup();
    if (display_enabled) touch.setup(disp_width_pix, disp_height_pix);
    neo.setup();
    int32_t idiots = smin((uint32_t)arraysize(idiotlights), neo.neopixelsAvailable());
    for (int32_t idiot = 0; idiot < idiots; idiot++) neo.newIdiotLight(idiot, idiotcolors[idiot], *(idiotlights[idiot]));
    std::cout << "set up heartbeat led and " << idiots << " neopixel idiot lights" << std::endl;
    for (int32_t i=0; i<NUM_ERR_TYPES; i++) for (int32_t j=0; j<E_NUM_SENSORS; j++) err_sensor[i][j] = false; // Initialize sensor error flags to false
    xTaskCreate(update_web, "Update Web Services", 4096, NULL, 6, NULL);  // wifi/web task. 2048 is too low, it crashes when client connects
    printf("Setup done%s\n", console_enabled ? "" : ". stopping console during runtime");
    if (!console_enabled) Serial.end();  // close serial console to prevent crashes due to error printing
    panicTimer.reset();
    looptime_setup();
}
void loop() {
    ignition_panic_update();  // handler for ignition pin output and panicstop status.
    basicsw_update();
    starter_update();  // Runs starter bidirectional handler
    encoder.update();  // Read encoder input signals
    pot.update();
    brkpos.update();  // Brake position
    tach.update();  // Tach
    speedo.update();  // Speedo
    // lightbox.update(run.mode, speedo.human());
    pressure.update();  // Brake pressure
    mulebatt.update();
    lipobatt.update();
    airvelo.update();
    mapsens.update();  // MAP sensor  // takes 6800 us (!!)
    maf_gps = massairflow();  // Recalculate intake mass airflow
    hotrc.update();
    hotrc_events_update(run.mode);
    if (sim.potmapping(sens::joy)) hotrc.set_pc(HORZ, FILT, pot.mapToRange(steer.pc_to_us(steer.pc[OPMIN]), steer.pc_to_us(steer.pc[OPMAX])));  // Also need to similarly override joyh value if simulating it
    run.mode_logic();  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    gas.update(run.mode);
    brake.update(run.mode);
    steer.update(run.mode);
    touch.update(); // Handle touch events and actions
    tuner.update(run.mode);
    diag_update();  // notice any screwy conditions or suspicious shenigans
    neo.update(colorcard[run.mode], !syspower);
    looptime_mark("-");
    screen.update(run.mode);  // Display updates
    looptime_mark("dis");
    looptime_update();
}