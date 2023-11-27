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
    set_pin(syspower_pin, OUTPUT, syspower);
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
    idlectrl.setup(gas.pid.target_ptr(), tach.human_ptr(), tach.filt_ptr(), tempsens.get_sensor(loc::ENGINE), temp_lims_f[ENGINE][OPMIN], temp_lims_f[ENGINE][WARNING], 50, IdleControl::idlemodes::CONTROL);
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
    idiotlights_setup(&neo);
    web.setup();
    xTaskCreate(update_web, "Update Web Services", 4096, NULL, 6, NULL);  // wifi/web task. 2048 is too low, it crashes when client connects
    printf("Setup done%s\n", console_enabled ? "" : ". stopping console during runtime");
    if (!console_enabled) Serial.end();  // close serial console to prevent crashes due to error printing
    looptime_setup();
}
void loop() {
    ignition_panic_update();  // handler for ignition pin output and panicstop status.
    basicsw_update();  // handler for basic mode switch
    starter_update();  // Runs starter bidirectional handler  // time for 3x handler functions up to here = 110 us
    encoder.update();  // Read encoder input signals  // 20 us per loop
    pot.update();  // consistent 400 us per loop for analog read operation. we only see this for the pot (!?) changing pins is no help 
    brkpos.update();  // Brake position (consistent 120 us)
    pressure.update();  // Brake pressure  // ~50 us
    tach.update();  // Tach
    speedo.update();  // Speedo
    mulebatt.update();
    lipobatt.update();  // tach + speedo + mulebatt + lipobatt = 120 us
    airvelo.update();  // Air velocity sensor  // 20us + 900us every 4 loops
    mapsens.update();  // Manifold air pressure sensor  // 70 us + 2ms every 9 loops
    maf_gps = massairflow();  // Recalculate intake mass airflow
    hotrc.update();  // ~100us for all hotrc functions
    hotrc_events_update(run.mode);
    if (sim.potmapping(sens::joy)) hotrc.set_pc(HORZ, FILT, pot.mapToRange(steer.pc_to_us(steer.pc[OPMIN]), steer.pc_to_us(steer.pc[OPMAX])));
    run.mode_logic();  // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    gas.update(run.mode);
    brake.update(run.mode);
    steer.update(run.mode);
    touch.update(); // Handle touch events and actions
    tuner.update(run.mode);
    diag_update();  // notice any screwy conditions or suspicious shenigans - consistent 200us
    neo.update(colorcard[run.mode], !syspower);  // ~100us
    screen.update(run.mode);  // Display updates (50us + 3.5ms every 8 loops. screensaver add 15ms every 4 loops)
    // lightbox.update(run.mode, speedo.human());
    looptime_update();  // looptime_mark("F");
}