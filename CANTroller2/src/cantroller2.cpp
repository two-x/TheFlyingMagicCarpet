// Carpet CANTroller II  Source Code  - For ESP32-S3-DevKitC-1-N8
#include "objects.h"
#include "display.h"  // includes neopixel.h, touch.h
#include "RunModeManager.h"
static NeopixelStrip neo(neopixel_pin);
static IdiotLights idiots;
static TouchScreen touch(touch_cs_pin);
static Display screen(&neo, &touch, &idiots);
static Tuner tuner(&neo, &touch);
static RunModeManager run(&screen, &encoder);

void setup() {
    initialize_pins();
    running_on_devboard = (read_pin(uart_tx_pin));  // detect breadboard vs. real car without use of an additional pin (add weak pullup resistor on your breadboard)
    Serial.begin(115200);  // Open console serial port (will reassign tx pin as output)
    delay(800);            // This is needed to allow the uart to initialize and the screen board enough time after a cold boot
    set_board_defaults();  // set variables as appropriate if on a breadboard
    if (RUN_TESTS) run_tests();
    hotrc.setup();
    pot.setup();
    encoder.setup();
    pressure.setup();
    brkpos.setup();
    mulebatt.setup();
    tach.setup();
    speedo.setup();
    i2c.setup();
    airvelo.setup(); // must be done after i2c is started
    mapsens.setup();
    lightbox.setup();
    tempsens.setup();  // Onewire bus and temp sensors
    xTaskCreate(update_temperature_sensors, "Update Temperature Sensors", 2048, NULL, 5, NULL);  // Temperature sensors task
    sim_setup();  // simulator initialize devices and pot map
    for (int ch=0; ch<4; ch++) ESP32PWM::allocateTimer(ch);
    gas.setup(&hotrc, &speedo, &tach, &pot, &tempsens);
    brake.setup(&hotrc, &speedo, &mulebatt, &pressure, &brkpos);
    steer.setup(&hotrc, &speedo, &mulebatt);
    prefs.begin("FlyByWire", false);
    datapage = prefs.getUInt("dpage", PG_RUN);
    datapage_last = prefs.getUInt("dpage", PG_TEMP);
    if (display_enabled) screen.setup();
    if (display_enabled) touch.setup(disp_width_pix, disp_height_pix);
    neo.setup();              // set up external neopixel strip for idiot lights visible in daylight from top of carpet
    idiots.setup(&neo);       // assign same idiot light variable associations and colors to neopixels as on screen  
    diag.setup();             // initialize diagnostic codes
    web.setup();              // start up access point, web server, and json-enabled web socket for diagnostic phone interface
    xTaskCreate(update_web, "Update Web Services", 4096, NULL, 6, NULL);  // wifi/web task. 2048 is too low, it crashes when client connects
    printf("Setup done%s\n", console_enabled ? "" : ". stopping console during runtime");
    if (!console_enabled) Serial.end();  // close serial console to prevent crashes due to error printing
    looptimer.setup();
}
void loop() {                 // code takes about 1 ms to loop on average
    ignition_panic_update();  // manage panic stop condition and drive ignition signal as needed
    basicsw_update();         // see if basic mode switch got hit
    starter_update();         // read or drive starter motor  // total for all 3 digital signal handlers is 110 us
    encoder.update();         // read encoder input signals  // 20 us per loop
    pot.update();             // consistent 400 us per loop for analog read operation. we only see this for the pot (!?) changing pins is no help 
    brkpos.update();          // brake position (consistent 120 us)
    pressure.update();        // brake pressure  // ~50 us
    tach.update();            // get pulse timing from hall effect tachometer on flywheel
    speedo.update();          // get pulse timing from hall effect speedometer on axle
    mulebatt.update();        // vehicle battery voltage
    airvelo.update();         // manifold air velocity sensor  // 20us + 900us every 4 loops
    mapsens.update();         // manifold air pressure sensor  // 70 us + 2ms every 9 loops
    maf_gps = massairflow();  // calculate grams/sec of air molecules entering the engine (Mass Air Flow) using velocity, pressure, and temperature of manifold air 
    hotrc.update();           // ~100us for all hotrc functions
    hotrc_events(run.mode);   // turn hotrc button events into handler requests depending on the runmode
    if (sim.potmapping(sens::joy)) hotrc.set_pc(HORZ, FILT, pot.mapToRange(steer.pc_to_us(steer.pc[OPMIN]), steer.pc_to_us(steer.pc[OPMAX])));
    run.mode_logic();         // Runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    gas.update(run.mode);     // drive servo output based on controller inputs, idle controller, (possible) feedback, run mode, etc.
    brake.update(run.mode);   // drive motor output based on controller inputs, feedback, run mode, etc.
    steer.update(run.mode);   // drive motor output based on controller inputs, run mode, etc.
    touch.update();           // read touchscreen input and do what it tells us to
    tuner.update(run.mode);   // if tuning edits are instigated by the encoder or touch, modify the corresponding variable values
    diag.update();            // notice any screwy conditions or suspicious shenanigans - consistent 200us
    neo.update(colorcard[run.mode]);  // ~100us
    screen.update(run.mode);  // Display updates (50us + 3.5ms every 8 loops. screensaver add 15ms every 4 loops)
    // lightbox.update(run.mode, speedo.human());  // communicate any relevant data to the lighting controller
    looptimer.update();       // looptimer.mark("F");
}