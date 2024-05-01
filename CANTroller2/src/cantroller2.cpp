// Carpet CANTroller III  main source Code  - see README.md
#include "objects.h"
TaskHandle_t temptask = nullptr, webtask = nullptr, pushTaskHandle = NULL, drawTaskHandle = NULL;

void setup() {
    initialize_pins();
    running_on_devboard = (read_pin(uart_tx_pin));  // detect breadboard vs. real car without use of an additional pin (add weak pullup resistor on your breadboard)
    Serial.begin(115200);      // open console serial port (will reassign tx pin as output)
    delay(1000);               // This is needed to allow the uart to initialize and the screen board enough time after a cold boot
    partition_table();
    set_board_defaults();      // set variables as appropriate if on a breadboard
    run_tests();
    psram_setup();
    semaphore_setup();
    watchdog.setup(&temptask, &webtask, &drawTaskHandle, &pushTaskHandle);
    bootbutton.setup();
    hotrc.setup();
    pot.setup();
    encoder.setup();
    pressure.setup();
    brkpos.setup();
    mulebatt.setup();
    tach.setup();
    speedo.setup();
    i2c.setup(touch.addr, lightbox.addr, airvelo.addr, mapsens.addr);
    airvelo.setup();          // must be done after i2c is started
    mapsens.setup();
    lightbox.setup();
    fuelpump.setup();
    starter.setup();
    tempsens.setup();         // onewire bus and temp sensors
    xTaskCreatePinnedToCore(update_temperature_sensors, "Update Temp Sensors", 2048, NULL, 6, &temptask, 1 - CONFIG_ARDUINO_RUNNING_CORE);  // Temperature sensors task  // 2048 works, 1024 failed,  priority is from 0 to 24=highest
    for (int ch=0; ch<4; ch++) ESP32PWM::allocateTimer(ch);  // added for servos I think
    gas.setup(&hotrc, &speedo, &tach, &pot, &tempsens);
    brake.setup(&hotrc, &speedo, &mulebatt, &pressure, &brkpos, &gas, &tempsens);
    steer.setup(&hotrc, &speedo, &mulebatt);
    datapage = prefs.getUInt("dpage", PG_RUN);
    datapage_last = prefs.getUInt("dpage", PG_TEMP);
    sim_setup();              // simulator initialize devices and pot map
    touch.setup(&lcd, &i2c, disp_width_pix, disp_height_pix);
    screen.setup();
    #if VIDEO_TASKS
        xTaskCreatePinnedToCore(push_task_wrapper, "taskPush", 2048, NULL, 4, &pushTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);  // 2048 works, 1024 failed
        xTaskCreatePinnedToCore(draw_task_wrapper, "taskDraw", 4096, NULL, 4, &drawTaskHandle, 1 - CONFIG_ARDUINO_RUNNING_CORE);  // 4096 works, 2048 failed
    #endif
    neo.setup();              // set up external neopixel strip for idiot lights visible in daylight from top of carpet
    idiots.setup(&neo);       // assign same idiot light variable associations and colors to neopixels as on screen  
    diag.setup();             // initialize diagnostic engine
    run.setup();              // initialize runmode state machine
    web.setup();              // start up access point, web server, and json-enabled web socket for diagnostic phone interface
    xTaskCreatePinnedToCore(update_web, "Update Web Services", 4096, NULL, 6, &webtask, CONFIG_ARDUINO_RUNNING_CORE);  // wifi/web task. with 4096 wifi runs but fails to connect (maybe unrelated?).  2048 is too low, it crashes when client connects  16384
    printf("** Setup done%s\n", console_enabled ? "" : ". stopping console during runtime");
    if (!console_enabled) Serial.end();  // close serial console to prevent crashes due to error printing
    looptimer.setup();
}
void loop() {                 // arduino-style loop() is like main() but with a builtin infinite while(1) loop.
    watchdog.update();           // pet the watchdog regularly to prevent reset
    ignition.update(run.mode);  // manage panic stop condition and drive ignition signal as needed
    bootbutton.update();      // read the builtin button

    // temporarily added for development convenience
    if (bootbutton.longpress()) screen.auto_saver(!auto_saver_enabled);
    if (bootbutton.shortpress()) {
        if (auto_saver_enabled) animations.change_saver();
        else sim.toggle();
    }
    
    basicsw_update();         // see if basic mode switch got hit
    starter.update();         // read or drive starter motor  // total for all 3 digital signal handlers is 110 us
    encoder.update();         // read encoder input signals  // 20 us per loop
    pot.update();             // consistent 400 us per loop for analog read operation. we only see this for the pot (!?) changing pins is no help 
    fuelpump.update();        // drives power to the fuel pump when the engine is turning
    brkpos.update();          // brake position (consistent 120 us)
    pressure.update();        // brake pressure  // ~50 us
    tach.update();            // get pulse timing from hall effect tachometer on flywheel
    speedo.update();          // get pulse timing from hall effect speedometer on axle
    mulebatt.update();        // vehicle battery voltage
    airvelo.update();         // manifold air velocity sensor  // 20us + 900us every 4 loops
    mapsens.update();         // manifold air pressure sensor  // 70 us + 2ms every 9 loops
    maf_gps = massairflow();  // calculate grams/sec of air molecules entering the engine (Mass Air Flow) using velocity, pressure, and temperature of manifold air 
    hotrc.update();           // ~100us for all hotrc functions
    run.mode_logic();         // runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in shutdown mode with no activity
    gas.update();             // drive servo output based on controller inputs, idle controller, (possible) feedback, run mode, etc.
    brake.update();           // drive motor output based on controller inputs, feedback, run mode, etc.
    steer.update();           // drive motor output based on controller inputs, run mode, etc.
    touch.update();           // read touchscreen input and do what it tells us to
    tuner.update(run.mode);   // if tuning edits are instigated by the encoder or touch, modify the corresponding variable values
    diag.update(run.mode);            // notice any screwy conditions or suspicious shenanigans - consistent 200us
    neo.update(colorcard[run.mode]);  // ~100us
    screen.update(run.mode);  // display updates (50us + 3.5ms every 8 loops. screensaver add 15ms every 4 loops)
    lightbox.update(run.mode, speedo.human());  // communicate any relevant data to the lighting controller
    looptimer.update();       // looptimer.mark("F");
}