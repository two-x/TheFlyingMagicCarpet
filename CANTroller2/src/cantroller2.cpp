// Carpet CANTroller III  main source Code  - see README.md
#include "objects.h"
TaskHandle_t temptask = NULL, maftask = NULL, pushtask = NULL, drawtask = NULL;

void setup() {             // runs once automatically immediately upon boot
    initialize_boot();     // set pins, read the basic switch (only possible w/o console), then start the serial console for boot msgs. run 1st!
    semaphore_setup();     // needed by display buffering, so run before screen setup
    i2c.setup(touch.addr, lightbox.addr, airvelo.addr, mapsens.addr);  // run early as i2c addrs used to autodetect touchscreen type, & other sensors
    touch.setup(&lcd, &i2c); // set up touchscreen object. must run before screen driver, which is also driver for touchscreen
    screen.setup();        // start up the screen asap so we can monitor the boot progress on the ezread console. also inits touchscreen
    xTaskCreatePinnedToCore(push_task, "taskPush", 2048, NULL, 4, &pushtask, CONFIG_ARDUINO_RUNNING_CORE);     // display bus-push task  // 2048 works, 1024 failed
    xTaskCreatePinnedToCore(draw_task, "taskDraw", 4096, NULL, 4, &drawtask, 1 - CONFIG_ARDUINO_RUNNING_CORE); // display drawing task   // 4096 works, 2048 failed
    running_on_devboard = !tempsens.setup();  // initialize onewire bus & temp sensors. The addrs of detected sensors informs if running on vehicle
    xTaskCreatePinnedToCore(tempsens_task, "taskTemp", 4096, NULL, 6, &temptask, 1 - CONFIG_ARDUINO_RUNNING_CORE); // temp sensors read task // 4096 works, 3072 failed,  priority is from 0 to 24=highest    
    set_board_defaults();  // changes some configuration options based on whether we're running on the real car
    run_tests();           // runs some sanity checks to ensure the code is self-consistent - Anders wrote this but it needs expansion
    watchdog.setup(&temptask, &drawtask, &pushtask, &maftask);  // the watchdog must frequently be pet by the code, or it will assume we crashed & automatically reset it. disabled due to some bug i forgot
    bootbutton.setup();    // init button on the esp board, for misc use
    hotrc.setup();         // init hotrc remote control handle, source of steering/throttle/brake commands (how we drive the car)
    pot.setup();           // init potentiometer knob which is a user input device, used for adjusting analog values including emulating sensor or motor activity
    encoder.setup();       // init rotary encoder which is a user input device, used for digital adjustments including graphical ui navigation
    pressure.setup();      // init analog brake pressure sensor, used as feedback to control application of brake
    brkpos.setup();        // init analog brake position sensor, used as feedback to control release of brake
    mulebatt.setup();      // init car battery voltage monitor
    tach.setup();          // init tachometer, which is based on pulses from a magnet on the crankshaft passing near a hall effect sensor
    speedo.setup();        // init speedometer, which is based on pulses from magnets on the drive axle passing near a hall effect sensor
    airvelo.setup();       // init i2c air velocity sensor, to calc mass airflow value needed for proper throttle pid feedback (currently only monitored)
    mapsens.setup();       // init i2c manifold air pressure sensor, to calc mass airflow value needed for proper throttle pid feedback (currently only monitored)
    xTaskCreatePinnedToCore(maf_task, "taskMAF", 4096, NULL, 4, &maftask, CONFIG_ARDUINO_RUNNING_CORE);  // update mass airflow determination, including reading map and airvelo sensors
    lightbox.setup();      // init object for the lighting control box (an i2c slave)
    starter.setup();       // init handler for the car starter motor
    gas.setup(&hotrc, &speedo, &tach, &pot, &tempsens);
    brake.setup(&hotrc, &speedo, &mulebatt, &pressure, &brkpos, &gas, &tempsens);
    steer.setup(&hotrc, &speedo, &mulebatt);
    sim_setup();           // simulator initialize devices and pot map
    neo.setup();           // set up external neopixel strip for idiot lights visible in daylight from top of carpet
    idiots.setup(&neo);    // assign same idiot light variable associations and colors to neopixels as on screen  
    diag.setup();          // initialize diagnostic engine
    ignition.setup();      // must be after diag setup
    run.setup();           // initialize runmode state machine. must be after diag setup
    finalize_boot();       // will stop the console if appropriate
}
void loop() {              // arduino-style loop() is like main() but with a builtin infinite while(1) loop
    watchdog.update();     // pet the watchdog regularly to prevent reset
    ignition.update();     // manage panic stop condition and drive ignition signal as needed
    bootbutton.update();   // read the builtin button
    starter.update();      // read or drive starter motor  // total for all 3 digital signal handlers is 110 us
    encoder.update();      // read encoder input signals  // 20 us per loop
    pot.update();          // consistent 400 us per loop for analog read operation. we only see this for the pot (!?) changing pins is no help 
    brkpos.update();       // brake position (consistent 120 us)
    pressure.update();     // brake pressure  // ~50 us
    tach.update();         // get pulse timing from hall effect tachometer on flywheel
    speedo.update();       // get pulse timing from hall effect speedometer on axle
    mulebatt.update();     // vehicle battery voltage
    hotrc.update();        // ~100us for all hotrc functions.
    run.mode_logic();      // runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in standby mode with no activity
    gas.update();          // drive servo output based on controller inputs, idle controller, (possible) feedback, run mode, etc.
    brake.update();        // drive motor output based on controller inputs, feedback, run mode, etc.
    steer.update();        // drive motor output based on controller inputs, run mode, etc.
    touch.update();        // read touchscreen input and do what it tells us to
    tuner.update();        // if tuning edits are instigated by the encoder or touch, modify the corresponding variable values
    diag.update();         // notice any screwy conditions or suspicious shenanigans - consistent 200us
    neo.update();          // update/send neopixel colors 
    lightbox.update(speedo.val());  // communicate any relevant data to the lighting controller
    ezread.update();       // allow the ezread spam detector to evaporate some old data from its buffer
    looptimer.update();    // looptimer.mark("F");
 }  // vTaskDelay(pdMS_TO_TICKS(1));   // pause continuous execution for 1ms to allow other tasks some cpu time