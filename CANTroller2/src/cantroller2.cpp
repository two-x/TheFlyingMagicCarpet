// Carpet CANTroller III  main source Code  - see README.md
#include "objects.h"
#include "unittests.h"
TaskHandle_t temptask = NULL, maftask = NULL, pushtask = NULL, drawtask = NULL, neotask = NULL;

void setup() {             // runs once automatically immediately upon boot
    initialize_boot();     // set pins, read the basic switch (only possible w/o console), then start the serial console for boot msgs. run 1st!
    semaphore_setup();     // needed by display buffering, so run before screen setup
    i2c.setup(known_i2c_addr[I2CTouch], known_i2c_addr[I2CLightbox], known_i2c_addr[I2CAirVelo], known_i2c_addr[I2CMAP]);  // must run before touchscreen setup - inits Wire
    touch.setup(&i2c);      // set up touchscreen object (Wire-based touch driver - see inputs.h) - must run after i2c.setup(), which inits Wire
    screen.setup();        // start up the screen asap so we can monitor the boot progress on the ezread console. no longer touches touch/i2c at all
    xTaskCreatePinnedToCore(push_task, "taskPush", 2048, NULL, 4, &pushtask, 1 - CONFIG_ARDUINO_RUNNING_CORE); // display bus-push task  // 2048 works, 1024 failed // moved off loop()'s core: the screen bus write is slow, and at prio 4 (> loop()'s prio 1) it would otherwise preempt and stall loop() every ~1ms tick
    xTaskCreatePinnedToCore(draw_task, "taskDraw", 4096, NULL, 4, &drawtask, 1 - CONFIG_ARDUINO_RUNNING_CORE); // display drawing task   // 4096 works, 2048 failed
    running_on_devboard = !tempsens.setup();  // initialize onewire bus & temp sensors. The addrs of detected sensors informs if running on vehicle
    xTaskCreatePinnedToCore(tempsens_task, "taskTemp", 4096, NULL, 6, &temptask, 1 - CONFIG_ARDUINO_RUNNING_CORE); // temp sensors read task // 4096 works, 3072 failed — priority 6 is highest of the tasks sharing this core (draw/push/maf), so it wins contention among them; doesn't affect loop(), which is alone on the other core; ESP-IDF internals run above 24
    set_board_defaults();  // changes some configuration options based on whether we're running on the real car
    unittests.run_all();     // runs unit tests for pure-logic code and reports any failures - comment out to skip
    watchdog.setup(&temptask, &drawtask, &pushtask, &maftask, &neotask);  // the watchdog must frequently be pet by the code, or it will assume we crashed & automatically reset it. disabled due to some bug i forgot
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
    // if either i2c sensor above reports nan indefinitely after a reflash, try a full power cycle (not just a reset/reflash) before assuming something's broken - see I2CSensor::_stuck_recovery_timer in sensors.h
    xTaskCreatePinnedToCore(maf_task, "taskMAF", 4096, NULL, 4, &maftask, 1 - CONFIG_ARDUINO_RUNNING_CORE);  // update mass airflow determination, including reading map and airvelo sensors // moved off loop()'s core: mapsens.update()/airvelo.update() are i2c reads with multi-ms waits (up to ~2ms), and at prio 4 (> loop()'s prio 1) would otherwise stall loop() when they periodically wake
    lightbox.setup();      // init object for the lighting control box (an i2c slave)
    starter.setup();       // init handler for the car starter motor
    gas.setup(&hotrc, &speedo, &tach, &pot, &tempsens);
    brake.setup(&hotrc, &speedo, &mulebatt, &pressure, &brkpos, &gas, &tempsens);
    steer.setup(&hotrc, &speedo, &mulebatt);
    sim_setup();           // simulator initialize devices and pot map
    neo.setup();          // init neopixels
    xTaskCreatePinnedToCore(neo_task, "taskNeo", 4096, NULL, 4, &neotask, 1 - CONFIG_ARDUINO_RUNNING_CORE); // neopixel update/send task // moved off loop()'s core: neoobj.Show() is a blocking write to the RMT hw peripheral (~300-400us), same class of offender as taskTemp/taskMAF
    idiots.setup();        // print idiot lights count to boot log
    diag.setup();          // initialize diagnostic engine
    ignition.setup();      // must be after diag setup
    // fan.setup();           // init vehicle thermostat controlling radiator cooling fan
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
    hotrc.update();        // ~100us for all hotrc functions. run just before run.update()
    run.update();          // runmode state machine. Gas/brake control targets are determined here.  - takes 36 us in standby mode with no activity
    gas.update();          // drive servo output based on controller inputs, idle controller, (possible) feedback, run mode, etc.
    brake.update();        // drive motor output based on controller inputs, feedback, run mode, etc.
    steer.update();        // drive motor output based on controller inputs, run mode, etc.
    touch.update();        // read touchscreen input and do what it tells us to
    tuner.update();        // if tuning edits are instigated by the encoder or touch, modify the corresponding variable values
    diag.update();         // notice any screwy conditions or suspicious shenanigans - consistent 200us
    // neo.update() no longer called here — moved to its own taskNeo (see setup()), since neoobj.Show() blocks on the RMT peripheral
    // lightbox.update() no longer called here — moved into taskMAF (see objects.h), grouped with the other i2c peripherals off loop()'s core
    // fan.update();          // update vehicle thermostat controlling radiator cooling fan
    ezread.update();       // allow the ezread spam detector to evaporate some old data from its buffer
    looptimer.update();    // looptimer.mark("F");
 }  // vTaskDelay: not needed — loop() has core CONFIG_ARDUINO_RUNNING_CORE all to itself now (all other tasks are pinned to the opposite core), so yielding here does not help any other task and wastes ~1ms per loop