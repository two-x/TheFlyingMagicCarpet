#pragma once
#include <type_traits>
// Unit tests for pure-logic (hardware-independent) code and structural code-consistency checks. Run once at boot via unittests.setup() (see
// cantroller2.cpp) - comment out that one line to skip entirely. A failing check never halts, blocks, or reboots the device: it's logged and we
// move on, so a bug here can never prevent the rest of setup() (and the vehicle) from continuing normally. Failures print sadcolor by default (a
// coding bug, but not inherently dangerous on its own) - checks that guard something with real safety implications (e.g. the constrain() clamp
// primitive that every actuator/sensor limit in this codebase is built on) print madcolor instead. Each test_* method covers one class or file
// section, printed as a highlightcolor heading matching the rest of the boot log, with FAIL lines indented 2 spaces and starting with (file:line).
//
// Scope note: this covers pure logic/math (Timer, Param, QPID, generic utility functions, color conversions, mass airflow math) and structural
// consistency (class hierarchy via compile-time static_assert, enum-labeled lookup arrays being fully populated, runmode-indexed tables holding
// valid values). It deliberately does NOT do broad simulated/swept-input testing looking for emergent bad behavior (e.g. "does this value ever
// decrease when it shouldn't across a range of inputs") - that's a different kind of test (a functional/property test, not a consistency check)
// and belongs in a separate future test suite. It also doesn't cover anything that touches real hardware (i2c sensors, motor PWM output,
// encoder/touch input, the i2c bus itself) - testing that meaningfully would need mock/fake hardware behind an interface, which none of those
// classes currently have - a bigger, separate undertaking if ever wanted.

// --- Class hierarchy checks: these are compile-time (static_assert), not runtime, so they're checked on every single build regardless of whether
// unittests.setup() ever runs, and a broken hierarchy is a hard build error rather than something you have to notice in a log. ---
static_assert(std::is_base_of<Device, Transducer>::value, "Transducer must inherit from Device");
static_assert(std::is_base_of<Transducer, Sensor>::value, "Sensor must inherit from Transducer");
static_assert(std::is_base_of<Sensor, I2CSensor>::value, "I2CSensor must inherit from Sensor");
static_assert(std::is_base_of<I2CSensor, AirVeloSensor>::value, "AirVeloSensor must inherit from I2CSensor");
static_assert(std::is_base_of<I2CSensor, MAPSensor>::value, "MAPSensor must inherit from I2CSensor");
static_assert(std::is_base_of<Sensor, AnalogSensor>::value, "AnalogSensor must inherit from Sensor");
static_assert(std::is_base_of<AnalogSensor, CarBattery>::value, "CarBattery must inherit from AnalogSensor");
static_assert(std::is_base_of<AnalogSensor, PressureSensor>::value, "PressureSensor must inherit from AnalogSensor");
static_assert(std::is_base_of<AnalogSensor, BrakePositionSensor>::value, "BrakePositionSensor must inherit from AnalogSensor");
static_assert(std::is_base_of<Sensor, PulseSensor>::value, "PulseSensor must inherit from Sensor");
static_assert(std::is_base_of<PulseSensor, Tachometer>::value, "Tachometer must inherit from PulseSensor");
static_assert(std::is_base_of<PulseSensor, Speedometer>::value, "Speedometer must inherit from PulseSensor");
static_assert(std::is_base_of<ServoMotor, JagMotor>::value, "JagMotor must inherit from ServoMotor");
static_assert(std::is_base_of<ServoMotor, ThrottleControl>::value, "ThrottleControl must inherit from ServoMotor");
static_assert(std::is_base_of<JagMotor, BrakeControl>::value, "BrakeControl must inherit from JagMotor");
static_assert(std::is_base_of<JagMotor, SteeringControl>::value, "SteeringControl must inherit from JagMotor");
static_assert(std::is_base_of<ToggleSwitch, BasicModeSwitch>::value, "BasicModeSwitch must inherit from ToggleSwitch");
static_assert(std::is_base_of<MomentarySwitch, BootButton>::value, "BootButton must inherit from MomentarySwitch");
// Note: Param has no subclasses anywhere in the codebase (checked) - nothing to verify there.

// Motor pc[]/si[]/us[]/volt[] arrays (globals.h motor_val enum, Out=3 onward) are indexed 0-2 by two OTHER separate, unscoped enums (hotrc_val's
// OpMin=0/OpMax=2, and stop_val's Stop=1) rather than motor_val itself - these three enums only work together because their numeric values happen
// to line up. Editing any one of them (e.g. inserting a new hotrc_val member before OpMax) would silently shift every motor pc[]/si[]/us[] array's
// indexing without any compile error. This is a hard guarantee that they still line up, checked on every build.
static_assert(OpMin == 0 && Stop == 1 && OpMax == 2 && Out == 3, "hotrc_val/stop_val/motor_val's shared index space (OpMin/Stop/OpMax/Out) has drifted out of alignment - motor pc[]/si[]/us[] arrays index by this");

class UnitTests {
  private:
    int checks = 0, fails = 0;
    void check(bool cond, bool safety, const char* file, int line, const char* condstr) {
        checks++;
        if (!cond) {
            fails++;
            ezread.squintf(safety ? ezread.madcolor : ezread.sadcolor, "  (%s:%d) %s\n", file, line, condstr);
        }
    }
    #define UT_CHECK(cond) check((cond), false, __FILE__, __LINE__, #cond)
    #define UT_CHECK_SAFETY(cond) check((cond), true, __FILE__, __LINE__, #cond)

    // For enum-labeled lookup arrays (e.g. std::string modecard[NumRunModes] = {...}): if you add an enum value but forget to add a matching
    // label, the array (already correctly sized, since it's declared with the enum's own Num* sentinel) just silently zero-fills the missing
    // entry with an empty string rather than erroring. This walks every entry and flags any that are empty, which is the sign of exactly that.
    void check_card_complete(const std::string* arr, int count, const char* name, const char* file, int line) {
        for (int i = 0; i < count; i++) {
            checks++;
            if (arr[i].empty()) {
                fails++;
                ezread.squintf(ezread.sadcolor, "  (%s:%d) %s[%d] is empty - missing label for an enum value?\n", file, line, name, i);
            }
        }
    }
    #define UT_CARD_COMPLETE(arr, count) check_card_complete(arr, count, #arr, __FILE__, __LINE__)

    void test_utility_functions() {  // globals.h
        // iszero()
        UT_CHECK(iszero(0.0f));
        UT_CHECK(iszero(0.00001f));
        UT_CHECK(!iszero(1.0f));
        UT_CHECK(iszero(0.05f, 0.1f));   // within a given margin
        UT_CHECK(!iszero(0.2f, 0.1f));   // outside a given margin
        // isinfinite()
        UT_CHECK(isinfinite(100000000.0f));
        UT_CHECK(!isinfinite(1.0f));
        UT_CHECK(!isinfinite(0.0f));     // zero is explicitly guarded, not treated as infinite
        // cleanzero()
        float a = 0.00001f;
        cleanzero(&a);
        UT_CHECK(a == 0.0f);
        float b = 1.0f;
        cleanzero(&b);
        UT_CHECK(b == 1.0f);
        // constrain() - the shared clamp primitive nearly every actuator/sensor limit in this codebase is built on
        UT_CHECK_SAFETY(constrain(5.0f, 0.0f, 10.0f) == 5.0f);
        UT_CHECK_SAFETY(constrain(-5.0f, 0.0f, 10.0f) == 0.0f);
        UT_CHECK_SAFETY(constrain(15.0f, 0.0f, 10.0f) == 10.0f);
        UT_CHECK_SAFETY(constrain(5, 0, 10) == 5);
        UT_CHECK_SAFETY(constrain(-5, 0, 10) == 0);
        UT_CHECK_SAFETY(constrain(15, 0, 10) == 10);
        UT_CHECK_SAFETY(constrain(5L, 0L, 10L) == 5L);
        UT_CHECK_SAFETY(constrain(15L, 0L, 10L) == 10L);
        UT_CHECK_SAFETY(constrain(5u, 0u, 10u) == 5u);
        UT_CHECK_SAFETY(constrain(15u, 0u, 10u) == 10u);
        // map() float
        UT_CHECK(std::fabs(map(5.0f, 0.0f, 10.0f, 0.0f, 100.0f) - 50.0f) < 0.001f);
        UT_CHECK(std::fabs(map(0.0f, 0.0f, 10.0f, 0.0f, 100.0f) - 0.0f) < 0.001f);
        UT_CHECK(std::fabs(map(10.0f, 0.0f, 10.0f, 0.0f, 100.0f) - 100.0f) < 0.001f);
        UT_CHECK(std::isnan(map(NAN, 0.0f, 10.0f, 0.0f, 100.0f)));   // nan input -> nan, not a crash
        UT_CHECK(std::isnan(map(5.0f, 5.0f, 5.0f, 0.0f, 100.0f)));   // zero-width input range (would divide by zero) -> nan, not a crash
        // map() int
        UT_CHECK(map(5, 0, 10, 0, 100) == 50);
        UT_CHECK(map(5, 5, 5, 0, 100) == 100);   // zero-width input range -> documented out_max fallback, not a crash
        // ema_filt()
        UT_CHECK(std::fabs(ema_filt(10.0f, 0.0f, 1.0f) - 10.0f) < 0.001f);    // alpha=1 -> pure raw
        UT_CHECK(std::fabs(ema_filt(10.0f, 0.0f, 0.0f) - 0.0f) < 0.001f);     // alpha=0 -> filt unchanged
        UT_CHECK(std::fabs(ema_filt(10.0f, 0.0f, 0.5f) - 5.0f) < 0.001f);     // alpha=0.5 -> halfway
        UT_CHECK(std::fabs(ema_filt(10.0f, 0.0f, 2.0f) - 10.0f) < 0.001f);    // out-of-range alpha gets constrained to 1.0
        UT_CHECK(!std::isnan(ema_filt(NAN, 5.0f, 0.5f)));                    // nan raw -> keeps filt (5.0), doesn't propagate nan
        UT_CHECK(std::fabs(ema_filt(NAN, 5.0f, 0.5f) - 5.0f) < 0.001f);
        UT_CHECK(std::isnan(ema_filt(10.0f, NAN, 0.5f)));                    // nan filt -> nan (nothing valid to recover)
    }

    void test_Timer() {  // globals.h
        Timer t1(100000);  // 100ms timeout
        UT_CHECK(t1.timeout() == 100000);
        UT_CHECK(!t1.expired());  // freshly created, shouldn't be expired
        UT_CHECK(t1.elapsed() >= 0);
        Timer t2(1000);  // 1ms timeout, short enough to actually observe expiry without a long test-time delay
        delay(3);        // 3ms > 1ms timeout
        UT_CHECK(t2.expired());
        UT_CHECK(t2.elapsed() >= 1000);
        bool was_expired = t2.expireset();  // should report expired, and reset the timer as a side effect
        UT_CHECK(was_expired);
        UT_CHECK(!t2.expired());  // immediately after expireset()'s reset, should no longer read as expired
        Timer t3(500);
        t3.set(50000);  // set() should both change the timeout and restart the clock
        UT_CHECK(t3.timeout() == 50000);
        UT_CHECK(!t3.expired());
    }

    void test_Param() {  // sensors.h
        // default constructor
        Param p1;
        UT_CHECK(p1.val() == 0.0f);
        UT_CHECK(p1.min() == 0.0f);
        UT_CHECK(p1.max() == 0.0f);
        // value-only constructor: constant Param (min==max==val), cannot be changed
        Param p2(13.0f);
        UT_CHECK(p2.val() == 13.0f);
        UT_CHECK(p2.min() == 13.0f);
        UT_CHECK(p2.max() == 13.0f);
        UT_CHECK_SAFETY(!p2.set(20.0f));         // out of (zero-width) range - rejected, value unchanged
        UT_CHECK(p2.val() == 13.0f);
        // value+limits constructor
        Param p3(5.0f, 1.0f, 10.0f);
        UT_CHECK(p3.val() == 5.0f);
        UT_CHECK(p3.min() == 1.0f);
        UT_CHECK(p3.max() == 10.0f);
        UT_CHECK_SAFETY(p3.set(3.0f));           // in range - accepted, and set() reports a real change
        UT_CHECK(p3.val() == 3.0f);
        UT_CHECK_SAFETY(!p3.set(3.0f));          // same value again - set() correctly reports "no change"
        // set_limits() clamping behavior - this enforcement is what every downstream actuator/sensor limit relies on
        UT_CHECK_SAFETY(p3.set(20.0f) == false || p3.val() == 10.0f);  // regardless of set()'s return, value must never exceed max
        p3.set_limits(0.0f, 2.0f);               // narrow the range below the current value
        UT_CHECK_SAFETY(p3.val() <= 2.0f);        // constrain_value() must pull the current value back within new limits
        UT_CHECK_SAFETY(p3.min() == 0.0f && p3.max() == 2.0f);
        // set_limits() rejects an inverted range rather than accepting nonsense limits
        Param p4(5.0f, 0.0f, 10.0f);
        p4.set_limits(8.0f, 3.0f, false);  // min > max: invalid, should be rejected (logged, not applied)
        UT_CHECK_SAFETY(p4.min() == 0.0f && p4.max() == 10.0f);  // limits unchanged from before the rejected call
        // externally shared limits: two Params can reference the same live min/max - plo/phi need an actual range (not the value-only
        // constant-Param constructor) since the test below calls plo.set() expecting it to really take effect
        Param plo(2.0f, 0.0f, 100.0f), phi(8.0f, 0.0f, 100.0f);
        Param p5(5.0f, plo.ptr(), phi.ptr());
        UT_CHECK(p5.min() == 2.0f && p5.max() == 8.0f);
        plo.set(4.0f);  // change the referenced limit...
        UT_CHECK_SAFETY(p5.min() == 4.0f);  // ...and p5 should see it live, since it points at plo's value directly
        // set() nan handling
        Param p6(5.0f, 0.0f, 10.0f);
        UT_CHECK(p6.set(NAN));         // nan is considered a real change from a valid value
        UT_CHECK(std::isnan(p6.val()));
        UT_CHECK(!p6.set(NAN));        // nan -> nan again is correctly reported as no change
    }

    void test_PulseSensor_math() {  // sensors.h - us_to_hz/scaling_ema_filt are protected, exposed here via a throwaway test-only subclass
        class Accessor : public Tachometer {
          public:
            Accessor() : Tachometer(-1, 1.0f) {}  // pin -1: never calls setup()/presetup(), so no real hardware is touched
            using PulseSensor::us_to_hz;
            using PulseSensor::scaling_ema_filt;
        } acc;
        UT_CHECK(std::fabs(acc.us_to_hz(1000.0f) - 1000.0f) < 0.001f);      // 1000us period -> 1000Hz
        UT_CHECK(std::fabs(acc.us_to_hz(1000000.0f) - 1.0f) < 0.001f);      // 1s period -> 1Hz
        UT_CHECK(std::isnan(acc.us_to_hz(0.0f)));                          // zero period is invalid -> nan
        UT_CHECK(acc.us_to_hz(NAN) == 0.0f);                                // nan period means "timed out" -> 0Hz by design, not nan
        UT_CHECK(std::fabs(acc.scaling_ema_filt(10.0f, 0.0f, NAN, 100.0f) - 0.0f) < 0.001f);   // nan dt -> bypass, filt unchanged
        UT_CHECK(std::fabs(acc.scaling_ema_filt(10.0f, 0.0f, 0.0f, 100.0f) - 0.0f) < 0.001f);  // zero dt -> bypass, filt unchanged
        UT_CHECK(std::fabs(acc.scaling_ema_filt(10.0f, 0.0f, 100.0f, 100.0f) - 5.0f) < 0.001f); // dt==tau -> alpha=0.5 -> halfway
    }

    void test_color_conversions() {  // idiots.h
        // full-scale white and black are the least ambiguous cases to hand-verify (every bit lane either fully set or fully clear)
        UT_CHECK(color_to_565((uint32_t)0xFFFFFF) == 0xFFFF);
        UT_CHECK(color_to_332((uint32_t)0xFFFFFF) == 0xFF);
        UT_CHECK(color_to_565((uint32_t)0x000000) == 0x0000);
        UT_CHECK(color_to_332((uint32_t)0x000000) == 0x00);
        neorgb_t white888 = color_to_neo((uint32_t)0xFFFFFF);
        UT_CHECK(white888.R == 255 && white888.G == 255 && white888.B == 255);
        neorgb_t black888 = color_to_neo((uint32_t)0x000000);
        UT_CHECK(black888.R == 0 && black888.G == 0 && black888.B == 0);
        // pure red at full scale, checked in each direction
        UT_CHECK(color_to_565((uint32_t)0xFF0000) == 0xF800);
        UT_CHECK(color_to_332((uint32_t)0xFF0000) == 0xE0);
        UT_CHECK(color_to_888((uint16_t)0xF800) == 0xF80000);  // 565->888 is lossy (no bit replication), so this is the correct (not "pure") expected value
        UT_CHECK(color_to_888((uint8_t)0xE0) == 0xE00000);     // same lossiness for 332->888
        neorgb_t neowhite = neorgb_t(255, 255, 255);
        UT_CHECK(color_to_888(neowhite) == 0xFFFFFF);
        UT_CHECK(color_to_565(neowhite) == 0xFFFF);
        UT_CHECK(color_to_332(neowhite) == 0xFF);
    }

    void test_massairflow() {  // objects.h - passing all 3 args explicitly bypasses live sensor/temp reads, making this deterministic
        UT_CHECK(massairflow(1.0f, 0.0f, 68.0f) == 0.0f);  // zero air velocity must give exactly zero flow, regardless of pressure
        float low = massairflow(1.0f, 5.0f, 68.0f);
        float high = massairflow(1.0f, 10.0f, 68.0f);
        UT_CHECK(!std::isnan(low) && !std::isnan(high));
        UT_CHECK(high > low);  // more air velocity must mean more mass airflow, holding pressure/temp fixed
        float lowP = massairflow(0.5f, 10.0f, 68.0f);
        float highP = massairflow(1.0f, 10.0f, 68.0f);
        UT_CHECK(highP > lowP);  // more pressure must mean more mass airflow, holding velocity/temp fixed
        // pressure enters linearly, so doubling it should roughly double the result, holding velocity/temp fixed
        UT_CHECK(std::fabs((highP / lowP) - 2.0f) < 0.01f);
    }

    void test_enum_card_arrays() {  // globals.h, motors.h, i2cbus.h, sensors.h, display.h, diag.h - lookup arrays indexed by an enum's Num* sentinel
        UT_CARD_COMPLETE(modecard, NumRunModes);
        UT_CARD_COMPLETE(requestcard, NumReqs);
        UT_CARD_COMPLETE(motorctrlcard, NumMotorCtrls);
        UT_CARD_COMPLETE(motoractioncard, NumMotorActions);
        UT_CARD_COMPLETE(cruiseschemecard, NumCruiseSchemes);
        UT_CARD_COMPLETE(brakefeedbackcard, NumBrakeFB);
        UT_CARD_COMPLETE(openloopcard, NumOpenLoops);
        UT_CARD_COMPLETE(uicontextcard, NumContextsUI);
        UT_CARD_COMPLETE(i2ccard, NumI2CSlaves);
        UT_CARD_COMPLETE(transdircard, static_cast<int>(TransDir::NumTransDir));
        UT_CARD_COMPLETE(ch4_menu_buttons, NumRunModes);
        UT_CARD_COMPLETE(diag.err_type_card, NumErrTypes);
        // Note: codestatuscard (BootMonitor/watchdog) and startreqcard (Starter) are private members of their classes, so they aren't reachable
        // here without a source change to expose them - not covered.

        // Spot checks for specific index->value pairs, catching a reordering/swap bug that a mere "is it non-empty" check can't see
        UT_CHECK(i2ccard[I2CTouch] == "touch");
        UT_CHECK(i2ccard[I2CLightbox] == "litbox");
        UT_CHECK(i2ccard[I2CAirVelo] == "airvel");
        UT_CHECK(i2ccard[I2CMAP] == "mapsns");
        UT_CHECK(modecard[Basic] == "Basic");
        UT_CHECK(modecard[LowPower] == "LowPwr");
        UT_CHECK(modecard[Fly] == "Fly");
        UT_CHECK(modecard[Cruise] == "Cruise");
        UT_CHECK(transdircard[static_cast<int>(TransDir::Rev)] == "rev");
        UT_CHECK(transdircard[static_cast<int>(TransDir::Fwd)] == "fwd");
    }

    void test_runmode_tables() {  // globals.h - per-runmode initial ctrlmode/action tables for gas/brake/steer
        // every (runmode, actuator) entry must be a valid enum value - catches a typo'd/garbage value, though not a silently-zero-filled missing
        // row (both CtrlDisable and ActionCruise happen to be enum value 0, so a truly missing row wouldn't fail these bounds checks - see the
        // recommendation about compiler-counted array sizes in the boot summary for a way to close that gap for real)
        bool ctrlmodes_valid = true, actions_valid = true;
        for (int rm = 0; rm < NumRunModes; rm++) {
            for (int actuator = 0; actuator < 3; actuator++) {
                int cm = run_motor_ctrlmode[rm][actuator];
                int ac = run_motor_action[rm][actuator];
                if (cm < 0 || cm >= NumMotorCtrls) ctrlmodes_valid = false;
                if (ac < 0 || ac >= NumMotorActions) actions_valid = false;
            }
        }
        UT_CHECK_SAFETY(ctrlmodes_valid);
        UT_CHECK_SAFETY(actions_valid);
    }

    void test_sensorcard() {  // display.h - sensorcard[] labels the sens enum for the PgSim potmap display. Confirmed mismatch found live: when the
        // `sens` enum lost its trailing ignition/syspower members (see the enum's own trailing comment), sensorcard wasn't fully updated to match -
        // mulebatt/engtemp/basicsw/starter's labels got scrambled relative to the enum starting at index 8. Tagged safety: a wrong label here could
        // lead an operator to believe they're looking at/simulating a different sensor than they actually are.
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::none)] == "none");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::joy)] == "joy");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::pressure)] == "bkpres");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::brkpos)] == "brkpos");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::speedo)] == "speedo");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::tach)] == "tach");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::airvelo)] == "airvel");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::mapsens)] == "mapsns");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::mulebatt)] == "batery");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::engtemp)] == "engtmp");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::basicsw)] == "basic");
        UT_CHECK_SAFETY(sensorcard[static_cast<int>(sens::starter)] == "startr");
    }

    void test_QPID() {  // motors.h - the shared PID controller class used by gas/brake/steer
        // NOTE: QPID::init() never explicitly sets lastin/lasterr - it relies on the class only ever being instantiated inside a static global
        // object (true everywhere in this codebase today), where C++ zero-initializes those members before the constructor runs. A QPID built on
        // the stack or heap would read uninitialized memory on its first compute(). Using `static` locals here to match real usage and avoid
        // that undefined behavior in the test itself - but this is a latent fragility worth hardening in QPID::init() directly at some point.
        static float in = 0.0f, outmin = -100.0f, outmax = 100.0f;
        static QPID pid(&in, &outmin, &outmax, 1.0f, 0.0f, 0.0f, QPID::pmod::onerr, QPID::dmod::onmeas, QPID::awmod::cond, QPID::cdir::direct, 100000, QPID::ctrl::manual);
        pid.set_target(10.0f);
        in = 0.0f;
        float out = pid.compute();
        UT_CHECK(out > 0.0f);  // measurement below target, direct-acting -> output should push upward

        static float in2 = 0.0f, outmin2 = -100.0f, outmax2 = 100.0f;
        static QPID pidrev(&in2, &outmin2, &outmax2, 1.0f, 0.0f, 0.0f, QPID::pmod::onerr, QPID::dmod::onmeas, QPID::awmod::cond, QPID::cdir::reverse, 100000, QPID::ctrl::manual);
        pidrev.set_target(10.0f);
        float outrev = pidrev.compute();
        UT_CHECK_SAFETY(outrev < 0.0f);  // identical setup but reverse-acting -> output must push the opposite direction

        // output clamping - safety-critical: must never exceed configured limits no matter how extreme the gain/error is
        static float in3 = 0.0f, outmin3 = -10.0f, outmax3 = 10.0f;
        static QPID pidclamp(&in3, &outmin3, &outmax3, 1000.0f, 0.0f, 0.0f, QPID::pmod::onerr, QPID::dmod::onmeas, QPID::awmod::cond, QPID::cdir::direct, 100000, QPID::ctrl::manual);
        pidclamp.set_target(1000.0f);  // huge target with a huge gain would massively exceed limits if clamping were broken
        float outclamp = pidclamp.compute();
        UT_CHECK_SAFETY(outclamp <= 10.0f && outclamp >= -10.0f);
    }

  public:
    void run_all() {
        ezread.squintf(ezread.highlightcolor, "Running unit tests..\n");
        test_utility_functions();
        test_Timer();
        test_Param();
        test_PulseSensor_math();
        test_color_conversions();
        test_massairflow();
        test_enum_card_arrays();
        test_runmode_tables();
        test_sensorcard();
        test_QPID();
        if (fails == 0) ezread.squintf("  all %d checks passed.\n", checks);
        else ezread.squintf(ezread.sadcolor, "  %d of %d checks failed.\n", fails, checks);
    }
};
static UnitTests unittests;