// globals.h - not dependent on anything, so include this first
#pragma once
#include "Arduino.h"
// pin assignments  ESP32-S3-DevkitC series   (note: "!" are free or free-ish pins, "*" are pins we can reclaim if needed, with some rework/rewrite necessary)
#define     boot_sw_pin  0 // button0/strap1  * // input, the esp "boot" button. if more pins are needed, move encoder_sw_pin to this pin, no other signal of ours can work on this pin due to high-at-boot requirement
#define      tft_dc_pin  1 // adc1.0            // output, assert when sending data to display chip to indicate commands vs. screen data
#define        free_pin  2 // adc1.1          ! // full-capability pin just waiting for you to go balls-out! (was prev reserved for driving a choke servo)
#define   sdcard_cs_pin  3 // adc1.2/strapX   ! // output, chip select for SD card controller on SPI bus. sdcard is not implemented, we may not even need it, so if pin is needed, disconnect from the sd slave chip and it's good to go
#define    mulebatt_pin  4 // adc1.3            // analog input, mule battery voltage sense, full scale is 16V
#define         pot_pin  5 // adc1.4            // analog in from 20k pot
#define   brake_pos_pin  6 // adc1.5            // analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#define    pressure_pin  7 // adc1.6            // analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V
#define     i2c_sda_pin  8 // sda0/adc1.7       // i2c bus for airvelo/map sensors, lighting board, cap touchscreen
#define     i2c_scl_pin  9 // qhd0/scl0/adc1.8  // i2c bus for airvelo/map sensors, lighting board, cap touchscreen
#define      tft_cs_pin 10 // cs0/adc1.9      * // output, active low, chip select allows ILI9341 display chip use of the spi bus. can reclaim pin if tft is the only spi device (no sd card or resist. touch), if so the tft CS pin must be grounded
#define    spi_mosi_pin 11 // mosi0/adc2.0      // used as spi interface data for tft screen, sd card and resistive touch panel
#define    spi_sclk_pin 12 // sclk0/adc2.1      // used as spi interface clock for tft screen, sd card and resistive touch panel
#define    spi_miso_pin 13 // miso0/adc2.2    * // used as spi interface data from sd card and resistive touch panel. can reclaim this pin if only the tft is on the spi bus
#define hotrc_ch2_v_pin 14 // qwp0/pwm0/adc2.3  // hotrc ch2 bidirectional trigger input
#define hotrc_ch1_h_pin 15 // pwm1/adc2.4     * // hotrc ch1 thumb joystick input. can reclaim this pin and the steer_pwm pin by connecting the hotrc horz chan straight to the steering jaguar 
#define     gas_pwm_pin 16 // pwm1/adc2.5       // output, pwm signal duty cycle controls throttle target. on Due this is the pin labeled DAC1 (where A13 is on Mega)
#define   brake_pwm_pin 17 // pwm0/adc2.6/tx1   // output, pwm signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% = stop)
#define   steer_pwm_pin 18 // pwm0/adc2.7/rx1 * // output, pwm signal positive pulse width sets steering motor speed from full left to full speed right, (50% = stop). can reclaim this and hotrc_horz pins by connecting the hotrc horz chan straight to the steering jaguar 
#define steer_enc_a_pin 19 // usb-d-/adc2.8   ! // reserved for usb or a steering quadrature encoder. since we don't have this encoder, if not using for jtag this pin may be used for whatever
#define steer_enc_b_pin 20 // usb-d+/adc2.9   ! // reserved for usb or a steering quadrature encoder. since we don't have this encoder, if not using for jtag this pin may be used for whatever
#define     onewire_pin 21 // pwm0              // onewire bus for temperature sensor data. note: tested this does not work on higher-numbered pins (~35+)
#define      speedo_pin 35 // spiram/octspi     // int input, active high, asserted when magnet south is in range of sensor. 1 pulse per driven pulley rotation. (Open collector sensors need pullup)
#define     starter_pin 36 // sram/ospi/glitch  // input/Output (both active high), output when starter is being driven.  ability to sense externally-driven start signal is disabled in hardware
#define        tach_pin 37 // spiram/octspi     // int Input, active high, asserted when magnet south is in range of sensor. 1 pulse per engine rotation. (no pullup) - note: maybe better on p36 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on
#define  encoder_sw_pin 38 // spiram/octspi   * // input, rotary encoder push switch, for the UI. active low (needs pullup). signal can be moved to pin 0 to free up this pin. Pin 38 is the neopixel pin on v1.1 boards
#define   fan_tp_cs_pin 39 // jtck/glitch     ! // output, drives external 12V switch to cooling fan, -or- touchpanel chip select for resistive types (if detected on devboard).  note possible known glitch: 80ns low pulse when certain rtc devices power on (see errata 3.11)
#define   hotrc_ch4_pin 40 // jtdo              // syspower, starter, and cruise mode toggle control. hotrc ch4 pwm toggle signal
#define   hotrc_ch3_pin 41 // jtdi              // ignition control, hotrc Ch3 PWM toggle signal
#define   encoder_a_pin 42 // jtms              // int input, the A (aka CLK) pin of the encoder. both A and B complete a negative pulse in between detents. if A pulse goes low first, turn is CCW. (needs pullup)
#define    tx_basic_pin 43 // "TX"/tx0          // serial monitor data out. Also used to read basic mode switch using pullup/pulldown by temporarily interrupting the console to read
#define          rx_pin 44 // "RX"/rx0          // serial monitor data in. maybe could repurpose during runtime since we only need outgoing console data?
#define    ignition_pin 45 // strap0            // output to an nfet/pfet pair to control the car ignition
#define    syspower_pin 46 // strap0            // output to an nfet/pfet pair to power all the tranducers.
#define   encoder_b_pin 47 // NA                // int input, the B (aka DT) pin of the encoder. both A and B complete a negative pulse in between detents. if B pulse goes low first, turn is CW. (needs pullup)
#define    neopixel_pin 48 // neopix            // data line to onboard neopixel WS281x (on all v1 devkit boards - pin 38 is used on v1.1 boards). also used for onboard and external neopoxels - ! pin is also defined in neopixel.h
// external components used (pullup/pulldown resistors, capacitors, etc.): (note: "BB" = on dev breadboards only, "PCB" = on vehicle PCB only)
// 1. onewire_pin, tach_pin, speedo_pin: Add 4.7k-ohm to 3.3V, needed for open collector sensor output to define logic-high voltage level
// 2. tach_pin: also requires a 8x frequency divider, to slow down the pulse stream
// 3. tx_basic_pin: (BB) connect a 22k-ohm from pin to gnd to avoid always being in basic mode
// 4. resistor dividers are needed for these inputs: starter_pin (16V->3.3V), mulebatt_pin (16V->3.3V), and (possibly) pressure_pin (5V->3.3V)
// 5. ignition_pin, syspower_pin, starter_pin: require pulldowns to gnd, this is provided by nfet gate pulldown
// 6. gas_pwm_pin: should have a series ~680-ohm R going to the servo
// 7. encoder_a_pin, encoder_b_pin, button_pin: should have 10nF to gnd, tho it should work w/o it. pullups to 3.3V (4.7k-ohm is good) are also necessary, but the encoder we're using includes these
// 8. neopixel_pin: (PCB) Add 330 ohm in series (between pin and the DataIn pin of the 1st pixel). (BB) same, but this one is likely optional, e.g. mine works w/o it.  for signal integrity over long wires
// 9. mulebatt_pin, pressure_pin, brake_pos_pin, pot_wipe_pin, tach_pin, speedo_pin: (ADC inputs) should have 100nF cap to gnd, to improve signal stability, tho it works w/o it

// note onewire works on pins 19-21 but not on pins 39-42
// if more pins become needed, encoder_sw may be moved to pin 0, freeing up pin 38 (pin 0 requires a pullup, which encoder_sw has)
// if more pins become needed, we can abandon the sd card and permanently enable the tft, getting us back 2 pins for those chip selects
// ESP32-S3 TRM: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#dma
// ESP32-S3 datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
// ESP32-S3 has 5 DMA channels in each direction. We would use them for SPI data out to TFT, neopixel data out, and possibly out to the 3 motor outputs and in from the 4 hotrc channels.
// dma works with: RMT, I2S0, I2S1, SPI2, SPI3, ADC, internal RAM, external PSRAM, and a few others (see the TRM)
// official pin capabilities: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html?highlight=devkitc#user-guide-s3-devkitc-1-v1-1-header-blocks
// external flash uses pins 27-32. ADC ch2 will not work if wifi is enabled
// bootstrap pins: Pin 0 must be pulled high, and pins 45 and 46 pulled low during bootup
// glitch: pins 36 and 39 will be erroneously pulled low for ~80ns when "certain RTC peripherals power up" (ESP32 errata 3.11). can run adc_power_acquire() to work around glitch but draw ~1mA more power. avoid interrupts on these pins
// spi bus page including DMA information: https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/spi_master.html
#define tft_rst_pin -1     // tft reset allows us to reboot the screen hardware when it crashes. Otherwise connect screen reset line to esp reset pin
#define tft_ledk_pin -1    // output, optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define touch_irq_pin -1   // input, optional touch occurence interrupt signal (for resistive touchscreen, prevents spi bus delays) - set to 255 if not used
// bm2023 box compatibility: steer_enc_a_pin 1, steer_enc_b_pin 2, tft_dc_pin 3, onewire_pin 19, hotrc_ch3_pin 20, hotrc_ch4_pin 21, tach_pin 36, ignition_pin 37, syspower_pin 38, encoder_b_pin 40, encoder_a_pin 41, encoder_sw_pin 42, starter_pin 45, sdcard_cs_pin 46, tp_cs_fuel_pin 47

#define adcbits 12
#define adcrange_adc 4095     // = 2^adcbits-1
#define adcmidscale_adc 2047  // = 2^(adcbits-1)-1

// global enums. these can be super convenient but are also prone to errors, some more than others (see comments)
//
// this group has enums which are relatively straightforward, i.e. each used in only one context 
enum runmode { Basic=0, LowPower=1, Standby=2, Stall=3, Hold=4, Fly=5, Cruise=6, Cal=7, NumRunModes=8 };
enum req { ReqOff=0, ReqOn=1, ReqTog=2, ReqNA=3, NumReqs=4 };  // requesting handler actions of digital values with handler functions
enum starter_requestors { StartUnknown=0, StartClass=1, StartHotrc=2, StartTouch=3, StartRunmode=4, NumStartReq=5 };  // for identification of starter motor request source
enum cruise_modes { OnePull=0, HoldTime=1, NumCruiseSchemes=2 };
enum sw_presses { SwNone=0, SwShort=1, SwLong=2 };
enum motor_modes { NA=0, Halt=1, Idle=2, Release=3, OpenLoop=4, PropLoop=5, ActivePID=6, AutoStop=7, AutoHold=8, ParkMotor=9, CruiseMode=10, Calibrate=11, Starting=12, AutoPID=13, NumMotorModes=14 };
enum brakefeedbacks { PositionFB=0, PressureFB=1, HybridFB=2, NoneFB=3, NumBrakeFB=4 };
enum openloopmodes { MedianPoint=0, AutoRelease=1, AutoRelHoldable=2, NumOpenLoopModes=3 };
enum datapages { PgRun=0, PgHrc=1, PgSens=2, PgPuls=3, PgPWMs=4, PgIdle=5, PgMotr=6, PgBPID=7, PgGPID=8, PgCPID=9, PgTemp=10, PgSim=11, PgDiag=12, PgUI=13, NumDataPages=14 };
enum diag_val { DiagVal=0, DiagMin=1, DiagMax=2, DiagMargin=3, NumDiagVals=4 };
enum joydirs { HrcRt=-2, HrcDn=-1, HrcCent=0, HrcUp=1, HrcLt=2, HrcPlus=3, HrcMinus=4 };
enum panel_apps { EZReadUI=0, MuleChassisUI=1, ScreensaverUI=2, NumContextsUI=3 };  // uses for the multi purpose panel
enum codestatus { StConfused=0, StAsleep=1, StBooting=2, StParked=3, StStopped=4, StDriving=5, StInBasic=6, StPanicking=7, NumCodeStatuses=8 };
enum pcba_glow_modes { GlowOff=0, GlowSimple=1, GlowHeart=2, GlowXFade=3, GlowSine=4, GlowNumModes=5 };
enum err_type { ErrLost=0, ErrRange=1, ErrWarn=2, NumErrTypes=3 };
enum directional { DirNone=0, DirDown=1, DirUp=2, DirLeft=3, DirRight=4, NumDirs=5 };
enum cycle_dirs { DirRev=-1, DirFwd=1 };
enum cruisepids { GasOpen=0, GasPID=1 };
enum rgb { Red=0, Grn=1, Blu=2, NumRGB=3 };
//
// this group is used in multiple places in different ways, and is thus high-risk.
// for example, arrays exist with indexes drawn from different combinations of multiple ones of these enums
// search for all uses and understand them before changing anything
enum hotrc_axis { Horz=0, Vert=1, Ch3=2, Ch4=3 };
enum hotrc_val { OpMin=0, Cent=1, OpMax=2, Raw=3, Filt=4, DBBot=5, DBTop=6 };
enum motor_val { Parked=1, Out=3, Govern=4 , AbsMin=5, AbsMax=6, Margin=7, NumMotorVals=8 }; // Idle=8, NumMotorVals=9 };
enum stop_val { Stop=1 };
enum temp_val { Alarm=3 };
enum size_enums { NumAxes=2, NumChans=4, NumValues=8 };
//
// the Off and On values in these two enums are used in multiple places I think
enum tunerstuff { Off=0, Select=1, Edit=2 };
enum boolean_states { On=1 };
//
// these telemetry enums overlap in their use and so be cautious when making changes to check everything
enum brakeextra { NumBrakeSens=2 };
enum telemetry_idiots {                              // list of transducers which have onscreen idiotlights showing status
    _Hybrid=-3, _None=-2, _NA=-1,                    // these meta values indicate no transducer, useful for some contexts  
    _Throttle=0, _BrakeMotor=1, _SteerMotor=2,       // these transducers are mission-critical actuators, driven by us.  note: these enums are also used to index global runmode:motormode array 
    _HotRC=3, _Speedo=4, _Tach=5, _BrakePres=6, _BrakePosn=7,  // these transducers are mission-critical sensors, we read from
    _Temps=8, _MuleBatt=9, _Other=10, _GPIO=11,          // these are actually groups of multiple sensors (see below)
    NumTelemetryIdiots=12,                           // size of the list of values with idiot lights
};
enum telemetry_full {                                                                                 // complete list expanding sensor groups
    _HotRCHorz=12, _HotRCVert=13, _HotRCCh3=14, _HotRCCh4=15,                                         // _HotRC sensor group
    _AirVelo=16, _MAP=17, _Pot=18,                                                      // _Other sensor group
    _TempEng=19, _TempWhFL=20, _TempWhFR=21, _TempWhRL=22, _TempWhRR=23, _TempBrake=24, _TempAmb=25,  // _Temps sensor group
    _Ignition=26, _Starter=27, _BasicSw=28,                                                           // _GPIO signal group (with simple boolean values)
    _TempWheel=29,                                                                                    // flag for any wheel temp out of range
    NumTelemetryFull=30,                                                                              // size of both telemetry lists combined
};

// global configuration settings - these settings are initialized for devboard use. if actual vehicle is detected
//                                  at boot, then are overwritten as appropriate in set_board_defaults() function
bool autostop_disabled = false;      // temporary measure to keep brake behaving until we get it debugged. Eventually should be false
bool allow_rolling_start = true;     // are we lenient that it's ok to go to fly mode if the car is already moving? may be a smart prerequisite, may be us putting obstacles in our way
bool flip_the_screen = false;        // did you mount your screen upside-down?
bool cruise_speed_lowerable = true;  // allows use of trigger to adjust cruise speed target without leaving cruise mode.  Otherwise cruise button is a "lock" button, and trigger activity cancels lock
bool display_enabled = true;         // should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool use_i2c_baton = false;          // use soren's custom homemade semaphores to prevent i2c bus collisions?
bool limit_framerate = true;         // set to true to enforce a cap on screen frame draws during operational modes, otherwise draw as fast as we can. fullscreen screensaver ignores this, always balls-out
bool brake_before_starting = true;   // if true, the starter motor attempts to apply the brake pedal before turning on the starter motor
bool check_brake_before_starting = true;  // if true, the starter motor won't turn on until or unless it senses the brake pressure is enough. otherwise then after a timeout it will start anyway
bool push_gas_when_starting = false;  // will help start esp when engine is cold, in lieu of choke access, however if it lurches then disable. automatically disables if brake check is disabled
bool two_click_starter = false;      // to start the starter requires two requests within a timeframe
bool watchdog_enabled = false;       // enable the esp's built-in watchdog circuit, it will reset us if it doesn't get pet often enough (to prevent infinite hangs). disabled cuz it seems to mess with the hotrc (?)
bool print_task_stack_usage = false; // enable to have remaining heap size and free task memory printed to console every so often. for tuning memory allocation
bool autosaver_display_fps = true;   // do you want to see the fps performance of the fullscreen saver in the corner?
bool pot_tuner_acceleration = false; // when editing values, can we use the pot to control acceleration of value changes? (assuming we aren't pot mapping some sensor at the time)
bool dont_take_temperatures = false; // disables temp sensors. in case debugging dallas sensors or causing problems
bool console_enabled = true;         // completely disables the console serial output. it can be safer disabled b/c serial printing itself can easily cause new problems, and libraries might do it whenever
bool keep_system_powered = false;    // ensures syspower is always high.
bool looptime_print = false;         // makes code write out timestamps throughout loop to serial port. for analyzing what parts of the code take the most time
bool touch_reticles = true;          // draws tiny little plus reticles to aim at for doing touchscreen calibration
bool button_test_heartbeat_color = false;  // makes boot button short press change heartbeat color. useful for testing code on bare esp
bool wifi_client_mode = false;       // should wifi be in client or access point mode?
bool screensaver_enabled = true;     // does fullscreen screensaver start automatically (after a delay) when in standby mode?
bool print_framebuffers = false;     // dumps out ascii representations of screen buffer contents to console. for debugging frame buffers. *hella* slow
bool use_tft_colors_for_neo = true;  // should neopixel colors be based on onscreen icon colors? (otherwise they'll split the full hue spectrum amongst themselves)
bool print_error_changes = false;    // !!! temporarily suppressed !!! should diag print status changes and new error events to console?
bool pot_controls_animation_timeout = false; // when showing fullscreen animations, should the pot value control the next animation timeout?
bool cruise_brake = true;            // does brake work in cruise mode
bool use_idle_boost = false;         // should we try to manage a dynamic idle speed?
bool overtemp_shutoff_brake = true;  // should a brake temp beyond opmax cause a brake motor and engine shutoff?
bool overtemp_shutoff_engine = true; // should an engine temp beyond opmax cause engine shutoff and possible panic?
bool overtemp_shutoff_wheel = true;  // should a wheel temp beyond opmax cause engine shutoff?
bool throttle_linearize_trigger = true;  // should trigger values be linearized for gas determination?
bool throttle_linearize_cruise = false;  // should trigger values be linearized when in cruise mode?
// bool stall_mode_timeout = true;    // should stall mode time out after a while, to mitigate potential safety issues w/ ghost starter bug
bool throttle_pid_default = false;    // default throttle control mode. values: ActivePID (use the rpm-sensing pid), OpenLoop, or Linearized
bool cruise_pid_default = false;       // default throttle control mode. values: ActivePID (use the rpm-sensing pid), OpenLoop, or Linearized
// bool force_hotrc_button_filter = false; // always force button filtration for all actions. otherwise unfiltered presses are allowed for safety events (ie ignition or starter kill), in case of radio interference
bool ezread_suppress_spam = true;       // activates ezread feature to suppress data coming into the console too fast (to prevent overrun crashes)
bool panic_on_boot_after_crash = true;  // causes bootmanager to do a panic on boot if car was in a drive state when reset 
bool require_radiolost_test = true;     // should we refuse ignition to be on if radiolost feature hasn't been tested
bool holdmode_ch4_drivetoggle = false;  // should a ch4 press in hold mode toggle the preferred drivemode, assuming starter is off?

// global tunable variables
int operational_framerate_limit_fps = 100;  // max display frame rate to enforce while driving whenever limit_framerate == true
float wheeldifferr = 35.0f;             // how much hotter the hottest wheel is allowed to exceed the coldest wheel before idiot light
constexpr float float_conversion_zero = 0.001f;
constexpr int unlikely_int = -92935762; // random ass unlikely value for detecting unintended arguments
int sprite_color_depth = 8;
int looptime_linefeed_threshold = 0;    // when looptime_print == 1, will linefeed after printing loops taking > this value. set to 0 linefeeds all prints
float flycruise_vert_margin_pc = 3.0f;  // margin of error (in percent) for determining hard brake value for dropping out of cruise mode
float cruise_holdtime_attenuator_pc = 10.0f; // adjustment rate multiplier for cruise HoldTime mode
float cruise_onepull_attenuator_pc = 14.0f;  // adjustment rate multiplier for cruise OnePull mode
float maf_min_gps = 0.0;                // in grams per second
float maf_max_gps = 50.0f;              // i just made this number up as i have no idea what's normal for MAF
float tuning_rate_pcps = 7.5f;          // values being edited by touch buttons change value at this percent of their overall range per second
float neobright = 20.0f;                // default for us dim/brighten the neopixels in percent
float neosat = 90.0f;                   // default saturation of neopixels in percent
int i2c_frequency = 400000;             // in kHz. standard freqs are: 100k, 400k, 1M, 3.4M, 5M
float governor = 95.0f;                 // software governor will only allow this percent of full-open throttle (percent 0-100)

int run_motor_mode[NumRunModes][3] = {   // Array of which motor mode the gas/brake/steer get set to upon entering each runmode
    { Halt, Halt, OpenLoop },  // Basic mode    (_Throttle/_BrakeMotor/_SteerMotor)
    { ParkMotor, ParkMotor, Halt },      // LowPower mode (_Throttle/_BrakeMotor/_SteerMotor)
    { ParkMotor, Halt, OpenLoop },       // Standby mode  (_Throttle/_BrakeMotor/_SteerMotor)
    { OpenLoop, ActivePID, OpenLoop },   // Stall mode    (_Throttle/_BrakeMotor/_SteerMotor)
    { AutoPID, AutoHold, OpenLoop },     // Hold mode     (_Throttle/_BrakeMotor/_SteerMotor)
    { AutoPID, ActivePID, OpenLoop },    // Fly mode      (_Throttle/_BrakeMotor/_SteerMotor)
    { CruiseMode, ActivePID, OpenLoop }, // Cruise mode   (_Throttle/_BrakeMotor/_SteerMotor)
    { Idle, Halt, Halt },                // Cal mode      (_Throttle/_BrakeMotor/_SteerMotor)
};

#ifndef MonitorBaudrate
#define MonitorBaudrate 115200
#endif

// non-tunable values. probably these belong with their related code, but are global to allow accessibility from everywhere
int serial_monitor_baudrate = (int)MonitorBaudrate;
volatile int refresh_limit_us = 1000000 / operational_framerate_limit_fps; // 60 Hz -> 16666 us, 90 Hz -> 11111 us, 120 Hz -> 8333 us
std::string modecard[NumRunModes] = { "Basic", "LowPwr", "Stndby", "Stall", "Hold", "Fly", "Cruise", "Cal" };
std::string requestcard[NumReqs] = { "Off", "On", "Tog", "NA" };

float permanan = NAN;
// bool first_drive = true;                // goes false upon first entry into a drive mode after a power cycle. thereafter the car will autohold the brake in standby/stall modes (to help prevent lurching forward on remote start)
bool wheeltemperr;
float* nanptr = &permanan;
uint32_t codestatus = StBooting;
int runmode = Standby;
bool bootbutton_val;                    // added for debug, so other functions defined before bootbutton can read bootbutton value. likely can be removed later if i forget to do it
bool flashdemo = false;
bool running_on_devboard = false;       // will overwrite with value read thru pull resistor on tx pin at boot
bool fun_flag = false;                  // since now using temp sensor address to detect vehicle, our tx resistor can be used for who knows what else!
bool shutting_down = true;              // minor state variable for standby mode - standby mode has not completed its work and can't yet stop activity
bool parking = false;                   // indicates in process of parking the brake & gas motors so the pedals can be used manually without interference
bool releasing = false;                 // indicates in process of releasing the brake to the zero brake point
bool cruise_adjusting = false;
bool cal_brakemode = false;             // allows direct control of brake motor using controller vert
bool cal_brakemode_request = false;     // allows direct control of brake motor using controller vert
bool cal_gasmode = false;               // allows direct control of gas servo using pot. First requires pot to be in valid position before mode is entered
bool cal_gasmode_request = false;
bool car_hasnt_moved = false;           // minor state variable for fly mode - Whether car has moved at all since entering fly mode
bool powering_up = false;               // minor state variable for lowpower mode
bool powering_down = false;             // minor state variable for lowpower mode
bool calmode_request = false;
bool flycruise_toggle_request = false;
bool in_basicmode = false;              // bool basicmode_request = false;
int tunctrl = Off, tunctrl_last = Off;
int datapage = PgRun;                  // which of the dataset pages is currently displayed and available to edit?
int autosaver_request = ReqNA;
volatile bool auto_saver_enabled = false;
volatile int sel = 0;                   // in the real time tuning UI, which of the editable values is selected. -1 for none 
volatile int sel_last = 0;
volatile int sel_last_last = 0;
// bool syspower = HIGH, not_syspower = !syspower; // set by handler only. Reflects current state of the signal
int sleep_request = ReqNA;
float maf_gps = 0;                              // manifold mass airflow in grams per second
uint16_t heartbeat_override_color = 0x0000;
bool nowtouch = false, ts_tapped = false, ts_doubletapped = false, ts_longpressed = false, ts_swiped = false;  // touchscreen globals
int ts_swipedir = DirNone;  // touchscreen
// bool nowtouch2 = false;
bool captouch = true;
float loop_avg_us;
bool sensidiots[NumTelemetryIdiots];  // array holds an error flag for each critical sensor or sensor group 
int ui_app = EZReadUI, ui_app_default = EZReadUI;
bool panicstop = false;
bool bootup_complete = false;

constexpr float float_zero = 0.000069f;  // minimum required float precision. use for comparisons & zero checking
inline bool iszero(float num, float margin=NAN) noexcept {  // safe check for if a float is effectively zero (avoid hyperprecision errors)
    if (std::isnan(margin)) margin = float_zero;  // assume global default margin value if nothing specific is given
    if (!std::isnan(num)) return (std::abs(num) <= margin);  // if input is valid return result
    Serial.printf("err: iszero(NAN) called\n");  // print err rather than crash. would prefer ezread if it were defined
    return true;  // default to true if given nan (even tho likely inaccurate), b/c likely to best inform calls intending to prevent divzero
}
inline bool isinfinite(float num, float margin=NAN) noexcept {  // safe check for if a float is effectively infinite (handle value explosions)
    if (std::isnan(margin)) margin = float_zero;  // assume global default margin value if nothing specific is given
    if (iszero(num, margin)) return false;        // prevent divzero errors
    if (!std::isnan(num)) return (1.0f / std::abs(num) <= margin);  // if input is valid return result
    Serial.printf("err: iszero(NAN) called\n");  // print err rather than crash. would prefer ezread if it were defined
    return true;  // default to true if given nan, b/c likely accurate, also likely to best inform calls intending to prevent divzero
}
inline void cleanzero(float* num, float margin=NAN) noexcept {  // zeroes float values which are obnoxiously close to zero
    if (num && iszero(*num, margin)) *num = 0.0f;  // notes: 1) lets iszero() deal w/ nan margin, 2) checks for null pointer
}
// fast macros
#define arraysize(x) ((int)(sizeof(x) / sizeof((x)[0])))  // a macro function to determine the size of arrays 
#undef constrain
inline float constrain(float amt, float low, float high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline int constrain(int amt, int low, int high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline long constrain(long amt, long low, long high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline unsigned int constrain(unsigned int amt, unsigned int low, unsigned int high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
#undef map
inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    if (!iszero(in_max - in_min)) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    Serial.printf("err: map(%3.3f, %3.3f, %3.3f, %3.3f, %3.3f) called\n", in_min, in_max, out_min, out_max); // would prefer ezread if it were defined
    return out_max;  // instead of dividing by zero, return the highest valid result
}
inline int map(int x, int in_min, int in_max, int out_min, int out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    Serial.printf("err: map(%d, %d, %d, %d, %d) called\n", x, in_min, in_max, out_min, out_max); // would prefer ezread if it were defined
    return out_max;  // instead of dividing by zero, return the highest valid result
}

// pin operations
void set_pin(int pin, int mode) { if (pin >= 0 && pin != 255) pinMode (pin, mode); }
void write_pin(int pin, int val) {  if (pin >= 0 && pin != 255) digitalWrite (pin, val); }
void set_pin(int pin, int mode, int val) { set_pin(pin, mode); write_pin(pin, val); }
int read_pin(int pin) { return (pin >= 0 && pin != 255) ? digitalRead (pin) : -1; }

float convert_units(float from_units, float convert_factor, bool invert, float in_offset = 0.0, float out_offset = 0.0) {
    if (!invert) return out_offset + convert_factor * (from_units - in_offset);
    if (from_units - in_offset) return out_offset + convert_factor / (from_units - in_offset);
    Serial.printf("err: convert_units(%3.3f, %3.3f, %d, %3.3f, %3.3f) called\n", from_units, convert_factor, invert, in_offset, out_offset); // would prefer ezread if it were defined
    return -1;
}
// Exponential Moving Average filter : smooth out noise on inputs. 0 < alpha < 1 where lower = smoother and higher = more responsive
// pass in a fresh raw value, current filtered value, and alpha factor, new filtered value is returned
float ema_filt(float _raw, float _filt, float _alpha) {
    if (std::isnan(_raw) || std::isnan(_filt)) {
        Serial.printf("err: ema_filt received NAN value\n");
        if (std::isnan(_raw)) return _filt;  // try to save the runtime value in case of spurious glitch
        return NAN;
    }
    _alpha = constrain(_alpha, 0.0, 1.0);
    return (_alpha * _raw) + ((1.0 - _alpha) * _filt);
}
// template<typename Raw_T, typename Filt_T>
// void ema_filt(Raw_T _raw, Filt_T* _filt, float _alpha) {
//     float _raw_f = static_cast<float>(_raw);
//     float _filt_f = static_cast<float>(*_filt);
//     *_filt = static_cast<Filt_T>(ema_filt(_raw_f, _filt_f, _alpha));
// }

// linearizer(): applies an exponential curve transformation to a signed percent value at the given ptr
// - 'ex' is the exponent (>= 1.0); higher values increase curvature (ex=1.0 leaves value unchanged)
// - math: out = 100*(|in|/100)^ex = 100*e^(ex*ln(|in|/100)) , use identity x^a=e^(a*ln(x)) avoids expensive pow()
// - can scale positive or negative vals (sign of input is preserved). input==0 will leave value unchanged
bool linearizer(float* in_pc_ptr, float ex) {  // returns true if transformation applied, or false if invalid args
    float in = *in_pc_ptr;   // store input value so we can mess around with it
    cleanzero(&in);          // if value is obnoxiously close to zero, set it to zero (avoid float precision bugs)
    if (iszero(in) || ex < 1.0f) return false; // avoid ln(0) crash or potential sub-1.0 exponent crashes
    *in_pc_ptr = ((in >= 0.0f) ? 100.0f : -100.0f) * expf(ex * logf(fabsf(in) / 100.0f));  // do nerdy math
    return true;             // indicate transformation successfully applied
}

// code to capture compile-time git related info coming from our script
#ifdef GIT_ENABLED
    #ifndef GIT_SHA
        #define GIT_SHA "unknown"
    #endif
    #ifndef GIT_BRANCH
        #define GIT_BRANCH "unknown"
    #endif
    #ifndef BUILD_TIME_UTC
        #define BUILD_TIME_UTC "unknown"
    #endif
    #ifndef GIT_DIRTY
        #define GIT_DIRTY 0    // numeric macro 0/1
    #endif
    #ifndef GIT_ERROR
        #define GIT_ERROR 0    // numeric macro 0/1
    #endif
    static const std::string kBuildSha       = std::string(GIT_SHA);
    static const std::string kBuildBranch    = std::string(GIT_BRANCH);
    static const std::string kBuildBuiltUtc  = std::string(BUILD_TIME_UTC);
    static constexpr bool    kBuildDirty     = (GIT_DIRTY != 0);
    static constexpr bool    kBuildGitError  = (GIT_ERROR != 0);
    // handy accessors
    // static inline const std::string& build_sha()       { return kBuildSha; }
    // static inline const std::string& build_branch()    { return kBuildBranch; }
    // static inline const std::string& build_built_utc() { return kBuildBuiltUtc; }
    static inline bool build_dirty()     { return kBuildDirty; }
    static inline bool build_git_error() { return kBuildGitError; }
#endif

// Timer objects are prolific in every corner of the code and thus this is global
class Timer {  // !!! beware, this 54-bit microsecond timer overflows after every 571 years
  protected:
    volatile int64_t start, tout;
  public:
    Timer() { reset(); }
    Timer(int arg_timeout) { set (arg_timeout); }
    void set(int arg_timeout) {                                               // sets the timeout to the given number (in us) and zeroes the timer
        tout = (int64_t)arg_timeout;
        start = esp_timer_get_time();
    }
    void reset() { start = esp_timer_get_time(); }                            // zeroes the timer
    int elapsed() { return esp_timer_get_time() - start; }                    // returns microseconds elapsed since last reset
    bool elapsed(int check) { return esp_timer_get_time() - start >= check; } // returns whether the given amount of us have elapsed since last reset
    int timeout() { return tout; }                                            // getter function returns the currently set timeout value in us
    bool expired() { return esp_timer_get_time() >= start + tout; }           // returns whether more than the previously-set timeout has elapsed since last reset
    bool expireset() {                                                        // like expired() but automatically resets if expired
        int64_t now = esp_timer_get_time();
        if (now < start + tout) return false;
        start = now;
        return true;
    }
};

// color macros and color conversion functions. these are global to allow accessibility from multiple places
//
template <typename T>  // feed me hue/sat/bright values and get back an rgb color formatted as rgb332 (8b), rgb565 (16b), or rgb888 (32b)
T hsv_to_rgb(uint16_t hue, uint8_t sat = 255, uint8_t val = 255) {
    uint8_t rgb[NumRGB] = { 0, 0, 0 };  // [Red,Grn,Blu];
    hue = (hue * 1530L + 32768) / 65536;
    if (hue < 510) { // Red to Grn-1
        if (hue < 255) { rgb[Red] = 255; rgb[Grn] = hue; }  //   Red to Yel-1, Grn = 0 to 254
        else { rgb[Red] = 510 - hue; rgb[Grn] = 255; }  //  Yel to Grn-1, Red = 255 to 1
    }
    else if (hue < 1020) { // Grn to Blu-1
        if (hue < 765) { rgb[Grn] = 255; rgb[Blu] = hue - 510; }  //  Grn to Cyn-1, Blu = 0 to 254
        else { rgb[Grn] = 1020 - hue; rgb[Blu] = 255; }  // Cyn to Blu-1, Grn = 255 to 1
    }
    else if (hue < 1530) {  // Blu to Red-1
        if (hue < 1275) { rgb[Red] = hue - 1020; rgb[Blu] = 255; }  // Blu to Mgt-1, Red = 0 to 254
        else { rgb[Red] = 255; rgb[Blu] = 1530 - hue; }  //   Mgt to Red-1, Blu = 255 to 1
    }
    else { rgb[Red] = 255; }  // Last 0.5 Red (quicker than % operator)
    uint32_t v1 = 1 + val;  // 1 to 256; allows >>8 instead of /255
    uint16_t s1 = 1 + sat;  // 1 to 256; same reason
    uint8_t s2 = 255 - sat; // 255 to 0
    uint16_t out[NumRGB];
    for (int led=Red; led<NumRGB; led++) out[led] = (((((rgb[led] * s1) >> 8) + s2) * v1) & 0xff00) >> 8;
    // if (fake_color332) if (std::is_same<T, uint16_t>::value) return (T)((out[Red] & 0xe0) << 8) | ((out[Grn] & 0xe0) << 5) | ((out[Blu] & 0xc0) >> 3);
    if (std::is_same<T, uint16_t>::value) return (T)((out[Red] & 0xf8) << 8) | ((out[Grn] & 0xfc) << 5) | (out[Blu] >> 3);
    else if (std::is_same<T, uint8_t>::value) return (T)((out[Red] & 0xe0) | ((out[Grn] & 0xe0) >> 3) | ((out[Blu] & 0xc0) >> 6));
    else if (std::is_same<T, uint32_t>::value) return (T)((out[Red] << 16) | (out[Grn] << 8) | out[Blu]);
}
uint8_t rando_color() {
    return ((uint8_t)random(0b111) << 5) | ((uint8_t)random(0b111) << 2) | (uint8_t)random(0b11); 
}
// named 8-bit colors (332 format)
const uint8_t BLK  = 0x00;  // greyscale: full black (RGB elements off)
const uint8_t DGRY = 0x49;  // pseudo-greyscale: very dark grey (blueish)
const uint8_t MGRY = 0x6d;  // pseudo-greyscale: medium grey (yellowish)
const uint8_t LGRY = 0xb6;  // greyscale: very light grey
const uint8_t WHT  = 0xff;  // greyscale: full white (RGB elements full on)
const uint8_t RED  = 0xe0;  // red (R element full on)
const uint8_t YEL  = 0xfc;  // yellow (RG elements full on)
const uint8_t GRN  = 0x1c;  // green (G element full on)
const uint8_t CYN  = 0x1f;  // cyan (GB elements full on)
const uint8_t BLU  = 0x03;  // blue (B element full on)
const uint8_t MGT  = 0xe2;  // magenta (RB elements full on)
const uint8_t ZRED = 0x20;  // the darkest red
const uint8_t ZYEL = 0x24;  // the darkest yellow
const uint8_t ZGRN = 0x04;  // the darkest green
const uint8_t ZCYN = 0x05;  // the darkest cyan
const uint8_t ZBLU = 0x01;  // the darkest blue
const uint8_t ZMGT = 0x21;  // the darkest magenta
const uint8_t DRED = 0x80;  // dark red
const uint8_t BORG = 0xe8;  // blood orange (very reddish orange)
const uint8_t SALM = 0xc9;  // salmon (0xe9 is less muted)
const uint8_t BRN  = 0x88;  // dark orange aka brown
const uint8_t DBRN = 0x44;  // dark brown
const uint8_t ORG  = 0xf0;  // orange
const uint8_t LYEL = 0xfe;  // lemon yellow
const uint8_t GROD = 0xf9;  // goldenrod
const uint8_t MYEL = 0xd5;  // mustard yellow
const uint8_t GGRN = 0x9e;  // a low saturation greyish pastel green
const uint8_t LGRN = 0x55;  // a muted lime green
const uint8_t TEAL = 0x1e;  // this teal is barely distinguishable from cyan
const uint8_t STBL = 0x9b;  // steel blue is desaturated light blue
const uint8_t DCYN = 0x12;  // dark cyan
const uint8_t RBLU = 0x0b;  // royal blue
const uint8_t MBLU = 0x02;  // midnight blue
const uint8_t INDG = 0x43;  // indigo (deep blue with a hint of purple)
const uint8_t ORCD = 0xaf;  // orchid (lighter and less saturated purple)
const uint8_t VIO  = 0x83;  // violet
const uint8_t PUR  = 0x63;  // purple
const uint8_t GPUR = 0x6a;  // a low saturation greyish pastel purple
const uint8_t LPUR = 0xd3;  // a light pastel purple
const uint8_t PNK  = 0xe3;  // pink is the best color
const uint8_t MPNK = 0xeb;  // we need all shades of pink
const uint8_t LPNK = 0xf3;  // especially light pink, the champagne of pinks
const uint8_t NON  = 0x45;  // used as default value when color is unspecified

uint8_t colorcard[NumRunModes] = { MGT, WHT, RED, ORG, YEL, GRN, TEAL, PUR };

// kick_inactivity_timer() function to call whenever human activity occurs, for accurate inactivity timeout feature
//   integer argument encodes which source of human activity has kicked the timer. Here are the codes:
enum human_activities { HuNone=-1, HuMomDown=0, HuMomUp=1, HuEncTurn=2, HuTouch=3, HuRCTog=4, HuRCJoy=5, HuRCTrig=6, HuPot=7, HuTogSw=8, HuNumActivities=9 };
std::string activitiescard[HuNumActivities] = { "buttdn", "buttup", "encodr", "touch", "rcbutt", "rcjoy", "rctrig", "pot", "toggle" };
Timer user_inactivity_timer;  // how long of not touching it before it goes to low power mode
int last_activity = HuNone;
void kick_inactivity_timer(int source=-1) {
    if (source < 0) return;
    user_inactivity_timer.reset();  // evidence of user activity
    last_activity = source;
    // ezread.squintf("kick%d ", source);
}

// class AbsTimer {  // absolute timer ensures consecutive timeouts happen on regular intervals
//   protected:
//     volatile int64_t end, timeout_us;
//   public:
//     AbsTimer() { reset(); }
//     AbsTimer(int arg_timeout) { set ((int64_t)arg_timeout); }
//     void IRAM_ATTR set (int64_t arg_timeout) {
//         timeout_us = arg_timeout;
//         end = esp_timer_get_time() + timeout_us;
//     }
//     void IRAM_ATTR set() { end = esp_timer_get_time() + timeout_us; }  // use to rezero the timer phase
//     void IRAM_ATTR reset() {  // move expiration to the next timeout multiple
//         int64_t now = esp_timer_get_time();
//         if (now >= end) end += timeout_us * (1 + (now - end) / timeout_us);
//     }
//     bool IRAM_ATTR expired() { return esp_timer_get_time() >= end; }
//     int64_t IRAM_ATTR elapsed() { return esp_timer_get_time() + timeout_us - end; }
//     int64_t timeout() { return timeout_us; }    
//     // never finished writing this ...
//     //
//     // bool IRAM_ATTR expireset() {  // Like expired() but immediately resets if expired
//     //     int64_t now_us = esp_timer_get_time();
//     //     if (now >= end) 
//     //     end += timeout_us * (1 + (now - end) / timeout_us);
//     //     if (now_us < start_us + timeout_us) return false;
//     //     start_us = now_us;
//     //     return true;
//     // }
// };

// EZRead is a text-logging console for display on a small low-res LCD in a window (or fullscreen if you feel like coding it).
//   the output text is very space-efficient, except the most recent message at bottom, which is zoomed in "enormously".
//   the user-obsessed legibility of EZRead is something you'll write home to your parents about after every use
//   lookback(int);   scrolls the given number of lines into the past
//   printf();   writes formatted text to ezread. use the same arguments as printf, plus an optional rgb332 color as the 1st arg
//   squintf();  like running ezread.printf(...) followed by Serial.printf(...) w/ the same arguments. splits content to both.
//               ok to use the optional extra 1st argument to color the ezread content, this is ignored by the console content.
//               the documentation explains why this is called squintf, but I wasn't able to read it w/o my eye loupe.
//   debugf();   is like squintf() except it will always wait between prints to avoid causing a deluge (for debugging in loops)
//   arguments for last two: ([optional uint8_t color], "printf-compatible format string", <other_printf_supported_args>);
#include <iostream>
#include <string>
#include <sstream>
#include <stdarg.h>
class EZReadConsole {
  private:  // behavior parameters for ezread's data spam suppression feature
    int spam_enable_thresh_cps = 3500;  // threshold data rate (avg over window) beyond which begins spam suppression
    int spam_disable_thresh_cps = 650;  // threshold data rate (avg over window) below which ends spam suppression
    int spam_window_us = 200000;  // console history epoch over which to calculate average data rate into buffer
    Timer passthrutimer{300000};  // during suppressing spam, allow one print call to slip thru this often
    Timer updatetimer{20000};
    // int boot_graceperiod_timeout_us = 3500000;  // spam detection is suspended for this long after intitial boot
  public:
    bool ezread_serial_console_enabled = console_enabled;  // when true then ezread.squintf() does the same thing as ezread.printf()
    bool dirty = true, has_wrapped = false, graceperiod = true, graceperiod_valid = true;  // on boot spam detector is in a grace period for the boot messages, until end_bootgraceperiod() is called
    Timer offsettimer{60000000};  // if scrolled to see history, after a delay jump back to showing most current line
    EZReadConsole() {}
    static constexpr int num_lines = 300;
    static constexpr int bufferSize = num_lines;
    int maxlength=40, last_drawn = bufferSize; // size_t bufferSize; // Size of the ring buffer
    std::string textlines[bufferSize];
    int newest_content = bufferSize, current_index = 0, offset = 0;
    uint8_t linecolors[num_lines], color, usecolor;
    uint8_t defaultcolor = LGRY, happycolor = LGRN, sadcolor = ORG, madcolor = RED, announcecolor = LPUR, highlightcolor = DCYN;    // std::vector<std::string> textlines; // Ring buffer array
    bool spam_active = false, spam_notice_shown = false;
    float avg_spamrate_cps = 0.0f, window_accum_char = 0.0f;  // variables to dynamically manage moving average
    void update() {
        if (graceperiod && !graceperiod_valid) {
            graceperiod = false;
            updatetimer.reset();
        }
        if (!ezread_suppress_spam || graceperiod || !updatetimer.expireset()) return;
        window_accum_char = std::max(0.0f, window_accum_char - avg_spamrate_cps * updatetimer.timeout() / 1e6f);  // let old spam fall out of the buffer.
        cleanzero(&window_accum_char, 0.1);
        avg_spamrate_cps = window_accum_char * 1e6f / spam_window_us;  // calc a new avg rate.
        cleanzero(&avg_spamrate_cps, 0.1);
        if (spam_active && ((int)avg_spamrate_cps < spam_disable_thresh_cps)) {
            spam_active = false;
            this->printf(happycolor, "ezread spam suppression off\n");
        }
        else if (!spam_active && ((int)avg_spamrate_cps > spam_enable_thresh_cps)) {
            spam_active = true;
            this->printf(sadcolor, "ezread spam suppression on\n");
        }
    }
    void end_bootgraceperiod() {
        graceperiod_valid = false;
    }
  private:
    int last_allowed_us = 0;
    bool should_allow_output(size_t upcoming_chars) {
        if (!ezread_suppress_spam || graceperiod) return true;
        window_accum_char += upcoming_chars;  // add new spam to buffer
        if (!spam_active) return true;
        return passthrutimer.expireset();  // only return true upon passthru timer expiration, otherwise false
    }
    void printf_impl(uint8_t _color, const char* format, va_list args) {  // this is not called directly but by one ots overloads below
        char preview[100];
        va_list args_copy;
        va_copy(args_copy, args);
        vsnprintf(preview, sizeof(preview), format, args_copy);
        va_end(args_copy);
        if (!should_allow_output(strlen(preview))) return;  // if suppression is active then disregard this call altogether, abandoning the data
        char temp[100];
        color = _color;
        vsnprintf(temp, sizeof(temp), format, args);
        std::string str = temp;
        std::string::size_type start = 0;
        std::string::size_type end = str.find_first_of("\r\n");
        textlines[current_index] = "";  // erase the current buffer entry (b/c old data will wrap around the ring buffer)
        while (end != std::string::npos) {  // if string contains at least one newline, chop off up to the first one and tack onto current line, and enqueue
            textlines[current_index] += str.substr(start, end - start);  // Append content up to the first newline
            linecolors[current_index] = color;  // Set color for this line
            start = end + 1;
            end = str.find_first_of("\r\n", start);
            textlines[current_index] = remove_nonprintable(textlines[current_index]);
            ++current_index %= bufferSize; // Update next insertion index
            if (current_index == 0) has_wrapped = true;
        }
        if (start < str.size()) {
            textlines[current_index] += str.substr(start);  // Append the remaining part of the string
            textlines[current_index] = remove_nonprintable(textlines[current_index]);
            linecolors[current_index] = color;
        }
        dirty = true;
    }
    std::string remove_nonprintable(const std::string& victim) {
        std::string result;
        for (char ch : victim) {
            // if (ch == '\r' || ch == '\n') result += " | "; else
            if (isprint(static_cast<unsigned char>(ch))) result += ch;
        }
        return result;
    }
  public:
    void setup() {
        std::string blank = "";
        for (int i=0; i<bufferSize; i++) {
            // linecolors[i] = MGRY;
            this->printf("%s", blank.c_str());
            linecolors[i] = defaultcolor;
        }
        // this->printf(highlightcolor, "Welcome to EZ-Read console\n");
        dirty = true;
    }
    void printf(const char* format, ...) {  // for if we're called with same arguments as printf would take
        va_list args;
        va_start(args, format);
        printf_impl(defaultcolor, format, args);  // Use default color
        va_end(args);
    }
    void printf(uint8_t color, const char* format, ...) {  // otherwise you can insert a custom color as the first argument
        va_list args;
        va_start(args, format);
        printf_impl(color, format, args);  // Use provided color
        va_end(args);
    }
    void squintf(const char* format, ...) {  // prints string to both serial and ezread consoles, except you have to squint to see it
        va_list args;
        va_start(args, format);
        printf_impl(defaultcolor, format, args);
        va_end(args);
        if (ezread_serial_console_enabled) {
            char temp[100];
            va_list args;  // suggested by ai tool
            va_start(args, format);  // suggested by ai tool
            vsnprintf(temp, sizeof(temp), format, args);
            va_end(args);  // suggested by ai tool
            Serial.printf("%s", temp);
            Serial.flush();
        }
    }
    void squintf(uint8_t color, const char* format, ...) {  // prints string to both serial and ezread consoles, except you have to squint to see it
        va_list args;
        va_start(args, format);
        printf_impl(color, format, args);  
        va_end(args);
        if (ezread_serial_console_enabled) {
            char temp[100];
            va_list args;  // suggested by ai tool
            va_start(args, format);  // suggested by ai tool
            vsnprintf(temp, sizeof(temp), format, args);
            va_end(args);  // suggested by ai tool
            Serial.printf("%s", temp);
            Serial.flush();
        }
    }
    void debugf(const char* format, ...) {
        static Timer debugtimer{1000};
        if (!debugtimer.expired()) return;
        debugtimer.set(passthrutimer.timeout());
        char temp[100];
        va_list args;
        va_start(args, format);
        vsnprintf(temp, sizeof(temp), format, args);
        va_end(args);
        this->squintf(highlightcolor, "%s", temp);
    }
    void lookback(int off) {
        int offset_old = offset;
        offset = constrain(off, 0, bufferSize);  //  - ez->num_lines);
        if (offset) offsettimer.reset();
        if (offset != offset_old) dirty = true;
    }
};
static EZReadConsole ezread;

// most and least significant place functions:  pass them an int or a float, returns the decimal place
// of the most|least signigicant digit. Avoids using pow() functions by using looped multiplies/divides.
// examples [most|least]: 1234.5678 [4|-4], 1.0 [1|1], 1500 [4|3], 0.45 [-1|-2], 0.0 [1|1] (or [0|0] w/ arg)
// used for sensibly guessing precision of edits, and for efficiently displaying numeric values
template<typename T>  // note by default will return 1 if value is 0
int most_significant_place(T value, int zeroplace=1) {
    if constexpr (std::is_integral<T>::value) {  // if value is an integer
        if (value == 0) return zeroplace;
        value = std::abs(value);
        int place = 1;
        while (value >= 10) {
            value /= 10;
            place++;
        }
        return place;
    }
    else {  // if value is a float
        if (value == 0.0f) return zeroplace;  // if (iszero(value)) return zeroplace; breaks tune crossing zero
        value = std::fabs(value);  // safer to directly use fabs within a type-ambiguous template
        int place = 0;
        while (value >= 10.0) {
            value /= 10.0;
            place++;
        }
        while (value < 1.0) {
            value *= 10.0;
            place--;
        }
        return place;
    }
}
template<typename T>  // note by default will return 1 if value is 0
int least_significant_place(T value, int zeroplace=1) {
    if constexpr (std::is_integral<T>::value) {
        if (value == 0) return zeroplace;
        value = std::abs(value);
        int place = 1;
        while ((value % 10) == 0) {
            value /= 10;
            place++;
        }
        return place;
    }
    else {
        if (value == 0.0f) return zeroplace;  // bad: if (iszero(value)) return zeroplace;
        value = std::fabs(value);  // safer to directly use fabs within a type-ambiguous template
        int place = 0;
        float test = static_cast<float>(value);
        while (place > -10 && std::fabs(test - std::round(test)) > 1e-6f) {
            test *= 10.0f;
            place--;
        }
        return place;
    }
}

// tune() : returns a modified float/int/bool value according to given [nonzero int] delta value. respects temporal acceleration used by touchscreen and encoder classes.  replaces adj_val() function
// alternately, give a pointer instead of a number to change the [float|int|bool] value directly instead of returning it.  call w/ only idelta value to get a corresponding bool (<=0 values give false)
// numeric edits are scaled proportional to the magnitude of the current value. or you can specify a minimum decimal place to scale to, presumably b/c otherwise this makes it impossible to cross zero
// optional min/max values may be supplied for int|float edits and if so are respected.  in case these are not needed but subsequent optional args are, set to NAN (float) or unlikely_int (int) to ignore
// for integer values, there is an optional "dropdown" argument, which if set true will disable any temporal acceleration (constrains edit to w/i -1,1).  useful for when selecting options from lists

float tunetest = 1.234567f;  // just for debugging tune() zero crossing behavior
// int tunetest_enc_accel = 1, tunetest_enc_id = 0, tunetest_ts_id = 0;
// float tunetest_lastval = tunetest, tunetest_change = 0.0f; 
// int edit_id_persist = 0;

#define disp_default_float_sig_dig 3  // significant digits displayed for float values. Higher causes more screen draws.  (?)If changed then also must change scale in tune() function!
float tune(float orig_val, int idelta, float min_val=NAN, float max_val=NAN, int min_sig_edit_place=unlikely_int) {  // feed in float value, get new constrianed float val, modified by idelta scaled to the magnitude of the value
    int sig_digits = disp_default_float_sig_dig;
    if (min_sig_edit_place == unlikely_int) min_sig_edit_place = -1 * disp_default_float_sig_dig;
    int sig_place = std::max(least_significant_place(orig_val), min_sig_edit_place + sig_digits);
    float scale = 1.0;  // needs to change if disp_default_float_sig_dig is modified !!
    while (sig_place > sig_digits) {
        scale *= 10.0;
        sig_place--;
    }
    while (sig_place < sig_digits) {
        scale /= 10.0;
        sig_place++;
    }
    float ret = orig_val + (float)(idelta) * scale;
    if (((orig_val > 0.0f) && (ret <= 0.0f)) || ((orig_val < 0.0f) && (ret >= 0.0f))) ret = 0.0f;
    if (std::isnan(min_val)) min_val = ret;
    if (std::isnan(max_val)) max_val = ret;
    ret = constrain(ret, min_val, max_val);  // Serial.printf("o:%lf id:%d sc:%lf, min:%lf, max:%lf ret:%lf\n", orig_val, idelta, scale, min_val, max_val, ret);
    return ret;
}
int tune(int orig_val, int idelta, int min_val=unlikely_int, int max_val=unlikely_int, bool dropdown=false) {  // feed in int value, get new constrianed int val, modified by idelta scaled to the magnitude of the value
    int sig_place = least_significant_place(orig_val);
    int scale = 1;
    if (dropdown) idelta = constrain(idelta, -1, 1);
    else while (sig_place > 4) {
        scale *= 10;
        sig_place--;
    }
    int ret = orig_val + idelta * scale;
    if ((max_val <= min_val) || (max_val == unlikely_int)) max_val = ret;
    if (min_val == unlikely_int) min_val = ret;
    return constrain(ret, min_val, max_val);
}
bool tune(int idelta) {  // overloaded to return bool value. idelta == 0 or -1 return false and 1+ returns true.
    bool ret = (idelta > 0);
    return ret;
}
void tune(float* orig_ptr, int idelta, float min_val=NAN, float max_val=NAN, int sig_digits=-1) {  // overloaded to directly modify float at given address
    *orig_ptr = tune(*orig_ptr, idelta, min_val, max_val, sig_digits);
}
void tune(int* orig_ptr, int idelta, int min_val=-1, int max_val=-1, bool dropdown=false) {  // overloaded to directly modify int at given address
    *orig_ptr = tune(*orig_ptr, idelta, min_val, max_val, dropdown);
}
void tune(bool* orig_ptr, int idelta) {  // overloaded to directly modify bool at given address
    *orig_ptr = tune(idelta);
}

#include <random>
std::random_device rd;
std::mt19937 gen(rd());  // randomizer
int rn(int values=256) {  // Generate a random number between 0 and values-1
    std::uniform_int_distribution<> dis(0, values - 1);
    return dis(gen);
}
