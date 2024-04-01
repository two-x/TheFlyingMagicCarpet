// globals.h - not dependent on anything, so include this first
#pragma once
#include "Arduino.h"
// pin assignments  ESP32-S3-DevkitC series   (note: "*" are pins we can reclaim if needed)
#define     boot_sw_pin  0 // button0/strap1   // input, the esp "boot" button. if more pins are needed, move encoder_sw_pin to this pin, no other signal of ours can work on this pin due to high-at-boot requirement
#define      tft_dc_pin  1 // adc1.0           // output, assert when sending data to display chip to indicate commands vs. screen data
#define    touch_cs_pin  2 // adc1.1         * // output, chip select for resistive touchscreen, active low. Not used on car, so can assign pin to something else if needed
#define   sdcard_cs_pin  3 // adc1.2/strapX  * // output, chip select for SD card controller on SPI bus. sdcard functionality is not implemented, we may not even need it
#define    mulebatt_pin  4 // adc1.3           // analog input, mule battery voltage sense, full scale is 16V
#define         pot_pin  5 // adc1.4           // analog in from 20k pot
#define   brake_pos_pin  6 // adc1.5           // analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#define    pressure_pin  7 // adc1.6           // analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
#define     i2c_sda_pin  8 // sda0/adc1.7      // i2c bus for airspeed/map sensors, lighting board, cap touchscreen
#define     i2c_scl_pin  9 // qhd0/scl0/adc1.8 // i2c bus for airspeed/map sensors, lighting board, cap touchscreen
#define      tft_cs_pin 10 // cs0/adc1.9       // output, active low, chip select allows ILI9341 display chip use of the spi bus
#define    spi_mosi_pin 11 // mosi0/adc2.0     // used as spi interface data for tft screen, sd card and resistive touch panel
#define    spi_sclk_pin 12 // sclk0/adc2.1     // used as spi interface clock for tft screen, sd card and resistive touch panel
#define    spi_miso_pin 13 // miso0/adc2.2     // used as spi interface data from sd card, resistive touch panel, and possibly (?) tft screen
#define hotrc_ch2_v_pin 14 // qwp0/pwm0/adc2.3 // hotrc ch2 bidirectional trigger input
#define hotrc_ch1_h_pin 15 // pwm1/adc2.4      // hotrc ch1 thumb joystick input
#define     gas_pwm_pin 16 // pwm1/adc2.5      // output, pwm signal duty cycle controls throttle target. on Due this is the pin labeled DAC1 (where A13 is on Mega)
#define   brake_pwm_pin 17 // pwm0/adc2.6/tx1  // output, pwm signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped)
#define   steer_pwm_pin 18 // pwm0/adc2.7/rx1  // output, pwm signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). jaguar asks for an added 150ohm series R when high is 3.3V
#define        free_pin 19 // usb-d-/adc2.8  * // reserved for usb or a steering quadrature encoder.
#define    fuelpump_pin 20 // usb-d+/adc2.9  * // alternate fuel pump power signal, or use for usb or a steering quadrature encoder. 
#define     onewire_pin 21 // pwm0             // onewire bus for temperature sensor data. note: tested this does not work on higher-numbered pins (~35+)
#define      speedo_pin 35 // spiram/octspi    // int input, active high, asserted when magnet south is in range of sensor. 1 pulse per driven pulley rotation. (Open collector sensors need pullup)
#define     starter_pin 36 // sram/ospi/glitch // input/Output (both active high), output when starter is being driven, otherwise input senses external starter activation
#define        tach_pin 37 // spiram/octspi    // int Input, active high, asserted when magnet south is in range of sensor. 1 pulse per engine rotation. (no pullup) - note: placed on p36 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on
#define  encoder_sw_pin 38 // spiram/octspi  * // input, Rotary encoder push switch, for the UI. active low (needs pullup). signal can be moved to pin 0 to free up this pin. Pin 38 is the neopixel pin on v1.1 boards
#define basicmodesw_pin 39 // jtck/glitch      // input, asserted to tell us to run in basic mode, active low (has ext pullup) - note: placed on p39 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
#define   hotrc_ch4_pin 40 // jtdo             // syspower, starter, and cruise mode toggle control. hotrc ch4 pwm toggle signal
#define   hotrc_ch3_pin 41 // jtdi             // ignition control, hotrc Ch3 PWM toggle signal
#define   encoder_a_pin 42 // jtms             // int input, the A (aka CLK) pin of the encoder. both A and B complete a negative pulse in between detents. if A pulse goes low first, turn is CCW. (needs pullup)
#define     uart_tx_pin 43 // "TX"/tx0         // serial monitor data out. Also used to detect devboard vs. pcb at boot time (using pullup/pulldown, see below)
#define     uart_rx_pin 44 // "RX"/rx0         // serial monitor data in. maybe could repurpose during runtime since we only need outgoing console data?
#define    ignition_pin 45 // strap0           // output to an nfet/pfet pair to control the car ignition
#define    syspower_pin 46 // strap0           // output to an nfet/pfet pair to power all the tranducers.
#define   encoder_b_pin 47 // NA               // int input, the B (aka DT) pin of the encoder. both A and B complete a negative pulse in between detents. if B pulse goes low first, turn is CW. (needs pullup)
#define    neopixel_pin 48 // neopix           // data line to onboard neopixel WS281x (on all v1 devkit boards - pin 38 is used on v1.1 boards). Also used for onboard and external neopoxels - ! pin is also defined in neopixel.h
// external components needed (pullup/pulldown resistors, capacitors, etc.): (note: "BB" = on dev breadboards only, "PCB" = on vehicle PCB only)
// 1. onewire_pin, tach_pin, speedo_pin: Add 4.7k-ohm to 3.3V, needed for open collector sensor output to define logic-high voltage level.
// 2. uart_tx_pin: (PCB) add 22k-ohm to GND. (BB) connect the 22k-ohm to 3.3V instead. for boot detection of vehicle PCB, so defaults are set appropriately.
// 3. resistor dividers are needed for these inputs: starter_pin (16V->3.3V), mulebatt_pin (16V->3.3V), and pressure_pin (5V->3.3V).
// 4. ignition_pin, syspower_pin, starter_pin: require pulldowns to gnd, this is provided by nfet gate pulldown.
// 5. gas_pwm_pin: should have a series ~680-ohm R going to the servo.
// 6. encoder_a_pin, encoder_b_pin, button_pin: should have 10nF to gnd, tho it should work w/o it. pullups to 3.3V (4.7k-ohm is good) are also necessary, but the encoder we're using includes these.
// 7. neopixel_pin: (PCB) Add 330 ohm in series (between pin and the DataIn pin of the 1st pixel). (BB) same, but this one is likely optional, e.g. mine works w/o it.  for signal integrity over long wires. 
// 6. mulebatt_pin, pressure_pin, brake_pos_pin, pot_wipe_pin: (ADC inputs) should have 100nF cap to gnd, to improve signal stability, tho it works w/o it.

// note onewire works on pins 19-21 but not on pins 39-42
// if one more pin becomes needed, encoder_sw may be moved to pin 0, freeing up pin 38 (pin 0 requires a pullup, which encoder_sw has)
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
// bm2023 pins: tft_dc_pin 3, onewire_pin 19, hotrc_ch3_pin 20, hotrc_ch4_pin 21, tach_pin 36, ignition_pin 37, syspower_pin 38, encoder_b_pin 40, encoder_a_pin 41, encoder_sw_pin 42, starter_pin 45, sdcard_cs_pin 46, touch_cs_pin 47
#ifdef USE_BM23_PINS       // for testing using box from bm23, override new pin assignments with old ones
#define fuelpump_pin 1     // not used in bm23, but moving to an unused pin
#define free_pin 2         // not used in bm23, but moving to an unused pin
#define tft_dc_pin 3
#define onewire_pin 19
#define hotrc_ch3_pin 20`
#define hotrc_ch4_pin 21
#define tach_pin 36
#define ignition_pin 37
#define syspower_pin 38
#define encoder_b_pin 40
#define encoder_a_pin 41
#define encoder_sw_pin 42
#define starter_pin 45
#define sdcard_cs_pin 46
#define touch_cs_pin 47
#endif

#define adcbits 12
#define adcrange_adc 4095     // = 2^adcbits-1
#define adcmidscale_adc 2047  // = 2^(adcbits-1)-1

// these global enums are super convenient, just take care when making changes
enum hotrc_axis : int { HORZ=0, VERT=1, CH3=2, CH4=3 };
enum hotrc_val : int { OPMIN=0, CENT=1, OPMAX=2, RAW=3, FILT=4, DBBOT=5, DBTOP=6 };
enum motor_val : int { PARKED=1, OUT=3, GOVERN=4 , ABSMIN=5, ABSMAX=6, MARGIN=7, NUM_MOTORVALS=8 };
enum stop_val : int { STOP=1 };
enum steer_val : int { SAFE=1 };
enum size_enums : int { NUM_AXES=2, NUM_CHANS=4, NUM_VALUS=8 };
enum joydirs : int { JOY_RT=-2, JOY_DN=-1, JOY_CENT=0, JOY_UP=1, JOY_LT=2, JOY_PLUS=3, JOY_MINUS=4 };
enum runmode : int { BASIC, ASLEEP, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL, NUM_RUNMODES };
enum req : int { REQ_NA=-1, REQ_OFF=0, REQ_ON=1, REQ_TOG=2 };  // requesting handler actions of digital values with handler functions
enum cruise_modes : int { PID_SUSPEND_FLY, THROTTLE_ANGLE, THROTTLE_DELTA };
enum sw_presses : int { swNONE, swSHORT, swLONG };
enum motor_modes : int { NA=-1, Halt=0, Idle=1, Release=2, OpenLoop=3, ActivePID=4, AutoStop=5, AutoHold=6, ParkMotor=7, Cruise=8, Calibrate=9, NumMotorModes=10 };
enum brake_pid_modes : int { PositionPID=0, PressurePID=1, HybridPID=2, NumBrakePIDs=4 };  // OpenLoop=3
enum tunerstuff : int { ERASE=-1, OFF=0, SELECT=1, EDIT=2 };
enum boolean_states : int { ON=1 };
enum datapages : int { PG_RUN, PG_JOY, PG_SENS, PG_PWMS, PG_IDLE, PG_BPID, PG_GPID, PG_CPID, PG_TEMP, PG_SIM, PG_UI, NUM_DATAPAGES };
enum temp_categories : int { AMBIENT=0, ENGINE=1, WHEEL=2, NUM_TEMP_CATEGORIES=3 };  // 
enum temp_lims : int { DISP_MIN=1, WARNING=3, ALARM=4, DISP_MAX=5 };   // possible sources of gas, brake, steering commands
enum ui_modes : int { DatapagesUI=0, ScreensaverUI=1 };
enum codemodes : int { Confused=0, Booting=1, Parked=2, Driving=3 };
enum telemetry_short : int { _GasServo=0, _BrakeMotor=1, _SteerMotor=2, _HotRC=3, _Speedo=4, _Tach=5, _BrakePres=6, _BrakePosn=7, _Temps=8, _Other=9, _GPIO=10 };  // _MuleBatt, _MAP, _MAF, _Pot,
enum telemetry_full : int { _HotRCHorz=11, _HotRCVert=12, _MuleBatt=13, _AirVelo=14, _MAP=15, _Pot=16, _MAF=17, _TempEng=18, _TempWhFL=19, _TempWhFR=20, _TempWhRL=21, _TempWhRR=22, _TempAmb=23 };  // 10 per line
enum telemetry_nums : int { _None=-1, NumTelemetryBool=9, NumTelemetryShort=11, NumTelemetryFull=24 };
enum telemetry_bool : int { _Ignition=1, _PanicStop=2, _SysPower=3, _HotRCCh3=4, _HotRCCh4=5, _StarterDr=6, _StarterExt=7, _BasicSw=8 };
enum err_type : int { LOST=0, RANGE=1, VALUE=2, STATE=3, WARN=4, CRIT=5, INFO=6, NUM_ERR_TYPES=7 };

// global configuration settings
bool brake_hybrid_pid = true;
int brake_default_pid = PressurePID;
bool brake_linearize_target_extremes = true;  // keep brake target values linear near endpoints despite decreasing accuracy of one of the sensors? (this is the more accurate choice)  otherwise jump inaccurate-sensor influence near endpoint to actual endpoint (this is more predictable/consistent)
bool starter_signal_support = true;
bool remote_start_support = true;
bool autostop_disabled = false;      // temporary measure to keep brake behaving until we get it debugged. Eventually should be false
bool allow_rolling_start = true;    // may be a smart prerequisite, may be us putting obstacles in our way
bool flip_the_screen = false;
bool cruise_speed_lowerable = true;  // allows use of trigger to adjust cruise speed target without leaving cruise mode.  Otherwise cruise button is a "lock" button, and trigger activity cancels lock
bool display_enabled = true;         // should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool use_i2c_baton = true;
bool always_max_refresh = false;
bool brake_before_starting = true;
bool watchdog_enabled = false;
bool fuelpump_supported = true;
int throttle_ctrl_mode = OpenLoop;
// dev-board-only options:  Note these are ignored and set false at boot by set_board_defaults() unless running on a breadboard with a 22k-ohm pullup to 3.3V the TX pin
// bool usb_jtag = true;                // if you will need the usb otg port for jtag debugging (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/jtag-debugging/configure-builtin-jtag.html)
bool dont_take_temperatures = false; // in case debugging dallas sensors or causing problems
bool console_enabled = true;         // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
bool keep_system_powered = false;    // use true during development
bool looptime_print = false;         // makes code write out timestamps throughout loop to serial port
bool touch_reticles = true;
bool button_test_heartbeat_color = false; // Encoder short press when not tuning makes heartbeat a random color (for testing)
bool wifi_client_mode = false;       // Should wifi be in client or access point mode?
bool saver_on_sleep = true;
bool print_framebuffers = false;

// global tunable variables
int sprite_color_depth = 8;
uint32_t looptime_linefeed_threshold = 0;   // when looptime_print == 1, will linefeed after printing loops taking > this value. Set to 0 linefeeds all prints
float flycruise_vert_margin_pc = 0.3;       // Margin of error for determining hard brake value for dropping out of cruise mode
int cruise_setpoint_scheme = THROTTLE_DELTA;
int32_t cruise_delta_max_pc_per_s = 16;  // (in THROTTLE_DELTA mode) What's the fastest rate cruise adjustment can change pulse width (in us per second)
float cruise_angle_attenuator = 0.016;   // (in THROTTLE_ANGLE mode) Limits the change of each adjust trigger pull to this fraction of what's possible
float temp_lims_f[3][6]{
    {45.0, 0.0, 115.0, 120.0, 130.0, 220.0},  // [AMBIENT] [OPMIN/DISP_MIN/OPMAX/WARNING/ALARM]
    {178.0, 0.0, 198.0, 202.0, 205.0, 220.0}, // [ENGINE] [OPMIN/DISP_MIN/OPMAX/WARNING/ALARM]
    {50.0, 0.0, 120.0, 130.0, 140.0, 220.0},  // [WHEEL] [OPMIN/DISP_MIN/OPMAX/WARNING/ALARM] (applies to all wheels)
};
float temp_room = 77.0;          // "room" temperature is 25 C = 77 F  Who cares?
float temp_sensor_min_f = -67.0; // minimum reading of sensor is -25 C = -67 F
float temp_sensor_max_f = 257.0; // maximum reading of sensor is 125 C = 257 F
float maf_min_gps = 0.0;
float maf_max_gps = 50.0; // i just made this number up as i have no idea what's normal for MAF
bool flashdemo = false;
int32_t neobright = 10;   // default for us dim/brighten the neopixels
int32_t neodesat = 0;     // default for lets us de/saturate the neopixels
float tuning_rate_pcps = 7.5;  // values being edited by touch buttons change value at this percent of their overall range per second

// non-tunable values. probably these belong with their related code
uint bootcount;                         // variable to track total number of boots of this code build
uint codemode = Booting;
uint codemode_postmortem;
bool running_on_devboard = false;       // will overwrite with value read thru pull resistor on tx pin at boot
bool shutdown_incomplete = true;        // minor state variable for shutdown mode - Shutdown mode has not completed its work and can't yet stop activity
bool parking = false;                   // indicates in process of parking the brake & gas motors so the pedals can be used manually without interference
bool releasing = false;                 // indicates in process of releasing the brake to the zero brake point
bool cruise_adjusting = false;
bool cal_brakemode = false;             // allows direct control of brake motor using controller vert
bool cal_brakemode_request = false;             // allows direct control of brake motor using controller vert
bool cal_gasmode_ready = false;         // whether pot is in valid range
bool cal_gasmode = false;               // allows direct control of gas servo using pot. First requires pot to be in valid position before mode is entered
bool cal_gasmode_request = false;
bool autostopping = false;              // true when in process of stopping the car (hold or shutdown modes)
bool autoholding = false;              // true when in actively preventing the stopped car from moving (hold or shutdown modes)
bool car_hasnt_moved = false;           // minor state variable for fly mode - Whether car has moved at all since entering fly mode
bool powering_up = false;               // minor state variable for asleep mode
bool calmode_request = false;
bool flycruise_toggle_request = false;
bool screensaver = false;                // can enable experiment with animated screen draws
bool basicmode_request = false;
int tunctrl = OFF, tunctrl_last = OFF;
int datapage = PG_RUN, datapage_last = PG_TEMP;  // which of the dataset pages is currently displayed and available to edit?
volatile int sel_val = 0;               // in the real time tuning UI, which of the editable values is selected. -1 for none 
volatile int sel_val_last = 0;          
volatile int sel_val_last_last = 0;          
bool syspower = HIGH;                   // set by handler only. Reflects current state of the signal
bool ignition = LOW;                    // set by handler only. Reflects current state of the signal
bool panicstop = false;                 // initialize NOT in panic, but with an active panic request, this puts us in panic mode with timer set properly etc.
bool basicmodesw = LOW;
int ignition_request = REQ_NA;
int panicstop_request = REQ_ON;         // on powerup we assume the code just rebooted during a drive, because for all we know it could have 
int sleep_request = REQ_NA;
float maf_gps = 0;                      // manifold mass airflow in grams per second
uint16_t heartbeat_override_color = 0x0000;
bool nowtouch = false;
bool captouch = true;
float loop_avg_us;
bool sensidiots[11];
bool web_disabled = false;
int ui_context = DatapagesUI;
// bool sensor_present[telemetry_full];

// fast macros
#define arraysize(x) ((int32_t)(sizeof(x) / sizeof((x)[0])))  // a macro function to determine the length of string arrays
#define floor(amt, lim) ((amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)
#undef min
#undef max
inline float smax(float a, float b) { return (a > b) ? a : b; }
inline int32_t smax(int32_t a, int32_t b) { return (a > b) ? a : b; }
inline uint32_t smax(uint32_t a, uint32_t b) { return (a > b) ? a : b; }
inline uint32_t smax(uint32_t a, uint32_t b, uint32_t c) { return (a > b) ? ((c > a) ? c : a) : ((c > b) ? c : b); }
inline float smin(float a, float b) { return (a < b) ? a : b; }
inline int32_t smin(int32_t a, int32_t b) { return (a < b) ? a : b; }
inline uint32_t smin(uint32_t a, uint32_t b) { return (a < b) ? a : b; }
inline uint32_t smin(uint32_t a, uint32_t b, uint32_t c) { return (a < b) ? ((c < a) ? c : a) : ((c < b) ? c : b); }
#undef constrain
inline float constrain(float amt, float low, float high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline int32_t constrain(int32_t amt, int32_t low, int32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline uint32_t constrain(uint32_t amt, uint32_t low, uint32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline long constrain(long amt, long low, long high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
// the above should be templated, i'm sure. i tried the following but it didn't work
// template <typename T> inline T smax(T a, T b) { return (a > b) ? a : b; }
// template <typename T> inline T smax(T a, T b, T c) { return (a > b) ? ((c > a) ? c : a) : ((c > b) ? c : b); }
// template <typename T> inline T smin(T a, T b) { return (a < b) ? a : b; }
// template <typename T> inline T smin(T a, T b, T c) { return (a < b) ? ((c < a) ? c : a) : ((c < b) ? c : b); }
// template <typename T> inline T constrain(T amt, T low, T high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
// template <typename T> inline T map(T x, T in_min, T in_max, T out_min, T out_max) {
//     if (in_max - in_min > (T)0.001) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
//     printf ("map not dividing by zero\n");
//     return out_max;  // Instead of dividing by zero, return the highest valid result
// }
#undef map
inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // instead of dividing by zero, return the highest valid result
}
inline int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // instead of dividing by zero, return the highest valid result
}
bool rounding = true;
float dround(float val, int32_t digits) { return (rounding) ? (std::round(val * std::pow (10, digits)) / std::pow (10, digits)) : val; }

// pin operations
void set_pin(int pin, int mode) { if (pin >= 0 && pin != 255) pinMode (pin, mode); }
void write_pin(int pin, int val) {  if (pin >= 0 && pin != 255) digitalWrite (pin, val); }
void set_pin(int pin, int mode, int val) { set_pin(pin, mode); write_pin(pin, val); }
int32_t read_pin(int32_t pin) { return (pin >= 0 && pin != 255) ? digitalRead (pin) : -1; }

float convert_units(float from_units, float convert_factor, bool invert, float in_offset = 0.0, float out_offset = 0.0) {
    if (!invert) return out_offset + convert_factor * (from_units - in_offset);
    if (from_units - in_offset) return out_offset + convert_factor / (from_units - in_offset);
    printf ("convert_units refused to divide by zero: %lf, %lf, %d, %lf, %lf", from_units, convert_factor, invert, in_offset, out_offset);
    return -1;
}
// Exponential Moving Average filter : smooth out noise on inputs. 0 < alpha < 1 where lower = smoother and higher = more responsive
// pass in a fresh raw value, address of filtered value, and alpha factor, filtered value will get updated
float ema_filt(float _raw, float _filt, float _alpha) {
    return (_alpha * _raw) + ((1 - _alpha) * _filt);
}
template<typename RAW_T, typename FILT_T>
void ema_filt(RAW_T _raw, FILT_T* _filt, float _alpha) {
    float _raw_f = static_cast<float>(_raw);
    float _filt_f = static_cast<float>(*_filt);
    *_filt = static_cast<FILT_T>(ema_filt(_raw_f, _filt_f, _alpha));
}
// functions for changing values while respecting min and max constraints. used in tuner ui
template <typename T>
T adj_val(T variable, T modify, T low_limit, T high_limit) {
    T oldval = variable;
    if (std::is_same<T, int32_t>::value) variable += (T)modify;
    else if (std::is_same<T, float>::value) variable += (T)(modify * tuning_rate_pcps * (high_limit-low_limit) * loop_avg_us / (1000000 * 100.0));
    return variable < low_limit ? low_limit : (variable > high_limit ? high_limit : variable);
}
bool adj_val(int32_t *variable, int32_t modify, int32_t low_limit, int32_t high_limit) { // sets an int reference to new val constrained to given range
    int32_t oldval = *variable;
    *variable = adj_val(*variable, modify, low_limit, high_limit);
    return (*variable != oldval);
}
bool adj_val(float *variable, float modify, float low_limit, float high_limit) { // sets an int reference to new val constrained to given range
    float oldval = *variable;
    *variable = adj_val(*variable, modify, low_limit, high_limit);
    return (*variable != oldval);
}
bool adj_bool(bool val, int32_t delta) { return delta != 0 ? delta > 0 : val; } // returns 1 on delta=1, 0 on delta=-1, or val on delta=0
void adj_bool(bool *val, int32_t delta) { *val = adj_bool(*val, delta); }       // sets a bool reference to 1 on 1 delta or 0 on -1 delta

// // hue: 0,255 = red, 85 = grn, 170 = blu | sat: 0 = saturated up to greyscale, 255 = pure color | bright: 0 = blk, 255 = "full" | bright_flat: if =1, "full" brightness varies w/ hue for consistent luminance, otherwise "full" always ranges to 255 (mixed-element colors are brighter) | blu_boost: adds blu_boost/255 desaturation as a ratio of blu dominance
// template <typename T>
// T hsv_to_rgb_old(uint8_t hue, uint8_t sat = 255, uint8_t bright = 255, bool bright_flat = 1, uint8_t blu_boost = 0) {  // returns uint32 color in format 0x00RRGGBB
//     uint32_t rgb[3] = { 255 - 3 * (uint32_t)((255 - hue) % 85), 0, 3 * (uint32_t)((255 - hue) % 85) };
//     float maxc = (float)((rgb[0] > rgb[2]) ? rgb[0] : rgb[2]);
//     if (hue <= 85) { rgb[1] = rgb[0]; rgb[0] = rgb[2]; rgb[2] = 0; }
//     else if (hue <= 170) { rgb[1] = rgb[2]; rgb[2] = rgb[0]; rgb[0] = 0; }
//     float brightener = (float)bright / (bright_flat ? 255.0 : maxc);
//     float blu_booster = 1 + (float)(blu_boost * rgb[2]) / (float)(255.0 * (rgb[0] + rgb[1] + rgb[2]));
//     for (int led=0; led<=2; led++) 
//         rgb[led] = brightener * ((float)rgb[led] + blu_booster * (255.0 - sat) * (float)(maxc - rgb[led]) / 255.0);
//     if (std::is_same<T, uint16_t>::value) return (T)((rgb[0] & 0xf8) << 8) | ((rgb[1] & 0xfc) << 5) | (rgb[2] >> 3);
//     else if (std::is_same<T, uint32_t>::value) return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
// }
template <typename T>
T hsv_to_rgb(uint16_t hue, uint8_t sat = 255, uint8_t val = 255) {
    uint8_t rgb[3];  // [r,g,b];
    hue = (hue * 1530L + 32768) / 65536;
    if (hue < 510) { // Red to Green-1
        rgb[2] = 0;
        if (hue < 255) { rgb[0] = 255; rgb[1] = hue; }  //   Red to Yellow-1, g = 0 to 254
        else { rgb[0] = 510 - hue; rgb[1] = 255; }  //  Yellow to Green-1, r = 255 to 1
    }
    else if (hue < 1020) { // Green to Blue-1
        rgb[0] = 0;
        if (hue < 765) { rgb[1] = 255; rgb[2] = hue - 510; }  //  Green to Cyan-1, b = 0 to 254
        else { rgb[1] = 1020 - hue; rgb[2] = 255; }  // Cyan to Blue-1, g = 255 to 1
    }
    else if (hue < 1530) {  // Blue to Red-1
        rgb[1] = 0;
        if (hue < 1275) { rgb[0] = hue - 1020; rgb[2] = 255; }  // Blue to Magenta-1, r = 0 to 254
        else { rgb[0] = 255; rgb[2] = 1530 - hue; }  //   Magenta to Red-1, b = 255 to 1
    }
    else { rgb[0] = 255; rgb[1] = rgb[2] = 0; }  // Last 0.5 Red (quicker than % operator)
    uint32_t v1 = 1 + val;  // 1 to 256; allows >>8 instead of /255
    uint16_t s1 = 1 + sat;  // 1 to 256; same reason
    uint8_t s2 = 255 - sat; // 255 to 0
    uint16_t out[3];
    for (int led=0; led<3; led++) out[led] = (((((rgb[led] * s1) >> 8) + s2) * v1) & 0xff00) >> 8;
    // if (fake_color332) {
    //     if (std::is_same<T, uint16_t>::value) return (T)((out[0] & 0xe0) << 8) | ((out[1] & 0xe0) << 5) | ((out[2] & 0xc0) >> 3);
    // }
    if (std::is_same<T, uint16_t>::value) return (T)((out[0] & 0xf8) << 8) | ((out[1] & 0xfc) << 5) | (out[2] >> 3);
    else if (std::is_same<T, uint8_t>::value) return (T)((out[0] & 0xe0) | ((out[1] & 0xe0) >> 3) | ((out[2] & 0xc0) >> 6));
    else if (std::is_same<T, uint32_t>::value) return (T)((out[0] << 16) | (out[1] << 8) | out[2]);
}
// template <typename T>
// T hsv_to_rgb(uint8_t hue, uint8_t sat, uint8_t val) {
//     return hsv_to_rgb((uint16_t)hue << 8, sat, val);
// }
uint8_t rando_color() {
    return ((uint8_t)random(0x7) << 5) | ((uint8_t)random(0x7) << 2) | (uint8_t)random(0x3); 
}
// uint16_t rando_color() {
//     return ((uint16_t)random(0x1f) << 11) | ((uint16_t)random(0x3f) << 5) | (uint16_t)random(0x1f); 
// }
// uint16_t hue_shift(uint16_t orig, int addme) {
//     return (orig >> 11(uint16_t)random(0x1f) << 11) | ((uint16_t)random(0x3f) << 5) | (uint16_t)random(0x1f); 
// }

class Timer {  // !!! beware, this 54-bit microsecond timer overflows after every 571 years
  protected:
    volatile int64_t start_us, timeout_us;
  public:
    Timer() { reset(); }
    Timer(uint32_t arg_timeout_us) { set ((int64_t)arg_timeout_us); }
    void IRAM_ATTR set (int64_t arg_timeout_us) {
        timeout_us = arg_timeout_us;
        start_us = esp_timer_get_time();
    }
    void IRAM_ATTR reset() { start_us = esp_timer_get_time(); }
    bool IRAM_ATTR expired() { return esp_timer_get_time() >= start_us + timeout_us; }
    int64_t IRAM_ATTR elapsed() { return esp_timer_get_time() - start_us; }
    int64_t timeout() { return timeout_us; }
    bool IRAM_ATTR expireset() {  // Like expired() but immediately resets if expired
        int64_t now_us = esp_timer_get_time();
        if (now_us < start_us + timeout_us) return false;
        start_us = now_us;
        return true;
    }    
};
class AbsTimer {  // absolute timer ensures consecutive timeouts happen on regular intervals
  protected:
    volatile int64_t end, timeout_us;
  public:
    AbsTimer() { reset(); }
    AbsTimer(uint32_t arg_timeout) { set ((int64_t)arg_timeout); }
    void IRAM_ATTR set (int64_t arg_timeout) {
        timeout_us = arg_timeout;
        end = esp_timer_get_time() + timeout_us;
    }
    void IRAM_ATTR set() { end = esp_timer_get_time() + timeout_us; }  // use to rezero the timer phase
    void IRAM_ATTR reset() {  // move expiration to the next timeout multiple
        int64_t now = esp_timer_get_time();
        if (now >= end) end += timeout_us * (1 + (now - end) / timeout_us);
    }
    bool IRAM_ATTR expired() { return esp_timer_get_time() >= end; }
    int64_t IRAM_ATTR elapsed() { return esp_timer_get_time() + timeout_us - end; }
    int64_t timeout() { return timeout_us; }    
    // never finished writing this ...
    //
    // uint32_t IRAM_ATTR expireset() {  // Like expired() but immediately resets if expired
    //     int64_t now_us = esp_timer_get_time();
    //     if (now >= end) 
    //     end += timeout_us * (1 + (now - end) / timeout_us);
    //     if (now_us < start_us + timeout_us) return false;
    //     start_us = now_us;
    //     return true;
    // }
};
Timer sleep_inactivity_timer(300000000);
Timer starterTimer(5000000);  // If remotely-started starting event is left on for this long, end it automatically
Timer pushBrakeTimer(2500000);  // time allowed to push the brake pedal before remotely powering the starter motor
Timer panicTimer(15000000);  // How long should a panic stop last?  we can't stay mad forever

void kick_inactivity_timer(int source=0) {
    sleep_inactivity_timer.reset();  // evidence of user activity
    // Serial.printf("kick%d ", source);
}
