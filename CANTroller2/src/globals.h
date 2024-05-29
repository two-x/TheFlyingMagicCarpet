// globals.h - not dependent on anything, so include this first
#pragma once
#include "Arduino.h"
// pin assignments  ESP32-S3-DevkitC series   (note: "*" are pins we can reclaim if needed)
#define     boot_sw_pin  0 // button0/strap1   // input, the esp "boot" button. if more pins are needed, move encoder_sw_pin to this pin, no other signal of ours can work on this pin due to high-at-boot requirement
#define      tft_dc_pin  1 // adc1.0           // output, assert when sending data to display chip to indicate commands vs. screen data
#define  tp_cs_fuel_pin  2 // adc1.1           // output, this controls touch panel chip select for resistive touchscreen (dev board) OR drives the vehicle's fuel pump (on car)
#define   sdcard_cs_pin  3 // adc1.2/strapX  * // output, chip select for SD card controller on SPI bus. sdcard functionality is not implemented, we may not even need it
#define    mulebatt_pin  4 // adc1.3           // analog input, mule battery voltage sense, full scale is 16V
#define         pot_pin  5 // adc1.4           // analog in from 20k pot
#define   brake_pos_pin  6 // adc1.5           // analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#define    pressure_pin  7 // adc1.6           // analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
#define     i2c_sda_pin  8 // sda0/adc1.7      // i2c bus for airspeed/map sensors, lighting board, cap touchscreen
#define     i2c_scl_pin  9 // qhd0/scl0/adc1.8 // i2c bus for airspeed/map sensors, lighting board, cap touchscreen
#define      tft_cs_pin 10 // cs0/adc1.9     * // output, active low, chip select allows ILI9341 display chip use of the spi bus. pin can be reclaimed if tft is the only spi device (lose the sd card)
#define    spi_mosi_pin 11 // mosi0/adc2.0     // used as spi interface data for tft screen, sd card and resistive touch panel
#define    spi_sclk_pin 12 // sclk0/adc2.1     // used as spi interface clock for tft screen, sd card and resistive touch panel
#define    spi_miso_pin 13 // miso0/adc2.2     // used as spi interface data from sd card, resistive touch panel, and possibly (?) tft screen
#define hotrc_ch2_v_pin 14 // qwp0/pwm0/adc2.3 // hotrc ch2 bidirectional trigger input
#define hotrc_ch1_h_pin 15 // pwm1/adc2.4      // hotrc ch1 thumb joystick input
#define     gas_pwm_pin 16 // pwm1/adc2.5      // output, pwm signal duty cycle controls throttle target. on Due this is the pin labeled DAC1 (where A13 is on Mega)
#define   brake_pwm_pin 17 // pwm0/adc2.6/tx1  // output, pwm signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped)
#define   steer_pwm_pin 18 // pwm0/adc2.7/rx1  // output, pwm signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). jaguar asks for an added 150ohm series R when high is 3.3V
#define steer_enc_a_pin 19 // usb-d-/adc2.8  * // reserved for usb or a steering quadrature encoder.
#define steer_enc_b_pin 20 // usb-d+/adc2.9  * // reserved for usb or a steering quadrature encoder.
#define     onewire_pin 21 // pwm0             // onewire bus for temperature sensor data. note: tested this does not work on higher-numbered pins (~35+)
#define      speedo_pin 35 // spiram/octspi    // int input, active high, asserted when magnet south is in range of sensor. 1 pulse per driven pulley rotation. (Open collector sensors need pullup)
#define     starter_pin 36 // sram/ospi/glitch // input/Output (both active high), output when starter is being driven, otherwise input senses external starter activation
#define        tach_pin 37 // spiram/octspi    // int Input, active high, asserted when magnet south is in range of sensor. 1 pulse per engine rotation. (no pullup) - note: placed on p36 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on
#define  encoder_sw_pin 38 // spiram/octspi  * // input, Rotary encoder push switch, for the UI. active low (needs pullup). signal can be moved to pin 0 to free up this pin. Pin 38 is the neopixel pin on v1.1 boards
#define     basicsw_pin 39 // jtck/glitch      // input, asserted to tell us to run in basic mode, active low (has ext pullup) - note: placed on p39 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
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

// these global enums are super convenient, just don't change any values without checking that everywhere it's used won't get broken
enum hotrc_axis { HORZ=0, VERT=1, CH3=2, CH4=3 };
enum hotrc_val { OPMIN=0, CENT=1, OPMAX=2, RAW=3, FILT=4, DBBOT=5, DBTOP=6 };
enum motor_val { PARKED=1, OUT=3, GOVERN=4 , ABSMIN=5, ABSMAX=6, MARGIN=7, NUM_MOTORVALS=8 }; // IDLE=8, NUM_MOTORVALS=9 };
enum stop_val { STOP=1 };
enum steer_val { SAFE=1 };
enum size_enums { NUM_AXES=2, NUM_CHANS=4, NUM_VALUS=8 };
enum joydirs { JOY_RT=-2, JOY_DN=-1, JOY_CENT=0, JOY_UP=1, JOY_LT=2, JOY_PLUS=3, JOY_MINUS=4 };
enum runmode { BASIC=0, LOWPOWER=1, STANDBY=2, STALL=3, HOLD=4, FLY=5, CRUISE=6, CAL=7, NUM_RUNMODES=8 };
enum req { REQ_NA=-1, REQ_OFF=0, REQ_ON=1, REQ_TOG=2 };  // requesting handler actions of digital values with handler functions
enum cruise_modes { SuspendFly=0, TriggerPull=1, TriggerHold=2, NumCruiseSchemes=3 };
enum sw_presses { swNONE=0, swSHORT=1, swLONG=2 };
enum motor_modes { NA=0, Halt=1, Idle=2, Release=3, OpenLoop=4, ThreshLoop=5, ActivePID=6, AutoStop=7, AutoHold=8, ParkMotor=9, Cruise=10, Calibrate=11, Starting=12, NumMotorModes=13 };
enum brakefeedbacks { PositionFB=0, PressureFB=1, HybridFB=2, NoneFB=3, NumBrakeFB=4 };
enum brakeextra { NumBrakeSens=2 };
enum tunerstuff { ERASE=-1, OFF=0, SELECT=1, EDIT=2 };
enum boolean_states { ON=1 };
enum datapages { PG_RUN=0, PG_JOY=1, PG_SENS=2, PG_PULS=3, PG_PWMS=4, PG_IDLE=5, PG_MOTR=6, PG_BPID=7, PG_GPID=8, PG_CPID=9, PG_TEMP=10, PG_SIM=11, PG_UI=12, NUM_DATAPAGES=13 };
enum temp_categories { AMBIENT=0, ENGINE=1, WHEEL=2, BRAKE=3, NUM_TEMP_CATEGORIES=4 };  // 
enum temp_lims { DISP_MIN=1, WARNING=3, ALARM=4, DISP_MAX=5 };      // possible sources of gas, brake, steering commands
enum ui_modes { MuleChassisUI=0, DiagConsoleUI=1, ScreensaverUI=2, NumContextsUI=3 };  // uses for the multi purpose panel
enum codestatus { Confused=0, Booting=1, Parked=2, Stopped=3, Driving=4, NumCodeStatuses=5 };
enum err_type { LOST=0, RANGE=1, WARN=2, NUM_ERR_TYPES=3 };  // VALUE=2, STATE=3, WARN=4, CRIT=5, INFO=6, 
enum telemetry_idiots {                              // list of transducers which have onscreen idiotlights showing status
    _Hybrid=-3, _None=-2, _NA=-1,                    // these meta values indicate no transducer, useful for some contexts  
    _GasServo=0, _BrakeMotor=1, _SteerMotor=2,       // these transducers are actuators, driven by us
    _Speedo=3, _Tach=4, _BrakePres=5, _BrakePosn=6,  // these transducers are sensors, we read from
    _HotRC=7, _Temps=8, _Other=9, _GPIO=10,          // these are actually groups of multiple sensors (see below)
    NumTelemetryIdiots=11,                           // size of the list of values with idiot lights
};                        
enum telemetry_full {                                                                                 // complete list expanding sensor groups
    _HotRCHorz=11, _HotRCVert=12, _HotRCCh3=13, _HotRCCh4=14,                                         // _HotRC sensor group
    _MuleBatt=15, _AirVelo=16, _MAP=17, _Pot=18,                                                      // _Other sensor group
    _TempEng=19, _TempWhFL=20, _TempWhFR=21, _TempWhRL=22, _TempWhRR=23, _TempBrake=24, _TempAmb=25,  // _Temps sensor group
    _Ignition=26, _Starter=27, _BasicSw=28, _FuelPump=29,                                             // _GPIO signal group (with simple boolean values)
    NumTelemetryFull=30,                                                                              // size of both telemetry lists combined
};

// global configuration settings
bool autostop_disabled = false;      // temporary measure to keep brake behaving until we get it debugged. Eventually should be false
bool allow_rolling_start = true;     // are we lenient that it's ok to go to fly mode if the car is already moving? may be a smart prerequisite, may be us putting obstacles in our way
bool flip_the_screen = false;        // did you mount your screen upside-down?
bool cruise_speed_lowerable = true;  // allows use of trigger to adjust cruise speed target without leaving cruise mode.  Otherwise cruise button is a "lock" button, and trigger activity cancels lock
bool display_enabled = true;         // should we run 325x slower in order to get bombarded with tiny numbers?  Probably.
bool use_i2c_baton = true;           // use soren's custom homemade semaphores to prevent i2c bus collisions?
bool always_max_refresh = false;     // set to true to enforce a cap on screen frame draws (90 Hz I think it is), otherwise craw as fast as we can. fullscreen screensaver ignores this
bool brake_before_starting = true;   // if true, the starter motor pushes the brake pedal first, and won't turn on until it senses the pressure
bool watchdog_enabled = false;       // enable the esp's built-in watchdog circuit, it will reset us if it doesn't get pet often enough (to prevent infinite hangs). disabled cuz it seems to mess with the hotrc (?)
bool fuelpump_supported = true;      // do we drive power to vehicle fuel pump?  note if resistive touchscreen is present then fuelpump is automatically not supported regardless of this
int throttle_ctrl_mode = OpenLoop;   // should gas servo use the rpm-sensing pid? values: ActivePID or OpenLoop
bool print_task_stack_usage = true;  // enable to have remaining heap size and free task memory printed to console every so often. for tuning memory allocation
bool autosaver_display_fps = true;   // do you want to see the fps performance of the fullscreen saver in the corner?
bool crash_driving_recovery = true;  // if code crashes while driving, should it continue driving after reboot?
bool pot_tuner_acceleration = true;  // when editing values, can we use the pot to control acceleration of value changes? (assuming we aren't pot mapping some sensor at the time)
// dev-board-only options:  Note these are ignored and set false at boot by set_board_defaults() unless running on a breadboard with a 22k-ohm pullup to 3.3V the TX pin
bool dont_take_temperatures = false; // disables temp sensors. in case debugging dallas sensors or causing problems
bool console_enabled = true;         // completely disables the console serial output. idea being, it may be safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
bool keep_system_powered = false;    // equivalent to syspower always being high.
bool looptime_print = false;         // makes code write out timestamps throughout loop to serial port. for analyzing what parts of the code take the most time
bool touch_reticles = true;          // draws tiny little plus reticles to aim at for doing touchscreen calibration
bool button_test_heartbeat_color = false; // makes boot button short press change heartbeat color. useful for testing code on bare esp
bool wifi_client_mode = false;       // should wifi be in client or access point mode?
bool screensaver_enabled = true;     // does fullscreen screensaver start automatically when in powerdown, after a delay?
bool print_framebuffers = false;     // dumps out ascii representations of screen buffer contents to console. for debugging frame buffers. *hella* slow

// global tunable variables
float float_zero = 0.000069;         // if two floats being compared are closer than this, we consider them equal
int sprite_color_depth = 8;
uint32_t looptime_linefeed_threshold = 0;   // when looptime_print == 1, will linefeed after printing loops taking > this value. Set to 0 linefeeds all prints
float flycruise_vert_margin_pc = 0.3;       // Margin of error for determining hard brake value for dropping out of cruise mode
int32_t cruise_delta_max_pc_per_s = 16;  // (in TriggerHold mode) What's the fastest rate cruise adjustment can change pulse width (in us per second)
float cruise_angle_attenuator = 0.016;   // (in TriggerPull mode) Limits the change of each adjust trigger pull to this fraction of what's possible
float temp_lims_f[NUM_TEMP_CATEGORIES][6]{
    {45.0, 0.0, 115.0, 120.0, 130.0, 220.0},  // [AMBIENT] [OPMIN/DISP_MIN/OPMAX/WARNING/ALARM]
    {178.0, 0.0, 198.0, 202.0, 205.0, 220.0}, // [ENGINE] [OPMIN/DISP_MIN/OPMAX/WARNING/ALARM]
    {50.0, 0.0, 120.0, 130.0, 140.0, 220.0},  // [WHEEL] [OPMIN/DISP_MIN/OPMAX/WARNING/ALARM] (applies to all wheels)
    {45.0, 0.0, 120.0, 130.0, 140.0, 220.0},  // [BRAKE] [OPMIN/DISP_MIN/OPMAX/WARNING/ALARM]
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
uint32_t codestatus = Booting;
bool running_on_devboard = false;       // will overwrite with value read thru pull resistor on tx pin at boot
bool fun_flag = false;                  // since now using temp sensor address to detect vehicle, our tx resistor can be used for who knows what else!
bool standby_incomplete = true;        // minor state variable for standby mode - standby mode has not completed its work and can't yet stop activity
bool parking = false;                   // indicates in process of parking the brake & gas motors so the pedals can be used manually without interference
bool releasing = false;                 // indicates in process of releasing the brake to the zero brake point
bool cruise_adjusting = false;
bool cal_brakemode = false;             // allows direct control of brake motor using controller vert
bool cal_brakemode_request = false;     // allows direct control of brake motor using controller vert
bool cal_gasmode = false;               // allows direct control of gas servo using pot. First requires pot to be in valid position before mode is entered
bool cal_gasmode_request = false;
bool car_hasnt_moved = false;           // minor state variable for fly mode - Whether car has moved at all since entering fly mode
bool powering_up = false;               // minor state variable for lowpower mode
bool calmode_request = false;
bool flycruise_toggle_request = false;
// bool screensaver = false;                // can enable experiment with animated screen draws
bool basicmode_request = false;
int autosaver_request = REQ_NA;
int tunctrl = OFF, tunctrl_last = OFF;
int datapage = PG_RUN;                  // which of the dataset pages is currently displayed and available to edit?
volatile int sel_val = 0;               // in the real time tuning UI, which of the editable values is selected. -1 for none 
volatile int sel_val_last = 0;          
volatile int sel_val_last_last = 0;          
bool syspower = HIGH;                   // set by handler only. Reflects current state of the signal
// bool basicmodesw = LOW;
int sleep_request = REQ_NA;
float maf_gps = 0;                      // manifold mass airflow in grams per second
uint16_t heartbeat_override_color = 0x0000;
bool nowtouch = false;
bool captouch = true;
float loop_avg_us;
bool sensidiots[11];
bool web_disabled = false;
int ui_context = DiagConsoleUI;
int ui_default = DiagConsoleUI;
bool panicstop = false;
// bool sensor_present[telemetry_full];

// fast macros
#define arraysize(x) ((int32_t)(sizeof(x) / sizeof((x)[0])))  // a macro function to determine the length of string arrays
#undef constrain
inline float constrain(float amt, float low, float high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline int32_t constrain(int32_t amt, int32_t low, int32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline uint32_t constrain(uint32_t amt, uint32_t low, uint32_t high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
inline long constrain(long amt, long low, long high) { return (amt < low) ? low : ((amt > high) ? high : amt); }
#undef map
inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // instead of dividing by zero, return the highest valid result
}
inline int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // instead of dividing by zero, return the highest valid result
}

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

template <typename T>
T hsv_to_rgb(uint16_t hue, uint8_t sat = 255, uint8_t val = 255) {
    uint8_t rgb[3] = { 0, 0, 0 };  // [r,g,b];
    hue = (hue * 1530L + 32768) / 65536;
    if (hue < 510) { // Red to Green-1
        if (hue < 255) { rgb[0] = 255; rgb[1] = hue; }  //   Red to Yellow-1, g = 0 to 254
        else { rgb[0] = 510 - hue; rgb[1] = 255; }  //  Yellow to Green-1, r = 255 to 1
    }
    else if (hue < 1020) { // Green to Blue-1
        if (hue < 765) { rgb[1] = 255; rgb[2] = hue - 510; }  //  Green to Cyan-1, b = 0 to 254
        else { rgb[1] = 1020 - hue; rgb[2] = 255; }  // Cyan to Blue-1, g = 255 to 1
    }
    else if (hue < 1530) {  // Blue to Red-1
        if (hue < 1275) { rgb[0] = hue - 1020; rgb[2] = 255; }  // Blue to Magenta-1, r = 0 to 254
        else { rgb[0] = 255; rgb[2] = 1530 - hue; }  //   Magenta to Red-1, b = 255 to 1
    }
    else { rgb[0] = 255; }  // Last 0.5 Red (quicker than % operator)
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

class Timer {  // !!! beware, this 54-bit microsecond timer overflows after every 571 years
  protected:
    volatile int64_t start_us, timeout_us;
  public:
    Timer() { reset(); }
    Timer(uint32_t arg_timeout_us) { set ((int64_t)arg_timeout_us); }
    void set (int64_t arg_timeout_us) {
        timeout_us = arg_timeout_us;
        start_us = esp_timer_get_time();
    }
    void reset() { start_us = esp_timer_get_time(); }
    bool expired() { return esp_timer_get_time() >= start_us + timeout_us; }
    int64_t elapsed() { return esp_timer_get_time() - start_us; }
    int64_t timeout() { return timeout_us; }
    bool expireset() {  // Like expired() but immediately resets if expired
        int64_t now_us = esp_timer_get_time();
        if (now_us < start_us + timeout_us) return false;
        start_us = now_us;
        return true;
    }    
};
Timer user_inactivity_timer;  // how long of not touching it before it goes to low power mode

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
const uint8_t DRED = 0x80;  // dark red
const uint8_t BORG = 0xe8;  // blood orange (very reddish orange)
const uint8_t SALM = 0xc9;  // muted salmon
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
const uint8_t ORCD = 0x8f;  // orchid (lighter and less saturated purple)
const uint8_t VIO  = 0x83;  // violet
const uint8_t PUR  = 0x63;  // purple
const uint8_t GPUR = 0x6a;  // a low saturation greyish pastel purple
const uint8_t LPUR = 0xb3;  // a light pastel purple
const uint8_t PNK  = 0xe3;  // pink is the best color
const uint8_t MPNK = 0xeb;  // we need all shades of pink
const uint8_t LPNK = 0xf3;  // especially light pink, the champagne of pinks
const uint8_t NON  = 0x45;  // used as default value when color is unspecified

// kick_inactivity_timer() function to call whenever human activity occurs, for accurate inactivity timeout feature
//   integer argument encodes which source of human activity has kicked the timer. Here are the codes:
//   0 = momentary button down (encoder sw or boot button)
//   1 = momentary button up (encoder sw or boot button)
//   2 = encoder turned
//   3 = web interactivity
//   4 = touchscreen touching
//   5 = hotrc button presses
//   6 = hotrc trigger or joystick off center
//   7 = pot movement
//   8 = toggle switch flipped (basic switch)
void kick_inactivity_timer(int source=0) {
    user_inactivity_timer.reset();  // evidence of user activity
    // Serial.printf("kick%d ", source);
}

// class AbsTimer {  // absolute timer ensures consecutive timeouts happen on regular intervals
//   protected:
//     volatile int64_t end, timeout_us;
//   public:
//     AbsTimer() { reset(); }
//     AbsTimer(uint32_t arg_timeout) { set ((int64_t)arg_timeout); }
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
//     // uint32_t IRAM_ATTR expireset() {  // Like expired() but immediately resets if expired
//     //     int64_t now_us = esp_timer_get_time();
//     //     if (now >= end) 
//     //     end += timeout_us * (1 + (now - end) / timeout_us);
//     //     if (now_us < start_us + timeout_us) return false;
//     //     start_us = now_us;
//     //     return true;
//     // }
// };