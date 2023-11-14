// Contains utility functions, classes, and defines
#pragma once
#include <Wire.h>
#include <cstdint> // for uint types
#include <cstdio> // for printf
#include "Arduino.h"
// pin assignments  ESP32-S3-DevkitC series
#define      button_pin  0 // button0/strap-1  // Input, Rotary encoder push switch, for the UI. active low (needs pullup). Also the esp "Boot" button does the same thing
#define    lipobatt_pin  1 // adc1ch0          // Analog input, LiPo cell voltage, full scale is 4.8V
#define   encoder_b_pin  2 // adc1ch1          // Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
#define      tft_dc_pin  3 // adc1ch2/strap-X  // Output, Assert when sending data to display chip to indicate commands vs. screen data - ! pin is also defined in tft_setup.h
#define    mulebatt_pin  4 // adc1ch3          // Analog input, mule battery voltage sense, full scale is 16V
#define    pot_wipe_pin  5 // adc1ch4          // Analog in from 20k pot
#define   brake_pos_pin  6 // adc1ch5          // Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#define    pressure_pin  7 // adc1ch6          // Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
#define     i2c_sda_pin  8 // i2c0sda/adc1ch7  // i2c bus for airspeed/map sensors, lighting board, cap touchscreen
#define     i2c_scl_pin  9 // i2c0scl/adc1ch8  // i2c bus for airspeed/map sensors, lighting board, cap touchscreen
#define      tft_cs_pin 10 // spi0cs/adc1ch9   // Output, active low, Chip select allows ILI9341 display chip use of the SPI bus - ! pin is also defined in tft_setup.h
#define    spi_mosi_pin 11 // spi0mosi/adc2ch0 // Used as spi interface data for touchscreen, sd card and tft screen - ! pin is also defined in tft_setup.h
#define    spi_sclk_pin 12 // spi0sclk/adc2ch1 // Used as spi interface clock for touchscreen, sd card and tft screen - ! pin is also defined in tft_setup.h
#define    spi_miso_pin 13 // spi0miso/adc2ch2 // Used as spi interface data from sd card and possibly (?) tft screen - ! pin is also defined in tft_setup.h
#define hotrc_ch2_v_pin 14 // pwm0/adc2ch3     // Hotrc Ch2 bidirectional trigger input
#define hotrc_ch1_h_pin 15 // pwm1/adc2ch4     // Hotrc Ch1 thumb joystick input
#define     gas_pwm_pin 16 // pwm1/adc2ch5     // Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
#define   brake_pwm_pin 17 // pwm0/adc2ch6/tx1 // Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped)
#define   steer_pwm_pin 18 // pwm0/adc2ch7/rx1 // Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
#define steer_enc_a_pin 19 // usb-d-/adc2ch8   // Reserved for a steering quadrature encoder. Encoder "A" signal
#define steer_enc_b_pin 20 // usb-d+/adc2ch9   // Reserved for a steering quadrature encoder. Encoder "B" signal
#define     onewire_pin 21 // pwm0             // Onewire bus for temperature sensor data. note: tested this does not work on higher-numbered pins (~35+)
#define      speedo_pin 35 // spiram/octspi    // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. (Open collector sensors need pullup)
#define     starter_pin 36 // sram/ospi/glitch // Input/Output (both active high), output when starter is being driven, otherwise input senses external starter activation
#define        tach_pin 37 // spiram/octspi    // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup) - Note: placed on p36 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
#define   sdcard_cs_pin 38 // spiram/octspi    // Output, chip select for SD card controller on SPI bus
#define basicmodesw_pin 39 // jtck/glitch      // Input, asserted to tell us to run in basic mode, active low (has ext pullup) - Note: placed on p39 because filtering should negate any effects of 80ns low pulse when certain rtc devices power on (see errata 3.11)
#define   hotrc_ch4_pin 40 // jtdo             // Syspower, starter, and cruise mode toggle control. Hotrc Ch4 PWM toggle signal
#define   hotrc_ch3_pin 41 // jtdi             // Ignition control, Hotrc Ch3 PWM toggle signal
#define   encoder_a_pin 42 // jtms             // Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
#define     uart_tx_pin 43 // "TX"/uart0tx     // Serial monitor data out. Also used to detect devboard vs. pcb at boot time (using pullup/pulldown, see below)
#define     uart_rx_pin 44 // "RX"/uart0rx     // Serial monitor data in. Maybe could repurpose during runtime since we only need outgoing console data?
#define    ignition_pin 45 // strap-0          // Output to an nfet/pfet pair to control the car ignition
#define    syspower_pin 46 // strap-0          // Output to an nfet/pfet pair to power all the tranducers.
#define    touch_cs_pin 47 // NA               // Output, chip select for resistive touchscreen, active low - ! pin is also defined in tft_setup.h
#define    neopixel_pin 48 // neopix           // Data line to onboard Neopixel WS281x (on all v1 devkit boards - pin 38 is used on v1.1 boards). Also used for onboard and external neopoxels - ! pin is also defined in neopixel.h
// External components needed (pullup/pulldown resistors, capacitors, etc.): (Note: "BB" = On dev breadboards only, "PCB" = On vehicle PCB only)
// 1. brake_pos_pin: Add 1M-ohm to GND. Allows detecting unconnected sensor or broken connection.
// 2. onewire_pin: Add 4.7k-ohm to 3.3V. Needed for open collector sensor output, to define logic-high voltage level.
// 3. tach_pin, speedo_pin: (PCB) Add 4.7k-ohm to 3.3V. For open collector sensor outputs. (BB) If no sensor is present: connect 4.7k-ohm to GND instead. Allows sensor detection.
// 4. neopixel_pin: (PCB) Add 330 ohm in series (between pin and the DataIn pin of the 1st pixel). (BB) Same, but this one is likely optional, e.g. mine works w/o it.  For signal integrity over long wires. 
// 5. uart_tx_pin: (PCB) Add 22k-ohm to GND. (BB) Connect the 22k-ohm to 3.3V instead. For boot detection of vehicle PCB, so defaults are set appropriately.
// 6. ADC inputs (mulebatt_pin, pressure_pin, brake_pos_pin, pot_wipe_pin) should have 100nF cap to gnd, tho it works w/o it.
// 7. encoder_a_pin, encoder_b_pin, button_pin: should have 10nF to gnd, tho it should work w/o it. Pullups to 3.3V (4.7k-ohm is good) are also necessary, but the encoder we're using includes these.
// 8. Resistor dividers are needed for these inputs: starter_pin (16V->3.3V), mulebatt_pin (16V->3.3V), and pressure_pin (5V->3.3V).
// 9. ignition_pin, syspower_pin, starter_pin: require pulldowns to gnd, this is provided by nfet gate pulldown.
// 10. gas_pwm_pin: should have a series ~680-ohm R going to the servo.

// Note onewire works on pins 19-21 but not on pins 39-42
// If one more pin becomes needed, encoder_sw may be moved to pin 0, freeing up pin 38 (pin 0 requires a pullup, which encoder_sw has)
// ESP32-S3 TRM: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf#dma
// ESP32-S3 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
// ESP32-S3 has 5 DMA channels in each direction. We would use them for SPI data out to TFT, Neopixel data out, and possibly out to the 3 motor outputs and in from the 4 hotrc channels.
// DMA works with: RMT, I2S0, I2S1, SPI2, SPI3, ADC, internal RAM, external PSRAM, and a few others (see the TRM)
// Official pin capabilities: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html?highlight=devkitc#user-guide-s3-devkitc-1-v1-1-header-blocks
// ADC ch2 will not work if wifi is enabled
// Bootstrap pins: Pin 0 must be pulled high, and pins 45 and 46 pulled low during bootup
// ESP32 errata 3.11: Pin 36 and 39 will be pulled low for ~80ns when "certain RTC peripherals power up"
// SPI bus page including DMA information: https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/spi_master.html
// BM2023 pins: onewire 19, hotrc_ch3_pin 20, hotrc_ch4_pin 21, tach_pin 36, ignition_pin 37, encoder_b_pin 40, encoder_a_pin 41, encoder_sw_pin 42

#define tft_ledk_pin -1   // Output, optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define touch_irq_pin 255 // Input, optional touch occurence interrupt signal (for resistive touchscreen, prevents spi bus delays) - Set to 255 if not used
#define tft_rst_pin -1    // TFT Reset allows us to reboot the screen hardware when it crashes. Otherwise connect screen reset line to esp reset pin

#define adcbits 12
#define adcrange_adc 4095     // = 2^adcbits-1
#define adcmidscale_adc 2047  // = 2^(adcbits-1)-1

// fast macros
#define arraysize(x) ((int32_t)(sizeof(x) / sizeof((x)[0])))  // A macro function to determine the length of string arrays
#define floor(amt, lim) ((amt <= lim) ? lim : amt)
#define ceiling(amt, lim) ((amt >= lim) ? lim : amt)
#undef min
#undef max
// these global enums are reeeeally convenient and make code very readable, but error-prone.
// we need to be careful when changing anything - best to keep all of this here in one place!
enum hotrc_axis : int { horz=0, vert=1, ch3=2, ch4=3 };
enum hotrc_val : int { opmin=0, cent=1, opmax=2, raw=3, filt=4, dbbot=5, dbtop=6, margin=7 };
enum motor_val : int { parked=1, out=3, govern=4 , absmin=5, absmax=6};
enum stop_val : int { stop=1 };
enum steer_val : int { safe=1 };
enum size_enums : int { num_axes=2, num_chans=4, num_motorvals=7, num_valus=8 };
enum joydirs : int { joy_rt=-2, joy_down=-1, joy_cent=0, joy_up=1, joy_lt=2 };
enum runmode : int { BASIC, ASLEEP, SHUTDOWN, STALL, HOLD, FLY, CRUISE, CAL, num_runmodes };
enum req : int { req_na=-1, req_off=0, req_on=1, req_tog=2 };  // requesting handler actions of digital values with handler functions
enum cruise_modes { pid_suspend_fly, throttle_angle, throttle_delta };
enum sw_presses { NONE, SHORT, LONG }; // used by encoder sw and button algorithms

// global configuration settings
bool starter_signal_support = true;
bool remote_start_support = true;
bool autostop_disabled = false;       // Temporary measure to keep brake behaving until we get it debugged. Eventually should be false
bool allow_rolling_start = false;    // May be a smart prerequisite, may be us putting obstacles in our way
bool flip_the_screen = true;
bool cruise_speed_lowerable = true;  // Allows use of trigger to adjust cruise speed target without leaving cruise mode.  Otherwise cruise button is a "lock" button, and trigger activity cancels lock
// Dev-board-only options:  Note these are ignored and set false at boot by set_board_defaults() unless running on a breadboard with a 22k-ohm pullup to 3.3V the TX pin
bool usb_jtag = true;  // If you will need the usb otg port for jtag debugging (see https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/jtag-debugging/configure-builtin-jtag.html)
bool dont_take_temperatures = false;  // In case debugging dallas sensors or causing problems
bool gamma_correct_enabled = false;
bool console_enabled = true;         // safer to disable because serial printing itself can easily cause new problems, and libraries might do it whenever
bool keep_system_powered = false;    // Use true during development
bool screensaver = false;  // Can enable experiment with animated screen draws
bool looptime_print = false;         // Makes code write out timestamps throughout loop to serial port
bool touch_reticles = false;

// global state variables.  maybe should belong to runmodemanager class
bool shutdown_incomplete = true;     // minor state variable for shutdown mode - Shutdown mode has not completed its work and can't yet stop activity
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool park_the_motors = false;        // Indicates we should release the brake & gas so the pedals can be used manually without interference
bool autostopping = false, autostopping_last = false;           // true when in process of stopping the car (hold or shutdown modes)
bool car_hasnt_moved = false;        // minor state variable for fly mode - Whether car has moved at all since entering fly mode
bool joy_centered = false;  // minor state variable for hold mode
bool powering_up = false;  // minor state variable for asleep mode
float flycruise_vert_margin_pc = 0.3; // Margin of error for determining hard brake value for dropping out of cruise mode
bool cal_joyvert_brkmotor_mode = false; // Allows direct control of brake motor using controller vert
bool cal_pot_gasservo_mode = false;     // Allows direct control of gas servo using pot. First requires pot to be in valid position before mode is entered
bool cal_pot_gasservo_ready = false;    // Whether pot is in valid range
bool running_on_devboard = false;    // will overwrite with value read thru pull resistor on tx pin at boot
bool flashdemo = false;
int sleep_request = req_na;
bool calmode_request = false;
bool flycruise_toggle_request = false;
int32_t idelta = 0, idelta_touch = 0, idelta_encoder = 0;

// Cruise : is active on demand while driving.
// Pick from 3 different styles of adjusting cruise setpoint. I prefer throttle_delta.
// pid_suspend_fly : (PID) Moving trigger from center disables pid and lets you adjust the rpm target directly like Fly mode does. Whatever speed you're at when trigger releases is new pid target  
// throttle_angle : Cruise locks throttle angle, instead of pid. Moving trigger from center adjusts setpoint proportional to how far you push it before releasing (and reduced by an attenuation factor)
// throttle_delta : Cruise locks throttle angle, instead of pid. Any non-center trigger position continuously adjusts setpoint proportional to how far it's pulled over time (up to a specified maximum rate)
int cruise_setpoint_mode = throttle_delta;
bool cruise_adjusting = false;
int32_t cruise_delta_max_pc_per_s = 16;  // (in throttle_delta mode) What's the fastest rate cruise adjustment can change pulse width (in us per second)
// float cruise_angle_attenuator = 0.25;  // (in throttle_angle mode) Limits the change of each adjust trigger pull to this fraction of what's possible
float cruise_angle_attenuator = 0.016;  // (in throttle_angle mode) Limits the change of each adjust trigger pull to this fraction of what's possible

bool starter = LOW;  // Set by handler only. Reflects current state of starter signal (does not indicate source)
bool starter_drive = false;  // Set by handler only. High when we're driving starter, otherwise starter is an input
req starter_request = req_na;
bool syspower = HIGH;  // Set by handler only. Reflects current state of the signal
bool ignition = LOW;  // Set by handler only. Reflects current state of the signal
req ignition_request = req_na;
bool panicstop = false;
req panicstop_request = req_on;  // On powerup we assume the code just crashed during a drive, because it could have
bool basicmodesw = LOW;

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

// the above should be templated, i'm sure
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
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
inline int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max - in_min) return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
    return out_max;  // Instead of dividing by zero, return the highest valid result
}
bool rounding = true;
float dround(float val, int32_t digits) { return (rounding) ? (std::round(val * std::pow (10, digits)) / std::pow (10, digits)) : val; }

// pin operations
void set_pin(int32_t pin, int32_t mode) { if (pin >= 0 && pin != 255) pinMode (pin, mode); }
void write_pin(int32_t pin, int32_t val) {  if (pin >= 0 && pin != 255) digitalWrite (pin, val); }
int32_t read_pin(int32_t pin) { return (pin >= 0 && pin != 255) ? digitalRead (pin) : -1; }

float convert_units(float from_units, float convert_factor, bool invert, float in_offset = 0.0, float out_offset = 0.0) {
    if (!invert) return out_offset + convert_factor * (from_units - in_offset);
    if (from_units - in_offset) return out_offset + convert_factor / (from_units - in_offset);
    printf ("convert_units refused to divide by zero: %lf, %lf, %d, %lf, %lf", from_units, convert_factor, invert, in_offset, out_offset);
    return -1;
}

// Exponential Moving Average filter : Smooth out noise on inputs. 0 < alpha < 1 where lower = smoother and higher = more responsive
// Pass in a fresh raw value, address of filtered value, and alpha factor, filtered value will get updated
float ema_filt(float raw, float filt, float alpha) {
    return (alpha * raw) + ((1 - alpha) * filt);
}
template<typename RAW_T, typename FILT_T>
void ema_filt(RAW_T raw, FILT_T* filt, float alpha) {
    float raw_f = static_cast<float>(raw);
    float filt_f = static_cast<float>(*filt);
    *filt = static_cast<FILT_T>(ema_filt(raw_f, filt_f, alpha));
}

template <typename T>
T adj_val(T variable, T modify, T low_limit, T high_limit) {
    T oldval = variable;
    variable += modify;
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

class Timer {  // !!! beware, this 54-bit microsecond timer overflows after every 571 years !!!
  protected:
    volatile int64_t start_us, timeout_us;
  public:
    Timer() { reset(); }
    Timer(int arg_timeout_us) { set ((int64_t)arg_timeout_us); }
    void IRAM_ATTR set (int64_t arg_timeout_us) {
        timeout_us = arg_timeout_us;
        start_us = esp_timer_get_time();
    }
    void IRAM_ATTR reset() { start_us = esp_timer_get_time(); }
    bool IRAM_ATTR expired() { return esp_timer_get_time() >= start_us + timeout_us; }
    bool IRAM_ATTR expireset() {  // Like expired() but immediately resets if expired
        int64_t now_us = esp_timer_get_time();
        if (now_us < start_us + timeout_us) return false;
        start_us = now_us;
        return true;
    }    
    int64_t IRAM_ATTR elapsed() { return esp_timer_get_time() - start_us; }
    int64_t IRAM_ATTR timeout() { return timeout_us; }
};
class I2C {
    private:
        int32_t _devicecount = 0;
        uint8_t _addrs[10];
        uint8_t _sda_pin, _scl_pin;
        Timer scanTimer;
    public:
        I2C(uint8_t sda_pin_arg, uint8_t scl_pin_arg) : _sda_pin(sda_pin_arg), _scl_pin(scl_pin_arg) {}

        void init() {
            printf("I2C driver ");
            scanTimer.reset();
            Wire.begin(_sda_pin, _scl_pin);  // I2c bus needed for airflow sensor
            byte error, address;
            printf(" scanning ...");
            _devicecount = 0;
            for (address = 1; address < 127; address++ ) {
                Wire.beginTransmission(address);
                error = Wire.endTransmission();
                if (error == 0) {
                    printf (" found addr: 0x%s%x", (address < 16) ? "0" : "", address);
                    _addrs[_devicecount++] = address;
                }
                else if (error==4) printf (" error addr: 0x%s%x", (address < 16) ? "0" : "", address);
            }
            if (scanTimer.elapsed() > 5000000) printf(" timeout & fail bus scan.");
            if (_devicecount == 0) printf(" no devices found.");
            printf(" done\n");
        }

        bool device_detected(uint8_t addr) {
            for (int32_t i=0; i < _devicecount; i++) {
                if (_addrs[i] == addr) return true;
            }
            return false;
        }
};