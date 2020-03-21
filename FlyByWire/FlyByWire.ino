// Carpet CANTroller II  Source Code  - For Arduino Mega 2560 with Adafruit 2.8inch Captouch TFT shield.

// Libraries to include.  Note all these have example code when installed into arduino ide
#include <Servo.h>   // Makes PWM output to control motors (for our gas and steering)
#include <FastPID.h>  // Fixed-point math based PID loop (for our brakes and maybe cruise control)
#include <SPI.h>   // SPI serial bus needed to talk to the LCD and the SD card
#include <Wire.h>      // Contains I2C serial bus, needed to talk to touchscreen chip
#include <Adafruit_ILI9341.h>   // For interfacing with the TFT LCD controller chip
#include <Adafruit_FT6206.h>   // For interfacing with the cap touchscreen controller chip
#include <SdFat.h>        // SD card & FAT filesystem library
#include <Adafruit_GFX.h>    // For drawing pictures & text on the screen
//#include <Adafruit_ImageReader.h>  // We don't really need thihs but can use it to test that SD flash is working
//#include "TouchScreen.h"  // Touchscreen library (I don't think we need it? Maybe for resistive TP we're not using)

/*
# Here are the different runmodes documented
#
# ** Shutdown Mode **
# - Required: Ignition Off
# - Priority: 1 (Highest)
# This mode is active whenever the ignition is off.  In other words, whenever the
# little red pushbutton switch by the joystick is unclicked.  This happens before the
# ignition is pressed before driving, but it also may happen if the driver needs to
# panic and E-stop due to loss of control or any other reason.  The ignition will get cut
# independent of the controller, but we can help stop the car faster by applying the
# brakes. Once car is stopped, we release all actuators and go to low power mode.
# - Actions: 1. Release throttle. If car is moving AND BasicMode Off, apply brakes to stop car
# - Actions: 2: Deactivate all actuators including steering
#
# ** Basic Mode **
# - Required: BasicMode switch On & Ignition On
# - Priority: 2
# The gas and brake don't do anything in Basic Mode. Just the steering works, so use the pedals.
# This mode is enabled with a toggle switch in the controller box.  When in Basic Mode, the only
# other valid mode is Shutdown Mode. Shutdown Mode may override Basic Mode.
# - Actions: Release and deactivate brake and gas actuators.  Steering PID keep active 
#
# ** Stall Mode **
# - Required: Engine stopped & BasicMode switch Off & Ignition On
# - Priority: 3
# This mode is active when the engine is not running.  If car is moving, then it presumably may
# coast to a stop.  The actuators are all enabled and work normally.  Starting the engine will 
# bring you into Hold Mode.  Shutdown Mode and Basic Mode both override Stall Mode. Note: This
# mode allows for driver to steer while being towed or pushed, or working on the car.
# - Actions: Enable all actuators
#
# ** Hold Mode **
# - Required: Engine running & JoyVert<=Center & BasicMode switch Off & Ignition On
# - Priority: 4
# This mode is entered from Stall Mode once engine is started, and also, whenever the car comes
# to a stop while driving around in Fly Mode.  This mode releases the throttle and will 
# continuously increase the brakes until the car is stopped, if it finds the car is moving. 
# Pushing up on the joystick from Hold mode releases the brakes & begins Fly Mode.
# Shutdown, Basic & Stall Modes override Hold Mode.
# # Actions: Close throttle, and Apply brake to stop car, continue to ensure it stays stopped.
#
# ** Fly Mode **
# - Required: (Car Moving OR JoyVert>Center) & In gear & Engine running & BasicMode Off & Ign On
# - Priority: 5
# This mode is for driving under manual control. In Fly Mode, vertical joystick positions
# result in a proportional level of gas or brake (AKA "Manual" control).  Fly Mode is
# only active when the car is moving - Once stopped or taken out of gear, we go back to Hold Mode.
# If the driver performs a special secret "cruise gesture" on the joystick, then go to Cruise Mode.
# Special cruise gesture might be: Pair of sudden full-throttle motions in rapid succession
# - Actions: Enable all actuators, Watch for gesture
#
# ** Cruise Mode **
# - Required: Car Moving & In gear & Engine running & BasicMode switch Off & Ignition On
# - Priority: 6 (Lowest)
# This mode is entered from Fly Mode by doing a special joystick gesture. In Cruise Mode,
# the brake is disabled, and the joystick vertical is different: If joyv at center, the
# throttle will actively maintain current car speed.  Up or down momentary joystick presses
# serve to adjust that target speed. A sharp, full-downward gesture will drop us back to 
# Fly Mode, promptly resulting in braking (if kept held down).
# - Actions: Release brake, Maintain car speed, Handle joyvert differently, Watch for gesture
*/

// Some human readable integers for array indexing
#define STR 0   // indexer for steering values
#define BRK 1   // indexer for brake values
#define GAS 2   // indexer for throttle values
#define H 0   // indexer for horizontal values
#define V 1   // indexer for vertical values
#define BOOT -1
#define SHUTDOWN 0
#define BASIC 1
#define STALL 2
#define HOLD 3
#define FLY 4
#define CRUISE 5

#define arraysize(x)  (sizeof(x) / sizeof((x)[0]))  // To determine the length of string arrays


// LCD is 2.8in diagonal, 240x320 pixels
// LCD supports 18-bit color, but GFX library uses 16-bit color, organized (MSB) 5b-red, 6b-green, 5b-blue (LSB)
// Since the RGB don't line up with the nibble boundaries, it's tricky to quantify a color, here are some colors:
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

// void drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
//void setCursor(uint16_t x0, uint16_t y0);
//void setTextColor(uint16_t color);
//void setTextColor(uint16_t color, uint16_t backgroundcolor);
//void setTextSize(uint8_t size);
//void setTextWrap(boolean w);

// Defines for all the GPIO pins we're using
#define enc_b_pin 2  // Int input, Encoder is user input knob for the UI.  This is its B quadrature output, active low (needs pullup)
#define enc_a_pin 3  // Int input, Encoder is user input knob for the UI.  This is its A quadrature output, active low (needs pullup)
#define usd_cs_pin 4  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
#define tft_ledk_pin 5   // Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
#define brake_pwm_pin 6  // Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
#define tp_irq_pin 7  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
#define steer_pwm_pin 8  // Output, PWM signal duty cycle sets speed of steering motor from full speed left, to full speed right, (50% is stopped)
#define tft_dc_pin 9  // Output, Assert when sending data to display chip to indicate commands vs. screen data
#define tft_cs_pin 10  // Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
#define gas_pwm_pin 13 // Output, PWM signal duty cycle controls throttle target
#define ignition_pin 15  // Input tells us if ignition signal is on or off, active high (no pullup)
#define neutral_pin 16  // Input, active low, is asserted when car is in neutral, i.e. out of gear.  (needs pullup)
#define basicmodesw_pin 17  // Input, active low, asserted to tell us to run in basic mode.   (needs pullup)
#define tach_pulse_pin 18  // Int Input, active high, asserted when magnet is in range of sensor. 4 pulses per engine rotation. (no pullup)
#define speedo_pulse_pin 19  // Int Input, active high, asserted when magnet is in range of sensor. 8 pulses per driven pulley rotation. (no pullup)
#define enc_sw_pin 20  // Int input, Encoder is user input knob for the UI.  This is its pushbutton output, active low (needs pullup)
#define joy_horz_pin A0  // Analog input, tells us up-down position of joystick
#define joy_vert_pin A1  // Analog input, tells us left-right position of joystick
#define brake_pres_pin A2  // Analog input, tells us brake fluid pressure (5V = 1000psi)
#define brake_pos_pin A6  // Analog input, tells us linear position of brake actuator 
#define mule_volt_pin A7  // Analog input, mule battery voltage level, full scale is 5V * 3.1276 = 15.638V

// Readily available possibilities we could wire up if we want
//
// * Status LEDs (digital out)
// * UI Potentiometer (analog in)
// * Control of steering motor coast vs. brake
// * Control of brake motor coast vs. brake
// * CAN bus as a superior interface to brake and steering Jaguars (only on Due I think?)
// * Steering limit switches left and right, handle here instead of in Jaguar (digital in)
// * Engine temperature module overheat panic input (digital in)
// * Remote E-Stop panic inputs (digital in)
// * Serial interface to the lighting controller (if we can think of a reason)
// * Mule starter (digital out)
// * E-brake handle position (digital in)

#define disp_lineheight 15
#define disp_linewidth 320
#define disp_screenheight 240

#define adcrange 1024    // = 1024
#define adcmidscale 512    // = 1024
//int adcrange = 2^10;    // = 1024
//int adcmidscale = adcrange/2;    // = 1024

// Constants
//
char telemetry[16][14] = {  
    "   Run Mode: ",
    "  Air Speed: ",
    "   Joy Horz: ",
    "   Joy Vert: ",
    "SteerTarget: ",
    "  Steer PWM: ",
    "BrakeTarget: ",
    "  Brake PWM: ",
    "BrakePressr: ",   
    "  GasTarget: ",
    "    Gas PWM: ",
    " Engine RPM: ",
    "    Touch X: ",
    "    Touch Y: ",
    "JoyHorz Raw: ",
    "         NA: "
};
char modecard[6][9] = {
    "Shutdown",
    "Basic   ",
    "Stall   ",
    "Hold    ",
    "Fly     ",
    "Cruise  "
};
/*
char actioncard[6][9] = {
    "Unknown ",
    "Stopping",
    "New Mode",
    "Flying  ",
    "Cruising",
    "Releasng",    
    "Holding "
};
*/

// Settable calibration values and control parameters
//
float steer_kp = 0.0;  // PID proportional coefficient (steering)
float steer_ki = 0.0;  // PID integral coefficient (steering)
float steer_kd = 0.0;  // PID derivative coefficient (steering)
int steer_pid_freq = 10;  // How fast to loop in Hz
int steer_stop = 90;  // Servo target corresponding to no steering (zero steering speed)
int steer_max_speed = 90;  // Fastest steering we want.  (in change of PWM duty from center)
float brake_kp = 0.1;  // PID proportional coefficient (brake)
float brake_ki = 0.5;  // PID integral coefficient (brake)
float brake_kd = 0.0;  // PID derivative coefficient (brake)
int brake_pid_freq = 10;  // How fast to loop in Hz
int brake_min_release = 0;  // Brake pressure when brakes are effectively off
int brake_max_pressure = 0;  // Highest possible pressure achievable by the actuator
int brake_hold_initial = 0;  // Pressure initially applied when brakes are hit to auto-stop the car
int brake_hold_increment = 0;  // Incremental pressure per second added until car stops (when auto stopping)
int brake_pos_full_retract = 0;  // Brake position value corresponding to retract limit of actuator
int brake_pos_full_extend = 0;  // Brake position value corresponding to extend limit of actuator
int brake_increment_interval = 1000;  // How often to apply increment during auto-stopping
float gas_kp = 0.0;  // PID proportional coefficient (gas)
float gas_ki = 0.0;  // PID integral coefficient (gas)
float gas_kd = 0.0;  // PID derivative coefficient (gas)
int gas_pid_freq = 10;  // How fast to loop in Hz
int throttle_min_closed = 45;  // servo target angle setting (in degrees) corresponding to fully closed throttle
int throttle_max_open = 120;  // servo target angle setting (in degrees) corresponding to full open throttle
int engine_rpm_min_idle = 0;  // Min value for engine_rpm, corresponding to low idle
int engine_rpm_max_redline = 0;  // Max value for engine_rpm, pedal to the metal
int engine_rpm_timeout = 400;  // ms since last magnet pulse when we can assume the engine is stopped
float cruise_kp = 0.0;  // PID proportional coefficient (cruise)
float cruise_ki = 0.0;  // PID integral coefficient (cruise)
float cruise_kd = 0.0;  // PID derivative coefficient (cruise)
int cruise_pid_freq = 10;  // How fast to loop in Hz
int cruise_max_change = 0;  // What's the max car speed change from a single joystick motion in cruise mode? 
int cruise_max_carspeed = 0;  // How fast can we go?
int joy_min_v = 0;  // ADC count of furthest joy position in down direction 
int joy_max_v = 0;  // ADC count of furthest joy position in up direction 
int joy_min_h = 0;  // ADC count of furthest joy position in left direction 
int joy_max_h = 0;  // ADC count of furthest joy position in right direction 
int joy_deadband_v = 140;  // ADC count width of inert readings around center which we should treat as center (vertical)
int joy_deadband_h = 140;  // ADC count width of inert readings around center which we should treat as center (horizontal)
int gesture_adj_timeout = 1000;  // How many ms allowed to complete a speed adjustment gesture
int gesture_fly_timeout = 500;  // How many ms allowed to push joy down to go to Fly mode from Cruise mode
int car_speed_timeout = 400;  // ms since last magnet pulse when we can assume the car is stopped

// Non-settable control variables
//
int steer_max_left = steer_stop+steer_max_speed;   // PWM duty cycle from 0-255
int steer_max_right = steer_stop-steer_max_speed;    // PWM duty cycle from 0-255
int steer_fullrange = abs(steer_max_left-steer_max_right);  // Possible range for target values
int steer_target = 0;  // Stores new setpoint to give to the pid loop (steering)
int steer_pwm = 0;  // pid loop output to send to the actuator (steering)
int brake_fullrange = brake_max_pressure-brake_min_release;  // Possible range for target values
int brake_target = 0;  // Stores new setpoint to give to the pid loop (brake)
int brake_pwm = 0;  // pid loop output to send to the actuator (brake)
int gas_target = 0;  // Stores new setpoint to give to the pid loop (gas)
int gas_pwm = 0;  // pid loop output to send to the actuator (gas)
int throttle_fullrange = throttle_max_open-throttle_min_closed;  // Possible range for target values
int engine_rpm_fullrange = engine_rpm_max_redline-engine_rpm_min_idle;  // Possible range for target values
int joy_deadband_bot_v = int((adcrange-joy_deadband_v)/2);  // Lower threshold of vertical joystick deadband
int joy_deadband_top_v = int((adcrange+joy_deadband_v)/2);  // Upper threshold of vertical joystick deadband
int joy_deadband_bot_h = int((adcrange-joy_deadband_h)/2);  // Lower threshold of horizontal joystick deadband
int joy_deadband_top_h = int((adcrange+joy_deadband_h)/2);  // Upper threshold of horizontal joystick deadband
int car_speed = -1;  // This is in units of rotations per minute of the driven torque converter pulley
int engine_rpm = -1;  // This is in units of rotations per minute of the engine crankshaft
const int gesture_complete = 0;  // How many steps are there in the Cruise Mode gesture
int gesture_extent = 0;  // Keeps track of max joystick movement during cruise mode speed adjustments

int brake_timer = 0;  // Timer used to control braking increments
int cruise_target = 0;  // Stores new setpoint to give to the pid loop (cruise)
int gesture_timer = 0;  // Used to keep track of time for gesturing
int gesture_progress = 0;  // How many steps of the Cruise Mode gesture have you completed successfully (from Fly Mode)
long steer_fullrange_long = (long)steer_fullrange;
const long adcrange_long = (long)adcrange;
long joy_deadband_bot_h_long = (long)joy_deadband_bot_h;
long joy_deadband_top_h_long = (long)joy_deadband_top_h;

// State variables
//
int runmode = SHUTDOWN;  // Variable to store what mode we're in
int oldmode = SHUTDOWN;  // So we can tell when the mode has just changed
bool we_just_switched_modes = true;  // For mode logic to set things up upon first entry into mode
bool shutdown_complete = true;  // Shutdown mode has completed its work and can stop activity

// Initialization of global variables, tunable parameters, etc.
int output_bits = 8;
bool output_signed = false;

boolean RecordOn = false;

unsigned long tach_old_time, speedo_old_time, tach_time, speedo_time;

// Instantiate objects 
Adafruit_FT6206 touchpanel = Adafruit_FT6206();
Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_cs_pin, tft_dc_pin);
static Servo steer_motor;
static Servo gas_servo;
FastPID brake_pid(brake_kp, brake_ki, brake_kd, brake_pid_freq, output_bits, output_signed);
SdFat SD;         // SD card filesystem

// Instantiate PID loops
//
// Steering:
//   Setpoint Value: Proportional to Joystick Horz ADC value.  0V = Full Left, 2.5V = Stop, 5V = Full Right
//   Measured Value: We have no feedback.  Try using a value proportional to the current actuator output
//   Actuator Output Value: PWM signal to Steering Jaguar unit.  0% duty = Full Left, 50% = Stop, 100% = Full Right
//   Limits: Reed switch limit signals for left and right may be handled by us, or by the jaguar controller
//   Setpoint scaling: Kp/Ki/Kd values should decrease appropriately as a function of vehicle speed (safety) 
//
//   Notes: The steering has no feedback sensing, other than two digital limit switches at the ends of travel.  
//   So just consider the error to be the difference between the joystick position and the last output value.
//
// Brakes:
//   Setpoint Value: * Default: Setpoint proportional to Joystick Vert distance from center when below center.
//       * In Hold Mode: Brake adjusts automatically to keep car stopped, as long as joystick below center
//       * In Cruise Mode: Brake is disabled 
//   Measured Value: Analog voltage from brake fluid pressure sensor. 0-3.3V proportional to 0-1000psi
//   Actuator Output Value: PWM signal to Brake Jaguar unit.
//       0% duty = Full speed extend (less brake), 50% = Stop, 100% = Full speed Retract (more brake)
//   Position: Analog 0-3.3V proportional to the travel length of the actuator (not used as feedback)
//
// Gas:
//   Setpoint Value: * Default: Servo angle is proportional to Joystick Vert distance from center when above center.
//       * In Cruise Mode: Upward or downward joy vert motions modify vehicle speed setpoint
//   Measured Value: * Default: Engine speed determined from tach pulses
//   Actuator Output Value: PWM signal to throttle servo
//       0% duty = Fully close throttle.  This will idle.  100% duty = Fully open throttle.
//
// Cruise:
//   Setpoint Value: * Default: Set to the current vehicle speed when mode is entered.
//       * In Cruise Mode: Upward or downward joy vert motions modify vehicle speed setpoint
//   Measured Value: * Vehicle speed determined from tach pulses
//   Actuator Output Value: Cruise PID output values are setpoint values for the Gas PID above
//       0% duty = Car stopped.  100% duty = Car max speed.

static Servo steering_servo;
// Setup code runs once at boot time

bool encb_isr_flag = false;
bool enca_isr_flag = false;
bool encsw_isr_flag = false;
bool tach_isr_flag = false;
bool speedo_isr_flag = false;

void setup() {
    pinMode(enc_b_pin, INPUT_PULLUP);
    pinMode(enc_a_pin, INPUT_PULLUP);
    pinMode(brake_pwm_pin, OUTPUT);
    pinMode(steer_pwm_pin, OUTPUT);
    pinMode(tft_dc_pin, OUTPUT);
    pinMode(enc_sw_pin, INPUT_PULLUP);
    pinMode(gas_pwm_pin, OUTPUT);
    pinMode(ignition_pin, INPUT);
    pinMode(neutral_pin, INPUT_PULLUP);
    pinMode(basicmodesw_pin, INPUT_PULLUP);
    pinMode(tach_pulse_pin, INPUT);
    pinMode(speedo_pulse_pin, INPUT);
    pinMode(joy_horz_pin, INPUT);
    pinMode(joy_vert_pin, INPUT);
    pinMode(brake_pres_pin, INPUT);
    pinMode(brake_pos_pin, INPUT);
    pinMode(mule_volt_pin, INPUT);
    pinMode(usd_cs_pin, OUTPUT);
    pinMode(tft_cs_pin, OUTPUT);
    //pinMode(tft_ledk_pin, OUTPUT);
    //pinMode(tp_irq_pin, INPUT);

    // Set all outputs to known sensible values
    digitalWrite(tft_cs_pin, HIGH);   // Prevent bus contention
    digitalWrite(usd_cs_pin, HIGH);   // Prevent bus contention
    digitalWrite(tft_dc_pin, LOW);
    analogWrite(brake_pwm_pin, 128);   // Write values range from 0 to 255
    analogWrite(steer_pwm_pin, 128);
    analogWrite(gas_pwm_pin, throttle_min_closed);

    // Timers T0 (8b): pins 4,13 - T1 (16b): pins 11,12 - T2 (8b): pins 9,10 - T3 (16b): pins 2,3,5 - T4 (16b): pins 6,7,8 - T5 (16b): pins 44,45,46
    // Registers: TCCRx = Timer control, prescalar, TCNTx = Timer value, OCRx = Output compare, ICRx = Input Capture, TIMSKx = Int mask, TIFRx = Int Flags
    TCCR0B = TCCR0B & B11111000 | B00000101; // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz (Gas Servo)
    TCCR4B = TCCR4B & B11111000 | B00000100; // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz (Brake & Steering)
    
    //while (!Serial);     // needed for debugging?!
    Serial.begin(115200);
    tft.begin();
    
    Serial.println(F("Captouch initialization"));
    if (! touchpanel.begin(40)) {     // pass in 'sensitivity' coefficient
        Serial.println("Couldn't start FT6206 touchscreen controller");
        while (1);
    }
    Serial.println("Capacitive touchscreen started");
    tft.fillScreen(BLACK);
    tft.setRotation(1);  // origin = left,top landscape (USB left upper)
    
    Serial.print(F("Initializing filesystem..."));  // SD card is pretty straightforward, a single call. 
    if (! SD.begin(usd_cs_pin, SD_SCK_MHZ(25))) {   // ESP32 requires 25 MHz limit
        Serial.println(F("SD begin() failed"));
        for(;;); // Fatal error, do not continue
    }
    Serial.println(F("Filesystem started"));
    //tft.fillScreen(ILI9341_BLUE);  // Fill screen blue. 

    steer_motor.attach(steer_pwm_pin);
    gas_servo.attach(gas_pwm_pin);

    // Set up our interrupts
    attachInterrupt(digitalPinToInterrupt(enc_b_pin), EncoderB_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_a_pin), EncoderA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_sw_pin), EncoderSw_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(tach_pulse_pin), Tach_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(speedo_pulse_pin), Speedo_ISR, RISING);

    int i;
    for (i=0; i<=arraysize(telemetry); i++)  {
        writeline(i+1, telemetry[i]);
    }
}

void EncoderB_ISR(void) {
    encb_isr_flag = true;
}
void EncoderA_ISR(void) {
    enca_isr_flag = true;
}
void EncoderSw_ISR(void) {
    encsw_isr_flag = true;
}
void Tach_ISR(void) {
    tach_time = millis();  // This might screw up things.  Anders would remember
    tach_isr_flag = true;
}
void Speedo_ISR(void) {
    speedo_time = millis();  // This might screw up things.  Anders would remember
    speedo_isr_flag = true;
}

void set_steer_pwm(int target_in_degrees) {
    steer_motor.write(target_in_degrees);
}

void set_gas_pwm(int target_in_degrees) {
    gas_servo.write(target_in_degrees);
}

// Returns: The value read on the joystick from 0 (maximum left) to 1024 (maximum right).
// Zero values correspond to the deadband at the center position.
int read_joy_vert(void)  {
    int adc_value = analogRead(joy_vert_pin);
    int joy_vert;
  
    if ((adc_value > joy_deadband_bot_v) && (adc_value < joy_deadband_top_v)) {
        joy_vert = adcmidscale;  // return 0 input if value is in the deadband
    }
    else  {
        joy_vert = adcrange-adc_value;
    }
    return joy_vert;
}

int read_joy_horz(void)  {
    int adc_value = analogRead(joy_horz_pin);
    int joy_horz;
  
    if ((adc_value > joy_deadband_bot_h) && (adc_value < joy_deadband_top_h)) {
        joy_horz = adcmidscale;  // return 0 input if value is in the deadband
    }
    else  {
        joy_horz = adcrange-adc_value;
    }
    return joy_horz;
}

// Inputs: The joystick input between -512 and 512.
// Returns: The joystick target to output from 0 to 180 degrees.
int joy_horz_to_target(int joy_input) {
    int target;
    long joy_input_long = (long)joy_input;
    long max_target_deg_long = (long)steer_fullrange;
    long adcrange_long = (long)adcrange;
  
    target = (int)((joy_input_long*max_target_deg_long)/adcrange_long);
    return target;
}

int joy_vert_to_target(int joy_input) {
    int target;
    long joy_input_long = (long)joy_input;

    long max_target_deg_long;
    long adcrange_long = (long)adcrange;
    if (joy_input > joy_deadband_top_v)  {
        max_target_deg_long = (long)engine_rpm_max_redline;
    }
    else  {
        max_target_deg_long = (long)brake_max_pressure;   
    }
    target = (int)((joy_input_long*max_target_deg_long)/(2*adcrange_long));
    return target;
}
/*int* read_joy_values(void) {
    int joy_raw_values[2];
    joy_raw_values[H] = analogRead(joy_horz_pin);
    joy_raw_values[V] = analogRead(joy_vert_pin);

    int joy_values[2];  

    for (axis=H; axis<=V; axis++) {
        //check if the value is in the deadband, return 0 input if it is
        if ((joy_raw_values[axis] > joy_deadband_bot_) && (joy_raw_values[axis] < joy_deadband_top[axis])) {
            joy_values[axis] = adcmidscale;
        }
        else {
            joy_values[axis] = adcrange-joy_raw_values[axis];
        }
    return joy_values;
}

int *joy_to_actuator_targets(int *joy_values) {
    long servo_max_target_deg_long = (long)steer_fullrange;
    long adcmidscale_long = (long)adcmidscale;
    long adcrange_long = (long)adcrange;

    int actuator_targets[3];
    long joy_values_long[2];
    
    joy_values_long[H] = (long)joy_values[H];  
    joy_values_long[V] = (long)joy_values[V];
    
    actuator_targets[STR] = (int)((joy_values_long[H]*servo_max_target_deg_long)/adcrange_long);
    
    //actuator_targets[BRK] = BRAKE_FULL_EXTEND_TARGET;
    //if (joy_values[V] < adcmidscale)
    //{
    //  actuator_targets[BRK] = (int)((adcmidscale_long-joy_values_long[V])*servo_max_target_deg_long/adcmidscale_long);
    //}

    actuator_targets[GAS] = throttle_min_closed;
    if (joy_values[V] > adcmidscale) {
        actuator_targets[GAS] = (int)((joy_values_long[V]-adcmidscale_long)/adcmidscale_long);
    }

    return actuator_targets;
}
*/

void writevalue(int lineno, char *value) {
    tft.fillRect(80, (lineno-1)*disp_lineheight, 55, disp_lineheight, BLACK);
    //tft.fillRect(0, (lineno-1)*disp_lineheight, disp_linewidth, disp_lineheight, ILI9341_BLACK);
    tft.setCursor(86, ((lineno-1)*disp_lineheight)+disp_lineheight/2);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println(value);
    RecordOn = false;
}

void writeline(int lineno, char *value) {
    tft.fillRect(0, (lineno-1)*disp_lineheight, disp_linewidth, disp_lineheight, BLACK);
    tft.setCursor(6, ((lineno-1)*disp_lineheight)+disp_lineheight/2);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println(value);
    RecordOn = false;
}

void loop() {
    // Main loop.  Each time through we do these eight steps:
    //
    // 1) Gather new telemetry
    // 2) Check if our current runmode has been overridden by certain specific conditions
    // 3) Read joystick horizontal and determine new steering setpoint
    // 4) Do actions based on which runmode we are in (including gas & brake setpoint), and possibly change runmode 
    // 5) Step the PID loops and update the actuation outputs
    // 6) Service the user interface
    // 7) Log to SD card
    // 8) Do the control loop bookkeeping at the end of each loop

    // Initialize variables
    //
    int joy_vert, joy_horz, old_joy_vert, old_joy_horz; 
    char buffer[6];
    int steer_target, brake_target, gas_target, old_steer_target, old_brake_target, old_gas_target;
    int steer_pwm, brake_pwm, gas_pwm, old_steer_pwm, old_brake_pwm, old_gas_pwm; 
    int engine_rpm, old_engine_rpm, car_speed, old_car_speed;
    int before, after;
    int old_brake_pressure;
    int pwmtimer;

    // 1) Gather new telemetry
    //
    int now = millis();
    if (tach_isr_flag == true) {       // Update engine rpm value
        float engine_rpm = 60*1000/(4*(tach_time-tach_old_time));
        tach_old_time = tach_time;
        tach_isr_flag = false;
    } 
    else if (now-tach_old_time > engine_rpm_timeout) {
        engine_rpm = 0;
    }
    if (speedo_isr_flag == true) {   // Update car speed value
        float car_speed = 60*1000/(8*(speedo_time-speedo_old_time));
        speedo_old_time = speedo_time;
        speedo_isr_flag = false;
    }  // Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
    else if (now-speedo_old_time > car_speed_timeout) {
        car_speed = 0;
    }
    
    // Read the sensors
    int brake_pressure = analogRead(brake_pres_pin);
    int brake_pos = analogRead(brake_pos_pin);
    int mule_voltage = analogRead(mule_volt_pin);
    int neutral = digitalRead(neutral_pin);
    int ignition = digitalRead(ignition_pin);
    //int basicmodesw = digitalRead(basicmodesw_pin);
    int basicmodesw = 1;  // Disable for now until connected

    // 2) Check if our current runmode has been overridden by certain specific conditions
    //
    if (ignition == 0)  {           // if ignition off --> Shutdown Mode
        runmode = SHUTDOWN;
    }
    else if (basicmodesw == 0) {    // elif basicmode switch on --> Basic Mode
        runmode = BASIC;
    }
    else if (engine_rpm == 0)  {    // elif engine not running --> Stall Mode
        runmode = STALL; 
    }

    // 3) Read joystick horizontal and determine new steering setpoint
    //
    joy_vert = read_joy_vert();
    joy_horz = read_joy_horz();

    if (!(runmode == SHUTDOWN && (car_speed == 0 or shutdown_complete == true)))  { // The only time we don't want steering
        long joy_horz_long = (long)joy_horz;
         if (joy_horz < joy_deadband_bot_h)  {    // If we are trying to turn left
            // This should determine how hard we're trying to turn (0 to 1 range)
            int temp = (int)((joy_deadband_bot_h_long-joy_horz_long)/joy_deadband_bot_h_long);
            int steer_target = (steer_fullrange_long/2)*(1-temp);
        }
        else if (joy_horz > joy_deadband_top_h)  {  // If we are trying to turn right
            // This should determine how hard we're trying to turn (0 to 1 range)
            int temp = (int)((joy_horz_long-joy_deadband_top_h_long)/joy_deadband_bot_h_long);
            int steer_target = (steer_fullrange_long/2)*(1+temp);
            //int steer_target = (steer_fullrange_long/2)*(1+(int)((joy_horz_long-joy_deadband_top_h_long)/joy_deadband_bot_h_long));
        }
        else {
            steer_target = steer_stop;  // adjust to actual center
        }
    }
    // Try this with the commented function above
    // joy_values = read_joy_values();
    // joy_horz = joy_values[0];
    // joy_vert = joy_values[1];

    // Old: steer_target = joy_horz_to_target(joy_horz);
    // Old: gas_target = joy_vert_to_target(joy_vert);

    // 4) Do actions based on which runmode we are in (and set gas/brake setpoints), and possibly change runmode 
    //
    now = millis();

    if (runmode == SHUTDOWN)  {
        if (basicmodesw == 0)  {   // If basic mode switch is enabled
            shutdown_complete = true;  
        }
        else if (we_just_switched_modes == true)  {  // If basic switch is off, we need to stop the car then release brakes and gas before shutting down                
            gas_target = engine_rpm_min_idle;  //  Begin Letting off the gas all the way
            shutdown_complete = false;
            if (car_speed > 0)  {
                brake_target = brake_hold_initial;  // More brakes, etc. to stop the car
                brake_timer = now;
            }
        }
        if (shutdown_complete == false)  {  // If we haven't yet stopped the car and released the brakes and gas all the way
            if (car_speed == 0)  {  // Car is stopped, but maybe we still need to release the brakes
                brake_target = brake_min_release;  // Start to Fully release brakes
                if (brake_pressure == brake_min_release)  {
                    shutdown_complete = true;  // With this set, we will do nothing from here on out (until mode changes, i.e. ignition)
                }
            }
            else if (brake_timer-now > brake_increment_interval)  {
                brake_target += brake_hold_increment;  // Slowly add more brakes until car stops
                brake_timer = now;  
            }
        }
    }
    else if (runmode == BASIC)  {
        if (basicmodesw == 1 && engine_rpm > 0)  {  // If we turned off the basic mode switch with engine running, go to Hold mode
            runmode = HOLD;   //  If engine is not running, we'll end up in Stall Mode automatically
        }
    }
    else if (runmode == STALL)  {   // In stall mode, the gas doesn't have feedback}
        // PID limits don't apply any more when not using engine rpm as fb.  Fix this
        if (engine_rpm > 0)  {  //  enter Hold Mode if we started the car
            runmode = HOLD;
        }
        else {
            gas_target = engine_rpm_min_idle;  // Default when joystick not pressed
            brake_target = brake_min_release;  // Default when joystick not pressed
            if (joy_vert > joy_deadband_top_v)  { //  If we are pushing up
                gas_target = engine_rpm_fullrange*(joy_vert-joy_deadband_top_v)/joy_deadband_bot_v;
            }
            else if (joy_vert < joy_deadband_bot_v)  {  // If we are pushing down
                brake_target = brake_fullrange*(joy_deadband_bot_v-joy_vert)/joy_deadband_bot_v;
            }
        }
    }
    else if (runmode == HOLD)  {
        if (joy_vert > joy_deadband_top_v)  { // Enter Fly Mode if joystick is pushed up
            runmode = FLY;
        }
        else if (we_just_switched_modes == true)  {  // Release throttle and push brake upon entering hold mode
            gas_target = engine_rpm_min_idle;  // Let off gas 
            brake_target = brake_hold_initial;  // Apply brakes to preset setpoint
            brake_timer = now;
        }
        else if (car_speed > 0 && brake_timer-now > brake_increment_interval)  {  // If car starts moving again, push harder
            brake_target += brake_hold_increment; //  Increase the brake pressure slowly when trying to stop car
            brake_timer = now;
        }
    }
    else if (runmode == FLY)  {
        if (we_just_switched_modes == true)  {
            gesture_progress = 0;
            gesture_timer = now;
        }
        // else if (gesture motion detected)  {   // If next motion of gesture completed in time
        //     // Add checks to identify conditions to satisfy steps of the gesture
        //     gesture_progress++;
        //     gesture_timer = now;
        // }
        else if (gesture_progress == gesture_complete)  { // Series of motions is complete
            runmode = CRUISE;
        }
        else if (now-gesture_timer > gesture_adj_timeout)  {  // Gesture timed out
            gesture_progress = 0; 
        }
        if ((car_speed == 0 && joy_vert < joy_deadband_bot_v) || neutral)  {
            runmode = HOLD;  // Back to Hold Mode if we have braked to a stop or shifted out of gear
        }
        else  {  // Use PID to drive
            gas_target = engine_rpm_min_idle;  // Default when joystick not pressed
            brake_target = brake_min_release;  // Default when joystick not pressed
            if (joy_vert > joy_deadband_top_v)  {  // If we are trying to accelerate
                gas_target = engine_rpm_fullrange*(joy_vert-joy_deadband_top_v)/joy_deadband_bot_v;
            }
            else if (joy_vert < joy_deadband_bot_v)  {  // If we are trying to brake
                brake_target = brake_fullrange*(joy_deadband_bot_v-joy_vert)/joy_deadband_bot_v;
            }
        }
    }   
    else if (runmode == CRUISE)  { // Maybe the elif clauses within here should be if clauses.  Maybe doesn't matter
        if (car_speed == 0 or neutral > 0)  {  // this should never happen
            runmode = HOLD;  // Back to Hold Mode because we don't understand what's going on
        }
        else if (we_just_switched_modes == true)  {
            cruise_target = car_speed;
            brake_target = brake_min_release;
            gesture_extent = adcrange/2;  // Variable to keep track of our joystick motion for speed adjustments
        }
        if (joy_deadband_bot_v < joy_vert && joy_deadband_top_v > joy_vert)  {  // joystick within deadband near center: do speed adjustment, and reset gesture timer
            if (gesture_extent > joy_deadband_top_v && now-gesture_timer < gesture_adj_timeout)  { // Adjust speed upward based on how high we were pushed
                cruise_target += cruise_max_change*(gesture_extent-joy_deadband_top_v)/joy_deadband_bot_v;
            }
            else if (gesture_extent < joy_deadband_bot_v && now-gesture_timer < gesture_adj_timeout)  {
                cruise_target -= cruise_max_change*(joy_deadband_bot_v-gesture_extent)/joy_deadband_bot_v;                    
            }
            gesture_extent = adcrange/2;  // Reset for next time
            gesture_timer = now;
        }
        else if (joy_vert == joy_min_v && now-gesture_timer < gesture_fly_timeout)  { // If joystick quickly pushed down
            runmode = FLY;
        }
        else if (joy_vert > joy_deadband_top_v && joy_vert > gesture_extent)  {  // We pushed it even further up
            gesture_extent = joy_vert;  // Remember our highest point
        }
        else if (joy_vert < joy_deadband_bot_v && joy_vert < gesture_extent)  {  // We pushed it even further down
            gesture_extent = joy_vert;  // Remember our lowest point
        }
        // Commented out during initial debug 
        //gas_target = cruise_pid.step(feedback=car_speed, setpoint = cruise_target);
    }
    else  { // Obviously this should never happen
        runmode = HOLD;
    }

    // 5) Step the pids, update the actuator outputs
    //
    // The pid approach is the most cool.  Fix and uncomment these as we can
    if (! (runmode == SHUTDOWN && shutdown_complete == true))  {  // The only time we don't want steering
        // steering_pwm = steering_pid.step(feedback=none, setpoint = steer_target);
    
        if (! basicmodesw == 0)  {  // If basicmode switch is disabled we want brake and gas
            // brake_pwm = brake_pid.step(feedback=brake_pressure, setpoint = brake_target);

            if (runmode == STALL)  {
                // gas_pwm = throttle_pid.step(feedback=none, setpoint = gas_target);
            }
            else {
                // gas_pwm = throttle_pid.step(feedback=engine_rpm, setpoint = gas_target);   
            }
        }
    }

    now = millis();
    
    // Update gas and steering output only every 15ms
    if (now-pwmtimer > 15)  {
        set_steer_pwm(steer_target);
        set_gas_pwm(gas_target);
        pwmtimer = now;
    }
    
    // Step the brake pid
    before = micros();
    brake_target = brake_pid.step(brake_min_release, brake_pressure);
    // uint8_t brake_target = brake_pid.step(brake_min_release, brake_pressure);
    after = micros();
    analogWrite(brake_pwm_pin, brake_pwm);
    Serial.print("runtime: "); 
    Serial.print(after - before);
    Serial.print(" sp: "); 
    Serial.print(brake_min_release); 
    Serial.print(" fb: "); 
    Serial.print(brake_pressure);
    Serial.print(" out: ");
    Serial.println(brake_target);

    // Try this later
    // if (car_speed != old_car_speed) {
    //    writevalue(11, (char*)car_speed;
    //    old_car_speed = car_speed;
    // }

    set_steer_pwm(steer_target); 
    set_gas_pwm(gas_target); 
    
    // 6) Service the user interface
    //
    // Act on any encoder action
    
    // See if someone is groping the touch screen
    if (touchpanel.touched())  {  // See if there's any  touch data for us
        TS_Point touchpoint = touchpanel.getPoint();   // Retreive a point
        touchpoint.x = map(touchpoint.x, 0, 240, 240, 0);  // Rotate touch coordinates to match tft 
        touchpoint.y = map(touchpoint.y, 0, 320, 320, 0);
        int y = tft.height()-touchpoint.x;
        int x = touchpoint.y;
        memset(buffer,0,strlen(buffer));
        itoa(x, buffer, 10);
        writevalue(13, buffer);
        memset(buffer,0,strlen(buffer));
        itoa(y, buffer, 10);
        writevalue(14, buffer);
    }
    
    // Write new values to the screen

    if (runmode != oldmode) {
        writevalue(1, modecard[runmode]);
    }
    if (car_speed != old_car_speed) {
        memset(buffer,0,strlen(buffer));
        itoa(car_speed, buffer, 10);
        writevalue(2, buffer);
        old_car_speed = car_speed;
    }
    if (joy_horz != old_joy_horz) {
        memset(buffer,0,strlen(buffer));
        itoa(joy_horz, buffer, 10);
        writevalue(3, buffer);
        old_joy_horz = joy_horz;
    }
    if (joy_vert != old_joy_vert) {
        memset(buffer,0,strlen(buffer));
        itoa(joy_vert, buffer, 10);
        writevalue(4, buffer);
        old_joy_vert = joy_vert;
    }
    if (steer_target != old_steer_target) {
        memset(buffer,0,strlen(buffer));
        itoa(steer_target, buffer, 10);
        writevalue(5, buffer);
        old_steer_target = steer_target;
    }
    if (steer_pwm != old_steer_pwm) {
        memset(buffer,0,strlen(buffer));
        itoa(steer_pwm, buffer, 10);
        writevalue(6, buffer);
        old_steer_pwm = steer_pwm;
    }
    if (brake_target != old_brake_target) {
        memset(buffer,0,strlen(buffer));
        itoa(brake_target, buffer, 10);
        writevalue(7, buffer);
        old_brake_target = brake_target;
    }
    if (brake_pwm != old_brake_pwm) {
        memset(buffer,0,strlen(buffer));
        itoa(brake_pwm, buffer, 10);
        writevalue(8, buffer);
        old_brake_pwm = brake_pwm;
    }
    if (brake_pressure != old_brake_pressure) {
        memset(buffer,0,strlen(buffer));
        itoa(brake_pressure, buffer, 10);
        writevalue(9, buffer);
        old_brake_pressure = brake_pressure;
    }
    if (gas_target != old_gas_target) {
        memset(buffer,0,strlen(buffer));
        itoa(gas_target, buffer, 10);
        writevalue(10, buffer);
        old_gas_target = gas_target;
    }
    if (gas_pwm != old_gas_pwm) {
        memset(buffer,0,strlen(buffer));
        itoa(gas_pwm, buffer, 10);
        writevalue(11, buffer);
        old_gas_pwm = gas_pwm;
    }
    if (engine_rpm != old_engine_rpm) {
        memset(buffer,0,strlen(buffer));
        itoa(engine_rpm, buffer, 10);
        writevalue(12, buffer);
        old_engine_rpm = engine_rpm;
    }
    memset(buffer,0,strlen(buffer));
    itoa(analogRead(joy_horz_pin), buffer, 10);
    writevalue(15, buffer);
    
    // 7) Log to SD card
    //
    // [write current state with timestamp to the SD card]

    // 8) Do the control loop bookkeeping at the end of each loop
    //
    old_joy_vert = joy_vert;
    if (! oldmode == runmode)  {      // If changing runmode
        we_just_switched_modes = true;    // So new mode logic can perform initial actions
    }
    else {
        we_just_switched_modes = false;    // Reset this variable
    }
    oldmode = runmode;   // remember what mode we're in for next time

    delay(10);

}