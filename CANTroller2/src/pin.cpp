#include "pin.h"
#include "Arduino.h"

Pin::Pin(PinNumber pin_num, PinMode mode) {
  // Validate pin number range
  validate_pin_number(static_cast<int>(pin_num));
  pin_num_ = pin_num;
  set_mode(mode); 
}

void Pin::set_mode(PinMode mode) {
  ::pinMode(static_cast<uint8_t>(pin_num_), static_cast<uint8_t>(mode));
}

void Pin::digital_write(uint8_t value) {
  ::digitalWrite(static_cast<uint8_t>(pin_num_), value);
}

uint8_t Pin::digital_read() const {
  return ::digitalRead(static_cast<uint8_t>(pin_num_)); 
}

void Pin::attach_interrupt(void (*isr)(), InterruptMode mode) {
  ::attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(pin_num_)), isr, static_cast<int>(mode));
}

void Pin::validate_pin_number(int pin_num) {
  if(pin_num < -1 || pin_num > NUM_PINS){
      throw std::invalid_argument("Pin number out of range");
  }
}

// (button0 / strap to 1) - This is the "Boot" button on the esp32 board
Pin button(PinNumber::GPIO_NUM_0, PinMode::INPUT_PULLUP);

// (adc) - Either analog left-right input (joystick)  
Pin joy_horz(PinNumber::GPIO_NUM_1, PinMode::INPUT); 

// (adc) - Either analog up-down input (joystick)
Pin joy_vert(PinNumber::GPIO_NUM_2, PinMode::INPUT);

// (adc* / strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
Pin tft_dc(PinNumber::GPIO_NUM_3, PinMode::OUTPUT);

// (adc) - Analog input, ignition signal and battery voltage sense, full scale is 15.638V
Pin ign_batt(PinNumber::GPIO_NUM_4, PinMode::INPUT);

// (adc) - Analog in from 20k pot. Use 1% series R=22k to 5V on wipe=CW-0ohm side, and R=15k to gnd on wipe-CCW-0ohm side. Gives wipe range of 1.315V (CCW) to 3.070V (CW) with 80 uA draw.  
Pin pot_wipe(PinNumber::GPIO_NUM_5, PinMode::INPUT);

// (adc) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
Pin brake_pos(PinNumber::GPIO_NUM_6, PinMode::INPUT);   

// (adc) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
Pin pressure(PinNumber::GPIO_NUM_7, PinMode::INPUT);

// (i2c0 sda / adc) - i2c bus for airspeed sensor, lighting board, cap touch screen  
Pin i2c_sda(PinNumber::GPIO_NUM_8, PinMode::INPUT);

// (i2c0 scl / adc) - i2c bus for airspeed sensor, lighting board, cap touch screen
Pin i2c_scl(PinNumber::GPIO_NUM_9, PinMode::INPUT); 

// (spi0 cs / adc*) - Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
Pin tft_cs(PinNumber::GPIO_NUM_10, PinMode::OUTPUT);

// (spi0 mosi / adc*) - Used as spi interface data to sd card and tft screen
Pin tft_mosi(PinNumber::GPIO_NUM_11, PinMode::OUTPUT);

// (spi0 sclk / adc*) - Used as spi interface clock for sd card and tft screen  
Pin tft_sclk(PinNumber::GPIO_NUM_12, PinMode::OUTPUT);

// (spi0 miso / adc*) - Used as spi interface data from sd card and possibly (?) tft screen
Pin tft_miso(PinNumber::GPIO_NUM_13, PinMode::INPUT);

// (pwm0 / adc*) - Hotrc Ch2 bidirectional trigger input
Pin hotrc_ch2_vert(PinNumber::GPIO_NUM_14, PinMode::INPUT);

// (pwm1 / adc*) - Hotrc Ch1 thumb joystick input  
Pin hotrc_ch1_horz(PinNumber::GPIO_NUM_15, PinMode::INPUT); 

// (pwm1 / adc*) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
Pin gas_pwm(PinNumber::GPIO_NUM_16, PinMode::OUTPUT);

// (pwm0 / adc* / tx1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped)
Pin brake_pwm(PinNumber::GPIO_NUM_17, PinMode::OUTPUT);

// (pwm0 / adc* / rx1) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
Pin steer_pwm(PinNumber::GPIO_NUM_18, PinMode::OUTPUT);  

// (usb-otg / adc*) - Onewire bus for temperature sensor data
Pin onewire(PinNumber::GPIO_NUM_19, PinMode::INPUT);

// (usb-otg / adc*) - Ignition control, Hotrc Ch3 PWM toggle signal
Pin hotrc_ch3_ign(PinNumber::GPIO_NUM_20, PinMode::INPUT);

// (pwm0) - Cruise control, Hotrc Ch4 PWM toggle signal
Pin hotrc_ch4_cruise(PinNumber::GPIO_NUM_21, PinMode::INPUT);

// (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
Pin tach_pulse(PinNumber::GPIO_NUM_35, PinMode::INPUT_PULLUP);

// (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)  
Pin speedo_pulse(PinNumber::GPIO_NUM_36, PinMode::INPUT_PULLUP); 

// (spi-ram / oct-spi) - Output for Hotrc to a relay to kill the car ignition. Note, Joystick ign button overrides this if connected and pressed
Pin ign_out(PinNumber::GPIO_NUM_37, PinMode::OUTPUT);

// (spi-ram / oct-spi) - Output, flips a relay to power all the tranducers 
Pin syspower(PinNumber::GPIO_NUM_38, PinMode::OUTPUT);

// Use as chip select for resistive touchscreen
Pin touch_cs(PinNumber::GPIO_NUM_39, PinMode::OUTPUT);

// Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
Pin encoder_b(PinNumber::GPIO_NUM_40, PinMode::INPUT_PULLUP);

// Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
Pin encoder_a(PinNumber::GPIO_NUM_41, PinMode::INPUT_PULLUP);

// Input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
Pin encoder_sw(PinNumber::GPIO_NUM_42, PinMode::INPUT_PULLUP);

// "TX" (uart0 tx) - Serial monitor terminal and potentially a better connection to jaguar controllers
Pin uart_tx(PinNumber::GPIO_NUM_43, PinMode::OUTPUT);

// "RX" (uart0 rx) - Serial monitor terminal and potentially a better connection to jaguar controllers  
Pin uart_rx(PinNumber::GPIO_NUM_44, PinMode::INPUT);

// (strap to 0) - Input, active high when vehicle starter is engaged (needs pulldown)
Pin starter(PinNumber::GPIO_NUM_45, PinMode::INPUT_PULLDOWN);

// (strap X) - Input, asserted to tell us to run in basic mode, active low (needs pullup)
Pin basicmodesw(PinNumber::GPIO_NUM_46, PinMode::INPUT_PULLUP); 

// Output, chip select allows SD card controller chip use of the SPI bus, active low
Pin sdcard_cs(PinNumber::GPIO_NUM_47, PinMode::OUTPUT);

// (rgb led) - Data line to onboard Neopixel WS281x  
Pin neopixel(PinNumber::GPIO_NUM_48, PinMode::OUTPUT);
