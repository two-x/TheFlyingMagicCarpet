#include "Arduino.h"
#include "pins.h"
#include "pin.h"

Pins::Pins(){}
void Pins::init() {
    button = Pin(PinNumber::GPIO_NUM_0, INPUT_PULLUP); // (button0 / strap to 1) - This is the "Boot" button on the esp32 board
    joy_horz = Pin(PinNumber::GPIO_NUM_1, INPUT); // (adc) - Either analog left-right input (joystick)  
    joy_vert = Pin(PinNumber::GPIO_NUM_2, INPUT); // (adc) - Either analog up-down input (joystick)
    tft_dc = Pin(PinNumber::GPIO_NUM_3, OUTPUT); // (adc* / strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
    ign_batt = Pin(PinNumber::GPIO_NUM_4, INPUT); // (adc) - Analog input, ignition signal and battery voltage sense, full scale is 15.638V
    pot_wipe = Pin(PinNumber::GPIO_NUM_5, INPUT); // (adc) - Analog in from 20k pot. Use 1% series R=22k to 5V on wipe=CW-0ohm side, and R=15k to gnd on wipe-CCW-0ohm side. Gives wipe range of 1.315V (CCW) to 3.070V (CW) with 80 uA draw.  
    brake_pos = Pin(PinNumber::GPIO_NUM_6, INPUT); // (adc) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
    pressure = Pin(PinNumber::GPIO_NUM_7, INPUT); // (adc) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
    i2c_sda = Pin(PinNumber::GPIO_NUM_8, INPUT); // (i2c0 sda / adc) - i2c bus for airspeed sensor, lighting board, cap touch screen
    i2c_scl = Pin(PinNumber::GPIO_NUM_9, INPUT); // (i2c0 scl / adc) - i2c bus for airspeed sensor, lighting board, cap touch screen  
    tft_cs = Pin(PinNumber::GPIO_NUM_10, OUTPUT); // (spi0 cs / adc*) - Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
    tft_mosi = Pin(PinNumber::GPIO_NUM_11, OUTPUT); // (spi0 mosi / adc*) - Used as spi interface data to sd card and tft screen
    tft_sclk = Pin(PinNumber::GPIO_NUM_12, OUTPUT); // (spi0 sclk / adc*) - Used as spi interface clock for sd card and tft screen
    tft_miso = Pin(PinNumber::GPIO_NUM_13, INPUT); // (spi0 miso / adc*) - Used as spi interface data from sd card and possibly (?) tft screen
    hotrc_ch2_vert = Pin(PinNumber::GPIO_NUM_14, INPUT); // (pwm0 / adc*) - Hotrc Ch2 bidirectional trigger input  
    hotrc_ch1_horz = Pin(PinNumber::GPIO_NUM_15, INPUT); // (pwm1 / adc*) - Hotrc Ch1 thumb joystick input
    gas_pwm = Pin(PinNumber::GPIO_NUM_16, OUTPUT); // (pwm1 / adc*) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
    brake_pwm = Pin(PinNumber::GPIO_NUM_17, OUTPUT); // (pwm0 / adc* / tx1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped)  
    steer_pwm = Pin(PinNumber::GPIO_NUM_18, OUTPUT); // (pwm0 / adc* / rx1) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
    onewire = Pin(PinNumber::GPIO_NUM_19, INPUT); // (usb-otg / adc*) - Onewire bus for temperature sensor data
    hotrc_ch3_ign = Pin(PinNumber::GPIO_NUM_20, INPUT); // (usb-otg / adc*) - Ignition control, Hotrc Ch3 PWM toggle signal
    hotrc_ch4_cruise = Pin(PinNumber::GPIO_NUM_21, INPUT); // (pwm0) - Cruise control, Hotrc Ch4 PWM toggle signal  
    tach_pulse = Pin(PinNumber::GPIO_NUM_35, INPUT_PULLUP); // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
    speedo_pulse = Pin(PinNumber::GPIO_NUM_36, INPUT_PULLUP); // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
    ign_out = Pin(PinNumber::GPIO_NUM_37, OUTPUT); // (spi-ram / oct-spi) - Output for Hotrc to a relay to kill the car ignition. Note, Joystick ign button overrides this if connected and pressed
    syspower = Pin(PinNumber::GPIO_NUM_38); // (spi-ram / oct-spi) - Output, flips a relay to power all the tranducers  
    touch_cs = Pin(PinNumber::GPIO_NUM_39, OUTPUT); // Use as chip select for resistive touchscreen
    encoder_b = Pin(PinNumber::GPIO_NUM_40, INPUT_PULLUP); // Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
    encoder_a = Pin(PinNumber::GPIO_NUM_41, INPUT_PULLUP); // Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
    encoder_sw = Pin(PinNumber::GPIO_NUM_42, INPUT_PULLUP); // Input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
    uart_rx = Pin(PinNumber::GPIO_NUM_44, INPUT); // "RX" (uart0 rx) - Serial monitor terminal and potentially a better connection to jaguar controllers  
    starter = Pin(PinNumber::GPIO_NUM_45, INPUT_PULLDOWN); // (strap to 0) - Input, active high when vehicle starter is engaged (needs pulldown)
    basicmodesw = Pin(PinNumber::GPIO_NUM_46, INPUT_PULLUP); // (strap X) - Input, asserted to tell us to run in basic mode, active low (needs pullup)
    sdcard_cs = Pin(PinNumber::GPIO_NUM_47, OUTPUT); // Output, chip select allows SD card controller chip use of the SPI bus, active low
    neopixel = Pin(PinNumber::GPIO_NUM_48, OUTPUT); // (rgb led) - Data line to onboard Neopixel WS281x
    tft_rst = Pin(PinNumber::GPIO_DISABLED); // TFT Reset allows us to reboot the screen hardware when it crashes  
    tft_ledk = Pin(PinNumber::GPIO_DISABLED); // Output, optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
    touch_irq = Pin(PinNumber::GPIO_TOUCH); // Input, optional touch occurence interrupt signal (for resistive touchscreen, prevents spi bus delays) - Set to 255 if not used

    tft_cs.digital_write(HIGH);   // Prevent bus contention
    sdcard_cs.digital_write(HIGH);   // Prevent bus contention
    tft_dc.digital_write(LOW);
    tft_rst.digital_write(HIGH);
    ign_out.digital_write(LOW);

    // This bit is here as a way of autdetecting soren's breadboard, since his LCD is wired upside-down.
    // Soren put a strong external pulldown on the pin, so it'll read low for autodetection. 
    // syspower.set_mode(INPUT); // Temporarily use syspower pin to read a pullup/down resistor to configure screen flip
    // flip_the_screen = !syspower.digital_read();  // Will cause the LCD to be upside down TODO figure out how to use this without race conditions from setup()
    syspower.set_mode(OUTPUT); // Then set the put as an output as normal.
    syspower.digital_write(HIGH);
}