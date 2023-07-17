#include "Arduino.h"
#include "pins.h"
#include "pin.h"
// #include "globals.h"

Pins::Pins(){}
void Pins::init() {

    Pin button(PinNumber::GPIO_NUM_0, INPUT_PULLUP); // (button0 / strap to 1) - This is the "Boot" button on the esp32 board
    Pin joy_horz(PinNumber::GPIO_NUM_1, INPUT); // (adc) - Either analog left-right input (joystick)
    Pin joy_vert(PinNumber::GPIO_NUM_2, INPUT); // (adc) - Either analog up-down input (joystick) 
    Pin tft_dc(PinNumber::GPIO_NUM_3, OUTPUT); // (adc* / strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
    Pin ign_batt(PinNumber::GPIO_NUM_4, INPUT); // (adc) - Analog input, ignition signal and battery voltage sense, full scale is 15.638V
    Pin pot_wipe(PinNumber::GPIO_NUM_5, INPUT); // (adc) - Analog in from 20k pot. Use 1% series R=22k to 5V on wipe=CW-0ohm side, and R=15k to gnd on wipe-CCW-0ohm side. Gives wipe range of 1.315V (CCW) to 3.070V (CW) with 80 uA draw.
    Pin brake_pos(PinNumber::GPIO_NUM_6, INPUT); // (adc) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.  
    Pin pressure(PinNumber::GPIO_NUM_7, INPUT); // (adc) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
    Pin i2c_sda(PinNumber::GPIO_NUM_8, INPUT); // (i2c0 sda / adc) - i2c bus for airspeed sensor, lighting board, cap touch screen
    Pin i2c_scl(PinNumber::GPIO_NUM_9, INPUT); // (i2c0 scl / adc) - i2c bus for airspeed sensor, lighting board, cap touch screen
    Pin tft_cs(PinNumber::GPIO_NUM_10, OUTPUT); // (spi0 cs / adc*) - Output, active low, Chip select allows ILI9341 display chip use of the SPI bus  
    Pin tft_mosi(PinNumber::GPIO_NUM_11, OUTPUT); // (spi0 mosi / adc*) - Used as spi interface data to sd card and tft screen
    Pin tft_sclk(PinNumber::GPIO_NUM_12, OUTPUT); // (spi0 sclk / adc*) - Used as spi interface clock for sd card and tft screen
    Pin tft_miso(PinNumber::GPIO_NUM_13, INPUT); // (spi0 miso / adc*) - Used as spi interface data from sd card and possibly (?) tft screen
    Pin hotrc_ch2_vert(PinNumber::GPIO_NUM_14, INPUT); // (pwm0 / adc*) - Hotrc Ch2 bidirectional trigger input
    Pin hotrc_ch1_horz(PinNumber::GPIO_NUM_15, INPUT); // (pwm1 / adc*) - Hotrc Ch1 thumb joystick input
    Pin gas_pwm(PinNumber::GPIO_NUM_16, OUTPUT); // (pwm1 / adc*) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)  
    Pin brake_pwm(PinNumber::GPIO_NUM_17, OUTPUT); // (pwm0 / adc* / tx1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped)
    Pin steer_pwm(PinNumber::GPIO_NUM_18, OUTPUT); // (pwm0 / adc* / rx1) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
    Pin onewire(PinNumber::GPIO_NUM_19, INPUT); // (usb-otg / adc*) - Onewire bus for temperature sensor data
    Pin hotrc_ch3_ign(PinNumber::GPIO_NUM_20, INPUT); // (usb-otg / adc*) - Ignition control, Hotrc Ch3 PWM toggle signal  
    Pin hotrc_ch4_cruise(PinNumber::GPIO_NUM_21, INPUT); // (pwm0) - Cruise control, Hotrc Ch4 PWM toggle signal
    Pin tach_pulse(PinNumber::GPIO_NUM_35, INPUT_PULLUP); // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
    Pin speedo_pulse(PinNumber::GPIO_NUM_36, INPUT_PULLUP); // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
    Pin ign_out(PinNumber::GPIO_NUM_37, OUTPUT); // (spi-ram / oct-spi) - Output for Hotrc to a relay to kill the car ignition. Note, Joystick ign button overrides this if connected and pressed  
    Pin syspower(PinNumber::GPIO_NUM_38); // (spi-ram / oct-spi) - Output, flips a relay to power all the tranducers
    Pin touch_cs(PinNumber::GPIO_NUM_39, OUTPUT); // Use as chip select for resistive touchscreen
    Pin encoder_b(PinNumber::GPIO_NUM_40, INPUT_PULLUP); // Int input, The B (aka DT) pin of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
    Pin encoder_a(PinNumber::GPIO_NUM_41, INPUT_PULLUP); // Int input, The A (aka CLK) pin of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)  
    Pin encoder_sw(PinNumber::GPIO_NUM_42, INPUT_PULLUP); // Input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
    Pin uart_tx(PinNumber::GPIO_NUM_43, OUTPUT); // "TX" (uart0 tx) - Serial monitor terminal and potentially a better connection to jaguar controllers
    Pin uart_rx(PinNumber::GPIO_NUM_44, INPUT); // "RX" (uart0 rx) - Serial monitor terminal and potentially a better connection to jaguar controllers
    Pin starter(PinNumber::GPIO_NUM_45, INPUT_PULLDOWN); // (strap to 0) - Input, active high when vehicle starter is engaged (needs pulldown)
    Pin basicmodesw(PinNumber::GPIO_NUM_46, INPUT_PULLUP); // (strap X) - Input, asserted to tell us to run in basic mode, active low (needs pullup)
    Pin sdcard_cs(PinNumber::GPIO_NUM_47, OUTPUT); // Output, chip select allows SD card controller chip use of the SPI bus, active low 
    Pin neopixel(PinNumber::GPIO_NUM_48, OUTPUT); // (rgb led) - Data line to onboard Neopixel WS281x
    Pin tft_rst(PinNumber::GPIO_DISABLED); // TFT Reset allows us to reboot the screen hardware when it crashes
    Pin tft_ledk(PinNumber::GPIO_DISABLED); // Output, optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
    Pin touch_irq(PinNumber::GPIO_DISABLED); // Input, optional touch occurence interrupt signal (for resistive touchscreen, prevents spi bus delays) - Set to 255 if not used // TODO check on this, it seems that -1 is better

    tft_cs.digital_write(HIGH);   // Prevent bus contention
    sdcard_cs.digital_write(HIGH);   // Prevent bus contention
    tft_dc.digital_write(LOW);
    tft_rst.digital_write(HIGH);
    ign_out.digital_write(LOW);

    // This bit is here as a way of autdetecting soren's breadboard, since his LCD is wired upside-down.
    // Soren put a strong external pulldown on the pin, so it'll read low for autodetection. 
    syspower.set_mode(INPUT); // Temporarily use syspower pin to read a pullup/down resistor to configure screen flip
    // flip_the_screen = !syspower.digital_read();  // Will cause the LCD to be upside down TODO figure out how to use this without race conditions from setup()
    syspower.set_mode(OUTPUT); // Then set the put as an output as normal.
    syspower.digital_write(HIGH);

}