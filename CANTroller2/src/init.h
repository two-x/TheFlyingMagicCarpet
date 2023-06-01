#ifndef INIT_H
#define INIT_H
#include <stdio.h>
#ifdef DUE
    #include <LibPrintf.h>  // This works on Due but not ESP32
#endif
#include "Arduino.h"
#include "devices.h"
#include "spid.h"
#include "ui.h"
#include "globals.h"

// #ifdef ESP32_SX_DEVKIT
    // // library landing page: https://api.riot-os.org/group__cpu__esp__common.html
    // #include "types.h"  // 
    // #include "architecture_arch.h"  // 
    // #include "atomic_utils_arch.h"  // 
    // #include "cpu.h"  // 
    // #include "esp_common.h"  // 
    // #include "esp_common_log.h"  // 
    // #include "exceptions.h"  // 
    // #include "gpio_arch_common.h"  // 
    // #include "irq_arch_common.h"  // irqs
    // #include "log_module.h"  // 
    // #include "syscalls_common.h"  // 
    // #include "thread_arch.h"  // threads
    // #include "tools.h"  // 
    // #include "xtensa_conf.h"  //
    // // module common: https://api.riot-os.org/group__cpu__esp__common__conf.html
    // #include "esp_wifi_params.h"  //
    // #include "cpu_conf_common.h"  //
    // // module esp_now: https://api.riot-os.org/group__cpu__esp__common__esp__wifi.html   
    // #include "esp_wifi_netdev.h"  //
    // #include "esp_wifi_params.h"  //
    // // module wifi netdev: https://api.riot-os.org/group__cpu__esp__common__esp__now.html
// #endif

// esp32 examples:
// Sensor examples: https://api.riot-os.org/group__drivers__sensors.html
// SD card example: https://api.riot-os.org/group__config__drivers__storage.html
// Wifi radio docs: https://api.riot-os.org/group__config__drivers__netdev.html
// Servo examples: https://api.riot-os.org/group__drivers__servo.html
// Rgb led example: https://api.riot-os.org/group__drivers__rgbled.html
// Onboard leds: https://api.riot-os.org/group__drivers__led.html
// Pulse counter: https://api.riot-os.org/group__drivers__pulse__counter.html
// Arduino Due vs ESP32: https://api.riot-os.org/dir_cd0ef0289302642cf5c8e6bdd1295777.html
// ILI9341 driver: https://api.riot-os.org/group__drivers__ili9341.html

// Defines for all the GPIO pins we're using
#ifdef ESP32_SX_DEVKIT
    #define button_pin 0  // (button0 / strap to 1) - This is one of the little buttons on the esp32 board. Active low
    #define joy_horz_pin 1  // (adc) - Analog input, tells us left-right position of joystick. Take complement of ADC value gives:  Low values for left, High values for right.
    #define joy_vert_pin 2  // (adc) - Analog input, tells us up-down position of joystick. Take complement of ADC value gives:  Low values for down, High values for up.
    #define tft_dc_pin 3  // (strap X) - Output, Assert when sending data to display chip to indicate commands vs. screen data
    #define battery_pin 4  // (adc) -  Analog input, mule battery voltage level, full scale is 15.638V
    #define pot_wipe_pin 5  // (adc) - Analog input, tells us position of attached potentiometer (useful for debug, etc.)
    #define brake_pos_pin 6  // (adc) - Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
    #define pressure_pin 7  // (adc) - Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
    #define i2c_sda_pin 8  // (i2c0 sda / adc) - Hijack these pins for the touchscreen and micro-sd i2c bus
    #define i2c_scl_pin 9  // (i2c0 scl / adc) -  Hijack these pins for the touchscreen and micro-sd i2c bus
    #define tft_cs_pin 10  // (spi2 cs0) -  Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
    #define tft_mosi_pin 11  // (spi2 mosi) - Used as spi interface to tft screen
    #define tft_sclk_pin 12  // (spi2 sclk) - Used as spi interface to tft screen
    #define tft_miso_pin 13  // (spi2 miso) - Used as spi interface to tft screen
    #define steer_pwm_pin 14  // (pwm0) - Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
    #define brake_pwm_pin 15  // (pwm1) - Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
    #define gas_pwm_pin 16  // (pwm1) - Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
    #define hotrc_horz_pin 17  // (pwm0 / tx1) - Hotrc thumb joystick input.
    #define hotrc_vert_pin 18  // (pwm0 / rx1) - Hotrc bidirectional trigger input
    #define tp_irq_pin 19  // (usb-otg) - Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
    #define hotrc_ch4_pin 20  // (usb-otg) - Hotrc Ch3 toggle output, used to toggle cruise mode
    #define hotrc_ch3_pin 21  // (pwm0) - Hotrc Ch4 toggle output, used to panic stop & kill ignition
    // #define gpio26_pin 26  // (spi-ram / oct-spi) - This pin is listed in pin summary, but is it available on board?
    // #define gpio33_pin 33  // (spi-ram / oct-spi) - This pin is listed in pin summary, but is it available on board?
    // #define gpio34_pin 34  // (spi-ram / oct-spi) - This pin is listed in pin summary, but is it available on board?
    #define tach_pulse_pin 35  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
    #define speedo_pulse_pin 36  // (spi-ram / oct-spi) - Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
    #define ignition_pin 37  // (spi-ram / oct-spi) - Output flips a relay to kill the car ignition, active high (no pullup)
    #define basicmodesw_pin 38  // (spi-ram / oct-spi) - Input, asserted to tell us to run in basic mode.   (needs pullup)
    #define encoder_sw_pin 39  // Int input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
    #define encoder_b_pin 40  // Int input, The B pin (aka DT pin) of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
    #define encoder_a_pin 41  // Int input, The A pin (aka CLK pin) of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
    #define heartbeat_led_pin 42  // There is not actually an LED here, so this is basically a free pin
    #define uart_tx_pin 43  // (tx0) - Reserve for possible improved jaguar interface
    #define uart_rx_pin 44  // (rx0) - Reserve for possible improved jaguar interface
    #define tft_ledk_pin 45  // (strap to 0) - Input, momentary button low pulse >500ms in fly mode means start cruise mode. Any pulse in cruise mode goes to fly mode. Active low. (needs pullup)
    #define cruise_sw_pin 46  // (strap X) - Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
    #define usd_cs_pin 47  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
    #define neopixel_pin 48  // (rgb led) - Data line to onboard Neopixel WS281x

    #define pot_pwr_pin -1  // Unused on esp32
    #define led_rx_pin -1  // Unused on esp32
    #define led_tx_pin -1  // Unused on esp32

    // gpio27-32 used by spi flash, unavailable. gpio22-25 mystery? All gpio can pup/pdn. gpio 1-20 can adc. gpio 0-21 can work in deep-sleep.
    // Pin summary page: https://api.riot-os.org/group__cpu__esp32__esp32s3.html#esp32_gpio_pins_esp32s3
    // ESP32 common Landing page: https://api.riot-os.org/group__cpu__esp__common.html
    // S3 features Landing page of functions: https://api.riot-os.org/group__boards__esp32s3__devkit.html
    // Things to research & add below.   (Need to study and implement initialization options described here: https://api.riot-os.org/group__cpu__esp32.html#esp32_adc_channels )
    // wifi :
    // i2c: run i2c_init(...) and i2c_init_pins(...) 
    // fast i2c : #I2C0_SPEED is I2C_SPEED_FAST (400kbps) by default. Need I2C_SPEED_NORMAL (100kbps) (disable fast mode for tft compatibility?).  See https://api.riot-os.org/group__drivers__periph__i2c.html#ga9f14916eda80b19ff41d08e25eee56fb
    // timer0 and timer2 use for hotrc / pwm-outs
    // threads : thread_arch.h
    // irqs : irq_arch_common.h

    // strap pin summary:   ( see https://api.riot-os.org/group__cpu__esp32__esp32s3.html#esp32_gpio_pins_esp32s3 )
    // gpio0 : (!! need float or high at boot !!) - If 1 (default), boot from flash and ignore gpio 46. If 0 then gpio46 is read  (pulled up by default)
    // gpio46 : If 1 (and gpio0 = 0), update firmware mode. If 0 ?. Is dont-care as long as gpio0 is assured to be 1 at boot (pulled down by default)
    // gpio3 : Dont-care assuming efuse-...-jtag are not set, otherwise it allows choice of interface
    // gpio45 : (!! need float or low at boot !!) - Select spi voltage level: 0 = 3.3v, 1 = 1.8v (pulled down by default)

#else  // Applies to Due
    #define usd_cs_pin 4  // Output, active low, Chip select allows SD card controller chip use of the SPI bus
    #define tft_ledk_pin 5  // Output, Optional PWM signal to control brightness of LCD backlight (needs modification to shield board to work)
    #define tp_irq_pin 7  // Optional int input so touchpanel can interrupt us (need to modify shield board for this to work)
    #define tft_dc_pin 9  // Output, Assert when sending data to display chip to indicate commands vs. screen data
    #define tft_cs_pin 10  // Output, active low, Chip select allows ILI9341 display chip use of the SPI bus
    #define heartbeat_led_pin 13  // Output, This is the LED labeled "L" onboard the arduino due.  Active high.
    #define encoder_sw_pin 18  // Int input, Encoder above, for the UI.  This is its pushbutton output, active low (needs pullup)
    #define encoder_b_pin 19  // Int input, The B pin (aka DT pin) of the encoder. Both A and B complete a negative pulse in between detents. If B pulse goes low first, turn is CW. (needs pullup)
    #define encoder_a_pin 21  // Int input, The A pin (aka CLK pin) of the encoder. Both A and B complete a negative pulse in between detents. If A pulse goes low first, turn is CCW. (needs pullup)
    #define speedo_pulse_pin 23  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per driven pulley rotation. Open collector sensors need pullup)
    #define tach_pulse_pin 25  // Int Input, active high, asserted when magnet South is in range of sensor. 1 pulse per engine rotation. (no pullup)
    #define pot_pwr_pin 27  // Output, Lets us supply the optional external potentiometer with 3.3V power
    #define steer_pwm_pin 29  // Output, PWM signal positive pulse width sets steering motor speed from full left to full speed right, (50% is stopped). Jaguar asks for an added 150ohm series R when high is 3.3V
    #define neopixel_pin 31  // Output, no neopixel for due
    #define hotrc_horz_pin 35
    #define hotrc_vert_pin 37
    #define hotrc_ch3_pin 39
    #define hotrc_ch4_pin 41
    #define brake_pwm_pin 43  // Output, PWM signal duty cycle sets speed of brake actuator from full speed extend to full speed retract, (50% is stopped) 
    #define gas_pwm_pin 45  // Output, PWM signal duty cycle controls throttle target. On Due this is the pin labeled DAC1 (where A13 is on Mega)
    #define basicmodesw_pin 47  // Input, asserted to tell us to run in basic mode.   (needs pullup)
    #define ignition_pin 49  // Output flips a relay to kill the car ignition, active high (no pullup)
    #define cruise_sw_pin 51  // Input, momentary button low pulse >500ms in fly mode means start cruise mode. Any pulse in cruise mode goes to fly mode. Active low. (needs pullup)
    #define led_rx_pin 72  // Another on-board led
    #define led_tx_pin 73  // Another on-board led
    #define pot_wipe_pin A6  // Analog input, tells us position of attached potentiometer (useful for debug, etc.)
    #define battery_pin A7  // Analog input, mule battery voltage level, full scale is 15.638V
    #define joy_horz_pin A8  // Analog input, tells us left-right position of joystick. Take complement of ADC value gives:  Low values for left, High values for right.
    #define joy_vert_pin A9  // Analog input, tells us up-down position of joystick. Take complement of ADC value gives:  Low values for down, High values for up.
    #define pressure_pin A10  // Analog input, tells us brake fluid pressure. Needs a R divider to scale max possible pressure (using foot) to 3.3V.
    #define brake_pos_pin A11  // Analog input, tells us linear position of brake actuator. Blue is wired to ground, POS is wired to white.
#endif

void set_pin(int32_t pin, int32_t mode) { if (pin >= 0) pinMode(pin, mode); }  // allows me to set unused pins to -1 depending on board to avoid errors

void cantroller2_init() {
    set_pin(encoder_a_pin, INPUT_PULLUP);
    set_pin(encoder_b_pin, INPUT_PULLUP);
    set_pin(encoder_sw_pin, INPUT_PULLUP);
    set_pin(brake_pwm_pin, OUTPUT);
    set_pin(steer_pwm_pin, OUTPUT);
    set_pin(gas_pwm_pin, OUTPUT);
    set_pin(ignition_pin, OUTPUT);  // drives relay to turn on/off car. Active high
    set_pin(basicmodesw_pin, INPUT_PULLUP);
    set_pin(tach_pulse_pin, INPUT_PULLUP);
    set_pin(speedo_pulse_pin, INPUT_PULLUP);
    set_pin(hotrc_horz_pin, INPUT);
    set_pin(hotrc_vert_pin, INPUT);
    set_pin(hotrc_ch3_pin, INPUT);
    set_pin(hotrc_ch4_pin, INPUT);
    set_pin(joy_horz_pin, INPUT);
    set_pin(joy_vert_pin, INPUT);
    set_pin(pressure_pin, INPUT);
    set_pin(brake_pos_pin, INPUT);
    set_pin(battery_pin, INPUT);
    set_pin(pot_wipe_pin, INPUT);
    set_pin(usd_cs_pin, OUTPUT);
    set_pin(tft_cs_pin, OUTPUT);
    set_pin(tft_dc_pin, OUTPUT);
    set_pin(heartbeat_led_pin, OUTPUT);
    set_pin(neopixel_pin, OUTPUT);
    set_pin(tft_ledk_pin, OUTPUT);
    set_pin(pot_pwr_pin, OUTPUT);
    set_pin(cruise_sw_pin, INPUT_PULLUP);
    set_pin(tp_irq_pin, INPUT_PULLUP);
    set_pin(led_rx_pin, OUTPUT);
    set_pin(led_tx_pin, OUTPUT);
    
    digitalWrite(ignition_pin, ignition);
    digitalWrite(tft_cs_pin, HIGH);   // Prevent bus contention
    digitalWrite(usd_cs_pin, HIGH);   // Prevent bus contention
    digitalWrite(tft_dc_pin, LOW);
    if (led_rx_pin >= 0) digitalWrite(led_rx_pin, LOW);  // Light up
    if (led_tx_pin >= 0) digitalWrite(led_tx_pin, HIGH);  // Off
    if (pot_pwr_pin >= 0) digitalWrite(pot_pwr_pin, HIGH);

    analogReadResolution(adc_bits);  // Set Arduino Due to 12-bit resolution (default is same as Mega=10bit)
    Serial.begin(115200);  // Open serial port
    // printf("Serial port open\n");  // This works on Due but not ESP32
    
    delay(500); // This is needed to allow the screen board enough time after a cold boot before we start trying to talk to it.
    if (display_enabled) {
        Serial.print(F("Init LCD... "));
        tft.begin();
        tft.setRotation(1);  // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
        for (int32_t lineno=0; lineno <= arraysize(telemetry); lineno++)  {
            disp_age_quanta[lineno] = -1;
            memset(disp_values[lineno],0,strlen(disp_values[lineno]));
        }
        for (int32_t row=0; row<arraysize(disp_bool_values); row++) disp_bool_values[row] = 1;
        for (int32_t row=0; row<arraysize(disp_needles); row++) disp_needles[row] = -5;  // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
        for (int32_t row=0; row<arraysize(disp_targets); row++) disp_targets[row] = -5;  // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen

        tft.fillScreen(BLK);  // Black out the whole screen
        draw_fixed(false);
        draw_touchgrid(false);
        Serial.println(F("Success"));

        Serial.print(F("Captouch initialization... "));
        if (! touchpanel.begin(40)) {     // pass in 'sensitivity' coefficient
            Serial.println(F("Couldn't start FT6206 touchscreen controller"));
            // while (1);
        }
        else Serial.println(F("Capacitive touchscreen started"));
    }
    strip.begin();  // start datastream
    strip.show();  // Turn off the pixel
    strip.setBrightness(50);  // It truly is incredibly bright
    // 230417 removing sdcard init b/c boot is hanging here unless I insert this one precious SD card
    // Serial.print(F("Initializing filesystem...  "));  // SD card is pretty straightforward, a single call. 
    // if (! sd.begin(usd_cs_pin, SD_SCK_MHZ(25))) {   // ESP32 requires 25 mhz limit
    //     Serial.println(F("SD begin() failed"));
    //     for(;;); // Fatal error, do not continue
    // }
    // sd_init();
    // Serial.println(F("Filesystem started"));
    attachInterrupt(digitalPinToInterrupt(tach_pulse_pin), tach_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(speedo_pulse_pin), speedo_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder_a_pin), encoder_a_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder_b_pin), encoder_b_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hotrc_horz_pin), hotrc_horz_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hotrc_vert_pin), hotrc_vert_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(hotrc_ch3_pin), hotrc_ch3_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(hotrc_ch4_pin), hotrc_ch4_isr, FALLING);    
    // Set up the soren pid loops
    brakeSPID.set_output_center(brake_pulse_stop_us);  // Sets actuator centerpoint and puts pid loop in output centerpoint mode. Becasue actuator value is defined as a deviation from a centerpoint
    brakeSPID.set_input_limits(d_pressure_min_adc, d_pressure_max_adc);  // Make sure pressure target is in range
    brakeSPID.set_output_limits((double)brake_pulse_retract_us, (double)brake_pulse_extend_us);
    gasSPID.set_input_limits((double)engine_idle_rpm, (double)engine_govern_rpm);
    gasSPID.set_output_limits((double)gas_pulse_redline_us, (double)gas_pulse_idle_us);
    cruiseSPID.set_input_limits((double)carspeed_idle_mmph, (double)carspeed_redline_mmph);
    cruiseSPID.set_output_limits((double)engine_idle_rpm, (double)engine_govern_rpm);
      
    steer_servo.attach(steer_pwm_pin);
    gas_servo.attach(gas_pwm_pin);

    // for (int32_t x; x<arraysize(ui_pot_addrs); x++) ui_pot_addrs[x] = &pot_adc;
    // ui_pot_addrs[BRKPOS] = &brake_pos_adc;
    // ui_pot_addrs[PRESS] = &pressure_adc;
    // ui_pot_addrs[TACH] = &engine_rpm;
    // ui_pot_addrs[SPEEDO] = &carspeed_mmph;
    
    //  = { &pot_adc, &pot_adc, &pot_adc, &pot_adc, &brake_pos_adc, &pressure_adc, &engine_rpm, &carspeed_mmph, -1, -1, -1 };

    // carspeed_mmph = (int32_t)(179757270/(double)speedo_delta_us); // Update car speed value  
    // magnets/us * 179757270 (1 rot/magnet * 1000000 us/sec * 3600 sec/hr * 1/19.85 gearing * 20*pi in/rot * 1/12 ft/in * 1000/5280 milli-mi/ft gives milli-mph  // * 1/1.15 knots/mph gives milliknots
    // Mule gearing:  Total -19.845x (lo) ( Converter: -3.5x to -0.96x Tranny -3.75x (lo), -1.821x (hi), Final drive -5.4x )
    Speedo.set_conversion_factor( (double)179757270 );
    Speedo.set_ema_alpha(carspeed_ema_alpha);
    Speedo.set_lp_spike_thresh(carspeed_lp_thresh_mmph, carspeed_spike_thresh_mmph);
    Pressure.set_limits(d_pressure_min_adc, d_pressure_max_adc);

    loopTimer.reset();  // start timer to measure the first loop
    Serial.println(F("Setup finished"));
}

#endif  // INIT_H
