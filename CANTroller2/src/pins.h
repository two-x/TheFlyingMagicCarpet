#ifndef PINS_H 
#define PINS_H

#include "pin.h"

class Pins {
public:

  Pins();
  void init();
  
  Pin button;
  Pin joy_horz; 
  Pin joy_vert;
  Pin tft_dc;
  Pin ign_batt;
  Pin pot_wipe;
  Pin brake_pos;
  Pin pressure;
  Pin i2c_sda;
  Pin i2c_scl;
  Pin tft_cs;
  Pin tft_mosi;
  Pin tft_sclk;
  Pin tft_miso; 
  Pin hotrc_ch2_vert;
  Pin hotrc_ch1_horz;
  Pin gas_pwm;
  Pin brake_pwm;
  Pin steer_pwm;
  Pin onewire;
  Pin hotrc_ch3_ign;
  Pin hotrc_ch4_cruise;
  Pin tach_pulse;
  Pin speedo_pulse;
  Pin ign_out;
  Pin syspower;
  Pin touch_cs;
  Pin encoder_b;
  Pin encoder_a;
  Pin encoder_sw;
  Pin uart_tx;
  Pin uart_rx;
  Pin starter;
  Pin basicmodesw;
  Pin sdcard_cs;
  Pin neopixel;
  Pin tft_rst;
  Pin tft_ledk;
  Pin touch_irq;
};
#endif // PINS_H