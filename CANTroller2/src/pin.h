#ifndef PIN_H
#define PIN_H

#include <cstdint>
 
/**
 * @brief Pin class handles pin numbering and pin operations.
 */
class Pin {
public:
  Pin(PinNumber pin_num, PinMode mode);
  static const uint8_t NUM_PINS = 48; // Total number of pins on the board

  void set_mode(PinMode mode);
  void digital_write(uint8_t value);
  uint8_t digital_read() const;
  void attach_interrupt(void (*isr)(), InterruptMode mode);

private:
  PinNumber pin_num_;
  PinMode pin_mode_;

  void validate_pin_number(int pin_num);
};

void initialize_pins();

enum class PinMode
{
    INPUT = 0x01,
    OUTPUT = 0x03,
    PULLUP = 0x04,
    INPUT_PULLUP = 0x05,
    PULLDOWN = 0x08,
    INPUT_PULLDOWN = 0x09,
    OPEN_DRAIN = 0x10,
    OUTPUT_OPEN_DRAIN = 0x12,
    ANALOG = 0xC0
};

enum class InterruptMode {
  DISABLED = 0x00,
  RISING = 0x01,
  FALLING = 0x02,
  CHANGE = 0x03,
  ONLOW = 0x04,
  ONHIGH = 0x05,
  ONLOW_WE = 0x0C,
  ONHIGH_WE = 0x0D
};

enum class PinNumber
{
    DISABLED = -1,
    GPIO_NUM_0 = 0,
    GPIO_NUM_1 = 1,
    GPIO_NUM_2 = 2,
    GPIO_NUM_3 = 3,
    GPIO_NUM_4 = 4,
    GPIO_NUM_5 = 5,
    GPIO_NUM_6 = 6,
    GPIO_NUM_7 = 7,
    GPIO_NUM_8 = 8,
    GPIO_NUM_9 = 9,
    GPIO_NUM_10 = 10,
    GPIO_NUM_11 = 11,
    GPIO_NUM_12 = 12,
    GPIO_NUM_13 = 13,
    GPIO_NUM_14 = 14,
    GPIO_NUM_15 = 15,
    GPIO_NUM_16 = 16,
    GPIO_NUM_17 = 17,
    GPIO_NUM_18 = 18,
    GPIO_NUM_19 = 19,
    GPIO_NUM_20 = 20,
    GPIO_NUM_21 = 21,
    GPIO_NUM_22 = 22,
    GPIO_NUM_23 = 23,
    GPIO_NUM_24 = 24,
    GPIO_NUM_25 = 25,
    GPIO_NUM_26 = 26,
    GPIO_NUM_27 = 27,
    GPIO_NUM_28 = 28,
    GPIO_NUM_29 = 29,
    GPIO_NUM_30 = 30,
    GPIO_NUM_31 = 31,
    GPIO_NUM_32 = 32,
    GPIO_NUM_33 = 33,
    GPIO_NUM_34 = 34,
    GPIO_NUM_35 = 35,
    GPIO_NUM_36 = 36,
    GPIO_NUM_37 = 37,
    GPIO_NUM_38 = 38,
    GPIO_NUM_39 = 39,
    GPIO_NUM_40 = 40,
    GPIO_NUM_41 = 41,
    GPIO_NUM_42 = 42,
    GPIO_NUM_43 = 43,
    GPIO_NUM_44 = 44,
    GPIO_NUM_45 = 45,
    GPIO_NUM_46 = 46,
    GPIO_NUM_47 = 47,
    GPIO_NUM_48 = 48
};
#endif