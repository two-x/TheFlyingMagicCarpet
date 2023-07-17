#include "pin.h"
#include "Arduino.h"

Pin::Pin(PinNumber pin_num) {
  set_pin(pin_num);
}

Pin::Pin(PinNumber pin_num, uint8_t mode) {
  set_pin(pin_num);
  set_mode(mode); 
}

void Pin::set_pin(PinNumber pin_num) {
  validate_pin_number(static_cast<int>(pin_num));  
  pin_num_ = pin_num;
}

void Pin::set_mode(uint8_t mode) {
  ::pinMode(static_cast<uint8_t>(pin_num_), mode);
}

void Pin::digital_write(uint8_t value) {
  ::digitalWrite(static_cast<uint8_t>(pin_num_), value);
}

uint8_t Pin::digital_read() const {
  return ::digitalRead(static_cast<uint8_t>(pin_num_)); 
}

void Pin::attach_interrupt(void (*isr)(), int mode) {
  ::attachInterrupt(digitalPinToInterrupt(static_cast<uint8_t>(pin_num_)), isr, mode);
}

void Pin::validate_pin_number(int pin_num) {
  if(pin_num < -1 || pin_num > NUM_PINS){
      throw std::invalid_argument("Pin number out of range");
  }
}
