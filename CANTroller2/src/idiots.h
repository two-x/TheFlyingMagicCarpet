#pragma once
#include <NeoPixelBus.h>
#include "globals.h"

#define neorgb_t RgbColor
#define idiot_light_led_count 7  // NeoPixel LEDs 3-9 mirror the first 7 on-screen idiot lights

// Color format conversion functions (332 = 3R 3G 2B, 565 = 5R 6G 5B, 888 = 8R 8G 8B, neo = RgbColor)
uint32_t color_to_888(uint16_t color565) { return (static_cast<uint32_t>(color565 & 0xf800) << 8) | (static_cast<uint32_t>(color565 & 0x7e0) << 5) | (static_cast<uint32_t>(color565 & 0x1f) << 3); }
uint32_t color_to_888(uint8_t color332) { return (static_cast<uint32_t>(color332 & 0xe0) << 16) | (static_cast<uint32_t>(color332 & 0x1c) << 11) | (static_cast<uint32_t>(color332 & 0x3) << 6); }
uint32_t color_to_888(neorgb_t colorneo) { return (static_cast<uint32_t>(colorneo.R) << 16) | (static_cast<uint32_t>(colorneo.G) << 8) | static_cast<uint32_t>(colorneo.B); }
uint16_t color_to_565(uint32_t color888) { return static_cast<uint16_t>(((color888 & 0xf80000) >> 8) | ((color888 & 0xfc00) >> 5) | ((color888 & 0xf8) >> 3)); }
uint16_t color_to_565(uint8_t color332) { return ((static_cast<uint16_t>(color332) & 0xe0) << 8) | ((static_cast<uint16_t>(color332) & 0x1c) << 6) | ((static_cast<uint16_t>(color332) & 0x3) << 3); }
uint16_t color_to_565(neorgb_t colorneo) { return ((static_cast<uint16_t>(colorneo.R) & 0xf8) << 8) | ((static_cast<uint16_t>(colorneo.G) & 0xfc) << 3) | (((static_cast<uint16_t>(colorneo.B) & 0xf8) >> 3)); }
uint8_t color_to_332(uint16_t color565) { return static_cast<uint8_t>(((color565 & 0xe000) >> 8) | ((color565 & 0x700) >> 6) | ((color565 & 0x18) >> 3)); }
uint8_t color_to_332(uint32_t color888) { return static_cast<uint8_t>(((color888 & 0xe00000) >> 16) | ((color888 & 0xe000) >> 11) | ((color888 & 0xc0) >> 6)); }
uint8_t color_to_332(neorgb_t colorneo) { return (colorneo.R & 0xe0) | ((colorneo.G & 0xe0) >> 3) | ((colorneo.B & 0xc0) >> 6); }
neorgb_t color_to_neo(uint32_t color888) { return neorgb_t((color888 >> 16) & 0xff, (color888 >> 8) & 0xff, color888 & 0xff); }
neorgb_t color_to_neo(uint16_t color565) { return neorgb_t((color565 & 0xf800) >> 8, (color565 & 0x7e0) >> 3, (color565 & 0x1f) << 3); }
neorgb_t color_to_neo(uint8_t color332) { return neorgb_t(color332 & 0xe0, (color332 & 0x1c) << 3, (color332 & 0x3) << 6); }

class IdiotLights {
  public:
    IdiotLights() { for (int i=0; i<iconcount; i++) lastvals[i] = *vals[i]; }
    void setup() {
        ezread.squintf(ezread.highlightcolor, "Idiot lights: %d icons & %d neopix hazards\n", iconcount, idiot_light_led_count);
    }
    int row_count() { return 12; }
    int row_height() { return 11; }
    int num_idiots() { return iconcount; }
    bool val(int index) { return *vals[index]; }
    bool lastval(int index) { return lastvals[index]; }
    bool* val_ptr(int index) { return vals[index]; }
    bool* lastval_ptr(int index) { return &lastvals[index]; }
    uint8_t color(int index, int state=On) { return colors[state][index]; }
    std::string letter_string(int index) { return letter_strings[index]; }
  private:
    static constexpr int iconcount = 36;
    bool lastvals[iconcount];
    uint8_t colors[2][iconcount] = {
        { 0x21, 0x41, 0x61, 0x40, 0x44, 0x68, 0x48, 0x28, 0x08, 0x09, 0x05, 0x01,
          0x21, 0x41, 0x61, 0x40, 0x44, 0x68, 0x48, 0x28, 0x08, 0x09, 0x05, 0x01,
          0x21, 0x41, 0x61, 0x40, 0x44, 0x68, 0x48, 0x28, 0x08, 0x09, 0x05, 0x01, }, // [Off] dim colors
        { 0x63, 0xa3, 0xc2, 0xc0, 0xec, 0xf4, 0xd8, 0x9c, 0x1d, 0x1b, 0x13, 0x0b,
          0x63, 0xa3, 0xc2, 0xc0, 0xec, 0xf4, 0xd8, 0x9c, 0x1d, 0x1b, 0x13, 0x0b,
          0x63, 0xa3, 0xc2, 0xc0, 0xec, 0xf4, 0xd8, 0x9c, 0x1d, 0x1b, 0x13, 0x0b, }  // [On] bright colors
    };
    std::string letter_strings[iconcount] = {
        "SL", "SR", "\xf7""E", "\xf7""W", "RC", "P\x13", "SI", "Pk", "AS", "AH", "Aj", "HM",
        "St", "FP", "Pn", "NF", "SM", "TM", "Bt", "NT", "eA", "WD", "Dv", "Pw",
        "Th", "Br", "St", "RC", "Sp", "Tc", "Pr", "Ps", "Tm", "Bt", "Ot", "IO",
    };
    bool* vals[iconcount] = {
        // row 1 onscreen.  the 1st 7 of these are true hazard lights (lit only on error), also copied onto the last 7 neopixel idiot lights
        &diag.err_sens_alarm[ErrLost], &diag.err_sens_alarm[ErrRange], &diag.eng_alarm_active, &diag.brake_alarm_active, &diag.wheel_alarm_active, &panicstop,
        hotrc.radiolost_ptr(), hotrc.radiolost_untested_ptr(), &brake.autostopping, &brake.autoholding, &parking, &releasing,
        // row 2 onscreen.  these are all just informational values
        &cruise_adjusting, &car_hasnt_moved, &starter.motor, &brake.posn_pid_active, &brake.no_feedback, speedo.pin_level_ptr(),
        tach.pin_level_ptr(), speedo.stopped_ptr(), tach.stopped_ptr(), &nowtouch, &shutting_down, &running_on_devboard,
        // row 3 onscreen.  this row has one light per actuator or sensor, lit when there's any kind of error w/ that device
        &sensidiots[_Throttle], &sensidiots[_BrakeMotor], &sensidiots[_SteerMotor], &sensidiots[_HotRC], &sensidiots[_Speedo], &sensidiots[_Tach],
        &sensidiots[_BrakePres], &sensidiots[_BrakePosn], &sensidiots[_Temps], &diag.battrangeerr, &sensidiots[_GPIO], &sensidiots[_Other],
    };
  public:
    uint8_t icon[iconcount][11] = {
        // row 1
        { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x3e, 0x71, 0x59, 0x4d, 0x47, 0x3e, },  // "S" w/ crossout symbol       // &diag.err_sens_alarm[ErrLost]
        { 0x6e, 0x6b, 0x6b, 0x3b, 0x00, 0x78, 0x70, 0x59, 0x4d, 0x07, 0x0f, },  // "S" w/ double arrow          // &diag.err_sens_alarm[ErrRange]
        { 0x7f, 0x7f, 0x6b, 0x6b, 0x00, 0x70, 0x10, 0x10, 0x77, 0x65, 0x07, },  // "En" w/ degree symbol        // &diag.err_sens[ErrRange][_TempEng]
        { 0x7f, 0x49, 0x49, 0x36, 0x36, 0x00, 0x78, 0x10, 0x37, 0x25, 0x07, },  // "Br" w/ degree symbol        // &diag.err_sens[ErrRange][_TempBrake]
        { 0x7f, 0x30, 0x18, 0x30, 0x7f, 0x00, 0x7e, 0x10, 0x77, 0x65, 0x07, },  // "Wh" w/ degree symbol        // &wheeltemperr
        { 0x7f, 0x7f, 0x09, 0x09, 0x77, 0x16, 0x70, 0x60, 0x00, 0x6f, 0x6f, },  // "Pn!"                        // &panicstop
        { 0x7a, 0x7f, 0x4f, 0x17, 0x06, 0x00, 0x22, 0x1c, 0x00, 0x41, 0x3e, },  // hotrc w/ radio waves         // hotrc.radiolost_ptr()
        { 0x7a, 0x7f, 0x4f, 0x17, 0x06, 0x00, 0x02, 0x03, 0x51, 0x5d, 0x06, },  // hotrc w/ "?"                 // hotrc.radiolost_untested_ptr()
        { 0x3e, 0x49, 0x1c, 0x00, 0x6e, 0x6b, 0x3b, 0x00, 0x1c, 0x49, 0x3e, },  // brake assembly w/ "S"        // &brake.autostopping
        { 0x3e, 0x49, 0x1c, 0x00, 0x7f, 0x18, 0x7f, 0x00, 0x1c, 0x49, 0x3e, },  // brake assembly w/ "H"        // &brake.autoholding
        { 0x3e, 0x63, 0x01, 0x7d, 0x7d, 0x15, 0x15, 0x1d, 0x09, 0x63, 0x3e, },  // opencircle-"P"               // &parking
        { 0x3e, 0x63, 0x01, 0x7d, 0x7d, 0x15, 0x35, 0x5d, 0x09, 0x63, 0x3e, },  // opencircle-"R"               // &releasing
        // row 2
        { 0x08, 0x1c, 0x36, 0x00, 0x3e, 0x63, 0x63, 0x00, 0x36, 0x1c, 0x08, },  // "<C>"                        // &cruise_adjusting
        { 0x3d, 0x43, 0x07, 0x00, 0x3e, 0x63, 0x55, 0x49, 0x55, 0x63, 0x3e, },  // rotation arrow w/ X wheel    // &car_hasnt_moved
        { 0x3e, 0x41, 0x7f, 0x7b, 0x7b, 0x7b, 0x3e, 0x1c, 0x7f, 0x55, 0x7f, },  // motor w/ spur gear           // &starter.motor
        { 0x7c, 0x46, 0x7f, 0x7f, 0x33, 0x12, 0x12, 0x12, 0x1e, 0x12, 0x0c, },  // linear actuator or schlong   // &brake.posn_pid_active
        { 0x7f, 0x49, 0x49, 0x36, 0x00, 0x63, 0x14, 0x1c, 0x36, 0x22, 0x3e, },  // "B" w/ open loop             // &brake.no_feedback
        { 0x63, 0x63, 0x63, 0x63, 0x77, 0x3e, 0x1c, 0x00, 0x6e, 0x6b, 0x3b, },  // "S" magnet w/ zap            // speedo.pin_level_ptr()
        { 0x63, 0x63, 0x63, 0x63, 0x77, 0x3e, 0x1c, 0x00, 0x06, 0x7e, 0x06, },  // "T" magnet w/ zap            // tach.pin_level_ptr()
        { 0x22, 0x36, 0x1c, 0x08, 0x1c, 0x36, 0x22, 0x00, 0x6e, 0x6b, 0x3b, },  // "S" w/ X                     // speedo.stopped_ptr()
        { 0x22, 0x36, 0x1c, 0x08, 0x1c, 0x36, 0x22, 0x00, 0x06, 0x7e, 0x06, },  // "T" w/ X                     // tach.stopped_ptr()
        { 0x78, 0x7c, 0x7f, 0x7f, 0x7c, 0x7c, 0x1c, 0x0c, 0x0c, 0x0c, 0x0c, },  // finger                       // &nowtouch
        { 0x16, 0x15, 0x0d, 0x60, 0x6f, 0x04, 0x6f, 0x60, 0x0f, 0x69, 0x66, },  // "SHD..."                     // &shutting_down
        { 0x7f, 0x63, 0x3e, 0x00, 0x7f, 0x6b, 0x6b, 0x00, 0x7f, 0x30, 0x1f, },  // "DEV"                        // &running_on_devboard
        // row 3
        { 0x3e, 0x63, 0x7b, 0x00, 0x7e, 0x13, 0x7e, 0x00, 0x6e, 0x6b, 0x3b, },  // "GAS"                        // &sensidiots[_Throttle]
        { 0x3e, 0x49, 0x08, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x08, 0x49, 0x3e, },  // brakes or tie fighter        // &sensidiots[_BrakeMotor]
        { 0x00, 0x1c, 0x26, 0x45, 0x49, 0x79, 0x49, 0x45, 0x26, 0x1c, 0x00, },  // steering wheel               // &sensidiots[_SteerMotor]
        { 0x00, 0x00, 0x00, 0x7a, 0x7f, 0x7f, 0x4f, 0x17, 0x06, 0x00, 0x00, },  // hotrc                        // &sensidiots[_HotRC]
        { 0x60, 0x66, 0x06, 0x10, 0x30, 0x63, 0x43, 0x00, 0x6e, 0x6b, 0x3b, },  // gauge "S"                    // &sensidiots[_Speedo]
        { 0x60, 0x66, 0x06, 0x10, 0x30, 0x63, 0x43, 0x00, 0x06, 0x7e, 0x06, },  // gauge "T"                    // &sensidiots[_Tach]
        { 0x42, 0x24, 0x2f, 0x44, 0x42, 0x20, 0x22, 0x44, 0x4f, 0x24, 0x22, },  // pressure waves               // &sensidiots[_BrakePres]
        { 0x7e, 0x42, 0x46, 0x42, 0x4e, 0x42, 0x46, 0x42, 0x4e, 0x42, 0x7e, },  // ruler                        // &sensidiots[_BrakePosn]
        { 0x02, 0x07, 0x35, 0x77, 0x7a, 0x1c, 0x0e, 0x1f, 0x13, 0x06, 0x04, },  // thermometer                  // &sensidiots[_Temps]
        { 0x7e, 0x77, 0x63, 0x77, 0x7e, 0x7e, 0x7e, 0x77, 0x77, 0x77, 0x7e, },  // car battery                  // &diag.battrangeerr
        { 0x2a, 0x2a, 0x2a, 0x7f, 0x7d, 0x7f, 0x7f, 0x7f, 0x2a, 0x2a, 0x2a, },  // chip                         // &sensidiots[_GPIO]
        { 0x7f, 0x6b, 0x6b, 0x00, 0x03, 0x7f, 0x03, 0x00, 0x3e, 0x63, 0x63, },  // "ETC"                        // &sensidiots[_Other]
    };
};
