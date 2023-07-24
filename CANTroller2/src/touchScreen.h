#ifdef CAP_TOUCH
    #include <Adafruit_FT6206.h>  // For interfacing with the capacitive touchscreen controller chip
#else
    #include <XPT2046_Touchscreen.h>
#endif

#define touch_cs_pin 39  // Use as chip select for resistive touchscreen
#define touch_irq_pin 255  // Input, optional touch occurence interrupt signal (for resistive touchscreen, prevents spi bus delays) - Set to 255 if not used

class TouchScreen {
    private:
        #ifdef CAP_TOUCH
            #include <Adafruit_FT6206.h>  // For interfacing with the cap touchscreen controller chip
            Adafruit_FT6206 _ts;  // 2.8in cap touch panel on tft lcd
        #else
            #include <XPT2046_Touchscreen.h>
            XPT2046_Touchscreen _ts;  // 3.2in resistive touch panel on tft lcd
            // XPT2046_Touchscreen ts (touch_cs_pin);  // 3.2in resistive touch panel on tft lcd
        #endif

    public:
        #ifdef CAP_TOUCH
            TouchScreen() : _ts() {}
        #else
            TouchScreen(uint8_t csPin, uint8_t irqPin) : _ts(csPin, irqPin) {}
        #endif

        void init() {
            _ts.begin();
            // _ts.setRotation(1); do we need to rotate?
        }

        bool touched() {
            return _ts.touched(); 
        }

        TS_Point getPoint() {
            return _ts.getPoint();
        }

        int16_t getX() {
            return getPoint().x;
        }

        int16_t getY() {
            return getPoint().y; 
        }
};