#pragma once
#include <iostream>
#include <string>
#include <iomanip>

#if defined (ARDUINO_WIO_TERMINAL)
    #include <Seeed_FS.h>
    #include <SD/Seeed_SD.h>
#else
    #include <SD.h>
    #include <SPIFFS.h>
#endif
#include <LovyanGFX.hpp>

class SdCard {
  private:
    LGFX* lcd;
    Timer capture_timer{5000000};
    int capcount = 0, capmax = 100;
    char filenamebase[11] = "/screencap";
    char extension[5] = ".bmp";
    #ifndef SDCARD_SPI
        #define SDCARD_SPI SPI
    #endif
    std::string filename_seq(int num) {
        std::string writefile = filenamebase + std::to_string(num);
        writefile += std::string(2 - std::to_string(num).length(), '0'); // Add leading zeros
        return writefile + extension;
    }
    bool saveToSD_16bit(std::string filename) {
        bool result = false;
        File file = SD.open(filename.c_str(), "w");
        if (file) {
            int width  = lcd->width();
            int height = lcd->height();

            int rowSize = (2 * width + 3) & ~ 3;

            lgfx::bitmap_header_t bmpheader;
            bmpheader.bfType = 0x4D42;
            bmpheader.bfSize = rowSize * height + sizeof(bmpheader);
            bmpheader.bfOffBits = sizeof(bmpheader);

            bmpheader.biSize = 40;
            bmpheader.biWidth = width;
            bmpheader.biHeight = height;
            bmpheader.biPlanes = 1;
            bmpheader.biBitCount = 16;
            bmpheader.biCompression = 3;

            file.write((std::uint8_t*)&bmpheader, sizeof(bmpheader));
            std::uint8_t buffer[rowSize];
            memset(&buffer[rowSize - 4], 0, 4);
            for (int y = lcd->height() - 1; y >= 0; y--) {
                lcd->readRect(0, y, lcd->width(), 1, (lgfx::rgb565_t*)buffer);
                file.write(buffer, rowSize);
            }
            file.close();
            result = true;
        }
        else Serial.print("error:file open failure\n");
        return result;
    }
    bool saveToSD_24bit(std::string filename) {
        bool result = false;
        File file = SD.open(filename.c_str(), "w");
        if (file) {
            int width  = lcd->width();
            int height = lcd->height();

            int rowSize = (3 * width + 3) & ~ 3;

            lgfx::bitmap_header_t bmpheader;
            bmpheader.bfType = 0x4D42;
            bmpheader.bfSize = rowSize * height + sizeof(bmpheader);
            bmpheader.bfOffBits = sizeof(bmpheader);

            bmpheader.biSize = 40;
            bmpheader.biWidth = width;
            bmpheader.biHeight = height;
            bmpheader.biPlanes = 1;
            bmpheader.biBitCount = 24;
            bmpheader.biCompression = 0;

            file.write((std::uint8_t*)&bmpheader, sizeof(bmpheader));
            std::uint8_t buffer[rowSize];
            memset(&buffer[rowSize - 4], 0, 4);
            for (int y = lcd->height() - 1; y >= 0; y--) {
                lcd->readRect(0, y, lcd->width(), 1, (lgfx::rgb888_t*)buffer);
                file.write(buffer, rowSize);
            }
            file.close();
            result = true;
        }
        else Serial.print("error:file open failure\n");
        return result;
    }
    void draw_bmp_file(void) {
        ++capcount %= capmax;
        std::string writefile = filenamebase + std::to_string(capcount);
        writefile += std::string(3 - std::to_string(capcount).length(), '0'); // Add leading zeros
        writefile += extension;
        SD.begin();
        lcd->drawBmpFile(SD, writefile.c_str(), random(-20,20), random(-20, 20));
        SD.end();
        Serial.printf("wrote: %s\n", writefile.c_str());
    }
  public:
    SdCard(LGFX* _lcd) : lcd(_lcd) {}

    void setup(void) {
        // lcd->init();
        // Serial.begin(115200);
        // lcd->setColorDepth(16);
        // lcd->setColor(TFT_WHITE);

        // lcd->startWrite();
        // lcd->setAddrWindow(0, 0, lcd->width(), lcd->height());
        // for (int y = 0; y < lcd->height(); ++y) 
        //     for (int x = 0; x < lcd->width(); ++x) 
        //         lcd->writeColor( lcd->color888(x << 1, x + y, y << 1), 1);
        // Serial.printf("BMP save test\n");
        // lcd->endWrite();
        
        // for (int i=0; i<capmax; i++) {
        //     std::string filename = filename_seq(i);
        //     do {
        //         SD.end();
        //         delay(1000);
        //         SD.begin(sdcard_cs_pin, SDCARD_SPI, 25000000);
        //     } while (!saveToSD_16bit(filename));
        //     Serial.printf("BMP save %s success\n", filename.c_str());
        // }

            // std::string filename = filename_seq(i);
            // do {
            //     SD.end();
            //     delay(1000);
            //     SD.begin(sdcard_cs_pin, SDCARD_SPI, 25000000);
            // } while (!saveToSD_16bit(filename));
            // Serial.printf("BMP save %s success\n", filename.c_str());
    }

    void update() {
        if (capture_timer.expireset()) draw_bmp_file();
    }
};