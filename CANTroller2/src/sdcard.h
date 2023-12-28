#pragma once
#include <iostream>
#include <string>
#include <iomanip>
#include <SD.h>
#include <FFat.h>
#include <LovyanGFX.hpp>

#define FORMAT_FFAT true  // You only need to format FFat the first time you use a specific card

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
        std::string writefile = filenamebase + std::string(2 - std::to_string(num).length(), '0'); // Add leading zeros
        writefile += std::to_string(num) + extension;
        return writefile;
    }
    std::string saveToSD(int num) {
        std::string filename = filename_seq(num);
        bool result = false;
        File file = SD.open(filename.c_str(), "w");
        if (file) {
            int width  = lcd->width();
            int height = lcd->height();
            int rowSize = (2 * width + 3) & ~ 3;  // int rowSize = (3 * width + 3) & ~ 3;  // 24-bit version
            lgfx::bitmap_header_t bmpheader;
            bmpheader.bfType = 0x4D42;
            bmpheader.bfSize = rowSize * height + sizeof(bmpheader);
            bmpheader.bfOffBits = sizeof(bmpheader);
            bmpheader.biSize = 40;
            bmpheader.biWidth = width;
            bmpheader.biHeight = height;
            bmpheader.biPlanes = 1;
            bmpheader.biBitCount = 16;  // bmpheader.biBitCount = 24;  // 24-bit version
            bmpheader.biCompression = 3;  // bmpheader.biCompression = 0;  // 24-bit version
            file.write((std::uint8_t*)&bmpheader, sizeof(bmpheader));
            std::uint8_t buffer[rowSize];
            memset(&buffer[rowSize - 4], 0, 4);
            for (int y = lcd->height() - 1; y >= 0; y--) {
                lcd->readRect(0, y, lcd->width(), 1, (lgfx::rgb565_t*)buffer);
                // lcd->readRect(0, y, lcd->width(), 1, (lgfx::rgb888_t*)buffer);  // 24 bit color version
                file.write(buffer, rowSize);
            }
            file.close();
            result = true;
        }
        else Serial.print("error:file open failure\n");
        return filename;
    }
    void draw_bmp_file(void) {
        ++capcount %= capmax;
        std::string wfile = filename_seq(capcount);

        // lcd->drawBmpFile(SD, wfile.c_str(), random(-20,20), random(-20, 20));  // drawBmpFile(fs, path, x, y, maxWidth, maxHeight, offX, offY, scale_x, scale_y, datum);

        Serial.printf("wrote: %s\n", wfile.c_str());
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
        // int i = 0;
        // std::string filename = filename_seq(i);
        // do {
        //     SD.end();
        //     delay(1000);
        //     SD.begin(sdcard_cs_pin, SDCARD_SPI, 25000000);
        // } while (!saveToSD(filename));
        // Serial.printf("BMP save %s success\n", filename.c_str());
        lcd->setAddrWindow(0, 0, lcd->width(), lcd->height());

    }

    void update() {
        if (capture_timer.expireset()) {
            ++capcount %= capmax;
            std::string wfile = saveToSD(capcount);
            Serial.printf("wrote: %s\n", wfile.c_str());
        }
    }
};
class FatFs {
  public:
    // This file should be compiled with 'Partition Scheme' (in Tools menu)
    // set to 'Default with ffat' if you have a 4MB ESP32 dev module or
    // set to '16M Fat' if you have a 16MB ESP32 dev module.
    FatFs() {}
    void setup() {
        // format: allocation unit size must be a power of 2, w/ min=sector_size, max=128*sector_size. setting to 0 will result in allocation unit set to the sector size. larger is faster but with more wasted space when files are small
        // sector size is always 512 B. For wear levelling, sector size is determined by CONFIG_WL_SECTOR_SIZE option of esp_vfs_fat_mount_config_t.
        Serial.setDebugOutput(true);
        if (FORMAT_FFAT) FFat.format();
        if (!FFat.begin()){
            Serial.println("fatfs mount failed\n");
            return;
        }
        Serial.println("fatfs mounted on /sd\n");
    }
    void fattest() {
        Serial.printf("Total space: %10u\n", FFat.totalBytes());
        Serial.printf("Free space: %10u\n", FFat.freeBytes());
        listDir(FFat, "/", 0);
        writeFile(FFat, "/hello.txt", "Hello ");
        appendFile(FFat, "/hello.txt", "World!\r\n");
        readFile(FFat, "/hello.txt");
        renameFile(FFat, "/hello.txt", "/foo.txt");
        readFile(FFat, "/foo.txt");
        deleteFile(FFat, "/foo.txt");
        testFileIO(FFat, "/test.txt");
        Serial.printf("Free space: %10u\n", FFat.freeBytes());
        deleteFile(FFat, "/test.txt");
        Serial.println( "Test complete" );
    }
    void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
        Serial.printf("Listing directory: %s\r\n", dirname);

        File root = fs.open(dirname);
        if (!root){
            Serial.println("- failed to open directory");
            return;
        }
        if (!root.isDirectory()){
            Serial.println(" - not a directory");
            return;
        }

        File file = root.openNextFile();
        while(file){
            if (file.isDirectory()){
                Serial.print("  DIR : ");
                Serial.println(file.name());
                if (levels) listDir(fs, file.path(), levels -1);
            }
            else {
                Serial.print("  FILE: ");
                Serial.print(file.name());
                Serial.print("\tSIZE: ");
                Serial.println(file.size());
            }
            file = root.openNextFile();
        }
    }

    void readFile(fs::FS &fs, const char * path) {
        Serial.printf("Reading file: %s\r\n", path);

        File file = fs.open(path);
        if (!file || file.isDirectory()){
            Serial.println("- failed to open file for reading");
            return;
        }

        Serial.println("- read from file:");
        while (file.available()) Serial.write(file.read());
        file.close();
    }

    void writeFile(fs::FS &fs, const char * path, const char * message) {
        Serial.printf("Writing file: %s\r\n", path);

        File file = fs.open(path, FILE_WRITE);
        if (!file){
            Serial.println("- failed to open file for writing");
            return;
        }
        if (file.print(message)) Serial.println("- file written");
        else Serial.println("- write failed");
        file.close();
    }

    void appendFile(fs::FS &fs, const char * path, const char * message) {
        Serial.printf("Appending to file: %s\r\n", path);

        File file = fs.open(path, FILE_APPEND);
        if (!file){
            Serial.println("- failed to open file for appending");
            return;
        }
        if (file.print(message)) Serial.println("- message appended");
        else Serial.println("- append failed");
        file.close();
    }
    void renameFile(fs::FS &fs, const char * path1, const char * path2) {
        Serial.printf("Renaming file %s to %s\r\n", path1, path2);
        if (fs.rename(path1, path2)) Serial.println("- file renamed");
        else Serial.println("- rename failed");
    }
    void deleteFile(fs::FS &fs, const char * path) {
        Serial.printf("Deleting file: %s\r\n", path);
        if (fs.remove(path)) Serial.println("- file deleted");
        else Serial.println("- delete failed");
    }
    void testFileIO(fs::FS &fs, const char * path) {
        Serial.printf("Testing file I/O with %s\r\n", path);

        static uint8_t buf[512];
        size_t len = 0;
        File file = fs.open(path, FILE_WRITE);
        if (!file) {
            Serial.println("- failed to open file for writing");
            return;
        }
        size_t i;
        Serial.print("- writing");
        uint32_t start = millis();
        for (i=0; i<2048; i++) {
            if ((i & 0x001F) == 0x001F) Serial.print(".");
            file.write(buf, 512);
        }
        Serial.println("");
        uint32_t end = millis() - start;
        Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
        file.close();

        file = fs.open(path);
        start = millis();
        end = start;
        i = 0;
        if (file && !file.isDirectory()) {
            len = file.size();
            size_t flen = len;
            start = millis();
            Serial.print("- reading" );
            while (len){
                size_t toRead = len;
                if (toRead > 512) toRead = 512;
                file.read(buf, toRead);
                if ((i++ & 0x001F) == 0x001F) Serial.print(".");
                len -= toRead;
            }
            Serial.println("");
            end = millis() - start;
            Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
            file.close();
        }
        else Serial.println("- failed to open file for reading");
    }
};