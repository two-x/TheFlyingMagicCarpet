#pragma once
#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>
#include "WiFi.h"  // <Wifi.h>
#include <ESPAsyncWebServer.h>  // To run wifi in Soft Access Point (SAP) mode (standalone w/o router)
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
// #include <AsyncJson.h>  // "json.h"
#define FORMAT_LITTLEFS_IF_FAILED true
class FileSystem {
  private:
    void cleanLittleFS() {
        Serial.println("Cleaning LittleFS...");
        LittleFS.format();
        Serial.println("LittleFS cleaned.");
    }
  public:
    FileSystem() {}
    void setup() {
        printf("Littlefs start%s\n", LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED) ? ".." : " failed");
        listDir(LittleFS, "/", 3);
    }
    void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
        printf("listing %s :", dirname);
        File root = fs.open(dirname);
        if(!root) {
            printf("\n  failed to open directory");
            return;
        }
        if (!root.isDirectory()) {
            printf("\n  not a directory");
            return;
        }
        File file = root.openNextFile();
        while(file) {
            if(file.isDirectory()) {
                printf("\n  dir: %s", file.name());
                if(levels) listDir(fs, file.path(), levels -1);
            }
            else printf("\n  file: %s  size: %d", file.name(), file.size());
            file = root.openNextFile();
        }
        printf("\n");
    }
};
class AccessPoint {
  private:
    IPAddress localip, gateway, subnet;
    const char* ssid = "magiccarpet";
    const char* password = "checkmate";
    void connect_existing_wifi() {
        printf("connecting to %s", ssid);
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        printf("wifi connected. ip addr: ");
        Serial.println(WiFi.localIP());
    }
  public:
    AccessPoint() : localip(192,168,1,69), gateway(192,168,1,5), subnet(255,255,255,0) {}
    void setup() {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        printf("Wifi access point..\n");
        WiFi.softAPConfig(localip, gateway, subnet);
        WiFi.softAP(ssid, password);
        printf(" ip = %s\n", WiFi.softAPIP());
    }
};
class WebServer {
  private:
    AsyncWebServer webserver;
  public:
    WebServer() : webserver(80) {}
    void setup() {
        printf("Webserver start..\n");
        webserver.on("/tester", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(200, "text/plain", "Hello, tester");
        });
        webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/index.html", "text/html");  // Serve the HTML page
            request->send(response);
        });
        webserver.on("/chessboard.js", HTTP_GET, [](AsyncWebServerRequest *request) {
            AsyncWebServerResponse *response = request->beginResponse(LittleFS, "/chessboard.js", "application/javascript");  // Serve the JavaScript file
            request->send(response);
        });
        webserver.begin();
    }
};
class WebSocket {
  private:
    WebSocketsServer socket = WebSocketsServer(81);
    Timer socket_timer;
    uint32_t socket_refresh_us = 1000000, dumdum = 1;
    void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
        switch (type) {
            case WStype_DISCONNECTED:
                Serial.printf("[%u] Disconnected!\n", num);
                break;
            case WStype_CONNECTED:
                {
                    IPAddress ip = socket.remoteIP(num);
                    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
                    String sendme = String(dumdum);
                    socket.sendTXT(num, sendme.c_str());
                }
                break;
            case WStype_TEXT:
                Serial.printf("[%u] get Text: %s\n", num, payload);
                break;
        }
    }
  public:
    WebSocket() {}
    void setup() {
        printf("Websocket start..\n");
        socket.begin();
        // socket.onEvent(webSocketEvent);
        socket_timer.set(socket_refresh_us);
    }
    void update() {
        if (socket_timer.expireset()) {
            String sendme = String(dumdum);
            socket.broadcastTXT(sendme.c_str());
            socket.loop();
        }
    }
};
class WebManager {
  private:
  public:
    FileSystem fs;
    AccessPoint wifi;
    WebServer server;
    WebSocket socket; 
    WebManager() {}
    void setup() {
        if (!web_enabled) return;
        wifi.setup();
        fs.setup();
        server.setup();
        socket.setup();
    }
    String processor(const String &var) {
        if (var == "CHESSBOARD_COLOR") return getRandomColor();  // Replace placeholders in the HTML with dynamic content
        return String();
    }
    String getRandomColor() {
        return String("rgb(") + String(random(256)) + "," + String(random(256)) + "," + String(random(256)) + ")";  // Generate a random RGB color
    }
    void update() {
        if (!web_enabled) return;
        socket.update();
    }
};