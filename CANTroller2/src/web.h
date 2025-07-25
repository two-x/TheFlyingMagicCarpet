#pragma once
// platformio includes necessary for web function:
//   ottowinter/ESPAsyncWebServer-esphome ;@^3.1.0
//   links2004/WebSockets                 ;@^2.4.1
//   ayushsharma82/ElegantOTA             ;@^3.1.0
//   bblanchon/ArduinoJson                ;@^6.21.5
//   esphome/AsyncTCP-esphome             ;@^2.1.1

// set up our flash filesystem
#define FORMAT_LITTLEFS_IF_FAILED true
#include <FS.h>
#include <LittleFS.h>
class FileSystem {
  private:
    void cleanLittleFS() {
        ezread.squintf("Cleaning LittleFS... \n");
        LittleFS.format();
        ezread.squintf("LittleFS cleaned\n");
    }
  public:
    FileSystem() {}
    void setup() {
        ezread.squintf("Littlefs mount %s", LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED) ? "point " : "failed ");
        
        // esp_vfs_fat_sdmmc_mount("/", );  // esp_vfs_fat_sdmmc_mount(const char *base_path, const sdmmc_host_t *host_config, const void *slot_config, const esp_vfs_fat_mount_config_t *mount_config, sdmmc_card_t **out_card)

        listDir(LittleFS, "/", 3);
    }
    void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
        ezread.squintf("%s", dirname);
        File root = fs.open(dirname);
        if (!root) {
            ezread.squintf("\n  failed to open directory");
            return;
        }
        if (!root.isDirectory()) {
            ezread.squintf("\n  not a directory");
            return;
        }
        File file = root.openNextFile();
        while(file) {
            if (file.isDirectory()) {
                ezread.squintf("\n  dir: %s", file.name());
                if (levels) listDir(fs, file.path(), levels -1);
            }
            else ezread.squintf("\n  file: %s  size: %d", file.name(), file.size());
            file = root.openNextFile();
        }
        ezread.squintf("\n");
    }
};

#if !WifiSupported  
// if wifi/web disabled (WifiSupported == false), set up just a dummy web manager

class WebManager {  // just a useless dummy version for code compatibility 
  private:
    FileSystem fs;
  public:
    WebManager(LoopTimer* unused) {}
    void setup() {
        fs.setup();
        ezread.squintf("Wifi/Web features are disabled..\n");
    }
    void update() { web_disabled = true; }
};

#else  // if WifiSupported == true  // (from here to the end of the file)

#include <Arduino.h>
// #include <FFat.h>
#include <WiFi.h>  // "Wifi.h"
#include <ESPAsyncWebServer.h>  // To run wifi in Soft Access Point (SAP) mode (standalone w/o router)
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>  // <AsyncJson.h>  // "json.h"  needed for JSON encapsulation (send multiple variables with one string)
#include <ElegantOTA.h>  // includes <AsyncTCP.h>

// create a callback function, triggered by web socket events
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) {  // the parameters of this callback function are always the same -> num: id of the client who send the event, type: type of message, payload: actual data sent and length: length of payload
    switch (type) {                                     // switch on the type of information sent
      case WStype_DISCONNECTED:                         // if a client is disconnected, then type == WStype_DISCONNECTED
        Serial.println("Client " + String(num) + " disconnected");
        break;
      case WStype_CONNECTED:                            // if a client is connected, then type == WStype_CONNECTED
        Serial.println("Client " + String(num) + " connected");
        // optionally you can add code here what to do when connected
        break;
      case WStype_TEXT:                                 // if a client has sent data, then type == WStype_TEXT
        // try to decipher the JSON string received
        StaticJsonDocument<200> doc;                    // create a JSON container
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }
        else {
            // JSON string was received correctly, so information can be retrieved:
            const char* g_brand = doc["brand"];
            const char* g_type = doc["type"];
            const int g_year = doc["year"];
            const char* g_color = doc["color"];
            Serial.println("Received guitar info from user: " + String(num));
            Serial.println("Brand: " + String(g_brand));
            Serial.println("Type: " + String(g_type));
            Serial.println("Year: " + String(g_year));
            Serial.println("Color: " + String(g_color));
        }
        Serial.println("");
        break;
        // for (int i=0; i<length; i++) {                  // print received data from client
        //     Serial.print((char)payload[i]);
        //     if ((char)payload[i] == 'Y') heartbeat_override_color = 0xfff8;
        //     else if ((char)payload[i] == 'N') heartbeat_override_color = 0x5cac;
        // }
        // Serial.println("");
        // break;
    //   case WStype_BIN:
    //     printf("[%u] get binary length: %u\n", num, length);
    //     printf("incloming data: 0x");
    //     for (int byt=0; byt<length; byt++) printf("%02x");
    //     Serial.println("");
    //     break;
    }
}
class AccessPoint {
  private:
    IPAddress localip, gateway, subnet, primarydns, secondarydns;
    const char* apssid = "artcarpet";
    const char* appassword = "checkmate";
    const char* ssid = "";  // non-ap mode need real credentials here, but we don't want this in github
    const char* password = "";  // non-ap mode need real credentials here, but we don't want this in github
    void connect_existing_wifi() {
        ezread.squintf("connecting to %s", ssid);
        primarydns = IPAddress(8, 8, 8, 8);
        secondarydns = IPAddress(8, 8, 4, 4);
        WiFi.setAutoReconnect(true);
        WiFi.config(localip, gateway, subnet, primarydns, secondarydns);
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        ezread.squintf("wifi connected. ip addr: ");
        ezread.squintf(WiFi.localIP());
    }
  public:
    AccessPoint() : localip(192,168,1,69), gateway(192,168,1,5), subnet(255,255,255,0) {}
    void setup() {
        ezread.squintf("Wifi access point.. ");
        WiFi.disconnect();  // in case already connected to another wifi as client or something
        WiFi.mode(WIFI_STA);
        WiFi.persistent(false);  // Don't store wifi config in eeprom, b/c it can get stuck there
        WiFi.setSleep(false);  // ensure server is awake for accessibility
        WiFi.softAPConfig(localip, gateway, subnet);
        WiFi.softAP(apssid, appassword);
        ezread.squintf("active. ssid:%s, pwd:%s, ip:", apssid, appassword);
        ezread.squintf(WiFi.softAPIP());
        // std::cout << "ip = " << WiFi.softAPIP() << std::endl;
        // printf(" ip = %s\n", my_ip.c_str());
    }
    void enable(bool sw) { WiFi.setSleep(!sw); }
};
class Web {
  private:
    AsyncWebServer webserver;
    WebSocketsServer socket;
    Timer socket_timer;
    int socket_refresh_us = 1000000, dumdum = 1;
    LoopTimer* looptimer;
  public:
    Web() : webserver(80), socket(81) {}
    void setup(LoopTimer* _lt) {
        looptimer = _lt;
        ezread.squintf("Web services..\n");
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
        webserver.onNotFound([](AsyncWebServerRequest *request) {  // Send back a plain text message (can be made html if required)
            request->send(404, "text/plain", "404 - Page Not Found.  Try '/' or '/update'");
        });
        ezread.squintf("  Over-The-Air update support..\n");
        ElegantOTA.begin(&webserver);  // start OTA after all server.on requests, before starting web server
        ezread.squintf("  Webserver..\n");
        webserver.begin();
        ezread.squintf("  Websockets..\n");
        socket.begin();
        socket.onEvent(webSocketEvent);
        socket_timer.set(socket_refresh_us);
    }
    void update () {
        // webserver.handleClient();
        socket.loop();
        if (socket_timer.expireset()) {
            // String sendme = String(random(100));
            // socket.broadcastTXT(sendme.c_str());
            String jsonString = "";                           // create a JSON string for sending data to the client
            StaticJsonDocument<200> doc;                      // create a JSON container
            JsonObject object = doc.to<JsonObject>();         // create a JSON Object
            object["rand1"] = random(100);                    // write data into the JSON object -> I used "rand1" and "rand2" here, but you can use anything else
            object["rand2"] = random(100);
            object["loopavg"] = (int32_t)(loop_avg_us);                    // write data into the JSON object -> I used "rand1" and "rand2" here, but you can use anything else
            object["looppeak"] = looptimer->loop_peak_us;
            serializeJson(doc, jsonString);                   // convert JSON object to string
            // Serial.println(jsonString);                       // print JSON string to console for debug purposes (you can comment this out)
            socket.broadcastTXT(jsonString);               // send JSON string to clients
        }
    }
};
class WebManager {
  private:
    bool web_started = false;
    LoopTimer* looptimer;
  public:
    FileSystem fs;
    AccessPoint wifi;
    Web server;
    WebManager(LoopTimer* _lt) : looptimer(_lt) {}
    void setup() {
        if (!wifi_web_supported) return;
        wifi.setup();
        fs.setup();
        server.setup(looptimer);
        web_started = true;
    }
    String processor(const String &var) {
        if (var == "CHESSBOARD_COLOR") return getRandomColor();  // Replace placeholders in the HTML with dynamic content
        return String();
    }
    String getRandomColor() {
        return String("rgb(") + String(random(256)) + "," + String(random(256)) + "," + String(random(256)) + ")";  // Generate a random RGB color
    }
    void update() {
        if (runmode == LowPower) return;
        if (!wifi_web_supported) web_disabled = true;
        if (web_disabled) return;
        if (!web_started) setup();
        wifi.enable(syspower.val());
        if (syspower.val()) server.update();
        // if (<activity in web page is detected>) kick_inactivity_timer(HuWeb);  // evidence of user activity
    }
};
#endif  // end of if wifi/web is enabled

static WebManager web(&looptimer);

void web_task(void *parameter) {
    while (true) {
        web.update();
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 20ms, hopefully that's fast enough
    }
}
