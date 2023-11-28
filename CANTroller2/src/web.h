#pragma once
#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>
#include <WiFi.h>  // "Wifi.h"
#include <ESPAsyncWebServer.h>  // To run wifi in Soft Access Point (SAP) mode (standalone w/o router)
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>  // <AsyncJson.h>  // "json.h"  needed for JSON encapsulation (send multiple variables with one string)
#define FORMAT_LITTLEFS_IF_FAILED true
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
        printf("Littlefs mount %s", LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED) ? "point " : "failed ");
        listDir(LittleFS, "/", 3);
    }
    void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
        printf("%s :", dirname);
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
        Serial.print("Wifi access point.. ip = ");
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        WiFi.softAPConfig(localip, gateway, subnet);
        WiFi.softAP(ssid, password);
        Serial.println(WiFi.softAPIP());

        // std::cout << "ip = " << WiFi.softAPIP() << std::endl;
        // printf(" ip = %s\n", my_ip.c_str());
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
    void update () {
        // webserver.handleClient();
    }
};
class WebSocket {
  private:
    WebSocketsServer socket;
    Timer socket_timer;
    uint32_t socket_refresh_us = 1000000, dumdum = 1;
  public:
    WebSocket() : socket(81) {}
    void setup() {
        printf("Websocket start..\n");
        socket.begin();
        socket.onEvent(webSocketEvent);
        socket_timer.set(socket_refresh_us);
    }
    void update() {
        socket.loop();
        if (socket_timer.expireset()) {
            // String sendme = String(random(100));
            // socket.broadcastTXT(sendme.c_str());
            String jsonString = "";                           // create a JSON string for sending data to the client
            StaticJsonDocument<200> doc;                      // create a JSON container
            JsonObject object = doc.to<JsonObject>();         // create a JSON Object
            object["rand1"] = random(100);                    // write data into the JSON object -> I used "rand1" and "rand2" here, but you can use anything else
            object["rand2"] = random(100);
            serializeJson(doc, jsonString);                   // convert JSON object to string
            // Serial.println(jsonString);                       // print JSON string to console for debug purposes (you can comment this out)
            socket.broadcastTXT(jsonString);               // send JSON string to clients
        }
    }
};
class WebManager {
  private:
    bool web_started = false;
  public:
    FileSystem fs;
    AccessPoint wifi;
    WebServer server;
    WebSocket socket; 
    WebManager() {}
    void setup() {
        wifi.setup();
        fs.setup();
        server.setup();
        socket.setup();
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
        if (!web_enabled) return;
        if (!web_started) setup();
        socket.update();
    }
};