#include <WiFi.h>
#include "wifi_manager.h" 

const char* ssid = "UET-Wifi-T3 2.4Ghz";
const char* password = "";

void initWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}