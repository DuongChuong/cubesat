#include <WiFi.h>
#include "wifi_manager.h" 

const char* ssid = "nguyen du";
const char* password = "06052004";

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