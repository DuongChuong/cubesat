#include <esp_camera.h>
#include "web_server.h"

// Forward declaration
extern String getSensorDataAsJson();

WebServer server(80);

// Frame buffer for snapshot
camera_fb_t* currentFrame = nullptr;
unsigned long lastCaptureTime = 0;
const unsigned long CAPTURE_INTERVAL = 5000; // 5 seconds

void handleCapture() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb == nullptr) {
        server.send(500, "text/plain", "Camera capture failed");
        return;
    }
    
    server.sendHeader("Content-Type", "image/jpeg");
    server.sendHeader("Content-Length", String(fb->len));
    server.sendHeader("Cache-Control", "no-cache");
    server.send_P(200, "image/jpeg", (char*)fb->buf, fb->len);
    
    esp_camera_fb_return(fb);
}

void handleData() {
    String json = getSensorDataAsJson();
    server.send(200, "application/json", json);
}

void handleRoot() {
    // Simple status page (optional)
    String html = "<html><body>";
    html += "<h1>ESP32-CAM Dashboard Server</h1>";
    html += "<p>Endpoints:</p>";
    html += "<ul>";
    html += "<li><a href='/capture'>/capture - Get JPEG snapshot</a></li>";
    html += "<li><a href='/data'>/data - Get sensor data JSON</a></li>";
    html += "</ul>";
    html += "<p>Use with Python dashboard client.</p>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/capture", handleCapture);
    server.on("/data", handleData);
    server.begin();
    
    Serial.println("WebServer started!");
    Serial.println("Python Dashboard endpoints:");
    Serial.println("  http://[IP]/capture - Camera snapshot");
    Serial.println("  http://[IP]/data - Sensor data");
}

void loopWebServer() {
    server.handleClient();
}