#pragma once
#include <WebServer.h>

extern WebServer server;

// Khai báo các hàm
void setupWebServer();
void loopWebServer();
void handleRoot();
void handleCapture();
void handleData();
