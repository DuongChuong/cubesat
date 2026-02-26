#pragma once
#include <WebServer.h>

extern WebServer server;

// Define function
void setupWebServer();
void loopWebServer();
void handleRoot();
void handleCapture();
void handleData();
