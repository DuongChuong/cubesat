#pragma once 

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>

extern const char* ssid;
extern const char* password;

void initWiFi();

#endif