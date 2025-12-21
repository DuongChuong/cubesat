#pragma once
#include <Adafruit_MPU6050.h>

// Khai báo chân
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Khai bao dia chi
#define MPU6050_ADDR 0x68
#define LM75_ADDR 0x48

extern Adafruit_MPU6050 mpu;

// Khai báo hàm
void initSensor();
String getSensorDataAsJson();