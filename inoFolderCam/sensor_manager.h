#pragma once
#include <Adafruit_MPU6050.h>

// Khai báo chân
#define I2C_SDA_PIN 14
#define I2C_SCL_PIN 13

#define MPU6050_ADDR 0x64
#define LM75_ADDR 0x48

extern Adafruit_MPU6050 mpu;


// Khai báo hàm
void initSensor();
String getSensorDataAsJson();