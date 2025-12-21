#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <LM75.h>
#include "sensor_manager.h"

Adafruit_MPU6050 mpu;
LM75 lm75(LM75_ADDR);

void initSensor() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!mpu.begin(MPU6050_ADDR, &Wire)) {
        Serial.println("Failed to find MPU6050. Check wiring!");
        while (1) delay(10);
    }
    Serial.println("MPU6050 Found!");
    // Set up giai do 
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Check LM75
    Wire.beginTransmission(LM75_ADDR);
    byte error = Wire.endTransmission();
    
    if (error == 0){
        Serial.println("LM75 found");
    }
    else{
        Serial.println("LM75 not found");
    } 
    delay(100);
}



String getSensorDataAsJson() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float lm75_temp = lm75.getTemperature();

    String json = "{";
    json += "\"ax\":" + String(a.acceleration.x) + ",";
    json += "\"ay\":" + String(a.acceleration.y) + ",";
    json += "\"az\":" + String(a.acceleration.z) + ",";
    json += "\"gx\":" + String(g.gyro.x) + ",";
    json += "\"gy\":" + String(g.gyro.y) + ",";
    json += "\"gz\":" + String(g.gyro.z) + ",";
    json += "\"temp\":" + String(lm75_temp);
    json += "}";
    return json;
}