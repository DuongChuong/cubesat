#include "wifi_manager.h"
#include "sensor_manager.h"
#include "web_server.h"
#include "camera_manager.h"

#include <Wire.h>
#include <PID_v1.h>


// Motor A
#define AIN1 12
#define AIN2 15
#define PWMA_PIN 4


// ESP32 PWM CONFIGURATION 
const int PWM_FREQ = 5000; 
const int PWM_RESOLUTION = 8; 
const int PWM_CHANNEL_A = 0;

// PID TUNING PARAMETERS
double Kp = 20.0;  
double Ki = 0.3;  
double Kd = 4.5; 

// BALANCE SETPOINT
double BALANCE_SETPOINT_DEG = 0.0;

// Safety limits
#define MAX_ANGLE 90.0
#define DEAD_ZONE 10 

// GLOBAL OBJECTS & VARIABLES 
// MPU6050 Sensor
sensors_event_t a, g, temp;

// PID Controller
double setpoint, input, output;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Angle calculation variables
float currentAngle_rad = 0.0;
float gyroY_cal_rad = 0.0;

// Loop timing
const int LOOP_TIME_MS = 10; // 10ms loop time = 100Hz
unsigned long lastLoopTime = 0;

// Filter parameters
const float GYRO_WEIGHT = 0.98;
const float ACCEL_WEIGHT = 0.02;

//  SETUP FUNCTION 
void setup() {
  Serial.begin(115200);

  initSensor();
  initCamera();
  initWiFi();
  setupWebServer();

  // Calibrate Gyro
  calibrateGyro();
  Serial.println("Calibration Complete.");

  // Configure Motor Driver Pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  pinMode(PWMA_PIN, OUTPUT);


  // Stop motors initially
  driveMotors(0);

  // Initialize angle from accelerometer for better startup
  mpu.getEvent(&a, &g, &temp);
  float accX = a.acceleration.x;
  float accZ = a.acceleration.z;
  currentAngle_rad = atan2(accX, accZ);

  // Configure PID Controller
  setpoint = BALANCE_SETPOINT_DEG;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(LOOP_TIME_MS);

  Serial.println("SETUP COMPLETE");
  lastLoopTime = millis();
}

// --- GYRO CALIBRATION ---
void calibrateGyro() {
  const int numSamples = 1000;
  double sumY = 0;

  Serial.print("Calibrating");
  for (int i = 0; i < numSamples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumY += g.gyro.y;
    if (i % 100 == 0) Serial.print(".");
    delay(2);
  }
  gyroY_cal_rad = sumY / numSamples;

  Serial.println();
  Serial.print("Gyro Y Offset (rad/s): ");
  Serial.println(gyroY_cal_rad, 6);
}

// MOTOR CONTROL FUNCTION
void driveMotors(int speed) {
  if (abs(speed) < DEAD_ZONE) {
    speed = 0;
  }
  
  int motorSpeed = constrain(speed, -10, 10);

  // Motor control
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

  }

  int pwmValue = abs(motorSpeed);
  analogWrite(PWMA_PIN, pwmValue);
}

// FIND BALANCE POINT FUNCTION
void findBalancePoint() {
  Serial.println("FINDING BALANCE POINT");
  
  while (!Serial.available()) {
    mpu.getEvent(&a, &g, &temp);
    float accX = a.acceleration.x;
    float accZ = a.acceleration.z;
    float angle_deg = atan2(accX, accZ) * (180.0 / PI);
    
    Serial.print("Current Angle: ");
    Serial.print(angle_deg);
    Serial.println(" degrees");
    delay(200);
  }
  while (Serial.available()) Serial.read(); // Clear buffer
}

// --- TEST MOTORS FUNCTION ---
void testMotors() {
  Serial.println("Testing forward direction");
  driveMotors(100);
  delay(1000);
  driveMotors(0);
  delay(500);
  
  Serial.println("Testing reverse direction");
  driveMotors(-100);
  delay(1000);
  driveMotors(0);
  
  Serial.println("Motor test complete!");
  delay(1000);
}

void loop() {
  // Enforce fixed loop time
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_TIME_MS) {
    return;
  }
  float dt = (now - lastLoopTime) / 1000.0f;
  lastLoopTime = now;

  // Read Sensor Data
  mpu.getEvent(&a, &g, &temp);

  // Calculate Angle (Complementary Filter)
  float gyroY_rad = g.gyro.y - gyroY_cal_rad;
  float accX = a.acceleration.x;
  float accZ = a.acceleration.z;

  // Angle from accelerometer (radians)
  float angleAcc_rad = atan2(accX, accZ);

  // Angle from gyroscope (integration)
  float angleGyro_rad = currentAngle_rad + gyroY_rad * dt;

  // Complementary Filter
  currentAngle_rad = GYRO_WEIGHT * angleGyro_rad + ACCEL_WEIGHT * angleAcc_rad;

  // Convert to degrees for PID
  float currentAngle_deg = currentAngle_rad * (180.0 / PI);
  input = currentAngle_deg;


  // Compute PID
  myPID.Compute();

  // Control Motors
  driveMotors((int)output);

  // Debug Output (less verbose)
  static unsigned long lastDebug = 0;
  if (now - lastDebug > 100) {
    Serial.print("Angle: ");
    Serial.print(currentAngle_deg, 1);
    Serial.print("°\tPID: ");
    Serial.print(output);
    Serial.print("\tPWM: ");
    Serial.println(abs((int)output));
    lastDebug = now;
  }

  loopWebServer(); 
}