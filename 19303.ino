#include <Wire.h>             // Include Wire library for I2C devices
#include <Adafruit_Sensor.h>  // Include Adafruit sensor library
#include <Adafruit_BMP280.h>  // Include Adafruit library for BMP280 sensor
#include <MPU6050.h>

// Pin definitions
const int BuzzerPin = 3;  // Buzzer connected to digital pin 3
const int Pin = 2;        // Proximity sensor connected to digital pin 2

// BMP280 I2C address
#define BMP280_I2C_ADDRESS 0x76

Adafruit_BMP280 bmp280;  // BMP280 sensor object
MPU6050 mpu;             // MPU6050 sensor object

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  
  // Initialize pins
  pinMode(BuzzerPin, OUTPUT);
  pinMode(Pin, INPUT);

  // Print setup status
  Serial.println(F("Initializing Arduino + BMP280 + MPU6050"));

  // Initialize I2C and sensors
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  mpu.setFullScaleGyroRange(250); // Set full-scale range to ±250°/s
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed. Check wiring!");
    while (1);
  }

  // Initialize BMP280
  if (!bmp280.begin(BMP280_I2C_ADDRESS)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read pressure from BMP280
  float pressure = bmp280.readPressure();
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" Pa");
  Serial.println(); // Blank line for clarity

  // Read angular velocity from MPU6050
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Calculate angular velocity in degrees per second
  float angularVelocityX = gx / 131.0;
  float angularVelocityY = gy / 131.0;
  float angularVelocityZ = gz / 131.0;

  // Calculate magnitude of angular velocity vector
  float angularVelocityMagnitude = sqrt(pow(angularVelocityX, 2) + pow(angularVelocityY, 2) + pow(angularVelocityZ, 2));

  // Buzzer control based on angular velocity magnitude
  if (angularVelocityMagnitude > 0.1) {
    digitalWrite(BuzzerPin, HIGH); // Turn the buzzer ON
    Serial.println("WARNING : Angular velocity exceeded!");
  } else {
    digitalWrite(BuzzerPin, LOW);  // Turn the buzzer OFF
  }

  Serial.print("Angular Velocity Magnitude: ");
  Serial.print(angularVelocityMagnitude/100);
  Serial.println(" deg/s");
  Serial.println(); // Blank line for clarity

  // Read proximity sensor value
  int sensorValue = digitalRead(Pin);
  if (sensorValue == HIGH) {
    digitalWrite(BuzzerPin, HIGH); // Turn the buzzer ON
    Serial.println("WARNING : object comeing");
  } else {
    Serial.println("NO Object Detected");
    digitalWrite(BuzzerPin, LOW); // Turn the buzzer ON
    
  }

  // Delay for 2 seconds between loops
  delay(2000);
}
