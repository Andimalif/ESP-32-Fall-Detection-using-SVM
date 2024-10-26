#include <Wire.h>
#include <MPU9250_asukiaaa.h>  // Library for interfacing with MPU9250
#include <eloquent_tinyml.h>   // Library for machine learning functionalities
#include "model.h"      // Header file for trained machine learning model
#include <WiFi.h>              // WiFi library for connectivity

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 8  // Define the SDA pin for I2C communication
#define SCL_PIN 9  // Define the SCL pin for I2C communication
#define NUMBER_OF_INPUTS 22   // Number of inputs expected by the ML model
#define NUMBER_OF_OUTPUTS 2   // Number of outputs from the ML model
#define BLYNK_TEMPLATE_ID ""  // Blynk template ID
#define BLYNK_TEMPLATE_NAME ""  // Blynk template name
#define BLYNK_AUTH_TOKEN ""  // Blynk authentication token
#endif

#include <BlynkSimpleEsp32.h>  // Blynk library for connecting to its server

// WiFi credentials
const char ssid[] = "";       
const char pass[] = ""; 

// Define pins for auxiliary devices
const int ADD0 = 5;  // I2C address select pin
const int BUZ = 10;  // Buzzer pin

bool fallDetected = false;  // Boolean flag for fall detection status

// Acceleration thresholds for detecting a fall
const float ACCEL_THRESHOLD_HIGH = 15.0;
const float ACCEL_THRESHOLD_LOW = 2.0;

// Complementary filter constant for sensor fusion
const float alpha = 0.98;
float filteredPitch = 0.0;  // Filtered pitch angle
float filteredRoll = 0.0;   // Filtered roll angle

// Define MPU9250 sensor objects
MPU9250_asukiaaa MPU1(MPU9250_ADDRESS_AD0_LOW);
MPU9250_asukiaaa MPU2(MPU9250_ADDRESS_AD0_HIGH);

// Structure to hold sensor data
struct SensorData {
  unsigned long timestamp;  // Timestamp of the sensor reading
  float aX, aY, aZ;         // Accelerometer readings
  float gX, gY, gZ;         // Gyroscope readings
  float mX, mY, mZ;         // Magnetometer readings
  float pitch, roll;        // Orientation angles
  float prevPitch, prevRoll;  // Previous orientation angles
};

// Instances of sensor data for two MPU9250 sensors
SensorData sensorData1;
SensorData sensorData2;

// Structure to hold calibration offsets
struct CalibrationOffsets {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
};

// Instances of calibration offsets for each sensor
CalibrationOffsets calibrationOffsets1;
CalibrationOffsets calibrationOffsets2;

// Machine learning classifier object
Eloquent::ML::Port::SVM classifier;

// Function to calibrate sensors
void calibrateSensors(MPU9250_asukiaaa &MPU, CalibrationOffsets &calibrationOffsets) {
  Serial.println("Calibrating sensors...");
  int numSamples = 600;  // Number of samples for calibration
  float sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  float sumMagX = 0, sumMagY = 0, sumMagZ = 0;

  for (int i = 0; i < numSamples; i++) {
    // Update all sensor values
    MPU.accelUpdate();
    MPU.gyroUpdate();
    MPU.magUpdate();

    // Accumulate sensor readings for calibration
    sumAccelX += MPU.accelX();
    sumAccelY += MPU.accelY();
    sumAccelZ += MPU.accelZ();
    sumGyroX += MPU.gyroX();
    sumGyroY += MPU.gyroY();
    sumGyroZ += MPU.gyroZ();
    sumMagX += MPU.magX();
    sumMagY += MPU.magY();
    sumMagZ += MPU.magZ();

    delay(20);  // Delay to allow sensor settling
  }

  // Calculate average offsets
  calibrationOffsets.accelX = sumAccelX / numSamples;
  calibrationOffsets.accelY = sumAccelY / numSamples;
  calibrationOffsets.accelZ = sumAccelZ / numSamples;
  calibrationOffsets.gyroX = sumGyroX / numSamples;
  calibrationOffsets.gyroY = sumGyroY / numSamples;
  calibrationOffsets.gyroZ = sumGyroZ / numSamples;
  calibrationOffsets.magX = sumMagX / numSamples;
  calibrationOffsets.magY = sumMagY / numSamples;
  calibrationOffsets.magZ = sumMagZ / numSamples;

  Serial.println("Calibration complete.");
}

// Function to read and process sensor data
void readSensorData(MPU9250_asukiaaa &MPU, SensorData &sensorData, CalibrationOffsets &calibrationOffsets) {
  // Update and apply offsets to sensor readings
  MPU.accelUpdate();
  sensorData.aX = MPU.accelX() - calibrationOffsets.accelX;
  sensorData.aY = MPU.accelY() - calibrationOffsets.accelY;
  sensorData.aZ = MPU.accelZ() - calibrationOffsets.accelZ;

  MPU.gyroUpdate();
  sensorData.gX = MPU.gyroX() - calibrationOffsets.gyroX;
  sensorData.gY = MPU.gyroY() - calibrationOffsets.gyroY;
  sensorData.gZ = MPU.gyroZ() - calibrationOffsets.gyroZ;

  MPU.magUpdate();
  sensorData.mX = MPU.magX() - calibrationOffsets.magX;
  sensorData.mY = MPU.magY() - calibrationOffsets.magY;
  sensorData.mZ = MPU.magZ() - calibrationOffsets.magZ;

  // Time calculation for integration
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds
  lastTime = currentTime;

  // Calculate pitch and roll using accelerometer data
  float accelPitch = atan2(sensorData.aY, sqrt(pow(sensorData.aX, 2) + pow(sensorData.aZ, 2))) * 180.0 / PI;
  float accelRoll = atan2(-sensorData.aX, sensorData.aZ) * 180.0 / PI;

  // Integrate gyroscope data
  float gyroPitch = sensorData.gX * dt;
  float gyroRoll = sensorData.gY * dt;

  // Apply complementary filter for sensor fusion
  filteredPitch = alpha * (filteredPitch + gyroPitch) + (1.0 - alpha) * accelPitch;
  filteredRoll = alpha * (filteredRoll + gyroRoll) + (1.0 - alpha) * accelRoll;

  // Store filtered values in sensor data structure
  sensorData.pitch = filteredPitch;
  sensorData.roll = filteredRoll;
}

void setup() {
  Serial.begin(115200);  // Initialize serial communication for debugging
  while (!Serial)
    ;  // Wait for serial port to connect
  Serial.println("started");

  pinMode(ADD0, OUTPUT);  // Set ADD0 pin mode
  pinMode(BUZ, OUTPUT);   // Set BUZ pin mode
  digitalWrite(ADD0, 1);  // Set ADD0 high for I2C addressing

#ifdef _ESP32_HAL_I2C_H_
  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C communication
  MPU1.setWire(&Wire);           // Set I2C wire for MPU1
  MPU2.setWire(&Wire);           // Set I2C wire for MPU2
#endif

  // Initialize sensors
  MPU1.beginAccel();
  MPU1.beginGyro();
  MPU1.beginMag();

  MPU2.beginAccel();
  MPU2.beginGyro();
  MPU2.beginMag();

  // Calibrate sensors
  calibrateSensors(MPU1, calibrationOffsets1);
  calibrateSensors(MPU2, calibrationOffsets2);

  // Read initial sensor data
  readSensorData(MPU1, sensorData1, calibrationOffsets1);
  readSensorData(MPU2, sensorData2, calibrationOffsets2);

  // Buzzer initialization to indicate startup
  digitalWrite(BUZ, 1);
  delay(500);
  digitalWrite(BUZ, 0);

  // Connect to Wi-Fi and initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  // Continuously read sensor data
  readSensorData(MPU1, sensorData1, calibrationOffsets1);
  readSensorData(MPU2, sensorData2, calibrationOffsets2);

  // Calculate total acceleration for fall detection
  float totalAccel1 = sqrt(sensorData1.aX * sensorData1.aX + sensorData1.aY * sensorData1.aY + sensorData1.aZ * sensorData1.aZ);
  float totalAccel2 = sqrt(sensorData2.aX * sensorData2.aX + sensorData2.aY * sensorData2.aY + sensorData2.aZ * sensorData2.aZ);

  // Perform prediction only if acceleration exceeds threshold
  if (totalAccel1 > ACCEL_THRESHOLD_HIGH || totalAccel2 > ACCEL_THRESHOLD_HIGH) {
    // Prepare features array for prediction
    float features[NUMBER_OF_INPUTS] = {
      // Sensor data from MPU1
      sensorData1.aX, sensorData1.aY, sensorData1.aZ,
      sensorData1.gX, sensorData1.gY, sensorData1.gZ,
      sensorData1.mX, sensorData1.mY, sensorData1.mZ,
      sensorData1.pitch, sensorData1.roll,
      // Sensor data from MPU2
      sensorData2.aX, sensorData2.aY, sensorData2.aZ,
      sensorData2.gX, sensorData2.gY, sensorData2.gZ,
      sensorData2.mX, sensorData2.mY, sensorData2.mZ,
      sensorData2.pitch, sensorData2.roll,
    };

    unsigned long startTime = millis();  // Start time for computation timing

    // Perform prediction using the trained ML model
    int result = classifier.predict(features);

    unsigned long endTime = millis();    // End time for computation timing
    unsigned long computationTime = endTime - startTime;  // Calculate computation time

    // Output prediction result and computation time
    Serial.print("Prediksi kelas: ");
    Serial.println(result);
    Serial.print("Waktu komputasi: ");
    Serial.print(computationTime);
    Serial.println(" ms");

    // Execute actions based on prediction result
    if (result == 1) {  // Fall detected
      Serial.println("Fall detected!");
      fallDetected = true;  // Set fall detected flag
      digitalWrite(BUZ, 1);  // Activate buzzer
      delay(2000);           // Keep buzzer on for 2 seconds
      digitalWrite(BUZ, 0);  // Turn off buzzer
      Blynk.logEvent("falldetected", "Jatuh Terdeteksi !!! Segera Bantu !!!");
    } else {
      fallDetected = false;  // Reset fall detected flag
      digitalWrite(BUZ, 0);  // Ensure buzzer is off if no fall
    }
  }

  Blynk.run();  // Run Blynk communication handler
}
