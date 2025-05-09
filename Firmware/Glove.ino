#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

Adafruit_MPU6050 mpu;

// Pin Definitions
const int flexPins[4] = {34, 35, 33, 32}; // Flex Sensor Pins (ESP32 ADC Pins)
int readings = 0;                         // Moving Average Filter
int flexValues[4] = {0, 0, 0, 0};         // Current flex sensor values
int lastFlexValues[4] = {0, 0, 0, 0};     // Previous flex sensor values

// Hardcoded Flex Sensor Calibration Values
const int FLEX_MIN[4] = {2080, 1920, 2220, 2150}; // Min values (unbent)
const int FLEX_MAX[4] = {2180, 1980, 2280, 2220}; // Max values (bent)
const int FLEX_RANGE[4] = {100, 60, 60, 70};      // Range calculation (MAX - MIN)

// Gesture Recognition Variables
String currentCharacter = "";
String recognizedText = "";
unsigned long lastGestureTime = 0;
const unsigned long gestureTimeout = 1000; // Time window for a gesture (ms)
bool newGestureDetected = false;
bool gestureProcessed = true;

// MPU6050 Variables
float accelX, accelY, accelZ;
float lastAccelX, lastAccelY, lastAccelZ;
bool motionDetected = false;

// Firebase Configuration
#define WIFI_SSID "Ocean134" // Your WiFi SSID
#define WIFI_PASSWORD "ocean123" // Your WiFi Password
#define API_KEY "AIzaSyA8fW4R878m7V5Vqqm0G4CiTqEn3lmcqqE" // Firebase API Key
#define DATABASE_URL "https://gesture-glove-default-rtdb.asia-southeast1.firebasedatabase.app/" // Firebase DB URL

FirebaseData fbdo; // Firebase data object
FirebaseAuth auth; // Firebase authentication
FirebaseConfig config; // Firebase configuration
bool signupOK = false;
unsigned long sendDataPrevMillis = 0;
unsigned long lastUpdateTime = 0;
const int sampleInterval = 100;  // Optimized sampling rate (ms)

// Threshold values - adjust these based on your calibration
const int FLEX_THRESHOLD = 40;  // Threshold to detect flex sensor bending
const float ACCEL_THRESHOLD = 2.0; // Threshold for motion detection

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nConnected to Wi-Fi!");
  Serial.println("IP Address: " + WiFi.localIP().toString());

  // Firebase Setup
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback; // For token status updates
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase Sign-Up Successful!");
    signupOK = true;
  } else {
    Serial.println("Firebase Sign-Up Failed: " + String(config.signer.signupError.message.c_str()));
  }
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true); // Auto-reconnect WiFi

  Serial.println("Gesture to Text System Initialized");
  Serial.println("Using hardcoded calibration values:");
  for (int i = 0; i < 4; i++) {
    Serial.print("Flex Sensor ");
    Serial.print(i + 1);
    Serial.print(" range: ");
    Serial.print(FLEX_MIN[i]);
    Serial.print(" - ");
    Serial.println(FLEX_MAX[i]);
  }
  
  Serial.println("System ready! Start making gestures");
}

void loop() {
  // Check if it's time to sample the sensors
  if (millis() - lastUpdateTime >= sampleInterval) {
    lastUpdateTime = millis();
    
    // Read sensor values
    readFlexSensors();
    readAccelerometer();
    
    // Check for gestures
    detectGesture();
    
    // Process any detected gesture
    if (newGestureDetected && !gestureProcessed) {
      interpretGesture();
      gestureProcessed = true;
      newGestureDetected = false;
      
      // Print current recognized text
      Serial.print("Current Text: ");
      Serial.println(recognizedText);
    }
    
    // Update Firebase every 2 seconds or when a new character is added
    if (Firebase.ready() && signupOK && 
        (millis() - sendDataPrevMillis > 2000 || currentCharacter != "")) {
      sendDataPrevMillis = millis();
      updateFirebase();
    } 
  }
  
}

void readFlexSensors() {
  // Save previous values
  for (int i = 0; i < 4; i++) {
    lastFlexValues[i] = flexValues[i];
  }
  
  // Read current values with filtering
  for (int i = 0; i < 4; i++) {
    readings = 0;
    // Moving Average Filter (Noise Reduction)
    for (int j = 0; j < 10; j++) {  // Reduced from 20 to 10 for faster response
      readings += analogRead(flexPins[i]);
      delay(1);
    }
    flexValues[i] = readings / 10;
  }

  // Debug output
  Serial.println("\nFlex Sensor Values:");
  for (int i = 0; i < 4; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(flexValues[i]);
    
    // Calculate bend percentage based on calibration values
    int bendPercentage = map(flexValues[i], FLEX_MIN[i], FLEX_MAX[i], 0, 100);
    bendPercentage = constrain(bendPercentage, 0, 100);
    Serial.print(" (");
    Serial.print(bendPercentage);
    Serial.println("% bent)");
  }
}

void readAccelerometer() {
  // Save previous values
  lastAccelX = accelX;
  lastAccelY = accelY;
  lastAccelZ = accelZ;
  
  // Read current values
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  
  // Debug output
  Serial.print("Acceleration - X: ");
  Serial.print(accelX);
  Serial.print(", Y: ");
  Serial.print(accelY);
  Serial.print(", Z: ");
  Serial.println(accelZ);
}

void detectGesture() {
  // Check if there's significant change in flex sensors
  bool flexChanged = false;
  for (int i = 0; i < 4; i++) {
    if (abs(flexValues[i] - lastFlexValues[i]) > FLEX_THRESHOLD) {
      flexChanged = true;
      break;
    }
  }
  
  // Check if there's significant acceleration
  bool accelChanged = false;
  if (abs(accelX - lastAccelX) > ACCEL_THRESHOLD || 
      abs(accelY - lastAccelY) > ACCEL_THRESHOLD || 
      abs(accelZ - lastAccelZ) > ACCEL_THRESHOLD) {
    accelChanged = true;
  }
  
  // If any sensor changed significantly, consider it a new gesture
  if ((flexChanged || accelChanged) && 
      (millis() - lastGestureTime > gestureTimeout)) {
    newGestureDetected = true;
    gestureProcessed = false;
    lastGestureTime = millis();
    Serial.println("New gesture detected!");
  }
}

bool isFingerBent(int fingerIndex) {
  // Calculate bend percentage
  int value = flexValues[fingerIndex];
  int threshold = FLEX_MIN[fingerIndex] + (FLEX_RANGE[fingerIndex] * 0.5); // 50% of range
  
  // Return true if the finger is considered bent
  return (value > threshold);
}

void interpretGesture() {
  // Check if each finger is bent based on calibrated values
  bool finger1Bent = isFingerBent(0);
  bool finger2Bent = isFingerBent(1);
  bool finger3Bent = isFingerBent(2);
  bool finger4Bent = isFingerBent(3);
  
  // Debug finger positions
  Serial.println("Finger positions:");
  Serial.println(finger1Bent ? "Finger 1: BENT" : "Finger 1: STRAIGHT");
  Serial.println(finger2Bent ? "Finger 2: BENT" : "Finger 2: STRAIGHT");
  Serial.println(finger3Bent ? "Finger 3: BENT" : "Finger 3: STRAIGHT");
  Serial.println(finger4Bent ? "Finger 4: BENT" : "Finger 4: STRAIGHT");
  
  // Check orientation (palm facing up/down/forward)
  bool palmDown = accelZ < -5.0;
  bool palmUp = accelZ > 5.0;
  bool palmForward = abs(accelY) > 7.0;
  
  // Map gestures to characters - using the hardcoded calibration values
  String detectedChar = "";
  
  // Hello - All Finger Straight
  if (!finger1Bent && !finger2Bent && !finger3Bent && !finger4Bent) {
    detectedChar = "Hello ";
  }
  // Bye - All Bent
  else if (finger1Bent && finger2Bent && !finger3Bent && !finger4Bent) {
    detectedChar = "Bye ";
  }
  // Thank You-1st Bent
  else if (finger1Bent && !finger2Bent && !finger3Bent && !finger4Bent) {
    detectedChar = "Thank you ";
  }
  // Yes-2nd Bent
  else if (!finger1Bent && finger2Bent && !finger3Bent && !finger4Bent) {
    detectedChar = "Yes ";
  }
   // No-3nd Bent
  else if (!finger1Bent && !finger2Bent && finger3Bent && !finger4Bent) {
    detectedChar = "No ";
  }
    // ok-4nd Bent
  else if (!finger1Bent && !finger2Bent && !finger3Bent && finger4Bent) {
    detectedChar = "OK `";
  }
  // Space - Palm forward motion
  else if (palmForward && abs(accelY) > 6.0) {
    detectedChar = " ";
  }
  // Backspace - Palm down quick motion
  else if (palmDown && abs(accelX) > 5.0) {
    if (recognizedText.length() > 0) {
      recognizedText = recognizedText.substring(0, recognizedText.length() - 1);
    }
    Serial.println("Backspace detected!");
    return;
  }
  // Clear all text - Shake motion
  else if (abs(accelX) > 8.0 && abs(accelY) > 8.0) {
    recognizedText = "";
    Serial.println("Text cleared!");
    return;
  }
  
  // Only add the character if one was detected
  if (detectedChar != "") {
    recognizedText += detectedChar;
    currentCharacter = detectedChar;
    Serial.print("Detected character: ");
    Serial.println(detectedChar);
  } else {
    Serial.println("Unrecognized gesture");
    currentCharacter = "";
  }
}

void updateFirebase() {
  // Send current sensor data
  if (Firebase.RTDB.setFloat(&fbdo, "sensor/acceleration_x", accelX) &&
      Firebase.RTDB.setFloat(&fbdo, "sensor/acceleration_y", accelY) &&
      Firebase.RTDB.setFloat(&fbdo, "sensor/acceleration_z", accelZ)) {
    //Serial.println("Accelerometer data sent to Firebase.");
  }

  // Send flex sensor data
  for (int i = 0; i < 4; i++) {
    String path = "sensor/flex" + String(i + 1);
    if (Firebase.RTDB.setInt(&fbdo, path, flexValues[i])) {
      //Serial.print("Flex Sensor ");
      //Serial.print(i + 1);
      //Serial.println(" data sent to Firebase.");
    } else {
      Serial.print("Failed to send Flex Sensor data. Reason: ");
      Serial.println(fbdo.errorReason());
    }
  }
  
  // Send recognized text
  if (recognizedText != "") {
    if (Firebase.RTDB.setString(&fbdo, "gesture/text", recognizedText)) {
    } else {
      Serial.print("Failed to send text data. Reason: ");
      Serial.println(fbdo.errorReason());
    }
  }
  
  // Send latest detected character
  if (currentCharacter != "") {
    if (Firebase.RTDB.setString(&fbdo, "gesture/latest_char", currentCharacter)) {
      Serial.println("Latest character sent to Firebase: " + currentCharacter);
      currentCharacter = ""; // Reset after sending
    }
  }
  
}
