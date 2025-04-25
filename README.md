# Sign Language to Text Converter Smart Hand Glove (ESP32 + Flex Sensors + MPU6050)

This repository contains the source code and documentation for **Sign Language to Text Converter Hand Glove**,  
where an **ESP32 microcontroller** interfaces with **4 Flex Sensors** and an **MPU6050 IMU** to recognize ASL hand gestures.  
Detected gestures are sent to a **Firebase Realtime Database**, and corresponding text is displayed live on a **GitHub Pages hosted website**.

## 🚀 Features
- **Gesture Recognition**: Detects 6 Sign Language Gestures using flex sensors (finger bends) and MPU6050 (orientation).
- **ESP32 Firmware**: Reads sensor data, applies gesture logic, and pushes recognized text to Firebase.
- **Firebase Realtime Sync**: Uses Firebase Realtime Database for storing sensor readings and text output.
- **Live Web Display**: A website hosted using GitHub Pages displays recognized text in real time.
- **Compact Wearable Design**: Glove-mounted sensors provide a user-friendly and portable experience.
- 
## 🧰 Components Used
- **ESP32 Dev Board**
- **4x Flex Sensors** (attached to index, middle, ring, and little fingers)
- **MPU6050** (3-axis gyroscope + 3-axis accelerometer)
- **Firebase Realtime Database**
- **GitHub Pages**
- **Jumper Wires**, **Glove**, and **Breadboard**
- 
## 📁 Project Structure
- **Firmware**: ESP32 code to read sensor data and push to Firebase 
- **Project Photograph**:Demonstrates hardware setup, wiring, and Firebase realtime database and Website
