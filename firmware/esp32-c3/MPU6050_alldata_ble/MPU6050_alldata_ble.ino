/*
  ESP32-C3 MPU6050 BLE Server

  This program reads accelerometer and gyroscope data from an MPU6050
  and sends it over Bluetooth Low Energy (BLE) using a notify characteristic.

  It emulates a common "UART-like" service.
*/

// Core MPU6050 libraries
#include <MPU6050_WE.h>
#include <Wire.h>

// Core BLE libraries for ESP32
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h> // For adding descriptors (e.g., for notifications)

// MPU6050 setup
#define MPU6050_ADDR 0x68
MPU6050_WE myMPU6050 = MPU6050_WE(MPU6050_ADDR);

// BLE Server setup
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL; // Characteristic to send data (Notify)
bool deviceConnected = false;
bool oldDeviceConnected = false;

// These UUIDs are commonly used for "Nordic UART Service"
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Server callbacks to manage connection state
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected - advertising again");
      // Immediately start advertising again
      pServer->getAdvertising()->start();
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");

  // --- MPU6050 Initialization ---
  Wire.begin(6, 7); // Your I2C pins
  if(!myMPU6050.init()){
    Serial.println("MPU6050 does not respond");
  }
  else{
    Serial.println("MPU6050 is connected");
  }
  
  Serial.println("Position you MPU6050 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6050.autoOffsets();
  Serial.println("Done!");

  myMPU6050.setGyrRange(MPU6050_GYRO_RANGE_250);
  myMPU6050.setAccRange(MPU6050_ACC_RANGE_2G);
  
  delay(200);

  // --- BLE Initialization ---
  Serial.println("Starting BLE server...");
  BLEDevice::init("HandTracker-MPU"); 

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic for sending data (TX)
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY // Allow notifications
                    );
  
  // Add a 2902 descriptor to the characteristic.
  // This is required for clients to register for notifications
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("Ready to connect. Advertising started.");
}

void loop() {
  
  // Check if a client is connected
  if (deviceConnected) {
    xyzFloat gValue = myMPU6050.getGValues(); // Accel data
    xyzFloat gyr = myMPU6050.getGyrValues();   // Gyro data

    // Format the data into a single string
    // Format: "ax,ay,az,gx,gy,gz"
    // Example: "0.01,-0.02,0.99,1.23,-0.45,0.05"
    char dataString[80]; // Buffer to hold the data
    snprintf(dataString, sizeof(dataString), "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", 
             gValue.x, gValue.y, gValue.z,
             gyr.x, gyr.y, gyr.z);

    // Set the characteristic value
    pTxCharacteristic->setValue(dataString);
    
    // Send the notification
    pTxCharacteristic->notify();
    
    // Print to serial monitor *only if* connected, for debugging
    if(oldDeviceConnected == false) {
      Serial.println("Client is now connected, sending data.");
      oldDeviceConnected = true;
    }
  }
  else {
    // Update connection status for debugging print
    if(oldDeviceConnected == true) {
      Serial.println("Client disconnected.");
      oldDeviceConnected = false;
    }
  }

  delay(10); // Maintain your 10ms loop delay
}
