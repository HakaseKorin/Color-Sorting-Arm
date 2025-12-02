#include "Config.h"
#include "Hiwonder.hpp"
#include "Robot_arm.hpp"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

LeArm_t arm;

BusServo_t claw;          // servo 1
BusServo_t joint;         // servo 4
BusServo_t claw_joint;    // servo 2
BusServo_t claw_swivel;   // servo 3
BusServo_t base;          // servo 5
BusServo_t base_swivel;   // servo 6
Button_t button;

volatile uint8_t pendingCommand = 0;

// transforms input from RPi4 input to ascii values eg 1->49, 2->50 3->51... double check on expansion
// Serial outputs are on 9600
void routine(uint8_t command) {
  switch (command) {
    case 49: // 1
      Serial.println("Running: PICKUP");
      pickup(20,0);
      break;

    case 50: // 2
      Serial.println("Running: DROPOFF");
      dropoff(10,0);
      break;

    case 51: // 3
      Serial.println("Running: DELIVER ZONE 1");
      dropoff(10,-10);
      break;

    case 52: // 4
      Serial.println("Running: DELIVER ZONE 2");
      dropoff(5,20);
      break;

    case 53: // 5
      Serial.println("Running: DELIVER ZONE 3");
      dropoff(5,-20);
      break;

    case 54: // 6
      Serial.println("Running: RETRIVE ZONE 1");
      retrive(10,-10);
      break;

    case 55: // 7
      Serial.println("Running: RETRIVE ZONE 2");
      retrive(5,20);
      break;
    
    case 56: // 8
      Serial.println("Running: RETRIVE ZONE 3");
      retrive(5,-20);
      break;

    default:
      Serial.println("Invalid command");
      break;
  }
}

// BLE SERVER Callback for whenever data is received from the Pi
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {

    String rx = pCharacteristic->getValue();

    if (rx.length() == 0) return;

    Serial.print("Received data: ");
    Serial.println(rx.c_str());

    if (rx.length() == 1) {
      pendingCommand = rx[0];
      Serial.printf("Stored command: %d\n", pendingCommand);
    } else {
      Serial.println("Multi-byte command detected");
    }
  }
};

void setup() {
  delay(1000);
  pinMode(IO_BLE_CTL, OUTPUT);
  digitalWrite(IO_BLE_CTL, LOW);  // Set the Bluetooth control pin to low to cut off the Bluetooth module power (设置蓝牙控制引脚为低电平时，断开蓝牙模块电源)
  
  arm.init();
  Serial1.begin(115200 ,SERIAL_8N1 , BUS_RX , BUS_TX);
  claw.init(&Serial1);
  joint.init(&Serial1);
  claw_joint.init(&Serial1);
  claw_swivel.init(&Serial1);
  base.init(&Serial1);
  base_swivel.init(&Serial1);
  button.init();
  
  Serial.begin(9600);

  // BLE SERVER initializes Server
  BLEDevice::init("ESP32_Server");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
  );

  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  BLEDevice::startAdvertising();

  Serial.println("BLE Server Ready. Waiting for client writes...");

  delay(2000);
}

void coord_movement(int x, int y, int z) {
  // uint8_t coordinate_set(float target_x,float target_y,float target_z,float pitch,float min_pitch,float max_pitch,uint32_t time);
  arm.coordinate_set(x,y,z,0,-90,180,1000);
  delay(2000);
}

void pickup(int x, int y) {
  // claw occasionally doesnt completely close or open, best guess is a faulty wire/connection
  arm.coordinate_set(x,y,-6,0,-90,90,1000);
  delay(3000);
  claw.set_angle(1,180,100);
  delay(6000); // extra delay is to ensure that its able to grab the object
  arm.coordinate_set(x,y,20,0,-90,90,1000);
  delay(3000);
}

void pickup_from_sensor(int x, int y) {
  arm.coordinate_set(x,y,0,0,-90,90,1000);
  delay(3000);
  claw.set_angle(1,180,100);
  delay(6000); // extra delay is to ensure that its able to grab the object
  arm.coordinate_set(x,y,20,0,-90,90,1000);
  delay(3000);
}

void dropoff(int x, int y) {
  arm.coordinate_set(x,y,20,0,-90,90,1000);
  delay(3000);
  arm.coordinate_set(x,y,-5,0,-90,90,1000);
  delay(3000);
  claw.set_angle(1,0,100);
  delay(3000);
  arm.coordinate_set(x,y,20,0,-90,90,1000);
  delay(3000);
}

void retrive(int x, int y) {
  pickup(x,y);
  dropoff(10,0);
}

void loop() {

  if (pendingCommand != 0) {
    uint8_t cmd = pendingCommand;
    pendingCommand = 0;

    routine(cmd);  // Safe to run long servos here
  }

  delay(10); // Keeps BLE alive
}
