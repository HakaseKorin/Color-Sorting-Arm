#include "Config.h"
#include "Hiwonder.hpp"
#include "Robot_arm.hpp"

LeArm_t arm;
/*
servo 6 - base-swivel
servo 5 - base
servo 4 - joint
servo 3 - claw-swivel
servo 2 - claw-joint
servo 1 - claw
*/
BusServo_t claw;
BusServo_t joint;
BusServo_t claw_joint;
BusServo_t claw_swivel;
BusServo_t base;
BusServo_t base_swivel;
Button_t button;

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
  delay(2000);
}

void coord_movement(byte x, byte y, byte z) {
  // Check range
  // hypothesis ( x y z set in cm), coordinates based off the claw of the robot
  // uint8_t coordinate_set(float target_x,float target_y,float target_z,float pitch,float min_pitch,float max_pitch,uint32_t time);
  arm.coordinate_set(x,y,z,0,-90,90,1000);
  delay(2000);
}

void claw_op(bool isOpen) {
  if (isOpen) {
    // 
    claw.set_angle(1, 0, 1000);
  } else {
    claw.set_angle(1, 180, 1000);
  }
  delay(2000);

}

void base_rotation(byte deg) {
  // 0 - 180
  base_swivel.set_angle(6, deg, 1000);
  delay(2000);
}

void pickup() {
  // pick up program
  // claw occasionally doesnt completely close or open, best guess is a faulty wire/connection
  arm.coordinate_set(20,0,-6,0,-90,90,1000);
  delay(3000);
  claw.set_angle(1,180,100);
  delay(3000);
}
void dropoff(byte x, byte y, byte z) {
  arm.coordinate_set(x,y,z,0,-90,90,1000);
  delay(3000);
  claw.set_angle(1,0,100);
  delay(3000);
}

void routine(byte command) {
  switch(command) {
    // pick up
    case 1: pickup();
    // open claw
    // move to ...
    // reset
  }
}

void loop() {
  if(button.read(1) == 1) {
    pickup();
  }
  if(button.read(2) == 1) {
    dropoff();
  }
  delay(50);
  //pickup();
  //dropoff(20,0,20);
}

// robot arm drops down after powering down
// no negatives on x,y,z in coordinate_set(x,y,z,0,-90,90,1000);
// coordinate set better for delivery into specific space
