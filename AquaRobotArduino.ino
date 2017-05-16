/*
 * 電子回路班 水中ロボット用 ROSノードスケッチ
 * TODO: モーター名を番号ではなく、位置がわかるようなものにする
 */

#include <ros.h>
#include <aqua_robot_messages/MotorVelocity.h>
#include <aqua_robot_messages/State.h>

#include "Wire.h"
#include "Servo.h"
#include "MPU6050_6Axis_MotionApps20.h"

void setMotorVelocity(const aqua_robot_messages::MotorVelocity& motor_velocity);

const unsigned int MOTOR_PINS[4] = {3, 5, 6, 9};
const unsigned int BATTERY_PIN = 0;
const unsigned int BATTERY_CHECK_PIN = 2;

Servo esc[4];

aqua_robot_messages::State stateMsg;

// デフォルトのノード設定だとメモリを食い過ぎて死ぬので、制限をかけておく
// ros::NodeHandle nodeHandle;
ros::NodeHandle_<ArduinoHardware, 1, 1, 80, 250> nodeHandle; 
ros::Subscriber<aqua_robot_messages::MotorVelocity> motorSubscriber("set_motor_velocity", &setMotorVelocity);
ros::Publisher statePublisher("status", &stateMsg);

MPU6050 mpu;
uint16_t mpuPacketSize;

void setup() {
  nodeHandle.initNode();
  nodeHandle.advertise(statePublisher);
  nodeHandle.subscribe(motorSubscriber);
  
  Wire.begin();
  
  // ESCの初期化 ESCへの入力が0の場合、モータは動かずにブザーが鳴る
  for(int i = 0; i < sizeof(MOTOR_PINS) / sizeof(unsigned int); i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    esc[i].attach(MOTOR_PINS[i]);
    esc[i].writeMicroseconds(0);
  }
  
  // バッテリがオンになるまで待機？
  pinMode(BATTERY_CHECK_PIN, INPUT);
  while(digitalRead(BATTERY_CHECK_PIN) == 0);

  // ESCが1000以下の場合、モータは停止したまま
  for(int i = 0; i < sizeof(MOTOR_PINS) / sizeof(unsigned int); i++) {
    esc[i].writeMicroseconds(1000);
  }
  
  // MPUの初期化
  mpu.initialize();
  mpu.dmpInitialize();
  
  // これらのオフセット値はサンプルからそのままコピーしてきたもの
  // 有効・正確であるかは未検証
  mpu.setXGyroOffset(270);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  
  mpu.setDMPEnabled(true);
  
  mpuPacketSize = mpu.dmpGetFIFOPacketSize();
}

void loop() {
  stateMsg.battery = analogRead(BATTERY_PIN) / 1023.0 * 5.0;
  
  // MPUのFIFOにデータが貯まるまで待機
  mpu.resetFIFO();
  for(uint16_t fifoCount = mpu.getFIFOCount(); fifoCount < mpuPacketSize; fifoCount = mpu.getFIFOCount()){
  }
  uint8_t fifoBuffer[64];
  mpu.getFIFOBytes(fifoBuffer, mpuPacketSize);
  
  Quaternion quaternion;
  VectorFloat gravity;
  float yawPitchRoll[3];
  mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &quaternion);
  mpu.dmpGetYawPitchRoll(yawPitchRoll, &quaternion, &gravity);
  stateMsg.yaw = yawPitchRoll[0] * 180 / M_PI;
  stateMsg.pitch = yawPitchRoll[1] * 180 / M_PI;
  stateMsg.roll = yawPitchRoll[2] * 180 / M_PI;
  
  VectorInt16 accelWithGravity;
  VectorInt16 accel;
  mpu.dmpGetAccel(&accelWithGravity, fifoBuffer);
  mpu.dmpGetLinearAccel(&accel, &accelWithGravity, &gravity);
  stateMsg.accel.x = accel.x / 8192.0;
  stateMsg.accel.y = accel.y / 8192.0;
  stateMsg.accel.z = accel.z / 8192.0;
  
  VectorInt16 angularVelocity;
  mpu.dmpGetGyro(&angularVelocity, fifoBuffer);
  stateMsg.angular_velocity.x = angularVelocity.x / 16.4;
  stateMsg.angular_velocity.y = angularVelocity.y / 16.4;
  stateMsg.angular_velocity.z = angularVelocity.z / 16.4;
  
  statePublisher.publish(&stateMsg);
  nodeHandle.spinOnce();
}

void setMotorVelocity(const aqua_robot_messages::MotorVelocity& motor_velocity) {
  // writeMicroseconds(2000)がESCの最大出力
  esc[0].writeMicroseconds(1000 + (motor_velocity.motor1 * 1000 / 255));
  esc[1].writeMicroseconds(1000 + (motor_velocity.motor2 * 1000 / 255));
  esc[2].writeMicroseconds(1000 + (motor_velocity.motor3 * 1000 / 255));
  esc[3].writeMicroseconds(1000 + (motor_velocity.motor4 * 1000 / 255));
}
