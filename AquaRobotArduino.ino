/*
 * 電子回路班 水中ロボット用 ROSノードスケッチ
 * TODO: 構造体・enumなどを使ってコードをクリーンにする
 */

#include <ros.h>
#include <aqua_robot_messages/MotorVelocity.h>
#include <aqua_robot_messages/State.h>

#include "Wire.h"
#include "Servo.h"
#include "MPU6050_6Axis_MotionApps20.h"

void setMotorVelocity(const aqua_robot_messages::MotorVelocity& motor_velocity);
void setupESC();

const unsigned int ESC_PIN_VERTICAL_RIGHT = 9;
const unsigned int ESC_PIN_VERTICAL_LEFT = 5;
const unsigned int ESC_PIN_HORIZONTAL_RIGHT = 6;
const unsigned int ESC_PIN_HORIZONTAL_LEFT = 3;
const unsigned int ESC_PINS[4] = {ESC_PIN_VERTICAL_RIGHT, ESC_PIN_VERTICAL_LEFT, ESC_PIN_HORIZONTAL_RIGHT, ESC_PIN_HORIZONTAL_LEFT};

const unsigned int BATTERY_PIN = 0;
const unsigned int BATTERY_CHECK_PIN = 2;

const unsigned int ESC_SERVO_INDEX_VERTICAL_RIGHT = 0;
const unsigned int ESC_SERVO_INDEX_VERTICAL_LEFT = 1;
const unsigned int ESC_SERVO_INDEX_HORIZONTAL_RIGHT = 2;
const unsigned int ESC_SERVO_INDEX_HORIZONTAL_LEFT = 3;

Servo esc[4];

aqua_robot_messages::State stateMsg;

// デフォルトのノード設定だとメモリを食い過ぎて死ぬので、制限をかけておく
// ros::NodeHandle nodeHandle;
ros::NodeHandle_<ArduinoHardware, 1, 1, 16, 64> nodeHandle; 
ros::Subscriber<aqua_robot_messages::MotorVelocity> motorSubscriber("set_motor_velocity", &setMotorVelocity);
ros::Publisher statePublisher("status", &stateMsg);

MPU6050 mpu;
uint16_t mpuPacketSize;

void setup() {
  nodeHandle.initNode();
  nodeHandle.advertise(statePublisher);
  nodeHandle.subscribe(motorSubscriber);
  
  Wire.begin();
  
  for(int i = 0; i < sizeof(ESC_PINS) / sizeof(unsigned int); i++) {
    pinMode(ESC_PINS[i], OUTPUT);
    esc[i].attach(ESC_PINS[i]);
  }
  // setupESC(); // ESCの設定が必要ならコメント解除
  // モータを停止状態に、ESCへの入力が0の場合ブザーが鳴る
  // クライアントからの入力が来なければ、ブザーが鳴り続ける
  for(int i = 0; i < sizeof(ESC_PINS) / sizeof(unsigned int); i++)
    esc[i].writeMicroseconds(0);
  
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

// ESCの最大出力・最小出力に対応する入力パルス波形を設定する関数
void setupESC()
{
  // ESCの電源を切った状態で、最大出力時のパルス波形を入力
  for(int i = 0; i < sizeof(ESC_PINS) / sizeof(unsigned int); i++){
    esc[i].writeMicroseconds(2000);
  }
  
  // ESCの電源が入るまで待機
  pinMode(BATTERY_CHECK_PIN,INPUT);
  while(digitalRead(BATTERY_CHECK_PIN) == 0);
  
  delay(2000);
  
  // 最小出力（モータは無回転）に対応するパルス波形を入力 これ以下のパルス波系はすべて無回転として扱われる
  // また、0を入力するとブザー音が鳴る
  for(int i = 0; i < sizeof(ESC_PINS) / sizeof(unsigned int); i++){
    esc[i].writeMicroseconds(1000);
  }
  
  delay(3000);
}

void setMotorVelocity(const aqua_robot_messages::MotorVelocity& motor_velocity) {
  // writeMicroseconds(2000)がESCの最大出力
  esc[ESC_SERVO_INDEX_VERTICAL_RIGHT].writeMicroseconds(1000 + (motor_velocity.motor_vertical_right * 1000 / 255));
  esc[ESC_SERVO_INDEX_VERTICAL_LEFT].writeMicroseconds(1000 + (motor_velocity.motor_vertical_left * 1000 / 255));
  esc[ESC_SERVO_INDEX_HORIZONTAL_RIGHT].writeMicroseconds(1000 + (motor_velocity.motor_horizontal_right * 1000 / 255));
  esc[ESC_SERVO_INDEX_HORIZONTAL_LEFT].writeMicroseconds(1000 + (motor_velocity.motor_horizontal_left * 1000 / 255));
}
