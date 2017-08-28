/*
 * 電子回路班 水中ロボット用 ROSノードスケッチ
 */

#include <ros.h>
#include <aqua_robot_messages/MotorVelocity.h>
#include <aqua_robot_messages/State.h>

#include "Wire.h"
#include "Servo.h"
#include "MPU6050_6Axis_MotionApps20.h"

void setMotorVelocity(const aqua_robot_messages::MotorVelocity& motor_velocity);
void setESCMinMax();
void initMPU();
bool getMPUData();

const unsigned int ESC_INDEX_VERTICAL_RIGHT = 0;
const unsigned int ESC_INDEX_VERTICAL_LEFT = 1;
const unsigned int ESC_INDEX_HORIZONTAL_RIGHT = 2;
const unsigned int ESC_INDEX_HORIZONTAL_LEFT = 3;
// 上記のESC_INDEXと対応させるよう注意
const unsigned int ESC_PINS[] = {9, 6, 5, 3};
const unsigned int ESC_NUM = 4;

const unsigned int ESC_INPUT_MAX = 2000;
const unsigned int ESC_INPUT_MIN = 1000;

const unsigned int BATTERY0_PIN = 0;
const unsigned int BATTERY1_PIN = 1;
const unsigned int ESC_CHECK_PIN = 2;

// MotorVelocityの入力が途絶えてから、モータを停止するまでの時間
const unsigned int MOTOR_STOP_TIME_MILLISECOND = 500;

// この時間内にMPUから新しいデータを取得できなかった場合、MPUをリセットする
const unsigned int MPU_RESET_TIME_MILLISECOND = 100;

unsigned long lastMotorCommandTime = 0;

Servo esc[4];

aqua_robot_messages::State stateMsg;

// デフォルトのノード設定だとメモリを食い過ぎて死ぬので、制限をかけておく
// ros::NodeHandle nodeHandle;
ros::NodeHandle_<ArduinoHardware, 1, 1, 150, 150> nodeHandle;
ros::Subscriber<aqua_robot_messages::MotorVelocity> motorSubscriber("set_motor_velocity", &setMotorVelocity);
ros::Publisher statePublisher("status", &stateMsg);

MPU6050 mpu;
uint16_t mpuPacketSize;
unsigned long lastMpuDateTime = 0;

void setup() {
  nodeHandle.initNode();
  nodeHandle.advertise(statePublisher);
  nodeHandle.subscribe(motorSubscriber);

  Wire.begin();

  for(int i = 0; i < ESC_NUM; i++) {
    pinMode(ESC_PINS[i], OUTPUT);
    esc[i].attach(ESC_PINS[i]);
  }
  //setESCMinMax(); // ESCの最大・最小入力値の設定が必要ならコメント解除
  // モータを停止状態に、ESCへの入力が0の場合ブザーが鳴る
  // クライアントからの入力が来なければ、ブザーが鳴り続ける
  for(int i = 0; i < ESC_NUM; i++)
    esc[i].writeMicroseconds(0);

  initMPU();
}

void loop() {
  // 各センサデータの送信
  if(getMPUData()) {
    stateMsg.battery0 = analogRead(BATTERY0_PIN) / 1023.0 * 5.0 * 3.0; // 分圧：2k 1k
    stateMsg.battery1 = analogRead(BATTERY1_PIN) / 1023.0 * 5.0 * 2.5; // 分圧：15k 10k
    statePublisher.publish(&stateMsg);
  }

  // 最後のモータ指令から、MOTOR_STOP_TIME_MILLISECOND以上経過していればモータを停止する
  unsigned long now = millis();
  if(now < lastMotorCommandTime) { // オーバーフロー時
    lastMotorCommandTime = 0;
  }
  if(now - lastMotorCommandTime > MOTOR_STOP_TIME_MILLISECOND) {
    for(int i = 0; i < ESC_NUM; i++)
      esc[i].writeMicroseconds(0);
  }

  nodeHandle.spinOnce();
}

// ESCの最大出力・最小出力に対応する入力パルス波形を設定する関数
void setESCMinMax()
{
  // ESCの電源を切った状態で、最大出力時のパルス波形を入力
  for(int i = 0; i < ESC_NUM; i++){
    esc[i].writeMicroseconds(ESC_INPUT_MAX);
  }

  // ESCの電源が入るまで待機
  //pinMode(ESC_CHECK_PIN,INPUT);
  //while(digitalRead(ESC_CHECK_PIN) == 0);

  // この間にESCに電源をつなぐ
  delay(15000);

  // 最小出力（モータは無回転）に対応するパルス波形を入力 これ以下のパルス波系はすべて無回転として扱われる
  // また、0を入力するとブザー音が鳴る
  for(int i = 0; i < ESC_NUM; i++){
    esc[i].writeMicroseconds(ESC_INPUT_MIN);
  }

  delay(20000);
}

void setMotorVelocity(const aqua_robot_messages::MotorVelocity& motor_velocity) {
  unsigned int motorVelocity[ESC_NUM];
  motorVelocity[ESC_INDEX_VERTICAL_RIGHT] = motor_velocity.motor_vertical_right;
  motorVelocity[ESC_INDEX_VERTICAL_LEFT] = motor_velocity.motor_vertical_left;
  motorVelocity[ESC_INDEX_HORIZONTAL_RIGHT] = motor_velocity.motor_horizontal_right;
  motorVelocity[ESC_INDEX_HORIZONTAL_LEFT] = motor_velocity.motor_horizontal_left;

  for(int i = 0; i < ESC_NUM; i++)
    esc[i].writeMicroseconds(ESC_INPUT_MIN + (motorVelocity[i] * (ESC_INPUT_MAX - ESC_INPUT_MIN) / 255));

  lastMotorCommandTime = millis();
}

void initMPU() {
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

  mpu.resetFIFO(); // FIFOをリセットする必要があるのか検証する必要あり

  lastMpuDateTime = millis();
}

// MPU6050より各種データを取得、publish用messageにセットする
bool getMPUData() {
  uint16_t fifoCount = mpu.getFIFOCount();
  /*
  String message("fifo:");
  message += fifoCount;
  message += " size:";
  message += mpuPacketSize;
  nodeHandle.loginfo(message.c_str());
  */
  if(fifoCount < mpuPacketSize) {
    unsigned long now = millis();
    if(now < lastMpuDateTime) // オーバーフロー時
      lastMpuDateTime = 0;

    if(now - lastMpuDateTime > MPU_RESET_TIME_MILLISECOND) {
      nodeHandle.loginfo("reset FIFO.");
      initMPU();
    }

    return false;
  }

  uint8_t fifoBuffer[64];
  mpu.getFIFOBytes(fifoBuffer, mpuPacketSize);

  mpu.resetFIFO(); // FIFOをリセットする必要があるのか検証する必要あり

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

  stateMsg.temperature = (mpu.getTemperature()+12412.0)/340.0;

  lastMpuDateTime = millis();

  return true;
}
