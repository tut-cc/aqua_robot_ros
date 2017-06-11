# AquaRobotArduino

電子回路班：水中ロボット内蔵Arduino用 ROSノードスケッチ

動作させるには以下のライブラリが必要

- i2cdev、MPU6050：https://github.com/jrowberg/i2cdevlib
- rosserial_arduino：http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

また、https://github.com/tut-cc/aqua_robot_messages から生成できるArduino向けヘッダも必要

## ノードの起動方法

以下を参照

http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World#Running_the_Code

## Publish Topic

state: MPU6050から取得したデータとバッテリ電圧、emergencyModeの状態

## Subscribe Topic

set\_motor\_velocity: 各モーターの回転数を指定
set\_emergency\_mode: emergencyModeのオン・オフ

## メモ

emergencyMode時には、全てのモータが停止して、ESCから定期的にブザー音が鳴ります。

また、set\_motor\_velocityからの受信が一定期間（ALLOW\_DISCONNECTED\_TIME）秒以上止まると、自動でemergencyModeが有効になります。

## TODO

- バッテリ電圧の取得テスト
- モーター回転のテスト
- emergencyModeの動作テスト
