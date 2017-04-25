# AquaRobotArduino

電子回路班：水中ロボット内蔵Arduino用 ROSノードスケッチ

動作させるには以下のライブラリが必要

- i2cdev、MPU6050：https://github.com/jrowberg/i2cdevlib
- rosserial_arduino：http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

また、https://github.com/tut-cc/aqua_robot_messages から生成できるArduino向けヘッダも必要

## Publish Topic

state: MPU6050から取得したデータとバッテリ電圧

## Subscribe Topic

set\_motor\_velocity: 各モーターの回転数を指定

## TODO

- バッテリ電圧の取得テスト
- モーター回転のテスト
