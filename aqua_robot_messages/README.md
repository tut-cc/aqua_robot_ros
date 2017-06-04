# aqua_robot_messages

水中ロボットをROSで制御するときに用いるメッセージ型

Arduino用のヘッダファイルを生成する場合、以下のリンクを参照

http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Install_ros_lib_into_the_Arduino_Environment

## msg/MotorVelocity.msg

各モータへの指令値

## msg/State.msg

水中ロボットの状態（各センサーの値）

- 加速度
- 角速度
- バッテリ電圧
- ピッチ、ロール、ヨー