# 水中ロボット 手動コントローラノード

Xbox 360有線コントローラを用いて、水中ロボットを操作するためのノードです

joyノードからの入力をMotorVelocityメッセージに変換して、Arduino側に渡します

## このノードを含めた、水中ロボット ROS システムの起動方法

USER_NAMEとPASSWORDに適切な値を設定して、以下のコマンドを実行してください。

    ROSLAUNCH_SSH_UNKNOWN=1 ROS_IP=192.168.1.254 roslaunch aqua_robot_manual_controller manual_controller.launch raspberry_pi3_username:=<USER_NAME> raspberry_pi3_password:=<PASSWORD>

## 操作方法

- 前進：左スティックを↑に傾ける。
- 水平左モータの回転：左トリガー（ただし、前進中は無効）
- 水平右モータの回転：右トリガー（ただし、前進中は無効）
- 潜航：右スティックを↑に傾ける
- 緊急停止：Aボタン
- 緊急停止解除：XBOXボタン
