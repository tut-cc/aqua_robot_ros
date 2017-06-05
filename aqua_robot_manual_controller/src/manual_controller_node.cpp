#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <aqua_robot_messages/MotorVelocity.h>

// Joyメッセージの要素Axisのインデックス
// 参考：http://wiki.ros.org/joy#Microsoft_Xbox_360_Wired_Controller_for_Linux
const unsigned int JOY_AXIS_LEFT_STICK_UP_DOWN = 1;
const unsigned int JOY_AXIS_RIGHT_STICK_UP_DOWN = 4;
const unsigned int JOY_AXIS_LEFT_TRIGGER = 2;
const unsigned int JOY_AXIS_RIGHT_TRIGGER = 5;

const unsigned int MOTOR_VEROCITY_MAX = 255;

aqua_robot_messages::MotorVelocity motor_msg;

void joyCallback(const sensor_msgs::Joy::ConstPtr);

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "aqua_robot_manual_controller");
  ros::NodeHandle node_handle;
  
  // カメラ画像は60fps以下のため、Arduinoへの命令送信周期も60Hz程度で十分
  ros::Rate publish_rate(60);
  
  motor_msg.motor_horizontal_left = 0;
  motor_msg.motor_horizontal_right =0;
  motor_msg.motor_vertical_left = 0;
  motor_msg.motor_vertical_right = 0;
  
  // キューサイズは適当に決めた
  ros::Publisher motor_publisher = node_handle.advertise<aqua_robot_messages::MotorVelocity>("set_motor_velocity", 10);
  ros::Subscriber joy_subscriber = node_handle.subscribe("joy", 10, joyCallback);
  
  ROS_INFO_STREAM("publish: set motor velocity node: " << motor_publisher.getTopic());
  ROS_INFO_STREAM("subscribe: xbox controller node: " << joy_subscriber.getTopic());
  
  while(ros::ok()) {
    motor_publisher.publish(motor_msg);
    ros::spinOnce();
    publish_rate.sleep();
  }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr joy_msg_ptr) {
  // 右スティックの前方向への傾きで潜航速度を制御する
  double right_stick_up_down = joy_msg_ptr->axes[JOY_AXIS_RIGHT_STICK_UP_DOWN];
  if(right_stick_up_down > 0) {
    motor_msg.motor_vertical_left = right_stick_up_down * MOTOR_VEROCITY_MAX;
    motor_msg.motor_vertical_right = right_stick_up_down * MOTOR_VEROCITY_MAX;
  } else {
    motor_msg.motor_vertical_left = 0;
    motor_msg.motor_vertical_right = 0;
  }
  
  // 左スティックの傾きで前進速度を制御する
  // また、左スティックに入力が無い場合は、左右のトリガーで水平方向の左右モータをそれぞれ制御する
  double left_stick_up_down = joy_msg_ptr->axes[JOY_AXIS_LEFT_STICK_UP_DOWN];
  if(left_stick_up_down > 0) {
    motor_msg.motor_horizontal_left = left_stick_up_down * MOTOR_VEROCITY_MAX;
    motor_msg.motor_horizontal_right = left_stick_up_down * MOTOR_VEROCITY_MAX;
  } else {    
    // 左右トリガーの値は無入力状態で1.0、強く引くほど-1.0に近づく
    // このままだと扱い辛いので0（無入力状態）~1.0（最大限引いた状態）に正規化する
    double left_trigger = 1.0 - ((1.0 + joy_msg_ptr->axes[JOY_AXIS_LEFT_TRIGGER]) / 2.0);
    motor_msg.motor_horizontal_left = left_trigger * MOTOR_VEROCITY_MAX;
    double right_trigger = 1.0 - ((1.0 + joy_msg_ptr->axes[JOY_AXIS_RIGHT_TRIGGER]) / 2.0);
    motor_msg.motor_horizontal_right = right_trigger * MOTOR_VEROCITY_MAX;
  }
}