#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aqua_robot_line_trace/Line2d.h"
#include "aqua_robot_front_camera_imageproc/Gate.h"
#include "aqua_robot_messages/MotorVelocity.h"
#include <cmath>

const double PI = 3.14;
ros::Publisher pub;
aqua_robot_front_camera_imageproc::Gate gate;

void line2dCallBack(const aqua_robot_line_trace::Line2dConstPtr msg)
{
  aqua_robot_messages::MotorVelocity order;
  if(gate.detected == false) {
    order.motor_vertical_left = 255;
    order.motor_vertical_right = 255;
  } else if(gate.y < msg->image_height / 2){
    order.motor_vertical_left = 0;
    order.motor_vertical_right = 0;
  } else {
    order.motor_vertical_left = 255;
    order.motor_vertical_right = 255;
  }
  if(msg->vx == 0) {
    order.motor_horizontal_left = 255;
    order.motor_horizontal_right = 255;
  } else {
    double slope = msg->vy / msg->vx;
    double intercept = msg->py - slope * msg->px;

    double distance = (msg->image_width / 2.0) - ((msg->image_height / 2.0 - intercept) / slope);

    // 線がロボットの右にあるとき
    double slope_rad = atan(slope);
    if(distance >= 0) {
      if(slope_rad > 60.0 / 180.0 * PI) {
        order.motor_horizontal_left = 0;
        order.motor_horizontal_right = 255;
      }else if(slope_rad < 30.0 / 180.0 * PI){ // ロボットが線に向かっていない場合
        order.motor_horizontal_left = 255;
        order.motor_horizontal_right = 0;
      }else{
        order.motor_horizontal_left = 255;
        order.motor_horizontal_right = 255;
      }
      // 線がロボットの左にあるとき
    }else{
      if(slope_rad < 30.0 / 180.0 * PI) {
        order.motor_horizontal_left = 255;
        order.motor_horizontal_right = 0;
      }else if(slope_rad > 60.0 / 180.0 * PI){ // ロボットが線に向かっていない場合
        order.motor_horizontal_left = 0;
        order.motor_horizontal_right = 255;
      }else{
        order.motor_horizontal_left = 255;
        order.motor_horizontal_right = 255;
      }
    }
  }

  pub.publish(order);
}

void gateCallBack(const aqua_robot_front_camera_imageproc::GateConstPtr& msg) {
  gate = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aqua_robot_autonomous_node");
  ros::NodeHandle nh;

  gate.detected = false;
  pub = nh.advertise<aqua_robot_messages::MotorVelocity>("motor_velocity", 100);
  ros::Subscriber sub = nh.subscribe("line", 1, line2dCallBack);
  ros::Subscriber sub_gate = nh.subscribe("gate", 1, gateCallBack);

  ros::spin();

  return 0;
}

