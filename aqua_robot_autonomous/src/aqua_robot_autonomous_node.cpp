#include "ros/ros.h"
#include "std_msgs/String.h"
#include "aqua_robot_line_trace/Line2d.h"
#include "aqua_robot_messages/MotorVelocity.h"
#include <cmath>

const double PI = 3.14;
ros::Publisher pub;

void line2dCallBack(const aqua_robot_line_trace::Line2dConstPtr msg)
{
  double slope = msg->vy / msg->vx;
  double intercept = msg->py - slope * msg->px;

  double distance = (msg->image_width / 2.0) - ((msg->image_height / 2.0 - intercept) / slope);

  aqua_robot_messages::MotorVelocity order;
  order.motor_vertical_left = 0;
  order.motor_vertical_right = 0;
  // 線がロボットの左にあるとき
  if(distance >= 0) {
    if(atan(slope) > PI / 4) {
      order.motor_horizontal_left = 255;
      order.motor_horizontal_right = 0;
    }else{ // ロボットが線に向かっていない場合
      order.motor_horizontal_left = 0;
      order.motor_horizontal_right = 255;
    }
  // 線がロボットの右にあるとき
  }else{
    if(atan(slope) < PI / 4) {
      order.motor_horizontal_left = 0;
      order.motor_horizontal_right = 255;
    }else{ // ロボットが線に向かっていない場合
      order.motor_horizontal_left = 255;
      order.motor_horizontal_right = 0;
    }
  }

  pub.publish(order);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aqua_robot_autonomous_node");
  ros::NodeHandle nh;

  pub = nh.advertise<aqua_robot_messages::MotorVelocity>("motor_velocity", 100);
  ros::Subscriber sub = nh.subscribe("line", 1, line2dCallBack);

  ros::spin();

  return 0;
}

