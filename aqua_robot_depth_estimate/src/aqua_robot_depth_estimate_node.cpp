#include "ros/ros.h"
#include "aqua_robot_messages/State.h"
#include "aqua_robot_depth_estimate/Depth.h"
#include <vector>
#include <iostream>

ros::Publisher pub;

ros::Time before;
ros::Time now;

std::vector<double> accel_z;

double position_z = 0;

double sample_number;
double accel_z_offset;

bool first = true;
double velocity_z = 0;

void stateCallback(const aqua_robot_messages::StateConstPtr& msg)
{
  /*
  double accel = (msg->accel).z - accel_z_offset;
  now = ros::Time::now();
  if(first) {
    first = false;
    before = now;
    accel_z_offset = (msg->accel).z;
    return;
  }
  ros::Duration dt = now - before;
  velocity += accel * dt.toSec();
  position_z += velocity * dt.toSec();
  */

  accel_z.push_back((msg->accel).z - accel_z_offset);
  now = ros::Time::now();

  if(accel_z.size() < sample_number) {
    before = now;
    return;
  }else if(accel_z.size() > sample_number) {
    accel_z.erase(accel_z.begin());
  }

  double accel_z_average = 0;
  for(int i = 0; i < accel_z.size(); i++)
    accel_z_average += accel_z[i];

  accel_z_average = accel_z_average / accel_z.size();
  ROS_INFO("accel average: %f", accel_z_average);

  ros::Duration dt = now - before;
  velocity_z += accel_z_average * dt.toSec();

  position_z += velocity_z * dt.toSec();

  aqua_robot_depth_estimate::Depth pub_msg;
  pub_msg.depth = position_z;
  pub_msg.velocity_z = velocity_z;
  /*
  aqua_robot_depth_estimate::Depth pub_msg;
  pub_msg.depth = position_z;
  pub_msg.velocity_z = velocity;
  */
  pub.publish(pub_msg);

  before = now;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aqua_robot_depth_estimate_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("state", 10, stateCallback);
  pub = nh.advertise<aqua_robot_depth_estimate::Depth>("depth", 10);

  ros::NodeHandle private_nh("~");
  accel_z_offset = private_nh.param("accel_z_offset", 0.263);
  sample_number = private_nh.param("sample_number", 10);

  ros::spin();

  return 0;
}
