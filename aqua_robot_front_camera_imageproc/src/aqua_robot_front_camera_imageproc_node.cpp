#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "aqua_robot_front_camera_imageproc/Gate.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

ros::Publisher gate_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv::Mat cv_input;
  try
  {
    cv_input = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat cv_hsv;
  cvtColor(cv_input, cv_hsv, CV_BGR2HSV);

  // 緑かそうでないかで二値化 閾値は要調整
  cv::Mat cv_binary_green;
  cv::inRange(cv_hsv, cv::Scalar(40, 70, 70), cv::Scalar(80, 255, 255), cv_binary_green);
  std::vector< std::vector<cv::Point> > contours;
  cv::findContours(cv_binary_green, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double max_length = 0;
  int max_length_contour = -1;
  // 長さが最大の輪郭を探す
  for(int i = 0; i < contours.size(); i++) {
    int tmp_length = cv::arcLength(contours[i], true);

    if(max_length < tmp_length) {
      max_length = tmp_length;
      max_length_contour = i;
    }
  }

  aqua_robot_front_camera_imageproc::Gate pub_msg;
  pub_msg.detected = false;
  if(max_length_contour != -1 && max_length > 400) {
    cv::Moments moments = cv::moments(contours[max_length_contour]);
    cv::Point2f mass_center = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
    pub_msg.detected = true;
    pub_msg.x = mass_center.x;
    pub_msg.y = mass_center.y;
  }
  gate_pub.publish(pub_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aqua_robot_line_trace");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("image", 1, imageCallback);
  gate_pub = nh.advertise<aqua_robot_front_camera_imageproc::Gate>("gate", 100);

  ros::spin();

  return 0;
}
