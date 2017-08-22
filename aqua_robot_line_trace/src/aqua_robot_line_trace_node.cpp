#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "aqua_robot_line_trace/Line2d.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

#define DEBUG_GUI // デバッグ用のGUIが不要ならコメントアウト

static const std::string OPENCV_WINDOW = "Image window";
ros::Publisher pub;

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
#ifdef DEBUG_GUI
  cv::Mat cv_out;
  cv_out = cv_input.clone();
#endif

  cv::Mat cv_blured;
  // ksize, sigma x/y,borderTypeを最適な値にする必要あるかも
  cv::GaussianBlur(cv_input, cv_blured, cv::Size(3,3),0,0);
  cv::Mat cv_gray;
  cv::cvtColor(cv_blured, cv_gray, cv::COLOR_BGR2GRAY);
  cv::Mat cv_canny;
  // thresholdを始めとして、引数を調整する必要あり
  cv::Canny(cv_gray, cv_canny, 100, 200);

  std::vector< std::vector<cv::Point> > contours;
  cv::findContours(cv_canny, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double max_length = 0;
  int max_length_contour = -1;
  // 長さが最大の輪郭を探す
  for(int i = 0; i < contours.size(); i++) {
    int tmp_length = cv::arcLength(contours[i], false);

    if(max_length < tmp_length) {
      max_length = tmp_length;
      max_length_contour = i;
    }
  }

  std::cout << "length" << max_length << std::endl;
  if(max_length_contour != -1 && max_length > std::min(msg->height, msg->width) / 2) {
    std::vector<double> fit_line;
    cv::fitLine(contours[max_length_contour], fit_line, cv::DIST_L2, 0, 0.01, 0.01);

    aqua_robot_line_trace::Line2d line;
    line.vx = fit_line[0];
    line.vy = fit_line[1];
    line.px = fit_line[2];
    line.py = fit_line[3];

    pub.publish(line);

#ifdef DEBUG_GUI
    drawContours(cv_out, contours, max_length_contour, cv::Scalar(255,0,0), 5);
    int image_max_side = std::max(msg->height, msg->width);
    cv::Point line_start, line_end;
    line_start.x = fit_line[2] - image_max_side * fit_line[0];
    line_start.y = fit_line[3] - image_max_side * fit_line[1];
    line_end.x = fit_line[2] + image_max_side * fit_line[0];
    line_end.y = fit_line[3] + image_max_side * fit_line[1];
    cv::clipLine(cv::Size(msg->width, msg->height), line_start, line_end);
    cv::line(cv_out, line_start, line_end, cv::Scalar(0,255,0), 5);
#endif
  }
#ifdef DEBUG_GUI
  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_out);
  cv::waitKey(3);
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aqua_robot_line_trace");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("image", 1, imageCallback);
  pub = nh.advertise<aqua_robot_line_trace::Line2d>("line", 100);

#ifdef DEBUG_GUI
  cv::namedWindow(OPENCV_WINDOW);
#endif

  ros::spin();

  return 0;
}
