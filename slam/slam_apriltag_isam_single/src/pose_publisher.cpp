
#include "slam_apriltag_isam/pose_publisher.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  //uncomment following 2 line to create a cv window
  //cv::namedWindow("view", CV_WINDOW_AUTOSIZE);
  //cv::startWindowThread();

  AprilTagDetector detector(nh);
  ros::spin();
  //uncomment following line to close window when exit.
  //cv::destroyWindow("view");
}
