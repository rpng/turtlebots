#ifndef POSE_PUBLISHER_H_
#define POSE_PUBLISHER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <slam_apriltag_isam_single/PoseStampedArray.h>
#include <apriltags/TagDetector.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
/*
This class is used to process images sent from turtlebot
It will detect the apriltags on the image and out put the
tags information and the odom information at the time the
processed image is received
*/
class AprilTagDetector{
  AprilTags::TagDetector* tag_detector_;

 public:
  AprilTagDetector(ros::NodeHandle& nh);
  ~AprilTagDetector();

 private:
  // Subscribe the images
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
  // Subscribe turtlebot location
  void TurtleCallback(const nav_msgs::OdometryConstPtr& Odometry);
  // Subscribe camerainfo
  void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera);
 private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  // Publisher for the processed AprilTags image
  image_transport::Publisher image_pub_;
  // Subscriber for odom
  ros::Subscriber turtle_sub_;
  ros::Subscriber camera_info_;
  // Publisher for the tag detections with odom information of the turtlebot
  ros::Publisher pose_pub_;
  //broadcast tf info
  tf::TransformBroadcaster tf_pub_;
  nav_msgs::Odometry odom;
  //calibration for rgb camera
  float fx;
  float fy;
  float px;
  float py;
};




#endif //POSE_PUBLISHER_H_
