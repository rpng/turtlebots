#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <slam_apriltag_isam_single/PoseStampedArray.h>
#include "apriltags/TagDetector.h"
#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"
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
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);//subscribe the images
  void TurtleCallback(const nav_msgs::OdometryConstPtr& Odometry);//subscribe turtlebot location
  void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera);//subscribe camerainfo
 private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;//publish the processed AprilTags image
  ros::Subscriber turtle_sub_;
  ros::Subscriber camera_info_;
  ros::Publisher pose_pub_;//publish the tag detections with odom information of the turtlebot
  tf::TransformBroadcaster tf_pub_;//broadcast tf info
  nav_msgs::Odometry odom;
  //calibration for rgb camera
  float fx;
  float fy;
  float px;
  float py;
};


AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh): it_(nh){
  AprilTags::TagCodes tag_codes = AprilTags::tagCodes36h11;//load the apriltags tag library
  tag_detector_ = new AprilTags::TagDetector(tag_codes);//add library to the detector
  //subscribe image from kinect
  image_sub_ = it_.subscribe("camera/rgb/image_raw", 1, &AprilTagDetector::ImageCallback, this);
  //advertise processed image
  image_pub_ = it_.advertise("tag_detections_image", 1);
  //subscribe the odometry
  turtle_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &AprilTagDetector::TurtleCallback, this);
  //subscribe CameraInfo
  camera_info_ = nh.subscribe<sensor_msgs::CameraInfo>("camera/rgb/camera_info", 1, &AprilTagDetector::CameraInfoCallback, this);
  //advertise tag position, orientation and odometry
  pose_pub_ = nh.advertise<slam_apriltag_isam_single::PoseStampedArray>("tag_detections_pose", 1);
  //initialize camera calibration
  fx=525.;fy=525.;px=320.;py=240.;
}
//deconstructor
AprilTagDetector::~AprilTagDetector(){
  image_sub_.shutdown();
}
void AprilTagDetector::TurtleCallback(const nav_msgs::OdometryConstPtr& Odometry){
  odom.pose=Odometry->pose;
  odom.twist=Odometry->twist;
  odom.header=Odometry->header;
}
void AprilTagDetector::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera){
  fx = camera->K[0];
  fy = camera->K[4];
  px = camera->K[2];
  py = camera->K[5];
}
void AprilTagDetector::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //create a array to store tags for publishing
  slam_apriltag_isam_single::PoseStampedArray tag_pose_array;
  //copy the odom information
  tag_pose_array.odom = odom;
  //convert the image to BGR8
  cv_bridge::CvImagePtr cv_ptr;
  try
  { cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  // create a grayscale image
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  // detect tag from grayscale image
  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);
  //assign the header
  tag_pose_array.header = cv_ptr->header;
  //loop to add each tage to tag_pose_array
  for (int i = 0; i < detections.size(); i++) {
    detections[i].draw(cv_ptr->image);//draw tags on the received image
    //getRelativeTransform(size of the tag in meter, fx,fy,px,py)
    Eigen::Matrix4d transform = detections[i].getRelativeTransform(0.161925, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0,0,3,3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);
    geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0,3);
    tag_pose.pose.position.y = transform(1,3);
    tag_pose.pose.position.z = transform(2,3);
    tag_pose.pose.orientation.x = rot_quaternion.x();
    tag_pose.pose.orientation.y = rot_quaternion.y();
    tag_pose.pose.orientation.z = rot_quaternion.z();
    tag_pose.pose.orientation.w = rot_quaternion.w();
    tag_pose.header = cv_ptr->header;
    //store the tag ID in frame_id
    ostringstream convert;
    convert << detections[i].id;
    tag_pose.header.frame_id = convert.str();
    //pose_pub_1.publish(tag_pose);
    tag_pose_array.poses.push_back(tag_pose);
  }
  //uncomment following 2 lines to show the tag detections video without open rviz
  //cv::imshow("show", cv_bridge::toCvShare(msg, "mono8")->image);
  //cv::imshow("view", cv_ptr->image);//view the video in the open cv, this line could be hidded
  pose_pub_.publish(tag_pose_array);//publish the pose array
  image_pub_.publish(cv_ptr->toImageMsg()); // publish the processed image
}
