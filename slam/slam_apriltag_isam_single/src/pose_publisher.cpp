
#include "slam_apriltag_isam/pose_publisher.h"
#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh): it_(nh){
  AprilTags::TagCodes tag_codes = AprilTags::tagCodes36h11;//load the apriltags tag library
  tag_detector_ = new AprilTags::TagDetector(tag_codes);//add library to the detector
  // Subscribe image from kinect
  // Change here to subscribe different camera
  image_sub_ = it_.subscribe("camera/rgb/image_raw", 1, &AprilTagDetector::ImageCallback, this);
  // Subscribe CameraInfo
  // Change here to subscribe different camera
  camera_info_ = nh.subscribe<sensor_msgs::CameraInfo>("camera/rgb/camera_info", 1, &AprilTagDetector::CameraInfoCallback, this);
  // Advertise processed image
  image_pub_ = it_.advertise("tag_detections_image", 1);
  // Subscribe the odometry, delete it if you don not need odom
  turtle_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &AprilTagDetector::TurtleCallback, this);
  // Advertise tag position, orientation and odometry
  pose_pub_ = nh.advertise<slam_apriltag_isam_single::PoseStampedArray>("tag_detections_pose", 1);
  // Initialize camera calibration
  fx=525.;fy=525.;px=320.;py=240.;
}
// Deconstructor
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
  // Create a array to store tags for publishing
  slam_apriltag_isam_single::PoseStampedArray tag_pose_array;
  // Copy the odom information
  tag_pose_array.odom = odom;
  // Convert the image to BGR8
  cv_bridge::CvImagePtr cv_ptr;
  try
  { cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  // Create a grayscale image
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  // Detect tag from grayscale image
  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);
  // Assign the header
  tag_pose_array.header = cv_ptr->header;
  // Loop to add each tage to tag_pose_array
  for (int i = 0; i < detections.size(); i++) {
    detections[i].draw(cv_ptr->image);//draw tags on the received image
    // getRelativeTransform(size of the tag in meter, fx,fy,px,py)
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
    // Store the tag ID in frame_id
    ostringstream convert;
    convert << detections[i].id;
    tag_pose.header.frame_id = convert.str();
    // pose_pub_1.publish(tag_pose);
    tag_pose_array.poses.push_back(tag_pose);
  }
  // Uncomment following 2 lines to show the tag detections video without open rviz
  //cv::imshow("show", cv_bridge::toCvShare(msg, "mono8")->image);
  //cv::imshow("view", cv_ptr->image);//view the video in the open cv, this line could be hidded
  pose_pub_.publish(tag_pose_array);//publish the pose array
  image_pub_.publish(cv_ptr->toImageMsg()); // publish the processed image
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // Uncomment following 2 line to create a cv window
  //cv::namedWindow("view", CV_WINDOW_AUTOSIZE);
  //cv::startWindowThread();
  AprilTagDetector detector(nh);
  ros::spin();
  // Uncomment following line to close window when exit.
  //cv::destroyWindow("view");
}
