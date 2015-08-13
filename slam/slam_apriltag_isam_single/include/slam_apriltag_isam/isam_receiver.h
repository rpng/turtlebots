#ifndef ISAM_RECEIVER_H_
#define ISAM_RECEIVER_H_

#include <ros/ros.h>
#include <isam/isam.h>
#include <slam_apriltag_isam_single/PoseStampedArray.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
using namespace std;
using namespace isam;
using namespace Eigen;
/* Class iSAMprocessor, subscribe PoseStampedArray from pose_publisher
process the apriltags an odometry information to generate map using iSAM.
And publish the interactive markers for rviz to display
*/
class iSAMprocessor{
  public:
   iSAMprocessor(ros::NodeHandle& nh);
   ~iSAMprocessor();
   void Processing();
   // Function to output the map to a text file
   void Finish(string name);
 private:
   // Subscribe the information pose_publisher published
   void AprilCallback(const slam_apriltag_isam_single::PoseStampedArrayConstPtr& PoseArray);
   void InitSlam();// Initialize slam
   void InitMarker();// Intialize marker
   void AddNode(double distance,double angle);// Add nodes to slam
   void AddPoint();// Add point to slam
   void PublishMarker();// Publish the map of markers of the trajectory and tags
   int FindID(string ID);// Find point of the tagID correspond to
   bool copyEnable;
   ros::Subscriber april_sub;

   // Publish the path after isam process
   ros::Publisher marker_pub;
   // Markers for rviz
   visualization_msgs::Marker tags, line_strip;
   Slam slam;
   vector<Pose2d_Node*> pose_nodes;// Turtlebot's pose
   vector<Point2d_Node*> point_nodes;// AprilTags' point
   // A array to store IDs of AprilTags
   vector<string> ID_array;
   // Record for received PoseArray and Odometry
   slam_apriltag_isam_single::PoseStampedArray new_posearray;
   // Information extract from odom
   tf::Vector3 old_position;
   tf::Vector3 new_position;
   tf::Quaternion new_orientation;
   tf::Quaternion old_orientation;
   // Noise for slam
   Noise noise3;
   Noise noise2;

};
#endif //ISAM_RECEIVER_H_
