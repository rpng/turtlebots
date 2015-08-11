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
   iSAMprocessor(ros::NodeHandle& nh);//constructer
   ~iSAMprocessor();
   void Processing();//function for isam Processing;
   void Finish(string name);//function to output the map to a text file
 private:
   // subscribe the information pose_publisher published
   void AprilCallback(const slam_apriltag_isam_single::PoseStampedArrayConstPtr& PoseArray);
   void InitSlam();//initialize slam
   void InitMarker();//intialize marker
   void AddNode(double distance,double angle);//add nodes to slam
   void AddPoint();//add point to slam
   void PublishMarker();//publish the map of markers of the trajectory and tags
   int FindID(string ID);//find point of the tagID correspond to
   bool copyEnable;
   ros::Subscriber april_sub;

   // publish the path after isam process
   ros::Publisher marker_pub;
   //markers for rviz
   visualization_msgs::Marker tags, line_strip;
   Slam slam;
   vector<Pose2d_Node*> pose_nodes;//turtlebot's pose
   vector<Point2d_Node*> point_nodes;//AprilTags' point
   // a array to store IDs of AprilTags
   vector<string> ID_array;
   // record for received PoseArray and Odometry
   slam_apriltag_isam_single::PoseStampedArray new_posearray;
   //information extract from odom
   tf::Vector3 old_position;
   tf::Vector3 new_position;
   tf::Quaternion new_orientation;
   tf::Quaternion old_orientation;
   //noise for slam
   Noise noise3;
   Noise noise2;

};
