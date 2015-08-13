Guide to use slam_apriltag_isam_single

1. install all dependencies at the terminal:
sudo apt-get install cmake libsuitesparse-dev libeigen3-dev libsdl1.2-dev doxygen graphviz subversion libopencv-dev libv4l-dev

2. copy slam_apriltag_isam_single to your catkin workspace.

3. use catkin_make

4.1 run the rosbag without using turtlebot
or
4.2 run the rosrun turtlebot_bringup minimal.launch and rosrun turtlebot_bringup 3dsensor.launch on the turtlebot machine

5. run the launch file or run the node one by one on the connected machine or turtlebot:
roslaunch slam_apriltag_isam_single slam.launch


Note: 
1.when running pose_publisher not on the turtlebot, please add _image_transport:=compressed at the end of rosrun command for faster image transfer speed.
2. the maximum processing speed for the pose_publisher is about 7 fps on 3.5GHz cpu. It will only running on a single core. The processing speed is corresponds to cpu speed. On a 1.7Ghz cpu the processing speed is about 4 fps. 



