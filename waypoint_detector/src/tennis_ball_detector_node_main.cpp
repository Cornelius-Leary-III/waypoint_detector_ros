// file: tennis_ball_detector_node_main.cpp
//
// date:   10/29/21
// author: Cornelius Leary
//

#include <ros/ros.h>
#include "waypoint_detector/tennis_ball_detector_node.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "tennis_ball_detector_node");
   ros::NodeHandle node_handle;

   TennisBallDetectorNode tennis_ball_detector_node(&node_handle);

   ros::spin();

   return 0;
}
