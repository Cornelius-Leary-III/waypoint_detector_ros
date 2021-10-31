// file: distance_reporter_node_main.cpp
//
// date:   10/30/21
// author: Cornelius Leary
//

#include <ros/ros.h>
#include "waypoint_detector/distance_reporter_node.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "distance_reporter_node");
   ros::NodeHandle node_handle;

   //   TennisBallDetectorNode tennis_ball_detector_node(&node_handle);

   ros::spin();

   return 0;
}
