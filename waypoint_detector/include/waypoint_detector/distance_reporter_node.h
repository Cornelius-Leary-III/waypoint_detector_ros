// file: distance_reporter_node.cpp
//
// date:   10/30/21
// author: Cornelius Leary
//

#ifndef DISTANCE_REPORTER_H
#define DISTANCE_REPORTER_H

#include <ros/ros.h>

#include <vector>

class DistanceReporterNode
{
public:
   DistanceReporterNode(ros::NodeHandle* node_handle);
   ~DistanceReporterNode();

   //   void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

private:
   ros::NodeHandle mNodeHandle;

   //   image_transport::ImageTransport mImageTransport;
   //   image_transport::Subscriber     mImageSubscriber;
   //   image_transport::Publisher      mImagePublisher;

   //   TennisBallDetector mDetector;

   //   waypoint_detector_msgs::WaypointDetector mCurrentWaypointDetectorMsg;
   //   ros::Publisher                           mWaypointDetectorPublisher;
   //   std::vector<cv::Vec3f>                   mCurrentDetectedCircles;
};

#endif // DISTANCE_REPORTER_H
