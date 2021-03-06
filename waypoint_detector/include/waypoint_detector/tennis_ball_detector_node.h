// file: tennis_ball_detector_node.h
//
// date:   10/30/21
// author: Cornelius Leary
//
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
//

#ifndef TENNIS_BALL_DETECTOR_NODE_H
#define TENNIS_BALL_DETECTOR_NODE_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "waypoint_detector/tennis_ball_detector.h"
#include "waypoint_detector_msgs/WaypointDetector.h"
#include "waypoint_detector_msgs/WaypointDistanceReport.h"

#include <vector>

class TennisBallDetectorNode
{
public:
   TennisBallDetectorNode(ros::NodeHandle* node_handle);
   ~TennisBallDetectorNode();

   void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

private:
   ros::NodeHandle mNodeHandle;

   image_transport::ImageTransport mImageTransport;
   image_transport::Subscriber     mDepthCameraColorImageSubscriber;
   image_transport::Publisher      mImagePublisher;

   TennisBallDetector mDetector;

   image_transport::Subscriber mDepthCameraDepthImageSubscriber;
   sensor_msgs::Image          mCurrentDepthCameraDepthImage;

   waypoint_detector_msgs::WaypointDetector mCurrentWaypointDetectorMsg;
   ros::Publisher                           mWaypointDetectorPublisher;
   std::vector<cv::Vec3f>                   mCurrentDetectedCircles;

   waypoint_detector_msgs::WaypointDistanceReport mCurrentWaypointDistanceReport;
   ros::Publisher                                 mWaypointDistanceReportPublisher;
};

#endif // TENNIS_BALL_DETECTOR_NODE_H
