// file: distance_reporter_node.cpp
//
// date:   10/30/21
// author: Cornelius Leary
//

#ifndef DISTANCE_REPORTER_H
#define DISTANCE_REPORTER_H

#include <ros/ros.h>

#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <waypoint_detector_msgs/WaypointDetector.h>
#include <waypoint_detector_msgs/WaypointDistanceReport.h>

typedef message_filters::sync_policies::ApproximateTime<waypoint_detector_msgs::WaypointDetector,
                                                        sensor_msgs::Image>
      approx_time_sync_policy;

class DistanceReporterNode
{
public:
   DistanceReporterNode(ros::NodeHandle* node_handle);
   ~DistanceReporterNode();

   void onSynchronizedCallback(
         const waypoint_detector_msgs::WaypointDetectorConstPtr& detected_waypoint_msg,
         const sensor_msgs::ImageConstPtr&                       image_msg);

private:
   ros::NodeHandle                 mNodeHandle;
   image_transport::ImageTransport mImageTransport;

   waypoint_detector_msgs::WaypointDetector mCurrentWaypointImageData;
   message_filters::Subscriber<waypoint_detector_msgs::WaypointDetector>*
         mWaypointImageDataSubscriber;

   sensor_msgs::Image                               mCurrentDepthCameraDepthImage;
   message_filters::Subscriber<sensor_msgs::Image>* mDepthCameraDepthImageSubscriber;

   message_filters::Synchronizer<approx_time_sync_policy>* mTimeSync;

   waypoint_detector_msgs::WaypointDistanceReport mCurrentWaypointDistanceReport;
   ros::Publisher                                 mWaypointDistanceReportPublisher;
};

#endif // DISTANCE_REPORTER_H
