// file: distance_reporter_node.cpp
//
// date:   10/30/21
// author: Cornelius Leary
//

#include "waypoint_detector/distance_reporter_node.h"

//-----
DistanceReporterNode::DistanceReporterNode(ros::NodeHandle* node_handle)
  : mNodeHandle(*node_handle),
    mCurrentWaypointImageData(),
    mImageTransport(*node_handle),
    mWaypointImageDataSubscriber(nullptr),
    mCurrentDepthCameraDepthImage(),
    mDepthCameraDepthImageSubscriber(nullptr),
    mCurrentWaypointDistanceReport(),
    mWaypointDistanceReportPublisher()
{
   // set up subscriber object to read the current waypoint detector image data messages.
   mWaypointImageDataSubscriber =
         new message_filters::Subscriber<waypoint_detector_msgs::WaypointDetector>(
               mNodeHandle,
               "/waypoint_detector/image_data",
               1);

   // set up subscriber object to read the current depth camera image messages.
   mDepthCameraDepthImageSubscriber =
         new message_filters::Subscriber<sensor_msgs::Image>(mNodeHandle,
                                                             "/realsense/depth/image_rect_raw",
                                                             1);

   // set up time-synchronizing object for multiple, incoming topic subscribers.
   mTimeSync = new message_filters::Synchronizer<approx_time_sync_policy>(
         approx_time_sync_policy(10),
         *mWaypointImageDataSubscriber,
         *mDepthCameraDepthImageSubscriber);

   // register the synchronized callback.
   mTimeSync->registerCallback(
         boost::bind(&DistanceReporterNode::onSynchronizedCallback, this, _1, _2));

   // set up publisher object to publish the current waypoint distance report messages.
   mWaypointDistanceReportPublisher =
         mNodeHandle.advertise<waypoint_detector_msgs::WaypointDistanceReport>(
               "/waypoint_detector/distance_reports",
               1);
}

//-----
DistanceReporterNode::~DistanceReporterNode()
{
   if (mWaypointImageDataSubscriber != nullptr)
   {
      delete mWaypointImageDataSubscriber;
      mWaypointImageDataSubscriber = nullptr;
   }

   if (mDepthCameraDepthImageSubscriber != nullptr)
   {
      delete mDepthCameraDepthImageSubscriber;
      mDepthCameraDepthImageSubscriber = nullptr;
   }

   if (mTimeSync != nullptr)
   {
      delete mTimeSync;
      mTimeSync = nullptr;
   }
}

//-----
void DistanceReporterNode::onSynchronizedCallback(
      const waypoint_detector_msgs::WaypointDetectorConstPtr& detected_waypoint_msg,
      const sensor_msgs::ImageConstPtr&                       image_msg)
{
   // store current waypoints detected message.
   mCurrentWaypointImageData = *detected_waypoint_msg;

   // store current depth camera depth image message.
   mCurrentDepthCameraDepthImage = *image_msg;

   // reset current distance report.
   mCurrentWaypointDistanceReport = waypoint_detector_msgs::WaypointDistanceReport();

   mCurrentWaypointDistanceReport.header.frame_id = mCurrentWaypointImageData.header.frame_id;
   mCurrentWaypointDistanceReport.header.seq      = mCurrentWaypointImageData.header.seq;
   mCurrentWaypointDistanceReport.header.stamp    = ros::Time::now();

   cv_bridge::CvImagePtr cv_ptr;

   try
   {
      // attempt to create a copy of the ROS image data, ensuring that the image is
      // encoded in Grayscale.
      cv_ptr = cv_bridge::toCvCopy(image_msg);
      //      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }

   cv::Mat depth_image = cv_ptr->image;

   if (depth_image.empty())
   {
      // publish the empty distance report message.
      mWaypointDistanceReportPublisher.publish(mCurrentWaypointDistanceReport);
      return;
   }

   // process current set of detected waypoints.
   for (int index = 0; index < mCurrentWaypointImageData.num_waypoints_detected; index++)
   {
      // read (x,y) position of current detected waypoint in color image.
      int x_waypoint_position = mCurrentWaypointImageData.x_coords[index];
      int y_waypoint_position = mCurrentWaypointImageData.y_coords[index];

      // read the depth value stored at the (x,y) position within the depth image.
      uint16_t depth = depth_image.at<uint16_t>(x_waypoint_position, y_waypoint_position);

      // increment number of waypoints reported.
      mCurrentWaypointDistanceReport.num_waypoints_detected++;

      geometry_msgs::PoseStamped current_waypoint_xy_depth;
      current_waypoint_xy_depth.header.frame_id = mCurrentWaypointImageData.header.frame_id;
      current_waypoint_xy_depth.header.seq      = mCurrentWaypointImageData.header.seq;
      current_waypoint_xy_depth.header.stamp    = ros::Time::now();

      // store the pixel location and the corresponding depth value.
      current_waypoint_xy_depth.pose.position.x = x_waypoint_position;
      current_waypoint_xy_depth.pose.position.y = y_waypoint_position;
      current_waypoint_xy_depth.pose.position.z = depth;

      mCurrentWaypointDistanceReport.poses.push_back(current_waypoint_xy_depth);

      //      uint32_t row_length = mCurrentDepthCameraDepthImage.step;

      //      // read current image step size.
      //      int num_full_rows = y_waypoint_position / row_length;
      //      int remainder     = y_waypoint_position % row_length;

      //      mCurrentDepthCameraDepthImage.data[num_full_rows][remainder];
   }

   // publish the set of distance reports for the detected waypoints.
   mWaypointDistanceReportPublisher.publish(mCurrentWaypointDistanceReport);
}
