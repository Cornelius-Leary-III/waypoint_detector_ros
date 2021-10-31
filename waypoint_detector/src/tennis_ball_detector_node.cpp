// file: tennis_ball_detector_node.cpp
//
// date:   10/30/21
// author: Cornelius Leary
//
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
//

#include "waypoint_detector/tennis_ball_detector_node.h"

//-----
TennisBallDetectorNode::TennisBallDetectorNode(ros::NodeHandle* node_handle)
  : mNodeHandle(*node_handle),
    mImageTransport(*node_handle),
    mImageSubscriber(),
    mImagePublisher(),
    mDetector(),
    mCurrentWaypointDetectorMsg(),
    mWaypointDetectorPublisher(),
    mCurrentDetectedCircles()
{
   // set up subscriber object to read images from the onboard camera sensor.
   mImageSubscriber = mImageTransport.subscribe("/realsense/color/image_raw",
                                                1,
                                                &TennisBallDetectorNode::imageCallback,
                                                this);

   // set up publisher object to publish converted image types that are OpenCV-friendly.
   mImagePublisher = mImageTransport.advertise("/image_converter/output_video", 1);

   // set up publisher object to publish the current waypoint detector messages.
   mWaypointDetectorPublisher = mNodeHandle.advertise<waypoint_detector_msgs::WaypointDetector>(
         "/waypoint_detector/image_data",
         1);
}

//-----
TennisBallDetectorNode::~TennisBallDetectorNode()
{
}

//-----
void TennisBallDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
   cv_bridge::CvImagePtr cv_ptr;

   try
   {
      // attempt to create a copy of the ROS image data, ensuring that the image is
      // encoded in BGR (which is required for OpenCV color images).
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }

   try
   {
      if (cv_ptr->image.empty())
      {
         ROS_ERROR("tennis ball detector: empty image file");
         return;
      }

      // perform tennis-ball-detection steps.
      mDetector.detectCircles(cv_ptr->image);
   }
   catch (cv::Exception& e)
   {
      ROS_ERROR("tennis ball detector exception: %s", e.what());
      return;
   }

   // get original image, annotated with any detected tennis ball(s).
   cv::Mat annotated_image = mDetector.getAnnotatedImage();

   if (annotated_image.empty())
   {
      ROS_ERROR("tennis ball detector: empty result image");
      return;
   }

   // create the output image object.
   cv_bridge::CvImage result;

   if (cv_ptr != nullptr)
   {
      // set up the header object field within the output image data structure.
      result.header.frame_id = cv_ptr->header.frame_id;
      result.header.seq      = cv_ptr->header.seq;
      result.header.stamp    = ros::Time::now();

      // set up the encoding and image fields within the output image data structure.
      result.encoding = cv_ptr->encoding;
      result.image    = annotated_image;

      // convert the output image data structure to a ROS-friendly image data structure,
      // then publish the ROS-friendly image.
      mImagePublisher.publish(result.toImageMsg());
   }

   mCurrentDetectedCircles = mDetector.getDetectedCircles();

   //   if (mCurrentDetectedCircles.empty())
   //   {
   //      return;
   //   }

   // reset current waypoint message.
   mCurrentWaypointDetectorMsg = waypoint_detector_msgs::WaypointDetector();

   // set up the header object field within the waypoint detector data structure.
   mCurrentWaypointDetectorMsg.header.frame_id = result.header.frame_id;
   mCurrentWaypointDetectorMsg.header.seq      = result.header.seq;
   mCurrentWaypointDetectorMsg.header.stamp    = ros::Time::now();

   size_t num_waypoints                               = mCurrentDetectedCircles.size();
   mCurrentWaypointDetectorMsg.num_waypoints_detected = num_waypoints;

   for (int index = 0; index < num_waypoints; index++)
   {
      // store the current waypoint's location within the image frame.
      cv::Vec3i waypoint = mCurrentDetectedCircles[index];

      // add the current waypoint's location within the image frame to the message.
      mCurrentWaypointDetectorMsg.x_coords.push_back(waypoint[0]);
      mCurrentWaypointDetectorMsg.y_coords.push_back(waypoint[1]);
      mCurrentWaypointDetectorMsg.radii.push_back(waypoint[2]);
   }

   // publish the current waypoint detector msg.
   mWaypointDetectorPublisher.publish(mCurrentWaypointDetectorMsg);
}
