// file: waypoint_detector_video_main.cpp
//
// date:   10/29/21
// author: Cornelius Leary
//

#include "tennis_ball_detector.h"

#include <string>
#include <iostream>

#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/video.hpp>

const int gMaxNumArgsAllowedforImageFileName = 2;
const int gIndexExpectedImageFileName        = 1;

const int gMinNumArgsNeededForVideoDeviceID = 3;
const int gIndexExpectedVideoDeviceFlag     = 1;
const int gIndexExpectedVideoDeviceID       = 2;

const int gAsciiCharQuitLowerQ  = 113;
const int gAsciiCharQuitUpperQ  = 81;
const int gWaitForKeyPressDelay = 20; // 0 --> wait indefinitely until key pressed.

int main(int argc, char** argv)
{
   for (int i = 1; i < argc; i++)
   {
      std::cout << "\n\tcurrent arg #" << i << ": ___" << argv[i] << "___" << std::endl;
   }

   cv::String video_file_name;

   cv::VideoCapture video_capture;

   if (argc < gMaxNumArgsAllowedforImageFileName)
   {
      // not enough input arguments.
      // throw exception?
      return -1;
   }
   else if (argc == gMaxNumArgsAllowedforImageFileName)
   {
      video_file_name = argv[gIndexExpectedImageFileName];

      if (video_file_name.empty())
      {
         // empty file path provided.
         // throw exception?
         return -2;
      }

      video_capture.open(video_file_name, cv::CAP_ANY);
   }
   else if (argc >= gMinNumArgsNeededForVideoDeviceID)
   {
      std::string input_arg(argv[gIndexExpectedVideoDeviceFlag]);
      std::string id_option("--id");

      if (input_arg == id_option)
      {
         std::string video_index_text = argv[gIndexExpectedVideoDeviceID];

         int video_index = std::stoi(video_index_text);

         video_capture.open(video_index, cv::CAP_ANY);
      }
   }

   if (!video_capture.isOpened())
   {
      std::cout << "\n\t" << __PRETTY_FUNCTION__ << "\tvideo capture not opened" << std::endl;

      // cannot open the video file.
      // throw exception?
      return -3;
   }

   TennisBallDetector waypoint_detector;

   int current_waitkey = 0;

   // loop until user presses quit keys ('q' or 'Q')
   while (current_waitkey != gAsciiCharQuitLowerQ && current_waitkey != gAsciiCharQuitUpperQ)
   {
      // read the original image.
      cv::Mat original_image;

      // read new frame from the video.
      bool frame_read_successfully = video_capture.read(original_image);

      // only process the image if one or more of the Hough transform or HSV color space parameters
      // have changed.
      if (waypoint_detector.haveParamsChanged() || frame_read_successfully)
      {
         // check for empty image frame.
         if (original_image.empty())
         {
            // empty image frame.

            // throw exception?
            // - should not be providing empty image
            break;
         }

         // detect any circle shapes in the provided image.
         // annotate the provided image with the detected circles.
         waypoint_detector.detectCircles(original_image);
      }

      // wait for the user to press a key.
      current_waitkey = cv::waitKey(gWaitForKeyPressDelay);
   }

   return 0;
}
