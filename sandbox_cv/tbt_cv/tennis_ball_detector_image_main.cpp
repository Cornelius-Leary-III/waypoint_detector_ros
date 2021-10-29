// file: waypoint_detector_image_main.cpp
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

const int gIndexExpectedImageFileName = 1;

const int gAsciiCharQuitLowerQ  = 113;
const int gAsciiCharQuitUpperQ  = 81;
const int gWaitForKeyPressDelay = 0; // 0 --> wait indefinitely until key pressed.

int main(int argc, char** argv)
{
   for (int i = 1; i < argc; i++)
   {
      std::cout << "\n\tcurrent arg #" << i << ": ___" << argv[i] << "___" << std::endl;
   }

   if (argc < 2)
   {
      // not enough input arguments.
      // throw exception?
      return -1;
   }

   cv::String image_file_name(argv[gIndexExpectedImageFileName]);

   if (image_file_name.empty())
   {
      // empty file path provided.
      // throw exception?
      return -2;
   }

   TennisBallDetector waypoint_detector;

   int current_waitkey = 0;

   // loop until user presses quit keys ('q' or 'Q')
   while (current_waitkey != gAsciiCharQuitLowerQ && current_waitkey != gAsciiCharQuitUpperQ)
   {
      // only process the image if one or more of the Hough transform or HSV color space parameters
      // have changed.
      if (waypoint_detector.haveParamsChanged())
      {
         // read the original image.
         cv::Mat original_image = cv::imread(image_file_name, cv::IMREAD_COLOR);

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
