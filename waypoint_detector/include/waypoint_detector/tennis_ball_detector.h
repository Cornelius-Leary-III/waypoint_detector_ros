// file: tennis_ball_detector.h
//
// date:   10/30/21
// author: Cornelius Leary
//
// https://docs.opencv.org/4.5.2/d4/d70/tutorial_hough_circle.html
//

// C++ STL library includes
#include <string>

// OpenCV library includes
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

class TennisBallDetector
{
public:
   TennisBallDetector();
   ~TennisBallDetector();

   void detectCircles(cv::Mat input_image);
   bool haveParamsChanged();

   cv::Mat getAnnotatedImage();

   static bool s_needToProcessImageParamChanged;

private:
   void processInputImage(cv::Mat input_image);
   void drawDetectedCircles(cv::Mat image);

   static void onTrackbar(int, void*);

   std::string mImageFileName;

   cv::Mat mImageFrameOriginal;
   cv::Mat mImageFrameHSVConverted;
   cv::Mat mImageFrameHSVThresholded;

   std::vector<cv::Vec3f> mDetectedCirclesCurrent;

   bool mIsCurrentImageValid;

   double mParamInvAccumulator;
   int    mParamHoughMethod1;
   int    mParamHoughMethod2;
   int    mParamCircleRadiusMin;
   int    mParamCircleRadiusMax;

   int mParamHue;
   int mParamHueLow;
   int mParamHueHigh;

   int mParamSaturation;
   int mParamSaturationLow;
   int mParamSaturationHigh;

   int mParamValue;
   int mParamValueLow;
   int mParamValueHigh;

   int mParamMinDistDenominator;
   int mParamElementRectSize;
};
