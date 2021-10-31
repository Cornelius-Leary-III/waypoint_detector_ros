// file: tennis_ball_detector.cpp
//
// date:   10/30/21
// author: Cornelius Leary
//
// https://docs.opencv.org/4.5.2/d4/d70/tutorial_hough_circle.html
//

#include "waypoint_detector/tennis_ball_detector.h"

#include <iostream>

const int gCircleIndexX      = 0;
const int gCircleIndexY      = 1;
const int gCircleIndexRadius = 2;

const double gDefaultInvAccumulatorDpParam = 1;
const double gDefaultHoughMethodParam1     = 100;
const double gDefaultHoughMethodParam2     = 20;
const int    gDefaultCircleRadiusMin       = 0;
const int    gDefaultCircleRadiusMax       = 1000;
const int    gDefaultCircleCenterPointSize = 1;
const int    gDefaultCircleLineThickness   = 3;

const int gDefaultMinDistDenominator = 1;
const int gDefaultElementSize        = 3;

const int gDefaultHueLow  = 34;
const int gDefaultHueHigh = 53;

const int gDefaultSaturationLow  = 68;
const int gDefaultSaturationHigh = 255;

const int gDefaultValueLow  = 0;
const int gDefaultValueHigh = 255;

static const cv::Scalar gColorBGRCenterPoint(0, 100, 100);
static const cv::Scalar gColorBGRCircleOutline(255, 0, 255);

const int gWaitForKeyPressDuration = 0;

// static const std::string gWindowNameDetected("detected circles");
// static const std::string gWindowNameThresh("thresholded image");

bool TennisBallDetector::s_needToProcessImageParamChanged = true;

//-----
void TennisBallDetector::onTrackbar(int, void*)
{
   TennisBallDetector::s_needToProcessImageParamChanged = true;
}

//-----
TennisBallDetector::TennisBallDetector()
  : mImageFrameOriginal(),
    mImageFrameHSVConverted(),
    mImageFrameHSVThresholded(),
    mDetectedCirclesCurrent(),
    mIsCurrentImageValid(false),
    mParamInvAccumulator(gDefaultInvAccumulatorDpParam),
    mParamHoughMethod1(gDefaultHoughMethodParam1 * 100),
    mParamHoughMethod2(gDefaultHoughMethodParam2 * 100),
    mParamCircleRadiusMin(gDefaultCircleRadiusMin),
    mParamCircleRadiusMax(gDefaultCircleRadiusMax),
    mParamHueLow(gDefaultHueLow),
    mParamHueHigh(gDefaultHueHigh),
    mParamSaturationLow(gDefaultSaturationLow),
    mParamSaturationHigh(gDefaultSaturationHigh),
    mParamValueLow(gDefaultValueLow),
    mParamValueHigh(gDefaultValueHigh),
    mParamMinDistDenominator(gDefaultMinDistDenominator),
    mParamElementRectSize(gDefaultElementSize)
{
   //   // Create a window
   //   cv::namedWindow(gWindowNameDetected, 1);
   //   cv::namedWindow(gWindowNameThresh, 1);

   //   cv::createTrackbar("Hough param 1 (scaled up by 100)",
   //                      gWindowNameDetected,
   //                      &mParamHoughMethod1,
   //                      100000,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("Hough param 2 (scaled up by 100)",
   //                      gWindowNameDetected,
   //                      &mParamHoughMethod2,
   //                      10000,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("Hough circle min radius",
   //                      gWindowNameDetected,
   //                      &mParamCircleRadiusMin,
   //                      1000,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("Hough circle max radius",
   //                      gWindowNameDetected,
   //                      &mParamCircleRadiusMax,
   //                      1000,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("Hough circle min dist denom.",
   //                      gWindowNameDetected,
   //                      &mParamMinDistDenominator,
   //                      32,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("Blur/Dilate/Erode rect. element size",
   //                      gWindowNameDetected,
   //                      &mParamElementRectSize,
   //                      10,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("HSV - Hue Low",
   //                      gWindowNameThresh,
   //                      &mParamHueLow,
   //                      255,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("HSV - Hue High",
   //                      gWindowNameThresh,
   //                      &mParamHueHigh,
   //                      255,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("HSV - Saturation Low",
   //                      gWindowNameThresh,
   //                      &mParamSaturationLow,
   //                      255,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("HSV - Saturation High",
   //                      gWindowNameThresh,
   //                      &mParamSaturationHigh,
   //                      255,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("HSV - Value Low",
   //                      gWindowNameThresh,
   //                      &mParamValueLow,
   //                      255,
   //                      &TennisBallDetector::onTrackbar);

   //   cv::createTrackbar("HSV - Value High",
   //                      gWindowNameThresh,
   //                      &mParamValueHigh,
   //                      255,
   //                      &TennisBallDetector::onTrackbar);
}

//-----
TennisBallDetector::~TennisBallDetector()
{
}

//-----
cv::Mat TennisBallDetector::getAnnotatedImage()
{
   return mImageFrameOriginal;
}

//-----
bool TennisBallDetector::haveParamsChanged()
{
   return s_needToProcessImageParamChanged;
}

//-----
void TennisBallDetector::detectCircles(cv::Mat input_image)
{
   s_needToProcessImageParamChanged = false;

   // prepare for circle-detection algorithm.
   processInputImage(input_image);

   if (!mIsCurrentImageValid)
   {
      // throw exception here?
      return;
   }

   mDetectedCirclesCurrent.clear();

   if (mParamMinDistDenominator <= 0 || mParamHoughMethod1 <= 0 || mParamHoughMethod2 <= 0)
   {
      return;
   }

   // perform circle detection on the grayscale image.
   cv::HoughCircles(
         mImageFrameHSVThresholded, // image to process.
         mDetectedCirclesCurrent,   // container to store detected circles.
         cv::HOUGH_GRADIENT,        // gradient used in Hough transform.
         mParamInvAccumulator,      // "inverse ratio of the accumulator resolution to the image
                                    // resolution."
         mImageFrameHSVConverted.rows / mParamMinDistDenominator, // minimum distance between the
                                                                  // centers of the detected circles
         double(mParamHoughMethod1 / 100.0),                      // first method-specific parameter
         double(mParamHoughMethod2 / 100.0), // second method-specific parameter
         mParamCircleRadiusMin,              // minimum circle radius
         mParamCircleRadiusMax); // maximum circle radius (if < 0, uses max image dimension)

   // annotate the original image with the detected circles.
   drawDetectedCircles(mImageFrameOriginal);

   //   // display the annotated original image.
   //   cv::imshow(gWindowNameDetected, mImageFrameOriginal);
   //   cv::imshow(gWindowNameThresh, mImageFrameHSVThresholded);

   // wait for the user to press any key to exit the display GUI.
   //   cv::waitKey(gWaitForKeyPressDuration);
}

//-----
void TennisBallDetector::processInputImage(cv::Mat input_image)
{
   mIsCurrentImageValid = false;

   mImageFrameOriginal = input_image;

   if (mImageFrameOriginal.empty())
   {
      mIsCurrentImageValid = false;
      return;
   }

   // OpenCV represents color image in BGR color format
   // want to convert image to another color format
   cv::cvtColor(mImageFrameOriginal, mImageFrameHSVConverted, cv::COLOR_BGR2HSV_FULL);

   if (mImageFrameHSVConverted.empty())
   {
      mIsCurrentImageValid = false;
      return;
   }

   cv::Scalar lowerBoundHSV(mParamHueLow, mParamSaturationLow, mParamValueLow);
   cv::Scalar higherBoundHSV(mParamHueHigh, mParamSaturationHigh, mParamValueHigh);

   // threshold the image.
   cv::inRange(mImageFrameHSVConverted, lowerBoundHSV, higherBoundHSV, mImageFrameHSVThresholded);

   if (mParamElementRectSize <= 0 || (mParamElementRectSize % 2 == 0))
   {
      mIsCurrentImageValid = false;
      return;
   }

   // blur the image to reduce false-positives.
   cv::GaussianBlur(mImageFrameHSVThresholded,
                    mImageFrameHSVThresholded,
                    cv::Size(mParamElementRectSize, mParamElementRectSize),
                    0);

   // get structuring element.
   cv::Mat element =
         cv::getStructuringElement(cv::MORPH_RECT,
                                   cv::Size(mParamElementRectSize, mParamElementRectSize));

   // dilate the image.
   cv::dilate(mImageFrameHSVThresholded, mImageFrameHSVThresholded, element);

   // erode the image.
   cv::erode(mImageFrameHSVThresholded, mImageFrameHSVThresholded, element);

   mIsCurrentImageValid = true;
}

//-----
void TennisBallDetector::drawDetectedCircles(cv::Mat image)
{
   // annotate the image with the detected circles.
   for (size_t index = 0; index < mDetectedCirclesCurrent.size(); index++)
   {
      cv::Vec3i circle_current = mDetectedCirclesCurrent[index];

      // store circle center point.
      cv::Point center = cv::Point(circle_current[gCircleIndexX], circle_current[gCircleIndexY]);

      // draw circle center point.
      cv::circle(image,
                 center,
                 gDefaultCircleCenterPointSize,
                 gColorBGRCenterPoint,
                 gDefaultCircleLineThickness,
                 cv::LINE_AA);

      // store circle radius.
      int radius = circle_current[gCircleIndexRadius];

      // draw circle outline.
      cv::circle(image,
                 center,
                 radius,
                 gColorBGRCircleOutline,
                 gDefaultCircleLineThickness,
                 cv::LINE_AA);
   }
}
