/*
 * tdetector_opencv.cpp
 *
 *  Created on: Jul 27, 2014
 *      Author: jheller
 */

#include "tdetector_opencv.h"

#ifndef RBA_NO_OPENCV
#include <opencv2/opencv.hpp>

bool OpenCVTargetDetector::detectOpenCVTarget(const char * const target_path)
{
  ipoints.clear();

  cv::Mat img = cv::imread(target_path);

  if (img.empty())
    LOG_FATAL << "Cannot load image file: " << target_path;


  if (target_type == CHESSBOARD)
    target_found = cv::findChessboardCorners(img, cv::Size(width, height), ipoints,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
  else if (target_type == CIRCLES_GRID)
    target_found = cv::findCirclesGrid(img, cv::Size(width, height), ipoints);
  else if (target_type == ASYMMETRIC_CIRCLES_GRID)
    target_found = cv::findCirclesGrid(img, cv::Size(width, height), ipoints, cv::CALIB_CB_ASYMMETRIC_GRID);

  if (target_found)
    {
      // improve the found corners' coordinate accuracy for chessboard
      if( target_type == CHESSBOARD)
        {
          cv::Mat viewGray;
          cv::cvtColor(img, viewGray, cv::COLOR_BGR2GRAY);
          cv::cornerSubPix(viewGray, ipoints, cv::Size(11,11), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::Type::EPS + cv::TermCriteria::Type::MAX_ITER, 30, 0.1));
        }
    }
  else
    return false;

  return true;
}

void OpenCVTargetDetector::getDetections(list<Vector3f> & points, list<Vector2f> & detections, list<unsigned long long int> &indices)
{
  points.clear();
  detections.clear();
  indices.clear();

  if (!target_found)
    return;

  int ipt = 0;
  for(unsigned int i = 0; i < height; ++i)
    {
      for(unsigned int j = 0; j < width; ++j)
        {
          Vector3f pt = Vector3f(0.0, 0.0, 0.0);
          Vector2f dt = Vector2f(ipoints[ipt].x, ipoints[ipt].y);

          if ((target_type == CHESSBOARD) || target_type == (CIRCLES_GRID))
            pt = Vector3f(j * xstride, i * ystride, 0);
          else if (target_type == ASYMMETRIC_CIRCLES_GRID)
            pt = Vector3f((2*j + i % 2)* xstride, i * ystride, 0);

          points.push_back(pt);
          detections.push_back(dt);
          indices.push_back(pointIndex(i,j));

          ipt++;
        }
    }
}

#else

bool OpenCVTargetDetector::detectOpenCVTarget(const char * const target_path)
{
        LOG_FATAL << "RBA compiled without OpenCV support. Recompile with OpenCV support to be able to detect OpenCV style targets";
        return false;
}

void OpenCVTargetDetector::getDetections(list<Vector3f> & points, list<Vector2f> & detections, list<unsigned long long int> &indices)
{
        LOG_FATAL << "RBA compiled without OpenCV support. Recompile with OpenCV support to be able to detect OpenCV style targets";
}

#endif

int OpenCVTargetDetector::identify(void)
{
  // Nothing to do here, the target is identified in the detection step.
  return (target_found) ? height * width : 0;
}

