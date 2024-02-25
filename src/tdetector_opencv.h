/*
 * tdetector_opencv.h
 *
 *  Created on: Jul 27, 2014
 *      Author: jheller
 */

#ifndef TDETECTOR_OPENCV_H_
#define TDETECTOR_OPENCV_H_

#include "tdetector.h"

#ifndef RBA_NO_OPENCV
  #include <opencv2/opencv.hpp>
#endif

class OpenCVTargetDetector : public TargetDetector {
protected:
#ifndef RBA_NO_OPENCV
  vector<cv::Point2f> ipoints;
#endif 
  bool target_found;

public:

  virtual ~OpenCVTargetDetector() {}

  bool detectOpenCVTarget(const char  * const);
  virtual int identify(void);
  virtual void getDetections(list<Vector3f> &, list<Vector2f> &, list<unsigned long long int> &);
};


#endif /* TDETECTOR_OPENCV_H_ */
