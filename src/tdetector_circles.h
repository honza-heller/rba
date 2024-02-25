/*
 * tdetector_circles.h
 *
 *  Created on: Jul 27, 2014
 *      Author: jheller
 */

#ifndef TDETECTOR_CIRCLES_H_
#define TDETECTOR_CIRCLES_H_


#include "tdetector_opencv.h"

class CirclesTargetDetector : public OpenCVTargetDetector {
public:

  CirclesTargetDetector() {
    target_type = CIRCLES_GRID;
  }

  virtual bool detect(const char * const target_path) {
    return detectOpenCVTarget(target_path);
  }

};


#endif /* TDETECTOR_CIRCLES_H_ */
