/*
 * tdetector_acircles.h
 *
 *  Created on: Jul 27, 2014
 *      Author: jheller
 */

#ifndef TDETECTOR_ACIRCLES_H_
#define TDETECTOR_ACIRCLES_H_


#include "tdetector_opencv.h"

class AsymCirclesTargetDetector : public OpenCVTargetDetector {
public:

  AsymCirclesTargetDetector() {
    target_type = ASYMMETRIC_CIRCLES_GRID;
  }

  virtual bool detect(const char * const target_path) {
    return detectOpenCVTarget(target_path);
  }

};


#endif /* TDETECTOR_ACIRCLES_H_ */
