/*
 * tdetector_chboard.h
 *
 *  Created on: Jul 27, 2014
 *      Author: jheller
 */

#ifndef TDETECTOR_CHBOARD_H_
#define TDETECTOR_CHBOARD_H_


#include "tdetector_opencv.h"

class ChessboardTargetDetector : public OpenCVTargetDetector {
public:

  ChessboardTargetDetector() {
    target_type = CHESSBOARD;
  }

  virtual bool detect(const char * const target_path) {
    return detectOpenCVTarget(target_path);
  }

};


#endif /* TDETECTOR_CHBOARD_H_ */
