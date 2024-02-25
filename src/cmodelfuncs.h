/*
 * cmodelfuncs.h
 *
 *  Created on: Mar 24, 2016
 *      Author: jheller
 */

#ifndef CMODELFUNCS_H_
#define CMODELFUNCS_H_

/*
#include "definitions.h"
#include "rpose_mdh.h"
#include "camera_opencv.h"

#include <ceres/ceres.h>
#include <ceres/dynamic_autodiff_cost_function.h>

using namespace ceres;
*/

class CameraModels {
public:

  typedef enum {
    OPENCV_RATIONAL = 0,
    CUBIC_RATIONAL = 1,
    FISHEYE_D3P = 2,
    FISHEYE_P8P = 3,
    FISHEYE_E7P = 4
  } CameraModelType;

  static bool isFishEye(const CameraModelType cmodel_type) {
    return ((cmodel_type == FISHEYE_D3P) || (cmodel_type == FISHEYE_P8P) || (cmodel_type == FISHEYE_E7P));
  }

  static bool isFishEye(const int cmodel_type) {
    return isFishEye(CameraModelType(cmodel_type));
  }

};


#endif /* CMODELFUNCS_H_ */
