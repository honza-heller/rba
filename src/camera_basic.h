/*
 * camera_basic.h
 *
 *  Created on: Sep 4, 2012
 *      Author: jheller
 */

#ifndef CAMERA_BASIC_H_
#define CAMERA_BASIC_H_

#include "camera.h"
#include "etransform.h"

class BasicCamera : public Camera {

public:
  BasicCamera() {
    camera_type = BASIC;
  };

 virtual ~BasicCamera() {};

  virtual Ray getRay(const Track * const point) const {
    Ray ray;
    LOG_FATAL << "BasicCamera does not implement getRay method";
    return ray;
  }

  virtual Vector2f getProjection(const Track * const point, const Track::PointType ptype,
                                 const Track::Basis2D basis, const ETrans * const trans = NULL) const
  {
    Vector2f point2D;
    LOG_FATAL << "BasicCamera does not implement getProjection method";
    return point2D;
  }

  virtual void updateDetections(void)
  {
    LOG_FATAL << "BasicCamera does not implement updateDetections method";
  }

  virtual void loadCameraData(ifstream &iFile) {
    cpose.loadRt(iFile);
  }
};



#endif /* CAMERA_BASIC_H_ */
