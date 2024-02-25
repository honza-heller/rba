/*
 * pose.h
 *
 *  Created on: Aug 1, 2012
 *      Author: hellej1
 */

#ifndef RPOSE_H_
#define RPOSE_H_

#include "definitions.h"
#include "camera.h"
#include "track.h"

class RobotPose {
public:
  typedef enum {
    NONPARAMETRIC = 1,
    MODIFIED_DENAVITHARTENBERG = 2,
  } RobotPoseType;

protected:
  RobotPoseType type;
  int           index;
  Camera        *camera;
  ETrans        pose;
  double        aat[6];
  double        iaat[6];

public:

  virtual ~RobotPose() {
  }

  virtual void loadPoseData(ifstream&) = 0;
  virtual void updatePoseData(void) = 0;

  RobotPoseType getPoseType(void) const {
    return type;
  }

  void setIndex(const int i) {
    index = i;
  }

  int getIndex(void) const {
    return index;
  }

  void setCamera(Camera * _camera) {
    camera = _camera;
  }

  Camera * getCamera(void) const {
    return camera;
  }

  void updateResidualErrors(const ETrans & hec, const ETrans & wbc, const double scale)
  {
    if (camera)
      {
        ETrans w2c(hec.getMatrix() * getInverseMatrix() * wbc.getMatrix() * Vector4f(scale, scale, scale, 1).asDiagonal());
        camera->updateResidualErrors(w2c, Track::ROBOT_IMAGE_SPACE_RESIDUAL, Camera::IMAGE_SPACE);
        camera->updateResidualErrors(w2c, Track::ROBOT_OBJECT_SPACE_RESIDUAL, Camera::OBJECT_SPACE);
      }
  }

  Matrix4f getMatrix(void) const {
    return pose.getMatrix();
  }

  Matrix4f getInverseMatrix(void) const {
    return pose.getInverseMatrix();
  }

  template<typename T> void getInverseAngleAxisTranslation(T * const _iaat) const {
    for (int i = 0; i < 6; i++)
      _iaat[i] = T(iaat[i]);
  }

  template<typename T> void getAngleAxisTranslation(T * const _aat) const {
    for (int i = 0; i < 6; i++)
      _aat[i] = T(aat[i]);
  }

  bool isZero(void) const {
    return pose.isZero();
  }
};


#endif /* RPOSE_H_ */
