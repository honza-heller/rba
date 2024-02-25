/*
 * pose_nonpar.h
 *
 *  Created on: Aug 1, 2012
 *      Author: hellej1
 */

#ifndef RPOSE_NONPAR_H_
#define RPOSE_NONPAR_H_

#include "rpose.h"
#include "etransform.h"

class NonParametricRobotPose : public RobotPose {

public:
  NonParametricRobotPose() {
    type = NONPARAMETRIC;
  };

 virtual ~NonParametricRobotPose() {};

 virtual void loadPoseData(ifstream &iFile) {
   pose.loadRt(iFile);
   pose.getAngleAxisTranslation(aat);
   pose.getInverseAngleAxisTranslation(iaat);
 }

 virtual void updatePoseData(void) {
   // nothing to do here, NonParametricRobotPose has static data
 }

};


#endif /* RPOSE_NONPAR_H_ */

