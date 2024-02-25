/*
 * robot.h
 *
 *  Created on: Mar 13, 2014
 *      Author: hellej1
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "definitions.h"

#include "rdesc.h"
#include "rdesc_mdh.h"
#include "rpose.h"
#include "scene.h"

#include <ceres/ceres.h>

class Robot {
  RbaOptions          *opts;

  int                 no_poses;
  vector<RobotPose*>  poses;
  RobotDescription    *rdesc;
  const Scene         *scene;

  void loadRobotPoses(void);
  void loadRobotDescription(void);
  void loadRobotMask(void);
  void saveRobotResidualError(const string &, const Track::ResidualError) const;

  void clearPoses(void) {
    for (int i = 0; i < (int) poses.size(); i++)
      delete poses[i];
    poses.clear();
  }

public:
  Robot() {
    opts = NULL;
    rdesc = NULL;
    no_poses = 0;
    scene = NULL;
  }

  Robot(RbaOptions * const _opts) {
    Robot();
    opts = _opts;
  }

  void setOptions(RbaOptions * const _opts) {
    opts = _opts;
  }

  ~Robot() {
    if (rdesc != NULL)
      delete rdesc;

    clearPoses();
  }

  int getNumPoses(void) const {
    return no_poses;
  }

  inline const vector<RobotPose*>& getPoses(void) const {
    return poses;
  }

  inline void setScene(const Scene * const _scene) {
    scene = _scene;

    vector<Camera*> const & cameras = scene->getCameras();

    if (no_poses != (int) cameras.size())
      LOG_FATAL << "Cannot assign cameras to robot poses. The number of robot poses (" << no_poses
      << ") does not equal the number of cameras (" << cameras.size() << ")";

    for (int i = 0; i < no_poses; i++)
      poses[i]->setCamera(cameras[i]);
  }

  inline int getParameters(double * &params, vector<int> &rpmask) const
  {
    if (rdesc == NULL)
      {
        params = NULL;
        rpmask.clear();
        return 0;
      }
    else
      return rdesc->getParameters(params, rpmask);
  }

  inline void setParameters(const double* const params) const {
    if (rdesc)
      {
        rdesc->setParameters(params);
        for (int i = 0; i < no_poses; i++)
          poses[i]->updatePoseData();
      }
  }

  inline void addLinearPenalties(Problem * const problem, const int no_projections) const {
    if (rdesc)
      rdesc->addLinearPenalties(problem, no_projections);
  }

  void updateResidualErrors(const ETrans & hec, const ETrans & wbc, const double scale) {
    for (int i = 0; i < no_poses; i++)
      poses[i]->updateResidualErrors(hec, wbc, scale);
  }

  void load(void);
  void save(void) const;
};


#endif /* ROBOT_H_ */
