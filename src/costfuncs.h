/*
 * rba_cfuncs.h
 *
 *  Created on: Mar 19, 2014
 *      Author: jheller
 */

#ifndef COSTFUNCS_H_
#define COSTFUNCS_H_

#include "definitions.h"
#include "rpose_mdh.h"
#include "camera_opencv.h"

#include <ceres/ceres.h>
#include <ceres/dynamic_autodiff_cost_function.h>

using namespace ceres;

class RbaCostFunctions {
private:

  inline static void addAbsNonParametricResidual(const bool osflag,
                                 const RobotPose * const rpose,
                                 const Track * const track,
                                 double * const hec,
                                 double * const wbc,
                                 double * const scale,
                                 Problem * const problem)
  {
    CostFunction *cost_function;
    if (osflag)
      cost_function = new AutoDiffCostFunction<absOSpaceNonParametricFunctor, 3, 6, 6, 1>(
                        new absOSpaceNonParametricFunctor(rpose, track));
    else
       cost_function = new AutoDiffCostFunction<absISpaceNonParametricFunctor, 2, 6, 6, 1>(
                         new absISpaceNonParametricFunctor(rpose, track));
    problem->AddResidualBlock(cost_function, NULL, hec, wbc, scale);
  }

  inline static void addRelNonParametricResidual(const bool osflag,
                                                 const RobotPose * const rpose1,
                                                 const RobotPose * const rpose2,
                                                 const Track * const track,
                                                 double * const hec,
                                                 double * const scale,
                                                 Problem * const problem)
  {
    CostFunction *cost_function;
    if (osflag)
      cost_function = new AutoDiffCostFunction<relOSpaceNonParametricFunctor, 3, 6, 1>(
                        new relOSpaceNonParametricFunctor(rpose1, rpose2, track));
    else
     cost_function = new AutoDiffCostFunction<relISpaceNonParametricFunctor, 2, 6, 1>(
                       new relISpaceNonParametricFunctor(rpose1, rpose2, track));
    problem->AddResidualBlock(cost_function, NULL, hec, scale);
  }

  inline static void addAbsParametricResidual(const bool osflag,
                                 const RobotPose * const rpose,
                                 const Track * const track,
                                 double * const hec,
                                 double * const wbc,
                                 double * const scale,
                                 const int no_rparams,
                                 double  * const rparams,
                                 Problem * const problem)
  {
    if (osflag)
      {
        DynamicAutoDiffCostFunction<absOSpaceParametricFunctor, 4> *cost_function =
          new DynamicAutoDiffCostFunction<absOSpaceParametricFunctor, 4>(
           new absOSpaceParametricFunctor(rpose, track));
        cost_function->AddParameterBlock(6);
        cost_function->AddParameterBlock(6);
        cost_function->AddParameterBlock(1);
        cost_function->AddParameterBlock(no_rparams);
        cost_function->SetNumResiduals(3);
        problem->AddResidualBlock(cost_function, NULL, hec, wbc, scale, rparams);
      }
    else
      {
        DynamicAutoDiffCostFunction<absISpaceParametricFunctor, 4> *cost_function =
          new DynamicAutoDiffCostFunction<absISpaceParametricFunctor, 4>(
            new absISpaceParametricFunctor(rpose, track));
        cost_function->AddParameterBlock(6);
        cost_function->AddParameterBlock(6);
        cost_function->AddParameterBlock(1);
        cost_function->AddParameterBlock(no_rparams);
        cost_function->SetNumResiduals(2);
        problem->AddResidualBlock(cost_function, NULL, hec, wbc, scale, rparams);
      }
  }

  inline static void addRelParametricResidual(const bool osflag,
                                              const RobotPose * const rpose1,
                                              const RobotPose * const rpose2,
                                              const Track * const track,
                                              double * const hec,
                                              double * const scale,
                                              const int no_rparams,
                                              double  * const rparams,
                                              Problem * const problem)
  {
    if (osflag)
      {
        DynamicAutoDiffCostFunction<relOSpaceParametricFunctor, 4> *cost_function =
          new DynamicAutoDiffCostFunction<relOSpaceParametricFunctor, 4>(
           new relOSpaceParametricFunctor(rpose1, rpose2, track));
        cost_function->AddParameterBlock(6);
        cost_function->AddParameterBlock(1);
        cost_function->AddParameterBlock(no_rparams);
        cost_function->SetNumResiduals(3);
        problem->AddResidualBlock(cost_function, NULL, hec, scale, rparams);
      }
    else
      {
        DynamicAutoDiffCostFunction<relISpaceParametricFunctor, 4> *cost_function =
          new DynamicAutoDiffCostFunction<relISpaceParametricFunctor, 4>(
            new relISpaceParametricFunctor(rpose1, rpose2, track));
        cost_function->AddParameterBlock(6);
        cost_function->AddParameterBlock(1);
        cost_function->AddParameterBlock(no_rparams);
        cost_function->SetNumResiduals(2);
        problem->AddResidualBlock(cost_function, NULL, hec, scale, rparams);
      }
  }

  template <int no_rparams> inline static void addAbsFixedParametricResidual(const bool osflag,
                                                                           const RobotPose * const rpose,
                                                                           const Track * const track,
                                                                           double * const hec,
                                                                           double * const wbc,
                                                                           double * const scale,
                                                                           double * const rparams,
                                                                           Problem * const problem)
  {
    CostFunction *cost_function;
    if (osflag)
      cost_function = new AutoDiffCostFunction<absOSpaceFixedParametricFunctor, 3, 6, 6, 1, no_rparams>(
                        new absOSpaceFixedParametricFunctor(rpose, track));
    else
       cost_function = new AutoDiffCostFunction<absISpaceFixedParametricFunctor, 2, 6, 6, 1, no_rparams>(
                         new absISpaceFixedParametricFunctor(rpose, track));
    problem->AddResidualBlock(cost_function, NULL, hec, wbc, scale, rparams);
  }

  template <int no_rparams> inline static void addRelFixedParametricResidual(const bool osflag,
                                                                           const RobotPose * const rpose1,
                                                                           const RobotPose * const rpose2,
                                                                           const Track * const track,
                                                                           double * const hec,
                                                                           double * const scale,
                                                                           double * const rparams,
                                                                           Problem * const problem)
  {
    CostFunction *cost_function;
    if (osflag)
      cost_function = new AutoDiffCostFunction<relOSpaceFixedParametricFunctor, 3, 6, 1, no_rparams>(
                        new relOSpaceFixedParametricFunctor(rpose1, rpose2, track));
    else
     cost_function = new AutoDiffCostFunction<relISpaceFixedParametricFunctor, 2, 6, 1, no_rparams>(
                       new relISpaceFixedParametricFunctor(rpose1, rpose2, track));
    problem->AddResidualBlock(cost_function, NULL, hec, scale, rparams);
  }

public:

  inline static void addResidual(const bool osflag,
                                 const RobotPose * const rpose,
                                 const Track * const track,
                                 double * const hec,
                                 double * const wbc,
                                 double * const scale,
                                 const int no_rparams,
                                 double  * const rparams,
                                 Problem * const problem)
  {

    if (no_rparams == 0)
      addAbsNonParametricResidual(osflag, rpose, track, hec, wbc, scale, problem);
    else if (no_rparams == 24)
      addAbsFixedParametricResidual<24>(osflag, rpose, track, hec, wbc, scale, rparams, problem);
    else if (no_rparams == 64)
      addAbsFixedParametricResidual<64>(osflag, rpose, track, hec, wbc, scale, rparams, problem);
    else
      addAbsParametricResidual(osflag, rpose, track, hec, wbc, scale, no_rparams, rparams, problem);
  }

  inline static void addResidual(const bool osflag,
                                 const RobotPose * const rpose1,
                                 const RobotPose * const rpose2,
                                 const Track * const track,
                                 double * const hec,
                                 double * const scale,
                                 const int no_rparams,
                                 double  * const rparams,
                                 Problem * const problem)
  {
    if (no_rparams == 0)
      addRelNonParametricResidual(osflag, rpose1, rpose2, track, hec, scale, problem);
    else if (no_rparams == 24)
      addRelFixedParametricResidual<24>(osflag, rpose1, rpose2, track, hec, scale, rparams, problem);
    else
      addRelParametricResidual(osflag, rpose1, rpose2, track, hec, scale, no_rparams, rparams, problem);
  }

private:

  template<typename T> inline static void transformPoint(const T * const aat, T * const pt) {
    T tpt[3];

    ceres::AngleAxisRotatePoint(aat, pt, tpt);

    pt[0] = tpt[0] + aat[3];
    pt[1] = tpt[1] + aat[4];
    pt[2] = tpt[2] + aat[5];
  }

  template<typename T> inline static void inverseTransformPoint(const T * const aat, T * const pt) {
    T tpt[3];
    T iaat[6];

    // Prepare the inverse transformation

    tpt[0] = -aat[3];
    tpt[1] = -aat[4];
    tpt[2] = -aat[5];

    iaat[0] = -aat[0];
    iaat[1] = -aat[1];
    iaat[2] = -aat[2];

    ceres::AngleAxisRotatePoint(iaat, tpt, iaat + 3);

    // Transform the original point
    transformPoint(iaat, pt);
  }

  template<typename T> inline static void scalePoint(const T * const scale, T * const pt) {
    pt[0] = pt[0] * scale[0];
    pt[1] = pt[1] * scale[0];
    pt[2] = pt[2] * scale[0];
  }

  // Fixed-parametric functors ------------------------------------------------

  class relISpaceFixedParametricFunctor {
    const RobotPose *rpose1;
    const RobotPose *rpose2;
    const Track *track;

    public:
    relISpaceFixedParametricFunctor(const RobotPose * const _rpose1, const RobotPose * const _rpose2, const Track * const _track)
      : rpose1(_rpose1), rpose2(_rpose2), track(_track) {}

    template <typename T>
    bool operator() (const T * const hec, const T * const scale, const T * rparams, T * const res) const {
      T pt[3], c1[6];

      track->getPoint(pt);
      rpose1->getCamera()->getAngleAxisTranslation(c1);

      transformPoint(c1, pt);
      scalePoint(scale, pt);
      inverseTransformPoint(hec, pt);

      if ((rpose1->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG) &&
          (rpose2->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG))
        {
            ((ModifiedDHRobotPose *)rpose1)->transformPoint(rparams, pt);
            ((ModifiedDHRobotPose *)rpose2)->inverseTransformPoint(rparams, pt);
        }
      else
        LOG_FATAL << "Unsupported robot pose type in absISpaceParametricFunctor";

      transformPoint(hec, pt);

      Camera *camera = rpose2->getCamera();
      if (camera->getCameraType() == Camera::OPENCV)
        ((OpenCVCamera *)camera)->getImageSpaceResiduals(track, pt, res);
      else
          LOG_FATAL << "Unsupported camera type in absISpaceNonParametricFunctor";

      return true;
    }
  };

  class relOSpaceFixedParametricFunctor {
    const RobotPose *rpose1;
    const RobotPose *rpose2;
    const Track *track;

    public:
    relOSpaceFixedParametricFunctor(const RobotPose * const _rpose1, const RobotPose * const _rpose2, const Track * const _track)
      : rpose1(_rpose1), rpose2(_rpose2), track(_track) {}

    template <typename T>
    bool operator() (const T * const hec, const T * const scale, const T * rparams, T * const res) const {
      T pt[3], c1[6];

      track->getPoint(pt);
      rpose1->getCamera()->getAngleAxisTranslation(c1);

      transformPoint(c1, pt);
      scalePoint(scale, pt);
      inverseTransformPoint(hec, pt);

      if ((rpose1->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG) &&
          (rpose2->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG))
        {
            ((ModifiedDHRobotPose *)rpose1)->transformPoint(rparams, pt);
            ((ModifiedDHRobotPose *)rpose2)->inverseTransformPoint(rparams, pt);
        }
      else
        LOG_FATAL << "Unsupported robot pose type in absISpaceParametricFunctor";

      transformPoint(hec, pt);

      rpose2->getCamera()->getObjectSpaceResiduals(track, pt, res);

      return true;
    }
  };

  class absISpaceFixedParametricFunctor {
    const RobotPose *rpose;
    const Track *track;

    public:
    absISpaceFixedParametricFunctor(const RobotPose * const _rpose, const Track * const _track)
    : rpose(_rpose), track(_track) {}

    template <typename T>
    bool operator() (const T * const hec, const T * const wbc, const T * const scale, const T * rparams, T * const res) const {
      T pt[3];

      track->getPoint(pt);
      scalePoint(scale, pt);
      transformPoint(wbc, pt);

      if (rpose->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG)
        ((ModifiedDHRobotPose *)rpose)->inverseTransformPoint(rparams, pt);
      else
        LOG_FATAL << "Unsupported robot pose type in absISpaceParametricFunctor";

      transformPoint(hec, pt);

      Camera *camera = rpose->getCamera();
      if (camera->getCameraType() == Camera::OPENCV)
        ((OpenCVCamera *)camera)->getImageSpaceResiduals(track, pt, res);
      else
          LOG_FATAL << "Unsupported camera type in absISpaceNonParametricFunctor";

      return true;
    }
  };

  class absOSpaceFixedParametricFunctor {
    const RobotPose *rpose;
    const Track *track;

    public:
    absOSpaceFixedParametricFunctor(const RobotPose * const _rpose, const Track * const _track)
    : rpose(_rpose), track(_track) {}

    template <typename T>
    bool operator() (const T * const hec, const T * const wbc, const T * const scale, const T * rparams, T * const res) const {
      T pt[3];

      track->getPoint(pt);

      scalePoint(scale, pt);
      transformPoint(wbc, pt);

      if (rpose->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG)
        ((ModifiedDHRobotPose *)rpose)->inverseTransformPoint(rparams, pt);
      else
        LOG_FATAL << "Unsupported robot pose type in absISpaceParametricFunctor";

      transformPoint(hec, pt);

      rpose->getCamera()->getObjectSpaceResiduals(track, pt, res);

      return true;
    }
  };

  // Parametric functors ------------------------------------------------------

  class relISpaceParametricFunctor {
    const RobotPose *rpose1;
    const RobotPose *rpose2;
    const Track *track;

    public:
    relISpaceParametricFunctor(const RobotPose * const _rpose1, const RobotPose * const _rpose2, const Track * const _track)
    : rpose1(_rpose1), rpose2(_rpose2), track(_track) {}

    template <typename T>  bool operator() (T const* const* params, T * const res) const {
      T pt[3], c1[6];

      const T *hec = params[0];
      const T *scale = params[1];
      const T *rparams = params[2];

      track->getPoint(pt);
      rpose1->getCamera()->getAngleAxisTranslation(c1);

      transformPoint(c1, pt);
      scalePoint(scale, pt);
      inverseTransformPoint(hec, pt);

      if ((rpose1->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG) &&
          (rpose2->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG))
        {
            ((ModifiedDHRobotPose *)rpose1)->transformPoint(rparams, pt);
            ((ModifiedDHRobotPose *)rpose2)->inverseTransformPoint(rparams, pt);
        }
      else
        LOG_FATAL << "Unsupported robot pose type in absISpaceParametricFunctor";

      transformPoint(hec, pt);

      Camera *camera = rpose2->getCamera();
      if (camera->getCameraType() == Camera::OPENCV)
        ((OpenCVCamera *)camera)->getImageSpaceResiduals(track, pt, res);
      else
          LOG_FATAL << "Unsupported camera type in absISpaceNonParametricFunctor";

      return true;
    }
  };

  class relOSpaceParametricFunctor {
    const RobotPose *rpose1;
    const RobotPose *rpose2;
    const Track *track;

    public:
    relOSpaceParametricFunctor(const RobotPose * const _rpose1, const RobotPose * const _rpose2, const Track * const _track)
    : rpose1(_rpose1), rpose2(_rpose2), track(_track) {}

    template <typename T> bool operator() (T const* const* params, T * const res) const {
      T pt[3], c1[6];

      const T *hec = params[0];
      const T *scale = params[1];
      const T *rparams = params[2];

      track->getPoint(pt);
      rpose1->getCamera()->getAngleAxisTranslation(c1);

      transformPoint(c1, pt);
      scalePoint(scale, pt);
      inverseTransformPoint(hec, pt);

      if ((rpose1->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG) &&
          (rpose2->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG))
        {
            ((ModifiedDHRobotPose *)rpose1)->transformPoint(rparams, pt);
            ((ModifiedDHRobotPose *)rpose2)->inverseTransformPoint(rparams, pt);
        }
      else
        LOG_FATAL << "Unsupported robot pose type in absISpaceParametricFunctor";

      transformPoint(hec, pt);

      rpose2->getCamera()->getObjectSpaceResiduals(track, pt, res);

      return true;
    }
  };

  class absISpaceParametricFunctor {
    const RobotPose *rpose;
    const Track *track;

    public:
    absISpaceParametricFunctor(const RobotPose * const _rpose, const Track * const _track)
    : rpose(_rpose), track(_track) {}

    template <typename T>  bool operator() (T const* const* params, T * const res) const {
      T pt[3];

      const T *hec = params[0];
      const T *wbc = params[1];
      const T *scale = params[2];
      const T *rparams = params[3];

      track->getPoint(pt);
      scalePoint(scale, pt);
      transformPoint(wbc, pt);

      if (rpose->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG)
        ((ModifiedDHRobotPose *)rpose)->inverseTransformPoint(rparams, pt);
      else
        LOG_FATAL << "Unsupported robot pose type in absISpaceParametricFunctor";

      transformPoint(hec, pt);

      Camera *camera = rpose->getCamera();
      if (camera->getCameraType() == Camera::OPENCV)
        ((OpenCVCamera *)camera)->getImageSpaceResiduals(track, pt, res);
      else
          LOG_FATAL << "Unsupported camera type in absISpaceNonParametricFunctor";

      return true;
    }
  };

  class absOSpaceParametricFunctor {
    const RobotPose *rpose;
    const Track *track;

    public:
    absOSpaceParametricFunctor(const RobotPose * const _rpose, const Track * const _track)
    : rpose(_rpose), track(_track) {}

    template <typename T> bool operator() (T const* const* params, T * const res) const {
      T pt[3];

      const T *hec = params[0];
      const T *wbc = params[1];
      const T *scale = params[2];
      const T *rparams = params[3];

      track->getPoint(pt);

      scalePoint(scale, pt);
      transformPoint(wbc, pt);

      if (rpose->getPoseType() == RobotPose::MODIFIED_DENAVITHARTENBERG)
        ((ModifiedDHRobotPose *)rpose)->inverseTransformPoint(rparams, pt);
      else
        LOG_FATAL << "Unsupported robot pose type in absISpaceParametricFunctor";

      transformPoint(hec, pt);

      rpose->getCamera()->getObjectSpaceResiduals(track, pt, res);

      return true;
    }
  };

  // Non-parametric functors --------------------------------------------------

  class relISpaceNonParametricFunctor {
    const RobotPose *rpose1;
    const RobotPose *rpose2;
    const Track *track;

    public:
    relISpaceNonParametricFunctor(const RobotPose * const _rpose1, const RobotPose * const _rpose2, const Track * const _track)
      : rpose1(_rpose1), rpose2(_rpose2), track(_track) {}

    template <typename T>
    bool operator() (const T * const hec, const T * const scale, T * const res) const {
      T pt[3], c1[6], r1[6], r2[6];

      track->getPoint(pt);
      rpose1->getCamera()->getAngleAxisTranslation(c1);
      rpose1->getAngleAxisTranslation(r1);
      rpose2->getInverseAngleAxisTranslation(r2);

      transformPoint(c1, pt);
      scalePoint(scale, pt);
      inverseTransformPoint(hec, pt);
      transformPoint(r1, pt);
      transformPoint(r2, pt);
      transformPoint(hec, pt);

      Camera *camera = rpose2->getCamera();
      if (camera->getCameraType() == Camera::OPENCV)
        ((OpenCVCamera *)camera)->getImageSpaceResiduals(track, pt, res);
      else
        {
          LOG_FATAL << "Unsupported camera type in relParametricFunctor";
          return false;
        }

      return true;
    }
  };

  class relOSpaceNonParametricFunctor {
    const RobotPose *rpose1;
    const RobotPose *rpose2;
    const Track *track;

    public:
    relOSpaceNonParametricFunctor(const RobotPose * const _rpose1, const RobotPose * const _rpose2, const Track * const _track)
      : rpose1(_rpose1), rpose2(_rpose2), track(_track) {}

    template <typename T>
    bool operator() (const T * const hec, const T * const scale, T * const res) const {
      T pt[3], c1[6], r1[6], r2[6];

      track->getPoint(pt);
      rpose1->getCamera()->getAngleAxisTranslation(c1);
      rpose1->getAngleAxisTranslation(r1);
      rpose2->getInverseAngleAxisTranslation(r2);

      transformPoint(c1, pt);
      scalePoint(scale, pt);
      inverseTransformPoint(hec, pt);
      transformPoint(r1, pt);
      transformPoint(r2, pt);
      transformPoint(hec, pt);

      rpose2->getCamera()->getObjectSpaceResiduals(track, pt, res);

      return true;
    }
  };

  class absISpaceNonParametricFunctor {
    const RobotPose *rpose;
    const Track *track;

    public:
    absISpaceNonParametricFunctor(const RobotPose * const _rpose, const Track * const _track)
    : rpose(_rpose), track(_track) {}

    template <typename T>
    bool operator() (const T * const hec, const T * const wbc, const T * const scale, T * const res) const {
      T pt[3], ir[6];

      track->getPoint(pt);
      rpose->getInverseAngleAxisTranslation(ir);

      scalePoint(scale, pt);
      transformPoint(wbc, pt);
      transformPoint(ir, pt);
      transformPoint(hec, pt);

      Camera *camera = rpose->getCamera();

      if (camera->getCameraType() == Camera::OPENCV)
        ((OpenCVCamera *)camera)->getImageSpaceResiduals(track, pt, res);
      else
        {
          LOG_FATAL << "Unsupported camera type in absOSpaceNonParametricFunctor";
          return false;
        }

      return true;
    }
  };

  class absOSpaceNonParametricFunctor {
    const RobotPose *rpose;
    const Track *track;

    public:
    absOSpaceNonParametricFunctor(const RobotPose * const _rpose, const Track * const _track)
    : rpose(_rpose), track(_track) {}

    template <typename T>
    bool operator() (const T * const hec, const T * const wbc, const T * const scale, T * const res) const {
      T pt[3], ir[6];

      track->getPoint(pt);
      rpose->getInverseAngleAxisTranslation(ir);

      scalePoint(scale, pt);
      transformPoint(wbc, pt);
      transformPoint(ir, pt);
      transformPoint(hec, pt);

      rpose->getCamera()->getObjectSpaceResiduals(track, pt, res);

      return true;
    }
  };

};


#endif /* COSTFUNCS_H_ */
