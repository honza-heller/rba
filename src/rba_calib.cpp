/*
 * rba_calib.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: jheller
 */

#include "definitions.h"
#include "rba.h"
#include "camera.h"
#include "rpose.h"
#include "costfuncs.h"

#include <ceres/ceres.h>


void RoboticBundleAdjuster::computeTsai89Calibration(const list<ETrans> & crposes, const list<ETrans> & rrposes,
                                                     ETrans & hec, double & dscale, const bool trans_flag = true) const
{
  int i;
  double r,nt, na, nb;
  list<ETrans>::const_iterator citer;
  list<ETrans>::const_iterator riter;
  Vector3f arot, brot, trot, ta, tb, tx;
  Matrix3f M = Matrix3f::Zero(), Ra, Rx;

  LOG_INFO << "Running Tsai89 hand-eye calibration with " << crposes.size() << " relative poses";

  if (crposes.size() != rrposes.size())
    LOG_FATAL << "Number of relative camera poses (" << crposes.size() <<
      ") does not match the number of relative robot poses (" << rrposes.size() <<")";

  if (crposes.size() < 2)
    LOG_FATAL << "At least 2 relative poses needed for Tsai89 calibration :" << crposes.size();

  Eigen::MatrixXd c(3 * crposes.size(), 3);
  Eigen::MatrixXd d(3 * crposes.size(), 1);

  for (citer = crposes.begin(), riter = rrposes.begin(), i = 0;
       citer != crposes.end();
       citer++, riter++, i++)
    {
      (*citer).getAngleAxis(arot);
      (*riter).getAngleAxis(brot);

      na = arot.norm();
      nb = brot.norm();

      arot *= 2.0f * std::sin(na / 2.0f) / na;
      brot *= 2.0f * std::sin(nb / 2.0f) / nb;

      trot = arot + brot;
      M(0, 1) = -trot(2);
      M(1, 0) =  trot(2);
      M(0, 2) =  trot(1);
      M(2, 0) = -trot(1);
      M(1, 2) = -trot(0);
      M(2, 1) =  trot(0);

      c.block(i * 3, 0, 3, 3) = M;
      d.block(i * 3, 0, 3, 1) = arot - brot;
    }

  trot = c.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(d);
  nt = trot.norm();
  r = 2.0f * std::atan(nt);
  trot *= r / nt;

  hec.setAngleAxis(trot);
  hec.inverse();

  if (!trans_flag)
    return;

  hec.getRotation(Rx);

  if (!scale_init)
    c.resize(3 * crposes.size(), 4);

  for (citer = crposes.begin(), riter = rrposes.begin(), i = 0;
       citer != crposes.end();
       citer++, riter++, i++)
    {
      (*citer).getRotation(Ra);
      (*citer).getTranslation(ta);
      (*riter).getTranslation(tb);

      if (!scale_init)
        {
          c.block(i * 3, 0, 3, 3) = Ra - Matrix3f::Identity();
          c.block(i * 3, 3, 3, 1) = ta;
          d.block(i * 3, 0, 3, 1) = Rx * tb;
        }
      else
        {
          c.block(i * 3, 0, 3, 3) = Ra - Matrix3f::Identity();
          d.block(i * 3, 0, 3, 1) = Rx * tb - scale * ta;
        }
    }

  Eigen::MatrixXd u = c.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(d);

  tx = u.block(0, 0, 3, 1);
  hec.setMatrix(Rx, tx);

  if (!scale_init)
    dscale = u(3);
}

void RoboticBundleAdjuster::computeRelativePoses(list<ETrans> & crposes, list<ETrans> & rrposes, const bool rtype) const
{
  Matrix4f aAbs1, aAbs2;
  Matrix4f bAbs1, bAbs2;
  ETrans aRel, bRel;

  LOG_INFO << "Computing relative poses using all available pose pairs";

  const vector<Camera*> & cameras = scene.getCameras();
  const vector<RobotPose*> & rposes = robot.getPoses();

  int no_cposes = scene.getNumCameras();
  int no_rposes = robot.getNumPoses();

  if (no_cposes != no_rposes)
    LOG_FATAL << "The number of cameras (" << no_cposes << ") does not match number of robot poses (" << no_rposes << ")";

  crposes.clear();
  rrposes.clear();

  for (int i = 0; i < no_cposes; i++)
    {
      if (!cameras[i]->isCalibCamera())
        continue;

      for (int j = i + 1; j < no_cposes; j++)
        {
          if (!cameras[j]->isCalibCamera())
            continue;

          if (rtype == true)
            { // aRel * HEC = HEC * bRel
              aAbs1 = cameras[i]->getPose()->getInverseMatrix();
              aAbs2 = cameras[j]->getPose()->getMatrix();
              bAbs1 = rposes[i]->getMatrix();
              bAbs2 = rposes[j]->getInverseMatrix();
            }
          else
            { // aRel * WBC^-1 = WBC^-1 * bRel
              aAbs1 = cameras[i]->getPose()->getMatrix();
              aAbs2 = cameras[j]->getPose()->getInverseMatrix();
              bAbs1 = rposes[i]->getInverseMatrix();
              bAbs2 = rposes[j]->getMatrix();
            }

          aRel.setMatrix(aAbs2 * aAbs1);
          bRel.setMatrix(bAbs2 * bAbs1);

          crposes.push_back(aRel);
          rrposes.push_back(bRel);
        }
    }
}

void RoboticBundleAdjuster::recoverInitCalibRelativePose(void)
{
  list<ETrans> crposes, rrposes;

  LOG_INFO << "Recovering initial Hand-Eye calibrations";

  computeRelativePoses(crposes, rrposes, true);
  computeTsai89Calibration(crposes, rrposes, hec, scale, true);
}

void RoboticBundleAdjuster::recoverInitCalibAbsolutePose(void)
{
  int i, j;
  Matrix3f Rx, Rz, Ra, iRa, Rb, U, V;
  Vector3f tx, tz, ita, tb;
  list<ETrans> crposes, rrposes;
  ETrans thec;
  double dscale;

  LOG_INFO << "Recovering initial Hand-Eye & World-Base calibrations";

  vector<Camera*> const & cameras = scene.getCameras();
  vector<RobotPose*> const & rposes = robot.getPoses();

  int no_cposes = scene.getNumCameras();
  int no_calib = scene.getNumCalibCameras();
  int no_rposes = robot.getNumPoses();

  if (no_cposes != no_rposes)
    LOG_FATAL << "The number of cameras (" << no_cposes << ") does not match number of robot poses (" << no_rposes << ")";

  if (!hec_init)
    {
      LOG_INFO << "Initial Hand-Eye calibration unknown, running calibration";
      computeRelativePoses(crposes, rrposes, true);
      computeTsai89Calibration(crposes, rrposes, thec, dscale, false);
      thec.getRotation(Rx);
    }
  else
    {
      LOG_INFO << "Using known initial Hand-Eye calibration";
      hec.getRotation(Rx);
    }

  if (!wbc_init)
    {
      LOG_INFO << "Initial World-Base calibration unknown, running calibration";
      computeRelativePoses(crposes, rrposes, false);
      computeTsai89Calibration(crposes, rrposes, thec, dscale, false);
      thec.getRotation(Rz);
    }
  else
    {
      LOG_INFO << "Using known initial World-Base calibration";
      wbc.getRotation(Rz);
      Rz.transposeInPlace();
    }

  Eigen::MatrixXd J;
  Eigen::MatrixXd b(3 * no_calib, 1);

  if (!scale_init)
    J.resize(3 * no_calib, 7);
  else
    J.resize(3 * no_calib, 6);

  for (i = 0, j = 0; i < (int) cameras.size(); i++)
    {
      if (!cameras[i]->isCalibCamera())
        continue;

      cameras[i]->getPose()->getInverseMatrix(iRa, ita);
      tb = rposes[i]->getMatrix().block(0, 3, 3, 1);

      J.block(j*3, 0, 3, 3) = iRa;
      J.block(j*3, 3, 3, 3) = -Matrix3f::Identity();

      if (!scale_init)
        {
          J.block(j*3, 6, 3, 1) = ita;
          b.block(j*3, 0, 3, 1) = Rz * tb;
        }
      else
        {
          b.block(j*3, 0, 3, 1) = Rz * tb - scale * ita;
        }
      j++;
    }

 // u = J \ b
  Eigen::MatrixXd u = J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  tx = u.block(0, 0, 3, 1);
  Rz.transposeInPlace();
  tz = -Rz * u.block(3, 0, 3, 1);

  if (!hec_init)
    hec.setMatrix(Rx, tx);

  if (!wbc_init)
    wbc.setMatrix(Rz, tz);

  if (!scale_init)
    scale = u(6);
}

void RoboticBundleAdjuster::bundleAdjustCalibration(void)
{
  LOG_INFO << "Running Calibration BA";

  double dhec[6], dwbc[6], dscale, *rparams, no_rparams;
  int no_projections = 0;
  vector<int> rpmask;

  ceres::Problem problem;

  vector<RobotPose*> const &rposes = robot.getPoses();

  // Set initial calibration
  hec.getAngleAxisTranslation(dhec);
  wbc.getAngleAxisTranslation(dwbc);
  dscale = scale;

  no_rparams = robot.getParameters(rparams, rpmask);

  // Construct ceres optimization problem
  if (!opts.calib_relpose)
    {
      for (int i = 0; i < (int) rposes.size(); i++)
        {
          RobotPose *rpose = rposes[i];
          Camera *camera = rpose->getCamera();
          if (!camera->isCalibCamera())
            continue;

          list<Track*> const &tracks = camera->getTracks();
          for (list<Track*>::const_iterator itrack = tracks.begin(); itrack != tracks.end(); itrack++, no_projections++)
            RbaCostFunctions::addResidual(opts.calib_ospace, rpose, *itrack, dhec, dwbc,
                                          &dscale, no_rparams, rparams, &problem);
        }
    }
  else
    {
      for (int i = 0; i < (int) rposes.size(); i++)
        {
          RobotPose *rpose1 = rposes[i];
          Camera *camera1 = rpose1->getCamera();

          if (!camera1->isCalibCamera())
            continue;

          for (int j = 0; j < (int) rposes.size(); j++)
            {
              RobotPose *rpose2 = rposes[j];
              Camera *camera2 = rpose2->getCamera();

              if (!camera2->isCalibCamera() || (i == j))
                continue;

              list<Track*> const &tracks = camera2->getTracks();

              for (list<Track*>::const_iterator itrack = tracks.begin(); itrack != tracks.end(); itrack++, no_projections++)
                RbaCostFunctions::addResidual(opts.calib_ospace, rpose1, rpose2, *itrack, dhec,
                                              &dscale, no_rparams, rparams, &problem);
            }
        }
    }

  // Add additional constraints
  if (opts.rdmask_is_penalty)
    robot.addLinearPenalties(&problem, no_projections);

  // Set constant parameters
  if (!opts.ba_hec)
    problem.SetParameterBlockConstant(dhec);
  if (!opts.ba_wbc && !opts.calib_relpose)
    problem.SetParameterBlockConstant(dwbc);
  if (!opts.ba_scale)
    problem.SetParameterBlockConstant(&dscale);

  if (no_rparams && (int) (rpmask.size() != 0))
    {
      if ((int) rpmask.size() < no_rparams)
        {
          SubsetParameterization *subparam =  new SubsetParameterization(no_rparams, rpmask);
          problem.SetParameterization(rparams, subparam);
        }
      else
        problem.SetParameterBlockConstant(rparams);
    }

  // Solve BA problem
  Solve(ceres_options, &problem, &ceres_summary);
  LOG_INFO << ceres_summary.FullReport() << endl;

  // Save calibration results
  hec.setAngleAxisTranslation(dhec);
  wbc.setAngleAxisTranslation(dwbc);
  scale = dscale;

  robot.setParameters(rparams);
}

void RoboticBundleAdjuster::calibrate(void)
{
  scene.recoverScene();

  // if rposes_txt not set, then perform camera calibration only
  if (opts.rposes_txt == "")
    return;

  robot.setScene(&scene);

  if (opts.calib_relpose)
    recoverInitCalibRelativePose();
  else
  recoverInitCalibAbsolutePose();

  bundleAdjustCalibration();

  robot.updateResidualErrors(hec, wbc, scale);
}
