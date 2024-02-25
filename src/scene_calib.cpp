/*
 * scene.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: hellej1
 */

#ifdef OPENMP_FOUND

#include "omp.h"

#endif

#include <stdexcept>
#include <string>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "definitions.h"
#include "cmodelfuncs.h"
#include "scene.h"

#include "camera_basic.h"
#include "camera_opencv.h"
#include "track_calibdev.h"

#include "tdetector_acircles.h"
#include "tdetector_chboard.h"
#include "tdetector_circles.h"
// #include "tdetector_lmcircles.h"
// #include "tdetector_lmchboard.h"
// #include "tdetector_lmcorners.h"

// Calibration

Vector6f Scene::getV(const Matrix3f &H, const int i, const int j) const
{
  Vector6f v;

  v(0) = H(0, i) * H(0, j);
  v(1) = H(0, i) * H(1, j) + H(1, i) * H(0, j);
  v(2) = H(1, i) * H(1, j);
  v(3) = H(2, i) * H(0, j) + H(0, i) * H(2, j);
  v(4) = H(2, i) * H(1, j) + H(1, i) * H(2, j);
  v(5) = H(2, i) * H(2, j);

  return v;
}

bool Scene::getKfromB(const Matrix3f &B, Matrix3f &K) const
{
  double v0 = (B(0, 1) * B(0, 2) - B(0, 0) * B(1, 2)) / (B(0, 0) * B(1, 1) - B(0, 1) * B(0, 1));
  double l = B(2, 2) - (B(0, 2) * B(0, 2) + v0 * (B(0, 1) * B(0, 2) - B(0, 0) * B(1, 2))) / B(0, 0);

  double a0 = double(l / B(0, 0));
  double b0 = double(l * B(0, 0) / (B(0, 0) * B(1, 1) - B(0, 1) * B(0, 1)));

  if ((a0 <= 0.0f) && (b0 > 0.0f))
    a0 = b0;
  else if ((a0 > 0.0f) && (b0 <= 0.0f))
    b0 = a0;
  else if ((a0 <= 0.0f) || (b0 <= 0.0f))
    {
      cout << "A0 = " << a0 << endl;
      cout << "B0 = " << b0 << endl;

      return false;
    }

  double a = std::sqrt(a0);
  double b = std::sqrt(b0);

  double g = -B(0, 1) * a * a * b / l;
  double u0 = g * v0 / b - B(0, 2) * a * a / l;

  K << a, 0, u0,
       0, b, v0,
       0, 0, 1.0f;

  //std::cout << " getKfromB B = " << endl << B << endl << std::endl;
  //std::cout << " getKfromB K = " << endl << K << endl << std::endl;
  return true;
}

void Scene::recoverCamerasZhang(void)
{
  vector<int> detcams;
  OpenCVCamera *camera;
  vector<Matrix3f> Hs;
  
  LOG_INFO << "Running camera calibrations (Zhang98)";

  if (cameras.size() == 0)
    LOG_FATAL << "There are no cameras to be calibrated";

  // Find cameras that can be calibrated
  for (unsigned int i = 0; i < cameras.size(); i++)
    {
       camera = (OpenCVCamera *) cameras[i];
       list<Track*> tracks = camera->getTracks();
       if (tracks.size() > 0 && (!camera->getXValidFlag()))
         detcams.push_back(i);
    }

  if (detcams.size() == 0)
    LOG_FATAL << "Not enough cameras with detected points";

  Hs.resize(detcams.size());
  MatrixXf A(detcams.size() * 2, 6);
  MatrixXf Avp(detcams.size() * 2, 2);
  VectorXf bvp(detcams.size() * 2);
  Matrix3f B, K, iK;
  Vector8f dist = Vector8f::Zero();

  A.setZero();

  for (int i = 0; i < (int) detcams.size(); i++)
    {
      Matrix3f H, Hc;

      OpenCVCamera *camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->recoverHomography(Hs[i]);

      A.block<1,6>(i * 2,     0) = getV(Hs[i], 0, 1);
      A.block<1,6>(i * 2 + 1, 0) = getV(Hs[i], 0, 0) - getV(Hs[i], 1, 1);
    }

  Eigen::JacobiSVD<MatrixXf> svdA(A, Eigen::ComputeThinV);
  const MatrixXf &V = svdA.matrixV();

  B << V(0, 5), V(1, 5), V(3, 5),
       V(1, 5), V(2, 5), V(4, 5),
       V(3, 5), V(4, 5), V(5, 5);

  if (!getKfromB(B, K))
    {
      LOG_WARN << "Cannot recover K matrix using Zhang98, using a guess instead";

      OpenCVCamera *camera = (OpenCVCamera *) cameras[detcams[0]];
      double w = camera->getImageWidth();
      double h = camera->getImageHeight();
      K << w / 2.0f, 0, w / 2.0f, 0, w / 2.0f, h / 2.0f, 0, 0, 1.0f;
    }

  iK = K.inverse();
  
  for (unsigned int i = 0; i < detcams.size(); i++)
    {
      Matrix3f R;
      Vector3f t, C;

      Camera::decomposeH(Hs[i], iK, R, t);
      C = - R.transpose() * t;
      if (C(2) < 0.0)
        {
          R.block<3, 1>(0, 0) *= -1;
          R.block<3, 1>(0, 1) *= -1;
          t = -t;
        }
    
      OpenCVCamera *camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->setPose(ETrans(R, t));
    }

  for (int i = 0; i < no_cameras; i++)
    {
      camera = (OpenCVCamera *) cameras[i];
      camera->setIntrinsics(K, dist);
    }

}

void Scene::recoverCamerasBouguet(void)
{
  vector<int> detcams;
  OpenCVCamera *camera;
  vector<Matrix3f> Hs;
  Matrix3f Sc;
  Vector2f c_0;
  
  LOG_INFO << "Running camera calibrations (Bouguet Matlab calibration toolbox)";

  if (cameras.size() == 0)
    LOG_FATAL << "There are no cameras to be calibrated";

  // Find cameras that can be calibrated
  for (unsigned int i = 0; i < cameras.size(); i++)
    {
       camera = (OpenCVCamera *) cameras[i];
       list<Track*> tracks = camera->getTracks();
       if (tracks.size() > 0 && (!camera->getXValidFlag()))
         detcams.push_back(i);
    }

  if (detcams.size() == 0)
    LOG_FATAL << "Not enough cameras with detected points";

  Hs.resize(detcams.size());
  MatrixXf Avp(detcams.size() * 2, 2);
  VectorXf bvp(detcams.size() * 2);
  Matrix3f B, K, iK;
  Vector8f dist = Vector8f::Zero();
  
  // Princial point initialization
  camera = (OpenCVCamera *) cameras[detcams[0]];
  c_0 = 0.5f * Vector2f(camera->getImageWidth(), camera->getImageHeight()) - Vector2f(0.5f, 0.5f);
  Sc << 1, 0, -c_0(0), 0, 1, - c_0(1), 0, 0, 1;
  
  for (int i = 0; i < (int) detcams.size(); i++)
    {
      Matrix3f H, Hc;

      OpenCVCamera *camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->recoverHomography(Hs[i]);
      
      Hc = Sc * Hs[i];  
      
      Vector3f Vhp = Hc.col(0).normalized();
      Vector3f Vvp = Hc.col(1).normalized();
      Vector3f Vd1 = (0.5f * (Hc.col(0) + Hc.col(1))).normalized();
      Vector3f Vd2 = (0.5f * (Hc.col(0) - Hc.col(1))).normalized();
      
      Avp.block<2, 2>(i * 2, 0) << Vhp(0) * Vvp(0), Vhp(1) * Vvp(1), Vd1(0) * Vd2(0), Vd1(1) * Vd2(1);
      bvp.segment<2>(i * 2) << -Vhp(2) * Vvp(2), -Vd1(2) * Vd2(2);
    }
    
  Vector2f f_0 = Avp.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bvp);
  f_0 = f_0.cwiseInverse().cwiseAbs().cwiseSqrt();
  double f = 0.5 * (f_0(0) + f_0(1));
  
  K << f, 0, c_0(0), 0, f, c_0(1), 0, 0, 1.0f;
  iK = K.inverse();

  cout << "rbaK = " << endl << K << endl << endl;

  for (unsigned int i = 0; i < detcams.size(); i++)
    {
      Matrix3f R;
      Vector3f t, C;

      Camera::decomposeH(Hs[i], iK, R, t);
      C = - R.transpose() * t;

      if (C(2) < 0.0)
        {
          R.block<3, 1>(0, 0) *= -1;
          R.block<3, 1>(0, 1) *= -1;
          t = -t;
        }
        
      OpenCVCamera *camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->setPose(ETrans(R, t));
    }

  for (int i = 0; i < no_cameras; i++)
    {
      camera = (OpenCVCamera *) cameras[i];
      camera->setIntrinsics(K, dist);
    }

}


void Scene::recoverCameras(void)
{
  vector<int> detcams;
  OpenCVCamera *camera;
  double cx, cy;

  LOG_INFO << "Running internal camera calibration";

  if (cameras.size() == 0)
    LOG_FATAL << "There are no cameras to be calibrated";

  // Find cameras that can be calibrated
  for (unsigned int i = 0; i < cameras.size(); i++)
    {
       camera = (OpenCVCamera *) cameras[i];
       list<Track*> tracks = camera->getTracks();
       if (tracks.size() > 30 && (!camera->getXValidFlag()))
         detcams.push_back(i);
    }

  if (detcams.size() == 0)
    LOG_FATAL << "Not enough cameras with detected points";

  vector<double> focal_lengths(detcams.size());
  vector<bool> bad_pose(detcams.size());

//#pragma omp parallel for
  for (unsigned int i = 0; i < detcams.size(); i++)
    {
      Matrix3f R, K;
      Vector3f t;
      Vector8f dist;
      double k;

      OpenCVCamera *camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->recoverPoseKPlanar(R, t, K);
      camera->setPose(ETrans(R, t));
      /*
      camera->setIntrinsics(K, Vector8f::Zero());
      camera->updateDetections();
      camera->bundleAdjustPose();
      camera->getIntrinsics(K, dist);
       */

      focal_lengths[i] = K(0,0);

      double maxf = std::max(double(K(0,0)), double(K(1,1)));
      double minf = std::min(double(K(0,0)), double(K(1,1)));

      bad_pose[i] = (((maxf - minf) / maxf) > 0.2) ? true : false;
    }

  // common focal length as median value
  camera = (OpenCVCamera *) cameras[detcams[0]];
  cx = (camera->getImageWidth() - 1.0f) / 2.0f;
  cy = (camera->getImageHeight() - 1.0f) / 2.0f;

  size_t n = focal_lengths.size() / 2;
  nth_element(focal_lengths.begin(), focal_lengths.begin() + n, focal_lengths.end());
  double flength = focal_lengths[n-1];

  Matrix3f K;
  Vector8f dist;

  K << flength, 0, cx, 0, flength, cy, 0, 0, 1;
  dist << 0, 0, 0, 0, 0, 0, 0, 0;

  for (unsigned int i = 0; i < detcams.size(); i++)
    {
      cout << K << endl << endl;
    }


  for (int i = 0; i < no_cameras; i++)
    {
      camera = (OpenCVCamera *) cameras[i];
      camera->setIntrinsics(K, dist);
    }
}

void Scene::bundleAdjustCameras(void)
{
  bundleAdjustCameras(opts->cmodel_mask, (OpenCVCamera::CameraModelType) opts->cmodel_type);
}

void Scene::bundleAdjustCameras(const vector<int> & cmodel_mask, const CameraModels::CameraModelType &cmodel_type)
{
  unsigned int i;
  OpenCVCamera *camera;
  vector<int> detcams;
  std::vector<int> lcvars;
  double (*cposes)[6] = new double[no_cameras][6];
  double ck[4];
  double cd[8];

  ceres::Problem problem;
  ceres::CostFunction *cost_function;
  Solver::Summary summary;
  Solver::Options coptions;

  LOG_INFO << "Running scene bundle adjustment, camera type " << cmodel_type;

  for (i = 0; i < cameras.size(); i++)
    {
      if (cameras[i]->getCameraType() != Camera::OPENCV)
        LOG_FATAL << "Camera type != OpenCV in scene BA";

      camera = (OpenCVCamera *) cameras[i];

      camera->setAspectRatioFlag((bool) cmodel_mask[7]);
      camera->setCameraModelType((CameraModels::CameraModelType) cmodel_type);
      if ((camera->getNumTracks() > 0) && (!camera->getXValidFlag()))
        detcams.push_back(i);
    }

  camera = (OpenCVCamera *) cameras[detcams[0]];
  camera->getIntrinsics(ck, cd);

  if (detcams.size() == 0)
    LOG_FATAL << "Not enough cameras with points for scene BA";

  for (int i = 0; i < (int) detcams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[detcams[i]];

      list<Track*> const & tracks = camera->getTracks();
      camera->getPose()->getAngleAxisTranslation(cposes[i]);

      LOG_INFO << "Adding camera no. " << detcams[i] << " to BA";

      for (list<Track*>::const_iterator iter = tracks.begin(); iter != tracks.end(); iter++)
        {
          cost_function =
            new AutoDiffCostFunction<OpenCVCamera::reprojectionErrorCalib, 2, 4, 8, 6>(
            new OpenCVCamera::reprojectionErrorCalib(camera, *iter));
          problem.AddResidualBlock(cost_function, NULL, ck, cd, cposes[i]);
        }
    }

  lcvars.clear();
  if (!cmodel_mask[0]) lcvars.push_back(0);
  if (!cmodel_mask[1]) lcvars.push_back(1);
  if (!cmodel_mask[2]) lcvars.push_back(4);
  if (!cmodel_mask[3]) lcvars.push_back(5);
  if (!cmodel_mask[4]) lcvars.push_back(6);
  if (!cmodel_mask[5]) lcvars.push_back(7);
  if (!cmodel_mask[6])
    {
      lcvars.push_back(2);
      lcvars.push_back(3);
    }

  if (lcvars.size() == 8)
    problem.SetParameterBlockConstant(cd);
  else if (lcvars.size() > 0)
    {
      SubsetParameterization *subset_parameterization = new SubsetParameterization(8, lcvars);
      problem.SetParameterization(cd, subset_parameterization);
    }

  if (!cmodel_mask[7])
    {
      lcvars.clear();
      lcvars.push_back(1);
      SubsetParameterization *subset_parameterization = new SubsetParameterization(4, lcvars);
      problem.SetParameterization(ck, subset_parameterization);
    }

    // Ceres Options
    coptions.linear_solver_type = ceres::SPARSE_SCHUR;//     DENSE_NORMAL_CHOLESKY;
    coptions.num_threads = opts->no_threads;
    //coptions.num_linear_solver_threads = opts->no_threads;
    coptions.use_nonmonotonic_steps = true;
    coptions.minimizer_progress_to_stdout = false;
    //coptions.function_tolerance = 1e-15;
    //coptions.gradient_tolerance = 1e-15;
    //coptions.parameter_tolerance = 1e-15;

    Solve(coptions, &problem, &summary);
    LOG_INFO << summary.FullReport() << "\n";

    LOG_INFO << "Setting camera calibration";

    if (!cmodel_mask[7])
      ck[1] = ck[0];

    // Set new internal camera parameters for all cameras
    for (i = 0; i < cameras.size(); i++)
      {
        camera = (OpenCVCamera *) cameras[i];
        camera->setIntrinsics(ck, cd);
      }

    for (i = 0; i < detcams.size(); i++)
      {
        camera = (OpenCVCamera *) cameras[detcams[i]];
        camera->getPose()->setAngleAxisTranslation(cposes[i]);
      }

   delete [] cposes;
}

class AsinError {
private:
  double rnorm;
  double angle;

public:
  AsinError(const double & _rnorm, const double &_angle) : rnorm(_rnorm), angle(_angle) {}

  template <typename T>
  bool operator()(const T* const k, T* const res) const {
    T d = T(rnorm) / k[0];
    res[0] = T(angle) - k[1] * incasin(d);

    return true;
  }

};

void Scene::updateRational2FisheyeP8PModel(void)
{
  vector<int> detcams;
  OpenCVCamera *camera;
  int no_tracks;

  Matrix3f K;
  Vector8f dist;

  LOG_INFO << "Running Distortion model update from Rational to FisheyeP8P";

  K.setZero();
  dist.setZero();

  if (cameras.size() == 0)
    LOG_FATAL << "There are no cameras to be calibrated";

  // Find cameras that can be calibrated
  no_tracks = 0;
  for (unsigned int i = 0; i < cameras.size(); i++)
    {
       camera = (OpenCVCamera *) cameras[i];
       list<Track*> tracks = camera->getTracks();
       if (tracks.size() > 0 && (!camera->getXValidFlag()))
         {
           detcams.push_back(i);
           no_tracks += tracks.size();
         }
    }

  if (detcams.size() == 0)
    LOG_FATAL << "Not enough cameras with detected points";

  // Fit 2-parametric rational model & Update to 2-parameters SIN model
  MatrixXf A(no_tracks, 2);
  VectorXf b(no_tracks);
  double k[2] = {0.0f, 1.0f};
  int c = 0;

  ceres::Problem         problem;
  ceres::Solver::Options coptions;
  ceres::Solver::Summary summary;
  ceres::CostFunction    *cost_function;

  for (int i = 0; i < (int) detcams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->getIntrinsics(K, dist);
      Vector2f center = Vector2f(K(0, 2), K(1, 2));

      list<Track*> const & tracks = camera->getTracks();

      for (list<Track*>::const_iterator iter = tracks.begin(); iter != tracks.end(); iter++)
        {
          Vector4f cpr1 = camera->getPose()->getMatrix() * (*iter)->getPoint(Track::SOURCEPT);
          Vector3f cpr2 = cpr1.head<3>().normalized();
          double angle = std::acos(double(cpr2(2)));

          Vector2f ipr = (*iter)->getDetection(detcams[i], Track::SOURCE);
          double rnorm = (ipr - center).norm();

          A(c, 0) = -rnorm;
          A(c, 1) = angle * rnorm * rnorm;
          b(c) = -angle;
          c++;

          cost_function = new ceres::AutoDiffCostFunction<AsinError, 1, 2>(new AsinError(rnorm, angle));
          problem.AddResidualBlock(cost_function, NULL, k);
        }
    }

  Vector2f rs = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);


  cout << "rs = " << rs(0) << " " << rs(1) << endl;


  double denom = 2.0f * rs(1) * PI_2;

  if (denom != 0.0f)
    {
      double d = std::sqrt(double(rs(0) * rs(0) - 2.0f * denom * PI_2));
      k[0] = std::max((rs(0) + d) / denom, (rs(0) - d) / denom);
    }

  // Nonlinear optimization of the 2-parameter SIN model
  coptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  coptions.num_threads = 1;
  //coptions.num_linear_solver_threads = 1;
  coptions.use_nonmonotonic_steps = true;
  coptions.minimizer_progress_to_stdout = false;
  coptions.logging_type = ceres::SILENT;

  cout << "K = " << k[0] << " " << k[1] << endl;

  ceres::Solve(coptions, &problem, &summary);

  cout << "K = " << k[0] << " " << k[1] << endl;

  // Update camera intrinsics
  double rad_fov = k[0] * incsin(k[1] * PI_2);
  K(0, 0) = rad_fov;
  K(1, 1) = rad_fov;
  dist.setZero();
  dist(0) = k[0] / rad_fov;
  dist(1) = k[1];

  for (int i = 0; i < (int) detcams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->setIntrinsics(K, dist);
    }

  opts->cmodel_type = CameraModels::FISHEYE_P8P;

  bundleAdjustCameras();

  for (int i = 0; i < (int) detcams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->updateDetections();
    }
}

void Scene::updateRational2FisheyeE7PModel(void)
{
  vector<int> detcams;
  vector<int> nocams;
  OpenCVCamera *camera;
  int no_tracks;

  Matrix3f K;
  Vector8f dist;

  LOG_INFO << "Running Distortion model update from Rational to FisheyeE7P";

  K.setZero();
  dist.setZero();

  if (cameras.size() == 0)
    LOG_FATAL << "There are no cameras to be calibrated";

  // Find cameras that can be calibrated
  no_tracks = 0;
  for (unsigned int i = 0; i < cameras.size(); i++)
    {
       camera = (OpenCVCamera *) cameras[i];
       list<Track*> tracks = camera->getTracks();
       if (tracks.size() > opts->target_mindets && (!camera->getXValidFlag()))
         {
           detcams.push_back(i);
           no_tracks += tracks.size();
         }
       else
         {
           nocams.push_back(i);
         }
    }

  if (detcams.size() == 0)
    LOG_FATAL << "Not enough cameras with detected points";

  MatrixXf A(no_tracks, 2);
  int c = 0;

  camera = (OpenCVCamera *) cameras[detcams[0]];
  camera->getIntrinsics(K, dist);
  Vector2f center = Vector2f(K(0, 2), K(1, 2));

  for (int i = 0; i < (int) detcams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[detcams[i]];
      list<Track*> const & tracks = camera->getTracks();

      for (list<Track*>::const_iterator iter = tracks.begin(); iter != tracks.end(); iter++)
        {
          Vector4f cpr1 = camera->getPose()->getMatrix() * (*iter)->getPoint(Track::SOURCEPT);
          Vector3f cpr2 = cpr1.head<3>().normalized();
          double angle = std::acos(double(cpr2(2)));

          Vector2f ipr = (*iter)->getDetection(detcams[i], Track::SOURCE);
          double rnorm = (ipr - center).norm();

          A(c, 0) = angle;
          A(c, 1) = rnorm;
          c++;
        }
    }

  Vector2f mean = A.colwise().sum() / no_tracks;
  A.col(0) = A.col(0).array() - mean(0);
  A.col(1) = A.col(1).array() - mean(1);

  Eigen::JacobiSVD<MatrixXf> svdA(A, Eigen::ComputeThinV);
  const MatrixXf &V = svdA.matrixV();
  Vector2f dir = V.col(0) * M_PI / (2.0f * V(0, 0));

  // Update camera intrinsics
  double rad_fov = dir(1);
  K(0, 0) = rad_fov;
  K(1, 1) = rad_fov;
  dist.setZero();
  dist(0) = 2.0f / M_PI;

  for (int i = 0; i < (int) detcams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->setIntrinsics(K, dist);
    }

  bundleAdjustCameras(opts->cmodel_mask, CameraModels::FISHEYE_E7P);

  camera = (OpenCVCamera *) cameras[detcams[0]];
  camera->getIntrinsics(K, dist);

  for (int i = 0; i < (int) nocams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[nocams[i]];
      camera->setIntrinsics(K, dist);
      camera->setPose(ETrans(Matrix4f::Zero()));
    }

  for (int i = 0; i < (int) detcams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[detcams[i]];
      camera->updateDetections();
    }
}


void Scene::updateTracks(const vector<list<Vector3f> > & points, const vector<list<Vector2f> > & detections, const vector<list<unsigned long long int> > & indices)
{
  CalibdevTrack *track;
  map<unsigned long long int, Vector3f> mtarget;
  map<unsigned long long int, list<pair<unsigned int, Vector2f> > > mtracks;

  clearTracks();
  no_tracks = 0;

  // Parse the detector results into Cameras/Tracks lists
  for (unsigned int i = 0; i < cameras.size(); i++)
    {
      list<Vector3f>::const_iterator piter = points[i].begin();
      list<Vector2f>::const_iterator diter = detections[i].begin();
      list<unsigned long long int>::const_iterator iiter = indices[i].begin();

      while (iiter != indices[i].end())
        {
          mtarget[*iiter] = *piter;
          mtracks[*iiter].push_back(make_pair(i, *diter));
          iiter++;
          piter++;
          diter++;
        }
    }

  map<unsigned long long int, Vector3f>::iterator target_iter = mtarget.begin();

  while (target_iter != mtarget.end())
    {
      unsigned long long int key = target_iter->first;
      unsigned int no_cams = mtracks[key].size();

      track = new CalibdevTrack(no_cams);
      Vector4f pt = Vector4f(target_iter->second[0], target_iter->second[1], target_iter->second[2], 1);
      track->setIndex(no_tracks);
      track->setPoint(Track::SOURCEPT, pt);
      track->setPoint(Track::TRIANGULATEDPT, pt);

      tracks.push_back(track);
      no_tracks++;

      int i = 0;
      for (list<pair<unsigned int, Vector2f> >::iterator diter = mtracks[key].begin(); diter != mtracks[key].end(); diter++)
        {
          track->setCameraIndex(diter->first, i);
          track->setDetection(i, Track::SOURCE, diter->second, true);
          cameras[diter->first]->addTrack(track);
          i++;
        }

      target_iter++;
    }
}

void Scene::initTargetDetectors(vector<TargetDetector *> & detectors)
{
  deleteTargetDetectors(detectors);
  detectors.resize(no_cameras);

  for (int i = 0; i < no_cameras; i++)
    {
      TargetDetector *target_detector = NULL;
      if (ct_type == TargetDetector::CHESSBOARD)
        target_detector = new ChessboardTargetDetector();
      else if (ct_type == TargetDetector::CIRCLES_GRID)
        target_detector = new CirclesTargetDetector();
      else if (ct_type == TargetDetector::ASYMMETRIC_CIRCLES_GRID)
        target_detector = new AsymCirclesTargetDetector();
      // else if (ct_type == TargetDetector::ELMARK_CIRCLES_GRID)
      //   target_detector = new ElMarkCirclesDetector();
      // else if (ct_type == TargetDetector::ELMARK_CHESSBOARD_GRID)
      //   target_detector = new ElMarkChessboardDetector();
      // else if (ct_type == TargetDetector::ELMARK_CIRCLES_CORNERS)
      //   target_detector = new ElMarkCornersDetector();

      target_detector->setWidth(ct_width);
      target_detector->setHeight(ct_height);
      target_detector->setStrides(ct_xstride, ct_ystride);
      target_detector->setOptions(opts);

      detectors[i] = target_detector;
    }
}

void Scene::deleteTargetDetectors(vector<TargetDetector *> & detectors)
{
  for (int i = 0; i < (int) detectors.size(); i++)
    {
      if (detectors[i])
        delete detectors[i];
      detectors[i] = NULL;
    }

  detectors.clear();
}

void Scene::runTargetDetectors(vector<TargetDetector *> & detectors)
{
  string str_err = "";

#pragma omp parallel for schedule(dynamic, 1)
  for (int i = 0; i < no_cameras; i++)
    {
      try
      {
          if (detectors[i]->detect(cameras[i]->getImagePath().c_str()))
            LOG_INFO << "Target detected: " << cameras[i]->getImagePath();
          else
            LOG_WARN << "Target NOT detected: " << cameras[i]->getImagePath();
      }
      catch (exception& e)
      {
#pragma omp critical(error_detector)
          {
            str_err = e.what();
          }
      }
    }

  if (str_err != "")
    throw(runtime_error(str_err));
}

void Scene::identifyTargetDetections(vector<TargetDetector *> & detectors)
{
  vector<list<Vector3f> > points(no_cameras);
  vector<list<Vector2f> > detections(no_cameras);
  vector<list<unsigned long long int> > indices(no_cameras);

  Matrix3f K;
  Vector8f dist;
  vector<int> cmodel_mask(8, 0);
  OpenCVCamera::CameraModelType cmodel_type, opts_cmodel_type;
  opts_cmodel_type = (OpenCVCamera::CameraModelType) opts->cmodel_type;
  
  cmodel_mask = opts->cmodel_mask;
  cmodel_type = opts_cmodel_type;

  // Simple detectors that cannot be guided by geometry
  if ((ct_type == TargetDetector::CHESSBOARD) ||
      (ct_type == TargetDetector::CIRCLES_GRID) ||
      (ct_type == TargetDetector::ASYMMETRIC_CIRCLES_GRID))
    {
      // Identify targets
      for (int i = 0; i < detectors.size(); i++)
        {
          detectors[i]->identify();
          detectors[i]->getDetections(points[i], detections[i], indices[i]);
        }

      updateTracks(points, detections, indices);
      return;
    }

  // If camera calibration is user supplied, use it for target identifications
  if (scene_type == CALIBDEV_4)
    {
      ((OpenCVCamera *) cameras[0])->getIntrinsics(K, dist);

      throw std::runtime_error("ElMarkTargetDetector not supported");
      // for (int i = 0; i < detectors.size(); i++)
      //   ((ElMarkTargetDetector *) detectors[i])->setCameraParameters(K, dist, cmodel_mask, cmodel_type);

      // Identify targets
      for (int i = 0; i < detectors.size(); i++)
        {
          detectors[i]->identify();
          detectors[i]->getDetections(points[i], detections[i], indices[i]);

          if (indices[i].size() < opts->target_mindets)
            {
              points[i].clear();
              detections[i].clear();
              indices[i].clear();
            }               
        }
        
      updateTracks(points, detections, indices);
      return;
    }

  // If not fisheye and not CALIBDEV_4
  if (!CameraModels::isFishEye(opts_cmodel_type))
    {
      // Identify targets
      for (int i = 0; i < detectors.size(); i++)
        {
          detectors[i]->identify();
          detectors[i]->getDetections(points[i], detections[i], indices[i]);

          if (indices[i].size() < opts->target_mindets)
            {
              points[i].clear();
              detections[i].clear();
              indices[i].clear();
            }            
        }

      updateTracks(points, detections, indices);
      return;
    }

  // If fisheye and not CALIBDEV_4
  if (CameraModels::isFishEye(opts_cmodel_type))
    {
      // First, identify using the full rational model
      cmodel_type = CameraModels::OPENCV_RATIONAL;
      std::fill(cmodel_mask.begin(), cmodel_mask.end(), 1);

      throw std::runtime_error("ElMarkTargetDetector not supported");
      // for (int i = 0; i < detectors.size(); i++)
      //   ((ElMarkTargetDetector *) detectors[i])->setCameraParameters(cmodel_mask, cmodel_type);

      // Identify targets
      for (int i = 0; i < detectors.size(); i++)
        {
          detectors[i]->identify();
          detectors[i]->getDetections(points[i], detections[i], indices[i]);
          
          if (indices[i].size() < opts->target_mindets)
            {
	
              points[i].clear();
              detections[i].clear();
              indices[i].clear();
            }            
        }

      updateTracks(points, detections, indices);
  
      // Recover and bundle the scene assuming the full rational model 
      if (opts->ocvcalib)
        recoverCamerasOpenCV();
      else
        recoverCamerasBouguet();

      bundleAdjustCameras(cmodel_mask, cmodel_type);
      bundleAdjustPoses();

      // Update camera model to fisheye
      if (opts_cmodel_type == CameraModels::FISHEYE_P8P)
        updateRational2FisheyeP8PModel();
      else if (opts_cmodel_type == CameraModels::FISHEYE_E7P)
        updateRational2FisheyeE7PModel();

      bundleAdjustPoses();
	
      // Reidentify targets with the correct camera model, but without optimization
      cmodel_type = opts_cmodel_type;
      std::fill(cmodel_mask.begin(), cmodel_mask.end(), 0);
      ((OpenCVCamera *) cameras[0])->getIntrinsics(K, dist);
      
      // Set camera calibration
      throw std::runtime_error("ElMarkTargetDetector not supported");
      // for (int i = 0; i < detectors.size(); i++)
      //   ((ElMarkTargetDetector *) detectors[i])->setCameraParameters(K, dist, cmodel_mask, cmodel_type);

      for (int i = 0; i < detectors.size(); i++)
        {
          detectors[i]->identify();
          detectors[i]->getDetections(points[i], detections[i], indices[i]);
          
          if (indices[i].size() < opts->target_mindets)// || cameras[i]->getPose()->isZero())
            {
              points[i].clear();
              detections[i].clear();
              indices[i].clear();
            }
        }

      updateTracks(points, detections, indices);
      
      recoverPoses();
      bundleAdjustCameras();
      
      // Reidentify targets with the correct camera model, but without optimization
      cmodel_type = opts_cmodel_type;
      std::fill(cmodel_mask.begin(), cmodel_mask.end(), 0);
      ((OpenCVCamera *) cameras[0])->getIntrinsics(K, dist);
      
     // std::cout << __LINE__ << " : " << dist.transpose() << std::endl;

      // Set camera calibration
      throw std::runtime_error("ElMarkTargetDetector not supported");
      // for (int i = 0; i < detectors.size(); i++)
      //   ((ElMarkTargetDetector *) detectors[i])->setCameraParameters(K, dist, cmodel_mask, cmodel_type);

      for (int i = 0; i < detectors.size(); i++)
        {
          detectors[i]->identify();
          detectors[i]->getDetections(points[i], detections[i], indices[i]);
          
          if (indices[i].size() < opts->target_mindets)// || cameras[i]->getPose()->isZero())
            {
              points[i].clear();
              detections[i].clear();
              indices[i].clear();
            }
        }

      updateTracks(points, detections, indices);      
      
      recoverPoses();
    }
}

void Scene::detectTargets(void)
{
  vector<TargetDetector *> detectors;

  LOG_INFO << "Running target detections";

  // Let's do some inputs check
  if (! ((ct_type == TargetDetector::CHESSBOARD) || (ct_type == TargetDetector::CIRCLES_GRID) ||
      (ct_type == TargetDetector::ASYMMETRIC_CIRCLES_GRID) || (ct_type == TargetDetector::ELMARK_CIRCLES_GRID) ||
      (ct_type == TargetDetector::ELMARK_CHESSBOARD_GRID) || (ct_type == TargetDetector::ELMARK_CIRCLES_CORNERS) ))
    LOG_FATAL << "Unknown target type: " << ct_type;

  if (((ct_type == TargetDetector::CHESSBOARD) || (ct_type == TargetDetector::CIRCLES_GRID) ||
      (ct_type == TargetDetector::ASYMMETRIC_CIRCLES_GRID) || (ct_type == TargetDetector::ELMARK_CIRCLES_CORNERS)))
      {
        if (ct_width <= 0)
          LOG_FATAL << "Target width must be a positive integer";
        if (ct_height <= 0)
          LOG_FATAL << "Target height must be a positive integer";
      }

  for (int i = 0; i < no_cameras; i++)
    {
      if (!bfs::is_regular_file(cameras[i]->getImagePath()) && !bfs::is_symlink(cameras[i]->getImagePath()))
        LOG_FATAL << "Not a file: " << cameras[i]->getImagePath();
    }

  // Initialize detectors
  initTargetDetectors(detectors);
  // Run detectors
  runTargetDetectors(detectors);
  // Identify target detections and update scene tracks accordingly
  identifyTargetDetections(detectors);
  // Delete detectors
  deleteTargetDetectors(detectors);
}

#ifndef RBA_NO_OPENCV

void Scene::recoverCamerasOpenCV(void)
{
  unsigned int i;
  vector<int> detcams;
  OpenCVCamera *camera;
  ETrans *ext_trans;
  Mat cv_K;
  Mat cv_dist;
  int calib_flags;
  list<Track*>::iterator iter;
  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > image_points;
  vector<Mat> rvecs;
  vector<Mat> tvecs;

  LOG_INFO << "Running internal camera calibration using OpenCV";

  if (cameras.size() == 0)
    LOG_FATAL << "There are no cameras to be calibrated";

  camera = (OpenCVCamera *) cameras[0];
  camera->getIntrinsics(cv_K, cv_dist);

  for (i = 0; i < cameras.size(); i++)
    {
       camera = (OpenCVCamera *) cameras[i];
       list<Track*> tracks = camera->getTracks();
       if (tracks.size() > 0 && (!camera->getXValidFlag()))
         detcams.push_back(i);
    }

  if (detcams.size() == 0)
    LOG_FATAL << "Not enough cameras with detected points";

  object_points.resize(detcams.size());
  image_points.resize(detcams.size());

  for (i = 0; i < detcams.size(); i++)
    {
       camera = (OpenCVCamera *) cameras[detcams[i]];
       list<Track*> tracks = camera->getTracks();

       object_points[i].reserve(tracks.size());
       image_points[i].reserve(tracks.size());

       for (iter = tracks.begin(); iter != tracks.end(); iter++)
         {
           Vector4f point3d = (*iter)->getPoint(Track::SOURCEPT);
           object_points[i].push_back(Point3f(point3d(0), point3d(1), point3d(2)));

           Vector2f point2d = (*iter)->getDetection(camera->getIndex(), Track::SOURCE);
           image_points[i].push_back(Point2f(point2d(0), point2d(1)));
         }
    }

  calib_flags = cv::CALIB_RATIONAL_MODEL;
  if (!opts->cmodel_mask[0]) calib_flags |= cv::CALIB_FIX_K1;
  if (!opts->cmodel_mask[1]) calib_flags |= cv::CALIB_FIX_K2;
  if (!opts->cmodel_mask[2]) calib_flags |= cv::CALIB_FIX_K3;
  if (!opts->cmodel_mask[3]) calib_flags |= cv::CALIB_FIX_K4;
  if (!opts->cmodel_mask[4]) calib_flags |= cv::CALIB_FIX_K5;
  if (!opts->cmodel_mask[5]) calib_flags |= cv::CALIB_FIX_K6;
  if (!opts->cmodel_mask[6]) calib_flags |= cv::CALIB_ZERO_TANGENT_DIST;
  if (!opts->cmodel_mask[7]) calib_flags |= cv::CALIB_FIX_ASPECT_RATIO;

  if (opts->ocvcalib_linear)
    {
	  LOG_INFO << "Estimating only linear parameters in cv.calibrateCamera";
	  calib_flags |= cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 |
	  	  	  	  	cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6 | cv::CALIB_ZERO_TANGENT_DIST;
    }

  LOG_INFO << "Running cv.calibrateCamera";

  calibrateCamera(object_points, image_points, Size(camera->getImageWidth(), camera->getImageHeight()),
      cv_K, cv_dist, rvecs, tvecs, calib_flags);

  LOG_INFO << "Setting internal camera calibration";

  for (i = 0; i < cameras.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[i];
      camera->setIntrinsics(cv_K, cv_dist);
    }
    
/*    
  Vector8f dist;
  Matrix3f K;
  camera->getIntrinsics(K, dist);
  cout << "K = " << endl << K << endl << endl;   
*/

  for (i = 0; i < detcams.size(); i++)
    {
      camera = (OpenCVCamera *) cameras[detcams[i]];

      ext_trans = camera->getPose();
      if (ext_trans->isZero())
        {
          LOG_INFO << "Setting pose for camera " << detcams[i];
          camera->setPose(ETrans(rvecs[i].ptr<double>(0), tvecs[i].ptr<double>(0)));
        }
    }
}

#else

void Scene::recoverCamerasOpenCV(void)
{
  LOG_FATAL << "Recompile with OpenCV support for OpenCV camera calibration procedure";
}
#endif


void Scene::recoverPoses(void)
{
  Camera *camera;

  // Based on data already loaded, recover missing calibration information
  LOG_INFO << "Recovering missing camera poses";

  for (int i = 0; i < no_cameras; i++)
    {
      camera = cameras[i];
      if (camera->getPose()->isZero())
        {
          LOG_INFO << "Camera id " << camera->getIndex() <<
              " pose unknown, running camera resection first";

#ifndef RBA_NO_OPENCV
          camera->recoverPose(Camera::CV_EPNP);
#else          
          camera->recoverPosePlanar();
#endif
          if ((camera->getCameraType() == Camera::OPENCV))
            ((OpenCVCamera *)camera)->bundleAdjustPose();
        }
    }
}

void Scene::bundleAdjustPoses(void)
{
  Camera *camera;

  // Based on data already loaded, recover missing calibration information
  LOG_INFO << "Running BA on camera poses";

  for (int i = 0; i < no_cameras; i++)
    {
      camera = cameras[i];
      if (!camera->getPose()->isZero() && opts->ba_cposes)
        {
          if ((camera->getCameraType() == Camera::OPENCV))
            {
              ((OpenCVCamera *)camera)->bundleAdjustPose();
            }
        }
    }
}

void Scene::selectXValidationCameras(void)
{
  LOG_INFO << "Selecting cameras for cross-validation";
  for (int i = 0; i < (int) opts->xvalid_cameras.size(); i++)
    {
      int idx = opts->xvalid_cameras[i];
      if ((idx >= 0) && (idx < no_cameras))
        {
          LOG_INFO << "Camera no. " << idx << " selected for cross-validation";
          cameras[idx]->setXvalidFlag(true);
        }
      else
        {
          LOG_WARN << "Camera XValidation index out of bounds: " << idx;
        }
    }
}

void Scene::updateResidualErrors(void)
{
  for (int i = 0; i < no_cameras; i++)
    cameras[i]->updateResidualErrors();
}

void Scene::recoverScene(void)
{
  // Based on data already loaded, recover missing calibration information
  LOG_INFO << "Recovering missing scene information";

  if (no_cameras == 0)
    {
      LOG_WARN << "No cameras loaded, no scene to recover";
      return;
    }

  selectXValidationCameras();

  if (scene_type == CALIBDEV_4)
    {
      // We have a calibrated camera and unknown detections, so we just detect targets and
      // recover poses.
      detectTargets();

      for (int i = 0; i < (int) cameras.size(); i++)
          cameras[i]->updateDetections();

      // Recover missing camera poses.
      recoverPoses();
      bundleAdjustPoses();
    }

  if ((scene_type == CALIBDEV_3) && CameraModels::isFishEye(opts->cmodel_type))
    {
      detectTargets();
  
      for (int i = 0; i < (int) cameras.size(); i++)
        cameras[i]->updateDetections();
      
      // Recover missing camera poses.
     // recoverPoses();
      bundleAdjustCameras();

   //   for (int i = 0; i < (int) cameras.size(); i++)
   //     cameras[i]->updateDetections();
      
    }
  else if ((scene_type == CALIBDEV_3) || (scene_type == CALIBDEV_2))
    {
      if (scene_type == CALIBDEV_3)
        detectTargets();

      // Calibrate the scene assuming all images were taken
      // by the same camera using the same settings.
      if (opts->ocvcalib)
        recoverCamerasOpenCV();
      else //if (!((opts->ba_scene == 1) && (ct_type == TargetDetector::ELMARK_CIRCLES_GRID)))
        //recoverCamerasZhang();
        recoverCamerasBouguet();
      
      if (opts->ba_scene)
        bundleAdjustCameras();

      for (int i = 0; i < (int) cameras.size(); i++)
          cameras[i]->updateDetections();

      // Recover missing camera poses.
      recoverPoses();
      bundleAdjustPoses();
    }

  if (scene_type == CALIBDEV_1)
    {
      for (int i = 0; i < (int) cameras.size(); i++)
          cameras[i]->updateDetections();

      recoverPoses();

      if (opts->ba_scene)
        bundleAdjustCameras();

      bundleAdjustPoses();
    }

  // Remove cameras with less that target_mindets identified detections
  for (int i = 0; i < (int) cameras.size(); i++)
    {
       OpenCVCamera *camera = (OpenCVCamera *) cameras[i];
       if (camera->getNumTracks() < opts->target_mindets)
         camera->setPose(ETrans(Matrix4f::Zero()));
    }

  no_calib_cameras = 0;;
  for (int i = 0; i < (int) cameras.size(); i++)
      if (cameras[i]->isCalibCamera())
        no_calib_cameras++;

  updateResidualErrors();
}

