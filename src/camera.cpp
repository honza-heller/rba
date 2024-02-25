/*
 * camera.cpp
 *
 *  Created on: Oct 16, 2012
 *      Author: jheller
 */


#include "definitions.h"
#include "camera.h"
#include <cfloat>

#ifndef RBA_NO_OPENCV
#include <opencv2/opencv.hpp>
using namespace cv;
#endif

void Camera::recoverPose(PoseMethod method)
{
  if (method == CV_EPNP)
    {
#ifndef RBA_NO_OPENCV
      recoverPoseCVEPNP();
#else
      LOG_WARN << "RBA compiled without OpenCV support. Using fallback PNP method.";
      recoverPosePPNP();
#endif
    }
  else if (method == PPNP)
    recoverPosePPNP();
  else
    LOG_FATAL << "Unknown PNP method";
}

void Camera::recoverPosePlanar()
{
  LOG_INFO << "Recovering Pose from planar target";

  Matrix3f H, R;
  Vector3f t;

  if (tracks.size() == 0)
    {
      LOG_WARN << "Cannot recover pose for camera " << getIndex() << ": there are no points connected with this camera";
      return;
    }

  recoverHomography(H, true);
  
//  std::cout << "Camera::recoverPosePlanar" << std::endl;
//  std::cout << " H = \n" << H << std::endl << std::endl;
  decomposeH(H, Matrix3f::Identity(), R, t);

  // Make the scene lie in front of the camera
  //if (t(2) < 0.0)
  //  decomposeH(-H, Matrix3f::Identity(), R, t);
  Vector3f C = - R.transpose() * t;
    if (C(2) < 0.0)
      {
        R.block<3, 1>(0, 0) *= -1;
        R.block<3, 1>(0, 1) *= -1;
        t = -t;
      }  
    
  cpose.setMatrix(R, t);
}

#ifndef RBA_NO_OPENCV
void Camera::recoverHomographyOcv(Matrix3f &H) const
{
    std::vector<cv::Point2f> us;
    std::vector<cv::Point2f> Xs;
    
    list<Track *>::const_iterator iter;
  for (iter = tracks.begin(); iter != tracks.end(); iter++)
    {
      Vector2f u = ((*iter)->getDetection(getIndex(), Track::SOURCE));
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT);
      
      us.push_back(cv::Point2f(u(0), u(1)));
      Xs.push_back(cv::Point2f(X(0), X(1)));
    }
    cv::Mat mH = cv::findHomography(Xs, us);
   H << mH.at<double>(0, 0), mH.at<double>(0, 1), mH.at<double>(0, 2),
        mH.at<double>(1, 0), mH.at<double>(1, 1), mH.at<double>(1, 2),
        mH.at<double>(2, 0), mH.at<double>(2, 1), mH.at<double>(2, 2);
}

#endif

void Camera::recoverHomography(Matrix3f &H, bool use_rays) const
{
  LOG_INFO << "Recovering homography";

  double xscale = -1.0f, yscale = -1.0f, zscale = -1.0f;
  double tscale = -1.0f, iscale = -1.0f;
  list<Track *>::const_iterator iter;
  Matrix3f Ti, Tt, iTi, iTt;
  int j;

  // Find maximal coordinate of the calibration target
  for (iter = tracks.begin(); iter != tracks.end(); iter++)
    {
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT).cwiseAbs();
      if (double(X(0)) > xscale)
        xscale = double(X(0));
      if (double(X(1)) > yscale)
        yscale = double(X(1));
      if (double(X(2)) > zscale)
        zscale = double(X(2));
    }

  if (!iszero(zscale))
    LOG_FATAL << "recoverHomography method works for planar scenes only";

  tscale = std::max(xscale, yscale);

  Tt << tscale,      0,  xscale / 2.0f,
             0, tscale,  yscale / 2.0f,
             0,      0,           1.0f;

  iTt = Tt.inverse();

  if (use_rays)
    {
      Ti = Matrix3f::Identity();
      iTi = Matrix3f::Identity();
    }
  else
    {
      iscale = std::max(getImageWidth(), getImageHeight());

      Ti << iscale,      0,  double(getImageWidth()) / 2.0f,
                 0, iscale, double(getImageHeight()) / 2.0f,
                 0,      0,                            1.0f;

      iTi = Ti.inverse();
    }

  MatrixXf A(tracks.size() * 2, 9);
  ceres::Problem problem;
  ceres::CostFunction *cost_function;
  ceres::Solver::Summary summary;
  ceres::Solver::Options coptions;

  // Set Ceres optimization parameters
  coptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  coptions.num_threads = 1;
  //coptions.num_linear_solver_threads = 1;
  coptions.use_nonmonotonic_steps = true;
  coptions.minimizer_progress_to_stdout = false;

  for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++)
    {
      Vector2f u;
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT);

      if (use_rays)
        {
          Vector4f dir = getRay(*iter).getDirection();
          u(0) = dir(0) / dir(2);
          u(1) = dir(1) / dir(2);
        }
      else
        u = ((*iter)->getDetection(getIndex(), Track::SOURCE));

      Vector3f Xm = iTt * Vector3f(X(0), X(1), 1.0f);
      Vector3f um = iTi * Vector3f(u(0), u(1), 1.0f);

      A.block<1,3>(j,0) =  um(2) * Xm;
      A.block<1,3>(j,3) = Vector3f::Zero().transpose();
      A.block<1,3>(j,6) = -um(0) * Xm;
      j++;

      A.block<1,3>(j,0) = Vector3f::Zero().transpose();
      A.block<1,3>(j,3) = -um(2) * Xm;
      A.block<1,3>(j,6) =  um(1) * Xm;
      j++;

      cost_function = new ceres::AutoDiffCostFunction<HomographyError, 2, 9>(
                        new HomographyError(Xm, um));
      problem.AddResidualBlock(cost_function, NULL, H.data());
    }

  Eigen::JacobiSVD<MatrixXf> svdA(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const MatrixXf &V = svdA.matrixV();

  /*
  H << V(0, 8), V(3, 8), V(6, 8),
       V(1, 8), V(4, 8), V(7, 8),
       V(2, 8), V(5, 8), V(8, 8);
  */

  H << V(0, 8), V(1, 8), V(2, 8),
       V(3, 8), V(4, 8), V(5, 8),
       V(6, 8), V(7, 8), V(8, 8);

 /*
 double cost = 0.0;
 problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
 std::cout << "cost(+H) = " << cost << std::endl;

H = -H;

 problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, NULL, NULL, NULL);
 std::cout << "cost(-H) = " << cost << std::endl;
 */

// std::cout << "Hl = " << std::endl;
       
//  std::cout << "Homo linear" << std::endl;
//  std::cout << H << std::endl;

  Solve(coptions, &problem, &summary); // This is not necessary

//  std::cout << "Hnl = " << std::endl;
//  std::cout << "Homo ceres" << std::endl;
//  std::cout << H << std::endl;
  
//  std::cout << "Ti" << std::endl;
//  std::cout << Ti << std::endl;

//  std::cout << "iTt" << std::endl;
//  std::cout << iTt << std::endl;
  
  //  std::cout << summary.FullReport()  << std::endl;

 // H = Ti * H.transpose() * iTt;
  H = Ti * H * iTt;
 // H /= H(2,2);
/*
  Matrix3f H2 = Ti * H.transpose() * iTt;
  H2 /= H2(2,2);

  for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++)
    {
      Vector2f u;
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT);
        u = ((*iter)->getDetection(getIndex(), Track::SOURCE));

        X(2) = 1.0f;
        Vector3f v = H * X.head<3>();
        v /= v(2);

        std::cout << u.transpose() << ", " << X.transpose() << " -> " << (u-v.head<2>()).norm() << "    " << v.transpose() << std::endl;


        v = H2 * X.head<3>();
        v /= v(2);

        std::cout << u.transpose() << ", " << X.transpose() << " -> " << (u-v.head<2>()).norm() << "    " << v.transpose() << std::endl;
        std::cout << std::endl;
    }
*/
}

void Camera::recoverRadialHomography(Matrix3f &H, double &k, bool use_rays) const
{
  LOG_INFO << "Recovering radial homography";

  double xscale = -1.0f, yscale = -1.0f, zscale = -1.0f;
  double tscale = -1.0f, iscale = -1.0f;
  list<Track *>::const_iterator iter;
  Matrix3f Ti, Tt, iTi, iTt;
  int j;

  // Find maximal coordinate of the calibration target
  for (iter = tracks.begin(); iter != tracks.end(); iter++)
    {
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT).cwiseAbs();
      if (double(X(0)) > xscale)
        xscale = double(X(0));
      if (double(X(1)) > yscale)
        yscale = double(X(1));
      if (double(X(2)) > zscale)
        zscale = double(X(2));
    }

  if (!iszero(zscale))
    LOG_FATAL << "recoverHomography method works for planar scenes only";

  tscale = std::max(xscale, yscale);

  Tt << tscale,      0,  xscale / 2.0f,
             0, tscale,  yscale / 2.0f,
             0,      0,           1.0f;

  iTt = Tt.inverse();

  if (use_rays)
    {
      Ti = Matrix3f::Identity();
      iTi = Matrix3f::Identity();
    }
  else
    {
      iscale = std::max(getImageWidth(), getImageHeight());

      Ti << iscale,      0,  double(getImageWidth()) / 2.0f,
                 0, iscale, double(getImageHeight()) / 2.0f,
                 0,      0,                            1.0f;

      iTi = Ti.inverse();
    }

  ceres::Problem problem;
  ceres::CostFunction *cost_function;
  ceres::Solver::Summary summary;
  ceres::Solver::Options coptions;

  // Set Ceres optimization parameters
  coptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  coptions.num_threads = 1;
  //coptions.num_linear_solver_threads = 1;
  coptions.use_nonmonotonic_steps = true;
  coptions.minimizer_progress_to_stdout = false;

  // Recover h1, h2, h4, h5, h7, h8
  MatrixXf A(tracks.size(), 6);
  for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++)
    {
      Vector2f u;
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT);

      if (use_rays)
        {
          Vector4f dir = getRay(*iter).getDirection();
          u(0) = dir(0) / dir(2);
          u(1) = dir(1) / dir(2);
        }
      else
        u = ((*iter)->getDetection(getIndex(), Track::SOURCE));

      Vector3f Xm = iTt * Vector3f(X(0), X(1), 1.0f);
      Vector3f um = iTi * Vector3f(u(0), u(1), 1.0f);

      A.block<1, 6>(j++, 0) << -um(1) * Xm(0), um(0) * Xm(0), -um(1) * Xm(1), um(0) * Xm(1), -um(1), um(0);

      cost_function = new ceres::AutoDiffCostFunction<RadialHomographyError, 2, 9, 1>(
                        new RadialHomographyError(Xm, um));
      problem.AddResidualBlock(cost_function, NULL, H.data(), &k);
    }

  Eigen::JacobiSVD<MatrixXf, Eigen::FullPivHouseholderQRPreconditioner> svdA(A, Eigen::ComputeThinV);
  const MatrixXf &V = svdA.matrixV();
  double h1 = V(0, 5);
  double h2 = V(1, 5);
  double h4 = V(2, 5);
  double h5 = V(3, 5);
  double h7 = V(4, 5);
  double h8 = V(5, 5);

  // Recover h3, h6, h9, k
  A.resize(tracks.size() * 2, 5);
  for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++)
    {
      Vector2f u;
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT);

      if (use_rays)
        {
          Vector4f dir = getRay(*iter).getDirection();
          u(0) = dir(0) / dir(2);
          u(1) = dir(1) / dir(2);
        }
      else
        u = ((*iter)->getDetection(getIndex(), Track::SOURCE));

      Vector3f Xm = iTt * Vector3f(X(0), X(1), 1.0f);
      Vector3f um = iTi * Vector3f(u(0), u(1), 1.0f);
      double r = um(0) * um(0) + um(1) * um(1);

      A.block<1, 5>(j++, 0) <<  um(1)*Xm(0),  um(1)*Xm(1),  um(1), -h8*r - h2*r*Xm(0) - h5*r*Xm(1), -h8 - h2*Xm(0) - h5*Xm(1);
      A.block<1, 5>(j++, 0) << -um(0)*Xm(0), -um(0)*Xm(1), -um(0),  h7*r + h1*r*Xm(0) + h4*r*Xm(1),  h7 + h1*Xm(0) + h4*Xm(1);
    }

  VectorXf x = A.block(0, 0, tracks.size() * 2, 4).colPivHouseholderQr().solve(-A.block(0, 4, tracks.size() * 2, 1));

  H <<   h1,   h4,   h7,
         h2,   h5,   h8,
       x(0), x(1), x(2);
  H /= H(2,2);
  k = x(3);

 // Solve(coptions, &problem, &summary);

  H = Ti * H * iTt;
  H /= H(2,2);
}


void Camera::recoverPoseKrPlanar(Matrix3f &R, Vector3f &t, Matrix3f &K, double &k) const
{

  LOG_INFO << "Recovering pose with unknown focal length and radial distortion";

  double xscale = -1.0f, yscale = -1.0f, zscale = -1.0f;
  double tscale = -1.0f, iscale = -1.0f;
  list<Track *>::const_iterator iter;
  int j;

  // Find maximal coordinate of the calibration target
  for (iter = tracks.begin(); iter != tracks.end(); iter++)
    {
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT);
      if (std::abs((double)X(0)) > xscale)
        xscale = X(0);
      if (std::abs((double)X(1)) > yscale)
        yscale = X(1);
      if (std::abs((double)X(2)) > zscale)
        zscale = X(2);
    }

  if (!iszero(zscale))
    LOG_FATAL << "recoverPoseKrPlanar method works for planar scenes only";

  tscale = std::max(xscale, yscale);
  iscale = std::max(getImageWidth(), getImageHeight());

  // Initial optical axis/ radial distortion center
  double cx = (getImageWidth() - 1.0f) / 2.0f;
  double cy = (getImageHeight() - 1.0f) / 2.0f;

  // Recover P matrix
  MatrixXf A(tracks.size(), 6);
  MatrixXf B(tracks.size() * 2, 4);
  VectorXf b(tracks.size() * 2);

  double p_11, p_12, p_13, p_14, p_21, p_22, p_23, p_24, p_31, p_32, p_33, p_34;

  for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++, j++)
    {
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT) / tscale;
      Vector2f u = ((*iter)->getDetection(getIndex(), Track::SOURCE) - Vector2f(cx, cy)) / iscale;

      A(j, 0) = -X(0) * u(1);
      A(j, 1) = -X(1) * u(1);
      A(j, 2) = -u(1);
      A(j, 3) =  X(0) * u(0);
      A(j, 4) =  X(1) * u(0);
      A(j, 5) =  u(0);
    }

  Eigen::JacobiSVD<MatrixXf> svdA(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  p_11 = svdA.matrixV()(0, 5);
  p_12 = svdA.matrixV()(1, 5);
  p_14 = svdA.matrixV()(2, 5);
  p_21 = svdA.matrixV()(3, 5);
  p_22 = svdA.matrixV()(4, 5);
  p_24 = svdA.matrixV()(5, 5);

  for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++)
    {
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT) / tscale;
      Vector2f u = ((*iter)->getDetection(getIndex(), Track::SOURCE) - Vector2f(cx, cy)) / iscale;
      double r = u(0) * u(0) + u(1) * u(1);

      B(j, 0) = -p_24*r - p_21*X(0)*r - p_22*X(1)*r;
      B(j, 1) = X(0) * u(1);
      B(j, 2) = X(1) * u(1);
      B(j, 3) = u(1);
      b(j) = p_24 + X(0)*p_21 + X(1)*p_22;
      j++;

      B(j, 0) = p_14*r + p_11*X(0)*r + p_12*X(1)*r;
      B(j, 1) = -X(0) * u(0);
      B(j, 2) = -X(1) * u(0);
      B(j, 3) = -u(0);
      b(j) = -p_14 - X(0)*p_11 - X(1)*p_12;
      j++;
    }

  Vector4f bs = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  k    = bs(0);
  p_31 = bs(1);
  p_32 = bs(2);
  p_34 = bs(3);

  double c1 = - (p_11*p_21 + p_12*p_22);
  double c2 = - (p_11*p_11 + p_12*p_12 - p_22*p_22 - p_21*p_21);

  MatrixXf Compan(4, 4);
  Compan << 0, -c2, 0, c1*c1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
  Eigen::VectorXcd eivals = Compan.eigenvalues();
  VectorXf reivals = eivals.real();
  VectorXf ieivals = eivals.imag();

  for (int k = 0; k < eivals.rows(); k++)
    {
      MatrixXf P(3, 4), Ps(3, 4);
      Vector3f C;
      Matrix3f Rt, Kt;

      if (!iszero((double)ieivals(k)))
        continue;

      p_23 = reivals(k);
      p_13 = c1 / p_23;

      MatrixXf A(2, 1);
      Vector2f b;

      A << p_13, p_23;
      b << - p_11*p_31 - p_12*p_32, - p_21*p_31 - p_22*p_32;

      VectorXf bs = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
      p_33 = bs(0);

      Ps << p_11, p_12, p_13, p_14, p_21, p_22, p_23, p_24, p_31, p_32, p_33, p_34;
      Kt << iscale, 0, cx, 0, iscale, cy, 0, 0, 1;
      //P = Vector3f(iscale, iscale, 1.0f).asDiagonal() * Ps * Vector4f(1.0f/tscale, 1.0f/tscale, 1.0f/tscale, 1.0f).asDiagonal();
      P = Kt * Ps * Vector4f(1.0f/tscale, 1.0f/tscale, 1.0f/tscale, 1.0f).asDiagonal();

      Camera::decomposeP(P, Kt, Rt, C);

      if (Kt(2, 2) < 0)
        continue;

      R = Rt;
      K = Kt;
      t = -R * C;

      // Detect cameras "behind" the calibration target based on the chirality condition
      // This means that only cameras up to 180 FOV are supported
      for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++, j++)
        {
          Vector3f X = R * (*iter)->getPoint(Track::SOURCEPT).block(0,0,3,1) + t;
          if (X(2) < 0.0f)
          {
            // Mirror camera along XY plane
            R.block(0,0,3,2) = -R.block(0,0,3,2);
            t = -t;
            break;
          }
        }
    }
}

void Camera::recoverPoseKPlanar(Matrix3f &R, Vector3f &t, Matrix3f &K) const
{

  LOG_INFO << "Recovering pose with unknown focal length and radial distortion";

  double xscale = -1.0f, yscale = -1.0f, zscale = -1.0f;
  double tscale = -1.0f, iscale = -1.0f;
  list<Track *>::const_iterator iter;
  int j;

  // Find maximal coordinate of the calibration target
  for (iter = tracks.begin(); iter != tracks.end(); iter++)
    {
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT);
      if (std::abs((double)X(0)) > xscale)
        xscale = X(0);
      if (std::abs((double)X(1)) > yscale)
        yscale = X(1);
      if (std::abs((double)X(2)) > zscale)
        zscale = X(2);
    }

  if (!iszero(zscale))
    LOG_FATAL << "recoverPoseKrPlanar method works for planar scenes only";

  tscale = std::max(xscale, yscale);
  iscale = std::max(getImageWidth(), getImageHeight());

  // Initial optical axis/ radial distortion center
  double cx = (getImageWidth() - 1.0f) / 2.0f;
  double cy = (getImageHeight() - 1.0f) / 2.0f;

  // Recover P matrix
  MatrixXf A(tracks.size(), 6);
  MatrixXf B(tracks.size() * 2, 3);
  VectorXf b(tracks.size() * 2);

  double p_11, p_12, p_13, p_14, p_21, p_22, p_23, p_24, p_31, p_32, p_33, p_34;

  for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++, j++)
    {
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT) / tscale;
      Vector2f u = ((*iter)->getDetection(getIndex(), Track::SOURCE) - Vector2f(cx, cy)) / iscale;

      A(j, 0) = -X(0) * u(1);
      A(j, 1) = -X(1) * u(1);
      A(j, 2) = -u(1);
      A(j, 3) =  X(0) * u(0);
      A(j, 4) =  X(1) * u(0);
      A(j, 5) =  u(0);
    }

  Eigen::JacobiSVD<MatrixXf> svdA(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  p_11 = svdA.matrixV()(0, 5);
  p_12 = svdA.matrixV()(1, 5);
  p_14 = svdA.matrixV()(2, 5);
  p_21 = svdA.matrixV()(3, 5);
  p_22 = svdA.matrixV()(4, 5);
  p_24 = svdA.matrixV()(5, 5);

  for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++)
    {
      Vector4f X = (*iter)->getPoint(Track::SOURCEPT) / tscale;
      Vector2f u = ((*iter)->getDetection(getIndex(), Track::SOURCE) - Vector2f(cx, cy)) / iscale;

      B(j, 0) = X(0) * u(1);
      B(j, 1) = X(1) * u(1);
      B(j, 2) = u(1);
      b(j) = p_24 + X(0)*p_21 + X(1)*p_22;
      j++;

      B(j, 0) = -X(0) * u(0);
      B(j, 1) = -X(1) * u(0);
      B(j, 2) = -u(0);
      b(j) = -p_14 - X(0)*p_11 - X(1)*p_12;
      j++;
    }

  Vector4f bs = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  p_31 = bs(0);
  p_32 = bs(1);
  p_34 = bs(2);

  double c1 = - (p_11*p_21 + p_12*p_22);
  double c2 = - (p_11*p_11 + p_12*p_12 - p_22*p_22 - p_21*p_21);

  MatrixXf Compan(4, 4);
  Compan << 0, -c2, 0, c1*c1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
  Eigen::VectorXcd eivals = Compan.eigenvalues();
  VectorXf reivals = eivals.real();
  VectorXf ieivals = eivals.imag();

  for (int k = 0; k < eivals.rows(); k++)
    {
      MatrixXf P(3, 4), Ps(3, 4);
      Vector3f C;
      Matrix3f Rt, Kt;

      if (!iszero((double)ieivals(k)))
        continue;

      p_23 = reivals(k);
      p_13 = c1 / p_23;

      MatrixXf A(2, 1);
      Vector2f b;

      A << p_13, p_23;
      b << - p_11*p_31 - p_12*p_32, - p_21*p_31 - p_22*p_32;

      VectorXf bs = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
      p_33 = bs(0);

      Ps << p_11, p_12, p_13, p_14, p_21, p_22, p_23, p_24, p_31, p_32, p_33, p_34;
      P = Vector3f(iscale, iscale, 1.0f).asDiagonal() * Ps * Vector4f(1.0f/tscale, 1.0f/tscale, 1.0f/tscale, 1.0f).asDiagonal();

      Camera::decomposeP(P, Kt, Rt, C);

      if (Kt(2, 2) < 0)
        continue;

      R = Rt;
      K = Kt;
      t = -R * C;

      // Detect cameras "behind" the calibration target based on the chirality condition
      // This means that only cameras up to 180 FOV are supported
      for (iter = tracks.begin(), j = 0; iter != tracks.end(); iter++, j++)
        {
          Vector3f X = R * (*iter)->getPoint(Track::SOURCEPT).block(0,0,3,1) + t;
          if (X(2) < 0.0f)
          {
            // Mirror camera along XY plane
            R.block(0,0,3,2) = -R.block(0,0,3,2);
            t = -t;
            break;
          }
        }
    }
}

#ifndef RBA_NO_OPENCV

void Camera::recoverPoseCVEPNP()
{
  list<Track*>::iterator iter;
  //int no_points = points.size();

  Mat K = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  Mat dist = Mat(0,0, CV_64F);
  Mat rvec, tvec;

  Vector4f point3d;
  vector<Point3f> cv_points3d;
  vector<Point2f> cv_points2d;

  LOG_INFO << "Recovering camera pose using CV_EPNP method for camera id " << getIndex();

  if (tracks.size() == 0)
    LOG_WARN << "Cannot recover pose for camera " << getIndex() << ": there are no points connected with this camera";
  else
    {
      for (iter = tracks.begin(); iter != tracks.end(); iter++)
        {
          point3d = (*iter)->getPoint(Track::SOURCEPT);
          cv_points3d.push_back(Point3f(point3d(0), point3d(1), point3d(2)));
          point3d = getRay(*iter).getDirection();
          cv_points2d.push_back(Point2f(point3d(0) / point3d(2), point3d(1) / point3d(2)));
        }

      solvePnP(cv_points3d, cv_points2d, K, dist, rvec, tvec, false, CV_EPNP);
      cpose.setMatrix((double *) rvec.data, (double *) tvec.data);
    }
}
#endif


Eigen::MatrixXd Camera::fliptrans(Eigen::MatrixXd X)
{
  Eigen::MatrixXd Y(X.cols(), X.rows());
  for (int j = 0; j < X.cols(); j++)
    for (int i = 0; i < X.rows(); i++)
      Y(X.cols() - 1 - j, X.rows() - 1 - i) = X(i, j);
  return Y;
}

// https://ksimek.github.io/2012/08/14/decompose/
void Camera::rq(const Eigen::MatrixXd &S, Eigen::MatrixXd &U, Eigen::MatrixXd &Q)
{
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(fliptrans(S));
  Q  = fliptrans(qr.householderQ()*(Eigen::MatrixXd::Identity(S.rows(),S.cols())));
  U  = fliptrans(qr.matrixQR().block(0,0,S.cols(),S.cols()).triangularView<Eigen::Upper>());

  if (Q.determinant() < 0)
    {
      U.block(0,0,U.rows(),1) *= -1;
      Q.block(0,0,1,Q.cols()) *= -1;
    }
}

void Camera::decomposeH(const Matrix3f &H, const Matrix3f &iK, Matrix3f &R, Vector3f &t)
{
  double l = 1.0f / (iK * H.col(0)).norm();
  t = l * iK * H.col(2);

  R.col(0) = (l * iK * H.col(0));
  R.col(1) = (l * iK * H.col(1));
  R.col(2) = R.col(0).cross(R.col(1));

  R.col(0).normalize();
  R.col(1).normalize();
  R.col(2).normalize();

  // Make R a proper rotation matrix
  Eigen::JacobiSVD<MatrixXf> svdR(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const MatrixXf & Ur = svdR.matrixU();
  const MatrixXf & Vr = svdR.matrixV();

  R = Ur * Vector3f(1.0f, 1.0f, Ur.determinant() * Vr.determinant()).asDiagonal() * Vr.transpose();
}

void Camera::decomposeP(const MatrixXf &P, Matrix3f &K, Matrix3f &R, Vector3f &C)
{
  Eigen::MatrixXd Pm, Km, Rm;
  Pm = P.block(0,0,3,3);
  rq(Pm, Km, Rm);

  bool aflag = (iszero(double(Km(2,2)))) ? true : false;

  if (~aflag)
    Km /= std::abs(double(Km(2,2)));
  else
    Km /= std::abs(double(Km(1,1)));

  if (Km(0, 0) < 0)
    {
      Km = Km * Vector3f(-1, -1, 1).asDiagonal();
      Rm = Vector3f(-1, -1, 1).asDiagonal() * Rm;
    }
  if (Km(1, 1) < 0)
    {
      Km = Km * Vector3f(1, -1, -1).asDiagonal();
      Rm = Vector3f(1, -1, -1).asDiagonal() * Rm;
    }

  if (~aflag)
    {
      C = (-Pm).fullPivHouseholderQr().solve(P.block(0,3,3,1));
    }
  else
    {
      Vector2f C2 = (-Pm.block(0,0,2,3)).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Pm.block(0,3,2,1));
      C = Vector3f(C2(0), C2(1), 0);
    }

/*
  Eigen::JacobiSVD<MatrixXf> svdR(Rm, Eigen::ComputeThinU | Eigen::ComputeThinV);
  MatrixXf Ur = svdR.matrixU();
  MatrixXf Vr = svdR.matrixV();

  R = Ur * Vector3f(1.0f, 1.0f, Ur.determinant() * Vr.determinant()).asDiagonal() * Vr.transpose();
*/

  K = Km;
  R = Rm;
}

// Valeria Garro, Fabio Crosilla, Andrea Fusiello
// Solving the PnP Problem with Anisotropic Orthogonal Procrustes Analysis
// 2012 Second Joint 3DIM/3DPVT Conference: 3D Imaging, Modeling, Processing, Visualization & Transmission
void Camera::recoverPosePPNP()
{
  int i, nop = tracks.size();
  double err = DBL_MAX, tol = 0.000001, d;
  list<Track*>::iterator iter;
  Vector4f point3d;

  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  Eigen::MatrixXd P(nop, 3);
  Eigen::MatrixXd PR(nop, 3);
  Eigen::MatrixXd Y(nop, 3);
  Eigen::MatrixXd S(nop, 3);
  Eigen::MatrixXd Z(nop, nop);
  Eigen::MatrixXd PRY(nop, nop);
  Eigen::MatrixXd e(nop, 1);
  Eigen::MatrixXd pp(nop, 1);
  Eigen::MatrixXd c(3, 1);
  Eigen::MatrixXd A(nop, nop);
  Eigen::MatrixXd II(nop, 1);
  Eigen::MatrixXd E(nop, 3);
  Eigen::MatrixXd Eo(nop, 3);
  Eigen::MatrixXd R1(3, 3), R2(3, 3);
  Eigen::MatrixXd Vt(3, 3);
  Eigen::MatrixXd UVt(3, 3);

  LOG(INFO) << "[Camera::recoverPosePPNP] Recovering camera pose using PPNP method";

  Z = Z.Zero(nop, nop);
  e = e.Ones(nop, 1);
  A = A.Identity(nop, nop) - (e * e.transpose()) / nop;
  II = e / nop;
  Eo = Eo.Ones(nop, 3) * 1000;

  i = 0;
  for (iter = tracks.begin(); iter != tracks.end(); iter++)
    {
      point3d = (*iter)->getPoint(Track::SOURCEPT);
      S(i, 0) = point3d(0);
      S(i, 1) = point3d(1);
      S(i, 2) = point3d(2);

      point3d = getRay(*iter).getDirection();
      P(i, 0) = point3d(0) / point3d(2);
      P(i, 1) = point3d(1) / point3d(2);
      P(i, 2) = 1.0;

      pp(i, 0) = P(i, 0) * P(i, 0) + P(i, 1) * P(i, 1) + P(i, 2) * P(i, 2);

      Z(i, i) = 1000.0f; // wait, what?
      i++;
    }

  while (err > tol)
    {

      R1 = P.transpose()*Z*A*S;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(R1, Eigen::ComputeThinU | Eigen::ComputeThinV);

      Vt = svd.matrixV();
      Vt.transposeInPlace();
      UVt = svd.matrixU() * Vt;
      R1 << 1, 0, 0, 0, 1, 0, 0, 0, UVt.determinant();
      R2 = svd.matrixU() * R1 * Vt;

      PR = P * R2;
      c = (S - Z * PR).transpose() * II;
      Y = S - e * c.transpose();
      PRY = PR * Y.transpose();

      for (i = 0; i < nop; i++)
        {
          d = PRY(i, i) / pp(i, 0);
          Z(i, i) = (d < 0.0f) ? 0.0f : d;
        }

      E = Y - Z * PR;
      err = (E-Eo).norm();
      Eo = E;
    }

  cout << "A = " << endl << cpose.getMatrix() << endl;
  R = R2.block(0,0,3,3);
  t = -R * c.block(0,0,3,1);
  cpose.setMatrix(R, t);
}

