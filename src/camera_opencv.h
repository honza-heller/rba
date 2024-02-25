/*
 * camera_opencv.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: hellej1
 */


#ifndef CAMERA_OPENCV_H_
#define CAMERA_OPENCV_H_

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "camera.h"
#include "track.h"
#include "etransform.h"
#include "cmodelfuncs.h"

using namespace ceres;

#ifndef RBA_NO_OPENCV
#include <opencv2/opencv.hpp>
using namespace cv;
#endif

class OpenCVCamera : public Camera {
public:
  typedef CameraModels::CameraModelType CameraModelType;

private:

  Matrix3f K;
  Vector8f dist;
  bool aratio_flag;
  CameraModelType camera_model_type;

  void updateDetectionsFisheyeD3P(void) {
    Track *point;
    Vector2f psrc, pcam;
    double r = 0, m;

    for (list<Track*>::iterator iter = tracks.begin(); iter != tracks.end(); iter++)
      {
        point = *iter;
        psrc = point->getDetection(index, Track::SOURCE);

        point->setDetection(index, Track::IMAGE, psrc);

        // Unapply K matrix
        pcam(0) = (psrc(0) - K(0,2)) / K(0,0);
        pcam(1) = (psrc(1) - K(1,2)) / K(1,1);

        r = pcam.squaredNorm();

        if (r == 0.0f)
          {
          pcam.setZero();
          }
        else
          {
            // Unapply 1-parameter division model
            pcam /= 1.0f + dist(4) * r;

            // Unapply fisheye
            r = pcam.norm();
            m = std::tan(incasin(double(r/dist(0))) / double(dist(1)));
            pcam *= m / r;
          }

        point->setDetection(index, Track::CAMERA, pcam);
      }
  }

  void updateDetectionsFisheyeP8P(void) {
    Track *point;
    Vector2f psrc, pcam, pold;
    int i, iters = 500;
    double e2 = 1e-16f;
    double r = 0, m, icdist, dx, dy;

    for (list<Track*>::iterator iter = tracks.begin(); iter != tracks.end(); iter++)
      {
        point = *iter;
        psrc = point->getDetection(index, Track::SOURCE);

        point->setDetection(index, Track::IMAGE, psrc);

        // Unapply K matrix
        psrc(0) = (psrc(0) - K(0,2)) / K(0,0);
        psrc(1) = (psrc(1) - K(1,2)) / K(1,1);
        pcam = psrc;

        r = pcam.squaredNorm();

        if (r == 0.0f)
          {
            pcam.setZero();
          }
        else
          {
            // Unapply rational model
            for (i = 0; i < iters; i++)
              {
                pold = pcam;
                r = pcam.squaredNorm();

                icdist = (1.0f + (dist(7)*r + dist(6))*r)
                        /(1.0f + (dist(5)*r + dist(4))*r);

                dx = 2.0f * dist(2) * pcam(0) * pcam(1) + dist(3) * (r + 2.0f * pcam(0) * pcam(0));
                dy = 2.0f * dist(3) * pcam(0) * pcam(1) + dist(2) * (r + 2.0f * pcam(1) * pcam(1));

                pcam(0) = (psrc(0) - dx) * icdist;
                pcam(1) = (psrc(1) - dy) * icdist;

                if ((pcam - pold).dot(pcam - pold) < e2)
                  break;
              }

            // Unapply fisheye
            r = pcam.norm();
            m = std::tan(incasin(double(r/dist(0))) / double(dist(1)));
            pcam *= m / r;
          }

        point->setDetection(index, Track::CAMERA, pcam);
      }
  }

  void updateDetectionsFisheyeE7P(void) {
    Track *point;
    Vector2f psrc, pcam, pold;
    int i, iters = 500;
    double e2 = 1e-16f;
    double r = 0, m, icdist, dx, dy;

    for (list<Track*>::iterator iter = tracks.begin(); iter != tracks.end(); iter++)
      {
        point = *iter;
        psrc = point->getDetection(index, Track::SOURCE);

        point->setDetection(index, Track::IMAGE, psrc);

        // Unapply K matrix
        psrc(0) = (psrc(0) - K(0,2)) / K(0,0);
        psrc(1) = (psrc(1) - K(1,2)) / K(1,1);
        pcam = psrc;

        r = pcam.squaredNorm();

        if (r == 0.0f)
          {
            pcam.setZero();
          }
        else
          {
            // Unapply rational model
            for (i = 0; i < iters; i++)
              {
                pold = pcam;
                r = pcam.squaredNorm();

                icdist = (1.0f + (dist(7)*r + dist(6))*r)
                        /(1.0f + (dist(5)*r + dist(4))*r);

                dx = 2.0f * dist(2) * pcam(0) * pcam(1) + dist(3) * (r + 2.0f * pcam(0) * pcam(0));
                dy = 2.0f * dist(3) * pcam(0) * pcam(1) + dist(2) * (r + 2.0f * pcam(1) * pcam(1));

                pcam(0) = (psrc(0) - dx) * icdist;
                pcam(1) = (psrc(1) - dy) * icdist;

                if ((pcam - pold).dot(pcam - pold) < e2)
                  break;
              }

            // Unapply fisheye
            r = pcam.norm();
            m = std::tan(double(r/dist(0)));
            pcam *= m / r;
          }

        point->setDetection(index, Track::CAMERA, pcam);
      }
  }

  void updateDetectionsRational(void) {
    Track *point;
    Vector2f psrc, pcam, pold;
    int i, iters = 500;
    double e2 = 1e-16f;
    double r = 0, r2, icdist, dx, dy;

    for (list<Track*>::iterator iter = tracks.begin(); iter != tracks.end(); iter++)
      {
        point = *iter;
        psrc = point->getDetection(index, Track::SOURCE);

        point->setDetection(index, Track::IMAGE, psrc);

        // Undistort point
        psrc(0) = (psrc(0) - K(0,2)) / K(0,0);
        psrc(1) = (psrc(1) - K(1,2)) / K(1,1);
        pcam = psrc;

        for (i = 0; i < iters; i++)
          {
            pold = pcam;
            r2 = pcam(0) * pcam(0) + pcam(1) * pcam(1);

            if (camera_model_type == CameraModels::CUBIC_RATIONAL)
                r = ceres::sqrt(r2);
            else if (camera_model_type == CameraModels::OPENCV_RATIONAL)
                r = r2;
            else
              LOG_FATAL << "Unknown camera model type";

            icdist = (1.0f + ((dist(7)*r + dist(6))*r + dist(5))*r)
                    /(1.0f + ((dist(4)*r + dist(1))*r + dist(0))*r);
            dx = 2.0f * dist(2) * pcam(0) * pcam(1) + dist(3) * (r + 2.0f * pcam(0) * pcam(0));
            dy = 2.0f * dist(3) * pcam(0) * pcam(1) + dist(2) * (r + 2.0f * pcam(1) * pcam(1));

            pcam(0) = (psrc(0) - dx) * icdist;
            pcam(1) = (psrc(1) - dy) * icdist;

            if ((pcam - pold).dot(pcam - pold) < e2)
              break;
          }
        point->setDetection(index, Track::CAMERA, pcam);
      }
  }



public:

  OpenCVCamera() {
    camera_type = OPENCV;
    setCanonicalIntrinsics();
    camera_model_type = CameraModels::OPENCV_RATIONAL;
  }


  void copyIntrinsics(const OpenCVCamera* const ocvcam)
  {
    K = ocvcam->K;
    dist = ocvcam->dist;
  }

  void setCanonicalIntrinsics(void) {
    K << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    dist << 0, 0, 0, 0, 0, 0, 0, 0;
    setImageWidth(1);
    setImageHeight(1);
  }

  template<typename T>
  void getIntrinsics(T * const ck, T * const cd) const {
    ck[0] = T(K(0,0));
    ck[1] = T(K(1,1));
    ck[2] = T(K(0,2));
    ck[3] = T(K(1,2));
    copyVector(dist, cd, 8);
  }

  void getIntrinsics(Matrix3f &_K, Vector8f &_dist) const {
    _K = K;
    _dist = dist;
  }

#ifndef RBA_NO_OPENCV
  void getIntrinsics(Mat &_K, Mat &_dist) const {
    _K = (Mat_<double>(3,3) << K(0,0), K(0,1), K(0,2),
                               K(1,0), K(1,1), K(1,2),
                               K(2,0), K(2,1), K(2,2));
    _dist = (Mat_<double>(8,1) << dist(0), dist(1), dist(2), dist(3), dist(4), dist(5), dist(6), dist(7));
  }
#endif

  void setIntrinsics(const Matrix3f &_K, const Vector8f &_dist) {
    K = _K;
    dist = _dist;
  }

  template<typename T>
  void setIntrinsics(const T * const ck, const T * const cd) {
    K << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    K(0,0) = ck[0];
    K(1,1) = ck[1];
    K(0,2) = ck[2];
    K(1,2) = ck[3];

    for(int i = 0; i < 8; i++)
      dist(i) = cd[i];
  }

#ifndef RBA_NO_OPENCV
  void setIntrinsics(const Mat &_K, const Mat &_dist) {
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        K(i,j) = _K.ptr<double>(i)[j];

    for (int i = 0; i < 8; i++)
     dist(i) = _dist.ptr<double>(0)[i];
  }
#endif

  void setAspectRatioFlag(const bool _aratio_flag) {
    aratio_flag = _aratio_flag;
  }

  bool getAspectRatioFlag(void) const {
    return aratio_flag;
  }

  void setCameraModelType(const CameraModelType _camera_model_type) {
	  camera_model_type = _camera_model_type;
  }

  CameraModelType getCameraModelType(void) const {
	  return camera_model_type;
  }

  virtual void loadCameraData(ifstream &iFile) {
  // OpenCVCamera loads only internal camera calibration

    iFile >> K(0,0) >> K(0,1) >> K(0,2)
          >> K(1,0) >> K(1,1) >> K(1,2)
          >> K(2,0) >> K(2,1) >> K(2,2);

    iFile >> dist(0) >> dist(1) >> dist(2) >> dist(3) >> dist(4) >> dist(5) >> dist(6) >> dist(7);
  }

  virtual Ray getRay(const Track * const point) const {
    Ray ray;
    Vector2f point2D;
    Vector4f dir, orig;

    point2D = point->getDetection(index, Track::CAMERA);

    orig = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
    dir = Vector4f(0.0f, 0.0f, 1.0f, 0.0f) + Vector4f(point2D(0), point2D(1), 0.0f, 0.0f);
    dir.normalize();
    ray.setRay(dir, orig);

    return ray;
  }

  virtual Vector2f getProjection(const Track * const point, const Track::PointType ptype,
                                 const Track::Basis2D basis, const ETrans * trans = NULL) const {
    Matrix4f T;

    if (trans == NULL)
      T = cpose.getMatrix();
    else
      T = trans->getMatrix();

    if (ptype != Track::SOURCEPT)
      LOG_FATAL << "Only Track::SOURCEPT supported by OpenCVCamera";

    if (T.isZero())
      LOG_FATAL << "Cannot project point using zero projection matrix";

    Vector4f point3D = T * point->getPoint(ptype);

    if (basis == Track::CAMERA)
      {
        Vector2f pr;
        pr(0) = point3D(0) / point3D(2);
        pr(1) = point3D(1) / point3D(2);
        return pr;
      }
    else if ((basis == Track::IMAGE) || (basis == Track::SOURCE))
      {
        double ck[4], cd[8], pt[3], pr[2];

        getIntrinsics(ck, cd);
        copyVector(point3D, pt, 3);
        getImageSpaceProjection(ck, cd, pt, pr, true, getCameraModelType());

        return Vector2f(pr[0], pr[1]);
      }
    else
      {
        LOG_FATAL << "Basis not supported: " << basis;
      }

    // Should not be reached
    return Vector2f(0.0, 0.0);
  }


  virtual void updateDetections(void) {
    if (camera_model_type == CameraModels::FISHEYE_D3P)
      updateDetectionsFisheyeD3P();
    else if (camera_model_type == CameraModels::FISHEYE_P8P)
      updateDetectionsFisheyeP8P();
    else if (camera_model_type == CameraModels::FISHEYE_E7P)
      updateDetectionsFisheyeE7P();
    else if (camera_model_type == CameraModels::CUBIC_RATIONAL)
      updateDetectionsRational();
    else if (camera_model_type == CameraModels::OPENCV_RATIONAL)
      updateDetectionsRational();
    else
      LOG_FATAL << "Unknown camera model type";
  }

  void bundleAdjustPose(void)
  {
     list<Track*>::iterator iter;
     double cpos[6];
     ceres::Problem problem;
     ceres::CostFunction *cost_function;
     Solver::Summary summary;
     Solver::Options coptions;

     LOG_INFO << "Running camera pose adjustment on OpenCVCamera type camera id " << getIndex();

     if (tracks.size() == 0)
       {
         LOG_WARN << "Cannot BA pose for camera " << getIndex() << ": there are no points connected with this camera";
         return;
       }

     cpose.getAngleAxisTranslation(cpos);

     for (iter = tracks.begin(); iter != tracks.end(); iter++)
       {
         cost_function =
           new AutoDiffCostFunction<OpenCVCamera::reprojectionErrorPose, 2, 6>(
             new OpenCVCamera::reprojectionErrorPose(this, *iter));
         problem.AddResidualBlock(cost_function, NULL, cpos);
       }

     // Ceres Options
     coptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
     coptions.num_threads = 1;
     //coptions.num_linear_solver_threads = 1;
     coptions.use_nonmonotonic_steps = true;
     coptions.minimizer_progress_to_stdout = false;

     Solve(coptions, &problem, &summary);
     cpose.setAngleAxisTranslation(cpos);
     
     //std::cout << summary.FullReport();
  }


  template<typename T>
  inline static void getImageSpaceResidualsFisheyed3p(const T* const K, const T* const dist, const T* const pt, const T* const det, T * const res, const bool aflag)
  {
    T Y[3], u[2], v[2];
    T r, m, n1;

    // Normalize input vector
    r = T(1.0f) / ceres::sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
    Y[0] = pt[0] * r;
    Y[1] = pt[1] * r;
    Y[2] = pt[2] * r;

    // Apply fisheye model
    r = ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1]);
    if (r == T(0.0f))
      {
        res[0] = K[2] - det[0];
        res[1] = K[3] - det[1];
        return;
      }

    n1 = ceres::acos(Y[2]);
    m = dist[0] * incsin(dist[1] * n1) / r;
    u[0] = Y[0] * m;
    u[1] = Y[1] * m;

    // Unapply K matrix
    v[0] = (det[0] - K[2]) / K[0];
    if (aflag)
      v[1] = (det[1] - K[3]) / K[1];
    else
      v[1] = (det[1] - K[3]) / K[0];

    // Apply division model
    r = v[0] * v[0] + v[1] * v[1];

    m = T(1.0f) / (T(1.0f) + dist[4] * r);

    v[0] = v[0] * m;
    v[1] = v[1] * m;

    // Compute residuals
    res[0] = u[0] - v[0];
    res[1] = u[1] - v[1];
  }

  template<typename T>
  inline static void getImageSpaceProjectionFisheyed3p(const T* const K, const T* const dist, const T* const pt, T * const pr, const bool aflag)
  {
    T Y[3], u[2];
    T r, m, n1, n2;

    // Normalize input vector
    r = T(1.0f) / ceres::sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
    Y[0] = pt[0] * r;
    Y[1] = pt[1] * r;
    Y[2] = pt[2] * r;

    // Apply fisheye model
    r = ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1]);
    if (r == T(0.0f))
      {
        pr[0] = K[2];
        pr[1] = K[3];
        return;
      }

    n1 = ceres::acos(Y[2]);
    m = dist[0] * incsin(dist[1] * n1) / r;
    u[0] = Y[0] * m;
    u[1] = Y[1] * m;

    // Apply division model
    r = u[0] * u[0] + u[1] * u[1];
    n1 = T(2.0f) * dist[4] * r;
    n2 = T(1.0f) - T(2.0f) * n1;

    if ((n1 == T(0.0)) || (n2 <= T(0.0)))
      m = T(1.0f);
    else
      m = (T(1.0f) - ceres::sqrt(n2)) / n1;

    u[0] = u[0] * m;
    u[1] = u[1] * m;

    // Apply K matrix
    pr[0] = K[0] * u[0] + K[2];

    if (aflag)
      pr[1] = K[1] * u[1] + K[3];
    else
      pr[1] = K[0] * u[1] + K[3];
  }

  template<typename T>
  inline static void getImageSpaceProjectionFisheyeP8P(const T* const K, const T* const dist, const T* const pt, T* const pr, const bool aflag)
  {
    T Y[3], u[2], v[2];
    T r, m, n1, d1, d2;

    // Normalize input vector
    r = T(1.0f) / ceres::sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
    Y[0] = pt[0] * r;
    Y[1] = pt[1] * r;
    Y[2] = pt[2] * r;

    // Apply fisheye model
    r = ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1]);
    if (r == T(0.0f))
      {
        pr[0] = K[2];
        pr[1] = K[3];
        return;
      }

    n1 = ceres::acos(Y[2]);
    m = dist[0] * incsin(dist[1] * n1) / r;
    u[0] = Y[0] * m;
    u[1] = Y[1] * m;

    // Apply rational model + tangential distortion
    r = u[0] * u[0] + u[1] * u[1];

    d1 = T(1.0f) + dist[4] * r + dist[5] * r * r;
    d2 = T(1.0f) + dist[6] * r + dist[7] * r * r;

    v[0] = (d1/d2) * u[0] + T(2.0f) * dist[2] * u[0] * u[1] + dist[3] * (r + T(2.0f) * u[0] * u[0]);
    v[1] = (d1/d2) * u[1] + T(2.0f) * dist[3] * u[0] * u[1] + dist[2] * (r + T(2.0f) * u[1] * u[1]);

    // Apply K matrix
    pr[0] = K[0] * v[0] + K[2];

    if (aflag)
      pr[1] = K[1] * v[1] + K[3];
    else
      pr[1] = K[0] * v[1] + K[3];
  }

  template<typename T>
  inline static void getImageSpaceResidualsFisheyeP8P(const T* const K, const T* const dist, const T* const pt, const T* const det, T * const res, const bool aflag)
  {
    T pr[2];

    getImageSpaceProjectionFisheyeP8P(K, dist, pt, pr, aflag);

    // Compute residuals
    res[0] = det[0] - pr[0];
    res[1] = det[1] - pr[1];
  }

  template<typename T>
  inline static void getImageSpaceProjectionFisheyeE7P(const T* const K, const T* const dist, const T* const pt, T* const pr, const bool aflag)
  {
    T Y[3], u[2], v[2];
    T d, r, m, n1, d1, d2;

    // Normalize input vector
    d = ceres::sqrt(pt[0] * pt[0] + pt[1] * pt[1] + pt[2] * pt[2]);
    if (d == T(0.0f))
      {
        pr[0] = K[2];
        pr[1] = K[3];
        return;
      }

    r = T(1.0f) / d;
    Y[0] = pt[0] * r;
    Y[1] = pt[1] * r;
    Y[2] = pt[2] * r;

    // Apply fisheye model
    r = ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1]);
    if (r == T(0.0f))
      {
        pr[0] = K[2];
        pr[1] = K[3];
        return;
      }

    n1 = ceres::acos(Y[2]);
    m = (dist[0] * n1) / r;
    u[0] = Y[0] * m;
    u[1] = Y[1] * m;

    // Apply rational model + tangential distortion
    r = u[0] * u[0] + u[1] * u[1];

    d1 = T(1.0f) + dist[4] * r + dist[5] * r * r;
    d2 = T(1.0f) + dist[6] * r + dist[7] * r * r;

    if (d2 == T(0.0f))
      {
        pr[0] = K[2];
        pr[1] = K[3];
        return;
      }  

    v[0] = (d1/d2) * u[0] + T(2.0f) * dist[2] * u[0] * u[1] + dist[3] * (r + T(2.0f) * u[0] * u[0]);
    v[1] = (d1/d2) * u[1] + T(2.0f) * dist[3] * u[0] * u[1] + dist[2] * (r + T(2.0f) * u[1] * u[1]);

    // Apply K matrix
    pr[0] = K[0] * v[0] + K[2];

    if (aflag)
      pr[1] = K[1] * v[1] + K[3];
    else
      pr[1] = K[0] * v[1] + K[3];
  }

  template<typename T>
  inline static void getImageSpaceResidualsFisheyeE7P(const T* const K, const T* const dist, const T* const pt, const T* const det, T * const res, const bool aflag)
  {
    T pr[2];

    getImageSpaceProjectionFisheyeE7P(K, dist, pt, pr, aflag);

    // Compute residuals
    res[0] = det[0] - pr[0];
    res[1] = det[1] - pr[1];
  }

  template<typename T>
  inline static void getImageSpaceProjectionRational(const T* const K, const T* const dist, const T* const pt, T * const pr, const bool aflag, const CameraModelType cmodel)
  {
    T p1[2], p2[2];
    T r = T(0), r2, d1, d2;

    p1[0] = pt[0] / pt[2];
    p1[1] = pt[1] / pt[2];

    r2 = p1[0] * p1[0] + p1[1] * p1[1];

    if (cmodel == CameraModels::CUBIC_RATIONAL)
        r = ceres::sqrt(r2);
    else if (cmodel == CameraModels::OPENCV_RATIONAL)
        r = r2;
    else
      LOG_FATAL << "Unknown camera type";

    d1 = T(1.0f) + dist[0] * r + dist[1] * r * r +  dist[4] * r * r * r;
    d2 = T(1.0f) + dist[5] * r + dist[6] * r * r +  dist[7] * r * r * r;

    p2[0] = (d1/d2) * p1[0] + T(2.0f) * dist[2] * p1[0] * p1[1] + dist[3] * (r + T(2.0f) * p1[0] * p1[0]);
    p2[1] = (d1/d2) * p1[1] + T(2.0f) * dist[3] * p1[0] * p1[1] + dist[2] * (r + T(2.0f) * p1[1] * p1[1]);

    pr[0] = K[0] * p2[0] + K[2];

    if (aflag)
      pr[1] = K[1] * p2[1] + K[3];
    else
      pr[1] = K[0] * p2[1] + K[3];
  }

  template<typename T>
  inline static void getImageSpaceProjection(const T* const K, const T* const dist, const T* const pt, T * const pr, const bool aflag, const CameraModelType cmodel)
  {
    if (cmodel == CameraModels::FISHEYE_D3P)
      getImageSpaceProjectionFisheyed3p(K, dist, pt, pr, aflag);
    else if (cmodel == CameraModels::FISHEYE_P8P)
      getImageSpaceProjectionFisheyeP8P(K, dist, pt, pr, aflag);
    else if (cmodel == CameraModels::FISHEYE_E7P)
      getImageSpaceProjectionFisheyeE7P(K, dist, pt, pr, aflag);
    else if (cmodel == CameraModels::CUBIC_RATIONAL)
      getImageSpaceProjectionRational(K, dist, pt, pr, aflag, cmodel);
    else if (cmodel == CameraModels::OPENCV_RATIONAL)
      getImageSpaceProjectionRational(K, dist, pt, pr, aflag, cmodel);
    else
      LOG_FATAL << "Unknown camera type: " << cmodel;
  }

  template<typename T>
  inline static void getImageSpaceResiduals(const T* const K, const T* const dist,
                                            const T* const point, const T* const det, T* const res,
                                            const bool aflag, const CameraModelType cmodel)
  {
    T p1[2];

    if (cmodel == CameraModels::FISHEYE_D3P)
      {
        getImageSpaceResidualsFisheyed3p(K, dist, point, det, res, aflag);
      }
    else if (cmodel == CameraModels::FISHEYE_P8P)
      {
        getImageSpaceResidualsFisheyeP8P(K, dist, point, det, res, aflag);
      }
    else if (cmodel == CameraModels::FISHEYE_E7P)
      {
        getImageSpaceResidualsFisheyeE7P(K, dist, point, det, res, aflag);
      }
    else
      {
        getImageSpaceProjection(K, dist, point, p1, aflag, cmodel);
        //T r = det[0] * det[0] + det[1] * det[1];
        res[0] = (p1[0] - det[0]);
        res[1] = (p1[1] - det[1]);
      }
  }

  template<typename T>
  inline void getImageSpaceResiduals(const Track * const track, const T* const point, T* const res) const
  {
    T ck[4], cd[8], dt[2];

    getIntrinsics(ck, cd);
    getDetection(track, Track::SOURCE, dt);
    getImageSpaceResiduals(ck, cd, point, dt, res, true, getCameraModelType());
  }

  class reprojectionErrorCalib {
  private:
    const OpenCVCamera * camera;
    const Track * track;

  public:
    reprojectionErrorCalib(const OpenCVCamera * const _camera, const Track * const _tack) {
      camera = _camera;
      track = _tack;
    }

    template <typename T>
    bool operator()(const T* const ck, const T* const cd, const T* const cpose, T* const residuals) const {
      T dt[2], pt[3];

      track->getPoint(pt);
      camera->getDetection(track, Track::SOURCE, dt);
      transformAngleAxisTranslate(cpose, pt);
      OpenCVCamera::getImageSpaceResiduals(ck, cd, pt, dt, residuals,
                                           camera->getAspectRatioFlag(), camera->getCameraModelType());

      return true;
    }
  };

  class reprojectionErrorPose {
  private:
    const OpenCVCamera *camera;
    const Track *track;

  public:
    reprojectionErrorPose(const OpenCVCamera * const _camera, const Track * const _track) {
      camera = _camera;
      track = _track;
    }

    template <typename T>
    bool operator()(const T* cpose, T* residuals) const {
      T pt[3];

      track->getPoint(pt);
      transformAngleAxisTranslate(cpose, pt);
      camera->getImageSpaceResiduals(track, pt, residuals);
      return true;
    }
  };

};


#endif /* CAMERA_OPENCV_H_ */
