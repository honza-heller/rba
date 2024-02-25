/*
 * ttransform.h
 *
 *  Created on: Sep 29, 2014
 *      Author: jheller
 */

#ifndef TTRANSFORM_H_
#define TTRANSFORM_H_

#include "definitions.h"
#include "cmodelfuncs.h"

#include "ceres/ceres.h"

#include <Eigen/QR>

class TargetTransform {
private:
  typedef CameraModels::CameraModelType CameraModelType;

  int                     iwidth;
  int                     iheight;

  Matrix3f                H;
  Matrix3f                K;
  Matrix3f                Ko;
  Eigen::FullPivHouseholderQR<Matrix3f> iH;
  Eigen::FullPivHouseholderQR<Matrix3f> iK;
  Vector8f                dist;
  std::vector<int>        dist_cvars;
  CameraModelType         cmodel;
  bool                    has_external_calib;

  ceres::Problem          *problem;
  ceres::Solver::Options  coptions;

  const RbaOptions        *opts;

  void setDistParamsConstant(const vector<int> & cmodel_mask)
  {
    dist_cvars.clear();

    if (!cmodel_mask[0]) dist_cvars.push_back(0);
    if (!cmodel_mask[1]) dist_cvars.push_back(1);
    if (!cmodel_mask[2]) dist_cvars.push_back(4);
    if (!cmodel_mask[3]) dist_cvars.push_back(5);
    if (!cmodel_mask[4]) dist_cvars.push_back(6);
    if (!cmodel_mask[5]) dist_cvars.push_back(7);
    if (!cmodel_mask[6])
    {
      dist_cvars.push_back(2);
      dist_cvars.push_back(3);
    }
  }

public:

  TargetTransform() {
    // Set Ceres optimization parameters
    coptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    coptions.num_threads = 1;
    //coptions.num_linear_solver_threads = 1;
    coptions.use_nonmonotonic_steps = true;
    coptions.minimizer_progress_to_stdout = false;
    coptions.logging_type = ceres::SILENT;
    has_external_calib = false;
    dist_cvars.clear();

    opts = NULL;
    problem = NULL;
  }

  void setParams(int _iwidth, int _iheight, const RbaOptions * const _opts) {
    iwidth = _iwidth;
    iheight = _iheight;
    opts = _opts;

    // Set Transformation parameters
    H = Matrix3f::Zero();

    if (!has_external_calib)
      {
        double f = double(std::max(iwidth, iheight));
        double cx = (iwidth - 1.0f) / 2.0f;
        double cy = (iheight - 1.0f) / 2.0f;

        K <<    f, 0.0f,   cx,
             0.0f,    f,   cy,
             0.0f, 0.0f, 1.0f;

        iK.compute(K);

        // Set distortion model
        dist = Vector8f::Zero();

        // Set distortion model mask
        setDistParamsConstant(opts->cmodel_mask);

        // Set camera model type
        cmodel = (CameraModelType) opts->cmodel_type;
      }
  }

  void setParams(const vector<int> & cmodel_mask, const CameraModelType &_cmodel)
  {
    setDistParamsConstant(cmodel_mask);
    cmodel = _cmodel;
  }

  void setParams(const Matrix3f &_K, const Vector8f &_dist, const vector<int> & cmodel_mask, const CameraModelType &_cmodel)
  {
    has_external_calib = true;
    dist = _dist;
    setDistParamsConstant(cmodel_mask);
    Ko = K;
    K = _K;
    iK.compute(K);
    cmodel = _cmodel;
  }

  int getWidth() const {
    return iwidth;
  }

  int getHeight() const {
    return iheight;
  }

  void setHomography(const Matrix3f &_H) {
    H = _H;

    // Normalize H
    H /= H(2,2);

    // Get QR decomposition to use instead of H.inverse()
    iH.compute(H);
  }

  const Matrix3f & getHomography(void) {
    return H;
  }

  Vector8f getDist() const {
    return dist;
  }

  void setDist(const Vector8f &_dist) {
    dist = _dist;
  }

  void addHomographyResidual(const Vector2f &pos, const Vector2f &det) {
    ceres::CostFunction *cost_function;

    if (problem == NULL)
      problem = new ceres::Problem();

    cost_function =
      new ceres::AutoDiffCostFunction<HomoDistError, 2, 9, 8>(
        new HomoDistError(pos, det, this));

    problem->AddResidualBlock(cost_function, NULL, H.data(), dist.data());
  }

  void fitHomography() {
    ceres::Solver::Summary summary;

    if (problem == NULL)
      LOG_FATAL << "Add residuals first";

    if (dist_cvars.size() == 8)
      {
        problem->SetParameterBlockConstant(dist.data());
      }
    else if (dist_cvars.size() > 0)
      {
        ceres::SubsetParameterization *subset_parameterization = new ceres::SubsetParameterization(8, dist_cvars);
        problem->SetParameterization(dist.data(), subset_parameterization);
      }

    Solve(coptions, problem, &summary);

    delete problem;
    problem = NULL;

    // Normalize H
    H /= H(2,2);

    // Get QR decomposition to use instead of H.inverse()
    iH.compute(H);
  }



  void projImg2CamD(const Vector2f &u, Vector2f &v) const
   {
     Vector3f v3 = iK.solve(Vector3f(u(0), u(1), 1.0f));
     v(0) = v3(0);
     v(1) = v3(1);
   }

   Vector2f projImg2CamD(const Vector2f &u) const
   {
     Vector2f v;
     projImg2CamD(u, v);
     return v;
   }

   double projImg2CamD(const double d) const
   {
     return std::max(double(d / K(0,0)), double(d / K(1,1)));
   }

   void projCamD2Img(const Matrix3f &K, const Vector2f &u, Vector2f &v) const
   {
     Vector3f v3 = K * Vector3f(u(0), u(1), 1.0f);
     v(0) = v3(0);
     v(1) = v3(1);
   }

   void projCamD2Img(const Vector2f &u, Vector2f &v, bool use_old = false) const
   {
     if (use_old)
       projCamD2Img(Ko, u, v);
     else
       projCamD2Img(K, u, v);
   }

   Vector2f projCamD2Img(const Vector2f &u) const
   {
     Vector2f v;
     projCamD2Img(u, v);
     return v;
   }

   void projCamD2Img(const Vector2f &u, int &x, int &y) const
   {
     Vector2f v;
     projCamD2Img(u, v);
     x = int(round(double(v(0))));
     y = int(round(double(v(1))));
   }

   void projTrg2CamD(const Vector2f &u, Vector2f &v) const
   {
     projTrg2CamD(H.data(), dist.data(), u.data(), v.data(), cmodel);
   }

   Vector2f projTrg2CamD(const Vector2f &u) const
   {
     Vector2f v;
     projTrg2CamD(u, v);
     return v;
   }

   void projCamD2CamUCamDRational(const Vector2f &u, Vector2f &v) const
   {
     int i, iters = 500;
     double e2 = 1e-16f;
     Vector2f w;
     double r, dx, dy, icdist;

     v = u;

     // Unapply rational  model
     for (i = 0; i < iters; i++)
       {
         w = v;
         r = v.squaredNorm();

         if (cmodel == CameraModels::CUBIC_RATIONAL)
           r = v.norm();
         else
           r = v.squaredNorm();

         icdist = (1.0f + ((dist(7)*r + dist(6))*r + dist(5))*r)
                 /(1.0f + ((dist(4)*r + dist(1))*r + dist(0))*r);

         dx = 2.0f * dist(2) * v(0) * v(1) + dist(3) * (r + 2.0f * v(0) * v(0));
         dy = 2.0f * dist(3) * v(0) * v(1) + dist(2) * (r + 2.0f * v(1) * v(1));

         v(0) = (u(0) - dx) * icdist;
         v(1) = (u(1) - dy) * icdist;

         if ((v - w).squaredNorm() < e2)
           break;
       }
   }

   void projCamD2CamUFisheyeP8P(const Vector2f &u, Vector2f &v) const
   {
     int i, iters = 500;
     double e2 = 1e-16f;
     Vector2f w;
     double m, r, dx, dy, icdist;

     v = u;

     // Unapply rational  model
     for (i = 0; i < iters; i++)
       {
         w = v;
         r = v.squaredNorm();

         icdist = (1.0f + (dist(7)*r + dist(6))*r)
                 /(1.0f + (dist(5)*r + dist(4))*r);

         dx = 2.0f * dist(2) * v(0) * v(1) + dist(3) * (r + 2.0f * v(0) * v(0));
         dy = 2.0f * dist(3) * v(0) * v(1) + dist(2) * (r + 2.0f * v(1) * v(1));

         v(0) = (u(0) - dx) * icdist;
         v(1) = (u(1) - dy) * icdist;

         if ((v - w).squaredNorm() < e2)
           break;
       }

     // Unapply fisheye model
     r = v.norm();
     m = std::tan(incasin(double(r/dist(0))) / double(dist(1)));
     v *= m / r;
   }

   void projCamD2CamUFisheyeE7P(const Vector2f &u, Vector2f &v) const
   {
     int i, iters = 500;
     double e2 = 1e-16f;
     Vector2f w;
     double m, r, dx, dy, icdist;

     v = u;

     // Unapply rational  model
     for (i = 0; i < iters; i++)
       {
         w = v;
         r = v.squaredNorm();

         icdist = (1.0f + (dist(7)*r + dist(6))*r)
                 /(1.0f + (dist(5)*r + dist(4))*r);

         dx = 2.0f * dist(2) * v(0) * v(1) + dist(3) * (r + 2.0f * v(0) * v(0));
         dy = 2.0f * dist(3) * v(0) * v(1) + dist(2) * (r + 2.0f * v(1) * v(1));

         v(0) = (u(0) - dx) * icdist;
         v(1) = (u(1) - dy) * icdist;

         if ((v - w).squaredNorm() < e2)
           break;
       }

       // Unapply fisheye
       r = v.norm();
       m = std::tan(double(r / dist(0)));
       v *= m / r;
   }


  void projCamD2CamU(const Vector2f &u, Vector2f &v) const
   {
     if (cmodel == CameraModels::FISHEYE_D3P) // FISHEYE_D3P
       LOG_FATAL << "Not implemented";
     else if (cmodel == CameraModels::FISHEYE_P8P) // FISHEYE_D3P
       projCamD2CamUFisheyeP8P(u, v);
     else if (cmodel == CameraModels::FISHEYE_E7P) // FISHEYE_D3P
       projCamD2CamUFisheyeE7P(u, v);
     else if (cmodel == CameraModels::CUBIC_RATIONAL) // CUBIC_RATIONAL
       projCamD2CamUCamDRational(u, v);
     else if (cmodel == CameraModels::OPENCV_RATIONAL) // OPENCV_RATIONAL
       projCamD2CamUCamDRational(u, v);
     else
       LOG_FATAL << "Unknown camera type";
   }

  void projImg2CamU(const Vector2f &u, Vector2f &v) const
   {
     Vector2f w = projImg2CamD(u);
     projCamD2CamU(w, v);
   }

  Vector2f projImg2CamU(const Vector2f &u) const
   {
     Vector2f v;
     Vector2f w = projImg2CamD(u);
     projCamD2CamU(w, v);

     return v;
   }

   void projCamD2Trg(const Vector2f &u, Vector2f &v) const
   {
     if (cmodel == CameraModels::FISHEYE_P8P)
       {
         projCamD2CamUFisheyeP8P(u, v);
         Vector3f vh = iH.solve(Vector3f(v(0), v(1), 1.0f));
         v(0) = vh(0) / vh(2);
         v(1) = vh(1) / vh(2);
       }
     else
       {
         // As there is no analytical inverse to u = projTrg2CamD(v),
         // we will solve v = projCamD2Trg(u) as a root to projTrg2CamD(v) - u = 0
         // by minimizing || projTrg2CamD(v) - u ||^2, where v_0 == inv(H) * u
         Vector3f vh = iH.solve(Vector3f(u(0), u(1), 1.0f));
         v(0) = vh(0) / vh(2);
         v(1) = vh(1) / vh(2);

         ceres::Problem problem;
         ceres::CostFunction *cost_function;
         ceres::Solver::Summary summary;

         cost_function = new ceres::AutoDiffCostFunction<InvProjError, 2, 2>(new InvProjError(u, this));
         problem.AddResidualBlock(cost_function, NULL, v.data());
         Solve(coptions, &problem, &summary);
       }
   }

   Vector2f projCamD2Trg(const Vector2f &u) const
   {
     Vector2f v;
     projCamD2Trg(u, v);
     return v;
   }

   void projCamD2TrgRnd(const Vector2f &u, Vector2f &v) const
   {
     projCamD2Trg(u, v);
     v(0) = round(double(v(0)));
     v(1) = round(double(v(1)));
   }


   void projImg2Trg(const Vector2f &u, Vector2f &v) const
   {
     Vector2f w;
     projImg2CamD(u, w);
     projCamD2Trg(w, v);
   }

  void projImg2TrgRnd(const Vector2f &u, Vector2f &v) const
   {
     projImg2Trg(u, v);
     v(0) = round(double(v(0)));
     v(1) = round(double(v(1)));
   }

   Vector2f projImg2Trg(const Vector2f &u) const
   {
     Vector2f v;
     projImg2Trg(u, v);
     return v;
   }

   void projTrg2Img(const Vector2f &u, Vector2f &v) const {
     Vector2f w;
     projTrg2CamD(u, w);
     projCamD2Img(w, v);
   }


   template<typename T>
   inline static void projTrg2CamDFisheyeP8P(const T* const H, const T* const dist,
                                   const T* const pt, T * const pr)
   {
     T Y[3], u[2];
     T r, m, theta, d1, d2;

     // Apply homography to the input vector
     Y[0] = H[0] * pt[0] + H[3] * pt[1] + H[6];
     Y[1] = H[1] * pt[0] + H[4] * pt[1] + H[7];
     Y[2] = H[2] * pt[0] + H[5] * pt[1] + H[8];

     // Normalize input vector
     r = T(1.0f) / ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1] + Y[2] * Y[2]);
     if (r != T(0.0))
       {
         Y[0] = Y[0] * r;
         Y[1] = Y[1] * r;
         Y[2] = Y[2] * r;
       }

     // Apply fisheye model
     r = ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1]);
     if (r == T(0.0f))
       {
         pr[0] = T(0.0);
         pr[1] = T(0.0);
         return;
       }

     theta = ceres::acos(Y[2]);
     m = dist[0] * incsin(dist[1] * theta) / r;
     u[0] = Y[0] * m;
     u[1] = Y[1] * m;

     // Apply rational model + tangential distortion
     r = u[0] * u[0] + u[1] * u[1];

     d1 = T(1.0f) + dist[4] * r + dist[5] * r * r;
     d2 = T(1.0f) + dist[6] * r + dist[7] * r * r;

     pr[0] = (d1/d2) * u[0] + T(2.0f) * dist[2] * u[0] * u[1] + dist[3] * (r + T(2.0f) * u[0] * u[0]);
     pr[1] = (d1/d2) * u[1] + T(2.0f) * dist[3] * u[0] * u[1] + dist[2] * (r + T(2.0f) * u[1] * u[1]);
   }

   template<typename T>
   inline static void projTrg2CamDFisheyeE7P(const T* const H, const T* const dist,
                                   const T* const pt, T * const pr)
   {
     T Y[3], u[2];
     T r, m, theta, d1, d2;

     // Apply homography to the input vector
     Y[0] = H[0] * pt[0] + H[3] * pt[1] + H[6];
     Y[1] = H[1] * pt[0] + H[4] * pt[1] + H[7];
     Y[2] = H[2] * pt[0] + H[5] * pt[1] + H[8];

     // Normalize input vector
     r = T(1.0f) / ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1] + Y[2] * Y[2]);
     if (r != T(0.0))
       {
         Y[0] = Y[0] * r;
         Y[1] = Y[1] * r;
         Y[2] = Y[2] * r;
       }

     // Apply fisheye model
     r = ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1]);
     if (r == T(0.0f))
       {
         pr[0] = T(0.0);
         pr[1] = T(0.0);
         return;
       }

     theta = ceres::acos(Y[2]);
     m = (dist[0] * theta) / r;
     u[0] = Y[0] * m;
     u[1] = Y[1] * m;

     // Apply rational model + tangential distortion
     r = u[0] * u[0] + u[1] * u[1];

     d1 = T(1.0f) + dist[4] * r + dist[5] * r * r;
     d2 = T(1.0f) + dist[6] * r + dist[7] * r * r;

     pr[0] = (d1/d2) * u[0] + T(2.0f) * dist[2] * u[0] * u[1] + dist[3] * (r + T(2.0f) * u[0] * u[0]);
     pr[1] = (d1/d2) * u[1] + T(2.0f) * dist[3] * u[0] * u[1] + dist[2] * (r + T(2.0f) * u[1] * u[1]);
   }

   template<typename T>
   inline static void projTrg2CamDFisheyeD3P(const T* const H, const T* const dist,
                                   const T* const pt, T * const pr)
   {
     T Y[3], u[2];
     T r, m, theta;

     // Apply homography to the input vector
     Y[0] = H[0] * pt[0] + H[3] * pt[1] + H[6];
     Y[1] = H[1] * pt[0] + H[4] * pt[1] + H[7];
     Y[2] = H[2] * pt[0] + H[5] * pt[1] + H[8];

     // Normalize input vector
     r = T(1.0f) / ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1] + Y[2] * Y[2]);
     if (r != T(0.0))
       {
         Y[0] = Y[0] * r;
         Y[1] = Y[1] * r;
         Y[2] = Y[2] * r;
       }

     // Apply fisheye model
     r = ceres::sqrt(Y[0] * Y[0] + Y[1] * Y[1]);
     if (r == T(0.0f))
       {
         pr[0] = T(0.0);
         pr[1] = T(0.0);
         return;
       }

     theta = ceres::acos(Y[2]);
     m = dist[0] * incsin(dist[1] * theta) / r;
     u[0] = pt[0] * m;
     u[1] = pt[1] * m;

     // Apply division model
     r = u[0] * u[0] + u[1] * u[1];
     if (r == T(0.0f))
       m = T(1.0f);
     else
       m = (T(1.0f) - ceres::sqrt(T(1.0f) - T(4.0f) * dist[4] * r)) / (T(2.0f) * dist[4] * r);

     pr[0] = u[0] * m;
     pr[1] = u[1] * m;
   }

   template<typename T>
   inline static void projTrg2CamDRational(const T* const H, const T* const dist,
                                   const T* const pt, T * const pr, const int cmodel)
   {
     T p[3], r2, d1, d2;
     T r = T(0.0);

       p[0] = H[0] * pt[0] + H[3] * pt[1] + H[6];
       p[1] = H[1] * pt[0] + H[4] * pt[1] + H[7];
       p[2] = H[2] * pt[0] + H[5] * pt[1] + H[8];

       if (p[2] != T(0.0f))
         {
           p[0] = p[0] / p[2];
           p[1] = p[1] / p[2];
         }

       r2 = p[0] * p[0] + p[1] * p[1];

       if (cmodel == 1)
           r = ceres::sqrt(r2);
       else if (cmodel == 0)
           r = r2;
       else
         LOG_FATAL << "[projTrg2CamD] Unknown camera model type";

       d1 = T(1.0f) + dist[0] * r + dist[1] * r * r +  dist[4] * r * r * r;
       d2 = T(1.0f) + dist[5] * r + dist[6] * r * r +  dist[7] * r * r * r;

       pr[0] = (d1/d2) * p[0] + T(2.0f) * dist[2] * p[0] * p[1] + dist[3] * (r + T(2.0f) * p[0] * p[0]);
       pr[1] = (d1/d2) * p[1] + T(2.0f) * dist[3] * p[0] * p[1] + dist[2] * (r + T(2.0f) * p[1] * p[1]);
   }

   template<typename T>
   inline static void projTrg2CamD(const T* const H, const T* const dist,
                                   const T* const pt, T * const pr, const int cmodel)
   {
     if (cmodel == CameraModels::FISHEYE_D3P) // FISHEYE_D3P
       projTrg2CamDFisheyeD3P(H, dist, pt, pr);
     else if (cmodel == CameraModels::FISHEYE_P8P) // FISHEYE_P8P
       projTrg2CamDFisheyeP8P(H, dist, pt, pr);
     else if (cmodel == CameraModels::FISHEYE_E7P) // FISHEYE_E7P
       projTrg2CamDFisheyeE7P(H, dist, pt, pr);
     else if (cmodel == CameraModels::CUBIC_RATIONAL) // CUBIC_RATIONAL
       projTrg2CamDRational(H, dist, pt, pr, cmodel);
     else if (cmodel == CameraModels::OPENCV_RATIONAL) // OPENCV_RATIONAL
       projTrg2CamDRational(H, dist, pt, pr, cmodel);
     else
       LOG_FATAL << "Unknown camera type";
   }

   template<typename T>
   inline static void resTrg2CamD(const T* const H, const T* const dist,
                                  const T* const pt, const T* const det, T* const res, const int cmodel)
   {
     T p[2];

     projTrg2CamD(H, dist, pt, p, cmodel);



     res[0] = (p[0] - det[0]);
     res[1] = (p[1] - det[1]);
   }


   template<typename T>
   inline static void resTrg2Img(const T* const H, const T* const dist,
                                  const T* const pt, const T* const det, T* const res, const T* const K, const int cmodel)
   {
     T p[2], q[2];

     projTrg2CamD(H, dist, pt, p, cmodel);

     q[0] = K[0] * p[0] + K[3] * p[1] + K[6];
     q[1] = K[1] * p[0] + K[4] * p[1] + K[7];

     res[0] = (q[0] - det[0]);
     res[1] = (q[1] - det[1]);
   }


   class InvProjError {
   private:
     double x;
     double y;
     const TargetTransform * trans;

   public:
     InvProjError(const Vector2f p, const TargetTransform * _trans) {
       x = p(0);
       y = p(1);
       trans = _trans;
     }

     template <typename T>
     bool operator()(const T* const p, T* const res) const {
       T det[2], H[9], dist[8];

       det[0] = T(x);
       det[1] = T(y);

       copyVector((double *) trans->H.data(), H, 9);
       copyVector((double *) trans->dist.data(), dist, 8);
       resTrg2CamD(H, dist, p, det, res, trans->cmodel);

       return true;
     }

   };

   class HomoDistError {
   private:
     const Vector2f pos;
     const Vector2f det;
     const TargetTransform * trans;

   public:
     HomoDistError(const Vector2f & _pos, const Vector2f &_det, const TargetTransform * _trans) : pos(_pos), det(_det), trans(_trans) {}

     template <typename T>
     bool operator()(const T* const H, const T* const k, T* const res) const {
       T pt[2], dt[2], K[9];

       pt[0] = T(pos(0));
       pt[1] = T(pos(1));

       dt[0] = T(det(0));
       dt[1] = T(det(1));

       copyVector((double *) trans->K.data(), K, 9);
       resTrg2Img(H, k, pt, dt, res, K, trans->cmodel);

       return true;
     }

   };

};


#endif /* TTRANSFORM_H_ */
