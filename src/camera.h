/*
 * camera.h
 *
 *  Created on: Jul 31, 2012
 *      Author: hellej1
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <fstream>
#include <iostream>

#include <list>
#include <vector>

#include "definitions.h"
#include "track.h"
#include "ray.h"
#include "etransform.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace ceres;

#include "CImg.h"
namespace ci = cimg_library;


class Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef enum {
    NONTYPE,
    BASIC,
    OPENCV
  } CameraType;

  typedef enum {
    IMAGE_SPACE = 0,
    OBJECT_SPACE = 1
  } ResidualErrorType;

  typedef enum  {
    CV_EPNP,
    PPNP
  } PoseMethod;

protected:

  int                   index;
  string                img_path;
  int                   iwidth;
  int                   iheight;
  CameraType            camera_type;

  list<Track*>          tracks;
  vector<int>           pidxs;
  bool                  xvalid;

  ETrans                cpose;

  void recoverPoseCVEPNP(void);
  void recoverPosePPNP(void);

  static Eigen::MatrixXd fliptrans(Eigen::MatrixXd);
  static void rq(const Eigen::MatrixXd &, Eigen::MatrixXd &, Eigen::MatrixXd &);

  class RadialHomographyError {
  private:
    const Vector3f X, u;

  public:
    RadialHomographyError(const Vector3f _X, const Vector3f _u) : X(_X), u(_u) {}

    template <typename T>
    bool operator()(const T* const H, const T* const k, T* const res) const {
      T p[3], pt[3];

      pt[0] = T(X(0));
      pt[1] = T(X(1));
      pt[2] = T(X(2));

      p[0] = H[0] * pt[0] + H[3] * pt[1] + H[6] * pt[2];
      p[1] = H[1] * pt[0] + H[4] * pt[1] + H[7] * pt[2];
      p[2] = H[2] * pt[0] + H[5] * pt[1] + H[8] * pt[2];

      p[0] = p[0] / p[2];
      p[1] = p[1] / p[2];

      res[0] = T(u(0)) - p[0];
      res[1] = T(u(1)) - p[1];

      return true;
    }
  };

  class HomographyError {
  private:
    const Vector3f X, u;

  public:
    HomographyError(const Vector3f _X, const Vector3f _u) : X(_X), u(_u) {}

    template <typename T>
    bool operator()(const T* const H, T* const res) const {
      T p[3], pt[3];

      pt[0] = T(X(0));
      pt[1] = T(X(1));
      pt[2] = T(X(2));

      p[0] = H[0] * pt[0] + H[3] * pt[1] + H[6] * pt[2];
      p[1] = H[1] * pt[0] + H[4] * pt[1] + H[7] * pt[2];
      p[2] = H[2] * pt[0] + H[5] * pt[1] + H[8] * pt[2];

      p[0] = p[0] / p[2];
      p[1] = p[1] / p[2];

      res[0] = T(u(0)) - p[0];
      res[1] = T(u(1)) - p[1];

      return true;
    }
  };

public:

  Camera() {
    xvalid = false;
    iwidth = -1;
    iheight = -1;
    index = -1;
    camera_type = NONTYPE;
  }

  virtual ~Camera() {}

  // Virtual methods
  virtual void loadCameraData(ifstream&) = 0;
  virtual Ray getRay(const Track* const) const = 0;
  virtual Vector2f getProjection(const Track * const point, const Track::PointType ptype,
                                 const Track::Basis2D basis, const ETrans * const trans = NULL) const = 0;
  virtual void updateDetections(void) = 0;


  // Common methods
  void recoverPose(PoseMethod method);
  void recoverPosePlanar();
  void recoverHomography(Matrix3f &, bool = false) const;
  void recoverRadialHomography(Matrix3f &, double &, bool = false) const;
  void recoverPoseKrPlanar(Matrix3f &, Vector3f &, Matrix3f &, double &) const;
  void recoverPoseKPlanar(Matrix3f &, Vector3f &, Matrix3f &) const;
  static void decomposeH(const Matrix3f &, const Matrix3f &, Matrix3f &, Vector3f &);
  static void decomposeP(const MatrixXf &, Matrix3f &, Matrix3f &, Vector3f &);
  
  #ifndef RBA_NO_OPENCV
  void recoverHomographyOcv(Matrix3f &H) const;
  #endif

  void loadImageData(ci::CImg<unsigned char> & img) {
    img.load(img_path.c_str());

    iwidth = img.width();
    iheight = img.height();
    LOG_INFO << "Image loaded: " << img_path << "[" << getImageWidth() << "x" << getImageHeight() << "]";
  }

  void createImageData(ci::CImg<unsigned char> & img) {
	  img.assign(getImageWidth(), getImageHeight(), 1, 3);
  }

  int getImageWidth(void) const {
    return iwidth;
  }

  int getImageHeight(void) const {
    return iheight;
  }

  CameraType getCameraType(void) const {
    return camera_type;
  }

  void setImageWidth(const int w) {
    iwidth = w;
  }

  void setImageHeight(const int h) {
    iheight = h;
  }

  void setImagePath(const string _img_path) {
    img_path = _img_path;
  }

  string getImagePath(void) const {
    return img_path;
  }

  bool hasPose(void) const {
    return cpose.isZero();
  }

  ETrans *getPose(void) {
    return &cpose;
  }

  void setPose(const ETrans * const ext) {
    cpose.setMatrix(ext->getMatrix());
  }

  void setPose(const ETrans &ext) {
    cpose = ext;
  }

  void setIndex(const int i) {
    index = i;
  }

  int getIndex(void) const {
    return index;
  }

  void setXvalidFlag(const bool _xvalid) {
    xvalid = _xvalid;
  }

  bool getXValidFlag(void) const {
    return xvalid;
  }

  bool isCalibCamera(void) const {
    return !(xvalid || cpose.isZero());
  }

  void clearTracks() {
    tracks.clear();
    pidxs.clear();
  }

  void addTrack(Track * const track) {
    tracks.push_back(track);
  }

  const list<Track*>& getTracks(void) const {
    return tracks;
  }

  int getNumTracks(void) const {
    return tracks.size();
  }

  Vector2f getDetection(const Track * const point, const Track::Basis2D basis) const {
    return point->getDetection(index, basis);
  }

  template <typename T>
  void getDetection(const Track * const point, const Track::Basis2D basis, T * const dt) const {
    Vector2f p = getDetection(point, basis);
    dt[0] = T(p(0));
    dt[1] = T(p(1));
  }


  void updateResidualErrors(const Track::ResidualError rerr = Track::CAMERA_RESIDUAL,
                            const ResidualErrorType rtype = IMAGE_SPACE) {
    updateResidualErrors(cpose, rerr, rtype);
  }

  void updateResidualErrors(const ETrans & trans, const Track::ResidualError rerr = Track::CAMERA_RESIDUAL,
                            const ResidualErrorType rtype = IMAGE_SPACE) {
    list<Track*>::iterator iter;

    if (trans.isZero())
      {
        for (iter = tracks.begin(); iter != tracks.end(); iter++)
          (*iter)->setResidualError(index, rerr, -1.0f);
      }
    else
      {
        if (rtype == IMAGE_SPACE)
          {
            for (iter = tracks.begin(); iter != tracks.end(); iter++)
              {
                Track *track = *iter;
                Vector2f res = getDetection(track, Track::IMAGE) - getProjection(track, Track::SOURCEPT, Track::IMAGE, &trans);
                track->setResidualError(index, rerr, res.norm());
              }
          }
        else if (rtype == OBJECT_SPACE)
          {
            for (iter = tracks.begin(); iter != tracks.end(); iter++)
              {
                Track *track = *iter;
                double res[3], pt[3];

                copyVector(trans * track->getPoint(Track::SOURCEPT), pt, 3);
                getObjectSpaceResiduals(track, pt, res);
                track->setResidualError(index, rerr, Vector3f(res[0], res[1], res[2]).norm());
              }
          }
        else
          {
            LOG_FATAL << "Unknown residual error type";
          }
      }
  }

  template<typename T> inline void getObjectSpaceResiduals(const Track * const track, const  T* const point, T* const residuals)
  {
    Vector4f ray = getRay(track).getDirection();
    Matrix3f V = Matrix3f::Identity() - ray.head<3>() * ray.head<3>().transpose();

    residuals[0] = T(V(0,0)) * point[0] + T(V(0,1)) * point[1] + T(V(0,2)) * point[2];
    residuals[1] = T(V(1,0)) * point[0] + T(V(1,1)) * point[1] + T(V(1,2)) * point[2];
    residuals[2] = T(V(2,0)) * point[0] + T(V(2,1)) * point[1] + T(V(2,2)) * point[2];
  }

  template<typename T> void getAngleAxisTranslation(T * const aat) const {
    double caat[6];

    cpose.getAngleAxisTranslation(caat);

    for (int i = 0; i < 6; i++)
      aat[i] = T(caat[i]);
  }

 template<typename T>
 inline static void transformAngleAxisTranslate(const T * const aat, T * const p) {
   T tp[3];

   ceres::AngleAxisRotatePoint(aat, p, tp);

   p[0] = tp[0] + aat[3];
   p[1] = tp[1] + aat[4];
   p[2] = tp[2] + aat[5];
 }

};


#endif /* CAMERA_H_ */
