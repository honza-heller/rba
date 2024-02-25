/*
 * track.h
 *
 *  Created on: Jul 31, 2012
 *      Author: jheller
 */

#ifndef TRACK_H_
#define TRACK_H_


#include "definitions.h"
#include <fstream>

class Track {
protected:
  Vector4f point[2];
  Vector2f *detections[3];

  double   *residuals[3];

  int   index;

  int   cams_no;
  int   pcams_no;

  int   *cams_idx;
  int   *pcams_idx;

  int   *keys;

  int  pose_idx;
  bool  src_flag;

public:

  Track() {
    cams_idx = NULL;
    pcams_idx = NULL;
    cams_no = 0;
    pcams_no = 0;
    keys = NULL;

    detections[0] = NULL;
    detections[1] = NULL;
    detections[2] = NULL;

    residuals[0] = NULL;
    residuals[1] = NULL;
    residuals[2] = NULL;

    index = -1;
    pose_idx = 0;
    src_flag = false;
  }

  Track(int _cams_no)
  {
    cams_no = _cams_no;
    cams_idx = new int[cams_no];
    keys = new int[cams_no];
    detections[0] = new Vector2f[cams_no];
    detections[1] = new Vector2f[cams_no];
    detections[2] = new Vector2f[cams_no];
    residuals[0] = new double[cams_no];
    residuals[1] = new double[cams_no];
    residuals[2] = new double[cams_no];

    index = -1;
    pcams_no = 0;
    pcams_idx = NULL;
    pose_idx = 0;
    src_flag = false;
  }


  void deleteData(void) {
    if (cams_idx != NULL)
      {
        delete [] cams_idx;
        cams_idx = NULL;
      }
     if (pcams_idx != NULL)
       {
         delete [] pcams_idx;
         pcams_idx = NULL;
       }
     if (keys != NULL)
       {
         delete [] keys;
         keys = NULL;
       }

     for (int i = 0; i < 3; i++)
       {
         if (residuals[i] != NULL)
           {
             delete [] (residuals[i]);
             residuals[i] = NULL;
           }
       }

     for (int i = 0; i < 3; i++)
       {
         if (detections[i] != NULL)
           {
             delete [] (detections[i]);
             detections[i] = NULL;
           }
       }

     cams_no = 0;
  }

  virtual ~Track() {
    deleteData();
  }

  typedef enum {
    SOURCE = 0,
    IMAGE = 1,
    CAMERA = 2,
  } Basis2D;

  typedef enum {
    SOURCEPT = 0,
    TRIANGULATEDPT = 1
  } PointType;

  typedef enum {
    CAMERA_RESIDUAL = 0,
    ROBOT_IMAGE_SPACE_RESIDUAL = 1,
    ROBOT_OBJECT_SPACE_RESIDUAL = 2
  } ResidualError;

  void setIndex(int i) {
    index = i;
  }

  // Virtual methods
  virtual void loadTrackData(std::ifstream&) = 0;

  // Common methods
  inline int getIndex(void) const {
    return index;
  }

  inline void setPoseIdx(const int idx) {
    pose_idx = idx;
  }

  inline int getPoseIdx(void) const {
    return pose_idx;
  }

  inline bool getPoseFlag(void) const {
    if (pose_idx >= 0)
      return true;
    else
      return false;
  }

  inline bool getSrcFlag(void) const {
    return src_flag;
  }

  inline int getNumCameras(void) const {
    return cams_no;
  }

  inline int *getCamerasIdx(void) const {
    return cams_idx;
  }

  inline int getNumPoseCameras(void) const {
    return pcams_no;
  }

  inline int *getPoseCamerasIdx(void) const {
    return pcams_idx;
  }

  void setPoseCameras(const int _pcams_no, const int * const _pcams_idx) {
    if (pcams_idx != NULL)
      {
        delete [] pcams_idx;
        pcams_idx = NULL;
      }

    pcams_no = _pcams_no;
    pcams_idx = new int[pcams_no];

    for (int i = 0; i < pcams_no; i++)
      pcams_idx[i] = _pcams_idx[i];
  }

  inline void setCameraIndex(const int camera, const int cam_idx)
  {
    if ((cam_idx < 0) || (cam_idx >= cams_no))
      LOG_FATAL << "Camera Index " << cam_idx << " out of bounds [0, " << cams_no  << "]";

    cams_idx[cam_idx] = camera;
  }

  inline int getCameraIndex(const int camera, const bool cam_idx_flag = false) const {
    int cam_idx = -1;

    if (cam_idx_flag)
      {
        cam_idx = camera;
      }
    else
      {
        for (int i = 0; i < cams_no; i++)
          {
            if (camera == cams_idx[i])
              {
                cam_idx = i;
                break;
              }
          }
      }
    return cam_idx;
  }


  inline void setPoint(const PointType ptype, const Vector4f &p) {
      point[ptype] = p;
  }

  inline void setPoint(const PointType ptype, const double * const p) {
      point[ptype] = Vector4f(p[0], p[1], p[2], 1.0f);
  }

  inline Vector4f getPoint(const PointType ptype) const {
    return point[ptype];
  }

  template <typename T> void getPoint(T * const pt, const PointType ptype = SOURCEPT) const {
    pt[0] = T(point[ptype](0));
    pt[1] = T(point[ptype](1));
    pt[2] = T(point[ptype](2));
  }

  void setDetection(const int camera, const Basis2D basis, const Vector2f &det, bool cam_idx_flag = false) {
    int cam_idx = getCameraIndex(camera, cam_idx_flag);

    if (cam_idx < 0)
      LOG_FATAL << "Invalid Camera Index " << cam_idx << " for camera " << camera << " and track no. " << index;

    detections[basis][cam_idx] = det;
  }

  Vector2f getDetection(const int camera, const Basis2D basis, const bool cam_idx_flag = false) const {
    int cam_idx = getCameraIndex(camera, cam_idx_flag);

    if (cam_idx < 0)
      LOG_FATAL << "Invalid Camera Index " << cam_idx << " for camera " << camera << " and track no. " << index;

    return detections[basis][cam_idx];
  }


  void setResidualError(const int pose, const ResidualError rtype, const double res, const bool cam_idx_flag = false) {
    int cam_idx = getCameraIndex(pose, cam_idx_flag);

    if (cam_idx < 0)
      LOG_FATAL << "Invalid Camera Index " << cam_idx << " for pose " << pose << " and track no. " << index;

    residuals[rtype][cam_idx] = res;
  }

  double getResidualError(const int pose, const ResidualError rtype, const bool cam_idx_flag = false) const {
    int cam_idx = getCameraIndex(pose, cam_idx_flag);

    if (cam_idx < 0)
      LOG_FATAL << "Invalid Camera Index " << cam_idx << " for camera " << pose << " and track no. " << index;

    return residuals[rtype][cam_idx];
  }

};


#endif /* TRACK_H_ */
