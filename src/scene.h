/*
 * scene.h
 *
 *  Created on: Feb 25, 2014
 *      Author: hellej1
 */

#ifndef SCENE_H_
#define SCENE_H_

#include "definitions.h"

#include "track.h"
#include "camera.h"
#include "cmodelfuncs.h"
#include "rpose.h"
#include "tdetector.h"

using namespace std;

class Scene {
public:
  typedef enum {
    UNDEF_SCENE  = 0,
    CALIBDEV_1 = 1,
    CALIBDEV_2 = 2,
    CALIBDEV_3 = 3,
    CALIBDEV_4 = 4,
    BUNDLER    = 5,
    VISUALSFM  = 6
  } SceneType;

private:
  // Scene data
  int              no_cameras;  // number of cameras
  int              no_calib_cameras;  // number of cameras with poses associated to them
  int              no_poses;
  int              no_tracks;
  int              no_ptracks; // number of point visible in more than x cameras with poses associated to them
  int              max_cameras; // maximum number of cameras a point is visible in

  vector<Camera*> cameras;
  vector<Camera*> pcameras;
  vector<Track*>  tracks;
  vector<Track*>  ptracks; // points that can be reconstructed in robot frame

  RbaOptions     *opts;

  // Scene info
  SceneType       scene_type;

  // Calibration target
  TargetDetector::TargetType ct_type;
  double                     ct_width;
  double                     ct_height;
  double                     ct_xstride;
  double                     ct_ystride;

  void loadCalibDev(void);
  void loadCameraPoses(void);

  void saveCameraPoses(void);
  void saveCameraIntrinsics(void);
  void saveCameraResidualErrors(void);
  void saveTargetDetections(void);
  void saveTargetPoints(void);

  void detectTargets(void);
  void initTargetDetectors(vector<TargetDetector *> &);
  void runTargetDetectors(vector<TargetDetector *> &);
  void identifyTargetDetections(vector<TargetDetector *> &);
  void deleteTargetDetectors(vector<TargetDetector *> &);
  void updateTracks(const vector<list<Vector3f> > &, const vector<list<Vector2f> > &, const vector<list<unsigned long long int> > &);

  void updateRational2FisheyeP8PModel(void);
  void updateRational2FisheyeE7PModel(void);

  Vector6f getV(const Matrix3f &, const int, const int) const;
  bool getKfromB(const Matrix3f &, Matrix3f &) const;
  void recoverCamerasZhang(void);
  void recoverCamerasBouguet(void);

  void recoverCameras(void);
  void recoverCamerasOpenCV(void);
  void bundleAdjustCameras(void);
  void bundleAdjustCameras(const vector<int> &, const CameraModels::CameraModelType &);
  void recoverPoses(void);
  void bundleAdjustPoses(void);
  void selectXValidationCameras(void);
  void updateResidualErrors(void);

  void clearTracks(void) {
    for (int i = 0; i < (int) cameras.size(); i++)
      cameras[i]->clearTracks();

    for (int i = 0; i < (int) tracks.size(); i++)
      delete tracks[i];

    tracks.clear();
  }

  void clearCameras(void) {
    for (int i = 0; i < (int) cameras.size(); i++)
      delete cameras[i];
    cameras.clear();
  }

public:
  Scene() {
    no_cameras = 0;
    no_poses = 0;
    no_tracks = 0;

    no_calib_cameras = 0;
    no_ptracks = 0;
    max_cameras = 0;

    scene_type = UNDEF_SCENE;

    opts = NULL;
  }

  Scene(RbaOptions *const _opts) {
    Scene();
    opts = _opts;
  }

  ~Scene() {
    clearCameras();
    clearTracks();
  }

  void load(void);
  void save(void);
  void recoverScene(void);

  inline SceneType getSceneType(void) const {
    return scene_type;
  }

  inline void setOptions(RbaOptions * const _opts) {
    opts = _opts;
  }

  inline int getNumCameras(void) const {
    return no_cameras;
  }

  inline int getNumCalibCameras(void) const {
    return no_calib_cameras;
  }

  inline const vector<Track*>& getTracks(void) const {
    return tracks;
  }

  inline const vector<Camera*>& getCameras(void) const {
    return cameras;
  }


};

#endif /* SCENE_H_ */
