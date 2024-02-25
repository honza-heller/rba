/*
 * scene_save.cpp
 *
 *  Created on: Mar 2, 2014
 *      Author: jheller
 */

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <boost/lexical_cast.hpp>
namespace bfs = boost::filesystem;
namespace bs = boost::system;

#include "definitions.h"
#include "scene.h"

#include "camera_basic.h"
#include "camera_opencv.h"
#include "track_calibdev.h"


void Scene::save(void)
{
  if (opts->cposes_res_txt != "")
    saveCameraPoses();
  if (opts->cmodel_res_txt != "")
    saveCameraIntrinsics();
  if (opts->crerrs_res_txt != "")
    saveCameraResidualErrors();
  if (opts->target_res_txt != "")
    saveTargetPoints();
  if (opts->tidets_res_txt != "")
    saveTargetDetections();
}

void Scene::saveTargetDetections(void)
{
  bfs::path fpath(opts->tidets_res_txt);
  bs::error_code ec;
  Track *point;
  int    *cams;

  LOG_INFO << "Saving image detections to " << fpath.string();

  if (exists(fpath, ec))
    {
      if (is_directory(fpath, ec))
        LOG_FATAL << "Not a regular file: " << fpath.string() << ": " << ec.message();
      else if (is_regular_file(fpath)|| is_symlink(fpath))
        LOG_WARN << "File already exists and will be rewritten: " << fpath.string();
    }

  ofstream rFile(fpath.c_str());

  if (!rFile)
    LOG_FATAL << "Cannot open file for writing: " << fpath.string();

  if (no_tracks == 0)
    LOG_WARN << "There are no image detections to be saved";

  rFile << no_cameras << " " << no_tracks << " " << 0 << " " << 0 << endl;
  rFile << std::setprecision(20) << std::scientific;

  for (int i = 0; i < no_tracks; i++)
    {
      point = tracks[i];
      cams = point->getCamerasIdx();

      for (int j = 0; j < point->getNumCameras(); j++)
        {
          Vector2f det = point->getDetection(j, Track::SOURCE, true);
          rFile << cams[j] << " " << i << " " << (double) det(0) << " " << (double) det(1) << endl;
        }
    }

  rFile.close();
}

void Scene::saveTargetPoints(void)
{
  bfs::path fpath(opts->target_res_txt);
  bs::error_code ec;

  LOG_INFO << "Saving scene 3D points to " << fpath.string();

  if (exists(fpath, ec))
    {
      if (is_directory(fpath, ec))
        LOG_FATAL << "Not a regular file: " << fpath.string() << ": " << ec.message();
      else if (is_regular_file(fpath))
        LOG_WARN << "File already exists and will be rewritten: " << fpath.string();
    }

  ofstream rFile(fpath.c_str());

  if (!rFile)
    LOG_FATAL << "Cannot open file for writing: " << fpath.string();

  if (no_tracks == 0)
    LOG_WARN << "There are no scene points to be saved";

  rFile << std::setprecision(20) << std::scientific;

  for (int i = 0; i < no_tracks; i++)
    {
      Vector4f pt = tracks[i]->getPoint(Track::SOURCEPT);
      rFile << (double) pt(0) << " " << (double) pt(1) << " " << (double) pt(2) << endl;
    }

  rFile.close();
}

void Scene::saveCameraResidualErrors(void)
{
  bfs::path fpath(opts->crerrs_res_txt);
  bs::error_code ec;
  Track *point;
  int    *cams;

  LOG_INFO << "Saving camera residual errors to " << fpath.string();

  if (exists(fpath, ec))
    {
      if (is_directory(fpath, ec))
        LOG_FATAL << "Not a regular file: " << fpath.string() << ": " << ec.message();
      else if (is_regular_file(fpath))
        LOG_WARN << "File already exists and will be rewritten: " << fpath.string();
    }

  ofstream rFile(fpath.c_str());

  if (!rFile)
    LOG_FATAL << "Cannot open file for writing: " << fpath.string();

  if (no_tracks == 0)
    LOG_WARN << "There are no residual errors to be saved";

  rFile << no_cameras << " " << no_tracks << " " << 0 << endl;
  rFile << std::setprecision(20) << std::scientific;

  for (int i = 0; i < no_tracks; i++)
    {
      point = tracks[i];
      cams = point->getCamerasIdx();

      for (int j = 0; j < point->getNumCameras(); j++)
        rFile << cams[j] << " " << i << " " << point->getResidualError(j, Track::CAMERA_RESIDUAL, true) << endl;
    }

  rFile.close();
}

void Scene::saveCameraIntrinsics(void)
{
  bfs::path fpath(opts->cmodel_res_txt);
  bs::error_code ec;
  OpenCVCamera *camera;
  Matrix3f K;
  Vector8f dist;

  LOG_INFO << "Saving OpenCV internal camera calibration to " << fpath.string();

  if (cameras.size() == 0)
    {
      LOG_WARN << "There are no cameras to be saved";
      return;
    }

  if (cameras[0]->getCameraType() != Camera::OPENCV)
    LOG_FATAL << "Only OpenCV camera model supported at the time";

  camera = (OpenCVCamera *) cameras[0];

  if (exists(fpath, ec))
    {
      if (is_directory(fpath, ec))
        LOG_FATAL << "Not a regular file: " << fpath.string() << ": " << ec.message();
      else if (is_regular_file(fpath))
        LOG_WARN << "File already exists and will be rewritten: " << fpath.string();
    }

  ofstream rFile(fpath.c_str());

  if (!rFile)
    LOG_FATAL << "Cannot open file for writing: " << fpath.string();

  camera->getIntrinsics(K, dist);

  rFile << std::setprecision(20) << std::scientific;

  rFile << K << endl;
  rFile << (double) dist(0) << " " << (double) dist(1) << " " << (double) dist(2) << endl;
  rFile << (double) dist(3) << " " << (double) dist(4) << " " << (double) dist(5) << endl;
  rFile << (double) dist(6) << " " << (double) dist(7) << " " << (double) 0.0f    << endl;

  rFile.close();
}

void Scene::saveCameraPoses(void)
{
  bfs::path fpath(opts->cposes_res_txt);
  bs::error_code ec;

  LOG_INFO << "Saving camera poses to " << fpath.string();

  if (exists(fpath, ec))
    {
      if (is_directory(fpath, ec))
        LOG_FATAL << "Not a regular file: " << fpath.string() << ": " << ec.message();
      else if (is_regular_file(fpath))
          LOG_WARN << "File already exists and will be rewritten: " << fpath.string();
    }

  ofstream rFile(fpath.c_str());

  if (!rFile)
    LOG_FATAL << "Cannot open file for writing: " << fpath.string();

  rFile << std::setprecision(20) << std::scientific;

  for (int i = 0; i < no_cameras; i++)
    cameras[i]->getPose()->writeRt(rFile);

  rFile.close();
}

