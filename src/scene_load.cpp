/*
 * scene_load.cpp
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

#include "CImg.h"
namespace ci = cimg_library;

#include "definitions.h"
#include "scene.h"

#include "camera_basic.h"
#include "camera_opencv.h"
#include "track_calibdev.h"

// Load

void Scene::load(void) {
  if (!opts)
    LOG_FATAL << "[Scene::load] Options not set, don't known what to do";

  if (opts->calibdev_txt != "")
    loadCalibDev();
  else
    LOG_FATAL << "Please provide calibdev.txt data file";

  // Load camera poses for classical hand-eye or as and initial estimate for CalibDev data
  if (opts->cposes_txt != "")
    loadCameraPoses();

}

void Scene::loadCameraPoses(void)
{
  BasicCamera *bcamera;
  int i, noc;
  bool cflag = false;

  LOG_INFO << "Loading camera poses cposes.txt file: " << opts->cposes_txt;

  ifstream bFile(opts->cposes_txt.c_str());
  if (!bFile)
    LOG_FATAL << "Cannot open camera poses cposes.txt file: " << opts->cposes_txt;

  if (no_cameras > 0)
    {
      // Cameras already created, only load external calibration
      LOG_INFO << "Cameras already created, loading external calibration";

      bFile >> noc;
      if (noc != no_cameras)
        LOG_FATAL << "The number of cameras in the input file (" << noc << ") " <<
                   "does not equal the number of existing cameras (" << no_cameras << ")";

      cflag = true;
    }
  else
    {
      bFile >> no_cameras;
    }

  if (cflag)
    {
      BasicCamera bcamera;
      for (i = 0; i < no_cameras; i++)
        {
          bcamera.loadCameraData(bFile);
          cameras[i]->setPose(bcamera.getPose());
        }
    }
  else
    {
      for (i = 0; i < no_cameras; i++)
        {
          bcamera = new BasicCamera();
          bcamera->setIndex(i);
          bcamera->loadCameraData(bFile);
          cameras.push_back(bcamera);
        }
    }

  bFile.close();

  LOG_INFO << "Loaded " << cameras.size() << " cameras";
}

void Scene::loadCalibDev(void) {
  string line, linebuf, fpath;
  OpenCVCamera *camera, icam;
  CalibdevTrack *track;
  int *camera_idx, no_views;
  int i, j;
  int iwidth = -1, iheight = -1;
  int stype;
  double target_type;
  bool read_flag = true;

  max_cameras = 0;

  LOG_INFO << "Loading calibdev.txt file: " << opts->calibdev_txt;

  ifstream bFile(opts->calibdev_txt.c_str());
  if (!bFile)
    LOG_FATAL << "Cannot open calibdev.txt file: " << opts->calibdev_txt;

  bFile >> no_cameras;
  bFile >> no_tracks;
  bFile >> stype;
  getline(bFile, linebuf); // read the rest of the line

  scene_type = (SceneType) stype;

  cameras.clear();
  cameras.reserve(no_cameras);

  tracks.clear();
  tracks.reserve(no_tracks);

  if (scene_type == CALIBDEV_4)
    {
      LOG_INFO << "Loading calibrated data without known target detections [CALIBDEV_4]";

      bFile >> target_type;
      bFile >> ct_width;
      bFile >> ct_height;
      bFile >> ct_xstride;
      bFile >> ct_ystride;

      ct_type = static_cast<TargetDetector::TargetType>(int(target_type));

      getline(bFile, linebuf); // read the rest of the line

      // Load camera calibration
      icam.loadCameraData(bFile);
      getline(bFile, linebuf); // read the rest of the line

    }

  if (scene_type == CALIBDEV_3)
    {
      LOG_INFO << "Loading uncalibrated data without known target detections [CALIBDEV_3]";

      bFile >> target_type;
      bFile >> ct_width;
      bFile >> ct_height;
      bFile >> ct_xstride;
      bFile >> ct_ystride;

      ct_type = static_cast<TargetDetector::TargetType>(int(target_type));

      getline(bFile, linebuf); // read the rest of the line
    }

  if (scene_type == CALIBDEV_2)
    {
      LOG_INFO << "Loading uncalibrated data with known target detections [CALIBDEV_2]";
    }

  if (scene_type == CALIBDEV_1)
    {
      LOG_INFO << "Loading calibrated data with known target detections [CALIBDEV_1]";
      icam.loadCameraData(bFile);
      getline(bFile, linebuf); // read the rest of the line
    }

  for (i = 0; i < no_cameras; i++)
    {
      if (read_flag)
        getline(bFile, linebuf);

      camera = new OpenCVCamera();
      camera->setIndex(i);
      camera->setCameraModelType((OpenCVCamera::CameraModelType) opts->cmodel_type);

      // is linebuf filename or file sizes?
      if ((linebuf.length() > 0) && (linebuf.c_str()[0] == '@'))
        {
          if (iwidth == -1)
            {
              vector<string> fields;
              boost::split(fields, linebuf, boost::is_any_of(" "), boost::token_compress_on);

              if (fields.size() != 3)
                LOG_FATAL << "Cannot parse image size data: " << linebuf;

              iwidth = boost::lexical_cast<int>(fields[1]);
              iheight = boost::lexical_cast<int>(fields[2]);
            }

          camera->setImageWidth(iwidth);
          camera->setImageHeight(iheight);
          read_flag = false;
        }
      else
        {
          fpath = bfs::path(linebuf).is_absolute() ?
              linebuf : (bfs::path(opts->calibdev_txt.c_str()).parent_path() / linebuf).string();
          camera->setImagePath(fpath);

          if (iwidth == -1)
            {
        	  ci::CImg<unsigned char> img;
              camera->loadImageData(img);
              iwidth = camera->getImageWidth();
              iheight = camera->getImageHeight();
            }
          else
            {
              camera->setImageWidth(iwidth);
              camera->setImageHeight(iheight);
            }
        }

      if ((scene_type == CALIBDEV_1) || (scene_type == CALIBDEV_4))
        camera->copyIntrinsics(&icam);

      cameras.push_back(camera);
    }

  for (i = 0; i < no_tracks; i++)
    {
      track = new CalibdevTrack();
      track->setIndex(i);
      track->loadTrackData(bFile);
      tracks.push_back(track);

      no_views = track->getNumCameras();
      max_cameras = (no_views > max_cameras) ? no_views : max_cameras;
      camera_idx = track->getCamerasIdx();
      for (j = 0; j < no_views; j++)
        cameras[camera_idx[j]]->addTrack(track);
    }

  bFile.close();

  LOG_INFO << "Loaded " << cameras.size() << " cameras";
  LOG_INFO << "Loaded " << tracks.size() << " points";
}

