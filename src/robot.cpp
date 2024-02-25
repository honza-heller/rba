/*
 * robot_load.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: hellej1
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
#include "robot.h"
#include "rpose_nonpar.h"
#include "rpose_mdh.h"
#include "rdesc_mdh.h"

void Robot::load(void)
{
  if (!opts)
    LOG_FATAL << "[Robot::load] Options not set, don't known what to do";

  if (opts->rdesc_txt != "")
    loadRobotDescription();

  if (opts->rdmask_txt != "")
    {
      if (rdesc == NULL)
        LOG_FATAL << "Received robot mask file without robot description file";
      else
        loadRobotMask();
    }

  if (opts->rposes_txt != "")
    loadRobotPoses();
}

void Robot::loadRobotMask(void)
{
  int rdmask_type;

  LOG_INFO << "Loading robot mask rdmask.txt file: " << opts->rdmask_txt;

  ifstream bFile(opts->rdmask_txt.c_str());
  if (!bFile)
    LOG_FATAL << "Cannot open robot mask rdmask.txt file: " << opts->rdmask_txt;

  bFile >> rdmask_type;

  if (RobotDescription::RobotType(rdmask_type) == rdesc->getType())
    rdesc->loadMask(bFile);
  else
    LOG_FATAL << "Robot mask type (" << rdmask_type <<
    ") does not match robot description type (" << rdesc->getType() << ")";

  bFile.close();

}

void Robot::loadRobotDescription(void)
{
  int rdesc_type;

  LOG_INFO << "Loading robot description rdesc.txt file: " << opts->rdesc_txt;

  ifstream bFile(opts->rdesc_txt.c_str());
  if (!bFile)
    LOG_FATAL << "Cannot open robot description rdesc.txt file: " << opts->rdesc_txt;

  bFile >> rdesc_type;

  if (RobotDescription::RobotType(rdesc_type) == RobotDescription::MODIFIED_DENAVITHARTENBERG)
    {
      rdesc = new ModifiedDHRobotDescription();
      rdesc->load(bFile);
    }
  else
    {
      LOG_FATAL << "Unknown robot description type: " << rdesc_type;
    }

  bFile.close();
}

void Robot::loadRobotPoses(void)
{
  int i, pose_type;
  string pose_fname, img_fname, rfunc_name;
  RobotPose *pose = NULL;

  LOG_INFO << "Loading robot poses rposes.txt file: " << opts->rposes_txt;

  ifstream bFile(opts->rposes_txt.c_str());
  if (!bFile)
    LOG_FATAL << "Cannot open robot poses rposes.txt file: " << opts->rposes_txt;

  bFile >> no_poses >> pose_type;

  poses.clear();
  poses.reserve(no_poses);

  if (RobotPose::RobotPoseType(pose_type) == RobotPose::NONPARAMETRIC)
    {
      for (i = 0; i < no_poses; i++)
        {
          pose = new NonParametricRobotPose();
          pose->loadPoseData(bFile);
          pose->setIndex(i);
          poses.push_back(pose);
        }
    }
  else if (RobotPose::RobotPoseType(pose_type) == RobotPose::MODIFIED_DENAVITHARTENBERG)
    {
      if (rdesc == NULL)
        LOG_FATAL << "Robot description for pose type MODIFIED_DENAVITHARTENBERG not loaded";
      if (rdesc->getType() != RobotDescription::MODIFIED_DENAVITHARTENBERG)
        LOG_FATAL << "Robot description file type does not correspond to pose type MODIFIED_DENAVITHARTENBERG";

      for (i = 0; i < no_poses; i++)
        {
          pose = new ModifiedDHRobotPose((ModifiedDHRobotDescription *) rdesc);
          pose->loadPoseData(bFile);
          pose->setIndex(i);
          poses.push_back(pose);
        }
    }
  else
    {
      LOG_FATAL << "Unknown pose type in rposes.txt: " << opts->rposes_txt;
    }

  bFile.close();

  if (no_poses == 0)
    LOG_FATAL << "Zero poses loaded from robot poses file rposes.txt file: " << opts->rposes_txt;
  else
    LOG_INFO << "Loaded " << no_poses << " robot poses";
}

void Robot::save(void) const
{
  if (opts->rierrs_res_txt != "")
    {
      if (opts->calib_relpose)
        LOG_WARN << "Cannot output residual errors while using relative poses for BA (calib_relpose==1)'";
      else
        saveRobotResidualError(opts->rierrs_res_txt, Track::ROBOT_IMAGE_SPACE_RESIDUAL);
    }

  if (opts->roerrs_res_txt != "")
    {
      if (opts->calib_relpose)
        LOG_WARN << "Cannot output residual errors while using relative poses for BA (calib_relpose==1)'";
      else
        saveRobotResidualError(opts->roerrs_res_txt, Track::ROBOT_OBJECT_SPACE_RESIDUAL);
    }

  if (opts->rdesc_res_txt != "")
    {
      if (rdesc == NULL)
        LOG_WARN << "Cannot output robot description, because no robot description was loaded";
      else
        rdesc->save(opts->rdesc_res_txt);
    }
}

void Robot::saveRobotResidualError(const string & path, const Track::ResidualError rtype) const
{
  bfs::path fpath(path);
  bs::error_code ec;
  Track *track;
  int *cams;
  vector<Track*> tracks;
  int no_tracks;

  LOG_INFO << "Saving robot residual errors to " << fpath.string();

  if (!scene)
    LOG_FATAL << "Cannot output residuals, have no scene pointer";

  tracks = scene->getTracks();
  no_tracks = tracks.size();

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

  rFile << no_poses << " " << no_tracks << " " << 0 << endl;
  rFile << std::setprecision(20) << std::scientific;

  for (int i = 0; i < no_tracks; i++)
    {
      track = tracks[i];
      cams = track->getCamerasIdx();

      for (int j = 0; j < track->getNumCameras(); j++)
        rFile << cams[j] << " " << i << " " << track->getResidualError(j, rtype, true) << endl;
    }

  rFile.close();
}
