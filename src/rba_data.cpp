/*
 * rba.cpp
 *
 *  Created on: Mar 16, 2014
 *      Author: jheller
 */

#include "definitions.h"
#include "rba.h"
#include "camera.h"
#include "rpose.h"

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <boost/lexical_cast.hpp>
namespace bfs = boost::filesystem;
namespace bs = boost::system;
using namespace std;


void RoboticBundleAdjuster::loadInitialCalibration(void)
{
  int no_calib, calib_type, i;

  if (opts.cinit_txt != "")
    {
      LOG_INFO << "Loading calibration initialization cinit.txt file: " << opts.cinit_txt;

      ifstream bFile(opts.cinit_txt.c_str());
      if (!bFile)
        LOG_FATAL << "Cannot open calibration initialization cinit.txt file: " << opts.cinit_txt;

      bFile >> no_calib;

      for (i = 0; i < no_calib; i++)
        {
          bFile >> calib_type;

          if (IniCalibType(calib_type) == HANDEYE)
            {
              hec.loadRt(bFile);
              hec_init = true;
            }
          else if (IniCalibType(calib_type) == WORLDBASE)
            {
              wbc.loadRt(bFile);
              wbc_init = true;
            }
          else if (IniCalibType(calib_type) == SCALE)
            {
              bFile >> scale;
              scale_init = true;
            }
        }

      bFile.close();
    }
}

void RoboticBundleAdjuster::load(void)
{
  scene.load();
  robot.load();
  loadInitialCalibration();
}

void RoboticBundleAdjuster::saveCalibration(void) const
{
  bfs::path fpath(opts.calib_res_txt);
  bs::error_code ec;

  LOG_INFO << "Saving calibration to " << fpath.string();

  if (exists(fpath, ec))
    {
      if (is_directory(fpath, ec))
        LOG_FATAL << "Not a regular file: " << fpath.string() << ": " << ec.message();
      else if (is_regular_file(fpath))
          LOG_WARN << "File already exists and will be rewritten: " << fpath.string();
    }

  ofstream oFile(fpath.c_str());
  oFile << std::setprecision(20) << std::scientific;

  hec.writeRt(oFile);
  wbc.writeRt(oFile);
  oFile << scale << " 0.0 0.0" << endl;

  oFile.close();
}

void RoboticBundleAdjuster::save(void)
{
  scene.save();
  robot.save();

  if (opts.calib_res_txt != "")
    saveCalibration();

  if (opts.report_dir != "")
    saveHTMLReport();
}
