/*
 * options.h
 *
 *  Created on: Feb 24, 2014
 *      Author: hellej1
 */

#ifndef OPTIONS_H_
#define OPTIONS_H_

#include <string>
#include <vector>
#include <list>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <stdexcept>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
namespace po = boost::program_options;

#include "logging.h"

using namespace std;

class RbaOptions {
public:
  // input paths
  string               calibdev_txt;
  string               cposes_txt;
  string               rposes_txt;
  string               rdesc_txt;
  string               rdmask_txt;
  string               cinit_txt;
  // output paths
  string               cposes_res_txt;
  string               cmodel_res_txt;
  string               crerrs_res_txt;
  string               rierrs_res_txt;
  string               roerrs_res_txt;
  string               target_res_txt;
  string               tidets_res_txt;
  string               rdesc_res_txt;
  string               calib_res_txt;
  string               report_dir;

  vector<int>          cmodel_mask;
  int                  no_threads;
  vector<int>          xvalid_cameras;
  int                  crecoef;
  int                  rrecoef;

  int                  cmodel_type;
  bool                 ocvcalib_linear;
  bool                 ocvcalib;
  bool                 ba_cposes;
  bool                 ba_scene;
  bool                 ba_hec;
  bool                 ba_wbc;
  bool                 ba_scale;
  bool                 calib_ospace;
  bool                 calib_relpose;
  bool                 rdmask_is_penalty;

  int                  target_maxe;
  int                  target_mina;
  int                  target_maxa;
  int                  target_delta;
  int                  target_maxd;
  int                  target_maxf;
  int                  target_noup;
  int                  target_mindets;
  string               target_detdir;

  bool                 verbose;

private:
  typedef enum {
    GENERAL = 0,
    CMD_ONLY = 1,
    CFG_ONLY = 2
  } OptionType;


  // list of options
  vector<bool*>        bool_opts;
  vector<OptionType>   bool_opts_type;
  vector<string>       bool_opts_name;
  vector<string>       bool_opts_info;
  vector<bool>         bool_opts_ini;

  vector<int*>         int_opts;
  vector<OptionType>   int_opts_type;
  vector<string>       int_opts_name;
  vector<string>       int_opts_info;
  vector<int>          int_opts_ini;

  vector<string*>      string_opts;
  vector<OptionType>   string_opts_type;
  vector<string>       string_opts_name;
  vector<string>       string_opts_info;
  vector<string>       string_opts_ini;

  vector<vector<int>*>   vint_opts;
  vector<OptionType>     vint_opts_type;
  vector<string>         vint_opts_name;
  vector<string>         vint_opts_info;
  vector<string>         vint_opts_ini;
  vector<pair<int,int> > vint_opts_bnds;

  // boost options
  po::options_description cmdline_options;
  po::options_description cfgfile_options;
  po::variables_map       vm;
  string                  cfgfile;


  // Helper methods

  void static str2vint(const string &str, vector<int> * const vint) {
    vector<string> fields;

    if (str == "")
      {
        vint->clear();
        return;
      }

    boost::split(fields, str, boost::is_any_of( "," ), boost::token_compress_on);

    vint->clear();
    for (int i = 0; i < (int) fields.size(); i++)
      if (fields[i].length() > 0)
        vint->push_back(boost::lexical_cast<int>(fields[i].c_str()));
  }

  void registerBln(bool &bopt, const OptionType &type, const string &name, const string &info, bool inivalue) {
    bool_opts.push_back(&bopt);
    bool_opts_type.push_back(type);
    bool_opts_name.push_back(name);
    bool_opts_info.push_back(info);
    bool_opts_ini.push_back(inivalue);

    bopt = inivalue;
  };

  void registerInt(int &iopt, const OptionType &type, const string &name, const string &info, int inivalue) {
    int_opts.push_back(&iopt);
    int_opts_type.push_back(type);
    int_opts_name.push_back(name);
    int_opts_info.push_back(info);
    int_opts_ini.push_back(inivalue);

    iopt = inivalue;
  };

  void registerStr(string &sopt, const OptionType &type, const string &name, const string &info, const string &inivalue) {
    string_opts.push_back(&sopt);
    string_opts_type.push_back(type);
    string_opts_name.push_back(name);
    string_opts_info.push_back(info);
    string_opts_ini.push_back(inivalue);

    sopt = inivalue;
  };

  void registerVnt(vector<int> &viopt, const OptionType &type, const string &name, const string &info, const int mins, const int maxs, const string &inivalue)
  {
    vint_opts.push_back(&viopt);
    vint_opts_type.push_back(type);
    vint_opts_name.push_back(name);
    vint_opts_info.push_back(info);
    vint_opts_ini.push_back(inivalue);
    vint_opts_bnds.push_back(pair<int,int>(mins, maxs));

    str2vint(inivalue, &viopt);
    if (((mins > 0) && ((int)viopt.size() < mins)) || ((maxs > 0) && ((int)viopt.size() > maxs)))
      LOG_FATAL << "Initial vector length out of bounds: " << viopt.size() << " [" << mins << ", " << maxs << "]";
  };

  void updateOptions(void) {
    po::options_description opts_shared("RBA options");
    po::options_description opts_cmd("Command line options only");
    po::options_description opts_cfg("Config file options only");
    po::options_description *opts[] = {&opts_shared, &opts_cmd, &opts_cfg};

    opts_cmd.add_options()
      ("help,h", "produce help message")
      ("version", "print version string")
      ("config,c", po::value<string>(&cfgfile)->default_value(""),
         "name of a configuration file")
    ;

    for (int i = 0; i < (int) bool_opts.size(); i++)
      {
        opts[bool_opts_type[i]]->add_options()
            (bool_opts_name[i].c_str(),
             po::value<bool>()->implicit_value(true),
             bool_opts_info[i].c_str())
        ;
      }

    for (int i = 0; i < (int) int_opts.size(); i++)
      {
        opts[int_opts_type[i]]->add_options()
            (int_opts_name[i].c_str(),
             po::value<int>(int_opts[i])->default_value(int_opts_ini[i]),
             int_opts_info[i].c_str())
        ;
      }

    for (int i = 0; i < (int) string_opts.size(); i++)
      {
        opts[string_opts_type[i]]->add_options()
            (string_opts_name[i].c_str(),
             po::value<string>(string_opts[i])->default_value(string_opts_ini[i].c_str()),
             string_opts_info[i].c_str())
        ;
      }

    for (int i = 0; i < (int) vint_opts.size(); i++)
      {
        opts[vint_opts_type[i]]->add_options()
            (vint_opts_name[i].c_str(),
             po::value<string>()->default_value(vint_opts_ini[i].c_str()),
             vint_opts_info[i].c_str())
        ;
      }

    cmdline_options.add(opts_cmd).add(opts_shared);
    cfgfile_options.add(opts_cfg).add(opts_shared);
  }

  void storeOptions(void) {
    // Store options that are not parsed automatically by boost
    for (int i = 0; i < (int) bool_opts.size(); i++)
      {
        if (vm.count(bool_opts_name[i]))
          {
            if (vm[bool_opts_name[i]].as<bool>())
              *(bool_opts[i]) = true;
            else
              *(bool_opts[i]) = false;
          }
      }

    for (int i = 0; i < (int) vint_opts.size(); i++)
      {
        if (vm.count(vint_opts_name[i]))
          {
            str2vint(vm[vint_opts_name[i]].as<string>(), vint_opts[i]);
            int mins = vint_opts_bnds[i].first;
            int maxs = vint_opts_bnds[i].second;
            if (((mins > 0) && ((int)vint_opts[i]->size() < mins)) || ((maxs > 0) && ((int)vint_opts[i]->size() > maxs)))
              LOG_FATAL << "Initial vector length out of bounds: " << vint_opts[i]->size() << " [" << mins << ", " << maxs << "]";
          }
      }
  }

public:
  RbaOptions() {

    // Input files
    registerStr(calibdev_txt, GENERAL, "calibdev_txt", "Path to calibdev.txt file", "");
    registerStr(cposes_txt, GENERAL, "cposes_txt", "Path to camera_poses.txt file", "");
    registerStr(rposes_txt, GENERAL, "rposes_txt", "Path to robot_poses.txt file", "");
    registerStr(rdesc_txt, GENERAL, "rdesc_txt", "Path to robot_description.txt file", "");
    registerStr(rdmask_txt, GENERAL, "rdmask_txt", "Path to robot_description_mask.txt file", "");
    registerStr(cinit_txt, GENERAL, "cinit_txt", "Path to calibration initialization cinit.txt file", "");

    // Output files
    registerStr(cposes_res_txt, GENERAL, "cposes_res_txt", "Path to a file where the camera poses should be output.", "");
    registerStr(cmodel_res_txt, GENERAL, "cmodel_res_txt", "Path to a file where the camera model calibration should be output.", "");
    registerStr(crerrs_res_txt, GENERAL, "crerrs_res_txt", "Path to a file where the camera residual errors should be output.", "");
    registerStr(rierrs_res_txt, GENERAL, "rierrs_res_txt", "Path to a file where the robot image space residual errors should be output.", "");
    registerStr(roerrs_res_txt, GENERAL, "roerrs_res_txt", "Path to a file where the robot object space residual errors should be output.", "");
    registerStr(target_res_txt, GENERAL, "target_res_txt", "Path to a file where the target 3D coordinates should be output.", "");
    registerStr(tidets_res_txt, GENERAL, "tidets_res_txt", "Path to a file where the target image detections should be output.", "");
    registerStr(rdesc_res_txt,  GENERAL, "rdesc_res_txt", "Path to a file where the robot description should be output.", "");
    registerStr(calib_res_txt,  GENERAL, "calib_res_txt", "Path to a file where the calibration should be output.", "");

    // HTML calibration report
    registerStr(report_dir,  GENERAL, "report_dir", "Path to a directory where the HTML report should be output.", "");
    registerInt(crecoef, GENERAL, "crecoef", "Multiplication coefficient for camera residual error representation", 10);
    registerInt(rrecoef, GENERAL, "rrecoef", "Multiplication coefficient for robot residual error representation", 10);

    // Camera parameters
    registerVnt(cmodel_mask, GENERAL, "cmodel_mask", "Camera model mask", 8, 8, "1,1,0,0,0,0,1,0");
    registerVnt(xvalid_cameras, GENERAL, "xvalid_cameras", "Indices of cameras to be used for cross-validation", 0, -1, "");
    registerInt(cmodel_type, GENERAL, "cmodel_type", "Camera model to use/interpret. 0 == opencv, 1 == cubic_rational, 2 == fisheye_d3p, 3 == fisheye_p8p, 4 == fisheye_e7p", 0);
    registerBln(ocvcalib_linear, GENERAL, "ocvcalib_linear", "Do not estimate radial and tangential distortion in OpenCV calibration step", false);
    registerBln(ocvcalib, GENERAL, "ocvcalib", "Force OpenCV calibration", false);

    // Bundle adjustment parameters
    registerBln(ba_cposes, GENERAL, "ba_cposes", "Run BA on separate camera poses", true);
    registerBln(ba_scene, GENERAL, "ba_scene", "Run BA on all camera parameters", true);
    registerBln(ba_hec, GENERAL, "ba_hec", "Run BA on Hand-Eye transformation", true);
    registerBln(ba_wbc, GENERAL, "ba_wbc", "Run BA on World-Base transformation", true);
    registerBln(ba_scale, GENERAL, "ba_scale", "Run BA on Camera-Robot scale", true);

    // Calibration
    registerBln(calib_ospace, GENERAL, "calib_ospace", "Use object space error in calibration BA", false);
    registerBln(calib_relpose, GENERAL, "calib_relpose", "Use relative poses (no Z) in calibration BA", false);
    registerBln(rdmask_is_penalty, GENERAL, "rdmask_is_penalty", "Interpret positive values in rdmask_txt as penalty weights", false);

    // ElMark Target
    registerInt(target_maxe, GENERAL, "target_maxe", "Maximal error of ellipse fit [px], -1 == auto", -1);
    registerInt(target_delta, GENERAL, "target_delta", "Contrast parameter", 30);
    registerInt(target_mina, GENERAL, "target_mina", "Minimal area of a detected dot [px], -1 == auto", -1);
    registerInt(target_maxa, GENERAL, "target_maxa", "Maximal area of a detected dot [px], -1 == auto", -1);
    registerInt(target_maxd, GENERAL, "target_maxd", "Maximal distance between neighboring dots [px], -1 == auto", -1);
    registerInt(target_noup, GENERAL, "target_noup", "Number of detection update loops", 0);
    registerInt(target_mindets, GENERAL, "target_mindets", "Minimal number of detections to consider target correctly reconstructed", 20);
    registerStr(target_detdir, GENERAL, "target_detdir", "Path to directory where target detection images should be output", "");

    // General
    registerBln(verbose, CMD_ONLY, "verbose", "Print verbose runtime information", false);
    registerInt(no_threads, GENERAL, "no_threads", "Number of threads", std::max(boost::thread::hardware_concurrency(),(unsigned int)1));

    updateOptions();
  }

  void parseFromCfgFile(const string &cfg_file) {
    try
      {
        ifstream ifs(cfg_file.c_str());
        if (!ifs)
          LOG_FATAL << "Cannot open configuration file: " << cfg_file;

        vm.clear();
        po::store(parse_config_file(ifs, cfgfile_options), vm);
        po::notify(vm);

        storeOptions();
      }
    catch(exception& e)
      {
        LOG_FATAL << e.what();
      }
    catch(...)
      {
        LOG_FATAL << "Exception of unknown type while parsing command line / config file";
      }
  }

  bool parseFromCmdLine(const int ac, const char * const av[]) {
    vm.clear();
    cfgfile = "";
    po::store(po::parse_command_line(ac, av, cmdline_options), vm);
    po::notify(vm);

    if (cfgfile != "")
      {
        ifstream ifs(cfgfile.c_str());
        if (!ifs)
          LOG_FATAL << "Cannot open configuration file: " << cfgfile;
        po::store(parse_config_file(ifs, cfgfile_options), vm);
        po::notify(vm);
      }

    if (vm.count("help") || (ac == 1)) {
        cout << "This is RBA, v" << RBA_VERSION << endl;
        cout << cmdline_options << "\n";
        return true;
    }

    if (vm.count("version")) {
        cout << RBA_VERSION << endl;
        return true;
    }

    storeOptions();
    return false;
  }
};


#endif /* OPTIONS_H_ */
