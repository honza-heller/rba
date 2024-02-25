/*
 * rba.h
 *
 *  Created on: Mar 16, 2014
 *      Author: jheller
 */

#ifndef RBA_H_
#define RBA_H_

#if defined(_OPENMP)
   #include <omp.h>
#endif

#include <boost/filesystem.hpp>

#include <ceres/ceres.h>

#include "definitions.h"
#include "etransform.h"
#include "scene.h"
#include "robot.h"
#include "datastats.h"

#include "CImg.h"
namespace ci = cimg_library;

class RoboticBundleAdjuster {
  typedef enum {
    HANDEYE = 1,
    WORLDBASE = 2,
    SCALE = 3,
  } IniCalibType;

  Scene scene;
  Robot robot;

  ETrans hec;
  ETrans wbc;
  double scale;

  bool hec_init;
  bool wbc_init;
  bool scale_init;

  RbaOptions opts;

  ceres::Solver::Options ceres_options;
  ceres::Solver::Summary ceres_summary;

  void loadInitialCalibration(void);
  void saveCalibration(void) const;

  void saveHTMLReport(void);
  void getResidualStatistics(vector<DataStats> &, vector<DataStats> &, DataStats &, DataStats &, DataStats &, DataStats &, const bool);
  void outputHistogram(ofstream &, DataStats &, DataStats &, const string &, const string &, const string &, const string &, const string &);
  void outputHistogram(ofstream &, DataStats &, const string &, const string &, const string &, const string &);
  void outputHistogramDiv(ofstream &, const string &, const int, const int);
  void outputImageTag(ofstream &, const string &, const string &, const int);
  void outputStatsTable(ofstream &, const DataStats * const, const char * const, const DataStats * const, const char * const);
  void outputMathMLMatrix(ofstream &, const char * const, const Eigen::MatrixXd &, const int);
  void generateResidualsImages(const boost::filesystem::path &, const bool);

  void recoverInitCalibAbsolutePose(void);
  void recoverInitCalibRelativePose(void);
  void bundleAdjustCalibration(void);
  void computeRelativePoses(list<ETrans> &, list<ETrans> &, const bool) const;
  void computeTsai89Calibration(const list<ETrans> &, const list<ETrans> &, ETrans &, double &, const bool) const;

public:

  RoboticBundleAdjuster() {
    hec_init = false;
    wbc_init = false;
    scale_init = false;
    scale = 0;

    ceres_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    ceres_options.num_threads = opts.no_threads;
    //ceres_options.num_linear_solver_threads = opts.no_threads;
    ceres_options.use_nonmonotonic_steps = true;
    ceres_options.minimizer_progress_to_stdout = false;
  }

  bool init(const int argc, const char * const argv[]) {
    bool rflag = opts.parseFromCmdLine(argc, argv);

    if (opts.verbose)
      SimpleLogger::setLevel(SimpleLogger::INFO);

    ci::cimg::exception_mode(0);

    ceres_options.num_threads = opts.no_threads;
    //ceres_options.num_linear_solver_threads = opts.no_threads;

#if defined(_OPENMP)
    omp_set_num_threads(opts.no_threads);
#endif

    scene.setOptions(&opts);
    robot.setOptions(&opts);
    return rflag;
  }

  void load(void);
  void save(void);
  void calibrate(void);
};



#endif /* RBA_H_ */
