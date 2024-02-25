#include <math.h>

#include "rba.h"
#include "track.h"
#include "datastats.h"
#include "camera_opencv.h"

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <boost/lexical_cast.hpp>
namespace bfs = boost::filesystem;
namespace bs = boost::system;

#include "CImg.h"
namespace ci = cimg_library;

static const char *html1 =
    "<!DOCTYPE HTML>\n<html>\n"
    "<head>\n"
    " <title>RBA Calibration Report</title>\n"
    " <script type=\"text/javascript\" src=\"https://www.google.com/jsapi\"></script>\n"
    " <style type=\"text/css\">\n"
    "   table.errtable { border-collapse:collapse; } \n"
    "   table.errtable td, table.errtable th { border:1px solid black;padding:5px; }\n"
    "</style>\n";

static const char *html_hist1 =
    "<script type=\"text/javascript\">\n"
    " google.load(\"visualization\", \"1\", {packages:[\"corechart\"]});\n"
    " google.setOnLoadCallback(drawChart);\n"
    " function drawChart() {\n"
    "  var data = google.visualization.arrayToDataTable([\n";

void RoboticBundleAdjuster::outputMathMLMatrix(ofstream &htmlFile, const char * const name, const Eigen::MatrixXd &m, const int prec = 20)
{
  htmlFile << "<math xmlns=\"http://www.w3.org/1998/Math/MathML\"><mrow>\n";
  htmlFile << "<mi>" << name <<  "</mi><mo> = </mo><mo> ( </mo><mtable>\n";

  for (int i = 0; i < m.rows(); i++)
    {
      htmlFile << "<mtr>\n";
      htmlFile << std::setprecision(prec);
      for (int j = 0; j < m.cols(); j++)
          htmlFile << "<mtd> <mn>" << ((double) m(i,j)) << "</mn> </mtd>";
      htmlFile << "</mtr>\n";
    }

  htmlFile << "</mtable><mo> ) </mo></mrow></math>\n";
  htmlFile << "<br/>\n";
}

void RoboticBundleAdjuster::outputHistogramDiv(ofstream &htmlFile, const string & div_name, const int width = 600, const int height = 480)
{
  htmlFile << "<div id=\"" << div_name << "\" style=\"width: " << width << "px; height: " << height << "px;\"></div>\n";
}

void RoboticBundleAdjuster::outputImageTag(ofstream &htmlFile, const string & img_name, const string & img_path, const int width = 640)
{
  htmlFile << "<a href=\"" << img_path << "\"><img style=\"width : " << width << "px\" alt=\"" << img_name << "\" src=\"" << img_path << "\" /></a>\n";
}

void RoboticBundleAdjuster::outputHistogram(ofstream &htmlFile,
                                            DataStats &stats1, DataStats &stats2,
                                            const string &title, const string &xlabel,
                                            const string &legend1, const string &legend2,
                                            const string &id)
{
  if (stats2.getNumData() == 0)
    {
      outputHistogram(htmlFile, stats1, title, xlabel, legend1, id);
      return;
    }

  int hres =std::max(stats1.getHistogramResolution(), stats2.getHistogramResolution());
  stats1.setHistogramResolution(hres);
  stats2.setHistogramResolution(hres);

  double max_err = std::max(stats1.getMaxError(), stats2.getMaxError());
  stats1.updateHistogram(max_err);
  stats2.updateHistogram(max_err);

  const vector<int> &hist1 = stats1.getHistogram();
  const vector<int> &hist2 = stats2.getHistogram();
  const vector<double> &ticks = stats1.getHistogramTicks();

  htmlFile << html_hist1;
  htmlFile << std::setprecision(2);

  htmlFile << "['" << xlabel << "', '" << legend1 << "', '" << legend2 << "']," << endl;
  for (int i = 0; i < (int) hist1.size(); i++)
    htmlFile << "['" << ticks[i] << "'," << hist1[i] << ", " << hist2[i] << "]," << endl;
  htmlFile << "]);" << endl;

  htmlFile << "var options = {" << endl;
  htmlFile << "  title: '" << title << "'," << endl;
  htmlFile << "  hAxis: {title: '" << xlabel << "',  titleTextStyle: {color: 'black'}}," << endl;
  htmlFile << "  colors: ['#00B800', '#CC0000']" << endl;
  htmlFile << "};" << endl;
  htmlFile << "var chart = new google.visualization.AreaChart(document.getElementById('" << id << "'));" << endl;
  htmlFile << "chart.draw(data, options);}" << endl;
  htmlFile << "</script>" << endl;
}

void RoboticBundleAdjuster::outputHistogram(ofstream &htmlFile, DataStats &stats,
                                            const string &title, const string &xlabel,
                                            const string &legend, const string &id)
{
  stats.updateHistogram();
  const vector<int> &hist = stats.getHistogram();
  const vector<double> &ticks = stats.getHistogramTicks();

  htmlFile << html_hist1;
  htmlFile << std::setprecision(2);

  htmlFile << "['" << xlabel << "', '" << legend << "']," << endl;
  for (int i = 0; i < (int) hist.size(); i++)
    htmlFile << "['" << ticks[i] << "'," << hist[i] << "]," << endl;
  htmlFile << "]);" << endl;

  htmlFile << "var options = {" << endl;
  htmlFile << "  title: '" << title << "'," << endl;
  htmlFile << "  hAxis: {title: '" << xlabel << "',  titleTextStyle: {color: 'black'}}," << endl;
  htmlFile << "  colors: ['#00B800']" << endl;
  htmlFile << "};" << endl;
  htmlFile << "var chart = new google.visualization.AreaChart(document.getElementById('" << id << "'));" << endl;
  htmlFile << "chart.draw(data, options);}" << endl;
  htmlFile << "</script>" << endl;
}

void RoboticBundleAdjuster::outputStatsTable(ofstream &htmlFile, const DataStats * const stats1, const char * const legend1,
                                             const DataStats * const stats2 = NULL,  const char * const legend2 = NULL)
{
 htmlFile << "<table class=\"errtable\">" << endl;
 htmlFile << "<tr>" << endl;
 htmlFile << "<td>&nbsp;</td>" << endl;
 htmlFile << "<td><b>" << legend1 <<"</b></td>" << endl;
 if (stats2 && stats2->getNumData())
     htmlFile << "<td><b>" << legend2 << "</b></td>" << endl;
 htmlFile << "</tr>" << endl;

 htmlFile << std::setprecision(4);

 htmlFile << "<tr>" << endl;
 htmlFile << "<td><b>Mean</b></td>" << endl;
 htmlFile << "<td>" << stats1->getMeanError() << "</td>" << endl;
 if (stats2 && stats2->getNumData())
   htmlFile << "<td>" << stats2->getMeanError() << "</td>" << endl;
 htmlFile << "</tr>" << endl;

 htmlFile << "<tr>" << endl;
 htmlFile << "<td><b>Max</b></td>" << endl;
 htmlFile << "<td>" << stats1->getMaxError() << "</td>" << endl;
 if (stats2 && stats2->getNumData())
   htmlFile << "<td>" << stats2->getMaxError() << "</td>" << endl;
 htmlFile << "</tr>" << endl;

 htmlFile << "<tr>" << endl;
 htmlFile << "<td><b>RMS</b></td>" << endl;
 htmlFile << "<td>" << stats1->getRmsError() << "</td>" << endl;
 if (stats2 && stats2->getNumData())
   htmlFile << "<td>" << stats2->getRmsError() << "</td>" << endl;
 htmlFile << "</tr>" << endl;

 htmlFile << std::setprecision(6);

 htmlFile << "</table>" << endl;
}

void RoboticBundleAdjuster::getResidualStatistics(vector<DataStats> & csstats, vector<DataStats> & rsstats,
                                                  DataStats & ccalib_stats, DataStats & cvalid_stats,
                                                  DataStats & rcalib_stats, DataStats & rvalid_stats,
                                                  const bool rflag)
{
  const vector<Camera*> & cameras = scene.getCameras();
  DataStats dstats;

  csstats.resize(cameras.size());
  rsstats.resize(cameras.size());

  ccalib_stats.clear();
  rcalib_stats.clear();
  cvalid_stats.clear();
  rvalid_stats.clear();

  for (int i = 0; i < (int) cameras.size(); i++)
    {
      list<Track*> const &tracks = cameras[i]->getTracks();
      csstats[i].clear();
      rsstats[i].clear();

      for (list<Track*>::const_iterator itrack = tracks.begin(); itrack != tracks.end(); itrack++)
        {
          double cres = (*itrack)->getResidualError(i, Track::CAMERA_RESIDUAL);
          if (cres >= 0.0f)
            {
              if (cameras[i]->isCalibCamera())
                ccalib_stats.add(cres);
              else
                cvalid_stats.add(cres);

              csstats[i].add(cres);
            }

          if (rflag)
            {
              double rres = (*itrack)->getResidualError(i, Track::ROBOT_IMAGE_SPACE_RESIDUAL);
              if (rres >= 0.0f)
                {
                  if (cameras[i]->isCalibCamera())
                    rcalib_stats.add(rres);
                  else
                    rvalid_stats.add(rres);

                  rsstats[i].add(rres);
                }
            }
        }

      csstats[i].updateStatistics();

      if (rflag)
        rsstats[i].updateStatistics();
    }

  ccalib_stats.updateStatistics();
  cvalid_stats.updateStatistics();

 if (rflag)
   {
     rcalib_stats.updateStatistics();
     rvalid_stats.updateStatistics();
   }
}

void RoboticBundleAdjuster::generateResidualsImages(const bfs::path & rdir_path, const bool rflag)
{
  const vector<Camera*> & cameras = scene.getCameras();
  vector<RobotPose*> const & rposes = robot.getPoses();
  const unsigned char red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 };

#pragma omp parallel for
  for (int i = 0; i < (int) cameras.size(); i++)
    {
      Camera *camera = cameras[i];
      bfs::path rimg_path, img_path = bfs::path(camera->getImagePath());
      ci::CImg<unsigned char> img;

      if (bfs::is_regular_file(img_path))
        {
          camera->loadImageData(img);
          if (img.spectrum() == 1)
            {
              img.channels(0, 2);
              img.draw_image(0, 0, 0, 1, img.get_channel(0));
              img.draw_image(0, 0, 0, 2, img.get_channel(0));
            }
        }
      else
        {
          img_path = bfs::path(boost::lexical_cast<string>(i + 1) + ".jpg");
          camera->createImageData(img);
        }

      list<Track*> const &tracks = cameras[i]->getTracks();
      for (list<Track*>::const_iterator itrack = tracks.begin(); itrack != tracks.end(); itrack++)
        {
          Track *track = *itrack;
          Vector2f dt, pr, eline;

          dt = camera->getDetection(track, Track::SOURCE);
          pr = camera->getProjection(track, Track::SOURCEPT, Track::IMAGE);

          eline = (double) opts.crecoef * (pr - dt) + dt;

          img.draw_circle((int) round((double) dt(0)), (int) round((double) dt(1)), 1, blue, 1, 1);
          img.draw_circle((int) round((double) pr(0)), (int) round((double) pr(1)), 1, green, 1, 1);
          img.draw_line((int) round((double) dt(0)), (int) round((double) dt(1)),
        		        (int) round((double) eline(0)), (int) round((double) eline(1)), green);

          if (rflag)
            {
              ETrans w2c(hec.getMatrix() * rposes[i]->getInverseMatrix() * wbc.getMatrix() * Vector4f(scale, scale, scale, 1).asDiagonal());
              pr = camera->getProjection(track, Track::SOURCEPT, Track::IMAGE, &w2c);
              eline = (double) opts.rrecoef * (pr - dt) + dt;

              img.draw_circle((int) round((double) pr(0)), (int) round((double) pr(1)), 1, red, 1, 1);
              img.draw_line((int) round((double) pr(0)), (int) round((double) pr(1)),
            		        (int) round((double) eline(0)), (int) round((double) eline(1)), red);
            }
        }

      rimg_path = rdir_path / img_path.filename();
      rimg_path.replace_extension(".jpg");

      if(bfs::is_regular_file(rimg_path))
        LOG_WARN << "File already exists and will be rewritten: " << rimg_path.string();

      img.save_jpeg(rimg_path.string().c_str(), 85);
      img.clear();
    }
}

void RoboticBundleAdjuster::saveHTMLReport(void)
{
  const vector<Camera*> & cameras = scene.getCameras();
  vector<DataStats> csstats, rsstats;
  DataStats         ccalib_stats, cvalid_stats, rcalib_stats, rvalid_stats;
  bool rflag = true;

  bfs::path rdir_path(opts.report_dir), report_path;
  bs::error_code ec;

  LOG_INFO << "Saving HTML report to " << opts.report_dir;

  if (bfs::is_regular_file(rdir_path))
    LOG_FATAL << "Output path already exists as a regular file: " << rdir_path.string();

  if (!(bfs::exists(rdir_path, ec)))
    {
      if (!bfs::create_directory(rdir_path, ec))
        LOG_FATAL << "Cannot create directory: " << rdir_path.string() << ": " << ec.message();
    }

  if (opts.rposes_txt == "")
    {
      LOG_INFO << "Robot poses not available, only camera calibration statistics will be output";
      rflag = false;
    }

  if (opts.calib_relpose)
    {
      LOG_WARN << "Since 'calib_relpose == true', only camera calibration statistics will be output";
      rflag = false;
    }

  if (opts.calib_ospace && rflag)
    LOG_WARN << "Only image space residuals will be output to HTML report, even though 'calib_ospace == true'";

  report_path = rdir_path / "index.html";
  ofstream htmlFile(report_path.c_str());

  if (bfs::exists(report_path, ec))
    {
      if (bfs::is_directory(report_path, ec))
        LOG_FATAL << "Not a regular file: " << report_path.string() << ": " << ec.message();
      else if (bfs::is_regular_file(report_path))
        LOG_WARN << "File already exists and will be rewritten: " << report_path.string();
    }

  if (!htmlFile)
    LOG_FATAL << "Cannot open file for writing: " << report_path.string();

  generateResidualsImages(rdir_path, rflag);
  getResidualStatistics(csstats, rsstats, ccalib_stats, cvalid_stats, rcalib_stats, rvalid_stats, rflag);

  htmlFile << html1;

  outputHistogram(htmlFile, ccalib_stats, cvalid_stats,
    "Camera Calibration", "Residual error in pixels", "Calibration cameras", "Validation cameras", "camcalib_hist");

  if (rflag)
    outputHistogram(htmlFile, rcalib_stats, rvalid_stats,
      "Calibration", "Residual error in pixels", "Calibration poses", "Validation poses", "calib_hist");

  for (int i = 0; i < (int) cameras.size(); i++)
    outputHistogram(htmlFile, csstats[i], rsstats[i],
      "Residual errors", "Residual error in pixels", "Camera", "Robot",
      "calib" + boost::lexical_cast<string>(i + 1) + "_hist");

  htmlFile << "</head>\n";
  htmlFile << "<body>\n";

  htmlFile << "<center>\n";
  htmlFile << "<h1>Calibration Report</h1>\n";
  htmlFile << "<h2>Created by RBA " << RBA_VERSION << "</h2>\n";
  htmlFile << "</center>\n";

  htmlFile << "<h2>Camera Calibration</h2>\n";
  Matrix3f K;
  Vector8f dist;
  ((OpenCVCamera *) cameras[0])->getIntrinsics(K, dist);
  outputMathMLMatrix(htmlFile, "K", K);
  outputMathMLMatrix(htmlFile, "dist", dist.transpose());

  if (rflag)
    {
      htmlFile << "<h2>Robot Calibration</h2>" << endl;
      outputMathMLMatrix(htmlFile, "X", hec.getMatrix());
      outputMathMLMatrix(htmlFile, "Z", wbc.getMatrix());
      Eigen::MatrixXd sc(1,1);
      sc << scale;
      outputMathMLMatrix(htmlFile, "scale", sc);

      htmlFile << "<h2>Ceres Report</h2>\n";
      htmlFile << "<pre>\n";
      htmlFile << ceres_summary.FullReport();
      htmlFile << "\n</pre>\n";
    }

  htmlFile << "<h2>Histograms</h2>\n";
  htmlFile << "<center><table><tr><td>\n";
  outputStatsTable(htmlFile, &ccalib_stats, "Calibration", &cvalid_stats, "Validation");
  htmlFile << "</td><td>\n";
  outputHistogramDiv(htmlFile, "camcalib_hist", 900);
  htmlFile << "</td></tr></table></center>\n";

  if (rflag)
    {
      htmlFile << "<center><table><tr><td>\n";
      outputStatsTable(htmlFile, &rcalib_stats, "Robot Calibration", &rvalid_stats, "Validation");
      htmlFile << "</td><td>\n";
      outputHistogramDiv(htmlFile, "calib_hist", 900);
      htmlFile << "</td></tr></table></center>\n";
    }

  htmlFile << "<h2>Images</h2>\n";
  htmlFile << "<center>\n";

  for (int i = 0; i < (int) cameras.size(); i++)
    {
      bfs::path img_path = bfs::path(cameras[i]->getImagePath());
      string img_name;

      if (!bfs::is_regular_file(img_path))
        img_path = bfs::path(boost::lexical_cast<string>(i + 1) + ".jpg");
      img_path = rdir_path / img_path.filename();
      img_name = img_path.filename().string();
      img_path.replace_extension(".jpg");

      htmlFile << "<h3>" << img_name << (!(cameras[i]->isCalibCamera()) ? " (x-validation)" : "") << "</h3>\n";
      htmlFile << "<table>\n";
      htmlFile << "<tr><td>\n";
      outputStatsTable(htmlFile, &csstats[i], "Camera", &rsstats[i], "Robot");
      htmlFile << "</td><td>\n";
      outputMathMLMatrix(htmlFile, "A", cameras[i]->getPose()->getMatrix(), 5);
      htmlFile << "</td></tr>\n";

      htmlFile << "<tr><td>\n";
      outputImageTag(htmlFile, img_name, img_path.filename().string());
      htmlFile << "</td><td>\n";
      outputHistogramDiv(htmlFile, "calib" + boost::lexical_cast<string>(i + 1) + "_hist", 600);
      htmlFile << "</td></tr>\n";
      htmlFile << "</table>\n";
    }

  htmlFile << "</body>\n";
  htmlFile << "</html>\n";

  htmlFile.close();
}
