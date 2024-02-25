/*
 * rdesc_mdh.cpp
 *
 *  Created on: Mar 25, 2014
 *      Author: hellej1
 */

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>
#include <boost/lexical_cast.hpp>
namespace bfs = boost::filesystem;
namespace bs = boost::system;

#include <ceres/dynamic_autodiff_cost_function.h>

#include "rdesc_mdh.h"

void ModifiedDHRobotDescription::addLinearPenalties(ceres::Problem *const problem, const int no_projections) const
{
  for (int i = 0; i < no_links; i++)
     {
       for (int j = 0; j < 4; j++)
         {
           if (pmask(i, j) > 0.0f)
             {
               int idx = 4*i + j;

               ceres::DynamicAutoDiffCostFunction<LinearPenaltyFunctor, 4> *cost_function =
                 new ceres::DynamicAutoDiffCostFunction<LinearPenaltyFunctor, 4>(
                  new  LinearPenaltyFunctor(idx, ptable(i, j), pmask(i, j) * no_projections));
               cost_function->AddParameterBlock(no_links * 4);
               cost_function->SetNumResiduals(1);
               problem->AddResidualBlock(cost_function, NULL, params);
             }
         }
     }
}


int ModifiedDHRobotDescription::getParameters(double* &rparams, vector<int> & rpmask)
{
  int no_params = 0;

  if (pmask.rows() == 0)
    {
      params = NULL;
      rpmask.clear();
    }
  else
    {
      no_params = no_links * 4;
      params = new double[no_params];
      rpmask.clear();

      for (int i = 0; i < no_links; i++)
        {
          for (int j = 0; j < 4; j++)
            {
              int idx = 4*i + j;
              params[idx] = ptable(i, j);
              if (!pmask(i, j))
                rpmask.push_back(idx);
            }
        }
    }

  rparams = params;
  return no_params;
}

void ModifiedDHRobotDescription::setParameters(const double * const rparams)
{
  if (rparams)
    {
      if (rparams != params)
        LOG_FATAL << "Parameter array passed to ModifiedDHRobotDescription::setParameters unknown";

      for (int i = 0; i < no_links; i++)
        {
          for (int j = 0; j < 4; j++)
            {
              int idx = 4*i + j;
              ptable(i, j) = rparams[idx];
            }
        }

      delete [] params;
    }
}

inline Matrix4f ModifiedDHRobotDescription::trnMatrix(const AxisType axis, const bool iflag, const double d) const
{
  Matrix4f t = Matrix4f::Identity();

  if (!iflag)
    {
      if (axis == X)
        t(0,3) = d;
      else if (axis == Y)
        t(1,3) = d;
      else if (axis == Z)
        t(2,3) = d;
    }
  else
    {
      if (axis == X)
        t(0,3) = -d;
      else if (axis == Y)
        t(1,3) = -d;
      else if (axis == Z)
        t(2,3) = -d;
    }

  return t;
}

inline Matrix4f ModifiedDHRobotDescription::rotMatrix(const AxisType axis, const bool iflag, const double a) const
{
  Matrix4f r = Matrix4f::Identity();

  if (!iflag)
    {
      if (axis == X)
        {
          r(1,1) =  std::cos(a);
          r(1,2) = -std::sin(a);
          r(2,1) =  std::sin(a);
          r(2,2) =  std::cos(a);
        }
      else if (axis == Y)
        {
          r(0,0) =  std::cos(a);
          r(0,2) =  std::sin(a);
          r(2,0) = -std::sin(a);
          r(2,2) =  std::cos(a);
        }
      else if (axis == Z)
        {
          r(0,0) =  std::cos(a);
          r(0,1) = -std::sin(a);
          r(1,0) =  std::sin(a);
          r(1,1) =  std::cos(a);
        }
    }
  else
    {
      if (axis == X)
        {
          r(1,1) =  std::cos(a);
          r(1,2) =  std::sin(a);
          r(2,1) = -std::sin(a);
          r(2,2) =  std::cos(a);
        }
      else if (axis == Y)
        {
          r(0,0) =  std::cos(a);
          r(0,2) = -std::sin(a);
          r(2,0) =  std::sin(a);
          r(2,2) =  std::cos(a);
        }
      else if (axis == Z)
        {
          r(0,0) =  std::cos(a);
          r(0,1) =  std::sin(a);
          r(1,0) = -std::sin(a);
          r(1,1) =  std::cos(a);
        }
    }

  return r;
}

Matrix4f ModifiedDHRobotDescription::getMatrix(const VectorXf &jparams) const
{
  if ((int) jparams.size() != no_links)
    LOG_FATAL << "Cannot compute matrix: number of parameters (" << jparams.size() << ") "
      << "does not match number of links (" << no_links << ")";

  Matrix4f T = Matrix4f::Identity();
  bool iflag;

  for (int i = 0; i < no_links; i++)
    {
      iflag = inverse_flags[i];
      if (!mdh_flags[i])
        {  // DH
          if (!iflag)
            { // DH
              if (link_types[i] == REVOLUTE)
                {
                  T = T *
                    rotMatrix(Z, iflag, ptable(i,2) + jparams[i]) *
                    trnMatrix(Z, iflag, ptable(i,3)) *
                    trnMatrix(X, iflag, ptable(i,1)) *
                    rotMatrix(X, iflag, ptable(i,0));
                }
              else if (link_types[i] == PRISMATIC)
                {
                  T = T *
                    rotMatrix(Z, iflag, ptable(i,2)) *
                    trnMatrix(Z, iflag, ptable(i,3) + jparams[i]) *
                    trnMatrix(X, iflag, ptable(i,1)) *
                    rotMatrix(X, iflag, ptable(i,0));
                }
            }
          else
            { // inverse DH
              if (link_types[i] == REVOLUTE)
                {
                  T = T *
                    rotMatrix(X, iflag, ptable(i,0)) *
                    trnMatrix(X, iflag, ptable(i,1)) *
                    trnMatrix(Z, iflag, ptable(i,3)) *
                    rotMatrix(Z, iflag, ptable(i,2) + jparams[i]);
                }
              else if (link_types[i] == PRISMATIC)
                {
                  T = T *
                    rotMatrix(X, iflag, ptable(i,0)) *
                    trnMatrix(X, iflag, ptable(i,1)) *
                    trnMatrix(Z, iflag, ptable(i,3) + jparams[i]) *
                    rotMatrix(Z, iflag, ptable(i,2));
                }

            }
        }
      else
        { // MDH
          if (!iflag)
            {
              if (link_types[i] == REVOLUTE)
                {
                  T = T *
                    rotMatrix(Z, iflag, ptable(i,2) + jparams[i]) *
                    trnMatrix(X, iflag, (double) ptable(i,3)) *
                    rotMatrix(X, iflag, ptable(i,0)) *
                    rotMatrix(Y, iflag, ptable(i,1));
                }
              else if (link_types[i] == PRISMATIC)
                {
                  T = T *
                    rotMatrix(Z, iflag, ptable(i,2)) *
                    trnMatrix(X, iflag, ptable(i,3) + jparams[i]) *
                    rotMatrix(X, iflag, ptable(i,0)) *
                    rotMatrix(Y, iflag, ptable(i,1));
                }
            }
          else
            {
              if (link_types[i] == REVOLUTE)
                {
                  T = T *
                    rotMatrix(Y, iflag, ptable(i,1)) *
                    rotMatrix(X, iflag, ptable(i,0)) *
                    trnMatrix(X, iflag, ptable(i,3)) *
                    rotMatrix(Z, iflag, ptable(i,2) + jparams[i]);
                }
              else if (link_types[i] == PRISMATIC)
                {
                  T = T *
                    rotMatrix(Y, iflag, ptable(i,1)) *
                    rotMatrix(X, iflag, ptable(i,0)) *
                    trnMatrix(X, iflag, ptable(i,3) + jparams[i]) *
                    rotMatrix(Z, iflag, ptable(i,2));
                }
            }
        }
    }
  return T;
}

void ModifiedDHRobotDescription::loadMask(ifstream &iFile)
{
  int nolinks;
  string line;
  vector<string> fields;

  iFile >> nolinks;

  if (nolinks != no_links)
    LOG_FATAL << "The number of links in the robot parameter mask (" << nolinks
      << ") does not match the number of links in the robot description (" << no_links << ")";

  pmask = Eigen::MatrixXd(no_links, 4);

  for (int i = 0; i < no_links; i++)
    {
      do {
          getline(iFile, line);
      } while (line == "");

      boost::split(fields, line, boost::is_any_of(" "), boost::token_compress_on);

      if ((int) fields.size() != 4)
        LOG_FATAL << "Cannot parse robot mask: the number of parameters(" << fields.size()
          << ") does not match the number of expected parameters (4): " << line;

      for (int j = 0; j < 4; j++)
        pmask(i, j) = boost::lexical_cast<double>(fields[j]);
    }
}

void ModifiedDHRobotDescription::load(ifstream &iFile)
{
  double mdhflag, iflag, ltype;
  string line;
  vector<string> fields;

  iFile >> no_links;

  ptable = Eigen::MatrixXd(no_links, 4);
  mdh_flags.reserve(no_links);
  inverse_flags.reserve(no_links);
  link_types.reserve(no_links);

  for (int i = 0; i < no_links; i++)
    {
      do {
          getline(iFile, line);
      } while (line == "");

      boost::split(fields, line, boost::is_any_of(" "), boost::token_compress_on);

      if ((int) fields.size() != 7)
        LOG_FATAL << "Cannot parse robot description: the number of parameters(" << fields.size()
          << ") does not match the number of expected parameters (7): " << line;

      mdhflag = boost::lexical_cast<double>(fields[0]);
      iflag = boost::lexical_cast<double>(fields[1]);
      ltype = boost::lexical_cast<double>(fields[2]);

      mdh_flags[i] = (mdhflag != 0.0) ? true : false;
      inverse_flags[i] = (iflag != 0.0) ? true : false;

      if ((LinkType) (int) ltype == REVOLUTE)
        link_types[i] = REVOLUTE;
      else if ((LinkType) (int) ltype == PRISMATIC)
        link_types[i] = PRISMATIC;
      else
        LOG_FATAL << "Unknown link type " << ltype << " for link no. " << i;

      for (int j = 0; j < 4; j++)
        ptable(i, j) =  boost::lexical_cast<double>(fields[j + 3]);
    }
}

void ModifiedDHRobotDescription::save(const string &path) const
{
  bfs::path fpath(path);
  bs::error_code ec;

  LOG_INFO << "Saving Modified DH table to " << fpath.string();

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
  for (int i = 0; i < no_links; i++)
      rFile << mdh_flags[i] << " " << inverse_flags[i] << " " << link_types[i] << " "
            << ptable(i, 0) << " " << ptable(i, 1) << " " << ptable(i, 2) << " " << ptable(i, 3) << endl;

  rFile.close();
}
