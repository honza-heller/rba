/*
 * rdesc_mdh.h
 *
 *  Created on: Mar 13, 2014
 *      Author: jheller
 */

#ifndef RDESC_MDH_H_
#define RDESC_MDH_H_

#include "rdesc.h"
#include <ceres/ceres.h>

#include <cmath>

class ModifiedDHRobotDescription : public RobotDescription {
public:
  typedef enum {
    REVOLUTE = 0,
    PRISMATIC = 1
  } LinkType;

  typedef enum {
    X = 0,
    Y = 1,
    Z = 2
  } AxisType;

private:
  Eigen::MatrixXd   ptable;
  Eigen::MatrixXd   pmask;

  int               no_links;
  vector<bool>      mdh_flags;
  vector<bool>      inverse_flags;
  vector<LinkType>  link_types;

  inline Matrix4f trnMatrix(const AxisType, const bool, const double) const;
  inline Matrix4f rotMatrix(const AxisType, const bool, const double) const;

  template<typename T> inline void trnPoint(const AxisType axis, const bool iflag, const T d, T * const pt) const {
    if (!iflag)
      {
        if (axis == X)
          pt[0] = pt[0] + d;
        else if (axis == Y)
          pt[1] = pt[1] + d;
        else if (axis == Z)
          pt[2] = pt[2] + d;
      }
    else
      {
        if (axis == X)
          pt[0] = pt[0] - d;
        else if (axis == Y)
          pt[1] = pt[1] - d;
        else if (axis == Z)
          pt[2] = pt[2] - d;
      }
  }

  template<typename T> inline void rotPoint(const AxisType axis, const bool iflag, const T a, T * const pt) const {
    T pp[3];

    if (!iflag)
      {
        if (axis == X)
          {
            pp[0] = pt[0];
            pp[1] = pt[1] * ceres::cos(a) - pt[2] * ceres::sin(a);
            pp[2] = pt[2] * ceres::cos(a) + pt[1] * ceres::sin(a);
          }
        if (axis == Y)
          {
            pp[0] = pt[0] * ceres::cos(a) + pt[2] * ceres::sin(a);
            pp[1] = pt[1];
            pp[2] = pt[2] * ceres::cos(a) - pt[0] * ceres::sin(a);
          }
        if (axis == Z)
          {
            pp[0] = pt[0] * ceres::cos(a) - pt[1] * ceres::sin(a);
            pp[1] = pt[1] * ceres::cos(a) + pt[0] * ceres::sin(a);
            pp[2] = pt[2];
          }
      }
    else
      {
        if (axis == X)
          {
            pp[0] = pt[0];
            pp[1] = pt[1] * ceres::cos(a) + pt[2] * ceres::sin(a);
            pp[2] = pt[2] * ceres::cos(a) - pt[1] * ceres::sin(a);
          }
        if (axis == Y)
          {
            pp[0] = pt[0] * ceres::cos(a) - pt[2] * ceres::sin(a);
            pp[1] = pt[1];
            pp[2] = pt[2] * ceres::cos(a) + pt[0] * ceres::sin(a);
          }
        if (axis == Z)
          {
            pp[0] = pt[0] * ceres::cos(a) + pt[1] * ceres::sin(a);
            pp[1] = pt[1] * ceres::cos(a) - pt[0] * ceres::sin(a);
            pp[2] = pt[2];
          }
      }

    pt[0] = pp[0];
    pt[1] = pp[1];
    pt[2] = pp[2];
  }

  template<typename T> inline void cdhPoint(const LinkType ltype, const bool iflag, const T * const rparams, const double jparam, T * const pt) const {
    if (!iflag)
      { // Classical DH
        if (ltype == REVOLUTE)
          {
            rotPoint(X, iflag, rparams[0], pt);
            trnPoint(X, iflag, rparams[1], pt);
            trnPoint(Z, iflag, rparams[3], pt);
            rotPoint(Z, iflag, rparams[2] + T(jparam), pt);
          }
        else if (ltype == PRISMATIC)
          {
            rotPoint(X, iflag, rparams[0], pt);
            trnPoint(X, iflag, rparams[1], pt);
            trnPoint(Z, iflag, rparams[3] + T(jparam), pt);
            rotPoint(Z, iflag, rparams[2], pt);
          }
      }
    else
      { // inverse DH
        if (ltype == REVOLUTE)
          {
            rotPoint(Z, iflag, rparams[2] + T(jparam), pt);
            trnPoint(Z, iflag, rparams[3], pt);
            trnPoint(X, iflag, rparams[1], pt);
            rotPoint(X, iflag, rparams[0], pt);
          }
        else if (ltype == PRISMATIC)
          {
            rotPoint(Z, iflag, rparams[2], pt);
            trnPoint(Z, iflag, rparams[3] + T(jparam), pt);
            trnPoint(X, iflag, rparams[1], pt);
            rotPoint(X, iflag, rparams[0], pt);
          }
      }
  }

  template<typename T> inline void mdhPoint(const LinkType ltype, const bool iflag, const T * const rparams, const double jparam, T * const pt) const {
    if (!iflag)
       { // Modified DH
         if (ltype == REVOLUTE)
           {
             rotPoint(Y, iflag, rparams[1], pt);
             rotPoint(X, iflag, rparams[0], pt);
             trnPoint(X, iflag, rparams[3], pt);
             rotPoint(Z, iflag, rparams[2] + T(jparam), pt);
           }
         else if (ltype == PRISMATIC)
           {
             rotPoint(Y, iflag, rparams[1], pt);
             rotPoint(X, iflag, rparams[0], pt);
             trnPoint(X, iflag, rparams[3] + T(jparam), pt);
             rotPoint(Z, iflag, rparams[2], pt);
           }
       }
     else
       {  // inverse DH
         if (ltype == REVOLUTE)
           {
             rotPoint(Z, iflag, rparams[2] + T(jparam), pt);
             trnPoint(X, iflag, rparams[3], pt);
             rotPoint(X, iflag, rparams[0], pt);
             rotPoint(Y, iflag, rparams[1], pt);
           }
         else if (ltype == PRISMATIC)
           {
             rotPoint(Z, iflag, rparams[2], pt);
             trnPoint(X, iflag, rparams[3] + T(jparam), pt);
             rotPoint(X, iflag, rparams[0], pt);
             rotPoint(Y, iflag, rparams[1], pt);
           }
       }
  }

public:
  ModifiedDHRobotDescription() {
    no_links = -1;
    type = MODIFIED_DENAVITHARTENBERG;
  }

  virtual ~ModifiedDHRobotDescription() {}

  Matrix4f getMatrix(const VectorXf&) const;
  virtual void setParameters(const double * const);
  virtual int getParameters(double* &, vector<int>&);
  virtual void addLinearPenalties(ceres::Problem *const, const int) const;

  virtual void loadMask(ifstream&);
  virtual void load(ifstream&);
  virtual void save(const string &) const;


  inline int getNumLinks(void) const {
    return no_links;
  }

  inline bool isInverseLink(const int i) const {
    if ((i < 0) || (i >= no_links))
      LOG_FATAL << "Link position out of bounds: " << i << " [0," << (no_links - 1) << "]" << endl;

    return inverse_flags[i];
  }

  inline bool isModifiedDHLink(const int i) const {
    if ((i < 0) || (i >= no_links))
      LOG_FATAL << "Link position out of bounds: " << i << " [0," << (no_links - 1) << "]" << endl;

    return mdh_flags[i];
  }

  inline LinkType getLinkType(const int i) const {
    if ((i < 0) || (i >= no_links))
      LOG_FATAL << "Link position out of bounds: " << i << " [0," << (no_links - 1) << "]" << endl;

    return link_types[i];
  }

  template<typename T> inline void transformPoint(const VectorXf &jparams, const T * const rparams, T * const pt, const bool inv_flag = false) const  {
    for (int i = no_links - 1; i >= 0; i--)
      {
        if (mdh_flags[i])
          mdhPoint(link_types[i], inverse_flags[i], rparams + 4*i, jparams[i], pt);
        else
          cdhPoint(link_types[i], inverse_flags[i], rparams + 4*i, jparams[i], pt);
      }
  }

  template<typename T> inline void inverseTransformPoint(const VectorXf &jparams, const T * const rparams, T * const pt) const  {
    for (int i = 0; i < no_links; i++)
      {
        if (mdh_flags[i])
          mdhPoint(link_types[i], !inverse_flags[i], rparams + 4*i, jparams[i], pt);
        else
          cdhPoint(link_types[i], !inverse_flags[i], rparams + 4*i, jparams[i], pt);
      }
  }
};

#endif /* RDESC_MDH_H_ */
