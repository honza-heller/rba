/*
 * rdesc.h
 *
 *  Created on: Mar 13, 2014
 *      Author: jheller
 */

#ifndef RDESC_H_
#define RDESC_H_

#include "definitions.h"
#include <ceres/ceres.h>

class RobotDescription {
public:

  typedef enum {
    BASE = 0,
    MODIFIED_DENAVITHARTENBERG = 1,
  } RobotType;

protected:
  RobotType type;
  double *params;

  class LinearPenaltyFunctor {
    int    idx;
    double value_0;
    double penalty;

    public:
    LinearPenaltyFunctor(const int _idx, const double _value_0, const double _penalty)
    : idx(_idx), value_0(_value_0), penalty(_penalty) {}

    template <typename T>
    bool operator() (T const* const* params, T * const res) const {
      res[0] = T(penalty) * (params[0][idx] - value_0);
      return true;
    }
  };

public:
  RobotDescription() {
    type = BASE;
    params = NULL;
  }

  virtual ~RobotDescription() {}

  virtual void loadMask(ifstream &) = 0;
  virtual void load(ifstream &) = 0;
  virtual void save(const string &) const = 0;

  virtual int getParameters(double* &, vector<int>&) = 0;
  virtual void setParameters(const double * const) = 0;
  virtual void addLinearPenalties(ceres::Problem * const, const int) const = 0;

  RobotType getType(void) const {
    return type;
  }
};


#endif /* RDESC_H_ */
