/*
 * definitions.h
 *
 *  Created on: Jul 31, 2012
 *      Author: jheller
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#define RBA_VERSION "0.4.0"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <vector>
#include <list>
#include <string>
#include <stdexcept>
#include <iostream>
using namespace std;

#include "logging.h"
#include "options.h"

//typedef Eigen::Matrix<double, 4, 1> Vector4f;
//typedef Eigen::Matrix<double, 4, 4> Matrix4f;

typedef Eigen::Matrix<int, 2, 1> Vector2i;
typedef Eigen::Matrix<int, 3, 1> Vector3i;

typedef Eigen::Matrix<double, 2, 1> Vector2f;
typedef Eigen::Matrix<double, 3, 1> Vector3f;
typedef Eigen::Matrix<double, 4, 1> Vector4f;
typedef Eigen::Matrix<double, 5, 1> Vector5f;
typedef Eigen::Matrix<double, 6, 1> Vector6f;
typedef Eigen::Matrix<double, 7, 1> Vector7f;
typedef Eigen::Matrix<double, 8, 1> Vector8f;

typedef Eigen::Matrix<double, 2, 2> Matrix2f;
typedef Eigen::Matrix<double, 3, 3> Matrix3f;
typedef Eigen::Matrix<double, 4, 4> Matrix4f;

typedef Eigen::VectorXd VectorXf;
typedef Eigen::MatrixXd MatrixXf;


#define PI_8 0.39269908169872413950
#define PI_6 0.52359877559829887308
#define PI_2 1.57079632679489661923
#define PI   3.14159265358979323846
#define PI_R 0.31830988618379069122

/*
template <typename T> bool iszero(T val) {
  return (+val < (100.0 * std::numeric_limits<T>::epsilon())) &&
         (-val < (100.0 * std::numeric_limits<T>::epsilon()));
}
*/

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T> T incsin(T val) {
  if (val <= T(PI_2))
    return sin(val);
  else if (val <= PI)
    return sin(val - T(PI)) + T(2.0f);
  else
    return val - T(PI) + T(2.0f);
}

template <typename T> T incasin(T val) {
  if (val <= T(1.0f))
    return asin(val);
  else if (val <= T(2.0f))
    return asin(val - T(2.0f)) + T(PI);
  else
    return val - T(2.0f) + T(PI);
}


inline void roundVector(const Vector2f &u, int &x, int &y)
{
  x = int(round(double(u(0))));
  y = int(round(double(u(1))));
}

template<typename T1, typename T2> void copyVector(const T1 &v2, T2 *v1, const int n) {
  for (int i = 0; i < n; i++)
    v1[i] = T2(v2[i]);
}

template<typename _Matrix_Type_> _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

#ifdef _MSC_VER
#if _MSC_VER <= 1600
 #include <boost/math/special_functions/round.hpp>

template<typename T> T round(T val) {
  try
    {
      return boost::math::round(val);
    }
  catch (exception &e)
    {
      return T(0);
    }
}

#endif
#endif


#ifdef _MSC_VER
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif

#endif /* DEFINITIONS_H_ */
