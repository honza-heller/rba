/*
 * tmatrix.h
 *
 *  Created on: Aug 1, 2012
 *      Author: jheller
 */

#include <fstream>
#include <iostream>

#include "definitions.h"

#include "ceres/rotation.h"

using namespace std;

#ifndef ETRANSFORMATION_H_
#define ETRANSFORMATION_H_

class ETrans {
  Matrix4f matrix;
  Matrix4f imatrix;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ETrans() {
    matrix = Matrix4f::Zero();
    imatrix = Matrix4f::Zero();
  }

  ETrans(const double * const angleaxis, const double * const translation) {
    matrix = Matrix4f::Zero();
    imatrix = Matrix4f::Zero();
    setMatrix(angleaxis, translation);
  }

  ETrans(const Matrix3f &r, const Vector3f &t) {
    matrix = Matrix4f::Zero();
    imatrix = Matrix4f::Zero();
    setMatrix(r, t);
  }

  ETrans(const Matrix4f &m) {
    setMatrix(m);
  }

  void loadRt(ifstream& iFile) {
    matrix = Matrix4f::Zero();
    matrix(3, 3) = 1.0f;

    iFile >>
      matrix(0, 0) >> matrix(0, 1) >> matrix(0, 2) >>
      matrix(1, 0) >> matrix(1, 1) >> matrix(1, 2) >>
      matrix(2, 0) >> matrix(2, 1) >> matrix(2, 2) >>
      matrix(0, 3) >> matrix(1, 3) >> matrix(2, 3);

    imatrix = Matrix4f::Zero();
    imatrix(3, 3) = 1.0f;
    imatrix.block(0, 0, 3, 3) = matrix.block(0, 0, 3, 3).transpose();
    imatrix.block(0, 3, 3, 1) = -(matrix.block(0, 0, 3, 3).transpose() * matrix.block(0, 3, 3, 1));
  }

  void writeRt(ostream &oFile) const {
    oFile << double(matrix(0, 0)) << " " << double(matrix(0, 1)) << " " << double(matrix(0, 2)) << endl;
    oFile << double(matrix(1, 0)) << " " << double(matrix(1, 1)) << " " << double(matrix(1, 2)) << endl;
    oFile << double(matrix(2, 0)) << " " << double(matrix(2, 1)) << " " << double(matrix(2, 2)) << endl;
    oFile << double(matrix(0, 3)) << " " << double(matrix(1, 3)) << " " << double(matrix(2, 3)) << endl;
  }

  inline void setIdentity(void) {
    matrix = Matrix4f::Identity();
    imatrix = Matrix4f::Identity();
  }

  inline Matrix4f getMatrix(void) const {
    return matrix;
  }

  inline void getMatrix(Matrix3f &r, Vector3f &t) {
    r = matrix.block(0, 0, 3, 3);
    t = matrix.block(0, 3, 3, 1);
  }

  inline void getInverseMatrix(Matrix3f &r, Vector3f &t) {
    r = imatrix.block(0, 0, 3, 3);
    t = imatrix.block(0, 3, 3, 1);
  }

  inline void getRotation(Matrix3f &r) const {
    r = matrix.block(0, 0, 3, 3);
  }

  inline void getInverseRotation(Matrix3f &r) const {
    r = imatrix.block(0, 0, 3, 3);
  }

  inline void getAngleAxis(Vector3f &angleaxis) const {
    double arot[3];
    getAngleAxis(arot);
    angleaxis << arot[0], arot[1], arot[2];
  }

  inline void getAngleAxis(double* const angleaxis) const {
    double rotation[3][3];

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        rotation[i][j] = matrix(j,i);

    ceres::RotationMatrixToAngleAxis((double *) rotation, angleaxis);
  }

  inline void getAngleAxis(double * const angleaxis, double * const translation) const {
    double rotation[3][3];

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        rotation[i][j] = matrix(j,i);

    ceres::RotationMatrixToAngleAxis((double *) rotation, angleaxis);

    translation[0] = matrix(0,3);
    translation[1] = matrix(1,3);
    translation[2] = matrix(2,3);
  }

  inline void getInverseAngleAxis(double * const angleaxis, double * const translation) const {
    double rotation[3][3];

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        rotation[i][j] = imatrix(j,i);

    ceres::RotationMatrixToAngleAxis((double *) rotation, angleaxis);

    translation[0] = imatrix(0,3);
    translation[1] = imatrix(1,3);
    translation[2] = imatrix(2,3);
  }

  inline void getAngleAxisTranslation(double * const aat) const {
    getAngleAxis(aat, aat + 3);
  }

  inline void getInverseAngleAxisTranslation(double * const aat) const {
    getInverseAngleAxis(aat, aat + 3);
  }

  inline void getQuaternion(double * const quaternion) const {
    double rotation[3][3];
    double angleaxis[3];

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        rotation[i][j] = matrix(j,i);

    ceres::RotationMatrixToAngleAxis((double *) rotation, angleaxis);
    ceres::AngleAxisToQuaternion(angleaxis, quaternion);

    double scale = 1.0f /  std::sqrt(quaternion[0] * quaternion[0] +
                                quaternion[1] * quaternion[1] +
                                quaternion[2] * quaternion[2] +
                                quaternion[3] * quaternion[3]);

    quaternion[0] *= scale;
    quaternion[1] *= scale;
    quaternion[2] *= scale;
    quaternion[3] *= scale;
  }

  inline void getTranslation(double * const translation) const {
    translation[0] = matrix(0,3);
    translation[1] = matrix(1,3);
    translation[2] = matrix(2,3);
  }

  inline void getTranslation(Vector3f &t) const {
    t = matrix.block(0, 3, 3, 1);
  }

  inline void getInverseTranslation(Vector3f &t) const {
    t = imatrix.block(0, 3, 3, 1);
  }

  inline void setMatrix(const Matrix3f &r, const Vector3f &t) {
    matrix.block(0, 0, 3, 3) = r;
    matrix.block(0, 3, 3, 1) = t;
    matrix.block(3, 0, 1, 4) = Vector4f(0.0f, 0.0f, 0.0f, 1.0f).transpose();

    imatrix.block(0, 0, 3, 3) = matrix.block(0, 0, 3, 3).transpose();
    imatrix.block(0, 3, 3, 1) = -(matrix.block(0, 0, 3, 3).transpose() * matrix.block(0, 3, 3, 1));
    imatrix.block(3, 0, 1, 4) = Vector4f(0.0f, 0.f, 0.0f, 1.0f).transpose();
  }

  inline void setMatrix(const Matrix4f &m) {
    matrix = m;
    imatrix = Matrix4f::Zero();
    imatrix(3, 3) = 1.0f;
    imatrix.block(0, 0, 3, 3) = matrix.block(0, 0, 3, 3).transpose();
    imatrix.block(0, 3, 3, 1) = -(matrix.block(0, 0, 3, 3).transpose() * matrix.block(0, 3, 3, 1));
  }

  inline void setMatrix(const double * const pose) {
    double quaternion[4];

    quaternion[0] = pose[0];
    quaternion[1] = pose[1];
    quaternion[2] = pose[2];
    quaternion[3] = std::sqrt(1.0 - pose[0]*pose[0] - pose[1]*pose[1] - pose[2]*pose[2]);

    setMatrix(quaternion, pose + 3, true);
  }

  inline void setAngleAxisTranslation(const double * const aat) {
    setMatrix(aat, aat + 3, false);
  }

  inline void setMatrix(const double * const rot, const double * const tran, const bool qflag=false) {
    double rotation[3][3];
    double r[3];

    if (qflag)
      {
        ceres::QuaternionToAngleAxis(rot, r);
        ceres::AngleAxisToRotationMatrix(r, (double *) rotation);
      }
    else
      {
        ceres::AngleAxisToRotationMatrix(rot, (double *) rotation);
      }

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        matrix(j,i) = rotation[i][j];

    matrix(0,3) = tran[0];
    matrix(1,3) = tran[1];
    matrix(2,3) = tran[2];
    matrix(3, 3) = 1.0f;

    imatrix(3, 3) = 1.0f;
    imatrix.block(0, 0, 3, 3) = matrix.block(0, 0, 3, 3).transpose();
    imatrix.block(0, 3, 3, 1) = -(matrix.block(0, 0, 3, 3).transpose() * matrix.block(0, 3, 3, 1));
  }

  inline void setAngleAxis(const Vector3f &angleaxis) {
    double rotation[3][3];
    double aa[3];

    aa[0] = angleaxis(0);
    aa[1] = angleaxis(1);
    aa[2] = angleaxis(2);

    ceres::AngleAxisToRotationMatrix(aa, (double *) rotation);

    for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
          {
            matrix(j, i) = rotation[i][j];
            imatrix(i, j) = rotation[i][j];
          }
      }
  }

  inline Matrix4f getInverseMatrix(void) const {
    return imatrix;
  }

  inline operator Matrix4f () const {
    return getMatrix();
  }

  inline Vector4f operator*(const Vector4f &v) const {
    return matrix * v;
  }

  inline void inverse() {
    Matrix4f t = matrix;
    matrix = imatrix;
    imatrix = t;
  }

  bool isZero(void) const {
    return (Matrix4f::Zero() == matrix);
  }

  bool isIdentity(void) const {
    return (Matrix4f::Identity() == matrix);
  }
};


#endif /* ETRANSFORMATION_H_ */
