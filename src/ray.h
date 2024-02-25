/*
 * ray.h
 *
 *  Created on: Aug 2, 2012
 *      Author: hellej1
 */

#ifndef RAY_H_
#define RAY_H_

#include "definitions.h"
#include "etransform.h"

class Ray {
  Vector4f  dir;  // == (a,b,c,0)
  Vector4f  orig; // == (d,e,f,1)

public:
  Ray() {
    dir = Vector4f::Zero();
    orig = Vector4f::Zero();
  }

  void setRay(const Vector4f _dir, const Vector4f _orig) {
    dir = _dir;
    orig = _orig;
  }


  void transformRay(const ETrans &trans) {
    dir = trans * dir;
    orig = trans * orig;
  }

  Vector4f getDirection(void) const {
    return dir;
  }

  Vector4f getOrigin(void) const {
    return orig;
  }

};

#endif /* RAY_H_ */
