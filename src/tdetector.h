/*
 * tdetector.h
 *
 *  Created on: Jul 27, 2014
 *      Author: jheller
 */

#ifndef TDETECTOR_H_
#define TDETECTOR_H_

#include "definitions.h"
#include "math.h"

class TargetDetector {
public:
  typedef enum {
    UNDEF_TARGET = 0,
    CHESSBOARD = 1,
    CIRCLES_GRID = 2,
    ASYMMETRIC_CIRCLES_GRID = 3,
    ELMARK_CIRCLES_GRID = 4,
    ELMARK_CHESSBOARD_GRID = 5,
    ELMARK_CIRCLES_CORNERS = 6
  } TargetType;

protected:
  TargetType         target_type;
  double             xstride;
  double             ystride;
  unsigned int       width;
  unsigned int       height;

  const RbaOptions * opts;

public:

  virtual ~TargetDetector() {}

  void setStrides(double _xstride, double _ystride) {
    xstride = _xstride;
    ystride = _ystride;
  }

  void setWidth(int _width) {
    width = _width;
  }

  void setHeight(int _height) {
    height =_height;
  }

  inline void setOptions(const RbaOptions * const _opts) {
    opts = _opts;
  }

  virtual bool detect(const char * const) = 0;
  virtual int identify(void) = 0;
  virtual void getDetections(list<Vector3f> &, list<Vector2f> &, list<unsigned long long int> &) = 0;

protected:
  // pointIndex : Z x Z -> N assigns a positive integer index to a point on the integer plane
  // by spiraling out of the origin: https://en.wikipedia.org/wiki/Ulam_spiral
  inline unsigned long long int pointIndex(const int x, const int y) const  {
    int a0 = 2 * std::max(std::abs(x), std::abs(y)) - 1;
    int a1 = a0 + 1;
    int s = x + y;
    int d;

    if (x >= y)
      d = ((s <= 0) ? 1 : 0) * a1 * 4 + s;
    else
      d = a1 * 2 - s;

    return a0 * a0 + d - 1;
  }

};


#endif /* TDETECTOR_H_ */
