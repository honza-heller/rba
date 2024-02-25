/*
 * track_calibdev.h
 *
 *  Created on: Nov 27, 2012
 *      Author: hellej1
 */

#ifndef TRACK_CALIBDEV_H_
#define TRACK_CALIBDEV_H_

#include "track.h"

class CalibdevTrack : public Track {

public:
  CalibdevTrack() : Track() {}
  CalibdevTrack(int _cams_no) : Track(_cams_no) {}

  virtual void loadTrackData(ifstream &iFile) {
    int i;

    deleteData();

    point[SOURCE] = Vector4f::Zero();

    iFile >> point[SOURCEPT](0) >> point[SOURCEPT](1) >> point[SOURCEPT](2);
    point[SOURCEPT](3) = 1.0f;
    src_flag = true;

    // We don't need to triangulate
    point[TRIANGULATEDPT] = point[SOURCEPT];

    iFile >> cams_no;

    cams_idx = new int[cams_no];
    keys = NULL;
    detections[0] = new Vector2f[cams_no];
    detections[1] = new Vector2f[cams_no];
    detections[2] = new Vector2f[cams_no];
    residuals[0] = new double[cams_no];
    residuals[1] = new double[cams_no];
    residuals[2] = new double[cams_no];

    for(i = 0; i < cams_no; i++)
      {
        iFile >> cams_idx[i] >> detections[SOURCE][i](0) >> detections[SOURCE][i](1);
        residuals[0][i] = 0.0f;
        residuals[1][i] = 0.0f;
      }
  }
};


#endif /* TRACK_CALIBDEV_H_ */
