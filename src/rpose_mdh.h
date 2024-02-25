/*
 * rpose_mdh.h
 *
 *  Created on: Mar 13, 2014
 *      Author: jheller
 */

#ifndef RPOSE_MDH_H_
#define RPOSE_MDH_H_

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>

#include "rpose.h"
#include "rdesc_mdh.h"

class ModifiedDHRobotPose : public RobotPose {
  ModifiedDHRobotDescription *rdesc;
  VectorXf jparams;

public:

  ModifiedDHRobotPose(ModifiedDHRobotDescription *_rdesc) {
    if (_rdesc == NULL)
      LOG_FATAL << "ModifiedDHRobotDescription must NOT be NULL in ModifiedDHRobotPose";

    rdesc = _rdesc;
    jparams.resize(rdesc->getNumLinks());
    type = MODIFIED_DENAVITHARTENBERG;
  };

 virtual ~ModifiedDHRobotPose() {};

 virtual void loadPoseData(ifstream &iFile) {
   string line;
   vector<string> fields;

   do {
       getline(iFile, line);
   } while (line == "");

   boost::split(fields, line, boost::is_any_of(" "), boost::token_compress_on);

   if ((int) fields.size() != rdesc->getNumLinks())
     LOG_FATAL << "Cannot parse robot pose data: the number of parameters (" << fields.size() <<
     ") does not match the number of links (" << rdesc->getNumLinks() << ")";

   for (int i = 0; i < rdesc->getNumLinks(); i++)
     jparams(i) = boost::lexical_cast<double>(fields[i]);

   pose.setMatrix(rdesc->getMatrix(jparams));
   pose.getAngleAxisTranslation(aat);
   pose.getInverseAngleAxisTranslation(iaat);
 }

 virtual void updatePoseData(void) {
   pose.setMatrix(rdesc->getMatrix(jparams));
   pose.getAngleAxisTranslation(aat);
   pose.getInverseAngleAxisTranslation(iaat);
 }

 template<typename T> inline void transformPoint(const T * const rparams, T * const pt) const {
   rdesc->transformPoint(jparams, rparams, pt);
 }

 template<typename T> inline void inverseTransformPoint(const T * const rparams, T * const pt) const {
   rdesc->inverseTransformPoint(jparams, rparams, pt);
 }
};

#endif /* RPOSE_MDH_H_ */
