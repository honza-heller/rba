/*
 * main.cpp
 *
 *  Created on: Jul 30, 2012
 *      Author: hellej1
 */

#include "stdio.h"
#include "stdlib.h"

#include <stdexcept>
#include <iostream>
#include "rba.h"


int main(int argc, char **argv)
{
  try
    {
      RoboticBundleAdjuster rba;

      bool stop_flag = rba.init(argc, argv);

      // Don't continue if --help of --version used
      if (stop_flag)
        return 0;

      rba.load();
      rba.calibrate();
      rba.save();
    }
  catch (runtime_error & e)
    {
      cerr << "RBA ERROR: " << e.what() << endl;
      return 1;
    }
  catch (exception& e)
    {
      cerr << "RBA ERROR: " << e.what() << endl;
      return 1;
    }

  return 0;
}

