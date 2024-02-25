/*
 * stats.h
 *
 *  Created on: Mar 27, 2014
 *      Author: hellej1
 */

#ifndef DATASTATS_H_
#define DATASTATS_H_

#include <vector>
#include <cmath>
using namespace std;

 class DataStats {
 private:
   unsigned int         hresolution;
   double               mean;
   double               max;
   double               rms;
   vector<int>          hist;
   vector<double>       ticks;
   vector<double>       data;

public:

   DataStats(unsigned int _hresolution = 100) {
     mean = std::numeric_limits<double>::quiet_NaN();
     max = std::numeric_limits<double>::quiet_NaN();
     rms = std::numeric_limits<double>::quiet_NaN();
     hresolution = _hresolution;
     hist.resize(hresolution);
   }

   void clear() {
     mean = 0;
     max = 0;
     rms = 0;
     data.clear();
     hist.clear();
     ticks.clear();
   }

   void add(double datum) {
     data.push_back(datum);
     max = (max > datum) ? max : datum;
     mean += datum;
     rms += datum * datum;
   }

   void updateStatistics() {
     if (data.size() != 0.0f)
       {
         mean /= data.size();
         rms = std::sqrt(rms / data.size());
       }
     else
       {
         mean = std::numeric_limits<double>::quiet_NaN();
         rms = std::numeric_limits<double>::quiet_NaN();
       }
   }

   void updateHistogram() {
     updateHistogram(max);
   }

   void updateHistogram(double merr) {
     hist.resize(hresolution);
     ticks.resize(hresolution);

     if (merr <= 0.0f)
       {
         for (unsigned int i = 0; i < hresolution; i++)
           {
             hist[i] = std::numeric_limits<double>::quiet_NaN();
             ticks[i] = std::numeric_limits<double>::quiet_NaN();
           }
       }
     else
       {
         for (unsigned int i = 0; i < hresolution; i++)
           {
             hist[i] = 0;
             ticks[i] = merr * (double) i / hresolution;
           }

         for (unsigned int i = 0; i < data.size(); i++)
           {
             int pos = std::floor(hresolution * data[i] / merr);
             if (pos <= (int) hresolution)
               hist[pos]++;
           }
       }
   }

   double getMeanError() const {
     return mean;
   }

   double getRmsError() const {
     return rms;
   }

   double getMaxError() const {
     return max;
   }

   unsigned int getNumData() const {
     return data.size();
   }

   const std::vector<int> & getHistogram() const {
     return hist;
   }

   const std::vector<double> & getHistogramTicks() const {
     return ticks;
   }

   void setHistogramResolution(unsigned int _hresolution = 100) {
     hresolution = _hresolution;
     hist.resize(hresolution);
   }

   unsigned int getHistogramResolution(void) const {
     return hresolution;
   }

 };


#endif /* DATASTATS_H_ */
