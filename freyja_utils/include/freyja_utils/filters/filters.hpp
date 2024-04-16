/* Aj's classy filter library.
This implements a set of commonly used data filters, each suited for
its own purpose. An instance of this class can be created anywhere
(usually in state managers), and the object is passed the vector to be
filtered. The vector (and shuffling) is *not* maintained by this class!
That is: no previous data memory is held here, the implementation is
re-entrant (strictly Markovian, in this respect). Only the filter
properties (coeffs, type, len. etc) are held here.

  ~ aj / Nov 2016.
*/

#ifndef FREYJA_UTILS_FILTERS_HPP
#define FREYJA_UTILS_FILTERS_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>
#include <mutex>  // new
#include <thread>  // new
#include <functional>
#include "eigen3/Eigen/Dense"
#include <static_sort.h>

namespace freyja_utils
{
  class Filter
  {
    protected:
      unsigned int filter_len_;
      std::string filter_name_;

    public:
      Filter() {}

      // derived classes must implement an init function
      virtual void init() = 0;

      // for markov-type state filters
      virtual void setMeasurementInput( const Eigen::Vector3d ) {}
      virtual void setMeasurementInput( const std::vector<double> & ) {}
      virtual void getStateEstimate( Eigen::VectorXd &x_est ) {}
      virtual void getStateEstimate( Eigen::VectorXd &x_est, const int& n_states ) {}
      virtual void getStateEstimate( std::vector<double> &x_est ) {}
      virtual void getStateEstimate( std::vector<double> &x_est, const int& n_states ) {}
      virtual void printStateEstimate() {};

      // Eigen versions
      virtual void filterObservations( const Eigen::VectorXd &obs, double &retVal ) {}
      virtual void filterObservations( const Eigen::MatrixXd &obs, Eigen::VectorXd &retVal ) {}

      // STL versions
      virtual void filterObservations( const std::vector<double> &obs, double &retVal ) {}
      virtual void filterObservations(  const std::vector<double> &obs1, 
                                        const std::vector<double> &obs2, 
                                        const std::vector<double> &obs3, 
                                        double &retVal1,
                                        double &retVal2,
                                        double &retVal3 ) {}

      // common accessors
      unsigned int getFilterLen() { return filter_len_; }
  };

}

#include "freyja_conv_filters.hpp"
#include "freyja_median_filter.hpp"
#include "freyja_kalman_filter.hpp"

#endif
