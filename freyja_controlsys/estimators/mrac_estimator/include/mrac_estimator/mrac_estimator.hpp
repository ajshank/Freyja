#ifndef MRAC_ESTIMATOR__MRAC_ESTIMATOR_HPP_
#define MRAC_ESTIMATOR__MRAC_ESTIMATOR_HPP_

#include "mrac_estimator/visibility_control.h"

#include "freyja_control/freyja_control.hpp"

class MracEstimator
{
  double estimator_dt_;

  Eigen::Matrix<double, 6, 6>   P_;
  Eigen::Matrix<double, 6, 3>   B_;
  Eigen::Matrix<double, -1, 3>  W_;
  Eigen::Matrix<double, -1, 3>  Wdot_;
  Eigen::Matrix<double, -1, 1>  basis_vec_;
  double gamma_w_;

  Eigen::Vector3d  u_adapt_;

  PosVelNED   curstate_ned_;
  PosVelNED   refstate_ned_;

  std::thread estimator_thread_;
  void runEstimator();
  bool estimator_alive_;

public:
  MracEstimator();
  ~MracEstimator();

  inline void init() { initEstimator(); }
  inline void stop() { stopEstimator(); }
  void initEstimator();
  void stopEstimator() { estimator_alive_ = false; }

  void setupBasisVec();
  void updateBasisVec();
  void updateWeightMatrix();

  void getAdaptiveCtrl(Eigen::Vector3d& _u_adapt) { _u_adapt = u_adapt_; }

  inline void setCurPosVel(const PosVelNED _pv) { curstate_ned_ = std::move(_pv); }
  inline void setRefPosVel(const PosVelNED _pv) { refstate_ned_ = std::move(_pv); }


};

#endif
