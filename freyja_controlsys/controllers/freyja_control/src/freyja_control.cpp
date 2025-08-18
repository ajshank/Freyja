#include "freyja_control/freyja_control.hpp"

void FreyjaControl::preinitContainers()
{
  std::fill(curstate_.pos_ned.begin(), curstate_.pos_ned.end(), 0.0);
  std::fill(curstate_.vel_ned.begin(), curstate_.vel_ned.end(), 0.0);
  std::fill(curstate_.acc_ned.begin(), curstate_.acc_ned.end(), 0.0);
  std::fill(curstate_.ang_rpy.begin(), curstate_.ang_rpy.end(), 0.0);

  RCLCPP_INFO(get_logger(), "Done init!");
}

void FreyjaControl::initBaseController()
{
  // set up feedback matrix
  basectrl_lqr_k_ <<  1.1180, 0.0, 0.0, 1.4995, 0.0, 0.0, 0.0,
                      0.0, 1.1180, 0.0, 0.00, 1.4995, 0.0, 0.0,
                      0.0, 0.0, 3.1623, 0.0, 0.0, 2.5347, 0.0,
                      0.0, 0.0, 0.0, 0.0000, 0.0, 0.0, 1.000;
}

void FreyjaControl::baseCtrlComputeFeedback()
{
  basectrl_lqr_u_acc_ = -basectrl_lqr_k_ * (pvy_curstate_ - pvy_refstate_)
                      + static_cast<double>(basectrl_opts_.ctrl_use_ff_)*ref_accel_ff_;
}
