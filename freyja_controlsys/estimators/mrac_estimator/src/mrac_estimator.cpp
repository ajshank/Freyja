#include "mrac_estimator/mrac_estimator.hpp"



MracEstimator::MracEstimator()
{
  estimator_alive_ = false;
}

MracEstimator::~MracEstimator()
{
  estimator_alive_ = false;
  estimator_thread_.join();
}



void MracEstimator::initEstimator()
{
  curstate_ned_.setZero();
  refstate_ned_.setZero();
  P_ << -0.50,     0,      0,   0.01,      0,      0,
         0,    -0.50,      0,      0,   0.01,      0,
         0,        0,  -0.50,      0,      0,   0.01,
         0.01,     0,      0,  -0.50,      0,      0,
         0,     0.01,      0,      0,  -0.50,      0,
         0,        0,   0.01,      0,      0,  -0.50;
  B_ << 0.001250,          0,          0,
         0,         0.001250,          0,
         0,                0,   0.001250,
         0.050000,         0,          0,
         0,         0.050000,          0,
         0,                0,   0.050000;
  gamma_w_ = 0.9;

  setupBasisVec();

  // configure estimator to run as an independent thread
  estimator_dt_ = 0.05;
  estimator_alive_ = true;
  estimator_thread_ = std::move(std::thread(&MracEstimator::runEstimator, this));
}

void MracEstimator::setupBasisVec()
{
  int n_bases = 5;
  basis_vec_ = Eigen::MatrixXd::Zero(n_bases, 1);
  W_ = 0.1*Eigen::MatrixXd::Random(n_bases, 3);
  Wdot_ = Eigen::MatrixXd::Zero(n_bases, 3);
}


void MracEstimator::updateBasisVec()
{
  basis_vec_ << curstate_ned_.coeff(0),
                curstate_ned_.coeff(1),
                curstate_ned_.coeff(2),
                curstate_ned_.segment<2>(3).squaredNorm(),
                curstate_ned_.coeff(5);
}
void MracEstimator::updateWeightMatrix()
{
  Eigen::Ref<PosVelNED> x = curstate_ned_;
  Eigen::Ref<PosVelNED> xr = refstate_ned_;
  Wdot_ = gamma_w_ * basis_vec_ * (x-xr).transpose() * P_ * B_;
  W_ += (Wdot_ * estimator_dt_);
}

void MracEstimator::runEstimator()
{
  int estimator_dt_ms = estimator_dt_*1000.0;
  auto t_update = std::chrono::high_resolution_clock::now();
  // this function runs on a thread
  while(estimator_alive_)
  {
    // note the current time point
    t_update = std::chrono::high_resolution_clock::now();
    // assume setter has given us current and reference states

    // update new basis function(s)
    updateBasisVec();

    // calculate weight update
    updateWeightMatrix();

    // calculate adaptive control
    u_adapt_ = W_.transpose() * basis_vec_;

    // sleep until next time iteration
    std::this_thread::sleep_until(t_update + std::chrono::milliseconds(estimator_dt_ms));
  }
}
