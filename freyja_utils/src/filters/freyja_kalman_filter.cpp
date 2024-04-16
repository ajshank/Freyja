/*
  ** State model and the choice of variable names **
  State propagation model:
    x(k+1) = A.x(k) + B.u(k)
    z(k) = C.x(k) + Q
    Process noise: Q
    State covariance: P
    
  Variable names have both a colloquial part and the mathematical-term part.
  >> `state_cov_P_` is the state covariance matrix, P.
  >> `proc_noise_Q_` is noise in the state propagation (aka process noise).
  >> `meas_matrix_C_` is the state observation matrix
  >> `measurement_z_` is the z(k) observation vector (and not altitude).
  >> `best_estimate_` represents the x(k) vector.
  
  State updation model:
    G = P * C' * (C*P*C' + R)'
    x = x + G*(Z - Cx)
    P = (I-GC)*P
    Measurement noise: R
  
*/
#include <thread>
#include <mutex>

namespace freyja_utils
{
  /* Linear quadratic estimator (LQE) implementation.

  State propagation happens at a rate higher than the controller. External
  sources can trigger an update function at sporadic times.

  -- aj / Apr 18, 2019
  */

  #define ZERO3x3 Eigen::MatrixXd::Zero(3,3)
  #define ZERO6x6 Eigen::MatrixXd::Zero(6,6)
  #define ZERO9x9 Eigen::MatrixXd::Zero(9,9)
  #define IDEN3x3 Eigen::MatrixXd::Identity(3,3)
  #define IDEN6x6 Eigen::MatrixXd::Identity(6,6)
  #define IDEN9x9 Eigen::MatrixXd::Identity(9,9)
  #define IDEN12x12 Eigen::MatrixXd::Identity(12,12)

  typedef std::chrono::microseconds uSeconds;

  KalmanFilter::KalmanFilter( int estimator_rate, const std::vector<double> params )
  {
    n_stprops_since_update_ = 0;
    state_propagation_alive_ = false;
    init( estimator_rate, params );
  }

  /* LQE initialisation requires a rate argument, and a list of parameters as:
      params: {sigmaMeas, sigmaPos, sigmaVel, sigmaAcc [,sigmaJrk]}
    and the number of parameters is used to infer the filter order.
  */
  void KalmanFilter::init( int estimator_rate, const std::vector<double> &params )
  {
    estimator_rate_ = std::move( estimator_rate );
    estimator_period_us_ = std::round( (1.0/estimator_rate_)*1000*1000 );

    estimator_params_ = std::move(params);
    use_standard_q_ = (estimator_params_.size()==2);

    init( );
  }

  void KalmanFilter::init( )
  {
    // constants for the lq-estimator
    initLqeSystem();
                            
    // separate thread for state prop
    state_propagation_alive_ = true;
    last_prop_t_ = std::chrono::high_resolution_clock::now();
    state_prop_thread_ = std::thread( &KalmanFilter::state_propagation, this );

    std::cout << "KalmanFilter init! Using auto-proc-cov: " << use_standard_q_ << std::endl;
  }



  KalmanFilter::~KalmanFilter()
  {
    state_propagation_alive_ = false;
    state_prop_thread_.join();
    std::cout << "KalmanFilter: propagation thread stopped!" << std::endl;
  }

  void KalmanFilter::initLqeSystem( )
  {
    /* LQE model is a single/double integrator with downscaled ctrl inputs:
        x(k+1) = A.x(k) + B.u(k)
        z(k) = C.x(k) + Q
        
      Note that the identity terms in the B matrix are scaled down to `dt` so as
      to keep the influence of control input somewhat limited and also not strong
      enough to cause feedback oscillations (initiated by offsets). The reality
      is that I'm not smart enough to get it absolutely correct.
      Control input is a 3D vector of accelerations commanded by the control node.
      LQE_INTEGRATOR_ORDER is a compile-time flag from gcc. Check CMakeLists. This
      allows Eigen to statically optimize its matrices. *This* I am smart about :)
    */
    
    float dt = 1.0/estimator_rate_;
    float dt2 = dt*dt/2.0;
    float dt3 = dt*dt*dt/6.0;;
    double meas_sigma = estimator_params_[0];

    // load noise covariances
    if( use_standard_q_ )
    {
      double proc_sigma = estimator_params_[1];
      int r = 11;  // 12 states
      int c = 11;  // 12 states
      Eigen::Vector4d dtvec = {dt3, dt2, dt, 1.0};
      Eigen::Matrix<double, 4, 4> q = dtvec * (proc_sigma*proc_sigma) * dtvec.transpose();
      // fill actual Q matrix by replicating this for all axes
      for(int ridx=0; ridx<4; ridx++)
      {
        for(int cidx=0; cidx<4; cidx++)
        {
          proc_noise_Q_.block<3,3>(ridx*3,cidx*3) = q.coeff(ridx,cidx)*IDEN3x3;
        }
      }
      // trim values in Q that are small enough to be zeros
      proc_noise_Q_ = (proc_noise_Q_.array() < 1e-6).select(0, proc_noise_Q_);
    }
    else
    {
      double sPos = estimator_params_[1];
      double sVel = estimator_params_[2];
      double sAcc = estimator_params_[3];
      double sJrk = estimator_params_[4];
      proc_noise_Q_ <<  sPos*IDEN3x3, ZERO3x3, ZERO3x3, ZERO3x3,
                        ZERO3x3, sVel*IDEN3x3, ZERO3x3, ZERO3x3,
                        ZERO3x3, ZERO3x3, sAcc*IDEN3x3, ZERO3x3,
                        ZERO3x3, ZERO3x3, ZERO3x3, sJrk*IDEN3x3;
    }

    // note that A is a 12x12 matrix regardless (i.e., has jerk states, whose
    // coeffs may or mayn't be zero)
    sys_A_ = Eigen::MatrixXd::Identity(nStates, nStates);
    sys_A_.topRightCorner<3,3>().diagonal().setConstant(dt3);
    sys_A_.topRightCorner<6,6>().diagonal().setConstant(dt2);
    sys_A_.topRightCorner<9,9>().diagonal().setConstant(dt);
    
    sys_A_t_ = sys_A_.transpose();

    sys_B_ << ZERO3x3,
              IDEN3x3,
              ZERO3x3,
              ZERO3x3;
    
    meas_noise_R_ << meas_sigma*IDEN3x3;

    // only positions are measured (e.g., mocap)
    meas_matrix_C_ << IDEN3x3, ZERO3x3, ZERO3x3, ZERO3x3;
    meas_matrix_C_t_ = meas_matrix_C_.transpose();
    
    // guess some initial state, cov 
    best_estimate_.setZero();
    ctrl_input_u_.setZero();
    state_cov_P_ = 100*IDEN12x12;
  }

  /* State propagation, done at a fixed rate on a separate thread.
  On smaller/less-powerful machines, the thread::sleep_for() methods may not
  be highly accurate, and thus, values of "dt" in A and B matrices can't be 
  assumed to be constant (=1.0/estimator_rate). The function measures the time
  since the propagation uses that for "dt". The time it sleeps for is also
  stabilized by accounting for the time it took to run state propagation.
  */
  void KalmanFilter::state_propagation( )
  {
    while( state_propagation_alive_ )
    {
      ts = std::chrono::high_resolution_clock::now();
      prop_interval_ = ts - last_prop_t_;
      double dt = prop_interval_.count();
      double dt2 = 0.5*dt*dt;
      double dt3 = dt*dt*dt/6.0;
      
      sys_A_.topRightCorner<3,3>().diagonal().setConstant(dt3);
      sys_A_.topRightCorner<6,6>().diagonal().setConstant(dt2);
      sys_A_.topRightCorner<9,9>().diagonal().setConstant(dt);
      
      sys_A_t_ = sys_A_.transpose();

      std::unique_lock<std::mutex> spmtx( state_prop_mutex_, std::defer_lock );
      if( spmtx.try_lock() )
      {
        // propagate state forward
        best_estimate_ = sys_A_ * best_estimate_ +  sys_B_ * ctrl_input_u_;
        state_cov_P_ = sys_A_*state_cov_P_*sys_A_t_ + proc_noise_Q_;

        spmtx.unlock();

        last_prop_t_ = te = std::chrono::high_resolution_clock::now();
        int dt = std::chrono::duration_cast<uSeconds>(te-ts).count();
        std::this_thread::sleep_for( uSeconds(estimator_period_us_ - dt) );
      }
      else
        std::this_thread::sleep_for( uSeconds(estimator_period_us_) );
    }
  }

  void KalmanFilter::state_updation()
  {
    /* State update equations:
          G = P_k * C' * (C*P_k*C' + R)'
          x = x + G*(Z - Cx)
          P = (I-GC)*P
      If C = I, speed-up/de-clutter the code here.
    */
    Eigen::MatrixXd innov = ( meas_matrix_C_ * state_cov_P_ * meas_matrix_C_t_ + meas_noise_R_ ).inverse();
    state_prop_mutex_.lock();
      kalman_gain_G_ = state_cov_P_ * meas_matrix_C_t_ * innov;
      best_estimate_ = best_estimate_ + kalman_gain_G_*(measurement_z_ - meas_matrix_C_*best_estimate_);
      state_cov_P_ = (IDEN12x12 - kalman_gain_G_*meas_matrix_C_)*state_cov_P_;
    state_prop_mutex_.unlock();
    //std::cout << "meas applied: " << measurement_z_.transpose() << std::endl;
  }

  void KalmanFilter::setControlInput( const Eigen::Matrix<double, nCtrl, 1> &ctrl_u )
  {
    /* Listening to debug from controller is neat because it contains the raw
      accelerations commanded, and also the feedback linearized attitudes that
      were computed. At the moment, we rely on accelerations, but in the future,
      one could incorporate attitude commands to do more non-linear stuff here.
    */
    // note: X = [pn, pe, pd ..]
    state_prop_mutex_.lock();
      ctrl_input_u_ = std::move(ctrl_u);
      n_stprops_since_update_ = 0;
    state_prop_mutex_.unlock();
  }

  void KalmanFilter::setMeasurementInput( const Eigen::Matrix<double, nMeas, 1> meas )
  {

    // no need to mutexify this, since measurement_z_ is only used in update function
    measurement_z_ = std::move(meas);

    // call state update, reset propagation counter
    state_updation();
    n_stprops_since_update_ = 0;
  }

  void KalmanFilter::setMeasurementInput( const std::vector<double> &meas )
  {
    measurement_z_ << meas[0], meas[1], meas[2];
    state_updation();
    n_stprops_since_update_ = 0;
  }

  /* Return estimated state 9x1 [posNED, velNED, accNed] */
  void KalmanFilter::getStateEstimate( Eigen::Matrix<double, 9, 1> &x_est )
  {
    state_prop_mutex_.lock();
      x_est = best_estimate_.head<9>();
    state_prop_mutex_.unlock();
  }

  void KalmanFilter::getStateEstimate( Eigen::VectorXd &x_est )
  {
    static Eigen::Matrix<double, 9, 1> _x;
    getStateEstimate( _x );
    x_est = _x;
  }

  void KalmanFilter::getStateEstimate( Eigen::VectorXd &x_est, const int &n_states )
  {
    static Eigen::Matrix<double, 9, 1> _x;
    getStateEstimate( _x );
    x_est = _x.head(n_states);
  }

  void KalmanFilter::printStateEstimate()
  {
    state_prop_mutex_.lock();
      std::cout << best_estimate_.head<9>().transpose() << ";" << std::endl;
    state_prop_mutex_.unlock();
  }
}  // namespace
