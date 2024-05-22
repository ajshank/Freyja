/* Implementation of the LQR-feedback loop.

   Note: at this time, the feedback gain K is computed from Matlab. If somebody
   wants to write a solution to the algebraic Riccati equations that compute the
   optimal feedback gain K, please feel absolutely encouraged to do so in the
   init function :)
   
   -- aj / 17th Nov, 2017.
*/

#include "mpc_ctrl_flight_osqp.h"

#define ROS_NODE_NAME "mpc_control"
#define pi 3.1416


LQRController::LQRController(BiasEstimator &b) : Node( ROS_NODE_NAME ),
                                                 bias_est_( b )
{
  int controller_rate_default = 30;
  float mass_default = 0.85;
  
  declare_parameter<int>( "controller_rate", controller_rate_default );
  declare_parameter<float>( "total_mass", mass_default );
  declare_parameter<bool>( "use_stricter_gains", false );
  declare_parameter<bool>( "enable_flatness_ff", false );
  declare_parameter<bool>( "mass_correction", false );
  declare_parameter<bool>( "mass_estimation", true );
  declare_parameter<std::string>( "bias_compensation", "auto" );
  declare_parameter<bool>( "apply_est_biases", false );
  declare_parameter<bool>( "apply_extf_corr", false );
  declare_parameter<double>( "ctrl_horizon", 2.0 );

  declare_parameter<std::string>( "controller_type", "pos-vel" );
  
  get_parameter( "controller_rate", controller_rate_ );
  get_parameter( "total_mass", total_mass_ );
  get_parameter( "use_stricter_gains", use_stricter_gains_ );
  
  
  /* initialise system, matrices and controller configuration */
  initLqrSystem();
  initMPCSystem();
  
  
  /* Associate a subscriber for the current vehicle state */
  state_sub_ = create_subscription<CurrentState>( "current_state", 1,
                  std::bind(&LQRController::stateCallback, this, _1) );
  /* Associate a subscriber for the current reference state */
  reference_sub_ = create_subscription<TrajRef>( "reference_state", 1,
                  std::bind(&LQRController::trajectoryReferenceCallback, this, _1) );
  /* Associate a subscriber for measured external forces */
  extf_sub_ = create_subscription<GeomVec3Stamped>( "pred_ext_forces", 1,
                  std::bind(&LQRController::externalForceCallback, this, _1) );
  
  /* Service provider for bias compensation */
  bias_enable_serv_ = create_service <BoolServ> ( "set_bias_compensation",
                  std::bind(&LQRController::biasEnableServer, this, _1, _2 ) );
  
  
  /* Announce publishers for controller output */
  atti_cmd_pub_ = create_publisher <RPYT_Command>
                    ( "rpyt_command", 1 );
  controller_debug_pub_ = create_publisher <CTRL_Debug>
                    ( "controller_debug", 1 );
  est_mass_pub_ = create_publisher <std_msgs::msg::Float32>
                    ( "freyja_estimated_mass", 1 );

  /* Timer to run the LQR controller perdiodically */
  float controller_period = 1.0/controller_rate_;
  controller_timer_ = rclcpp::create_timer( this, get_clock(), std::chrono::duration<float>(controller_period),
                           std::bind( &LQRController::computeFeedback, this ) );
  
  /* Checks for correctness */
  STATEFB_MISSING_INTRV_ = 0.5;
  have_state_update_ = false;
  have_reference_update_ = false;  
  
  /* Bias compensation parameters */
  std::string _bcomp = "auto";
  get_parameter( "bias_compensation", _bcomp );
  if( _bcomp == "always-on" )
    bias_compensation_req_ = true;                // always on (be careful!!)
  else if( _bcomp == "auto" || _bcomp == "always-off" )
    bias_compensation_req_ = false;               // off, or on by service call
  
  bias_compensation_off_ = (_bcomp == "always-off")? true : false;   // always off
  
  f_biases_.setZero();
  get_parameter( "apply_est_biases", apply_bias_corr_ );

  /* Should we apply external forces provided to controller */
  get_parameter( "apply_extf_corr", apply_extf_corr_ );
  f_ext_.setZero();
  
  /* Differential flatness feed-forward accelerations */
  get_parameter( "enable_flatness_ff", enable_flatness_ff_ );
  reference_ff_.setZero();
  
  
  /* Mass estimation */
  enable_dyn_mass_correction_ = false;
  get_parameter( "mass_correction", enable_dyn_mass_correction_ );
  get_parameter( "mass_estimation", enable_dyn_mass_estimation_ );
  if( enable_dyn_mass_correction_ )
  {
    enable_dyn_mass_estimation_ = true;
    RCLCPP_WARN( get_logger(), "LQR: Mass correction active at init! This is discouraged." );
  }
}

void setDynamicsMatrices(Eigen::Matrix<double, 7, 7> &a, Eigen::Matrix<double, 7, 4> &b)
{
    a = Eigen::MatrixXd::Identity(7, 7);
    double dt = 1/50.0;
    a.topLeftCorner<6,6>().topRightCorner<3,3>().diagonal().setConstant(dt);
    b.setZero();
    b.bottomRightCorner<4,4>().diagonal().setConstant(dt);
}


void setInequalityConstraints(Eigen::Matrix<double, 7, 1> &xMax_, Eigen::Matrix<double, 7, 1> &xMin_,
                              Eigen::Matrix<double, 4, 1> &uMax_, Eigen::Matrix<double, 4, 1> &uMin_)
{
    // state inequality constraints
    xMin_.setConstant(-100);
    xMax_.setConstant(100);
    uMin_.setConstant(-20);
    uMax_.setConstant(20);
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, n_mpc_states_> &Q, Eigen::DiagonalMatrix<double, 4> &R)
{
    Q.diagonal() << 10, 10, 30., 5., 5., 1., 2.0;
    R.diagonal() << 0.1, 0.1, 0.1, 0.1;
}

void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, n_mpc_states_> &Q, const Eigen::DiagonalMatrix<double, 4> &R, int mpcWindow,
                        Eigen::SparseMatrix<double> &hessianMatrix)
{

    hessianMatrix.resize(n_mpc_states_*(mpcWindow+1) + 4 * mpcWindow, n_mpc_states_*(mpcWindow+1) + 4 * mpcWindow);

    //populate hessian matrix
    for(int i = 0; i<n_mpc_states_*(mpcWindow+1) + 4 * mpcWindow; i++){
        if(i < n_mpc_states_*(mpcWindow+1)){
            int posQ=i%n_mpc_states_;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
        else{
            int posR=i%4;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix.insert(i,i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, n_mpc_states_> &Q, const Eigen::Matrix<double, n_mpc_states_, 1> &xRef, int mpcWindow,
                         Eigen::VectorXd &gradient)
{

    Eigen::Matrix<double,n_mpc_states_,1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(n_mpc_states_*(mpcWindow+1) +  4*mpcWindow, 1);
    for(int i = 0; i<n_mpc_states_*(mpcWindow+1); i++){
        int posQ=i%n_mpc_states_;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, n_mpc_states_, n_mpc_states_> &dynamicMatrix, const Eigen::Matrix<double, n_mpc_states_, 4> &controlMatrix,
                                 int mpcWindow, Eigen::SparseMatrix<double> &constraintMatrix)
{
    constraintMatrix.resize(n_mpc_states_*(mpcWindow+1)  + n_mpc_states_*(mpcWindow+1) + 4 * mpcWindow, n_mpc_states_*(mpcWindow+1) + 4 * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<n_mpc_states_*(mpcWindow+1); i++){
        constraintMatrix.insert(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<n_mpc_states_; j++)
            for(int k = 0; k<n_mpc_states_; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(n_mpc_states_ * (i+1) + j, n_mpc_states_ * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < n_mpc_states_; j++)
            for(int k = 0; k < 4; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix.insert(n_mpc_states_*(i+1)+j, 4*i+k+n_mpc_states_*(mpcWindow + 1)) = value;
                }
            }

    for(int i = 0; i<n_mpc_states_*(mpcWindow+1) + 4*mpcWindow; i++){
        constraintMatrix.insert(i+(mpcWindow+1)*n_mpc_states_,i) = 1;
    }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, n_mpc_states_, 1> &xMax_, const Eigen::Matrix<double, n_mpc_states_, 1> &xMin_,
                                   const Eigen::Matrix<double, 4, 1> &uMax_, const Eigen::Matrix<double, 4, 1> &uMin_,
                                   const Eigen::Matrix<double, n_mpc_states_, 1> &x0,
                                   int mpcWindow, Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(n_mpc_states_*(mpcWindow+1) +  4 * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(n_mpc_states_*(mpcWindow+1) +  4 * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
        lowerInequality.block(n_mpc_states_*i,0,n_mpc_states_,1) = xMin_;
        upperInequality.block(n_mpc_states_*i,0,n_mpc_states_,1) = xMax_;
    }
    for(int i=0; i<mpcWindow; i++){
        lowerInequality.block(4 * i + n_mpc_states_ * (mpcWindow + 1), 0, 4, 1) = uMin_;
        upperInequality.block(4 * i + n_mpc_states_ * (mpcWindow + 1), 0, 4, 1) = uMax_;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(n_mpc_states_*(mpcWindow+1),1 );
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0,0,n_mpc_states_,1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(2*n_mpc_states_*(mpcWindow+1) +  4*mpcWindow,1 );
    lowerBound << lowerEquality,
        lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(2*n_mpc_states_*(mpcWindow+1) +  4*mpcWindow,1 );
    upperBound << upperEquality,
        upperInequality;
}


void updateConstraintVectors(const Eigen::Matrix<double, n_mpc_states_, 1> &x0,
                             Eigen::VectorXd &lowerBound, Eigen::VectorXd &upperBound)
{
    lowerBound.block(0,0,n_mpc_states_,1) = -x0;
    upperBound.block(0,0,n_mpc_states_,1) = -x0;
}

void LQRController::initLqrSystem()
{
  /* Implementation in Matlab, the values in lqr_K_ are copy-pasta from there.
    sys_A_ = [..];  (7x7)
    sys_B_ = [..];  (7x4)
    
    lqr_Q_ = [..];  (7x7) Q.diag = [1, 1, 8, 0.01, 0.01, 0.1, 1]
    lqr_R_ = [..];  (4x4) R.diag = [0.8, 0.8, 0.8, 1]
  */
  /* smooth and usually fine:
    lqr_Q_ = [..];  (7x7) Q.diag = [1, 1, 8, 0.01, 0.01, 0.1, 1]
    lqr_R_ = [..];  (4x4) R.diag = [0.8, 0.8, 0.8, 1]
  */
  std::string controller_type;
  get_parameter( "controller_type", controller_type );

  if( controller_type == "pos-vel" )
  {
    lqr_K_ << 1.118, 0.0, 0.0, 1.4995, 0.0, 0.0, 0.0,
            0.0, 1.118, 0.0, 0.00, 1.4995, 0.0, 0.0,
            0.0, 0.0, 3.1623, 0.0, 0.0, 2.5347, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            
    /* little more aggresive:
      lqr_Q_ = [..];  (7x7) Q.diag = [1.2, 1.2, 8.2, 0.02, 0.02, 0.1, 1]
      lqr_R_ = [..];  (4x4) R.diag = [0.8, 0.8, 0.8, 1]
    */
    if( use_stricter_gains_ )
    {
      lqr_K_ << 1.225, 0.0, 0.0, 1.5731, 0.0, 0.0, 0.0,
                0.0, 1.225, 0.0, 0.00, 1.5731, 0.0, 0.0,
                0.0, 0.0, 3.2016, 0.0, 0.0, 2.550, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
      RCLCPP_WARN( get_logger(), "LQR: stricter gains requested!" );
    }
  }
  else if( controller_type == "vel-only" )
  {
    lqr_K_ << 0.0, 0.0, 0.0, 1.4995, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.00, 1.4995, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 2.5347, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    RCLCPP_WARN( get_logger(), "LQR: controlling only 3D velocities." );
  }
  else if( controller_type == "NEvel-Dpos" )
  {
    lqr_K_ << 0.0, 0.0, 0.0, 1.4995, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.00, 1.4995, 0.0, 0.0,
            0.0, 0.0, 3.1623, 0.0, 0.0, 2.5347, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    RCLCPP_WARN( get_logger(), "LQR: controlling 2D velocity and vertical position." );
  }
  else
    RCLCPP_ERROR( get_logger(), "LQR: Controller type unknown!" );
}

void LQRController::initMPCSystem()
{
  // allocate the dynamics matrices; are copied into oqsp-eigen
  Eigen::Matrix<double, n_mpc_states_, n_mpc_states_> sys_A;
  Eigen::Matrix<double, n_mpc_states_, 4> sys_B;

  // allocate the weight matrices; Q is stack allocated for reuse.
  Eigen::DiagonalMatrix<double, 4> sys_R;

  // allocate the initial and the reference state space
  Eigen::Matrix<double, n_mpc_states_, 1> x0;
  Eigen::Matrix<double, n_mpc_states_, 1> xRef;

  // allocate QP problem matrices and vectores
  Eigen::SparseMatrix<double> hessian;
  
  Eigen::SparseMatrix<double> linearMatrix;
  Eigen::VectorXd lowerBound;
  Eigen::VectorXd upperBound;

  // set the initial and the desired states
  x0 << 0, 0, 0, 0, 0, 0, 0;
  xRef <<  0, 0, -1, 0, 0, 0, 0;

  // set MPC problem quantities
  setDynamicsMatrices(sys_A, sys_B);
  setInequalityConstraints(xMax_, xMin_, uMax_, uMin_);
  setWeightMatrices(sys_Q_, sys_R);

  // cast the MPC problem as QP problem
  castMPCToQPHessian(sys_Q_, sys_R, mpcWindow_, hessian);
  castMPCToQPGradient(sys_Q_, xRef, mpcWindow_, gradient_);
  castMPCToQPConstraintMatrix(sys_A, sys_B, mpcWindow_, linearMatrix);
  castMPCToQPConstraintVectors(xMax_, xMin_, uMax_, uMin_, x0, mpcWindow_, lb_, ub_);

  // settings
  mpcSolver_.settings()->setVerbosity(false);
  mpcSolver_.settings()->setWarmStart(true);

  // set the initial data of the QP mpcSolver_
  mpcSolver_.data()->setNumberOfVariables(n_mpc_states_ * (mpcWindow_ + 1) + 4 * mpcWindow_);
  mpcSolver_.data()->setNumberOfConstraints(2 * n_mpc_states_ * (mpcWindow_ + 1) + 4 * mpcWindow_);
  int setup_errs = 0;
  if(!mpcSolver_.data()->setHessianMatrix(hessian)) setup_errs++;
  if(!mpcSolver_.data()->setGradient(gradient_)) setup_errs++;
  if(!mpcSolver_.data()->setLinearConstraintsMatrix(linearMatrix)) setup_errs++;
  if(!mpcSolver_.data()->setLowerBound(lb_)) setup_errs++;
  if(!mpcSolver_.data()->setUpperBound(ub_)) setup_errs++;

  // instantiate the mpcSolver_
  if(setup_errs>0 || !mpcSolver_.initSolver())
    RCLCPP_ERROR(get_logger(), "Could not initialise OSQP! Err count: %d", setup_errs);
  else
    RCLCPP_INFO(get_logger(), "MPC Node is ready!");
}

void LQRController::biasEnableServer( const BoolServ::Request::SharedPtr rq,
                                      const BoolServ::Response::SharedPtr rp )
{
  if( bias_compensation_off_ )
  {
    RCLCPP_WARN( get_logger(), "LQR: Bias compensation remains off throughout." );
    rp -> success = false;       // service unsuccessful
    return;
  }
  
  bias_compensation_req_ = rq -> data;
  if( bias_compensation_req_ )
  {
    RCLCPP_WARN( get_logger(), "LQR: Bias compensation active!" );
    bias_est_.enable();
  }
  else
  {
    RCLCPP_WARN( get_logger(), "LQR: Bias compensation inactive!" );
    bias_est_.disable();
  }
  rp -> success = true;  // service successful
}

constexpr double LQRController::calcYawError( const double &a, const double &b )
{
  double yd = fmod( a - b + pi, 2*pi );
  yd = yd < 0 ? yd+2*pi : yd;
  return yd-pi; 
}

void LQRController::stateCallback( const CurrentState::ConstSharedPtr msg )
{
  /* Parse message to obtain state and reduced state information */
  //static Eigen::Matrix<double, 7, 1> current_state_;
  static Eigen::Matrix<double, 7, 1> state_err;

  float yaw = msg->state_vector[8];
  rot_yaw_ << std::cos(yaw), std::sin(yaw), 0,
             -std::sin(yaw), std::cos(yaw), 0,
              0, 0, 1;
             
  /* state is the first 6 elements, and yaw */
  current_state_ << Eigen::Map<const PosVelNED>( msg->state_vector.data() ),
                   double(yaw);

  /* compute x - xr here in case we default to lqr later */
  std::unique_lock<std::mutex> rsmtx( reference_state_mutex_, std::defer_lock );
  if( have_reference_update_ )
  {
    rsmtx.lock();
    state_err.head<6>() = current_state_.head<6>() - reference_state_.head<6>();
    /* yaw-error is done differently */
    state_err(6) = calcYawError( current_state_(6), reference_state_(6) );
    rsmtx.unlock();
    reduced_state_ = std::move( state_err );
  }

  last_state_update_t_ = now();
  have_state_update_ = true;
}

void LQRController::trajectoryReferenceCallback( const TrajRef::ConstSharedPtr msg )
{
  /* Simply copy over the reference state. Note that this might happen quite
    frequently (I expect a well-written trajectory provider to be using a nearly
    "continous" time update to the reference state). Any optimizations here are
    greatly welcome.
    Not currently using accelerations for FF.
  */
  reference_state_mutex_.lock();
  reference_state_ << msg->pn, msg->pe, msg->pd, msg->vn, msg->ve, msg->vd, msg->yaw;
  reference_state_mutex_.unlock();
  
  reference_ff_ << msg->an, msg->ae, msg->ad, 0.0;    // only NED accelerations
  have_reference_update_ = true;
}

void LQRController::externalForceCallback( const GeomVec3Stamped::ConstSharedPtr msg )
{
  f_ext_ << msg->vector.x, msg->vector.y, msg->vector.z;
}

__attribute__((optimize("unroll-loops")))
void LQRController::computeFeedback()
{
  /* Wait for atleast one update, or architect the code better */
  if( !have_state_update_ || !have_reference_update_ )
    return;
  
  float roll, pitch, yaw;
  double T;
  static Eigen::Matrix<double, 4, 1> control_input_;
  static Eigen::Matrix<double, 7, 1> state_err;
  
  bool state_valid = true;
  auto tnow = now();
  
  /* Check the difference from last state update time */
  float state_age = (tnow - last_state_update_t_).seconds();
  if( state_age > STATEFB_MISSING_INTRV_ )
  {
    /* State feedback missing for more than specified interval. Try descend */
    roll = pitch = yaw = 0.0;
    T = (total_mass_ * 9.81) - 0.5;
    control_input_.setZero();
    state_valid = false;
  }
  else
  {
    /* Compute control inputs (accelerations, in this case) */
    bool mpcsol_valid = getMPCSolution(control_input_);
    
    /* Default to an LQR control if mpc-solver has failed for some reason */
    if(!mpcsol_valid)
    {
      state_err = std::move( reduced_state_ );
      control_input_ = -1 * lqr_K_ * state_err 
                    + static_cast<double>(enable_flatness_ff_) * reference_ff_;
      RCLCPP_ERROR_THROTTLE(get_logger(), *(get_clock()), 250,
                             "OSQP solver failed; reverting to LQR!");
    }
  
    /* Force saturation on downward acceleration */
    control_input_(2) = std::min( control_input_(2), 8.0 );
    control_input_(2) -= 9.81;
    
    /* See if bias compensation was requested .. */
    if( bias_compensation_req_ )
    {
      bias_est_.getEstimatedBiases( f_biases_ );
      control_input_.head<3>() -= (apply_bias_corr_ * f_biases_.head<3>() );
    }

    /* Correct for external forces */
    control_input_.head<3>() -= (apply_extf_corr_ * f_ext_.head<3>() );
  
    /* Thrust */
    T = total_mass_ * control_input_.head<3>().norm();
  
    /* Roll, pitch and yawrate */
    Eigen::Matrix<double, 3, 1> Z = rot_yaw_ * control_input_.head<3>() * (-total_mass_/T);
    roll = std::asin( -Z(1) );
    pitch = std::atan( Z(0)/Z(2) );
    yaw = control_input_(3);
  }

  /* Actual commanded input */
  RPYT_Command ctrl_cmd;
  ctrl_cmd.roll = roll;
  ctrl_cmd.pitch = pitch;
  ctrl_cmd.yaw = yaw;
  ctrl_cmd.thrust = T;
  ctrl_cmd.ctrl_mode = 0b00001111;
  atti_cmd_pub_ -> publish( ctrl_cmd );

  /* Debug information */
  static CTRL_Debug debug_msg;
  debug_msg.header.stamp = tnow;
  for( uint8_t idx=0; idx<4; idx++ )
    debug_msg.lqr_u[idx] = static_cast<float>(control_input_.coeff(idx));
  for( uint8_t idx=0; idx<3; idx++ )
    debug_msg.biasv[idx] = static_cast<float>(f_biases_.coeff(idx));
  for( uint8_t idx=0; idx<3; idx++ )
    debug_msg.ext_f[idx] = static_cast<float>(f_ext_.coeff(idx));
  for( uint8_t idx=0; idx<7; idx++ )
    debug_msg.errv[idx] = static_cast<float>(state_err.coeff(idx));

  debug_msg.flags = (debug_msg.BIAS_EN * bias_compensation_req_) |
                    (debug_msg.MASS_CR * enable_dyn_mass_correction_ ) |
                    (debug_msg.FLAT_FF * enable_flatness_ff_ ) |
                    (debug_msg.CTRL_OK * state_valid );
  controller_debug_pub_ -> publish( debug_msg ); 
}

bool LQRController::getMPCSolution(Eigen::Vector4d &ctrl)
{
  /* Update gradient computed from new refstate  */
  castMPCToQPGradient(sys_Q_, reference_state_, mpcWindow_, gradient_);
  auto grad_upd_ok = mpcSolver_.updateGradient(gradient_);

  /* update constraints (could be redundant; @TODO) */
  // castMPCToQPConstraintVectors(xMax_, xMin_, uMax_, uMin_, current_state_, mpcWindow_, lb_, ub_);
  updateConstraintVectors(current_state_, lb_, ub_);
  auto bound_upd_ok = mpcSolver_.updateBounds(lb_, ub_);
  
  /* solve problem and get solution */
  auto mpc_ok = mpcSolver_.solveProblem();
  Eigen::VectorXd QPSolution = mpcSolver_.getSolution();
  ctrl = QPSolution.block(n_mpc_states_ * (mpcWindow_ + 1), 0, 4, 1);
  return grad_upd_ok && bound_upd_ok && (mpc_ok == OsqpEigen::ErrorExitFlag::NoError);
}



void LQRController::estimateMass( const Vector4d &c, rclcpp::Time &t )
{
  /* Experimental module that estimates the true mass of the system in air.
    It uses the provided mass and observes the deviation from expected output
    of the controller - and attributes *all* of that error to an incorrect
    mass parameter. This is recommended only if mass may be off by ~200-300g.
    Errors larger than that can induce major second-order oscillations, and are
    usually better addressed elsewhere in the architecture (or get a better scale).
  */
  static float prev_estimated_mass = total_mass_;
  static std_msgs::msg::Float32 estmass;
  static rclcpp::Time t_last = now();

  if( (t-t_last).seconds() < 3.0 )
    return;

  float ctrl_effort = c(2) + 9.81;
  float estimated_mass = total_mass_*(9.81 - ctrl_effort)/9.81;
  // basic low-pass filter to prevent wild jitter
  estimated_mass = (prev_estimated_mass + estimated_mass)/2.0;
  
  estmass.data = estimated_mass;
  est_mass_pub_ -> publish( estmass );
  
  /* if correction is allowed- clip between min and max */
  if( enable_dyn_mass_correction_ )
    total_mass_ = std::min( 2.0f, std::max( 0.8f, estimated_mass ) );
  
  // book-keeping
  t_last = t;
  prev_estimated_mass = estimated_mass;
}

int main( int argc, char** argv )
{
  rclcpp::init( argc, argv );
  BiasEstimator estimator;
  rclcpp::spin( std::make_shared<LQRController>(estimator) );
  rclcpp::shutdown();
  return 0;
}
