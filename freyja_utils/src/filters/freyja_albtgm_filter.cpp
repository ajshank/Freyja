namespace freyja_utils
{
  AlphaBetaGammaFilter::AlphaBetaGammaFilter()
  {
    // initialise a random filter
    AlphaBetaGammaFilter(0.8, 0.05, 0.001, 0.01, false);
  }

  AlphaBetaGammaFilter::AlphaBetaGammaFilter(double proc_var, double noise_var, double dt, bool axes_angular)
  {

  }

  AlphaBetaGammaFilter::AlphaBetaGammaFilter( double a, double b, double g, double dt, bool axes_angular )
  {
    alpha_ = a;
    beta_ = b;
    gamma_ = g;
    dt_ = dt;
    axes_angular_ = axes_angular;

    x_.setZero();
    x_dot_.setZero();
    x_ddot_.setZero();
    init();
  }

  void AlphaBetaGammaFilter::init()
  {
    filter_name_ = "albtgm";
    std::cout << "Alpha-Beta-Gamma filter init!" << std::endl;
  }

  void AlphaBetaGammaFilter::setMeasurementInput( const std::vector<double> &m )
  {
    x_meas_ << m[0], m[1], m[2];
    update_filter();
  }

  void AlphaBetaGammaFilter::setMeasurementInput( const Eigen::Vector3d m)
  {
    x_meas_ = m;
    update_filter();
  }

  void AlphaBetaGammaFilter::update_filter()
  {
    x_ = x_ + (dt_)*x_dot_ + (0.5*dt_*dt_)*x_ddot_;

    if( axes_angular_ )
      residual_ = x_meas_.binaryExpr(x_, &fast_approx::angleDiffRad);
    else
      residual_ = x_meas_ - x_;

    x_ += alpha_*residual_;
    x_dot_ += (1.0/dt_)*beta_*residual_;
    x_ddot_ += (2*gamma_/(dt_*dt_))*residual_;
  }

  void AlphaBetaGammaFilter::getStateEstimate( Eigen::VectorXd &x_est )
  {
    x_est = x_;
  }
  
  void AlphaBetaGammaFilter::getStateEstimate( Eigen::VectorXd &x_est, const int& n_states )
  {
    x_est << x_, x_dot_, x_ddot_;
  }

}