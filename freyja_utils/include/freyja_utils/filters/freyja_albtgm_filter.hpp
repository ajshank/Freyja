namespace freyja_utils
{
  class AlphaBetaGammaFilter : public Filter
  {
    double alpha_;
    double beta_;
    double gamma_;
    double dt_;

    Eigen::Vector3d x_, x_dot_, x_ddot_;
    Eigen::Vector3d x_meas_;
    Eigen::Vector3d residual_;
    bool axes_angular_;

    public:
      using Filter::Filter;

      void init();
      AlphaBetaGammaFilter();
      AlphaBetaGammaFilter( double a, double b, double g, double dt, bool axes_angular );
      AlphaBetaGammaFilter( double proc_var, double noise_var, double dt, bool axes_angular );
      
      void update_filter();
      
      void setMeasurementInput( const Eigen::Vector3d ) override;
      void setMeasurementInput( const std::vector<double> & ) override;
      void getStateEstimate( Eigen::VectorXd &x_est ) override;
      void getStateEstimate( Eigen::VectorXd &x_est, const int& n_states ) override;
    };
 }