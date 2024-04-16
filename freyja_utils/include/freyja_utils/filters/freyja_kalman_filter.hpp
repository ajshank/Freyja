namespace freyja_utils
{
  const int nStates = 12;
  const int nCtrl = 3;
  const int nMeas = 3;
  class KalmanFilter : public Filter
  {
    // state matrices
    Eigen::Matrix<double, nStates, nStates> sys_A_, sys_A_t_;
    Eigen::Matrix<double, nStates, nCtrl> sys_B_;
    Eigen::Matrix<double, nStates, nStates> state_cov_P_;
    Eigen::Matrix<double, nStates, nStates> proc_noise_Q_;
    Eigen::Matrix<double, nMeas, nMeas> meas_noise_R_;
    Eigen::Matrix<double, nStates, nMeas> kalman_gain_G_;
    
    Eigen::Matrix<double, nStates, 1> best_estimate_;
    Eigen::Matrix<double, nCtrl, 1> ctrl_input_u_;
    Eigen::Matrix<double, nMeas, 1> measurement_z_;
    Eigen::Matrix<double, nMeas, nStates> meas_matrix_C_;
    Eigen::Matrix<double, nStates, nMeas> meas_matrix_C_t_;

    // Eigen::Matrix<double, 6, 3> timing_matrix_;
    
    // more parameters
    int estimator_rate_;
    int estimator_period_us_;
    int user_nstates_;          // number of states user code is workin' with
    bool use_standard_q_;
    std::vector<double> estimator_params_;

    // prevent state propagation when other callbacks are happening
    std::mutex state_prop_mutex_;
    std::thread state_prop_thread_;
    volatile bool state_propagation_alive_;
    volatile int n_stprops_since_update_;
    
    // timing related objects
    std::chrono::time_point<std::chrono::high_resolution_clock> ts, te, last_prop_t_;
    std::chrono::duration<double> prop_interval_;
    
    public:
      KalmanFilter(int estimator_rate, const std::vector<double> params);
      ~KalmanFilter();
      
      void init();
      void init( int estimator_rate, const std::vector<double>& params );

      // measurement input: z
      void setMeasurementInput( const Eigen::Matrix<double, nMeas, 1> ) override;
      void setMeasurementInput( const std::vector<double> & ) override;

      // control update
      void setControlInput( const Eigen::Matrix<double, nCtrl, 1> & );

      // run state propagation : separate thread
      void state_propagation( );

      // run state updation when required
      void state_updation();

      // init values
      void initLqeSystem();

      // accessor for state estimate
      void getStateEstimate( Eigen::Matrix<double, 9, 1> & );
      void getStateEstimate( Eigen::VectorXd& ) override;
      void getStateEstimate( Eigen::VectorXd&, const int& ) override;
      void printStateEstimate();
  };
}
