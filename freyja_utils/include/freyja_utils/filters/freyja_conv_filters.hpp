namespace freyja_utils
{
  class ConvFilters : public Filter
  {
    std::vector<double> filter_coeffs_;
    double weight_scaler_;

    public:
      using Filter::Filter;
      ConvFilters() { }
      ConvFilters( const std::string &f_name, const unsigned int& len );
      ConvFilters( const std::string&, const unsigned int&, const std::vector<double> );
      void init();
      void initGenericFilter( const std::string&, const int&, const std::vector<double>&, double );
      
      void filterObservations( const Eigen::VectorXd &obs, double &retVal ) override {}
      void filterObservations( const Eigen::MatrixXd &obs, Eigen::VectorXd &retVal ) override {}
      void filterObservations( const std::vector<double> &obs, double &retVal ) override;
      void filterObservations(  const std::vector<double> &obs1, 
                                const std::vector<double> &obs2, 
                                const std::vector<double> &obs3, 
                                double &retVal1,
                                double &retVal2,
                                double &retVal3 ) override;
    };
 }
