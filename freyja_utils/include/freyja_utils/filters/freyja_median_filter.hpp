namespace freyja_utils
{
  #ifndef MEDIAN_WINDOW
    #define MEDIAN_WINDOW 13
  #endif

  class MedianFilter : public Filter
  {
    StaticSort<MEDIAN_WINDOW> static_sorter_;
    std::vector<double> obs_vec_;
    bool using_static_sort_;
    int median_idx_;

    std::function<void(std::vector<double>&)> sorter_;

    public:
      using Filter::Filter;
    
      MedianFilter( );
      MedianFilter( const int &len );
      int getFilterLen() { return Filter::filter_len_; }
      void init();
      void filterObservations( const std::vector<double> &obs, double &retVal ) override;
      void filterObservations( const std::vector<double> &obs1, 
                                const std::vector<double> &obs2, 
                                const std::vector<double> &obs3, 
                                double &retVal1,
                                double &retVal2,
                                double &retVal3 ) override;
  };
}
