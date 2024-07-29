
#include "state_manager.hpp"
#include "callback_implementations.cpp"

#define rclcpp_NODE_NAME "state_manager"

StateManager::StateManager() : Node( rclcpp_NODE_NAME )
{
  state_vector_.resize( STATE_VECTOR_LEN );
  pose_filter_ = nullptr;
  rate_filter_ = nullptr;
  
  /* find out what the source of state information is:
      > "vicon" for a motion capture system with ENU frame
      > "asctec" for Ascending Tech autopilots (see corresponding handler)
      > "apm" for ardupilot (uses mavros, see corresponding handler)
      > "onboard_camera" for a downward-facing camera (pose+velocity)
  */ 
  
  declare_parameter<std::string>( "state_source", "mocap" );
  declare_parameter<std::string>( "filter_type", "median" );
  declare_parameter<int>( "filter_length", 21 );
  declare_parameter<std::vector<double>> ("kf_params", std::vector<double>({0.1, 0.1, 0.2, 0.5}));
  declare_parameter<bool>("enable_depr_statepub", false);


  get_parameter( "state_source", state_source_ );
  get_parameter( "filter_type", filter_type_ );
  get_parameter( "filter_length", filter_len_ );
  get_parameter( "enable_depr_statepub", enable_depr_statepub_ );
  
  if( state_source_ == "mocap" )
    initMocapManager();
  else if( state_source_ == "tf-mocap" )
    initTfManager();
  else if (state_source_ == "apm" )
     initPixhawkManager();

  is_markov_filter_ = false;

  /* Announce state publisher */
  state_pub_ = create_publisher <CurrentState> ( "current_state", 1 ); 
  named_state_pub_ = create_publisher <CurrentStateNamed> ( "current_state_named", 1 ); 
  

  if( filter_type_ == "gauss" )
  {
    std::vector<double> fc;
    /* Init filter: mean 10, stddev = 5 */
    if( filter_len_ == 21 )
      fc = { 0.0108, 0.0158, 0.0222, 0.0299, 0.0388, 0.0484, 0.0579, 0.0666,
             0.0737, 0.0782, 0.0798, 0.0782, 0.0737, 0.0666, 0.0579, 0.0484,
             0.0388, 0.0299, 0.0222, 0.0158, 0.0108 };
    else if( filter_len_ == 5 )
      fc = { 0.1534,	0.2214,	0.2504,	0.2214,	0.1534 };

    pose_filter_ = std::make_shared<freyja_utils::ConvFilters>( "gauss", filter_len_, fc );
    rate_filter_ = std::make_shared<freyja_utils::ConvFilters>( "gauss", filter_len_, fc );
    RCLCPP_INFO( get_logger(), "Gaussian filter init!" );
  }
  else if( filter_type_ == "lwma" )
  {
    /* The init function automatically fills in the coeffs for lwma */
    pose_filter_ = std::make_shared<freyja_utils::ConvFilters>( "lwma-cubic", filter_len_ );
    rate_filter_ = std::make_shared<freyja_utils::ConvFilters>( "lwma-cubic", filter_len_ );
    RCLCPP_INFO( get_logger(), "LWMA filter init!" );
  }
  else if( filter_type_ == "median" )
  {
    pose_filter_ = std::make_shared<freyja_utils::MedianFilter>();
    rate_filter_ = std::make_shared<freyja_utils::MedianFilter>();
    filter_len_ = pose_filter_->getFilterLen();
  }
  else if( filter_type_ == "kalman" )
  {
    std::vector<double> kparams;
    get_parameter("kf_params", kparams);
    pose_filter_ = std::make_shared<freyja_utils::KalmanFilter>(200, kparams);
    ang_filter_ = std::make_shared<freyja_utils::AlphaBetaGammaFilter>();
    is_markov_filter_ = true;
    RCLCPP_INFO( get_logger(), "Kalman filter init!" );
    filter_len_ = 0;
  }
  else if( filter_type_ == "albtgm" )
  {
    pose_filter_ = std::make_shared<freyja_utils::AlphaBetaGammaFilter>();
    is_markov_filter_ = true;
    filter_len_ = 0;
    RCLCPP_INFO( get_logger(), "Alpha-beta-gamma filter init!" );
  }
  else
    RCLCPP_WARN( get_logger(), "No filter initialised by Freyja." );
  
  // init history containers
  if( !is_markov_filter_ )
  {
    prev_pn_.resize( filter_len_ );
    prev_pe_.resize( filter_len_ );
    prev_pd_.resize( filter_len_ );
  
    prev_vn_.resize( filter_len_ );
    prev_ve_.resize( filter_len_ );
    prev_vd_.resize( filter_len_ );
  }
  lastUpdateTime_ = now();
  have_location_fix_ = false;
}

void StateManager::initMocapManager()
{
  declare_parameter<std::string>( "mocap_topic",  "/vicon/FENRIR/FENRIR" );
  std::string mocap_topic;

  get_parameter( "mocap_topic", mocap_topic );
  /* Associate mocap callback */
  mocap_data_sub_ = create_subscription<TFStamped> ( mocap_topic, 1,
                                    std::bind( &StateManager::mocapCallback, this, _1 ) );
}

void StateManager::initTfManager()
{
  declare_parameter<int>( "tf_rate", 180 );
  declare_parameter<int>( "tf_buffer_time", 2 );
  declare_parameter<std::string>( "tf_baseframe", "map" );
  declare_parameter<std::string>( "tf_myframe", "agent_1" );
  
  int tf_lookup_rate, tf_buffer_time;
  float tf_timer_freq;
  auto tf_qos = rclcpp::QoS(10); //.best_effort().durability_volatile();
  
  get_parameter( "tf_rate", tf_lookup_rate );
  get_parameter( "tf_buffer_time", tf_buffer_time );
  get_parameter( "tf_baseframe", tf_base_frame_ );
  get_parameter( "tf_myframe", tf_my_frame_ );

  RCLCPP_INFO( get_logger(), "Registered tf frames: [%s] to [%s]", tf_base_frame_.c_str(), tf_my_frame_.c_str() );
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>( get_clock(), std::chrono::seconds(tf_buffer_time) );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_, rclcpp::Node::make_shared("_"), true, tf_qos );
  
  // create fixed-rate timer
  tf_timer_ = rclcpp::create_timer( this, get_clock(), std::chrono::duration<float>(1.0/tf_lookup_rate),
                                    std::bind(&StateManager::timerTfCallback, this) );

}

// void StateManager::initAsctecManager()
// {
//   declare_parameter<std::string>( "vehicle_topic", "/asctec_onboard_data" );
//   std::string vehicle_topic;
//   get_parameter( "vehicle_topic", vehicle_topic );
//   /*Associate a vehicle data callback */
//   asctec_data_sub_ = create_subscription<freyja_msgs::msg::AsctecData>( vehicle_topic, 1,
//                                 std::bind(&StateManager::asctecDataCallback, this, _1) );
// }

void StateManager::initPixhawkManager()
{
  declare_parameter<bool>( "use_zvel_from_ap", true );
  declare_parameter<bool>( "use_rtkbaseframe", false );
  declare_parameter<std::string>( "rtkbase_offset_type", "base" );

  get_parameter( "use_zvel_from_ap", use_zvel_from_ap_ );
  get_parameter( "use_rtkbaseframe", use_rtkbaseframe_ );
  get_parameter( "rtkbase_offset_type", rtkbase_offset_type_ );

  have_arming_origin_ = false;
  // mavros_gpsraw_sub_ = create_subscription( "/mavros/global_position/global", 1,
  //                               std::bind(&StateManager::mavrosGpsRawCallback, this, _1 );
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
  mavros_gpsodom_sub_ = create_subscription<nav_msgs::msg::Odometry> ( "mavros/global_position/local", qos,
                                std::bind( &StateManager::mavrosGpsOdomCallback, this, _1 ) );
  compass_sub_ = create_subscription<std_msgs::msg::Float64> ( "mavros/global_position/compass_hdg", qos, 
				                        std::bind( &StateManager::mavrosCompassCallback, this, _1 ) );
				                
  maplock_srv_ = create_service <BoolServ> ( "lock_arming_mapframe", 
                        std::bind(&StateManager::maplockArmingHandler, this, _1, _2 ) );

  if( !use_zvel_from_ap_ )
  {
    /* We're asked not to trust z-axis vel from autopilot; must estimate it ourselves.
       Since positions are likely produced by an EKF onboard, we'll use a simple
       gauss filter here.
    */
    RCLCPP_INFO( get_logger(), "Using gauss filter to estimate z-vel!" );
    std::vector<double> fc{ 0.1534,	0.2214,	0.2504,	0.2214,	0.1534 };
    int flen = 5;
    rate_filter_ = std::make_shared<freyja_utils::ConvFilters>( "gauss", flen, fc );
    prev_vd_.resize( flen );
    last_pd_ = 0.0;
    lastUpdateTime_ = now();
  }
  
  std::string bframe_topic = "ubxf9p_rtkbase_offset";
  if( rtkbase_offset_type_ == "custom" )
  {
    bframe_topic = "custom_rtkbase_offset";
    RCLCPP_WARN( get_logger(), "Using custom base-frame!" );
  }

  rtkbase_offs_sub_ = create_subscription<GeomVec3> ( bframe_topic, 1,
                                std::bind( &StateManager::rtkBaselineCallback, this, _1 ) );

  /* initialise containers to avoid garbage */
  map_rtk_pose_.setZero();
  gps_odom_pose_.setZero();
  arming_gps_pose_.setZero();
  rtkbase_offsets_.setZero();
}

// void StateManager::initCameraManager()
// {
//   camera_estimate_sub_ = create_subscription( "/onboard_camera/position_velocity", 1,
//                                 std::bind(&StateManager::cameraUpdatesCallback, this, _1 );
// }


int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<StateManager>() );
  rclcpp::shutdown();
  return 0;
}
