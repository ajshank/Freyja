void StateManager::mocapCallback( const TFStamped::ConstSharedPtr msg )
{
  /* Handle most of the state information directly, except for velocity
    and that needs to be computed after filtering positions */
  
  /* Note that Vicon gives us ENU, while we use NED */
  
  static CurrentState state_msg;
  double time_since = (now() - lastUpdateTime_).seconds();
  static Eigen::VectorXd best_estimate_;

  if( use_kf_ )
  {
    static Eigen::Matrix<double, 3, 1> meas_z_;

    meas_z_ <<  msg->transform.translation.y,
                msg->transform.translation.x,
                -msg->transform.translation.z;

    pose_filter_->setMeasurementInput( meas_z_ );
    pose_filter_->getStateEstimate( best_estimate_, 9 );
    for( int idx=0; idx<6; idx++ )
      state_vector_[idx] = best_estimate_.coeff(idx);
  }
  else
  {
    double x, y, z;
    x = msg -> transform.translation.y;
    y = msg -> transform.translation.x;
    z = -(msg -> transform.translation.z);

    prev_pn_.erase( prev_pn_.begin() );
    prev_pn_.push_back( x );
    prev_pe_.erase( prev_pe_.begin() );
    prev_pe_.push_back( y );
    prev_pd_.erase( prev_pd_.begin() );
    prev_pd_.push_back( z );

    pose_filter_->filterObservations( prev_pn_, prev_pe_, prev_pd_, x, y, z );

    /* positions */
    state_vector_[0] = x;
    state_vector_[1] = y;
    state_vector_[2] = z;
    
    /* velocities */
    double vx, vy, vz;
    vx = ( x - last_pn_ )/time_since; //[3]
    vy = ( y - last_pe_ )/time_since; //[4]
    vz = ( z - last_pd_ )/time_since;
    prev_vn_.erase( prev_vn_.begin() );
    prev_vn_.push_back( vx );
    prev_ve_.erase( prev_ve_.begin() );
    prev_ve_.push_back( vy );
    prev_vd_.erase( prev_vd_.begin() );
    prev_vd_.push_back( vz );

    rate_filter_->filterObservations( prev_vn_, prev_ve_, prev_vd_, vx, vy, vz );

    state_vector_[3] = vx;
    state_vector_[4] = vy;
    state_vector_[5] = vz;
  }
  /* rpy */
  tf2::Quaternion q;
  double roll, pitch, yaw;

  // Equivalent / alternative style:
  //    tf2::fromMsg( msg -> transform.rotation, q );
  //    tf2::Matrix3x3(q).getRPY( roll, pitch, yaw );
  tf2::convert( msg -> transform.rotation, q );
  tf2::impl::getEulerYPR( q, yaw, pitch, roll );

  yaw = F_PI/2.0 - yaw;         // convert ENU yaw to NED yaw
  state_vector_[6] = roll;
  state_vector_[7] = pitch;
  state_vector_[8] = yaw;

  
  /* rpy rates (no filter yet, use with caution!) */
  if( use_kf_ )
  {
    state_vector_[9] = best_estimate_.coeff(6);    //  ( roll - last_roll_ )/time_since;
    state_vector_[10] = best_estimate_.coeff(7);   //  ( pitch - last_pitch_ )/time_since;
    state_vector_[11] = best_estimate_.coeff(8);   //  std::fmod( (yaw - last_yaw_)/time_since, 2*F_PI );
  }
  /* age of this data */
  state_vector_[12] = time_since;
  
  /* Update the current time this happened */
  lastUpdateTime_ = now();
  
  /* bookkeeping stuff */
  last_pn_ = state_vector_[0];
  last_pe_ = state_vector_[1];
  last_pd_ = state_vector_[2];
  last_roll_ = roll;
  last_pitch_ = pitch;
  last_yaw_ = yaw;

  for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
    state_msg.state_vector[idx] = state_vector_[idx];
  
  state_msg.state_valid = 1;
  /* Copy over and publish right away */
  state_msg.header.stamp = now();
  state_pub_ -> publish( state_msg );
}

void StateManager::timerTfCallback()
{
  /* This is a timer callback */
  static TFStamped tform;
  static std::string err_msg;
  
  if( tf_buffer_ -> canTransform( tf_my_frame_, tf_base_frame_, tf2::TimePointZero, tf2::Duration(0), &err_msg ) )
  {
    tform = tf_buffer_ -> lookupTransform( tf_base_frame_, tf_my_frame_, tf2::TimePointZero );
    mocapCallback( std::make_shared<const TFStamped>( tform ) );
  }
  else
    RCLCPP_WARN_THROTTLE( get_logger(), *(get_clock()), 500, "Freyja: TF warn: %s", err_msg.c_str() );

}

// void StateManager::asctecDataCallback( const freyja_msgs::AsctecData::ConstPtr &msg )
// {
//   double time_since = (ros::Time::now() - lastUpdateTime_).toSec();
//   double x, y, z, vx, vy, vz;
  
//   if( (msg -> motor1rpm) > 0 && !have_location_fix_ )
//   {
//     have_location_fix_ = true;
//     home_lat_ = (msg -> best_lat)/10000000.0;
//     home_lon_ = (msg -> best_lon)/10000000.0;
//     ROS_INFO( "[StateManager]: Asctec: Home location set!" );
//   }
  
//   if( !have_location_fix_ )
//     return;
  
//   x = ( (msg->best_lat)/10000000.0 - home_lat_ )*111050.51;
//   y = ( (msg->best_lon)/10000000.0 - home_lon_ )*84356.28;
//   z = -(msg -> hgt)/1000.0;
  
//   vy = (msg -> best_sp_x)/1000.0;
//   vx = (msg -> best_sp_y)/1000.0;
//   vz = -(msg -> best_sp_z)/1000.0;
//   prev_vn_.erase( prev_vn_.begin() );
//   prev_vn_.push_back( vx );
//   prev_ve_.erase( prev_ve_.begin() );
//   prev_ve_.push_back( vy );
//   prev_vd_.erase( prev_vd_.begin() );
//   prev_vd_.push_back( vz );
//   rate_filter_.filterObservations( prev_vn_, prev_ve_, prev_vd_, vx, vy, vz );

//   /* positions */
//   state_vector_[0] = x;
//   state_vector_[1] = y;
//   state_vector_[2] = z;
  
//   /* velocities from vehicle */
//   state_vector_[3] = vx; //(msg -> best_sp_x)/1000.0;
//   state_vector_[4] = vy; //(msg -> best_sp_y)/1000.0;
//   state_vector_[5] = vz; //(msg -> best_sp_z)/1000.0;
  
//   /* Attitude */
//   state_vector_[6] = (msg -> roll_angle)/1000.0;
//   state_vector_[7] = (msg -> pitch_angle)/1000.0;
//   double yaw_c = DEG2RAD( (msg -> yaw_angle)/1000.0 );
//   state_vector_[8] = ( yaw_c > F_PI )? yaw_c - 2*F_PI : yaw_c;
  
//   /* rpy rates */
//   state_vector_[9] = 0.0;
//   state_vector_[10] = 0.0;
//   state_vector_[11] = 0.0;
  
//   /* age */
//   state_vector_[12] = time_since;
  
//   /* Update the current time this happened */
//   lastUpdateTime_ = ros::Time::now();
  
//   /* bookkeeping stuff */
//   last_pn_ = state_vector_[0];
//   last_pe_ = state_vector_[1];
//   last_pd_ = state_vector_[2];
  
//   /* Copy over and publish right away */
//   freyja_msgs::CurrentState state_msg;
//   state_msg.header.stamp = ros::Time::now();
//   for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
//     state_msg.state_vector[idx] = state_vector_[idx];
//   state_pub_.publish( state_msg );
// }

// void StateManager::mavrosGpsRawCallback( const sensor_msgs::NavSatFix::ConstPtr& msg )
// {
//   /*
//   if( msg -> status.status >= 0 && !have_location_fix_ )
//   {
//     // first location fix
//     home_lat_ = msg -> latitude;
//     home_lon_ = msg -> longitude;
//     have_location_fix_ = true;
//     ROS_INFO( "[StateManager]: APM: Home location set!" );
//   }
//   else
//     return;

//   state_vector_[0] = ( (msg->latitude)/10000000.0 - home_lat_ )*111050.51;
//   state_vector_[1] = ( (msg->longitude)/10000000.0 - home_lon_ )*84356.28;
//   */
// }



void StateManager::mavrosCompassCallback( const std_msgs::msg::Float64::ConstSharedPtr msg )
{
  compass_yaw_ = msg -> data;
}

void StateManager::mavrosGpsOdomCallback( const nav_msgs::msg::Odometry::ConstSharedPtr msg )
{
  /* Callback for odometry direct from the Pixhawk.
  Note that this is Pixhawk's best estimate of where it is in the world,
  there is no need to filter it, only structure it to our format.
  Likely topics:
        /mavros/global_position/local --> does not zero at arming.
        /mavros/local_position/local  --> zeros at arming, has IMU frame quirk.
  */
  
  static CurrentState state_msg;
  static Eigen::Matrix<double, 6, 1> pos_vel_;
  
  // update containers anyway (needed for capturing arming location)
  gps_odom_pose_ << msg -> pose.pose.position.y,
                    msg -> pose.pose.position.x,
                   -( msg -> pose.pose.position.z );
  
  if( !have_arming_origin_ )
  {
    RCLCPP_INFO_THROTTLE( get_logger(), *(get_clock()), 2500, "Waiting for arming .. state may have discont." );
    state_msg.state_valid = 0;
  }
  else
    state_msg.state_valid = 1;
  
  // armed at this point
  pos_vel_.head<3>() = gps_odom_pose_ + map_rtk_pose_ - arming_gps_pose_;
  pos_vel_.tail<3>() << msg -> twist.twist.linear.y,
                        msg -> twist.twist.linear.x;
                        ( msg -> twist.twist.linear.z );  // @TODO unclear what frame this is in from mavros
  
  for( unsigned int idx=0; idx<6; idx++ )
    state_msg.state_vector[idx] = pos_vel_[idx];

  state_msg.state_vector[8] = DEG2RAD( compass_yaw_ );
  
  state_msg.header.stamp = now();
  state_pub_ -> publish( state_msg );
}


void StateManager::mavrosRtkBaselineCallback( const geometry_msgs::msg::Vector3::ConstSharedPtr msg )
{
  rtk_baseoffsets_ << msg -> x, msg -> y, msg -> z;
}

void StateManager::maplockArmingHandler( const BoolServ::Request::SharedPtr rq, 
                                         const BoolServ::Response::SharedPtr rp )
{
  /*  Service handler for when the vehicle is armed or disarmed.
     This must lock in/release all the map offsets needed by manager.
  */
  if( !have_arming_origin_ )
  {
    lockArmingGps();  // set this locatin as origin
    lockMapRTK();     // capture current offset from rtk base (zero if no rtk)
    have_arming_origin_ = true;
    RCLCPP_WARN( get_logger(), "StateManager::Origin set, locking RTK map-frame!" );
    rp -> success = true;
  }
  else
  {
    lockArmingGps( false );
    lockMapRTK( false );
    have_arming_origin_ = false;
    RCLCPP_WARN( get_logger(), "StateManager::Origin cleared, unlocking RTK map-frame!" );
    rp -> success = true;
  }
}

// void StateManager::cameraUpdatesCallback( const CameraOdom::ConstPtr &msg )
// {
//   /* Assume that camera directly gives us positions AND velocity. This is
//     ok to do, so as to allow different modes of computation at the camera node;
//     meaning that it should handle its own velocity calculations (particularly
//     useful when using image Jacobians and such). This callback then simply
//     reformats the acquired data into a state vector of our style.
//     Odometry has to be one of the most ridiculously named structures ..
//     nav_msgs::Odometry - pose
//                           - pose
//                             - position {x,y,z}
//                             - orientation {Quaternion}
//                           - covariance (6x6)
//                        - twist
//                           - twist
//                             - linear {x,y,z}
//                             - angular {x,y,z}
//                           - covariance (6x6).
//      NOTE: axis convention aligns with camera plane axes!
//           x   -- lateral/horizontal/left(-) to right(+)
//           y   -- longitudinal/vertical/bottom(-) to top(+)
//           z   -- away from the camera (always positive)
//           yaw -- not handled yet (set to zero)
//           r/p -- not handled
//     */
    
//   double vx, vy, vz;
  
//   state_vector_[0] = msg -> pose.pose.position.y;
//   state_vector_[1] = msg -> pose.pose.position.x;
//   state_vector_[2] = -( msg -> pose.pose.position.z );
  
//   // filter velocities maybe? Prefer a small-length filter ..
//   vx = msg -> twist.twist.linear.y;
//   vy = msg -> twist.twist.linear.x;
//   vz = -( msg -> twist.twist.linear.z );
//   prev_vn_.erase( prev_vn_.begin() );
//   prev_vn_.push_back( vx );
//   prev_ve_.erase( prev_ve_.begin() );
//   prev_ve_.push_back( vy );
//   prev_vd_.erase( prev_vd_.begin() );
//   prev_vd_.push_back( vz );
//   rate_filter_.filterObservations( prev_vn_, prev_ve_, prev_vd_, vx, vy, vz );
//   state_vector_[3] = vx;
//   state_vector_[4] = vy;
//   state_vector_[5] = vz;
  
//   /* unfiltered:
//   state_vector_[3] = msg -> twist.twist.linear.y;
//   state_vector_[4] = msg -> twist.twist.linear.x;
//   state_vector_[5] = -( msg -> twist.twist.linear.z );
//   */
  
//   tf::Quaternion q;
//   tf::quaternionMsgToTF( msg -> pose.pose.orientation, q );
//   double roll, pitch, yaw;
//   tf::Matrix3x3(q).getRPY( roll, pitch, yaw );
//   state_vector_[8] = yaw;

//   // publish away!
//   freyja_msgs::CurrentState state_msg;
//   state_msg.header.stamp = ros::Time::now();
//   for( uint8_t idx = 0; idx < STATE_VECTOR_LEN; idx++ )
//     state_msg.state_vector[idx] = state_vector_[idx];
//   state_pub_.publish( state_msg );
// }
