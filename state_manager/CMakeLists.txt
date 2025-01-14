cmake_minimum_required(VERSION 3.8)
project(state_manager)

set( ignoreMe "${NO_PIXHAWK}${NO_ROBOMASTER}" )

# outdated flag, has no effect. Will be removed in the future.
set( USINGVICON 1 )
set( LQE_ORDER 2 )
add_compile_options( -O3 -ffast-math
      -DUSE_VICON_=${USINGVICON}
      -DLQE_INTEGRATOR_ORDER_=${LQE_ORDER} 
   )
set( CMAKE_CXX_STANDARD 17 )

find_package(ament_cmake REQUIRED)

find_package( rclcpp REQUIRED )
find_package( std_msgs REQUIRED )
find_package( geometry_msgs REQUIRED )
find_package( freyja_msgs REQUIRED )
find_package( freyja_utils REQUIRED )
find_package( sensor_msgs REQUIRED )
find_package( nav_msgs REQUIRED ) 
find_package( std_srvs REQUIRED ) 
find_package( tf2 REQUIRED )
find_package( tf2_ros REQUIRED )
find_package( tf2_geometry_msgs REQUIRED )
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions( ${EIGEN3_DEFINITIONS} )

include_directories(include)
include_directories(
  ${Eigen_INCLUDE_DIRS}
)

# we need some means to support older ROS2 distros for a while
if(DEFINED ENV{ROS_DISTRO})
    if("$ENV{ROS_DISTRO}" MATCHES "foxy" OR "$ENV{ROS_DISTRO}" MATCHES "galactic")
        add_compile_options( -DROSVER_FOXY_OR_GALAC )
    endif()
endif()

# add_executable(state_manager_node src/state_manager.cpp src/state_estimator.cpp)
add_executable(state_manager_node src/state_manager.cpp)
target_include_directories( state_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies( state_manager_node 
  rclcpp
  std_msgs
  freyja_msgs
  geometry_msgs
  std_srvs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  freyja_utils
)

install(TARGETS
  state_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

### Can use state estimator INSTEAD of manager with the right modifications
### This contains an LQE (Kalman) for full state.

#add_executable(bias_estimator_node src/state_estimator.cpp)
#add_dependencies(bias_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
#target_link_libraries(bias_estimator_node
#   ${catkin_LIBRARIES}
# )

ament_package()
