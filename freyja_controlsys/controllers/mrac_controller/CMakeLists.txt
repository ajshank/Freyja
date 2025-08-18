cmake_minimum_required(VERSION 3.20)
project(mrac_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(freyja_control REQUIRED)
find_package(mrac_estimator REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mrac_control_node src/mrac_control_node.cpp)
target_include_directories(mrac_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mrac_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(
  mrac_control_node
  freyja_control::freyja_control
  mrac_estimator::mrac_estimator
  ${freyja_control_TARGETS}
  ${tf2_ros_TARGETS}
)

install(TARGETS mrac_control_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
