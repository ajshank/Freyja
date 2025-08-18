cmake_minimum_required(VERSION 3.20)
project(mrac_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(freyja_control REQUIRED)

add_library(mrac_estimator STATIC src/mrac_estimator.cpp)
add_library(mrac_estimator::mrac_estimator ALIAS mrac_estimator)
target_compile_features(mrac_estimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(mrac_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  mrac_estimator PUBLIC
  ${freyja_control_TARGETS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mrac_estimator PRIVATE "MRAC_ESTIMATOR_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS mrac_estimator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  mrac_estimator
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
