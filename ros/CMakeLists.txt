# ROS wrapper for GenZ-LIO.
#
# One source tree builds under catkin (ROS 1) and ament (ROS 2); which half is
# taken depends on the environment that is sourced. Everything version-specific
# is in src/RosCompat.hpp and the two entry points.
cmake_minimum_required(VERSION 3.16)
project(genz_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(DEFINED ENV{ROS_VERSION})
  set(GENZ_LIO_ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION is not set; source a ROS 1 or ROS 2 environment first")
endif()
message(STATUS "GenZ-LIO: building for ROS ${GENZ_LIO_ROS_VERSION}")

# The core is built from this workspace rather than installed separately.
add_subdirectory(../cpp/genz_lio ${CMAKE_CURRENT_BINARY_DIR}/genz_lio_core)

find_package(PCL REQUIRED COMPONENTS common io)
get_target_property(_genz_eigen_includes genz_lio_core GENZ_LIO_EIGEN_INCLUDE_DIRS)

set(GENZ_LIO_SHARED_SOURCES
  src/OdometryServer.cpp
  src/CloudConversion.cpp
  src/VoxelMapVisualizer.cpp)

if(GENZ_LIO_ROS_VERSION EQUAL 2)
  # ---------------------------------------------------------------- ROS 2 ----
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(pcl_conversions REQUIRED)

  find_package(livox_ros_driver2 QUIET)
  if(livox_ros_driver2_FOUND)
    message(STATUS "livox_ros_driver2 found: Livox CustomMsg support enabled")
  else()
    message(WARNING
      "livox_ros_driver2 not found. GenZ-LIO will build without Livox CustomMsg support; "
      "`lidar_type: livox` will refuse to start. Livox sensors published as PointCloud2 still "
      "work with `lidar_type: livox_pcl`. To enable it, build and source livox_ros_driver2, "
      "then rebuild this workspace.")
  endif()

  add_executable(genz_lio_node src/genz_lio_node_ros2.cpp ${GENZ_LIO_SHARED_SOURCES})
  ament_target_dependencies(genz_lio_node
    rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2_ros pcl_conversions)
  if(livox_ros_driver2_FOUND)
    ament_target_dependencies(genz_lio_node livox_ros_driver2)
    target_compile_definitions(genz_lio_node PRIVATE GENZ_LIO_WITH_LIVOX)
  endif()

  install(TARGETS genz_lio_node DESTINATION lib/${PROJECT_NAME})

  # Fixed-map pure localization (run_loclite_online), interface and cold start
  # as in Super-LIO. KISS-Matcher (prior-free global registration) is built from
  # the vendored source; its ROBIN dependency is fetched at configure time. Never
  # pick up a robin/kiss_matcher left in a shared install prefix by another
  # package (lightning-lm / hikari_loclite / Super-LIO vendor the same libraries).
  find_package(std_srvs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(TBB REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io filters kdtree search registration)
  if(NOT TARGET kiss_matcher::kiss_matcher_core)
    set(USE_SYSTEM_TBB ON CACHE BOOL "Use system TBB for KISS-Matcher" FORCE)
    set(USE_SYSTEM_ROBIN OFF CACHE BOOL "Fetch ROBIN for KISS-Matcher" FORCE)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    add_subdirectory(
      ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/KISS-Matcher/cpp/kiss_matcher
      ${CMAKE_BINARY_DIR}/_deps/kiss_matcher-build
      EXCLUDE_FROM_ALL)
  endif()

  # loclite: ROS surface + relocalization shared with the se3-lio fork. The
  # vendored GenZ-ICP uses its own Sophus and the core's tsl::robin_map.
  add_library(loclite STATIC
    src/loclite/fc_gate.cpp
    src/loclite/genz_refiner.cpp
    src/loclite/kiss_global_matcher.cpp
    src/loclite/loc_interface.cpp
    src/loclite/relocalizer.cpp
    thirdparty/genz_icp/genz_icp/VoxelHashMap.cpp
    thirdparty/genz_icp/genz_icp/Registration.cpp)
  target_include_directories(loclite
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${PCL_INCLUDE_DIRS}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/genz_icp
            ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/genz_lio/3rdparty)
  target_include_directories(loclite SYSTEM BEFORE PUBLIC ${_genz_eigen_includes})
  target_link_libraries(loclite
    PUBLIC ${PCL_LIBRARIES}
    PRIVATE kiss_matcher::kiss_matcher_core TBB::tbb)
  # Keyword signature only (works with and without ament_target_dependencies).
  foreach(dep rclcpp std_srvs std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs
          pcl_conversions tf2_ros)
    if(${dep}_TARGETS)
      target_link_libraries(loclite PUBLIC ${${dep}_TARGETS})
    else()
      target_link_libraries(loclite PUBLIC ${${dep}_LIBRARIES})
      target_include_directories(loclite PUBLIC ${${dep}_INCLUDE_DIRS})
    endif()
  endforeach()

  add_executable(run_loclite_online
    src/run_loclite_online.cpp src/LocalizationNode.cpp src/CloudConversion.cpp)
  target_compile_definitions(run_loclite_online PRIVATE GENZ_LIO_ROS_VERSION=2)
  target_include_directories(run_loclite_online PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_include_directories(run_loclite_online SYSTEM BEFORE PRIVATE ${_genz_eigen_includes})
  ament_target_dependencies(run_loclite_online
    rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2_ros pcl_conversions)
  if(livox_ros_driver2_FOUND)
    ament_target_dependencies(run_loclite_online livox_ros_driver2)
    target_compile_definitions(run_loclite_online PRIVATE GENZ_LIO_WITH_LIVOX)
  endif()
  target_link_libraries(run_loclite_online loclite genz_lio::core genz_lio::config yaml-cpp)
  install(TARGETS run_loclite_online DESTINATION lib/${PROJECT_NAME})

  # KISS-Matcher and its fetched ROBIN/PMC/xenium carry their own install rules,
  # which would put static libs, headers and CMake exports into the prefix and
  # collide in dpkg with other packages shipping the same files. Everything is
  # linked statically, so drop them after the subtree installed.
  # DESTDIR-aware, multiarch-libdir-aware, exact paths only.
  include(GNUInstallDirs)
  string(CONFIGURE [[
    set(_genz_lio_strip_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
    file(REMOVE_RECURSE
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_INCLUDEDIR@/kiss_matcher"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_INCLUDEDIR@/robin"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_INCLUDEDIR@/pmc"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_INCLUDEDIR@/xenium"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/cmake/kiss_matcher"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/cmake/robin"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/cmake/pmc"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/cmake/xenium"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/libkiss_matcher_core.a"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/librobin.a"
      "${_genz_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/libpmc.a")
  ]] _genz_lio_strip_code @ONLY)
  install(CODE "${_genz_lio_strip_code}")

  install(PROGRAMS scripts/save_odometry.py DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/genz_lio_fastdds.sh.in")
  ament_package()
else()
  # ---------------------------------------------------------------- ROS 1 ----
  find_package(catkin REQUIRED COMPONENTS
    roscpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf tf2_ros pcl_ros pcl_conversions)

  find_package(livox_ros_driver QUIET)
  if(livox_ros_driver_FOUND)
    message(STATUS "livox_ros_driver found: Livox CustomMsg support enabled")
  else()
    message(WARNING
      "livox_ros_driver not found. GenZ-LIO will build without Livox CustomMsg support; "
      "`lidar_type: livox` will refuse to start. Livox sensors published as PointCloud2 still "
      "work with `lidar_type: livox_pcl`. To enable it, build and source livox_ros_driver, "
      "then rebuild this workspace.")
  endif()

  catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf tf2_ros
    DEPENDS PCL)

  add_executable(genz_lio_node src/genz_lio_node.cpp ${GENZ_LIO_SHARED_SOURCES})
  target_include_directories(genz_lio_node PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(genz_lio_node ${catkin_LIBRARIES})
  if(livox_ros_driver_FOUND)
    target_include_directories(genz_lio_node PRIVATE ${livox_ros_driver_INCLUDE_DIRS})
    target_link_libraries(genz_lio_node ${livox_ros_driver_LIBRARIES})
    target_compile_definitions(genz_lio_node PRIVATE GENZ_LIO_WITH_LIVOX)
  endif()

  install(TARGETS genz_lio_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  catkin_install_python(PROGRAMS scripts/save_odometry.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

target_compile_definitions(genz_lio_node PRIVATE GENZ_LIO_ROS_VERSION=${GENZ_LIO_ROS_VERSION})
target_include_directories(genz_lio_node PRIVATE ${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(genz_lio_node genz_lio::core genz_lio::config ${PCL_LIBRARIES} yaml-cpp)

target_include_directories(genz_lio_node SYSTEM BEFORE PRIVATE ${_genz_eigen_includes})

if(GENZ_LIO_BUILD_TESTS)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME save_odometry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_save_odometry.py)
  add_executable(test_terminal_status tests/test_terminal_status.cpp)
  target_include_directories(test_terminal_status PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_terminal_status genz_lio::core)
  add_test(NAME terminal_status COMMAND test_terminal_status)
  add_executable(test_input_timing tests/test_input_timing.cpp src/CloudConversion.cpp)
  target_compile_definitions(test_input_timing PRIVATE GENZ_LIO_ROS_VERSION=${GENZ_LIO_ROS_VERSION})
  target_include_directories(test_input_timing PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_input_timing genz_lio::core)
  target_include_directories(test_input_timing SYSTEM BEFORE PRIVATE ${_genz_eigen_includes})
  if(GENZ_LIO_ROS_VERSION EQUAL 2)
    ament_target_dependencies(test_input_timing rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2_ros)
  else()
    target_include_directories(test_input_timing PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_input_timing ${catkin_LIBRARIES})
  endif()
  add_test(NAME input_timing COMMAND test_input_timing)
  if(GENZ_LIO_ROS_VERSION EQUAL 2)
    add_test(NAME fastdds_environment
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_fastdds_environment.py)
    add_executable(test_ros2_topics tests/test_ros2_topics.cpp ${GENZ_LIO_SHARED_SOURCES})
    target_compile_definitions(test_ros2_topics PRIVATE GENZ_LIO_ROS_VERSION=2)
    target_include_directories(test_ros2_topics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_ros2_topics
      rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2_ros pcl_conversions)
    target_link_libraries(test_ros2_topics genz_lio::core genz_lio::config ${PCL_LIBRARIES} yaml-cpp)
    target_include_directories(test_ros2_topics SYSTEM BEFORE PRIVATE ${_genz_eigen_includes})
    add_test(NAME ros2_topics COMMAND test_ros2_topics ${CMAKE_CURRENT_SOURCE_DIR}/config)
    set_tests_properties(ros2_topics PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=198")
  endif()
endif()
