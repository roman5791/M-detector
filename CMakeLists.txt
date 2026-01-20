cmake_minimum_required(VERSION 3.1.0)
project(m_detector)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -ltbb" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ltbb -pthread -std=c++1z -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  # if(N GREATER 15)
  #   add_definitions(-DMP_EN)
  #   add_definitions(-DMP_PROC_NUM=14)
  #   message("core for MP:  4")
  # elseif(N GREATER 5)
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# Detect ROS1 vs ROS2
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(catkin_FOUND)
  message(STATUS "Building for ROS1 with catkin")
  set(TBB_DIR "/usr/local/tbb-2019_U8")
  if(EXISTS "/home/huajie/package/oneTBB-2019_U8/cmake/TBBBuild.cmake")
    include("/home/huajie/package/oneTBB-2019_U8/cmake/TBBBuild.cmake")
    tbb_build(TBB_ROOT "/home/huajie/package/oneTBB-2019_U8" CONFIG_DIR TBB_DIR)
  endif()
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    roscpp
    rospy
    std_msgs
    pcl_ros
    tf
    livox_ros_driver
    message_generation
    eigen_conversions
  )
elseif(ament_cmake_FOUND)
  message(STATUS "Building for ROS2 with ament_cmake")
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(pcl_ros REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
else()
  message(FATAL_ERROR "Neither catkin nor ament_cmake found. Please source ROS1 or ROS2.")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

if(catkin_FOUND)
  include_directories(
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    include)
  
  generate_messages(
    DEPENDENCIES
    geometry_msgs
  )
  
  catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS
  )
else()
  # ROS2 - use ament_target_dependencies instead
  include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    include)
endif()


if(catkin_FOUND)
  # ROS1 executables
  add_executable(dynfilter src/dynfilter_with_odom.cpp src/DynObjFilter.cpp src/DynObjCluster.cpp)
  target_link_libraries(dynfilter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
  target_include_directories(dynfilter PRIVATE ${PYTHON_INCLUDE_DIRS})
  
  add_executable(display_prediction src/display_prediction.cpp )
  target_link_libraries(display_prediction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
  target_include_directories(display_prediction PRIVATE ${PYTHON_INCLUDE_DIRS})
  
  add_executable(cal_recall src/cal_recall_multi.cpp)
  target_link_libraries(cal_recall ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
  target_include_directories(cal_recall PRIVATE ${PYTHON_INCLUDE_DIRS})
elseif(ament_cmake_FOUND)
  # ROS2 executables
  
  # dynfilter_odom node (ROS2)
  add_executable(dynfilter_odom src/dynfilter_with_odom_ros2.cpp src/DynObjFilter.cpp src/DynObjCluster.cpp)
  target_link_libraries(dynfilter_odom ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
  target_include_directories(dynfilter_odom PRIVATE ${PYTHON_INCLUDE_DIRS})
  ament_target_dependencies(dynfilter_odom
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    pcl_ros
    pcl_conversions
    tf2
    tf2_eigen
    tf2_geometry_msgs
    visualization_msgs
  )
  
  # Install executables
  install(TARGETS
    dynfilter_odom
    DESTINATION lib/${PROJECT_NAME}
  )
  
  # ament package
  ament_package()
endif()


