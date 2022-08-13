cmake_minimum_required(VERSION 3.0.2)
project(realsense)
enable_language(CUDA)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Release")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -fopenmp")

SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fopenmp")
if (WIN32)
else()
set(CMAKE_CXX_FLAGS " -lglfw  -fPIE -fPIC -std=c++14 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
endif()
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(cv_bridge_DIR /home/agx/Documents/zgq/ros/catkin_cvbg/devel/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  nodelet
  pcl_conversions
  pcl_ros
  # rockauto_msgs
  smartcar_msgs
  roscpp
  sensor_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense
#  CATKIN_DEPENDS cv_bridge message_generation nodelet pcl_conversions pcl_ros rockauto_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
)
set(ALL_TARGET_LIBRARIES "")
# include(../cmake/realsense.cmake)
include(../cmake/realsense.cmake)
include(../cmake/trt_yolo.cmake)
include(../cmake/OPENCV.cmake)
include(../cmake/OpenGL.cmake)
include(../cmake/glfw3.cmake)
include(../cmake/YAML.cmake)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/realsense.cpp
# )
add_library(real
    src/realsense/realsense_nodelet.cpp
    src/realsense/realsense.hpp
)
add_dependencies(real ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(real
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
  ${DEPENDENCIES}
)
file(GLOB_RECURSE sources src/trt_yolo/modules/*.hpp src/trt_yolo/modules/*.cpp 
                        src/trt_yolo/modules/*.h src/trt_yolo/modules/*.cu src/trt_yolo/extra/*.h)
add_library(detector SHARED ${sources})
target_include_directories(detector PRIVATE src/trt_yolo/extra/ src/trt_yolo/modules/ ${OpenCV_INCLUDE_DIRS} ${CUDA_TOOLKIT_ROOT_DIR}/include)
target_link_libraries(detector nvinfer nvinfer_plugin nvparsers "stdc++fs")

# LINK_DIRECTORIES(/home/agx/Documents/zgq/ros/catkin_trt/devel/lib/)
add_executable(${PROJECT_NAME}_node src/realsense_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_node PRIVATE src/trt_yolo/modules/ src/trt_yolo/extra/)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  detector
  ${ALL_TARGET_LIBRARIES}
  ${DEPENDENCIES}
  ${realsense2_LIBRARY}
)
add_executable(trafficlight_node src/trafficlight_node.cpp)
add_dependencies(trafficlight_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_include_directories(trafficlight_node PRIVATE rc/trt_yolo/modules/ rc/trt_yolo/extra/)
target_link_libraries(trafficlight_node
  ${catkin_LIBRARIES}
  
  ${ALL_TARGET_LIBRARIES}
  ${DEPENDENCIES}
  # ${realsense2_LIBRARY}
)