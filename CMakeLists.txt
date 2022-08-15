cmake_minimum_required(VERSION 2.8.3)
project(flir_pol_cam)

add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")


set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(SPINNAKER_INCLUDE_DIR "/opt/spinnaker/include" CACHE PATH "Spinnaker Include Directory")
set(SPINNAKER_LIB_DIR "/opt/spinnaker/lib" CACHE PATH "Spinnaker Libs Directory")

#set(CUDA_USE_STATIC_CUDA_RUNTIME  OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

# Find Packages
find_package(OpenCV 4 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS OpenCV
)
include_directories(
      ${PROJECT_INCLUDE_DIR}
      ${catkin_INCLUDE_DIRS}
      SYSTEM ${SPINNAKER_INCLUDE_DIR}
      SYSTEM ${OpenCV_INCLUDE_DIRS}
    )
link_directories( ${SPINNAKER_LIB_DIR} )

set(LIBS Spinnaker ${OpenCV_LIBS})

# DEMO
add_executable(DEMO src/Polarization.cpp)
target_link_libraries (DEMO ${LIBS} ${catkin_LIBRARIES})

# OpenCV view
if(OpenCV_FOUND)
  add_executable(
    PCam
    src/main.cpp
    src/camera.cpp
  )
  target_link_libraries (PCam ${LIBS} ${catkin_LIBRARIES})
endif()

# ROS publisher
add_executable(
  ROSPCam
  src/main_ros.cpp
  src/camera.cpp
)
target_link_libraries (ROSPCam ${LIBS} ${catkin_LIBRARIES})
