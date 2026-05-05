cmake_minimum_required(VERSION 3.0.2)
project(geomagic_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs sensor_msgs genmsg rosconsole tf urdf)

add_message_files(FILES PhantomButtonEvent.msg OmniFeedback.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs genmsg rosconsole tf urdf)

include_directories(${catkin_INCLUDE_DIRS})

option(BUILD_GEOMAGIC_TOUCH_DRIVER "Build omni_cartesian against OpenHaptics (Geomagic Touch)" ON)

set(OPENHAPTICS_ROOT "$ENV{OPENHAPTICS_ROOT}")

find_path(
  OPENHAPTICS_HD_INCLUDE_DIR
  NAMES HD/hd.h
  PATHS "${OPENHAPTICS_ROOT}/include" /usr/include /opt/openhaptics/Developer/include)

find_library(
  OPENHAPTICS_HD_LIBRARY
  NAMES HD
  PATHS "${OPENHAPTICS_ROOT}/lib" "${OPENHAPTICS_ROOT}/lib64" /usr/lib /opt/openhaptics/Developer/lib)

find_library(
  OPENHAPTICS_HDU_LIBRARY
  NAMES HDU
  PATHS "${OPENHAPTICS_ROOT}/lib" "${OPENHAPTICS_ROOT}/lib64" /usr/lib /opt/openhaptics/Developer/lib)

if(BUILD_GEOMAGIC_TOUCH_DRIVER
   AND OPENHAPTICS_HD_INCLUDE_DIR
   AND OPENHAPTICS_HD_LIBRARY
   AND OPENHAPTICS_HDU_LIBRARY)
  include_directories(${OPENHAPTICS_HD_INCLUDE_DIR})
  add_executable(omni_cartesian src/omni_cartesian.cpp)
  target_link_libraries(
    omni_cartesian ${OPENHAPTICS_HD_LIBRARY} ${OPENHAPTICS_HDU_LIBRARY} rt ncurses
                    pthread ${catkin_LIBRARIES})
  install(TARGETS omni_cartesian RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  message(STATUS "Building Geomagic Touch driver (OpenHaptics found).")
else()
  message(
    STATUS
      "Skipping omni_cartesian: OpenHaptics not found. Set OPENHAPTICS_ROOT or BUILD_GEOMAGIC_TOUCH_DRIVER=OFF."
  )
endif()

install(DIRECTORY launch cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
