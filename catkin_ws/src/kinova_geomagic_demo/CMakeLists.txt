cmake_minimum_required(VERSION 3.0.2)
project(kinova_geomagic_demo)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(
  PROGRAMS
  scripts/geomagic_kinova_teleop.py
  scripts/geomagic_kinova_cartesian_teleop.py
  scripts/kdl_haptic_feedback.py
  scripts/fake_geomagic_joints.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
