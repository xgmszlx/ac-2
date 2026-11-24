cmake_minimum_required(VERSION 3.0.2)
project(explorb_scenarios)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/scenario_driver.py
  scripts/run_recorder.py
  scripts/budget_guard.py
  scripts/dynamic_mask.py
  scripts/slam_export.py
  scripts/goto_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch worlds models urdf param DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
