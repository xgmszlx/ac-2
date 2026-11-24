cmake_minimum_required(VERSION 3.0.2)
project(visual_query)

# Pure C++ (Eigen only) so that the query can also be compiled and unit-checked
# without catkin at all:
#
#   g++ -O2 -std=c++14 -I include -I/usr/include/eigen3 \
#       src/local_visual_query.cpp src/selfcheck_visual_query.cpp -o /tmp/vq_selfcheck
#   /tmp/vq_selfcheck
#
# That is the path used in this line's logs when the shared heavy lock is taken by
# the other research line and no catkin_make may run.

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visual_query
  DEPENDS Eigen3
)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(visual_query
  src/local_visual_query.cpp
)
target_compile_options(visual_query PRIVATE -O3)
add_dependencies(visual_query ${catkin_EXPORTED_TARGETS})

# Offline self-check (no ROS node, no master): run it as
#   ./devel/lib/visual_query/selfcheck_visual_query
add_executable(selfcheck_visual_query src/selfcheck_visual_query.cpp)
target_link_libraries(selfcheck_visual_query visual_query)

install(TARGETS visual_query selfcheck_visual_query
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
