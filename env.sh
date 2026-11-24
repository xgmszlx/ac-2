#!/usr/bin/env bash
# Environment for ExplORB-SLAM parallel line (no-sudo local prefix).
LINE_ROOT=/home/wcqw/explorb_line_20261121
LR=$LINE_ROOT/localroot
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=$LR/opt/ros/noetic/share:$ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=$LR/opt/ros/noetic/lib:$LR/usr/lib/x86_64-linux-gnu:$LR/usr/lib/python3/dist-packages/llvmlite/binding:${LD_LIBRARY_PATH}
export PYTHONPATH=$LR/pysite:$LR/usr/lib/python3/dist-packages:${PYTHONPATH}
export CMAKE_PREFIX_PATH=$LR/opt/ros/noetic:${CMAKE_PREFIX_PATH}
export PKG_CONFIG_PATH=$LR/opt/ros/noetic/lib/pkgconfig:$LR/usr/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}
export PATH=$LR/opt/ros/noetic/bin:$PATH
export GAZEBO_MODEL_PATH=$LINE_ROOT/scenarios/models:${GAZEBO_MODEL_PATH}
# keep ROS/our logs inside the line dir
export ROS_LOG_DIR=$LINE_ROOT/logs/ros
export ROS_HOME=$LINE_ROOT/logs/roshome
# Gazebo media/shaders must stay on GAZEBO_RESOURCE_PATH, otherwise OGRE cannot
# find its material scripts and model loading can abort (author launch files
# overwrite this variable, so keep the default here and only append).
set +u
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi
export GAZEBO_MODEL_DATABASE_URI=""
mkdir -p $ROS_LOG_DIR $ROS_HOME
