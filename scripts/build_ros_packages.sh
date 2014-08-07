#!/bin/bash

function build_ros_package {
    cd $1
    rosmake
}  

# TODO: Make sure that RE05 driver is built before trying to build robot_eye_driver
# TODO: As we using a custom OpenCV version, we should compile cv_bridge from source as well (as it uses OpenCV internally)

export ROMA_ROOT=${PWD}/..
CURRENT_DIR=${PWD}
export ROS_PACKAGE_PATH=${PWD}/../ros_packages:${ROS_PACKAGE_PATH}
rospack profile
rosstack profile
build_ros_package ../ros_packages/robot_eye_driver
cd ${CURRENT_DIR}
build_ros_package ../ros_packages/boss_ros
cd ${CURRENT_DIR}
