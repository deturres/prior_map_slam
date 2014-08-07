#!/bin/bash

function build_dir {
   ROMA_ROOT=${PWD}/..
   echo "roma root is ${ROMA_ROOT}"
    cd $1
    mkdir build
    cd build
    #cmake ../ -DEIGEN3_INCLUDE_DIR=${ROMA_ROOT}/third_parties/eigen/include/eigen3 -DOpenCV_DIR=${ROMA_ROOT}/third_parties/opencv/share/OpenCV
    make -j8
}

CURRENT_DIR=${PWD}
build_dir ../boss
cd ${CURRENT_DIR}
build_dir ../boss_map
cd ${CURRENT_DIR}
build_dir ../map_building
cd ${CURRENT_DIR}
build_dir ../pwn
cd ${CURRENT_DIR}
build_dir ../pwn_slam
cd ${CURRENT_DIR}
build_dir ../traversability
cd ${CURRENT_DIR}
