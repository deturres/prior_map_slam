#!/bin/bash

CURRENT_DIR=${PWD}
./build_eigen3_2_1.sh
cd ${CURRENT_DIR}
./build_opencv_2_4_8.sh
cd ${CURRENT_DIR}
./build_g2o.sh
cd ${CURRENT_DIR}
