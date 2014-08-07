#!/bin/bash

CURRENT_DIR=${PWD}

mkdir ../third_parties
./fetch_eigen3_2_1.sh
cd ${CURRENT_DIR}
./fetch_opencv_2_4_8.sh
cd ${CURRENT_DIR}
./fetch_g2o.sh
cd ${CURRENT_DIR}
