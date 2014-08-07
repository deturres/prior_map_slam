#!/bin/bash

# fetch g2o
cd ../
ROMA_ROOT=$PWD
cd g2o
mkdir build
cd build
cmake ../ -DG2O_EIGEN3_INCLUDE=${ROMA_ROOT}/third_parties/eigen/include/eigen3
make -j8
