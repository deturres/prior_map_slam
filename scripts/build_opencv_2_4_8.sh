#!/bin/bash

# fetch opencv
ROOTDIR=${PWD}/..
cd ../third_parties/fetched/opencv
mkdir build
cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=${ROOTDIR}/third_parties/opencv
make -j8
make install
