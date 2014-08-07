#!/bin/bash

#fetch eigen 3.2
ROOTDIR=${PWD}/..
cd ../third_parties/fetched/eigen-eigen-6b38706d90a9
mkdir build
cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=${ROOTDIR}/third_parties/eigen -DEIGEN_BUILD_PKGCONFIG=OFF
make -j8
make install
#sudo make install

