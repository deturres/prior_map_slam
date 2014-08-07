#!/bin/bash

mkdir ../third_parties/freenect_fetched
cd ../third_parties/freenect_fetched
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build
cmake ../
make -j8
sudo make install

