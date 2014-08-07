#!/bin/bash

cd ../third_parties/fetched/libfreenect
mkdir build
cd build
cmake ../
make -j8
