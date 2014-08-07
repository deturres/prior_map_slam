#!/bin/bash

# fetch opencv
ROOTDIR=${PWD}/..
mkdir ../third_parties/fetched
cd ../third_parties/fetched
git clone git://code.opencv.org/opencv.git
cd opencv
git checkout 2.4.8
