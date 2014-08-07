#!/bin/bash

#fetch eigen 3.2
ROOTDIR=${PWD}/..
mkdir ../third_parties/fetched
cd ../third_parties/fetched
wget http://bitbucket.org/eigen/eigen/get/3.2.1.tar.bz2
tar -xjvf 3.2.1.tar.bz2

