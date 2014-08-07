#!/bin/bash

# fetch g2o
cd ../
ROMA_ROOT=$PWD
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout g2o_hierarchical
