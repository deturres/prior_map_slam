#!/bin/bash

function clean_dir {
    cd $1
    cd build
    make clean
    cd ..
    rm -rf build
}  

CURRENT_DIR=${PWD}
clean_dir ../boss
cd ${CURRENT_DIR}
clean_dir ../boss_map
cd ${CURRENT_DIR}
clean_dir ../map_building
cd ${CURRENT_DIR}
clean_dir ../pwn
cd ${CURRENT_DIR}
clean_dir ../pwn_slam
cd ${CURRENT_DIR}
clean_dir ../traversability
cd ${CURRENT_DIR}

