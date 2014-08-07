#!/bin/bash

mkdir ../third_parties/openni2_fetched
cd ../third_parties/openni2_fetched
git clone https://github.com/OpenNI/OpenNI2.git
cd OpenNI2
make
cd Packaging
./ReleaseVersion x64

# cd Final/
# sudo cp OpenNI-Linux-x64-2.2.tar.bz2 /opt/
# cd /opt/
# sudo tar -jxf OpenNI-Linux-x64-2.2.tar.bz2
# cd /opt/OpenNI-Linux-x64-2.2/
# sudo ./install.sh
# cd ~/
# gedit .bashrc
# source /opt/OpenNI-Linux-x64-2.2/OpenNIDevEnvironment 
