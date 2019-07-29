#!/bin/bash
git clone https://github.com/boostorg/boost.git --recurse-submodules
cd boost/
./bootstrap.sh --with-toolset=gcc --with-libraries=all && \
./b2 toolset=gcc -j7 && \
sudo ./b2 install
