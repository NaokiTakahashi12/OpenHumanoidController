#!/bin/bash
sudo apt update && \
sudo apt install -y \
	 git \
	 build-essential \
&& \
git clone https://github.com/boostorg/boost.git --recurse-submodules && \
cd boost/ && \
./bootstrap.sh --with-toolset=gcc --with-libraries=all && \
./b2 toolset=gcc -j8 && \
sudo ./b2 install
