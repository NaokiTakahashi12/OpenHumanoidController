#!/bin/sh
sudo apt update && \
sudo apt install -y \
	 git \
	 build-essential \
&& \
git clone https://github.com/boostorg/boost.git --recurse-submodules && \
cd boost/ && \
./bootstrap.sh --with-libraries=all && \
./b2 && \
sudo ./b2 install
