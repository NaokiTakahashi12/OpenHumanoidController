#!/bin/bash
sudo apt update && \
sudo apt install -y \
	build-essential \
	 wget \
&& \
wget https://github.com/Kitware/CMake/releases/download/v3.13.2/cmake-3.13.2.tar.gz && \
tar xf cmake-3.13.2.tar.gz && \
cd cmake-3.13.2 && \
./bootstrap && \
make && \
sudo make install
