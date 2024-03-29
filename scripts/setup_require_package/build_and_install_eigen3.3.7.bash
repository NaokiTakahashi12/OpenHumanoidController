#!/bin/bash
sudo apt update && \
sudo apt install -y \
	 build-essential \
	 make \
	 cmake \
	 wget \
	 libzip2 \
&& \
wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2 && \
tar xf 3.3.7.tar.bz2 && \
mkdir eigen-eigen-323c052e1731/build && \
cd eigen-eigen-323c052e1731/build && \
cmake .. && \
sudo make install
