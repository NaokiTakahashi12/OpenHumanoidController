#!/bin/bash
sudo apt update && \
sudo apt install software-properties-common -y && \
sudo add-apt-repository ppa:ubuntu-toolchain-r/test && \
sudo apt update && \
sudo apt install -y \
	 gcc-9 \
	 g++-9 \
	 && \
echo "export CC=gcc-9" >> ~/.bashrc && \
echo "export CXX=g++-9" >> ~/.bashrc
