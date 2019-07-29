#!/bin/bash
sudo apt update
sudo apt install -y \
		git build-essential libboost-thread-dev libboost-system-dev \
		libeigen3-dev libunittest++-dev libpython2.7-dev \
		libjpeg-dev libncurses5-dev \
		wget unzip cmake-curses-gui \
		jq
