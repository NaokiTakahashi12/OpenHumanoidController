#!/bin/sh
EIGEN3_SOURCE_DIR=eigen3
EIGEN3_SOURCE_BUILD_DIR=$EIGEN3_SOURCE_DIR/build
EIGEN3_SOURCE_TAR=$EIGEN3_SOURCE_DIR.tar.bz2

sudo apt update && \
sudo apt install -y \
	 build-essential \
	 make \
	 cmake \
	 wget \
	 tar \
&& \
wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2 -O $EIGEN3_SOURCE_TAR && \
mkdir $EIGEN3_SOURCE_DIR && \
tar xf $EIGEN3_SOURCE_TAR -C $EIGEN3_SOURCE_DIR --strip-components=1 && \
mkdir $EIGEN3_SOURCE_BUILD_DIR && \
cd $EIGEN3_SOURCE_BUILD_DIR && \
cmake .. && \
sudo make install && \
cd ../../ && \
echo -n "Remeve eigen3 source [y/n]: " && \
read ANSWER && \
case $ANSWER in
	y)
		rm -rv $EIGEN3_SOURCE_TAR $EIGEN3_SOURCE_DIR
		;;
	*)
		echo "Not remove eigen3 source"
		;;
esac

