#!/bin/sh
RBDL_SOURCE_ZIP=rbdl.zip
RBDL_SOURCE_DIR=rbdl
RBDL_INSTALL_PATH=~/.local

mkdir $RBDL_INSTALL_PATH

wget https://bitbucket.org/rbdl/rbdl/get/default.zip -O $RBDL_SOURCE_ZIP && \
unzip $RBDL_SOURCE_ZIP -d $RBDL_SOURCE_DIR
mkdir $RBDL_SOURCE_DIR/rbdl-rbdl-0879ee8c548a/build && \
cd $RBDL_SOURCE_DIR/rbdl-rbdl-0879ee8c548a/build && \
cmake \
	  -DCMAKE_INSTALL_PREFIX=$RBDL_INSTALL_PATH \
	  -DRBDL_BUILD_TESTS=OFF \
	  -DRBDL_BUILD_ADDON_BENCHMARK=ON \
	  -DRBDL_BUILD_ADDON_GEOMETRY=ON \
	  -DRBDL_BUILD_ADDON_MUSCLE=ON \
	  -DRBDL_BUILD_ADDON_URDFREADER=ON \
	  .. \
	  && \
sudo make install && \
echo "export RBDL_PATH='$RBDL_INSTALL_PATH'" >> ~/.bashrc && \
echo "export RBDL_INCLUDE_PATH='$RBDL_INSTALL_PATH/include'" >> ~/.bashrc && \
echo "export RBDL_LIBRARY_PATH='$RBDL_INSTALL_PATH/lib'" >> ~/.bashrc && \
cd ../../../ && \
echo -n "Remove rbdl source [y/n]: " && \
read ANSWER && \
case $ANSWER in
	y)
		rm -rv $RBDL_SOURCE_ZIP $RBDL_SOURCE_DIR
		;;
	*)
		echo "Not remove rbdl source"
		;;
esac
