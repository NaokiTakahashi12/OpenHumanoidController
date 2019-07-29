#!/bin/bash
wget https://bitbucket.org/rbdl/rbdl/get/default.zip && \
unzip default.zip
cd rbdl-rbdl-0879ee8c548a && \
cmake \
	  -DRBDL_BUILD_TESTS=ON \
	  -DRBDL_BUILD_ADDON_BENCHMARK=ON \
	  -DRBDL_BUILD_ADDON_GEOMETRY=ON \
	  -DRBDL_BUILD_ADDON_URDFREADER=ON \
	  -DRBDL_BUILD_TESTS=ON . \
	  && \
make && \
sudo make install
echo "export RBDL_PATH='/usr/local/include'" >> ~/.bashrc
echo "export RBDL_LIBRARY_PATH='/usr/local/lib'" >> ~/.bashrc
