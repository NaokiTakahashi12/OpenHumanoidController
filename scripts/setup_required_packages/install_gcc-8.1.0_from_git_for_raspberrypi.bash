#!/bin/bash
git clone https://bitbucket.org/sol_prog/raspberry-pi-gcc-binary.git
cd raspberry-pi-gcc-binary && \
tar xf gcc-8.1.0.tar.bz2 && \
sudo mv gcc-8.1.0 /usr/local && \
echo 'export PATH=/usr/local/gcc-8.1.0/bin:$PATH' >> .bashrc
