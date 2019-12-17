#!/bin/sh
SCRIPTS_DIR=$(dirname $0)
sh $SCRIPTS_DIR/install_package_from_apt.sh
sh $SCRIPTS_DIR/build_and_install_rbdl_latest.sh
