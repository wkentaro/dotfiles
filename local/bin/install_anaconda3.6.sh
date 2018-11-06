#!/bin/bash

if [ ! $# -eq 1 ]; then
  echo "Usage: $0 INSTALL_DIR"
  exit 1
fi

INSTALL_DIR=$1
INSTALL_DIR=$(cd $INSTALL_DIR && pwd)

install_anaconda3.sh $INSTALL_DIR

source $INSTALL_DIR/.anaconda3/bin/activate
conda install -y python=3.6
source $INSTALL_DIR/.anaconda3/bin/deactivate
