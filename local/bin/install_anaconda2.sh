#!/bin/bash

if [ ! $# -eq 1 ]; then
  echo "Usage: $0 INSTALL_DIR"
  exit 1
fi

INSTALL_DIR=$1
INSTALL_DIR=$(cd $INSTALL_DIR && pwd)

if [ -e $INSTALL_DIR/.anaconda2 ]; then
  echo "Anaconda2 is already installed: $INSTALL_DIR/.anaconda2"
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

if [ "$(uname)" = "Linux" ]; then
  wget -q 'https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh' -O miniconda2.sh
elif [ "$(uname)" = "Darwin" ]; then
  wget -q 'https://repo.continuum.io/miniconda/Miniconda2-latest-MacOSX-x86_64.sh' -O miniconda2.sh
else
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

bash ./miniconda2.sh -p $INSTALL_DIR/.anaconda2 -b
cd -
rm -rf $TMPDIR

source $INSTALL_DIR/.anaconda2/bin/activate
conda update -n base -y conda
source $INSTALL_DIR/.anaconda2/bin/deactivate
