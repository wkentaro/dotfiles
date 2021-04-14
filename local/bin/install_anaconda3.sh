#!/bin/bash

if [ ! $# -eq 1 ]; then
  echo "Usage: $0 INSTALL_DIR"
  exit 1
fi

INSTALL_DIR=$1
INSTALL_DIR=$(cd $INSTALL_DIR && pwd)

if [ -e $INSTALL_DIR/.anaconda3 ]; then
  echo "Anaconda3 is already installed: $INSTALL_DIR/.anaconda3"
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

if [ "$(uname)" = "Linux" ]; then
  URL='https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh'
elif [ "$(uname)" = "Darwin" ]; then
  URL='https://repo.continuum.io/miniconda/Miniconda3-latest-MacOSX-x86_64.sh'
else
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

if which wget &>/dev/null; then
  wget --no-check-certificate -q $URL -O miniconda3.sh
else
  curl -s -L $URL -o miniconda3.sh
fi

bash ./miniconda3.sh -p $INSTALL_DIR/.anaconda3 -b
cd -
rm -rf $TMPDIR

source $INSTALL_DIR/.anaconda3/bin/activate
conda update -n base -y conda
source $INSTALL_DIR/.anaconda3/bin/deactivate
