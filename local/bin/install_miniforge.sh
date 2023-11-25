#!/bin/bash

if [ ! $# -eq 1 ]; then
  echo "Usage: $0 INSTALL_DIR"
  exit 1
fi

INSTALL_DIR=$1
INSTALL_DIR=$(cd $INSTALL_DIR && pwd)

if [ -e $INSTALL_DIR/.miniforge ]; then
  echo "Miniforge is already installed: $INSTALL_DIR/.miniforge"
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"

if which wget &>/dev/null; then
  wget --no-check-certificate -q $URL -O miniforge.sh
else
  curl -s -L $URL -o miniforge.sh
fi

unset PYTHONPATH

bash ./miniforge.sh -p $INSTALL_DIR/.miniforge -b

cd -
rm -rf $TMPDIR
