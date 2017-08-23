#!/bin/sh

if [ -e ~/.anaconda2 ]; then
  echo '[install_anaconda2.sh] Already installed.'
  exit 0
fi

set -x

cd $(mktemp -d)

if [ "$(uname)" = "Linux" ]; then
  wget -q 'https://repo.continuum.io/miniconda/Miniconda2-latest-Linux-x86_64.sh'
  bash ./Miniconda2-latest-Linux-x86_64.sh -p $HOME/.anaconda2 -b
elif [ "$(uname)" = "Darwin" ]; then
  wget -q 'https://repo.continuum.io/miniconda/Miniconda2-latest-MacOSX-x86_64.sh'
  bash ./Miniconda2-latest-MacOSX-x86_64.sh -p $HOME/.anaconda2 -b
fi

set +x
