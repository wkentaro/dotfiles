#!/bin/bash

if [ $(uname) != Linux ]; then
  exit 0
fi

if which timg &>/dev/null; then
  exit 0
fi

TMPDIR=$(mktemp -d)

sudo apt-get install -qq -y libwebp-dev libgraphicsmagick++-dev

cd $TMPDIR
git clone https://github.com/hzeller/timg.git
cd timg/src
make -j
cp timg $HOME/.local/bin/timg

rm -rf $TMPDIR
