#!/bin/bash

if [ $(uname) != Linux ]; then
  exit 0
fi

if which timg &>/dev/null; then
  exit 0
fi

TMPDIR=$(mktemp -d)

sudo apt-get install libwebp-dev libgraphicsmagick++-dev

cd $TMPDIR
git clone https://github.com/hzeller/timg.git
cd timg/src
make
cp timg ~/.local/bin/timg

rm -rf $TMPDIR
