#!/bin/bash

if [ -e $HOME/.local/bin/gotop ]; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

git clone https://github.com/cjbassi/gotop.git
cd gotop
./download.sh
cp gotop $HOME/.local/bin/gotop

rm -rf $TMPDIR
