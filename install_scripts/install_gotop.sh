#!/bin/bash

if [ -e $HOME/.local/bin/gotop ]; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

curl -q -L -O https://raw.githubusercontent.com/cjbassi/gotop/master/scripts/download.sh
bash ./download.sh
mv gotop $HOME/.local/bin/gotop

rm -rf $TMPDIR
