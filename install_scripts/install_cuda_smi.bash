#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0
fi

if which cuda-smi &>/dev/null; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

git clone https://github.com/wkentaro/cuda-smi.git -b fix-1
cd cuda-smi

make
cp $(pwd)/cuda-smi $HOME/.local/bin/cuda-smi

rm -rf $TMPDIR
