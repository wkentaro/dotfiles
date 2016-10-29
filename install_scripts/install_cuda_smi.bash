#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0
fi

if which cuda-smi &>/dev/null; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

git clone https://github.com/al42and/cuda-smi.git
cd cuda-smi
git checkout 58c9cbc16d3f4e070cb061cdae4550bf1c6c70ea

make
cp $(pwd)/cuda-smi $HOME/.local/bin/cuda-smi

rm -rf $TMPDIR
