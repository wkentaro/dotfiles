#!/bin/bash

if ! which nvcc &>/dev/null; then
  echo "[$(basename $0)] nvcc is not found."
  exit 0
fi

if which cuda-smi &>/dev/null; then
  echo "[$(basename $0)] cuda-smi is already installed."
  exit 0
fi

TMPDIR=""

if [ "$(uname)" = "Linux" ]; then
  TMPDIR=$(mktemp -d)
elif [ "$(uname)" = "Darwin" ]; then
  TMPDIR=$(mktemp -d /tmp/tmp.XXXX)
else
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

cd $TMPDIR

git clone https://github.com/al42and/cuda-smi
cd cuda-smi

make
mkdir -p $HOME/.local/bin
cp $(pwd)/cuda-smi $HOME/.local/bin/cuda-smi

rm -rf $TMPDIR
