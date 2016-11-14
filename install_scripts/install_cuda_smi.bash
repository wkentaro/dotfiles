#!/bin/bash

if ! which nvcc &>/dev/null; then
  exit 0
fi

if which cuda-smi &>/dev/null; then
  exit 0
fi

TMPDIR=""

if [ "$(uname)" = "Linux" ]; then
  TMPDIR=$(mktemp -d)
elif [ "$(uname)" = "Darwin" ]; then
  TMPDIR=$(mktemp -d /tmp/tmp.XXXX)
else
  echo "[install_cuda_smi.bash] Unsupported arch: $(uname)"
  exit 1
fi

cd $TMPDIR

git clone https://github.com/al42and/cuda-smi
cd cuda-smi

make
cp $(pwd)/cuda-smi $HOME/.local/bin/cuda-smi

rm -rf $TMPDIR
