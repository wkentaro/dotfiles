#!/bin/bash

if which cuda-smi &>/dev/null; then
  exit 0
fi

TMPDIR=""

if [ "$(uname)" = "Linux" ]; then
  TMPDIR=$(mktemp -d)
  cd $TMPDIR

  git clone https://github.com/al42and/cuda-smi.git
  cd cuda-smi
  git checkout 58c9cbc16d3f4e070cb061cdae4550bf1c6c70ea
elif [ "$(uname)" = "Darwin" ]; then
  TMPDIR=$(mktemp -d /tmp/tmp.XXXX)
  cd $TMPDIR

  git clone https://github.com/wkentaro/mac-cuda-smi.git
  cd mac-cuda-smi
  git checkout v1.1
else
  echo "[install_cuda_smi.bash] Unsupported arch: $(uname)"
  exit 1
fi

make
cp $(pwd)/cuda-smi $HOME/.local/bin/cuda-smi

rm -rf $TMPDIR
