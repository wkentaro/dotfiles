#!/bin/bash

if which nvcc &>/dev/null; then
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

git clone https://github.com/al42and/cuda-smi.git -b mac-build
cd cuda-smi
git checkout 58c9cbc16d3f4e070cb061cdae4550bf1c6c70ea

make
cp $(pwd)/cuda-smi $HOME/.local/bin/cuda-smi

rm -rf $TMPDIR
