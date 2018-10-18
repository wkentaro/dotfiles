#!/bin/bash


if [ "$(uname)" != 'Linux' ]; then
  exit 0
fi

if which nvtop &>/dev/null; then
  exit 0
fi


sudo apt install -y cmake libncurses5-dev git

TMPDIR=$(mktemp -d)
cd $TMPDIR

git clone https://github.com/Syllo/nvtop.git
mkdir -p nvtop/build && cd nvtop/build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/.local

# If it errors with "Could NOT find NVML (missing: NVML_INCLUDE_DIRS)"
# try the following command instead, otherwise skip to the build with make.
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/.local -DNVML_RETRIEVE_HEADER_ONLINE=True

make
make install # You may need sufficient permission for that (root)

cd -
rm -rf $TMPDIR
