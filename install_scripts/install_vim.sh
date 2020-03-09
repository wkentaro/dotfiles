#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0
fi

HERE=$(realpath $(dirname ${BASH_SOURCE[0]}))

if [ "$(lsb_release -sr)" = "16.04" ]; then
  bash $HERE/install_vim.xenial.opt.sh
elif [ "$(lsb_release -sr)" = "18.04" ]; then
  bash $HERE/install_vim.bionic.opt.sh
fi
