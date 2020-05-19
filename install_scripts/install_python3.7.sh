#! /bin/sh
#
# install_python3.7.sh
# Copyright (C) 2020 wkentaro <wkentaro@jararacussu>
#
# Distributed under terms of the MIT license.
#


if [ ! "$(uname)" = "Linux" -a "$(lsb_release -cs)" = "xenial" ]; then
  exit 1
fi

sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.7
