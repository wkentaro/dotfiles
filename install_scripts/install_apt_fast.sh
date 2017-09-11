#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

if type apt-fast &>/dev/null; then
  exit 0
fi

set -x

sudo add-apt-repository ppa:saiarcot895/myppa

sudo apt-get update -qq
echo debconf apt-fast/maxdownloads string 16 | sudo debconf-set-selections
echo debconf apt-fast/dlflag boolean true | sudo debconf-set-selections
echo debconf apt-fast/aptmanager string apt-get | sudo debconf-set-selections
sudo apt-get install -qq -y apt-fast

set +x
