#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

if type apt-fast &>/dev/null; then
  echo "[$(basename $0)] apt-fast is already installed."
  exit 0
fi

if [ "$PS1" = "" ]; then
  echo "[$(basename $0)] Must be interactive mode."
  exit 0
fi

set -x

sudo add-apt-repository -y ppa:saiarcot895/myppa

sudo apt-get update -qq
echo debconf apt-fast/maxdownloads string 16 | sudo debconf-set-selections
echo debconf apt-fast/dlflag boolean true | sudo debconf-set-selections
echo debconf apt-fast/aptmanager string apt-get | sudo debconf-set-selections
sudo apt-get install -qq -y apt-fast

set +x
