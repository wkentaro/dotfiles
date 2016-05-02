#!/bin/sh

if which apt-fast &>/dev/null; then
  echo "apt-fast is already installed"
  exit 0
fi

sudo add-apt-repository ppa:saiarcot895/myppa
sudo apt-get update
sudo apt-get -y install apt-fast
