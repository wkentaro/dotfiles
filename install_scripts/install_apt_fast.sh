#!/bin/sh

if hash apt-fast &>/dev/null; then
  echo "apt-fast is already installed"
  exit 0
fi

sudo apt-get install -qq -y software-properties-common
sudo add-apt-repository ppa:saiarcot895/myppa -y
sudo apt-get update -qq
sudo apt-get install -qq -y apt-fast
