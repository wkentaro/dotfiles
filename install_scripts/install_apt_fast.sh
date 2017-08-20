#!/bin/sh

if hash apt-fast &>/dev/null; then
  exit 0
fi

sudo add-apt-repository ppa:saiarcot895/myppa < /dev/null
sudo apt-get update
echo debconf apt-fast/maxdownloads string 16 | sudo debconf-set-selections
echo debconf apt-fast/dlflag boolean true | sudo debconf-set-selections
echo debconf apt-fast/aptmanager string apt-get | sudo debconf-set-selections
sudo apt-get install -y apt-fast
