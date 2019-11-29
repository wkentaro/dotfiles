#!/bin/bash

if [ ! "$(uname)" = "Linux" -a "$(lsb_release -cs)" = "xenial" ]; then
  exit 1
fi

cd /tmp
curl -q https://build.opensuse.org/projects/home:manuelschneid3r/public_key | sudo apt-key add -

sudo sh -c "echo 'deb http://download.opensuse.org/repositories/home:/manuelschneid3r/xUbuntu_16.04/ /' > /etc/apt/sources.list.d/home:manuelschneid3r.list"
wget -nv https://download.opensuse.org/repositories/home:manuelschneid3r/xUbuntu_16.04/Release.key -O Release.key
sudo apt-key add - < Release.key
sudo apt-get update
sudo apt-get install albert
