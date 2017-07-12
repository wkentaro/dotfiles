#!/bin/sh

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

if [ "$(uname)" = "Linux" -a "$(lsb_release -cs)" != "trusty" ]; then
  exit 1
fi

sudo apt-get purge handbrake # remove any old versions
sudo add-apt-repository ppa:stebbins/handbrake-releases
sudo apt-get update
sudo apt-get install handbrake-gtk
