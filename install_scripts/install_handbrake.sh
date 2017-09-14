#!/bin/sh

if [ "$(uname)" != "Linux" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

if [ "$(uname)" = "Linux" -a "$(lsb_release -cs)" != "trusty" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname), $(lsb_release -cs)"
  exit 0
fi

sudo apt-get purge handbrake # remove any old versions
sudo add-apt-repository ppa:stebbins/handbrake-releases
sudo apt-get update
sudo apt-get install handbrake-gtk
