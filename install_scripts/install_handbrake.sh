#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

if [ "$(uname)" = "Linux" -a "$(lsb_release -cs)" != "trusty" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname), $(lsb_release -cs)"
  exit 0
fi

set -x

sudo apt-get purge handbrake # remove any old versions
sudo add-apt-repository -y ppa:stebbins/handbrake-releases
sudo apt-get update -qq
sudo apt-get install -y handbrake-gtk

set +x
