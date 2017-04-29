#!/bin/sh

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

if [ "$(uname)" = "Linux" -a "$(lsb_release -cs)" != "trusty" ]; then
  exit 1
fi

if which ffmpeg &>/dev/null; then
  exit 1
fi

sudo apt-get install -qq -y software-properties-common
sudo add-apt-repository ppa:mc3man/trusty-media -y
sudo apt-get update -qq
sudo apt-get install -qq -y ffmpeg
