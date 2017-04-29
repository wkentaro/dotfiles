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

sudo add-apt-repository ppa:mc3man/trusty-media
sudo apt-get update
sudo apt-get install ffmpeg