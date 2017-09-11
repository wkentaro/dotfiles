#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

if type ffmpeg &>/dev/null; then
  exit 0
fi

set -x

sudo add-apt-repository ppa:jonathonf/ffmpeg-3
if [ "$(lsb_release -cs)" = "trusty" ]; then
  sudo add-apt-repository ppa:jonathonf/tesseract
fi
sudo apt-get update -qq
sudo apt-get install -qq -y ffmpeg

set +x
