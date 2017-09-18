#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

if type ffmpeg &>/dev/null; then
  echo "[$(basename $0)] ffmpeg is already installed."
  exit 0
fi

set -x

sudo add-apt-repository -y ppa:jonathonf/ffmpeg-3
if [ "$(lsb_release -cs)" = "trusty" ]; then
  sudo add-apt-repository -y ppa:jonathonf/tesseract
fi
sudo apt-get update -qq
sudo apt-get install -qq -y ffmpeg

set +x
