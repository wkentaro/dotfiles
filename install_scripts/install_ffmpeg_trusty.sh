#!/bin/sh

if which ffmpeg &>/dev/null; then
  exit 1
fi

sudo add-apt-repository ppa:mc3man/trusty-media
sudo apt-get update
sudo apt-get install ffmpeg