#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

sudo add-apt-repository ppa:octave/stable
sudo apt-get update
sudo apt-get install octave

# sudo apt install liboctave-dev
# octave> pkg install --forge image
# octave> pkg load image
# octave> bwareaopen
