#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

sudo add-apt-repository ppa:octave/stable
sudo apt-get update
sudo apt-get install octave
