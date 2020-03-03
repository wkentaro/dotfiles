#!/bin/bash

if [ -e $HOME/.local/bin/imgcat ]; then
  exit 0
fi

pip3 install -q --user imgcat
