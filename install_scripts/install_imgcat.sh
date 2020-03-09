#!/bin/bash

if [ -e $HOME/.local/bin/imgcat ]; then
  exit 0
fi

if which pip3 &>/dev/null; then
  pip3 install -q --user imgcat
fi
