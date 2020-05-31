#!/bin/bash

if [ "$(uname)" != 'Linux' ]; then
  exit 0
fi

if which pip &>/dev/null; then
  pip install --user cudnnenv
fi
