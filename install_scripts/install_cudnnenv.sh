#!/bin/bash

if [ "$(uname)" != 'Linux' ]; then
  exit 0
fi

pip install --user cudnnenv
