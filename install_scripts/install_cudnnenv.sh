#!/bin/bash

if [ "$(uname)" != 'Linux' ]; then
  exit 0
fi

sudo -H python3 -m pip install cudnnenv
cudnnenv install -h
