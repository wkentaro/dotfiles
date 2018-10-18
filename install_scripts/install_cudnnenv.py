#!/bin/bash

if [ "$(uname)" != 'Linux' ]; then
  exit 0
fi

sudo -H pip install cudnnenv
cudnnenv install -h
