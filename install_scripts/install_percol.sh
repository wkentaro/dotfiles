#!/bin/bash

if [ -e $HOME/.local/bin/percol ]; then
  exit 0
fi

if which pip &>/dev/null; then
  pip install -q --user percol
fi
