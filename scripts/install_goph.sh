#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0;
fi

set -e

sudo apt-get install dconf-cli uuid-runtime
bash -c  "$(curl -fsSL https://git.io/vQgMr)"
