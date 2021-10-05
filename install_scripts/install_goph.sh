#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0;
fi

set -e

sudo apt-get install dconf-cli uuid-runtime
bash -c  "$(wget -qO- https://git.io/vQgMr)"
