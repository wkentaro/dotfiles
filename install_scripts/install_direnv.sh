#!/bin/bash

if [ ! -e $HOME/.local/bin/direnv ]; then
  export PATH=$HOME/.local/bin:$PATH
  curl -sfL https://direnv.net/install.sh | bash
fi
