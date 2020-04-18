#!/bin/bash

if [ ! -e $HOME/.local/bin/direnv ]; then
  curl -sfL https://direnv.net/install.sh | bash
fi
