#!/usr/bin/env bash

if hash git-mv-with-history &>/dev/null; then
  exit 0
fi

mkdir -p $HOME/.local/bin
wget -q https://gist.githubusercontent.com/emiller/6769886/raw/ae47266e867438b9cbd188fb6851ca6566e241d0/git-mv-with-history -O $HOME/.local/bin/git-mv-with-history --no-check-certificate
chmod u+x $HOME/.local/bin/git-mv-with-history
