#!/bin/bash

if which rg &>/dev/null; then
  echo "ripgrep is already installed"
  exit 1
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

curl -LO https://github.com/BurntSushi/ripgrep/releases/download/11.0.2/ripgrep_11.0.2_amd64.deb
sudo dpkg -i ripgrep_11.0.2_amd64.deb

rm -rf $TMPDIR
