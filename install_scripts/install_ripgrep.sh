#!/bin/bash

if which rg &>/dev/null; then
  echo "ripgrep is already installed"
  exit 1
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

VERSION=13.0.0

curl -LO https://github.com/BurntSushi/ripgrep/releases/download/${VERSION}/ripgrep_${VERSION}_amd64.deb
sudo dpkg -i ripgrep_${VERSION}_amd64.deb

rm -rf $TMPDIR
