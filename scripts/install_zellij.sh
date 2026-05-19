#!/bin/bash

if hash zellij &>/dev/null; then
  exit 0
fi

set -e

tmp_path=$(mktemp -d)
cd $tmp_path

curl -L https://github.com/zellij-org/zellij/releases/download/v0.44.1/zellij-x86_64-unknown-linux-musl.tar.gz -O
tar zxvf zellij-x86_64-unknown-linux-musl.tar.gz

mkdir -p ~/.local/bin
mv zellij ~/.local/bin/
