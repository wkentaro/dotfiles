#!/bin/sh

if [ -f ~/.local/bin/antibody ]; then
  echo "[$(basename $0)] antibody is already installed."
  exit 0
fi

set -x

BASE_URL="https://github.com/caarlos0/antibody/releases/download"
VERSION="v3.1.4"
OS="$(uname -s | tr "[:upper:]" "[:lower:]")"
ARCH="$(uname -m)"
mkdir -p ~/.zsh/antibody/antibody
wget -q -O /tmp/antibody.tar.gz \
  "${BASE_URL}/${VERSION}/antibody_${OS}_${ARCH}.tar.gz"
tar xvzf /tmp/antibody.tar.gz -C ~/.zsh/antibody/antibody

mkdir -p ~/.local/bin
ln -s ~/.zsh/antibody/antibody/antibody ~/.local/bin/antibody

set +x
