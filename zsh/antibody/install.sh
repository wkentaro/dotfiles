#!/bin/sh

if [ -f ~/.local/bin/antibody ]; then
  exit 0
fi

BASE_URL="https://github.com/caarlos0/antibody/releases/download"
VERSION="v2.3.6"
ARCH="386"
OS="$(uname -s | tr "[:upper:]" "[:lower:]")"
ARCH="$(uname -m)"
mkdir -p ~/.zsh/antibody/antibody
wget -q -O /tmp/antibody.tar.gz \
  "${BASE_URL}/${VERSION}/antibody_${OS}_${ARCH}.tar.gz"
tar xvzf /tmp/antibody.tar.gz -C ~/.zsh/antibody/antibody

ln -s ~/.zsh/antibody/antibody/antibody ~/.local/bin/antibody
