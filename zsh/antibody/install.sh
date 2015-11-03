#!/bin/sh

if [ -f ~/.zsh/antibody/antibody/bin/antibody ]; then
  exit 0
fi

BASE_URL="https://github.com/caarlos0/antibody/releases/download"
VERSION="v0.3.5"
ARCH="386"
OS="$(uname -s | tr "[:upper:]" "[:lower:]")"
if [ "$(uname -m)" = "x86_64" ]; then
  ARCH="amd64"
fi
mkdir -p ~/.zsh/antibody/antibody
wget -O /tmp/antibody.tar.gz \
  "${BASE_URL}/${VERSION}/antibody_${OS}_${ARCH}.tar.gz"
tar xvzf /tmp/antibody.tar.gz -C ~/.zsh/antibody/antibody
