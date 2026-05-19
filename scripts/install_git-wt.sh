#!/bin/bash

if hash git-wt &>/dev/null; then
  exit 0
fi

set -e

VERSION=v0.28.0

case "$(uname)" in
  Linux)  os=linux ;;
  Darwin) os=darwin ;;
  *) echo "Unsupported OS: $(uname)" >&2; exit 1 ;;
esac

case "$(uname -m)" in
  x86_64|amd64) arch=amd64 ;;
  aarch64|arm64) arch=arm64 ;;
  *) echo "Unsupported arch: $(uname -m)" >&2; exit 1 ;;
esac

if [ "$os" = "darwin" ]; then
  asset="git-wt_${VERSION}_${os}_${arch}.zip"
else
  asset="git-wt_${VERSION}_${os}_${arch}.tar.gz"
fi

tmp_path=$(mktemp -d)
cd "$tmp_path"

curl -fL "https://github.com/k1LoW/git-wt/releases/download/${VERSION}/${asset}" -o "$asset"

if [ "$os" = "darwin" ]; then
  unzip -q "$asset"
else
  tar zxf "$asset"
fi

mkdir -p "$HOME/.local/bin"
mv git-wt "$HOME/.local/bin/"
chmod u+x "$HOME/.local/bin/git-wt"
