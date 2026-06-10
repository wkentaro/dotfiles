#!/bin/bash

if hash ghq &>/dev/null; then
  exit 0
fi

set -e

VERSION=v1.10.1

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

target="ghq_${os}_${arch}"
asset="${target}.zip"

tmp_path=$(mktemp -d)
cd "$tmp_path"

curl -fL "https://github.com/x-motemen/ghq/releases/download/${VERSION}/${asset}" -o "$asset"
unzip -q "$asset"

mkdir -p "$HOME/.local/bin"
mv "${target}/ghq" "$HOME/.local/bin/"
chmod u+x "$HOME/.local/bin/ghq"
