#!/bin/bash

if hash bun &>/dev/null; then
  exit 0
fi

set -e

VERSION=bun-v1.4.5

case "$(uname)" in
  Linux)  os=linux ;;
  Darwin) os=darwin ;;
  *) echo "Unsupported OS: $(uname)" >&2; exit 1 ;;
esac

case "$(uname -m)" in
  x86_64|amd64) arch=x64 ;;
  aarch64|arm64) arch=aarch64 ;;
  *) echo "Unsupported arch: $(uname -m)" >&2; exit 1 ;;
esac

target="bun-${os}-${arch}"
asset="${target}.zip"

tmp_path=$(mktemp -d)
cd "$tmp_path"

curl -fL "https://github.com/oven-sh/bun/releases/download/${VERSION}/${asset}" -o "$asset"
unzip -q "$asset"

mkdir -p "$HOME/.local/bin"
mv "${target}/bun" "$HOME/.local/bin/"
chmod u+x "$HOME/.local/bin/bun"
