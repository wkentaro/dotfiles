#!/bin/bash

if hash delta &>/dev/null; then
  exit 0
fi

set -e

VERSION=0.19.2

case "$(uname)" in
  Linux)  os=unknown-linux-gnu ;;
  Darwin) os=apple-darwin ;;
  *) echo "Unsupported OS: $(uname)" >&2; exit 1 ;;
esac

case "$(uname -m)" in
  x86_64|amd64) arch=x86_64 ;;
  aarch64|arm64) arch=aarch64 ;;
  *) echo "Unsupported arch: $(uname -m)" >&2; exit 1 ;;
esac

target="${arch}-${os}"
asset="delta-${VERSION}-${target}.tar.gz"

tmp_path=$(mktemp -d)
cd "$tmp_path"

curl -fL "https://github.com/dandavison/delta/releases/download/${VERSION}/${asset}" -o "$asset"
tar zxf "$asset"

mkdir -p "$HOME/.local/bin"
mv "delta-${VERSION}-${target}/delta" "$HOME/.local/bin/"
chmod u+x "$HOME/.local/bin/delta"
