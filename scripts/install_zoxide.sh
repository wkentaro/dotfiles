#!/bin/bash

if hash zoxide &>/dev/null; then
  exit 0
fi

set -e

VERSION=0.9.9

case "$(uname)" in
  Linux)  os=unknown-linux-musl ;;
  Darwin) os=apple-darwin ;;
  *) echo "Unsupported OS: $(uname)" >&2; exit 1 ;;
esac

case "$(uname -m)" in
  x86_64|amd64) arch=x86_64 ;;
  aarch64|arm64) arch=aarch64 ;;
  *) echo "Unsupported arch: $(uname -m)" >&2; exit 1 ;;
esac

target="${arch}-${os}"
asset="zoxide-${VERSION}-${target}.tar.gz"

tmp_path=$(mktemp -d)
cd "$tmp_path"

curl -fL "https://github.com/ajeetdsouza/zoxide/releases/download/v${VERSION}/${asset}" -o "$asset"
tar zxf "$asset"

mkdir -p "$HOME/.local/bin"
mv zoxide "$HOME/.local/bin/"
chmod u+x "$HOME/.local/bin/zoxide"
