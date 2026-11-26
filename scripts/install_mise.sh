#!/bin/bash

if hash mise &>/dev/null; then
  exit 0
fi

set -e

VERSION=v2026.11.14

case "$(uname)" in
  Linux)  os=linux ;;
  Darwin) os=macos ;;
  *) echo "Unsupported OS: $(uname)" >&2; exit 1 ;;
esac

case "$(uname -m)" in
  x86_64|amd64) arch=x64 ;;
  aarch64|arm64) arch=arm64 ;;
  *) echo "Unsupported arch: $(uname -m)" >&2; exit 1 ;;
esac

asset="mise-${VERSION}-${os}-${arch}"

tmp_path=$(mktemp -d)
cd "$tmp_path"

curl -fL "https://github.com/jdx/mise/releases/download/${VERSION}/${asset}" -o mise

mkdir -p "$HOME/.local/bin"
mv mise "$HOME/.local/bin/"
chmod u+x "$HOME/.local/bin/mise"
