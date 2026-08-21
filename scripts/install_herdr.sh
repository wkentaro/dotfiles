#!/bin/bash

set -e

VERSION=v0.8.0-fork.4
EXPECTED_VERSION="herdr 0.8.0-fork.4"

if command -v herdr >/dev/null 2>&1 && [ "$(herdr --version 2>/dev/null)" = "$EXPECTED_VERSION" ]; then
  exit 0
fi

case "$(uname)" in
  Linux)  os=linux ;;
  Darwin) os=macos ;;
  *) echo "Unsupported OS: $(uname)" >&2; exit 1 ;;
esac

case "$(uname -m)" in
  x86_64|amd64) arch=x86_64 ;;
  aarch64|arm64) arch=aarch64 ;;
  *) echo "Unsupported arch: $(uname -m)" >&2; exit 1 ;;
esac

asset="herdr-${os}-${arch}"

case "${os}-${arch}" in
  linux-aarch64) checksum=031f8eff4a0a3b2d67a37eb2fd6547094c945bc88df8464c83ae8f9a0e60d0b5 ;;
  linux-x86_64)  checksum=7ba8d175bdf3cae9aafb6f81edf2942404365088da6bb9bcb301d2023cf4509f ;;
  macos-aarch64) checksum=fb08baa1a0cbfc337869a24e0325df486a8a0ed03607c06dd805b0caf75577d4 ;;
  macos-x86_64)  checksum=00ff36c565c5f8488d6d4a0ebb605d1bf9ce2ef49a112c2c5606165637486f5b ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
