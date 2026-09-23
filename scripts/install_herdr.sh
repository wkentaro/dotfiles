#!/bin/bash

set -e

VERSION=v0.9.0-fork.3
EXPECTED_VERSION="herdr 0.9.0-fork.3"

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
  linux-aarch64) checksum=0473de76d2d8518fbf5798724755bc3093a58a7e7761447b794f5f431541eb89 ;;
  linux-x86_64)  checksum=84432872ea60ba8e1c14ddc4af027ce08417544811bc29d32ef6f8fc58bd578d ;;
  macos-aarch64) checksum=30ce5c998b4bbc293b4f1d7c86561be00d524fc78ad456d9ca6e78c0c43d643e ;;
  macos-x86_64)  checksum=d149ba958c481034913e658712c6a49265d377280c7e691ad4ca4a7fae01247d ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
