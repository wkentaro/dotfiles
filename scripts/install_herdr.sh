#!/bin/bash

set -e

VERSION=v0.8.201
EXPECTED_VERSION="herdr 0.8.201"

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
  linux-aarch64) checksum=962658239978cb20c79c2d38022877430899ca8c84da39797f059e090e4766f5 ;;
  linux-x86_64)  checksum=1c3448968eea20cf605762b7a5011a4ff5061b8f2441cc29077ba75da24a026e ;;
  macos-aarch64) checksum=7a2cf0bb69891c0308e9bfb0891c7b7a9256f09b0fdf88f164ae49e7aa83e94c ;;
  macos-x86_64)  checksum=7f4c6b2336a32980c88ede685ea0ded663c56735cd3bd1662e7312ae674ed344 ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
