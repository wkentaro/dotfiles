#!/bin/bash

set -e

VERSION=v0.8.0-fork.6
EXPECTED_VERSION="herdr 0.8.0-fork.6"

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
  linux-aarch64) checksum=9a1d01f9b4966a47a62473565b50eeac61f0fdaa45c1c1977d001854f10f14d5 ;;
  linux-x86_64)  checksum=aa410db46a6ecd31ff177e64d9a7034766ad03b0f5635dea936000bf5f4c64d7 ;;
  macos-aarch64) checksum=1b0ea5cf0fd6e095d01322598f68f91f5a9d63d8e5260ff69a93eb466701c2c6 ;;
  macos-x86_64)  checksum=d23af482a49e008d4a3ed782fec04d686a283bb41bb39b4e67ac0eb091c437fb ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
