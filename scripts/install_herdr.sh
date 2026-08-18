#!/bin/bash

set -e

VERSION=v0.8.0-fork.3
EXPECTED_VERSION="herdr 0.8.0-fork.3"

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
  linux-aarch64) checksum=25b8aac3e0db1785f340e57a083bf32ca2733caee26db707557dbd9091933a39 ;;
  linux-x86_64)  checksum=485a5d2674d29b34560ab0b1c0bc7e33050b1e8c363252c2e96648d747c6cfa3 ;;
  macos-aarch64) checksum=fb6f6c19ae23ed3df7d0c6d218b380d86e322933c43e6eebb261c08eb79ebe18 ;;
  macos-x86_64)  checksum=c0e626cf5ecb218ad53566cfd40aba5ec14124072e620489ec555a9f134d3d30 ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
