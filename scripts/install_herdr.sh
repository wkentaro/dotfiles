#!/bin/bash

set -e

VERSION=v0.8.2-fork.1
EXPECTED_VERSION="herdr 0.8.2-fork.1"

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
  linux-aarch64) checksum=8380fb995344a4f5f588e621518acc534aa0bb07799ec787254293af00b825e9 ;;
  linux-x86_64)  checksum=c68df8fa4803610a16382fce08c62fd836f33bea05c675a6d6d4ce50374f670b ;;
  macos-aarch64) checksum=372393d09bd08af07ef5f88f454ab83deb78dc05c28129cda35ca06effb5ffd7 ;;
  macos-x86_64)  checksum=6f2e5510b16322bb072fe3e82c2dfb256e99353397b135435613d5342bb737e4 ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
