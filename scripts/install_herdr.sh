#!/bin/bash

set -e

VERSION=v0.8.2-fork.3
EXPECTED_VERSION="herdr 0.8.2-fork.3"

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
  linux-aarch64) checksum=853700563c7c51858cad87ec5984424d384ac7926483fe09e67b5c46f344e157 ;;
  linux-x86_64)  checksum=82ddc3c2e8defaa5a09b110f3509594ff6eb7c51df2cefcceb2680ce29b1f037 ;;
  macos-aarch64) checksum=91d5d19c45f8c82cb84f1a575dbccd5a6c88fdb1971731725515034de856678a ;;
  macos-x86_64)  checksum=072b915d0e0d5333a403b863025c1cd2ea4f707ef67cbc04b472f7b46b1f843b ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
