#!/bin/bash

set -e

VERSION=v0.8.2-fork.2
EXPECTED_VERSION="herdr 0.8.2-fork.2"

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
  linux-aarch64) checksum=8fbaff45cf6a8dd18d6120023078fbc37395efa8be6c592857bec532f579f3f7 ;;
  linux-x86_64)  checksum=7c81ea5d5bb5f8c8af059ed57b5fca75cbcaf61f7ef166c6a70a0f578ff02210 ;;
  macos-aarch64) checksum=331ef4961c94a205a8192419909c114d6e8343581aae09a9fd5bbaa863e558eb ;;
  macos-x86_64)  checksum=e385cb9c63e12360d74e1d7a448e39ba8afc6d54f3a62f87137afe8e0fb1d609 ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
