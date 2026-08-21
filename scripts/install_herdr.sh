#!/bin/bash

set -e

VERSION=v0.8.0-fork.5
EXPECTED_VERSION="herdr 0.8.0-fork.5"

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
  linux-aarch64) checksum=2f1124b602c3c9801b5ab59201594058c076e3e295d1dab816f6072d9f033990 ;;
  linux-x86_64)  checksum=ba1a99a94b255ccec4c8ba0fbe9e4e6347b4755ef76bae7efd41adecc4e8215a ;;
  macos-aarch64) checksum=de534ddcc4a4a8c277841a49720e75ac75f2d978bb7afc2512e861290475af34 ;;
  macos-x86_64)  checksum=ab5d39d7eed080721010a92134fba661bdbca790d57e7a7600f816b11ebd4633 ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
