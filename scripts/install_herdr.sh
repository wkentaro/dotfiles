#!/bin/bash

set -e

VERSION=v0.8.0-fork.2
EXPECTED_VERSION="herdr 0.8.0-fork.2"

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
  linux-aarch64) checksum=13d75e7d09bd6d70a23077160dd07710f24ff32c4085b661643e2e1b7bc31f10 ;;
  linux-x86_64)  checksum=5574b83e0f64552710f57b28e4afb4091a0f0e9f1aa60d80bd875fa32ecad25a ;;
  macos-aarch64) checksum=4c770566e3d0e2c4fcbc07212fa5f50a51a1062c28969b1e8bd57f83a92e173e ;;
  macos-x86_64)  checksum=5a6ccc4a2fcd9cd7266db06969a10d4a5f191fdc5bf10651d94070ba446c5f1b ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
