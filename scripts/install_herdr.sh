#!/bin/bash

set -e

VERSION=v0.8.0-fork.1
EXPECTED_VERSION="herdr 0.8.0-fork.1"

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
  linux-aarch64) checksum=d0021b4b3112eb2154212eabd9eb24dec405f8eb472c419b799a96548c4d52f2 ;;
  linux-x86_64)  checksum=b7640af4933faaa5428e0879cea93ca8f13b4828e24141f6166f64babc6bc91a ;;
  macos-aarch64) checksum=d3611a3bc480851446fcfebb7797a1c19cf41aedfa360424ff1e28cfbd31f3dd ;;
  macos-x86_64)  checksum=610cb70e96cf69de8838f06226be62fbe658a82e74db35c05df0553e5979127a ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
