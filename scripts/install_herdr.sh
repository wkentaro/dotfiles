#!/bin/bash

set -e

VERSION=v0.9.0-fork.1
EXPECTED_VERSION="herdr 0.9.0-fork.1"

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
  linux-aarch64) checksum=0a6284b2c3836027728062df9b3cd3b48ffac0d418453428fe51857671566ee0 ;;
  linux-x86_64)  checksum=56187457a723e1d8665f64755b433eeda019459801d8fc317f22683973ae5b99 ;;
  macos-aarch64) checksum=0b8066e767acfe85bee5ef3cfd9109d7b523d3e8cf374fb8fb0c6e02400e8981 ;;
  macos-x86_64)  checksum=74986e3c5fa5e82abd8350f3b5c6fb22f78dc81351c3cfc46c77c6d8529f1b80 ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
