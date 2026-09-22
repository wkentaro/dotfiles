#!/bin/bash

set -e

VERSION=v0.9.0-fork.2
EXPECTED_VERSION="herdr 0.9.0-fork.2"

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
  linux-aarch64) checksum=3e5f131b0b0971903f0a7abc76c7001c9d590072b383bd79d5637632d1ce963b ;;
  linux-x86_64)  checksum=14366f858ee2064a67e53fa520b7a20e5fce355b2b29229dd0c52f62fa28d2b5 ;;
  macos-aarch64) checksum=04e12f769f5f5f92a471a8df6da95200d52ed2cb754c3f080a501aa317b95308 ;;
  macos-x86_64)  checksum=7c75b1d50dfe515016d555b8bb12948eb89645ae9c8657653eef20d239aa57e7 ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
