#!/bin/bash

set -e

VERSION=v0.8.2-fork.4
EXPECTED_VERSION="herdr 0.8.2-fork.4"

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
  linux-aarch64) checksum=d29487f03141e80a54ac944e483a018c4d58bc3908b75183cf8981c1d475fb1a ;;
  linux-x86_64)  checksum=73f65f8d0cf5a98e450bfef69205c25f3405814b62cbf548b7e0e59029193473 ;;
  macos-aarch64) checksum=da8eca4fe313d8cfc5f43b2ba4485e19f7892913b08a7e8c616918f9a693c872 ;;
  macos-x86_64)  checksum=1d6eb74d7c33e112f66906a04eba12c4598aef0b1e83ae0647f6093dbcb36aab ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT

curl -fL "https://github.com/wkentaro/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

mkdir -p "$HOME/.local/bin"
install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
