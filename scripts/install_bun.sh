#!/bin/bash

if hash bun &>/dev/null; then
  exit 0
fi

set -e

VERSION=bun-v1.4.5

case "$(uname)" in
  Linux)  os=linux ;;
  Darwin) os=darwin ;;
  *) echo "Unsupported OS: $(uname)" >&2; exit 1 ;;
esac

case "$(uname -m)" in
  x86_64|amd64) arch=x64 ;;
  aarch64|arm64) arch=aarch64 ;;
  *) echo "Unsupported arch: $(uname -m)" >&2; exit 1 ;;
esac

target="bun-${os}-${arch}"
asset="${target}.zip"

case "$target" in
  bun-darwin-aarch64) sha256=b5d6bc2e13caf14e57c89661e89efd073ff16165b41bfe43882616410609cd10 ;;
  bun-darwin-x64)     sha256=a4972e778cce4b6a194d7d289f326aa61576d71d80b7a53e9da2f3739566e5fd ;;
  bun-linux-aarch64)  sha256=18656a3bf0ea5641f5796ae4213b75255579a70a0fded2f337933785568cdf84 ;;
  bun-linux-x64)      sha256=94559b3b528995a49452f9f2404cec74f2efd7d06e0a000a8cd214b02c3666eb ;;
esac

tmp_path=$(mktemp -d)
trap 'rm -rf "$tmp_path"' EXIT
cd "$tmp_path"

curl -fL "https://github.com/oven-sh/bun/releases/download/${VERSION}/${asset}" -o "$asset"
if hash sha256sum &>/dev/null; then
  echo "${sha256}  ${asset}" | sha256sum -c -
else
  echo "${sha256}  ${asset}" | shasum -a 256 -c -
fi
unzip -q "$asset"

mkdir -p "$HOME/.local/bin"
mv "${target}/bun" "$HOME/.local/bin/"
chmod u+x "$HOME/.local/bin/bun"
