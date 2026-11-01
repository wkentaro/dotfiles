#!/bin/bash

set -e

VERSION=v0.9.5
EXPECTED_VERSION="herdr 0.9.5"
HERDR_BAR_REF=v0.14.0

install_herdr() {
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
    linux-aarch64) checksum=17d7536f2da50859523680ef279dffcac8e4a235f4e46fe8da6c2ec44141897c ;;
    linux-x86_64)  checksum=14d9e8bc8b4a9e0118d77c32266f76d461f09b40342eb23752960bd89f59929d ;;
    macos-aarch64) checksum=fb77c2232a05a8035421382c894392f4174e4e38a18e48af3dd0290157c3a7e2 ;;
    macos-x86_64)  checksum=5fb0a908ec7f6745e3e245738b06bfad56c46391c391ab3c0955fbead01f1369 ;;
  esac

  tmp_path=$(mktemp -d)
  trap 'rm -rf "$tmp_path"' EXIT

  curl -fL "https://github.com/herdrdev/herdr/releases/download/${VERSION}/${asset}" -o "${tmp_path}/herdr"
  printf '%s  %s\n' "$checksum" "${tmp_path}/herdr" | shasum -a 256 -c -

  mkdir -p "$HOME/.local/bin"
  install -m 0755 "${tmp_path}/herdr" "$HOME/.local/bin/herdr"
}

if [ "$("$HOME/.local/bin/herdr" --version 2>/dev/null)" != "$EXPECTED_VERSION" ]; then
  install_herdr
fi

if ! "$HOME/.local/bin/herdr" plugin list 2>/dev/null | grep -q "^- herdr-bar "; then
  "$HOME/.local/bin/herdr" plugin install jeffarese/herdr-bar --ref "$HERDR_BAR_REF" --yes
fi
