#!/bin/bash

if hash fpp &>/dev/null; then
  exit 0
fi

set -e

VERSION=0.9.5

if ! hash python3 &>/dev/null; then
  echo "fpp requires python3" >&2
  exit 1
fi

src_path="$HOME/.local/share/fpp"

if [ -d "$src_path/.git" ]; then
  git -C "$src_path" fetch --depth 1 origin "refs/tags/$VERSION:refs/tags/$VERSION"
  git -C "$src_path" checkout -q "$VERSION"
else
  rm -rf "$src_path"
  git clone --depth 1 --branch "$VERSION" \
    https://github.com/facebook/PathPicker.git "$src_path"
fi

mkdir -p "$HOME/.local/bin"
ln -fs "$src_path/fpp" "$HOME/.local/bin/fpp"
