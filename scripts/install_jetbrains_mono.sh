#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 0
fi

if ! which fc-cache &>/dev/null; then
  echo "fc-cache is not installed"
  exit 0
fi

fonts_dir="$HOME/.local/share/fonts/JetBrainsMono"
if [ -f "$fonts_dir/JetBrainsMono-Regular.ttf" ]; then
  exit 0
fi

set -e

VERSION=2.304

tmp_path=$(mktemp -d)
cd "$tmp_path"

curl -fL "https://github.com/JetBrains/JetBrainsMono/releases/download/v${VERSION}/JetBrainsMono-${VERSION}.zip" -o JetBrainsMono.zip
unzip -qq JetBrainsMono.zip

mkdir -p "$fonts_dir"
cp fonts/ttf/*.ttf "$fonts_dir/"
fc-cache -f "$fonts_dir"

rm -rf "$tmp_path"
