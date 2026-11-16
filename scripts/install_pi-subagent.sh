#!/bin/bash

dst_path="$HOME/.pi/agent/extensions/subagent"

if [ -e "$dst_path" ]; then
  exit 0
fi

set -e

VERSION=v1.5.2

url="https://raw.githubusercontent.com/earendil-works/pi/${VERSION}/packages/coding-agent/examples/extensions/subagent"

tmp_path=$(mktemp -d)
curl -fsSL "${url}/index.ts" -o "${tmp_path}/index.ts"
curl -fsSL "${url}/agents.ts" -o "${tmp_path}/agents.ts"

mkdir -p "$(dirname "$dst_path")"
chmod 755 "$tmp_path"
mv "$tmp_path" "$dst_path"
