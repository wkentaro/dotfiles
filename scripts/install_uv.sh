#!/bin/bash

if hash uv &>/dev/null; then
  exit 0
fi

set -euo pipefail

curl -LsSf https://astral.sh/uv/install.sh | sh
