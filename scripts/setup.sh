#!/bin/bash
# One-time setup: installs codebase-memory-mcp binary, then delegates to setup.py.
# Run once after cloning: bash scripts/setup.sh
set -euo pipefail

if ! command -v codebase-memory-mcp &>/dev/null; then
  echo "Installing codebase-memory-mcp..."
  curl -fsSL https://raw.githubusercontent.com/DeusData/codebase-memory-mcp/main/install.sh \
    | bash -s -- --ui --skip-config
  export PATH="$HOME/.local/bin:$PATH"
fi

exec python3 "$(dirname "$0")/setup.py"
