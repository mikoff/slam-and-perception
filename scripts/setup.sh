#!/bin/bash
# Post-create setup: indexes the repo and seeds ADRs via codebase-memory-mcp.
# The binary itself is installed in the Dockerfile (build stage).
# Run once after container creation: bash scripts/setup.sh
set -euo pipefail

if ! command -v codebase-memory-mcp &>/dev/null; then
  echo "ERROR: codebase-memory-mcp not found on PATH." >&2
  echo "It should be pre-installed in the Docker image. Check the Dockerfile." >&2
  exit 1
fi

exec "${PYTHON:-python3}" "$(dirname "$0")/setup.py"
